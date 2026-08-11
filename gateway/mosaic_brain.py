#!/usr/bin/env python3
"""
mosaic_brain.py — ESP32-Mosaic world-model brain (v0.2: device_class + wifi).

Turns the raw sighting stream into ENTITIES:
  - Per-device RSSI stats (min/max/avg/spread) → stationary vs moving
  - Device labels/notes (from devices table, seeded from device labels JSON)
  - Entity slots: rotating BLE MACs bind to stable anchors via co-occurrence
  - Class coherence: same device_class at same RSSI level = same entity
    (FindMy/Meta accessories keep their class across MAC rotation)
  - FindMy/AirTag entities labeled distinctly ("findmy-*" / "airtag-*")
  - WiFi places registry: type:"wifi" beacons → BSSID → PLACE entities
  - Probe-request log: which client MACs seek which SSIDs (identity from the air)
  - MOVING = MUTED: windows where the device moved are not location evidence

Usage:
  python3 mosaic_brain.py --status          # entity view of recent data
  python3 mosaic_brain.py --devices         # device table
  python3 mosaic_brain.py --seed-labels     # import device labels JSON
  python3 mosaic_brain.py --chains          # class-weighted entity chains
  python3 mosaic_brain.py --places          # BSSID registry → PLACE entities
  python3 mosaic_brain.py --probes          # probe-request identity log
  python3 mosaic_brain.py --backfill-beacons  # rebuild beacon_samples from events
  python3 mosaic_brain.py --where "home"    # resolve a place label via BSSIDs
"""

import argparse
import json
import math
import os
import sqlite3
import sys
from datetime import datetime, timedelta, timezone

DB = os.path.expanduser("~/.orb/orb.db")
LABELS_FILE = os.path.expanduser("~/.mosaic/device_labels.json")
LOCATIONS_FILE = os.path.expanduser("~/.orb/locations.json")

# Time-window filter normalization.
# received_at is stored ISO-8601 ('2026-08-10T22:37:33+00:00') while SQLite's
# datetime('now') yields '2026-08-10 22:37:33'. Naive string comparison of the
# two formats mis-orders them ('T' (0x54) > ' ' (0x20)), so every --hours N
# window silently returned ALL rows from the same UTC day (including old test
# payloads). Normalize the stored column to SQLite's format before comparing.
# Use via: WHERE {TS_WINDOW}   (add the alias prefix for correlated subqueries)
TS_WINDOW = "REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)"

# Tuning (static for now — tuned the hard way, see gateway config)
# NOTE: spread is now ROBUST p10-p90 (not raw max-min). A single deep-fade
# sample (-106 in a -58..-66 cluster) previously flipped a stationary AirTag
# to MOVING. Thresholds anchored on live data:
#   STATIC  <= 15  (Aqara 3, ThermoBeacon 14, findmy cluster 5-8)
#   MOVING  >= 25  (anchor: cd:c4:ba genuine excursion reads 23.6 → 25 catches it)
#   MUTED   >= 20  (any real mover must mute its location evidence)
STATIONARY_MAX_SPREAD = 15   # dB: below this = stationary (robust p10-p90)
MOVING_MIN_SPREAD = 25       # dB: above this = moving (robust p10-p90)
MUTED_SPREAD = 20            # dB: above this = location evidence muted
SPREAD_MIN_SAMPLES = 10      # below this, p10-p90 is unreliable → fall back to raw
CO_OCCUR_BIND_SECONDS = 120  # new MAC seen within this of an anchor = slot candidate
CO_OCCUR_LEVEL_GATE = 8      # dB: same-entity rotation keeps signal level (data: -62 cluster ±3)
# Class coherence (BLE): the SAME device_class at the SAME RSSI level is much
# stronger evidence of one rotating entity; DIFFERENT classes at one level are
# different devices (a phone and an AirTag at -62 dB are NOT the same entity).
# Recognized classes carry identity; "unknown" carries none.
RECOGNIZED_CLASSES = {"findmy", "meta", "flipper"}

# New knobs (overridable via gateway/config.yaml → world_model)
DEFAULT_WM = {
    "class_match_multiplier": 1.5,      # same recognized class at same level
    "class_mismatch_penalty": 0.3,      # different classes at same level
    "place_min_seen": 3,                # beacon sightings before a BSSID is a place
    "place_max_rssi_variance": 8,       # dB: robust p10-p90 spread gate for PLACE
    # Data (Aug 11, 4h of real beacons): the home AP — the strongest, most
    # important BSSID — swings 8dB all-time / 7dB p10-p90 / std 2.5 from rare
    # dips. 6dB would permanently reject it as a PLACE once the 24h span gate
    # opens (all-time min/max only grows). 8dB admits it while still excluding
    # genuinely noisy/moving BSSIDs.
    "place_min_span_seconds": 86400,    # min first→last span for PLACE (24h)
    "place_variance_window_seconds": 86400,  # recent window for stability (24h)
    "place_variance_min_samples": 5,    # min samples in window before trusting it
    # Stream bind (Aug 11): entity chains sever because real rotation cadence
    # (~14 min for the desk iPhone's dual BLE identities) >> tau=90s decay gate.
    # The greedy temporal walk reconnects fragments: ended MAC A → new MAC B at
    # the same level/company within stream_max_gap. Observed handoff gaps: 20-23s;
    # the 14-min period is the ceiling for a missed scan. Level gate: the two
    # -43dB streams sit at ±1.5dB of each other; 4dB keeps streams/entities apart
    # while tolerating RSSI drift.
    "stream_max_gap_seconds": 600,      # max B.first - A.last for a stream edge
    "stream_level_gate": 4,             # dB: |avgA - avgB| for a stream edge
}


def load_wm():
    """Read world_model from gateway/config.yaml (mirrors orb_gateway.load_config).
    Falls back to DEFAULT_WM when missing — the brain must always run."""
    wm = dict(DEFAULT_WM)
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "config.yaml")
    if os.path.exists(path):
        try:
            import yaml
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
            wm_cfg = (cfg.get("world_model") or {}) or {}
            wm.update({k: v for k, v in wm_cfg.items() if k in wm and v is not None})
        except Exception:
            pass
    return wm


WM = load_wm()


def conn():
    c = sqlite3.connect(DB)
    c.row_factory = sqlite3.Row
    return c


def ensure_schema(c):
    c.execute("""
    CREATE TABLE IF NOT EXISTS devices (
        mac         TEXT PRIMARY KEY,
        label       TEXT,
        note        TEXT,
        entity_id   TEXT,
        stable      INTEGER DEFAULT 0,
        first_seen  TEXT,
        last_seen   TEXT
    )
    """)
    # WiFi places registry (written by the gateway from type:"wifi" beacons)
    c.execute("""
    CREATE TABLE IF NOT EXISTS places (
        bssid       TEXT PRIMARY KEY,
        ssid        TEXT,
        channel     INTEGER,
        first_seen  TEXT,
        last_seen   TEXT,
        seen_count  INTEGER DEFAULT 1,
        min_rssi    INTEGER,
        max_rssi    INTEGER,
        avg_rssi    REAL,
        stable      INTEGER DEFAULT 0
    )
    """)
    # WiFi probe-request log (written by the gateway)
    c.execute("""
    CREATE TABLE IF NOT EXISTS probes (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        received_at TEXT,
        node_id     TEXT,
        client_mac  TEXT,
        ssid        TEXT,
        channel     INTEGER,
        rssi        INTEGER
    )
    """)
    # Per-beacon RSSI samples (written by the gateway from type:"wifi" beacons).
    # PLACE stability is computed from a RECENT window of these samples, not
    # the places table's monotonic all-time min/max (outlier-poisoned).
    c.execute("""
    CREATE TABLE IF NOT EXISTS beacon_samples (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        bssid       TEXT,
        received_at TEXT,
        rssi        INTEGER
    )
    """)
    c.execute("CREATE INDEX IF NOT EXISTS idx_beacon_samples_bssid ON beacon_samples(bssid, received_at)")


def load_labels():
    """Seed labels from device labels JSON (manual annotations)."""
    if not os.path.exists(LABELS_FILE):
        return {}
    with open(LABELS_FILE) as f:
        return json.load(f)


def seed_labels(c):
    labels = load_labels()
    for mac, info in labels.items():
        c.execute("""
        INSERT INTO devices (mac, label, note, stable, first_seen, last_seen)
        VALUES (?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
        ON CONFLICT(mac) DO UPDATE SET
            label = excluded.label,
            note = excluded.note,
            stable = excluded.stable
        """, (mac.lower(), info.get("label", ""), info.get("note", ""),
              1 if "WIFI" in info.get("label", "") or "STABLE" in info.get("label", "").upper() else 0))
    c.commit()
    print(f"Seeded {len(labels)} device labels")


def robust_spread(vals):
    """p10-p90 of a sorted RSSI sample list — outlier-proof spread.

    Raw max-min lets ONE deep-fade sample (-106 in a -58..-66 cluster)
    flip a stationary AirTag to MOVING. The p10-p90 range trims the tails:
    same medicine as the PLACE gate (places_view). Falls back to raw
    max-min when there are too few samples for percentiles to mean anything.
    """
    if len(vals) < SPREAD_MIN_SAMPLES:
        return max(vals) - min(vals)
    s = sorted(vals)
    n = len(s)
    lo = s[max(0, int(n * 0.10) - 1)]
    hi = s[min(n - 1, int(n * 0.90) - 1)]
    return hi - lo


def device_stats(c, hours=24):
    """Per-device RSSI stats over the window, plus the latest device_class.

    spread = ROBUST p10-p90 (max-min is outlier-poisoned: one deep-fade
    sample flips a stationary device to MOVING). raw spread kept for the
    display so both numbers are visible.
    """
    cur = c.execute("""
    SELECT mac,
           MIN(rssi) AS min_rssi,
           MAX(rssi) AS max_rssi,
           ROUND(AVG(rssi),1) AS avg_rssi,
           (MAX(rssi)-MIN(rssi)) AS spread_raw,
           COUNT(*) AS n,
           MAX(name) AS name,
           MAX(s.received_at) AS last_seen,
           (SELECT device_class FROM sightings s2
            WHERE s2.mac = s.mac AND s2.device_class IS NOT NULL
            ORDER BY s2.received_at DESC LIMIT 1) AS device_class
    FROM sightings s
    WHERE REPLACE(substr(s.received_at,1,19),'T',' ') > datetime('now', ?)
    GROUP BY mac
    HAVING n >= 3
    """, (f"-{hours} hours",))
    rows = [dict(r) for r in cur.fetchall()]

    # Robust spread pass: fetch each device's sample list in one go.
    # (Percentiles can't be computed in the aggregate query, so pull the
    # raw values per MAC and reduce in Python.)
    for r in rows:
        vals = [v[0] for v in c.execute(
            "SELECT rssi FROM sightings WHERE mac=? "
            "AND REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?) "
            "ORDER BY rssi", (r["mac"], f"-{hours} hours"))]
        r["spread"] = robust_spread(vals)

    rows.sort(key=lambda r: r["spread"], reverse=True)
    return rows


def entity_view(c, hours=24):
    """The entity picture: devices → labels → movement class."""
    ensure_schema(c)
    stats = device_stats(c, hours)
    labels = {r["mac"]: r for r in c.execute("SELECT * FROM devices")}

    rows = []
    for s in stats:
        mac = s["mac"]
        dclass = s["device_class"]
        lbl = labels.get(mac)
        label = lbl["label"] if lbl and lbl["label"] else (s["name"] or mac[:8])
        stable = bool(lbl["stable"]) if lbl else False

        # Movement class from RSSI spread (the data's own detector)
        if s["spread"] <= STATIONARY_MAX_SPREAD:
            move_class = "STATIC"
        elif s["spread"] >= MOVING_MIN_SPREAD:
            move_class = "MOVING"
        else:
            move_class = "AMBI"
        muted = s["spread"] >= MUTED_SPREAD  # moving = location evidence muted

        # FindMy/AirTag labels: distinct namespace, auto-labeled when the
        # firmware classifies a device as findmy and nobody named it yet.
        # A STATIC findmy device is a PLACE candidate (rotates slowly, stays put).
        if dclass == "findmy" and not (lbl and lbl["label"]):
            label = f"findmy-{mac[:8]}"
        if dclass == "findmy":
            label = label if label.startswith(("findmy-", "airtag-")) else f"findmy/{label}"

        # KNOWN = has an earned identity: explicit devices-table label, the
        # stable flag, or a device-reported name (TV/ThermoBeacon/...). The
        # findmy-* auto labels are placeholders, not names — rotating AirTag
        # MACs stay UNKNOWN until someone names the underlying object.
        named = bool(lbl and lbl["label"]) or bool(s["name"] and dclass != "findmy")
        known = stable or named

        # Operator tier (mission-driven order, see sort below):
        #   0 KNOWN  — the labeled/stable world: "is my entity here?" (M1)
        #   1 MOVING — the event: what is moving right now (philosophy #5)
        #   2 STATIC — the stationary world (places/anchors), strongest first
        #   3 AMBI   — the ambiguous noise floor, last
        if known:
            tier = 0
        elif move_class == "MOVING":
            tier = 1
        elif move_class == "STATIC":
            tier = 2
        else:
            tier = 3

        rows.append({
            "mac": mac,
            "label": label,
            "stable": stable,
            "min": s["min_rssi"], "max": s["max_rssi"], "avg": s["avg_rssi"],
            "spread": s["spread"], "spread_raw": s["spread_raw"], "n": s["n"],
            "class": move_class, "muted": muted,
            "dclass": dclass or "-",
            "place_candidate": dclass == "findmy" and move_class == "STATIC",
            "known": known, "tier": tier,
        })

    # The old sort (spread desc, from device_stats) let unknown rotating MACs
    # bury the labeled anchors — the opposite of what an operator checking
    # "is my world normal?" needs. Re-sort by the mission tiers:
    # within KNOWN/MOVING keep most-mobile first; within STATIC strongest
    # signal first (a strong static unknown = close = the M3 anomaly shape,
    # must not be buried under -100 dB neighbor noise).
    rows.sort(key=lambda r: (r["tier"], -r["avg"] if r["tier"] == 2 else -r["spread"]))
    return rows


def print_entity_view(rows, limit=40):
    if not rows:
        print("No data in window (need >=3 sightings per device).")
        return
    print(f"{'LABEL':<26} {'class':<7} {'dclass':<9} {'avg':>5} {'spr':>4} {'raw':>4} {'n':>5}  mac")
    print("-" * 96)
    for r in rows[:limit]:
        mark = "MUTED" if r["muted"] else ""
        stable_m = "*" if r["stable"] else " "
        pc = "PLACE?" if r["place_candidate"] else ""
        print(f"{stable_m}{r['label'][:25]:<25} {r['class']:<7} {r['dclass']:<9} {r['avg']:>5} {r['spread']:>4} {r['spread_raw']:>4} {r['n']:>5}  {r['mac']} {mark} {pc}")
    tiers = ("KNOWN", "MOVING", "STATIC", "AMBI")
    counts = {t: sum(1 for r in rows if r["tier"] == i) for i, t in enumerate(tiers)}
    summary = " · ".join(f"{t}={counts[t]}" for t in tiers)
    print(f"  {summary}  (of {len(rows)} devices; spr = robust p10-p90 spread, raw = max-min)")


def collapse_chains(rows, macs=None, min_weight=0.7):
    """Turn pairwise rotation binds into transitive chains.

    A→B, B→C, C→D (each a rotation bind above min_weight) = ONE chain = ONE
    entity slot. Union-find over the bind graph. Returns list of chains,
    each = ordered list of MACs (by first appearance).

    TWO edge sources (Aug 11, stream bind):
    1. PAIRWISE rotation binds — tight time-decay gate (weight ≥ min_weight,
       STRONG both sides, level-coherent). Catches handoffs the scanner sees
       within ~30s AND class-multiplied AirTag rotations. Excludes pairs with
       both companies known-but-DIFFERENT (a phone advertising TWO parallel
       BLE streams — two manufacturer companies — rotating in lockstep: the
       gap-0 cross-stream pair used to merge them into one chain).
    2. STREAM CONTINUATION — greedy temporal walk over per-MAC aggregates.
       Real rotation cadence (~14 min for the desk phone) is far beyond the
       tau=90s decay gate, and single-sample RSSI at the handoff is too noisy
       for the 6dB continuity check — so chains fragment. The walk reconnects:
       for each MAC B (first-seen order), bind the best ENDED predecessor A
       (gap ≤ stream_max_gap, STRONG both, |avgΔ| ≤ stream_level_gate, same
       company preferred, class-coherent) that no other MAC already claimed
       (one successor per MAC = no fan-out, no chain merging).
    """
    wm = WM
    # 1) pairwise rotation edges — plus company coherence: two KNOWN but
    #    different companies are different identities, never one slot.
    edges = [(r["from"], r["to"], r["weight"]) for r in rows
             if r["kind"] == "ROTATION?" and r["weight"] >= min_weight
             and r["from_tier"] == "STRONG" and r["to_tier"] == "STRONG"
             and abs(r["from_avg"] - r["to_avg"]) <= CO_OCCUR_LEVEL_GATE
             and not (r.get("from_company") and r.get("to_company")
                      and r["from_company"] != r["to_company"])]

    # 2) stream continuation edges
    if macs:
        from datetime import datetime
        max_gap = wm.get("stream_max_gap_seconds", 600)
        level_gate = wm.get("stream_level_gate", 4)
        claimed = {a for a, _b, _w in edges}  # MACs already bound as predecessor
        ordered = sorted(macs.values(), key=lambda m: m["first_ts"])
        for B in ordered:
            try:
                t_b = datetime.fromisoformat(B["first_ts"].replace("Z", "+00:00"))
            except Exception:
                continue
            best, best_key = None, None
            for A in ordered:
                if A["mac"] == B["mac"] or A["mac"] in claimed:
                    continue
                # ordered by first_ts: any candidate needs A.first < B.first
                # (A must end before B starts, so it starts earlier too)
                if A["first_ts"] >= B["first_ts"]:
                    break
                # temporal order: A must have ENDED before B started. Same
                # second = same scan batch (the node timestamps a whole scan
                # batch with one received_at) — still sequential, allow it;
                # the company filter below is what keeps parallel identities
                # apart.
                try:
                    t_a = datetime.fromisoformat(A["last_ts"].replace("Z", "+00:00"))
                except Exception:
                    continue
                if t_a > t_b:
                    continue
                gap = (t_b - t_a).total_seconds()
                if gap > max_gap:
                    continue
                if tier_of(A["avg_rssi"]) != "STRONG" or tier_of(B["avg_rssi"]) != "STRONG":
                    continue
                if abs(A["avg_rssi"] - B["avg_rssi"]) > level_gate:
                    continue
                # company: different KNOWN companies = different identities
                ca, cb = A.get("company_id"), B.get("company_id")
                if ca and cb and ca != cb:
                    continue
                # class coherence (Layer 4): recognized-class vs unknown is a
                # mismatch — an AirTag slot must not swallow an unknown device
                ca_cls, cb_cls = A.get("device_class"), B.get("device_class")
                if ((ca_cls in RECOGNIZED_CLASSES) != (cb_cls in RECOGNIZED_CLASSES)):
                    continue
                # rank: same known company first, then tightest gap
                key = (0 if (ca and cb and ca == cb) else 1, gap)
                if best_key is None or key < best_key:
                    best, best_key = A["mac"], key
            if best is not None:
                edges.append((best, B["mac"], 1.0))
                claimed.add(best)

    parent = {}
    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for a, b, _w in edges:
        union(a, b)

    # Group by root, preserve first-seen order within chain
    chains = {}
    for a, b, w in edges:
        root = find(a)
        chains.setdefault(root, set()).add(a)
        chains.setdefault(root, set()).add(b)

    first_seen = {m["mac"]: m["first_ts"] for m in macs.values()} if macs else {}
    ordered = []
    for root, macs_set in chains.items():
        if len(macs_set) < 2:
            continue
        if first_seen:
            ordered.append(sorted(macs_set, key=lambda m: first_seen.get(m, "")))
        else:
            ordered.append(sorted(macs_set))
    return ordered


def print_chains(c, hours=24):
    rows, macs = analyze_handoffs(c, hours, report=False, return_macs=True)
    chains = collapse_chains(rows, macs)
    print(f"\nENTITY CHAINS ({len(chains)} from {len(rows)} handoff pairs):")
    print("-" * 70)
    for chain in sorted(chains, key=len, reverse=True):
        # Look up labels for the first MAC
        lbl = ""
        cur = c.execute("SELECT label FROM devices WHERE mac=?", (chain[0],))
        r = cur.fetchone()
        if r and r["label"]:
            lbl = f" [{r['label']}]"
        print(f"  Entity{lbl}: {len(chain)} MACs in slot")
        print(f"    {' → '.join(m[:8] for m in chain[:12])}{' …' if len(chain) > 12 else ''}")
    return chains


def tier_of(avg):
    """LAYER 1: signal quality floor. STRONG resolves, EDGE counts only."""
    if avg >= -70:
        return "STRONG"
    if avg >= -85:
        return "MID"
    return "EDGE"


def analyze_handoffs(c, hours=24, report=True, return_macs=False):
    """Three-layer handoff analysis (data association in RSSI space).

    LAYER 1 — TIER: signal quality floor. STRONG resolves, EDGE counts only.
    LAYER 2 — TIME DECAY: P(same) = e^(-gap/tau). Tight gaps bind strongly.
    LAYER 3 — CONTINUITY + JUMP: rotation (level same) vs entry/exit (level jumped).
    LAYER 4 — CLASS COHERENCE: same recognized device_class at the same level
              = same entity (weight ×class_match_multiplier); different classes
              = different entities (weight ×class_mismatch_penalty).

    Each layer independently queryable — see --handoffs output columns.
    """
    ensure_schema(c)
    tau = 90.0           # Layer 2: decay constant (seconds) — tuned the hard way
    continuity_gate = 6  # Layer 3: |ΔRSSI| <= this = rotation (position preserved)
    jump_gate = 15       # Layer 3: |ΔRSSI| >= this = entry/exit (position changed)
    window_start = f"-{hours} hours"
    wm = WM  # class coherence knobs (config-tunable)

    # Per-minute presence per MAC
    cur = c.execute("""
    SELECT mac, substr(received_at,1,16) AS minute
    FROM sightings
    WHERE REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)
    GROUP BY mac, minute
    """, (window_start,))
    presence = {}
    for mac, minute in cur.fetchall():
        presence.setdefault(mac, set()).add(minute)

    # First/last RSSI + timestamps per MAC in window (+ latest device_class/company)
    cur = c.execute("""
    SELECT s1.mac,
           (SELECT rssi FROM sightings s2 WHERE s2.mac = s1.mac
            AND REPLACE(substr(s2.received_at,1,19),'T',' ') > datetime('now', ?) ORDER BY s2.received_at ASC LIMIT 1) AS first_rssi,
           (SELECT rssi FROM sightings s3 WHERE s3.mac = s1.mac
            AND REPLACE(substr(s3.received_at,1,19),'T',' ') > datetime('now', ?) ORDER BY s3.received_at DESC LIMIT 1) AS last_rssi,
           ROUND(AVG(s1.rssi),1) AS avg_rssi,
           MIN(s1.received_at) AS first_ts,
           MAX(s1.received_at) AS last_ts,
           COUNT(*) AS n,
           (SELECT device_class FROM sightings s4
            WHERE s4.mac = s1.mac AND s4.device_class IS NOT NULL
            AND REPLACE(substr(s4.received_at,1,19),'T',' ') > datetime('now', ?)
            ORDER BY s4.received_at DESC LIMIT 1) AS device_class,
           (SELECT company_id FROM sightings s5
            WHERE s5.mac = s1.mac AND s5.company_id IS NOT NULL
            AND REPLACE(substr(s5.received_at,1,19),'T',' ') > datetime('now', ?)
            ORDER BY s5.received_at DESC LIMIT 1) AS company_id
    FROM sightings s1
    WHERE REPLACE(substr(s1.received_at,1,19),'T',' ') > datetime('now', ?)
    GROUP BY s1.mac
    """, (window_start, window_start, window_start, window_start, window_start))
    macs = {r[0]: dict(r) for r in cur.fetchall()}

    # LAYER 1: tier by average RSSI
    def tier(avg):
        if avg >= -70:
            return "STRONG"
        if avg >= -85:
            return "MID"
        return "EDGE"

    # LAYER 4: class coherence factor for a pair.
    #  - both recognized (findmy/meta/flipper) and equal  → class_match   ×mult
    #  - one or both recognized but different              → class_mismatch ×pen
    #  - unknown/missing class carries no identity signal  → ×1.0
    def class_factor(ca, cb):
        ra, rb = ca in RECOGNIZED_CLASSES, cb in RECOGNIZED_CLASSES
        if ra and rb and ca == cb:
            return wm["class_match_multiplier"], "match"
        if (ra or rb) and ca != cb:
            return wm["class_mismatch_penalty"], "mismatch"
        return 1.0, "-"

    rows = []
    mac_list = list(macs.keys())
    for i, a in enumerate(mac_list):
        for b in mac_list[i+1:]:
            A, B = macs[a], macs[b]
            # temporal order: A ends before B starts
            if A["last_ts"] > B["first_ts"]:
                continue
            # Layer 2: gap between A's last and B's first
            from datetime import datetime
            try:
                t_a = datetime.fromisoformat(A["last_ts"].replace("Z", "+00:00"))
                t_b = datetime.fromisoformat(B["first_ts"].replace("Z", "+00:00"))
                gap_s = (t_b - t_a).total_seconds()
            except Exception:
                continue
            if gap_s < 0:
                continue
            # Layer 2: decay weight
            weight = math.exp(-gap_s / tau)

            # Layer 3: continuity vs jump
            delta = abs(A["last_rssi"] - B["first_rssi"])
            if delta <= continuity_gate:
                kind = "ROTATION?"
            elif delta >= jump_gate:
                kind = "ENTRY/EXIT?"
            else:
                kind = "AMBI"

            # Layer 4: class coherence weight
            cfac, cmatch = class_factor(A.get("device_class"), B.get("device_class"))
            weight = round(weight * cfac, 2)

            rows.append({
                "from": a, "to": b,
                "from_tier": tier(A["avg_rssi"]),
                "to_tier": tier(B["avg_rssi"]),
                "from_avg": A["avg_rssi"], "to_avg": B["avg_rssi"],
                "from_class": A.get("device_class") or "-",
                "to_class": B.get("device_class") or "-",
                "from_company": A.get("company_id"),
                "to_company": B.get("company_id"),
                "class_match": cmatch,
                "delta": delta, "gap_s": int(gap_s), "weight": weight,
                "kind": kind,
            })

    # Report: rotations first (tight gap + continuity), then jumps
    rows.sort(key=lambda r: (-(r["kind"] == "ROTATION?"), -r["weight"]))
    if report:
        print(f"Three-layer handoffs + class coherence ({len(rows)} pairs):")
        print(f"{'FROM':<20} {'TO':<20} {'ΔdB':>4} {'gap_s':>6} {'w':>5}  class       kind")
        print("-" * 78)
        shown = 0
        for r in rows:
            if r["from_tier"] != "STRONG" or r["to_tier"] != "STRONG":
                continue  # only STRONG tier resolves entities
            cm = {"match": "SAME", "mismatch": "DIFF", "-": "-"}[r["class_match"]]
            print(f"{r['from']:<20} {r['to']:<20} {r['delta']:>4} {r['gap_s']:>6} {r['weight']:>5}  {cm:<10} {r['kind']}")
            shown += 1
            if shown >= 40:
                break
        if shown == 0:
            print("  (no STRONG-tier handoffs in window)")
    if return_macs:
        return rows, macs
    return rows


def load_locations():
    """BSSID → label map (locations.json). 'Location is learned, not claimed.'"""
    if not os.path.exists(LOCATIONS_FILE):
        return {}
    try:
        with open(LOCATIONS_FILE) as f:
            return {k.lower(): v for k, v in (json.load(f) or {}).items()}
    except Exception:
        return {}


def places_view(c):
    """BSSID registry → PLACE entities.

    A BSSID that never moves (stable RSSI profile) + enough beacon sightings
    + a meaningful observation span = a PLACE. Labels resolve from
    locations.json when present (e.g. A8:F5:DD:CA:AC:3C → "home").

    Stability is measured from beacon_samples over a RECENT window (robust
    p10-p90 spread), NOT the places table's all-time min/max — a single
    outlier dip would otherwise permanently disqualify a stable place
    (all-time min/max only grows; the home AP dips -31 once and is "8dB
    unstable" forever). Falls back to all-time min/max when a BSSID has no
    samples yet (pre-beacon_samples collection).
    """
    ensure_schema(c)
    wm = WM
    rows = [dict(r) for r in c.execute(
        "SELECT * FROM places ORDER BY seen_count DESC")]
    locations = load_locations()

    # Recent-window sample spread per BSSID: (spread, n_samples, span_s).
    # SQLite window: normalize ISO 'T' → ' ' like everywhere else.
    w_start = (datetime.now(timezone.utc) -
               timedelta(seconds=wm["place_variance_window_seconds"])).isoformat(timespec="seconds")
    w_start_norm = w_start.replace("T", " ")
    sample_stats = {}
    for r in c.execute(
            """SELECT bssid,
                      COUNT(*) AS n,
                      MIN(rssi) AS mn, MAX(rssi) AS mx,
                      MIN(received_at) AS first_t, MAX(received_at) AS last_t
               FROM beacon_samples
               WHERE REPLACE(substr(received_at,1,19),'T',' ') > ?
               GROUP BY bssid""", (w_start_norm,)):
        # p10-p90 robust spread needs the sorted list — compute per-BSSID below
        sample_stats[r["bssid"]] = {
            "n": r["n"], "mn": r["mn"], "mx": r["mx"],
            "first_t": r["first_t"], "last_t": r["last_t"],
        }
    # Robust spread: p10-p90 of the recent samples (percentile, not max-min).
    robust = {}
    for bssid, st in sample_stats.items():
        vals = [r[0] for r in c.execute(
            """SELECT rssi FROM beacon_samples WHERE bssid=?
               AND REPLACE(substr(received_at,1,19),'T',' ') > ?
               ORDER BY rssi""", (bssid, w_start_norm))]
        if len(vals) >= wm["place_variance_min_samples"]:
            n = len(vals)
            lo = vals[max(0, int(n * 0.10) - 1)]
            hi = vals[min(n - 1, int(n * 0.90) - 1)]
            robust[bssid] = hi - lo

    places = []
    for r in rows:
        # Recent-window robust spread when we have samples; else all-time min/max.
        st = sample_stats.get(r["bssid"])
        if st and r["bssid"] in robust:
            variance = robust[r["bssid"]]
            variance_note = f"p10p90/{st['n']}"
        else:
            variance = (r["max_rssi"] or 0) - (r["min_rssi"] or 0)
            variance_note = f"minmax/{r['seen_count']}"
        span_s = 0
        try:
            t0 = datetime.fromisoformat(r["first_seen"].replace("Z", "+00:00"))
            t1 = datetime.fromisoformat(r["last_seen"].replace("Z", "+00:00"))
            span_s = (t1 - t0).total_seconds()
        except Exception:
            pass
        is_place = (r["seen_count"] >= wm["place_min_seen"]
                    and variance <= wm["place_max_rssi_variance"]
                    and span_s >= wm["place_min_span_seconds"])
        if is_place:
            c.execute("UPDATE places SET stable=1 WHERE bssid=?", (r["bssid"],))
        label = locations.get(r["bssid"], {}).get("label") if isinstance(locations.get(r["bssid"]), dict) else None
        places.append({
            "bssid": r["bssid"], "ssid": r["ssid"] or "?", "channel": r["channel"],
            "first_seen": r["first_seen"], "last_seen": r["last_seen"],
            "seen": r["seen_count"], "min": r["min_rssi"], "max": r["max_rssi"],
            "avg": r["avg_rssi"], "variance": variance, "var_note": variance_note,
            "span_h": round(span_s / 3600, 1),
            "place": is_place, "label": label,
        })
    c.commit()
    return places


def print_places(places):
    if not places:
        print("No BSSIDs in the places registry yet — wait for type:\"wifi\" "
              "beacon envelopes (firmware sends a batch every 5 min).")
        return
    print(f"{'PLACE':<6} {'SSID':<20} {'BSSID':<20} {'ch':>3} {'seen':>5} {'rssi':>10} {'var':>5} {'span_h':>6}  label")
    print("-" * 95)
    for p in places:
        mark = "PLACE" if p["place"] else "    "
        rng = f"{p['min']}..{p['max']}" if p["min"] is not None else "?"
        print(f"{mark:<6} {(p['ssid'] or '?')[:19]:<20} {p['bssid']:<20} {p['channel'] or '?':>3} "
              f"{p['seen']:>5} {rng:>10} {p['variance']:>4}{p['var_note']:>6} {p['span_h']:>6}  {p['label'] or ''}")
    n_places = sum(1 for p in places if p["place"])
    print(f"\n{len(places)} BSSIDs in registry, {n_places} resolve to PLACE "
          f"(seen>={WM['place_min_seen']}, var<={WM['place_max_rssi_variance']}dB "
          f"recent-{WM['place_variance_window_seconds']}s robust, "
          f"span>={WM['place_min_span_seconds']}s).")


def probes_view(c, limit=20):
    """Probe-request identity log: which client MACs seek which SSIDs.

    A client probing for an SSID registered as a place = a device that KNOWS
    our network (returning entity) — identity from the air, no connection.
    """
    ensure_schema(c)
    rows = [dict(r) for r in c.execute(
        """SELECT ssid, COUNT(*) AS n, COUNT(DISTINCT client_mac) AS clients,
                  GROUP_CONCAT(DISTINCT client_mac) AS macs,
                  MAX(received_at) AS last_seen
           FROM probes WHERE ssid IS NOT NULL AND ssid != ''
           GROUP BY ssid ORDER BY n DESC LIMIT ?""", (limit,))]
    if not rows:
        print("No probe requests logged yet — they arrive with type:\"wifi\" "
              "envelopes (any nearby device seeking a network).")
        return rows
    known = {r["ssid"] for r in c.execute("SELECT ssid FROM places WHERE ssid IS NOT NULL")}
    print(f"{'SSID':<24} {'probes':>6} {'clients':>7}  known-network seekers")
    print("-" * 76)
    for r in rows:
        seek = "RETURNING?" if r["ssid"] in known else ""
        print(f"{(r['ssid'] or '?')[:23]:<24} {r['n']:>6} {r['clients']:>7}  {seek}")
        macs = (r["macs"] or "").split(",")[:6]
        for m in macs:
            print(f"    └ {m}")
    return rows


def backfill_beacons(c):
    """Rebuild beacon_samples from the events JSONL history.

    The events table stores the full payload JSON of every envelope, including
    type:"wifi" beacon frames. This reconstructs the per-beacon sample log for
    deployments that collected wifi beacons before beacon_samples existed
    (idempotent: wipes and re-inserts from events).
    """
    ensure_schema(c)
    c.execute("DELETE FROM beacon_samples")
    rows = c.execute("SELECT received_at, payload FROM events WHERE type='wifi'").fetchall()
    n = 0
    for received_at, payload in rows:
        try:
            frames = json.loads(payload).get("frames", [])
        except Exception:
            continue
        for fr in frames:
            if not isinstance(fr, dict) or fr.get("kind") != "beacon":
                continue
            bssid = (fr.get("bssid") or fr.get("mac") or "").lower()
            rssi = fr.get("rssi")
            if bssid and rssi is not None:
                c.execute(
                    "INSERT INTO beacon_samples (bssid, received_at, rssi) VALUES (?,?,?)",
                    (bssid, received_at, rssi))
                n += 1
    c.commit()
    print(f"backfilled {n} beacon samples from {len(rows)} wifi envelopes.")


def where_view(c, label):
    """mosaic_where: resolve a place label → BSSIDs → current signal picture."""
    ensure_schema(c)
    locations = load_locations()
    bssids = [b for b, info in locations.items()
              if isinstance(info, dict) and info.get("label") == label]
    if not bssids:
        print(f"No location label '{label}' in {LOCATIONS_FILE}.")
        return
    for bssid in bssids:
        r = c.execute("SELECT * FROM places WHERE bssid=?", (bssid,)).fetchone()
        if not r:
            print(f"  {bssid} ({label}): no beacon sightings yet")
            continue
        print(f"  {bssid} ({label}) — ssid={r['ssid']} ch={r['channel']} "
              f"seen={r['seen_count']} rssi={r['min_rssi']}..{r['max_rssi']} "
              f"avg={r['avg_rssi']} last={r['last_seen']} "
              f"{'PLACE' if r['stable'] else 'not-yet-place'}")


def main():
    ap = argparse.ArgumentParser(description="ESP32-Mosaic world-model brain")
    ap.add_argument("--status", action="store_true", help="entity view of recent data")
    ap.add_argument("--devices", action="store_true", help="list device table")
    ap.add_argument("--seed-labels", action="store_true", help="import device labels JSON")
    ap.add_argument("--bind-slots", action="store_true", help="three-layer handoff analysis")
    ap.add_argument("--handoffs", action="store_true", help="alias for --bind-slots")
    ap.add_argument("--chains", action="store_true", help="collapse handoffs into entity chains")
    ap.add_argument("--places", action="store_true", help="BSSID registry → PLACE entities")
    ap.add_argument("--probes", action="store_true", help="probe-request identity log")
    ap.add_argument("--backfill-beacons", action="store_true",
                    help="rebuild beacon_samples from events JSON history")
    ap.add_argument("--where", metavar="LABEL", help="resolve a place label via BSSIDs")
    ap.add_argument("--hours", type=int, default=24, help="lookback window (default 24)")
    args = ap.parse_args()

    c = conn()
    ensure_schema(c)

    if args.seed_labels:
        seed_labels(c)
    if args.devices:
        for r in c.execute("SELECT mac, label, stable FROM devices ORDER BY stable DESC"):
            print(f"{'*' if r['stable'] else ' '} {r['mac']}  {r['label']}")
    if args.status:
        print_entity_view(entity_view(c, args.hours))
    if args.bind_slots or args.handoffs:
        analyze_handoffs(c, args.hours)
    if args.chains:
        print_chains(c, args.hours)
    if args.places:
        print_places(places_view(c))
    if args.probes:
        probes_view(c)
    if args.backfill_beacons:
        backfill_beacons(c)
    if args.where:
        where_view(c, args.where)
    if not (args.status or args.devices or args.seed_labels or args.bind_slots
            or args.handoffs or args.chains or args.places or args.probes
            or args.backfill_beacons or args.where):
        print_entity_view(entity_view(c, args.hours))


if __name__ == "__main__":
    main()
