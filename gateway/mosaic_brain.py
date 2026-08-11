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
  python3 mosaic_brain.py --owner           # M1: is the owner home? (seed→chain resolution)
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


def _parse_ts(iso):
    """Parse any stored ISO timestamp into an AWARE UTC datetime.

    The DB mixes formats: legacy/test-era rows carry naive
    'YYYY-MM-DDTHH:MM:SS' (no offset), newer rows carry '+00:00'. Comparing
    naive vs aware datetimes raises TypeError — which crashed full-history
    chain passes (24h windows never touched the old rows). Normalize
    everything to aware UTC.
    """
    if not iso:
        return None
    s = iso.replace("Z", "+00:00")
    try:
        dt = datetime.fromisoformat(s)
    except ValueError:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt

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
    # Lockstep (Aug 11): one physical object can advertise TWO parallel BLE
    # identities (the desk iPhone rotates a company-76 and a company-301 MAC
    # in lockstep: same -43dB level, ~14 min cadence each, rotations aligned
    # within 0-2 min for 25 consecutive rotations). The chains stay separate
    # (different companies never merge into one slot) but the LOCKSTEP pattern
    # identifies them as ONE object with TWO slots. Two independent devices
    # (e.g. two AirTags) rotate on their own schedules — alignment fails.
    "lockstep_min_macs": 4,             # min chain size to even consider
    "lockstep_level_gate": 4,           # dB: |avgA - avgB| (same as stream gate)
    "lockstep_cadence_tol": 1.5,        # cadence ratio A/B within [1/tol, tol]
    "lockstep_align_window_seconds": 180,  # rotation events within this = aligned
    "lockstep_gap_min_minutes": 6,      # silence this long = a real absence
    "lockstep_gap_global_frac": 0.5,    # gap mostly silent WORLDWIDE = node artifact
    "lockstep_gap_align_minutes": 5,    # shared gaps must START within this
    "lockstep_gap_min_shared_minutes": 15,  # shared overlap must be this long
    "lockstep_gap_shared_min": 0.5,     # min shared fraction of the shorter list
    # Resurrection (Aug 11): before the firmware attributed company_id
    # (~20:30 UTC 10-Aug) the desk phone's chains carried company '-' —
    # primary grouping (same company+class) can't join them to today's
    # 76/301 streams, hiding a departure that spans the attribution change
    # (measured: both identities silent 17:23:26→20:30:09 while the -94
    # noise floor kept streaming). A known-company stream inherits the
    # preceding same-level '-' stream as an ANCESTOR SLOT when the gap is
    # within this cap; the MAC set merges so the absence is computed from
    # real presence. 6h covers evening-out departures; a full workday away
    # stays unbridged until data shows it is safe.
    "lockstep_resurrect_max_gap_seconds": 21600,
    # Owner presence (Aug 11): the M1 resolver ('is the owner home?') traces
    # explicitly labeled owner seeds (devices table, label prefix below)
    # through entity chains to their current active identity. Prefix is
    # deployment-local (the live config.yaml sets the real one); the repo
    # ships a neutral example. owner_present_seconds = slot activity within
    # this = PRESENT (the desk phone is seen every ~20s scan; 10 min covers
    # scan gaps while staying crisp for a person walking out).
    "owner_label_prefix": "OWNER_",
    "owner_present_seconds": 600,
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


# --- OWNER PRESENCE (M1) ----------------------------------------------------
# 'Is the owner home?' — the killer question. Labeled owner BLE MACs rotate,
# so a seed MAC goes stale while the object is still here (the Aug-9 binding
# was 46h before today's streams). Resolution: trace the seed through its
# ENTITY SLOT (full-history chain) to the slot's most recent member = the
# object's current identity; presence = slot activity. When the seed slot is
# stale but an active STRONG device sits at the seed's signal level, surface
# it as an OWNER-SHAPED CANDIDATE: visible but NOT bound — labels are earned,
# and the WiFi-join correlation (the documented rebind path) is confirmation.

def owner_view(c, hours=24, report=True):
    """M1 resolver: owner presence through entity chains."""
    ensure_schema(c)
    prefix = WM.get("owner_label_prefix", "OWNER_")
    seeds = [dict(r) for r in c.execute(
        "SELECT mac, label, note FROM devices WHERE label LIKE ?",
        (prefix + "%",))]
    if not seeds:
        if report:
            print(f"OWNER: no seeded owner labels (devices label prefix '{prefix}'). "
                  "Seed via --seed-labels, then labels propagate here.")
        return []
    # Resolve against full history: seeds may predate the status window.
    rows, macs = analyze_handoffs(c, 24 * 7, report=False, return_macs=True)
    chains = collapse_chains(rows, macs)
    slot_of = {}
    for ch in chains:
        for m in ch:
            slot_of[m] = ch
    present_s = WM.get("owner_present_seconds", 600)
    level_gate = WM.get("stream_level_gate", 4)
    now = datetime.now(timezone.utc)

    out = []
    for s in seeds:
        mac = s["mac"]
        row = {"label": s["label"], "seed": mac}
        info = macs.get(mac)
        if info is None:
            row["state"] = "UNSEEN"
            row["detail"] = ("never sighted by the BLE sniffer (WiFi-only identity — "
                             "resolve via WiFi-join correlation)")
            out.append(row)
            continue
        slot = slot_of.get(mac)
        # Level anchor: the slot's mean level, or the seed's own mean when
        # the seed never joined a chain (isolated legacy MAC).
        if slot:
            mem = [macs[m] for m in slot if m in macs]
            mem.sort(key=lambda m: m["first_ts"])
            anchor_avg = sum(m["avg_rssi"] for m in mem) / len(mem)
            active = max(mem, key=lambda m: m["last_ts"])
            last_dt = _parse_ts(active["last_ts"])
            row["slot_n"] = len(slot)
            row["anchor_avg"] = round(anchor_avg, 1)
            row["active_mac"] = active["mac"]
            row["active_avg"] = active["avg_rssi"]
        else:
            anchor_avg = info["avg_rssi"]
            last_dt = _parse_ts(info["last_ts"])
            row["slot_n"] = 1
            row["anchor_avg"] = round(anchor_avg, 1)
            row["active_mac"] = mac
            row["active_avg"] = info["avg_rssi"]
        if last_dt is None:
            row["state"] = "UNRESOLVED"
            row["detail"] = "unparseable timestamps — cannot resolve"
            out.append(row)
            continue
        age = (now - last_dt).total_seconds()
        row["age_s"] = age
        if age <= present_s:
            row["state"] = "PRESENT"
            row["detail"] = (f"slot active {int(age)}s ago via {row['active_mac']} "
                             f"@ {row['active_avg']:.1f} dB (slot {row['slot_n']} MACs, "
                             f"level {row['anchor_avg']})")
        else:
            row["state"] = "STALE"
            row["detail"] = (f"slot ended {age/3600:.1f}h ago (last {row['active_mac'][:8]}…, "
                             f"level {row['anchor_avg']})")
            # owner-shaped candidate: active STRONG device at the seed level,
            # not part of the seed's own slot. Reported, never auto-bound.
            cands = []
            for m in macs.values():
                if m["mac"] in (slot or [mac]):
                    continue
                if tier_of(m["avg_rssi"]) != "STRONG":
                    continue
                if abs(m["avg_rssi"] - anchor_avg) > level_gate:
                    continue
                cd = _parse_ts(m["last_ts"])
                if cd is None or (now - cd).total_seconds() > present_s:
                    continue
                cands.append(m)
            cands.sort(key=lambda m: abs(m["avg_rssi"] - anchor_avg))
            if cands:
                m = cands[0]
                cd = (now - _parse_ts(m["last_ts"])).total_seconds()
                row["candidate"] = (f"{m['mac']} @ {m['avg_rssi']:.1f} dB "
                                    f"(Δ{abs(m['avg_rssi']-anchor_avg):.1f}) "
                                    f"co={m['company_id'] or '-'} last {int(cd)}s ago "
                                    f"— owner-shaped, unbound")
        out.append(row)
    if report:
        print_owner_view(out)
    return out


def print_owner_view(out):
    print("\nOWNER — seeded identity presence (resolved through entity chains):")
    print("-" * 78)
    for r in out:
        print(f"  {r['label']:<24} {r['state']:<10} {r['detail']}")
        if r.get("candidate"):
            print(f"{'':<24} {'':<10} └ candidate: {r['candidate']}")
    print("  PRESENT = slot active ≤ 10min · candidate = unlabeled STRONG device at "
          "seed level (unbound until WiFi-join confirmation)")


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
            t_b = _parse_ts(B["first_ts"])
            if t_b is None:
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
                    t_a = _parse_ts(A["last_ts"])
                except Exception:
                    continue
                if t_a is None or t_a > t_b:
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


def _chain_meta(macs, chain):
    """Aggregate a chain's members: level, span, rotation events, cadence.

    Rotation event = a successor MAC's first appearance (the identity
    changed). Cadence = median gap between consecutive rotation events.
    Returns dict or None when the chain is too small to reason about."""
    from datetime import datetime
    mem = [macs[m] for m in chain if m in macs]
    if len(mem) < 3:
        return None
    mem.sort(key=lambda m: m["first_ts"])
    avg = sum(m["avg_rssi"] for m in mem) / len(mem)
    t_first, t_last = _parse_ts(mem[0]["first_ts"]), _parse_ts(mem[-1]["last_ts"])
    span_s = (t_last - t_first).total_seconds() if t_first and t_last else 0
    events = []
    for m in mem[1:]:
        t = _parse_ts(m["first_ts"])
        if t is not None:
            events.append(t)
    gaps = [(b - a).total_seconds() for a, b in zip(events[:-1], events[1:])]
    cadence = sorted(gaps)[len(gaps) // 2] if gaps else None
    company = next((m["company_id"] for m in mem if m["company_id"]), None)
    cls = next((m["device_class"] for m in mem if m["device_class"]), None)
    return {"chain": chain, "n": len(chain), "avg": avg, "span_s": span_s,
            "events": events, "cadence_s": cadence, "company": company,
            "cls": cls, "first": mem[0]["first_ts"], "last": mem[-1]["last_ts"]}


def _chain_gaps(mac_minutes, chain, gap_min_minutes=6):
    """Absence intervals of a chain: silent runs >= gap_min_minutes within
    the chain's active span. A chain's presence = union of its members'
    active minutes (all slots of one identity)."""
    from datetime import timedelta
    mins = set()
    for m in chain:
        mins |= mac_minutes.get(m, set())
    if not mins:
        return []
    active = sorted(mins)
    t = active[0]
    end = active[-1]
    gaps = []
    silent_start = None
    while t <= end:
        if t in mins:
            if silent_start is not None:
                if (t - silent_start).total_seconds() >= gap_min_minutes * 60:
                    gaps.append((silent_start, t))
                silent_start = None
        else:
            if silent_start is None:
                silent_start = t
        t += timedelta(minutes=1)
    if silent_start is not None and (end - silent_start).total_seconds() >= gap_min_minutes * 60:
        gaps.append((silent_start, end + timedelta(minutes=1)))
    return gaps


def _group_streams(metas, level_gate):
    """Merge chains into IDENTITY STREAMS.

    A rotating identity (one company+class at one level) gets SPLIT into
    multiple chains whenever it is absent longer than stream_max_gap (the
    95-min departure splits the desk phone into night+day chains). Chains
    with the same company+class whose levels agree and whose spans are
    SEQUENTIAL (no overlap) are one continuing identity = one stream.
    Overlapping chains are parallel identities (two findmy devices alive
    at once) and stay separate streams.
    """
    by_key = {}
    for m in metas:
        by_key.setdefault((m["company"], m["cls"]), []).append(m)
    streams = []
    for key, ms in by_key.items():
        ms.sort(key=lambda m: m["first"])
        for m in ms:
            best_idx = -1
            for idx, s in enumerate(streams):
                if s["company"] != key[0] or s["cls"] != key[1]:
                    continue
                if abs(s["avg"] - m["avg"]) > level_gate:
                    continue
                # sequential: new chain starts after the stream's last end
                if m["first"] < s["last"]:
                    continue
                # prefer the stream with the earliest end among candidates
                if best_idx < 0 or s["last"] < streams[best_idx]["last"]:
                    best_idx = idx
            if best_idx >= 0:
                streams[best_idx]["chains"].append(m)
                streams[best_idx]["last"] = max(streams[best_idx]["last"], m["last"])
                n = streams[best_idx]["n"] + m["n"]
                streams[best_idx]["n"] = n
                streams[best_idx]["avg"] = (streams[best_idx]["avg"] * (n - m["n"]) + m["avg"] * m["n"]) / n
                streams[best_idx]["events"].extend(m["events"])
                streams[best_idx]["cadences"].append(m["cadence_s"])
            else:
                streams.append({"company": key[0], "cls": key[1], "chains": [m],
                                "n": m["n"], "avg": m["avg"],
                                "first": m["first"], "last": m["last"],
                                "events": list(m["events"]),
                                "cadences": [m["cadence_s"]]})
    for s in streams:
        s["events"].sort()
        s["cadence_s"] = sorted(s["cadences"])[len(s["cadences"]) // 2]
        s["macs"] = [mac for m in s["chains"] for mac in m["chain"]]
    return streams


def lockstep_pairs(c, hours=24, report=True):
    """Parallel-identity detection: identity streams that co-move = ONE object.

    The desk iPhone advertises TWO BLE identities (company 76 + company 301)
    at the same -43 dB level on independent ~14 min rotation timers. The
    timers run at slightly different periods, so rotation alignment beats and
    is NOT reliable evidence (two same-cadence rotators align by chance ~70%
    of the time). Cadence match is weak too (all Apple devices rotate on
    similar timers).

    The reliable signature is SHARED ABSENCE: when the object leaves, ALL of
    its identities leave together and return together. Gaps must be
    device-level (a gap that is silent WORLDWIDE is a node/gateway outage,
    not co-movement — the 05:00-06:36 gateway death looked like a shared
    departure until the global check caught it), start-aligned (departure
    is simultaneous), and long enough to be real. Level match screens
    candidates; shared absence confirms.

    Chains are first grouped into identity streams (same company+class+level,
    sequential spans) so that departures longer than the stream-bind gap
    don't hide the absence. Legacy streams from before company attribution
    (company '-') are RESURRECTED as ancestor slots of the same-level
    known-company stream that follows them — the 17:23→20:30 joint departure
    on 10-Aug spanned the attribution change and was otherwise invisible.
    Confirmed pairs are ONE object with two slots.
    Chains/streams are never merged: identities stay separate slots.
    """
    wm = WM
    min_macs = wm.get("lockstep_min_macs", 4)
    level_gate = wm.get("lockstep_level_gate", 4)
    align_win = wm.get("lockstep_align_window_seconds", 180)
    gap_min = wm.get("lockstep_gap_min_minutes", 6)
    gap_global_frac = wm.get("lockstep_gap_global_frac", 0.5)
    gap_align = wm.get("lockstep_gap_align_minutes", 5)
    gap_min_shared = wm.get("lockstep_gap_min_shared_minutes", 15)
    gap_shared_min = wm.get("lockstep_gap_shared_min", 0.5)

    rows, macs = analyze_handoffs(c, hours, report=False, return_macs=True)
    chains = collapse_chains(rows, macs)
    metas = [m for m in (_chain_meta(macs, ch) for ch in chains) if m
             and m["n"] >= min_macs and m["cadence_s"]]

    # per-MAC minute presence (same query the handoff layer uses)
    cur = c.execute("""
    SELECT mac, substr(received_at,1,16) AS minute
    FROM sightings
    WHERE REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)
    GROUP BY mac, minute
    """, (f"-{hours} hours",))
    from datetime import datetime, timedelta
    mac_minutes = {}
    global_minutes = set()
    for mac, minute in cur.fetchall():
        t = _parse_ts(minute)
        if t is not None:
            mac_minutes.setdefault(mac, set()).add(t)
            global_minutes.add(t)
    for m in metas:
        m["gaps"] = _chain_gaps(mac_minutes, m["chain"], gap_min)

    streams = _group_streams(metas, level_gate)

    # --- CROSS-COMPANY RESURRECTION ---------------------------------------
    # The node did not attribute company_id before the firmware fix
    # (~20:30 UTC 10-Aug), so that era's chains all carry company '-' and
    # primary grouping (same company+class) cannot join them to today's
    # 76/301 streams — a departure spanning the change is invisible to
    # shared-absence detection. Resurrection: a known-company stream may
    # inherit the preceding same-level legacy stream as an ANCESTOR SLOT
    # (sequential spans, gap within lockstep_resurrect_max_gap_seconds).
    # The MAC set merges, so the absence is computed from REAL presence
    # minutes (node-alive filtering below still applies). Multiple streams
    # may inherit the same ancestor: the legacy chain conflated both
    # parallel radios of one device, so a fork (both successors sharing
    # the ancestor's absence history) is the honest model.
    resurrect_max = wm.get("lockstep_resurrect_max_gap_seconds", 21600)
    inheritors = {}
    if resurrect_max > 0:
        def _p(iso):
            t = _parse_ts(iso)
            # unparseable → epoch sentinel so ordering comparisons stay safe
            return t if t is not None else datetime.min.replace(tzinfo=timezone.utc)
        legacy = [s for s in streams if s.get("company") is None]
        for s in streams:
            if s.get("company") is None:
                continue
            best = None
            for a in legacy:
                if abs(s["avg"] - a["avg"]) > level_gate:
                    continue
                if _p(a["last"]) > _p(s["first"]):
                    continue  # overlapping spans = parallel, not sequential
                if (_p(s["first"]) - _p(a["last"])).total_seconds() > resurrect_max:
                    continue
                if best is None or _p(a["last"]) > _p(best["last"]):
                    best = a
            if best is not None:
                best["consumed"] = True
                s["ancestor"] = best
                s["first"] = best["first"]
                s["n"] += best["n"]
                s["macs"] = best["macs"] + s["macs"]
                inheritors.setdefault(id(best), []).append(s)

    for s in streams:
        gaps = _chain_gaps(mac_minutes, s["macs"], gap_min)
        # tag node-level gaps: mostly silent WORLDWIDE (gateway/node outage
        # silences every device — that is not co-movement, it is an artifact;
        # the 05:00-06:36 gateway death looked like a shared departure)
        tagged = []
        for ga in gaps:
            tot = int((ga[1] - ga[0]).total_seconds() // 60)
            silent = sum(1 for i in range(tot)
                         if ga[0] + timedelta(minutes=i) not in global_minutes)
            tagged.append((ga, silent / tot if tot else 1.0))
        s["gaps"] = [ga for ga, frac in tagged if frac < gap_global_frac]
        s["global_gaps"] = sum(1 for _ga, frac in tagged if frac >= gap_global_frac)

    def fmt_ts(iso):
        return iso[11:19] if iso else "-"

    if report:
        print(f"\nIDENTITY STREAMS ({len(streams)} — rotating identities, {hours}h window):")
        print(f"{'#':>2} {'n':>3} {'co':>5} {'class':>8} {'avg':>6} {'cadence':>8} {'absences':>9}  first→last")
        print("-" * 78)
        for i, s in enumerate(streams):
            cad = f"{s['cadence_s']/60:.1f}min" if s["cadence_s"] else "-"
            glob = f" (+{s['global_gaps']} node)" if s.get("global_gaps") else ""
            anc = " *" if s.get("ancestor") else ""
            cons = ""
            if s.get("consumed"):
                kids = ",".join(f"#{streams.index(x)}" for x in inheritors.get(id(s), []))
                cons = f"  †→{kids}"
            print(f"{i:>2} {s['n']:>3} {str(s['company'] or '-'):>5} {str(s['cls'] or '-'):>8} "
                  f"{s['avg']:>6.1f} {cad:>8} {len(s['gaps']):>9}{glob}  "
                  f"{fmt_ts(s['first'])}→{fmt_ts(s['last'])}{anc}{cons}")
        if any(s.get("ancestor") for s in streams):
            print("  * = resurrected: legacy '-' era ancestor merged (company attribution changed "
                  "~20:30 UTC 10-Aug); absence now spans the change. † = consumed legacy stream.")

    # alignment fraction (informational — beat drift makes it weak evidence)
    def align_frac(A, B):
        short, long = (A, B) if len(A["events"]) <= len(B["events"]) else (B, A)
        aligned = sum(1 for e in short["events"]
                      if any(abs((e - f).total_seconds()) <= align_win for f in long["events"]))
        return aligned, len(short["events"])

    confirmed, candidates = [], []
    for i in range(len(streams)):
        for j in range(i + 1, len(streams)):
            A, B = streams[i], streams[j]
            if A.get("consumed") or B.get("consumed"):
                continue  # legacy slot already inherited by a live stream
            if abs(A["avg"] - B["avg"]) > level_gate:
                continue
            if tier_of(A["avg"]) != tier_of(B["avg"]):
                continue
            al, tot = align_frac(A, B)
            ratio = A["cadence_s"] / B["cadence_s"] if A["cadence_s"] and B["cadence_s"] else 0
            # shared-absence evidence: gaps of the shorter list that are
            # START-ALIGNED with a gap of the other stream (departure is
            # simultaneous; return can lag — one identity may resume late)
            # and share a real overlap (>= gap_min_shared minutes). Mere
            # containment is not alignment (a 3h gap that happens to contain
            # another's 1h gap is not co-movement).
            short_g, long_g = (A["gaps"], B["gaps"]) if len(A["gaps"]) <= len(B["gaps"]) else (B["gaps"], A["gaps"])
            shared = []
            for ga in short_g:
                for gb in long_g:
                    if abs((ga[0] - gb[0]).total_seconds()) > gap_align * 60:
                        continue
                    overlap = (min(ga[1], gb[1]) - max(ga[0], gb[0])).total_seconds()
                    if overlap >= gap_min_shared * 60:
                        shared.append(ga)
                        break
            if short_g and len(shared) / len(short_g) >= gap_shared_min:
                confirmed.append((A, B, shared, al, tot, ratio))
            else:
                candidates.append((A, B, shared, al, tot, ratio))

    if report:
        print(f"\nLOCKSTEP OBJECTS ({len(confirmed)} — co-moving identity streams = ONE object):")
        if not confirmed:
            print("  (none — no shared-absence evidence in window)")
        for A, B, shared, al, tot, ratio in sorted(confirmed, key=lambda p: -p[3]):
            print("-" * 78)
            print(f"  A: #{streams.index(A)} {A['n']} MACs co={A['company']} class={A['cls']} "
                  f"avg={A['avg']:.1f} cadence={A['cadence_s']/60:.1f}min {fmt_ts(A['first'])}→{fmt_ts(A['last'])}")
            print(f"  B: #{streams.index(B)} {B['n']} MACs co={B['company']} class={B['cls']} "
                  f"avg={B['avg']:.1f} cadence={B['cadence_s']/60:.1f}min {fmt_ts(B['first'])}→{fmt_ts(B['last'])}")
            for ga in shared[:4]:
                print(f"     shared absence: {ga[0].strftime('%H:%M')}→{ga[1].strftime('%H:%M')} "
                      f"({int((ga[1]-ga[0]).total_seconds()//60)}min) — left together")
            print(f"     level Δ={abs(A['avg']-B['avg']):.1f}dB · cadence ratio={ratio:.2f} · "
                  f"rotation alignment {al}/{tot} within {align_win//60}min (beat-drifts) → ONE object, 2 slots")

        print(f"\nCO-LOCATED CANDIDATES ({len(candidates)} — level-matched, no shared absence):")
        for A, B, shared, al, tot, ratio in candidates:
            print(f"  #{streams.index(A)}/{streams.index(B)}: level Δ={abs(A['avg']-B['avg']):.1f}dB "
                  f"cadence {A['cadence_s']/60:.1f} vs {B['cadence_s']/60:.1f}min "
                  f"alignment {al}/{tot} — no departure in window; unresolved")
    return confirmed


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
    ap.add_argument("--lockstep", action="store_true",
                    help="detect parallel-identity objects (chains rotating in lockstep)")
    ap.add_argument("--places", action="store_true", help="BSSID registry → PLACE entities")
    ap.add_argument("--probes", action="store_true", help="probe-request identity log")
    ap.add_argument("--owner", action="store_true",
                    help="M1: owner presence — resolve seeded owner labels through entity chains")
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
        owner_view(c, args.hours)          # M1 first: 'is the owner home?'
        print_entity_view(entity_view(c, args.hours))
    if args.owner:
        owner_view(c, args.hours)
    if args.bind_slots or args.handoffs:
        analyze_handoffs(c, args.hours)
    if args.chains:
        print_chains(c, args.hours)
    if args.lockstep:
        lockstep_pairs(c, args.hours)
    if args.places:
        print_places(places_view(c))
    if args.probes:
        probes_view(c)
    if args.backfill_beacons:
        backfill_beacons(c)
    if args.where:
        where_view(c, args.where)
    if not (args.status or args.devices or args.seed_labels or args.bind_slots
            or args.handoffs or args.chains or args.lockstep or args.places
            or args.probes or args.owner or args.backfill_beacons or args.where):
        print_entity_view(entity_view(c, args.hours))


if __name__ == "__main__":
    main()
