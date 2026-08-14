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
  python3 mosaic_brain.py --devices         # known-device inventory (labeled + named)
  python3 mosaic_brain.py --seeds           # dump the devices table (identity seeds)
  python3 mosaic_brain.py --seed-labels     # import device labels JSON
  python3 mosaic_brain.py --chains          # class-weighted entity chains
  python3 mosaic_brain.py --places          # BSSID registry → PLACE entities
  python3 mosaic_brain.py --probes          # probe-request identity log
  python3 mosaic_brain.py --owner           # M1: is the owner home? (seed→chain resolution)
  python3 mosaic_brain.py --backfill-beacons  # rebuild beacon_samples from events
  python3 mosaic_brain.py --where "home"    # resolve device/node/owner/MAC/place label
"""

import argparse
import json
import math
import os
import re
import sqlite3
import sys
from datetime import datetime, timedelta, timezone

DB = os.path.expanduser("~/.orb/orb.db")
LABELS_FILE = os.path.expanduser("~/.mosaic/device_labels.json")
LOCATIONS_FILE = os.path.expanduser("~/.orb/locations.json")
# Owner-assignment cooldown cache (runtime state, next to the DB — never in
# the repo): the last candidate pick per owner seed, so minute-scale MAC
# rotation churn cannot flip the per-seed identity answer on every query.
OWNER_CACHE_FILE = os.path.expanduser("~/.orb/owner_assignment.json")

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


def _is_phantom_node(first_seen, last_seen, last_ip, model, firmware):
    """Detect a one-off test-registration phantom by its signature.

    Phantoms are 30-second test nodes that register from loopback, vanish
    forever, and pollute the --status node roster. The user-agent has filed
    this repeatedly (clawd, probe, route-probe, ws-test, test-node,
    gym-kali-01-test) — each is a new name, so chasing names in a config
    list is a losing game. Instead, detect by STRUCTURAL signature:

      - LOOPBACK IP (127.x.x.x): real nodes never register from loopback.
      - SHORT LIFE + NO IDENTITY: lifetime < phantom_min_lifetime_seconds
        AND no model/firmware reported (real ESP32 nodes report both).

    Either condition flags a phantom. A real node that briefly connected
    (power-loss reboot, etc.) but has a model/firmware survives — it has
    an identity, so it's a real node that may come back.

    Returns True if this row is a phantom, False otherwise.
    """
    # Loopback = always phantom (real nodes register from a real LAN IP).
    if last_ip and last_ip.startswith("127."):
        return True
    # Short life + no identity = phantom (30s test registrations).
    fs, ls = _parse_ts(first_seen), _parse_ts(last_seen)
    if fs and ls:
        life = (ls - fs).total_seconds()
        if life < WM.get("phantom_min_lifetime_seconds", 120) \
                and not model and not firmware:
            return True
    return False

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
    # Owner assignment cooldown (Aug 12): the owner-class devices at desk
    # level rotate MACs every 1-3 min, so the plain no-reuse best-fit pick
    # flips which candidate answers for which seed on almost every query
    # ('ask twice, get different MACs' — the 07:12 MEDIUM). The resolver
    # now HOLDS the last pick per seed (persisted to ~/.orb/owner_assignment.json)
    # and only re-assigns when the held candidate is absent longer than
    # owner_cooldown_absent_seconds (a real rotation has clearly happened)
    # or a new candidate's level fit is > owner_cooldown_improve_db better
    # (a genuinely better answer — not rotation noise).
    "owner_cooldown_absent_seconds": 600,
    "owner_cooldown_improve_db": 2.0,
    # Channel health (Aug 11): the --status heartbeat surface. Per-channel
    # max age before a channel reads STALE (2x = QUIET/OFFLINE). scan/wifi
    # are periodic streams (the node scans every ~20s, wifi cycle every few
    # min). probes are opportunistic (modern phones mostly passive-scan —
    # sparse is NORMAL): when a node still reports but nothing was heard,
    # probes reads QUIET, not OFFLINE (silence ≠ broken channel). csi is
    # event-driven: a still room produces no events for hours, so only a
    # 12h+ silence means the channel itself is gone (today no node sends
    # type:'csi' — the body channel is dead, now VISIBLE).
    "channel_scan_max_age_seconds": 600,
    "channel_wifi_max_age_seconds": 900,
    "channel_probes_max_age_seconds": 3600,
    "channel_csi_max_age_seconds": 43200,
    # Node registry hygiene: one-off test registrations (sub-minute lives,
    # loopback IPs) would show 'GONE' in --status forever. Two layers of
    # defense: (1) AUTO-DETECT by phantom signature (loopback IP or
    # lifetime < threshold with no model/firmware reported) — this catches
    # every test registration without chasing names; (2) the manual
    # node_ignore_list as a supplementary escape hatch for edge cases.
    # Auto-detect is the primary mechanism so the roster stays clean
    # across deployments without config edits.
    "phantom_min_lifetime_seconds": 120,
    "node_ignore_list": [],
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


def _collapse_named_products(rows):
    """Collapse rows sharing a broadcast name into one product row.

    BLE devices (earbuds, headphones) rotate their MAC address for privacy.
    A single physical product produces 5-15 rows — one per rotating MAC.
    This collapses them: the strongest avg-RSSI MAC is the representative;
    aggregated stats (total n, widest spread, latest last_seen) summarize
    the whole product. Seeded identities and findmy auto-labels are never
    collapsed (they are the identity anchor / distinct namespace).

    This is the pragmatic name-collapse fix for the rotating-MAC labeling
    issue (WF-C510, Px7 S3, MOMENTUM 4, etc.). The structural graph-based
    solution (Louvain co-occurrence, per research 04:29) is the full
    upgrade path; this handles the 90% case cleanly today.
    """
    by_name = {}
    ungrouped = []
    for r in rows:
        bn = r.get("broadcast_name")
        if bn and not r.get("seed") and not r.get("stable"):
            by_name.setdefault(bn, []).append(r)
        else:
            r["n_macs"] = 1
            ungrouped.append(r)

    out = []
    for name, group in by_name.items():
        if len(group) == 1:
            group[0]["n_macs"] = 1
            out.append(group[0])
            continue
        # Representative: strongest avg RSSI, most sightings as tiebreak.
        rep = max(group, key=lambda r: (r["avg"], r["n"]))
        rep = dict(rep)
        # Aggregate: total sightings, widest spread (the product's true
        # movement envelope across all its MACs), latest last_seen.
        rep["n"] = sum(r["n"] for r in group)
        rep["spread"] = max(r["spread"] for r in group)
        rep["spread_raw"] = max(r["spread_raw"] for r in group)
        rep["n_macs"] = len(group)
        rep["label"] = name
        out.append(rep)

    out.extend(ungrouped)
    return out


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
        broadcast_name = (s["name"].strip() if s["name"] and dclass != "findmy"
                          else None)

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
            "broadcast_name": broadcast_name,
            "seed": bool(lbl and lbl["label"]),
        })

    # Collapse rotating-MAC products: MACs sharing a broadcast name (and
    # not seeded) merge into one row. Representative = strongest avg RSSI.
    rows = _collapse_named_products(rows)

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
    print(f"{'LABEL':<26} {'class':<7} {'dclass':<9} {'avg':>5} {'spr':>4} {'raw':>4} {'n':>5} {'macs':>4}  mac")
    print("-" * 102)
    for r in rows[:limit]:
        mark = "MUTED" if r["muted"] else ""
        stable_m = "*" if r["stable"] else " "
        pc = "PLACE?" if r["place_candidate"] else ""
        nmacs = r.get("n_macs", 1)
        nmacs_s = f"{nmacs}" if nmacs > 1 else " "
        print(f"{stable_m}{r['label'][:25]:<25} {r['class']:<7} {r['dclass']:<9} {r['avg']:>5} {r['spread']:>4} {r['spread_raw']:>4} {r['n']:>5} {nmacs_s:>4}  {r['mac']} {mark} {pc}")
    tiers = ("KNOWN", "MOVING", "STATIC", "AMBI")
    counts = {t: sum(1 for r in rows if r["tier"] == i) for i, t in enumerate(tiers)}
    summary = " · ".join(f"{t}={counts[t]}" for t in tiers)
    collapsed = sum(1 for r in rows if r.get("n_macs", 1) > 1)
    total_macs = sum(r.get("n_macs", 1) for r in rows)
    cnote = f" · {collapsed} rotating-MAC products collapsed ({total_macs} MACs → {len(rows)} rows)" if collapsed else ""
    print(f"  {summary}  (of {len(rows)} device rows; spr = robust p10-p90 spread, raw = max-min){cnote}")


def _movement_class(spread):
    """Movement class from the robust p10-p90 spread (entity_view vocabulary)."""
    if spread <= STATIONARY_MAX_SPREAD:
        return "STATIC"
    if spread >= MOVING_MIN_SPREAD:
        return "MOVING"
    return "AMBI"


def devices_view(c, hours=24):
    """KNOWN-device inventory: every device with an EARNED identity.

    Earned = explicit label in the devices table (seeds/manual annotations)
    OR a device-reported broadcast name (TV/ThermoBeacon/earbuds/...).
    findmy-* auto labels are placeholders, not names — rotating AirTag MACs
    stay out of the inventory until someone names the underlying object
    (same rule as entity_view's KNOWN tier).

    ROTATING-MAC PRODUCTS: BLE devices like earbuds/headphones rotate their
    MAC address (privacy). A single product (e.g. "WF-C510-GFP") produces
    5-15 rows — one per rotating MAC. This view COLLAPSES MACs sharing a
    broadcast name into one row: the strongest/most recent MAC is the
    representative, and n_macs shows the rotation count. Seeded identities
    (devices table) are always one-row-per-MAC (they are the identity anchor).

    Each row: mac, label, dclass, window movement class, window avg/n,
    all-time last_seen, entity chain id (E-rank, same order as --chains),
    stable flag, seed flag. Window stats come from device_stats (needs >=3
    window sightings); seeds with zero sightings ever report UNSEEN, seeds
    last sighted outside the window report STALE (slot vocabulary of
    --owner). Entity ids come from the SAME full-history chain pass the
    owner/where resolvers use (168h) — a seed's entity = the slot chain its
    rotating MAC belongs to, so identity anchors link to live entities
    instead of dead seed MACs.
    """
    ensure_schema(c)
    labels = {r["mac"]: r for r in c.execute("SELECT * FROM devices")}
    stats = {r["mac"]: r for r in device_stats(c, hours)}

    # Device-reported names: the LATEST broadcast name per MAC (non-findmy
    # classes — a findmy name is not an earned label, see entity_view).
    named = {}
    for r in c.execute("""
        SELECT mac, name, device_class
        FROM sightings s
        WHERE name IS NOT NULL AND trim(name) != ''
          AND COALESCE(device_class, '') != 'findmy'
          AND id IN (SELECT MAX(id) FROM sightings
                     WHERE name IS NOT NULL AND trim(name) != ''
                     GROUP BY mac)
    """):
        named[r["mac"]] = dict(r)

    known_macs = sorted(set(labels) | set(named))

    # All-time last sighting per known MAC (one indexed pass).
    last_seen = {}
    if known_macs:
        q = ",".join("?" * len(known_macs))
        for r in c.execute(
            f"SELECT mac, MAX(received_at) AS last_seen FROM sightings "
            f"WHERE mac IN ({q}) GROUP BY mac", known_macs):
            last_seen[r["mac"]] = r["last_seen"]

    # Entity membership: full-history chain pass (168h — the same basis as
    # --owner/--where resolution), ranked by size like the --chains print
    # order (largest = E1). Best-effort — the inventory must not die
    # because the chain pass failed.
    chain_of = {}
    try:
        rows, macs_full = analyze_handoffs(c, 24 * 7, report=False,
                                           return_macs=True)
        for i, chain in enumerate(
                sorted(collapse_chains(rows, macs_full), key=len,
                       reverse=True), 1):
            for m in chain:
                chain_of.setdefault(m, f"E{i}")
    except Exception:
        pass

    # --- Collapse rotating-MAC products ---
    # Build per-MAC rows first (same as before), then group name-reported
    # rows by their broadcast name. Seeded identities (devices table) are
    # never collapsed — they are the identity anchor (one row per MAC).
    raw = []
    for mac in known_macs:
        lbl = labels.get(mac)
        nm = named.get(mac)
        st = stats.get(mac)
        seed = bool(lbl)
        label = (lbl["label"] if lbl and lbl["label"]
                 else (nm["name"] if nm else None)) or mac
        if st:
            cls = _movement_class(st["spread"])
            avg, n = st["avg_rssi"], st["n"]
        elif seed:
            cls = "UNSEEN" if not last_seen.get(mac) else "STALE"
            avg, n = None, None
        else:
            cls, avg, n = "-", None, None
        dclass = (st or {}).get("device_class") or (nm or {}).get("device_class") or "-"
        raw.append({
            "mac": mac, "label": label, "dclass": dclass, "class": cls,
            "avg": avg, "n": n, "last_seen": last_seen.get(mac),
            "entity": chain_of.get(mac, "-"),
            "stable": bool(lbl and lbl["stable"]), "seed": seed,
            "broadcast_name": nm["name"].strip() if nm else None,
        })

    # Group by broadcast name (collapse rotating MACs of one product).
    # Seeded rows and MACs with no name keep their own row.
    by_name = {}  # broadcast_name → [list of row dicts]
    ungrouped = []
    for r in raw:
        bn = r["broadcast_name"]
        if bn and not r["seed"]:
            by_name.setdefault(bn, []).append(r)
        else:
            ungrouped.append(r)

    # Collapse each name group into one representative row.
    # Representative = strongest avg RSSI (closest = best signal = the MAC
    # most likely to be seen). n = total sightings across all MACs.
    # last_seen = most recent across all MACs. entity = the representative's.
    out = []
    for name, group in by_name.items():
        # Sort: strongest first (avg RSSI closest to 0), most sightings as tiebreak
        with_stats = [r for r in group if r["avg"] is not None]
        without_stats = [r for r in group if r["avg"] is None]
        with_stats.sort(key=lambda r: (r["avg"], -r["n"]))
        ordered = with_stats + without_stats
        rep = dict(ordered[0])  # representative row
        # Aggregate: total n, latest last_seen, MAC count
        total_n = sum((r["n"] or 0) for r in group)
        latest_seen = max((r["last_seen"] for r in group
                          if r["last_seen"]), default=None)
        rep["n"] = total_n if total_n > 0 else None
        rep["last_seen"] = latest_seen or rep["last_seen"]
        rep["n_macs"] = len(group)
        # Label: keep the broadcast name; the MAC column shows the rep MAC
        rep["label"] = name
        out.append(rep)

    out.extend(ungrouped)
    out.sort(key=lambda r: (0 if r["stable"] else 1, r["label"].lower()))
    return out


def print_devices_view(rows, hours=24):
    """Render the known-device inventory."""
    if not rows:
        print("No known devices (empty devices table and no named broadcasts).")
        return
    collapsed = sum(1 for r in rows if r.get("n_macs", 1) > 1)
    total_macs = sum(r.get("n_macs", 1) for r in rows)
    suffix = (f", {total_macs} MACs → {len(rows)} rows: "
              f"{collapsed} rotating-MAC products collapsed"
              if collapsed else "no rotating-MAC products")
    print(f"\nKNOWN DEVICES ({len(rows)} products from {total_macs} MACs, "
          f"{hours}h window):")
    print(f"{'LABEL':<24} {'cls':<7} {'dclass':<9} {'avg':>5} {'n':>5} "
          f"{'macs':>4} {'last_seen':<23} {'entity':<7}  mac")
    print("-" * 118)
    for r in rows:
        stable_m = "*" if r["stable"] else " "
        avg = f"{r['avg']:>5.1f}" if r['avg'] is not None else "    -"
        n = f"{r['n']:>5}" if r['n'] is not None else "    -"
        nmacs = r.get("n_macs", 1)
        nmacs_s = f"{nmacs}" if nmacs > 1 else " "
        last = r["last_seen"] or "-"
        print(f"{stable_m}{r['label'][:23]:<23} {r['class']:<7} {r['dclass']:<9} "
              f"{avg} {n} {nmacs_s:>4} {last:<23} {r['entity']:<7}  {r['mac']}")
    seeds = sum(1 for r in rows if r["seed"])
    unseen = sum(1 for r in rows if r["class"] == "UNSEEN")
    stale = sum(1 for r in rows if r["class"] == "STALE")
    print(f"  {len(rows)} known products ({total_macs} MACs): {seeds} labeled seeds "
          f"({stale} STALE — outside window, {unseen} never sighted by BLE), "
          f"{len(rows) - seeds} name-reported; "
          f"entity = chain slot id (168h pass, see --chains, largest = E1)")


def print_seeds(c):
    """The devices-table dump (identity anchors). Formerly --devices."""
    rows = c.execute("SELECT mac, label, stable FROM devices "
                     "ORDER BY stable DESC, label").fetchall()
    print("SEEDS (devices table — labeled identity anchors; * = stable/WiFi):")
    for r in rows:
        print(f"{'*' if r['stable'] else ' '} {r['mac']}  {r['label']}")
    if not rows:
        print("  (empty — run --seed-labels to import device labels JSON)")



# --- OWNER PRESENCE (M1) ----------------------------------------------------
# 'Is the owner home?' — the killer question. Labeled owner BLE MACs rotate,
# so a seed MAC goes stale while the object is still here (the Aug-9 binding
# was 46h before today's streams). Resolution: trace the seed through its
# ENTITY SLOT (full-history chain) to the slot's most recent member = the
# object's current identity; presence = slot activity. When the seed slot is
# stale but an active STRONG device sits at the seed's signal level, surface
# it as an OWNER-SHAPED CANDIDATE and headline the seed PROBABLE: visible but
# NOT bound — labels are earned, and the WiFi-join correlation (the documented
# rebind path) is confirmation. STALE only when no fresh owner-shaped
# candidate exists.

def _seed_slot(slot_of, macs, mac):
    """Resolve a seed MAC through rotation chains → its slot's current occupant.

    Shared by owner_view (M1) and where_view (device lookup): a seed may be
    an old rotating MAC whose slot is now occupied by a newer MAC at the
    same signal level (the physical object did NOT move when its MAC
    rotated — the level anchor says so). Returns
    {active_mac, active_avg, anchor_avg, slot_n, age_s} with age_s = seconds
    since the slot's latest occupant was last sighted, or None when the seed
    was never sighted or its timestamps are unparseable. anchor_avg is
    unrounded — callers round for display.
    """
    info = macs.get(mac)
    if info is None:
        return None
    slot = slot_of.get(mac)
    if slot:
        mem = [macs[m] for m in slot if m in macs]
        mem.sort(key=lambda m: m["first_ts"])
        anchor_avg = sum(m["avg_rssi"] for m in mem) / len(mem)
        active = max(mem, key=lambda m: m["last_ts"])
        slot_n = len(slot)
    else:
        anchor_avg = info["avg_rssi"]
        active = info
        slot_n = 1
    last_dt = _parse_ts(active["last_ts"])
    if last_dt is None:
        return None
    age = (datetime.now(timezone.utc) - last_dt).total_seconds()
    return {"active_mac": active["mac"], "active_avg": active["avg_rssi"],
            "anchor_avg": anchor_avg, "slot_n": slot_n, "age_s": age}


def _level_candidates(macs, slot, anchor_avg, level_gate, present_s, now):
    """Strong, fresh, level-matched devices OUTSIDE a seed's own slot.

    The candidate surface for a stale slot: an active STRONG device at
    the seed's signal level (the physical object did not move when its
    MAC rotated — the level anchor says so). Reported, never auto-bound:
    labels are earned, WiFi-join stays the confirmation path. Shared by
    owner_view and where_view so both commands see the same candidate
    universe (sorted best level fit first).
    """
    cands = []
    for m in macs.values():
        if m["mac"] in (slot or []):
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
    return cands


def _no_reuse_assign(pending, exclude=None, skip=None):
    """Global no-reuse assignment across stale-slot seeds.

    Best level fit (smallest |Δ|) claims first; ties break by label for
    determinism. One candidate MAC serves at most ONE seed — a strong
    device never answers for two identities (the dual-radio phone case:
    one object advertising parallel streams must not double-count as
    phone AND watch). Shared by owner_view (M1) and where_view's chain
    path so both commands always agree on which candidate answers for
    which seed. `exclude` = MACs already claimed elsewhere (the cooldown
    holds of _stabilized_assign) — they are not re-claimed here.
    `skip` = pending indices that already hold a claim — they must not
    participate in the fresh greedy at all (a seed showing its held pick
    must not phantom-claim a candidate for itself).
    Returns (assigned, claimed_by): assigned[pending-index] -> candidate
    mac, claimed_by[mac] -> {"label": ..., "delta": ...}.
    """
    pairs = []
    for i, p in enumerate(pending):
        if skip and i in skip:
            continue
        anchor = p["res"]["anchor_avg"]
        for m in p["cands"]:
            if exclude and m["mac"] in exclude:
                continue
            pairs.append((abs(m["avg_rssi"] - anchor), i, m))
    pairs.sort(key=lambda t: (t[0], pending[t[1]]["seed"]["label"]))
    claimed_by = {}  # candidate mac -> {"label": ..., "delta": ...}
    assigned = {}    # pending index -> candidate mac
    for delta, i, m in pairs:
        if i in assigned or m["mac"] in claimed_by:
            continue
        assigned[i] = m["mac"]
        claimed_by[m["mac"]] = {"label": pending[i]["seed"]["label"],
                                "delta": delta}
    return assigned, claimed_by


def _load_owner_cache():
    """Read the owner-assignment cooldown cache (best-effort).

    Missing/corrupt cache = empty dict — the resolver degrades to the
    plain no-reuse assignment, never crashes. The file lives next to the
    DB (runtime state, never in the repo).
    """
    try:
        with open(OWNER_CACHE_FILE) as f:
            data = json.load(f)
        seeds = data.get("seeds") or {}
        return {k: v for k, v in seeds.items()
                if isinstance(v, dict) and v.get("cand")}
    except Exception:
        return {}


def _save_owner_cache(cache):
    """Persist the cooldown cache atomically (tmp + rename, best-effort)."""
    try:
        payload = {"v": 1,
                   "updated": datetime.now(timezone.utc).isoformat(
                       timespec="seconds"),
                   "seeds": cache}
        path = OWNER_CACHE_FILE
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f, indent=1)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        pass


def _stabilized_assign(pending, cache, macs, now):
    """No-reuse assignment with per-seed cooldown holds (churn killer).

    The owner-class cluster rotates MACs every 1-3 min at one RSSI level,
    so the plain greedy best-fit pick flips which candidate answers for
    which seed on almost every query. Each seed with a VALID cache entry
    HOLDS its last pick: the held candidate keeps answering while it is
    still level-matched and was seen within owner_cooldown_absent_seconds
    (a rotation just happened — the physical object did not move), unless
    a fresh candidate's level fit is > owner_cooldown_improve_db better
    (a genuinely better answer). Holds claim BEFORE fresh best-fit picks,
    so a seed's identity does not flip when a slightly-better-fitting
    MAC appears mid-rotation; the global no-reuse rule still holds (one
    device never answers for two identities — a seed whose held pick is
    claimed by a better-fitting seed falls back to the fresh pool).
    Never invents presence: a seed with no fresh level-matched candidate
    in the pool gets no assignment regardless of the cache.
    Returns (assigned, claimed_by, new_cache): assigned/claimed_by as in
    _no_reuse_assign (claimed_by includes holds), new_cache = the cache
    dict reflecting this run's final picks (entries dropped for seeds
    that ended without an assignment).
    """
    level_gate = WM.get("stream_level_gate", 4)
    present_s = WM.get("owner_present_seconds", 600)
    absent_s = WM.get("owner_cooldown_absent_seconds", 600)
    improve_db = WM.get("owner_cooldown_improve_db", 2.0)

    # Fresh no-reuse baseline: the best fit per seed without any hold.
    fresh_assigned, _ = _no_reuse_assign(pending)

    holds = []  # (delta, label, pending-index, mac)
    for i, p in enumerate(pending):
        c = (cache or {}).get(p["seed"]["mac"])
        if not c:
            continue
        m = macs.get(c["cand"])
        if m is None:
            continue  # candidate fell out of the window — hold is dead
        ld = _parse_ts(m["last_ts"])
        if ld is None:
            continue
        age = (now - ld).total_seconds()
        if age > absent_s:
            continue  # gone too long — adopt the new rotation
        delta = abs(m["avg_rssi"] - p["res"]["anchor_avg"])
        if delta > level_gate:
            continue  # the object moved level — hold is stale
        best_mac = fresh_assigned.get(i)
        if best_mac and best_mac != c["cand"]:
            best = macs[best_mac]
            best_delta = abs(best["avg_rssi"] - p["res"]["anchor_avg"])
            if delta - best_delta > improve_db:
                continue  # a genuinely better answer exists — switch
        holds.append((delta, p["seed"]["label"], i, c["cand"]))

    # Phase 1: valid holds claim first (better fit wins on conflict).
    holds.sort(key=lambda t: (t[0], t[1]))
    assigned = {}
    claimed_by = {}
    for delta, label, i, mac in holds:
        if i in assigned or mac in claimed_by:
            continue
        assigned[i] = mac
        claimed_by[mac] = {"label": label, "delta": delta}

    # Phase 2: seeds WITHOUT a hold claim from the remaining pool (seeds
    # that already hold must not participate — a seed showing its held
    # pick must not phantom-claim a candidate for itself and starve a
    # hold-less seed of its best fit).
    fresh, fresh_claims = _no_reuse_assign(pending,
                                           exclude=set(claimed_by),
                                           skip=set(assigned))
    for i, mac in fresh.items():
        if i not in assigned:
            assigned[i] = mac
    claimed_by.update(fresh_claims)

    # New cache: keep only seeds that ended with an assignment.
    new_cache = {}
    for i, p in enumerate(pending):
        mac = assigned.get(i)
        if mac is None:
            continue
        m = macs[mac]
        new_cache[p["seed"]["mac"]] = {
            "cand": mac,
            "avg": m["avg_rssi"],
            "delta": abs(m["avg_rssi"] - p["res"]["anchor_avg"]),
            "ts": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }
    return assigned, claimed_by, new_cache


def _resolve_owner_prefix(c):
    """Resolve the owner-label prefix, auto-detecting when the configured
    prefix matches zero seeds but seeds exist.

    The local config.yaml (gitignored) sets the real prefix for each
    deployment. If it's missing or stale (e.g., lost during a branch
    switch), the DEFAULT_WM fallback 'OWNER_' may not match the actual
    seed labels. Rather than silently breaking M1 (the product's headline
    question), auto-detect: if the configured prefix matches zero device
    labels but the devices table has labels with a shared uppercase prefix
    ending in '_', use that instead. This makes the owner resolver
    resilient to the exact failure that broke M1 when config.yaml was lost.

    Returns (prefix, auto_detected: bool).
    """
    configured = WM.get("owner_label_prefix", "OWNER_")
    count = c.execute(
        "SELECT COUNT(*) FROM devices WHERE label LIKE ?",
        (configured + "%",)
    ).fetchone()[0]
    if count > 0:
        return configured, False

    # Configured prefix matches nothing — try to auto-detect from actual
    # seed labels. Look for labels sharing an uppercase prefix ending in '_'.
    rows = c.execute(
        "SELECT DISTINCT label FROM devices "
        "WHERE label IS NOT NULL AND label != '' ORDER BY label"
    ).fetchall()
    if not rows:
        return configured, False  # no seeds at all — genuine empty state

    labels = [r["label"] for r in rows]
    # Find the longest common prefix among all labels.
    common = os.path.commonprefix(labels)
    # Trim to the last '_' so we get a clean namespace boundary
    # (e.g., 'TIGER_IPHONE' → 'TIGER_', not 'TIGER_I').
    if "_" in common:
        common = common[: common.rfind("_") + 1]
    else:
        common = ""

    # Only accept if the detected prefix is reasonable (2-20 chars,
    # uppercase + underscore) and matches at least 2 labels.
    if (common and 2 <= len(common) <= 20
            and common.replace("_", "").isupper()
            and common.endswith("_")):
        matched = sum(1 for l in labels if l.startswith(common))
        if matched >= 2:
            return common, True

    return configured, False


def _seed_probe_evidence(c, mac, hours=168):
    """Most recent DIRECTED probe-request sighting of a seed MAC.

    A directed probe (SSID present) from a seeded per-network WiFi MAC is a
    join-flow signal: the device seeks/rejoins that network with the same
    MAC it associates with. The BLE sniffer never sees WiFi MACs, so the
    probes channel is the only LIVE feed that can observe the WiFi identity
    (ARP firmware is merged but not deployed on any node). Evidence is
    transient — one probe does not equal connected — but it is real
    presence data: per-network MACs persist for weeks/months on modern
    OSes (research: McDougall 2022, Fenske 2021), so a seed MAC sighting
    is the same identity. Returns dict(ssid, received_at, rssi, channel,
    age_h) or None.
    """
    since = (datetime.now(timezone.utc) - timedelta(hours=hours)).isoformat(
        timespec="seconds").replace("T", " ")
    row = c.execute(
        """SELECT ssid, received_at, rssi, channel FROM probes
           WHERE client_mac = ? AND ssid IS NOT NULL AND ssid != ''
             AND REPLACE(substr(received_at,1,19),'T',' ') > ?
           ORDER BY received_at DESC LIMIT 1""", (mac, since)).fetchone()
    if not row:
        return None
    d = dict(row)
    dt = _parse_ts(d["received_at"])
    d["age_h"] = ((datetime.now(timezone.utc) - dt).total_seconds() / 3600
                  if dt else None)
    return d


def _seed_arp_evidence(c, mac, hours=168):
    """Most recent network-layer (ARP) sighting of a seed MAC.

    The ARP feed (firmware arp_join/arp_leave envelopes, payload carries
    mac + ip) is the WiFi-join confirmation the M1 resolver has been
    waiting for: a per-network WiFi MAC appearing as a network member is
    the strongest identity evidence this pipeline can produce for
    WiFi-only seeds (the BLE sniffer never sees WiFi MACs). Sampling
    caveat: node ARP cache expiry churns leave->join pairs within
    minutes, so the MOST RECENT event's type is the current-membership
    read. Returns dict(type, received_at, ip, age_h) or None.
    """
    since = (datetime.now(timezone.utc) - timedelta(hours=hours)).isoformat(
        timespec="seconds").replace("T", " ")
    row = c.execute(
        """SELECT type, received_at, payload FROM events
           WHERE type IN ('arp_join','arp_leave')
             AND json_valid(payload) AND json_extract(payload, '$.mac') = ?
             AND REPLACE(substr(received_at,1,19),'T',' ') > ?
           ORDER BY received_at DESC LIMIT 1""", (mac, since)).fetchone()
    if not row:
        return None
    d = dict(row)
    try:
        d["ip"] = json.loads(d["payload"]).get("ip")
    except Exception:
        d["ip"] = None
    dt = _parse_ts(d["received_at"])
    d["age_h"] = ((datetime.now(timezone.utc) - dt).total_seconds() / 3600
                  if dt else None)
    return d


def _seed_network_evidence(c, mac, hours=168):
    """Best network-layer evidence for a WiFi-only (BLE-blind) seed.

    Priority (strongest -> weakest): ARP join <=24h = network member now
    (the join-confirmation the resolver's PROBABLE/UNSEEN wording waits
    for); directed probe <=24h = transient seek signal (existing PROBED
    semantics); ARP leave <=24h = was a network member until that
    capture, not on the network as of it. Older evidence is folded into
    the UNSEEN annotation so no sighting is ever lost. Returns
    (state, detail, note); note carries the older-evidence tail.
    """
    arp = _seed_arp_evidence(c, mac, hours)
    probe = _seed_probe_evidence(c, mac, hours)
    note = ""
    if arp and arp["age_h"] is not None:
        ts = arp["received_at"][:19].replace("T", " ")
        if arp["type"] == "arp_join" and arp["age_h"] <= 24:
            return ("PRESENT",
                    f"joined home WiFi at {ts} (ip {arp['ip']}) — "
                    f"network-layer join-confirmation (ARP feed); BLE "
                    f"sniffer never sees this MAC", "")
        note = (f"; ARP: {'joined' if arp['type'] == 'arp_join' else 'left'} "
                f"home WiFi {arp['age_h']:.1f}h ago (ip {arp['ip']})")
    if probe and probe["age_h"] is not None:
        if probe["age_h"] <= 24:
            return ("PROBED",
                    f"WiFi radio probed known SSID '{probe['ssid']}' "
                    f"{probe['age_h']:.1f}h ago @ {probe['rssi']} dBm — "
                    f"transient join-confirmation (one probe ≠ connected; "
                    f"BLE sniffer never sees this MAC)", "")
        note += (f"; last directed probe of known SSID '{probe['ssid']}' "
                 f"was {probe['age_h']:.1f}h ago @ {probe['rssi']} dBm")
    if arp and arp["age_h"] is not None and arp["type"] == "arp_leave" \
            and arp["age_h"] <= 24:
        ts = arp["received_at"][:19].replace("T", " ")
        return ("LEFT",
                f"left home WiFi at {ts} ({arp['age_h']:.1f}h ago) — "
                f"network-layer departure (ARP feed); was join-confirmed "
                f"until then, not on the network as of that capture", "")
    return ("UNSEEN", "never sighted by the BLE sniffer (WiFi-only "
            "identity — resolve via WiFi-join correlation)", note)


def owner_view(c, hours=24, report=True):
    """M1 resolver: owner presence through entity chains."""
    ensure_schema(c)
    prefix, auto = _resolve_owner_prefix(c)
    seeds = [dict(r) for r in c.execute(
        "SELECT mac, label, note FROM devices WHERE label LIKE ? ORDER BY label",
        (prefix + "%",))]
    if not seeds:
        if report:
            print(f"OWNER: no seeded owner labels (devices label prefix '{prefix}'). "
                  "Seed via --seed-labels, then labels propagate here.")
        return []
    if auto and report:
        print(f"OWNER: auto-detected label prefix '{prefix}' "
              f"(configured '{WM.get('owner_label_prefix', 'OWNER_')}' "
              f"matched 0 seeds — add it to config.yaml to silence this).")
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
    pending = []  # stale-slot seeds still needing candidate assignment
    for s in seeds:
        mac = s["mac"]
        row = {"label": s["label"], "seed": mac}
        if macs.get(mac) is None:
            state, detail, note = _seed_network_evidence(c, mac)
            row["state"] = state
            row["detail"] = detail + note
            out.append(row)
            continue
        slot = slot_of.get(mac)
        res = _seed_slot(slot_of, macs, mac)
        if res is None:
            row["state"] = "UNRESOLVED"
            row["detail"] = "unparseable timestamps — cannot resolve"
            out.append(row)
            continue
        row["slot_n"] = res["slot_n"]
        row["anchor_avg"] = round(res["anchor_avg"], 1)
        row["active_mac"] = res["active_mac"]
        row["active_avg"] = res["active_avg"]
        row["age_s"] = res["age_s"]
        age = res["age_s"]
        anchor_avg = res["anchor_avg"]
        if age <= present_s:
            row["state"] = "PRESENT"
            row["detail"] = (f"slot active {int(age)}s ago via {row['active_mac']} "
                             f"@ {row['active_avg']:.1f} dB (slot {row['slot_n']} MACs, "
                             f"level {row['anchor_avg']})")
            out.append(row)
            continue
        # stale slot → owner-shaped candidate: active STRONG device at the
        # seed level, not part of the seed's own slot. Reported, never
        # auto-bound. Candidates are collected for ALL stale seeds first,
        # then assigned globally with NO REUSE — one strong device answers
        # for at most ONE identity (two seeds grabbing the same candidate
        # double-counted one device as phone AND watch).
        cands = _level_candidates(macs, slot or [mac], anchor_avg,
                                  level_gate, present_s, now)
        pending.append({"seed": s, "row": row, "res": res, "cands": cands})

    # Global no-reuse assignment with per-seed cooldown holds (shared
    # machinery — where_view uses the same, so both commands agree):
    # each seed HOLDS its last pick while it stays fresh/level-matched,
    # so minute-scale MAC rotation cannot flip which device answers for
    # which identity (the 07:12 churn MEDIUM). The hold is persisted to
    # ~/.orb/owner_assignment.json and survives across runs.
    cache = _load_owner_cache()
    assigned, claimed_by, new_cache = _stabilized_assign(pending, cache,
                                                         macs, now)
    _save_owner_cache(new_cache)

    for i, p in enumerate(pending):
        row, res = p["row"], p["res"]
        age = res["age_s"]
        anchor_avg = res["anchor_avg"]
        mac = assigned.get(i)
        if mac is None:
            if p["cands"]:
                # Fresh level-matched devices exist, but every one was claimed
                # by a better-fitting seed — one device cannot answer two
                # identities. Honest UNRESOLVED, not a borrowed answer.
                best = p["cands"][0]
                claim = claimed_by[best["mac"]]
                row["state"] = "UNRESOLVED"
                row["detail"] = (f"slot ended {age/3600:.1f}h ago; the strong "
                                 f"owner-shaped device at this level ({best['mac']} "
                                 f"@ {best['avg_rssi']:.1f} dB) is claimed by "
                                 f"{claim['label']} (Δ{claim['delta']:.1f}) — one "
                                 f"device cannot answer two identities; awaiting "
                                 f"WiFi-join confirmation")
            else:
                row["state"] = "STALE"
                row["detail"] = (f"slot ended {age/3600:.1f}h ago (last {row['active_mac']}, "
                                 f"level {row['anchor_avg']})")
            out.append(row)
            continue
        m = macs[mac]
        ld = _parse_ts(m["last_ts"])
        cd = (now - ld).total_seconds() if ld else present_s + 1
        held = " [held]" if (cache or {}).get(p["seed"]["mac"],
                                              {}).get("cand") == mac else ""
        row["candidate"] = (f"{m['mac']} @ {m['avg_rssi']:.1f} dB "
                            f"(Δ{abs(m['avg_rssi']-anchor_avg):.1f}) "
                            f"co={m['company_id'] or '-'} last {int(cd)}s ago "
                            f"— owner-shaped, unbound{held}")
        row["state"] = "PROBABLE"
        row["detail"] = (f"slot ended {age/3600:.1f}h ago, but an owner-shaped "
                         f"device is ACTIVE now ({int(cd)}s ago @ {m['avg_rssi']:.1f} "
                         f"dB, slot level {row['anchor_avg']}) — awaiting "
                         f"WiFi-join confirmation")
        # Level-coincidence honesty: another seed's ASSIGNED candidate at the
        # same level = the dual-radio phone signature (one object, two
        # parallel streams) or two co-located objects — BLE alone cannot tell
        # which is which. Say so instead of pretending the split is certain.
        for j, q in enumerate(pending):
            if j == i:
                continue
            om = macs.get(assigned.get(j, ""))
            if om is None:
                continue
            if abs(om["avg_rssi"] - m["avg_rssi"]) <= level_gate:
                row["candidate_note"] = (
                    f"level-coincident with {q['seed']['label']}'s candidate "
                    f"{om['mac'][:8]}… @ {om['avg_rssi']:.1f} dB — a dual-radio "
                    f"phone or two co-located devices; identities not separable "
                    f"by BLE alone (WiFi-join will split them)")
                break
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
        if r.get("candidate_note"):
            print(f"{'':<24} {'':<10} └ note: {r['candidate_note']}")
    print("  PRESENT = BLE slot active ≤ 10min, or WiFi-only seed confirmed "
          "on the network by the ARP feed (join ≤24h) · "
          "PROBABLE = slot stale but a strong "
          "owner-shaped device is ACTIVE at seed level now (unbound until "
          "WiFi-join confirmation) · PROBED = WiFi-only seed unseen by BLE but "
          "its radio directed-probed a known SSID <24h ago (transient join "
          "evidence — one probe ≠ connected) · "
          "LEFT = WiFi-only seed whose most recent network-layer sighting is "
          "a departure — was join-confirmed until then, not on the network "
          "as of that capture (ARP feed) · "
          "STALE = no owner-shaped evidence in window · "
          "UNRESOLVED = the only level-matched device is already claimed by "
          "another seed — one device never answers for two identities · "
          "[held] = this pick is held from the last run (rotation cooldown: "
          "the answer only changes when the held device is gone >10min or a "
          "fit >2dB better appears)")


def channel_health(c, report=True):
    """System channel heartbeat — is each sensing channel alive?

    Answers the user-agent HIGH issue: 'a dead sensor is visible instead
    of silently absent'. Channels:
      scan   — BLE sightings stream (node scans every ~20s)
      wifi   — WiFi beacon snapshots (offline scan cycle)
      probes — probe requests (opportunistic: sparse is NORMAL)
      csi    — body/motion channel (event-driven: a still room produces
               no events for hours — only a long silence (knob, 12h)
               means the channel itself is gone, e.g. no node sends
               type:'csi')
    States: LIVE (age <= max) / STALE (<= 2*max) / QUIET (> 2*max on a
    channel whose silence is normal, while a node still reports) /
    OFFLINE (> 2*max and nothing feeding the channel — the sensor
    itself stopped talking).
    Also lists known nodes with last contact, so the operator sees WHICH
    sensor stopped talking.
    """
    now = datetime.now(timezone.utc)
    # node heartbeats first — needed to tell QUIET (pipeline alive,
    # nothing to hear) from OFFLINE (no node reporting = channel unfed)
    ignore = set(WM.get("node_ignore_list", []) or [])
    nodes = []
    try:
        for r in c.execute(
                "SELECT node_id, first_seen, last_seen, last_ip, model, "
                "firmware FROM nodes ORDER BY last_seen DESC"):
            if r["node_id"] in ignore:
                continue
            if _is_phantom_node(r["first_seen"], r["last_seen"],
                                r["last_ip"], r["model"], r["firmware"]):
                continue
            dt = _parse_ts(r["last_seen"])
            age = (now - dt).total_seconds() if dt else None
            nodes.append({"node": r["node_id"], "age_s": age})
    except Exception:
        nodes = []
    feeder_alive = any(n["age_s"] is not None and n["age_s"] <= 7200
                       for n in nodes)
    channels = [
        ("scan",   "sightings",      "received_at", "channel_scan_max_age_seconds",
         "periodic stream — node should be reporting every ~20s", False),
        ("wifi",   "beacon_samples", "received_at", "channel_wifi_max_age_seconds",
         "periodic stream — beacon snapshots every few minutes", False),
        ("probes", "probes",         "received_at", "channel_probes_max_age_seconds",
         "opportunistic: sparse is normal (phones mostly passive-scan)", True),
        ("csi",    "csi_events",     "received_at", "channel_csi_max_age_seconds",
         "feature snapshots every ~5s + motion events; OFFLINE = no CSI-capable "
         "node reporting", False),
    ]
    out = []
    for name, table, col, knob, caveat, quiet_ok in channels:
        max_age = WM.get(knob, 600)
        try:
            row = c.execute(f"SELECT MAX({col}) v FROM {table}").fetchone()
            last = row["v"] if row else None
        except Exception:
            last = None
        if not last:
            out.append({"channel": name, "state": "EMPTY", "age_s": None,
                        "detail": f"{table} has no rows — channel never written"})
            continue
        dt = _parse_ts(last)
        if dt is None:
            out.append({"channel": name, "state": "UNPARSEABLE", "age_s": None,
                        "detail": f"last {table} ts '{last}' unparseable"})
            continue
        age = (now - dt).total_seconds()
        if age <= max_age:
            state = "LIVE"
        elif age <= max_age * 2:
            state = "STALE"
        elif quiet_ok and feeder_alive:
            state = "QUIET"
        else:
            state = "OFFLINE"
        detail = f"last event {age/3600:.1f}h ago"
        if state == "OFFLINE":
            detail += f" ({caveat})"
        elif state == "QUIET":
            detail += " — channel alive, nothing to hear (silence is normal here)"
        out.append({"channel": name, "state": state, "age_s": age, "detail": detail})
    if report:
        print_channel_health(out, nodes)
    return out, nodes


def print_channel_health(chans, nodes):
    print("\nCHANNELS — sensing pipeline heartbeat (LIVE ≤ threshold · STALE ≤ 2× · QUIET = alive, nothing to hear · OFFLINE = sensor gone):")
    print("-" * 78)
    for ch in chans:
        age = f"{ch['age_s']/3600:6.1f}h" if ch["age_s"] is not None else "     n/a"
        print(f"  {ch['channel']:<7} {ch['state']:<9} age {age}  {ch['detail']}")
    for n in nodes:
        age = f"{n['age_s']/3600:.1f}h" if n["age_s"] is not None else "n/a"
        state = "LIVE" if n["age_s"] is not None and n["age_s"] <= 600 else \
                ("STALE" if n["age_s"] is not None and n["age_s"] <= 7200 else "GONE")
        print(f"  node {n['node']:<22} {state:<6} last contact {age}")
    print("  OFFLINE = a sensor that stopped talking — visible instead of silent. QUIET = channel alive, silence is normal.")


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
        from bisect import bisect_left, bisect_right
        from datetime import datetime, timedelta
        max_gap = wm.get("stream_max_gap_seconds", 600)
        level_gate = wm.get("stream_level_gate", 4)
        claimed = {a for a, _b, _w in edges}  # MACs already bound as predecessor
        ordered = sorted(macs.values(), key=lambda m: m["first_ts"])
        # End-time index (the 12:50 perf HIGH's second O(M^2) loop: 16M
        # iterations at 4k MACs). A predecessor A can only bind B when
        # A's LAST sighting falls within stream_max_gap before B's FIRST —
        # the window shrinks the scan to the handful of MACs that actually
        # ended recently. _parse_ts normalizes naive→aware, so one index
        # (chronologically correct, no naive/aware compare issues) suffices.
        t_last = {m["mac"]: _parse_ts(m["last_ts"]) for m in macs.values()}
        end_sorted = sorted((t_last[m["mac"]], m["mac"]) for m in macs.values()
                            if t_last[m["mac"]] is not None)
        end_ts = [t for t, _m in end_sorted]
        end_mac = [m for _t, m in end_sorted]
        for B in ordered:
            t_b = _parse_ts(B["first_ts"])
            if t_b is None:
                continue
            lo = bisect_left(end_ts, t_b - timedelta(seconds=max_gap))
            hi = bisect_right(end_ts, t_b)
            window = [end_mac[i] for i in range(lo, hi)]
            if not window:
                continue
            # same iteration order as the old scan (first-seen order) so
            # equal-fit ties resolve identically
            window.sort(key=lambda m: (macs[m]["first_ts"], m))
            best, best_key = None, None
            for cand in window:
                if cand == B["mac"] or cand in claimed:
                    continue
                A = macs[cand]
                # temporal order: A must have ENDED before B started —
                # guaranteed by the window (t_a <= t_b). The old code's
                # string-order break also excluded same-first-seen MACs
                # (zero-length predecessors); mirror that exactly.
                if A["first_ts"] >= B["first_ts"]:
                    continue
                gap = (t_b - t_last[cand]).total_seconds()
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
                    best, best_key = cand, key
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
    gap = WM.get("stream_max_gap_seconds", 600)
    print(f"\nENTITY CHAINS ({len(chains)} from {len(rows)} handoff pairs, "
          f"{hours}h window, ≤{gap}s gap):")
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
        # MM-DD HH:MM — date-inclusive so cross-midnight spans don't look
        # backwards (time-only showed e.g. 04:17→04:16 for a 24h window)
        return f"{iso[5:10]} {iso[11:16]}" if iso and len(iso) >= 16 else "-"

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
                print(f"     shared absence: {ga[0].strftime('%m-%d %H:%M')}→{ga[1].strftime('%m-%d %H:%M')} "
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


def zone_of(avg):
    """Average RSSI → the README zone model (Z1 apartment / Z2 hallway /
    Z3 deep bleed). Same boundaries as tier_of — the zone is the
    operator-facing name of the tier."""
    if avg >= -70:
        return "Z1 apartment"
    if avg >= -85:
        return "Z2 hallway"
    return "Z3 deep bleed"


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
    # Pairs are generated from a WINDOWED scan instead of the full O(M^2)
    # loop (the 12:50 perf HIGH: 8M iterations + 3.78M dicts ~2.5-3GB at
    # 72h on 4k MACs — the 168h pass OOM'd a no-swap box). Only pairs with
    # A ending within stream_max_gap of B starting can ever matter:
    #   - chain edges need weight >= min_weight (0.7); the strongest class
    #     multiplier (1.5) caps the gap at ~69s — far inside the window.
    #   - the report shows the top-40 STRONG pairs by weight; beyond the
    #     window weight is ~0.00 and nothing displays.
    # Timestamps are parsed ONCE per MAC (same expression the loop used).
    # Naive/aware mixing: the old loop only ever subtracted same-kind
    # timestamps (a mixed pair raised TypeError and was skipped) — two
    # parallel end-time indexes preserve that exactly.
    from bisect import bisect_left, bisect_right
    from datetime import datetime, timedelta

    def _pair_parse(iso):
        try:
            return datetime.fromisoformat(iso.replace("Z", "+00:00"))
        except Exception:
            return None

    t_first = {m: _pair_parse(v["first_ts"]) for m, v in macs.items()}
    t_last = {m: _pair_parse(v["last_ts"]) for m, v in macs.items()}
    pair_window = wm.get("stream_max_gap_seconds", 600)
    naive_ends = sorted((t_last[m], m) for m in macs
                        if t_last[m] is not None and t_last[m].tzinfo is None)
    aware_ends = sorted((t_last[m], m) for m in macs
                        if t_last[m] is not None and t_last[m].tzinfo is not None)
    def _sortable(dt):
        return dt.replace(tzinfo=None) if dt is not None else None

    for b in sorted(macs.keys(), key=lambda m: (t_first[m] is None,
                                                _sortable(t_first[m]) or datetime.min)):
        t_b = t_first[b]
        if t_b is None:
            continue  # unparseable B — every pair involving it was skipped
        ends = aware_ends if t_b.tzinfo is not None else naive_ends
        lo = bisect_left(ends, (t_b - timedelta(seconds=pair_window), ""))
        hi = bisect_right(ends, (t_b, "\xff" * 32))
        for _t_a, a in ends[lo:hi]:
            if a == b:
                continue
            A, B = macs[a], macs[b]
            t_a = t_last[a]
            try:
                gap_s = (t_b - t_a).total_seconds()
            except Exception:
                continue  # mixed naive/aware — skipped by the old loop too
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
        gap = wm.get("stream_max_gap_seconds", 600)
        print(f"Three-layer handoffs + class coherence "
              f"({len(rows)} pairs, ≤{gap}s gap):")
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
    Per-client strength + recency are printed so an in-apartment probe
    (-33 dBm, hours ago) is distinguishable from neighbor noise (-90 dBm,
    days ago) without raw SQL.
    """
    ensure_schema(c)
    rows = [dict(r) for r in c.execute(
        """SELECT ssid, COUNT(*) AS n, COUNT(DISTINCT client_mac) AS clients,
                  MAX(received_at) AS last_seen
           FROM probes WHERE ssid IS NOT NULL AND ssid != ''
           GROUP BY ssid ORDER BY n DESC LIMIT ?""", (limit,))]
    if not rows:
        print("No probe requests logged yet — they arrive with type:\"wifi\" "
              "envelopes (any nearby device seeking a network).")
        return rows
    known = {r["ssid"] for r in c.execute("SELECT ssid FROM places WHERE ssid IS NOT NULL")}
    seed_label = {r["mac"]: r["label"] for r in c.execute(
        "SELECT mac, label FROM devices WHERE label IS NOT NULL")}
    clients = {r["ssid"]: [] for r in rows}
    for r in c.execute(
            """SELECT ssid, client_mac, COUNT(*) AS n, MAX(rssi) AS best_rssi,
                      MAX(received_at) AS last_seen
               FROM probes WHERE ssid IS NOT NULL AND ssid != ''
               GROUP BY ssid, client_mac ORDER BY COUNT(*) DESC"""):
        if r["ssid"] in clients:
            clients[r["ssid"]].append(dict(r))
    now = datetime.now(timezone.utc)
    print(f"{'SSID':<24} {'probes':>6} {'clients':>7}  known-network seekers")
    print("-" * 76)
    for r in rows:
        seek = "RETURNING?" if r["ssid"] in known else ""
        print(f"{(r['ssid'] or '?')[:23]:<24} {r['n']:>6} {r['clients']:>7}  {seek}")
        for m in clients[r["ssid"]][:6]:
            mark = f"  ★ {seed_label[m['client_mac']]}" if m["client_mac"] in seed_label else ""
            dt = _parse_ts(m["last_seen"])
            age = (now - dt).total_seconds() / 3600 if dt else None
            if age is None:
                age_s = "?"
            elif age >= 1:
                age_s = f"{age:.1f}h ago"
            else:
                age_s = f"{age * 60:.0f}min ago"
            print(f"    └ {m['client_mac']}{mark}  {m['n']}×  "
                  f"best {m['best_rssi']} dBm  last {m['last_seen'][:19]} ({age_s})")
    seen_seed = any(m["client_mac"] in seed_label
                    for r in rows for m in clients[r["ssid"]])
    if seen_seed:
        print("  ★ = labeled seed — a seed MAC directed-probing an SSID is "
              "join-confirmation evidence (see --owner/--where)")
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


def _owner_assignment(c, macs, slot_of):
    """The owner resolver's stabilized no-reuse assignment for where_view.

    Same seed set, same candidate collection, same claim order + the same
    per-seed cooldown holds as owner_view — so --where and --owner always
    agree on which candidate answers for which owner seed (and a queried
    non-owner seed can check whether its best candidate is already
    claimed: one device never answers for two identities). Persists the
    cooldown cache exactly like owner_view, so either command can
    release/refresh a hold and the next command sees the same answer.
    Returns (assign_by_seed_mac, claimed_by): assign_by_seed_mac maps
    seed MAC -> candidate MAC for seeds that won a claim; claimed_by maps
    candidate MAC -> {"label": ..., "delta": ...}.
    """
    prefix, _auto = _resolve_owner_prefix(c)
    seeds = [dict(r) for r in c.execute(
        "SELECT mac, label, note FROM devices WHERE label LIKE ? ORDER BY label",
        (prefix + "%",))]
    present_s = WM.get("owner_present_seconds", 600)
    level_gate = WM.get("stream_level_gate", 4)
    now = datetime.now(timezone.utc)
    pending = []
    for s in seeds:
        if macs.get(s["mac"]) is None:
            continue
        res = _seed_slot(slot_of, macs, s["mac"])
        if res is None or res["age_s"] <= present_s:
            continue
        cands = _level_candidates(macs, slot_of.get(s["mac"]) or [s["mac"]],
                                  res["anchor_avg"], level_gate, present_s, now)
        pending.append({"seed": s, "res": res, "cands": cands})
    cache = _load_owner_cache()
    assigned, claimed_by, new_cache = _stabilized_assign(pending, cache,
                                                         macs, now)
    _save_owner_cache(new_cache)
    by_mac = {}
    for i, p in enumerate(pending):
        if i in assigned:
            by_mac[p["seed"]["mac"]] = assigned[i]
    return by_mac, claimed_by


def _where_chain_path(d, macs, slot_of, owner_assign, owner_claim):
    """Chain-trace branch of where_view: the seed has no fresh sightings.

    Resolve the seed through rotation chains to its slot's CURRENT occupant
    (the physical object did not move when its MAC rotated — the level
    anchor says so), then, when the slot itself is stale, surface a live
    level-matched STRONG device at the slot's level — same evidence rule as
    owner_view's candidate (never auto-bound: labels are earned). Its zone
    IS the answer to 'where is it right now'.

    The live candidate comes from the SAME no-reuse assignment as --owner:
    an owner seed shows exactly the candidate the owner resolver assigned
    it, and a non-owner seed never borrows a candidate an owner seed
    already claimed — one device cannot answer for two identities.
    """
    res = _seed_slot(slot_of, macs, d["mac"])
    if res is None:
        print(f"  {d['label']:<24} UNSEEN — seed {d['mac']} never sighted "
              "by the BLE sniffer (WiFi-only identity?)")
        return
    zone = zone_of(res["active_avg"])
    rotated = "" if res["active_mac"] == d["mac"] else \
        f" (slot rotated from {d['mac']})"
    present_s = WM.get("owner_present_seconds", 600)
    if res["age_s"] <= present_s:
        verdict = f"PRESENT — active {int(res['age_s'])}s ago"
    else:
        verdict = f"slot ended {res['age_s']/3600:.1f}h ago"
    print(f"  {d['label']:<24} {verdict} — {res['active_mac']} @ "
          f"{res['active_avg']:.1f} dB, zone {zone}, slot {res['slot_n']} "
          f"MACs level {res['anchor_avg']:.1f}{rotated}")
    if res["age_s"] <= present_s:
        return
    level_gate = WM.get("stream_level_gate", 4)
    now = datetime.now(timezone.utc)
    cands = _level_candidates(macs, slot_of.get(d["mac"]) or [d["mac"]],
                              res["anchor_avg"], level_gate, present_s, now)
    # Owner seed: show exactly the shared no-reuse assignment (--owner and
    # --where then agree on which candidate answers for which seed).
    if d["mac"] in owner_assign:
        m = macs[owner_assign[d["mac"]]]
        cd = (now - _parse_ts(m["last_ts"])).total_seconds()
        print(f"  {'':<24} └ live: {m['mac']} @ {m['avg_rssi']:.1f} dB "
              f"(Δ{abs(m['avg_rssi']-res['anchor_avg']):.1f}) "
              f"zone {zone_of(m['avg_rssi'])} last {int(cd)}s ago "
              f"— level-matched, unbound")
        return
    if cands:
        best = cands[0]
        claim = (owner_claim or {}).get(best["mac"])
        if claim:
            print(f"  {'':<24} └ live: the strong level-matched device "
                  f"({best['mac'][:8]}… @ {best['avg_rssi']:.1f} dB) is "
                  f"claimed by {claim['label']} (Δ{claim['delta']:.1f}) — "
                  f"one device cannot answer two identities; awaiting "
                  f"WiFi-join confirmation")
        else:
            cd = (now - _parse_ts(best["last_ts"])).total_seconds()
            print(f"  {'':<24} └ live: {best['mac']} @ {best['avg_rssi']:.1f} dB "
                  f"(Δ{abs(best['avg_rssi']-res['anchor_avg']):.1f}) "
                  f"zone {zone_of(best['avg_rssi'])} last {int(cd)}s ago "
                  f"— level-matched, unbound")


_MAC_RE = re.compile(r"^([0-9a-f]{2}:){5}[0-9a-f]{2}$", re.IGNORECASE)


def _named_devices(c):
    """Latest non-findmy broadcast name per MAC — the name-reported label
    space (shared by --devices, --where exact and --where prefix)."""
    out = []
    for r in c.execute("""
        SELECT mac, name, device_class
        FROM sightings s
        WHERE name IS NOT NULL AND trim(name) != ''
          AND COALESCE(device_class, '') != 'findmy'
          AND id IN (SELECT MAX(id) FROM sightings
                     WHERE name IS NOT NULL AND trim(name) != ''
                     GROUP BY mac)
    """):
        out.append({"mac": r["mac"], "name": r["name"].strip(),
                    "device_class": r["device_class"]})
    return out


def _label_for_mac(c, mac):
    """Human label for a MAC: seeded label, else latest broadcast name."""
    r = c.execute("SELECT label FROM devices WHERE mac=?", (mac,)).fetchone()
    if r and r["label"]:
        return r["label"]
    r = c.execute(
        "SELECT name FROM sightings WHERE mac=? AND name IS NOT NULL "
        "AND trim(name) != '' ORDER BY id DESC LIMIT 1", (mac,)).fetchone()
    return r["name"] if r else None


def _where_print_device(c, label):
    """DEVICE lookup: seeded label or seed MAC (devices table) resolves
    FIRST — 'where is my phone' is the killer question. A seeded MAC may be
    an old rotating identity, so a seed with no fresh sightings is traced
    through entity chains to its slot's CURRENT occupant (same machinery as
    owner_view). Returns True when the label resolved to a device."""
    devs = [dict(r) for r in c.execute(
        "SELECT mac, COALESCE(label, mac) AS label, note, stable FROM devices "
        "WHERE lower(label) = lower(?) OR lower(mac) = lower(?)",
        (label, label))]
    if not devs:
        return False
    # Quick path: the seed itself was seen in the last 24h → answer
    # without the (costly) chain pass. Only stale seeds need chains.
    w_start = (datetime.now(timezone.utc) -
               timedelta(hours=24)).isoformat(timespec="seconds").replace("T", " ")
    quick = {}
    for d in devs:
        r = c.execute(
            """SELECT COUNT(*) n, MIN(rssi) mn, MAX(rssi) mx,
                      ROUND(AVG(rssi),1) avg, MAX(received_at) last_seen
               FROM sightings
               WHERE mac=? AND REPLACE(substr(received_at,1,19),'T',' ') > ?""",
            (d["mac"], w_start)).fetchone()
        quick[d["mac"]] = dict(r) if r and r["n"] else None
    macs = slot_of = None
    owner_assign = owner_claim = None
    # The chain pass only helps seeds that HAVE sightings history (a
    # stale slot resolves to its current occupant). A never-sighted
    # seed (WiFi-only identity) is UNSEEN unconditionally — running
    # the full 168h pass just to print that was the 12:50 timeout.
    need_chains = False
    for d in devs:
        if quick[d["mac"]] is None:
            r = c.execute("SELECT 1 FROM sightings WHERE mac=? LIMIT 1",
                          (d["mac"],)).fetchone()
            if r is not None:
                need_chains = True
                break
    if need_chains:
        rows, macs = analyze_handoffs(c, 24 * 7, report=False, return_macs=True)
        chains = collapse_chains(rows, macs)
        slot_of = {}
        for ch in chains:
            for m in ch:
                slot_of[m] = ch
        # The live-candidate step must agree with --owner: compute the
        # SAME no-reuse assignment over the owner seeds (a queried seed
        # never borrows a candidate another identity already claimed).
        owner_assign, owner_claim = _owner_assignment(c, macs, slot_of)
    print(f"\nDEVICE '{label}' — resolved through entity chains (168h):")
    print("-" * 78)
    for d in devs:
        q = quick[d["mac"]]
        if q is not None:
            print(f"  {d['label']:<24} last seen {q['last_seen']} — "
                  f"n={q['n']} avg {q['avg']} dB ({q['mn']}..{q['mx']}), "
                  f"zone {zone_of(q['avg'])}")
        elif macs is None or slot_of is None:
            state, detail, note = _seed_network_evidence(c, d["mac"])
            print(f"  {d['label']:<24} {state} — {detail}{note}")
        else:
            _where_chain_path(d, macs, slot_of, owner_assign, owner_claim)
    return True


def _where_print_place(c, label):
    """PLACE lookup: locations.json label → BSSIDs → current beacon picture."""
    locations = load_locations()
    ll = label.lower()
    bssids = [b for b, info in locations.items()
              if isinstance(info, dict) and
              str(info.get("label", "")).lower() == ll]
    if not bssids:
        return False
    print(f"\nPLACE '{label}' — learned place (locations.json → BSSID beacons):")
    print("-" * 78)
    for bssid in bssids:
        r = c.execute("SELECT * FROM places WHERE bssid=?", (bssid,)).fetchone()
        if not r:
            print(f"  {bssid} ({label}): no beacon sightings yet")
            continue
        print(f"  {bssid} ({label}) — ssid={r['ssid']} ch={r['channel']} "
              f"seen={r['seen_count']} rssi={r['min_rssi']}..{r['max_rssi']} "
              f"avg={r['avg_rssi']:.1f} last={r['last_seen']} "
              f"{'PLACE' if r['stable'] else 'not-yet-place'}")
    return True


def _where_print_named(c, label):
    """NAME-REPORTED broadcast devices — the label space --status/--devices
    print (SnowPax, Galaxy Fit3, ThermoBeacon, ...). Earned from the device
    itself, not from the devices table — 'every label the system shows is
    queryable' (04:55 LOW)."""
    hits = [d for d in _named_devices(c)
            if d["name"].lower() == label.lower()]
    if not hits:
        return False
    stats = {r["mac"]: r for r in device_stats(c, 24)}
    print(f"\nDEVICE '{label}' — name-reported broadcast label (the device "
          f"tells us its name):")
    print("-" * 78)
    for h in hits:
        mac = h["mac"]
        st = stats.get(mac)
        last = c.execute("SELECT MAX(received_at) v FROM sightings WHERE mac=?",
                         (mac,)).fetchone()["v"]
        seed = c.execute("SELECT label FROM devices WHERE mac=?",
                         (mac,)).fetchone()
        if st:
            cls = _movement_class(st["spread"])
            line = (f"  {mac} — last seen {last} · window n={st['n']} "
                    f"avg {st['avg_rssi']} dB (spread {st['spread']:.0f} dB → "
                    f"{cls}) · zone {zone_of(st['avg_rssi'])}")
            if st.get("device_class"):
                line += f" · class {st['device_class']}"
        else:
            line = f"  {mac} — last seen {last} · outside the 24h window"
        print(line)
        if seed and seed["label"]:
            print(f"    └ also a labeled seed: {seed['label']}")
    return True


def _where_print_node(c, q):
    """NODE lookup — 'where is my orb/robot' (20:50/00:50/13:20 filings).
    Answers with the node's liveness, identity, and what it currently sees."""
    row = c.execute(
        "SELECT node_id, first_seen, last_seen, last_ip, model, firmware "
        "FROM nodes WHERE lower(node_id) = lower(?)", (q,)).fetchone()
    if not row:
        return False
    if _is_phantom_node(row["first_seen"], row["last_seen"],
                        row["last_ip"], row["model"], row["firmware"]):
        return False  # phantom — treat as not found
    now = datetime.now(timezone.utc)
    dt = _parse_ts(row["last_seen"])
    age = (now - dt).total_seconds() if dt else None
    state = ("LIVE" if age is not None and age <= 600
             else ("STALE" if age is not None and age <= 7200 else "GONE"))
    age_s = f"{age/3600:.1f}h" if age is not None else "n/a"
    print(f"\nNODE '{row['node_id']}' — a sensing node (the brain's eye):")
    print("-" * 78)
    print(f"  node {row['node_id']} {state} — last contact {age_s} ago "
          f"({row['last_seen']})")
    if row["last_ip"] or row["model"] or row["firmware"]:
        print(f"  ip {row['last_ip'] or '-'}  model {row['model'] or '-'}  "
              f"firmware {row['firmware'] or '-'}")
    # What this node currently sees: distinct devices + its strongest few.
    w_start = (now - timedelta(hours=24)).isoformat(timespec="seconds").replace("T", " ")
    try:
        n_macs = c.execute(
            "SELECT COUNT(DISTINCT mac) n FROM sightings "
            "WHERE node_id=? AND REPLACE(substr(received_at,1,19),'T',' ') > ?",
            (row["node_id"], w_start)).fetchone()["n"]
        top = c.execute(
            "SELECT mac, MAX(rssi) best, MAX(received_at) last FROM sightings "
            "WHERE node_id=? AND REPLACE(substr(received_at,1,19),'T',' ') > ? "
            "GROUP BY mac ORDER BY best DESC LIMIT 3",
            (row["node_id"], w_start)).fetchall()
    except Exception:
        n_macs, top = None, []
    if n_macs:
        print(f"  sees {n_macs} distinct devices in the last 24h; strongest:")
        for t in top:
            nm = _label_for_mac(c, t["mac"])
            print(f"    {t['mac']}  {('(' + nm + ') ') if nm else ''}"
                  f"@ {t['best']} dB  last {t['last']}")
    else:
        print("  no sightings reported in the last 24h")
    return True


def _where_print_mac(c, mac):
    """RAW-MAC lookup (any MAC, sighted or not)."""
    mac = mac.lower()
    print(f"\nMAC {mac} — raw address lookup:")
    print("-" * 78)
    last = c.execute("SELECT MAX(received_at) v FROM sightings WHERE mac=?",
                     (mac,)).fetchone()["v"]
    if not last:
        print("  never sighted by the BLE sniffer")
        return
    st = next((r for r in device_stats(c, 24) if r["mac"] == mac), None)
    if st:
        cls = _movement_class(st["spread"])
        print(f"  last seen {last} · window n={st['n']} avg {st['avg_rssi']} dB "
              f"(spread {st['spread']:.0f} dB → {cls}) · "
              f"zone {zone_of(st['avg_rssi'])}")
    else:
        print(f"  last seen {last} · outside the 24h window (no stats)")


def _where_prefix(c, q):
    """Case-insensitive PREFIX match over devices + broadcast names + nodes.
    A unique match resolves through its own path; ambiguity is listed."""
    ql = q.lower()
    cands = []
    for r in c.execute("SELECT DISTINCT label FROM devices "
                       "WHERE label IS NOT NULL AND trim(label) != ''"):
        if r["label"].lower().startswith(ql):
            cands.append(("device", r["label"]))
    for d in _named_devices(c):
        if d["name"].lower().startswith(ql):
            cands.append(("device", d["name"]))
    for r in c.execute("SELECT DISTINCT node_id, first_seen, last_seen, "
                       "last_ip, model, firmware FROM nodes"):
        if r["node_id"].lower().startswith(ql):
            if _is_phantom_node(r["first_seen"], r["last_seen"],
                                r["last_ip"], r["model"], r["firmware"]):
                continue
            cands.append(("node", r["node_id"]))
    cands = sorted(set(cands))
    if not cands:
        return False
    if len(cands) == 1:
        typ, name = cands[0]
        if typ == "node":
            _where_print_node(c, name)
        else:
            if not _where_print_named(c, name):
                _where_print_device(c, name)
        return True
    print(f"'{q}' matches {len(cands)} labels — be specific:")
    for typ, name in cands:
        print(f"  {typ:<7} {name}")
    return True


def where_view(c, label):
    """mosaic_where: resolve a label to its current location picture.

    Resolution order — every name the system prints is queryable:
      1. 'owner' alias → the seeded owner entity group (M1 resolution)
      2. exact device label or seed MAC (devices table; chains when stale)
      3. exact place label (locations.json → BSSIDs → beacon picture)
      4. exact name-reported broadcast label (--devices' label space)
      5. exact node name (nodes registry — 'where is my orb/robot')
      6. raw MAC (any MAC, sighted or not)
      7. case-insensitive prefix (unique match resolves, else lists)
    Unknown labels list what IS known (devices, places, nodes).
    """
    ensure_schema(c)
    q = (label or "").strip()
    if not q:
        print("--where needs a label, node name, 'owner', or raw MAC.")
        return
    if q.lower() == "owner":
        print("\n'owner' → the seeded owner entity group (M1 resolution):")
        print("-" * 78)
        owner_view(c)
        return
    if _where_print_device(c, q):
        return
    if _where_print_place(c, q):
        return
    if _where_print_named(c, q):
        return
    if _where_print_node(c, q):
        return
    if _MAC_RE.match(q):
        _where_print_mac(c, q)
        return
    if _where_prefix(c, q):
        return
    known_devs = [r["label"] for r in c.execute(
        "SELECT DISTINCT label FROM devices WHERE label IS NOT NULL "
        "ORDER BY label")]
    known_places = sorted({lbl for info in load_locations().values()
                           if isinstance(info, dict)
                           and (lbl := info.get("label"))})
    known_nodes = [r["node_id"] for r in c.execute(
        "SELECT node_id, first_seen, last_seen, last_ip, model, firmware "
        "FROM nodes ORDER BY node_id")
        if not _is_phantom_node(r["first_seen"], r["last_seen"],
                                r["last_ip"], r["model"], r["firmware"])]
    print(f"No label '{q}' in the devices table, places, or node registry.")
    if known_devs:
        print(f"  Known device labels: {', '.join(known_devs)}")
    if known_places:
        print(f"  Known place labels:  {', '.join(known_places)}")
    if known_nodes:
        print(f"  Known node names:    {', '.join(known_nodes)}")
    print("  Tip: prefixes match case-insensitively ('--where tiger' works), "
          "'owner' aliases the owner group, raw MACs resolve too.")


def csi_patterns(c, hours=168):
    """Phase 2 pattern-layer seed: per-hour-of-day presence/motion counts
    from csi_events, with a 7-day exponential decay. The start of the
    "learn the normal" baseline.

    Shows typical activity by hour (7-day weighted average) vs today's
    activity so far. Breathing-band energy stats per hour too (Phase 2).
    """
    now = datetime.now(timezone.utc)
    # 7-day decay: weight = 0.5^(age_days/7). Queries the raw csi_events.
    # hour-of-day = CAST(strftime('%H', received_at) AS INTEGER)
    # We compute the weighted presence/motion counts per hour bucket.
    # NOTE: breathing columns may not exist yet (pre-Phase 2 DB). Try with
    # them, fall back to without.
    try:
        rows = c.execute("""
            SELECT CAST(strftime('%H', received_at) AS INTEGER) AS hod,
                   event, someone, moved, breathing_energy,
                   breathing_rate_hz, breathing_samples,
                   received_at
            FROM csi_events
            WHERE REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)
        """, (f"-{hours} hours",)).fetchall()
    except sqlite3.OperationalError:
        rows = c.execute("""
            SELECT CAST(strftime('%H', received_at) AS INTEGER) AS hod,
                   event, someone, moved, NULL AS breathing_energy,
                   NULL AS breathing_rate_hz, NULL AS breathing_samples,
                   received_at
            FROM csi_events
            WHERE REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)
        """, (f"-{hours} hours",)).fetchall()

    if not rows:
        print("csi-patterns: no csi_events in the lookback window.")
        return

    # Build per-hour weighted stats.
    from collections import defaultdict
    hod_data = defaultdict(lambda: {
        "presence_count": 0.0, "motion_count": 0.0,
        "breath_energy_sum": 0.0, "breath_energy_n": 0,
        "breath_rate_values": [],
        "today_presence": 0, "today_motion": 0,
    })
    today_str = now.strftime("%Y-%m-%d")

    for r in rows:
        hod = r["hod"]
        rt = r["received_at"] or ""
        # Parse age for decay weight
        try:
            # received_at is ISO format like 2026-08-13T14:30:00+00:00
            evt_time = datetime.fromisoformat(rt.replace("Z", "+00:00"))
            age_days = max(0.0, (now - evt_time).total_seconds() / 86400.0)
        except Exception:
            age_days = 7.0  # fallback: oldest weight
        weight = 0.5 ** (age_days / 7.0)

        d = hod_data[hod]
        if r["someone"]:
            d["presence_count"] += weight
        if r["moved"]:
            d["motion_count"] += weight
        if r["breathing_energy"] is not None:
            d["breath_energy_sum"] += r["breathing_energy"] * weight
            d["breath_energy_n"] += 1
            if r["breathing_rate_hz"] and r["breathing_rate_hz"] > 0:
                d["breath_rate_values"].append((r["breathing_rate_hz"], weight))

        # Today's actual counts (unweighted)
        if rt.startswith(today_str):
            if r["someone"]:
                d["today_presence"] += 1
            if r["moved"]:
                d["today_motion"] += 1

    # Print the pattern table
    print(f"\n{'='*72}")
    print(f"CSI PATTERNS — hour-of-day activity baseline (7-day decay)")
    print(f"{'='*72}")
    print(f"{'Hour':>4}  {'Pres(w)':>8} {'Mot(w)':>8}  {'BrE avg':>8} {'BrRate':>8}"
          f"  {'Today P':>8} {'Today M':>8}")
    print(f"{'-'*4}  {'-'*8} {'-'*8}  {'-'*8} {'-'*8}  {'-'*8} {'-'*8}")

    total_presence = 0.0
    total_motion = 0.0
    for hod in range(24):
        d = hod_data.get(hod)
        if not d:
            print(f"{hod:4d}  {'—':>8} {'—':>8}  {'—':>8} {'—':>8}"
                  f"  {'—':>8} {'—':>8}")
            continue
        breath_avg = (d["breath_energy_sum"] / d["breath_energy_n"]
                      if d["breath_energy_n"] > 0 else None)
        # Weighted average breathing rate
        if d["breath_rate_values"]:
            total_w = sum(w for _, w in d["breath_rate_values"])
            breath_rate = sum(r * w for r, w in d["breath_rate_values"]) / total_w if total_w > 0 else None
        else:
            breath_rate = None

        total_presence += d["presence_count"]
        total_motion += d["motion_count"]

        def fmt(v, spec=".3f"):
            return f"{v:{spec}}" if v is not None else "—"

        print(f"{hod:4d}  {d['presence_count']:8.2f} {d['motion_count']:8.2f}"
              f"  {fmt(breath_avg):>8} {fmt(breath_rate, '.2f'):>8}"
              f"  {d['today_presence']:8d} {d['today_motion']:8d}")

    print(f"{'─'*72}")
    print(f"{'TOTAL':>4}  {total_presence:8.2f} {total_motion:8.2f}"
          f"  {'':>8} {'':>8}  {'':>8} {'':>8}")
    print(f"\nPres(w)/Mot(w) = 7-day exponentially-decayed weighted counts")
    print(f"BrE avg = mean breathing_energy (0..1 band-power ratio)")
    print(f"BrRate  = weighted-mean breathing_rate_hz (0.2-0.5 = 12-30 BPM)")
    print(f"Today P/M = unweighted counts for today ({today_str})")
    print(f"Window: last {hours}h  |  Rows: {len(rows)}")

    # Quick anomaly flag: today's motion vs historical for the current hour
    current_hod = now.hour
    d = hod_data.get(current_hod)
    if d and d["today_motion"] > 0:
        # Compare today's count to the weighted historical average
        hist_motion = d["motion_count"]
        if hist_motion > 0 and d["today_motion"] > hist_motion * 2:
            print(f"\n⚠  Hour {current_hod}: today's motion ({d['today_motion']})"
                  f" is {d['today_motion']/max(hist_motion,0.01):.1f}× the 7-day"
                  f" weighted average ({hist_motion:.2f}) — above-normal activity")
        elif hist_motion > 0:
            print(f"\n✓  Hour {current_hod}: today ({d['today_motion']}) vs"
                  f" 7-day avg ({hist_motion:.2f}) — within normal range")


def main():
    ap = argparse.ArgumentParser(description="ESP32-Mosaic world-model brain")
    ap.add_argument("--status", action="store_true", help="entity view of recent data")
    ap.add_argument("--devices", action="store_true",
                    help="known-device inventory (labeled seeds + device-reported names)")
    ap.add_argument("--seeds", action="store_true",
                    help="dump the devices table (identity seed labels)")
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
    ap.add_argument("--channels", action="store_true",
                    help="sensing pipeline heartbeat — per-channel + per-node liveness")
    ap.add_argument("--csi-patterns", action="store_true",
                    help="CSI Phase 2: per-hour-of-day presence/motion patterns with 7-day decay")
    ap.add_argument("--backfill-beacons", action="store_true",
                    help="rebuild beacon_samples from events JSON history")
    ap.add_argument("--where", metavar="LABEL",
                    help="resolve device label, seed MAC, node name, 'owner', "
                         "raw MAC, or place label (locations.json → BSSIDs)")
    ap.add_argument("--hours", type=int, default=24, help="lookback window (default 24)")
    args = ap.parse_args()

    c = conn()
    ensure_schema(c)

    if args.seed_labels:
        seed_labels(c)
    if args.devices:
        print_devices_view(devices_view(c, args.hours), args.hours)
    if args.seeds:
        print_seeds(c)
    if args.status:
        channel_health(c)              # pipeline health gate FIRST — readings
        owner_view(c, args.hours)      # are only as good as the channels
        print_entity_view(entity_view(c, args.hours))
    if args.owner:
        owner_view(c, args.hours)
    if args.channels:
        channel_health(c)
    if args.csi_patterns:
        csi_patterns(c, args.hours)
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
    if not (args.status or args.devices or args.seeds or args.seed_labels
            or args.bind_slots
            or args.handoffs or args.chains or args.lockstep or args.places
            or args.probes or args.owner or args.channels or args.csi_patterns
            or args.backfill_beacons or args.where):
        print_entity_view(entity_view(c, args.hours))


if __name__ == "__main__":
    main()
