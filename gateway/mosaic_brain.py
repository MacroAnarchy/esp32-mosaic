#!/usr/bin/env python3
"""
mosaic_brain.py — ESP32-Mosaic world-model brain (v0.1 barebones).

Turns the raw sighting stream into ENTITIES:
  - Per-device RSSI stats (min/max/avg/spread) → stationary vs moving
  - Device labels/notes (from devices table, seeded from owner_devices.json)
  - Entity slots: rotating BLE MACs bind to stable anchors via co-occurrence
  - MOVING = MUTED: windows where the device moved are not location evidence

Usage:
  python3 mosaic_brain.py --status          # entity view of recent data
  python3 mosaic_brain.py --devices         # device table
  python3 mosaic_brain.py --seed-labels     # import owner_devices.json labels
"""

import argparse
import json
import math
import os
import sqlite3
import sys
from datetime import datetime, timezone

DB = os.path.expanduser("~/.orb/orb.db")
LABELS_FILE = os.path.expanduser("~/.orb/owner_devices.json")

# Tuning (static for now — tuned the hard way, see gateway config)
STATIONARY_MAX_SPREAD = 15   # dB: below this = stationary (data: Aqara 12, Philips 12)
MOVING_MIN_SPREAD = 30       # dB: above this = clearly moving (data: PAX 38, watch 46)
MUTED_SPREAD = 25            # dB: above this = location evidence muted
CO_OCCUR_BIND_SECONDS = 120  # new MAC seen within this of an anchor = slot candidate


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


def load_labels():
    """Seed labels from owner_devices.json (manual annotations)."""
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


def device_stats(c, hours=24):
    """Per-device RSSI stats over the window."""
    cur = c.execute("""
    SELECT mac,
           MIN(rssi) AS min_rssi,
           MAX(rssi) AS max_rssi,
           ROUND(AVG(rssi),1) AS avg_rssi,
           (MAX(rssi)-MIN(rssi)) AS spread,
           COUNT(*) AS n,
           MAX(name) AS name,
           MAX(s.received_at) AS last_seen
    FROM sightings s
    WHERE s.received_at > datetime('now', ?)
    GROUP BY mac
    HAVING n >= 3
    ORDER BY spread DESC
    """, (f"-{hours} hours",))
    return cur.fetchall()


def entity_view(c, hours=24):
    """The entity picture: devices → labels → movement class."""
    ensure_schema(c)
    stats = device_stats(c, hours)
    labels = {r["mac"]: r for r in c.execute("SELECT * FROM devices")}

    rows = []
    for s in stats:
        mac = s["mac"]
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

        rows.append({
            "mac": mac,
            "label": label,
            "stable": stable,
            "min": s["min_rssi"], "max": s["max_rssi"], "avg": s["avg_rssi"],
            "spread": s["spread"], "n": s["n"],
            "class": move_class, "muted": muted,
        })
    return rows


def print_entity_view(rows, limit=40):
    if not rows:
        print("No data in window (need >=3 sightings per device).")
        return
    print(f"{'LABEL':<26} {'class':<7} {'avg':>5} {'spr':>4} {'n':>5}  mac")
    print("-" * 78)
    for r in rows[:limit]:
        mark = "MUTED" if r["muted"] else ""
        stable_m = "*" if r["stable"] else " "
        print(f"{stable_m}{r['label'][:25]:<25} {r['class']:<7} {r['avg']:>5} {r['spread']:>4} {r['n']:>5}  {r['mac']} {mark}")


def analyze_handoffs(c, hours=24):
    """Three-layer handoff analysis (data association in RSSI space).

    LAYER 1 — TIER: signal quality floor. STRONG resolves, EDGE counts only.
    LAYER 2 — TIME DECAY: P(same) = e^(-gap/tau). Tight gaps bind strongly.
    LAYER 3 — CONTINUITY + JUMP: rotation (level same) vs entry/exit (level jumped).

    Each layer independently queryable — see --handoffs output columns.
    """
    ensure_schema(c)
    tau = 90.0           # Layer 2: decay constant (seconds) — tuned the hard way
    continuity_gate = 6  # Layer 3: |ΔRSSI| <= this = rotation (position preserved)
    jump_gate = 15       # Layer 3: |ΔRSSI| >= this = entry/exit (position changed)
    window_start = f"-{hours} hours"

    # Per-minute presence per MAC
    cur = c.execute("""
    SELECT mac, substr(received_at,1,16) AS minute
    FROM sightings
    WHERE received_at > datetime('now', ?)
    GROUP BY mac, minute
    """, (window_start,))
    presence = {}
    for mac, minute in cur.fetchall():
        presence.setdefault(mac, set()).add(minute)

    # First/last RSSI + timestamps per MAC in window
    cur = c.execute("""
    SELECT s1.mac,
           (SELECT rssi FROM sightings s2 WHERE s2.mac = s1.mac
             AND s2.received_at > datetime('now', ?) ORDER BY s2.received_at ASC LIMIT 1) AS first_rssi,
           (SELECT rssi FROM sightings s3 WHERE s3.mac = s1.mac
             AND s3.received_at > datetime('now', ?) ORDER BY s3.received_at DESC LIMIT 1) AS last_rssi,
           MIN(s1.received_at) AS first_ts,
           MAX(s1.received_at) AS last_ts,
           COUNT(*) AS n
    FROM sightings s1
    WHERE s1.received_at > datetime('now', ?)
    GROUP BY s1.mac
    """, (window_start, window_start, window_start))
    macs = {r[0]: dict(r) for r in cur.fetchall()}

    # LAYER 1: tier by average RSSI
    def tier(avg):
        if avg >= -70:
            return "STRONG"
        if avg >= -85:
            return "MID"
        return "EDGE"

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

            rows.append({
                "from": a, "to": b,
                "from_tier": tier(A.get("avg_rssi", A["first_rssi"])),
                "to_tier": tier(B.get("avg_rssi", B["first_rssi"])),
                "delta": delta, "gap_s": int(gap_s), "weight": round(weight, 2),
                "kind": kind,
            })

    # Report: rotations first (tight gap + continuity), then jumps
    rows.sort(key=lambda r: (-(r["kind"] == "ROTATION?"), -r["weight"]))
    print(f"Three-layer handoffs ({len(rows)} pairs):")
    print(f"{'FROM':<20} {'TO':<20} {'ΔdB':>4} {'gap_s':>6} {'w':>5}  kind")
    print("-" * 66)
    shown = 0
    for r in rows:
        if r["from_tier"] != "STRONG" or r["to_tier"] != "STRONG":
            continue  # only STRONG tier resolves entities
        print(f"{r['from']:<20} {r['to']:<20} {r['delta']:>4} {r['gap_s']:>6} {r['weight']:>5}  {r['kind']}")
        shown += 1
        if shown >= 40:
            break
    if shown == 0:
        print("  (no STRONG-tier handoffs in window)")
    return rows


def main():
    ap = argparse.ArgumentParser(description="ESP32-Mosaic world-model brain")
    ap.add_argument("--status", action="store_true", help="entity view of recent data")
    ap.add_argument("--devices", action="store_true", help="list device table")
    ap.add_argument("--seed-labels", action="store_true", help="import owner_devices.json")
    ap.add_argument("--bind-slots", action="store_true", help="three-layer handoff analysis")
    ap.add_argument("--handoffs", action="store_true", help="alias for --bind-slots")
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
    if not (args.status or args.devices or args.seed_labels or args.bind_slots or args.handoffs):
        print_entity_view(entity_view(c, args.hours))


if __name__ == "__main__":
    main()
