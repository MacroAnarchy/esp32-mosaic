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


def bind_slots(c, hours=24):
    """Barebones slot binding: for each STABLE device, find NEW MACs that
    co-occur (seen in the same minute as the anchor). Those become
    slot candidates for the anchor's entity. MUTED devices excluded."""
    ensure_schema(c)
    stable_devs = [r for r in c.execute("SELECT mac, label FROM devices WHERE stable=1")]
    if not stable_devs:
        print("No stable anchors seeded yet (run --seed-labels first).")
        return

    # snapshot of anchor appearances (minute buckets)
    for anchor in stable_devs:
        amac = anchor["mac"]
        cur = c.execute("""
        SELECT DISTINCT substr(received_at,1,16) AS minute
        FROM sightings WHERE mac=? AND rssi > -85
        """, (amac,))
        anchor_minutes = {r["minute"] for r in cur}
        if not anchor_minutes:
            continue

        # MACs seen in those same minutes (excluding the anchor itself)
        qmarks = ",".join("?" * len(anchor_minutes))
        cur = c.execute(f"""
        SELECT mac, COUNT(DISTINCT substr(received_at,1,16)) AS hits
        FROM sightings
        WHERE substr(received_at,1,16) IN ({qmarks})
          AND mac != ?
          AND rssi > -85
        GROUP BY mac
        ORDER BY hits DESC
        LIMIT 8
        """, (*anchor_minutes, amac))
        print(f"\nEntity slot for {anchor['label']} ({amac}):")
        for r in cur:
            ratio = round(r["hits"] / max(len(anchor_minutes), 1), 2)
            if ratio >= 0.5:
                print(f"  → {r['mac']}  co-occur {r['hits']}/{len(anchor_minutes)} ({ratio})")


def main():
    ap = argparse.ArgumentParser(description="ESP32-Mosaic world-model brain")
    ap.add_argument("--status", action="store_true", help="entity view of recent data")
    ap.add_argument("--devices", action="store_true", help="list device table")
    ap.add_argument("--seed-labels", action="store_true", help="import owner_devices.json")
    ap.add_argument("--bind-slots", action="store_true", help="find slot candidates for anchors")
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
    if args.bind_slots:
        bind_slots(c, args.hours)
    if not (args.status or args.devices or args.seed_labels or args.bind_slots):
        print_entity_view(entity_view(c, args.hours))


if __name__ == "__main__":
    main()
