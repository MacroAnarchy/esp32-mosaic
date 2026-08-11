#!/usr/bin/env python3
"""
Mosaic MCP server — exposes the world model to any MCP-capable agent.

Hermes (or Claude, or any MCP client) connects to this server and gains
first-class tools to SENSE the physical world:
  - who is present right now
  - what entities the brain has resolved
  - what devices are in range, are they moving
  - raw queries against the sightings stream

Run:
  pip install mcp
  python3 mosaic_mcp.py                 # stdio transport (for Hermes config)
  python3 mosaic_mcp.py --http 8899     # optional HTTP transport

Hermes config (~/.hermes/config.yaml):
  mcp_servers:
    mosaic:
      command: "python3"
      args: ["/path/to/esp32-mosaic/gateway/mosaic_mcp.py"]
"""

import argparse
import json
import os
import sqlite3
from datetime import datetime, timezone

DB = os.path.expanduser("~/.orb/orb.db")

# Time-window filter normalization — same bug as mosaic_brain.py:
# received_at is ISO-8601 ('...T22:37:33+00:00'), SQLite datetime('now') is
# '... 22:37:33'; raw string comparison of 'T' vs ' ' silently ignored the
# hour and returned the whole UTC day for every --hours window. Normalize the
# stored column before comparing.
TS_WINDOW = "REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)"

# Movement-class thresholds — ROBUST p10-p90 spread (same as mosaic_brain):
# raw max-min lets one deep-fade sample (-106) flip a stationary AirTag to
# MOVING. p10-p90 trims the tails. Keep in sync with mosaic_brain.py.
STATIONARY_MAX_SPREAD = 15
MOVING_MIN_SPREAD = 25
SPREAD_MIN_SAMPLES = 10


def robust_spread(vals):
    if len(vals) < SPREAD_MIN_SAMPLES:
        return max(vals) - min(vals)
    s = sorted(vals)
    n = len(s)
    lo = s[max(0, int(n * 0.10) - 1)]
    hi = s[min(n - 1, int(n * 0.90) - 1)]
    return hi - lo


def conn():
    c = sqlite3.connect(DB)
    c.row_factory = sqlite3.Row
    return c


def _rows_to_dicts(rows):
    return [dict(r) for r in rows]


def tools():
    from mcp.server.fastmcp import FastMCP

    mcp = FastMCP("mosaic")

    @mcp.tool()
    def presence(hours: int = 2) -> str:
        """How many devices are currently in range, split by tier and movement class."""
        c = conn()
        cur = c.execute("""
        SELECT mac,
               MIN(rssi) AS min_rssi, MAX(rssi) AS max_rssi,
               ROUND(AVG(rssi),1) AS avg_rssi,
               (MAX(rssi)-MIN(rssi)) AS spread_raw,
               COUNT(*) AS n, MAX(name) AS name
        FROM sightings
        WHERE REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)
        GROUP BY mac HAVING n >= 3
        """, (f"-{hours} hours",))
        rows = _rows_to_dicts(cur.fetchall())

        # Robust spread per device (p10-p90) — one deep-fade sample must not
        # flip a stationary device to MOVING.
        for r in rows:
            vals = [v[0] for v in c.execute(
                "SELECT rssi FROM sightings WHERE mac=? "
                "AND REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?) "
                "ORDER BY rssi", (r["mac"], f"-{hours} hours"))]
            r["spread"] = robust_spread(vals)
            r["spread_raw"] = r.pop("spread_raw")

        def tier(avg):
            if avg >= -70:
                return "STRONG"
            if avg >= -85:
                return "MID"
            return "EDGE"

        def move_class(spread):
            if spread <= STATIONARY_MAX_SPREAD:
                return "STATIC"
            if spread >= MOVING_MIN_SPREAD:
                return "MOVING"
            return "AMBI"

        for r in rows:
            r["tier"] = tier(r["avg_rssi"])
            r["class"] = move_class(r["spread"])

        strong = [r for r in rows if r["tier"] == "STRONG"]
        mid = [r for r in rows if r["tier"] == "MID"]
        edge = [r for r in rows if r["tier"] == "EDGE"]
        moving = [r for r in rows if r["class"] == "MOVING"]
        static = [r for r in rows if r["class"] == "STATIC"]

        out = {
            "window_hours": hours,
            "total_devices": len(rows),
            "by_tier": {"STRONG": len(strong), "MID": len(mid), "EDGE": len(edge)},
            "by_movement": {"STATIC": len(static), "MOVING": len(moving), "AMBI": len(rows) - len(static) - len(moving)},
            "strong_devices": strong[:15],
        }
        return json.dumps(out, indent=1)

    @mcp.tool()
    def entities(hours: int = 24) -> str:
        """Labeled devices the brain knows about (stable anchors + labels)."""
        c = conn()
        cur = c.execute("""
        SELECT d.mac, d.label, d.note, d.stable, d.entity_id,
               (SELECT COUNT(*) FROM sightings s WHERE s.mac = d.mac
                 AND REPLACE(substr(s.received_at,1,19),'T',' ') > datetime('now', ?)) AS recent_hits
        FROM devices d
        ORDER BY d.stable DESC, d.label
        """, (f"-{hours} hours",))
        return json.dumps(_rows_to_dicts(cur.fetchall()), indent=1)

    @mcp.tool()
    def find_device(query: str, hours: int = 24) -> str:
        """Find a device by MAC prefix, label, or broadcast name. Returns stats."""
        c = conn()
        q = f"%{query}%"
        cur = c.execute("""
        SELECT mac, MIN(rssi) AS min_rssi, MAX(rssi) AS max_rssi,
               ROUND(AVG(rssi),1) AS avg_rssi,
               (MAX(rssi)-MIN(rssi)) AS spread_raw,
               COUNT(*) AS n, MAX(name) AS name,
               MAX(received_at) AS last_seen
        FROM sightings
        WHERE mac LIKE ? OR name LIKE ? OR mac IN (
            SELECT mac FROM devices WHERE label LIKE ? OR note LIKE ?
        )
        AND REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)
        GROUP BY mac ORDER BY n DESC LIMIT 20
        """, (q, q, q, q, f"-{hours} hours"))
        rows = _rows_to_dicts(cur.fetchall())
        for r in rows:
            vals = [v[0] for v in c.execute(
                "SELECT rssi FROM sightings WHERE mac=? "
                "AND REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?) "
                "ORDER BY rssi", (r["mac"], f"-{hours} hours"))]
            r["spread"] = robust_spread(vals) if vals else r.pop("spread_raw")
        return json.dumps(rows, indent=1)

    @mcp.tool()
    def nodes() -> str:
        """Which Mosaic nodes are connected and when they last reported."""
        c = conn()
        cur = c.execute("""
        SELECT n.node_id, n.first_seen, n.last_seen, n.last_ip, n.model,
               (SELECT COUNT(*) FROM sightings s WHERE s.node_id = n.node_id
                 AND REPLACE(substr(s.received_at,1,19),'T',' ') > datetime('now', '-1 hour')) AS hits_1h
        FROM nodes n ORDER BY n.last_seen DESC
        """)
        return json.dumps(_rows_to_dicts(cur.fetchall()), indent=1)

    @mcp.tool()
    def query(sql: str, limit: int = 50) -> str:
        """Run a read-only SQL query against the mosaic sightings database."""
        if not sql.strip().lower().startswith(("select", "with")):
            return json.dumps({"error": "read-only: only SELECT/WITH allowed"})
        c = conn()
        try:
            cur = c.execute(sql)
            rows = cur.fetchmany(limit)
            return json.dumps(_rows_to_dicts(rows), indent=1)
        except Exception as e:
            return json.dumps({"error": str(e)})

    @mcp.tool()
    def csi_status(hours: int = 24) -> str:
        """CSI motion events detected (ESP32-S3 nodes only)."""
        c = conn()
        cur = c.execute("""
        SELECT node_id, event, COUNT(*) AS n, MAX(received_at) AS last_seen
        FROM csi_events
        WHERE REPLACE(substr(received_at,1,19),'T',' ') > datetime('now', ?)
        GROUP BY node_id, event ORDER BY n DESC
        """, (f"-{hours} hours",))
        return json.dumps(_rows_to_dicts(cur.fetchall()), indent=1)

    return mcp


def main():
    ap = argparse.ArgumentParser(description="Mosaic MCP server")
    ap.add_argument("--http", type=int, default=None,
                    help="serve HTTP on this port instead of stdio")
    args = ap.parse_args()

    mcp = tools()
    if args.http:
        mcp.run(transport="http", host="0.0.0.0", port=args.http)
    else:
        mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
