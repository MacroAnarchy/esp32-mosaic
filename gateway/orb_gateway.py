#!/usr/bin/env python3
"""
orb_gateway.py — ORB Protocol v1 gateway (ingest layer).

Implements the ORB Protocol (see ~/.orb/protocol.md):
  - HTTP POST /orb/ingest   (deep-sleep swarm nodes)
  - WebSocket /ws           (interactive nodes: Orb, CSI)
  - Envelope validation (v1)
  - Canonical JSONL log (source of truth)
  - SQLite query layer (nodes, sightings, events)
  - Live subscriber forwarding (Hermes, dashboards)

Usage:
  python3 orb_gateway.py --port 9000
"""
import argparse
import asyncio
import json
import os
import sqlite3
import time
from datetime import datetime, timezone

from aiohttp import web, WSMsgType

HOST = "0.0.0.0"
DEFAULT_PORT = 9000
DATA_DIR = os.path.expanduser("~/.orb")
LOG_FILE = os.path.join(DATA_DIR, "presence.jsonl")
DB_FILE = os.path.join(DATA_DIR, "orb.db")

VALID_TYPES = {"scan", "csi", "imu", "state"}

# ---------------------------------------------------------------------------
# SQLite layer
# ---------------------------------------------------------------------------

SCHEMA = """
CREATE TABLE IF NOT EXISTS nodes (
    node_id     TEXT PRIMARY KEY,
    first_seen  TEXT,
    last_seen   TEXT,
    last_ip     TEXT,
    model       TEXT,
    firmware    TEXT
);
CREATE TABLE IF NOT EXISTS events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    received_at TEXT,
    node_id     TEXT,
    type        TEXT,
    payload     TEXT
);
CREATE TABLE IF NOT EXISTS sightings (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    received_at TEXT,
    node_id     TEXT,
    mac         TEXT,
    rssi        INTEGER,
    name        TEXT
);
CREATE TABLE IF NOT EXISTS csi_events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    received_at TEXT,
    node_id     TEXT,
    event       TEXT,
    someone     INTEGER,
    moved       INTEGER,
    wander      REAL,
    jitter      REAL
);
CREATE INDEX IF NOT EXISTS idx_sightings_mac ON sightings(mac);
CREATE INDEX IF NOT EXISTS idx_sightings_time ON sightings(received_at);
CREATE INDEX IF NOT EXISTS idx_events_node ON events(node_id);
"""


class DB:
    def __init__(self, path):
        self.conn = sqlite3.connect(path)
        self.conn.executescript(SCHEMA)
        self.conn.commit()

    def record(self, rec):
        """Store one stamped envelope."""
        now = rec.get("server_received_at", "")
        node = rec.get("node", "?")
        typ = rec.get("type", "?")
        payload = rec.get("payload", {})
        cur = self.conn.cursor()

        # events
        cur.execute(
            "INSERT INTO events (received_at, node_id, type, payload) VALUES (?,?,?,?)",
            (now, node, typ, json.dumps(payload)),
        )

        # nodes upsert
        model = payload.get("model") if isinstance(payload, dict) else None
        fw = payload.get("firmware") if isinstance(payload, dict) else None
        cur.execute(
            """INSERT INTO nodes (node_id, first_seen, last_seen, last_ip, model, firmware)
               VALUES (?,?,?,?,?,?)
               ON CONFLICT(node_id) DO UPDATE SET
                 last_seen=excluded.last_seen, last_ip=excluded.last_ip,
                 model=COALESCE(excluded.model, model), firmware=COALESCE(excluded.firmware, firmware)""",
            (node, now, now, rec.get("source_ip", ""), model, fw),
        )

        # sightings
        if typ == "scan":
            for dev in payload.get("devices", []):
                cur.execute(
                    "INSERT INTO sightings (received_at, node_id, mac, rssi, name) VALUES (?,?,?,?,?)",
                    (now, node, dev.get("mac", "?"), dev.get("rssi"), dev.get("name")),
                )

        # csi events
        if typ == "csi":
            cur.execute(
                """INSERT INTO csi_events (received_at, node_id, event, someone, moved, wander, jitter)
                   VALUES (?,?,?,?,?,?,?)""",
                (now, node, payload.get("event"), int(payload.get("someone", False)),
                 int(payload.get("moved", False)), payload.get("wander"), payload.get("jitter")),
            )

        self.conn.commit()

    def recent_nodes(self):
        cur = self.conn.execute("SELECT node_id, last_seen, last_ip FROM nodes ORDER BY last_seen DESC")
        return cur.fetchall()

    def counts(self):
        cur = self.conn.execute(
            "SELECT (SELECT COUNT(*) FROM events), (SELECT COUNT(*) FROM sightings), (SELECT COUNT(*) FROM csi_events)"
        )
        return cur.fetchone()


# ---------------------------------------------------------------------------
# Gateway
# ---------------------------------------------------------------------------

class Gateway:
    def __init__(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        self.db = DB(DB_FILE)
        self.subscribers = set()  # WebSocket clients

    def stamp(self, envelope, source_ip):
        """Validate + stamp an envelope. Returns (rec, error)."""
        if not isinstance(envelope, dict):
            return None, "invalid_envelope"
        if envelope.get("v") != 1:
            return None, "invalid_envelope"
        node = envelope.get("node")
        typ = envelope.get("type")
        payload = envelope.get("payload")
        if not node or not isinstance(node, str):
            return None, "invalid_envelope"
        if typ not in VALID_TYPES:
            # forward-compat: accept unknown types, just log
            typ = str(typ) if typ else "unknown"
        if not isinstance(payload, dict):
            return None, "invalid_envelope"
        rec = {
            "v": 1,
            "node": node,
            "type": typ,
            "ts": envelope.get("ts"),
            "payload": payload,
            "server_received_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "source_ip": source_ip,
        }
        return rec, None

    def ingest(self, envelope, source_ip):
        """Validate, log, store, broadcast. Returns (http_status, body)."""
        rec, err = self.stamp(envelope, source_ip)
        if err:
            return 400, {"error": err}
        # Canonical JSONL log (source of truth)
        with open(LOG_FILE, "a") as f:
            f.write(json.dumps(rec) + "\n")
        # SQLite
        try:
            self.db.record(rec)
        except Exception as e:
            print(f"[db] error: {e}")
        # Broadcast to live subscribers
        for ws in list(self.subscribers):
            try:
                ws.send_str(json.dumps(rec))
            except Exception:
                self.subscribers.discard(ws)
        return 200, {"ok": True}

    # -- HTTP handlers ------------------------------------------------------

    async def handle_ingest(self, request):
        try:
            envelope = await request.json()
        except Exception:
            return web.json_response({"error": "bad_json"}, status=400)
        status, body = self.ingest(envelope, request.remote)
        return web.json_response(body, status=status)

    async def handle_status(self, request):
        events, sightings, csi = self.db.counts()
        return web.json_response({
            "status": "ok",
            "protocol": "orb-v1",
            "events": events,
            "sightings": sightings,
            "csi_events": csi,
            "nodes": [dict(zip(("node", "last_seen", "last_ip"), r)) for r in self.db.recent_nodes()],
        })

    async def handle_ws(self, request):
        ws = web.WebSocketResponse(heartbeat=30)
        await ws.prepare(request)
        self.subscribers.add(ws)
        print(f"[ws] client connected ({len(self.subscribers)} total)")
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    # nodes can also POST via WS
                    try:
                        envelope = json.loads(msg.data)
                    except Exception:
                        await ws.send_str(json.dumps({"error": "bad_json"}))
                        continue
                    status, body = self.ingest(envelope, request.remote)
                    await ws.send_str(json.dumps(body))
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            self.subscribers.discard(ws)
            print(f"[ws] client disconnected ({len(self.subscribers)} total)")
        return ws


def main():
    ap = argparse.ArgumentParser(description="ORB Protocol v1 gateway")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = ap.parse_args()

    gw = Gateway()
    app = web.Application()
    app.router.add_post("/orb/ingest", gw.handle_ingest)
    app.router.add_get("/ws", gw.handle_ws)
    app.router.add_get("/status", gw.handle_status)
    app.router.add_get("/", gw.handle_status)

    print(f"ORB gateway v1 on :{args.port} (log={LOG_FILE}, db={DB_FILE})")
    web.run_app(app, host=HOST, port=args.port)


if __name__ == "__main__":
    main()
