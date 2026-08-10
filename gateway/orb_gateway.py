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

# Default world-model tuning (overridden by config.yaml when present)
DEFAULT_WORLD_MODEL = {
    "rssi_floor": -85,
    "stationary_min_seconds": 604800,
    "stationary_max_rssi_variance": 8,
    "visit_absence_gap_seconds": 300,
    "composite_min_consistency": 0.8,
    "composite_presence_threshold": 0.33,
    "reclassify_after_moves": 3,
}


def load_config():
    """Load config.yaml next to this script. Returns (server, world_model, token).
    Falls back to defaults when config.yaml is missing — the gateway must
    always start, even on a fresh clone."""
    cfg = {}
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "config.yaml")
    if os.path.exists(path):
        try:
            import yaml
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        except ImportError:
            print("WARN: PyYAML not installed, using defaults (pip install pyyaml)")
        except Exception as e:
            print(f"WARN: config.yaml unreadable ({e}), using defaults")

    server = {
        "host": cfg.get("host", HOST),
        "port": int(cfg.get("port", DEFAULT_PORT)),
        "data_dir": os.path.expanduser(cfg.get("data_dir", DATA_DIR)),
    }
    server["log_file"] = os.path.join(server["data_dir"], cfg.get("log_file", "presence.jsonl"))
    server["db_file"] = os.path.join(server["data_dir"], cfg.get("db_file", "orb.db"))

    wm = dict(DEFAULT_WORLD_MODEL)
    wm_cfg = cfg.get("world_model", {}) or {}
    wm.update({k: v for k, v in wm_cfg.items() if v is not None})

    token = cfg.get("ingest_token", "") or ""
    return server, wm, token

VALID_TYPES = {"scan", "csi", "imu", "state", "wifi"}

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
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    received_at   TEXT,
    node_id       TEXT,
    mac           TEXT,
    rssi          INTEGER,
    name          TEXT,
    device_class  TEXT,
    company_id    INTEGER,
    service_uuids TEXT
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
-- WiFi places registry: BSSIDs seen in type:"wifi" beacon frames.
-- A BSSID that never moves + stable RSSI = a PLACE (the brain marks it).
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
);
-- WiFi probe-request log: client MACs seeking SSIDs ("identity from the air").
CREATE TABLE IF NOT EXISTS probes (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    received_at TEXT,
    node_id     TEXT,
    client_mac  TEXT,
    ssid        TEXT,
    channel     INTEGER,
    rssi        INTEGER
);
CREATE INDEX IF NOT EXISTS idx_sightings_mac ON sightings(mac);
CREATE INDEX IF NOT EXISTS idx_sightings_time ON sightings(received_at);
CREATE INDEX IF NOT EXISTS idx_events_node ON events(node_id);
CREATE INDEX IF NOT EXISTS idx_places_last_seen ON places(last_seen);
CREATE INDEX IF NOT EXISTS idx_probes_ssid ON probes(ssid);
CREATE INDEX IF NOT EXISTS idx_probes_mac ON probes(client_mac);
"""

# Columns added after the original v1 schema shipped — ALTERed in on existing
# databases so old orb.db files keep working (CREATE TABLE IF NOT EXISTS does
# not add columns to an existing table).
SIGHTINGS_ADDED_COLUMNS = {
    "device_class": "TEXT",
    "company_id": "INTEGER",
    "service_uuids": "TEXT",
}


class DB:
    def __init__(self, path):
        self.conn = sqlite3.connect(path)
        self.conn.executescript(SCHEMA)
        self._migrate()
        self.conn.commit()

    def _migrate(self):
        """Add columns that landed after the first shipped schema (idempotent)."""
        cols = {r[1] for r in self.conn.execute("PRAGMA table_info(sightings)")}
        for name, decl in SIGHTINGS_ADDED_COLUMNS.items():
            if name not in cols:
                self.conn.execute(f"ALTER TABLE sightings ADD COLUMN {name} {decl}")

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
                su = dev.get("service_uuids")
                cur.execute(
                    """INSERT INTO sightings
                       (received_at, node_id, mac, rssi, name, device_class, company_id, service_uuids)
                       VALUES (?,?,?,?,?,?,?,?)""",
                    (now, node, dev.get("mac", "?"), dev.get("rssi"), dev.get("name"),
                     dev.get("device_class"), dev.get("company_id"),
                     json.dumps(su) if su else None),
                )

        # wifi: beacon frames feed the places registry (BSSID → place),
        # probe requests are logged as identity-from-the-air events.
        if typ == "wifi":
            for fr in payload.get("frames", []):
                if not isinstance(fr, dict):
                    continue
                kind = fr.get("kind")
                if kind == "beacon":
                    bssid = (fr.get("bssid") or fr.get("mac") or "").lower()
                    if not bssid:
                        continue
                    rssi = fr.get("rssi")
                    cur.execute(
                        """INSERT INTO places (bssid, ssid, channel, first_seen, last_seen,
                                               seen_count, min_rssi, max_rssi, avg_rssi)
                           VALUES (?,?,?,?,?,1,?,?,?)
                           ON CONFLICT(bssid) DO UPDATE SET
                             ssid=COALESCE(excluded.ssid, places.ssid),
                             channel=COALESCE(excluded.channel, places.channel),
                             first_seen=MIN(places.first_seen, excluded.first_seen),
                             last_seen=MAX(places.last_seen, excluded.last_seen),
                             seen_count=places.seen_count+1,
                             min_rssi=CASE WHEN excluded.min_rssi IS NULL THEN places.min_rssi
                                           ELSE MIN(places.min_rssi, excluded.min_rssi) END,
                             max_rssi=CASE WHEN excluded.max_rssi IS NULL THEN places.max_rssi
                                           ELSE MAX(places.max_rssi, excluded.max_rssi) END,
                             avg_rssi=CASE WHEN excluded.avg_rssi IS NULL THEN places.avg_rssi
                                           ELSE (places.avg_rssi*places.seen_count + excluded.avg_rssi)
                                                / (places.seen_count+1) END""",
                        (bssid, fr.get("ssid"), fr.get("channel"),
                         now, now, rssi, rssi, rssi),
                    )
                elif kind == "probe_req":
                    cur.execute(
                        """INSERT INTO probes (received_at, node_id, client_mac, ssid, channel, rssi)
                           VALUES (?,?,?,?,?,?)""",
                        (now, node, (fr.get("mac") or "?").lower(), fr.get("ssid"),
                         fr.get("channel"), fr.get("rssi")),
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
    def __init__(self, server=None, world_model=None, ingest_token=""):
        server = server or {
            "host": HOST,
            "port": DEFAULT_PORT,
            "data_dir": DATA_DIR,
            "log_file": LOG_FILE,
            "db_file": DB_FILE,
        }
        self.server = server
        self.world_model = world_model or dict(DEFAULT_WORLD_MODEL)
        self.ingest_token = ingest_token
        os.makedirs(server["data_dir"], exist_ok=True)
        self.log_file = server["log_file"]
        self.db = DB(server["db_file"])
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
        with open(self.log_file, "a") as f:
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
    ap.add_argument("--port", type=int, default=None, help="override config.yaml port")
    args = ap.parse_args()

    server, world_model, token = load_config()
    if args.port:
        server["port"] = args.port

    gw = Gateway(server)
    gw.world_model = world_model
    gw.ingest_token = token
    app = web.Application()
    app.router.add_post("/orb/ingest", gw.handle_ingest)
    app.router.add_get("/ws", gw.handle_ws)
    app.router.add_get("/status", gw.handle_status)
    app.router.add_get("/", gw.handle_status)

    print(f"ORB gateway v1 on :{server['port']} (log={server['log_file']}, db={server['db_file']})")
    print(f"world_model: {json.dumps(world_model)}")
    web.run_app(app, host=server["host"], port=server["port"])


if __name__ == "__main__":
    main()
