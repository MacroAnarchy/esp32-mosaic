# ORB Protocol v1

**The wire protocol for environment-aware AI companions.**
Status: DRAFT (Aug 9, 2026 — the authors)
License: open (intended to be shared when Orb goes open source)

## Design Principles

1. **Thin sensors, fat brain.** Nodes (Orb, swarm) are dumb eyes/ears. All intelligence lives server-side. The protocol reflects this: nodes send compact structured events, never analysis.
2. **One envelope, many payloads.** Every message uses the same v1 envelope. Only the `type` changes. New node types plug in without protocol changes.
3. **Server time is truth.** Node clocks drift. The gateway stamps `server_received_at` on every message; clients must never trust node timestamps for ordering.
4. **Location is learned, not claimed.** Nodes report the BSSID they can see; the server maps BSSID → location label. The sensor doesn't decide where it is.
5. **Lossy is fine.** Events are ephemeral presence snapshots. If one HTTP POST is lost, the next scan is 30s away. No delivery guarantees, no acks, no retry storms.
6. **Backward compatible forever.** v1 fields never change meaning. New fields get added, never repurposed. Old nodes keep working against new servers.

## Transport

- **WebSocket** (primary for interactive nodes: Orb, CSI-heavy nodes) — persistent, bidirectional, low overhead. Path: `ws://host:9000/ws`
- **HTTP POST** (primary for deep-sleep swarm nodes) — stateless, wake-scan-sleep. Path: `POST http://host:9000/orb/ingest`
- Both carry the same envelope JSON. A server MUST accept both. A node MAY support only one.

## The Envelope (v1)

```json
{
  "v": 1,
  "node": "orb",
  "type": "scan",
  "ts": 1754240000000,
  "payload": { }
}
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `v` | int | yes | Protocol version. Always 1 for this spec. |
| `node` | string | yes | Stable node identifier: `orb`, `node-01`, `swarm-03`. Lowercase, no spaces. |
| `type` | string | yes | Payload type (see below). |
| `ts` | int64 | yes | Node clock, epoch milliseconds. Best effort only. |
| `payload` | object | yes | Type-specific body (see below). |

The gateway ADDS on receipt (server-side, not sent by node):

```json
{
  "server_received_at": "2026-08-09T18:30:00",
  "source_ip": "192.168.0.168"
}
```

## Payload Types

### `scan` — BLE presence snapshot
```json
{
  "type": "scan",
  "payload": {
    "ap_bssid": "aa:bb:cc:dd:ee:ff",
    "devices": [
      {"mac": "24:5f:9f:db:9f:72", "rssi": -62, "name": "HUAWEI WATCH FIT 3-F72"}
    ]
  }
}
```
- `ap_bssid` = BSSID of the AP the node is currently associated with (location fingerprint). Optional.
- `devices[].mac` = MAC as seen (may be randomized — server dedups by its own logic).
- `devices[].rssi` = signal strength in dBm.
- `devices[].name` = BLE advertised name, if any.

### `csi` — radar/presence event from CSI processing
```json
{
  "type": "csi",
  "payload": {
    "event": "moved",
    "someone": true,
    "moved": true,
    "wander": 0.0055,
    "jitter": 0.5706
  }
}
```
- `event` ∈ `empty | present | moving | moved`
- `someone` = presence flag (someone in room, through walls)
- `moved` = movement flag
- `wander`/`jitter` = waveform metrics (implementation detail, kept for debugging)

### `imu` — motion/context from the node's IMU
```json
{
  "type": "imu",
  "payload": {
    "steps": 4521,
    "activity": "walking",
    "heading": 42.5,
    "carried": true
  }
}
```
- `steps` = hardware step counter (QMI8658)
- `activity` ∈ `still | walking | running | shaken`
- `heading` = magnetometer degrees
- `carried` = inferred carried-vs-placed state

### `state` — node health/config
```json
{
  "type": "state",
  "payload": {
    "battery_pct": 87,
    "firmware": "orb-v0.3.0",
    "model": "waveshare-1.75c"
  }
}
```
Sent periodically (heartbeat) and on config change.

## Server Behavior

1. Validate envelope (v=1, node present, type known, payload object).
2. Stamp `server_received_at` + `source_ip`.
3. Append raw envelope to the canonical JSONL log (source of truth).
4. Upsert into SQLite query layer (nodes, sightings, events).
5. MAY forward to subscribers (Hermes, dashboards) over the same WebSocket.
6. Unknown types: accept + log (forward-compat), do not reject.

## Error Handling

- Malformed JSON → `400 {"error": "bad_json"}`
- Missing required envelope field → `400 {"error": "invalid_envelope"}`
- Everything else → `200 {"ok": true}`
- Nodes should NOT retry on failure. Next scheduled scan is the retry.

## Naming

- Protocol: **ORB Protocol** (Open Radio/Body protocol — fits both meanings)
- File: `orb-protocol-v1.json` (machine-readable schema, TBD) + this doc
