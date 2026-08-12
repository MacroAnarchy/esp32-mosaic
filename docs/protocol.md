# ORB Protocol v1

**The wire protocol for environment-aware AI companions.**
Status: DRAFT (Aug 9, 2026)
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
| `node` | string | yes | Stable node identifier: `node-01`, `sentinel-3`, `swarm-03`. Lowercase, no spaces. |
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
      {
        "mac": "24:5f:9f:db:9f:72",
        "rssi": -62,
        "name": "HUAWEI WATCH FIT 3-F72",
        "device_class": "unknown",
        "company_id": 76,
        "service_uuids": ["fd44"],
        "service_data_uuid": "fd5f"
      }
    ]
  }
}
```
- `ap_bssid` = BSSID of the AP the node is currently associated with (location fingerprint). Optional.
- `devices[].mac` = MAC as seen (may be randomized — server dedups by its own logic).
- `devices[].rssi` = signal strength in dBm.
- `devices[].name` = BLE advertised name, if any.
- `devices[].device_class` = passive classification of the advertiser (see below). *Added v1.1 (BLE port).*
- `devices[].company_id` = Bluetooth SIG company/manufacturer ID from the advertisement's manufacturer data, little-endian uint16. Present only when manufacturer data exists. *Added v1.1.*
- `devices[].service_uuids` = array of 16-bit service UUID strings the device advertises (e.g. `["fd44"]`). Omitted when the device advertises none. *Added v1.1.*
- `devices[].service_data_uuid` = 16-bit service data UUID, when the device broadcasts service data (e.g. `"fd5f"`). *Added v1.1.*

#### `device_class` values (v1.1)
| Value | Meaning | Detection |
|---|---|---|
| `findmy` | Apple FindMy network device (AirTag, AirPods, third-party FindMy tags) | Advertises FMNA service `0000fd44-...` or FMDN service `7dfc9001-...`, or carries an Apple offline-finding payload signature (`0x1E 0xFF 0x4C 0x00` / `0x4C 0x00 0x12`) |
| `meta` | Meta Quest / Ray-Ban (Luxottica) headset or controller | Company ID ∈ {0xFD5F, 0xFEB7, 0xFEB8, 0x01AB, 0x058E, 0x0D53} |
| `flipper` | Flipper Zero | Company ID `0x0FBA` (note: a Flipper impersonating a FindMy tag is reported as `findmy`) |
| `unknown` | Everything else, including high-volume consumer IDs (Apple `0x004C`, Samsung `0xFD5A`/`0xFD69`, Microsoft `0x0006`, phone `0xFEF3`) | — |

All classification is passive (advertisement parsing only): no deauth, no BLE spam, no replay, no jamming. New fields are additive — v1 clients and the gateway ignore fields they don't know.

### `wifi` — passive WiFi beacon/probe batch (offline scan cycle)
The node has ONE radio: while associated to WiFi it is blind to the rest of the
spectrum. Every `wifi_scan_interval_seconds` (default 300 = every 5 min) the node
briefly drops its WiFi association ("the mosaic goes offline to see"), enters
promiscuous mode, hops channels 1..13 (~80ms each, ~1s total), captures 802.11
beacons (0x80) and probe requests (0x40), then reconnects and reports everything
captured in one batch envelope. PASSIVE LISTEN ONLY — no deauth, no injection,
no beacon spam. *Added v1.2 (WiFi scan port).*

```json
{
  "type": "wifi",
  "payload": {
    "frames": [
      {
        "kind": "beacon",
        "mac": "aa:bb:cc:dd:ee:ff",
        "bssid": "aa:bb:cc:dd:ee:ff",
        "ssid": "HomeWiFi",
        "channel": 6,
        "rssi": -58
      },
      {
        "kind": "probe_req",
        "mac": "11:22:33:44:55:66",
        "ssid": "CafeFreeWifi",
        "channel": 1,
        "rssi": -70
      }
    ]
  }
}
```
- `frames[]` = all frames captured during the ~1s sweep, deduped by kind+MAC+SSID within the cycle.
- `frames[].kind` = `beacon` (AP broadcasting) or `probe_req` (client seeking).
- `frames[].mac` = source MAC: the AP's MAC for beacons, the probing client's MAC for probe requests.
- `frames[].bssid` = AP BSSID (beacons only; same as `mac` since BSSID == src for beacons). Omitted for probe requests.
- `frames[].ssid` = broadcast ESSID (beacon) or the SSID the client is requesting (probe). Omitted when empty (hidden network / broadcast probe).
- `frames[].channel` = 802.11 channel (1..13) the frame was heard on.
- `frames[].rssi` = received signal strength in dBm.
- After the sweep the node MUST rejoin WiFi before reporting; if the join fails it keeps retrying on subsequent cycles while BLE sensing continues (a node that can't rejoin is dead).

### `arp_join` / `arp_leave` — WiFi neighbor join/leave events (ARP)
The node is an ordinary WiFi client, so it participates in ARP like any
other host on the subnet: every station learns its neighbors by asking
"who has IP X?". Every `arp_interval_seconds` (default 60) the node
ARP-probes its own subnet (range derived from its live IP + netmask),
keeps a seen-table of MAC→IP pairs with first/last-seen, and reports
transitions:

- a MAC that appears → `arp_join`
- a MAC unseen for `arp_leave_timeout_intervals` consecutive cycles
  (default 3 × 60s ≈ 3 min) → `arp_leave`

This is standard, legal network membership traffic — no injection, no
promiscuous mode, nothing beyond what every station does. On APs with
client isolation the probes simply go unanswered: the node sees no
neighbors, emits no events, and stays silent. *Added v1.3 (ARP neighbor discovery).*

```json
{
  "type": "arp_join",
  "payload": {
    "ap_bssid": "aa:bb:cc:dd:ee:ff",
    "mac": "11:22:33:44:55:66",
    "ip": "192.168.1.42",
    "first_seen": 1754240000000,
    "last_seen": 1754240000000
  }
}
```

```json
{
  "type": "arp_leave",
  "payload": {
    "ap_bssid": "aa:bb:cc:dd:ee:ff",
    "mac": "11:22:33:44:55:66",
    "ip": "192.168.1.42",
    "first_seen": 1754239000000,
    "last_seen": 1754240000000
  }
}
```
- `ap_bssid` = BSSID of the AP the node is currently associated with (location fingerprint, same as `scan`).
- `mac` = the neighbor's MAC as learned from its ARP reply. Unlike BLE MACs this is the real, stable device MAC (the node's ARP table only holds members of its own subnet).
- `ip` = the neighbor's IPv4 address on the node's subnet.
- `first_seen` / `last_seen` = node-clock epoch ms of the first / most recent sighting (best effort, server stamps the real time).
- A MAC that changes IP (DHCP renumber) is re-reported under the same identity — the node keeps `first_seen`, only `ip`/`last_seen` update.
- Join events are intended for server-side entity confirmation: a MAC with a live IP on the node's subnet is stronger evidence of presence than a radio sighting alone (e.g. it can split two co-located devices that look identical to BLE).

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
