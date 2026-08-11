# ESP32-Mosaic

**Your ESP32 already has the radios. Here's how to make it aware.**

ESP32-Mosaic turns any ESP32 board into a passive radio observer — it senses the
devices around it (BLE), the WiFi landscape (beacons + probes), and builds a
persistent picture of its environment. Fragments of radio evidence become a
coherent world model: who's here, what's nearby, what stays and what moves.

Different boards working together form a **mosaic** — each one a tile, together
a picture of your world.

## Why

Every ESP32 scanner shows you devices and forgets them. Mosaic **remembers**:
it learns the radio fingerprint of places, clusters devices into entities,
and derives meaning from patterns over time. The remembering radar.

The model is passive, local and private: no cameras, no microphones, no cloud.
It listens to the radio that's already around you and builds understanding
from it — then exposes that understanding to whatever you want to use it for.

## Use cases

- **Presence-gated AI** — your assistant only listens to you, because it knows
  who you are by radio (entity cluster + proximity + body verification).
- **Home security** — the radar knows the RF baseline of your home. An unknown
  device appearing at -30 dBm at 3am, or a CSI field deviation in an empty room,
  is an event your home can report itself. No cameras, no microphones, no cloud.
- **Surveillance / tracking** — watch what enters and leaves a space, learn
  habitual paths (bedroom → kitchen → door every morning), know when a person
  returns by their device constellation. The remembering radar — it sees what
  other scanners forget.
- **Smart-home automation** — "you're home" triggers scenes from RF presence,
  not phone GPS or app state. The room knows when you're actually in it.

## What's inside

```
esp32-mosaic/
├── firmware/        Sense engine (PlatformIO/Arduino)
│   ├── src/main.cpp BLE + WiFi passive scanning → HTTP POST to gateway
│   └── linux_node.py  Same protocol from any Linux/Android box (monitor mode)
├── gateway/         Python brain (aiohttp)
│   ├── orb_gateway.py    HTTP + WebSocket ingest, JSONL + SQLite
│   ├── mosaic_brain.py   world model: entities, chains, places, movement
│   └── mosaic_mcp.py     MCP server — give your AI physical senses
└── docs/
    └── protocol.md  The envelope protocol (v1)
```

## Architecture

**Thin sensors, fat brain.** Nodes are dumb eyes/ears — they scan and report.
All intelligence lives server-side. The protocol is one envelope, many payloads.

```
┌──────────┐   HTTP/WS   ┌──────────────┐   MCP   ┌─────────────┐
│ ESP32    │ ──────────► │ Gateway      │ ◄─────► │ MCP client  │
│ node     │  envelope   │ (the brain)  │  tools  │ (Hermes,    │
└──────────┘             │              │         │  Claude, …) │
   BLE scan              ├── JSONL (canonical)     └─────────────┘
   WiFi probes           ├── SQLite (query)
   (CSI on S3)           └── world model
```

- **Transport:** WebSocket (interactive) + HTTP POST (deep-sleep nodes)
- **Storage:** JSONL canonical + SQLite query layer
- **Location:** learned, not claimed — the server maps BSSID → label
- **Identity:** from MACs heard in the air, not connection state
- **The brain thinks, the node just listens** — nodes never classify, they report raw

## The world model

### Zones — the signal tiers

The brain separates the world by signal strength. Each zone answers a different
question:

```
Z1  apartment   -30..-70 dBm   STRONG — full resolution: entities, slots, places
Z2  hallway     -70..-85 dBm   MID    — temporal patterns, no fine position
Z3  deep bleed  -85..-105 dBm  EDGE   — counted, not resolved (neighbor/street)
```

Z3 devices that are *always* there are structural — part of the building's RF
skeleton (a neighbor's router through two walls). Transient Z3 = someone passing
by. Learning the difference is how the radar maps the building.

### Entities — MACs rotate, people persist

Phones randomize their MAC addresses. A naive scanner sees a new "device" every
few hours. Mosaic binds rotating MACs into **entities**:

```
ENTITY: "the owner's iPhone"   (stable, labeled once)
├── slot: WiFi MAC    aa:bb:cc:dd:ee:ff   (stable anchor)
└── slot: BLE MAC    11:22:33:44:55:66     (rotating occupant)
                     → next MAC → next MAC → (slot holds history)
```

Binding evidence: RSSI continuity at handoff (a rotation preserves position),
device class coherence (same class at same level), and co-occurrence with known
anchors. The entity persists; the MACs are just occupants.

## MCP server — give your AI physical senses

The world model is exposed as an MCP (Model Context Protocol) server, so any
MCP-capable AI agent can *feel* the room:

```
mosaic_presence    → who's home right now (entities, confidence)
mosaic_entities    → entity chains with MAC counts + classification
mosaic_devices     → device registry (labeled + known)
mosaic_where       → resolve a label/MAC → current status + location
mosaic_raw_query   → ad-hoc SQL on the world model
```

Wire it into any MCP client (Claude, Hermes, Cursor, …):

```yaml
mcp_servers:
  mosaic:
    command: python3
    args: ["/path/to/esp32-mosaic/gateway/mosaic_mcp.py"]
```

Now your agent knows when you're home, what moved, and who's near — without a
camera, a microphone, or a cloud API. The AI gets eyes made of radio.

## Protocol (v1 — the envelope)

One envelope, many payloads. Nodes never speak anything else.

```json
{
  "v": 1,
  "node": "node-01",
  "type": "scan",
  "ts": 1754316000,
  "payload": {
    "ap_bssid": "a8:f5:dd:ca:ac:3c",
    "devices": [
      {
        "mac": "24:5f:9f:db:9f:72",
        "rssi": -62,
        "name": "HUAWEI WATCH FIT 3",
        "device_class": "unknown",
        "company_id": 258
      }
    ]
  }
}
```

Types: `scan` (BLE devices), `wifi` (beacon/probe frames, batched), `csi`
(channel state info on capable chips), `imu`, `state`.

Full field reference: [`docs/protocol.md`](docs/protocol.md)

## Quick start

### 1. Gateway (on any always-on machine)

```bash
cd gateway
cp config.example.yaml config.yaml   # edit port/data_dir if needed
pip install aiohttp
python3 orb_gateway.py
```

### 2. Firmware (on an ESP32)

```bash
cd firmware
cp include/config.example.h include/config.h   # fill in WiFi + gateway IP + node name
pio run -t upload
```

> The build refuses to flash while `config.h` still has placeholder values —
> a node that can't join WiFi is a dead node.

The node scans BLE every ~15s and POSTs the device list to the gateway.
Watch it learn your apartment:

```bash
tail -f ~/.orb/presence.jsonl
```

### 3. Ask your AI what it sees

```bash
cd gateway
pip install mcp
python3 mosaic_mcp.py    # or wire into your MCP client
```

## Status

- **v0.2** — BLE presence + device classification (findmy/meta/flipper),
  WiFi beacon/probe capture (offline scan cycle), world-model brain
  (entities, chains, places), MCP server
- **Next:** CSI sensing (body presence/vitals on S3), network-layer identity
  feed (ARP), entity persistence across runs, multi-node swarm

## License

MIT — go build things.
