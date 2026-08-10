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

## What's inside

```
esp32-mosaic/
├── firmware/        Sense engine (PlatformIO/Arduino)
│   └── src/main.cpp BLE scan → HTTP POST to gateway
├── gateway/         Python ingest layer (aiohttp)
│   └── orb_gateway.py  HTTP + WebSocket, JSONL + SQLite
└── docs/
    └── protocol.md  The envelope protocol (v1)
```

## Architecture

**Thin sensors, fat brain.** Nodes are dumb eyes/ears — they scan and report.
All intelligence lives server-side. The protocol is one envelope, many payloads.

```
┌──────────┐   HTTP/WS   ┌──────────────┐
│ ESP32    │ ──────────► │ Gateway      │──► JSONL (canonical)
│ node     │  envelope   │ (your brain) │──► SQLite (query)
└──────────┘             └──────────────┘
   BLE scan
   WiFi probes
   (CSI on capable chips)
```

- **Transport:** WebSocket (interactive) + HTTP POST (deep-sleep nodes)
- **Storage:** JSONL canonical + SQLite query layer
- **Location:** learned, not claimed — the server maps BSSID → label
- **Identity:** from MACs heard in the air, not connection state

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

The node scans BLE every ~15s and POSTs the device list to the gateway.
Watch it learn your apartment:

```bash
tail -f ~/.orb/presence.jsonl
```

## Status

- **v0.1** — BLE presence sensing proven (25-31 devices seen from one room)
- **Next:** WiFi beacon/probe scanning (location fingerprints), entity clustering,
  CSI on capable boards, ESP-NOW mesh for swarm nodes

## License

MIT — go build things.
