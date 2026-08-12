# Spec: Mosaic UI Node — "the face of the swarm"

**Status:** draft (Aug 12, 2026) · **Module:** `firmware/ui/` · **License:** MIT

## What

A Mosaic node with a display. The same passive-sensing firmware as every
other tile, plus an optional **face layer**: a 466×466 QSPI AMOLED
(CO5300) driven by a direct-framebuffer glow engine. The node's radio
observations and brain state become light.

```
ESP32 tile (senses radio)  ──►  brain (gateway)  ──►  face (this node's display)
```

## Why

- Most sensing nodes are invisible — the swarm's picture lives only in
  the gateway. A UI node makes the network's "state of mind" physical.
- The face is a **demo surface** for the project: presence → light.
- The display layer is strictly optional at build time (compile flag),
  so the same firmware builds for bare tiles and for UI tiles.

## Build

The UI layer is in `firmware/ui/`; the sense engine is the ESP-IDF
component `firmware/components/sense/` (ported from the Arduino sense
engine, same envelope protocol):

```
firmware/
├── platformio.ini            [env:ui] = ESP-IDF unified build
├── CMakeLists.txt            project root (registers ui/components)
├── components/sense/         BLE sense engine (native NimBLE) + ARP
├── ui/
│   ├── CMakeLists.txt        app component (face + main)
│   ├── display_face.h/.cpp   face API + CO5300 panel wiring (QSPI)
│   ├── fx/fx_glow.h/.cpp     framebuffer glow engine (additive stamps)
│   └── components/
│       └── esp_lcd_co5300/   panel driver (vendored, MIT)
```

- `pio run -e ui` builds ONE firmware: BLE sensing + WiFi reporting +
  the face + OTA partition table. `pio run -e esp32` still builds the
  Arduino bare-tile firmware unchanged.
- PlatformIO 6.1.x does not apply per-env `src_dir` to the ESP-IDF
  builder, so `[env:ui]` uses `scripts/espidf_env.py` (pre-script) to
  point the app component at `ui/`.
- Panel wiring (verified on hardware Aug 12):
  - QSPI bus SPI2_HOST: PCLK=38, DATA0..3 = 4,5,6,7
  - Panel CS=12, RST=3, RGB565, 466×466, gap x=6
  - Flush = `esp_lcd_panel_draw_bitmap`, 64-row banded (DMA-safe)
- Init sequence = official CO5300 command list (brightness 0x51=0xFF,
  sleep-out 0x11, display-on 0x29, RGB565 0x3A, QSPI mode flag).

## Face states

| State          | Meaning                          | Visual                         |
|----------------|----------------------------------|--------------------------------|
| FACE_IDLE      | alone, calm                      | slow cool drift                |
| FACE_OWNER_NEAR| owner nearby                     | warm amber swarm               |
| FACE_VOICE     | listening/talking                | snaps to mic energy            |
| FACE_ALERT     | agitated/fast                    | hot, erratic                   |
| FACE_SLEEP     | dim, barely breathing            | slow faint pulse               |

## Integration

- The brain (gateway) sends `face_state` commands over WS — the same
  envelope channel every node already uses.
- The node maps its own observations to states locally (e.g. an
  owner-class device in range → OWNER_NEAR) and obeys brain commands
  for the rest.
- Envelope protocol: see `docs/protocol.md` (v1.x, WS command path).

## Roadmap

1. [x] Face layer ported to `firmware/ui/` (scrubbed, neutral)
2. [x] Unified firmware: sense engine (native ESP-IDF) + face in one build
   (`pio run -e ui`)
3. [ ] Wire `face_state` WS command handling into the node firmware
4. [ ] Local observation → state mapping (owner in range, etc.) — the
   integration seam is `sense_engine_get_device_count()` / scan results
5. [x] CI build for the UI env (env builds clean; CI wiring TBD)
