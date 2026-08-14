# Example 1 — Home Security / Intruder Detection

**A passive intrusion alarm built on the mosaic stack.** No cameras, no
microphones, no cloud. The room watches its own RF baseline and reports the
moment something *new* happens.

## The sensing

| Signal | What it catches |
|--------|-----------------|
| BLE presence | Unknown device appearing in Z1 at -30 dBm at 3 am |
| CSI field | Deviation in the CSI field of an empty room (a body changed the reflections) |
| Entity slots | A known entity's device reappearing when it shouldn't (left-phone-at-home + phone-arrives = impossible → anomaly) |
| ARP feed | A new MAC joining your network that isn't in the entity registry |

## The math

- The brain learns the apartment's **RF baseline per hour of day**: what devices
  are *structural* (the neighbor's router through two walls — always there,
  Z3) versus *transient* (someone passing by).
- An **anomaly** is a deviation from that baseline that survives the
  confidence filter: unknown device at strong RSSI outside known hours, or a
  CSI field change in a room the brain thinks is empty.
- **Explainable** — every alert carries the evidence chain: *how* it deviated,
  *from what* baseline, *when* relative to the learned pattern. No black box.

## Why it beats a camera

- **Privacy-positive** — it senses presence, not footage. Nothing is recorded
  but radio statistics.
- **Cheap** — one ESP32-class board. No subscription, no video cloud.
- **Passive** — the radar listens; it doesn't shout into the room.
- **The remembering advantage** — a camera shows you the door; Mosaic knows
  the *whole room's* radio signature and notices when it lies.

## Build status

1. ✅ BLE presence + device classification + entity slots
2. ✅ CSI presence + motion (Phase 1)
3. 🔄 CSI wander calibration (quiet-window + adaptive threshold)
4. ⬜ Baseline-per-hour anomaly engine
5. ⬜ Alerting (MCP tool, push, local)

> This is the *technical* capability. The **pet wellness station** example
> (see `pet-wellness-station.md`) builds directly on this — the same anomaly
> engine watching a pet's baseline instead of an intruder's.
