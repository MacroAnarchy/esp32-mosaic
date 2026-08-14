# Example 2 — Pet Wellness Station

**The natural peak of the mosaic stack.** A stationary radio puck with a round
screen + phone app. It watches your cat or dog through radio alone — no collar,
no camera, no wearables — and tells you *normal / stressed / something's off*,
based on the pet's own learned baseline.

This example **combines everything the stack does**: presence (who's home),
movement (what's moving), CSI (a body's reflections), and the anomaly engine

(when the pattern breaks). It is the flagship demonstration that all the
technical pieces compose into a real product.

## The product

Plug it in, pair it, leave it. The puck learns the pet's personal behavior
baseline — nap spots + hours, activity rhythm, food-bowl circuits — and flags
deviations: stress signals after the owner leaves (pacing spike), drift over
long absences, reunion reactions, and early signs of illness (hiding, reduced
movement) **days before visible symptoms**.

## Why this works technically

- **CSI + BLE presence** detect a pet-sized body (4+ kg) moving through the
  room — presence, movement intensity, and location-over-time are all
  measurable. CSI is the *sensing* layer; BLE is the *identity* layer.
- The **pattern layer** (per-hour-of-day activity, 7-day decay) learns the
  pet's personal rhythm: nap spots + hours, activity windows, food-bowl
  circuits.
- The **anomaly engine** flags deviations from that baseline — the same
  engine as the home-security example, pointed at a pet instead of an
  intruder.

## What it measures (all from existing data)

| Signal | Meaning |
|--------|---------|
| Departure spike | movement burst right after owner leaves (pacing = separation response) |
| Absence drift | activity/appetite rhythm shifts over days 1-3 of an absence |
| Reunion reaction | movement burst correlated with owner's device re-entering range |
| Rhythm changes | nap spots, activity windows, night behavior vs baseline |
| Lethargy trend | reduced movement amplitude over days — early illness signal |

## Key properties

- **No AI magic** — explainable pattern math on real radio data
- **No wearables** — the pet wears nothing
- **No camera** — privacy-positive; radio presence, no footage
- **Simple hardware** — stationary: no battery, no case engineering, no mobile
  ergonomics. One ESP32-class board + round AMOLED + gateway
- **Market position** — adjacent to Furbo/Petcube camera feeders but delivers
  health/behavior insight they can't, at lower BOM (no camera, no video cloud)

## Build roadmap

1. ✅ CSI presence + motion (Phase 1)
2. ✅ Breathing band + pattern-layer seed (Phase 2)
3. 🔄 Wander calibration (quiet-window + adaptive threshold)
4. ⬜ Pet baseline profiles (per-pet, place-memories per BSSID)
5. ⬜ Anomaly flags: departure spike / absence drift / reunion reaction
6. ⬜ Phone app: stats, daily report, "what changed" view
