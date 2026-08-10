# Mosaic Brain: Consume device_class + wifi data

**Goal:** Make the brain USE the new data the firmware ports produce.
Currently the brain binds entities via RSSI continuity alone. New signals:

1. `device_class` from BLE port (`findmy|meta|flipper|unknown` + company_id + service_uuids)
2. `type:"wifi"` envelopes from WiFi scan (beacons + probe requests)

## 1. device_class as an identity signal

**Current binding:** rotating BLE MACs bound by RSSI continuity (|ΔRSSI| < 6dB at
handoff) + STRONG tier + level coherence. Works, but pure signal-space.

**Add: class coherence as a binding dimension.**
- Two MACs with the SAME device_class at the SAME RSSI level = same entity, much
  stronger confidence. (A FindMy device that rotates MACs keeps the same class.)
- company_id + service_uuids fingerprint can DISAMBIGUATE: two entities at the same
  RSSI level but different classes (phone vs watch vs airtag) are NOT the same.
- Implementation: add `class_match` and `company_match` weight to the rotation
  binding score in mosaic_brain.py. E.g.:
  ```
  weight = decay * continuity * (1.5 if class_match else 1.0)
  ```
  (Tune the multiplier — data first, don't guess.)

## 2. FindMy/AirTag tracking (new device class)

- AirTags/FindMy accessories broadcast CONSTANTLY with recognizable signature.
- Brain should: label `device_class:"findmy"` entities distinctly, track them like
  any entity (slots, chains, movement) — they're perfect anchors because they
  rotate slowly and stay put (a stationary AirTag = a PLACE candidate).
- New entity label namespace: "airtag-*", "findmy-*" — visible in `--status`.

## 3. WiFi data → places (the BSSID registry)

**Key insight from project doc:** "Location is learned, not claimed — the brain
maps BSSID → label." Now we get actual BSSIDs from beacons.

- Build a `places` table from `type:"wifi"` beacon envelopes:
  - bssid → first seen, last seen, ssid, channel, rssi profile, seen count
  - A BSSID that never moves + stable RSSI = a PLACE (entity that never moves)
- Probe requests (`kind:"probe_req"`) = clients seeking known SSIDs:
  - Probe for "HomeWiFi" from an unknown MAC = a device that KNOWS our network
    (returning entity — e.g. phone with WiFi off was nearby, probed, left)
  - This is the "identity from the air" layer: probes reveal device relationships
    to networks even without connection.
- New `mosaic_where "home"` semantics: place entities resolve from BSSID registry.

## 4. Envelope acceptance

Verify gateway accepts new fields (`device_class`, `type:"wifi"`):
- orb_gateway.py: check `type` handling — "wifi" needs to route to the same
  sightings pipeline (or a parallel one) WITHOUT breaking existing scan handling.
- mosaic_brain.py queries: sightings table needs the new columns IF we want to
  query by class. (Check if gateway stores raw payload JSONL only vs SQLite
  columns. If SQLite is column-based, add device_class column.)

## Files to modify (brain-side, gateway-side)
- `gateway/orb_gateway.py` — accept `type:"wifi"`, store device_class
  ⚠ LIVE on port 9000 — modify carefully, restart only when safe
- `gateway/mosaic_brain.py` — class coherence weight, findmy labels, places table
- `gateway/config.example.yaml` — new params (class match multiplier, place thresholds)

## Constraints
- The self-tuner cron (job 69e1af65907f) may touch mosaic_brain.py every 2h —
  coordinate: big structural changes should be committed BEFORE the tuner runs,
  or the tuner may conflict. (Check mosaic-notes.md before editing.)
- Do NOT break the running gateway. If orb_gateway.py changes need a restart,
  stage them and restart during a quiet window.
- Keep everything additive — old envelopes still work.

## Definition of done
- Brain shows findmy/airtag entities in --status
- Places table populated from beacon data
- Binding weight includes class coherence (tunable)
- Gateway accepts type:"wifi" without error
- Docs updated, committed
