# ESP32-Mosaic BLE Capability Port

**Goal:** Port the valuable BLE sniffing capabilities from ESP32Marauder into the
ESP32-Mosaic firmware, WITHOUT the attack/spam modes. Passive + legal only.

**Source reference:** `/home/owner/workspace/ESP32Marauder/esp32_marauder/WiFiScan.cpp` + `WiFiScan.h`
**Target:** `/home/owner/workspace/esp32-mosaic/firmware/src/main.cpp` (current sense engine, ESP32 classic, PlatformIO)

## Current state (main.cpp, 103 lines)
- BLEDevice + BLEScan (Arduino BLE lib)
- Active scan, 5s window, dumps MAC + RSSI + name
- No dedup strategy, no AD parsing, no device classification

## What to port (all passive — legal in Germany, no deauth/replay/spam)

### 1. NimBLE swap
Replace `BLEDevice` with `NimBLE-Arduino` (already vendored in Marauder's
`esp32_marauder/libraries/NimBLE-Arduino` — copy it into `firmware/lib/` or add
as PlatformIO dependency `nimble-arduino`).
- `NimBLEDevice::setScanFilterMode(CONFIG_BTDM_SCAN_DUPL_TYPE_DEVICE)`
- `NimBLEDevice::setScanDuplicateCacheSize(200)`
- Callback: `class bluetoothScanAllCallback : public NimBLEScanCallbacks { onResult(...) }`

### 2. AD payload parsing (the gold)
In the callback, extract and emit:
- `getManufacturerData()` → company ID (little-endian uint16 from bytes 0-1)
- `getServiceUUID()` list → 16-bit UUIDs
- `getServiceDataUUID()` → 16-bit identifier
- device address, RSSI, name (existing)

### 3. Device classification (identity anchors for the brain)
- `META_IDENTIFIERS[6]` = {0xFD5F, 0xFEB7, 0xFEB8, 0x01AB, 0x058E, 0x0D53}
  (Meta Quest + Luxottica) → "meta_device"
- `BLOCKED_IDENTIFIERS[5]` = {0xFD5A, 0xFD69, 0x004C, 0x0006, 0xFEF3}
  (Samsung, Apple, Microsoft, phone) → "blocked" (exclude from meta, KEEP for tracking)
- Emit a `device_class` field in the envelope payload so the brain can use it.

### 4. FindMy / AirTag detection (new device class we currently miss)
- `FMNA_SERVICE_UUID = "0000fd44-0000-1000-8000-00805f9b34fb"`
- `FMDN_SERVICE_UUID = "7dfc9001-7d1c-4951-86aa-8d9728f8d66c"`
- payload signature: bytes `0x1E 0xFF 0x4C 0x00` (Apple) or `0x4C 0x00 0x12`
- Emit `device_class: "findmy"` + flag in envelope.

### 5. Flipper / meta detection
- `isAdvertisingService(FMNA_SERVICE_UUID)` + identifier table match → "flipper"/"meta"

## Envelope format (existing protocol — do NOT change)
```json
{"v":1,"node":"orb","type":"scan","ts":<epoch>,"payload":{
  "mac":"aa:bb:cc:dd:ee:ff","rssi":-62,"name":"...",
  "device_class":"findmy|meta|flipper|unknown",
  "company_id":76,"service_uuids":["fd44"],"service_data_uuid":"fd5f"
}}
```
New fields are ADDITIVE — brain already ignores unknown fields (verify in
gateway/orb_gateway.py + mosaic_brain.py before relying on this).

## Constraints
- PASSIVE ONLY. No deauth, no spam, no replay, no jam. (Offensive lives in a
  separate Orb firmware target later — NOT this repo's default env.)
- Keep `config.example.h` pattern: enable features via #define, defaults ON for
  the new passive fields.
- Keep the scan loop duty cycle sane (95% steady / 5% sweep is the design).
- PlatformIO: add nimble-arduino dep. Current env `[env:esp32]` for ESP32 classic
  (NODE-01). Do NOT break the build.
- Do NOT touch gateway/orb_gateway.py — it's LIVE on port 9000 collecting data.
- Do NOT touch mosaic_brain.py / mosaic_mcp.py — they're working (tuner active).

## Files to modify
- `firmware/platformio.ini` — add nimble-arduino
- `firmware/src/main.cpp` — the port
- `firmware/include/config.example.h` — feature flags
- `docs/protocol.md` — document new envelope fields

## Verification
- `cd firmware && /opt/miniconda3/bin/platformio run` must compile (ESP32 classic env)
- If a board is attached (ttyUSB0), flash + observe envelopes with the new fields.
- Gateway MUST keep running untouched on port 9000.

## Definition of done
- Compiles clean with platformio
- Envelope includes device_class + company_id + service_uuids when present
- FindMy/AirTag devices get `device_class:"findmy"`
- README/docs updated
- Committed to git (repo is PUBLIC — no secrets)
