# ESP32-Mosaic WiFi Sniffing Port (offline scan cycle)

**Goal:** Add passive WiFi sniffing (beacons + probe requests) to the Mosaic
firmware using Marauder's pattern: drop WiFi association, hop channels
promiscuously, capture, reconnect, report. The "mosaic goes offline to see" cycle.

**Source reference:** `/home/owner/workspace/ESP32Marauder/esp32_marauder/WiFiScan.cpp`
- `beaconSnifferCallback` (~line 8024) — the packet parser
- `setWiFiMode(WIFI_MODE_NULL, cb)` + `esp_wifi_set_promiscuous(true)` (~line 3297)
- `changeChannel()` — channel hopping
- Probe capture pattern from `wifiSnifferCallback` (~line 9245)

**Target:** `/home/owner/workspace/esp32-mosaic/firmware/src/main.cpp`
(after the BLE port lands — this builds on top of it)

## The design (the design)

The ESP32 has ONE radio. Connected to WiFi = blind to everything else.
The mosaic goes OFFLINE to see, then comes back to report:

```
[CONNECTED 95%]  BLE scan + report to gateway
[OFFLINE 5%]     drop WiFi → WIFI_MODE_NULL → promiscuous on
                 → hop ch 1..13, ~70-80ms each (~1s total)
                 → capture beacons (0x80) + probe requests (0x40)
                 → disconnect promiscuous → reconnect WiFi
                 → report collected frames as one batch envelope
```

Duty cycle = the SCANNER_DUTY_CYCLE design (95% steady / 5% sweep) already in the
project doc. Config knob: `wifi_scan_interval_seconds` (default e.g. 300 = every
5 min) + `wifi_scan_channel_hold_ms` (default 80).

## Packet parsing (from Marauder)

```c
wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
// src MAC: payload[10..15], dst MAC: payload[4..9]
// RSSI: pkt->rx_ctrl.rssi, channel: pkt->rx_ctrl.channel
// type byte: payload[0]
//   0x80 = beacon  (AP broadcasting)   → BSSID + ESSID
//   0x40 = probe request (client seeking) → client MAC + requested SSID
// ESSID: len at payload[37], name at payload[38..38+len]
```

## New envelope fields (additive)

```json
{"v":1,"node":"orb","type":"wifi","ts":<epoch>,"payload":{
  "kind":"beacon|probe_req",
  "mac":"aa:bb:cc:dd:ee:ff",     // src (AP for beacon, client for probe)
  "bssid":"...",                 // beacon: the AP
  "ssid":"HomeWiFi",             // beacon: broadcast name / probe: requested
  "channel":6,
  "rssi":-58
}}
```

## Files to modify
- `firmware/src/main.cpp` — WiFi scan cycle + callback
- `firmware/include/config.example.h` — `WIFI_SCAN_INTERVAL_SECONDS`, `WIFI_SCAN_CHANNEL_HOLD_MS`, enable flag
- `docs/protocol.md` — document `type:"wifi"`

## Constraints
- PASSIVE ONLY. No deauth, no injection, no evil portal, no pwnagotchi beacon injection.
- The gateway is LIVE on port 9000 — do not touch gateway files.
- BLE scan must keep working (the cycle is sequential: BLE phase, WiFi phase).
- Reconnect MUST work reliably — a node that can't rejoin WiFi is a dead node.
  (Use a reconnect timeout + retry; if join fails, keep trying while still collecting.)
- Verify: `wifi_scan_interval_seconds` honored, reconnect works after each cycle.

## Definition of done
- Compiles clean with platformio
- On a live node: beacons + probe requests appear as `type:"wifi"` envelopes
- Reconnects to WiFi after each offline scan
- Interval configurable, default sane
- docs/protocol.md updated, committed
