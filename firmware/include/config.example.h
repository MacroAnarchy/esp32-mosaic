// ESP32-Mosaic — firmware configuration
//
// Copy this file to config.h (same folder) and fill in your values.
// config.h is gitignored — your credentials never get committed.
//
//   cp include/config.example.h include/config.h

#pragma once

// --- WiFi ---
#define MOSAIC_WIFI_SSID     "YOUR_WIFI_SSID"
#define MOSAIC_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// --- Gateway ---
#define MOSAIC_GATEWAY_HOST  "192.168.1.10"
#define MOSAIC_GATEWAY_PORT  9000

// --- Node identity (unique per board in your swarm) ---
#define MOSAIC_NODE_NAME     "mosaic-01"

// --- Scan interval (ms) ---
#define MOSAIC_SCAN_INTERVAL 15000

// --- BLE passive sense features (all default ON) ---
// These add AD-payload parsing + device classification to the scan envelope.
// All passive: no deauth, no BLE spam, no replay, no jamming.
// Ported from ESP32Marauder (WiFiScan.cpp/h) — see docs/spec-ble-port.md.

// Parse manufacturer data (company_id), service UUIDs and service data from
// every advertisement. Adds: payload.company_id, payload.service_uuids,
// payload.service_data_uuid.
#define MOSAIC_BLE_ENABLE_AD_PARSING 1

// Classify devices and emit payload.device_class:
//   "findmy" | "meta" | "flipper" | "unknown"
#define MOSAIC_BLE_ENABLE_CLASSIFY 1

// Apple FindMy / AirTag detection: FMNA service (0xFD44), FMDN service
// (7dfc9001-...) and the Apple offline-finding payload signatures
// (0x1E 0xFF 0x4C 0x00 / 0x4C 0x00 0x12) -> device_class "findmy".
#define MOSAIC_BLE_ENABLE_FINDMY 1

// Controller-level duplicate filter: report each device once per scan window.
// 200 entries is the same cache size ESP32Marauder uses.
#define MOSAIC_BLE_DUPLICATE_CACHE_SIZE 200

// --- WiFi passive offline scan (default ON) ---
// The "mosaic goes offline to see" cycle: every N seconds the node drops its
// WiFi association for ~1s, hops channels 1..13 in promiscuous mode, captures
// 802.11 beacons + probe requests (PASSIVE listen only — no deauth, no
// injection, no beacon spam), reconnects, then reports the whole batch as one
// type:"wifi" envelope. See docs/spec-wifi-scan.md.

// Enable the periodic WiFi offline scan cycle (runs sequentially with BLE).
#define MOSAIC_WIFI_SCAN_ENABLE 1

// How often (seconds) the node goes offline to sweep the WiFi spectrum.
// Default 300 = every 5 minutes (duty cycle ~95% connected / ~5% offline).
#define MOSAIC_WIFI_SCAN_INTERVAL_SECONDS 300

// Time (ms) spent listening on each channel. 13 channels x 80ms ≈ 1s sweep.
#define MOSAIC_WIFI_SCAN_CHANNEL_HOLD_MS 80
