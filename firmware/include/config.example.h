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
