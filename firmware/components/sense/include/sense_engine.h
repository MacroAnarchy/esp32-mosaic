/*
 * sense_engine: ESP-IDF port of the Mosaic BLE sense engine.
 *
 * The same passive sensing every bare tile runs, now native ESP-IDF so it
 * can be built together with the UI face layer in one firmware:
 *
 *   1. BLE scan via native NimBLE (esp_nimble → Apache NimBLE host API)
 *   2. AD payload parsing: manufacturer data (company_id), service UUIDs,
 *      service data UUID
 *   3. Device classification: "findmy" | "meta" | "flipper" | "unknown"
 *   4. HTTP POST → gateway /ingest (Mosaic protocol v1 envelope)
 *   5. Periodic WiFi offline scan cycle (promiscuous, channels 1..13)
 *   6. ARP neighbor discovery (see arp_neighbors.h)
 *
 * C-callable so the face node's app main (ui/main_ui.c, C) can drive it.
 * Ported from the Arduino sense engine (firmware/src/main.cpp, NimBLE-
 * Arduino) — the API maps 1:1 onto the native stack. Protocol unchanged.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize NVS + WiFi + the NimBLE host/controller and start the sense
 * task (BLE scan → gateway POST → WiFi offline scan → ARP, in a loop).
 * Safe to call once from app_main. Returns ESP_OK on success. */
esp_err_t sense_engine_init(void);

/* Number of devices seen in the last completed BLE scan (for the face
 * layer / logging). Thread-safe read. */
int sense_engine_get_device_count(void);

/* True when the node currently has an IP lease (gateway reachable). */
bool sense_wifi_is_connected(void);

/* Connected AP BSSID as "aa:bb:cc:dd:ee:ff"; false when not connected. */
bool sense_wifi_get_ap_bssid(char *out, size_t out_len);

/* Current IPv4 + netmask octets; false when no lease. */
bool sense_wifi_get_ipv4(uint8_t ip[4], uint8_t mask[4]);

#ifdef __cplusplus
}
#endif
