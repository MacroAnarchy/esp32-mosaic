/*
 * sense_common: small shared helpers for the ESP-IDF sense engine
 * (components/sense). Framework-agnostic glue: uptime clock, MAC
 * formatting and the gateway HTTP POST used by every envelope sender
 * (BLE scan batches, WiFi offline-scan batches, ARP join/leave events).
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace sense {

inline uint32_t uptime_ms() {
  return (uint32_t)(esp_timer_get_time() / 1000);
}

// Format a 6-byte MAC as "aa:bb:cc:dd:ee:ff".
inline void mac_to_str(const uint8_t mac[6], char *out, size_t out_len) {
  snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Gateway /orb/ingest URL from config.h's MOSAIC_GATEWAY_HOST/PORT. Requires
// config.h to be included first (same convention as the rest of the
// component). NOTE: the gateway (orb_gateway.py) serves POST /orb/ingest —
// plain /ingest returns 404 (orb-01 outage 2026-08-13, fixed).
inline std::string gateway_ingest_url() {
  return std::string("http://") + MOSAIC_GATEWAY_HOST + ":" +
         std::to_string(MOSAIC_GATEWAY_PORT) + "/orb/ingest";
}

// POST a JSON body to url (Mosaic protocol v1 envelope). Returns the HTTP
// status code (200 etc.) or -1 on transport error. The gateway expects the
// envelope shape {v, node, type, ts, payload} — senders build it, we only
// carry it.
inline int post_json(const std::string &url, const std::string &body) {
  esp_http_client_config_t cfg = {};
  cfg.url = url.c_str();
  cfg.method = HTTP_METHOD_POST;
  cfg.timeout_ms = 10000;
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) {
    ESP_LOGE("sense", "http client init failed for %s", url.c_str());
    return -1;
  }
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_post_field(client, body.data(), (int)body.size());
  esp_err_t err = esp_http_client_perform(client);
  int status = (err == ESP_OK) ? esp_http_client_get_status_code(client) : -1;
  if (err != ESP_OK) {
    ESP_LOGW("sense", "POST %s failed: %s", url.c_str(), esp_err_to_name(err));
  }
  esp_http_client_cleanup(client);
  return status;
}

}  // namespace sense
