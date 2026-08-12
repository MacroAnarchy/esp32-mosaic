/*
 * ESP32-Mosaic — BLE sense engine, native ESP-IDF (Apache NimBLE host).
 *
 * Passive BLE presence detection for the Mosaic node swarm:
 *   1. BLE scan (controller-level duplicate filter, DEVICE mode)
 *   2. AD payload parsing: manufacturer data (company_id), service UUIDs,
 *      service data UUID
 *   3. Device classification: "findmy" | "meta" | "flipper" | "unknown"
 *   4. HTTP POST → gateway /ingest (Mosaic protocol v1 envelope)
 *
 * Ported from the Arduino sense engine (firmware/src/main.cpp, NimBLE-
 * Arduino) — the API maps 1:1 onto the native esp_nimble stack. The
 * envelope protocol is byte-identical: {v, node, type, ts, payload}.
 *
 * Passive only: NO deauth, NO BLE spam, NO replay, NO jamming.
 * See docs/spec-ble-port.md.
 */

#include <cstdio>
#include <cstring>
#include <string>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"

// Local config (gitignored — copy from include/config.example.h).
#include "config.h"

// ARP neighbor discovery — normal WiFi client behavior: periodically probe
// the subnet, track MAC→IP pairs, report join/leave events to the gateway.
#include "arp_neighbors.h"

// WiFi CSI motion sensing (Tier 1) — monostatic router geometry via the
// Espressif esp_wifi_sensing component. See csi_sensing.h.
#include "csi_sensing.h"

#include "sense_common.h"
#include "sense_engine.h"

// ---- Feature flags (defaults ON — override in config.h) ----
#ifndef MOSAIC_BLE_ENABLE_AD_PARSING
#define MOSAIC_BLE_ENABLE_AD_PARSING 1
#endif
#ifndef MOSAIC_BLE_ENABLE_CLASSIFY
#define MOSAIC_BLE_ENABLE_CLASSIFY 1
#endif
#ifndef MOSAIC_BLE_ENABLE_FINDMY
#define MOSAIC_BLE_ENABLE_FINDMY 1
#endif
#ifndef MOSAIC_BLE_DUPLICATE_CACHE_SIZE
#define MOSAIC_BLE_DUPLICATE_CACHE_SIZE 200
#endif
#ifndef MOSAIC_SCAN_INTERVAL
#define MOSAIC_SCAN_INTERVAL 15000
#endif

// WiFi offline scan cycle (see docs/spec-wifi-scan.md): drop WiFi ->
// promiscuous -> hop ch 1..13 -> capture beacons/probes -> reconnect ->
// report as one type:"wifi" batch envelope.
#ifndef MOSAIC_WIFI_SCAN_ENABLE
#define MOSAIC_WIFI_SCAN_ENABLE 1
#endif
#ifndef MOSAIC_WIFI_SCAN_INTERVAL_SECONDS
#define MOSAIC_WIFI_SCAN_INTERVAL_SECONDS 300
#endif
#ifndef MOSAIC_WIFI_SCAN_CHANNEL_HOLD_MS
#define MOSAIC_WIFI_SCAN_CHANNEL_HOLD_MS 80
#endif
#ifndef MOSAIC_WIFI_RECONNECT_TIMEOUT_MS
#define MOSAIC_WIFI_RECONNECT_TIMEOUT_MS 15000
#endif
#ifndef MOSAIC_WIFI_RECONNECT_RETRIES
#define MOSAIC_WIFI_RECONNECT_RETRIES 3
#endif
#define WIFI_SCAN_MAX_FRAMES 64  // batch cap per cycle
#define WIFI_SCAN_MAX_CHANNEL 13  // 2.4GHz channels 1..13

// WiFi credentials: MOSAIC_* names from include/config.h (fallback so a
// config.h copied from config.example.h compiles too).
#ifndef MOSAIC_WIFI_SSID
#define MOSAIC_WIFI_SSID "YOUR_WIFI_SSID"
#define MOSAIC_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

static const char *TAG = "sense";

// ---- Identity anchors (ported from ESP32Marauder WiFiScan.h/WiFiScan.cpp) ----

// Meta / Luxottica company IDs -> device_class "meta"
static const uint16_t META_IDENTIFIERS[6] = {
    0xFD5F,  // Meta (0xFD5F)
    0xFEB7,  // Meta (0xFEB7)
    0xFEB8,  // Meta (0xFEB8)
    0x01AB,  // Meta (0x01AB)
    0x058E,  // Meta (0x058E)
    0x0D53,  // Luxottica (0x0D53)
};

// High-volume consumer device IDs (Samsung / Apple / Microsoft / phones).
// These are still tracked like any other device, but NEVER classified "meta".
static const uint16_t BLOCKED_IDENTIFIERS[5] = {
    0xFD5A,  // Samsung
    0xFD69,  // Samsung
    0x004C,  // Apple
    0x0006,  // Microsoft
    0xFEF3,  // phone vendors
};

// Apple FindMy / AirTag signatures (Marauder: FMNA_SERVICE_UUID + FMDN).
// 16-bit FMNA service UUID 0xFD44, and the FMDN 128-bit UUID
// 7dfc9001-7d1c-4951-86aa-8d9728f8d66c stored little-endian (the order
// they appear in the AD payload).
static const uint16_t FMNA_SERVICE_UUID16 = 0xFD44;
static const uint8_t FMDN_SERVICE_UUID_LE[16] = {
    0x01, 0x90, 0xfc, 0x7d, 0x1c, 0x7d, 0x51, 0x49,
    0xaa, 0x86, 0x28, 0x97, 0xf8, 0xd6, 0x6c, 0x7d,
};

// Flipper Zero manufacturer ID (Marauder: 0x0FBA)
static const uint16_t FLIPPER_MANUFACTURER_ID = 0x0FBA;

// ---- Scan window limits ----
#define MAX_DEVICES 64
#define MAX_SERVICE_UUIDS 8
#define MAC_STR_LEN 18   // "aa:bb:cc:dd:ee:ff" + NUL
#define NAME_MAX_LEN 32  // printable name bytes kept

// One parsed device sighting within the current scan window.
struct DeviceRecord {
  char mac[MAC_STR_LEN];
  int8_t rssi;
  bool haveName;
  char name[NAME_MAX_LEN + 1];
  bool hasCompanyId;
  uint16_t companyId;
  uint8_t serviceUuidCount;
  char serviceUuids[MAX_SERVICE_UUIDS][5];  // 16-bit UUIDs only ("fd44")
  bool hasServiceDataUuid;
  char serviceDataUuid[5];                  // 16-bit identifier ("fd5f")
  char deviceClass[16];                     // findmy | meta | flipper | unknown
};

static DeviceRecord g_records[MAX_DEVICES];
static volatile uint8_t g_recordCount = 0;
static volatile bool g_ble_synced = false;
static volatile bool g_scan_done = false;
static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;

// Guards the device table: written by the NimBLE host task (scan
// callbacks) + the sense task (window reset), read by the UI render
// task via sense_engine_get_devices(). Short critical sections only.
static portMUX_TYPE s_records_lock = portMUX_INITIALIZER_UNLOCKED;

static bool inUint16Table(uint16_t v, const uint16_t *table, size_t n) {
  for (size_t i = 0; i < n; i++)
    if (v == table[i]) return true;
  return false;
}

// Classify a device: findmy -> meta -> flipper -> unknown.
static std::string classifyDevice(const DeviceRecord &rec, bool findmy) {
#if MOSAIC_BLE_ENABLE_FINDMY
  if (findmy) return "findmy";
#endif  // MOSAIC_BLE_ENABLE_FINDMY

  if (rec.hasCompanyId) {
    // High-volume consumer IDs (Samsung/Apple/MS/phones) are still tracked,
    // but never classified — the brain sees them as "unknown".
    if (inUint16Table(rec.companyId, BLOCKED_IDENTIFIERS,
                      sizeof(BLOCKED_IDENTIFIERS) / sizeof(BLOCKED_IDENTIFIERS[0])))
      return "unknown";
    // Meta / Luxottica identity anchors
    if (inUint16Table(rec.companyId, META_IDENTIFIERS,
                      sizeof(META_IDENTIFIERS) / sizeof(META_IDENTIFIERS[0])))
      return "meta";
    // Flipper Zero
    if (rec.companyId == FLIPPER_MANUFACTURER_ID) return "flipper";
  }
  return "unknown";
}

// Apple offline-finding payload signatures anywhere in the AD payload
// (Marauder: 0x1E 0xFF 0x4C 0x00 / 0x4C 0x00 0x12).
static bool findmyPayloadSignature(const uint8_t *data, size_t len) {
  for (size_t i = 0; i + 3 < len; i++) {
    if ((data[i] == 0x1E && data[i + 1] == 0xFF && data[i + 2] == 0x4C &&
         data[i + 3] == 0x00) ||
        (data[i] == 0x4C && data[i + 1] == 0x00 && data[i + 2] == 0x12)) {
      return true;
    }
  }
  return false;
}

// FindMy service presence: FMNA (16-bit) or FMDN (128-bit) advertised.
static bool findmyFromFields(const struct ble_hs_adv_fields *f) {
  for (int i = 0; i < f->num_uuids16; i++) {
    if (f->uuids16[i].value == FMNA_SERVICE_UUID16) return true;
  }
  for (int i = 0; i < f->num_uuids128; i++) {
    if (memcmp(f->uuids128[i].value, FMDN_SERVICE_UUID_LE, 16) == 0) return true;
  }
  return false;
}

// Copy a possibly non-NUL-terminated AD name into a printable C string.
static void copyAdvName(const uint8_t *name, uint8_t nameLen, char *out) {
  if (nameLen > NAME_MAX_LEN) nameLen = NAME_MAX_LEN;
  for (uint8_t i = 0; i < nameLen; i++) {
    char c = (char)name[i];
    out[i] = (c >= 0x20 && c <= 0x7E) ? c : '?';  // printable only
  }
  out[nameLen] = '\0';
}

// Scan callback — runs on the NimBLE host task, once per unique device
// (controller duplicate filter handles dedup; we dedup by MAC as a safety net).
static int bleScanCallback(struct ble_gap_event *event, void *arg) {
  (void)arg;
  switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
      const struct ble_gap_disc_desc *d = &event->disc;
      if (g_recordCount >= MAX_DEVICES) break;  // window full — drop extras

      DeviceRecord rec;
      sense::mac_to_str(d->addr.val, rec.mac, sizeof(rec.mac));
      rec.rssi = d->rssi;
      rec.haveName = false;
      rec.name[0] = '\0';
      rec.hasCompanyId = false;
      rec.companyId = 0;
      rec.serviceUuidCount = 0;
      rec.hasServiceDataUuid = false;
      rec.serviceDataUuid[0] = '\0';
      strncpy(rec.deviceClass, "unknown", sizeof(rec.deviceClass));

      bool findmy = false;
#if MOSAIC_BLE_ENABLE_AD_PARSING || MOSAIC_BLE_ENABLE_FINDMY
      struct ble_hs_adv_fields fields;
      if (ble_hs_adv_parse_fields(&fields, d->data, d->length_data) == 0) {
        if (fields.name != NULL) {
          copyAdvName(fields.name, fields.name_len, rec.name);
          rec.haveName = fields.name_len > 0;
        }
#if MOSAIC_BLE_ENABLE_AD_PARSING
        // Manufacturer data -> company ID (little-endian, bytes 0-1)
        if (fields.mfg_data_len >= 2) {
          rec.companyId = (uint8_t)fields.mfg_data[0] | ((uint8_t)fields.mfg_data[1] << 8);
          rec.hasCompanyId = true;
        }
        // Service UUIDs (16-bit only, e.g. "fd44")
        for (int i = 0;
             i < fields.num_uuids16 && rec.serviceUuidCount < MAX_SERVICE_UUIDS;
             i++) {
          snprintf(rec.serviceUuids[rec.serviceUuidCount], 5, "%04x",
                   fields.uuids16[i].value);
          rec.serviceUuidCount++;
        }
        // Service data -> 16-bit identifier (e.g. "fd5f")
        if (fields.svc_data_uuid16_len >= 2) {
          uint16_t uuid = (uint8_t)fields.svc_data_uuid16[0] |
                          ((uint8_t)fields.svc_data_uuid16[1] << 8);
          snprintf(rec.serviceDataUuid, sizeof(rec.serviceDataUuid), "%04x", uuid);
          rec.hasServiceDataUuid = true;
        }
#endif  // MOSAIC_BLE_ENABLE_AD_PARSING
#if MOSAIC_BLE_ENABLE_FINDMY
        findmy = findmyFromFields(&fields);
#endif  // MOSAIC_BLE_ENABLE_FINDMY
      }
#endif  // MOSAIC_BLE_ENABLE_AD_PARSING || MOSAIC_BLE_ENABLE_FINDMY

#if MOSAIC_BLE_ENABLE_FINDMY
      if (!findmy) findmy = findmyPayloadSignature(d->data, d->length_data);
#endif  // MOSAIC_BLE_ENABLE_FINDMY

#if MOSAIC_BLE_ENABLE_CLASSIFY
      std::string cls = classifyDevice(rec, findmy);
      strncpy(rec.deviceClass, cls.c_str(), sizeof(rec.deviceClass) - 1);
#endif  // MOSAIC_BLE_ENABLE_CLASSIFY

      // Dedup by MAC within the window (controller filter is the primary guard)
      portENTER_CRITICAL(&s_records_lock);
      for (uint8_t i = 0; i < g_recordCount; i++) {
        if (strcmp(g_records[i].mac, rec.mac) == 0) {
          g_records[i].rssi = rec.rssi;  // refresh signal strength
          portEXIT_CRITICAL(&s_records_lock);
          return 0;
        }
      }
      uint8_t idx = g_recordCount;
      g_records[idx] = rec;
      g_recordCount = idx + 1;
      portEXIT_CRITICAL(&s_records_lock);
      break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
      g_scan_done = true;
      break;
    default:
      break;
  }
  return 0;
}

static void bleHostTask(void *param) {
  ESP_LOGI(TAG, "NimBLE host task started");
  /* This function returns only when nimble_port_stop() is executed */
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void bleOnSync(void) {
  g_ble_synced = true;
  int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_hs_id_infer_auto failed rc=%d — using public addr", rc);
    g_own_addr_type = BLE_OWN_ADDR_PUBLIC;
  }
}

static void bleOnReset(int reason) {
  g_ble_synced = false;
  ESP_LOGW(TAG, "NimBLE reset, reason=%d", reason);
}

static std::string jsonEscape(const std::string &s) {
  std::string out;
  for (size_t i = 0; i < s.size(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else if ((uint8_t)c < 0x20) {
      out += '?';  // drop other control chars
    } else {
      out += c;
    }
  }
  return out;
}

// Gateway endpoint (from config.h)
static std::string gatewayIngestUrl() {
  return sense::gateway_ingest_url();
}

// Envelope payload entry for one device — additive fields only.
static std::string recordToJson(const DeviceRecord &rec) {
  std::string j = "{\"mac\":\"" + std::string(rec.mac) + "\",\"rssi\":" +
                  std::to_string(rec.rssi);
  if (rec.haveName) j += ",\"name\":\"" + jsonEscape(rec.name) + "\"";
  j += ",\"device_class\":\"" + std::string(rec.deviceClass) + "\"";
#if MOSAIC_BLE_ENABLE_AD_PARSING
  if (rec.hasCompanyId) j += ",\"company_id\":" + std::to_string(rec.companyId);
  if (rec.serviceUuidCount > 0) {
    j += ",\"service_uuids\":[";
    for (uint8_t i = 0; i < rec.serviceUuidCount; i++) {
      if (i > 0) j += ",";
      j += "\"" + std::string(rec.serviceUuids[i]) + "\"";
    }
    j += "]";
  }
  if (rec.hasServiceDataUuid)
    j += ",\"service_data_uuid\":\"" + std::string(rec.serviceDataUuid) + "\"";
#endif  // MOSAIC_BLE_ENABLE_AD_PARSING
  j += "}";
  return j;
}

// Report one type:"scan" envelope for the last window.
static void postScanEnvelope(int count) {
  char bssid[MAC_STR_LEN] = "";
  sense_wifi_get_ap_bssid(bssid, sizeof(bssid));

  std::string payload = "{\"v\":1,\"node\":\"" + std::string(MOSAIC_NODE_NAME) +
                        "\",\"type\":\"scan\",\"ts\":" +
                        std::to_string(sense::uptime_ms()) +
                        ",\"payload\":{\"ap_bssid\":\"" + bssid +
                        "\",\"devices\":[";
  for (int i = 0; i < count; i++) {
    if (i > 0) payload += ",";
    payload += recordToJson(g_records[i]);
  }
  payload += "]}}";

  int code = sense::post_json(gatewayIngestUrl(), payload);
  ESP_LOGI(TAG, "Gateway POST: %d", code);
}

// =====================================================================
// WiFi offline scan cycle ("the mosaic goes offline to see")
// ---------------------------------------------------------------------
// The ESP32 has ONE radio: while associated to WiFi it is blind to the
// rest of the spectrum. Every MOSAIC_WIFI_SCAN_INTERVAL_SECONDS we drop
// the association, enter promiscuous mode, hop channels 1..13 (~80ms
// each), capture 802.11 beacons (0x80) + probe requests (0x40) — PASSIVE
// listen only — then reconnect and report the batch as one envelope.
// Ported (passive parts) from ESP32Marauder's WiFiScan.cpp.
// =====================================================================

struct WifiFrame {
  char kind[12];   // "beacon" | "probe_req"
  char mac[MAC_STR_LEN];
  char bssid[MAC_STR_LEN];  // beacon: the AP BSSID; probe: empty
  char ssid[33];            // beacon: broadcast name / probe: requested SSID
  uint8_t channel;
  int8_t rssi;
};

static WifiFrame g_wifiFrames[WIFI_SCAN_MAX_FRAMES];
static volatile uint8_t g_wifiFrameCount = 0;
static volatile bool g_wifiScanActive = false;

// Promiscuous RX callback — runs on the WiFi task, keep it fast.
// Parses management frames only: 0x80 = beacon, 0x40 = probe request.
static void wifiPromiscuousCallback(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;  // mgmt frames only
  if (!g_wifiScanActive) return;
  if (g_wifiFrameCount >= WIFI_SCAN_MAX_FRAMES) return;

  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  const uint8_t *payload = pkt->payload;
  if (payload[0] != 0x80 && payload[0] != 0x40) return;  // beacon / probe req

  // Sanity: frame must at least reach the ESSID length bytes
  if (pkt->rx_ctrl.sig_len < 40) return;

  WifiFrame fr;
  char macBuf[MAC_STR_LEN];
  fr.channel = pkt->rx_ctrl.channel;
  fr.rssi = pkt->rx_ctrl.rssi;

  // src MAC at payload[10..15], dst at [4..9]
  sense::mac_to_str(&payload[10], macBuf, sizeof(macBuf));
  snprintf(fr.mac, sizeof(fr.mac), "%s", macBuf);

  if (payload[0] == 0x80) {
    // Beacon: AP advertising — BSSID == src MAC; ESSID len at payload[37],
    // name at payload[38..38+len]
    snprintf(fr.kind, sizeof(fr.kind), "beacon");
    sense::mac_to_str(&payload[10], macBuf, sizeof(macBuf));  // BSSID == src for beacons
    snprintf(fr.bssid, sizeof(fr.bssid), "%s", macBuf);
    uint8_t essidLen = payload[37];
    if (essidLen > 32) essidLen = 32;
    if (essidLen > 0) {
      size_t n = 0;
      for (uint8_t i = 0; i < essidLen; i++) {
        char c = (char)payload[38 + i];
        fr.ssid[n++] = (c >= 0x20 && c <= 0x7E) ? c : '?';  // printable only
      }
      fr.ssid[n] = '\0';
    } else {
      fr.ssid[0] = '\0';
    }
  } else {
    // Probe request: client seeking — requested SSID len at payload[25],
    // name at payload[26..26+len]; BSSID field is empty (we don't chase it)
    snprintf(fr.kind, sizeof(fr.kind), "probe_req");
    fr.bssid[0] = '\0';
    uint8_t essidLen = payload[25];
    if (essidLen > 32) essidLen = 32;
    if (essidLen > 0) {
      size_t n = 0;
      for (uint8_t i = 0; i < essidLen; i++) {
        char c = (char)payload[26 + i];
        fr.ssid[n++] = (c >= 0x20 && c <= 0x7E) ? c : '?';
      }
      fr.ssid[n] = '\0';
    } else {
      fr.ssid[0] = '\0';
    }
  }

  // Dedup within the cycle: same kind + mac + ssid => keep the stronger RSSI
  for (uint8_t i = 0; i < g_wifiFrameCount; i++) {
    WifiFrame &e = g_wifiFrames[i];
    if (strcmp(e.kind, fr.kind) == 0 && strcmp(e.mac, fr.mac) == 0 &&
        strcmp(e.ssid, fr.ssid) == 0) {
      if (fr.rssi > e.rssi) e.rssi = fr.rssi;
      return;
    }
  }
  uint8_t idx = g_wifiFrameCount;
  g_wifiFrames[idx] = fr;
  g_wifiFrameCount = idx + 1;
}

// =====================================================================
// WiFi STA management (esp_netif + esp_wifi, event driven)
// =====================================================================

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_GOT_IP_BIT BIT1
static EventGroupHandle_t s_wifiEvents = NULL;

static void wifiEventHandler(void *arg, esp_event_base_t base, int32_t id,
                             void *data) {
  (void)arg;
  (void)data;
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    xEventGroupSetBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);
  }
}

// Blocking WiFi join with timeout; returns true when connected (IP lease).
static bool wifiJoinBlocking(int timeoutMs, int retries) {
  for (int attempt = 1; attempt <= retries; attempt++) {
    esp_wifi_connect();
    EventBits_t bits = xEventGroupWaitBits(s_wifiEvents, WIFI_GOT_IP_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(timeoutMs));
    if (bits & WIFI_GOT_IP_BIT) return true;
    xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);
    ESP_LOGW(TAG, "  join timeout (attempt %d/%d)", attempt, retries);
  }
  return false;
}

// One offline scan cycle. Assumes caller holds no other radio activity.
static void runWifiScanCycle() {
  ESP_LOGI(TAG, "--- WiFi offline scan (%ds interval) ---",
           MOSAIC_WIFI_SCAN_INTERVAL_SECONDS);

  g_wifiFrameCount = 0;
  g_wifiScanActive = true;

#if MOSAIC_CSI_ENABLE
  // CSI rides the STA association — the radio is about to leave STA mode
  // for the offline sweep, so pause CSI (stop FSM + router ping). It is
  // resumed below after the reconnect.
  csi_sensing_pause();
#endif  // MOSAIC_CSI_ENABLE

  // 1) Drop association, go NULL, enter promiscuous
  esp_wifi_disconnect();
  xEventGroupClearBits(s_wifiEvents, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(wifiPromiscuousCallback);

  // 2) Hop channels 1..13, holding each ~80ms (~1s total)
  for (uint8_t ch = 1; ch <= WIFI_SCAN_MAX_CHANNEL; ch++) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    vTaskDelay(pdMS_TO_TICKS(MOSAIC_WIFI_SCAN_CHANNEL_HOLD_MS));
  }

  // 3) Exit promiscuous, back to STA
  esp_wifi_set_promiscuous(false);
  g_wifiScanActive = false;
  esp_wifi_set_mode(WIFI_MODE_STA);

  // 4) Reconnect with retry loop — a node that can't rejoin is dead.
  //    If join fails we KEEP retrying (outer loop also retries) while the
  //    rest of the sensing cycle (BLE) keeps collecting.
  bool joined = wifiJoinBlocking(MOSAIC_WIFI_RECONNECT_TIMEOUT_MS,
                                 MOSAIC_WIFI_RECONNECT_RETRIES);
  if (joined) {
    char bssid[MAC_STR_LEN] = "";
    sense_wifi_get_ap_bssid(bssid, sizeof(bssid));
    ESP_LOGI(TAG, "  reconnected. BSSID: %s", bssid);
#if MOSAIC_CSI_ENABLE
    // Back on the AP channel — CSI resumes and relearns its baseline.
    csi_sensing_resume();
#endif  // MOSAIC_CSI_ENABLE
  } else {
    ESP_LOGW(TAG, "  reconnect FAILED — will keep retrying on next cycle");
  }

  // 5) Report the batch envelope (only if back online and frames captured)
  if (joined && g_wifiFrameCount > 0) {
    std::string payload = "{\"v\":1,\"node\":\"" + std::string(MOSAIC_NODE_NAME) +
                          "\",\"type\":\"wifi\",\"ts\":" +
                          std::to_string(sense::uptime_ms()) +
                          ",\"payload\":{\"frames\":[";
    for (uint8_t i = 0; i < g_wifiFrameCount; i++) {
      if (i > 0) payload += ",";
      const WifiFrame &fr = g_wifiFrames[i];
      payload += "{\"kind\":\"" + std::string(fr.kind) + "\",\"mac\":\"" +
                 fr.mac + "\"";
      if (fr.bssid[0] != '\0') payload += ",\"bssid\":\"" + std::string(fr.bssid) + "\"";
      if (fr.ssid[0] != '\0') payload += ",\"ssid\":\"" + jsonEscape(fr.ssid) + "\"";
      payload += ",\"channel\":" + std::to_string(fr.channel) +
                 ",\"rssi\":" + std::to_string(fr.rssi) + "}";
    }
    payload += "]}}";

    int code = sense::post_json(gatewayIngestUrl(), payload);
    ESP_LOGI(TAG, "  wifi batch POST: %d (%d frames)", code, g_wifiFrameCount);
  }
  g_wifiFrameCount = 0;
}

// =====================================================================
// Public WiFi accessors (used by the ARP module + face layer)
// =====================================================================

bool sense_wifi_is_connected(void) {
  if (!s_wifiEvents) return false;
  return (xEventGroupGetBits(s_wifiEvents) & WIFI_GOT_IP_BIT) != 0;
}

bool sense_wifi_get_ap_bssid(char *out, size_t out_len) {
  if (!out || out_len < MAC_STR_LEN) return false;
  wifi_ap_record_t ap;
  if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return false;
  sense::mac_to_str(ap.bssid, out, out_len);
  return true;
}

bool sense_wifi_get_ipv4(uint8_t ip[4], uint8_t mask[4]) {
  esp_netif_t *n = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (!n) return false;
  esp_netif_ip_info_t info;
  if (esp_netif_get_ip_info(n, &info) != ESP_OK) return false;
  ip[0] = esp_ip4_addr1(&info.ip);
  ip[1] = esp_ip4_addr2(&info.ip);
  ip[2] = esp_ip4_addr3(&info.ip);
  ip[3] = esp_ip4_addr4(&info.ip);
  mask[0] = esp_ip4_addr1(&info.netmask);
  mask[1] = esp_ip4_addr2(&info.netmask);
  mask[2] = esp_ip4_addr3(&info.netmask);
  mask[3] = esp_ip4_addr4(&info.netmask);
  return true;
}

int sense_engine_get_device_count(void)
{
  int n;
  portENTER_CRITICAL(&s_records_lock);
  n = (int)g_recordCount;
  portEXIT_CRITICAL(&s_records_lock);
  return n;
}

int sense_engine_get_devices(sense_device_t *out, int max_records)
{
  if (out == NULL || max_records <= 0) return 0;
  portENTER_CRITICAL(&s_records_lock);
  int n = (int)g_recordCount;
  if (n > max_records) n = max_records;
  for (int i = 0; i < n; i++) {
    strncpy(out[i].mac, g_records[i].mac, sizeof(out[i].mac) - 1);
    out[i].mac[sizeof(out[i].mac) - 1] = '\0';
    out[i].rssi = g_records[i].rssi;
    strncpy(out[i].deviceClass, g_records[i].deviceClass,
            sizeof(out[i].deviceClass) - 1);
    out[i].deviceClass[sizeof(out[i].deviceClass) - 1] = '\0';
    out[i].haveName = g_records[i].haveName;
    strncpy(out[i].name, g_records[i].name, sizeof(out[i].name) - 1);
    out[i].name[sizeof(out[i].name) - 1] = '\0';
  }
  portEXIT_CRITICAL(&s_records_lock);
  return n;
}

// =====================================================================
// Sense task — the loop the Arduino build ran in loop(), now a FreeRTOS
// task so the face render loop keeps running alongside it.
// =====================================================================
static void senseTask(void *arg) {
  (void)arg;

  // Wait for the NimBLE host to sync (up to 15s)
  uint32_t t0 = sense::uptime_ms();
  while (!g_ble_synced && (sense::uptime_ms() - t0) < 15000)
    vTaskDelay(pdMS_TO_TICKS(100));
  if (!g_ble_synced) ESP_LOGW(TAG, "NimBLE not synced after 15s — will retry each cycle");

  // Join WiFi; a node that can't rejoin is dead — keep retrying forever.
  while (!sense_wifi_is_connected()) {
    wifiJoinBlocking(MOSAIC_WIFI_RECONNECT_TIMEOUT_MS, 1);
    if (!sense_wifi_is_connected()) vTaskDelay(pdMS_TO_TICKS(5000));
  }
  char bssid[MAC_STR_LEN] = "";
  sense_wifi_get_ap_bssid(bssid, sizeof(bssid));
  ESP_LOGI(TAG, "WiFi connected. BSSID: %s", bssid);

#if MOSAIC_CSI_ENABLE
  // WiFi CSI motion sensing — needs the STA association (AP BSSID + gateway
  // for ping-assisted sampling). WiFi is up here; on the rare failure just
  // log — the node keeps sensing BLE/ARP, CSI is additive.
  esp_err_t csiInit = csi_sensing_init();
  if (csiInit != ESP_OK) {
    ESP_LOGW(TAG, "CSI init failed: %s", esp_err_to_name(csiInit));
  }
#endif  // MOSAIC_CSI_ENABLE

  uint32_t lastWifiScanMs = 0;

  while (1) {
#if MOSAIC_CSI_ENABLE
    // CSI motion events are drained once per loop (≤ loop period latency)
    // and POSTed as type:"csi" envelopes. Queue full / gateway down → drop
    // (lossy by design).
    csi_sensing_drain_and_report();
#endif  // MOSAIC_CSI_ENABLE
    // Scan for 5 seconds (callbacks fill g_records meanwhile)
    ESP_LOGI(TAG, "--- BLE scan (5s) ---");
    portENTER_CRITICAL(&s_records_lock);
    g_recordCount = 0;
    portEXIT_CRITICAL(&s_records_lock);
    g_scan_done = false;
    if (g_ble_synced) {
      struct ble_gap_disc_params dp;
      memset(&dp, 0, sizeof(dp));
      dp.itvl = 100;
      dp.window = 99;
      dp.passive = 0;            // active scan: fetch names via scan requests
      dp.filter_duplicates = 1;  // controller-level dedup (DEVICE mode)
      int rc = ble_gap_disc(g_own_addr_type, 5000, &dp, bleScanCallback, NULL);
      if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_disc failed rc=%d", rc);
      } else {
        uint32_t waitStart = sense::uptime_ms();
        while (!g_scan_done && (sense::uptime_ms() - waitStart) < 7000)
          vTaskDelay(pdMS_TO_TICKS(50));
        if (!g_scan_done) ble_gap_disc_cancel();  // safety net
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
    int count = (int)g_recordCount;
    ESP_LOGI(TAG, "Found %d devices", count);

    // Report to gateway — Mosaic protocol v1 envelope with real BSSID
    if (count > 0 && sense_wifi_is_connected()) postScanEnvelope(count);

#if MOSAIC_WIFI_SCAN_ENABLE
    // WiFi offline scan cycle — sequential with BLE (one radio, phases
    // never overlap). Every MOSAIC_WIFI_SCAN_INTERVAL_SECONDS: drop WiFi,
    // sniff channels 1..13, reconnect, report the batch.
    uint32_t now = sense::uptime_ms();
    if (lastWifiScanMs == 0 ||
        (now - lastWifiScanMs) >= (uint32_t)MOSAIC_WIFI_SCAN_INTERVAL_SECONDS * 1000UL) {
      if (sense_wifi_is_connected()) {
        runWifiScanCycle();
        lastWifiScanMs = sense::uptime_ms();
      } else {
        // Still offline from a failed cycle — keep retrying the join while
        // BLE keeps collecting. A node that can't rejoin WiFi is dead.
        ESP_LOGI(TAG, "--- WiFi down — retrying join (BLE keeps running) ---");
        wifiJoinBlocking(MOSAIC_WIFI_RECONNECT_TIMEOUT_MS, 1);
        if (sense_wifi_is_connected()) {
          lastWifiScanMs = sense::uptime_ms();
#if MOSAIC_CSI_ENABLE
          // Back online after a failed offline-scan reconnect — bring CSI
          // back (no-op when CSI is already running or was never started).
          csi_sensing_resume();
#endif  // MOSAIC_CSI_ENABLE
        }
      }
    }
#endif  // MOSAIC_WIFI_SCAN_ENABLE

#if MOSAIC_ARP_ENABLE
    // ARP neighbor discovery — the node is an ordinary WiFi station, so it
    // probes its subnet like any other host (normal network membership).
    // Runs only while associated; reports join/leave as protocol envelopes.
    arpNeighborsLoop();
#endif  // MOSAIC_ARP_ENABLE

    // Pause between scans (configurable; default 15s per config.example.h)
    vTaskDelay(pdMS_TO_TICKS(MOSAIC_SCAN_INTERVAL));
  }
}

// =====================================================================
// Init
// =====================================================================
esp_err_t sense_engine_init(void) {
  // NVS — used by NimBLE and the WiFi driver (PHY calibration)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // ---- BLE: native NimBLE host + controller ----
  ret = nimble_port_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
    return ret;
  }
  ble_hs_cfg.sync_cb = bleOnSync;
  ble_hs_cfg.reset_cb = bleOnReset;
  nimble_port_freertos_init(bleHostTask);

  // ---- WiFi: esp_netif + esp_wifi, event driven ----
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  s_wifiEvents = xEventGroupCreate();
  if (!s_wifiEvents) return ESP_ERR_NO_MEM;

  wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
  ret = esp_wifi_init(&wc);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             wifiEventHandler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             wifiEventHandler, NULL));

  wifi_config_t wcfg = {};
  strncpy((char *)wcfg.sta.ssid, MOSAIC_WIFI_SSID, sizeof(wcfg.sta.ssid) - 1);
  strncpy((char *)wcfg.sta.password, MOSAIC_WIFI_PASSWORD,
          sizeof(wcfg.sta.password) - 1);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "connecting to %s...", MOSAIC_WIFI_SSID);

  // ---- Sense loop task (runs alongside the face render task) ----
  BaseType_t ok = xTaskCreate(senseTask, "sense", 4096, NULL, 5, NULL);
  if (ok != pdPASS) return ESP_ERR_NO_MEM;

  return ESP_OK;
}
