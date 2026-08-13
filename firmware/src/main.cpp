/**
 * ESP32-Mosaic — BLE sense engine (NimBLE-Arduino, passive)
 *
 * Passive BLE presence detection for the Mosaic node swarm:
 *   1. BLE scan (controller-level dedup, DEVICE mode / 200-entry cache)
 *   2. AD payload parsing: manufacturer data (company_id), service UUIDs,
 *      service data UUID
 *   3. Device classification: "findmy" | "meta" | "flipper" | "unknown"
 *   4. HTTP POST → gateway /ingest (Mosaic protocol v1 envelope)
 *
 * Ported from ESP32Marauder's WiFiScan.cpp/h (bluetoothScanAllCallback,
 * FindMy detection, META/BLOCKED identifier tables) — passive parts only.
 * NO deauth, NO BLE spam, NO replay, NO jamming. See docs/spec-ble-port.md.
 *
 * Build: pio run -e esp32      (or: pio run)
 * Flash: pio run -e esp32 -t upload   (USB, /dev/ttyUSB0)
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <esp_wifi.h>   // promiscuous mode APIs (passive sniffing)
#include <vector>
#include <string>

// Local config (gitignored — copy from include/config.example.h)
#include "config.h"

// ARP neighbor discovery — normal WiFi client behavior: periodically probe
// the subnet, track MAC→IP pairs, report join/leave events to the gateway.
#include "arp_neighbors.h"

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
#define MOSAIC_SCAN_INTERVAL 10000
#endif

// WiFi offline scan cycle (see docs/spec-wifi-scan.md):
// drop WiFi -> promiscuous -> hop ch 1..13 -> capture beacons/probes ->
// reconnect -> report as one type:"wifi" batch envelope.
#ifndef MOSAIC_WIFI_SCAN_ENABLE
#define MOSAIC_WIFI_SCAN_ENABLE 1
#endif
#ifndef MOSAIC_WIFI_SCAN_INTERVAL_SECONDS
#define MOSAIC_WIFI_SCAN_INTERVAL_SECONDS 300   // every 5 min by default
#endif
#ifndef MOSAIC_WIFI_SCAN_CHANNEL_HOLD_MS
#define MOSAIC_WIFI_SCAN_CHANNEL_HOLD_MS 80     // per-channel listen time
#endif
#ifndef MOSAIC_WIFI_RECONNECT_TIMEOUT_MS
#define MOSAIC_WIFI_RECONNECT_TIMEOUT_MS 15000  // join timeout per attempt
#endif
#ifndef MOSAIC_WIFI_RECONNECT_RETRIES
#define MOSAIC_WIFI_RECONNECT_RETRIES 3         // full join attempts
#endif
#define WIFI_SCAN_MAX_FRAMES 64                 // batch cap per cycle
#define WIFI_SCAN_MAX_CHANNEL 13                // 2.4GHz channels 1..13
#define WIFI_SSID_MAX_LEN 33                    // 32 + NUL

// WiFi credentials: MOSAIC_* names from include/config.h
// (fallback so a config.h copied from config.example.h compiles too)
#ifndef MOSAIC_WIFI_SSID
#define MOSAIC_WIFI_SSID "YOUR_WIFI_SSID"
#define MOSAIC_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

// ---- Identity anchors (ported from ESP32Marauder WiFiScan.h/WiFiScan.cpp) ----

// Meta / Luxottica company IDs → device_class "meta"
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

// Apple FindMy / AirTag signatures (Marauder: FMNA_SERVICE_UUID + FMDN)
static const NimBLEUUID FMNA_SERVICE_UUID("0000fd44-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID FMDN_SERVICE_UUID("7dfc9001-7d1c-4951-86aa-8d9728f8d66c");

// Flipper Zero manufacturer ID (Marauder: 0x0FBA)
static const uint16_t FLIPPER_MANUFACTURER_ID = 0x0FBA;

// ---- Scan window limits ----
#define MAX_DEVICES 64
#define MAX_SERVICE_UUIDS 8

// One parsed device sighting within the current scan window.
struct DeviceRecord {
  String mac;
  int8_t rssi;
  bool haveName;
  String name;
  bool hasCompanyId;
  uint16_t companyId;
  uint8_t serviceUuidCount;
  String serviceUuids[MAX_SERVICE_UUIDS];   // 16-bit UUIDs only ("fd44")
  bool hasServiceDataUuid;
  String serviceDataUuid;                   // 16-bit identifier ("fd5f")
  String deviceClass;                       // findmy | meta | flipper | unknown
};

static DeviceRecord g_records[MAX_DEVICES];
static uint8_t g_recordCount = 0;
static NimBLEScan* pBLEScan = nullptr;

static bool inUint16Table(uint16_t v, const uint16_t* table, size_t n) {
  for (size_t i = 0; i < n; i++)
    if (v == table[i]) return true;
  return false;
}

// Classify a device: findmy → meta → flipper → unknown.
static String classifyDevice(const NimBLEAdvertisedDevice* dev, const DeviceRecord& rec) {
#if MOSAIC_BLE_ENABLE_FINDMY
  // 1) Apple FindMy / AirTag: FMNA or FMDN service advertised, or Apple
  //    offline-finding payload signatures anywhere in the AD payload.
  bool findmy = dev->isAdvertisingService(FMNA_SERVICE_UUID) ||
                dev->isAdvertisingService(FMDN_SERVICE_UUID);
  if (!findmy) {
    const std::vector<uint8_t>& payload = dev->getPayload();
    for (size_t i = 0; i + 3 < payload.size(); i++) {
      if ((payload[i] == 0x1E && payload[i + 1] == 0xFF &&
           payload[i + 2] == 0x4C && payload[i + 3] == 0x00) ||
          (payload[i] == 0x4C && payload[i + 1] == 0x00 && payload[i + 2] == 0x12)) {
        findmy = true;
        break;
      }
    }
  }
  if (findmy) return "findmy";
#endif  // MOSAIC_BLE_ENABLE_FINDMY

  if (rec.hasCompanyId) {
    // High-volume consumer IDs (Samsung/Apple/MS/phones) are still tracked,
    // but never classified — the brain sees them as "unknown".
    if (inUint16Table(rec.companyId, BLOCKED_IDENTIFIERS,
                      sizeof(BLOCKED_IDENTIFIERS) / sizeof(BLOCKED_IDENTIFIERS[0])))
      return "unknown";
    // 2) Meta / Luxottica identity anchors
    if (inUint16Table(rec.companyId, META_IDENTIFIERS,
                      sizeof(META_IDENTIFIERS) / sizeof(META_IDENTIFIERS[0])))
      return "meta";
    // 3) Flipper Zero
    if (rec.companyId == FLIPPER_MANUFACTURER_ID) return "flipper";
  }
  return "unknown";
}

// Scan callback — runs on the NimBLE host task, once per unique device
// (controller duplicate filter handles dedup; we dedup by MAC as a safety net).
class BLEPassiveScanCallback : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (g_recordCount >= MAX_DEVICES) return;  // window full — drop extras

    DeviceRecord rec;
    rec.mac = advertisedDevice->getAddress().toString().c_str();
    rec.rssi = advertisedDevice->getRSSI();
    rec.haveName = advertisedDevice->haveName();
    rec.name = rec.haveName ? advertisedDevice->getName().c_str() : "";
    rec.hasCompanyId = false;
    rec.companyId = 0;
    rec.serviceUuidCount = 0;
    rec.hasServiceDataUuid = false;
    rec.deviceClass = "unknown";

#if MOSAIC_BLE_ENABLE_AD_PARSING
    // Manufacturer data → company ID (little-endian, bytes 0-1)
    if (advertisedDevice->getManufacturerDataCount() > 0) {
      std::string mfg = advertisedDevice->getManufacturerData(0);
      if (mfg.size() >= 2) {
        rec.companyId = (uint8_t)mfg[0] | ((uint8_t)mfg[1] << 8);
        rec.hasCompanyId = true;
      }
    }
    // Service UUIDs (16-bit only, e.g. "fd44")
    uint8_t uuidCount = advertisedDevice->getServiceUUIDCount();
    for (uint8_t i = 0; i < uuidCount && rec.serviceUuidCount < MAX_SERVICE_UUIDS; i++) {
      String s = advertisedDevice->getServiceUUID(i).toString().c_str();
      if (s.length() == 4) rec.serviceUuids[rec.serviceUuidCount++] = s;
    }
    // Service data → 16-bit identifier (e.g. "fd5f")
    if (advertisedDevice->getServiceDataCount() > 0) {
      String s = advertisedDevice->getServiceDataUUID(0).toString().c_str();
      if (s.length() == 4) {
        rec.serviceDataUuid = s;
        rec.hasServiceDataUuid = true;
      }
    }
#endif  // MOSAIC_BLE_ENABLE_AD_PARSING

#if MOSAIC_BLE_ENABLE_CLASSIFY
    rec.deviceClass = classifyDevice(advertisedDevice, rec);
#endif  // MOSAIC_BLE_ENABLE_CLASSIFY

    // Dedup by MAC within the window (controller filter is the primary guard)
    for (uint8_t i = 0; i < g_recordCount; i++) {
      if (g_records[i].mac == rec.mac) {
        g_records[i].rssi = rec.rssi;  // refresh signal strength
        return;
      }
    }
    g_records[g_recordCount++] = rec;
  }
};

static String jsonEscape(const String& s) {
  String out;
  for (unsigned int i = 0; i < s.length(); i++) {
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

// Envelope payload entry for one device — additive fields only.
static String recordToJson(const DeviceRecord& rec) {
  String j = "{\"mac\":\"" + rec.mac + "\",\"rssi\":" + String(rec.rssi);
  if (rec.haveName) j += ",\"name\":\"" + jsonEscape(rec.name) + "\"";
  j += ",\"device_class\":\"" + rec.deviceClass + "\"";
#if MOSAIC_BLE_ENABLE_AD_PARSING
  if (rec.hasCompanyId) j += ",\"company_id\":" + String(rec.companyId);
  if (rec.serviceUuidCount > 0) {
    j += ",\"service_uuids\":[";
    for (uint8_t i = 0; i < rec.serviceUuidCount; i++) {
      if (i > 0) j += ",";
      j += "\"" + rec.serviceUuids[i] + "\"";
    }
    j += "]";
  }
  if (rec.hasServiceDataUuid) j += ",\"service_data_uuid\":\"" + rec.serviceDataUuid + "\"";
#endif  // MOSAIC_BLE_ENABLE_AD_PARSING
  j += "}";
  return j;
}

// Gateway endpoint (from config.h)
const char* gatewayHost = MOSAIC_GATEWAY_HOST;
const int gatewayPort = MOSAIC_GATEWAY_PORT;

// =====================================================================
// WiFi offline scan cycle ("the mosaic goes offline to see")
// ---------------------------------------------------------------------
// The ESP32 has ONE radio: while associated to WiFi it is blind to the
// rest of the spectrum. Every MOSAIC_WIFI_SCAN_INTERVAL_SECONDS we drop
// the association, enter promiscuous mode, hop channels 1..13 (~80ms
// each), capture 802.11 beacons (0x80) + probe requests (0x40) — PASSIVE
// listen only — then reconnect and report the batch as one envelope.
// Ported (passive parts) from ESP32Marauder's WiFiScan.cpp:
//   beaconSnifferCallback (~line 8024) + setWiFiMode (~line 3297).
// =====================================================================

struct WifiFrame {
  String kind;        // "beacon" | "probe_req"
  String mac;         // src MAC (AP for beacon, client for probe)
  String bssid;       // beacon: the AP BSSID; probe: empty
  String ssid;        // beacon: broadcast name / probe: requested SSID
  uint8_t channel;
  int8_t rssi;
};

static WifiFrame g_wifiFrames[WIFI_SCAN_MAX_FRAMES];
static uint8_t g_wifiFrameCount = 0;
static volatile bool g_wifiScanActive = false;

// Format a 6-byte MAC as "aa:bb:cc:dd:ee:ff"
static void macToString(const uint8_t* mac, char* out, size_t outLen) {
  snprintf(out, outLen, "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Promiscuous RX callback — runs on the WiFi task, keep it fast.
// Parses management frames only: 0x80 = beacon, 0x40 = probe request.
static void wifiPromiscuousCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;               // mgmt frames only
  if (!g_wifiScanActive) return;
  if (g_wifiFrameCount >= WIFI_SCAN_MAX_FRAMES) return;

  wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* payload = pkt->payload;
  if (payload[0] != 0x80 && payload[0] != 0x40) return;  // beacon / probe req

  // Sanity: frame must at least reach the ESSID length bytes
  if (pkt->rx_ctrl.sig_len < 40) return;

  WifiFrame fr;
  char macBuf[18];
  fr.channel = pkt->rx_ctrl.channel;
  fr.rssi = pkt->rx_ctrl.rssi;

  // src MAC at payload[10..15], dst at [4..9]
  macToString(&payload[10], macBuf, sizeof(macBuf));
  fr.mac = String(macBuf);

  if (payload[0] == 0x80) {
    // Beacon: AP advertising — BSSID == src MAC; ESSID len at payload[37],
    // name at payload[38..38+len]
    fr.kind = "beacon";
    macToString(&payload[10], macBuf, sizeof(macBuf));  // BSSID == src for beacons
    fr.bssid = String(macBuf);
    uint8_t essidLen = payload[37];
    if (essidLen > 32) essidLen = 32;
    if (essidLen > 0) {
      String s;
      for (uint8_t i = 0; i < essidLen; i++) {
        char c = (char)payload[38 + i];
        s += (c >= 0x20 && c <= 0x7E) ? c : '?';  // printable only
      }
      fr.ssid = s;
    }
  } else {
    // Probe request: client seeking — requested SSID len at payload[25],
    // name at payload[26..26+len]; BSSID field is empty (we don't chase it)
    fr.kind = "probe_req";
    fr.bssid = "";
    uint8_t essidLen = payload[25];
    if (essidLen > 32) essidLen = 32;
    if (essidLen > 0) {
      String s;
      for (uint8_t i = 0; i < essidLen; i++) {
        char c = (char)payload[26 + i];
        s += (c >= 0x20 && c <= 0x7E) ? c : '?';
      }
      fr.ssid = s;
    }
  }

  // Dedup within the cycle: same kind + mac + ssid => keep the stronger RSSI
  for (uint8_t i = 0; i < g_wifiFrameCount; i++) {
    WifiFrame& e = g_wifiFrames[i];
    if (e.kind == fr.kind && e.mac == fr.mac && e.ssid == fr.ssid) {
      if (fr.rssi > e.rssi) e.rssi = fr.rssi;
      return;
    }
  }
  g_wifiFrames[g_wifiFrameCount++] = fr;
}

// Set channel (Marauder changeChannel() equivalent)
static void wifiSetChannel(uint8_t ch) {
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

// Blocking WiFi join with timeout; returns true when connected.
static bool wifiJoin(const char* ssid, const char* pass) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > MOSAIC_WIFI_RECONNECT_TIMEOUT_MS) {
      Serial.println("  join timeout");
      return false;
    }
    delay(300);
  }
  return true;
}

// One offline scan cycle. Assumes caller holds no other radio activity.
static void runWifiScanCycle() {
  Serial.printf("--- WiFi offline scan (%ds interval) ---\n",
                MOSAIC_WIFI_SCAN_INTERVAL_SECONDS);

  g_wifiFrameCount = 0;
  g_wifiScanActive = true;

  // 1) Drop association, go NULL, enter promiscuous (Marauder setWiFiMode)
  WiFi.disconnect();
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(wifiPromiscuousCallback);

  // 2) Hop channels 1..13, holding each ~80ms (~1s total)
  for (uint8_t ch = 1; ch <= WIFI_SCAN_MAX_CHANNEL; ch++) {
    wifiSetChannel(ch);
    delay(MOSAIC_WIFI_SCAN_CHANNEL_HOLD_MS);
  }

  // 3) Exit promiscuous, back to STA
  esp_wifi_set_promiscuous(false);
  g_wifiScanActive = false;
  WiFi.mode(WIFI_STA);

  // 4) Reconnect with retry loop — a node that can't rejoin is dead.
  //    If join fails we KEEP retrying (outer loop also retries) while the
  //    rest of the sensing cycle (BLE) keeps collecting.
  bool joined = false;
  for (int attempt = 1; attempt <= MOSAIC_WIFI_RECONNECT_RETRIES && !joined; attempt++) {
    Serial.printf("  reconnect attempt %d/%d...\n", attempt, MOSAIC_WIFI_RECONNECT_RETRIES);
    joined = wifiJoin(MOSAIC_WIFI_SSID, MOSAIC_WIFI_PASSWORD);
  }
  if (joined) {
    Serial.printf("  reconnected. IP: %s BSSID: %s\n",
                  WiFi.localIP().toString().c_str(), WiFi.BSSIDstr().c_str());
  } else {
    Serial.println("  reconnect FAILED — will keep retrying on next cycle");
  }

  // 5) Report the batch envelope (only if back online and frames captured)
  if (joined && g_wifiFrameCount > 0) {
    HTTPClient http;
    String url = String("http://") + gatewayHost + ":" + gatewayPort + "/orb/ingest";
    String payload = "{\"v\":1,\"node\":\"" + String(MOSAIC_NODE_NAME) +
                     "\",\"type\":\"wifi\",\"ts\":" + String(millis()) +
                     ",\"payload\":{\"frames\":[";
    for (uint8_t i = 0; i < g_wifiFrameCount; i++) {
      if (i > 0) payload += ",";
      const WifiFrame& fr = g_wifiFrames[i];
      payload += "{\"kind\":\"" + fr.kind + "\",\"mac\":\"" + fr.mac + "\"";
      if (fr.bssid.length() > 0) payload += ",\"bssid\":\"" + fr.bssid + "\"";
      if (fr.ssid.length() > 0) payload += ",\"ssid\":\"" + jsonEscape(fr.ssid) + "\"";
      payload += ",\"channel\":" + String(fr.channel) +
                 ",\"rssi\":" + String(fr.rssi) + "}";
    }
    payload += "]}}";

    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(payload);
    Serial.printf("  wifi batch POST: %d (%d frames)\n", httpCode, g_wifiFrameCount);
    http.end();
  }
  g_wifiFrameCount = 0;
}

// Mosaic protocol v1 — location is LEARNED, not claimed.
// The firmware reports the real BSSID of the AP it's connected to;
// the brain maps BSSID → label ("home", "gym", ...). See docs/protocol.md.

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== MOSAIC SENSE ENGINE (NimBLE passive) ===\n");

  // Controller-level duplicate filter: DEVICE mode + 200-entry cache.
  // Must be set BEFORE NimBLEDevice::init (same pattern as ESP32Marauder).
  NimBLEDevice::setScanFilterMode(CONFIG_BTDM_SCAN_DUPL_TYPE_DEVICE);
  NimBLEDevice::setScanDuplicateCacheSize(MOSAIC_BLE_DUPLICATE_CACHE_SIZE);
  NimBLEDevice::init("");

  pBLEScan = NimBLEDevice::getScan();
  // wantDuplicates=false → controller filters repeats (one callback/device/window)
  pBLEScan->setScanCallbacks(new BLEPassiveScanCallback(), false);
  pBLEScan->setActiveScan(true);  // fetch names via scan requests (standard BLE, no attacks)
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setMaxResults(0);     // unlimited (we cap in software: MAX_DEVICES)

  // Connect WiFi (for gateway POST)
  Serial.printf("Connecting to %s...\n", MOSAIC_WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(MOSAIC_WIFI_SSID, MOSAIC_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected. IP: %s BSSID: %s\n",
                WiFi.localIP().toString().c_str(), WiFi.BSSIDstr().c_str());
}

void loop() {
  // Scan for 5 seconds (blocking — callbacks fill g_records meanwhile)
  Serial.println("\n--- BLE scan (5s) ---");
  g_recordCount = 0;
  pBLEScan->getResults(5000, false);
  int count = g_recordCount;
  Serial.printf("Found %d devices\n", count);

  // Report to gateway — Mosaic protocol v1 envelope with real BSSID
  if (count > 0 && WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String("http://") + gatewayHost + ":" + gatewayPort + "/orb/ingest";
    String bssid = WiFi.BSSIDstr();
    String payload = "{\"v\":1,\"node\":\"" + String(MOSAIC_NODE_NAME) +
                     "\",\"type\":\"scan\",\"ts\":" + String(millis()) +
                     ",\"payload\":{\"ap_bssid\":\"" + bssid +
                     "\",\"devices\":[";
    for (int i = 0; i < count; i++) {
      if (i > 0) payload += ",";
      payload += recordToJson(g_records[i]);
    }
    payload += "]}}";

    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(payload);
    Serial.printf("Gateway POST: %d\n", httpCode);
    http.end();
  }

#if MOSAIC_WIFI_SCAN_ENABLE
  // WiFi offline scan cycle — sequential with BLE (one radio, phases never
  // overlap). Every MOSAIC_WIFI_SCAN_INTERVAL_SECONDS: drop WiFi, sniff
  // channels 1..13, reconnect, report the batch.
  static unsigned long lastWifiScanMs = 0;
  unsigned long now = millis();
  if (lastWifiScanMs == 0 ||
      (now - lastWifiScanMs) >= (unsigned long)MOSAIC_WIFI_SCAN_INTERVAL_SECONDS * 1000UL) {
    if (WiFi.status() == WL_CONNECTED) {
      runWifiScanCycle();
      lastWifiScanMs = millis();
    } else {
      // Still offline from a failed cycle — keep retrying the join while
      // BLE keeps collecting. A node that can't rejoin WiFi is dead.
      Serial.println("--- WiFi down — retrying join (BLE keeps running) ---");
      wifiJoin(MOSAIC_WIFI_SSID, MOSAIC_WIFI_PASSWORD);
      if (WiFi.status() == WL_CONNECTED) lastWifiScanMs = millis();
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
  delay(MOSAIC_SCAN_INTERVAL);
}
