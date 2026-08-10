/**
 * ESP32-Mosaic — BLE sense engine (NimBLE-Arduino, passive)
 *
 * Passive BLE presence detection for the Orb/NODE-01 swarm:
 *   1. BLE scan (controller-level dedup, DEVICE mode / 200-entry cache)
 *   2. AD payload parsing: manufacturer data (company_id), service UUIDs,
 *      service data UUID
 *   3. Device classification: "findmy" | "meta" | "flipper" | "unknown"
 *   4. HTTP POST → gateway /orb/ingest (ORB protocol v1 envelope)
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
#include <vector>
#include <string>

// Local config (gitignored — copy from include/config.example.h)
#include "config.h"

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

// WiFi credentials: legacy MOSAIC_* names, with MOSAIC_* fallback so a config.h
// copied straight from config.example.h compiles too.
#ifndef MOSAIC_WIFI_SSID
#define MOSAIC_WIFI_SSID MOSAIC_WIFI_SSID
#define MOSAIC_WIFI_PASSWORD MOSAIC_WIFI_PASSWORD
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

// ORB protocol v1 — location is LEARNED, not claimed.
// The firmware reports the real BSSID of the AP it's connected to;
// the brain maps BSSID → label ("home", "gym", ...). See docs/protocol.md.

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ORB SENSE ENGINE (NimBLE passive) ===\n");

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

  // Report to gateway — ORB protocol v1 envelope with real BSSID
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

  // Pause between scans (configurable; default 15s per config.example.h)
  delay(MOSAIC_SCAN_INTERVAL);
}
