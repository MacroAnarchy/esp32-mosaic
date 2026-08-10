/**
 * orb-sense-test — Orb Sense Engine prototype on M5Stack (NODE-01 testbed)
 *
 * TESTS the core presence-detection concept for the Orb:
 * 1. BLE scan → collect device MACs + RSSI + names
 * 2. HTTP POST → report device list to XPS gateway endpoint
 * 3. Runs in a loop, printing results to serial
 *
 * This is a TEST FIRMWARE for the NODE-01 hardware (M5Stack Fire ESP32).
 * BALA rover firmware is untouched in git (~/NODE-01) and restored after.
 *
 * Build: pio run -e m5stack-grey
 * Flash: pio run -e m5stack-grey --target upload  (USB, /dev/ttyUSB0)
 */

#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <HTTPClient.h>

// Scan results callback
class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("BLE: %s | RSSI: %d dBm | %s\n",
      advertisedDevice.getAddress().toString().c_str(),
      advertisedDevice.getRSSI(),
      advertisedDevice.haveName() ? advertisedDevice.getName().c_str() : "(no name)");
  }
};

// Gateway endpoint (set via build flags in platformio.ini / platformio.local.ini)
#ifndef MOSAIC_GATEWAY_HOST
#define MOSAIC_GATEWAY_HOST "192.168.1.10"
#endif
#ifndef MOSAIC_GATEWAY_PORT
#define MOSAIC_GATEWAY_PORT 9000
#endif
const char* gatewayHost = MOSAIC_GATEWAY_HOST;
const int gatewayPort = MOSAIC_GATEWAY_PORT;

// ORB protocol v1 — location is LEARNED, not claimed.
// The firmware reports the real BSSID of the AP it's connected to;
// the XPS brain maps BSSID → label ("home", "gym", ...).
// See ~/.orb/protocol.md

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ORB SENSE ENGINE TEST ===\n");

  // Init BLE
  BLEDevice::init("OrbSenseTest");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCallback());
  pBLEScan->setActiveScan(true);    // request names (active scan)
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  // Note: no setMaxResults in this BLE lib version — scan returns all by default

  // Connect WiFi
  Serial.printf("Connecting to %s...\n", MOSAIC_WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(MOSAIC_WIFI_SSID, MOSAIC_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected. IP: %s BSSID: %s\n", WiFi.localIP().toString().c_str(), WiFi.BSSIDstr().c_str());
}

void loop() {
  // Scan for 5 seconds
  Serial.println("\n--- BLE scan (5s) ---");
  BLEScan* pBLEScan = BLEDevice::getScan();
  BLEScanResults foundDevices = pBLEScan->start(5, false);
  int count = foundDevices.getCount();
  Serial.printf("Found %d devices\n", count);

  // Report to gateway — ORB protocol v1 envelope with real BSSID
  if (count > 0 && WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String("http://") + gatewayHost + ":" + gatewayPort + "/orb/ingest";
    String bssid = WiFi.BSSIDstr();
    String payload = "{\"v\":1,\"node\":\"node-01\",\"type\":\"scan\",\"ts\":" + String(millis()) +
                     ",\"payload\":{\"ap_bssid\":\"" + bssid +
                     "\",\"devices\":[";
    for (int i = 0; i < count; i++) {
      BLEAdvertisedDevice dev = foundDevices.getDevice(i);
      if (i > 0) payload += ",";
      payload += "{\"mac\":\"" + String(dev.getAddress().toString().c_str()) + "\",\"rssi\":" + String(dev.getRSSI());
      if (dev.haveName()) payload += ",\"name\":\"" + String(dev.getName().c_str()) + "\"";
      payload += "}";
    }
    payload += "]}}";

    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(payload);
    Serial.printf("Gateway POST: %d\n", httpCode);
    http.end();
  }

  // 10s pause between scans
  delay(10000);
}
