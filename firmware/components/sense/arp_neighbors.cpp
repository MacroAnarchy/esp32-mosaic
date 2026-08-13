/*
 * ARP neighbor discovery — ESP-IDF port (see arp_neighbors.h).
 *
 * Ported from firmware/src/arp_neighbors.h (Arduino). All logic is
 * identical; only the framework glue differs:
 *   WiFi.status()/localIP()/subnetMask()/BSSIDstr() -> sense_wifi_* accessors
 *   HTTPClient + String payload            -> sense::post_json (std::string)
 *   millis()/delay()                       -> sense::uptime_ms()/vTaskDelay
 *   Serial.printf                          -> ESP_LOGI
 */

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"  // esp_netif_get_netif_impl
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/etharp.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"  // LOCK_TCPIP_CORE / UNLOCK_TCPIP_CORE

#include "config.h"

#include "arp_neighbors.h"
#include "sense_common.h"
#include "sense_engine.h"

static const char *TAG = "arp";

// lwIP keeps a small ARP cache (ARP_TABLE_SIZE, default 10) — sweep in
// batches that fit so every probe's answer is still cached when we read.
#define ARP_NEIGHBOR_BATCH_SIZE 10
#define ARP_MAC_STR_LEN 18   // "aa:bb:cc:dd:ee:ff" + NUL
#define ARP_IP_STR_LEN 16    // "255.255.255.255" + NUL

// One tracked neighbor.
struct ArpNeighbor {
  uint8_t mac[6];
  char ip[ARP_IP_STR_LEN];
  uint32_t firstSeenMs;  // uptime when first sighted
  uint32_t lastSeenMs;   // uptime of the most recent sighting
  uint8_t missedCycles;  // consecutive cycles without a sighting
};

static ArpNeighbor g_arpNeighbors[MOSAIC_ARP_MAX_NEIGHBORS];
static uint8_t g_arpNeighborCount = 0;

// POST one Mosaic protocol v1 envelope for an ARP join/leave event.
//   {"v":1,"node":"node-01","type":"arp_join","ts":...,
//    "payload":{"ap_bssid":"...","mac":"aa:bb:cc:dd:ee:ff","ip":"192.168.1.42",
//               "first_seen":...,"last_seen":...}}
static void arpPostEvent(const char *type, const uint8_t *mac, const char *ip,
                         uint32_t firstSeenMs, uint32_t lastSeenMs) {
  char macStr[ARP_MAC_STR_LEN];
  sense::mac_to_str(mac, macStr, sizeof(macStr));
  char bssid[ARP_MAC_STR_LEN] = "";
  sense_wifi_get_ap_bssid(bssid, sizeof(bssid));

  std::string url = std::string("http://") + MOSAIC_GATEWAY_HOST + ":" +
                    std::to_string(MOSAIC_GATEWAY_PORT) + "/orb/ingest";
  std::string payload = std::string("{\"v\":1,\"node\":\"") + MOSAIC_NODE_NAME +
                        "\",\"type\":\"" + type + "\",\"ts\":" +
                        std::to_string(sense::uptime_ms()) +
                        ",\"payload\":{\"ap_bssid\":\"" + bssid +
                        "\",\"mac\":\"" + macStr + "\",\"ip\":\"" + ip +
                        "\",\"first_seen\":" + std::to_string(firstSeenMs) +
                        ",\"last_seen\":" + std::to_string(lastSeenMs) + "}}";
  int code = sense::post_json(url, payload);
  ESP_LOGI(TAG, "%s POST %d (mac=%s ip=%s)", type, code, macStr, ip);
}

// Record a MAC sighting at ip. Emits arp_join on the first sighting.
static void arpSighting(const uint8_t *mac, const char *ip, uint32_t nowMs) {
  for (uint8_t i = 0; i < g_arpNeighborCount; i++) {
    ArpNeighbor &nb = g_arpNeighbors[i];
    if (memcmp(nb.mac, mac, 6) == 0) {
      if (strcmp(nb.ip, ip) != 0) {  // DHCP renumbering — follow the MAC
        char macStr[ARP_MAC_STR_LEN];
        sense::mac_to_str(mac, macStr, sizeof(macStr));
        ESP_LOGI(TAG, "%s moved %s -> %s (same device, new lease)",
                 macStr, nb.ip, ip);
        snprintf(nb.ip, ARP_IP_STR_LEN, "%s", ip);
      }
      nb.lastSeenMs = nowMs;
      nb.missedCycles = 0;
      return;
    }
  }
  if (g_arpNeighborCount >= MOSAIC_ARP_MAX_NEIGHBORS) {
    // Table full — evict the entry idle longest (oldest lastSeen).
    uint8_t oldest = 0;
    for (uint8_t i = 1; i < g_arpNeighborCount; i++)
      if (g_arpNeighbors[i].lastSeenMs < g_arpNeighbors[oldest].lastSeenMs)
        oldest = i;
    g_arpNeighborCount--;
    for (uint8_t i = oldest; i < g_arpNeighborCount; i++)
      g_arpNeighbors[i] = g_arpNeighbors[i + 1];
  }
  ArpNeighbor &nb = g_arpNeighbors[g_arpNeighborCount++];
  memcpy(nb.mac, mac, 6);
  snprintf(nb.ip, ARP_IP_STR_LEN, "%s", ip);
  nb.firstSeenMs = nowMs;
  nb.lastSeenMs = nowMs;
  nb.missedCycles = 0;
  arpPostEvent("arp_join", mac, ip, nowMs, nowMs);
}

// Emit arp_leave for neighbors past the timeout, then drop them.
static void arpExpire() {
  for (int8_t i = g_arpNeighborCount - 1; i >= 0; i--) {
    ArpNeighbor &nb = g_arpNeighbors[i];
    if (nb.missedCycles >= MOSAIC_ARP_LEAVE_TIMEOUT_INTERVALS) {
      arpPostEvent("arp_leave", nb.mac, nb.ip, nb.firstSeenMs, nb.lastSeenMs);
      g_arpNeighborCount--;
      for (uint8_t j = i; j < g_arpNeighborCount; j++)
        g_arpNeighbors[j] = g_arpNeighbors[j + 1];
    }
  }
}

// Parse "a.b.c.d" into octets; returns false on garbage.
static bool arpParseIpv4(const char *s, uint8_t out[4]) {
  unsigned a, b, c, d;
  if (sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
  if (a > 255 || b > 255 || c > 255 || d > 255) return false;
  out[0] = (uint8_t)a; out[1] = (uint8_t)b;
  out[2] = (uint8_t)c; out[3] = (uint8_t)d;
  return true;
}

// One sweep cycle. Returns true when a sweep actually ran (advances the
// interval timer), false when skipped (WiFi down / no lease / unsupported
// subnet) — the caller then retries on the next loop pass.
static bool arpRunCycle() {
  if (!sense_wifi_is_connected()) return false;
  uint8_t local[4], mask[4];
  if (!sense_wifi_get_ipv4(local, mask)) return false;
  if (local[0] == 0) return false;  // no lease yet

  esp_netif_t *espNetif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (!espNetif) return false;
  struct netif *n = (struct netif *)esp_netif_get_netif_impl(espNetif);
  if (!n) return false;

  // Scan range = the node's own subnet. Typical home/office LANs are /24
  // (host bits in the last octet). For anything else, fall back to the
  // example subnet constants only when the node's own IP lives inside them;
  // otherwise skip with a single warning (graceful, no error spam).
  uint8_t baseA = local[0] & mask[0];
  uint8_t baseB = local[1] & mask[1];
  uint8_t baseC = local[2] & mask[2];
  if (mask[0] != 0xFF || mask[1] != 0xFF || mask[2] != 0xFF) {
    uint8_t cfg[4], cfgMask[4];
    if (arpParseIpv4(MOSAIC_ARP_SCAN_NETWORK, cfg) &&
        arpParseIpv4(MOSAIC_ARP_SCAN_MASK, cfgMask) &&
        (local[0] & cfgMask[0]) == cfg[0] &&
        (local[1] & cfgMask[1]) == cfg[1] &&
        (local[2] & cfgMask[2]) == cfg[2] &&
        (local[3] & cfgMask[3]) == cfg[3]) {
      baseA = cfg[0]; baseB = cfg[1]; baseC = cfg[2];
      mask[0] = cfgMask[0]; mask[1] = cfgMask[1];
      mask[2] = cfgMask[2]; mask[3] = cfgMask[3];
    } else {
      static bool warned = false;
      if (!warned) {
        warned = true;
        ESP_LOGW(TAG, "subnet mask %u.%u.%u.%u outside the example shape — "
                      "sweep skipped", mask[0], mask[1], mask[2], mask[3]);
      }
      return false;
    }
  }
  uint8_t firstD = (local[3] & mask[3]) + 1;         // skip network address
  uint8_t lastD = (local[3] & mask[3]) | (~mask[3]); // broadcast address
  if (firstD > lastD) return false;                  // degenerate /32

  // Age every tracked neighbor; sightings below refresh them.
  for (uint8_t i = 0; i < g_arpNeighborCount; i++) g_arpNeighbors[i].missedCycles++;

  // Sweep the subnet in batches that fit lwIP's ARP cache.
  uint8_t d = firstD;
  uint8_t alive = 0;
  bool done = false;
  while (!done) {
    uint8_t batch[ARP_NEIGHBOR_BATCH_SIZE][4];
    uint8_t batchCount = 0;
    while (batchCount < ARP_NEIGHBOR_BATCH_SIZE) {
      bool isSelf = (d == local[3] && baseA == local[0] &&
                     baseB == local[1] && baseC == local[2]);
      if (!isSelf) {
        batch[batchCount][0] = baseA;
        batch[batchCount][1] = baseB;
        batch[batchCount][2] = baseC;
        batch[batchCount][3] = d;
        batchCount++;
      }
      if (d == lastD) { done = true; break; }
      d++;
    }
    if (batchCount == 0) break;  // only our own address on the subnet

    // Probe the batch (normal ARP traffic — every station does this).
    for (uint8_t i = 0; i < batchCount; i++) {
      ip4_addr_t target;
      IP4_ADDR(&target, batch[i][0], batch[i][1], batch[i][2], batch[i][3]);
      LOCK_TCPIP_CORE();
      etharp_request(n, &target);
      UNLOCK_TCPIP_CORE();
      vTaskDelay(pdMS_TO_TICKS(MOSAIC_ARP_REQUEST_PACING_MS));
    }
    // Give the replies time to land in lwIP's ARP cache.
    uint32_t waitStart = sense::uptime_ms();
    while (sense::uptime_ms() - waitStart < (uint32_t)MOSAIC_ARP_RESPONSE_WINDOW_MS)
      vTaskDelay(pdMS_TO_TICKS(2));

    // Read the answers. etharp_find_addr only reports entries whose MAC is
    // actually known — unanswered probes (host absent, or client isolation)
    // stay pending and are simply not seen.
    for (uint8_t i = 0; i < batchCount; i++) {
      ip4_addr_t target;
      IP4_ADDR(&target, batch[i][0], batch[i][1], batch[i][2], batch[i][3]);
      struct eth_addr *eth = NULL;
      const ip4_addr_t *ipRet = NULL;
      LOCK_TCPIP_CORE();
      int found = etharp_find_addr(n, &target, &eth, &ipRet);
      UNLOCK_TCPIP_CORE();
      if (found >= 0 && eth) {
        char ipStr[ARP_IP_STR_LEN];
        snprintf(ipStr, sizeof(ipStr), "%u.%u.%u.%u",
                 batch[i][0], batch[i][1], batch[i][2], batch[i][3]);
        arpSighting(eth->addr, ipStr, sense::uptime_ms());
        alive++;
      }
    }
  }

  arpExpire();
  ESP_LOGI(TAG, "sweep done: %u alive, %u tracked",
           (unsigned)alive, (unsigned)g_arpNeighborCount);
  return true;
}

// Call from the sense task: runs the sweep at MOSAIC_ARP_INTERVAL_SECONDS
// cadence. While WiFi is down the timer does not advance, so the sense-loop
// cadence (BLE scan interval) retries until the node is back online.
void arpNeighborsLoop() {
  static uint32_t lastRunMs = 0;
  uint32_t now = sense::uptime_ms();
  if (lastRunMs != 0 &&
      (now - lastRunMs) < (uint32_t)MOSAIC_ARP_INTERVAL_SECONDS * 1000UL)
    return;
  if (arpRunCycle()) lastRunMs = sense::uptime_ms();
}
