/**
 * ESP32-Mosaic — ARP neighbor discovery (normal WiFi client behavior)
 *
 * ESP-IDF port of firmware/src/arp_neighbors.h (Arduino). Same logic,
 * same lwIP etharp calls, same envelopes — only the framework glue was
 * swapped (WiFi/HTTPClient/String -> sense_wifi_* accessors + esp_http
 * via sense_common.h). See the Arduino original for the design notes.
 *
 *   - Every MOSAIC_ARP_INTERVAL_SECONDS it ARP-probes the local subnet
 *     (scan range derived from the live IP + netmask; example subnet
 *     constants below are the documented default shape).
 *   - A seen-table tracks MAC→IP pairs with first/last-seen.
 *   - A MAC that appears → type:"arp_join" envelope to the gateway.
 *   - A MAC unseen for MOSAIC_ARP_LEAVE_TIMEOUT_INTERVALS consecutive
 *     cycles → type:"arp_leave" envelope.
 *
 * Client isolation: some APs isolate clients so ARP probes get no
 * answers. That is fine — the sweep simply finds no neighbors.
 *
 * Implementation: lwIP's etharp — the same stack the WiFi driver already
 * runs. etharp_request() sends each probe; etharp_find_addr() reads the
 * answers. The etharp calls are wrapped in LOCK_TCPIP_CORE() so they are
 * safe from the sense task (the macros are no-ops when core locking is
 * disabled in lwIP config).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Configuration (defaults ON — override in include/config.h)
// ---------------------------------------------------------------------------
#ifndef MOSAIC_ARP_ENABLE
#define MOSAIC_ARP_ENABLE 1
#endif

// How often (seconds) the node probes the subnet for neighbors.
#ifndef MOSAIC_ARP_INTERVAL_SECONDS
#define MOSAIC_ARP_INTERVAL_SECONDS 60
#endif

// Time (ms) to wait for ARP replies after each request batch.
#ifndef MOSAIC_ARP_RESPONSE_WINDOW_MS
#define MOSAIC_ARP_RESPONSE_WINDOW_MS 250
#endif

// Pacing between requests inside a batch (ms) — keeps the sweep polite.
#ifndef MOSAIC_ARP_REQUEST_PACING_MS
#define MOSAIC_ARP_REQUEST_PACING_MS 5
#endif

// A MAC unseen for this many consecutive cycles emits an arp_leave event.
#ifndef MOSAIC_ARP_LEAVE_TIMEOUT_INTERVALS
#define MOSAIC_ARP_LEAVE_TIMEOUT_INTERVALS 3
#endif

// Seen-table size cap (oldest entry evicted when full).
#ifndef MOSAIC_ARP_MAX_NEIGHBORS
#define MOSAIC_ARP_MAX_NEIGHBORS 32
#endif

// Example subnet (documented default shape). The sweep normally derives the
// range from the node's live IP + netmask at runtime; these constants are
// the fallback when the live mask is unusual and the node's IP falls inside
// this example network.
#ifndef MOSAIC_ARP_SCAN_NETWORK
#define MOSAIC_ARP_SCAN_NETWORK "192.168.1.0"
#endif
#ifndef MOSAIC_ARP_SCAN_MASK
#define MOSAIC_ARP_SCAN_MASK "255.255.255.0"
#endif

/* Call from the sense task: runs the sweep at
 * MOSAIC_ARP_INTERVAL_SECONDS cadence. While WiFi is down the timer does
 * not advance, so the sense-loop cadence retries until back online. */
void arpNeighborsLoop(void);
