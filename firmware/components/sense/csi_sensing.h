/*
 * csi_sensing: WiFi CSI motion + presence sensing (Tier 1) for the Mosaic
 * sense engine.
 *
 * Monostatic geometry: the node is an ordinary WiFi STA associated with its
 * home router, and its radio captures Channel State Information from the
 * router's traffic (data/ACK/beacon frames). The Espressif esp_wifi_sensing
 * component (Apache-2.0, Espressif Systems — see
 * https://github.com/espressif/esp-csi) converts that CSI stream into
 * motion start/stop events per peer channel. We register ONE channel: the
 * connected AP's BSSID (the router). Router ping keeps the sampling path
 * fed with traffic.
 *
 * Scope (Tier 1): motion events (moved/empty) + periodic feature snapshots
 * (event:"feature", ~5s cadence) carrying presence + waveform metrics.
 * An ACTIVE transition means radio-visible movement on the channel;
 * INACTIVE means the channel went quiet. Presence ("presence_someone")
 * comes from the component's calibrated wander channel — it only becomes
 * meaningful after the auto-calibration (train) window completes at boot
 * (the room should stay static for ~20s). No vitals, no activity
 * classification.
 *
 * Events are queued (the FSM owns a background task; the callback must be
 * non-blocking) and drained by the sense task, which POSTs one
 * type:"csi" envelope per event to the gateway — reusing the existing
 * gateway envelope channel (no new sockets, no UDP). Feature snapshots
 * are posted from the same sense loop (it cycles at ~20s, comfortably
 * inside the channel-liveness window) so POSTs never collide with the
 * BLE scan or the WiFi offline-scan cycle.
 *
 * Radio coexistence: CSI runs on the same STA association as the normal
 * WiFi client (the router-based Espressif examples do exactly this).
 * During the WiFi offline scan cycle the node leaves STA mode, so the
 * sense task calls csi_sensing_pause() before the drop and
 * csi_sensing_resume() after reconnection (the FSM relearns its baseline).
 * BLE scanning shares the 2.4 GHz radio via coexistence arbitration and
 * may thin the CSI sample stream slightly — acceptable for Tier 1.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Feature flag (default ON). Defined here so both this module and the
 * sense engine can gate on it; config.h (included before this header)
 * may override it.
 */
#ifndef MOSAIC_CSI_ENABLE
#define MOSAIC_CSI_ENABLE 1
#endif

/* Create the sensing FSM with one channel (the AP BSSID) and start
 * processing. Requires the STA association (for the AP BSSID and the
 * gateway used by ping-assisted sampling). Safe to call once; returns
 * ESP_OK when already running. ESP_ERR_INVALID_STATE when WiFi is not
 * associated yet — callers may retry later. */
esp_err_t csi_sensing_init(void);

/* Stop the FSM + router ping (radio leaves STA mode). No-op when idle. */
void csi_sensing_pause(void);

/* Restart the FSM + router ping. Re-targets the channel if the node
 * roamed to a different AP. No-op when running. */
void csi_sensing_resume(void);

/* Drain queued motion events and POST one type:"csi" envelope per event.
 * Drops events when the gateway is unreachable (lossy by design — the
 * next event is one motion transition away). Returns the number posted. */
int csi_sensing_drain_and_report(void);

#ifdef __cplusplus
}
#endif
