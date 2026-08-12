/*
 * ESP32-Mosaic — WiFi CSI motion sensing (Tier 1), monostatic router
 * geometry. Wraps the Espressif esp_wifi_sensing component (Apache-2.0):
 * one FSM channel = the connected AP's BSSID. ACTIVE/INACTIVE transitions
 * become type:"csi" envelopes to the gateway.
 *
 * See csi_sensing.h for the design notes and scope statement.
 */

#include <cstdio>
#include <cstring>
#include <inttypes.h>
#include <string>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_wifi_sensing.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Local config (gitignored — copy from include/config.example.h).
#include "config.h"

#include "sense_common.h"
#include "sense_engine.h"

#include "csi_sensing.h"

// ---- Tuning knobs (defaults match the esp_wifi_sensing Kconfig defaults;
// override in config.h) ----
#ifndef MOSAIC_CSI_PING_FREQUENCY_HZ
#define MOSAIC_CSI_PING_FREQUENCY_HZ 100  // router ping Hz feeding the CSI path
#endif
#ifndef MOSAIC_CSI_SENSITIVITY
#define MOSAIC_CSI_SENSITIVITY 500  // x1000 scale, 1..1000 (0.5 = component default)
#endif

static const char *TAG = "csi";

static esp_wifi_sensing_fsm_handle_t s_fsm = NULL;
static uint8_t s_peerMac[6] = {0};  // AP BSSID — the single sensing channel
static bool s_running = false;

// ---------------------------------------------------------------------
// Event queue — the FSM's event callback runs on the component's own
// background task, so it must never block. Events are queued here and
// drained by the sense task, which does the HTTP POST.
// ---------------------------------------------------------------------
#define CSI_QUEUE_CAP 8

struct CsiQueueEntry {
  uint8_t event;  // ESP_WIFI_SENSING_FSM_EVENT_ACTIVE / _INACTIVE
  uint32_t score; // smoothed feature value (component passes it as `data`)
  uint8_t peer[6];
};

static CsiQueueEntry s_queue[CSI_QUEUE_CAP];
static volatile uint8_t s_qHead = 0;
static volatile uint8_t s_qCount = 0;
static portMUX_TYPE s_qMux = portMUX_INITIALIZER_UNLOCKED;

static bool queuePush(const CsiQueueEntry &e) {
  bool pushed = false;
  portENTER_CRITICAL(&s_qMux);
  if (s_qCount < CSI_QUEUE_CAP) {
    s_queue[(s_qHead + s_qCount) % CSI_QUEUE_CAP] = e;
    s_qCount = (uint8_t)(s_qCount + 1);  // no ++ on volatile (deprecated)
    pushed = true;
  }
  portEXIT_CRITICAL(&s_qMux);
  return pushed;
}

static bool queuePop(CsiQueueEntry &e) {
  bool popped = false;
  portENTER_CRITICAL(&s_qMux);
  if (s_qCount > 0) {
    e = s_queue[s_qHead];
    s_qHead = (uint8_t)((s_qHead + 1) % CSI_QUEUE_CAP);
    s_qCount = (uint8_t)(s_qCount - 1);  // no -- on volatile (deprecated)
    popped = true;
  }
  portEXIT_CRITICAL(&s_qMux);
  return popped;
}

// ---------------------------------------------------------------------
// FSM event callback — runs on the esp_wifi_sensing background task.
// Non-blocking: queue only. Events are lossy by design (queue full →
// drop; the next transition is one motion event away).
// ---------------------------------------------------------------------
static void onMotionEvent(esp_wifi_sensing_fsm_handle_t handle,
                          const uint8_t peer_mac[6],
                          esp_wifi_sensing_fsm_event_t event,
                          uint32_t data,
                          void *user_data) {
  (void)handle;
  (void)user_data;

  CsiQueueEntry e;
  e.event = (uint8_t)event;
  e.score = data;
  memcpy(e.peer, peer_mac, 6);
  if (!queuePush(e)) {
    ESP_LOGW(TAG, "event queue full — dropping");
    return;
  }
  ESP_LOGI(TAG, "%s peer=" MACSTR " score=%" PRIu32,
           (event == ESP_WIFI_SENSING_FSM_EVENT_ACTIVE) ? "ACTIVE" : "INACTIVE",
           MAC2STR(peer_mac), data);
}

// ---------------------------------------------------------------------
// Envelope producer — one type:"csi" envelope per queued event.
// ---------------------------------------------------------------------
static std::string fmtFloat(float v) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%.6g", (double)v);
  return std::string(buf);
}

static int postCsiEvent(const CsiQueueEntry &e) {
  const bool active = (e.event == ESP_WIFI_SENSING_FSM_EVENT_ACTIVE);

  // Wander/jitter are the latest waveform metrics (best effort snapshot;
  // diagnostics are per-channel state, read from the sense task).
  float wander = 0.0f;
  float jitter = 0.0f;
  esp_wifi_sensing_fsm_channel_diag_t diag;
  if (s_fsm && esp_wifi_sensing_fsm_get_channel_diag(s_fsm, e.peer, &diag) == ESP_OK) {
    wander = diag.wander_value;
    jitter = diag.jitter_value;
  }

  // Tier 1 honesty: "someone" means radio-visible motion on the channel
  // (moved=true). Verified stationary presence is a later tier — not
  // claimed here.
  std::string payload = "{\"v\":1,\"node\":\"" + std::string(MOSAIC_NODE_NAME) +
                        "\",\"type\":\"csi\",\"ts\":" +
                        std::to_string(sense::uptime_ms()) +
                        ",\"payload\":{\"event\":\"" +
                        (active ? "moved" : "empty") +
                        "\",\"someone\":" + (active ? "true" : "false") +
                        ",\"moved\":" + (active ? "true" : "false") +
                        ",\"wander\":" + fmtFloat(wander) +
                        ",\"jitter\":" + fmtFloat(jitter) +
                        ",\"score\":" + std::to_string(e.score) + "}}";

  int code = sense::post_json(sense::gateway_ingest_url(), payload);
  ESP_LOGI(TAG, "POST %s: %d", active ? "moved" : "empty", code);
  return code;
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------
esp_err_t csi_sensing_init(void) {
  if (s_fsm) return ESP_OK;  // already created

  // Need the STA association for the AP BSSID (the sensing channel) and
  // the gateway (ping-assisted sampling).
  wifi_ap_record_t ap = {};
  if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
    ESP_LOGW(TAG, "not associated yet — CSI init deferred");
    return ESP_ERR_INVALID_STATE;
  }
  memcpy(s_peerMac, ap.bssid, 6);

  esp_wifi_sensing_fsm_config_t cfg = DEFAULT_ESP_WIFI_SENSING_FSM_CONFIG();
  cfg.max_channel_num = 1;  // monostatic: one channel = the router
  cfg.ping_frequency_hz = MOSAIC_CSI_PING_FREQUENCY_HZ;
  cfg.default_channel_config.sensitivity = (float)MOSAIC_CSI_SENSITIVITY / 1000.0f;

  esp_err_t ret = esp_wifi_sensing_fsm_create(&cfg, &s_fsm);
  if (ret != ESP_OK) {
    s_fsm = NULL;
    ESP_LOGE(TAG, "fsm create failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_wifi_sensing_fsm_add_channel(s_fsm, s_peerMac);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "add channel failed: %s", esp_err_to_name(ret));
    esp_wifi_sensing_fsm_delete(s_fsm);
    s_fsm = NULL;
    return ret;
  }

  ret = esp_wifi_sensing_fsm_register_event_cb(s_fsm,
                                               ESP_WIFI_SENSING_FSM_EVENT_ACTIVE,
                                               onMotionEvent, NULL);
  if (ret == ESP_OK) {
    ret = esp_wifi_sensing_fsm_register_event_cb(s_fsm,
                                                 ESP_WIFI_SENSING_FSM_EVENT_INACTIVE,
                                                 onMotionEvent, NULL);
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "register cb failed: %s", esp_err_to_name(ret));
    esp_wifi_sensing_fsm_delete(s_fsm);
    s_fsm = NULL;
    return ret;
  }

  ESP_ERROR_CHECK(esp_wifi_sensing_fsm_control(s_fsm,
                                               ESP_WIFI_SENSING_FSM_CTRL_START,
                                               NULL));
  s_running = true;
  ESP_LOGI(TAG, "CSI sensing started on AP " MACSTR " (ping %u Hz, sens %.2f)",
           MAC2STR(s_peerMac), (unsigned)cfg.ping_frequency_hz,
           (double)cfg.default_channel_config.sensitivity);

  esp_err_t perr = esp_wifi_sensing_fsm_ping_router_start(s_fsm);
  if (perr == ESP_OK) {
    ESP_LOGI(TAG, "router ping started");
  } else if (perr == ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "router ping deferred: STA or gateway not ready");
  } else {
    ESP_LOGE(TAG, "router ping start failed: %s", esp_err_to_name(perr));
  }
  return ESP_OK;
}

void csi_sensing_pause(void) {
  if (!s_fsm || !s_running) return;
  esp_wifi_sensing_fsm_control(s_fsm, ESP_WIFI_SENSING_FSM_CTRL_STOP, NULL);
  esp_wifi_sensing_fsm_ping_router_stop(s_fsm);
  s_running = false;
  ESP_LOGI(TAG, "CSI paused (leaving STA mode)");
}

void csi_sensing_resume(void) {
  if (!s_fsm || s_running) return;

  // Re-target the channel if the node roamed to a different AP.
  wifi_ap_record_t ap = {};
  if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK && memcmp(ap.bssid, s_peerMac, 6) != 0) {
    esp_wifi_sensing_fsm_remove_channel(s_fsm, s_peerMac);
    memcpy(s_peerMac, ap.bssid, 6);
    esp_err_t ret = esp_wifi_sensing_fsm_add_channel(s_fsm, s_peerMac);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "re-add channel failed: %s", esp_err_to_name(ret));
      return;
    }
    ESP_LOGI(TAG, "AP changed — re-targeted channel to " MACSTR, MAC2STR(s_peerMac));
  }

  // START also relearns the channel baseline — right after a reconnect
  // that is exactly what we want (the room state may have changed).
  ESP_ERROR_CHECK(esp_wifi_sensing_fsm_control(s_fsm,
                                               ESP_WIFI_SENSING_FSM_CTRL_START,
                                               NULL));
  s_running = true;
  ESP_LOGI(TAG, "CSI resumed (baseline relearning)");

  esp_err_t perr = esp_wifi_sensing_fsm_ping_router_start(s_fsm);
  if (perr != ESP_OK && perr != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "router ping restart failed: %s", esp_err_to_name(perr));
  }
}

int csi_sensing_drain_and_report(void) {
  if (!s_fsm) return 0;
  int posted = 0;
  CsiQueueEntry e;
  while (queuePop(e)) {
    // Lossy by design: drop the event when the gateway is unreachable —
    // the next motion transition is one event away.
    if (!sense_wifi_is_connected()) continue;
    postCsiEvent(e);
    posted++;
  }
  return posted;
}
