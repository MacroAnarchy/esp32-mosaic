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
#include "esp_timer.h"
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
#ifndef MOSAIC_CSI_FEATURE_INTERVAL_SECONDS
#define MOSAIC_CSI_FEATURE_INTERVAL_SECONDS 15  // periodic feature snapshot cadence — keeps the gateway csi channel LIVE in quiet rooms (posted from the sense loop, which cycles at ~20s)
#endif
#ifndef MOSAIC_CSI_TRAIN_DURATION_MS
#define MOSAIC_CSI_TRAIN_DURATION_MS 20000     // presence calibration window (room must stay static while training)
#endif
#ifndef MOSAIC_CSI_TRAIN_ENABLE
#define MOSAIC_CSI_TRAIN_ENABLE 1              // one-time presence calibration at boot (best effort — retries, gives up after 3 attempts)
#endif

static const char *TAG = "csi";

static esp_wifi_sensing_fsm_handle_t s_fsm = NULL;
static uint8_t s_peerMac[6] = {0};  // AP BSSID — the single sensing channel
static bool s_running = false;
static uint32_t s_csiStartMs = 0;
static uint32_t s_lastFeatureMs = 0;

// Presence calibration (esp-radar train workflow): 0 idle, 1 training,
// 2 settled (done or gave up after retries). Motion detection works
// without it; the presence channel only becomes meaningful after a
// successful train_stop caches thresholds on the channel. Driven from
// the sense loop (never from a separate task — see csiFeatureTick).
static uint8_t s_trainState = 0;
static uint32_t s_trainStartMs = 0;
static uint8_t s_trainAttempts = 0;

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
// Envelope producer — one type:"csi" envelope per queued event, plus a
// periodic feature snapshot that keeps the gateway csi channel LIVE.
// ---------------------------------------------------------------------
static std::string fmtFloat(float v) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%.6g", (double)v);
  return std::string(buf);
}

// Shared payload builder. Presence/train fields are additive — the
// gateway csi_events table gains columns for them (idempotent ALTER);
// older gateway code ignores unknown payload keys.
static std::string csiPayloadJson(const char *event, bool someone, bool moved,
                                  float wander, float jitter, uint32_t score,
                                  const esp_wifi_sensing_fsm_channel_diag_t &diag) {
  std::string p = "{\"event\":\"" + std::string(event) +
                  "\",\"someone\":" + (someone ? "true" : "false") +
                  ",\"moved\":" + (moved ? "true" : "false") +
                  ",\"wander\":" + fmtFloat(wander) +
                  ",\"jitter\":" + fmtFloat(jitter) +
                  ",\"score\":" + std::to_string(score) +
                  ",\"presence_ready\":" + (diag.presence_ready ? "true" : "false") +
                  ",\"presence_wander_average\":" + fmtFloat(diag.presence_wander_average) +
                  ",\"presence_someone_threshold\":" + fmtFloat(diag.presence_someone_threshold) +
                  ",\"presence_someone\":" + (diag.presence_someone_status ? "true" : "false") +
                  ",\"train_valid\":" + (diag.train_thresholds_valid ? "true" : "false") +
                  ",\"train_wander_threshold\":" + fmtFloat(diag.train_wander_threshold) +
                  ",\"train_jitter_threshold\":" + fmtFloat(diag.train_jitter_threshold) + "}";
  return p;
}

static int postCsiEvent(const CsiQueueEntry &e) {
  const bool active = (e.event == ESP_WIFI_SENSING_FSM_EVENT_ACTIVE);

  // Wander/jitter are the latest waveform metrics (best effort snapshot;
  // diagnostics are per-channel state, read from the sense task).
  float wander = 0.0f;
  float jitter = 0.0f;
  esp_wifi_sensing_fsm_channel_diag_t diag = {};
  if (s_fsm && esp_wifi_sensing_fsm_get_channel_diag(s_fsm, e.peer, &diag) == ESP_OK) {
    wander = diag.wander_value;
    jitter = diag.jitter_value;
  }

  // Tier 1 honesty: "someone" means radio-visible motion on the channel
  // (moved=true) OR the calibrated presence channel says someone is
  // stationary (presence_someone). Untrained presence reports false.
  std::string payload = "{\"v\":1,\"node\":\"" + std::string(MOSAIC_NODE_NAME) +
                        "\",\"type\":\"csi\",\"ts\":" +
                        std::to_string(sense::uptime_ms()) +
                        ",\"payload\":" +
                        csiPayloadJson(active ? "moved" : "empty", active, active,
                                       wander, jitter, e.score, diag) + "}";

  int code = sense::post_json(sense::gateway_ingest_url(), payload);
  ESP_LOGI(TAG, "POST %s: %d", active ? "moved" : "empty", code);
  return code;
}

// Periodic feature snapshot (event:"feature"): presence + motion waveform
// metrics at a ~5s cadence. Keeps the gateway csi channel LIVE in quiet
// rooms (event-driven motion alone ages it to OFFLINE) and feeds the
// anomaly engine a continuous feature stream. ~70 B/s worst case, well
// under the features-only ~200 B/s research budget.
static int postCsiFeature(void) {
  if (!s_fsm || !s_running || !sense_wifi_is_connected()) return 0;
  esp_wifi_sensing_fsm_channel_diag_t diag = {};
  if (esp_wifi_sensing_fsm_get_channel_diag(s_fsm, s_peerMac, &diag) != ESP_OK) return 0;
  const bool moved = (diag.state == ESP_WIFI_SENSING_FSM_PROCESS_ACTIVE ||
                      diag.state == ESP_WIFI_SENSING_FSM_PROCESS_DEBOUNCE_ACTIVE);
  std::string payload = "{\"v\":1,\"node\":\"" + std::string(MOSAIC_NODE_NAME) +
                        "\",\"type\":\"csi\",\"ts\":" +
                        std::to_string(sense::uptime_ms()) +
                        ",\"payload\":" +
                        csiPayloadJson("feature", diag.presence_someone_status, moved,
                                       diag.wander_value, diag.jitter_value,
                                       diag.smooth_scaled, diag) + "}";
  int code = sense::post_json(sense::gateway_ingest_url(), payload);
  ESP_LOGI(TAG, "feature POST: %d wander=%.4f jitter=%.4f presence=%s train_valid=%s",
           code, (double)diag.wander_value, (double)diag.jitter_value,
           diag.presence_someone_status ? "someone" : "nobody",
           diag.train_thresholds_valid ? "yes" : "no");
  return code;
}

// ---------------------------------------------------------------------
// Presence calibration + feature snapshot tick. Runs INSIDE the sense
// loop (csi_sensing_drain_and_report), never from a separate task: the
// sense loop already sequences HTTP POSTs around the BLE scan and the
// WiFi offline-scan cycle, and a concurrent POSTing task was verified to
// wedge the lwIP TX path (zombie STA, 2026-08-13 — see git log).
// ---------------------------------------------------------------------
static void driveTraining(void) {
#if MOSAIC_CSI_TRAIN_ENABLE
  if (!s_fsm || !s_running) return;
  const uint32_t now = sense::uptime_ms();
  if (s_trainState == 0) {
    // Let the FSM learn its baseline first, then calibrate presence
    // thresholds over a static window (cat/empty room = background).
    if (now - s_csiStartMs >= 5000) {
      if (esp_wifi_sensing_fsm_train_start(s_fsm, s_peerMac) == ESP_OK) {
        s_trainState = 1;
        s_trainStartMs = now;
        ESP_LOGI(TAG, "presence calibration: collecting %u ms (keep room static)",
                 (unsigned)MOSAIC_CSI_TRAIN_DURATION_MS);
      } else {
        ESP_LOGW(TAG, "train start failed — retrying");
      }
    }
  } else if (s_trainState == 1 && (now - s_trainStartMs) >= MOSAIC_CSI_TRAIN_DURATION_MS) {
    float wt = 0.0f, jt = 0.0f;
    esp_err_t tr = esp_wifi_sensing_fsm_train_stop(s_fsm, s_peerMac, &wt, &jt);
    if (tr == ESP_OK) {
      s_trainState = 2;
      ESP_LOGI(TAG, "presence calibration done: wander_thr=%.4f jitter_thr=%.4f",
               (double)wt, (double)jt);
    } else {
      ESP_LOGW(TAG, "train stop failed: %s — retrying", esp_err_to_name(tr));
      esp_wifi_sensing_fsm_train_remove(s_fsm, s_peerMac);
      if (++s_trainAttempts >= 3) {
        s_trainState = 2;
        ESP_LOGW(TAG, "presence calibration gave up after %u attempts",
                 (unsigned)s_trainAttempts);
      } else {
        s_trainState = 0;  // retry the whole window on a later tick
      }
    }
  }
#else
  (void)0;  // calibration disabled via MOSAIC_CSI_TRAIN_ENABLE=0
#endif  // MOSAIC_CSI_TRAIN_ENABLE
}

// One feature snapshot per interval. Called from the sense loop.
static void csiFeatureTick(void) {
  if (!s_fsm || !s_running) return;
  const uint32_t now = sense::uptime_ms();
  if (now - s_lastFeatureMs < (uint32_t)MOSAIC_CSI_FEATURE_INTERVAL_SECONDS * 1000UL) return;
  s_lastFeatureMs = now;
  driveTraining();
  postCsiFeature();
}

// ---------------------------------------------------------------------
// UI feature cache — the face renderer's live data seam.
//
// The dome visualization is driven by the REAL channel features
// (wander / jitter / presence / motion), so the UI needs a fresh
// snapshot more often than the ~15s gateway feature cadence. A 5Hz
// esp_timer reads the FSM's channel diagnostics (a cheap public read —
// no HTTP, no radio-path writes; the zombie-STA rule only forbids
// concurrent POSTing tasks) into a tiny spinlock-guarded cache. The
// render task never touches the FSM or the component API.
// ---------------------------------------------------------------------
#define CSI_UI_SAMPLE_MS 200  // 5Hz — live enough for the halo, ~nothing on CPU

static sense_csi_features_t s_csiUi = {};
static portMUX_TYPE s_csiUiLock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_csiUiSampleId = 0;
static esp_timer_handle_t s_uiTimer = NULL;

static void csiUiTick(void *arg) {
  (void)arg;
  if (!s_fsm || !s_running) return;  // paused (offline scan): keep last sample
  esp_wifi_sensing_fsm_channel_diag_t diag = {};
  if (esp_wifi_sensing_fsm_get_channel_diag(s_fsm, s_peerMac, &diag) != ESP_OK) return;
  sense_csi_features_t f;
  f.wander = diag.wander_value;
  f.jitter = diag.jitter_value;
  f.smooth = (float)diag.smooth_scaled;
  f.someone = diag.presence_someone_status;
  f.moved = (diag.state == ESP_WIFI_SENSING_FSM_PROCESS_ACTIVE ||
             diag.state == ESP_WIFI_SENSING_FSM_PROCESS_DEBOUNCE_ACTIVE);
  f.presenceReady = diag.presence_ready;
  f.trainValid = diag.train_thresholds_valid;
  f.updatedMs = sense::uptime_ms();
  f.sampleId = ++s_csiUiSampleId;
  portENTER_CRITICAL(&s_csiUiLock);
  s_csiUi = f;
  portEXIT_CRITICAL(&s_csiUiLock);
}

static void csiUiTimerStart(void) {
  if (s_uiTimer) return;
  const esp_timer_create_args_t targs = {
      .callback = csiUiTick,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "csi_ui",
      .skip_unhandled_events = true,
  };
  if (esp_timer_create(&targs, &s_uiTimer) != ESP_OK) {
    ESP_LOGW(TAG, "UI feature timer create failed — dome renders without CSI");
    return;
  }
  esp_timer_start_periodic(s_uiTimer, CSI_UI_SAMPLE_MS * 1000ULL);
}

// Close the boot-time DEBUG window (see csi_sensing_init) without needing
// a task: a one-shot esp_timer restores INFO ~5s after init.
static void closeDebugWindow(void *arg) {
  (void)arg;
  esp_log_level_set("esp_wifi_sensing_fsm", ESP_LOG_INFO);
  ESP_LOGI(TAG, "FSM debug diag closed (INFO)");
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------
esp_err_t csi_sensing_init(void) {
  if (s_fsm) return ESP_OK;  // already created

  // Prove CSI frames are flowing: the prebuilt lib logs a DEBUG diag line
  // (tag esp_wifi_sensing_fsm) whose radar_cb counter counts CSI frames
  // fed into the FSM and ping_to/ping_reply_hz show ping-assisted sampling
  // health. Prebuilt libs respect runtime levels — no Kconfig change.
  // BOUNDED WINDOW (~5s): the same DEBUG level also makes the FSM log every
  // radar sample (~20-100/s), which saturates the 115200 UART and stalls
  // the WiFi driver during the offline-scan reconnect (zombie STA, verified
  // live 2026-08-13). The feature task restores INFO on its first tick.
  esp_log_level_set("esp_wifi_sensing_fsm", ESP_LOG_DEBUG);

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

  // Close the DEBUG window ~5s after init (one-shot timer). The window
  // captures the radar_cb diag lines (frames-flowing proof) but must not
  // persist: the same DEBUG level makes the FSM log every radar sample
  // (~50-100/s), which saturates the 115200 UART and stalls the WiFi
  // driver's offline-scan reconnect (zombie STA, verified live 2026-08-13).
  s_csiStartMs = sense::uptime_ms();
  s_lastFeatureMs = s_csiStartMs;
  const esp_timer_create_args_t targs = {
      .callback = closeDebugWindow,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "csi_dbg_close",
      .skip_unhandled_events = false,
  };
  esp_timer_handle_t t = NULL;
  if (esp_timer_create(&targs, &t) == ESP_OK) {
    esp_timer_start_once(t, 5000 * 1000ULL);  // 5s
  }

  // Live UI feature cache for the face renderer (5Hz diag sampling —
  // see csiUiTick above).
  csiUiTimerStart();

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
  // Periodic feature snapshot (presence + waveform metrics) keeps the
  // gateway csi channel LIVE in quiet rooms. Runs in the sense loop —
  // same task that sequences every other POST, so it never collides
  // with the BLE scan or the WiFi offline-scan cycle.
  if (sense_wifi_is_connected()) {
    csiFeatureTick();
  }
  return posted;
}

// ---------------------------------------------------------------------
// UI feature getter (declared in sense_engine.h — the face layer's data
// seam). Copies the ~5Hz diag cache out for the render task.
// ---------------------------------------------------------------------
bool csi_sensing_get_ui_features(struct sense_csi_features *out) {
  if (out == NULL) return false;
  portENTER_CRITICAL(&s_csiUiLock);
  *out = s_csiUi;
  portEXIT_CRITICAL(&s_csiUiLock);
  return out->sampleId > 0;
}
