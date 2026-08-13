/*
 * display_face: Mosaic node UI — the face of a sensing node.
 *
 * "PRESENCE becomes light." A particle field rendered with a direct
 * framebuffer glow engine (see fx/fx_glow.h) whose density, color and
 * motion are driven by the brain state. The panel is a 466x466 QSPI
 * AMOLED (CO5300 controller) driven through M5GFX (LovyanGFX) with a
 * PSRAM framebuffer — the StopWatch-Flux pattern.
 *
 * A FreeRTOS task (core 1, moderate priority) owns the render loop at
 * a 33fps target. All public calls are thread-safe and may be used
 * from any task (e.g. the WS/gateway task).
 *
 * C-callable so the app shell (C or C++) can drive it.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Face states driven by brain presence/voice events. */
typedef enum {
    FACE_IDLE = 0,        /* alone, calm: slow cool drift                */
    FACE_OWNER_NEAR,      /* owner nearby: warm amber swarm              */
    FACE_VOICE,           /* listening/talking: snaps to mic energy      */
    FACE_ALERT,           /* agitated/fast: hot, erratic                 */
    FACE_SLEEP,           /* dim, barely breathing                       */
    FACE_STATE_COUNT
} face_state_t;

/* Initialize the panel + glow engine and start the render task.
 * Safe to call once from app_main. Returns ESP_OK on success. */
esp_err_t display_face_init(void);

/* Start the 33fps render task (core 1, moderate priority). Call after
 * display_face_init(). */
void display_face_start_render_task(void);

/* Request a face state; transitions blend over ~0.3s. Thread-safe. */
esp_err_t display_face_set_state(face_state_t state);

face_state_t display_face_get_state(void);

/* VOICE placeholder input: mic energy 0..1 (M3 will feed real RMS).
 * Only meaningful in FACE_VOICE; stored regardless. Thread-safe. */
void display_face_set_voice_energy(float energy);

/* Pause/resume rendering (e.g. before deep sleep). */
esp_err_t display_face_set_suspended(bool suspended);

/* CSI visualization modes — the face renders the radio dome plus a
 * live CSI layer driven by the sense engine's real channel features
 * (wander / jitter / presence / motion). */
typedef enum {
    CSI_MODE_OFF = 0,      /* plain radio dome (pre-CSI behavior)     */
    CSI_MODE_MERGED,       /* dome + Siri-like morphing halo around the core */
    CSI_MODE_STANDALONE,   /* full-screen signal anatomy (frequency rings,
                            * polar waveform, aberration waves)        */
    CSI_MODE_COUNT
} csi_mode_t;

/* Pin the CSI visualization mode: -1 = auto-cycle (default: every
 * ~30s, merged -> standalone -> off -> ...), 0..2 = pinned. Thread-safe. */
esp_err_t display_face_set_csi_mode(int mode);

csi_mode_t display_face_get_csi_mode(void);

/* ---- Settings menu integration (orb-settings backbone) ---- */

/* Set display brightness 0..100. Maps to M5GFX setBrightness internally.
 * Thread-safe. The settings menu calls this. */
void display_face_set_brightness(int pct);
int  display_face_get_brightness(void);

/* Get the raw M5GFX display handle for direct drawing (fillRect,
 * drawString, etc. — used by the settings menu). Returns void* =
 * M5GFX*. NULL if display init failed. The caller must hold the
 * face mutex (via display_face_render_menu_frame) before drawing. */
void *display_face_get_display(void);

/* Full framebuffer clear + flush — used by the settings menu to render
 * a clean frame (the glow canvas's beginFrame/push path doesn't handle
 * direct M5GFX draws). Clears the whole framebuffer to black, then the
 * caller draws via the display handle, then calls this again to flush. */
void display_face_clear_frame(void);
void display_face_flush_frame(void);

#ifdef __cplusplus
}
#endif
