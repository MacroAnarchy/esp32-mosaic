/*
 * display_face: Mosaic node UI — the face of a sensing node.
 *
 * "PRESENCE becomes light." A particle field rendered with a direct
 * framebuffer glow engine (see fx/fx_glow.h) whose density, color and
 * motion are driven by the brain state. The panel is a 466x466 QSPI
 * AMOLED (CO5300 controller) driven directly through esp_lcd — no
 * board-support package, so the module builds on any CO5300 AMOLED.
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

#ifdef __cplusplus
}
#endif
