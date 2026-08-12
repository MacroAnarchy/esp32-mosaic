/*
 * main_ui: Mosaic UI node — "the face of the swarm".
 *
 * Unified firmware: the BLE sense engine (components/sense) runs in its
 * own task while this main owns the display face (CO5300 AMOLED + glow
 * engine, firmware/ui/display_face.cpp) and cycles face states so the
 * panel visibly works.
 *
 * Roadmap: replace the demo cycle with sense-engine → face-state mapping
 * (owner in range → FACE_OWNER_NEAR, brain WS face_state commands, ...).
 * sense_engine_get_device_count() is the integration seam.
 */

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display_face.h"
#include "sense_engine.h"

static const char *TAG = "mosaic-ui";

void app_main(void)
{
    ESP_LOGI(TAG, "mosaic-ui unified firmware starting (sense + face)");

    esp_err_t ret = display_face_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "display_face_init failed: %s", esp_err_to_name(ret));
        return;
    }
    display_face_start_render_task();
    ESP_LOGI(TAG, "face initialized — IDLE");

    /* Sense engine: NVS + NimBLE host + WiFi, then its own task. */
    ret = sense_engine_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sense_engine_init failed: %s — continuing with face only",
                 esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "sense engine started (BLE scan -> gateway)");
    }

    /* Demo cycle: prove every state renders. */
    const face_state_t cycle[] = {
        FACE_IDLE, FACE_OWNER_NEAR, FACE_VOICE,
        FACE_ALERT, FACE_SLEEP, FACE_IDLE,
    };
    const int n = sizeof(cycle) / sizeof(cycle[0]);
    int i = 0;

    while (1) {
        display_face_set_state(cycle[i]);
        ESP_LOGI(TAG, "face state -> %d (last BLE scan saw %d devices)",
                 (int)cycle[i], sense_engine_get_device_count());
        i = (i + 1) % n;
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}
