/*
 * main_ui: Mosaic UI node — "the face of the swarm".
 *
 * Unified firmware: the BLE sense engine (components/sense) runs in its
 * own task while this main owns the display face (CO5300 AMOLED + glow
 * engine, firmware/ui/display_face.cpp). The face's default live view is
 * THE RADIO DOME: the local BLE device table rendered as a living
 * aurora dome — center = this node, rings = RSSI zones, orbs = sensed
 * devices (onboard data only, no gateway).
 *
 * The existing face state system (face_set_state) stays intact — states
 * tint/boost the dome (amber wisps, voice ring, alert sweep, sleep dim)
 * and future brain/WS face_state commands can drive it from here.
 * sense_engine_get_device_count() remains the integration seam for logs.
 */

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display_face.h"
#include "sense_engine.h"
#include "touch_gestures.h"

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
    ESP_LOGI(TAG, "face initialized — radio dome live (IDLE)");

    /* Sense engine: NVS + NimBLE host + WiFi, then its own task. */
    ret = sense_engine_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sense_engine_init failed: %s — continuing with face only",
                 esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "sense engine started (BLE scan -> gateway)");
    }

    /* Touch (CST9217): double-tap / swipe cycles the CSI display mode.
     * Failure is non-fatal — the face keeps rendering. */
    ret = touch_gestures_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "touch_gestures_init failed: %s — auto-cycle only",
                 esp_err_to_name(ret));
    }

    /* The dome is the default live view: calm idle when the ether is
     * empty, orbs + streams appear as the node's senses wake up. */
    display_face_set_state(FACE_IDLE);

    while (1) {
        ESP_LOGI(TAG, "radio dome: last scan saw %d device(s)",
                 sense_engine_get_device_count());
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
