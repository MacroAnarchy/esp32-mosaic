/*
 * touch_gestures: CST9217 capacitive touch -> CSI display mode switching.
 *
 * The Waveshare ESP32-S3-Touch-AMOLED-1.75C carries a Hynitron CST9217
 * touch controller on the board's main I2C bus (SDA=15, SCL=14) at
 * 0x5A. Its reset line is TP_RESET = GPIO2 (driven here — the floating-
 * reset failure mode that clamped the bus on the orb bring-up is gone)
 * and the interrupt line (GPIO11) is unused: this module polls.
 *
 * Nothing else in the [env:ui] firmware owns I2C_NUM_0 (the AXP2101 PMU
 * module is not compiled in), so this module creates the bus with the
 * same wiring verified in orb-csi-test (main/input_touch.cpp). M5GFX's
 * own touch classes cover CST816S/CST226 only, so the CST9217 is driven
 * through the vendored esp_lcd_touch_cst9217 component
 * (ui/components/cst9217).
 *
 * Gesture model (single-touch panel):
 *   - DOUBLE-TAP: two taps (press+release within 300ms, drift < 24px)
 *                 separated by <= 400ms -> cycle CSI display mode
 *   - SWIPE:      press+release dragged >= 48px (any direction)
 *                 -> cycle CSI display mode
 * Mode cycling goes merged -> standalone -> plain dome -> merged via the
 * existing display_face_set_csi_mode() API. A short post-fire cooldown
 * swallows the release of the double-tap's second press. The render
 * task's auto-cycle (~60s) remains the fallback until the first touch
 * pins a mode.
 *
 * C-callable so the app shell (C or C++) can drive it.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Panel resolution in pixels (466x466 QSPI AMOLED). */
#define TOUCH_RES_X 466
#define TOUCH_RES_Y 466

/* Initialize the shared I2C bus + CST9217 and start the poll task.
 * Safe to call once. Returns ESP_OK on success; a failure is logged and
 * the face keeps rendering (touch is a UI nicety, not a dependency). */
esp_err_t touch_gestures_init(void);

/* The shared I2C master bus handle (NULL until touch_gestures_init).
 * Exposed so a future PMU/IMU module can add devices to the same bus. */
i2c_master_bus_handle_t touch_gestures_get_bus(void);

#ifdef __cplusplus
}
#endif
