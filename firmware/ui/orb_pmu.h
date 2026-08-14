#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Attach the AXP2101 PMIC to the shared I2C bus, run the official
 * Waveshare init (power rails, IRQ masks, charging config) and start
 * the 1s event poller. Call AFTER input_touch_init() — it owns the bus.
 * Without this, the board powers off when USB is removed and the power
 * key does nothing.
 */
esp_err_t orb_pmu_init(void);

#ifdef __cplusplus
}
#endif
