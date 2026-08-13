#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Attach the AXP2101 PMIC to the I2C bus, run the Waveshare init (power
 * rails, IRQ masks, charging config) and start the event poller + button
 * event dispatcher.
 *
 * WITHOUT this, the board powers off when USB is removed and the power
 * key (PEK) does nothing.
 *
 * Call AFTER input_touch_init() — the touch controller owns I2C_NUM_0
 * creation; this module shares that bus (touch_gestures_get_bus()).
 */
esp_err_t orb_pmu_init(void);

/* ---- PEK (power-key) button events ---- */

typedef enum {
    ORB_PEK_NONE = 0,
    ORB_PEK_SHORT_PRESS,   /* short press (<1.5s)   */
    ORB_PEK_LONG_PRESS,    /* long press (>=1.5s)   */
} orb_pek_event_t;

/**
 * Consume the next PEK event (returns ORB_PEK_NONE if none pending).
 * Called from the UI/settings layer. Once consumed the event is cleared.
 */
orb_pek_event_t orb_pmu_get_event(void);

/* ---- Power management readout (thread-safe) ---- */

typedef struct {
    bool    pmu_ok;         /* AXP2101 initialized successfully      */
    bool    batt_present;   /* battery detected on the BAT pins       */
    bool    vbus_present;   /* USB/external power connected           */
    bool    charging;       /* actively charging                      */
    uint8_t batt_percent;   /* 0..100 (AXP2101 fuel gauge, or est)    */
    uint16_t batt_mv;       /* battery voltage in mV (0 if no batt)   */
    uint16_t vbus_mv;       /* VBUS voltage in mV (0 if no VBUS)      */
    uint16_t system_mv;     /* system bus voltage in mV               */
    float   temp_c;         /* PMU die temperature °C                 */
} orb_power_info_t;

/** Fill *out with the latest power readings. Thread-safe. */
void orb_pmu_get_power_info(orb_power_info_t *out);

#ifdef __cplusplus
}
#endif
