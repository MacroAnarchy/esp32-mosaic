/*
 * touch_gestures: implementation — CST9217 init + double-tap/swipe FSM.
 *
 * See touch_gestures.h. The poll task runs on core 0 (the sense core)
 * at ~60Hz; the render task (core 1) never touches I2C. Mode changes
 * are pushed through the thread-safe display_face_set_csi_mode() API.
 */
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_cst9217.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display_face.h"
#include "touch_gestures.h"

static const char *TAG = "touch";

/* Board wiring (Waveshare ESP32-S3-Touch-AMOLED-1.75C). */
#define TOUCH_I2C_PORT    I2C_NUM_0
#define TOUCH_I2C_SDA     GPIO_NUM_15
#define TOUCH_I2C_SCL     GPIO_NUM_14
#define TOUCH_I2C_CLK_HZ  400000
#define TOUCH_RST_GPIO    GPIO_NUM_2   /* TP_RESET — driven, never floating */

/* Gesture thresholds. */
#define TOUCH_POLL_MS       16   /* ~60 Hz poll rate */
#define TAP_MAX_MS          300  /* press shorter than this + no move = tap */
#define TAP_MAX_MOVE_PX     24   /* max press-release drift for a tap */
#define SWIPE_MIN_MOVE_PX   48   /* min drag distance to count as a swipe */
#define PRESS_TIMEOUT_MS    800  /* drop stale presses (no release seen) */
#define DOUBLE_TAP_MAX_MS   400  /* two taps closer than this = double-tap */
#define FIRE_COOLDOWN_MS    700  /* ignore releases right after a mode change */

typedef enum {
    GEST_IDLE = 0,
    GEST_PRESSED,
} gesture_state_t;

static i2c_master_bus_handle_t s_bus = NULL;
static esp_lcd_panel_io_handle_t s_tp_io = NULL;
static esp_lcd_touch_handle_t s_tp = NULL;

/* Gesture tracking. */
static gesture_state_t s_gest = GEST_IDLE;
static uint32_t s_press_ts_ms = 0;
static uint16_t s_press_x = 0, s_press_y = 0;
static uint16_t s_release_x = 0, s_release_y = 0;
static uint32_t s_last_tap_ms = 0;      /* release ts of the previous tap */
static uint32_t s_cooldown_until_ms = 0;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static const char *csi_mode_name(csi_mode_t m)
{
    switch (m) {
    case CSI_MODE_MERGED:     return "merged";
    case CSI_MODE_STANDALONE: return "standalone";
    default:                  return "off";
    }
}

/* One cycle step: merged -> standalone -> off (plain dome) -> merged. */
static void cycle_csi_mode(const char *why, uint16_t x, uint16_t y,
                           int16_t dx, int16_t dy)
{
    csi_mode_t cur = display_face_get_csi_mode();
    int next = ((int)cur + 1) % CSI_MODE_COUNT;
    display_face_set_csi_mode(next);
    ESP_LOGI(TAG, "gesture %s at (%u,%u) d=(%d,%d): csi mode %d -> %d (%s)",
             why, (unsigned)x, (unsigned)y, (int)dx, (int)dy,
             (int)cur, next, csi_mode_name((csi_mode_t)next));
}

/* Classify a completed press/release pair and fire gestures. */
static void gesture_finish(void)
{
    int dx = (int)s_release_x - (int)s_press_x;
    int dy = (int)s_release_y - (int)s_press_y;
    uint32_t now = now_ms();
    uint32_t dur = now - s_press_ts_ms;
    float dist = sqrtf((float)(dx * dx + dy * dy));

    if (now < s_cooldown_until_ms) {
        return;  /* right after a mode change — swallow this release */
    }

    if (dist < TAP_MAX_MOVE_PX && dur <= TAP_MAX_MS) {
        if (s_last_tap_ms != 0 && (now - s_last_tap_ms) <= DOUBLE_TAP_MAX_MS) {
            s_last_tap_ms = 0;
            s_cooldown_until_ms = now + FIRE_COOLDOWN_MS;
            cycle_csi_mode("double-tap", s_release_x, s_release_y, dx, dy);
        } else {
            /* first tap — wait for a possible second one */
            s_last_tap_ms = now;
        }
    } else if (dist >= SWIPE_MIN_MOVE_PX) {
        s_last_tap_ms = 0;
        s_cooldown_until_ms = now + FIRE_COOLDOWN_MS;
        cycle_csi_mode("swipe", s_release_x, s_release_y, dx, dy);
    } else {
        /* long press without movement, or a tiny drag: ignore */
        ESP_LOGD(TAG, "gesture ignored: dist=%.0f dur=%lu", dist, (unsigned long)dur);
    }
}

static void touch_poll_task(void *arg)
{
    (void)arg;
    esp_lcd_touch_point_data_t pt[1] = {};
    uint8_t points = 0;

    for (;;) {
        if (esp_lcd_touch_read_data(s_tp) != ESP_OK) {
            /* Read error: treat as no contact this cycle. */
            points = 0;
        } else {
            esp_lcd_touch_get_data(s_tp, pt, &points, 1);
        }

        if (points > 0) {
            if (s_gest == GEST_IDLE) {
                s_gest = GEST_PRESSED;
                s_press_ts_ms = now_ms();
                s_press_x = pt[0].x;
                s_press_y = pt[0].y;
            } else if (now_ms() - s_press_ts_ms > PRESS_TIMEOUT_MS) {
                /* Stale press: restart tracking at the new contact. */
                s_press_ts_ms = now_ms();
                s_press_x = pt[0].x;
                s_press_y = pt[0].y;
            }
            s_release_x = pt[0].x;
            s_release_y = pt[0].y;
        } else if (s_gest == GEST_PRESSED) {
            s_gest = GEST_IDLE;
            gesture_finish();
        }

        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_MS));
    }
}

esp_err_t touch_gestures_init(void)
{
    if (s_tp != NULL) {
        return ESP_OK; /* already initialized */
    }

    /* Own the main I2C bus (SDA=15, SCL=14) — nothing else in this
     * firmware claims I2C_NUM_0 (the AXP2101 PMU module is not
     * compiled into the ui env). Same config as orb_pmu/orb-csi-test. */
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = TOUCH_I2C_PORT;
    bus_cfg.sda_io_num = TOUCH_I2C_SDA;
    bus_cfg.scl_io_num = TOUCH_I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.trans_queue_depth = 4;
    bus_cfg.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Touch controller panel IO on the shared bus (CST9217 @ 0x5A). */
    esp_lcd_panel_io_i2c_config_t tp_io_cfg = {};
    tp_io_cfg.dev_addr = ESP_LCD_TOUCH_IO_I2C_CST9217_ADDRESS;
    tp_io_cfg.control_phase_bytes = 1;
    tp_io_cfg.dc_bit_offset = 0;
    tp_io_cfg.lcd_cmd_bits = 8;
    tp_io_cfg.scl_speed_hz = TOUCH_I2C_CLK_HZ;
    tp_io_cfg.flags.disable_control_phase = 1;

    ret = esp_lcd_new_panel_io_i2c(s_bus, &tp_io_cfg, &s_tp_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_io_i2c failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_touch_config_t tp_cfg = {};
    tp_cfg.x_max = TOUCH_RES_X;
    tp_cfg.y_max = TOUCH_RES_Y;
    /* Drive TP_RESET (GPIO2) — the driver toggles it during init and
     * on read errors. Keeps the chip out of the floating-reset state
     * that clamped the bus during bring-up. INT is unused (polling). */
    tp_cfg.rst_gpio_num = TOUCH_RST_GPIO;
    tp_cfg.int_gpio_num = GPIO_NUM_NC;
    tp_cfg.levels.reset = 0;
    tp_cfg.levels.interrupt = 0;

    ret = esp_lcd_touch_new_i2c_cst9217(s_tp_io, &tp_cfg, &s_tp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_touch_new_i2c_cst9217 failed: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(touch_poll_task, "touch",
                                            3072, NULL, 3, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "touch poll task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "CST9217 ready on I2C0 (SDA=%d SCL=%d @%dkHz, rst=GPIO%d, "
             "res=%dx%d) — double-tap or swipe cycles the CSI view",
             TOUCH_I2C_SDA, TOUCH_I2C_SCL, TOUCH_I2C_CLK_HZ / 1000,
             TOUCH_RST_GPIO, TOUCH_RES_X, TOUCH_RES_Y);
    return ESP_OK;
}

i2c_master_bus_handle_t touch_gestures_get_bus(void)
{
    return s_bus;
}
