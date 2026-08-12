/*
 * orb_pmu: AXP2101 power management adapter for the Mosaic face node.
 *
 * Self-contained: creates its own I2C bus (GPIO 14/15 — the Waveshare
 * S3 board's main I2C, where the AXP2101 PMIC lives) and provides the
 * register primitives the official Waveshare port_axp2101 driver needs.
 * A polling task drains PMU events (battery, VBUS, power-key) once per
 * second — the official example does the same; no GPIO IRQ wiring.
 *
 * Without this init the PMIC keeps its ROM defaults, which do NOT keep
 * the power rails alive, and the power key (PEK) does nothing useful.
 * Initializing it = board stays on + the side button works (short press
 * = wake, long press = power off).
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "orb_pmu.h"

#define PMU_I2C_SDA    GPIO_NUM_15
#define PMU_I2C_SCL    GPIO_NUM_14
#define PMU_I2C_SPEED  100000
#define PMU_I2C_ADDR   0x34
#define PMU_TIMEOUT_MS 50
#define PMU_POLL_MS    1000

static const char *TAG = "orb_pmu";
static i2c_master_bus_handle_t s_pmu_bus = NULL;
static i2c_master_dev_handle_t s_pmu_dev = NULL;

/* ---- Register primitives the official port_axp2101.cpp expects ---- */
int pmu_register_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    (void)devAddr;
    if (s_pmu_dev == NULL) return -1;
    esp_err_t ret = i2c_master_transmit_receive(s_pmu_dev, &regAddr, 1,
                                                data, len, PMU_TIMEOUT_MS);
    return (ret == ESP_OK) ? 0 : -1;
}

int pmu_register_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    (void)devAddr;
    if (s_pmu_dev == NULL) return -1;
    uint8_t *buf = (uint8_t *)malloc(len + 1);
    if (buf == NULL) return -1;
    buf[0] = regAddr;
    memcpy(&buf[1], data, len);
    esp_err_t ret = i2c_master_transmit(s_pmu_dev, buf, len + 1, PMU_TIMEOUT_MS);
    free(buf);
    return (ret == ESP_OK) ? 0 : -1;
}

/* Declared by the official driver (port_axp2101.cpp). */
extern esp_err_t pmu_init(void);
extern void pmu_isr_handler(void);

static void pmu_poll_task(void *arg)
{
    (void)arg;
    while (1) {
        pmu_isr_handler();
        vTaskDelay(pdMS_TO_TICKS(PMU_POLL_MS));
    }
}

esp_err_t orb_pmu_init(void)
{
    if (s_pmu_bus != NULL) return ESP_OK;

    /* Own I2C bus: the PMU is on the board's main I2C (GPIO 14/15). */
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = PMU_I2C_SDA;
    bus_cfg.scl_io_num = PMU_I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_pmu_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "create I2C bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = PMU_I2C_ADDR,
        .scl_speed_hz    = PMU_I2C_SPEED,
    };
    ret = i2c_master_bus_add_device(s_pmu_bus, &dev, &s_pmu_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add PMU device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Chip-id sanity: AXP2101 reports 0x4A at reg 0x03. */
    uint8_t id = 0;
    if (pmu_register_read(0, 0x03, &id, 1) != 0 || id == 0) {
        ESP_LOGW(TAG, "PMU chip id read failed (0x%02x) — continuing anyway", id);
    } else {
        ESP_LOGI(TAG, "AXP2101 chip id 0x%02x", id);
    }

    ret = pmu_init();   /* official driver: outputs, IRQ masks, charging */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AXP2101 init failed — board may power off on unplug");
        return ret;
    }

    xTaskCreate(pmu_poll_task, "orb_pmu", 4 * 1024, NULL, 10, NULL);
    ESP_LOGI(TAG, "AXP2101 PMU live — rails kept on battery, PEK active");
    return ESP_OK;
}
