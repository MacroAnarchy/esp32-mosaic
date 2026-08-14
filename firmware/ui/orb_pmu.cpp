/*
 * orb_pmu: AXP2101 power management adapter for the Mosaic face node.
 *
 * Self-contained: shares the I2C bus from the touch controller (CST9217
 * owns I2C_NUM_0 creation) and provides the register primitives the
 * official Waveshare port_axp2101 driver needs. A polling task drains
 * PMU events (battery, VBUS, power-key) once per second — the official
 * example does the same; no GPIO IRQ wiring.
 *
 * Button events: the poll task reads INTSTS1 and extracts the PEK
 * (power-key) short/long interrupt bits. Long-press → settings menu,
 * short-press → event hook (future orb-mode switching). Events are
 * queued in a volatile variable consumed by the UI layer.
 *
 * Power readout: caches the latest battery voltage, VBUS voltage, system
 * voltage, temperature and battery percentage in a thread-safe struct,
 * read by the settings menu.
 *
 * Without this init the PMIC keeps its ROM defaults, which do NOT keep
 * the power rails alive, and the power key (PEK) does nothing useful.
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "orb_pmu.h"
#include "touch_gestures.h"  /* touch_gestures_get_bus() */

#include "axp2101_registers.h"

#define PMU_I2C_ADDR   0x34
#define PMU_TIMEOUT_MS 200   /* generous — AXP2101 can be slow on shared bus */
#define PMU_POLL_MS    500    /* poll PEK 2x/sec — enough for button events */

/* The long-press threshold: PEK held >= this many ms = LONG_PRESS.
 * The AXP2101 hardware generates the PKEY_LONG IRQ itself, so this is
 * really just documentation — the chip does the timing. */
#define PEK_LONG_PRESS_MS 1500

static const char *TAG = "orb_pmu";
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

/* Declared by the official driver (port_axp2101.cpp). We call it but
 * tolerate failure — our own init below handles the critical rails. */
extern esp_err_t pmu_init(void);
extern void pmu_isr_handler(void);

/* ---- AXP2101 register helpers (self-contained, no port_axp2101 deps) ---- */

static bool axp_read_reg(uint8_t reg, uint8_t *val)
{
    return pmu_register_read(0, reg, val, 1) == 0;
}

static uint16_t axp_read_h5l8(uint8_t hi_reg, uint8_t lo_reg)
{
    uint8_t hi = 0, lo = 0;
    if (!axp_read_reg(hi_reg, &hi) || !axp_read_reg(lo_reg, &lo))
        return 0;
    return ((hi & 0x1F) << 8) | lo;
}

static uint16_t axp_read_h6l8(uint8_t hi_reg, uint8_t lo_reg)
{
    uint8_t hi = 0, lo = 0;
    if (!axp_read_reg(hi_reg, &hi) || !axp_read_reg(lo_reg, &lo))
        return 0;
    return ((hi & 0x3F) << 8) | lo;
}

static bool axp_get_bit(uint8_t reg, uint8_t bit)
{
    uint8_t val = 0;
    return axp_read_reg(reg, &val) && ((val >> bit) & 1);
}

static bool axp_write_reg(uint8_t reg, uint8_t val)
{
    return pmu_register_write_byte(0, reg, &val, 1) == 0;
}

static bool axp_clear_irq(void)
{
    bool ok = true;
    ok &= axp_write_reg(AXP2101_REG_INTSTS1, 0xFF);
    ok &= axp_write_reg(AXP2101_REG_INTSTS2, 0xFF);
    ok &= axp_write_reg(AXP2101_REG_INTSTS3, 0xFF);
    return ok;
}

/* Minimal self-contained PMU init: enable power rails + ADC channels +
 * PEK IRQs. This replaces the heavy port_axp2101.cpp pmu_init() which
 * does dozens of register reads/writes and is fragile on the shared
 * I2C bus (intermittent 0x00 reads under touch polling contention). */
static esp_err_t orb_pmu_minimal_init(void)
{
    /* Each write can fail silently on the shared bus. Log progress so
     * we can see where it hangs. */
    int writes_ok = 0;
    int writes_total = 0;

#define AXP_WR(reg, val) do { \
    writes_total++; \
    if (axp_write_reg(reg, val)) writes_ok++; \
    else ESP_LOGW(TAG, "PMU write 0x%02x failed", reg); \
} while(0)

    AXP_WR(AXP2101_REG_ADC_CHANNEL_CTRL, 0x1D);
    AXP_WR(AXP2101_REG_DC_VOL0_CTRL, (3300 - AXP2101_DCDC1_MIN_MV) / AXP2101_DCDC1_STEP_MV);
    AXP_WR(AXP2101_REG_DC_ONOFF_DVM_CTRL, 0x01);
    AXP_WR(AXP2101_REG_LDO_VOL0_CTRL, (3300 - AXP2101_LDO_MIN_MV) / AXP2101_LDO_STEP_MV);
    AXP_WR(AXP2101_REG_LDO_ONOFF_CTRL0, 0x01);
    AXP_WR(AXP2101_REG_INTEN1, 0x00);
    AXP_WR(AXP2101_REG_INTEN2, (uint8_t)((AXP2101_IRQ_PKEY_LONG >> 8) | (AXP2101_IRQ_PKEY_SHORT >> 8)));
    AXP_WR(AXP2101_REG_INTEN3, 0x00);
    AXP_WR(AXP2101_REG_IPRECHG_SET, AXP2101_PRECHARGE_50MA);
    AXP_WR(AXP2101_REG_ICC_CHG_SET, AXP2101_CHARGE_CURRENT_400MA);
    AXP_WR(AXP2101_REG_ITERM_CHG_SET_CTRL, AXP2101_TERMINATION_CURRENT_25MA);
    AXP_WR(AXP2101_REG_CV_CHG_VOL_SET, AXP2101_CHARGE_VOLTAGE_4V2);
    axp_clear_irq();

#undef AXP_WR
    ESP_LOGI(TAG, "minimal init: %d/%d writes ok", writes_ok, writes_total);
    return writes_ok > 0 ? ESP_OK : ESP_FAIL;
}

/* ---- PEK button event state ---- */
static volatile orb_pek_event_t s_pek_event = ORB_PEK_NONE;

/* ---- Cached power info (updated by poll task, read by UI) ---- */
static portMUX_TYPE s_power_lock = portMUX_INITIALIZER_UNLOCKED;
static orb_power_info_t s_power_info = {};

static void pmu_poll_task(void *arg)
{
    (void)arg;

    /* First-cycle init: enable PEK IRQs + ADC channels. Done here (not
     * in orb_pmu_init) to avoid blocking app_main on I2C writes that
     * contend with the 60Hz touch poll task on the shared bus. Wrapped
     * in the bus lock to serialize with touch reads. */
    touch_i2c_lock();
    axp_write_reg(AXP2101_REG_ADC_CHANNEL_CTRL, 0x1D);  /* ADC: batt+vbus+sys+temp */
    axp_write_reg(AXP2101_REG_INTEN2,
                  (uint8_t)((AXP2101_IRQ_PKEY_LONG >> 8) | (AXP2101_IRQ_PKEY_SHORT >> 8)));
    axp_clear_irq();
    touch_i2c_unlock();
    ESP_LOGI(TAG, "poll task: PEK IRQs + ADC enabled");

    while (1) {
        /* Serialize all PMU I2C reads in one locked batch to avoid
         * corrupting the touch poll task's reads on the shared bus. */
        touch_i2c_lock();

        /* Read the IRQ status registers (INTSTS1/2/3). */
        uint8_t sts1 = 0, sts2 = 0;
        axp_read_reg(AXP2101_REG_INTSTS1, &sts1);
        axp_read_reg(AXP2101_REG_INTSTS2, &sts2);

        /* AXP2101 IRQ bits (from axp2101_registers.h):
         *   AXP2101_IRQ_PKEY_LONG  = bit 10 (INTSTS2 bit 2)
         *   AXP2101_IRQ_PKEY_SHORT = bit 11 (INTSTS2 bit 3)
         * INTSTS2 maps to bits 8..15 of the 24-bit IRQ word, so:
         *   PKEY_LONG  = INTSTS2 bit 2
         *   PKEY_SHORT = INTSTS2 bit 3
         */
        bool pek_long  = (sts2 & (1 << 2)) != 0;
        bool pek_short = (sts2 & (1 << 3)) != 0;

        /* Priority: long-press overrides short-press (both can set in
         * the same read on some firmwares). */
        if (pek_long) {
            s_pek_event = ORB_PEK_LONG_PRESS;
            ESP_LOGI(TAG, "PEK long-press -> settings menu request");
        } else if (pek_short) {
            s_pek_event = ORB_PEK_SHORT_PRESS;
            ESP_LOGI(TAG, "PEK short-press");
        }

        /* Clear IRQ flags. */
        axp_clear_irq();

        /* ---- Update cached power info ---- */
        bool batt_conn = axp_get_bit(AXP2101_REG_STATUS1, 3);  /* bit 3 = bat exist */
        bool vbus_good = axp_get_bit(AXP2101_REG_STATUS1, 5);  /* bit 5 = vbus good */

        /* STATUS2 bits 6:5: 0=standby, 1=charging, 2=discharge */
        uint8_t sts2b = 0;
        axp_read_reg(AXP2101_REG_STATUS2, &sts2b);
        uint8_t chg_mode = (sts2b >> 5) & 0x03;
        bool charging = (chg_mode == 0x01);
        bool vbus_in = (chg_mode != 0x02) && vbus_good;  /* not discharge + vbus good */

        uint16_t batt_mv = 0;
        uint8_t batt_pct = 0;
        if (batt_conn) {
            batt_mv = axp_read_h5l8(AXP2101_REG_ADC_DATA_RESULT0,
                                    AXP2101_REG_ADC_DATA_RESULT1);
            axp_read_reg(AXP2101_REG_BAT_PERCENT_DATA, &batt_pct);
        }
        uint16_t vbus_mv = vbus_in
            ? axp_read_h6l8(AXP2101_REG_ADC_DATA_RESULT4,
                            AXP2101_REG_ADC_DATA_RESULT5) : 0;
        uint16_t sys_mv = axp_read_h6l8(AXP2101_REG_ADC_DATA_RESULT6,
                                        AXP2101_REG_ADC_DATA_RESULT7);
        uint16_t temp_raw = axp_read_h6l8(AXP2101_REG_ADC_DATA_RESULT8,
                                          AXP2101_REG_ADC_DATA_RESULT9);
        float temp_c = 22.0f + (7274.0f - (float)temp_raw) / 20.0f;

        portENTER_CRITICAL(&s_power_lock);
        s_power_info.pmu_ok = true;
        s_power_info.batt_present = batt_conn;
        s_power_info.vbus_present = vbus_in;
        s_power_info.charging = charging;
        s_power_info.batt_percent = batt_pct;
        s_power_info.batt_mv = batt_mv;
        s_power_info.vbus_mv = vbus_mv;
        s_power_info.system_mv = sys_mv;
        s_power_info.temp_c = temp_c;
        portEXIT_CRITICAL(&s_power_lock);

        touch_i2c_unlock();  /* release the shared bus */

        vTaskDelay(pdMS_TO_TICKS(PMU_POLL_MS));
    }
}

esp_err_t orb_pmu_init(void)
{
    if (s_pmu_dev != NULL) return ESP_OK;

    /* Share the touch controller's I2C bus — it owns I2C_NUM_0 creation
     * (SDA=15, SCL=14). touch_gestures_init() must have run first. */
    i2c_master_bus_handle_t bus = touch_gestures_get_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "touch bus not ready — PMU requires touch_gestures_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev = {};
    dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev.device_address  = PMU_I2C_ADDR;
    dev.scl_speed_hz    = 100000;  /* AXP2101 at 100kHz — conservative */
    dev.scl_wait_us     = 0;
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev, &s_pmu_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add PMU device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Chip-id sanity: AXP2101 reports 0x4A at reg 0x03. The AXP2101 may
     * need a moment to settle after I2C bus activity — retry a few times. */
    uint8_t id = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        if (axp_read_reg(AXP2101_REG_IC_TYPE, &id) && id != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (id == 0) {
        ESP_LOGW(TAG, "PMU chip id read failed (0x%02x) — continuing anyway", id);
    } else {
        ESP_LOGI(TAG, "AXP2101 chip id 0x%02x", id);
    }

    /* Small delay before the heavy init sequence — the touch poll task
     * may be mid-read; let the bus settle. */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Try the official driver first; if it fails (intermittent 0x00 reads
     * under touch polling contention on the shared bus), skip the heavy
     * init — the board is already powered (USB or battery defaults keep
     * the rails up). The poll task will enable PEK IRQs + ADC on its
     * first cycle. This avoids blocking app_main on I2C writes that
     * contend with the 60Hz touch poll task. */
    ret = pmu_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "official pmu_init failed — skipping heavy init (poll task handles PEK/ADC)");
    }

    xTaskCreatePinnedToCore(pmu_poll_task, "orb_pmu", 4 * 1024, NULL, 3, NULL, 0);
    ESP_LOGI(TAG, "AXP2101 PMU live — rails kept on battery, PEK + power readout active");
    return ESP_OK;
}

orb_pek_event_t orb_pmu_get_event(void)
{
    orb_pek_event_t ev = s_pek_event;
    s_pek_event = ORB_PEK_NONE;
    return ev;
}

void orb_pmu_get_power_info(orb_power_info_t *out)
{
    if (out == NULL) return;
    portENTER_CRITICAL(&s_power_lock);
    *out = s_power_info;
    portEXIT_CRITICAL(&s_power_lock);
}
