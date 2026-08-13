/*
 * settings_menu: minimal settings UI for the Orb's round display.
 *
 * Rendered via M5GFX text/rect primitives (the glow canvas is for the
 * dome — text needs fillRect/drawString). The menu takes over the
 * whole screen when visible; the render task calls settings_menu_render()
 * at ~10fps instead of the dome/CSI path.
 *
 * Menu items:
 *   1. Brightness  — swipe up/down to adjust 0..100 (live setBrightness)
 *   2. WiFi        — SSID (BSSID), IP address, connected/disconnected
 *   3. Power       — battery %, voltage mV, VBUS state, charging state
 *   4. CSI Mode    — dome/merged/standalone/off (cycle on select)
 *   5. Close       — exit menu (also long-press PEK)
 *
 * Navigation: touch swipe up/down to move selection, tap to
 * select/adjust. Brightness adjusts with up/down swipes while selected.
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <M5GFX.h>

#include "settings_menu.h"
#include "display_face.h"
#include "orb_pmu.h"
#include "sense_engine.h"

static const char *TAG = "settings_menu";

/* ---- Menu state ---- */
typedef enum {
    MENU_BRIGHTNESS = 0,
    MENU_WIFI,
    MENU_POWER,
    MENU_CSI_MODE,
    MENU_CLOSE,
    MENU_ITEM_COUNT
} menu_item_t;

static const char *kCsiModeNames[] = {
    "Off (dome)", "Merged", "Standalone"
};

static SemaphoreHandle_t s_menu_mutex = NULL;
static volatile bool s_menu_visible = false;
static volatile int  s_selected = 0;
static volatile int  s_brightness = 50;

/* Layout constants for the 466x466 round screen. */
static constexpr int kTitleY     = 30;
static constexpr int kItemStartY = 95;
static constexpr int kItemH      = 62;
static constexpr int kItemX      = 40;
static constexpr int kItemW      = 386;
static constexpr int kTextSize   = 2;   /* M5GFX font size 2 = ~16px */
static constexpr int kTextSmall  = 1;   /* small text = ~8px */

static M5GFX *get_display(void)
{
    return (M5GFX *)display_face_get_display();
}

esp_err_t settings_menu_init(void)
{
    if (s_menu_mutex == NULL) {
        s_menu_mutex = xSemaphoreCreateMutex();
        if (s_menu_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    s_brightness = display_face_get_brightness();
    ESP_LOGI(TAG, "settings menu initialized");
    return ESP_OK;
}

void settings_menu_set_visible(bool visible)
{
    if (s_menu_mutex) xSemaphoreTake(s_menu_mutex, portMAX_DELAY);
    s_menu_visible = visible;
    if (s_menu_mutex) xSemaphoreGive(s_menu_mutex);
    ESP_LOGI(TAG, "menu %s", visible ? "visible" : "hidden");
}

bool settings_menu_is_visible(void)
{
    return s_menu_visible;
}

void settings_menu_set_brightness(int pct)
{
    if (pct < 5) pct = 5;   /* floor at 5% — 0% blanks the display */
    if (pct > 100) pct = 100;
    if (s_menu_mutex) xSemaphoreTake(s_menu_mutex, portMAX_DELAY);
    s_brightness = pct;
    if (s_menu_mutex) xSemaphoreGive(s_menu_mutex);
    display_face_set_brightness(pct);
}

int settings_menu_get_brightness(void)
{
    return s_brightness;
}

void settings_menu_navigate(int direction)
{
    if (s_menu_mutex) xSemaphoreTake(s_menu_mutex, portMAX_DELAY);
    if (direction == -1) {
        /* up */
        s_selected = (s_selected - 1 + MENU_ITEM_COUNT) % MENU_ITEM_COUNT;
    } else if (direction == 1) {
        /* down */
        s_selected = (s_selected + 1) % MENU_ITEM_COUNT;
    } else if (direction == 0) {
        /* select/tap */
        int sel = s_selected;
        if (s_menu_mutex) xSemaphoreGive(s_menu_mutex);
        switch (sel) {
        case MENU_BRIGHTNESS:
            /* tap on brightness does nothing special (swipe adjusts) */
            break;
        case MENU_CSI_MODE: {
            /* cycle CSI mode */
            int cur = (int)display_face_get_csi_mode();
            int next = (cur + 1) % CSI_MODE_COUNT;
            display_face_set_csi_mode(next);
            break;
        }
        case MENU_CLOSE:
            settings_menu_set_visible(false);
            break;
        default:
            break;
        }
        return;
    }
    if (s_menu_mutex) xSemaphoreGive(s_menu_mutex);
}

/* Draw a menu item row with a label and value. */
static void draw_item(M5GFX *d, int idx, const char *label,
                      const char *value, bool selected)
{
    int y = kItemStartY + idx * kItemH;
    uint16_t bg = selected ? 0x2104 : 0x0000;  /* dark blue if selected, black otherwise */
    uint16_t fg = selected ? 0xFFFF : 0x8410;  /* white if selected, gray otherwise */

    d->fillRect(kItemX, y, kItemW, kItemH - 6, bg);
    if (selected) {
        /* highlight bar on the left edge */
        d->fillRect(kItemX, y, 4, kItemH - 6, 0x07FF);  /* cyan accent */
    }

    d->setTextColor(fg, bg);
    d->setTextSize(kTextSize);
    d->setCursor(kItemX + 14, y + 8);
    d->print(label);

    /* Value on the right side, smaller */
    d->setTextSize(kTextSmall);
    d->setTextColor(selected ? 0xB5F6 : 0x4208, bg);
    d->setCursor(kItemX + 14, y + 32);
    d->print(value);
}

/* Brightness bar — a horizontal progress bar. */
static void draw_brightness_bar(M5GFX *d, int y)
{
    int barX = kItemX + 14;
    int barW = kItemW - 28;
    int barH = 8;
    d->fillRect(barX, y, barW, barH, 0x2104);  /* dark bg */
    int fillW = (barW * s_brightness) / 100;
    uint16_t color;
    if (s_brightness > 66) color = 0x07E0;  /* green */
    else if (s_brightness > 33) color = 0xFFE0; /* yellow */
    else color = 0xF800;  /* red */
    d->fillRect(barX, y, fillW, barH, color);
}

void settings_menu_render(void)
{
    M5GFX *d = get_display();
    if (d == NULL) return;

    if (s_menu_mutex) xSemaphoreTake(s_menu_mutex, portMAX_DELAY);
    int sel = s_selected;
    int bright = s_brightness;
    if (s_menu_mutex) xSemaphoreGive(s_menu_mutex);

    /* Clear to black */
    display_face_clear_frame();

    /* Title */
    d->setTextColor(0x07FF, 0x0000);  /* cyan on black */
    d->setTextSize(kTextSize);
    d->setCursor(140, kTitleY);
    d->print("SETTINGS");

    /* Divider line */
    d->drawFastHLine(60, kTitleY + 24, 346, 0x2104);

    /* ---- Gather live data ---- */
    char val[64];

    /* Brightness */
    snprintf(val, sizeof(val), "%d%%", bright);
    draw_item(d, MENU_BRIGHTNESS, "Brightness", val, sel == MENU_BRIGHTNESS);
    if (sel == MENU_BRIGHTNESS) {
        draw_brightness_bar(d, kItemStartY + MENU_BRIGHTNESS * kItemH + 44);
    }

    /* WiFi status */
    char wifi_val[96] = {};
    bool connected = sense_wifi_is_connected();
    if (connected) {
        uint8_t ip[4] = {}, mask[4] = {};
        char bssid[32] = {};
        sense_wifi_get_ipv4(ip, mask);
        sense_wifi_get_ap_bssid(bssid, sizeof(bssid));
        snprintf(wifi_val, sizeof(wifi_val), "CONNECTED\nIP: %d.%d.%d.%d\n%s",
                 ip[0], ip[1], ip[2], ip[3], bssid);
    } else {
        snprintf(wifi_val, sizeof(wifi_val), "DISCONNECTED");
    }
    draw_item(d, MENU_WIFI, "WiFi", wifi_val, sel == MENU_WIFI);

    /* Power management */
    orb_power_info_t pwr;
    orb_pmu_get_power_info(&pwr);
    if (!pwr.pmu_ok) {
        snprintf(val, sizeof(val), "PMU not init");
    } else if (!pwr.batt_present) {
        if (pwr.vbus_present) {
            snprintf(val, sizeof(val), "NO BATTERY\nUSB: %dmV\n%s",
                     pwr.vbus_mv, pwr.charging ? "charging" : "powered");
        } else {
            snprintf(val, sizeof(val), "NO BATTERY\nNO USB");
        }
    } else {
        snprintf(val, sizeof(val), "%d%%  %dmV%s\nVBUS: %dmV  %.0fC",
                 pwr.batt_percent, pwr.batt_mv,
                 pwr.charging ? " (chg)" : "",
                 pwr.vbus_mv, pwr.temp_c);
    }
    draw_item(d, MENU_POWER, "Power", val, sel == MENU_POWER);

    /* CSI mode */
    int csi = (int)display_face_get_csi_mode();
    const char *mode_name = (csi >= 0 && csi < 3) ? kCsiModeNames[csi] : "?";
    snprintf(val, sizeof(val), "%s  (tap=cycle)", mode_name);
    draw_item(d, MENU_CSI_MODE, "CSI Mode", val, sel == MENU_CSI_MODE);

    /* Close */
    draw_item(d, MENU_CLOSE, "Close Menu", "long-press PEK", sel == MENU_CLOSE);

    /* Footer hint */
    d->setTextSize(kTextSmall);
    d->setTextColor(0x4208, 0x0000);
    d->setCursor(80, kItemStartY + MENU_ITEM_COUNT * kItemH + 10);
    d->print("Swipe up/down: navigate   Tap: select");

    /* Flush the frame to the panel */
    display_face_flush_frame();
}
