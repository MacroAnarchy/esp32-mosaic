/*
 * display_face: Mosaic node UI — the face of a sensing node.
 *
 * "PRESENCE becomes light." A particle field rendered with a direct
 * framebuffer glow engine (fx/fx_glow.h) whose density, color and motion
 * are driven by the brain state. The panel is a 466x466 QSPI AMOLED
 * (CO5300 controller).
 *
 * Panel wiring (board bring-up, Aug 12 — hardware in hand):
 *   QSPI bus: PCLK=38, DATA0..3 = 4,5,6,7 (SPI2_HOST)
 *   Panel:    CS=12, RGB565, 466x466, gap x=6, QSPI mode
 *   Flush:    esp_lcd_panel_draw_bitmap via the CO5300 driver
 *   RST:      GPIO 3
 *
 * A FreeRTOS task (core 1, moderate priority) owns the render loop at a
 * 33fps target. All public calls are thread-safe (mutex-protected state).
 */
#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_co5300.h"

#include "display_face.h"
#include "fx/fx_glow.h"

static const char *TAG = "display_face";

using namespace fx;

/* ------------------------------------------------------------------ */
/* Face configuration per state                                        */
/* ------------------------------------------------------------------ */

struct FaceStyle {
    Rgb color;
    int particle_count;
    float speed;      /* base drift speed, px/frame-ish */
    float rnd_speed;  /* random jitter amplitude */
    bool dim;         /* SLEEP: heavily dimmed */
    bool agitated;    /* ALERT: erratic fast motion */
};

static const FaceStyle kStyles[FACE_STATE_COUNT] = {
    /* FACE_IDLE       */ {{ 40, 90, 160 },  90, 0.35f, 0.25f, false, false },
    /* FACE_OWNER_NEAR */ {{ 255, 140, 40 }, 170, 0.55f, 0.40f, false, false },
    /* FACE_VOICE      */ {{ 120, 220, 120 }, 150, 0.70f, 0.60f, false, false },
    /* FACE_ALERT      */ {{ 255, 50, 50 },   200, 1.40f, 1.20f, false, true  },
    /* FACE_SLEEP      */ {{ 20, 30, 60 },    40,  0.10f, 0.05f, true,  false },
};

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */

static fx::GlowCanvas s_canvas;
static SemaphoreHandle_t s_mutex = NULL;
static volatile face_state_t s_state = FACE_IDLE;
static volatile float s_voice_energy = 0.0f;
static volatile bool s_suspended = false;

/* Particle field. */
struct Particle {
    float x, y;
    float vx, vy;
    float phase;
};
static Particle s_particles[220];  /* max count across styles */

/* ------------------------------------------------------------------ */
/* Panel (CO5300 QSPI AMOLED) — board bring-up wiring                  */
/* ------------------------------------------------------------------ */

#define MOSAIC_PANEL_SPI_HOST  SPI2_HOST
#define MOSAIC_PANEL_PCLK      GPIO_NUM_38
#define MOSAIC_PANEL_DATA0     GPIO_NUM_4
#define MOSAIC_PANEL_DATA1     GPIO_NUM_5
#define MOSAIC_PANEL_DATA2     GPIO_NUM_6
#define MOSAIC_PANEL_DATA3     GPIO_NUM_7
#define MOSAIC_PANEL_CS        GPIO_NUM_12
#define MOSAIC_PANEL_RST       GPIO_NUM_3
#define MOSAIC_PANEL_TRANS_SZ  (466 * 64 * 2)   /* 64-row band: DMA-safe */

static esp_lcd_panel_handle_t s_panel = NULL;

static void mosaic_panel_flush(const uint16_t *fb)
{
    if (s_panel == NULL) {
        return;
    }
    /* The full 466x466 frame (~434KB) exceeds internal DMA memory, so
     * push it in 64-row bands — the CO5300 accepts partial y-ranges. */
    const int band_rows = 64;
    for (int y = 0; y < kScreenH; y += band_rows) {
        int rows = band_rows;
        if (y + rows > kScreenH) {
            rows = kScreenH - y;
        }
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, kScreenW, y + rows,
                                  fb + (size_t)y * kScreenW);
    }
}

static esp_err_t mosaic_panel_init(void)
{
    esp_err_t ret;

    /* Build the QSPI bus config manually in declaration order.
     * The dataN names are union aliases; C++ designator order requires
     * the FIRST member of each anonymous union (mosi/miso/quadwp/quadhd).
     * Same memory, QSPI mode. */
    const spi_bus_config_t buscfg = {
        .mosi_io_num = MOSAIC_PANEL_DATA0,
        .miso_io_num = MOSAIC_PANEL_DATA1,
        .sclk_io_num = MOSAIC_PANEL_PCLK,
        .quadwp_io_num = MOSAIC_PANEL_DATA2,
        .quadhd_io_num = MOSAIC_PANEL_DATA3,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = MOSAIC_PANEL_TRANS_SZ,
    };
    ret = spi_bus_initialize(MOSAIC_PANEL_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = CO5300_PANEL_IO_QSPI_CONFIG(
        MOSAIC_PANEL_CS, NULL, NULL);
    io_config.trans_queue_depth = 4;
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)MOSAIC_PANEL_SPI_HOST,
                                   &io_config, &io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel io init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* CO5300 init sequence from the Waveshare BSP (board bring-up). */
    static const co5300_lcd_init_cmd_t init_cmds[] = {
        {0xFE, (uint8_t[]){0x20}, 1, 0},
        {0x19, (uint8_t[]){0x10}, 1, 0},
        {0x1C, (uint8_t[]){0xA0}, 1, 0},
        {0xFE, (uint8_t[]){0x00}, 1, 0},
        {0xC4, (uint8_t[]){0x80}, 1, 0},
        {0x3A, (uint8_t[]){0x55}, 1, 0},   /* RGB565 */
        {0x35, (uint8_t[]){0x00}, 1, 0},
        {0x53, (uint8_t[]){0x20}, 1, 0},   /* brightness ctrl on */
        {0x51, (uint8_t[]){0xFF}, 1, 0},   /* brightness 100% */
        {0x63, (uint8_t[]){0xFF}, 1, 0},
        {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xD7}, 4, 0}, /* col 6..471 */
        {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 600},/* row 0..465 */
        {0x11, NULL, 0, 600},              /* sleep out */
        {0x29, NULL, 0, 0},                /* display on */
    };
    co5300_vendor_config_t vendor_config = {
        .init_cmds = init_cmds,
        .init_cmds_size = sizeof(init_cmds) / sizeof(init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = MOSAIC_PANEL_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    ret = esp_lcd_new_panel_co5300(io, &panel_config, &s_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "co5300 panel init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_set_gap(s_panel, 0x06, 0);
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_disp_on_off(s_panel, true);
    ESP_LOGI(TAG, "CO5300 panel ready (466x466 QSPI RGB565)");
    return ESP_OK;
}

static void flush_cb(void *ctx, const uint16_t *fb)
{
    (void)ctx;
    mosaic_panel_flush(fb);
}

/* ------------------------------------------------------------------ */
/* Face state API (thread-safe)                                        */
/* ------------------------------------------------------------------ */

esp_err_t display_face_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    /* Board bring-up: bring the CO5300 panel up first. */
    esp_err_t ret = mosaic_panel_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "panel init failed (%s) — face renders framebuffer only",
                 esp_err_to_name(ret));
        /* continue: state machine still runs, flush is a no-op */
    }

    if (!s_canvas.init(nullptr, 0, flush_cb, nullptr)) {
        ESP_LOGE(TAG, "glow canvas init failed");
        return ESP_FAIL;
    }
    s_canvas.clear();

    /* Seed particles with the IDLE style. */
    const FaceStyle &st = kStyles[FACE_IDLE];
    for (int i = 0; i < st.particle_count; i++) {
        s_particles[i].x = rndf(0, kScreenW);
        s_particles[i].y = rndf(0, kScreenH);
        s_particles[i].vx = rndf(-st.speed, st.speed);
        s_particles[i].vy = rndf(-st.speed, st.speed);
        s_particles[i].phase = rndf(0, 6.28f);
    }

    ESP_LOGI(TAG, "face initialized (glow canvas %dx%d)", kScreenW, kScreenH);
    return ESP_OK;
}

esp_err_t display_face_set_state(face_state_t state)
{
    if (state >= FACE_STATE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state = state;
    if (s_mutex) xSemaphoreGive(s_mutex);
    ESP_LOGD(TAG, "state -> %d", (int)state);
    return ESP_OK;
}

void display_face_set_voice_energy(float energy)
{
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_voice_energy = energy > 1.0f ? 1.0f : (energy < 0.0f ? 0.0f : energy);
    if (s_mutex) xSemaphoreGive(s_mutex);
}

esp_err_t display_face_set_suspended(bool suspended)
{
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_suspended = suspended;
    if (s_mutex) xSemaphoreGive(s_mutex);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Render loop                                                         */
/* ------------------------------------------------------------------ */

static void render_particles(const FaceStyle &st, float dt)
{
    const int n = st.particle_count;
    for (int i = 0; i < n; i++) {
        Particle &p = s_particles[i];

        /* Drift + jitter (agitated styles move erratically). */
        p.phase += dt * (st.agitated ? 6.0f : 1.2f);
        float jx = st.rnd_speed * sinf(p.phase * 1.7f + i);
        float jy = st.rnd_speed * cosf(p.phase * 2.3f + i);
        p.x += (p.vx + jx) * dt;
        p.y += (p.vy + jy) * dt;

        /* Wrap around the round screen (distance from center <= rim). */
        float dx = p.x - kCenterX, dy = p.y - kCenterY;
        float r2 = dx * dx + dy * dy;
        if (r2 > kRimR * kRimR) {
            /* Re-seed at a random point inside the disc. */
            float ang = rndf(0, 6.28f), rad = sqrtf(rndf(0, 1.0f)) * kRimR * 0.8f;
            p.x = kCenterX + cosf(ang) * rad;
            p.y = kCenterY + sinf(ang) * rad;
            p.vx = rndf(-st.speed, st.speed);
            p.vy = rndf(-st.speed, st.speed);
        }

        /* Glow dot — voice state snaps intensity to mic energy. */
        float intensity = 0.5f;
        if (s_state == FACE_VOICE) {
            intensity = 0.3f + s_voice_energy * 1.2f;
        } else if (st.dim) {
            intensity = 0.12f;
        } else if (st.agitated) {
            intensity = 0.55f + 0.35f * sinf(p.phase);
        }
        int radius = 4 + (i % 5);
        s_canvas.addGlowDot(p.x, p.y, st.color, intensity, radius);
    }
}

static void face_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    const TickType_t frame_ms = pdMS_TO_TICKS(1000 / 33); /* 33fps */

    for (;;) {
        if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
        face_state_t state = s_state;
        bool suspended = s_suspended;
        if (s_mutex) xSemaphoreGive(s_mutex);

        s_canvas.beginFrame();

        if (!suspended) {
            const FaceStyle &st = kStyles[state];
            render_particles(st, 1.0f);
        }

        s_canvas.push();

        vTaskDelayUntil(&last, frame_ms);
    }
}

/* ------------------------------------------------------------------ */
/* Task startup                                                        */
/* ------------------------------------------------------------------ */

void display_face_start_render_task(void)
{
    xTaskCreatePinnedToCore(face_task, "face-render", 8192, NULL, 5, NULL, 1);
    ESP_LOGI(TAG, "render task started (core 1, 33fps)");
}
