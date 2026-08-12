/*
 * display_face: Mosaic node UI — the face of a sensing node.
 *
 * "PRESENCE becomes light." A living aurora dome — THE RADIO DOME —
 * rendered with a direct framebuffer glow engine (fx/fx_glow.h) whose
 * rings, orbs and particle streams are driven by the node's OWN senses:
 * the local BLE device table (onboard only, no gateway).
 *
 *   CENTER = the node itself (the receiver, "I" of the system)
 *   RINGS  = RSSI distance zones (near / mid / far)
 *   ORBS   = one per sensed device, colored by device class, at a
 *            MAC-stable angle so devices never jitter
 *   MOTION = discovery bursts fly out from center, pulse streams flow
 *            center->device while present, soft implosions on departure
 *
 * The panel is a 466x466 QSPI AMOLED (CO5300 controller) driven directly
 * through esp_lcd — no board-support package. A FreeRTOS task (core 1,
 * moderate priority) owns the render loop at a 33fps target. All public
 * calls are thread-safe and may be used from any task.
 *
 * C-callable so the app shell (C or C++) can drive it.
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
#include "sense_engine.h"

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

/* Ambient dust field (the calm empty-state base). */
struct Particle {
    float x, y;
    float vx, vy;
    float phase;
};
static Particle s_particles[220];  /* max count across styles */

/* ------------------------------------------------------------------ */
/* The Radio Dome — local BLE device table rendered as a living dome    */
/* ------------------------------------------------------------------ */

#define DOME_MAX_DEVICES 64   /* matches the sense engine's record cap */
#define DOME_POOL_SIZE   288  /* stream / burst / implosion particles  */

/* RSSI -> ring-radius mapping: stronger signal = smaller radius.
 *   -42 dBm -> r=55 (near, inner) ... -90 dBm -> r=199 (far, rim)   */
#define DOME_RSSI_NEAR   (-42.0f)
#define DOME_RSSI_FAR    (-90.0f)
#define DOME_RAD_MIN     48.0f
#define DOME_RAD_MAX     200.0f

/* Ring zone radii — the RSSI thresholds fall exactly on the rings. */
static const float kRingR[3] = { 94.0f, 154.0f, 199.0f };
static const Rgb kRingColor[3] = {
    { 70, 150, 255 },   /* near: ice blue  */
    { 110, 130, 255 },  /* mid: periwinkle */
    { 150, 110, 255 },  /* far: soft violet */
};

/* Device class -> palette (gorgeous-on-OLED hues). */
struct ClassPalette {
    Rgb glow;  /* soft halo hue */
    Rgb core;  /* hot center hue */
};

static const ClassPalette kClassPaletteFindmy = { { 90, 205, 255 },  { 235, 250, 255 } };  /* ice cyan   */
static const ClassPalette kClassPaletteMeta   = { { 185, 115, 255 }, { 248, 232, 255 } };  /* violet     */
static const ClassPalette kClassPaletteFlipper= { { 255, 82, 40 },  { 255, 235, 215 } };   /* hot ember  */
static const ClassPalette kClassPaletteUnknown= { { 122, 142, 172 },{ 218, 228, 242 } };   /* soft slate */

/* Fade timings (frames at 33fps). */
static const float kDomeFadeInFrames = 15.0f;   /* ~0.45s orb fade-in  */
static const float kDomeFadeOutFrames = 23.0f;  /* ~0.70s orb fade-out */

/* One orb on the dome — a sensed device with a MAC-stable position. */
struct DomeDevice {
    char mac[18];
    uint8_t active;      /* slot in use */
    uint8_t phase;       /* 0 NEW (fade in), 1 LIVE, 2 DYING (fade out) */
    float age;           /* seconds (frames) in current phase */
    float baseAngle;     /* stable MAC-derived angle (rad) */
    float driftAmp;      /* slow sway amplitude (rad) */
    float driftPhase;
    float rssi;          /* smoothed rssi */
    float targetRssi;    /* latest from the sense table */
    float radius;        /* current ring radius (eased) */
    float alpha;         /* orb opacity 0..1 */
    float pulsePhase;
    uint8_t sizeClass;   /* 0..3 from MAC hash */
    Rgb glow, core;
};
static DomeDevice s_dome[DOME_MAX_DEVICES];

/* Flowing particles: discovery bursts (center->device), pulse streams
 * (center->device while present), implosions (device->center on loss). */
enum DomePKind { DOME_P_STREAM = 0, DOME_P_BURST, DOME_P_IMPLODE };

struct DomeParticle {
    float x, y;          /* current position */
    float tx, ty;        /* travel target */
    float u, speed;      /* 0..1 progress along the trip */
    float px, py;        /* unit perpendicular of the trip */
    float wobAmp, wobFreq, wobPhase;
    float age, life;
    uint8_t r, g, b;
    uint8_t radius;
    uint8_t intensity;   /* 0..255 */
    uint8_t kind;
    uint8_t alive;
};
static DomeParticle s_pool[DOME_POOL_SIZE];

static sense_device_t s_snap[DOME_MAX_DEVICES];  /* sense table snapshot */
static uint32_t s_frame = 0;                     /* render frame counter */
static float s_time = 0.0f;                      /* frames as float */
static float s_sweepAngle = 0.0f;                /* radar sweep (rad) */

/* ------------------------------------------------------------------ */
/* Vignette for the round cutout — soft edge falloff at the rim.       */
/* ------------------------------------------------------------------ */

#define VIGN_OUTER_R 226.0f   /* == kRimR: hard clip edge */
#define VIGN_INNER_R 218.0f   /* full-bright disc radius  */

struct VignRow {
    int16_t z0;  /* [0, z0)      : zero (outside disc)   */
    int16_t f0;  /* [z0, f0)     : falloff (left side)   */
    int16_t f1;  /* [f0, f1)     : untouched             */
    int16_t z1;  /* [f1, z1)     : falloff (right side)  */
                 /* [z1, W)      : zero                  */
};
static VignRow s_vign[kScreenH];
static uint8_t s_vignFactor[512];  /* radial falloff LUT (d -> 0..255) */

static void vignette_build(void)
{
    const float rOut2 = VIGN_OUTER_R * VIGN_OUTER_R;
    const float rIn2 = VIGN_INNER_R * VIGN_INNER_R;

    for (int d = 0; d < 512; d++) {
        float t = ((float)d - VIGN_INNER_R) / (VIGN_OUTER_R - VIGN_INNER_R);
        if (t <= 0.0f) {
            s_vignFactor[d] = 255;
        } else if (t >= 1.0f) {
            s_vignFactor[d] = 0;
        } else {
            /* smoothstep 1 -> 0 across the falloff band */
            float f = 1.0f - t * t * (3.0f - 2.0f * t);
            s_vignFactor[d] = (uint8_t)(f * 255.0f);
        }
    }

    for (int y = 0; y < kScreenH; y++) {
        float dy = (float)y - kCenterY;
        float dy2 = dy * dy;
        VignRow &v = s_vign[y];
        if (dy2 > rOut2) {
            v.z0 = 0; v.f0 = 0; v.f1 = kScreenW; v.z1 = kScreenW;
            continue;
        }
        int o = (int)sqrtf(rOut2 - dy2);   /* outer half-width */
        int i = dy2 > rIn2 ? 0 : (int)sqrtf(rIn2 - dy2);
        v.z0 = (int16_t)(kCenterX - o);
        v.f0 = (int16_t)(kCenterX - i);
        v.f1 = (int16_t)(kCenterX + i);
        v.z1 = (int16_t)(kCenterX + o);
    }
}

/* Soften the disc edge for the round lens. fb is the canvas's own
 * framebuffer; the mask is static so it is idempotent (dirty-rect
 * erases only ever touch content inside the disc). */
static void vignette_apply(uint16_t *fb)
{
    for (int y = 0; y < kScreenH; y++) {
        const VignRow &v = s_vign[y];
        if (v.z0 <= 0 && v.z1 >= kScreenW) {
            continue;  /* fully inside the disc — nothing to do */
        }
        uint16_t *row = fb + (size_t)y * kScreenW;
        if (v.z0 > 0) {
            memset(row, 0, (size_t)v.z0 * sizeof(uint16_t));
        }
        if (v.z1 < kScreenW) {
            memset(row + v.z1, 0, (size_t)(kScreenW - v.z1) * sizeof(uint16_t));
        }
        const float dy = (float)y - kCenterY;
        const float dy2 = dy * dy;
        /* left falloff band [z0, f0) */
        for (int x = v.z0; x < v.f0; x++) {
            float dx = (float)x - kCenterX;
            int d = (int)sqrtf(dx * dx + dy2);
            uint8_t f = s_vignFactor[d > 511 ? 511 : d];
            uint16_t px = row[x];
            int rr = ((px >> 11) & 0x1F) * f >> 8;
            int gg = ((px >> 5) & 0x3F) * f >> 8;
            int bb = (px & 0x1F) * f >> 8;
            row[x] = (uint16_t)((rr << 11) | (gg << 5) | bb);
        }
        /* right falloff band [f1, z1) */
        for (int x = v.f1; x < v.z1; x++) {
            float dx = (float)x - kCenterX;
            int d = (int)sqrtf(dx * dx + dy2);
            uint8_t f = s_vignFactor[d > 511 ? 511 : d];
            uint16_t px = row[x];
            int rr = ((px >> 11) & 0x1F) * f >> 8;
            int gg = ((px >> 5) & 0x3F) * f >> 8;
            int bb = (px & 0x1F) * f >> 8;
            row[x] = (uint16_t)((rr << 11) | (gg << 5) | bb);
        }
    }
}

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

/* Persistent DMA-capable band buffer: the PSRAM canvas is not DMA-safe
 * for the SPI controller, and allocating a private TX buffer per
 * transaction fragments internal RAM until one 59KB alloc fails
 * (frozen screen after ~6 min). One pre-allocated band = zero churn. */
static uint16_t *s_dmaBand = NULL;
static const int kBandRows = 64;

static void mosaic_panel_flush(uint16_t *fb)
{
    if (s_panel == NULL || s_dmaBand == NULL) {
        return;
    }
    /* Round cutout: soften the disc edge once per frame, in place. */
    vignette_apply(fb);
    /* Push 64-row bands through the persistent DMA buffer — the CO5300
     * accepts partial y-ranges, and the copy is cheap (~59KB memcpy). */
    for (int y = 0; y < kScreenH; y += kBandRows) {
        int rows = kBandRows;
        if (y + rows > kScreenH) {
            rows = kScreenH - y;
        }
        memcpy(s_dmaBand, fb + (size_t)y * kScreenW,
               (size_t)rows * kScreenW * sizeof(uint16_t));
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, kScreenW, y + rows,
                                  s_dmaBand);
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

    /* CO5300 GRAM is taller than the visible 466×466 round area; the
     * physical center sits 70 rows below the GRAM origin. Without the
     * y-gap every implementation renders ~70px too high. The known-good
     * value across the ecosystem is 70.
     */
    esp_lcd_panel_set_gap(s_panel, 0x06, 70);
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_disp_on_off(s_panel, true);
    ESP_LOGI(TAG, "CO5300 panel ready (466x466 QSPI RGB565)");
    return ESP_OK;
}

static void flush_cb(void *ctx, const uint16_t *fb)
{
    (void)ctx;
    /* The vignette masks the canvas's own framebuffer in place (round
     * cutout softness) — the canvas owns it and never reads it back. */
    mosaic_panel_flush(const_cast<uint16_t *>(fb));
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

    /* Persistent DMA band — one 59KB internal-RAM block, allocated ONCE.
     * Without it the SPI driver allocates a priv TX buffer per band
     * transaction and internal RAM fragments until a frame dies. */
    if (s_dmaBand == NULL) {
        s_dmaBand = (uint16_t *)heap_caps_malloc(
            (size_t)kBandRows * kScreenW * sizeof(uint16_t),
            MALLOC_CAP_DMA);
        if (s_dmaBand == NULL) {
            ESP_LOGE(TAG, "DMA band alloc failed — display will freeze");
        } else {
            ESP_LOGI(TAG, "DMA band ready (%u bytes)",
                     (unsigned)(kBandRows * kScreenW * sizeof(uint16_t)));
        }
    }

    if (!s_canvas.init(nullptr, 0, flush_cb, nullptr)) {
        ESP_LOGE(TAG, "glow canvas init failed");
        return ESP_FAIL;
    }
    s_canvas.clear();

    /* Round cutout vignette (precomputed per-row bounds + falloff LUT). */
    vignette_build();

    /* Seed particles with the IDLE style. */
    const FaceStyle &st = kStyles[FACE_IDLE];
    for (int i = 0; i < st.particle_count; i++) {
        s_particles[i].x = rndf(0, kScreenW);
        s_particles[i].y = rndf(0, kScreenH);
        s_particles[i].vx = rndf(-st.speed, st.speed);
        s_particles[i].vy = rndf(-st.speed, st.speed);
        s_particles[i].phase = rndf(0, 6.28f);
    }

    ESP_LOGI(TAG, "face initialized (radio dome %dx%d, %d-device table)",
             kScreenW, kScreenH, DOME_MAX_DEVICES);
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
/* Dome helpers                                                        */
/* ------------------------------------------------------------------ */

/* FNV-1a over the MAC string — stable per-device identity. */
static uint32_t dome_hash_mac(const char *mac)
{
    uint32_t h = 2166136261u;
    for (const char *p = mac; *p; p++) {
        h ^= (uint8_t)*p;
        h *= 16777619u;
    }
    return h;
}

static void dome_class_palette(const char *cls, Rgb &glow, Rgb &core)
{
    if (strcmp(cls, "findmy") == 0) {
        glow = kClassPaletteFindmy.glow; core = kClassPaletteFindmy.core;
    } else if (strcmp(cls, "meta") == 0) {
        glow = kClassPaletteMeta.glow; core = kClassPaletteMeta.core;
    } else if (strcmp(cls, "flipper") == 0) {
        glow = kClassPaletteFlipper.glow; core = kClassPaletteFlipper.core;
    } else {
        glow = kClassPaletteUnknown.glow; core = kClassPaletteUnknown.core;
    }
}

static inline float dome_radius_for_rssi(float rssi)
{
    float r = DOME_RAD_MIN + (DOME_RSSI_NEAR - rssi) *
                             (DOME_RAD_MAX - DOME_RAD_MIN) /
                             (DOME_RSSI_NEAR - DOME_RSSI_FAR);
    if (r < DOME_RAD_MIN) r = DOME_RAD_MIN;
    if (r > DOME_RAD_MAX) r = DOME_RAD_MAX;
    return r;
}

static DomeParticle *dome_alloc_particle(void)
{
    for (int i = 0; i < DOME_POOL_SIZE; i++) {
        if (!s_pool[i].alive) {
            s_pool[i].alive = 1;
            return &s_pool[i];
        }
    }
    return NULL;  /* pool full — drop (lossy by design) */
}

static void dome_spawn_burst(const DomeDevice &d)
{
    uint32_t h = dome_hash_mac(d.mac);
    int cnt = 14 + (int)(h % 7);  /* 14..20 shards */
    for (int k = 0; k < cnt; k++) {
        DomeParticle *p = dome_alloc_particle();
        if (p == NULL) break;
        float a = d.baseAngle + rndf(-0.14f, 0.14f);
        float rr = d.radius + rndf(-3.0f, 5.0f);
        p->kind = DOME_P_BURST;
        p->age = 0.0f;
        p->life = 40.0f;
        p->u = rndf(0.0f, 0.22f);           /* some shards already flying */
        p->speed = rndf(0.045f, 0.075f);
        p->tx = kCenterX + cosf(a) * rr;
        p->ty = kCenterY + sinf(a) * rr;
        p->px = -sinf(a); p->py = cosf(a);
        p->wobAmp = rndf(0.5f, 2.5f);
        p->wobFreq = 0.0f;
        p->wobPhase = rndf(0.0f, 6.28f);
        p->r = d.glow.r; p->g = d.glow.g; p->b = d.glow.b;
        p->radius = 2;
        p->intensity = 210;
    }
}

static void dome_spawn_implosion(const DomeDevice &d)
{
    uint32_t h = dome_hash_mac(d.mac);
    int cnt = 10 + (int)((h >> 3) % 6);  /* 10..15 shards */
    for (int k = 0; k < cnt; k++) {
        DomeParticle *p = dome_alloc_particle();
        if (p == NULL) break;
        float a = d.baseAngle + rndf(-0.30f, 0.30f);
        float rr = d.radius * rndf(0.94f, 1.06f);
        p->kind = DOME_P_IMPLODE;
        p->age = 0.0f;
        p->life = 36.0f;
        p->u = rndf(0.0f, 0.15f);
        p->speed = rndf(0.05f, 0.09f);
        p->x = kCenterX + cosf(a) * rr;    /* start: at the orb */
        p->y = kCenterY + sinf(a) * rr;
        p->tx = kCenterX + rndf(-26.0f, 26.0f);  /* target: center cluster */
        p->ty = kCenterY + rndf(-26.0f, 26.0f);
        p->px = -sinf(a); p->py = cosf(a);
        p->wobAmp = rndf(0.0f, 2.0f);
        p->wobFreq = 0.0f;
        p->wobPhase = rndf(0.0f, 6.28f);
        p->r = (uint8_t)(d.glow.r * 0.85f);
        p->g = (uint8_t)(d.glow.g * 0.85f);
        p->b = (uint8_t)(d.glow.b * 0.85f);
        p->radius = 3;
        p->intensity = 200;
    }
}

static void dome_spawn_stream(const DomeDevice &d, float angle)
{
    DomeParticle *p = dome_alloc_particle();
    if (p == NULL) return;
    uint32_t h = dome_hash_mac(d.mac);
    float frac = (float)((h >> 4) & 0xFF) / 255.0f;
    float a = angle;
    float rr = d.radius;
    p->kind = DOME_P_STREAM;
    p->age = 0.0f;
    p->life = 40.0f;
    p->u = 0.0f;
    p->speed = 0.045f + frac * 0.035f;   /* 12..22 frames per trip */
    p->tx = kCenterX + cosf(a) * rr;
    p->ty = kCenterY + sinf(a) * rr;
    p->px = -sinf(a); p->py = cosf(a);
    p->wobAmp = 2.5f + frac * 5.0f;
    p->wobFreq = 0.5f + frac * 0.6f;
    p->wobPhase = frac * 6.28f;
    p->r = (uint8_t)(d.glow.r * 0.72f);
    p->g = (uint8_t)(d.glow.g * 0.72f);
    p->b = (uint8_t)(d.glow.b * 0.72f);
    p->radius = 1 + (int)((h >> 9) & 1); /* 1 or 2 */
    p->intensity = (uint8_t)(110 + ((h >> 10) % 3) * 25);
}

/* Diff the sense table against the dome; fire arrival/departure
 * transitions (bursts / implosions). Returns the device count. */
static int dome_refresh(void)
{
    int n = sense_engine_get_devices(s_snap, DOME_MAX_DEVICES);

    bool seen[DOME_MAX_DEVICES];
    memset(seen, 0, sizeof(seen));

    for (int i = 0; i < n; i++) {
        int idx = -1;
        for (int j = 0; j < DOME_MAX_DEVICES; j++) {
            if (s_dome[j].active && strcmp(s_dome[j].mac, s_snap[i].mac) == 0) {
                idx = j;
                break;
            }
        }
        if (idx >= 0) {
            DomeDevice &d = s_dome[idx];
            seen[idx] = true;
            if (d.phase == 2) {
                /* reappeared before the fade-out finished — revive + burst */
                d.phase = 1;
                d.age = 0.0f;
                dome_spawn_burst(d);
            }
            d.targetRssi = s_snap[i].rssi;
            dome_class_palette(s_snap[i].deviceClass, d.glow, d.core);
        } else {
            int free = -1;
            for (int j = 0; j < DOME_MAX_DEVICES; j++) {
                if (!s_dome[j].active) { free = j; break; }
            }
            if (free < 0) continue;  /* table full — skip this arrival */
            DomeDevice &d = s_dome[free];
            strncpy(d.mac, s_snap[i].mac, sizeof(d.mac) - 1);
            d.mac[sizeof(d.mac) - 1] = '\0';
            d.active = 1;
            d.phase = 0;
            d.age = 0.0f;
            d.targetRssi = s_snap[i].rssi;
            d.rssi = s_snap[i].rssi;
            d.radius = dome_radius_for_rssi(d.rssi);
            d.alpha = 0.0f;
            uint32_t h = dome_hash_mac(d.mac);
            d.baseAngle = (float)(h % 6283) * 0.001f;   /* 0..2pi */
            d.driftAmp = 0.06f + (float)((h >> 7) % 100) * 0.0014f;
            d.driftPhase = (float)((h >> 13) % 6283) * 0.001f;
            d.pulsePhase = (float)((h >> 19) % 6283) * 0.001f;
            d.sizeClass = (uint8_t)((h >> 24) & 3);
            dome_class_palette(s_snap[i].deviceClass, d.glow, d.core);
            seen[free] = true;
            dome_spawn_burst(d);   /* discovery burst: center -> orb */
        }
    }

    for (int j = 0; j < DOME_MAX_DEVICES; j++) {
        DomeDevice &d = s_dome[j];
        if (!d.active) continue;
        d.age += 1.0f;                     /* one owner of age: this loop */
        if (d.phase == 0 && d.age > kDomeFadeInFrames) {
            d.phase = 1;                   /* fade-in done */
            d.age = 0.0f;
        }
        if (!seen[j] && d.phase != 2) {
            d.phase = 2;                   /* gone: fade out + implode */
            d.age = 0.0f;
            dome_spawn_implosion(d);
        }
        if (d.phase == 2 && d.age > kDomeFadeOutFrames) {
            d.active = 0;                  /* fully faded — free the slot */
        }
    }

    /* Pulse streams: each live device emits a particle every 8 frames. */
    for (int j = 0; j < DOME_MAX_DEVICES; j++) {
        DomeDevice &d = s_dome[j];
        if (d.active && d.phase == 1 && ((s_frame + j * 3) % 8) == 0) {
            float a = d.baseAngle +
                      d.driftAmp * sinf(s_time * 0.011f + d.driftPhase);
            dome_spawn_stream(d, a);
        }
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* Render helpers                                                      */
/* ------------------------------------------------------------------ */

static inline float dome_ease_out_cubic(float u)
{
    float v = 1.0f - u;
    return 1.0f - v * v * v;
}

static inline float dome_ease_in_cubic(float u)
{
    return u * u * u;
}

/* Ambient dust — the calm empty-state field. Dims when the dome is
 * populated so the orbs read clearly. */
static void render_dust(const FaceStyle &st, int deviceCount, float gain)
{
    const int n = st.particle_count;
    float domeGain = deviceCount > 0 ? 0.5f : 1.0f;
    for (int i = 0; i < n; i++) {
        Particle &p = s_particles[i];

        /* Drift + jitter (agitated styles move erratically). */
        p.phase += st.agitated ? 6.0f : 1.2f;
        float jx = st.rnd_speed * sinf(p.phase * 1.7f + i);
        float jy = st.rnd_speed * cosf(p.phase * 2.3f + i);
        p.x += p.vx + jx;
        p.y += p.vy + jy;

        /* Wrap around the round screen (distance from center <= rim). */
        float dx = p.x - kCenterX, dy = p.y - kCenterY;
        float r2 = dx * dx + dy * dy;
        if (r2 > kRimR * kRimR) {
            float ang = rndf(0, 6.28f), rad = sqrtf(rndf(0, 1.0f)) * kRimR * 0.8f;
            p.x = kCenterX + cosf(ang) * rad;
            p.y = kCenterY + sinf(ang) * rad;
            p.vx = rndf(-st.speed, st.speed);
            p.vy = rndf(-st.speed, st.speed);
        }

        /* Glow dot — voice state snaps intensity to mic energy. */
        float intensity = 0.5f * domeGain * gain;
        if (st.dim) {
            intensity *= 0.24f;
        } else if (st.agitated) {
            intensity *= 0.55f + 0.35f * sinf(p.phase);
        }
        int radius = 3 + (i % 4);
        s_canvas.addGlowDot(p.x, p.y, st.color, intensity, radius);
    }
}

/* The three RSSI zone rings — soft glow with a slow standing-wave
 * shimmer, brightened just behind the radar sweep. */
static void render_rings(float sweepAngle, float gain)
{
    static const int kDots[3] = { 100, 160, 208 };
    for (int k = 0; k < 3; k++) {
        float r = kRingR[k];
        float baseI = (0.13f - k * 0.015f) * gain;
        for (int i = 0; i < kDots[k]; i++) {
            float a = (float)i / kDots[k] * 6.2831853f;
            float shimmer = 0.72f + 0.28f *
                           sinf(a * 3.0f + s_time * 0.021f + k * 1.7f);
            float intensity = baseI * shimmer;
            /* sweep proximity boost */
            float da = sweepAngle - a;
            while (da > 3.14159f) da -= 6.28318f;
            while (da < -3.14159f) da += 6.28318f;
            if (da > -0.9f && da < 0.9f) {
                intensity *= 1.0f + 0.45f * (1.0f - fabsf(da) / 0.9f);
            }
            s_canvas.addGlowDot(kCenterX + cosf(a) * r,
                                kCenterY + sinf(a) * r,
                                kRingColor[k], intensity, 2);
        }
    }
}

/* Rotating comet sweep — nested arc segments fading behind the edge. */
static void render_sweep(float gain)
{
    static const float kArcR[4] = { 60.0f, 105.0f, 150.0f, 195.0f };
    for (int k = 0; k < 4; k++) {
        float r = kArcR[k];
        for (float da = 0.0f; da <= 0.85f; da += 0.05f) {
            float a = s_sweepAngle - da;
            float fade = 1.0f - da / 0.85f;
            float intensity = 0.17f * fade * (0.85f - k * 0.10f) * gain;
            if (intensity <= 0.0f) continue;
            s_canvas.addGlowDot(kCenterX + cosf(a) * r,
                                kCenterY + sinf(a) * r,
                                { 160, 210, 255 }, intensity, 2);
        }
        /* bright leading-edge spot */
        s_canvas.addGlowDot(kCenterX + cosf(s_sweepAngle) * r,
                            kCenterY + sinf(s_sweepAngle) * r,
                            { 200, 235, 255 }, 0.30f * gain, 3);
    }
}

/* The node itself — a hot white core with a soft breathing halo. */
static void render_center(float gain)
{
    float pulse = 0.5f + 0.5f * sinf(s_time * 0.052f);
    s_canvas.addGlowDot(kCenterX, kCenterY, { 165, 205, 255 },
                        0.26f + 0.14f * pulse, 12);
    s_canvas.addGlowDot(kCenterX, kCenterY, { 255, 255, 255 },
                        0.55f + 0.25f * pulse, 3);

    /* CALIBRATION OVERLAY — first 90 frames (~2.7s): center dot, crosshair,
     * and the assumed visible-circle edge. Shows where the software thinks
     * the panel center is, vs the physical glass. */
    if (s_frame < 90) {
        for (int a = 0; a < 360; a++) {
            float ang = (float)a / 360.0f * 6.2831853f;
            s_canvas.addGlowDot(kCenterX + cosf(ang) * 226.0f,
                                kCenterY + sinf(ang) * 226.0f,
                                { 255, 255, 255 }, 0.9f, 2);
            s_canvas.addGlowDot(kCenterX + cosf(ang) * 113.0f,
                                kCenterY + sinf(ang) * 113.0f,
                                { 120, 200, 255 }, 0.5f, 2);
        }
        for (int d = -200; d <= 200; d += 20) {
            s_canvas.addGlowDot(kCenterX + d, kCenterY, { 255, 255, 255 }, 0.7f, 2);
            s_canvas.addGlowDot(kCenterX, kCenterY + d, { 255, 255, 255 }, 0.7f, 2);
        }
    }
}

/* The orbs: one per device, MAC-stable angle, RSSI-driven radius,
 * class-colored halo + hot core + off-center specular. */
static void render_orbs(face_state_t state, float gain)
{
    float jitter = (state == FACE_ALERT) ? 2.4f : 0.0f;
    for (int j = 0; j < DOME_MAX_DEVICES; j++) {
        DomeDevice &d = s_dome[j];
        if (!d.active) continue;

        /* ease rssi -> radius (no jumps when the table refreshes) */
        d.rssi += (d.targetRssi - d.rssi) * 0.06f;
        d.radius = dome_radius_for_rssi(d.rssi);
        /* age advances in dome_refresh (single owner); map to alpha. */
        if (d.phase == 0) {
            d.alpha = d.age / kDomeFadeInFrames;   /* fade in */
            if (d.alpha > 1.0f) d.alpha = 1.0f;
        } else if (d.phase == 2) {
            d.alpha = 1.0f - (d.age - 1.0f) / kDomeFadeOutFrames;  /* fade out */
            if (d.alpha < 0.0f) d.alpha = 0.0f;
        } else {
            d.alpha = 1.0f;
        }
        if (d.alpha <= 0.01f) continue;

        float a = d.baseAngle +
                  d.driftAmp * sinf(s_time * 0.011f + d.driftPhase);
        float rr = d.radius;
        float x = kCenterX + cosf(a) * rr;
        float y = kCenterY + sinf(a) * rr;
        if (jitter > 0.0f) {
            x += rndf(-jitter, jitter);
            y += rndf(-jitter, jitter);
        }

        /* slow breathe — near devices pulse a touch faster */
        float breathe = 1.0f + 0.09f * sinf(s_time * 0.045f + d.pulsePhase);
        /* closer devices read slightly larger */
        float closeness = 1.0f + (DOME_RAD_MAX - rr) / DOME_RAD_MAX * 0.35f;

        int haloR = (int)((8 + d.sizeClass * 2) * breathe * closeness * d.alpha);
        if (haloR < 4) haloR = 4;
        int coreR = 2 + (d.sizeClass >> 1);
        float coreScale = 0.5f + 0.5f * breathe;

        s_canvas.addGlowDot(x, y, d.glow, 0.32f * d.alpha * gain, haloR);
        s_canvas.addGlowDot(x, y, d.core,
                            0.90f * d.alpha * gain,
                            (int)(coreR * coreScale) + 1);
        /* off-center specular — makes the orb read as a ball */
        s_canvas.addGlowDot(x - coreR * 0.4f, y - coreR * 0.4f,
                            { 255, 255, 255 }, 0.45f * d.alpha * gain, 2);
    }
}

/* Advance + draw the particle pool (pulse streams, bursts, implosions). */
static void render_pool(float gain)
{
    for (int i = 0; i < DOME_POOL_SIZE; i++) {
        DomeParticle &p = s_pool[i];
        if (!p.alive) continue;
        p.age += 1.0f;
        p.u += p.speed;
        if (p.u >= 1.0f || p.age > p.life) {
            p.alive = 0;
            continue;
        }
        float u = p.u;
        float x, y, intensity;
        int radius = p.radius;
        switch (p.kind) {
        case DOME_P_BURST: {
            float eu = dome_ease_out_cubic(u);
            x = kCenterX + (p.tx - kCenterX) * eu;
            y = kCenterY + (p.ty - kCenterY) * eu;
            /* slight angular scatter so the burst fans out */
            x += p.px * p.wobAmp * sinf(u * 8.0f + p.wobPhase) * u;
            y += p.py * p.wobAmp * sinf(u * 8.0f + p.wobPhase) * u;
            intensity = (u < 0.05f) ? u / 0.05f : 1.0f;
            intensity *= 0.82f * (1.0f - 0.65f * u) * gain;  /* merge into orb */
            /* comet tail */
            float eu2 = dome_ease_out_cubic(u * 0.82f);
            s_canvas.addGlowDot(kCenterX + (p.tx - kCenterX) * eu2,
                                kCenterY + (p.ty - kCenterY) * eu2,
                                { p.r, p.g, p.b }, intensity * 0.45f,
                                p.radius > 1 ? p.radius - 1 : 1);
            break;
        }
        case DOME_P_IMPLODE: {
            float eu = dome_ease_in_cubic(u);
            x = p.x + (p.tx - p.x) * eu;
            y = p.y + (p.ty - p.y) * eu;
            intensity = 0.75f * (1.0f - eu) * gain;
            radius = 3 - (int)(2.0f * eu);
            if (radius < 1) radius = 1;
            break;
        }
        default: {  /* DOME_P_STREAM */
            x = kCenterX + (p.tx - kCenterX) * u;
            y = kCenterY + (p.ty - kCenterY) * u;
            float wob = sinf(u * p.wobFreq * 12.566f + p.wobPhase) *
                        sinf(u * 3.14159f);   /* envelope: no wobble at ends */
            x += p.px * p.wobAmp * wob;
            y += p.py * p.wobAmp * wob;
            float edge = u * 5.0f;
            if (edge > 1.0f) edge = 1.0f;
            float tail = (1.0f - u) * 5.0f;
            if (tail < edge) edge = tail;
            intensity = (float)p.intensity / 255.0f * edge * gain;
            break;
        }
        }
        s_canvas.addGlowDot(x, y, { p.r, p.g, p.b }, intensity, radius);
    }
}

/* FACE_OWNER_NEAR: warm amber aurora wisps orbiting the center. */
static void render_wisps(float gain)
{
    static const float kWispA[7] = { 0.0f, 0.9f, 1.8f, 2.7f, 3.6f, 4.5f, 5.4f };
    static const float kWispR[7] = { 42, 68, 55, 80, 62, 74, 48 };
    static const float kWispP[7] = { 0.2f, 1.1f, 2.4f, 3.0f, 4.2f, 5.0f, 0.8f };
    static const int   kWispS[7] = { 16, 20, 14, 22, 17, 19, 15 };
    for (int i = 0; i < 7; i++) {
        float a = kWispA[i] + s_time * 0.004f;
        float r = kWispR[i] + 6.0f * sinf(s_time * 0.010f + kWispP[i]);
        float intensity = (0.10f + 0.08f * sinf(s_time * 0.02f + kWispP[i])) * gain;
        s_canvas.addGlowDot(kCenterX + cosf(a) * r, kCenterY + sinf(a) * r,
                            { 255, 165, 60 }, intensity, kWispS[i]);
    }
}

/* FACE_VOICE: green ring pulse breathing with mic energy. */
static void render_voice_ring(float energy, float gain)
{
    float r = 48.0f + energy * 26.0f;
    float baseI = (0.10f + 0.45f * energy) * gain;
    for (int i = 0; i < 96; i++) {
        float a = (float)i / 96.0f * 6.2831853f;
        float shimmer = 0.8f + 0.2f * sinf(a * 5.0f + s_time * 0.05f);
        s_canvas.addGlowDot(kCenterX + cosf(a) * r, kCenterY + sinf(a) * r,
                            { 140, 255, 150 }, baseI * shimmer, 2);
    }
    float r2 = r + 16.0f;
    for (int i = 0; i < 64; i++) {
        float a = (float)i / 64.0f * 6.2831853f;
        s_canvas.addGlowDot(kCenterX + cosf(a) * r2, kCenterY + sinf(a) * r2,
                            { 140, 255, 150 }, baseI * 0.45f, 1);
    }
}

/* ------------------------------------------------------------------ */
/* Render loop                                                         */
/* ------------------------------------------------------------------ */

static void render_dome(face_state_t state)
{
    /* State overlays: the dome is the base, states tint/boost it. */
    float gain = 1.0f;
    float sweepSpeed = 0.020f;
    if (state == FACE_SLEEP) {
        gain = 0.10f;
    } else if (state == FACE_ALERT) {
        sweepSpeed = 0.052f;   /* agitated: fast sweep */
    }
    float voiceEnergy = 0.0f;
    if (state == FACE_VOICE) {
        voiceEnergy = s_voice_energy;
        sweepSpeed += 0.05f * voiceEnergy;
    }

    int deviceCount = dome_refresh();
    s_sweepAngle += sweepSpeed;
    if (s_sweepAngle > 6.28318f) s_sweepAngle -= 6.28318f;

    const FaceStyle &st = kStyles[state];
    render_dust(st, deviceCount, gain);
    render_rings(s_sweepAngle, gain);
    render_sweep(gain);
    render_pool(gain);          /* streams + bursts + implosions */
    render_orbs(state, gain);
    render_center(gain);

    if (state == FACE_VOICE) {
        render_voice_ring(voiceEnergy, gain);
    } else if (state == FACE_OWNER_NEAR) {
        render_wisps(gain);
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
            s_frame++;
            s_time += 1.0f;
            render_dome(state);
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
