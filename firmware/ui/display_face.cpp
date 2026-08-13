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
 * The panel is a 466x466 QSPI AMOLED (CO5300 controller) driven through
 * M5GFX (LovyanGFX) with a PSRAM framebuffer — the M5StopWatch-Flux
 * pattern: render glow stamps straight into the framebuffer rows, flush
 * once per frame via the fb panel's display(). A FreeRTOS task (core 1,
 * moderate priority) owns the render loop at a 33fps target. All public
 * calls are thread-safe and may be used from any task.
 *
 * C-callable so the app shell (C or C++) can drive it.
 */
#include <cstring>
#include <cmath>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <M5GFX.h>
#include <lgfx/v1/panel/Panel_AMOLED.hpp>

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
/* CSI visualization state — driven by REAL sense-engine features      */
/* (sense_engine_get_csi_features: wander / jitter / presence / move). */
/* ------------------------------------------------------------------ */

/* Mode: -1 = auto-cycle (default), else pinned CSI_MODE_* value. */
static volatile int s_csiModeReq = -1;
static csi_mode_t s_csiMode = CSI_MODE_MERGED;

#define CSI_AUTO_CYCLE_FRAMES (30 * 33)  /* ~30s per mode at 33fps */

/* Motion-pulse state: rising edge of `moved` fires an expanding ripple. */
static float s_csiPulse = 0.0f;   /* 1.0 fresh spike -> 0.0 decayed */
static bool  s_csiMovedPrev = false;

/* Polar waveform history — the combined living energy signal, plotted
 * faithfully. Appended at the ~5Hz cache cadence (sampleId changes);
 * 96 samples = ~19s of live signal. Wander alone is calibration-gated
 * (0.0 on noisy channels), so the buffer holds csi_energy() — the same
 * drive the rings use — guaranteeing a living trace at all times. */
#define CSI_WAVE_N 96
static float s_wave[CSI_WAVE_N];
static int   s_waveHead = 0;
static uint32_t s_waveLastId = 0xFFFFFFFFu;

/* Feature normalization — observed value ranges (mosaic-research
 * capability map: wander spikes on motion, jitter 0.29..0.99 on the
 * live gateway rows) mapped to a 0..1 visual drive. Linear, clamped. */
static inline float csi_norm_wander(float w) { float v = w * 2.5f; return v > 1.0f ? 1.0f : (v < 0.0f ? 0.0f : v); }
static inline float csi_norm_jitter(float j) { float v = j * 1.3f;  return v > 1.0f ? 1.0f : (v < 0.0f ? 0.0f : v); }

/* Combined living energy — the master visual drive.
 *
 * wander is calibration-gated: esp-radar only computes waveform_wander
 * from templates captured during a successful train, which fails on
 * noisy channels (verified live: wander=0.0000 train_valid=0 on every
 * gateway row while jitter runs 0.23..0.95 — see csi_sensing.h). The
 * viz must therefore drive from BOTH features: whichever is alive wins.
 * A calm room shows the jitter floor (clear visible structure at rest);
 * motion pushes the drive bright. */
static inline float csi_energy(float wander, float jitter)
{
    float w = csi_norm_wander(wander);
    float j = csi_norm_jitter(jitter);
    return w > j ? w : j;
}

/* Staleness factor: 1.0 while fresh, decaying to 0.0 once the cache
 * stops updating (~5s) — the CSI viz calms honestly when the radio
 * goes quiet/offline instead of holding a frozen pattern. */
static inline float csi_freshness(const sense_csi_features_t &c)
{
    uint32_t ageMs = (uint32_t)(esp_timer_get_time() / 1000) - c.updatedMs;
    if (ageMs > 5000u) return 0.0f;
    return 1.0f - (float)ageMs / 5000.0f;
}

static void csi_wave_push(float v)
{
    s_wave[s_waveHead] = v;
    s_waveHead = (s_waveHead + 1) % CSI_WAVE_N;
}

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
    const uint8_t *lf;  /* per-x falloff factor in [z0, f0)   */
    const uint8_t *rf;  /* per-x falloff factor in [f1, z1)   */
};
static VignRow s_vign[kScreenH];
static uint8_t s_vignFactor[512];  /* radial falloff LUT (d -> 0..255) */
static uint8_t *s_vignFalloff = nullptr;  /* per-row factor arrays (11KB) */

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

    /* Precompute the per-pixel falloff factors once. The per-frame pass is
     * then pure integer math — sqrtf per pixel is ~13ms/frame on the S3's
     * software float. */
    size_t total = 0;
    for (int y = 0; y < kScreenH; y++) {
        float dy = (float)y - kCenterY;
        float dy2 = dy * dy;
        VignRow &v = s_vign[y];
        if (dy2 > rOut2) {
            v.z0 = 0; v.f0 = 0; v.f1 = kScreenW; v.z1 = kScreenW;
            v.lf = v.rf = nullptr;
            continue;
        }
        int o = (int)sqrtf(rOut2 - dy2);   /* outer half-width */
        int i = dy2 > rIn2 ? 0 : (int)sqrtf(rIn2 - dy2);
        v.z0 = (int16_t)(kCenterX - o);
        v.f0 = (int16_t)(kCenterX - i);
        v.f1 = (int16_t)(kCenterX + i);
        v.z1 = (int16_t)(kCenterX + o);
        total += (size_t)(v.f0 - v.z0) + (size_t)(v.z1 - v.f1);
    }

    s_vignFalloff = (uint8_t *)malloc(total ? total : 1);
    if (s_vignFalloff != nullptr) {
        uint8_t *p = s_vignFalloff;
        for (int y = 0; y < kScreenH; y++) {
            VignRow &v = s_vign[y];
            if (v.f0 <= v.z0) {
                v.lf = nullptr;
            } else {
                v.lf = p;
                for (int x = v.z0; x < v.f0; x++) {
                    float dx = (float)x - kCenterX;
                    float dy = (float)y - kCenterY;
                    int d = (int)sqrtf(dx * dx + dy * dy);
                    if (d > 511) d = 511;
                    *p++ = s_vignFactor[d];
                }
            }
            if (v.z1 <= v.f1) {
                v.rf = nullptr;
            } else {
                v.rf = p;
                for (int x = v.f1; x < v.z1; x++) {
                    float dx = (float)x - kCenterX;
                    float dy = (float)y - kCenterY;
                    int d = (int)sqrtf(dx * dx + dy * dy);
                    if (d > 511) d = 511;
                    *p++ = s_vignFactor[d];
                }
            }
        }
    }
}

/* Soften the disc edge for the round lens. Operates in place on the
 * canvas's row pointers (the M5GFX panel framebuffer); the mask is
 * static so it is idempotent (dirty-rect erases only ever touch content
 * inside the disc). */
static void vignette_apply(void)
{
    auto rows = s_canvas.rows();
    for (int y = 0; y < kScreenH; y++) {
        const VignRow &v = s_vign[y];
        if (v.z0 <= 0 && v.z1 >= kScreenW) {
            continue;  /* fully inside the disc — nothing to do */
        }
        uint16_t *row = rows[y];
        if (v.z0 > 0) {
            memset(row, 0, (size_t)v.z0 * sizeof(uint16_t));
        }
        if (v.z1 < kScreenW) {
            memset(row + v.z1, 0, (size_t)(kScreenW - v.z1) * sizeof(uint16_t));
        }
        if (v.lf != nullptr) {
            /* left falloff band [z0, f0) — precomputed factors */
            for (int x = v.z0; x < v.f0; x++) {
                uint8_t f = v.lf[x - v.z0];
                uint16_t px = row[x];
                int rr = ((px >> 11) & 0x1F) * f >> 8;
                int gg = ((px >> 5) & 0x3F) * f >> 8;
                int bb = (px & 0x1F) * f >> 8;
                row[x] = (uint16_t)((rr << 11) | (gg << 5) | bb);
            }
            /* right falloff band [f1, z1) */
            for (int x = v.f1; x < v.z1; x++) {
                uint8_t f = v.rf[x - v.f1];
                uint16_t px = row[x];
                int rr = ((px >> 11) & 0x1F) * f >> 8;
                int gg = ((px >> 5) & 0x3F) * f >> 8;
                int bb = (px & 0x1F) * f >> 8;
                row[x] = (uint16_t)((rr << 11) | (gg << 5) | bb);
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Panel (CO5300 QSPI AMOLED) — M5GFX (LovyanGFX) framebuffer driver   */
/*                                                                     */
/* Ported from M5StopWatch-Flux hal_display.cpp: a Panel_CO5300 over   */
/* lgfx::Panel_AMOLED (native CO5300 QSPI envelope: 0x02 command /     */
/* 0x32 pixel writes) on an 80MHz quad-SPI bus, plus a PSRAM panel     */
/* framebuffer (enableFrameBuffer). The glow canvas renders straight   */
/* into the framebuffer rows; one display() per frame pushes the       */
/* whole 466x466 frame over the bus. Board pins differ from M5Stack's; */
/* there is no TE pin on this board.                                   */
/* ------------------------------------------------------------------ */

static constexpr gpio_num_t cfg_pin_sclk = GPIO_NUM_38;
static constexpr gpio_num_t cfg_pin_io0  = GPIO_NUM_4;
static constexpr gpio_num_t cfg_pin_io1  = GPIO_NUM_5;
static constexpr gpio_num_t cfg_pin_io2  = GPIO_NUM_6;
static constexpr gpio_num_t cfg_pin_io3  = GPIO_NUM_7;
static constexpr gpio_num_t cfg_pin_cs   = GPIO_NUM_12;
static constexpr gpio_num_t cfg_pin_rst  = GPIO_NUM_3;

class Panel_CO5300 : public lgfx::Panel_AMOLED {
public:
    Panel_CO5300(void)
    {
        _cfg.memory_width  = _cfg.panel_width  = 480;
        _cfg.memory_height = _cfg.panel_height = 480;
        _write_depth       = lgfx::color_depth_t::rgb565_2Byte;
        _read_depth        = lgfx::color_depth_t::rgb565_2Byte;
    }

    const uint8_t *getInitCommands(uint8_t listno) const override
    {
        /* M5StopWatch-Flux sequence (list format: cmd, len, args...,
         * delay byte when the CMD_INIT_DELAY flag is set): sleep out
         * 150ms, interface control, tearing line on + tear line 466,
         * brightness control on, MADCTL 0, display brightness 0xA0,
         * display on. 0x3A (RGB565) is sent by Panel_AMOLED::
         * setColorDepth during the LGFX init path. */
        static constexpr uint8_t list0[] = {
            0x11, 0 + CMD_INIT_DELAY, 150,  /* sleep out */
            0xC4, 1, 0x80,                  /* interface control */
            0x35, 1, 0x80,                  /* tearing effect line on */
            0x44, 2, 0x01, 0xD2,            /* tear line = 0x1D2 == 466 */
            0x53, 1, 0x20,                  /* brightness control on */
            0x36, 1, 0x00,                  /* MADCTL */
            0x51, 1, 0xA0,                  /* display brightness */
            0x29, 0,                        /* display on */
            0xff, 0xff                      /* end */
        };
        switch (listno) {
            case 0:
                return list0;
            default:
                return nullptr;
        }
    }
};

class MosaicDisplay : public M5GFX {
    lgfx::Bus_SPI _bus_instance;
    Panel_CO5300 _panel_instance;
    bool _fb_enabled = false;

public:
    bool init_impl(bool use_reset, bool use_clear) override
    {
        {
            auto cfg = _bus_instance.config();

            cfg.freq_write = 80000000;
            cfg.freq_read  = 10000000;  /* irrelevant (readable=false) */

            cfg.pin_sclk = cfg_pin_sclk;
            cfg.pin_io0  = cfg_pin_io0;
            cfg.pin_io1  = cfg_pin_io1;
            cfg.pin_io2  = cfg_pin_io2;
            cfg.pin_io3  = cfg_pin_io3;

            cfg.spi_host    = SPI2_HOST;
            cfg.spi_mode    = 0;  /* SPI_MODE0 */
            cfg.spi_3wire   = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();

            cfg.pin_rst      = cfg_pin_rst;
            cfg.pin_cs       = cfg_pin_cs;
            cfg.panel_width  = 468;
            cfg.panel_height = 466;
            cfg.offset_x     = 6;  /* confirmed centering — do not change */
            cfg.offset_y     = 0;
            cfg.readable     = false;

            _panel_instance.config(cfg);
        }

        setPanel(&_panel_instance);

        if (!LGFX_Device::init_impl(use_reset, use_clear)) {
            return false;
        }

        enableFrameBuffer(true);

        _panel_instance.setBrightness(128);

        return true;
    }

    bool enableFrameBuffer(bool auto_display = false)
    {
        _fb_enabled = false;
        if (_panel_instance.initPanelFb()) {
            auto fbPanel = _panel_instance.getPanelFb();
            if (fbPanel) {
                fbPanel->setBus(&_bus_instance);
                fbPanel->setAutoDisplay(auto_display);
                setPanel(fbPanel);
                _fb_enabled = true;
                return true;
            }
        }
        return false;
    }

    bool fbEnabled() const { return _fb_enabled; }
};

static MosaicDisplay *s_display = nullptr;

/* After enableFrameBuffer() the active panel is the framebuffer panel
 * (Panel_AMOLED_Framebuffer) — the glow canvas renders into its rows
 * and the flush pushes via its display(). */
static lgfx::Panel_FrameBufferBase *fb_panel(void)
{
    if (s_display == nullptr || !s_display->fbEnabled()) {
        return nullptr;
    }
    return static_cast<lgfx::Panel_FrameBufferBase *>(s_display->getPanel());
}

static void mosaic_panel_flush(void *ctx)
{
    (void)ctx;
    if (s_display == nullptr) {
        return;
    }
    /* Round cutout: soften the disc edge once per frame, in place. */
    vignette_apply();
    /* Push the whole 466x466 frame from the PSRAM framebuffer over QSPI.
     * The fb panel's display() handles the 0x32 pixel-write envelope,
     * per-line DMA double buffering and the GRAM offset (offset_x=6). */
    auto fb = fb_panel();
    if (fb != nullptr) {
        fb->display(0, 0, kScreenW, kScreenH);
    }
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

    /* Board bring-up: bring the CO5300 panel up through M5GFX with a
     * PSRAM framebuffer (the StopWatch-Flux pattern). */
    s_display = new MosaicDisplay();
    if (s_display == nullptr || !s_display->init()) {
        ESP_LOGE(TAG, "M5GFX display init failed — face renders framebuffer only");
        delete s_display;
        s_display = nullptr;
        /* continue: state machine still runs, flush is a no-op */
    }

    /* Hand the M5GFX panel-framebuffer row pointers to the glow canvas:
     * it renders glow stamps straight into the panel framebuffer and the
     * flush pushes the frame with one display() call per frame. */
    static uint16_t *s_rows[kScreenH];  /* static: too big for app_main stack */
    bool have_rows = false;
    auto fb = fb_panel();
    if (fb != nullptr) {
        auto lines = fb->getLinesBuffer();
        if (lines != nullptr) {
            for (int y = 0; y < kScreenH; y++) {
                s_rows[y] = (uint16_t *)lines[y];
            }
            have_rows = true;
        }
    }

    if (!s_canvas.init(have_rows ? s_rows : nullptr, mosaic_panel_flush, nullptr)) {
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

esp_err_t display_face_set_csi_mode(int mode)
{
    if (mode < -1 || mode >= (int)CSI_MODE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_csiModeReq = mode;
    if (mode >= 0) {
        s_csiMode = (csi_mode_t)mode;  /* pinned — switch immediately */
    }
    if (s_mutex) xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "csi mode -> %s",
             mode < 0 ? "auto" : (mode == CSI_MODE_MERGED ? "merged" :
             mode == CSI_MODE_STANDALONE ? "standalone" : "off"));
    return ESP_OK;
}

csi_mode_t display_face_get_csi_mode(void)
{
    return s_csiMode;
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
/* CSI visualization — the radio dome's second sense, rendered live.   */
/*                                                                     */
/* Both modes are driven by the sense engine's REAL channel features:  */
/*   energy (max of wander/jitter norms) -> shape/amplitude — the      */
/*              master drive (see csi_energy: wander alone is          */
/*              calibration-gated to 0 on noisy channels)              */
/*   jitter  -> angular motion + sparkle (motion energy)               */
/*   someone -> warm presence tint + slow breathing-band envelope      */
/*              (0.2-0.5Hz annotated on top, per the capability map)   */
/*   moved   -> expanding ripple pulses from the core                  */
/* Raw signal rendered faithfully, known patterns annotated — no       */
/* labels, no fake animation: when the ether is calm the pattern is    */
/* calm, when someone walks past it visibly reacts.                    */
/* ------------------------------------------------------------------ */

/* Motion ripple: one frame of edge-trigger state per render frame. */
static void csi_motion_tick(const sense_csi_features_t &c, bool have, float freshness)
{
    bool moved = have && c.moved && freshness > 0.0f;
    if (moved && !s_csiMovedPrev) {
        s_csiPulse = 1.0f;
    }
    s_csiMovedPrev = moved;
    s_csiPulse *= 0.94f;   /* ~1.3s visible tail */
}

/* Expanding double ring fired on a motion edge. Shared by both modes. */
static void render_csi_ripple(float gain)
{
    if (s_csiPulse <= 0.03f) return;
    float p = s_csiPulse;
    float r1 = 26.0f + (1.0f - p) * 130.0f;
    for (int i = 0; i < 96; i++) {
        float a = (float)i / 96.0f * 6.2831853f;
        s_canvas.addGlowDot(kCenterX + cosf(a) * r1, kCenterY + sinf(a) * r1,
                            { 255, 235, 210 }, p * 0.42f * gain, 1);
    }
    float r2 = r1 + 15.0f;
    for (int i = 0; i < 72; i++) {
        float a = (float)i / 72.0f * 6.2831853f + p * 0.5f;
        s_canvas.addGlowDot(kCenterX + cosf(a) * r2, kCenterY + sinf(a) * r2,
                            { 255, 200, 160 }, p * 0.22f * gain, 1);
    }
}

/* CSI MERGED: a living membrane around the core — Siri-like morphing
 * ring inside the dome's inner ring. combined energy morphs its shape,
 * jitter spins + sparkles it, presence opens it warm with a 0.2-0.5Hz
 * breath. Clearly visible in IDLE (jitter floor), alive when the
 * channel reacts. */
static void render_csi_halo(float gain)
{
    sense_csi_features_t c;
    bool have = sense_engine_get_csi_features(&c);
    float fresh = have ? csi_freshness(c) : 0.0f;
    float wm = have ? csi_norm_wander(c.wander) : 0.0f;
    float jm = have ? csi_norm_jitter(c.jitter) : 0.0f;
    float en = have ? csi_energy(c.wander, c.jitter) : 0.0f;
    float someone = have && c.someone ? 1.0f : 0.0f;
    if (fresh <= 0.0f) { wm = 0.0f; jm = 0.0f; en = 0.0f; someone = 0.0f; }  /* radio quiet/offline: calm */
    csi_motion_tick(c, have, fresh);

    const int N = 150;
    /* breathing-band annotation (~0.29Hz at 33fps) — the known pattern,
     * scaled by reality: only audible when presence/motion is real. */
    float breath = 0.5f + 0.5f * sinf(s_time * 0.0088f + 1.7f);
    float baseR = 76.0f + 10.0f * someone * breath;         /* presence opens the halo — big base ring, clearly visible */
    float morph = 9.0f + 24.0f * en;                        /* combined energy morphs the shape (wander OR jitter) */
    float rot = s_time * (0.0035f + 0.018f * jm);           /* jitter spins it faster */
    Rgb col = lerpRgb({ 120, 190, 255 }, { 255, 175, 90 }, someone * 0.8f);

    for (int i = 0; i < N; i++) {
        float a = (float)i / N * 6.2831853f + rot;
        float w1 = 0.5f + 0.5f * sinf(a * 3.0f + s_time * 0.016f);
        float w2 = 0.5f + 0.5f * sinf(a * 5.0f - s_time * 0.011f + (float)i * 0.63f);
        float r = baseR + (w1 * w2 - 0.25f) * 2.0f * morph +
                  2.0f * sinf(s_time * 0.02f + (float)i);
        if (r < 30.0f) r = 30.0f;
        if (r > 108.0f) r = 108.0f;   /* stay clear of the dome's inner ring (94) */
        a += jm * 0.06f * sinf(s_time * 0.08f + (float)i * 2.17f);
        float flick = 0.55f + 0.45f * sinf(s_time * 0.10f + (float)i * 1.31f);
        /* Energy-driven: the halo stays clearly visible at the jitter
         * floor (calm room) and flares with motion — never hidden by
         * the wander calibration gate. */
        float intensity = (0.16f + 0.14f * someone + 0.16f * en * flick + 0.06f * wm) * gain;
        if (intensity <= 0.01f) continue;
        s_canvas.addGlowDot(kCenterX + cosf(a) * r, kCenterY + sinf(a) * r,
                            col, intensity, 2);
    }

    /* presence aura behind the core */
    if (someone > 0.0f) {
        s_canvas.addGlowDot(kCenterX, kCenterY, { 255, 190, 110 },
                            (0.10f + 0.06f * breath) * gain, 18);
    }

    render_csi_ripple(gain);
}

/* CSI STANDALONE: full-screen signal anatomy, denser than the dome.
 *   - frequency rings: each ring reads one ENERGY band (multi-scale
 *     EMA on the combined wander/jitter drive — wander alone is
 *     calibration-gated to 0, so the rings would vanish) so the
 *     channel's living dynamics spread from fast (inner) to slow
 *     (outer) rings; aberration waves displace the dots ∝ jitter
 *   - polar waveform: the combined living energy plotted faithfully
 *     over ~19s of history (5Hz cache samples)
 *   - presence: warm breathing aura + breath-brightened pattern
 *   - motion: expanding ripple + radial sparks from the core
 *   - absence: a quiet room rests the rings and the waveform at their
 *     visible base intensities — the calm IS the absence annotation
 *     (no labels needed) */
static void render_csi_standalone(float gain)
{
    sense_csi_features_t c;
    bool have = sense_engine_get_csi_features(&c);
    float fresh = have ? csi_freshness(c) : 0.0f;
    float jm = have ? csi_norm_jitter(c.jitter) : 0.0f;
    float en = have ? csi_energy(c.wander, c.jitter) : 0.0f;
    float someone = have && c.someone ? 1.0f : 0.0f;
    if (fresh <= 0.0f) { jm = 0.0f; en = 0.0f; someone = 0.0f; }
    csi_motion_tick(c, have, fresh);

    /* keep the dome device table + particle pool alive while standalone
     * owns the screen (devices must not age-out or pile up invisibly) */
    dome_refresh();
    for (int i = 0; i < DOME_POOL_SIZE; i++) {
        DomeParticle &p = s_pool[i];
        if (!p.alive) continue;
        p.age += 1.0f;
        p.u += p.speed;
        if (p.u >= 1.0f || p.age > p.life) p.alive = 0;
    }

    /* append fresh waveform samples (one per 5Hz cache tick) — store the
     * combined energy (0..1) so the trace is alive even when wander is
     * calibration-gated at 0 */
    if (have && c.sampleId != s_waveLastId) {
        s_waveLastId = c.sampleId;
        csi_wave_push(csi_energy(c.wander, c.jitter));
    }

    float breath = 0.5f + 0.5f * sinf(s_time * 0.0088f + 2.3f);   /* 0.2-0.5Hz annotation */

    /* --- frequency rings + aberration waves ---
     * Each ring reads one ENERGY band (multi-scale EMA on the combined
     * wander+jitter drive — NOT wander alone: wander is calibration-gated
     * to 0.0 on noisy channels, so a wander-only drive blanks the rings).
     * Base intensities are floored well above the additive-glow noise
     * floor: a calm room shows clear living rings, motion goes bright. */
    static const float kRingR[6] = { 46.0f, 72.0f, 98.0f, 170.0f, 196.0f, 218.0f };
    static const int   kRingN[6] = { 40, 56, 72, 96, 108, 116 };
    static const float kRingRate[6] = { 0.30f, 0.19f, 0.12f, 0.07f, 0.042f, 0.024f };
    static float s_wband[6] = { 0, 0, 0, 0, 0, 0 };
    for (int k = 0; k < 6; k++) {
        s_wband[k] += (en - s_wband[k]) * kRingRate[k];
    }
    for (int k = 0; k < 6; k++) {
        float r = kRingR[k];
        float band = s_wband[k];
        float baseI = (0.14f + 0.34f * band) * gain;
        if (baseI <= 0.004f) continue;
        Rgb col = lerpRgb({ 80, 160, 255 }, { 255, 170, 90 }, someone * 0.75f);
        float sh[8];   /* 8-phase shimmer LUT — one sinf per phase, not per dot */
        for (int q = 0; q < 8; q++) {
            sh[q] = 0.62f + 0.38f * sinf((float)q * 0.7853982f * (3.0f + k) +
                                         s_time * (0.010f + 0.0035f * (float)k) - (float)k * 1.9f);
        }
        float wphase = s_time * 0.028f + (float)k * 1.3f;
        float breathBoost = someone ? (1.0f + 0.30f * breath) : 1.0f;
        for (int i = 0; i < kRingN[k]; i++) {
            float a = (float)i / kRingN[k] * 6.2831853f;
            float aa = a + jm * 0.30f * sinf(a * 2.0f + wphase);   /* aberration ∝ jitter */
            float ri = baseI * sh[i & 7] * breathBoost;
            if (ri <= 0.004f) continue;
            s_canvas.addGlowDot(kCenterX + cosf(aa) * r, kCenterY + sinf(aa) * r,
                                col, ri, 1);
        }
    }

    /* --- polar waveform: the combined living energy, faithfully ---
     * Buffer holds csi_energy (0..1), scaled to the ACTUAL observed
     * range: jitter floor ~0.29 (norm ~0.38) already reads as a living
     * ring; motion pushes it out bright. No *2.5 wander scaling here —
     * that mapped calibration-gated 0.0 into a pinned dim dot. */
    {
        Rgb wcol = lerpRgb({ 120, 200, 255 }, { 255, 170, 110 }, jm * 0.9f);
        for (int i = 0; i < CSI_WAVE_N; i++) {
            float v = s_wave[(s_waveHead + i) % CSI_WAVE_N];
            float nv = v > 1.0f ? 1.0f : (v < 0.0f ? 0.0f : v);
            float a = (float)i / CSI_WAVE_N * 6.2831853f + 0.02f * sinf(s_time * 0.005f);
            float r = 133.0f + (nv - 0.5f) * 50.0f;   /* lane between rings 98 and 170 */
            s_canvas.addGlowDot(kCenterX + cosf(a) * r, kCenterY + sinf(a) * r,
                                wcol, (0.30f + 0.45f * nv) * gain, 1);
        }
    }

    /* --- presence aura + breathing --- */
    if (someone > 0.0f) {
        s_canvas.addGlowDot(kCenterX, kCenterY, { 255, 180, 100 },
                            (0.16f + 0.08f * breath) * gain, 26);
        s_canvas.addGlowDot(kCenterX, kCenterY, { 255, 150, 70 },
                            (0.10f + 0.06f * breath) * gain, 14);
    }

    /* --- motion: ripple + radial sparks --- */
    if (s_csiPulse > 0.03f) {
        for (int k = 0; k < 28; k++) {
            float a = (float)k / 28.0f * 6.2831853f + s_time * 0.01f;
            float r = 30.0f + (1.0f - s_csiPulse) * 185.0f + (float)(k % 3) * 6.0f;
            s_canvas.addGlowDot(kCenterX + cosf(a) * r, kCenterY + sinf(a) * r,
                                { 255, 240, 220 }, s_csiPulse * 0.40f * gain, 2);
        }
    }
    render_csi_ripple(gain);

    /* --- rotating tick on the rim — the instrument's hand --- */
    {
        float tickA = s_time * (0.004f + 0.03f * jm);
        s_canvas.addGlowDot(kCenterX + cosf(tickA) * 218.0f, kCenterY + sinf(tickA) * 218.0f,
                            { 240, 250, 255 }, 0.5f * gain, 3);
    }

    render_center(gain);
}

/* ------------------------------------------------------------------ */
/* Render loop                                                         */
/* ------------------------------------------------------------------ */

static void render_dome(face_state_t state)
{
    /* State overlays: the dome is the base, states tint/boost it. */
    float gain = 1.0f;
    if (state == FACE_SLEEP) {
        gain = 0.10f;
    }
    float voiceEnergy = 0.0f;
    if (state == FACE_VOICE) {
        voiceEnergy = s_voice_energy;
    }

    int deviceCount = dome_refresh();

    const FaceStyle &st = kStyles[state];
    render_dust(st, deviceCount, gain);
    render_pool(gain);          /* streams + bursts + implosions */
    render_orbs(state, gain);
    render_center(gain);

    if (state == FACE_VOICE) {
        render_voice_ring(voiceEnergy, gain);
    } else if (state == FACE_OWNER_NEAR) {
        render_wisps(gain);
    }

    /* CSI MERGED: the dome stays the base, the halo morphs around the
     * core in the same frame, driven by the live channel features. */
    if (s_csiMode == CSI_MODE_MERGED) {
        render_csi_halo(gain);
    }
}

static void face_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    const TickType_t frame_ms = pdMS_TO_TICKS(1000 / 33); /* 33fps */
    int64_t tFrame0 = esp_timer_get_time();

    for (;;) {
        if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
        face_state_t state = s_state;
        bool suspended = s_suspended;
        if (s_mutex) xSemaphoreGive(s_mutex);

        /* CSI mode: pinned, or auto-cycling every ~30s (merged ->
         * standalone -> plain dome -> ...). */
        int modeReq;
        if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
        modeReq = s_csiModeReq;
        if (s_mutex) xSemaphoreGive(s_mutex);
        csi_mode_t prevMode = s_csiMode;
        if (modeReq < 0) {
            if (s_frame > 0 && (s_frame % CSI_AUTO_CYCLE_FRAMES) == 0) {
                s_csiMode = (csi_mode_t)(((int)s_csiMode + 1) % CSI_MODE_COUNT);
                ESP_LOGI(TAG, "CSI mode auto -> %d (%s)", (int)s_csiMode,
                         s_csiMode == CSI_MODE_MERGED ? "merged" :
                         s_csiMode == CSI_MODE_STANDALONE ? "standalone" : "off");
            }
        } else {
            s_csiMode = (csi_mode_t)modeReq;
        }
        if (s_csiMode != prevMode) {
            /* mode switch: drop stale motion/ripple transients */
            s_csiPulse = 0.0f;
            s_csiMovedPrev = false;
        }

        s_canvas.beginFrame();

        if (!suspended) {
            s_frame++;
            s_time += 1.0f;
            if (s_csiMode == CSI_MODE_STANDALONE) {
                float gain = (state == FACE_SLEEP) ? 0.10f : 1.0f;
                render_csi_standalone(gain);
            } else {
                render_dome(state);
            }
        }

        s_canvas.push();

        /* Periodic verification log: live features + mode + frame time
         * (every ~4.5s). Proves the viz is driven by REAL data. */
        if ((s_frame % 150) == 0) {
            int64_t tNow = esp_timer_get_time();
            float frameMs = (float)(tNow - tFrame0) / 1000.0f;
            tFrame0 = tNow;
            sense_csi_features_t c;
            bool have = sense_engine_get_csi_features(&c);
            ESP_LOGI(TAG,
                     "frame=%lu mode=%d devs=%d csi=%s id=%u age=%ums "
                     "wander=%.4f jitter=%.4f smooth=%.0f someone=%d moved=%d "
                     "ready=%d train=%d render=%.1fms",
                     (unsigned long)s_frame, (int)s_csiMode,
                     sense_engine_get_device_count(), have ? "yes" : "no",
                     (unsigned)c.sampleId,
                     have ? (unsigned)((uint32_t)(tNow / 1000) - c.updatedMs) : 0u,
                     have ? (double)c.wander : 0.0, have ? (double)c.jitter : 0.0,
                     have ? (double)c.smooth : 0.0,
                     have ? (int)c.someone : 0, have ? (int)c.moved : 0,
                     have ? (int)c.presenceReady : 0, have ? (int)c.trainValid : 0,
                     (double)frameMs);
        }

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
