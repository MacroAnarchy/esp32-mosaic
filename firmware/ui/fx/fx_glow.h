/*
 * fx_glow: self-contained direct-framebuffer glow engine for the Mosaic
 * node face.
 *
 * M5StopWatch-Flux style: render straight into a RGB565 framebuffer with
 * additive-blended integer glow stamps, erase only the previous frame's
 * dirty regions, then flush the whole frame once (33fps target on the
 * 466x466 QSPI AMOLED). No LVGL, no M5GFX, no display driver headers:
 * the engine works on a plain uint16_t row buffer + an optional flush
 * callback (the owner wires it to esp_lcd_panel_draw_bitmap or similar).
 *
 * Threading contract: all draw calls must come from one render task.
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace fx {

inline constexpr int kScreenW = 466;  // AMOLED logical resolution
inline constexpr int kScreenH = 466;
inline constexpr float kCenterX = 233.0f;
inline constexpr float kCenterY = 233.0f;
inline constexpr float kRimR = 226.0f;  // visible disc radius (round screen)

/* Fast xorshift rng (not for security) */
uint32_t rnd();
float rndf();                    // 0..1
float rndf(float lo, float hi);  // lo..hi

struct Rgb {
    uint8_t r = 0, g = 0, b = 0;
};

Rgb lerpRgb(const Rgb& a, const Rgb& b, float t);

/* Flush hook: called once per frame after rendering. fb points at the
 * framebuffer (kScreenW * kScreenH uint16_t RGB565, row-major). */
using FlushFn = void (*)(void* ctx, const uint16_t* fb);

class GlowCanvas {
public:
    static constexpr int kMaxGlowRadius = 56;

    /* fb==nullptr -> the canvas allocates its own buffer (caller frees with
     * fb_free() afterwards; on ESP-IDF builds this lands in PSRAM via the
     * allocator). fb_bytes is only checked when fb != nullptr. */
    bool init(uint16_t* fb, size_t fb_bytes, FlushFn flush, void* flush_ctx);
    void deinit();

    uint16_t* fb() const { return _fb; }
    int width() const { return kScreenW; }
    int height() const { return kScreenH; }

    /* Full clear (use at init and on explicit resets). */
    void clear();
    /* Erase only the regions drawn in the previous frame — much cheaper
     * than a full clear. Anything drawn outside the tracked primitives
     * must be reported with markDirty(). */
    void beginFrame();
    void markDirty(int x, int y, int w, int h);
    /* Flush the whole framebuffer to the panel (no-op without a flush fn). */
    void push();

    /* Additive-blend a single pixel; channels 8bit 0..255. */
    inline void addPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
    {
        if ((unsigned)x >= (unsigned)kScreenW || (unsigned)y >= (unsigned)kScreenH) {
            return;
        }
        add_px(_fb + (size_t)y * kScreenW + x, r, g, b);
    }

    /* Additive glow dot with precomputed falloff stamp, radius 1..56. */
    void addGlowDot(float x, float y, const Rgb& color, float intensity, int radius);
    /* Additive line (motion trails). */
    void addLine(float x0, float y0, float x1, float y1, uint8_t r, uint8_t g, uint8_t b);

    /* Free a buffer that init() allocated internally. */
    static void fb_free(uint16_t* fb);

private:
    static inline void add_px(uint16_t* p, uint8_t r, uint8_t g, uint8_t b)
    {
        uint16_t v = *p;  // native little-endian RGB565
        int rr      = ((v >> 11) & 0x1F) + (r >> 3);
        int gg      = ((v >> 5) & 0x3F) + (g >> 2);
        int bb      = (v & 0x1F) + (b >> 3);
        rr          = rr > 31 ? 31 : rr;
        gg          = gg > 63 ? 63 : gg;
        bb          = bb > 31 ? 31 : bb;
        *p          = (uint16_t)((rr << 11) | (gg << 5) | bb);
    }

    uint16_t* _fb       = nullptr;
    bool _owns_fb       = false;
    FlushFn _flush      = nullptr;
    void* _flush_ctx    = nullptr;
    bool _full_clear    = true;  // dirty overflow fallback

    struct Rect {
        uint16_t x, y, w, h;
    };
    struct Seg {
        int16_t x0, y0, x1, y1;
    };
    static constexpr int kMaxDirty = 2048;
    static constexpr int kMaxSegs  = 512;
    Rect _dirty[kMaxDirty];
    Seg _segs[kMaxSegs];
    int _n_dirty = 0;
    int _n_segs  = 0;
};

}  // namespace fx
