/*
 * fx_glow: self-contained direct-framebuffer glow engine for the Mosaic
 * node face.
 *
 * M5StopWatch-Flux style: render straight into a RGB565 framebuffer with
 * additive-blended integer glow stamps, erase only the previous frame's
 * dirty regions, then flush the whole frame once (33fps target on the
 * 466x466 QSPI AMOLED). No LVGL, no M5GFX, no display driver headers:
 * the engine works on a plain uint16_t row-pointer array + an optional
 * flush callback (the owner hands it the M5GFX panel-framebuffer rows and
 * wires the flush to the fb panel's display()).
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

/* Flush hook: called once per frame after rendering. The owner pushes the
 * canvas rows to the panel (M5GFX fb panel display(), or similar). */
using FlushFn = void (*)(void* ctx);

class GlowCanvas {
public:
    static constexpr int kMaxGlowRadius = 56;

    /* rows: kScreenH row pointers into an RGB565 framebuffer — e.g. the
     * M5GFX panel framebuffer's lines (Panel_FrameBufferBase::getLinesBuffer).
     * The canvas never owns the buffer. rows==nullptr -> init fails. */
    bool init(uint16_t** rows, FlushFn flush, void* flush_ctx);
    void deinit();

    /* The attached row pointers (for in-place post-pass, e.g. vignette). */
    uint16_t* const* rows() const { return _rows; }
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
        add_px(_rows[y] + x, r, g, b);
    }

    /* Additive glow dot with precomputed falloff stamp, radius 1..56. */
    void addGlowDot(float x, float y, const Rgb& color, float intensity, int radius);
    /* Additive line (motion trails). */
    void addLine(float x0, float y0, float x1, float y1, uint8_t r, uint8_t g, uint8_t b);

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

    uint16_t* _rows[kScreenH] = {nullptr};
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
