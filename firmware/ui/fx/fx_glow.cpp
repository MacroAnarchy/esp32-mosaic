/*
 * fx_glow: implementation — additive glow stamps, dirty-rect erase, flush.
 * Pure C++11, no ESP-IDF / board headers (see fx_glow.h).
 */
#include "fx/fx_glow.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace fx {

static uint32_t _rng_state = 0x2A9D7E4Bu;

uint32_t rnd()
{
    uint32_t x = _rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    _rng_state = x;
    return x;
}

float rndf()
{
    return (rnd() & 0xFFFFFF) / 16777215.0f;
}

float rndf(float lo, float hi)
{
    return lo + (hi - lo) * rndf();
}

Rgb lerpRgb(const Rgb& a, const Rgb& b, float t)
{
    t       = std::clamp(t, 0.0f, 1.0f);
    Rgb out;
    out.r = (uint8_t)(a.r + (b.r - a.r) * t);
    out.g = (uint8_t)(a.g + (b.g - a.g) * t);
    out.b = (uint8_t)(a.b + (b.b - a.b) * t);
    return out;
}

/* ------------------------------ Glow stamps ------------------------------ */
/* stamp[r] is a (2r+1)^2 grid of 0..255 falloff weights, built lazily and
 * cached forever. Sum of all radii 1..56 stays ~600KB (PSRAM-friendly);
 * the face engine only ever touches radii 1..3. */
static uint8_t* _stamps[GlowCanvas::kMaxGlowRadius + 1] = {nullptr};

static const uint8_t* stamp_for(int r)
{
    if (_stamps[r] != nullptr) {
        return _stamps[r];
    }
    const int side    = 2 * r + 1;
    uint8_t* s        = (uint8_t*)std::malloc((size_t)side * side);
    const float denom = (float)(r * r + 1);
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            int d2  = dx * dx + dy * dy;
            float w = 1.0f - (float)d2 / denom;
            if (w < 0.0f) {
                w = 0.0f;
            }
            s[(dy + r) * side + (dx + r)] = (uint8_t)(w * w * 255.0f);
        }
    }
    _stamps[r] = s;
    return s;
}

/* --------------------------------- Canvas -------------------------------- */

bool GlowCanvas::init(uint16_t* fb, size_t fb_bytes, FlushFn flush, void* flush_ctx)
{
    const size_t need = (size_t)kScreenW * kScreenH * sizeof(uint16_t);
    if (fb != nullptr) {
        if (fb_bytes < need) {
            return false;
        }
        _fb     = fb;
        _owns_fb = false;
    } else {
        _fb      = (uint16_t*)std::malloc(need);
        _owns_fb = _fb != nullptr;
        if (_fb == nullptr) {
            return false;
        }
    }
    _flush      = flush;
    _flush_ctx  = flush_ctx;
    _n_dirty    = 0;
    _n_segs     = 0;
    _full_clear = false;
    clear();
    return true;
}

void GlowCanvas::deinit()
{
    if (_owns_fb && _fb != nullptr) {
        std::free(_fb);
    }
    _fb        = nullptr;
    _owns_fb   = false;
    _flush     = nullptr;
    _flush_ctx = nullptr;
    _n_dirty   = 0;
    _n_segs    = 0;
}

void GlowCanvas::fb_free(uint16_t* fb)
{
    std::free(fb);
}

void GlowCanvas::clear()
{
    if (_fb != nullptr) {
        std::memset(_fb, 0, (size_t)kScreenW * kScreenH * sizeof(uint16_t));
    }
}

void GlowCanvas::markDirty(int x, int y, int w, int h)
{
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > kScreenW) {
        w = kScreenW - x;
    }
    if (y + h > kScreenH) {
        h = kScreenH - y;
    }
    if (w <= 0 || h <= 0) {
        return;
    }
    if (_n_dirty >= kMaxDirty) {
        _full_clear = true;  // overflow: fall back to a full erase next frame
        return;
    }
    _dirty[_n_dirty++] = {(uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h};
}

void GlowCanvas::beginFrame()
{
    if (_full_clear) {
        clear();
        _full_clear = false;
        _n_dirty    = 0;
        _n_segs     = 0;
        return;
    }
    for (int i = 0; i < _n_dirty; i++) {
        const Rect& r = _dirty[i];
        for (int y = r.y; y < r.y + r.h; y++) {
            std::memset(_fb + (size_t)y * kScreenW + r.x, 0, (size_t)r.w * sizeof(uint16_t));
        }
    }
    _n_dirty = 0;

    for (int i = 0; i < _n_segs; i++) {
        int x0 = _segs[i].x0, y0 = _segs[i].y0;
        const int x1 = _segs[i].x1, y1 = _segs[i].y1;
        int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;
        int err      = dx + dy;
        int guard    = 1200;
        while (guard--) {
            if ((unsigned)x0 < (unsigned)kScreenW && (unsigned)y0 < (unsigned)kScreenH) {
                _fb[(size_t)y0 * kScreenW + x0] = 0;
            }
            if (x0 == x1 && y0 == y1) {
                break;
            }
            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
    _n_segs = 0;
}

void GlowCanvas::push()
{
    if (_flush != nullptr && _fb != nullptr) {
        _flush(_flush_ctx, _fb);
    }
}

void GlowCanvas::addGlowDot(float fx_, float fy_, const Rgb& color, float intensity, int radius)
{
    if (radius < 1) {
        radius = 1;
    }
    if (radius > kMaxGlowRadius) {
        radius = kMaxGlowRadius;
    }
    const int cx = (int)fx_ - radius;
    const int cy = (int)fy_ - radius;
    const int i256 = (int)(std::clamp(intensity, 0.0f, 1.0f) * 256.0f);

    markDirty(cx, cy, 2 * radius + 1, 2 * radius + 1);

    const int pr = color.r * i256 >> 8;
    const int pg = color.g * i256 >> 8;
    const int pb = color.b * i256 >> 8;

    const int side       = 2 * radius + 1;
    const uint8_t* s     = stamp_for(radius);

    for (int dy = 0; dy < side; dy++) {
        const int y = cy + dy;
        if ((unsigned)y >= (unsigned)kScreenH) {
            s += side;
            continue;
        }
        uint16_t* row = _fb + (size_t)y * kScreenW;
        for (int dx = 0; dx < side; dx++) {
            const int w = *s++;
            if (w == 0) {
                continue;
            }
            const int x = cx + dx;
            if ((unsigned)x >= (unsigned)kScreenW) {
                continue;
            }
            add_px(row + x, (uint8_t)(pr * w >> 8), (uint8_t)(pg * w >> 8), (uint8_t)(pb * w >> 8));
        }
    }
}

void GlowCanvas::addLine(float x0f, float y0f, float x1f, float y1f, uint8_t r, uint8_t g, uint8_t b)
{
    int x0 = (int)x0f, y0 = (int)y0f, x1 = (int)x1f, y1 = (int)y1f;
    if (_n_segs < kMaxSegs) {
        _segs[_n_segs++] = {(int16_t)x0, (int16_t)y0, (int16_t)x1, (int16_t)y1};
    } else {
        _full_clear = true;
    }
    int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int err      = dx + dy;
    int guard    = 1200;
    while (guard--) {
        addPixel(x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

}  // namespace fx
