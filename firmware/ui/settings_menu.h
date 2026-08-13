#pragma once

#include <stdbool.h>

#include "display_face.h"
#include "orb_pmu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the settings menu subsystem. Safe to call once. Allocates
 * a 5x7 bitmap font in PSRAM (cheap). Must be called AFTER
 * display_face_init() (uses the canvas).
 */
esp_err_t settings_menu_init(void);

/**
 * Show / hide the settings menu overlay. When visible, the render task
 * takes a menu render path instead of the dome/CSI path (the menu
 * consumes the whole screen). Touch gestures are suspended while the
 * menu is open. Thread-safe.
 */
void settings_menu_set_visible(bool visible);
bool settings_menu_is_visible(void);

/**
 * Set brightness 0..100. Calls M5GFX setBrightness under the hood.
 * Thread-safe (locks the face mutex). Also cached for the menu display.
 */
void settings_menu_set_brightness(int pct);
int  settings_menu_get_brightness(void);

/**
 * Navigate the menu: previous/next item (touch swipe up/down or the
 * power key short-press), select/adjust (tap or long-press).
 *
 * Direction: -1 = up, +1 = down, 0 = select/toggle.
 */
void settings_menu_navigate(int direction);

/**
 * Render the settings menu into the glow canvas. Called from the render
 * task instead of the dome/CSI render when the menu is visible.
 * Uses addLine/addPixel/addGlowDot — the same primitives as the dome.
 */
void settings_menu_render(void);

#ifdef __cplusplus
}
#endif
