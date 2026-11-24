/*
 * ui_pages.h; page rendering. The frame is drawn in UI_BANDS horizontal bands
 * so that each background step stays short; every band call redraws the whole
 * page clipped to its rows, so a view must stay unchanged across a frame's bands.
 */
#ifndef UI_PAGES_H
#define UI_PAGES_H

#include "ui_types.h"
#include "gfx.h"

#define UI_BANDS      8
#define UI_BAND_ROWS  (UI_DISP_H / UI_BANDS)

void ui_render_band(const UiView *v, int band);

// Strings shared with the controller for toasts.
extern const char *const ui_fx_labels[UI_FX_COUNT];   // 3-char tile labels
extern const char *const ui_fx_names[UI_FX_COUNT];    // full names for toasts

#endif
