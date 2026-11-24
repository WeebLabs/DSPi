/* ui_theme.h; OP-1-flavoured palette (RGB565) */
#ifndef UI_THEME_H
#define UI_THEME_H
#include "gfx.h"

#define C_BG      GFX_RGB(0, 0, 0)
#define C_BLUE    GFX_RGB(44, 108, 255)
#define C_GREEN   GFX_RGB(40, 205, 110)
#define C_WHITE   GFX_RGB(236, 236, 236)
#define C_ORANGE  GFX_RGB(255, 104, 20)
#define C_RED     GFX_RGB(255, 48, 64)
#define C_GREY    GFX_RGB(84, 84, 92)
#define C_DIM     GFX_RGB(34, 34, 40)
#define C_DARKFILL GFX_RGB(10, 22, 52)

// The four encoder colours of the OP-1, cycled over items.
static inline gfx_color_t ui_cycle_color(unsigned i) {
    static const gfx_color_t t[4] = { C_BLUE, C_GREEN, C_WHITE, C_ORANGE };
    return t[i & 3u];
}
#endif
