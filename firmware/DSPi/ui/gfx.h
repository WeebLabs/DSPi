/*
 * gfx.h; tiny RGB565 framebuffer renderer for the 96x64 front-panel display
 *
 * Immediate-mode, no heap. The framebuffer holds byte-swapped pixels so it can
 * be sent to the SSD1331 as-is (big-endian RGB565) by an 8-bit SPI DMA.
 * Everything honours a row clip window, which lets the UI render a frame in
 * bands across several background steps by calling the same page-draw code
 * once per band.
 */
#ifndef GFX_H
#define GFX_H

#include <stdint.h>
#include <stdbool.h>
#include "ui_board.h"

typedef uint16_t gfx_color_t;   // native RGB565

#define GFX_RGB(r, g, b) ((gfx_color_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xFF) >> 3)))

void gfx_init(void);
uint16_t *gfx_fb(void);                     // byte-swapped pixels, UI_DISP_W * UI_DISP_H

void gfx_set_clip(int y0, int y1);          // draw only rows y0 <= y < y1
void gfx_clear_rows(int y0, int y1, gfx_color_t c);

void gfx_pixel(int x, int y, gfx_color_t c);
void gfx_hline(int x, int y, int w, gfx_color_t c);
void gfx_vline(int x, int y, int h, gfx_color_t c);
void gfx_rect(int x, int y, int w, int h, gfx_color_t c);        // outline
void gfx_fill_rect(int x, int y, int w, int h, gfx_color_t c);
void gfx_line(int x0, int y0, int x1, int y1, gfx_color_t c);
void gfx_fill_circle(int cx, int cy, int r, gfx_color_t c);
void gfx_circle(int cx, int cy, int r, gfx_color_t c);           // 1 px outline

// Arc band between radii r_in..r_out. Angles in degrees, 0 = 12 o'clock,
// clockwise; a1 may exceed 360.
void gfx_arc(int cx, int cy, int r_in, int r_out, int a0, int a1, gfx_color_t c);

// 5x7 text. scale >= 1. Returns the x just past the last glyph.
int  gfx_text(int x, int y, const char *s, gfx_color_t c, int scale);
int  gfx_text_width(const char *s, int scale);
void gfx_text_center(int cx, int y, const char *s, gfx_color_t c, int scale);

#endif
