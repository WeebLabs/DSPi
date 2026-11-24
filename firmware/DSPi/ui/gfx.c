#include "gfx.h"
#include "font5x7.h"
#include <math.h>
#include <string.h>

#define W UI_DISP_W
#define H UI_DISP_H

static uint16_t s_fb[W * H];
static int s_clip_y0 = 0, s_clip_y1 = H;
static int16_t s_sin[360];      // sin(deg) * 1024

static inline uint16_t swap16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }

void gfx_init(void) {
    for (int i = 0; i < 360; i++)
        s_sin[i] = (int16_t)lrintf(sinf((float)i * 0.01745329252f) * 1024.0f);
    s_clip_y0 = 0;
    s_clip_y1 = H;
    memset(s_fb, 0, sizeof(s_fb));
}

uint16_t *gfx_fb(void) { return s_fb; }

void gfx_set_clip(int y0, int y1) {
    s_clip_y0 = y0 < 0 ? 0 : y0;
    s_clip_y1 = y1 > H ? H : y1;
}

void gfx_clear_rows(int y0, int y1, gfx_color_t c) {
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    uint16_t v = swap16(c);
    for (int y = y0; y < y1; y++)
        for (int x = 0; x < W; x++) s_fb[y * W + x] = v;
}

void gfx_pixel(int x, int y, gfx_color_t c) {
    if ((unsigned)x >= (unsigned)W || y < s_clip_y0 || y >= s_clip_y1) return;
    s_fb[y * W + x] = swap16(c);
}

void gfx_hline(int x, int y, int w, gfx_color_t c) {
    if (y < s_clip_y0 || y >= s_clip_y1 || w <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > W) w = W - x;
    uint16_t v = swap16(c);
    uint16_t *p = &s_fb[y * W + x];
    for (int i = 0; i < w; i++) p[i] = v;
}

void gfx_vline(int x, int y, int h, gfx_color_t c) {
    if ((unsigned)x >= (unsigned)W || h <= 0) return;
    if (y < s_clip_y0) { h -= s_clip_y0 - y; y = s_clip_y0; }
    if (y + h > s_clip_y1) h = s_clip_y1 - y;
    uint16_t v = swap16(c);
    for (int i = 0; i < h; i++) s_fb[(y + i) * W + x] = v;
}

void gfx_fill_rect(int x, int y, int w, int h, gfx_color_t c) {
    if (w <= 0 || h <= 0) return;
    if (y < s_clip_y0) { h -= s_clip_y0 - y; y = s_clip_y0; }
    if (y + h > s_clip_y1) h = s_clip_y1 - y;
    for (int i = 0; i < h; i++) gfx_hline(x, y + i, w, c);
}

void gfx_rect(int x, int y, int w, int h, gfx_color_t c) {
    if (w <= 0 || h <= 0) return;
    gfx_hline(x, y, w, c);
    gfx_hline(x, y + h - 1, w, c);
    gfx_vline(x, y, h, c);
    gfx_vline(x + w - 1, y, h, c);
}

void gfx_line(int x0, int y0, int x1, int y1, gfx_color_t c) {
    int dx = x1 > x0 ? x1 - x0 : x0 - x1, sx = x0 < x1 ? 1 : -1;
    int dy = y1 > y0 ? y0 - y1 : y1 - y0, sy = y0 < y1 ? 1 : -1;   // -|dy|
    int err = dx + dy;
    for (;;) {
        gfx_pixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void gfx_fill_circle(int cx, int cy, int r, gfx_color_t c) {
    for (int dy = -r; dy <= r; dy++) {
        int yy = cy + dy;
        if (yy < s_clip_y0 || yy >= s_clip_y1) continue;
        int dx = 0;
        while ((dx + 1) * (dx + 1) + dy * dy <= r * r + r) dx++;
        gfx_hline(cx - dx, yy, 2 * dx + 1, c);
    }
}

void gfx_circle(int cx, int cy, int r, gfx_color_t c) {
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        gfx_pixel(cx + x, cy + y, c); gfx_pixel(cx - x, cy + y, c);
        gfx_pixel(cx + x, cy - y, c); gfx_pixel(cx - x, cy - y, c);
        gfx_pixel(cx + y, cy + x, c); gfx_pixel(cx - y, cy + x, c);
        gfx_pixel(cx + y, cy - x, c); gfx_pixel(cx - y, cy - x, c);
        y++;
        if (err < 0) err += 2 * y + 1;
        else { x--; err += 2 * (y - x) + 1; }
    }
}

void gfx_arc(int cx, int cy, int r_in, int r_out, int a0, int a1, gfx_color_t c) {
    // Skip the whole arc when its bounding rows are outside the clip window.
    if (cy + r_out < s_clip_y0 || cy - r_out >= s_clip_y1) return;
    for (int a = a0; a <= a1; a++) {
        int ai = ((a % 360) + 360) % 360;
        int s = s_sin[ai];
        int co = s_sin[(ai + 90) % 360];
        for (int r = r_in; r <= r_out; r++) {
            int px = cx + ((s * r + (s >= 0 ? 512 : -512)) / 1024);
            int py = cy - ((co * r + (co >= 0 ? 512 : -512)) / 1024);
            gfx_pixel(px, py, c);
        }
    }
}

static void glyph(int x, int y, char ch, gfx_color_t c, int scale) {
    if (ch < FONT_FIRST || ch > FONT_LAST) ch = '?';
    const uint8_t *g = &font5x7[(ch - FONT_FIRST) * 5];
    if (y + 8 * scale <= s_clip_y0 || y >= s_clip_y1) return;
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 8; row++) {
            if (!(bits & (1u << row))) continue;
            if (scale == 1) gfx_pixel(x + col, y + row, c);
            else gfx_fill_rect(x + col * scale, y + row * scale, scale, scale, c);
        }
    }
}

int gfx_text(int x, int y, const char *s, gfx_color_t c, int scale) {
    for (; *s; s++) {
        glyph(x, y, *s, c, scale);
        x += 6 * scale;
    }
    return x;
}

int gfx_text_width(const char *s, int scale) {
    int n = (int)strlen(s);
    return n ? (n * 6 - 1) * scale : 0;
}

void gfx_text_center(int cx, int y, const char *s, gfx_color_t c, int scale) {
    gfx_text(cx - gfx_text_width(s, scale) / 2, y, s, c, scale);
}
