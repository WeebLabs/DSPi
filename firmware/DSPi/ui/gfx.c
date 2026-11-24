#include "gfx.h"
#include "font5x7.h"
#include "font_block.h"
#include "font_round.h"
#include <math.h>
#include <string.h>

#define W UI_DISP_W
#define H UI_DISP_H

static uint16_t s_fb[W * H];
static int s_clip_y0 = 0, s_clip_y1 = H;
static int16_t s_sin[360];      // sin(deg) * 1024
static uint16_t s_dim = 256;    // colour scale /256 (the display level)

// Display font: 1 = "Block" squared capitals (font_block.h), 0 = the classic
// 5x7 font, smoothed when scaled. With Block, UI_FONT_ROUND = 1 draws the 2x,
// 3x and 4x sizes in the pre-rendered "geometric rounded" style (font_round.h);
// 0 keeps them crisp and square.
#ifndef UI_FONT_BLOCK
#define UI_FONT_BLOCK 1
#endif
#ifndef UI_FONT_ROUND
#define UI_FONT_ROUND 1
#endif
#if UI_FONT_BLOCK
static int8_t s_blk_idx[FONT_LAST - FONT_FIRST + 1];   // char -> font_block entry, -1 = none
static bool   s_keep_case;      // draw lowercase glyphs the font has (just "d", for dB)
#endif

static inline uint16_t swap16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }

// Native RGB565 -> framebuffer word: scaled by the display level, then
// byte-swapped for the 8-bit SPI DMA.
static inline uint16_t px(gfx_color_t c) {
    if (s_dim < 256) {
        uint32_t r = ((c >> 11) & 31u) * s_dim >> 8;
        uint32_t g = ((c >> 5) & 63u) * s_dim >> 8;
        uint32_t b = (c & 31u) * s_dim >> 8;
        c = (gfx_color_t)((r << 11) | (g << 5) | b);
    }
    return swap16(c);
}

void gfx_init(void) {
    for (int i = 0; i < 360; i++)
        s_sin[i] = (int16_t)lrintf(sinf((float)i * 0.01745329252f) * 1024.0f);
    s_clip_y0 = 0;
    s_clip_y1 = H;
    s_dim = 256;
    memset(s_fb, 0, sizeof(s_fb));
#if UI_FONT_BLOCK
    for (int i = 0; i <= FONT_LAST - FONT_FIRST; i++) s_blk_idx[i] = -1;
    for (unsigned k = 0; k < FONT_BLOCK_COUNT; k++)
        s_blk_idx[font_block[k].ch - FONT_FIRST] = (int8_t)k;
#endif
}

uint16_t *gfx_fb(void) { return s_fb; }

static const uint16_t s_level_scale[4] = { 64, 128, 192, 256 };

void gfx_set_level(uint8_t level) { s_dim = s_level_scale[level & 3]; }

uint8_t gfx_get_level(void) {
    for (uint8_t i = 0; i < 4; i++) if (s_level_scale[i] == s_dim) return i;
    return 3;
}

void gfx_set_clip(int y0, int y1) {
    s_clip_y0 = y0 < 0 ? 0 : y0;
    s_clip_y1 = y1 > H ? H : y1;
}

void gfx_clear_rows(int y0, int y1, gfx_color_t c) {
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    uint16_t v = px(c);
    for (int y = y0; y < y1; y++)
        for (int x = 0; x < W; x++) s_fb[y * W + x] = v;
}

void gfx_pixel(int x, int y, gfx_color_t c) {
    if ((unsigned)x >= (unsigned)W || y < s_clip_y0 || y >= s_clip_y1) return;
    s_fb[y * W + x] = px(c);
}

void gfx_hline(int x, int y, int w, gfx_color_t c) {
    if (y < s_clip_y0 || y >= s_clip_y1 || w <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > W) w = W - x;
    uint16_t v = px(c);
    uint16_t *p = &s_fb[y * W + x];
    for (int i = 0; i < w; i++) p[i] = v;
}

void gfx_vline(int x, int y, int h, gfx_color_t c) {
    if ((unsigned)x >= (unsigned)W || h <= 0) return;
    if (y < s_clip_y0) { h -= s_clip_y0 - y; y = s_clip_y0; }
    if (y + h > s_clip_y1) h = s_clip_y1 - y;
    uint16_t v = px(c);
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
            int px_ = cx + ((s * r + (s >= 0 ? 512 : -512)) / 1024);
            int py = cy - ((co * r + (co >= 0 ? 512 : -512)) / 1024);
            gfx_pixel(px_, py, c);
        }
    }
}

// ---------------------------------------------------------------- text ----
// Glyph rows as bit masks, bit i = column i (bit 0 leftmost), so a pixel's
// left neighbour is (row << 1) and its right neighbour (row >> 1).  Scaled
// text is smoothed instead of pixel-doubled: 2x is EPX (Scale2x), 3x is
// Scale3x and 4x is EPX twice, each evaluated a whole row at a time.

static uint32_t row1(uint8_t ch, int r) {
    if (r < 0 || r > 7) return 0;
    const uint8_t *g = &font5x7[(ch - FONT_FIRST) * 5];
    uint32_t m = 0;
    for (int i = 0; i < 5; i++)
        if ((g[i] >> r) & 1) m |= 1u << i;
    return m;
}

static uint32_t interleave2(uint32_t a, uint32_t b, int w) {
    uint32_t out = 0;
    for (int i = 0; i < w; i++)
        out |= (((a >> i) & 1u) << (2 * i)) | (((b >> i) & 1u) << (2 * i + 1));
    return out;
}

static uint32_t interleave3(uint32_t a, uint32_t b, uint32_t c, int w) {
    uint32_t out = 0;
    for (int i = 0; i < w; i++)
        out |= (((a >> i) & 1u) << (3 * i)) | (((b >> i) & 1u) << (3 * i + 1)) |
               (((c >> i) & 1u) << (3 * i + 2));
    return out;
}

// EPX on one source row (with its neighbours), top or bottom output row.
static uint32_t epx_row(uint32_t up, uint32_t p, uint32_t dn, int w, int bottom) {
    const uint32_t mask = (1u << w) - 1u;
    uint32_t A = up, D = dn, C = (p << 1) & mask, B = p >> 1;
    uint32_t o1, o2;
    if (!bottom) {
        uint32_t c1 = ~(C ^ A) & (C ^ D) & (A ^ B);   // top-left takes A
        uint32_t c2 = ~(A ^ B) & (A ^ C) & (B ^ D);   // top-right takes B
        o1 = ((c1 & A) | (~c1 & p)) & mask;
        o2 = ((c2 & B) | (~c2 & p)) & mask;
    } else {
        uint32_t c3 = ~(D ^ C) & (D ^ B) & (C ^ A);   // bottom-left takes C
        uint32_t c4 = ~(B ^ D) & (B ^ A) & (D ^ C);   // bottom-right takes D
        o1 = ((c3 & C) | (~c3 & p)) & mask;
        o2 = ((c4 & D) | (~c4 & p)) & mask;
    }
    return interleave2(o1, o2, w);
}

static uint32_t row2(uint8_t ch, int r2) {
    if (r2 < 0 || r2 > 15) return 0;
    int r = r2 >> 1;
    return epx_row(row1(ch, r - 1), row1(ch, r), row1(ch, r + 1), 5, r2 & 1);
}

static uint32_t row4(uint8_t ch, int r4) {
    int r2 = r4 >> 1;
    return epx_row(row2(ch, r2 - 1), row2(ch, r2), row2(ch, r2 + 1), 10, r4 & 1);
}

// Scale3x on the neighbourhood  A B C / D E F / G H I.
static uint32_t row3(uint8_t ch, int r3) {
    if (r3 < 0 || r3 > 23) return 0;
    int r = r3 / 3, k = r3 % 3;
    const uint32_t mask = 0x1F;
    uint32_t U = row1(ch, r - 1), E = row1(ch, r), L = row1(ch, r + 1);
    uint32_t A = (U << 1) & mask, B = U, C = U >> 1;
    uint32_t D = (E << 1) & mask, F = E >> 1;
    uint32_t G = (L << 1) & mask, Hh = L, I = L >> 1;
#define EQ(x, y) (~((x) ^ (y)))
#define NE(x, y) ((x) ^ (y))
#define SEL(c, x) ((((c) & (x)) | (~(c) & E)) & mask)
    uint32_t db = EQ(D, B) & NE(B, F) & NE(D, Hh);
    uint32_t bf = EQ(B, F) & NE(B, D) & NE(F, Hh);
    uint32_t dh = EQ(D, Hh) & NE(D, B) & NE(Hh, F);
    uint32_t hf = EQ(Hh, F) & NE(D, Hh) & NE(B, F);
    uint32_t o0, o1, o2;
    if (k == 0) {
        o0 = SEL(db, D);
        o1 = SEL((db & NE(E, C)) | (bf & NE(E, A)), B);
        o2 = SEL(bf, F);
    } else if (k == 1) {
        o0 = SEL((db & NE(E, G)) | (dh & NE(E, A)), D);
        o1 = E;
        o2 = SEL((bf & NE(E, I)) | (hf & NE(E, C)), F);
    } else {
        o0 = SEL(dh, D);
        o1 = SEL((dh & NE(E, I)) | (hf & NE(E, G)), Hh);
        o2 = SEL(hf, F);
    }
#undef EQ
#undef NE
#undef SEL
    return interleave3(o0, o1, o2, 5);
}

// Plain scaling: each source pixel becomes a scale x scale block, drawn as
// horizontal runs.
static void crisp_glyph(int x, int y, const uint8_t *cols, gfx_color_t c, int scale) {
    for (int r = 0; r < 8; r++) {
        uint32_t m = 0;
        for (int i = 0; i < 5; i++)
            if ((cols[i] >> r) & 1) m |= 1u << i;
        if (!m) continue;
        for (int k = 0; k < scale; k++) {
            int yy = y + r * scale + k;
            if (yy < s_clip_y0 || yy >= s_clip_y1) continue;
            uint32_t mm = m;
            while (mm) {
                int j = __builtin_ctz(mm);
                int n = __builtin_ctz(~(mm >> j));
                gfx_hline(x + j * scale, yy, n * scale, c);
                mm &= ~(((1u << n) - 1u) << j);
            }
        }
    }
}

static void glyph(int x, int y, char ch, gfx_color_t c, int scale) {
    if (ch < FONT_FIRST || ch > FONT_LAST) ch = '?';
    if (ch == ' ') return;
    if (y + 8 * scale + 2 <= s_clip_y0 || y - 2 >= s_clip_y1) return;   // +-2: rounded margin
#if UI_FONT_BLOCK
    // Block capitals where the font has the character, else the classic
    // glyph, both crisp so the two styles sit together. Lowercase draws as
    // capitals unless the caller asked to keep case.
    if (ch >= 'a' && ch <= 'z' && !(s_keep_case && s_blk_idx[ch - FONT_FIRST] >= 0))
        ch = (char)(ch - 32);
    int bi = s_blk_idx[ch - FONT_FIRST];
#if UI_FONT_ROUND
    if (bi >= 0 && scale >= 2 && scale <= 4) {
        int h = scale == 2 ? FONT_ROUND2_H : scale == 3 ? FONT_ROUND3_H : FONT_ROUND4_H;
        int x0 = x - FONT_ROUND_MARGIN, y0 = y - FONT_ROUND_MARGIN;
        for (int r = 0; r < h; r++) {
            int yy = y0 + r;
            if (yy < s_clip_y0) continue;
            if (yy >= s_clip_y1) break;
            uint32_t m = scale == 2 ? font_round2[bi][r]
                       : scale == 3 ? font_round3[bi][r] : font_round4[bi][r];
            while (m) {
                int j = __builtin_ctz(m);
                int n = __builtin_ctz(~(m >> j));
                gfx_hline(x0 + j, yy, n, c);
                m &= ~(((n < 32) ? ((1u << n) - 1u) : 0xFFFFFFFFu) << j);
            }
        }
        return;
    }
#endif
    crisp_glyph(x, y, bi >= 0 ? font_block[bi].col : &font5x7[(ch - FONT_FIRST) * 5], c, scale);
    return;
#endif
    if (scale > 4) {   // beyond the smoothers: plain blocks
        const uint8_t *g = &font5x7[(ch - FONT_FIRST) * 5];
        for (int col = 0; col < 5; col++)
            for (int row = 0; row < 8; row++)
                if (g[col] & (1u << row))
                    gfx_fill_rect(x + col * scale, y + row * scale, scale, scale, c);
        return;
    }
    for (int rr = 0; rr < 8 * scale; rr++) {
        int yy = y + rr;
        if (yy < s_clip_y0) continue;
        if (yy >= s_clip_y1) break;
        uint32_t m = scale == 2 ? row2((uint8_t)ch, rr)
                   : scale == 3 ? row3((uint8_t)ch, rr)
                   : scale == 4 ? row4((uint8_t)ch, rr)
                   : row1((uint8_t)ch, rr);
        // Runs of set bits become horizontal spans.
        while (m) {
            int j = __builtin_ctz(m);
            int n = __builtin_ctz(~(m >> j));
            gfx_hline(x + j, yy, n, c);
            m &= ~(((n < 32) ? ((1u << n) - 1u) : 0xFFFFFFFFu) << j);
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

void gfx_text_center_cased(int cx, int y, const char *s, gfx_color_t c, int scale) {
#if UI_FONT_BLOCK
    s_keep_case = true;
    gfx_text_center(cx, y, s, c, scale);
    s_keep_case = false;
#else
    gfx_text_center(cx, y, s, c, scale);
#endif
}

int gfx_text_fit(int x, int y, const char *s, gfx_color_t c, int max_w) {
    int scale = gfx_text_width(s, 2) <= max_w ? 2 : 1;
    return gfx_text(x, y + (scale == 1 ? 4 : 0), s, c, scale);
}
