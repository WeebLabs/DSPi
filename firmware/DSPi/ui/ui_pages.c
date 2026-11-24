/*
 * ui_pages.c; the five pages, laid out for the 160x128 ST7735.
 *
 * Coordinates are absolute. Rows 0-17 are the header (page squares, page
 * name or sample rate), rows 108-127 the footer line, where toasts also
 * appear. Most text is 2x (the same share of the screen height that 1x had
 * on the original 96x64 OLED); gfx smooths scaled glyphs.
 */
#include "ui_pages.h"
#include "ui_theme.h"
#include "ui_nav.h"
#include "ui_eq.h"
#include <string.h>
#include <math.h>

const char *const ui_fx_labels[UI_FX_COUNT] = { "ldn", "xfd", "lvl", "tub", "bas", "eq" };
const char *const ui_fx_names[UI_FX_COUNT]  = { "loudness", "crossfeed", "leveller", "tube", "psybass", "eq" };

static const char *const page_name[UI_PG_COUNT] = { "now", "eq", "fx", "presets", "sys" };
static const gfx_color_t page_color[UI_PG_COUNT] = { C_BLUE, C_GREEN, C_WHITE, C_ORANGE, C_RED };

#define XR        156     // right text margin
#define FOOT_Y    110     // footer text line (2x)
#define TOAST_Y   108     // toast bar top

// ---------------------------------------------------------------- formatting

static char *put_int(char *o, int v) {
    char t[12];
    int n = 0;
    unsigned u;
    if (v < 0) { *o++ = '-'; u = (unsigned)(-v); } else u = (unsigned)v;
    do { t[n++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (n) *o++ = t[--n];
    *o = 0;
    return o;
}

// value with `dec` decimals (0 or 1), optional leading '+'
static char *put_fixed(char *o, float v, int dec, bool plus) {
    bool neg = v < 0.0f;
    float a = neg ? -v : v;
    int scaled = (int)(a * (dec ? 10.0f : 1.0f) + 0.5f);
    if (neg && scaled != 0) *o++ = '-'; else if (plus) *o++ = '+';
    o = put_int(o, dec ? scaled / 10 : scaled);
    if (dec) { *o++ = '.'; *o++ = (char)('0' + scaled % 10); *o = 0; }
    return o;
}

static void fmt_freq(char *o, float f) {
    if (f >= 10000.0f)      { o = put_fixed(o, f / 1000.0f, 0, false); *o++ = 'k'; *o = 0; }
    else if (f >= 1000.0f)  { o = put_fixed(o, f / 1000.0f, 1, false); *o++ = 'k'; *o = 0; }
    else                    { put_fixed(o, f, 0, false); }
}

static void upper_copy(char *dst, const char *src, int max) {
    int i = 0;
    for (; src[i] && i < max; i++) {
        char c = src[i];
        dst[i] = (c >= 'a' && c <= 'z') ? (char)(c - 32) : c;
    }
    dst[i] = 0;
}

static void lower_copy(char *dst, const char *src, int max) {
    int i = 0;
    for (; src[i] && i < max; i++) {
        char c = src[i];
        dst[i] = (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c;
    }
    dst[i] = 0;
}

// ------------------------------------------------------------------- pieces

static void draw_header(const UiView *v) {
    for (int i = 0; i < UI_PG_COUNT; i++) {
        int x = 4 + i * 13;
        if (i == (int)v->page) gfx_fill_rect(x, 4, 10, 10, page_color[i]);
        else                   gfx_rect(x, 4, 10, 10, C_GREY);
    }
    if (v->page == UI_PG_NOW) {
        static const char *const rate[3] = { "44k", "48k", "96k" };
        const char *r = rate[v->live.rate_idx < 3 ? v->live.rate_idx : 1];
        gfx_color_t c = v->live.streaming ? C_WHITE : C_GREY;
        int w = gfx_text_width(r, 2);
        gfx_text(XR - w, 2, r, c, 2);
        gfx_fill_circle(XR - w - 9, 9, 4, v->live.streaming ? C_GREEN : C_GREY);
    } else {
        const char *n = page_name[v->page];
        int w = gfx_text_width(n, 2);
        if (v->in_menu) {
            gfx_fill_rect(XR - w - 4, 0, w + 8, 18, page_color[v->page]);
            gfx_text(XR - w, 2, n, C_BG, 2);
        } else {
            gfx_text(XR - w, 2, n, page_color[v->page], 2);
        }
    }
}

// "p3 movie": the number in orange, the name 2x when it fits before
// right_x, else 1x (names are up to 12 characters).
static void draw_preset_footer(const UiView *v, int idx, int y, int right_x) {
    char buf[16], nm[14];
    char *p = buf;
    *p++ = 'p';
    put_int(p, idx);
    gfx_text(4, y, buf, C_ORANGE, 2);
    int x = 4 + gfx_text_width(buf, 2) + 12;
    if (idx >= 0 && idx < UI_PRESETS && v->preset_valid[idx]) {
        lower_copy(nm, v->preset_name[idx], 12);
        gfx_text_fit(x, y, nm, C_WHITE, right_x - x);
    } else {
        gfx_text(x, y, "empty", C_GREY, 2);
    }
}

// "<" drawn as two 3-pixel strokes; s = half height.
static void draw_chevron(int cx, int cy, int s, gfx_color_t c) {
    for (int k = 0; k < 3; k++) {
        gfx_line(cx + s / 2 + k, cy - s, cx - s / 2 + k, cy, c);
        gfx_line(cx - s / 2 + k, cy, cx + s / 2 + k, cy + s, c);
    }
}

static void draw_tile_frame(int x, int y, int w, int h, bool sel) {
    gfx_rect(x + 1, y + 1, w - 2, h - 2, sel ? C_WHITE : C_DIM);
    if (sel) gfx_rect(x + 2, y + 2, w - 4, h - 4, C_WHITE);
}

// ---------------------------------------------------------------------- NOW

#define MTR_SEGS   14
#define MTR_BOTTOM 102     // y of the lowest segment
#define MTR_PITCH  5
#define MTR_ORANGE 8       // first orange segment
#define MTR_RED    10      // first of the top quarter: the near-clip zone

static void draw_meter(int x, int ch, const UiView *v) {
    int lit = (int)(v->meter[ch] * MTR_SEGS + 0.5f);
    int pk = (int)(v->peak[ch] * MTR_SEGS);
    if (pk > MTR_SEGS - 1) pk = MTR_SEGS - 1;
    for (int i = 0; i < MTR_SEGS; i++) {
        // The near-clip zone always shows, dim red, and lights bright red
        // only when the level reaches it.
        gfx_color_t col = i < MTR_ORANGE ? C_GREEN : (i < MTR_RED ? C_ORANGE : C_RED);
        gfx_color_t c = (i < lit) ? col : (i >= MTR_RED ? C_DIMRED : C_DIM);
        if (i == pk && pk >= lit && v->peak[ch] > 0.02f) c = C_WHITE;
        gfx_fill_rect(x, MTR_BOTTOM - i * MTR_PITCH, 18, 3, c);
    }
}

static void draw_now(const UiView *v) {
    const UiLive *l = &v->live;
    const int cx = 51, cy = 68;          // dial: radius 40-47, clear of the meters at x 104

    float span = l->vol_hi_db - l->vol_lo_db;
    float t = span > 0.1f ? (l->vol_db - l->vol_lo_db) / span : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    gfx_arc(cx, cy, 40, 47, 225, 495, C_DIM);
    if (l->mute) {
        gfx_text_center(cx, cy - 10, "mute", C_ORANGE, 3);
    } else {
        int a1 = 225 + (int)(270.0f * t + 0.5f);
        if (a1 > 225) gfx_arc(cx, cy, 40, 47, 225, a1, C_ORANGE);
        gfx_arc(cx, cy, 37, 50, a1 - 2, a1 + 2, C_WHITE);       // knob tick

        char buf[12];
        put_int(buf, (int)lrintf(l->vol_db));
        // 4x when it fits inside the dial (80 px across), else 3x.
        int sc = gfx_text_width(buf, 4) <= 72 ? 4 : 3;
        gfx_text_center(cx, cy - (sc == 4 ? 20 : 15), buf, C_WHITE, sc);
        gfx_text_center_cased(cx, cy + 14, "dB", C_GREY, 2);
    }

    gfx_text_center(104 + 9, 20, "l", C_GREY, 2);
    gfx_text_center(130 + 9, 20, "r", C_GREY, 2);
    draw_meter(104, 0, v);
    draw_meter(130, 1, v);

    // Footer: the input source in capitals, white while it has signal.
    int right = XR;
    if (l->clip) {
        int w = gfx_text_width("clip", 2);
        gfx_text(XR - w, FOOT_Y, "clip", C_RED, 2);
        right = XR - w - 8;
    }
    char nm[14];
    upper_copy(nm, l->src, 12);
    gfx_text_fit(4, FOOT_Y, nm, l->src_active ? C_WHITE : C_GREY, right - 4);
}

// ----------------------------------------------------------------------- EQ

#define GX0   4
#define GY0   20
#define GH    76
#define GZERO 58      // 0 dB row: 38 px above (+12.7 dB), 37 below

// curve_q is in quarter-dB; 3 px per dB.
static int q_to_y(int q) {
    int y = GZERO - ((q * 3 + (q >= 0 ? 2 : -2)) / 4);
    if (y < GY0) y = GY0;
    if (y > GY0 + GH - 1) y = GY0 + GH - 1;
    return y;
}

static void draw_eq(const UiView *v) {
    const UiEq *e = &v->eq;

    static const float marks[3] = { 100.0f, 1000.0f, 10000.0f };
    static const char *const mark_txt[3] = { "100", "1k", "10k" };
    for (int m = 0; m < 3; m++) {
        int x = GX0 + ui_eq_col_for_freq(marks[m]);
        for (int y = GY0; y < GY0 + GH; y += 3) gfx_pixel(x, y, C_DIM);
        gfx_text_center(x, GY0 + GH + 2, mark_txt[m], C_GREY, 1);
    }
    for (int x = GX0; x < GX0 + UI_EQ_COLS; x += 2) gfx_pixel(x, GZERO, C_GREY);

    if (e->curve_valid) {
        int prev_y = 0;
        for (int c = 0; c < UI_EQ_COLS; c++) {
            int y = q_to_y(e->curve_q[c]);
            if (y < GZERO) gfx_vline(GX0 + c, y, GZERO - y, C_DARKFILL);
            else if (y > GZERO) gfx_vline(GX0 + c, GZERO + 1, y - GZERO, C_DARKFILL);
            if (c > 0) {
                gfx_line(GX0 + c - 1, prev_y, GX0 + c, y, C_WHITE);
                gfx_line(GX0 + c - 1, prev_y + 1, GX0 + c, y + 1, C_WHITE);   // 2 px curve
            }
            prev_y = y;
        }
        for (int i = 0; i < e->nb; i++) {
            int col = ui_eq_col_for_freq(e->band[i].freq);
            int x = GX0 + col, y = q_to_y(e->curve_q[col]);
            gfx_color_t c = ui_cycle_color((unsigned)i);
            if (e->band[i].bypass) { gfx_circle(x, y, 4, C_GREY); gfx_circle(x, y, 3, C_GREY); }
            else                   gfx_fill_circle(x, y, 4, c);
            if (v->in_menu && v->cursor == i) { gfx_circle(x, y, 7, C_WHITE); gfx_circle(x, y, 8, C_WHITE); }
        }
    }

    if (v->in_menu && v->cursor < e->nb) {
        const UiEqBand *b = &e->band[v->cursor];
        char buf[24], *p = buf;
        fmt_freq(p, b->freq); p += strlen(p);
        *p++ = ' ';
        p = put_fixed(p, b->gain_db, 1, true);
        if (b->bypass) { memcpy(p, " off", 5); }
        else { *p++ = ' '; *p++ = 'q'; put_fixed(p, b->q, 1, false); }
        gfx_text_fit(4, FOOT_Y, buf, b->bypass ? C_GREY : ui_cycle_color(v->cursor), XR - 4);
    } else if (v->in_menu) {
        gfx_text(4, FOOT_Y, "< back", C_WHITE, 2);
    } else if (e->nb == 0) {
        gfx_text(4, FOOT_Y, "no eq bands", C_GREY, 2);
    } else {
        draw_preset_footer(v, v->live.preset, FOOT_Y, XR);
    }
}

// ----------------------------------------------------------------- tile grid

static void draw_fx(const UiView *v) {
    for (int i = 0; i < 8; i++) {
        int tx = (i % 4) * 40, ty = 20 + (i / 4) * 54;
        bool sel = v->in_menu && v->cursor == i;
        if (i == UI_FX_COUNT + 1) continue;                 // spare slot
        draw_tile_frame(tx, ty, 40, 52, sel);
        if (i < UI_FX_COUNT) {
            bool on = v->live.fx_on[i];
            gfx_color_t c = ui_cycle_color((unsigned)i);
            if (on) gfx_fill_circle(tx + 20, ty + 17, 10, c);
            else  { gfx_circle(tx + 20, ty + 17, 10, C_GREY); gfx_circle(tx + 20, ty + 17, 9, C_GREY); }
            gfx_text_center(tx + 20, ty + 32, ui_fx_labels[i], on || sel ? C_WHITE : C_GREY, 2);
        } else {
            draw_chevron(tx + 18, ty + 26, 9, sel ? C_WHITE : C_GREY);
        }
    }
}

static void draw_preset(const UiView *v) {
    for (int i = 0; i < UI_MENU_PRESET_ROWS; i++) {
        int tx = 2 + (i % 6) * 26, ty = 20 + (i / 6) * 42;
        bool sel = v->in_menu && v->cursor == i;
        if (i < UI_PRESETS) {
            bool active = (i == v->live.preset);
            if (active) gfx_fill_rect(tx + 1, ty + 1, 24, 38, C_ORANGE);
            else        draw_tile_frame(tx, ty, 26, 40, sel);
            if (active && sel) { gfx_rect(tx, ty, 26, 40, C_WHITE); gfx_rect(tx - 1, ty - 1, 28, 42, C_WHITE); }
            char d[2] = { (char)('0' + i), 0 };
            gfx_color_t c = active ? C_BG : (v->preset_valid[i] ? C_WHITE : C_GREY);
            gfx_text_center(tx + 13, ty + 9, d, c, 3);
        } else if (i == UI_PRESETS) {                        // save
            draw_tile_frame(tx, ty, 26, 40, sel);
            for (int k = 0; k < 2; k++) {
                gfx_vline(tx + 12 + k, ty + 7, 18, C_GREEN);
                gfx_line(tx + 5 + k, ty + 17, tx + 12 + k, ty + 24, C_GREEN);
                gfx_line(tx + 20 - k, ty + 17, tx + 13 - k, ty + 24, C_GREEN);
            }
            gfx_fill_rect(tx + 5, ty + 30, 16, 2, C_GREEN);
        } else {                                             // back
            draw_tile_frame(tx, ty, 26, 40, sel);
            draw_chevron(tx + 12, ty + 20, 8, C_WHITE);
        }
    }
    if (v->in_menu && v->cursor == UI_PRESETS) {
        char b[16], *p = b;
        memcpy(p, "save to p", 9); p += 9;
        put_int(p, v->live.preset);
        gfx_text(4, FOOT_Y, b, C_GREEN, 2);
    } else if (v->in_menu && v->cursor > UI_PRESETS) {
        gfx_text(4, FOOT_Y, "< back", C_WHITE, 2);
    } else {
        draw_preset_footer(v, v->in_menu ? v->cursor : v->live.preset, FOOT_Y, XR);
    }
}

static void draw_pips(int x, int y, int lit, gfx_color_t c) {
    for (int i = 0; i < 4; i++)
        gfx_fill_rect(x + i * 8, y, 6, 6, i < lit ? c : C_DIM);
}

static void draw_sys(const UiView *v) {
    static const char *const rate[3] = { "44.1k", "48k", "96k" };
    char buf[24], *p;

    p = buf; memcpy(p, "rate ", 5); p += 5;
    strcpy(p, rate[v->live.rate_idx < 3 ? v->live.rate_idx : 1]);
    gfx_text(4, 22, buf, C_WHITE, 2);

    p = buf; memcpy(p, "cpu ", 4); p += 4;
    p = put_int(p, v->live.cpu_pct); *p++ = '%'; *p = 0;
    gfx_text(4, 40, buf, C_WHITE, 2);

    gfx_text(4, 58, v->live.streaming ? "usb streaming" : "usb idle",
             v->live.streaming ? C_GREEN : C_GREY, 2);

    for (int i = 0; i < UI_MENU_SYS_ROWS; i++) {
        int tx = i * 40, ty = 76;
        bool sel = v->in_menu && v->cursor == i;
        draw_tile_frame(tx, ty, 40, 50, sel);
        const char *lab = i == 0 ? "bri" : i == 1 ? "led" : i == 2 ? "clr" : NULL;
        if (i == 0)      draw_pips(tx + 5, ty + 9, v->disp_level + 1, C_ORANGE);
        else if (i == 1) draw_pips(tx + 5, ty + 9, v->led_level, C_GREEN);
        else if (i == 2) gfx_fill_circle(tx + 20, ty + 12, 6, v->live.clip ? C_RED : C_GREY);
        if (lab) gfx_text_center(tx + 20, ty + 26, lab, sel ? C_WHITE : C_GREY, 2);
        else     draw_chevron(tx + 18, ty + 25, 9, sel ? C_WHITE : C_GREY);
    }
}

// --------------------------------------------------------------------- band

void ui_render_band(const UiView *v, int band) {
    int y0 = band * UI_BAND_ROWS, y1 = y0 + UI_BAND_ROWS;
    gfx_set_clip(y0, y1);
    gfx_clear_rows(y0, y1, C_BG);

    draw_header(v);
    switch (v->page) {
    case UI_PG_NOW:    draw_now(v);    break;
    case UI_PG_EQ:     draw_eq(v);     break;
    case UI_PG_FX:     draw_fx(v);     break;
    case UI_PG_PRESET: draw_preset(v); break;
    case UI_PG_SYS:    draw_sys(v);    break;
    default: break;
    }

    if (v->toast_on) {
        gfx_fill_rect(0, TOAST_Y, UI_DISP_W, UI_DISP_H - TOAST_Y, C_ORANGE);
        int sc = gfx_text_width(v->toast, 2) <= UI_DISP_W - 8 ? 2 : 1;
        gfx_text_center(UI_DISP_W / 2, sc == 2 ? TOAST_Y + 3 : TOAST_Y + 7, v->toast, C_BG, sc);
    }
    if (v->hold_pct) {
        gfx_fill_rect(0, UI_DISP_H - 3, (UI_DISP_W * v->hold_pct) / 100, 3, C_WHITE);
    }
}
