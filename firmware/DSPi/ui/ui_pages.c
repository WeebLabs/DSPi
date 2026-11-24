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
        int x = 2 + i * 10;
        if (i == (int)v->page) gfx_fill_rect(x, 1, 7, 7, page_color[i]);
        else                   gfx_rect(x, 1, 7, 7, C_GREY);
    }
    if (v->page == UI_PG_NOW) {
        static const char *const rate[3] = { "44k", "48k", "96k" };
        const char *r = rate[v->live.rate_idx < 3 ? v->live.rate_idx : 1];
        gfx_color_t c = v->live.streaming ? C_WHITE : C_GREY;
        int w = gfx_text_width(r, 1);
        gfx_text(95 - w, 1, r, c, 1);
        gfx_fill_circle(95 - w - 5, 4, 2, v->live.streaming ? C_GREEN : C_GREY);
    } else {
        const char *n = page_name[v->page];
        int w = gfx_text_width(n, 1);
        if (v->in_menu) {
            gfx_fill_rect(95 - w - 3, 0, w + 4, 9, page_color[v->page]);
            gfx_text(95 - w - 1, 1, n, C_BG, 1);
        } else {
            gfx_text(95 - w, 1, n, page_color[v->page], 1);
        }
    }
}

static void draw_preset_footer(const UiView *v, int idx, int y, int maxname) {
    char buf[16], nm[14];
    char *p = buf;
    *p++ = 'p';
    put_int(p, idx);
    gfx_text(2, y, buf, C_ORANGE, 1);
    int x = 2 + gfx_text_width(buf, 1) + 6;
    if (idx >= 0 && idx < UI_PRESETS && v->preset_valid[idx]) {
        lower_copy(nm, v->preset_name[idx], maxname);
        gfx_text(x, y, nm, C_WHITE, 1);
    } else {
        gfx_text(x, y, "empty", C_GREY, 1);
    }
}

static void draw_chevron(int cx, int cy, gfx_color_t c) {
    gfx_line(cx + 2, cy - 4, cx - 2, cy, c);
    gfx_line(cx - 2, cy, cx + 2, cy + 4, c);
    gfx_line(cx + 3, cy - 4, cx - 1, cy, c);
    gfx_line(cx - 1, cy, cx + 3, cy + 4, c);
}

// ---------------------------------------------------------------------- NOW

static void draw_meter(int x, int ch, const UiView *v) {
    static const int SEGS = 10;
    int lit = (int)(v->meter[ch] * SEGS + 0.5f);
    int pk = (int)(v->peak[ch] * SEGS);
    if (pk > SEGS - 1) pk = SEGS - 1;
    for (int i = 0; i < SEGS; i++) {
        gfx_color_t col = i < 6 ? C_GREEN : (i < 8 ? C_ORANGE : C_RED);
        gfx_color_t c = (i < lit) ? col : C_DIM;
        if (i == pk && pk >= lit && v->peak[ch] > 0.02f) c = C_WHITE;
        gfx_fill_rect(x, 49 - i * 3, 9, 2, c);
    }
}

static void draw_now(const UiView *v) {
    const UiLive *l = &v->live;
    const int cx = 24, cy = 34;

    float span = l->vol_hi_db - l->vol_lo_db;
    float t = span > 0.1f ? (l->vol_db - l->vol_lo_db) / span : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    gfx_arc(cx, cy, 17, 20, 225, 495, C_DIM);
    if (l->mute) {
        gfx_text_center(cx, cy - 3, "mute", C_ORANGE, 1);
    } else {
        int a1 = 225 + (int)(270.0f * t + 0.5f);
        if (a1 > 225) gfx_arc(cx, cy, 17, 20, 225, a1, C_ORANGE);
        gfx_arc(cx, cy, 15, 22, a1 - 1, a1 + 1, C_WHITE);       // knob tick

        char buf[12];
        put_int(buf, (int)lrintf(l->vol_db));
        gfx_text_center(cx, cy - 8, buf, C_WHITE, 2);
        gfx_text_center(cx, cy + 9, "db", C_GREY, 1);
    }

    gfx_text(62 + 3, 12, "l", C_GREY, 1);
    gfx_text(78 + 3, 12, "r", C_GREY, 1);
    draw_meter(62, 0, v);
    draw_meter(78, 1, v);

    draw_preset_footer(v, l->preset, 56, l->clip ? 6 : 11);
    if (l->clip) gfx_text(95 - gfx_text_width("clip", 1), 56, "clip", C_RED, 1);
}

// ----------------------------------------------------------------------- EQ

#define GX0 2
#define GY0 12
#define GH  38
#define GZERO 31

static int q_to_y(int q) {
    int y = GZERO - ((q * 3 + (q >= 0 ? 4 : -4)) / 8);
    if (y < GY0) y = GY0;
    if (y > GY0 + GH - 1) y = GY0 + GH - 1;
    return y;
}

static void draw_eq(const UiView *v) {
    const UiEq *e = &v->eq;

    static const float marks[3] = { 100.0f, 1000.0f, 10000.0f };
    for (int m = 0; m < 3; m++) {
        int x = GX0 + ui_eq_col_for_freq(marks[m]);
        for (int y = GY0; y < GY0 + GH; y += 3) gfx_pixel(x, y, C_DIM);
    }
    for (int x = GX0; x < GX0 + UI_EQ_COLS; x += 2) gfx_pixel(x, GZERO, C_GREY);

    if (e->curve_valid) {
        int prev_y = 0;
        for (int c = 0; c < UI_EQ_COLS; c++) {
            int y = q_to_y(e->curve_q[c]);
            if (y < GZERO) gfx_vline(GX0 + c, y, GZERO - y, C_DARKFILL);
            else if (y > GZERO) gfx_vline(GX0 + c, GZERO + 1, y - GZERO, C_DARKFILL);
            if (c > 0) gfx_line(GX0 + c - 1, prev_y, GX0 + c, y, C_WHITE);
            prev_y = y;
        }
        for (int i = 0; i < e->nb; i++) {
            int col = ui_eq_col_for_freq(e->band[i].freq);
            int x = GX0 + col, y = q_to_y(e->curve_q[col]);
            gfx_color_t c = ui_cycle_color((unsigned)i);
            if (e->band[i].bypass) gfx_circle(x, y, 2, C_GREY);
            else                   gfx_fill_circle(x, y, 2, c);
            if (v->in_menu && v->cursor == i) gfx_circle(x, y, 4, C_WHITE);
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
        gfx_text(2, 54, buf, b->bypass ? C_GREY : ui_cycle_color(v->cursor), 1);
    } else if (v->in_menu) {
        gfx_text(2, 54, "< back", C_WHITE, 1);
    } else if (e->nb == 0) {
        gfx_text(2, 54, "no eq bands", C_GREY, 1);
    } else {
        draw_preset_footer(v, v->live.preset, 54, 11);
    }
}

// ----------------------------------------------------------------- tile grid

static void draw_tile_frame(int x, int y, int w, int h, bool sel) {
    gfx_rect(x + 1, y + 1, w - 2, h - 2, sel ? C_WHITE : C_DIM);
}

static void draw_fx(const UiView *v) {
    for (int i = 0; i < 8; i++) {
        int tx = (i % 4) * 24, ty = 11 + (i / 4) * 25;
        bool sel = v->in_menu && v->cursor == i;
        if (i == UI_FX_COUNT + 1) continue;                 // spare slot
        draw_tile_frame(tx, ty, 24, 24, sel);
        if (i < UI_FX_COUNT) {
            bool on = v->live.fx_on[i];
            gfx_color_t c = ui_cycle_color((unsigned)i);
            if (on) gfx_fill_circle(tx + 12, ty + 8, 5, c);
            else    gfx_circle(tx + 12, ty + 8, 5, C_GREY);
            gfx_text_center(tx + 12, ty + 15, ui_fx_labels[i], on || sel ? C_WHITE : C_GREY, 1);
        } else {
            draw_chevron(tx + 12, ty + 12, sel ? C_WHITE : C_GREY);
        }
    }
}

static void draw_preset(const UiView *v) {
    for (int i = 0; i < UI_MENU_PRESET_ROWS; i++) {
        int tx = (i % 6) * 16, ty = 11 + (i / 6) * 22;
        bool sel = v->in_menu && v->cursor == i;
        if (i < UI_PRESETS) {
            bool active = (i == v->live.preset);
            if (active) gfx_fill_rect(tx + 1, ty + 1, 14, 20, C_ORANGE);
            else        gfx_rect(tx + 1, ty + 1, 14, 20, sel ? C_WHITE : C_DIM);
            if (active && sel) gfx_rect(tx, ty, 16, 22, C_WHITE);
            char d[2] = { (char)('0' + i), 0 };
            gfx_color_t c = active ? C_BG : (v->preset_valid[i] ? C_WHITE : C_GREY);
            gfx_text_center(tx + 8, ty + 4, d, c, 2);
        } else if (i == UI_PRESETS) {                        // save
            draw_tile_frame(tx, ty, 16, 22, sel);
            gfx_vline(tx + 8, ty + 4, 9, C_GREEN);
            gfx_line(tx + 4, ty + 9, tx + 8, ty + 13, C_GREEN);
            gfx_line(tx + 12, ty + 9, tx + 8, ty + 13, C_GREEN);
            gfx_hline(tx + 3, ty + 17, 11, C_GREEN);
        } else {                                             // back
            draw_tile_frame(tx, ty, 16, 22, sel);
            draw_chevron(tx + 8, ty + 11, C_WHITE);
        }
    }
    if (v->in_menu && v->cursor == UI_PRESETS) {
        char b[16], *p = b;
        memcpy(p, "save to p", 9); p += 9;
        put_int(p, v->live.preset);
        gfx_text(2, 56, b, C_GREEN, 1);
    } else if (v->in_menu && v->cursor > UI_PRESETS) {
        gfx_text(2, 56, "< back", C_WHITE, 1);
    } else {
        draw_preset_footer(v, v->in_menu ? v->cursor : v->live.preset, 56, 11);
    }
}

static void draw_pips(int x, int y, int lit, gfx_color_t c) {
    for (int i = 0; i < 4; i++)
        gfx_fill_rect(x + i * 5, y, 3, 3, i < lit ? c : C_DIM);
}

static void draw_sys(const UiView *v) {
    static const char *const rate[3] = { "44.1k", "48k", "96k" };
    char buf[24], *p;

    p = buf; memcpy(p, "rate ", 5); p += 5;
    strcpy(p, rate[v->live.rate_idx < 3 ? v->live.rate_idx : 1]);
    gfx_text(2, 12, buf, C_WHITE, 1);

    p = buf; memcpy(p, "cpu ", 4); p += 4;
    p = put_int(p, v->live.cpu_pct); *p++ = '%'; *p = 0;
    gfx_text(2, 21, buf, C_WHITE, 1);

    gfx_text(2, 30, v->live.streaming ? "usb streaming" : "usb idle",
             v->live.streaming ? C_GREEN : C_GREY, 1);

    for (int i = 0; i < UI_MENU_SYS_ROWS; i++) {
        int tx = i * 24, ty = 41;
        bool sel = v->in_menu && v->cursor == i;
        draw_tile_frame(tx, ty, 24, 22, sel);
        const char *lab = i == 0 ? "bri" : i == 1 ? "led" : i == 2 ? "clr" : NULL;
        if (i == 0)      draw_pips(tx + 3, ty + 5, v->disp_level + 1, C_ORANGE);
        else if (i == 1) draw_pips(tx + 3, ty + 5, v->led_level, C_GREEN);
        else if (i == 2) gfx_fill_circle(tx + 12, ty + 7, 3, v->live.clip ? C_RED : C_GREY);
        if (lab) gfx_text_center(tx + 12, ty + 13, lab, sel ? C_WHITE : C_GREY, 1);
        else     draw_chevron(tx + 12, ty + 11, sel ? C_WHITE : C_GREY);
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
        gfx_fill_rect(0, 54, UI_DISP_W, 10, C_ORANGE);
        gfx_text_center(UI_DISP_W / 2, 55, v->toast, C_BG, 1);
    }
    if (v->hold_pct) {
        gfx_fill_rect(0, 62, (UI_DISP_W * v->hold_pct) / 100, 2, C_WHITE);
    }
}
