/*
 * Host simulator: builds the pure UI core (gfx, pages, nav, button, eq) with
 * gcc, renders every page to a PPM, and runs logic checks. No hardware, no SDK.
 *   make -C ui/test run
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "../gfx.h"
#include "../ui_pages.h"
#include "../ui_nav.h"
#include "../ui_eq.h"
#include "../ui_button.h"

static int g_fail;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } } while (0)

static void dump(const char *path, int scale) {
    FILE *f = fopen(path, "wb");
    fprintf(f, "P6\n%d %d\n255\n", UI_DISP_W * scale, UI_DISP_H * scale);
    const uint16_t *fb = gfx_fb();
    for (int y = 0; y < UI_DISP_H; y++)
        for (int sy = 0; sy < scale; sy++)
            for (int x = 0; x < UI_DISP_W; x++) {
                uint16_t v = (uint16_t)((fb[y * UI_DISP_W + x] << 8) | (fb[y * UI_DISP_W + x] >> 8));
                uint8_t px[3] = { (uint8_t)((v >> 8) & 0xF8), (uint8_t)((v >> 3) & 0xFC), (uint8_t)((v << 3) & 0xF8) };
                for (int sx = 0; sx < scale; sx++) fwrite(px, 1, 3, f);
            }
    fclose(f);
}

static void render(const UiView *v, const char *path) {
    for (int b = 0; b < UI_BANDS; b++) ui_render_band(v, b);
    dump(path, 6);
}

static void make_eq(UiEq *e) {
    memset(e, 0, sizeof(*e));
    e->nb = 5;
    e->band[0] = (UiEqBand){ EQT_LOWSHELF, false, 80.0f, 3.0f, 0.7f };
    e->band[1] = (UiEqBand){ EQT_PEAKING, false, 250.0f, -4.0f, 1.2f };
    e->band[2] = (UiEqBand){ EQT_PEAKING, false, 1200.0f, 2.5f, 1.4f };
    e->band[3] = (UiEqBand){ EQT_PEAKING, true, 4000.0f, -6.0f, 2.0f };
    e->band[4] = (UiEqBand){ EQT_HIGHSHELF, false, 9000.0f, 4.0f, 0.7f };
    ui_eq_calc_all(e, 48000.0f);
}

static void make_view(UiView *v) {
    memset(v, 0, sizeof(*v));
    v->live.vol_db = -23.0f; v->live.vol_lo_db = -60.0f; v->live.vol_hi_db = 0.0f;
    v->live.streaming = true; v->live.rate_idx = 1; v->live.preset = 3; v->live.cpu_pct = 23;
    v->live.fx_on[0] = true; v->live.fx_on[2] = true; v->live.fx_on[5] = true;
    v->meter[0] = 0.72f; v->meter[1] = 0.55f; v->peak[0] = 0.9f; v->peak[1] = 0.7f;
    const char *names[UI_PRESETS] = { "Flat", "Desk", "Late Night", "Movie", "Bass+", "", "", "Gaming", "", "" };
    for (int i = 0; i < UI_PRESETS; i++) {
        strncpy(v->preset_name[i], names[i], 12);
        v->preset_valid[i] = names[i][0] != 0;
    }
    make_eq(&v->eq);
    v->disp_level = 2; v->led_level = 2;
}

static void test_button(void) {
    UiBtn b; ui_btn_init(&b);
    UiBtnEvent e; uint32_t t = 1000;
    int shorts = 0, longs = 0;
    // short press: 120 ms
    for (uint32_t i = 0; i < 400; i += 5) {
        bool down = (i >= 50 && i < 170);
        e = ui_btn_update(&b, down, t + i);
        shorts += e == UI_BTN_SHORT; longs += e == UI_BTN_LONG;
    }
    CHECK(shorts == 1 && longs == 0, "short press yields exactly one SHORT");
    // long press: 900 ms; LONG fires once during hold, none at release
    shorts = longs = 0; t = 5000;
    uint32_t long_at = 0;
    for (uint32_t i = 0; i < 1500; i += 5) {
        bool down = (i >= 50 && i < 950);
        e = ui_btn_update(&b, down, t + i);
        if (e == UI_BTN_LONG && !long_at) long_at = i;
        shorts += e == UI_BTN_SHORT; longs += e == UI_BTN_LONG;
    }
    CHECK(longs == 1 && shorts == 0, "long press yields one LONG and no SHORT");
    CHECK(long_at >= 50 + UI_BTN_LONG_MS && long_at <= 50 + UI_BTN_LONG_MS + 40, "LONG fires at threshold");
    // contact bounce (10 ms glitch) is ignored
    shorts = longs = 0; t = 9000;
    for (uint32_t i = 0; i < 300; i += 5) {
        bool down = (i >= 50 && i < 60);
        e = ui_btn_update(&b, down, t + i);
        shorts += e == UI_BTN_SHORT; longs += e == UI_BTN_LONG;
    }
    CHECK(shorts == 0 && longs == 0, "10 ms glitch ignored");
    // hold progress
    ui_btn_init(&b);
    for (uint32_t i = 0; i < 400; i += 5) ui_btn_update(&b, true, 20000 + i);
    uint8_t pct = ui_btn_hold_pct(&b, 20400);
    CHECK(pct > 40 && pct < 80, "hold progress mid-way");
}

static void test_nav(void) {
    UiNav n = { UI_PG_NOW, false, 0 };
    UiAction a;
    // pages cycle on short
    for (int i = 0; i < UI_PG_COUNT; i++) ui_nav_event(&n, UI_BTN_SHORT, 5, 0);
    CHECK(n.page == UI_PG_NOW, "five shorts return to NOW");
    // long on NOW toggles mute
    a = ui_nav_event(&n, UI_BTN_LONG, 5, 0);
    CHECK(a.type == UA_TOGGLE_MUTE && !n.in_menu, "long on NOW = mute");
    // FX page: enter, move, toggle, back
    ui_nav_event(&n, UI_BTN_SHORT, 5, 0); ui_nav_event(&n, UI_BTN_SHORT, 5, 0);   // -> FX
    CHECK(n.page == UI_PG_FX, "FX reached");
    a = ui_nav_event(&n, UI_BTN_LONG, 5, 0);
    CHECK(n.in_menu && n.cursor == 0 && a.type == UA_NONE, "long enters menu");
    ui_nav_event(&n, UI_BTN_SHORT, 5, 0);
    a = ui_nav_event(&n, UI_BTN_LONG, 5, 0);
    CHECK(a.type == UA_TOGGLE_FX && a.arg == 1 && n.in_menu, "toggle fx 1, stay in menu");
    for (int i = 0; i < UI_MENU_FX_ROWS - 2; i++) ui_nav_event(&n, UI_BTN_SHORT, 5, 0);
    CHECK(n.cursor == UI_MENU_FX_ROWS - 1, "cursor on back");
    a = ui_nav_event(&n, UI_BTN_LONG, 5, 0);
    CHECK(!n.in_menu && a.type == UA_NONE, "long on back leaves menu");
    // wrap in menu
    n.in_menu = true; n.cursor = UI_MENU_FX_ROWS - 1;
    ui_nav_event(&n, UI_BTN_SHORT, 5, 0);
    CHECK(n.cursor == 0, "menu wraps");
    // preset page starts on the active preset
    n = (UiNav){ UI_PG_PRESET, false, 0 };
    ui_nav_event(&n, UI_BTN_LONG, 5, 7);
    CHECK(n.in_menu && n.cursor == 7, "preset menu opens on active preset");
    a = ui_nav_event(&n, UI_BTN_LONG, 5, 7);
    CHECK(a.type == UA_LOAD_PRESET && a.arg == 7, "load preset 7");
    n.cursor = UI_PRESETS;
    a = ui_nav_event(&n, UI_BTN_LONG, 5, 7);
    CHECK(a.type == UA_SAVE_PRESET, "save tile");
    // EQ with no bands: only "back"
    n = (UiNav){ UI_PG_EQ, false, 0 };
    ui_nav_event(&n, UI_BTN_LONG, 0, 0);
    CHECK(n.in_menu && ui_menu_len(UI_PG_EQ, 0) == 1, "EQ menu with no bands has one row");
    a = ui_nav_event(&n, UI_BTN_LONG, 0, 0);
    CHECK(!n.in_menu, "back exits empty EQ menu");
}

static void test_eq(void) {
    UiEq e; memset(&e, 0, sizeof(e));
    e.nb = 1;
    e.band[0] = (UiEqBand){ EQT_PEAKING, false, 1000.0f, 6.0f, 1.0f };
    ui_eq_calc_all(&e, 48000.0f);
    int c = ui_eq_col_for_freq(1000.0f);
    CHECK(abs(e.curve_q[c] - 24) <= 1, "peaking +6 dB at centre reads +6 dB");
    CHECK(abs(e.curve_q[0]) <= 1 && abs(e.curve_q[UI_EQ_COLS - 1]) <= 2, "peaking is flat far from centre");

    e.band[0] = (UiEqBand){ EQT_LOWSHELF, false, 200.0f, -6.0f, 0.7071f };
    ui_eq_calc_all(&e, 48000.0f);
    CHECK(abs(e.curve_q[0] + 24) <= 2, "low shelf -6 dB at 20 Hz");
    CHECK(abs(e.curve_q[UI_EQ_COLS - 1]) <= 1, "low shelf flat at 20 kHz");

    e.band[0] = (UiEqBand){ EQT_HIGHPASS, false, 100.0f, 0.0f, 0.7071f };
    ui_eq_calc_all(&e, 48000.0f);
    CHECK(e.curve_q[0] < -40, "high-pass attenuates 20 Hz");
    int c100 = ui_eq_col_for_freq(100.0f);
    CHECK(abs(e.curve_q[c100] + 12) <= 2, "high-pass is -3 dB at corner");

    e.band[0] = (UiEqBand){ EQT_PEAKING, true, 1000.0f, 6.0f, 1.0f };
    ui_eq_calc_all(&e, 48000.0f);
    CHECK(abs(e.curve_q[c]) <= 1, "bypassed band does nothing");

    // incremental stepping matches the blocking result
    UiEq a, b; make_eq(&a); b = a; memset(b.curve_q, 0, sizeof(b.curve_q)); b.curve_valid = false;
    ui_eq_calc_begin(&b, 48000.0f);
    int steps = 0; while (!ui_eq_calc_step(&b, 8)) steps++;
    CHECK(memcmp(a.curve_q, b.curve_q, sizeof(a.curve_q)) == 0 && steps > 5, "stepped curve equals one-shot curve");
}

int main(int argc, char **argv) {
    const char *dir = argc > 1 ? argv[1] : ".";
    char p[256];
    gfx_init();
    test_button(); test_nav(); test_eq();

    UiView v; make_view(&v);
    snprintf(p, sizeof p, "%s/now.ppm", dir);        v.page = UI_PG_NOW;    render(&v, p);
    v.live.mute = true; snprintf(p, sizeof p, "%s/now_mute.ppm", dir); render(&v, p); v.live.mute = false;
    snprintf(p, sizeof p, "%s/eq.ppm", dir);         v.page = UI_PG_EQ;     render(&v, p);
    v.in_menu = true; v.cursor = 3; snprintf(p, sizeof p, "%s/eq_menu.ppm", dir); render(&v, p);
    v.in_menu = false;
    snprintf(p, sizeof p, "%s/fx.ppm", dir);         v.page = UI_PG_FX;     render(&v, p);
    v.in_menu = true; v.cursor = 2; snprintf(p, sizeof p, "%s/fx_menu.ppm", dir); render(&v, p);
    v.in_menu = false;
    snprintf(p, sizeof p, "%s/preset.ppm", dir);     v.page = UI_PG_PRESET; render(&v, p);
    v.in_menu = true; v.cursor = 2; snprintf(p, sizeof p, "%s/preset_menu.ppm", dir); render(&v, p);
    v.in_menu = false;
    v.live.clip = true;
    snprintf(p, sizeof p, "%s/sys.ppm", dir);        v.page = UI_PG_SYS;    render(&v, p);
    v.in_menu = true; v.cursor = 1; v.hold_pct = 55;
    snprintf(p, sizeof p, "%s/sys_menu_hold.ppm", dir); render(&v, p);
    v.hold_pct = 0; v.toast_on = true; strcpy(v.toast, "saved p3");
    snprintf(p, sizeof p, "%s/toast.ppm", dir);      render(&v, p);

    // font sheet
    gfx_set_clip(0, UI_DISP_H); gfx_clear_rows(0, UI_DISP_H, 0);
    gfx_text(1, 1,  "abcdefghijklm", GFX_RGB(255,255,255), 1);
    gfx_text(1, 11, "nopqrstuvwxyz", GFX_RGB(255,255,255), 1);
    gfx_text(1, 21, "ABCDEFGHIJKLM", GFX_RGB(255,255,255), 1);
    gfx_text(1, 31, "NOPQRSTUVWXYZ", GFX_RGB(255,255,255), 1);
    gfx_text(1, 41, "0123456789+-.%", GFX_RGB(255,255,255), 1);
    gfx_text(1, 51, "<>:/ k db 12k", GFX_RGB(255,255,255), 1);
    snprintf(p, sizeof p, "%s/font.ppm", dir); dump(p, 6);

    printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
