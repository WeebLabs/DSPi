/*
 * ui.c; see ui.h
 */
#include "ui.h"
#include "ui_board.h"
#include "ui_types.h"
#include "ui_button.h"
#include "ui_nav.h"
#include "ui_eq.h"
#include "ui_pages.h"
#include "ui_theme.h"
#include "ui_hw.h"
#include "st7735.h"
#include "rgb_led.h"
#include "gfx.h"

#include "config.h"
#include "control_surfaces.h"
#include "flash_storage.h"
#include "input_source_names.h"
#include "usb_audio.h"
#include "bg_job.h"
#include "pico/time.h"
#include <string.h>

// ---- the EQ module's constants must track DSPi's FilterType -----------------
_Static_assert(EQT_FLAT == FILTER_FLAT && EQT_PEAKING == FILTER_PEAKING &&
               EQT_LOWSHELF == FILTER_LOWSHELF && EQT_HIGHSHELF == FILTER_HIGHSHELF &&
               EQT_LOWPASS == FILTER_LOWPASS && EQT_HIGHPASS == FILTER_HIGHPASS &&
               EQT_NOTCH == FILTER_NOTCH && EQT_ALLPASS == FILTER_ALLPASS &&
               EQT_ALLPASS1 == FILTER_ALLPASS1 && EQT_LOWSHELF1 == FILTER_LOWSHELF1 &&
               EQT_HIGHSHELF1 == FILTER_HIGHSHELF1 && EQT_LINKWITZ == FILTER_LINKWITZ_TRANSFORM &&
               EQT_LOWPASS1 == FILTER_LOWPASS1 && EQT_HIGHPASS1 == FILTER_HIGHPASS1,
               "ui_eq.h EQT_* out of step with enum FilterType");
_Static_assert(UI_PRESETS == PRESET_SLOTS, "UI preset count must match PRESET_SLOTS");
_Static_assert(UI_EQ_MAX_BANDS <= XOVER_BAND_BASE, "UI EQ bands must be PEQ bands");

#define UI_FRAME_MS        40u       // 25 fps while awake
#define UI_LIVE_MS         20u
#define UI_EQ_POLL_MS      250u
#define UI_NAME_POLL_MS    500u
#define UI_TOAST_MS        1200u
#define UI_DIM_MS          30000u
#define UI_SLEEP_MS        120000u   // display off after this, only while nothing streams
#define UI_ACTION_TRIES    100       // main-loop passes a deferred dispatch may retry
#define UI_EQ_CH           0         // master EQ, left input channel

typedef enum { PH_IDLE, PH_EQ, PH_RENDER, PH_FLUSH } Phase;

static struct {
    UiView   view;          // live model, updated between frames
    UiView   snap;          // frozen copy the render job draws from
    UiEq     eq_work;       // curve under construction
    UiNav    nav;
    UiBtn    btn;
    BgJob    job;
    Phase    phase;
    uint8_t  band;
    uint32_t eq_unit_us;
    bool     eq_dirty;
    bool     started;
    uint8_t  last_rate;
    uint32_t t_frame, t_live, t_eq, t_name, t_toast, t_active, t_peak[2];
    uint8_t  name_cursor;
    // single pending action with retry
    UiAction pend;
    int      pend_tries;
    bool     asleep, dimmed;
} S;

bool ui_owns_pin(uint8_t gpio) { return gpio < 32 && ((UI_OWNED_PIN_MASK >> gpio) & 1u); }

static uint32_t now_ms(void) { return (uint32_t)(time_us_64() / 1000u); }

static float noun(uint8_t n, uint8_t t, uint8_t i) { return cs_noun_get(n, t, i); }

static void toast(const char *s) {
    strncpy(S.view.toast, s, sizeof S.view.toast - 1);
    S.view.toast[sizeof S.view.toast - 1] = 0;
    S.view.toast_on = true;
    S.t_toast = now_ms();
}

static void toast2(const char *a, const char *b) {
    char buf[20];
    size_t la = strlen(a);
    if (la > 12) la = 12;
    memcpy(buf, a, la);
    buf[la++] = ' ';
    strncpy(buf + la, b, sizeof buf - la - 1);
    buf[sizeof buf - 1] = 0;
    toast(buf);
}

static const uint8_t fx_noun[UI_FX_COUNT] = {
    CS_NOUN_LOUDNESS, CS_NOUN_CROSSFEED, CS_NOUN_LEVELLER, CS_NOUN_TUBE, CS_NOUN_PSYBASS,
    CS_NOUN_EQ_BYPASS,       // "eq" tile is on when the master EQ is NOT bypassed
};

// ---------------------------------------------------------------- model ----

static void refresh_live(void) {
    UiView *v = &S.view;
    UiLive *l = &v->live;
    l->vol_lo_db = -(float)CENTER_VOLUME_INDEX;
    l->vol_hi_db = 0.0f;
    l->vol_db    = noun(CS_NOUN_USER_VOLUME, 0, 0);
    l->mute      = noun(CS_NOUN_USER_MUTE, 0, 0) >= 0.5f;
    l->streaming = noun(CS_NOUN_USB_STREAMING, 0, 0) >= 0.5f;
    l->clip      = noun(CS_NOUN_CLIP, 0, 0) >= 0.5f;
    int r = (int)noun(CS_NOUN_SAMPLE_RATE, 0, 0);
    l->rate_idx  = (uint8_t)(r < 0 ? 0 : r > 2 ? 2 : r);
    int p = (int)noun(CS_NOUN_PRESET, 0, 0);
    l->preset    = (uint8_t)(p < 0 ? 0 : p >= UI_PRESETS ? UI_PRESETS - 1 : p);
    // Input source: the user's name for it when set, else the built-in one.
    static const char *const src_names[INPUT_SOURCE_MAX + 1] = { "usb", "spdif", "i2s", "adat",
                                             "spdif 2", "spdif 3", "spdif 4" };
    int src = (int)noun(CS_NOUN_INPUT_SOURCE, 0, 0);
    if (src < 0 || src > INPUT_SOURCE_MAX) src = INPUT_SOURCE_USB;
    char nm[INPUT_SOURCE_NAME_LEN];
    if (input_source_name_get((uint8_t)src, nm) && nm[0]) {
        size_t n = 0;
        while (n < sizeof l->src - 1 && nm[n]) n++;
        memcpy(l->src, nm, n);
        l->src[n] = 0;
    } else {
        strncpy(l->src, src_names[src], sizeof l->src - 1);
        l->src[sizeof l->src - 1] = 0;
    }
    l->src_active = (src == INPUT_SOURCE_USB) ? l->streaming
                  : (src == INPUT_SOURCE_I2S) ? true
                  : noun(CS_NOUN_SPDIF_LOCK, 0, 0) >= 0.5f;
    float cpu = noun(CS_NOUN_CPU_LOAD, 0, 0);
    l->cpu_pct   = (uint8_t)(cpu < 0 ? 0 : cpu > 100 ? 100 : cpu);
    for (int i = 0; i < UI_FX_COUNT; i++) {
        bool b = noun(fx_noun[i], 0, 0) >= 0.5f;
        l->fx_on[i] = (i == UI_FX_COUNT - 1) ? !b : b;
    }

    // meters: instant attack, linear fall, 0.8 s peak hold
    uint32_t t = now_ms();
    for (int c = 0; c < 2; c++) {
        l->lvl_db[c] = l->streaming ? noun(CS_NOUN_LEVEL, (uint8_t)(CH_OUT_1 + c), 0) : -60.0f;
        float pos = (l->lvl_db[c] + 60.0f) * (1.0f / 60.0f);
        pos = pos < 0 ? 0 : pos > 1 ? 1 : pos;
        v->meter[c] = pos > v->meter[c] ? pos : (v->meter[c] > 0.04f ? v->meter[c] - 0.04f : 0.0f);
        if (pos >= v->peak[c]) { v->peak[c] = pos; S.t_peak[c] = t; }
        else if ((int32_t)(t - S.t_peak[c]) > 800) {
            v->peak[c] = v->peak[c] > 0.03f ? v->peak[c] - 0.03f : 0.0f;
        }
    }
}

static bool band_differs(const UiEqBand *a, const UiEqBand *b) {
    return a->type != b->type || a->bypass != b->bypass ||
           a->freq != b->freq || a->gain_db != b->gain_db || a->q != b->q;
}

static void refresh_eq(void) {
    UiEq *e = &S.view.eq;
    UiEq nw;
    memset(&nw, 0, sizeof nw);
    nw.ch = UI_EQ_CH;
    for (int b = 0; b < UI_EQ_MAX_BANDS; b++) {
        uint8_t type = (uint8_t)noun(CS_NOUN_FILTER_TYPE, UI_EQ_CH, (uint8_t)b);
        if (type == FILTER_FLAT) continue;
        UiEqBand *d = &nw.band[nw.nb];
        d->type    = type;
        d->bypass  = noun(CS_NOUN_FILTER_BYPASS, UI_EQ_CH, (uint8_t)b) >= 0.5f;
        d->freq    = noun(CS_NOUN_FILTER_FREQ, UI_EQ_CH, (uint8_t)b);
        d->gain_db = noun(CS_NOUN_FILTER_GAIN, UI_EQ_CH, (uint8_t)b);
        d->q       = noun(CS_NOUN_FILTER_Q, UI_EQ_CH, (uint8_t)b);
        nw.slot[nw.nb] = (uint8_t)b;
        nw.nb++;
    }
    bool changed = nw.nb != e->nb || !e->curve_valid;
    for (int i = 0; !changed && i < nw.nb; i++)
        changed = nw.slot[i] != e->slot[i] || band_differs(&nw.band[i], &e->band[i]);
    if (!changed) return;
    // the old curve stays in e->curve_q until the new one is ready
    e->nb = nw.nb;
    e->ch = nw.ch;
    memcpy(e->slot, nw.slot, sizeof e->slot);
    memcpy(e->band, nw.band, sizeof e->band);
    S.eq_dirty = true;
    ui_nav_clamp(&S.nav, e->nb);
}

static void refresh_names(void) {
    uint8_t s = S.name_cursor;
    S.name_cursor = (uint8_t)((s + 1) % UI_PRESETS);
    char raw[PRESET_NAME_LEN];
    memset(raw, 0, sizeof raw);
    uint8_t rc = preset_get_name(s, raw);
    S.view.preset_valid[s] = (rc == PRESET_OK);
    char *d = S.view.preset_name[s];
    if (rc != PRESET_OK || raw[0] == 0) {
        d[0] = 'p'; d[1] = (char)('0' + s); d[2] = 0;     // unnamed slot shows as its number
        return;
    }
    size_t n = 0;
    while (n < 12 && n < PRESET_NAME_LEN && raw[n]) n++;
    memcpy(d, raw, n);
    d[n] = 0;
}

// -------------------------------------------------------------- actions ----

static bool run_action(const UiAction *a) {
    const UiView *v = &S.view;
    switch (a->type) {
    case UA_TOGGLE_MUTE:
        return cs_noun_dispatch(CS_NOUN_USER_MUTE, 0, 0, v->live.mute ? 0.0f : 1.0f);
    case UA_TOGGLE_FX: {
        uint8_t i = a->arg;
        if (i >= UI_FX_COUNT) return true;
        // fx_on[] is "enabled"; EQ_BYPASS is the inverse, so writing !enabled-as-bypass flips both
        float val = (i == UI_FX_COUNT - 1) ? (v->live.fx_on[i] ? 1.0f : 0.0f)
                                           : (v->live.fx_on[i] ? 0.0f : 1.0f);
        return cs_noun_dispatch(fx_noun[i], 0, 0, val);
    }
    case UA_TOGGLE_BAND: {
        if (a->arg >= v->eq.nb) return true;
        const UiEqBand *b = &v->eq.band[a->arg];
        return cs_noun_dispatch(CS_NOUN_FILTER_BYPASS, UI_EQ_CH, v->eq.slot[a->arg], b->bypass ? 0.0f : 1.0f);
    }
    case UA_LOAD_PRESET:
        return cs_noun_dispatch(CS_NOUN_PRESET, 0, 0, (float)a->arg);
    case UA_SAVE_PRESET:
        return cs_noun_dispatch(CS_NOUN_PRESET_SAVE, 0, 0, 1.0f);
    case UA_CLEAR_CLIP:
        return cs_noun_dispatch(CS_NOUN_CLIP, 0, 0, 1.0f);
    default:
        return true;
    }
}

static void announce(const UiAction *a) {
    const UiView *v = &S.view;
    char num[4] = { 0 };
    switch (a->type) {
    case UA_TOGGLE_MUTE: toast(v->live.mute ? "unmuted" : "muted"); break;
    case UA_TOGGLE_FX:
        if (a->arg < UI_FX_COUNT) toast2(ui_fx_names[a->arg], v->live.fx_on[a->arg] ? "off" : "on");
        break;
    case UA_TOGGLE_BAND:
        if (a->arg < v->eq.nb) { num[0] = (char)('1' + v->eq.slot[a->arg]); if (v->eq.slot[a->arg] == 9) { num[0] = '1'; num[1] = '0'; }
            toast2(num, v->eq.band[a->arg].bypass ? "on" : "off"); }
        break;
    case UA_LOAD_PRESET: num[0] = 'p'; num[1] = (char)('0' + a->arg); toast2("load", num); break;
    case UA_SAVE_PRESET: num[0] = 'p'; num[1] = (char)('0' + v->live.preset); toast2("saved", num); break;
    case UA_CLEAR_CLIP:  toast("clip cleared"); break;
    default: break;
    }
}

static void set_disp_level(uint8_t lv) {
    S.view.disp_level = lv & 3;
    gfx_set_level(S.view.disp_level);   // backlight not driven: dim the picture
}

static void handle_action(UiAction a) {
    switch (a.type) {
    case UA_NONE: return;
    case UA_CYCLE_DISP:
        set_disp_level((uint8_t)(S.view.disp_level + 1));
        toast("brightness");
        return;
    case UA_CYCLE_LED:
        S.view.led_level = (uint8_t)((S.view.led_level + 1) & 3);
        toast("led level");
        return;
    default:
        break;
    }
    announce(&a);
    if (S.pend.type != UA_NONE) run_action(&S.pend);   // flush a still-queued one rather than drop it
    S.pend = a;
    S.pend_tries = UI_ACTION_TRIES;
    if (a.type == UA_LOAD_PRESET) S.eq_dirty = true;
}

static void pump_pending(void) {
    if (S.pend.type == UA_NONE) return;
    if (run_action(&S.pend) || --S.pend_tries <= 0) S.pend.type = UA_NONE;
}

// ------------------------------------------------------------------ LED ----

static void update_led(uint32_t t) {
    static const uint8_t scale[4] = { 0, 10, 40, 110 };
    uint8_t k = scale[S.view.led_level & 3];
    const UiLive *l = &S.view.live;
    uint8_t r = 0, g = 0, b = 0;
    if (S.asleep || k == 0)            { r = g = b = 0; }
    else if (l->clip)                  { r = 255; }
    else if (l->mute)                  { r = 255; g = 70; }
    else if (S.view.hold_pct)          { r = g = b = 200; }
    else if (S.nav.in_menu) {
        uint16_t c = ui_cycle_color(S.nav.page);
        r = (uint8_t)(((c >> 11) & 31) << 3); g = (uint8_t)(((c >> 5) & 63) << 2); b = (uint8_t)((c & 31) << 3);
    }
    else if (l->streaming)             { g = 255; }
    else                               { b = (uint8_t)(90 + 40 * (((t / 700) & 1) ? 1 : 0)); }   // idle: slow blue blink
    rgb_led_set((uint8_t)(r * k / 255), (uint8_t)(g * k / 255), (uint8_t)(b * k / 255));
}

// ------------------------------------------------------------ bg render ----

static bool ui_step(void *ctx, uint32_t deadline_us) {
    (void)ctx;
    if (S.phase == PH_EQ) {
        for (;;) {
            uint32_t t0 = time_us_32();
            bool done = ui_eq_calc_step(&S.eq_work, 2);
            S.eq_unit_us = time_us_32() - t0;
            if (done) {
                memcpy(S.view.eq.curve_q, S.eq_work.curve_q, sizeof S.view.eq.curve_q);
                S.view.eq.curve_valid = true;
                S.phase = PH_RENDER;
                S.band = 0;
                S.snap = S.view;                 // freeze the view for the bands
                return false;
            }
            if (bg_out_of_time(deadline_us, S.eq_unit_us)) return false;
        }
    }
    ui_render_band(&S.snap, S.band);
    if (++S.band >= UI_BANDS) { S.phase = PH_FLUSH; return true; }
    return false;
}

static const uint16_t rate_hz_tab[3] = { 44100, 48000, 0 };   // 96k handled below

static void frame_begin(uint32_t t) {
    S.view.now_ms = t;
    S.view.page = S.nav.page;
    S.view.in_menu = S.nav.in_menu;
    S.view.cursor = S.nav.cursor;
    S.view.hold_pct = ui_btn_hold_pct(&S.btn, t);
    if (S.view.toast_on && (int32_t)(t - S.t_toast) > (int32_t)UI_TOAST_MS) S.view.toast_on = false;

    bool need_eq = S.view.page == UI_PG_EQ && (S.eq_dirty || S.last_rate != S.view.live.rate_idx);
    if (need_eq) {
        S.eq_work = S.view.eq;
        float fs = S.view.live.rate_idx == 2 ? 96000.0f : (float)rate_hz_tab[S.view.live.rate_idx];
        ui_eq_calc_begin(&S.eq_work, fs);
        S.eq_dirty = false;
        S.last_rate = S.view.live.rate_idx;
        S.phase = PH_EQ;
    } else {
        S.snap = S.view;
        S.band = 0;
        S.phase = PH_RENDER;
    }
    bg_job_start(&S.job);
}

// --------------------------------------------------------------- public ----

void ui_init(void) {
    memset(&S, 0, sizeof S);
    S.view.disp_level = 3;              // full; the TFT dims by scaling colours
    S.view.led_level  = 2;
    S.job.id = BG_JOB_UI;
    S.job.step = ui_step;
    bg_job_register(&S.job);
    gfx_init();
    ui_hw_init();
    rgb_led_init();
    ui_btn_init(&S.btn);
    st7735_begin();
    for (int i = 0; i < UI_PRESETS; i++) refresh_names();
    refresh_live();
    refresh_eq();
    S.t_active = now_ms();
    S.started = true;
}

void ui_service(void) {
    if (!S.started) return;
    uint32_t t = now_ms();
    bool ready = st7735_service();

    // --- input -------------------------------------------------------------
    UiBtnEvent ev = ui_btn_update(&S.btn, ui_hw_button_pressed(t), t);
    ui_hw_user_led(ui_btn_is_down(&S.btn));
    if (ev != UI_BTN_NONE) {
        bool woke = S.asleep || S.dimmed;
        S.t_active = t;
        if (S.asleep) { st7735_power(true); S.asleep = false; }
        if (S.dimmed) { set_disp_level(S.view.disp_level); S.dimmed = false; }
        if (!woke) handle_action(ui_nav_event(&S.nav, ev, S.view.eq.nb, S.view.live.preset));
    }
    pump_pending();

    // --- model -------------------------------------------------------------
    if ((int32_t)(t - S.t_live) >= (int32_t)UI_LIVE_MS) { S.t_live = t; refresh_live(); }
    if ((int32_t)(t - S.t_eq) >= (int32_t)UI_EQ_POLL_MS) { S.t_eq = t; refresh_eq(); }
    if ((int32_t)(t - S.t_name) >= (int32_t)UI_NAME_POLL_MS) { S.t_name = t; refresh_names(); }

    // --- idle policy -------------------------------------------------------
    if (S.view.live.streaming || S.view.live.clip) { if (S.asleep) { st7735_power(true); S.asleep = false; } }
    uint32_t idle = t - S.t_active;
    if (!S.asleep && idle > UI_SLEEP_MS && !S.view.live.streaming) {
        st7735_power(false); S.asleep = true;   // black frame
    } else if (!S.dimmed && !S.asleep && idle > UI_DIM_MS) {
        gfx_set_level(0); S.dimmed = true;
    }
    update_led(t);

    // --- frame pipeline ----------------------------------------------------
    if (!ready || S.asleep) return;
    switch (S.phase) {
    case PH_IDLE:
        // Never draw into the framebuffer while DMA is still sending it.
        if ((int32_t)(t - S.t_frame) >= (int32_t)UI_FRAME_MS && !st7735_busy()) {
            S.t_frame = t;
            frame_begin(t);
        }
        break;
    case PH_FLUSH:
        if (st7735_flush(gfx_fb())) S.phase = PH_IDLE;
        break;
    default:
        break;   // job still working
    }
}
