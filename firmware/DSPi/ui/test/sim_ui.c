// Host integration test for ui.c: fake DSPi (nouns), fake display/LED/button, scripted presses.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "../ui.h"
#include "../ui_hw.h"
#include "../ssd1331.h"
#include "../rgb_led.h"
#include "../gfx.h"
#include "../ui_board.h"
#include "../ui_types.h"
#include "control_surfaces.h"
#include "flash_storage.h"
#include "bg_job.h"
#include "config.h"

uint64_t sim_us;

// ---- fake DSPi ------------------------------------------------------------
static struct {
    float vol; bool mute, streaming, clip, fx[5], eq_bypass; int rate, preset; float cpu, lvl[2];
    struct { int type; bool byp; float f, g, q; } band[10];
} D = { .vol = -23, .streaming = true, .rate = 1, .preset = 3, .cpu = 23, .lvl = { -12, -9 } };

typedef struct { int noun, tgt, idx; float val; } Call;
static Call calls[64]; static int ncalls; static int busy_left;

float cs_noun_get(uint8_t n, uint8_t t, uint8_t i) {
    switch (n) {
    case CS_NOUN_USER_VOLUME: return D.vol;
    case CS_NOUN_USER_MUTE: return D.mute;
    case CS_NOUN_LOUDNESS: return D.fx[0];
    case CS_NOUN_CROSSFEED: return D.fx[1];
    case CS_NOUN_LEVELLER: return D.fx[2];
    case CS_NOUN_TUBE: return D.fx[3];
    case CS_NOUN_PSYBASS: return D.fx[4];
    case CS_NOUN_EQ_BYPASS: return D.eq_bypass;
    case CS_NOUN_PRESET: return D.preset;
    case CS_NOUN_CLIP: return D.clip;
    case CS_NOUN_SAMPLE_RATE: return D.rate;
    case CS_NOUN_USB_STREAMING: return D.streaming;
    case CS_NOUN_CPU_LOAD: return D.cpu;
    case CS_NOUN_LEVEL: return D.lvl[t - CH_OUT_1];
    case CS_NOUN_FILTER_TYPE: return D.band[i].type;
    case CS_NOUN_FILTER_BYPASS: return D.band[i].byp;
    case CS_NOUN_FILTER_FREQ: return D.band[i].f;
    case CS_NOUN_FILTER_GAIN: return D.band[i].g;
    case CS_NOUN_FILTER_Q: return D.band[i].q;
    }
    return 0;
}
bool cs_noun_dispatch(uint8_t n, uint8_t t, uint8_t i, float v) {
    if (busy_left > 0) { busy_left--; return false; }
    if (ncalls < 64) calls[ncalls++] = (Call){ n, t, i, v };
    // apply like the real device would
    switch (n) {
    case CS_NOUN_USER_MUTE: D.mute = v >= .5f; break;
    case CS_NOUN_EQ_BYPASS: D.eq_bypass = v >= .5f; break;
    case CS_NOUN_LOUDNESS: D.fx[0] = v >= .5f; break;
    case CS_NOUN_PRESET: D.preset = (int)v; break;
    case CS_NOUN_FILTER_BYPASS: D.band[i].byp = v >= .5f; break;
    case CS_NOUN_CLIP: D.clip = false; break;
    }
    return true;
}
uint8_t preset_get_name(uint8_t s, char *o) {
    static const char *nm[10] = { "flat", "music", "late night", "movie", "", "", "", "", "", "" };
    if (!nm[s][0]) return 1;
    strcpy(o, nm[s]); return PRESET_OK;
}

// ---- fake bg job runner ---------------------------------------------------
static BgJob *job;
void bg_job_register(BgJob *j) { job = j; }
void bg_run_one(void) { if (job && job->active && job->step(job->ctx, (uint32_t)sim_us + 100)) job->active = false; }

// ---- fake hardware --------------------------------------------------------
static bool btn_down; static int disp_level_calls, last_level = -1, power_on_calls, power_off_calls;
static int flushes; static const char *dump_name; static bool disp_ready_after_ms_done;
static uint8_t led_r, led_g, led_b;
void ui_hw_init(void) {}
bool ui_hw_button_pressed(uint32_t t) { (void)t; return btn_down; }
void ui_hw_user_led(bool on) { (void)on; }
void rgb_led_init(void) {}
void rgb_led_set(uint8_t r, uint8_t g, uint8_t b) { led_r = r; led_g = g; led_b = b; }
void ssd1331_begin(void) {}
bool ssd1331_service(void) { return sim_us > 150000; }
bool ssd1331_ready(void) { return sim_us > 150000; }
bool ssd1331_busy(void) { return false; }
static void dump(const char *name) {
    char p[96]; snprintf(p, sizeof p, "out_ui/%s.ppm", name);
    FILE *f = fopen(p, "wb"); fprintf(f, "P6\n%d %d\n255\n", UI_DISP_W, UI_DISP_H);
    const uint16_t *fb = gfx_fb();
    for (int i = 0; i < UI_DISP_W * UI_DISP_H; i++) {
        uint16_t c = (uint16_t)((fb[i] >> 8) | (fb[i] << 8));     // stored byte-swapped
        fputc(((c >> 11) & 31) << 3, f); fputc(((c >> 5) & 63) << 2, f); fputc((c & 31) << 3, f);
    }
    fclose(f);
}
bool ssd1331_flush(const uint16_t *fb) { (void)fb; flushes++; if (dump_name) { dump(dump_name); dump_name = NULL; } return true; }
void ssd1331_set_level(uint8_t l) { last_level = l; disp_level_calls++; }
void ssd1331_power(bool on) { if (on) power_on_calls++; else power_off_calls++; }

// ---- driver ---------------------------------------------------------------
static int fails;
#define SETB(i,t,b,f_,g_,q_) do { D.band[i].type=(t); D.band[i].byp=(b); D.band[i].f=(f_); D.band[i].g=(g_); D.band[i].q=(q_); } while (0)
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void run_ms(int ms) {
    for (int i = 0; i < ms; i++) { sim_us += 1000; ui_service(); for (int k = 0; k < 3; k++) bg_run_one(); }
}
static void press(int hold_ms) { btn_down = true; run_ms(hold_ms); btn_down = false; run_ms(80); }
static void shortp(void) { press(90); }
static void longp(void)  { press(700); }
static void snap(const char *n) { dump_name = n; run_ms(250); CHECK(!dump_name, "no frame flushed for %s", n); }

int main(void) {
    if (system("mkdir -p out_ui")) return 2;
    SETB(0, FILTER_HIGHPASS, false, 60, 0, 0.7f);
    SETB(1, FILTER_PEAKING, false, 250, -4, 1.2f);
    SETB(2, FILTER_PEAKING, false, 3000, 3, 2.0f);
    SETB(3, FILTER_HIGHSHELF, false, 9000, 2, 0.7f);

    ui_init();
    run_ms(500);
    snap("t_now");
    CHECK(led_g > 0 && led_r == 0, "streaming LED should be green, got %u/%u/%u", led_r, led_g, led_b);
    CHECK(ncalls == 0, "no dispatch expected yet");

    // NOW: long press toggles mute
    longp();
    CHECK(ncalls == 1 && calls[0].noun == CS_NOUN_USER_MUTE && calls[0].val == 1.0f, "mute dispatch");
    CHECK(led_r > 0 && led_g > 0 && led_b == 0, "muted LED is orange");
    snap("t_muted");
    longp();
    CHECK(ncalls == 2 && calls[1].val == 0.0f, "unmute dispatch");
    ncalls = 0;

    // short -> EQ page; open menu; step to band 2 (index 1); long toggles its bypass (slot 1)
    shortp(); run_ms(300); snap("t_eq");
    longp();                       // open EQ menu (cursor 0)
    shortp();                      // cursor -> band index 1
    longp();                       // toggle that band
    CHECK(ncalls == 1 && calls[0].noun == CS_NOUN_FILTER_BYPASS && calls[0].idx == 1 && calls[0].val == 1.0f,
          "band bypass: noun=%d idx=%d val=%g (n=%d)", calls[0].noun, calls[0].idx, calls[0].val, ncalls);
    run_ms(300); snap("t_eq_menu");
    ncalls = 0;

    // leave the menu: step to back row (nb=4 -> rows 0..4, back = 4) and long-press
    for (int i = 0; i < 3; i++) shortp();
    longp();
    CHECK(ncalls == 0, "back row must not dispatch");

    // FX page: toggle loudness (row 0)
    shortp();                      // -> FX
    longp();                       // open FX menu
    longp();                       // toggle row 0 (loudness)
    CHECK(ncalls == 1 && calls[0].noun == CS_NOUN_LOUDNESS && calls[0].val == 1.0f, "loudness toggle");
    run_ms(300); snap("t_fx");
    ncalls = 0;

    // busy retry: the next toggle gets BUSY twice before succeeding, and must land exactly once
    busy_left = 2;
    for (int i = 0; i < 5; i++) shortp();      // FX menu cursor 0 -> 5 (the eq row)
    longp();
    run_ms(100);
    CHECK(ncalls == 1 && calls[0].noun == CS_NOUN_EQ_BYPASS, "eq toggle after busy retries: n=%d noun=%d", ncalls, calls[0].noun);
    ncalls = 0;
    shortp(); longp();                         // back row -> leave the FX menu
    CHECK(ncalls == 0, "back row must not dispatch");

    // PRESET page: menu opens on the active preset (3); step to 4 and load it
    shortp();                                  // FX -> PRESET
    longp();                                   // open menu
    shortp();                                  // 3 -> 4
    longp();                                   // load slot 4
    CHECK(ncalls == 1 && calls[0].noun == CS_NOUN_PRESET && calls[0].val == 4.0f, "preset load n=%d val=%g", ncalls, calls[0].val);
    run_ms(300); snap("t_preset");
    ncalls = 0;
    for (int i = 0; i < 6; i++) shortp();      // 4 -> 10 (the save row)
    longp();
    CHECK(ncalls == 1 && calls[0].noun == CS_NOUN_PRESET_SAVE, "preset save n=%d noun=%d", ncalls, calls[0].noun);
    run_ms(100); snap("t_saved");
    ncalls = 0;
    shortp(); longp();                         // back row -> leave menu
    CHECK(ncalls == 0, "back row must not dispatch");
    D.preset = 3;

    // Idle policy: dim after 30 s, off after 120 s without streaming
    disp_level_calls = 0; power_off_calls = 0;
    run_ms(31000);
    CHECK(last_level == 0, "display should dim after 30 s (last_level=%d)", last_level);
    D.streaming = false;
    run_ms(100000);
    CHECK(power_off_calls == 1, "display should power off after 120 s idle without streaming (%d)", power_off_calls);
    int c0 = ncalls;
    shortp();                      // wake press must be swallowed
    CHECK(power_on_calls >= 1, "press should wake the display");
    CHECK(ncalls == c0, "wake press must not trigger an action");

    // clip -> red LED
    D.clip = true; run_ms(200);
    CHECK(led_r > 0 && led_g == 0, "clip LED red");
    D.clip = false;

    printf(fails ? "\n%d check(s) FAILED\n" : "\nall ui integration checks passed\n", fails);
    return fails ? 1 : 0;
}
