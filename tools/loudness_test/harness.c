// Checks on-demand loudness.c against the old table build and drives its update state machine.
#include <stdio.h>
#include <string.h>
#include "loudness.h"
#include "bg_job.h"

extern LoudnessCoeffs (*ref_loudness_active_table)[LOUDNESS_BIQUAD_COUNT];
void ref_loudness_recompute_table(float ref_spl, float intensity_pct, float sample_rate);

static BgJob *job;
void bg_job_register(BgJob *j) { job = j; }

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static bool same(const LoudnessCoeffs *a, const LoudnessCoeffs *b) {
    for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
#if PICO_RP2350
        float x[6] = { a[j].sva1, a[j].sva2, a[j].sva3, a[j].svm0, a[j].svm1, a[j].svm2 };
        float y[6] = { b[j].sva1, b[j].sva2, b[j].sva3, b[j].svm0, b[j].svm1, b[j].svm2 };
#else
        int32_t x[5] = { a[j].b0, a[j].b1, a[j].b2, a[j].a1, a[j].a2 };
        int32_t y[5] = { b[j].b0, b[j].b1, b[j].b2, b[j].a1, b[j].a2 };
#endif
        if (memcmp(x, y, sizeof x) || a[j].bypass != b[j].bypass) return false;
    }
    return true;
}

// Reference pair for one state, from the old full-table build.
static LoudnessCoeffs ref_pair[LOUDNESS_BIQUAD_COUNT];
static const LoudnessCoeffs *ref(int idx, float spl, float inten, float rate) {
    ref_loudness_recompute_table(spl, inten, rate);
    memcpy(ref_pair, ref_loudness_active_table[idx], sizeof ref_pair);
    return ref_pair;
}

// Firmware state and main loop, as main.c / usb_audio.c drive loudness.c.
static struct { bool en; uint8_t idx; float spl, inten, rate; bool pending; } fw;
static int jobs_run;
static void set_vol(uint8_t i) { fw.idx = i; if (fw.en) fw.pending = true; }
static void set_en(bool e)     { fw.en = e; fw.pending = true; }
static void set_spl(float v)   { fw.spl = v; fw.pending = true; }
static void set_int(float v)   { fw.inten = v; fw.pending = true; }
static void set_rate(float v)  { fw.rate = v; fw.pending = true; }
static void main_pass(void) {
    if (fw.pending) {
        fw.pending = false;
        loudness_update_begin(fw.en, fw.idx, fw.spl, fw.inten, fw.rate);
    }
    if (job && job->active) {
        const LoudnessCoeffs *before = current_loudness_coeffs;
        if (job->step(job->ctx, 0)) job->active = false;
        jobs_run++;
        CHECK(before == NULL || current_loudness_coeffs != before, "published buffer was rewritten in place");
    }
}
static void settle_and_check(const char *what) {
    main_pass(); main_pass();
    CHECK(!fw.pending && !(job && job->active), "%s: did not settle", what);
    if (!fw.en) { CHECK(current_loudness_coeffs == NULL, "%s: not NULL while disabled", what); return; }
    CHECK(current_loudness_coeffs && same(current_loudness_coeffs, ref(fw.idx, fw.spl, fw.inten, fw.rate)),
          "%s: idx %d spl %g int %g rate %g differs from table", what, fw.idx, fw.spl, fw.inten, fw.rate);
}

int main(void) {
    static const float spls[] = { 40, 60, 87, 100 }, ints[] = { 0, 50, 100, 200 }, rates[] = { 44100, 48000, 96000 };
    int eq = 0, total = 0;
    for (unsigned s = 0; s < 4; s++) for (unsigned i = 0; i < 4; i++) for (unsigned r = 0; r < 3; r++) {
        ref_loudness_recompute_table(spls[s], ints[i], rates[r]);
        for (int v = 0; v < LOUDNESS_VOL_STEPS; v++) {
            loudness_update_now(true, (uint8_t)v, spls[s], ints[i], rates[r]);
            bool ok = current_loudness_coeffs && same(current_loudness_coeffs, ref_loudness_active_table[v]);
            CHECK(ok, "now: idx %d spl %g int %g rate %g", v, spls[s], ints[i], rates[r]);
            eq += ok; total++;
        }
    }
    printf("bit-exact: %d/%d (vol x spl x intensity x rate)\n", eq, total);

    fw.en = true; fw.idx = 30; fw.spl = 87; fw.inten = 100; fw.rate = 48000;
    loudness_update_now(fw.en, fw.idx, fw.spl, fw.inten, fw.rate);
    CHECK(same(current_loudness_coeffs, ref(30, 87, 100, 48000)), "boot publish");

    set_vol(10); settle_and_check("volume change");
    set_vol(20); main_pass(); set_vol(25); settle_and_check("volume change while pending");
    jobs_run = 0;
    for (int v = 0; v <= 60; v++) set_vol((uint8_t)v);
    set_vol(5); settle_and_check("burst");
    CHECK(jobs_run == 1, "burst ran %d computes, want 1", jobs_run);
    set_vol(40); main_pass(); for (int v = 0; v < 50; v++) set_vol((uint8_t)v); settle_and_check("burst across a compute");
    set_spl(60); settle_and_check("ref SPL change");
    set_int(200); settle_and_check("intensity change");
    set_rate(96000); settle_and_check("rate change");
    set_spl(100); set_int(50); set_rate(44100); set_vol(3); settle_and_check("all at once");
    set_en(false); settle_and_check("disable");
    jobs_run = 0; set_vol(55); set_spl(40); settle_and_check("changes while disabled");
    CHECK(jobs_run == 0, "computed while disabled");
    set_en(true); settle_and_check("enable");
    set_vol(12); main_pass(); set_en(false); settle_and_check("disable with compute pending");
    set_vol(33); set_en(true); set_en(false); set_en(true); settle_and_check("toggle burst");
    for (int k = 0; k < 7; k++) { set_vol((uint8_t)(k * 9)); settle_and_check("alternating buffers"); }

    printf("%s: %d checks, %d failed\n", fails ? "FAIL" : "PASS", checks, fails);
    return fails != 0;
}
