/* Host check of the RP2350 loudness and psybass kernels against frozen pre-rewrite models and a double reference. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "loudness.h"
#include "psybass.h"
#include "bg_job.h"

void bg_job_register(BgJob *j) { (void)j; }

#define N 24000
static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rng;
static float frand(void) { rng = rng * 1664525u + 1013904223u; return (float)(int32_t)rng / 2147483648.0f; }
static uint32_t blk(void) { rng = rng * 1664525u + 1013904223u; return 1 + (rng >> 8) % 96; }

// Frozen pre-rewrite float kernels (sva3 = g*sva2, as the old design code stored it).
static void loud_old(const LoudnessCoeffs *lc, float st[2][2], float *buf, uint32_t n) {
    for (int j = 0; j < 2; j++) {
        if (lc[j].bypass) { st[j][0] = st[j][1] = 0.0f; continue; }
        float a1 = lc[j].sva1, a2 = lc[j].sva2, a3 = lc[j].g * lc[j].sva2, ic1 = st[j][0], ic2 = st[j][1];
        for (uint32_t i = 0; i < n; i++) {
            float x = buf[i], v3 = x - ic2;
            float v1 = a1 * ic1 + a2 * v3;
            float v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2.0f * v1 - ic1; ic2 = 2.0f * v2 - ic2;
            buf[i] = lc[j].svm0 * x + lc[j].svm1 * v1 + lc[j].svm2 * v2;
        }
        st[j][0] = ic1; st[j][1] = ic2;
    }
}
static void loud_ref(const LoudnessCoeffs *lc, double *buf, uint32_t n) {
    for (int j = 0; j < 2; j++) {
        if (lc[j].bypass) continue;
        double a1 = lc[j].sva1, a2 = lc[j].sva2, a3 = (double)lc[j].g * lc[j].sva2, ic1 = 0, ic2 = 0;
        for (uint32_t i = 0; i < n; i++) {
            double x = buf[i], v3 = x - ic2, v1 = a1 * ic1 + a2 * v3, v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2 * v1 - ic1; ic2 = 2 * v2 - ic2;
            buf[i] = lc[j].svm0 * x + lc[j].svm1 * v1 + lc[j].svm2 * v2;
        }
    }
}

typedef struct { float lp1, lp2, hp1, hp2, hl; } PbOld;
#define PB_BODY(T, FABS) \
    T x = buf[i], v3 = x - lp2, v1 = (T)c->lp_a1 * lp1 + (T)c->lp_a2 * v3; \
    T v2 = lp2 + (T)c->lp_a2 * lp1 + a3 * v3; lp1 = 2 * v1 - lp1; lp2 = 2 * v2 - lp2; T low = v2; \
    T even = FABS(low), d = (T)c->drive * low; if (d > 1) d = 1; else if (d < -1) d = -1; \
    T odd = d * ((T)1.5 - (T)0.5 * d * d), h = (T)c->even_w * even + (T)c->odd_w * odd; \
    v3 = h - hp2; v1 = (T)c->hp_a1 * hp1 + (T)c->hp_a2 * v3; v2 = hp2 + (T)c->hp_a2 * hp1 + ha3 * v3; \
    hp1 = 2 * v1 - hp1; hp2 = 2 * v2 - hp2; T hpo = h - (T)c->hp_k * v1 - v2; \
    hl = (T)c->hl_a0 * hpo + (T)c->hl_b1 * hl; buf[i] = x + (T)c->orig_delta * low + (T)c->harm_gain * hl;
static void pb_old(const PsybassCoeffs *c, PbOld *s, float *buf, uint32_t n) {
    float lp1 = s->lp1, lp2 = s->lp2, hp1 = s->hp1, hp2 = s->hp2, hl = s->hl;
    float a3 = c->lp_g * c->lp_a2, ha3 = c->hp_g * c->hp_a2;
    for (uint32_t i = 0; i < n; i++) { PB_BODY(float, fabsf) }
    s->lp1 = lp1; s->lp2 = lp2; s->hp1 = hp1; s->hp2 = hp2; s->hl = hl;
}
static void pb_ref(const PsybassCoeffs *c, double *buf, uint32_t n) {
    double lp1 = 0, lp2 = 0, hp1 = 0, hp2 = 0, hl = 0;
    double a3 = (double)c->lp_g * c->lp_a2, ha3 = (double)c->hp_g * c->hp_a2;
    for (uint32_t i = 0; i < n; i++) { PB_BODY(double, fabs) }
}

static float in[N], a[N], b[N], whole[N];
static double r[N];

static double err_db(const float *y, const double *ref) {
    double e = 0, s = 0;
    for (int i = 0; i < N; i++) { double d = y[i] - ref[i]; e += d * d; s += ref[i] * ref[i]; }
    return 10 * log10((e + 1e-300) / (s + 1e-300));
}

static void make_input(int kind, int seed) {
    rng = (uint32_t)seed * 2654435761u + 1;
    for (int i = 0; i < N; i++)
        in[i] = kind == 0 ? 0.5f * frand() : 0.5f * sinf(2.0f * 3.14159265f * 997.3f * (float)i / 48000.0f) + 0.01f * frand();
}

// Runs one kernel over in[] in random block lengths into out[].
#define RUN_BLOCKS(out, CALL) do { memcpy(out, in, sizeof in); uint32_t p_ = 0; \
    while (p_ < N) { uint32_t m_ = blk(); if (m_ > N - p_) m_ = N - p_; CALL(out + p_, m_); p_ += m_; } } while (0)

static double pooled_new, pooled_old; static int pooled_n;

static void test_loudness(const LoudnessCoeffs *lc, const char *tag) {
    for (int kind = 0; kind < 2; kind++) for (int seed = 0; seed < 3; seed++) {
        make_input(kind, seed);
        LoudnessOutputState st; memset(&st, 0, sizeof st);
        float os[2][2] = {{0}};
#define LNEW(p, m) loudness_process_output_block(lc, &st, p, m)
#define LOLD(p, m) loud_old(lc, os, p, m)
        RUN_BLOCKS(b, LNEW);
        RUN_BLOCKS(a, LOLD);
        memset(&st, 0, sizeof st); memcpy(whole, in, sizeof in); loudness_process_output_block(lc, &st, whole, N);
        CHECK(!memcmp(whole, b, sizeof b), "loudness %s: block split changes output", tag);
        for (int i = 0; i < N; i++) r[i] = in[i];
        loud_ref(lc, r, N);
        double en = err_db(b, r), eo = err_db(a, r);
        pooled_new += pow(10, en / 10); pooled_old += pow(10, eo / 10); pooled_n++;
        CHECK(en < -100.0, "loudness %s: new error %.1f dB", tag, en);
    }
}

static void test_psybass(const PsybassCoeffs *c, const char *tag) {
    for (int kind = 0; kind < 2; kind++) for (int seed = 0; seed < 3; seed++) {
        make_input(kind, seed);
        PsybassOutputState st; memset(&st, 0, sizeof st);
        PbOld os; memset(&os, 0, sizeof os);
#define PNEW(p, m) psybass_process_output_block(c, &st, p, m)
#define POLD(p, m) pb_old(c, &os, p, m)
        RUN_BLOCKS(b, PNEW);
        RUN_BLOCKS(a, POLD);
        memset(&st, 0, sizeof st); memcpy(whole, in, sizeof in); psybass_process_output_block(c, &st, whole, N);
        CHECK(!memcmp(whole, b, sizeof b), "psybass %s: block split changes output", tag);
        for (int i = 0; i < N; i++) r[i] = in[i];
        pb_ref(c, r, N);
        double en = err_db(b, r), eo = err_db(a, r);
        pooled_new += pow(10, en / 10); pooled_old += pow(10, eo / 10); pooled_n++;
        CHECK(en < -90.0, "psybass %s: new error %.1f dB", tag, en);
    }
}

static void gate(const char *what) {
    double n = 10 * log10(pooled_new / pooled_n), o = 10 * log10(pooled_old / pooled_n);
    printf("%s: error vs double, new %.2f dB, old %.2f dB, over %d runs\n", what, n, o, pooled_n);
    CHECK(n <= o + 1.5, "%s: new error exceeds old by more than 1.5 dB", what);
    pooled_new = pooled_old = 0; pooled_n = 0;
}

// A bypassed shelf as compute_shelf_coeffs() publishes it: identity coefficients.
static void bypass_shelf(LoudnessCoeffs *c) {
    c->bypass = true; c->sva1 = c->sva2 = c->g = 0.0f; c->svm0 = 1.0f; c->svm1 = c->svm2 = 0.0f;
}

int main(void) {
    const float rates[] = { 44100, 48000, 96000 };
    char tag[96];
    int one_bypass = 0;
    for (int ri = 0; ri < 3; ri++)
        for (int vol = 0; vol < LOUDNESS_VOL_STEPS; vol += 6)
            for (int spl = 75; spl <= 95; spl += 10) {
                loudness_update_now(true, (uint8_t)vol, (float)spl, 100.0f, rates[ri]);
                const LoudnessCoeffs *lc = (const LoudnessCoeffs *)current_loudness_coeffs;
                if (!lc) continue;
                LoudnessCoeffs cp[2]; memcpy(cp, lc, sizeof cp);
                snprintf(tag, sizeof tag, "rate %.0f vol %d spl %d", rates[ri], vol, spl);
                test_loudness(cp, tag);
                if (!cp[0].bypass && !cp[1].bypass) {        // single-shelf paths
                    LoudnessCoeffs one[2]; memcpy(one, cp, sizeof one); bypass_shelf(&one[0]); test_loudness(one, tag);
                    memcpy(one, cp, sizeof one); bypass_shelf(&one[1]); test_loudness(one, tag); one_bypass++;
                }
            }
    gate("loudness");
    for (int ri = 0; ri < 3; ri++)
        for (float fc = 40; fc <= 160; fc += 40)
            for (float drive = 0; drive <= 18; drive += 9)
                for (float ch = 0; ch <= 100; ch += 50) {
                    PsybassConfig cfg = { true, fc, 0.0f, drive, ch, -6.0f, 0x1FF };
                    PsybassCoeffs c; psybass_compute_coefficients(&c, &cfg, rates[ri]);
                    snprintf(tag, sizeof tag, "rate %.0f fc %.0f drive %.0f char %.0f", rates[ri], fc, drive, ch);
                    test_psybass(&c, tag);
                }
    gate("psybass");
    printf("%s: %d checks, %d failed (single-shelf cases %d)\n", fails ? "FAIL" : "PASS", checks, fails, one_bypass);
    return fails != 0;
}
