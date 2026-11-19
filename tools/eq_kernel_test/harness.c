// Host test of the RP2350 float EQ kernels (dsp_svf.h, dsp_biquad.h, dsp_cascade_block).
// Build and run with run.sh. Background: Documentation/current_architecture.md, "EQ kernel arithmetic".
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

volatile uint32_t output_ring_target_frames = 192;
void xover_init_default_filters(void) {}
void xover_recalculate_all(float sample_rate) { (void)sample_rate; }

void kfin_dsp_compute_coefficients(EqParamPacket *, Filter *, float);
void kfin_svf1(Filter *, float *, uint32_t);
void kfin_svf2(Filter *, float *, uint32_t);
void kfin_bq1(Filter *, float *, uint32_t);
void kfin_bq2(Filter *, float *, uint32_t);
void kfin_svf2x2(Filter *, Filter *, float *, uint32_t);
void kfin_bq2x2(Filter *, Filter *, float *, uint32_t);
void kfin_dsp_cascade_block(Filter *, uint32_t, float *, uint32_t);

#define FS 48000.0f
#define NMAX 96000
#ifndef SEEDS
#define SEEDS 24          // 6 left a 10 Hz Q20 SVF setting flagged by seed scatter alone
#endif
#define WORSE_DB 1.5      // arithmetic noise gate: new may not exceed old by more than this, seed-pooled
static float bin[NMAX], bo[NMAX], bn[NMAX];
static double dref[NMAX];
static int fails;

static uint64_t rng = 88172645463325252ull;
static double urand(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (rng >> 11) * (1.0 / 9007199254740992.0); }
static int cmp(const void *a, const void *b) { double x = *(const double *)a, y = *(const double *)b; return (x > y) - (x < y); }

static Filter design(int type, float fc, float q, float gain, int bypass) {
    Filter f; memset(&f, 0, sizeof f);
    EqParamPacket p = { 0, 0, (uint8_t)type, (uint8_t)bypass, fc, q, gain };
    kfin_dsp_compute_coefficients(&p, &f, FS);
    return f;
}

// Signal kinds: 0 noise -6 dBFS, 1 noise -100 dBFS, 2 sine ~1 kHz -6 dBFS, 3 sine near fc -6 dBFS.
// Sine frequencies avoid divisors of Fs: a periodic input repeats its rounding pattern and the
// error adds coherently into a gain error whose size is luck rather than kernel quality.
static const char *SIGNAME[] = { "noise-6", "noise-100", "sin1k-6", "sinfc-6" };
static void gen(float *b, int n, int kind, double fc) {
    double amp = kind == 1 ? 1e-5 : 0.5, hz = kind == 3 ? fc * 1.01234567 : 997.0, ph = 2 * M_PI * urand();
    for (int i = 0; i < n; i++)
        b[i] = (float)(kind <= 1 ? amp * (2 * urand() - 1) : amp * sin(2 * M_PI * hz * i / FS + ph));
}
static void errpow(const float *y, const double *r, int n, double *e, double *s) {
    for (int i = 0; i < n; i++) { double d = y[i] - r[i]; *e += d * d; *s += r[i] * r[i]; }
}
static void run1(void (*k)(Filter *, float *, uint32_t), Filter *f, float *b, int n, int blk) {
    for (int i = 0; i < n; i += blk) k(f, b + i, (uint32_t)(n - i < blk ? n - i : blk));
}

// ---------------------------------------------------------------- frozen previous kernels
// Float models of the SVF kernels this change replaced, kept for the numerics comparison only.
static void old_svf2(Filter *f, float *b, int n) {
    float a1 = f->sva1, a2 = f->sva2, g = f->g, m0 = f->svm0, m1 = f->svm1, m2 = f->svm2, i1 = f->svic1eq, i2 = f->svic2eq;
    for (int i = 0; i < n; i++) {
        float x = b[i], v3 = x - i2, v1 = a1 * i1 + a2 * v3, v2 = i2 + g * v1, y;
        i1 = v1 + v1 - i1; i2 = v2 + v2 - i2;
        switch (f->filter_type) {
            case FILTER_LOWPASS: y = v2; break;
            case FILTER_HIGHPASS: y = x + m1 * v1 - v2; break;
            case FILTER_PEAKING: case FILTER_NOTCH: case FILTER_ALLPASS: y = x + m1 * v1; break;
            default: y = x * m0 + m1 * v1 + m2 * v2; break;
        }
        b[i] = y;
    }
    f->svic1eq = i1; f->svic2eq = i2;
}
static void old_svf1(Filter *f, float *b, int n) {
    float a1 = f->sva1, a2 = f->sva2, m0 = f->svm0, m1 = f->svm1, m2 = f->svm2, i1 = f->svic1eq;
    for (int i = 0; i < n; i++) {
        float x = b[i], v1 = a2 * x + a1 * i1, y;
        i1 = v1 + v1 - i1;
        switch (f->filter_type) {
            case FILTER_LOWPASS1: y = v1; break;
            case FILTER_HIGHPASS1: y = x - v1; break;
            case FILTER_ALLPASS1: y = v1 + v1 - x; break;
            case FILTER_HIGHSHELF1: y = x + m2 * (x - v1); break;
            case FILTER_LOWSHELF1: y = x + m1 * v1; break;
            default: y = x * m0 + m1 * v1 + m2 * (x - v1); break;
        }
        b[i] = y;
    }
    f->svic1eq = i1;
}
static void old_svf2_blk(Filter *f, float *b, uint32_t n) { old_svf2(f, b, (int)n); }

// ---------------------------------------------------------------- test 1: fused == singles
static int arm_class(int t) {
    if (t == FILTER_LOWPASS || t == FILTER_HIGHPASS) return 0;
    if (t == FILTER_PEAKING || t == FILTER_NOTCH || t == FILTER_ALLPASS) return 1;
    return 2;
}
static const int T2[] = { FILTER_PEAKING, FILTER_NOTCH, FILTER_ALLPASS, FILTER_LOWPASS, FILTER_HIGHPASS,
                          FILTER_LOWSHELF, FILTER_HIGHSHELF, FILTER_LINKWITZ_TRANSFORM };
static const char *tname(int t) {
    static const char *n[] = { "FL", "PK", "LS", "HS", "LP", "HP", "NO", "AP", "AP1", "LS1", "HS1", "LT", "LP1", "HP1" };
    return t < 14 ? n[t] : "?";
}
static Filter design_t(int t, int svf, int salt) {
    float fc = svf ? 40.0f + 37.0f * salt : 7000.0f + 900.0f * salt;
    float gain = (t == FILTER_LINKWITZ_TRANSFORM) ? fc * 0.6f : (salt & 1 ? 9.0f : -7.5f);
    return design(t, fc, 0.5f + 1.3f * salt, gain, 0);
}
// Singles model of one fused biquad pair: pairs outside PK+PK and LP/HP fuse through the generic TDF2
// arm, which LT reaches in the single kernel.
static void bq_pair_model(Filter *f, Filter *g, float *b, uint32_t len) {
    int t0 = f->filter_type, t1 = g->filter_type;
    int l0 = t0 == FILTER_LOWPASS || t0 == FILTER_HIGHPASS, l1 = t1 == FILTER_LOWPASS || t1 == FILTER_HIGHPASS;
    if (!(t0 == FILTER_PEAKING && t1 == FILTER_PEAKING) && !(l0 && l1))
        f->filter_type = g->filter_type = FILTER_LINKWITZ_TRANSFORM;
    kfin_bq2(f, b, len); kfin_bq2(g, b, len);
    f->filter_type = t0; g->filter_type = t1;
}
static int same_state(const Filter *a, const Filter *b) {
    return !memcmp(&a->s1, &b->s1, 8) && !memcmp(&a->svic1eq, &b->svic1eq, 8);
}
static int fused_pair(int svf, Filter a0, Filter a1, const int *lens, int nl);
static int fused_case(int svf, int t0, int t1, const int *lens, int nl) {
    return fused_pair(svf, design_t(t0, svf, 1), design_t(t1, svf, 2), lens, nl);
}
static int fused_pair(int svf, Filter a0, Filter a1, const int *lens, int nl) {
    Filter b0 = a0, b1 = a1;
    int n = 0;
    for (int i = 0; i < nl; i++) n += lens[i];
    gen(bin, n, 0, 0);
    memcpy(bo, bin, n * 4); memcpy(bn, bin, n * 4);
    for (int i = 0, o = 0; i < nl; o += lens[i], i++) {
        if (svf) { kfin_svf2x2(&a0, &a1, bo + o, lens[i]); kfin_svf2(&b0, bn + o, lens[i]); kfin_svf2(&b1, bn + o, lens[i]); }
        else     { kfin_bq2x2(&a0, &a1, bo + o, lens[i]);  bq_pair_model(&b0, &b1, bn + o, lens[i]); }
    }
    return memcmp(bo, bn, n * 4) == 0 && same_state(&a0, &b0) && same_state(&a1, &b1);
}
static void test1(void) {
    static const int L[] = { 1, 2, 3, 4, 5, 7, 45, 48, 96 };
    static const int MIX[] = { 45, 1, 96, 3, 48, 2, 7, 4 };
    int ok = 0, total = 0;
    for (int svf = 0; svf <= 1; svf++)
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++) {
                if (svf && arm_class(T2[i]) != arm_class(T2[j])) continue;   // the walker never fuses these
                for (int l = 0; l <= 9; l++) {
                    int lens[5] = { L[l % 9], L[l % 9], L[l % 9], L[l % 9], L[l % 9] };
                    int r = l < 9 ? fused_case(svf, T2[i], T2[j], lens, 5) : fused_case(svf, T2[i], T2[j], MIX, 8);
                    ok += r; total++;
                    if (!r) printf("  FAIL fused != singles %s %s+%s %s\n", svf ? "SVF" : "BQ", tname(T2[i]), tname(T2[j]),
                                   l < 9 ? "fixed length" : "mixed lengths");
                }
            }
    printf("  fused == sequential singles (outputs + state, bit-exact, lengths 1..96 over 5+ blocks): %d/%d\n", ok, total);
    fails += total - ok;
}

// ---------------------------------------------------------------- test 2: numerics
static void svf_ref(const Filter *f, const float *in, int n) {
    double a1 = f->sva1, a2 = f->sva2, g = f->g, m0 = f->svm0, m1 = f->svm1, m2 = f->svm2, i1 = 0, i2 = 0;
    for (int i = 0; i < n; i++) {
        double v0 = in[i], v3 = v0 - i2, v1 = a1 * i1 + a2 * v3, v2 = i2 + g * v1;
        i1 = 2 * v1 - i1; i2 = 2 * v2 - i2;
        dref[i] = m0 * v0 + m1 * v1 + m2 * v2;
    }
}
// One SVF setting over SEEDS stimuli of one kind; seed-pooled error dB for the frozen old and new kernels.
static void pooled(Filter proto, double fc, int kind, double *odb, double *ndb) {
    const int N = NMAX;
    double eo = 0, so = 0, en = 0, sn = 0;
    for (int s = 0; s < SEEDS; s++) {
        Filter fo = proto, fn = proto;
        gen(bin, N, kind, fc);
        svf_ref(&proto, bin, N);
        memcpy(bo, bin, N * 4); memcpy(bn, bin, N * 4);
        run1(old_svf2_blk, &fo, bo, N, 48);
        run1(kfin_svf2, &fn, bn, N, 48);
        errpow(bo, dref, N, &eo, &so); errpow(bn, dref, N, &en, &sn);
    }
    *odb = 10 * log10(eo / so + 1e-300); *ndb = 10 * log10(en / sn + 1e-300);
}

static void test2(void) {
    // First-order SVF carries only the exact ic1 rescaling: bit-identical to the frozen old form.
    static const int T1[] = { FILTER_LOWPASS1, FILTER_HIGHPASS1, FILTER_ALLPASS1, FILTER_LOWSHELF1, FILTER_HIGHSHELF1 };
    static const float F1[] = { 10, 30, 100, 1000, 6000 };
    int fo_bad = 0, fo_n = 0;
    for (int t = 0; t < 5; t++)
        for (int fi = 0; fi < 5; fi++) {
            Filter fo = design(T1[t], F1[fi], 0.7f, 6, 0), fn = fo;
            gen(bin, 9600, 0, 0);
            memcpy(bo, bin, 9600 * 4); memcpy(bn, bin, 9600 * 4);
            old_svf1(&fo, bo, 9600); run1(kfin_svf1, &fn, bn, 9600, 45);
            fo_n++; fo_bad += memcmp(bo, bn, 9600 * 4) || memcmp(&fo.svic1eq, &fn.svic1eq, 4);
        }
    printf("  first-order SVF vs frozen old form: %d/%d cases differ (expect 0)\n", fo_bad, fo_n);
    fails += fo_bad;

    static const float F[] = { 10, 20, 50, 100, 300, 1000, 3000, 6000 };
    static const float Q[] = { 0.3f, 0.707f, 2, 8, 20 };
    static const float G[] = { -12, -3, 0.5f, 3, 12 };
    printf("  second-order SVF vs double reference, %d seeds pooled per setting, dB re output (fail: new > old + %.1f dB)\n",
           SEEDS, WORSE_DB);
    printf("    %-3s %5s %8s %8s %8s %8s  %s\n", "typ", "sets", "old med", "new med", "d med", "d worst", "worst setting");
    int nfail = 0, ntot = 0;
    for (int ti = 0; ti < 8; ti++) {
        int t = T2[ti], hasg = (t == FILTER_PEAKING || t == FILTER_LOWSHELF || t == FILTER_HIGHSHELF);
        static double od[2000], nd[2000], dd[2000];
        int c = 0; double worst = -1e9; char wbuf[96] = "";
        for (int fi = 0; fi < 8; fi++) for (int qi = 0; qi < 5; qi++) for (int gi = 0; gi < (hasg ? 5 : 1); gi++)
        for (int k = 0; k < 4; k++) {
            float gain = t == FILTER_LINKWITZ_TRANSFORM ? F[fi] * 0.6f : G[gi];
            if (t == FILTER_LINKWITZ_TRANSFORM && gain < 10.0f) continue;
            Filter f = design(t, F[fi], Q[qi], gain, 0);
            if (f.bypass || !f.use_svf) continue;
            pooled(f, F[fi], k, &od[c], &nd[c]);
            dd[c] = nd[c] - od[c];
            if (dd[c] > worst) { worst = dd[c]; snprintf(wbuf, sizeof wbuf, "%gHz Q%g g%g %s: old %.1f new %.1f", F[fi], Q[qi], gain, SIGNAME[k], od[c], nd[c]); }
            if (dd[c] > WORSE_DB) { nfail++; printf("    FAIL %s %gHz Q%g g%g %s: old %.1f new %.1f\n", tname(t), F[fi], Q[qi], gain, SIGNAME[k], od[c], nd[c]); }
            c++; ntot++;
        }
        qsort(od, c, 8, cmp); qsort(nd, c, 8, cmp); qsort(dd, c, 8, cmp);
        printf("    %-3s %5d %8.1f %8.1f %+8.2f %+8.2f  %s\n", tname(t), c, od[c / 2], nd[c / 2], dd[c / 2], worst, wbuf);
    }
    printf("    SVF settings: %d, failing: %d\n", ntot, nfail);
    fails += nfail;

}

// ---------------------------------------------------------------- test 3: state continuity
// Alternates kernels per block against one continuous frozen-old run; a state-meaning mismatch would
// show as deviation far above the rounding floor of an unswitched new run.
static void test3(void) {
    static const struct { float fc, q, g; } C[] = { { 30, 8, 12 }, { 1000, 2, -6 }, { 15, 20, 3 } };
    static const int LENS[] = { 48, 1, 3, 45, 96, 2, 7 };
    printf("  max |deviation| from a continuous frozen-old run, dB re peak (switched / unswitched new):\n");
    for (unsigned i = 0; i < sizeof C / sizeof C[0]; i++) {
        const int n = 48 * 80;
        Filter fr = design(FILTER_PEAKING, C[i].fc, C[i].q, C[i].g, 0), fs = fr, fn = fr;
        gen(bin, n, 0, 0);
        static float bx[NMAX];
        memcpy(bo, bin, n * 4); memcpy(bn, bin, n * 4); memcpy(bx, bin, n * 4);
        for (int o = 0, blk = 0; o < n; blk++) {
            int len = LENS[blk % 7]; if (o + len > n) len = n - o;
            old_svf2(&fr, bo + o, len); kfin_svf2(&fn, bx + o, len);
            if (blk % 2) kfin_svf2(&fs, bn + o, len); else old_svf2(&fs, bn + o, len);
            o += len;
        }
        double pk = 0, d = 0, b = 0;
        for (int j = 0; j < n; j++) { pk = fmax(pk, fabs(bo[j])); d = fmax(d, fabs(bn[j] - bo[j])); b = fmax(b, fabs(bx[j] - bo[j])); }
        double ddb = 20 * log10(d / pk + 1e-300), bdb = 20 * log10(b / pk + 1e-300);
        int bad = ddb > bdb + 6 || ddb > -90;
        printf("    SVF PK %gHz Q%g %+gdB: %.1f / %.1f%s\n", C[i].fc, C[i].q, C[i].g, ddb, bdb, bad ? "  FAIL" : "");
        fails += bad;
    }
}

// ---------------------------------------------------------------- test 4: dispatch
static const int TALL[] = { FILTER_PEAKING, FILTER_NOTCH, FILTER_ALLPASS, FILTER_LOWPASS, FILTER_HIGHPASS,
                            FILTER_LOWSHELF, FILTER_HIGHSHELF, FILTER_LINKWITZ_TRANSFORM, FILTER_LOWPASS1,
                            FILTER_HIGHPASS1, FILTER_ALLPASS1, FILTER_LOWSHELF1, FILTER_HIGHSHELF1 };
// Independent model of the dsp_cascade_block pairing rules, built from single-section kernels only.
static void sequential(Filter *s, uint32_t n, float *b, uint32_t len) {
    uint32_t i = 0;
    while (i < n) {
        Filter *f = &s[i];
        if (f->bypass) { i++; continue; }
        uint32_t j = i + 1;
        while (j < n && s[j].bypass) j++;
        Filter *g = j < n ? &s[j] : NULL;
        if (g && !f->first_order && !g->first_order && f->use_svf == g->use_svf &&
            (!f->use_svf || arm_class(f->filter_type) == arm_class(g->filter_type))) {
            if (f->use_svf) { kfin_svf2(f, b, len); kfin_svf2(g, b, len); }
            else bq_pair_model(f, g, b, len);
            i = j + 1;
            continue;
        }
        if (f->use_svf) (f->first_order ? kfin_svf1 : kfin_svf2)(f, b, len);
        else            (f->first_order ? kfin_bq1 : kfin_bq2)(f, b, len);
        i++;
    }
}
static void test4(int trials) {
    static const int L[] = { 1, 2, 3, 45, 48, 96 };
    int ok = 0;
    for (int t = 0; t < trials; t++) {
        Filter c[12], s[12];
        uint32_t n = 1 + (uint32_t)(urand() * 12);
        int low = urand() < 0.5;   // half the trials are all-SVF so long same-arm runs occur
        for (uint32_t i = 0; i < n; i++) {
            int ty = TALL[(int)(urand() * (urand() < 0.7 ? 8 : 13))];
            float fc = (float)(20 * pow(low ? 300 : 1000, urand()));
            float g = ty == FILTER_LINKWITZ_TRANSFORM ? fc * 0.6f : (float)(24 * urand() - 12);
            c[i] = s[i] = design(ty, fc, (float)(0.3 + 8 * urand()), g, urand() < 0.25);
        }
        int total = 0;
        for (int b = 0; b < 6; b++) total += L[(t + b) % 6];
        gen(bin, total, 0, 0);
        memcpy(bo, bin, total * 4); memcpy(bn, bin, total * 4);
        for (int b = 0, o = 0; b < 6; o += L[(t + b) % 6], b++) {
            kfin_dsp_cascade_block(c, n, bo + o, L[(t + b) % 6]);
            sequential(s, n, bn + o, L[(t + b) % 6]);
        }
        int same = memcmp(bo, bn, total * 4) == 0;
        for (uint32_t i = 0; i < n; i++) same &= same_state(&c[i], &s[i]);
        ok += same;
        if (!same) printf("  FAIL cascade != pairing model, trial %d (n=%u)\n", t, n);
    }
    printf("  dsp_cascade_block == pairing model (bit-exact): %d/%d random cascades\n", ok, trials);
    fails += trials - ok;
}

int main(void) {
    printf("[1] fused vs sequential singles\n");          test1();
    printf("[2] numerics: new vs frozen old vs double reference\n"); test2();
    printf("[3] state continuity across kernels and odd block lengths\n"); test3();
    printf("[4] dsp_cascade_block dispatch\n");           test4(3000);
    printf("%s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
    return fails != 0;
}
