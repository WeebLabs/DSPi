/*
 * Subharmonic synthesizer: coefficient design, publish, and the headroom
 * bound.  The kernel lives in subharm.h; the design rationale and the
 * headroom formula are in Documentation/Features/subharmonic_synth_spec.md.
 *
 * All filters are TPT SVFs.  The band split and post-divider lowpasses are
 * Butterworth (k = sqrt 2); the bell uses the Cytomic form with k = 1/(Q*A).
 * The corners are far below Fs/7.5 at every supported rate, so the SVF form
 * is both the cheaper and the more precise choice on both platforms.
 */

#include <math.h>
#include <string.h>
#include "subharm.h"

// Live configuration; vendor handlers write it and raise the pending flag,
// the main loop recomputes + publishes.  Defaults match apply_factory_defaults.
volatile SubharmConfig subharm_config = {
    .enabled = false,
    .low_db = SUBHARM_DEFAULT_LOW,
    .high_db = SUBHARM_DEFAULT_HIGH,
    .boost_db = SUBHARM_DEFAULT_BOOST,
    .output_mask = SUBHARM_DEFAULT_OUTPUT_MASK,
};
volatile bool subharm_update_pending = false;

SubharmOutputState subharm_output_state[NUM_OUTPUT_CHANNELS];

volatile const SubharmCoeffs *current_subharm_coeffs = NULL;

// Double buffer so subharm_apply_config() can compute into the inactive
// buffer and publish, never writing through the currently published pointer.
static SubharmCoeffs sh_coeff_bufs[2];
static uint8_t sh_coeff_idx = 0;

static const float sh_pi = 3.1415926535f;
static const float sh_k_butterworth = 1.4142135624f;
static const float sh_sub_lp_hz[SUBHARM_NUM_BANDS] = { SUBHARM_SUB_LP0_HZ, SUBHARM_SUB_LP1_HZ };

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline sh_num_t sh_from_float(float v) {
#if PICO_RP2350
    return v;
#else
    return (int32_t)(v * (float)(1 << FILTER_SHIFT));
#endif
}

static void sh_svf_design(SubharmSvf *o, float fc, float k, float fs) {
    float g = tanf(sh_pi * fc / fs);
    float a1 = 1.0f / (1.0f + g * (g + k));
    o->a1 = sh_from_float(a1);
    o->a2 = sh_from_float(g * a1);
    o->a3 = sh_from_float(g * g * a1);
    o->k  = sh_from_float(k);
}

// Band level in dB to linear; the floor means off.
static float sh_level_gain(float db) {
    db = clampf(db, SUBHARM_LEVEL_MIN, SUBHARM_LEVEL_MAX);
    return db <= SUBHARM_LEVEL_MIN ? 0.0f : powf(10.0f, db / 20.0f);
}

// Bell amplitude A = sqrt of the linear gain, 1.0 when the boost is off.
static float sh_boost_amp(float db) {
    db = clampf(db, SUBHARM_BOOST_MIN, SUBHARM_BOOST_MAX);
    return db <= 0.0f ? 1.0f : powf(10.0f, db / 40.0f);
}

void subharm_compute_coefficients(SubharmCoeffs *c, const SubharmConfig *config, float sample_rate) {
    if (!config->enabled || sample_rate < 1.0f) {
        memset(c, 0, sizeof(SubharmCoeffs));
        return;
    }

    sh_svf_design(&c->hp_lo, SUBHARM_BAND_LO_HZ,  sh_k_butterworth, sample_rate);
    sh_svf_design(&c->lp_hi, SUBHARM_BAND_HI_HZ,  sh_k_butterworth, sample_rate);
    sh_svf_design(&c->split, SUBHARM_BAND_MID_HZ, sh_k_butterworth, sample_rate);

    const float level_db[SUBHARM_NUM_BANDS] = { config->low_db, config->high_db };
    for (int b = 0; b < SUBHARM_NUM_BANDS; b++) {
        sh_svf_design(&c->band[b].lp, sh_sub_lp_hz[b], sh_k_butterworth, sample_rate);
        c->band[b].gain = sh_from_float(sh_level_gain(level_db[b]));
    }

    {
        float g = tanf(sh_pi * SUBHARM_ALIGN_AP_HZ / sample_rate);
        c->align_g = sh_from_float(g / (1.0f + g));
    }

    float amp = sh_boost_amp(config->boost_db);
    if (amp > 1.0f) {
        float k = 1.0f / (SUBHARM_BOOST_Q * amp);
        sh_svf_design(&c->bell, SUBHARM_BOOST_HZ, k, sample_rate);
        c->bell_m1 = sh_from_float(k * (amp * amp - 1.0f));
    } else {
        memset(&c->bell, 0, sizeof(c->bell));
        c->bell_m1 = SH_ZERO;
    }

    c->env_decay = sh_from_float(expf(-1000.0f / (SUBHARM_ENV_TAU_MS * sample_rate)));
}

// ---------------------------------------------------------------------------
// Kernel.  Sample-major single pass, in place, no scratch buffer.  State
// lives in locals across the loop; the helpers take pointers to those locals
// and are always inlined, so nothing escapes to memory.
// ---------------------------------------------------------------------------

// One SVF step: returns the lowpass output and hands back v1 so the caller
// can form the highpass (x - k*v1 - lp) or the bell (x + m1*v1).
static inline __attribute__((always_inline))
sh_num_t sh_svf(const SubharmSvf *c, sh_num_t *ic1, sh_num_t *ic2, sh_num_t x, sh_num_t *v1_out) {
    sh_num_t v3 = x - *ic2;
    sh_num_t v1 = sh_mul(c->a1, *ic1) + sh_mul(c->a2, v3);
    sh_num_t v2 = *ic2 + sh_mul(c->a2, *ic1) + sh_mul(c->a3, v3);
    *ic1 = sh_twice(v1) - *ic1;
    *ic2 = sh_twice(v2) - *ic2;
    *v1_out = v1;
    return v2;
}

// Octave divider for one band: flip polarity at the first non-negative sample
// after the band has dipped below -env/4, then lowpass and scale.  Arming on
// the envelope-relative threshold is what stops beating partials or noise
// from re-triggering inside one cycle; flipping at the crossing itself keeps
// the divided waveform continuous.
static inline __attribute__((always_inline))
sh_num_t sh_band_tick(const SubharmBandCoeffs *c, sh_num_t decay, sh_num_t s,
                      sh_num_t *ic1, sh_num_t *ic2, sh_num_t *env,
                      uint8_t *armed, uint8_t *neg) {
    s = sh_band_limit(s);
    sh_num_t e = sh_mul(*env, decay);
    sh_num_t a = sh_abs(s);
    if (a > e) e = a;
    *env = e;
    if (*armed) {
        if (s >= SH_ZERO) { *neg ^= 1u; *armed = 0; }
    } else if (s < -sh_quarter(e)) {
        *armed = 1;
    }
    sh_num_t d = *neg ? -s : s;
    sh_num_t v1;
    sh_num_t sub = sh_svf(&c->lp, ic1, ic2, d, &v1);
    return sh_mul(c->gain, sub);
}

DSP_TIME_CRITICAL
void subharm_process_output_block(const SubharmCoeffs * __restrict c,
                                  SubharmOutputState * __restrict st,
                                  sh_num_t * __restrict buf, uint32_t n) {
    // A skipped band or bell keeps zeroed state so switching it back on is
    // transient-free (the pipeline only resets whole outputs).
    if (c->band[0].gain == SH_ZERO) memset(&st->band[0], 0, sizeof(st->band[0]));
    if (c->band[1].gain == SH_ZERO) { memset(&st->band[1], 0, sizeof(st->band[1])); st->align = SH_ZERO; }
    if (c->bell_m1 == SH_ZERO)      memset(&st->bell, 0, sizeof(st->bell));

    sh_num_t hl1 = st->hp_lo.ic1, hl2 = st->hp_lo.ic2;
    sh_num_t lh1 = st->lp_hi.ic1, lh2 = st->lp_hi.ic2;
    sh_num_t sp1 = st->split.ic1, sp2 = st->split.ic2;
    sh_num_t b0_1 = st->band[0].lp.ic1, b0_2 = st->band[0].lp.ic2, b0_env = st->band[0].env;
    uint8_t  b0_armed = st->band[0].armed, b0_neg = st->band[0].neg;
    sh_num_t b1_1 = st->band[1].lp.ic1, b1_2 = st->band[1].lp.ic2, b1_env = st->band[1].env;
    uint8_t  b1_armed = st->band[1].armed, b1_neg = st->band[1].neg;
    sh_num_t al = st->align;
    sh_num_t be1 = st->bell.ic1, be2 = st->bell.ic2;
    const sh_num_t decay = c->env_decay;

    for (uint32_t i = 0; i < n; i++) {
        sh_num_t x = buf[i];
        sh_num_t v1;

        // Band split: HP at LO, LP at HI, then one SVF at MID yields both bands
        sh_num_t lp = sh_svf(&c->hp_lo, &hl1, &hl2, x, &v1);
        sh_num_t hp = x - sh_mul(c->hp_lo.k, v1) - lp;
        sh_num_t s  = sh_svf(&c->lp_hi, &lh1, &lh2, hp, &v1);
        sh_num_t lo = sh_svf(&c->split, &sp1, &sp2, s, &v1);
        sh_num_t hi = s - sh_mul(c->split.k, v1) - lo;

        sh_num_t acc = x;
        if (c->band[0].gain != SH_ZERO)
            acc += sh_band_tick(&c->band[0], decay, lo, &b0_1, &b0_2, &b0_env, &b0_armed, &b0_neg);
        if (c->band[1].gain != SH_ZERO) {
            sh_num_t s1 = sh_band_tick(&c->band[1], decay, hi, &b1_1, &b1_2, &b1_env, &b1_armed, &b1_neg);
            // TPT one-pole allpass (2*lp - x) phase-aligns band 1 to band 0
            sh_num_t v = sh_mul(s1 - al, c->align_g);
            sh_num_t l = v + al;
            al = l + v;
            acc += sh_twice(l) - s1;
        }

        // LF boost bell after the sum (Cytomic bell: x + k*(A^2-1)*v1)
        if (c->bell_m1 != SH_ZERO) {
            (void)sh_svf(&c->bell, &be1, &be2, acc, &v1);
            acc += sh_mul(c->bell_m1, v1);
        }

        buf[i] = acc;
    }

    st->hp_lo.ic1 = hl1; st->hp_lo.ic2 = hl2;
    st->lp_hi.ic1 = lh1; st->lp_hi.ic2 = lh2;
    st->split.ic1 = sp1; st->split.ic2 = sp2;
    st->band[0].lp.ic1 = b0_1; st->band[0].lp.ic2 = b0_2; st->band[0].env = b0_env;
    st->band[0].armed = b0_armed; st->band[0].neg = b0_neg;
    st->band[1].lp.ic1 = b1_1; st->band[1].lp.ic2 = b1_2; st->band[1].env = b1_env;
    st->band[1].armed = b1_armed; st->band[1].neg = b1_neg;
    st->align = al;
    st->bell.ic1 = be1; st->bell.ic2 = be2;
}

void subharm_apply_config(const SubharmConfig *config, float sample_rate) {
    // Compute into the inactive buffer, then publish the pointer.  The
    // pipeline snapshots current_subharm_coeffs once per packet, so a plain
    // atomic pointer store suffices; the published buffer is never mutated.
    SubharmCoeffs *next = &sh_coeff_bufs[sh_coeff_idx ^ 1];
    subharm_compute_coefficients(next, config, sample_rate);
    if (config->enabled) {
        sh_coeff_idx ^= 1;
        current_subharm_coeffs = next;
    } else {
        current_subharm_coeffs = NULL;
    }
}

// ---------------------------------------------------------------------------
// Headroom bound.  Analog-prototype magnitudes (bilinear warping is
// negligible this far below Fs), scanned on an eighth-octave grid.
// ---------------------------------------------------------------------------

// Fourier amplitudes of the divided waveform (band sine times a half-rate
// square wave) at odd multiples of the sub frequency: 8/(3 pi), then
// (8/pi)/(n^2-4).  Terms past the 7th are below the post-lowpass floor.
#define SH_SCAN_HARMONICS 4
static const float sh_div_harm[SH_SCAN_HARMONICS] = { 0.8488f, 0.5093f, 0.1213f, 0.0566f };

#define SH_SCAN_F0     16.0f
#define SH_SCAN_STEP    1.0905077f   // 2^(1/8)
#define SH_SCAN_POINTS 33            // 16 Hz .. 256 Hz inclusive

static inline float sh_lp2_mag(float w) {
    float w2 = w * w;
    return 1.0f / sqrtf(1.0f + w2 * w2);
}

static inline float sh_hp2_mag(float w) {
    float w2 = w * w;
    return w2 / sqrtf(1.0f + w2 * w2);
}

static inline float sh_bell_mag(float w, float amp) {
    if (amp <= 1.0f) return 1.0f;
    float w2 = w * w;
    float d = (1.0f - w2) * (1.0f - w2);
    float num = w * amp / SUBHARM_BOOST_Q;
    float den = w / (amp * SUBHARM_BOOST_Q);
    return sqrtf((d + num * num) / (d + den * den));
}

float subharm_headroom_db(const SubharmConfig *config) {
    if (!config->enabled) return 0.0f;

    const float gain[SUBHARM_NUM_BANDS] = {
        sh_level_gain(config->low_db), sh_level_gain(config->high_db)
    };
    const float amp = sh_boost_amp(config->boost_db);

    float peak_max = 1.0f;
    float f = SH_SCAN_F0;
    for (int i = 0; i < SH_SCAN_POINTS; i++) {
        float pre = sh_hp2_mag(f / SUBHARM_BAND_LO_HZ) * sh_lp2_mag(f / SUBHARM_BAND_HI_HZ);
        const float band_mag[SUBHARM_NUM_BANDS] = {
            pre * sh_lp2_mag(f / SUBHARM_BAND_MID_HZ),
            pre * sh_hp2_mag(f / SUBHARM_BAND_MID_HZ),
        };

        // Direct tone at f plus every band's divided components at n*f/2,
        // all through the bell, summed as amplitudes (worst-case phase).
        float peak = sh_bell_mag(f / SUBHARM_BOOST_HZ, amp);
        for (int b = 0; b < SUBHARM_NUM_BANDS; b++) {
            if (gain[b] <= 0.0f) continue;
            float sum = 0.0f;
            for (int h = 0; h < SH_SCAN_HARMONICS; h++) {
                float fn = (float)(2 * h + 1) * 0.5f * f;
                sum += sh_div_harm[h] * sh_lp2_mag(fn / sh_sub_lp_hz[b])
                                      * sh_bell_mag(fn / SUBHARM_BOOST_HZ, amp);
            }
            peak += gain[b] * band_mag[b] * sum;
        }
        if (peak > peak_max) peak_max = peak;
        f *= SH_SCAN_STEP;
    }
    return 20.0f * log10f(peak_max);
}
