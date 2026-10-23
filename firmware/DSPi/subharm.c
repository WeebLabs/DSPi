/*
 * Subharmonic synthesizer: coefficient design, publish, the kernel, the
 * per-packet output pass, and the headroom bound.  The design rationale,
 * the alignment / parity-slaving model results and the headroom formula are
 * in Documentation/Features/subharmonic_synth_spec.md.
 *
 * All filters are TPT SVFs.  The band split and post-divider lowpasses are
 * Butterworth (k = sqrt 2); the bell uses the Cytomic form with k = 1/(Q*A).
 * The band split, dividers, detectors and ceiling run at the decimated rate
 * (about 8 kHz); the anti-alias lowpass, interpolator and bell at full rate.
 */

#include <math.h>
#include <string.h>
#include "subharm.h"
#include "usb_audio.h"   // matrix_mixer: per-output enable / mute gate the pass
#include "siggen.h"      // siggen_raw_mask: RAW outputs bypass the effect

// Live configuration; vendor handlers write it and raise the pending flag,
// the main loop recomputes + publishes.  Defaults match apply_factory_defaults.
volatile SubharmConfig subharm_config = {
    .enabled = false,
    .low_db = SUBHARM_DEFAULT_LOW,
    .high_db = SUBHARM_DEFAULT_HIGH,
    .boost_db = SUBHARM_DEFAULT_BOOST,
    .output_mask = SUBHARM_DEFAULT_OUTPUT_MASK,
    .top_db = SUBHARM_DEFAULT_TOP,
    .select_mode = SUBHARM_DEFAULT_SELECT_MODE,
    .select_depth = SUBHARM_DEFAULT_DEPTH,
    .select_hold_ms = SUBHARM_DEFAULT_HOLD_MS,
    .ceiling_db = SUBHARM_DEFAULT_CEILING,
    .link_pairs = SUBHARM_DEFAULT_LINK_PAIRS,
    .solo = false,
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
static const float sh_sub_lp_hz[SUBHARM_NUM_BANDS] = {
    SUBHARM_SUB_LP0_HZ, SUBHARM_SUB_LP1_HZ, SUBHARM_SUB_LP2_HZ
};

// NaN-safe: a NaN from a bad host float lands on the floor, never in a filter.
static inline float clampf(float v, float lo, float hi) {
    return v >= lo ? (v <= hi ? v : hi) : lo;
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
    o->g  = sh_from_float(g);
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

static inline float sh_onepole_decay(float tau_ms, float fs) {
    return expf(-1000.0f / (tau_ms * fs));
}

void subharm_compute_coefficients(SubharmCoeffs *c, const SubharmConfig *config, float sample_rate) {
    if (!config->enabled || sample_rate < 1.0f) {
        memset(c, 0, sizeof(SubharmCoeffs));
        c->decim = 1;
        return;
    }

    int decim = (int)(sample_rate / SUBHARM_DECIM_TARGET_HZ + 0.5f);
    if (decim < 1) decim = 1;
    if (decim > 255) decim = 255;
    const float fl = sample_rate / (float)decim;
    c->decim = (uint8_t)decim;
    c->inv_decim = sh_from_float(1.0f / (float)decim);

    sh_svf_design(&c->aa,     SUBHARM_BAND_HI_HZ,   sh_k_butterworth, sample_rate);
    sh_svf_design(&c->hp_lo,  SUBHARM_BAND_LO_HZ,   sh_k_butterworth, fl);
    sh_svf_design(&c->split2, SUBHARM_BAND_MID2_HZ, sh_k_butterworth, fl);
    sh_svf_design(&c->split1, SUBHARM_BAND_MID1_HZ, sh_k_butterworth, fl);

    const float level_db[SUBHARM_NUM_BANDS] = { config->low_db, config->high_db, config->top_db };
    const bool top_on = sh_level_gain(config->top_db) > 0.0f;
    const float align_hz[SUBHARM_NUM_BANDS] = {
        0.0f,
        top_on ? SUBHARM_ALIGN_AP1_3B_HZ : SUBHARM_ALIGN_AP1_2B_HZ,
        SUBHARM_ALIGN_AP2_HZ,
    };
    for (int b = 0; b < SUBHARM_NUM_BANDS; b++) {
        sh_svf_design(&c->band[b].lp, sh_sub_lp_hz[b], sh_k_butterworth, fl);
        c->band[b].gain = sh_from_float(sh_level_gain(level_db[b]));
        if (align_hz[b] > 0.0f) {
            float g = tanf(sh_pi * align_hz[b] / fl);
            c->band[b].align_g = sh_from_float(g / (1.0f + g));
        } else {
            c->band[b].align_g = SH_ZERO;
        }
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

    c->env_decay = sh_from_float(sh_onepole_decay(SUBHARM_ENV_TAU_MS, fl));

    // Selectivity
    uint8_t mode = config->select_mode;
    if (mode > SUBHARM_SELECT_MODE_MAX) mode = SUBHARM_SELECT_MODE_MAX;
    c->select_mode = mode;
    float hold_ms = clampf(config->select_hold_ms, SUBHARM_HOLD_MIN, SUBHARM_HOLD_MAX);
    c->hold        = (uint16_t)(hold_ms * fl / 1000.0f);
    c->slow_decay  = sh_from_float(sh_onepole_decay(SUBHARM_SLOW_TAU_MS, fl));
    c->slow_rise   = sh_from_float(1.0f - sh_onepole_decay(SUBHARM_SLOW_RISE_MS, fl));
    c->gate_open   = sh_from_float(1.0f - sh_onepole_decay(SUBHARM_GATE_OPEN_MS, fl));
    c->gate_close  = sh_from_float(1.0f - sh_onepole_decay(SUBHARM_GATE_CLOSE_MS, fl));
    c->depth       = sh_from_float(clampf(config->select_depth, SUBHARM_DEPTH_MIN, SUBHARM_DEPTH_MAX) / 100.0f);
    c->alive_floor = sh_from_float(SUBHARM_ALIVE_FLOOR);

    // Ceiling
    float ceil_db = clampf(config->ceiling_db, SUBHARM_CEILING_MIN, SUBHARM_CEILING_MAX);
    c->ceil         = ceil_db < 0.0f ? sh_from_float(powf(10.0f, ceil_db / 20.0f)) : SH_ZERO;
    c->ceil_env_rel = sh_from_float(sh_onepole_decay(SUBHARM_CEIL_RELEASE_MS, fl));
    c->ceil_attack  = sh_from_float(1.0f - sh_onepole_decay(SUBHARM_CEIL_ATTACK_MS, fl));
    c->ceil_release = sh_from_float(1.0f - sh_onepole_decay(SUBHARM_CEIL_RELEASE_MS, fl));

    c->meter_decay = sh_from_float(sh_onepole_decay(SUBHARM_METER_TAU_MS, fl));
}

// ---------------------------------------------------------------------------
// Kernel.  Full-rate loop with state in locals; low-rate helpers are
// out-of-line RAM functions (inlining them cost ~4 KB of RAM).
// ---------------------------------------------------------------------------

// One SVF step: returns the lowpass output and hands back v1 so the caller
// can form the highpass (x - k*v1 - lp) or the bell (x + m1*v1).
static inline __attribute__((always_inline))
sh_num_t sh_svf(const SubharmSvf *c, sh_num_t *ic1, sh_num_t *ic2, sh_num_t x, sh_num_t *v1_out) {
    sh_num_t v3 = x - *ic2;
    sh_num_t v1 = sh_mul(c->a1, *ic1) + sh_mul(c->a2, v3);
    sh_num_t v2 = *ic2 + sh_mul(c->g, v1);
    *ic1 = sh_twice(v1) - *ic1;
    *ic2 = sh_twice(v2) - *ic2;
    *v1_out = v1;
    return v2;
}

// Selectivity gate for one band, from its just-updated envelope.  An attack
// (fast env above 2x the slow follower) or silence resets the hold counter;
// percussive opens instantly and closes after the hold, sustained opens after
// the hold and shuts instantly on an attack so a kick under a held note ducks.
static DSP_TIME_CRITICAL __attribute__((noinline))
sh_num_t sh_select_gain(const SubharmCoeffs *c, SubharmBandState *bs, sh_num_t e) {
    sh_num_t es = sh_mul(bs->env_slow, c->slow_decay);
    if (e > es) es += sh_mul(e - es, c->slow_rise);
    bs->env_slow = es;
    const bool attack = e > es * SUBHARM_ATTACK_RATIO;
    const bool alive  = e > c->alive_floor;
    if (attack || !alive) bs->hold_cnt = 0;
    else if (bs->hold_cnt <= c->hold) bs->hold_cnt++;

    const bool in_hold = bs->hold_cnt < c->hold;
    sh_num_t target = SH_ZERO;
    if (alive && (c->select_mode == SUBHARM_SELECT_PERCUSSIVE ? in_hold : !in_hold))
        target = SH_ONE;

    sh_num_t w = bs->weight;
    if (c->select_mode == SUBHARM_SELECT_SUSTAINED) {
        if (target > w)      w += sh_mul(target - w, c->gate_open);
        else if (attack)     w = target;
        else                 w += sh_mul(target - w, c->gate_close);
    } else {
        if (target > w)      w = target;
        else                 w += sh_mul(target - w, c->gate_close);
    }
    bs->weight = w;
    return SH_ONE - sh_mul(c->depth, SH_ONE - w);
}

// Octave divider for one band: flip polarity at the first non-negative sample
// after the band has dipped below -env/4, then gate, lowpass, align and scale.
// `force` < 0 toggles at the flip; otherwise the parity is set to `force`
// (band 2 slaved to band 0, see subharm_low_rate_tick).
static DSP_TIME_CRITICAL __attribute__((noinline))
sh_num_t sh_band_tick(const SubharmCoeffs *c, const SubharmBandCoeffs *bc,
                      SubharmBandState *bs, sh_num_t s, int force) {
    s = sh_band_limit(s);
    sh_num_t e = sh_mul(bs->env, c->env_decay);
    sh_num_t a = sh_abs(s);
    if (a > e) e = a;
    bs->env = e;
    if (bs->period_cnt < 0xFFFFu) bs->period_cnt++;
    if (bs->armed) {
        if (s >= SH_ZERO) {
            bs->neg = (uint8_t)(force < 0 ? (bs->neg ^ 1u) : (unsigned)force);
            bs->armed = 0;
            bs->period = bs->period_cnt;
            bs->period_cnt = 0;
        }
    } else if (s < -sh_quarter(e)) {
        bs->armed = 1;
    }
    sh_num_t d = bs->neg ? -s : s;
    if (c->select_mode != SUBHARM_SELECT_ALL)
        d = sh_mul(d, sh_select_gain(c, bs, e));
    sh_num_t v1;
    sh_num_t sub = sh_svf(&bc->lp, &bs->lp.ic1, &bs->lp.ic2, d, &v1);
    if (bc->align_g != SH_ZERO) {
        // TPT one-pole allpass (2*lp - x)
        sh_num_t v = sh_mul(sub - bs->align, bc->align_g);
        sh_num_t l = v + bs->align;
        bs->align = l + v;
        sub = sh_twice(l) - sub;
    }
    return sh_band_out_limit(sh_mul(bc->gain, sub));
}

// One low-rate step on the anti-aliased input: band split, three dividers,
// ceiling, meter.  Returns the synthesized sub for this low-rate sample.
static DSP_TIME_CRITICAL __attribute__((noinline))
sh_num_t subharm_low_rate_tick(const SubharmCoeffs *c, SubharmOutputState *st, sh_num_t y) {
    sh_num_t v1;
    sh_num_t lp  = sh_svf(&c->hp_lo, &st->hp_lo.ic1, &st->hp_lo.ic2, y, &v1);
    sh_num_t s   = y - sh_mul(c->hp_lo.k, v1) - lp;
    sh_num_t lo2 = sh_svf(&c->split2, &st->split2.ic1, &st->split2.ic2, s, &v1);
    sh_num_t b2  = s - sh_mul(c->split2.k, v1) - lo2;
    sh_num_t b0  = sh_svf(&c->split1, &st->split1.ic1, &st->split1.ic2, lo2, &v1);
    sh_num_t b1  = lo2 - sh_mul(c->split1.k, v1) - b0;

    sh_num_t acc = SH_ZERO;
    const bool on0 = c->band[0].gain != SH_ZERO;
    if (on0)
        acc += sh_band_tick(c, &c->band[0], &st->band[0], b0, -1);
    if (c->band[1].gain != SH_ZERO)
        acc += sh_band_tick(c, &c->band[1], &st->band[1], b1, -1);
    if (c->band[2].gain != SH_ZERO) {
        // Bands 0 and 2 are not adjacent, so no allpass can hold them in
        // quadrature.  When one note leaks into both (equal flip periods)
        // band 2's parity is set from band 0's divided sign instead; the
        // model shows this is what keeps the three-band sum history-free.
        int force = -1;
        const SubharmBandState *s0 = &st->band[0];
        const SubharmBandState *s2 = &st->band[2];
        if (on0 && s0->env > c->alive_floor && s0->period != 0 && s2->period != 0) {
            int dp = (int)s0->period - (int)s2->period;
            if (dp < 0) dp = -dp;
            if (dp * 16 <= (int)s2->period)
                force = (int)((((b0 > SH_ZERO) ? 1u : 0u) ^ s0->neg) ^ 1u);
        }
        acc += sh_band_tick(c, &c->band[2], &st->band[2], b2, force);
    }

    if (c->ceil != SH_ZERO) {
        sh_num_t e = sh_mul(st->ceil_env, c->ceil_env_rel);
        sh_num_t a = sh_abs(acc);
        if (a > e) e = a;
        st->ceil_env = e;
        sh_num_t target = (e > c->ceil) ? sh_ratio(c->ceil, e) : SH_ONE;
        sh_num_t cg = st->ceil_gain;
        if (target < cg) cg += sh_mul(target - cg, c->ceil_attack);
        else             cg += sh_mul(target - cg, c->ceil_release);
        st->ceil_gain = cg;
        acc = sh_mul(acc, cg);
    }

    acc = sh_sub_limit(acc);
    sh_num_t m = sh_mul(st->meter, c->meter_decay);
    sh_num_t a = sh_abs(acc);
    st->meter = a > m ? a : m;
    return acc;
}

DSP_TIME_CRITICAL
void subharm_process_block(const SubharmCoeffs * __restrict c,
                           SubharmOutputState * __restrict st,
                           SubharmOutputState * __restrict st_b,
                           sh_num_t * __restrict buf, sh_num_t * __restrict buf_b,
                           uint32_t n, uint8_t phase0, bool solo) {
    if (!st->active) { subharm_reset_output_state(st); st->active = 1; }
    // A skipped band, detector or bell keeps zeroed state so switching it
    // back on is transient-free (the pipeline only resets whole outputs).
    for (int b = 0; b < SUBHARM_NUM_BANDS; b++) {
        if (c->band[b].gain == SH_ZERO) memset(&st->band[b], 0, sizeof(st->band[b]));
        else if (c->select_mode == SUBHARM_SELECT_ALL) {
            st->band[b].weight = SH_ZERO; st->band[b].env_slow = SH_ZERO; st->band[b].hold_cnt = 0;
        }
    }
    if (c->ceil == SH_ZERO) { st->ceil_env = SH_ZERO; st->ceil_gain = SH_ONE; }
    const bool pair = buf_b != NULL;
    const bool bell = c->bell_m1 != SH_ZERO && !solo;
    if (!bell) {
        // Also while soloed: a frozen bell integrator would dump into the
        // output the moment solo is released.
        memset(&st->bell, 0, sizeof(st->bell));
        if (st_b) memset(&st_b->bell, 0, sizeof(st_b->bell));
    }

    sh_num_t aa1 = st->aa.ic1, aa2 = st->aa.ic2;
    sh_num_t cur = st->cur, step = st->step;
    uint8_t  phase = phase0;
    sh_num_t be1 = st->bell.ic1, be2 = st->bell.ic2;
    sh_num_t bb1 = SH_ZERO, bb2 = SH_ZERO;
    if (st_b) { bb1 = st_b->bell.ic1; bb2 = st_b->bell.ic2; }
    const uint8_t decim = c->decim;

    for (uint32_t i = 0; i < n; i++) {
        sh_num_t x = sh_input_limit(sh_from_pipe(pair ? (sh_half(buf[i]) + sh_half(buf_b[i])) : buf[i]));
        sh_num_t v1;

        // Full-rate anti-alias lowpass (also the top of band 2), then one
        // low-rate step every `decim` samples with linear interpolation
        // between steps.  The sub lags the dry signal by one low-rate period.
        sh_num_t y = sh_svf(&c->aa, &aa1, &aa2, x, &v1);
        if (phase == 0) {
            sh_num_t nxt = subharm_low_rate_tick(c, st, y);
            step = sh_mul(nxt - cur, c->inv_decim);
        }
        if (++phase >= decim) phase = 0;
        cur += step;

        sh_num_t out = sh_to_pipe(cur);
        if (solo) {
            buf[i] = out;
            if (pair) buf_b[i] = out;
            continue;
        }
        sh_num_t acc = sh_add_sat(buf[i], out);
        if (bell) {
            (void)sh_svf(&c->bell, &be1, &be2, acc, &v1);
            acc = sh_add_sat(acc, sh_mul(c->bell_m1, v1));
        }
        buf[i] = acc;
        if (pair) {
            sh_num_t accb = sh_add_sat(buf_b[i], out);
            if (bell) {
                (void)sh_svf(&c->bell, &bb1, &bb2, accb, &v1);
                accb = sh_add_sat(accb, sh_mul(c->bell_m1, v1));
            }
            buf_b[i] = accb;
        }
    }

    st->aa.ic1 = aa1; st->aa.ic2 = aa2;
    st->cur = cur; st->step = step;
    st->bell.ic1 = be1; st->bell.ic2 = be2;
    if (st_b) {
        // Mirror A's state into B so an unlink continues both dividers from
        // the same parity instead of restarting B's against a running A.
        SubharmSvfState bell_b = st_b->bell;
        *st_b = *st;
        st_b->bell = bell_b;
        st_b->bell.ic1 = bb1; st_b->bell.ic2 = bb2;
    }
}

static inline bool sh_output_eligible(const SubharmCoeffs *c, uint16_t mask, int out) {
    return c && ((mask >> out) & 1u)
        && matrix_mixer.outputs[out].enabled
        && !matrix_mixer.outputs[out].mute
        && !(siggen_raw_mask & (1u << out));
}

// A fresh output whose pair partner is running takes the partner's state
// (bell aside), so both dividers continue with the same parity.  Two
// independently restarted dividers on one program land anti-phase half the
// time, and a zero-crossing divider never re-synchronizes on its own.
static DSP_TIME_CRITICAL __attribute__((noinline)) void sh_seed_from_partner(int out) {
    if (out >= 2 * NUM_SPDIF_INSTANCES) return;
    const SubharmOutputState *p = &subharm_output_state[out ^ 1];
    SubharmOutputState *st = &subharm_output_state[out];
    if (st->active || !p->active) return;
    SubharmSvfState bell = st->bell;
    *st = *p;
    st->bell = bell;
}

DSP_TIME_CRITICAL
void subharm_process_outputs(const SubharmCoeffs *c, uint16_t mask, uint8_t flags,
                             uint8_t phase0, int first_out, int last_out,
                             sh_num_t (*buf_out)[AUDIO_BUFFER_SAMPLES], uint32_t n) {
    const bool solo = (flags & SUBHARM_FLAG_SOLO) != 0;
    const bool link = (flags & SUBHARM_FLAG_LINK) != 0;
    for (int out = first_out; out <= last_out; out++) {
        const bool ok = sh_output_eligible(c, mask, out);
        // Linked S/PDIF pair: even output + its partner, both eligible, both
        // inside this core's range.  The PDM output is never part of a pair.
        if (link && ok && (out & 1) == 0 && out + 1 <= last_out
            && out + 1 < 2 * NUM_SPDIF_INSTANCES
            && sh_output_eligible(c, mask, out + 1)) {
            sh_seed_from_partner(out);
            subharm_process_block(c, &subharm_output_state[out], &subharm_output_state[out + 1],
                                  buf_out[out], buf_out[out + 1], n, phase0, solo);
            out++;
            continue;
        }
        if (ok) {
            sh_seed_from_partner(out);
            subharm_process_block(c, &subharm_output_state[out], NULL,
                                  buf_out[out], NULL, n, phase0, solo);
        } else {
            subharm_reset_output_state(&subharm_output_state[out]);
        }
    }
}

uint16_t subharm_meter_u16(uint8_t out) {
    if (out >= NUM_OUTPUT_CHANNELS) return 0;
    sh_num_t m = subharm_output_state[out].meter;
#if PICO_RP2350
    if (m < 0.0f) m = 0.0f;
    return (uint16_t)(fminf(1.0f, m) * 32767.0f);
#else
    if (m < 0) m = 0;
    if (m >= (1 << FILTER_SHIFT)) return 32767;
    return (uint16_t)(m >> (FILTER_SHIFT - 15));
#endif
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
#define SH_SCAN_POINTS 37            // 16 Hz .. 362 Hz inclusive

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
        sh_level_gain(config->low_db), sh_level_gain(config->high_db), sh_level_gain(config->top_db)
    };
    const float amp = sh_boost_amp(config->boost_db);
    float ceil_db = clampf(config->ceiling_db, SUBHARM_CEILING_MIN, SUBHARM_CEILING_MAX);
    const float ceil_lin = ceil_db < 0.0f ? powf(10.0f, ceil_db / 20.0f) : 0.0f;

    // Per scan point: direct tone through the bell, and the sub sum with and
    // without the bell (amplitude sums = worst-case phase).
    float direct[SH_SCAN_POINTS], sub_flat[SH_SCAN_POINTS], sub_bell[SH_SCAN_POINTS];
    float f = SH_SCAN_F0;
    for (int i = 0; i < SH_SCAN_POINTS; i++) {
        float pre = sh_hp2_mag(f / SUBHARM_BAND_LO_HZ) * sh_lp2_mag(f / SUBHARM_BAND_HI_HZ);
        float lo2 = pre * sh_lp2_mag(f / SUBHARM_BAND_MID2_HZ);
        const float band_mag[SUBHARM_NUM_BANDS] = {
            lo2 * sh_lp2_mag(f / SUBHARM_BAND_MID1_HZ),
            lo2 * sh_hp2_mag(f / SUBHARM_BAND_MID1_HZ),
            pre * sh_hp2_mag(f / SUBHARM_BAND_MID2_HZ),
        };
        float flat_sum = 0.0f, bell_sum = 0.0f;
        for (int b = 0; b < SUBHARM_NUM_BANDS; b++) {
            if (gain[b] <= 0.0f) continue;
            float flat = 0.0f, bell = 0.0f;
            for (int h = 0; h < SH_SCAN_HARMONICS; h++) {
                float fn = (float)(2 * h + 1) * 0.5f * f;
                float m = sh_div_harm[h] * sh_lp2_mag(fn / sh_sub_lp_hz[b]);
                flat += m;
                bell += m * sh_bell_mag(fn / SUBHARM_BOOST_HZ, amp);
            }
            flat_sum += gain[b] * band_mag[b] * flat;
            bell_sum += gain[b] * band_mag[b] * bell;
        }
        direct[i] = sh_bell_mag(f / SUBHARM_BOOST_HZ, amp);
        sub_flat[i] = flat_sum;
        sub_bell[i] = bell_sum;
        f *= SH_SCAN_STEP;
    }

    // Worst-case peak for a tone at input level g (full scale = 1).  The
    // ceiling is absolute, so it only bites once g * sub_flat exceeds it.
    float peak_max = 1.0f;
    for (int i = 0; i < SH_SCAN_POINTS; i++) {
        float p = direct[i] + sub_bell[i];
        if (p > peak_max) peak_max = p;
    }
    if (ceil_lin <= 0.0f) return 20.0f * log10f(peak_max);

    // With a ceiling the answer is the smallest headroom H such that an input
    // at -H dBFS cannot exceed full scale: bisect on g = 10^(-H/20), which is
    // monotone in g.  Without the ceiling term the bound above is the limit.
    float g_lo = 1.0f / peak_max, g_hi = 1.0f;
    for (int it = 0; it < 24; it++) {
        float g = 0.5f * (g_lo + g_hi);
        float worst = 0.0f;
        for (int i = 0; i < SH_SCAN_POINTS; i++) {
            float sub = g * sub_bell[i];
            if (g * sub_flat[i] > ceil_lin) sub = ceil_lin * sub_bell[i] / sub_flat[i];
            float p = g * direct[i] + sub;
            if (p > worst) worst = p;
        }
        if (worst > 1.0f) g_hi = g; else g_lo = g;
    }
    return -20.0f * log10f(g_lo);
}
