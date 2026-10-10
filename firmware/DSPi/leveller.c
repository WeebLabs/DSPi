/*
 * leveller.c; Volume Leveller (Dynamic Range Compressor)
 *
 * Implements a feedforward, channel-linked, single-band RMS compressor over
 * the active input channels.  See leveller.h for the multichannel model
 * (detector_mask / apply_mask) and coefficient conventions.
 *
 * Signal flow per block:
 *   1. Per-sample: update RMS envelopes for detector-mask channels
 *   2. Per-block:  link = loudest envelope; compute gain via soft-knee curve
 *   3. Per-block:  smooth gain with asymmetric attack/release
 *   4. Per-sample: lookahead delay on ALL active channels (if enabled),
 *      interpolated gain + safety limiter on apply-mask channels
 *
 * Safety limiter design and its no-overshoot argument: "Safety limiter" in
 * the Volume Leveller section of Documentation/current_architecture.md.
 *
 * Alignment invariant: with lookahead on, every active input channel passes
 * through its ring (same depth), so applied and non-applied channels stay
 * sample-aligned and mask changes never cause a time shift.
 */

#include <math.h>
#include <string.h>
#include "leveller.h"
#include "dsp_pipeline.h"

// ---------------------------------------------------------------------------
// Speed preset tables: {attack_sec, release_sec, rms_window_sec}
// ---------------------------------------------------------------------------

static const float speed_presets[LEVELLER_SPEED_COUNT][3] = {
    /* Slow   */ { 0.100f, 2.000f, 0.400f },   // Music, orchestral
    /* Medium */ { 0.050f, 1.000f, 0.200f },   // General purpose
    /* Fast   */ { 0.020f, 0.500f, 0.100f },   // Speech, dialogue
};

// ---------------------------------------------------------------------------
// Coefficient Computation
// ---------------------------------------------------------------------------

// Compute one-pole retention coefficient for a given time constant.
// Form A: env = alpha * env + (1-alpha) * x
// alpha near 1.0 = slow, alpha near 0.0 = fast.
// T is the 0%-to-90% step response time.
static float compute_alpha(float sample_rate, float time_sec) {
    if (time_sec <= 0.0f || sample_rate <= 0.0f) return 0.0f;
    return expf(-logf(10.0f) / (sample_rate * time_sec));
}

void leveller_compute_coefficients(LevellerCoeffs *out,
                                   const LevellerConfig *cfg,
                                   float sample_rate) {
    if (sample_rate < 1.0f) sample_rate = 48000.0f;

    // Clamp speed to valid range
    uint8_t spd = cfg->speed;
    if (spd >= LEVELLER_SPEED_COUNT) spd = LEVELLER_SPEED_MEDIUM;

    // Time constants from speed preset
    float attack_sec  = speed_presets[spd][0];
    float release_sec = speed_presets[spd][1];
    float rms_sec     = speed_presets[spd][2];

    // One-pole retention coefficients (Form A)
    out->alpha_rms     = compute_alpha(sample_rate, rms_sec);
    out->alpha_attack  = compute_alpha(sample_rate, attack_sec);
    out->alpha_release = compute_alpha(sample_rate, release_sec);

    // Fixed compression curve parameters
    out->threshold_db      = LEVELLER_THRESHOLD_DB;
    out->knee_width_db     = LEVELLER_KNEE_WIDTH_DB;
    // Gate threshold from config (user-configurable)
    float gate = cfg->gate_threshold_db;
    if (gate < LEVELLER_GATE_MIN) gate = LEVELLER_GATE_MIN;
    if (gate > LEVELLER_GATE_MAX) gate = LEVELLER_GATE_MAX;
    out->gate_threshold_db = gate;

    // Clamp amount
    float amount = cfg->amount;
    if (amount < LEVELLER_AMOUNT_MIN) amount = LEVELLER_AMOUNT_MIN;
    if (amount > LEVELLER_AMOUNT_MAX) amount = LEVELLER_AMOUNT_MAX;

    // Ratio: 1:1 at amount=0%, 20:1 at amount=100%
    float norm = amount / 100.0f;
    out->ratio = 1.0f + norm * 19.0f;

    // Max gain ceiling (clamp from config)
    float max_g = cfg->max_gain_db;
    if (max_g < LEVELLER_MAX_GAIN_MIN) max_g = LEVELLER_MAX_GAIN_MIN;
    if (max_g > LEVELLER_MAX_GAIN_MAX) max_g = LEVELLER_MAX_GAIN_MAX;
#if !PICO_RP2350
    // Q28 linear gain tops out just below 8.0 (18 dB); higher settings
    // would overflow the fixed-point conversion in the apply loop.
    if (max_g > 18.0f) max_g = 18.0f;
#endif
    out->max_gain_db = max_g;

    // No makeup gain; upward compression provides the boost directly.
    // Content below the threshold is boosted, content above is untouched.
    out->makeup_db = 0.0f;

    // Safety limiter release and hold, per limiter block
    float blk_sec = (float)LEVELLER_LIM_BLOCK / sample_rate;
    out->lim_release = powf(10.0f, LEVELLER_LIM_RELEASE_DBPS * blk_sec / 20.0f);
    out->lim_release_q28 = (int32_t)(out->lim_release * (float)(1 << FILTER_SHIFT) + 0.5f);
    out->lim_hold_blocks = (uint16_t)ceilf(LEVELLER_LIM_HOLD_SEC / blk_sec);
}

// ---------------------------------------------------------------------------
// State Reset
// ---------------------------------------------------------------------------

// Limiter gain ceiling: above any boost the leveller can apply, so a cap at
// LIM_GMAX never binds.
#if PICO_RP2350
#define LIM_GMAX   64.0f
#else
#define LIM_GMAX   INT32_MAX                  // Q28, just under 8.0
#define LIM_UNITY  (1 << FILTER_SHIFT)
#define LIM_CEIL   ((uint32_t)(LEVELLER_LIMITER_CEIL * (float)(1 << FILTER_SHIFT)))
#endif

void leveller_reset_state(LevellerState *state) {
    memset(state, 0, sizeof(LevellerState));
#if PICO_RP2350
    state->gain_linear = 1.0f;
    state->gain_prev_linear = 1.0f;
#else
    state->gain_q28 = (1 << FILTER_SHIFT);       // 1.0 in Q28
    state->gain_prev_q28 = (1 << FILTER_SHIFT);
#endif
    state->gain_smooth_db = 0.0f;  // 0 dB = unity

    // Limiter idle; the sentinel apply set forces a cap rescan on first use.
    for (int b = 0; b < LEVELLER_LIM_BLOCKS; b++) state->lim_cap[b] = LIM_GMAX;
    state->lim_l0 = state->lim_end = state->lim_inst = state->lim_env = LIM_GMAX;
    state->lim_app_prev = 0xFFFF;
}

// ---------------------------------------------------------------------------
// Upward Compression Gain Computer
//
// Returns the gain boost in dB for a given input level.
// Boosts content BELOW the threshold, leaves content ABOVE untouched.
// Uses a quadratic soft knee around the threshold for smooth transition.
//
//   Above knee:  no boost (loud content untouched)
//   Within knee: quadratic blend from full boost to 0
//   Below knee:  full upward compression = (threshold - x) * (1 - 1/R)
//
// This is the inverse of a traditional downward compressor: instead of
// pushing loud content down, it lifts quiet content up. No makeup gain
// needed; the boost IS the compression. Loud content passes through
// at unity, so the limiter rarely engages.
// ---------------------------------------------------------------------------

static inline float gain_computer(float x_db, float threshold, float ratio,
                                  float knee_width) {
    float half_knee = knee_width * 0.5f;

    if (x_db > (threshold + half_knee)) {
        // Above knee: no boost (leave loud content alone)
        return 0.0f;
    } else if (x_db >= (threshold - half_knee)) {
        // Within soft knee: quadratic transition from boost to unity
        float d = threshold + half_knee - x_db;
        return (1.0f - 1.0f / ratio) * d * d / (2.0f * knee_width);
    } else {
        // Below knee: full upward compression
        return (threshold - x_db) * (1.0f - 1.0f / ratio);
    }
}

// ---------------------------------------------------------------------------
// RP2350 Float Block Processing
// ---------------------------------------------------------------------------

#if PICO_RP2350

// Largest boost that keeps a peak of pk at or under the ceiling, never below
// unity: content already over the ceiling passes untouched.
DSP_TIME_CRITICAL __attribute__((noinline)) static float lim_cap_of(float pk) {
    const float ceil = LEVELLER_LIMITER_CEIL;
    if (pk * LIM_GMAX <= ceil) return LIM_GMAX;
    if (pk >= ceil) return 1.0f;
    return ceil / pk;
}

// Runs after the last sample of a limiter block.  b_done is the ring block
// just played (and just refilled); lev is the leveller gain for the next sample.
static inline __attribute__((always_inline))
void lim_boundary(LevellerState *s, const LevellerCoeffs *c, bool use_la,
                  float lev, uint32_t b_done) {
    float c_out;
    if (use_la) {
        c_out = s->lim_cap[b_done];
        s->lim_cap[b_done] = lim_cap_of(s->lim_pin);
    } else {
        c_out = lim_cap_of(s->lim_pout);
    }
    s->lim_pin = s->lim_pout = 0.0f;

    // Hold the lowest recent cap, then release; a new cap at or under the
    // held one re-arms the hold, so steady peaks never pump the gain.
    float e = s->lim_env;
    if (s->lim_hold == 0) e = fminf(e * c->lim_release, LIM_GMAX);
    if (c_out <= e) {
        e = c_out;
        s->lim_hold = c->lim_hold_blocks;
    } else if (s->lim_hold) {
        s->lim_hold--;
    }
    s->lim_env = e;

    // Every cap in the ring binds from the first boundary after it is
    // measured, so a peak is ramped for over one block before it plays.
    float l0 = fminf(fminf(s->lim_end, s->lim_inst), lev);
    float l1 = e;
    if (use_la) {
        for (uint32_t b = 0; b < LEVELLER_LIM_BLOCKS; b++) l1 = fminf(l1, s->lim_cap[b]);
    }
    s->lim_inst = LIM_GMAX;
    s->lim_l0 = l0;
    s->lim_end = l1;
    s->lim_step = (l1 - l0) * (1.0f / LEVELLER_LIM_BLOCK);
}

// Applied set changed with lookahead on: the ring holds samples the caps
// never measured.  Re-measure them and drop the ramp to the lowest cap now.
DSP_TIME_CRITICAL
static void lim_rescan(LevellerState *s, uint32_t app_mask, uint32_t pos) {
    float lo = LIM_GMAX;
    for (uint32_t b = 0; b < LEVELLER_LIM_BLOCKS; b++) {
        float pk = 0.0f;
        for (uint32_t k = 0; k < NUM_INPUT_CHANNELS; k++) {
            if (!(app_mask & (1u << k))) continue;
            const float *r = &s->lookahead_buf[k][b * LEVELLER_LIM_BLOCK];
            for (uint32_t i = 0; i < LEVELLER_LIM_BLOCK; i++) pk = fmaxf(pk, fabsf(r[i]));
        }
        s->lim_cap[b] = lim_cap_of(pk);
        lo = fminf(lo, s->lim_cap[b]);
        // The block being refilled: its cap also covers the samples already in.
        if (b == s->la_write_idx / LEVELLER_LIM_BLOCK) s->lim_pin = pk;
    }
    float cur = fminf(s->lim_l0 + s->lim_step * (float)pos, lo);
    s->lim_l0 = s->lim_end = cur;
    s->lim_step = 0.0f;
}

DSP_TIME_CRITICAL
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            float *const *bufs, uint32_t n_bufs,
                            uint32_t count) {
    if (count == 0 || n_bufs == 0) return;
    if (n_bufs > NUM_INPUT_CHANNELS) n_bufs = NUM_INPUT_CHANNELS;

    // Inputs that just became active carry stale ring/envelope content from
    // the last time this many channels streamed; clear before use.
    if (n_bufs != state->active_prev) {
        for (uint32_t k = state->active_prev; k < n_bufs; k++) {
            memset(state->lookahead_buf[k], 0, sizeof(state->lookahead_buf[k]));
            state->env_sq[k] = 0.0f;
        }
        state->active_prev = (uint8_t)n_bufs;
    }

    // Snapshot masks once per block, gated to the active channel set
    const uint32_t active_mask = (1u << n_bufs) - 1u;
    const uint32_t det_mask = (uint32_t)cfg->detector_mask & active_mask;
    const uint32_t app_mask = (uint32_t)cfg->apply_mask & active_mask;

    // ---- Per-sample: update RMS envelopes (detector channels only) ----
    const float a_rms = coeffs->alpha_rms;
    const float one_minus_a_rms = 1.0f - a_rms;
    float link_sq = 0.0f;   // Loudest detector envelope (linked level)

    for (uint32_t k = 0; k < n_bufs; k++) {
        if (!(det_mask & (1u << k))) {
            // Outside the detector set: drop the envelope so a later mask
            // re-enable starts fresh instead of pumping from a stale level.
            state->env_sq[k] = 0.0f;
            continue;
        }
        const float *s = bufs[k];
        float env = state->env_sq[k];
        for (uint32_t i = 0; i < count; i++) {
            float x = s[i];
            env = a_rms * env + one_minus_a_rms * (x * x);
        }
        // Prevent denormals in silent passages
        if (env < 1e-30f) env = 0.0f;
        state->env_sq[k] = env;
        if (env > link_sq) link_sq = env;
    }

    // ---- Per-block: compute target gain from the linked level ----
    float rms_db = 10.0f * log10f(link_sq + 1e-30f);

    float gc_db;
    if (rms_db < coeffs->gate_threshold_db) {
        // Below silence gate: unity gain (no boost, prevents noise pumping)
        gc_db = 0.0f;
    } else {
        // Soft-knee compression curve
        gc_db = gain_computer(rms_db, coeffs->threshold_db,
                              coeffs->ratio, coeffs->knee_width_db);
        gc_db += coeffs->makeup_db;

        // Clamp to max gain ceiling
        if (gc_db > coeffs->max_gain_db) gc_db = coeffs->max_gain_db;
    }

    // ---- Per-block: asymmetric gain smoothing ----
    // alpha_attack/release are per-SAMPLE coefficients. Since we apply the
    // smoother once per BLOCK, raise to the block size to get the correct
    // per-block alpha. Without this, time constants are block_size× too slow.
    float alpha_sample = (gc_db < state->gain_smooth_db) ? coeffs->alpha_attack
                                                          : coeffs->alpha_release;
    float alpha = powf(alpha_sample, (float)count);
    state->gain_smooth_db = alpha * state->gain_smooth_db
                          + (1.0f - alpha) * gc_db;

    // Save previous gain for interpolation, compute new linear gain
    state->gain_prev_linear = state->gain_linear;
    state->gain_linear = powf(10.0f, state->gain_smooth_db / 20.0f);

    // Snapshot the applied-channel buffers once (not per sample)
    float *ap[NUM_INPUT_CHANNELS];
    uint32_t na = 0;
    for (uint32_t k = 0; k < n_bufs; k++) {
        if (app_mask & (1u << k)) ap[na++] = bufs[k];
    }

    // With lookahead the block phase follows the ring, so a lookahead toggle
    // seen before its state reset can never walk la_idx off the ring.  The
    // toggle also forces a rescan: the ring then holds unmeasured samples.
    bool use_la = cfg->lookahead;
    uint32_t la_idx = state->la_write_idx;
    uint32_t pos = use_la ? la_idx % LEVELLER_LIM_BLOCK : state->lim_pos;
    uint16_t lim_key = (uint16_t)(app_mask | (use_la ? 0x100u : 0u));
    if (lim_key != state->lim_app_prev) {
        state->lim_app_prev = lim_key;
        if (use_la) lim_rescan(state, app_mask, pos);
    }
    if (!use_la && na == 0) return;   // Gain state updated; nothing to touch

    // ---- Per-sample: lookahead delay + interpolated gain + limiter ----
    // Applied gain = min(leveller gain, limiter cap).  The cap only ever
    // lowers the boost and never goes below unity (see file header).
    float gain_prev = state->gain_prev_linear;
    float gain_cur  = state->gain_linear;
    float gain, gain_step;

    if (count == 1) {
        gain = gain_cur;
        gain_step = 0.0f;
    } else {
        gain_step = (gain_cur - gain_prev) / (float)(count - 1);
        gain = gain_prev;
    }

    const float ceil = LEVELLER_LIMITER_CEIL;
    uint32_t i = 0;

    while (i < count) {
        uint32_t seg = LEVELLER_LIM_BLOCK - pos;
        if (seg > count - i) seg = count - i;
        const float l0 = state->lim_l0, lstep = state->lim_step;
        // gv holds outgoing peaks, then the applied gain.  With lookahead the
        // peaks stay zero: the caps already cover every sample that plays.
        float gv[LEVELLER_LIM_BLOCK];
        for (uint32_t n = 0; n < seg; n++) gv[n] = 0.0f;

        if (use_la) {
            // Every active channel goes through its ring so applied and
            // non-applied channels stay sample-aligned (see header).  A
            // segment never crosses a block end, so it never wraps the ring.
            float pin = state->lim_pin;
            for (uint32_t j = 0; j < na; j++) {
                const float *x = &ap[j][i];
                for (uint32_t n = 0; n < seg; n++) pin = fmaxf(pin, fabsf(x[n]));
            }
            state->lim_pin = pin;
            for (uint32_t k = 0; k < n_bufs; k++) {
                float *ring = &state->lookahead_buf[k][la_idx];
                float *x = &bufs[k][i];
                for (uint32_t n = 0; n < seg; n++) {
                    float d = ring[n];
                    ring[n] = x[n];
                    x[n] = d;
                }
            }
            la_idx += seg;
        } else {
            for (uint32_t j = 0; j < na; j++) {
                const float *x = &ap[j][i];
                for (uint32_t n = 0; n < seg; n++) gv[n] = fmaxf(gv[n], fabsf(x[n]));
            }
        }

        // Gain = min(leveller, cap ramp, instant cap).  The instant cap is the
        // no-lookahead attack: it drops at the peak and holds for the block.
        float inst = state->lim_inst, pout = state->lim_pout;
        for (uint32_t n = 0; n < seg; n++) {
            float pk = gv[n];
            float g = fminf(gain + gain_step * (float)n,
                            fminf(l0 + lstep * (float)(pos + n), inst));
            pout = fmaxf(pout, pk);
            if (g > 1.0f && pk * g > ceil) {
                float h = lim_cap_of(pk);
                inst = fminf(inst, h);
                g = fminf(g, h);
            }
            gv[n] = g;
        }
        state->lim_inst = inst;
        state->lim_pout = pout;

        for (uint32_t j = 0; j < na; j++) {
            float *x = &ap[j][i];
            for (uint32_t n = 0; n < seg; n++) x[n] *= gv[n];
        }
        gain += gain_step * (float)seg;
        i += seg;

        pos += seg;
        if (pos == LEVELLER_LIM_BLOCK) {
            pos = 0;
            uint32_t b_done = 0;
            if (use_la) {
                b_done = la_idx / LEVELLER_LIM_BLOCK - 1;
                if (la_idx >= LEVELLER_LOOKAHEAD_SAMPLES) la_idx = 0;
            }
            // At a packet end the next sample starts at gain_cur, not one step past it
            lim_boundary(state, coeffs, use_la, (i == count) ? gain_cur : gain, b_done);
        }
    }

    state->lim_pos = (uint8_t)pos;
    if (use_la) state->la_write_idx = la_idx;
}

#else  // RP2040

// ---------------------------------------------------------------------------
// RP2040 Q28 Fixed-Point Block Processing
//
// Envelope update and gain application use Q28 arithmetic via fast_mul_q28().
// Gain computation (log/exp/soft knee) uses float; it runs once per block
// (~1ms), so the cost of the Pico SDK ROM float routines is acceptable.
// RP2040 has exactly 2 input channels, so the masked loops stay 2-wide.
// ---------------------------------------------------------------------------

static inline __attribute__((always_inline)) uint32_t lim_abs(int32_t v) {
    return v < 0 ? 0u - (uint32_t)v : (uint32_t)v;
}

// Q28 twin of the RP2350 lim_cap_of.  Its float divide runs once per block,
// or per sample only while a no-lookahead attack is still deepening.
DSP_TIME_CRITICAL __attribute__((noinline)) static int32_t lim_cap_of(uint32_t pk) {
    if (pk >= LIM_CEIL) return LIM_UNITY;
    if (pk <= LIM_CEIL / 8) return LIM_GMAX;   // Cap of 8.0 or more: no Q28 room
    float h = (float)LIM_CEIL / (float)pk;
    if (h >= 7.9999f) return LIM_GMAX;
    return (int32_t)(h * (float)LIM_UNITY);    // Truncates: never over the cap
}

#define LIM_INV_BLOCK_Q28  ((int32_t)((1 << FILTER_SHIFT) / LEVELLER_LIM_BLOCK))

// See the RP2350 lim_boundary.  The ramp step rounds toward zero: a floored
// falling step would carry the ramp below unity and attenuate.
static inline __attribute__((always_inline))
void lim_boundary(LevellerState *s, const LevellerCoeffs *c, bool use_la,
                  int32_t lev, uint32_t b_done) {
    int32_t c_out;
    if (use_la) {
        c_out = s->lim_cap[b_done];
        s->lim_cap[b_done] = lim_cap_of(s->lim_pin);
    } else {
        c_out = lim_cap_of(s->lim_pout);
    }
    s->lim_pin = s->lim_pout = 0;

    int32_t e = s->lim_env;
    if (s->lim_hold == 0)   // The headroom test keeps e * release inside int32
        e = (e > LIM_GMAX - (LIM_GMAX >> 5)) ? LIM_GMAX : fast_mul_q28(e, c->lim_release_q28);
    if (c_out <= e) {
        e = c_out;
        s->lim_hold = c->lim_hold_blocks;
    } else if (s->lim_hold) {
        s->lim_hold--;
    }
    s->lim_env = e;

    int32_t l0 = s->lim_end;
    if (s->lim_inst < l0) l0 = s->lim_inst;
    if (lev < l0) l0 = lev;
    int32_t l1 = e;
    if (use_la) {
        for (uint32_t b = 0; b < LEVELLER_LIM_BLOCKS; b++)
            if (s->lim_cap[b] < l1) l1 = s->lim_cap[b];
    }
    s->lim_inst = LIM_GMAX;
    s->lim_l0 = l0;
    s->lim_end = l1;
    int32_t d = l1 - l0;
    s->lim_step = (d >= 0) ? fast_mul_q28(d, LIM_INV_BLOCK_Q28) : -fast_mul_q28(-d, LIM_INV_BLOCK_Q28);
}

// See the RP2350 lim_rescan.
DSP_TIME_CRITICAL
static void lim_rescan(LevellerState *s, bool app_l, bool app_r, uint32_t pos) {
    int32_t lo = LIM_GMAX;
    for (uint32_t b = 0; b < LEVELLER_LIM_BLOCKS; b++) {
        uint32_t pk = 0;
        for (uint32_t k = 0; k < 2; k++) {
            if (!(k == 0 ? app_l : app_r)) continue;
            const int32_t *r = &s->lookahead_buf[k][b * LEVELLER_LIM_BLOCK];
            for (uint32_t i = 0; i < LEVELLER_LIM_BLOCK; i++) {
                uint32_t a = lim_abs(r[i]);
                if (a > pk) pk = a;
            }
        }
        s->lim_cap[b] = lim_cap_of(pk);
        if (s->lim_cap[b] < lo) lo = s->lim_cap[b];
        if (b == s->la_write_idx / LEVELLER_LIM_BLOCK) s->lim_pin = pk;
    }
    int32_t cur = s->lim_l0 + s->lim_step * (int32_t)pos;
    if (lo < cur) cur = lo;
    s->lim_l0 = s->lim_end = cur;
    s->lim_step = 0;
}

DSP_TIME_CRITICAL
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            int32_t *const *bufs, uint32_t n_bufs,
                            uint32_t count) {
    if (count == 0 || n_bufs == 0) return;
    if (n_bufs > NUM_INPUT_CHANNELS) n_bufs = NUM_INPUT_CHANNELS;

    int32_t *buf_l = bufs[0];
    int32_t *buf_r = (n_bufs > 1) ? bufs[1] : NULL;

    const uint32_t active_mask = (1u << n_bufs) - 1u;
    const uint32_t det_mask = (uint32_t)cfg->detector_mask & active_mask;
    const uint32_t app_mask = (uint32_t)cfg->apply_mask & active_mask;
    const bool det_l = (det_mask & 1u) != 0;
    const bool det_r = (det_mask & 2u) != 0 && buf_r;
    const bool app_l = (app_mask & 1u) != 0;
    const bool app_r = (app_mask & 2u) != 0 && buf_r;

    // ---- Per-sample: update RMS envelopes (Q28, detector channels only) ----

    // Pre-compute (1 - alpha_rms) in Q28 for the envelope update.
    // alpha_rms is a float in [0,1]; convert both coefficients to Q28.
    int32_t a_rms_q28 = (int32_t)(coeffs->alpha_rms * (float)(1 << FILTER_SHIFT));
    int32_t one_minus_a_q28 = (1 << FILTER_SHIFT) - a_rms_q28;

    int32_t env_l = state->env_sq[0];
    int32_t env_r = state->env_sq[1];

    // Channels outside the detector set drop their envelope so a later
    // mask re-enable starts fresh instead of pumping from a stale level.
    if (det_l) {
        for (uint32_t i = 0; i < count; i++) {
            int32_t sl = buf_l[i];
            int32_t sq = fast_mul_q28(sl, sl);
            env_l = fast_mul_q28(a_rms_q28, env_l) + fast_mul_q28(one_minus_a_q28, sq);
        }
        state->env_sq[0] = env_l;
    } else {
        env_l = 0;
        state->env_sq[0] = 0;
    }
    if (det_r) {
        for (uint32_t i = 0; i < count; i++) {
            int32_t sr = buf_r[i];
            int32_t sq = fast_mul_q28(sr, sr);
            env_r = fast_mul_q28(a_rms_q28, env_r) + fast_mul_q28(one_minus_a_q28, sq);
        }
        state->env_sq[1] = env_r;
    } else {
        env_r = 0;
        state->env_sq[1] = 0;
    }

    // ---- Per-block: compute target gain from the linked level (float) ----

    const float inv_q28 = 1.0f / (float)(1 << FILTER_SHIFT);
    float link_sq = 0.0f;
    if (det_l) {
        float e = (float)env_l * inv_q28;
        if (e > link_sq) link_sq = e;
    }
    if (det_r) {
        float e = (float)env_r * inv_q28;
        if (e > link_sq) link_sq = e;
    }
    float rms_db = 10.0f * log10f(link_sq + 1e-30f);

    float gc_db;
    if (rms_db < coeffs->gate_threshold_db) {
        gc_db = 0.0f;
    } else {
        gc_db = gain_computer(rms_db, coeffs->threshold_db,
                              coeffs->ratio, coeffs->knee_width_db);
        gc_db += coeffs->makeup_db;
        if (gc_db > coeffs->max_gain_db) gc_db = coeffs->max_gain_db;
    }

    // Asymmetric gain smoothing (float)
    // Raise per-sample alpha to block size for correct per-block time constant
    float alpha_sample = (gc_db < state->gain_smooth_db) ? coeffs->alpha_attack
                                                          : coeffs->alpha_release;
    float alpha = powf(alpha_sample, (float)count);
    state->gain_smooth_db = alpha * state->gain_smooth_db
                          + (1.0f - alpha) * gc_db;

    // Convert smoothed gain to Q28 linear; clamp below 8.0 so the
    // conversion cannot overflow (coeffs already cap max gain at 18 dB)
    float gain_linear = powf(10.0f, state->gain_smooth_db / 20.0f);
    if (gain_linear > 7.99f) gain_linear = 7.99f;
    state->gain_prev_q28 = state->gain_q28;
    state->gain_q28 = (int32_t)(gain_linear * (float)(1 << FILTER_SHIFT));

    // Block phase and rescan key follow the RP2350 path
    bool use_la = cfg->lookahead;
    uint32_t la_idx = state->la_write_idx;
    uint32_t pos = use_la ? la_idx % LEVELLER_LIM_BLOCK : state->lim_pos;
    uint16_t lim_key = (uint16_t)(app_mask | (use_la ? 0x100u : 0u));
    if (lim_key != state->lim_app_prev) {
        state->lim_app_prev = lim_key;
        if (use_la) lim_rescan(state, app_l, app_r, pos);
    }
    if (!use_la && !app_l && !app_r) return;   // Gain state updated; no apply

    // ---- Per-sample: lookahead delay + interpolated gain + limiter ----
    // Applied gain = min(leveller gain, limiter cap); see the RP2350 path.
    int32_t gain, gain_step;
    if (count == 1) {
        gain = state->gain_q28;
        gain_step = 0;
    } else {
        gain = state->gain_prev_q28;
        gain_step = (state->gain_q28 - state->gain_prev_q28) / (int32_t)(count - 1);
    }

    uint32_t i = 0;
    while (i < count) {
        uint32_t seg = LEVELLER_LIM_BLOCK - pos;
        if (seg > count - i) seg = count - i;
        int32_t lim = state->lim_l0 + state->lim_step * (int32_t)pos;
        const int32_t lstep = state->lim_step;
        int32_t *xl = &buf_l[i], *xr = buf_r ? &buf_r[i] : NULL;

        // gv holds outgoing peaks, then the applied gain (see RP2350).
        int32_t gv[LEVELLER_LIM_BLOCK];
        for (uint32_t n = 0; n < seg; n++) gv[n] = 0;

        if (use_la) {
            // Both channels always pass through their rings so applied and
            // non-applied channels stay sample-aligned.
            uint32_t pin = state->lim_pin;
            if (app_l) for (uint32_t n = 0; n < seg; n++) { uint32_t a = lim_abs(xl[n]); if (a > pin) pin = a; }
            if (app_r) for (uint32_t n = 0; n < seg; n++) { uint32_t a = lim_abs(xr[n]); if (a > pin) pin = a; }
            state->lim_pin = pin;
            for (uint32_t k = 0; k < 2; k++) {
                int32_t *x = k ? xr : xl;
                if (!x) continue;
                int32_t *ring = &state->lookahead_buf[k][la_idx];
                for (uint32_t n = 0; n < seg; n++) {
                    int32_t d = ring[n];
                    ring[n] = x[n];
                    x[n] = d;
                }
            }
            la_idx += seg;
        } else {
            for (uint32_t k = 0; k < 2; k++) {
                const int32_t *x = k ? xr : xl;
                if (!(k ? app_r : app_l)) continue;
                for (uint32_t n = 0; n < seg; n++) {
                    uint32_t a = lim_abs(x[n]);
                    if (a > (uint32_t)gv[n]) gv[n] = (int32_t)a;
                }
            }
        }

        // Gain = min(leveller, cap ramp, instant cap); see the RP2350 path.
        int32_t inst = state->lim_inst;
        uint32_t pout = state->lim_pout;
        for (uint32_t n = 0; n < seg; n++) {
            uint32_t pk = (uint32_t)gv[n];
            int32_t g = (gain < lim) ? gain : lim;
            if (inst < g) g = inst;
            if (pk > pout) pout = pk;
            // pk < LIM_CEIL bounds the product under 8.0 * LIM_CEIL: no overflow
            if (g > LIM_UNITY && (pk >= LIM_CEIL || fast_mul_q28((int32_t)pk, g) > (int32_t)LIM_CEIL)) {
                int32_t h = lim_cap_of(pk);
                if (h < inst) inst = h;
                if (h < g) g = h;
            }
            gv[n] = g;
            gain += gain_step;
            lim += lstep;
        }
        state->lim_inst = inst;
        state->lim_pout = pout;

        if (app_l) for (uint32_t n = 0; n < seg; n++) xl[n] = fast_mul_q28(xl[n], gv[n]);
        if (app_r) for (uint32_t n = 0; n < seg; n++) xr[n] = fast_mul_q28(xr[n], gv[n]);
        i += seg;

        pos += seg;
        if (pos == LEVELLER_LIM_BLOCK) {
            pos = 0;
            uint32_t b_done = 0;
            if (use_la) {
                b_done = la_idx / LEVELLER_LIM_BLOCK - 1;
                if (la_idx >= LEVELLER_LOOKAHEAD_SAMPLES) la_idx = 0;
            }
            lim_boundary(state, coeffs, use_la, (i == count) ? state->gain_q28 : gain, b_done);
        }
    }

    state->lim_pos = (uint8_t)pos;
    if (use_la) state->la_write_idx = la_idx;
}

#endif  // PICO_RP2350
