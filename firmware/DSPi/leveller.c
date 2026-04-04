/*
 * leveller.c — Volume Leveller (Dynamic Range Compressor)
 *
 * Implements a feedforward, stereo-linked, single-band RMS compressor.
 * See leveller.h for algorithm overview and coefficient conventions.
 *
 * Signal flow per block:
 *   1. Per-sample: update RMS envelope (one-pole IIR on squared signal)
 *   2. Per-block:  compute gain from envelope via soft-knee compressor
 *   3. Per-block:  smooth gain with asymmetric attack/release
 *   4. Per-sample: apply interpolated gain, optional lookahead delay, safety limiter
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
    out->max_gain_db = max_g;

    // No makeup gain — upward compression provides the boost directly.
    // Content below the threshold is boosted, content above is untouched.
    out->makeup_db = 0.0f;

}

// ---------------------------------------------------------------------------
// State Reset
// ---------------------------------------------------------------------------

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
// needed — the boost IS the compression. Loud content passes through
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

DSP_TIME_CRITICAL
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            float *buf_l, float *buf_r,
                            uint32_t count) {
    if (count == 0) return;

    // ---- Per-sample: update RMS envelopes ----
    float env_l = state->env_sq_l;
    float env_r = state->env_sq_r;
    const float a_rms = coeffs->alpha_rms;
    const float one_minus_a_rms = 1.0f - a_rms;

    for (uint32_t i = 0; i < count; i++) {
        float sl = buf_l[i];
        float sr = buf_r[i];
        env_l = a_rms * env_l + one_minus_a_rms * (sl * sl);
        env_r = a_rms * env_r + one_minus_a_rms * (sr * sr);
    }

    // Prevent denormals in silent passages
    if (env_l < 1e-30f) env_l = 0.0f;
    if (env_r < 1e-30f) env_r = 0.0f;
    state->env_sq_l = env_l;
    state->env_sq_r = env_r;

    // ---- Per-block: compute target gain ----

    // Stereo-linked: use the louder channel
    float rms_sq = (env_l > env_r) ? env_l : env_r;
    float rms_db = 10.0f * log10f(rms_sq + 1e-30f);

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

    // ---- Per-sample: apply gain with interpolation + optional lookahead ----
    // The limiter caps the GAIN (not the output level) so the leveller never
    // creates content above the ceiling, but content already above it passes
    // through untouched. Per-sample: gain = min(leveller_gain, ceil / |input|).
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
    bool use_la = cfg->lookahead;
    uint32_t la_idx = state->la_write_idx;

    for (uint32_t i = 0; i < count; i++) {
        float out_l, out_r;

        if (use_la) {
            out_l = state->lookahead_buf[0][la_idx];
            out_r = state->lookahead_buf[1][la_idx];
            state->lookahead_buf[0][la_idx] = buf_l[i];
            state->lookahead_buf[1][la_idx] = buf_r[i];
            la_idx++;
            if (la_idx >= LEVELLER_LOOKAHEAD_SAMPLES) la_idx = 0;
        } else {
            out_l = buf_l[i];
            out_r = buf_r[i];
        }

        // Cap gain so the leveller never boosts a sample above the ceiling.
        // If the sample is already above the ceiling, gain is capped at 1.0
        // (pass-through) — existing loud content is never attenuated.
        float peak = fabsf(out_l);
        float pr = fabsf(out_r);
        if (pr > peak) peak = pr;

        float g = gain;
        if (peak > 0.0f && g > 1.0f) {
            float max_g = ceil / peak;
            if (max_g < g) g = (max_g > 1.0f) ? max_g : 1.0f;
        }

        buf_l[i] = out_l * g;
        buf_r[i] = out_r * g;
        gain += gain_step;
    }

    state->la_write_idx = la_idx;
}

#else  // RP2040

// ---------------------------------------------------------------------------
// RP2040 Q28 Fixed-Point Block Processing
//
// Envelope update and gain application use Q28 arithmetic via fast_mul_q28().
// Gain computation (log/exp/soft knee) uses float — runs once per block (~1ms),
// so the cost of the Pico SDK ROM float routines is acceptable.
// ---------------------------------------------------------------------------

DSP_TIME_CRITICAL
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            int32_t *buf_l, int32_t *buf_r,
                            uint32_t count) {
    if (count == 0) return;

    // ---- Per-sample: update RMS envelopes (Q28) ----

    // Pre-compute (1 - alpha_rms) in Q28 for the envelope update.
    // alpha_rms is a float in [0,1]; convert both coefficients to Q28.
    int32_t a_rms_q28 = (int32_t)(coeffs->alpha_rms * (float)(1 << FILTER_SHIFT));
    int32_t one_minus_a_q28 = (1 << FILTER_SHIFT) - a_rms_q28;

    int32_t env_l = state->env_sq_l;
    int32_t env_r = state->env_sq_r;

    for (uint32_t i = 0; i < count; i++) {
        int32_t sl = buf_l[i];
        int32_t sr = buf_r[i];
        int32_t sq_l = fast_mul_q28(sl, sl);
        int32_t sq_r = fast_mul_q28(sr, sr);
        env_l = fast_mul_q28(a_rms_q28, env_l) + fast_mul_q28(one_minus_a_q28, sq_l);
        env_r = fast_mul_q28(a_rms_q28, env_r) + fast_mul_q28(one_minus_a_q28, sq_r);
    }

    state->env_sq_l = env_l;
    state->env_sq_r = env_r;

    // ---- Per-block: compute target gain (float math) ----

    // Convert Q28 envelope to float for gain computation
    const float inv_q28 = 1.0f / (float)(1 << FILTER_SHIFT);
    float env_l_f = (float)env_l * inv_q28;
    float env_r_f = (float)env_r * inv_q28;
    float rms_sq = (env_l_f > env_r_f) ? env_l_f : env_r_f;
    float rms_db = 10.0f * log10f(rms_sq + 1e-30f);

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

    // Convert smoothed gain to Q28 linear
    float gain_linear = powf(10.0f, state->gain_smooth_db / 20.0f);
    state->gain_prev_q28 = state->gain_q28;
    state->gain_q28 = (int32_t)(gain_linear * (float)(1 << FILTER_SHIFT));

    // ---- Per-sample: apply gain with interpolation + optional lookahead ----
    // Limiter caps the GAIN so the leveller never boosts a sample above the
    // ceiling, but existing loud content passes through untouched.
    int32_t g_prev = state->gain_prev_q28;
    int32_t g_cur  = state->gain_q28;
    const int32_t unity_q28 = (1 << FILTER_SHIFT);
    const float ceil = LEVELLER_LIMITER_CEIL;

    bool use_la = cfg->lookahead;
    uint32_t la_idx = state->la_write_idx;

    for (uint32_t i = 0; i < count; i++) {
        int32_t gain;
        if (count == 1) {
            gain = g_cur;
        } else {
            gain = g_prev + (int32_t)(((int64_t)(g_cur - g_prev) * i) / (int32_t)(count - 1));
        }

        int32_t out_l, out_r;

        if (use_la) {
            out_l = state->lookahead_buf[0][la_idx];
            out_r = state->lookahead_buf[1][la_idx];
            state->lookahead_buf[0][la_idx] = buf_l[i];
            state->lookahead_buf[1][la_idx] = buf_r[i];
            la_idx++;
            if (la_idx >= LEVELLER_LOOKAHEAD_SAMPLES) la_idx = 0;
        } else {
            out_l = buf_l[i];
            out_r = buf_r[i];
        }

        // Cap gain so leveller never boosts above ceiling; pass-through if already loud
        if (gain > unity_q28) {
            float peak = fabsf((float)out_l * inv_q28);
            float pr = fabsf((float)out_r * inv_q28);
            if (pr > peak) peak = pr;
            if (peak > 0.0f) {
                float max_g_f = ceil / peak;
                int32_t max_g_q28 = (int32_t)(max_g_f * (float)unity_q28);
                if (max_g_q28 < gain) gain = (max_g_q28 > unity_q28) ? max_g_q28 : unity_q28;
            }
        }

        out_l = fast_mul_q28(out_l, gain);
        out_r = fast_mul_q28(out_r, gain);

        buf_l[i] = out_l;
        buf_r[i] = out_r;
    }

    state->la_write_idx = la_idx;
}

#endif  // PICO_RP2350
