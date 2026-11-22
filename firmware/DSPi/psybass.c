/*
 * Psychoacoustic Bass Enhancement
 *
 * Generates harmonics of the sub-cutoff low band so small speakers convey
 * bass they cannot physically reproduce (missing-fundamental effect).
 * See psybass.h for the signal flow and module pattern.
 *
 * Coefficient design:
 *   - Split lowpass and harmonic highpass are 2nd-order Butterworth
 *     (Q = 0.7071) at the cutoff.  RP2350 uses the TPT SVF form (the cutoff
 *     is always far below Fs/7.5, deep in SVF territory); RP2040 uses RBJ
 *     biquads scaled to Q28.
 *   - Harmonic band top is a one-pole lowpass at 4x cutoff: a gentle 6 dB/oct
 *     rolloff that mimics natural harmonic decay and avoids audible edge.
 *   - Gains are precomputed to linear (Q28 on RP2040).  Range clamps keep
 *     every Q28 coefficient inside the +/-8.0 representable range.
 */

#include <math.h>
#include <string.h>
#include "psybass.h"
#include "dsp_svf.h"

// Live configuration; vendor handlers write it and raise the pending flag,
// the main loop recomputes + publishes.  Defaults match apply_factory_defaults.
volatile PsybassConfig psybass_config = {
    .enabled = false,
    .cutoff_hz = PSYBASS_DEFAULT_CUTOFF,
    .harmonics_db = PSYBASS_DEFAULT_HARMONICS,
    .drive_db = PSYBASS_DEFAULT_DRIVE,
    .character_pct = PSYBASS_DEFAULT_CHARACTER,
    .original_db = PSYBASS_DEFAULT_ORIGINAL,
    .output_mask = PSYBASS_DEFAULT_OUTPUT_MASK,
};
volatile bool psybass_update_pending = false;

// Per-output filter state, one entry per output channel.
PsybassOutputState psybass_output_state[NUM_OUTPUT_CHANNELS];

// Published coefficient set the pipeline snapshots each packet; NULL = disabled.
volatile const PsybassCoeffs *current_psybass_coeffs = NULL;

// Double buffer so psybass_apply_config() can compute into the inactive
// buffer and publish, never writing through the currently published pointer.
static PsybassCoeffs pb_coeff_bufs[2];
static uint8_t pb_coeff_idx = 0;

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void psybass_compute_coefficients(PsybassCoeffs *coeffs, const PsybassConfig *config, float sample_rate) {
    if (!config->enabled || sample_rate < 1.0f) {
        memset(coeffs, 0, sizeof(PsybassCoeffs));
        return;
    }

    float fc = clampf(config->cutoff_hz, PSYBASS_CUTOFF_MIN, PSYBASS_CUTOFF_MAX);
    float harm_db = clampf(config->harmonics_db, PSYBASS_HARMONICS_MIN, PSYBASS_HARMONICS_MAX);
    float drive_db = clampf(config->drive_db, PSYBASS_DRIVE_MIN, PSYBASS_DRIVE_MAX);
    float character = clampf(config->character_pct, PSYBASS_CHARACTER_MIN, PSYBASS_CHARACTER_MAX);
    float orig_db = clampf(config->original_db, PSYBASS_ORIGINAL_MIN, PSYBASS_ORIGINAL_MAX);

    const float pi = 3.1415926535f;
    const float k = 1.4142135624f;   // 1/Q, Q = 0.7071 (Butterworth)

    // Gains
    float drive_f = powf(10.0f, drive_db / 20.0f);        // 1.0 .. 7.94
    float harm_gain_f = powf(10.0f, harm_db / 20.0f);     // 0.063 .. 3.98
    float odd_w_f = character * 0.01f;
    float even_w_f = 1.0f - odd_w_f;
    float orig_delta_f = powf(10.0f, orig_db / 20.0f) - 1.0f;   // -1 .. 0

    // One-pole harmonic lowpass at 4x cutoff (max 1200 Hz, always valid)
    float fh = fc * PSYBASS_HARM_LP_RATIO;
    float xh = expf(-2.0f * pi * fh / sample_rate);
    float hl_a0_f = 1.0f - xh;
    float hl_b1_f = xh;

#if PICO_RP2350
    // TPT SVF integrator coefficients, shared corner for split LP and
    // harmonic HP (both Butterworth at fc)
    float g = tanf(pi * fc / sample_rate);
    float a1 = 1.0f / (1.0f + g * (g + k));
    float a2 = g * a1;

    coeffs->lp_a1 = a1; coeffs->lp_a2 = a2; coeffs->lp_g = g;
    coeffs->hp_a1 = a1; coeffs->hp_a2 = a2; coeffs->hp_g = g;
    coeffs->hp_k = k;
    coeffs->hl_a0 = hl_a0_f;
    coeffs->hl_b1 = hl_b1_f;
    coeffs->drive = drive_f;
    coeffs->even_w = even_w_f;
    coeffs->odd_w = odd_w_f;
    coeffs->harm_gain = harm_gain_f;
    coeffs->orig_delta = orig_delta_f;
#else
    // RBJ lowpass + highpass at fc, Q = 0.7071, scaled to Q28
    float omega = 2.0f * pi * fc / sample_rate;
    float sn = sinf(omega), cs = cosf(omega);
    float alpha = sn * (0.5f * k);   // sn / (2*Q)
    float a0_f = 1.0f + alpha;
    float inv_a0 = 1.0f / a0_f;
    float scale = (float)(1LL << FILTER_SHIFT);

    float lp_b1_f = (1.0f - cs) * inv_a0;
    coeffs->lp_b0 = (int32_t)(0.5f * lp_b1_f * scale);
    coeffs->lp_b1 = (int32_t)(lp_b1_f * scale);
    coeffs->lp_b2 = (int32_t)(0.5f * lp_b1_f * scale);

    float hp_b0_f = (1.0f + cs) * 0.5f * inv_a0;
    coeffs->hp_b0 = (int32_t)(hp_b0_f * scale);
    coeffs->hp_b1 = (int32_t)(-2.0f * hp_b0_f * scale);
    coeffs->hp_b2 = (int32_t)(hp_b0_f * scale);

    int32_t a1_q = (int32_t)((-2.0f * cs) * inv_a0 * scale);
    int32_t a2_q = (int32_t)((1.0f - alpha) * inv_a0 * scale);
    coeffs->lp_a1 = a1_q; coeffs->lp_a2 = a2_q;
    coeffs->hp_a1 = a1_q; coeffs->hp_a2 = a2_q;

    coeffs->hl_a0 = (int32_t)(hl_a0_f * scale);
    coeffs->hl_b1 = (int32_t)(hl_b1_f * scale);
    coeffs->drive = (int32_t)(drive_f * scale);
    coeffs->even_w = (int32_t)(even_w_f * scale);
    coeffs->odd_w = (int32_t)(odd_w_f * scale);
    coeffs->harm_gain = (int32_t)(harm_gain_f * scale);
    coeffs->orig_delta = (int32_t)(orig_delta_f * scale);
#endif
}

void psybass_apply_config(const PsybassConfig *config, float sample_rate) {
    // Compute into the inactive buffer, then publish the pointer.  The
    // pipeline snapshots current_psybass_coeffs once per packet, so a plain
    // atomic pointer store suffices; the published buffer is never mutated.
    PsybassCoeffs *next = &pb_coeff_bufs[pb_coeff_idx ^ 1];
    psybass_compute_coefficients(next, config, sample_rate);
    if (config->enabled) {
        pb_coeff_idx ^= 1;
        current_psybass_coeffs = next;
    } else {
        current_psybass_coeffs = NULL;
    }
}

#if PICO_RP2350
// Both SVFs use the half-state arms from dsp_svf.h: section 0 is the split lowpass,
// section 1 the harmonic highpass (m1 = -k). Stored state keeps its standard meaning.
DSP_TIME_CRITICAL
void psybass_process_output_block(const PsybassCoeffs * __restrict c,
                                  PsybassOutputState * __restrict st,
                                  float * __restrict buf, uint32_t n) {
    const float A1_0 = c->lp_a1 + c->lp_a1, a2_0 = c->lp_a2, g_0 = c->lp_g;
    const float A1_1 = c->hp_a1 + c->hp_a1, a2_1 = c->hp_a2, g_1 = c->hp_g, m1_1 = -c->hp_k;
    const float drive = c->drive, even_w = c->even_w, odd_w = c->odd_w;
    const float hl_a0 = c->hl_a0, hl_b1 = c->hl_b1, orig_delta = c->orig_delta, harm_gain = c->harm_gain;
    float h1_0 = 0.5f * st->lp_ic1, ic2_0 = st->lp_ic2;
    float h1_1 = 0.5f * st->hp_ic1, ic2_1 = st->hp_ic2;
    float v1_0, v2_0, v3_0, gv_0, v1_1, v2_1, v3_1, gv_1;
    float hl = st->hl_state;

    for (uint32_t i = 0; i < n; i++) {
        float x = buf[i], low, hpo;
        SVF_LP(x, low, 0)

        // NLD: even harmonics from rectification, odd from the cubic clipper
        float even = fabsf(low);
        float d = drive * low;
        if (d > 1.0f) d = 1.0f; else if (d < -1.0f) d = -1.0f;
        float odd = d * (1.5f - 0.5f * d * d);
        float h = even_w * even + odd_w * odd;

        SVF_HP(h, hpo, 1)

        // One-pole lowpass caps harmonic brightness
        hl = hl_a0 * hpo + hl_b1 * hl;

        buf[i] = x + orig_delta * low + harm_gain * hl;
    }

    st->lp_ic1 = h1_0 + h1_0; st->lp_ic2 = ic2_0;
    st->hp_ic1 = h1_1 + h1_1; st->hp_ic2 = ic2_1;
    st->hl_state = hl;
}
#endif
