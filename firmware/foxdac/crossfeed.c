/*
 * BS2B Crossfeed Implementation
 *
 * Implements Bauer Stereophonic-to-Binaural (BS2B) crossfeed for headphone listening.
 * Reduces unnatural stereo separation by mixing a filtered portion of each channel
 * into the opposite channel, simulating speaker listening in a room.
 *
 * Uses complementary filter design with ITD:
 *   - Lowpass filter computes the crossfeed signal (ILD/head shadow)
 *   - First-order all-pass adds interaural time delay to the crossfeed path
 *   - Direct path is the complement: input - lowpass(input)
 *   - Output: out_L = (in_L - lp_L) + allpass(lp_R)
 *
 * Mono signals pass through at unity gain at DC (complementary property).
 * Hard-panned HF content is unchanged (lowpass → 0 at HF).
 */

#include <math.h>
#include <string.h>
#include "crossfeed.h"

// Preset definitions: {cutoff_hz, feed_db}
// feed_db = level difference between direct and crossfeed at DC
static const float presets[][2] = {
    { 700.0f,  4.5f },  // Default - balanced, most popular
    { 700.0f,  6.0f },  // Chu Moy - stronger spatial effect
    { 650.0f,  9.5f },  // Jan Meier - subtle, natural
};

void crossfeed_init(CrossfeedState *state) {
    memset(state, 0, sizeof(CrossfeedState));
}

void crossfeed_compute_coefficients(CrossfeedState *state, const CrossfeedConfig *config, float sample_rate) {
    if (!config->enabled || sample_rate < 1.0f) {
        crossfeed_init(state);
        return;
    }

    // Get cutoff and feed level from preset or custom
    float fc, feed_db;
    if (config->preset < 3) {
        fc = presets[config->preset][0];
        feed_db = presets[config->preset][1];
    } else {
        fc = config->custom_fc;
        feed_db = config->custom_feed_db;
        if (fc < CROSSFEED_FREQ_MIN) fc = CROSSFEED_FREQ_MIN;
        if (fc > CROSSFEED_FREQ_MAX) fc = CROSSFEED_FREQ_MAX;
        if (feed_db < CROSSFEED_FEED_MIN) feed_db = CROSSFEED_FEED_MIN;
        if (feed_db > CROSSFEED_FEED_MAX) feed_db = CROSSFEED_FEED_MAX;
    }

    // =========================================================================
    // Compute crossfeed gain G using complementary constraint
    //
    // feed_db is the level difference: 20*log10(direct_dc / cross_dc)
    // With complementary constraint: direct_dc + cross_dc = 1
    //
    //   direct_dc / cross_dc = 10^(feed_db/20) = level_ratio
    //   cross_dc = 1 / (1 + level_ratio) = G
    //   direct_dc = 1 - G
    //
    // Example (4.5 dB): level_ratio=1.679, G=0.373, direct=0.627
    // =========================================================================
    float level_ratio = powf(10.0f, feed_db / 20.0f);
    float G = 1.0f / (1.0f + level_ratio);

    // =========================================================================
    // Lowpass filter (crossfeed path) - single pole IIR
    // H(z) = G*(1-x) / (1 - x*z^-1)   where x = exp(-2π*Fc/Fs)
    // DC gain = G, HF gain → 0
    // =========================================================================
    float x = expf(-2.0f * 3.1415926535f * fc / sample_rate);
    float lp_a0_f = G * (1.0f - x);
    float lp_b1_f = x;

    // =========================================================================
    // All-pass filter for Interaural Time Delay (ITD)
    //
    // The lowpass filter already introduces phase delay at DC:
    //   τ_lp = x / ((1-x) * Fs)  seconds
    //
    // The remaining delay is provided by a first-order all-pass:
    //   H_ap(z) = (a + z^-1) / (1 + a*z^-1)
    //   Group delay at DC = (1-a) / (1+a) samples
    //
    // Solving for a:  a = (1 - D) / (1 + D)
    // where D = remaining delay in samples
    //
    // For 700Hz @ 48kHz: lp_delay ≈ 217µs, ITD = 220µs, remainder ≈ 3µs
    // For 2000Hz @ 48kHz: lp_delay ≈ 80µs, ITD = 220µs, remainder ≈ 140µs
    //
    // When ITD is disabled, a = 1.0 makes the all-pass a pure passthrough.
    // =========================================================================
    float ap_a_f;
    if (config->itd_enabled) {
        float lp_delay_sec = x / ((1.0f - x) * sample_rate);
        float remaining_sec = CROSSFEED_ITD_SEC - lp_delay_sec;
        if (remaining_sec > 0.0f) {
            float D = remaining_sec * sample_rate;  // remaining delay in samples
            ap_a_f = (1.0f - D) / (1.0f + D);
        } else {
            ap_a_f = 1.0f;  // No additional delay needed (lowpass already provides enough)
        }
    } else {
        ap_a_f = 1.0f;  // ITD disabled: all-pass is passthrough
    }

#if PICO_RP2350
    state->lp_a0 = lp_a0_f;
    state->lp_b1 = lp_b1_f;
    state->ap_a = ap_a_f;
#else
    float scale = (float)(1LL << 28);
    state->lp_a0 = (int32_t)(lp_a0_f * scale);
    state->lp_b1 = (int32_t)(lp_b1_f * scale);
    state->ap_a = (int32_t)(ap_a_f * scale);
#endif

    // Clear filter states
    state->lp_state_L = 0;
    state->lp_state_R = 0;
    state->ap_state_L = 0;
    state->ap_state_R = 0;
}

#if PICO_RP2350
// RP2350 Float processing
DSP_TIME_CRITICAL
void crossfeed_process_stereo(CrossfeedState *state, float *left, float *right) {
    float in_L = *left;
    float in_R = *right;

    // Lowpass filter both channels: cross = G × L(z) × input
    float lp_out_L = state->lp_a0 * in_L + state->lp_b1 * state->lp_state_L;
    float lp_out_R = state->lp_a0 * in_R + state->lp_b1 * state->lp_state_R;
    state->lp_state_L = lp_out_L;
    state->lp_state_R = lp_out_R;

    // All-pass filter on crossfeed signals for ITD
    // First-order all-pass, transposed direct form II:
    //   y[n] = a * x[n] + s[n]
    //   s[n+1] = x[n] - a * y[n]
    float ap_out_L = state->ap_a * lp_out_L + state->ap_state_L;
    state->ap_state_L = lp_out_L - state->ap_a * ap_out_L;
    float ap_out_R = state->ap_a * lp_out_R + state->ap_state_R;
    state->ap_state_R = lp_out_R - state->ap_a * ap_out_R;

    // Complementary mixing with ITD:
    //   direct = input - own_lowpass (undelayed complement)
    //   output = direct + allpass(opp_lowpass) (delayed crossfeed from opposite)
    *left  = (in_L - lp_out_L) + ap_out_R;
    *right = (in_R - lp_out_R) + ap_out_L;
}

#else
// RP2040 Fixed-point processing (Q28)
DSP_TIME_CRITICAL
void crossfeed_process_stereo(CrossfeedState *state, int32_t *left, int32_t *right) {
    int32_t in_L = *left;
    int32_t in_R = *right;

    // Lowpass filter both channels
    int64_t lp_acc_L = (int64_t)state->lp_a0 * in_L + (int64_t)state->lp_b1 * state->lp_state_L;
    int64_t lp_acc_R = (int64_t)state->lp_a0 * in_R + (int64_t)state->lp_b1 * state->lp_state_R;
    int32_t lp_out_L = (int32_t)(lp_acc_L >> 28);
    int32_t lp_out_R = (int32_t)(lp_acc_R >> 28);
    state->lp_state_L = lp_out_L;
    state->lp_state_R = lp_out_R;

    // All-pass filter on crossfeed signals for ITD (Q28)
    int32_t ap_out_L = (int32_t)(((int64_t)state->ap_a * lp_out_L) >> 28) + state->ap_state_L;
    state->ap_state_L = lp_out_L - (int32_t)(((int64_t)state->ap_a * ap_out_L) >> 28);
    int32_t ap_out_R = (int32_t)(((int64_t)state->ap_a * lp_out_R) >> 28) + state->ap_state_R;
    state->ap_state_R = lp_out_R - (int32_t)(((int64_t)state->ap_a * ap_out_R) >> 28);

    // Complementary mixing with ITD
    *left  = (in_L - lp_out_L) + ap_out_R;
    *right = (in_R - lp_out_R) + ap_out_L;
}
#endif
