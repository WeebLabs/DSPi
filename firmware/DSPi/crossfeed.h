#ifndef CROSSFEED_H
#define CROSSFEED_H

#include "config.h"

// BS2B Crossfeed Presets
#define CROSSFEED_PRESET_DEFAULT    0   // 700 Hz / 4.5 dB - Balanced, most popular
#define CROSSFEED_PRESET_CHUMOY     1   // 700 Hz / 6.0 dB - Stronger spatial effect
#define CROSSFEED_PRESET_MEIER      2   // 650 Hz / 9.5 dB - Subtle, natural
#define CROSSFEED_PRESET_CUSTOM     3   // User-defined

// Custom parameter limits
#define CROSSFEED_FREQ_MIN      500.0f
#define CROSSFEED_FREQ_MAX      2000.0f
#define CROSSFEED_FEED_MIN      0.0f
#define CROSSFEED_FEED_MAX      15.0f

// Interaural Time Delay for standard 60-degree stereo speaker placement
// Computed from head model: head_width=0.15m, distance=1.0m, speed=340m/s
// d_far  = sqrt(1 + 0.005625 + 0.075) = 1.0395m
// d_near = sqrt(1 + 0.005625 - 0.075) = 0.9647m
// ITD = (d_far - d_near) / 340 = 220us
#define CROSSFEED_ITD_SEC       0.000220f

// Configuration (persisted to flash)
typedef struct {
    bool enabled;
    bool itd_enabled;       // Interaural time delay on/off
    uint8_t preset;         // 0-3 (CROSSFEED_PRESET_*)
    float custom_fc;        // Custom cutoff frequency (500-2000 Hz)
    float custom_feed_db;   // Custom feed level (0-15 dB)
} CrossfeedConfig;

// Filter state (runtime only, not persisted)
//
// Signal flow per sample:
//   lp_out   = lowpass(input)             // crossfeed component (ILD)
//   ap_out   = allpass(lp_out)            // add ITD to crossfeed path
//   direct   = input - lp_out            // complementary direct path
//   output   = direct + ap_opposite      // mix
//
// The complementary subtraction guarantees mono unity at DC.
// The all-pass on the crossfeed path adds interaural time delay (~220us)
// to simulate sound traveling around the head.
#if PICO_RP2350
typedef struct {
    float lp_a0, lp_b1;                // Lowpass coefficients
    float lp_state_L, lp_state_R;      // Lowpass filter state
    float ap_a;                         // All-pass coefficient (ITD)
    float ap_state_L, ap_state_R;      // All-pass filter state
} CrossfeedState;
#else
typedef struct {
    int32_t lp_a0, lp_b1;
    int32_t lp_state_L, lp_state_R;
    int32_t ap_a;
    int32_t ap_state_L, ap_state_R;
} CrossfeedState;
#endif

// API Functions
void crossfeed_init(CrossfeedState *state);
void crossfeed_compute_coefficients(CrossfeedState *state, const CrossfeedConfig *config, float sample_rate);

// Time-critical stereo processing - modifies left/right in place
#if PICO_RP2350
void crossfeed_process_stereo(CrossfeedState *state, float *left, float *right);
#else
void crossfeed_process_stereo(CrossfeedState *state, int32_t *left, int32_t *right);
#endif

#endif // CROSSFEED_H
