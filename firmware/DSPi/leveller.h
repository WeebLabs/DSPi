/*
 * leveller.h — Volume Leveller (Dynamic Range Compressor)
 *
 * Feedforward, stereo-linked, single-band RMS compressor with soft knee.
 * Applied to the master L/R input pair after Master EQ, before Crossfeed.
 *
 * Features:
 *   - RMS-based level detection (correlates with perceived loudness)
 *   - Soft-knee compression curve for transparent gain control
 *   - Asymmetric attack/release with configurable speed presets
 *   - Optional 10ms lookahead for predictive transient handling
 *   - Safety limiter at 0 dBFS to prevent downstream clipping
 *   - Silence gate to prevent noise-floor pumping
 *
 * Coefficient convention (Form A — "retention" form):
 *   alpha = exp(-log(10) / (Fs * T))
 *   Gives exact 90% step response at time T (0% -> 90%).
 *   alpha near 1.0 = slow (retains previous value)
 *   alpha near 0.0 = fast (tracks input immediately)
 *   Update rule: env = alpha * env + (1 - alpha) * x
 */

#ifndef LEVELLER_H
#define LEVELLER_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define LEVELLER_LOOKAHEAD_SAMPLES  480   // 10ms at 48kHz

// Speed presets
#define LEVELLER_SPEED_SLOW    0   // Music, orchestral
#define LEVELLER_SPEED_MEDIUM  1   // General purpose
#define LEVELLER_SPEED_FAST    2   // Speech, dialogue
#define LEVELLER_SPEED_COUNT   3

// Parameter limits
#define LEVELLER_AMOUNT_MIN      0.0f
#define LEVELLER_AMOUNT_MAX    100.0f
#define LEVELLER_MAX_GAIN_MIN    0.0f
#define LEVELLER_MAX_GAIN_MAX   35.0f
#define LEVELLER_GATE_MIN      (-96.0f)
#define LEVELLER_GATE_MAX        0.0f

// Fixed internal parameters
#define LEVELLER_THRESHOLD_DB     (-20.0f)   // Compression threshold (dBFS)
#define LEVELLER_KNEE_WIDTH_DB      6.0f     // Soft knee width (dB)
#define LEVELLER_LIMITER_CEIL     0.70795f   // -3 dBFS gain ceiling

// ---------------------------------------------------------------------------
// Configuration (persisted to flash and wire format)
// ---------------------------------------------------------------------------

typedef struct {
    bool    enabled;
    float   amount;          // 0.0 - 100.0 (compression strength %)
    uint8_t speed;           // LEVELLER_SPEED_SLOW/MEDIUM/FAST
    float   max_gain_db;     // 0.0 - 35.0 dB (max boost for quiet content)
    bool    lookahead;       // Enable 10ms lookahead delay
    float   gate_threshold_db; // -96.0 - 0.0 dBFS (silence gate level)
} LevellerConfig;

// Factory defaults
#define LEVELLER_DEFAULT_ENABLED      false
#define LEVELLER_DEFAULT_AMOUNT       50.0f
#define LEVELLER_DEFAULT_SPEED        LEVELLER_SPEED_SLOW
#define LEVELLER_DEFAULT_MAX_GAIN_DB  15.0f
#define LEVELLER_DEFAULT_LOOKAHEAD    true
#define LEVELLER_DEFAULT_GATE_DB      (-96.0f)

// ---------------------------------------------------------------------------
// Derived Coefficients (recomputed on config or sample rate change)
// ---------------------------------------------------------------------------

typedef struct {
    // One-pole IIR retention coefficients (Form A: env = alpha*env + (1-alpha)*x)
    float alpha_rms;         // RMS envelope time constant
    float alpha_attack;      // Gain smoother attack (gain decreasing)
    float alpha_release;     // Gain smoother release (gain recovering)

    // Compression curve parameters
    float threshold_db;      // = LEVELLER_THRESHOLD_DB
    float ratio;             // 1:1 at amount=0%, 20:1 at amount=100%
    float knee_width_db;     // = LEVELLER_KNEE_WIDTH_DB
    float makeup_db;         // Auto makeup gain (derived from ratio + threshold)
    float gate_threshold_db; // From config (user-configurable)

    // Pre-computed limits
    float max_gain_db;       // Cached from config for hot path

} LevellerCoeffs;

// ---------------------------------------------------------------------------
// Runtime State (not persisted)
// ---------------------------------------------------------------------------

#if PICO_RP2350

typedef struct {
    // Per-channel RMS squared envelopes
    float env_sq_l;
    float env_sq_r;

    // Smoothed gain output
    float gain_smooth_db;    // Current smoothed gain (dB)
    float gain_linear;       // Current linear gain multiplier
    float gain_prev_linear;  // Previous block's gain (for interpolation)

    // Lookahead circular delay buffer (always allocated, only used when enabled)
    float lookahead_buf[2][LEVELLER_LOOKAHEAD_SAMPLES];
    uint32_t la_write_idx;
} LevellerState;

#else  // RP2040

typedef struct {
    // Per-channel RMS squared envelopes (Q28)
    int32_t env_sq_l;
    int32_t env_sq_r;

    // Smoothed gain output (gain computation done in float, application in Q28)
    float gain_smooth_db;    // Current smoothed gain (dB) — always float
    int32_t gain_q28;        // Current Q28 linear gain
    int32_t gain_prev_q28;   // Previous block's Q28 gain (for interpolation)

    // Lookahead circular delay buffer (Q28)
    int32_t lookahead_buf[2][LEVELLER_LOOKAHEAD_SAMPLES];
    uint32_t la_write_idx;
} LevellerState;

#endif

// ---------------------------------------------------------------------------
// API Functions
// ---------------------------------------------------------------------------

// Recompute derived coefficients from config and sample rate.
// Called from main loop on: boot, config change, sample rate change.
void leveller_compute_coefficients(LevellerCoeffs *out,
                                   const LevellerConfig *cfg,
                                   float sample_rate);

// Reset all runtime state to initial values (zero envelopes, unity gain,
// clear lookahead buffer).  Called when leveller is enabled or lookahead toggled.
void leveller_reset_state(LevellerState *state);

// Process a block of stereo audio in-place.
// Applies RMS envelope update, gain computation, lookahead delay (if enabled),
// gain interpolation, and safety limiter.
// Marked DSP_TIME_CRITICAL — runs in the audio callback.
#if PICO_RP2350
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            float *buf_l, float *buf_r,
                            uint32_t count);
#else
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            int32_t *buf_l, int32_t *buf_r,
                            uint32_t count);
#endif

#endif // LEVELLER_H
