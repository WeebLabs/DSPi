/*
 * leveller.h; Volume Leveller (Dynamic Range Compressor)
 *
 * Feedforward, channel-linked, single-band RMS compressor with soft knee.
 * Runs pre-matrix on the input channels (PASS 2.5, after per-input EQ).
 *
 * Multichannel model:
 *   - detector_mask selects which input channels feed the RMS detector
 *     (bit k = input channel k).  The link is the loudest selected envelope,
 *     so one shared gain preserves the mix balance across channels.
 *   - apply_mask selects which input channels the shared gain is applied to.
 *     CPU is only spent on selected channels.
 *   - With lookahead enabled, EVERY active input channel is delayed by the
 *     lookahead length (applied channels through the gain stage, the rest
 *     as a plain delay) so inter-channel alignment is preserved exactly.
 *     Mask changes therefore never cause a time shift.
 *
 * Features:
 *   - RMS-based level detection (correlates with perceived loudness)
 *   - Soft-knee upward compression for transparent gain control
 *   - Asymmetric attack/release with configurable speed presets
 *   - Optional 5ms lookahead for predictive transient handling
 *   - Safety limiter: a smoothed peak limiter on the boost, never a clipper
 *   - Silence gate to prevent noise-floor pumping
 *
 * Coefficient convention (Form A; "retention" form):
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

#define LEVELLER_LOOKAHEAD_SAMPLES  240   // 5ms at 48kHz

// Safety limiter granularity.  The lookahead must hold a whole number of
// blocks: the limiter keeps one peak cap per ring block.
#define LEVELLER_LIM_BLOCK          48
#define LEVELLER_LIM_BLOCKS         (LEVELLER_LOOKAHEAD_SAMPLES / LEVELLER_LIM_BLOCK)
_Static_assert(LEVELLER_LOOKAHEAD_SAMPLES % LEVELLER_LIM_BLOCK == 0,
               "lookahead ring must split into whole limiter blocks");

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
#define LEVELLER_LIM_HOLD_SEC       0.050f   // Covers a half cycle down to 10 Hz
#define LEVELLER_LIM_RELEASE_DBPS  60.0f     // Release rate after the hold

// ---------------------------------------------------------------------------
// Configuration (persisted to flash and wire format)
// ---------------------------------------------------------------------------

typedef struct {
    bool    enabled;
    float   amount;          // 0.0 - 100.0 (compression strength %)
    uint8_t speed;           // LEVELLER_SPEED_SLOW/MEDIUM/FAST
    float   max_gain_db;     // 0.0 - 35.0 dB (max boost for quiet content)
    bool    lookahead;       // Enable 5ms lookahead delay
    float   gate_threshold_db; // -96.0 - 0.0 dBFS (silence gate level)
    uint8_t detector_mask;   // Bit k: input channel k feeds the detector
    uint8_t apply_mask;      // Bit k: gain is applied to input channel k
} LevellerConfig;

// Factory defaults
#define LEVELLER_DEFAULT_ENABLED      false
#define LEVELLER_DEFAULT_AMOUNT       50.0f
#define LEVELLER_DEFAULT_SPEED        LEVELLER_SPEED_SLOW
#define LEVELLER_DEFAULT_MAX_GAIN_DB  15.0f
#define LEVELLER_DEFAULT_LOOKAHEAD    true
#define LEVELLER_DEFAULT_GATE_DB      (-96.0f)
#define LEVELLER_DEFAULT_DETECTOR_MASK 0xFF   // All inputs
#define LEVELLER_DEFAULT_APPLY_MASK    0xFF   // All inputs

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

    // Safety limiter, per LEVELLER_LIM_BLOCK samples
    float    lim_release;      // Release multiplier on the held cap
    int32_t  lim_release_q28;  // Same, Q28 (RP2040)
    uint16_t lim_hold_blocks;  // Hold before the release starts

} LevellerCoeffs;

// ---------------------------------------------------------------------------
// Runtime State (not persisted)
// ---------------------------------------------------------------------------

#if PICO_RP2350

typedef struct {
    // Per-input-channel RMS squared envelopes
    float env_sq[NUM_INPUT_CHANNELS];

    // Smoothed gain output
    float gain_smooth_db;    // Current smoothed gain (dB)
    float gain_linear;       // Current linear gain multiplier
    float gain_prev_linear;  // Previous block's gain (for interpolation)

    // Per-input lookahead rings (always allocated, only used when enabled)
    float lookahead_buf[NUM_INPUT_CHANNELS][LEVELLER_LOOKAHEAD_SAMPLES];
    uint32_t la_write_idx;

    // Active-input count seen last block; rings of newly activated inputs
    // are cleared so stale samples never reach the matrix.
    uint8_t active_prev;

    // Safety limiter: gain caps (max boost keeping peaks under the ceiling).
    // lim_cap[b] covers ring block b, so the ramp can meet it before it plays.
    float lim_cap[LEVELLER_LIM_BLOCKS];
    float lim_l0, lim_step;  // Cap ramp over the current block
    float lim_end;           // Ramp value at the next block boundary
    float lim_inst;          // Instant cap this block (lookahead off)
    float lim_env;           // Held / releasing cap from blocks already played
    float lim_pin, lim_pout; // Running peaks of incoming / outgoing samples
    uint16_t lim_hold;
    uint16_t lim_app_prev;   // Apply set (+0x100 lookahead) the caps measured
    uint8_t lim_pos;         // Sample position inside the limiter block
} LevellerState;

#else  // RP2040

typedef struct {
    // Per-input-channel RMS squared envelopes (Q28)
    int32_t env_sq[NUM_INPUT_CHANNELS];

    // Smoothed gain output (gain computation done in float, application in Q28)
    float gain_smooth_db;    // Current smoothed gain (dB); always float
    int32_t gain_q28;        // Current Q28 linear gain
    int32_t gain_prev_q28;   // Previous block's Q28 gain (for interpolation)

    // Per-input lookahead rings (Q28)
    int32_t lookahead_buf[NUM_INPUT_CHANNELS][LEVELLER_LOOKAHEAD_SAMPLES];
    uint32_t la_write_idx;

    uint8_t active_prev;    // Unused on RP2040 (input count fixed at 2)

    // Safety limiter, Q28 gains (see the RP2350 struct)
    int32_t lim_cap[LEVELLER_LIM_BLOCKS];
    int32_t lim_l0, lim_step;
    int32_t lim_end;
    int32_t lim_inst;
    int32_t lim_env;
    uint32_t lim_pin, lim_pout;
    uint16_t lim_hold;
    uint16_t lim_app_prev;
    uint8_t lim_pos;
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

// Process a block of n_bufs active input channels in-place.
// bufs[k] is input channel k's block (the pipeline's input_bufs).
// Applies RMS envelope update (detector_mask), shared gain computation,
// lookahead delay on ALL active channels (if enabled), gain interpolation
// and the safety limiter on apply_mask channels.
// Marked DSP_TIME_CRITICAL; runs in the audio callback.
#if PICO_RP2350
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            float *const *bufs, uint32_t n_bufs,
                            uint32_t count);
#else
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            int32_t *const *bufs, uint32_t n_bufs,
                            uint32_t count);
#endif

#endif // LEVELLER_H
