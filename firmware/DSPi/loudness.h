#ifndef LOUDNESS_H
#define LOUDNESS_H

#include "config.h"

#define LOUDNESS_BIQUAD_COUNT 2
#define LOUDNESS_VOL_STEPS    61

// Output-channel mask: bit k = loudness compensation processes output k.
// Bits above NUM_OUTPUT_CHANNELS are ignored.
#define LOUDNESS_DEFAULT_OUTPUT_MASK 0xFFFFu

// Accepted spans for the two curve parameters, shared by the vendor SET
// clamps, loudness_update_begin, and the Control Surfaces noun table.
#define LOUDNESS_REF_SPL_MIN     40.0f
#define LOUDNESS_REF_SPL_MAX    100.0f
#define LOUDNESS_INTENSITY_MIN    0.0f
#define LOUDNESS_INTENSITY_MAX  200.0f

// Coefficients-only struct (state lives separately per output channel)
#if PICO_RP2350
typedef struct {
    float sva1, sva2, g;       // SVF integrator coefficients and g, for the half-state kernel
    float svm0, svm1, svm2;    // SVF output mix coefficients (shelf: general formula)
    bool bypass;
} LoudnessCoeffs;

// Minimal SVF state for loudness filters (separate from main EQ Filter struct)
typedef struct {
    float ic1eq, ic2eq;
} LoudnessSvfState;

typedef struct { LoudnessSvfState f[LOUDNESS_BIQUAD_COUNT]; } LoudnessOutputState;
#else
typedef struct { int32_t b0, b1, b2, a1, a2; bool bypass; } LoudnessCoeffs;

// Minimal DF2 state (s1/s2 only; coefficients come from current_loudness_coeffs)
typedef struct {
    int32_t s1, s2;
} LoudnessBqState;

typedef struct { LoudnessBqState f[LOUDNESS_BIQUAD_COUNT]; } LoudnessOutputState;

int32_t fast_mul_q28(int32_t a, int32_t b);   // dsp_pipeline.c
#endif

// Per-output filter state, indexed by output channel.  Each output is only
// ever touched by the core that owns it in the current pipeline mode.
extern LoudnessOutputState loudness_output_state[NUM_OUTPUT_CHANNELS];

// Clear one output's filter state so a masked-off / muted / disabled output
// re-enters compensation without a stale-state transient.
static inline void loudness_reset_output_state(LoudnessOutputState *st) {
    for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
#if PICO_RP2350
        st->f[j].ic1eq = 0.0f;
        st->f[j].ic2eq = 0.0f;
#else
        st->f[j].s1 = 0;
        st->f[j].s2 = 0;
#endif
    }
}

// Run the loudness shelf cascade over one output's block, in place; a bypassed
// shelf (0 dB at this volume step) clears its state.  RP2350: one out-of-line
// copy in loudness.c serves both cores, shelves fused per sample.  RP2040:
// inline, filter-major (each shelf sweeps the block before the next).
#if PICO_RP2350
void loudness_process_output_block(const LoudnessCoeffs * __restrict lc_arr,
                                   LoudnessOutputState * __restrict st_out,
                                   float * __restrict buf, uint32_t n);
#else
static inline void loudness_process_output_block(const LoudnessCoeffs * __restrict lc_arr,
                                                 LoudnessOutputState * __restrict st_out,
                                                 int32_t * __restrict buf, uint32_t n) {
    for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
        const LoudnessCoeffs *lc = &lc_arr[j];
        LoudnessBqState *st = &st_out->f[j];
        if (lc->bypass) { st->s1 = 0; st->s2 = 0; continue; }
        int32_t s1 = st->s1, s2 = st->s2;
        for (uint32_t i = 0; i < n; i++) {
            int32_t x = buf[i];
            int32_t y = fast_mul_q28(lc->b0, x) + s1;
            s1 = fast_mul_q28(lc->b1, x) - fast_mul_q28(lc->a1, y) + s2;
            s2 = fast_mul_q28(lc->b2, x) - fast_mul_q28(lc->a2, y);
            buf[i] = y;
        }
        st->s1 = s1;
        st->s2 = s2;
    }
}
#endif

// Coefficients for the current volume step, or NULL while loudness is off.
// Written only by the functions below (Core 0 main loop); the audio path reads it.
extern const LoudnessCoeffs *volatile current_loudness_coeffs;

// Recompute for this state as a background job (bg_job.h) and publish when done.
// A later call before the job runs replaces the state; enabled=false publishes NULL now.
void loudness_update_begin(bool enabled, uint8_t vol_index, float ref_spl, float intensity_pct, float sample_rate);
// Boot only: the same, computed and published before returning.
void loudness_update_now(bool enabled, uint8_t vol_index, float ref_spl, float intensity_pct, float sample_rate);

#endif // LOUDNESS_H
