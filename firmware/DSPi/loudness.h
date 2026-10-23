#ifndef LOUDNESS_H
#define LOUDNESS_H

#include "config.h"

#define LOUDNESS_BIQUAD_COUNT 2
#define LOUDNESS_VOL_STEPS    61

// Output-channel mask: bit k = loudness compensation processes output k.
// Bits above NUM_OUTPUT_CHANNELS are ignored.
#define LOUDNESS_DEFAULT_OUTPUT_MASK 0xFFFFu

// Accepted spans for the two table parameters, shared by the vendor SET
// clamps, loudness_recompute_table, and the Control Surfaces noun table.
#define LOUDNESS_REF_SPL_MIN     40.0f
#define LOUDNESS_REF_SPL_MAX    100.0f
#define LOUDNESS_INTENSITY_MIN    0.0f
#define LOUDNESS_INTENSITY_MAX  200.0f

// Coefficients-only struct (state lives separately per output channel)
#if PICO_RP2350
typedef struct {
    float sva1, sva2, sva3;    // SVF integrator coefficients
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

// Minimal DF2 state (s1/s2 only; coefficients come from the shared table)
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

// Run the loudness shelf cascade over one output's block, in place.
// Filter-major order: each shelf processes the whole block before the next,
// which is equivalent to the per-sample cascade for LTI filters and cheaper.
// A bypassed shelf (0 dB at this volume step) clears its state and costs
// nothing.  Callers are RAM-resident; this inlines into them.
#if PICO_RP2350
static inline void loudness_process_output_block(const LoudnessCoeffs * __restrict lc_arr,
                                                 LoudnessOutputState * __restrict st_out,
                                                 float * __restrict buf, uint32_t n) {
    for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; j++) {
        const LoudnessCoeffs *lc = &lc_arr[j];
        LoudnessSvfState *st = &st_out->f[j];
        if (lc->bypass) { st->ic1eq = 0.0f; st->ic2eq = 0.0f; continue; }
        float ic1 = st->ic1eq, ic2 = st->ic2eq;
        for (uint32_t i = 0; i < n; i++) {
            float x = buf[i];
            float v3 = x - ic2;
            float v1 = lc->sva1 * ic1 + lc->sva2 * v3;
            float v2 = ic2 + lc->sva2 * ic1 + lc->sva3 * v3;
            ic1 = 2.0f * v1 - ic1;
            ic2 = 2.0f * v2 - ic2;
            buf[i] = lc->svm0 * x + lc->svm1 * v1 + lc->svm2 * v2;
        }
        st->ic1eq = ic1;
        st->ic2eq = ic2;
    }
}
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

// Double-buffered RAM tables: compute into inactive, then swap pointer
extern LoudnessCoeffs loudness_tables[2][LOUDNESS_VOL_STEPS][LOUDNESS_BIQUAD_COUNT];
extern LoudnessCoeffs (*loudness_active_table)[LOUDNESS_BIQUAD_COUNT];

// Recompute the entire loudness table for current parameters
// Called from main loop on: boot, ref SPL change, intensity change, sample rate change
void loudness_recompute_table(float ref_spl, float intensity_pct, float sample_rate);
// The same build, sliced for the main loop: begin, then step every pass; step
// returns true on the pass that makes the new table active (re-key there).
void loudness_recompute_begin(float ref_spl, float intensity_pct, float sample_rate);
bool loudness_recompute_step(void);

#endif // LOUDNESS_H
