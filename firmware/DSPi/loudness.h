#ifndef LOUDNESS_H
#define LOUDNESS_H

#include "config.h"

#define LOUDNESS_BIQUAD_COUNT 2
#define LOUDNESS_VOL_STEPS    61

// Coefficients-only struct (state lives separately per channel)
#if PICO_RP2350
typedef struct {
    float sva1, sva2, sva3;    // SVF integrator coefficients
    float svm0, svm1, svm2;    // SVF output mix coefficients (shelf: general formula)
    bool bypass;
} LoudnessCoeffs;

// Minimal SVF state for loudness filters (separate from main EQ Biquad struct)
typedef struct {
    float ic1eq, ic2eq;
} LoudnessSvfState;
#else
typedef struct { int32_t b0, b1, b2, a1, a2; bool bypass; } LoudnessCoeffs;
#endif

// Double-buffered RAM tables: compute into inactive, then swap pointer
extern LoudnessCoeffs loudness_tables[2][LOUDNESS_VOL_STEPS][LOUDNESS_BIQUAD_COUNT];
extern LoudnessCoeffs (*loudness_active_table)[LOUDNESS_BIQUAD_COUNT];

// Recompute the entire loudness table for current parameters
// Called from main loop on: boot, ref SPL change, intensity change, sample rate change
void loudness_recompute_table(float ref_spl, float intensity_pct, float sample_rate);

#endif // LOUDNESS_H
