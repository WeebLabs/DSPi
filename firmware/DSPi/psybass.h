#ifndef PSYBASS_H
#define PSYBASS_H

#include <math.h>
#include <string.h>
#include "config.h"

// Psychoacoustic bass enhancement (missing fundamental).
//
// Small speakers cannot reproduce content below their low-frequency limit.
// This effect extracts the low band below a configurable cutoff, generates
// harmonics from it with a nonlinear device (NLD), band-limits those
// harmonics into the speaker's reproducible range, and mixes them back in.
// The ear reconstructs the missing fundamental from the harmonic series.
//
// Signal flow per output channel (in place, zero added latency):
//   low  = LP2(x)                          // 2nd-order lowpass at cutoff
//   even = |low|                           // full-wave rectifier: even harmonics,
//                                          // level-proportional (dynamics track)
//   odd  = softclip(drive * low)           // cubic clipper: odd harmonics
//   h    = (1-t)*even + t*odd              // character blend t; even+odd gives a
//                                          // consecutive 2f/3f/4f series, which is
//                                          // what pitches the missing fundamental
//   h    = OP4fc(HP2(h))                   // HP2 at cutoff kills DC + fundamental,
//                                          // one-pole at 4x cutoff caps brightness
//   out  = x + (g_orig - 1)*low + g_harm*h // original low band is already split
//                                          // out, so its level control is free
//
// Pure IIR, no delay lines: inter-output-slot sample alignment is untouched
// by construction (CLAUDE.md hard rule).
//
// Module pattern follows loudness/crossfeed: one shared coefficient set
// (double-buffered, pointer-published), per-output filter state owned by
// whichever core owns that output, per-packet snapshot of pointer + mask.

// Parameter limits and defaults
#define PSYBASS_CUTOFF_MIN       30.0f
#define PSYBASS_CUTOFF_MAX      300.0f
#define PSYBASS_HARMONICS_MIN   -24.0f   // harmonic mix level (dB)
#define PSYBASS_HARMONICS_MAX    12.0f   // capped so Q28 headroom (+/-8.0) holds
#define PSYBASS_DRIVE_MIN         0.0f   // odd-path clipper drive (dB)
#define PSYBASS_DRIVE_MAX        18.0f   // 10^(18/20) = 7.94 < 8.0 Q28 coefficient
                                         // ceiling; kernel pre-clamps the low band
                                         // so drive*low never wraps fast_mul_q28
#define PSYBASS_CHARACTER_MIN     0.0f   // 0 = even only (warm)
#define PSYBASS_CHARACTER_MAX   100.0f   // 100 = odd only (aggressive)
#define PSYBASS_ORIGINAL_MIN    -60.0f   // original low-band level (dB)
#define PSYBASS_ORIGINAL_MAX      0.0f

#define PSYBASS_DEFAULT_CUTOFF       80.0f
#define PSYBASS_DEFAULT_HARMONICS     0.0f
#define PSYBASS_DEFAULT_DRIVE         6.0f
#define PSYBASS_DEFAULT_CHARACTER    50.0f
#define PSYBASS_DEFAULT_ORIGINAL      0.0f
#define PSYBASS_DEFAULT_OUTPUT_MASK 0xFFFFu

// Harmonic band top = ratio * cutoff (one-pole rolloff above)
#define PSYBASS_HARM_LP_RATIO    4.0f

// Configuration (persisted to flash / wire)
typedef struct {
    bool enabled;
    float cutoff_hz;        // speaker LF limit; harmonics generated from below it
    float harmonics_db;     // generated-harmonics mix level
    float drive_db;         // odd-path clipper drive
    float character_pct;    // even<->odd harmonic blend
    float original_db;      // original low-band level (0 = untouched)
    uint16_t output_mask;   // bit k = process output channel k
} PsybassConfig;

// Coefficients-only struct (state lives separately per output channel)
#if PICO_RP2350
typedef struct {
    float lp_a1, lp_a2, lp_a3;          // split lowpass, TPT SVF at cutoff
    float hp_a1, hp_a2, hp_a3, hp_k;    // harmonic highpass, TPT SVF at cutoff
    float hl_a0, hl_b1;                 // harmonic one-pole lowpass at 4x cutoff
    float drive;                        // linear odd-path pre-clip drive
    float even_w, odd_w;                // NLD blend weights
    float harm_gain;                    // linear harmonics mix gain
    float orig_delta;                   // original low-band gain minus 1 (-1..0)
} PsybassCoeffs;

typedef struct {
    float lp_ic1, lp_ic2;   // split SVF integrators
    float hp_ic1, hp_ic2;   // harmonic HP SVF integrators
    float hl_state;         // one-pole state
} PsybassOutputState;
#else
typedef struct {
    int32_t lp_b0, lp_b1, lp_b2, lp_a1, lp_a2;  // RBJ lowpass at cutoff (Q28)
    int32_t hp_b0, hp_b1, hp_b2, hp_a1, hp_a2;  // RBJ highpass at cutoff (Q28)
    int32_t hl_a0, hl_b1;                       // one-pole lowpass at 4x cutoff
    int32_t drive;                              // Q28 odd-path pre-clip drive
    int32_t even_w, odd_w;                      // Q28 NLD blend weights
    int32_t harm_gain;                          // Q28 harmonics mix gain
    int32_t orig_delta;                         // Q28 original gain minus 1
} PsybassCoeffs;

typedef struct {
    int32_t lp_s1, lp_s2;   // split biquad state (TDF2)
    int32_t hp_s1, hp_s2;   // harmonic HP biquad state
    int32_t hl_state;       // one-pole state
} PsybassOutputState;

int32_t fast_mul_q28(int32_t a, int32_t b);   // dsp_pipeline.c
#endif

// Live configuration + main-loop recompute flag (defined in psybass.c).
// Vendor SET handlers write the config and raise the flag; the main loop
// recomputes coefficients and publishes.  The audio path only ever reads
// the published snapshot pointer.
extern volatile PsybassConfig psybass_config;
extern volatile bool psybass_update_pending;

// Per-output filter state, indexed by output channel.  Each output is only
// ever touched by the core that owns it in the current pipeline mode.
extern PsybassOutputState psybass_output_state[NUM_OUTPUT_CHANNELS];

// Published coefficient set the pipeline snapshots each packet.
// NULL means the effect is disabled.
extern volatile const PsybassCoeffs *current_psybass_coeffs;

// Clear one output's filter state so a masked-off / muted / disabled output
// re-enters processing without a stale-state transient.
static inline void psybass_reset_output_state(PsybassOutputState *st) {
    memset(st, 0, sizeof(PsybassOutputState));
}

// Compute a coefficient set from config (clamped) at the given sample rate.
void psybass_compute_coefficients(PsybassCoeffs *coeffs, const PsybassConfig *config, float sample_rate);

// Recompute shared coefficients from config and publish current_psybass_coeffs.
// Called from the main loop while audio runs; never touches per-output state.
void psybass_apply_config(const PsybassConfig *config, float sample_rate);

// Run the effect over one output's block, in place.  Sample-major single
// pass: no scratch buffer, so it is safe on either core's stack.  Callers
// are RAM-resident; this inlines into them.
#if PICO_RP2350
static inline void psybass_process_output_block(const PsybassCoeffs * __restrict c,
                                                PsybassOutputState * __restrict st,
                                                float * __restrict buf, uint32_t n) {
    float lp_ic1 = st->lp_ic1, lp_ic2 = st->lp_ic2;
    float hp_ic1 = st->hp_ic1, hp_ic2 = st->hp_ic2;
    float hl = st->hl_state;

    for (uint32_t i = 0; i < n; i++) {
        float x = buf[i];

        // Low-band split: TPT SVF, lowpass output = v2
        float v3 = x - lp_ic2;
        float v1 = c->lp_a1 * lp_ic1 + c->lp_a2 * v3;
        float v2 = lp_ic2 + c->lp_a2 * lp_ic1 + c->lp_a3 * v3;
        lp_ic1 = 2.0f * v1 - lp_ic1;
        lp_ic2 = 2.0f * v2 - lp_ic2;
        float low = v2;

        // NLD: even harmonics from rectification, odd from the cubic clipper
        float even = fabsf(low);
        float d = c->drive * low;
        if (d > 1.0f) d = 1.0f; else if (d < -1.0f) d = -1.0f;
        float odd = d * (1.5f - 0.5f * d * d);
        float h = c->even_w * even + c->odd_w * odd;

        // Harmonic shaping: SVF highpass at cutoff (hp = x - k*v1 - v2)
        v3 = h - hp_ic2;
        v1 = c->hp_a1 * hp_ic1 + c->hp_a2 * v3;
        v2 = hp_ic2 + c->hp_a2 * hp_ic1 + c->hp_a3 * v3;
        hp_ic1 = 2.0f * v1 - hp_ic1;
        hp_ic2 = 2.0f * v2 - hp_ic2;
        float hpo = h - c->hp_k * v1 - v2;

        // One-pole lowpass caps harmonic brightness
        hl = c->hl_a0 * hpo + c->hl_b1 * hl;

        buf[i] = x + c->orig_delta * low + c->harm_gain * hl;
    }

    st->lp_ic1 = lp_ic1; st->lp_ic2 = lp_ic2;
    st->hp_ic1 = hp_ic1; st->hp_ic2 = hp_ic2;
    st->hl_state = hl;
}
#else
static inline void psybass_process_output_block(const PsybassCoeffs * __restrict c,
                                                PsybassOutputState * __restrict st,
                                                int32_t * __restrict buf, uint32_t n) {
    int32_t lp_s1 = st->lp_s1, lp_s2 = st->lp_s2;
    int32_t hp_s1 = st->hp_s1, hp_s2 = st->hp_s2;
    int32_t hl = st->hl_state;
    const int32_t one = 1 << FILTER_SHIFT;

    for (uint32_t i = 0; i < n; i++) {
        int32_t x = buf[i];

        // Low-band split: TDF2 biquad
        int32_t low = fast_mul_q28(c->lp_b0, x) + lp_s1;
        lp_s1 = fast_mul_q28(c->lp_b1, x) - fast_mul_q28(c->lp_a1, low) + lp_s2;
        lp_s2 = fast_mul_q28(c->lp_b2, x) - fast_mul_q28(c->lp_a2, low);

        // NLD: even harmonics from rectification, odd from the cubic clipper,
        // which runs at 0 dBFS = 1.0 (samples arrive at 2^29): halve in, double out.
        // Clamp before the drive multiply (drive up to 7.94, fast_mul_q28 wraps
        // past +/-8.0); the second clamp keeps d^3 in range.
        int32_t even = low < 0 ? -low : low;
        int32_t dl = low >> 1;
        if (dl > one) dl = one; else if (dl < -one) dl = -one;
        int32_t d = fast_mul_q28(c->drive, dl);
        if (d > one) d = one; else if (d < -one) d = -one;
        int32_t d3 = fast_mul_q28(fast_mul_q28(d, d), d);
        int32_t odd = (d + (d >> 1) - (d3 >> 1)) * 2;
        int32_t h = fast_mul_q28(c->even_w, even) + fast_mul_q28(c->odd_w, odd);

        // Harmonic shaping: TDF2 highpass at cutoff
        int32_t hpo = fast_mul_q28(c->hp_b0, h) + hp_s1;
        hp_s1 = fast_mul_q28(c->hp_b1, h) - fast_mul_q28(c->hp_a1, hpo) + hp_s2;
        hp_s2 = fast_mul_q28(c->hp_b2, h) - fast_mul_q28(c->hp_a2, hpo);

        // One-pole lowpass caps harmonic brightness
        hl = fast_mul_q28(c->hl_a0, hpo) + fast_mul_q28(c->hl_b1, hl);

        buf[i] = x + fast_mul_q28(c->orig_delta, low) + fast_mul_q28(c->harm_gain, hl);
    }

    st->lp_s1 = lp_s1; st->lp_s2 = lp_s2;
    st->hp_s1 = hp_s1; st->hp_s2 = hp_s2;
    st->hl_state = hl;
}
#endif

#endif // PSYBASS_H
