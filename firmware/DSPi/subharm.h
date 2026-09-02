#ifndef SUBHARM_H
#define SUBHARM_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "config.h"

// Subharmonic synthesizer (dbx 120A style octave divider).
//
// Two fixed input bands (48-72 Hz and 72-112 Hz) are split off per output.
// Each is polarity-flipped once per cycle by a hysteresis zero-crossing
// divider (one octave down, amplitude follows the band), lowpassed to strip
// the switching harmonics, and mixed back at its own level.  A bell boost
// then fills 40-120 Hz after the sum, as on the dbx.  Pure IIR with zero
// added latency, so inter-slot alignment is untouched by construction.
// Full description: Documentation/Features/subharmonic_synth_spec.md.
//
// Module pattern follows psybass: one shared coefficient set (double-
// buffered, pointer-published, NULL = off), per-output state owned by the
// core that owns the output, per-packet snapshot of pointer + mask.

#define SUBHARM_NUM_BANDS        2

// Input band edges (Hz).  Band 0 = LO..MID (sub 24-36 Hz), band 1 = MID..HI
// (sub 36-56 Hz).  Post-divider lowpass corners sit just above each sub top.
#define SUBHARM_BAND_LO_HZ      48.0f
#define SUBHARM_BAND_MID_HZ     72.0f
#define SUBHARM_BAND_HI_HZ     112.0f
#define SUBHARM_SUB_LP0_HZ      40.0f
#define SUBHARM_SUB_LP1_HZ      62.0f

// First-order allpass on band 1's sub path.  It matches the phase lag of the
// two post-divider lowpasses (40 vs 62 Hz) across 24-56 Hz, which keeps the
// two bands' subs in quadrature so their sum is independent of the arbitrary
// flip-flop parity left behind by the previous note (otherwise up to 4.7 dB).
#define SUBHARM_ALIGN_AP_HZ    160.0f

// LF boost bell: centred between the synthesized sub and the program mid-bass.
#define SUBHARM_BOOST_HZ        70.0f
#define SUBHARM_BOOST_Q          0.9f

// Divider envelope follower time constant (arming threshold = env / 4).
#define SUBHARM_ENV_TAU_MS      40.0f

// Parameter limits and defaults.  A band at SUBHARM_LEVEL_MIN is off and its
// processing is skipped.  Ceilings are set so every Q28 product stays inside
// +/-8.0 on RP2040 (see the band clamp below and the spec's headroom notes).
#define SUBHARM_LEVEL_MIN      -30.0f   // band level (dB); floor = band off
#define SUBHARM_LEVEL_MAX        6.0f
#define SUBHARM_BOOST_MIN        0.0f   // LF boost (dB); 0 = stage skipped
#define SUBHARM_BOOST_MAX        6.0f

#define SUBHARM_DEFAULT_LOW          0.0f
#define SUBHARM_DEFAULT_HIGH         0.0f
#define SUBHARM_DEFAULT_BOOST        0.0f
#define SUBHARM_DEFAULT_OUTPUT_MASK 0xFFFFu

// Configuration (persisted to flash / wire)
typedef struct {
    bool     enabled;
    float    low_db;        // 24-36 Hz band level (SUBHARM_LEVEL_MIN = off)
    float    high_db;       // 36-56 Hz band level (SUBHARM_LEVEL_MIN = off)
    float    boost_db;      // LF boost bell gain (0 = off)
    uint16_t output_mask;   // bit k = process output channel k
} SubharmConfig;

// ---------------------------------------------------------------------------
// Number type.  The kernel is written once against these helpers; RP2350
// runs it in float, RP2040 in Q28 through fast_mul_q28.
// ---------------------------------------------------------------------------
#if PICO_RP2350
typedef float sh_num_t;
#define SH_ZERO 0.0f
static inline sh_num_t sh_mul(sh_num_t a, sh_num_t b) { return a * b; }
static inline sh_num_t sh_twice(sh_num_t v)           { return 2.0f * v; }
static inline sh_num_t sh_quarter(sh_num_t v)         { return 0.25f * v; }
static inline sh_num_t sh_abs(sh_num_t v)             { return fabsf(v); }
static inline sh_num_t sh_band_limit(sh_num_t v)      { return v; }
#else
typedef int32_t sh_num_t;
#define SH_ZERO 0
int32_t fast_mul_q28(int32_t a, int32_t b);   // dsp_pipeline.c
static inline sh_num_t sh_mul(sh_num_t a, sh_num_t b) { return fast_mul_q28(a, b); }
static inline sh_num_t sh_twice(sh_num_t v)           { return v * 2; }
static inline sh_num_t sh_quarter(sh_num_t v)         { return v >> 2; }
static inline sh_num_t sh_abs(sh_num_t v)             { return v < 0 ? -v : v; }
// Clamp a band signal to +/-1.0 before the divider.  fast_mul_q28 wraps past
// +/-8.0, so the divider input must be bounded for the +6 dB band level and
// +6 dB bell to stay inside the representable range on hot inputs.
static inline sh_num_t sh_band_limit(sh_num_t v) {
    const int32_t one = 1 << FILTER_SHIFT;
    return v > one ? one : (v < -one ? -one : v);
}
#endif

// TPT state-variable filter (Cytomic form).  Integrator coefficients plus the
// damping k the highpass/bell outputs need.
typedef struct {
    sh_num_t a1, a2, a3, k;
} SubharmSvf;

typedef struct {
    sh_num_t ic1, ic2;
} SubharmSvfState;

typedef struct {
    SubharmSvf lp;          // post-divider lowpass
    sh_num_t   gain;        // linear band level; SH_ZERO = band off (skipped)
} SubharmBandCoeffs;

typedef struct {
    SubharmSvf hp_lo;       // HP2 at SUBHARM_BAND_LO_HZ (highpass output)
    SubharmSvf lp_hi;       // LP2 at SUBHARM_BAND_HI_HZ (lowpass output)
    SubharmSvf split;       // at SUBHARM_BAND_MID_HZ: LP out = band 0, HP out = band 1
    SubharmBandCoeffs band[SUBHARM_NUM_BANDS];
    sh_num_t   align_g;     // one-pole allpass G = g/(1+g) on band 1's sub
    SubharmSvf bell;        // LF boost bell
    sh_num_t   bell_m1;     // k * (A^2 - 1) bell mix; SH_ZERO = boost off (skipped)
    sh_num_t   env_decay;   // per-sample envelope follower decay
} SubharmCoeffs;

typedef struct {
    SubharmSvfState lp;
    sh_num_t env;           // band peak follower
    uint8_t  armed;         // band dipped below -env/4 since the last flip
    uint8_t  neg;           // divider polarity: 1 = inverting this cycle
} SubharmBandState;

typedef struct {
    SubharmSvfState  hp_lo, lp_hi, split;
    SubharmBandState band[SUBHARM_NUM_BANDS];
    sh_num_t         align;  // band 1 allpass state
    SubharmSvfState  bell;
} SubharmOutputState;

// Live configuration + main-loop recompute flag (defined in subharm.c).
// Vendor SET handlers write the config and raise the flag; the main loop
// recomputes coefficients and publishes.  The audio path only ever reads
// the published snapshot pointer.
extern volatile SubharmConfig subharm_config;
extern volatile bool subharm_update_pending;

// Per-output state, indexed by output channel.  Each output is only ever
// touched by the core that owns it in the current pipeline mode.
extern SubharmOutputState subharm_output_state[NUM_OUTPUT_CHANNELS];

// Published coefficient set the pipeline snapshots each packet.
// NULL means the effect is disabled.
extern volatile const SubharmCoeffs *current_subharm_coeffs;

// Clear one output's state so a masked-off / muted / disabled output
// re-enters processing without a stale-state transient.
static inline void subharm_reset_output_state(SubharmOutputState *st) {
    memset(st, 0, sizeof(SubharmOutputState));
}

// Compute a coefficient set from config (clamped) at the given sample rate.
void subharm_compute_coefficients(SubharmCoeffs *coeffs, const SubharmConfig *config, float sample_rate);

// Recompute shared coefficients from config and publish current_subharm_coeffs.
// Called from the main loop while audio runs; never touches per-output state.
void subharm_apply_config(const SubharmConfig *config, float sample_rate);

// Worst-case output gain of this configuration in dB (0 when disabled): the
// preamp headroom a host must free so the effect cannot clip.  Steady-state
// tone bound scanned over the bass band; pure function of the config.
float subharm_headroom_db(const SubharmConfig *config);

// Run the effect over one output's block, in place.  One RAM-resident copy
// (subharm.c) shared by both cores: the kernel is too large to inline at
// every call site without spending several KB of RAM on duplicates.
void subharm_process_output_block(const SubharmCoeffs * __restrict c,
                                  SubharmOutputState * __restrict st,
                                  sh_num_t * __restrict buf, uint32_t n);

#endif // SUBHARM_H
