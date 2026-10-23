#ifndef SUBHARM_H
#define SUBHARM_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "config.h"

// Subharmonic synthesizer (dbx 120A style octave divider, extended): three
// program bands divided one octave down at a decimated rate, mixed back per
// output.  Module pattern follows psybass (shared published coeffs, per-output
// state).  Design: Documentation/Features/subharmonic_synth_spec.md.

#define SUBHARM_NUM_BANDS        3

// Input band edges (Hz).  Band 0 = LO..MID1 (sub 24-36 Hz), band 1 =
// MID1..MID2 (sub 36-56 Hz), band 2 = MID2..HI (sub 56-80 Hz).  The HI
// lowpass runs at full rate and doubles as the decimation anti-alias filter.
#define SUBHARM_BAND_LO_HZ      48.0f
#define SUBHARM_BAND_MID1_HZ    72.0f
#define SUBHARM_BAND_MID2_HZ   112.0f
#define SUBHARM_BAND_HI_HZ     160.0f
#define SUBHARM_SUB_LP0_HZ      40.0f
#define SUBHARM_SUB_LP1_HZ      62.0f
#define SUBHARM_SUB_LP2_HZ      80.0f

// Low-rate target.  D = round(fs / target): 6 at 44.1/48 kHz, 12 at 96 kHz.
#define SUBHARM_DECIM_TARGET_HZ  8000.0f

// Phase-alignment allpasses.  Band 1's matches its post-lowpass lag to band
// 0's so the two subs stay in quadrature and their sum does not depend on the
// flip-flop parity the previous note left behind.  With band 2 on, band 2 is
// parity-slaved to band 0 and both corners move (offline model, see the spec).
#define SUBHARM_ALIGN_AP1_2B_HZ  160.0f
#define SUBHARM_ALIGN_AP1_3B_HZ  105.0f
#define SUBHARM_ALIGN_AP2_HZ     130.0f

// LF boost bell: centred between the synthesized sub and the program mid-bass.
#define SUBHARM_BOOST_HZ        70.0f
#define SUBHARM_BOOST_Q          0.9f

// Divider envelope follower time constant (arming threshold = env / 4).
#define SUBHARM_ENV_TAU_MS      40.0f

// Selectivity detector: slow follower for attack detection (attack = fast env
// above SUBHARM_ATTACK_RATIO x slow), gate slew times, and the alive floor
// below which a band's hold counter is reset.
#define SUBHARM_SLOW_TAU_MS    400.0f
#define SUBHARM_SLOW_RISE_MS    30.0f
#define SUBHARM_GATE_OPEN_MS    30.0f
#define SUBHARM_GATE_CLOSE_MS  100.0f
#define SUBHARM_ATTACK_RATIO     2       // x2 = +6 dB
#define SUBHARM_ALIVE_FLOOR      1.0e-3f // -60 dBFS

// Sub ceiling limiter slews and the meter decay.
#define SUBHARM_CEIL_ATTACK_MS   3.0f
#define SUBHARM_CEIL_RELEASE_MS 200.0f
#define SUBHARM_METER_TAU_MS   300.0f

// Parameter limits and defaults.  A band at SUBHARM_LEVEL_MIN is off and its
// processing is skipped.  Level ceilings plus the Q28 input, band and sub
// clamps below keep every stored RP2040 value inside +/-8.0.
#define SUBHARM_LEVEL_MIN      -30.0f   // band level (dB); floor = band off
#define SUBHARM_LEVEL_MAX       12.0f
#define SUBHARM_BOOST_MIN        0.0f   // LF boost (dB); 0 = stage skipped
#define SUBHARM_BOOST_MAX        6.0f

// Selectivity: weight each band's sub toward percussive (short burst after an
// attack) or sustained (open only after the band has rung for the hold time).
#define SUBHARM_SELECT_ALL         0
#define SUBHARM_SELECT_PERCUSSIVE  1
#define SUBHARM_SELECT_SUSTAINED   2
#define SUBHARM_SELECT_MODE_MAX    2
#define SUBHARM_DEPTH_MIN        0.0f   // % of full gating
#define SUBHARM_DEPTH_MAX      100.0f
#define SUBHARM_HOLD_MIN        50.0f   // ms
#define SUBHARM_HOLD_MAX       400.0f

// Sub ceiling: soft limiter on the synthesized sub before it is mixed in.
// 0 dB means off (a ceiling at full scale limits nothing useful).
#define SUBHARM_CEILING_MIN    -40.0f   // dBFS
#define SUBHARM_CEILING_MAX      0.0f   // = off

#define SUBHARM_DEFAULT_LOW          0.0f
#define SUBHARM_DEFAULT_HIGH         0.0f
#define SUBHARM_DEFAULT_TOP          SUBHARM_LEVEL_MIN   // third band ships off
#define SUBHARM_DEFAULT_BOOST        0.0f
#define SUBHARM_DEFAULT_OUTPUT_MASK 0xFFFFu
#define SUBHARM_DEFAULT_SELECT_MODE  SUBHARM_SELECT_ALL
#define SUBHARM_DEFAULT_DEPTH      100.0f
#define SUBHARM_DEFAULT_HOLD_MS    150.0f
#define SUBHARM_DEFAULT_CEILING      0.0f
#define SUBHARM_DEFAULT_LINK_PAIRS   true

// Per-packet flag snapshot (read live like the mask, no recompute).
#define SUBHARM_FLAG_LINK  0x01u
#define SUBHARM_FLAG_SOLO  0x02u

// Configuration (persisted to flash / wire, except `solo`).
typedef struct {
    bool     enabled;
    float    low_db;         // 24-36 Hz band level (SUBHARM_LEVEL_MIN = off)
    float    high_db;        // 36-56 Hz band level (SUBHARM_LEVEL_MIN = off)
    float    boost_db;       // LF boost bell gain (0 = off)
    uint16_t output_mask;    // bit k = process output channel k
    float    top_db;         // 56-80 Hz band level (SUBHARM_LEVEL_MIN = off)
    uint8_t  select_mode;    // SUBHARM_SELECT_*
    float    select_depth;   // 0..100 %: how far the selected-off material is gated
    float    select_hold_ms; // percussive burst length / sustained onset delay
    float    ceiling_db;     // sub ceiling in dBFS; 0 = off
    bool     link_pairs;     // synthesize each S/PDIF pair from its mono sum
    // Monitoring only: masked outputs carry just the synthesized sub.  Runtime
    // state, never written to a preset slot or the bulk wire format, so no
    // saved configuration can boot with the program signal removed.
    bool     solo;
} SubharmConfig;

// ---------------------------------------------------------------------------
// Number type.  The kernel is written once against these helpers; RP2350
// runs it in float, RP2040 in Q28 through fast_mul_q28.
// ---------------------------------------------------------------------------
#if PICO_RP2350
typedef float sh_num_t;
#define SH_ZERO 0.0f
#define SH_ONE  1.0f
static inline sh_num_t sh_mul(sh_num_t a, sh_num_t b) { return a * b; }
static inline sh_num_t sh_twice(sh_num_t v)           { return 2.0f * v; }
static inline sh_num_t sh_half(sh_num_t v)            { return 0.5f * v; }
static inline sh_num_t sh_quarter(sh_num_t v)         { return 0.25f * v; }
static inline sh_num_t sh_from_pipe(sh_num_t v)       { return v; }
static inline sh_num_t sh_to_pipe(sh_num_t v)         { return v; }
static inline sh_num_t sh_add_sat(sh_num_t a, sh_num_t b) { return a + b; }
static inline sh_num_t sh_abs(sh_num_t v)             { return fabsf(v); }
static inline sh_num_t sh_band_limit(sh_num_t v)      { return v; }
static inline sh_num_t sh_input_limit(sh_num_t v)     { return v; }
static inline sh_num_t sh_band_out_limit(sh_num_t v)  { return v; }
static inline sh_num_t sh_sub_limit(sh_num_t v)       { return v; }
static inline sh_num_t sh_ratio(sh_num_t a, sh_num_t b) { return a / b; }
#else
typedef int32_t sh_num_t;
#define SH_ZERO 0
#define SH_ONE  (1 << FILTER_SHIFT)
int32_t fast_mul_q28(int32_t a, int32_t b);   // dsp_pipeline.c
static inline sh_num_t sh_mul(sh_num_t a, sh_num_t b) { return fast_mul_q28(a, b); }
static inline sh_num_t sh_twice(sh_num_t v)           { return v * 2; }
static inline sh_num_t sh_half(sh_num_t v)            { return v >> 1; }
static inline sh_num_t sh_quarter(sh_num_t v)         { return v >> 2; }
// The sub path runs with 0 dBFS = 1.0 Q28; pipeline samples reach it at 2^29.
static inline sh_num_t sh_from_pipe(sh_num_t v)       { return v >> 1; }
static inline sh_num_t sh_to_pipe(sh_num_t v)         { return v * 2; }
// Dry + sub can pass the int32 limit on a hot input; saturate instead of wrapping.
static inline sh_num_t sh_add_sat(sh_num_t a, sh_num_t b) {
    int32_t s = (int32_t)((uint32_t)a + (uint32_t)b);
    if (((a ^ s) & (b ^ s)) < 0) s = a < 0 ? INT32_MIN : INT32_MAX;
    return s;
}
static inline sh_num_t sh_abs(sh_num_t v)             { return v < 0 ? -v : v; }
// Clamp a band signal to +/-1.0 before the divider.  fast_mul_q28 wraps past
// +/-8.0, so the divider input must be bounded for the +6 dB band level and
// +6 dB bell to stay inside the representable range on hot inputs.
static inline sh_num_t sh_band_limit(sh_num_t v) {
    const int32_t one = 1 << FILTER_SHIFT;
    return v > one ? one : (v < -one ? -one : v);
}
// Sub-path input clamp (+/-3.0), per-band clamp (+/-2.5, three bands at
// +12 dB on a hot input would otherwise wrap the sum) and sub-sum clamp
// (+/-2.0).  The dry path is unclamped.
static inline sh_num_t sh_input_limit(sh_num_t v) {
    const int32_t lim = 3 << FILTER_SHIFT;
    return v > lim ? lim : (v < -lim ? -lim : v);
}
static inline sh_num_t sh_band_out_limit(sh_num_t v) {
    const int32_t lim = 5 << (FILTER_SHIFT - 1);
    return v > lim ? lim : (v < -lim ? -lim : v);
}
static inline sh_num_t sh_sub_limit(sh_num_t v) {
    const int32_t lim = 2 << FILTER_SHIFT;
    return v > lim ? lim : (v < -lim ? -lim : v);
}
// Q28 quotient a/b for 0 < a < b (ceiling gain).  Normalizes b to 16 bits so
// a single 32-bit hardware divide suffices; no libgcc 64-bit divide or clz,
// which would call into flash from the RAM-resident kernel.
static inline sh_num_t sh_ratio(sh_num_t a, sh_num_t b) {
    uint32_t bn = (uint32_t)b;
    int sh = 0;
    while (!(bn & 0x80000000u)) { bn <<= 1; sh++; }
    uint32_t q = ((uint32_t)a << 4) / (bn >> 16);   // a/b * 2^(20-sh)
    return (int32_t)(q << (8 + sh));
}
#endif

// TPT state-variable filter (Cytomic form).  a3 is not stored: with a2 = g a1
// and a3 = g a2 the lowpass update folds to v2 = ic2 + g v1, one multiply
// fewer per step.  k is the damping the highpass/bell outputs need.
typedef struct {
    sh_num_t a1, a2, g, k;
} SubharmSvf;

typedef struct {
    sh_num_t ic1, ic2;
} SubharmSvfState;

typedef struct {
    SubharmSvf lp;          // post-divider lowpass (low rate)
    sh_num_t   gain;        // linear band level; SH_ZERO = band off (skipped)
    sh_num_t   align_g;     // one-pole allpass G = g/(1+g); SH_ZERO = none
} SubharmBandCoeffs;

typedef struct {
    // Full rate
    SubharmSvf aa;          // LP2 at SUBHARM_BAND_HI_HZ: band top + anti-alias
    SubharmSvf bell;        // LF boost bell
    sh_num_t   bell_m1;     // k * (A^2 - 1) bell mix; SH_ZERO = boost off (skipped)
    sh_num_t   inv_decim;   // 1/D for the interpolation step
    uint8_t    decim;       // D: low-rate tick every D samples
    uint8_t    select_mode; // SUBHARM_SELECT_*
    uint16_t   hold;        // selectivity hold in low-rate samples
    // Low rate
    SubharmSvf hp_lo;       // HP2 at SUBHARM_BAND_LO_HZ (highpass output)
    SubharmSvf split2;      // at MID2: LP out -> split1, HP out = band 2
    SubharmSvf split1;      // at MID1: LP out = band 0, HP out = band 1
    SubharmBandCoeffs band[SUBHARM_NUM_BANDS];
    sh_num_t   env_decay;   // per-sample divider envelope decay
    sh_num_t   slow_decay;  // selectivity slow follower decay
    sh_num_t   slow_rise;   // selectivity slow follower rise coefficient
    sh_num_t   gate_open;   // selectivity weight slew toward 1 (sustained)
    sh_num_t   gate_close;  // selectivity weight slew toward 0
    sh_num_t   depth;       // 0..1 gating depth
    sh_num_t   alive_floor; // band envelope below this resets the hold counter
    sh_num_t   ceil;        // sub ceiling, linear; SH_ZERO = off (skipped)
    sh_num_t   ceil_env_rel;// ceiling detector release
    sh_num_t   ceil_attack; // ceiling gain slew down
    sh_num_t   ceil_release;// ceiling gain slew up
    sh_num_t   meter_decay; // sub meter decay per low-rate sample
} SubharmCoeffs;

typedef struct {
    SubharmSvfState lp;
    sh_num_t env;           // band peak follower
    sh_num_t env_slow;      // selectivity slow follower
    sh_num_t weight;        // selectivity gate weight 0..1
    sh_num_t align;         // allpass state
    uint16_t hold_cnt;      // low-rate samples since attack / since alive
    uint16_t period_cnt;    // low-rate samples since the last flip
    uint16_t period;        // last measured flip period (0 = none yet)
    uint8_t  armed;         // band dipped below -env/4 since the last flip
    uint8_t  neg;           // divider polarity: 1 = inverting this cycle
} SubharmBandState;

typedef struct {
    SubharmSvfState  aa;
    SubharmSvfState  hp_lo, split2, split1;
    SubharmBandState band[SUBHARM_NUM_BANDS];
    sh_num_t         ceil_env;   // ceiling detector peak
    sh_num_t         ceil_gain;  // ceiling gain (SH_ONE when idle)
    sh_num_t         cur, step;  // interpolator: current sub and per-sample step
    sh_num_t         meter;      // decaying peak of the mixed-in sub (0..1)
    uint8_t          active;     // 0 = fresh (reset or never run), 1 = running
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

// Clear one output's state (ceiling gain idles at unity, not zero).  A fresh
// output is seeded from its running pair partner when it re-enters, so two
// outputs carrying the same program never restart their dividers apart.
static inline void subharm_reset_output_state(SubharmOutputState *st) {
    memset(st, 0, sizeof(SubharmOutputState));
    st->ceil_gain = SH_ONE;
}

// Per-packet flag snapshot from the live config (link + solo).
static inline uint8_t subharm_flags_snapshot(void) {
    return (subharm_config.link_pairs ? SUBHARM_FLAG_LINK : 0u)
         | (subharm_config.solo ? SUBHARM_FLAG_SOLO : 0u);
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

// Per-output sub meter as a status-packet style peak (0..32767): decaying
// peak of the synthesized sub that was mixed into that output.
uint16_t subharm_meter_u16(uint8_t out);

// Run one output's block in place; with `buf_b` the two buffers are a linked
// pair fed one sub from their mono sum (state `st`, mirrored into `st_b`).
// `phase0` is the packet's decimation phase, common to every output.
void subharm_process_block(const SubharmCoeffs * __restrict c,
                           SubharmOutputState * __restrict st,
                           SubharmOutputState * __restrict st_b,
                           sh_num_t * __restrict buf, sh_num_t * __restrict buf_b,
                           uint32_t n, uint8_t phase0, bool solo);

// Pre-crossover pass over outputs first_out..last_out (inclusive): linked
// pairs, single outputs, or a state reset for everything not eligible.
// Eligibility = coeffs && mask bit && matrix-enabled && !mute && !RAW.
void subharm_process_outputs(const SubharmCoeffs *coeffs, uint16_t mask, uint8_t flags,
                             uint8_t phase0, int first_out, int last_out,
                             sh_num_t (*buf_out)[AUDIO_BUFFER_SAMPLES], uint32_t n);

// Decimation phase for a packet that starts at absolute sample `sample_ctr`
// (Core 0 keeps the counter; Core 1 receives the phase in Core1EqWork).
static inline uint8_t subharm_packet_phase(const SubharmCoeffs *c, uint32_t sample_ctr) {
    return (c && c->decim > 1) ? (uint8_t)(sample_ctr % c->decim) : 0;
}

#endif // SUBHARM_H
