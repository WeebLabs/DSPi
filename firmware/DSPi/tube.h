#ifndef TUBE_H
#define TUBE_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "config.h"

// Tube preamp emulation: biased asymmetric waveshaper with supply sag and an
// optional transformer stage, per output.  Module pattern follows psybass
// (shared published coeffs, per-output state, output mask).
// Design: Documentation/Features/tube_preamp_spec.md.

// Indexed parameter ids (REQ_SET/GET_TUBE_PARAM wValue).  Wire/flash order.
enum {
    TUBE_PARAM_ENABLED = 0,
    TUBE_PARAM_OUTPUT_MASK,
    TUBE_PARAM_TUBE_TYPE,
    TUBE_PARAM_DRIVE_DB,
    TUBE_PARAM_BIAS_PCT,
    TUBE_PARAM_ASYM_DB,
    TUBE_PARAM_HARDNESS_PCT,
    TUBE_PARAM_SAG_PCT,
    TUBE_PARAM_RECTIFIER,
    TUBE_PARAM_XFMR_ENABLED,
    TUBE_PARAM_XFMR_LF_HZ,
    TUBE_PARAM_XFMR_SAT_PCT,
    TUBE_PARAM_XFMR_HF_HZ,
    TUBE_PARAM_MIX_PCT,
    TUBE_PARAM_TRIM_DB,
    TUBE_NUM_PARAMS
};

// Tube styles (spec section 2.3).  0 = custom, rows never renumber.
#define TUBE_TYPE_CUSTOM         0
#define TUBE_TYPE_MAX           16
// Rectifier styles (spec section 2.9): solid state, GZ34, 5U4, 5Y3.
#define TUBE_RECT_SOLID_STATE    0
#define TUBE_RECT_MAX            3

// Parameter limits and defaults
#define TUBE_DRIVE_MIN           -6.0f   // knee 6 dB above full scale; floor keeps s_n = kn/(c1 m) <= 5.3 in Q28
#define TUBE_DRIVE_MAX           24.0f   // 10^(24/20) = 15.85: carried in Q24 on RP2040
#define TUBE_BIAS_MIN          -100.0f
#define TUBE_BIAS_MAX           100.0f
#define TUBE_ASYM_MIN           -12.0f
#define TUBE_ASYM_MAX            12.0f   // knee ratio 3.98 < 8.0 Q28 ceiling
#define TUBE_HARDNESS_MIN         0.0f
#define TUBE_HARDNESS_MAX       100.0f
#define TUBE_SAG_MIN              0.0f
#define TUBE_SAG_MAX            100.0f
#define TUBE_XFMR_LF_MIN         20.0f
#define TUBE_XFMR_LF_MAX        300.0f
#define TUBE_XFMR_SAT_MIN         0.0f
#define TUBE_XFMR_SAT_MAX       100.0f   // knee -18 dBFS: 1/ks = 7.94 < 8.0 Q28
#define TUBE_XFMR_HF_MIN       2000.0f
#define TUBE_XFMR_HF_MAX      20000.0f   // treated as bypass
#define TUBE_MIX_MIN              0.0f
#define TUBE_MIX_MAX            100.0f
#define TUBE_TRIM_MIN           -12.0f
#define TUBE_TRIM_MAX            12.0f

#define TUBE_DEFAULT_TUBE_TYPE      1     // 12AX7
#define TUBE_DEFAULT_DRIVE         -6.0f  // clean: 0.5 % THD at -12 dBFS, level-neutral
#define TUBE_DEFAULT_BIAS          10.0f  // 12AX7 row
#define TUBE_DEFAULT_ASYM           3.0f
#define TUBE_DEFAULT_HARDNESS      40.0f
#define TUBE_DEFAULT_SAG           15.0f
#define TUBE_DEFAULT_RECTIFIER      1     // GZ34
#define TUBE_DEFAULT_XFMR_LF       80.0f
#define TUBE_DEFAULT_XFMR_SAT      30.0f
#define TUBE_DEFAULT_XFMR_HF    20000.0f
#define TUBE_DEFAULT_MIX          100.0f
#define TUBE_DEFAULT_TRIM           0.0f
#define TUBE_DEFAULT_OUTPUT_MASK 0xFFFFu

#define TUBE_DC_BLOCK_HZ          5.0f
#define TUBE_METER_TAU_MS       300.0f
#define TUBE_SAG_DEPTH_MAX        0.9f   // gain never falls to zero

// RP2040 Q28 headroom clamps (spec section 7): fast_mul_q28 needs the two
// operand magnitudes to sum below 8.0, so the wet signal is bounded after
// the DC blocker and again ahead of the transformer HF one-pole.
#define TUBE_Q28_Y_LIM            3.4f   // 2 * 3.4 + a_lf < 8 for the one-pole difference
#define TUBE_Q28_Y2_LIM           3.4f

// Configuration (persisted to flash / wire).  Field order matches the wire
// section and the parameter index table.
typedef struct {
    bool     enabled;
    uint8_t  tube_type;      // 0 = custom, 1..TUBE_TYPE_MAX
    uint8_t  rectifier;      // 0..TUBE_RECT_MAX
    bool     xfmr_enabled;
    uint16_t output_mask;    // bit k = process output channel k
    float    drive_db;
    float    bias_pct;
    float    asym_db;
    float    hardness_pct;
    float    sag_pct;
    float    xfmr_lf_hz;
    float    xfmr_sat_pct;
    float    xfmr_hf_hz;
    float    mix_pct;
    float    trim_db;
} TubeConfig;

// Number type: the kernel is written once against these helpers.  RP2350
// runs it in float, RP2040 in Q28 through fast_mul_q28.
#if PICO_RP2350
typedef float tb_num_t;
#define TB_ZERO 0.0f
#define TB_ONE  1.0f
#else
typedef int32_t tb_num_t;
#define TB_ZERO 0
#define TB_ONE  (1 << FILTER_SHIFT)
int32_t fast_mul_q28(int32_t a, int32_t b);   // dsp_pipeline.c
#endif

// Shared coefficient set.  On RP2040, `m`, `sagk` and `bias` are Q24 raw
// values (value * 2^24) so drive up to 15.85 fits; everything else is Q28.
typedef struct {
    tb_num_t m;            // drive gain (knee fixed at t = 1)
    tb_num_t sagk;         // m * sag depth
    tb_num_t bias;         // operating point b in knee units
    tb_num_t ratio_n;      // positive knee / negative knee
    tb_num_t c1, c3, c5;   // blended polynomial p(t) = t (c1 + t^2 (c3 + t^2 c5))
    tb_num_t s_p, s_n;     // output scale per half, includes 1/m makeup (unity small-signal gain)
    tb_num_t v0;           // shaper output at rest, subtracted
    tb_num_t sag_att;      // envelope coefficients
    tb_num_t sag_rel;
    tb_num_t dc_r;         // DC blocker pole
    tb_num_t meter_decay;
    tb_num_t xf_a_lf;      // transformer split one-pole
    tb_num_t xf_inv_ks;    // 1 / low-band knee (RP2040: Q26, shifted back in the kernel)
    tb_num_t xf_s;         // low-band knee / 1.5
    tb_num_t xf_a_hf;      // transformer HF one-pole (1.0 = bypass)
    tb_num_t dry_w;        // 1 - mix
    tb_num_t wet_w;        // mix * trim
    tb_num_t wet_lim;      // RP2040 wet clamp: (7.5 - 4 dry_w) / wet_w capped at Y2_LIM; unused on RP2350
    uint8_t  xfmr_on;
    uint8_t  sag_on;
} TubeCoeffs;

typedef struct {
    tb_num_t env;          // sag envelope of |t|
    tb_num_t dc_x1, dc_y1; // DC blocker
    tb_num_t xf_lp;        // transformer split state
    tb_num_t xf_hf;        // transformer HF one-pole state
    tb_num_t meter;        // decaying peak of |t| (0..1)
} TubeOutputState;

// Live configuration + main-loop recompute flag (defined in tube.c).
// Vendor SET handlers go through tube_set_param(); the main loop recomputes
// and publishes.  The audio path only ever reads the published pointer.
extern volatile TubeConfig tube_config;
extern volatile bool tube_update_pending;

// Per-output state, indexed by output channel.  Each output is only ever
// touched by the core that owns it in the current pipeline mode.
extern TubeOutputState tube_output_state[NUM_OUTPUT_CHANNELS];

// Published coefficient set the pipeline snapshots each packet; NULL = off.
extern volatile const TubeCoeffs *current_tube_coeffs;

static inline void tube_reset_output_state(TubeOutputState *st) {
    memset(st, 0, sizeof(TubeOutputState));
}

// Indexed parameter access for the vendor handlers and Control Surfaces.
// tube_set_param clamps, writes the config, raises the pending flag where
// needed, applies tube-type rows / custom reset, and emits change
// notifications for every field it changed.  Returns false for a bad index.
bool  tube_set_param(uint8_t index, float value);
bool  tube_get_param(uint8_t index, float *value);

// Compute a coefficient set from config (clamped) at the given sample rate.
void tube_compute_coefficients(TubeCoeffs *coeffs, const TubeConfig *config, float sample_rate);

// Recompute shared coefficients from config and publish current_tube_coeffs.
// Called from the main loop while audio runs; never touches per-output state.
void tube_apply_config(const TubeConfig *config, float sample_rate);

// Per-output saturation meter on the status-packet scale (0..32767).
uint16_t tube_meter_u16(uint8_t out);

// Run one output's block in place.  Non-inline RAM-resident kernel shared by
// every call site so its text is paid once.
void tube_process_output_block(const TubeCoeffs * __restrict c,
                               TubeOutputState * __restrict st,
                               tb_num_t * __restrict buf, uint32_t n);

#endif // TUBE_H
