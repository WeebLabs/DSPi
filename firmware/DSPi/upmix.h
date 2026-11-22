#ifndef UPMIX_H
#define UPMIX_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// Stereo upmixer (RP2350 only): derives Centre / Left-Surround / Right-Surround
// virtual source channels from the stereo input pair.
//
// Runs as a pipeline pass between the leveller and the matrix, ONLY when the
// active input is stereo (active_input_channel_count() == 2) and the sample
// rate is 48 kHz or below (ADAT-style park above; rings are sized for 48 kHz).  It reads the
// post-EQ/leveller stereo bus (buf_l/buf_r), writes the derived channels into
// the otherwise-idle multichannel input rows (buf_in_ext[0..2] = matrix source
// rows 2..4), and applies centre removal to L/R in place.  Centre removal is
// the ONLY thing the pass does to the mains; the surround engine never writes
// L/R, so with the centre engine OFF (or width at 100%) the stereo pair passes
// through bit-exact and the surrounds are derived from it.  The matrix then
// treats the derived channels as ordinary sources: routing, crosspoint gains,
// and the whole per-output chain (PEQ, crossover, delay, gain, loudness) are
// reused unchanged.  In multichannel input modes those rows carry real inputs
// and the upmixer parks (state reset, rows not exposed).
//
// Two independent engines:
//
//   Centre engine (owns row 2 whenever the pass runs):
//     OFF     : no extraction and no removal.  Row 2 goes silent but stays
//               reserved so Ls/Rs keep matrix rows 3/4, and L/R are untouched,
//               which is the "surround effects only, mains as recorded" setup.
//     PASSIVE : C = 0.7071*(L+R), fixed gain (power-preserving for
//               uncorrelated content; correlated content sums to +3 dB).
//     ADAPTIVE: running normalized cross-correlation and L/R balance (one-pole
//               estimators on a bass-cut detector path) drive the centre gain
//               through a threshold gate and attack/release ballistics, so only
//               genuinely centre-panned correlated content is extracted and the
//               image does not pump.
//     Extracted centre energy is subtracted from L/R (scaled by width) so a
//     physical centre speaker and the L/R phantom don't comb-filter.
//     Both modes: a broad presence bell (TPT SVF, 3 kHz, Q 0.6, +-12 dB) on
//     the extracted C moves voices forward/back (Syn-style presence control).
//
//   Surround engine (rows 3..4 when mode != OFF):
//     PASSIVE : difference feed, Ls = 0.7071*(L-R) and Rs mirrored (-S).
//     ADAPTIVE: prediction-residual feed (Ls = L' - w*R', Rs mirrored), side-
//               specific balance gate, front/back dominance and a transient
//               duck.  L'/R' are band-limited and delayed before the feed, so
//               steering sees the Haas delay ahead (see upmixer_spec.md).
//     Both modes share the conditioning: 2nd-order Butterworth HP/LP band
//     limit, Haas delay, mirrored-gain Schroeder allpass decorrelators, with the
//     same total delay, so routing "Upmix Ls/Rs" to output slots is the setup.
//
// Steering is pure gain (zero latency) on C/L/R; the surround Haas delay is a
// deliberate, identical-per-source feature, so inter-output-slot alignment is
// preserved by construction (CLAUDE.md hard rule).
//
// Module pattern follows psybass: volatile config + update_pending flag,
// main-loop recompute, double-buffered published coefficient pointer (NULL =
// disabled).  All state is Core 0 only (the pass runs before the matrix).

#if PICO_RP2350

// Derived-channel matrix rows (contiguous above the stereo pair)
#define UPMIX_ROW_C          2
#define UPMIX_ROW_LS         3
#define UPMIX_ROW_RS         4
#define UPMIX_NUM_DERIVED    3
_Static_assert(NUM_STEREO_INPUTS + UPMIX_NUM_DERIVED <= NUM_INPUT_CHANNELS,
               "upmix derived rows must fit in the matrix source space");

// Engine modes
#define UPMIX_CENTER_PASSIVE    0
#define UPMIX_CENTER_ADAPTIVE   1
// OFF is appended rather than renumbered to 0 (which would mirror the surround
// enum): the vendor interface has no per-command version negotiation, so moving
// PASSIVE/ADAPTIVE would silently remap every existing host and saved preset.
#define UPMIX_CENTER_OFF        2
#define UPMIX_SURROUND_OFF      0
#define UPMIX_SURROUND_PASSIVE  1
#define UPMIX_SURROUND_ADAPTIVE 2

// Parameter limits and defaults
#define UPMIX_STRENGTH_MIN        0.0f   // centre extraction strength (%)
#define UPMIX_STRENGTH_MAX      100.0f
#define UPMIX_WIDTH_MIN           0.0f   // centre width: 0 = full removal from L/R
#define UPMIX_WIDTH_MAX         100.0f   // 100 = L/R untouched (phantom kept)
#define UPMIX_THRESH_MIN          0.0f   // correlation gate (%)
#define UPMIX_THRESH_MAX         95.0f
#define UPMIX_ATTACK_MIN          1.0f   // centre gain ballistics (ms)
#define UPMIX_ATTACK_MAX        500.0f
#define UPMIX_RELEASE_MIN         5.0f
#define UPMIX_RELEASE_MAX      2000.0f
#define UPMIX_DET_HPF_MIN        20.0f   // detector bass cut (Hz)
#define UPMIX_DET_HPF_MAX      1000.0f
#define UPMIX_SUR_DELAY_MIN       0.0f   // surround Haas delay (ms)
#define UPMIX_SUR_DELAY_MAX      20.0f
#define UPMIX_SUR_HPF_MIN        20.0f   // surround band-limit corners (Hz)
#define UPMIX_SUR_HPF_MAX      2000.0f
#define UPMIX_SUR_LPF_MIN      1000.0f
#define UPMIX_SUR_LPF_MAX     20000.0f
#define UPMIX_DECORR_MIN          0.0f   // decorrelator amount (%); G = 0.5 * pct/100
#define UPMIX_DECORR_MAX        100.0f
#define UPMIX_PRESENCE_MIN      -12.0f   // centre presence bell gain (dB); Syn-style
#define UPMIX_PRESENCE_MAX       12.0f   // "presence" knob, both centre modes

#define UPMIX_DEFAULT_CENTER_MODE    UPMIX_CENTER_ADAPTIVE
#define UPMIX_DEFAULT_SURROUND_MODE  UPMIX_SURROUND_ADAPTIVE
#define UPMIX_DEFAULT_STRENGTH     100.0f
#define UPMIX_DEFAULT_WIDTH         25.0f
#define UPMIX_DEFAULT_THRESH        30.0f
#define UPMIX_DEFAULT_ATTACK        10.0f
#define UPMIX_DEFAULT_RELEASE      100.0f
#define UPMIX_DEFAULT_DET_HPF      200.0f
#define UPMIX_DEFAULT_SUR_DELAY     12.0f
#define UPMIX_DEFAULT_SUR_HPF      300.0f
#define UPMIX_DEFAULT_SUR_LPF     7000.0f
#define UPMIX_DEFAULT_DECORR        90.0f
#define UPMIX_DEFAULT_PRESENCE       0.0f
#define UPMIX_DEFAULT_SRC_A          0      // source pair: input channels
#define UPMIX_DEFAULT_SRC_B          1
#define UPMIX_DEFAULT_ACTIVE_COUNTS  0x01   // stereo input only (v1 activation)

// Config profiles: the lowest enabled one whose active_counts holds the live
// input count runs (Documentation/Features/upmixer_v2_spec.md section 3.2).
#define UPMIX_NUM_PROFILES     2
#define UPMIX_COUNTS_MASK      0x0F   // bit k = 2*(k+1) active input channels

// active_counts bit for a live input count; 0 for a count no profile can name.
static inline uint8_t upmix_count_bit(uint8_t n_inputs) {
    return (n_inputs >= 2 && n_inputs <= 8 && !(n_inputs & 1))
           ? (uint8_t)(1u << (n_inputs / 2 - 1)) : 0;
}

// Clamp a centre-mode value arriving from wire, bulk, or flash.  Out-of-range
// falls back to the default instead of the top of the range (the surround
// clamp's convention): OFF is the top value here, and a corrupt byte must not
// silently switch the centre engine off.
static inline uint8_t upmix_clamp_center_mode(long m) {
    return (m >= 0 && m <= UPMIX_CENTER_OFF) ? (uint8_t)m : UPMIX_DEFAULT_CENTER_MODE;
}

// Fixed internals
#define UPMIX_CORR_TAU_MS       100.0f   // correlation/balance estimator time constant
#define UPMIX_DOM_TAU_MS         40.0f   // front/back dominance smoother
#define UPMIX_DOM_CLIP_GAIN    1024.0f   // pre-clip gain (makes it a smoothed sign)
#define UPMIX_FB_NEUTRAL         0.75f   // adaptive surround gain at neutral front/back
#define UPMIX_W_TAU_MS           40.0f   // residual-weight / balance estimators
#define UPMIX_W_FLOOR          0.5623f   // weight floor (PLII 0.4898/0.871): covers estimator lag
#define UPMIX_W_CEIL             4.0f    // weight ceiling: bounds the noise gain of -w*R'
#define UPMIX_W_EPS             1e-3f    // weight regulariser, relative to ll+rr
#define UPMIX_GATE_K             0.75f   // quiet-side gate exponent (dominant side uses 1)
#define UPMIX_FLOOR_Q            0.0f    // floor relax exponent; above 0 leaks centre into the quiet side
#define UPMIX_DUCK_THR           4.0f    // fast/slow power ratio that counts as an onset
#define UPMIX_DUCK_DEPTH         0.5f    // duck gain = (thr/ratio)^depth above threshold
#define UPMIX_DUCK_FAST_MS        2.0f
#define UPMIX_DUCK_SLOW_MS       60.0f
#define UPMIX_DUCK_REL_MS        60.0f
#define UPMIX_DECORR_DELAY_MS    10.0f   // Schroeder allpass delay (patent value)
#define UPMIX_PRESENCE_HZ      3000.0f   // presence bell centre (vocal presence region)
#define UPMIX_PRESENCE_Q          0.6f   // broad bell, roughly 1.5-6 kHz
#define UPMIX_FADE_MS            10.0f   // activation fade-in
#define UPMIX_RATE_MAX        48000.0f   // parks above this (rings sized for 48 kHz)
#define UPMIX_HAAS_RING        1024      // samples; holds the 20 ms delay ceiling at 48 kHz
#define UPMIX_AP_RING           512      // samples; holds the 10 ms decorrelator at 48 kHz
#define UPMIX_CHUNK              32      // samples per render chunk where output is staged

// Generated channels, the out[] rows of upmix_render().
enum {
    UPMIX_GEN_A = 0,   // source A after centre removal (v1 L')
    UPMIX_GEN_B,       // source B after centre removal (v1 R')
    UPMIX_GEN_C,       // extracted centre
    UPMIX_GEN_SA,      // side ambience, A side (v1 Ls)
    UPMIX_GEN_SB,      // side ambience, B side (v1 Rs)
    UPMIX_NUM_GEN
};

// Configuration (persisted to flash / wire)
typedef struct {
    bool enabled;
    uint8_t center_mode;       // UPMIX_CENTER_*
    uint8_t surround_mode;     // UPMIX_SURROUND_*
    float strength_pct;
    float center_width_pct;
    float corr_threshold_pct;
    float attack_ms;
    float release_ms;
    float detector_hpf_hz;
    float surround_delay_ms;
    float surround_hpf_hz;
    float surround_lpf_hz;
    float decorr_pct;
    float presence_db;         // centre presence bell (Syn-style), both centre modes
    uint8_t src_a, src_b;      // source pair, input channels (A plays L, B plays R)
    uint8_t active_counts;     // input counts this profile serves (upmix_count_bit)
} UpmixConfig;

// Wire image of UpmixConfig for FX.UPMIX_CONFIG (44 bytes).
// presence_q1 (V26+, was reserved) carries presence_db in 0.5 dB steps
// (int8, dB * 2, -24..+24); older V25 hosts wrote 0 = 0 dB, so the byte is
// backward-compatible in both directions and no struct size changes.
typedef struct __attribute__((packed)) {
    uint8_t enabled;
    uint8_t center_mode;
    uint8_t surround_mode;
    int8_t presence_q1;
    float strength_pct;
    float center_width_pct;
    float corr_threshold_pct;
    float attack_ms;
    float release_ms;
    float detector_hpf_hz;
    float surround_delay_ms;
    float surround_hpf_hz;
    float surround_lpf_hz;
    float decorr_pct;
} UpmixConfigPacket;
_Static_assert(sizeof(UpmixConfigPacket) == 44, "UpmixConfigPacket wire size");

// Parameter ids, in the order of FX.UPMIX_ENABLE..FX.UPMIX_PRESENCE.  All values
// travel as a 4-byte float; mode/enable params are rounded to integer on SET.
enum {
    UPMIX_PARAM_ENABLED       = 0,
    UPMIX_PARAM_CENTER_MODE   = 1,
    UPMIX_PARAM_SURROUND_MODE = 2,
    UPMIX_PARAM_STRENGTH      = 3,
    UPMIX_PARAM_CENTER_WIDTH  = 4,
    UPMIX_PARAM_THRESHOLD     = 5,
    UPMIX_PARAM_ATTACK        = 6,
    UPMIX_PARAM_RELEASE       = 7,
    UPMIX_PARAM_DET_HPF       = 8,
    UPMIX_PARAM_SUR_DELAY     = 9,
    UPMIX_PARAM_SUR_HPF       = 10,
    UPMIX_PARAM_SUR_LPF       = 11,
    UPMIX_PARAM_DECORR        = 12,
    UPMIX_PARAM_PRESENCE      = 13,
    UPMIX_PARAM_COUNT
};

// Wire encoding for the presence byte (0.5 dB steps; see UpmixConfigPacket)
static inline int8_t upmix_presence_encode(float db) {
    db = db < UPMIX_PRESENCE_MIN ? UPMIX_PRESENCE_MIN
       : (db > UPMIX_PRESENCE_MAX ? UPMIX_PRESENCE_MAX : db);
    return (int8_t)lrintf(db * 2.0f);
}
static inline float upmix_presence_decode(int8_t q1) {
    return 0.5f * (float)q1;
}

// Live telemetry for FX.UPMIX_STATUS (16 bytes)
typedef struct __attribute__((packed)) {
    uint8_t active;            // 1 = processing this packet stream
    uint8_t parked_reason;     // 0 = active, 1 = disabled, 2 = no profile serves
                               // this input count, 3 = sample rate above 48 kHz,
                               // 4 = a source channel is not present
    int16_t corr_q14;          // smoothed L/R correlation, Q14 (-16384..16384)
    int16_t balance_q14;       // smoothed |L/R| balance, Q14 (0..16384)
    uint16_t center_gain_q15;  // smoothed centre extraction gain, Q15
    uint16_t ls_gain_q15;      // surround steering gains, Q15
    uint16_t rs_gain_q15;
    uint8_t reserved[4];
} UpmixStatus;
_Static_assert(sizeof(UpmixStatus) == 16, "UpmixStatus wire size");

// Published coefficient set; snapshot once per packet.  NULL = disabled.
typedef struct {
    uint8_t profile;           // index of the profile it was computed from
    uint8_t src_a, src_b;      // source pair (< NUM_INPUT_CHANNELS)
    uint8_t active_counts;     // masked to UPMIX_COUNTS_MASK
    uint8_t center_mode;
    uint8_t surround_mode;
    uint8_t n_derived;         // 1 (C only) or 3 (C + Ls/Rs)
    // Centre engine
    float strength;            // 0..1
    float width;               // 0..1 (residual retention)
    float corr_thresh;         // 0..1
    float inv_thresh_range;    // 1 / (1 - corr_thresh)
    float alpha_att, alpha_rel;// per-sample gain smoothing retentions
    float alpha_corr;          // per-sample estimator retention
    float det_hp_a;            // detector one-pole HP coefficient
    float pres_a1, pres_a2, pres_a3, pres_m1;  // centre presence bell, TPT SVF
                                               // (m1 = 0 at 0 dB: exact passthrough)
    // Adaptive surround
    float alpha_dom;           // per-sample front/back dominance retention
    float alpha_w;             // per-sample residual-weight estimator retention
    float duck_af, duck_as;    // duck fast/slow envelope coefficients
    float alpha_drel;          // per-sample duck release retention
    // Surround engine
    float ls_cl, ls_cr;        // passive feed: Ls = ls_cl*L' + ls_cr*R' (Rs mirrored)
    float ap_g;                // Schroeder allpass gain (+ on Ls, - on Rs)
    uint32_t haas_delay;       // samples, < UPMIX_HAAS_RING
    uint32_t ap_delay;         // samples, < UPMIX_AP_RING
    float shp_a1, shp_a2, shp_a3, shp_k;  // surround HP, TPT SVF Butterworth
    float slp_a1, slp_a2, slp_a3;         // surround LP, TPT SVF Butterworth
    float env_step;            // activation fade increment per sample
} UpmixCoeffs;

// Live configuration + main-loop recompute flag (defined in upmix.c).
// Vendor SET handlers write the config and raise the flag; the main loop
// recomputes coefficients and publishes.  The audio path only ever reads
// the published snapshot pointer.
extern volatile UpmixConfig upmix_profiles[UPMIX_NUM_PROFILES];
extern volatile bool upmix_update_pending;
extern volatile const UpmixCoeffs *current_upmix_coeffs[UPMIX_NUM_PROFILES];

// Factory defaults for one profile (disabled, v1 sources and activation).
void upmix_profile_defaults(volatile UpmixConfig *cfg);

// Compute a coefficient set from config (clamped) at the given sample rate.
void upmix_compute_coefficients(UpmixCoeffs *coeffs, const UpmixConfig *config, float sample_rate);

// Recompute every profile's coefficients and publish current_upmix_coeffs[].
// Called from the main loop while audio runs; never touches processing state.
void upmix_apply_profiles(float sample_rate);

// The profile that runs at this input count: the lowest published one whose
// active_counts holds it, or NULL.  Its sources may still be absent.
static inline const UpmixCoeffs *upmix_select(uint8_t n_inputs) {
    uint8_t bit = upmix_count_bit(n_inputs);
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) {
        const UpmixCoeffs *c = (const UpmixCoeffs *)current_upmix_coeffs[p];
        if (c && (c->active_counts & bit)) return c;
    }
    return NULL;
}

// Coefficients the v1 row bridge runs this packet, or NULL.  The bridge writes
// matrix rows 2..4, which carry real inputs above stereo; removed in phase 3.
static inline const UpmixCoeffs *upmix_bridge_coeffs(uint8_t n_inputs) {
    const UpmixCoeffs *c = upmix_select(n_inputs);
    if (!c || n_inputs != NUM_STEREO_INPUTS) return NULL;
    return (c->src_a < n_inputs && c->src_b < n_inputs) ? c : NULL;
}

// Packet control update: call once per packet with the packet length, then
// upmix_render() over consecutive chunks that sum to exactly that length, all
// with the same coeffs.  Output is identical for any chunking.  A set from a
// different profile than the running one restarts from reset.  Core 0 only.
void upmix_begin(const UpmixCoeffs *coeffs, uint32_t sample_count);

// Render the next n samples from the read-only sources a/b into out[UPMIX_GEN_*].
// No out row may alias a, b or another row.  SA/SB are written only while surround is on.
void upmix_render(const UpmixCoeffs *coeffs, const float *a, const float *b,
                  uint32_t n, float *const out[UPMIX_NUM_GEN]);

// v1 delivery bridge over begin/render: A'/B' back into the source rows a/b
// (which may be one row), C/SA/SB into the given rows.  Temporary; see
// Documentation/Features/upmixer_v2_plan.md.
void upmix_process_block(const UpmixCoeffs *coeffs,
                         float *a, float *b,
                         float *cbuf, float *lsbuf, float *rsbuf,
                         uint32_t sample_count);

// Reset all processing state (rings, estimators, envelope).  Cold; called on
// the running -> parked transition and when the running profile changes.
void upmix_reset_state(void);

// True while processing state holds audio (set by upmix_process_block).
extern bool upmix_state_dirty;

// Called by the pipeline every packet the pass is not running.  Inline so the
// steady-state disabled path is a load + branch in RAM, no flash call.
static inline void upmix_park(void) {
    if (upmix_state_dirty) upmix_reset_state();
}

// Fill a status packet from the live processing state at this input count.
void upmix_get_status(UpmixStatus *st, uint8_t n_inputs);

// Encode profile 0 as its wire image (the only profile on the wire until v2
// phase 5).  Shared by GET_CONFIG, bulk collect and notify, so the notify
// shadow matches collect.
void upmix_config_to_packet(UpmixConfigPacket *out);

// Push every upmix field through notify_param_write; only changed fields emit
// PARAM_CHANGED.  Every SET path must call this after writing a profile.
void upmix_notify_config(void);

#endif // PICO_RP2350

#endif // UPMIX_H
