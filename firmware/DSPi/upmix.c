/*
 * Upmixer pair engine and its output routing (RP2350 only).
 * Design: Documentation/Features/upmixer_v2_spec.md.
 *
 * Coefficient design notes:
 *   - All smoothers are one-pole with retention alpha = exp(-1/(fs*tau)).
 *     Centre-gain ballistics apply per block via powf(alpha, n) (leveller
 *     pattern), so behavior is independent of packet size.
 *   - Correlation is estimated on a bass-cut detector path (one-pole HP);
 *     long-wavelength content correlates by coincidence and would pump the
 *     steering (industry-standard "bass steering" mitigation).
 *   - Adaptive surround: prediction-residual feed, side-specific balance
 *     gate, front/back dominance and a transient duck, all steered from the
 *     band-limited L'/R' ahead of the Haas delay so control leads the audio.
 *     Design and measurements: Documentation/Features/upmixer_spec.md.
 *   - Surround band-limit filters are TPT SVF Butterworth (psybass form).
 *   - Centre presence bell is a Cytomic TPT SVF bell at fixed 3 kHz / Q 0.6,
 *     boost/cut symmetric (k = 1/(Q*A)); the knob is the gain only.
 */

#include <math.h>
#include <string.h>
#include "upmix.h"
#include "bulk_params.h"   // WireBulkParams offsets for change notifications
#include "notify.h"

#if PICO_RP2350

#define UM_PROFILE_DEFAULTS { \
    .enabled = false, \
    .center_mode = UPMIX_DEFAULT_CENTER_MODE, \
    .surround_mode = UPMIX_DEFAULT_SURROUND_MODE, \
    .strength_pct = UPMIX_DEFAULT_STRENGTH, \
    .center_width_pct = UPMIX_DEFAULT_WIDTH, \
    .corr_threshold_pct = UPMIX_DEFAULT_THRESH, \
    .attack_ms = UPMIX_DEFAULT_ATTACK, \
    .release_ms = UPMIX_DEFAULT_RELEASE, \
    .detector_hpf_hz = UPMIX_DEFAULT_DET_HPF, \
    .surround_delay_ms = UPMIX_DEFAULT_SUR_DELAY, \
    .surround_hpf_hz = UPMIX_DEFAULT_SUR_HPF, \
    .surround_lpf_hz = UPMIX_DEFAULT_SUR_LPF, \
    .decorr_pct = UPMIX_DEFAULT_DECORR, \
    .presence_db = UPMIX_DEFAULT_PRESENCE, \
    .back_offset_ms = UPMIX_DEFAULT_BACK_OFFSET, \
    .src_a = UPMIX_DEFAULT_SRC_A, \
    .src_b = UPMIX_DEFAULT_SRC_B, \
    .active_counts = UPMIX_DEFAULT_ACTIVE_COUNTS, \
}

// Live profiles; vendor handlers write them and raise the pending flag, the
// main loop recomputes + publishes.
_Static_assert(UPMIX_NUM_PROFILES == 2, "profile initializer lists two entries");
volatile UpmixConfig upmix_profiles[UPMIX_NUM_PROFILES] = { UM_PROFILE_DEFAULTS, UM_PROFILE_DEFAULTS };
volatile bool upmix_update_pending = false;

// Published coefficient sets the pipeline snapshots each packet; NULL = disabled.
volatile const UpmixCoeffs *current_upmix_coeffs[UPMIX_NUM_PROFILES];

// Double buffer per profile so a recompute never writes through a published pointer.
static UpmixCoeffs um_coeff_bufs[UPMIX_NUM_PROFILES][2];
static uint8_t um_coeff_idx[UPMIX_NUM_PROFILES];

void upmix_profile_defaults(volatile UpmixConfig *cfg) {
    const UpmixConfig d = UM_PROFILE_DEFAULTS;
    *cfg = d;
}

// Processing state (Core 0 only; the pass runs before the matrix).
typedef struct {
    // Detector / correlation estimator
    float det_lp_l, det_lp_r;      // one-pole HP states
    float acc_lr, acc_ll, acc_rr;  // smoothed products
    float g_c;                     // smoothed centre gain, 0..strength
    // Previous effective per-sample centre gains (ramp start points)
    float p_gc, p_rem;
    // Activation fade
    float env;
    // Telemetry snapshots
    float t_corr, t_bal;
    // Centre presence bell
    float pres_ic1, pres_ic2;      // TPT SVF integrators
    // Surround state, cleared as one unit whenever the surround engine
    // (re)starts so nothing stale can replay.
    struct {
        float dom_fb;                  // front/back dominance smoother
        float w_lr, w_ll, w_rr;        // raw band moments (weights, gate)
        float ef[2], es[2];            // duck fast/slow power envelopes
        float duck_att[2];             // duck attenuation, 0 = open (zero-init safe)
        float p_gls, p_grs, p_wl, p_wr;// ramp start points
        float senv;                    // adaptive rear fade, starts at emergence
        float benv, p_benv;            // adaptive back fade and its ramp start
        uint32_t fill;                 // samples written since restart, to the deepest tap
        bool back_on;                  // back pair rendered last packet
        float shp_ic1[2], shp_ic2[2];  // HP SVF integrators
        float slp_ic1[2], slp_ic2[2];  // LP SVF integrators
        float haas[2][UPMIX_HAAS_RING];// adaptive: band-limited L'/R'; passive: Ls/Rs
        float ap[2][UPMIX_AP_RING];
        float bap[2][UPMIX_BACK_AP_RING];
    } s;
    // Per-sample ramp cursors (value, step) set by upmix_begin() for the whole
    // packet and advanced by each upmix_render() chunk.
    struct {
        float gc, gc_d, rem, rem_d;
        float gls, gls_d, grs, grs_d;
        float wl, wl_d, wr, wr_d;
        float benv, benv_d;
    } rp;
    uint32_t widx;                 // shared ring write index
    bool sur_active;               // surround engine ran last packet
    uint8_t profile;               // profile whose coefficients it runs on
    uint8_t sur_mode;              // mode it ran in (ring contents differ per mode)
    volatile bool running;
} UpmixState;

static UpmixState um;

// Routing and lifecycle state (spec 6.2 to 6.4).  Engine resets leave it alone.
static struct {
    UpmixCoeffs latch;                            // set the engine runs; copied on publish
    float e[UPMIX_NUM_GEN][NUM_OUTPUT_CHANNELS];  // contribution gains, current value
    float w[NUM_OUTPUT_CHANNELS];                 // claim weights
    uint16_t gen_peak[UPMIX_NUM_GEN];
    uint32_t rate;
    uint8_t n_inputs;
    uint32_t clip_hold[UPMIX_NUM_GEN];            // samples left showing a clip bit
    bool held;                                    // latch valid: running or fading out
    bool active;                                  // targets on; false while fading out
} ur;

// One render chunk of every generated channel.
static float um_chunk[UPMIX_NUM_GEN][UPMIX_CHUNK];

// Host harness tap on each rendered chunk (tools/upmix_proto/harness).
#ifndef UPMIX_HOST_TAP
#define UPMIX_HOST_TAP(gen, off, k)
#endif

// Park fast-path flag (see upmix_park() in upmix.h)
bool upmix_state_dirty = false;

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void upmix_compute_coefficients(UpmixCoeffs *c, const UpmixConfig *cfg, float sample_rate) {
    memset(c, 0, sizeof(UpmixCoeffs));
    // Rates above 48 kHz are unsupported (rings sized for 48 kHz); park.
    if (!cfg->enabled || sample_rate < 1.0f || sample_rate > UPMIX_RATE_MAX) return;

    c->src_a = cfg->src_a < NUM_INPUT_CHANNELS ? cfg->src_a : UPMIX_DEFAULT_SRC_A;
    c->src_b = cfg->src_b < NUM_INPUT_CHANNELS ? cfg->src_b : UPMIX_DEFAULT_SRC_B;
    c->active_counts = cfg->active_counts & UPMIX_COUNTS_MASK;

    const float pi = 3.1415926535f;
    float fs = sample_rate;

    c->center_mode = upmix_clamp_center_mode(cfg->center_mode);
    c->surround_mode = cfg->surround_mode > UPMIX_SURROUND_ADAPTIVE
                       ? UPMIX_SURROUND_ADAPTIVE : cfg->surround_mode;
    for (int g = 0; g < UPMIX_NUM_GEN; g++) {
        int t = cfg->routes[g].trim_q1;
        t = t < UPMIX_TRIM_MIN_Q1 ? UPMIX_TRIM_MIN_Q1 : (t > UPMIX_TRIM_MAX_Q1 ? UPMIX_TRIM_MAX_Q1 : t);
        float gain = powf(10.0f, (float)t * (0.5f / 20.0f));   // exactly 1 at 0 dB
        c->route_mask[g] = cfg->routes[g].out_mask & UPMIX_OUT_MASK;
        c->route_gain[g] = (cfg->routes[g].flags & UPMIX_ROUTE_INVERT) ? -gain : gain;
        c->claim_mask |= c->route_mask[g];
    }

    c->strength = clampf(cfg->strength_pct, UPMIX_STRENGTH_MIN, UPMIX_STRENGTH_MAX) * 0.01f;
    // Both the C output and the removal from L/R scale by strength, so zeroing
    // it is the whole of centre OFF.
    if (c->center_mode == UPMIX_CENTER_OFF) c->strength = 0.0f;
    c->width    = clampf(cfg->center_width_pct, UPMIX_WIDTH_MIN, UPMIX_WIDTH_MAX) * 0.01f;
    c->corr_thresh = clampf(cfg->corr_threshold_pct, UPMIX_THRESH_MIN, UPMIX_THRESH_MAX) * 0.01f;
    c->inv_thresh_range = 1.0f / (1.0f - c->corr_thresh);

    float att_s = clampf(cfg->attack_ms, UPMIX_ATTACK_MIN, UPMIX_ATTACK_MAX) * 0.001f;
    float rel_s = clampf(cfg->release_ms, UPMIX_RELEASE_MIN, UPMIX_RELEASE_MAX) * 0.001f;
    c->alpha_att  = expf(-1.0f / (fs * att_s));
    c->alpha_rel  = expf(-1.0f / (fs * rel_s));
    c->alpha_corr = expf(-1.0f / (fs * UPMIX_CORR_TAU_MS * 0.001f));
    c->alpha_dom  = expf(-1.0f / (fs * UPMIX_DOM_TAU_MS * 0.001f));

    float det_hp = clampf(cfg->detector_hpf_hz, UPMIX_DET_HPF_MIN, UPMIX_DET_HPF_MAX);
    c->det_hp_a = 1.0f - expf(-2.0f * pi * det_hp / fs);

    // Centre presence bell: Cytomic TPT SVF (Simper), fixed corner/Q.
    // m1 = k*(A^2-1) is exactly 0 at 0 dB, so neutral is a true passthrough.
    {
        float pres = clampf(cfg->presence_db, UPMIX_PRESENCE_MIN, UPMIX_PRESENCE_MAX);
        float A = powf(10.0f, pres * (1.0f / 40.0f));
        float g = tanf(pi * UPMIX_PRESENCE_HZ / fs);
        float k = 1.0f / (UPMIX_PRESENCE_Q * A);
        c->pres_a1 = 1.0f / (1.0f + g * (g + k));
        c->pres_a2 = g * c->pres_a1;
        c->pres_a3 = g * c->pres_a2;
        c->pres_m1 = k * (A * A - 1.0f);
    }

    c->ls_cl = 0.7071f;   // passive difference feed (Rs mirrors to -S); adaptive
    c->ls_cr = -0.7071f;  // uses the residual feed instead

    c->alpha_w = expf(-1.0f / (fs * UPMIX_W_TAU_MS * 0.001f));
    c->duck_af = 1.0f - expf(-1.0f / (fs * UPMIX_DUCK_FAST_MS * 0.001f));
    c->duck_as = 1.0f - expf(-1.0f / (fs * UPMIX_DUCK_SLOW_MS * 0.001f));
    c->alpha_drel = expf(-1.0f / (fs * UPMIX_DUCK_REL_MS * 0.001f));

    c->ap_g = 0.5f * clampf(cfg->decorr_pct, UPMIX_DECORR_MIN, UPMIX_DECORR_MAX) * 0.01f;

    float dly = clampf(cfg->surround_delay_ms, UPMIX_SUR_DELAY_MIN, UPMIX_SUR_DELAY_MAX);
    uint32_t hd = (uint32_t)(dly * 0.001f * fs + 0.5f);
    c->haas_delay = hd > UPMIX_HAAS_RING - 1 ? UPMIX_HAAS_RING - 1 : hd;
    uint32_t ad = (uint32_t)(UPMIX_DECORR_DELAY_MS * 0.001f * fs + 0.5f);
    c->ap_delay = ad > UPMIX_AP_RING - 1 ? UPMIX_AP_RING - 1 : ad;
    float boff = clampf(cfg->back_offset_ms, UPMIX_BACK_OFFSET_MIN, UPMIX_BACK_OFFSET_MAX);
    uint32_t bd = c->haas_delay + (uint32_t)((UPMIX_BACK_GAP_MS + boff) * 0.001f * fs + 0.5f);
    c->back_delay = bd > UPMIX_HAAS_RING - 1 ? UPMIX_HAAS_RING - 1 : bd;   // clamped, never refused
    uint32_t bad = (uint32_t)(UPMIX_BACK_AP_MS * 0.001f * fs + 0.5f);
    c->bap_delay = bad > UPMIX_BACK_AP_RING - 1 ? UPMIX_BACK_AP_RING - 1 : bad;
    c->back = c->surround_mode != UPMIX_SURROUND_OFF
              && (c->route_mask[UPMIX_GEN_BA] | c->route_mask[UPMIX_GEN_BB]);

    // Surround band-limit: 2nd-order Butterworth TPT SVF (HP + LP)
    const float k = 1.4142135624f;
    float fhp = clampf(cfg->surround_hpf_hz, UPMIX_SUR_HPF_MIN, UPMIX_SUR_HPF_MAX);
    float flp = clampf(cfg->surround_lpf_hz, UPMIX_SUR_LPF_MIN, UPMIX_SUR_LPF_MAX);
    if (flp > 0.45f * fs) flp = 0.45f * fs;
    float g = tanf(pi * fhp / fs);
    c->shp_a1 = 1.0f / (1.0f + g * (g + k));
    c->shp_a2 = g * c->shp_a1;
    c->shp_a3 = g * c->shp_a2;
    c->shp_k = k;
    g = tanf(pi * flp / fs);
    c->slp_a1 = 1.0f / (1.0f + g * (g + k));
    c->slp_a2 = g * c->slp_a1;
    c->slp_a3 = g * c->slp_a2;

    c->env_step = 1.0f / (fs * UPMIX_FADE_MS * 0.001f);
    c->slew_step = 1000.0f / (fs * UPMIX_FADE_MS);   // exact 1/480 at 48 kHz
}

// Rate gate snapshot for status reporting (written by upmix_apply_profiles).
static bool um_rate_ok = true;
static uint32_t um_seq;

void upmix_apply_profiles(float sample_rate) {
    // Compute into each profile's inactive buffer, then publish the pointer
    // (psybass pattern).  NULL when disabled or above 48 kHz.
    um_rate_ok = sample_rate <= UPMIX_RATE_MAX;
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) {
        UpmixConfig cfg = upmix_profiles[p];
        UpmixCoeffs *next = &um_coeff_bufs[p][um_coeff_idx[p] ^ 1];
        upmix_compute_coefficients(next, &cfg, sample_rate);
        next->profile = (uint8_t)p;
        next->seq = ++um_seq;
        if (cfg.enabled && um_rate_ok) {
            um_coeff_idx[p] ^= 1;
            current_upmix_coeffs[p] = next;
        } else {
            current_upmix_coeffs[p] = NULL;
        }
    }
}

// RAM-resident: runs from the audio thread on the running -> parked
// transition; must not fetch from flash there.
DSP_TIME_CRITICAL
void upmix_reset_state(void) {
    memset(&um, 0, sizeof(um));
    upmix_state_dirty = false;
}

void upmix_get_status(UpmixStatus *st, uint8_t n_inputs, uint16_t out_enabled) {
    memset(st, 0, sizeof(*st));
    bool enabled = false;
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) enabled |= upmix_profiles[p].enabled;
    const UpmixCoeffs *sel = upmix_select(n_inputs);
    bool absent = sel && (sel->src_a >= n_inputs || sel->src_b >= n_inputs);
    // Parked only for want of audio: report what will run once packets flow.
    bool would_run = !ur.held && sel && !absent && (sel->claim_mask & out_enabled);
    st->active = ((ur.held && ur.active) || would_run) ? 1 : 0;
    st->parked_reason = st->active ? 0
                      : ur.held ? 5
                      : !enabled ? 1
                      : !um_rate_ok ? 3
                      : absent ? 4
                      : sel ? 6 : 2;
    st->corr_q14 = (int16_t)(clampf(um.t_corr, -1.0f, 1.0f) * 16384.0f);
    st->balance_q14 = (int16_t)(clampf(um.t_bal, 0.0f, 1.0f) * 16384.0f);
    st->center_gain_q15 = (uint16_t)(clampf(um.p_gc * 1.4142135624f, 0.0f, 1.0f) * 32767.0f);
    st->ls_gain_q15 = (uint16_t)(clampf(um.s.p_gls, 0.0f, 1.0f) * 32767.0f);
    st->rs_gain_q15 = (uint16_t)(clampf(um.s.p_grs, 0.0f, 1.0f) * 32767.0f);
    st->active_profile = ur.held ? ur.latch.profile : would_run ? sel->profile : 0xFF;
    for (int g = 0; g < UPMIX_NUM_GEN; g++)
        if (ur.clip_hold[g]) st->gen_clip |= (uint8_t)(1u << g);
    memcpy(st->gen_peak, ur.gen_peak, sizeof(st->gen_peak));
}

void upmix_config_to_packet(uint8_t p, UpmixConfigPacket *out) {
    volatile const UpmixConfig *cfg = &upmix_profiles[p < UPMIX_NUM_PROFILES ? p : 0];
    memset(out, 0, sizeof(*out));
    out->enabled            = cfg->enabled ? 1 : 0;
    out->center_mode        = cfg->center_mode;
    out->surround_mode      = cfg->surround_mode;
    out->presence_q1        = upmix_presence_encode(cfg->presence_db);
    out->strength_pct       = cfg->strength_pct;
    out->center_width_pct   = cfg->center_width_pct;
    out->corr_threshold_pct = cfg->corr_threshold_pct;
    out->attack_ms          = cfg->attack_ms;
    out->release_ms         = cfg->release_ms;
    out->detector_hpf_hz    = cfg->detector_hpf_hz;
    out->surround_delay_ms  = cfg->surround_delay_ms;
    out->surround_hpf_hz    = cfg->surround_hpf_hz;
    out->surround_lpf_hz    = cfg->surround_lpf_hz;
    out->decorr_pct         = cfg->decorr_pct;
    out->src_a              = cfg->src_a;
    out->src_b              = cfg->src_b;
    out->active_counts      = cfg->active_counts;
    out->back_offset_ms     = cfg->back_offset_ms;
    for (int g = 0; g < UPMIX_NUM_GEN; g++) {
        out->routes[g].out_mask = cfg->routes[g].out_mask;
        out->routes[g].trim_q1  = cfg->routes[g].trim_q1;
        out->routes[g].flags    = cfg->routes[g].flags;
    }
}

static int8_t um_clamp_trim(int t) {
    return (int8_t)(t < UPMIX_TRIM_MIN_Q1 ? UPMIX_TRIM_MIN_Q1 : (t > UPMIX_TRIM_MAX_Q1 ? UPMIX_TRIM_MAX_Q1 : t));
}

// A non-finite float from bulk or flash would reach the coefficient maths.
static inline float um_fin(float v, float def) { return isfinite(v) ? v : def; }

void upmix_packet_to_config(uint8_t p, const UpmixConfigPacket *in, uint8_t parts) {
    if (p >= UPMIX_NUM_PROFILES) return;
    volatile UpmixConfig *cfg = &upmix_profiles[p];
    if (parts & UPMIX_PKT_HEAD) {
        cfg->enabled            = in->enabled != 0;
        cfg->center_mode        = upmix_clamp_center_mode(in->center_mode);
        cfg->surround_mode      = in->surround_mode > UPMIX_SURROUND_ADAPTIVE ? UPMIX_SURROUND_ADAPTIVE
                                                                              : in->surround_mode;
        cfg->presence_db        = upmix_presence_decode(in->presence_q1);
        cfg->strength_pct       = um_fin(in->strength_pct, UPMIX_DEFAULT_STRENGTH);
        cfg->center_width_pct   = um_fin(in->center_width_pct, UPMIX_DEFAULT_WIDTH);
        cfg->corr_threshold_pct = um_fin(in->corr_threshold_pct, UPMIX_DEFAULT_THRESH);
        cfg->attack_ms          = um_fin(in->attack_ms, UPMIX_DEFAULT_ATTACK);
        cfg->release_ms         = um_fin(in->release_ms, UPMIX_DEFAULT_RELEASE);
        cfg->detector_hpf_hz    = um_fin(in->detector_hpf_hz, UPMIX_DEFAULT_DET_HPF);
        cfg->surround_delay_ms  = um_fin(in->surround_delay_ms, UPMIX_DEFAULT_SUR_DELAY);
        cfg->surround_hpf_hz    = um_fin(in->surround_hpf_hz, UPMIX_DEFAULT_SUR_HPF);
        cfg->surround_lpf_hz    = um_fin(in->surround_lpf_hz, UPMIX_DEFAULT_SUR_LPF);
        cfg->decorr_pct         = um_fin(in->decorr_pct, UPMIX_DEFAULT_DECORR);
    }
    if (parts & UPMIX_PKT_TAIL) {
        cfg->src_a          = in->src_a < NUM_INPUT_CHANNELS ? in->src_a : UPMIX_DEFAULT_SRC_A;
        cfg->src_b          = in->src_b < NUM_INPUT_CHANNELS ? in->src_b : UPMIX_DEFAULT_SRC_B;
        cfg->active_counts  = in->active_counts & UPMIX_COUNTS_MASK;
        cfg->back_offset_ms = um_fin(in->back_offset_ms, UPMIX_DEFAULT_BACK_OFFSET);
        for (int g = 0; g < UPMIX_NUM_GEN; g++) {
            cfg->routes[g].out_mask = in->routes[g].out_mask & UPMIX_OUT_MASK;
            cfg->routes[g].trim_q1  = um_clamp_trim(in->routes[g].trim_q1);
            cfg->routes[g].flags    = in->routes[g].flags & UPMIX_ROUTE_INVERT;
        }
    }
}

void upmix_notify_config(uint8_t p) {
    if (p >= UPMIX_NUM_PROFILES) return;
    UpmixConfigPacket k;
    upmix_config_to_packet(p, &k);
    // Profile 0's first 44 bytes sit at the V25 upmix section, the rest of both
    // profiles in upmix2 (bulk_params.h); packet offsets map onto both.
    const size_t base = p ? offsetof(WireBulkParams, upmix2.p1) : offsetof(WireBulkParams, upmix);
    const size_t ext = (p ? offsetof(WireBulkParams, upmix2.p1_ext) : offsetof(WireBulkParams, upmix2.p0_ext))
                       - UPMIX_PACKET_V1_SIZE;
    #define UM_NOTIFY(at, f) notify_param_write((uint16_t)((at) + offsetof(UpmixConfigPacket, f)), \
                                                sizeof(k.f), (const void *)&k.f)
    UM_NOTIFY(base, enabled);
    UM_NOTIFY(base, center_mode);
    UM_NOTIFY(base, surround_mode);
    UM_NOTIFY(base, presence_q1);
    UM_NOTIFY(base, strength_pct);
    UM_NOTIFY(base, center_width_pct);
    UM_NOTIFY(base, corr_threshold_pct);
    UM_NOTIFY(base, attack_ms);
    UM_NOTIFY(base, release_ms);
    UM_NOTIFY(base, detector_hpf_hz);
    UM_NOTIFY(base, surround_delay_ms);
    UM_NOTIFY(base, surround_hpf_hz);
    UM_NOTIFY(base, surround_lpf_hz);
    UM_NOTIFY(base, decorr_pct);
    UM_NOTIFY(ext, src_a);
    UM_NOTIFY(ext, src_b);
    UM_NOTIFY(ext, active_counts);
    UM_NOTIFY(ext, back_offset_ms);
    for (int g = 0; g < UPMIX_NUM_GEN; g++)
        UM_NOTIFY(ext, routes[g]);
    #undef UM_NOTIFY
}

void upmix_routes_from_v1(UpmixRoute routes[UPMIX_NUM_GEN],
                          const UpmixV1Crosspoint xp[5][NUM_OUTPUT_CHANNELS], uint16_t out_enabled) {
    float best[UPMIX_NUM_GEN];
    memset(routes, 0, sizeof(UpmixRoute) * UPMIX_NUM_GEN);
    for (int g = 0; g < UPMIX_NUM_GEN; g++) best[g] = -INFINITY;
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
        if (!(out_enabled & (1u << o))) continue;
        int row = -1, rows = 0;
        for (int r = 0; r < 5; r++)
            if (xp[r][o].enabled && xp[r][o].gain_db > -120.0f) { row = r; rows++; }   // -120 dB is the matrix mute
        if (rows != 1) continue;   // several rows: the output keeps its matrix feed
        // Rows 0..4 were L', R', C, Ls, Rs: generated channels A', B', C, SA, SB.
        routes[row].out_mask |= (uint16_t)(1u << o);
        if (xp[row][o].gain_db > best[row]) {   // one trim per channel: the largest gain wins
            best[row] = xp[row][o].gain_db;
            routes[row].trim_q1 = um_clamp_trim((int)lrintf(xp[row][o].gain_db * 2.0f));
            routes[row].flags = xp[row][o].invert ? UPMIX_ROUTE_INVERT : 0;
        }
    }
}

// x^e for a compile-time constant e; folds to a multiply-free form for the
// tuned values so the control update stays off powf.
static inline float upmix_powc(float x, float e) {
    if (e == 1.0f) return x;
    if (e == 0.5f) return sqrtf(x);
    if (e == 0.75f) { float s = sqrtf(x); return s * sqrtf(s); }
    if (e == 0.0f) return 1.0f;
    return powf(x, e);
}

// One TPT SVF Butterworth stage pair: HP then LP (surround band-limit).
static inline float upmix_band(const UpmixCoeffs *c, float x, int ch) {
    float v3 = x - um.s.shp_ic2[ch];
    float v1 = c->shp_a1 * um.s.shp_ic1[ch] + c->shp_a2 * v3;
    float v2 = um.s.shp_ic2[ch] + c->shp_a2 * um.s.shp_ic1[ch] + c->shp_a3 * v3;
    um.s.shp_ic1[ch] = 2.0f * v1 - um.s.shp_ic1[ch];
    um.s.shp_ic2[ch] = 2.0f * v2 - um.s.shp_ic2[ch];
    x = x - c->shp_k * v1 - v2;
    v3 = x - um.s.slp_ic2[ch];
    v1 = c->slp_a1 * um.s.slp_ic1[ch] + c->slp_a2 * v3;
    v2 = um.s.slp_ic2[ch] + c->slp_a2 * um.s.slp_ic1[ch] + c->slp_a3 * v3;
    um.s.slp_ic1[ch] = 2.0f * v1 - um.s.slp_ic1[ch];
    um.s.slp_ic2[ch] = 2.0f * v2 - um.s.slp_ic2[ch];
    return v2;
}

// Once per packet, so size beats speed: -O3 made this 2 KB of RAM code
// against a .data budget with 280 B left (see check_ram_placement.py).
DSP_TIME_CRITICAL
__attribute__((optimize("Os")))
void upmix_begin(const UpmixCoeffs * __restrict c, uint32_t n) {
    if (n == 0) return;
    // A profile change restarts from reset, so its fades run from zero.
    if (um.running && um.profile != c->profile) upmix_reset_state();
    um.profile = c->profile;
    upmix_state_dirty = true;
    um.running = true;

    const bool sur = c->surround_mode != UPMIX_SURROUND_OFF;
    const bool sur_adaptive = sur && c->surround_mode == UPMIX_SURROUND_ADAPTIVE;
    const float inv_n = 1.0f / (float)n;

    // ---- Control update (from the estimators as of the previous block) ----
    float env_new = um.env + (float)n * c->env_step;
    if (env_new > 1.0f) env_new = 1.0f;

    // Centre gain target from smoothed correlation + balance
    float g_target;
    if (c->center_mode == UPMIX_CENTER_ADAPTIVE) {
        float denom = sqrtf(um.acc_ll * um.acc_rr) + 1e-12f;
        float corr = um.acc_lr / denom;
        corr = clampf(corr, -1.0f, 1.0f);
        float bal = fabsf(um.acc_ll - um.acc_rr) / (um.acc_ll + um.acc_rr + 1e-12f);
        um.t_corr = corr;
        um.t_bal = bal;
        float p = (corr > 0.0f ? corr : 0.0f) * (1.0f - bal);
        // Gate below threshold, re-normalize above it (continuous at the knee)
        float m = (p - c->corr_thresh) * c->inv_thresh_range;
        g_target = c->strength * clampf(m, 0.0f, 1.0f);
    } else {
        um.t_corr = 0.0f;
        um.t_bal = 0.0f;
        g_target = c->strength;
    }
    // Attack/release ballistics, applied per block (packet-size independent)
    float alpha_blk = powf(g_target > um.g_c ? c->alpha_att : c->alpha_rel, (float)n);
    um.g_c = g_target + (um.g_c - g_target) * alpha_blk;
    // Centre OFF skips the release ballistic: the gain must reach exactly zero,
    // not decay toward it forever, or L/R would never be bit-exact again.  The
    // per-sample ramp below still glides out over this one block.
    if (c->center_mode == UPMIX_CENTER_OFF) um.g_c = 0.0f;

    // Surround (re)start, including a PASSIVE <-> ADAPTIVE switch (the ring
    // holds different signals per mode): clear all surround state so nothing
    // stale replays, and ramp the gains up from zero.
    if (sur && (!um.sur_active || um.sur_mode != c->surround_mode)) {
        memset(&um.s, 0, sizeof(um.s));
    }
    // Back pair start: its allpass and fade restart; the shared Haas ring is already valid.
    const bool back = sur && c->back;
    if (back && !um.s.back_on) {
        memset(um.s.bap, 0, sizeof(um.s.bap));
        um.s.benv = um.s.p_benv = 0.0f;
    }
    um.s.back_on = back;
    um.sur_active = sur;
    um.sur_mode = c->surround_mode;

    // Adaptive surround targets (see upmixer_spec.md for the derivation)
    float gls_t = 1.0f, grs_t = 1.0f, wl_t = 0.0f, wr_t = 0.0f;
    if (sur_adaptive) {
        float fbb = 0.5f * um.s.dom_fb - 0.5f;          // front/back bias: [-1, 0]
        float sfb = 1.0f - (fbb + 1.0f) * (fbb + 1.0f); // 1 = back dominant, 0 = front
        float g = UPMIX_FB_NEUTRAL * sfb;
        // Gate and dominance read the raw (pre-removal) band moments: centre
        // removal makes L'/R' symmetric and would hide the imbalance.
        float ll = um.s.w_ll, rr = um.s.w_rr, lr = um.s.w_lr;
        float d = (ll - rr) / (ll + rr + 1e-12f);       // signed balance, + = left
        float dl = d > 0.0f ? d : 0.0f, dr = d < 0.0f ? -d : 0.0f;
        // Side gate closes the dominant side fully, the quiet side by (1-bal)^K.
        // K = 1 with FLOOR_Q = 0 reproduces the symmetric gate and fixed floor.
        gls_t = g * ((1.0f - dl) * upmix_powc(1.0f - dr, UPMIX_GATE_K));
        grs_t = g * ((1.0f - dr) * upmix_powc(1.0f - dl, UPMIX_GATE_K));
        // Floor relaxes only on the weight feeding the dominant side across
        float fl = UPMIX_W_FLOOR * upmix_powc(1.0f - dr, UPMIX_FLOOR_Q);
        float fr = UPMIX_W_FLOOR * upmix_powc(1.0f - dl, UPMIX_FLOOR_Q);
        // Weights predict L'/R' (L' = a*L + b*R, R' = b*L + a*R), so map the
        // raw moments through the removal as it stood at the end of last block.
        float a = 1.0f - um.p_rem, b = -um.p_rem;
        float aa = a * a, bb = b * b, ab2 = 2.0f * a * b;
        float ll1 = aa * ll + ab2 * lr + bb * rr;
        float rr1 = bb * ll + ab2 * lr + aa * rr;
        // Rounding can leave these variances just below zero. A zero divisor
        // under a zero lr1 would put a NaN into the weight ramp for good.
        if (ll1 < 0.0f) ll1 = 0.0f;
        if (rr1 < 0.0f) rr1 = 0.0f;
        float lr1 = a * b * (ll + rr) + (aa + bb) * lr;
        float reg = UPMIX_W_EPS * (ll1 + rr1) + 1e-12f;
        wl_t = clampf(lr1 / (rr1 + reg), fl, UPMIX_W_CEIL);
        wr_t = clampf(lr1 / (ll1 + reg), fr, UPMIX_W_CEIL);
        // Transient duck: attenuation jumps to the onset excess, then releases
        float rel = powf(c->alpha_drel, (float)n);
        for (int ch = 0; ch < 2; ch++) {
            float ratio = um.s.ef[ch] / (um.s.es[ch] + 1e-12f);
            float tgt = ratio <= UPMIX_DUCK_THR ? 1.0f
                      : upmix_powc(UPMIX_DUCK_THR / ratio, UPMIX_DUCK_DEPTH);
            float relv = um.s.duck_att[ch] * rel;
            float att = 1.0f - tgt;
            um.s.duck_att[ch] = att > relv ? att : relv;
        }
        gls_t *= 1.0f - um.s.duck_att[0];
        grs_t *= 1.0f - um.s.duck_att[1];
    }

    // Effective per-sample coefficients, ramped linearly across the block
    float gc_new  = 0.7071f * um.g_c * env_new;                  // C = gc * (L+R)
    float rem_new = 0.5f * (1.0f - c->width) * um.g_c * env_new; // L' = L - rem*(L+R)
    // Adaptive rears fade in from their own envelope, started only once the
    // block begins at or past the point where delayed audio first emerges.
    float senv = env_new, benv = 1.0f;
    if (sur_adaptive) {
        senv = um.s.senv;
        if (um.s.fill >= c->haas_delay) {
            senv += (float)n * c->env_step;
            if (senv > 1.0f) senv = 1.0f;
        }
        // The backs emerge later, from the deeper tap, so they ramp on their own.
        benv = um.s.benv;
        if (back && um.s.fill >= c->back_delay) {
            benv += (float)n * c->env_step;
            if (benv > 1.0f) benv = 1.0f;
        }
        if (um.s.fill < (back ? c->back_delay : c->haas_delay)) um.s.fill += n;
        um.s.senv = senv;
        um.s.benv = benv;
    }
    um.rp.benv = um.s.p_benv; um.rp.benv_d = (benv - um.s.p_benv) * inv_n;
    um.s.p_benv = benv;
    float gls_new = gls_t * senv;
    float grs_new = grs_t * senv;
    um.rp.gc  = um.p_gc;    um.rp.gc_d  = (gc_new  - um.p_gc)    * inv_n;
    um.rp.rem = um.p_rem;   um.rp.rem_d = (rem_new - um.p_rem)   * inv_n;
    um.rp.gls = um.s.p_gls; um.rp.gls_d = (gls_new - um.s.p_gls) * inv_n;
    um.rp.grs = um.s.p_grs; um.rp.grs_d = (grs_new - um.s.p_grs) * inv_n;
    um.rp.wl  = um.s.p_wl;  um.rp.wl_d  = (wl_t    - um.s.p_wl)  * inv_n;
    um.rp.wr  = um.s.p_wr;  um.rp.wr_d  = (wr_t    - um.s.p_wr)  * inv_n;
    um.p_gc = gc_new; um.p_rem = rem_new; um.s.p_gls = gls_new; um.s.p_grs = grs_new;
    um.s.p_wl = wl_t; um.s.p_wr = wr_t;
    um.env = env_new;
}

DSP_TIME_CRITICAL
void upmix_render(const UpmixCoeffs * __restrict c,
                  const float * __restrict l, const float * __restrict r,
                  uint32_t n, float *const out[UPMIX_NUM_GEN]) {
    const bool sur = c->surround_mode != UPMIX_SURROUND_OFF;
    const bool sur_adaptive = sur && c->surround_mode == UPMIX_SURROUND_ADAPTIVE;
    float * __restrict abuf = out[UPMIX_GEN_A];
    float * __restrict bbuf = out[UPMIX_GEN_B];
    float * __restrict cbuf = out[UPMIX_GEN_C];
    float * __restrict lsbuf = out[UPMIX_GEN_SA];
    float * __restrict rsbuf = out[UPMIX_GEN_SB];
    float * __restrict babuf = out[UPMIX_GEN_BA];
    float * __restrict bbbuf = out[UPMIX_GEN_BB];
    const bool back = sur && c->back;
    float benv_i = um.rp.benv;
    const float benv_step = um.rp.benv_d;
    const uint32_t bdly = c->back_delay, bapd = c->bap_delay;

    float gc_i = um.rp.gc, rem_i = um.rp.rem;
    float gls_i = um.rp.gls, grs_i = um.rp.grs, wl_i = um.rp.wl, wr_i = um.rp.wr;
    const float gc_step = um.rp.gc_d, rem_step = um.rp.rem_d;
    const float gls_step = um.rp.gls_d, grs_step = um.rp.grs_d;
    const float wl_step = um.rp.wl_d, wr_step = um.rp.wr_d;

    float det_lp_l = um.det_lp_l, det_lp_r = um.det_lp_r;
    float acc_lr = um.acc_lr, acc_ll = um.acc_ll, acc_rr = um.acc_rr;
    const float hp_a = c->det_hp_a;
    const float a_corr = c->alpha_corr, omc = 1.0f - c->alpha_corr;
    const float a_dom = c->alpha_dom, omd = 1.0f - c->alpha_dom;
    const float a_w = c->alpha_w, omw = 1.0f - c->alpha_w;
    const float daf = c->duck_af, das = c->duck_as;
    const float ls_cl = c->ls_cl, ls_cr = c->ls_cr;
    const float ap_g = c->ap_g;
    const uint32_t hdly = c->haas_delay, adly = c->ap_delay;
    uint32_t widx = um.widx;

    for (uint32_t i = 0; i < n; i++) {
        float l0 = l[i], r0 = r[i];

        // Detector: bass-cut HP, then one-pole product estimators
        det_lp_l += hp_a * (l0 - det_lp_l);
        det_lp_r += hp_a * (r0 - det_lp_r);
        float el = l0 - det_lp_l, er = r0 - det_lp_r;
        acc_lr = a_corr * acc_lr + omc * (el * er);
        acc_ll = a_corr * acc_ll + omc * (el * el);
        acc_rr = a_corr * acc_rr + omc * (er * er);

        // Centre extraction + constant-power removal from the mains
        float mid = l0 + r0;
        float c0 = gc_i * mid;
        // Presence bell on the extracted centre (both centre modes).  Runs
        // unconditionally so gain sweeps through 0 dB stay continuous; at
        // 0 dB pres_m1 = 0 and the output is bit-exact c0.
        {
            float pv3 = c0 - um.pres_ic2;
            float pv1 = c->pres_a1 * um.pres_ic1 + c->pres_a2 * pv3;
            float pv2 = um.pres_ic2 + c->pres_a2 * um.pres_ic1 + c->pres_a3 * pv3;
            um.pres_ic1 = 2.0f * pv1 - um.pres_ic1;
            um.pres_ic2 = 2.0f * pv2 - um.pres_ic2;
            c0 += c->pres_m1 * pv1;
        }
        cbuf[i] = c0;
        const float rem_c = rem_i;
        float rem = rem_i * mid;
        float l1 = l0 - rem, r1 = r0 - rem;
        abuf[i] = l1;
        bbuf[i] = r1;
        gc_i += gc_step;
        rem_i += rem_step;

        if (!sur) continue;
        float x0, x1, xb0 = 0.0f, xb1 = 0.0f;
        if (sur_adaptive) {
            // Band-limit the raw input before the Haas delay so steering leads
            // the audio by the delay; L'/R' follow by linearity of the filter.
            float b0 = upmix_band(c, l0, 0), b1 = upmix_band(c, r0, 1);
            float f = 0.5f * (b0 + b1), b = 0.5f * (b0 - b1);
            float dfb = clampf(UPMIX_DOM_CLIP_GAIN * (fabsf(f) - fabsf(b)), -1.0f, 1.0f);
            um.s.dom_fb = a_dom * um.s.dom_fb + omd * dfb;
            um.s.w_lr = a_w * um.s.w_lr + omw * (b0 * b1);
            um.s.w_ll = a_w * um.s.w_ll + omw * (b0 * b0);
            um.s.w_rr = a_w * um.s.w_rr + omw * (b1 * b1);
            // Duck envelopes follow the residual, so cancelled centre onsets
            // (kicks, vocals) do not trigger it.
            float rb = rem_c * (b0 + b1);
            float p0 = b0 - rb, p1 = b1 - rb;
            float q0 = p0 - wl_i * p1, q1 = p1 - wr_i * p0;
            q0 = q0 * q0; q1 = q1 * q1;
            um.s.ef[0] += daf * (q0 - um.s.ef[0]); um.s.es[0] += das * (q0 - um.s.es[0]);
            um.s.ef[1] += daf * (q1 - um.s.ef[1]); um.s.es[1] += das * (q1 - um.s.es[1]);

            um.s.haas[0][widx & (UPMIX_HAAS_RING - 1)] = p0;
            um.s.haas[1][widx & (UPMIX_HAAS_RING - 1)] = p1;
            float d0 = um.s.haas[0][(widx - hdly) & (UPMIX_HAAS_RING - 1)];
            float d1 = um.s.haas[1][(widx - hdly) & (UPMIX_HAAS_RING - 1)];
            x0 = gls_i * (d0 - wl_i * d1);
            x1 = grs_i * (d1 - wr_i * d0);
            if (back) {   // same weights and steering gains at the deeper tap, on its own fade
                float e0 = um.s.haas[0][(widx - bdly) & (UPMIX_HAAS_RING - 1)];
                float e1 = um.s.haas[1][(widx - bdly) & (UPMIX_HAAS_RING - 1)];
                xb0 = benv_i * (gls_i * (e0 - wl_i * e1));
                xb1 = benv_i * (grs_i * (e1 - wr_i * e0));
                benv_i += benv_step;
            }
            gls_i += gls_step; grs_i += grs_step;
            wl_i += wl_step; wr_i += wr_step;
        } else {
            // Passive: feed, then band-limit, then Haas delay
            x0 = gls_i * (ls_cl * l1 + ls_cr * r1);
            x1 = grs_i * (ls_cl * r1 + ls_cr * l1);
            gls_i += gls_step;
            grs_i += grs_step;
            x0 = upmix_band(c, x0, 0);
            x1 = upmix_band(c, x1, 1);
            um.s.haas[0][widx & (UPMIX_HAAS_RING - 1)] = x0;
            um.s.haas[1][widx & (UPMIX_HAAS_RING - 1)] = x1;
            x0 = um.s.haas[0][(widx - hdly) & (UPMIX_HAAS_RING - 1)];
            x1 = um.s.haas[1][(widx - hdly) & (UPMIX_HAAS_RING - 1)];
            if (back) {   // the ring holds gained feed, so the activation ramp is already in it
                xb0 = um.s.haas[0][(widx - bdly) & (UPMIX_HAAS_RING - 1)];
                xb1 = um.s.haas[1][(widx - bdly) & (UPMIX_HAAS_RING - 1)];
            }
        }

        // Schroeder allpass, mirrored gains (+g / -g) for max inter-channel
        // phase spread: w[n] = x + g*w[n-D], y = w[n-D] - g*w[n]
        float wd0 = um.s.ap[0][(widx - adly) & (UPMIX_AP_RING - 1)];
        float w0 = x0 + ap_g * wd0;
        um.s.ap[0][widx & (UPMIX_AP_RING - 1)] = w0;
        lsbuf[i] = wd0 - ap_g * w0;

        float wd1 = um.s.ap[1][(widx - adly) & (UPMIX_AP_RING - 1)];
        float w1 = x1 - ap_g * wd1;
        um.s.ap[1][widx & (UPMIX_AP_RING - 1)] = w1;
        rsbuf[i] = wd1 + ap_g * w1;

        if (back) {   // signs mirrored against the sides, so SA and BA differ in their first tap
            float bd0 = um.s.bap[0][(widx - bapd) & (UPMIX_BACK_AP_RING - 1)];
            float bw0 = xb0 - ap_g * bd0;
            um.s.bap[0][widx & (UPMIX_BACK_AP_RING - 1)] = bw0;
            babuf[i] = bd0 + ap_g * bw0;
            float bd1 = um.s.bap[1][(widx - bapd) & (UPMIX_BACK_AP_RING - 1)];
            float bw1 = xb1 + ap_g * bd1;
            um.s.bap[1][widx & (UPMIX_BACK_AP_RING - 1)] = bw1;
            bbbuf[i] = bd1 - ap_g * bw1;
        }

        widx++;
    }

    um.det_lp_l = det_lp_l; um.det_lp_r = det_lp_r;
    um.acc_lr = acc_lr; um.acc_ll = acc_ll; um.acc_rr = acc_rr;
    um.widx = widx;
    um.rp.gc = gc_i; um.rp.rem = rem_i;
    um.rp.gls = gls_i; um.rp.grs = grs_i; um.rp.wl = wl_i; um.rp.wr = wr_i;
    um.rp.benv = benv_i;
}

__attribute__((always_inline)) static inline void um_release(void) {
    if (upmix_state_dirty) upmix_reset_state();
    ur.held = false;
    ur.active = false;
    memset(ur.gen_peak, 0, sizeof(ur.gen_peak));
    memset(ur.clip_hold, 0, sizeof(ur.clip_hold));
}

DSP_TIME_CRITICAL
__attribute__((optimize("Os")))
const UpmixCoeffs *upmix_claim(uint8_t n_in, uint32_t rate, uint16_t out_en, UpmixClaim *cl) {
    cl->claimed = 0;
    const UpmixCoeffs *sel = rate <= (uint32_t)UPMIX_RATE_MAX ? upmix_select(n_in) : NULL;
    if (sel && (sel->src_a >= n_in || sel->src_b >= n_in)) sel = NULL;
    // Count and rate changes park hard (spec 6.4).  Source switches at the same
    // count and rate arrive through upmix_hard_park() from the pipeline reset.
    if (ur.held && (n_in != ur.n_inputs || rate != ur.rate)) {
        memset(ur.e, 0, sizeof(ur.e));
        memset(ur.w, 0, sizeof(ur.w));
        um_release();
    }
    ur.n_inputs = n_in;
    ur.rate = rate;
    if (ur.held) {
        // A source edit fades out and restarts like a profile change (spec 3.2).
        ur.active = sel && sel->profile == ur.latch.profile
                    && sel->src_a == ur.latch.src_a && sel->src_b == ur.latch.src_b;
        if (ur.active && sel->seq != ur.latch.seq) ur.latch = *sel;
    }
    // Second pass: a finished fade-out lets the waiting profile start this packet.
    for (int pass = 0; pass < 2; pass++) {
        if (!ur.held) {
            if (!sel || !(sel->claim_mask & out_en)) break;   // nothing routed: no engine
            ur.latch = *sel;
            ur.held = ur.active = true;
        }
        uint16_t claimed = 0;
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
            uint16_t bit = (uint16_t)(1u << o);
            bool live = ur.w[o] != 0.0f || (ur.active && (ur.latch.claim_mask & bit));
            for (int g = 0; g < UPMIX_NUM_GEN; g++) {
                if (!(out_en & bit)) ur.e[g][o] = 0.0f;   // disabled: fade in again on return
                live |= ur.e[g][o] != 0.0f;
            }
            if (!(out_en & bit)) { ur.w[o] = 0.0f; continue; }
            if (live) claimed |= bit;
        }
        if (claimed || (ur.active && (ur.latch.claim_mask & out_en))) {
            // The back pair renders while routed or while BA/BB still fade out.
            uint16_t bm = ur.latch.route_mask[UPMIX_GEN_BA] | ur.latch.route_mask[UPMIX_GEN_BB];
            bool back = ur.active && (bm & out_en);
            for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++)
                back |= ur.e[UPMIX_GEN_BA][o] != 0.0f || ur.e[UPMIX_GEN_BB][o] != 0.0f;
            ur.latch.back = back && ur.latch.surround_mode != UPMIX_SURROUND_OFF;
            cl->claimed = claimed;
            cl->src_a = ur.latch.src_a;
            cl->src_b = ur.latch.src_b;
            memset(cl->side_n, 0, sizeof(cl->side_n));
            return &ur.latch;
        }
        um_release();
    }
    if (ur.held) um_release();
    else upmix_park();
    return NULL;
}

// Fixed-rate slew that lands exactly on the target (spec 6.4).
static inline float um_slew(float v, float t, float step) {
    if (v < t) { v += step; return v > t ? t : v; }
    v -= step;
    return v < t ? t : v;
}

// Adds output o's generated contributions and restores its omitted rows by 1 - w.
// Separate VMUL/VADD: VFMA is slower on the M33 (current_architecture.md, FP contraction).
DSP_TIME_CRITICAL __attribute__((noinline, optimize("O2", "fp-contract=off")))
static void um_inject(const UpmixCoeffs * __restrict c, const UpmixClaim * __restrict cl,
                      int o, float * __restrict dst, uint32_t off, uint32_t k, uint8_t live) {
    const uint16_t bit = (uint16_t)(1u << o);
    const float step = c->slew_step;
    for (int g = 0; g < UPMIX_NUM_GEN; g++) {
        float e = ur.e[g][o];
        const float t = (ur.active && (c->route_mask[g] & bit)) ? c->route_gain[g] : 0.0f;
        const float * __restrict s = um_chunk[g];
        if (e == t) {
            if (e == 0.0f || !(live & (1u << g))) continue;
            for (uint32_t i = 0; i < k; i++) dst[i] += e * s[i];
            continue;
        }
        if (live & (1u << g)) {
            for (uint32_t i = 0; i < k; i++) { e = um_slew(e, t, step); dst[i] += e * s[i]; }
        } else {
            for (uint32_t i = 0; i < k; i++) e = um_slew(e, t, step);
        }
        ur.e[g][o] = e;
    }

    float w = ur.w[o];
    const float tw = (ur.active && (c->claim_mask & bit)) ? 1.0f : 0.0f;
    const uint32_t ns = cl->side_n[o];
    if (w == tw && w == 1.0f) return;   // fully claimed: rows stay out, gen is bit-exact
    // One loop form for settled and slewing weights, so the sum cannot depend
    // on where a chunk ends.
    const float *s0 = ns ? cl->side_src[o][0] + off : NULL;
    const float *s1 = ns > 1 ? cl->side_src[o][1] + off : NULL;
    const float g0 = ns ? cl->side_gain[o][0] : 0.0f, g1 = ns > 1 ? cl->side_gain[o][1] : 0.0f;
    for (uint32_t i = 0; i < k; i++) {
        w = um_slew(w, tw, step);
        float r = ns > 1 ? g0 * s0[i] + g1 * s1[i] : (ns ? g0 * s0[i] : 0.0f);
        dst[i] += (1.0f - w) * r;
    }
    ur.w[o] = w;
}

DSP_TIME_CRITICAL
__attribute__((optimize("O2")))
void upmix_run(const UpmixCoeffs * __restrict c, const UpmixClaim * __restrict cl,
               float *const in[], float (*out)[AUDIO_BUFFER_SAMPLES], uint32_t n) {
    if (n == 0) return;
    upmix_begin(c, n);
    const float *a = in[c->src_a], *b = in[c->src_b];
    uint8_t live = (1u << UPMIX_GEN_A) | (1u << UPMIX_GEN_B) | (1u << UPMIX_GEN_C);
    if (c->surround_mode != UPMIX_SURROUND_OFF) live |= (1u << UPMIX_GEN_SA) | (1u << UPMIX_GEN_SB);
    if (c->back) live |= (1u << UPMIX_GEN_BA) | (1u << UPMIX_GEN_BB);
    float *const gen[UPMIX_NUM_GEN] = { um_chunk[0], um_chunk[1], um_chunk[2], um_chunk[3],
                                        um_chunk[4], um_chunk[5], um_chunk[6] };
    float pk[UPMIX_NUM_GEN] = { 0 };
    for (uint32_t off = 0; off < n; off += UPMIX_CHUNK) {
        uint32_t k = n - off < UPMIX_CHUNK ? n - off : UPMIX_CHUNK;
        upmix_render(c, a + off, b + off, k, gen);
        UPMIX_HOST_TAP(gen, off, k);
        for (int g = 0; g < UPMIX_NUM_GEN; g++) {
            if (!(live & (1u << g))) continue;
            float m = pk[g];
            for (uint32_t i = 0; i < k; i++) { float v = fabsf(um_chunk[g][i]); m = v > m ? v : m; }
            pk[g] = m;
        }
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++)
            if (cl->claimed & (1u << o)) um_inject(c, cl, o, out[o] + off, off, k, live);
    }
    // A clip shows for one second, so every poller sees it and no read clears it.
    for (int g = 0; g < UPMIX_NUM_GEN; g++) {
        ur.gen_peak[g] = (uint16_t)(fminf(1.0f, pk[g]) * 32767.0f);
        uint32_t h = ur.clip_hold[g];
        ur.clip_hold[g] = pk[g] > CLIP_THRESH_F ? ur.rate : (h > n ? h - n : 0);
    }
}

void upmix_hard_park(void) {
    if (!ur.held) return;   // parked already: every fade is at zero
    memset(ur.e, 0, sizeof(ur.e));
    memset(ur.w, 0, sizeof(ur.w));
    um_release();
}

#endif // PICO_RP2350
