/*
 * Stereo upmixer (RP2350 only)
 *
 * Derives Centre / Ls / Rs virtual matrix sources from the stereo bus.
 * See upmix.h for the signal flow, engine modes, and module pattern.
 *
 * Coefficient design notes:
 *   - All smoothers are one-pole with retention alpha = exp(-1/(fs*tau)).
 *     Centre-gain ballistics apply per block via powf(alpha, n) (leveller
 *     pattern), so behavior is independent of packet size.
 *   - Correlation is estimated on a bass-cut detector path (one-pole HP);
 *     long-wavelength content correlates by coincidence and would pump the
 *     steering (industry-standard "bass steering" mitigation).
 *   - Adaptive surround steering follows Dolby WO2007067320A2: rectified
 *     level difference per axis, gain 1024 + hard clip, 40 ms one-pole
 *     smoother, polynomial pan law (1 - x^2), PLII surround decode feed
 *     coefficients.  The patent's front/back bias maps FB into [-1, 0] so
 *     neutral material keeps ~0.75 surround pan-law gain.
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

// Live configuration; vendor handlers write it and raise the pending flag,
// the main loop recomputes + publishes.  Defaults match apply_factory_defaults.
volatile UpmixConfig upmix_config = {
    .enabled = false,
    .center_mode = UPMIX_DEFAULT_CENTER_MODE,
    .surround_mode = UPMIX_DEFAULT_SURROUND_MODE,
    .strength_pct = UPMIX_DEFAULT_STRENGTH,
    .center_width_pct = UPMIX_DEFAULT_WIDTH,
    .corr_threshold_pct = UPMIX_DEFAULT_THRESH,
    .attack_ms = UPMIX_DEFAULT_ATTACK,
    .release_ms = UPMIX_DEFAULT_RELEASE,
    .detector_hpf_hz = UPMIX_DEFAULT_DET_HPF,
    .surround_delay_ms = UPMIX_DEFAULT_SUR_DELAY,
    .surround_hpf_hz = UPMIX_DEFAULT_SUR_HPF,
    .surround_lpf_hz = UPMIX_DEFAULT_SUR_LPF,
    .decorr_pct = UPMIX_DEFAULT_DECORR,
    .presence_db = UPMIX_DEFAULT_PRESENCE,
};
volatile bool upmix_update_pending = false;

// Published coefficient set the pipeline snapshots each packet; NULL = disabled.
volatile const UpmixCoeffs *current_upmix_coeffs = NULL;

// Double buffer so upmix_apply_config() never writes through the published pointer.
static UpmixCoeffs um_coeff_bufs[2];
static uint8_t um_coeff_idx = 0;

// Processing state (Core 0 only; the pass runs before the matrix).
typedef struct {
    // Detector / correlation estimator
    float det_lp_l, det_lp_r;      // one-pole HP states
    float acc_lr, acc_ll, acc_rr;  // smoothed products
    float g_c;                     // smoothed centre gain, 0..strength
    // Dominance smoothers (adaptive surround)
    float dom_lr, dom_fb;
    // Previous effective per-sample gains (ramp start points)
    float p_gc, p_rem, p_gls, p_grs;
    // Activation fade
    float env;
    // Telemetry snapshots
    float t_corr, t_bal;
    // Centre presence bell
    float pres_ic1, pres_ic2;      // TPT SVF integrators
    // Surround conditioning
    float shp_ic1[2], shp_ic2[2];  // HP SVF integrators
    float slp_ic1[2], slp_ic2[2];  // LP SVF integrators
    float haas[2][UPMIX_HAAS_RING];
    float ap[2][UPMIX_AP_RING];
    uint32_t widx;                 // shared ring write index
    bool sur_active;               // surround engine ran last packet
    volatile bool running;
} UpmixState;

static UpmixState um;

// Park fast-path flag (see upmix_park() in upmix.h)
bool upmix_state_dirty = false;

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void upmix_compute_coefficients(UpmixCoeffs *c, const UpmixConfig *cfg, float sample_rate) {
    // Rates above 48 kHz are unsupported (rings sized for 48 kHz); park.
    if (!cfg->enabled || sample_rate < 1.0f || sample_rate > UPMIX_RATE_MAX) {
        memset(c, 0, sizeof(UpmixCoeffs));
        return;
    }

    const float pi = 3.1415926535f;
    float fs = sample_rate;

    c->center_mode = upmix_clamp_center_mode(cfg->center_mode);
    c->surround_mode = cfg->surround_mode > UPMIX_SURROUND_ADAPTIVE
                       ? UPMIX_SURROUND_ADAPTIVE : cfg->surround_mode;
    // n_derived tracks the surround mode alone: row 2 stays reserved with the
    // centre engine OFF so switching it does not renumber the Ls/Rs source rows
    // under existing matrix routing.
    c->n_derived = (c->surround_mode == UPMIX_SURROUND_OFF) ? 1 : UPMIX_NUM_DERIVED;

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

    if (c->surround_mode == UPMIX_SURROUND_ADAPTIVE) {
        c->ls_cl = 0.8710f;   // PLII surround decode (WO2007067320A2)
        c->ls_cr = -0.4898f;
    } else {
        c->ls_cl = 0.7071f;   // passive difference feed (Rs mirrors to -S)
        c->ls_cr = -0.7071f;
    }

    c->ap_g = 0.5f * clampf(cfg->decorr_pct, UPMIX_DECORR_MIN, UPMIX_DECORR_MAX) * 0.01f;

    float dly = clampf(cfg->surround_delay_ms, UPMIX_SUR_DELAY_MIN, UPMIX_SUR_DELAY_MAX);
    uint32_t hd = (uint32_t)(dly * 0.001f * fs + 0.5f);
    c->haas_delay = hd > UPMIX_HAAS_RING - 1 ? UPMIX_HAAS_RING - 1 : hd;
    uint32_t ad = (uint32_t)(UPMIX_DECORR_DELAY_MS * 0.001f * fs + 0.5f);
    c->ap_delay = ad > UPMIX_AP_RING - 1 ? UPMIX_AP_RING - 1 : ad;

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
}

// Rate gate snapshot for status reporting (written by upmix_apply_config).
static bool um_rate_ok = true;

void upmix_apply_config(const UpmixConfig *config, float sample_rate) {
    // Compute into the inactive buffer, then publish the pointer (psybass
    // pattern).  The pipeline snapshots current_upmix_coeffs once per packet.
    // NULL is published when disabled OR the rate is above 48 kHz (ADAT-style
    // park; the pipeline then resets state via upmix_park).
    um_rate_ok = sample_rate <= UPMIX_RATE_MAX;
    UpmixCoeffs *next = &um_coeff_bufs[um_coeff_idx ^ 1];
    upmix_compute_coefficients(next, config, sample_rate);
    if (config->enabled && um_rate_ok) {
        um_coeff_idx ^= 1;
        current_upmix_coeffs = next;
    } else {
        current_upmix_coeffs = NULL;
    }
}

// RAM-resident: runs from the audio thread on the running -> parked
// transition; must not fetch from flash there.
DSP_TIME_CRITICAL
void upmix_reset_state(void) {
    memset(&um, 0, sizeof(um));
    upmix_state_dirty = false;
}

void upmix_get_status(UpmixStatus *st) {
    memset(st, 0, sizeof(*st));
    bool enabled = upmix_config.enabled;
    st->active = um.running ? 1 : 0;
    st->parked_reason = um.running ? 0
                        : (!enabled ? 1 : (!um_rate_ok ? 3 : 2));
    st->corr_q14 = (int16_t)(clampf(um.t_corr, -1.0f, 1.0f) * 16384.0f);
    st->balance_q14 = (int16_t)(clampf(um.t_bal, 0.0f, 1.0f) * 16384.0f);
    st->center_gain_q15 = (uint16_t)(clampf(um.p_gc * 1.4142135624f, 0.0f, 1.0f) * 32767.0f);
    st->ls_gain_q15 = (uint16_t)(clampf(um.p_gls, 0.0f, 1.0f) * 32767.0f);
    st->rs_gain_q15 = (uint16_t)(clampf(um.p_grs, 0.0f, 1.0f) * 32767.0f);
}

void upmix_config_to_packet(UpmixConfigPacket *out) {
    out->enabled            = upmix_config.enabled ? 1 : 0;
    out->center_mode        = upmix_config.center_mode;
    out->surround_mode      = upmix_config.surround_mode;
    out->presence_q1        = upmix_presence_encode(upmix_config.presence_db);
    out->strength_pct       = upmix_config.strength_pct;
    out->center_width_pct   = upmix_config.center_width_pct;
    out->corr_threshold_pct = upmix_config.corr_threshold_pct;
    out->attack_ms          = upmix_config.attack_ms;
    out->release_ms         = upmix_config.release_ms;
    out->detector_hpf_hz    = upmix_config.detector_hpf_hz;
    out->surround_delay_ms  = upmix_config.surround_delay_ms;
    out->surround_hpf_hz    = upmix_config.surround_hpf_hz;
    out->surround_lpf_hz    = upmix_config.surround_lpf_hz;
    out->decorr_pct         = upmix_config.decorr_pct;
}

void upmix_notify_config(void) {
    UpmixConfigPacket p;
    upmix_config_to_packet(&p);
    // Offsets come from WireBulkParams by name so each event lands on its own
    // field; one PARAM_CHANGED per field the SET actually moved.
    #define UM_NOTIFY(f) notify_param_write(offsetof(WireBulkParams, upmix.f), \
                                            sizeof(p.f), (const void *)&p.f)
    UM_NOTIFY(enabled);
    UM_NOTIFY(center_mode);
    UM_NOTIFY(surround_mode);
    UM_NOTIFY(presence_q1);
    UM_NOTIFY(strength_pct);
    UM_NOTIFY(center_width_pct);
    UM_NOTIFY(corr_threshold_pct);
    UM_NOTIFY(attack_ms);
    UM_NOTIFY(release_ms);
    UM_NOTIFY(detector_hpf_hz);
    UM_NOTIFY(surround_delay_ms);
    UM_NOTIFY(surround_hpf_hz);
    UM_NOTIFY(surround_lpf_hz);
    UM_NOTIFY(decorr_pct);
    #undef UM_NOTIFY
}

DSP_TIME_CRITICAL
void upmix_process_block(const UpmixCoeffs * __restrict c,
                         float * __restrict l, float * __restrict r,
                         float * __restrict cbuf,
                         float * __restrict lsbuf, float * __restrict rsbuf,
                         uint32_t n) {
    if (n == 0) return;
    upmix_state_dirty = true;
    um.running = true;

    const bool sur = c->n_derived == UPMIX_NUM_DERIVED;
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

    // Surround steering gains from the smoothed dominance (patent pan law)
    float gls_t = 1.0f, grs_t = 1.0f;
    if (sur_adaptive) {
        float fbb = 0.5f * um.dom_fb - 0.5f;            // front/back bias: [-1, 0]
        float sfb = 1.0f - (fbb + 1.0f) * (fbb + 1.0f); // 1 = back dominant, 0 = front
        float ul = 0.5f * (um.dom_lr + 1.0f);           // 1 = right dominant
        float ur = 0.5f * (1.0f - um.dom_lr);
        gls_t = (1.0f - ul * ul) * sfb;
        grs_t = (1.0f - ur * ur) * sfb;
    }

    // Surround OFF -> ON while running (no park in between): clear the
    // conditioning state so stale ring/filter content cannot replay, and
    // ramp the steering gains up from zero.
    if (sur && !um.sur_active) {
        memset(um.shp_ic1, 0, sizeof(um.shp_ic1));
        memset(um.shp_ic2, 0, sizeof(um.shp_ic2));
        memset(um.slp_ic1, 0, sizeof(um.slp_ic1));
        memset(um.slp_ic2, 0, sizeof(um.slp_ic2));
        memset(um.haas, 0, sizeof(um.haas));
        memset(um.ap, 0, sizeof(um.ap));
        um.p_gls = 0.0f;
        um.p_grs = 0.0f;
    }
    um.sur_active = sur;

    // Effective per-sample coefficients, ramped linearly across the block
    float gc_new  = 0.7071f * um.g_c * env_new;                  // C = gc * (L+R)
    float rem_new = 0.5f * (1.0f - c->width) * um.g_c * env_new; // L' = L - rem*(L+R)
    float gls_new = gls_t * env_new;
    float grs_new = grs_t * env_new;
    float gc_i  = um.p_gc,  gc_step  = (gc_new  - um.p_gc)  * inv_n;
    float rem_i = um.p_rem, rem_step = (rem_new - um.p_rem) * inv_n;
    float gls_i = um.p_gls, gls_step = (gls_new - um.p_gls) * inv_n;
    float grs_i = um.p_grs, grs_step = (grs_new - um.p_grs) * inv_n;
    um.p_gc = gc_new; um.p_rem = rem_new; um.p_gls = gls_new; um.p_grs = grs_new;
    um.env = env_new;

    // ---- Audio pass ----
    float det_lp_l = um.det_lp_l, det_lp_r = um.det_lp_r;
    float acc_lr = um.acc_lr, acc_ll = um.acc_ll, acc_rr = um.acc_rr;
    float dom_lr = um.dom_lr, dom_fb = um.dom_fb;
    const float hp_a = c->det_hp_a;
    const float a_corr = c->alpha_corr, omc = 1.0f - c->alpha_corr;
    const float a_dom = c->alpha_dom, omd = 1.0f - c->alpha_dom;
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

        if (sur_adaptive) {
            // Axis dominance: rectified level difference, heavy gain + clip,
            // one-pole smoother (patent stage 2)
            float dlr = UPMIX_DOM_CLIP_GAIN * (fabsf(r0) - fabsf(l0));
            dlr = clampf(dlr, -1.0f, 1.0f);
            float f = 0.5f * (l0 + r0), b = 0.5f * (l0 - r0);
            float dfb = UPMIX_DOM_CLIP_GAIN * (fabsf(f) - fabsf(b));
            dfb = clampf(dfb, -1.0f, 1.0f);
            dom_lr = a_dom * dom_lr + omd * dlr;
            dom_fb = a_dom * dom_fb + omd * dfb;
        }

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
        float rem = rem_i * mid;
        float l1 = l0 - rem, r1 = r0 - rem;
        l[i] = l1;
        r[i] = r1;
        gc_i += gc_step;
        rem_i += rem_step;

        if (sur) {
            // Steered feeds, then per-channel conditioning:
            // band-limit -> Haas delay -> Schroeder allpass decorrelator
            float x0 = gls_i * (ls_cl * l1 + ls_cr * r1);
            float x1 = grs_i * (ls_cl * r1 + ls_cr * l1);
            gls_i += gls_step;
            grs_i += grs_step;

            float v3, v1, v2;
            // Ls: HP (output = x - k*v1 - v2)
            v3 = x0 - um.shp_ic2[0];
            v1 = c->shp_a1 * um.shp_ic1[0] + c->shp_a2 * v3;
            v2 = um.shp_ic2[0] + c->shp_a2 * um.shp_ic1[0] + c->shp_a3 * v3;
            um.shp_ic1[0] = 2.0f * v1 - um.shp_ic1[0];
            um.shp_ic2[0] = 2.0f * v2 - um.shp_ic2[0];
            x0 = x0 - c->shp_k * v1 - v2;
            // Ls: LP (output = v2)
            v3 = x0 - um.slp_ic2[0];
            v1 = c->slp_a1 * um.slp_ic1[0] + c->slp_a2 * v3;
            v2 = um.slp_ic2[0] + c->slp_a2 * um.slp_ic1[0] + c->slp_a3 * v3;
            um.slp_ic1[0] = 2.0f * v1 - um.slp_ic1[0];
            um.slp_ic2[0] = 2.0f * v2 - um.slp_ic2[0];
            x0 = v2;
            // Rs: HP
            v3 = x1 - um.shp_ic2[1];
            v1 = c->shp_a1 * um.shp_ic1[1] + c->shp_a2 * v3;
            v2 = um.shp_ic2[1] + c->shp_a2 * um.shp_ic1[1] + c->shp_a3 * v3;
            um.shp_ic1[1] = 2.0f * v1 - um.shp_ic1[1];
            um.shp_ic2[1] = 2.0f * v2 - um.shp_ic2[1];
            x1 = x1 - c->shp_k * v1 - v2;
            // Rs: LP
            v3 = x1 - um.slp_ic2[1];
            v1 = c->slp_a1 * um.slp_ic1[1] + c->slp_a2 * v3;
            v2 = um.slp_ic2[1] + c->slp_a2 * um.slp_ic1[1] + c->slp_a3 * v3;
            um.slp_ic1[1] = 2.0f * v1 - um.slp_ic1[1];
            um.slp_ic2[1] = 2.0f * v2 - um.slp_ic2[1];
            x1 = v2;

            // Haas delay (shared write index, per-channel rings)
            um.haas[0][widx & (UPMIX_HAAS_RING - 1)] = x0;
            um.haas[1][widx & (UPMIX_HAAS_RING - 1)] = x1;
            x0 = um.haas[0][(widx - hdly) & (UPMIX_HAAS_RING - 1)];
            x1 = um.haas[1][(widx - hdly) & (UPMIX_HAAS_RING - 1)];

            // Schroeder allpass, mirrored gains (+g / -g) for max inter-channel
            // phase spread: w[n] = x + g*w[n-D], y = w[n-D] - g*w[n]
            float wd0 = um.ap[0][(widx - adly) & (UPMIX_AP_RING - 1)];
            float w0 = x0 + ap_g * wd0;
            um.ap[0][widx & (UPMIX_AP_RING - 1)] = w0;
            lsbuf[i] = wd0 - ap_g * w0;

            float wd1 = um.ap[1][(widx - adly) & (UPMIX_AP_RING - 1)];
            float w1 = x1 - ap_g * wd1;
            um.ap[1][widx & (UPMIX_AP_RING - 1)] = w1;
            rsbuf[i] = wd1 + ap_g * w1;

            widx++;
        }
    }

    um.det_lp_l = det_lp_l; um.det_lp_r = det_lp_r;
    um.acc_lr = acc_lr; um.acc_ll = acc_ll; um.acc_rr = acc_rr;
    um.dom_lr = dom_lr; um.dom_fb = dom_fb;
    um.widx = widx;
}

#endif // PICO_RP2350
