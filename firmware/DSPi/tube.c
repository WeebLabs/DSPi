/*
 * Tube Preamp Emulation
 *
 * Biased asymmetric polynomial waveshaper with supply sag, a DC blocker and
 * an optional output-stage model (damping-factor bell at the speaker
 * resonance plus a top shelf), run per output channel.  Tube types are rows of the four character
 * parameters, applied at SET time only, so every type costs the same.
 * Signal flow, parameter ranges and Q28 ceilings:
 * Documentation/Features/tube_preamp_spec.md.
 */

#include <math.h>
#include <string.h>
#include "tube.h"
#include "bulk_params.h"   // WireBulkParams offsets for change notifications
#include "notify.h"

// Live configuration; defaults are the 12AX7 row (spec section 2).
volatile TubeConfig tube_config = {
    .enabled      = false,
    .tube_type    = TUBE_DEFAULT_TUBE_TYPE,
    .rectifier    = TUBE_DEFAULT_RECTIFIER,
    .xfmr_enabled = false,
    .output_mask  = TUBE_DEFAULT_OUTPUT_MASK,
    .drive_db     = TUBE_DEFAULT_DRIVE,
    .bias_pct     = TUBE_DEFAULT_BIAS,
    .asym_db      = TUBE_DEFAULT_ASYM,
    .hardness_pct = TUBE_DEFAULT_HARDNESS,
    .sag_pct      = TUBE_DEFAULT_SAG,
    .xfmr_damping = TUBE_DEFAULT_XFMR_DAMPING,
    .xfmr_res_hz  = TUBE_DEFAULT_XFMR_RES,
    .mix_pct      = TUBE_DEFAULT_MIX,
    .trim_db      = TUBE_DEFAULT_TRIM,
};
volatile bool tube_update_pending = false;

TubeOutputState tube_output_state[NUM_OUTPUT_CHANNELS];

volatile const TubeCoeffs *current_tube_coeffs = NULL;

// Double buffer so tube_apply_config() never writes through the published
// pointer while a packet is using it.
static TubeCoeffs tb_coeff_bufs[2];
static uint8_t tb_coeff_idx = 0;

// Tube style rows: bias_pct, asym_db, hardness_pct, sag_pct (spec 2.3), scaled
// for a clean default; drive brings the character up.  Never renumber.
typedef struct { float bias, asym, hardness, sag; } TubeRow;
static const TubeRow tube_rows[TUBE_TYPE_MAX] = {
    { 10.0f, 3.0f, 40.0f, 15.0f },   //  1 12AX7 / ECC83
    {  8.0f, 3.0f, 35.0f, 12.0f },   //  2 5751
    {  5.0f, 2.0f, 55.0f, 10.0f },   //  3 12AT7 / ECC81
    {  7.0f, 4.0f, 25.0f, 15.0f },   //  4 12AY7
    {  5.0f, 5.0f, 20.0f,  8.0f },   //  5 12AU7 / ECC82
    {  7.0f, 6.0f, 15.0f, 10.0f },   //  6 6SN7
    { 10.0f, 3.0f, 30.0f, 15.0f },   //  7 6SL7
    {  3.0f, 2.0f, 60.0f,  5.0f },   //  8 6DJ8 / ECC88 / 6922
    {  2.0f, 0.0f, 75.0f, 12.0f },   //  9 EF86 / 6267
    {  3.0f, 1.0f, 65.0f, 15.0f },   // 10 6SJ7
    {  0.0f, 0.0f, 50.0f, 25.0f },   // 11 EL84 / 6BQ5
    {  0.0f, 0.0f, 60.0f, 30.0f },   // 12 EL34
    {  0.0f, 0.0f, 55.0f, 18.0f },   // 13 6L6 / 5881
    {  0.0f, 0.0f, 35.0f, 30.0f },   // 14 6V6
    {  0.0f, 0.0f, 45.0f, 10.0f },   // 15 KT88 / 6550
    { 12.0f, 6.0f, 10.0f, 12.0f },   // 16 300B / 2A3
};

// Rectifier rows: sag depth scale, attack ms, release ms (spec 2.9).
typedef struct { float scale, att_ms, rel_ms; } RectRow;
static const RectRow rect_rows[TUBE_RECT_MAX + 1] = {
    { 0.0f,  0.0f,   0.0f },   // solid state: sag off
    { 0.6f,  5.0f, 120.0f },   // GZ34 / 5AR4
    { 1.0f,  8.0f, 200.0f },   // 5U4
    { 1.3f, 10.0f, 300.0f },   // 5Y3
};

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// Indexed parameter access
// ---------------------------------------------------------------------------

// Write a clamped float field and notify at its wire offset.  Returns true
// when the stored value changed (drives the tube-type custom reset).
static bool set_float_field(volatile float *field, float v, float lo, float hi,
                            uint16_t wire_off) {
    v = clampf(v, lo, hi);
    bool changed = (*field != v);
    *field = v;
    notify_param_write(wire_off, sizeof(float), &v);
    return changed;
}

static void set_u8_field(volatile uint8_t *field, uint8_t v, uint16_t wire_off) {
    *field = v;
    notify_param_write(wire_off, 1, &v);
}

static void set_bool_field(volatile bool *field, bool v, uint16_t wire_off) {
    *field = v;
    uint8_t b = v ? 1 : 0;
    notify_param_write(wire_off, 1, &b);
}

// Integer-valued parameters arrive as floats: round after clamping.
static uint32_t round_clamp_u(float v, float hi) {
    v = clampf(v, 0.0f, hi);
    return (uint32_t)(v + 0.5f);
}

// Character knobs drop the tube type back to custom when they actually
// change; a SET landing on the stored value leaves the type alone.
static void custom_reset_if(bool changed) {
    if (changed && tube_config.tube_type != TUBE_TYPE_CUSTOM) {
        set_u8_field(&tube_config.tube_type, TUBE_TYPE_CUSTOM,
                     offsetof(WireBulkParams, tube.tube_type));
    }
}

bool tube_set_param(uint8_t index, float value) {
    if (index >= TUBE_NUM_PARAMS) return false;
    if (value != value) return true;   // NaN: ignore, never store

    #define TUBE_OFF(f) ((uint16_t)offsetof(WireBulkParams, tube.f))
    bool recompute = true;
    switch (index) {
    case TUBE_PARAM_ENABLED:
        set_bool_field(&tube_config.enabled, value != 0.0f, TUBE_OFF(enabled));
        break;
    case TUBE_PARAM_OUTPUT_MASK: {
        // Read live by the pipeline each packet; no recompute needed.
        uint16_t m = (uint16_t)round_clamp_u(value, 65535.0f);
        tube_config.output_mask = m;
        notify_param_write(TUBE_OFF(output_mask), 2, &m);
        recompute = false;
        break;
    }
    case TUBE_PARAM_TUBE_TYPE: {
        uint8_t t = (uint8_t)round_clamp_u(value, (float)TUBE_TYPE_MAX);
        if (t != TUBE_TYPE_CUSTOM) {
            const TubeRow *r = &tube_rows[t - 1];
            set_float_field(&tube_config.bias_pct, r->bias, TUBE_BIAS_MIN, TUBE_BIAS_MAX, TUBE_OFF(bias_pct));
            set_float_field(&tube_config.asym_db, r->asym, TUBE_ASYM_MIN, TUBE_ASYM_MAX, TUBE_OFF(asym_db));
            set_float_field(&tube_config.hardness_pct, r->hardness, TUBE_HARDNESS_MIN, TUBE_HARDNESS_MAX, TUBE_OFF(hardness_pct));
            set_float_field(&tube_config.sag_pct, r->sag, TUBE_SAG_MIN, TUBE_SAG_MAX, TUBE_OFF(sag_pct));
        }
        set_u8_field(&tube_config.tube_type, t, TUBE_OFF(tube_type));
        break;
    }
    case TUBE_PARAM_DRIVE_DB:
        set_float_field(&tube_config.drive_db, value, TUBE_DRIVE_MIN, TUBE_DRIVE_MAX, TUBE_OFF(drive_db));
        break;
    case TUBE_PARAM_BIAS_PCT:
        custom_reset_if(set_float_field(&tube_config.bias_pct, value, TUBE_BIAS_MIN, TUBE_BIAS_MAX, TUBE_OFF(bias_pct)));
        break;
    case TUBE_PARAM_ASYM_DB:
        custom_reset_if(set_float_field(&tube_config.asym_db, value, TUBE_ASYM_MIN, TUBE_ASYM_MAX, TUBE_OFF(asym_db)));
        break;
    case TUBE_PARAM_HARDNESS_PCT:
        custom_reset_if(set_float_field(&tube_config.hardness_pct, value, TUBE_HARDNESS_MIN, TUBE_HARDNESS_MAX, TUBE_OFF(hardness_pct)));
        break;
    case TUBE_PARAM_SAG_PCT:
        custom_reset_if(set_float_field(&tube_config.sag_pct, value, TUBE_SAG_MIN, TUBE_SAG_MAX, TUBE_OFF(sag_pct)));
        break;
    case TUBE_PARAM_RECTIFIER:
        set_u8_field(&tube_config.rectifier, (uint8_t)round_clamp_u(value, (float)TUBE_RECT_MAX), TUBE_OFF(rectifier));
        break;
    case TUBE_PARAM_XFMR_ENABLED:
        set_bool_field(&tube_config.xfmr_enabled, value != 0.0f, TUBE_OFF(xfmr_enabled));
        break;
    case TUBE_PARAM_XFMR_DAMPING:
        set_float_field(&tube_config.xfmr_damping, value, TUBE_XFMR_DAMPING_MIN, TUBE_XFMR_DAMPING_MAX, TUBE_OFF(xfmr_damping));
        break;
    case TUBE_PARAM_XFMR_RES_HZ:
        set_float_field(&tube_config.xfmr_res_hz, value, TUBE_XFMR_RES_MIN, TUBE_XFMR_RES_MAX, TUBE_OFF(xfmr_res_hz));
        break;
    case TUBE_PARAM_MIX_PCT:
        set_float_field(&tube_config.mix_pct, value, TUBE_MIX_MIN, TUBE_MIX_MAX, TUBE_OFF(mix_pct));
        break;
    case TUBE_PARAM_TRIM_DB:
        set_float_field(&tube_config.trim_db, value, TUBE_TRIM_MIN, TUBE_TRIM_MAX, TUBE_OFF(trim_db));
        break;
    default:
        return false;
    }
    #undef TUBE_OFF
    if (recompute) tube_update_pending = true;
    return true;
}

bool tube_get_param(uint8_t index, float *value) {
    switch (index) {
    case TUBE_PARAM_ENABLED:      *value = tube_config.enabled ? 1.0f : 0.0f; break;
    case TUBE_PARAM_OUTPUT_MASK:  *value = (float)tube_config.output_mask; break;
    case TUBE_PARAM_TUBE_TYPE:    *value = (float)tube_config.tube_type; break;
    case TUBE_PARAM_DRIVE_DB:     *value = tube_config.drive_db; break;
    case TUBE_PARAM_BIAS_PCT:     *value = tube_config.bias_pct; break;
    case TUBE_PARAM_ASYM_DB:      *value = tube_config.asym_db; break;
    case TUBE_PARAM_HARDNESS_PCT: *value = tube_config.hardness_pct; break;
    case TUBE_PARAM_SAG_PCT:      *value = tube_config.sag_pct; break;
    case TUBE_PARAM_RECTIFIER:    *value = (float)tube_config.rectifier; break;
    case TUBE_PARAM_XFMR_ENABLED: *value = tube_config.xfmr_enabled ? 1.0f : 0.0f; break;
    case TUBE_PARAM_XFMR_DAMPING: *value = tube_config.xfmr_damping; break;
    case TUBE_PARAM_XFMR_RES_HZ:  *value = tube_config.xfmr_res_hz; break;
    case TUBE_PARAM_MIX_PCT:      *value = tube_config.mix_pct; break;
    case TUBE_PARAM_TRIM_DB:      *value = tube_config.trim_db; break;
    default: return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Coefficients
// ---------------------------------------------------------------------------

// One-pole coefficient a for y += a (x - y) at corner f (Hz), or time
// constant tau (s) via f = 1 / (2 pi tau).
static float onepole_a(float f_hz, float fs) {
    return 1.0f - expf(-2.0f * 3.1415926535f * f_hz / fs);
}
static float onepole_a_tau(float tau_s, float fs) {
    return 1.0f - expf(-1.0f / (tau_s * fs));
}

// Float shaper, used only to find the at-rest output v0 at recompute time.
static float shaper_f(float t, float ratio_n, float c1, float c3, float c5,
                      float s_p, float s_n) {
    if (t < 0.0f) t *= ratio_n;
    t = clampf(t, -1.0f, 1.0f);
    float t2 = t * t;
    float p = t * (c1 + t2 * (c3 + t2 * c5));
    return p * (t >= 0.0f ? s_p : s_n);
}

void tube_compute_coefficients(TubeCoeffs *coeffs, const TubeConfig *config, float sample_rate) {
    if (!config->enabled || sample_rate < 1.0f) {
        memset(coeffs, 0, sizeof(TubeCoeffs));
        return;
    }

    float drive_db = clampf(config->drive_db, TUBE_DRIVE_MIN, TUBE_DRIVE_MAX);
    float bias_pct = clampf(config->bias_pct, TUBE_BIAS_MIN, TUBE_BIAS_MAX);
    float asym_db  = clampf(config->asym_db, TUBE_ASYM_MIN, TUBE_ASYM_MAX);
    float hard_pct = clampf(config->hardness_pct, TUBE_HARDNESS_MIN, TUBE_HARDNESS_MAX);
    float sag_pct  = clampf(config->sag_pct, TUBE_SAG_MIN, TUBE_SAG_MAX);
    float df       = clampf(config->xfmr_damping, TUBE_XFMR_DAMPING_MIN, TUBE_XFMR_DAMPING_MAX);
    float res_hz   = clampf(config->xfmr_res_hz, TUBE_XFMR_RES_MIN, TUBE_XFMR_RES_MAX);
    float mix_pct  = clampf(config->mix_pct, TUBE_MIX_MIN, TUBE_MIX_MAX);
    float trim_db  = clampf(config->trim_db, TUBE_TRIM_MIN, TUBE_TRIM_MAX);
    uint8_t rect   = config->rectifier > TUBE_RECT_MAX ? TUBE_RECT_MAX : config->rectifier;

    // Shaper: knee fixed at t = 1; the 1/m makeup keeps small-signal gain at
    // unity so drive moves the knee, not the level (s_n <= 84 at -30 dB)
    float m = powf(10.0f, drive_db / 20.0f);              // 0.032 .. 15.85
    float h = hard_pct * 0.01f;
    float c1 = 1.5f + 0.375f * h;
    float c3 = -0.5f - 0.75f * h;
    float c5 = 0.375f * h;
    float kn = powf(10.0f, asym_db / 20.0f);              // 0.25 .. 3.98
    float ratio_n = 1.0f / kn;
    float s_p = 1.0f / (c1 * m);
    float s_n = kn / (c1 * m);
    float b = bias_pct * 0.005f;                          // -0.5 .. 0.5
    float v0 = shaper_f(b, ratio_n, c1, c3, c5, s_p, s_n);

    // Supply sag
    const RectRow *rr = &rect_rows[rect];
    float depth = clampf(sag_pct * 0.01f * rr->scale, 0.0f, TUBE_SAG_DEPTH_MAX);
    bool sag_on = (rect != TUBE_RECT_SOLID_STATE) && depth > 0.0f;
    float sagk = m * depth;
    float sag_att = sag_on ? onepole_a_tau(rr->att_ms * 1e-3f, sample_rate) : 0.0f;
    float sag_rel = sag_on ? onepole_a_tau(rr->rel_ms * 1e-3f, sample_rate) : 0.0f;

    float dc_r = expf(-2.0f * 3.1415926535f * TUBE_DC_BLOCK_HZ / sample_rate);

    // Output stage: source impedance Zn/df against a speaker whose impedance
    // rises to ZP x nominal at resonance and ZH x nominal at the top, so the
    // terminal voltage lifts by ZP (df+1)/(ZP df+1) at the bell and
    // ZH (df+1)/(ZH df+1) on the shelf.  Bell is a Cytomic TPT SVF peaking
    // section, Q fixed, A = sqrt(gain), k = 1/(Q A), mix k (A^2 - 1).
    float g_bump = TUBE_XFMR_Z_PEAK_RATIO * (df + 1.0f) / (TUBE_XFMR_Z_PEAK_RATIO * df + 1.0f);
    float g_top  = TUBE_XFMR_Z_HF_RATIO * (df + 1.0f) / (TUBE_XFMR_Z_HF_RATIO * df + 1.0f);
    float bA = sqrtf(g_bump);
    float bg = tanf(3.1415926535f * res_hz / sample_rate);
    float bk = 1.0f / (TUBE_XFMR_BELL_Q * bA);
    float bl_a1 = 1.0f / (1.0f + bg * (bg + bk));
    float bl_a2 = bg * bl_a1;
    float bl_a3 = bg * bl_a2;
    float bl_m1 = bk * (bA * bA - 1.0f);                  // <= 0.67
    float sh_a = onepole_a(TUBE_XFMR_SHELF_HZ, sample_rate);
    float sh_g = g_top - 1.0f;                            // <= 0.33

    float mix = mix_pct * 0.01f;
    float dry_w = 1.0f - mix;
    float wet_w = mix * powf(10.0f, trim_db / 20.0f);     // <= 3.98
    // RP2040 mix budget: dry term <= 4 dry_w, wet term <= wet_w wet_lim, sum
    // held at 7.5 of the 8.0 Q28 ceiling.  Bites only far above full scale.
    float wet_lim = (wet_w > 1e-6f) ? clampf((7.5f - 4.0f * dry_w) / wet_w, 0.0f, TUBE_Q28_Y2_LIM)
                                    : TUBE_Q28_Y2_LIM;

    coeffs->xfmr_on = config->xfmr_enabled ? 1 : 0;
    coeffs->sag_on = sag_on ? 1 : 0;
    coeffs->t_shift = 0;
    coeffs->s_shift = 0;

#if PICO_RP2350
    coeffs->m = m;           coeffs->sagk = sagk;       coeffs->bias = b;
    coeffs->ratio_n = ratio_n;
    coeffs->c1 = c1;         coeffs->c3 = c3;           coeffs->c5 = c5;
    coeffs->s_p = s_p;       coeffs->s_n = s_n;         coeffs->v0 = v0;
    coeffs->sag_att = sag_att; coeffs->sag_rel = sag_rel;
    coeffs->dc_r = dc_r;
    coeffs->bl_a1 = bl_a1;   coeffs->bl_a2 = bl_a2;     coeffs->bl_a3 = bl_a3;
    coeffs->bl_m1 = bl_m1;   coeffs->sh_a = sh_a;       coeffs->sh_g = sh_g;
    coeffs->dry_w = dry_w;   coeffs->wet_w = wet_w;     coeffs->wet_lim = wet_lim;
#else
    // Drive product 4 m + |b| and shaper output s_p + s_n must each stay under
    // 7.5 in their own domain.  t_shift starts at 1 because the kernel's +/-4
    // pre-clamp times ratio_n (up to 3.98) must stay under 8.  Drive limits
    // cap both shifts at 4 (spec section 7).
    uint8_t ts = 1, ss = 0;
    while (ts < 4 && m >= 1.75f * (float)(1 << ts)) ts++;
    while (ss < 4 && s_p + s_n >= 7.5f * (float)(1 << ss)) ss++;
    coeffs->t_shift = ts;
    coeffs->s_shift = ss;

    const float q28 = (float)(1LL << FILTER_SHIFT);
    const float qt = (float)(1LL << (FILTER_SHIFT - ts));
    const float qs = (float)(1LL << (FILTER_SHIFT - ss));
    coeffs->m = (int32_t)(m * qt);
    coeffs->sagk = (int32_t)(sagk * qt);
    coeffs->bias = (int32_t)(b * qt);
    coeffs->ratio_n = (int32_t)(ratio_n * q28);
    coeffs->c1 = (int32_t)(c1 * q28);
    coeffs->c3 = (int32_t)(c3 * q28);
    coeffs->c5 = (int32_t)(c5 * q28);
    coeffs->s_p = (int32_t)(s_p * qs);
    coeffs->s_n = (int32_t)(s_n * qs);
    coeffs->v0 = (int32_t)(v0 * qs);
    coeffs->sag_att = (int32_t)(sag_att * q28);
    coeffs->sag_rel = (int32_t)(sag_rel * q28);
    coeffs->dc_r = (int32_t)(dc_r * q28);
    coeffs->bl_a1 = (int32_t)(bl_a1 * q28);
    coeffs->bl_a2 = (int32_t)(bl_a2 * q28);
    coeffs->bl_a3 = (int32_t)(bl_a3 * q28);
    coeffs->bl_m1 = (int32_t)(bl_m1 * q28);
    coeffs->sh_a = (int32_t)(sh_a * q28);
    coeffs->sh_g = (int32_t)(sh_g * q28);
    coeffs->dry_w = (int32_t)(dry_w * q28);
    coeffs->wet_w = (int32_t)(wet_w * q28);
    coeffs->wet_lim = (int32_t)(wet_lim * q28);
#endif
}

void tube_apply_config(const TubeConfig *config, float sample_rate) {
    // Compute into the inactive buffer, then publish the pointer.  The
    // pipeline snapshots current_tube_coeffs once per packet.
    TubeCoeffs *next = &tb_coeff_bufs[tb_coeff_idx ^ 1];
    tube_compute_coefficients(next, config, sample_rate);
    if (config->enabled) {
        tb_coeff_idx ^= 1;
        current_tube_coeffs = next;
    } else {
        current_tube_coeffs = NULL;
    }
}

// ---------------------------------------------------------------------------
// Kernel
// ---------------------------------------------------------------------------

#if PICO_RP2350

// Branch-free float body.  Every sign-dependent select is written as a
// fmaxf/fminf split (VMAXNM/VMINNM, no flag transfer), which is bit-exact
// with the branchy form; a VCMP+VMRS pair or a taken branch costs more
// on the M33 than the arithmetic it would skip.  `xfmr` is a literal at
// each call so the compiler drops the unused arm.
static inline __attribute__((always_inline))
void tube_block_f(const TubeCoeffs * __restrict c, TubeOutputState * __restrict st,
                  float * __restrict buf, uint32_t n, const bool xfmr) {
    float env = st->env, dc_x1 = st->dc_x1, dc_y1 = st->dc_y1;
    float bl_ic1 = st->bl_ic1, bl_ic2 = st->bl_ic2, sh_lp = st->sh_lp;
    const float m = c->m, sagk = c->sagk, bias = c->bias, ratio_n = c->ratio_n;
    const float c1 = c->c1, c3 = c->c3, c5 = c->c5, s_p = c->s_p, s_n = c->s_n, v0 = c->v0;
    const float sag_att = c->sag_att, sag_rel = c->sag_rel, dc_r = c->dc_r;
    const float bl_a1 = c->bl_a1, bl_a2 = c->bl_a2, bl_a3 = c->bl_a3, bl_m1 = c->bl_m1;
    const float sh_a = c->sh_a, sh_g = c->sh_g;
    const float dry_w = c->dry_w, wet_w = c->wet_w;

    for (uint32_t i = 0; i < n; i++) {
        float x = buf[i];

        // Sag pulls the drive down with the previous sample's knee drive
        // (sagk is zero when sag is off, so no branch is needed)
        float t = (m - sagk * env) * x + bias;
        t = fmaxf(t, 0.0f) + fminf(t, 0.0f) * ratio_n;
        t = fminf(fmaxf(t, -1.0f), 1.0f);

        float t2 = t * t;
        float p = t * (c1 + t2 * (c3 + t2 * c5));
        // p carries the sign of t, so the half select splits on p
        float v = fmaxf(p, 0.0f) * s_p + fminf(p, 0.0f) * s_n - v0;

        // DC blocker: asymmetric clipping leaves a level-dependent offset
        float y = v - dc_x1 + dc_r * dc_y1;
        dc_x1 = v; dc_y1 = y;

        float a = fabsf(t);
        float d = a - env;
        env += fmaxf(d, 0.0f) * sag_att + fminf(d, 0.0f) * sag_rel;

        if (xfmr) {
            // Output stage: bell at the speaker resonance, then the top shelf
            float v3 = y - bl_ic2;
            float bv1 = bl_a1 * bl_ic1 + bl_a2 * v3;
            float bv2 = bl_ic2 + bl_a2 * bl_ic1 + bl_a3 * v3;
            bl_ic1 = 2.0f * bv1 - bl_ic1;
            bl_ic2 = 2.0f * bv2 - bl_ic2;
            y += bl_m1 * bv1;
            sh_lp += sh_a * (y - sh_lp);
            y += sh_g * (y - sh_lp);
        }

        buf[i] = dry_w * x + wet_w * y;
    }

    st->env = env; st->dc_x1 = dc_x1; st->dc_y1 = dc_y1;
    st->bl_ic1 = bl_ic1; st->bl_ic2 = bl_ic2; st->sh_lp = sh_lp;
}

DSP_TIME_CRITICAL
void tube_process_output_block(const TubeCoeffs * __restrict c,
                               TubeOutputState * __restrict st,
                               float * __restrict buf, uint32_t n) {
    if (c->xfmr_on) tube_block_f(c, st, buf, n, true);
    else            tube_block_f(c, st, buf, n, false);
}

#else

static inline int32_t clamp_lim(int32_t v, int32_t lim) {
    return v > lim ? lim : (v < -lim ? -lim : v);
}

DSP_TIME_CRITICAL
void tube_process_output_block(const TubeCoeffs * __restrict c,
                               TubeOutputState * __restrict st,
                               int32_t * __restrict buf, uint32_t n) {
    int32_t env = st->env, dc_x1 = st->dc_x1, dc_y1 = st->dc_y1;
    int32_t bl_ic1 = st->bl_ic1, bl_ic2 = st->bl_ic2, sh_lp = st->sh_lp;
    const bool sag_on = c->sag_on, xfmr_on = c->xfmr_on;
    const uint32_t ts = c->t_shift, ss = c->s_shift;
    const int32_t one_t = 1 << (FILTER_SHIFT - ts);
    const int32_t four = 4 << FILTER_SHIFT;
    const int32_t y_lim = (int32_t)(TUBE_Q28_Y_LIM * (1 << FILTER_SHIFT));
    const int32_t y_lim_s = y_lim >> ss;
    const int32_t y2_lim = (int32_t)(TUBE_Q28_Y2_LIM * (1 << FILTER_SHIFT));
    const int32_t bell_in = (int32_t)(TUBE_Q28_BELL_IN * (1 << FILTER_SHIFT));
    const int32_t wet_lim = c->wet_lim;

    for (uint32_t i = 0; i < n; i++) {
        int32_t x = buf[i];
        int32_t x4 = clamp_lim(x, four);

        // Drive product in the /2^ts domain stays under 7.5.  Pre-clamping t
        // to +/-4 before the negative knee ratio keeps that product in range;
        // see spec 2.4 for the one lossy corner past +12 dBFS.
        int32_t m_eff = sag_on ? c->m - fast_mul_q28(c->sagk, env) : c->m;
        int32_t tt = fast_mul_q28(m_eff, x4) + c->bias;
        if (tt < 0) tt = fast_mul_q28(clamp_lim(tt, 4 * one_t), c->ratio_n);
        if (tt > one_t) tt = one_t; else if (tt < -one_t) tt = -one_t;
        int32_t t = tt << ts;

        int32_t t2 = fast_mul_q28(t, t);
        int32_t p = fast_mul_q28(t, c->c1 + fast_mul_q28(t2, c->c3 + fast_mul_q28(t2, c->c5)));
        // Shaper output is formed in the /2^ss domain (s_n reaches 84), then
        // clamped so the DC blocker's 2 max|v| stays under the Q28 ceiling.
        int32_t v = clamp_lim(fast_mul_q28(p, t >= 0 ? c->s_p : c->s_n) - c->v0, y_lim_s) << ss;

        // DC blocker output <= 6.8; the state keeps the true value, the
        // clamped copy bounds every later operand.
        int32_t y = v - dc_x1 + fast_mul_q28(c->dc_r, dc_y1);
        dc_x1 = v; dc_y1 = y;
        y = clamp_lim(y, y_lim);

        int32_t a = t < 0 ? -t : t;
        env += fast_mul_q28(a - env, a > env ? c->sag_att : c->sag_rel);

        if (xfmr_on) {
            // Bell input clamp bounds the SVF difference term; bell output is
            // clamped again so the shelf difference stays under 6.8.
            y = clamp_lim(y, bell_in);
            int32_t v3 = y - bl_ic2;
            int32_t bv1 = fast_mul_q28(c->bl_a1, bl_ic1) + fast_mul_q28(c->bl_a2, v3);
            int32_t bv2 = bl_ic2 + fast_mul_q28(c->bl_a2, bl_ic1) + fast_mul_q28(c->bl_a3, v3);
            bl_ic1 = 2 * bv1 - bl_ic1;
            bl_ic2 = 2 * bv2 - bl_ic2;
            y = clamp_lim(y + fast_mul_q28(c->bl_m1, bv1), y2_lim);
            sh_lp += fast_mul_q28(c->sh_a, y - sh_lp);
            y += fast_mul_q28(c->sh_g, y - sh_lp);
        }

        // Mix budget: 4 dry_w + wet_w wet_lim <= 7.5 (see wet_lim)
        y = clamp_lim(y, wet_lim);
        buf[i] = fast_mul_q28(c->dry_w, x4) + fast_mul_q28(c->wet_w, y);
    }

    st->env = env; st->dc_x1 = dc_x1; st->dc_y1 = dc_y1;
    st->bl_ic1 = bl_ic1; st->bl_ic2 = bl_ic2; st->sh_lp = sh_lp;
}

#endif
