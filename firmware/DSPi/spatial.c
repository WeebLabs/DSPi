/*
 * Stereo spatializer (both platforms).  Width (mid/side with a Blumlein/Gerzon
 * shuffler) and recursive crosstalk cancellation (Atal/Schroeder, Cooper/Bauck
 * shuffler form, Glasgal RACE).  Design: Documentation/Features/stereo_spatializer_spec.md.
 */

#include <math.h>
#include <stddef.h>
#include <string.h>
#include "spatial.h"
#include "dsp_pipeline.h"   // fast_mul_q28
#include "usb_audio.h"      // matrix_mixer: per-output enables gate pair processing
#include "siggen.h"         // siggen_raw_mask: RAW outputs are never processed
#include "bulk_params.h"    // WireBulkParams offsets for change notifications
#include "notify.h"

#define SP_DEFAULTS { \
    .enabled = false, .mode = SPATIAL_DEFAULT_MODE, .pair_mask = SPATIAL_DEFAULT_PAIRS, \
    .width_pct = SPATIAL_DEFAULT_WIDTH, .bass_width_pct = SPATIAL_DEFAULT_BASS_WIDTH, \
    .shelf_hz = SPATIAL_DEFAULT_SHELF, .span_deg = SPATIAL_DEFAULT_SPAN, \
    .atten_db = SPATIAL_DEFAULT_ATTEN, .mid_pct = SPATIAL_DEFAULT_MID, .trim_db = SPATIAL_DEFAULT_TRIM }

volatile SpatialConfig spatial_config = SP_DEFAULTS;
volatile bool spatial_update_pending = false;
volatile const SpatialCoeffs *current_spatial_coeffs = NULL;
SpatialPairState spatial_pair_state[NUM_SPDIF_INSTANCES];

// Double buffer: compute into the inactive one, then publish the pointer.
static SpatialCoeffs sp_coeff_bufs[2];
static uint8_t sp_coeff_idx;

void spatial_config_defaults(volatile SpatialConfig *cfg) {
    const SpatialConfig d = SP_DEFAULTS;
    *cfg = d;
}

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// Arithmetic: float on RP2350, Q28 on RP2040 (spec 7.3 bounds every value).
// ---------------------------------------------------------------------------
#if PICO_RP2350
#define SP_ONE            1.0f
#define SP_MUL(a, b)      ((a) * (b))
#define SP_CLAMP(x, lim)  (x)
static inline sp_t sp_from_f(float v) { return v; }
static inline sp_t sp_out(sp_t x, sp_t fd, bool trim) { (void)trim; return x + fd; }
#else
#define SP_ONE            ((sp_t)(1 << 28))
#define SP_MUL(a, b)      fast_mul_q28((a), (b))
#define SP_Q(x)           ((sp_t)((x) * 268435456.0))
static inline sp_t sp_clamp(sp_t x, sp_t lim) { return x > lim ? lim : (x < -lim ? -lim : x); }
#define SP_CLAMP(x, lim)  sp_clamp((x), SP_Q(lim))
static inline sp_t sp_from_f(float v) { return (sp_t)lrintf(v * 268435456.0f); }
// Saturating output add: +-6.9 when the trim multiply follows (its operands must sum under 8).
static inline sp_t sp_out(sp_t x, sp_t fd, bool trim) {
    int64_t o = (int64_t)x + fd;
    int64_t lim = trim ? SP_Q(6.9) : SP_Q(7.9);
    return (sp_t)(o > lim ? lim : (o < -lim ? -lim : o));
}
#endif

// ---------------------------------------------------------------------------
// Coefficients
// ---------------------------------------------------------------------------

// Peak of |B| for B = LP5 * (1 - LP250), both one-poles y += a (x - y), on a log grid.
static float sp_band_peak(float a5, float ah, float fs) {
    const float pi = 3.1415926535f;
    float peak = 0.0f;
    for (int i = 0; i < 128; i++) {
        float f = 20.0f * powf(0.45f * fs / 20.0f, (float)i / 127.0f);
        float w = 2.0f * pi * f / fs, cr = cosf(w), ci = -sinf(w);   // z^-1
        // one-pole a / (1 - (1 - a) z^-1)
        float dr5 = 1.0f - (1.0f - a5) * cr, di5 = -(1.0f - a5) * ci;
        float drh = 1.0f - (1.0f - ah) * cr, dih = -(1.0f - ah) * ci;
        float m5 = a5 / sqrtf(dr5 * dr5 + di5 * di5);
        // 1 - ah / D = (D - ah) / D
        float nr = drh - ah, ni = dih;
        float mh = sqrtf(nr * nr + ni * ni) / sqrtf(drh * drh + dih * dih);
        float m = m5 * mh;
        if (m > peak) peak = m;
    }
    return peak;
}

void spatial_compute_coefficients(SpatialCoeffs *c, const SpatialConfig *cfg, float fs) {
    const float pi = 3.1415926535f;
    memset(c, 0, sizeof(*c));
    if (fs < 1.0f) fs = 48000.0f;
    c->fs = (uint32_t)lrintf(fs);
    c->enabled = cfg->enabled ? 1 : 0;
    c->mode = cfg->mode <= SPATIAL_MODE_MAX ? cfg->mode : SPATIAL_MODE_WIDTH;
    int fl = (int)lrintf(SPATIAL_FADE_MS * 0.001f * fs);
    c->fade_len = (uint16_t)(fl < 1 ? 1 : fl);
    c->fade_step = sp_from_f(1.0f / (float)c->fade_len);

    float w = clampf(cfg->width_pct, SPATIAL_WIDTH_MIN, SPATIAL_WIDTH_MAX) * 0.01f;
    float wb = clampf(cfg->bass_width_pct, SPATIAL_WIDTH_MIN, SPATIAL_WIDTH_MAX) * 0.01f;
    float fc = clampf(cfg->shelf_hz, SPATIAL_SHELF_MIN, SPATIAL_SHELF_MAX);
    c->w1 = sp_from_f(w - 1.0f);
    c->dwb = sp_from_f(wb - w);
    c->aw = sp_from_f(1.0f - expf(-2.0f * pi * fc / fs));

    float theta = clampf(cfg->span_deg, SPATIAL_SPAN_MIN, SPATIAL_SPAN_MAX) * pi / 180.0f;
    float d = SPATIAL_HEAD_RADIUS_M / SPATIAL_SOUND_M_S * (theta + sinf(theta)) * fs;
    float x = expf(-2.0f * pi * SPATIAL_XTC_LP_HZ / fs);
    float dl = d - x / (1.0f - x);               // the loop low-pass already delays this much
    dl = clampf(dl, 1.0f, (float)(SPATIAL_RING - 2));
    float a5 = 1.0f - x;
    float ah = 1.0f - expf(-2.0f * pi * SPATIAL_XTC_HP_HZ / fs);
    // k depends only on fs; the scan is soft float on RP2040, so it runs once per rate.
    static float k_fs = 0.0f, k_val = 1.0f;
    if (fs != k_fs) { k_val = 0.999f / sp_band_peak(a5, ah, fs); k_fs = fs; }
    float k = k_val;
    float g = powf(10.0f, -clampf(cfg->atten_db, SPATIAL_ATTEN_MIN, SPATIAL_ATTEN_MAX) / 20.0f) * k;
    c->a5 = sp_from_f(a5);
    c->ah = sp_from_f(ah);
    c->g = sp_from_f(g);
    c->gm = sp_from_f(g * clampf(cfg->mid_pct, SPATIAL_MID_MIN, SPATIAL_MID_MAX) * 0.01f);
    c->t = sp_from_f(powf(10.0f, clampf(cfg->trim_db, SPATIAL_TRIM_MIN, SPATIAL_TRIM_MAX) / 20.0f));
#if PICO_RP2350
    c->d = dl;
#else
    c->d = (sp_t)lrintf(dl * 65536.0f);          // Q16.16
#endif
}

void spatial_apply_config(float fs) {
    SpatialConfig cfg = spatial_config;
    SpatialCoeffs *next = &sp_coeff_bufs[sp_coeff_idx ^ 1];
    spatial_compute_coefficients(next, &cfg, fs);
    sp_coeff_idx ^= 1;
    current_spatial_coeffs = next;
}

// ---------------------------------------------------------------------------
// Codec and vendor access
// ---------------------------------------------------------------------------

void spatial_config_to_packet(SpatialConfigPacket *out) {
    memset(out, 0, sizeof(*out));
    out->enabled = spatial_config.enabled ? 1 : 0;
    out->mode = spatial_config.mode;
    out->pair_mask = spatial_config.pair_mask;
    out->width_pct = spatial_config.width_pct;
    out->bass_width_pct = spatial_config.bass_width_pct;
    out->shelf_hz = spatial_config.shelf_hz;
    out->span_deg = spatial_config.span_deg;
    out->atten_db = spatial_config.atten_db;
    out->mid_pct = spatial_config.mid_pct;
    out->trim_db = spatial_config.trim_db;
}

static inline float sp_fin(float v, float def) { return isfinite(v) ? v : def; }

void spatial_packet_to_config(const SpatialConfigPacket *in) {
    bool mode_ok = in->mode <= SPATIAL_MODE_MAX;
    spatial_config.enabled = mode_ok && in->enabled != 0;   // unknown mode: stage off
    spatial_config.mode = mode_ok ? in->mode : SPATIAL_MODE_WIDTH;
    spatial_config.pair_mask = in->pair_mask & SPATIAL_PAIR_MASK;
    spatial_config.width_pct = sp_fin(in->width_pct, SPATIAL_DEFAULT_WIDTH);
    spatial_config.bass_width_pct = sp_fin(in->bass_width_pct, SPATIAL_DEFAULT_BASS_WIDTH);
    spatial_config.shelf_hz = sp_fin(in->shelf_hz, SPATIAL_DEFAULT_SHELF);
    spatial_config.span_deg = sp_fin(in->span_deg, SPATIAL_DEFAULT_SPAN);
    spatial_config.atten_db = sp_fin(in->atten_db, SPATIAL_DEFAULT_ATTEN);
    spatial_config.mid_pct = sp_fin(in->mid_pct, SPATIAL_DEFAULT_MID);
    spatial_config.trim_db = sp_fin(in->trim_db, SPATIAL_DEFAULT_TRIM);
}

void spatial_notify_config(void) {
    SpatialConfigPacket p;
    spatial_config_to_packet(&p);
    #define SP_NOTIFY(f) notify_param_write((uint16_t)offsetof(WireBulkParams, spatial.f), sizeof(p.f), &p.f)
    SP_NOTIFY(enabled);
    SP_NOTIFY(mode);
    SP_NOTIFY(pair_mask);
    SP_NOTIFY(width_pct);
    SP_NOTIFY(bass_width_pct);
    SP_NOTIFY(shelf_hz);
    SP_NOTIFY(span_deg);
    SP_NOTIFY(atten_db);
    SP_NOTIFY(mid_pct);
    SP_NOTIFY(trim_db);
    #undef SP_NOTIFY
}

float spatial_get_param(uint8_t id) {
    switch (id) {
    case SPATIAL_PARAM_ENABLED:    return spatial_config.enabled ? 1.0f : 0.0f;
    case SPATIAL_PARAM_MODE:       return (float)spatial_config.mode;
    case SPATIAL_PARAM_PAIRS:      return (float)spatial_config.pair_mask;
    case SPATIAL_PARAM_WIDTH:      return spatial_config.width_pct;
    case SPATIAL_PARAM_BASS_WIDTH: return spatial_config.bass_width_pct;
    case SPATIAL_PARAM_SHELF:      return spatial_config.shelf_hz;
    case SPATIAL_PARAM_SPAN:       return spatial_config.span_deg;
    case SPATIAL_PARAM_ATTEN:      return spatial_config.atten_db;
    case SPATIAL_PARAM_MID:        return spatial_config.mid_pct;
    case SPATIAL_PARAM_TRIM:       return spatial_config.trim_db;
    default:                       return 0.0f;
    }
}

bool spatial_set_param(uint8_t id, float v) {
    if (!isfinite(v) || id >= SPATIAL_PARAM_COUNT) return false;
    switch (id) {
    case SPATIAL_PARAM_ENABLED:    spatial_config.enabled = v != 0.0f; break;
    case SPATIAL_PARAM_MODE:
        if (v < 0.0f || v > (float)SPATIAL_MODE_MAX) return false;
        spatial_config.mode = (uint8_t)lrintf(v);
        break;
    case SPATIAL_PARAM_PAIRS:      spatial_config.pair_mask = (uint8_t)lrintf(clampf(v, 0, 255)) & SPATIAL_PAIR_MASK; break;
    case SPATIAL_PARAM_WIDTH:      spatial_config.width_pct = clampf(v, SPATIAL_WIDTH_MIN, SPATIAL_WIDTH_MAX); break;
    case SPATIAL_PARAM_BASS_WIDTH: spatial_config.bass_width_pct = clampf(v, SPATIAL_WIDTH_MIN, SPATIAL_WIDTH_MAX); break;
    case SPATIAL_PARAM_SHELF:      spatial_config.shelf_hz = clampf(v, SPATIAL_SHELF_MIN, SPATIAL_SHELF_MAX); break;
    case SPATIAL_PARAM_SPAN:       spatial_config.span_deg = clampf(v, SPATIAL_SPAN_MIN, SPATIAL_SPAN_MAX); break;
    case SPATIAL_PARAM_ATTEN:      spatial_config.atten_db = clampf(v, SPATIAL_ATTEN_MIN, SPATIAL_ATTEN_MAX); break;
    case SPATIAL_PARAM_MID:        spatial_config.mid_pct = clampf(v, SPATIAL_MID_MIN, SPATIAL_MID_MAX); break;
    case SPATIAL_PARAM_TRIM:       spatial_config.trim_db = clampf(v, SPATIAL_TRIM_MIN, SPATIAL_TRIM_MAX); break;
    }
    if (id != SPATIAL_PARAM_PAIRS) spatial_update_pending = true;   // the mask is read live
    spatial_notify_config();
    return true;
}

bool spatial_set_config(const SpatialConfigPacket *in) {
    const float f[] = { in->width_pct, in->bass_width_pct, in->shelf_hz, in->span_deg,
                        in->atten_db, in->mid_pct, in->trim_db };
    for (unsigned i = 0; i < sizeof(f) / sizeof(f[0]); i++)
        if (!isfinite(f[i])) return false;
    if (in->mode > SPATIAL_MODE_MAX) return false;
    spatial_config.enabled = in->enabled != 0;
    spatial_config.mode = in->mode;
    spatial_config.pair_mask = in->pair_mask & SPATIAL_PAIR_MASK;
    spatial_config.width_pct = clampf(in->width_pct, SPATIAL_WIDTH_MIN, SPATIAL_WIDTH_MAX);
    spatial_config.bass_width_pct = clampf(in->bass_width_pct, SPATIAL_WIDTH_MIN, SPATIAL_WIDTH_MAX);
    spatial_config.shelf_hz = clampf(in->shelf_hz, SPATIAL_SHELF_MIN, SPATIAL_SHELF_MAX);
    spatial_config.span_deg = clampf(in->span_deg, SPATIAL_SPAN_MIN, SPATIAL_SPAN_MAX);
    spatial_config.atten_db = clampf(in->atten_db, SPATIAL_ATTEN_MIN, SPATIAL_ATTEN_MAX);
    spatial_config.mid_pct = clampf(in->mid_pct, SPATIAL_MID_MIN, SPATIAL_MID_MAX);
    spatial_config.trim_db = clampf(in->trim_db, SPATIAL_TRIM_MIN, SPATIAL_TRIM_MAX);
    spatial_update_pending = true;
    spatial_notify_config();
    return true;
}

// ---------------------------------------------------------------------------
// Processing
// ---------------------------------------------------------------------------

static inline void sp_reset(SpatialPairState *st, uint32_t fs) {
    memset(st, 0, sizeof(*st));
    st->fs = fs;
}

// Ramp step that lands on `target` after n samples (the end snaps exactly).
static inline sp_t sp_step(sp_t cur, sp_t target, uint32_t n) {
#if PICO_RP2350
    return (target - cur) / (float)n;
#else
    return (target - cur) / (int32_t)n;   // |target - cur| < 4.0 in Q28, fits int32
#endif
}

// Fade gain for a fade position (exactly 0 and exactly 1 at the ends).
static inline sp_t sp_fade(const SpatialCoeffs *c, uint16_t fade) {
    if (fade >= c->fade_len) return SP_ONE;
#if PICO_RP2350
    return (float)fade * c->fade_step;
#else
    return (sp_t)fade * c->fade_step;
#endif
}

// One XTC path step: reads the tap at delay d, returns the scaled, band-limited
// feedback (g * H applied to the path's past output) clamped to the delta bound.
DSP_TIME_CRITICAL __attribute__((noinline))
static sp_t sp_xtc_tap(SpatialXtcPath *p, uint8_t widx, sp_t d, sp_t g, sp_t a5, sp_t ah) {
#if PICO_RP2350
    int di = (int)d;
    sp_t fr = d - (float)di;
#else
    int di = d >> 16;
    sp_t fr = (sp_t)((d & 0xFFFF) << 12);
#endif
    sp_t a = p->ring[(uint8_t)(widx - di) & (SPATIAL_RING - 1)];
    sp_t b = p->ring[(uint8_t)(widx - di - 1) & (SPATIAL_RING - 1)];
    sp_t v = SP_MUL(g, SP_MUL(a, SP_ONE - fr) + SP_MUL(b, fr));
    p->lp += SP_MUL(a5, v - p->lp);
    p->hl += SP_MUL(ah, p->lp - p->hl);
    return SP_CLAMP(p->lp - p->hl, 2.9);
}

// Per-packet setup shared by both kernels: mode adoption at fade 0, priming, and a fresh
// 10 ms ramp from the values in use whenever a target, the shelf coefficient included, moves (spec 6).
typedef struct {
    int dir;
    bool trim;
} SpRamp;

DSP_TIME_CRITICAL __attribute__((noinline))
static void sp_begin(const SpatialCoeffs *c, SpatialPairState *st, bool want, SpRamp *r) {
    if (st->fade == 0 && st->mode != c->mode) {      // a mode change completes at fade 0
        uint32_t fs = st->fs;
        sp_reset(st, fs);
        st->mode = c->mode;
    }
    r->dir = (want && st->mode == c->mode) ? 1 : -1;
    if (!st->primed) {               // first packet after a reset: no ramp, the fade covers it
        st->w1 = st->tw1 = c->w1; st->dwb = st->tdwb = c->dwb; st->aw = st->taw = c->aw; st->g = st->tg = c->g;
        st->gm = st->tgm = c->gm; st->t = st->tt = c->t; st->d = c->d;
        // XTC starts with an empty ring, so its loop gains ramp up from 0 with the fade;
        // at full gain the band-pass sees a cold onset and clicks (spec 6).
        if (st->mode == SPATIAL_MODE_XTC) st->g = st->tg = st->gm = st->tgm = 0;
        st->ramp = 0;
        st->primed = true;
    }
    st->dirty = true;
    if (c->w1 != st->tw1 || c->dwb != st->tdwb || c->aw != st->taw || c->g != st->tg || c->gm != st->tgm
        || c->t != st->tt) {
        uint32_t n = c->fade_len;
        st->tw1 = c->w1; st->tdwb = c->dwb; st->taw = c->aw; st->tg = c->g; st->tgm = c->gm; st->tt = c->t;
        st->sw1 = sp_step(st->w1, c->w1, n);
        st->sdwb = sp_step(st->dwb, c->dwb, n);
        st->saw = sp_step(st->aw, c->aw, n);
        st->sg = sp_step(st->g, c->g, n);
        st->sgm = sp_step(st->gm, c->gm, n);
        st->st = sp_step(st->t, c->t, n);
        st->ramp = (uint16_t)n;
    }
    r->trim = st->t != SP_ONE || st->tt != SP_ONE;
}

// After the loop: a finished ramp leaves every value exactly on its target.
static inline void sp_end(SpatialPairState *st) {
    if (st->ramp == 0) {
        st->w1 = st->tw1; st->dwb = st->tdwb; st->aw = st->taw; st->g = st->tg; st->gm = st->tgm; st->t = st->tt;
    }
}

static inline uint16_t sp_fade_step(const SpatialCoeffs *c, uint16_t fade, int dir) {
    int fi = (int)fade + dir;
    return (uint16_t)(fi < 0 ? 0 : (fi > c->fade_len ? c->fade_len : fi));
}

static inline void sp_write(sp_t *L, sp_t *R, uint32_t i, sp_t fdm, sp_t fds, sp_t f, sp_t t, bool trim) {
    sp_t lo = sp_out(L[i], fdm + fds, trim), ro = sp_out(R[i], fdm - fds, trim);
    if (trim) {
        sp_t T = SP_ONE + SP_MUL(f, t - SP_ONE);
        lo = SP_MUL(T, lo);
        ro = SP_MUL(T, ro);
    }
    L[i] = lo;
    R[i] = ro;
}

DSP_TIME_CRITICAL __attribute__((noinline, optimize("O2")))
static void sp_width(const SpatialCoeffs *c, SpatialPairState *st, sp_t *L, sp_t *R, uint32_t n, bool want) {
    SpRamp r;
    sp_begin(c, st, want, &r);
    sp_t w1 = st->w1, dwb = st->dwb, aw = st->aw, t = st->t, lpw = st->lpw;
    uint16_t fade = st->fade, ramp = st->ramp;
    for (uint32_t i = 0; i < n; i++) {
        fade = sp_fade_step(c, fade, r.dir);
        sp_t f = sp_fade(c, fade);
        if (ramp) {
            w1 += st->sw1; dwb += st->sdwb; aw += st->saw; t += st->st;
            if (--ramp == 0) { w1 = st->tw1; dwb = st->tdwb; aw = st->taw; t = st->tt; }
        }
#if PICO_RP2350
        sp_t S = 0.5f * (L[i] - R[i]);
#else
        sp_t S = SP_CLAMP((L[i] >> 1) - (R[i] >> 1), 2.0);
#endif
        lpw += SP_MUL(aw, S - lpw);
        sp_t dS = SP_CLAMP(SP_MUL(w1, S) + SP_MUL(dwb, lpw), 5.8);
        sp_write(L, R, i, 0, SP_MUL(f, dS), f, t, r.trim);
    }
    st->w1 = w1; st->dwb = dwb; st->aw = aw; st->t = t;
    st->lpw = lpw;
    st->fade = fade;
    st->ramp = ramp;
    sp_end(st);
}

DSP_TIME_CRITICAL __attribute__((noinline, optimize("O2")))
static void sp_xtc(const SpatialCoeffs *c, SpatialPairState *st, sp_t *L, sp_t *R, uint32_t n, bool want) {
    SpRamp r;
    sp_begin(c, st, want, &r);
    sp_t g = st->g, gm = st->gm, t = st->t, d = st->d;
    // The mid path runs on for a fade length after gm settles at 0, so its filter tail decays.
    const bool m_live = gm != 0 || st->tgm != 0;
    if (m_live) st->m_tail = c->fade_len;
    const bool m_on = m_live || st->m_tail != 0;
    if (!m_on && st->m_dirty) { st->m.lp = st->m.hl = 0; st->m_dirty = false; }
#if PICO_RP2350
    const sp_t slew = 1.0f / 64.0f;
#else
    const sp_t slew = 1 << 10;       // 1/64 sample in Q16.16
#endif
    uint16_t fade = st->fade, ramp = st->ramp;
    uint8_t widx = st->widx;
    for (uint32_t i = 0; i < n; i++) {
        fade = sp_fade_step(c, fade, r.dir);
        sp_t f = sp_fade(c, fade);
        if (ramp) {
            g += st->sg; gm += st->sgm; t += st->st;
            if (--ramp == 0) { g = st->tg; gm = st->tgm; t = st->tt; }
        }
        if (d < c->d) { d += slew; if (d > c->d) d = c->d; }
        else if (d > c->d) { d -= slew; if (d < c->d) d = c->d; }
#if PICO_RP2350
        sp_t S = 0.5f * (L[i] - R[i]), M = 0.5f * (L[i] + R[i]);
#else
        sp_t S = SP_CLAMP((L[i] >> 1) - (R[i] >> 1), 2.0);
        sp_t M = SP_CLAMP((L[i] >> 1) + (R[i] >> 1), 2.0);
#endif
        // S' = S / (1 - g H): dS = +g H(S').  M' = M / (1 + gm H): dM = -gm H(M').
        sp_t dS = sp_xtc_tap(&st->s, widx, d, g, c->a5, c->ah);
        st->s.ring[widx & (SPATIAL_RING - 1)] = SP_CLAMP(S + dS, 3.9);
        // While the mid path is off its output is M, so the ring keeps M and gm can ramp in warm.
        sp_t dM = 0;
        if (m_on) dM = -sp_xtc_tap(&st->m, widx, d, gm, c->a5, c->ah);
        st->m.ring[widx & (SPATIAL_RING - 1)] = SP_CLAMP(M + dM, 3.9);
        widx++;
        sp_write(L, R, i, SP_MUL(f, dM), SP_MUL(f, dS), f, t, r.trim);
    }
    if (m_on) st->m_dirty = true;
    if (!m_live) st->m_tail = st->m_tail > n ? (uint16_t)(st->m_tail - n) : 0;
    st->g = g; st->gm = gm; st->t = t;
    st->widx = widx;
    st->d = d;
    st->fade = fade;
    st->ramp = ramp;
    sp_end(st);
}

DSP_TIME_CRITICAL
void spatial_process_pairs(const SpatialCoeffs *c, uint32_t pair_mask, int first_pair, int last_pair,
                           sp_t (*buf_out)[AUDIO_BUFFER_SAMPLES], uint32_t n) {
    if (!c || n == 0) return;
    for (int p = first_pair; p <= last_pair; p++) {
        SpatialPairState *st = &spatial_pair_state[p];
        int l = 2 * p;
        // RAW generator outputs and half-enabled pairs skip at once (spec 6, hard skips).
        bool hard = (siggen_raw_mask & (3u << l)) || !matrix_mixer.outputs[l].enabled
                    || !matrix_mixer.outputs[l + 1].enabled;
        if (hard || st->fs != c->fs) {
            if (st->dirty || st->fs != c->fs) sp_reset(st, c->fs);
            if (hard) continue;
        }
        bool want = c->enabled && ((pair_mask >> p) & 1u);
        if (!want && st->fade == 0) {
            if (st->dirty) sp_reset(st, c->fs);
            continue;
        }
        // The kernel that runs is the target mode at fade 0 (sp_begin adopts it), else the running one.
        uint8_t m = st->fade == 0 ? c->mode : st->mode;
        if (m == SPATIAL_MODE_XTC) sp_xtc(c, st, buf_out[l], buf_out[l + 1], n, want);
        else sp_width(c, st, buf_out[l], buf_out[l + 1], n, want);
    }
}
