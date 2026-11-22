/* Host harness: compiles the real upmix.c (copied next to the stubs by
 * harness.py) into a shared library driven from Python through ctypes. */
#include <stdlib.h>
#include <stdint.h>

/* Every rendered chunk lands in h_tap at h_tap_base + off (route check). */
static float *h_tap[7];
static int h_tap_base;
static void h_tap_chunk(float *const *gen, uint32_t off, uint32_t k);
#define UPMIX_HOST_TAP(gen, off, k) h_tap_chunk((gen), (off), (k))

#include "upmix.c"

static void h_tap_chunk(float *const *gen, uint32_t off, uint32_t k) {
    for (int g = 0; g < 7; g++)
        if (h_tap[g]) memcpy(h_tap[g] + h_tap_base + off, gen[g], k * sizeof(float));
}

#ifdef UPMIX_TRIM_MIN_Q1
/* The firmware's v1 row bridge left with module routing; the harness keeps its call shape. */
static float hb_ab[2][UPMIX_CHUNK], hb_back[2][UPMIX_CHUNK];
static void h_process_block(const UpmixCoeffs *c, float *a, float *b, float *cb,
                            float *ls, float *rs, uint32_t n) {
    upmix_begin(c, n);
    for (uint32_t off = 0; off < n; off += UPMIX_CHUNK) {
        uint32_t k = n - off < UPMIX_CHUNK ? n - off : UPMIX_CHUNK;
        float *const out[UPMIX_NUM_GEN] = { hb_ab[0], hb_ab[1], cb + off, ls + off, rs + off,
                                            hb_back[0], hb_back[1] };
        upmix_render(c, a + off, b + off, k, out);
        memcpy(a + off, hb_ab[0], k * sizeof(float));
        memcpy(b + off, hb_ab[1], k * sizeof(float));
    }
}
#else
#define h_process_block upmix_process_block
#endif

static void set_cfg(UpmixCoeffs *c, const float *p, int cmode, int smode, float fs) {
    UpmixConfig cfg = {
        .enabled = true, .center_mode = (uint8_t)cmode, .surround_mode = (uint8_t)smode,
        .strength_pct = p[0], .center_width_pct = p[1], .corr_threshold_pct = p[2],
        .attack_ms = p[3], .release_ms = p[4], .detector_hpf_hz = p[5],
        .surround_delay_ms = p[6], .surround_hpf_hz = p[7], .surround_lpf_hz = p[8],
        .decorr_pct = p[9], .presence_db = p[10],
    };
    upmix_compute_coefficients(c, &cfg, fs);
}

/* Coefficients in the model's K order, plus block-rate powf values the model cannot reproduce bit-exactly. */
int h_coeffs(const float *p, int cmode, int smode, int n, float *K) {
    UpmixCoeffs c;
    set_cfg(&c, p, cmode, smode, 48000.0f);
    float v[] = {c.strength, c.width, c.corr_thresh, c.inv_thresh_range, c.alpha_att, c.alpha_rel,
                 c.alpha_corr, c.alpha_dom, c.det_hp_a, c.pres_a1, c.pres_a2, c.pres_a3, c.pres_m1,
                 c.ls_cl, c.ls_cr, c.ap_g, (float)c.haas_delay, (float)c.ap_delay,
                 c.shp_a1, c.shp_a2, c.shp_a3, c.shp_k, c.slp_a1, c.slp_a2, c.slp_a3, c.env_step,
                 (float)c.center_mode,
                 powf(c.alpha_att, (float)n), powf(c.alpha_rel, (float)n),
#ifdef UPMIX_NEW
                 c.alpha_w, c.duck_af, c.duck_as, powf(c.alpha_drel, (float)n),
                 UPMIX_GATE_K, UPMIX_FLOOR_Q,
#endif
    };
    memcpy(K, v, sizeof(v));
    return (int)(sizeof(v) / sizeof(v[0]));
}

/* Whole-signal run from reset state in fixed blocks. */
void h_run(const float *p, int cmode, int smode, int blk, const float *L, const float *R, int N,
           float *oc, float *ol, float *orr, float *ols, float *ors) {
    UpmixCoeffs c;
    set_cfg(&c, p, cmode, smode, 48000.0f);
    upmix_reset_state();
    for (int s = 0; s < N; s += blk) {
        int n = N - s < blk ? N - s : blk;
        memcpy(ol + s, L + s, n * sizeof(float));
        memcpy(orr + s, R + s, n * sizeof(float));
        h_process_block(&c, ol + s, orr + s, oc + s, ols + s, ors + s, (uint32_t)n);
    }
}

/* Mode switch mid-stream without a park: sm_a before cut, sm_b from cut on. */
void h_run_switch(const float *p, int cmode, int sm_a, int sm_b, int cut, int blk,
                  const float *L, const float *R, int N,
                  float *oc, float *ol, float *orr, float *ols, float *ors) {
    UpmixCoeffs ca, cb;
    set_cfg(&ca, p, cmode, sm_a, 48000.0f);
    set_cfg(&cb, p, cmode, sm_b, 48000.0f);
    upmix_reset_state();
    for (int s = 0; s < N; s += blk) {
        int n = N - s < blk ? N - s : blk;
        memcpy(ol + s, L + s, n * sizeof(float));
        memcpy(orr + s, R + s, n * sizeof(float));
        h_process_block(s < cut ? &ca : &cb, ol + s, orr + s, oc + s, ols + s, ors + s, (uint32_t)n);
    }
}

/* Reference run with both modes switchable at sample `cut`, through the v1 call shape. */
void h_run_ref(const float *p, int cm_a, int sm_a, int cm_b, int sm_b, int cut, int blk,
               const float *L, const float *R, int N,
               float *oc, float *ol, float *orr, float *ols, float *ors) {
    UpmixCoeffs ca, cb;
    set_cfg(&ca, p, cm_a, sm_a, 48000.0f);
    set_cfg(&cb, p, cm_b, sm_b, 48000.0f);
    upmix_reset_state();
    for (int s = 0; s < N; s += blk) {
        int n = N - s < blk ? N - s : blk;
        memcpy(ol + s, L + s, n * sizeof(float));
        memcpy(orr + s, R + s, n * sizeof(float));
        h_process_block(s < cut ? &ca : &cb, ol + s, orr + s, oc + s, ols + s, ors + s, (uint32_t)n);
    }
}

#ifdef UPMIX_CHUNK
/* Same run through upmix_begin()/upmix_render() at an arbitrary chunk size. */
void h_run_split(const float *p, int cm_a, int sm_a, int cm_b, int sm_b, int cut, int blk, int chunk,
                 const float *L, const float *R, int N,
                 float *oc, float *ol, float *orr, float *ols, float *ors) {
    float *dst[UPMIX_NUM_GEN] = { ol, orr, oc, ols, ors };
    UpmixCoeffs ca, cb;
    set_cfg(&ca, p, cm_a, sm_a, 48000.0f);
    set_cfg(&cb, p, cm_b, sm_b, 48000.0f);
    upmix_reset_state();
    for (int s = 0; s < N; s += blk) {
        int n = N - s < blk ? N - s : blk;
        const UpmixCoeffs *c = s < cut ? &ca : &cb;
        upmix_begin(c, (uint32_t)n);
        for (int o = 0; o < n; o += chunk) {
            int k = n - o < chunk ? n - o : chunk;
            float *const out[UPMIX_NUM_GEN] = { dst[0] + s + o, dst[1] + s + o, dst[2] + s + o,
                                                dst[3] + s + o, dst[4] + s + o };
            upmix_render(c, L + s + o, R + s + o, (uint32_t)k, out);
        }
    }
}
#endif

#ifdef UPMIX_NUM_PROFILES
#include <stdio.h>
static int sel_fail;
static void expect(int ok, const char *what) {
    if (!ok) { sel_fail++; printf("  SELECT FAIL: %s\n", what); }
}
static int sel_of(uint8_t n) {
    const UpmixCoeffs *c = upmix_select(n);
    return c ? c->profile : -1;
}
/* The selected profile runs: its sources are present at this count. */
static int runs(uint8_t n) {
    const UpmixCoeffs *c = upmix_select(n);
    return c && c->src_a < n && c->src_b < n;
}
static int reason_of(uint8_t n) {
    UpmixStatus st;
#ifdef UPMIX_TRIM_MIN_Q1
    upmix_get_status(&st, n, 0x1FF);   /* routing builds also take the output enables */
#else
    upmix_get_status(&st, n);
#endif
    return st.parked_reason;
}

/* Profile selection, source gating and park reasons; returns the failure count. */
int h_select_check(void) {
    sel_fail = 0;
    upmix_reset_state();
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) upmix_profile_defaults(&upmix_profiles[p]);
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == -1 && reason_of(2) == 1, "all disabled: no profile, reason 1");

    upmix_profiles[0].enabled = true;
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == 0 && runs(2), "defaults: profile 0 on stereo");
    expect(sel_of(6) == -1 && reason_of(6) == 2, "defaults: nothing on 6 ch, reason 2");
    expect(sel_of(3) == -1 && sel_of(0) == -1, "odd or zero counts select nothing");

    upmix_profiles[1].enabled = true;
    upmix_profiles[1].src_a = 4; upmix_profiles[1].src_b = 5;
    upmix_profiles[1].active_counts = 0x0C;
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == 0 && sel_of(6) == 1 && sel_of(8) == 1 && sel_of(4) == -1, "disjoint counts");
    expect(runs(6) && runs(8), "profile 1 runs above stereo on inputs 4 and 5");

    upmix_profiles[1].active_counts = 0x0D;
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == 0, "overlap: lowest profile wins");
    upmix_profiles[0].enabled = false;
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == 1, "overlap with profile 0 off: profile 1");
    expect(!runs(2) && reason_of(2) == 4, "profile 1 sources absent on stereo: reason 4");

    upmix_profiles[0].enabled = true;
    upmix_profiles[0].src_a = 7; upmix_profiles[0].src_b = 9;
    upmix_apply_profiles(48000.0f);
    const UpmixCoeffs *c0 = upmix_select(2);
    expect(c0 && c0->profile == 0 && c0->src_a == 7 && c0->src_b == UPMIX_DEFAULT_SRC_B,
           "source above the channel count clamps to default");
    expect(!runs(2) && reason_of(2) == 4, "profile 0 source 7 absent on stereo: reason 4");

    upmix_apply_profiles(96000.0f);
    expect(sel_of(2) == -1 && reason_of(2) == 3, "above 48 kHz: nothing, reason 3");
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) upmix_profile_defaults(&upmix_profiles[p]);
    upmix_apply_profiles(48000.0f);
    return sel_fail;
}

/* Run profile 0 for `cut` samples then profile 1 to the end; outputs from cut on
 * must equal profile 1 run alone from reset over the same input. */
int h_profile_switch_check(const float *p0, const float *p1, int blk, const float *L, const float *R, int N, int cut) {
    float *o1[UPMIX_NUM_GEN], *o2[UPMIX_NUM_GEN];
    for (int g = 0; g < UPMIX_NUM_GEN; g++) {
        o1[g] = calloc((size_t)N, sizeof(float)); o2[g] = calloc((size_t)N, sizeof(float));
    }
    UpmixCoeffs ca, cb;
    set_cfg(&ca, p0, 1, 2, 48000.0f);
    set_cfg(&cb, p1, 0, 1, 48000.0f);
    ca.profile = 0; cb.profile = 1;
    for (int pass = 0; pass < 2; pass++) {
        upmix_reset_state();
        float **o = pass ? o2 : o1;
        for (int s = pass ? cut : 0; s < N; s += blk) {
            int n = N - s < blk ? N - s : blk;
            const UpmixCoeffs *c = s < cut ? &ca : &cb;
            float *const out[UPMIX_NUM_GEN] = { o[0] + s, o[1] + s, o[2] + s, o[3] + s, o[4] + s };
            upmix_begin(c, (uint32_t)n);
            upmix_render(c, L + s, R + s, (uint32_t)n, out);
        }
    }
    int bad = 0;
    for (int g = 0; g < UPMIX_NUM_GEN; g++) {
        if (memcmp(o1[g] + cut, o2[g] + cut, (size_t)(N - cut) * sizeof(float))) bad++;
        free(o1[g]); free(o2[g]);
    }
    return bad;
}
#endif

#ifdef UPMIX_NEW
/* Stem-through: control runs on the mix, each stem reuses the mix's control state
 * per block and keeps only its own audio-path state (filters, rings, bell). */
typedef struct {
    float pres_ic1, pres_ic2;
    float shp_ic1[2], shp_ic2[2], slp_ic1[2], slp_ic2[2];
    float haas[2][UPMIX_HAAS_RING], ap[2][UPMIX_AP_RING];
} AudioState;

static void save_audio(AudioState *a) {
    a->pres_ic1 = um.pres_ic1; a->pres_ic2 = um.pres_ic2;
    memcpy(a->shp_ic1, um.s.shp_ic1, sizeof(a->shp_ic1)); memcpy(a->shp_ic2, um.s.shp_ic2, sizeof(a->shp_ic2));
    memcpy(a->slp_ic1, um.s.slp_ic1, sizeof(a->slp_ic1)); memcpy(a->slp_ic2, um.s.slp_ic2, sizeof(a->slp_ic2));
    memcpy(a->haas, um.s.haas, sizeof(a->haas)); memcpy(a->ap, um.s.ap, sizeof(a->ap));
}
static void load_audio(const AudioState *a) {
    um.pres_ic1 = a->pres_ic1; um.pres_ic2 = a->pres_ic2;
    memcpy(um.s.shp_ic1, a->shp_ic1, sizeof(a->shp_ic1)); memcpy(um.s.shp_ic2, a->shp_ic2, sizeof(a->shp_ic2));
    memcpy(um.s.slp_ic1, a->slp_ic1, sizeof(a->slp_ic1)); memcpy(um.s.slp_ic2, a->slp_ic2, sizeof(a->slp_ic2));
    memcpy(um.s.haas, a->haas, sizeof(a->haas)); memcpy(um.s.ap, a->ap, sizeof(a->ap));
}

static UpmixState pre, post;

/* stems: S x N planar L and R; outputs S x N Ls/Rs; trace: per block p_gls, p_grs, p_wl, p_wr. */
void h_run_stems(const float *p, int cmode, int smode, int blk, const float *L, const float *R, int N,
                 int S, const float *sl, const float *sr, float *sls, float *srs,
                 float *ols, float *ors, float *trace) {
    UpmixCoeffs c;
    set_cfg(&c, p, cmode, smode, 48000.0f);
    upmix_reset_state();
    AudioState *st = calloc((size_t)S, sizeof(AudioState));
    float *bl = malloc(blk * sizeof(float)), *br = malloc(blk * sizeof(float)), *bc = malloc(blk * sizeof(float));
    for (int s = 0, b = 0; s < N; s += blk, b++) {
        int n = N - s < blk ? N - s : blk;
        pre = um;
        memcpy(bl, L + s, n * sizeof(float)); memcpy(br, R + s, n * sizeof(float));
        h_process_block(&c, bl, br, bc, ols + s, ors + s, (uint32_t)n);
        trace[4 * b] = um.s.p_gls; trace[4 * b + 1] = um.s.p_grs;
        trace[4 * b + 2] = um.s.p_wl; trace[4 * b + 3] = um.s.p_wr;
        post = um;
        for (int k = 0; k < S; k++) {
            um = pre;
            if (s > 0) load_audio(&st[k]);
            memcpy(bl, sl + (size_t)k * N + s, n * sizeof(float));
            memcpy(br, sr + (size_t)k * N + s, n * sizeof(float));
            h_process_block(&c, bl, br, bc, sls + (size_t)k * N + s, srs + (size_t)k * N + s, (uint32_t)n);
            save_audio(&st[k]);
        }
        um = post;
    }
    free(st); free(bl); free(br); free(bc);
}
#endif

#ifdef UPMIX_TRIM_MIN_Q1
/* ---- Routing check: a mini pipeline (claims, matrix omission, upmix_run)
 * against an independent float64 model of spec 3.2 and 6.2 to 6.4. ---- */
#define HO NUM_OUTPUT_CHANNELS
#define HG UPMIX_NUM_GEN
#define HDUR (48000 * 3)              /* samples per scene */

typedef struct {
    int n_in;
    uint32_t rate;
    uint16_t out_en;
    float mx[NUM_INPUT_CHANNELS][HO]; /* signed crosspoint gains, 0 = none */
    int post_k, post_o;               /* edit landing between PASS 4 and injection, -1 none */
    float post_g;
} HMix;

typedef struct {                      /* model lifecycle and fades */
    int held, active, n_in;
    uint32_t rate;
    UpmixConfig latch;
    float e[HG][HO], w[HO];
} HRef;

/* A scene edits config and matrix for the packet covering samples [t0, t1). */
typedef void (*HScene)(int t0, int t1, HMix *m);
#define AT(ms) (t0 <= (ms) * 48 && (ms) * 48 < t1)

/* Spec values, not the firmware's: 10 ms full-scale slew, 0.5 dB trim steps from -40 to +6 dB. */
static float ref_step;
static float ref_slew(float v, float t) {   /* fixed-rate rule in float32, as the device runs it */
    if (v < t) { v += ref_step; return v > t ? t : v; }
    v -= ref_step;
    return v < t ? t : v;
}
static float ref_gain(const UpmixRoute *r) {
    int t = r->trim_q1 < -80 ? -80 : (r->trim_q1 > 12 ? 12 : r->trim_q1);
    float g = (float)pow(10.0, t / 40.0);
    return (r->flags & 1) ? -g : g;
}
static int ref_src(int s, int d) { return s < NUM_INPUT_CHANNELS ? s : d; }
static uint16_t ref_claim_mask(const UpmixConfig *c) {
    uint16_t m = 0;
    for (int g = 0; g < HG; g++) m |= c->routes[g].out_mask & ((1u << HO) - 1u);
    return m;
}
static UpmixConfig ref_cfg(int p) { UpmixConfig c; memcpy(&c, (const void *)&upmix_profiles[p], sizeof(c)); return c; }
static int ref_select(int n_in, uint32_t rate) {
    if (rate > 48000) return -1;
    int bit = (n_in >= 2 && n_in <= 8 && !(n_in & 1)) ? 1 << (n_in / 2 - 1) : 0;
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) {
        UpmixConfig c = ref_cfg(p);
        if (!c.enabled || !(c.active_counts & 0x0F & bit)) continue;
        return (ref_src(c.src_a, 0) < n_in && ref_src(c.src_b, 1) < n_in) ? p : -1;
    }
    return -1;
}

/* One packet of the model; returns the claimed mask. gen[] and in[] are offset to the packet. */
static uint16_t ref_packet(HRef *r, const HMix *m, int n, const float *const *in,
                           const float *const *gen, double *const *out) {
    int sel = ref_select(m->n_in, m->rate);
    if (r->held >= 0 && (m->n_in != r->n_in || m->rate != r->rate)) {
        memset(r->e, 0, sizeof(r->e)); memset(r->w, 0, sizeof(r->w)); r->held = -1;
    }
    r->n_in = m->n_in; r->rate = m->rate;
    if (r->held >= 0) {
        UpmixConfig c = ref_cfg(r->held);
        r->active = sel == r->held && ref_src(c.src_a, 0) == ref_src(r->latch.src_a, 0)
                    && ref_src(c.src_b, 1) == ref_src(r->latch.src_b, 1);
        if (r->active) r->latch = c;
    }
    uint16_t claimed = 0;
    int running = 0;
    for (int pass = 0; pass < 2 && !running; pass++) {
        if (r->held < 0) {
            if (sel < 0) break;
            UpmixConfig c = ref_cfg(sel);
            if (!(ref_claim_mask(&c) & m->out_en)) break;
            r->held = sel; r->latch = c; r->active = 1;
        }
        uint16_t cm = ref_claim_mask(&r->latch);
        claimed = 0;
        for (int o = 0; o < HO; o++) {
            if (!(m->out_en & (1u << o))) {
                r->w[o] = 0; for (int g = 0; g < HG; g++) r->e[g][o] = 0;
                continue;
            }
            int live = r->w[o] != 0 || (r->active && (cm & (1u << o)));
            for (int g = 0; g < HG; g++) live |= r->e[g][o] != 0;
            if (live) claimed |= (uint16_t)(1u << o);
        }
        if (claimed || (r->active && (cm & m->out_en))) running = 1;
        else r->held = -1;
    }
    if (!running) claimed = 0;
    int sa = ref_src(r->latch.src_a, 0), sb = ref_src(r->latch.src_b, 1);
    int sur = r->latch.surround_mode != UPMIX_SURROUND_OFF;
    uint16_t cm = ref_claim_mask(&r->latch);
    /* BA/BB carry audio while routed to an enabled output or while either still fades. */
    int back = r->active && ((r->latch.routes[5].out_mask | r->latch.routes[6].out_mask) & m->out_en);
    for (int o = 0; o < HO; o++) back |= r->e[5][o] != 0 || r->e[6][o] != 0;
    back = back && sur;
    for (int o = 0; o < HO; o++) {
        uint16_t bit = (uint16_t)(1u << o);
        int cl = claimed & bit;
        for (int i = 0; i < n; i++) {
            double acc = 0, side = 0;
            if (m->out_en & bit) for (int k = 0; k < m->n_in; k++) {
                if (m->mx[k][o] == 0) continue;
                if (cl && (k == sa || k == sb)) side += (double)m->mx[k][o] * in[k][i];
                else acc += (double)m->mx[k][o] * in[k][i];
            }
            if (cl) {
                for (int g = 0; g < HG; g++) {
                    float t = (r->active && (r->latch.routes[g].out_mask & bit)) ? ref_gain(&r->latch.routes[g]) : 0.0f;
                    r->e[g][o] = ref_slew(r->e[g][o], t);
                    int live = g <= 2 || (sur && g <= 4) || back;   /* A', B', C; sides; backs */
                    if (live) acc += (double)r->e[g][o] * gen[g][i];
                }
                r->w[o] = ref_slew(r->w[o], (r->active && (cm & bit)) ? 1.0f : 0.0f);
                acc += (1.0 - (double)r->w[o]) * side;
            }
            out[o][i] = acc;
        }
    }
    return claimed;
}

/* Device path for one packet: claim, PASS 4 with omission, a late crosspoint edit, upmix_run. */
static float h_out[HO][AUDIO_BUFFER_SAMPLES];
static uint16_t fw_packet(HMix *m, int n, float *const *in, int *ran) {
    UpmixClaim cl;
    const UpmixCoeffs *c = upmix_claim((uint8_t)m->n_in, m->rate, m->out_en, &cl);
    for (int o = 0; o < HO; o++) {
        memset(h_out[o], 0, sizeof(h_out[o]));
        if (!(m->out_en & (1u << o))) continue;
        int claimed = c && (cl.claimed & (1u << o));
        for (int k = 0; k < m->n_in; k++) {
            float g = m->mx[k][o];
            if (g == 0.0f) continue;
            if (claimed && (k == cl.src_a || k == cl.src_b)) {
                uint8_t j = cl.side_n[o]++;
                cl.side_src[o][j] = in[k]; cl.side_gain[o][j] = g;
                continue;
            }
            for (int i = 0; i < n; i++) h_out[o][i] += g * in[k][i];
        }
    }
    if (m->post_k >= 0) { m->mx[m->post_k][m->post_o] = m->post_g; m->post_k = -1; }
    if (c) upmix_run(c, &cl, in, h_out, (uint32_t)n);
    *ran = c != NULL;
    return c ? cl.claimed : 0;
}

static void h_route(int p, int g, uint16_t mask, int q1, int flags) {
    upmix_profiles[p].routes[g].out_mask = mask;
    upmix_profiles[p].routes[g].trim_q1 = (int8_t)q1;
    upmix_profiles[p].routes[g].flags = (uint8_t)flags;
}

/* Stereo to 5.1 plus a phantom-centre output fed by both source rows, with edits
 * spaced 4 to 5 ms so they land mid-fade at every packet size. */
static void scene_51(int t0, int t1, HMix *m) {
    if (AT(0)) {
        m->n_in = 2; m->out_en = 0x7F;
        m->mx[0][0] = 1.0f; m->mx[1][1] = 1.0f; m->mx[0][3] = 0.5f; m->mx[1][3] = 0.5f;
        m->mx[0][6] = 0.5f; m->mx[1][6] = 0.5f;
        upmix_profiles[0].enabled = true;
        h_route(0, UPMIX_GEN_A, 1u << 0, 0, 0);
        h_route(0, UPMIX_GEN_B, 1u << 1, 0, 0);
        h_route(0, UPMIX_GEN_C, (1u << 2) | (1u << 6), -12, 0);
        h_route(0, UPMIX_GEN_SA, 1u << 4, 0, 0);
        h_route(0, UPMIX_GEN_SB, 1u << 5, 6, UPMIX_ROUTE_INVERT);
    }
    if (AT(600)) h_route(0, UPMIX_GEN_C, (1u << 2) | (1u << 6), 4, 0);
    if (AT(604)) h_route(0, UPMIX_GEN_C, (1u << 2) | (1u << 6), -40, 0);   /* second edit mid-fade */
    if (AT(900)) { h_route(0, UPMIX_GEN_SA, 1u << 5, 0, 0); h_route(0, UPMIX_GEN_SB, 1u << 4, 0, 0); }
    if (AT(905)) h_route(0, UPMIX_GEN_SA, (1u << 5) | (1u << 0), -6, 0);
    if (AT(1100)) upmix_profiles[0].surround_mode = UPMIX_SURROUND_PASSIVE;
    if (AT(1300)) { m->post_k = 1; m->post_o = 6; m->post_g = 0.25f; }    /* after the snapshot */
    if (AT(1500)) upmix_profiles[0].enabled = false;
    if (AT(1504)) m->mx[0][0] = 0.5f;                                     /* during the fade-out */
    if (AT(1800)) upmix_profiles[0].enabled = true;
    if (AT(2000)) m->out_en &= (uint16_t)~(1u << 1);
    if (AT(2100)) m->out_en |= 1u << 1;
    if (AT(2300)) h_route(0, UPMIX_GEN_A, 0, 0, 0);                       /* last bit leaves output 0 */
    if (AT(2400)) h_route(0, UPMIX_GEN_C, (1u << 2) | (1u << 6), 12, 0);
    if (AT(2700)) h_route(0, UPMIX_GEN_C, 0, 12, 0);                      /* +6 dB fade outlasts w */
}

/* One source row for both sides: only that row is omitted. */
static void scene_mono(int t0, int t1, HMix *m) {
    if (AT(0)) {
        m->n_in = 2; m->out_en = 0x07;
        m->mx[0][0] = 1.0f; m->mx[0][1] = 0.5f; m->mx[1][1] = 0.5f; m->mx[1][2] = 1.0f;
        upmix_profiles[0].enabled = true;
        upmix_profiles[0].src_a = 0; upmix_profiles[0].src_b = 0;
        h_route(0, UPMIX_GEN_A, 1u << 0, 0, 0);
        h_route(0, UPMIX_GEN_C, (1u << 1) | (1u << 2), -6, 0);
    }
    if (AT(1500)) upmix_profiles[0].enabled = false;
}

/* 6 channels, two profiles: live changes and a source edit are sequential fades;
 * count and rate changes park hard. */
static void scene_profiles(int t0, int t1, HMix *m) {
    if (AT(0)) {
        m->n_in = 6; m->out_en = 0xFF;
        for (int k = 0; k < 6; k++) m->mx[k][k] = 1.0f;
        upmix_profiles[0].enabled = true; upmix_profiles[0].active_counts = 0x04;
        upmix_profiles[0].src_a = 4; upmix_profiles[0].src_b = 5;
        h_route(0, UPMIX_GEN_A, 1u << 4, 0, 0);
        h_route(0, UPMIX_GEN_B, 1u << 5, 0, 0);
        h_route(0, UPMIX_GEN_SA, 1u << 6, 0, 0);
        h_route(0, UPMIX_GEN_SB, 1u << 7, 0, 0);
        upmix_profiles[1].enabled = true; upmix_profiles[1].active_counts = 0x04;
        h_route(1, UPMIX_GEN_A, 1u << 0, 0, 0);
        h_route(1, UPMIX_GEN_B, 1u << 1, 0, 0);
        h_route(1, UPMIX_GEN_C, 1u << 2, -6, 0);   /* row 2 stays on output 2 */
    }
    if (AT(800)) upmix_profiles[0].enabled = false;
    if (AT(1300)) upmix_profiles[0].enabled = true;
    if (AT(1600)) upmix_profiles[0].src_a = 2;      /* source edit on the held profile */
    if (AT(2000)) m->n_in = 2;
    if (AT(2100)) upmix_profiles[1].active_counts = 0x05;
    if (AT(2400)) m->rate = 96000;
    if (AT(2600)) m->rate = 48000;
}

/* An enabled profile with nothing routed parks, and starts with its first route. */
static void scene_noroute(int t0, int t1, HMix *m) {
    if (AT(0)) {
        m->n_in = 2; m->out_en = 0x03;
        m->mx[0][0] = 1.0f; m->mx[1][1] = 1.0f;
        upmix_profiles[0].enabled = true;
    }
    if (AT(500)) h_route(0, UPMIX_GEN_A, 1u << 0, 0, 0);
    if (AT(1000)) h_route(0, UPMIX_GEN_A, 0, 0, 0);
    if (AT(1500)) h_route(0, UPMIX_GEN_C, 1u << 1, -6, 0);
}

/* Stereo to 7.1: the back pair starts, fades out after its routes go (still
 * rendering), restarts, and follows a surround mode switch. */
static void scene_71(int t0, int t1, HMix *m) {
    if (AT(0)) {
        m->n_in = 2; m->out_en = 0xFF;
        m->mx[0][0] = 1.0f; m->mx[1][1] = 1.0f;
        upmix_profiles[0].enabled = true;
        h_route(0, UPMIX_GEN_A, 1u << 0, 0, 0);
        h_route(0, UPMIX_GEN_B, 1u << 1, 0, 0);
        h_route(0, UPMIX_GEN_C, 1u << 2, -6, 0);
        h_route(0, UPMIX_GEN_SA, 1u << 4, 0, 0);
        h_route(0, UPMIX_GEN_SB, 1u << 5, 0, 0);
        h_route(0, UPMIX_GEN_BA, 1u << 6, 0, 0);
        h_route(0, UPMIX_GEN_BB, 1u << 7, 0, 0);
    }
    if (AT(1000)) h_route(0, UPMIX_GEN_BA, 0, 0, 0);
    if (AT(1004)) h_route(0, UPMIX_GEN_BB, 1u << 7, 12, 0);
    if (AT(1300)) h_route(0, UPMIX_GEN_BB, 0, 0, 0);       /* +6 dB fade-out with no back route left */
    if (AT(1800)) { h_route(0, UPMIX_GEN_BA, 1u << 6, 0, 0); h_route(0, UPMIX_GEN_BB, 1u << 7, 0, 0); }
    if (AT(2300)) upmix_profiles[0].surround_mode = UPMIX_SURROUND_PASSIVE;
#ifdef UPMIX_BACK_AP_RING   /* builds of the engine before the back pair lack the field */
    if (AT(2600)) upmix_profiles[0].back_offset_ms = 9.0f;
#endif
}

static const HScene scenes[] = { scene_51, scene_mono, scene_profiles, scene_noroute, scene_71 };

/* Runs scene sc at packet size blk over src (8 x HDUR). Returns claim and lifecycle
 * disagreements; *err is the worst |device - model|, *exact counts settled unity
 * samples that were not bit-exact, *sbad counts status fields that disagree. */
int h_route_check(int sc, int blk, const float *src, double *err, int *exact, int *sbad) {
    float *in[NUM_INPUT_CHANNELS];
    for (int k = 0; k < NUM_INPUT_CHANNELS; k++) in[k] = (float *)src + (size_t)k * HDUR;
    for (int g = 0; g < HG; g++) h_tap[g] = calloc((size_t)HDUR, sizeof(float));
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) upmix_profile_defaults(&upmix_profiles[p]);
    upmix_apply_profiles(48000.0f);
    upmix_hard_park();
    HMix m; memset(&m, 0, sizeof(m)); m.rate = 48000; m.post_k = -1;
    HRef r; memset(&r, 0, sizeof(r)); r.held = -1;
    double ref[HO][AUDIO_BUFFER_SAMPLES];
    double *refp[HO];
    for (int o = 0; o < HO; o++) refp[o] = ref[o];
    int bad = 0, unity_prev[HO] = { 0 };
    *err = 0; *exact = 0; *sbad = 0;
    for (int s = 0; s < HDUR; s += blk) {
        int n = HDUR - s < blk ? HDUR - s : blk;
        scenes[sc](s, s + n, &m);
        upmix_apply_profiles((float)m.rate);
        ref_step = (float)(1.0 / (0.010 * m.rate));
        float *pin[NUM_INPUT_CHANNELS];
        const float *cin[NUM_INPUT_CHANNELS];
        for (int k = 0; k < NUM_INPUT_CHANNELS; k++) { pin[k] = in[k] + s; cin[k] = pin[k]; }
        h_tap_base = s;
        const float *gp[HG];
        for (int g = 0; g < HG; g++) gp[g] = h_tap[g] + s;
        int ran;
        HMix pre = m;                                          /* the matrix as PASS 4 saw it */
        uint16_t fc = fw_packet(&m, n, pin, &ran);
        uint16_t rc = ref_packet(&r, &pre, n, cin, gp, refp);
        if (fc != rc || ur.held != (r.held >= 0) || ran != (r.held >= 0)) bad++;

        UpmixStatus st;
        upmix_get_status(&st, (uint8_t)m.n_in, m.out_en);
        if (r.held >= 0) {
            if (st.active != r.active || st.parked_reason != (r.active ? 0 : 5) || st.active_profile != r.held)
                (*sbad)++;
            float pk = 0;
            for (int i = 0; i < n; i++) pk = fmaxf(pk, fabsf(gp[UPMIX_GEN_A][i]));
            if (st.gen_peak[UPMIX_GEN_A] != (uint16_t)(fminf(1.0f, pk) * 32767.0f)) (*sbad)++;
        } else {
            int sel = ref_select(m.n_in, m.rate);
            UpmixConfig c = sel >= 0 ? ref_cfg(sel) : (UpmixConfig){ 0 };
            if (st.active || st.active_profile != 0xFF) (*sbad)++;
            if (sel >= 0 && !(ref_claim_mask(&c) & m.out_en) && st.parked_reason != 6) (*sbad)++;
        }

        for (int o = 0; o < HO; o++) {
            int unity = rc & (1u << o);
            for (int g = 0; g < HG && unity; g++) {
                int on = r.active && (r.latch.routes[g].out_mask & (1u << o));
                if (r.e[g][o] != (on ? ref_gain(&r.latch.routes[g]) : 0.0f)) unity = 0;
            }
            for (int i = 0; i < n; i++) {
                double d = fabs((double)h_out[o][i] - ref[o][i]);
                if (d > *err) *err = d;
            }
            /* Settled across the whole packet, single unity claim, no other rows:
             * the output is the generated channel bit for bit. */
            int g1 = -1, cnt = 0, rows = 0;
            if (unity && r.w[o] == 1.0f) {
                for (int g = 0; g < HG; g++)
                    if (r.latch.routes[g].out_mask & (1u << o)) { g1 = g; cnt++; }
                int sa = ref_src(r.latch.src_a, 0), sb = ref_src(r.latch.src_b, 1);
                for (int k = 0; k < m.n_in; k++) if (m.mx[k][o] != 0 && k != sa && k != sb) rows++;
            }
            int unity_now = cnt == 1 && rows == 0 && r.latch.routes[g1].trim_q1 == 0 && !r.latch.routes[g1].flags;
            if (unity_now && unity_prev[o] == g1 + 1)
                for (int i = 0; i < n; i++)
                    if (memcmp(&h_out[o][i], &gp[g1][i], sizeof(float))) (*exact)++;
            unity_prev[o] = unity_now ? g1 + 1 : 0;
        }
    }
    for (int g = 0; g < HG; g++) { free(h_tap[g]); h_tap[g] = NULL; }
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) upmix_profile_defaults(&upmix_profiles[p]);
    upmix_apply_profiles(48000.0f);
    upmix_hard_park();
    return bad;
}

int h_route_scenes(void) { return (int)(sizeof(scenes) / sizeof(scenes[0])); }
#endif

#ifdef UPMIX_BACK_AP_RING
/* Engine run with the back pair rendered: out is 7 x N planar (UPMIX_GEN_* order).
 * Returns the back Haas tap in samples. */
int h_run_back(const float *p, int cmode, int smode, float back_ms, int blk, int chunk,
               const float *L, const float *R, int N, float *out) {
    UpmixConfig cfg = {
        .enabled = true, .center_mode = (uint8_t)cmode, .surround_mode = (uint8_t)smode,
        .strength_pct = p[0], .center_width_pct = p[1], .corr_threshold_pct = p[2],
        .attack_ms = p[3], .release_ms = p[4], .detector_hpf_hz = p[5],
        .surround_delay_ms = p[6], .surround_hpf_hz = p[7], .surround_lpf_hz = p[8],
        .decorr_pct = p[9], .presence_db = p[10], .back_offset_ms = back_ms,
    };
    cfg.routes[UPMIX_GEN_BA].out_mask = 1;
    cfg.routes[UPMIX_GEN_BB].out_mask = 2;
    UpmixCoeffs c;
    upmix_compute_coefficients(&c, &cfg, 48000.0f);
    upmix_reset_state();
    for (int s = 0; s < N; s += blk) {
        int n = N - s < blk ? N - s : blk;
        upmix_begin(&c, (uint32_t)n);
        for (int o = 0; o < n; o += chunk) {
            int k = n - o < chunk ? n - o : chunk;
            float *const dst[UPMIX_NUM_GEN] = {
                out + s + o, out + (size_t)N + s + o, out + 2 * (size_t)N + s + o,
                out + 3 * (size_t)N + s + o, out + 4 * (size_t)N + s + o,
                out + 5 * (size_t)N + s + o, out + 6 * (size_t)N + s + o };
            upmix_render(&c, L + s + o, R + s + o, (uint32_t)k, dst);
        }
    }
    return (int)c.back_delay;
}
#endif
