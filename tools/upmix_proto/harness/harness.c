/* Host harness: compiles the real upmix.c (copied next to the stubs by
 * harness.py) into a shared library driven from Python through ctypes. */
#include <stdlib.h>
#include "upmix.c"

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
        upmix_process_block(&c, ol + s, orr + s, oc + s, ols + s, ors + s, (uint32_t)n);
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
        upmix_process_block(s < cut ? &ca : &cb, ol + s, orr + s, oc + s, ols + s, ors + s, (uint32_t)n);
    }
}

/* Reference run with both modes switchable at sample `cut`, through upmix_process_block. */
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
        upmix_process_block(s < cut ? &ca : &cb, ol + s, orr + s, oc + s, ols + s, ors + s, (uint32_t)n);
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
static int reason_of(uint8_t n) {
    UpmixStatus st;
    upmix_get_status(&st, n);
    return st.parked_reason;
}

/* Profile selection, bridge gating and park reasons; returns the failure count. */
int h_select_check(void) {
    sel_fail = 0;
    upmix_reset_state();
    for (int p = 0; p < UPMIX_NUM_PROFILES; p++) upmix_profile_defaults(&upmix_profiles[p]);
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == -1 && reason_of(2) == 1, "all disabled: no profile, reason 1");

    upmix_profiles[0].enabled = true;
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == 0 && upmix_bridge_coeffs(2) != NULL, "defaults: profile 0 on stereo");
    expect(sel_of(6) == -1 && reason_of(6) == 2, "defaults: nothing on 6 ch, reason 2");
    expect(sel_of(3) == -1 && sel_of(0) == -1, "odd or zero counts select nothing");

    upmix_profiles[1].enabled = true;
    upmix_profiles[1].src_a = 4; upmix_profiles[1].src_b = 5;
    upmix_profiles[1].active_counts = 0x0C;
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == 0 && sel_of(6) == 1 && sel_of(8) == 1 && sel_of(4) == -1, "disjoint counts");
    expect(upmix_bridge_coeffs(6) == NULL, "bridge never runs above stereo");

    upmix_profiles[1].active_counts = 0x0D;
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == 0, "overlap: lowest profile wins");
    upmix_profiles[0].enabled = false;
    upmix_apply_profiles(48000.0f);
    expect(sel_of(2) == 1, "overlap with profile 0 off: profile 1");
    expect(upmix_bridge_coeffs(2) == NULL && reason_of(2) == 4, "profile 1 sources absent on stereo: reason 4");

    upmix_profiles[0].enabled = true;
    upmix_profiles[0].src_a = 7; upmix_profiles[0].src_b = 9;
    upmix_apply_profiles(48000.0f);
    const UpmixCoeffs *c0 = upmix_select(2);
    expect(c0 && c0->profile == 0 && c0->src_a == 7 && c0->src_b == UPMIX_DEFAULT_SRC_B,
           "source above the channel count clamps to default");
    expect(upmix_bridge_coeffs(2) == NULL && reason_of(2) == 4, "profile 0 source 7 absent on stereo: reason 4");

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
        upmix_process_block(&c, bl, br, bc, ols + s, ors + s, (uint32_t)n);
        trace[4 * b] = um.s.p_gls; trace[4 * b + 1] = um.s.p_grs;
        trace[4 * b + 2] = um.s.p_wl; trace[4 * b + 3] = um.s.p_wr;
        post = um;
        for (int k = 0; k < S; k++) {
            um = pre;
            if (s > 0) load_audio(&st[k]);
            memcpy(bl, sl + (size_t)k * N + s, n * sizeof(float));
            memcpy(br, sr + (size_t)k * N + s, n * sizeof(float));
            upmix_process_block(&c, bl, br, bc, sls + (size_t)k * N + s, srs + (size_t)k * N + s, (uint32_t)n);
            save_audio(&st[k]);
        }
        um = post;
    }
    free(st); free(bl); free(br); free(bc);
}
#endif
