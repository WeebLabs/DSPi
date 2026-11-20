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
