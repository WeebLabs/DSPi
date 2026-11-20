"""Float32 models of the DSPi stereo upmixer (firmware/DSPi/upmix.c).

baseline_run() is a line-for-line port of upmix_process_block() for diffing
against the C. engine() is the switchable proposed engine; with every item off
it reproduces baseline_run() (checked in run_eval.py).
"""
import math
import numpy as np
from numba import njit

f32 = np.float32
FS = 48000.0
BLOCK = 48                 # 1 ms USB packet at 48 kHz
HAAS_RING = 1024
AP_RING = 512

# Constants are float32 globals so numba keeps arithmetic in single precision.
ZERO = f32(0.0); ONE = f32(1.0); HALF = f32(0.5); TWO = f32(2.0)
C7071 = f32(0.7071); TINY = f32(1e-12); CLIPG = f32(1024.0)
DOM_LR_NEUTRAL = f32(0.75)   # pan-law value at dom_lr = 0 (dom mode 2 keeps only the F/B axis)

# Indices into the coefficient array K (UpmixCoeffs, all as float32).
(K_STRENGTH, K_WIDTH, K_THRESH, K_INVTHR, K_AATT, K_AREL, K_ACORR, K_ADOM,
 K_DETHP, K_PA1, K_PA2, K_PA3, K_PM1, K_LSCL, K_LSCR, K_APG, K_HDLY, K_ADLY,
 K_SHA1, K_SHA2, K_SHA3, K_SHK, K_SLA1, K_SLA2, K_SLA3, K_ENVSTEP,
 K_CMODE, K_N) = range(28)

DEFAULTS = dict(center_mode=1, strength=100.0, width=25.0, thresh=30.0,
                attack=10.0, release=100.0, det_hpf=200.0, sur_delay=12.0,
                sur_hpf=300.0, sur_lpf=7000.0, decorr=90.0, presence=0.0)


def svf_coeffs(fc, fs=FS, k=1.4142135624):
    g = math.tan(math.pi * fc / fs)
    a1 = 1.0 / (1.0 + g * (g + k))
    a2 = g * a1
    a3 = g * a2
    return f32(a1), f32(a2), f32(a3)


def compute_coeffs(fs=FS, **over):
    """upmix_compute_coefficients() for adaptive surround; expf/tanf done in double then rounded."""
    c = dict(DEFAULTS, **over)
    K = np.zeros(K_N, dtype=np.float32)
    cm = c['center_mode']
    K[K_CMODE] = cm
    K[K_STRENGTH] = 0.0 if cm == 2 else f32(c['strength']) * f32(0.01)
    K[K_WIDTH] = f32(c['width']) * f32(0.01)
    K[K_THRESH] = f32(c['thresh']) * f32(0.01)
    K[K_INVTHR] = f32(1.0) / (f32(1.0) - K[K_THRESH])
    K[K_AATT] = math.exp(-1.0 / (fs * c['attack'] * 0.001))
    K[K_AREL] = math.exp(-1.0 / (fs * c['release'] * 0.001))
    K[K_ACORR] = math.exp(-1.0 / (fs * 100.0 * 0.001))
    K[K_ADOM] = math.exp(-1.0 / (fs * 40.0 * 0.001))
    K[K_DETHP] = 1.0 - math.exp(-2.0 * math.pi * c['det_hpf'] / fs)
    A = 10.0 ** (c['presence'] / 40.0)
    g = math.tan(math.pi * 3000.0 / fs)
    k = 1.0 / (0.6 * A)
    a1 = 1.0 / (1.0 + g * (g + k))
    K[K_PA1] = a1; K[K_PA2] = g * a1; K[K_PA3] = g * g * a1; K[K_PM1] = k * (A * A - 1.0)
    K[K_LSCL] = 0.8710; K[K_LSCR] = -0.4898
    K[K_APG] = 0.5 * c['decorr'] * 0.01
    K[K_HDLY] = min(int(c['sur_delay'] * 0.001 * fs + 0.5), HAAS_RING - 1)
    K[K_ADLY] = min(int(10.0 * 0.001 * fs + 0.5), AP_RING - 1)
    K[K_SHA1], K[K_SHA2], K[K_SHA3] = svf_coeffs(c['sur_hpf'], fs)
    K[K_SHK] = 1.4142135624
    K[K_SLA1], K[K_SLA2], K[K_SLA3] = svf_coeffs(min(c['sur_lpf'], 0.45 * fs), fs)
    K[K_ENVSTEP] = 1.0 / (fs * 10.0 * 0.001)
    return K


@njit(cache=True, inline='always')
def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


@njit(cache=True)
def baseline_run(l_in, r_in, K):
    """Port of upmix_process_block() (adaptive surround), run in 48-sample blocks from zeroed state.

    Returns C, L', R', Ls, Rs and the per-sample gains actually applied (gc, rem, gls, grs).
    """
    N = l_in.size
    oc = np.zeros(N, np.float32); ol = np.zeros(N, np.float32); orr = np.zeros(N, np.float32)
    ols = np.zeros(N, np.float32); ors = np.zeros(N, np.float32)
    rgc = np.zeros(N, np.float32); rrem = np.zeros(N, np.float32)
    rgls = np.zeros(N, np.float32); rgrs = np.zeros(N, np.float32)
    det_lp_l = ZERO; det_lp_r = ZERO
    acc_lr = ZERO; acc_ll = ZERO; acc_rr = ZERO; g_c = ZERO
    dom_lr = ZERO; dom_fb = ZERO
    p_gc = ZERO; p_rem = ZERO; p_gls = ZERO; p_grs = ZERO; env = ZERO
    pres_ic1 = ZERO; pres_ic2 = ZERO
    shp1 = np.zeros(2, np.float32); shp2 = np.zeros(2, np.float32)
    slp1 = np.zeros(2, np.float32); slp2 = np.zeros(2, np.float32)
    haas = np.zeros((2, HAAS_RING), np.float32); ap = np.zeros((2, AP_RING), np.float32)
    widx = 0
    hdly = int(K[K_HDLY]); adly = int(K[K_ADLY])
    adaptive = K[K_CMODE] == 1; coff = K[K_CMODE] == 2
    for s0 in range(0, N, BLOCK):
        n = min(BLOCK, N - s0)
        inv_n = ONE / f32(n)
        env_new = env + f32(n) * K[K_ENVSTEP]
        if env_new > ONE:
            env_new = ONE
        if adaptive:
            denom = np.sqrt(acc_ll * acc_rr) + TINY
            corr = clampf(acc_lr / denom, -ONE, ONE)
            bal = abs(acc_ll - acc_rr) / (acc_ll + acc_rr + TINY)
            p = (corr if corr > ZERO else ZERO) * (ONE - bal)
            m = (p - K[K_THRESH]) * K[K_INVTHR]
            g_target = K[K_STRENGTH] * clampf(m, ZERO, ONE)
        else:
            g_target = K[K_STRENGTH]
        a = K[K_AATT] if g_target > g_c else K[K_AREL]
        alpha_blk = f32(math.pow(a, n))
        g_c = g_target + (g_c - g_target) * alpha_blk
        if coff:
            g_c = ZERO
        fbb = HALF * dom_fb - HALF
        sfb = ONE - (fbb + ONE) * (fbb + ONE)
        ul = HALF * (dom_lr + ONE)
        ur = HALF * (ONE - dom_lr)
        gls_t = (ONE - ul * ul) * sfb
        grs_t = (ONE - ur * ur) * sfb
        gc_new = C7071 * g_c * env_new
        rem_new = HALF * (ONE - K[K_WIDTH]) * g_c * env_new
        gls_new = gls_t * env_new
        grs_new = grs_t * env_new
        gc_i = p_gc; gc_step = (gc_new - p_gc) * inv_n
        rem_i = p_rem; rem_step = (rem_new - p_rem) * inv_n
        gls_i = p_gls; gls_step = (gls_new - p_gls) * inv_n
        grs_i = p_grs; grs_step = (grs_new - p_grs) * inv_n
        p_gc = gc_new; p_rem = rem_new; p_gls = gls_new; p_grs = grs_new
        env = env_new
        hp_a = K[K_DETHP]; a_corr = K[K_ACORR]; omc = ONE - a_corr
        a_dom = K[K_ADOM]; omd = ONE - a_dom
        for i in range(s0, s0 + n):
            l0 = l_in[i]; r0 = r_in[i]
            det_lp_l += hp_a * (l0 - det_lp_l)
            det_lp_r += hp_a * (r0 - det_lp_r)
            el = l0 - det_lp_l; er = r0 - det_lp_r
            acc_lr = a_corr * acc_lr + omc * (el * er)
            acc_ll = a_corr * acc_ll + omc * (el * el)
            acc_rr = a_corr * acc_rr + omc * (er * er)
            dlr = clampf(CLIPG * (abs(r0) - abs(l0)), -ONE, ONE)
            f = HALF * (l0 + r0); b = HALF * (l0 - r0)
            dfb = clampf(CLIPG * (abs(f) - abs(b)), -ONE, ONE)
            dom_lr = a_dom * dom_lr + omd * dlr
            dom_fb = a_dom * dom_fb + omd * dfb
            mid = l0 + r0
            c0 = gc_i * mid
            pv3 = c0 - pres_ic2
            pv1 = K[K_PA1] * pres_ic1 + K[K_PA2] * pv3
            pv2 = pres_ic2 + K[K_PA2] * pres_ic1 + K[K_PA3] * pv3
            pres_ic1 = TWO * pv1 - pres_ic1
            pres_ic2 = TWO * pv2 - pres_ic2
            c0 += K[K_PM1] * pv1
            oc[i] = c0
            rem = rem_i * mid
            l1 = l0 - rem; r1 = r0 - rem
            ol[i] = l1; orr[i] = r1
            rgc[i] = gc_i; rrem[i] = rem_i; rgls[i] = gls_i; rgrs[i] = grs_i
            gc_i += gc_step; rem_i += rem_step
            x0 = gls_i * (K[K_LSCL] * l1 + K[K_LSCR] * r1)
            x1 = grs_i * (K[K_LSCL] * r1 + K[K_LSCR] * l1)
            gls_i += gls_step; grs_i += grs_step
            for ch in range(2):
                x = x0 if ch == 0 else x1
                v3 = x - shp2[ch]
                v1 = K[K_SHA1] * shp1[ch] + K[K_SHA2] * v3
                v2 = shp2[ch] + K[K_SHA2] * shp1[ch] + K[K_SHA3] * v3
                shp1[ch] = TWO * v1 - shp1[ch]; shp2[ch] = TWO * v2 - shp2[ch]
                x = x - K[K_SHK] * v1 - v2
                v3 = x - slp2[ch]
                v1 = K[K_SLA1] * slp1[ch] + K[K_SLA2] * v3
                v2 = slp2[ch] + K[K_SLA2] * slp1[ch] + K[K_SLA3] * v3
                slp1[ch] = TWO * v1 - slp1[ch]; slp2[ch] = TWO * v2 - slp2[ch]
                if ch == 0:
                    x0 = v2
                else:
                    x1 = v2
            haas[0, widx & (HAAS_RING - 1)] = x0
            haas[1, widx & (HAAS_RING - 1)] = x1
            x0 = haas[0, (widx - hdly) & (HAAS_RING - 1)]
            x1 = haas[1, (widx - hdly) & (HAAS_RING - 1)]
            apg = K[K_APG]
            wd0 = ap[0, (widx - adly) & (AP_RING - 1)]
            w0 = x0 + apg * wd0
            ap[0, widx & (AP_RING - 1)] = w0
            ols[i] = wd0 - apg * w0
            wd1 = ap[1, (widx - adly) & (AP_RING - 1)]
            w1 = x1 - apg * wd1
            ap[1, widx & (AP_RING - 1)] = w1
            ors[i] = wd1 + apg * w1
            widx += 1
    return oc, ol, orr, ols, ors, rgc, rrem, rgls, rgrs


# ---------------------------------------------------------------- proposed engine
# Integer flags F and float parameters P for engine().
(F_RESID, F_GATE, F_DET, F_LOOK, F_NB, F_DUCK, F_DOM, F_SIDE, F_FLOOR, F_N) = range(10)
(P_GATEP, P_WMAX, P_EPS, P_AW, P_DTHR, P_DQ, P_DAF, P_DAS, P_DREL, P_G0,
 P_X1A1, P_X1A2, P_X1A3, P_X2A1, P_X2A2, P_X2A3,
 P_BHA1, P_BHA2, P_BHA3, P_BLA1, P_BLA2, P_BLA3, P_WMIN, P_FLOORQ, P_SIDEK, P_N) = range(26)

PARAM_DEFAULTS = dict(
    gate_p=1.0,       # balance-gate exponent: gain *= (1 - bal)^p
    side_k=0.0,       # side gate: quiet side gets (1 - bal)^(p*k); 0 = dominant side only
    floor_q=1.0,      # floor relax exponent: wmin_eff = wmin * (1 - bal)^q
    wmax=4.0,         # residual weight ceiling (bounds noise gain of -w*R')
    wmin=0.5623,      # residual weight floor = PLII 0.4898/0.871; keeps the fixed-feed centre
                      # rejection while the estimator lags an onset, anti-phase still passes
    eps=1e-3,         # weight regulariser, relative to ll+rr
    tau_w=40.0,       # ms, residual / balance estimators
    duck_thr=4.0,     # fast/slow power ratio that counts as an onset (6 dB)
    duck_q=0.5,       # duck = (thr/ratio)^q above threshold (q=0.5: amplitude)
    duck_fast=2.0, duck_slow=60.0, duck_rel=60.0,   # ms
    g0=0.5625,        # fixed surround gain when dominance steering is dropped
    xover2=(1500.0,), xover3=(800.0, 2500.0),
    det_band=(300.0, 7000.0),
)


def compute_params(flags, **over):
    p = dict(PARAM_DEFAULTS, **over)
    P = np.zeros(P_N, dtype=np.float32)
    P[P_GATEP] = p['gate_p']; P[P_WMAX] = p['wmax']; P[P_EPS] = p['eps']; P[P_WMIN] = p['wmin']; P[P_FLOORQ] = p['floor_q']; P[P_SIDEK] = p['side_k']
    P[P_AW] = math.exp(-1.0 / (FS * p['tau_w'] * 1e-3))
    P[P_DTHR] = p['duck_thr']; P[P_DQ] = p['duck_q']
    P[P_DAF] = 1.0 - math.exp(-1.0 / (FS * p['duck_fast'] * 1e-3))
    P[P_DAS] = 1.0 - math.exp(-1.0 / (FS * p['duck_slow'] * 1e-3))
    P[P_DREL] = math.exp(-BLOCK / (FS * p['duck_rel'] * 1e-3))   # per-block retention
    P[P_G0] = p['g0']
    nb = flags[F_NB]
    xo = p['xover2'] if nb == 2 else p['xover3']
    if nb >= 2:
        P[P_X1A1], P[P_X1A2], P[P_X1A3] = svf_coeffs(xo[0])
    if nb == 3:
        P[P_X2A1], P[P_X2A2], P[P_X2A3] = svf_coeffs(xo[1])
    P[P_BHA1], P[P_BHA2], P[P_BHA3] = svf_coeffs(p['det_band'][0])
    P[P_BLA1], P[P_BLA2], P[P_BLA3] = svf_coeffs(p['det_band'][1])
    return P


@njit(cache=True, inline='always')
def svf(x, s1, s2, a1, a2, a3):
    """TPT SVF step; returns (v1 band, v2 low, new ic1, new ic2). HP = x - k*v1 - v2."""
    v3 = x - s2
    v1 = a1 * s1 + a2 * v3
    v2 = s2 + a2 * s1 + a3 * v3
    return v1, v2, TWO * v1 - s1, TWO * v2 - s2


@njit(cache=True)
def engine(l_in, r_in, K, F, P, replay, rgc, rrem, rgls, rgrs, rwl, rwr):
    """Proposed engine. replay=False: control from the input, records written to r*.

    replay=True: control path skipped and the recorded per-sample gains are applied
    (stem-through measurement). Returns C, L', R', Ls, Rs.
    """
    N = l_in.size
    resid = F[F_RESID] != 0; gate = F[F_GATE] != 0; detm = F[F_DET]
    look = F[F_LOOK] != 0; nb = F[F_NB]; duck = F[F_DUCK] != 0; dom = F[F_DOM] != 0
    fb_only = F[F_DOM] == 2
    side = F[F_SIDE] != 0; floor_m = F[F_FLOOR]
    need_acc = resid or gate or duck
    oc = np.zeros(N, np.float32); ol = np.zeros(N, np.float32); orr = np.zeros(N, np.float32)
    ols = np.zeros(N, np.float32); ors = np.zeros(N, np.float32)
    det_lp_l = ZERO; det_lp_r = ZERO
    acc_lr = ZERO; acc_ll = ZERO; acc_rr = ZERO; g_c = ZERO
    p_gc = ZERO; p_rem = ZERO; env = ZERO
    pres_ic1 = ZERO; pres_ic2 = ZERO
    # per-band state
    b_lr = np.zeros(3, np.float32); b_ll = np.zeros(3, np.float32); b_rr = np.zeros(3, np.float32)
    d_lr = np.zeros(3, np.float32); d_fb = np.zeros(3, np.float32)
    efl = np.zeros(3, np.float32); esl = np.zeros(3, np.float32)
    efr = np.zeros(3, np.float32); esr = np.zeros(3, np.float32)
    dkl = np.ones(3, np.float32); dkr = np.ones(3, np.float32)
    p_gls = np.zeros(3, np.float32); p_grs = np.zeros(3, np.float32)
    p_wl = np.zeros(3, np.float32); p_wr = np.zeros(3, np.float32)
    gls_i = np.zeros(3, np.float32); grs_i = np.zeros(3, np.float32)
    wl_i = np.zeros(3, np.float32); wr_i = np.zeros(3, np.float32)
    gls_s = np.zeros(3, np.float32); grs_s = np.zeros(3, np.float32)
    wl_s = np.zeros(3, np.float32); wr_s = np.zeros(3, np.float32)
    # filter state: [channel, stage]
    dbs1 = np.zeros((2, 2), np.float32); dbs2 = np.zeros((2, 2), np.float32)   # det band HP/LP
    dxs1 = np.zeros((2, 2), np.float32); dxs2 = np.zeros((2, 2), np.float32)   # det split
    axs1 = np.zeros((2, 2), np.float32); axs2 = np.zeros((2, 2), np.float32)   # audio split
    shp1 = np.zeros(2, np.float32); shp2 = np.zeros(2, np.float32)
    slp1 = np.zeros(2, np.float32); slp2 = np.zeros(2, np.float32)
    haas = np.zeros((2, HAAS_RING), np.float32); ap = np.zeros((2, AP_RING), np.float32)
    dB = np.zeros((2, 3), np.float32); aB = np.zeros((2, 3), np.float32)
    widx = 0
    hdly = int(K[K_HDLY]); adly = int(K[K_ADLY])
    adaptive = K[K_CMODE] == 1; coff = K[K_CMODE] == 2
    cl = K[K_LSCL]; cr = K[K_LSCR]
    aw = P[P_AW]; omw = ONE - aw
    for s0 in range(0, N, BLOCK):
        n = min(BLOCK, N - s0)
        inv_n = ONE / f32(n)
        if not replay:
            env_new = env + f32(n) * K[K_ENVSTEP]
            if env_new > ONE:
                env_new = ONE
            if adaptive:
                denom = np.sqrt(acc_ll * acc_rr) + TINY
                corr = clampf(acc_lr / denom, -ONE, ONE)
                bal = abs(acc_ll - acc_rr) / (acc_ll + acc_rr + TINY)
                p = (corr if corr > ZERO else ZERO) * (ONE - bal)
                m = (p - K[K_THRESH]) * K[K_INVTHR]
                g_target = K[K_STRENGTH] * clampf(m, ZERO, ONE)
            else:
                g_target = K[K_STRENGTH]
            a = K[K_AATT] if g_target > g_c else K[K_AREL]
            g_c = g_target + (g_c - g_target) * f32(math.pow(a, n))
            if coff:
                g_c = ZERO
            gc_new = C7071 * g_c * env_new
            rem_new = HALF * (ONE - K[K_WIDTH]) * g_c * env_new
            gc_i = p_gc; gc_step = (gc_new - p_gc) * inv_n
            rem_i = p_rem; rem_step = (rem_new - p_rem) * inv_n
            p_gc = gc_new; p_rem = rem_new
            for bi in range(nb):
                if dom:
                    fbb = HALF * d_fb[bi] - HALF
                    sfb = ONE - (fbb + ONE) * (fbb + ONE)
                    if fb_only:
                        gl_t = DOM_LR_NEUTRAL * sfb; gr_t = gl_t
                    else:
                        ul = HALF * (d_lr[bi] + ONE)
                        ur = HALF * (ONE - d_lr[bi])
                        gl_t = (ONE - ul * ul) * sfb
                        gr_t = (ONE - ur * ur) * sfb
                else:
                    gl_t = P[P_G0]; gr_t = P[P_G0]
                ll = b_ll[bi]; rr = b_rr[bi]; lr = b_lr[bi]
                sd = (ll - rr) / (ll + rr + TINY)     # signed balance, + = left dominant
                dpl = sd if sd > ZERO else ZERO; dpr = -sd if sd < ZERO else ZERO
                if gate:
                    if side:
                        # close only the surround on the dominant side
                        pq = P[P_GATEP] * P[P_SIDEK]
                        gl_t *= f32(math.pow(ONE - dpl, P[P_GATEP])) * f32(math.pow(ONE - dpr, pq))
                        gr_t *= f32(math.pow(ONE - dpr, P[P_GATEP])) * f32(math.pow(ONE - dpl, pq))
                    else:
                        gg = f32(math.pow(ONE - abs(sd), P[P_GATEP]))
                        gl_t *= gg; gr_t *= gg
                wl_t = ZERO; wr_t = ZERO
                if resid:
                    reg = P[P_EPS] * (ll + rr) + TINY
                    fl = P[P_WMIN]; fr = P[P_WMIN]
                    if floor_m == 1:
                        fl = fl * f32(math.pow(ONE - abs(sd), P[P_FLOORQ])); fr = fl
                    elif floor_m == 2:
                        # wr carries L' into Rs: relax it only when L dominates, and vice versa
                        fr = fr * f32(math.pow(ONE - dpl, P[P_FLOORQ]))
                        fl = fl * f32(math.pow(ONE - dpr, P[P_FLOORQ]))
                    wl_t = clampf(lr / (rr + reg), fl, P[P_WMAX])
                    wr_t = clampf(lr / (ll + reg), fr, P[P_WMAX])
                if duck:
                    thr = P[P_DTHR]; rel = P[P_DREL]
                    for ch in range(2):
                        ef = efl[bi] if ch == 0 else efr[bi]
                        es = esl[bi] if ch == 0 else esr[bi]
                        ratio = ef / (es + TINY)
                        tgt = ONE if ratio <= thr else f32(math.pow(thr / ratio, P[P_DQ]))
                        prev = dkl[bi] if ch == 0 else dkr[bi]
                        relv = ONE - (ONE - prev) * rel
                        nv = tgt if tgt < relv else relv
                        if ch == 0:
                            dkl[bi] = nv
                        else:
                            dkr[bi] = nv
                    gl_t *= dkl[bi]; gr_t *= dkr[bi]
                gl_n = gl_t * env_new; gr_n = gr_t * env_new
                gls_i[bi] = p_gls[bi]; gls_s[bi] = (gl_n - p_gls[bi]) * inv_n
                grs_i[bi] = p_grs[bi]; grs_s[bi] = (gr_n - p_grs[bi]) * inv_n
                wl_i[bi] = p_wl[bi]; wl_s[bi] = (wl_t - p_wl[bi]) * inv_n
                wr_i[bi] = p_wr[bi]; wr_s[bi] = (wr_t - p_wr[bi]) * inv_n
                p_gls[bi] = gl_n; p_grs[bi] = gr_n; p_wl[bi] = wl_t; p_wr[bi] = wr_t
            env = env_new
        hp_a = K[K_DETHP]; a_corr = K[K_ACORR]; omc = ONE - a_corr
        a_dom = K[K_ADOM]; omd = ONE - a_dom
        for i in range(s0, s0 + n):
            l0 = l_in[i]; r0 = r_in[i]
            if replay:
                gc_i = rgc[i]; rem_i = rrem[i]
            else:
                det_lp_l += hp_a * (l0 - det_lp_l)
                det_lp_r += hp_a * (r0 - det_lp_r)
                el = l0 - det_lp_l; er = r0 - det_lp_r
                acc_lr = a_corr * acc_lr + omc * (el * er)
                acc_ll = a_corr * acc_ll + omc * (el * el)
                acc_rr = a_corr * acc_rr + omc * (er * er)
                # detector inputs (raw, pre centre removal)
                if detm == 0:
                    xl = l0; xr = r0
                elif detm == 1:
                    xl = el; xr = er
                else:
                    for ch in range(2):
                        x = l0 if ch == 0 else r0
                        v1, v2, dbs1[ch, 0], dbs2[ch, 0] = svf(x, dbs1[ch, 0], dbs2[ch, 0], P[P_BHA1], P[P_BHA2], P[P_BHA3])
                        x = x - K[K_SHK] * v1 - v2
                        v1, v2, dbs1[ch, 1], dbs2[ch, 1] = svf(x, dbs1[ch, 1], dbs2[ch, 1], P[P_BLA1], P[P_BLA2], P[P_BLA3])
                        if ch == 0:
                            xl = v2
                        else:
                            xr = v2
                for ch in range(2):
                    x = xl if ch == 0 else xr
                    if nb == 1:
                        dB[ch, 0] = x
                    else:
                        v1, v2, dxs1[ch, 0], dxs2[ch, 0] = svf(x, dxs1[ch, 0], dxs2[ch, 0], P[P_X1A1], P[P_X1A2], P[P_X1A3])
                        dB[ch, 0] = v2
                        rest = x - v2
                        if nb == 2:
                            dB[ch, 1] = rest
                        else:
                            v1, v2, dxs1[ch, 1], dxs2[ch, 1] = svf(rest, dxs1[ch, 1], dxs2[ch, 1], P[P_X2A1], P[P_X2A2], P[P_X2A3])
                            dB[ch, 1] = v2
                            dB[ch, 2] = rest - v2
                for bi in range(nb):
                    bl = dB[0, bi]; br = dB[1, bi]
                    if dom:
                        if not fb_only:
                            dlr = clampf(CLIPG * (abs(br) - abs(bl)), -ONE, ONE)
                            d_lr[bi] = a_dom * d_lr[bi] + omd * dlr
                        f = HALF * (bl + br); b = HALF * (bl - br)
                        dfb = clampf(CLIPG * (abs(f) - abs(b)), -ONE, ONE)
                        d_fb[bi] = a_dom * d_fb[bi] + omd * dfb
                    if need_acc:
                        # HP/split are linear, so removal can be applied after them
                        rm = rem_i * (bl + br)
                        pl = bl - rm; pr = br - rm
                        b_lr[bi] = aw * b_lr[bi] + omw * (pl * pr)
                        b_ll[bi] = aw * b_ll[bi] + omw * (pl * pl)
                        b_rr[bi] = aw * b_rr[bi] + omw * (pr * pr)
                        if duck:
                            if resid:
                                ql = pl - wl_i[bi] * pr; qr = pr - wr_i[bi] * pl
                            else:
                                ql = cl * pl + cr * pr; qr = cl * pr + cr * pl
                            ql = ql * ql; qr = qr * qr
                            efl[bi] += P[P_DAF] * (ql - efl[bi]); esl[bi] += P[P_DAS] * (ql - esl[bi])
                            efr[bi] += P[P_DAF] * (qr - efr[bi]); esr[bi] += P[P_DAS] * (qr - esr[bi])
                rgc[i] = gc_i; rrem[i] = rem_i
            mid = l0 + r0
            c0 = gc_i * mid
            pv3 = c0 - pres_ic2
            pv1 = K[K_PA1] * pres_ic1 + K[K_PA2] * pv3
            pv2 = pres_ic2 + K[K_PA2] * pres_ic1 + K[K_PA3] * pv3
            pres_ic1 = TWO * pv1 - pres_ic1
            pres_ic2 = TWO * pv2 - pres_ic2
            c0 += K[K_PM1] * pv1
            oc[i] = c0
            rem = rem_i * mid
            l1 = l0 - rem; r1 = r0 - rem
            ol[i] = l1; orr[i] = r1
            if not replay:
                gc_i += gc_step; rem_i += rem_step
            # ---- surround audio path
            al = l1; ar = r1
            if look:
                haas[0, widx & (HAAS_RING - 1)] = al
                haas[1, widx & (HAAS_RING - 1)] = ar
                al = haas[0, (widx - hdly) & (HAAS_RING - 1)]
                ar = haas[1, (widx - hdly) & (HAAS_RING - 1)]
            for ch in range(2):
                x = al if ch == 0 else ar
                if nb == 1:
                    aB[ch, 0] = x
                else:
                    v1, v2, axs1[ch, 0], axs2[ch, 0] = svf(x, axs1[ch, 0], axs2[ch, 0], P[P_X1A1], P[P_X1A2], P[P_X1A3])
                    aB[ch, 0] = v2
                    rest = x - v2
                    if nb == 2:
                        aB[ch, 1] = rest
                    else:
                        v1, v2, axs1[ch, 1], axs2[ch, 1] = svf(rest, axs1[ch, 1], axs2[ch, 1], P[P_X2A1], P[P_X2A2], P[P_X2A3])
                        aB[ch, 1] = v2
                        aB[ch, 2] = rest - v2
            x0 = ZERO; x1 = ZERO
            for bi in range(nb):
                if replay:
                    gl = rgls[i, bi]; gr = rgrs[i, bi]; wl = rwl[i, bi]; wr = rwr[i, bi]
                else:
                    gl = gls_i[bi]; gr = grs_i[bi]; wl = wl_i[bi]; wr = wr_i[bi]
                    rgls[i, bi] = gl; rgrs[i, bi] = gr; rwl[i, bi] = wl; rwr[i, bi] = wr
                    gls_i[bi] += gls_s[bi]; grs_i[bi] += grs_s[bi]
                    wl_i[bi] += wl_s[bi]; wr_i[bi] += wr_s[bi]
                bl = aB[0, bi]; br = aB[1, bi]
                if resid:
                    x0 += gl * (bl - wl * br)
                    x1 += gr * (br - wr * bl)
                else:
                    x0 += gl * (cl * bl + cr * br)
                    x1 += gr * (cl * br + cr * bl)
            for ch in range(2):
                x = x0 if ch == 0 else x1
                v1, v2, shp1[ch], shp2[ch] = svf(x, shp1[ch], shp2[ch], K[K_SHA1], K[K_SHA2], K[K_SHA3])
                x = x - K[K_SHK] * v1 - v2
                v1, v2, slp1[ch], slp2[ch] = svf(x, slp1[ch], slp2[ch], K[K_SLA1], K[K_SLA2], K[K_SLA3])
                if ch == 0:
                    x0 = v2
                else:
                    x1 = v2
            if not look:
                haas[0, widx & (HAAS_RING - 1)] = x0
                haas[1, widx & (HAAS_RING - 1)] = x1
                x0 = haas[0, (widx - hdly) & (HAAS_RING - 1)]
                x1 = haas[1, (widx - hdly) & (HAAS_RING - 1)]
            apg = K[K_APG]
            wd0 = ap[0, (widx - adly) & (AP_RING - 1)]
            w0 = x0 + apg * wd0
            ap[0, widx & (AP_RING - 1)] = w0
            ols[i] = wd0 - apg * w0
            wd1 = ap[1, (widx - adly) & (AP_RING - 1)]
            w1 = x1 - apg * wd1
            ap[1, widx & (AP_RING - 1)] = w1
            ors[i] = wd1 + apg * w1
            widx += 1
    return oc, ol, orr, ols, ors


def make_flags(resid=0, gate=0, det=0, look=0, nb=1, duck=0, dom=1, side=0, floor=0):
    """det: 0 full band, 1 one-pole HP, 2 surround band. dom: 0 off (fixed g0), 1 patent, 2 F/B axis only.
    side: 1 = gate only the dominant side. floor: 0 fixed wmin, 1 relaxed by balance, 2 relaxed on the dominant-feed weight."""
    F = np.zeros(F_N, dtype=np.int64)
    F[F_RESID] = resid; F[F_GATE] = gate; F[F_DET] = det; F[F_LOOK] = look
    F[F_NB] = nb; F[F_DUCK] = duck; F[F_DOM] = dom
    F[F_SIDE] = side; F[F_FLOOR] = floor
    return F


def run(l, r, K, F, P):
    N = l.size; nb = int(F[F_NB])
    rec = (np.zeros(N, np.float32), np.zeros(N, np.float32),
           np.zeros((N, nb), np.float32), np.zeros((N, nb), np.float32),
           np.zeros((N, nb), np.float32), np.zeros((N, nb), np.float32))
    out = engine(l.astype(np.float32), r.astype(np.float32), K, F, P, False, *rec)
    return out, rec


def replay(l, r, K, F, P, rec):
    return engine(l.astype(np.float32), r.astype(np.float32), K, F, P, True, *rec)


# ---------------------------------------------------------------- firmware port
# port_engine() mirrors the ported ADAPTIVE surround in upmix.c operation for
# operation (C_p1_B2_k05 with the band limit moved ahead of the Haas delay).
(X_AW, X_DAF, X_DAS, X_DRELB, X_AATTB, X_ARELB, X_GK, X_FQ, X_N) = range(9)
PORT = dict(fb_neutral=0.75, w_floor=0.5623, w_ceil=4.0, w_eps=1e-3, gate_k=0.5, floor_q=1.0,
            duck_thr=4.0, duck_depth=0.5)
PF_NEUTRAL = f32(0.75); PW_FLOOR = f32(0.5623); PW_CEIL = f32(4.0); PW_EPS = f32(1e-3)
PDUCK_THR = f32(4.0)


def port_extra(K, w_tau=40.0, fast=2.0, slow=60.0, rel=60.0, n=BLOCK):
    """Python fallback for the coefficients harness.h_coeffs() returns from the C."""
    X = np.zeros(X_N, np.float32)
    X[X_AW] = math.exp(-1.0 / (FS * w_tau * 1e-3))
    X[X_DAF] = 1.0 - math.exp(-1.0 / (FS * fast * 1e-3))
    X[X_DAS] = 1.0 - math.exp(-1.0 / (FS * slow * 1e-3))
    X[X_DRELB] = f32(math.exp(-1.0 / (FS * rel * 1e-3))) ** n
    X[X_AATTB] = float(K[K_AATT]) ** n
    X[X_ARELB] = float(K[K_AREL]) ** n
    X[X_GK] = 0.75; X[X_FQ] = 0.0
    return X


@njit(cache=True, inline='always')
def powc(x, e):
    """upmix_powc(): exact forms for the exponents the firmware folds."""
    if e == ONE:
        return x
    if e == HALF:
        return np.sqrt(x)
    if e == f32(0.75):
        s = np.sqrt(x)
        return s * np.sqrt(s)
    if e == ZERO:
        return ONE
    return f32(math.pow(x, e))


@njit(cache=True, inline='always')
def band2(x, s1, s2, ch, K):
    v1, v2, s1[ch], s2[ch] = svf(x, s1[ch], s2[ch], K[K_SHA1], K[K_SHA2], K[K_SHA3])
    x = x - K[K_SHK] * v1 - v2
    return x


@njit(cache=True)
def port_engine(l_in, r_in, K, X, replay, rgc, rrem, rgls, rgrs, rwl, rwr):
    N = l_in.size
    oc = np.zeros(N, np.float32); ol = np.zeros(N, np.float32); orr = np.zeros(N, np.float32)
    ols = np.zeros(N, np.float32); ors = np.zeros(N, np.float32)
    det_lp_l = ZERO; det_lp_r = ZERO
    acc_lr = ZERO; acc_ll = ZERO; acc_rr = ZERO; g_c = ZERO
    p_gc = ZERO; p_rem = ZERO; env = ZERO; pres_ic1 = ZERO; pres_ic2 = ZERO
    dom_fb = ZERO; w_lr = ZERO; w_ll = ZERO; w_rr = ZERO
    ef = np.zeros(2, np.float32); es = np.zeros(2, np.float32); att = np.zeros(2, np.float32)
    p_gls = ZERO; p_grs = ZERO; p_wl = ZERO; p_wr = ZERO
    shp1 = np.zeros(2, np.float32); shp2 = np.zeros(2, np.float32)
    slp1 = np.zeros(2, np.float32); slp2 = np.zeros(2, np.float32)
    haas = np.zeros((2, HAAS_RING), np.float32); ap = np.zeros((2, AP_RING), np.float32)
    widx = 0
    hdly = int(K[K_HDLY]); adly = int(K[K_ADLY])
    adaptive = K[K_CMODE] == 1; coff = K[K_CMODE] == 2
    aw = X[X_AW]; omw = ONE - aw
    gls_i = ZERO; grs_i = ZERO; wl_i = ZERO; wr_i = ZERO
    gls_step = ZERO; grs_step = ZERO; wl_step = ZERO; wr_step = ZERO
    gc_i = ZERO; rem_i = ZERO; gc_step = ZERO; rem_step = ZERO; senv = ZERO; fill = 0
    for s0 in range(0, N, BLOCK):
        n = min(BLOCK, N - s0)
        inv_n = ONE / f32(n)
        if not replay:
            env_new = env + f32(n) * K[K_ENVSTEP]
            if env_new > ONE:
                env_new = ONE
            if adaptive:
                denom = np.sqrt(acc_ll * acc_rr) + TINY
                corr = clampf(acc_lr / denom, -ONE, ONE)
                bal = abs(acc_ll - acc_rr) / (acc_ll + acc_rr + TINY)
                p = (corr if corr > ZERO else ZERO) * (ONE - bal)
                m = (p - K[K_THRESH]) * K[K_INVTHR]
                g_target = K[K_STRENGTH] * clampf(m, ZERO, ONE)
            else:
                g_target = K[K_STRENGTH]
            ab = X[X_AATTB] if g_target > g_c else X[X_ARELB]
            g_c = g_target + (g_c - g_target) * ab
            if coff:
                g_c = ZERO
            fbb = HALF * dom_fb - HALF
            sfb = ONE - (fbb + ONE) * (fbb + ONE)
            g = PF_NEUTRAL * sfb
            d = (w_ll - w_rr) / (w_ll + w_rr + TINY)
            dl = d if d > ZERO else ZERO
            dr = -d if d < ZERO else ZERO
            gls_t = g * ((ONE - dl) * powc(ONE - dr, X[X_GK]))
            grs_t = g * ((ONE - dr) * powc(ONE - dl, X[X_GK]))
            fl = PW_FLOOR * powc(ONE - dr, X[X_FQ])
            fr = PW_FLOOR * powc(ONE - dl, X[X_FQ])
            a = ONE - p_rem; b = -p_rem
            aa = a * a; bb2 = b * b; ab2 = TWO * a * b
            ll1 = aa * w_ll + ab2 * w_lr + bb2 * w_rr
            rr1 = bb2 * w_ll + ab2 * w_lr + aa * w_rr
            if ll1 < ZERO:
                ll1 = ZERO
            if rr1 < ZERO:
                rr1 = ZERO
            lr1 = a * b * (w_ll + w_rr) + (aa + bb2) * w_lr
            reg = PW_EPS * (ll1 + rr1) + TINY
            wl_t = clampf(lr1 / (rr1 + reg), fl, PW_CEIL)
            wr_t = clampf(lr1 / (ll1 + reg), fr, PW_CEIL)
            for ch in range(2):
                ratio = ef[ch] / (es[ch] + TINY)
                tgt = ONE if ratio <= PDUCK_THR else np.sqrt(PDUCK_THR / ratio)
                relv = att[ch] * X[X_DRELB]
                a = ONE - tgt
                att[ch] = a if a > relv else relv
            gls_t *= ONE - att[0]
            grs_t *= ONE - att[1]
            gc_new = C7071 * g_c * env_new
            rem_new = HALF * (ONE - K[K_WIDTH]) * g_c * env_new
            if fill >= hdly:
                senv += f32(n) * K[K_ENVSTEP]
                if senv > ONE:
                    senv = ONE
            else:
                fill += n
            gls_new = gls_t * senv; grs_new = grs_t * senv
            gc_i = p_gc; gc_step = (gc_new - p_gc) * inv_n
            rem_i = p_rem; rem_step = (rem_new - p_rem) * inv_n
            gls_i = p_gls; gls_step = (gls_new - p_gls) * inv_n
            grs_i = p_grs; grs_step = (grs_new - p_grs) * inv_n
            wl_i = p_wl; wl_step = (wl_t - p_wl) * inv_n
            wr_i = p_wr; wr_step = (wr_t - p_wr) * inv_n
            p_gc = gc_new; p_rem = rem_new; p_gls = gls_new; p_grs = grs_new
            p_wl = wl_t; p_wr = wr_t
            env = env_new
        hp_a = K[K_DETHP]; a_corr = K[K_ACORR]; omc = ONE - a_corr
        a_dom = K[K_ADOM]; omd = ONE - a_dom
        for i in range(s0, s0 + n):
            l0 = l_in[i]; r0 = r_in[i]
            if replay:
                gc_i = rgc[i]; rem_i = rrem[i]
                gls_i = rgls[i, 0]; grs_i = rgrs[i, 0]; wl_i = rwl[i, 0]; wr_i = rwr[i, 0]
            else:
                det_lp_l += hp_a * (l0 - det_lp_l)
                det_lp_r += hp_a * (r0 - det_lp_r)
                el = l0 - det_lp_l; er = r0 - det_lp_r
                acc_lr = a_corr * acc_lr + omc * (el * er)
                acc_ll = a_corr * acc_ll + omc * (el * el)
                acc_rr = a_corr * acc_rr + omc * (er * er)
                rgc[i] = gc_i; rrem[i] = rem_i
                rgls[i, 0] = gls_i; rgrs[i, 0] = grs_i; rwl[i, 0] = wl_i; rwr[i, 0] = wr_i
            mid = l0 + r0
            c0 = gc_i * mid
            pv3 = c0 - pres_ic2
            pv1 = K[K_PA1] * pres_ic1 + K[K_PA2] * pv3
            pv2 = pres_ic2 + K[K_PA2] * pres_ic1 + K[K_PA3] * pv3
            pres_ic1 = TWO * pv1 - pres_ic1
            pres_ic2 = TWO * pv2 - pres_ic2
            c0 += K[K_PM1] * pv1
            oc[i] = c0
            rem_c = rem_i
            rem = rem_i * mid
            l1 = l0 - rem; r1 = r0 - rem
            ol[i] = l1; orr[i] = r1
            gc_i += gc_step; rem_i += rem_step
            b = np.zeros(2, np.float32)
            for ch in range(2):
                x = l0 if ch == 0 else r0
                x = band2(x, shp1, shp2, ch, K)
                v1, v2, slp1[ch], slp2[ch] = svf(x, slp1[ch], slp2[ch], K[K_SLA1], K[K_SLA2], K[K_SLA3])
                b[ch] = v2
            b0 = b[0]; b1 = b[1]
            rb = rem_c * (b0 + b1)
            p0 = b0 - rb; p1 = b1 - rb
            if not replay:
                f = HALF * (b0 + b1); bb = HALF * (b0 - b1)
                dfb = clampf(CLIPG * (abs(f) - abs(bb)), -ONE, ONE)
                dom_fb = a_dom * dom_fb + omd * dfb
                w_lr = aw * w_lr + omw * (b0 * b1)
                w_ll = aw * w_ll + omw * (b0 * b0)
                w_rr = aw * w_rr + omw * (b1 * b1)
                q0 = p0 - wl_i * p1; q1 = p1 - wr_i * p0
                q0 = q0 * q0; q1 = q1 * q1
                ef[0] += X[X_DAF] * (q0 - ef[0]); es[0] += X[X_DAS] * (q0 - es[0])
                ef[1] += X[X_DAF] * (q1 - ef[1]); es[1] += X[X_DAS] * (q1 - es[1])
            haas[0, widx & (HAAS_RING - 1)] = p0
            haas[1, widx & (HAAS_RING - 1)] = p1
            d0 = haas[0, (widx - hdly) & (HAAS_RING - 1)]
            d1 = haas[1, (widx - hdly) & (HAAS_RING - 1)]
            x0 = gls_i * (d0 - wl_i * d1)
            x1 = grs_i * (d1 - wr_i * d0)
            if not replay:
                gls_i += gls_step; grs_i += grs_step; wl_i += wl_step; wr_i += wr_step
            apg = K[K_APG]
            wd0 = ap[0, (widx - adly) & (AP_RING - 1)]
            w0 = x0 + apg * wd0
            ap[0, widx & (AP_RING - 1)] = w0
            ols[i] = wd0 - apg * w0
            wd1 = ap[1, (widx - adly) & (AP_RING - 1)]
            w1 = x1 - apg * wd1
            ap[1, widx & (AP_RING - 1)] = w1
            ors[i] = wd1 + apg * w1
            widx += 1
    return oc, ol, orr, ols, ors


def port_run(l, r, K, X):
    N = l.size
    rec = (np.zeros(N, np.float32), np.zeros(N, np.float32)) + tuple(np.zeros((N, 1), np.float32) for _ in range(4))
    return port_engine(l.astype(np.float32), r.astype(np.float32), K, X, False, *rec), rec


def port_replay(l, r, K, X, rec):
    return port_engine(l.astype(np.float32), r.astype(np.float32), K, X, True, *rec)
