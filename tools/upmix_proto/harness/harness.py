"""Compile the real upmix.c on the host and check it against HEAD and the Python model.

Usage (from tools/upmix_proto): <python with numpy+scipy+numba> harness/harness.py
Writes out/harness_report.txt and out/harness_metrics.csv.
"""
import ctypes
import csv
import os
import shutil
import subprocess
import sys
import tempfile
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import upmix_model as m   # noqa: E402
import signals as S       # noqa: E402
import run_side as RS     # noqa: E402
import accept as A        # noqa: E402
from run_eval import band, p, db, win_energy, WARM, D, OUT   # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
FW = os.path.join(REPO, 'firmware', 'DSPi')
DEFP = [100.0, 25.0, 30.0, 10.0, 100.0, 200.0, 12.0, 300.0, 7000.0, 90.0, 0.0]
FP = ctypes.POINTER(ctypes.c_float)


def build(src_c, src_h, new):
    """Copy upmix.c/h beside the stubs so their "config.h" resolves to the stub."""
    d = tempfile.mkdtemp(prefix='upmix_h_')
    for f in os.listdir(os.path.join(HERE, 'stubs')):
        shutil.copy(os.path.join(HERE, 'stubs', f), d)
    open(os.path.join(d, 'upmix.c'), 'w').write(src_c)
    open(os.path.join(d, 'upmix.h'), 'w').write(src_h)
    shutil.copy(os.path.join(HERE, 'harness.c'), d)
    so = os.path.join(d, 'libupmix.so')
    # No FMA contraction, so the host result is comparable operation for operation.
    cmd = ['cc', '-O2', '-ffp-contract=off', '-fno-fast-math', '-shared', '-fPIC', '-I', d,
           os.path.join(d, 'harness.c'), '-o', so, '-lm'] + (['-DUPMIX_NEW'] if new else [])
    subprocess.check_call(cmd)
    lib = ctypes.CDLL(so)
    lib.h_run.argtypes = [FP, ctypes.c_int, ctypes.c_int, ctypes.c_int, FP, FP, ctypes.c_int] + [FP] * 5
    lib.h_coeffs.argtypes = [FP, ctypes.c_int, ctypes.c_int, ctypes.c_int, FP]
    lib.h_run_switch.argtypes = ([FP] + [ctypes.c_int] * 5 + [FP, FP, ctypes.c_int] + [FP] * 5)
    if new:
        lib.h_run_stems.argtypes = ([FP, ctypes.c_int, ctypes.c_int, ctypes.c_int, FP, FP, ctypes.c_int,
                                     ctypes.c_int, FP, FP, FP, FP, FP, FP, FP])
    return lib


def ptr(a):
    return a.ctypes.data_as(FP)


def c_run(lib, L, R, cm=1, sm=2, params=None, blk=48):
    pa = np.array(params or DEFP, np.float32)
    L = np.ascontiguousarray(L, np.float32); R = np.ascontiguousarray(R, np.float32)
    outs = [np.zeros(L.size, np.float32) for _ in range(5)]
    lib.h_run(ptr(pa), cm, sm, blk, ptr(L), ptr(R), L.size, *[ptr(o) for o in outs])
    return outs   # C, L', R', Ls, Rs


def c_coeffs(lib, cm=1, sm=2, params=None):
    pa = np.array(params or DEFP, np.float32)
    out = np.zeros(64, np.float32)
    k = lib.h_coeffs(ptr(pa), cm, sm, m.BLOCK, ptr(out))
    return out[:k]


def model_KX(v):
    K = v[:m.K_N].copy()
    X = np.zeros(m.X_N, np.float32)
    X[m.X_AATTB], X[m.X_ARELB] = v[27], v[28]
    X[m.X_AW], X[m.X_DAF], X[m.X_DAS], X[m.X_DRELB] = v[29], v[30], v[31], v[32]
    X[m.X_GK], X[m.X_FQ] = v[33], v[34]
    return K, X


def mix_of(stems):
    return (sum(s[0] for s in stems.values()).astype(np.float32),
            sum(s[1] for s in stems.values()).astype(np.float32))


def maxdiff(a, b):
    return max(float(np.max(np.abs(np.asarray(x, np.float64) - np.asarray(y, np.float64)))) for x, y in zip(a, b))


def head_equivalence(new, head, tests, log):
    """Every non-ADAPTIVE-surround path, and C/L'/R' in ADAPTIVE, against HEAD."""
    variants = [('default', DEFP), ('width0_pres+6', DEFP[:1] + [0.0] + DEFP[2:10] + [6.0]),
                ('width100_pres-9', DEFP[:1] + [100.0] + DEFP[2:10] + [-9.0]),
                ('dly0_hpf80_lpf12k', DEFP[:6] + [0.0, 80.0, 12000.0] + DEFP[9:]),
                ('dly20_decorr0', DEFP[:6] + [20.0] + DEFP[7:9] + [0.0, 0.0])]
    worst = 0.0
    for tname in ('mix_vox_gtr_amb', 'mix_kick_keysL', 'antiphase', 'diffuse'):
        L, R = mix_of(tests[tname])
        for vname, pv in variants:
            for cm in (0, 1, 2):
                for sm in (0, 1, 2):
                    a = c_run(new, L, R, cm, sm, pv); b = c_run(head, L, R, cm, sm, pv)
                    chans = (0, 1, 2) if sm == 2 else (0, 1, 2, 3, 4)
                    d = maxdiff([a[k] for k in chans], [b[k] for k in chans])
                    worst = max(worst, d)
                    if d != 0.0:
                        log('  HEAD MISMATCH %s %s cm=%d sm=%d: %g' % (tname, vname, cm, sm, d))
    log('HEAD equivalence (passive/off surround all outputs, adaptive C/L/R; 4 tests x 5 settings x 9 modes): worst %g' % worst)


def latency(new, head, log):
    """Impulse onset and peak through Ls/Rs for every engine and delay setting.

    The new adaptive gate closes an isolated impulse completely, so its impulse rides as a
    -60 dB stem on steady uncorrelated noise and is measured stem-through with the mix's control.
    """
    N = 24000; t0 = 12000; amp = 1e-3
    rng = np.random.default_rng(5)
    nl = (rng.standard_normal(N) * 0.05).astype(np.float32); nr = (rng.standard_normal(N) * 0.05).astype(np.float32)
    for dly in (0.0, 1.0, 6.0, 12.0, 20.0):
        pv = DEFP[:6] + [dly] + DEFP[7:9] + [0.0, 0.0]
        res = {}
        for side in (0, 1):
            imp = np.zeros(N, np.float32); imp[t0] = 1.0
            L, R = (imp, 0 * imp) if side == 0 else (0 * imp, imp)
            for nm, lib, sm in (('HEAD passive', head, 1), ('HEAD adaptive', head, 2), ('new passive', new, 1)):
                o = c_run(lib, L, R, 2, sm, pv)
                res[(nm, side)] = (o[3], o[4])
            stems = {'noise': (nl, nr, 'ambience', None), 'imp': (L * amp, R * amp, 'direct', None)}
            names, sls, srs, _, _, _ = c_stems(new, stems, 2, 2, pv)
            k = names.index('imp')
            res[('new adaptive', side)] = (sls[k] / amp, srs[k] / amp)
        out = set()
        for key, (ls, rs) in res.items():
            for x in (ls, rs):
                a = np.abs(x[t0 - 2000:].astype(np.float64))
                nz = np.nonzero(a > 1e-7 * a.max())[0] if a.max() > 0 else np.array([-1])
                out.add((key[0], int(nz[0]) - 2000, int(np.argmax(a)) - 2000))
        hd = int(dly * 48 + 0.5)
        log('latency dly=%4.1f ms (Haas %d + allpass 480 = %d samples): %s'
            % (dly, hd, hd + 480, sorted({(o[1], o[2]) for o in out})))
        for o in sorted(out):
            if o[1] != hd + 480:
                log('  LATENCY MISMATCH %s' % (o,))


def model_compare(new, tests, log):
    worst = 0.0; worst_rel = 0.0
    K, X = model_KX(c_coeffs(new))
    for tname, stems in tests.items():
        L, R = mix_of(stems)
        a = c_run(new, L, R)
        b, _ = m.port_run(L, R, K, X)
        b = [b[0], b[1], b[2], b[3], b[4]]
        d = maxdiff(a, b)
        pk = max(float(np.max(np.abs(x))) for x in b) + 1e-30
        worst = max(worst, d); worst_rel = max(worst_rel, d / pk)
        log('  C vs port model %-18s max abs %.3g (rel to peak %.3g)' % (tname, d, d / pk))
    # same check off the defaults
    for pv, cm in ((DEFP[:6] + [0.0] + DEFP[7:], 1), (DEFP[:6] + [20.0, 120.0, 15000.0] + DEFP[9:], 0),
                   (DEFP[:1] + [100.0] + DEFP[2:], 2)):
        K, X = model_KX(c_coeffs(new, cm, 2, pv))
        L, R = mix_of(tests['mix_vox_gtr_amb'])
        a = c_run(new, L, R, cm, 2, pv); b, _ = m.port_run(L, R, K, X)
        d = maxdiff(a, b); worst = max(worst, d)
        log('  C vs port model mix_vox_gtr_amb cm=%d dly=%g hpf=%g lpf=%g: max abs %.3g' % (cm, pv[6], pv[7], pv[8], d))
    log('C vs port model worst abs diff %.3g, worst relative %.3g' % (worst, worst_rel))
    return K, X


def head_model_check(head, tests, log):
    """HEAD adaptive C against baseline_run: the earlier prototype's baseline."""
    v = c_coeffs(head)
    K = v[:m.K_N].copy()
    L, R = mix_of(tests['mix_vox_gtr_amb'])
    a = c_run(head, L, R)
    b = m.baseline_run(L, R, K)
    log('HEAD C vs baseline_run (model) on mix_vox_gtr_amb: max abs %.3g' % maxdiff(a, b[:5]))


def reset_checks(new, head, tests, log):
    """After a mode switch without a park, the rears must start from cleared rings: exactly zero for the Haas delay."""
    L, R = mix_of(tests['mix_snareL_amb'])
    cut = 48 * 2000; h = 576
    for lib, nm in ((new, 'new'), (head, 'HEAD')):
        for a, b in ((0, 2), (1, 2), (2, 1), (2, 0)):
            o = switch_run(lib, L, R, cut, a, b)
            if b == 0:
                continue
            z = max(float(np.max(np.abs(o[3][:h]))), float(np.max(np.abs(o[4][:h]))))
            nz = max(float(np.max(np.abs(o[3][h:h + 4800]))), float(np.max(np.abs(o[4][h:h + 4800]))))
            log('restart %s %s -> %s: max |Ls,Rs| over first %d samples %g (after them %.3g)'
                % (nm, ('OFF', 'PASSIVE', 'ADAPTIVE')[a], ('OFF', 'PASSIVE', 'ADAPTIVE')[b], h, z, nz))


def switch_run(lib, L, R, cut, sm_a, sm_b):
    return switch_run_p(lib, L, R, cut, sm_a, sm_b, DEFP)


def switch_run_p(lib, L, R, cut, sm_a, sm_b, params):
    pa = np.array(params, np.float32)
    L = np.ascontiguousarray(L, np.float32); R = np.ascontiguousarray(R, np.float32)
    outs = [np.zeros(L.size, np.float32) for _ in range(5)]
    lib.h_run_switch(ptr(pa), 1, sm_a, sm_b, cut, 48, ptr(L), ptr(R), L.size, *[ptr(o) for o in outs])
    return [o[cut:] for o in outs]


def c_stems(new, stems, cm=1, sm=2, params=None):
    pa = np.array(params or DEFP, np.float32)
    L, R = mix_of(stems)
    N = L.size; names = list(stems); Sn = len(names)
    sl = np.ascontiguousarray(np.stack([stems[k][0] for k in names]), np.float32)
    sr = np.ascontiguousarray(np.stack([stems[k][1] for k in names]), np.float32)
    sls = np.zeros((Sn, N), np.float32); srs = np.zeros((Sn, N), np.float32)
    ols = np.zeros(N, np.float32); ors = np.zeros(N, np.float32)
    tr = np.zeros(4 * ((N + 47) // 48), np.float32)
    new.h_run_stems(ptr(pa), cm, sm, 48, ptr(L), ptr(R), N, Sn, ptr(sl), ptr(sr), ptr(sls), ptr(srs),
                    ptr(ols), ptr(ors), ptr(tr))
    return names, sls, srs, ols, ors, tr.reshape(-1, 4)


def c_metrics(new, tests, log):
    """run_side metrics measured from the compiled C (stem-through inside the C)."""
    rows = []
    for tname, stems in tests.items():
        names, sls, srs, ols, ors, tr = c_stems(new, stems)
        lin = float(np.max(np.abs(sls.sum(0) - ols)) / (np.max(np.abs(ols)) + 1e-12))
        for k, sname in enumerate(names):
            sl, sr, kind, on = stems[sname]
            fl, fr = band(sl), band(sr)
            ref = max(p(fl[WARM:]), p(fr[WARM:]))
            pl, pr = p(sls[k][WARM:]), p(srs[k][WARM:])
            row = dict(config='C_firmware', test=tname, stem=sname, kind=kind, ls_pow=pl, rs_pow=pr,
                       ls_db=db(pl / ref), rs_db=db(pr / ref), tot_db=db((pl + pr) / ref))
            if on is not None and len(on):
                fe = max(win_energy(fl, on, 0), win_energy(fr, on, 0))
                row['ons_ls_db'] = db(win_energy(sls[k], on, D) / fe)
                row['ons_rs_db'] = db(win_energy(srs[k], on, D) / fe)
            if kind == 'ambience':
                row['mot_ls_std'], row['mot_ls_step'] = RS.level_motion(sls[k])
                row['mot_rs_std'], row['mot_rs_step'] = RS.level_motion(srs[k])
            rows.append(row)
        r = dict(config='C_firmware', test=tname, stem='_mix', kind='mix', lin_err=lin)
        for ch, nm in ((0, 'ls'), (1, 'rs')):
            g = tr[:, ch].astype(np.float64); w = tr[:, 2 + ch].astype(np.float64)
            t = db(g * g * (1 + w * w) + 1e-20)[WARM // 48:]
            r['pump_%s_std' % nm] = float(np.std(t))
            r['pump_%s_rate' % nm] = float(np.sqrt(np.mean((t[10:] - t[:-10]) ** 2)))
        rows.append(r)
    return rows


def main():
    os.makedirs(OUT, exist_ok=True)
    lines = []

    def log(s):
        print(s, flush=True); lines.append(s)
    head_c = subprocess.check_output(['git', '-C', REPO, 'show', 'HEAD:firmware/DSPi/upmix.c']).decode()
    head_h = subprocess.check_output(['git', '-C', REPO, 'show', 'HEAD:firmware/DSPi/upmix.h']).decode()
    new = build(open(os.path.join(FW, 'upmix.c')).read(), open(os.path.join(FW, 'upmix.h')).read(), True)
    head = build(head_c, head_h, False)
    tests = S.build_tests()
    tests['alternating'] = RS.alternating_test()
    head_model_check(head, tests, log)
    head_equivalence(new, head, tests, log)
    latency(new, head, log)
    model_compare(new, tests, log)
    reset_checks(new, head, tests, log)
    log('Acceptance matrix (new C vs HEAD; per side Ls/Rs dB re front, (c) = centre stem):')
    f1, worst, _ = A.matrix(sys.modules[__name__], new, tests, log)
    log('Start-up fade:')
    f2 = A.fade_check(sys.modules[__name__], new, log)
    f3 = A.motion_check(sys.modules[__name__], new, log)
    log('ACCEPTANCE: %d failing cells (matrix %d, fade %d, motion %d); worst stem margin vs HEAD %+.2f dB'
        % (f1 + f2 + f3, f1, f2, f3, worst))
    rows = c_metrics(new, tests, log)
    log('C stem-through linearity worst %.3g' % max(r.get('lin_err', 0) for r in rows))
    # baseline and model rows from run_side for the same table
    K = m.compute_coeffs()
    ref = []
    for name, fl, po in RS.CONFIGS:
        if name in ('baseline', 'c12346_fb', 'C_p1_B2_k05'):
            ref += RS.evaluate(name, fl, po, tests, K, ())
    ref += port_model_rows(tests, new)
    allr = ref + rows
    RS.derive(allr)
    keys = sorted({k for r in allr for k in r})
    with open(os.path.join(OUT, 'harness_metrics.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, keys); w.writeheader(); w.writerows(allr)
    log(RS.summary(allr))
    open(os.path.join(OUT, 'harness_report.txt'), 'w').write('\n'.join(lines) + '\n')


def port_model_rows(tests, new):
    """port_engine through the same stem-through metrics, for the side-by-side row."""
    K, X = model_KX(c_coeffs(new))
    rows = []
    for tname, stems in tests.items():
        L, R = mix_of(stems)
        out, rec = m.port_run(L, R, K, X)
        for sname, (sl, sr, kind, on) in stems.items():
            so = m.port_replay(sl, sr, K, X, rec)
            fl, fr = band(sl), band(sr)
            ref = max(p(fl[WARM:]), p(fr[WARM:]))
            pl, pr = p(so[3][WARM:]), p(so[4][WARM:])
            row = dict(config='port_model', test=tname, stem=sname, kind=kind, ls_pow=pl, rs_pow=pr,
                       ls_db=db(pl / ref), rs_db=db(pr / ref), tot_db=db((pl + pr) / ref))
            if on is not None and len(on):
                fe = max(win_energy(fl, on, 0), win_energy(fr, on, 0))
                row['ons_ls_db'] = db(win_energy(so[3], on, D) / fe)
                row['ons_rs_db'] = db(win_energy(so[4], on, D) / fe)
            if kind == 'ambience':
                row['mot_ls_std'], row['mot_ls_step'] = RS.level_motion(so[3])
                row['mot_rs_std'], row['mot_rs_step'] = RS.level_motion(so[4])
            rows.append(row)
        r = dict(config='port_model', test=tname, stem='_mix', kind='mix')
        F = m.make_flags(resid=1)
        for ch, nm in ((0, 'ls'), (1, 'rs')):
            from run_eval import gain_trace
            t = gain_trace(rec, F, ch)[WARM // m.BLOCK:]
            r['pump_%s_std' % nm] = float(np.std(t))
            r['pump_%s_rate' % nm] = float(np.sqrt(np.mean((t[10:] - t[:-10]) ** 2)))
        rows.append(r)
    return rows


if __name__ == '__main__':
    main()
