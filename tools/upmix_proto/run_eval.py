"""Run every configuration over every test, stem-through, and write out/results.csv.

Usage: python run_eval.py [--sweep] [--nowav]  (interpreter setup in README.md)
"""
import csv
import os
import sys
import numpy as np
from scipy import signal
from scipy.io import wavfile

import upmix_model as m
import signals as S

FS = m.FS
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
WARM = int(1.0 * FS)             # estimator settling excluded from steady metrics
D = 576                          # Haas delay, samples (12 ms)
ONSET_WIN = int(0.050 * FS)

CONFIGS = [
    ('baseline', dict()),
    ('1_resid', dict(resid=1)),
    ('2_gate', dict(gate=1)),
    ('3a_detHP', dict(det=1)),
    ('3b_detBand', dict(det=2)),
    ('4_look', dict(look=1)),
    ('5_mb2', dict(nb=2)),
    ('5_mb3', dict(nb=3)),
    ('6_duck', dict(duck=1)),
    ('c1', dict(resid=1)),
    ('c12', dict(resid=1, gate=1)),
    ('c123', dict(resid=1, gate=1, det=2)),
    ('c1234', dict(resid=1, gate=1, det=2, look=1)),
    ('c12345_2b', dict(resid=1, gate=1, det=2, look=1, nb=2)),
    ('c12345_3b', dict(resid=1, gate=1, det=2, look=1, nb=3)),
    ('c123456', dict(resid=1, gate=1, det=2, look=1, nb=3, duck=1)),
    ('c123456_nodom', dict(resid=1, gate=1, det=2, look=1, nb=3, duck=1, dom=0)),
    ('c12346_nodom', dict(resid=1, gate=1, det=2, look=1, nb=1, duck=1, dom=0)),
    ('c123456_fb', dict(resid=1, gate=1, det=2, look=1, nb=3, duck=1, dom=2)),
    ('c12346_fb', dict(resid=1, gate=1, det=2, look=1, nb=1, duck=1, dom=2)),
    ('c12456_fb', dict(resid=1, gate=1, det=0, look=1, nb=3, duck=1, dom=2)),
    ('c1246_fb_detHP', dict(resid=1, gate=1, det=1, look=1, nb=1, duck=1, dom=2)),
]

_bl = [signal.butter(2, 300, 'high', fs=FS, output='sos'), signal.butter(2, 7000, 'low', fs=FS, output='sos')]


def band(x):
    """Same Butterworth corners as the surround SVFs (bilinear, prewarped: identical response)."""
    return signal.sosfilt(_bl[1], signal.sosfilt(_bl[0], x))


def p(x):
    return float(np.mean(np.asarray(x, np.float64) ** 2)) + 1e-30


def db(x):
    return 10 * np.log10(x)


def win_energy(x, onsets, off):
    return sum(float(np.sum(np.asarray(x[o + off:o + off + ONSET_WIN], np.float64) ** 2))
               for o in onsets if o >= WARM and o + off + ONSET_WIN < len(x)) + 1e-30


def gain_trace(rec, F, ch):
    """Effective surround gain on uncorrelated input, per 1 ms block, power-averaged over bands."""
    g = rec[2 + ch][::m.BLOCK].astype(np.float64)
    w = rec[4 + ch][::m.BLOCK].astype(np.float64)
    fe = (1 + w * w) if F[m.F_RESID] else (0.871 ** 2 + 0.4898 ** 2)
    return db(np.mean(g * g * fe, axis=1) + 1e-20)


def pumping(rec, F):
    out = []
    for ch in range(2):
        t = gain_trace(rec, F, ch)[WARM // m.BLOCK:]
        d10 = t[10:] - t[:-10]
        out.append((float(np.std(t)), float(np.sqrt(np.mean(d10 ** 2)))))
    return max(o[0] for o in out), max(o[1] for o in out)


def evaluate(name, flags, tests, K, pover=None, wav_tests=()):
    F = m.make_flags(**flags)
    P = m.compute_params(F, **(pover or {}))
    rows = []
    for tname, stems in tests.items():
        L = sum(s[0] for s in stems.values()).astype(np.float32)
        R = sum(s[1] for s in stems.values()).astype(np.float32)
        out, rec = m.run(L, R, K, F, P)
        acc_ls = np.zeros(L.size); acc_rs = np.zeros(L.size)
        for sname, (sl, sr, kind, on) in stems.items():
            so = m.replay(sl, sr, K, F, P, rec)
            acc_ls += so[3]; acc_rs += so[4]
            ls, rs = so[3][WARM:], so[4][WARM:]
            fl, fr = band(sl), band(sr)
            ref = max(p(fl[WARM:]), p(fr[WARM:]))
            row = dict(config=name, test=tname, stem=sname, kind=kind,
                       ls_db=db(p(ls) / ref), rs_db=db(p(rs) / ref),
                       sur_pow=p(ls) + p(rs))
            row['leak_db'] = max(row['ls_db'], row['rs_db'])
            if on is not None:
                fe = max(win_energy(fl, on, 0), win_energy(fr, on, 0))
                se = max(win_energy(so[3], on, D), win_energy(so[4], on, D))
                row['onset_db'] = db(se / fe)
            rows.append(row)
        lin = float(np.max(np.abs(acc_ls - out[3])) / (np.max(np.abs(out[3])) + 1e-12))
        pst, prate = pumping(rec, F)
        ls, rs = out[3][WARM:].astype(np.float64), out[4][WARM:].astype(np.float64)
        corr = float(np.sum(ls * rs) / np.sqrt(np.sum(ls * ls) * np.sum(rs * rs) + 1e-30))
        rows.append(dict(config=name, test=tname, stem='_mix', kind='mix', pump_std_db=pst,
                         pump_rate_db10ms=prate, lsrs_corr=corr, lin_err=lin))
        if tname in wav_tests:
            os.makedirs(OUT, exist_ok=True)
            if name == 'baseline':
                wavfile.write(os.path.join(OUT, '%s_in.wav' % tname), int(FS), np.stack([L, R], 1))
            wavfile.write(os.path.join(OUT, '%s_%s_LRCLsRs.wav' % (tname, name)), int(FS),
                          np.stack([out[1], out[2], out[0], out[3], out[4]], 1).astype(np.float32))
    return rows


def add_retention(rows):
    """Ambience/antiphase surround power relative to baseline on the same test."""
    base = {(r['test'], r['stem']): r['sur_pow'] for r in rows if r['config'] == 'baseline' and 'sur_pow' in r}
    for r in rows:
        if r.get('kind') in ('ambience', 'antiphase'):
            r['retention_db'] = db(r['sur_pow'] / base[(r['test'], r['stem'])])
    # Leak improvement net of ambience loss: a plain gain cut scores 0 here.
    leak0 = {(r['test'], r['stem']): r['leak_db'] for r in rows if r['config'] == 'baseline' and 'leak_db' in r}
    amb = {(r['config'], r['test']): r['retention_db'] for r in rows if r.get('stem') == 'amb'}
    for r in rows:
        if r.get('kind') == 'direct' and (r['config'], r['test']) in amb:
            r['net_db'] = r['leak_db'] - leak0[(r['test'], r['stem'])] - amb[(r['config'], r['test'])]


def summary(rows):
    get = {(r['config'], r['test'], r['stem']): r for r in rows}
    cols = [('cen', 'pan000', 'src', 'leak_db'), ('p50', 'pan050', 'src', 'leak_db'),
            ('p75', 'pan075', 'src', 'leak_db'), ('p90', 'pan090', 'src', 'leak_db'),
            ('hardL', 'hardL', 'gtrL', 'leak_db'), ('uncL', 'uncorrLR', 'gtrL', 'leak_db'),
            ('apRet', 'antiphase', 'ap', 'retention_db'),
            ('m1vox', 'mix_vox_gtr_amb', 'vox', 'leak_db'), ('m1gtr', 'mix_vox_gtr_amb', 'gtrL', 'leak_db'),
            ('m1ons', 'mix_vox_gtr_amb', 'gtrL', 'onset_db'), ('m1amb', 'mix_vox_gtr_amb', 'amb', 'retention_db'),
            ('m2kick', 'mix_kick_keysL', 'kick', 'leak_db'), ('m2keys', 'mix_kick_keysL', 'keysL', 'leak_db'),
            ('m2amb', 'mix_kick_keysL', 'amb', 'retention_db'),
            ('m3vox', 'mix_vox_hatsL', 'vox', 'leak_db'), ('m3hat', 'mix_vox_hatsL', 'hatsL', 'leak_db'),
            ('m3amb', 'mix_vox_hatsL', 'amb', 'retention_db'),
            ('m4sn', 'mix_snareL_amb', 'snareL', 'leak_db'), ('m4ons', 'mix_snareL_amb', 'snareL', 'onset_db'),
            ('m4amb', 'mix_snareL_amb', 'amb', 'retention_db'),
            ('n1gtr', 'mix_vox_gtr_amb', 'gtrL', 'net_db'), ('n1vox', 'mix_vox_gtr_amb', 'vox', 'net_db'),
            ('n2keys', 'mix_kick_keysL', 'keysL', 'net_db'), ('n3hat', 'mix_vox_hatsL', 'hatsL', 'net_db'),
            ('n4sn', 'mix_snareL_amb', 'snareL', 'net_db'),
            ('dAmb', 'diffuse', 'amb', 'retention_db'), ('dTail', 'diffuse', 'tail', 'retention_db'),
            ('dCorr', 'diffuse', '_mix', 'lsrs_corr'),
            ('pStd', 'diffuse', '_mix', 'pump_std_db'), ('pRate', 'diffuse', '_mix', 'pump_rate_db10ms'),
            ('m1pStd', 'mix_vox_gtr_amb', '_mix', 'pump_std_db'), ('m1pRt', 'mix_vox_gtr_amb', '_mix', 'pump_rate_db10ms')]
    configs = list(dict.fromkeys(r['config'] for r in rows))
    lines = ['| config | ' + ' | '.join(c[0] for c in cols) + ' |', '|' + '---|' * (len(cols) + 1)]
    for c in configs:
        vals = []
        for _, t, s, k in cols:
            v = get.get((c, t, s), {}).get(k)
            vals.append('' if v is None else ('%.2f' % v if k == 'lsrs_corr' else '%.1f' % v))
        lines.append('| %s | %s |' % (c, ' | '.join(vals)))
    return '\n'.join(lines)


def check_baseline(tests, K):
    """Bit-exact engine vs port, plus the three hand-derived baseline leak figures."""
    F = m.make_flags(); P = m.compute_params(F)
    st = tests['mix_vox_gtr_amb']
    L = sum(s[0] for s in st.values()).astype(np.float32); R = sum(s[1] for s in st.values()).astype(np.float32)
    b = m.baseline_run(L, R, K)
    o, rec = m.run(L, R, K, F, P)
    exact = max(float(np.max(np.abs(b[k] - o[k]))) for k in range(5))
    # centre-feed leak with steering forced fully open: Ls / L' through the band limit
    vl, vr = st['vox'][0], st['vox'][1]
    rec2 = (rec[0], rec[1], np.ones_like(rec[2]), np.ones_like(rec[3]), rec[4], rec[5])
    so = m.replay(vl, vr, K, F, P, rec2)
    cfeed = db(p(so[3][WARM:]) / p(band(so[1])[WARM:]))
    gls = rec[2][WARM:, 0]
    return exact, cfeed, float(np.mean(gls))


def recombination():
    """Equal-gain band sum vs single band, and a 1/0.5/1 gain pattern, through the audio path."""
    K = m.compute_coeffs(center_mode=2, decorr=0.0, sur_delay=0.0)
    n = 8192
    x = np.zeros(n, np.float32); x[0] = 1
    res = {}
    for nb, gains in ((1, (1,)), (2, (1, 1)), (3, (1, 1, 1)), (3, (1, 0.5, 1)), (2, (1, 0.5))):
        F = m.make_flags(resid=1, nb=nb); P = m.compute_params(F)
        rec = (np.zeros(n, np.float32), np.zeros(n, np.float32),
               np.tile(np.array(gains, np.float32), (n, 1)), np.ones((n, nb), np.float32),
               np.zeros((n, nb), np.float32), np.zeros((n, nb), np.float32))
        h = m.replay(x, 0 * x, K, F, P, rec)[3]
        H = np.abs(np.fft.rfft(h.astype(np.float64)))
        res[(nb, gains)] = H
    f = np.fft.rfftfreq(n, 1 / FS)
    sel = (f > 300) & (f < 7000)
    ref = res[(1, (1,))]
    out = {}
    for k, H in res.items():
        r = db((H[sel] / ref[sel]) ** 2)
        out[k] = (float(r.min()), float(r.max()))
    return out


def main():
    sweep = '--sweep' in sys.argv
    tests = S.build_tests()
    K = m.compute_coeffs()
    exact, cfeed, mg = check_baseline(tests, K)
    print('engine(all off) vs baseline_run max abs diff: %g' % exact)
    print('centre feed through open steering (Ls/L\'): %.2f dB; mean gls on mix1: %.3f' % (cfeed, mg))
    print('recombination (min,max dB re single band, 300-7k):', recombination())
    rows = []
    wav = () if '--nowav' in sys.argv else ('mix_vox_gtr_amb', 'mix_kick_keysL', 'mix_snareL_amb', 'mix_vox_hatsL')
    for name, fl in CONFIGS:
        wt = wav if name in ('baseline', 'c123456_fb', 'c12346_fb') else ()
        rows += evaluate(name, fl, tests, K, wav_tests=wt)
        print('done', name, flush=True)
    if sweep:
        full = dict(resid=1, gate=1, det=2, look=1, nb=3, duck=1, dom=2)
        for k, vals in (('gate_p', (0.5, 2.0)), ('tau_w', (20.0, 100.0)), ('wmax', (2.0, 8.0)),
                        ('eps', (1e-2, 1e-4)), ('duck_thr', (2.0, 8.0)), ('duck_q', (1.0,)),
                        ('duck_rel', (30.0, 150.0)), ('xover3', ((600.0, 2000.0), (1000.0, 3500.0))), ('wmin', (0.0, 0.3))):
            for v in vals:
                nm = 'sw_%s=%s' % (k, v if not isinstance(v, tuple) else '/'.join('%g' % x for x in v))
                rows += evaluate(nm, full, tests, K, pover={k: v})
                print('done', nm, flush=True)
    add_retention(rows)
    os.makedirs(OUT, exist_ok=True)
    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(OUT, 'results.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(rows)
    print(summary(rows))
    print('max stem-through linearity error:', max(r.get('lin_err', 0) for r in rows))


if __name__ == '__main__':
    main()
