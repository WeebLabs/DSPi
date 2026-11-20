"""Side-specific gate and balance-relaxed floor round. Writes out/side_results.csv, out/side_summary.txt.

Usage: <python with numpy+scipy+numba> run_side.py [--nowav]
"""
import csv
import os
import sys
import numpy as np
from scipy.io import wavfile

import upmix_model as m
import signals as S
from run_eval import band, p, db, win_energy, gain_trace, WARM, D, OUT, FS

B = dict(resid=1, gate=1, det=2, look=1, nb=1, duck=1, dom=2)
H = dict(B, det=1)
CONFIGS = [('baseline', dict(), {}), ('c12346_fb', B, {}), ('c1246_fb_detHP', H, {})]
for tag, base in (('', B), ('_detHP', H)):
    CONFIGS += [
        ('A_p1' + tag, dict(base, side=1), {}),
        ('A_p2' + tag, dict(base, side=1), dict(gate_p=2.0)),
        ('B1_q1' + tag, dict(base, floor=1), {}),
        ('B2_q1' + tag, dict(base, floor=2), {}),
        ('C_p1_B2' + tag, dict(base, side=1, floor=2), {}),
        ('C_p2_B2' + tag, dict(base, side=1, floor=2), dict(gate_p=2.0)),
        ('C_p2_B1' + tag, dict(base, side=1, floor=1), dict(gate_p=2.0)),
        ('C_p1_B2_k05' + tag, dict(base, side=1, floor=2), dict(side_k=0.5)),
    ]
SWEEP = [('sw_C_p%g_q%g' % (gp, q), dict(B, side=1, floor=2), dict(gate_p=gp, floor_q=q))
         for gp in (1.0, 2.0, 3.0) for q in (0.5, 1.0, 2.0, 4.0)]
SWEEP += [('sw_C_p%g_q1_k%g' % (gp, k), dict(B, side=1, floor=2), dict(gate_p=gp, side_k=k))
          for gp in (1.0, 2.0) for k in (0.25, 0.5)]


def alternating_test():
    """Call-and-response plucks, 1 s hard left then 1 s hard right, over steady ambience."""
    g, on = S.plucks(90, period=0.2)
    g = S.norm(g, -12)
    sec = int(FS)
    left = (np.arange(g.size) // sec) % 2 == 0
    al, ar = S.amb_noise(91)
    ampl = 10 ** (-24 / 20)
    al = al * ampl / S.rms(al); ar = ar * ampl / S.rms(ar)
    onl = np.array([o for o in on if left[o]]); onr = np.array([o for o in on if not left[o]])
    return {'callL': (g * left, 0 * g, 'direct', onl), 'callR': (0 * g, g * ~left, 'direct', onr),
            'amb': (al, ar, 'ambience', None)}


def level_motion(x):
    """Ambience level std (dB) and RMS step between adjacent 50 ms windows (dB)."""
    w = int(0.05 * FS)
    x = np.asarray(x[WARM:], np.float64)
    e = db(np.mean(x[:x.size // w * w].reshape(-1, w) ** 2, axis=1) + 1e-30)
    return float(np.std(e)), float(np.sqrt(np.mean(np.diff(e) ** 2)))


def evaluate(name, flags, pover, tests, K, wav_tests):
    F = m.make_flags(**flags)
    P = m.compute_params(F, **pover)
    rows = []
    for tname, stems in tests.items():
        L = sum(s[0] for s in stems.values()).astype(np.float32)
        R = sum(s[1] for s in stems.values()).astype(np.float32)
        out, rec = m.run(L, R, K, F, P)
        for sname, (sl, sr, kind, on) in stems.items():
            so = m.replay(sl, sr, K, F, P, rec)
            fl, fr = band(sl), band(sr)
            ref = max(p(fl[WARM:]), p(fr[WARM:]))
            pl, pr = p(so[3][WARM:]), p(so[4][WARM:])
            row = dict(config=name, test=tname, stem=sname, kind=kind, ls_pow=pl, rs_pow=pr,
                       ls_db=db(pl / ref), rs_db=db(pr / ref), tot_db=db((pl + pr) / ref))
            if on is not None and len(on):
                fe = max(win_energy(fl, on, 0), win_energy(fr, on, 0))
                row['ons_ls_db'] = db(win_energy(so[3], on, D) / fe)
                row['ons_rs_db'] = db(win_energy(so[4], on, D) / fe)
            if kind == 'ambience':
                row['mot_ls_std'], row['mot_ls_step'] = level_motion(so[3])
                row['mot_rs_std'], row['mot_rs_step'] = level_motion(so[4])
            rows.append(row)
        r = dict(config=name, test=tname, stem='_mix', kind='mix')
        for ch, nm in ((0, 'ls'), (1, 'rs')):
            t = gain_trace(rec, F, ch)[WARM // m.BLOCK:]
            r['pump_%s_std' % nm] = float(np.std(t))
            r['pump_%s_rate' % nm] = float(np.sqrt(np.mean((t[10:] - t[:-10]) ** 2)))
        rows.append(r)
        if tname in wav_tests:
            os.makedirs(OUT, exist_ok=True)
            if name == 'baseline' and tname == 'alternating':
                wavfile.write(os.path.join(OUT, 'alternating_in.wav'), int(FS), np.stack([L, R], 1))
            wavfile.write(os.path.join(OUT, '%s_%s_LRCLsRs.wav' % (tname, name)), int(FS),
                          np.stack([out[1], out[2], out[0], out[3], out[4]], 1).astype(np.float32))
    return rows


def derive(rows):
    """Retention per side and total vs baseline; net = leak change minus ambience change, per side and total."""
    base = {(r['test'], r['stem']): r for r in rows if r['config'] == 'baseline' and r['kind'] != 'mix'}
    for r in rows:
        if r['kind'] in ('ambience', 'antiphase'):
            b = base[(r['test'], r['stem'])]
            r['ret_ls'] = db(r['ls_pow'] / b['ls_pow']); r['ret_rs'] = db(r['rs_pow'] / b['rs_pow'])
            r['ret_tot'] = db((r['ls_pow'] + r['rs_pow']) / (b['ls_pow'] + b['rs_pow']))
    amb = {(r['config'], r['test']): r for r in rows if r['stem'] == 'amb'}
    for r in rows:
        a = amb.get((r['config'], r['test']))
        if r['kind'] == 'direct' and a is not None:
            b = base[(r['test'], r['stem'])]
            r['net_ls'] = r['ls_db'] - b['ls_db'] - a['ret_ls']
            r['net_rs'] = r['rs_db'] - b['rs_db'] - a['ret_rs']
            r['net_tot'] = r['tot_db'] - b['tot_db'] - a['ret_tot']


COLS = [('cen', 'pan000', 'src', 'tot_db'), ('p25', 'pan025', 'src', 'tot_db'), ('p50', 'pan050', 'src', 'tot_db'),
        ('hardL Ls/Rs', 'hardL', 'gtrL', 'ls_db rs_db'), ('uncL Ls/Rs', 'uncorrLR', 'gtrL', 'ls_db rs_db'),
        ('apRet', 'antiphase', 'ap', 'ret_tot'),
        ('m1vox', 'mix_vox_gtr_amb', 'vox', 'tot_db'), ('m1gtr Ls/Rs', 'mix_vox_gtr_amb', 'gtrL', 'ls_db rs_db'),
        ('m1amb Ls/Rs/T', 'mix_vox_gtr_amb', 'amb', 'ret_ls ret_rs ret_tot'),
        ('m1net Ls/Rs/T', 'mix_vox_gtr_amb', 'gtrL', 'net_ls net_rs net_tot'),
        ('m2keys Ls/Rs', 'mix_kick_keysL', 'keysL', 'ls_db rs_db'),
        ('m2amb Ls/Rs/T', 'mix_kick_keysL', 'amb', 'ret_ls ret_rs ret_tot'),
        ('m2net Ls/Rs/T', 'mix_kick_keysL', 'keysL', 'net_ls net_rs net_tot'),
        ('m3vox', 'mix_vox_hatsL', 'vox', 'tot_db'), ('m3hat Ls/Rs', 'mix_vox_hatsL', 'hatsL', 'ls_db rs_db'),
        ('m3amb Ls/Rs/T', 'mix_vox_hatsL', 'amb', 'ret_ls ret_rs ret_tot'),
        ('m3net Ls/Rs/T', 'mix_vox_hatsL', 'hatsL', 'net_ls net_rs net_tot'),
        ('m4sn Ls/Rs', 'mix_snareL_amb', 'snareL', 'ls_db rs_db'), ('m4ons Ls', 'mix_snareL_amb', 'snareL', 'ons_ls_db'),
        ('m4amb T', 'mix_snareL_amb', 'amb', 'ret_tot'),
        ('dAmb T', 'diffuse', 'amb', 'ret_tot'),
        ('dPump Ls/Rs', 'diffuse', '_mix', 'pump_ls_std pump_rs_std'),
        ('m1Pump Ls/Rs', 'mix_vox_gtr_amb', '_mix', 'pump_ls_std pump_rs_std'),
        ('altCallL Ls/Rs', 'alternating', 'callL', 'ls_db rs_db'),
        ('altAmb Ls/Rs/T', 'alternating', 'amb', 'ret_ls ret_rs ret_tot'),
        ('altAmbMot std Ls/Rs', 'alternating', 'amb', 'mot_ls_std mot_rs_std'),
        ('altAmbMot step Ls/Rs', 'alternating', 'amb', 'mot_ls_step mot_rs_step'),
        ('altPump Ls/Rs', 'alternating', '_mix', 'pump_ls_std pump_rs_std'),
        ('altPumpRate Ls/Rs', 'alternating', '_mix', 'pump_ls_rate pump_rs_rate')]


def summary(rows):
    get = {(r['config'], r['test'], r['stem']): r for r in rows}
    configs = list(dict.fromkeys(r['config'] for r in rows))
    out = ['| config | ' + ' | '.join(c[0] for c in COLS) + ' |', '|' + '---|' * (len(COLS) + 1)]
    for c in configs:
        vals = []
        for _, t, s, ks in COLS:
            r = get.get((c, t, s), {})
            vals.append('/'.join('%.1f' % r[k] if k in r else '' for k in ks.split()))
        out.append('| %s | %s |' % (c, ' | '.join(vals)))
    return '\n'.join(out)


def main():
    tests = S.build_tests()
    tests['alternating'] = alternating_test()
    K = m.compute_coeffs()
    wav = () if '--nowav' in sys.argv else ('mix_vox_gtr_amb', 'mix_kick_keysL', 'alternating')
    rows = []
    for name, fl, po in CONFIGS + SWEEP:
        wt = wav if name in ('baseline', 'c12346_fb', 'C_p1_B2_k05') else ()
        rows += evaluate(name, fl, po, tests, K, wt)
    derive(rows)
    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(OUT, 'side_results.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(rows)
    s = summary(rows)
    open(os.path.join(OUT, 'side_summary.txt'), 'w').write(s + '\n')
    print(s)


if __name__ == '__main__':
    main()
