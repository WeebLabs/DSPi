"""Acceptance matrix for the ported ADAPTIVE surround against HEAD (imported by harness.py).

HEAD is measured stem-through with the model's engine() at all-off, which is bit-exact
to the HEAD C (checked in harness.py), so both sides use the same stem method.
"""
import re
import numpy as np

import upmix_model as m
from run_eval import band, p, db, win_energy, WARM, D
import run_side as RS

MIXES = (('m1', 'mix_vox_gtr_amb'), ('m2', 'mix_kick_keysL'), ('m3', 'mix_vox_hatsL'), ('m4', 'mix_snareL_amb'))
CENTRE = ('vox', 'kick')
FLOOR_DB = -60.0
# Ls levels the first port reached at defaults; acceptance allows 1 dB back from them.
DEFAULT_TARGETS = (('m1', 'gtrL', 'ls', -9.1), ('m2', 'keysL', 'ls', -21.7), ('m3', 'hatsL', 'ls', -15.6),
                   ('m4', 'snareL', 'onset', -29.4))


def lib_variant(H, src_c, src_h, k=None, q=None):
    h = src_h
    if k is not None:
        h = re.sub(r'(#define UPMIX_GATE_K\s+)[0-9.]+f', r'\g<1>%sf' % k, h)
    if q is not None:
        h = re.sub(r'(#define UPMIX_FLOOR_Q\s+)[0-9.]+f', r'\g<1>%sf' % q, h)
    return H.build(src_c, h, True)


def head_stems(H, stems, cm, pv):
    K = m.compute_coeffs(center_mode=cm, width=pv[1]); F = m.make_flags(); P = m.compute_params(F)
    L, R = H.mix_of(stems)
    _, rec = m.run(L, R, K, F, P)
    return {k: m.replay(s[0], s[1], K, F, P, rec)[3:5] for k, s in stems.items()}


def new_stems(H, lib, stems, cm, pv):
    names, sls, srs, *_ = H.c_stems(lib, stems, cm, 2, pv)
    return {k: (sls[i], srs[i]) for i, k in enumerate(names)}


def side_levels(stem, o):
    ref = max(p(band(stem[0])[WARM:]), p(band(stem[1])[WARM:]))
    return db(p(o[0][WARM:]) / ref), db(p(o[1][WARM:]) / ref)


def matrix(H, lib, tests, log, verbose=True):
    """Centre ADAPTIVE/PASSIVE/OFF x width 0/25/100 on m1-m4. Returns (fails, worst margin, defaults)."""
    fails = 0; worst = -99.0; dflt = {}
    for cm, cname in ((1, 'ADAPTIVE'), (0, 'PASSIVE'), (2, 'OFF')):
        for w in (0.0, 25.0, 100.0):
            pv = H.DEFP[:1] + [w] + H.DEFP[2:]
            for tag, t in MIXES:
                st = tests[t]
                hs = head_stems(H, st, cm, pv); ns = new_stems(H, lib, st, cm, pv)
                parts = []; ok = True
                for s, (sl, sr, kind, on) in st.items():
                    if kind != 'direct':
                        continue
                    hl, hr = side_levels(st[s], hs[s]); nl, nr = side_levels(st[s], ns[s])
                    # below -60 dB re front is inaudible; float noise there is not a regression
                    marg = max(nl - hl if nl > FLOOR_DB else -99.0, nr - hr if nr > FLOOR_DB else -99.0)
                    worst = max(worst, marg)
                    good = marg <= 1.0
                    ok &= good
                    lab = s + ('(c)' if s in CENTRE else '')
                    parts.append('%s %.1f/%.1f vs %.1f/%.1f%s' % (lab, nl, nr, hl, hr, '' if good else ' FAIL'))
                    if cm == 1 and w == 25.0:
                        dflt[(tag, s, 'ls')] = nl
                        if on is not None and len(on):
                            fl, fr = band(sl), band(sr)
                            fe = max(win_energy(fl, on, 0), win_energy(fr, on, 0))
                            dflt[(tag, s, 'onset')] = db(win_energy(ns[s][0], on, D) / fe)
                ha = p(hs['amb'][0][WARM:]) + p(hs['amb'][1][WARM:])
                na = p(ns['amb'][0][WARM:]) + p(ns['amb'][1][WARM:])
                amb = db(na / ha)
                good = amb <= 2.0
                ok &= good
                parts.append('amb %+.1f%s' % (amb, '' if good else ' FAIL'))
                fails += 0 if ok else 1
                if verbose:
                    log('  %-4s %-8s w%-3d %s: %s  [%s]' % (tag, cname, w, t, '; '.join(parts), 'PASS' if ok else 'FAIL'))
    for tag, s, kind, target in DEFAULT_TARGETS:
        v = dflt.get((tag, s, kind))
        good = v is not None and v <= target + 1.0
        fails += 0 if good else 1
        if verbose:
            log('  default %s %s %s: %.1f (first port %.1f, limit %.1f) [%s]'
                % (tag, s, kind, v, target, target + 1.0, 'PASS' if good else 'FAIL'))
    return fails, worst, dflt


def motion_check(H, lib, log):
    """Ambience level motion per side on the alternating test, new against HEAD."""
    st = RS.alternating_test()
    hs = head_stems(H, st, 1, H.DEFP); ns = new_stems(H, lib, st, 1, H.DEFP)
    h = (RS.level_motion(hs['amb'][0])[0], RS.level_motion(hs['amb'][1])[0])
    n = (RS.level_motion(ns['amb'][0])[0], RS.level_motion(ns['amb'][1])[0])
    # 1 dB tolerance, as for the stem rule: the metric carries its own estimation spread.
    good = n[0] <= h[0] + 1.0 and n[1] <= h[1] + 1.0
    log('  alternating ambience motion std Ls/Rs: new %.2f/%.2f, HEAD %.2f/%.2f [%s]'
        % (n[0], n[1], h[0], h[1], 'PASS' if good else 'FAIL'))
    return 0 if good else 1


def fade_check(H, lib, log):
    """Rears from reset and after OFF/PASSIVE -> ADAPTIVE on steady diffuse noise: silent, then a ramp from zero."""
    rng = np.random.default_rng(11)
    N = 48000
    L = (rng.standard_normal(N) * 0.1).astype(np.float32); R = (rng.standard_normal(N) * 0.1).astype(np.float32)
    fails = 0
    for dly in (0.0, 1.0, 12.0, 20.0):
        pv = H.DEFP[:6] + [dly] + H.DEFP[7:]
        hd = int(dly * 48 + 0.5)
        runs = [('reset', H.c_run(lib, L, R, 1, 2, pv))]
        for a in (0, 1):
            o = H.switch_run_p(lib, L, R, 9600, a, 2, pv)
            runs.append((('OFF', 'PASSIVE')[a] + '->ADAPTIVE', o))
        for nm, o in runs:
            ls, rs = o[3].astype(np.float64), o[4].astype(np.float64)
            steady = np.sqrt(np.mean(ls[24000:] ** 2))
            first = []
            for x in (ls, rs):
                nz = np.nonzero(x)[0]
                first.append(int(nz[0]) if nz.size else -1)
            f0 = first[0]
            pre_silent = f0 >= hd
            s0 = abs(ls[f0]) / steady
            w5 = db(np.mean(ls[f0:f0 + 240] ** 2) / steady ** 2)
            w10 = db(np.mean(ls[f0 + 480:f0 + 960] ** 2) / steady ** 2)
            good = pre_silent and first[0] == first[1] and s0 < 0.02 and w5 < -6.0
            fails += 0 if good else 1
            log('  fade dly=%4.1f %-17s first nonzero Ls/Rs %d/%d (delay %d), first |x| %.4f of rms, '
                'first 5 ms %.1f dB, 10-20 ms %.1f dB [%s]' % (dly, nm, first[0], first[1], hd, s0, w5, w10,
                                                                 'PASS' if good else 'FAIL'))
    return fails
