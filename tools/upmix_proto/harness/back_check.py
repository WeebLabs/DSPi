"""Back pair (upmixer v2 phase 4) checks on the working-tree engine: chunk invariance,
arrival timing, hard-pan leak, snare onset, side-to-back decorrelation, the
mirrored allpass signs, the back fade, and the 7 ms fallback the spec names.
Needs only numpy. Usage (from tools/upmix_proto): python3 harness/back_check.py
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hbuild import DEFP, build, ptr, tree_sources   # noqa: E402

FS = 48000
SA, SB, BA, BB = 3, 4, 5, 6
PASSIVE, ADAPTIVE, C_OFF = 1, 2, 2


def run(lib, L, R, cm=1, sm=ADAPTIVE, back_ms=2.0, blk=48, chunk=32, p=DEFP):
    pa = np.array(p, np.float32)
    out = np.zeros((7, L.size), np.float32)
    bd = lib.h_run_back(ptr(pa), cm, sm, back_ms, blk, chunk, ptr(L), ptr(R), L.size, ptr(out))
    return out, bd


def db(x):
    return 10 * np.log10(np.mean(np.asarray(x, np.float64) ** 2) + 1e-30)


def max_xcorr(a, b, max_lag):
    a = a - a.mean(); b = b - b.mean()
    na, nb = np.sqrt(np.dot(a, a)), np.sqrt(np.dot(b, b))
    best = 0.0
    for lag in range(-max_lag, max_lag + 1):
        x, y = (a[lag:], b[:b.size - lag]) if lag >= 0 else (a[:a.size + lag], b[-lag:])
        best = max(best, abs(float(np.dot(x, y))) / (na * nb + 1e-30))
    return best


def variant(src_c, src_h, edits):
    for a, b in edits:
        assert src_c.count(a) + src_h.count(a) == 1, a
        src_c, src_h = src_c.replace(a, b), src_h.replace(a, b)
    return build(src_c, src_h, True)


def main():
    c, h = tree_sources()
    lib = build(c, h, True)
    rng = np.random.default_rng(7)
    fails = []

    def check(ok, what):
        print('  %s %s' % ('ok  ' if ok else 'FAIL', what))
        if not ok:
            fails.append(what)

    n = FS * 2
    amb_l, amb_r, mid = (rng.standard_normal(n).astype(np.float32) for _ in range(3))
    L = (0.1 * amb_l + 0.05 * mid).astype(np.float32)
    R = (0.1 * amb_r + 0.05 * mid).astype(np.float32)

    print('chunk invariance (all seven channels, bit for bit)')
    for sm in (PASSIVE, ADAPTIVE):
        ref, _ = run(lib, L, R, sm=sm, chunk=48)
        bad = [k for k in (1, 7, 32) if not np.array_equal(run(lib, L, R, sm=sm, chunk=k)[0].view(np.uint32),
                                                           ref.view(np.uint32))]
        check(not bad, 'surround mode %d: chunks 1, 7, 32 match the whole packet %s' % (sm, bad or ''))

    print('arrival: back main tap lands back_offset after the sides (passive, impulse)')
    imp = np.zeros(n, np.float32); imp[FS // 2] = 1.0
    zero = np.zeros(n, np.float32)
    for off, want_ms in ((0.0, 0.0), (2.0, 2.0), (10.0, None)):
        out, bd = run(lib, imp, zero, cm=C_OFF, sm=PASSIVE, back_ms=off)
        t_side = int(np.argmax(np.abs(out[SA]))) - FS // 2
        t_back = int(np.argmax(np.abs(out[BA]))) - FS // 2
        got = (t_back - t_side) / 48.0
        if want_ms is None:   # clamped to the ring: back tap 1023, about 4.5 ms after the sides
            check(bd == 1023 and abs(got - (1023 + 251 - t_side) / 48.0) < 0.05,
                  'offset 10 ms clamps the tap to 1023: back %.2f ms after the sides' % got)
        else:
            check(abs(got - want_ms) < 0.1, 'offset %g ms: back %.2f ms after the sides' % (off, got))

    print('hard-pan leak (adaptive)')
    hl = (0.3 * rng.standard_normal(n)).astype(np.float32)
    out, _ = run(lib, hl, zero)
    tail = slice(FS, n)
    lv = {k: db(out[k][tail]) - db(hl[tail]) for k in (SA, SB, BA, BB)}
    check(max(lv.values()) < -80.0, 'hard left: SA %.0f, SB %.0f, BA %.0f, BB %.0f dB, all below -80'
          % (lv[SA], lv[SB], lv[BA], lv[BB]))
    ll = (0.3 * rng.standard_normal(n)).astype(np.float32)
    rr = (0.1 * rng.standard_normal(n)).astype(np.float32)
    out, _ = run(lib, ll, rr)
    d_a, d_b = db(out[BA][tail]) - db(out[SA][tail]), db(out[BB][tail]) - db(out[SB][tail])
    check(abs(d_a) < 1.5 and abs(d_b) < 1.5,
          'left-leaning uncorrelated: back minus side %+.2f dB (A), %+.2f dB (B)' % (d_a, d_b))

    print('snare onset: duck depth at the back tap against the side tap (adaptive)')
    t = np.arange(n) / FS
    bed_l, bed_r = 0.05 * rng.standard_normal(n), 0.05 * rng.standard_normal(n)
    sn_l, sn_r = np.zeros(n), np.zeros(n)
    hits = list(range(FS // 2, n - FS // 4, FS // 4))
    for h0 in hits:
        k = np.arange(int(0.08 * FS))
        env = np.minimum(k / 48.0, 1.0) * np.exp(-k / (0.025 * FS))
        sn_l[h0:h0 + k.size] += 0.6 * env * rng.standard_normal(k.size)
        sn_r[h0:h0 + k.size] += 0.6 * env * rng.standard_normal(k.size)
    Ls, Rs = (bed_l + sn_l).astype(np.float32), (bed_r + sn_r).astype(np.float32)
    out, bd = run(lib, Ls, Rs)
    hd = int(round(DEFP[6] * 48))

    def onset_ratio(x, d):
        on = [db(x[h0 + d:h0 + d + 480]) for h0 in hits]
        bed = [db(x[h0 + d + 7200:h0 + d + 9600]) for h0 in hits]
        return float(np.mean(on) - np.mean(bed))
    # Windows start at each pair's allpass main tap (10 ms sides, 251 samples backs).
    r_side, r_back = onset_ratio(out[SA], hd + 480), onset_ratio(out[BA], bd + 251)
    check(r_back - r_side < 1.5, 'onset over bed: side %+.1f dB, back %+.1f dB (back within 1.5 dB)'
          % (r_side, r_back))

    print('side-to-back decorrelation (adaptive, diffuse input), max |r| over +-30 ms')
    win = slice(FS // 2, n)

    def deco(lb):
        o, _ = run(lb, L, R)
        return max_xcorr(o[SA][win], o[BA][win], 1440), max_xcorr(o[BA][win], o[BB][win], 1440)
    r_fw, r_bb = deco(lib)
    r_none, _ = deco(variant(c, h, [('babuf[i] = bd0 + ap_g * bw0;', 'babuf[i] = xb0; (void)bw0;')]))
    r_same, _ = deco(variant(c, h, [('float bw0 = xb0 - ap_g * bd0;', 'float bw0 = xb0 + ap_g * bd0;'),
                                    ('babuf[i] = bd0 + ap_g * bw0;', 'babuf[i] = bd0 - ap_g * bw0;')]))
    r_7ms, _ = deco(variant(c, h, [('#define UPMIX_BACK_AP_MS         5.23f', '#define UPMIX_BACK_AP_MS         7.0f'),
                                   ('#define UPMIX_BACK_AP_RING      256', '#define UPMIX_BACK_AP_RING      512'),
                                   ('#define UPMIX_BACK_GAP_MS        4.8f', '#define UPMIX_BACK_GAP_MS        3.03f')]))
    r_side_pair = deco(lib)[1]
    print('    SA-BA: firmware %.3f, no back allpass %.3f, unmirrored signs %.3f, 7 ms fallback %.3f; BA-BB %.3f'
          % (r_fw, r_none, r_same, r_7ms, r_bb))
    check(r_fw < r_none - 0.15, 'the back allpass decorrelates SA from BA (%.3f against %.3f plain)' % (r_fw, r_none))
    check(r_fw <= r_same + 0.02, 'mirrored signs are no worse than unmirrored (%.3f against %.3f)' % (r_fw, r_same))
    check(r_fw <= r_7ms + 0.05, '5.23 ms is no worse than the 7 ms fallback (%.3f against %.3f)' % (r_fw, r_7ms))
    _ = r_side_pair

    print('back fade: emerges from zero at its own tap (adaptive, from reset)')
    for name, lb in (('firmware', lib),
                     ('fade started at the side tap (must fail)',
                      variant(c, h, [('if (back && um.s.fill >= c->back_delay) {', 'if (back) {')]))):
        o, bd = run(lb, L, R)
        nz = int(np.flatnonzero(np.abs(o[BA]) > 0)[0]) if np.any(o[BA]) else -1
        first = db(o[BA][bd:bd + 240]) - db(o[BA][bd + 480:bd + 960])
        ok = first < -15.0
        if name == 'firmware':
            check(ok and nz >= bd, 'first nonzero BA at %d (tap %d), first 5 ms %+.1f dB against 10-20 ms' % (nz, bd, first))
        else:
            check(not ok, '%s: first 5 ms %+.1f dB' % (name, first))

    print('%d failing checks' % len(fails))
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
