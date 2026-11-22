"""Bit-exact check of upmix_begin()/upmix_render() against the committed engine, plus profile selection.

Runs the committed upmix.c (HEAD, or the rev given as argv[1]) through
upmix_process_block and the working tree through begin/render at several chunk
sizes, over every centre and surround mode, mid-stream mode switches, packet
sizes and parameter sets. Every generated channel must match bit for bit.
Needs only numpy. Usage (from tools/upmix_proto): python3 harness/split_check.py [rev]
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hbuild import DEFP, build, ptr, head_sources, tree_sources   # noqa: E402

FS = 48000
N = FS * 3 // 2
CHUNKS = (1, 7, 32, 192)
BLOCKS = (48, 44, 192, 37)
VARIANTS = {
    'default': DEFP,
    'width0_pres+6': DEFP[:1] + [0.0] + DEFP[2:10] + [6.0],
    'width100_pres-9': DEFP[:1] + [100.0] + DEFP[2:10] + [-9.0],
    'dly0_hpf80_lpf12k': DEFP[:6] + [0.0, 80.0, 12000.0] + DEFP[9:],
    'dly20_decorr0_fast': [60.0, 25.0, 10.0, 1.0, 5.0, 50.0, 20.0, 300.0, 7000.0, 0.0, 3.0],
}
# (cm_a, sm_a, cm_b, sm_b): steady modes, then switches that exercise the
# surround restart, the centre OFF ramp and the rear fade paths.
SWITCHES = [(cm, sm, cm, sm) for cm in (0, 1, 2) for sm in (0, 1, 2)] + [
    (1, 0, 1, 2), (1, 2, 1, 1), (1, 1, 1, 0), (1, 2, 2, 2), (2, 2, 1, 2), (0, 1, 1, 2)]


def signal():
    """Centre voice-like tone bursts, a hard-left part, diffuse ambience and transients."""
    rng = np.random.default_rng(11)
    t = np.arange(N) / FS
    voice = 0.3 * np.sin(2 * np.pi * 220 * t) * (np.sin(2 * np.pi * 1.5 * t) > 0)
    left = 0.2 * rng.standard_normal(N) * (np.sin(2 * np.pi * 0.7 * t) > 0.3)
    amb_l, amb_r = 0.05 * rng.standard_normal(N), 0.05 * rng.standard_normal(N)
    hits = np.zeros(N)
    hits[::9600] = 0.9
    L = voice + left + amb_l + hits
    R = voice + amb_r - 0.5 * hits
    return L.astype(np.float32), R.astype(np.float32)


def outs():
    return [np.zeros(N, np.float32) for _ in range(5)]


def main():
    rev = sys.argv[1] if len(sys.argv) > 1 else 'HEAD'
    head = build(*head_sources(rev), False)
    new = build(*tree_sources(), True)
    if not hasattr(new, 'h_run_split'):
        sys.exit('working-tree upmix.h has no UPMIX_CHUNK: nothing to check')
    L, R = signal()
    cut = N // 2 + 13
    runs = fails = 0
    for vname, pv in VARIANTS.items():
        pa = np.array(pv, np.float32)
        for cm_a, sm_a, cm_b, sm_b in SWITCHES:
            for blk in BLOCKS:
                ref = outs()
                head.h_run_ref(ptr(pa), cm_a, sm_a, cm_b, sm_b, cut, blk, ptr(L), ptr(R), N,
                               *[ptr(o) for o in ref])
                trials = [('bridge', None)] + [('chunk%d' % c, c) for c in CHUNKS]
                for tname, chunk in trials:
                    got = outs()
                    if chunk is None:
                        new.h_run_ref(ptr(pa), cm_a, sm_a, cm_b, sm_b, cut, blk, ptr(L), ptr(R), N,
                                      *[ptr(o) for o in got])
                    else:
                        new.h_run_split(ptr(pa), cm_a, sm_a, cm_b, sm_b, cut, blk, chunk,
                                        ptr(L), ptr(R), N, *[ptr(o) for o in got])
                    runs += 1
                    bad = [k for k in range(5) if not np.array_equal(ref[k].view(np.uint32),
                                                                       got[k].view(np.uint32))]
                    if bad:
                        fails += 1
                        print('MISMATCH %s modes=%s blk=%d %s channels=%s' %
                              (vname, (cm_a, sm_a, cm_b, sm_b), blk, tname, bad))
    print('%d runs against %s, %d mismatches' % (runs, rev, fails))
    if hasattr(new, 'h_select_check'):
        bad = new.h_select_check()
        print('profile selection: %d failures' % bad)
        fails += bad
        p0 = np.array(VARIANTS['default'], np.float32)
        p1 = np.array(VARIANTS['dly0_hpf80_lpf12k'], np.float32)
        for blk in (48, 192):
            bad = new.h_profile_switch_check(ptr(p0), ptr(p1), blk, ptr(L), ptr(R), N, blk * (N // blk // 2))
            print('profile switch restarts from reset (blk %d): %d channel mismatches' % (blk, bad))
            fails += bad
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
