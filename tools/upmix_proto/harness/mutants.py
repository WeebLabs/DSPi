"""Seeded faults in the working-tree upmix.c that split_check's route check must catch.

Each mutant is one textual edit; a mutant that passes every route cell means the
check has a blind spot. Needs only numpy. Usage (from tools/upmix_proto):
python3 harness/mutants.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hbuild import build, tree_sources   # noqa: E402
from split_check import route_check      # noqa: E402

MUTANTS = {
    'no restore term': ('dst[i] += (1.0f - w) * r;', 'dst[i] += 0.0f * r;'),
    'second side row dropped': ('float r = ns > 1 ? g0 * s0[i] + g1 * s1[i] : (ns ? g0 * s0[i] : 0.0f);',
                                'float r = ns ? g0 * s0[i] : 0.0f;'),
    'claim ignores fading gains': ('                live |= ur.e[g][o] != 0.0f;', '                (void)0;'),
    'park before the fade ends': ('if (claimed || (ur.active && (ur.latch.claim_mask & out_en))) {',
                                  'if (ur.active && (ur.latch.claim_mask & out_en)) {'),
    'stale latch': ('if (ur.active && sel->seq != ur.latch.seq) ur.latch = *sel;', ';'),
    'no hard park': ('if (ur.held && (n_in != ur.n_inputs || rate != ur.rate)) {', 'if (0) {'),
    'slew overshoot': ('return v > t ? t : v; }', 'return v; }'),
    'source edit swaps hard': ('                    && sel->src_a == ur.latch.src_a && sel->src_b == ur.latch.src_b;',
                               ';'),
    'engine keeps running unrouted': ('if (claimed || (ur.active && (ur.latch.claim_mask & out_en))) {',
                                      'if (claimed || ur.active) {'),
    'fade rate': ('1000.0f / (fs * UPMIX_FADE_MS)', '1000.0f / (fs * 9.0f)'),
    'back stops before its fade': ('                back |= ur.e[UPMIX_GEN_BA][o] != 0.0f || ur.e[UPMIX_GEN_BB][o] != 0.0f;',
                                   '                (void)0;'),
    'trim law': ('powf(10.0f, (float)t * (0.5f / 20.0f))', 'powf(10.0f, (float)t * (0.5f / 10.0f))'),
}


def main():
    c, h = tree_sources()
    missed = 0
    for name, (a, b) in MUTANTS.items():
        if c.count(a) != 1:
            sys.exit('mutant "%s": anchor not found once in upmix.c; update this list' % name)
        fails = route_check(build(c.replace(a, b), h, True), quiet=True)
        missed += fails == 0
        print('%-28s %s (%d failing cells)' % (name, 'caught' if fails else 'MISSED', fails))
    sys.exit(1 if missed else 0)


if __name__ == '__main__':
    main()
