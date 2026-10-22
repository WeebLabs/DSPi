#!/usr/bin/env python3
"""Compare two interleaved float32 decodes (ours vs an ffmpeg reference).

Both files hold frames of `channels` floats.  Prints per-channel peak
difference and SNR and exits non-zero when any channel's SNR is below the
threshold, so a port regression fails run.sh.  No numpy on purpose.

Usage: compare.py ours.f32 ref.f32 --channels 6 [--min-snr 80]
"""

import argparse
import array
import math
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ours")
    ap.add_argument("ref")
    ap.add_argument("--channels", type=int, default=6)
    ap.add_argument("--min-snr", type=float, default=80.0)
    a = ap.parse_args()

    ours = array.array("f")
    ref = array.array("f")
    with open(a.ours, "rb") as f:
        ours.frombytes(f.read())
    with open(a.ref, "rb") as f:
        ref.frombytes(f.read())
    if sys.byteorder != "little":
        ours.byteswap()
        ref.byteswap()

    ch = a.channels
    n = min(len(ours), len(ref)) // ch
    if n == 0:
        print("no overlapping samples")
        return 1
    if len(ours) != len(ref):
        print("length differs: ours %d ref %d samples (comparing %d frames)"
              % (len(ours) // ch, len(ref) // ch, n))

    worst = 0.0
    fail = False
    for c in range(ch):
        err = 0.0
        sig = 0.0
        peak = 0.0
        for i in range(n):
            o = ours[i * ch + c]
            r = ref[i * ch + c]
            d = o - r
            err += d * d
            sig += r * r
            if abs(d) > peak:
                peak = abs(d)
        if sig == 0.0 and err == 0.0:
            print("ch%d: silent in both" % c)
            continue
        snr = 10 * math.log10(sig / err) if err > 0 else float("inf")
        print("ch%d: peak diff %.3e, rms ref %.4f, SNR %.1f dB" % (c, peak, math.sqrt(sig / n), snr))
        if snr < a.min_snr:
            fail = True
    print("RESULT", "FAIL" if fail else "PASS")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
