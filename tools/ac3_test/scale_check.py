#!/usr/bin/env python3
"""Check that b.f32 equals a.f32 scaled by a constant gain (dialnorm test).

Usage: scale_check.py a.f32 b.f32 gain_db [--tol 1e-5]
Exits non-zero when any sample differs by more than tol (relative to full
scale) after scaling.  No numpy on purpose.
"""
import array, math, sys
a = array.array("f"); b = array.array("f")
a.frombytes(open(sys.argv[1], "rb").read()); b.frombytes(open(sys.argv[2], "rb").read())
g = 10 ** (float(sys.argv[3]) / 20.0)
tol = float(sys.argv[5]) if len(sys.argv) > 5 else 1e-5
n = min(len(a), len(b)); worst = 0.0
for i in range(n):
    d = abs(a[i] * g - b[i])
    if d > worst: worst = d
print("samples=%d gain=%.6f worst=%.3e %s" % (n, g, worst, "PASS" if worst <= tol else "FAIL"))
sys.exit(0 if worst <= tol else 1)
