#!/usr/bin/env python3
"""Edge analysis of RAW frame dumps: per-frame rising/falling edge residues.

Usage: python3 analyze_raw.py LOG [first_settled_frame]
"""
import sys, collections

U = [(15 * j) & 127 for j in range(128)]

def residues(words):
    a = [[] for _ in range(128)]
    for j in range(128):
        a[U[j]].append((words[j >> 5] >> (j & 31)) & 1)
    return a

def edges(a):
    seq = [(u, a[u]) for u in range(128) if a[u]]
    rise, fall, flick = [], [], 0
    for k in range(len(seq)):
        (u0, v0), (u1, v1) = seq[k - 1], seq[k]
        if len(set(v1)) > 1:
            flick += 1
        if v0[-1] == 0 and v1[0] == 1:
            rise.append(u1)
        if v0[-1] == 1 and v1[0] == 0:
            fall.append(u1)
    return rise, fall, flick

first = int(sys.argv[2]) if len(sys.argv) > 2 else 4
hs, bad, n = collections.Counter(), 0, 0
per_s = collections.defaultdict(collections.Counter)
for line in open(sys.argv[1]):
    if not line.startswith("RAW,"):
        continue
    p = line.strip().split(",")
    if int(p[1]) < first:
        continue
    w = [int(x, 16) for x in p[3:7]]
    r, f, fl = edges(residues(w))
    n += 1
    if len(r) != 1 or len(f) != 1:
        bad += 1
        continue
    H = (f[0] - r[0]) & 127
    hs[H] += 1
    per_s[r[0]][(H, int(p[7]), int(p[8]))] += 1
print("frames", n, "multi-edge", bad, "H", dict(hs))
for s in sorted(per_s):
    print(s, dict(per_s[s]))
