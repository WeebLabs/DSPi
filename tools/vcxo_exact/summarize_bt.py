#!/usr/bin/env python3
"""Summarize BT/CT phase tables: totals per sign and phases with miscounts.

Usage: python3 summarize_bt.py LOG [TAG]   (TAG = BT or CT, default BT)
"""
import collections, sys

tag = sys.argv[2] if len(sys.argv) > 2 else "BT"
tabs = collections.defaultdict(dict)
for line in open(sys.argv[1]):
    if not line.startswith(tag + ","):
        continue
    d = dict(kv.split("=") for kv in line.strip().split(",")[1:])
    key = (d.get("W", "128"), d["sign"])
    tabs[key][int(d["phase"])] = {k: int(d[k]) for k in ("c3", "c4", "c5", "c6", "c7", "other", "fail")}
for key, t in tabs.items():
    tot = collections.Counter()
    for h in t.values():
        tot.update(h)
    n = sum(v for k, v in tot.items() if k != "fail")
    mean = sum(int(k[1]) * v for k, v in tot.items() if k.startswith("c")) / max(n, 1)
    print(f"W={key[0]} sign={key[1]} n={n} mean={mean:.5f} " + " ".join(f"{k}={v}" for k, v in sorted(tot.items())))
    line = []
    for p in range(128):
        h = t.get(p)
        if not h:
            continue
        bad = {k: v for k, v in h.items() if k != "c5" and v}
        if bad:
            line.append(f"{p}:" + "/".join(f"{k[1:] if k[0]=='c' else k}x{v}" for k, v in bad.items()) + f"(n{sum(h.values())})")
    print("   ", " ".join(line))
