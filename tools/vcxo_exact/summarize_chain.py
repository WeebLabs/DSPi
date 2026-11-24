#!/usr/bin/env python3
"""Per-delay summary of chain-pacer XS lines: kind, cdelay, + and - sumabs.

Usage: python3 summarize_chain.py LOG [LOG ...]
"""
import collections, sys

for path in sys.argv[1:]:
    t = collections.OrderedDict()
    load = "?"
    for line in open(path):
        if line.startswith("XS"):
            d = dict(kv.split("=") for kv in line.strip().split(",")[1:])
            load = d["load"]
            k = (d["load"], d["kind"], d["cdelay"])
            t.setdefault(k, {})[d["mode"]] = (int(d["sumabs"]), int(d["other"]), int(d["pulses"]))
    print(f"== {path.split('/')[-1]} load={load}")
    for (ld, kind, cd), v in t.items():
        cells = " ".join(f"m{m}:{a}{'+o' + str(o) if o else ''}/{p // 1000}k" for m, (a, o, p) in sorted(v.items()))
        print(f"  load={ld:>3} kind={kind} cdelay={cd:>3} {cells}")
