#!/usr/bin/env python3
"""Summarize D (burst) results: deviation histograms and per-pulse miscount rates.

Usage: python3 summarize_d.py LOG [LOG ...]
sumabs/pulses is a lower bound on the miscount rate (a 4 and a 6 in one burst cancel).
"""
import sys

for path in sys.argv[1:]:
    ds = None
    for line in open(path):
        p = line.strip().split(",")
        if p[0] == "DS":
            ds = dict(kv.split("=") for kv in p[1:])
        elif p[0] == "DH" and ds:
            hist = {int(k): int(v) for k, v in (x.split(":") for x in p[4:])}
            n = int(ds["pulses"])
            rate = int(ds["sumabs"]) / n if n else 0
            nz = {k: v for k, v in hist.items() if v and k}
            print(f"{path.split('/')[-1]} mode={ds['mode']} sp={ds['sp']} load={ds['load']} park={ds['park']} "
                  f"bursts={ds['bursts']} pulses={n} sumdev={ds['sumdev']} sumabs={ds['sumabs']} "
                  f"rate>={rate:.3e} unlock={ds['unlock']} fails={ds['fails']} uncertain={ds.get('uncertain')} "
                  f"dev_hist_nonzero={nz}")
            ds = None
