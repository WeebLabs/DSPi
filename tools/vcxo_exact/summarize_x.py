#!/usr/bin/env python3
"""Summarize crystal-pacer XS/XH lines: one row per (load, d, mode).

Usage: python3 summarize_x.py LOG [LOG ...]
"""
import sys

for path in sys.argv[1:]:
    xs = None
    for line in open(path):
        p = line.strip().split(",")
        if p[0] == "XS":
            xs = dict(kv.split("=") for kv in p[1:])
        elif p[0] == "XH" and xs:
            h = " ".join(x for x in p[4:] if not x.startswith("0:"))
            print(f"load={xs['load']:>3} N={xs['N']} e={xs.get('edge','0')} d={xs['d']:>2} mode={xs['mode']} arg={xs['arg']} "
                  f"pulses={xs['pulses']} sumdev={xs['sumdev']} sumabs={xs['sumabs']} nz={xs['nz']} "
                  f"unlock={xs['unlock']} lockn={xs['lockn']} fails={xs['fails']} other={xs['other']} "
                  f"tmo={xs['timeout']} hist[{h}]")
            xs = None
