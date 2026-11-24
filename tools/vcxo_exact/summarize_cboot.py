#!/usr/bin/env python3
"""Landing-phase signature per chain re-init: + miscounts per cdelay 0..25 under load.

Usage: python3 summarize_cboot.py LOG [LOG ...]
Prints one row per re-init: kind, then sumabs per cdelay (dot for zero).
"""
import sys

for path in sys.argv[1:]:
    rows, cur, kind = [], None, None
    for line in open(path):
        line = line.strip()
        if line.startswith("> c ") and line.endswith(" 2"):
            kind = line.split()[2]
            cur = {}
            rows.append((kind, cur))
        elif line.startswith("XS") and cur is not None:
            d = dict(kv.split("=") for kv in line.split(",")[1:])
            cur[int(d["cdelay"])] = int(d["sumabs"]) + 1000 * int(d["other"])
    print(f"== {path.split('/')[-1]}")
    for kind, r in rows:
        cells = " ".join(f"{r.get(i, -1):>5}" if r.get(i, 0) else "    ." for i in range(26))
        print(f"  kind={kind} {cells}")
