#!/usr/bin/env python3
"""Per-stage cycle profile of the pipeline at a few batch sizes (RP2350 bench build).

Splits each stage's time per call into a fixed part and a per-frame part.
Section map: STAGES_C0 / STAGES_C1 below, matching the BENCH_PROF checkpoints.
"""

from __future__ import annotations

import json
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import batch_bench as bb  # noqa: E402
from tools.dspi_test.device import DspiDevice  # noqa: E402
from tools.dspi_test.profile import build_profile  # noqa: E402

PROF_FLAG = 0x4000
PLAN_FLAG = 0x0800
STAGES_C0 = ["entry, ring admission", "mute envelope, snapshots", "input EQ + meters",
             "leveller, upmixer", "matrix, siggen", "limiter setup, core 1 dispatch",
             "crossfeed, subharmonic", "output EQ/gain loop", "state resets, limiter, delay",
             "output peaks, finalize", "wait for core 1", "limiter end",
             "loopback, ring writes", "ADAT push, RTA end"]
STAGES_C1 = ["crossfeed, subharmonic", "output EQ/gain loop", "limiter, delay",
             "output peaks", "finalize"]


def prof(dev, core):
    return struct.unpack("<16I", bytes(dev.get(bb.REQ_BATCH_BENCH, 64, wvalue=PROF_FLAG | core)))


def main():
    out = Path(sys.argv[1])
    ks = [int(x) for x in sys.argv[2].split(",")]
    window = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0
    variant = sys.argv[4] if len(sys.argv) > 4 else "current"   # "plan" runs filters on precomputed plans
    dev = DspiDevice()
    p = build_profile(dev)
    hz = bb.status_u32(dev, 13)
    snap_mask = 1                                   # slot 0 holds the board's own preset
    empty = next(s for s in range(10) if not snap_mask & (1 << s))
    result = {"sys_clk_hz": hz, "ks": ks, "variant": variant, "profiles": {}}
    for name in ("minimal", "4way"):
        bb.bench(dev, 0)
        dev.get(bb.REQ_BATCH_BENCH, 4, wvalue=PLAN_FLAG)          # never load a preset with stale plans
        bb.load_slot(dev, empty)
        bb.apply_profile(dev, p, name)
        if variant == "plan":
            steps = struct.unpack("<I", bytes(dev.get(bb.REQ_BATCH_BENCH, 4, wvalue=PLAN_FLAG | 1)))[0]
            print(f"{name}: running on precomputed plans, {steps} steps", flush=True)
        ref = (out / f"config_{name}.bin").read_bytes()
        same = bb.bulk(dev, p) == ref
        print(f"{name}: config matches sweep reference: {same}", flush=True)
        rows = []
        with bb.Player():
            time.sleep(2.0)
            for k in ks:
                bb.bench(dev, k)
                time.sleep(2.0)
                bb.bench(dev, k)
                time.sleep(window)
                c0, c1 = prof(dev, 0), prof(dev, 1)
                st = bb.bench(dev)
                fills = bb.ring_fill_frames(dev)
                rows.append({"k": k, "calls": st["calls"], "frames": st["frames"], "c0_busy_us": st["c0_busy_us"],
                             "c0": list(c0), "c1": list(c1), "ring_fill": fills})
                print(f"  k={k:2d} calls={st['calls']} us/call={st['c0_busy_us']/st['calls']:.2f} "
                      f"prof_sum_us/call={sum(c0)/st['calls']/hz*1e6:.2f} fill={fills}", flush=True)
        bb.bench(dev, 0)
        dev.get(bb.REQ_BATCH_BENCH, 4, wvalue=PLAN_FLAG)
        result["profiles"][name] = {"config_matches": same, "rows": rows}
        time.sleep(6.0)
    (out / f"stage_profile_{variant}.json").write_text(json.dumps(result, indent=1))

    for name, pr in result["profiles"].items():
        print(f"\n== {name}: per-call cost by stage (us), fit = fixed + per_frame * k")
        for core, labels in ((0, STAGES_C0), (1, STAGES_C1)):
            K = np.array([r["k"] for r in pr["rows"]], float)
            for i, lab in enumerate(labels):
                y = np.array([r[f"c{core}"][i] / r["calls"] / hz * 1e6 for r in pr["rows"]])
                if not y.any():
                    continue
                b, a = np.polyfit(K, y, 1)
                cells = " ".join(f"{v:7.2f}" for v in y)
                print(f"  c{core} {i:2d} {lab:32s} {cells}   fixed={a:6.2f} per_frame={b:6.3f}")


if __name__ == "__main__":
    main()
