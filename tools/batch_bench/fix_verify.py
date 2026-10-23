#!/usr/bin/env python3
"""Checks for the overhead fixes (Documentation/Features/low_latency_overhead_plan.md).

1. Idle-reset invariant: toggles crossfeed, subharmonic, psybass, tube and
   output mutes at random while streaming, and reads the firmware's count of
   inactive feature states that were not clean (must be 0).
2. Fix 3 prototype: per output, current cascade vs precomputed plan cycles at
   0 and 16 samples, plus bit-exactness of outputs and final filter state.
"""

from __future__ import annotations

import json
import random
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import batch_bench as bb  # noqa: E402
from tools.dspi_test.device import DspiDevice, OP  # noqa: E402
from tools.dspi_test.profile import build_profile  # noqa: E402

VERIFY, CASCADE = 0x1000, 0x2000


def verify(dev, on):
    return struct.unpack("<II", bytes(dev.get(bb.REQ_BATCH_BENCH, 8, wvalue=VERIFY | int(on))))


def setup_4way(dev, p):
    bb.bench(dev, 0)
    bb.load_slot(dev, 1)
    bb.apply_profile(dev, p, "4way")


def toggle_test(dev, p, actions=120, seed=7):
    rnd = random.Random(seed)
    u8 = lambda v: bytes([v & 0xFF])
    u16 = lambda v: struct.pack("<H", v)
    f32 = lambda v: struct.pack("<f", v)
    ops = [
        lambda: dev.set(OP.SET_SUBHARM, u8(rnd.randint(0, 1))),
        lambda: dev.set(OP.SET_SUBHARM_MASK, u16(rnd.getrandbits(8))),
        lambda: dev.set(OP.SET_PSYBASS, u8(rnd.randint(0, 1))),
        lambda: dev.set(OP.SET_PSYBASS_MASK, u16(rnd.getrandbits(8))),
        lambda: dev.set(OP.SET_TUBE_PARAM, f32(float(rnd.randint(0, 1))), wvalue=0),
        lambda: dev.set(OP.SET_TUBE_PARAM, f32(float(rnd.getrandbits(8))), wvalue=1),
        lambda: dev.set(OP.SET_CROSSFEED, u8(rnd.randint(0, 1))),
        lambda: dev.set(OP.SET_CROSSFEED_OUTPUTS, u8(rnd.getrandbits(4))),
        lambda: dev.set(OP.SET_OUTPUT_MUTE, u8(rnd.randint(0, 1)), wvalue=rnd.randint(0, 7)),
    ]
    verify(dev, False)
    verify(dev, True)
    with bb.Player():
        time.sleep(1.5)
        for _ in range(actions):
            rnd.choice(ops)()
            time.sleep(0.12)
        time.sleep(1.0)
        fills = bb.ring_fill_frames(dev)
    checks, bad = verify(dev, True)
    verify(dev, False)
    return {"actions": actions, "checks": checks, "violations": bad, "ring_fill_end": fills}


def cascade_test(dev):
    rows = []
    for out in range(8):
        r = struct.unpack("<8I", bytes(dev.get(bb.REQ_BATCH_BENCH, 32, wvalue=CASCADE | out)))
        rows.append(dict(zip(["sections", "steps", "legacy0", "legacy16", "plan0", "plan16",
                              "mismatches", "iters"], r), out=out))
    return rows


def main():
    out = Path(sys.argv[1])
    dev = DspiDevice()
    p = build_profile(dev)
    setup_4way(dev, p)
    t = toggle_test(dev, p)
    print(f"invariant: {t['checks']} checks, {t['violations']} violations, ring fill at end {t['ring_fill_end']}")
    setup_4way(dev, p)                      # undo the toggles before timing
    c = cascade_test(dev)
    print(f"{'out':>3} {'sect':>4} {'steps':>5} {'legacy0':>8} {'plan0':>6} {'legacy16':>9} {'plan16':>7} {'mism':>5}")
    for r in c:
        print(f"{r['out']:3d} {r['sections']:4d} {r['steps']:5d} {r['legacy0']:8d} {r['plan0']:6d} "
              f"{r['legacy16']:9d} {r['plan16']:7d} {r['mismatches']:5d}")
    (out / "fix_verify.json").write_text(json.dumps({"toggle": t, "cascade": c}, indent=1))


if __name__ == "__main__":
    main()
