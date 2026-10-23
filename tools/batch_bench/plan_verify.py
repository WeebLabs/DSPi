#!/usr/bin/env python3
"""Filter-plan coherence under live edits (RP2350 bench build).

Applies random PEQ, crossover, band-bypass, master-bypass, preset-load and
bulk-parameter changes while streaming, and after each one asks the firmware
how many stored channel plans differ from a fresh build (must be 0).
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

PLAN_CHECK = 0x0800
PEQ_TYPES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13]
XOVER_TYPES = [0, 32, 33, 34, 35, 38, 39, 40, 41, 46, 47, 56, 57, 62, 63]


def incoherent(dev):
    return struct.unpack("<I", bytes(dev.get(bb.REQ_BATCH_BENCH, 4, wvalue=PLAN_CHECK)))[0]


def main():
    out = Path(sys.argv[1])
    actions = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    rnd = random.Random(11)
    dev = DspiDevice()
    p = build_profile(dev)
    n_in = p.num_input_channels
    bb.bench(dev, 0)
    bb.load_slot(dev, 1)
    bb.apply_profile(dev, p, "4way")
    blob = bb.bulk(dev, p)
    counts, failures = {}, []

    def eq(ch, band, t, byp, f, q, g):
        dev.set(OP.SET_EQ_PARAM, struct.pack("<BBBBfff", ch, band, t, byp, f, q, g))

    def log_freq():
        return 10 ** rnd.uniform(1.3, 4.3)          # 20 Hz .. 20 kHz, crosses the Fs/7.5 SVF split

    ops = {
        "peq": lambda: eq(rnd.randrange(p.num_channels), rnd.randrange(10), rnd.choice(PEQ_TYPES),
                          int(rnd.random() < 0.2), log_freq(), rnd.uniform(0.3, 4.0), rnd.uniform(-12, 12)),
        "xover": lambda: eq(n_in + rnd.randrange(8), 20 + rnd.randrange(4), rnd.choice(XOVER_TYPES),
                            int(rnd.random() < 0.2), log_freq(), 0.707, 0.0),
        "band_bypass": lambda: dev.set(OP.SET_BAND_BYPASS, bytes([rnd.randint(0, 1)]),
                                       wvalue=(rnd.randrange(p.num_channels) << 8) | rnd.randrange(10)),
        "master_bypass": lambda: dev.set(OP.SET_BYPASS, bytes([rnd.randint(0, 1)])),
        "preset_load": lambda: bb.load_slot(dev, rnd.choice([1, 2])),
        "bulk_apply": lambda: (dev.set(OP.SET_ALL_PARAMS, blob), time.sleep(0.4), dev.wait_ready()),
    }
    weights = {"peq": 50, "xover": 30, "band_bypass": 12, "master_bypass": 3, "preset_load": 2, "bulk_apply": 3}
    names = list(weights)
    with bb.Player():
        time.sleep(1.5)
        for i in range(actions):
            name = rnd.choices(names, [weights[n] for n in names])[0]
            ops[name]()
            time.sleep(0.05)
            bad = incoherent(dev)
            counts[name] = counts.get(name, 0) + 1
            if bad:
                failures.append({"action": i, "op": name, "incoherent_channels": bad})
        fills = bb.ring_fill_frames(dev)
    res = {"actions": actions, "by_op": counts, "failures": failures, "ring_fill_end": fills}
    print(json.dumps(res, indent=1))
    (out / "plan_verify.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
