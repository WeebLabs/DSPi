#!/usr/bin/env python3
"""Input batch-size CPU benchmark for a DSPI_BATCH_BENCH firmware build.

Method, profiles and safety notes: tools/batch_bench/README.md.
Run from the repo root with a Python that has pyusb, numpy and sounddevice.
"""

from __future__ import annotations

import argparse
import json
import pickle
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np
import sounddevice as sd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.dspi_test.device import DspiDevice, OP  # noqa: E402
from tools.dspi_test.profile import build_profile  # noqa: E402
from tools.dspi_test import lifecycle  # noqa: E402
from tools.dspi_test import coreaudio  # noqa: E402

REQ_BATCH_BENCH = 0x6F
SET_FLAG = 0x8000
FS = 48000
STATS_FMT = "<HHIIIII"            # BatchBenchStats in firmware/DSPi/batch_bench.h
DEFAULT_KS = [0, 96, 80, 64, 56, 48, 40, 32, 28, 24, 20, 16, 14, 12, 10, 8, 6, 5, 4, 3, 2, 1]
OVERLOAD_PCT = 92.0               # stop descending once core 0 passes this

FILTER_FLAT, FILTER_PEAKING = 0, 1
FILTER_LR4_LP, FILTER_LR4_HP = 34, 35
XOVER_BAND_BASE = 20


# --- device helpers ----------------------------------------------------------

def bench(dev, k=None):
    wv = 0 if k is None else (SET_FLAG | k)
    raw = dev.get(REQ_BATCH_BENCH, struct.calcsize(STATS_FMT), wvalue=wv)
    k_, _, calls, frames, c0, c0max, c1 = struct.unpack(STATS_FMT, raw)
    return dict(k=k_, calls=calls, frames=frames, c0_busy_us=c0, c0_max_busy_us=c0max, c1_busy_us=c1)


def status_u32(dev, wv):
    return dev.get_u32(OP.GET_STATUS, wvalue=wv)


def stock_meters(dev):
    v = status_u32(dev, 2)
    return (v >> 16) & 0xFF, (v >> 24) & 0xFF


def ring_fill_frames(dev):
    b = bytes(dev.get(OP.GET_BUFFER_STATS, 44))
    return [round(struct.unpack_from("<H", b, 4 + 8 * i + 6)[0] * 768 / 10000) for i in range(b[0])]


def bulk(dev, prof):
    return bytes(dev.get_ready(OP.GET_ALL_PARAMS, max(prof.bulk_payload_len, 2960)))


def load_slot(dev, slot, timeout_s=5.0):
    dev.get(OP.PRESET_LOAD, 1, wvalue=slot)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(0.2)
        try:
            if dev.get_u8(OP.PRESET_GET_ACTIVE) == slot:
                time.sleep(0.5)
                dev.wait_ready()
                return
        except Exception:  # noqa: BLE001  (busy window during the deferred load)
            pass
    raise RuntimeError(f"preset slot {slot} did not become active")


def eq(dev, ch, band, ftype, freq, q, gain):
    dev.set(OP.SET_EQ_PARAM, struct.pack("<BBBBfff", ch, band, ftype, 0, freq, q, gain))


def route(dev, inp, out, enabled, gain_db=0.0):
    dev.set(OP.SET_MATRIX_ROUTE, struct.pack("<BBBBf", inp, out, int(enabled), 0, gain_db))


# --- benchmark profiles --------------------------------------------------------
# Both start from factory DSP defaults (an empty preset slot).

PEQ_FREQS = [40, 80, 160, 315, 630, 1250, 2500, 5000, 10000, 16000]


def apply_profile(dev, prof, name):
    if name == "minimal":
        return {"profile": "minimal", "desc": "factory DSP defaults"}
    if name != "4way":
        raise SystemExit(f"unknown profile {name}")
    n_in, n_out = prof.num_input_channels, prof.num_output_channels
    spdif_outs = n_out - 1                      # last output is PDM
    pdm = n_out - 1
    dev.set_bytes(OP.SET_OUTPUT_ENABLE, b"\x00", wvalue=pdm)   # PDM excludes the core 1 EQ worker
    for o in range(spdif_outs):
        dev.set_bytes(OP.SET_OUTPUT_ENABLE, b"\x01", wvalue=o)
    for i in range(n_in):
        for o in range(n_out):
            route(dev, i, o, i < 2 and o < spdif_outs and (o % 2) == i)
    peq_channels = [0, 1] + [n_in + o for o in range(spdif_outs)]
    for ch in peq_channels:
        for b, f in enumerate(PEQ_FREQS):
            eq(dev, ch, b, FILTER_PEAKING, float(f), 1.4, 2.0 if b % 2 == 0 else -2.0)
    # 4-way LR4: pairs (0,1) sub, (2,3) low-mid, (4,5) mid, (6,7) HF.
    edges = [(None, 80.0), (80.0, 500.0), (500.0, 3000.0), (3000.0, None)]
    for o in range(spdif_outs):
        hp, lp = edges[min(o // 2, 3)]
        ch = n_in + o
        eq(dev, ch, XOVER_BAND_BASE + 0, FILTER_LR4_HP if hp else FILTER_FLAT, hp or 1000.0, 0.707, 0.0)
        eq(dev, ch, XOVER_BAND_BASE + 1, FILTER_LR4_LP if lp else FILTER_FLAT, lp or 1000.0, 0.707, 0.0)
        dev.set_f32(OP.SET_DELAY, 1.5 if o < 2 else 0.0, wvalue=ch)
    time.sleep(0.5)
    dev.wait_ready()
    return {"profile": "4way",
            "desc": (f"stereo USB in; outputs 1-{spdif_outs} on, PDM off; 10 peaking PEQ on both inputs "
                     f"and every output; LR4 4-way crossover; 1.5 ms delay on outputs 1-2; "
                     f"everything else at factory defaults")}


# --- playback ----------------------------------------------------------------

class Player:
    """Loops seeded -60 dBFS white noise to the DSPi output at 48 kHz, 16-bit stereo."""

    def __init__(self, name="DSPi"):
        dev_idx = next(i for i, d in enumerate(sd.query_devices())
                       if name in d["name"] and d["max_output_channels"] >= 2)
        coreaudio.set_rate_by_name(name, FS)
        rng = np.random.default_rng(1)
        amp = 10 ** (-60 / 20) * 32767
        self.buf = np.clip(rng.normal(0, amp / 2, (FS * 2, 2)), -32768, 32767).astype(np.int16)
        self.pos = 0
        self.underflows = 0          # callbacks PortAudio flagged as output underflow
        self.stream = sd.OutputStream(device=dev_idx, samplerate=FS, channels=2, dtype="int16",
                                      callback=self._cb, latency="high")

    def _cb(self, out, frames, _t, status):
        if status.output_underflow:
            self.underflows += 1
        n = len(self.buf)
        idx = (np.arange(frames) + self.pos) % n
        out[:] = self.buf[idx]
        self.pos = (self.pos + frames) % n

    def __enter__(self):
        self.stream.start()
        return self

    def __exit__(self, *a):
        self.stream.stop()
        self.stream.close()


# --- measurement ---------------------------------------------------------------

def measure(dev, k, window_s, settle_s):
    bench(dev, k)
    time.sleep(settle_s)
    und0 = status_u32(dev, 8)
    starv0 = status_u32(dev, 17)
    bench(dev, k)                             # clear: the window starts here
    t0 = time.time()
    c0s, c1s, fills = [], [], []
    while time.time() - t0 < window_s:
        time.sleep(0.25)
        a, b = stock_meters(dev)
        c0s.append(a)
        c1s.append(b)
        fills.append(min(ring_fill_frames(dev)))
    st = bench(dev)
    elapsed = time.time() - t0
    budget_us = st["frames"] / FS * 1e6
    res = dict(st)
    res.update(
        elapsed_s=elapsed,
        frames_per_call=st["frames"] / st["calls"] if st["calls"] else 0.0,
        c0_pct=100.0 * st["c0_busy_us"] / budget_us if budget_us else 0.0,
        c1_pct=100.0 * st["c1_busy_us"] / budget_us if budget_us else 0.0,
        us_per_call=st["c0_busy_us"] / st["calls"] if st["calls"] else 0.0,
        delivered_ratio=st["frames"] / (elapsed * FS),
        stock_c0_mean=float(np.mean(c0s)), stock_c1_mean=float(np.mean(c1s)),
        underruns=status_u32(dev, 8) - und0,
        dma_starvations=status_u32(dev, 17) - starv0,
        usb_alt=status_u32(dev, 11),
        ring_fill_min=min(fills), ring_fill_mean=float(np.mean(fills)),
    )
    # An overload empties the output rings, and they stay stopped until the stream restarts.
    res["healthy"] = res["ring_fill_min"] > 0 and abs(res["delivered_ratio"] - 1.0) < 0.003
    return res


def cmd_snapshot(args):
    dev = DspiDevice()
    prof = build_profile(dev)
    snap = lifecycle.capture(dev, prof)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "original_state.pkl").write_bytes(pickle.dumps(snap))
    print(f"captured: active slot {snap.active_slot}, bulk {len(snap.bulk)} B -> {out}")


def cmd_restore(args):
    dev = DspiDevice()
    prof = build_profile(dev)
    snap = pickle.loads((Path(args.out) / "original_state.pkl").read_bytes())
    if snap.occupied_mask & (1 << snap.active_slot):
        load_slot(dev, snap.active_slot)
    lifecycle.restore_live(dev, snap, settle_s=1.0)
    diffs = lifecycle.diff_bulk(dev, snap, prof)
    active = dev.get_u8(OP.PRESET_GET_ACTIVE)
    print(f"restored: active slot {active} (was {snap.active_slot}), bulk diffs {len(diffs)}")


def cmd_run(args):
    dev = DspiDevice()
    prof = build_profile(dev)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    snap = pickle.loads((out / "original_state.pkl").read_bytes())
    empty = next(s for s in range(10) if not snap.occupied_mask & (1 << s))
    bench(dev, 0)
    load_slot(dev, empty)
    meta = apply_profile(dev, prof, args.profile)
    ref = bulk(dev, prof)
    (out / f"config_{args.profile}.bin").write_bytes(ref)
    meta.update(platform=prof.summary(), sys_clk_hz=status_u32(dev, 13), sample_rate=status_u32(dev, 15),
                preset_slot_used=empty, bulk_len=len(ref), window_s=args.window, settle_s=args.settle)
    ks = [int(x) for x in args.ks.split(",")] if args.ks else DEFAULT_KS
    rows = []
    overloaded_at = None                       # smallest k never measured below
    for rep in range(args.repeats):
        order = ks if rep % 2 == 0 else list(reversed(ks))
        player = Player().__enter__()
        time.sleep(2.0)
        for k in order:
            if overloaded_at is not None and 0 < k < overloaded_at:
                continue
            r = measure(dev, k, args.window, args.settle)
            same = bulk(dev, prof) == ref
            r.update(rep=rep, config_unchanged=same)
            rows.append(r)
            print(f"rep{rep} k={k:3d} calls={r['calls']:6d} f/call={r['frames_per_call']:6.2f} "
                  f"c0={r['c0_pct']:6.2f}% c1={r['c1_pct']:6.2f}% us/call={r['us_per_call']:7.2f} "
                  f"max={r['c0_max_busy_us']:4d}us stock={r['stock_c0_mean']:.1f}/{r['stock_c1_mean']:.1f} "
                  f"deliv={r['delivered_ratio']:.3f} fill={r['ring_fill_min']}/{r['ring_fill_mean']:.0f} "
                  f"alt={r['usb_alt']} cfg={'ok' if same else 'CHANGED'} {'' if r['healthy'] else 'OVERLOAD'}",
                  flush=True)
            if not same:
                raise SystemExit("configuration drifted; aborting")
            if not r["healthy"] or (k > 0 and r["c0_pct"] > OVERLOAD_PCT):
                overloaded_at = k if overloaded_at is None else max(overloaded_at, k)
            if not r["healthy"]:
                bench(dev, 0)
                player.__exit__()
                time.sleep(6.0)                # host idle at alt 0 clears the stopped rings
                player = Player().__enter__()
                time.sleep(2.0)
        bench(dev, 0)
        player.__exit__()
        time.sleep(6.0)
    (out / f"results_{args.profile}.json").write_text(json.dumps({"meta": meta, "rows": rows}, indent=1))
    print(f"wrote {out}/results_{args.profile}.json")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("snapshot", "restore", "run"):
        p = sub.add_parser(name)
        p.add_argument("--out", required=True)
        if name == "run":
            p.add_argument("--profile", default="4way", choices=["minimal", "4way"])
            p.add_argument("--ks", default="")
            p.add_argument("--window", type=float, default=10.0)
            p.add_argument("--settle", type=float, default=2.0)
            p.add_argument("--repeats", type=int, default=2)
    args = ap.parse_args()
    {"snapshot": cmd_snapshot, "restore": cmd_restore, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    main()
