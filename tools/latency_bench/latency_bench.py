#!/usr/bin/env python3
"""Output latency stability bench: every selectable latency on every source the
bench can drive (USB, I2S input in master mode, the signal-generator pump, and
S/PDIF input when one locks). Method and pass criteria: tools/latency_bench/README.md.
"""

from __future__ import annotations

import argparse
import json
import pickle
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.dspi_test.device import DspiDevice, PagedStall  # noqa: E402
from tools.dspi_test.profile import build_profile  # noqa: E402
from tools.dspi_test import lifecycle  # noqa: E402
from tools.latency_bench.paged import (  # noqa: E402
    buffer_stats, pdm_stats, pget_ready, pipeline_stats, spdif_tx_stats, stats_reset, system_status,
    usb_audio_status, wait_ready)

LAT_FMT = "<BBHHHBBH"             # OutputLatencyStatus in firmware/DSPi/config.h
SRC_USB, SRC_SPDIF, SRC_I2S = 0, 1, 2
LAT_LOW, LAT_LOW_1MS, LAT_2MS, LAT_4MS, LAT_8MS = 1, 2, 3, 5, 7   # OutputLatencyId (output_latency.h)
OPTION_IDS = [1, 2, 3, 4, 5, 6, 7]
FS_HOST = 48000


def counters(dev):
    p, u, t, d = pipeline_stats(dev), usb_audio_status(dev), spdif_tx_stats(dev), pdm_stats(dev)
    return {"underruns": p["ring_underruns"], "drops": p["ring_drops"],
            "usb_ring_overruns": u["ring_overruns"], "usb_packet_gaps": t["spdif_underruns"],
            "pdm_ring_underruns": d["ring_underruns"], "pdm_dma_underruns": d["dma_underruns"]}


def sample_rate(dev):
    return system_status(dev)["sample_rate_hz"]


def latency_get(dev):
    """OUTPUT.LATENCY: the stored option ID, or the queued one while a change is pending."""
    return dev.pget("OUTPUT.LATENCY", length=1)[0]


def latency_status(dev):
    req, eff, eff_us, frames, batch, pending, count, _ = struct.unpack(
        LAT_FMT, dev.pget("OUTPUT.LATENCY_STATUS", length=12))
    return dict(requested_id=req, effective_id=eff, effective_us=eff_us, target_frames=frames,
                batch_frames=batch, pending=pending, option_count=count)


def set_latency(dev, opt_id, timeout_s=8.0):
    """An unknown option ID STALLs with VC_ERR_VALUE; the apply is a flash write and an output restart."""
    dev.pset("OUTPUT.LATENCY", bytes([opt_id]))
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(0.1)
        try:
            st = latency_status(dev)
        except Exception:  # noqa: BLE001  (busy during the flash window)
            continue
        if st["requested_id"] == opt_id and not st["pending"]:
            return st
    raise RuntimeError(f"latency option {opt_id} was not applied")


def slot_fills(dev):
    """Per-slot live fill in frames for playing slots (fill_centi_pct on the 768-frame scale)."""
    return [s["fill_centi_pct"] * 768 / 10000 for s in buffer_stats(dev)["spdif"] if s["consumer_playing"]]


def set_source(dev, src, rate=None):
    """The I2S rate is a stored setting and is not put back afterwards."""
    if rate is not None:
        dev.pset("INPUT.I2S_RX_RATE", struct.pack("<I", rate))
    dev.pset("INPUT.SOURCE", bytes([src]))


def wait_rate(dev, rate, timeout_s=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(0.2)
        try:
            if sample_rate(dev) == rate:
                return True
        except Exception:  # noqa: BLE001
            pass
    return False


def siggen(dev, on):
    if on:
        cfg = struct.pack("<BBHHBBfIHHffff", 1, 0, 0x00FF, 0, 0, 0, -40.0, 0, 0, 0, 997.0, 0, 0, 0)
        dev.pset("MEAS.SIGGEN_CONFIG", cfg)
        dev.pset("MEAS.SIGGEN_START")
    else:
        dev.pset("MEAS.SIGGEN_STOP_NOW")


def preset_active(dev):
    return dev.pget("STORE.PRESET_ACTIVE", length=1)[0]


def bulk(dev, prof):
    return pget_ready(dev, "STORE.ALL_PARAMS", length=max(prof.bulk_payload_len, 2960))


def load_slot(dev, slot, timeout_s=5.0):
    dev.pset("STORE.PRESET_LOAD", addr=slot)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(0.2)
        try:
            if preset_active(dev) == slot:
                time.sleep(0.5)
                wait_ready(dev)
                return
        except Exception:  # noqa: BLE001  (busy window during the deferred load)
            pass
    raise RuntimeError(f"preset slot {slot} did not become active")


# --- 4-way profile (the heavy profile of tools/batch_bench), from factory DSP defaults ----------

FILTER_FLAT, FILTER_PEAKING = 0, 1
FILTER_LR4_LP, FILTER_LR4_HP = 34, 35
XOVER_BAND_BASE = 20
PEQ_FREQS = [40, 80, 160, 315, 630, 1250, 2500, 5000, 10000, 16000]


def band(dev, ch, b, ftype, freq, q, gain, tries=100):
    """FILTER.BAND refuses with VC_ERR_BUSY until the main loop takes the previous band write."""
    data = struct.pack("<BBHfff", ftype, 0, 0, freq, q, gain)
    for _ in range(tries):
        try:
            dev.pset("FILTER.BAND", data, addr=(ch << 8) | b)
            return
        except PagedStall as e:
            if e.result_name != "VC_ERR_BUSY":
                raise
            time.sleep(0.005)
    raise RuntimeError(f"FILTER.BAND {ch}/{b} still busy after {tries} tries")


def route(dev, inp, out, enabled, gain_db=0.0):
    dev.pset("ROUTE.MATRIX_ROUTE", struct.pack("<BBHf", int(enabled), 0, 0, gain_db), addr=(inp << 8) | out)


def apply_4way(dev, prof):
    n_in, n_out = prof.num_input_channels, prof.num_output_channels
    spdif_outs = n_out - 1                      # last output is PDM
    pdm = n_out - 1
    dev.pset("ROUTE.OUTPUT_ENABLE", b"\x00", addr=pdm)   # PDM excludes the core 1 EQ worker
    for o in range(spdif_outs):
        dev.pset("ROUTE.OUTPUT_ENABLE", b"\x01", addr=o)
    for i in range(n_in):
        for o in range(n_out):
            route(dev, i, o, i < 2 and o < spdif_outs and (o % 2) == i)
    for ch in [0, 1] + [n_in + o for o in range(spdif_outs)]:
        for b, f in enumerate(PEQ_FREQS):
            band(dev, ch, b, FILTER_PEAKING, float(f), 1.4, 2.0 if b % 2 == 0 else -2.0)
    # LR4 pairs: (0,1) sub, (2,3) low-mid, (4,5) mid, (6,7) HF.
    edges = [(None, 80.0), (80.0, 500.0), (500.0, 3000.0), (3000.0, None)]
    for o in range(spdif_outs):
        hp, lp = edges[min(o // 2, 3)]
        ch = n_in + o
        band(dev, ch, XOVER_BAND_BASE + 0, FILTER_LR4_HP if hp else FILTER_FLAT, hp or 1000.0, 0.707, 0.0)
        band(dev, ch, XOVER_BAND_BASE + 1, FILTER_LR4_LP if lp else FILTER_FLAT, lp or 1000.0, 0.707, 0.0)
        dev.pset("ROUTE.CHANNEL_DELAY", struct.pack("<f", 1.5 if o < 2 else 0.0), addr=ch)
    time.sleep(0.5)
    wait_ready(dev)


# --- host playback -----------------------------------------------------------------------------

def audio_available():
    try:
        import sounddevice  # noqa: F401
        return True
    except ImportError:
        return False


class Player:
    """Loops seeded -60 dBFS white noise to the DSPi at 48 kHz, 16-bit stereo. Needs sounddevice."""

    def __init__(self, name="DSPi"):
        import sounddevice as sd
        from tools.dspi_test import coreaudio
        dev_idx = next(i for i, d in enumerate(sd.query_devices())
                       if name in d["name"] and d["max_output_channels"] >= 2)
        coreaudio.set_rate_by_name(name, FS_HOST)
        rng = np.random.default_rng(1)
        amp = 10 ** (-60 / 20) * 32767
        self.buf = np.clip(rng.normal(0, amp / 2, (FS_HOST * 2, 2)), -32768, 32767).astype(np.int16)
        self.pos = 0
        self.underflows = 0          # callbacks PortAudio flagged as output underflow
        self.stream = sd.OutputStream(device=dev_idx, samplerate=FS_HOST, channels=2, dtype="int16",
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


def fill_ok(mean, expect, block_cap):
    """Prefill-anchored sources (I2S master has no servo) may sit up to one block high."""
    return -block_cap <= mean - expect <= block_cap + 2.0


def measure(dev, opt_id, rate, window_s, steady_s):
    """Window starts the moment the change applies, so the post-reset transient counts."""
    st = set_latency(dev, opt_id)
    c0 = counters(dev)
    stats_reset(dev)
    t0 = time.time()
    samples, spreads, c0s, c1s = [], [], [], []
    while time.time() - t0 < window_s:
        time.sleep(0.1)
        f = slot_fills(dev)
        if not f:
            continue
        samples.append((time.time() - t0, float(np.mean(f)), min(f)))
        spreads.append(max(f) - min(f))
        ss = system_status(dev)
        c0s.append(ss["cpu0_load"])
        c1s.append(ss["cpu1_load"])
    c1 = counters(dev)
    minpre = pipeline_stats(dev)["min_prewrite"]
    d = {k: c1[k] - c0[k] for k in c0}
    block_cap = st["batch_frames"] + 1
    steady = [m for t, m, _ in samples if t >= window_s - steady_s]
    expect = st["target_frames"] - block_cap / 2.0     # live fill averages half a block below
    r = dict(st, rate=rate, window_s=window_s, **d,
             min_prewrite_frames=None if minpre == 0xFFFFFFFF else minpre,
             fill_mean=float(np.mean(steady)) if steady else None,
             fill_expect=expect,
             fill_min=float(min(x for _, _, x in samples)) if samples else None,
             fill_max=float(max(m for _, m, _ in samples)) if samples else None,
             slot_spread_max=float(max(spreads)) if spreads else None,
             # The stats GET reads slots in turn with interrupts on, so an ISR
             # between two reads shows as a rare 2-3 frame spread; judge the bulk.
             slot_spread_within1=(sum(x <= 1.1 for x in spreads) / len(spreads)) if spreads else None,
             cpu0_mean=float(np.mean(c0s)) if c0s else None,
             cpu1_mean=float(np.mean(c1s)) if c1s else None,
             trace=[(round(t, 2), round(m, 1)) for t, m, _ in samples[::5]])
    r["pass"] = bool(d["underruns"] == 0 and d["drops"] == 0 and r["min_prewrite_frames"]
                     and r["fill_mean"] is not None and fill_ok(r["fill_mean"], expect, block_cap)
                     and r["slot_spread_within1"] is not None and r["slot_spread_within1"] >= 0.98)
    return r


def fmt(r):
    return (f"{r['source']:7s} {r['rate']:6d} opt={r['requested_id']}->{r['effective_id']} "
            f"eff={r['effective_us']:4d}us batch={r['batch_frames']:2d} "
            f"tgt={r['target_frames']:3d} mean={r['fill_mean']:6.1f}/{r['fill_expect']:5.1f} "
            f"range={r['fill_min']:5.1f}..{r['fill_max']:5.1f} "
            f"pre={r['min_prewrite_frames']} spread={r['slot_spread_max']:.1f} "
            f"ur={r['underruns']} drop={r['drops']} usbov={r['usb_ring_overruns']} "
            f"gap={r['usb_packet_gaps']} pdm={r['pdm_ring_underruns']}/{r['pdm_dma_underruns']} "
            f"cpu={r['cpu0_mean']:.0f}/{r['cpu1_mean']:.0f} {'PASS' if r['pass'] else 'FAIL'}")


def run_source(dev, name, args, rows, out):
    lat = [int(x) for x in args.options.split(",")] if args.options else OPTION_IDS
    player = None
    if name == "usb48":
        if not audio_available():
            print("usb48: sounddevice not installed, skipped", flush=True)
            return
        set_source(dev, SRC_USB)
        player = Player().__enter__()
        rate = FS_HOST
    elif name.startswith("i2s"):
        rate = {"i2s44": 44100, "i2s48": 48000, "i2s96": 96000}[name]
        set_source(dev, SRC_I2S, rate)
    elif name == "siggen":
        set_source(dev, SRC_USB)
        time.sleep(1.0)
        rate = sample_rate(dev)
        siggen(dev, True)
    elif name == "spdif":
        set_source(dev, SRC_SPDIF)
        time.sleep(4.0)
        state = dev.pget("INPUT.SPDIF_RX_STATUS", length=16)
        if state[0] != 2:
            print("spdif: no lock, skipped", flush=True)
            return
        rate = struct.unpack_from("<I", state, 4)[0]
    else:
        raise SystemExit(f"unknown source {name}")
    if not wait_rate(dev, rate):
        print(f"{name}: device did not reach {rate} Hz, skipped", flush=True)
        if player:
            player.__exit__()
        return
    time.sleep(2.0)
    try:
        for opt_id in lat:
            r = measure(dev, opt_id, rate, args.window, args.steady)
            r.update(source=name, profile=args.profile)
            rows.append(r)
            print(fmt(r), flush=True)
            (out / f"results_{args.profile}.json").write_text(json.dumps(rows, indent=1))
    finally:
        if player:
            player.__exit__()
        if name == "siggen":
            siggen(dev, False)


def run_checks(dev, out):
    """Rate-follow, source-fallback and rapid-switch alignment checks."""
    res = {}
    set_source(dev, SRC_I2S, 48000)
    wait_rate(dev, 48000)
    set_latency(dev, LAT_8MS)
    a = latency_status(dev)
    set_source(dev, SRC_I2S, 96000)
    wait_rate(dev, 96000)
    time.sleep(2.0)
    b = latency_status(dev)
    set_source(dev, SRC_I2S, 48000)
    wait_rate(dev, 48000)
    time.sleep(2.0)
    c = latency_status(dev)
    res["rate_follow"] = dict(at48=a, at96=b, back48=c,
                              ok=(a["target_frames"] == 384 and b["requested_id"] == LAT_8MS
                                  and b["target_frames"] == 351 and c["target_frames"] == 384))
    set_latency(dev, LAT_LOW)
    i2s = latency_status(dev)
    set_source(dev, SRC_USB)
    time.sleep(3.0)
    usb = latency_status(dev)
    set_source(dev, SRC_I2S, 48000)
    wait_rate(dev, 48000)
    time.sleep(3.0)
    back = latency_status(dev)
    res["source_fallback"] = dict(i2s=i2s, usb=usb, back=back,
                                  ok=(i2s["effective_id"] == LAT_LOW and i2s["target_frames"] == 32
                                      and i2s["batch_frames"] == 16
                                      and usb["requested_id"] == LAT_LOW and usb["effective_id"] == LAT_2MS
                                      and usb["batch_frames"] == 48
                                      and back["effective_id"] == LAT_LOW))
    spreads = []
    for i in range(20):
        set_latency(dev, LAT_LOW if i % 2 == 0 else LAT_8MS)
        time.sleep(1.5)
        f = [slot_fills(dev) for _ in range(5)]
        spreads.append(max(max(x) - min(x) for x in f if x))
    res["rapid_switch"] = dict(spreads=spreads, ok=max(spreads) <= 2.0)
    (out / "checks.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1), flush=True)


def cmd_snapshot(args):
    dev = DspiDevice()
    snap = lifecycle.capture(dev, build_profile(dev))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "original_state.pkl").write_bytes(pickle.dumps(snap))
    (out / "original_latency.json").write_text(json.dumps(latency_status(dev)))
    print(f"captured: active slot {snap.active_slot}, source {snap.input_source}, bulk {len(snap.bulk)} B")


def cmd_restore(args):
    dev = DspiDevice()
    prof = build_profile(dev)
    out = Path(args.out)
    snap = pickle.loads((out / "original_state.pkl").read_bytes())
    if snap.occupied_mask & (1 << snap.active_slot):
        load_slot(dev, snap.active_slot)
    lifecycle.restore_live(dev, snap, settle_s=1.0)
    lat = json.loads((out / "original_latency.json").read_text())
    set_latency(dev, lat["requested_id"])
    diffs = lifecycle.diff_bulk(dev, snap, prof)
    print(f"restored: slot {preset_active(dev)} (was {snap.active_slot}), "
          f"latency option {latency_status(dev)['requested_id']}, bulk diffs {len(diffs)}")


def cmd_run(args):
    dev = DspiDevice()
    prof = build_profile(dev)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    snap = pickle.loads((out / "original_state.pkl").read_bytes())
    if args.profile == "4way":
        empty = next(s for s in range(10) if not snap.occupied_mask & (1 << s))
        load_slot(dev, empty)
        apply_4way(dev, prof)
    ref = bulk(dev, prof)
    rows = []
    # Muted throughout: an undriven I2S data pin reads as full-scale noise.
    # Mute is a gain stage only, so ring timing is unaffected.
    dev.pset("ROUTE.USER_MUTE", b"\x01")
    try:
        for name in args.sources.split(","):
            run_source(dev, name, args, rows, out)
        if args.checks:
            run_checks(dev, out)
    finally:
        set_source(dev, snap.input_source)
        dev.pset("ROUTE.USER_MUTE", bytes([snap.user_mute]))
    changed = bulk(dev, prof) != ref
    print(f"done: {sum(r['pass'] for r in rows)}/{len(rows)} pass, config "
          f"{'CHANGED' if changed else 'unchanged'}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("snapshot", "restore", "run"):
        p = sub.add_parser(name)
        p.add_argument("--out", required=True)
        if name == "run":
            p.add_argument("--profile", default="user", choices=["user", "4way"])
            p.add_argument("--sources", default="usb48,i2s44,i2s48,i2s96,siggen,spdif")
            p.add_argument("--options", default="")
            p.add_argument("--window", type=float, default=30.0)
            p.add_argument("--steady", type=float, default=10.0)
            p.add_argument("--checks", action="store_true")
    args = ap.parse_args()
    {"snapshot": cmd_snapshot, "restore": cmd_restore, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    main()
