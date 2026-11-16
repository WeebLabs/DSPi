#!/usr/bin/env python3
"""Real-use soak: run one source at one latency option for many minutes while the
host behaves like a busy DSPi Console session, and attribute every underrun,
refused block or long input delivery gap to the actions that preceded it.
Method: tools/latency_bench/README.md "Soak".
"""

from __future__ import annotations

import argparse
import json
import pickle
import random
import struct
import sys
import time
from collections import Counter, deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.dspi_test.device import DspiDevice, Stall  # noqa: E402
from tools.dspi_test.profile import build_profile  # noqa: E402
from tools.dspi_test import lifecycle  # noqa: E402
from tools.dspi_test.actions import live_param_actions  # noqa: E402
from tools.latency_bench import loop_prof  # noqa: E402
from tools.latency_bench.latency_bench import (  # noqa: E402
    SRC_USB, SRC_SPDIF, SRC_I2S, Player, audio_available, latency_get, sample_rate, set_latency,
    set_source, wait_rate)
from tools.dspi_test.helpers import has_command  # noqa: E402
from tools.dspi_test.telemetry import pipeline_stats, spdif_tx_stats, usb_audio_status  # noqa: E402
from tools.latency_bench.paged import stats_reset  # noqa: E402

SOURCES = {"usb": (SRC_USB, None), "spdif": (SRC_SPDIF, None), "i2s48": (SRC_I2S, 48000)}
CS_TYPE_POT, CS_TYPE_BUTTON = 3, 1
CS_NOUN_OUTPUT_GAIN, CS_NOUN_LOUDNESS = 17, 3   # never a mute noun: a pressed pin must not un-mute
CS_ACT_ADJUST, CS_ACT_TOGGLE = 0, 4
CS_STATUS_OK = 0
SLOW_PASS_US = 300          # worst-pass windows above this are logged as events
HOST_STALL_US = 5000        # a USB packet gap this long is the host's, not the device's
RTA_READ_S = 0.05           # Console's band polling cadence


def feedback_window(dev):
    """USB feedback min/max (Q16 frames per ms) since the last read, or None unless
    the firmware was built with DSPI_CLOCK_DIAG (DIAG.CLOCK_DIAG)."""
    if getattr(dev, "_clock_diag", None) is None:
        dev._clock_diag = has_command(dev, "DIAG.CLOCK_DIAG")
    if not dev._clock_diag:
        return None
    from tools.clkdiag_poll import decode as clkdiag_decode
    try:
        buf = dev.pget("DIAG.CLOCK_DIAG", length=256)
    except Stall:
        return None
    d = clkdiag_decode(buf) if buf else {}
    return (d["fb_min"], d["fb_max"]) if "fb_min" in d else None


def second_counters(dev):
    """Underruns, drops, USB ring overruns, USB packet gaps of 2-50 ms, packets, background
    steps over budget; then the worst delivery gap, lowest pre-write fill and worst packet gap."""
    p, u, t = pipeline_stats(dev), usb_audio_status(dev), spdif_tx_stats(dev)
    c = [p["ring_underruns"], p["ring_drops"], u["ring_overruns"], t["spdif_underruns"],
         u["usb_audio_packets"], p["bg_over_budget_count"]]
    return c, p["input_delivery_worst_gap_us"], p["min_prewrite"], u["worst_packet_gap_us"]


def binding(kind, noun, action, gpio, target=0):
    """24-byte CsBinding (firmware/DSPi/control_surfaces.h)."""
    return struct.pack("<BBBBBBBBBBhhhhHHBB", kind, noun, action, 0, gpio, 0xFF, 0, target, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0)


class Soak:
    def __init__(self, dev, prof, rng):
        self.dev, self.prof, self.rng = dev, prof, rng
        self.recent = deque()          # (t, action name) for event attribution
        self.siggen_on = False
        self.cs_slots = []             # slots this run bound (reverted at the end)
        self.rta_ch = 0
        no = prof.num_output_channels
        live = live_param_actions(dev, prof, rng)

        def flag(cmd, per_output=False):
            # Value is drawn before the address. Reordering changes what a given seed replays.
            return lambda: dev.pset(cmd, bytes([rng.randrange(2)]),
                                    addr=rng.randrange(no) if per_output else 0)

        toggles = [("crossfeed_on_off", flag("FX.CROSSFEED_ENABLE")),
                   ("loudness_on_off", flag("DYN.LOUDNESS_ENABLE")),
                   ("leveller_on_off", flag("DYN.LEVELLER_ENABLE")),
                   ("subharm_on_off", flag("FX.SUBHARM_ENABLE")),
                   ("psybass_on_off", flag("FX.PSYBASS_ENABLE")),
                   ("limiter_on_off", lambda: dev.pset("DYN.LIMITER_ENABLE",
                                                       struct.pack("<f", rng.randrange(2)),
                                                       addr=rng.randrange(no))),
                   ("output_mute", flag("ROUTE.OUTPUT_MUTE", per_output=True))]
        host = [("bulk_read", lambda: dev.pget_ready("STORE.ALL_PARAMS",
                                                     length=max(prof.bulk_payload_len, 2960))),
                ("rta_reconfigure", self.rta_reconfigure),
                ("siggen_toggle", self.siggen_toggle),
                ("cs_bind_or_revert", self.cs_bind_or_revert)]
        # Weighted toward parameter edits, as a Console session is.
        self.actions = live * 3 + toggles * 2 + host

    # --- host-side behaviours --------------------------------------------------------
    def rta_read(self):
        try:
            self.dev.pget("MEAS.RTA_BANDS", addr=self.rta_ch, length=82)
        except Stall:
            pass
        self.rta_ch = (self.rta_ch + 1) % self.prof.num_output_channels

    def rta_reconfigure(self):
        cfg = bytearray(self.dev.pget("MEAS.RTA_CONFIG", length=12))
        cfg[4] = self.rng.choice((8, 9, 10))
        self.dev.pset("MEAS.RTA_CONFIG", cfg)

    def siggen_toggle(self):
        if self.siggen_on:
            self.dev.pset("MEAS.SIGGEN_STOP")
        else:
            mask = 1 << self.rng.randrange(self.prof.num_output_channels)
            cfg = struct.pack("<BBHHBBfIHHffff", 1, self.rng.choice((0, 2, 4)), mask, 0, 0, 0, -40.0,
                              0, 0, 0, self.rng.uniform(40, 8000), 18000.0, 0, 0)
            self.dev.pset("MEAS.SIGGEN_CONFIG", cfg)
            self.dev.pset("MEAS.SIGGEN_START")
        self.siggen_on = not self.siggen_on

    def cs_bind_or_revert(self):
        """Bind a pot (ADC pin, floating: noise drives real dispatches) or a button,
        or revert every binding this run made. Input-only types: no pin is driven."""
        if self.cs_slots and self.rng.random() < 0.4:
            self.dev.pset("CS.REVERT")
            self.cs_slots = []
            return
        st = self.dev.pget("CS.STATUS", length=41)
        active = struct.unpack_from("<H", st, 4)[0]
        free = [s for s in range(st[2]) if not active & (1 << s) and s not in self.cs_slots]
        if not free:
            return
        slot = free[-1]
        if self.rng.random() < 0.6:
            b = binding(CS_TYPE_POT, CS_NOUN_OUTPUT_GAIN, CS_ACT_ADJUST, self.rng.choice((26, 27, 28)),
                        self.rng.randrange(self.prof.num_output_channels))
        else:
            b = binding(CS_TYPE_BUTTON, CS_NOUN_LOUDNESS, CS_ACT_TOGGLE, self.rng.choice((14, 15, 16)))
        self.dev.pset("CS.BINDING", b, addr=slot)
        time.sleep(0.05)
        st = self.dev.pget("CS.STATUS", length=41)
        if st[6 + slot] == CS_STATUS_OK and struct.unpack_from("<H", st, 4)[0] & (1 << slot):
            self.cs_slots.append(slot)

    # --- run -----------------------------------------------------------------------
    def act(self, now):
        name, fn = self.rng.choice(self.actions)
        try:
            fn()
        except Stall:
            name += " (stalled)"
        self.recent.append((now, name))
        return name

    def cleanup(self):
        if self.siggen_on:
            self.dev.pset("MEAS.SIGGEN_STOP_NOW")
        if self.cs_slots:
            self.dev.pset("CS.REVERT")


def profile_actions(dev, soak, reps, out):
    """Each action alone, reps times: the worst input delivery gap in the second after it.
    The analyser is held stopped (manual mode) so its own steps do not mask the action."""
    res = {}
    cfg = bytearray(dev.pget("MEAS.RTA_CONFIG", length=12))
    cfg[9] |= 0x01                                   # RTA_FLAG_MANUAL
    dev.pset("MEAS.RTA_CONFIG", cfg)
    dev.pset("MEAS.RTA_STOP")
    actions = {k: v for k, v in dict(soak.actions).items() if k != "rta_reconfigure"}
    actions["baseline"] = lambda: None
    for name, fn in actions.items():
        passes, sect, jobs, ops = [], {}, {}, {}
        for _ in range(reps):
            time.sleep(0.6)
            stats_reset(dev)
            loop_prof.reset(dev)
            try:
                fn()
            except Stall:
                pass
            time.sleep(1.0)
            passes.append(pipeline_stats(dev)["input_delivery_worst_gap_us"])
            loop_prof.merge_worst(sect, loop_prof.sections(dev))
            loop_prof.merge_worst(jobs, loop_prof.jobs(dev))
            loop_prof.merge_worst(ops, loop_prof.commands(dev))
        res[name] = dict(worst_us=max(passes), median_us=sorted(passes)[len(passes) // 2],
                         sections=sect, jobs=jobs, commands=ops)
        top = sorted(((v, k) for k, v in sect.items()), reverse=True)[:3]
        top_ops = sorted(((v, k) for k, v in ops.items()), reverse=True)[:2]
        print(f"{name:20s} worst {res[name]['worst_us']:5d} us  median {res[name]['median_us']:5d} us  "
              f"top sections {top}  top commands {top_ops}", flush=True)
    soak.cleanup()
    (out / "action_profile.json").write_text(json.dumps(res, indent=1))


def run(args):
    dev = DspiDevice()
    prof = build_profile(dev)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    if args.source == "usb" and not audio_available():
        raise SystemExit("--source usb plays to the device from the host and needs sounddevice")
    cs = dev.pget("CS.STATUS", length=41)
    if cs[3]:
        raise SystemExit("Control Surfaces has an unsaved preview; the soak's revert would discard it")
    snap = lifecycle.capture(dev, prof)
    lat0 = latency_get(dev)
    rta0 = dev.pget("MEAS.RTA_CONFIG", length=12)
    (out / "soak_state.pkl").write_bytes(pickle.dumps(dict(snap=snap, lat=lat0, rta=rta0)))
    rng = random.Random(args.seed)
    soak = Soak(dev, prof, rng)
    src, rate = SOURCES[args.source]
    player = None
    dev.pset("ROUTE.USER_MUTE", b"\x01")   # undriven inputs read as noise; mute is gain only
    rows, events, counts = [], [], Counter()
    try:
        set_source(dev, src, rate)
        if src == SRC_USB:
            player = Player().__enter__()
        time.sleep(4)
        if rate:
            wait_rate(dev, rate)
        st = set_latency(dev, args.option)
        time.sleep(3)
        if args.profile_actions:
            profile_actions(dev, soak, args.profile_actions, out)
            return
        fs = sample_rate(dev) or 48000
        gap_limit_us = (st["target_frames"] - st["batch_frames"]) * 1_000_000 // fs
        loop_prof.reset(dev)                   # the run's profiler worsts start from zero
        c_prev = second_counters(dev)[0]
        uf_prev = player.underflows if player else 0
        stats_reset(dev)
        t0 = time.time()
        end = t0 + args.minutes * 60
        next_action, next_read, next_sec = t0 + 1, t0, t0 + 1
        while time.time() < end:
            now = time.time()
            if now >= next_read:
                soak.rta_read()
                next_read += RTA_READ_S
            if now >= next_action:
                counts[soak.act(now)] += 1
                next_action = now + rng.uniform(1.0, 3.0)
            if now >= next_sec:
                c, worst, minpre, host_gap = second_counters(dev)
                fb = feedback_window(dev)
                uf = player.underflows if player else 0
                stats_reset(dev)
                d = [a - b for a, b in zip(c, c_prev)] + [uf - uf_prev]
                c_prev, uf_prev = c, uf
                t = round(now - t0, 1)
                while soak.recent and soak.recent[0][0] < now - 2.5:
                    soak.recent.popleft()
                row = dict(t=t, underruns=d[0], drops=d[1], usb_ring_overruns=d[2], usb_packet_gaps=d[3],
                           usb_packets=d[4], over_budget_steps=d[5], host_underflows=d[6],
                           fb_min_q16=fb[0] if fb else None, fb_max_q16=fb[1] if fb else None,
                           worst_pass_us=worst, worst_usb_packet_gap_us=host_gap,
                           min_prewrite=None if minpre == 0xFFFFFFFF else minpre)
                rows.append(row)
                if (d[0] or d[1] or d[2] or d[5] or d[6] or worst > SLOW_PASS_US
                        or host_gap > HOST_STALL_US):
                    ev = dict(row, preceding=[n for _, n in soak.recent])
                    events.append(ev)
                    print(json.dumps(ev), flush=True)
                next_sec += 1
            time.sleep(0.003)
        # Whole-run worsts; empty on builds without DSPI_LOOP_PROF.
        prof_worst = dict(sections=loop_prof.sections(dev), jobs=loop_prof.jobs(dev),
                          commands=loop_prof.commands(dev))
    finally:
        soak.cleanup()
        if player:
            player.__exit__()
        set_latency(dev, lat0)
        dev.pset("MEAS.RTA_CONFIG", rta0)
        set_source(dev, snap.input_source)
        time.sleep(2)
        lifecycle.restore_live(dev, snap, settle_s=1.0)
        dev.pset("ROUTE.USER_MUTE", bytes([snap.user_mute]))
    diffs = lifecycle.diff_bulk(dev, snap, prof)
    if args.profile_actions:
        print(f"restore bulk diffs {len(diffs)}")
        return
    ws = [r["worst_pass_us"] for r in rows]
    mp = [r["min_prewrite"] for r in rows if r["min_prewrite"] is not None]
    # A second the host stopped sending is the host's; its underruns are reported, not failed.
    host = [r for r in rows if r["worst_usb_packet_gap_us"] > HOST_STALL_US]
    own = [r for r in rows if r["worst_usb_packet_gap_us"] <= HOST_STALL_US]
    checks = dict(no_underruns=sum(r["underruns"] for r in own) == 0,
                  no_drops=sum(r["drops"] for r in rows) == 0,
                  no_over_budget_steps=sum(r["over_budget_steps"] for r in rows) == 0,
                  delivery_gap_under_limit=all(r["worst_pass_us"] < gap_limit_us for r in own),
                  prewrite_never_zero=(min(mp) > 0) if mp else True)
    summary = dict(source=args.source, option=args.option, minutes=args.minutes, seed=args.seed,
                   target_frames=st["target_frames"], batch_frames=st["batch_frames"],
                   underruns=sum(r["underruns"] for r in rows), drops=sum(r["drops"] for r in rows),
                   usb_ring_overruns=sum(r["usb_ring_overruns"] for r in rows),
                   seconds_with_underrun=sum(1 for r in rows if r["underruns"]),
                   over_budget_steps=sum(r["over_budget_steps"] for r in rows),
                   host_stall_seconds=len(host), host_stall_underruns=sum(r["underruns"] for r in host),
                   gap_limit_us=gap_limit_us,
                   worst_pass_max_us=max(ws), worst_pass_p99_us=sorted(ws)[int(len(ws) * 0.99)],
                   min_prewrite=min(mp) if mp else None, actions=dict(counts),
                   events=len(events), restore_bulk_diffs=len(diffs),
                   checks=checks, passed=all(checks.values()), profiler=prof_worst)
    name = f"soak_{args.source}_opt{args.option}"
    (out / f"{name}.json").write_text(json.dumps(dict(summary=summary, events=events, rows=rows), indent=1))
    print(json.dumps(summary, indent=1), flush=True)
    return summary["passed"]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--source", choices=sorted(SOURCES), required=True)
    ap.add_argument("--option", type=int, required=True, help="OutputLatencyId")
    ap.add_argument("--minutes", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--profile-actions", type=int, default=0, metavar="REPS",
                    help="instead of soaking, time each action alone REPS times")
    # Exit status 1 when a soak fails its checks, so scripted runs stop on it.
    if run(ap.parse_args()) is False:
        sys.exit(1)


if __name__ == "__main__":
    main()
