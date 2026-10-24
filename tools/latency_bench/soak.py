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
from tools.dspi_test.device import DspiDevice, OP, Stall  # noqa: E402
from tools.dspi_test.profile import build_profile  # noqa: E402
from tools.dspi_test import lifecycle  # noqa: E402
from tools.dspi_test.actions import live_param_actions  # noqa: E402
from tools.batch_bench.batch_bench import Player  # noqa: E402
from tools.clkdiag_poll import decode as clkdiag_decode  # noqa: E402
from tools.latency_bench.latency_bench import (  # noqa: E402
    SRC_USB, SRC_SPDIF, SRC_I2S, latency_status, set_latency, set_source, status_u32, wait_rate)

SOURCES = {"usb": (SRC_USB, None), "spdif": (SRC_SPDIF, None), "i2s48": (SRC_I2S, 48000)}
CS_TYPE_POT, CS_TYPE_BUTTON = 3, 1
CS_NOUN_OUTPUT_GAIN, CS_NOUN_LOUDNESS = 17, 3   # never a mute noun: a pressed pin must not un-mute
CS_ACT_ADJUST, CS_ACT_TOGGLE = 0, 4
CS_STATUS_OK = 0
SLOW_PASS_US = 300          # worst-pass windows above this are logged as events
HOST_STALL_US = 5000        # a USB packet gap this long is the host's, not the device's
RTA_READ_S = 0.05           # Console's band polling cadence
# DSPI_LOOP_PROF builds (firmware/DSPi/loop_prof.h); normal builds read 0.
LOOP_SECTIONS = ["head", "usb+vendor", "notify", "uart/lg/dac", "control_surfaces", "siggen_service",
                 "rta", "audio_inputs+dsp", "flash_sets", "eq_updates", "coeff_updates", "deferred_ops",
                 "idle_pump", "delivery_2"]


def feedback_window(dev):
    """USB feedback min/max (Q16 frames per ms) since the last read, or None unless
    the firmware was built with DSPI_CLOCK_DIAG (REQ_GET_CLOCK_DIAG 0x82)."""
    try:
        buf = dev.get(0x82, 256)
    except Stall:
        return None
    d = clkdiag_decode(buf) if buf else {}
    return (d["fb_min"], d["fb_max"]) if "fb_min" in d else None


def take_sections(dev):
    return [status_u32(dev, 64 + i) for i in range(len(LOOP_SECTIONS))]


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
        toggles = [("crossfeed_on_off", lambda: dev.set_u8(OP.SET_CROSSFEED, rng.randrange(2))),
                   ("loudness_on_off", lambda: dev.set_u8(OP.SET_LOUDNESS, rng.randrange(2))),
                   ("leveller_on_off", lambda: dev.set_u8(OP.SET_LEVELLER_ENABLE, rng.randrange(2))),
                   ("subharm_on_off", lambda: dev.set_u8(OP.SET_SUBHARM, rng.randrange(2))),
                   ("psybass_on_off", lambda: dev.set_u8(OP.SET_PSYBASS, rng.randrange(2))),
                   ("limiter_on_off", lambda: dev.set(OP.LIMITER, struct.pack("<f", rng.randrange(2)),
                                                      wvalue=(rng.randrange(no) << 8) | 0)),
                   ("output_mute", lambda: dev.set_u8(OP.SET_OUTPUT_MUTE, rng.randrange(2),
                                                      wvalue=rng.randrange(no)))]
        host = [("bulk_read", lambda: dev.get_ready(OP.GET_ALL_PARAMS, max(prof.bulk_payload_len, 2960))),
                ("rta_reconfigure", self.rta_reconfigure),
                ("siggen_toggle", self.siggen_toggle),
                ("cs_bind_or_revert", self.cs_bind_or_revert)]
        # Weighted toward parameter edits, as a Console session is.
        self.actions = live * 3 + toggles * 2 + host

    # --- host-side behaviours --------------------------------------------------------
    def rta_read(self):
        try:
            self.dev.get(OP.RTA_GET_BANDS, 82, wvalue=self.rta_ch)
        except Stall:
            pass
        self.rta_ch = (self.rta_ch + 1) % self.prof.num_output_channels

    def rta_reconfigure(self):
        cfg = bytearray(self.dev.get(OP.RTA_GET_CONFIG, 12))
        cfg[4] = self.rng.choice((8, 9, 10))
        self.dev.set(OP.RTA_SET_CONFIG, bytes(cfg))

    def siggen_toggle(self):
        if self.siggen_on:
            self.dev.get(OP.SIGGEN_CONTROL, 1, wvalue=0)
        else:
            mask = 1 << self.rng.randrange(self.prof.num_output_channels)
            cfg = struct.pack("<BBHHBBfIHHffff", 1, self.rng.choice((0, 2, 4)), mask, 0, 0, 0, -40.0,
                              0, 0, 0, self.rng.uniform(40, 8000), 18000.0, 0, 0)
            self.dev.set(OP.SIGGEN_SET_CONFIG, cfg)
            self.dev.get(OP.SIGGEN_CONTROL, 1, wvalue=1)
        self.siggen_on = not self.siggen_on

    def cs_bind_or_revert(self):
        """Bind a pot (ADC pin, floating: noise drives real dispatches) or a button,
        or revert every binding this run made. Input-only types: no pin is driven."""
        if self.cs_slots and self.rng.random() < 0.4:
            self.dev.get(OP.CS_REVERT, 1)
            self.cs_slots = []
            return
        st = self.dev.get(OP.GET_CS_STATUS, 41)
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
        self.dev.set(OP.SET_CS_BINDING, b, wvalue=slot)
        time.sleep(0.05)
        st = self.dev.get(OP.GET_CS_STATUS, 41)
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
            self.dev.get(OP.SIGGEN_CONTROL, 1, wvalue=2)
        if self.cs_slots:
            self.dev.get(OP.CS_REVERT, 1)


def profile_actions(dev, soak, reps, out):
    """Each action alone, reps times: the worst input delivery gap in the second after it.
    The analyser is held stopped (manual mode) so its own steps do not mask the action."""
    res = {}
    cfg = bytearray(dev.get(OP.RTA_GET_CONFIG, 12))
    cfg[9] |= 0x01                                   # RTA_FLAG_MANUAL
    dev.set(OP.RTA_SET_CONFIG, bytes(cfg))
    dev.get(OP.RTA_CONTROL, 1, wvalue=0)             # RTA_CTL_STOP
    actions = {k: v for k, v in dict(soak.actions).items() if k != "rta_reconfigure"}
    actions["baseline"] = lambda: None
    for name, fn in actions.items():
        passes, sect = [], [0] * len(LOOP_SECTIONS)
        for _ in range(reps):
            time.sleep(0.6)
            dev.get(OP.RESET_BUFFER_STATS, 1, wvalue=1)
            take_sections(dev)
            try:
                fn()
            except Stall:
                pass
            time.sleep(1.0)
            passes.append(status_u32(dev, 29))
            sect = [max(a, b) for a, b in zip(sect, take_sections(dev))]
        top = sorted(zip(sect, LOOP_SECTIONS), reverse=True)[:3]
        res[name] = dict(worst_us=max(passes), median_us=sorted(passes)[len(passes) // 2],
                         sections=dict(zip(LOOP_SECTIONS, sect)))
        print(f"{name:20s} worst {res[name]['worst_us']:5d} us  median {res[name]['median_us']:5d} us  "
              f"top sections {top}", flush=True)
    soak.cleanup()
    (out / "action_profile.json").write_text(json.dumps(res, indent=1))


def run(args):
    dev = DspiDevice()
    prof = build_profile(dev)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cs = dev.get(OP.GET_CS_STATUS, 41)
    if cs[3]:
        raise SystemExit("Control Surfaces has an unsaved preview; the soak's revert would discard it")
    snap = lifecycle.capture(dev, prof)
    lat0 = latency_status(dev)["requested_id"]
    rta0 = bytes(dev.get(OP.RTA_GET_CONFIG, 12))
    (out / "soak_state.pkl").write_bytes(pickle.dumps(dict(snap=snap, lat=lat0, rta=rta0)))
    rng = random.Random(args.seed)
    soak = Soak(dev, prof, rng)
    src, rate = SOURCES[args.source]
    player = None
    dev.set_bytes(OP.SET_USER_MUTE, b"\x01")   # undriven inputs read as noise; mute is gain only
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
        # Underruns, drops, USB ring overruns, USB packet gaps of 2-50 ms, packets.
        counters = (26, 27, 22, 8, 10)
        c_prev = [status_u32(dev, w) for w in counters]
        uf_prev = player.underflows if player else 0
        dev.get(OP.RESET_BUFFER_STATS, 1, wvalue=1)
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
                c = [status_u32(dev, w) for w in counters]
                fb = feedback_window(dev)
                uf = player.underflows if player else 0
                worst, minpre = status_u32(dev, 29), status_u32(dev, 28)
                host_gap = status_u32(dev, 30)
                dev.get(OP.RESET_BUFFER_STATS, 1, wvalue=1)
                d = [a - b for a, b in zip(c, c_prev)] + [uf - uf_prev]
                c_prev, uf_prev = c, uf
                t = round(now - t0, 1)
                while soak.recent and soak.recent[0][0] < now - 2.5:
                    soak.recent.popleft()
                row = dict(t=t, underruns=d[0], drops=d[1], usb_ring_overruns=d[2], usb_packet_gaps=d[3],
                           usb_packets=d[4], host_underflows=d[5],
                           fb_min_q16=fb[0] if fb else None, fb_max_q16=fb[1] if fb else None,
                           worst_pass_us=worst, worst_usb_packet_gap_us=host_gap,
                           min_prewrite=None if minpre == 0xFFFFFFFF else minpre)
                rows.append(row)
                if d[0] or d[1] or d[2] or d[5] or worst > SLOW_PASS_US or host_gap > HOST_STALL_US:
                    ev = dict(row, preceding=[n for _, n in soak.recent])
                    events.append(ev)
                    print(json.dumps(ev), flush=True)
                next_sec += 1
            time.sleep(0.003)
    finally:
        soak.cleanup()
        if player:
            player.__exit__()
        set_latency(dev, lat0)
        dev.set(OP.RTA_SET_CONFIG, rta0)
        set_source(dev, snap.input_source)
        time.sleep(2)
        lifecycle.restore_live(dev, snap, settle_s=1.0)
        dev.set_bytes(OP.SET_USER_MUTE, bytes([snap.user_mute]))
    diffs = lifecycle.diff_bulk(dev, snap, prof)
    if args.profile_actions:
        print(f"restore bulk diffs {len(diffs)}")
        return
    ws = [r["worst_pass_us"] for r in rows]
    mp = [r["min_prewrite"] for r in rows if r["min_prewrite"] is not None]
    summary = dict(source=args.source, option=args.option, minutes=args.minutes, seed=args.seed,
                   target_frames=st["target_frames"], batch_frames=st["batch_frames"],
                   underruns=sum(r["underruns"] for r in rows), drops=sum(r["drops"] for r in rows),
                   usb_ring_overruns=sum(r["usb_ring_overruns"] for r in rows),
                   seconds_with_underrun=sum(1 for r in rows if r["underruns"]),
                   worst_pass_max_us=max(ws), worst_pass_p99_us=sorted(ws)[int(len(ws) * 0.99)],
                   min_prewrite=min(mp) if mp else None, actions=dict(counts),
                   events=len(events), restore_bulk_diffs=len(diffs))
    name = f"soak_{args.source}_opt{args.option}"
    (out / f"{name}.json").write_text(json.dumps(dict(summary=summary, events=events, rows=rows), indent=1))
    print(json.dumps(summary, indent=1), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--source", choices=sorted(SOURCES), required=True)
    ap.add_argument("--option", type=int, required=True, help="OutputLatencyId")
    ap.add_argument("--minutes", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--profile-actions", type=int, default=0, metavar="REPS",
                    help="instead of soaking, time each action alone REPS times")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
