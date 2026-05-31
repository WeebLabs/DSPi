#!/usr/bin/env python3
"""
diag_reenum.py — isolate the heavy DSPi operations that cause USB re-enumeration
and classify each event as a benign bus reset vs a true device reboot.

Discriminator: `buffer_stats_sequence` (REQ_GET_BUFFER_STATS bytes 2-3) is a RAM
counter that increments once per read and is zero-initialised at firmware boot.
  * Across a benign USB bus reset (firmware keeps running) it keeps climbing.
  * Across an MCU reboot it resets to ~0, so it jumps "backward" by a huge amount.

For each operation we repeat N times and record: re-enumerations (handle had to
be re-acquired), reboots (sequence counter went backward), and whether a distinct
live value we wrote survived the operation.

    python3 -m tools.dspi_test.diag_reenum [N]
"""

import struct
import sys
import time

from .device import DspiDevice, OP, Stall, Disconnected
from .profile import build_profile
from . import lifecycle


def seq(dev):
    return struct.unpack_from("<H", dev.get_ready(OP.GET_BUFFER_STATS, 44), 2)[0]


def rebooted(s0, s1):
    # Normal: s1 == s0 + 1.  Reboot: s1 resets near 0 -> wrapped delta is huge.
    return ((s1 - s0) & 0xFFFF) > 0x8000


def measure(dev, name, do_op, repeats, probe_live=True):
    events = []
    for _ in range(repeats):
        # Stamp a distinctive live master-volume value (not a typical preset value).
        mv_probe = -77.0
        persisted = None
        if probe_live:
            try:
                dev.set_f32(OP.SET_MASTER_VOLUME, mv_probe)
            except (Stall, Disconnected):
                pass
        re0 = dev.reenumerations
        s0 = seq(dev)
        t0 = time.monotonic()
        do_op()
        dev.wait_ready()
        dt = (time.monotonic() - t0) * 1000.0
        s1 = seq(dev)
        re = dev.reenumerations - re0
        rb = rebooted(s0, s1)
        if probe_live:
            try:
                persisted = abs(dev.get_f32(OP.GET_MASTER_VOLUME) - mv_probe) < 1e-3
            except (Stall, Disconnected):
                persisted = None
        events.append((re, rb, persisted, dt, s0, s1))
        time.sleep(0.1)
    reenum = sum(1 for e in events if e[0] > 0)
    reboots = sum(1 for e in events if e[1])
    reverted = sum(1 for e in events if e[2] is False)
    avg_dt = sum(e[3] for e in events) / len(events)
    print(f"  {name:<28} runs={len(events)}  re-enum={reenum}  reboot={reboots}  "
          f"live-reverted={reverted}  avg={avg_dt:6.0f}ms")
    for re, rb, pers, dt, s0, s1 in events:
        tag = "REBOOT" if rb else ("re-enum" if re else "ok")
        pj = {True: "kept", False: "REVERTED", None: "?"}[pers]
        print(f"      {tag:<8} seq {s0}->{s1}  live={pj}  {dt:5.0f}ms")
    return reenum, reboots


def main(argv=None):
    n = int(argv[0]) if argv else 6
    dev = DspiDevice()
    profile = build_profile(dev)
    print(f"Device: {profile.summary()}")
    print(f"Probing each heavy op x{n} (reboot = buffer_stats_sequence reset to ~0)\n")
    snap = lifecycle.capture(dev, profile)

    def op_bulk():
        dev.set(OP.SET_ALL_PARAMS, snap.bulk)

    def op_type_switch():
        cur = dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=0)
        tgt = 1 - cur
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(tgt << 8) | 0)
        _poll(dev, OP.GET_OUTPUT_TYPE, 0, tgt)
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(cur << 8) | 0)
        _poll(dev, OP.GET_OUTPUT_TYPE, 0, cur)

    def op_input_switch():
        cur = dev.get_u8(OP.GET_INPUT_SOURCE)
        other = 1 - cur if cur in (0, 1) else 0
        dev.set_u8(OP.SET_INPUT_SOURCE, other)
        _poll(dev, OP.GET_INPUT_SOURCE, None, other)
        dev.set_u8(OP.SET_INPUT_SOURCE, cur)
        _poll(dev, OP.GET_INPUT_SOURCE, None, cur)

    def op_dac_config():
        cfg = bytearray(snap.dac_mute_cfg)
        # Toggle hold_ms between two valid values to force a real directory write.
        h = struct.unpack_from("<H", cfg, 4)[0]
        struct.pack_into("<H", cfg, 4, 5 if h != 5 else 7)
        if cfg[0] == 0:
            cfg[0] = 1  # ensure enabled so the write path is exercised
            if cfg[2] == 0xFF:
                cfg[2] = 11
        dev.set(OP.SET_DAC_HW_MUTE_CONFIG, bytes(cfg))

    def op_preset_load():
        dev.get(OP.PRESET_LOAD, 1, wvalue=snap.active_slot)

    from .tests.outputs import _free_pin

    def op_pin_move():
        free = _free_pin(dev, profile)
        if free is None:
            return
        orig = dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=0)
        dev.get_u8(OP.SET_OUTPUT_PIN, wvalue=(free << 8) | 0)
        time.sleep(0.05)
        dev.get_u8(OP.SET_OUTPUT_PIN, wvalue=(orig << 8) | 0)

    try:
        measure(dev, "bulk_apply (0xA1)", op_bulk, n, probe_live=False)
        measure(dev, "output_type_switch (0xC0)", op_type_switch, n)
        measure(dev, "input_source_switch (0xE0)", op_input_switch, n, probe_live=False)
        measure(dev, "dac_config_write (0xEA)", op_dac_config, n)
        measure(dev, "output_pin_move (0x7C)", op_pin_move, n)
        measure(dev, "preset_load (0x91)", op_preset_load, n, probe_live=False)
    finally:
        print("\nRestoring...")
        dev.wait_ready()
        # Restore DAC config and input source explicitly, then full live state.
        try:
            dev.set(OP.SET_DAC_HW_MUTE_CONFIG, bytes(snap.dac_mute_cfg))
            dev.wait_ready()
        except Exception:  # noqa: BLE001
            pass
        lifecycle.restore_live(dev, snap)
        diffs = lifecycle.diff_bulk(dev, snap, profile)
        print(f"  live restore: {'byte-for-byte' if not diffs else f'{len(diffs)} bytes differ'}; "
              f"total re-enumerations this run: {dev.reenumerations}")


def _poll(dev, get_op, wv, want, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            v = dev.get_u8(get_op, wvalue=(wv or 0))
            if v == want:
                return True
        except (Stall, Disconnected):
            pass
        time.sleep(0.03)
    return False


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
