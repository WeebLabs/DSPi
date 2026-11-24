#!/usr/bin/env python3
"""Bench driver for the shaped soft VCXO (DSPI_VCXO_SHAPED + DSPI_CLOCK_DIAG builds only).

Talks to DIAG.VCXO_STATUS / VCXO_FORCE / VCXO_DELAY (soft_vcxo_spec.md section 8.7) and
measures clk_sys against the crystal by averaging the clock-diag fc0 gates.
Run with /Users/weeblabs/venv/bin/python3 (pyusb).

  status                          decoded DIAG.VCXO_STATUS
  force PPM | force off           run a fixed trim in place of the servo, or release it
  delay D                         set the pacer delay (40..59, default 50); a running pacer restarts on it
  measure [SECONDS]               fc0 ppm, modulator ppm, replays and fill cost over a window
  sweep PPM [DMIN DMAX SECONDS]   per delay d: measured vs commanded ppm at +PPM and -PPM
  linearity SECONDS PPM [PPM...]  measured vs commanded ppm at each forced trim
"""
import struct
import sys
import time

import usb.core

VID, PID = 0x2E8B, 0xFEAA
BM_IN, BM_OUT = 0xC1, 0x41
IFACE = 2
SYS_PAGE, DIAG_PAGE = 0x01, 0xF0
CMD_PAGE_COMMANDS = 0x12
CMD_STATUS, CMD_FORCE, CMD_DELAY = 0x60, 0x61, 0x62
NOM_SYS_HZ = 307_200_000
FS_PPM = 1e6 / (128 * 6)            # one pulse every decision
FC0_HZ_PER_LSB = 1000.0 / 32        # fc0 result is kHz with 5 fraction bits

STATUS_FMT = "<BBBBfffiIIIIQqQIIiiIHBBIQII"
STATUS_KEYS = ("version state delay_d forced force_ppm servo_ppm run_ppm u_q24 starts stops replays "
               "fill_calls decisions net_pulses fill_cycles lead_min_bytes fill_gap_max_us i1_peak "
               "i2_peak sat_hits fbdiv_live sm_enabled dma_busy ring_bytes fc0_sum fc0_gates overtaken").split()
STATES = {0: "INERT", 1: "PARKED", 2: "RUNNING", 3: "DRAINING"}


def dev_open():
    d = usb.core.find(idVendor=VID, idProduct=PID)
    if d is None:
        raise SystemExit(f"no DSPi at {VID:04X}:{PID:04X}")
    bm = bytes(d.ctrl_transfer(BM_IN, CMD_PAGE_COMMANDS, DIAG_PAGE, (SYS_PAGE << 8) | IFACE, 64, timeout=1000))
    if not bm[CMD_STATUS >> 3] >> (CMD_STATUS & 7) & 1:
        raise SystemExit("DIAG.VCXO_STATUS is absent: flash a -DDSPI_VCXO_SHAPED=ON -DDSPI_CLOCK_DIAG=ON build")
    return d


def get(d, cmd, n):
    return bytes(d.ctrl_transfer(BM_IN, cmd, 0, (DIAG_PAGE << 8) | IFACE, n, timeout=1000))


def put(d, cmd, data):
    d.ctrl_transfer(BM_OUT, cmd, 0, (DIAG_PAGE << 8) | IFACE, data, timeout=1000)


def status(d):
    b = get(d, CMD_STATUS, 256)
    n = struct.calcsize(STATUS_FMT)
    if len(b) < n or b[0] != 1:
        raise SystemExit(f"unexpected VCXO_STATUS ({len(b)} bytes, version {b[0] if b else '-'})")
    return dict(zip(STATUS_KEYS, struct.unpack_from(STATUS_FMT, b)))


def force(d, ppm):
    if ppm is None:
        put(d, CMD_FORCE, struct.pack("<Bf", 0, 0.0))
    else:
        put(d, CMD_FORCE, struct.pack("<Bf", 1, ppm))


def set_delay(d, v):
    put(d, CMD_DELAY, bytes([v]))


def measure(d, seconds):
    """Window averages between two status reads (the first read also restarts the windowed fields)."""
    a = status(d)
    time.sleep(seconds)
    b = status(d)
    gates = b["fc0_gates"] - a["fc0_gates"]
    dec = b["decisions"] - a["decisions"]
    r = {"state": STATES.get(b["state"], "?"), "d": b["delay_d"], "run_ppm": b["run_ppm"],
         "gates": gates, "replays": b["replays"] - a["replays"], "overtaken": b["overtaken"] - a["overtaken"],
         "starts": b["starts"] - a["starts"], "stops": b["stops"] - a["stops"],
         "lead_min": b["lead_min_bytes"], "gap_max_us": b["fill_gap_max_us"],
         "i1_peak": b["i1_peak"] / 2**24, "i2_peak": b["i2_peak"] / 2**24, "fbdiv": b["fbdiv_live"]}
    r["fc0_ppm"] = ((b["fc0_sum"] - a["fc0_sum"]) / gates * FC0_HZ_PER_LSB / NOM_SYS_HZ - 1) * 1e6 if gates else float("nan")
    r["mod_ppm"] = (b["net_pulses"] - a["net_pulses"]) / dec * FS_PPM if dec else 0.0
    cyc = b["fill_cycles"] - a["fill_cycles"]
    r["cyc_per_dec"] = cyc / dec if dec else float("nan")
    r["core0_pct"] = cyc / (seconds * NOM_SYS_HZ) * 100
    return r


def fmt(r):
    return (f"{r['state']:8s} d={r['d']} run={r['run_ppm']:+.4f} fc0={r['fc0_ppm']:+.4f} mod={r['mod_ppm']:+.4f} "
            f"gates={r['gates']} replays={r['replays']} overtaken={r['overtaken']} lead_min={r['lead_min']}B gap_max={r['gap_max_us']}us "
            f"i1={r['i1_peak']:.2f} i2={r['i2_peak']:.2f} cyc/dec={r['cyc_per_dec']:.1f} core0={r['core0_pct']:.2f}%")


def settle_measure(d, seconds):
    time.sleep(1.0)        # PLL settle plus the ring lead (3.4 ms) at the new command
    return measure(d, seconds)


def main(argv):
    if len(argv) < 2:
        raise SystemExit(__doc__)
    d = dev_open()
    cmd = argv[1]
    if cmd == "status":
        for k, v in status(d).items():
            print(f"{k:16s} {STATES.get(v, v) if k == 'state' else v}")
    elif cmd == "force":
        force(d, None if argv[2] == "off" else float(argv[2]))
    elif cmd == "delay":
        set_delay(d, int(argv[2]))
    elif cmd == "measure":
        print(fmt(measure(d, float(argv[2]) if len(argv) > 2 else 10.0)))
    elif cmd == "sweep":
        ppm = float(argv[2])
        dmin, dmax = (int(argv[3]), int(argv[4])) if len(argv) > 4 else (40, 59)
        secs = float(argv[5]) if len(argv) > 5 else 10.0
        force(d, 0.0)
        base = settle_measure(d, secs)
        print(f"base   {fmt(base)}")
        for dv in range(dmin, dmax + 1):
            set_delay(d, dv)
            for sgn in (1, -1):
                force(d, sgn * ppm)
                r = settle_measure(d, secs)
                err = r["fc0_ppm"] - base["fc0_ppm"] - r["mod_ppm"]
                print(f"d={dv:2d} {sgn * ppm:+9.3f}  err={err:+.4f} ppm ({err / ppm * 100:+.3f}%)  {fmt(r)}")
        force(d, None)
    elif cmd == "linearity":
        secs = float(argv[2])
        force(d, 0.0)
        base = settle_measure(d, secs)
        print(f"base   {fmt(base)}")
        for p in map(float, argv[3:]):
            force(d, p)
            r = settle_measure(d, secs)
            print(f"{p:+10.4f}  meas={r['fc0_ppm'] - base['fc0_ppm']:+.4f}  {fmt(r)}")
        force(d, None)
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv)
