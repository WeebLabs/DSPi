#!/usr/bin/env python3
"""Run the on-device AC-3 decode bench and print the result in CPU terms.

Needs a DSPI_AC3_BENCH firmware (build-rp2350-ac3bench*) and pyusb, like
tools/dspi_poke.  Starts a run of N passes over the embedded frame set,
polls REQ_AC3_BENCH (0x9F) until it finishes, then reports microseconds per
frame and the share of one core that real-time decode would take.

Usage: bench_poll.py [--passes N]
"""

import argparse
import struct
import sys
import time

import usb.core

VID, PID = 0x2E8B, 0xFEAA
BM_VENDOR_IN = 0xC0
REQ_AC3_BENCH = 0x9F
FMT = "<BBBBIIIIIIIIII"      # Ac3BenchResult, 44 bytes
FRAME_SAMPLES = 1536


def read(dev, wvalue=0):
    buf = bytes(dev.ctrl_transfer(BM_VENDOR_IN, REQ_AC3_BENCH, wvalue, 0, 44, timeout=2000))
    if len(buf) != 44:
        sys.exit("got %d bytes: not a DSPI_AC3_BENCH firmware?" % len(buf))
    return dict(zip(("state", "ram_hot", "frames", "checksum_ok", "passes", "total_us",
                     "min_frame_us", "max_frame_us", "header_us", "block_us", "sys_clk_hz",
                     "checksum", "expected", "errors"), struct.unpack(FMT, buf)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--passes", type=int, default=50)
    a = ap.parse_args()
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit("no DSPi found")

    read(dev, a.passes)
    # The run blocks the main loop, so poll gently.
    for _ in range(200):
        time.sleep(0.1)
        r = read(dev)
        if r["state"] == 2 and r["passes"] == a.passes:
            break
    else:
        sys.exit("bench did not finish")

    frames = r["frames"] * r["passes"]
    us_frame = r["total_us"] / frames
    mhz = r["sys_clk_hz"] / 1e6
    # Real-time budget: one frame per 1536 samples at 48 kHz = 32 ms.
    rt_pct = 100.0 * us_frame / (FRAME_SAMPLES / 48000.0 * 1e6)
    print("variant: %s, sys_clk %.1f MHz, %d frames x %d passes, errors %d"
          % ("RAM-hot" if r["ram_hot"] else "XIP-only", mhz, r["frames"], r["passes"], r["errors"]))
    print("checksum: 0x%08x expected 0x%08x -> %s"
          % (r["checksum"], r["expected"], "OK" if r["checksum_ok"] else "MISMATCH"))
    print("per frame: mean %.0f us (min %d, max %d) = %.2f M cycles"
          % (us_frame, r["min_frame_us"], r["max_frame_us"], us_frame * mhz / 1e6))
    print("  header %.0f us, blocks %.0f us (%.0f us per block)"
          % (r["header_us"] / frames, r["block_us"] / frames, r["block_us"] / frames / 6))
    print("real-time share of one core at 48 kHz: %.1f %%" % rt_pct)
    return 0


if __name__ == "__main__":
    sys.exit(main())
