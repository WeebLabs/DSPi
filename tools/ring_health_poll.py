#!/usr/bin/env python3
"""Poll DSPi output-ring health and S/PDIF receiver status twice a second.

Shows the counters Console does not: underrun episodes, admission drops,
lowest pre-write fill, worst input delivery gap and background overruns
(REQ_GET_STATUS 26-31), next to each slot's sample-granular fill.
Needs pyusb (and libusb). On Linux, run with access to the DSPi USB device.

Usage: python3 ring_health_poll.py [seconds]   (default 60)
"""
import struct, sys, time
import usb.core

VID, PID = 0x2E8B, 0xFEAA
REQ_IN, IFACE = 0xC1, 2
REQ_GET_STATUS, REQ_GET_BUFFER_STATS, REQ_GET_SPDIF_RX_STATUS = 0x50, 0xB0, 0xE2


def get(dev, req, wvalue, n):
    return bytes(dev.ctrl_transfer(REQ_IN, req, wvalue, IFACE, n, timeout=500))


def stat(dev, idx):
    return struct.unpack("<I", get(dev, REQ_GET_STATUS, idx, 4))[0]


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit(f"no DSPi at {VID:04X}:{PID:04X}")
    print("t_s  rx  parity  lock/loss  in_hz  vcxo_ppm | underrun_eps  drops  min_prewrite  worst_in_gap_us  bg_over | fill_frames per slot")
    t0 = time.time()
    while time.time() - t0 < secs:
        rx = get(dev, REQ_GET_SPDIF_RX_STATUS, 0, 16)
        state, _src, locks, losses = rx[0], rx[1], rx[2], rx[3]
        parity = struct.unpack_from("<I", rx, 8)[0]
        under, drops, minpre, gap, bg = (stat(dev, i) for i in (26, 27, 28, 29, 31))
        # Status 32/33 exist from the 2026-11-05 firmware on; older builds read 0 for both.
        mhz = stat(dev, 32)
        in_hz = f"{mhz / 1000:.2f}" if mhz else "-"
        ppm = f"{struct.unpack('<i', struct.pack('<I', stat(dev, 33)))[0] / 100:+.2f}"
        b = get(dev, REQ_GET_BUFFER_STATS, 0, 44)
        # fill_centi_pct is 0.01% of a 768-frame scale (bytes 6-7 of each slot).
        fills = [struct.unpack_from("<H", b, 4 + 8 * i + 6)[0] * 768 // 10000 for i in range(b[0])]
        minpre_s = "-" if minpre == 0xFFFFFFFF else str(minpre)
        print(f"{time.time() - t0:5.1f} {state:3d} {parity:7d} {locks:4d}/{losses:<4d} {in_hz:>9} {ppm:>8} | "
              f"{under:12d} {drops:6d} {minpre_s:>13} {gap:16d} {bg:8d} | {fills}", flush=True)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
