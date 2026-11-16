#!/usr/bin/env python3
"""Poll DSPi output-ring health and S/PDIF receiver status twice a second (four struct reads per poll).
Shows what Console does not: underrun episodes, admission drops, lowest pre-write fill, worst input
delivery gap and background overruns, beside each slot's sample-granular fill. Needs pyusb.
Usage: python3 ring_health_poll.py [seconds]   (default 60)
"""
import struct, sys, time
import usb.core

VID, PID = 0x2E8B, 0xFEAA
BM_VENDOR_IN = 0xC1
SYS, INPUT = 0x01, 0x03
PROTOCOL_INFO, BUFFER_STATS, PIPELINE_STATS = 0x10, 0x42, 0x43   # SYS page
STATUS, SPDIF_RX_STATUS = 0x11, 0x20                              # INPUT page


def get(dev, page, cmd, n):
    return bytes(dev.ctrl_transfer(BM_VENDOR_IN, cmd, 0, (page << 8) | 2, n, timeout=500))


def probe(dev):
    # Firmware before 1.1.7 ignores the page and would run whichever old opcode shares the number.
    try:
        ok = get(dev, SYS, PROTOCOL_INFO, 16)[:4] == b"DSPi"
    except usb.core.USBError:
        ok = False
    if not ok:
        sys.exit("the device does not answer SYS.PROTOCOL_INFO (firmware 1.1.6 or older); "
                 "use the ring_health_poll.py from that release")


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit(f"no DSPi at {VID:04X}:{PID:04X}")
    probe(dev)
    print("t_s  rx  parity  lock/loss  in_hz  vcxo_ppm | underrun_eps  drops  min_prewrite  worst_in_gap_us  bg_over | fill_frames per slot")
    t0 = time.time()
    while time.time() - t0 < secs:
        rx = get(dev, INPUT, SPDIF_RX_STATUS, 16)
        state, _src, locks, losses = rx[0], rx[1], rx[2], rx[3]
        parity = struct.unpack_from("<I", rx, 8)[0]
        under, drops, minpre, gap, bg = struct.unpack_from("<5I", get(dev, SYS, PIPELINE_STATS, 24))
        mhz, cppm = struct.unpack_from("<Ii", get(dev, INPUT, STATUS, 12), 4)
        in_hz = f"{mhz / 1000:.2f}" if mhz else "-"
        ppm = f"{cppm / 100:+.2f}"
        b = get(dev, SYS, BUFFER_STATS, 44)
        # fill_centi_pct is 0.01 % of a 768-frame scale (bytes 6-7 of each slot).
        fills = [struct.unpack_from("<H", b, 4 + 8 * i + 6)[0] * 768 // 10000 for i in range(b[0])]
        minpre_s = "-" if minpre == 0xFFFFFFFF else str(minpre)
        print(f"{time.time() - t0:5.1f} {state:3d} {parity:7d} {locks:4d}/{losses:<4d} {in_hz:>9} {ppm:>8} | "
              f"{under:12d} {drops:6d} {minpre_s:>13} {gap:16d} {bg:8d} | {fills}", flush=True)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
