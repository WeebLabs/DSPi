#!/usr/bin/env python3
"""
clkdiag_poll.py - poll the DSPi REQ_GET_CLOCK_DIAG (0x82) snapshot at 1 Hz,
decode the packed ClockDiagPacket, print CLKDIAG-style lines, and append raw
hex + decoded lines to clkdiag.log in the current directory.

No console backend exists in the shipped firmware (USB CDC off, UART stdio
off), so the 1 Hz CLKDIAG values are read over the vendor control endpoint.
Requires DSPI_CLOCK_DIAG=1 firmware and pyusb.

Usage: python3 clkdiag_poll.py [seconds]   (default 130)
"""
import os, sys, time, struct
import usb.core

VID, PID = 0x2E8B, 0xFEAA
BM_VENDOR_IN = 0xC1          # device -> host, vendor, interface
REQ_GET_CLOCK_DIAG = 0x82
VENDOR_IFACE = 2
NOM_SYS_HZ = 307_200_000     # soft-VCXO nominal sys_clk

LOGPATH = os.path.join(os.getcwd(), "clkdiag.log")

def find_dev():
    d = usb.core.find(idVendor=VID, idProduct=PID)
    if d is None:
        raise SystemExit(f"no DSPi at {VID:04X}:{PID:04X}")
    return d

def decode(buf):
    b = bytes(buf)
    o = 0
    def u(fmt):
        nonlocal o
        v = struct.unpack_from("<" + fmt, b, o)
        o += struct.calcsize("<" + fmt)
        return v
    d = {}
    d["version"], d["platform"], d["num_slots"], d["pll_lock"] = u("BBBB")
    N = d["num_slots"]
    (d["sys_last"], d["sys_min"], d["sys_max"], d["sys_n"]) = u("IIII")
    (d["vcxo_last"], d["vcxo_min"], d["vcxo_max"]) = u("iii")   # centi-ppm
    (d["vcxo_pulse_hz"], d["vcxo_pwm_wrap"]) = u("II")
    (d["vcxo_pwm_div"], d["vcxo_fbdiv_live"]) = u("HH")
    (d["vcxo_set"], d["vcxo_clamp"], d["vcxo_sflip"]) = u("III")
    (d["vcxo_fbdiv0"], d["vcxo_parked"]) = u("BB")
    (d["l_pulse_busy"], d["l_pulse_en"], d["l_pace_busy"], d["l_pace_en"]) = u("BBBB")
    (d["l_pulse_tcr"], d["l_pace_tcr"], d["l_pace_ctdreq"], d["l_pwm_moves"]) = u("IIII")
    (d["l_pwm_ctr"],) = u("H")
    (d["servo_actual_mhz"],) = u("I")
    (d["servo_ff"], d["servo_tot"], d["servo_min"], d["servo_max"],
     d["servo_int"], d["servo_ferr"], d["servo_slot"]) = u("iiiiiii")  # cppm/ints
    (d["servo_ticks"],) = u("I")
    d["slot_fill"]     = list(u(f"{N}B"))
    d["slot_min_fill"] = list(u(f"{N}B"))
    d["slot_is_i2s"]   = list(u(f"{N}B"))
    d["slot_starv"]    = list(u(f"{N}I"))
    d["slot_dwc"]      = list(u(f"{N}I"))
    (d["pdm_enabled"], d["pdm_ring_fill"], d["pdm_ring_min"], d["pdm_dma_fill"]) = u("BBBB")
    (d["pdm_ring_ur"], d["pdm_dma_ur"]) = u("II")
    (d["in_src"], d["in_state"], d["in_lock"], d["in_loss"]) = u("BBBB")
    (d["in_rate"], d["in_parity"]) = u("II")
    (d["in_fifo_pct"],) = u("H")
    (d["in_which_long"], d["_rsv"]) = u("BB")
    (d["in_hz_long_mhz"], d["in_span_ms"]) = u("II")
    (d["loop_gapmax_us"],) = u("I")
    d["_raw_len"] = len(b)
    return d

def hz_ppm(hz):
    return (hz - NOM_SYS_HZ) / (NOM_SYS_HZ / 1e6)

STATE = {0: "INACTIVE", 1: "ACQUIRING", 2: "LOCKED", 3: "RELOCKING"}

def fmt_block(t, d):
    N = d["num_slots"]
    L = []
    L.append(f"CLKDIAG sysclk: hz={d['sys_last']} min={d['sys_min']} max={d['sys_max']} "
             f"ppm={hz_ppm(d['sys_last']):+.2f}[{hz_ppm(d['sys_min']):+.2f}..{hz_ppm(d['sys_max']):+.2f}] "
             f"n={d['sys_n']} lock={d['pll_lock']}")
    L.append(f"CLKDIAG vcxo: ppm={d['vcxo_last']/100:+.2f}[{d['vcxo_min']/100:+.2f}..{d['vcxo_max']/100:+.2f}] "
             f"park={d['vcxo_parked']} fbdiv0={d['vcxo_fbdiv0']} pulseHz={d['vcxo_pulse_hz']} "
             f"div={d['vcxo_pwm_div']} wrap={d['vcxo_pwm_wrap']} set={d['vcxo_set']} "
             f"clamp={d['vcxo_clamp']} sflip={d['vcxo_sflip']} fbdivLive={d['vcxo_fbdiv_live']}")
    L.append(f"CLKDIAG vcxoloop: PULSE busy={d['l_pulse_busy']} en={d['l_pulse_en']} tcr={d['l_pulse_tcr']} | "
             f"PACE busy={d['l_pace_busy']} en={d['l_pace_en']} tcr={d['l_pace_tcr']} ctdreq={d['l_pace_ctdreq']} | "
             f"pwmctr={d['l_pwm_ctr']} moves={d['l_pwm_moves']}")
    L.append(f"CLKDIAG servo: in={d['servo_actual_mhz']/1000:.3f} ff={d['servo_ff']/100:+.2f} "
             f"tot={d['servo_tot']/100:+.2f}[{d['servo_min']/100:+.2f}..{d['servo_max']/100:+.2f}] "
             f"int={d['servo_int']/100:+.2f} ferr={d['servo_ferr']} slot={d['servo_slot']} ticks={d['servo_ticks']}")
    slots = " ".join(
        f"s{i}={'i2s' if d['slot_is_i2s'][i] else 'spd'} {d['slot_fill'][i]}/min{d['slot_min_fill'][i]} "
        f"starv={d['slot_starv'][i]} dwc={d['slot_dwc'][i]}" for i in range(N))
    L.append(f"CLKDIAG slots: {slots}")
    if d["pdm_enabled"]:
        L.append(f"CLKDIAG pdm: ringfill={d['pdm_ring_fill']}/min{d['pdm_ring_min']} "
                 f"dmafill={d['pdm_dma_fill']} ringur={d['pdm_ring_ur']} dmaur={d['pdm_dma_ur']}")
    else:
        L.append("CLKDIAG pdm: (disabled)")
    L.append(f"CLKDIAG input: src={d['in_src']} st={d['in_state']}({STATE.get(d['in_state'],'?')}) "
             f"rate={d['in_rate']} lk={d['in_lock']} ls={d['in_loss']} par={d['in_parity']} "
             f"fifo={d['in_fifo_pct']}% hzL={d['in_hz_long_mhz']/1000:.3f} "
             f"use={'L' if d['in_which_long'] else 'S'} span={d['in_span_ms']}ms")
    L.append(f"CLKDIAG loop: gapmax={d['loop_gapmax_us']}us")
    return L

def main():
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 130
    dev = find_dev()
    t0 = time.time()
    n = 0
    with open(LOGPATH, "a") as f:
        hdr = f"\n===== CLKDIAG live capture start {time.strftime('%Y-%m-%d %H:%M:%S')} dur={dur}s =====\n"
        f.write(hdr); f.flush(); print(hdr.strip())
        while time.time() - t0 < dur:
            try:
                buf = dev.ctrl_transfer(BM_VENDOR_IN, REQ_GET_CLOCK_DIAG, 0, VENDOR_IFACE, 256)
            except usb.core.USBError as e:
                line = f"[{time.time()-t0:6.1f}s] USBError: {e} (retrying)"
                print(line); f.write(line + "\n"); f.flush()
                time.sleep(1.0)
                try: dev = find_dev()
                except SystemExit: pass
                continue
            d = decode(buf)
            ts = time.time() - t0
            raw = bytes(buf).hex()
            f.write(f"[{ts:6.1f}s] raw({d['_raw_len']}B)={raw}\n")
            block = fmt_block(ts, d)
            stamp = f"[{ts:6.1f}s]"
            for i, ln in enumerate(block):
                out = (stamp + " " + ln) if i == 0 else ("        " + " " + ln)
                print(out); f.write(out + "\n")
            f.write("\n"); f.flush()
            n += 1
            # pace to ~1 Hz (firmware latches once per second)
            nxt = t0 + n
            slp = nxt - time.time()
            if slp > 0: time.sleep(slp)
    print(f"captured {n} snapshots over {time.time()-t0:.1f}s -> {LOGPATH}")

if __name__ == "__main__":
    main()
