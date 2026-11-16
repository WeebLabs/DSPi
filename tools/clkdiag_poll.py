#!/usr/bin/env python3
"""Poll DIAG.CLOCK_DIAG at 1 Hz, print CLKDIAG lines and append raw hex plus decoded lines to
./clkdiag.log. Needs a -DDSPI_CLOCK_DIAG=ON build and pyusb. Decodes ClockDiagPacket versions 1
and 2 from firmware/DSPi/clock_diag.h. Usage: python3 clkdiag_poll.py [seconds] (default 130)
"""
import os, sys, time, struct
import usb.core

VID, PID = 0x2E8B, 0xFEAA
BM_VENDOR_IN = 0xC1          # device -> host, vendor, interface
SYS_PAGE, DIAG_PAGE = 0x01, 0xF0
CMD_PROTOCOL_INFO, CMD_PAGE_COMMANDS, CMD_LAST_RESULT = 0x10, 0x12, 0x14
CMD_CLOCK_DIAG = 0x10        # DIAG.CLOCK_DIAG, absent unless built with DSPI_CLOCK_DIAG
DECODER_VERSION = 2          # newest ClockDiagPacket.version decoded here
NOM_SYS_HZ = 307_200_000     # soft-VCXO nominal sys_clk, both platforms
VC_NAMES = {0x10: "VC_ERR_UNKNOWN", 0x11: "VC_ERR_DIRECTION", 0x13: "VC_ERR_ADDRESS",
            0x16: "VC_ERR_BLOCKED", 0x17: "VC_ERR_BUSY"}

LOGPATH = os.path.join(os.getcwd(), "clkdiag.log")
NOT_DIAG = ("this firmware is not a clock-diag build: DIAG.CLOCK_DIAG is absent.  Configure with "
            "-DDSPI_CLOCK_DIAG=ON (for example build-rp2350-clkdiag) and flash that build.")


def is_stall(e):
    return getattr(e, "errno", None) == 32 or "pipe" in str(e).lower()


def get(dev, page, cmd, addr, n):
    return bytes(dev.ctrl_transfer(BM_VENDOR_IN, cmd, addr, (page << 8) | 2, n, timeout=1000))


def find_dev():
    d = usb.core.find(idVendor=VID, idProduct=PID)
    if d is None:
        raise SystemExit(f"no DSPi at {VID:04X}:{PID:04X}")
    return d


def last_result(dev, cmd_id):
    """VC_* name of the last refusal when SYS.LAST_RESULT describes cmd_id as a GET."""
    try:
        rid, is_get, res, _addr = struct.unpack_from("<HBBH", get(dev, SYS_PAGE, CMD_LAST_RESULT, 0, 8))
    except usb.core.USBError:
        return None
    return VC_NAMES.get(res, f"0x{res:02X}") if (rid, is_get) == (cmd_id, 1) else None


def probe(dev):
    """PROTOCOL_INFO first (older firmware ignores the page), then the DIAG page bitmap."""
    try:
        r = get(dev, SYS_PAGE, CMD_PROTOCOL_INFO, 0, 16)
    except usb.core.USBError as e:
        if is_stall(e):
            raise SystemExit("the device STALLed SYS.PROTOCOL_INFO: firmware 1.1.6 or older, which "
                             "this tool does not support.  Use the clkdiag_poll.py from that release.")
        raise SystemExit(f"USB error on the SYS.PROTOCOL_INFO probe: {e}")
    if r[:4] != b"DSPi":
        raise SystemExit(f"SYS.PROTOCOL_INFO without the DSPi magic ({r.hex(' ')}); refusing")
    try:
        bm = get(dev, SYS_PAGE, CMD_PAGE_COMMANDS, DIAG_PAGE, 64)
    except usb.core.USBError as e:
        if is_stall(e):
            raise SystemExit(NOT_DIAG)   # VC_ERR_ADDRESS: no DIAG page on this build
        raise SystemExit(f"USB error reading SYS.PAGE_COMMANDS: {e}")
    if not bm[CMD_CLOCK_DIAG >> 3] >> (CMD_CLOCK_DIAG & 7) & 1:
        raise SystemExit(NOT_DIAG)

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
    if d["version"] >= 2:
        (d["fb"], d["fb_min"], d["fb_max"], d["fb_iir"], d["fb_long"],
         d["fb_span_ms"]) = u("IIIIII")
        (d["fb_servo"], d["fb_ferr"]) = u("ii")
        (d["fb_upd"], d["fb_merged"], d["fb_rej"], d["fb_reanch"]) = u("IIII")
    d["_raw_len"], d["_used"] = len(b), o
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
    if d["version"] >= 2:
        q = lambda v: v / 65536.0
        # The packet carries no USB nominal rate; take the nearest standard rate.
        nom = min((44.1, 48.0, 88.2, 96.0), key=lambda n: abs(n - q(d["fb_iir"])))
        ppm = (lambda v: f"{(q(v) / nom - 1) * 1e6:+.1f}ppm") if d["fb_iir"] else (lambda v: "-")
        L.append(f"CLKDIAG usbfb: fb={q(d['fb']):.6f}({ppm(d['fb'])})[{q(d['fb_min']):.6f}..{q(d['fb_max']):.6f}] "
                 f"iir={q(d['fb_iir']):.6f} long={q(d['fb_long']):.6f}{'(' + ppm(d['fb_long']) + ')' if d['fb_long'] else ''} "
                 f"span={d['fb_span_ms']}ms servo={q(d['fb_servo']):+.6f} "
                 f"ferr={q(d['fb_ferr']) * 48:+.1f}fr upd={d['fb_upd']} merged={d['fb_merged']} "
                 f"rej={d['fb_rej']} reanch={d['fb_reanch']}")
    return L

def main():
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 130
    dev = find_dev()
    probe(dev)
    t0 = time.time()
    n = 0
    warned = False
    with open(LOGPATH, "a") as f:
        hdr = f"\n===== CLKDIAG live capture start {time.strftime('%Y-%m-%d %H:%M:%S')} dur={dur}s =====\n"
        f.write(hdr); f.flush(); print(hdr.strip())
        while time.time() - t0 < dur:
            try:
                buf = get(dev, DIAG_PAGE, CMD_CLOCK_DIAG, 0, 256)
            except usb.core.USBError as e:
                if is_stall(e):
                    why = last_result(dev, (DIAG_PAGE << 8) | CMD_CLOCK_DIAG)
                    if why == "VC_ERR_UNKNOWN":
                        raise SystemExit(NOT_DIAG)
                    line = f"[{time.time()-t0:6.1f}s] STALL ({why or 'reason unknown'}), retrying"
                else:
                    line = f"[{time.time()-t0:6.1f}s] USBError: {e} (retrying)"
                print(line); f.write(line + "\n"); f.flush()
                time.sleep(1.0)
                try:
                    dev = find_dev(); probe(dev)
                except SystemExit as x:
                    if str(x) == NOT_DIAG:
                        raise
                continue
            d = decode(buf)
            if not warned and (d["version"] > DECODER_VERSION or d["_used"] != d["_raw_len"]):
                warned = True
                w = (f"warning: packet version {d['version']}, {d['_raw_len']} bytes, decoder read "
                     f"{d['_used']} (knows version {DECODER_VERSION}); check clock_diag.h")
                print(w); f.write(w + "\n")
            ts = time.time() - t0
            raw = buf.hex()
            f.write(f"[{ts:6.1f}s] raw({d['_raw_len']}B)={raw}\n")
            block = fmt_block(ts, d)
            stamp = f"[{ts:6.1f}s]"
            for i, ln in enumerate(block):
                out = (stamp + " " + ln) if i == 0 else ("        " + " " + ln)
                print(out); f.write(out + "\n")
            f.write("\n"); f.flush()
            n += 1
            # The firmware latches the snapshot once a second.
            nxt = t0 + n
            slp = nxt - time.time()
            if slp > 0: time.sleep(slp)
    print(f"captured {n} snapshots over {time.time()-t0:.1f}s -> {LOGPATH}")


if __name__ == "__main__":
    main()
