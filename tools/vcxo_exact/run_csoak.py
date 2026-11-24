#!/usr/bin/env python3
"""Chain-pacer soak: re-init the peripheral, map + miscounts per delay under load 109,
put the delay mid-way through the clean run, then soak (G3 stream set) at that delay.

Usage: python3 run_csoak.py LOGFILE KIND [LOAD] [all|mixed]
Landing phase is re-rolled by every peripheral init, so map and soak share one init.
"""
import glob, os, select, sys, termios

log = open(sys.argv[1], "a")
kind = int(sys.argv[2])
load = int(sys.argv[3]) if len(sys.argv) > 3 else 109
which = sys.argv[4] if len(sys.argv) > 4 else "all"
fd = os.open(sorted(glob.glob("/dev/cu.usbmodem*"))[0], os.O_RDWR | os.O_NOCTTY)
attrs = termios.tcgetattr(fd)
attrs[3] &= ~(termios.ICANON | termios.ECHO)
termios.tcsetattr(fd, termios.TCSANOW, attrs)
buf = b""


def cmd(c):
    global buf
    os.write(fd, c.encode() + b"\n")
    log.write(f"> {c}\n")
    out = []
    while True:
        r, _, _ = select.select([fd], [], [], 1.0)
        if not r:
            continue
        buf += os.read(fd, 65536)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode(errors="replace").strip()
            if not t or t == "READY" or t.startswith("#"):
                continue
            log.write(t + "\n")
            log.flush()
            if t == "DONE":
                return out
            out.append(t)


def xs(lines):
    for t in lines:
        if t.startswith("XS,"):
            return dict(kv.split("=") for kv in t.split(",")[1:])


for c in ("K 1", "W 50", "a 1", f"L {load}", f"c {kind} 0 2"):
    cmd(c)
bad = []
for d in range(26):
    cmd(f"c {kind} {d} 0")
    bad.append(int(xs(cmd("g 300 100 1"))["sumabs"]))
# Longest circular run of clean delays; soak at its centre.
best, run, start = (0, 0), 0, 0
for i in range(52):
    if bad[i % 26] == 0:
        if run == 0:
            start = i
        run += 1
        if best[0] < run <= 26:
            best = (run, start)
    else:
        run = 0
centre = (best[1] + best[0] // 2) % 26
print(f"kind={kind} map={bad} clean_run={best[0]} centre={centre}", flush=True)
cmd(f"c {kind} {centre} 0")
streams = ["g 40000 500 1", "g 40000 500 2"] if which == "all" else []
streams += ["g 40000 500 0", "i", "g 14000 1000 6 1000", "i", "g 14000 1000 6 -1000", "i", "g 14000 1000 6 30"]
for c in streams:
    d = xs(cmd(c))
    if d:
        print(f"SOAK kind={kind} cdelay={centre} mode={d['mode']} arg={d['arg']} pulses={d['pulses']} "
              f"sumdev={d['sumdev']} sumabs={d['sumabs']} nz={d['nz']} unlock={d['unlock']} "
              f"lockn={d['lockn']} fails={d['fails']} other={d['other']} timeout={d['timeout']}", flush=True)
for c in ("I", "L 0", "c 0 -1 0", "a 0"):
    cmd(c)
