#!/usr/bin/env python3
"""Test F driver: under one load setting, sweep the parked write phase of
alternating +/- bursts, then run long at the cleanest and the worst park.

Usage: python3 run_f.py LOGFILE LOAD_MASK [CLEAN_PULSES] [BAD_PULSES]
Park labels are only consistent within one session at one load, hence one script.
"""
import glob, os, select, sys, termios, time

log = open(sys.argv[1], "a")
load = int(sys.argv[2])
clean_pulses = int(sys.argv[3]) if len(sys.argv) > 3 else 2000000
bad_pulses = int(sys.argv[4]) if len(sys.argv) > 4 else 200000
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
            if not t or t == "READY":
                continue
            log.write(t + "\n")
            log.flush()
            if t == "DONE":
                return out
            out.append(t)


def ds(lines):
    for t in lines:
        if t.startswith("DS,"):
            return dict(kv.split("=") for kv in t.split(",")[1:])


for c in ("K 1", "W 50", f"L {load}"):
    cmd(c)
sweep = {}
for p in range(0, 128, 2):
    d = ds(cmd(f"D 60 100 12 5 {p}"))
    sweep[p] = int(d["sumabs"])
    print(f"sweep park={p} sumabs={sweep[p]} nz={d['nz']}", flush=True)
# Clean park: centre of the longest circular run of zero-miscount parks.
keys = sorted(sweep)
best, run, start = (0, 0), 0, 0
for i in range(2 * len(keys)):
    k = keys[i % len(keys)]
    if sweep[k] == 0:
        if run == 0:
            start = i
        run += 1
        if run > best[0] and run <= len(keys):
            best = (run, start)
    else:
        run = 0
clean = keys[(best[1] + best[0] // 2) % len(keys)]
bad = max(sweep, key=sweep.get)
print(f"clean_park={clean} (zero run {best[0]} x 2) bad_park={bad}", flush=True)
for park, n in ((clean, clean_pulses), (bad, bad_pulses)):
    d = ds(cmd(f"D {n // 100} 100 12 5 {park}"))
    print(f"F park={park} load={load} pulses={d['pulses']} sumdev={d['sumdev']} sumabs={d['sumabs']} "
          f"nz={d['nz']} fails={d['fails']} unlock={d['unlock']}", flush=True)
cmd("L 0")
