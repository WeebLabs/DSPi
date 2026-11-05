#!/usr/bin/env python3
"""Trigger the vcxo_profile suite over USB CDC and print/log its output.

Usage: python3 run_profile.py /dev/cu.usbmodemXXXX [logfile]
Uses termios only (no pyserial). The suite takes about two minutes.
"""
import os, sys, termios, time, select

port = sys.argv[1]
log = open(sys.argv[2] if len(sys.argv) > 2 else "vcxo_profile.log", "w")
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
attrs = termios.tcgetattr(fd)
attrs[3] &= ~(termios.ICANON | termios.ECHO)
termios.tcsetattr(fd, termios.TCSANOW, attrs)

os.write(fd, b"g")
buf, deadline = b"", time.time() + 400
while time.time() < deadline:
    r, _, _ = select.select([fd], [], [], 1.0)
    if not r:
        continue
    buf += os.read(fd, 4096)
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        text = line.decode(errors="replace").strip()
        if not text or text == "READY":
            continue
        print(text, flush=True)
        log.write(text + "\n")
        if text == "DONE":
            sys.exit(0)
sys.exit("timed out")
