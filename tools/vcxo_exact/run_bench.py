#!/usr/bin/env python3
"""Send bench commands to vcxo_exact over USB CDC and log the replies.

Usage: python3 run_bench.py LOGFILE CMD [CMD ...]
Each CMD is one line (e.g. "S 20"); the device ends every reply with DONE.
Uses termios only (no pyserial). The port is the first /dev/cu.usbmodem*.
"""
import glob, os, select, sys, termios, time

log = open(sys.argv[1], "a")
port = sorted(glob.glob("/dev/cu.usbmodem*"))[0]
fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
attrs = termios.tcgetattr(fd)
attrs[3] &= ~(termios.ICANON | termios.ECHO)
termios.tcsetattr(fd, termios.TCSANOW, attrs)

buf = b""
for cmd in sys.argv[2:]:
    os.write(fd, cmd.encode() + b"\n")
    log.write(f"> {cmd}\n")
    done, deadline = False, time.time() + 7200
    while not done and time.time() < deadline:
        r, _, _ = select.select([fd], [], [], 1.0)
        if not r:
            continue
        buf += os.read(fd, 65536)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").strip()
            if not text or text == "READY" or text.startswith("#"):
                continue
            print(text, flush=True)
            log.write(text + "\n")
            log.flush()
            if text == "DONE":
                done = True
                break
    if not done:
        sys.exit(f"timed out on {cmd}")
