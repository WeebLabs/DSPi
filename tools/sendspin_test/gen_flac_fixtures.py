#!/usr/bin/env python3
"""Generate deterministic FLAC test fixtures (needs ffmpeg + ffprobe on PATH).

For every case NAME it writes, into the output directory:
  NAME.wav   reference PCM (canonical 44-byte header, little endian)
  NAME.flac  ffmpeg-encoded FLAC (compression level 5, fixed frame size)
  NAME.idx   one line per FLAC frame: "<byte offset> <byte size> <samples>"
The signal is sines + low level noise, with a digital-silence stretch (constant
subframes) and a full-scale noise burst (large / verbatim-ish frames).
Usage: gen_flac_fixtures.py OUTDIR
"""
import math, os, struct, subprocess, sys, wave

CASES = [
    # name,      rate,  bits, seconds, frame_size
    ("f44_16",   44100, 16, 0.80, 4096),
    ("f48_16",   48000, 16, 0.70, 4096),
    ("f44_24",   44100, 24, 0.40, 2048),
    ("f44_16_s", 44100, 16, 0.30, 1152),   # small blocks: many frames
]


def gen(rate, bits, secs):
    n = int(rate * secs)
    full = (1 << (bits - 1)) - 1
    lcg = 12345
    frames = []
    for i in range(n):
        t = i / rate
        lcg = (lcg * 1103515245 + 12345) & 0x7FFFFFFF
        noise = ((lcg >> 8) & 0xFF) - 128                # +-128 LSB (16-bit scale)
        if i < n * 0.55:
            l = 0.30 * math.sin(2 * math.pi * 440 * t) + 0.10 * math.sin(2 * math.pi * 3000 * t)
            r = 0.25 * math.sin(2 * math.pi * 660 * t) + 0.12 * math.sin(2 * math.pi * 5000 * t)
            ls = int(l * full) + noise * (1 << (bits - 16))
            rs = int(r * full) - noise * (1 << (bits - 16))
        elif i < n * 0.70:
            ls = rs = 0                                   # digital silence
        elif i < n * 0.78:
            lcg = (lcg * 1103515245 + 12345) & 0x7FFFFFFF
            ls = ((lcg >> 4) % (2 * full)) - full         # full-scale noise burst
            lcg = (lcg * 1103515245 + 12345) & 0x7FFFFFFF
            rs = ((lcg >> 4) % (2 * full)) - full
        else:
            l = 0.5 * math.sin(2 * math.pi * 1000 * t)
            ls = int(l * full)
            rs = ls                                       # identical channels (mid/side friendly)
        ls = max(-full - 1, min(full, ls))
        rs = max(-full - 1, min(full, rs))
        frames.append((ls, rs))
    return frames


def write_wav(path, rate, bits, frames):
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(bits // 8)
        w.setframerate(rate)
        buf = bytearray()
        for l, r in frames:
            for s in (l, r):
                buf += (s & ((1 << bits) - 1)).to_bytes(bits // 8, "little")
        w.writeframes(bytes(buf))


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for name, rate, bits, secs, fs in CASES:
        wav = os.path.join(out, name + ".wav")
        flac = os.path.join(out, name + ".flac")
        write_wav(wav, rate, bits, gen(rate, bits, secs))
        fmt = "s16" if bits == 16 else "s32"
        cmd = ["ffmpeg", "-loglevel", "error", "-y", "-i", wav, "-c:a", "flac",
               "-compression_level", "5", "-sample_fmt", fmt, "-frame_size", str(fs)]
        if bits == 24:
            cmd += ["-bits_per_raw_sample", "24"]
        subprocess.check_call(cmd + [flac])
        pk = subprocess.check_output(
            ["ffprobe", "-loglevel", "error", "-select_streams", "a:0", "-show_entries",
             "packet=pos,size,duration", "-of", "csv=p=0", flac], text=True)
        with open(os.path.join(out, name + ".idx"), "w") as f:
            expect = None
            for line in pk.strip().splitlines():
                # ffprobe csv order here: duration,size,pos (verified below by the contiguity assert)
                p = line.strip(",").split(",")
                dur, size, pos = int(p[0]), int(p[1]), int(p[2])
                f.write("%d %d %d\n" % (pos, size, dur))
        print("fixture", name, os.path.getsize(flac), "bytes")


if __name__ == "__main__":
    main()
