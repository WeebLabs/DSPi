#!/usr/bin/env python3
"""Keep only L and R of a six-channel float32 decode, for comparing a 2/0
stream against ffmpeg's stereo output.  Usage: strip_channels.py in out"""

import array
import sys

src = array.array("f")
with open(sys.argv[1], "rb") as f:
    src.frombytes(f.read())
dst = array.array("f")
for i in range(0, len(src), 6):
    dst.append(src[i])
    dst.append(src[i + 1])
with open(sys.argv[2], "wb") as f:
    f.write(dst.tobytes())
