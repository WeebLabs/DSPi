#!/bin/sh
# Host test for firmware/DSPi/net_input.c (PCM ring, S16/S24 conversion, format
# gating, volume curve, config, WiFi state machine).  No external deps.
# Usage: sh build.sh     (optional: SANITIZE="-fsanitize=address,undefined")
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$HERE/../../firmware/DSPi"
B="$HERE/build"
rm -rf "$B"; mkdir -p "$B"
# real net_input.h/.c copied beside each other; everything else comes from stubs/
cp "$FW/net_input.c" "$FW/net_input.h" "$B/"
cp "$HERE/test_net_input.c" "$B/"
gcc -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-parameter $SANITIZE -DDSPI_ENABLE_WIFI \
    -I"$HERE/stubs" "$B/test_net_input.c" -lm -o "$B/test_net_input.exe"
"$B/test_net_input.exe"
