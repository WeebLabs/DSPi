#!/bin/sh
# Builds the S/PDIF ring stamping test with the host compiler and runs it.
set -e
here="$(cd "$(dirname "$0")" && pwd)"
inc="$here/../../firmware/pico-extras/src/rp2_common/pico_audio_spdif_multi/include"
out="${TMPDIR:-/tmp}/spdif_ring_stamp_test"
cc -std=c11 -O2 -Wall -Wextra -Werror -I"$inc" "$here/spdif_ring_stamp_test.c" -o "$out"
"$out"
