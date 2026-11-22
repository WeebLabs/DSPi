#!/bin/sh
# Builds the live RP2350 loudness and psybass kernels for the host and checks them (see harness.c).
# -ffp-contract=off matches the firmware build of loudness.c and psybass.c.
set -e
here=$(cd "$(dirname "$0")" && pwd)
fw="$here/../../firmware/DSPi"
out="${TMPDIR:-/tmp}/fx_kernel_test.$$"
mkdir -p "$out"
CC=${CC:-cc}
f="-std=gnu11 -O2 -ffp-contract=off -DPICO_RP2350=1 -DDSP_TIME_CRITICAL= -I$here/../loudness_test/stub -I$fw"
$CC $f -w -c "$fw/loudness.c" -o "$out/loudness.o"
$CC $f -w -c "$fw/psybass.c" -o "$out/psybass.o"
$CC $f -Wall -c "$here/harness.c" -o "$out/harness.o"
$CC "$out/harness.o" "$out/loudness.o" "$out/psybass.o" -lm -o "$out/harness"
rc=0
"$out/harness" || rc=$?
rm -rf "$out"
exit $rc
