#!/bin/sh
# Builds the harness against the live firmware sources and runs it.
# -ffp-contract=off matches the firmware build of dsp_pipeline.c; without it the host fuses FMAs.
set -e
here=$(cd "$(dirname "$0")" && pwd)
fw="$here/../../firmware/DSPi"
out="${TMPDIR:-/tmp}/eq_kernel_test.$$"
mkdir -p "$out"
CC=${CC:-cc}
$CC -std=gnu11 -O2 -ffp-contract=off -DPICO_RP2350=1 -I"$fw" -w -DKPFX=kfin -c "$here/kernel_tu.c" -o "$out/kfin.o"
$CC -std=gnu11 -O2 -ffp-contract=off -DPICO_RP2350=1 -I"$fw" -Wall "$here/harness.c" "$out/kfin.o" -lm -o "$out/harness"
rc=0
"$out/harness" || rc=$?
rm -rf "$out"
exit $rc
