#!/bin/sh
# Builds the live loudness.c and the pre-change table version (d35742e), both variants, and runs the checks.
# Set REF_REV to compare against another revision of the table code.
set -e
here=$(cd "$(dirname "$0")" && pwd)
fw="$here/../../firmware/DSPi"
out="${TMPDIR:-/tmp}/loudness_test.$$"
mkdir -p "$out"
git -C "$here" show "${REF_REV:-d35742e}:firmware/DSPi/loudness.c" > "$out/loudness_ref.c"
CC=${CC:-cc}
rc=0
for plat in 1 0; do
    f="-std=gnu11 -O2 -ffp-contract=off -DPICO_RP2350=$plat -DDSP_TIME_CRITICAL= -I$here/stub -I$fw"
    $CC $f -w -DREF_SRC="\"$out/loudness_ref.c\"" -c "$here/ref_tu.c" -o "$out/ref.o"
    $CC $f -w -c "$fw/loudness.c" -o "$out/new.o"
    $CC $f -Wall -c "$here/harness.c" -o "$out/harness.o"
    $CC "$out/harness.o" "$out/new.o" "$out/ref.o" -lm -o "$out/harness"
    echo "== PICO_RP2350=$plat"
    "$out/harness" || rc=1
done
rm -rf "$out"
exit $rc
