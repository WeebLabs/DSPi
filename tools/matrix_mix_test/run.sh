#!/bin/sh
# Builds the matrix kernel harness against the live matrix_mix.h and runs it.
# -ffp-contract=off matches the firmware build of matrix_mix.c; Apple clang fuses FMAs without it.
set -e
here=$(cd "$(dirname "$0")" && pwd)
fw="$here/../../firmware/DSPi"
out="${TMPDIR:-/tmp}/matrix_mix_test.$$"
mkdir -p "$out"
CC=${CC:-cc}
rc=0
for opt in -O0 -O2 -O3; do
    $CC -std=gnu11 $opt -ffp-contract=off -I"$fw" -Wall "$here/harness.c" -o "$out/harness"
    printf '%s ' "$opt"; "$out/harness" || rc=1
done
rm -rf "$out"
exit $rc
