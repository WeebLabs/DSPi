#!/bin/bash
# Chain-pacer landing phase across peripheral re-inits and boots, under load 109.
# Usage: run_cboot.sh FIRST COUNT (odd: soft reboot, even: picotool reload; 0: no reboot)
set -u
cd /Users/weeblabs/DSPi/bench_results/vcxo_exact
RB=/Users/weeblabs/DSPi/tools/vcxo_exact/run_bench.py
UF2=/Users/weeblabs/DSPi/tools/vcxo_exact/build-rp2350/vcxo_exact.uf2
for k in $(seq "$1" $(( $1 + $2 - 1 ))); do
    if (( k == 0 )); then how=none
    elif (( k % 2 )); then
        how=soft
        python3 - <<'PY'
import glob, os
fd = os.open(sorted(glob.glob("/dev/cu.usbmodem*"))[0], os.O_RDWR | os.O_NOCTTY)
os.write(fd, b"b\n")
PY
        sleep 7
    else
        how=picotool
        picotool reboot -f -u >/dev/null 2>&1; sleep 3
        picotool load -x "$UF2" >/dev/null 2>&1; sleep 6
    fi
    args=(I "K 1" "W 50" "a 1" "L 109")
    for kind in 1 2; do
        for r in 1 2 3; do
            args+=("c $kind 0 2")
            for d in $(seq 0 25); do args+=("c $kind $d 0" "g 300 100 1"); done
        done
    done
    args+=("L 0" "c 0 -1 0" "a 0")
    echo "# boot $k via $how" >> "65_cboot$k.log"
    perl -e 'alarm 1500; exec @ARGV' python3 "$RB" "65_cboot$k.log" "${args[@]}" > /dev/null
    echo "boot $k ($how) done"
done
