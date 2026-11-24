#!/bin/bash
# G4: quiet bad-delay map across boots. Usage: run_g4.sh FIRST_INDEX COUNT
# Odd boots: soft reboot (watchdog). Even boots: picotool reflash.
set -u
cd /Users/weeblabs/DSPi/bench_results/vcxo_exact
RB=/Users/weeblabs/DSPi/tools/vcxo_exact/run_bench.py
UF2=/Users/weeblabs/DSPi/tools/vcxo_exact/build-rp2350/vcxo_exact.uf2
for k in $(seq "$1" $(( $1 + $2 - 1 ))); do
    if (( k % 2 )); then
        how=soft
        python3 - <<'EOF'
import glob, os
fd = os.open(sorted(glob.glob("/dev/cu.usbmodem*"))[0], os.O_RDWR | os.O_NOCTTY)
os.write(fd, b"b\n")
EOF
        sleep 6
    else
        how=picotool
        picotool reboot -f -u >/dev/null 2>&1; sleep 3
        picotool load -x "$UF2" >/dev/null 2>&1; sleep 5
    fi
    args=(I "K 1" "W 50" "M")
    for d in 7 8 9 10 11 12 13; do args+=("x $d 30 1" "g 1000 100 2" "g 1000 100 1"); done
    for d in 17 18; do args+=("x $d 30 0" "g 1000 100 2" "g 1000 100 1"); done
    echo "# boot $k via $how" >> "47_G4_boot$k.log"
    python3 "$RB" "47_G4_boot$k.log" "${args[@]}" > /dev/null
    echo "boot $k ($how) done"
done
