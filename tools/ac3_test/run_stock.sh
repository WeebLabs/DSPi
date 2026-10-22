#!/bin/bash
# Bit-exact check of the port against an unmodified liba52 checkout
# (https://code.videolan.org/videolan/liba52).  Both decoders run the same
# vectors; any differing float bit pattern fails.
#
#   tools/ac3_test/run_stock.sh <liba52 checkout> [workdir]
#
# run.sh must have produced the vectors in the workdir first.

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
STOCK="$1"
WORK="${2:-$HERE/build}"
AC3="$HERE/../../firmware/DSPi/ac3"

# Minimal config.h the autotools build would have generated.
mkdir -p "$WORK/stock"
cat > "$WORK/stock/config.h" <<'EOF'
#define HAVE_BUILTIN_EXPECT 1
EOF
cc -O2 -ffp-contract=off -w -I"$WORK/stock" -I"$STOCK/include" -I"$STOCK/liba52" \
   -o "$WORK/stock_test" "$HERE/stock_test.c" \
   "$STOCK"/liba52/parse.c "$STOCK"/liba52/bitstream.c "$STOCK"/liba52/bit_allocate.c \
   "$STOCK"/liba52/downmix.c "$STOCK"/liba52/imdct.c "$STOCK"/liba52/crc.c \
   "$STOCK"/liba52/cpu_accel.c "$STOCK"/liba52/cpu_state.c -lm

status=0
for v in v51_640k v51_448k v20_192k_44k1; do
    "$WORK/stock_test" "$WORK/$v.ac3" "$WORK/${v}_stock.f32"
    if cmp -s "$WORK/${v}_ours.f32" "$WORK/${v}_stock.f32"; then
        echo "$v: bit-exact with stock liba52"
    else
        echo "$v: DIFFERS from stock liba52"
        python3 "$HERE/compare.py" "$WORK/${v}_ours.f32" "$WORK/${v}_stock.f32" --channels 6 || true
        status=1
    fi
done
exit $status
