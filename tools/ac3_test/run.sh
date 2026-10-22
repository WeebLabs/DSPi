#!/bin/bash
# Host verification of the ported AC-3 decoder (firmware/DSPi/ac3).
#
# Builds ac3_test against the firmware sources, checks the generated IMDCT
# tables, generates AC-3 vectors with ffmpeg's encoder, decodes them with
# both decoders and compares (compare.py).  Needs ffmpeg and a C compiler.
# -ffp-contract=off matches the device build so the checksums agree.
#
#   tools/ac3_test/run.sh [workdir]

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
AC3="$HERE/../../firmware/DSPi/ac3"
WORK="${1:-$HERE/build}"
mkdir -p "$WORK"

cc -O2 -ffp-contract=off -Wall -Wno-unused-function -I"$AC3" \
   -o "$WORK/ac3_test" "$HERE/ac3_test.c" \
   "$AC3"/parse.c "$AC3"/bitstream.c "$AC3"/bit_allocate.c "$AC3"/downmix.c \
   "$AC3"/imdct.c "$AC3"/crc.c "$AC3"/ac3_decoder.c -lm

"$WORK/ac3_test" tables

# Dither: both decoders replace zero-bit mantissas with noise from their own
# generators, so a noisy source only agrees with ffmpeg to 25-45 dB.  Tones
# keep nearly every coefficient coded and agree to 75+ dB; those vectors
# carry the strict check, the noisy ones exercise coupling and bit-allocation
# changes (loose check here, bit-exact against upstream in run_stock.sh).
TONES51='aevalsrc=0.5*sin(2*PI*440*t)|0.5*sin(2*PI*554*t)|0.45*sin(2*PI*659*t)|0.6*sin(2*PI*55*t)|0.3*sin(2*PI*880*t)|0.3*sin(2*PI*1109*t):c=5.1:s=48000'
TONES20='aevalsrc=0.5*sin(2*PI*440*t)|0.4*sin(2*PI*660*t):c=stereo:s=44100'
NOISY51='aevalsrc=0.5*sin(2*PI*440*t)*(0.6+0.4*sin(2*PI*0.7*t))+0.15*(random(0)-0.5)*(0.5+0.5*sin(2*PI*1.3*t))|0.5*sin(2*PI*554*t)*(0.6+0.4*sin(2*PI*0.9*t))+0.15*(random(1)-0.5)|0.45*sin(2*PI*659*t)+0.1*sin(2*PI*3300*t)*sin(2*PI*2*t)|0.6*sin(2*PI*55*t)*(0.5+0.5*sin(2*PI*0.3*t))|0.3*sin(2*PI*880*t)+0.2*(random(2)-0.5)*sin(2*PI*5*t)|0.3*sin(2*PI*1109*t)+0.2*(random(3)-0.5)*sin(2*PI*4*t):c=5.1:s=48000'
NOISY20='aevalsrc=0.5*sin(2*PI*440*t)+0.2*(random(0)-0.5)|0.4*sin(2*PI*660*t)+0.2*(random(1)-0.5):c=stereo:s=44100'

gen() {  # name src rate duration
    ffmpeg -v error -y -f lavfi -i "$2" -t "$4" -c:a ac3 -b:a "$3" "$WORK/$1.ac3"
    ffmpeg -v error -y -i "$WORK/$1.ac3" -f f32le "$WORK/$1_ref.f32"
}
gen t51_640k "$TONES51" 640k 1.0
gen t20_192k_44k1 "$TONES20" 192k 1.0
gen v51_640k "$NOISY51" 640k 1.0
gen v51_448k "$NOISY51" 448k 1.0
gen v20_192k_44k1 "$NOISY20" 192k 1.0

status=0
check() {  # name channels min_snr
    echo "== $1 (min SNR $3 dB)"
    "$WORK/ac3_test" decode "$WORK/$1.ac3" "$WORK/$1_ours.f32"
    ours="$WORK/$1_ours.f32"
    if [ "$2" = 2 ]; then
        python3 "$HERE/strip_channels.py" "$ours" "$WORK/$1_ours2.f32"; ours="$WORK/$1_ours2.f32"
    fi
    python3 "$HERE/compare.py" "$ours" "$WORK/$1_ref.f32" --channels "$2" --min-snr "$3" || status=1
}
check t51_640k 6 70
check t20_192k_44k1 2 70
check v51_640k 6 25
check v51_448k 6 20
check v20_192k_44k1 2 30

# The committed bench vector must still decode to the checksum the device
# bench compares against (tools/ac3_test/vectors/*.sum).
echo "== bench vector checksum"
BV="$HERE/vectors/bench_51_640k.ac3"
got=$("$WORK/ac3_test" decode "$BV" | sed -E 's/.*checksum=(0x[0-9a-f]+).*/\1/')
want=$(cut -d' ' -f1 "$BV.sum")
if [ "$got" = "$want" ]; then echo "bench vector: $got matches sidecar"; else echo "bench vector: $got != sidecar $want"; status=1; fi

# Dialnorm: a stream coded at dialnorm -27 decoded with dialnorm applied must
# equal the plain decode scaled by -4 dB (the AC-3 reference is -31).  DRC
# modes cannot be checked this way: ffmpeg's encoder writes no dynrng words,
# so off, full and half all decode identically here.
ffmpeg -v error -y -f lavfi -i "$TONES51" -t 0.5 -c:a ac3 -b:a 448k -dialnorm -27 "$WORK/dn27.ac3"
"$WORK/ac3_test" decode "$WORK/dn27.ac3" "$WORK/dn27_plain.f32" | tail -1
"$WORK/ac3_test" decode "$WORK/dn27.ac3" "$WORK/dn27_applied.f32" --dialnorm | tail -1
echo "== dialnorm -27 applied = plain -4 dB"
python3 "$HERE/scale_check.py" "$WORK/dn27_plain.f32" "$WORK/dn27_applied.f32" -4 || status=1
echo "== DRC modes identical on a stream without dynrng words"
"$WORK/ac3_test" decode "$WORK/dn27.ac3" "$WORK/dn27_drc0.f32" --drc 0 >/dev/null
"$WORK/ac3_test" decode "$WORK/dn27.ac3" "$WORK/dn27_drc2.f32" --drc 2 >/dev/null
cmp -s "$WORK/dn27_plain.f32" "$WORK/dn27_drc0.f32" && cmp -s "$WORK/dn27_plain.f32" "$WORK/dn27_drc2.f32" && echo PASS || { echo FAIL; status=1; }

# Stream layer (ac3_stream.c): deframer, ring and just-in-time scheduler
# against the direct decode delayed by one burst period.
cc -O2 -ffp-contract=off -Wall -Wno-unused-function -I"$AC3/.." -I"$AC3" \
   -o "$WORK/stream_test" "$HERE/stream_test.c" "$AC3"/../ac3_stream.c \
   "$AC3"/parse.c "$AC3"/bitstream.c "$AC3"/bit_allocate.c "$AC3"/downmix.c \
   "$AC3"/imdct.c "$AC3"/crc.c "$AC3"/ac3_decoder.c -lm
for v in v51_640k v51_448k v20_192k_44k1; do
    echo "== stream $v"; "$WORK/stream_test" "$WORK/$v.ac3" | tail -1 || status=1
done
echo "== stream swapped payload"; "$WORK/stream_test" "$WORK/v51_640k.ac3" --swap | tail -1 || status=1
echo "== stream 1-frame and 192-frame polls"
"$WORK/stream_test" "$WORK/v51_448k.ac3" --chunk 1 | tail -1 || status=1
"$WORK/stream_test" "$WORK/v51_640k.ac3" --chunk 192 | tail -1 || status=1
echo "== stream 40-poll stall"; "$WORK/stream_test" "$WORK/v51_640k.ac3" --stall 40 | tail -1 || status=1
echo "== stream corrupted frame and Pc error flag (rejected, silent)"
"$WORK/stream_test" "$WORK/v51_448k.ac3" --corrupt 5 | tail -1 || status=1
"$WORK/stream_test" "$WORK/v51_448k.ac3" --pcerr 5 | tail -1 || status=1

echo "== host timing (for curiosity only; the device bench is authoritative)"
"$WORK/ac3_test" bench "$WORK/v51_640k.ac3" 20
exit $status
