#!/bin/sh
# Build and run the Slimproto host test (gcc / MinGW). Usage: sh build.sh
# slimproto_client.c, sendspin_client.c (+noise) and net_flac.c are compiled as
# separate units against fake lwIP / net_input / ADAT-loan stubs.  FLAC tests
# need ffmpeg + ffprobe + python (fixtures come from ../sendspin_test).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$HERE/../../firmware/DSPi"
MBED="${MBEDTLS_DIR:-$HOME/pico-sdk/lib/mbedtls}"
B="$HERE/build"
rm -rf "$B"; mkdir -p "$B"
# copy the units under test next to the stubs so that "net_input.h" resolves to the stub
cp "$FW/slimproto_client.c" "$FW/slimproto_client.h" "$FW/sendspin_client.c" "$FW/sendspin_client.h" \
   "$FW/sendspin_noise.c" "$FW/sendspin_noise.h" "$FW/net_flac.c" "$FW/net_flac.h" "$B/"
cp "$HERE/test_slimproto.c" "$HERE/mbedtls_config_test.h" "$HERE/stubs/net_input.h" "$B/"

FIX=""
if command -v ffmpeg >/dev/null 2>&1 && command -v ffprobe >/dev/null 2>&1; then
  PY=python; command -v python >/dev/null 2>&1 || PY=python3
  if "$PY" "$HERE/../sendspin_test/gen_flac_fixtures.py" "$B/fixtures" >/dev/null; then FIX="$B/fixtures"; fi
fi
[ -n "$FIX" ] || echo "NOTE: ffmpeg/ffprobe/python missing, FLAC tests skipped"

LIBS=""
for f in platform_util constant_time bignum bignum_core bignum_mod bignum_mod_raw \
         ecp ecp_curves ecp_curves_new chacha20 chachapoly poly1305 sha256 sha1 \
         md ecdh base64 platform md5 sha512 sha3 ripemd160; do
  LIBS="$LIBS $MBED/library/$f.c"
done
gcc -std=gnu11 -O2 -w -DNDEBUG -I"$FW/third_party/foxen-flac" -c \
    "$FW/third_party/foxen-flac/foxen-flac.c" -o "$B/foxen-flac.o"
# -I order: our stubs (opaque pcb, net_input) first, then the sendspin stubs (pbuf, dns, time, rand, ...)
gcc -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-parameter $SANITIZE \
    -DSLIMPROTO_TEST_HOOKS -DMBEDTLS_CONFIG_FILE='"mbedtls_config_test.h"' \
    -I"$B" -I"$HERE/stubs" -I"$HERE/../sendspin_test/stubs" -I"$FW/third_party/foxen-flac" \
    -I"$MBED/include" -I"$MBED/library" \
    "$B/test_slimproto.c" "$B/slimproto_client.c" "$B/sendspin_client.c" "$B/sendspin_noise.c" \
    "$B/net_flac.c" "$B/foxen-flac.o" $LIBS -lm -o "$B/test_slimproto.exe"
"$B/test_slimproto.exe" ${FIX:+"$FIX"}
