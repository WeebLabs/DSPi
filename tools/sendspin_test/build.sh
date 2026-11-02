#!/bin/sh
# Build and run the Sendspin host test (gcc / MinGW). Usage: sh build.sh
set -e
# optional: SANITIZE="-fsanitize=address,undefined" (not available on MinGW)
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$HERE/../../firmware/DSPi"
MBED="${MBEDTLS_DIR:-$HOME/pico-sdk/lib/mbedtls}"
B="$HERE/build"
rm -rf "$B"; mkdir -p "$B"
# copy the units under test next to the stubs so that "net_input.h" resolves to
# the stub even if the real firmware header exists
cp "$FW/sendspin_client.c" "$FW/sendspin_client.h" "$FW/sendspin_noise.c" "$FW/sendspin_noise.h" "$B/"
cp "$HERE/test_sendspin.c" "$HERE/mbedtls_config_test.h" "$HERE/stubs/net_input.h" "$B/"

LIBS=""
for f in platform_util constant_time bignum bignum_core bignum_mod bignum_mod_raw \
         ecp ecp_curves ecp_curves_new chacha20 chachapoly poly1305 sha256 sha1 \
         md ecdh base64 platform md5 sha512 sha3 ripemd160; do
  LIBS="$LIBS $MBED/library/$f.c"
done
gcc -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-parameter $SANITIZE \
    -DMBEDTLS_CONFIG_FILE='"mbedtls_config_test.h"' \
    -I"$B" -I"$HERE/stubs" -I"$MBED/include" -I"$MBED/library" \
    "$B/test_sendspin.c" "$B/sendspin_noise.c" $LIBS -o "$B/test_sendspin.exe"
"$B/test_sendspin.exe"
