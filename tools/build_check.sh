#!/bin/sh
# build_check.sh - build every firmware target in its own build dir; fail on error.
#
#   pico      RP2040, plain Pico            (build-fw-check-pico)
#   pico_w    RP2040, Pico W, no WiFi       (build-fw-check-pico_w)
#   pico2_w   RP2350, Pico 2 W + WiFi       (build-fw-check-pico2_w)
#   pico2     RP2350, plain Pico 2          (build-fw-check-pico2)
# plus one configure-time guard: pico_w + DSPI_ENABLE_WIFI must FAIL with the RAM
# explanation (network audio is RP2350-only).
#
# Needs the toolchain on PATH (PlatformIO: .pio-core/packages/{toolchain-gccarmnoneeabi,
# tool-cmake,tool-ninja}) and network access once for the picotool download.
# Usage: tools/build_check.sh [target...]   (default: all)
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
PKG="$ROOT/.pio-core/packages"
for d in toolchain-gccarmnoneeabi tool-cmake tool-ninja; do
    [ -d "$PKG/$d/bin" ] && PATH="$PKG/$d/bin:$PATH"
    [ -d "$PKG/$d" ] && [ ! -d "$PKG/$d/bin" ] && PATH="$PKG/$d:$PATH"
done
export PATH
for t in cmake ninja arm-none-eabi-gcc; do
    command -v "$t" >/dev/null || { echo "build_check: $t not found on PATH" >&2; exit 2; }
done

COMMON="-G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_SDK_PATH=$ROOT/firmware/pico-sdk -DPICO_EXTRAS_PATH=$ROOT/firmware/pico-extras"
fail=0

configure() { # dir args...
    d=$1; shift
    # shellcheck disable=SC2086
    cmake $COMMON -B "$ROOT/$d" "$@" "$ROOT/firmware" >"$ROOT/$d.configure.log" 2>&1
}

build() { # name dir args...
    name=$1; d=$2; shift 2
    echo "== $name"
    if configure "$d" "$@" && ninja -C "$ROOT/$d" >"$ROOT/$d.build.log" 2>&1; then
        arm-none-eabi-size "$ROOT/$d/DSPi/DSPi.elf" | tail -1
        echo "   OK"
    else
        echo "   FAILED (see $d.configure.log / $d.build.log)"; fail=1
    fi
}

WANT="${*:-all}"
sel() { [ "$WANT" = all ] && return 0; for a in $WANT; do [ "$a" = "$1" ] && return 0; done; return 1; }

sel pico    && build pico    build-fw-check-pico    -DPICO_PLATFORM=rp2040 -DPICO_BOARD=pico
sel pico_w  && build pico_w  build-fw-check-pico_w  -DPICO_PLATFORM=rp2040 -DPICO_BOARD=pico_w
sel pico2   && build pico2   build-fw-check-pico2   -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2
sel pico2_w && build pico2_w build-fw-check-pico2_w -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2_w -DDSPI_ENABLE_WIFI=ON

if sel pico_w; then
    echo "== pico_w + WIFI must be refused (RAM)"
    d=build-fw-check-pico_w-wifi-refused
    if configure "$d" -DPICO_PLATFORM=rp2040 -DPICO_BOARD=pico_w -DDSPI_ENABLE_WIFI=ON; then
        echo "   FAILED: configure succeeded; the RP2040 WiFi gate is gone"; fail=1
    elif grep -q "does not fit in RAM" "$ROOT/$d.configure.log"; then
        echo "   OK (refused with the RAM explanation)"
    else
        echo "   FAILED: refused, but without the expected message ($d.configure.log)"; fail=1
    fi
fi

[ $fail -eq 0 ] && echo "build_check: all OK" || echo "build_check: FAILURES"
exit $fail
