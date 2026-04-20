cmake_minimum_required(VERSION 3.12)

# Default SDK path to submodule location if not set via environment
if (NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
endif()

# Always use local pico-extras (contains modified libraries)
set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-extras CACHE PATH "" FORCE)

# Override the "pico" board's PICO_FLASH_SPI_CLKDIV=2 default.  DSPi overclocks
# RP2040 to 307.2 MHz, making div=2 yield 153.6 MHz flash clock — above the
# W25Q080's 104-133 MHz rated max.  Reads survive (single-shot at boot) but
# erase/program can't be clocked reliably at that rate, and preset saves
# silently fail verification.  Divider 6 → 51.2 MHz, safely within spec.
# Must be set BEFORE pico_sdk_init() so it propagates to the boot2 build too.
add_compile_definitions(PICO_FLASH_SPI_CLKDIV=6)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

# We also need PICO EXTRAS
include(pico_extras_import.cmake)

project(pico_playground C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Local multi-instance S/PDIF library (replaces system pico_audio_spdif)
add_subdirectory(pico-extras/src/rp2_common/pico_audio_spdif_multi)

# Multi-instance I2S library (same patterns as S/PDIF multi, independent)
add_subdirectory(pico-extras/src/rp2_common/pico_audio_i2s_multi)

add_subdirectory(DSPi)
