cmake_minimum_required(VERSION 3.12)

# Default SDK path to submodule location if not set via environment
if (NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
endif()

# Always use local pico-extras (contains modified libraries)
set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-extras CACHE PATH "" FORCE)

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

add_subdirectory(foxdac)
