# wifi_heap_check.cmake - post-build malloc-heap budget report for the
# DSPI_ENABLE_WIFI build (run with cmake -P from CMakeLists.txt).
#
# Why: usb_sound_card_init() calloc()s the four S/PDIF producer pools at boot
# (4 x AUDIO_BUFFER_COUNT x 192 samples x 8 bytes) and pico_malloc panics when
# a calloc fails, so the heap between the linker's __end__ and __HeapLimit has
# to hold them.  The default (non-WiFi) build leaves only ~0.9 KB of slack; every
# static byte the WiFi stack adds comes out of that slack.  This script computes
# the free heap from the ELF and the pool demand from config.h and complains
# when the firmware could not get through boot.
#
# Inputs (-D): ELF, NM, CONFIG_H, MODE (WARN|FATAL), MARGIN (bytes of slack to keep)

if(NOT NM OR NOT EXISTS "${ELF}")
    message(STATUS "wifi heap check: nm/elf not available, skipped")
    return()
endif()

execute_process(COMMAND ${NM} ${ELF} OUTPUT_VARIABLE _syms RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
    message(STATUS "wifi heap check: nm failed, skipped")
    return()
endif()
string(REGEX MATCH "([0-9a-fA-F]+) [A-Za-z] __end__" _m1 "${_syms}")
set(_end "0x${CMAKE_MATCH_1}")
string(REGEX MATCH "([0-9a-fA-F]+) [A-Za-z] __HeapLimit" _m2 "${_syms}")
set(_lim "0x${CMAKE_MATCH_1}")
math(EXPR _heap "${_lim} - ${_end}")

set(_count 8)
file(STRINGS "${CONFIG_H}" _cfg REGEX "^#define[ \t]+AUDIO_BUFFER_COUNT[ \t]+[0-9]+")
if(_cfg)
    string(REGEX MATCH "AUDIO_BUFFER_COUNT[ \t]+([0-9]+)" _m3 "${_cfg}")
    set(_count ${CMAKE_MATCH_1})
endif()

# newlib malloc chunk = max(16, (request + 4 + 7) & ~7):
#   pool struct 44 -> 48, buffer array count*24 -> aligned, per buffer
#   mem_buffer_t 12 -> 16 and 192 * 8 = 1536 -> 1544
math(EXPR _arr "((${_count} * 24 + 4 + 7) / 8) * 8")
math(EXPR _pool "48 + ${_arr} + ${_count} * (16 + 1544)")
math(EXPR _need "4 * ${_pool}")
math(EXPR _slack "${_heap} - ${_need}")

message(STATUS "wifi heap budget: heap ${_heap} B (__end__ ${_end} .. __HeapLimit ${_lim}); "
               "producer pools at boot ~${_need} B (4 x AUDIO_BUFFER_COUNT=${_count}); slack ${_slack} B")
if(_slack LESS ${MARGIN})
    math(EXPR _short "${MARGIN} - ${_slack}")
    math(EXPR _per_buf "4 * 1560")
    math(EXPR _bufs "(${_short} + ${_per_buf} - 1) / ${_per_buf}")
    string(CONCAT _msg "DSPI_ENABLE_WIFI: the malloc heap is ${_short} B too small (slack ${_slack} B < ${MARGIN} B). "
             "usb_sound_card_init() would hit a failed calloc -> panic at boot. Cut ${_short} B of static RAM "
             "or reduce AUDIO_BUFFER_COUNT (config.h, now ${_count}) by ${_bufs} (each step frees ${_per_buf} B).")
    if(MODE STREQUAL "FATAL")
        message(FATAL_ERROR "${_msg}")
    else()
        message(WARNING "${_msg}")
    endif()
endif()
