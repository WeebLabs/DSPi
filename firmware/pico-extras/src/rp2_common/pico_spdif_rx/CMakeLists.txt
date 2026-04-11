# pico_spdif_rx — S/PDIF receiver library for DSPi
# Forked from elehobica/pico_spdif_rx v0.9.3 (BSD-2-Clause)
#
# DSPi patches:
#   - PIO2 support for RP2350
#   - Clock constants for 307.2 MHz sys_clk / 122.88 MHz PIO clock
#   - Removed pio_clear_instruction_memory() (destroys shared PIO programs)
#   - Pre-generated PIO files (no Python dependency)

cmake_minimum_required(VERSION 3.12)
if (NOT TARGET pico_spdif_rx)
    add_library(pico_spdif_rx INTERFACE)

    pico_generate_pio_header(pico_spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_48000.pio)
    pico_generate_pio_header(pico_spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_96000.pio)
    pico_generate_pio_header(pico_spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_192000.pio)
    pico_generate_pio_header(pico_spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_capture.pio)

    target_sources(pico_spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.c
    )

    target_include_directories(pico_spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(pico_spdif_rx INTERFACE
        pico_stdlib
        hardware_dma
        hardware_pio
        hardware_irq
    )
endif()
