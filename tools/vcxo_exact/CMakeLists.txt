cmake_minimum_required(VERSION 3.13)

# Standalone FBDIV pulse-count bench (RP2350): PIO TDC against the crystal,
# DMA-timer pulse tables, PIO-paced pulses, bus load. USB CDC command shell.
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../../firmware/pico-sdk)
add_compile_definitions(PICO_FLASH_SPI_CLKDIV=6)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(vcxo_exact C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(vcxo_exact main.c)
pico_generate_pio_header(vcxo_exact ${CMAKE_CURRENT_LIST_DIR}/tdc.pio)
target_link_libraries(vcxo_exact pico_stdlib pico_multicore
    hardware_dma hardware_pio hardware_pwm hardware_pll hardware_clocks hardware_vreg hardware_watchdog hardware_spi hardware_resets)
pico_set_binary_type(vcxo_exact copy_to_ram)
pico_enable_stdio_usb(vcxo_exact 1)
pico_enable_stdio_uart(vcxo_exact 0)
pico_add_extra_outputs(vcxo_exact)
