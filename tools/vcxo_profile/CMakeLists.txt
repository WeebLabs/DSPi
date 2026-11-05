cmake_minimum_required(VERSION 3.13)

# Standalone soft-VCXO profiler. Builds the firmware's real soft_vcxo.c
# against the DSPi 307.2 MHz clock plan; results go out over USB CDC.
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../../firmware/pico-sdk)
add_compile_definitions(PICO_FLASH_SPI_CLKDIV=6)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(vcxo_profile C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(DSPI_DIR ${CMAKE_CURRENT_LIST_DIR}/../../firmware/DSPi)
add_executable(vcxo_profile main.c ${DSPI_DIR}/soft_vcxo.c)
# main.c includes DSPi headers by path: the DSPi dir also holds a tusb_config.h without CDC.
target_compile_definitions(vcxo_profile PRIVATE DSPI_CLOCK_DIAG=1)
target_link_libraries(vcxo_profile pico_stdlib pico_multicore
    hardware_dma hardware_pwm hardware_pll hardware_clocks hardware_vreg)
# Run from RAM so the profile never waits on XIP at the overclocked plan.
pico_set_binary_type(vcxo_profile copy_to_ram)
pico_enable_stdio_usb(vcxo_profile 1)
pico_enable_stdio_uart(vcxo_profile 0)
pico_add_extra_outputs(vcxo_profile)
