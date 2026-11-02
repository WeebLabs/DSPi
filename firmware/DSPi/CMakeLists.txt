PROJECT(DSPi)

add_compile_options(-fcommon)
add_compile_options(-O2)

# Use -O3 for DSP-critical files
set_source_files_properties(
    dsp_pipeline.c usb_audio.c vendor_commands.c audio_pipeline.c crossfeed.c loudness.c psybass.c leveller.c crossover.c spdif_input.c i2s_input.c input_servo.c lg_sound_sync.c dac_hw_mute.c siggen.c upmix.c adat_output.c adat_input.c
    PROPERTIES COMPILE_FLAGS "-O3"
)

add_executable(DSPi
    adat_input.c
    adat_input.h
    adat_output.c
    adat_output.h
    audio_input.c
    audio_input.h
    audio_pipeline.c
    audio_pipeline.h
    bulk_params.c
    bulk_params.h
    config.h
    control_surfaces.c
    control_surfaces.h
    control_surfaces_ir.c
    control_surfaces_ir.h
    control_surfaces_nouns.c
    crossfeed.c
    crossfeed.h
    crossover.c
    crossover.h
    dac_hw_mute.c
    dac_hw_mute.h
    dcp_inline.h
    dsp_biquad.h
    dsp_pipeline.c
    dsp_pipeline.h
    dsp_svf.h
    flash_clkdiv.c
    flash_clkdiv.h
    flash_storage.c
    flash_storage.h
    i2c_control.c
    i2c_control.h
    i2s_input.c
    i2s_input.h
    input_servo.c
    input_servo.h
    leveller.c
    leveller.h
    lg_sound_sync.c
    lg_sound_sync.h
    loudness.c
    loudness.h
    main.c
    notify.c
    notify.h
    pdm_generator.c
    pdm_generator.h
    psybass.c
    psybass.h
    siggen.c
    siggen.h
    spdif_input.c
    spdif_input.h
    uart_control.c
    uart_control.h
    upmix.c
    upmix.h
    usb_audio.c
    usb_audio.h
    vendor_commands.c
    vendor_commands.h
    usb_descriptors.c
    usb_descriptors.h
    usb_feedback_controller.c
    usb_feedback_controller.h
    tusb_config.h
)

if (PICO_PLATFORM STREQUAL "rp2040")
    target_sources(DSPi PRIVATE dsp_process_rp2040.S)
endif()

# DSPI_LOOPBACK (debug/verification only): fold a USB audio CAPTURE function
# into DSPi that streams output slot 0 back to the host (replaces the external
# USBrx recorder).  Configure dedicated build dirs with -DDSPI_LOOPBACK=ON;
# normal build dirs leave it OFF so release artifacts are unaffected.
option(DSPI_LOOPBACK "Build the USB audio loopback capture (output slot 0 -> host)" OFF)
if (DSPI_LOOPBACK)
    target_sources(DSPi PRIVATE loopback.c loopback.h)
    set_source_files_properties(loopback.c PROPERTIES COMPILE_FLAGS "-O3")
    target_compile_definitions(DSPi PRIVATE DSPI_LOOPBACK)
endif()

if (PICO_PLATFORM STREQUAL "rp2040")
    # RP2040: SPDIF RX on PIO1 SM2 (SM0=PDM; SM1 free since MCK moved to
    # CLK_GPOUT). I2S RX reuses the same SM when SPDIF RX is inactive.
    target_compile_definitions(DSPi PRIVATE PICO_SPDIF_RX_PIO=1)
    # Hardware divider aeabi shims used by the fixed-point audio path; RP2350 uses native SDIV/UDIV.
    target_compile_definitions(DSPi PRIVATE PICO_DIVIDER_IN_RAM=1)
    # 64-bit mul and clz shims (feedback math, float internals); native on RP2350.
    target_compile_definitions(DSPi PRIVATE PICO_INT64_OPS_IN_RAM=1 PICO_BITS_IN_RAM=1)
else()
    # RP2350: SPDIF RX on PIO2 (dedicated block, proven working).
    # I2S RX reuses the same SM when SPDIF RX is inactive.
    target_compile_definitions(DSPi PRIVATE PICO_SPDIF_RX_PIO=2)
endif()
# PICO_FLASH_SPI_CLKDIV=6 is set at the top-level CMakeLists so it reaches
# the boot2 build (target-level defines don't propagate to it).

# XIP execution: cold code runs from flash through the XIP cache; hot
# audio/USB code stays in RAM via .time_critical / __not_in_flash_func
# attributes and the RAM code pull list in the custom scripts below.
# scripts/check_ram_placement.py enforces the hot-symbol placement.
pico_set_binary_type(DSPi default)

if (PICO_PLATFORM STREQUAL "rp2040")
    pico_set_linker_script(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/memmap_dspi_rp2040_xip.ld)
else()
    pico_set_linker_script(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/memmap_dspi_rp2350_xip.ld)
endif()

pico_generate_pio_header(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/i2s_input.pio)
pico_generate_pio_header(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/adat_input.pio)

target_include_directories(DSPi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(DSPi PRIVATE
    AUDIO_FREQ_MAX=48000

    # S/PDIF output configuration (pins defined in config.h)
    PICO_AUDIO_SPDIF_PIO=0
    PICO_AUDIO_SPDIF_DMA_IRQ=1
    # I2S TX on DMA_IRQ_0 — must stay separate from SPDIF TX (DMA_IRQ_1)
    # to avoid cross-driver IRQ gating between the two output libraries.
    PICO_AUDIO_I2S_DMA_IRQ=0

    # S/PDIF RX on DMA_IRQ_1 with SPDIF TX. Our library patches ensure
    # safe coexistence: private irq_handler_registered flag (not
    # irq_has_shared_handler), removed irq_set_enabled in teardown,
    # interrupt-safe _spdif_rx_common_end.
    PICO_SPDIF_RX_DMA_IRQ=1

    # RAM-places the TinyUSB rp2040-port IRQ path; both platforms use this USB port.
    PICO_RP2040_USB_FAST_IRQ=1
    # SDK float shims go to .time_critical; used per block by the feedback servo and leveller.
    PICO_FLOAT_IN_RAM=1
    # Double shims too: the leveller's powf reaches double helpers per block.
    PICO_DOUBLE_IN_RAM=1
    # RP2040 mem_ops ROM shims (memset/memcpy wraps) go to RAM; called from core 1
    # and per-block paths. No effect on RP2350 (compiler mem ops; see linker script).
    PICO_MEM_IN_RAM=1
)

# ----------------------------------------------------------------------------
# WiFi / network audio input (-DDSPI_ENABLE_WIFI=ON, RP2350 + pico2_w only)
# ----------------------------------------------------------------------------
if (DSPI_ENABLE_WIFI)
    if (NOT PICO_PLATFORM MATCHES "rp2350")
        message(FATAL_ERROR "DSPI_ENABLE_WIFI requires PICO_PLATFORM=rp2350 (Pico 2 W)")
    endif()
    if (NOT PICO_CYW43_SUPPORTED)
        message(FATAL_ERROR "DSPI_ENABLE_WIFI requires a CYW43 board: -DPICO_BOARD=pico2_w")
    endif()

    target_compile_definitions(DSPi PRIVATE DSPI_ENABLE_WIFI=1)
    # Core 0 runs lwIP -> Sendspin -> Noise/X25519 (mbedtls bignum) from the
    # main loop: 2 KB (SDK default) is too little.  The stack lives in
    # SCRATCH_Y, which has 2 KB of otherwise unused space below it.
    target_compile_definitions(DSPi PRIVATE PICO_STACK_SIZE=0x1000)
    # lwipopts.h (NO_SYS poll mode, small pools) is found via the DSPi include dir.
    target_sources(DSPi PRIVATE
        net_input.c
        net_input.h
        lwipopts.h
        dspi_mbedtls_config.h
    )
    set_source_files_properties(net_input.c PROPERTIES COMPILE_FLAGS "-O3")

    # Sendspin client + noise/secure-channel code (written separately).
    foreach(_f sendspin_noise.c sendspin_client.c)
        if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${_f})
            target_sources(DSPi PRIVATE ${_f})
        else()
            message(WARNING "DSPI_ENABLE_WIFI: ${_f} not found in DSPi/ (link will fail unless DSPI_NET_EXTRA_SOURCES provides it)")
        endif()
    endforeach()
    # Development aid: extra sources / include dirs (e.g. a local stub of the
    # Sendspin client kept outside the repo): -DDSPI_NET_EXTRA_SOURCES="a.c;b.c"
    # -DDSPI_NET_EXTRA_INCLUDES="dir".
    if (DSPI_NET_EXTRA_SOURCES)
        target_sources(DSPi PRIVATE ${DSPI_NET_EXTRA_SOURCES})
    endif()
    if (DSPI_NET_EXTRA_INCLUDES)
        target_include_directories(DSPi PRIVATE ${DSPI_NET_EXTRA_INCLUDES})
    endif()

    # lwIP in poll mode: cyw43_arch_poll() (called from net_input_poll) is the
    # only place the network stack runs; no background IRQ/async worker.
    target_link_libraries(DSPi
        pico_cyw43_arch_lwip_poll
        pico_lwip
        pico_mbedtls
        pico_rand
    )
endif()

target_link_libraries(DSPi
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pwm
    hardware_flash
    hardware_adc
    hardware_i2c
    pico_i2c_slave
    pico_audio_spdif_multi
    pico_audio_i2s_multi
    pico_spdif_rx
    tinyusb_device
)

# Debug UART removed; GPIO 16/17 are freed for the UART control interface.
pico_enable_stdio_uart(DSPi 0)

pico_add_extra_outputs(DSPi)
