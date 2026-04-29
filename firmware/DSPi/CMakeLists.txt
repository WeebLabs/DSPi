PROJECT(DSPi)

add_compile_options(-fcommon)
add_compile_options(-O2)

# Use -O3 for DSP-critical files
set_source_files_properties(
    dsp_pipeline.c usb_audio.c vendor_commands.c audio_pipeline.c crossfeed.c loudness.c leveller.c spdif_input.c
    PROPERTIES COMPILE_FLAGS "-O3"
)

add_executable(DSPi
    audio_input.c
    audio_input.h
    audio_pipeline.c
    audio_pipeline.h
    bulk_params.c
    bulk_params.h
    config.h
    control_executor.h
    control_transport.c
    control_transport.h
    crossfeed.c
    crossfeed.h
    dcp_inline.h
    dsp_pipeline.c
    dsp_pipeline.h
    flash_clkdiv.c
    flash_clkdiv.h
    flash_storage.c
    flash_storage.h
    leveller.c
    leveller.h
    loudness.c
    loudness.h
    main.c
    notify.c
    notify.h
    pdm_generator.c
    pdm_generator.h
    spdif_input.c
    spdif_input.h
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

if (PICO_PLATFORM STREQUAL "rp2040")
    # RP2040: SPDIF RX on PIO1 SM2 (SM0=PDM, SM1=MCK occupied)
    target_compile_definitions(DSPi PRIVATE PICO_SPDIF_RX_PIO=1)
else()
    # RP2350: SPDIF RX on PIO2 (dedicated block, proven working)
    target_compile_definitions(DSPi PRIVATE PICO_SPDIF_RX_PIO=2)
endif()
# PICO_FLASH_SPI_CLKDIV=6 is set at the top-level CMakeLists so it reaches
# the boot2 build (target-level defines don't propagate to it).

pico_set_binary_type(DSPi copy_to_ram)

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

    PICO_DEFAULT_UART_TX_PIN=12
)

foreach(dspi_control_define
    DSPI_CONTROL_I2C_PRIORITY
    DSPI_CONTROL_UART_PRIORITY
    DSPI_CONTROL_I2C_INSTANCE
    DSPI_CONTROL_I2C_ADDR
    DSPI_CONTROL_I2C_SDA_PIN
    DSPI_CONTROL_I2C_SCL_PIN
    DSPI_CONTROL_I2C_BAUD
    DSPI_CONTROL_UART_INSTANCE
    DSPI_CONTROL_UART_TX_PIN
    DSPI_CONTROL_UART_RX_PIN
    DSPI_CONTROL_UART_BAUD
)
    if (DEFINED ${dspi_control_define})
        target_compile_definitions(DSPi PRIVATE
            ${dspi_control_define}=${${dspi_control_define}}
        )
    endif()
endforeach()

target_link_libraries(DSPi
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_i2c
    hardware_uart
    hardware_pwm
    hardware_flash
    hardware_adc
    pico_audio_spdif_multi
    pico_audio_i2s_multi
    pico_spdif_rx
    tinyusb_device
)

pico_add_extra_outputs(DSPi)
