PROJECT(DSPi)

add_compile_options(-fcommon)
add_compile_options(-O2)

# Use -O3 for DSP-critical files
set_source_files_properties(
    dsp_pipeline.c usb_audio.c crossfeed.c loudness.c
    PROPERTIES COMPILE_FLAGS "-O3"
)

add_executable(DSPi
    bulk_params.c
    bulk_params.h
    config.h
    crossfeed.c
    crossfeed.h
    dcp_inline.h
    dsp_pipeline.c
    dsp_pipeline.h
    flash_storage.c
    flash_storage.h
    loudness.c
    loudness.h
    main.c
    pdm_generator.c
    pdm_generator.h
    usb_audio.c
    usb_audio.h
    usb_descriptors.c
    usb_descriptors.h
)

if (PICO_PLATFORM STREQUAL "rp2040")
    target_sources(DSPi PRIVATE dsp_process_rp2040.S)
endif()

pico_set_binary_type(DSPi copy_to_ram)

target_include_directories(DSPi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lufa
)

target_compile_definitions(DSPi PRIVATE
    AUDIO_FREQ_MAX=48000

    # S/PDIF output configuration (pins defined in config.h)
    PICO_AUDIO_SPDIF_PIO=0
    PICO_AUDIO_SPDIF_DMA_IRQ=1
    # Keep I2S on a separate DMA IRQ line to avoid cross-driver IRQ gating.
    PICO_AUDIO_I2S_DMA_IRQ=0

    PICO_DEFAULT_UART_TX_PIN=12

    # pico-extras usb_device configuration
    PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1
    PICO_USBDEV_MAX_DESCRIPTOR_SIZE=256
    PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE=2
)

target_link_libraries(DSPi
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pwm
    hardware_flash
    hardware_adc
    pico_audio_spdif_multi
    pico_audio_i2s_multi
    usb_device
)

pico_add_extra_outputs(DSPi)
