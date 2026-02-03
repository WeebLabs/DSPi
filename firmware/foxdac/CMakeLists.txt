PROJECT(foxdac)

add_compile_options(-fcommon)
add_compile_options(-O2)

add_executable(foxdac
    config.h
    dcp_inline.h
    dsp_pipeline.c
    dsp_pipeline.h
    flash_storage.c
    flash_storage.h
    main.c
    pdm_generator.c
    pdm_generator.h
    usb_audio.c
    usb_audio.h
    usb_descriptors.c
    usb_descriptors.h
)

if (PICO_PLATFORM STREQUAL "rp2040")
    target_sources(foxdac PRIVATE dsp_process_rp2040.S)
endif()

pico_set_binary_type(foxdac copy_to_ram)

target_include_directories(foxdac PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lufa
)

target_compile_definitions(foxdac PRIVATE
    AUDIO_FREQ_MAX=48000

    PICO_AUDIO_SPDIF_PIN=15 # 14 for v1 ; 15 for v2
    PICO_AUDIO_SPDIF_DMA_IRQ=1

    PICO_DEFAULT_UART_TX_PIN=12

    # pico-extras usb_device configuration
    PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1
    PICO_USBDEV_MAX_DESCRIPTOR_SIZE=256
    PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE=2
)

target_link_libraries(foxdac
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pwm
    hardware_flash
    pico_audio_spdif
    usb_device
)

pico_add_extra_outputs(foxdac)
