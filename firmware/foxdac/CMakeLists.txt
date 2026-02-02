PROJECT(foxdac)

add_compile_options(-fcommon)
add_compile_options(-O2)

add_executable(foxdac
    config.h
    dsp_pipeline.c
    dsp_pipeline.h
    flash_storage.c
    flash_storage.h
    main.c
    pdm_generator.c
    pdm_generator.h
    tusb_config.h
    usb_audio.c
    usb_audio.h
    usb_descriptors.c
    usb_descriptors.h
)

pico_set_binary_type(foxdac copy_to_ram)

target_include_directories(foxdac PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(foxdac PRIVATE
    AUDIO_FREQ_MAX=48000

    PICO_AUDIO_SPDIF_PIN=15 # 14 for v1 ; 15 for v2
    PICO_AUDIO_SPDIF_DMA_IRQ=1

    PICO_DEFAULT_UART_TX_PIN=12

    # CFG_TUSB_MCU is set automatically by pico-sdk based on target board
)

target_link_libraries(foxdac
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pwm
    hardware_flash
    pico_audio_spdif
    tinyusb_device
    tinyusb_board
)

pico_add_extra_outputs(foxdac)
