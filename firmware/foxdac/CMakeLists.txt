if (TARGET usb_device)
    PROJECT(foxdac)

    add_compile_options(-fcommon)
    add_compile_options(-O2)

    add_executable(foxdac config.h dsp_pipeline.c dsp_pipeline.h main.c pdm_generator.c pdm_generator.h usb_audio.c usb_audio.h usb_descriptors.c usb_descriptors.h)

    pico_set_binary_type(foxdac copy_to_ram)

    target_compile_definitions(foxdac PRIVATE
            AUDIO_FREQ_MAX=48000

            # ours are zero based, so say so
            PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1

            # need large descriptor
            PICO_USBDEV_MAX_DESCRIPTOR_SIZE=256

            PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE=2

            PICO_AUDIO_SPDIF_PIN=15 # 14 for v1 ; 15 for v2
            PICO_AUDIO_SPDIF_DMA_IRQ=1
            
            PICO_DEFAULT_UART_TX_PIN=12
    )

    target_link_libraries(foxdac pico_stdlib usb_device hardware_pwm pico_audio_spdif pico_multicore pico_unique_id)
    pico_add_extra_outputs(foxdac)
endif()
