if (NOT TARGET pico_audio_pwm)
    add_library(pico_audio_pwm INTERFACE)

    pico_generate_pio_header(pico_audio_pwm ${CMAKE_CURRENT_LIST_DIR}/audio_pwm.pio)

    target_sources(pico_audio_pwm INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/audio_pwm.c
            ${CMAKE_CURRENT_LIST_DIR}/sample_encoding.cpp
            )

    target_include_directories(pico_audio_pwm INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(pico_audio_pwm INTERFACE
            hardware_dma
            hardware_pio
            hardware_irq
            hardware_interp
            pico_audio
            pico_multicore)
endif()