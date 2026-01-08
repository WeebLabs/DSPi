if (NOT TARGET pico_audio_i2s)
    add_library(pico_audio_i2s INTERFACE)

    pico_generate_pio_header(pico_audio_i2s ${CMAKE_CURRENT_LIST_DIR}/audio_i2s.pio)

    target_sources(pico_audio_i2s INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/audio_i2s.c
    )

    target_include_directories(pico_audio_i2s INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(pico_audio_i2s INTERFACE hardware_dma hardware_pio hardware_irq pico_audio)
endif()