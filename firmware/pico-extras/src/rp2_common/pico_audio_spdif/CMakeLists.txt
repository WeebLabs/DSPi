if (NOT TARGET pico_audio_spdif)
    add_library(pico_audio_spdif INTERFACE)

    pico_generate_pio_header(pico_audio_spdif ${CMAKE_CURRENT_LIST_DIR}/audio_spdif.pio)

    target_sources(pico_audio_spdif INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/audio_spdif.c
            ${CMAKE_CURRENT_LIST_DIR}/sample_encoding.cpp
    )

    target_include_directories(pico_audio_spdif INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(pico_audio_spdif INTERFACE hardware_dma hardware_pio hardware_irq pico_audio)
endif()