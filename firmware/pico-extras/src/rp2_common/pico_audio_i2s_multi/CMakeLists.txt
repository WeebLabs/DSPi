add_library(pico_audio_i2s_multi INTERFACE)

pico_generate_pio_header(pico_audio_i2s_multi ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_24.pio)
pico_generate_pio_header(pico_audio_i2s_multi ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_24_slave.pio)
pico_generate_pio_header(pico_audio_i2s_multi ${CMAKE_CURRENT_LIST_DIR}/audio_mck.pio)

target_sources(pico_audio_i2s_multi INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_multi.c
)

target_include_directories(pico_audio_i2s_multi INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(pico_audio_i2s_multi INTERFACE hardware_dma hardware_pio hardware_irq hardware_clocks hardware_sync pico_audio)
