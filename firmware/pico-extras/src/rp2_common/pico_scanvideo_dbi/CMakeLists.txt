add_library(video_dbi INTERFACE)

# todo right now this is just a copy of video/
pico_generate_pio_header(video_dbi ${CMAKE_CURRENT_LIST_DIR}/video.pio)
pico_generate_pio_header(video_dbi ${CMAKE_CURRENT_LIST_DIR}/control.pio)

target_sources(video_dbi INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/video.h
        ${CMAKE_CURRENT_LIST_DIR}/vga_modes.c
        $<$<BOOL:${UseHardware}>:${CMAKE_CURRENT_LIST_DIR}/tft_driver.c>
        $<$<BOOL:${UseHardware}>:${CMAKE_CURRENT_LIST_DIR}/video_dbi.c>
        )

target_compile_definitions(video_dbi INTERFACE VIDEO_DBI)
target_include_directories(video_dbi INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(video_dbi INTERFACE dma pio)