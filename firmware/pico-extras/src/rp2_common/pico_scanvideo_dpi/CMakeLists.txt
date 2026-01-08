if (TARGET pico_scanvideo)
    add_library(pico_scanvideo_dpi INTERFACE)

    pico_generate_pio_header(pico_scanvideo_dpi ${CMAKE_CURRENT_LIST_DIR}/timing.pio)

    target_sources(pico_scanvideo_dpi INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/scanvideo.c
    )

    target_include_directories(pico_scanvideo_dpi INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_compile_definitions(pico_scanvideo_dpi INTERFACE VIDEO_DPI)
    target_link_libraries(pico_scanvideo_dpi INTERFACE hardware_dma hardware_pio hardware_irq pico_scanvideo)
endif()