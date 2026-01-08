if (NOT TARGET pico_sd_card)
    add_library(pico_sd_card INTERFACE)
    
    pico_generate_pio_header(pico_sd_card ${CMAKE_CURRENT_LIST_DIR}/sd_card.pio)
    
    target_sources(pico_sd_card INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/sd_card.c
    )
    
    target_include_directories(pico_sd_card INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(pico_sd_card INTERFACE pico_sd_card_headers hardware_dma hardware_pio)
endif()