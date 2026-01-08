if (NOT TARGET pico_sd_card_headers)
    add_library(pico_sd_card_headers INTERFACE)
    target_include_directories(pico_sd_card_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
endif()