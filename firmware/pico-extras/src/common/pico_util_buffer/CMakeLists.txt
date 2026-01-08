add_library(pico_util_buffer INTERFACE)

target_include_directories(pico_util_buffer INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_sources(pico_util_buffer INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/buffer.c
        )