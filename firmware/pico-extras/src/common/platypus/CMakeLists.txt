add_library(platypus INTERFACE)

target_sources(platypus INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/platypus.c
        $<$<BOOL:${PICO_ON_DEVICE}>:${CMAKE_CURRENT_LIST_DIR}/decompress_row.S>
        )

target_include_directories(platypus INTERFACE ${CMAKE_CURRENT_LIST_DIR})

if (PICO_ON_DEVICE)
    target_link_libraries(platypus INTERFACE hardware_interp)
endif()