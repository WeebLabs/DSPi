add_library(pico_extras_included INTERFACE)
target_compile_definitions(pico_extras_included INTERFACE
        -DPICO_EXTRAS=1
        )

pico_add_platform_library(pico_extras_included)

# note as we're a .cmake included by the SDK, we're relative to the pico-sdk build
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src ${CMAKE_BINARY_DIR}/pico_extras/src)

if (PICO_EXTRAS_TESTS_ENABLED OR PICO_EXTRAS_TOP_LEVEL_PROJECT)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test {CMAKE_BINARY_DIR}/pico_extras/test)
endif ()

