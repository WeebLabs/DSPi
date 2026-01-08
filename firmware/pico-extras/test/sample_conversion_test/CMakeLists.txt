if (NOT PICO_ON_DEVICE OR NOT PICO_NO_FLASH) # too big for RAM
    add_executable(sample_conversion_test sample_conversion_test.cpp)

    target_compile_definitions(sample_conversion_test PRIVATE
            #PICO_ENTER_USB_BOOT_ON_EXIT=1
            )
    target_link_libraries(sample_conversion_test PRIVATE pico_stdlib pico_audio)
    pico_add_extra_outputs(sample_conversion_test)
endif()
