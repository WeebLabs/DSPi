if (PICO_ON_DEVICE)
    if (TARGET pico_sd_card)
        add_executable(sd_test
                sd_test.c
                )

#        target_compile_definitions(sd_test PRIVATE
#                PICO_DEFAULT_UART_TX_PIN=28
#                PICO_DEFAULT_UART_RX_PIN=29
#                PICO_DEFAULT_UART=0
#                )
        target_link_libraries(sd_test pico_stdlib pico_sd_card)
        pico_add_extra_outputs(sd_test)
    endif()
endif()