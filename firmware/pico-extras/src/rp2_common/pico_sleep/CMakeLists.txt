pico_simple_hardware_target(sleep)
target_link_libraries(hardware_sleep INTERFACE
        hardware_clocks
        hardware_rosc
        hardware_rtc)