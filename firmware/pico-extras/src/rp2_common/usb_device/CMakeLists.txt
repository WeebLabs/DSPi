add_library(usb_device INTERFACE)

target_sources(usb_device INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/usb_device.c
        ${CMAKE_CURRENT_LIST_DIR}/usb_stream_helper.c
)

target_include_directories(usb_device INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(usb_device INTERFACE usb_common hardware_irq hardware_dma hardware_pio pico_fix_rp2040_usb_device_enumeration)