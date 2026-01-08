add_library(usb_device_msc INTERFACE)

target_sources(usb_device_msc INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/usb_device_msc.c
)

target_include_directories(usb_device_msc INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(usb_device_msc INTERFACE usb_device)