# This CMakeLists.txt intended to be included from other projectgs
pico_add_subdirectory(hardware_rosc)
pico_add_subdirectory(lwip)
pico_add_subdirectory(pico_sleep)
pico_add_subdirectory(pico_audio_i2s)
pico_add_subdirectory(pico_audio_pwm)
pico_add_subdirectory(pico_audio_spdif)
pico_add_subdirectory(pico_sd_card)
# currently very old and non-compiling
#pico_add_subdirectory(pico_scanvideo_dbi)
pico_add_subdirectory(pico_scanvideo_dpi)
pico_add_subdirectory(usb_common)
pico_add_subdirectory(usb_device)
pico_add_subdirectory(usb_device_msc)