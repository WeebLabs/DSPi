This repo has additional libraries that are not yet ready for inclusion the Pico SDK proper,
or are just useful but don't necessarily belong in the Pico SDK.

Note that any API here is a work in progress and subject to change.

See [pico-playground](https://github.com/raspberrypi/pico-playground) for buildable example code using these extra libraries.


Library|Description 
---|---
[hardware_rosc](src/rp2_common/hardware_rosc)| API for the ring oscillator
[lwip](src/rp2_common/lwip)| [LWIP Lightweight IP Library](https://savannah.nongnu.org/projects/lwip/) packed as an INTERFACE library for use with the Pico SDK
[pico_audio](src/common/pico_audio)|Audio output support; this is highly functional, but the API is subject to change 
&nbsp;&nbsp;&nbsp;[pico_audio_i2s](src/rp2_common/pico_audio_i2s)|Audio output via I2S on 3 GPIOs using PIO. Arbitrary frequency
&nbsp;&nbsp;&nbsp;[pico_audio_pwm](src/rp2_common/pico_audio_pwm)|Audio output via (PIO) PWM. Currently a bit limited in frequency support (it was developed on FPGA to do 22050Hz at 48Mhz system clock). It does however support error diffusion dithering and noise shaping with 16x oversampling to give surprisingly good audio quality. This code will be split to provide both a fixed frequencie(s) version and a slightly slower but rather better arbitrary frequency version supporting ever higher carrier frequencies 
&nbsp;&nbsp;&nbsp;[pico_audio_spdif](src/rp2_common/pico_audio_spdif)|Audio output in S/PDIF on a GPIO using PIO. Supports up to 192khz stereo. Consumed OK in test, haven't tried it with real hardware
[pico_sd_card](src/rp2_common/pico_sd_card)|1 and 4 bit SDIO support using PIO. This is functional (currently writing is only 1 bit), but the the code is very much prototype and the API is just a placeholder - the command set needs to be separated from the SDIO and shared with SPI
[pico_sleep](src/rp2_common/pico_sleep)|Low power related APIs, WIP because they are not sufficiently generic and also only handle core 0
[pico_scanvideo](src/common/pico_scanvideo)|Support for video output where every pixel is _scanned out_ every frame. VGA/DPI support is highly functional and stable, but the API is subject to change
&nbsp;&nbsp;&nbsp;[pico_scanvideo_dbi](src/rp2_common/pico_scanvideo_dbi)| currently non-compiling... placeholder for adding scanvideo over MIPI DBI support.
&nbsp;&nbsp;&nbsp;[pico_scanvideo_dpi](src/rp2_common/pico_scanvideo_dpi)| Highly functional and stable support for parallel RGB output and VSYNC/HSYNC/DEN/CLOCK for VGA/DPI.
[pico_util_buffer](src/common/pico_util_buffer)|Rather incomplete buffer abstraction, used by pico_audio and pico_scanvideo
[platypus](src/common/platypus)| Decoder for a custom image compression format suitable for dithered images (good for RGB555) and suitable for decoding on RP2040 at scanline speeds ... i.e you can easily decode a 320x240 image 60x per second to avoid storing the uncompressed image for scanout video. It gets about 50% compression (but is designed only for 4x4 fixed dithered RGB555 images, so is somewhat specific!). TODO add the encoder here :-)
[usb_device](src/rp2_common/usb_device), [usb_common](src/rp2_common/usb_common)| The custom and somewhat minimal USB device stack used in the bootrom. We now use TinyUSB in the Pico SDK but kept here for posterity
[usb_device_msc](src/rp2_common/usb_device_msc)| USB Mass Storage Class implementation using _usb_device_

You can add Pico Extras to your project similarly to the SDK (copying [external/pico_extras_import.cmake](external/pico_extras_import.cmake) into your project)
having set the `PICO_EXTRAS_PATH` variable in your environment or via cmake variable.

```cmake
cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

# We also need PICO EXTRAS
include(pico_extras_import.cmake)

project(pico_playground C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
``` 

Alternative you can inject it into an existing project without modifying it via `PICO_CMAKE_POST_LIST_DIRS`
 by passing `-DPICO_SDK_POST_LIST_DIRS=/path/to/pico_extras` to cmake
