PROJECT(DSPi)

add_compile_options(-fcommon)
add_compile_options(-O2)

# Use -O3 for DSP-critical files
set_source_files_properties(
    usb_audio.c vendor_commands.c audio_pipeline.c crossfeed.c loudness.c psybass.c leveller.c crossover.c spdif_input.c i2s_input.c input_servo.c lg_sound_sync.c dac_hw_mute.c siggen.c upmix.c adat_output.c adat_input.c
    PROPERTIES COMPILE_FLAGS "-O3"
)

# dsp_pipeline.c is the only TU emitting the EQ kernels (inlined into
# dsp_cascade_block). -ffp-contract=off is a measured 14-24% win there; M33
# VFMA occupies the FPU ~2-2.5 cycles vs 1 for VMUL/VADD. subharm.c gets the
# same flag on a static estimate (~9% fewer cycles, 25 -> 4 VMOVs); confirm on
# the CPU meter. tube.c gets it on the same static-estimate basis (its shaper
# is the same shape of short multiply-add chain); confirm on the CPU meter.
# See "FP contraction" in Documentation/current_architecture.md.
set_source_files_properties(
    dsp_pipeline.c subharm.c tube.c
    PROPERTIES COMPILE_FLAGS "-O3 -ffp-contract=off"
)

add_executable(DSPi
    adat_input.c
    adat_input.h
    adat_output.c
    adat_output.h
    audio_input.c
    audio_input.h
    audio_clock_div.h
    audio_pipeline.c
    audio_pipeline.h
    bulk_params.c
    bulk_params.h
    config.h
    control_surfaces.c
    control_surfaces.h
    control_surfaces_display.c
    control_surfaces_display.h
    control_surfaces_ir.c
    control_surfaces_ir.h
    control_surfaces_nouns.c
    clock_diag.c
    crossfeed.c
    crossfeed.h
    crossover.c
    crossover.h
    dac_hw_mute.c
    dac_hw_mute.h
    dcp_inline.h
    dsp_biquad.h
    dsp_pipeline.c
    dsp_pipeline.h
    dsp_svf.h
    flash_clkdiv.c
    flash_clkdiv.h
    flash_storage.c
    flash_storage.h
    i2c_control.c
    i2c_control.h
    i2s_input.c
    i2s_input.h
    input_servo.c
    input_servo.h
    leveller.c
    leveller.h
    lg_sound_sync.c
    lg_sound_sync.h
    loudness.c
    loudness.h
    main.c
    notify.c
    notify.h
    pdm_generator.c
    pdm_generator.h
    psybass.c
    psybass.h
    subharm.c
    subharm.h
    tube.c
    tube.h
    limiter.c
    limiter.h
    siggen.c
    siggen.h
    rta_bass.c
    rta_bass.h
    rta_bass_tables.h
    rta.c
    rta.h
    rta_fft.c
    rta_fft.h
    rta_tables.h
    soft_vcxo.c
    soft_vcxo.h
    spdif_input.c
    spdif_input.h
    iec61937.h
    uart_control.c
    uart_control.h
    upmix.c
    upmix.h
    usb_audio.c
    usb_audio.h
    vendor_commands.c
    vendor_commands.h
    usb_descriptors.c
    usb_descriptors.h
    usb_feedback_controller.c
    usb_feedback_controller.h
    tusb_config.h
)

if (PICO_PLATFORM STREQUAL "rp2040")
    target_sources(DSPi PRIVATE dsp_process_rp2040.S)
endif()

# DSPI_LOOPBACK (debug/verification only): fold a USB audio CAPTURE function
# into DSPi that streams output slot 0 back to the host (replaces the external
# USBrx recorder).  Configure dedicated build dirs with -DDSPI_LOOPBACK=ON;
# normal build dirs leave it OFF so release artifacts are unaffected.
option(DSPI_LOOPBACK "Build the USB audio loopback capture (output slot 0 -> host)" OFF)
if (DSPI_LOOPBACK)
    target_sources(DSPi PRIVATE loopback.c loopback.h)
    set_source_files_properties(loopback.c PROPERTIES COMPILE_FLAGS "-O3")
    target_compile_definitions(DSPi PRIVATE DSPI_LOOPBACK)
endif()

# DSPI_CLOCK_DIAG (bench instrumentation): clock/servo/ring diagnostics and
# the REQ_GET_CLOCK_DIAG (0x82) snapshot read by tools/clkdiag_poll.py.
# Release builds leave it OFF; 0x82 then answers with zero bytes.
option(DSPI_CLOCK_DIAG "Build the clock/audio-path diagnostics (vendor GET 0x82)" OFF)
if (DSPI_CLOCK_DIAG)
    target_compile_definitions(DSPi PRIVATE DSPI_CLOCK_DIAG=1)
endif()

# DSPI_AC3_BENCH (RP2350 bench only): compiles the ported liba52 AC-3 decoder
# (ac3/) plus ac3_bench.c, which decodes an embedded frame set on request
# (REQ_AC3_BENCH 0x9F) and reports timing and a host-checked checksum.
# DSPI_AC3_RAM_HOT pins the decoder's hot paths in RAM (.time_critical);
# OFF measures the XIP-only variant.  Release builds leave both OFF.
option(DSPI_AC3_BENCH "Build the AC-3 decode bench (vendor 0x9F)" OFF)
set(DSPI_AC3_RAM_HOT "1" CACHE STRING "AC-3 RAM placement: 0 none, 1 kernels, 2 kernels + block parser")
set(DSPI_AC3_OPT "-O2" CACHE STRING "Optimisation level for the AC-3 decoder sources")
if (DSPI_AC3_BENCH AND NOT PICO_PLATFORM STREQUAL "rp2040")
    set(AC3_SOURCES ac3/parse.c ac3/bitstream.c ac3/bit_allocate.c ac3/downmix.c
                    ac3/imdct.c ac3/crc.c ac3/ac3_decoder.c)
    target_sources(DSPi PRIVATE ${AC3_SOURCES} ac3_bench.c ac3_bench.h)
    # -ffp-contract=off keeps the device decode bit-exact with the host
    # harness (checksum check), whatever the speed cost of the lost FMAs.
    set_source_files_properties(${AC3_SOURCES} ac3_bench.c PROPERTIES
        COMPILE_FLAGS "${DSPI_AC3_OPT} -ffp-contract=off")
    target_compile_definitions(DSPi PRIVATE DSPI_AC3_BENCH=1)
    target_compile_definitions(DSPi PRIVATE DSPI_AC3_RAM_HOT=${DSPI_AC3_RAM_HOT})
    # The embedded frames come from the committed vector and its checksum.
    set(AC3_BENCH_VECTOR ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/ac3_test/vectors/bench_51_640k.ac3)
    set(AC3_BENCH_HEADER ${CMAKE_CURRENT_BINARY_DIR}/ac3_bench_frames.h)
    add_custom_command(
        OUTPUT ${AC3_BENCH_HEADER}
        COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/ac3_test/gen_bench_frames.py
                ${AC3_BENCH_VECTOR} ${AC3_BENCH_HEADER}
        DEPENDS ${AC3_BENCH_VECTOR} ${AC3_BENCH_VECTOR}.sum
                ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/ac3_test/gen_bench_frames.py
        COMMENT "Embedding AC-3 bench frames"
        VERBATIM)
    target_sources(DSPi PRIVATE ${AC3_BENCH_HEADER})
    target_include_directories(DSPi PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
endif()

if (PICO_PLATFORM STREQUAL "rp2040")
    # RP2040: SPDIF RX on PIO1 SM2 (SM0=PDM; SM1 free since MCK moved to
    # CLK_GPOUT). I2S RX reuses the same SM when SPDIF RX is inactive.
    target_compile_definitions(DSPi PRIVATE PICO_SPDIF_RX_PIO=1)
    # Hardware divider aeabi shims used by the fixed-point audio path; RP2350 uses native SDIV/UDIV.
    target_compile_definitions(DSPi PRIVATE PICO_DIVIDER_IN_RAM=1)
    # 64-bit mul and clz shims (feedback math, float internals); native on RP2350.
    target_compile_definitions(DSPi PRIVATE PICO_INT64_OPS_IN_RAM=1 PICO_BITS_IN_RAM=1)
else()
    # RP2350: SPDIF RX on PIO2 (dedicated block, proven working).
    # I2S RX reuses the same SM when SPDIF RX is inactive.
    target_compile_definitions(DSPi PRIVATE PICO_SPDIF_RX_PIO=2)
    # Float bass kernel: same FP-contraction rule as dsp_pipeline.c (8 VFMAs -> 0,
    # unmeasured). RP2040's fixed-point build keeps -O2; -O3 only added 1.6 KB of RAM code.
    set_source_files_properties(rta_bass.c PROPERTIES COMPILE_FLAGS "-O3 -ffp-contract=off")
endif()
# PICO_FLASH_SPI_CLKDIV=6 is set at the top-level CMakeLists so it reaches
# the boot2 build (target-level defines don't propagate to it).

# XIP execution: cold code runs from flash through the XIP cache; hot
# audio/USB code stays in RAM via .time_critical / __not_in_flash_func
# attributes and the RAM code pull list in the custom scripts below.
# scripts/check_ram_placement.py enforces the hot-symbol placement.
pico_set_binary_type(DSPi default)

if (PICO_PLATFORM STREQUAL "rp2040")
    pico_set_linker_script(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/memmap_dspi_rp2040_xip.ld)
else()
    pico_set_linker_script(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/memmap_dspi_rp2350_xip.ld)
endif()

pico_generate_pio_header(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/i2s_input.pio)
pico_generate_pio_header(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/adat_input.pio)

target_include_directories(DSPi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

# Git build stamp, regenerated on EVERY build so the header tracks the
# checkout rather than the last configure. The script writes only on change,
# so dependents recompile only when the hash, dirty state, or date moved.
set(BUILD_INFO_HEADER ${CMAKE_CURRENT_BINARY_DIR}/generated/build_info.h)
add_custom_target(dspi_build_info
    COMMAND ${CMAKE_COMMAND}
        -DREPO_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -DOUT_FILE=${BUILD_INFO_HEADER}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/gen_build_info.cmake
    BYPRODUCTS ${BUILD_INFO_HEADER}
    COMMENT "Stamping git build info"
    VERBATIM)
add_dependencies(DSPi dspi_build_info)

target_compile_definitions(DSPi PRIVATE
    AUDIO_FREQ_MAX=48000

    # S/PDIF output configuration (pins defined in config.h).  The output
    # rings use no DMA IRQ.
    PICO_AUDIO_SPDIF_PIO=0

    # S/PDIF RX: the only DMA IRQ user (spdif_input_init sets its priority).
    PICO_SPDIF_RX_DMA_IRQ=1

    # RAM-places the TinyUSB rp2040-port IRQ path; both platforms use this USB port.
    PICO_RP2040_USB_FAST_IRQ=1
    # SDK float shims go to .time_critical; used per block by the feedback servo and leveller.
    PICO_FLOAT_IN_RAM=1
    # Double shims too: the leveller's powf reaches double helpers per block.
    PICO_DOUBLE_IN_RAM=1
    # RP2040 mem_ops ROM shims (memset/memcpy wraps) go to RAM; called from core 1
    # and per-block paths. No effect on RP2350 (compiler mem ops; see linker script).
    PICO_MEM_IN_RAM=1
)

target_link_libraries(DSPi
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pwm
    hardware_flash
    hardware_adc
    hardware_i2c
    pico_i2c_slave
    pico_audio_spdif_multi
    pico_audio_i2s_multi
    pico_spdif_rx
    tinyusb_device
)

# Debug UART removed; GPIO 16/17 are freed for the UART control interface.
pico_enable_stdio_uart(DSPi 0)

pico_add_extra_outputs(DSPi)
