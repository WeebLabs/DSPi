# DSPi Firmware Architecture

> This document is a living architecture reference. Sections are updated incrementally as the firmware evolves. See change timestamps for when each section was last verified.

## Table of Contents

1. [System Overview](#system-overview)
2. [Source File Map](#source-file-map)
3. [Build System](#build-system)
4. [Initialization Flow](#initialization-flow)
5. [USB Audio Pipeline](#usb-audio-pipeline)
6. [DSP Processing Engine](#dsp-processing-engine)
7. [Matrix Mixer](#matrix-mixer)
8. [SPDIF Output System](#spdif-output-system)
9. [PDM Subsystem](#pdm-subsystem)
10. [Crossfeed](#crossfeed)
11. [Volume Leveller](#volume-leveller)
12. [Loudness Compensation](#loudness-compensation)
13. [Flash Storage](#flash-storage)
14. [Pin Configuration](#pin-configuration)
15. [Core 1 Architecture](#core-1-architecture)
16. [RP2040 vs RP2350 Comparison](#rp2040-vs-rp2350-comparison)
17. [Per-Channel Input Preamp](#per-channel-input-preamp)
18. [Master Volume](#master-volume)
19. [LG Sound Sync](#lg-sound-sync)
20. [Memory Layout](#memory-layout)
21. [Performance Characteristics](#performance-characteristics)
22. [External Control Interfaces (UART / I2C Target)](#external-control-interfaces-uart--i2c-target)
23. [Control Surfaces (User-Wired Physical Controls)](#control-surfaces-user-wired-physical-controls)
24. [Test Signal Generator](#test-signal-generator)
25. [Spectrum Analyser (RTA / FFT)](#spectrum-analyser-rta--fft)

---

## System Overview
*Last updated: 2026-07-05*

DSPi is a USB Audio Class 1 (UAC1) digital signal processor built on the Raspberry Pi Pico (RP2040) and Pico 2 (RP2350). It receives stereo PCM audio over USB and routes it through a configurable DSP pipeline to multiple output channels via PIO-based S/PDIF and PDM.

- **RP2350:** 9 output channels — 4 S/PDIF stereo pairs (8 channels) + 1 PDM sub
- **RP2040:** 5 output channels — 2 S/PDIF stereo pairs (4 channels) + 1 PDM sub

**Key capabilities:**
- Per-channel input preamp (independent gain per USB input channel)
- Parametric EQ: 11 channels on RP2350 (2 master + 9 outputs), 7 channels on RP2040 (2 master + 5 outputs)
- Matrix mixer with per-output gain, mute, phase invert, and delay
- Master volume: device-side attenuation-only ceiling on all outputs (post-output-gain)
- BS2B crossfeed for headphone listening (per output pair, selectable output-pair mask)
- ISO 226:2003 loudness compensation
- Per-output configurable delay lines
- Runtime pin reconfiguration
- Full parameter persistence to flash
- Vendor control interface (WinUSB/WCID) for real-time parameter control

**Firmware binary:** `default` (XIP) on both platforms; cold code executes from flash through the XIP cache while the hot audio/USB set is RAM-resident (see "Memory Layout").

---

## Source File Map
*Last updated: 2026-09-20 (tube preamp output stage is a damping-factor bell plus shelf; 2026-09-18: tube preamp module added)*

### Core Firmware (`firmware/DSPi/`)

| File | Purpose |
|------|---------|
| `main.c` | Entry point, initialization, main event loop |
| `usb_audio.c` | USB audio input decode (`process_audio_packet`), custom UAC1 TinyUSB class driver (`uac1_driver_*`), output slots, volume, init |
| `usb_audio.h` | USB audio interface API, AudioState struct, extern declarations for shared state |
| `tusb_config.h` | TinyUSB configuration (disables built-in classes; UAC1 handled by custom driver) |
| `audio_pipeline.c` | Input-agnostic DSP pipeline (`process_input_block`): loudness, EQ, leveller, matrix mixer, per-output-pair crossfeed, per-output EQ/gain/delay, output encoding, buffer stats |
| `audio_pipeline.h` | Pipeline entry point, shared buffer declarations (`buf_l`/`buf_r`/`buf_out`), buffer stats API |
| `vendor_commands.c` | Vendor USB control request handlers (GET/SET dispatch, pin/MCK helpers, diagnostics). Public entry `vendor_control_xfer_cb(rhport, stage, req)` is invoked from `usb_audio.c`'s UAC1 class driver. |
| `vendor_commands.h` | Vendor handler declarations, system stats and pin helper prototypes. |
| `dsp_pipeline.c` | Biquad coefficient computation, filter management |
| `dsp_pipeline.h` | Filter storage declarations, delay line API |
| `dsp_process_rp2040.S` | RP2040-only: hand-optimized ARM assembly biquad (per-sample + block-based) |
| `pdm_generator.c` | 2nd-order sigma-delta PDM modulator, Core 1 PDM mode |
| `pdm_generator.h` | PDM API, ring buffer communication |
| `crossfeed.c` | BS2B crossfeed filter (lowpass + allpass for ILD/ITD), per-output-pair dispatch + shared double-buffered coeffs |
| `crossfeed.h` | Crossfeed API, presets, `CrossfeedConfig`/`CrossfeedCoeffs`/`CrossfeedPairState` structs |
| `loudness.c` | ISO 226:2003 loudness curve computation, double-buffered tables |
| `loudness.h` | Loudness API, coefficient structs |
| `leveller.c` | Volume leveller (feedforward RMS compressor) |
| `leveller.h` | Volume leveller API, state/config structs |
| `tube.c` | Tube preamp emulation: biased asymmetric waveshaper, supply sag, optional damping-factor output stage, tube and rectifier tables, shared per-output kernel |
| `tube.h` | Tube preamp API, `TubeConfig`/`TubeCoeffs`/`TubeOutputState` structs, parameter index enum, ranges and defaults |
| `limiter.c` | Output limiter: per-output brickwall lookahead peak limiter, link groups, cross-core meeting, silent engage of the lookahead delay |
| `limiter.h` | Output limiter API, per-output config, parameter indices, lookahead and block constants, pipeline hooks |
| `lg_sound_sync.c` | LG Sound Sync detection state machine + apply path (drives host volume from LG-decoded TV remote) |
| `lg_sound_sync.h` | LG Sound Sync API, status struct, default constant |
| `i2s_input.c` | I2S RX integration: master/slave PIO lifecycle, IRQ-less DMA ring, poll into pipeline |
| `i2s_input.h` | I2S input API (start/stop/resync/poll), state enum |
| `i2s_input.pio` | I2S RX PIO programs (clock-master and wait-driven slave variants) |
| `input_capture_arena.h` | Shared capture-buffer union (SPDIF FIFO / I2S rings / ADAT ring) + ownership claim/release API |
| `audio_clock_div.h` | `audio_base_divider_16_8()`: the one 256 x Fs divider every output clock and the input servo derive from (see "Shared output divider") |
| `soft_vcxo.c` | Soft VCXO: DMA-pulsed pll_sys fbdiv dither giving ppm-level sys_clk trim (the single actuator for input clock tracking) |
| `soft_vcxo.h` | Soft VCXO API (`soft_vcxo_init` / `soft_vcxo_set_ppm` / `soft_vcxo_current_ppm`) |
| `input_servo.c` | Shared clock servo for externally clocked inputs: ppm feed-forward + deadbanded P + centering integrator, actuates the soft VCXO |
| `input_servo.h` | Input servo API (`input_servo_apply` / `input_servo_reset`) |
| `output_latency.c` | Output latency option table, the resolve rule (stream-only options on USB) and the live target / input batch / block cap state (see "Output Latency") |
| `output_latency.h` | Option IDs, flags and the live-state API (`output_latency_apply`) |
| `loop_prof.c` / `loop_prof.h` | Bench-only (`DSPI_LOOP_PROF`) main-loop profiler: named sections (`LOOP_PROF(id)`), background steps and vendor handlers by opcode, each a worst time read through `REQ_GET_STATUS` (64+section, 96+job, 256+opcode). The header's X-macro lists are also parsed by `tools/latency_bench/loop_prof.py` |
| `bg_job.c` / `bg_job.h` | Background jobs: long main-loop work run in budgeted steps between deliveries (`bg_run_one()`), the step budget and the over-budget counter (`REQ_GET_STATUS` 31); see Output Latency "Step budget and background jobs" |
| `clock_diag.c` | Bench clock/audio-path diagnostics: once-per-second `CLKDIAG` serial dump (gated by `DSPI_CLOCK_DIAG`). See "Clock Diagnostics" |
| `clock_diag.h` | Clock diagnostics API (`clock_diag_init` / `clock_diag_sample` / `clock_diag_poll`); no-op inlines when the flag is 0 |
| `flash_storage.c` | Parameter save/load to last 4KB flash sector |
| `flash_storage.h` | Flash storage API |
| `bulk_params.c` | Bulk parameter collect/apply (wire format ↔ live state) |
| `bulk_params.h` | Wire format structs (`WireBulkParams`), buffer size defines |
| `config.h` | Global config, data structures, vendor command IDs, channel defs |
| `usb_descriptors.c` | Hand-rolled UAC1 configuration descriptor as a packed byte array + TinyUSB `tud_descriptor_*_cb()` callbacks |
| `usb_descriptors.h` | Descriptor declarations, UAC1 request opcodes, per-alt EP descriptor pointer externs |
| `dcp_inline.h` | RP2350 DCP (Double Coprocessor) inline assembly wrappers |
| `memmap_dspi_rp2040_xip.ld` | RP2040 XIP linker script (SDK `memmap_default.ld` + hot-object excludes and RAM pull list) |
| `memmap_dspi_rp2350_xip.ld` | RP2350 XIP linker script (same derivation as the RP2040 variant) |

### Build / verification scripts (repo `scripts/`)

| File | Purpose |
|------|---------|
| `scripts/check_ram_placement.py` | Verifies the XIP build: hot symbols resolve to RAM (A), no hot-function branch reaches flash (B), no Core-1 flash literal-pool pointers (B2), sizes vs baseline + `.data` budgets (C). Accepts release and loopback ELFs. |
| `scripts/ram_baseline.json` | Recorded copy_to_ram baseline sizes used by `check_ram_placement.py` Check C. |

### LUFA Compatibility (`firmware/DSPi/lufa/`)

**Not on Phase 1 include path.** Folder retained on disk for Phase 2 reference; the UAC1 descriptor is now hand-rolled in `usb_descriptors.c` against TinyUSB's `AUDIO_*` constants.

### SPDIF Library (`firmware/pico-extras/src/rp2_common/pico_audio_spdif_multi/`)

Multi-instance S/PDIF output library (PIO-based, converted from pico-extras singleton).

---

## Build System
*Last updated: 2026-09-01 (git build stamp added)*

### CMake Configuration

**Build file:** `firmware/DSPi/CMakeLists.txt`

**Binary type:** `default` (XIP) on both platforms via `pico_set_binary_type(DSPi default)`. Cold code runs from flash through the XIP cache; only the hot audio/USB set is RAM-resident, driven by source attributes (`.time_critical`, `__not_in_flash_func`) plus two custom linker scripts `memmap_dspi_rp2040_xip.ld` / `memmap_dspi_rp2350_xip.ld`. See "Memory Layout" for the residency model, the RAM pull list, and the `scripts/check_ram_placement.py` verifier.

**Optimization levels:**
- General code: `-O2`
- DSP-critical files (`dsp_pipeline.c`, `usb_audio.c`, `crossfeed.c`, `loudness.c`, `audio_pipeline.c`, `leveller.c`): `-O3`

**Platform-specific sources:**
- RP2040: Includes `dsp_process_rp2040.S` (hand-coded biquad assembly)
- RP2350: Pure C with DCP inline assembly in `dcp_inline.h`

**USB stack:** `tinyusb_device` (TinyUSB via the Pico SDK). The pico-extras `usb_device` library is no longer linked. The legacy `PICO_USBDEV_*` compile definitions have been removed. The `lufa/` include directory is no longer on the target's include path.

**Key compile definitions:**
- `AUDIO_FREQ_MAX=48000`
- `PICO_AUDIO_SPDIF_PIO=0`
- `PICO_SPDIF_RX_DMA_IRQ=1` (S/PDIF RX is the only DMA IRQ user; the output rings use none)

**XIP residency compile definitions (both platforms unless noted):**
- `PICO_RP2040_USB_FAST_IRQ=1`; routes the TinyUSB rp2040-port USB ISR path to RAM (both platforms use this port)
- `PICO_FLOAT_IN_RAM=1`; float shims in RAM
- `PICO_DOUBLE_IN_RAM=1`; double shims in RAM (the leveller's powf reaches double helpers per block)
- `PICO_MEM_IN_RAM=1`; mem* shims in RAM
- `PICO_DIVIDER_IN_RAM=1`; RP2040 only, divider shims in RAM
- `PICO_INT64_OPS_IN_RAM=1` and `PICO_BITS_IN_RAM=1`; RP2040 only, 64-bit mul and clz shims in RAM

**Vendor commands** were temporarily excluded in Phase 1 and re-added in Phase 2; `vendor_commands.c` / `vendor_commands.h` are now back in `add_executable()`.

**Build commands:**
```bash
cmake --build build-rp2040 --clean-first   # RP2040 build
cmake --build build-rp2350 --clean-first   # RP2350 build
```

### Git Build Stamp
*Last updated: 2026-09-01*

Every build regenerates `<build dir>/DSPi/generated/build_info.h` via the `dspi_build_info` custom target, which runs `scripts/gen_build_info.cmake` at build time (never at configure time, which would go stale). The header defines `BUILD_GIT_DESCRIBE` (`git describe --always --dirty`, clamped from the front to 47 chars so the hash and dirty flag survive) and `BUILD_DATE`. The script writes the header only when the content changes, so dependents recompile only when the hash, dirty state, or date actually moved.

Consumers: `vendor_commands.c` serves the stamp over `REQ_GET_BUILD_INFO` (0x80), and `main.c` embeds it in the UF2's binary info (`bi_program_version_string`) so `picotool info` identifies a build file or a BOOTSEL-mode device. The stamp is provenance for humans; the version reported by `REQ_GET_PLATFORM` remains the only compatibility contract (see `Documentation/Features/firmware_versioning_spec.md`).

### Firmware Version Reporting
*Last updated: 2026-09-12*

`config.h` holds the version as four macros: `FW_VERSION_MAJOR`, `FW_VERSION_MINOR`, `FW_VERSION_PATCH`, and `FW_VERSION_BETA`. `FW_VERSION_PACKED` nibble-packs minor and patch for the legacy bytes of `REQ_GET_PLATFORM` and feeds nothing else.

`FW_VERSION_BETA` is the pre-release ordinal: 0 means a final release, 1 to 255 mean beta N of that patch. Betas of one patch share the patch number and count upward, so the ordinal is the only field distinguishing them, and it must return to 0 in the commit that gets tagged as final. A final release outranks every beta of its patch, so the comparison key a host must use is `(major, minor, patch, beta == 0 ? 256 : beta)`, not a plain four-field compare.

The ordinal ships as byte 6 of the `REQ_GET_PLATFORM` (0x7F) response, which the firmware now offers 7 bytes long. Every transport caps the data stage at the host's `wLength`, so hosts asking for 4 or 6 bytes see exactly the responses they saw before. A reply shorter than 7 bytes comes from firmware predating the ordinal, and every such build was final, so hosts decode the missing byte as 0. Wire layout and the host fallback rules live in `Documentation/Features/firmware_versioning_spec.md` and `Documentation/Features/device_identification_spec.md`.

The bulk parameter header carries `fw_version_major` / `fw_version_minor` separately and does not carry the ordinal.

---

## Initialization Flow
*Last updated: 2026-07-22*

Defined in `main.c`, function `core0_init()`:

1. **GPIO setup** — LED (GPIO 25), status pin (GPIO 23)
2. **Clock configuration** — PLL fixed at 307.2 MHz (VCO 1536 / 5 / 1), no runtime clock switching
   - RP2350: `set_sys_clock_hz(307200000)`, VREG 1.15V
   - RP2040: `set_sys_clock_pll(1536000000, 5, 1)`, VREG 1.15V
   *Last updated: 2026-03-31*
3. **Bus priority** — DMA gets highest system bus priority
4. **USB + SPDIF init** — Must happen BEFORE PDM (SPDIF requires DMA channel 0)
5. **Preset boot load** — `preset_boot_load()` always selects a preset. Reads preset directory, loads appropriate slot based on startup policy (specified default or last active). If the target slot is empty, applies factory defaults while keeping the slot selected. On first boot after upgrade, migrates legacy single-sector data into preset slot 0. A preset is always active — there is no "no preset" state.
   *Last updated: 2026-03-07*
6. **Loudness table computation** — Pre-compute ISO 226 curves for all 61 volume steps
7. **PDM setup** — Configure PIO1 hardware, determine Core 1 mode
8. **Core 1 launch** — `multicore_launch_core1(pdm_core1_entry)`
9. **Soft VCXO init**: `soft_vcxo_init()` claims its DMA channels / DMA timer / PWM pacer and (RP2350) grants the DMA master PLL_SYS access; inert unless `clock_get_hz(clk_sys) == 307.2 MHz`. See "Soft VCXO".

### Main Loop

- Watchdog refresh (8s timeout)
- EQ parameter updates (coefficient recomputation)
- Sample rate change handling (PLL reclocking + filter recalculation)
- Loudness table recomputation (background, double-buffered)
- Crossfeed coefficient updates
- LED heartbeat toggle

---

## USB Audio Pipeline
*Last updated: 2026-03-18*

### USB Stack
*Last updated: 2026-10-17 (RP2350 multichannel alts listed)*

**Library:** TinyUSB (vendored via Pico SDK) with a custom UAC1 class driver (`usb_audio.c`, registered via `usbd_app_driver_get_cb`). TinyUSB's built-in audio class driver is UAC2-only and is bypassed. See "TinyUSB Migration (Phase 1)" for details.

**Error handling:** TinyUSB's USB IRQ handler receives bus-level error interrupts and increments internal counters. In Phase 1, the vendor-command hooks for `REQ_GET_USB_ERROR_STATS` / `REQ_RESET_USB_ERROR_STATS` are unreachable (vendor interface dropped); Phase 2 will re-expose these once the vendor interface is wired into TinyUSB.

**Interfaces (Phase 1):**
1. **Audio Control (AC)** — Interface 0
2. **Audio Streaming (AS)** — Interface 1
   - Alt 0: Zero-bandwidth (idle)
   - Alt 1: 16-bit PCM, 2 channels (44.1/48/96 kHz), wMaxPacketSize=582
   - Alt 2: 24-bit PCM, 2 channels (44.1/48/96 kHz), wMaxPacketSize=582
   - RP2350 only: alts 3/4/5 = 4/6/8 channels 16-bit and alt 6 = 4 channels 24-bit, all 48 kHz. On RP2350 every alt uses wMaxPacketSize=788 (see "Multichannel USB Input").
   - EP OUT 0x01 (isochronous async): Audio data
   - EP IN 0x82 (isochronous feedback): 10.14 fixed-point rate, bRefresh=2 (4 ms)

The vendor interface (formerly interface 2) and its WinUSB/WCID descriptors are removed in Phase 1. They will be reintroduced in Phase 2 using TinyUSB's `CFG_TUD_VENDOR` mechanism and MS OS 2.0 descriptors.

### Microsoft OS 2.0 Descriptors (auto-bind WinUSB)
*Last updated: 2026-04-30*

When the vendor interface is active, DSPi advertises Microsoft OS 2.0 Platform Capability Descriptors so Windows 8.1+ auto-binds `winusb.sys` to the vendor function on first plug-in — no Zadig step required.

**Wire format additions:**
- `device_descriptor.bcdUSB = 0x0210` (was `0x0200`) so Windows queries the BOS descriptor.
- `device_descriptor.bDeviceClass / bDeviceSubClass / bDeviceProtocol = (0xEF, 0x02, 0x01)` (was all zeros) — the **IAD signaling triplet** required by the USB-IF IAD ECN whenever a device uses an Interface Association Descriptor. On Windows, this triplet causes `usbccgp.sys` to spawn a composite-device parent and apply per-function driver binding; without it, Windows inspects interface 0 (Audio Control) and may classify the whole device as Audio, breaking the per-function WinUSB binding our MS OS 2.0 Function Subset Header relies on. Matches every TinyUSB audio + IAD example.
- BOS descriptor (`desc_bos`, 33 bytes total) with one Microsoft OS 2.0 Platform Capability descriptor (28 bytes), built from TinyUSB helpers `TUD_BOS_DESCRIPTOR` + `TUD_BOS_MS_OS_20_DESCRIPTOR`. Returned via `tud_descriptor_bos_cb()` in `usb_descriptors.c`.
- MS OS 2.0 descriptor set (`desc_ms_os_20`, 178 bytes / 0xB2) consisting of:
  - Set Header (10 bytes; `dwWindowsVersion = 0x06030000` = Win 8.1).
  - Configuration Subset Header (8 bytes; configuration 0).
  - Function Subset Header (8 bytes; `bFirstInterface = ITF_NUM_VENDOR = 2` — scopes WinUSB binding to the vendor function only, leaving the audio function on the OS audio class driver).
  - Compatible ID Feature Descriptor (20 bytes; `WINUSB\0\0`).
  - Registry Property Feature Descriptor (132 bytes; `DeviceInterfaceGUIDs` REG_MULTI_SZ = `{9D9B8609-E6D1-4FF0-92AF-403119CB7692}`).

**Vendor request handler:** the existing `tud_vendor_control_xfer_cb` in `vendor_commands.c` has a top-of-SETUP dispatch for `bRequest == MS_VENDOR_CODE (0x01)`:
- IN direction with `wIndex == 7` (per MS OS 2.0 spec — "GET MS OS 2.0 Descriptor Set") → return `desc_ms_os_20`.
- Anything else with `bRequest == 0x01` (OUT direction, IN with `wIndex == 8` "SET_ALT_ENUMERATION", or unknown wIndex) → STALL.

The branch sits before both the IN-direction `vendor_handle_get` dispatcher and the OUT-direction generic SET path, so `bRequest = 0x01` can never reach the legacy 0x42+ application-opcode range or be silently ACK'd. The literal 7 and 8 are MS OS 2.0 spec constants — TinyUSB doesn't expose them as named enums.

**Host integration:** the host application must use the published GUID with `SetupDiGetClassDevs(&guid, …, DIGCF_DEVICEINTERFACE)` to enumerate DSPi on Windows. libusb 1.0 transparently uses the WinUSB backend behind the scenes; apps can also `WinUsb_Initialize` directly. The GUID is product-line-scoped and must not be changed without coordinated host-side updates and (for already-enumerated machines) re-enumeration via Device Manager → Uninstall device.

**No MS OS 1.0 fallback:** Windows < 8.1 (EOL January 2023) sees DSPi as an unrecognised vendor function and still requires manual driver install. We're forward-only on Windows.

### Sample-rate & Bit-depth Switching
*Last updated: 2026-10-17 (alt 3↔6 bit-depth switch). Previously, 2026-07-23 (consumer pools replaced by per-slot rings)*

Any host-driven format change (SET_INTERFACE between AS alts for a bit-depth switch, or SET_CUR on the endpoint sampling_freq control for a rate switch) must land on a muted, drained pipeline. Otherwise old-rate/old-bit-depth audio still queued in the slot rings plays out against the new PIO divider or gets decoded under the wrong bytes-per-frame assumption, producing an audible pitch shift or byte-scramble burst.

**`uac1_apply_alt()` (usb_audio.c):**
- **Idempotent early-return.** `SET_INTERFACE(alt=current)` is common from host driver probes and used to tear down / re-open iso endpoints for no reason. Now skipped.
- **Bit-depth switch (alt 1↔2, or alt 3↔6 on RP2350) is treated the same as a cold start (alt 0→>0).** Both paths engage the mute envelope inline (`preset_mute_counter = PRESET_MUTE_SAMPLES`, `preset_loading = true`) so any packet decoded between the SETUP ack and the main-loop's `complete_pipeline_reset()` is silenced. Both paths also raise `stream_restart_resync_pending`, reset the feedback controller (`fb_ctrl_stream_stop` + `feedback_10_14 = nominal_feedback_10_14`), and clear `sync_started` / `total_samples_produced` so gap detection and the feedback loop resume from a deterministic baseline.
- The pre-existing ring flush on `bit_depth_changed` still runs; combined with the mute, stale packets cannot reach the DSP pipeline in the wrong format.

**SET_CUR sampling_freq validation:** unsupported rates are now stalled at EP0 rather than silently committed. Previously any 24-bit value was accepted — `audio_state.freq` would store garbage that `perform_rate_change()` later coerced to 44100, so a subsequent GET_CUR returned a rate the device never actually applied. The accepted set is {44100, 48000, 96000} to match the Type-I format descriptors on both alts.

**USB rate ownership:** `usb_selected_rate` is the UAC1 playback endpoint's host-selected rate, while `audio_state.freq` is the rate actually applied to the live DSP pipeline. SET_CUR and the fixed-48-kHz multichannel alts always update the USB selection, but arm a live rate change only while USB is the active input; GET_CUR returns the retained USB selection. A later source switch into USB applies that selection, so decorative USB control traffic cannot interrupt S/PDIF or I2S playback and the host does not need to resend its rate when USB becomes active. Two races are closed inside the source-switch handler: `rate_change_pending` is cleared right after `active_input_source` flips (a SET_CUR armed for the old source in the same loop iteration would otherwise fire under the new source; mirrors the clock-mode flip handler's clear), and the switching-to-USB tail re-checks the retained selection against `audio_state.freq` and re-arms if a SET_CUR landed in USB IRQ context between the branch reading the selection and the source flip. Every return to USB goes through this handler, including a factory reset, an empty-slot preset load, and an active-slot preset delete, so each one applies the retained USB rate and restores nominal output dividers. *Last updated: 2026-10-10*

**`perform_rate_change()` (main.c):** bracketed with `prepare_pipeline_reset(PRESET_MUTE_SAMPLES)` / `complete_pipeline_reset()`. Without the bracket the new PIO divider would apply while old-rate audio still sits in each slot ring, playing out at the new bit-clock; audible pitch wobble for ~16 ms. `complete_pipeline_reset()` aborts DMA on every enabled slot, resets each slot ring, and restarts all outputs in sync at the new divider. The I2S `audio_i2s_update_all_frequencies()` call inside `perform_rate_change()` still runs for its own divider+clkdiv_restart pass; the subsequent `complete_pipeline_reset()` re-aborts/re-enables idempotently and costs only microseconds.

**Output dividers at rate changes (`apply_nominal_output_rate()`, main.c):** output dividers are never trimmed. Input clock tracking pulls `sys_clk` in ppm through the soft VCXO, which moves every output together (see "Soft VCXO" and "Clock Servo"). `perform_rate_change()` therefore only programs the nominal divider for the new rate on each set-up S/PDIF slot and MCK, while `audio_i2s_update_all_frequencies()` retunes the I2S slots. Leaving a tracked input (SPDIF, ADAT slave, I2S slave) needs no divider work at all: its stop path calls `input_servo_reset()`, which parks the VCXO at nominal. `input_servo_reset()` must never run on the pipeline-reset path, because it would park a locked VCXO mid-lock. *Last updated: 2026-10-10*

### Clock Diagnostics (bench instrument)
*Last updated: 2026-10-16 (packet version 2: USB feedback controller block and `usbfb:` line)*

**Building it.** Diagnostics are compiled in only with `-DDSPI_CLOCK_DIAG=ON` at configure time (a CMake option, OFF by default, like `DSPI_LOOPBACK`); release builds answer `REQ_GET_CLOCK_DIAG` with zero bytes. The UART and I2C control transports also get zero bytes, because the `ClockDiagPacket` (226 bytes on RP2040, 248 on RP2350; static-asserted at most 256, the length `clkdiag_poll.py` requests) exceeds their response buffer and would otherwise arrive silently truncated; read it over USB with `tools/clkdiag_poll.py`.

`clock_diag.c` is a serial-console instrument for isolating DAC-cutout failure classes while the soft VCXO tracks an input (sys_clk excursion, servo oscillation, transient consumer starvation, TX/main-loop stall, wedged VCXO DMA loop). It exposes the same once-per-second values two ways: the `CLKDIAG` console `printf` dump, and a read-only `REQ_GET_CLOCK_DIAG` (0x82) vendor GET that returns the latched `ClockDiagPacket` snapshot (used when no console backend is enabled; this firmware ships with USB CDC and UART stdio both off, so the printf output is otherwise discarded). Gated entirely by `DSPI_CLOCK_DIAG` (config.h, default 1 for bench; set 0 for release, which compiles everything to no-op inlines and makes `REQ_GET_CLOCK_DIAG` return a zero-length response). `clock_diag_sample()` runs once per main-loop pass (services a non-blocking fc0 frequency counter, tracks the max loop gap, per-slot min consumer fill, and PWM-pacer counter movement); `clock_diag_poll()` prints once per second and resets the windowed aggregates.

The dump is seven-to-nine lines, each prefixed `CLKDIAG ` for grepping:

- `sysclk:` fc0-measured sys_clk `hz`/`min`/`max`, derived `ppm` offset vs the nominal `clock_get_hz(clk_sys)` `[min..max]`, sample count `n`, PLL `lock` bit. (fc0 is otherwise unused in the tree.)
- `vcxo:` last commanded `ppm[min..max]`, `park` state, commanded `fbdiv0` (127/128/129), effective `pulseHz`, PWM `div`/`wrap`, cumulative `set`/`clamp`/`sflip` (sign-flip) counts, live `fbdivLive` (pll_sys `fbdiv_int`), PLL `cs` lock.
- `vcxoloop:` PULSE and PACE DMA channel `busy`/`en`/`tcr` (transfer count remaining, from the debug registers)/PULSE `rd` addr/PACE `ctdreq` (DREQ counter), plus the PWM pacer `pwmctr` and its `moves` count since last dump (loop-alive evidence).
- `servo:` measured input `in` Hz, `ff` feed-forward ppm, total `tot[min..max]` ppm, integrator `int` ppm, `ferr` fill error, `slot` used, `ticks`.
- `slots:` per output slot: type (spd/i2s), current `fill`/`min` since dump, `starv` (per-instance underrun count, now in **frames** of silence exposed to the wire, from the ring's always-on underrun accounting), `dwc` (delta words consumed = TX DMA throughput, derived from the unwrapped ring read pointer; drops to 0 on a stalled channel).
- `pdm:` (when PDM enabled) ring/dma fill, `ringur` (ring-empty silence insertions) / `dmaur` (write-behind-read) underruns.
- `input:` SPDIF RX `st` state, `rate`, `lk`/`ls` lock/loss counts, `par` parity errors, `fifo` fill %, estimator `hzL`/`hzS` (long-window count vs IIR bridge) and which is in `use`, long-window `span`.
- `loop:` worst main-loop `gapmax` since last dump.
- `usbfb:` USB feedback controller: output `fb[min..max]` (samples/frame, windowed), short-window `iir` and `long` rate (0 while the long window is not valid), long-window `span`, `servo` term, filtered fill error `ferr` (buffers), and cumulative `upd`/`merged`/`rej`/`reanch` counts (controller updates, merged-SOF windows, implausible short samples, long-window restarts). It reports the controller output, which is what goes on the wire only while USB is the input source. The host decoder adds ppm against the nearest standard rate and shows `ferr` in frames.

Starvation counters are the authoritative instantaneous-empty signal (H3). With the ring model they count **frames** of silence exposed to the wire (ring underrun accounting is always on, no monitor bool, no IRQ path). All `clock_diag.*`, plus the diagnostic getters in `soft_vcxo.c`/`input_servo.c`/`spdif_input.c`, live in flash and run only from the main loop.

The `REQ_GET_CLOCK_DIAG` (0x82) snapshot is a packed little-endian `ClockDiagPacket` (`clock_diag.h`, version byte = 2; version 2 appended the twelve `fb_*` words, and the decoder still parses version 1) carrying every dump field: ppm values are centi-ppm (ppm times 100), frequencies flagged `_mhz` are milli-Hz, per-slot arrays are sized `NUM_SPDIF_INSTANCES` with a `num_slots` count. `clock_diag_poll()` latches it each second next to the printf; `clock_diag_get_snapshot()` copies it for the handler. Tearing is acceptable (both the poll and the control transfer run on the core 0 main loop). Host decoder: `tools`-adjacent `clkdiag_poll.py` polls it at 1 Hz. *Last updated: 2026-10-16 (version 2).*

### Multichannel USB Input + Per-Input EQ/Metering (RP2350)
*Last updated: 2026-10-17 (alt 6 = 4ch/24-bit; shared 24-bit unpack). Previously, 2026-07-10 (crossfeed moved to per-output-pair stage; no longer bypassed in multichannel)*

**Unified channel model (no "master").** Channels are now `[inputs 0..NUM_INPUT_CHANNELS-1][outputs NUM_INPUT_CHANNELS..]`; `NUM_CHANNELS = NUM_INPUT_CHANNELS + NUM_OUTPUT_CHANNELS` (17 on RP2350, 7 on RP2040). The former "master EQ" was always the EQ on the 2 stereo inputs — it is generalized so **every input channel is a first-class channel with its own 10-band PEQ + peak/clip metering** (no crossover). Output channels keep PEQ + crossover + gain/delay/mute. `CH_OUT_1 = NUM_INPUT_CHANNELS`; the output-slot→channel-index mapping (`CH_OUT_1 + slot`) shifts automatically, so the slot-aligned output path is structurally unchanged. **Input EQ and metering reuse the existing `REQ_SET/GET_EQ_PARAM` (channel = input index) and `REQ_GET_STATUS` (`peaks[]`/`clip_flags`) commands — no new commands.** `clip_flags` is `uint32_t` (17 channels > 16 bits).

**USB formats.** RP2350 advertises AudioStreaming alts **1 = 2ch/16, 2 = 2ch/24** (44.1/48/96 kHz), and **3 = 4ch/16, 4 = 6ch/16, 5 = 8ch/16, 6 = 4ch/24** (all 48 kHz fixed). No 6ch or 8ch 24-bit format exists. Alt 6 is packed 3-byte PCM (`S24_3LE` on ALSA) and needs at most 49 × 12 = 588 B per packet, so it fits the shared 788 B endpoint and ring slots. The host picks the format; the firmware sets the active input count (`usb_input_channels` = 2/4/6/8) and bit depth (`usb_input_bit_depth`, 24 for alts 2 and 6) in `uac1_apply_alt()`, which forces 48 kHz for multichannel alts (SET_CUR rejects non-48k), flushes the ring, and resyncs. Alt 3 ↔ alt 6 changes only the bit depth, which still counts as a format change. RP2040 is stereo-only (2 SPDIF = 4 output channels). The multichannel alt blocks are emitted by a parameterized `AS_MULTICH_ALT(alt,nch,bits)` macro; descriptor offsets/`CONFIG_TOTAL_LEN` (421 on RP2350: FU +6, four 52-byte alt blocks) are arithmetic + `_Static_assert`-verified. `USB_BCD_DEVICE` is 0x0204 so Windows re-reads the changed descriptors. The single Input Terminal/Feature Unit declare the 8-channel superset (`wChannelConfig=0x063F`); iso OUT max-packet `AUDIO_EP_MAX_PKT=788`.

**Decode + pipeline.** `process_audio_packet()` deinterleaves N channels (stride = `channels`) into `input_bufs[c]`. The 24-bit stereo path casts the packet buffer to `uint32_t*` and the compiler may emit `ldrd` (which faults on a non-word-aligned address), so the USB ring slot's `data[]` is `__attribute__((aligned(4)))` (`usb_audio_ring.h`); macOS opens a stereo device at 24-bit, so this path runs even in "2-channel" mode. Bytes per frame is `channels × bit_depth / 8`, and the decoder drops any 24-bit packet whose channel count is not 2 or 4. Both 24-bit formats share the inline `unpack_s24x4()`, which turns three aligned words (12 bytes) into four floats. One call is one 4-channel frame, or two stereo frames. A stereo packet with an odd frame count decodes its last 6 bytes byte-wise, so the decoder never reads past the packet. The decode stays inside the RAM-resident `usb_audio_drain_ring()`; alt 6 costs about the same per frame as alt 3. (inputs 0/1 → buf_l/buf_r, 2-7 → buf_in_ext) with per-input preamp. In `process_input_block()` the **input-EQ pass** runs `dsp_process_channel_block(filters[k], input_bufs[k], …)` for each active input `k`, then meters its post-EQ peak/clip into `global_status.peaks[k]`/`clip_flags`. `n_active_inputs` = `usb_input_channels` for USB else 2. **Crossfeed no longer sits on the input bus; it runs per output pair post-matrix (PASS 4.5), so it is input-count agnostic and never bypassed in multichannel** (see "Crossfeed"). (Loudness no longer sits on the input bus; it runs per output post-gain, mask-selected, in both stereo and multichannel modes; see "Loudness Compensation".) The **volume leveller now runs channel-count-agnostic over the active inputs** (mask-driven; see "Volume Leveller"), so it is no longer bypassed in multichannel. The matrix iterates `n_active_inputs`, so `buf_in_ext` is read only when those inputs are active; inactive input peaks are zeroed. Inter-output sample alignment is preserved bit-for-bit (the leveller delays every active input identically through its per-channel lookahead ring). Default factory routing is stereo pass-through; multichannel routing is configured by the host app.

**Live active-input-count for the host.** The app reads the active count (2/4/6/8) from the `REQ_GET_STATUS` combined packet (a 1-byte `active_input_channels` field, polled with the meters) or the scalar `wValue==23`; the bulk header `num_input_channels` stays the device max (8). Both report sites are **source-aware** via `active_input_channel_count()` (`audio_pipeline.c`), the single source of truth shared with the DSP pipeline's input dimension: USB → `usb_input_channels`, I2S → `i2s_input_channels`, S/PDIF → 2. A `NOTIFY_EVT_INPUT_FORMAT` (0x05) push event fires whenever the active count can change — USB alt change, input-source switch, and live I2S channel-count change (`REQ_SET_I2S_INPUT_CHANNELS` while I2S is active) — so the app relayouts immediately regardless of source.

**Persistence (compat-breaking).** Wire `WIRE_FORMAT_VERSION=16` (direct 8-input matrix/preamp + 17-channel EQ; no tail-append/version gates; 5864 B). Flash `SLOT_DATA_VERSION=21` (direct layout; the slot spans **2 flash sectors** on RP2350; 1 on RP2040). No migration — pre-version data loads factory defaults.

### Notification Endpoint (device→host push)
*Last updated: 2026-10-20 (NOTIFY_EVT_RTA_CONFIG 0x0D added); 2026-09-22 (notify_rebaseline collects in place, scratch copy removed; 2026-09-07: NOTIFY_EVT_CS_AUX 0x0C is now 9 bytes, carrying the binding slot and an 8.8 percent level; 2026-09-05: NOTIFY_EVT_CS_AUX 0x0C added; 2026-07-13: NOTIFY_EVT_ADAT_INPUT_STATE 0x0B added)*

The vendor interface carries one **bulk IN** endpoint (EP 0x83, wMaxPacketSize = 64) for out-of-band device→host notifications. The transport runs two protocol versions in parallel: v1 (8-byte `MASTER_VOLUME` packets, kept for existing host apps) and v2 (generic `PARAM_CHANGED` + discrete events, the primary protocol going forward). `USB_BCD_DEVICE = 0x0201` so Windows re-reads descriptors after the 8→64 byte EP bump.

See `Documentation/Features/notification_protocol_v2_spec.md` for the full protocol specification.

**Why bulk rather than interrupt:** an earlier draft used an interrupt IN endpoint at 4 ms polling. Under heavy EP0 control-transfer traffic (rapid `REQ_SET_MASTER_VOLUME` from a slider drag in the host app) the RP2040/2350 USB controller crashed after ~20–40 s and the device re-enumerated. Switching the EP to bulk IN eliminates the crash: bulk uses opportunistic host scheduling rather than a fixed bInterval poll cadence. The v2 protocol preserves the bulk transport.

**v2 core design (`notify.c/notify.h`):** every parameter is identified by its `offsetof` into `WireBulkParams`. A single event ID (`NOTIFY_EVT_PARAM_CHANGED = 0x02`) carries `(wire_offset, wire_size, source, value)`. Host dispatch is a flat lookup on offset, not a hand-written switch — adding a parameter requires zero wire-format changes.

**Subsystem state:**
- `param_shadow`: mirror of `WireBulkParams` (6,136 B BSS at V32). `notify_param_write` compares writes against it; notifications only fire on real byte-level changes. `notify_rebaseline()` collects straight into it with no scratch copy, which is safe only because every param write runs on the Core 0 main loop *(2026-09-22)*.
- `notify_ring[32]`: single-producer, **multi-consumer** ring of pending events (1920 B BSS). Coalesces PARAM_CHANGED entries on `(event_id, offset, size)`; a swept knob generates one queued entry, not hundreds. Coalescing only mutates entries no active consumer has consumed yet (window starts at the fastest consumer's tail). See "Multi-consumer ring" below.
- `notify_bulk_depth`: nesting counter. While `> 0`, per-field `param_write` calls are suppressed (shadow still updates) and the outermost `notify_end_bulk()` emits a single `BULK_INVALIDATED` event.
- `notify_current_source`: global source tag set by scoped brackets (see below).

**Source tagging:** every notification carries a `ParamSource` byte:

| Value | Source | Set by |
|-------|--------|--------|
| 0 | UNKNOWN | Default |
| 1 | HOST_SET | `vendor_handle_set_data` / `vendor_handle_get` brackets in `vendor_commands.c` |
| 2 | BULK_SET | `bulk_params_apply()` |
| 3 | PRESET | `preset_load()` |
| 4 | FACTORY | `flash_factory_reset()` |
| 5 | GPIO | Future hardware knob/encoder handlers |
| 6 | INTERNAL | Firmware-initiated (clamps, auto-recalc) |
| 7 | UAC1 | UAC1 Feature Unit SET_CUR data-stage handler in `usb_audio.c` (OS volume slider writes via standard audio class control transfers) |

**Emit hookpoints:** `update_master_volume` emits both v1 (`notify_push_master_volume_v1`) and v2 (`notify_param_write`). `update_preamp` emits v2. Direct-write setters in `vendor_commands.c` (delays, gain/mute, loudness, crossfeed, leveller, matrix, pins, I2S, MCK, SPDIF RX pin, channel names) each call `notify_param_write` after the live-state write. Deferred setters (EQ band, input source) emit at apply time in `main.c`. **UAC1 Feature Unit VOLUME SET_CUR** (`usb_audio.c` data-stage handler) also emits a PARAM_CHANGED on `user_volume.user_volume_db` with source `PARAM_SRC_UAC1`, so v2 hosts see OS volume slider movements on the same field they already listen to for `REQ_SET_USER_VOLUME` (tagged `PARAM_SRC_HOST_SET`) and LG Sound Sync writes — `audio_state.volume` is shared across all three controllers and the notify field mirrors it source-agnostically, while the distinct source byte lets hosts attribute the change to the right controller. UAC1 Feature Unit MUTE has no notify (no parallel WireBulkParams field — `user_volume.user_mute` represents the *vendor* mute with different gating semantics, not `audio_state.mute`).

**Bulk operations** (preset load, factory reset, bulk SET): wrapped in `notify_begin_bulk(source)` / `notify_end_bulk()`. Per-field writes don't flood the ring; the host sees one `BULK_INVALIDATED` and reads `REQ_GET_ALL_PARAMS` for the full state. Preset load also emits `NOTIFY_EVT_PRESET_LOADED(slot)` before the bulk opens.

**Discrete event IDs** on this transport: `NOTIFY_EVT_PARAM_CHANGED` (0x02), `NOTIFY_EVT_BULK_INVALIDATED` (0x03), `NOTIFY_EVT_PRESET_LOADED` (0x04), `NOTIFY_EVT_INPUT_FORMAT` (0x05), `NOTIFY_EVT_SIGGEN_STATE` (0x07), `NOTIFY_EVT_ADAT_STATE` (0x08), `NOTIFY_EVT_I2S_SLAVE_STATE` (0x09, 9-byte packet `[ver=2, 0x09, flags=0, seq, state, rate_LE32]`; pushed on every I2S clock-slave lock-state transition), `NOTIFY_EVT_CS_IR_LEARN` (0x0A), `NOTIFY_EVT_ADAT_INPUT_STATE` (0x0B), `NOTIFY_EVT_CS_AUX` (0x0C), and `NOTIFY_EVT_RTA_CONFIG` (0x0D). Siggen announces test-signal-generator start/stop/completion as an 8-byte packet `[ver=2, 0x07, flags=0, seq, state, reason, signal_type, channel]` (state = `SiggenState`, reason = `SIGGEN_STOP_*`, channel = walk channel or 0xFF); pushed from `siggen_service()` in the main loop, never from the render path (see "Test Signal Generator"). CS_IR_LEARN announces IR learn completion as a 12-byte packet `[ver=2, 0x0A, flags=0, seq, state, protocol, 0, 0, code_LE32]` (state = `CS_IR_LEARN_DONE`/`_TIMEOUT`), pushed from the Control Surfaces tick. ADAT_INPUT_STATE (RP2350) announces every ADAT input lock-state transition as a 10-byte packet `[ver=2, 0x0B, flags=0, seq, state, rate_LE32, clock_mode]` (state = `AdatInputState`; rate = 0 unless LOCKED), pushed from the ADAT RX poll (see "ADAT Input"). CS_AUX announces every Control Surfaces auxiliary output change as a 9-byte packet `[ver=2, 0x0C, flags=0, seq, slot, state, level_q8_LE, src]` (`slot` = the binding slot holding the aux component, `level_q8` = 8.8 percent and 0 on a `CS_TYPE_AUX_OUT` slot; both values on every event; `src` = the dispatch's `ParamSource`, so a bound control reads as GPIO and a host write as HOST_SET), pushed from the `REQ_SET_CS_AUX_STATE` / `_LEVEL` handlers (see "Control Surfaces"). RTA_CONFIG announces every applied `REQ_RTA_SET_CONFIG` as a 17-byte packet `[ver=2, 0x0D, flags=0, seq, RtaConfig (12 B, as applied), src]`, pushed from `apply_config()` in `rta.c` on the main loop *(2026-10-20)*. Because the apply runs after the dispatch has closed its source bracket, the SET handler passes `_dispatch_src` into `rta_set_config()`, which stores it beside the staged config (`cfg_next_src`); `src` is therefore HOST_SET on USB and UART / I2C on those transports (see "Spectrum Analyser").

**Drain:** each consumer drains its own tail via `notify_peek_next_for(consumer, ...)` / `notify_commit_pop_for(consumer)`. The USB consumer (`usb_notify_drain` in usb_audio.c) claims EP 0x83 via `usbd_edpt_claim`, formats the next packet into the stable TX buffer, and submits via `usbd_edpt_xfer`; on success `notify_commit_pop_for(USB)` advances the USB tail, and on xfer rejection the entry stays queued for the next tick. The UART consumer drains from `uart_ctrl_poll` (see "Multi-consumer ring").

**Multi-consumer ring:** the ring supports independent consumers, each with its own tail: `NOTIFY_CONSUMER_USB` (always active) and `NOTIFY_CONSUMER_UART` (active while the UART transport is live with `notify_enable` set). `notify_consumer_set_active(c, active)` snaps a newly activated consumer's tail to `head` so it sees no stale backlog. A lagging consumer never stalls the producer or another consumer: if a push would collide with a consumer's tail, that consumer's oldest entry is force-dropped (counted in `notify_consumer_drops[c]`) and the push proceeds; the consumer detects the loss as a seq gap and re-reads `REQ_GET_ALL_PARAMS`. Sequence numbers are stamped at **push** time and stored in the ring entry (not allocated at drain time), so a re-peek or xfer retry does not burn a number; this removed a historical phantom-gap artifact for USB hosts where a rejected transfer had allocated but not delivered a seq. The v1 legacy master-volume entry carries no seq, consumes none, and is delivered to the USB consumer only (every v1 event has a v2 twin, which the UART consumer receives instead). The old displace-oldest-on-full path for `BULK_INVALIDATED` is gone; force-advance guarantees delivery for every event type.

**Initialisation:** `notify_init()` is called from `core0_init()` after `preset_boot_load()` so `bulk_params_collect(&param_shadow)` sees a fully-populated live state. The control-interface init block (UART / I2C) is now **deliberately placed after** `notify_init()` in `core0_init()`, so the UART bring-up's notify-consumer activation is not wiped by the consumer-table reset inside `notify_init()`. The USB reset path (`uac1_driver_reset`) calls `notify_reset_queue()`, which resets **only** the USB consumer's view (drops its backlog) and the source/bulk brackets; the global seq counter and other consumers' backlogs are left intact (resetting seq would fake a wrap-around gap for a mid-stream UART consumer).

**v1 back-compat:** `update_master_volume` still pushes an 8-byte `MASTER_VOLUME` (0x01) event into the ring. Existing v1 host apps that only recognise byte 0 = 0x01 continue to work; v2 hosts receive the parallel PARAM_CHANGED event and dispatch by offset.

### Volume & Mute
*Last updated: 2026-10-04 (audio_set_volume() returns true when applied, restoring the UAC1 volume notify)*

**Volume range:** -60 dB to 0 dB (1 dB resolution, 61 steps). Bottom step is fully silent. USB Audio Class 8.8 fixed-point dB encoding. Q15 lookup table (`db_to_vol[61]`) maps dB index to linear multiplier; index 0 = 0x0000 (silent), index 60 = 0x7FFF (unity).

**User volume — multiple owners, one source of truth:** `audio_state.volume` (and `audio_state.mute`) hold the user-perceived volume/mute. Three controllers can drive these fields, all funneling through the same `apply_vol_index_to_audio()` helper so `vol_mul` and the loudness coefficient pointer always move in lock-step:

1. **UAC1 host slider/mute** (`audio_set_volume()` / UAC1 Feature Unit MUTE). This is the primary controller while USB is the active input source. When `active_input_source != INPUT_SOURCE_USB`, `audio_set_volume()` does not touch `audio_state.volume` at all. It parks the value in a private `uac1_host_volume` with a pending flag and returns false. UAC1 GET_CUR returns the parked value while it is pending, so the host's round-trip still matches what it wrote. An applied UAC1 SET_CUR VOLUME emits a `PARAM_CHANGED` on `user_volume.user_volume_db` with source `PARAM_SRC_UAC1` (distinct from `PARAM_SRC_HOST_SET`, which tags `REQ_SET_USER_VOLUME`). A parked write emits nothing. The data-stage handler gates the notify on `audio_set_volume()` returning true, so the applied path must end in an explicit `return true`. Without it the return value is undefined in C, and GCC at -O2 deleted the notify call entirely (the build has no `-Wreturn-type` to catch it). UAC1 Feature Unit MUTE does NOT emit (no parallel WireBulkParams field; see "Mute" below).
2. **LG Sound Sync** (`lg_sound_sync.c`) — owns `vol_mul` and re-keys the loudness pointer while SPDIF input is active and an LG TV is producing optical TC values.
3. **Vendor channel** (`REQ_SET_USER_VOLUME` / `REQ_SET_USER_MUTE`) — `update_user_volume()` always applies, regardless of input source, so an external control surface or app can drive user volume during non-USB playback. Caller-side conventions (e.g. "Console only writes during non-USB input") are not enforced by the firmware. While LG Sound Sync is locked, its next ~20 ms poll will overwrite the vendor write — that's intentional ownership during LG-driven playback.

**Mute (two flags, different gating):**
- `audio_state.mute` — UAC1 Feature Unit MUTE control. **USB-gated:** the OS mute key has no audible effect when the active input is not USB, so the host can't accidentally silence SPDIF playback. Host volume is parked the same way by `audio_set_volume()` when source != USB.
- `user_mute` — vendor-channel mute (`REQ_SET_USER_MUTE`). **Always honored, no input-source guard** — symmetric with `REQ_SET_USER_VOLUME`'s always-apply contract. An external control surface that mutes via the vendor channel expects audio to actually go silent.

The pipeline ORs them: `muted = (audio_state.mute && host_active) || user_mute`. Either flag forces the per-packet target gain to zero; the apply then ramps to zero over the per-sample envelope used for volume changes (see below), so mute toggles are click-free regardless of which flag triggered them. `REQ_GET_USER_MUTE` returns `user_mute` only — UAC1's mute remains queryable via UAC1 GET_CUR (each surface owns its native interface).

**Invariant:** `audio_state.volume` always equals the volume actually applied to `vol_mul`. The control-surface display (`CS_NOUN_USER_VOLUME`), Console's `REQ_GET_USER_VOLUME`, bulk params, and preset save all read it, so none of them may ever see a value that is not audible. The host's parked value lives only in `uac1_host_volume`. Before 2026-09-28 the parked value was written into `audio_state.volume`, so an OS volume restore on enumeration during SPDIF playback made the LCD and Console show a volume the device was not playing at, and a preset save then persisted it.

**Parked value on switch to USB:** `audio_thaw_host_volume()` runs in the switching-to-USB branch of the input-source handler. It applies the parked host value through `update_user_volume()` only if the pending flag is still set. Any later `update_user_volume()` call (control surface, vendor command, preset load, LG Sound Sync) clears the flag, so the most recent writer wins.

**Persistence:** User volume IS saved per-preset as of `SLOT_DATA_VERSION` = 15 — stored as a single-byte `user_vol_index` (range [0, CENTER_VOLUME_INDEX]) consuming the last V14 padding byte. Restore on preset load funnels through `update_user_volume()` (so `vol_mul`, the loudness coefficient pointer, the LG apply-cache invalidation, and the v2 notify all run via the single helper). Pre-V15 slots leave user volume UNTOUCHED on load — asymmetric vs master volume which falls back to a directory value, but there is no directory-level fallback for user volume; the user wasn't expecting that legacy preset to set their listening level. `user_mute` is NOT persisted (session-only, cleared on factory reset) — matches the existing user-mute design contract.

**Per-sample output volume ramping (click-free transitions):** The composite output gain — host volume × `preset_mute_gain` × master volume — is linearly interpolated within each input packet from the previous packet's ending value (`vol_mul_master_prev`, file-scope state in `audio_pipeline.c`) to the new target. Both Core 0 and Core 1 receive the same `vol_mul_start` / `vol_mul_step` pair via `Core1EqWork` and apply identical per-sample ramps to their respective outputs, preserving inter-slot phase alignment (CLAUDE.md hard rule). When `vol_mul_step == 0` (steady state, no host adjustment in flight) the gain loops fall through to the original constant-gain branches (memset for zero, scalar multiply for non-unity, no-op for unity) — no per-sample overhead. RP2350 carries the ramp in float; RP2040 keeps it as Q15 int32 to avoid float→int conversion in the inner loop. The mechanism also smooths preset-mute transitions and master-volume changes, since they all funnel through the same composite gain. Zero-length packets do not advance `vol_mul_master_prev`, so a stray empty packet between two real packets cannot eat a pending volume delta.

**Scope of click-free guarantee:** This ramp covers the output gain stage only. With loudness compensation enabled, `audio_set_volume()` swaps `current_loudness_coeffs` to a new table entry on every host volume step, and the SVF/biquad filter state continues from the previous coefficients — producing a small frequency-response transient at each step. With loudness disabled (or when the user does not change volume), the path is fully click-free; with loudness enabled, the broadband gain click is gone but a faint per-step zipper artifact can remain on signal-rich content.

### Asynchronous Feedback Endpoint
*Last updated: 2026-10-16 (clean-window gate limits; missed-SOF paragraph scoped to the IIR bridge). Previously 2026-10-15 (long-window rate as the primary Loop A term; dual-slope fill servo; diagnostics; earlier the same day frame-number decimation and merged-SOF normalization)*

The device declares itself as a USB asynchronous sink, meaning it drives the audio clock from its own crystal oscillator rather than locking to the host's SOF timing. The feedback endpoint is re-armed from the `xfer_cb` completion in the custom UAC1 class driver (`uac1_driver_xfer_cb` on EP 0x82 in `usb_audio.c`) with the current `feedback_10_14` value, reporting the actual device sample rate to the host in 10.14 fixed-point format (samples per USB frame).

**Architecture:** Q16.16 dual-loop controller (`usb_feedback_controller.c/h`) with 10.14 wire serialization. All internal math uses Q16.16 fixed-point with rounded updates; only the endpoint-facing value is quantized to 10.14.

- **SOF handler** (`uac1_driver_sof()` in `usb_audio.c`, registered via the class driver's `.sof` pointer): Runs at each USB Start-of-Frame (1 kHz) in USB IRQ context (TinyUSB's DCD dispatches SOF-consumer driver callbacks directly from `dcd_event_handler` without going through the task queue). Reads slot 0's unwrapped ring read pointer (SPDIF or I2S, see "Ring consumed-words" below) and the slot 0 fill snapshot, and passes both with the SOF's 11-bit hardware frame number to `fb_ctrl_sof_update()`. The controller updates once every 4 USB frames, decimating on the frame number rather than on callback count (see "Missed-SOF handling" below).
- **Rate estimator (Loop A):** two terms. The primary term is a long-window count rate: DMA words consumed between two anchors 8 to 16 s apart, divided by the frames between them (see "Long-window rate" below). Until that window has 1 s of span, and after any drop, a short-window IIR bridges. The IIR is first-order with α=1/16 and `round_div_pow2_s32()` (symmetric half-away-from-zero rounding, int64_t-safe), time constant τ≈64ms at the 4ms update rate (bRefresh=2). Its raw rate for a normal 4-frame window is a shift: SPDIF `delta_words << 12` (4 words per frame), I2S `delta_words << 13` (2 words per frame). A longer window is normalized by its true frame count with one rounded division (RAM-resident hardware divider wrapper on RP2040, `udiv` on RP2350). A raw sample more than 50% from nominal (`FB_RATE_SANE_SHIFT`), or one whose word delta would overflow the shift, is discarded; anchors still advance.
- **Fill servo (Loop B):** dual-slope proportional correction on slot 0's ring fill, taken from the coherent `snap_produced_frames` / `snap_consumed_frames` pair and passed in Q16.16 units of 48-frame buffers. The fill error against the 8-buffer target is IIR-smoothed with the same α=1/16. Within ±1 buffer (`FB_FILL_KNEE_Q16`) the slope is `FB_FILL_KP_Q16` = 256, i.e. 1/256 sample/frame per buffer (about 81 ppm per buffer at 48 kHz, a time constant of about 12 s). Beyond the knee `FB_FILL_KP2_Q16` = 3840 is added, so the outer slope is the former single gain of 1/16 sample/frame per buffer (about 0.8 s). The term is clamped to ±0.5 sample/frame. No integrator. The long-window rate carries the static offset, and a sub-ppm rate error leaves well under one frame of standing fill offset.
- **Startup/reset gating:** After any reset, resync, stream activation, or slot-0 output-type switch, the servo is held at zero for 2 controller updates (~8ms). During holdoff, nominal feedback is emitted. On stream deactivation (alt 0), the controller is invalidated and all filter state cleared.
- **Rate change:** `perform_rate_change()` pre-computes `nominal_feedback_10_14 = (freq << 14) / 1000` and calls `reset_usb_feedback_loop()` → `fb_ctrl_reset()`, reseeding the rate estimator at nominal and re-latching the measurement anchors on the next SOF.
- **Flash blackout recovery:** `flash_program_staged()` and `preset_delete()` call `fb_ctrl_reset()` after the ~45ms interrupt blackout, reseeding the controller at nominal.
- **Endpoint serialization:** `fb_ctrl_get_10_14()` converts Q16.16 to 10.14 via rounded shift: `(q16 + 2) >> 2`. Fallback to `nominal_feedback_10_14` if the controller has never been reset.
- **Total clamp:** nominal ±1.0 sample/frame (65536 in Q16.16).

**Key variables:**
| Variable | Type | Location | Description |
|----------|------|----------|-------------|
| `fb_ctrl` | `usb_feedback_ctrl_t` | `main.c` | Controller state (rate estimate, filtered fill error, holdoff, window anchors) |
| `feedback_10_14` | `volatile uint32_t` | `main.c` | Serialized endpoint value (written by SOF handler) |
| `nominal_feedback_10_14` | `volatile uint32_t` | `main.c` | Pre-computed nominal for current rate |

**Ring consumed-words used by feedback (2026-07-23):** with the free-running rings there is no `words_consumed` DMA-IRQ counter. At SOF the feedback loop reads `audio_spdif_ring_consumed_words()` (mirror `audio_i2s_*`), which unwraps the DMA read pointer to a monotonic total DMA words consumed since the last ring reset; this is sample-exact and needs no in-flight-transfer size term. The unwrap must be sampled at least once per ring period (~21 ms), which the main loop's fill checks and the per-SOF read both guarantee.

**Missed-SOF handling (2026-10-15):** if the USB IRQ is held off past 1 ms, the controller hardware merges two SOFs into one callback. The controller previously decimated by counting callbacks, so a merged SOF stretched its "4 ms" window to 5 ms and read the rate 25% high. After the 1/16 IIR the estimate jumped about +0.75 sample/frame, and the host over-delivered one full packet of audio (48 frames at 48 kHz) per missed SOF, which the fill servo then took about 0.8 s to remove. The controller now latches the frame number with the word total at every anchor and divides each window by its true frame span (11-bit, wrap-masked). A late anchor inflates one window and shortens the next by the same number of words, so the error cancels exactly as long as neither window is dropped. That is why the plausibility band is relative and wide. A genuine anchor up to 1 ms late moves one half-window by at most 25% of nominal, and the band is 50% for margin (an SOF can land between the frame-number read and the word read). A fixed ±4 samples/frame band rejected only the short half of a pair at 96 kHz for anchors about 0.2 ms late, leaving roughly 6 to 14 samples uncancelled per event in a host-side simulation. With the 50% band, simulated merged SOFs at 44.1, 48 and 96 kHz on both output types net zero over-delivery wherever the one-update rate spike stays inside the ±1.0 sample/frame total clamp. The spike reaches about 1.4 sample/frame at 96 kHz for a ~1 ms late anchor, so there the clamp clips its positive half and a residual of roughly 2 to 7 frames of under-delivery remains per event (the following window still cancels the unclipped part). Since the long-window rate became the primary Loop A term (next paragraph), this mechanism reaches the feedback only while the IIR bridges, that is the first second after a reset or after a long-window drop. With the long rate in use a merged SOF only fails the clean-window gate and moves nothing. During a bridge the residual sits inside the fill servo's inner slope, so it clears with the 12 s time constant rather than in a few hundred milliseconds. The baseline anchor is now latched on the first SOF after a reset instead of the fourth. HW-untested.

**Long-window rate and dual-slope servo (2026-10-15).** Before this change the feedback value was the 64 ms IIR plus a fill servo of 1/16 sample/frame per buffer. That fed every packet-size and processing-phase wobble in the fill reading straight back to the host as feedback changes. The S/PDIF input servo, by contrast, uses an 8 to 16 s block count and a fill gain of a few ppm per buffer. USB now follows the same shape.

- *Anchors.* `frame_total` extends the 11-bit frame number by summing each update's elapsed frames. It restarts at 0 on the baseline call after every reset. Two `(frame_total, words)` anchors rotate every `FB_LONG_HALF_FRAMES` = 8000 frames, so the span runs 8 to 16 s. The rate is `words / (span * words_per_frame)`, done in 32 bits as a quotient plus a rounded remainder division. That needs `span * words_per_frame` below 65536. Rotation waits for a clean window, so a run of unclean windows near the 8000-frame threshold can push an S/PDIF span (4 words per frame) past 16383 frames. The window then rotates early instead of dropping, and is discarded only after a genuine gap. The baseline call after a reset also clears the window, because an SOF ISR landing inside `fb_ctrl_reset()` could otherwise seed it from the previous epoch.
- *Clean windows only.* An anchor endpoint taken while the SOF ISR was late would bias the whole 8 to 16 s span. A 1 ms late anchor is 125 ppm over 8 s, and with the gentle inner slope that would hold about 1.5 buffers of standing offset. So the long window seeds, rotates and samples only on a window that is exactly 4 frames and whose short raw rate is within 0.5 sample/frame (`FB_LONG_ANCHOR_TOL_Q16`) of the IIR. That passes ordinary jitter and word quantization (1/8 sample/frame per word for I2S). A late endpoint makes its own window long by the same amount, so it fails the gate. Up to about 2 samples of undetected lateness (42 µs at 48 kHz) remains, which is at most about 5 ppm over 8 s and 42 ppm at the 1 s minimum span, decaying as the span grows. The gate sees only a change in lateness between consecutive anchors, never the absolute lateness. A sustained plateau (every SOF late by the same amount for many updates) passes it, and an anchor taken inside the plateau biases the long rate by plateau divided by span until that anchor ages out: 0.5 ms over 8 s is 62 ppm (reproduced in a host harness). Realistic plateaus are tens of microseconds, for example packet processing landing ahead of the SOF handler in the same USB IRQ, and give a few ppm, which the fill servo absorbs. Absolute lateness is not observable from the frame number and word count alone.
- *Drops.* A long rate more than `nominal >> 7` (0.78%) from nominal, a quotient that would overflow, or a span too large for the 32-bit division (a gap with no reset) discards the anchors. The IIR then bridges until 1 s of fresh span. Resets (rate change, flash write, output switch, stream start) also clear the window, so after a preset save the IIR carries the rate for the first second, as it always did.
- *Servo split.* With the rate accurate to about 1 ppm, the fill loop only has to absorb real displacements. Fill noise after the 64 ms IIR is a frame or two, far inside the 1-buffer knee, so it moves the feedback by a few ppm instead of tens. A real displacement, such as dropped host packets, reaches the outer slope and recovers at the old speed.

Host-side closed-loop simulation results follow. The model has the host sizing packets from the 10.14 feedback with a 4 to 8 ms read delay and optional smoothing of 0 to 2 s, 0 to 0.5 ms packet-processing jitter, SOF ISR jitter and periodic merged SOFs.
- Feedback standard deviation fell from about 62 ppm (35 ppm with the long rate but the old servo) to about 2 ppm at 44.1, 48 and 96 kHz.
- The live ring fill statistics were unchanged (standard deviation 13.8 frames at 48 kHz). That figure is the 1 ms packet sawtooth sampled at a random phase (48/√12), which is also what the Console's live fill readout shows on any input.
- Recovery from two dropped host packets (96 frames at 48 kHz) to within 24 frames of target took about 2 to 4.5 s depending on the host model, against 0.7 to 1.7 s before. A knee at ±2 buffers made that 10 s; a knee at ±0.5 buffer recovered in about 1.2 s but let processing jitter of several ms through to the feedback.

The model's host honours feedback exactly, on average. It therefore cannot reproduce wander caused by host behaviour, and that is what the `usbfb:` CLKDIAG line is for. HW-untested.

**IRQ safety:** The SOF handler runs inside `isr_usbctrl`. Output DMA raises no IRQ, so it cannot preempt the SOF read; the read-pointer unwrap is done under a brief IRQ-off section inside the getter. The DMA/USB IRQ priority pinning is retained for the S/PDIF RX path (the only remaining DMA IRQ user).

### USB Audio Decoupling (SPSC Ring Buffer)
*Last updated: 2026-04-18*

The DSP pipeline is decoupled from USB audio transfer completion via a lock-free SPSC ring buffer (`usb_audio_ring.h`). The UAC1 class driver's `xfer_cb` pushes raw packets into the ring; the main loop drains the ring and runs the full DSP pipeline. This prevents the USB stack from being blocked for hundreds of microseconds per packet and eliminates ISR-context spinlock contention.

**Context change vs. pico-extras:** under pico-extras, `_as_audio_packet` ran in USB IRQ context. Under TinyUSB, `DCD_EVENT_XFER_COMPLETE` is enqueued in the USB IRQ and dispatched to `uac1_driver_xfer_cb` from `tud_task()` (main-loop context). SOF still runs in IRQ. The ring itself is unchanged — the producer moved from IRQ to task, the consumer remained in the main loop.

**Ring buffer:** 4 fixed-size slots × 578 bytes (576 payload + 2 length). ~2.3KB BSS. Placed in RAM (`__not_in_flash`) for flash-operation safety. Peek/consume pattern (zero-copy consumer).

**Memory barriers:** `__dmb()` at publish/acquire points. Redundant on RP2040 (Cortex-M0+ in-order single-bus) but required on RP2350 (Cortex-M33 write buffer).

**Gap detection:** USB packet arrival gap measurement runs in `uac1_driver_xfer_cb` (task context under TinyUSB) using file-scope `audio_ring_last_push_us`, reset on stream lifecycle transitions in `uac1_apply_alt()` and `usb_audio_flush_ring()`.

**Ring overruns:** `audio_ring.overrun_count`, queryable via `REQ_GET_STATUS` wValue=22. wValue=7 (`spdif_overruns`, producer-pool take failures) is retired and always reads 0. *Last updated: 2026-10-12*

**Deferred flash SET commands:** `REQ_PRESET_SET_NAME`, `REQ_PRESET_SET_STARTUP`, `REQ_SET_OUTPUT_CONFIG_MODE` (and the action-style `REQ_SAVE_OUTPUT_CONFIG`) use separate pending flags per command type. Main loop copies payload under brief interrupt-off (~1µs) to prevent ISR/thread race, then drains ring and executes the flash write. GET-style flash commands (SAVE/LOAD/DELETE) remain synchronous in the vendor handler with real result codes.

### Packet Flow
*Last updated: 2026-04-18*

`uac1_driver_xfer_cb(EP 0x01)` → `usb_audio_ring_push()` → (main loop) → `usb_audio_drain_ring()` → `process_audio_packet(data, len)` [usb_audio.c] → `process_input_block(sample_count)` [audio_pipeline.c]

1. **Ring push (task context)** — Copy raw packet into SPSC ring, detect arrival gaps
2. **Ring drain (main loop)** — Peek/process/consume loop, highest priority in main loop
3. **USB decode (`process_audio_packet` in `usb_audio.c`)** — Gap detection, sync tracking, USB byte decode (16/24-bit) with per-channel preamp into `buf_l[]`/`buf_r[]`
4. **DSP pipeline (`process_input_block` in `audio_pipeline.c`)**; input-agnostic: buffer acquisition, preset mute envelope, EQ, leveller, matrix mixer, per-output-pair crossfeed (PASS 4.5), per-output EQ/gain/loudness/limiter/delay, output encoding, buffer return, CPU metering
5. **Ring write**: the blocking give hands each finished producer block to the writer, which encodes it directly into the slot ring for the free-running DMA

The `process_input_block()` function reads from `buf_l[]`/`buf_r[]` arrays (extern, defined in `audio_pipeline.c`, filled by the input decode stage). This separation enables future alternative input sources (S/PDIF, I2S) to fill the same buffers and call `process_input_block()` directly. Buffer statistics helpers (`get_slot_consumer_fill()`, `get_slot_consumer_stats()`, `reset_buffer_watermarks()`) also live in `audio_pipeline.c`.

### RP2350 Float Pipeline
*Last updated: 2026-10-10 (delay row: ring size and cap; 2026-09-22: output limiter row; 2026-08-01: upmixer centre engine gains an OFF mode)*

All processing in IEEE 754 single-precision float. Hybrid SVF/biquad EQ filtering (SVF for bands below Fs/7.5, TDF2 biquad above).

| Stage | Description |
|-------|-------------|
| Input conversion | USB int16/24-bit or SPDIF RX 24-bit → float full-scale, per-channel preamp gain (`global_preamp_linear[ch]`) |
| Per-Input EQ + metering | Per active input: block-based `dsp_process_channel_block()`, 10 bands, hybrid SVF/biquad; then peak/clip into `peaks[k]`/`clip_flags` |
| Volume Leveller | Upward RMS compressor over active inputs (2 to 8), mask-driven detector/apply link, smoothed peak limiter on the boost, 5 ms per-channel lookahead (float throughout) |
| Stereo Upmixer | Stereo input only: derives Centre/Ls/Rs into matrix source rows 2..4 and removes centre from L/R (nothing removed with the centre engine OFF, leaving L/R bit-exact); raises the matrix source count to 3 or 5. Parks in multichannel modes. Zero-latency steering; deliberate identical-per-row surround Haas delay (see "Stereo Upmixer") |
| Matrix mixing | Block-based: 2 inputs × 9 outputs (8 inputs in 8-channel USB mode, or 3/5 sources with the upmixer active) with gain/phase (loudness now per-output post-gain; leveller still runs) |
| Crossfeed | BS2B lowpass + allpass (ILD + ITD) per output pair, post-matrix (PASS 4.5), pre-output-EQ; `output_pair_mask`-selected (up to 4 pairs); works in every input mode |
| Output EQ | Block-based, 10 bands per output (Core 0: outputs 0-1, Core 1: outputs 2-7) |
| Output gain | Per-output gain × host volume × master volume |
| Loudness | Per masked output, post-gain: 2 SVF shelf filters, volume-keyed; `loudness_output_mask` selects outputs; skipped-and-cleared when masked off / muted / RAW test signal |
| Output limiter | Per output, post-loudness, pre-delay: brickwall lookahead peak limiter (16-sample blocks, 32-sample lookahead, float). The lookahead delay is on every output while any limiter is enabled and absent otherwise (see "Output Limiter") |
| Delay | Float circular buffers, 2048-sample ring, 2047 samples max (42 ms at 48kHz) |
| S24 finalization | Mode is snapshotted once per packet from `adat_output_is_active()` and shared with Core 1 via `core1_eq_work.finalize_s24`. ADAT active: after the last float consumer, each output row of `buf_out` (rows 0-7) is converted to a clamped 24-bit sample **in place** via `output_block_to_s24_inplace()` (`output_s24.h`, `out_s24_t` = `may_alias` int32), once per sample per channel, even when a slot pool is starved, so ADAT always sees converted data. ADAT inactive: the staging pass is skipped and each slot pair uses the fused `output_pair_convert_interleave()` (rows stay float; no second memory pass). Slot bytes are bit-identical in both modes. Row 8 (PDM sub) stays float always. |
| SPDIF output | ADAT active: pure integer copy of the finalized S24 rows into the 4 stereo slot pairs (`output_pair_interleave_s24()`). ADAT inactive: fused float->int24 convert+interleave per pair |
| ADAT output | Encodes the same finalized S24 rows 0-7 directly; the push is gated on the same per-packet snapshot (see "ADAT Bulk Output") |
| PDM output | Float → Q28 for sigma-delta modulation |

### RP2040 Fixed-Point Pipeline
*Last updated: 2026-10-10 (delay row: 1024-sample ring, cap is ring size minus one; 2026-09-22: output limiter row; 2026-07-10: crossfeed now per-output-pair, post-matrix)*

Block-based two-phase architecture with dual-core EQ processing, all in Q28 fixed-point (28 fractional bits). 2 S/PDIF stereo pairs + 1 PDM sub (5 output channels).

**Phase 1 (Core 0, block-based where possible):**

| Stage | Description |
|-------|-------------|
| Input conversion | USB int16 → Q28 (`<< 14`), USB/SPDIF RX 24-bit → Q28 (`sample << 6`), per-channel preamp via `fast_mul_q28()` (`global_preamp_mul[ch]`) — block loop to `buf_l[192]`, `buf_r[192]` |
| Per-Input EQ + metering | Per active input: **block-based** `dsp_process_channel_block()`, 10 bands; then peak/clip into `peaks[k]`/`clip_flags` |
| Volume Leveller | Upward RMS compressor on the 2 inputs, mask-driven detector/apply link, smoothed peak limiter on the boost (Q28 gains), 5 ms per-channel lookahead (Q28 envelope + float gain) |
| Matrix mixing | Q15 gains via `fast_mul_q15()` (16-bit partial products), 2 inputs × 5 outputs → `buf_out[5][192]` |
| Crossfeed | BS2B block via `fast_mul_q28()` (Q28 coefficients) per output pair, post-matrix (PASS 4.5), pre-output-EQ; `output_pair_mask`-selected (up to 2 pairs) |

**Phase 2 (per-output block, dual-core or single-core):**

| Stage | Description |
|-------|-------------|
| Output EQ | **Block-based** `dsp_process_channel_block()`, 10 bands per output |
| Output gain + volume | Combined Q15 multiply via `fast_mul_q15()` (output gain × host volume × master volume) |
| Loudness | Per masked output, post-gain: 2 Q28 biquads via `fast_mul_q28()`, volume-keyed; `loudness_output_mask` selects outputs; skipped-and-cleared when masked off / muted / RAW test signal |
| Output limiter | Per output, post-loudness, pre-delay: same algorithm as RP2350 with Q30 gains, one hardware divide per 16-sample block and a two-multiply Q15 gain apply (see "Output Limiter") |
| Delay | int32 circular buffers, 1024-sample ring, 1023 samples max (21 ms at 48kHz) |
| SPDIF output | Q28 → int24 (`>> 6` with rounding), 2 stereo pairs |
| PDM output | Q28 direct to sigma-delta modulator (single-core fallback only) |

**Dual-core mode:** Core 0 handles input pipeline + matrix mix + SPDIF pair 1 (outputs 0-1), Core 1 handles SPDIF pair 2 (outputs 2-3) — both cores process per-output EQ, gain, delay, and S/PDIF conversion in parallel. PDM sub (output 4) runs on Core 1 in PDM mode; PDM and EQ worker outputs (2-3) are mutually exclusive.

**Performance advantage of block-based processing:** Biquad coefficients are loaded once per biquad per block instead of once per sample. For a 2-band output channel with 192-sample blocks, this reduces coefficient loads from 384 to 2. On RP2350 adjacent second-order biquad sections are additionally fused into one buffer sweep; see "Fused two-section cascade kernels" under Hybrid SVF/Biquad Filtering.

---

## USB Audio Loopback Capture (DSPI_LOOPBACK)
*Last updated: 2026-08-02 (capture glitch counters on REQ_GET_STATUS 24/25)*

A **debug/verification-only** feature, compiled in only when the `DSPI_LOOPBACK`
build flag is defined. It folds the standalone `~/USBrx` S/PDIF-to-USB recorder's
capability into DSPi itself: a USB audio **capture** (recording) endpoint that
streams **output slot 0** back to the host, so the loopback rig no longer needs a
second board or an S/PDIF cable. Excluded entirely from release builds.

### Build flag
`option(DSPI_LOOPBACK ... OFF)` in `firmware/DSPi/CMakeLists.txt`. When ON it adds
`loopback.c` to the target, marks it `-O3`, and defines `DSPI_LOOPBACK`. All
firmware changes are `#ifdef DSPI_LOOPBACK`-gated, so the normal `build-rp2040` /
`build-rp2350` outputs are byte-for-byte unchanged. Use **dedicated** build dirs:

```
cmake -S firmware -B build-rp2040-loopback -DPICO_PLATFORM=rp2040 -DPICO_BOARD=pico  -DDSPI_LOOPBACK=ON
cmake -S firmware -B build-rp2350-loopback -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 -DDSPI_LOOPBACK=ON
```

### USB topology
A **second, self-contained UAC1 audio function** is appended after the vendor
interface (existing interfaces 0/1/2 are unchanged):

- IAD grouping **interface 3** (capture AudioControl) + **interface 4** (capture
  AudioStreaming). `ITF_NUM_TOTAL` becomes 5.
- Input terminal (internal, ID 4) → output terminal (USB streaming, ID 5).
- AS alt 0 zero-bandwidth, alt 1 with one **isochronous async IN endpoint `0x81`**.
- Format: 2-channel, 24-bit PCM, discrete rates 44.1/48 kHz (DSPi's operating
  range), `wMaxPacketSize` = 312 B (`LOOPBACK_EP_IN_SIZE`, 52 frames).
- The MS OS 2.0 / WinUSB descriptors are untouched — their Function Subset still
  scopes WinUSB to the vendor interface (2).

### Full-speed isochronous bandwidth budget
The host reserves FS isochronous bandwidth from each endpoint's declared
`wMaxPacketSize`. The input OUT EP declares the 8-channel max
(`AUDIO_EP_MAX_PKT` = 788 B on RP2350) for **every** input alt, so
`input (788) + capture-IN + feedback (3)` must stay under the ~1150 B/frame FS
periodic ceiling. The capture EP is therefore sized at **52 frames (312 B)** —
the servo's hard max is 50 frames/packet at 48 kHz — not 64 (384 B): at 384 the
total was 1175 B and the host dropped output frames whenever the capture stream
was active (periodic ~100-200 ms drop-outs); at 312 the total is 1103 B and fits
for every input alt (stereo through 8-channel). **Do not** instead shrink the
input OUT EP's per-alt `wMaxPacketSize` to save bandwidth: the device allocates
and queues that endpoint at `AUDIO_EP_MAX_PKT` (`usbd_edpt_iso_alloc` /
`usbd_edpt_xfer`), so a smaller descriptor value makes `iso_activate`'s maxpacket
disagree with the queued length and desyncs iso reception (audible crunch). The
capture EP is safe to shrink because it is the loopback driver's own endpoint —
its `wMaxPacketSize`, `iso_alloc`, and `g_pkt` all derive from
`LOOPBACK_EP_IN_SIZE`.

### Data path
```
slot-0 final 24-bit samples (audio_pipeline.c slot_stage[0], admitted blocks only)
  -> loopback_push_slot0() -> SPSC ring (1024 frames, .bss, ~8 KB)
      -> rate-matching servo (fill_audio_packet, per USB frame, in xfer_cb)
          -> iso async IN EP 0x81 -> USB host
```
- **Tap:** a single call before slot 0's buffer is handed to the output DMA, in
  `process_input_block()`. Since 2026-10-11 it runs only for blocks the output
  ring admission accepted, so the capture holds exactly what slot 0 plays. It reads the already-finalized
  `audio_buf[0]->buffer->bytes` (24-bit sign-extended `int32` interleaved L/R) for
  every pipeline variant (RP2350 dual/single-core, RP2040 dual/single-core),
  including the silence branch. **Read-only** — it copies out and never writes
  back, so it cannot perturb inter-slot output alignment.
- **Servo:** the capture IN endpoint is asynchronous to the host SOF clock in
  every input mode (USB input is feedback-master on DSPi's crystal; S/PDIF/I2S
  input runs on the external clock), so each USB frame sends a feed-forward count
  (`audio_state.freq / 1000` stereo frames) plus a clamped proportional
  correction toward `TARGET_FILL_FRAMES` = 256 (~5.3 ms), with a fractional-sample
  accumulator. Primes to target before streaming; underrun emits silence and
  re-primes. Ported from USBrx, reading the internal ring instead of a S/PDIF FIFO.

### Glitch counters (`REQ_GET_STATUS` wValue 24/25)
The capture is asynchronous to the host, so it can lose or insert frames in two
ways: the ring **overflows** when the host is not draining (`loopback_push_slot0`
drops the rest of the block), or it **underruns** when the ring runs dry
(`fill_audio_packet` emits a silence packet and re-primes). Either splices the
captured stream, and a spliced stream fails a strict comparison (bit-exact
residual, inter-channel lag) exactly as a real DSP fault would.

Two free-running counters make the two distinguishable:

| wValue | Returns | Increments |
|---|---|---|
| 24 | `loopback_get_overflow_count()` | once per frame dropped with the ring full |
| 25 | `loopback_get_underrun_count()` | once per underrun episode (the following packets take the prime branch, so an episode counts once) |

Both are `#ifdef DSPI_LOOPBACK`, so indices 24/25 STALL on a release build and
the host treats them as optional. The harness samples them either side of a
capture (`_clean_capture` in `tools/dspi_test/tests/audio_loopback.py`): a
measurement whose counters moved is retried once, and a persistent glitch is
reported as a transport fault instead of being asserted on. With the counters
absent the harness degrades to its previous behaviour.

### Driver registration
The capture driver (`loopback_uac1_driver`, in `loopback.c`) is registered
alongside the playback driver: `usbd_app_driver_get_cb()` returns a 2-element
array under `DSPI_LOOPBACK`. Because both audio-control interfaces match
`class==AUDIO && subclass==CONTROL && alt==0`, the playback driver's `open()` is
scoped to `bInterfaceNumber == ITF_NUM_AUDIO_CONTROL` (gated) and the capture
driver to `ITF_NUM_LOOPBACK_AC`, so neither hijacks the other's function.

### Host side (deferred)
The capture appears on the **DSPi composite device** (DSPi's VID/PID), not as a
separate "USBrx" device — on macOS, one Core Audio device named "DSPi" exposes
both output and input channels. The loopback harness
(`tools/dspi_test/audio.py`, `USBRX_IN_NAME`) must point its input at "DSPi"; that
host-side change is **not yet made**.

### Files
`loopback.c` / `loopback.h` (ring + driver + servo), `usb_descriptors.c` / `.h`
(gated capture descriptor block + interface/length macros), `usb_audio.c` (driver
registration + interface guard), `audio_pipeline.c` (slot-0 tap), `CMakeLists.txt`
(flag).

### Memory / platform note
The capture ring + buffers add **~8.7 KB BSS** on both platforms (measured: RP2040
BSS 129436 → 138120; RP2350 237404 → 246092). Under the XIP build both platforms
have ample RAM headroom for this BSS (RP2040 free RAM ~91 KB, RP2350 ~192 KB before
the loopback additions), so the RP2040 RAM-tightness noted under the old
`copy_to_ram` image no longer applies. Behavior is identical on both platforms. The
loopback ELFs are also accepted by `scripts/check_ram_placement.py`.

---

## DSP Processing Engine
*Last updated: 2026-06-17*

### Channel Layout
*Last updated: 2026-06-25*

Channels are **inputs first, then outputs** (no "master"): indices `0..NUM_INPUT_CHANNELS-1`
are input channels (each PEQ + metering, no crossover); `CH_OUT_1 = NUM_INPUT_CHANNELS` begins
the output channels (PEQ + crossover + gain/delay/mute). Inputs 0/1 are the stereo bus used by
every source; inputs ≥2 carry audio only in a multichannel USB alt.

**RP2350 (17 channels = 8 inputs + 9 outputs):**

| Channel | Index | Description |
|---------|-------|-------------|
| Input 0 | 0 | USB L (stereo bus) |
| Input 1 | 1 | USB R (stereo bus) |
| Input 2..7 | 2..7 | Multichannel USB inputs (FC/LFE/BL/BR/SL/SR) |
| CH_OUT_1 | 8 | S/PDIF 1 Left |
| CH_OUT_2..CH_OUT_8 | 9..15 | S/PDIF 1R … S/PDIF 4R |
| CH_OUT_9_PDM | 16 | PDM Subwoofer |

**RP2040 (7 channels = 2 inputs + 5 outputs):**

| Channel | Index | Description |
|---------|-------|-------------|
| Input 0 | 0 | USB L |
| Input 1 | 1 | USB R |
| CH_OUT_1 | 2 | S/PDIF 1 Left |
| CH_OUT_2..CH_OUT_4 | 3..5 | S/PDIF 1R, S/PDIF 2 L/R |
| CH_OUT_5_PDM | 6 | PDM Subwoofer |

### Biquad Filter
*Last updated: 2026-08-04 (first-order low/high pass, types 12-13)*

**Types:** Flat (bypass), Peaking, Low Shelf, High Shelf, Low Pass, High Pass, Notch, All-Pass (2nd-order RBJ), First-Order All-Pass (`FILTER_ALLPASS1`), First-Order Low Shelf (`FILTER_LOWSHELF1`), First-Order High Shelf (`FILTER_HIGHSHELF1`), Linkwitz Transform (`FILTER_LINKWITZ_TRANSFORM`), First-Order Low Pass (`FILTER_LOWPASS1`), First-Order High Pass (`FILTER_HIGHPASS1`)

**Coefficient computation:** RBJ Audio-EQ-Cookbook formulas for biquad path, Cytomic SVF equations for SVF path (RP2350 only), both in `dsp_compute_coefficients()`

**First-order types (`FILTER_ALLPASS1`, added 2026-06-17; `FILTER_LOWSHELF1` / `FILTER_HIGHSHELF1`, added 2026-06-20; `FILTER_LOWPASS1` / `FILTER_HIGHPASS1`, added 2026-07-27):** Genuine 1st-order sections. The all-pass has flat magnitude with phase 0° → -180° (-90° at the corner), single parameter `freq` (`Q`/`gain_db` unused). The shelves are gentle 6 dB/oct shelves (monotonic, no `Q`); the first-order shelf prewarps `g` by `A` (not `sqrt(A)` like the 2nd-order shelves). The low/high pass are single-pole 6 dB/oct rolloffs, -3 dB at the corner with no resonance, single parameter `freq` (`Q`/`gain_db` unused); on the SVF path the low pass takes the one-pole output directly and the high pass takes `in - lp`, and on the biquad path both are exact-bilinear degenerate sections (`b2 = a2 = 0`, and `b0 = b1` for the low pass, which its inner loop exploits). On RP2350 these follow the **same hybrid rule as every other type** (changed 2026-06-20): a one-pole TPT SVF below `Fs/7.5` (`bq->svf_first_order`), a degenerate TDF2 biquad (`b2 = a2 = 0`) above. The one-pole SVF folds the `1/(1+g)` reciprocal into a coefficient so its inner loop is multiply-only. On RP2040 (no SVF) they always run as a Q28 degenerate biquad through the existing assembly kernel. Both realizations produce the identical RBJ-cookbook response, verified across the `Fs/7.5` boundary by `tools/filter_tester`.

**Linkwitz Transform (`FILTER_LINKWITZ_TRANSFORM` = 11, added 2026-07-12):** A pole/zero bass-extension biquad that replaces a driver's measured 2nd-order rolloff (`f0`, `Q0`) with a target alignment (`fp`, `Qp`): `H(s) = (s^2 + s*w0/Q0 + w0^2) / (s^2 + s*wp/Qp + wp^2)`. Unlike every other PEQ type it needs **four** parameters, so it borrows the wire packet unconventionally: `EqParamPacket.freq` = `f0`, `.Q` = `Q0`, `.gain_db` carries `fp` in Hz (`gain_db` is otherwise unused, and the front-panel gain noun is a no-op on LT bands), and the fourth parameter `Qp` rides a parallel array `peq_qp_x512[NUM_CHANNELS][MAX_BANDS]` (`dsp_pipeline.c`) as a `uint16` LE `Q*512` (`0` = the 0.707 default). Clamps: `f0` and `fp` to `[10, 0.15*Fs]` (tighter than the generic 0.45*Fs PEQ clamp; bounds every normalized coefficient inside the RP2040 Q28 range), `Q0` and `Qp` to `[0.1, 20]`; `fp <= 0` (or `freq <= 0`) is treated as flat. On RP2350 it runs on the hybrid path (see below); on RP2040 it is the same exact-bilinear TDF2 biquad in Q28 with both corners independently prewarped. The sidecar `Qp` is carried through the wire (`WireBandParams.reserved[2]`), flash (`PresetSlot.peq_qp_x512` tail-append), the `REQ_SET_EQ_PARAM` optional 18-byte payload, and `REQ_GET_EQ_PARAM` param code 5; these bumped `WIRE_FORMAT_VERSION` to 22 and `SLOT_DATA_VERSION` to 30 (V21..V29 slots still load, `qp` defaulting to 0 = 0.707). Full host-facing spec: [`Documentation/Features/peq_filters.md`](Features/peq_filters.md).

**Filter type value space (`enum FilterType`):** PEQ types occupy 0–7 plus the first-order all-pass at 8, the first-order low/high shelves at 9–10 (added 2026-06-20), the Linkwitz Transform at 11 (added 2026-07-12), and the first-order low/high pass at 12–13 (added 2026-07-27), with 14–31 reserved for future PEQ types; crossover types occupy 32–63 (see [Crossover Filters](#crossover-filters)). `filter_is_peq_type()` (config.h) is the single classifier (any value below `FILTER_XOVER_FIRST`); `is_filter_flat()` uses it to flatten a crossover type that lands in a PEQ band slot. The crossover types were renumbered from the old 8–39 range on 2026-06-17 to open the contiguous PEQ block, which bumped `SLOT_DATA_VERSION` to 18 (pre-V18 presets migrated on load via `remap_filter_type_pre_v18()`) and `WIRE_FORMAT_VERSION` to 13. Adding the first-order shelves (new enum values only; no renumber, no on-disk layout change, no migration) bumped `SLOT_DATA_VERSION` to 19 and `WIRE_FORMAT_VERSION` to 14. The Linkwitz Transform adds a per-band target-`Q` sidecar (`peq_qp_x512`), tail-appended to `PresetSlot` and carried in the `WireBandParams.reserved[2]` bytes, which bumped `SLOT_DATA_VERSION` to 30 and `WIRE_FORMAT_VERSION` to 22 (V21..V29 slots still load with `qp` defaulting to 0 = 0.707). The first-order low/high pass added enum values only (no renumber, no layout change, no migration) and bumped **neither** version, unlike the first-order shelves before them; nothing is incompatible as a result, but hosts have no version signal for types 12–13 and must probe by setting one. `CS_PEQ_TYPE_COUNT` (`control_surfaces_nouns.c`) is anchored to `FILTER_HIGHSHELF1`, so a Control Surfaces `FILTER_TYPE` control still cycles only 0–10 and cannot reach them.

**RP2350 biquad (hybrid SVF/biquad):**
```c
{ float b0, b1, b2, a1, a2; float s1, s2;
  float sva1, sva2, sva3; float svm0, svm1, svm2;
  float svic1eq, svic2eq; uint32_t svf_type;
  bool use_svf; bool svf_first_order; bool bypass; }
```
Single-precision throughout. Per-band SVF or TDF2 biquad path selected at coefficient computation time. See [Hybrid SVF/Biquad Filtering](#hybrid-svfbiquad-filtering-rp2350) for details.

**RP2040 biquad:**
```c
{ int32_t b0, b1, b2, a1, a2, s1, s2; bool bypass; }
```
Q28 fixed-point. Both per-sample and block-based biquad processing implemented in hand-optimized ARM assembly (`dsp_process_rp2040.S`). Block-based `dsp_process_channel_block()` keeps s1/s2 state in high registers across the entire sample loop, shares operand decompositions across multiply groups, and uses r12 for intermediate saves — eliminating per-sample struct access, function call overhead, and redundant decompositions vs the C `fast_mul_q28()` version.

### Hybrid SVF/Biquad Filtering (RP2350)
*Last updated: 2026-08-05*

The RP2350 uses a hybrid filter architecture that selects between a State Variable Filter (SVF) and a Transposed Direct Form II (TDF2) biquad on a per-band basis. This provides better numerical stability at low frequencies (where single-precision biquad pole quantization is worst) while retaining the efficiency of biquads at higher frequencies.

**Crossover frequency:** `Fs / 7.5` (e.g. ~6400 Hz at 48 kHz). Bands below this use SVF; bands at or above use TDF2 biquad. The crossover is evaluated at coefficient computation time and stored in `bq->use_svf`.

**SVF implementation:** Based on Andrew Simper's "SvfLinearTrapAllOutputs" (Cytomic, 2021). The linear trapezoidal integrator SVF is unconditionally stable and has zero delay-free loops. Shelf filters use `k = 1/Q` (not `1/(Q*sqrt(A))`), which produces an exact match with the RBJ Audio-EQ-Cookbook shelf response — eliminating any discontinuity when bands cross the SVF/biquad crossover boundary.

*Last updated: 2026-03-02*

**SVF coefficient equations:**

| Filter Type | g adjustment | k adjustment |
|-------------|-------------|--------------|
| Lowpass / Highpass | none | `1/Q` |
| Peaking | none | `1/(Q*A)` |
| Low Shelf | `g / sqrt(A)` | `1/Q` |
| High Shelf | `g * sqrt(A)` | `1/Q` |

Where `g = tan(pi * freq / Fs)` and `A = 10^(gain_dB/40)`.

**Per-type inner loop specialization:** The block-based `dsp_process_channel_block()` uses `switch(bq->svf_type)` to select a specialized inner loop for each filter type, eliminating zero-multiplies:
- **Lowpass:** output = v2 (no multiply for m0=0, m1=0)
- **Highpass:** output = in + m1*v1 - v2 (m0=1, m2=-1 folded)
- **Peaking:** output = in + m1*v1 (m0=1, m2=0 folded)
- **Shelf (default):** output = m0*in + m1*v1 + m2*v2 (general form)

**First-order (one-pole) SVF (added 2026-06-20):** First-order types (`FILTER_ALLPASS1`, `FILTER_LOWSHELF1`, `FILTER_HIGHSHELF1`, and the crossover BW1 / odd-order 1st-order sections) cannot use the 2nd-order SVF, so below `Fs/7.5` they run a one-pole TPT integrator instead: `v1 = sva2*in + sva1*ic1; ic1 = 2*v1 - ic1`, output `m0*in + m1*v1 + m2*(in - v1)` (lp = v1, hp = in - v1). The `1/(1+g)` reciprocal is folded into `sva1` (with `sva2 = g*sva1`) so the loop is multiply-only; only `svic1eq` is used. The path is gated by `bq->svf_first_order`. Above `Fs/7.5` these run a degenerate TDF2 biquad, exactly as the 2nd-order types switch to biquad. This makes first-order types follow the identical hybrid rule as every other type (previously they were forced to TDF2 at every frequency).

**Linkwitz Transform SVF (added 2026-07-12):** The Linkwitz Transform (`FILTER_LINKWITZ_TRANSFORM`) is realized on the hybrid path as a full 2nd-order Simper SVF tuned at the **pole** pair: `g = tan(pi*fp/Fs)`, `k = 1/Qp`, with the output mix `m0 = 1`, `m1 = (g0/g)/Q0 - k`, `m2 = (g0/g)^2 - 1` (where `g0 = tan(pi*f0/Fs)` encodes the driver's measured corner). The SVF form is used only when **both** corners (`f0` and `fp`) sit below `Fs/7.5`; if either corner is at or above the boundary the band falls back to the exact-bilinear TDF2 biquad with both corners independently prewarped. This both-corners rule differs from the single-frequency test the other types use, because the transform's zero and pole must both be in the SVF-accurate region for the realizations to match.

**Fused two-section cascade kernels (added 2026-08-05, extended to SVF pairs 2026-08-06, RP2350 only):** The block kernels used to be strictly band-major: every active band did its own read-modify-write sweep of the sample buffer, so an N-band cascade moved the block through memory N times. `dsp_cascade_block()` (`dsp_pipeline.c`, declared in `dsp_cascade.h`) now walks the cascade and, whenever the next two active sections are second-order and on the **same** path, runs them through one fused kernel: `dsp_biquad_second_order_x2()` (`dsp_biquad.h`) for biquad-path pairs, `dsp_svf_second_order_x2()` (`dsp_svf.h`) for SVF-path pairs. Each sample is loaded once, pushed through both sections while it stays in registers, and stored once, so a fused pair halves buffer traffic and loop administration versus two sweeps. Since 2026-08-05 the kernel also carries per-type specialized arms (see dispatch rules below), so a fused pair costs no more FP ops than the two single sweeps it replaces. Hardware A/B against a build with pairing disabled measures the specialized fused kernel at 19 % less EQ cost on peaking-heavy biquad cascades, 17 % on shelf-heavy ones, and 8 % on LR8 crossovers; against the earlier generic-only fused kernel it is 13 % better on peaking and 8 % on crossovers. Cost is ~1.2 KB of RAM text.

**SVF pairs (added 2026-08-06).** An earlier fused SVF kernel that forced the generic three-term output mix on every pair was measured on hardware at +14 % on peaking-heavy cascades and removed: the extra FPU ops outweighed the saved memory traffic, because the single SVF kernel is switch-specialized per filter type. `dsp_svf_second_order_x2()` repeats the biquad kernel's fix instead: each arm reuses the matching single-kernel output expression, so no fused section ever pays for another type's mix, and the pair still collects the load/store saving. Hardware A/B at 48 kHz / 307.2 MHz over 17 channels measures the SVF band-sweep slope falling from 2.29 to 1.85 % CPU per band (-19 %); on 10-band cascades the EQ cost (net of the flat-pipeline floor) drops 17 % for peaking, 21 % for an 8-active/2-bypassed load, 8 % for shelves, and 9 % for an LR8 HP+LP crossover whose SVF high-pass sections now pair. Biquad-path rows and SVF/biquad alternations are unchanged, as expected. Cost is ~4.1 KB of RAM text for the specialized arms plus ~1.2 KB for the generic arm.

The M33 FMA pipeline is throughput-bound, not latency-bound: interleaving two independent recurrences gains nothing, and loads/stores already overlap FPU work, so only reducing FP-op count pays on this core. That is why arm-for-arm op-count parity, not fusion itself, is the load-bearing property.

Dispatch rules:
- Section order is preserved exactly; bypassed sections are skipped and do not break the pair around them.
- First-order sections and mixed biquad/SVF neighbours are never fused; they fall back to the existing single kernels. Two second-order sections fuse only when they share a path (`use_svf`) **and**, on the SVF path, the same kernel arm.
- The fused biquad kernel picks one of three loop bodies **once per call**, never per sample: a peaking arm when both sections are `FILTER_PEAKING` (each section reuses the single kernel's `b1 = a1` form, 8 FP ops per section per sample), an LP/HP arm when both sections are `FILTER_LOWPASS` or `FILTER_HIGHPASS` in any combination (each reuses the `b0 = b2` form with the `b0*x` product computed once, also 8 ops), and the generic TDF2 form for everything else (9 ops). Because each arm's expressions match the corresponding single-kernel arm exactly, a fused sweep is bit-identical to two sequential single sweeps for those types, verified over 240 host cases covering every tail remainder and state carry-over. `FILTER_NOTCH` and `FILTER_ALLPASS` pairs (7 ops each) are deliberately **not** specialized: adjacent same-type notch or all-pass pairs are rare in practice and each extra arm costs ~580 bytes of RAM text against an already-exceeded `.data` budget.
- The fused SVF kernel picks its loop body once per call from `dsp_svf_arm_class()` (`dsp_svf.h`), which buckets a section into one of three arms: `LPHP` (`FILTER_LOWPASS` / `FILTER_HIGHPASS`), `PEAK` (`FILTER_PEAKING` / `FILTER_NOTCH` / `FILTER_ALLPASS`, which share one arm in the single kernel), and `GENERIC` (shelves, Linkwitz Transform, anything else). Only equal-class neighbours pair, so each fused section keeps its single-kernel op count; the `LPHP` arm splits further into the four low/high-pass orderings because low-pass emits `v2` directly while high-pass needs the three-term mix. Bit-exactness against two sequential single sweeps is verified over 440 host cases covering every arm, all four `LPHP` orderings, every tail remainder, and state carry-over across calls. `DSPI_FUSE_GENERIC_SVF_PAIRS` in `dsp_pipeline.c` gates the generic arm; it is on because hardware measured shelf cascades 12 % cheaper with it than without.
- The walker is shared: the crossover kernel (`xover_process_channel_block()`) calls the same `dsp_cascade_block()` per band, so crossover cascades pair on both paths: biquad sections above `Fs/7.5`, and SVF sections below it through the `LPHP` arm (every second-order crossover section carries `FILTER_LOWPASS` or `FILTER_HIGHPASS`). It is deliberately **out-of-line** so the unrolled kernel bodies are emitted into RAM once rather than duplicated into both callers; this made RP2350 `.text` and RAM image ~3.3 KB *smaller* than the pre-fusion band-major code.

RP2040 is unaffected: its band-major assembly kernels (`dsp_process_rp2040.S`) and their dispatch are unchanged.

**State reset:** When a band changes filter topology — crossing the SVF/biquad boundary (e.g. due to sample rate change) or switching between the one-pole and 2nd-order SVF (`bq->svf_first_order` flips) — both biquad state (`s1`, `s2`) and SVF state (`svic1eq`, `svic2eq`) are reset to zero to prevent transients.

**Input validation:** Frequency clamped to [10 Hz, 0.45×Fs], Q clamped to [0.1, 20].

**FPU configuration (RP2350):** Both cores set FPSCR flush-to-zero (FZ) and default-NaN (DN) bits at startup. This prevents denormalized floats from causing performance penalties as SVF integrator and biquad states decay toward zero after silence.

**FP contraction (added 2026-08-05, RP2350 only):** `dsp_pipeline.c` is compiled with `-ffp-contract=off`, so the EQ kernels use separate VMUL/VADD instead of fused VFMA. Hardware measurement on the CPU meter established that the M33 FMA pipeline is throughput-bound with an effective VFMA occupancy of roughly 2 to 2.5 cycles versus 1 for VMUL/VADD: de-contraction emits ~58 % more FP instructions (and eliminates the VMOV accumulator copies VFMA's destructive form forces) yet measures 14 to 24 % less EQ cost on every kernel path, SVF and biquad alike, with no register spills. Loads/stores already overlap FPU issue, so FP-op *occupancy* is the only currency that matters in these loops. That is why the fused kernel only became worthwhile once its arms were specialized to match the single kernels' op counts (above): while it used the generic 9-op form it was giving back most of what the saved memory traffic won. Precision cost of double rounding is negligible and was verified on hardware: loopback THD, noise floor, and flat-path residual byte-identical to the contracted build, filter responses within 0.01 dB, and host analysis puts the noise penalty at ~1.5 dB on a −137 dB re-signal error floor. The flag is per-file: other DSP translation units (loudness, psybass, leveller, crossfeed, upmix) still contract and are candidates for the same measure-then-decide treatment. `subharm.c` (added 2026-09-02) also compiles with `-ffp-contract=off`: a static count of its kernel loop shows FMAs 70 to 0, VMOV accumulator copies 25 to 4, FP instructions 109 to 175, and an occupancy-weighted estimate about 9 % fewer cycles per sample; the hardware CPU-meter confirmation is still pending. `tube.c` (added 2026-09-18) compiles with `-O3 -ffp-contract=off` on the same static-estimate basis as `subharm.c`, and is likewise unmeasured on the CPU meter. `rta_bass.c` (added 2026-09-12) compiles with `-O3 -ffp-contract=off` on RP2350 only. Its float bass kernel goes from 8 fused multiply-adds to none, at a cost of 256 B of RAM code, and the speed gain is unmeasured (`RtaStatus.bass_busy_us_per_s`). RP2040 keeps `-O2`: its fixed-point kernel has no FP ops, and `-O3` only unrolled the loops for 1.6 KB more RAM code.

*Last updated: 2026-09-18 (tube.c FP contraction, static estimate only; 2026-09-12: rta_bass.c FP contraction, RP2350 only; previously 2026-08-06)*

**Memory impact:** Biquad struct grows from ~48 to ~68 bytes on RP2350. With 110 EQ biquads at the larger size: ~3 KB additional BSS. (Loudness no longer uses the full `Biquad` struct; since 2026-07-09 its per-output shelf state is a separate minimal array, `loudness_output_state`, 144 B on RP2350 / 80 B on RP2040.)

**RP2040:** Completely unaffected. All SVF code is inside `#if PICO_RP2350` blocks.

### Per-Band Bypass
*Last updated: 2026-10-23 (defaults clear the bypass byte). Previously, 2026-05-04*

Each EQ band has a user-controllable bypass flag in `EqParamPacket.bypass` (config.h). When set to exactly `1` it forces `Biquad.bypass = true` inside `dsp_compute_coefficients()`, which causes the audio inner loops in `dsp_pipeline.c`, `dsp_process_rp2040.S`, `audio_pipeline.c`, and `pdm_generator.c` to skip the band entirely (`if (bq->bypass) continue;`). User bypass and the existing auto-bypass for `FILTER_FLAT` / zero-gain peaking/shelf filters share the same `Biquad.bypass` flag — `channel_bypassed[ch]` (true when *every* band on the channel is bypassed) automatically benefits, allowing the audio path to skip the entire channel.

**0xFF safety:** the byte at offset 3 of `EqParamPacket` was previously named `reserved`. To stay backward-compatible with hosts that may zero-pad with `0xFF` rather than `0x00`, the firmware treats *only* the literal value `1` as bypass — every other value (0, 0xFF, 0x42, …) leaves the band active. Intake points (`REQ_SET_EQ_PARAM`, `REQ_SET_BAND_BYPASS`, bulk-params apply, flash preset load) normalize the byte to 0 or 1 so GET round-trips and the live recipe stay clean.

**Wire format:** `WireBandParams.reserved[3]` was split into `bypass` (1 byte) + `reserved[2]` (still zero-padded). Same byte layout, no `WIRE_FORMAT_VERSION` bump needed.

**Persistence:** `filter_recipes` is `memcpy`'d into `PresetSlot.filter_recipes` (flash_storage.c:500/625) so the bypass byte rides through preset save/load with no `SLOT_DATA_VERSION` bump. Legacy presets had `0` in that byte → they load as "active", preserving original behavior. `apply_slot_to_live()` re-normalizes after `memcpy` for defense-in-depth.

**Factory-default channel/band fix (2026-05-04):** `dsp_init_default_filters()` now writes `.channel = ch` and `.band = b` into every `filter_recipes` slot. Previously these fields stayed at BSS-zero, which caused `REQ_SET_BAND_BYPASS` to silently misroute writes to slot (0,0) since the handler copies the recipe (carrying the stale `.channel`/`.band`) into `pending_packet`, and the main-loop apply does `filter_recipes[p.channel][p.band] = p`. Symptom: bypass toggle didn't take effect on factory-defaulted bands until `REQ_SET_EQ_PARAM` rewrote the recipe with correct channel/band. `apply_slot_to_live()` (`flash_storage.c`) also normalizes `.channel` / `.band` after the recipe `memcpy` so old presets saved before the fix don't carry the bug into live state.

**Factory defaults are now flat (2026-05-04):** `dsp_init_default_filters()` no longer installs the 80 Hz highpass on output channels nor the 80 Hz lowpass on the PDM sub. Every band starts as `FILTER_FLAT` with `freq = 1000`, `Q = 0.707`, `gain_db = 0`, `bypass = 0`. Crossover/sub configuration is now an explicit user choice via the host, not a baked-in default that the user has to discover and override.

**Defaults clear the bypass byte (2026-10-23):** until v1.1.6-beta5, `dsp_init_default_filters()` wrote every PEQ recipe field except `bypass`, so the `bypass = 0` above held only while the BSS zero survived. A band bypassed earlier in the session stayed bypassed through factory reset, an empty-slot load or delete, and a boot with no preset. Bulk GET then reported it bypassed, the Console's preserve-bypass policy kept it bypassed through later edits, and a preset save persisted it. The initializer now writes `bypass = 0` explicitly, matching `xover_init_default_filters()`.

**Vendor commands:**
- `REQ_SET_EQ_PARAM` (0x42): host can set the byte directly inside the `EqParamPacket` payload.
- `REQ_GET_EQ_PARAM` (0x43): added `param=4` → returns 1-byte bypass.
- `REQ_SET_BAND_BYPASS` (0xD8) / `REQ_GET_BAND_BYPASS` (0xD9): direct toggle, `wValue = (channel<<8)|band`, payload/return = 1 byte.

**CPU impact:** zero in the audio path (the `bq->bypass` check already existed); a bypassed band skips its biquad/SVF math, so user-bypass is a small *win* vs. an active band. One added byte comparison per coefficient recompute (only on user write or rate change).

Spec: [`Documentation/Features/band_bypass_spec.md`](Features/band_bypass_spec.md).

### Band Counts

| Platform | Master (ch 0-1) | Outputs | Max total biquads |
|----------|-----------------|---------|-------------------|
| RP2350 | 10 bands | 10 bands × 9 outputs | 110 |
| RP2040 | 10 bands | 10 bands × 5 outputs | 70 |

### Delay Lines
*Last updated: 2026-10-10*

NUM_DELAY_CHANNELS = NUM_OUTPUT_CHANNELS (platform-dependent).

| Platform | Channels | Type | Ring size | Max delay (48kHz) | RAM usage |
|----------|----------|------|-----------|--------------------|-----------|
| RP2350 | 9 | float | 2048 | 42.6 ms (2047 samples) | 72 KB |
| RP2040 | 5 | int32_t | 1024 | 21.3 ms (1023 samples) | 20 KB |

Circular buffer: `delay_lines[ch][(write_idx - delay_samples) & MAX_DELAY_MASK]`

`dsp_update_delay_samples()` caps every channel at `MAX_DELAY_SAMPLES - 1`. A delay of exactly `MAX_DELAY_SAMPLES` would read back the sample written in the same step, which is zero delay. Before 2026-10-10 the cap was `MAX_DELAY_SAMPLES`, so any over-range request (which the vendor handlers accept unbounded) silently played undelayed. The cap is applied in samples at recompute time, so a stored millisecond value that fits at 48 kHz is capped, not rejected, after a switch to 96 kHz. GET returns the stored, uncapped value.

PDM sub gets automatic alignment compensation: the ring fill target minus `PDM_BUFFER_SAMPLES` (256), added before the cap. It follows the selectable output latency (see "Output Latency"): +128 samples at 8 ms and 48 kHz, but negative below 256 frames (for example -64 at the 4 ms default at 48 kHz), which shortens the sub's delay. *Updated 2026-10-23*

**Known limitation (temporary, until PDM becomes an assignable slot type without hardcoded alignment):** because a positive sub offset is added before the cap, the sub reaches the cap that many samples of user delay earlier than the other outputs (128 at an 8 ms target at 48 kHz). With a user delay within that band of the ring size, the sub plays up to the offset (2.7 ms at 48 kHz) earlier than other outputs set to the same delay. Below that band all outputs align exactly. A sub-aware cap (`MAX_DELAY_SAMPLES - 1 - SUB_ALIGN_SAMPLES` applied to the user delay, offset added after) was considered and deferred to the PDM overhaul, which removes the offset entirely. The Console delay sliders (42 ms RP2040, 85 ms RP2350) also exceed the ring and should be bounded by a firmware-reported maximum.

---

## Crossover Filters
*Last updated: 2026-09-22*

### Purpose

A dedicated per-output crossover stage between the matrix mixer (PASS 4) and per-output PEQ (PASS 5+). Lets each output be band-limited to a specific driver — woofer LP, tweeter HP, midrange BP, sub LP. Standard pro-audio active-monitor signal flow: matrix → driver split → driver-correction PEQ → output.

### User model

- 4 crossover bands per channel (`MAX_XOVER_BANDS = 4`).
- Each band is one user-visible filter; internally a cascade of up to 4 biquad sections.
- Same `EqParamPacket` wire shape as PEQ; `Q` and `gain_db` fields exist but are ignored for crossover filter types.
- Crossover applies to output channels only. Storage is uniform across NUM_CHANNELS, but writes to input channels (`ch < CH_OUT_1`) are rejected at the vendor handler.

### Band-index addressing

Crossover bands share the band-index space with PEQ for all band-addressing vendor commands (REQ_SET_EQ_PARAM, REQ_GET_EQ_PARAM, REQ_SET_BAND_BYPASS, REQ_GET_BAND_BYPASS):

- 0..9 = active PEQ band
- 10..19 = reserved (rejected); wide gap so PEQ can grow without moving crossover
- 20..23 = crossover band 0..3 (`XOVER_BAND_BASE = 20`)

The crossover base was widened from 12 to 20 on 2026-06-04. Because `REQ_GET_EQ_PARAM` previously packed the band into a 4-bit `wValue` field (max 15), its `wValue` layout changed to a 5-bit band field: `(channel << 8) | (band << 3) | param` (param dropped from 4 bits to 3; only 5 values exist). The other three band-addressing commands and the bulk transfer already use an 8-bit band field and were unaffected. The wire-format size, persistence layout, and filter-type enum were all unchanged by that 2026-06-04 change.

**Filter-type renumber (2026-06-17):** the crossover *type* values (`enum FilterType`, distinct from the band indices above) moved from the old 8–39 range to **32–63** so the PEQ type block could grow contiguously (first-order all-pass added at type 8, 9–31 reserved for future PEQ types). Band-index addressing is unchanged. This bumped `SLOT_DATA_VERSION` to 18 (pre-V18 presets remap their stored type values on load via `remap_filter_type_pre_v18()`) and `WIRE_FORMAT_VERSION` to 13. See `crossover_filters_spec.md` §2 and the FilterType value-space contract comment in `config.h`.

### Filter families

`FilterType` enum, crossover types at indices 32..63 covering 32 types: LR2/4/6/8 × LP/HP, BW1..BW8 × LP/HP, Bes2/4/6/8 × LP/HP. Per-band section count derived from filter order. As of 2026-06-20 first-order sub-sections (BW1/3/5/7 and LR6, which is `(BW3)²`) follow the same per-section hybrid rule as the 2nd-order sections: a one-pole TPT SVF below Fs/7.5 (`bq->svf_first_order`) and a degenerate TDF2 biquad above. (Previously they were forced to TDF2 at every frequency, since the Cytomic SVF is fundamentally a 2nd-order topology; the one-pole TPT integrator added 2026-06-20 covers the 1st-order case.) Second-order sections use SVF below Fs/7.5, TDF2 above — the same rule per section that PEQ uses per filter.

### Pipeline insertion

```
matrix mixer → CROSSOVER → per-output PEQ → gain → delay → output encoding
```

Implemented as `xover_process_channel_block()` calls in:
- `audio_pipeline.c` single-core and dual-core branches on both platforms (4 sites)
- `pdm_generator.c` Core 1 EQ worker on both platforms (2 sites)

Kernel reuses the existing per-section TDF2 (RP2040) and SVF/TDF2 (RP2350) inner loops. RP2040 calls a new assembly entry point `dsp_process_band_cascade_block` that shares the inner-loop body with `dsp_process_channel_block` via local labels — only the band-loop terminator differs (parameter-supplied `num_sections` vs `channel_band_counts[channel]` lookup). Since 2026-08-05 the RP2350 side calls the shared `dsp_cascade_block()` per band, so a crossover cascade of second-order biquad-path sections pairs into fused two-section sweeps (SVF sections stay single; see "Fused two-section cascade kernels").

### State

- `xover_filters[NUM_OUTPUT_CHANNELS][MAX_XOVER_BANDS]`: designed biquad cascades, indexed by **output** (`ch - CH_OUT_1`)
- `xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS]`: user-supplied recipe (EqParamPacket), indexed by **channel**
- `output_xover_bypassed[NUM_OUTPUT_CHANNELS]`: fast-path flag per output; the stage is skipped entirely for an output when all 4 bands are bypassed (the default)

*Last updated: 2026-09-22.* The cascades and bypass flags carry no input rows because input channels can never run a crossover. The recipes keep the full `NUM_CHANNELS` width because they are copied verbatim into the preset slot and the bulk wire struct, both of which are laid out at that width; their input rows are inert. `xover_update_output_bypass()` takes an output index. The `eq_update_pending` handler in `main.c` converts `p.channel` to an output index and drops a crossover packet aimed at an input channel, since the upstream handlers should never let one through. Shrinking the cascades saved 9,352 B of BSS on RP2350 and 1,058 B on RP2040.

### State preservation across redesigns (2026-06-11)

`xover_design_filter()` snapshots each section's filter state (`s1`/`s2`, `svic1eq`/`svic2eq`) before the passthrough reset and restores it for sections that survive the redesign, so live edits (dragging fc, changing order) do not zero the cascade's memory and click. State is reset only when a section's SVF/TDF2 path changes, when the section drops out of the new cascade, or when the band toggles bypass; this matches the PEQ convention in `dsp_compute_coefficients()`.

### Interaction with global EQ bypass
*Last updated: 2026-06-26*

`REQ_SET_BYPASS` (`bypass_master_eq`) scopes to the **input/master PEQ only**. In the unified channel model the input channels carry the "master EQ" (the pre-matrix per-channel PEQ), so the global bypass gates the input-EQ pass (`!is_bypassed && !channel_bypassed[k]`) and nothing else. It does NOT bypass per-output PEQ and does NOT bypass crossover.

- Per-output PEQ is gated by its own `channel_bypassed[eq_ch]` fast-path flag only, independent of `bypass_master_eq`, on both platforms.
- Crossover is never gated by the global bypass on either platform: crossover filters are speaker-protection critical (a tweeter high-pass must survive a global "EQ off" toggle). Only per-band bypass and the channel-level fast path disable crossover processing.

Previously the RP2040 path also folded `is_bypassed` into its per-output PEQ gates (Core 0 dual/single-core output loops in `audio_pipeline.c` and the Core 1 EQ-worker output loop in `pdm_generator.c`), so a global bypass silently dropped output EQ on RP2040 but not RP2350. That gate was removed so both platforms now match: global bypass affects input/master EQ only.

### Band-field normalization (critical correctness invariant)

`xover_recipes[ch][i].band` always stores the **wire band index** (`XOVER_BAND_BASE + i` = 20..23), NOT the local 0..3. The dispatch path through `pending_packet → main.c::eq_update_pending` keys on `p.band` to choose between PEQ and crossover storage. If a stale local index leaked through (via REQ_SET_BAND_BYPASS's read-modify-write of the existing recipe), the update would misroute to PEQ band 0. Init, preset load, bulk apply, and the vendor handlers all explicitly normalize the band field — see `crossover_filters_spec.md` for the full discussion.

### Defaults

Every default band: `type=FILTER_FLAT, freq=1000.0, Q=0.707, gain_db=0, bypass=0, band=XOVER_BAND_BASE+i`. Because FLAT is not a crossover type, the design routine produces a bypassed cascade and `output_xover_bypassed[*] = true`. Zero per-sample cost until the user picks a real crossover type.

### Files

- `firmware/DSPi/crossover.h` / `.c` — coefficient design + per-platform processing kernels
- `firmware/DSPi/dsp_process_rp2040.S` — adds `dsp_process_band_cascade_block` entry sharing inner loop with PEQ kernel
- Pipeline insertion in `audio_pipeline.c` and `pdm_generator.c`
- Persistence in `flash_storage.c` (PresetSlot V16)
- Wire format in `bulk_params.h` / `.c` (WireBulkParams V11, new WireCrossoverConfig section)
- Live-edit dispatch in `main.c::eq_update_pending`
- Vendor handlers in `vendor_commands.c` (band-range extension)

Spec: `Documentation/Features/crossover_filters_spec.md`.

---

## Matrix Mixer
*Last updated: 2026-07-18 (loop bound generalized to n_matrix_sources; rows 2..4 shared by multichannel inputs 3..5 and the stereo upmixer's C/Ls/Rs)*

### Architecture

NUM_INPUT_CHANNELS inputs × NUM_OUTPUT_CHANNELS outputs with per-crosspoint control. `NUM_INPUT_CHANNELS` is **2 on RP2040** (USB L/R) and **8 on RP2350** (inputs 0/1 are the USB L/R bus shared by every input source; inputs 2-7 carry the extra channels only in 8-channel USB mode — see "8-Channel USB Input"). Outputs: 5 on RP2040, 9 on RP2350.

```c
typedef struct {
    MatrixCrosspoint crosspoints[NUM_INPUT_CHANNELS][NUM_OUTPUT_CHANNELS];
    OutputChannel outputs[NUM_OUTPUT_CHANNELS];
} MatrixMixer;
```

The matrix loop iterates `n_matrix_sources`, snapshotting the enabled `(source, signed gain)` pairs per output once per block. This is `n_active_inputs` (2 for stereo / S/PDIF / I2S, 4/6/8 for multichannel USB) plus, on RP2350 with the stereo upmixer active on a stereo input, 1 or 3 derived rows (2 + 1 centre-only, 2 + 3 with surround). Matrix-route vendor commands accept input indices 0-7 on RP2350.

**Row sharing (RP2350).** Source rows 2..4 are dual-purpose: they carry multichannel inputs 3..5 in multichannel input modes, and the stereo upmixer's Upmix C / Ls / Rs derived channels in stereo upmix mode. The crosspoint storage is the same in both cases, so a host must present those rows contextually by the active input mode (see "Stereo Upmixer").

**Crosspoint:** enabled, phase_invert, gain_db, gain_linear (pre-computed)

**Output channel:** enabled, mute, gain_db, gain_linear, delay_ms, delay_samples

### Signal Flow

```
USB L ──┐                    ┌── Output 1 (SPDIF 1L)
        ├── Matrix Mixer ────┤── Output 2 (SPDIF 1R)
USB R ──┘    (gain/phase)    ├── Output 3 (SPDIF 2L)
                             ├── ...
                             ├── Output 8 (SPDIF 4R)
                             └── Output 9 (PDM Sub)
```

Each output: `sample = L * gain_L + R * gain_R` (with phase invert option)

### Vendor Commands

| Command | Code | Description |
|---------|------|-------------|
| REQ_SET_MATRIX_ROUTE | 0x70 | Set crosspoint (input, output, enabled, phase, gain) |
| REQ_GET_MATRIX_ROUTE | 0x71 | Get crosspoint state |
| REQ_SET_OUTPUT_ENABLE | 0x72 | Enable/disable output channel |
| REQ_GET_OUTPUT_ENABLE | 0x73 | Get output enable state |
| REQ_SET_OUTPUT_GAIN | 0x74 | Set per-output gain |
| REQ_GET_OUTPUT_GAIN | 0x75 | Get per-output gain |
| REQ_SET_OUTPUT_MUTE | 0x76 | Set per-output mute |
| REQ_GET_OUTPUT_MUTE | 0x77 | Get per-output mute |
| REQ_SET_OUTPUT_DELAY | 0x78 | Set per-output delay (ms) |
| REQ_GET_OUTPUT_DELAY | 0x79 | Get per-output delay |

---

## SPDIF Output System
*Last updated: 2026-10-11 (hybrid PIO biphase encoder)*

### Multi-Instance Architecture

S/PDIF outputs share PIO0, each using one state machine. RP2350 has 4 instances; RP2040 has 2.

**RP2350 (4 instances):**

| Instance | GPIO | PIO SM | DMA Ch | Outputs |
|----------|------|--------|--------|---------|
| 1 | 6 | SM0 | CH0 | 1-2 (stereo pair) |
| 2 | 7 | SM1 | CH1 | 3-4 |
| 3 | 8 | SM2 | CH2 | 5-6 |
| 4 | 9 | SM3 | CH3 | 7-8 |

**RP2040 (2 instances):**

| Instance | GPIO | PIO SM | DMA Ch | Outputs |
|----------|------|--------|--------|---------|
| 1 | 6 | SM0 | CH0 | 1-2 (stereo pair) |
| 2 | 7 | SM1 | CH1 | 3-4 |

### PIO Program
*Last updated: 2026-10-11*

Two programs exist and the slot block (PIO0) holds exactly one of them, selected per configuration by `process_type_switches()` through `audio_spdif_select_encoder()` (full design and equivalence proof: `Documentation/Features/spdif_pio_bmc_encoder_spec.md`).

- **`audio_spdif_bmc.pio` (16 instructions, default).** Biphase-mark encoding in the PIO: two words per subframe, a control word carrying the preamble as two low-run counts and a data word carrying the 24-bit sample plus V, U and C. The parity bit is derived from the line level after the 27 data bits, so every subframe ends low and preamble polarity is fixed by construction. Shift right, no autopull, pull threshold 27. The CPU writes four words and two masks per frame. The writer's per-slot cost, measured at about 1.3% of core 0 before the change, is estimated at roughly 0.3% afterwards (not yet measured on hardware).
- **`audio_spdif.pio` (4 instructions).** The original NRZI transition-bit program fed by the software encoder (`spdif_update_subframe()` lookup table). Used only while I2S clock-slave mode is active **and** at least one output slot is I2S, because the checked I2S external-clock program (26 instructions) cannot share the block with the 16-instruction encoder.

Both programs run two PIO cycles per half-cell (256 cycles per frame, 256 Fs PIO clock from the shared base divider), consume four ring words per frame, and their entry points emit the same two low cycles before the first preamble bit, so the synchronized start, ring accounting and inter-slot alignment are identical whichever is loaded. Each instance records the block's loaded format in `pio_bmc` at setup; the format can only change inside a full output rebuild, where every staying S/PDIF slot is torn down and re-set-up with the retyped ones (`spdif_encoder_flip`). The last S/PDIF teardown on a block releases its program, and the I2S library releases its programs when its last instance on the block goes, so the lazy loads always find room. A program that does not fit panics with the block and program named.

The PIO0 budget per configuration is 16 slots with no I2S output slots, 32 of 32 with I2S slots under internal clocking (16 + clkout 8 + dataout 8), and 30 with I2S slots under an external master (4 + extclk 26). Every encoder flip empties the block before reloading, and the internal-clock programs are all multiples of 8 slots, so the top-down placement in `pio_add_program()` cannot fragment the memory. Clock divider automatically adjusted for 44.1/48/96 kHz.

### Instance State
*Last updated: 2026-10-12 (producer connection and formats removed). Previously 2026-07-23 (pre-write recovery fill snapshot added)*

```c
typedef struct audio_spdif_instance {
    PIO pio;
    uint8_t pio_sm, dma_channel, dma_reload_channel, pin, instance_index;
    bool enabled;
    uint32_t *ring;                     // per-slot encoded ring (caller-owned)
    volatile uint32_t wr_frames;        // frames of real audio written
    uint32_t erased_frames;             // silence stamped through this total
    volatile uint32_t underrun_frames;  // silence frames exposed to the wire
    volatile uint32_t drop_frames;      // producer frames dropped on ring-full
    uint32_t last_read_addr;            // consumed-side unwrap state
    uint32_t last_read_time_us;          // whole-lap selector for long polls
    volatile uint32_t consumed_words;
    volatile uint32_t snap_produced_frames, snap_consumed_frames;  // servo snapshot
    volatile uint32_t recovery_prewrite_fill_frames;  // UINT32_MAX until first write
} audio_spdif_instance_t;
```

### Buffer Configuration
*Last updated: 2026-10-23 (rings halved to 512 frames, erase-ahead 64, runtime fill target; see "Output Latency"). Previously 2026-10-22 (ring writers accept reader-ahead writes; per-slot drop deadlock). Previously 2026-10-16 (ring fill target unified into one constant). Previously 2026-10-12 (producer pools removed: per-slot staging, direct ring writes). Previously 2026-10-11 (dedicated S/PDIF silence stamper; consumed-count reader: whole-lap selector moved to a slow path). Previously 2026-07-23 (PDM/slot recovery made block-size exact)*

The consumer buffer pool is gone. Each slot now drains a single **encoded ring** filled by the writer and read by a free-running DMA with **zero output IRQs**:

- One static 8 KB ring per slot (`slot_ring_store[NUM_SPDIF_INSTANCES][PICO_AUDIO_SPDIF_RING_BYTES]` in usb_audio.c), caller-owned and passed via `config.ring_base`. The same store backs the slot's S/PDIF and I2S instances across output-type switches. `PICO_AUDIO_SPDIF_RING_FRAMES = 512` frames (10.7 ms at 48 kHz, 1024 before 2026-10-23), 2 channels × 8 bytes per frame; I2S uses the identical geometry (`_Static_assert`ed equal).
- **Staging and direct writes** (2026-10-12). The pipeline renders each slot's block into a static per-slot staging buffer, `slot_stage[NUM_SPDIF_INSTANCES][AUDIO_BUFFER_SAMPLES * 2]` in audio_pipeline.c (interleaved S24, 1.5 KB per slot). Core 0 writes pair 0 and, in the single-core variants, every pair. The Core 1 EQ worker writes its pairs through `core1_eq_work.spdif_out[]`, which always points at staging. Once `output_rings_pre_give()` admits the block, `output_rings_write_block()` copies staging into every set-up slot ring in slot order with `audio_spdif_ring_write_s32()` or `audio_i2s_ring_write_s32()`, chosen by `output_types[]`. The copy covers set-up rings that are not yet enabled, because a ring may be prefilling before its synchronized start. A refused block is never copied. The S/PDIF writer stamps the IEC preamble and channel status per frame from the absolute frame index. There is no `pico_audio` producer pool, connection or heap allocation anywhere in the output path, and neither output library links `pico_audio`.
- **Rate.** `audio_spdif_setup()` and `audio_i2s_setup()` take the rate in Hz and program the nominal divider themselves; the firmware passes `output_rate_hz` (usb_audio.c), which only `perform_rate_change()` writes. Nothing retunes lazily: `perform_rate_change()` reprograms every set-up slot (`apply_nominal_output_rate()` and `audio_i2s_update_all_frequencies()`) and then restarts the outputs in sync.
- Erase-ahead silence margin: the writer keeps `PICO_AUDIO_SPDIF_ERASE_AHEAD` = 64 frames (192 before 2026-10-23) of valid silence stamped ahead of real audio, so a stalled producer plays silence rather than a loop of stale ring content. The margin only has to last until `output_rings_idle_pump()` extends the silence to a full ring. The pump acts only once a reader has passed its writer (a stalled producer; gated on the signed fill below zero since 2026-10-23, previously below 240 frames and then half the target, which put the gate inside the fill sawtooth at small targets). Since 2026-10-26 it runs at every main-loop delivery point (`deliver_stream_input()` in `main.c`), so the margin covers one segment of the pass, which the delivery points keep short, rather than a whole pass; 64 frames is 1.3 ms at 48 kHz and 0.67 ms at 96 kHz. With four delivery points a whole pass could exceed 0.67 ms, which a 2026-10-26 audit flagged as a stale-audio risk at 96 kHz during a producer stall. Every frame of it is subtracted from the writable span, so it bounds the latency ceiling (see "Output Latency"). It still divides the 192-frame IEC block.
- On both platforms, `stamp_silence_frame()` writes zero-PCM S/PDIF words directly, retaining the phase-dependent Z/X and Y preambles, live channel-status bit, and matching parity. The writer's erase-ahead pass, underrun skip, and silence pump (including reset and flash-stall protection) use it instead of the general sample encoder. It is byte-identical to `stamp_frame(..., 0, 0)`, requires no lookup table or new storage, and is forced inline into the RAM-resident callers. Ring positions, frame counts, silence margin, and group alignment behavior are unchanged. Since 2026-10-11 the writer, skip and pump go through `stamp_audio_run()` / `stamp_silence_run()`, which pick the word format once per run from the instance's `pio_bmc` flag: `stamp_frame_bmc()` / `stamp_silence_frame_bmc()` write the PIO-encoder control and data words, the original stampers write the NRZI words, and both carry a running IEC phase.
- Free-running DMA: **ENDLESS** transfer count with hardware read-address wrap on RP2350 (power-of-two ring, no reload channel); on RP2040 a data + reload channel pair (`dma_reload_channel = OUTPUT_SLOT_RELOAD_DMA_BASE + slot`, base 6) rewrites the read address at the ring end. No DMA completes, so there is no output DMA IRQ on either platform.
- Fill, consumed-words, and underrun accounting are pointer arithmetic (always on). `wr_frames − consumed` gives fill. The DMA address supplies the exact position modulo the 512-frame ring; for calls separated by more than one lap, elapsed microseconds and the configured sample rate select the nearest whole-lap count, so the ~45 ms flash blackout cannot disappear from recovery accounting. Time is only the lap selector; the hardware pointer remains the sample-granular remainder. Since 2026-10-11 the selector is a slow path: a read less than half a ring after the previous one cannot have missed a lap (the rounded lap count is provably zero there, checked exhaustively for every rate and in-ring delta), so the reader decides that with one 64-bit multiply and compare and runs the 64-bit divide only after a genuinely late read. On the hot path that is about 13 reads per block saved. A coherent produced/consumed **snapshot pair** (`snap_produced_frames` / `snap_consumed_frames`) is latched at the end of each block write, giving the clock servo a constant-production-phase observation free of the block-arrival sawtooth; the fractional fill getter returns that snapshot difference (cumulative-count differencing). A separate `recovery_prewrite_fill_frames` snapshot captures fill immediately **before** each successful write (`UINT32_MAX` until the first write after reset). The separation is load-bearing: the servo wants post-write fill, while underrun recovery must exclude the final real block that PDM consumes before inserting zeros. The USB feedback loop consumes the same unwrapped ring read pointer at SOF (`audio_spdif_ring_consumed_words()`), which is sample-exact (see "Asynchronous Feedback Endpoint"). `REQ_GET_BUFFER_STATS` keeps byte compatibility via a 48-frames-per-bucket mapping (`SLOT_FILL_FRAMES_PER_BUCKET`; a ring fill of N frames maps to N/48 legacy "buffers"). The bucket is only a unit. Both clock servos read the frame-exact snapshot fill and express it as a fractional bucket count, and their gains are per bucket.

**Fill target.** Since 2026-10-23 the steady-state ring fill is a runtime value, `output_ring_target_frames`, derived from the selectable output latency. Its derivation, users and limits are in "Output Latency" below. Before that it was the constant `OUTPUT_RING_TARGET_FILL_FRAMES` (384 frames).
- **Underrun recovery** is a group re-anchor: `output_rings_pre_give()` (audio_pipeline.c) selects the worst enabled slot and applies one identical `*_ring_skip()` to every enabled slot, so inter-slot sample alignment is preserved exactly. Its skip is `deficit + recovery_prewrite_fill_frames`, with no bucket rounding. Algebraically this is the missing-source interval after the final successfully written block; it therefore equals the number of zeros PDM inserts after draining that same real block, preserving PDM-to-slot content alignment for arbitrary block sizes. A valid zero pre-write fill remains distinguishable from the reset sentinel. The starvation counters (`audio_spdif_get_dma_starvations_instance()` / I2S mirror) return frames of silence exposed to the wire, not silence-buffer DMA starts.
- **First anchor after a reset** (2026-10-23): a ring with no content since its reset (`recovery_prewrite_fill_frames == UINT32_MAX`) is anchored by its first real block, with the skip chosen so the post-write fill (what the servos regulate) lands on the target: `deficit + target - frames`. The idle pump (`output_rings_pre_give(0)`) no longer re-anchors such a ring unless the deficit passes 2^24 frames (about 5.8 minutes at 48 kHz, inside the signed fill range); its silence pump keeps the reader on fresh framed silence meanwhile, and ADAT's keepalive covers the dry spell as before. Previously the idle pump anchored first, at the full target with no block in hand, so the first block landed the fill anywhere up to a block above target and the slow USB fill servo took 10 to 20 s to bleed it off. Measured on RP2350 with USB at 48 kHz over 8 changes each: the largest overshoot after a change fell from 45-84 frames to 4-35. The anchor is one identical skip on every enabled slot, so alignment is unaffected.
- **Atomic re-anchor** (2026-10-11): `*_ring_reset()` and the ring arm set the software read anchor and reprogram the DMA read address inside one interrupt-disabled section. The USB SOF ISR reads slot 0's consumed position, and landing between those two writes would count the stale address as a full ring lap: slot 0's accounting would sit a full ring ahead of its physically aligned DMA, and admission and the underrun skip would fight over the phantom deficit until the next reset.
- **Group admission** (same function): a block is admitted only if every enabled slot ring can take all of it, and while Core 1 drains PDM's push queue, that queue must fit it too. A refused block reaches no output (slots, PDM, ADAT, loopback). PDM's queue is an admission input only while `pdm_queue_is_draining()` holds (PDM mode, enabled, not held, hardware running). An undrained queue fills up and would otherwise refuse every block for every output, for example while a PDM pin move holds Core 1's hardware. *Last updated: 2026-10-11*
- **Reader-ahead writes** (2026-10-22): the ring writers (`audio_spdif_ring_write_s32()` / `audio_i2s_ring_write_s32()`) treat a reader that has overtaken the writer as zero fill and write the block. Admission runs before the block's DSP, so a reader can overtake its writer in the few hundred microseconds before the write. The I2S reader runs about three frames ahead of the S/PDIF readers (deeper FIFO prefetch), so on a low-margin resume it is the first to cross. The writers used to compute fill unsigned, read "reader ahead" as "ring full" and drop the block for that slot alone. Every peer's `wr_frames` advanced and that slot's did not, so the slots split by one block per event. Once the split exceeded the writable headroom, admission refused every block forever while identical skips kept firing (the S/PDIF slots read 100% fill and the I2S slot read empty). Audio stayed silent until a full pipeline reset. Reproduced on a mixed S/PDIF + I2S layout by stopping USB packets for ~70 ms with the alt setting unchanged, as a host pause does. The pre-write recovery snapshot is clamped to 0 in this case: the frames the reader already passed are lost to that slot whatever the skip, so the PDM-to-slot offset can move by those few frames, and only during an underrun that is already audible. Every enabled slot must keep an identical `wr_frames`. Any per-slot write reject breaks that and deadlocks the group.
Since 2026-07-23 each per-slot record also carries `fill_centi_pct` (uint16, formerly the two pad bytes): sample-granular fill in 0.01% units of the same 16-bucket scale (5000 = 50.00%; values above 10000 are legal, the ring holds more than the legacy capacity). Older firmware reports 0 in this field; the coarse bucket fields are unchanged, so existing Console builds keep working.

### Output Latency
*Last updated: 2026-10-24 (underrun episodes, USB packet gap readout, host stall root cause). Previously 2026-10-23 (option table and low-latency option; module `output_latency.c`; two input delivery points per main-loop pass)*

The output latency setting is how full the slot rings are kept. Every slot shares one write index and one target, so the setting moves every ring output by the same amount and never moves one against another. The host picks it from a fixed table of options (`output_latency.c`), set and read with `REQ_OUTPUT_LATENCY` (0x03). The choice is stored device-globally (directory V23, never in a preset; a factory reset keeps it). The default is 4 ms.

**Options.** Each entry is a stable ID (the wire and flash value; 0 is never valid), flags, the ring fill target and the input batch the option needs:

| ID | Option | Target | Input batch | Sources |
|---|---|---|---|---|
| 1 | Low latency, 0.67 ms | 0.667 ms (29 / 32 / 64 frames at 44.1 / 48 / 96 kHz) | 0.333 ms (15 / 16 / 32 frames) | S/PDIF, I2S, ADAT (`OUTPUT_LATENCY_FLAG_STREAM_ONLY`) |
| 2 | Low latency, 1 ms | 1 ms (44 / 48 / 96 frames) | 0.333 ms | S/PDIF, I2S, ADAT |
| 3 | 2 ms | 2 ms | 1 ms | all |
| 4 | 3 ms | 3 ms | 1 ms | all |
| 5 | 4 ms (default) | 4 ms | 1 ms | all |
| 6 | 6 ms | 6 ms | 1 ms | all |
| 7 | 8 ms | 8 ms | 1 ms | all |

The two low-latency options share the 0.33 ms batch and differ only in margin: about 16 frames before scheduling jitter at 0.67 ms against 32 at 1 ms (see "Main-loop budget" below).

**Why low latency is stream-only.** USB delivers one packet per millisecond, so the ring must ride out a whole packet interval between arrivals, and a target under one packet runs dry every packet. Splitting packets into smaller pipeline calls does not help, because they all land together when the packet arrives. Sample-stream inputs arrive continuously, so their batch, and with it the target, can shrink. `output_latency_resolve()` runs a stream-only request on USB as the first table entry without the flag (2 ms) and keeps the request stored, so switching back to a stream input restores it.

**Live state.** `output_latency_apply(fs, source)` resolves the stored request for the active source and publishes three values together, which every user reads:
- `output_ring_target_frames`, the ring fill target: the option's latency in frames, clamped to the ring ceiling.
- `input_batch_frames`, the batch: the fewest frames a stream input hands `process_input_block()` per call (each call has a fixed cost; see `Documentation/Features/low_latency_overhead_plan.md`). The S/PDIF, I2S and ADAT readers all use it. Before 2026-10-23 S/PDIF batched by time (1 ms) but I2S and ADAT batched a fixed 48 frames, which is 0.5 ms at 96 kHz; standard options now batch all three at 1 ms, so a 96 kHz I2S input delivers twice the frames per call (less CPU, 0.5 ms more input-side wait).
- `output_ring_block_cap`, the most frames any producer may deliver per call: one batch plus one frame. A USB packet can carry one frame over nominal, and the stream readers use the same slack to catch up after a late poll. Every reader caps at it, and so do the I2S slave prefill and the test-signal pump.

It runs at boot (default, then the stored option after `preset_boot_load()`), in `perform_rate_change()` before the filter recompute, in the input-source switch right after the source flips (before the new source's restart or prefill), and when a SET applies. Each of those runs inside a pipeline reset bracket or before audio starts, so the restart that follows re-anchors every ring at the new target together.

**Ceiling.** One block must still fit above the target inside the writable span (ring minus erase-ahead), or group admission refuses blocks in steady state. The ceiling is therefore `512 - 64 - block_cap` frames. With the standard 1 ms batch:

| Rate | Block cap | Ceiling | Default 4 ms |
|---|---|---|---|
| 44.1 kHz | 46 | 402 frames (9.1 ms; 8 ms fits) | 176 frames |
| 48 kHz | 49 | 399 frames (8.3 ms; 8 ms fits) | 192 frames |
| 96 kHz | 97 | 351 frames (3.66 ms) | 351 frames (clamped) |

A device at 96 kHz runs the 4, 6 and 8 ms options at the ceiling and returns to them when the rate drops. `output_latency.c` static-asserts that the low-latency and 2 ms targets each exceed one block at 44.1 kHz, that 2 ms fits the ring at 96 kHz, and that the option table reply fits one EP0 packet.

**Protocol.** OUT: a 1-byte option ID. An unknown ID is ignored over USB (the data stage is acknowledged before any handler runs, as for every SET) and reported as `CTRL_STATUS_ERROR` over UART/I2C. Re-sending the stored option is a no-op. IN, wValue 0: the 12-byte `OutputLatencyStatus` (requested and effective ID, effective microseconds at the current rate, target frames, batch frames, pending flag, option count). IN, wValue 1: the option table as 8-byte `OutputLatencyOptionWire` entries (ID, flags with `OUTPUT_LATENCY_WIRE_AVAILABLE` set when the option runs as itself on the active source, nominal latency and batch, and what selecting it gives now). Other wValues STALL. A host builds its menu from the table, so a firmware can change the options without a host release.

**Users of the target.** The USB feedback servo (passed into `fb_ctrl_sof_update()` per SOF), the input servo (`servo_fill_target()`, read live; its smoother reseeds when the target changes, since the rings re-anchor at the new one), the prefill handshakes (`slot_fill_at_target()` and `producer_block_bounds()`) and the test-signal pump (`slot_fill_at_target()`), the underrun re-anchor fallback, the post-restart dwell and the PDM sub offset (`dsp_update_delay_samples()`).

**Applying a change.** The SET handler only records the request (`flash_set_output_latency_pending`). The flag, which the GET reports as `pending`, stays set until the restart below has completed, and a newer request that arrives meanwhile keeps it set for the next pass. The main loop runs it inside the standard flash bracket: `prepare_flash_write_operation()`, the directory write, `output_latency_request()` and `output_latency_apply()`, the sub-delay recompute, then `complete_flash_write_operation_full()`. That completion is the full synchronized restart (or, for S/PDIF input, the lock handshake's drain and prefill), so slot alignment is preserved by construction. The cost is one short mute per change. The servos are never used to move the fill: they work in ppm and would take minutes, and nothing per-slot is touched.

**Ring health readouts.** `REQ_GET_STATUS` wValue 26 returns underrun episodes (see "Underrun episodes" below), 27 returns blocks refused by group admission, and 28 returns the lowest enabled-slot fill seen just before a write, in frames, since `REQ_RESET_BUFFER_STATS` bit 0 (`0xFFFFFFFF` when there has been no write). Index 29 returns the worst gap between input deliveries in microseconds since the same reset (`input_delivery_mark()`, called by every `pipeline_service_producer()`): how long a stream input can go unread, which is what bounds the usable margin at small targets. Until the second delivery point (below) this was the worst main-loop pass. Index 30 returns the longest gap between audio-carrying USB packets since the same reset (`usb_audio_worst_packet_gap_us`), with no upper cap, unlike the 2 to 50 ms gap count at index 8. Indices 26 and 27 are cumulative.

**Underrun episodes (2026-10-24).** A dry episode runs from the first recovery to the next block that finds no ring dry. While a producer is stalled the idle pump re-anchors the rings on every main-loop pass, and each of those used to count, so one 100 ms host stall read as about 1000 underruns. `output_rings_pre_give()` now judges an episode when it ends. It counts once if a producer owed audio throughout, and not at all if any part of it fell in a mute window (`preset_loading`) or a closed USB stream (alt 0, with no generator running). The closure rule matters because a host stops sending a few milliseconds before it selects alt 0, so every deliberate stream stop would otherwise count. Recovery itself (the skip and the alignment it preserves) is unchanged.

**Measured, first pass (2026-10-23, RP2350, `tools/latency_bench`, before the option table).** Every setting from 2 to 8 ms in 0.5 ms steps was run for 30 s from the moment the change applied, on USB at 48 kHz, I2S input (clock master) at 44.1, 48 and 96 kHz, and the test-signal pump, under the stock configuration (core 0 about 10 %) and a 4-way active profile (core 0 / core 1 about 32 / 22 % at 48 kHz, 66 / 46 % at 96 kHz). Across all 130 points there were no underrun recoveries, no refused blocks and no USB ring overruns. The fill spread between playing slots never exceeded 1.1 frames, including across 20 rapid 2 ms / 8 ms switches. The lowest pre-write fill sat 50 to 70 frames below the target on USB (one packet plus scheduling jitter), so the margin at 2 ms is about 32 frames (0.67 ms) on the stock configuration and 27 frames (0.56 ms) under the 4-way load. The rate-follow check (8 ms at 48 kHz, clamped to 351 frames at 96 kHz, back to 384) and persistence across a reboot both held.

**Prefill landing (2026-10-23).** Sources that start through a prefill handshake (S/PDIF, ADAT, I2S in either role) used to enable the outputs at the first block that took the ring to the target. They landed up to a block high, plus whatever input queued during the last poll. The input servo pulled S/PDIF, ADAT and I2S slave back over minutes, and I2S in clock-master mode, which has no servo, stayed high until the next restart (measured +55 frames, 0.6 ms, at 96 kHz under the 4-way load). The handshakes now land on the target:
- At prefill start each source discards what it has already queued (`spdif_input_discard_queued()`, `i2s_input_discard_queued()` on every pair alike, `adat_input_discard_queued()` in whole frames). Nothing has been rendered since the outputs stopped, so the stream simply starts later and there is no content jump.
- Every stream reader takes its per-call limits from `producer_block_bounds()` (audio_pipeline.c): at least the option's batch, at most one block. While the slots are stopped for a prefill both limits shrink to what still brings slot 0 to the target, so the final top-up lands exactly on it and happens as soon as that many frames have arrived. The I2S slave-role path, which synthesizes silence because its input has no clock while the outputs are stopped, goes through the same limit.
- The handshake's existing `slot_fill_at_target(0)` check then starts the outputs on the next main-loop pass, with the ring exactly at the target. Only input that arrives within that pass lands on top.

A first attempt counted the queued input toward the target instead, starting the outputs with the ring as low as target minus one block and relying on the queue being delivered within that slack. On S/PDIF it dipped about 100 frames below the target right after every start and underran at 2 ms and below, so it was replaced by the bounds above.

**Main-loop budget (2026-10-23).** Stream inputs are read, and every block processed, on the Core 0 main loop, so the longest gap between deliveries bounds how long a ring can go unfed. At the low-latency targets that budget is about 0.3 ms (0.67 ms option) or 0.6 ms (1 ms option) beyond a normal pass. `tools/latency_bench/soak.py --profile-actions` times each host action in isolation, and a `DSPI_LOOP_PROF` bench build (`loop_prof.h`) splits the pass into named sections (`LOOP_PROF_SECTION_LIST` in `loop_prof.h`), read through `REQ_GET_STATUS` 64+. Measured on RP2350 with S/PDIF input:
- The quiet pass is about 70 to 80 µs. Every section carries a floor of about 53 µs from interrupts landing inside it (the S/PDIF receiver's block check every 4 ms), so any pass can grow by that much.
- Three tasks ran far longer and were reworked. The spectrum analyser's FINISH ran 530 µs in one call (now a background job in 16-bin chunks; see "Spectrum Analyser"). The loudness table rebuild ran 540 µs on every reference or intensity change (now a background job that builds into the inactive table and swaps when complete; see "Step budget and background jobs"). The idle pump gated at half the target, inside the fill sawtooth at small targets, and spent about 55 µs per pass stamping silence while streaming (now gated on a reader that has passed its writer).
- After those, single host actions add 20 to 170 µs over the quiet pass (vendor request handling, deferred applies, the first block they affect), with a Control Surfaces binding add or revert the largest at up to about 350 µs.

**Delivery points (2026-10-23, four points since 2026-10-24).** The first real-use soak (below) traced its one 0.67 ms underrun to a Control Surfaces revert. The revert itself took about 300 µs, but the input gap was that plus the rest of the pass, 470 µs. Shortening one task at a time would not end, because any two moderate tasks in one pass add up. The input service is therefore split three ways in `main.c`, and delivery runs between every segment of the pass that can run long:
- `input_service_handshakes()` runs the lock, prefill and signal-loss handshakes of the active source, once per pass.
- `pipeline_service_producer(true)` delivers whatever the active source (or the test-signal generator) has ready. It runs four times per pass: after host commands (`tud_task()`, notifications, the UART and I2C transports), after housekeeping (Control Surfaces tick, generator, idle pump, analyser service; this one sits between the handshakes and the servos), after the background step, and at the end, after the deferred operations. The reset spins call it with `lock_gated` false, so a fade completes on whatever still arrives; the main loop gates a clock-slave I2S input on LOCKED, as before.
- `input_service_servos()` runs the clock servos once per pass, because each rate-limits by call count.

The worst input gap is the longest single segment rather than a sum of them. The 2026-10-23 version had two points; Phase 4 of the background-jobs plan measured what was left (`Documentation/Features/main_loop_bg_jobs_plan.md`, "Phase 4 status") and found host commands and the Control Surfaces tick still sharing one segment: a bulk parameter read (up to 217 µs in a soak) on top of the tick with pots bound (up to 154 µs) could have reached about 370 µs against the 333 µs limit at 0.67 ms, and a background step shared its segment with the deferred block. An idle delivery is a few microseconds.

With two points, and before the revert became a background job, a Control Surfaces revert (then the longest single deferred task) measured about 290 µs of gap (it was 300 to 400 µs of pass). Roughly half of a revert is first-run flash cache misses (a cold revert takes 170 to 190 µs, an immediate second one about 95 µs), since that code is XIP-resident and rarely run. The revert and the stored-config load also no longer rebuild the button gesture groups once per slot (`cs_apply_binding_slot()` plus one `cs_bindings_changed()`; see Control Surfaces "Deferred SET, Apply/Save/Revert").

**Step budget and background jobs (2026-10-24).** `Documentation/Features/main_loop_bg_jobs_plan.md`, Phases 1 and 2. Long main-loop work runs as a job (`bg_job.h`): a step function that works until a deadline (always at least one unit, so it advances) and reports when the job is finished. `bg_run_one()` runs one step of the next active job, round-robin, after the housekeeping delivery point (the second of the four in each pass, see "Delivery points"), with the deadline `now + bg_step_budget_us()`. The budget is half of what one delivery gap can absorb after a quiet pass (`BG_PASS_FLOOR_US`, 120 us, measured on RP2350) and an interrupt landing (`BG_IRQ_FLOOR_US`, 60 us), clamped to 50 to 300 us:

| Option (48 kHz) | Target minus batch | Step budget |
|---|---|---|
| 0.67 ms | 16 frames, 333 us | 76 us |
| 1 ms | 32 frames, 667 us | 243 us |
| 2 ms and above | 48 frames or more, 1000 us or more | 300 us (cap) |

Because steps spend to a deadline rather than doing a fixed amount, the same code adapts to RP2040, RP2350 and the system clock. A step checks before each unit whether it still fits (`bg_out_of_time(deadline, estimate)`), so it ends on time rather than one unit late; an interrupt landing in a unit is what the 60 us margin covers. The estimate is the last time the same kind of unit took: the analyser keeps one per FFT stage and one for the finish, because chunk costs differ by stage (8 to 34 us on RP2350 at order 10), and the loudness rebuild uses its last volume step, since those cost the same. A kind of unit not yet measured carries the estimate `BG_UNIT_UNMEASURED` and so always starts a step of its own: its first run is often cold XIP code, and before this rule the first Control Surfaces revert after boot overran its budget by a few microseconds in every soak. Units have to stay small; a single large indivisible unit defeats the deadline. The first Phase 2 run showed this: the FFT permutation was one 140 to 150 us unit, and steps that reached it late ran to 440 us (15,000 to 30,000 overruns per soak) until it became resumable.

Jobs today:
- **Analyser transform** (`transform_job`, `rta.c`). `rta_packet_end()` starts it when a frame fills; it runs the FFT in 16-unit chunks (`RtaFftCursor`: prescale samples, butterflies within a stage, permutation cycle starts with the placed-slot bitmap kept in the cursor, split bin pairs), the finish in 16-bin chunks, and finishes when the frame is published. On RP2350 at order 10 a chunk costs 8 to 34 us. The publish (about 60 us, indivisible) only ever starts a step. `stop_engine()` cancels it.
- **Control Surfaces revert** (`control_surfaces.c`). `control_surfaces_revert_begin()` starts it; it runs the reload one slot or table at a time, and the module refuses or defers every write and answers reads with the reverted state while it runs (Control Surfaces "Deferred SET, Apply/Save/Revert").
- **Loudness rebuild** (`loudness.c`). `loudness_recompute_begin()` starts or restarts it; it builds volume steps into the inactive table until the deadline, swaps when complete, and `loudness_recompute_take_done()` tells the main loop to re-key. Boot keeps the blocking `loudness_recompute_table()`, which runs before any job.

`bg_step_account(job, us)` records every step. It increments `bg_over_budget_count` (`REQ_GET_STATUS` 31, all builds) when a step ran longer than its budget plus `BG_OVERRUN_MARGIN_US` (110 us, two interrupt landings), and on `DSPI_LOOP_PROF` builds keeps the job's worst step (96+job). The margin is wider than the one landing (`BG_IRQ_FLOOR_US`, 60 us) the budget sets aside, because under real use a step absorbs two landings about once in 10 s: with a 60 us margin the 0.67 ms soak counted 97 overruns of up to 79 us, all in seconds whose delivery gap stayed under 222 us against a 333 us limit. GET_STATUS 29 stays the ground truth for audio; the counter says which step to blame. `soak.py` fails a run that moves it.

**Real-use soaks (2026-10-23, RP2350, `tools/latency_bench/soak.py`).** Each run lasts 15 minutes with the same action seed. The analyser is read every 50 ms while 448 random host actions run (parameter edits, feature toggles, bulk reads, analyser reconfiguration, generator bursts, Control Surfaces binds and reverts). Results with two delivery points (one delivery point in brackets):

| Source, option | Underruns | Refused blocks | Worst gap max / p99 | Lowest pre-write fill (min / median) |
|---|---|---|---|---|
| S/PDIF, 0.67 ms | 0 (1) | 0 | 408 / 273 µs (491 / 334) | 0 / 10 frames (2 / 14) |
| S/PDIF, 1 ms | 0 (0) | 0 | 408 / 299 µs (482 / 364) | 17 / 26 frames (10 / 26) |
| USB, 2 ms | 2 host stalls (0); read as 178 before the episode count | 0 | 530 / 279 µs (598 / 334) | 33 / 41 frames (34 / 41) |

The 0.67 ms option no longer underran, but its lowest pre-write fill touched zero during a Control Surfaces revert, so it remains a thin-margin option. The 1 ms option kept at least 17 frames throughout.

**USB host stalls (resolved 2026-10-24).** The two USB bursts (88 and 90 recoveries, ten seconds apart), and an earlier 69-recovery window, were the Mac's doing. A diagnostic build logged every recovery: each burst was the idle pump re-anchoring on every pass for about 100 ms with no block arriving, on an open stream. The macOS log matches each burst to the second. `coreaudiod` reports `HALS_IOA1Engine::EndWriting: got an error from the kernel trap, Error: 0xE00002D7` (`kIOReturnOffline`), then `IO Stopped Context ... after 512 frames`, and the kernel stops sending isochronous data until the IO context restarts about 100 ms later, without a SET_INTERFACE. The HAL logs "skipping cycle due to overload" alongside, and the same `EndWriting` error appears all night on another audio engine on the same Mac, so it is a host IO-thread overload, not a DSPi fault. Firmware cannot fill audio the host never sends. What it did wrong was the reporting: the stall read as hundreds of underruns and was invisible to the 2 to 50 ms gap count. Both are fixed (underrun episodes above, `REQ_GET_STATUS` 30), and `soak.py` logs any second with a packet gap over 5 ms as a host stall. Verified on RP2350: a bench hook that discarded USB audio for 100 ms on an open stream counted exactly one underrun per stall (five of five), deliberate host stream stops of 0.15 to 8 s (the longer ones closing to alt 0) counted none, and steady streaming counted none.

**Low-latency option: what it depends on.** The margin is the target minus one batch minus scheduling jitter: 32 - 16 frames before jitter at 48 kHz, against 96 - 48 at 2 ms. Jitter does not shrink with the batch (USB control requests, the I2C display, the spectrum analyser and flash brackets all run on the same main loop), so heavy configurations are the risk. The CPU cost is the fixed per-call cost three times as often; `Documentation/Features/low_latency_overhead_plan.md` measures it and describes the overhead reductions (on the unmerged `bench/input-batch` work) that would recover most of it. The USB feedback servo's fixed ±1 bucket inner zone (48 frames, `FB_FILL_KNEE_Q16`) would be larger than the whole target, which is a further reason USB never runs this option. The PDM sub offset (target minus 256) clamps the sub's delay at zero below 256 frames; see PDM "Known alignment defect".

### IEC 60958-1 Block Phase
*Last updated: 2026-07-23*

Each 192-frame audio block carries channel status bits and a Z preamble at frame 0. With the ring, the block phase is simply the **absolute frame index mod 192**: `audio_spdif_ring_write_s32()` stamps the correct Z/X preamble and channel-status C bit per frame from the running write count, and the erase-ahead silence carries the same phase. In the PIO-encoder word format (default since 2026-10-11) the preamble is the control word (`BMC_CTRL_Z/X/Y`) and C is data-word bit 26 with parity generated by the PIO; in the NRZI word format C is h[29] with parity P at h[31] XOR'd alongside. `stamp_audio_run()` / `stamp_silence_run()` seed the phase once per run and carry it as a counter rather than taking a modulo per frame. No per-buffer template fixup or LIFO free-list correction is needed. `_Static_assert`: `PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT % PICO_AUDIO_SPDIF_ERASE_AHEAD == 0` style invariants keep the block/margin aligned at compile time.

### 24-bit Output Encoding
*Last updated: 2026-10-17 (RP2350 alt 6 adds 4ch/24-bit input)*

The USB input supports both 16-bit and 24-bit PCM via two stereo alternate settings on the Audio Streaming interface. On RP2350, alt 6 also carries 24-bit PCM for 4-channel input; the other multichannel alts are 16-bit. The host OS selects the desired bit depth; a runtime variable (`usb_input_bit_depth`) tracks the active format and branches the input conversion accordingly. With 24-bit input and 24-bit SPDIF output, the full precision signal path is maintained end-to-end. The DSP pipeline operates at >16-bit precision internally (float on RP2350, Q28 fixed-point on RP2040).

**Alt-change safety (2026-04-18):** With `TUP_DCD_EDPT_ISO_ALLOC` enabled on RP2040/RP2350, TinyUSB's `usbd_edpt_close()` is a no-op — it does not clear the `busy` flag left by the previous alt's in-flight iso xfer. On the next `usbd_edpt_xfer()` this trips `TU_ASSERT(busy == 0)` and crashes the device. `uac1_open_stream_eps()` therefore calls `usbd_edpt_clear_stall()` on both stream endpoints after `usbd_edpt_iso_activate()` to force-clear the stale busy bit (the same workaround TinyUSB's stock audio class driver applies at `audio_device.c:1871`). `uac1_apply_alt()` also flushes the USB audio ring whenever `usb_input_bit_depth` changes so queued pre-switch packets aren't re-decoded under the new bytes/frame assumption. See "Sample-rate & Bit-depth Switching" for the full mute/resync flow that brackets every format change.

**Input conversion (24-bit):**
- **RP2350:** 3-byte little-endian → sign-extended int32 → float via `÷ 8388608.0f`
- **RP2040:** 3-byte little-endian → Q28 via left-justify and `>> 2` (net `<< 6`); same full-scale as 16-bit (`<< 14`)
- **SPDIF RX:** extracted 24-bit samples follow the same internal full-scale convention: RP2350 sign-extends to int32 full-scale and divides by `2147483648.0f`; RP2040 shifts the sign-extended full-scale word by `>> 2` into Q28, matching the USB 24-bit `sample << 6` path so unity processing survives the later Q28 `>> 6` output conversion.

- **RP2350:** `float → int32_t` via `(int32_t)(sample * 8388607.0f)` (24-bit full-scale)
- **RP2040:** `Q28 → int24` via `clip_s24((sample + (1 << 5)) >> 6)` (shift right 6 with rounding)
- **Encoding:** with the PIO encoder loaded (default) the 24-bit sample is written raw into the data word, bits [23:0], and the PIO generates the biphase-mark cells and parity; in the external-clock I2S configuration `spdif_update_subframe()` encodes 3 bytes through the NRZI lookup table instead (see "PIO Program", 2026-10-11)
- **PIO/DMA:** Unchanged — BMC encoding is bit-width agnostic, subframe size is the same

### Channel Status (IEC 60958-3)

Channel status is encoded as a 5-byte array (40 bits) per IEC 60958-3 consumer format:

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | 0x04 | Consumer, PCM, copy permitted |
| 1 | 0x00 | General category |
| 2 | 0x00 | Source/channel unspecified |
| 3 | Dynamic | Sample rate (0x00=44.1k, 0x02=48k, 0x0A=96k) |
| 4 | 0x0B | Word length: max 24-bit, actual 24-bit |

Byte 3 is updated dynamically in `update_pio_frequency()` when sample rate changes.

### IRQ Handling
*Last updated: 2026-07-23*

Output S/PDIF and I2S generate **no DMA IRQs** at all: the rings are drained by free-running DMA (ENDLESS on RP2350, data+reload pair on RP2040). Since 2026-10-11 the libraries carry no IRQ plumbing either: the `dma_irq` config and instance fields, the shared enable refcount, the no-op `*_set_starvation_monitoring()` calls, the output IRQ priority setup and the type-switch IRQ mask are gone. S/PDIF RX is the only DMA IRQ user. It owns `DMA_IRQ_1` outright: `pico_spdif_rx` enables the line at start and disables it in teardown (the upstream behaviour, restored now that nothing shares the line), and `spdif_input_init()` sets it to the highest priority so the capture IRQ preempts the receiver's own decode-timeout alarm.

### Output Ring Stall Bracket
*Last updated: 2026-10-23 (lap times for the 512-frame ring)*

A slot ring is 512 frames, so it laps every 10.7 ms at 48 kHz and every 5.3 ms at 96 kHz (1024 frames before 2026-10-23). While the producer runs, the writer keeps an erase-ahead margin of silence in front of the reader, and the main loop's `output_rings_idle_pump()` keeps the ring silent when no producer runs. Neither happens while the main loop is stalled. If the reader laps during a stall, it replays whatever the ring held one lap earlier. Even when that content is silence, its S/PDIF framing is wrong: every frame carries its IEC 60958 block position from its absolute frame number (`frames mod 192`), and 512 is not a multiple of 192, so replayed frames sit 128 frames out of block step on every lap. Receivers that check preamble spacing can unlock.

The stall bracket (`output_rings_stall_begin()` / `output_rings_stall_end()`, `audio_pipeline.c`) closes that gap. `output_rings_stall_init()` claims a spare hardware timer alarm at boot and installs `output_rings_stall_irq` as its handler. While a bracket is open, the alarm fires every 1 ms (`OUTPUT_RINGS_STALL_PERIOD_US`) and runs `output_rings_stamp_silence_ahead()`, which stamps correctly framed silence from the write position up to one full ring ahead of every enabled slot's reader. It never touches unplayed audio, so pending frames play exactly once and are followed by silence with continuous block framing. On RP2350 the same tick runs `adat_output_keepalive()`. The 1 ms period is set by ADAT: its 96-frame cushion lasts 2 ms at 48 kHz, while a slot ring lap is 5.3 ms even at 96 kHz.

Rules for using it:

- **Open a bracket only where no ring is written, reset or torn down.** The alarm pumps without any lock, so the main loop must not touch a ring while the bracket is open. That is why the flash bracket cannot span the whole prepare-to-complete window: several handlers run `apply_restored_outputs()` in between.
- **Current brackets.** `flash_program_staged()` and `preset_delete()` open one around Core 1 lockout, `pdm_flash_silence()` and the erase/program window. `process_type_switches()` opens one around its DAC hardware-mute hold spin, and `pdm_hold_for_pin_move()` (main.c) around the PDM hold handshake (up to `PDM_HOLD_TIMEOUT_US`). Any new main-loop wait that can exceed a few milliseconds while slots run needs one too.
- **Nesting.** Brackets nest through a depth counter. `begin` stamps once immediately, so the first lap is covered before the alarm's first tick.
- **Flash windows.** `flash_irq_blackout_begin()` keeps exactly this alarm's NVIC line live (`output_rings_stall_nvic_mask()`). The handler and everything it calls are RAM-resident, enforced by `scripts/check_ram_placement.py`: the handler is the single `FLASH_WINDOW_ROOTS` entry and is also in the Check A/B hot list.
- **Flash prepare.** `prepare_flash_write_operation()` ends with one `output_rings_stamp_silence_ahead()` call, which covers the short main-loop stretch before the first flash primitive opens its bracket.

PDM is outside the alarm. It keeps its own producer on Core 1 except in a flash window, where `pdm_flash_silence()` covers it. ADAT output's handling across stalls is described under "ADAT Bulk Output".

### Synchronized Start

Every output-slot start in the firmware goes through one shared helper, `start_output_slots_in_sync()` in `main.c` (used by both `complete_pipeline_reset()` Phase 2 and `enable_outputs_in_sync()`; there is deliberately no other start path). It runs the prepare-only halves of both output libraries (`audio_spdif_enable_sync_prepare()` / `audio_i2s_enable_sync_prepare()`), then starts every slot of BOTH output types in a single `pio_enable_sm_mask_in_sync()` write (valid because both types share `PICO_AUDIO_SPDIF_PIO`). The single mask write starts all SM clocks in the same sys_clk cycle and resets their fractional clkdiv accumulators together, so mixed-type configs get the same cycle-exact start alignment single-type configs always had. Before 2026-07-21 the non-slave path issued two group starts (`audio_spdif_enable_sync()` then `audio_i2s_enable_sync()`) with the I2S DMA priming between them, leaving a fixed few-microsecond inter-type start skew and unsynchronized divider phase at fractional rates; unifying the paths also eliminated a persistent 1-2 buffer slot-0 consumer-fill deficit observed on hardware after every reset. Callers hold interrupts disabled across the helper so nothing can complete a primed DMA transfer for one type before the other type's clocks run.

In I2S clock-slave mode the same helper additionally gates on an external LRCLK falling edge between priming and the mask write (bounded ~35 us, only when an extclk I2S slot exists). Together with the external-clock I2S program's one-frame discard this makes the SPDIF-vs-I2S inter-slot offset reset-invariant. Both prepare halves first rewind every SM to its program entry with empty FIFOs (`pio_sm_clear_fifos` + `pio_sm_restart` + jump to the entry point), because a stopped SM keeps a partial OSR while a re-initialised one (pin move, type switch) starts empty. On the S/PDIF side the rewind runs before the ring arm, since the arm's FIFO prefill is already counted as consumed. *Last updated: 2026-10-10*
## PDM Subsystem
*Last updated: 2026-02-14*

### Purpose

Generate 1-bit PDM (Pulse Density Modulation) for subwoofer output via sigma-delta modulation.

### Hardware

- **PIO:** PIO1 SM0
- **DMA:** Dynamically claimed channel (typically 4+)
- **Pin:** GPIO 10 (default, reconfigurable)
- **Oversample:** 256x (12.288 MHz bitstream at 48 kHz audio)

### PIO Program

Single instruction: `out pins, 1` — shifts 1 bit from OSR to GPIO pin.

### Sigma-Delta Modulator

2nd-order error-feedback topology:
- Accumulator 1: `err1 += (target - output)`
- Accumulator 2: `err2 += (err1 - output)`
- Comparator: `output = (err2 >= 0) ? 65535 : 0`

**Noise shaping:** 2nd-order IIR highpass (Butterworth, fc=8 kHz at 384 kHz effective rate)

**Dither:** TPDF via PRNG, mask 0x1FF

**Leakage:** Both accumulators decay with shift 16 (~1.4s time constant at 48 kHz) to prevent DC offset buildup

### Communication (Core 0 → Core 1)

Ring buffer of 256 entries:
```c
typedef struct { int32_t sample; bool reset; } pdm_msg_t;
volatile pdm_msg_t pdm_ring[256];
```

Core 0 pushes Q28 samples; Core 1 pops, runs sigma-delta, writes DMA buffer. `__sev()` wakeups for low-latency handoff.

### DMA Ring Buffer

- Size: 2048 words (8192 bytes)
- Pre-filled with 50% duty cycle silence (0xAAAAAAAA)
- Core 1 keeps its write position `TARGET_LEAD` = 256 **words** (32 samples, 8 words per sample) ahead of the DMA read pointer. The DMA ring itself holds 256 samples. *Corrected 2026-10-11: this line previously said 256 samples.*

**Known alignment defect (found 2026-10-11 by code reading, open, not measured).** Two properties of the PDM path make its offset to the slots both wrong and non-deterministic. This predates the output rings.

- **Calibration.** The sub offset is the ring fill target minus `PDM_BUFFER_SAMPLES` (256), which assumes PDM latency equals its whole 256-sample DMA ring. Core 1 actually consumes the push queue eagerly and writes each sample about 32 samples (`TARGET_LEAD`) plus up to one block ahead of the reader. At an 8 ms target slots play a sample about 384 frames after it is written, while PDM plays it after about 32 + 128 samples of delay line, so the sub plays roughly 200 samples (about 4 ms at 48 kHz) ahead of the mains. Below a 256-frame target the offset clamps the sub's delay at zero, so the error shrinks to about the target minus 32 to 80 samples (about 2.3 ms at the 4 ms default). *Updated 2026-10-23 for the selectable target.*
- **Ratchet.** When a block arrives late and the lead falls below `TARGET_LEAD`, Core 1 inserts zero samples. They shift PDM permanently later against the slots, which absorb the same jitter in their ring fill without shifting. A lead above half the ring (a burst) is treated as an underrun and re-seats the lead, shifting PDM earlier. The offset therefore wanders with producer jitter until the next reset.

Every reset re-establishes the same starting point. The slots re-anchor to the fill target at the first block, and PDM sits at `TARGET_LEAD`, both after a pipeline reset and after `pdm_flash_silence()`'s forced re-anchor. The underrun skip's `deficit + recovery_prewrite_fill_frames` term preserves whatever offset exists. The fix is to give PDM the slots' accounting (a paced, slot-sized cushion and group-skip participation instead of eager consumption and local zero insertion), which belongs with the planned move of PDM to an assignable slot type.

### Input Limiting
*Last updated: 2026-10-23*

Both platforms hand the modulator samples with 0 dBFS at 2^29. The modulator clamps at 0 dBFS, then scales full scale onto `PDM_FULL_SCALE` = 29500 (about 90% modulation), the stable ceiling of the sigma-delta loop. The modulator therefore clamps only what the sub clip flag already reports. Before 2026-10-23 the RP2350 put 0 dBFS at 50% modulation and the RP2040 clamped from -0.9 dBFS. The new mapping moved the RP2350 sub up 5.1 dB and the RP2040 sub down 0.9 dB.

### Soft Start

*Last updated: 2026-02-17*

Linear fade-in/fade-out ramp applied to `pcm_val` after hard limiting, before the sigma-delta modulator. Eliminates pops on both turn-on and turn-off.

- **Fade-in:** Ramps from 0 to full scale over 1024 samples (~21 ms at 48 kHz) on every fresh entry to `pdm_processing_loop()`. Tracks effective `pcm_val` in `fade_base_pcm` for potential fade-out.
- **Fade-out:** When `pdm_enabled` goes false, the loop continues for 1024 more samples, ramping the held `fade_base_pcm` to zero. Sample acquisition is bypassed — the sigma-delta is fed a synthesized ramp so it smoothly converges to 50% duty cycle (silence) before PIO+DMA are stopped. The loop condition (`core1_mode == CORE1_MODE_PDM || fade_out_pos > 0`) keeps the loop alive during fade-out even if the mode has already changed.
- **Arithmetic safety:** pcm_val max 29500 × 1024 = 30 M, well within int32_t on M0+.

---

## Crossfeed
*Last updated: 2026-07-10 (per-output-pair stage; output-pair mask; dual-core split)*

### Purpose

BS2B (Bauer Stereophonic-to-Binaural) crossfeed for natural headphone spatialization.

### Presets

| Preset | Frequency | Feed Level | Character |
|--------|-----------|------------|-----------|
| Default | 700 Hz | 4.5 dB | Balanced |
| Chu Moy | 700 Hz | 6.0 dB | Stronger effect |
| Jan Meier | 650 Hz | 9.5 dB | Subtle |
| Custom | 500-2000 Hz | 0-15 dB | User-defined |

### Filter Topology

Per channel:
```
lp_out  = lowpass(input)           // ILD (head shadow simulation)
ap_out  = allpass(lp_out)          // ITD (interaural time delay)
direct  = input - lp_out           // Complementary highpass
output  = direct + ap_opposite     // Mix with opposite channel's crossfeed
```

**Complementary property:** Mono signals pass at unity gain (DC).

**ITD target:** 220 us (60 degree stereo speakers, 15 cm head width), implemented as 1st-order allpass.

### Per-Output-Pair Stage

Crossfeed runs as a **per-output-pair** stage (PASS 4.5) rather than a stereo-only input-bus stage. It sits **post-matrix and post test-signal injection**, immediately before the per-output crossover/EQ/gain/loudness/delay chain. This keeps its position relative to output EQ identical to before (a headphone EQ still shapes the post-crossfeed signal), while making it work in **every input mode** (2/4/6/8-channel USB, S/PDIF, I2S): it processes whatever stereo program the matrix routed to each pair, so the old multichannel bypass is gone.

- **Pairs are stereo output slots:** outputs `2p`/`2p+1`, `NUM_SPDIF_INSTANCES` pairs (4 on RP2350, 2 on RP2040). The mono PDM sub is not a pair and is excluded.
- **Output-pair mask:** `CrossfeedConfig.output_pair_mask` (bit `p` = run crossfeed on pair `p`). Default `0x01` (pair 1 only) at factory reset and when loading pre-V27 presets. The filter settings (preset / fc / feed / ITD) stay **global** and are shared by every selected pair; only the mask picks which pairs run.
- **Per-pair state, shared coefficients:** `crossfeed.c` publishes one shared `CrossfeedCoeffs` (lp_a0, lp_b1, ap_a) via `volatile const CrossfeedCoeffs *current_crossfeed_coeffs` (NULL = disabled; this replaces the old `crossfeed_bypassed` flag). Coefficients are double-buffered: `crossfeed_apply_config()` computes into the inactive buffer, then atomically publishes the pointer, so it never mutates the buffer the pipeline is reading (the `main.c` `crossfeed_update_pending` handler calls it). Each pair owns an independent `CrossfeedPairState crossfeed_pair_state[NUM_SPDIF_INSTANCES]` (4 filter states each: lp L/R, ap L/R) so pairs run in isolation.
- **Skip / reset predicate:** a pair runs when coeffs are published AND its mask bit is set AND neither channel of the pair carries a RAW test signal (`siggen_raw_mask`) AND both channels are matrix-enabled (crossfeed writes both buffers; a half-enabled pair would leak bleed into the disabled channel's zeroed, never re-zeroed buffer). Skipped pairs have their filter state reset each packet (`crossfeed_reset_pair_state`) so re-enabling starts clean; selected-off pairs cost zero DSP cycles.
- **Dispatch:** `crossfeed_process_pairs(coeffs, mask, first_pair, last_pair, buf_out, n)` iterates the assigned pairs, running `crossfeed_process_pair_block()` on selected pairs in place and resetting the rest.

### Dual-Core Split

Each packet snapshots the coefficient pointer + mask once and hands them to Core 1 via new `Core1EqWork` fields `xfeed_coeffs`/`xfeed_mask` (same single-view rationale as the loudness fields, so both cores apply the same view). **Core 0 owns pair 0**; **Core 1 crossfeeds its own pairs** (RP2350: pairs 1-3; RP2040: pair 1) at the top of `eq_worker_loop`, before its per-output EQ loop. The single-core path runs all pairs on Core 0. Crossfeed is pure IIR (no buffer delay), with identical `sample_count` on every output, so **inter-slot sample alignment is untouched**.

### RAM Cost

Double-buffered coeffs plus 4 (RP2350) / 2 (RP2040) pair states; net a few dozen bytes.

### Persistence & Control

- **Wire format V20:** `WireCrossfeedParams` reserved byte (offset 3) is now `output_pair_mask`; struct sizes unchanged.
- **Preset slot V27:** `uint8 crossfeed_output_pair_mask` tail-appended (struct grows 1 byte), gated on `version >= 27` at apply; older slots load `0x01`; factory default `0x01`; V26 CRC sizes still validate.
- **Vendor commands:** `REQ_SET_CROSSFEED_OUTPUTS` (0xFC, OUT, 1 byte pair mask, clamped to valid pair bits, notify on write) and `REQ_GET_CROSSFEED_OUTPUTS` (0xFD, IN, 1 byte). The existing 0x5E-0x67 crossfeed commands are unchanged. The mask is read live each packet, so 0xFC needs no coefficient recompute.

---

## Psychoacoustic Bass
*Last updated: 2026-07-13*

### Purpose

Missing-fundamental bass enhancement for small speakers. A speaker that cannot physically reproduce content below its low-frequency limit can still convey that bass psychoacoustically: the ear reconstructs a missing fundamental from a consecutive harmonic series (2f, 3f, 4f...). Psybass extracts the sub-cutoff low band per output, generates harmonics of it with a nonlinear device (NLD), band-limits those harmonics into the speaker's reproducible range, and mixes them back in. Status: **HW-untested**.

### Signal Flow (per output channel, in place)

```
y      = LP2_160(x)                        // full rate: top of band 2 and the decimation anti-alias filter
every D samples (D = round(fs/8000): 6 at 44.1/48k, 12 at 96k):
  s      = HP2_48(y)
  lo, b2 = LP/HP of SVF_112(s)             // band 2 = 112-160 Hz
  b0, b1 = LP/HP of SVF_72(lo)             // band 0 = 48-72, band 1 = 72-112
  d_b    = divider(b_b) * gate_b           // polarity flip once per cycle; optional selectivity weight
  sub0   = LP2_40(d_0)
  sub1   = AP1(LP2_62(d_1))                // 160 Hz with two bands, 105 Hz with three
  sub2   = AP1_130(LP2_80(d_2))
  sub    = ceiling(g_low*sub0 + g_high*sub1 + g_top*sub2);  meter = max(|sub|, meter*decay)
out    = bell_70(x + interp(sub))          // linear interpolation back to full rate; solo: out = interp(sub)
```

The divider is a peak follower (40 ms decay) plus a hysteresis comparator: it arms once the band dips below a quarter of its envelope and flips polarity at the next non-negative sample, so the divided waveform is continuous and beating partials or noise cannot re-trigger it inside one cycle. A band sine flipped every cycle carries 0.849 of its amplitude at f/2, 0.509 at 3f/2, 0.121 at 5f/2 and nothing at f; the post lowpass keeps f/2.

**Why one divider per band.** Each band has its own divider so a kick in the lower band and a bass note in the upper band divide independently (offline model: spurious content -21 dB below the subs on a 55 + 100 Hz mixture). A single divider over the whole range loses lock on that mixture.

**Why the allpasses and the parity slaving.** The dividers' flip-flops are left in an arbitrary relative state by the previous note. The SVF's LP and HP outputs are 180 degrees apart, which puts two adjacent subs in quadrature, so their sum is parity-independent once the post lowpasses' phase lags are matched: a first-order allpass on band 1 (160 Hz with two bands) and on band 2 (130 Hz) does that, and with three bands band 1's corner moves to 105 Hz. Bands 0 and 2 are not adjacent, so no allpass can hold them in quadrature, yet a 90 Hz note leaks into both at about -6 dB and left three free flip-flops the model showed up to 7 dB of history dependence. When bands 0 and 2 measure the same flip period (within 1/16) band 2's flip-flop takes its polarity from the sign of band 0's divided signal at the flip instant instead of toggling; a polarity set at a zero crossing is click-free, and different notes never match periods. Result, all eight parity states in the model: spread under 0.8 dB with two bands and under 1.1 dB with three, versus 4.7 and 7 dB without.

**Why decimate.** Nothing below the bell has content above 160 Hz, so the split, dividers, gates and ceiling run at about 8 kHz behind the 160 Hz lowpass (folded content at least 68 dB down) and the sub is linearly interpolated back (images below -99 dB). The cost per sample roughly halves against the full-rate kernel even with the new stages on, and the RP2040 Q28 precision improves by about 13 dB because the filter corners sit six times higher relative to the rate. The sub lags the dry signal by one low-rate period (0.125 ms) identically on every processed output; the dry signal is not delayed.

**Selectivity gate.** Per band, from the divider's own envelope: a 400 ms slow follower (30 ms rise) marks an attack when the fast envelope exceeds twice it; an attack or silence (below -60 dBFS) resets a hold counter. Percussive mode opens the gate at once and closes it over 100 ms once the counter passes the hold; sustained mode opens over 30 ms after the hold and shuts at once on an attack so a kick under a held note ducks. `depth` scales how far the gate closes. The weight multiplies the divided signal before the post lowpass, which smooths the steps. It cannot separate two sources sounding at once in one band (see the spec's edge cases).

**Sub ceiling.** A peak follower on the sub sum (200 ms release) drives a gain of ceiling / peak whenever the peak exceeds the ceiling, slewed with a 3 ms attack and 200 ms release. Steady-state level sits at the ceiling with no added distortion; a loud onset overshoots by a few dB for a few milliseconds.

**Linked pairs.** With `link_pairs` (default on) an S/PDIF pair whose two outputs are both eligible synthesizes one sub from the pair's mono sum and mixes it into both, each with its own bell. Independent per-channel dividers would otherwise be knocked into opposite polarity by any panned event, putting a centred bass note's sub out of phase between the speakers. The pair costs one kernel plus one bell.

**Alignment and divider parity across outputs.** Pure IIR, no delay lines. The decimation phase comes from one absolute sample counter on Core 0 (`sh_sample_ctr`, handed to Core 1 as `Core1EqWork.subharm_phase`), so every processed output gets the identical one-period sub lag whatever its processing history, and inter-output-slot sample alignment is untouched by construction (the CLAUDE.md inviolable guarantee). A zero-crossing divider has no absolute phase reference, so an output whose state was reset (mute, mask, link toggle) must not restart its divider against a running neighbour: the pass seeds a fresh output from its running pair partner, and a linked pair mirrors its state into the odd output every block, so an unlink continues both in step. Verified on the host: unlinking at arbitrary packets leaves the two subs at correlation +1.000.

### Platform Implementation

- **RP2350:** TPT SVF (topology-preserving transform) in single-precision float. The split lowpass and harmonic highpass share one Butterworth corner at the cutoff; the cutoff is always far below Fs/7.5, deep in SVF territory. The harmonic one-pole lowpass is a plain float one-pole. `PsybassCoeffs` holds SVF integrator coefficients (`lp_a1-3`, `hp_a1-3`, `hp_k`), one-pole coeffs, `drive`, NLD blend weights (`even_w`/`odd_w`), `harm_gain`, and `orig_delta`; per-output state is the SVF integrators plus the one-pole state (`PsybassOutputState`).
- **RP2040:** RBJ biquads (TDF2) scaled to Q28, processed with `fast_mul_q28()`. Range clamps keep every Q28 coefficient inside the representable +/-8.0 range (harmonics capped at +12 dB, drive at +18 dB = 7.94). The kernel **pre-clamps the low band to +/-1.0 before the drive multiply**: drive can be up to 7.94 and `fast_mul_q28` wraps sign past +/-8.0, so `drive * low` on an over-full-scale low band would wrap; since drive >= 1.0 the result is identical to clamping only the product. A second clamp bounds `d` so `1.5d - 0.5d^3` (which peaks at exactly 1.0) stays in range.

### Parameters

One global config (`PsybassConfig`) applied to the output channels selected by `output_mask`:

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| enabled | bool | 0/1 | false | Enable/disable the effect |
| cutoff_hz | float | 30-300 Hz | 80 | Speaker LF limit; harmonics generated from below it |
| harmonics_db | float | -24..+12 dB | 0 | Generated-harmonics mix level (`g_harm`); +12 cap holds Q28 headroom |
| drive_db | float | 0..18 dB | 6 | Odd-path cubic-clipper drive (18 dB = 7.94 linear, below the Q28 ceiling) |
| character_pct | float | 0..100 | 50 | Even<->odd harmonic blend (0 = even only/warm, 100 = odd only/aggressive) |
| original_db | float | -60..0 dB | 0 | Original low-band level (0 = untouched) |
| output_mask | uint16 | 0x0000-0xFFFF | 0xFFFF | Bit k: process output channel k |

### Coefficient Publish & Per-Output State

Follows the loudness/crossfeed module pattern:

- **Double-buffered publish.** `psybass.c` computes one shared `PsybassCoeffs` into the inactive buffer (`pb_coeff_bufs[2]`) and atomically publishes the pointer via `volatile const PsybassCoeffs *current_psybass_coeffs` (**NULL = disabled**, replacing a bypass flag), never mutating the buffer the pipeline is reading. Vendor SET handlers write `psybass_config` and raise `psybass_update_pending`; the main loop consumes the flag and calls `psybass_apply_config()`. Coefficients are also recomputed on rate change (`perform_rate_change()` raises the flag). Initial setup runs once in `core0_init()`.
- **Per-output state ownership.** `PsybassOutputState psybass_output_state[NUM_OUTPUT_CHANNELS]` lives in `psybass.c`; each output is only ever touched by the core that owns it in the current pipeline mode.
- **Skip-and-reset predicate.** An output runs psybass when coeffs are published AND its mask bit is set AND it is matrix-enabled AND not muted AND not carrying a siggen RAW test signal; otherwise its state is reset each packet (`psybass_reset_output_state()`) so re-entry starts clean. This reset-on-skip is wired at all six pipeline call sites: the single-core and dual-core output loops on both the RP2350 and RP2040 branches of `audio_pipeline.c`, and the two Core 1 EQ-worker output loops in `pdm_generator.c`. Psybass runs **pre-crossover** (it must see the low band before any highpass crossover removes it) and the disabled PDM output's state is also kept cleared in EQ_WORKER mode.
- **Two-core coherence.** Core 0 snapshots the coefficient pointer + `output_mask` once per packet and hands them to Core 1 through new `Core1EqWork` fields `psybass_coeffs`/`psybass_mask`, so both cores apply one consistent view for the whole packet (same single-view rationale as the loudness/crossfeed fields).
- **RAM cost.** Double-buffered coeffs plus one `PsybassOutputState` per output; a few hundred bytes.

### Persistence & Control

- **Wire format V23:** `WirePsybassParams` (24 bytes) is tail-appended to `WireBulkParams` (`enabled` + `reserved0` + `output_mask` + five floats). Bulk collect/apply copy it straight to/from `psybass_config` and raise the pending flag.
- **Preset slot V31:** the config is tail-appended to `PresetSlot` (struct grows 24 bytes; `SLOT_DATA_SIZE_V31`), gated on `slot->version >= 31` in `apply_slot_to_live()`. Older slots have no psybass data and load the disabled/all-outputs defaults; V21..V30 slots still validate via `slot_data_size_for_version()`. Factory defaults set it disabled with `PSYBASS_DEFAULT_*` values.
- **Vendor commands:** `0x30-0x3D` (SET/GET enable, cutoff, harmonics, drive, character, original, mask). Each SET clamps to the parameter range, writes `psybass_config`, and emits a `notify_param_write`. All SET commands except the mask raise `psybass_update_pending`; the mask (0x3C) is read live each packet (skipped outputs reset themselves), so it needs no recompute. See the Vendor Command Reference table.

---

## Subharmonic Synthesizer
*Last updated: 2026-09-04*

### Purpose

A dbx 120A style octave divider, extended. The effect listens to the program bass in 48 to 160 Hz in three bands (the third off by default), synthesizes a note exactly one octave below it (24 to 80 Hz) whose envelope follows the bass that produced it, and mixes it back in at a user-set level, with a gentle LF boost bell after the sum. Since 2026-09-04 it also offers a percussive / sustained selectivity gate, a sub ceiling limiter, linked stereo pairs, a runtime solo and a per-output sub meter, and everything below the bell runs at a decimated rate. It is the mirror image of psybass: psybass adds harmonics above the bass for speakers that cannot play the fundamental, subharm adds a real fundamental below the bass for systems that can. Both platforms. Full spec: `Documentation/Features/subharmonic_synth_spec.md`. Status: **HW-untested** (verified against an offline model of the kernel).

### Signal Flow (per output channel, in place)

```
s      = LP2_112(HP2_48(x))               // program bass band
band0  = LP2_72(s), band1 = HP2_72(s)     // one SVF: LP out and HP out (48-72 / 72-112 Hz)
d_b    = divider(band_b)                  // polarity flip once per cycle: octave down, amplitude tracks the band
sub0   = LP2_40(d_0)                      // strips the switching harmonics (3f/2, 5f/2 ...)
sub1   = AP1_160(LP2_62(d_1))             // allpass phase-aligns band 1 to band 0 (see below)
out    = bell_70(x + g_low*sub0 + g_high*sub1)
```

The divider is a peak follower (40 ms decay) plus a hysteresis comparator: it arms once the band dips below a quarter of its envelope and flips polarity at the next non-negative sample, so the divided waveform is continuous and beating partials or noise cannot re-trigger it inside one cycle. A band sine flipped every cycle carries 0.849 of its amplitude at f/2, 0.509 at 3f/2, 0.121 at 5f/2 and nothing at f; the post lowpass keeps f/2.

**Why two dividers.** Each band has its own divider so a kick in the lower band and a bass note in the upper band divide independently (offline model: spurious content -21 dB below the subs on a 55 + 100 Hz mixture). A single divider over the whole band loses lock on that mixture.

**Why the allpass.** The two dividers' flip-flops are left in an arbitrary relative state by the previous note. The SVF's LP and HP outputs are 180 degrees apart, which puts the two subs in quadrature, so their sum would be parity-independent if the post lowpasses (40 vs 62 Hz) had the same phase lag. They do not, and without correction the level of a note near the band boundary varied by up to 4.7 dB between plays in the offline model. A first-order allpass at 160 Hz on band 1 matches the lags within 5 degrees over 24 to 56 Hz and brings the spread under 0.8 dB. It costs one multiply per sample.

**Zero added latency.** Pure IIR, no delay lines, so inter-output-slot sample alignment is untouched by construction (the CLAUDE.md inviolable guarantee).

### Platform Implementation

The kernel exists once, in `subharm.c`, written against a number-type abstraction in `subharm.h` (`sh_num_t` with `sh_mul`, `sh_half`, `sh_twice`, `sh_quarter`, `sh_abs`, `sh_band_limit`, `sh_ratio`). Every filter is a TPT state-variable filter; the corners are far below Fs/7.5 at every rate, so the SVF form is both cheaper (three multiplies per stage once the lowpass update is folded to `v2 = ic2 + g v1`, with LP and HP for free) and more precise than a direct-form biquad.

- **RP2350:** `sh_num_t` is float; the helpers are plain float arithmetic.
- **RP2040:** `sh_num_t` is Q28 `int32_t`; `sh_mul` is `fast_mul_q28`. The SVF's low-frequency precision matters here: an additive rounding error in a TDF2 biquad at 48 Hz is amplified by roughly 1/omega^2 (about 25 000x) into a DC offset, whereas in the TPT SVF it is amplified by roughly 1/g (about 300x), so the SVF holds a 30 dB better DC floor with the truncating multiplier. `sh_input_limit` clamps the sub path's input to +/-3.0, `sh_band_limit` each band signal to +/-1.0 before its divider, `sh_band_out_limit` each band's scaled sub to +/-2.5 and `sh_sub_limit` the sub sum to +/-2.0, with the boost ceiling at +6 dB, so no stored value can wrap `fast_mul_q28` past +/-8.0 on inputs up to +9.5 dBFS even with three bands at +12 dB (the dry path is not clamped). `sh_ratio` (the ceiling gain) normalizes the divisor to 16 bits and uses one 32-bit hardware divide, so the RAM kernel never calls a libgcc 64-bit divide or clz in flash. The host-compiled Q28 kernel matches the float kernel to about -102 dBFS with every stage on.

The block kernel `subharm_process_block` is an out-of-line `DSP_TIME_CRITICAL` function in `subharm.c`, shared by both cores, rather than a header inline like psybass: inlining it at the call sites cost several KB of RAM code. Its full-rate loop keeps the anti-alias SVF, interpolator and bell state in registers; the low-rate work (`subharm_low_rate_tick`, `sh_band_tick`, `sh_select_gain`) is out-of-line RAM code working through the state struct, where the memory traffic is cheap and inlining three band ticks into the loop cost 4 KB. The RAM code totals about 3 KB. A skipped band (level at the -30 dB floor), gate (mode ALL), ceiling (0 dB) or bell (0 dB, or any setting while soloed) has its state cleared once per block so re-enabling it is transient-free.

### Parameters

One global config (`SubharmConfig`) applied to the output channels selected by `output_mask`:

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| enabled | bool | 0/1 | false | Enable/disable the effect |
| low_db | float | -30..+12 dB | 0 | 24-36 Hz sub level; -30 = band off (skipped) |
| high_db | float | -30..+12 dB | 0 | 36-56 Hz sub level; -30 = band off (skipped) |
| boost_db | float | 0..+6 dB | 0 | LF boost bell (70 Hz, Q 0.9) after the sum; 0 = stage skipped |
| output_mask | uint16 | 0x0000-0xFFFF | 0xFFFF | Bit k: process output channel k |
| top_db | float | -30..+12 dB | -30 (off) | 56-80 Hz sub level from 112-160 Hz program; -30 = band off |
| select_mode | uint8 | 0..2 | 0 (all) | Favour all / percussive / sustained bass material |
| select_depth | float | 0..100 % | 100 | How far the unfavoured material is gated; ignored at mode 0 |
| select_hold_ms | float | 50..400 ms | 150 | Span the selectivity decision is made over; ignored at mode 0 |
| ceiling_db | float | -40..0 dBFS | 0 (off) | Soft limit on the synthesized sub before it is mixed in |
| link_pairs | bool | 0/1 | true | Synthesize each output pair's sub from the pair's mono sum |
| solo | bool | 0/1 | false | Monitoring: masked outputs carry the sub only. **Runtime state**, never in a preset slot or on the wire, so no saved configuration can boot with the program signal removed. A preset load leaves it alone; only a factory reset clears it. |

The kernel behaviour behind the third band, the selectivity gate, the ceiling and the pair link is in the Signal Flow section above; the rows are the host-facing contract.

### Headroom Reading

`REQ_GET_SUBHARM_HEADROOM` (0x1A) returns the worst-case gain of the current configuration in dB: the preamp headroom a host must free so the effect cannot clip. `subharm_headroom_db()` is a pure function of the config (not the rate or the mask) evaluated on each GET, so a host can SET and immediately GET without racing the main-loop recompute. It scans an eighth-octave grid from 16 to 362 Hz and sums, as amplitudes, the direct tone through the bell plus each of the three bands' divided components (n = 1, 3, 5, 7 at n f/2) through that band's post lowpass and the bell, using analog-prototype magnitudes; with a ceiling on it bisects on the input level for the smallest headroom at which an input that far below full scale cannot exceed it, because the ceiling is absolute and stops limiting once the preamp is lowered. Selectivity and the pair link only ever lower the sub, so they never raise the bound. The bound is conservative by 1 to 2 dB on real tones and never below the measured peak (offline model: one band at 0 dB 4.2 vs 3.3 dB measured; both bands 6.3 vs 4.4; everything at +6 dB 13.5 vs 11.2). Because the divider preserves band amplitude and every other stage is linear, a preamp cut of the reported amount is an exact correction. This is the first contributor to the planned per-output headroom budget; the budget will sum the worst-case boost of every stage in an output's chain.

### Coefficient Publish & Per-Output State

Follows the psybass module pattern:

- **Double-buffered publish.** `subharm.c` computes one shared `SubharmCoeffs` into the inactive buffer and atomically publishes the pointer via `volatile const SubharmCoeffs *current_subharm_coeffs` (**NULL = disabled**). Vendor SET handlers write `subharm_config` and raise `subharm_update_pending`; the main loop consumes the flag and calls `subharm_apply_config()`. Coefficients are also recomputed on rate change (`perform_rate_change()` raises the flag). Initial setup runs once in `core0_init()`.
- **Per-output state ownership.** `SubharmOutputState subharm_output_state[NUM_OUTPUT_CHANNELS]` (160 bytes each) lives in `subharm.c`; each output is only ever touched by the core that owns it in the current pipeline mode. `subharm_reset_output_state()` zeroes everything except the ceiling gain, which idles at unity so a fresh output is not faded in over the limiter's release; an `active` flag marks a state as fresh until its first block, when the partner seeding above applies.
- **Per-packet pass.** `subharm_process_outputs(coeffs, mask, flags, first, last, buf_out, n)` replaced the per-output snippets at all six pipeline call sites (single-core and dual-core loops on both platform branches of `audio_pipeline.c`, and both Core 1 EQ-worker loops in `pdm_generator.c`). It runs once per packet over that core's output range, right after crossfeed and before the per-output loop, so it still precedes psybass. Eligibility per output is the psybass predicate: coeffs published AND mask bit set AND matrix-enabled AND not muted AND not carrying a siggen RAW signal. With the link flag, an even output and its partner that are both eligible and both inside the range run as a linked pair (never across the core boundary: pair 0 is core 0's, pairs 1+ are core 1's, and the PDM output is never paired); the partner's state keeps only its bell and meter. Everything else runs singly or is reset. The disabled PDM output's state is kept cleared in EQ_WORKER mode.
- **Two-core coherence.** Core 0 snapshots the coefficient pointer, `output_mask`, the link/solo flags (`subharm_flags_snapshot()`) and the packet's decimation phase (`subharm_packet_phase()`) once per packet into the `Core1EqWork` fields `subharm_coeffs`/`subharm_mask`/`subharm_flags`/`subharm_phase`. Link and solo, like the mask, are read live and need no coefficient recompute.
- **RAM cost.** About 1.2 KB on RP2040 and 1.9 KB of `.bss` on RP2350 (state array plus two coefficient buffers), plus about 3 KB of RAM-resident code.
- **CPU cost.** Per processed output about 12 multiplies per sample with the default two bands and about 16 with three bands, selectivity and ceiling on, against 25 for the original full-rate kernel: one SVF, the interpolator and the bell at full rate, everything else at one sixth of the rate. A linked pair costs one kernel plus one bell. Skipped outputs, bands, gate, ceiling and bell cost nothing beyond a state clear.

### Persistence & Control

- **Wire format V29:** `WireSubharmParams` (16 bytes) is tail-appended to `WireBulkParams` at offset 5944 (`enabled` + `reserved0` + `output_mask` + three floats; total 5960 bytes). Bulk collect/apply copy it straight to/from `subharm_config` and raise the pending flag.
- **Wire format V30:** the section grows to 36 bytes by tail-appending `top_db`, `select_depth`, `select_hold_ms`, `ceiling_db`, `select_mode`, `link_pairs` and two reserved bytes, taking the packet total to **5980 bytes**. The section offset stays 5944. Apply clamps `select_mode`; the floats are clamped downstream. `solo` is not on the wire, so a bulk apply never changes it.
- **Preset slot V36:** the config is tail-appended to `PresetSlot` (struct grows 16 bytes; `SLOT_DATA_SIZE_V36`), gated on `slot->version >= 36` in `apply_slot_to_live()`. Older slots load the disabled/all-outputs defaults; V21..V35 slots still validate via `slot_data_size_for_version()`. Factory defaults set it disabled with `SUBHARM_DEFAULT_*` values.
- **Preset slot V37:** the V30 fields are tail-appended in turn (struct grows a further 20 bytes; `SLOT_DATA_SIZE_V37`), gated on `slot->version >= 37`. V36 slots load the `SUBHARM_DEFAULT_*` values for them. `solo` is not stored.
- **Vendor commands:** `0x10-0x1F`, `0x2C-0x2F` and `0xA9-0xAE` (SET/GET enable, low, high, boost, mask, top, selectivity mode/depth/hold, ceiling, link, solo; GET headroom and sub meter). 0x10-0x1F were the first application commands allocated inside 0x00-0x1F. Each SET clamps to the parameter range, writes `subharm_config`, and emits a `notify_param_write`. The mask, link and solo SETs do **not** raise `subharm_update_pending` (the pipeline reads them live each packet); every other SET does. Solo has no wire offset, so it emits no notification. See the Vendor Command Reference table.
- **Sub meter:** `REQ_GET_SUBHARM_METER` (0x1F) returns `NUM_OUTPUT_CHANNELS` uint16 LE values from `subharm_meter_u16()`, a decaying peak of the synthesized sub per output on the same 0..32767 scale as `SystemStatusPacket.peaks`. 18 bytes on RP2350, 10 on RP2040.
- **Control Surfaces:** caps v14 nouns 57-60 (`SUBHARM`, `SUBHARM_LOW`, `SUBHARM_HIGH`, `SUBHARM_BOOST`) and caps v15 nouns 61-67 (`SUBHARM_TOP`, `SUBHARM_SELECT`, `SUBHARM_DEPTH`, `SUBHARM_HOLD`, `SUBHARM_CEILING`, `SUBHARM_LINK`, `SUBHARM_SOLO`). Caps v16 (2026-09-05) widens the band-level nouns to -30..+12 dB.

---

## Tube Preamp Emulation
*Last updated: 2026-09-28 (output stage on by default at 95 Hz; default drive -12 dB; drive floor lowered to -30 dB; RP2040 drive and shaper-output terms now in per-coefficient-set shifted domains `t_shift` / `s_shift`; kernel 1,196 B; 2026-09-20: transformer split/saturation/HF rolloff replaced by a damping-factor output stage: bell at the speaker resonance plus a fixed top shelf, parameters 10 and 11 now xfmr_damping and xfmr_res_hz, 14 parameters in all; per-output saturation meter and its 0x81 command removed; 2026-09-19: drive range -6..24 dB with automatic makeup gain, level-neutral defaults, rescaled tube rows, RP2040 shaper-output clamp; RP2350 kernel rewritten branch-free, 660 B; 2026-09-18: RP2040 headroom rules and kernel size corrected; multiply counts; host-model result; indexed-SET coercion and notification rules)*

### Purpose

Valve-style harmonic colour, compression and an optional output-stage tonal signature, applied per output channel. The module sits in the same family as Psychoacoustic Bass and the Subharmonic Synthesizer. One global configuration, a 16-bit output mask, shared double-buffered coefficients, per-output state, zero added latency.

It is a characterful approximation, not a circuit simulation. A single static waveshaper with an adjustable operating point (bias), separate positive and negative knees (asymmetry), a knee-hardness blend and a slow supply-sag envelope covers the audible differences between the popular preamp and power tubes. A tube-type selector loads a row of those four character values at SET time, so the kernel costs the same for every type. Both platforms. Module: `firmware/DSPi/tube.c` / `tube.h`. Full spec: `Documentation/Features/tube_preamp_spec.md`. Status: **HW-untested** (verified against an offline model of the kernel).

### Signal Flow (per output channel, in place)

```
m_eff = m - m * depth * env                 // supply sag; env follows |t|
t     = m_eff * x + b                       // b = operating point in knee units
t     = (t < 0) ? t * ratio_n : t           // negative knee offset
t     = clamp(t, -1, +1)                    // the positive knee is fixed at t = 1
v     = t (c1 + t^2 (c3 + t^2 c5)) * (t >= 0 ? s_p : s_n) - v0
y     = DC_block(v)                         // one-pole high pass at 5 Hz
if output stage:
    y  += bl_m1 * bell_bp(y)                // TPT SVF peaking, Q 0.707 at xfmr_res_hz
    sh_lp += sh_a * (y - sh_lp)             // one-pole at 2.5 kHz
    y  += sh_g * (y - sh_lp)                // first-order top shelf
out   = (1 - mix) * x + mix * trim * y
env  += (|t| - env) * (|t| > env ? a_att : a_rel)
```

- **The shaper.** `p(t) = t (c1 + t^2 (c3 + t^2 c5))` blends a cubic soft knee at hardness 0 into a quintic hard knee at hardness 100. Both reach exactly 1 at t = 1 with zero slope, so the clamp is continuous in value and in slope at every hardness. `s_p` and `s_n` normalise the small-signal gain to unity, so neither hardness nor drive changes the level of clean material.
- **Drive and automatic makeup gain.** The output scales carry the makeup factor themselves. `s_p = 1 / (c1 m)` and `s_n = kn / (c1 m)`, where `m` is the linear drive. Small-signal gain is therefore unity at every drive setting, and the drive knob moves the knee relative to the signal rather than moving the level. Turning drive up adds harmonics without getting louder, which is what makes A/B comparison honest. Drive runs from **-30 to +24 dB**, and at -30 dB the knee sits 30 dB above full scale, so a full-scale input stays in the nearly linear part of the curve. The -30 dB floor is an RP2040 fixed-point limit rather than a musical one. `s_p + s_n` reaches 105 there, which is the most the shifted shaper-output domain holds (see the RP2040 notes below).
- **Even harmonics from the bias.** `b` shifts the operating point along the curve, so second-harmonic content rises with signal level the way it does in a real single-ended stage. `v0` is the shaper's output at rest and is subtracted, so enabling the effect produces no step. The DC blocker then removes the level-dependent offset that asymmetric clipping creates.
- **Supply sag.** A one-pole follower of how hard the stage is being driven into the knee pulls the drive down slowly. The rectifier selector presets the depth scale and the attack and release times. Effective depth is capped at 0.9 so the gain never reaches zero, and a solid-state rectifier disables the stage outright.
- **Output stage.** What a valve amplifier audibly does to a loudspeaker is not saturation, it is a low damping factor. The stage models a source impedance of `Zn / df` driving a speaker whose impedance rises to **4x nominal at its resonance** and **2x nominal at the top of the band**, so the terminal voltage lifts by `4 (df + 1) / (4 df + 1)` at the resonance and `2 (df + 1) / (2 df + 1)` in the treble. At `df = 1` that is **+4.1 dB** of bass bump and **+2.5 dB** of top lift; at `df = 20` it is +0.3 dB and +0.2 dB, which is what a solid-state amplifier measures. The bump is a Cytomic TPT SVF peaking section with Q fixed at 0.707, centred on `xfmr_res_hz`, and the lift is a first-order high shelf whose corner is fixed at 2.5 kHz (voice-coil inductance, not a user control). There is no saturator and no HF rolloff. With the stage off its state is zero and the kernel skips it on a flag hoisted out of the sample loop, rather than running pass-through coefficients.
- **Zero added latency.** Every stage is memoryless or a one-pole IIR and the dry path is never delayed, so inter-output-slot sample alignment is untouched by construction (the CLAUDE.md inviolable guarantee).

### Tube Types and Rectifiers

`tube_type` 1..16 names a row of `bias_pct`, `asym_db`, `hardness_pct` and `sag_pct`, running from the 12AX7 to the 300B. Setting the type copies its row into those four parameters, stores the type and notifies every field it changed. Setting any of those four to a value different from the one stored resets the type to 0 (Custom) and notifies that byte, while a SET that lands on the value already there leaves the type alone. Bulk apply and preset load restore the stored fields verbatim and never run the row lookup, so a preset saved as a 12AX7 reloads with the voicing it was saved with even if the row table changes in a later firmware. Push-pull power rows carry zero bias and zero asymmetry, because a push-pull stage cancels even harmonics by construction; their character comes from hardness, sag and the output stage.

The rows are scaled for a clean default rather than for maximum character, so drive is what brings the voicing up. Relative to the first implementation, bias is roughly a third of what it was and sag roughly half, while asymmetry and hardness are unchanged.

| Type | Name | bias_pct | asym_db | hardness_pct | sag_pct |
|------|------|----------|---------|--------------|---------|
| 1 | 12AX7 / ECC83 | 10 | 3 | 40 | 15 |
| 2 | 5751 | 8 | 3 | 35 | 12 |
| 3 | 12AT7 / ECC81 | 5 | 2 | 55 | 10 |
| 4 | 12AY7 | 7 | 4 | 25 | 15 |
| 5 | 12AU7 / ECC82 | 5 | 5 | 20 | 8 |
| 6 | 6SN7 | 7 | 6 | 15 | 10 |
| 7 | 6SL7 | 10 | 3 | 30 | 15 |
| 8 | 6DJ8 / ECC88 / 6922 | 3 | 2 | 60 | 5 |
| 9 | EF86 / 6267 | 2 | 0 | 75 | 12 |
| 10 | 6SJ7 | 3 | 1 | 65 | 15 |
| 11 | EL84 / 6BQ5 | 0 | 0 | 50 | 25 |
| 12 | EL34 | 0 | 0 | 60 | 30 |
| 13 | 6L6 / 5881 | 0 | 0 | 55 | 18 |
| 14 | 6V6 | 0 | 0 | 35 | 30 |
| 15 | KT88 / 6550 | 0 | 0 | 45 | 10 |
| 16 | 300B / 2A3 | 12 | 6 | 10 | 12 |

`rectifier` 0..3 selects solid state (sag off), GZ34, 5U4 or 5Y3. Each is a row of sag depth scale, attack time and release time.

### Platform Implementation

The kernel exists once, in `tube.c`, written against a number-type abstraction in `tube.h` (`tb_num_t`, with `TB_ZERO` and `TB_ONE`). `tube_process_output_block()` is an out-of-line `DSP_TIME_CRITICAL` function shared by all six call sites, so its RAM text is paid once. It measures 660 B on RP2350 and 1,196 B on RP2040. The RP2350 body is branch-free: every sign-dependent select is an fmaxf/fminf split (VMAXNM/VMINNM, bit-exact with the branchy form) because a VCMP+VMRS pair or a taken branch costs the M33 more than the arithmetic it skips; the first build with `if` selects had 16 compare pairs and 26 branches per loop and metered about 2 % CPU per output at 48 kHz; the rewrite has none and 5 and meters just over 1 % per output (2026-09-19), close to the arithmetic floor. The output-stage arm is unswitched into two loop bodies.

- **RP2350:** `tb_num_t` is float and the kernel is plain single-precision arithmetic.
- **RP2040:** `tb_num_t` is Q28 `int32_t` and every multiply is `fast_mul_q28`. Two shifts, chosen once per coefficient set, keep the drive range inside the Q28 budget. `m`, `sagk` and `bias` are carried in **Q(28 - `t_shift`)**, where `t_shift` (1..4) is the smallest value with `m < 1.75 x 2^t_shift`, so the drive product stays under 7.5. It never drops to 0, because the +/-4 knee-unit pre-clamp times a ratio of 3.98 would overflow there. `s_p`, `s_n` and `v0` are carried in **Q(28 - `s_shift`)**, where `s_shift` (0..4) is the smallest value with `s_p + s_n < 7.5 x 2^s_shift`. The shaper output is formed and clamped in that domain and then shifted back to Q28. Every other coefficient is Q28 and is verified against the +/-8.0 representable range. The negative-knee ratio is at most 3.98 and the wet weight at most 3.98; the output stage's own coefficients are small by construction, with the bell mix `bl_m1` at most 0.67 and the shelf lift `sh_g` at most 0.33, both at `df = 1`. The makeup gain puts `s_n` at up to **84**, at the -30 dB drive floor with the negative knee 12 dB later, and `s_p + s_n` at 105. That fits `s_shift` 4 (limit 120) and is what sets the floor. From -6 dB drive up, `s_shift` is 0, as in the original kernel.

  The constraint that shapes the whole kernel is `fast_mul_q28`'s own. It splits each operand into 16-bit halves and sums the cross products in an `int32`, so it needs the **sum of the two operand magnitudes** to stay below 8.0. A small product is not enough. Every operand is therefore bounded at the point of use.

  | Clamp | Limit | Where |
  |-------|-------|-------|
  | Shaper input | +/-4.0 | before the drive multiply |
  | Driven value | +/-4 knee units | before the negative-knee ratio multiply |
  | Shaper output `v` | +/-3.4 (`TUBE_Q28_Y_LIM`) | after the `v0` subtraction, applied in the `s_shift` domain before the shift back to Q28 and the DC blocker. `s_n` reaches 84 at -30 dB drive, so the raw value can overshoot |
  | DC-blocker output copy | +/-3.4 (`TUBE_Q28_Y_LIM`) | state keeps the true value |
  | Bell input | +/-2.5 (`TUBE_Q28_BELL_IN`) | before the SVF; keeps its `y - ic2` difference term under 6.5 |
  | Bell output | +/-3.4 (`TUBE_Q28_Y2_LIM`) | before the top shelf, so the shelf difference stays under 6.8 |
  | Wet signal | `wet_lim` = clamp((7.5 - 4 `dry_w`) / `wet_w`, 0, 3.4) | before the output mix |

  The shaper-output clamp is what keeps the DC blocker in range. Its output is bounded by twice the largest `v` it can be fed, so clamping `v` to 3.4 holds that at 6.8, whereas an unclamped `v` could reach many times full scale at low drive. This clamp is RP2040 only. The RP2350 float kernel has no equivalent, because nothing there needs bounding.

  The dry term in `wet_lim` uses the +/-4.0-clamped input, so the final sum can never exceed 7.5. The one lossy corner is an input beyond +12 dBFS at 0 dB drive with maximum bias and asymmetry, where the negative half saturates slightly early. Everywhere else the clamps land on material the +/-1 knee clamp would flatten anyway.

A host model that emulates `fast_mul_q28` exactly (16-bit halves, `int32` partial sums, wrap detection) reports **zero integer overflows** over 404 parameter combinations times two output-stage states, with hot (+6 dBFS) and realistic stimuli. The sweep was re-run against the output-stage kernel on 2026-09-20 with the same result: 404 combinations, both stage states, zero integer overflows, worst in-range difference about -75 dBFS. On 2026-09-28 the drive-floor change was checked with a wider sweep of 23,760 combinations (drive -30 to +24 dB, every character, mix and trim extreme), both stage states, with +6 dBFS sine, +12 dBFS square and in-range stimuli. It found zero integer overflows. The worst in-range difference was -72.3 dBFS at extreme corners and the median was -91 dBFS. The previous kernel measured -73.6 dBFS worst on the same sweep, so the shifts cost no accuracy. Its worst in-range difference from the float kernel is about **-75 dBFS**, which is the helper's truncation floor rather than a kernel error. Small-signal gain is unity at every hardness, and silence in gives exactly zero out for all 16 tube rows.

### Parameters

One global config (`TubeConfig`) applied to the output channels selected by `output_mask`. Every parameter is addressed by a small integer index through one SET/GET pair, and on the wire every value is a little-endian IEEE 754 float32, the booleans and enums included. All SETs clamp, so a GET after a SET returns the clamped value.

| Index | Parameter | Type | Range | Default | Description |
|-------|-----------|------|-------|---------|-------------|
| 0 | enabled | bool | 0/1 | false | Enable/disable the effect |
| 1 | output_mask | uint16 | 0x0000-0xFFFF | 0xFFFF | Bit k: process output channel k; read live, no recompute |
| 2 | tube_type | enum | 0..16 | 1 (12AX7) | 0 = Custom; 1..16 each load a character row |
| 3 | drive_db | float | -30..24 dB | -12 | Knee position, with automatic makeup gain; at 0 dB a full-scale input just reaches the knee, at -12 dB the knee sits 12 dB above full scale |
| 4 | bias_pct | float | -100..+100 % | 10 | Operating point, `b = bias_pct / 200` |
| 5 | asym_db | float | -12..+12 dB | 3 | How much later the negative half reaches its knee |
| 6 | hardness_pct | float | 0..100 | 40 | Cubic (0) to quintic (100) knee blend |
| 7 | sag_pct | float | 0..100 % | 15 | Supply sag depth before the rectifier scale |
| 8 | rectifier | enum | 0..3 | 1 (GZ34) | 0 = solid state, which turns sag off |
| 9 | xfmr_enabled | bool | 0/1 | true | Output stage (damping-factor bell plus top shelf) |
| 10 | xfmr_damping | float | 1..20 | 2 | Damping factor; 1 gives +4.1 dB at the resonance and +2.5 dB on top, 20 is near flat. A stage setting, not a tube-row field: editing it leaves `tube_type` alone |
| 11 | xfmr_res_hz | float | 30..150 Hz | 95 | Speaker resonance; the bell's centre frequency |
| 12 | mix_pct | float | 0..100 % | 100 | Dry/wet blend; the dry path is the untouched input |
| 13 | trim_db | float | -12..+12 dB | 0 | Level applied to the wet path only |

Defaults for indices 4 to 7 are the 12AX7 row. Indices 10 and 11 are not in any row, so a tube-type change never moves them.

**Why the defaults changed (2026-09-19).** The first set added +6 dB of gain and about 6 % THD at -12 dBFS, so switching the module on was a loudness change as much as a tonal one. The new default is level-neutral and measures 0.18 % THD at -20 dBFS, 0.50 % at -12 dBFS and 3.6 % at 0 dBFS on a 100 Hz sine in the host model, second-harmonic led.

**Near-transparent default (2026-09-28).** At the -6 dB floor no tube row could be transparent. Even with every character value at zero, the cubic term alone gave 2.2 % THD at 0 dBFS, because a full-scale peak still reached halfway to the knee. The floor is now -30 dB, where the 12AX7 row measures 0.029 % THD at -12 dBFS and 0.12 % at 0 dBFS and the push-pull rows about 0.014 % at 0 dBFS. The default drive is -12 dB, a subtle setting rather than a transparent one. There the 12AX7 row measures 0.092 % THD at -20 dBFS, 0.23 % at -12 dBFS and 1.25 % at 0 dBFS, with 0.1 to 0.4 dB of level change. The other rows range from 0.05 % to 0.22 % at -12 dBFS. Third-order products fall about 12 dB and second-order about 6 dB for every 6 dB of drive removed, so the biased rows keep a faint second harmonic. Hardness and asymmetry also act below the knee, through the cubic term's size and the negative half's scaling, but both fade with drive squared.

### Coefficient Publish & Per-Output State

Follows the psybass module pattern:

- **Double-buffered publish.** `tube.c` computes one shared `TubeCoeffs` (about 21 values) into the inactive buffer and atomically publishes the pointer via `volatile const TubeCoeffs *current_tube_coeffs` (**NULL = disabled**). Vendor SET handlers go through `tube_set_param()`, which clamps, writes `tube_config` and raises `tube_update_pending`; the main loop consumes the flag and calls `tube_apply_config()`. Coefficients are also recomputed on rate change (`perform_rate_change()` raises the flag). Initial setup runs once in `core0_init()`.
- **Per-output state ownership.** `TubeOutputState tube_output_state[NUM_OUTPUT_CHANNELS]` lives in `tube.c` and is 24 bytes per output, holding the sag envelope (`env`), the DC-blocker input and output (`dc_x1`, `dc_y1`), the bell SVF integrators (`bl_ic1`, `bl_ic2`) and the top-shelf one-pole state (`sh_lp`). Each output is only ever touched by the core that owns it in the current pipeline mode.
- **Skip-and-reset predicate.** An output runs tube when coeffs are published AND its mask bit is set AND it is matrix-enabled AND not muted AND not carrying a siggen RAW test signal. Otherwise its state is reset each packet (`tube_reset_output_state()`) so re-entry starts clean. The predicate is wired at all six pipeline call sites, the single-core and dual-core output loops on both platform branches of `audio_pipeline.c` and the two Core 1 EQ-worker output loops in `pdm_generator.c`, and the disabled PDM output's state is kept cleared in EQ_WORKER mode.
- **Two-core coherence.** Core 0 snapshots the coefficient pointer and `output_mask` once per packet and hands them to Core 1 through the `Core1EqWork` fields `tube_coeffs` and `tube_mask`, so both cores apply one consistent view for the whole packet (the same single-view rationale as the psybass and subharm fields).
- **Chain position.** Tube runs per output, post-matrix, after crossfeed, after the subharmonic synthesizer and after psybass, and before the crossover, per-output PEQ, gain, loudness and delay. Running after the two bass modules is deliberate, so the stage saturates the enhanced bass rather than the other way round. Pre-crossover placement puts the saturator where a real preamp sits ahead of an active crossover, so a subwoofer output saturates the full-band program and then low-passes the result while a tweeter output keeps the harmonics the bass generated. Because tube runs pre-gain, its character does not change with volume.
- **RAM cost.** 552 B of `.bss` on RP2350 (216 B of per-output state, two `TubeCoeffs` buffers and the config) and 452 B on RP2040 (120 B of state), plus the shared kernel's 660 B and 1,196 B of RAM text. See "Memory Layout".
- **CPU cost.** About 30 FP ops per sample per processed output on RP2350 with the output stage off and about 50 with it on (the bell is 13, the shelf 6). The branch-free shaper-only build metered just over 1 % per output at 48 kHz on 2026-09-19; the output-stage figure is unmeasured. RP2040 costs **10 `fast_mul_q28` per sample** in the base path, plus 1 with sag on, plus 1 on the negative half of the waveform, plus 7 with the output stage on, so 12 to 19 is the typical range, unmeasured. With the effect disabled the published pointer is NULL and a processed output costs one pointer test.

### Persistence & Control

- **Wire format V31:** `WireTubeParams` (48 bytes) is tail-appended to `WireBulkParams` at offset 5980, taking the packet total to **6028 bytes**. The section carries `enabled`, `tube_type`, `rectifier`, `xfmr_enabled`, `output_mask`, two reserved bytes, nine floats and one reserved float that holds the size at 48. V31 was revised in place when the output stage replaced the transformer stage, before any release carried it, so there is only one V31 layout. Bulk collect copies `tube_config` straight out and zeroes both reserved fields. Bulk apply copies the fields verbatim, clamps `tube_type` and `rectifier` because the coefficient tables index on them, leaves the floats to be clamped downstream and raises the pending flag. It never runs the tube-type row lookup, which would overwrite a saved Custom voicing.
- **Preset slot V38:** the same fourteen values plus the reserved float are tail-appended to `PresetSlot` (struct grows 48 bytes; `SLOT_DATA_SIZE_V38`), revised in place with V31 and gated on `slot->version >= 38` in `apply_slot_to_live()`. V21..V37 slots still validate via `slot_data_size_for_version()` and load the `TUBE_DEFAULT_*` values with the effect off. Factory reset applies the same defaults. The pending flag is raised on both branches, so a load that turns the effect off also unpublishes its coefficients.
- **Vendor commands:** `0x3E` and `0x3F` are one indexed SET/GET pair covering every parameter, with the index in the low byte of wValue and a 4-byte float32 LE payload. Adding a parameter later therefore costs no opcode. An out-of-range index makes the SET a no-op and the GET STALL, so a host can feature-detect the module from index 0 alone; a SET shorter than four bytes is a no-op, and a NaN is ignored while the SET still reports success. Coercion differs by type. The booleans (indices 0 and 9) are **non-zero-is-true**, not rounded, so 0.4 enables. The enums and `output_mask` are **rounded to nearest and then clamped**. `tube_set_param()` owns the clamping and the pending flag, and it calls `notify_param_write` on every SET whether or not the stored value changed (deduplicating a no-change write is the notify layer's job, not the module's), so a tube-type SET that names a row always issues five notifications (four knobs plus the type byte) and a knob SET that drops the type to Custom emits two. Only `output_mask` skips the recompute flag, because the pipeline reads it live each packet. See the Vendor Command Reference table.
- **Control Surfaces:** caps v19 nouns 70-73 (`TUBE`, `TUBE_DRIVE`, `TUBE_TYPE`, `TUBE_MIX`). All four dispatch through `REQ_SET_TUBE_PARAM` with the parameter index in wValue, so all four carry a float32 payload, the bool and the enum included. Display labels are "Tube", "Tube Drive", "Tube Type" and "Tube Mix", and the type noun has its own 17-entry short-form label table.

### Interactions and Edge Cases

- **Headroom.** The shaper output is bounded by `max(s_p, s_n)`, at most 5.3 at the -6 dB drive floor with the negative knee 12 dB later, and the output stage adds at most 4.1 dB of bell and 2.5 dB of shelf on top of it. Small-signal gain is unity at every drive, so the wet path can exceed 0 dBFS only through `trim_db` or through material already near the knee. On RP2040 that bound is clamped to 3.4 before the DC blocker; RP2350 carries it in float.
- **Aliasing.** The shaper runs at the native rate with no oversampling. Harmonic content is bounded to fifth order below the clamp, so aliasing is audible only on bright material at heavy drive at 44.1 and 48 kHz, and is negligible at 96 kHz.
- **RAW signal generator outputs** bypass the effect and reset its state, as for psybass.
- **Masking and alignment.** Masking the effect per output changes that output's phase response only through one-pole IIR stages, the same category as a PEQ band, and never its sample alignment.

---

## Output Limiter
*Last updated: 2026-09-28 (Control Surfaces nouns 74-78; 2026-09-26: link groups gang their settings; earlier: RAW test signals are limited, persistence follows `output_config_mode`, directory V22)*

### Purpose

A brickwall peak limiter on every output, for protecting amplifiers, drivers and downstream converters and for keeping the output converter out of hard clipping. Every output has its own enable, threshold (-30..0 dBFS), release (10..1000 ms) and link group (0 = unlinked, 1-4). Outputs in the same group act as one limiter: their enable, threshold and release are ganged, and every member applies the deepest gain reduction any member needs. Both platforms. Module: `firmware/DSPi/limiter.c` / `limiter.h`. Full spec, including the no-overshoot proof: `Documentation/Features/output_limiter_spec.md`. Status: **HW-untested** (the real `limiter.c` is verified on the host against stub headers, both platform variants; see "Verification").

### Algorithm

- **Block-rate gain, two-block lookahead.** The gain is decided once per `LIMITER_BLOCK` (16) samples and the audio is delayed `LIMITER_DELAY` (32) samples. When input block `j` completes, each output computes its peak `p_j`, the hard target `h_j = min(1, T / p_j)` and its envelope `E_j = min(E_(j-1) * r, 1, h_(j-1), h_j)`. The output block emitted next is `j-1`, and its gain ramps linearly from `E_(j-1)` to `E_j` (or the group minimums). Both ends contain `h_(j-1)`, so no sample of block `j-1` exceeds the threshold. The delay has to be two blocks because the ramp for block `k` ends at a target that needs the peak of block `k+1`.
- **Attack** is one block (0.33 ms at 48 kHz), fixed, and always complete before the peak. **Release** multiplies the envelope by `r = exp(B / (tau * fs))` per block, so the gain recovers at a constant 8.69 dB per release time.
- **Block phase** comes from one stream-wide counter (`lm_pos`, mod 32) shared by every output, so all outputs decide on the same samples and a ring segment never straddles the ring wrap.
- **RP2350** runs it in float. `h = T / max(p, T)` is exactly 1.0 below threshold, so detection needs no compare. **RP2040** keeps levels as raw Q28 samples, where output full scale is 2^29 (the `>> 6` output conversion), and gains in Q30. `h` is one 32/32 hardware divide, `(T << 1) / ceil(p >> 14)`, giving Q15 with the denominator rounded up so `h` rounds down. The release is one 64-bit multiply per block. The per-sample apply is a two-multiply Q15 floor multiply, and the ramp step is floored by an arithmetic shift, so the fixed-point ramp never rises above the float one.

### Pipeline Placement and Cores

- **Chain position.** Per output, after gain and loudness and before the per-output delay lines, so the threshold is the absolute level leaving the device and user time-alignment delays are untouched.
- **Hooks.** `limiter_packet_begin()` runs on Core 0 before the Core 1 dispatch. It takes the per-packet snapshot (published coefficients, the processed and limited output masks, the block phase, the boundary count and whether a link group spans both cores) and resets the state and ring of any output sitting the packet out, which is safe because Core 1 is idle at that point. `limiter_process_outputs(first, last, ...)` runs on each core for the outputs it owns: Core 0 for outputs 0-1 in dual-core mode and every output in single-core mode, Core 1 (in `pdm_generator.c`) for outputs 2-7 on RP2350 and 2-3 on RP2040. `limiter_packet_end()` advances the block phase after both cores finish.
- **Cross-core link groups.** Each core first measures its own limited outputs into a shared envelope table (`lm_env`, one row per output, up to 12 boundaries per 192-sample packet). When a live group has members on both cores, the cores meet (`lm_meet`: `__dmb`, set own flag, `__sev`, wait for the other flag with `__wfe`). Then each core takes the group minimum per boundary and applies it to its own outputs. The meeting only happens in packets that need it, and both cores take the same decision from the same snapshot, so neither can wait forever. Core 1's CPU meter counts its wait as busy time. A host test runs the real kernel on two threads and gets output bit-identical to the single-core run.
- **Which outputs are delayed.** While the delay is engaged, every processed, matrix-enabled output passes through its 32-sample ring. Outputs with their own limiter off pass through the ring with no gain (an exact delay), so inter-slot alignment holds. Signal-generator RAW outputs are limited like any other signal, because RAW skips the crossover and a full-range sweep can reach a tweeter; switch the output's limiter off for an unaltered full-scale measurement. Matrix-disabled outputs, and the PDM output while Core 1 runs the EQ worker, are skipped and their rings cleared, so they re-enter at the same latency with silence.
- **Joining and leaving.** An output entering the limited set scans its ring once (`lm_join`, in `limiter_packet_begin`) and holds the gain the ring's peak needs, because those 32 samples were never measured. An output leaving it while `gain_active` goes into `drain_mask`: it keeps its own envelope, measured with no threshold and its cached release (`st->rel`, since the coefficients may already be NULL), and returns to plain delay once the envelope, ramp target and step are exactly unity. Neither transition can overshoot, and leaving never steps the gain.

### Engaging the Lookahead Delay

The delay exists only while at least one output's limiter is enabled; with none enabled the limiter costs no CPU and no latency. `limiter_apply_config()` publishes NULL when no output is enabled, and that is the "want" signal. Making the 32-sample delay appear or vanish mid-stream would click, so the switch is made in silence and on every output at once:

1. The main loop's `limiter_engage_service()` (in `main.c`) sees want and engaged differ. With packets flowing it holds the soft mute for `PIPELINE_FADE_REQUEST_MS` through `pipeline_hold_soft_mute()`, a raise-only variant of `pipeline_request_soft_mute()` that cannot shorten another requester's dwell. It skips the hold when `limiter_switch_ready()` says the last 32 samples were already silent. With no producer, or with no packet processed for 50 ms of main-loop time (`limiter_packet_count()`, and a gap between service calls counts as a blocked loop, not an idle producer), it switches directly through `limiter_force_engage()`.
2. `limiter_packet_begin()` counts consecutive samples whose composite output gain (host volume x preset-mute envelope x master volume) is exactly zero for the whole packet. Once 32 have passed, every ring holds only zeros.
3. At the next packet start it clears every ring and state and flips `lm_engaged`. Both sides of the switch are silent.
4. The main loop stops holding the mute and audio fades back up. The dip is roughly 8 ms down, one or two packets, and 8 ms up. A user who is already muted gets the switch with no extra fade.

`pipeline_max_active_delay_samples()` adds the 32 lookahead samples while engaged, so the flash and reset fade brackets wait for the fade to drain through the limiter too. Preset load and factory reset clear the rings with the other delay lines (`limiter_reset_all()`).

### Persistence & Control

- **Wire format V32:** `WireLimiterParams` (108 bytes, nine 12-byte `WireLimiterOutput` records: `enabled`, `link_group`, two reserved bytes, `threshold_db`, `release_ms`) is tail-appended at offset 6028, taking the total to **6136 bytes**. Records past `num_output_channels` are zero on collect and ignored on apply. Apply copies values raw. `limiter_apply_config()` sanitizes them in place (NaN falls back to the default, then clamps), so a GET always reports what the audio path uses.
- **Persistence follows `output_config_mode`** (added 2026-09-22), like the output pins. `apply_output_config_from_mode()` is the only place the live limiter settings are sourced on a context change. With-preset takes them from the slot (V39+, defaults for an empty or older slot). Independent applies the directory copy at boot only and leaves the live settings untouched on preset load, active-slot delete and factory reset. `apply_slot_to_live()` and `apply_factory_defaults()` no longer touch the limiter. `REQ_SAVE_OUTPUT_CONFIG` (0x52) also snapshots the limiter into the directory, and `bulk_params_apply()` applies the wire limiter section only when `apply_pins` (with-preset) is set.
- **Preset slot V39:** per output `limiter_enabled[]`, `limiter_link_group[]`, `limiter_threshold_db[]`, `limiter_release_ms[]` (10 bytes x `NUM_OUTPUT_CHANNELS`) tail-appended to `PresetSlot` (`SLOT_DATA_SIZE_V39`). A save always stores the live settings; whether a load applies them follows the mode.
- **Directory V22:** `FlashLimiterConfig` (112 bytes: version byte, 3 reserved, nine 12-byte records fixed at the RP2350 output count) appended after `cs_display`. See "Preset Directory Fields".
- **Vendor command:** a single opcode, `REQ_LIMITER` `0x81`, in both dispatchers. OUT sets one parameter (`wValue = (output << 8) | index`, float32 payload; output `0xFF` sets every output). IN gets one parameter as float32, or the read-only blocks at index `0x80` (gain-reduction meter, one uint16 per output in 0.01 dB) and `0x81` (status: engaged, lookahead 32, block 16, output count). A bad output or index STALLs a GET and makes a SET a no-op. NaN is ignored. `limiter_set_param()` emits one `notify_param_write` per output changed.
- **Ganged link groups** (added 2026-09-26). A SET of enable, threshold or release on a linked output writes every member of its group. A `link_group` SET makes the joining output adopt the settings of the group's lowest-numbered other member (`lm_group_peer()`); the first member keeps its own. `limiter_apply_config()` runs `lm_gang_groups()` after sanitizing, so a raw bulk or preset restore with a disagreeing group is made consistent, with a notification per value changed. A stored `link_group` above 4 now falls back to 0 (unlinked) in both `limiter_apply_config()` and `dir_sanitize_limiter()`, so corrupt data cannot join a real group. Wire, slot and directory formats are unchanged. Spec section 2.2.
- **Control Surfaces:** caps v20 nouns 74-78, all targeting an output (added 2026-09-28). `LIMITER` (enable), `LIMITER_THRESHOLD` (-30..0 dB), `LIMITER_RELEASE` (10..1000 ms, in the new log-stepping `CS_UNIT_MS_LOG`) and `LIMITER_LINK` (enum 0-4, 0 = unlinked) dispatch through `REQ_LIMITER` with `wValue = (output << 8) | index`, so all four carry a float32 payload. They reach `limiter_set_param()` exactly as a host SET does, so a control on one member of a link group moves the whole group. `LIMITER_GR` is a read-only gain-reduction meter in dB (0..30, positive = reduction) read from `limiter_meter_centidb()`, for `IND_ABOVE` and `IND_LEVEL` LEDs. Front-panel edits are live edits: in independent `output_config_mode` they persist only when the host sends `REQ_SAVE_OUTPUT_CONFIG` (0x52), and in with-preset mode on the next preset save.

### Cost

- **CPU (estimate, unmeasured).** RP2350: per limited output per sample, one load, `VABS`, `VMAXNM` in the measure pass, and a ring read and write, one multiply and one add in the apply pass, plus one divide, three min/max and one multiply per block. About 0.1 % of a core per limited output at 48 kHz and 307.2 MHz. RP2040: integer abs and compare, a ring read and write and two 16-bit multiplies per sample, plus one hardware divide and one 64-bit multiply per block, about 0.4 to 0.5 % per output. A delayed-only output costs a ring read and write per sample. Nothing runs while the delay is disengaged.
- **RAM.** BSS 2,126 B on RP2350 (1,476 B of per-output state with nine 128-byte rings, 432 B envelope table, two 92-byte coefficient buffers) and 1,238 B on RP2040 (820 B of state, 240 B table, two 72-byte buffers). RAM text 2,200 B on RP2350 and 2,444 B on RP2040, plus the config in `.data` (108 B / 60 B). The `.data` budgets in `scripts/check_ram_placement.py` were raised 2 KB for this (see "Memory Layout").

### Verification

The real `limiter.c` was compiled on the host against stub headers for both `PICO_RP2350` values and driven with random packet sizes (1-192 samples), random thresholds and releases, sines with bursting envelopes and full-scale impulses up to +18 dB (+11 dB on the RP2040 variant, its Q28 headroom). Results: no sample above threshold on any limited output (float worst case 1e-6 dB, rounding; fixed point exactly 0), including right after a mid-stream join, and none from the ring after a mid-limit leave, whose gain then changes by at most 0.0006 per sample on its way back to exact unity. Delay-only outputs are an exact 32-sample delay. Linked outputs carry identical gain. Dual-core runs on two threads are bit-identical to single-core with groups spanning both cores. The engage protocol flips at 32 silent samples and not at 31. Release measures 8.6 dB per 100 ms against 8.69 designed, and the gain returns to exact unity (bit-exact passthrough) when quiet. Device-side protocol tests are in `tools/dspi_test/tests/dynamics.py` (`limiter_*`); they have not run on hardware yet.

### Interactions and Edge Cases

- **Enabling or disabling one output's limiter mid-stream** is covered by the join scan and the drain (see "Pipeline Placement and Cores").
- **Sample peaks, not true peaks.** A DAC can overshoot between samples by up to about 1 dB on worst-case material, hence the -1 dBFS default.
- **Rate change.** Release coefficients are recomputed (`perform_rate_change()` raises `limiter_update_pending`). The lookahead is fixed in samples, so attack in milliseconds halves at 96 kHz.

---

## Stereo Upmixer
*Last updated: 2026-10-10 (SET change notifications, shared wire encoder; 2026-08-01: centre engine OFF mode)*

### Purpose

Derives Centre / Left-Surround / Right-Surround virtual source channels from a stereo program so a two-channel input can drive a multi-speaker layout. It runs **only on RP2350** (the feature is compiled out on RP2040), **only when the active input is the plain stereo pair** (`active_input_channel_count() == 2`), and **only at sample rates of 48 kHz or below** (the delay rings are sized for 48 kHz; above that the upmixer parks, ADAT-style, and `upmix_apply_config` publishes NULL). In multichannel input modes those matrix rows carry real inputs and the upmixer parks the same way. Module: `firmware/DSPi/upmix.c/h`. Status: **HW-untested**.

### Signal Flow

The pass sits between the leveller and the matrix (PASS 3 in the RP2350 pipeline). It reads the post-EQ/leveller stereo bus (`buf_l`/`buf_r`), writes the derived channels into the otherwise-idle multichannel input rows (`buf_in_ext[0..2]` = matrix source rows 2..4 = `UPMIX_ROW_C`/`_LS`/`_RS`), and applies centre removal to L/R in place. The matrix then treats the derived channels as ordinary sources: routing, crosspoint gains, and the whole per-output chain (PEQ, crossover, delay, gain, loudness) are reused unchanged. The matrix loop bound is generalized from `n_active_inputs` to `n_matrix_sources` (2 + 1 for centre-only, 2 + 3 with surround), so only the rows actually carrying derived audio are summed. The derived rows have **no input PEQ** (they are synthesized after the per-input stage) but DO get peak/clip metering into `global_status.peaks[2..4]` while the upmixer runs; the extracted centre can legitimately reach +3 dBFS on hot correlated content, so hosts must watch these meters. Steering telemetry (correlation, balance, live gains) is exposed through `REQ_UPMIX_GET_STATUS`.

Two independent engines feed the derived rows.

**Centre engine** (owns row 2 whenever the pass runs):
- `PASSIVE`: `C = 0.7071 * (L + R)`, fixed constant-power sum.
- `OFF` (value 2, added 2026-08-01): `upmix_compute_coefficients()` forces `strength = 0`, and `upmix_process_block()` additionally snaps `um.g_c` to exactly zero instead of running it through the release ballistic (which only decays asymptotically). Both the C output and the L/R removal scale by that gain, so row 2 goes silent and `l[i] = l0 - 0.0f * mid` is bit-exact. The one-block linear gain ramp still applies, so the switch glides out over ~1 ms rather than stepping. Row 2 stays reserved (`n_derived` tracks the surround mode alone) so Ls/Rs keep rows 3/4 and existing matrix routing is not renumbered.
- `ADAPTIVE`: a running normalized cross-correlation and L/R balance (one-pole estimators on a bass-cut, one-pole-HP detector path) drive the centre gain through a threshold gate with renormalization above the knee, then attack/release ballistics applied per block (packet-size independent) and per-sample gain ramps. Only genuinely centre-panned correlated content is extracted, so the image does not pump; long-wavelength content is excluded by the detector HP (industry-standard bass-steering mitigation).

Extracted centre energy is subtracted from L/R scaled by strength and by centre-width (`0.5 * (1 - width)` removal), so a physical centre speaker and the L/R phantom do not comb-filter. Constant-power conventions: `0.7071` centre extraction, `0.5` removal.

That removal is the only write the pass makes to the mains: the surround engine reads L/R but never modifies them (the patent's front-channel gain riding is deliberately omitted). Centre `OFF` and `center_width_pct = 100` therefore both yield a bit-exact stereo pair with the surrounds still running. Because the removal is common-mode, it also barely affects the surrounds: the passive surround feed's coefficients sum to exactly zero so it is mathematically unchanged by centre width, and the adaptive feed changes only in its `0.3812 * M` mono term. Steering decisions are unaffected either way; the correlation and dominance estimators read `l0`/`r0` before removal.

Both centre modes then run a **presence bell** on the extracted C (added 2026-07-19, Syn-style presence control): a Cytomic TPT-SVF bell at fixed 3 kHz / Q 0.6, `presence_db` in [-12, +12] dB, boost/cut symmetric (`k = 1/(Q*A)`), `m1 = k*(A^2 - 1)` so 0 dB is an exact passthrough. Negative moves voices back, positive brings them forward. The filter runs unconditionally while the pass is active so gain sweeps through 0 dB stay continuous.

**Surround engine** (rows 3..4 when `surround_mode != OFF`):
- `OFF`: no surround rows (`n_derived = 1`).
- `PASSIVE`: `Ls = Rs = 0.7071 * (L - R)` difference feed, mirrored polarity.
- `ADAPTIVE`: Dolby low-complexity matrix decoder steering (WO2007067320A2): rectified level differences per axis, gain 1024 + hard clip, 40 ms one-pole dominance smoothers, polynomial pan law (`1 - x^2`) with front/back bias mapped into `[-1, 0]`, and Pro Logic II surround decode feed coefficients (`0.8710` / `-0.4898`). The patent's front L/R gain riding is deliberately omitted; the centre engine owns all modification of the mains.

Both surround modes then run a built-in conditioning chain per surround channel: a 2nd-order Butterworth TPT-SVF high-pass (default 300 Hz) and low-pass (default 7 kHz) band-limit, a Haas delay (default 12 ms, per-channel ring of 1024 samples; the 20 ms ceiling holds at 48 kHz), and a mirrored-gain Schroeder allpass decorrelator (10 ms, `G = 0.5 * decorr_pct / 100`, ring of 512 samples). Routing "Upmix Ls/Rs" to output slots is then the entire user setup.

**Zero added latency, alignment preserved.** Steering is pure gain (zero latency) on C/L/R. The surround Haas delay is a deliberate feature and is identical for every output slot fed from the same source row, so inter-output-slot sample alignment is preserved by construction (the CLAUDE.md inviolable guarantee).

### Coefficient Publish & State

Follows the psybass module pattern:

- **Double-buffered publish.** `upmix_apply_config()` computes one shared `UpmixCoeffs` into the inactive buffer (`um_coeff_bufs[2]`) and atomically publishes `volatile const UpmixCoeffs *current_upmix_coeffs` (**NULL = disabled**), never mutating the buffer the pipeline reads. Vendor SET handlers write `upmix_config`, raise `upmix_update_pending` and announce the change (see "Change notifications" below); the main loop consumes the flag and recomputes. Coefficients are also recomputed on rate change (`perform_rate_change()` raises the flag). Initial setup runs once in `core0_init()`.
- **Processing state.** All state is Core 0 only (the pass runs before the matrix). `upmix_process_block()` is `DSP_TIME_CRITICAL` (RAM-resident). Whenever the pass does not run (multichannel input, or disabled), the pipeline calls the inline `upmix_park()`, a dirty-flag check that runs the cold `upmix_reset_state()` once on the running -> parked transition so re-entry starts clean.
- **Telemetry.** `upmix_get_status()` fills a 16-byte `UpmixStatus` (active flag, `parked_reason` 0 active / 1 disabled / 2 input-not-stereo / 3 rate-above-48kHz, smoothed correlation and balance in Q14, and centre/Ls/Rs gains in Q15).

### Parameters

One global config (`UpmixConfig`); ranges are clamped downstream in `upmix_compute_coefficients()` (SET handlers only validate the enable/mode fields).

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| enabled | 0/1 | false | Enable/disable the upmixer |
| center_mode | 0..2 | ADAPTIVE (1) | Centre engine: 0 PASSIVE, 1 ADAPTIVE, 2 OFF. Out-of-range falls back to the default (not clamped up to OFF) via `upmix_clamp_center_mode()`, shared by the wire, bulk, and flash paths |
| surround_mode | 0..2 | ADAPTIVE (2) | Surround engine: 0 OFF, 1 PASSIVE, 2 ADAPTIVE |
| strength_pct | 0..100 | 100 | Centre extraction strength |
| center_width_pct | 0..100 | 25 | Residual L/R retention (0 = full removal, 100 = phantom kept) |
| corr_threshold_pct | 0..95 | 30 | Correlation gate for adaptive centre |
| attack_ms | 1..500 | 10 | Centre gain attack ballistics |
| release_ms | 5..2000 | 100 | Centre gain release ballistics |
| detector_hpf_hz | 20..1000 | 200 | Detector bass-cut corner (anti bass-steering) |
| surround_delay_ms | 0..20 | 12 | Surround Haas delay |
| surround_hpf_hz | 20..2000 | 300 | Surround band-limit high-pass |
| surround_lpf_hz | 1000..20000 | 7000 | Surround band-limit low-pass |
| decorr_pct | 0..100 | 90 | Decorrelator amount (`G = 0.5 * pct/100`) |
| presence_db | -12..+12 | 0 | Centre presence bell gain (dB), fixed 3 kHz / Q 0.6; both centre modes |

### Persistence & Control

- **Wire format V25 (presence byte V26; centre OFF V27):** `WireUpmixParams` (44 bytes, layout-identical to `UpmixConfigPacket`) is tail-appended to `WireBulkParams` at offset 5900 (total 5944 bytes at V25; later sections follow it). V26 claims the section's reserved byte for `presence_q1` (int8, dB * 2, 0.5 dB steps; no size change). V27 changes no layout at all; it exists so hosts can detect that `center_mode` accepts 2 (OFF). Bulk collect/apply copy it straight to/from `upmix_config` and raise the pending flag; the section is zeroed on collect and ignored on apply on RP2040. `upmix_config_to_packet()` is the single live-to-wire encoder, shared by GET_CONFIG, bulk collect and change notifications, so the notify shadow can never disagree with a bulk read.
- **Preset slot V33 (presence byte V34):** the config is tail-appended to `PresetSlot` (struct grows 44 bytes; `SLOT_DATA_SIZE_V33`), gated on `slot->version >= 33` in `apply_slot_to_live()` (RP2350). V34 claims the upmix reserved byte for `upmix_presence_q1` (no size change; `SLOT_DATA_SIZE_V34` = `SLOT_DATA_SIZE_V33`); V33 slots always wrote 0 there, which decodes to the 0 dB default, so no version gate is needed for the byte. Older slots have no upmix data and load the disabled defaults; V21..V33 slots still validate via `slot_data_size_for_version()`. Factory reset sets it disabled with `UPMIX_DEFAULT_*` values. RP2040 round-trips the fields as zeros and never applies them.
- **Vendor commands:** `0x4A-0x4E`; RP2350 only (SETs STALL on RP2040, GETs return zeros). `0x4A` SET_CONFIG takes the 44-byte `UpmixConfigPacket` (byte 3 = `presence_q1`, int8 dB * 2); `0x4B` GET_CONFIG returns it; `0x4C`/`0x4D` SET/GET_PARAM address a single field by `wValue` (`UPMIX_PARAM_*` 0..13; 13 = `UPMIX_PARAM_PRESENCE`, plain float dB) as a 4-byte float; `0x4E` GET_STATUS returns the 16-byte `UpmixStatus`. `0x4F` is reserved. See the Vendor Command Reference table.
- **Change notifications (added 2026-10-10):** both SET handlers call `upmix_notify_config()` after writing `upmix_config`. It runs `notify_param_write()` over all 14 fields at their `WireBulkParams.upmix` offsets, and the notify shadow comparison turns that into one `PARAM_CHANGED` per field that actually changed. Control Surfaces nouns and the UART/I2C transports reach the same handlers, so every live edit is announced with its source tag. Before this the upmixer was the only DSP block whose SETs were silent, so a second host or a front-panel edit left other UIs stale until the next bulk read. Preset load, factory reset and bulk SET still announce through the bulk-invalidation bracket. Presence is notified in its 0.5 dB wire encoding, so a SET_PARAM that moves the float within one step emits nothing. Any new upmixer SET path must also call `upmix_notify_config()`. Host-facing detail is in `upmixer_spec.md` section 6.5.

**Matrix row-sharing consequence.** Source rows 2..4 are dual-purpose: they carry multichannel inputs 3..5 in multichannel input modes and the Upmix C / Ls / Rs derived channels in stereo upmix mode. The matrix crosspoints for those rows are the same storage in both cases, so a host application must present them contextually (real input vs upmix-derived) based on the active input mode.

---

## Volume Leveller
*Last updated: 2026-10-10 (safety limiter rebuilt as a smoothed peak limiter on the boost, see "Safety Limiter"; 2026-07-10: crossfeed no longer stereo-only; runs per output pair)*

### Purpose

Automatic volume levelling via a feedforward, channel-linked, single-band RMS compressor applied to the active input channels (2 to 8 on RP2350; always 2 on RP2040), pre-matrix, after per-input EQ (PASS 2.5 in the pipeline). Runs at all input counts; no longer bypassed in multichannel (the earlier stereo-only restriction is gone). Crossfeed now runs per output pair post-matrix (`output_pair_mask`-selected), not on the stereo input bus, so it too works in every input mode (see "Crossfeed"); loudness likewise runs per output (post-gain, `loudness_output_mask`-selected), so it is no longer stereo-only (see "Loudness Compensation").

### Algorithm

- **Topology:** Feedforward upward compressor with soft knee — boosts content below the threshold, leaves content above the threshold completely untouched (no makeup gain needed)
- **Channel linking:** A single shared gain drives every selected channel. The link is the loudest selected detector envelope (max over `detector_mask` channels), preserving the mix balance; for stereo this is exactly the old "louder of L/R" behavior
- **Channel masks:** `detector_mask` (which inputs feed the RMS detector) and `apply_mask` (which inputs receive the shared gain), each a `uint8_t` bit-per-input, default 0xFF (all channels). Both are ANDed with the active-input set at runtime; CPU is only spent on selected channels. Example uses: dialog boost = both masks on the center channel; night mode = all channels
- **Envelope:** Asymmetric attack/release smoothing on the per-channel RMS envelopes
- **Lookahead:** Optional 5 ms (240-sample) lookahead, one ring per input channel. When enabled, EVERY active input is delayed through its ring (applied channels through the gain stage, others as a plain delay), so inter-channel and inter-output-slot alignment is preserved exactly and mask changes cause zero time shift. Rings of newly activated inputs are cleared via the `active_prev` field
- **Gain computation:** Upward compression curve: content below threshold is boosted by `(threshold - x_db) * (1 - 1/ratio)`, content above threshold + knee/2 passes at unity (0 dB gain), with soft knee transition between
- **Safety limiter:** Caps the boost so the leveller never pushes a sample above -3 dBFS. It is a smoothed peak limiter with a one-block attack ramp, 50 ms hold and 60 dB/s release, and never cuts below unity. See "Safety Limiter" below
- **Gate:** User-configurable silence gate prevents noise amplification when input is below the gate threshold
- **Glitch-free masks:** Writing masks (0xDE) recomputes coefficients but does NOT reset state; the rings keep running so a mask change is glitch-free

### Safety Limiter
*Last updated: 2026-10-10*

The limiter caps the leveller's boost so that the leveller never pushes a sample above the -3 dBFS ceiling (`LEVELLER_LIMITER_CEIL`). It only ever lowers the boost, and never below unity. Content already above the ceiling therefore passes untouched, and the leveller never attenuates. The gain applied to each sample is the smallest of the leveller gain, the limiter ramp and the instant cap. The cap is computed over the apply-mask channels only and shared by them, so the mix balance holds.

**Why it is an envelope, not a clamp.** The earlier limiter set the gain per sample to `min(gain, ceiling / |x|)` with no smoothing, which is a hard clipper. After a quiet passage at about +14 dB of boost, every sample of a loud hit above about -17 dBFS came out flat at -3 dBFS. A host run of the real `leveller.c` measured -11.6 dB THD+N over the first 20 ms of a -6 dBFS, 1 kHz burst on RP2350 and -10.9 dB on RP2040, with clipping lasting about 400 ms until the leveller's own attack caught up.

**Limiter blocks.** Timing works on 48-sample blocks (`LEVELLER_LIM_BLOCK`). The 240-sample lookahead ring holds exactly five of them, and `lim_cap[b]` holds the cap for ring block b. A block's cap is the largest gain that keeps its peak under the ceiling, clamped to at least unity. Blocks follow stream position, not packets, so a packet may start or end anywhere inside a block. With lookahead on, the block phase is derived from `la_write_idx`, so it can never walk off the ring.

**Attack with lookahead.** When a block finishes, the block just written into the ring gets its cap, measured from the incoming samples. The next block's ramp then runs linearly from `l0` to `l1`. `l0` is the gain actually in force at the boundary. `l1` is the smaller of the held envelope and every cap in the ring. A cap therefore binds from the first boundary after it is measured, four blocks before its samples play. Every later boundary includes it as well. Both ends of the ramp across a block are then at or under its cap, and a straight line between two such values stays under it, so no sample ever exceeds the ceiling. In practice the gain reaches its final value about 4 ms before the hit (at 48 kHz) through a 1 ms linear ramp on the quieter audio ahead of it, where the hit masks it. The same 1 kHz burst now measures -150 dB THD+N in the first 20 ms (-143 dB on RP2040).

The window minimum was chosen over a ramp that aims at each block's cap just in time. That alternative keeps more boost, but on a 50 Hz burst it follows the rising first quarter cycle down and measured -29.8 dB THD+N over the first 40 ms. The window minimum measured -64.6 dB on both platforms, against -13.9 dB for the old clipper.

**Attack without lookahead.** With no future samples to see, the instant cap (`lim_inst`) drops at the sample that would cross the ceiling and holds for the rest of the block. The waveform is shaped only while the first rising quarter cycle keeps deepening the cap. That measured about -26 dB THD+N at 1 kHz and -21 dB at 50 Hz in the first window, against about -10 dB and -13 dB for the clipper. Turning lookahead off trades this onset cleanliness for 5 ms less latency.

**Hold and release.** The cap of each block that has finished playing feeds `lim_env`. A cap at or under the held value replaces it and re-arms a 50 ms hold (`LEVELLER_LIM_HOLD_SEC`). That spans a half cycle down to 10 Hz, so steady peaks never let the gain rise between crests. Once the hold runs out, the envelope rises at 60 dB/s (`LEVELLER_LIM_RELEASE_DBPS`). On a sustained loud passage the leveller's own RMS detector catches up within its window. The leveller gain then falls under the cap and the limiter goes idle. After a short isolated hit the leveller's detector also dips the gain, and the limiter's hold stays inside that dip.

**Mask changes and lookahead toggles.** Masks change without a state reset (0xDE). A channel newly added to the apply set therefore has 5 ms of samples in its ring that no cap has measured. `lim_rescan` re-measures every ring block from the ring and drops the ramp straight to the lowest cap, a single step at most once per change. The last apply set is kept together with the lookahead flag in `lim_app_prev`. A lookahead toggle that the audio path sees before the main loop's state reset therefore rescans too.

**Platform notes.** RP2350 runs the limiter in float. RP2040 keeps the gains in Q28 with `LIM_GMAX = INT32_MAX` (just under 8.0, above the 18 dB boost limit). `fast_mul_q28` floors, so the ramp step is rounded toward zero instead. A floored falling step overshoots its target and can carry a ramp down to unity slightly below it, which would attenuate. `lim_cap_of` holds the only float divide, which runs once per block, and per sample only while a no-lookahead attack is still deepening. The per-segment loops are channel-major and share one per-sample gain array (`gv`). Sample-major inner channel loops were fully unrolled by `-O3` and overflowed the `.data` budget. The limiter costs 56 B of `LevellerState` and adds 336 B (RP2040) and 496 B (RP2350) of RAM code. That leaves 144 B and 64 B under the `.data` budgets.

**Verification.** A host harness compiles the real `leveller.c` for both platforms. It covers the THD+N figures above and 180 randomized stress runs (2-channel on both platforms, 8-channel with changing active counts on RP2350; random packet sizes, masks, levels and spikes). Those runs check that applied outputs never cross the ceiling unless the input already did, never fall below the input, and that unapplied channels stay exact delayed copies. Removing the rescan or limiting the window to the newest block makes the stress runs fail. The limiter is untested on hardware, and its CPU cost has not been measured on the CPU meter.

### Parameters

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| enabled | bool | 0/1 | false | Enable/disable the leveller |
| amount | float | 0.0–100.0 | 50.0 | Compression strength % (ratio = 1 + amount/100 * 19) |
| speed | uint8_t | 0/1/2 | 0 (Slow) | Envelope speed: 0 = Slow, 1 = Medium, 2 = Fast |
| max_gain_db | float | 0.0–35.0 | 15.0 | Maximum boost gain in dB (RP2040: effective cap 18 dB; Q28 gain range) |
| lookahead | bool | 0/1 | true | Enable 5 ms (240-sample) lookahead delay |
| gate_threshold_db | float | -96.0–0.0 | -96.0 | Silence gate level in dBFS (below = no boost) |
| detector_mask | uint8_t | 0x00–0xFF | 0xFF | Bit k: input channel k feeds the detector |
| apply_mask | uint8_t | 0x00–0xFF | 0xFF | Bit k: shared gain applied to input channel k |

### Signal Chain Position

```
USB Input → Per-Ch Preamp → Per-Input EQ + Metering → Volume Leveller (active inputs, mask-driven) → Matrix Mix → [Crossfeed, per masked output pair] → Output EQ → Output Gain × Host Vol × Master Vol → [Loudness, per masked output] → Delay → Output

(Crossfeed now runs per output pair post-matrix, gated by `output_pair_mask`, so it works in every input mode instead of being bypassed in multichannel; loudness runs per output post-gain in every mode, gated by `loudness_output_mask`; the leveller runs over all active inputs; each active input gets its own PEQ + metering before the matrix.)
                                                     (PASS 2.5)
```

### Platform Implementation

- **RP2350:** Float throughout — RMS envelope, gain computation, and gain application all in single-precision float. `NUM_INPUT_CHANNELS = 8`; per-channel `env_sq[8]` and eight lookahead rings
- **RP2040:** Q28 fixed-point for the RMS envelope accumulator, float for gain computation and smoothing. Gain applied to Q28 audio samples. `NUM_INPUT_CHANNELS = 2`; two lookahead rings. Max boost clamped to 18 dB at coefficient time (Q28 linear gain tops out below 8.0); the limiter cap conversion is likewise overflow-guarded

### Files

| File | Purpose |
|------|---------|
| `leveller.c` | RMS envelope tracking, gain computation, soft-knee curve, mask-driven link, per-channel lookahead rings |
| `leveller.h` | Public API, state struct, configuration struct (incl. detector_mask/apply_mask) |

### Vendor Commands (0xB4–0xBF, 0xDE/0xDF)

| Code | Command | Direction | Description |
|------|---------|-----------|-------------|
| 0xB4 | REQ_SET_LEVELLER_ENABLE | OUT | Enable/disable leveller |
| 0xB5 | REQ_GET_LEVELLER_ENABLE | IN | Get leveller enabled state |
| 0xB6 | REQ_SET_LEVELLER_AMOUNT | OUT | Set compression amount (0.0–100.0, float) |
| 0xB7 | REQ_GET_LEVELLER_AMOUNT | IN | Get compression amount |
| 0xB8 | REQ_SET_LEVELLER_SPEED | OUT | Set envelope speed (0=Slow, 1=Medium, 2=Fast) |
| 0xB9 | REQ_GET_LEVELLER_SPEED | IN | Get envelope speed |
| 0xBA | REQ_SET_LEVELLER_MAX_GAIN | OUT | Set max boost gain in dB (0.0–35.0, float) |
| 0xBB | REQ_GET_LEVELLER_MAX_GAIN | IN | Get max boost gain |
| 0xBC | REQ_SET_LEVELLER_LOOKAHEAD | OUT | Enable/disable 5 ms lookahead |
| 0xBD | REQ_GET_LEVELLER_LOOKAHEAD | IN | Get lookahead state |
| 0xBE | REQ_SET_LEVELLER_GATE | OUT | Set silence gate threshold (-96.0–0.0 dBFS, float) |
| 0xBF | REQ_GET_LEVELLER_GATE | IN | Get silence gate threshold |
| 0xDE | REQ_SET_LEVELLER_MASKS | OUT | Set channel masks (2 bytes: [detector_mask, apply_mask]); no state reset |
| 0xDF | REQ_GET_LEVELLER_MASKS | IN | Get channel masks (2 bytes: [detector_mask, apply_mask]) |

---

## Loudness Compensation
*Last updated: 2026-10-24 (table rebuild is a background job: `loudness_recompute_begin()` starts it, it builds volume steps into the inactive table until each step's deadline and swaps when complete, and `loudness_recompute_take_done()` triggers the re-key; boot keeps the one-call `loudness_recompute_table()`). Previously 2026-10-23 (rebuild sliced at 8 steps per pass), 2026-07-09 (per-output processing + output mask)*

### Purpose

ISO 226:2003 equal-loudness contour compensation to maintain perceived frequency balance at low listening volumes.

### Filter Architecture

2 shelf filters per processed OUTPUT channel:
1. Low shelf (200 Hz, Q=0.707); bass boost at low volume
2. High shelf (6000 Hz, Q=0.707); treble boost at low volume

**RP2350:** SVF shelf filters (Cytomic "SvfLinearTrapAllOutputs" with `k = 1/Q` for exact RBJ matching). Both loudness filters are always below the SVF crossover frequency at all supported sample rates, so SVF is used unconditionally. Coefficients (`LoudnessCoeffs`) are SVF integrator + mix coefficients (`sva1-3`, `svm0-2`) plus a `bypass` flag; per-output state is the minimal `LoudnessSvfState` (`ic1eq`, `ic2eq`).

**RP2040:** Q28 fixed-point RBJ biquad coefficients with `fast_mul_q28()` processing; per-output state is a minimal DF2 pair (`s1`, `s2`) in `LoudnessBqState`.

### Per-Output Processing (2026-07-09)

Loudness now runs **per output channel**, inside the per-output chain (PASS 5-7), immediately after the output gain stage and before delay, peak metering, and encode. Post-gain placement maximizes headroom (compensation only boosts when user volume is low) and keeps clip meters truthful. This replaces the previous design, which ran two volume-keyed shelves on the stereo input bus (`buf_l`/`buf_r`) before per-input EQ and the matrix and was hard-skipped whenever a multichannel USB alt was active on RP2350. The old stereo state arrays in `audio_pipeline.c` are gone, as is the multichannel skip.

**Output mask.** A global `volatile uint16_t loudness_output_mask` (default `0xFFFF` = all outputs; `LOUDNESS_DEFAULT_OUTPUT_MASK`) selects which output channels are compensated; bit k = output k. Only masked outputs cost CPU, and behavior is identical in stereo and multichannel input modes. Intended uses: compensate headphone outputs 0-1 but not speaker outputs 2-3, or all outputs for a 7.1 night mode. Outputs derived through crossovers or bass management should share the same mask setting to keep summed responses coherent; this is a user configuration choice.

**Per-output state.** `LoudnessOutputState loudness_output_state[NUM_OUTPUT_CHANNELS]` lives in `loudness.c` (RP2350: 2 SVF states = 16 B/output, 144 B total; RP2040: 2 DF2 states = 16 B/output, 80 B total). The coefficient table generation (ISO 226 math, double-buffered tables, recompute triggers) is UNCHANGED; only the filter state moved out of the shelves and into this per-output array.

**Block helper.** `loudness_process_output_block()` is a `static inline` in `loudness.h`, shared by Core 0 (`audio_pipeline.c`) and the Core 1 EQ worker (`pdm_generator.c`). It is filter-major (each shelf processes the whole block before the next, equivalent to the per-sample cascade for these LTI filters and cheaper); a bypassed shelf (0 dB at this volume step) clears its state and is skipped.

**Two-core coherence.** Core 0 snapshots the coefficient pointer (`current_loudness_coeffs`, or NULL when loudness is disabled) and the mask once per packet and passes them to Core 1 through new `Core1EqWork` fields (`loud_coeffs`, `loud_mask`), so both cores apply one consistent view for the whole packet.

**Skip-and-clear.** An output is skipped and its state cleared (`loudness_reset_output_state()`) when it is masked off, matrix-disabled, muted to zero gain for the whole packet, or carrying a siggen RAW test signal. Clearing prevents a stale-state transient when the output re-enters compensation.

**Alignment.** The shelves are IIR with no sample delay, so inter-output slot alignment is untouched (the CLAUDE.md inviolable guarantee holds). ADAT bulk output mirrors the finalized `buf_out`, so it inherits per-output loudness automatically.

**Persistence and control.** `loudness_output_mask` is set/read by vendor commands `REQ_SET_LOUDNESS_MASK` (0xFA, 2-byte little-endian mask) and `REQ_GET_LOUDNESS_MASK` (0xFB, returns 2 bytes). It persists via wire format **V19** (a `uint16 loudness_output_mask` in `WireGlobalParams`, replacing its `reserved[2]`) and preset slot **V26** (tail-append; older slots default to `0xFFFF` = all outputs).

### Parameters

- **Reference SPL:** 40-100 dB (default 87 dB)
- **Intensity:** 0-200% (default 100%)

### Table Architecture

Double-buffered for glitch-free updates:
```c
LoudnessCoeffs loudness_tables[2][61][2];  // [buffer][volume_step][biquad]
```

- 61 volume steps: -60 dB to 0 dB (1 dB increments), index 0 = silent
- Background computation writes inactive buffer, atomic pointer swap activates

### ISO 226 Constants Correction (2026-05-05)

Earlier firmware had three incorrect values in `loudness.c` claiming to be from ISO 226:2003 Table 1. They were not — `Lu` was sign-flipped at both evaluation points and `αf` at 10 kHz was off by ~10%. The contour difference produced by `iso226_spl()` came out almost flat (e.g. ~−0.14 dB SPL change at 50 Hz across a 20-phon step instead of the correct ~−15 dB), which made `compensation = freq_change − flat_change` ~ +20 dB instead of +5 dB. Net effect: the loudness compensation was ~2.5× too aggressive at both ends of the audio spectrum.

| Constant | Old value | Correct ISO 226:2003 Table 1 |
|---|---|---|
| 50 Hz Tf | 44.0 ✓ | 44.0 |
| 50 Hz αf | 0.432 ✓ | 0.432 |
| 50 Hz Lu | +80.4 ✗ | **−15.9** |
| 10 kHz Tf | 13.9 ✓ | 13.9 |
| 10 kHz αf | 0.301 ✗ | **0.271** |
| 10 kHz Lu | +17.8 ✗ | **−10.7** |

After the fix, hand-computed shelf gains at `ref_spl = 80 dB`, `intensity = 100%` (rounded to 0.1 dB):

| vol_idx | vol_db | effective_phon | 50 Hz boost (low shelf) | 10 kHz boost (high shelf) |
|---|---|---|---|---|
| 60 | 0 | 80 | 0.0 dB | 0.0 dB |
| 50 | −10 | 70 | +4.1 dB | +0.7 dB |
| 40 | −20 | 60 | +8.3 dB | +1.4 dB |
| 30 | −30 | 50 | +12.2 dB | +2.0 dB |
| 20 | −40 | 40 | +16.1 dB | +2.6 dB |
| 10 | −50 | 30 | +19.6 dB | +2.8 dB |
| 0 | −60 | 20 | +16.7 dB | −4.4 dB *(see note)* |

These match the qualitative shape of published Fletcher–Munson / ISO 226 contour-derived loudness curves (e.g. Yamaha YPAO, Audyssey Dynamic EQ), which typically apply +6 to +15 dB of bass boost and +2 to +6 dB of treble boost at −20 to −30 dB attenuation from the reference level.

*Note on the vol_idx = 0 entry:* `effective_phon` is clamped at 20 (ISO 226 isn't validated below 20 phon), but `flat_change` continues to use the clamped value as the contour reference. At deep attenuation the high-shelf comparison produces a small negative compensation. This is benign — the audio is effectively silent at vol_idx ≤ 5 — but if needed in future a `if (compensation < 0) compensation = 0;` guard in `loudness_compensation_db()` would cap it at zero.

### Migration note for existing presets

The wire format and persisted preset data are unchanged (only `intensity_pct` and `ref_spl` are stored, and they're applied through the corrected formula at runtime). Users who liked the prior intentionally-strong response can dial `intensity_pct` from 100 % up to ~250 % to roughly recover the old curve. A matching default of 100 % now corresponds to a defensible ISO-226-derived compensation rather than the previous over-boosted curve.

---

## Flash Storage
*Last updated: 2026-09-22 (single staging buffer: `flash_write_sector()` is now a thin copy-in wrapper over `flash_program_staged()`, and `preset_save()` builds its slot in place in `flash_stage`; the IRQ-context lockout skip is documented as a fallback, since no flash write runs from an IRQ). Previously, 2026-07-25 (selective NVIC blackout: the output DMA IRQ lines stay live through erase/program, so every slot keeps clocking framed silence instead of freezing mid-frame; PDM ring silenced and re-anchored). Previously, 2026-07-23 (every runtime flash write now completes via complete_flash_write_operation_full; light completion path removed)*

### Flash Operation Safety

Flash erase/program requires quiescing XIP (execute-in-place): only the erase/program windows forbid flash fetches; ordinary XIP execution is legal even with IRQs disabled. `flash_program_staged()` (which every record write goes through) and `preset_delete()` run under a selective interrupt blackout (below) and park Core 1 via a guarded `multicore_lockout` when Core 1 is a registered victim:

- **Guard condition:** `multicore_lockout_victim_is_initialized(1) && (__get_current_exception() == 0)`. The SDK function handles first-boot (Core 1 not launched) and launch-to-init race windows. The exception check skips lockout from IRQ context, where SDK lock internals are unsafe. This is a fallback only. No flash write runs from an IRQ today: vendor requests on every transport dispatch from the Core 0 main loop (USB callbacks run in `tud_task()`, and `pico_stdio_usb`, which would call `tud_task()` from an IRQ, is not linked). An IRQ-context save that skipped the lockout would still be safe because everything Core 1 can execute is RAM-resident (enforced by `scripts/check_ram_placement.py` Check B / B2), so a parked-or-not Core 1 never fetches from flash during the erase/program window. The audio consequence of a flash write is unchanged (muted window, feedback re-seed).
- **Staging buffer:** every record is written from one static, 256-byte-aligned `flash_stage` union (`PresetSlot` or `SLOT_BYTES` raw bytes), 0xFF-padded past the record. `flash_write_sector()` copies a caller's record in. `preset_save()` instead 0xFF-fills the buffer and calls `collect_live_state()` straight into `flash_stage.slot`, so nothing may use `flash_stage` between that collect and the slot write. The directory write that follows reuses the buffer only after the slot write has finished.
- **Core 1 victim init:** `multicore_lockout_victim_init()` called at the start of `pdm_core1_entry()`.
- **Interrupt blackout:** ~45 ms for sector erase + program. Since 2026-07-25 this is a *selective* blackout, not PRIMASK (see below). The existing mute strategy (`preset_loading` + `preset_mute_counter`) and feedback reseed still cover the audio gap.

#### Selective NVIC blackout — outputs keep clocking through the window
*Last updated: 2026-10-10 (output rings: the kept line is now the output-ring stall alarm, since the outputs no longer use DMA IRQs). Previously, 2026-07-25 (RP2350: NVIC word 1 now masked too — flash-resident UART/I2C control ISRs could fire mid-window; restore ordering word 0 then word 1)*

Historically the erase/program window ran under `save_and_disable_interrupts()`. With every IRQ masked the output DMA completion handlers could not re-arm: each slot's DMA finished its in-flight 48-sample transfer, the PIO TX FIFO drained within tens of microseconds, and the output pin then held a DC level for the rest of the window. AC-coupled outputs turned that step into a pop, SPDIF receivers saw a dead line and unlocked, and BCK-PLL DACs (PCM5102 with SCK grounded) could exit the halt mis-locked.

`flash_irq_blackout_begin()` / `flash_irq_blackout_end()` (`flash_storage.c`, both RAM-resident) replace it:

- **Thread context** (the only production path, since every runtime flash write is deferred to the main loop): save NVIC `ISER` word 0, then write `ICER` to disable everything *except* the output-ring stall alarm's line (`output_rings_stall_nvic_mask()`). `__dsb(); __isb();` orders the mask before the ROM flash calls; on exit `ISER` is restored.
- `ICER`/`ISER` are written directly rather than through `irq_set_mask_n_enabled()`, because the SDK helper clears pending bits on re-enable; direct writes preserve IRQs latched during the window (USB, timer, UART/I2C), matching the old PRIMASK semantics. Register shapes differ per platform (RP2040 scalar `iser`/`icer`, RP2350 `iser[2]`/`icer[2]`); the stall alarm is a timer IRQ (0 to 3) on both platforms, so the keep mask lives entirely in word 0.
- **RP2350 NVIC word 1 is masked wholesale.** RP2350 interrupt numbers run past 31: SPI1 (32), UART0/1 (33/34), ADC (35), I2C0/1 (36/37) live in `iser[1]`/`icer[1]`, which the initial implementation never touched. The UART and I2C control-interface ISRs (`uart_ctrl_irq`, `i2c_ctrl_slave_handler`) are flash-resident, so a byte arriving on an active control link mid-window vectored into quiesced XIP — a hard fault, or an intermittent one when the XIP cache happened to hold the handler. The old PRIMASK blackout covered these; the selective blackout now saves/clears word 1 alongside word 0. RP2040 has no IRQ above 31, so word 0 covers everything there. Control-link bytes arriving during the window can still overflow the 32-byte UART RX FIFO (~45 ms at 115200 baud is ~520 byte times); that loss is identical to the PRIMASK era and the frame CRC + status-response protocol already recovers from it.
- **Restore ordering (defensive).** An IRQ enabled by an `ISER` write can preempt at the next instruction boundary — the trailing `__dsb(); __isb();` order the writes, they do not gate preemption. The end path therefore restores word 0 first, then word 1, so a pending UART/I2C interrupt that preempts mid-restore finds the core IRQs (timer, USB) already enabled; the begin path masks in the reverse order. XIP is back before `flash_irq_blackout_end()` runs, so this is consistency hygiene, not a crash guard.
- **IRQ context** (defensive only): falls back to the PRIMASK blackout, since NVIC masking cannot guarantee the stall alarm preempts the current exception frame.

Since the output rings landed (2026-10-10), every slot's DMA free-runs with no interrupt at all, so the slots keep clocking through the window whatever is masked. What the kept alarm protects is the content: the flash primitives open an output-ring stall bracket (see "Output Ring Stall Bracket"), and its alarm keeps stamping correctly framed silence ahead of every reader for the whole window. The IEC 192-frame block structure therefore stays continuous, and no slot misses or repeats a frame, so **inter-slot alignment during and after a flash window is identical to steady-state streaming**. Before the rings, the two output DMA IRQ lines were the kept lines, and their handlers chained faded pool content and then per-instance silence buffers.

Why the alarm is safe while flash is unavailable: the vector table is the SDK's RAM copy (VTOR), the handler is installed with `irq_set_exclusive_handler()` into that table, and `output_rings_stall_irq`, `output_rings_stamp_silence_ahead()` and the ring libraries' silence pumps are all RAM-resident. Every word a DMA reads during the window is RAM-resident too: the slot rings, the RP2040 reload words (`spdif_ring_words_const`, `i2s_ring_words_const`), `adat_ring_base_holder`, and the soft VCXO's `fbdivs` / `fbdivs_addr`. `scripts/check_ram_placement.py` enforces all of it: Check A (hot symbols and DMA-read words in RAM), Check B (no branch from the RAM closure into flash), and Check B2, whose `FLASH_WINDOW_ROOTS` scan walks the alarm's call graph for flash-range literal-pool words and hard-fails if the root is not RAM-resident.

What stays masked: every other timer alarm (so `pico_spdif_rx`'s decode-timeout alarm, which uses the default alarm pool's own alarm and IRQ, cannot fire on the blackout edge, which historically crashed the core), USB (the host just NAKs; the ring is drained and the feedback controller re-seeded afterwards), the RP2350 word-1 peripherals (UART/I2C control interfaces, ADC, SPI1, see above), and everything else. SPDIF RX and I2S RX are stopped by the caller's flash bracket anyway.

**PDM through the window.** PDM's ring DMA is IRQ-less and free-running, so it keeps clocking regardless, but Core 1 (its producer) is parked, so it would loop the last ~45 ms of modulator output. `pdm_flash_silence()` fills the ring with the modulator's `0xAAAAAAAA` 50 % duty silence pattern and sets `pdm_force_reanchor`; content only, so the DMA, PIO and every pointer are untouched and PDM phase stays continuous. On resume the processing loop unconditionally re-seats its write lead to `read + TARGET_LEAD`: the ring lapped an unknown number of times while Core 1 was parked, and the loop's modulo write-read delta cannot distinguish a multi-lap underrun from a valid lead (the `delta > half-ring` test misses roughly half the outcomes), so without this PDM could resume with an arbitrary wrong lead. It is called only when the lockout actually parked Core 1 — a still-running Core 1 keeps filling the ring itself, and a forced re-anchor would then be a gratuitous phase jump.

**ADAT bulk output through the window (RP2350).** Its data/control DMA chain is IRQ-less and free-running, so the lightpipe stays framed. Since 2026-10-11 the output-ring stall alarm also runs `adat_output_keepalive()`, which keeps the ADAT ring topped up with level-neutral silence for the whole window, and the skip mirror keeps ADAT's offset to the slots exact when the slots re-anchor afterwards (see "ADAT Bulk Output", Alignment). The pool-era starvation rebaseline (`adat_output_rebaseline_starvations()`) is gone.

**Completion policy (2026-07-23): every runtime flash write ends in a synchronized output restart.** All flash-write call sites complete via `complete_flash_write_operation_full()`: preset save/delete and the legacy save path as before, and now also every metadata-only directory write (preset rename, startup policy, output-config mode/save, master-volume mode/save, UART/I2C control-interface config, Control Surface save, DAC-mute config). The former `complete_flash_write_operation_light()`, which skipped the output rebuild because DSP/output topology was unchanged, was removed. Rationale: during the ~45 ms IRQ blackout the output DMA handlers could not re-arm, the PIO TX FIFOs drained within tens of microseconds of the in-flight buffer completing, and every output SM stalled with BCK/LRCLK/DATA frozen mid-frame; the light path then resumed those clocks mid-frame with no DAC mute cycle and no restart. External DACs that derive their system clock from BCK (e.g. PCM5102 with SCK grounded, BCK PLL mode) can exit that halt mis-locked and render all subsequent, perfectly correct samples as full-scale distortion until the clocks are stopped and restarted cleanly. Field-reported (2026-07-22) as persistent "full scale noise resembling the music" after a preset rename and after a Control Surface save (Win10 USB source, I2S output to a PCM5102); pressing preset save fixed it because that path already ran the full completion. **The clock halt itself is gone as of 2026-07-25** (selective NVIC blackout, above), so that failure mode no longer exists; the unified completion is nevertheless retained, because it is what refills the drained pools from a deterministic synchronized state and is the shared owner of the feedback reset and the DAC mute release. It preserves the inviolable inter-slot alignment: `complete_pipeline_reset()` restarts every SM in sync, and for SPDIF/I2S/ADAT sources the input's prefill/re-lock handshake performs the equivalent synchronized restart. Dropping the restart for topology-unchanged writes is separate future work (no-teardown completions). One exception (2026-10-10): when deleting the active preset makes `apply_restored_outputs()` run an output transition, that transition is the synchronized restart, and the handler calls only `complete_flash_write_resume_inputs()` (the SPDIF/I2S input resume) instead of the full completion. Output type switches (`process_type_switches()`) and output data-pin changes (`process_pin_changes()`) need no equivalent treatment: they perform no flash writes and already end in a synchronized restart; their persistence commands (output-config save/mode) are among the metadata writes covered above.

### Preset System (replaces single-sector storage)

The firmware uses a 10-slot preset system. A preset is always active — there is no "no preset" state. Each slot can be either configured (has user data in flash) or unconfigured (loads factory defaults). Presets are stored in individual 4 KB flash sectors with a separate directory sector for metadata. Slot 0 has the default name "Default".
*Last updated: 2026-04-26*

### Flash Layout

Last 12 sectors (48 KB) of flash:

| Sector | Offset from end | Magic | Purpose |
|--------|-----------------|-------|---------|
| 0 | -48 KB | `0x44535032` ("DSP2") | Preset Directory (metadata, names, startup config) |
| 1-10 | -44 KB to -8 KB | `0x44535033` ("DSP3") | Preset Slots 0-9 (full DSP state) |
| 11 | -4 KB | `0x44535031` ("DSP1") | Legacy sector (migration source) |

### Preset Directory Fields (Version 23)
*Last updated: 2026-10-23 (V23 appends the 4-byte output latency block). Previously 2026-09-22 (V22 appends the 112-byte output limiter block; 2026-09-07: V21 drops the V20 auxiliary output table and restores the V19 layout; 2026-09-05: V20 appended that table; 2026-08-12: V19 appends the display config and page table)*

`DIR_VERSION_CURRENT` = 23. V4 renamed the former `include_pins` byte to
`output_config_mode` (same offset, 1:1 value mapping) and appended the
device-global `FlashOutputConfig` block. V5 grew that block by 3 bytes for the
I2S multichannel input pins (`i2s_rx_pin_ext[3]`). V6 appends the device-level
external control-interface config: an 8-byte `UartCtrlConfig` and an 8-byte
`I2cCtrlConfig`. `dir_load_cache()` migrates V1..V5 forward: the V5-to-V6 step
validates the V5 CRC, copies every field, then seeds the two new blocks with
`ctrl_iface_defaults()` (both disabled, default pins/baud/address) and reflushes
as V6. On every load, `dir_sanitize_ctrl_iface()` bounds-checks the two structs
(enabled 0/1, pins <= 29, baud in range, I2C address 0x08..0x77) and resets any
implausible one to defaults; deeper mux/collision checks run at apply time. Both
blocks are device-level (a listening profile, not per-preset) and **survive a
factory reset**, exactly like `dac_hw_mute`. V7 appends a 132-byte
`CsFlashConfig` (Control Surfaces bindings): the V6-to-V7 step validates the V6
CRC, copies every field, leaves the new block zeroed (every slot `CS_TYPE_NONE`
= feature idle), and reflushes as V7. `dir_sanitize_cs_config()` bounds-checks it
on load (an implausible blob version resets the whole block; an implausible
binding resets that slot; action-mask/pin-collision checks run at apply time).
Like the other board-level blocks it survives factory reset. V8 grew
`output_config` by 2 bytes for ADAT bulk output (`adat_enabled`, `adat_pin`). V9
upgrades the embedded Control Surfaces config from format v1 (132 bytes: 8x
16-byte bindings) to format v2 (388 bytes: 16x 24-byte bindings); the on-flash
v1 layout is frozen as `CsFlashConfig_v1` and translated field-by-field via
`cs_config_from_v1()`. Both older versions fan in to V9: the V8->V9 step copies
every field forward and upgrades `cs_config`, while V7->V9 additionally widens
the 23-byte v7 `output_config` into the 25-byte field (new ADAT bytes zeroed).
`dir_sanitize_cs_config()` now also range-checks each binding's `event`. V10
appends the per-slot Control Surfaces names (`cs_names[16][32]`, 512 bytes):
user labels set via `REQ_SET_CS_NAME` (0x8B) and read via `REQ_GET_CS_NAME`
(0x8C), slot metadata independent of the binding table (survive binding
changes; may be set before a binding exists). The V9->V10 step validates the
V9 CRC (frozen `PresetDirectory_v9` snapshot), copies every field forward, and
leaves the new block zeroed (all slots unnamed); `dir_sanitize_cs_config()`
additionally forces NUL termination on every name at load. V11
appends the Control Surfaces IR command table (132-byte `CsIrConfig`: version
+ 8x 16-byte `IrCommand`), device-global beside `cs_config` for the same
board-level reasons; the V10->V11 step validates the V10 CRC (frozen
`PresetDirectory_v10` snapshot), copies every field forward, and leaves the
new block zeroed (every sub-slot empty = feature idle).
`dir_sanitize_cs_ir()` bounds-checks it on load like `dir_sanitize_cs_config`.
The Control Surfaces bindings and IR table are persisted together in one
write by `preset_set_cs_all` (the `REQ_CS_SAVE` path; per-binding SETs no
longer persist). V12 through V16 each grow the embedded `output_config`
(optional SPDIF inputs 2/3, I2S clock mode, I2S clock-pin mode, ADAT input,
SPDIF input 4 pin); each has a frozen `PresetDirectory_vN` snapshot and a
prefix-memcpy widening step, with the zero-filled tail bytes chosen so they
mean "legacy default". V17 upgrades the IR command table from format v1 (132
bytes: 8 sub-slots) to v2 (260 bytes: 16), doubling the learnable remote
buttons. `cs_ir` is the last directory member, so no earlier field moves: the
V16->V17 step validates the V16 CRC (frozen `PresetDirectory_v16` snapshot),
copies the whole pre-`cs_ir` prefix in one memcpy (guarded by a `_Static_assert`
that the two prefixes match), and widens the block via `cs_ir_from_v1()`, which
carries the 8 learned commands over verbatim and leaves sub-slots 8-15 empty.
Every pre-V17 migration reads the old block through the frozen `CsIrConfig_v1`
and widens it the same way. This growth makes
`sizeof(PresetDirectory)` 1571 bytes at V17 (1443 at V16, 1433 at V11). V18
appends the Control Surfaces target-group table (324-byte `CsGroupConfig`) then
the macro table (1060-byte `CsMacroConfig`) after `cs_ir`, both device-global
beside the rest of the CS config, taking `sizeof(PresetDirectory)` to 2955
bytes; still within the single 4 KB directory sector. The V17 layout is
byte-identical to the V18 prefix (frozen `PresetDirectory_v17` snapshot plus
`_Static_assert`s pinning the geometry), so the V17->V18 step is a prefix copy
with the two new blobs zero-filled, which means "no groups, no macros";
`dir_sanitize_cs_groups()` and `dir_sanitize_cs_macros()` bound-check the
blob versions, `target_kind` and `step_count` on every load and zero
implausible entries (`member_mask` range and step records are
platform-dependent, so they validate at apply / fire time instead). V19
appends the Control Surfaces display blob (80-byte `CsDisplayFlash`: version +
12-byte `CsDisplayCfg` + 16x 4-byte `CsDisplayPage`) after `cs_macros`, taking
`sizeof(PresetDirectory)` to 3035 bytes of the 4 KB sector. The V18 layout is
byte-identical to the V19 prefix (frozen `PresetDirectory_v18` snapshot plus
`_Static_assert`s pinning the 2955-byte geometry and the `cs_macros` offset),
so the V18->V19 step is a prefix copy with the new blob zero-filled, which
means "display idle, no pages"; page seeding happens only when a display
binding is first applied, never at migration, so a migrated device shows
nothing until configured. `dir_sanitize_cs_display()` resets the whole blob on
an implausible version or dirty reserved bytes, clamps `mode` and `home_page`,
and clears any page carrying unknown flag bits (page nouns are
platform-dependent and validate at apply / render time). V20 appended a
292-byte Control Surfaces auxiliary output table after `cs_display`, taking
`sizeof(PresetDirectory)` to 3327 bytes. **V21 removes it again**, because
auxiliary outputs became binding-slot components (caps v18) rather than a
separate table, so `PresetDirectory` is back to 3035 bytes of the 4 KB sector
and V21 is byte-identical to V19. The V19->V21
step therefore copies the whole V19 data block unchanged (frozen
`PresetDirectory_v19` snapshot plus `_Static_assert`s pinning the 3035-byte
geometry and the `cs_display` offset). The V20->V21 step verifies the V20 CRC,
then copies the V20 data block up to (and excluding) the dropped 292-byte
block and discards it, so any aux output configured on a V20 build is lost and
must be recreated as a binding-slot component. `PresetDirectory_v20` is kept as
a frozen 3327-byte migration-only layout with the dropped block declared as an
opaque `uint8_t[292]`; `dir_sanitize_cs_aux()` is gone with the table it
guarded. The V19 `dir_sanitize_cs_display()` mask was also
missing `CS_DPAGE_BAR`, which wiped saved level-bar pages at every boot. It
now accepts the flag. See
`Documentation/Features/output_config_independent_load.md`,
`Documentation/Features/control_interfaces_spec.md`,
`Documentation/Features/control_surfaces_spec.md`,
`Documentation/Features/control_surfaces_display_spec.md`, and
`Documentation/Features/control_surfaces_aux_spec.md`.

**V23** (2026-10-23) appends `FlashOutputLatencyConfig output_latency` (4 bytes:
version, option ID, 2 reserved) directly after `limiter`, taking
`sizeof(PresetDirectory)` to 3151 bytes. V22 migrates by a whole-block copy of
its 3147 bytes (`PRESET_DIRECTORY_V22_SIZE`), and every older path already
zero-fills the cache, so `dir_sanitize_output_latency()` (run beside
`dir_sanitize_limiter()` on every path) seeds the default option (4 ms) when
the block is unset or holds an unknown option ID. A V23 directory read by older
firmware has no migration and is treated as missing (defaults, rewritten on the
next save), so downgrading loses directory settings. The other unmerged V23
users (`overclock`, `ac3-dts-decode`) must renumber when they land.

**V22** appends `FlashLimiterConfig limiter` (112 bytes) directly after
`cs_display`, taking `sizeof(PresetDirectory)` to 3147 bytes. It is the
independent-mode store for the output limiter settings (see "Output Limiter").
`DIR_VERSION_CURRENT` is 22. V19 and V21 directories migrate straight to V22 by
the same whole-block copy, and V20 by its pre-`cs_aux` prefix. Older chains
land the same way. Every path leaves the block at version 0, and
`dir_sanitize_limiter()`, which runs before every `dir_cache_valid = true`,
seeds the defaults (all off, -1 dBFS, 100 ms, unlinked). A stored block is
sanitized by the audio path's rule: NaN takes the default, then values clamp.
`_Static_assert`s pin the block at 112 bytes and at the offset where the V19
layout ends.

| Field | Description |
|-------|-------------|
| startup_mode | 0 = load specified default, 1 = load last active |
| default_slot | Slot to load in "specified default" mode (0-9) |
| last_active_slot | Last slot loaded/saved (always 0-9) |
| output_config_mode | Physical IO persistence mode (was `include_pins`): 1 = with preset (default), 0 = independent (device-global). Governs output pins/types, I2S MCK/BCK, SPDIF RX pin, and (since V22) the output limiter settings. |
| slot_occupied | 16-bit bitmask (bit N = slot N has valid data) |
| master_volume_mode | 0 = independent (default, mode 0 saved-to-directory), 1 = with preset (was include_master_volume) |
| spdif_rx_pin | Device-level SPDIF RX GPIO pin (legacy; superseded by `output_config.spdif_rx_pin`) |
| master_volume_db | Independent master volume (mode 0 source); float, default -20 dB |
| slot_names[10][32] | 32-byte NUL-terminated names per slot |
| dac_hw_mute | DAC hardware-mute config (V3+, board-level) |
| output_config | Device-global `FlashOutputConfig` (V4+, +3 bytes at V5): output pins/types, I2S BCK/MCK pin/enable/multiplier, SPDIF RX pin, I2S RX multichannel pins; the source of truth in independent mode |
| uart_ctrl | UART control-interface config (V6+, 8 bytes; `enabled=0` by default; survives factory reset) |
| i2c_ctrl | I2C target control-interface config (V6+, 8 bytes; `enabled=0` by default; survives factory reset) |
| cs_config | Control Surfaces bindings (V7 format v1 = 132 B / 8x 16-byte; V9 format v2 = 388-byte `CsFlashConfig`: version + 16x 24-byte `CsBinding`; all-zero = idle; board-level, survives factory reset) |
| cs_names[16][32] | Per-slot Control Surfaces names (V10+): 32-byte NUL-terminated user labels, independent of the bindings; all-zero = unnamed; board-level, survives factory reset |
| cs_ir | Control Surfaces IR command table (V11 format v1 = 132 B / 8x 16-byte; V17 format v2 = 260-byte `CsIrConfig`: version + 16x 16-byte `IrCommand`; all-zero = every sub-slot empty = idle; board-level, survives factory reset) |
| cs_groups | Control Surfaces target groups (V18+, 324-byte `CsGroupConfig`: version + 8x 40-byte `CsGroup`; all-zero = no groups; board-level, survives factory reset) |
| cs_macros | Control Surfaces macros (V18+, 1060-byte `CsMacroConfig`: version + 8x 132-byte `CsMacro`, each 32-byte name + step count + 8x 12-byte `CsMacroStep`; all-zero = no macros; board-level, survives factory reset) |
| cs_display | Control Surfaces display config and pages (V19+, 80-byte `CsDisplayFlash`: version + 12-byte `CsDisplayCfg` + 16x 4-byte `CsDisplayPage`; all-zero = display idle, no pages; board-level, survives factory reset) |
| output_latency | Output latency (V23+, 4-byte `FlashOutputLatencyConfig`: version (0 = never saved) + option ID (`OutputLatencyId`, 0 = unset) + 2 reserved; device-global, survives factory reset; see "Output Latency") |
| limiter | Output limiter settings for independent mode (V22+, 112-byte `FlashLimiterConfig`: version byte (0 = never saved) + 3 reserved + 9x 12-byte records of enabled, link group, threshold, release; RP2040 uses the first 5). Written by `REQ_SAVE_OUTPUT_CONFIG`, applied at boot in independent mode only |

Auxiliary outputs have no directory field of their own. Since caps v18 they
are `CS_TYPE_AUX_OUT` / `CS_TYPE_AUX_PWM` components inside `cs_config`, named
through `cs_names` like any other slot. (Historical: V20 alone carried a
292-byte `cs_aux` table here, dropped at V21.)

### Preset Slot Data (Version 12)
*Last updated: 2026-09-22 (output limiter, slot V39; `SLOT_DATA_VERSION` now 39; 2026-09-20: tube slot tail revised in place: `tube_xfmr_damping` / `tube_xfmr_res_hz` plus a reserved float, still 48 bytes at V38; 2026-09-18: tube preamp, slot V38; `SLOT_DATA_VERSION` now 38; 2026-09-04: subharm third band / selectivity / ceiling / link, slot V37)*

| Field | Description |
|-------|-------------|
| Magic | 0x44535033 ("DSP3") |
| Version | 12 |
| slot_index | Sanity-check slot number |
| CRC32 | Integrity check over data section |
| EQ recipes | NUM_CHANNELS x 12 bands |
| Preamp | `preamp_db` (legacy single value) + `preamp_db_per_ch[NUM_INPUT_CHANNELS]` (V12+) |
| Master volume | `master_volume_db` (V12+, -128 to 0 dB, -128 = mute) |
| Bypass | master bypass flag |
| Delays | NUM_CHANNELS delay values |
| Legacy gain/mute | 3 channels (backward compatibility) |
| Loudness | enabled, reference SPL, intensity |
| Crossfeed | enabled, preset, ITD, custom fc/feed, output_pair_mask (V27+, tail-appended; older slots default 0x01) |
| Psychoacoustic bass | enabled, output_mask, cutoff, harmonics, drive, character, original (V31+, tail-appended 24 bytes, `SLOT_DATA_VERSION` 31; older slots load disabled/all-outputs defaults) |
| Subharmonic synthesizer | enabled, output_mask, low_db, high_db, boost_db (V36+, tail-appended 16 bytes); top_db, select_depth, select_hold_ms, ceiling_db, select_mode, link_pairs (V37+, tail-appended 20 more bytes, `SLOT_DATA_VERSION` 37). Pre-V36 slots load disabled/all-outputs defaults; V36 slots load the `SUBHARM_DEFAULT_*` values for the V37 fields. `solo` is runtime-only and never stored |
| Tube preamp | enabled, tube_type, rectifier, xfmr_enabled, output_mask, nine floats and one reserved float (V38+, tail-appended 48 bytes, `SLOT_DATA_VERSION` 38). Pre-V38 slots load the `TUBE_DEFAULT_*` values with the effect off. The stored character fields are restored verbatim, never re-derived from the tube_type row, so a preset keeps its voicing across firmware |
| Output limiter | per output: enabled, link_group, threshold_db, release_ms (V39+, tail-appended 10 bytes x `NUM_OUTPUT_CHANNELS`, `SLOT_DATA_VERSION` 39). Pre-V39 slots load every limiter off at the defaults |
| Stereo upmixer | enabled, centre/surround modes, presence_q1 (V34+, int8 dB * 2, was reserved), ten floats (V33+, tail-appended 44 bytes; `SLOT_DATA_VERSION` now 34, size unchanged from V33; RP2350 only, gated on version >= 33; older slots load disabled defaults; RP2040 stores zeros and never applies them) |
| Matrix mixer | crosspoints + output channels |
| Pin config | NUM_PIN_OUTPUTS pin assignments (always stored, conditionally loaded) |
| Channel names | NUM_CHANNELS × 32-byte NUL-terminated names (V8, default names for V<8) |

### Boot Sequence

1. Read Preset Directory from flash
2. If valid: load slot based on startup_mode (specified default or last active)
3. If target slot empty/corrupt: apply factory defaults, keep slot selected
4. If no directory: attempt legacy migration (copy old single-sector data into slot 0)
5. If no legacy data: create fresh directory, select slot 0 with factory defaults
6. Always results in an active preset (never "no preset")

### Legacy Migration

On first boot after firmware upgrade, if the old `0x44535031` ("DSP1") magic is found in the last sector but no preset directory exists, the firmware automatically migrates the old data into preset slot 0 (named "Migrated") and sets it as the default.

### Legacy API Redirect

- `REQ_SAVE_PARAMS` (0x51): saves to the active preset slot
- `REQ_LOAD_PARAMS` (0x52): **removed and the opcode repurposed** as `REQ_SAVE_OUTPUT_CONFIG` (2026-06-01). It was a deprecated synchronous "revert to saved" that crashed on SPDIF input; hosts use the deferred, SPDIF-safe `REQ_PRESET_LOAD` (0x91) instead. See the Output-Config Persistence section.
- `REQ_FACTORY_RESET` (0x53): resets live state to defaults, active slot unchanged

### Preset-Switch Mute & Pipeline Reset
*Last updated: 2026-07-25 (pipeline reset now fades fully out before the teardown and fades back up after the restart)*

**Observed fade-out before any disruptive work (2026-07-25).** Arming the mute envelope does not make the wire silent. The envelope (`update_preset_mute_envelope()`, `audio_pipeline.c`) only advances when a packet is processed, its gain is applied *ahead* of the per-output delay lines, and the resulting samples still have to cross the consumer queues before the DMA plays them. The bracket used to arm the mute and tear down in the same main-loop iteration, so everything in flight was discarded and the clocks stopped on a full-level sample — the click the bracket exists to prevent. `main.c` now runs a shared, restartable settle state machine, `pipeline_fade_to_silence_poll()`, which reports complete only once (1) a processed packet has rendered the envelope down to zero and (2) every enabled slot ring has actually played those zeros. When the envelope first reads zero, `output_rings_silence_mark_arm()` latches each slot's write position, and the poll then waits until `output_rings_silence_mark_reached()` sees every reader past that position plus the longest active output delay line (`pipeline_max_active_delay_samples()`, which includes the limiter lookahead). The wait is measured from the ring pointers, so it is exact at every rate. A ring reset after the mark was latched counts as silent, since a reset ring restarts from stamped silence. PDM and ADAT need no separate check, because both stay sample-aligned to the slots. *Updated 2026-10-10: the measured mark replaced a wall-clock dwell sized from the old consumer-pool depth.*

The machine drives the envelope through a second, independent countdown (`pipeline_request_soft_mute()`), never through `preset_loading`: that flag doubles as the SPDIF/I2S/ADAT prefill handshake, and setting it early would make the main-loop lock blocks tear the outputs down at the wrong moment. The request is refreshed by the waiter each iteration and expires on its own (`PIPELINE_FADE_REQUEST_MS`, 40 ms of audio), so a fade that is armed and then abandoned unmutes instead of sticking. Two call paths use it:

- `pipeline_reset_ready()`, the non-blocking pre-gate the deferred handlers already poll, fades first and asserts the DAC hardware mute only afterwards. An analog mute engaged at full level would truncate the very ramp it exists to cover, and the step reappeared when the pin deasserted.
- `prepare_pipeline_reset()` spins on the same machine while servicing the active producer (`pipeline_settle_to_silence()`), so callers that do not pre-gate (boot, rate changes driven by an input's lock machinery, source switches, the flash bracket) still fade. It exits on the first test when the caller already settled, and immediately when no producer is running — in that case the envelope is latched at zero (`pipeline_latch_mute_silence()`) because no packet will ever arrive to advance it. `process_type_switches()` runs the settle *before* it masks the USB IRQ, since the fade needs the ring to keep refilling. The whole settle is bounded by `PIPELINE_FADE_CAP_US` (200 ms) for a producer that stalls after being declared live.

**Fade back up.** `complete_pipeline_reset()` Phase 3.5 holds the envelope at zero for `output_ring_target_frames` samples (the rings' steady-state fill, which the first admitted block re-anchors to; runtime since 2026-10-23) plus any configured `dac_hw_mute_release_ms()` after the synchronized restart, then the normal 8 ms ramp runs. Without the dwell the ramp could complete while the analog mute is still asserted (a step when it releases). *Updated 2026-10-10: this was a fixed 24 ms sized for consumer-pool refill.* It is a floor, never a shortening: the SPDIF/I2S/ADAT prefill handshakes clear the counter when they enable outputs and fade up through the audio they prefilled instead. `prepare_pipeline_reset()` floors its own mute counter at `PIPELINE_OP_MUTE_FLOOR_MS` (24 ms) so a 5 ms `PRESET_MUTE_SAMPLES` cannot expire mid-operation and start fading up into the disruptive window. A zero-length packet no longer snaps the envelope to its target. It renders nothing, so it must not skip the ramp.

Alignment is unaffected: the settle only adds ordinary shared-pipeline packets (every slot, PDM, delay index and DSP state advances by the same sample count), the envelope is one gain shared by both cores, and the teardown/synchronized-restart sequence is unchanged.

All preset operations (load, save, delete) are **deferred from the USB IRQ to the main loop** via pending flags (`preset_load_pending`, `preset_save_pending`, `preset_delete_pending` in `usb_audio.c`). This avoids running flash operations inside the USB ISR (which would cause a ~45ms interrupt blackout inside an interrupt handler) and allows proper pipeline reset bracketing.

**SPDIF-RX suspend across disruptive state swaps (required when SPDIF is the active input).** Any deferred handler that mutates the live DSP state (coefficients, matrix, delays, output types/pins) or tears down output slots must first stop the SPDIF receiver and restart it afterward. While RX runs, `pico_spdif_rx`'s decode-timeout alarm fires on a separate timer IRQ and can touch PIO/DMA state mid-mutation; left running, this races the swap and faults the core, which the 8-second `watchdog_enable()` then resets (RAM is re-initialised, so `buffer_stats_sequence` resets to ~0 — the signature of this reboot). `preset_load_pending`, `process_type_switches`, and `process_pin_changes` have always bracketed RX this way. The `bulk_params_pending` (`REQ_SET_ALL_PARAMS` / 0xA1) and `factory_reset_pending` (`REQ_FACTORY_RESET` / 0x53) handlers were **missing this bracket** and would reboot the device when applied with SPDIF input active (intermittent, ~4/6 under repeated `0xA1`). Both now mirror `preset_load`: after `prepare_pipeline_reset()`, `if (active_input_source == INPUT_SOURCE_SPDIF && spdif_input_get_state() != SPDIF_INPUT_INACTIVE) { spdif_input_stop(); spdif_prefilling = false; }`, and at the end restart via `spdif_input_start()` (guarded by `!input_source_change_pending`, since a bulk-applied source change defers RX management to that handler). `process_type_switches` leaves caller-stopped RX alone (`rx_was_running` check), so the change-mask path doesn't double-start. Verified: 0/10 reboots on `0xA1` with SPDIF locked after the fix (was 4/6).

The main loop handler for each operation follows the pattern:
1. `usb_audio_drain_ring()` — process in-flight audio packets
2. `prepare_pipeline_reset(PRESET_MUTE_SAMPLES)` — fade the wire to silence, wait for Core 1 idle, engage mute
3. Execute the preset operation (`preset_load/save/delete`)
4. `complete_pipeline_reset()` — drain stale consumer buffers, resync outputs, reset USB feedback

**Delay line zeroing:** `preset_load()` clears all delay line buffers (`memset(delay_lines, 0, ...)`) after `dsp_update_delay_samples()` to prevent stale audio from the previous preset's delay configuration bleeding through.

**Feedback recovery:** `flash_program_staged()` (called during save/delete) reseeds the feedback controller at nominal after the ~45ms interrupt blackout. `complete_pipeline_reset()` (called after load/delete) also resets feedback state.

**Underrun suppression:** All underrun/overrun counters are suppressed while `preset_loading` is true, preventing erroneous counts during intentional pipeline disruption.

### Operations

**Save:** drain ring → prepare reset → collect live state → build PresetSlot → CRC32 → flash erase + program → update directory

**Load:** drain ring → prepare reset → validate CRC + apply user data (or factory defaults) → recalculate filters/delays → zero delay lines → transition Core 1 mode → update directory → complete pipeline reset (drain stale buffers, resync outputs)

**Delete:** Engage mute → erase slot sector (feedback reset + re-mute) → update directory (feedback reset + re-mute) → if active slot: apply factory defaults + recalculate filters/delays + transition Core 1 mode (active slot selection unchanged)

### Channel Names — Type/Source-Aware Defaults
*Last updated: 2026-04-29*

Default channel names are derived from current device state, not hard-coded:

- **Input channels (0..7):** named by `active_input_source` (`get_default_channel_name()`, `usb_audio.c`), each source in its natural model:
  - **USB** - discrete channels `"USB 1"` .. `"USB 8"` (a USB stream's channels are independent, not stereo pairs, so per-channel numbers, no L/R).
  - **I2S** - stereo pairs `"I2S 1 L"`, `"I2S 1 R"`, `"I2S 2 L"`, ... (pair = `ch/2 + 1`; matches the output naming style; I2S input is 1..4 pairs).
  - **S/PDIF** - a single stereo pair `"SPDIF L"` / `"SPDIF R"`.

  On an input-source switch the default-name regeneration in `main.c` covers all `NUM_INPUT_CHANNELS`, so multichannel inputs relabel; custom names are preserved by string-inequality. Future per-channel sources fall through to the USB-style branch.
- **Output slot channels:** labelled by `output_types[slot]` — `"SPDIF N L/R"` for `OUTPUT_TYPE_SPDIF`, `"I2S N L/R"` for `OUTPUT_TYPE_I2S`, where N is 1-based slot index.
- **PDM (last channel):** always `"PDM"`.

`get_default_channel_name(int ch, uint8_t input_source, const uint8_t *output_types, char *buf)` (`usb_audio.c`) computes a default given a snapshot. `output_types` may be NULL (treated as all-SPDIF) for input/PDM channels or for fallback callers.

**Auto-regen on retype/source change:** `process_type_switches` (`main.c`) and the input-source deferred handler (`main.c`) regenerate the affected channel names *only if the live name still matches the would-be old default*. User customisations like `"Living Room Sub"` are preserved by string-inequality. Regeneration emits `notify_param_write` so the host UI updates live. RAM-only on event; persistence is via `REQ_PRESET_SAVE`. State is consistent on power loss because both the type/source and the name live in the same slot.

**Same persistence model as `output_pins[]` and `spdif_rx_pin`:** RAM-only changes that are persisted only when the user saves. Names are *not* part of the physical-IO config block (`output_config_mode`) — they have their own slot bytes (V8+) and always travel with presets.

**Heuristic note:** A user who renames a channel to the literal current default string (e.g., `"USB L"`) is treated as "default" and the name will re-default on the next event. Acceptable; the host UI can encourage non-default strings if stickiness matters.

**Bulk SET semantics:** `bulk_params_apply` overwrites `channel_names[]` directly from the wire payload — host bulk-writes are authoritative and bypass the regen heuristic.

---

## Pin Configuration
*Last updated: 2026-07-06 (ADAT bulk output default GPIO 12 added; the ADAT pin
is reserved in `is_pin_in_use` while ADAT is config-enabled)*

The debug UART was removed entirely (stdio_uart off), freeing GPIO 16/17; they
are no longer reserved and are now the default pins for the UART control
interface. GPIO 18/19 are the default I2C control-interface pins. Live
control-interface pins are reserved against all other pin-assignment commands
(they participate in `is_pin_in_use`); see the External Control Interfaces
section.

### Default Assignments

**RP2350:**

| GPIO | Function | Output |
|------|----------|--------|
| 6 | S/PDIF 1 | Outputs 1-2 |
| 7 | S/PDIF 2 | Outputs 3-4 |
| 8 | S/PDIF 3 | Outputs 5-6 |
| 9 | S/PDIF 4 | Outputs 7-8 |
| 10 | PDM Sub | Output 9 |
| 12 | ADAT bulk output (when enabled) | Outputs 1-8 mirror |
| 16 | UART control TX (default; disabled until enabled over USB) | Control |
| 17 | UART control RX (default) | Control |
| 18 | I2C control SDA (default; disabled until enabled over USB) | Control |
| 19 | I2C control SCL (default) | Control |
| 25 | LED | Heartbeat |

**RP2040:**

| GPIO | Function | Output |
|------|----------|--------|
| 6 | S/PDIF 1 | Outputs 1-2 |
| 7 | S/PDIF 2 | Outputs 3-4 |
| 10 | PDM Sub | Output 5 |
| 16 | UART control TX (default; disabled until enabled over USB) | Control |
| 17 | UART control RX (default) | Control |
| 18 | I2C control SDA (default; disabled until enabled over USB) | Control |
| 19 | I2C control SCL (default) | Control |
| 25 | LED | Heartbeat |

### Runtime Reconfiguration
*Last updated: 2026-10-10 (slot re-inits align every output clock to the locked servo's divider before the synchronized restart; restores and boot move pins through one coordinated output transition; PDM hold handshake; PDM pin change keeps the live rate; 2026-07-15: universal PIN_RESET_TO_DEFAULT 0xFF escape hatch across every single-pin SET command)*

Vendor commands `REQ_SET_OUTPUT_PIN` (0x7C) / `REQ_GET_OUTPUT_PIN` (0x7D).

**Constraints:** Valid GPIO range, not reserved (23-25 power/LED; GPIO 16/17 are no longer reserved now that the debug UART is gone), and not in use by another output, the I2S BCK/LRCLK or MCK pin, the SPDIF RX pin, the DAC hardware-mute pin, or a live UART/I2C control-interface pin (`is_pin_in_use`).

**Reset-to-default escape hatch (`PIN_RESET_TO_DEFAULT`, 0xFF):** every single-pin SET command — `REQ_SET_OUTPUT_PIN` (0x7C), `REQ_SET_I2S_BCK_PIN` (0xC2, per role), `REQ_SET_MCK_PIN` (0xC6), `REQ_SET_ADAT_PIN` (0xCC), `REQ_SET_SPDIF_RX_PIN` (0xE4, per index), `REQ_SET_ADAT_INPUT_PIN` (0x6A), `REQ_SET_I2S_RX_PIN` (0xF1, per pair) — maps a pin byte of 0xFF to the platform default for the addressed target before running its normal validation chain, so a reset can still fail (e.g. PIN_IN_USE if the default has been claimed). 0xFF is not a valid GPIO on either platform, so pin byte 0 always means GPIO 0; the previous `REQ_SET_ADAT_PIN` behavior of treating 0 as "reset to default 12" is gone (0 now addresses GPIO 0 like everywhere else). ADAT input has no free default GPIO, so its "default" is unset: 0xFF clears the pin (only while the input is disabled, since an enabled input with no pin would be unselectable). The persistence layers are unchanged: flash/bulk zero-fill "0 = default/absent" decoding is a migration artifact of zero-filled struct tails, not a wire semantic of the SET commands.

**S/PDIF and I2S slots:** Deferred to the main loop, not applied in the USB ISR. `REQ_SET_OUTPUT_PIN` writes the target into `output_pins[out_idx]` (RAM-only, like `spdif_rx_pin`) and sets a bit in `output_pin_change_mask`; the main-loop gate (shared `pipeline_reset_ready()` hold) then runs `process_pin_changes(mask)`. That helper mirrors `process_type_switches`: `prepare_pipeline_reset()` (soft mute + Core 1 fence + DAC hardware-mute assert) → suspend SPDIF RX if running → `drain_and_disable_outputs()` → `audio_spdif_change_pin()` / `audio_i2s_change_data_pin()` on each flagged slot while disabled → `complete_pipeline_reset()` for the **synchronized** restart of all slots → restart RX. The synchronized restart is the point: a moved slot re-enters in phase with the others, preserving the inviolable inter-slot sample alignment. (The prior implementation restarted only the changed slot live in the ISR, which clicked and left that slot phase-misaligned until the next full reset.) `change_pin` masks the channel's DMA IRQ before aborting the DMA so the handler can't start a conflicting transfer during PIO SM reinit, and clears any stale completion flag before unmasking. Back-to-back requests accumulate in the mask and apply in one batch. Persistence follows `output_config_mode`: in with-preset mode the pin travels with the preset (`REQ_PRESET_SAVE` captures it, applied on load); in independent mode it is device-global (`REQ_SAVE_OUTPUT_CONFIG` persists it, applied at boot).

**PDM:** Applied inline (must be disabled first; rebuilds PIO config). PDM has no running audio to realign at change time, so it needs no deferral or synchronized restart. The vendor path also refuses while `pdm_hw_is_active()`, i.e. while Core 1 is still fading the sub out after a disable. `pdm_change_pin()` restores the PIO clock at `audio_state.freq` (it hard-coded 48 kHz before 2026-10-10, which left the sub at the wrong rate after a pin change at 44.1 or 96 kHz).

**Restores and boot: one coordinated output transition.** Preset load, deleting the active preset, factory reset and bulk SET install a whole output config (`io_config_apply()` / `bulk_params_apply()` only write globals). Each handler ends in `apply_restored_outputs(old_types)`, which runs `process_type_switches()` over every slot. Boot uses the same call after `usb_sound_card_init()` creates every slot as SPDIF on the default pins. `process_type_switches()` compares the live hardware with the config (slot types, I2S clocking, each slot's data pin, the MCK pin and the PDM pin) and returns false without disruption if nothing differs; `apply_restored_outputs()` then moves a stopped PDM's pin, applies the MCK policy (which also re-pins a stopped MCK), and the handler runs its usual completion. A running MCK or sub whose pin moves forces the transition; a stopped one is bookkeeping. When preset delete runs a transition it still calls `complete_flash_write_resume_inputs()`, the input/ADAT half of the flash completion. Otherwise it performs one muted transition, in this order.

1. Settle to silence. If the PDM pin moves, `pdm_hold_hw()` raises a hold and waits for Core 1 to acknowledge that the PDM SM/DMA are stopped (its normal fade-out, bounded by `PDM_HOLD_TIMEOUT_US`). This happens before the USB IRQ is masked.
2. Mask the USB and output DMA IRQs, suspend SPDIF RX and I2S input, quiesce every slot.
3. Release every outgoing pin: ADAT output's pin if the config moves or disables it, each slot's old data pin if it moves, and the BCK/LRCLK pair of an I2S clock master unless the new master keeps the same pair.
4. Tear down every instance that will be rebuilt: same-type I2S slots whose clocking changes, then slots whose type changes (Pass 1). The library teardowns release their own data pin (and, for the last I2S instance, the clock pair), so they belong to the release stage too.
5. Move the single-owner pins: MCK (`audio_i2s_mck_apply_state(pin, false, ...)` releases the old pin and records the new) and PDM (`pdm_change_pin()` while held).
6. Claim: set up new types and rebuilt I2S clocking on `output_pins[]` (Pass 2); same-type slots whose pin moved use `audio_spdif_attach_pin()` / `audio_i2s_attach_data_pin()`, which claim the new pin without releasing the old.
7. MCK policy through `apply_mck_policy()` (on only when not slave-clocked, any I2S output or input is in use and MCK is enabled; the no-transition path applies the same rule), then `complete_pipeline_reset()` puts every slot on one divider (a locked input servo's trim, else nominal; see "Output Clock Restore") and restarts every slot in sync (ADAT resyncs onto its configured pin), release the PDM hold, restore IRQs, restart the inputs.

Releasing every outgoing pin before claiming any incoming one is what makes pin swaps safe. That covers two outputs exchanging pins, an output moving onto a pin MCK, PDM or ADAT leaves, and the reverse. Every release also checks that the pad is still muxed to the releasing owner (`release_slot_gpio()`, the library's `spdif_release_pin()` / `i2s_release_pin()`, MCK disable, `pdm_change_pin()`, and ADAT's existing `adat_release_pin()`), so a claim made before the transition by something outside it is never taken back. The bulk SET's DAC hardware-mute claim inside `bulk_params_apply()` is one example. Boot runs its transition before `dac_hw_mute_init()` for the same reason. The previous handler tails moved only slots whose type changed, so a same-type slot stayed on its old GPIO until reboot, and boot moved SPDIF pins one at a time with `audio_spdif_change_pin()`, whose release of each old pin could disconnect a slot that had just moved onto it. The vendor single-pin command keeps its own `process_pin_changes()` path; its validation forbids pins another output owns, so it cannot produce a swap.

**PDM hold handshake.** `pdm_hold_hw()` stores `pdm_hold`, issues a barrier and waits for `pdm_hw_active` to clear. Core 1 runs the PDM hardware only while `pdm_enabled && !pdm_hold`; before restarting the hardware it stores `pdm_hw_active`, barriers, and backs off if `pdm_hold` is set, so the two cores can never both believe they own the SM/DMA. Core 1 clears `pdm_hw_active` after stopping the SM and aborting DMA. `pdm_release_hw()` drops the hold, and Core 1 restarts through its normal re-anchored enable path if PDM is still enabled.

### Restore Order in `io_config_apply()` and `bulk_params_apply()`
*Last updated: 2026-10-02*

`io_config_apply()` (flash_storage.c) installs a stored `FlashOutputConfig` into the live pin globals at boot and on preset loads. `bulk_params_apply()` (bulk_params.c) does the same for a pushed `WireBulkParams`. Apart from the DAC hardware-mute GPIO claim in the bulk path, which is unrelated to the output slots, both only set globals. The audio hardware is rebuilt later in the normal pipeline-reset bracket, so the install order has no effect on slot timing.

The install order does matter for validation. Each pin check (`i2s_mck_pin_acceptable()`, `i2s_bck_pin_acceptable()`, `spdif_input_enable_acceptable()`, the I2S RX set check, the DAC-mute collision check, the ADAT checks) compares the incoming pin against whatever is live at that moment. A field that has not been installed yet still holds its previous value, which at boot is the compile-time default. Both paths therefore install or release pins first, then the I2S clocks, then the claims that check against the clocks.

1. Output pins. The flash path also installs output types here.
2. SPDIF RX pin 1, the optional SPDIF input 2..4 pins, and the optional-input disable pass.
3. ADAT release (RP2350). An enabled ADAT output or input that the incoming config disables or moves is switched off, so later checks are not judged against its old pin. The live ADAT input source is never released. Snapshots taken here let the ADAT claims restore `adat_output_config_dirty` and skip the ADAT input restart when the net result is unchanged.
4. MCK pin, enable and multiplier. The bulk path installs output types first. A non-GPOUT pin falls back to the default with MCK off. An enabled MCK on a pin another fixed peripheral owns is forced off.
5. Master BCK/LRCLK, then clock-pin mode and the slave BCK pair.
6. The optional-input enable pass, then the I2S RX data pins and channel count.
7. Bulk path only: the DAC hardware mute. Its collision check sees the final clocks, and only the ADAT pins that were kept.
8. ADAT output claim, then ADAT input claim (RP2350). Output goes first so the loopback share (ADAT input may sit on an enabled ADAT output's pin) sees the final output pin. A rejected ADAT output pin falls back to an acceptable pin or ADAT output is disabled. An ADAT input being newly enabled on its unchanged stored pin is checked too; on conflict, the flash path leaves it off and keeps the pin.

With one exception, each pair of claims is checked by whichever claim is installed second. For example, MCK against BCK is caught by the BCK check, BCK against an ADAT pin by the ADAT check, and the DAC mute against ADAT by the DAC-mute check. The exception is the clock pairs. Steps 6 to 8 check against them through `is_pin_in_use()`, which reserves BCK/LRCLK and the slave pair only while an I2S output or I2S input is active. A restore can therefore place an optional SPDIF input, ADAT pin or DAC mute on an idle clock pin, exactly as the live vendor commands allow. Only the I2S RX data pins, and MCK through the later BCK check, are always checked against the clock pairs.

In `bulk_params_apply()`, the SPDIF steps, the ADAT input release and claim, and the I2S RX step run only when `apply_pins` is true. Output types, MCK, BCK, clock-pin mode, the slave pair, the DAC mute and ADAT output run on every push, as they did before the reorder.

Before this order was established, the flash path checked BCK before installing SPDIF RX and MCK. At boot the BCK check then saw SPDIF RX on its default GPIO 5. A saved BCK of GPIO 4 (LRCLK 5) was rejected on every power-up and fell back to GPIO 14, even when the saved config had moved SPDIF RX elsewhere. Any new pin field added to either path must be installed before every check that reads it.

**Known residuals.**
- A rejected BCK, slave pair or RX set falls back to the live value, and that value is not re-checked against the newly installed pins.
- An optional SPDIF input that stays enabled while its pin moves is not re-checked.
- In the bulk path, the MCK and BCK checks still see the old DAC-mute pin, so a payload that moves the mute off a pin while moving MCK or BCK onto it is falsely rejected.
- `REQ_SET_OUTPUT_TYPE` does not check the clock pins when it turns an idle pair into a driven one.

**MCK enable.** MCK claims its pin only while enabled, so another function can take `i2s_mck_pin` while MCK is off. `REQ_SET_MCK_ENABLE` (0xC4) therefore checks `is_pin_in_use()` before enabling, and on a conflict returns PIN_IN_USE with MCK left off. Like `REQ_SET_MCK_PIN`, it reserves the clock pairs only while I2S is active. The restore paths use `i2s_mck_pin_acceptable()` instead, and rely on the later BCK check for the clock pair.

**ADAT loopback share.** `adat_input_pin_acceptable()` lets ADAT input share the ADAT output pin only while ADAT output is enabled. A disabled output's stored pin gets the normal ownership check.

---

## Core 1 Architecture
*Last updated: 2026-02-15*

### Operating Modes

```c
typedef enum {
    CORE1_MODE_IDLE      = 0,
    CORE1_MODE_PDM       = 1,
    CORE1_MODE_EQ_WORKER = 2,
} Core1Mode;
```

### Mode Selection

Determined at boot and runtime based on output enables:
- **PDM mode:** PDM sub output (last output) enabled
- **EQ_WORKER mode:** Any SPDIF output in Core 1 range enabled AND PDM disabled (RP2040: outputs 2-3, RP2350: outputs 2-7)
- **IDLE:** Neither condition met

**Mutual exclusion:** PDM sub and EQ worker outputs cannot coexist on either platform, enforced in `REQ_SET_OUTPUT_ENABLE`. RP2040: outputs 2-3 conflict with PDM. RP2350: outputs 2-7 conflict with PDM.

### EQ Worker (Both Platforms)
*Last updated: 2026-07-13 (RP2350 slot finalization mode selected per packet via core1_eq_work.finalize_s24)*

Core 0 processes input pipeline + matrix mix, then dispatches per-output work to Core 1.
Core 1 processes assigned SPDIF outputs in parallel: EQ, gain, delay, and S/PDIF conversion.
On RP2350 the slot finalization mode is snapshotted once per packet by Core 0
from `adat_output_is_active()` and passed to Core 1 as
`core1_eq_work.finalize_s24`, so both cores always agree. ADAT active:
`eq_worker_loop` (`pdm_generator.c`) converts its `buf_out` rows 2-7 to S24 in
place via `output_block_to_s24_inplace()` (the single conversion point shared
with ADAT; runs even when a slot pool is starved) and then interleaves pairs
1-3 as a pure integer copy; Core 0 symmetrically converts rows 0-1 and
interleaves pair 0, and the single-core fallback converts rows 0-7 and
interleaves pairs 0-3. ADAT inactive: both cores skip the staging pass and use
the fused `output_pair_convert_interleave()` per pair (rows stay float),
avoiding the second memory pass; slot bytes are bit-identical in both modes.

| Platform | Core 0 outputs | Core 1 outputs | spdif_out[] size |
|----------|----------------|----------------|------------------|
| RP2350 | 0-1 (pair 1) | 2-7 (pairs 2-4) | 3 |
| RP2040 | 0-1 (pair 1) | 2-3 (pair 2) | 1 |

**Handshake:**
```c
typedef struct {
    volatile bool work_ready;
    volatile bool work_done;
#if PICO_RP2350
    float (*buf_out)[192];
    float vol_mul_start;       // Master-scaled vol at sample 0 (linear ramp)
    float vol_mul_step;        // Per-sample increment
    int16_t *spdif_out[3];
#else
    int32_t (*buf_out)[192];
    int32_t vol_mul_start;     // Q15 master-scaled vol at sample 0
    int32_t vol_mul_step;      // Q15 per-sample increment
    int16_t *spdif_out[1];
#endif
    uint32_t sample_count;
    uint32_t delay_write_idx;
} Core1EqWork;
```

Uses `__dmb()` memory barriers + `__sev()` / `__wfe()` for low-latency synchronization.

**Platform differences in EQ worker:**
- RP2350: float pipeline, block-based hybrid SVF/biquad EQ via `dsp_process_channel_block()` (single-precision)
- RP2040: int32_t Q28 pipeline, **block-based** EQ via `dsp_process_channel_block()` (assembly in `dsp_process_rp2040.S`)

### PDM Mode (Both Platforms)

Core 1 runs sigma-delta modulation loop, popping samples from ring buffer and writing PDM bitstream to DMA buffer.

### CPU Load Tracking
*Last updated: 2026-04-12*

- Budget-based metering: `load = busy_us / (sample_count / sample_rate)`, reported as EMA (7/8 retention) via `global_status.cpu0_load` / `cpu1_load`
- Immune to bursty calling patterns (SPDIF RX DMA delivers 192-sample blocks every ~4ms; previous idle-time approach clamped inter-block gaps to zero → permanent 100%)
- Core 0: measured in `process_input_block()` (`audio_pipeline.c`)
- Core 1 EQ worker: same budget approach using `audio_state.freq` (`pdm_generator.c`)
- Core 1 PDM: accumulates active_us over 48-sample windows (already budget-based)
- Metering reset (`pipeline_reset_cpu_metering()`) called on: USB audio gap detection, input source switch away from SPDIF, and SPDIF lock loss

---

## RP2040 vs RP2350 Comparison
*Last updated: 2026-10-23 (DSP Processing table: sample full scale and metering row; Hardware table: slot rings halved to 8 KB, output latency row). Previously 2026-10-17 (Hardware table: USB input bit depth and AS alt rows add alt 6 = 4ch/24-bit). Previously, 2026-10-11 (Hardware table: RAM rows re-measured after the SPDIF RX overhaul merge, slot ring row). Previously, 2026-10-10 (Volume leveller row: safety limiter; Delay Lines table: RP2040 ring is 1024 samples, cap is ring size minus one; 2026-09-28: Control Surfaces nouns row: caps v20, 79 nouns; tube preamp row: drive -30..+24 dB, RP2040 shifted domains, kernel 1,196 B; 2026-09-22: output limiter row; wire/slot row V32/V39; 2026-09-20: tube preamp row: damping-factor output stage replaces the transformer stage, RP2040 bell clamps and multiply count; 2026-09-19: tube preamp row: RP2040 shaper-output clamp and kernel size 1,212 B; current .data and free-RAM figures; 2026-09-18: tube preamp row: RP2040 operand-sum headroom rule, multiply count and kernel size; wire/slot row V31/V38; 2026-09-12: 6th-order bass bands and RAM cost; continuous bass bank, V3 and RAM costs; spectrum analyser row: FFT ceiling lowered to 1024 points, RAM cost revised; 2026-09-07: LF FFT replaced by continuous bank, protocol V2 and RAM cost; 2026-09-04: subharm row: new parameters and per-output sub meter; wire/slot row V30/V37)*

### Hardware

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| CPU | Dual Cortex-M0+ @ 133 MHz (OC to 307.2 MHz) | Dual Cortex-M33 @ 150 MHz (OC to 307.2 MHz) |
| SRAM | 264 KB | 520 KB |
| FPU | None (software float) | Single-precision VFP |
| DCP | N/A | Double-precision coprocessor |
| VREG | 1.20V (for OC) | 1.10V |
| UART + I2C external control | Yes (identical) | Yes (identical) |
| Control Surfaces nouns (caps v20) | 79 in table, `ADAT_ACTIVE` + the 6 upmixer nouns unusable (empty action mask) | 79, all usable |
| Control Surfaces IR sub-slots | 16 (`CS_MAX_IR_COMMANDS`) | 16 (identical) |
| Binary type | `default` (XIP) | `default` (XIP) |
| Cold code location (control paths, storage, coeff design, init) | Flash XIP | Flash XIP |
| RAM code+rodata+data (.data) | 67,984 B of the 72,704 B budget (2026-10-12) | 95,800 B of the 100,352 B budget (2026-10-12) |
| Free heap (`__HeapLimit - end`; nothing allocates at runtime) | 42,772 B (2026-10-12) | 77,144 B (2026-10-12) |
| Output slot staging (`slot_stage`, BSS) | 2 × 1.5 KB | 4 × 1.5 KB |
| Output slot rings | 2 × 8 KB `.slot_rings` at the top of RAM (512 frames) | 4 × 8 KB `.slot_rings` at the top of RAM (512 frames) |
| Output latency (ring fill target) | Option table: low latency 0.67 ms (stream inputs), 2/3/4/6/8 ms, default 4 ms; identical on both | Same; 4-8 ms clamp to 3.66 ms at 96 kHz on both |
| Custom XIP linker script | `memmap_dspi_rp2040_xip.ld` (+divider/int64/bit-ops IN_RAM defines) | `memmap_dspi_rp2350_xip.ld` |

### DSP Processing

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Data type | Q28 fixed-point | IEEE 754 float |
| Sample full scale | 2^29 (Q28 value 2.0) | 1.0 |
| Peak meter / clip threshold | `peak_to_meter_q28()` (`>> 14`, clamped to 32767); `CLIP_THRESH_Q28` = 2^29 + 0.1% | `fminf(1, pk) * 32767`; `CLIP_THRESH_F` = 1.001 |
| Accumulator | int32/int64 | float (single-precision) |
| Filter architecture | TDF2 biquad only | Hybrid SVF/biquad (SVF below Fs/7.5, TDF2 above) |
| Biquad impl | Block-based assembly (`dsp_process_rp2040.S`) | C, per-type SVF specialization |
| Processing mode | Block-based (two-phase) | Block-based |
| EQ bands (master) | 10 | 10 |
| EQ bands (output) | 10 | 10 |
| Crossover bands (per output) | 4 | 4 |
| Max crossover sections per band | 4 | 4 |
| Max biquads (PEQ + crossover worst case) | 70 + 80 = 150 | 110 + 144 = 254 |
| Input channels (NUM_INPUT_CHANNELS) | 2 | 8 (active 2/4/6/8 by alt) |
| NUM_CHANNELS (inputs + outputs) | 7 | 17 |
| Per-input EQ + metering | 2 inputs | up to 8 inputs (active count) |
| Matrix outputs | 5 | 9 |
| S/PDIF outputs | 2 pairs | 4 pairs |
| ADAT bulk output | No | Yes (8 ch mirror of outputs 1-8; 44.1/48 kHz, auto-suspends above) |
| USB input channels | 2 (stereo) | 2 / 4 / 6 / 8 |
| I2S input channels | 2 (stereo) | 2 / 4 / 6 / 8 (configurable) |
| ADAT input | Config state only (never selectable) | Yes (8 ch, 24-bit, 44.1/48 kHz; master/slave clock; PIO1 SM2 + DMA CH15) |
| Input capture arena (shared SPDIF FIFO / I2S rings / ADAT ring) | 12,288 B, 4096-aligned (SPDIF FIFO is the largest member) | 32,768 B, 8192-aligned (the four I2S rings are the largest member) |
| USB input bit depth | 16-bit or 24-bit (alt) | 16/24-bit (stereo and 4ch) or 16-bit (6ch/8ch) |
| AS alt settings | 0, 1 (16-bit), 2 (24-bit) | 0, 1, 2, 3 (4ch/16), 4 (6ch/16), 5 (8ch/16), 6 (4ch/24) |
| Wire / slot version | V32 / V39 | V32 / V39 |
| S/PDIF bit depth | 24-bit | 24-bit |
| S/PDIF input conversion | 24-bit sign-extended full-scale → Q28 via `>> 2` (equivalent to `sample << 6`) | 24-bit sign-extended full-scale → float via `÷ 2147483648.0f` |
| S/PDIF output conversion | Q28 >> 6 → int24 | float × 8388607 → int24 |
| Volume leveller | Q28 envelope + float gain; 2 channels, 2 lookahead rings; max boost clamped to 18 dB (Q28 gain range); safety limiter in Q28 gains, floored steps, float divide only in `lim_cap_of` | Float throughout; 2 to 8 active channels, mask-driven, 8 lookahead rings; full 35 dB max boost; safety limiter in float. Both platforms: same block caps, hold and release (see "Safety Limiter") |
| Loudness | Per output, post-gain: 2 Q28 shelf biquads; `loudness_output_mask` (5 outputs) | Per output, post-gain: 2 SVF shelves; `loudness_output_mask` (9 outputs). Both platforms: volume-keyed, works in stereo and multichannel input modes |
| Crossfeed | Per output pair, post-matrix (PASS 4.5); 2 pairs; `output_pair_mask` (default pair 1) | Per output pair, post-matrix (PASS 4.5); 4 pairs; `output_pair_mask` (default pair 1). Both platforms: shared coeffs, per-pair state, works in every input mode |
| Psychoacoustic bass | Per output, pre-crossover; RBJ Q28 biquads (with pre-drive low-band clamp) | Per output, pre-crossover; TPT SVF float. Both platforms: missing-fundamental NLD, `output_mask`, zero added latency |
| Subharmonic synthesizer | Per output, pre-crossover, ahead of psybass; same kernel in Q28 through `fast_mul_q28` (band clamp before the divider); 10-byte sub meter (5 outputs) | Per output, pre-crossover, ahead of psybass; same kernel in float; 18-byte sub meter (9 outputs). Both platforms: TPT SVF band split, hysteresis octave dividers, phase-aligned sum, LF bell, `output_mask`, selectivity, sub ceiling, pair link, runtime solo, headroom reading, zero added latency |
| Tube preamp | Per output, pre-crossover, after psybass; same kernel in Q28 through `fast_mul_q28`, whose operand magnitudes must sum below 8.0 (drive and shaper-output terms in shifted domains chosen per coefficient set, clamps on the shaper input, driven value, shaper output, DC-blocker output, bell input and bell output, and the wet signal); 11 multiplies per sample base, 12 to 19 typical; 1,196 B of shared RAM text | Per output, pre-crossover, after psybass; same kernel in float, with no headroom clamps at all; 660 B of shared RAM text. Both platforms: biased asymmetric waveshaper with a blended knee hardness, supply sag, DC blocker, optional damping-factor output stage (bell at the speaker resonance plus a 2.5 kHz top shelf), 16 tube-type rows, drive -30..+24 dB with automatic makeup gain, `output_mask`, zero added latency |
| Output limiter | Per output, post-loudness, pre-delay; Q30 gains on raw Q28 levels (full scale 2^29), one hardware divide per 16-sample block, two-multiply Q15 apply; 5 outputs; 2,444 B RAM text, 1,238 B BSS | Per output, post-loudness, pre-delay; float; 9 outputs; 2,200 B RAM text, 2,126 B BSS. Both platforms: 32-sample lookahead engaged on every output only while any limiter is on (switched in silence), no overshoot, link groups with a per-packet cross-core meeting when a group spans both cores |
| Spectrum analyser (RTA) | Q15 `int16_t` kernel; default order 9 (512 points), max 10 (1024); measured per-bin dynamic range 78 dB (`RtaCaps.dynamic_range_db` = 78); 5 tracked channels; Q27 continuous 10–200 Hz bass bank (6th-order bands) with 64-bit power; ~6.9 KB analyser BSS | Float kernel; default order 10 (1024 points), max 10 (1024); dynamic range 120 dB, limited by the wire level byte rather than arithmetic; 9 tracked channels; float continuous 10–200 Hz bass bank (6th-order bands); ~11.2 KB analyser BSS |
| Stereo upmixer | Not available (compiled out; matrix untouched) | Stereo input only: derives C/Ls/Rs into matrix rows 2..4 (passive/adaptive/off centre; off/passive/adaptive surround). Zero-latency steering; deliberate per-row surround Haas delay |
| EQ channels | 7 (NUM_CHANNELS) | 11 (NUM_CHANNELS) |

### Delay Lines

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Channels | 5 | 9 |
| Type | int32_t | float |
| Ring size | 1024 | 2048 |
| Max delay (48kHz) | 21.3 ms (1023 samples) | 42.6 ms (2047 samples) |
| RAM usage | 20 KB | 72 KB |

### Core 1 Usage

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| PDM mode | Yes | Yes |
| EQ worker mode | Yes (outputs 2-3) | Yes (outputs 2-7) |
| Parallel EQ | Core 0: input + 0-1, Core 1: 2-3 | Core 0: input + 0-1, Core 1: 2-7 |
| Parallel crossover (V11+) | Same dispatch as PEQ — Core 1 owns its output range | Same dispatch as PEQ |
| EQ worker data type | int32_t Q28, block-based | float, block-based, hybrid SVF/biquad |
| EQ cascade structure | Band-major: one buffer sweep per band (assembly) | Paired: adjacent 2nd-order bands on the same path (biquad or SVF) fused into one sweep with per-type specialized arms (`dsp_cascade_block`) |
| Crossover-stage availability when PDM enabled | Single-core (Core 0 only) | Single-core (Core 0 only) |

### DMA
*Last updated: 2026-07-23 (output rings free-running: zero output DMA IRQs; RP2040 adds reload channels 6-7)*

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Priority | Global bus priority bits | Per-channel high-priority flag |
| Output TX channels (SPDIF **or** I2S, per slot) | 0-1 data + 6-7 reload (`OUTPUT_SLOT_RELOAD_DMA_BASE` 6 + slot) | 0-3 (ENDLESS, no reload) |
| Output TX IRQ (SPDIF + I2S) | None (free-running ring) | None (free-running ring) |
| SPDIF RX channels | CH4, CH5 | CH5, CH6 |
| SPDIF RX IRQ | DMA_IRQ_1, highest priority, the only DMA IRQ user | DMA_IRQ_1, highest priority, the only DMA IRQ user |
| I2S RX channels | CH4, CH5 (1 pair, data+reload; shared with SPDIF RX) | CH5 + CH6 + CH7 + CH8 (up to 4 pairs; one ENDLESS-mode channel each, no reload; pair 0's CH5 shared with SPDIF RX) |
| I2S RX IRQ | IRQ-less (chained ring) | IRQ-less (ENDLESS rings) |
| PDM channel | Dynamic (`dma_claim_unused_channel`) | Dynamic (`dma_claim_unused_channel`) |
| Soft VCXO channels | CH10 (PULSE) + CH11 (PACE), DMA timer 0, IRQ-less | CH11 (PULSE) + CH12 (PACE), DMA timer 0, IRQ-less |
| ADAT output channels | N/A | CH13 (data) + CH14 (control), IRQ-less chained ring |
| ADAT input channel | N/A | CH15 (RX ring, ENDLESS mode, no IRQ, no reload channel) |

**Per-slot DMA channel sharing (2026-06-27).** Each output slot owns exactly one
DMA channel (channel index == slot index), used by whichever output type is
currently active on that slot. The S/PDIF and I2S libraries each claim that
channel on setup and release it on teardown (`audio_spdif_teardown` /
`audio_i2s_teardown`), so an output-type switch hands the same channel from one
library to the other; the slot's PIO SM (index == slot index, both on
`PICO_AUDIO_SPDIF_PIO`) is handed over the same way. Previously S/PDIF TX held
channels 0-3 permanently (claimed at boot, never released) while I2S TX used a
disjoint, hardcoded range (8-11); the I2S range is now gone, freeing the high
DMA channels for input and clock use. On RP2040 each output slot also owns a
free-running reload channel (`OUTPUT_SLOT_RELOAD_DMA_BASE` 6 + slot) that
rewrites the ring read address at wrap; RP2350 uses ENDLESS mode and needs no
reload channel. Current RP2350 occupancy: outputs 0-3,
PDM 4, SPDIF RX 5-6 (or I2S RX 5-8 when the multichannel input runs), spare
9-10, soft VCXO 11-12, ADAT out 13-14, ADAT in 15. RP2040: outputs 0-1 (+ reload
pair 6-7), PDM 2, spare 3, RX 4-5, spare 8-9, soft VCXO 10-11. The RP2350 I2S RX rings were
converted from data+reload channel pairs (which spanned 5-12) to a single
ENDLESS-mode channel per pair (5-8, the same write-address-wrap pattern the ADAT
RX ring uses) to free 9-12; RP2040 has no ENDLESS mode and keeps its data+reload
pair (4/5), stereo-only. `_Static_assert`s in `i2s_input.c` pin the I2S RX
range below `SOFT_VCXO_DMA_PULSE` on both platforms. S/PDIF TX and I2S TX
share a slot's channel safely because neither enables a DMA IRQ, and a retype
runs synchronously on the main loop, so the brief window in which the channel
is unclaimed cannot race a `dma_claim_unused_channel` consumer (PDM).

### Clock Configuration

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| sys_clk (all rate families) | 307.2 MHz (VCO 1536 MHz / 5) | 307.2 MHz (VCO 1536 MHz / 5) |
| 44.1 kHz family | Fractional 16.8 PIO divider at fixed 307.2 MHz sys_clk (no clock switching) | Same |
| PLL config | Manual (`set_sys_clock_pll()`) | Automatic (`set_sys_clock_hz()`) |
| I2S BCK/LRCLK default (master + unified pair) | GPIO 14/15 | GPIO 14/15 |
| I2S slave-pair BCK/LRCLK default (SPLIT clock-pin mode; `PICO_I2S_BCK_PIN_SLAVE`) | GPIO 12/13 | GPIO 26/27 |
| Input clock tracking | Soft VCXO (ppm sys_clk trim; see "Soft VCXO") | Soft VCXO (ppm sys_clk trim; see "Soft VCXO") |
| Soft VCXO PWM pacer slice (no GPIO) | Slice 7 (reserved) | Slice 8 (reserved) |

---

## Memory Layout

> **Output rings halved (2026-10-23).** `PICO_AUDIO_SPDIF_RING_FRAMES` /
> `PICO_AUDIO_I2S_RING_FRAMES` go from 1024 to 512 frames and the erase-ahead
> from 192 to 64 frames, so each slot ring is 8 KB and `SLOT_RINGS_BYTES` halves
> to 16 KB (RP2040) / 32 KB (RP2350). The ADAT TX ring follows the slot ring's
> writable span (96 + 448 = 544-frame lead bound) and drops from 960 to 576
> frames (30 KB to 18 KB). Saving: 16 KB on RP2040, 44 KB on RP2350. Measured:
> `.data` 69,128 B / 97,232 B, `.bss` 112,936 B / 271,076 B plus `.slot_rings`
> 16 KB / 32 KB, free heap before runtime allocation 59,096 B / 122,140 B
> (RP2040 / RP2350). See "Output Latency" for why the ring size bounds the
> latency ceiling.

> **Producer pools removed (2026-10-12).** The `pico_audio` producer pools are
> gone, and nothing allocates from the heap at runtime any more (no `malloc` is
> linked). The pools held 8 × 192-frame buffers per slot, about 12.7 KB each with
> headers, so the change returns about 25.5 KB of heap on RP2040 and 51 KB on
> RP2350. The pipeline now renders into `slot_stage` (BSS, 1.5 KB per slot) and
> copies admitted blocks straight into the slot rings. Measured with diagnostics
> off, against `9cc9f1e`: `.data` 67,984 B / 95,800 B (down 2,368 B / 2,240 B, the
> RAM-resident pool code), `.bss` 112,876 B / 283,304 B (up 2,484 B / 5,660 B:
> staging less the instances' connection state), and free heap 42,772 B /
> 77,144 B (RP2040 / RP2350). Usable RAM after boot rises from about 20 KB to
> 42.8 KB on RP2040 and from about 24 KB to 77.1 KB on RP2350.

> **SPDIF RX overhaul merge (2026-10-11).** The consumer pools (24 KB RP2040,
> 48 KB RP2350) are gone; each output slot plays from a 16 KB ring in a NOLOAD
> `.slot_rings` section placed at the top of RAM by both linker scripts, so its
> 16 KB alignment costs no padding (as a tentative definition it landed in
> COMMON and dragged all of `.bss` onto 16 KB boundaries, leaving RP2040 too
> little heap for its producer pools). The heap ends below it: both
> `__HeapLimit` and `__StackLimit` are set to the section's address, because the
> SDK's `_sbrk` and `pico_malloc` bound the heap by `__StackLimit`, and the merge
> at first moved only `__HeapLimit`, which left malloc free to grow into the
> rings (fixed 2026-10-11 after the post-merge audit). The ADAT TX ring
> grew from 896 to 960 frames (+2 KB) to hold its cushion plus a full slot
> ring. Measured with diagnostics off: `.data` 70,064 B / 97,560 B against new
> budgets of 72,704 B / 100,352 B (RP2040 / RP2350); `.bss` 110,392 B /
> 277,644 B plus `.slot_rings` 32 KB / 64 KB; free heap before runtime
> allocation (`__HeapLimit - end`, now reported by `check_ram_placement.py`)
> 45,256 B / 82,804 B, leaving about 19 KB / 31 KB after the producer pools.
*Last updated: 2026-10-16 (USB feedback long-window rate, dual-slope servo and diagnostics: `fb_ctrl` grows to 84 B, BSS +56 B both platforms, RAM code +416 B RP2040 / +408 B RP2350, measured `.data` 68,496 B / 96,328 B, inside the 72,704 B / 100,352 B budgets). Previously 2026-10-12 (producer pools removed: heap unused, `slot_stage` staging in BSS; see the note above). Previously 2026-10-11 (SPDIF RX overhaul merge: output rings at the top of RAM, ADAT ring 960 frames, `DATA_BUDGET` 72,704 B / 100,352 B, `__StackLimit` bounded below the rings; see the note below). Previously, 2026-10-10 (leveller safety limiter: `LevellerState` +56 B both platforms, RAM code +336 B RP2040 / +496 B RP2350, measured `.data` 67,440 B and 94,144 B (64 B under the RP2350 budget); 2026-09-28: RP2040 tube kernel 1,196 B after the drive-floor change, BSS unchanged; 2026-09-22: directory V22: dir_cache 3,035 -> 3,147 B; buffer cleanup: notify rebaseline scratch removed, preset save builds in the flash staging buffer, bulk buffer sized to `WireBulkParams`: BSS -10,880 B RP2040 / -14,648 B RP2350; crossover cascades output-indexed: BSS -1,058 B RP2040 / -9,352 B RP2350; output limiter: BSS +1,238 B RP2040 / +2,126 B RP2350, RAM text +2,444 B / +2,200 B, `.data` budgets raised 2 KB each; 2026-09-19: RP2040 tube kernel 1,212 B after the shaper-output clamp, was 1,220 B; RP2350 tube kernel branch-free, 660 B; 2026-09-18 tube preamp: BSS +460 B RP2040 / +560 B RP2350, shared kernel RAM text (branch-free RP2350 rewrite 2026-09-19, was 858 B); current .data, BSS and free-RAM figures for both platforms; 2026-09-12: shared elliptic b0: BSS -16 B both platforms, RP2350 RAM code +256 B from -O3; bass bands 6th-order: BSS +1,456 B RP2040 / +2,352 B RP2350; continuous bass bank: BSS +1,580 B RP2040 / +2,000 B RP2350; spectrum analyser FFT ceiling lowered to 1024 points: BSS -2,560 B RP2040 / -4,608 B RP2350, flash -3.5 KB / -6.6 KB; 2026-09-07: auxiliary outputs reworked as binding-slot components: +~130 B BSS both platforms, preset directory back to 3035 B at V21; 2026-08-12: Control Surfaces display, +~750 B BSS both platforms, ~12 KB flash)*

> **Crossover cascades output-indexed (2026-09-22).** `xover_filters` and the
> crossover bypass flags dropped their input-channel rows, which no path could
> use. BSS falls by **1,058 B on RP2040** (`xover_filters` 3,696 B to 2,640 B) and
> **9,352 B on RP2350** (19,856 B to 10,512 B). Measured after the change, BSS is
> 151,412 B on RP2040 and 348,584 B on RP2350; free RAM is 43,660 B and 82,096 B.

> **Output limiter (2026-09-22).** The module adds **1,238 B of BSS on RP2040**
> (820 B of per-output state including five 128-byte lookahead rings, a 240 B
> envelope table, two 72-byte coefficient buffers) and **2,126 B on RP2350**
> (1,476 B of state with nine rings, a 432 B table, two 92-byte buffers). Its
> `DSP_TIME_CRITICAL` hooks are **2,444 B of RAM text on RP2040** and **2,200 B on
> RP2350**, and the per-output config adds 60 B / 108 B of `.data`. Measured after
> the change, `.data` is 67,056 B on RP2040 and 93,704 B on RP2350, which exceeded
> the old 65,536 B / 92,160 B budgets, so `DATA_BUDGET` in
> `scripts/check_ram_placement.py` is now 67,584 B / 94,208 B. BSS is 150,228 B on
> RP2040 and 355,616 B on RP2350; free RAM is 42,620 B and 72,648 B. The limiter
> rings are separate from the per-output delay lines, whose size is unchanged.

> **Tube preamp (2026-09-18).** The module adds **452 B of BSS on RP2040**
> (120 B of `tube_output_state` for 5 outputs, two `TubeCoeffs` buffers and the
> 48-byte config) and **552 B on RP2350** (216 B of state for 9 outputs, the same
> two buffers and the config). Its one shared `DSP_TIME_CRITICAL` kernel,
> `tube_process_output_block`, is **1,196 B of RAM text on RP2040** after the
> 2026-09-28 drive-floor change (1,212 B before it, 1,220 B before the 2026-09-19
> shaper-output clamp) and **660 B on
> RP2350**, paid once for all six per-output call sites. Measured after the change,
> `.data` is 64,448 B of the 65,536 B budget on RP2040 (was 62,816 B) and 91,288 B
> of 92,160 B on RP2350 (was 90,176 B), BSS is 150,940 B (was 150,480 B) and
> 355,480 B (was 354,920 B), and free RAM is 46,756 B (was 48,848 B) and 77,520 B
> (was 79,192 B). Both platforms stay inside the `check_ram_placement.py` `.data`
> budget. No audio buffers or delay lines change. See "Tube Preamp Emulation".

> **Spectrum analyser (2026-09-12).** Capture remains **2,048 B RP2040 /
> 4,096 B RP2350**, with a 529 B raw-bin frame. The continuous 10–200 Hz bank
> adds **1,580 B / 2,000 B BSS** over the single-FFT implementation, including
> protocol V3 and timing state. Sixth-order bass bands then added another
> **1,456 B / 2,352 B**. Analyzer BSS is now about **6,924 B / 11,200 B**.
> Bass state is 512 B per fixed-point channel (5 channels), 452 B per float
> channel (9 channels), plus a shared 684 B RAM coefficient block (the
> elliptic sections do not store b2). RP2350 compiles the bass kernel at
> `-O3 -ffp-contract=off`, which adds 256 B of RAM code. Removing
> duplicate bass EMA storage saves 56 B per channel. Band frames are 82 B,
> with 37 slots. RAM-pinned streaming code is additional to BSS; the RAM
> placement checker accounts for it in `.data`. The 1024-point ceiling still
> saves 2,560 B / 4,608 B versus 2048 points. No audio buffers or delay lines
> change. See "Spectrum Analyser (RTA / FFT)".

> **Input capture arena (2026-08-06).** The `pico_spdif_rx` FIFO (12 KB), the I2S
> RX rings (4 KB RP2040 / 32 KB RP2350) and the ADAT RX ring (8 KB, RP2350 only)
> are mutually exclusive and now overlay one BSS union, `input_capture_arena`
> (`.bss.input_capture_arena`, first in the alignment-sorted `.bss` cluster).
> Size is the largest member: **12,288 B on RP2040** (4096-aligned) and
> **32,768 B on RP2350** (8192-aligned), reclaiming 4,092 B and 20,480 B of BSS
> respectively. The explicit section keeps it out of COMMON, where its alignment
> padding would eat most of the saving. Ownership is a runtime-checked invariant;
> see "Input Capture Arena" under Audio Input Source System.

> **Soft VCXO (2026-07-22).** The clock-servo overhaul adds a negligible
> footprint: ~24 bytes of `.data` statics (the `fbdivs[]` excursion/nominal pair
> and its read-address pointer, kept in RAM so the DMA pulse train runs through
> flash blackout windows) plus ~0.3 KB of RAM-resident code (`soft_vcxo_set_ppm`,
> `DSP_TIME_CRITICAL`). It claims no BSS ring. See "Soft VCXO".

> **Linkwitz Transform target-Q sidecar (2026-07-12).** The Linkwitz Transform's
> fourth parameter (`Qp`) is stored out-of-band in a new BSS array
> `peq_qp_x512[NUM_CHANNELS][MAX_BANDS]` of `uint16` (`Q*512`): **408 B on RP2350**
> (17 channels x 12 bands x 2) and **168 B on RP2040** (7 x 12 x 2). Flash grows
> too: `PresetSlot` tail-appends a matching `peq_qp_x512` array, bumping
> `SLOT_DATA_VERSION` to 30 (V21..V29 slots still load, `qp` defaulting to 0 =
> 0.707). The array is zeroed at init and on preset load from a pre-V30 slot.

> **Multichannel volume leveller (2026-07-08).** The leveller now holds one
> lookahead ring per input channel (`LevellerState.lookahead_buf[NUM_INPUT_CHANNELS][240]`)
> and shortened the ring from 480 to 240 samples (5 ms at 48 kHz). On RP2350
> `LevellerState` is 7,732 bytes (eight rings); the change grew total BSS by
> ~3,880 bytes to 330,704 bytes. On RP2040 the two rings halved, shrinking BSS
> by ~1,904 bytes to 139,668 bytes. `leveller_process_block` stays
> `DSP_TIME_CRITICAL` and RAM-resident.

> **ADAT bulk output (2026-07-06, RP2350 only; encoder LUT added 2026-07-13).**
> The ADAT engine adds a fixed 30 KB BSS frame ring (960 frames x 32 bytes,
> `adat_ring` in adat_output.c; 896 frames / 28 KB before 2026-10-11) plus
> small state; the ring is sized to cover the alignment cushion (96 frames)
> plus a full slot ring's fill cap (1024 - 192 frames) so overwrite of
> unplayed frames is structurally impossible. The
> LUT-driven encoder adds 528 bytes of BSS (`adat_token_lut[256]` uint16 = 512 B,
> plus the per-entry-level header variants `adat_hdr_nrzi`/`adat_hdr_exit`),
> built once in `adat_output_init()`. The single float->S24 conversion point
> costs zero extra RAM: while ADAT is active it reuses `buf_out` rows 0-7 in
> place (the rows stay declared `float` but are read as `out_s24_t` =
> `may_alias` int32 after finalization); while ADAT is inactive the staging
> pass is skipped (fused per-pair convert+interleave). Only
> `adat_output_push_block()` and `adat_output_is_active()` (sampled per packet
> for the mode snapshot) are RAM-resident (`DSP_TIME_CRITICAL`); all other
> control paths stay cold in flash. RP2040 compiles the engine out entirely
> (zero cost). See "ADAT Bulk Output".

> **ADAT bulk input (2026-07-13, acquisition state updated 2026-07-15; ring moved into the shared arena 2026-08-06; RP2350 only).** The ADAT receiver uses a fixed
> **8 KB ring** (`adat_rx_ring`, 2048 x uint32, aligned to its own size for the
> DMA write-address wrap; since 2026-08-06 it is a member of the shared
> `input_capture_arena` and costs no BSS of its own) plus a few dozen bytes of state (including the current
> slave probe rate and its 64-bit start timestamp), and roughly **3 KB of
> RAM-pinned decode code**: the `DSP_TIME_CRITICAL` poll body, header check, frame
> decoder, slave-mode rate machine, and clock servo. The bounded sync-search scan
> and exact-timing acquisition probe stay cold in flash (they run only while the
> pipeline is muted). Ring and audio-buffer sizes are unchanged by the probe-based
> acquisition update.
> RP2040 compiles the receiver out entirely (config state only). This lifted the
> RP2350 `.data` budget in `scripts/check_ram_placement.py` from 64K to 72K. See
> "ADAT Input".

> **Test signal generator (2026-07-05).** The siggen subsystem adds a small amount
> of BSS (well under 1 KB: applied + staged `SiggenConfig`, three 44.1/48/96 kHz
> derived-parameter rows, per-channel RNG/pink state, and multitone/channel-ID
> increment tables) plus its RAM-resident render kernels (the synth kernels,
> planner, and `siggen_render` are `DSP_TIME_CRITICAL`). Control/config code stays
> cold in flash. See "Test Signal Generator".

> **DSPI_LOOPBACK debug build (2026-06-24, updated 2026-07-05).** The numbers below
> describe the normal (release) build. The optional `DSPI_LOOPBACK` build adds
> ~8.7 KB BSS on both platforms (a 1024-frame capture ring + a 384 B packet buffer
> + small state). Under the XIP build both platforms have ample RAM headroom for
> this (RP2040 free RAM ~91 KB, RP2350 ~192 KB before the loopback BSS); the
> RP2040 RAM-tightness noted under the old `copy_to_ram` image no longer applies.
> See "USB Audio Loopback Capture (DSPI_LOOPBACK)".

> **Output slot rings (2026-07-23; supersedes the static consumer pools).** The
> per-slot consumer buffer pool and its per-instance silence buffer are **deleted**.
> Each slot now owns a single static 16 KB BSS ring, `slot_ring_store[NUM_SPDIF_INSTANCES]`
> `[PICO_AUDIO_SPDIF_RING_BYTES]` in usb_audio.c (1024 frames × 2ch × 8 B), **shared
> by the slot's S/PDIF and I2S instances and reused across output-type switches**
> (`_Static_assert`ed to identical I2S geometry). The rings total 4 × 16 KB on
> RP2350 / 2 × 16 KB on RP2040, plus up to 16 KB of alignment padding (the RP2350
> ENDLESS read-address wrap requires the ring aligned to its own size). Drained by
> free-running DMA (see "Buffer Configuration"), the footprint stays deterministic
> and link-time-budgeted (a BSS overflow fails the build). The producer pools that
> still fed the writer at this point were removed on 2026-10-12 (see the note
> at the top of this section). Each
> statically allocated S/PDIF and I2S instance now also carries a 4-byte
> `recovery_prewrite_fill_frames` snapshot and a 4-byte `last_read_time_us`
> whole-lap anchor: +32 B BSS on RP2040 (two instances of each type) and +64 B
> on RP2350 (four instances of each type).

> **XIP execution model (2026-07-05).** Both platforms build binary type
> `default` (XIP): cold code executes from flash through the XIP cache and only
> the hot audio/USB set is copied to RAM. This replaces the previous
> `copy_to_ram` image (and its RP2040-only custom copy-to-RAM script with the
> `.xip_ctrl_text` section, both now deleted). The seven control-path objects the
> old script kept in flash (`vendor_commands`, `uart_control`, `i2c_control`,
> `control_surfaces`, `bulk_params`, `lg_sound_sync`, `usb_descriptors`) now
> simply run from flash like all other cold code, on both platforms.
>
> **What lands in RAM.** Residency is driven by source attributes: functions
> marked `.time_critical` (`DSP_TIME_CRITICAL` / `__time_critical_func`) and
> `__not_in_flash_func` land in RAM `.data` under the SDK default script rules.
> The hot set is the DSP pipeline, the USB ISR path (`PICO_RP2040_USB_FAST_IRQ=1`
> routes the TinyUSB rp2040-port ISR to RAM; both platforms use this port), the
> TinyUSB event queue (`tu_fifo`) and endpoint claim helpers (`tusb.c`), the DMA
> IRQ handlers, the feedback servo, the alarm pool + hardware timer + clocks
> (cancelled/re-armed per block by the SPDIF RX ISR; `clock_get_hz` supplies the
> nominal sys_clk on the clock-servo path), the notify ring (drained from the USB
> ISR while parameters change), the float/double/mem shims (`PICO_FLOAT_IN_RAM`,
> `PICO_DOUBLE_IN_RAM`, `PICO_MEM_IN_RAM` both platforms; `PICO_DIVIDER_IN_RAM`,
> `PICO_INT64_OPS_IN_RAM`, `PICO_BITS_IN_RAM` RP2040 only), and the entire
> Core-1 execution set.
> Newly attribute-marked hot functions include `fb_ctrl_sof_update` /
> `fb_ctrl_get_10_14` (per-SOF ISR), `spdif_input_update_clock_servo` and the
> SPDIF RX stable/lost callbacks, the shared clock-servo core (`input_servo_apply`)
> and its actuator (`soft_vcxo_set_ppm`, RAM-resident so the fbdiv-dither sources
> and the command path survive flash blackout windows), the full PDM Core-1 set (`pdm_core1_entry`,
> `pdm_processing_loop`, `pdm_push_sample`, `pdm_get_dma_fill_pct`,
> `pdm_get_ring_fill_pct`), `usb_audio_drain_ring` / `usb_audio_flush_ring`,
> `update_buffer_watermarks` / `get_slot_consumer_fill`, and the pico-extras
> spdif_rx DMA-ISR callees (`_check_block`, `_dma_done_and_restart`,
> `spdif_rx_callback_func`). The pico-extras `pico_audio` `AUDIO_TIME_CRITICAL`,
> `pico_audio_spdif_multi` `SPDIF_TIME_CRITICAL`, and `pico_audio_i2s_multi`
> `I2S_TIME_CRITICAL` attributes now apply on both platforms (all were
> RP2350-only, leaving the per-block producer/consumer and sample-encode
> functions in flash on RP2040). (`audio_i2s_mck_set_divider` was formerly on
> the clock-servo path and `I2S_TIME_CRITICAL`; MCK is no longer servoed, so it
> is retained in the library but no longer called.) The RP2040
> `dspi_flash_range_erase` / `dspi_flash_range_program` wrappers in
> `flash_clkdiv.c` are now `__no_inline_not_in_flash_func`, matching RP2350.
> `main.c` (including the main-loop glue) stays in flash deliberately: every
> block-rate entry point it calls is RAM-marked, and the USB ring plus consumer
> pools absorb microsecond-scale XIP misses.
>
> **Linker scripts.** `firmware/DSPi/memmap_dspi_rp2040_xip.ld` and
> `memmap_dspi_rp2350_xip.ld` each derive from the SDK 2.2.0 `memmap_default.ld`
> with three local edits: (a) the flash `.text` `EXCLUDE_FILE` list is extended
> with path-anchored hot objects (TinyUSB `usbd.c.o`, `usbd_control.c.o`,
> `dcd_rp2040.c.o`, `rp2040_usb.c.o`, `tusb_fifo.c.o`, `tusb.c.o`; SDK
> `float_math.c.o`, `pico_time/time.c.o`, `pheap.c.o`, `hardware_timer/timer.c.o`,
> `hardware_clocks/clocks.c.o`; project `notify.c.o`; fork `spdif_rx.c.o`) plus
> newlib `mem*` patterns covering the Arm GNU naming (`*libg.a:*libc_a-mem*.o`
> and friends; the stock SDK pattern misses them, so `memset`/`memmove` would
> silently land in flash); (b) the same exclusions on flash `.rodata`; (c) a RAM
> code pull list in `.data`, bracketed by `__ram_code_pull_start__` /
> `__ram_code_pull_end__` with an `ASSERT(> 3K)` rename tripwire that fails the
> link if the pull list stops resolving.
>
> **Verification.** `scripts/check_ram_placement.py` runs against the built ELFs
> (`python3 scripts/check_ram_placement.py build-rp2040/DSPi/DSPi.elf
> build-rp2350/DSPi/DSPi.elf`; the loopback ELFs are also accepted). Check A
> asserts a curated hot-symbol list resolves to RAM; Check B walks the transitive
> branch closure of the hot RAM functions, fails on any direct call into flash,
and warns on flash reached through linker long-call veneers (cold paths); Check
> B2 scans Core-1 function bodies for flash-range literal-pool pointers
> (whitelisting two provably init-time-only references); Check C compares sizes
> against `scripts/ram_baseline.json` (the recorded copy_to_ram baseline) and
> enforces the `.data` budgets (60 KB RP2040, 72 KB RP2350; RP2350 raised from
> 64K for the ADAT input receiver's RAM-pinned decode code). All four build
> variants currently pass with 0 FAIL.

> **Control Surfaces footprint (2026-07-06, format v2).** The engine adds
> ~1.4 KB of BSS on both platforms (was ~400 B): the 388-byte v2 live config
> (16 slots of 24-byte bindings, up from 132 B), 16 slots of runtime state,
> per-GPIO button gesture groups, and small cursors. Its code+rodata (now split
> across `control_surfaces.c` and `control_surfaces_nouns.c`) is cold and runs
> from flash XIP (above) so it does not consume RAM `.data`. In flash, the
> preset directory grows by the 388-byte `CsFlashConfig` (V9) plus the
> 512-byte per-slot name block `cs_names[16][32]` (V10, 2026-07-07);
> `sizeof(PresetDirectory)` is now 1301 bytes (789 at V9, ~533 before), still
> well within its 4 KB sector, so the flash layout is unchanged. The name
> block also grows the BSS `dir_cache` mirror by 512 bytes on both platforms.
>
> **Groups and macros (2026-08-09, caps v9).** Roughly 4.3 KB more BSS on
> RP2350 and 3.6 KB on RP2040: the live `CsGroupConfig` + `CsMacroConfig`
> (1,384 B), the same again in the `dir_cache` mirror, and 17 `CsGroupOp`
> contexts (one per binding slot plus the macro sequencer) whose
> `base[NUM_CHANNELS]` capture array makes each ~92 B on RP2350 and ~52 B on
> RP2040. The preset directory grows by the same 1,384 bytes to 2955 of its
> 4 KB sector at V18, so the flash layout is again unchanged.
>
> **I2C display (2026-08-12, caps v10).** Roughly 750 B more BSS on both
> platforms: ~410 B inside `control_surfaces_display.c` (the 80-byte live
> `CsDisplayFlash`, the 128-byte TX word ring, four 22-byte text line buffers,
> a 64-byte poll cache and driver state), the same 80 bytes again in the
> `dir_cache` mirror, plus the engine-side `PAGE_VALUE` resolved-item keys (16
> binding slots + 16 IR commands) and the two-entry grouped-IR `CsGroupOp`
> pool. Flash grows about 12 KB for the per-model drivers and init scripts,
> the ~480 B 5x8 font table and the per-noun label table; all of it is cold
> and runs from XIP. The preset directory grows by the 80-byte blob to 3035 of
> its 4 KB sector at V19, so the flash layout is again unchanged.
>
> **Control Surfaces auxiliary outputs (2026-09-07, caps v18).** Roughly 130 B
> more BSS on both platforms. That is a 4-byte value-carry entry per binding
> slot (the stash that keeps a relay from clicking across a revert or a
> same-type re-apply) plus 3 bytes per slot of runtime record for the live
> on/off flag and 8.8 level. All of it is control-path, none of it hot. The
> caps v17 model's roughly 650 B is gone, and the preset directory drops its
> 292-byte block to return to 3035 of its 4 KB sector at V21, so the flash
> layout is again unchanged. Current sizes after the change are 263,356 text
> and 150,016 bss on RP2040, 310,572 text and 355,188 bss on RP2350.

> **Buffer cleanup (2026-09-22).** Three duplicate buffers are gone. The
> notification refresh collects straight into `param_shadow` (its 6,136 B
> scratch copy is removed). `preset_save()` builds its slot in place in the
> flash staging buffer instead of in a separate `PresetSlot` (2,685 B RP2040,
> 6,453 B RP2350). `bulk_param_buf` is sized to `WireBulkParams` instead of
> 8 KB, because every writer is bounded by that size. BSS falls by 10,880 B on
> RP2040 and 14,648 B on RP2350. Flash layout and wire format are unchanged.

### RP2040 (264 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (5 × 2048 × 4) | 40 KB |
| Output buffers (5 × 192 × 4 + 2 × 192 × 4) | ~5.25 KB |
| Output slot staging (`slot_stage`, 2 × 192 × 2 × 4) | 3 KB |
| Filters + recipes (7 channels) | ~8 KB |
| Crossover filters + recipes (5 outputs × 4 bands cascades + 7 × 4 recipes) | ~3.1 KB |
| Loudness tables (2 × 61 × 2 × ~13B) | ~3 KB |
| Loudness per-output state (5 × 16 B) | 80 B |
| Preset system (dir_cache 3,147 B + `flash_stage` 4 KB, 256-B aligned) | ~7 KB |
| Bulk param buffer (`sizeof(WireBulkParams)`, 6,136 B at V32) | ~6 KB |
| Notification shadow (`param_shadow`, one `WireBulkParams`) | ~6 KB |
| USB audio ring buffer (4 × 578) | ~2.3 KB |
| Channel names (7 × 32) | ~224 B |
| Leveller state + lookahead (2 rings × 240 × 4, plus limiter caps) | ~2.0 KB (2,004 B) |
| Per-channel preamp + master volume | ~48 B |
| Output slot rings (`.slot_rings`, 2 × 8 KB at the top of RAM, shared SPDIF/I2S, no padding; 2026-10-23) | 16 KB |
| Input capture arena (SPDIF RX FIFO / I2S RX ring union, 4 KB aligned) | 12 KB |
| Other BSS | ~20 KB |
| **Total BSS** | **~110 KB** `.bss` (measured 112,936 B on 2026-10-23) plus the 16 KB `.slot_rings` section |
| RAM code+rodata+data (.data section, hot set only) | 67,984 B as of 2026-10-12, within the 72,704 B `check_ram_placement.py` budget |
| Flash-resident code (.text + .rodata + boot2, XIP) | ~98 KB |
| Free heap (`__HeapLimit - end`, nothing allocates at runtime) | 42,772 B (2026-10-12) |
| Stack | drawn from the free-RAM pool above |

### RP2350 (520 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (9 × 2048 × 4) | 72 KB |
| Filters + recipes | ~18 KB |
| Crossover filters + recipes (9 outputs × 4 bands cascades + 17 × 4 recipes) | ~11.6 KB |
| Output buffers (9 × 192 × 4) | ~7 KB |
| Output slot staging (`slot_stage`, 4 × 192 × 2 × 4) | 6 KB |
| Input buffers (buf_l + buf_r + buf_in_ext[6][192], 8-channel USB) | ~6 KB |
| Preset system (dir_cache 3,147 B + `flash_stage` 8 KB, 256-B aligned) | ~11 KB |
| Bulk param buffer (`sizeof(WireBulkParams)`, 6,136 B at V32) | ~6 KB |
| Notification shadow (`param_shadow`, one `WireBulkParams`) | ~6 KB |
| USB audio ring buffer (4 × 794, 8-channel packets) | ~3.2 KB |
| Channel names (11 × 32) | ~352 B |
| Leveller state + lookahead (8 rings × 240 × 4, plus limiter caps) | ~7.6 KB (7,788 B) |
| Per-channel preamp (8 ch) + master volume | ~120 B |
| Matrix mixer (8 × 9 crosspoints + outputs) | ~1.3 KB |
| Loudness tables + per-output state (2 × 61 × 2 coeffs + 9 × 16 B) | ~3 KB |
| Output slot rings (`.slot_rings`, 4 × 8 KB at the top of RAM, shared SPDIF/I2S, no padding; 2026-10-23) | 32 KB |
| Input capture arena (SPDIF RX FIFO / I2S RX rings / ADAT RX ring union, 8 KB aligned) | 32 KB |
| ADAT TX ring (`adat_ring`, 576 × 32 B; 2026-10-23) | 18 KB |
| Stereo upmixer state (Haas 2 × 1024 + allpass 2 × 512 floats + estimators + double-buffered coeffs) | ~12.3 KB |
| Other BSS | ~35 KB |
| **Total BSS** | **~265 KB** `.bss` (measured 271,076 B on 2026-10-23) plus the 32 KB `.slot_rings` section |
| RAM code+rodata+data (.data section, hot set only) | 95,800 B as of 2026-10-12, within the 100,352 B `check_ram_placement.py` budget |
| Flash-resident code (.text + .rodata + boot2, XIP) | ~98 KB |
| Free heap (`__HeapLimit - end`, nothing allocates at runtime) | 77,144 B (2026-10-12) |
| Stack | drawn from the free-RAM pool above |

### Flash Layout

| Region | RP2040 (2 MB) | RP2350 (4 MB) |
|--------|---------------|---------------|
| Firmware code (.text + .rodata + boot2; flash-resident under XIP) | ~98 KB | ~98 KB |
| Preset storage (12 sectors) | 48 KB | 48 KB |
| Free flash | ~1.9 MB | ~3.9 MB |

---

## Performance Characteristics
*Last updated: 2026-03-18*

### Buffer Sizes

| Buffer | Size |
|--------|------|
| USB packet | 44-49 samples (~1 ms at 48 kHz) |
| S/PDIF IEC block | 192 samples (IEC 60958-1 standard) |
| S/PDIF DMA transfer | Free-running (whole ring; no fixed transfer quantum) |
| S/PDIF slot ring | 512 frames per slot (10.7 ms at 48 kHz); target fill `output_ring_target_frames`, set by the output latency (192 at the 4 ms default; 25% of the 768-frame stats scale) |
| Output slot staging | 192 frames per slot (one block), copied into the slot ring on admission |
| PDM DMA ring | 2048 words |
| PDM sample ring | 256 entries (Core 0 → Core 1) |

### Latency (at 48 kHz)

| Path | Latency |
|------|---------|
| USB → S/PDIF | The output latency setting (default 4 ms) plus about half a 1 ms block; see "Output Latency" |
| S/PDIF latency jitter | ±1 ms (±1 buffer of 48 samples) |
| S/PDIF → PDM alignment | Ring target minus 256 samples (+2.67 ms at 8 ms; clamps the sub's delay below 256 frames); see PDM "Known alignment defect" |
| Total end-to-end | ~10-15 ms |

### CPU Utilization

| Metric | RP2040 | RP2350 |
|--------|--------|--------|
| Core 0 (single-core, all outputs) | ~40% (5 outputs, block-based) | ~30-40% |
| Core 0 (EQ worker mode) | ~15% (input pipeline only) | ~30% |
| Core 1 (PDM) | ~15% | ~15% |
| Core 1 (EQ worker) | ~25% (4 SPDIF outputs, block-based) | ~20% |

### Supported Sample Rates

44.1 kHz, 48 kHz, 96 kHz — automatic PLL switching on rate change.

---

## RP2040 Sample Scale
*Last updated: 2026-10-23*

RP2040 pipeline samples are int32 with 0 dBFS at 2^29 (`SAMPLE_FULL_SCALE_Q28` in `config.h`, Q28 value 2.0). Filter coefficients and gains are Q28 with 1.0 = 2^28. A Q28 multiply leaves a sample's scale unchanged, so the "Q28" name describes the arithmetic, not where full scale sits. Input conversion shifts 16-bit samples left by 14 and 24-bit samples left by 6, and the output conversion shifts right by 6, so a flat pass-through is bit-exact. The RP2350 float path puts 0 dBFS at 1.0.

Any RP2040 code that compares a sample-derived level with an absolute dBFS value, or generates samples at one, must use this scale. Kernels written with 0 dBFS = 1.0 Q28 convert at their boundary.

| Code | Handling |
|------|----------|
| Peak meters, clip flags | `peak_to_meter_q28()` (`>> 14`, clamped to 32767); `CLIP_THRESH_Q28` = 2^29 + 0.1% |
| Spectrum analyser | FFT tap `>> 14` to Q15; bass bank input `>> (2 + cic_shift)` |
| Signal generator | Amplitude is `lin * SAMPLE_FULL_SCALE_Q28` |
| PDM sub | Both platforms hand the modulator 2^29 full scale; 0 dBFS maps onto `PDM_FULL_SCALE` (see "PDM Subsystem", Input Limiting) |
| Volume leveller | Limiter peaks (`lim_abs`) and detector squares (`det_sq`) are halved into 1.0-based units; the detector clamps at +6 dBFS |
| Subharmonic synth | The sub path runs 1.0-based: `sh_from_pipe()` halves its input and `sh_to_pipe()` doubles the sub; dry + sub sums saturate (`sh_add_sat()`) |
| Tube preamp | Kernel input halved, output doubled with saturation |
| Psychoacoustic bass | Odd-path clipper input halved, output doubled |
| Output limiter | Uses 2^29 directly (`LM_FS_RAW`) |
| 24-bit output conversion | `q28_to_s24()` rounds like `(x + 32) >> 6` but cannot wrap: a sample saturated at `INT32_MAX` (+12 dBFS) clips to +full scale |

Until 2026-10-23 every row except the output limiter assumed 2^28, so each was 6 dB off on RP2040. The meters read hot, clip flags tripped at -6 dBFS and the signal generator played 6 dB low. The leveller, subharm ceiling, tube drive and psybass clip point all acted 6 dB away from their set levels. The RP2350 was the reference and is unchanged, except for the PDM sub.

## Channel Metering
*Last updated: 2026-10-23*

Full peak metering for all input and output channels. Peak values are `uint16_t` in Q15 format (0–32767 maps to 0.0–1.0 full scale). Per-channel clip detection via sticky `clip_flags` bitmask.

**RP2040 sample scale.** RP2040 samples reach 0 dBFS at 2^29, so every RP2040 peak site goes through `peak_to_meter_q28()` and both platforms report the same meter scale. See "RP2040 Sample Scale".

### Peak Array Layout (`global_status.peaks[NUM_CHANNELS]`)

| Index | RP2350 (11 channels) | RP2040 (7 channels) |
|-------|----------------------|---------------------|
| 0 | Input L | Input L |
| 1 | Input R | Input R |
| 2 | SPDIF 1 L | SPDIF 1 L |
| 3 | SPDIF 1 R | SPDIF 1 R |
| 4 | SPDIF 2 L | SPDIF 2 L |
| 5 | SPDIF 2 R | SPDIF 2 R |
| 6 | SPDIF 3 L | PDM Sub |
| 7 | SPDIF 3 R | — |
| 8 | SPDIF 4 L | — |
| 9 | SPDIF 4 R | — |
| 10 | PDM Sub | — |

### Dual-Core Peak Tracking

In EQ_WORKER mode, each core meters only the outputs it processes:

- **Core 0:** Input L/R peaks (always), SPDIF outputs 0 to `CORE1_EQ_FIRST_OUTPUT-1`, PDM peak zeroed (PDM inactive in this mode)
- **Core 1:** SPDIF outputs `CORE1_EQ_FIRST_OUTPUT` to `CORE1_EQ_LAST_OUTPUT`, written before `work_done` handshake

In single-core mode, Core 0 meters all outputs including PDM.

**Thread safety:** No race — Core 1 writes its channel peaks before `work_done`; Core 0 writes its channel peaks after `work_done`. The `__dmb()` + handshake guarantees memory visibility. Each core only OR's its own non-overlapping channel bits in `clip_flags`, so no torn-write risk.

### Clip Detection (OVER Indicator)
*Last updated: 2026-10-23*

`global_status.clip_flags` is a `uint16_t` bitmask — one bit per channel (bit position = channel index). A bit is **set** when the block peak exceeds the clip threshold (`CLIP_THRESH_F` = 1.001f on RP2350, `CLIP_THRESH_Q28` = (1<<29) + (1<<29)/1000 on RP2040, the same 0.1% margin over the RP2040's 2^29 full scale). The threshold includes ~+0.01 dB headroom above unity to avoid false positives from float precision noise when 0 dBFS signals pass through biquad filters. Bits are **sticky**: once set, they remain set until explicitly cleared by the host via `REQ_CLEAR_CLIPS` (0x83). The firmware never autonomously clears clip flags.

This matches the industry-standard sticky OVER indicator pattern (IEC 60268-18). Since DSPi is a DSP processor (not an ADC), any sample exceeding the threshold in `buf_out` is a genuine clip event — single-sample detection is correct.

**Detection cost:** One compare + conditional OR per channel per block on the already-computed peak value. Zero measurable overhead.

### Status Protocol (`REQ_GET_STATUS`, wValue=9)

Variable-size response: `NUM_CHANNELS * 2 + 4` bytes.

- RP2350: 26 bytes (11 peaks × 2 bytes + 2 CPU load bytes + 2 clip_flags bytes)
- RP2040: 18 bytes (7 peaks × 2 bytes + 2 CPU load bytes + 2 clip_flags bytes)

Format: peaks as little-endian `uint16_t` in channel index order, followed by `cpu0_load` and `cpu1_load` (each `uint8_t`, 0–100%), followed by `clip_flags` as little-endian `uint16_t`.

### REQ_CLEAR_CLIPS (0x83) — Clear Clip Flags
*Last updated: 2026-03-01*

Atomic read-then-clear: returns the current `clip_flags` value (2 bytes, little-endian `uint16_t`) and resets it to 0. This gives the host an acknowledgment of which channels had clipped since the last clear.

| Field | Value |
|-------|-------|
| `bmRequestType` | `0xC1` |
| `bRequest` | `0x83` |
| `wValue` | 0 |
| `wIndex` | 0 |
| `wLength` | 2 |

**Response (2 bytes):** The `clip_flags` value that was just cleared (little-endian `uint16_t`).

---

## External Control Interfaces (UART / I2C Target)
*Last updated: 2026-08-12 (the Control Surfaces display claims the other I2C instance)*

An external microcontroller can drive the entire vendor-command surface over a
UART or the I2C target (slave) interface, at parity with USB. Full integrator
detail lives in `Documentation/Features/control_interfaces_spec.md`; this section
covers the firmware-side structure.

### Orchestrator refactor

The vendor command surface was made transport-neutral. USB (EP0), UART, and I2C
each parse their own wire framing into the same `bRequest` / `wValue` / `wIndex` /
`wLength` shape and call a shared dispatcher in `vendor_commands.c`:

- `vendor_dispatch_get(CtrlSource, bRequest, wValue, wIndex, wLength, &resp_data, &resp_len)`
- `vendor_dispatch_set(CtrlSource, bRequest, wValue, wIndex, payload, wLength)`

`CtrlSource` is `USB` (0) / `UART` (1) / `I2C` (2). New commands added to the
SET/GET switches are automatically reachable from every transport with nothing
per-transport to implement.

**Response sink.** A GET dispatch returns a pointer/length into static storage
that stays valid until the next dispatch from any transport, so a transport poll
consumes or copies it before returning. `REQ_GET_ALL_PARAMS` points into
`bulk_param_buf` and holds the bulk lock until the caller releases it. Every
other GET is copied into the static `_ext_resp_copy` sink while the handler
frame is still alive, because an external transport consumes the bytes only
after the handler returns. That sink is sized by the largest non-bulk GET, so
it widened from 64 to 132 bytes for `REQ_GET_CS_MACRO`'s `CsMacro`; a longer
response than that would have to go through the bulk buffer path.

**USB SET-in-flight guard.** External dispatch runs from the main loop and is
refused with `CTRL_DISPATCH_BUSY` (wire `CTRL_STATUS_BUSY`) while a USB control
SET is mid-transfer, so the two paths never race the same state. The client
retries the whole request.

**Chunked USB bulk access (0xA2/0xA3, added 2026-07-04).** Windows/WinUSB caps
control-transfer data stages at 4 KB, so the 5864-byte V16 `WireBulkParams`
cannot cross a single `0xA0`/`0xA1` transfer on any Windows host (GitHub
issue #62). `REQ_GET_ALL_PARAMS_CHUNK` (0xA2) and `REQ_SET_ALL_PARAMS_CHUNK`
(0xA3) move the same payload in `wValue`-offset chunks riding the bulk owner
lock as a multi-transfer session: GET offset 0 snapshots the struct under the
lock (all chunks read one coherent image, released at the final chunk's ACK);
SET chunks land sequentially in `bulk_param_buf` and the final byte hands the
buffer to the normal `bulk_params_pending` apply. Sessions survive chunk
SETUPs but are torn down by any other vendor request or the 3 s stale reap;
both commands are USB-only (refused on UART/I2C, which have no size cap).
Host migration guide: `Documentation/Features/bulk_params_chunking.md`.

**bulk_param_buf owner lock.** The single shared bulk buffer is serialized across
USB/UART/I2C via `vendor_bulk_try_acquire` / `vendor_bulk_release` /
`vendor_bulk_touch` (IRQ-safe). External owners go stale after 500 ms unless they
refresh while actively streaming. A contended request returns
`CTRL_DISPATCH_BULK_LOCKED` (wire `CTRL_STATUS_BULK_LOCKED`).

**USB-only self-config.** `REQ_SET_UART_CONFIG` (`0xF5`) and `REQ_SET_I2C_CONFIG`
(`0xF7`) are refused on the external transports (`CTRL_DISPATCH_BLOCKED` / wire
`CTRL_STATUS_BLOCKED`) so an external controller can never reconfigure or lock
itself out of its own transport. All other commands, including the config GETs,
the status readback, presets, bulk, and bootloader entry, work on every
transport.

### Transport modules

- `uart_control.c` / `.h`: sync-byte + CRC16-CCITT-FALSE framing (poly `0x1021`,
  init `0xFFFF`), fixed 8N1, configurable baud 9600..1000000. Types `0x01`/`0x02`
  requests, `0x81`/`0x82` responses, and `0x40` **device-initiated notification**
  frames (`A5 40 00 lenL lenH packet crc16`). The notification `packet` is the
  verbatim v2 notify packet the USB EP 0x83 delivers; the UART transport is a
  second consumer of the notification ring (`NOTIFY_CONSUMER_UART`). Frames are
  opt-in via `UartCtrlConfig.notify_enable` (formerly a reserved byte; 0/1,
  default 0; validated in `uart_ctrl_validate`, sanitized in `flash_storage.c`;
  no directory version bump or wire change since all prior configs carry 0
  there). They are pushed only when the link is otherwise idle so a response is
  never split; a saturating requester can starve them, and the seq-gap contract
  (client re-reads `REQ_GET_ALL_PARAMS`) covers recovery. The v1 legacy
  master-volume packet is never sent over UART. Enabling/disabling tracks the
  live interface: `up()` activates the UART consumer when `notify_enable` is set,
  `down()` deactivates it.
- `i2c_control.c` / `.h`: target-only, 8-byte header write frames,
  `[status,lenL,lenH,payload]` read frames with `0xFF` padding, a `[0x01,0,0]`
  BUSY frame before the main loop dispatches, and resumable chunked reads. Up to
  400 kHz; weak internal pull-ups enabled but external pull-ups recommended.
  The two hardware I2C instances are shared with the caps v10 Control Surfaces
  display, which is an I2C **master**: a display binding must land on the
  instance this target interface does not occupy, checked at apply time via
  `i2c_ctrl_live_instance()` and rejected with `CS_STATUS_I2C_IN_USE` (0x24).
  The reverse direction is only partly covered: a display's pins are reserved
  against this interface through `control_surfaces_owns_pin()` inside
  `pin_used_by_fixed_peripheral()`, but a *different* pin pair on the instance
  a display already holds is not caught, so give the two functions separate
  instances by design.

### Non-blocking design and IRQ priorities

Both transports are strictly non-blocking. Their ISRs only shuttle bytes between
the peripheral FIFOs and module ring/buffers; all frame parsing, dispatch, and TX
happen in `uart_ctrl_poll()` / `i2c_ctrl_poll()` from the main loop, so the audio
pipeline is never touched. The I2C target stretches the clock only to ISR latency
at FIFO boundaries, never while waiting on application state (that is what the
BUSY frame is for). Both control IRQs run at priority `0xC0` (below the audio and
USB IRQs).

### Boot and main-loop wiring

- **Boot** (`main.c`): the control-interface init block is deliberately last in
  `core0_init()`: after `preset_boot_load()`, after the output / DAC-mute pin
  claims, and after `notify_init()` (so the UART bring-up's notify-consumer
  activation is not wiped by the consumer-table reset inside `notify_init()`).
  `preset_get_ctrl_iface()` supplies the persisted configs to `uart_ctrl_init()`
  / `i2c_ctrl_init()`. A stored config whose pins now collide is quietly kept
  down (live=false), visible via `REQ_GET_CTRL_IFACE_STATUS`.
- **Main loop:** `uart_ctrl_poll()` and `i2c_ctrl_poll()` run every iteration.
- **Deferred config SET:** a USB `0xF5`/`0xF7` sets `ctrl_set_uart_pending` /
  `ctrl_set_i2c_pending`; the main loop does the live apply first (GPIO/IRQ work,
  tearing the interface down before validating so its own pins are not seen as
  in-use), then persists to the directory **only on** `PIN_CONFIG_SUCCESS`, so a
  bad SET never clobbers a good stored config. Changes arriving over UART/I2C emit
  USB-host notifications tagged `PARAM_SRC_UART` (8) / `PARAM_SRC_I2C` (9).

Interface config is device-level, not part of `WireBulkParams`; the bulk wire
format version is unchanged by this feature.

---

## Control Surfaces (User-Wired Physical Controls)
*Last updated: 2026-09-28 (caps v20: output limiter nouns 74-78 and unit `CS_UNIT_MS_LOG`; 2026-09-18 caps v19: tube preamp nouns 70-73, all four through the indexed command 0x3E; 2026-09-07 caps v18: auxiliary outputs are binding-slot components CS_TYPE_AUX_OUT 9 / CS_TYPE_AUX_PWM 10, nouns 68-69 target the slot, commands 0x04-0x07, directory V21; caps v17 never shipped; caps v16: subharmonic band-level nouns widened to +12 dB; 2026-09-04 caps v15: subharmonic synthesizer nouns 61-67; 2026-09-02 caps v14: subharmonic synthesizer nouns 57-60; 2026-08-24: input-source stepping skips unselectable sources; caps v13: display level bars; caps v12: per-LED PWM brightness ceiling; caps v11: display line alignment and edit markers; caps v10: I2C display component, IR group support, nouns 53-56, commands 0x27-0x2B, directory V19; 2026-07-22: PWM LED and aux PWM binds reject the soft-VCXO reserved slice)*

User-wired push buttons, toggle switches, potentiometers, quadrature rotary
encoders, plain indicator LEDs, PWM-dimmed LEDs, an IR remote receiver, an
I2C character/OLED display, and switched or dimmable auxiliary outputs on
spare GPIOs, configured over vendor commands `0x84`-`0x87`, `0x8B`/`0x8C`
(per-slot names), `0x8D`-`0x8F` (IR commands and learn), `0x9D`/`0x9E`
(save/revert), `0x20`-`0x26` (target groups and macros), `0x27`-`0x2B`
(display config, pages and status), and `0x04`-`0x07` (auxiliary output
runtime state and level).
A binding attaches
one component (`CsType`)
to one firmware parameter (`CsNoun`) through one operation (`CsAction`), on one
or two GPIOs. The full integrator spec is
`Documentation/Features/control_surfaces_spec.md`, with groups and macros
specified in `Documentation/Features/control_surfaces_groups_macros_spec.md`
the display in `Documentation/Features/control_surfaces_display_spec.md`, and
the auxiliary outputs in
`Documentation/Features/control_surfaces_aux_spec.md`.

**Format v2** (`CS_CONFIG_VERSION` = 2, `caps_version` = 2) supersedes the
original v1 model. Bindings grew from 16 to 24 bytes and gained explicit
`event` / `target` / `index` fields; there are now 16 slots (was 8). Buttons
carry a `CsEvent` (press / long-press / double-press), so several button
bindings may share one GPIO as long as their events differ; they also support
hold-to-repeat (`CS_FLAG_REPEAT`) and momentary hold-to-engage (`CS_ACT_MOMENTARY`,
restore on release). Encoders support acceleration (`CS_FLAG_ACCEL`). The
`CS_TYPE_LED_PWM` type drives a hardware-PWM LED whose brightness follows a noun
(`CS_ACT_IND_LEVEL`), and `CS_ACT_IND_ABOVE` lights an LED while a noun is at or
above a threshold. Nouns carry a `unit` (`CS_UNIT_DB/HZ/Q/PERCENT/MS`) so
frequency and Q step logarithmically while dB/percent/ms step linearly, and a
`target_kind` so a binding can address a specific input/output channel or a
specific channel+filter-band.

The v2 noun catalog expanded from 9 to 35 entries (35-48 arrived later with
caps v4, below): per-input preamp; per-output
gain/mute/enable; per-filter freq/gain/Q/type/bypass with channel + band
targeting; siggen run; a DAC hardware-mute test trigger; and read-only indicator
nouns (per-channel clip latch and peak level, SPDIF lock, sample rate, USB
streaming, ADAT active, LG Sound Sync present/muted). `CS_NOUN_ADAT_ACTIVE` is
RP2350-only; its descriptor carries an empty action mask on RP2040 so no host UI
offers it and no binding to it validates there.

**Caps v4** (2026-07-19) appends 14 nouns (35-48) and one unit with no
structure or stored-config changes (directory stays V11): the stereo upmixer
(enable, centre mode Passive/Logic, surround mode Off/Passive/Logic,
strength, centre width, presence; all six RP2350-only via the empty-mask
convention, dispatched through `REQ_UPMIX_SET_PARAM`), psychoacoustic bass
(enable plus cutoff/harmonics/drive/character/original through the per-param
`0x30`-`0x3A` SETs), per-output delay (`CS_NOUN_OUTPUT_DELAY`, target =
output channel, `REQ_SET_OUTPUT_DELAY`; new `CS_UNIT_MS` = 8.8 ms, linear,
default step 0.1 ms, range = the full delay ring at 48 kHz: 21 ms RP2040 /
42 ms RP2350), and `CS_NOUN_PRESET_RELOAD` (a `TRIGGER` that reloads the
active preset from flash via the deferred `REQ_PRESET_LOAD` path, discarding
unsaved live edits).

**Caps v5** (2026-08-01) adds no nouns, units, or structure changes (directory
stays V11): it signals only that `CS_NOUN_UPMIX_CENTER_MODE` widened from two
values to three, the third being `Off`. Hosts reading `enum_count` from the
noun descriptor need no change; the bump is for hosts that hard-code the mode
labels. `Off` is value 2 rather than 0 because the vendor enum could not be
renumbered without silently remapping existing hosts and saved presets.

**Caps v6** (2026-08-04) doubles `CS_MAX_IR_COMMANDS` from 8 to 16, so one
receiver can carry a whole handset. `CsIrConfig` grows to format v2 (260 B),
bumping the directory to V17; `CsStatusPacket` grows to 41 B, with
`ir_active_mask` widened to a uint16 in place (offset 22), which pushes
`ir_learn_state` to 24 and `ir_cmd_status[16]` to 25. `CsCapsHeader` keeps its
40 bytes and its `max_ir_commands` field now reads 16; hosts must size the
command list from it rather than assume 8.

**Caps v7** (2026-08-04) appends the two remaining loudness parameters as
nouns 49-50, with no unit, structure, or stored-config changes (directory
stays V17). `CS_NOUN_LOUDNESS_SPL` is the reference listening level in dB SPL
(`CS_UNIT_DB`, 40-100, `REQ_SET_LOUDNESS_REF`) and
`CS_NOUN_LOUDNESS_INTENSITY` is the compensation depth
(`CS_UNIT_PERCENT`, 0-127, `REQ_SET_LOUDNESS_INTENSITY`); both join
`CS_NOUN_LOUDNESS` (enable, noun 3) so a front panel can voice the loudness
curve, not just switch it. The wire commands accept intensity up to 200 %, but
8.8 percent tops out at 127.99, so the front-panel span stops at 127 %.
Neither noun is `CS_NDF_DEFERRED`: the SET handler stores the value
immediately and only the 61-step coefficient rebuild is deferred, coalesced by
`loudness_recompute_pending` to at most one rebuild per main-loop pass however
fast a knob is swept. The accepted spans now live in `loudness.h`
(`LOUDNESS_REF_SPL_MIN`/`MAX`, `LOUDNESS_INTENSITY_MIN`/`MAX`) so the vendor
clamps and the noun table cannot drift apart.

**Caps v8** (2026-08-09) adds indicator condition timing and one noun, with no
structure or stored-config changes (directory stays V17). `CsBinding` gains
`on_delay` and `off_delay` (uint16, 0.1 s units, max ~109 min) carved from the
former `reserved2[6]`, so the struct stays 24 bytes and pre-v8 configs (zeros
there) mean "no delay". They filter the raw `IND_EQUALS`/`IND_ABOVE` condition
like a PLC TON/TOF timer pair: the condition must hold continuously for the
delay before the LED follows it, any blip restarts the pending edge, the
filter runs before `CS_FLAG_INVERT`, and elapsed time is wall-clock
(`time_us_64`-derived ms) so a slow main loop cannot stretch minute-scale
delays. Accepted only on the LED types with those
two actions; rejected (`CS_STATUS_INVALID_VALUE`) on `IND_LEVEL`, inputs, and
the IR container. Runtime cost is three fields in `CsRuntime` and integer
compares inside the existing 8 ms-decimated indicator tick; the audio path is
untouched. The appended noun `CS_NOUN_INPUT_LEVEL_MAX` (51, read-only dB
-60..0, untargeted) reads the loudest channel of the active input (max over
`global_status.peaks[0..active_input_channel_count())`), gated on
`pipeline_producer_is_streaming()` (promoted from static in main.c): peaks[]
freezes when block delivery stops, so a dead source must read as the -60 dB
silence floor or a trigger LED would latch on. This gives whole-source
signal presence. The motivating combination is an amplifier trigger output:
a plain LED binding with `INPUT_LEVEL_MAX` + `IND_ABOVE` + an `off_delay` of
minutes drives an amp's 12 V trigger on at the first signal and off only
after sustained silence (spec section 8.2g).

**Caps v9** (2026-08-09) adds target groups and macros (subsection below),
commands `0x20`-`0x26`, and directory V18. `CsBinding`, `IrCommand` and
`CsStatusPacket` are byte-identical to caps v8, so external clients doing
exact-length readback are unaffected until they opt into the new commands;
`CsCapsHeader` keeps its 40 bytes with the three reserved bytes after
`max_ir_commands` becoming `max_groups` / `max_macros` / `max_macro_steps` (a
pre-v9 host read them as zeros). One noun is appended, `CS_NOUN_MACRO` (52),
taking `noun_count` to 53.

**Caps v10** (2026-08-12) adds the I2C display component (subsection below),
commands `0x27`-`0x2B`, IR remote group support, four nouns (53-56), and
directory V19. `CsBinding`, `IrCommand`, `CsStatusPacket`, `CsExtStatusPacket`
and `CsNounDesc` are byte-identical to caps v9, but `REQ_GET_CS_CAPS` is the
one pre-existing GET whose **length changes**: adding `CS_TYPE_DISPLAY` (8)
grows the caps type table by one 4-byte `CsTypeDesc` row, so the header
response goes from 40 to 44 bytes. That is the self-describing mechanism
documented since caps v3 (hosts read `type_count` and locate
`max_ir_commands` / `max_groups` / `max_macros` / `max_macro_steps` at offset
`4 + 4 * type_count`), but external clients that do exact-length readback must
be updated before this firmware ships to them. The appended nouns are
`CS_NOUN_CPU_LOAD` (53, read-only percent from `global_status.cpu0_load`, the
same source as `REQ_GET_STATUS` wValue 9, so an `IND_LEVEL` PWM LED becomes a
CPU meter), `CS_NOUN_DISPLAY_PAGE` (54), `CS_NOUN_DISPLAY_EDIT` (55) and
`CS_NOUN_PAGE_VALUE` (56), taking `noun_count` to 57.

**Caps v11** (2026-08-16) adds per-line horizontal alignment for the display
and the bracketed edit markers (subsection below). Every structure, GET
length and command payload is byte-identical to caps v10; the bump exists
only so hosts know the new `CsDisplayCfg.flags` bits are honoured rather than
rejected as unknown. Older stored blobs read back with the bits clear, which
is left alignment, the pre-v11 behaviour.

**Caps v12** (2026-08-16) adds `CsBinding.base_bright`, the per-LED_PWM
brightness ceiling described above. The byte comes from the struct's spare
`reserved` field, so `CsBinding` stays 24 bytes, `CsFlashConfig` stays 388,
every GET keeps its length, and no directory version moves. Pre-v12 configs
carry 0, meaning full brightness.

**Caps v13** (2026-08-17) adds display level bars: `CsDisplayPage.flags` gains
`CS_DPAGE_BAR` (bit 3), rejected on nouns with no continuous span. The page
record stays 4 bytes and `CsDisplayFlash` 80, so again nothing migrates.

**Caps v14** (2026-09-02) appends four nouns (57-60) for the subharmonic
synthesizer with no structure or stored-config changes: `SUBHARM` (enable,
`REQ_SET_SUBHARM`), `SUBHARM_LOW` and `SUBHARM_HIGH` (band levels, -30..+6 dB
via the per-param `0x12`/`0x14` SETs) and `SUBHARM_BOOST` (0..+6 dB via
`0x16`), taking `noun_count` to 61. Display labels are "Subharm", "Sub 24-36",
"Sub 36-56" and "LF Boost".

**Caps v15** (2026-09-04) appends the rest of the subharmonic synthesizer's
parameters as nouns 61-67, again with no structure or stored-config changes:
`SUBHARM_TOP` (-30..+6 dB via `0x1B`), `SUBHARM_SELECT` (enum 0-2 via `0x1D`,
labels "All" / "Percussive" / "Sustained"), `SUBHARM_DEPTH` (0..100 % via
`0xA9`), `SUBHARM_HOLD` (ms via `0xAB`), `SUBHARM_CEILING` (-40..0 dB via
`0xAD`), `SUBHARM_LINK` (bool via `0x2E`) and `SUBHARM_SOLO` (bool via `0x2C`),
taking `noun_count` to 68. The hold noun's caps range stops at 127 ms because
`CsNounDesc.min_q`/`max_q` are signed 8.8 fixed point; the command still accepts
the documented 50..400 ms, the same limitation `CS_LOUDNESS_INTENSITY_MAX`
already carries. Display labels are "Sub 56-80", "Sub Select", "Sub Depth",
"Sub Hold", "Sub Ceil", "Sub Link" and "Sub Solo".

**Caps v16** (2026-09-05) widens the three subharmonic band-level nouns
(`SUBHARM_LOW`, `SUBHARM_HIGH`, `SUBHARM_TOP`) from -30..+6 to -30..+12 dB.
Range change only; no nouns, structures or stored config change.

**Caps v17** (2026-09-05) never shipped. It modelled auxiliary outputs as a
separate table of eight pinless on/off + level values (`CsAuxCfg` /
`CsAuxConfig`, commands `0x02`-`0x07`, directory V20) that an LED binding had
to follow to reach a pin. No device or host ever ran it, and caps v18 replaces
it outright.

**Caps v18** (2026-09-07) makes an auxiliary output a component in a binding
slot that owns its GPIO (subsection below). Two type-table entries are
appended, `CS_TYPE_AUX_OUT` (9, on/off) and `CS_TYPE_AUX_PWM` (10, dimmable),
so `CS_TYPE_COUNT` is 11 and `CsCapsHeader` grows from 44 to 52 bytes. Both are
containers with an all-zero action mask, one pin and pin class ANY. A host that
already locates the v3 tail at `4 + 4*type_count` needs no change; one that
hardcoded the header length does. The other additions are nouns `CS_NOUN_AUX`
(68) and `CS_NOUN_AUX_LEVEL` (69), the target kind `CS_TARGET_AUX` (5, which
addresses the binding slot holding the output), status code
`CS_STATUS_INVALID_AUX` (0x26), commands `0x04`-`0x07`, and directory V21.
`noun_count` goes to 70. `CsBinding` byte 22 becomes `extras`, which must stay
0 on every non-aux type, so pre-v18 bindings remain valid unchanged; no
structure changes size and no other GET changes length, so external clients
doing exact-length readback are unaffected until they opt in.

**Caps v19** (2026-09-18) appends four nouns (70-73) for the tube preamp, with no
structure or stored-config changes. `TUBE` (enable), `TUBE_DRIVE` (0..24 dB),
`TUBE_TYPE` (enum, 17 values, 0 = Custom) and `TUBE_MIX` (0..100 % dry/wet) all
dispatch through the one indexed command `REQ_SET_TUBE_PARAM` (`0x3E`) with the
parameter index in wValue, so all four carry a float32 payload, the bool and the
enum included. `noun_count` goes to 74. Display labels are "Tube", "Tube Drive",
"Tube Type" and "Tube Mix", and the type noun has its own 17-entry short-form
label table. The authority for the effect is `tube_preamp_spec.md`.

**Caps v20** (2026-09-28) appends five nouns (74-78) for the output limiter and
one unit, with no structure or stored-config changes. `noun_count` goes to 79.
Every limiter noun targets an output (`CS_TARGET_OUTPUT_CH`). `LIMITER`,
`LIMITER_THRESHOLD`, `LIMITER_RELEASE` and `LIMITER_LINK` dispatch through
`REQ_LIMITER` (`0x81`) with `wValue = (output << 8) | index` and a float32
payload, and `LIMITER_GR` is a read-only gain-reduction meter (0..30 dB). The
release spans 10..1000 ms, past the 127.99 ms ceiling of 8.8 `CS_UNIT_MS`, so it
uses the new `CS_UNIT_MS_LOG` (6): plain integer ms in the value and range
fields, stepping and pot mapping in octaves exactly like `CS_UNIT_HZ`, and a
default step of 1/12 octave. `cs_unit_is_log()` is now exported so the display
level bar and the engine share one list of octave-domain units. Display labels
are "Limiter", "Lim Thr", "Lim Rel", "Lim Link" and "Lim GR", with link groups
shown as "Off" and "1".."4". A host that does not know unit 6 must learn it
before it can label or encode the release noun. Link-group ganging stays in
`limiter_set_param()`, so a bound control behaves exactly like a host SET.

### File layout

- `control_surfaces.c` / `.h`: the engine and the wire/flash data model
  (`CsBinding` 24 B, `CsFlashConfig` 388 B, `IrCommand` 16 B, `CsIrConfig`
  260 B, `CsCapsHeader` 40 B, `CsNounDesc` 12 B, `CsStatusPacket` 41 B, with a
  `uint16_t active_mask` for the 16 slots and a `uint16_t ir_active_mask` for
  the 16 IR sub-slots).
  The type-capability table and the noun-descriptor table (`cs_noun_table`, in
  `control_surfaces_nouns.c`) are the single source of truth for the validity
  model and are served verbatim by `REQ_GET_CS_CAPS`, so host UIs and firmware
  can never disagree about which type/noun/action combinations are legal.
- `control_surfaces_nouns.c`: the noun catalog. `cs_noun_table` is
  the descriptor table; `cs_noun_get` reads a noun's live value in natural units;
  `cs_noun_dispatch` applies a resolved absolute target through the shared vendor
  surface; `cs_noun_validate_target` bounds-checks a binding's target/index
  against the live channel/band layout. Filter nouns do a read-modify-write
  through `REQ_SET_EQ_PARAM` behind an `eq_update_pending` BUSY guard;
  `CS_NOUN_FILTER_BYPASS` uses `REQ_SET_BAND_BYPASS`. Like `control_surfaces.c`,
  this file executes from flash XIP on RP2040 (not in the RAM pull list).
- `control_surfaces_ir.c` / `.h` (engine-internal): IR remote capture and
  decode for the `CS_TYPE_IR` component. A RAM-resident, lowest-priority
  IO_IRQ_BANK0 edge handler (the firmware's only GPIO interrupt) timestamps
  mark/space durations into a 128-entry SPSC ring; `cs_ir_poll()` (called
  from the CS tick) assembles frames (a >10 ms space terminates one) and
  decodes NEC/NECext including the dedicated repeat frame, RC5 and RC6 mode 0
  with the toggle bit masked out of the code (its value is kept for hold
  tracking), and a stable FNV-1a timing-signature hash for everything else,
  surfacing press / repeat / release events plus the learn state machine
  (arm, 10 s window, capture-first-press).
- `control_surfaces_display.c` / `.h` (engine-internal, caps v10): the
  `CS_TYPE_DISPLAY` component. Owns the live `CsDisplayFlash` blob (config +
  16 page slots), the content logic (home modes, event overlay, edit mode,
  page resolution for `PAGE_VALUE`), the per-model I2C init scripts and the
  byte-budget transmit state machine, plus the 5x8 flash font used by the
  graphic OLEDs. Everything runs on the shared 1 kHz tick from core 0 and
  never blocks. The host-facing apply/accessor entry points
  (`control_surfaces_apply_display_cfg` / `_page`,
  `control_surfaces_display_flash`, `control_surfaces_get_display_status`)
  are declared in `control_surfaces.h`.
- `vendor_commands.c`: the `0x84`-`0x87`, `0x8B`-`0x8F`, `0x9D`/`0x9E`
  handlers; `REQ_SET_CS_BINDING`, `REQ_SET_CS_NAME`, and `REQ_SET_CS_IR_CMD`
  latch deferred SETs, `REQ_CS_SAVE`/`REQ_CS_REVERT` latch deferred flags,
  `REQ_CS_IR_LEARN` arms/cancels/reads the learner, the GETs return live
  accessor data.
  `control_surfaces_owns_pin()` is wired into `pin_used_by_fixed_peripheral()`.
- `flash_storage.c`: directory V11 persistence (`preset_get_cs_config`,
  `preset_get_cs_ir_config`, the combined single-write setter
  `preset_set_cs_all` (bindings + IR commands + names), `preset_get_cs_name`,
  `dir_sanitize_cs_config`,
  `dir_sanitize_cs_ir`, the fan-in V7->current and V8->current migrations via
  `cs_config_from_v1()`, the V9->V10 name append, and the V10->V11 IR-table
  append).

### Dispatcher reuse via `CTRL_SOURCE_GPIO`

Every control action is applied by dispatching the same vendor command a host
would send, through `vendor_dispatch_set` / `vendor_dispatch_get` with a fourth
control source, `CTRL_SOURCE_GPIO` (3). This reuses all existing validation,
deferred pipeline-safe apply (e.g. preset load stops SPDIF RX and fences Core 1),
and output-slot alignment; nothing in the apply path is duplicated. Dispatches
are tagged `PARAM_SRC_GPIO` (5), so a knob turn emits the normal `PARAM_CHANGED`
notification. Because the engine runs from main-loop context it honors the same
`CTRL_DISPATCH_BUSY` back-pressure (a USB control SET mid-flight); it latches the
resolved **absolute** target and retries next tick, so a BUSY stall can never
double-apply a toggle.

### 1 kHz tick placement

`control_surfaces_tick()` runs every main-loop iteration (after
`dac_hw_mute_tick`), self-throttled to 1 kHz via `time_us_64` and an immediate
no-op while no binding is active. Per tick it debounces buttons/switches (shared
per GPIO, so a pin's gesture group sees one clean edge stream), decodes button
gestures (long-press >= 500 ms, double-press within a 350 ms window, and
hold-to-repeat with a 400 ms delay then 12.5 Hz), decodes encoders with a
quadrature transition table (one detent = 4 quarter-steps) plus optional
acceleration (inter-detent gaps of 128/64/32 ms multiply the step x2/x4/x8),
reads at most one pot ADC channel (round-robin, EMA + deadband + boot-sync
immediate takeover), drives plain and PWM LEDs, and dispatches resulting changes.
Stepping is unit-aware: dB and percent step linearly (default 1 dB / 1 %), Hz and
Q step in octaves (default one-twelfth octave). Enums that are sparse at run
time step across the values on offer rather than the raw index range
(`cs_enum_step`): `PRESET` skips empty slots, `DISPLAY_PAGE` inactive pages,
and `INPUT_SOURCE` sources that fail `input_source_selectable()`, meaning a
disabled optional SPDIF (2-4) and ADAT when disabled or unsupported. The skip
is what makes `CS_FLAG_WRAP` work at the ends: without it a step lands on a
value whose setter silently refuses it, and the control reads as dead.
Deferred nouns (`CS_NDF_DEFERRED`)
are stepped from a per-op float target shadow rather than re-read each tick (the
shadow/retry state was factored into a `CsOpState` shared by binding slots and IR
commands). Read-only indicator nouns are evaluated only every 8 ticks, staggered
by slot to spread the cost. No PIO is used; the IR receiver's edge interrupt is
the engine's only non-polled input (decode still runs on the tick).

PWM LEDs use a hardware PWM slice (wrap 4095 at `sysclk`/16) with a squared
perceptual-brightness curve; two PWM LEDs that would collide on the same slice +
channel are rejected at apply time (`CS_STATUS_PWM_CONFLICT`). The soft VCXO
reserves one PWM slice as its clock pacer (`SOFT_VCXO_PWM_SLICE`: slice 8 on
RP2350, slice 7 on RP2040; see "Soft VCXO"); a PWM LED bind whose GPIO maps to
that slice is likewise rejected with `CS_STATUS_PWM_CONFLICT` (checked on both
platforms). RP2350 slice 8 has no bondable GPIO on RP2350A, so it can never
actually collide; RP2040 slice 7 covers GPIO 14/15.

**Brightness ceiling (caps v12).** `CsBinding.base_bright` (percent 1-100,
0 = full) scales the computed duty in `cs_tick_led_pwm`, so an `IND_LEVEL`
meter keeps its whole sweep and only its top end moves; it covers the
full-on/off indicator actions too, which is how a set of indicators is dimmed
together. The scale is linear in duty, deliberately: the perceptual curve
already lives in the `IND_LEVEL` mapping, and squaring the ceiling as well
would round a 1% setting to fully off against wrap 4095. It is applied before
`CS_FLAG_INVERT`, so active-low wiring keeps a full-rail off state. The field
took the spare `reserved` byte at offset 9, leaving `CsBinding` byte-identical
at 24 bytes, and pre-v12 configs read 0 there, meaning full brightness. A
future global Panel Brightness noun is intended to multiply into this per-LED
value for LEDs that opt in; `CsBinding.flags` has no free bit, so that opt-in
is earmarked for a flags byte out of `reserved2`.

### Deferred SET, Apply/Save/Revert, and boot bring-up
*Last updated: 2026-10-26 (audit fixes: revert result ownership, status snapshot, pin ownership during a revert). Previously 2026-10-24 (revert is a background job with a busy gate and stored-image reads), 2026-10-23 (bulk applies rebuild derived state once)*

`REQ_SET_CS_BINDING` requires the full 24-byte payload; a short payload records
`CS_STATUS_INVALID_VALUE` and is dropped. It otherwise validates only the slot
index and latches the binding (`cs_set_binding_pending`), mirroring
`ctrl_set_uart_pending`. The main loop runs `control_surfaces_apply_binding`
(target validation + pin release/claim + runtime seed) and records
`cs_last_status` / `cs_last_slot` (read via `REQ_GET_CS_STATUS`).
`REQ_SET_CS_IR_CMD` follows the identical single-deep deferred shape for a
16-byte `IrCommand` sub-slot, reported as `cs_last_slot = 0x80 | sub`.

**Apply-live-only preview (v3):** no CS SET (binding, IR command, or slot
name) persists to flash. A successful apply marks the live config dirty
(`CsStatusPacket.dirty`, the former reserved byte). `REQ_CS_SAVE` (0x9D,
deferred via `cs_save_pending`) persists the bindings, the IR table, and the
slot names together in one directory write (`preset_set_cs_all`) inside the
usual `prepare_flash_write_operation` brackets and clears dirty;
`REQ_CS_REVERT` (0x9E, `cs_revert_pending`) re-applies the stored config from
the directory cache with no flash write. A reboot is an implicit revert.

**Revert as a background job (2026-10-24).** The reload is one sequence of
small units (`cs_reload_unit()`, phases `CS_RL_*`): stash live aux values,
tear down each slot through the normal release path, reset buttons and the IR
table and reset-and-reload groups and macros, load each stored aux slot,
reset-and-reload the display tables, load each other stored slot, load the IR
commands, then names, macro recount and `cs_bindings_changed()` once. Boot runs
it from the reset phase straight through; `control_surfaces_revert_begin()`
runs all of it as a background job (Output Latency "Step budget and background
jobs"), each unit estimated from the last unit of the same phase and binding
type (an empty slot costs about 1 us, a display attach about 120 us cold, so
one estimate per phase let a costly slot follow a cheap one past the deadline). Its completion
drops unclaimed aux stashes, clears the dirty flag and posts `cs_last_status`
SUCCESS, so hosts keep polling PENDING as before. It posts SUCCESS only while
the result is still its own (PENDING, slot 0xFF, no Control Surfaces request
queued): a request accepted mid-revert reports its own PENDING and runs next,
and a refusal for being busy never writes `cs_last_status` (2026-10-26). The stored image is
read in place (`preset_stored_cs_config()`, `preset_stored_cs_ir_config()`), so
the reload no longer copies the 388 and 260-byte blobs onto the stack.

While the job runs, `control_surfaces_busy()` is true and the module guarantees
that nothing interleaves with it or sees it half done:
- **Writes wait or report BUSY.** `main.c` skips its whole Control Surfaces
  deferred block, so queued binding, name, IR, group, macro, display, save and
  revert requests stay pending and run afterwards in order (a SET sent just
  after a revert is applied after it, where the one-call revert used to run
  after it and discard it). Every public apply, aux write, macro fire and IR
  learn returns `CS_STATUS_BUSY` (or false), whatever path reaches it.
- **Reads answer with the result.** A revert converges on the stored image for
  any image this firmware saved, so `REQ_GET_CS_BINDING` and the IR command
  reads answer from it, and aux state and level reads answer with the value the
  slot comes back with (live while still up as the same type, else the stash
  or its stored boot fields). Groups, macros, display tables and names each
  change within one unit, so their reads stay live. `REQ_GET_CS_STATUS` answers
  its per-slot fields (active masks, slot and IR command status) from a snapshot
  taken when the revert started, with the result and dirty fields live, so it is
  never torn either. No CS GET newly fails, which matters to clients that check
  exact readbacks.
- **Pins stay claimed.** `control_surfaces_owns_pin()` also reports the pins of
  the stored bindings while the revert runs, so a pin SET landing between a
  slot's teardown and its reload cannot take a GPIO the reload is about to claim.
- **The tick keeps running.** Torn-down slots are inactive, the button groups
  are rebuilt only at the finish, and a macro fired by a control mid-revert is
  refused.

`tools/dspi_test` group `control_surfaces` checks it: a revert lands on the
stored bindings with dirty cleared, a SET sent during a revert is applied after
it, and binding reads polled across 20 reverts all equal the stored image. A
revert lasts about a millisecond, shorter than one host read, so on a
`DSPI_LOOP_PROF` build the test slows background jobs to one unit per 50 passes
(`REQ_GET_STATUS` 512+n) to land reads mid-revert (159 of them on 2026-10-24).

The per-slot work and the state derived from the whole binding set are split.
`cs_apply_binding_slot()` releases, validates, claims and seeds one slot, and
`cs_bindings_changed()` recounts the any-active flag and rebuilds the button
gesture groups, which only the tick reads. `control_surfaces_apply_binding()`
is the two together; the reload runs the per-slot half per unit and the
derived half once, and gesture state restarts as at boot.

`control_surfaces_init()` runs last in `core0_init()` (after
all pin claims and `notify_init`); a stored binding whose pins now collide is
kept down but preserved, with the failure visible in `slot_status[]` (stored
IR commands behave the same via `ir_cmd_status[]`). v2
status codes: `CS_STATUS_INVALID_TARGET` (0x17), `CS_STATUS_INVALID_EVENT`
(0x18), `CS_STATUS_PWM_CONFLICT` (0x19), `CS_STATUS_EVENT_IN_USE` (0x1A, a
GPIO+event pair already claimed by another button binding), and
`CS_STATUS_BUSY` (0x1B, a SET arrived while a previous SET was still queued
for the main-loop apply; the new SET is dropped and the host retries). v3
adds `CS_STATUS_IR_IN_USE` (0x1D, a second IR component) and
`CS_STATUS_NO_IR` (0x1E, learn armed without a live IR component);
`CS_STATUS_FLASH_ERROR` (0x1C) now also reports a failed save.

### IR remote component (`CS_TYPE_IR`, 0x8D-0x8F)

One binding slot holds the receiver (one GPIO; `CS_FLAG_INVERT` = idle-low
module; every other binding field must be 0; single instance). Its remote
buttons are up to `CS_MAX_IR_COMMANDS` (16) `IrCommand` sub-slots in a
separate table: button-subset
noun/action records (INC/DEC/TOGGLE/SET/TRIGGER/MOMENTARY, WRAP/REPEAT flags)
fired by a learned `{protocol, code}` pair instead of a GPIO edge, validated
against the same caps masks and dispatched through the same `cs_noun_dispatch`
path with per-command `CsOpState` (BUSY retry, deferred-noun shadow,
momentary restore). Multiple commands may share one code (one button, several
actions). Hold semantics mirror physical buttons: a NEC repeat frame always
extends the hold, and 250 ms of silence releases it (restoring MOMENTARY);
REPEAT events are gated to the button feel of 400 ms delay then 12.5 Hz.

**Hold vs re-press (2026-07-25).** When a frame carries the code already held,
the strongest available evidence decides whether it is the same press
continuing or a fresh one, because no consumer IR protocol has a release
message:

1. *RC5/RC6* carry a toggle bit that flips once per new press and holds for
   the life of a hold. It is compared directly, so re-presses are exact and
   timing-independent. The bit stays masked out of `IrCommand.code` so one
   learned code still matches every press.
2. *Handsets observed marking holds with NEC repeat frames* cannot emit a data
   frame mid-hold, so an arriving data frame can only be a new press. The
   observation is made at runtime the first time a repeat frame extends a hold
   (an ordinary press longer than ~110 ms is enough) and remembered in an
   8-entry RAM table, keyed by NEC address so one hold teaches every button on
   that handset; other protocols have no address field and key on the full
   code. The table is not persisted and is kept across attach/detach.
3. *Otherwise* the repeat is a bit-identical re-transmission and only the gap
   separates the two, so the same code inside 250 ms is taken as a REPEAT.
   This caps such remotes at roughly 4 taps/second, which is unavoidable
   without protocol-level information.

Cases 1 and 2 lift the tap-rate cap entirely for the remotes they cover. The
250 ms window survives only as the release timeout (case 3's discriminator),
sized to outlast a dropped frame at the longest repeat period in use
(Kaseikyo ~130 ms).

Learn (`REQ_CS_IR_LEARN`: wValue 1
arm / 0 cancel / 2 read result) captures the next decoded press within 10 s,
suppressing dispatch while armed, and completion is pushed as notify event
`NOTIFY_EVT_CS_IR_LEARN` (0x0A: state, protocol, code) as well as being
readable synchronously. Commands may be stored before the component exists
and activate when it comes up.

### Per-slot names (0x8B/0x8C, V10)

Each of the 16 slots carries a device-persistent 32-byte NUL-terminated user
label (`CS_NAME_LEN`, same convention as preset and channel names), set by the
host app via `REQ_SET_CS_NAME` (0x8B) and read via `REQ_GET_CS_NAME` (0x8C),
so external MCUs on UART/I2C and apps on other hosts can display what each
control is for. Names are slot metadata independent of the binding: they
survive binding changes and slot clears, and may be set before a binding
exists. Like bindings and IR commands, the SET is an apply-live-only
preview: deferred to the main loop via `cs_set_name_pending`, it updates the
engine's live name table (`control_surfaces_apply_name`) and marks the config
dirty, with no flash write; `REQ_CS_SAVE` persists the names alongside the
bindings and IR commands, and `REQ_CS_REVERT` (or a reboot) restores the
stored names. The SET reports through the shared `cs_last_status` /
`cs_last_slot` channel: `CS_STATUS_PENDING` then `PIN_CONFIG_SUCCESS`;
`CS_STATUS_BUSY` if a previous name SET is still queued. The GET returns the
live name (`control_surfaces_get_name`; the unsaved preview while dirty). A
payload of one NUL byte clears the name; names are not part of
`WireBulkParams` and emit no notification.

### Target groups and macros (caps v9, 0x20-0x26)

*Last updated: 2026-10-10 (macro steps wait for a queued preset load, factory reset or bulk apply to land)*

Wire detail (every struct, status code and command payload) is in
`Documentation/Features/control_surfaces_groups_macros_spec.md`; this
subsection covers the firmware structure only.

**Target groups.** Eight device-global named channel sets, each a
`{target_kind, member_mask, name}` record in `CsGroup` (40 B) held in the live
`CsGroupConfig` (324 B). `target_kind` reuses the existing `CS_TARGET_*`
channel spaces, and bit N of `member_mask` is channel N of that space. A
binding (or macro step) references one by setting `CS_FLAG_GROUP` (0x20) in
`flags`, which makes the engine re-read `target` as a group index instead of a
channel index; the noun's `target_kind` must be compatible with the group's
(`DSP_BAND` nouns take a `DSP_CH` group and validate `index` per member).
Groups are pure configuration: created and edited by the host, persisted with
the rest of the CS config, and referenced by index. IR commands do not carry
the flag; a remote button reaches a group by firing a macro.

**Link laws.** Continuous nouns default to offset-preserving: relative ops
(`STEP`, `INC`/`DEC`) step every member from its own value so a trim between
members survives, and absolute `ADJUST` from a pot moves the group *mean* to
the pot position while each member keeps its offset from the mean captured at
the start of the gesture. `CS_FLAG_LINK_ABS` (0x40) selects
absolute-identical instead, driving every member to the same value; it is
valid only on `ADJUST`, the one action where the two laws differ. Clamping is
per member, so winding into a rail compresses the spread exactly like ganged
faders. Bool and enum nouns use the **anchor rule**: the lowest-numbered
member is read as the anchor, the action is computed against it, and every
member is driven to the anchor's new value, so a half-muted group becomes
coherent on the first press.

**Grouped indicators.** `IND_EQUALS` / `IND_ABOVE` evaluate per member and
combine with OR (lit when any member matches); `CS_FLAG_GROUP_ALL` (0x80)
selects AND. The caps v8 TON/TOF `on_delay`/`off_delay` filter applies to the
combined condition. `IND_LEVEL` follows the maximum member value and rejects
`GROUP_ALL`.

**Fan-out engine.** A grouped op computes every member's absolute target up
front (`CsGroupOp`, one per binding slot plus one for the macro sequencer),
then dispatches member-by-member in the same tick through the usual
`cs_noun_dispatch`. Members whose dispatch returns BUSY stay in a per-slot
`pend_mask` and retry on later ticks; new detents arriving while members are
pending fold into the same op, so no input is lost and no member
double-steps. Relative ops and offset-preserving `ADJUST` run as a *gesture
session*: at the first event after idle (or after `CS_GROUP_SESSION_TICKS` =
500 ms of quiet) the engine captures each member's live value as its base and
thereafter accumulates one delta for the whole session, so targets are always
`clamp(base_m + delta)`. That replaces the single-target per-op float shadow
for grouped nouns without needing per-member shadow state. `MOMENTARY`
captures every member at the press and restores each on release, including
when the binding is torn down mid-hold. Applying a group SET re-validates
every binding that references a group; one that no longer validates is
deactivated with its failure in `slot_status[]` and reactivates when a later
edit makes it valid again, so there is no in-use refusal.

**Macros.** Eight device-global `{name, step_count, steps[8]}` records
(`CsMacro` 132 B, `CsMacroConfig` 1060 B). A step is a stripped binding
(`CsMacroStep`, 12 B) restricted to the `SET` / `TOGGLE` / `INC` / `DEC` /
`TRIGGER` actions, optionally grouped, with a `pre_delay` in 10 ms units that
elapses before the step runs. Macros are fired through the appended noun
`CS_NOUN_MACRO` (52), an enum of `max_macros` positions that accepts `SET`
(fire macro `value`) and `IND_EQUALS` (lit while that macro runs), so any
button gesture or IR command fires one with no change to `CsBinding` or
`IrCommand`; `REQ_CS_MACRO_FIRE` gives hosts and external MCUs the same
trigger. There is a single sequencer, so one macro runs at a time: a new fire
cancels the running one at its current step boundary (remaining steps
dropped, the in-flight step not rolled back) and starts the new one. Steps
are re-validated at fire time and a step that no longer validates (its group
emptied or re-kinded) is skipped rather than fatal, which makes a torn host
edit skip-safe. Steps may not fire macros, so there is no nesting.

Every step dispatches through the same `vendor_dispatch_set` / `_get` surface
with `CTRL_SOURCE_GPIO` that a single-target binding uses, honouring the same
BUSY back-pressure, so a macro that loads a preset or switches input inherits
the existing deferred, pipeline-safe apply machinery; nothing here adds a path
to the audio pipeline and output slot alignment is untouched by construction.
The sequencer waits for a step's dispatches to leave the BUSY-pending state
before starting the next step's delay.

A dispatch that queues a whole-state replace (`preset_load_pending`,
`factory_reset_pending` or `bulk_params_pending`) returns "accepted" at once,
but the replace only lands after the pipeline fade, which can take tens of
milliseconds. Any step dispatched in that window would be overwritten when the
replace lands. The sequencer therefore treats a step as unfinished until all
three flags are clear, so the next step's `pre_delay` counts from the landed
load. It also holds a step at its fire point while one of the flags is set,
which covers a load queued mid-macro by the host or another control. The fade
is time-capped, so a held macro always resumes.

**Control plumbing.** `REQ_SET_CS_GROUP`, `REQ_SET_CS_MACRO` and
`REQ_SET_CS_MACRO_STEP` are single-deep deferred SETs in the established shape
(`cs_set_group_pending` / `cs_set_macro_hdr_pending` / `cs_set_macro_step_pending`
consumed by the main loop into `control_surfaces_apply_group` /
`_apply_macro_header` / `_apply_macro_step`), reported through the shared
`cs_last_status` / `cs_last_slot` channel with tags `0x40 | group` and
`0x60 | macro`. They are apply-live-only previews like every other CS SET:
`REQ_CS_SAVE` persists groups and macros alongside the bindings, names and IR
table in the one directory write, and `REQ_CS_REVERT` (or a reboot) restores
the stored set. `REQ_GET_CS_EXT_STATUS` returns the 24-byte
`CsExtStatusPacket` (limits, running macro and step, per-group and per-macro
validity). New status codes are `CS_STATUS_INVALID_GROUP` (0x1F),
`CS_STATUS_INVALID_MACRO` (0x20) and `CS_STATUS_INVALID_STEP` (0x21). At boot
groups and macros load before bindings, since bindings validate against
groups.

### I2C display component (caps v10-v13, 0x27-0x2B)
*Last updated: 2026-09-05 (CS_DPAGE_BAR added to the directory sanitize mask, so saved level-bar pages survive a boot; 2026-08-24: alignment-aware edit marker placement; caps v13: level bars; caps v11: per-line alignment, edit markers)*

Wire detail (every struct, status code, model table and command payload) is in
`Documentation/Features/control_surfaces_display_spec.md`; this subsection
covers the firmware structure only.

**The component.** `CS_TYPE_DISPLAY` (8) is a container binding like the IR
receiver: one live display per device, `gpio[0]` = SDA, `gpio[1]` = SCL,
`index` = model (1-8), `value` = 7-bit address or 0 for the model default,
every other field 0. Eight models are supported: HD44780 16x2 / 20x4 behind a
PCF8574 backpack (100 kHz), US2066/RW1063 character OLEDs in 16x2 / 20x2 /
20x4 (400 kHz), and SSD1306 128x64 / 128x32 plus SH1106 128x64 graphic OLEDs
(400 kHz, font-rendered). Validation requires SDA on an even GPIO and SCL on
an odd one mapping to the same hardware I2C instance (GPIO bit 1 selects
i2c0/i2c1, bit 0 selects SDA/SCL), that instance not being the one the I2C
target control interface occupies (`i2c_ctrl_live_instance()`), and no second
display slot; new status codes are `CS_STATUS_DISPLAY_IN_USE` (0x22),
`CS_STATUS_PIN_NOT_I2C` (0x23), `CS_STATUS_I2C_IN_USE` (0x24) and
`CS_STATUS_INVALID_PAGE` (0x25). `cs_claim_pins` calls `cs_display_attach()`
(which does the pin mux, pull-ups and peripheral setup itself) and
`cs_release_pins` calls `cs_display_detach()` before the ordinary GPIO
release.

**Driver.** The device is I2C master and everything runs on the existing
1 kHz CS tick with no blocking. Bus traffic goes through a 64-entry uint16 TX
ring whose words are the DW `data_cmd` encoding verbatim (low 8 bits data,
bit 9 STOP) plus `0x8000 | ms` delay sentinels, which is how the HD44780 long
commands are timed. Per tick a producer (the per-model init script first,
then dirty text rows) fills the ring while space allows, and a pump drains it
into the I2C TX FIFO while the FIFO has room, so the hardware transmits in
the background and a dirty row lands in a few tens of ticks. A TX abort (NAK,
arbitration loss) increments a saturating counter, flushes the ring, parks
the component in an ERROR state and re-runs init after ~1 s, which makes
hot-plug work. There is no framebuffer: content is two logical text lines
(label and value) diffed against their last-transmitted copies into a
per-physical-row dirty mask, and graphic models emit font columns on the fly
from a single 5x8 flash table (~480 B); LARGE text is that same font
pixel-doubled at render time into 10x16 glyphs (12 columns on 128 px, value
spanning two page rows). Render runs on demand or every 32 ticks.

**Line layout (caps v11).** Each line is laid out across its physical width
by `disp_lay_out()` after formatting and before the change diff, so alignment
is invisible to everything downstream and adds no I2C traffic (a dirty row
rewrites every column regardless). Width is the model's column count, except
a LARGE value on a graphic model, which is 12. Horizontal alignment is two
2-bit fields in `CsDisplayCfg.flags` (bits 3:2 label, 5:4 value; 0 left,
1 centre, 2 right), independent per line and applied to every view including
the overlay and the idle line; the reserved encoding 3 is rejected by the
apply path and read as left by the renderer. While edit is armed the value
line takes the `>` / `<` markers (`!` when the item is read-only) in the
margin the alignment leaves free, one blank column clear of the value: right
marker only when left-flushed, left marker only when right-flushed, both when
centred. The value is placed against the full width either way, so arming
never moves it at any alignment; the marker tracks the value's length
instead. A value long enough to reach a marker is truncated to width minus 2
(minus 4 centred), the only case where arming reflows the line. On the merged
label+value line of a 2-row bar page the value keeps the right edge, so the
marker sits two columns left of it and the label gives up the room.
Vertical placement stays a per-model
constant (`label_row` / `value_row`), which is what centres the pair on 4-row
character modules.

**Content model.** A page is a 4-byte `{noun, target, index, flags}` record
from the ordinary noun catalog (16 device-global slots), rendered generically
by the noun's kind and unit, with flags for ACTIVE / GROUP / LARGE. Enum
nouns with fixed value sets render names from per-noun label tables
(`disp_enum_label()`: input source, sample rate, crossfeed preset, leveller
speed, PEQ filter type incl. the host-only types 11-13, upmix centre and
surround modes; the upmixer's passive/adaptive engines are labelled
"Sinner"/"Logician"); preset and macro show their stored names; only enums
with no table entry (out-of-range or unrecognised values) fall back
to `#N`. Filter types use slope-based names (PEQ low/high pass = "High
Cut"/"Low Cut", e.g. "Low Shelf 12dB") from two tables: full words for the
small font, abbreviations plus "dB/oct" ("LS 12dB/oct") when the value
renders in the 12-column LARGE font on a graphic model. Home
content follows `CsDisplayCfg.mode`: fixed page, rotate the active pages at
`dwell`, or rotate the untargeted platform-available nouns. Over the top of
that, an **event overlay** pops whatever just changed for `overlay_hold`:
every op initiator in the engine calls `cs_display_note_adjust()`, so a knob,
remote key or macro step gives feedback even for an item with no configured
page (`CS_DCFG_OVERLAY_ANY`), and a ~8 Hz compare poll over the active pages
catches host-side changes that no control caused. The three display nouns and
`CS_NOUN_MACRO` never pop as themselves. The first attach on an empty page
table seeds volume / preset / input source / sample rate and the timing
defaults, marking the config dirty like any other live edit.

**Level bars (caps v13).** `CS_DPAGE_BAR` plots the value's position within
the noun's own range (`cs_noun_span()` decodes `min_q`/`max_q`, so no page
field grows), log-mapped for Hz and Q like the meter LEDs. Character panels
draw it from five CGRAM cells appended to the init script (codes 1-5 fill 1-5
glyph columns, giving `5 * cols` steps; code 0 stays unused so bar strings
remain NUL-terminated, and the solid cell comes from CGRAM rather than the
ROM block at 0xFF). `DispModelDesc.bar_row` says where it lands: a 4-row
panel spends its spare bottom row, a 2-row panel has none to spare so the
value folds into the label row (compact `O1`/`C3B4` prefixes, unit space
dropped, value flush right and never truncated, label giving up columns).
Graphic panels set `DISP_NO_BAR_ROW` and spend no row at all: since the
renderer emits font columns on the fly, the bar is an inverted run XORed over
the value row, 128 steps, glyphs knocked out of a lit block. Neither the
merged line nor a bar row takes the configured alignment. `CS_DPAGE_BAR` was
missing from `dir_sanitize_cs_display()`'s accepted flag mask, so since caps
v13 every saved page carrying a bar was cleared at the next boot. The mask now
includes it (fixed alongside the first auxiliary output work).

**Front-panel editing.** `CS_NOUN_DISPLAY_PAGE` (54) browses the active pages
(steps skip empty slots through the same `cs_enum_step` occupancy path as
`CS_NOUN_PRESET` and `CS_NOUN_INPUT_SOURCE`), `CS_NOUN_DISPLAY_EDIT` (55)
arms editing with an
inactivity auto-disarm, and `CS_NOUN_PAGE_VALUE` (56) is virtual: it resolves
the shown page's `{noun, target, index}` at event time and re-enters the
normal op helpers with a synthesized binding, so grouped pages fan out
through the group engine and deferred nouns keep their shadow/BUSY
behaviour. With `CS_DCFG_EDIT_GATED` set, which page seeding now does by
default, an unarmed `PAGE_VALUE` step navigates pages instead of adjusting,
which turns one encoder plus one button into browse / arm / adjust.
`PAGE_VALUE` accepts STEP/INC/DEC/TOGGLE only,
must carry zeroed value/step/range fields (nothing static to validate against),
and its op plus group-session state resets whenever the resolved item changes
(a per-binding and per-IR-command resolved-item key). Buttons, encoders and
IR commands may bind it; macro steps may **not**, rejected exactly like
`CS_NOUN_MACRO` because a stored sequence editing whatever is on screen is
non-deterministic.

**IR group support.** `IrCommand.flags` now accepts `CS_FLAG_GROUP` (0x20),
so a remote key reaches a target group directly instead of via a macro, with
hold-to-repeat on grouped INC/DEC and grouped MOMENTARY engage/restore. A
receiver delivers one key at a time, so grouped IR ops share a **pool of 2**
`CsGroupOp` contexts rather than one per command slot; one may be pinned by a
held grouped momentary while another key fires, and pool exhaustion drops the
new op for that press. The CS tick pumps BUSY members, ages sessions and
frees idle contexts back to the pool; receiver teardown restores any engaged
grouped momentary first.

**Control plumbing.** `REQ_SET_CS_DISPLAY_CFG` (0x27) and
`REQ_SET_CS_DISPLAY_PAGE` (0x29) are single-deep deferred SETs in the
established shape (`cs_set_disp_cfg_pending` / `cs_set_disp_page_pending`
consumed by the main loop into `control_surfaces_apply_display_cfg` /
`_apply_display_page`), reported through the shared `cs_last_status` /
`cs_last_slot` channel with tag `0x50` bare for the config and `0x50 | page`
for a page. They are apply-live-only previews: `REQ_CS_SAVE` persists the
80-byte blob alongside bindings, names, IR commands, groups and macros in the
one directory write, and `REQ_CS_REVERT` (or a reboot) restores the stored
set. `REQ_GET_CS_DISPLAY_CFG` (0x28) prefixes the live config with
`{max_pages, model_count, reserved[2]}` so a host learns the limits without a
caps change, `REQ_GET_CS_DISPLAY_PAGE` (0x2A) returns one 4-byte record, and
`REQ_GET_CS_DISPLAY_STATUS` (0x2B) returns the 8-byte `CsDisplayStatus`
(init state, shown page, overlay/edit flags, live model, cumulative NAK
count). At boot the display blob loads before the bindings, so a stored
display binding's attach sees its stored pages and does not re-seed.

### Auxiliary outputs (caps v18, 0x04-0x07)
*Last updated: 2026-09-07 (reworked as binding-slot components: CS_TYPE_AUX_OUT 9 / CS_TYPE_AUX_PWM 10, nouns 68-69 target the slot, 8.8 level, commands 0x04-0x07, directory V21; caps v17's pinless table never shipped)*

Wire detail (every field, status code and command payload) is in
`Documentation/Features/control_surfaces_aux_spec.md`; this subsection covers
the firmware structure only.

**What they are.** GPIOs the firmware attaches no audio meaning to, for an
amplifier trigger, a speaker relay, a panel lamp or a fan. Each is a component
in one of the 16 binding slots, either `CS_TYPE_AUX_OUT` (9, on/off) or
`CS_TYPE_AUX_PWM` (10, dimmable on a hardware PWM slice). Both are container
types like `CS_TYPE_IR` and `CS_TYPE_DISPLAY`, with an all-zero caps action
mask, one pin and pin class ANY. The slot owns the pin, the per-slot name, the
invert sense, the TON/TOF delays, the PWM ceiling and the boot values, so an
aux output is configured exactly the way a button or an LED is. Nothing in the
audio path reads them.

**How a control reaches one.** Through the noun, with `target` = the slot
holding the output and target kind `CS_TARGET_AUX` (5, `target_count` =
`CS_MAX_BINDINGS`). `CS_NOUN_AUX` (68) is the bool half (BOOL-RW) and
`CS_NOUN_AUX_LEVEL` (69) the continuous percent 0..100 (CONT-RW, PWM slots
only). Both dispatch through `vendor_dispatch_set(CTRL_SOURCE_GPIO, ...)` with
`REQ_SET_CS_AUX_STATE` / `REQ_SET_CS_AUX_LEVEL` like every other noun, so a
panel toggle and a host write take the same path. The level is 8.8 percent in
RAM, on the wire and in the noun value, so any `step` is valid and there is no
rounding nudge. Groups are not supported (a grouped reference or a `CsGroup`
with `target_kind` 5 is rejected with `CS_STATUS_INVALID_GROUP`), and the
display's `CYCLE_ALL` mode walks untargeted nouns only, so aux outputs reach
the panel as explicit pages. Ordinary LED bindings may still follow noun 68 or
69 as optional extra indicators.

**Validation and dependents.** `cs_noun_validate_target_ch()` is strict at bind
time: noun 68 needs either aux type in the target slot, noun 69 needs
`CS_TYPE_AUX_PWM`, and anything else is `CS_STATUS_INVALID_AUX` (0x26), whose
meaning is now "target slot is not an aux output, or the level noun targets a
non-PWM aux slot". An out-of-range target or non-zero index stays
`CS_STATUS_INVALID_TARGET`. When an aux slot appears, vanishes or changes kind,
`cs_revalidate_aux_dependents()` re-checks every non-grouped binding, IR
command and macro step on nouns 68/69 targeting it, taking active ones down
(releasing their pins) and resurrecting down ones that validate again, the same
rule a group edit follows. Display pages are not re-validated; a page whose
target stops being an aux simply shows Off or 0 %.

**Live values and pin drive.** The on/off flag and 8.8 level live in the slot's
`CsRuntime` record, and exist only while the slot is UP. A slot held down by a
pin conflict reads 0 and refuses writes, since a value set then would be lost
to the boot fields when it came back. `cs_tick_aux()` runs at the indicator
decimation (every 8 ms per slot). An `AUX_OUT` pin follows the flag through the
same `cs_ind_delay()` TON/TOF filter an indicator LED uses, with
`CS_FLAG_INVERT` applied last. An `AUX_PWM` pin emits its level only while the
flag is on, so off is 0 % duty; `cs_aux_duty()` squares the normalised level by
default (linear with `CS_AUX_X_LINEAR`), scales by `base_bright`, then inverts.
`cs_type_is_pwm()` now covers both `CS_TYPE_LED_PWM` and `CS_TYPE_AUX_PWM`, so
they share the slice-output conflict check and the slice-release rule. In
`cs_claim_pins()` the pin is driven to its initial value *before* the direction
or PWM mux is set, so a relay never glitches through the off state on the way
up.

**Config fields.** All in the ordinary 24-byte `CsBinding`. `gpio[0]` is the
pin, `flags` may carry only `CS_FLAG_INVERT`, `on_delay`/`off_delay` are the
TON/TOF filter, `base_bright` is the PWM ceiling (AUX_PWM only), `value` is the
boot level in 8.8 percent (AUX_PWM only), and byte 22, formerly `reserved2[0]`,
is now `extras`. Its bits are `CS_AUX_X_BOOT_ON` (0x01), `CS_AUX_X_BOOT_SAVED`
(0x02), `CS_AUX_X_LINEAR` (0x04, AUX_PWM only) and mask `CS_AUX_X_MASK` (0x07).
`extras` must be 0 on every other type, so pre-v18 configs stay valid.
`cs_validate_aux_container()` rejects any other non-zero field.

**Boot, save, revert.** `cs_load_stored_bindings()` runs a two-pass load with
aux slots applied first, so a control that targets one validates whatever the
slot order. The boot flag comes from `CS_AUX_X_BOOT_ON` and the boot level from
`value`, with the delay filter starting in agreement with the pin rather than
timing the boot value. `control_surfaces_aux_prepare_save()` (called from the
`REQ_CS_SAVE` handler in `main.c` before the config is copied out) folds each
UP `CS_AUX_X_BOOT_SAVED` slot's live flag and level back into
`CS_AUX_X_BOOT_ON` and `value`. Nothing writes flash on a toggle and nothing
sets the dirty flag, and there is deliberately no "remember on every change"
mode, because a flash write freezes the audio clocks for roughly 44 ms.
`REQ_CS_REVERT` stashes the live flag and level of every UP aux slot in
`s_aux_carry[]` and hands each back to the slot that reloads with the same
type, so a revert never snaps a relay to its boot value; `cs_seed_runtime()`
consumes the carry. A host binding SET that keeps the same type (rename, delay,
ceiling, boot-field edit) carries the value the same way. Changing the type
resets the slot to its boot value.

**Runtime SETs and notification.** `REQ_SET_CS_AUX_STATE` (0x04, 1 byte) and
`REQ_SET_CS_AUX_LEVEL` (0x06, 2 bytes 8.8 LE, clamped to 25600) apply in the
handler with no flash, no dirty flag and no deferral, from USB, UART, I2C or
the CS engine, with `wValue` = the binding slot. A slot that is not an aux
output that is UP is rejected with `CS_STATUS_INVALID_AUX` and `cs_last_slot` = the
plain slot number (UART/I2C return `CTRL_DISPATCH_ERROR`; USB acknowledges the
OUT stage as it does for every OUT command, so the host reads `REQ_GET_CS_STATUS`).
The old `0x70 | aux` tag is gone, and `REQ_GET_CS_STATUS` /
`slot_status` report an aux slot like any other binding slot. A short payload
is rejected with `CS_STATUS_INVALID_VALUE`. Each changing write pushes
`NOTIFY_EVT_CS_AUX` (0x0C), now a 9-byte packet
`[ver=2, 0x0C, flags=0, seq, slot, state, level_q8_LE, src]` carrying both
values and the dispatch source (`PARAM_SRC_GPIO` for a bound control,
`HOST_SET` / `UART` / `I2C` for the transports), which is how the Console
learns a panel button moved an aux output. Unchanged writes are silent.
`REQ_GET_CS_AUX_STATE` with `wValue = 0xFFFF` returns 48 bytes (`state[16]`
then sixteen little-endian 8.8 `level_q8`, zero on slots that are not UP aux
outputs) for a host's initial sync.

**Display.** An aux page's label is the slot's own name when set, else "Aux N"
(N = slot + 1), with " Level" (" Lvl" on the tight two-row bar layout)
appended for the level noun. Values render "On"/"Off" and "NN%", the latter
supporting the caps v13 level bar, and both nouns are writable so a page is
editable from the panel.

### Persistence and platform placement

The binding table is device-global in the preset directory (388-byte
`CsFlashConfig`: version + 16x 24-byte `CsBinding`), board-level like
`dac_hw_mute` and the control-interface config; it survives preset changes and
factory reset and is not part of `WireBulkParams`. The per-slot names live
next to it (V10, `cs_names[16][32]`) and the IR command table follows (V11,
132-byte `CsIrConfig`: version + 8x 16-byte `IrCommand`), with the group table
(V18, 324-byte `CsGroupConfig`), macro table (V18, 1060-byte `CsMacroConfig`)
and display blob (V19, 80-byte `CsDisplayFlash`) appended last, all with the
same lifetime. Auxiliary outputs need no blob of their own: since caps v18
they are components inside `CsFlashConfig`, named through `cs_names`. On RP2040
`control_surfaces.c.o`, `control_surfaces_nouns.c.o`, and the decode side of
`control_surfaces_ir.c.o` execute from flash XIP
(see Memory Layout); only the IR edge ISR is RAM-pinned
(`__not_in_flash_func`). Behavior is identical on both platforms (same 16
bindings, 8 IR sub-slots,
same ADC pins 26-28) except that `CS_NOUN_ADAT_ACTIVE` and the six upmixer
nouns (35-40) are RP2350-only (empty
action mask on RP2040), and the `OUTPUT_DELAY` range follows the platform's
delay ring (21 ms RP2040 / 42 ms RP2350 at 48 kHz). New BSS for the IR feature is roughly 0.9 KB on both
platforms (capture ring 256 B, frame buffer 224 B, command table plus per-command
op state ~400 B). Groups and macros add roughly 4.3 KB on RP2350 and 3.6 KB on
RP2040: the 1,384-byte live tables, the same again in the `dir_cache` mirror,
and 17 `CsGroupOp` contexts (16 binding slots plus the macro sequencer) whose
`base[NUM_CHANNELS]` array makes each ~92 B on RP2350 and ~52 B on RP2040.
The caps v10 display adds roughly 750 B more BSS on both platforms and ~12 KB
of flash (per-model drivers and init scripts, the 5x8 font, the noun label
table); `control_surfaces_display.c.o` is cold and runs from flash XIP on
RP2040 like the rest of the engine. The caps v18 auxiliary outputs add
roughly 130 B more BSS on both platforms. That is a 4-byte value-carry entry
per binding slot plus 3 bytes per slot of runtime record for the live on/off
flag and 8.8 level. All of it is main-loop / control path, so nothing new
lands on a hot path and the audio path is untouched.

---

## Vendor Command Reference
*Last updated: 2026-10-23 (0x03 `REQ_OUTPUT_LATENCY`, one opcode for both directions; `REQ_GET_STATUS` wValue 26-28 ring health readouts). Previously 2026-10-02 (0xC4 `SET_MCK_ENABLE` now rejects an enable onto a claimed pin with PIN_IN_USE); 2026-09-22 (0x52 also saves the output limiter settings device-wide; output limiter: 0x81 `REQ_LIMITER`, one opcode for both directions; 2026-09-20: tube preamp: parameter indices now 0-13; the 0x81 saturation meter is removed and 0x81 is unallocated again; 2026-09-18: tube preamp: indexed 0x3E/0x3F; 2026-09-12: RTA V3 bass capability and 82-byte band frames; 2026-09-07: Control Surfaces auxiliary outputs are now 0x04-0x07 slot-indexed with an 8.8 level; 0x02 and 0x03 removed; spectrum analyser V2 bank/fast-only bins, 0x08-0x0F; 2026-09-04: subharmonic synthesizer widened to 0x10-0x1F, 0x2C-0x2F and 0xA9-0xAE; 2026-09-02: subharmonic synthesizer 0x10-0x1A; REQ_GET_BUILD_INFO 0x80 added 2026-09-01: 64-byte git/date build stamp)*

**Band-index map (PEQ and crossover share one address space):**

| Band index | Meaning |
|---|---|
| 0..9 | Active PEQ band (10 bands per channel today) |
| 10..19 | Reserved for future PEQ-count growth; rejected by handlers |
| 20..23 | Crossover band 0..3 (`XOVER_BAND_BASE = 20`) |
| ≥24 | Rejected |

`REQ_SET_EQ_PARAM`, `REQ_GET_EQ_PARAM`, `REQ_SET_BAND_BYPASS`, and `REQ_GET_BAND_BYPASS` all accept the unified band range. Crossover bands (20..23) are rejected on master channels (channel < `CH_OUT_1` = 2). `REQ_GET_EQ_PARAM` packs the band into a 5-bit `wValue` field: `(channel << 8) | (band << 3) | param` (band 0..31, param 0..4); the other three commands carry the band in a full 8-bit field. See `Documentation/Features/crossover_filters_spec.md` for the complete crossover spec.


| Command | Code | Direction | Description |
|---------|------|-----------|-------------|
| REQ_OUTPUT_LATENCY | 0x03 | OUT | Select the output latency option: 1-byte `OutputLatencyId` (1 = 0.67 ms and 2 = 1 ms low latency, 3/4/5/6/7 = 2/3/4/6/8 ms). Deferred to the main loop: directory write plus a full synchronized output restart (one short mute). Re-sending the stored option is a no-op; an unknown ID is ignored over USB and returns ERROR over UART/I2C. See "Output Latency" |
| REQ_OUTPUT_LATENCY | 0x03 | IN | Same opcode, IN direction. wValue 0: 12-byte `OutputLatencyStatus` (requested and effective ID, effective us at the current rate, target frames, batch frames, pending, option count). wValue 1: the option table, `option_count` x 8-byte `OutputLatencyOptionWire` (ID, flags incl. available-now bit 0x80, nominal latency and batch us, effective us now). Other wValues STALL |
| REQ_SET_CS_AUX_STATE | 0x04 | OUT | Set an auxiliary output on/off (wValue = binding slot 0-15, 1 byte, non-zero = on). Immediate, runtime only, no flash, no dirty flag; pushes `NOTIFY_EVT_CS_AUX` on a change. Rejected with `CS_STATUS_INVALID_AUX` unless the slot is an aux output that is up, or `CS_STATUS_INVALID_VALUE` on a short payload (USB acknowledges the OUT stage; read `REQ_GET_CS_STATUS`). Auxiliary output *configuration* is an ordinary binding (`REQ_SET_CS_BINDING`), not a command of its own; 0x02 is unallocated |
| REQ_GET_CS_AUX_STATE | 0x05 | IN | Get an auxiliary output's state (wValue = binding slot: 1 byte; wValue = 0xFFFF: 48 bytes, state[16] then sixteen little-endian 8.8 percent levels, zero on slots that are not up aux outputs) |
| REQ_SET_CS_AUX_LEVEL | 0x06 | OUT | Set an auxiliary output's level (wValue = binding slot, 2 bytes little-endian 8.8 percent, clamped to 100 % = 25600). Immediate, runtime only; pushes `NOTIFY_EVT_CS_AUX` on a change. Rejected with `CS_STATUS_INVALID_AUX` unless the slot is an up `CS_TYPE_AUX_PWM` |
| REQ_GET_CS_AUX_LEVEL | 0x07 | IN | Get an auxiliary output's level (wValue = binding slot; 2 bytes little-endian 8.8 percent, 0 on a `CS_TYPE_AUX_OUT` slot) |
| REQ_RTA_SET_CONFIG | 0x08 | OUT | Set the V3 spectrum analyser config (12-byte `RtaConfig`). STALLs on wrong version or length, unknown tap, an `fft_order` outside the caps range (8..10), or an empty `channel_mask` after masking to the tap's width |
| REQ_RTA_GET_CONFIG | 0x09 | IN | Get the applied 12-byte `RtaConfig` (clamped `avg_ms` and `peak_decay_db_s`, masked `channel_mask`) |
| REQ_RTA_GET_CAPS | 0x0A | IN | wValue 0: the 16-byte `RtaCaps`. wValue 1..: a chunk of the band-centre table, 32 `uint16` Hz values per chunk; a chunk past the end STALLs |
| REQ_RTA_GET_BANDS | 0x0B | IN | wValue = channel; returns that channel's 82-byte `RtaBandFrame` at the applied tap. Counts as a read (auto-off keepalive + auto-start) |
| REQ_RTA_GET_BINS | 0x0C | IN | wValue = byte offset into the bin frame, wLength = chunk size; no lock, validate the header `seq` against the frame's final byte and re-read on mismatch. Counts as a read |
| REQ_RTA_GET_STATUS | 0x0D | IN | Get the 24-byte `RtaStatus`. Deliberately **not** a read: polling status neither keeps the analyser alive nor starts it |
| REQ_RTA_CONTROL | 0x0E | IN | GET-style, like `REQ_SIGGEN_CONTROL`: the action is in wValue (`RTA_CTL_STOP` / `START` / `RESET_AVG`) and the reply is one status byte; an unknown action STALLs |
| REQ_RTA_GET_BANDS_ALL | 0x0F | IN | USB only: every live channel's `RtaBandFrame` back to back, staged in `bulk_param_buf` under the bulk lock (generalised through `vendor_is_bulk_get`). Refused on UART and I2C, which poll 0x0B per channel to stay inside their 132-byte GET cap. An empty live mask answers with a zero-length data stage, not a STALL. Counts as a read |
| REQ_SET_SUBHARM | 0x10 | OUT | Enable/disable the subharmonic synthesizer (1 byte, 0/1) |
| REQ_GET_SUBHARM | 0x11 | IN | Get subharm enabled state (1 byte) |
| REQ_SET_SUBHARM_LOW | 0x12 | OUT | Set 24-36 Hz sub level (4-byte LE IEEE754 float, -30..+12 dB, clamped; -30 = band off) |
| REQ_GET_SUBHARM_LOW | 0x13 | IN | Get 24-36 Hz sub level (4-byte float) |
| REQ_SET_SUBHARM_HIGH | 0x14 | OUT | Set 36-56 Hz sub level (4-byte float, -30..+12 dB, clamped; -30 = band off) |
| REQ_GET_SUBHARM_HIGH | 0x15 | IN | Get 36-56 Hz sub level (4-byte float) |
| REQ_SET_SUBHARM_BOOST | 0x16 | OUT | Set LF boost bell gain (4-byte float, 0..+6 dB, clamped) |
| REQ_GET_SUBHARM_BOOST | 0x17 | IN | Get LF boost bell gain (4-byte float) |
| REQ_SET_SUBHARM_MASK | 0x18 | OUT | Set output mask (2-byte LE uint16; read live, no recompute) |
| REQ_GET_SUBHARM_MASK | 0x19 | IN | Get output mask (2-byte LE uint16) |
| REQ_GET_SUBHARM_HEADROOM | 0x1A | IN | Get the preamp headroom the current subharm config needs (4-byte float dB, 0 while disabled; computed on request) |
| REQ_SET_SUBHARM_TOP | 0x1B | OUT | Set 56-80 Hz sub level (4-byte float, -30..+12 dB, clamped; -30 = band off) |
| REQ_GET_SUBHARM_TOP | 0x1C | IN | Get 56-80 Hz sub level (4-byte float) |
| REQ_SET_SUBHARM_SELECT | 0x1D | OUT | Set selectivity mode (1 byte: 0 all, 1 percussive, 2 sustained; above 2 clamps to 2) |
| REQ_GET_SUBHARM_SELECT | 0x1E | IN | Get selectivity mode (1 byte) |
| REQ_GET_SUBHARM_METER | 0x1F | IN | Get the per-output synthesized-sub peak (NUM_OUTPUT_CHANNELS x uint16 LE, 0..32767 each; 18 B RP2350 / 10 B RP2040) |
| REQ_SET_SUBHARM_SOLO | 0x2C | OUT | Solo the synthesized sub on masked outputs (1 byte, 0/1). Runtime only: never persisted, never on the wire, no notification |
| REQ_GET_SUBHARM_SOLO | 0x2D | IN | Get subharm solo state (1 byte) |
| REQ_SET_SUBHARM_LINK | 0x2E | OUT | Synthesize each output pair's sub from the pair's mono sum (1 byte, 0/1; read live, no recompute) |
| REQ_GET_SUBHARM_LINK | 0x2F | IN | Get subharm pair-link state (1 byte) |
| REQ_SET_SUBHARM_DEPTH | 0xA9 | OUT | Set selectivity depth (4-byte float, 0..100 %, clamped) |
| REQ_GET_SUBHARM_DEPTH | 0xAA | IN | Get selectivity depth (4-byte float) |
| REQ_SET_SUBHARM_HOLD | 0xAB | OUT | Set selectivity hold time (4-byte float, 50..400 ms, clamped) |
| REQ_GET_SUBHARM_HOLD | 0xAC | IN | Get selectivity hold time (4-byte float) |
| REQ_SET_SUBHARM_CEILING | 0xAD | OUT | Set the sub ceiling (4-byte float, -40..0 dBFS, clamped; 0 = off) |
| REQ_GET_SUBHARM_CEILING | 0xAE | IN | Get the sub ceiling (4-byte float) |
| REQ_SET_CS_GROUP | 0x20 | OUT | Set a Control Surfaces target group (wValue=group 0-7, payload=40-byte CsGroup; all-zero clears); apply-live-only preview, deferred, poll 0x87 (last_slot = 0x40\|group) |
| REQ_GET_CS_GROUP | 0x21 | IN | Get the live 40-byte CsGroup (wValue=group 0-7) |
| REQ_SET_CS_MACRO | 0x22 | OUT | Set a macro's name and step count (wValue=macro 0-7, payload=36-byte CsMacroHeaderWire); apply-live-only, deferred, poll 0x87 (last_slot = 0x60\|macro) |
| REQ_GET_CS_MACRO | 0x23 | IN | Get the live 132-byte CsMacro (wValue=macro 0-7) |
| REQ_SET_CS_MACRO_STEP | 0x24 | OUT | Set one macro step (wValue=(step << 8)\|macro, payload=12-byte CsMacroStep; all-zero clears); apply-live-only, deferred, poll 0x87 |
| REQ_CS_MACRO_FIRE | 0x25 | IN | Fire a macro (wValue=macro 0-7; 0xFFFF cancels the running one); returns 1 status byte |
| REQ_GET_CS_EXT_STATUS | 0x26 | IN | Get the 24-byte CsExtStatusPacket (group/macro limits, running macro and step, per-group and per-macro validity) |
| REQ_SET_CS_DISPLAY_CFG | 0x27 | OUT | Set the Control Surfaces display config (payload=12-byte CsDisplayCfg); apply-live-only preview, deferred, poll 0x87 (last_slot = 0x50) |
| REQ_GET_CS_DISPLAY_CFG | 0x28 | IN | Get 16 bytes: `{max_pages, model_count, reserved[2]}` + the live 12-byte CsDisplayCfg |
| REQ_SET_CS_DISPLAY_PAGE | 0x29 | OUT | Set one display page (wValue=page 0-15, payload=4-byte CsDisplayPage; all-zero clears); apply-live-only, deferred, poll 0x87 (last_slot = 0x50\|page) |
| REQ_GET_CS_DISPLAY_PAGE | 0x2A | IN | Get the live 4-byte CsDisplayPage (wValue=page 0-15) |
| REQ_GET_CS_DISPLAY_STATUS | 0x2B | IN | Get the 8-byte CsDisplayStatus (init state, shown page, overlay/edit flags, live model, cumulative I2C NAK count) |
| REQ_SET_PSYBASS | 0x30 | OUT | Enable/disable psychoacoustic bass (1 byte, 0/1) |
| REQ_GET_PSYBASS | 0x31 | IN | Get psybass enabled state (1 byte) |
| REQ_SET_PSYBASS_CUTOFF | 0x32 | OUT | Set cutoff (4-byte LE IEEE754 float, 30-300 Hz, clamped) |
| REQ_GET_PSYBASS_CUTOFF | 0x33 | IN | Get cutoff (4-byte float) |
| REQ_SET_PSYBASS_HARMONICS | 0x34 | OUT | Set harmonics mix level (4-byte float, -24..+12 dB, clamped) |
| REQ_GET_PSYBASS_HARMONICS | 0x35 | IN | Get harmonics mix level (4-byte float) |
| REQ_SET_PSYBASS_DRIVE | 0x36 | OUT | Set odd-path drive (4-byte float, 0..18 dB, clamped) |
| REQ_GET_PSYBASS_DRIVE | 0x37 | IN | Get odd-path drive (4-byte float) |
| REQ_SET_PSYBASS_CHARACTER | 0x38 | OUT | Set even/odd blend (4-byte float, 0..100, clamped) |
| REQ_GET_PSYBASS_CHARACTER | 0x39 | IN | Get even/odd blend (4-byte float) |
| REQ_SET_PSYBASS_ORIGINAL | 0x3A | OUT | Set original low-band level (4-byte float, -60..0 dB, clamped) |
| REQ_GET_PSYBASS_ORIGINAL | 0x3B | IN | Get original low-band level (4-byte float) |
| REQ_SET_PSYBASS_MASK | 0x3C | OUT | Set output mask (2-byte LE uint16; read live, no recompute) |
| REQ_GET_PSYBASS_MASK | 0x3D | IN | Get output mask (2-byte LE uint16) |
| REQ_SET_TUBE_PARAM | 0x3E | OUT | Set one tube preamp parameter (wValue low byte = index 0-13, payload = 4-byte LE IEEE754 float for every parameter, the bool and enum ones included). Clamped; an out-of-range index, a payload shorter than 4 bytes and a NaN are all no-ops. Every index except 1 (`output_mask`) raises the coefficient recompute |
| REQ_GET_TUBE_PARAM | 0x3F | IN | Get one tube preamp parameter (wValue low byte = index; 4-byte float). An unknown index STALLs, so index 0 doubles as the feature probe |
| REQ_LIMITER | 0x81 | OUT | Set one output-limiter parameter: wValue = (output << 8) \| index (0 enabled, 1 threshold_db -30..0, 2 release_ms 10..1000, 3 link_group 0..4), payload = 4-byte LE float. Output 0xFF sets every output. Clamped; bad output or index, short payload and NaN are no-ops |
| REQ_LIMITER | 0x81 | IN | Same opcode, IN direction: get one parameter as a 4-byte float (wValue as above), or index 0x80 = gain-reduction meter (`NUM_OUTPUT_CHANNELS` x uint16 LE, 0.01 dB, output byte ignored), 0x81 = 4-byte status (engaged, lookahead 32, block 16, num_outputs). A bad output or index STALLs; index 0x81 is the feature probe |
| REQ_SET_EQ_PARAM | 0x42 | OUT | Set EQ band parameters; optional 18-byte payload appends a `uint16` LE Linkwitz-Transform `qp` (`Q*512`) at offsets 16-17 (a 16-byte payload preserves the stored `qp`) |
| REQ_GET_EQ_PARAM | 0x43 | IN | Get one EQ scalar; param codes 0-4 as before (type/freq/Q/gain_db/bypass), param code 5 returns `qp_x512` as a `u32` |
| REQ_SET_PREAMP | 0x44 | OUT | Set preamp gain (legacy: sets all input channels) |
| REQ_GET_PREAMP | 0x45 | IN | Get preamp gain (legacy: returns channel 0) |
| REQ_SET_BYPASS | 0x46 | OUT | Set master EQ bypass |
| REQ_GET_BYPASS | 0x47 | IN | Get master EQ bypass state |
| REQ_SET_DELAY | 0x48 | OUT | Set channel delay |
| REQ_GET_DELAY | 0x49 | IN | Get channel delay |
| REQ_UPMIX_SET_CONFIG | 0x4A | OUT | Set the whole stereo upmixer config (44-byte `UpmixConfigPacket`; RP2350 only, wrong length STALLs; RP2040 STALLs). Mode fields clamped; floats clamped downstream |
| REQ_UPMIX_GET_CONFIG | 0x4B | IN | Get the stereo upmixer config (44-byte `UpmixConfigPacket`; RP2040 returns 44 zero bytes) |
| REQ_UPMIX_SET_PARAM | 0x4C | OUT | Set one upmixer field: `wValue` = `UPMIX_PARAM_*` (0..13; 13 = presence dB), payload = 4-byte LE float (mode/enable rounded to int; RP2350 only, RP2040 STALLs) |
| REQ_UPMIX_GET_PARAM | 0x4D | IN | Get one upmixer field (`wValue` = `UPMIX_PARAM_*`, returns 4-byte float; RP2040 returns 0.0). Unknown param STALLs |
| REQ_UPMIX_GET_STATUS | 0x4E | IN | Get 16-byte `UpmixStatus` (active, parked_reason, corr_q14, balance_q14, center/ls/rs gains; RP2040 returns 16 zero bytes). 0x4F reserved |
| REQ_GET_STATUS | 0x50 | IN | Get all channel peaks + CPU load (see Channel Metering). Scalar wValue 26 = output ring underrun recoveries, 27 = blocks refused by ring admission (both cumulative), 28 = lowest pre-write ring fill in frames since `REQ_RESET_BUFFER_STATS` bit 0 (`0xFFFFFFFF` = none), 29 = worst gap between input deliveries in microseconds since the same reset, 30 = worst gap between audio-carrying USB packets in microseconds since the same reset; 31 = background steps over budget since boot; 26 counts one per dry episode in which a producer owed audio throughout (2026-10-24); on `DSPI_LOOP_PROF` bench builds 64+n = worst time in main-loop section n, 96+n = worst step of background job n, 256+op = worst vendor handler time for opcode op, each since its last read, and 512+n sets bench pacing (each background step does one unit, on one pass in n; returns n) (`loop_prof.h`) |
| REQ_SAVE_PARAMS | 0x51 | OUT | Save all params to flash |
| REQ_SAVE_OUTPUT_CONFIG | 0x52 | IN | Persist live physical IO config and output limiter settings to the directory's device-global blocks (independent mode; was the deprecated REQ_LOAD_PARAMS) |
| REQ_FACTORY_RESET | 0x53 | OUT | Reset to defaults |
| REQ_SET_CHANNEL_GAIN | 0x54 | OUT | Set legacy channel gain |
| REQ_GET_CHANNEL_GAIN | 0x55 | IN | Get legacy channel gain |
| REQ_SET_CHANNEL_MUTE | 0x56 | OUT | Set legacy channel mute |
| REQ_GET_CHANNEL_MUTE | 0x57 | IN | Get legacy channel mute |
| REQ_SET_LOUDNESS | 0x58 | OUT | Enable/disable loudness |
| REQ_GET_LOUDNESS | 0x59 | IN | Get loudness state |
| REQ_SET_LOUDNESS_REF | 0x5A | OUT | Set loudness reference SPL |
| REQ_GET_LOUDNESS_REF | 0x5B | IN | Get loudness reference SPL |
| REQ_SET_LOUDNESS_INTENSITY | 0x5C | OUT | Set loudness intensity |
| REQ_GET_LOUDNESS_INTENSITY | 0x5D | IN | Get loudness intensity |
| REQ_SET_CROSSFEED | 0x5E | OUT | Enable/disable crossfeed |
| REQ_GET_CROSSFEED | 0x5F | IN | Get crossfeed state |
| REQ_SET_CROSSFEED_PRESET | 0x60 | OUT | Set crossfeed preset |
| REQ_GET_CROSSFEED_PRESET | 0x61 | IN | Get crossfeed preset |
| REQ_SET_CROSSFEED_FREQ | 0x62 | OUT | Set custom crossfeed freq |
| REQ_GET_CROSSFEED_FREQ | 0x63 | IN | Get custom crossfeed freq |
| REQ_SET_CROSSFEED_FEED | 0x64 | OUT | Set custom crossfeed level |
| REQ_GET_CROSSFEED_FEED | 0x65 | IN | Get custom crossfeed level |
| REQ_SET_CROSSFEED_ITD | 0x66 | OUT | Set crossfeed ITD |
| REQ_GET_CROSSFEED_ITD | 0x67 | IN | Get crossfeed ITD |
| REQ_SET_ADAT_INPUT_ENABLE | 0x68 | OUT | Enable/disable ADAT input (1 byte, 0/1; RP2350 only). Selectable only when enabled AND a pin is set |
| REQ_GET_ADAT_INPUT_ENABLE | 0x69 | IN | Get ADAT input enable state (1 byte) |
| REQ_SET_ADAT_INPUT_PIN | 0x6A | OUT | Set ADAT RX GPIO (1 byte; 0xFF = reset to default = clear to unset, only while disabled). May equal the ADAT output pin for a zero-hardware loopback self-test |
| REQ_GET_ADAT_INPUT_PIN | 0x6B | IN | Get ADAT RX GPIO (1 byte) |
| REQ_SET_ADAT_INPUT_CLOCK_MODE | 0x6C | OUT | Set ADAT clock mode (1 byte: 0 = master, 1 = slave); deferred apply |
| REQ_GET_ADAT_INPUT_CLOCK_MODE | 0x6D | IN | Get ADAT clock mode (1 byte) |
| REQ_GET_ADAT_INPUT_STATUS | 0x6E | IN | Get 20-byte AdatInputStatusPacket (state, clock_mode, enabled, pin, rate_ok, lock/loss/slip counts, header_err, detected_rate, measured_hz). 0x6F reserved |
| REQ_SET_MATRIX_ROUTE | 0x70 | OUT | Set matrix crosspoint |
| REQ_GET_MATRIX_ROUTE | 0x71 | IN | Get matrix crosspoint |
| REQ_SET_OUTPUT_ENABLE | 0x72 | OUT | Enable/disable output |
| REQ_GET_OUTPUT_ENABLE | 0x73 | IN | Get output enable state |
| REQ_SET_OUTPUT_GAIN | 0x74 | OUT | Set output gain |
| REQ_GET_OUTPUT_GAIN | 0x75 | IN | Get output gain |
| REQ_SET_OUTPUT_MUTE | 0x76 | OUT | Set output mute |
| REQ_GET_OUTPUT_MUTE | 0x77 | IN | Get output mute |
| REQ_SET_OUTPUT_DELAY | 0x78 | OUT | Set output delay |
| REQ_GET_OUTPUT_DELAY | 0x79 | IN | Get output delay |
| REQ_GET_CORE1_MODE | 0x7A | IN | Get Core 1 operating mode |
| REQ_GET_CORE1_CONFLICT | 0x7B | IN | Get Core 1 conflict state |
| REQ_SET_OUTPUT_PIN | 0x7C | OUT | Set output GPIO pin (pin byte 0xFF = reset that output to its platform default) |
| REQ_GET_OUTPUT_PIN | 0x7D | IN | Get output GPIO pin |
| REQ_GET_SERIAL | 0x7E | IN | Get unique board serial |
| REQ_GET_PLATFORM | 0x7F | IN | Get platform ID, fw version (legacy nibbles + full-width minor/patch + beta ordinal), output count; 7 bytes |
| REQ_GET_BUILD_INFO | 0x80 | IN | Get 64-byte build stamp: git describe [0..47] + build date [48..59]; provenance only, never gated on |
| REQ_GET_CLOCK_DIAG | 0x82 | IN | Get packed ClockDiagPacket clock/audio-path snapshot. Built only with `-DDSPI_CLOCK_DIAG=ON`, USB only; zero-length otherwise. See Clock Diagnostics |
| REQ_CLEAR_CLIPS | 0x83 | IN | Read-then-clear clip flags (see Clip Detection) |
| REQ_SET_CS_BINDING | 0x84 | OUT | Set a Control Surfaces binding (wValue=slot 0-15, payload=24-byte CsBinding, required; short payload = INVALID_VALUE); apply-live-only preview, deferred, poll 0x87; persist via REQ_CS_SAVE (see Control Surfaces) |
| REQ_GET_CS_BINDING | 0x85 | IN | Get the live 24-byte CsBinding for a slot (wValue=slot) |
| REQ_GET_CS_CAPS | 0x86 | IN | Get capability tables (wValue=0xFFFF: 40-byte header+type table+max_ir_commands+max_groups/max_macros/max_macro_steps, caps v9; wValue=noun: 12-byte CsNounDesc) |
| REQ_GET_CS_STATUS | 0x87 | IN | Get 41-byte CsStatusPacket (last SET result, dirty flag, active_mask, per-slot status, ir_active_mask, learn state, per-sub-slot IR status) |
| REQ_SET_CS_NAME | 0x8B | OUT | Set a Control Surfaces slot name (wValue=slot 0-15, payload=1-32 bytes; one NUL byte clears); apply-live-only preview (persist via 0x9D), poll 0x87 for result |
| REQ_GET_CS_NAME | 0x8C | IN | Get a Control Surfaces slot name (wValue=slot, returns 32 bytes NUL-terminated, live) |
| REQ_SET_CS_IR_CMD | 0x8D | OUT | Set a Control Surfaces IR remote command (wValue=sub-slot 0-15, payload=16-byte IrCommand); apply-live-only, deferred; poll 0x87 (last_slot = 0x80\|sub) |
| REQ_GET_CS_IR_CMD | 0x8E | IN | Get an IR remote command (wValue=sub-slot, returns 16-byte IrCommand) |
| REQ_CS_IR_LEARN | 0x8F | IN | IR learn control: wValue 1=arm (10 s window), 0=cancel, 2=read result (8 bytes: state, protocol, 0, 0, code_LE32); completion also pushed as notify 0x0A |
| REQ_PRESET_SAVE | 0x90 | IN | Save live state to preset slot (wValue=slot) |
| REQ_PRESET_LOAD | 0x91 | IN | Load preset slot to live state (wValue=slot) |
| REQ_PRESET_DELETE | 0x92 | IN | Delete preset slot (wValue=slot) |
| REQ_PRESET_GET_NAME | 0x93 | IN | Get 32-byte preset name (wValue=slot) |
| REQ_PRESET_SET_NAME | 0x94 | OUT | Set preset name (wValue=slot, payload=32 bytes) |
| REQ_PRESET_GET_DIR | 0x95 | IN | Get directory summary (7 bytes: byte 5 = output_config_mode, byte 6 = master_volume_mode) |
| REQ_PRESET_SET_STARTUP | 0x96 | OUT | Set startup mode + default slot (2 bytes) |
| REQ_PRESET_GET_STARTUP | 0x97 | IN | Get startup config (3 bytes) |
| REQ_SET_OUTPUT_CONFIG_MODE | 0x98 | OUT | Set physical IO persistence mode: 1 = with-preset, 0 = independent (1 byte, was include-pins) |
| REQ_GET_OUTPUT_CONFIG_MODE | 0x99 | IN | Get physical IO persistence mode (1 byte) |
| REQ_PRESET_GET_ACTIVE | 0x9A | IN | Get active preset slot (1 byte, always 0-9) |
| REQ_SET_CHANNEL_NAME | 0x9B | OUT | Set channel name (wValue=channel, payload=1-32 bytes) |
| REQ_GET_CHANNEL_NAME | 0x9C | IN | Get channel name (wValue=channel, returns 32 bytes) |
| REQ_CS_SAVE | 0x9D | IN | Persist the whole live Control Surfaces config (bindings + names + IR commands + groups + macros) in one directory write; deferred, poll 0x87 (last_slot=0xFF); clears the dirty flag |
| REQ_CS_REVERT | 0x9E | IN | Discard the live Control Surfaces preview and re-apply the stored config; deferred, poll 0x87 (last_slot=0xFF); no flash write |
| REQ_GET_ALL_PARAMS | 0xA0 | IN | Get complete DSP state (3664 bytes at V11, multi-packet control transfer) |
| REQ_SET_ALL_PARAMS | 0xA1 | OUT | Set complete DSP state (3664 bytes at V11, multi-packet control transfer) |
| REQ_GET_ALL_PARAMS_CHUNK | 0xA2 | IN | Read WireBulkParams in <= 4 KB chunks (wValue = offset); USB-only, WinUSB 4 KB cap workaround |
| REQ_SET_ALL_PARAMS_CHUNK | 0xA3 | OUT | Write WireBulkParams in sequential chunks (wValue = offset); apply fires on the final byte; USB-only |
| REQ_SIGGEN_SET_CONFIG | 0xA4 | OUT | Stage a test-signal `SiggenConfig` (36 B); validates + stages, restarts if running, never auto-starts (see Test Signal Generator) |
| REQ_SIGGEN_GET_CONFIG | 0xA5 | IN | Get the applied `SiggenConfig` (36 B) |
| REQ_SIGGEN_CONTROL | 0xA6 | IN* | Test-signal action in wValue (`SIGGEN_CTL_*`: 0 stop, 1 start, 2 stop-now); returns 1-byte ack |
| REQ_SIGGEN_GET_STATUS | 0xA7 | IN | Get `SiggenStatus` (16 B: state, signal_type, walk channel, elapsed, cycles, stop reason, sweep freq) |
| REQ_SIGGEN_GET_CAPS | 0xA8 | IN | wValue=0xFFFF -> `SiggenCapsHeader` (8 B); wValue=type index -> `SiggenTypeDesc` (62 B) |
| REQ_GET_BUFFER_STATS | 0xB0 | IN | Get 44-byte buffer fill level statistics packet |
| REQ_RESET_BUFFER_STATS | 0xB1 | IN | Reset watermarks (wValue bit 0), returns 1-byte ack |
| REQ_SET_LEVELLER_ENABLE | 0xB4 | OUT | Enable/disable volume leveller |
| REQ_GET_LEVELLER_ENABLE | 0xB5 | IN | Get volume leveller enabled state |
| REQ_SET_LEVELLER_AMOUNT | 0xB6 | OUT | Set leveller compression amount (0.0–100.0, float) |
| REQ_GET_LEVELLER_AMOUNT | 0xB7 | IN | Get leveller compression amount |
| REQ_SET_LEVELLER_SPEED | 0xB8 | OUT | Set leveller envelope speed (0=Slow, 1=Med, 2=Fast) |
| REQ_GET_LEVELLER_SPEED | 0xB9 | IN | Get leveller envelope speed |
| REQ_SET_LEVELLER_MAX_GAIN | 0xBA | OUT | Set leveller max boost gain (0.0–35.0 dB, float) |
| REQ_GET_LEVELLER_MAX_GAIN | 0xBB | IN | Get leveller max boost gain |
| REQ_SET_LEVELLER_LOOKAHEAD | 0xBC | OUT | Enable/disable leveller 5 ms lookahead |
| REQ_GET_LEVELLER_LOOKAHEAD | 0xBD | IN | Get leveller lookahead state |
| REQ_SET_LEVELLER_GATE | 0xBE | OUT | Set leveller silence gate threshold (-96.0–0.0 dBFS, float) |
| REQ_GET_LEVELLER_GATE | 0xBF | IN | Get leveller silence gate threshold |
| SET_OUTPUT_TYPE | 0xC0 | OUT | Set output slot type (S/PDIF or I2S) |
| GET_OUTPUT_TYPE | 0xC1 | IN | Get output slot type |
| SET_I2S_BCK_PIN | 0xC2 | OUT | Set an I2S BCK pin (LRCLK = BCK + 1). `wValue = (role << 8) | GPIO`: role 0 = master/unified pair (legacy hosts send role 0 implicitly), role 1 = slave pair (`i2s_bck_pin_slave`, meaningful in SPLIT clock-pin mode; storable any time). Rejects with OUTPUT_ACTIVE while I2S output slots run on the pair being moved, PIN_IN_USE on overlap/collision, INVALID_OUTPUT for role > 1. The two pairs are kept mutually distinct. GPIO 0xFF = reset the addressed role's pair to its default. |
| GET_I2S_BCK_PIN | 0xC3 | IN | Get an I2S BCK pin. `wValue = role` (0 = master/unified pair, 1 = slave pair; invalid role returns 0) |
| SET_MCK_ENABLE | 0xC4 | OUT | Set MCK enable. Returns `PIN_CONFIG_*`: enabling returns PIN_IN_USE (MCK stays off) if another function claimed `i2s_mck_pin` while MCK was disabled. |
| GET_MCK_ENABLE | 0xC5 | IN | Get MCK enable |
| SET_MCK_PIN | 0xC6 | OUT | Set MCK pin (0xFF = reset to platform default) |
| GET_MCK_PIN | 0xC7 | IN | Get MCK pin |
| SET_MCK_MULTIPLIER | 0xC8 | OUT | Set MCK multiplier (0=128x, 1=256x) |
| GET_MCK_MULTIPLIER | 0xC9 | IN | Get MCK multiplier |
| REQ_SET_ADAT_ENABLE | 0xCA | OUT | Enable/disable ADAT bulk output (wValue 0/1; RP2350 only, RP2040 returns INVALID_OUTPUT) |
| REQ_GET_ADAT_ENABLE | 0xCB | IN | Get configured ADAT enable |
| REQ_SET_ADAT_PIN | 0xCC | OUT | Set ADAT data GPIO (wValue = pin; 0xFF = reset to default 12; 0 = GPIO 0) |
| REQ_GET_ADAT_PIN | 0xCD | IN | Get ADAT data GPIO |
| REQ_GET_ADAT_STATUS | 0xCE | IN | Get 8-byte AdatStatus (enabled, active, pin, rate_ok, resync/slip counters) |
| REQ_SET_PREAMP_CH | 0xD0 | OUT | Set per-channel preamp gain (wValue=channel) |
| REQ_GET_PREAMP_CH | 0xD1 | IN | Get per-channel preamp gain (wValue=channel) |
| REQ_SET_MASTER_VOLUME | 0xD2 | OUT | Set master volume (-128 to 0 dB, -128=mute) |
| REQ_GET_MASTER_VOLUME | 0xD3 | IN | Get master volume |
| REQ_SET_MASTER_VOLUME_MODE | 0xD4 | OUT | Set master volume persistence mode (0=independent, 1=with preset) |
| REQ_GET_MASTER_VOLUME_MODE | 0xD5 | IN | Get master volume persistence mode |
| REQ_SAVE_MASTER_VOLUME | 0xD6 | IN | Persist live master volume to directory's independent field (mode 0 source) |
| REQ_GET_SAVED_MASTER_VOLUME | 0xD7 | IN | Get the directory's independent master volume |
| REQ_SET_USER_VOLUME | 0xDA | OUT | Set user-perceived volume (float dB, [-CENTER_VOLUME_INDEX, 0]); shares `audio_state.volume` with UAC1 host slider, always applies regardless of input source so loudness compensation tracks the change |
| REQ_GET_USER_VOLUME | 0xDB | IN | Get user-perceived volume (float dB) |
| REQ_SET_USER_MUTE | 0xDC | OUT | Set vendor-channel `user_mute` flag (1 byte 0/1); always honored regardless of input source. Distinct from `audio_state.mute` (UAC1) which is USB-gated; pipeline ORs them. |
| REQ_GET_USER_MUTE | 0xDD | IN | Get vendor-channel `user_mute` (UAC1 mute is read via UAC1 GET_CUR) |
| REQ_SET_LEVELLER_MASKS | 0xDE | OUT | Set leveller channel masks (2 bytes: [detector_mask, apply_mask], bit k = input channel k); sets update-pending, no state reset (glitch-free) |
| REQ_GET_LEVELLER_MASKS | 0xDF | IN | Get leveller channel masks (2 bytes: [detector_mask, apply_mask]) |
| REQ_SET_I2S_CLOCK_MODE | 0x88 | OUT | Set I2S clock mode (uint8_t: 0=master, 1=slave); deferred apply, only meaningful while input source is I2S |
| REQ_GET_I2S_CLOCK_MODE | 0x89 | IN | Get live I2S clock mode (pending change not reflected until applied) |
| REQ_GET_I2S_SLAVE_STATUS | 0x8A | IN | Get 16-byte I2sSlaveStatusPacket (state, clock_mode, lock/loss/slip counts, detected + measured rate) |
| REQ_SET_INPUT_SOURCE | 0xE0 | OUT | Set active input source (0=USB, 1=SPDIF, 2=I2S, 3=ADAT, 4=SPDIF2, 5=SPDIF3, 6=SPDIF4). Optional SPDIF inputs rejected unless enabled; ADAT rejected unless enabled with a pin set (RP2350) |
| REQ_GET_INPUT_SOURCE | 0xE1 | IN | Get active input source |
| REQ_GET_SPDIF_RX_STATUS | 0xE2 | IN | Get SPDIF RX status (16-byte SpdifRxStatusPacket) |
| REQ_GET_SPDIF_RX_CH_STATUS | 0xE3 | IN | Get IEC 60958 channel status (24 bytes) |
| REQ_SET_SPDIF_RX_PIN | 0xE4 | IN* | Set a SPDIF input's RX GPIO (wValue = (index<<8)\|pin, index 0..3; old hosts sending just a pin target index 0; pin 0xFF = reset that input to its default). Enabled inputs conflict-check the new pin; a disabled optional input stores it as a preference. Returns status byte |
| REQ_GET_SPDIF_RX_PIN | 0xE5 | IN | Get a SPDIF input's RX GPIO (wValue = index 0..3) |
| REQ_SET_LG_SOUND_SYNC_ENABLE | 0xE6 | OUT | Set LG Sound Sync enable flag (per-preset; live until REQ_SAVE_PRESET) |
| REQ_GET_LG_SOUND_SYNC_ENABLE | 0xE7 | IN | Get LG Sound Sync enable flag |
| REQ_GET_LG_SOUND_SYNC_STATUS | 0xE8 | IN | Get 16-byte LgSoundSyncStatus (enabled/present/volume/muted + reserved) |
| REQ_SET_SPDIF_INPUT_ENABLE | 0xE9 | IN* | Enable/disable an optional SPDIF input (wValue = (index<<8)\|enable, index 1..3). Enabling validates the configured pin (PIN_IN_USE on conflict); disabling the live/pending source is rejected. RAM-only; persist via REQ_PRESET_SAVE. Returns PIN_CONFIG_* status byte |
| REQ_SET_INPUT_RATE | 0xED | OUT | Set I2S input sample rate (uint32_t Hz: 44100/48000/96000; applied live when I2S input active in master clock mode; stored only in slave mode, where the rate is auto-detected) |
| REQ_GET_INPUT_RATE | 0xEE | IN | Returns 8 bytes: 2x uint32_t {current pipeline Hz, selected I2S input Hz} |
| REQ_GET_SPDIF_INPUT_CONFIG | 0xEF | IN | Get 6 bytes: input count (4), enable mask over all inputs (bit 0 = input 1, always set), then GPIOs for inputs 1..4. Lets a host build its source list data-driven |
| REQ_SET_I2S_RX_PIN | 0xF1 | IN* | Set I2S RX data GPIO pin (wValue=(pair<<8)\|pin, returns status; pin 0xFF = reset that pair to its default). Checked against active pairs only; an inactive pair's parked pin does not block |
| REQ_GET_I2S_RX_PIN | 0xF2 | IN | Get I2S RX data GPIO pin |
| REQ_SET_UART_CONFIG | 0xF5 | OUT | Configure UART control interface (8-byte `UartCtrlConfig`; **USB only**, refused with BLOCKED over UART/I2C; deferred apply, persist on success; returns `PIN_CONFIG_*`) |
| REQ_GET_UART_CONFIG | 0xF6 | IN | Get persisted `UartCtrlConfig` (8 bytes) |
| REQ_SET_I2C_CONFIG | 0xF7 | OUT | Configure I2C target control interface (8-byte `I2cCtrlConfig`; **USB only**, refused with BLOCKED over UART/I2C; deferred apply, persist on success; returns `PIN_CONFIG_*`) |
| REQ_GET_I2C_CONFIG | 0xF8 | IN | Get persisted `I2cCtrlConfig` (8 bytes) |
| REQ_GET_CTRL_IFACE_STATUS | 0xF9 | IN | Get `CtrlIfaceStatus` (8 bytes: uart/i2c last_status + live flags, proto_version=1) |
| REQ_SET_LOUDNESS_MASK | 0xFA | OUT | Set `loudness_output_mask` (2-byte little-endian; bit k = compensate output k, default 0xFFFF). Selects which output channels get per-output loudness |
| REQ_GET_LOUDNESS_MASK | 0xFB | IN | Get `loudness_output_mask` (returns 2 bytes) |
| REQ_SET_CROSSFEED_OUTPUTS | 0xFC | OUT | Set `output_pair_mask` (1 byte; bit p = crossfeed on output pair p, default 0x01 = pair 1 only). Clamped to valid pair bits; read live each packet (no recompute) |
| REQ_GET_CROSSFEED_OUTPUTS | 0xFD | IN | Get `output_pair_mask` (returns 1 byte) |
| REQ_SET_I2S_CLOCK_PIN_MODE | 0xFE | IN* | Set I2S clock-pin mode (wValue = 0 unified / 1 split; returns `PIN_CONFIG_*`). Entering SPLIT always validates the slave pair (distinctness + occupancy), even as a dormant store: PIN_IN_USE on conflict. Rejects with OUTPUT_ACTIVE when slave clocking is (or is pending) live with I2S output slots, INVALID_PARAM for wValue > 1. A live input-only change arms a deferred input restart onto the new effective pair. |
| REQ_GET_I2S_CLOCK_PIN_MODE | 0xFF | IN | Get live I2S clock-pin mode (returns uint8_t: 0 = unified, 1 = split) |

### Bulk Parameter Transfer
*Last updated: 2026-09-22 (wire V32: output limiter section appended at 6028, total 6136; 2026-09-20: wire V31 tube section revised in place: `xfmr_damping` / `xfmr_res_hz` plus a reserved float, still 48 bytes and total 6028; 2026-09-18 wire V31: tube preamp section appended, total 6028; 2026-09-04 wire V30: subharm section grows 16 to 36 bytes, total 5980; 2026-08-02 wire V28: input-config `spdif_rx_pin_ext` grows to 3 entries for SPDIF input 4; section and total size unchanged)*

Transfers the complete DSP state in a single USB control transfer (3664 bytes at V11/V12), replacing dozens of individual vendor requests.

**Wire format:** `WireBulkParams` (`bulk_params.h`, `WIRE_FORMAT_VERSION` 32, total 6136 bytes); packed struct with header, global params, crossfeed, legacy channel gains, delays, matrix crosspoints, matrix outputs, pin config, EQ bands, channel names, I2S config, leveller config, preamp config (`WirePreampConfig`, 16 bytes), master volume config (`WireMasterVolume`, 16 bytes), input source config (`WireInputConfig`, 16 bytes), LG Sound Sync (`WireLgSoundSync`, 16 bytes), user volume/mute (`WireUserVolume`, 16 bytes), DAC hardware mute (`WireDacHwMute`, 16 bytes, V10+), and **crossover bands** (`WireCrossoverConfig`, 704 bytes = 11 × 4 × `WireBandParams`, V11+). V12 claims two reserved bytes inside `WireInputConfig` for `i2s_rx_pin` and `i2s_input_rate` (enum 0=44100, 1=48000, 2=96000); V12 payloads are byte-identical in size to V11. All arrays sized at platform maximums (RP2350: 11 channels, 9 outputs, 5 pins, 12 PEQ bands, 4 crossover bands per channel). Unused entries zero-padded; for crossover, master rows (channel < `CH_OUT_1`) are zeroed on collect and skipped on apply. **V20** repurposes the `WireCrossfeedParams` reserved byte (offset 3) as `output_pair_mask` (bit p = crossfeed on output pair p); struct sizes are unchanged. **V22** carries the Linkwitz-Transform target `Q` in the EQ `WireBandParams.reserved[2]` bytes (`uint16` LE, `Q*512`; zero for non-LT types), so struct sizes stay unchanged. (V21 claimed one `WireInputConfig` reserved byte for the I2S clock master/slave mode, also size-neutral.) **V23** tail-appends the 24-byte `WirePsybassParams` (psychoacoustic bass: `enabled` + `output_mask` + five floats), bringing the total to 5900 bytes. **V24** claims three `WireInputConfig` reserved bytes for the ADAT input (`adat_input_pin`, `adat_input_enabled_p1`, `adat_clock_mode_p1`, each 0 = absent/keep-live); struct sizes and the 5900-byte total are unchanged. **V25** tail-appends the 44-byte `WireUpmixParams` (RP2350 stereo upmixer: enabled + centre/surround modes + reserved + ten floats; layout-identical to `UpmixConfigPacket`), bringing the total to 5944 bytes; the section is zeroed on collect and ignored on apply on RP2040. **V28** widens `WireInputConfig.spdif_rx_pin_ext` from 2 to 3 entries (SPDIF input 4), consuming that section's last reserved byte and shifting `spdif_rx_enabled_ext_p1`, `i2s_clock_mode` and the ADAT input fields down one byte; the section stays 16 bytes and the 5944-byte total and every later section offset are unchanged. The input-config section now has no reserved bytes left. **V29** tail-appends the 16-byte `WireSubharmParams` (subharmonic synthesizer: `enabled` + `reserved0` + `output_mask` + `low_db`/`high_db`/`boost_db`) at offset 5944, bringing the total to 5960 bytes. **V30** grows that section to 36 bytes by tail-appending `top_db`, `select_depth`, `select_hold_ms`, `ceiling_db`, `select_mode`, `link_pairs` and two reserved bytes, bringing the total to 5980 bytes; the section offset stays 5944 and `solo` is deliberately absent (runtime-only). **V31** tail-appends the 48-byte `WireTubeParams` (tube preamp: `enabled` + `tube_type` + `rectifier` + `xfmr_enabled` + `output_mask` + two reserved bytes + nine floats + one reserved float) at offset 5980, bringing the total to 6028 bytes. The two output-stage floats are `xfmr_damping` and `xfmr_res_hz`; the layout was revised in place before release, so there is only one V31. Apply clamps `tube_type` and `rectifier` and copies the character fields verbatim, never running the tube-type row lookup, which would overwrite a saved Custom voicing. **V32** tail-appends the 108-byte `WireLimiterParams` (output limiter: nine 12-byte per-output records of `enabled`, `link_group`, two reserved bytes, `threshold_db`, `release_ms`) at offset 6028, bringing the total to 6136 bytes.

**Per-version size anchors** live in `bulk_params.h` (`WIRE_BULK_PARAMS_V{N}_SIZE`, N=2..12). Each legacy-section apply gate inside `bulk_params_apply()` compares `payload_length` against its own version's anchor, NOT against `sizeof(WireBulkParams)`. Without this discipline, growing the struct would silently lock older payloads out of the very tail sections they own (e.g. a V10 payload would stop applying its DAC-mute section the moment V11 was added). V<11 payloads leave crossover state untouched on apply; V<12 payloads leave the I2S input pin/rate untouched.

**Transport:** Multi-packet USB EP0 control transfers using `usb_stream_transfer` from pico-extras. Packets are 64 bytes. No modifications to `usb_device.c` required — uses only public API (`usb_stream_setup_transfer`, `usb_start_transfer`, `usb_start_empty_transfer`).

**GET (0xA0):** `bulk_params_collect()` snapshots live state into `bulk_param_buf`, then streams it out in 64-byte packets via `usb_stream_transfer`. ZLP appended if total length is a multiple of 64.

**SET (0xA1):** Incoming data accumulated into `bulk_param_buf` via `usb_stream_transfer`. On completion, `bulk_params_pending` flag is set (after status-phase ACK). Main loop processes deferred: snapshots `output_types[]`, waits for Core 1 idle, mutes audio (256 samples), calls `bulk_params_apply()` with pin application gated on `output_config_mode` (applied only in with-preset mode; independent mode leaves device-global IO to `REQ_SAVE_OUTPUT_CONFIG`), recalculates all filters and delays, transitions Core 1 mode to match the new output enable state, then diffs the new `output_types[]` against the snapshot. If any slot's type changed, dispatches `process_type_switches()` to reconfigure SPDIF/I2S hardware (mirrors the `preset_load_pending` pattern); otherwise calls `complete_pipeline_reset()` to resync output streams (or `reset_usb_feedback_loop()` when SPDIF input is active, to avoid disrupting the prefill handshake).

**Buffer:** 4 KB aligned static buffer in `usb_audio.c`, shared between GET and SET. Platform validation rejects mismatched `platform_id` or `num_channels`.

### Buffer Statistics
*Last updated: 2026-07-23 (consumer pool replaced by the slot ring; stats via a 48-frames-per-bucket mapping)*

Real-time buffer fill level monitoring for the SPDIF slot rings (DMA-side) and PDM buffers, accessible via USB vendor commands. Enables host applications to diagnose audio glitches, near-miss underruns, and pipeline health. There is no producer-side buffer to report: the pipeline renders into per-slot staging and copies it into the ring in the same call.

**Ring-to-bucket mapping (2026-07-23).** The wire format and thresholds are unchanged: each slot's ring fill is mapped to legacy "buffer" buckets of 48 frames (a ring fill of N frames reports N/48 buckets, capped at `SPDIF_CONSUMER_BUFFER_COUNT` = 16), so `REQ_GET_BUFFER_STATS` and its fill percentages keep byte- and semantics-compatibility with the old consumer-pool counts. Since 2026-10-16 the prefill thresholds compare frames against `OUTPUT_RING_TARGET_FILL_FRAMES` instead (see "Buffer Configuration").

**Wire format:** `BufferStatsPacket` (44 bytes, fits in a single 64-byte USB control transfer). Contains per-instance SPDIF consumer stats (`SpdifBufferStats` x4, 8 bytes each), PDM stats (`PdmBufferStats`, 8 bytes), instance count, flags (PDM active, audio streaming), and a monotonic sequence counter.

**SPDIF stats per instance:** consumer free/prepared/playing counts with fill percentage and min/max watermarks (DMA-side pool).

**PDM stats:** DMA circular buffer fill percentage and software ring buffer fill percentage, each with min/max watermarks.

**Fill percentage formulas:**
- SPDIF consumer: `(prepared + playing) * 100 / SPDIF_CONSUMER_BUFFER_COUNT` — healthy: 25-75%
- PDM DMA: `((write_idx - read_idx) & (PDM_DMA_BUFFER_SIZE-1)) * 100 / PDM_DMA_BUFFER_SIZE` — healthy: ~12.5%
- PDM ring: `((head - tail) & 0xFF) * 100 / RING_SIZE` — healthy: 0-10%

**Producer fill formula:** `(capacity - free) * 100 / capacity` — measures in-flight + prepared buffers, since `prepared` alone is always near zero (DMA IRQ drains it on-demand via the connection).

**Watermark tracking:** Consumer watermarks updated once per USB audio packet (~1ms) in `process_audio_packet()`. Overhead ~1-2us (ring pointer reads; consumer pool list traversals before the 2026-10-11 merge). Reset via `REQ_RESET_BUFFER_STATS` (0xB1, wValue bit 0).

**Implementation:** `audio_buffer_list_count()` inline in `pico/audio.h` for read-only list traversal. `pdm_stats_write_idx` volatile in `pdm_generator.c` exposes Core 1 write position to Core 0 (atomic on ARM). Helper functions in `usb_audio.c`: `count_pool_free()`, `count_pool_prepared()`, `update_buffer_watermarks()`, `reset_buffer_watermarks()`.

**BSS impact:** ~18 bytes total (watermark arrays + sequence counter + pdm_stats_write_idx).

---

## I2S Output Support
*Last updated: 2026-04-09*

### Overview

Each output slot can be independently configured as S/PDIF or I2S at runtime via vendor commands. A new `pico_audio_i2s_multi` library mirrors the proven `pico_audio_spdif_multi` patterns. The S/PDIF library is completely unchanged.

### Architecture

- **PIO0:** S/PDIF and I2S programs coexist in the block's 32-slot instruction memory, and since 2026-10-11 the budget is tight: the default S/PDIF PIO biphase encoder (16) plus clkout (8) plus dataout (8) fills all 32 slots in internal-clock mode, and under an external I2S master the S/PDIF side drops to its 4-instruction NRZI program so the 26-instruction extclk program fits (see "PIO Program" under SPDIF Output System). Each library releases its programs when its last instance on the block is torn down, so the lazy loads always find room. Each SM's side-set pins are independent. S/PDIF side-set is the data pin, I2S side-set is BCK/LRCLK.
- **MCK:** Generated by hardware **CLK_GPOUTn** (clock peripheral output), not PIO. No state machine consumed — see "Master Clock (CLK_GPOUTn)" below. **PIO1 SM1 is now free** (previously the MCK toggle program); reserved for future use.
- **OutputSlot abstraction** in `usb_audio.c` manages per-slot type, holding either a SPDIF or I2S instance.
- **DMA IRQ:** none. Both output types play from free-running rings (see "IRQ Handling"); S/PDIF RX alone uses DMA_IRQ_1.
- **Slot staging is format-identical** for both output types (interleaved S24 in int32, stride 8). `audio_i2s_ring_write_s32()` left-shifts samples by 8 for MSB-first I2S framing, then ORs `I2S_PAD_PATTERN` (0x01) into the unused bottom 8 padding bits. The 24-bit audio at [31:8] stays bit-perfect, but the 32-bit wire word is never zero. This defeats DAC zero-detect mute on chips configured for 32-bit input (PCM5102, ES9018/9038, CS43198, AKM in 32-bit mode), eliminating the auto-mute click on quiet content and stream gaps. 24-bit-mode DAC configs ignore the padding bits and are unaffected. The ring reset, erase-ahead silence and silence pump stamp the same pattern, so the DAC sees non-zero frames through underruns and on first stream start. *Last updated: 2026-10-12*
- **No audio callback changes.** Core 1 remains output-type-agnostic.
- **Pipeline reset API** (`main.c`): two-phase `prepare_pipeline_reset()` / `complete_pipeline_reset()` brackets any disruptive output work. `complete_pipeline_reset()` is structured to keep the IRQ-disabled critical section tiny; only the synchronized start (`start_output_slots_in_sync()`, single combined SM mask for both output types since 2026-07-21; see "Synchronized Start") needs atomicity (so all output slots start their PIO SMs on the same clock cycle, preserving CLAUDE.md's slot-alignment invariant). Per-slot teardown (`teardown_output_slot()` helper) runs with main interrupts ENABLED: the per-instance `enabled` flag is set false first (the shared DMA IRQ handler skips disabled instances), the channel's DMA IRQ is masked, and the pool/abort operations only touch this instance's state. The USB audio class ISR can continue to drain packets into the SPSC `audio_ring` throughout the teardown, eliminating the prior ~1 ms USB starvation that compounded I2S DAC pops during input-source switches. Core 1 is held idle by `prepare_pipeline_reset()` (spin-wait for `work_done`) and `preset_loading=true` (blocks new dispatch). `drain_and_disable_outputs()` shares the same `teardown_output_slot()` helper. *Last updated: 2026-05-12*
- **Boot-time I2S restoration:** `core0_init()` inspects `output_types[]` after `preset_boot_load()` and converts preset-saved I2S slots from the default SPDIF instances created by `usb_sound_card_init()`. For each I2S slot: disables SPDIF, unclaims the PIO SM, calls `audio_i2s_setup()` (which programs the divider for `output_rate_hz`), and enables the I2S instance. MCK is started if any I2S slot exists and `i2s_mck_enabled` is set.
- **MCK pin migration on preset apply:** Preset and bulk-params apply paths validate the loaded `i2s_mck_pin` against `GPIO_TO_GPOUT_CLOCK_HANDLE()`. If the stored pin has no GPOUTn mapping on the current platform (typical case: an RP2040 board loading a preset saved with `mck_pin = 13`, which is GPOUTn-capable on RP2350 only), the pin falls back to `PICO_I2S_MCK_PIN` (platform default) and `i2s_mck_enabled` is forced to `false`. Defaults: GPIO 13 on RP2350, GPIO 21 on RP2040 (the only board-friendly GPOUTn pin on RP2040 — GPIOs 23–25 are also GPOUTn-capable but reserved for control / SMPS / LED).
- **MCK enable order:** `process_type_switches()` writes the MCK clkdiv via `audio_i2s_mck_update_frequency()` *before* calling `audio_i2s_mck_set_enabled(true)`, so the CLK_GPOUTn block starts at the correct frequency rather than briefly running at whatever DIV value the block previously held (which would cause a transient PLL-relock chirp on connected DACs). Matches the `REQ_SET_MCK_ENABLE` vendor command order.
- **SPDIF RX is suspended across `process_type_switches()`:** its decode-timeout alarms (timer IRQ) could fire mid-transition and access PIO/DMA state being mutated. The function snapshots RX state at entry, calls `spdif_input_stop()` if it was running, and restarts it after `complete_pipeline_reset()` finishes — guarded by `active_input_source == INPUT_SOURCE_SPDIF && !input_source_change_pending` so a deferred input-source switch isn't pre-empted. Callers that have already stopped RX (e.g. `preset_load_pending` across the flash blackout) see no double-stop because `state == INACTIVE` triggers the no-op path; those callers are responsible for their own restart, which `preset_load_pending` now does once at the end of its block rather than before `process_type_switches`.

*Last updated: 2026-05-12*

### Master Clock (CLK_GPOUTn)
*Last updated: 2026-05-09*

- **No PIO state machine consumed.** MCK is generated by one of the four hardware **CLK_GPOUTn** clock outputs (`clk_gpout0..3`). The previous 2-instruction PIO toggle program (`audio_mck.pio`) is gone; PIO1 SM1 is free for future use.
- **MCK divider** (2026-10-11): `audio_i2s_mck_update_frequency()` takes the CLK_GPOUT divider as the shared base × (256 / multiplier) (see "Shared output divider"). It previously truncated sys × 256 / (Fs × multiplier) on its own, which put MCK one LSB (about 144 ppm) off BCK at 44.1 kHz on the nominal path.
- **Library API** (`audio_i2s_multi.c`, MCK section): `audio_i2s_mck_setup(pin)` records the pin only, no hardware effect; `audio_i2s_mck_set_enabled(bool)` enable routes pad mux + loads divider via `clock_gpio_init_int_frac8(pin, AUXSRC=clk_sys, int, frac8)`, disable disconnects pad mux (generator continues running internally; no public SDK API stops it); `audio_i2s_mck_update_frequency(Fs, mult)` recomputes the 24.8 divider, hot-loads if running; `audio_i2s_mck_change_pin(pin)` is pure book-keeping (asserts `!mck_running`); `audio_i2s_mck_set_divider(div_24_8)` is a raw divider write (formerly the SPDIF clock servo's MCK trim; no longer called since input tracking moved to the soft VCXO, which pulls sys_clk so MCK follows automatically).
- **Pin mapping** (`GPIO_TO_GPOUT_CLOCK_HANDLE` SDK macro): RP2040 → GPIO 21 maps to clk_gpout0 (only DSPi-friendly choice; 23–25 are also GPOUTn-capable but board-reserved). RP2350 → GPIO 13 (clk_gpout0, default), 15 (clk_gpout1, conflicts with LRCLK when I2S is active), 21 (clk_gpout0). The `REQ_SET_MCK_PIN` handler rejects non-GPOUTn pins via `GPIO_TO_GPOUT_CLOCK_HANDLE(pin, clk_sys) == clk_sys`.
- **Default pin** (config.h `PICO_I2S_MCK_PIN`): platform-conditional — 13 on RP2350, 21 on RP2040.
- **96 kHz × 256× clamp removed.** Previous PIO toggle had a 6.25 fractional divider in that combo (silently force-clamped to 128×). GPOUTn gives 12.5 there — still fractional but stable on real hardware. All other Fs × multiplier combinations are integer dividers (see Clock Math table below).
- **Migration:** Existing presets / bulk-params payloads with `mck_pin = 13` loaded on RP2040 fall back to `PICO_I2S_MCK_PIN` (GPIO 21) and force `i2s_mck_enabled = false` (see flash_storage.c apply path + bulk_params.c apply path). No `SLOT_DATA_VERSION` / `WIRE_FORMAT_VERSION` bump required; this is a value-only migration.

### Clock Math at 307.2 MHz

| Signal | Fs    | Frequency  | Divider (24.8) | Jitter      |
|--------|-------|------------|----------------|-------------|
| I2S BCK (Fs×64) | 48 kHz | 3.072 MHz  | 50.0  (PIO)    | Zero        |
| MCK 128×        | 48 kHz | 6.144 MHz  | 50.0  (GPOUTn) | Zero        |
| MCK 256×        | 48 kHz | 12.288 MHz | 25.0  (GPOUTn) | Zero        |
| MCK 128×        | 96 kHz | 12.288 MHz | 25.0  (GPOUTn) | Zero        |
| MCK 256×        | 96 kHz | 24.576 MHz | 12.5  (GPOUTn) | Fractional  |

MCK is driven directly by **CLK_GPOUTn** (hardware clock peripheral output) — `clock_gpio_init_int_frac8()` configures the 24.8 divider against `clk_sys` (AUXSRC_VALUE_CLK_SYS). The previous PIO-toggle implementation needed a `÷2` factor in the denominator (PIO clk = 2 × MCK), which halved divider precision and made every 256× combination fractional; with GPOUTn only 96 kHz × 256× remains fractional. The 96 kHz × 256× clamp that used to silently force 128× has been removed.

### Vendor Commands (0xC0–0xC9)

| Code | Command | Direction |
|------|---------|-----------|
| 0xC0 | SET_OUTPUT_TYPE | SET |
| 0xC1 | GET_OUTPUT_TYPE | GET |
| 0xC2 | SET_I2S_BCK_PIN | SET |
| 0xC3 | GET_I2S_BCK_PIN | GET |
| 0xC4 | SET_MCK_ENABLE | SET |
| 0xC5 | GET_MCK_ENABLE | GET |
| 0xC6 | SET_MCK_PIN | SET |
| 0xC7 | GET_MCK_PIN | GET |
| 0xC8 | SET_MCK_MULTIPLIER | SET |
| 0xC9 | GET_MCK_MULTIPLIER | GET |

### Persistence

- `SLOT_DATA_VERSION` = 9: adds `output_types[4]`, `i2s_bck_pin`, `i2s_mck_pin`, `i2s_mck_enabled`, `i2s_mck_multiplier` (8 bytes)
- `SLOT_DATA_VERSION` = 10: adds leveller fields (16 bytes)
- `SLOT_DATA_VERSION` = 11: changes `i2s_mck_multiplier` encoding from raw uint8_t (128 = 128x, 0 = 256x) to enum-style (0 = 128x, 1 = 256x); internal storage is `uint16_t`
- `SLOT_DATA_VERSION` = 12: adds `preamp_db_per_ch[NUM_INPUT_CHANNELS]` and `master_volume_db`; legacy `preamp_db` still populated for backward compat
- `SLOT_DATA_VERSION` = 13: adds `input_source` + `spdif_rx_pin` (consuming V12 padding bytes; size unchanged)
- `SLOT_DATA_VERSION` = 14: adds `lg_sound_sync_enabled` (uint8_t) + 3 bytes trailing padding for `WireLgSoundSync` future fields
- `SLOT_DATA_VERSION` = 15: adds `user_vol_index` (uint8_t, range [0, CENTER_VOLUME_INDEX]) consuming the LAST V14 padding byte. Pre-V15 slots leave user volume UNTOUCHED on load (asymmetric vs master volume's "fall back to directory" behavior — there is no directory-level fallback for user volume; the user wasn't expecting that preset to set their listening level when they originally saved it). Stored as vol_index rather than float dB because the audio path quantizes to integer dB anyway (`apply_vol_index_to_audio` truncates the 8-bit fractional part of `audio_state.volume`), so single-byte storage is lossless for the actual audio behavior. Restore funnels through `update_user_volume()` so vol_mul + loudness coefficient pointer + LG cache invalidation + v2 notify all happen via the single helper. **THIS IS THE LAST AVAILABLE PADDING BYTE** — future preset additions will need either struct growth (with explicit migration of pre-V15 slots) or directory-level storage in the master-volume "independent" pattern.
- `WIRE_FORMAT_VERSION` = 3: adds `WireI2SConfig` (16 bytes) to `WireBulkParams`
- `WIRE_FORMAT_VERSION` = 4: adds `WireLevellerConfig` (16 bytes) to `WireBulkParams` (total 2864 bytes)
- `WIRE_FORMAT_VERSION` = 5: changes `mck_multiplier` wire encoding in `WireI2SConfig` from raw value to enum-style (0 = 128x, 1 = 256x)
- `WIRE_FORMAT_VERSION` = 6: adds `WirePreampConfig` (16 bytes) and `WireMasterVolume` (16 bytes) to `WireBulkParams`
- `WIRE_FORMAT_VERSION` = 7: adds `WireInputConfig` (16 bytes) — input source + SPDIF RX pin
- `WIRE_FORMAT_VERSION` = 8: adds `WireLgSoundSync` (16 bytes) — LG Sound Sync per-preset gate + runtime status
- `WIRE_FORMAT_VERSION` = 9: adds `WireUserVolume` (16 bytes) — vendor-channel user volume + mute mirror
- `WIRE_FORMAT_VERSION` = 10: adds `WireDacHwMute` (16 bytes) — DAC hardware mute pin config
- `WIRE_FORMAT_VERSION` = 11: adds `WireCrossoverConfig` (704 bytes = 11 × 4 × WireBandParams) — per-channel crossover bands. Bulk total: 3664 bytes. **Per-version size anchors live in `bulk_params.h`** (`WIRE_BULK_PARAMS_V{N}_SIZE`) and each legacy section gate in `bulk_params_apply()` compares against its own version's anchor, NOT `sizeof(WireBulkParams)` — without this discipline, growing the struct silently locks older payloads out of their own tail sections.
- `SLOT_DATA_VERSION` = 16: appends `xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS]` to `PresetSlot` (struct grew — first growth since V12). **CRC migration:** `slot_data_size_for_version()` uses explicit per-version `case` labels so the validator picks the right byte range. V12-V15 share one size; V16 adds the crossover tail. `migrate_legacy()` produces a real V16 slot (sets `version = SLOT_DATA_VERSION`, CRCs over V16 size) — otherwise migrated slots would fail the new validator on next reboot. **Field-default discipline:** because the migrated slot is V16-tagged, every `slot->version >= N` gate in `apply_slot_to_live()` fires and reads the slot's bytes directly, so migrate must populate every V8–V16 field with the value the V<N default branch would have produced — including `user_vol_index = CENTER_VOLUME_INDEX` (NOT zero — zero maps to -CENTER dB which would silently mute migrated devices), default channel names via `get_default_channel_name()`, I2S pins at compile-time defaults, leveller `LEVELLER_DEFAULT_*` values, and crossover FLAT defaults with `band = XOVER_BAND_BASE+i`.
- Backward compatible: V<9 slots default to all-S/PDIF; V9-V10 slots use old MCK encoding; V<12 slots use single preamp value for all channels, default master volume 0 dB; V<11 bulk payloads leave crossover state untouched on apply; V<16 preset slots apply crossover defaults on load; older wire payloads accepted without new fields

### BSS Impact

| Platform | Delta |
|----------|-------|
| RP2040 | +292 bytes |
| RP2350 | +528 bytes |

Full specification: `Documentation/Features/i2s_output_spec.md`

---

## ADAT Bulk Output
*Last updated: 2026-10-23 (576-frame ring for the 512-frame slot rings). Previously 2026-10-11 (output rings: synchronous skip mirror, keep-alive, measured resync anchor, 960-frame ring; lead bound `ADAT_MAX_LEAD_FRAMES` on the resync and skip mirror). Previously, 2026-07-13 (LUT-driven NRZI encoder; single float->S24 conversion point shared with the slots)*

RP2350 only. Streams all 8 main output channels (post-matrix, post per-output
EQ/gain/delay/mute, the same finalized samples the SPDIF/I2S slots receive) as
one ADAT lightpipe signal on a single GPIO (default 12), fully concurrent with
the four output slots and PDM. 44.1/48 kHz only; the stream auto-suspends at
higher rates and auto-resumes when the rate returns. RP2040 compiles the
feature out (2 output channel pairs only).

**Engine** (`adat_output.c/h`): NRZI is encoded on the CPU, so the PIO program
is a single `out pins, 1` at 256 x Fs and the clkdiv is the IDENTICAL value the
S/PDIF TX SMs run. ADAT TX always runs the nominal `ceil(sys / Fs)` divider in
every input mode; the old servo-divider handoff (`adat_output_servo_divider()` /
`adat_servo_div`, plus the resync-time pickup of the input servo's trimmed
divider) is gone. Input clock tracking is now a `sys_clk` ppm trim (soft VCXO;
see "Clock Servo"), and because ADAT TX derives from `sys_clk` like the SPDIF/I2S
slots it stays rate-locked to them with zero long-term drift by construction, no
per-clock coordination needed. Frames (256 bits: `[1][10x0][1][u3..u0]` then 8 x 6 x `[1][nibble]`, user
bits 0) are encoded on Core 0 in `process_input_block()` after the slot gives
and written into a 576-frame (18 KB BSS) ring drained by DMA CH13, with CH14 as
an IRQ-less control channel that rewrites the read address at the ring end.

Encoding is LUT-driven. Each PCM byte stuffs to a fixed 10-bit token
`[1][hi nibble][1][lo nibble]`, and NRZI is linear (the entry-level-1 line
pattern is the bitwise complement of the entry-level-0 pattern), so a 256-entry
`uint16` table `adat_token_lut[256]` of pre-NRZI'd tokens (built in
`adat_output_init()`, level-0 wire pattern) plus one XOR mask per byte replaces
the old separate stuff, pack, and prefix-XOR-per-word passes. `adat_encode_frame()`
chains the line level through the three tokens of each channel as an XOR mask
(each token's LSB is the line level during its last bit), prepends the
per-entry-level NRZI'd 16-bit sync header (`adat_hdr_nrzi[2]` / `adat_hdr_exit[2]`),
and packs header + eight 30-bit chunks with the same fixed shifts as before;
silence templates are built at init through the same encoder. Bit-exactness
against the old encoder was proven by a 4-million-frame host differential test
(chained levels, silence templates, all 256 tokens x 3 positions x 2 levels,
float edge cases); codegen on Cortex-M33 -O3 dropped from 311 to 231
instructions per frame. Samples arrive already converted to S24 in place in
`buf_out` (see `output_s24.h`): while ADAT is active the pipeline's single
float->S24 pass feeds both the S/PDIF slots and this encoder, and
`adat_output_push_block()` takes `const out_s24_t (*)[192]` and encodes the
integers directly. While ADAT is inactive the pipeline skips the staging pass
(fused per-pair convert+interleave, rows stay float) and the push is gated off
by the same per-packet snapshot (`adat_output_is_active()`, RAM-resident;
shared with Core 1 via `core1_eq_work.finalize_s24`), so the ADAT-off steady
state pays no extra memory pass and the encoder can never read unconverted
rows. PIO-side NRZI was
rejected (a two-mode encoder would need mid-stream resync); the encode LUT is intentionally
not shared with the planned ADAT input, whose NRZI decode is `x ^ (x >> 1)` and
whose destuffing is two masks, needing no LUT.

**Alignment** *(rewritten 2026-10-11 for the output rings)*. ADAT plays a constant 96 frames (`ADAT_ALIGN_LEAD_FRAMES`, 2 ms at 48 kHz) behind the slots, counted in DMA words consumed. On the wire that is 95 frames behind an S/PDIF slot 0 and 93 behind an I2S slot 0, because every counter also includes the 8 words its DMA has prefetched into the PIO FIFO (1 ADAT frame, 2 S/PDIF frames, 4 I2S frames); the figure is constant per slot-0 type, so nothing drifts. The invariant is exact accounting: ADAT's ring lead always equals 96 + slot 0's signed ring fill (`output_ring_fill_signed(0)`) + `adat_excess_frames`, where the excess is silence ADAT has written ahead of the slot timeline and not yet matched. Three operations keep it:

- **Push.** `adat_output_push_block()` encodes the same admitted block the slot rings just received, so lead and fill grow together. Blocks the ring admission refuses are pushed nowhere.
- **Skip mirror.** When the ring admission (`output_rings_pre_give()`) applies an underrun re-anchor skip of S frames to every slot, it calls `adat_output_on_slot_skip(S)` in the same call, before the block that follows is pushed. ADAT writes S minus its excess in silence and clears the excess. The slots and ADAT therefore shift by the same amount, in the same order relative to the audio.
- **Keep-alive.** `adat_output_keepalive()` tops the lead back up to 96 frames whenever slot 0 runs dry, adding what it writes to the excess. It runs from `adat_output_task()` in the main loop and from the output-ring stall alarm (1 ms period, see "Output Ring Stall Bracket"), so the ADAT reader never passes its writer, even through a flash write. It computes the lead from slot 0's counters rather than the ring position, so it stays correct even if a stall outside any bracket let the reader overtake the writer. Excess only accrues while slot 0 is in deficit, and the skip that ends the deficit always covers it. A skip smaller than the excess is therefore unreachable, and is handled as a local resync. The skip mirror also recomputes the post-skip lead from slot 0's counters first; a lead above `ADAT_MAX_LEAD_FRAMES` (928, the most the slot admission cap can produce) means the slot accounting is broken, so it requests a local resync instead of writing a span that would lap the ADAT reader (added 2026-10-11).

Silence frames carry an even number of ones (2 in the header, 48 forced stuffing bits), so they leave the NRZI line level unchanged. An all-silence region replays seamlessly, and the write pointer can be placed anywhere inside one.

**Resync anchor.** `adat_output_resync()` fills the whole ring with silence, then with interrupts off measures slot 0's signed fill, sets the write pointer to 96 + fill frames ahead of the DMA start (or to 96 with the difference recorded as excess when slot 0 is in deficit; or clamped to `ADAT_MAX_LEAD_FRAMES` with `slip_count` incremented if slot 0 reports more fill than admission allows, so a broken reading can never index past the ring), arms the DMA and starts the SM. ADAT sits on PIO1 and cannot join the slots' synchronized start, so the anchor is exact to within one frame of slot 0's sub-frame position. It runs after every synchronized output restart (`complete_pipeline_reset` Phase 6, `enable_outputs_in_sync`), at boot, and for a local resync. Before 2026-10-11 the resync always seeded exactly 96 frames and ignored the slots' fill, so the ADAT-to-slot offset varied with how far slot 0 had played or been prefilled at that moment.

**Ring size.** The ring must hold the cushion plus a full slot ring: 96 + (512 - 64) = 544 frames (`ADAT_MAX_LEAD_FRAMES`), so `ADAT_RING_FRAMES` is 576 (960 for the 1024-frame rings before 2026-10-23) (static-asserted, along with the assumption that the I2S slot fill cap is no larger than the S/PDIF one). The pool-era 896 frames only covered the 768-frame blocking-give cap.

**Config/state machine:** vendor commands 0xCA-0xCE (see reference table)
record intent and set `adat_output_config_dirty`; the main loop applies it
inside the standard muted `prepare/complete_pipeline_reset` bracket (serviced
before `process_pin_changes` so a released ADAT GPIO cannot be clobbered).
`perform_rate_change()` drives the rate policy; `adat_output_resync()`
re-derives rate validity from `audio_state.freq` so a boot preset at 44.1 kHz
is handled without a rate-change event. The ADAT pin participates in
`is_pin_in_use` while config-enabled; the bulk/preset restore paths validate
it via `adat_pin_acceptable()`.

**Persistence:** identical to the other physical IO config, honoring
`output_config_mode`: per-preset fields in `PresetSlot` (SLOT_DATA_VERSION 23)
and device-global fields in `FlashOutputConfig` (directory V8; the pre-growth
23-byte layout is frozen as `FlashOutputConfig_v7` for the V5/V6/V7 directory
snapshots). Bulk params carry `WireAdatConfig` as the final section
(WIRE_FORMAT_VERSION 17, 5872 bytes). Notify event `NOTIFY_EVT_ADAT_STATE`
(0x08) reports enabled/active/pin on every state change.

Full specification: `Documentation/Features/adat_output_spec.md`

---

## Per-Channel Input Preamp
*Last updated: 2026-04-09*

### Overview

The input preamp is per-channel rather than a single global value. Each USB input channel (L/R) has an independent gain control, allowing asymmetric preamp adjustments. Arrays are sized by `NUM_INPUT_CHANNELS` (currently 2).

### Globals

| Variable | Type | Description |
|----------|------|-------------|
| `global_preamp_db[NUM_INPUT_CHANNELS]` | `float` | Per-channel preamp gain in dB |
| `global_preamp_mul[NUM_INPUT_CHANNELS]` | Platform-dependent | Pre-computed linear multiplier (float on RP2350, Q28 on RP2040) |
| `global_preamp_linear[NUM_INPUT_CHANNELS]` | `float` | Linear gain for metering/display |

### Vendor Commands

| Code | Command | Direction | Description |
|------|---------|-----------|-------------|
| 0xD0 | REQ_SET_PREAMP_CH | OUT | Set preamp gain for channel (wValue=channel index) |
| 0xD1 | REQ_GET_PREAMP_CH | IN | Get preamp gain for channel (wValue=channel index) |
| 0x44 | REQ_SET_PREAMP | OUT | Legacy: sets all input channels to the same gain |
| 0x45 | REQ_GET_PREAMP | IN | Legacy: returns channel 0 gain |

### Persistence

- `SLOT_DATA_VERSION` 12 adds `preamp_db_per_ch[NUM_INPUT_CHANNELS]` to `PresetSlot`
- Legacy `preamp_db` field still populated on save for backward compatibility with older firmware
- Slots with version < 12 initialize all per-channel preamp values from the single legacy `preamp_db`
- `WirePreampConfig` (16 bytes) section in `WireBulkParams` V6+

---

## Master Volume
*Last updated: 2026-05-27*

### Overview

Device-side master volume providing an attenuation-only ceiling on all output channels. This is independent of the USB Audio Class host volume control and is applied as the final gain stage before output.

### Range & Semantics

- **Range:** -127 dB to 0 dB (0 dB = unity, no attenuation)
- **Mute sentinel:** -128 dB = full mute
- **Direction:** Attenuation only — cannot boost above unity
- **Application point:** Post-output-gain: `output_gain * host_volume * master_volume`
- **Scope:** Affects all output channels uniformly
- **Does NOT affect:** Loudness compensation, volume leveller, EQ, crossfeed — only the final output gain stage

### Core 1 Integration

Core 1 sees the master-volume-scaled `vol_mul_master` transparently via the `Core1EqWork` handshake struct. No special handling needed in the EQ worker — the combined volume multiplier is pre-computed by Core 0.

### Vendor Commands

| Code | Command | Direction | Description |
|------|---------|-----------|-------------|
| 0xD2 | REQ_SET_MASTER_VOLUME | OUT | Set master volume (-128 to 0 dB) |
| 0xD3 | REQ_GET_MASTER_VOLUME | IN | Get master volume |
| 0xD4 | REQ_SET_MASTER_VOLUME_MODE | OUT | Set master volume persistence mode (0=independent, 1=with preset) |
| 0xD5 | REQ_GET_MASTER_VOLUME_MODE | IN | Get master volume persistence mode |
| 0xD6 | REQ_SAVE_MASTER_VOLUME | IN | Persist live master volume to directory's independent field |
| 0xD7 | REQ_GET_SAVED_MASTER_VOLUME | IN | Get the directory's independent master volume |

### Persistence

- `SLOT_DATA_VERSION` 12 adds `master_volume_db` to `PresetSlot`
- Directory-level `master_volume_mode` (default 0 = independent): mode 0 saves/restores master volume from a directory field decoupled from presets; mode 1 saves/restores it as part of each preset (legacy behavior)
- Preset directory response is 7 bytes — byte 6 = `master_volume_mode`
- Factory default master volume = `MASTER_VOL_DEFAULT_DB` (-20 dB) — applied at boot when the directory is fresh, and on legacy migration
- `apply_master_volume_db()` in `flash_storage.c` delegates to `update_master_volume()` so all paths emit host notifications
- Slots with version < 12 default to 0 dB master volume (unity, no attenuation)
- `WireMasterVolume` (16 bytes) section in `WireBulkParams` V6+

### Preset Context vs Factory Reset (2026-05-27)

Master volume is re-derived on every preset *context* change (preset load, active-slot delete, boot) via `apply_master_volume_from_mode(slot_or_null, is_boot)` — the single source of truth for "what master volume becomes when the context changes":

- **with-preset mode (1):** a V12+ slot uses its own `master_volume_db`; any context without one (empty slot, legacy pre-V12 preset, NULL) gets `MASTER_VOL_DEFAULT_DB` (−20 dB).
- **independent mode (0):** boot re-applies the saved directory value; **runtime is a no-op** so the live value survives every preset load. This honors the console contract "loading a preset never changes it" — see `Documentation/Features/master_volume_independent_load.md`.

`apply_factory_defaults()` deliberately does **not** touch master volume — it resets only the DSP processing chain. The context callers (`preset_load`, `preset_delete` active-slot branch, `preset_boot_load`) invoke `apply_master_volume_from_mode()` after the chain reset. `flash_factory_reset()` is not a context switch and does not call the helper, so the master-volume ceiling survives factory reset in both modes.

---

## Audio Input Source System
*Last updated: 2026-10-10 (only the deferred switch handler writes `active_input_source`; factory defaults queue a switch to USB)*

Abstraction layer enabling selection between multiple audio input sources. Supports USB (default), up to three SPDIF inputs, I2S, and (RP2350 only) an 8-channel ADAT lightpipe input.

### Files

- `audio_input.h`: `InputSource` enum, globals, constants, I2S rate enum helpers, SPDIF-input index/enable helpers
- `audio_input.c` — Global definitions

### Input Source Enum

```c
typedef enum {
    INPUT_SOURCE_USB    = 0,
    INPUT_SOURCE_SPDIF  = 1,   // SPDIF input 1 (always enabled)
    INPUT_SOURCE_I2S    = 2,
    INPUT_SOURCE_ADAT   = 3,   // 8-channel ADAT input (RP2350 only; disabled until enabled)
    INPUT_SOURCE_SPDIF2 = 4,   // Optional SPDIF input 2 (disabled until enabled by host)
    INPUT_SOURCE_SPDIF3 = 5,   // Optional SPDIF input 3 (disabled until enabled by host)
    INPUT_SOURCE_SPDIF4 = 6,   // Optional SPDIF input 4 (disabled until enabled by host)
} InputSource;
```

`input_source_valid()` accepts every value 0..6 (ADAT = 3 is structurally valid on both platforms so presets round-trip; RP2040 keeps only the config state and can never select it); `input_source_selectable()` additionally requires that an optional SPDIF be enabled, or that ADAT be enabled with a pin set on RP2350, before it is offered. `input_source_is_spdif()`, `spdif_index_for_source()` (0..3), and `spdif_source_for_index()` map between the two SPDIF representations. The optional sources must stay contiguous from `INPUT_SOURCE_SPDIF2` — those two helpers are arithmetic, not lookup tables, and a `_Static_assert` in `audio_input.h` enforces it.

### Multiple Selectable SPDIF Inputs
*Last updated: 2026-08-02 (raised from 3 to 4 selectable inputs)*

Up to `SPDIF_RX_NUM_INPUTS` (4) SPDIF inputs share the single RX PIO state machine and DMA pair; only the active one ever claims its GPIO. SPDIF input 1 (`INPUT_SOURCE_SPDIF`, default GPIO 5) is always enabled and behaves exactly as before. The optional inputs 2, 3 and 4 (`INPUT_SOURCE_SPDIF2`..`INPUT_SOURCE_SPDIF4`, default GPIOs 20, 21 and 22) are **disabled by default**.

- **Enable model.** `spdif_rx_enabled_ext` is a 3-bit mask (bit 0 = SPDIF2 .. bit 2 = SPDIF4; `SPDIF_RX_ENABLED_EXT_MASK` is the valid-bit mask applied to anything arriving from the wire or flash). While an input is disabled it is not offered by `REQ_SET_INPUT_SOURCE`, and its configured pin is a stored preference only: invisible to pin-conflict validation, so other functions may use GPIO 20/21/22 freely. Enabling validates the configured pin via `spdif_input_enable_acceptable()` (valid GPIO, not claimed by any other function) and is rejected with `PIN_IN_USE` on a conflict. Disabling the input that is the active source (or the target of a pending source switch) is rejected; the host must switch away first.
- **GPIO lifecycle.** `spdif_rx_pin_for_index()` resolves an input's pin (`spdif_rx_pin` for index 0, `spdif_rx_pin_ext[]` for 1..3), `spdif_rx_pin_default_for_index()` gives its factory GPIO (the `PIN_RESET_TO_DEFAULT` target and the unset-pin fallback), and `spdif_rx_active_pin()` gives the GPIO the RX library should run on for the current source. `spdif_input_stop()` resets the RX GPIO to `GPIO_FUNC_NULL` (the vendored `pico_spdif_rx` library never did); it releases the pin it recorded at start time, not the current config, so a pin change or an input switch actually frees the previous pad.
- **Switching between SPDIF inputs** reuses the existing deferred source-switch machinery (full stop, pipeline reset, restart on the new GPIO, outputs muted until lock). Inter-slot output alignment is preserved by the same complete-reset path as every other source switch.
- **Vendor commands.** `REQ_SET_SPDIF_RX_PIN` (0xE4) and `REQ_GET_SPDIF_RX_PIN` (0xE5) carry the input index (`wValue = (index<<8)|GPIO` on SET, `wValue = index` on GET, index 0..3); a bare pin from an old host targets index 0. `REQ_SET_SPDIF_INPUT_ENABLE` (0xE9, `wValue = (index<<8)|enable`, index 1..3) toggles the mask; `REQ_GET_SPDIF_INPUT_CONFIG` (0xEF) returns 6 bytes (was 5): the input count, the enable mask over all inputs (bit 0 always set), and all four GPIOs, so a host can build its source list data-driven. A host that asks for fewer bytes gets a short read of the same layout.
- **Default channel names.** SPDIF input 1 keeps "SPDIF L/R"; inputs 2/3/4 produce "SPDIF 2 L/R" .. "SPDIF 4 L/R" so the host can tell them apart. The name generator is index-driven, so it needed no change.
- **Persistence.** The device-global store is `FlashOutputConfig`: inputs 2/3 use `spdif_rx_pin_ext[2]` (directory V12) and input 4 the appended `spdif_rx_pin4` byte (directory **V16**). The array is not widened in place because the frozen `FlashOutputConfig_v11..v15` structs are strict prefixes that the directory migrations copy forward; `cfg_spdif_ext_pin()` / `cfg_spdif_ext_pin_get()` in `flash_storage.c` index the split storage as one array. Per-preset storage mirrors this: the enable mask and the 2/3 pins are slot V24 fields, input 4's pin is the slot **V35** tail byte. A V24..V34 slot carries a mask with bit 2 clear, so loading such a preset disables SPDIF 4 — the same "the slot's mask fully overrides" rule that already applied to inputs 2/3.
- **Wire format.** `WireInputConfig.spdif_rx_pin_ext` grows from 2 to 3 entries and consumes that section's last reserved byte, shifting `spdif_rx_enabled_ext_p1`, `i2s_clock_mode` and the ADAT input fields down one byte; the section stays 16 bytes and every later wire offset is unchanged. This is `WIRE_FORMAT_VERSION` **28**, so hosts pushing a V27 bulk payload are rejected until they are rebuilt. The input-config section now has no reserved bytes left; a further input-config field needs a new wire section.
- **Platform-independent:** identical on RP2040 and RP2350. GPIO 22 is free of every default assignment on both.

### Switching Behavior

- Source switching is deferred to the main loop via `input_source_change_pending` / `pending_input_source` flags (same pattern as output type switching)
- On switch: drain USB ring, `prepare_pipeline_reset()`, update `active_input_source`, `complete_pipeline_reset()`
- When input is not USB, `usb_audio_drain_ring()` is skipped — USB enumeration stays active but audio data is silently dropped
- SPDIF RX hardware only runs when SPDIF is the selected input source; I2S RX hardware only runs when I2S is the selected input source. The two share the same PIO SM and DMA channels, which is safe because inputs are switched, never mixed. The four SPDIF inputs likewise share the single RX SM/DMA pair; switching between them runs the same full stop/restart so only the active input's GPIO is ever claimed
- Switching to I2S applies the selected `i2s_input_rate` (via `perform_rate_change()` when it differs from the current rate), then runs the same drain/prefill-to-50%/enable-in-sync handshake as SPDIF (minus the lock wait, since the synchronous input runs as soon as it is started). See the I2S Input prefill note below

### Input Capture Arena
*Last updated: 2026-08-06 (new: the three capture buffers overlay one arena; receiver exclusivity is now a checked invariant)*

The three input receivers never run at the same time, so their capture buffers share one BSS arena (`firmware/DSPi/input_capture_arena.h`, defined in `audio_input.c`):

| Member | Owner | RP2040 | RP2350 |
|--------|-------|--------|--------|
| `spdif_fifo` | `pico_spdif_rx`'s `fifo_buff` | 12,288 B | 12,288 B |
| `i2s_ring` | `i2s_input.c`'s `i2s_rx_ring` | 4,096 B (1 pair) | 32,768 B (4 pairs) |
| `adat_ring` | `adat_input.c`'s `adat_rx_ring` | n/a | 8,192 B |
| **Arena** | union, aligned to the largest member's DMA wrap | **12,288 B / 4096-aligned** | **32,768 B / 8192-aligned** |

This reclaims 4,092 B of BSS on RP2040 and 20,480 B on RP2350. Each module aliases its old symbol name onto its member with a macro, so all other references are unchanged, and each static-asserts its own ring constants against the arena geometry. The arena's alignment equals one I2S row, which is what keeps every row inside its own DMA write-address wrap region; the ADAT ring wraps on the whole 8 KB member. The SPDIF FIFO is chained ping-pong DMA (no address wrap) and needs only 4-byte alignment. Buffers deliberately **not** in the arena: `adat_ring` (ADAT *output*, runs with any input), the slot rings (`.slot_rings`), `pdm_dma_buffer`, and the USB `audio_ring`.

**Ownership invariant.** At most one receiver runs, and only the one the active source owns. `input_arena_claim()` panics if the arena is already held by a different owner; each `*_input_start()` claims (USB claims nothing) and each `*_input_stop()` releases. Two enforcement points keep the invariant true rather than merely asserted:

- `stop_foreign_input_receivers(keep)` (main.c) stops every receiver that is *actually running* but is not `keep`, keyed on running state rather than on the old source enum. It runs once per main-loop iteration, and at the top of the deferred source switch with the incoming source as `keep`, which makes the per-branch stops no-ops.
- **Only the deferred switch handler writes `active_input_source`** (added 2026-10-10). `apply_factory_defaults()`, which a factory reset, an empty or corrupt slot load, and an active-slot preset delete all run, queues a switch to USB through `pending_input_source` + `input_source_change_pending` instead. It also retargets an in-flight switch to another source, so a factory reset always lands on USB. Earlier firmware wrote the source directly, which skipped the handler's rate change to the retained USB rate, the nominal divider restore after a servoed input, the receiver stop, the MCK stop, the LG Sound Sync and host notifications, and the stale `rate_change_pending` clear. The old post-restore receiver-stop calls at the preset-load, factory-reset and bulk-apply restart points were removed with it, since none of those paths can change the source underneath a running receiver any more. ADAT is the receiver this protects most, because it has its own DMA channel and PIO SM (DMA 15, PIO1 SM2) and nothing else interlocks it.

The flash-blackout behavior is unchanged: ADAT may keep running through a blackout while it *remains* the active source (see "ADAT Input", flash-write bracket).

**SPDIF teardown alarm race.** `spdif_rx_end()` now sets a `shutting_down` flag before cancelling the pending alarm (both under `save_and_disable_interrupts()`), and every alarm callback plus the DMA IRQ handler returns immediately when it is set; `spdif_rx_start()` clears it. An alarm that had already latched could otherwise fire inside the teardown window, re-run `_spdif_rx_capture_start()` (re-claiming DMA and re-arming a write into the shared FIFO) and schedule a successor the teardown never cancels, chaining a retry every 100 ms behind a stopped receiver.

### USB Behavior While Non-USB Input is Active (2026-05-04)

The device follows the always-accept architecture used by RME / UA / MOTU / Focusrite: when the user picks SPDIF (or any other future external source), the USB audio class interface stays fully enumerated, the iso OUT endpoint stays armed, and the SOF feedback endpoint keeps emitting valid timing — Windows usbaudio.sys and macOS CoreAudio see a normal, well-behaved UAC1 device at all times. The "switch" is purely a DSP routing decision; the host never knows.

To prevent the host stream from disturbing the SPDIF audio path, several spots that previously assumed USB is always the active source are now gated:

- **`uac1_apply_alt()` resync block (`usb_audio.c:927-950`)** — the `preset_loading = true` / `stream_restart_resync_pending = true` cascade only fires when `active_input_source == INPUT_SOURCE_USB`. Previously, every Windows audio-session open (e.g. touching the volume slider plays a notification ding) sent a `SET_INTERFACE alt=N` that triggered `complete_pipeline_reset()` inside `save_and_disable_interrupts()` and made the SPDIF input handler treat `preset_loading` as a lock-acquisition signal — yanking the outputs into prefill. The IRQ-disabled window also risked starving the `pico_spdif_rx` library's 10 ms decode-timeout alarm, causing lock loss after enough rapid alt-change cycles.
- **ISR ring push (`usb_audio.c:1174`)** — `usb_audio_ring_push()` is gated on `active_input_source == INPUT_SOURCE_USB`. In SPDIF mode the ring would never be drained and `overrun_count` would climb continuously while Windows streamed silence to the playback device.
- **Main-loop ring flush (`main.c:986-989`)** — when source isn't USB, the loop calls `usb_audio_flush_ring()` defensively to clear any packet pushed by the ISR in the brief window straddling an `active_input_source` change.
- **SOF feedback (`usb_audio.c:1236-1248`)** — when source isn't USB, `feedback_10_14` is forced to `nominal_feedback_10_14` regardless of what the servo computed. Output DMA can be transiently stalled during SPDIF prefill / lock loss, which would let the servo emit zero or garbage feedback values; Windows usbaudio.sys treats catastrophic feedback drift as a device fault and resets the device (which also drops the bulk Console pipe).

### Host Volume / Mute in Non-USB Mode (2026-05-04)

`audio_set_volume()` parks the host's value (`uac1_host_volume` + pending flag) instead of writing `audio_state.volume` when source isn't USB, and does not touch `audio_state.vol_mul` or `current_loudness_coeffs`. GET_CUR returns the parked value while pending so the host round-trip stays consistent. Mute application is gated symmetrically in the audio pipeline: both the RP2350 float and RP2040 Q15 gain paths guard `audio_state.mute` with `active_input_source == INPUT_SOURCE_USB`. Result: Windows volume slider and mute key have no audible or displayed effect during SPDIF playback. *Last updated: 2026-10-10*

The switch to USB in the deferred input-source switch handler calls `audio_thaw_host_volume()`, which applies a parked host value if no other controller has written user volume since. Factory defaults reach USB through the same handler, so it is the only thaw site. This keeps the rule that a parked value only ever exists while a non-USB input is live. See "User volume" above for the invariant.

This matches the user's product-level decision; it differs from the industry-standard pattern (RME TotalMix / UA Apollo, where host volume continues to act as master output gain on external sources) on purpose.

### SPDIF RX Pin
*Last updated: 2026-07-15 (pin byte 0xFF on SET = reset that input to its default)*

- Default: GPIO 5 (`PICO_SPDIF_RX_PIN_DEFAULT`) for input 1; GPIO 20/21 (`PICO_SPDIF_RX_PIN2_DEFAULT` / `PICO_SPDIF_RX_PIN3_DEFAULT`) for the optional inputs 2/3. Input 1's GPIO 5 moved off GPIO 11 to avoid colliding with `DAC_HW_MUTE_DEFAULT_PIN`; it is unclaimed by any default output, the UART, or the I²S pins, so the SPDIF RX defaults stop blocking a fresh-install enable of the DAC hardware-mute feature. A disabled input's pin is not reserved, so GPIO 20/21 stay free for other functions until the input is enabled.
- **Persistence follows `output_config_mode` (matches `output_pins[]`).** `REQ_SET_SPDIF_RX_PIN` updates the live `spdif_rx_pin` / `spdif_rx_pin_ext[]` global in RAM only; no implicit flash write. In with-preset mode the user `REQ_PRESET_SAVE`s to capture the pins in a slot (restored on load); in independent mode `REQ_SAVE_OUTPUT_CONFIG` persists them to the device-global block (applied at boot). The RX pins are part of the physical-IO config block, applied by `apply_output_config_from_mode()`.
- Configurable via `REQ_SET_SPDIF_RX_PIN` (0xE4) / `REQ_GET_SPDIF_RX_PIN` (0xE5); both carry the input index (0..3) in `wValue`. Enabled inputs conflict-check a new pin; a disabled optional input's pin is a stored preference validated at enable time (`REQ_SET_SPDIF_INPUT_ENABLE`, 0xE9).
- **On-flash layout:** `spdif_rx_pin` lives in one byte that V13 originally reserved as `input_source_padding[0]`. Reusing that byte keeps the `PresetSlot` size unchanged, so existing V13 presets remain CRC-valid (their padding bytes were zero-initialised, which fails GPIO validity and falls through to the live default — same observable behaviour as before this change).
- **Boot-time bootstrap.** `preset_boot_load` still reads the directory's legacy `spdif_rx_pin` field as the initial live value. This means users upgrading from auto-flush firmware keep their previously-configured pin until they save a preset under the new firmware. After that, the slot's value drives behaviour and the directory field is no longer consulted on subsequent boots that load the same slot.
- **Hot-swap supported.** Pin changes (from vendor command, bulk params apply, or preset load) while SPDIF input is active set `spdif_rx_pin_change_pending`; the main-loop deferred handler runs `spdif_input_stop()` → `prepare_pipeline_reset()` → `spdif_input_start()` so the running RX library picks up the new GPIO. Deferred to main loop because the `pico_spdif_rx` library's teardown (program removal, IRQ handler removal, DMA channel unclaim) is not safe to perform from USB ISR context.
- **`bulk_params_apply` integration.** `WireInputConfig.spdif_rx_pin` is applied on bulk SET when `apply_pins == true`, mirroring how `output_pins[]` is applied. If the new pin differs from the current one and SPDIF input is active, the hot-swap fires.

### SPDIF RX Implementation
*Last updated: 2026-10-17 (per-rate decoder programs for 44.1 and 88.2 kHz)*

**Library**: Forked from `elehobica/pico_spdif_rx` v0.9.3 at `firmware/pico-extras/src/rp2_common/pico_spdif_rx/`.

**DSPi library patches:**
- PIO2 support for RP2350
- Clock constants: 307.2 MHz sys_clk, 122.88 MHz PIO clock (divider 2.5 exact)
- Dedicated 44.1 kHz and 88.2 kHz decoder programs (`spdif_rx_44100.pio`, `spdif_rx_88200.pio`), selected per detected rate instead of per rate pair (see "Clock and decoder programs")
- Removed `pio_clear_instruction_memory()` (destroys shared PIO programs)
- Removed `irq_set_enabled(DMA_IRQ_x, false)` (disables entire shared IRQ line)
- Replaced `irq_has_shared_handler()` with private `irq_handler_registered` flag (prevents handler registration when other libraries share the IRQ line)
- Added `irq_remove_handler()` in `spdif_rx_end()` for clean lifecycle
- Added `save_and_disable_interrupts()` in `_spdif_rx_common_end()` to prevent re-entrant teardown
- Added `spdif_rx_get_block_ref()`: seqlock-published coherent pair of {decoded block count, entry timestamp of the DMA IRQ that completed that block}, written each block IRQ. While the receiver is stable and block-aligned every block is exactly 384 sub frames (192 frames), so deltas of the pair give an exact samples-over-time rate with only IRQ-latency jitter at the endpoints; used by the input clock servo's long-window rate estimator (see "Clock Servo" below)
- Added `spdif_rx_get_fifo_count_live()` / `spdif_rx_read_fifo_live()`: word-granular FIFO access that includes the words the in-flight DMA block transfer has already written, identified by probing which DMA channel's write address lies inside the block at `buff_wr_done_ptr` (a queued, just-completed, or mid-reconfigure channel points at another block base and is excluded; the completion-to-IRQ window safely reports 0). Availability is computed as (done_ptr advanced by in-flight words) minus the read pointer in the mod-2xFIFO pointer space; computing it as published-count plus in-flight instead double-counts once the reader is ahead of done_ptr and lets it run past the DMA write position (bug found on hardware: reader raced through stale ring data and pegged the consumer pool). `spdif_rx_get_fifo_count()` gained a guard reporting 0 when the read pointer is ahead of done_ptr so the IRQ overflow check never spuriously disposes a block

**PIO Allocation**:
- RP2350: PIO2 SM0 (dedicated block, no conflicts)
- RP2040: PIO1 SM2 (SM0=PDM occupies SM0 when active; SM1 was MCK pre-GPOUTn refactor and is now free — the patched `pio_clear_instruction_memory()` removal in the SPDIF RX library is still required because PDM and the I2S libraries share PIO program memory regardless of the MCK move)

**Clock and decoder programs**: sys_clk 307.2 MHz → PIO clock 122.88 MHz (divider 2.5, exact). The decoder re-synchronises on an edge, free-runs `cy` PIO cycles per symbol and latches at `lp` cycles into the next symbol, so `cy` must match the real symbol period or the latch points drift toward the following edge over the two- and three-symbol free runs. The upstream library shared one program per rate pair (44.1/48, 88.2/96, 176.4/192). Since 2026-10-17 each rate below 176.4 kHz has its own program, selected from the capture stage's detected rate (whose 6-symbol edge-interval windows separate 44.1 from 48 kHz without overlap); only one decoder program is resident at a time, so this costs no PIO instruction memory. 176.4 kHz still shares the 192 kHz program (neither is supported downstream).

| Rate | Symbol (PIO cycles) | Program | Latch points (symbols) |
|---|---|---|---|
| 44.1 kHz | 21.77 | `spdif_rx_44100` cy 22, lp 10 | 1.47 / 2.48 / 3.49 |
| 48 kHz | 20.00 | `spdif_rx_48000` cy 20, lp 10 | 1.50 / 2.50 / 3.50 |
| 88.2 kHz | 10.88 | `spdif_rx_88200` cy 11, lp 5 | 1.47 / 2.48 / 3.49 |
| 96 kHz | 10.00 | `spdif_rx_96000` cy 10, lp 5 | 1.50 / 2.50 / 3.50 |

The 44.1 kHz program uses lp 10 rather than cy/2 because the `wait` delay `cy-1+lp` must fit the 5-bit delay field (31). Previously 44.1 kHz ran the 48 kHz program and its latches sat at 1.38, 2.30 and 3.22 symbols, the last 0.22 symbol before the next edge. `tools/spdif_pio_rx_sim.py` models the decoder cycle by cycle (2-cycle input synchroniser, 2.5 fractional PIO clock) against a BMC stream and sweeps pulse-width distortion and random jitter until a word decodes wrongly over 400 frames:

| Rate | Program | Falling edge late | Falling edge early | Random jitter sigma |
|---|---|---|---|---|
| 44.1 kHz | shared 48 kHz (old) | 52.6 ns | 52.6 ns | 10.2 ns |
| 44.1 kHz | dedicated (new) | 82.2 ns | 80.4 ns | 15.6 ns |
| 48 kHz | dedicated | 71.6 ns | 81.4 ns | 12.9 ns |
| 88.2 kHz | shared 96 kHz (old) | 24.7 ns | 24.7 ns | 4.7 ns |
| 88.2 kHz | dedicated (new) | 37.2 ns | 35.3 ns | 6.5 ns |
| 96 kHz | dedicated | 32.5 ns | 42.3 ns | 6.9 ns |

Pulse-width distortion matters here because the decoder alternates rising- and falling-edge references, so an optical receiver's asymmetry shifts the latch points by its full amount. With the source off nominal by the 1 % decode-stability window the new 44.1 kHz program keeps 73 ns or more in both directions, and at the 3 % capture-window edge it keeps 60 ns, at least as much as the 48 kHz program at the same deviation. HW-untested; the simulation checks the program timing, not the pad or synchroniser behaviour.

**DMA**: Channels 5+6 (RP2350) or 4+5 (RP2040) on DMA_IRQ_1, which it owns outright at the highest priority (no output uses a DMA IRQ). *Corrected 2026-10-11: this line previously said DMA_IRQ_0.*

**State Machine**: `spdif_input.h/c`
- INACTIVE → ACQUIRING → LOCKED → RELOCKING (on signal loss) → LOCKED (on re-lock)
- Lock: ~64ms library internal stability + firmware debounce polls
- Loss: 10ms timeout
- Audio extraction: FIFO → 24-bit decode → per-channel preamp → buf_l/buf_r → process_input_block(). RP2040 scales decoded samples into Q28 with the same `sample << 6` full-scale convention as USB 24-bit input; RP2350 scales to float full-scale.
- **Live-tail (word-granular) delivery** (`spdif_rx_read_fifo_live`): the poll consumes words as the DMA writes them instead of waiting for whole 384-subframe block completions. Block-granular release delivered 4 consumer buffers (192 frames into 48-sample buffers) in one poll every 4 ms, which made the slot-0 consumer fill sawtooth across 4 buffers (observed 37-56% on hardware) even with a perfectly stable servo; word-granular delivery moves the fill continuously like the ADAT input's DMA ring. The in-flight block's sync/parity are not yet verified when its words are consumed; a corrupt block still drops lock within about one block time and the poll's pending-loss guard plus mute handling cover it, matching the previous behavior where the corrupt block had already been released to the FIFO.
- **Batched to ~1 ms granules**: the poll waits until `audio_state.freq / 1000` frames are available before calling `process_input_block()`. Word-granular availability with per-iteration processing fed the pipeline sliver-sized calls whose fixed cost (producer pool takes/gives, gain ramp + filter state setup) dominated: 54% CPU0 measured at 48 kHz, and each consumer buffer carried only a few samples so the fill count overstated the real cushion. Batching restores the USB-packet call profile: 9% CPU0, consumer fill flat at 56% with zero lock losses / parity errors over a 30 s hardware soak at 48 kHz (2026-07-20). *Last updated: 2026-07-20*

**Clock Servo**: `spdif_input_update_clock_servo()` runs every ~20 ms while locked and tracks the measured input rate. Since the soft-VCXO overhaul the servo actuates a **single physical clock**: it pulls `sys_clk` in ppm via `soft_vcxo_set_ppm()` (see "Soft VCXO") and writes **no** output PIO divider, MCK divider, or ADAT divider. Every output clock (SPDIF TX, internally clocked I2S TX, MCK, ADAT TX, PDM) derives from `sys_clk`, so they all move together and inter-slot alignment is preserved by construction; PDM is now tracked too (it never was before). The servo core is factored into `input_servo.c` (`input_servo_apply(float actual_freq, int fill_slot)` / `input_servo_reset()`) and shared by all three tracked input modes; callers own their own lock gating, rate limiting, and input-rate measurement. Per tick:

- **ppm feed-forward** against the *programmed* `ceil(sys_nominal / audio_state.freq)` divider: `ppm_ff = (actual_freq * div / sys_nominal - 1) * 1e6`. Referencing the actual divider (not the nominal rate) folds its ceiling-rounding residual into the feed-forward, so the integrator never has to hold it. `sys_nominal` is `clock_get_hz(clk_sys)`, which by convention keeps reporting 307.2 MHz; all nominal-divider math stays nominal, only the physical clock moves. A guard skips the tick when `|ppm_ff| > 2000` (a caller applied against the wrong pipeline rate).
- **Fill trim** (only when `fill_slot >= 0`), re-derived 2026-07-23 for the sample-granular fill signal: Loop B reads `get_slot_consumer_fill_frac()` (queued whole buffers + producer staging position + DMA in-flight remainder, in buffer units; negative = invalid, tick holds the previous command), IIR-smoothed (alpha 0.125). With the buffer quantizer gone a plain PI converges: P = 2 ppm/buffer of smoothed error plus a second 4 ppm/buffer slope beyond ±2 buffers; I = 5e-5 ppm/tick/buffer, clamp ±3 ppm, un-gated. A global slew limiter (0.5 ppm per ~20 ms tick) bounds carrier motion below what downstream receiver PLLs track; `input_servo_reset()` zeroes the limiter/integrator and re-seeds the IIR at target. History: the divider-servo gains on the continuous VCXO limit-cycled ±10s of ppm on whole-buffer fill and unlocked external DAC receivers; an interim 2026-07-22 retune (gated I, ±10 ppm clamp) bounded it, hardware-confirmed, before the fractional fill removed the cause. Signs follow the ppm convention (overfull consumer = outputs too slow = positive ppm = speed sys_clk up).

`input_servo_reset()` zeroes the integrator and parks the VCXO. It is called only when tracking starts (`spdif_input_start()`, `adat_input_start()`, `i2s_slave_arm()`) and on every stop/disarm path. A lock loss holds the last-good trim, so relock re-converges from where it was (soft_vcxo_spec.md section 5); since 2026-10-11 the ADAT lock path follows this too, instead of parking at every lock. The I2S slave cannot hold its trim: the main loop restarts its receiver on every RELOCKING, and that restart parks the VCXO through `i2s_slave_disarm()` and `i2s_slave_arm()`, so the I2S slave re-converges from nominal after every loss. No pipeline-reset path calls it, because there it would park a locked VCXO mid-lock. Every way out of a tracked input reaches a stop: source switches and factory defaults through the source-switch handler, clock-mode flips through an explicit stop, and any leaked receiver through `stop_foreign_input_receivers()`, so USB and I2S-master modes always run at exactly nominal sys_clk. **Servo gating**: the SPDIF and I2S-slave servos hold off until the detected rate equals `audio_state.freq` (the deferred rate change has landed; otherwise the feed-forward would be computed against the wrong divider). A signal-loss (RELOCKING) event on SPDIF or ADAT deliberately **holds the last ppm** rather than parking, so relock re-converges from where it was; full stop/disarm/source-switch paths park, and the I2S slave's loss path is such a restart. **I2S slave** now shares `input_servo_apply()` (its old private P-only divider-writing servo, `i2s_slave_update_clock_servo()`, is gone in substance), passing the first SPDIF-type slot as fill reference or -1 when none exists (rate term alone, ~0.1 ppm long-window); the edge-locked I2S TX slots follow the external BCK directly and need no servo, while PDM and SPDIF slots track via sys_clk. **Divider-ratio coupling moved to the nominal path**: the old `i2s_div = 2 * spdif_div` servo coupling is gone; a common sys_clk trim can never correct a divider *ratio* error, so the coupling lives in the nominal dividers themselves; `i2s_compute_divider()` (and the matching RX clock-master divider `rx_master_divider_24_8()`) now derive the I2S divider as exactly `2 * ceil(sys/f)` instead of `ceil(2*sys/f)` (identical at 44.1/48/96 kHz, future-proofs rates like 176.4 kHz where they differ by one LSB / ~287 ppm). Verified on RP2350 hardware at 48 kHz SPDIF input; the standalone fbdiv-dither mechanism is bench-proven, but the full-firmware servo soak (SPDIF, ADAT slave, I2S slave at 44.1/48/96 kHz on both platforms) and RP2040 fbdiv dithering are HW-untested. *Last updated: 2026-10-11 (I2S slave parks on every loss; previously 2026-07-22)*

**Rate measurement (long-window estimator)**: this estimator supplies the `actual_freq` argument to `input_servo_apply()` (feeding the ppm feed-forward above). The servo's rate term does not feed the library's `spdif_rx_get_samp_freq_actual()` straight in: that value is an 8-block moving average of DMA IRQ intervals which telescopes to two IRQ timestamps only ~32 ms apart (16 ms at 96 kHz), so a few microseconds of IRQ latency jitter is 100+ ppm of rate noise (which under the former divider-actuated servo slammed the 16.8 output dividers across several LSBs every tick, bounced the fill, and FM'd the outgoing SPDIF carrier; the soft VCXO would likewise wobble sys_clk). `spdif_input.c` mirrors the ADAT slave dual-anchor scheme: it reads the library's coherent `{decoded block count, IRQ timestamp}` pair (`spdif_rx_get_block_ref()`, exact 192-frame blocks while stable), keeps two anchors rotated every 8 s (steady-state span 8-16 s, endpoint jitter well under 1 ppm), and trusts the count-based rate once the span reaches 1 s (~10 ppm noise, well inside the servo's authority). The first second after lock is bridged by an IIR-smoothed (alpha 0.25) library estimate. Anchors re-arm on every lock acquisition, so counts never span the library's `block_count` reset on relock. *Last updated: 2026-07-22*

**Receiver hardening (2026-10-11).** Four `pico_spdif_rx` / `spdif_input.c` fixes landed with the overhaul merge. The decoded `block_count` is `uint32_t` (a signed `int` overflowed, which is undefined behaviour, after about 99 days at 48 kHz). The DMA IRQ takes its block timestamp before cancelling the decode-timeout alarm, whose variable cost otherwise jittered the estimator's endpoints. The live-tail count (`spdif_rx_get_fifo_count_live()`, `spdif_rx_read_fifo_live()`) returns 0 unless the library is STABLE, so unverified capture data is never delivered. `spdif_input_poll()` re-checks the library state at the lock edge, because a stable-then-lost pair of flags between two polls would otherwise enter LOCKED on a dead receiver. The mirror case, a lost-then-stable pair, is covered too (fixed 2026-10-11): the lock branch reads the rate from the library (`spdif_rx_get_samp_freq()`) instead of the IRQ-written copy, which the loss branch has just zeroed; with the copy, the poll stayed in RELOCKING, muted, until the next loss.

**Note (superseded by the soft VCXO)**: the earlier "slew-limit the applied divider to ±1 LSB per call" idea is moot now that the servo actuates a ppm sys_clk trim rather than a PIO divider. The soft VCXO's own control quantum is one pulse (~1.17 sys cycles of phase, ~3.8 ns), far finer than a 156 ppm divider LSB, and the long-window estimator keeps the rate term sub-ppm, so output clock wander is already bounded without an explicit slew limit. *Last updated: 2026-07-22*

**Output Prefill**: On SPDIF lock acquisition, outputs are disabled and consumer buffers drained via `drain_and_disable_outputs()`. The pipeline then feeds real audio into consumer buffers while outputs are stopped. Once slot 0's ring reaches `OUTPUT_RING_TARGET_FILL_FRAMES` (384 frames, 50% of the stats scale), outputs are started in sync via `enable_outputs_in_sync()`. This eliminates initial underruns after lock acquisition. Controlled by `spdif_prefilling` flag in `main.c`. *Last updated: 2026-04-12*

**Files**: `spdif_input.h` (API + status struct), `spdif_input.c` (lifecycle, audio extraction, clock servo, status queries)

### I2S Input
*Last updated: 2026-10-16 (inactive pairs' parked data pins reserve nothing; activation re-checks them). Previously 2026-07-15 (pin byte 0xFF on the pin SETs = reset the addressed target to its default)*

In the default MASTER clock mode (`i2s_clock_mode` = 0) I2S input keeps the device as the clock authority: the external source slaves to our BCK/LRCLK, the same shared clock pair the I2S outputs use (`i2s_bck_pin` / +1; master clocking always uses this pair regardless of clock-pin mode). Because the input is then synchronous to the device clock domain, there is **no clock servo, no rate detection and no lock state machine**; the subsystem is structurally a much simpler sibling of SPDIF RX. State is just INACTIVE / RUNNING. In SLAVE clock mode (`i2s_clock_mode` = 1) an EXTERNAL master drives BCK/LRCLK instead; that mode adds rate detection, a lock state machine and an output clock servo, described in the "I2S Clock-Slave Input Mode" section below. Everything in THIS section describes master mode unless noted.

**Multichannel input (RP2350).** The receiver fans out to up to `I2S_RX_MAX_PAIRS` (4) stereo pairs — **2/4/6/8 channels**, selected by `i2s_input_channels` — each one PIO state machine + one IRQ-less DMA ring + one independently-configurable data pin (`i2s_rx_pin[0..3]`), all sharing the single BCK/LRCLK. In `i2s_input.c` each pair is an `I2sRxPair` descriptor and every lifecycle step loops over `i2s_n_pairs` (= channels/2), so the single-pair path is just `n_pairs == 1` with no special-casing. In the clock-master role pair 0's SM runs `clkmaster` (drives BCK/LRCLK + reads pair 0) and pairs 1.. run the `slave` program waiting on the *same* pads pair 0 drives; in the slave role every pair runs `slave` against the external clock. All pairs are enabled on one cycle via `pio_enable_sm_mask_in_sync()` and advance in lockstep on the one shared clock. In the **slave role** every pair runs the same wait preamble, so all latch the same first frame. In the **master role** there is a deterministic one-frame asymmetry: pair 0 (`clkmaster`) starts sampling immediately and captures from frame 0, while the slave-program pairs cannot detect the start of frame 0 (LRCLK is already low at enable) and lock on the next LRCLK fall, capturing from frame 1. `i2s_input_start()` corrects this by advancing pair 0's read pointer one stereo frame (2 words) when `clock_master && pairs > 1`, so every pair's read pointer lands on the same physical frame (cycle-exact per the clkmaster/slave PIO start timing, independent of ADC settling). Either way the 2/4/6/8 channels are sample-aligned (the inviolable inter-channel guarantee), exactly as the TX path's `audio_*_enable_sync()` aligns the outputs. The poll consumes the per-pair minimum available and reads the same frame index from every ring, deinterleaving pair `p` into input channels `2p`/`2p+1`; the pipeline's `n_active_inputs` for the I2S source is `i2s_input_channels`, so the 8×9 matrix applies automatically (identical to 8-ch USB input); the leveller runs over all active inputs, and loudness and crossfeed run per output (loudness post-gain, crossfeed per output pair post-matrix) rather than being bypassed. A live channel-count *raise* is applied to `i2s_input_channels` immediately but the extra pairs are not allocated until the deferred restart fires, so for the intervening poll cycles the poll zeroes the not-yet-filled `buf_in_ext` rows up to `active_input_channel_count()` — the matrix sees silence on the new channels, never stale samples (preserving the no-leak invariant). A count *drop* needs nothing (the matrix just stops reading the surplus rows). RP2040 is stereo-only (`I2S_RX_MAX_PAIRS` = 1). **Config:** vendor `REQ_SET/GET_I2S_INPUT_CHANNELS` (0xF3/0xF4) and `REQ_SET/GET_I2S_RX_PIN` (0xF1/0xF2, now `wValue = (pair<<8)|gpio`); a channel-count or higher-pair pin change restarts the input (`i2s_input_restart_pending`) so every pair re-syncs (pair-0 stereo keeps its lighter hot-swap). The active pair pins are kept mutually distinct AND clear of the I2S clocks: `check_i2s_rx_pin(pin, pair, n_pairs)` rejects a pin that is invalid, a clock pin (BCK/LRCLK), used by a fixed peripheral, or already on another of the first `n_pairs` pairs. The clock check is **unconditional**. An I2S RX data pin coexists with BCK/LRCLK whenever the input runs, so it must avoid them even while I2S is inactive (closing the gap where `is_pin_in_use()` only reserves the clocks while they run). **Inactive pairs reserve nothing (since 2026-10-16).** A parked pin of an inactive pair blocks no other function and no other pair, so in stereo mode pair 0 may take GPIO 4 even though pair 3 is parked there. Two pairs may therefore share a parked pin, and the check moves to the moment a pair becomes active. `REQ_SET_I2S_RX_PIN` checks against the current active pair count. The count-*increase* path checks each newly active pair against the **new** count, so it also catches two newly active pairs parked on the same GPIO, and refuses the raise with PIN_IN_USE. The restore paths were already lazy (see below). DSPi Console mirrors this. Its `pinInUseBy` reserves only active pairs for every consumer, the I2S Input section exposes the parked pins under "Unused pairs" so a refused raise can be fixed, and its revert restores parked pins before a count raise. The four data-pin defaults are the contiguous block GPIO 1/2/3/4 (pairs 0/1/2/3), all clear of the default peripheral assignments so enabling 4/6/8-channel input out of the box never self-collides. The bulk and preset/flash **restore paths** validate the proposed pin set as a unit (`i2s_rx_pin_set_acceptable()`, against the effective BCK that transfer installs) and reject an inconsistent pushed/stored I2S config rather than apply it — so two state machines can never come up on one GPIO or on a clock pin regardless of how the config arrived. The restore paths also validate the incoming **BCK** itself before installing it (`i2s_bck_pin_acceptable()`: BCK + LRCLK valid GPIOs, no fixed-peripheral collision), keeping the live pin on failure — BCK/LRCLK are push-pull clock outputs, so an invalid GPIO could fault `pio_gpio_init()` and a collision with an output pin would be driver contention. (The vendor `REQ_SET_I2S_BCK_PIN` path keeps its additional "reject while an I2S output is active" guard; a restore omits it since it installs the output config and clock pair together.) **Persistence:** `WireInputConfig` carries the count + 3 ext pins in reserved bytes (`0 = absent`, no `WIRE_FORMAT_VERSION` break); per-preset `PresetSlot` V22 tail-append; device-global `FlashOutputConfig` via a directory V4→V5 migration.

**Files**: `i2s_input.h` (API), `i2s_input.c` (lifecycle, DMA ring, poll), `i2s_input.pio` (all three RX PIO programs).

**Clock-master election.** The input SM has two roles, decided by `i2s_input_should_be_master()` in `main.c`:

- **Slave** (at least one output slot is I2S): the lowest-index I2S output slot remains the clock master as before; the input SM runs a wait-driven program (`audio_i2s_rx_slave`, 7 instructions, divider 1.0) that samples the data pin against the BCK/LRCLK pads. Both slave variants are authored with placeholder GPIO indices in their `wait gpio` instructions (0 = BCK, 1 = LRCLK); `load_slave_program()` copies the selected variant to RAM and rewrites the 5-bit GPIO index of every WAIT-source-GPIO opcode because `i2s_bck_pin` is user-configurable. The external-clock role loads a different, CHECKED variant (`audio_i2s_rx_slave_checked`, 24 instructions) that re-verifies LRCLK framing every frame; see "I2S Clock-Slave Input Mode" below.
- **Clock master** (no output slot is I2S): the input SM runs `audio_i2s_rx_clkmaster` (12 instructions), driving BCK/LRCLK via side-set with the exact TX-master divider (`sys_clk * 2 / Fs`, 24.8 ceiling) while shifting data in. Cross-PIO-block sync is never needed: input-as-master only happens when zero I2S outputs exist, and I2S TX slaves are divider-locked within their own block.

All programs: in_base = data pin, IN shift left, autopush 32, RX FIFO joined, 24-bit audio MSB-aligned in 32-bit frames, standard I2S 1-bit delay (LRCLK transitions during the last bit cell, matching `audio_i2s_clkout.pio`). The first word pushed after any (re)start is always a LEFT word, and pushes stay in strict L,R alternation through every path (including the checked variant's resync), so a ring word's index parity fixes its channel permanently.

**Resources.** Reuses the SPDIF RX footprint (free whenever SPDIF input is inactive) and, for multichannel, the DMA channels the SPDIF/I2S TX DMA-sharing work freed: PIO1 SM2 (RP2040, 1 pair) / PIO2 SM0..3 (RP2350). DMA channel allocation now differs by platform. **RP2350**: one ENDLESS-mode data channel per pair, `I2S_RX_DMA_BASE + p` (pair 0 = `PICO_SPDIF_RX_DMA_CH0` = CH5, pairs 1..3 = CH6/7/8), no reload channels; this freed CH9-12 for the spare pool and the soft VCXO (see DMA map above). **RP2040** (no ENDLESS mode): pair 0 uses a data+reload channel pair `I2S_RX_DMA_BASE + 2p` / `+2p+1` = CH4/5 (`PICO_SPDIF_RX_DMA_CH0/CH1`), stereo-only. A `_Static_assert` in `i2s_input.c` pins the I2S RX range below `SOFT_VCXO_DMA_PULSE` on both platforms. All pairs' SMs and DMA channels are claimed in `i2s_input_start()` and unclaimed in `i2s_input_stop()` so the SDK claim table stays consistent across input switches.

**IRQ-less DMA ring.** Each pair's data channel moves PIO RX FIFO words into a power-of-2-aligned word ring (write-address wrap, transfer count = ring words). Zero IRQs, so capture survives IRQ-disabled windows. The re-arming mechanism is platform-specific. **RP2350** uses a single ENDLESS-mode channel per pair (transfer count auto-reloads, write address wraps in hardware), with no reload/chain channel; **RP2040** (no ENDLESS mode) pairs each data channel with a reload channel: the data channel chains to it, and it rewrites the data channel's `al2_write_addr_trig` with the ring base, retriggering it forever. **Teardown must be race-free.** On RP2040 the two chained channels re-trigger each other, so aborting them with two sequential `dma_channel_abort()` calls lets one re-arm the other in the gap, intermittently hanging the abort's busy-wait (watchdog reset) or leaving a channel live after unclaim. `stop_all_dma_rings()` instead disarms every data channel's chain via the non-triggering CTRL alias, then aborts all channels in a single grouped `dma_hw->abort` write (bounded busy-wait guard); on RP2350 the ENDLESS channels have no chain loop, so the grouped abort alone suffices, followed by a transfer-count=0 restore so a stale encoded count cannot confuse a later restart. Every input stop (input-source switch, output type switch, flash bracket, pin hot-swap) goes through this path. `i2s_input_poll()` derives the fill level from the DMA write address, consumes whole stereo pairs only (capped at 192 frames per poll), masks the low byte, applies preamp (same Q28/float conversion conventions as SPDIF RX), and feeds `process_input_block()`. **It batches to a 48-frame minimum** (`I2S_INPUT_MIN_BLOCK`): the ring's write address advances per word, so without batching the fast main loop would feed `process_input_block()` a handful of frames at a time, and the budget-based CPU meter (`busy_us / (frames / Fs)`) would amortize the fixed per-block cost (Core 1 EQ-worker handshake, pipeline setup) over too few samples and read ~4x inflated (measured cpu0 66% vs USB 16% before batching). 48 frames matches the USB packet / consumer-buffer granularity, so I2S CPU tracks USB; the input is continuous so the threshold never starves, and it adds ~1 ms of latency at 48 kHz. (SPDIF RX does not need this: its library FIFO already advances in DMA-block chunks.) Because the ring length is even and the read pointer only moves in pairs, a word's ring position fixes its channel permanently; even a writer-laps-reader overrun garbles audio momentarily but can never swap L/R.

**Slave resync invariant.** `complete_pipeline_reset()` and `enable_outputs_in_sync()` rewind the I2S TX clock master to its PIO entry point, resetting LRCLK phase. A running slave-role input SM would misframe permanently, so both functions end with `i2s_input_resync()`: a no-op unless the input is RUNNING in the slave role, otherwise disable SM, drain the RX FIFO into the ring, re-anchor the read pointer at the DMA write address, restart the SM at its sync preamble. This makes the invariant structural; no call site needs to remember it.

**Startup prefill (ring target fill).** I2S input uses the same prefill handshake as SPDIF so outputs start against an output ring at the fill target (`output_ring_target_frames`) instead of whatever low fill (~18%) the startup transient leaves; the extra margin absorbs main-loop scheduling jitter (a delayed `i2s_input_poll()` under load would otherwise risk an underrun). The handshake lives in the main-loop I2S block and is gated on `preset_loading` (set by every disruptive op via `prepare_pipeline_reset`, which also clears the `i2s_prefilling` flag so the handshake restarts cleanly): once the DAC-mute hold has elapsed it calls `drain_and_disable_outputs()`, clears `preset_loading`, sets `i2s_prefilling`; then on each later iteration, when `get_slot_consumer_fill(0)` reaches 50% it calls `enable_outputs_in_sync()` and releases the DAC mute. Unlike SPDIF there is no lock to wait for, so the bring-up helper `i2s_input_bringup_prefill()` (used by the input-source switch and boot) applies the rate and MCK, starts the input, and deliberately does NOT enable outputs; it leaves that to the main-loop block. When the selected rate differs from the live rate, `perform_rate_change()` still runs its full reset (so SPDIF TX dividers reprogram via instance teardown/restart), briefly enabling outputs that emit muted silence before the block re-drains and prefills.

How the pools get filled depends on the input's clock role, because the prefill drain stops ALL outputs:

- **Master role** (no output slot is I2S; the input SM drives BCK/LRCLK): the input keeps self-clocking through the drain, so the real input audio fills the pools via `i2s_input_poll()`. The fill reaches 50% naturally.
- **Slave role** (an I2S output is the clock master): draining the outputs also stops the very BCK/LRCLK the input is clocked from, so the input produces nothing and could never reach 50% (the original bug: outputs stayed disabled forever, fill stuck at 0%). Instead the block synthesizes silent blocks into the pools with `i2s_input_prefill_silence()` until 50%; real audio resumes after `enable_outputs_in_sync()` restarts the clock master and `i2s_input_resync()` re-phases the input ring. Because input and output are then perfectly synchronous, the half-pool lead is conserved and the fill holds at ~50% (it does not drift back to the old ~18%).

**Suspend/resume sites** (mirroring SPDIF RX): `perform_rate_change()` (stop before, restart after; covers the master-divider change), `process_type_switches()` (restart with a freshly elected role; this is the role re-election point), flash-write brackets (`i2s_suspended_for_flash` + `resume_i2s_after_flash()`), preset load, factory reset, bulk-params apply, and the input-switch handler. `process_pin_changes()` needs no suspension: output data-pin moves never change the input role, and the structural resync covers the slave re-phase.

**Sample-rate authority.** With USB the host picks Fs, with SPDIF the source does; with I2S input *we* do. `i2s_input_rate` (44100/48000/96000, default 48000) is set via `REQ_SET_INPUT_RATE` and applied through the standard deferred `pending_rate` / `rate_change_pending` mechanism whenever I2S input is active. As part of this feature, `perform_rate_change()` now writes `audio_state.freq = new_freq`; previously only the USB host path updated it, so SPDIF-driven rate changes left the loudness/crossfeed/leveller recompute handlers and `REQ_GET_STATUS` on a stale Fs (latent defect, fixed).

**MCK.** The external source may need MCK, so the MCK run condition is now "any output slot is I2S **or** I2S is the active input": extended in `process_type_switches()` and started/stopped by the input-switch handler when entering/leaving I2S input with no I2S outputs (divider before enable, matching `REQ_SET_MCK_ENABLE` ordering).

**I2S RX data pin.** Default GPIO 1 (`PICO_I2S_RX_PIN_DEFAULT`); the four data-pin defaults are the contiguous block GPIO 1/2/3/4 (pairs 0/1/2/3), all unused by any other default. Same persistence and hot-swap model as `spdif_rx_pin`: RAM-only on SET, slot-scoped via `REQ_PRESET_SAVE` (with-preset mode) or `REQ_SAVE_OUTPUT_CONFIG` (independent mode); changes while I2S input is active set `i2s_rx_pin_change_pending`, handled by a deferred main-loop restart. A BCK pin change while the input SM is the clock master (allowed; the any-I2S-output case is rejected as before) sets `i2s_input_restart_pending` for the same handler.

### Vendor Commands

| Code | Command | Direction | Description |
|------|---------|-----------|-------------|
| 0xE0 | REQ_SET_INPUT_SOURCE | OUT | Set active input source (uint8_t payload; 2 = I2S, 3 = ADAT, 4 = SPDIF2, 5 = SPDIF3, 6 = SPDIF4; optional SPDIF inputs rejected unless enabled, ADAT rejected unless enabled + pin set on RP2350) |
| 0xE1 | REQ_GET_INPUT_SOURCE | IN | Get active input source (returns uint8_t) |
| 0xE2 | REQ_GET_SPDIF_RX_STATUS | IN | Get SPDIF RX status (16-byte SpdifRxStatusPacket) |
| 0xE3 | REQ_GET_SPDIF_RX_CH_STATUS | IN | Get IEC 60958 channel status (24 bytes) |
| 0xE4 | REQ_SET_SPDIF_RX_PIN | IN* | Set a SPDIF input's RX pin (wValue = (index<<8)\|pin, index 0..3; old hosts sending a bare pin target index 0; pin 0xFF = reset that input to its default). Returns status byte |
| 0xE5 | REQ_GET_SPDIF_RX_PIN | IN | Get a SPDIF input's RX pin (wValue = index 0..3, returns uint8_t) |
| 0xE9 | REQ_SET_SPDIF_INPUT_ENABLE | IN* | Enable/disable an optional SPDIF input (wValue = (index<<8)\|enable, index 1..3). Returns PIN_CONFIG_* status byte |
| 0x88 | REQ_SET_I2S_CLOCK_MODE | OUT | Set clock mode (0=master, 1=slave); deferred apply |
| 0x89 | REQ_GET_I2S_CLOCK_MODE | IN | Get live clock mode (returns uint8_t) |
| 0x8A | REQ_GET_I2S_SLAVE_STATUS | IN | Get 16-byte I2sSlaveStatusPacket |
| 0xC2 | REQ_SET_I2S_BCK_PIN | IN* | Set a BCK pin; `wValue = (role<<8)|GPIO` (role 0 = master/unified pair, 1 = slave pair; GPIO 0xFF = reset that role's pair to its default) |
| 0xC3 | REQ_GET_I2S_BCK_PIN | IN | Get a BCK pin; `wValue = role` (0 = master/unified, 1 = slave) |
| 0xFE | REQ_SET_I2S_CLOCK_PIN_MODE | IN* | Set clock-pin mode (wValue = 0 unified / 1 split; returns `PIN_CONFIG_*`); SPLIT enable always validates the slave pair, live apply, deferred input restart when the effective pair moves |
| 0xFF | REQ_GET_I2S_CLOCK_PIN_MODE | IN | Get live clock-pin mode (returns uint8_t: 0=unified, 1=split) |
| 0xED | REQ_SET_INPUT_RATE | OUT | Set I2S input rate (uint32_t Hz: 44100/48000/96000; master mode only; stored-not-applied in slave mode) |
| 0xEE | REQ_GET_INPUT_RATE | IN | Returns 2x uint32_t {current pipeline Hz, selected I2S Hz} |
| 0xEF | REQ_GET_SPDIF_INPUT_CONFIG | IN | Returns 6 bytes: input count (4), enable mask (bit 0 = input 1, always set), GPIOs for inputs 1..4 |
| 0xF1 | REQ_SET_I2S_RX_PIN | IN* | Set I2S RX data pin (wValue=(pair<<8)\|pin, returns status byte; pin 0xFF = reset that pair to its default). Checked against active pairs only |
| 0xF2 | REQ_GET_I2S_RX_PIN | IN | Get I2S RX data pin (returns uint8_t) |
| 0x68 | REQ_SET_ADAT_INPUT_ENABLE | OUT | Enable/disable ADAT input (1 byte 0/1; RP2350 only) |
| 0x69 | REQ_GET_ADAT_INPUT_ENABLE | IN | Get ADAT input enable state (uint8_t) |
| 0x6A | REQ_SET_ADAT_INPUT_PIN | OUT | Set ADAT RX GPIO (uint8_t; 0xFF = reset to default = clear to unset, only while disabled; may equal the ADAT output pin for loopback self-test) |
| 0x6B | REQ_GET_ADAT_INPUT_PIN | IN | Get ADAT RX GPIO (uint8_t) |
| 0x6C | REQ_SET_ADAT_INPUT_CLOCK_MODE | OUT | Set ADAT clock mode (uint8_t 0=master, 1=slave); deferred apply |
| 0x6D | REQ_GET_ADAT_INPUT_CLOCK_MODE | IN | Get ADAT clock mode (uint8_t) |
| 0x6E | REQ_GET_ADAT_INPUT_STATUS | IN | Get 20-byte AdatInputStatusPacket (0x6F reserved) |

*0xE4/0xE9/0xF1 use the immediate-response SET pattern (same as `REQ_SET_I2S_BCK_PIN`).

### Persistence
*Last updated: 2026-10-10 (factory reset switches to USB through the deferred handler)*

- `SLOT_DATA_VERSION` 13 adds `input_source` (uint8_t) to `PresetSlot`
- Slots with version < 13 leave input source at its current value (USB by default)
- Factory reset, an empty-slot load and an active-slot delete queue a deferred switch to USB; they never write `active_input_source` directly
- `WireInputConfig` (16 bytes) section in `WireBulkParams` V7+; wire V12 claims two of its reserved bytes for `i2s_rx_pin` and `i2s_input_rate` (enum 0=44100, 1=48000, 2=96000), same byte size as V11
- SPDIF RX pin stored in `PresetDirectory` (consumed existing padding byte, no directory format change)
- `SLOT_DATA_VERSION` 17 appends `i2s_rx_pin` (0 = unset, use default) and `i2s_input_rate` to `PresetSlot` (struct grows 2 bytes; per-version CRC ranges via `slot_data_size_for_version()`, same mechanism as V16)
- `FlashOutputConfig` claims two reserved bytes for `i2s_rx_pin` (0 = unset) and `i2s_input_rate_p1` (+1 sentinel so old zeroed directories read as "unset" instead of 44.1 kHz), so both survive boot in independent IO mode via `REQ_SAVE_OUTPUT_CONFIG`
- **Multi-SPDIF (SLOT_DATA_VERSION 23 → 24).** `PresetSlot` tail-appends `spdif_rx_enabled_ext` + `spdif_rx_pin_ext[2]` (struct grows 3 bytes). Backward-compatible: older slots load with the extra inputs disabled and their pins unset. Same `output_config_mode` (with-preset vs independent) routing as the existing SPDIF RX pin.
- **Directory (V11 → V12).** `FlashOutputConfig` grows 25 → 28 bytes (adds `spdif_rx_enabled_ext` + `spdif_rx_pin_ext[2]`), so `DIR_VERSION_CURRENT` bumps to 12 with a frozen `FlashOutputConfig_v11` / `PresetDirectory_v11` used by the migration. All-zero new bytes read as disabled/unset.
- **Wire format (WIRE_FORMAT_VERSION stays 17).** `WireInputConfig` claims three of its reserved bytes: `spdif_rx_pin_ext[2]` (0 = absent, keep the live pin) and `spdif_rx_enabled_ext_p1` (enable mask PLUS ONE; 0 = absent), per the reserved-byte claim convention that needs no version break. Bulk apply refuses to disable the live input and validates any newly enabled input's pin (`spdif_input_enable_acceptable()`).
- `SLOT_DATA_VERSION` 28 appends `i2s_clock_mode` (uint8_t; 0=master, 1=slave) to `PresetSlot` (struct grows 1 byte; tail-append with the same per-version CRC-range mechanism, pre-V28 slots read the missing byte as 0=master). `WIRE_FORMAT_VERSION` 21 claims one `WireInputConfig` reserved byte for the same field (size unchanged; pre-V21 readers see a zero=master byte). Restore honors `output_config_mode`: in WITH_PRESET `io_config_from_slot()` resolves the mode (device-global baseline, slot override) with a live-vs-dormant split; when the input is dormant it sets `i2s_clock_mode` immediately, and when I2S is the live source it defers entirely via `pending_i2s_clock_mode` + `i2s_clock_mode_change_pending` (the main loop flips the global and rebuilds I2S clocking atomically). The bulk-apply path uses the same deferred semantics as the vendor path.
- `FlashOutputConfig` grows 1 byte for `i2s_clock_mode` (28 → 29 bytes), bumping `DIR_VERSION_CURRENT` 12→13 with a V12→V13 directory migration (`FlashOutputConfig_v12`/`PresetDirectory_v12` frozen snapshots; the new byte defaults to 0=master); the device-global value survives boot in independent IO mode via `REQ_SAVE_OUTPUT_CONFIG`, applied by `io_config_apply()`.
- `SLOT_DATA_VERSION` 29 appends `i2s_clock_pin_mode` (uint8_t; 0=unified, 1=split) and `i2s_bck_pin_slave` (uint8_t; slave-mode BCK GPIO, 0 = unset → `PICO_I2S_BCK_PIN_SLAVE`) to `PresetSlot` (struct grows 2 bytes; tail-append with the same per-version CRC-range mechanism as V28, `SLOT_DATA_SIZE_V29`). Pre-V29 slots read the missing bytes as 0 (= unified + unset), the correct defaults. Both fields are gated on `slot->version >= 29` in `io_config_from_slot()` and follow `output_config_mode` exactly like the other IO fields (WITH_PRESET resolves device-global baseline then slot override; INDEPENDENT uses the device-global directory value). Unlike `i2s_clock_mode` the pin mode has no pending mechanism: `io_config_apply()` installs it live (after the master BCK, before RX validation, since the RX check reads the live pin mode / slave pin), and arms a deferred input restart only if `i2s_effective_bck_pin()` actually moved while I2S is the live source.
- `WireI2SConfig` claims two of its reserved bytes for `clock_pin_mode_p1` (+1 sentinel, 0 = absent/keep-live; 1 = unified, 2 = split) and `bck_pin_slave` (0 = absent/keep-live); size unchanged, **no `WIRE_FORMAT_VERSION` bump** (stays 21; old hosts send zeros here). Apply validates the RX set against the values that will actually be installed, installs the pin mode live and the slave pin only if acceptable and non-overlapping with the master pair, and arms a deferred restart if the effective input pair moved.
- `FlashOutputConfig` grows 2 more bytes for `i2s_clock_pin_mode` + `i2s_bck_pin_slave`, bumping `DIR_VERSION_CURRENT` 13→14 (29 → 31 bytes) with a V13→V14 directory migration (`FlashOutputConfig_v13`/`PresetDirectory_v13` frozen snapshots; the new bytes default to 0 = unified + unset). The V12→V14 and earlier migrations copy their shorter configs forward with the trailing clock/clock-pin bytes staying 0. The device-global values survive boot in independent IO mode via `REQ_SAVE_OUTPUT_CONFIG`.
- **ADAT input (RP2350).** `WireInputConfig` gains `adat_input_pin`, `adat_input_enabled_p1` (enable + 1; 0 = absent), and `adat_clock_mode_p1` (mode + 1; 0 = absent, 1 = master, 2 = slave), bumping `WIRE_FORMAT_VERSION` to 24. `PresetSlot` tail-appends the same fields (`SLOT_DATA_VERSION` 32; pre-V32 slots load ADAT disabled / pin unset / master) and `FlashOutputConfig` carries them for the device-global path (`DIR_VERSION_CURRENT` 15), honoring `output_config_mode` exactly like the other physical-IO config. RP2040 stores and round-trips the fields but can never select the source.
- **Fourth SPDIF input (2026-08-02).** `SPDIF_RX_NUM_INPUTS` goes 3 → 4. The enable bit is bit 2 of the existing `spdif_rx_enabled_ext` byte everywhere, so only the pin needed new storage. `WireInputConfig.spdif_rx_pin_ext` grows 2 → 3 entries, consuming the section's last reserved byte and shifting `spdif_rx_enabled_ext_p1`, `i2s_clock_mode` and the ADAT input fields down one; the section stays 16 bytes and no later wire offset moves (`WIRE_FORMAT_VERSION` 28 — a V27 host's bulk SET is rejected until rebuilt). `PresetSlot` tail-appends `spdif_rx_pin4` (`SLOT_DATA_VERSION` 35, `SLOT_DATA_SIZE_V35`; V21..V34 slots still load and keep the device-level pin). `FlashOutputConfig` appends `spdif_rx_pin4` too (34 → 35 bytes, `DIR_VERSION_CURRENT` 16) with a V15→V16 migration over frozen `FlashOutputConfig_v15` / `PresetDirectory_v15`; it is a tail byte rather than a third array entry because the frozen v11..v15 configs are strict prefixes the older migrations copy forward, and `cfg_spdif_ext_pin()` / `cfg_spdif_ext_pin_get()` index the split storage as one array. All-zero new bytes read as unset and resolve to `PICO_SPDIF_RX_PIN4_DEFAULT` (GPIO 22).


### I2S Clock-Slave Input Mode
*Last updated: 2026-07-11 (clock-pin mode added: UNIFIED shares one BCK/LRCLK pair, SPLIT gives slave clocking its own `i2s_bck_pin_slave` pair; `i2s_effective_bck_pin()` resolves the active pair)*

`i2s_clock_mode` (0 = master, 1 = slave; persisted, see Persistence above) selects who owns BCK/LRCLK while I2S is the input source. In SLAVE mode an external master drives both pins (they become plain inputs; `gpio_init` + input enable, nothing on-chip drives them) and the device only moves data. The mode is dormant for other input sources. Full spec: `Documentation/Features/i2s_slave_input_spec.md`.

**Clock-pin mode (unified vs split).** A second setting, `i2s_clock_pin_mode` (`I2sClockPinMode` in `audio_input.h`), controls WHICH pins the two clock modes use:

- **UNIFIED (0, default, legacy behavior):** master and slave clock modes share one BCK/LRCLK pair (`i2s_bck_pin` / +1, default GPIO 14/15). The slave pair is fully dormant.
- **SPLIT (1):** master clock mode still drives `i2s_bck_pin`, while slave clock mode listens on a separate `i2s_bck_pin_slave` pair (LRCLK = BCK + 1 in both modes). Defaults: GPIO 12/13 on RP2040, GPIO 26/27 on RP2350 (`PICO_I2S_BCK_PIN_SLAVE`). This lets a board dedicate one pair to its own clock outputs and a different pair to the external master's clock inputs, so a hard-wired external master never contends with the device-driven pair.

The helper `i2s_effective_bck_pin()` resolves the pair the active clock mode actually uses (slave pin only when SPLIT **and** slave clock mode, else `i2s_bck_pin`). Every hardware consumer reads it rather than `i2s_bck_pin` directly: TX `clock_pin_base` in `process_type_switches()` (and its same-type-rebuild change detection), the RX start snapshot in `i2s_input.c` (`i2s_active_bck_pin = i2s_effective_bck_pin()`), and `i2s_input_active_bck_pin()`. In SPLIT mode BOTH pairs count as clock-claimed for RX-data, control-interface, and DAC-mute pin validation (`i2s_clock_pin_claimed()`), because the dormant pair is one deferred mode flip away from being driven or listened on; the two pairs are kept mutually distinct by the SET/apply validators. `REQ_SET_I2S_CLOCK_PIN_MODE` (0xFE) applies the mode live (no pending mechanism). Entering SPLIT always validates the slave pair (distinct from the master pair and free of other owners) even as a dormant store, because a later clock-mode flip adopts the pair without rechecking; leaving SPLIT needs no pin checks (the master pair is clock-claimed in every mode). A live slave-clocked set is additionally rejected with OUTPUT_ACTIVE if I2S output slots run, and otherwise arms a deferred input restart onto the new pair. `REQ_SET_I2S_BCK_PIN` (0xC2) takes `wValue = (role << 8) | GPIO` to address either pair (role 0 = master/unified, role 1 = slave); it restarts the input only when the pair being moved is the one the running input currently uses.

**Clock domains.** I2S output slots are edge-slaved with a dedicated wait-driven PIO program (`audio_i2s_dataout_extclk.pio`, 26 instructions, divider 1.0, `external_clock` instance flag in `pico_audio_i2s_multi`): an LRCLK preamble with a BCK skew guard, then one bit out per external BCK falling edge with per-frame LRCLK framing verification (see "Framing-slip watchdog" below). Like the RX slave programs it is authored with placeholder GPIO indices (wait gpio 0 = BCK, 1 = LRCLK) patched at load (`i2s_extclk_load_program()`, reloaded only on a BCK pin change). Because the checked program is too large to share PIO0's 32-slot instruction memory with the master-clocking programs (clkout 8 + dataout 8 + SPDIF 4), the two clocking modes evict each other's programs at load time (`i2s_extclk_evict_program()` and its mirror in the clkout/dataout load paths); safe because clocking mode is global and a mode rebuild tears down every I2S SM first. Since 2026-10-11 the S/PDIF side takes part in the same budget: while slave clocking is active and any output slot is I2S, `process_type_switches()` selects the 4-instruction NRZI S/PDIF program (software encoder) instead of the default 16-instruction PIO biphase encoder, and a change of that selection rebuilds every S/PDIF slot inside the same transition (see "PIO Program" under SPDIF Output System). With no I2S output slots the PIO encoder stays loaded even under an external master.

**Framing-slip watchdog.** Real-world external masters (USB-I2S bridges such as the Amanero Combo384) glitch or re-frame BCK/LRCLK around stream stop/start and 44.1/48-family rate switches. With free-running slave programs, one runt BCK pulse or LRCLK phase jump would shift the 32-bit word window one bit for the rest of the session; the sign bit lands mid-word (full-scale wrapped garbage) and the word RATE is unchanged, so the rate watchdog can never detect it. Both external-clock programs (`audio_i2s_rx_slave_checked` for RX, `audio_i2s_dataout_extclk` for TX) therefore move/sample the 64 bits per frame by count (deterministic 24-in-32 alignment, autopush/autopull 32) and then verify the LRCLK level at the TWO ADJACENT cells straddling the frame boundary (bit 63 = cell R31 must read HIGH, bit 64 = cell R32 must read LOW, both sampled half a BCK after a rising edge so the pad synchronizers are settled). The adjacent-pair check catches every sub-frame offset (a fixed slip of k cells passes both checks only for k = 0 mod 64); a slip of exactly a whole frame is inherently undetectable from LRCLK (documented spec limit). On a mismatch the program raises **PIO irq flag 7** on its block and re-frames itself at the next LRCLK falling edge, bounding the garbage to ~2 frames; the RX resync path pads the aborted right word (`in null, 1` -> autopush) and the TX path flushes the unsent right-word remainder (`out null, 32`), so L/R word alternation - and therefore channel identity - survives every slip. `i2s_slave_poll()` reads-and-clears both flags (`i2s_slave_slip_check()`: RX block directly, TX blocks via `audio_i2s_extclk_framing_slipped()`) and treats a slip exactly like a clock loss: increment `slip_count` (surfaced in `I2sSlaveStatusPacket`, claims a reserved byte), `i2s_slave_drop_lock()`, and let the main loop's RELOCKING path restart the receiver and re-frame every output through the prefill's gated synchronized start - necessary because a slipped pair/slot is no longer sample-aligned with its peers (inter-slot alignment invariant) and only a full synchronized restart re-establishes that. Stale flags are consumed at every receiver start and by `audio_i2s_enable_sync_prepare()` for extclk instances (the restart IS the handling). The on-chip slave role keeps the plain free-running RX program: our own TX master never glitches, and the checked variant would not fit alongside the clkmaster program. SPDIF, ADAT, and PDM all derive from `sys_clk`, and `i2s_slave_update_clock_servo()` in `i2s_input.c` rate-matches them by calling the shared `input_servo_apply()` (soft-VCXO ppm trim; see "Clock Servo"), referenced to the FIRST SPDIF-type slot as fill reference (an edge-locked I2S slot consumes at exactly the external rate, so its fill can never expose sys_clk error), or the rate term alone (`fill_slot = -1`) when no SPDIF-type slot exists. There are no per-output divider writes and no ADAT servo-divider handoff any more; ADAT and PDM (which formerly did not track in this mode) now follow sys_clk automatically. The I2S slave also gains the centering integrator it never had. MCK output is still forced off in slave mode (a local MCK would be asynchronous to the external clocks); it resumes per its stored config on leaving slave mode.

**Rate detection and lock.** All measurement derives from pair 0's DMA write pointer (2 words per external frame). A ~32 ms fast window snaps to 44100/48000/96000 (2% tolerance); two agreeing windows lock (`I2sSlaveState`: INACTIVE / ACQUIRING / RELOCKING / LOCKED, mirrored to hosts via `REQ_GET_I2S_SLAVE_STATUS` 0x8A and NOTIFY 0x09). A dual-anchor 8-16 s long window refines the servo reference to ~0.1 ppm so ADAT holds without a SPDIF fill reference. 5 ms without words = clock loss (mute and wait; no internal-clock fallback by design); a rate change is a lock drop + re-acquire, feeding the standard deferred `pending_rate` mechanism via `i2s_slave_check_rate_change()`. A poll-gap watchdog (4 ms) re-anchors measurement across long main-loop stalls instead of measuring through a possible ring wrap, and flash brackets suspend/re-arm the input as before.

**Main-loop flow (slave branch of the I2S block).** Mirrors the SPDIF lock-gated flow: `i2s_input_poll()` itself is gated on LOCKED (pre-lock audio may be at the wrong rate and would drain the soft-mute counter early; the lock transition re-anchors every pair's read pointer to a fresh shared frame boundary so the lapped acquisition backlog is never played), prefill is armed only when LOCKED (real input audio fills the pools; the external clocks never stop during the drain, so no silence synthesis is needed), `enable_outputs_in_sync()` + DAC-mute release at the target fill, and a RELOCKING event mutes (unless already muted) and ALWAYS stops/starts the receiver, including while preset_loading is set, so RX state machines that stalled mid-word re-frame when clocks return (the next gated enable re-frames the extclk TX SMs the same way). The prefill enable is additionally gated on LOCKED so a mid-prefill clock loss cannot re-enable against a misframed receiver.

**Alignment.** In slave mode the synchronized output start is LRCLK-gated with prepare/start split enable (see "Synchronized Start" under SPDIF Output System); the extclk program discards exactly one frame (`out null, 32` twice) so the edge-slaved slots land on the same sample index as the SPDIF slots started at the gate edge, making the inter-type offset reset-invariant.

**Role election and transitions.** `i2s_input_should_be_master()` returns false in slave mode and `i2s_input_start()` derives the external role itself (`i2s_role_extclk`); `i2s_input_resync()` is a no-op for the external role (output restarts never glitch the external LRCLK). `process_type_switches()` waits out a freshly armed DAC hardware-mute hold after its own mute assert (instant for pre-gated callers; protects rebuilds that run after a completed reset already released the mute) and treats a same-type I2S slot whose live `clock_master`/`external_clock`/`clock_pin_base` state mismatches the target as a change, which is how clock-mode and input-source transitions rebuild slot clocking without a type diff (`rebuild_i2s_output_clocking()`); its MCK block enforces the slave-mode force-off. Mode changes are deferred (`pending_i2s_clock_mode` + `i2s_clock_mode_change_pending`, handler placed before the input-source switch handler): dormant applies just record the global; a live apply mutes, restarts the input, rebuilds I2S output clocking, restores the selected rate + MCK policy when returning to master mode, and emits the apply-time `PARAM_CHANGED`. The input-source switch handler rebuilds I2S slot clocking whenever a switch crosses the slave-clocked I2S boundary, and the preset-load / preset-delete / factory-reset / bulk-apply flows (and boot) reach the same detection through `apply_restored_outputs()`, which runs `process_type_switches()` over every slot, covering restores that install a new `i2s_bck_pin`, slave pin, or clock-pin mode with an unchanged type map (the mismatch detection compares the instance's `clock_pin_base` against `i2s_effective_bck_pin()`, so a UNIFIED↔SPLIT flip under slave clocking rebuilds the slots onto the newly effective pair). In slave mode the stored master-mode `i2s_input_rate` is fully dormant: the vendor SET, the source-switch target rate, and the preset/bulk restart paths all skip arming it, leaving the detected external rate as the only authority. The restart-path gates check the EFFECTIVE mode (`pending_i2s_clock_mode` when a flip is deferred, else the live global), since a restore can defer a master-to-slave flip in the same apply; the mode handler additionally drops any pending rate change that raced the flip into slave.

**Hardware notes.** The external master must run BCK = 64 x Fs with standard LRCLK polarity (no runtime format detection). Whenever the device is NOT in slave-clocked I2S operation it drives BCK/LRCLK itself if any I2S output or I2S input is active, so a hard-wired external master and the device can transiently both drive the pins across boots and mode/source transitions (documented user constraint; the firmware minimizes the windows). SPLIT clock-pin mode addresses this directly: because master (and every non-slave) role drives `i2s_bck_pin` while the external master feeds the separate `i2s_bck_pin_slave` pair, the two never contend on the same GPIOs; the dual-driver caveat applies only in UNIFIED mode where both share one pair.

### ADAT Input (RP2350 only)
*Last updated: 2026-07-15*

`INPUT_SOURCE_ADAT` (3) is an 8-channel, 24-bit ADAT lightpipe input from one TOSLINK receiver, 44.1/48 kHz only (no SMUX/96k). Disabled by default; selectable only when `adat_input_enabled != 0` AND `adat_input_pin != 0xFF`, and only on RP2350 (RP2040 keeps the config state for wire/preset round-trips but has no PIO/DMA budget for the receiver). Files: `adat_input.c/h`, `adat_input.pio`.

**Receiver architecture.** PIO1 SM2 runs the `adat_rx` NRZI decoder (`adat_input.pio`, 15 instructions) at clock divider 1.0 (full sys_clk, zero divider jitter). Each wire bit cell is counted by a 2-cycle poll loop whose length is set per sample rate via the Y register (cell = 2Y+5 sys cycles: 27 at 44.1 kHz, 25 at 48 kHz at the 307.2 MHz sys clock); no transition within a cell decodes a 0, a transition decodes a 1 and re-anchors the cell grid within 2 sys cycles in both directions, so clock offset of either sign cannot accumulate (an earlier 8-PIO-cycles-per-cell fractional-divider design had a one-way ratchet that deleted bits at 44.1 kHz; see tools/adat_rx_test/adat_rx_bitdiff.c, which models both designs cycle-accurately and proves the current one bit-exact across at least +-1000 ppm source offsets at both rates). The SM emits the decoded bitstream MSB-first, autopushed every 32 bits. DMA channel 15 (the one permanently free channel) streams the words into an 8 KB / 2048-word ring in ENDLESS transfer-count mode with a hardware write-address wrap: a free-running ring with no IRQ and no reload channel. Frame handling is entirely CPU-side in the main-loop poll. ADAT input requires the 307.2 MHz sys clock; at the 150 MHz fallback clock the integer cell quantization is too coarse and the input does not lock.

**Frame sync and loss detection.** The sync header's 10-zero run cannot occur in channel data (ADAT forces a 1 every 5th bit, bounding data runs to 4), so `adat_rx_scan()` searches fresh decoded bits for the 12-bit structural pattern `[1][10x0][1]` (`0x801`) to find the frame boundary. Once found, frames sit at a fixed bit offset (edge resync in the PIO absorbs clock offset, so exactly 256 bits arrive per frame) and `adat_rx_decode_frame()` unstuffs the 8 channel fields (exact inverse of `adat_encode_frame` in adat_output.c) into int32 full-scale. Each frame's header is verified before its samples are trusted; header verification doubles as the loss detector, since a dark or unplugged line decodes as zeros and never matches the header. An isolated bad frame is skipped while the lock holds; `ADAT_HDR_FAIL_LIMIT` (2) consecutive bad headers drop the lock (`slip_count`++).

**Lock machine** (`AdatInputState`): INACTIVE (hardware stopped) -> ACQUIRING (slave: probing exact 48/44.1 kHz decoder timing; master: waiting for a valid device rate) -> SYNCING (candidate header-proven; completing the lock transition) -> LOCKED (decoding audio) -> RELOCKING (signal or rate lost; outputs muted exactly like a SPDIF lock loss). Slave acquisition starts with the 48 kHz cell timing, requires `ADAT_SYNC_VERIFY_FRAMES` = 8 consecutive valid headers at the fixed eight-word frame stride, and alternates to the 44.1 kHz timing after `ADAT_RX_PROBE_DWELL_US` = 10 ms without a valid run (continuing to alternate while absent/unsupported). Re-lock tries the last valid family first. This makes decoded frame structure the family-rate authority, not the DMA rate of a deliberately corrupt wrong-cell stream. The main loop reacts to RELOCKING with the same mute/drain/prefill flow as SPDIF, so the output slots remain aligned through every probe/re-lock transition.

**Clock modes** (`adat_clock_mode`, default MASTER):
- **MASTER:** the far end locks to DSPi's ADAT output, so the return stream is already in our clock domain; no rate detection, no servo. The device is the rate authority via `REQ_SET_INPUT_RATE` (shared with I2S master mode). Above 48 kHz the input parks with `rate_ok = false` (outputs stay muted through a never-completing prefill), mirroring the ADAT output's suspension above 48k. Parking is master-only: `rate_ok == false` always means "master mode above the device-rate ceiling".
- **SLAVE:** external gear owns the clock. Exact-timing header probes select the 44.1/48 kHz family as described above. Only after a candidate is proven does the receiver arm DMA-word measurement: 32 ms fast windows validate the family and supply the initial fine-rate estimate, while a dual-anchor 8-16 s long window supplies a ~0.1 ppm servo reference. All outputs (SPDIF, I2S, ADAT TX, MCK, PDM) track it via `input_servo_apply()`, exactly like SPDIF input; the servo pulls `sys_clk` in ppm (soft VCXO) rather than writing any output divider, so every derived output clock moves together. `adat_input_check_rate_change()` feeds the standard deferred `pending_rate` mechanism when the detected rate differs from `audio_state.freq`. Slave mode is never parked (`rate_ok` always true): if the device rate is above 48 kHz at switch-in, acquisition still reaches LOCKED and the deferred rate change retunes the pipeline under the switch-in mute before the synchronized prefill enables the output slots; the servo holds off until `adat_rx_detected_rate == audio_state.freq`, so no output clock is slewed during that muted window.

**Prefill flow.** On lock the poll batches whole frames (`ADAT_INPUT_MIN_BLOCK` = 48, ~1 ms at 48k, capped at 192/poll), decodes each into the pipeline input buffers (`buf_l`/`buf_r` + `buf_in_ext[0..5]`) with per-channel preamp, and calls `process_input_block()`. A lap guard skips whole frames (preserving frame phase, since the ring holds an exact number of frames) before the reader can be overwritten.

**Loopback self-test.** The RX pin may deliberately equal the ADAT output pin: `adat_input_start()` only sets the pad's input enable and never touches funcsel, so the PIO reads the pad the ADAT TX drives; this is a zero-hardware loopback self-test. `adat_input_stop()` clears input-enable only, safe on the shared pin.

**RP2350 pad isolation.** RP2350 pads reset isolated (`ISO=1` in `PADS_BANK0`), and `gpio_set_input_enabled()` never clears the isolation latch; only `gpio_set_function()` does. A dedicated external RX pad would therefore stay electrically isolated and the PIO would see a static level (same-pin loopback worked only because the ADAT TX `pio_gpio_init()` unisolated the shared pad). `adat_input_start()` clears `ISO` directly with `hw_clear_bits()` under `#if HAS_PADS_BANK0_ISOLATION`, keeping funcsel untouched so the loopback self-test still works even if RX and TX ever sit on different PIO blocks. *Last updated: 2026-07-15*

**Main-loop integration** (`main.c`): the poll + prefill block, the source-switch branches, the deferred clock-mode handler (`adat_clock_mode_change_pending`), boot-into-ADAT, the `adat_input_restart_pending` handler (enable/pin change while ADAT is the live source), and the `perform_rate_change` hook (`adat_input_on_rate_change`) mirror the SPDIF/I2S input plumbing. Status is surfaced by `REQ_GET_ADAT_INPUT_STATUS` (0x6E, 20-byte `AdatInputStatusPacket`) and `NOTIFY_EVT_ADAT_INPUT_STATE` (0x0B) on every state change.

**Flash-write bracket.** `prepare_flash_write_operation()` treats a LOCKED ADAT input as a live source: it appears in the pre-mute drain, the streaming test, and the fade-settle loop (which calls `adat_input_poll()` until both the settle interval and the DAC hardware-mute hold have elapsed), matching the guarantees given to USB/SPDIF/I2S (see "DAC Hardware Mute"). Unlike SPDIF and I2S, ADAT is deliberately NOT stopped for the ~45 ms blackout: its RX is an IRQ-less free-running DMA ring with no decode-timeout alarms to race the blackout edge, and the poll re-acquires frame sync after the ring laps; `preset_loading` (still set from `prepare_pipeline_reset()`) then re-runs the drain/prefill/enable handshake once LOCKED returns.

**Resources.** PIO1 SM2 (PIO1 SM0 = PDM, SM1 = ADAT TX); DMA channel 15 (RX ring, ENDLESS mode, no IRQ). The 8 KB `adat_rx_ring` lives in the shared `input_capture_arena` (see "Input Capture Arena"); the decode hot path (`adat_input_poll`, `adat_rx_header_ok`, `adat_rx_decode_frame`, `adat_rx_rate_machine`, `adat_input_update_clock_servo`) is `DSP_TIME_CRITICAL` and RAM-resident (the acquisition probe/header scan stays cold in flash, since it runs only while muted). See "Memory Layout".

**Persistence.** ADAT input config persists like the optional SPDIF inputs (disabled by default, pin `0xFF` = unset, GPIO claimed only while ADAT is the active source). `WireInputConfig` (V24) gains `adat_input_pin`, `adat_input_enabled_p1` (enable + 1; 0 absent), and `adat_clock_mode_p1` (mode + 1; 0 absent, 1 master, 2 slave). The fields also ride `PresetSlot` (`SLOT_DATA_VERSION` 32) and the device-global `FlashOutputConfig` (`DIR_VERSION` 15), honoring `output_config_mode` exactly like the other physical-IO config. Vendor commands: `REQ_SET/GET_ADAT_INPUT_ENABLE` (0x68/0x69), `REQ_SET/GET_ADAT_INPUT_PIN` (0x6A/0x6B), `REQ_SET/GET_ADAT_INPUT_CLOCK_MODE` (0x6C/0x6D), `REQ_GET_ADAT_INPUT_STATUS` (0x6E); 0x6F reserved.

---

## Shared output divider
*Last updated: 2026-10-11*

Every audio output clock and the input servo's feed-forward reference derive from one value, `audio_base_divider_16_8(fs)` in `audio_clock_div.h`: the 16.8 PIO divider for a 256 × Fs clock, rounded to nearest. S/PDIF TX and ADAT TX run it directly, PDM runs it (`PDM_OVERSAMPLE` is static-asserted to 256), I2S BCK (TX and the I2S RX clock master) runs exactly twice it, and MCK runs base × (256 / multiplier). `input_servo_apply()` references the same value, so its feed-forward folds the divider's rounding residual in.

The rule a future edit must not break: never compute an output divider independently. Independent rounding gives clocks different ppm errors, and a divider ratio error is a rate offset between outputs that the soft VCXO (a common sys_clk trim) cannot correct. At 307.2 MHz the nearest-rounded base equals the earlier ceiling form at every supported rate (44.1, 48, 88.2, 96, 192 kHz), so only MCK at 44.1 kHz changed. The helper and its name match the `overclock` branch.

## Soft VCXO
*Last updated: 2026-10-11 (clock plan read from pll_sys at init; any sys_clk works)*

The single actuator behind input clock tracking (SPDIF input, ADAT slave-clock mode, I2S slave). When the device follows an external audio clock its output clocks must match the source's rate to keep the consumer buffers centred. Previously each output type's PIO divider (and MCK, and ADAT TX) was trimmed independently; that had coarse steps (one 16.8 fractional LSB is 156 ppm at 48 kHz), many actuators to coordinate, and left PDM untracked. The soft VCXO replaces all of it with one knob: it pulls the system PLL by ppm amounts, exactly like warping a crystal. Every output clock (SPDIF TX, internally clocked I2S TX, MCK, ADAT TX, PDM) derives from `sys_clk`, so they all move together and inter-slot sample alignment is preserved by construction; there is no per-output write to get wrong and no divider rounding to coordinate. Files: `soft_vcxo.c/h`. Full spec: `Documentation/Features/soft_vcxo_spec.md`. Status: RP2350 mechanism bench-proven standalone; full-firmware behavior HW-untested; RP2040 fbdiv dithering HW-untested.

**Mechanism.** The PLL has no fractional divider, so one is synthesized by pulse-density modulation of its feedback divider N: DMA briefly writes N + 1 or N - 1, then restores N, at a controlled repetition rate. Each pulse lasts `SOFT_VCXO_PULSE_CYCLES` (150) sys cycles and injects 150 / N sys cycles of phase, which the loop filter smooths; the average frequency offset is proportional to the pulse rate, at sys_MHz × N / 150 Hz per ppm. `soft_vcxo_init()` reads refdiv, N and both post-dividers from `pll_sys_hw` and computes this constant at runtime. At the default plan (XOSC 12 MHz, refdiv 1, N = 128, VCO 1536 MHz, postdiv 5×1, 307.2 MHz) it is 262.144 Hz per ppm; a standalone RP2350 bench measured the tune ratio within 1% of analytic, and since the constant sits inside the servo loop the residual gain error is absorbed by the fill terms (no calibration pass). The static resolution floor is ~0.11 ppm; below that the VCXO parks (FBDIV excursion set to the nominal 128 so the pulses become harmless no-op writes, loop never stopped).

**Topology (zero IRQs, zero steady-state CPU).** Two DMA channels, DMA timer 0, and one PWM slice used purely as a DREQ pacer with no GPIO driven. A PACE channel (DREQ = PWM wrap) re-points and triggers a PULSE channel (DREQ = DMA timer 0), which streams `{excursion, nominal}` into `pll_sys_hw->fbdiv_int` and chains back to PACE; mutual re-arming keeps the loop running forever (both platforms; deliberately avoids the RP2350-only ENDLESS trans-count mode so the code is identical on RP2040). `fbdivs[]` and its address pointer are non-const RAM statics so the pulse train keeps running through flash blackout windows (a DMA read from XIP during a flash op would stall).

**Command path.** `soft_vcxo_set_ppm(float ppm)` (RAM-resident, `DSP_TIME_CRITICAL`) clamps to ±`SOFT_VCXO_MAX_PPM` (300), parks below the floor, sets the FBDIV excursion sign (129 for ppm > 0, 127 for < 0), and programs the PWM pacer (smallest integer clkdiv, up to 255, whose wrap fits 16 bits, `wrap = round(sys / clkdiv / freq)`). Commands below `SOFT_VCXO_PARK_PPM` (0.115 ppm) park; that floor stays within the pacer's reach for any feedback divider of 79 or more. The authority clamp is ±300 ppm; the mechanism could reach ~1000 ppm by raising the clamp and max PWM rate if ever needed.

**Init and bus access.** `soft_vcxo_init()` (called near the end of `core0_init()`) claims the DMA channels / DMA timer / PWM slice and bootstraps the loop. It reads the clock plan from `pll_sys_hw` and stays inert unless XOSC / refdiv × N / (postdiv1 × postdiv2) equals `clock_get_hz(clk_sys)`, i.e. unless clk_sys really is pll_sys at its programmed rate (covers the RP2350 150 MHz boot fallback, where tracked inputs fall back to untracked nominal outputs). A runtime sys_clk change (the `overclock` branch) must stop the pulse train before reprogramming the PLL, because the PULSE channel keeps writing the old N into `fbdiv_int`, and then re-run init. On RP2350 the DMA master cannot reach PLL_SYS by default (ACCESSCTRL resets it to core-only), so init writes `accessctrl_hw->pll_sys = 0xACCE0000 | 0xFF` to admit it; RP2040 has no ACCESSCTRL (APB is open to the DMA master). DMA-to-PLL writes on RP2040 are architecturally fine but not yet hardware-verified.

**Resource map** (see also the "DMA" and "Clock Configuration" comparison tables):

| Resource | RP2040 | RP2350 |
|---|---|---|
| DMA PULSE channel | 10 | 11 |
| DMA PACE channel | 11 | 12 |
| DMA timer | 0 | 0 |
| PWM slice (pacer, no GPIO) | 7 | 8 |
| RAM | ~24 B statics + ~0.3 KB code | ~24 B statics + ~0.3 KB code |

RP2350 slice 8 has no bondable GPIO on RP2350A; RP2040 slice 7 covers GPIO 14/15, and the control-surface PWM LED bind rejects pins on the reserved slice with `CS_STATUS_PWM_CONFLICT` (both platforms). The RP2350 I2S RX input rings were converted from data+reload pairs to single ENDLESS-mode channels (5-8) to free the VCXO's channels (11-12); see "I2S Input" and the "DMA" table.

**Invariants.** Slot alignment is unaffected by design: a sys_clk trim scales every output clock identically and no start/stop sequencing changed. `clock_get_hz(clk_sys)` intentionally keeps reporting the nominal 307.2 MHz (the trim is a closed-loop correction, not a reported frequency; do not "fix" call sites for it). USB (clk_usb from pll_usb) and `time_us_*` rate measurement (clk_ref from XOSC) are independent of the trim, so the servo loop is well-posed. clk_peri (UART) and the flash clkdiv shift by the < 300 ppm trim, orders of magnitude inside tolerance. The servo actuation lives in `input_servo.c` (see "Clock Servo"); this module is just the actuator.

---

## DAC Hardware Mute
*Last updated: 2026-07-25 (hardware mute is now asserted only after the software fade has reached silence, in both `prepare_pipeline_reset()` and the `pipeline_reset_ready()` gate; `dac_hw_mute_release_ms()` getter added so the post-restart fade-up waits out a configured release dwell). Previously, 2026-07-23 (flash-write completion unified on complete_flash_write_operation_full; light path and release_hw_mute_if_outputs_live removed. Previously, 2026-07-14: preset_loading now floored to outlast a pending hold: prepare_pipeline_reset() sizes preset_mute_counter to hold_ms + margin when the hold has not yet elapsed, closing the long-hold expiry cases below. Previously, 2026-07-13: Phase 4 deferral extended to ADAT and SPDIF inputs 2/3; flash-write fade-settle loop services a locked ADAT input)*

Configurable GPIO line that drives a hardware mute pin on an external I²S DAC (PCM5102A XSMT, WM8741 MUTEB, AK4493 SMUTE, etc.). Asserted before `complete_pipeline_reset()` halts the I²S state machine, so the DAC sees its analog output ramp to silence under its own internal control while BCK/LRCLK are still running — eliminating the audible thump that occurs when clocks stop mid-cycle with non-silent data in the DMA buffer. Spec doc: `Documentation/Features/dac_hardware_mute_spec.md`.

**Scope:** hardware pin only. Register-based mute (I²C/SPI) for ES9038Q2M, CS43198, modern AKM is explicitly out of scope — those chips ship with internal soft-mute and Popguard-style protection that mitigates the same problem chip-side.

### Module: `dac_hw_mute.c/h`

Self-contained module owning pin claim, lifecycle, persistence, and notify. Same structural pattern as `lg_sound_sync.c`, `leveller.c`, `crossfeed.c`.

- `dac_hw_mute_init(const DacHwMuteConfig *)` — called once from `core0_init()` after `preset_boot_load()` so the persisted config applies at boot, and again by `dac_hw_mute_set_config()` on every live config change. Idempotent. **Carries an in-flight lifecycle mute across a re-apply:** if a pipeline/flash mute is currently asserted (`s_lifecycle_asserted && s_pin_claimed`) it preserves the asserted state and brings the pin up *muted* under the new config rather than dropping it to the idle level; a same-pin re-apply (the common hold/release-timer edit) is not torn down to high-Z at all. See "Mute-carry invariant" below.
- `dac_hw_mute_set_config(const DacHwMuteConfig *)` — validates (pin range, conflict, no internal duplicates, hold/release range), persists to directory via `preset_set_dac_hw_mute()`, applies live pin claim, emits `WireBulkParams.dac_hw_mute` notify. Main-loop only (blocks ~45 ms for flash write).
- `dac_hw_mute_assert()` / `_release()` — pipeline-reset lifecycle hooks. Assert drives the claimed pin to muted polarity and arms the `hold_ms` deadline; it is **non-blocking** (no busy-wait) and idempotent (re-asserting does not re-arm/extend the hold). Release starts after clocks restart: `release_ms == 0` deasserts immediately, while `release_ms > 0` keeps the pin asserted, records a deadline, and returns.
- `dac_hw_mute_hold_elapsed()` — barrier predicate the caller polls before stopping clocks: true once the armed hold has elapsed (or the feature is off / no hold armed). Lets the assert→clock-stop hold run asynchronously instead of as a main-loop-stalling busy-wait.
- `dac_hw_mute_tick()` — main-loop deadline service for diagnostic test pulses and delayed pipeline releases. When a release deadline expires, it deasserts the pin only if no other mute reason is active.
- `dac_hw_mute_owns_pin(uint8_t pin)` — pin-conflict gate used by `is_pin_in_use()` in `vendor_commands.c`. Other pin-setting commands reject pins this module owns.
- `dac_hw_mute_test_start()` — asynchronous 1-second mute pulse for install verification (`REQ_TEST_DAC_HW_MUTE`).

### Integration with pipeline reset

`prepare_pipeline_reset()` first fades the digital output to silence and flushes it to the wire, then arms the soft envelope (`preset_loading + preset_mute_counter`) for the operation, then calls `dac_hw_mute_assert()`. Order (revised 2026-07-25): the analog mute must start from an already-silent signal, otherwise it ramps a full-level signal down under its own control, truncating the software fade, and the discontinuity reappears when the pin deasserts. `pipeline_reset_ready()` follows the same order — it runs the fade state machine first and asserts the hardware mute only once the fade is complete — so the two layers still cover their different failure modes (data-path discontinuity and analog DC-step on clock cessation) but no longer fight each other. See "Preset-Switch Mute & Pipeline Reset" for the fade machinery.

**The hold is asynchronous — the main loop never busy-waits.** `dac_hw_mute_assert()` only arms a deadline; the hold is enforced at the *clock-stop boundary*, which splits into two cases:
- **Synchronous reset handlers** (preset load, factory reset, bulk params, rate change, stream restart, output-type switch, input-source switch) stop clocks in the same iteration they start. Each gates its body on `pipeline_reset_ready()` — a thin helper = `dac_hw_mute_assert()` + `dac_hw_mute_hold_elapsed()`. While the hold is incomplete the body is skipped and the pending flag is left set, so the loop falls through and keeps servicing audio; the handler retries next iteration. The idempotent assert (no hold re-arm) makes calling the gate every iteration safe. **The gate engages only the hardware mute, never `preset_loading`:** `preset_loading` also triggers the earlier SPDIF lock-acquisition block, and holding it true across the wait would make that block run `drain_and_disable_outputs()` on the same iteration the body re-enables outputs — a double `enable_outputs_in_sync()` with no teardown between, breaking slot alignment. The body's own `prepare_pipeline_reset()` sets `preset_loading` at the proper time (right before teardown), preserving the SPDIF block's original ordering (it reacts on the *next* iteration, after the body's `complete`).
- **SPDIF lock/prefill path** already defers `drain_and_disable_outputs()` (its clock-stop) to a post-lock iteration; it adds `&& dac_hw_mute_hold_elapsed()` to that block's condition so even an instant re-lock still honors the hold. (These Category-B sites assert via `prepare_pipeline_reset()` directly, so they do set `preset_loading` — the block's trigger.)

`perform_rate_change()` and `process_type_switches()` keep their internal `prepare_pipeline_reset()` calls; reached from an already-gated handler these are harmless idempotent re-engages, and their teardown runs after the hold. Flash writes (inherently ≈45 ms IRQ-off blocking) fold the hold into their existing settle loop rather than gating: `prepare_flash_write_operation()` runs its loop until BOTH the settle interval and `dac_hw_mute_hold_elapsed()` are satisfied, servicing the active input (USB ring drain, `spdif_input_poll()`, `i2s_input_poll()`, or `adat_input_poll()`, plus the generator pump) throughout so the output pools stay fed with muted samples. Since 2026-07-25 that loop no longer owns the fade — `prepare_pipeline_reset()` has already faded the wire to silence and flushed the delay lines and consumer queues before it runs — so its remaining jobs are the pre-blackout pool fill and the hold. The loop is entered whenever the active source is live: USB `sync_started`, SPDIF LOCKED, I2S RUNNING, or ADAT LOCKED. (ADAT was originally omitted from this streaming test, the pre-mute drain, and the loop body, so with a locked ADAT input the settle loop was skipped and the blackout could begin before the hold elapsed and before any muted samples reached the outputs — exposing the clock-stop click on preset saves; fixed 2026-07-13.) The boot-time `process_type_switches()` (before the main loop, no audio) proceeds without a hold.

Previously the hold was a `time_us_64()` busy-wait inside `dac_hw_mute_assert()`; it stalled the main loop for up to `hold_ms`, starving the SPDIF in-to-out path and delaying boot-into-SPDIF and input-source switches. The async barrier removes that stall while preserving the exact teardown/synchronized-restart sequence (inter-slot phase alignment unchanged).

`complete_pipeline_reset()` Phase 3.5 holds the software envelope at zero for the post-restart dwell plus `dac_hw_mute_release_ms()`, so the digital fade back up begins after the pin has actually deasserted rather than under a still-muted analog stage.

`complete_pipeline_reset()` adds a Phase 4: after `reset_usb_feedback_loop()`, calls `dac_hw_mute_release()`. Clocks restarted first (Phase 2), then release begins. With `release_ms == 0` (the default), the mute pin deasserts immediately. With `release_ms > 0`, the pin remains asserted until `dac_hw_mute_tick()` observes the deadline; the main loop keeps draining USB/SPDIF audio during the hold, so consumer buffers do not pile up behind a busy-wait.

**I2S, SPDIF, and ADAT inputs defer Phase 4 to their prefill blocks.** Phase 4 is skipped when `preset_loading` is set and the active source is I2S, any SPDIF input (`input_source_is_spdif()`, covering inputs 2/3), or ADAT — each of those sources has a main-loop prefill handshake (gated on `preset_loading`) that, after the reset returns, DRAINS and disables the outputs again, re-enables them in sync, and owns the `dac_hw_mute_release()` (right after its `enable_outputs_in_sync()`). Releasing in Phase 4 would un-mute before that drain stops the clocks; with the default `release_ms == 0` the pin deasserts immediately, fully exposing the clock-stop click. The prefill blocks always run while their source is active and `preset_loading` is set (SPDIF and ADAT hold the mute until lock, the intended mute-until-lock behavior), so the mute is never left stuck asserted. History: the guard originally listed only I2S and SPDIF input 1, so a `complete_pipeline_reset()` with ADAT or SPDIF 2/3 active and `preset_loading` set (full flash-write completion, output pin/type switch, preset or bulk apply) released the mute one iteration before the prefill block's `drain_and_disable_outputs()` stopped the clocks; fixed 2026-07-13 by matching the prefill gates (`input_source_is_spdif()` plus `INPUT_SOURCE_ADAT`, the latter defined on both platforms but never active on RP2040). The companion case is `perform_rate_change()`, which gains a `defer_output_to_input_prefill` argument: callers pass it for I2S bring-up, runtime I2S rate change, and the source-switch *into* I2S, ADAT, or SPDIF (where `active_input_source` is still the old source, so the Phase-4 guard alone could not catch it) to skip `complete_pipeline_reset()` entirely and let the target's prefill block own the restart. USB is unaffected (it has no such prefill drain and releases in Phase 4 as before).

**SPDIF lock-acquisition path also releases the mute.** The USB→SPDIF switch (and boot-into-SPDIF, and SPDIF rx-pin hot-swap, and SPDIF re-lock after lock loss) does NOT call `complete_pipeline_reset()` — output must stay muted until SPDIF achieves lock and the output rings prefill. The lock-acquisition flow in the main loop replicates the relevant phases (`drain_and_disable_outputs()` → wait for lock + prefill → `enable_outputs_in_sync()`), then calls `dac_hw_mute_release()` directly to mirror Phase 4. If `release_ms > 0`, the pin remains asserted while `spdif_input_poll()` continues feeding the output buffers; without this release path, the XSMT pin asserted by the earlier `prepare_pipeline_reset()` would stay asserted indefinitely and the DAC's analog stage would never un-mute.

**`preset_loading` is floored to outlast a pending hold.** The deferrals above all hang off `preset_loading`, but that flag is not a plain latch: the soft-mute envelope (`update_preset_mute_envelope()`, `audio_pipeline.c`) auto-clears it once `preset_mute_counter` samples have been processed, and samples keep processing while a deferred consumer of the flag waits out the hardware-mute hold (`hold_ms` may be up to `DAC_HW_MUTE_HOLD_MS_MAX` = 500 ms). Two arming points enforce the floor (2026-07-14), both gated on `!dac_hw_mute_hold_elapsed()` and sized at `hold_ms` (via the `dac_hw_mute_hold_ms()` getter) + `PRESET_MUTE_HOLD_MARGIN_MS` (120 ms, `flash_storage.h`):
- `prepare_pipeline_reset()` (`main.c`) — closes the re-lock window: the SPDIF/ADAT/I2S-slave RELOCKING handlers arm only `PRESET_MUTE_SAMPLES` = 256 (~5 ms), and after a fast re-lock (or a pin-swap input restart) the source's poll keeps feeding the pipeline while the prefill drain waits on `dac_hw_mute_hold_elapsed()`; with a long hold the flag expired before the drain could run, the prefill handshake never fired, and `dac_hw_mute_release()` was never called (stuck mute). The floor engages only on a fresh hold; the synchronous reset handlers pre-gate on `pipeline_reset_ready()`, so their hold has already elapsed when their body runs prepare, and USB-path mute durations are unchanged.
- `flash_mute_hold_samples()` (`flash_storage.c`) — closes the non-streaming flash window: the flash brackets' fade-settle loop waits the hold out before the blackout, but only when the source is streaming; a flash write with the source dark leaves the hold pending at completion, and `flash_program_staged()`'s trailing re-arm (the last writer of `preset_mute_counter` before completion, ~10 ms un-floored) would let a subsequent fast lock burn the flag before the prefill block could run. The `preset_delete()` active-slot re-arm routes through the same helper. Note the streaming flash path never depended on the prepare-time floor: the settle loop's hold-wait plus the per-write re-arm already guarantee `preset_loading` is set with the hold elapsed at completion.

The flash blackout itself costs nothing against the counter (IRQs off, no samples processed).

**Flash-write completion paths release the mute too.** Every runtime flash write, including the metadata-only writes (preset rename, startup policy, output-config mode/save, master-volume mode/save, control-interface config, Control Surface save, DAC-mute config), completes via `complete_flash_write_operation_full()`; the former light path (`complete_flash_write_operation_light()` and its `release_hw_mute_if_outputs_live()` helper) was removed 2026-07-23 (see "Flash Operation Safety" for the field bug behind this). The completion asserts the mute via `prepare_flash_write_operation()` → `prepare_pipeline_reset()`, and the release follows the standard source split, now owned entirely by `complete_pipeline_reset()` Phase 4 and the input prefill blocks: for **USB input** the mute deasserts inside `complete_pipeline_reset()` (Phase 2 restarted the clocks first, so the deassert is clock-safe); for **SPDIF input** the completion skips the pipeline reset and the lock-acquisition prefill path owns the release after RX re-locks; for **I2S input** and **ADAT input** `complete_pipeline_reset()` runs but Phase 4 defers (`preset_loading` is still set), so the source's prefill block re-enables outputs in sync and owns the release. The old rule documented on `release_hw_mute_if_outputs_live()` (USB releases in the completion; every other source hands the release to its prefill/lock handshake) is unchanged in effect; it simply has one fewer expression now that no completion path leaves the outputs running through the operation, and the 833a51a hazard (a metadata write with EMC on USB input leaving the DAC silent indefinitely) can no longer occur because the USB release lives inside the one shared reset.

### Mute-carry invariant
*Last updated: 2026-07-23 (completion-path reference updated for the unified full flash completion. Previously, 2026-07-04: fix: config re-apply un-muted the DAC mid-flash-write, causing full-volume noise on hold/release-timer edits while streaming)*

`dac_hw_mute_set_config()` is invoked from *inside* the flash-write brackets: the deferred `REQ_SET_DAC_HW_MUTE_CONFIG` handler (`main.c`) and the bulk `REQ_SET_ALL_PARAMS` apply (`bulk_params_apply()` → `dac_hw_mute_set_config()`) both run `prepare_flash_write_operation()` / `prepare_pipeline_reset()` first — which has already **asserted** the mute and waited out `hold_ms` — and only then apply the new config, whose `preset_set_dac_hw_mute()` → `dir_flush()` runs a ~45 ms IRQ-off flash blackout. Historically `dac_hw_mute_init()` unconditionally released the pin and re-drove it to the *un-muted* level (and cleared `s_lifecycle_asserted`) on every re-apply, so the DAC un-muted for that blackout; with audio streaming, the output producer pool underruns during the IRQ-off window and the SPDIF/I²S DMA loops stale buffer content, which the now-live DAC plays as full-volume noise. The trailing `dac_hw_mute_release()` was then a no-op (latch already cleared). Symptoms: noise only while music plays (pool has content to loop), only on DAC-mute-config or bulk edits (other directory setters don't call `dac_hw_mute_init()`), gone after closing/reopening the console (no re-apply). Fix: `dac_hw_mute_init()` carries the asserted state across the re-apply (see the module bullet above), so the DAC stays muted through the config change and its flash write, and the completion path (`complete_pipeline_reset()` Phase 4 / `complete_flash_write_operation_full()`) owns the release exactly as for any other reset.

### Configuration model

`DacHwMuteConfig` (16 bytes, dac_hw_mute.h):
- `enabled` (0/1) — feature gate
- `active_low` (0/1) — assert level polarity (most DACs use active-low: PCM5102A XSMT, WM8741 MUTEB)
- `pin` — single GPIO that drives the DAC's MUTE input; `0xFF` = no pin (feature effectively disabled). One pin only, because `complete_pipeline_reset()` is a global event that disables and re-enables ALL output slots together — per-slot mute pins would give no behavioural benefit. Installations with multiple separate DACs wire their MUTE inputs together to one RP2 GPIO externally; the firmware sees one pin regardless of topology.
- `hold_ms` (1..500) — pre-clock-stop hold after assert, enforced asynchronously (caller polls `dac_hw_mute_hold_elapsed()`; no busy-wait). Sized to cover the DAC's internal soft-ramp at the lowest supported sample rate.
- `release_ms` (0..500) — optional post-clock-restart hold before the mute pin deasserts. Implemented asynchronously from `dac_hw_mute_tick()`, not as a busy-wait.
- `reserved` bytes — zero-fill padding to 16 bytes; NOT earmarked for register-mute (out of scope).

### Persistence (directory, not per-preset)

Board-level attribute. Lives in `PresetDirectory.dac_hw_mute` (V3+). `DIR_VERSION_CURRENT` bumped from 2 → 3 with v2→v3 migration in `dir_load_cache()` (zero-fills the new field — feature off — identical to factory-fresh).

### Vendor commands

| Code | Command                      | Direction | Description |
|------|------------------------------|-----------|-------------|
| 0xEA | REQ_SET_DAC_HW_MUTE_CONFIG   | OUT       | 16-byte `DacHwMuteConfig` payload. Deferred to main loop (`flash_set_dac_hw_mute_pending`); validate + persist + apply. |
| 0xEB | REQ_GET_DAC_HW_MUTE_CONFIG   | IN        | Returns 16-byte live `DacHwMuteConfig`. |
| 0xEC | REQ_TEST_DAC_HW_MUTE         | IN        | Triggers ~1 s mute pulse for installer verification. Returns `PIN_CONFIG_*` status. |

### Wire format

`WireDacHwMute` (16 bytes, byte-for-byte compatible with `DacHwMuteConfig`) in `WireBulkParams` V10+. Bulk apply funnels through `dac_hw_mute_set_config()` — same validation as the vendor-command path.

### Memory / CPU cost

| Item | RP2040 | RP2350 |
|------|--------|--------|
| `dac_hw_mute.c/.h` text | ~5 KB | ~4.6 KB |
| BSS (live config + pin-claimed + flags + async deadlines) | ~40 B | ~40 B |
| Flash directory growth | +16 B | +16 B |
| Wire format growth | +16 B (V9 → V10) | same |
| Audio-path overhead (enabled or disabled) | **0 cycles** in inner DSP loop | 0 |
| Pipeline-reset overhead (enabled) | + `hold_ms` busy-wait; `release_ms` is asynchronous | same |

The audio-path zero-overhead is critical: the inner DSP loops never see this feature. All work happens in the pipeline-reset handler, which fires on lifecycle events only — never per-packet.

---

## LG Sound Sync
*Last updated: 2026-05-10*

LG Sound Sync (optical) is a one-way side-channel that LG televisions multiplex onto their TOSLINK output: specific bytes of the IEC 60958 channel-status field carry the TV's current volume (0–100) and mute state. When DSPi locks an LG-Sound-Sync-marked SPDIF source and the feature is enabled, the TV remote becomes a host-volume control for DSPi. Spec doc: `Documentation/Features/lg_sound_sync_spec.md`.

### Why this drives host volume (not master volume)

Loudness compensation is keyed off the *raw user-perceived* vol_index (the same one `db_to_vol[]` indexes), not the device-side master ceiling. Driving host volume keeps the SPL/loudness loop coherent: lowering the TV vol drops SPL and the loudness EQ retunes its equal-loudness contour for the new reference. If Sound Sync drove master volume instead, loudness would compensate against a stale reference and over-emphasise bass + treble at low TV volumes.

### Module: `lg_sound_sync.c/h`

A single self-contained module owns detection and application. Public surface:

- `lg_sound_sync_init()` — boot-time RAM reset (does not touch the user-loaded enable flag).
- `lg_sound_sync_tick()` — main-loop tick, internally throttled to one channel-status poll every 50 ms. Cheap on the disabled / non-SPDIF path.
- `lg_sound_sync_set_enabled(bool)` / `lg_sound_sync_get_enabled()` — user gate.
- `lg_sound_sync_get_status(LgSoundSyncStatus *)` — IRQ-safe snapshot of `enabled`, `present`, `volume` (0..100 or 0xFF sentinel), `muted`.
- `lg_sound_sync_on_input_source_change(uint8_t)` — main.c hook; demotes to absent on switch away from SPDIF, re-arms streaks on switch into SPDIF.
- `lg_sound_sync_on_preset_loaded()` — flash_storage.c hook; resets streaks so detection re-evaluates against the freshly loaded `enabled` flag.

### Protocol decoding (LSB-first c_bits[24])

Two byte-position layouts are supported. `LG_LAYOUTS[]` in `lg_sound_sync.c` carries both; `lg_match_layout()` tries each in order and returns the first match (or NULL). Adding a third for a future model variant is a one-struct-literal change. Per-layout cost is 3 byte comparisons short-circuited, evaluated once per 50 ms — negligible.

| Layout | Signature `F` | Signature `04` | Signature `8A` | Vol high nibble | Vol low nibble | Source |
|--------|---------------|----------------|----------------|-----------------|----------------|--------|
| New (HiFiBerry) | `cs[16] & 0x0F == 0x0F` | `cs[17] == 0x04` | `cs[18] == 0x8A` | `cs[15] & 0x0F` | `(cs[16] & 0xF0) >> 4` | OLED55C9-era |
| B7-era (mirror) | `cs[7] & 0x0F == 0x0F`  | `cs[6] == 0x04`  | `cs[5] == 0x8A`  | `cs[8] & 0x0F`  | `(cs[7] & 0xF0) >> 4`  | 2017 OLED B7 |

The B7 layout is a true byte-position mirror around the middle of the 24-byte block (`N ↔ 23-N`); nibble layout *within* each byte is unchanged. Empirically verified at TV vol = 3 and 26.

When a layout matches, volume/mute decode the same way against that layout's offsets:

```c
uint8_t vol_byte = ((cs[L->vol_hi] & 0x0F) << 4) | ((cs[L->vol_lo] & 0xF0) >> 4);
bool    muted    = (vol_byte & 0x80) != 0;
uint8_t volume   = vol_byte & 0x7F;   // 0..100, clamped at 100
```

### Detection state machine

Asymmetric hysteresis on consecutive-poll streak counts:

| Threshold              | Polls | Time   | Rationale                                                                                                                  |
|------------------------|-------|--------|----------------------------------------------------------------------------------------------------------------------------|
| `LG_PRESENT_THRESHOLD` | 3     | 150 ms | Fast rise — user gets responsive control as soon as Sound Sync starts. Below human "instant" perceptual threshold.        |
| `LG_ABSENT_THRESHOLD`  | 10    | 500 ms | Slow fall — single corrupted CS block or brief signal hiccup must not snap vol_mul back to USB-cached value mid-listening. |

The tick early-exits on `(!enabled || active_input_source != SPDIF || !LOCKED)` and demotes to absent on each. Only when locked and enabled does it actually read `c_bits[24]` via `spdif_input_get_channel_status()` and feed the streak counters.

Once present, **every** signature-positive poll re-decodes and re-applies (not just the rising edge), so `lg_sound_sync_on_preset_loaded()` can reset streaks without freezing vol_mul for 150 ms while it re-acquires.

### Apply path — option 2 (LG drives user volume directly)

LG drives the user-facing volume directly: `audio_state.volume`, `vol_mul`, `current_loudness_coeffs`, and the `WireUserVolume.user_volume_db` notify all move together. The host UI's main volume widget tracks TV remote presses with no special-case binding — it just listens to the existing user-volume notification.

Implementation funnels through `update_user_volume(db)` in `usb_audio.c` — the same single funnel used by `REQ_SET_USER_VOLUME` and the bulk-params apply path. That funnel writes `audio_state.volume`, calls `apply_vol_index_to_audio()` (vol_mul + loudness coeffs), invalidates the LG apply-cache, and pushes the user-volume notify.

LG vol → vol_index mapping is proportional with rounded division: `vol_index = (lg_vol × 60 + 50) / 100`. dB = `vol_index - CENTER_VOLUME_INDEX`. Endpoints land cleanly (LG 0 → silent, LG 100 → unity) and intermediate steps approximate 1 dB each, matching `db_to_vol[]`'s shape.

Mute drives `user_mute` (the vendor mute, OR'd with `audio_state.mute` in the audio pipeline). `s_lg_imposed_mute` tracks whether the *current* `user_mute` was set by LG vs. by the user via `REQ_SET_USER_MUTE` — used on demote to clear LG-imposed mute (so the user isn't stuck silent if the TV stops broadcasting) while preserving a manual mute the user set before LG took over.

**No thaw cache** on demote / SPDIF→USB. `audio_state.volume` stays wherever LG last set it. The OS may re-issue UAC1 SET_CUR with its remembered per-device volume on enumeration / default-device-change events; for DSPi-internal input switches the user's slider position simply picks up from LG's last value. This trade-off is documented in spec §1.3 — the dual-widget alternative was rejected as more confusing than the occasional volume-stays-where-LG-left-it surprise.

Coalescing: the LG poll fires every 50 ms but the user only changes volume on remote presses. `apply_lg_state` skips `update_user_volume()` when the new vol_index matches `s_last_applied_vol_index`. Without this, the host would receive 20 redundant user-volume notifies per second of TV silence. `update_user_volume()` invalidates `s_last_applied_vol_index` to `-1`; we immediately re-establish the cache to the value just written so subsequent matching polls coalesce.

### Persistence (per-preset)

The `enabled` flag lives in `PresetSlot` (V14), not `PresetDirectory`. This matches the per-preset treatment of every other "what does the audio path do here" toggle (loudness, leveller, crossfeed, master EQ bypass). Different listening profiles can want different Sound Sync behavior — a "Headphone" preset may not want TV vol takeover, a "TV Listening" preset wants it on.

- `SLOT_DATA_VERSION` 14 adds `lg_sound_sync_enabled` (uint8_t) to `PresetSlot` (with 3 bytes of trailing padding).
- Pre-V14 slots default to the firmware constant `LG_SOUND_SYNC_DEFAULT_ENABLED` = 0 — non-LG users see no behavior change after firmware update.
- `apply_factory_defaults()` resets the live flag to the firmware default; the slot's stored value is unchanged (factory reset does not rewrite the active slot).
- `WireLgSoundSync` (16 bytes) section in `WireBulkParams` V8+; bulk SET honors only `enabled`, the runtime fields are read-only.

### Vendor commands

| Code | Command                       | Direction | Description                                              |
|------|-------------------------------|-----------|----------------------------------------------------------|
| 0xE6 | REQ_SET_LG_SOUND_SYNC_ENABLE  | OUT       | Set the enable flag (uint8_t payload). Live-only — flash persists on `REQ_SAVE_PRESET`. |
| 0xE7 | REQ_GET_LG_SOUND_SYNC_ENABLE  | IN        | Get the enable flag (returns uint8_t).                  |
| 0xE8 | REQ_GET_LG_SOUND_SYNC_STATUS  | IN        | Get the full 16-byte `LgSoundSyncStatus` struct.        |

### Notifications

Standard `NOTIFY_EVT_PARAM_CHANGED` events on the WireBulkParams offset of each changed field (`enabled`, `present`, `volume`, `muted`). Field-granular so host UIs can subscribe at any granularity; the notify-ring's coalesce stage collapses rapid LG-vol changes naturally.

### Output-slot alignment

The feature touches only `audio_state.vol_mul` and `current_loudness_coeffs`. It does **not** call `complete_pipeline_reset()`, `prepare_pipeline_reset()`, or any DMA / PIO / pool reset. Output slot alignment is preserved across every Sound Sync transition (enable/disable, present/absent, volume/mute change, input source switch, preset load, factory reset) per the CLAUDE.md hard constraint.

---

## TinyUSB Migration (Phases 1 + 2)
*Last updated: 2026-04-18*

Phase 1 swapped the USB library from pico-extras `usb_device` to TinyUSB with full UAC1 audio parity. Phase 2 brought the vendor control interface back under TinyUSB. MS OS 2.0 descriptors for WinUSB auto-binding are still deferred (Phase 2b) — on Windows the host app must bind WinUSB manually (e.g. via Zadig) until that lands. macOS and Linux need no extra binding.

### Why a custom UAC1 class driver

TinyUSB's built-in audio class driver (`lib/tinyusb/src/class/audio/audio_device.c`) hard-rejects any AC interface whose `bInterfaceProtocol` is not `AUDIO_INT_PROTOCOL_CODE_V2` (UAC2, 0x20) at `audiod_open():1576`. UAC1 uses `bInterfaceProtocol = 0x00`, so the built-in driver cannot claim our interface. Rather than patch vendored SDK code, DSPi registers its own minimal UAC1 class driver via TinyUSB's application-driver mechanism (`usbd_app_driver_get_cb`). Our driver's `.open()` callback implements the same descriptor walk + endpoint allocation flow as TinyUSB's audio driver but without the UAC2 protocol check.

### What lives where

| Area | File | Notes |
|------|------|-------|
| TinyUSB configuration | `tusb_config.h` | `CFG_TUD_AUDIO = 0` and all other classes off. The vendor interface is also handled by our custom driver (not `CFG_TUD_VENDOR`), because our vendor interface is control-transfer-only with no bulk endpoints. |
| UAC1 descriptors | `usb_descriptors.c` / `usb_descriptors.h` | Hand-rolled byte array (no LUFA). Layout: config (9B) → IAD (8B) → AC std itf + CS (49B) → AS alt 0/1/2 (125B) → vendor std itf (9B). Total 200B. Feature unit entity 2 exposes master mute + volume. |
| Class driver | `usb_audio.c` | `uac1_driver` struct is registered as the single app driver. Implements `init`/`reset`/`open`/`control_xfer_cb`/`xfer_cb`/`sof`. The same driver claims AC+AS (via IAD) AND the vendor interface 2 (class 0xFF). |
| Vendor command dispatch | `vendor_commands.c` | All existing SET/GET handlers preserved. Public entry point is `tud_vendor_control_xfer_cb(rhport, stage, req)` — TinyUSB's weakly-linked global callback. TinyUSB routes **every** vendor-type control transfer here directly from `process_control_request` (usbd.c:727-730), bypassing class drivers. A `vendor_send_response()` shim wraps `tud_control_xfer()` so case bodies stay unchanged. Bulk SET/GET (`REQ_SET_ALL_PARAMS` / `REQ_GET_ALL_PARAMS`) use `tud_control_xfer()`'s native EP0 chunking instead of the old `usb_stream_setup_transfer` plumbing. |
| USB init | `usb_audio.c:usb_sound_card_init()` | Calls `tud_init(0)` in place of the pico-extras `usb_interface_init()` / `usb_device_init()` / `usb_device_start()` block. |
| Main loop | `main.c` | `tud_task()` is called once per iteration before `usb_audio_drain_ring()`. |
| SOF feedback servo | `usb_audio.c:uac1_driver_sof()` | Replaces the former `usb_sof_irq()` in `main.c`. Runs in USB IRQ context (TinyUSB dispatches SOF-consumer callbacks synchronously from `dcd_event_handler`). |

### Context change for the audio RX path

Under pico-extras, `_as_audio_packet()` ran in USB IRQ context on every audio OUT packet completion. Under TinyUSB, `DCD_EVENT_XFER_COMPLETE` events are enqueued by the DCD IRQ and dispatched to our `uac1_driver_xfer_cb` from `tud_task()` (main-loop context). The SPSC ring is unchanged; the producer moved from IRQ to task. Gap detection timestamps still come from `time_us_32()` captured in `xfer_cb` — noise is bounded by the main-loop polling rate (~kHz), well below the 2 ms gap threshold. SOF still runs in IRQ, so feedback servo latency is unchanged.

### Descriptor layout (UAC1 + vendor + notifications, byte offsets into `usb_config_descriptor[]`)

| Offset | Length | Contents |
|--------|--------|----------|
| 0 | 9 | Configuration descriptor (total length 207) |
| 9 | 8 | IAD grouping AC + AS (bInterfaceCount = 2) |
| 17 | 9 | AC std interface (itf 0, 0 EPs, UAC1 protocol 0x00) |
| 26 | 9 | AC CS header (bcdADC 0x0100, bInCollection 1) |
| 35 | 12 | AC CS input terminal (ID 1, USB streaming, 2 ch L|R) |
| 47 | 10 | AC CS feature unit (ID 2, master MUTE|VOLUME, 2 logical ch) |
| 57 | 9 | AC CS output terminal (ID 3, generic speaker) |
| 66 | 9 | AS std interface alt 0 (zero-bandwidth) |
| 75 | 58 | AS alt 1 (16-bit, 44.1/48/96 kHz) incl. std + CS data EP 0x01 and feedback EP 0x82 |
| 133 | 58 | AS alt 2 (24-bit, 44.1/48/96 kHz) incl. std + CS data EP 0x01 and feedback EP 0x82 |
| 191 | 9 | Vendor std interface (itf 2, class 0xFF, 1 EP) |
| 200 | 7 | Std bulk EP IN 0x83 (notifications, 8 B) |

The vendor interface sits **outside** the IAD — it is its own USB function. TinyUSB's `process_set_config()` calls our `open()` a second time with the vendor interface descriptor; we recognize class 0xFF, claim it, and open the notification endpoint. See "Notification Interrupt Endpoint" below for the push channel that rides on EP 0x83.

**Why the IAD is required:** TinyUSB's `process_set_config()` in `usbd.c` binds interfaces to class drivers based on `bInterfaceCount` — and defaults to 1 when no IAD is present. Without the IAD, TinyUSB would bind only the AC interface (itf 0) to our UAC1 class driver, leaving the AS interface (itf 1) unbound. `SET_INTERFACE` requests for AS would then fail at `_usbd_dev.itf2drv[1] == DRVID_INVALID`, the isochronous endpoints would never open, and the device would fail to appear as a functional audio endpoint on the host. The IAD makes TinyUSB bind both interfaces (itf 0 + itf 1) to our driver in a single `open()` call.

### Control request handling

UAC1 uses discrete `bRequest` opcodes (`SET_CUR` 0x01, `GET_CUR` 0x81, `GET_MIN` 0x82, `GET_MAX` 0x83, `GET_RES` 0x84) that are *not* exposed by TinyUSB's `audio.h` (UAC2 uses a single `RANGE` opcode instead). They are defined in `usb_descriptors.h` as `UAC1_REQ_*`. `uac1_driver_control_xfer_cb()` dispatches on them directly:

- **Feature unit (interface recipient, entity 2):** MUTE + master VOLUME via the existing `audio_state` + `audio_set_volume()` path.
- **Sampling frequency (endpoint recipient, EP 0x01):** SET_CUR writes `audio_state.freq` and raises `rate_change_pending`. `perform_rate_change()` in `main.c` runs in the main loop as before.

### What is gone in Phase 1

- `vendor_commands.c` / `vendor_commands.h` — not compiled. `derive_core1_mode()` was the only non-vendor helper inside; it has been moved into `usb_audio.c`.
- `firmware/DSPi/lufa/` — no longer on the target's include path. Folder retained on disk.
- `PICO_USBDEV_USE_ZERO_BASED_INTERFACES` / `PICO_USBDEV_MAX_DESCRIPTOR_SIZE` / `PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE` compile definitions.
- MS OS / WCID descriptors + `device_setup_request_handler` WCID dispatch.
- All vendor commands (0x42 … 0xD5). The host configuration app will not function until Phase 2.

### Phase 2 status (done) and Phase 2b (deferred)

Done in Phase 2:

- Vendor interface (class 0xFF, 0 endpoints) re-added to the config descriptor at itf 2 (outside the AC+AS IAD).
- `vendor_commands.c` adapted: public entry point is `vendor_control_xfer_cb(rhport, stage, req)`, invoked from our UAC1 class driver's `control_xfer_cb` when a vendor-class request targets the vendor interface. A legacy `vendor_buffer_t` shim and a `vendor_send_response()` wrapper keep all 30+ SET/GET case bodies unchanged.
- `REQ_GET_ALL_PARAMS` / `REQ_SET_ALL_PARAMS` (3664 bytes at V11) now use `tud_control_xfer()`'s native EP0 chunking — the old `usb_stream_setup_transfer` / `_vendor_stream` / `_vendor_*_complete` plumbing is gone.
- `REQ_GET_USB_ERROR_STATS` / `REQ_RESET_USB_ERROR_STATS` return zeros / no-op under TinyUSB (pico-extras' per-category error counters have no TinyUSB equivalent yet).

Deferred to Phase 2b:

- MS OS 2.0 descriptors (BOS + platform capability UUID `D8DD60DF-4589-4CC7-9CD2-659D9E648A9F`) for automatic WinUSB binding on Windows. Until this lands, Windows hosts must bind WinUSB manually (e.g. via Zadig). macOS and Linux work without any additional binding.
- Resurface a meaningful USB error counter path if/when TinyUSB adds DCD-level error event hooks.

### Size impact

| Platform | text (pre-migration) | text (Phase 1) | text (Phase 2) | bss (Phase 2) |
|----------|-------------------:|---------------:|---------------:|--------------:|
| RP2350 | 89,720 | 80,812 | 91,240 | 210,696 |
| RP2040 | n/a | 84,844 | 95,612 | 90,704 |

Phase 1 removed the vendor surface entirely (~9 KB saved). Phase 2 re-added it (~10.5 KB), and also added the IAD (+8 bytes) and the vendor interface descriptor (+9 bytes). Net vs. pre-migration on RP2350: +1.5 KB text.

---

## Test Signal Generator
*Last updated: 2026-07-05*

The **test signal generator** ("siggen", `siggen.c` / `siggen.h`) synthesizes
measurement and diagnostic signals directly into the output pipeline, without a
host audio stream. It is used for room-correction sweeps, polarity and
channel-identification checks, THD/IMD and inter-sample-peak tests, and bring-up
diagnostics, driven from DSPi Console or any control bridge (USB / UART / I2C).
Full protocol: `Documentation/Features/test_signals_spec.md`.

**Injection point.** `siggen_render()` writes the generated signal into the
per-output mix buffers (`buf_out[]`) inside `process_input_block()`, **between the
matrix-mix pass and the per-output processing pass**. Masked output channels have
their routed audio replaced; unmasked channels keep playing program audio. Every
output slot still advances by the same `sample_count`, so **inter-slot alignment is
preserved by construction** (this is a hard project invariant). Downstream stages
(crossover, PEQ, output trim, master volume, mute, delay, encode) run unchanged.

**Signal catalogue.** Fifteen `SiggenType` values: sine, square (polyBLEP), white
and pink noise, log/linear/stepped sweeps, impulse, alternating clicks, polarity
pulse, tone burst, tone pair (SMPTE/CCIF IMD), multitone (Schroeder-phased,
sum-normalized; max 16 tones RP2350 / 8 RP2040), ISP inter-sample-peak patterns
(+3.01 / +1.25 dBTP), and channel-ID (per-channel pentatonic blip melody, always
walks). `level_db` is a peak level in dBFS (-120..0).

**Architecture.** A per-block segment planner (state machine: fade-in, run, gap,
fade-out, and cycle/repeat/walk sequencing) advances all timing once per block and
emits `[offset, length, gain-ramp, active-mask]` segments; per-type synth kernels
then fill each segment. Sequencing is platform-independent; only the sample kernels
fork between the RP2350 float path and the RP2040 Q28 fixed-point path. The sine
kernel is a 7th-order polynomial (THD approx -139 dB). Rate-dependent constants are
precomputed at apply time for 44.1/48/96 kHz, so the render path makes no `libm`
calls (and no float math on RP2040); any other pipeline rate falls back to the
48 kHz row.

**Flags.** `SIGGEN_FLAG_RAW` bypasses only crossover + PEQ on generator channels
(trim, master volume, mute, delay still apply) via `siggen_raw_mask`, which is read
once per block by the per-output EQ gating on both cores (`audio_pipeline.c`,
`pdm_generator.c`); the bypassed filter states freeze while RAW is active.
`SIGGEN_FLAG_DECORR` gives noise types an independent generator per channel.
`SIGGEN_FLAG_WALK` plays masked channels one per cycle. A channel emits only if it
is both in `channel_mask` and enabled in the matrix mixer (enabled-intersection
rule); `invert_mask` flips polarity per channel for cancellation tests.

**Fades.** Start/stop/config-swap use a 5 ms linear overlay fade
(`SIGGEN_FADE_MS`) that covers the first/last cycle rather than delaying it, so
sweep timing is exact. Sweeps and walked continuous signals additionally get
per-cycle attack/release windows, and per-cycle synth state resets so repeated
sweeps are bit-identical (needed for coherent averaging in room correction).

**Pump.** When the generator is running and no source is streaming, each input
delivery point (`pipeline_service_producer()` in `main.c`, twice per main-loop pass)
calls `siggen_pump()` (`audio_pipeline.c`), feeding zero-input blocks through the
full pipeline paced by the slot-0 consumer fill level (top-up to the ring target fill). It
stands down while a source streams, an input-source change or output-type switch is
pending, or before `usb_sound_card_init()` has set up the slots, so a real stream
seamlessly takes over pacing.

**Lifecycle.** Transient only: off at boot, never persisted; `siggen_stop_immediate
(SIGGEN_STOP_PRESET)` hard-stops it on preset load and factory reset. `main.c`
calls `siggen_service()` once per main-loop pass and `siggen_pump()` at each input
delivery point (2026-10-23);
`siggen_service()` applies staged configs after a fade-out (SET/START while
running restarts) and pushes the deferred `NOTIFY_EVT_SIGGEN_STATE` (0x07)
start/stop/completion notification. Vendor commands `0xA4..0xA8` cover
SET/GET config, CONTROL (start/stop/stop-now), GET status, and self-describing GET
caps; `0xA9..0xAF` are reserved.

**Memory / CPU.** Small BSS (under 1 KB) plus RAM-resident render kernels
(`DSP_TIME_CRITICAL`); control/config code stays cold in flash. The only cross-core
datum is `siggen_raw_mask`, written by Core 0 between blocks.

---

## Spectrum Analyser (RTA / FFT)
*Last updated: 2026-10-24 (FFT, FINISH and PUBLISH run as a background job in 16-unit and 16-bin chunks); 2026-10-23 (FINISH sliced and PUBLISH split out, for main-loop latency); 2026-10-20 (applied configurations announced with NOTIFY_EVT_RTA_CONFIG); 2026-09-12 (shared elliptic b0 and RP2350 -O3 bass kernel; Blackman-Harris FFT window; sixth-order bass bands; continuous bass bank and V3 protocol; FFT ceiling lowered to 1024 points; 2026-09-07: single transform, bass stream and bass bank removed)*

One FFT engine (`rta.c`, kernel in `rta_fft.c`, generated tables in
`rta_tables.h` from `scripts/gen_rta_tables.py`) that can be pointed at any set
of channels on the input side (after the per-input PEQ) or the output side
(after gain and delay, exactly what the slots transmit). It rotates through the
selected channels one frame at a time, keeps the latest third-octave band table
per channel, and publishes the latest frame's raw bins. Transient only: off at
boot, never persisted, not in presets or the bulk-params blob, and it switches
itself off after 5 s without a data read. Full protocol and design:
`Documentation/Features/spectrum_analyser_spec.md`. Status: **HW-untested**
(kernel verified against a numpy oracle, engine against a stubbed host build).

**Taps and alignment.** Both taps are one call, `rta_tap()`, inside the existing
per-channel peak-meter loops: the input meter loop in PASS 2 (plus the
upmix-derived-row meter loop), and every output meter loop on both cores,
which sit after gain and delay and before `output_block_to_s24_inplace()`.
The tap copies FFT samples and continuously updates the selected channel’s bass bank, then writes back its copy count and bass timing; it changes no sample and no sample count, so inter-slot alignment is
untouched by construction.

**Per-packet view.** Core 0 writes `rta_view` (`RtaPacketView`: tap, row, write
index, samples to take) in `rta_packet_begin()` at the top of
`process_input_block()`, ahead of the `work_ready` barrier, so both cores use
one view per packet. The owning core's tap copies to the snapshotted index and
writes back `produced`; Core 0 reads it in `rta_packet_end()` after the
`work_done` join and alone advances the index and changes phase. Every Core 0
pipeline entry (USB ring drain, S/PDIF, I2S, and ADAT polls, siggen pump) and
every vendor handler runs from the main loop, so `rta_service()` never
pre-empts `process_input_block()`.

**Frame machine.** `FILL` (taps copy) to `FFT` (prescale, radix-2/4 stages,
permutation, real split, resumable through `RtaFftCursor`) to `FINISH`
(frequency-domain Blackman-Harris, bin powers, band sums, level bytes, resumable
through `rta_fft_finish_step()`) to `PUBLISH` (band EMA/peaks, bin frame header,
advance) and then the next live channel. Since 2026-10-24 FFT, FINISH and PUBLISH
run as the analyser's background job (Output Latency "Step budget and background
jobs"), in chunks of `RTA_FFT_UNITS_PER_CHUNK` units and `RTA_FINISH_BINS_PER_CHUNK`
bins until each step's deadline; `rta_service()` keeps config apply, fill
liveness and auto-off.
Before 2026-10-23 FINISH and publish ran as one call of about 530 µs on RP2350,
which was the longest main-loop pass in the firmware (600 µs worst pass with the
analyser running against 70 µs without) and halved the low-latency option's
margin. Sliced at 64 bins and one butterfly stage per call it was about 210 µs,
mostly an order-10 radix-4 stage (about 140 µs) or the PUBLISH; the job model
splits stages by butterfly and keeps PUBLISH at the start of a step. The FINISH steps
write the bin levels into a staging buffer (`bin_stage`, 512 bytes) and PUBLISH
copies it into the bin frame together with the header and tail. Vendor reads
are served on the same main loop, so a read never sees a half-written frame.
Writing the frame in place across the steps, as first done, left its tail
invalid for eight passes per frame and failed `rta_bin_frame_seq_consistency`
under a heavy configuration. *Updated 2026-10-23.* `rta_service()` sits next to `siggen_service()`
in the main loop. Per-channel refresh interval = (fill + transform) x live
channel count; at 1024 points and 48 kHz that is 21 ms per channel, 171 ms
with eight channels live. If the channel being filled stops being live (USB
alt change, disabled output, upmixer parked) the main loop moves on, so a frame
never stalls.

**Products.** Hybrid third-octave bands (34 at 44.1/48 kHz, 37 at 96 kHz; 10–200 Hz from the continuous bank, higher bands from the FFT; DC bin in no
band; a band with no bin at the current size is empty, reads 0, and
`RtaStatus.first_band` is 0 because the bass bank reaches 10 Hz; FFT gaps above 200 Hz at small sizes remain empty) with power-domain EMA
averaging and peak hold whose decay time is carried between publishes; and the
raw bins of the latest frame. Levels are one byte in 0.5 dB steps with 243 =
0 dBFS. First FFT bin-containing band at 48 kHz: 200 Hz at 256 points, 100 Hz at 512,
50 Hz at 1024, which is the ceiling. 2048 points would reach 25 Hz and is not
offered: the capture buffer is always sized for the largest order, so it would
cost another 4,608 B of BSS on RP2350.

**Continuous bass bank.** `rta_bass.c` consumes every sample of every selected
live channel, including other channels' FFT turns. CIC3 /8 (44.1/48 kHz) or
/16 (96 kHz), followed by an eighth-order elliptic low-pass and /8, produces
689.0625/750 Hz streams. The elliptic sections' zeros lie on the unit circle,
so b2 equals b0: the tables store b0 once and the kernel reuses the b0*x
product, one multiply fewer per section at 6 kHz with bit-identical output.
Fourteen sixth-order Butterworth bandpass bands
(three sections each, generated with prefix gain normalisation) cover exact
base-10 centres 10–199.526 Hz. Unsigned modular CIC arithmetic avoids floating-point
integrator drift; the decimated kernel uses Q27 states/Q28 coefficients with
64-bit intermediates and Q48 power on RP2040, float on RP2350. Coefficients
are generated by `scripts/gen_rta_bass.py`, then copied to RAM at configuration.
Each owning core updates its channel only; Core 0 snapshots the bass mask and
clears entering/leaving channels before dispatch, then reads powers after the
join. Start/restart/RESET_AVG discard bass history. No audio alignment or reset
rules change.

The bass power EMA replaces duplicate per-frame EMA storage and uses the longer
of one frequency period or `avg_ms`. Results and peak hold publish on the
existing FFT rotation. Host tests with `avg_ms=0` measure 90% power response in
590/300/50 ms at 10/19.953/199.526 Hz, independent of selected channel count;
publication adds up to one ordinary FFT rotation. All 14 centres at all three
rates pass float64-oracle, -60/-70 dBFS, hot-input, DC, alias and selectivity
checks. Tested aliases are suppressed by more than 85 dB. A tone reads at least
40 dB down one octave away and 60 dB down two octaves away. The earlier
single-biquad bands leaked only about 16 dB at one octave, so a bass tone lit
every band. Calculated pink-noise power is 0.1–0.2 dB above disjoint
third-octave integration. The sections triple band-filter work at 689/750 Hz.
The 6 kHz elliptic stage is unchanged, so per-channel bass multiplies rise by
roughly a third. Raw FFT bins remain unchanged and receive no synthetic bass bins.

**V3 protocol.** `RTA_MAX_BANDS=37`; band frames grow to 82 B, centre indices
shift by three, and the whole-Hz caps table begins 10/13/16/20. Config/caps/
status/bin-header sizes remain 12/16/24/16 B. Caps exposes `bass_bands=14` and
`bass_dynamic_range_db=70`; existing dynamic range refers to FFT only. Status
adds `bass_busy_us_per_s` in its final formerly-reserved uint16: summed elapsed
bass tap time on both cores normalised to one second, saturated at 65535 us.
Clients must negotiate V3 and derive frame strides/centres from caps. The
in-repo test client is updated; Console is a separate repository and needs a
matching release. No vendor command IDs were added.

**Bin frame protocol.** `REQ_RTA_GET_BINS` is chunked by byte offset with no
lock. The frame is `16 + n_bins + 1` bytes with the sequence number in the
header and again as the last byte; the engine clears the tail to 0xFF before
writing and sets it last, and sequence numbers skip 0xFF, so a torn read can
never validate. The layout is fixed for the life of a config.

**Restart and auto-off.** `rta_restart()` is called from `perform_rate_change()`,
the input source switch, preset load, and factory reset, and `rta_service()`
also restarts itself if the device rate changes under it. Any band or bin read
is a keepalive and auto-starts the analyser; `REQ_RTA_GET_STATUS` is not.
`RTA_FLAG_MANUAL` hands the run state to `REQ_RTA_CONTROL` entirely (no
auto-start, no auto-off).

**Configuration announcements (2026-10-20).** The analyser has one
configuration shared by every host (USB, DSPi Link sessions through a hub, a
UART or I2C bridge). `apply_config()` pushes `NOTIFY_EVT_RTA_CONFIG` (0x0D)
once per applied SET, even when nothing changed, carrying the clamped and
masked `RtaConfig` exactly as `REQ_RTA_GET_CONFIG` would return it. The source
is captured at stage time (`rta_set_config(payload, len, src)` stores it in
`cfg_next_src`) because the apply runs later on the main loop. Restarts from
preset load, rate change, input switch or factory reset, auto-off, auto-start
and `REQ_RTA_CONTROL` never emit it, since none of them changes the
configuration. Hosts use the event to follow another host's configuration and
to confirm their own push without polling; see analyser spec section 7.

**RAM placement.** `rta_tap`, `rta_packet_begin`, `rta_packet_end`, `rta_bass_push` and the live-mask helper are
`DSP_TIME_CRITICAL` (they run inside the meter loops on both cores, including
during flash writes). The checker roots and their transitive call closure
verify their RAM placement. Generated source tables remain in flash; the
active bass coefficient copy resides in RAM. FFT and control paths stay in flash.

**Kernel accuracy (host harness, `tools/rta_test/run.sh`, orders 8 to 10).**
Full-scale sine within 0.062 dB in its band; bins within 0.255 dB (float) and
0.232 dB (Q15) of a numpy Blackman-Harris FFT; pink noise flat within 0.72 dB; Q15 dynamic
range 78.5 dB, float 121.5 dB (the wire floor). The shared level converter now uses multiplication rather than left-shifting a negative exponent, removing undefined C behavior without changing levels. A tone on a band edge splits
its window main lobe across two bands, so each reads up to 3.01 dB low while the
pair sums correctly. A -60 dBFS tone reads up to 1.24 dB high in Q15 at 1024
points (tolerance 1.25), because the band sums the per-bin Q15 floor across all its bins.

**FFT window (updated 2026-09-12).** The window is 4-term Blackman-Harris,
applied after the transform as a 7-tap kernel with conjugate-symmetric bins
past DC and Nyquist, scaled so bin levels keep the Hann normalisation. Hann's
slow sidelobe decay leaked loud 20-100 Hz tones into the first FFT bands at
-40 to -55 dB, a fixed hump next to the much cleaner bass bank. Engine
simulation at 1024 points now puts that leakage at -98 dB or lower (float).
The wider main lobe makes the lowest FFT band worse at 512 and 256 points and
reads a centred tone 3.02 dB low in one-bin bands. Q15 window products stay
under 1.50e9 and bin power is squared unsigned. FINISH does about twice the
per-bin work; on-device timing is unmeasured.

**CPU.** The transform is main-loop work and does not appear in `cpu0_load`;
`RtaStatus.busy_us_per_s` and `last_frame_us` report it instead. Host
operation counts put a 1024-point Q15 frame at about 110k cycles (1.7% of an
RP2040 core at the frame rate); the bench figure is pending. Bass filtering is synchronous audio work and scales with live channels; its new timer is separate from FFT timing, and existing audio CPU meters also include it. On-device CPU, deadline and physical alignment validation remain pending.

**Vendor surface.** Commands 0x08 to 0x0F (see the Vendor Command Reference).
`REQ_RTA_GET_BANDS_ALL` is USB-only and built in `bulk_param_buf` under the bulk
lock, which is now keyed at all three sites through `vendor_is_bulk_get()`.
Device tests: `tools/dspi_test/tests/rta.py` (group `rta`).
