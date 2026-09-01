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
*Last updated: 2026-07-05*

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
| `lg_sound_sync.c` | LG Sound Sync detection state machine + apply path (drives host volume from LG-decoded TV remote) |
| `lg_sound_sync.h` | LG Sound Sync API, status struct, default constant |
| `i2s_input.c` | I2S RX integration: master/slave PIO lifecycle, IRQ-less DMA ring, poll into pipeline |
| `i2s_input.h` | I2S input API (start/stop/resync/poll), state enum |
| `i2s_input.pio` | I2S RX PIO programs (clock-master and wait-driven slave variants) |
| `input_capture_arena.h` | Shared capture-buffer union (SPDIF FIFO / I2S rings / ADAT ring) + ownership claim/release API |
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
- `PICO_AUDIO_SPDIF_DMA_IRQ=1`
- `PICO_AUDIO_I2S_DMA_IRQ=0`

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

---

## Initialization Flow
*Last updated: 2026-03-17*

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
*Last updated: 2026-04-18*

**Library:** TinyUSB (vendored via Pico SDK) with a custom UAC1 class driver (`usb_audio.c`, registered via `usbd_app_driver_get_cb`). TinyUSB's built-in audio class driver is UAC2-only and is bypassed. See "TinyUSB Migration (Phase 1)" for details.

**Error handling:** TinyUSB's USB IRQ handler receives bus-level error interrupts and increments internal counters. In Phase 1, the vendor-command hooks for `REQ_GET_USB_ERROR_STATS` / `REQ_RESET_USB_ERROR_STATS` are unreachable (vendor interface dropped); Phase 2 will re-expose these once the vendor interface is wired into TinyUSB.

**Interfaces (Phase 1):**
1. **Audio Control (AC)** — Interface 0
2. **Audio Streaming (AS)** — Interface 1
   - Alt 0: Zero-bandwidth (idle)
   - Alt 1: 16-bit PCM, 2 channels (44.1/48/96 kHz), wMaxPacketSize=582
   - Alt 2: 24-bit PCM, 2 channels (44.1/48/96 kHz), wMaxPacketSize=582
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
*Last updated: 2026-07-11*

Any host-driven format change — SET_INTERFACE between AS alts (bit-depth switch) or SET_CUR on the endpoint sampling_freq control (rate switch) — must land on a muted, drained pipeline. Otherwise old-rate/old-bit-depth audio still queued in the consumer pools plays out against the new PIO divider or gets decoded under the wrong bytes-per-frame assumption, producing an audible pitch shift or byte-scramble burst.

**`uac1_apply_alt()` (usb_audio.c):**
- **Idempotent early-return.** `SET_INTERFACE(alt=current)` is common from host driver probes and used to tear down / re-open iso endpoints for no reason. Now skipped.
- **Bit-depth switch (alt 1↔2) is treated the same as a cold start (alt 0→>0).** Both paths engage the mute envelope inline (`preset_mute_counter = PRESET_MUTE_SAMPLES`, `preset_loading = true`) so any packet decoded between the SETUP ack and the main-loop's `complete_pipeline_reset()` is silenced. Both paths also raise `stream_restart_resync_pending`, reset the feedback controller (`fb_ctrl_stream_stop` + `feedback_10_14 = nominal_feedback_10_14`), and clear `sync_started` / `total_samples_produced` so gap detection and the feedback loop resume from a deterministic baseline.
- The pre-existing ring flush on `bit_depth_changed` still runs; combined with the mute, stale packets cannot reach the DSP pipeline in the wrong format.

**SET_CUR sampling_freq validation:** unsupported rates are now stalled at EP0 rather than silently committed. Previously any 24-bit value was accepted — `audio_state.freq` would store garbage that `perform_rate_change()` later coerced to 44100, so a subsequent GET_CUR returned a rate the device never actually applied. The accepted set is {44100, 48000, 96000} to match the Type-I format descriptors on both alts.

**USB rate ownership:** `usb_selected_rate` is the UAC1 playback endpoint's host-selected rate, while `audio_state.freq` is the rate actually applied to the live DSP pipeline. SET_CUR and the fixed-48-kHz multichannel alts always update the USB selection, but arm a live rate change only while USB is the active input; GET_CUR returns the retained USB selection. A later source switch into USB applies that selection, so decorative USB control traffic cannot interrupt S/PDIF or I2S playback and the host does not need to resend its rate when USB becomes active. Two races are closed inside the source-switch handler: `rate_change_pending` is cleared right after `active_input_source` flips (a SET_CUR armed for the old source in the same loop iteration would otherwise fire under the new source; mirrors the clock-mode flip handler's clear), and the switching-to-USB tail re-checks the retained selection against `audio_state.freq` and re-arms if a SET_CUR landed in USB IRQ context between the branch reading the selection and the source flip.

**`perform_rate_change()` (main.c):** bracketed with `prepare_pipeline_reset(PRESET_MUTE_SAMPLES)` / `complete_pipeline_reset()`. Without the bracket, the SPDIF `wrap_consumer_take` callback updates the PIO divider lazily on the next buffer-take, so old-rate audio already queued in each consumer pool plays out at the new bit-clock — audible pitch wobble for ~16 ms. `complete_pipeline_reset()` aborts DMA on every enabled slot, drains the consumer pool back to the free list, and restarts all outputs in sync at the new divider. The I2S `audio_i2s_update_all_frequencies()` call inside `perform_rate_change()` still runs for its own divider+clkdiv_restart pass; the subsequent `complete_pipeline_reset()` re-aborts/re-enables idempotently and costs only microseconds.

**Nominal SPDIF divider restore (`restore_nominal_spdif_dividers()`, main.c):** the input clock servos (SPDIF input, I2S slave) trim the SPDIF TX SM dividers directly and never update the library's `inst->freq` bookkeeping, so the lazy `wrap_consumer_take` update is blind to the trim and never fires when the pipeline rate value is unchanged. `perform_rate_change()` therefore restores nominal eagerly via `audio_spdif_apply_pio_frequency()` (new public wrapper in the SPDIF library) even at an unchanged rate, and the two equal-rate paths that skip `perform_rate_change()` — the source-switch-away-from-I2S branch and the slave-to-master clock-mode flip — call the restore explicitly. Without this, a switch away from a servoed input at an unchanged rate left the trim in place while ADAT resynced to nominal (its resync reads the input servo dividers, which report 0 once the input is stopped), and the nominal-vs-trimmed split made ADAT drift against the slots it mirrors until its slip machinery forced periodic corrective resyncs. All SPDIF slots receive the same divider, so inter-slot alignment is unaffected. *Last updated: 2026-07-11*

### Multichannel USB Input + Per-Input EQ/Metering (RP2350)
*Last updated: 2026-07-10 (crossfeed moved to per-output-pair stage; no longer bypassed in multichannel)*

**Unified channel model (no "master").** Channels are now `[inputs 0..NUM_INPUT_CHANNELS-1][outputs NUM_INPUT_CHANNELS..]`; `NUM_CHANNELS = NUM_INPUT_CHANNELS + NUM_OUTPUT_CHANNELS` (17 on RP2350, 7 on RP2040). The former "master EQ" was always the EQ on the 2 stereo inputs — it is generalized so **every input channel is a first-class channel with its own 10-band PEQ + peak/clip metering** (no crossover). Output channels keep PEQ + crossover + gain/delay/mute. `CH_OUT_1 = NUM_INPUT_CHANNELS`; the output-slot→channel-index mapping (`CH_OUT_1 + slot`) shifts automatically, so the slot-aligned output path is structurally unchanged. **Input EQ and metering reuse the existing `REQ_SET/GET_EQ_PARAM` (channel = input index) and `REQ_GET_STATUS` (`peaks[]`/`clip_flags`) commands — no new commands.** `clip_flags` is `uint32_t` (17 channels > 16 bits).

**USB formats.** RP2350 advertises AudioStreaming alts **1 = 2ch/16, 2 = 2ch/24** (44.1/48/96 kHz), and **3 = 4ch, 4 = 6ch, 5 = 8ch** (all 48 kHz/16-bit fixed). The host picks the format; the firmware sets the active input count (`usb_input_channels` = 2/4/6/8) in `uac1_apply_alt()`, which forces 48 kHz for multichannel alts (SET_CUR rejects non-48k), flushes the ring, and resyncs. RP2040 is stereo-only (2 SPDIF = 4 output channels). The multichannel alt blocks are emitted by a parameterized `AS_MULTICH_ALT(alt,nch)` macro; descriptor offsets/`CONFIG_TOTAL_LEN` (369 on RP2350) are arithmetic + `_Static_assert`-verified. The single Input Terminal/Feature Unit declare the 8-channel superset (`wChannelConfig=0x063F`); iso OUT max-packet `AUDIO_EP_MAX_PKT=788`.

**Decode + pipeline.** `process_audio_packet()` deinterleaves N channels (stride = `channels`) into `input_bufs[c]`. The 24-bit stereo path casts the packet buffer to `uint32_t*` and the compiler may emit `ldrd` (which faults on a non-word-aligned address), so the USB ring slot's `data[]` is `__attribute__((aligned(4)))` (`usb_audio_ring.h`); macOS opens a stereo device at 24-bit, so this path runs even in "2-channel" mode. (inputs 0/1 → buf_l/buf_r, 2-7 → buf_in_ext) with per-input preamp. In `process_input_block()` the **input-EQ pass** runs `dsp_process_channel_block(filters[k], input_bufs[k], …)` for each active input `k`, then meters its post-EQ peak/clip into `global_status.peaks[k]`/`clip_flags`. `n_active_inputs` = `usb_input_channels` for USB else 2. **Crossfeed no longer sits on the input bus; it runs per output pair post-matrix (PASS 4.5), so it is input-count agnostic and never bypassed in multichannel** (see "Crossfeed"). (Loudness no longer sits on the input bus; it runs per output post-gain, mask-selected, in both stereo and multichannel modes; see "Loudness Compensation".) The **volume leveller now runs channel-count-agnostic over the active inputs** (mask-driven; see "Volume Leveller"), so it is no longer bypassed in multichannel. The matrix iterates `n_active_inputs`, so `buf_in_ext` is read only when those inputs are active; inactive input peaks are zeroed. Inter-output sample alignment is preserved bit-for-bit (the leveller delays every active input identically through its per-channel lookahead ring). Default factory routing is stereo pass-through; multichannel routing is configured by the host app.

**Live active-input-count for the host.** The app reads the active count (2/4/6/8) from the `REQ_GET_STATUS` combined packet (a 1-byte `active_input_channels` field, polled with the meters) or the scalar `wValue==23`; the bulk header `num_input_channels` stays the device max (8). Both report sites are **source-aware** via `active_input_channel_count()` (`audio_pipeline.c`), the single source of truth shared with the DSP pipeline's input dimension: USB → `usb_input_channels`, I2S → `i2s_input_channels`, S/PDIF → 2. A `NOTIFY_EVT_INPUT_FORMAT` (0x05) push event fires whenever the active count can change — USB alt change, input-source switch, and live I2S channel-count change (`REQ_SET_I2S_INPUT_CHANNELS` while I2S is active) — so the app relayouts immediately regardless of source.

**Persistence (compat-breaking).** Wire `WIRE_FORMAT_VERSION=16` (direct 8-input matrix/preamp + 17-channel EQ; no tail-append/version gates; 5864 B). Flash `SLOT_DATA_VERSION=21` (direct layout; the slot spans **2 flash sectors** on RP2350; 1 on RP2040). No migration — pre-version data loads factory defaults.

### Notification Endpoint (device→host push)
*Last updated: 2026-07-13 (NOTIFY_EVT_ADAT_INPUT_STATE 0x0B added)*

The vendor interface carries one **bulk IN** endpoint (EP 0x83, wMaxPacketSize = 64) for out-of-band device→host notifications. The transport runs two protocol versions in parallel: v1 (8-byte `MASTER_VOLUME` packets, kept for existing host apps) and v2 (generic `PARAM_CHANGED` + discrete events, the primary protocol going forward). `USB_BCD_DEVICE = 0x0201` so Windows re-reads descriptors after the 8→64 byte EP bump.

See `Documentation/Features/notification_protocol_v2_spec.md` for the full protocol specification.

**Why bulk rather than interrupt:** an earlier draft used an interrupt IN endpoint at 4 ms polling. Under heavy EP0 control-transfer traffic (rapid `REQ_SET_MASTER_VOLUME` from a slider drag in the host app) the RP2040/2350 USB controller crashed after ~20–40 s and the device re-enumerated. Switching the EP to bulk IN eliminates the crash: bulk uses opportunistic host scheduling rather than a fixed bInterval poll cadence. The v2 protocol preserves the bulk transport.

**v2 core design (`notify.c/notify.h`):** every parameter is identified by its `offsetof` into `WireBulkParams`. A single event ID (`NOTIFY_EVT_PARAM_CHANGED = 0x02`) carries `(wire_offset, wire_size, source, value)`. Host dispatch is a flat lookup on offset, not a hand-written switch — adding a parameter requires zero wire-format changes.

**Subsystem state:**
- `param_shadow`: mirror of `WireBulkParams` (3664 B BSS at V11). `notify_param_write` compares writes against it; notifications only fire on real byte-level changes.
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

**Discrete event IDs** on this transport: `NOTIFY_EVT_PARAM_CHANGED` (0x02), `NOTIFY_EVT_BULK_INVALIDATED` (0x03), `NOTIFY_EVT_PRESET_LOADED` (0x04), `NOTIFY_EVT_INPUT_FORMAT` (0x05), `NOTIFY_EVT_SIGGEN_STATE` (0x07), `NOTIFY_EVT_ADAT_STATE` (0x08), `NOTIFY_EVT_I2S_SLAVE_STATE` (0x09, 9-byte packet `[ver=2, 0x09, flags=0, seq, state, rate_LE32]`; pushed on every I2S clock-slave lock-state transition), `NOTIFY_EVT_CS_IR_LEARN` (0x0A), and `NOTIFY_EVT_ADAT_INPUT_STATE` (0x0B). Siggen announces test-signal-generator start/stop/completion as an 8-byte packet `[ver=2, 0x07, flags=0, seq, state, reason, signal_type, channel]` (state = `SiggenState`, reason = `SIGGEN_STOP_*`, channel = walk channel or 0xFF); pushed from `siggen_service()` in the main loop, never from the render path (see "Test Signal Generator"). CS_IR_LEARN announces IR learn completion as a 12-byte packet `[ver=2, 0x0A, flags=0, seq, state, protocol, 0, 0, code_LE32]` (state = `CS_IR_LEARN_DONE`/`_TIMEOUT`), pushed from the Control Surfaces tick. ADAT_INPUT_STATE (RP2350) announces every ADAT input lock-state transition as a 10-byte packet `[ver=2, 0x0B, flags=0, seq, state, rate_LE32, clock_mode]` (state = `AdatInputState`; rate = 0 unless LOCKED), pushed from the ADAT RX poll (see "ADAT Input").

**Drain:** each consumer drains its own tail via `notify_peek_next_for(consumer, ...)` / `notify_commit_pop_for(consumer)`. The USB consumer (`usb_notify_drain` in usb_audio.c) claims EP 0x83 via `usbd_edpt_claim`, formats the next packet into the stable TX buffer, and submits via `usbd_edpt_xfer`; on success `notify_commit_pop_for(USB)` advances the USB tail, and on xfer rejection the entry stays queued for the next tick. The UART consumer drains from `uart_ctrl_poll` (see "Multi-consumer ring").

**Multi-consumer ring:** the ring supports independent consumers, each with its own tail: `NOTIFY_CONSUMER_USB` (always active) and `NOTIFY_CONSUMER_UART` (active while the UART transport is live with `notify_enable` set). `notify_consumer_set_active(c, active)` snaps a newly activated consumer's tail to `head` so it sees no stale backlog. A lagging consumer never stalls the producer or another consumer: if a push would collide with a consumer's tail, that consumer's oldest entry is force-dropped (counted in `notify_consumer_drops[c]`) and the push proceeds; the consumer detects the loss as a seq gap and re-reads `REQ_GET_ALL_PARAMS`. Sequence numbers are stamped at **push** time and stored in the ring entry (not allocated at drain time), so a re-peek or xfer retry does not burn a number; this removed a historical phantom-gap artifact for USB hosts where a rejected transfer had allocated but not delivered a seq. The v1 legacy master-volume entry carries no seq, consumes none, and is delivered to the USB consumer only (every v1 event has a v2 twin, which the UART consumer receives instead). The old displace-oldest-on-full path for `BULK_INVALIDATED` is gone; force-advance guarantees delivery for every event type.

**Initialisation:** `notify_init()` is called from `core0_init()` after `preset_boot_load()` so `bulk_params_collect(&param_shadow)` sees a fully-populated live state. The control-interface init block (UART / I2C) is now **deliberately placed after** `notify_init()` in `core0_init()`, so the UART bring-up's notify-consumer activation is not wiped by the consumer-table reset inside `notify_init()`. The USB reset path (`uac1_driver_reset`) calls `notify_reset_queue()`, which resets **only** the USB consumer's view (drops its backlog) and the source/bulk brackets; the global seq counter and other consumers' backlogs are left intact (resetting seq would fake a wrap-around gap for a mid-stream UART consumer).

**v1 back-compat:** `update_master_volume` still pushes an 8-byte `MASTER_VOLUME` (0x01) event into the ring. Existing v1 host apps that only recognise byte 0 = 0x01 continue to work; v2 hosts receive the parallel PARAM_CHANGED event and dispatch by offset.

### Volume & Mute
*Last updated: 2026-05-17 (UAC1 SET_CUR volume now emits PARAM_CHANGED tagged PARAM_SRC_UAC1)*

**Volume range:** -60 dB to 0 dB (1 dB resolution, 61 steps). Bottom step is fully silent. USB Audio Class 8.8 fixed-point dB encoding. Q15 lookup table (`db_to_vol[61]`) maps dB index to linear multiplier; index 0 = 0x0000 (silent), index 60 = 0x7FFF (unity).

**User volume — multiple owners, one source of truth:** `audio_state.volume` (and `audio_state.mute`) hold the user-perceived volume/mute. Three controllers can drive these fields, all funneling through the same `apply_vol_index_to_audio()` helper so `vol_mul` and the loudness coefficient pointer always move in lock-step:

1. **UAC1 host slider/mute** (`audio_set_volume()` / UAC1 Feature Unit MUTE) — primary controller while USB is the active input source. `audio_set_volume()` early-returns its apply path when `active_input_source != INPUT_SOURCE_USB`, so a host adjustment during SPDIF playback is cached but inert. UAC1 SET_CUR VOLUME emits a `PARAM_CHANGED` on `user_volume.user_volume_db` with source `PARAM_SRC_UAC1` (distinct from `PARAM_SRC_HOST_SET`, which tags `REQ_SET_USER_VOLUME` from EP0) — same field, different source byte, so vendor-channel host UIs track OS slider movements live and can attribute them to the OS rather than themselves. **Important edge case:** the emit also fires when the host writes during non-USB input (where the value is cached but audibly inert) — the notify field semantically tracks "OS slider position", not "what the listener is hearing", and the source byte lets a host disambiguate UAC1 vs LG writes if it needs both. UAC1 Feature Unit MUTE does NOT emit (no parallel WireBulkParams field — see "Mute" below).
2. **LG Sound Sync** (`lg_sound_sync.c`) — owns `vol_mul` and re-keys the loudness pointer while SPDIF input is active and an LG TV is producing optical TC values.
3. **Vendor channel** (`REQ_SET_USER_VOLUME` / `REQ_SET_USER_MUTE`) — `update_user_volume()` always applies, regardless of input source, so an external control surface or app can drive user volume during non-USB playback. Caller-side conventions (e.g. "Console only writes during non-USB input") are not enforced by the firmware. While LG Sound Sync is locked, its next ~20 ms poll will overwrite the vendor write — that's intentional ownership during LG-driven playback.

**Mute (two flags, different gating):**
- `audio_state.mute` — UAC1 Feature Unit MUTE control. **USB-gated:** the OS mute key has no audible effect when the active input is not USB, so the host can't accidentally silence SPDIF playback. `audio_state.vol_mul` itself is already frozen at the last USB-active value because `audio_set_volume()` bails before touching it when source != USB.
- `user_mute` — vendor-channel mute (`REQ_SET_USER_MUTE`). **Always honored, no input-source guard** — symmetric with `REQ_SET_USER_VOLUME`'s always-apply contract. An external control surface that mutes via the vendor channel expects audio to actually go silent.

The pipeline ORs them: `muted = (audio_state.mute && host_active) || user_mute`. Either flag forces the per-packet target gain to zero; the apply then ramps to zero over the per-sample envelope used for volume changes (see below), so mute toggles are click-free regardless of which flag triggered them. `REQ_GET_USER_MUTE` returns `user_mute` only — UAC1's mute remains queryable via UAC1 GET_CUR (each surface owns its native interface).

Note that `audio_state.volume` does NOT need a parallel field for the same reason — the volume value itself is source-agnostic; the gating lives inside `audio_set_volume()`'s apply path, not on the field.

**Persistence:** User volume IS saved per-preset as of `SLOT_DATA_VERSION` = 15 — stored as a single-byte `user_vol_index` (range [0, CENTER_VOLUME_INDEX]) consuming the last V14 padding byte. Restore on preset load funnels through `update_user_volume()` (so `vol_mul`, the loudness coefficient pointer, the LG apply-cache invalidation, and the v2 notify all run via the single helper). Pre-V15 slots leave user volume UNTOUCHED on load — asymmetric vs master volume which falls back to a directory value, but there is no directory-level fallback for user volume; the user wasn't expecting that legacy preset to set their listening level. `user_mute` is NOT persisted (session-only, cleared on factory reset) — matches the existing user-mute design contract.

**Per-sample output volume ramping (click-free transitions):** The composite output gain — host volume × `preset_mute_gain` × master volume — is linearly interpolated within each input packet from the previous packet's ending value (`vol_mul_master_prev`, file-scope state in `audio_pipeline.c`) to the new target. Both Core 0 and Core 1 receive the same `vol_mul_start` / `vol_mul_step` pair via `Core1EqWork` and apply identical per-sample ramps to their respective outputs, preserving inter-slot phase alignment (CLAUDE.md hard rule). When `vol_mul_step == 0` (steady state, no host adjustment in flight) the gain loops fall through to the original constant-gain branches (memset for zero, scalar multiply for non-unity, no-op for unity) — no per-sample overhead. RP2350 carries the ramp in float; RP2040 keeps it as Q15 int32 to avoid float→int conversion in the inner loop. The mechanism also smooths preset-mute transitions and master-volume changes, since they all funnel through the same composite gain. Zero-length packets do not advance `vol_mul_master_prev`, so a stray empty packet between two real packets cannot eat a pending volume delta.

**Scope of click-free guarantee:** This ramp covers the output gain stage only. With loudness compensation enabled, `audio_set_volume()` swaps `current_loudness_coeffs` to a new table entry on every host volume step, and the SVF/biquad filter state continues from the previous coefficients — producing a small frequency-response transient at each step. With loudness disabled (or when the user does not change volume), the path is fully click-free; with loudness enabled, the broadband gain click is gone but a faint per-step zipper artifact can remain on signal-rich content.

### Asynchronous Feedback Endpoint
*Last updated: 2026-04-18*

The device declares itself as a USB asynchronous sink, meaning it drives the audio clock from its own crystal oscillator rather than locking to the host's SOF timing. The feedback endpoint is re-armed from the `xfer_cb` completion in the custom UAC1 class driver (`uac1_driver_xfer_cb` on EP 0x82 in `usb_audio.c`) with the current `feedback_10_14` value, reporting the actual device sample rate to the host in 10.14 fixed-point format (samples per USB frame).

**Architecture:** Q16.16 dual-loop controller (`usb_feedback_controller.c/h`) with 10.14 wire serialization. All internal math uses Q16.16 fixed-point with rounded updates; only the endpoint-facing value is quantized to 10.14.

- **SOF handler** (`uac1_driver_sof()` in `usb_audio.c`, registered via the class driver's `.sof` pointer): Runs at each USB Start-of-Frame (1 kHz) in USB IRQ context (TinyUSB's DCD dispatches SOF-consumer driver callbacks directly from `dcd_event_handler` without going through the task queue). Reads the DMA transfer counter of slot 0 (SPDIF or I2S) and combines with `words_consumed` to get a sub-buffer-precise word total. Calls `fb_ctrl_sof_update()` which performs the 4-SOF decimated measurement and control update.
- **Rate estimator (Loop A):** First-order IIR with α=1/16 and `round_div_pow2_s32()` (symmetric half-away-from-zero rounding, int64_t-safe). Time constant τ≈64ms at the 4ms update rate (bRefresh=2). Raw rate computed via shifts only: SPDIF `delta_words << 12`, I2S `delta_words << 13`. The rounded update eliminates the truncation deadzone present in the previous `error >> 3` implementation.
- **Backlog servo (Loop B):** Proportional correction based on epoch-relative produced/consumed sample balance, replacing the former integer buffer-count fill servo. `slot0_produced_samples` is incremented in `usb_audio.c` when a slot-0 producer buffer is committed. Consumption is derived from DMA word progress: SPDIF `current_total_words << 14`, I2S `<< 15`. Backlog is computed in unsigned Q16.16 with modular arithmetic (wrap-safe as long as actual backlog remains far below 32768 stereo samples; steady-state ≈384, giving 85× margin). Servo gain Kp_q16=85 (equivalent to old 1024 per 48-sample buffer), clamped to ±0.25 sample/frame. No integrator.
- **Startup/reset gating:** After any reset, resync, stream activation, or slot-0 output-type switch, the servo is held at zero for 2 controller updates (~8ms). During holdoff, nominal feedback is emitted. On stream deactivation (alt 0), the controller is invalidated and all filter state cleared.
- **Rate change:** `perform_rate_change()` pre-computes `nominal_feedback_10_14 = (freq << 14) / 1000` and calls `reset_usb_feedback_loop()` → `fb_ctrl_reset()`, reseeding the rate estimator at nominal and establishing a new backlog epoch.
- **Flash blackout recovery:** `flash_write_sector()` and `preset_delete()` call `fb_ctrl_reset()` after the ~45ms interrupt blackout, reseeding the controller at nominal.
- **Endpoint serialization:** `fb_ctrl_get_10_14()` converts Q16.16 to 10.14 via rounded shift: `(q16 + 2) >> 2`. Fallback to `nominal_feedback_10_14` if the controller has never been reset.
- **Total clamp:** nominal ±1.0 sample/frame (65536 in Q16.16).

**Key variables:**
| Variable | Type | Location | Description |
|----------|------|----------|-------------|
| `fb_ctrl` | `usb_feedback_ctrl_t` | `main.c` | Controller state (rate estimate, backlog filter, holdoff) |
| `feedback_10_14` | `volatile uint32_t` | `main.c` | Serialized endpoint value (written by SOF handler) |
| `nominal_feedback_10_14` | `volatile uint32_t` | `main.c` | Pre-computed nominal for current rate |
| `slot0_produced_samples` | `volatile uint32_t` | `main.c` | Monotonic produced counter (incremented in `usb_audio.c`) |

**S/PDIF/I2S library fields used by feedback:**
| Field | Type | Description |
|-------|------|-------------|
| `words_consumed` | `volatile uint32_t` | Total DMA words completed (incremented in DMA IRQ) |
| `current_transfer_words` | `uint32_t` | Size of current in-flight DMA transfer |

**IRQ safety:** The SOF handler runs inside `isr_usbctrl`. DMA IRQ priorities are explicitly set to `PICO_HIGHEST_IRQ_PRIORITY` (`usb_audio.c:2755-2756`), matching the USB IRQ default. An init-time assertion (`NVIC_GetPriority(USBCTRL_IRQ) <= NVIC_GetPriority(DMA_IRQ)`) verifies that DMA cannot preempt the SOF handler's non-atomic multi-field read of `words_consumed` + `transfer_count`.

### USB Audio Decoupling (SPSC Ring Buffer)
*Last updated: 2026-04-18*

The DSP pipeline is decoupled from USB audio transfer completion via a lock-free SPSC ring buffer (`usb_audio_ring.h`). The UAC1 class driver's `xfer_cb` pushes raw packets into the ring; the main loop drains the ring and runs the full DSP pipeline. This prevents the USB stack from being blocked for hundreds of microseconds per packet and eliminates ISR-context spinlock contention.

**Context change vs. pico-extras:** under pico-extras, `_as_audio_packet` ran in USB IRQ context. Under TinyUSB, `DCD_EVENT_XFER_COMPLETE` is enqueued in the USB IRQ and dispatched to `uac1_driver_xfer_cb` from `tud_task()` (main-loop context). SOF still runs in IRQ. The ring itself is unchanged — the producer moved from IRQ to task, the consumer remained in the main loop.

**Ring buffer:** 4 fixed-size slots × 578 bytes (576 payload + 2 length). ~2.3KB BSS. Placed in RAM (`__not_in_flash`) for flash-operation safety. Peek/consume pattern (zero-copy consumer).

**Memory barriers:** `__dmb()` at publish/acquire points. Redundant on RP2040 (Cortex-M0+ in-order single-bus) but required on RP2350 (Cortex-M33 write buffer).

**Gap detection:** USB packet arrival gap measurement runs in `uac1_driver_xfer_cb` (task context under TinyUSB) using file-scope `audio_ring_last_push_us`, reset on stream lifecycle transitions in `uac1_apply_alt()` and `usb_audio_flush_ring()`.

**Ring overruns:** Separate `audio_ring.overrun_count` counter (distinct from `spdif_overruns`). Queryable via `REQ_GET_STATUS` wValue=22.

**Deferred flash SET commands:** `REQ_PRESET_SET_NAME`, `REQ_PRESET_SET_STARTUP`, `REQ_SET_OUTPUT_CONFIG_MODE` (and the action-style `REQ_SAVE_OUTPUT_CONFIG`) use separate pending flags per command type. Main loop copies payload under brief interrupt-off (~1µs) to prevent ISR/thread race, then drains ring and executes the flash write. GET-style flash commands (SAVE/LOAD/DELETE) remain synchronous in the vendor handler with real result codes.

### Packet Flow
*Last updated: 2026-04-18*

`uac1_driver_xfer_cb(EP 0x01)` → `usb_audio_ring_push()` → (main loop) → `usb_audio_drain_ring()` → `process_audio_packet(data, len)` [usb_audio.c] → `process_input_block(sample_count)` [audio_pipeline.c]

1. **Ring push (task context)** — Copy raw packet into SPSC ring, detect arrival gaps
2. **Ring drain (main loop)** — Peek/process/consume loop, highest priority in main loop
3. **USB decode (`process_audio_packet` in `usb_audio.c`)** — Gap detection, sync tracking, USB byte decode (16/24-bit) with per-channel preamp into `buf_l[]`/`buf_r[]`
4. **DSP pipeline (`process_input_block` in `audio_pipeline.c`)**; input-agnostic: buffer acquisition, preset mute envelope, EQ, leveller, matrix mixer, per-output-pair crossfeed (PASS 4.5), per-output EQ/gain/loudness/delay, output encoding, buffer return, CPU metering
5. **Buffer return** — Give completed buffers to consumer pools for DMA

The `process_input_block()` function reads from `buf_l[]`/`buf_r[]` arrays (extern, defined in `audio_pipeline.c`, filled by the input decode stage). This separation enables future alternative input sources (S/PDIF, I2S) to fill the same buffers and call `process_input_block()` directly. Buffer statistics helpers (`get_slot_consumer_fill()`, `get_slot_consumer_stats()`, `reset_buffer_watermarks()`) also live in `audio_pipeline.c`.

### RP2350 Float Pipeline
*Last updated: 2026-08-01 (upmixer centre engine gains an OFF mode)*

All processing in IEEE 754 single-precision float. Hybrid SVF/biquad EQ filtering (SVF for bands below Fs/7.5, TDF2 biquad above).

| Stage | Description |
|-------|-------------|
| Input conversion | USB int16/24-bit or SPDIF RX 24-bit → float full-scale, per-channel preamp gain (`global_preamp_linear[ch]`) |
| Per-Input EQ + metering | Per active input: block-based `dsp_process_channel_block()`, 10 bands, hybrid SVF/biquad; then peak/clip into `peaks[k]`/`clip_flags` |
| Volume Leveller | Upward RMS compressor over active inputs (2 to 8), mask-driven detector/apply link, gain-reduction limiter, 5 ms per-channel lookahead (float throughout) |
| Stereo Upmixer | Stereo input only: derives Centre/Ls/Rs into matrix source rows 2..4 and removes centre from L/R (nothing removed with the centre engine OFF, leaving L/R bit-exact); raises the matrix source count to 3 or 5. Parks in multichannel modes. Zero-latency steering; deliberate identical-per-row surround Haas delay (see "Stereo Upmixer") |
| Matrix mixing | Block-based: 2 inputs × 9 outputs (8 inputs in 8-channel USB mode, or 3/5 sources with the upmixer active) with gain/phase (loudness now per-output post-gain; leveller still runs) |
| Crossfeed | BS2B lowpass + allpass (ILD + ITD) per output pair, post-matrix (PASS 4.5), pre-output-EQ; `output_pair_mask`-selected (up to 4 pairs); works in every input mode |
| Output EQ | Block-based, 10 bands per output (Core 0: outputs 0-1, Core 1: outputs 2-7) |
| Output gain | Per-output gain × host volume × master volume |
| Loudness | Per masked output, post-gain: 2 SVF shelf filters, volume-keyed; `loudness_output_mask` selects outputs; skipped-and-cleared when masked off / muted / RAW test signal |
| Delay | Float circular buffers, 2048 samples max (42ms at 48kHz) |
| S24 finalization | Mode is snapshotted once per packet from `adat_output_is_active()` and shared with Core 1 via `core1_eq_work.finalize_s24`. ADAT active: after the last float consumer, each output row of `buf_out` (rows 0-7) is converted to a clamped 24-bit sample **in place** via `output_block_to_s24_inplace()` (`output_s24.h`, `out_s24_t` = `may_alias` int32), once per sample per channel, even when a slot pool is starved, so ADAT always sees converted data. ADAT inactive: the staging pass is skipped and each slot pair uses the fused `output_pair_convert_interleave()` (rows stay float; no second memory pass). Slot bytes are bit-identical in both modes. Row 8 (PDM sub) stays float always. |
| SPDIF output | ADAT active: pure integer copy of the finalized S24 rows into the 4 stereo slot pairs (`output_pair_interleave_s24()`). ADAT inactive: fused float->int24 convert+interleave per pair |
| ADAT output | Encodes the same finalized S24 rows 0-7 directly; the push is gated on the same per-packet snapshot (see "ADAT Bulk Output") |
| PDM output | Float → Q28 for sigma-delta modulation |

### RP2040 Fixed-Point Pipeline
*Last updated: 2026-07-10 (crossfeed now per-output-pair, post-matrix)*

Block-based two-phase architecture with dual-core EQ processing, all in Q28 fixed-point (28 fractional bits). 2 S/PDIF stereo pairs + 1 PDM sub (5 output channels).

**Phase 1 (Core 0, block-based where possible):**

| Stage | Description |
|-------|-------------|
| Input conversion | USB int16 → Q28 (`<< 14`), USB/SPDIF RX 24-bit → Q28 (`sample << 6`), per-channel preamp via `fast_mul_q28()` (`global_preamp_mul[ch]`) — block loop to `buf_l[192]`, `buf_r[192]` |
| Per-Input EQ + metering | Per active input: **block-based** `dsp_process_channel_block()`, 10 bands; then peak/clip into `peaks[k]`/`clip_flags` |
| Volume Leveller | Upward RMS compressor on the 2 inputs, mask-driven detector/apply link, gain-reduction limiter, 5 ms per-channel lookahead (Q28 envelope + float gain) |
| Matrix mixing | Q15 gains via `fast_mul_q15()` (16-bit partial products), 2 inputs × 5 outputs → `buf_out[5][192]` |
| Crossfeed | BS2B block via `fast_mul_q28()` (Q28 coefficients) per output pair, post-matrix (PASS 4.5), pre-output-EQ; `output_pair_mask`-selected (up to 2 pairs) |

**Phase 2 (per-output block, dual-core or single-core):**

| Stage | Description |
|-------|-------------|
| Output EQ | **Block-based** `dsp_process_channel_block()`, 10 bands per output |
| Output gain + volume | Combined Q15 multiply via `fast_mul_q15()` (output gain × host volume × master volume) |
| Loudness | Per masked output, post-gain: 2 Q28 biquads via `fast_mul_q28()`, volume-keyed; `loudness_output_mask` selects outputs; skipped-and-cleared when masked off / muted / RAW test signal |
| Delay | int32 circular buffers, 2048 samples max (42ms at 48kHz) |
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
slot-0 final 24-bit samples (audio_pipeline.c, before give_audio_buffer)
  -> loopback_push_slot0() -> SPSC ring (1024 frames, .bss, ~8 KB)
      -> rate-matching servo (fill_audio_packet, per USB frame, in xfer_cb)
          -> iso async IN EP 0x81 -> USB host
```
- **Tap:** a single call before slot 0's buffer is handed to the output DMA, in
  `process_input_block()`. It reads the already-finalized
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

**FP contraction (added 2026-08-05, RP2350 only):** `dsp_pipeline.c` is compiled with `-ffp-contract=off`, so the EQ kernels use separate VMUL/VADD instead of fused VFMA. Hardware measurement on the CPU meter established that the M33 FMA pipeline is throughput-bound with an effective VFMA occupancy of roughly 2 to 2.5 cycles versus 1 for VMUL/VADD: de-contraction emits ~58 % more FP instructions (and eliminates the VMOV accumulator copies VFMA's destructive form forces) yet measures 14 to 24 % less EQ cost on every kernel path, SVF and biquad alike, with no register spills. Loads/stores already overlap FPU issue, so FP-op *occupancy* is the only currency that matters in these loops. That is why the fused kernel only became worthwhile once its arms were specialized to match the single kernels' op counts (above): while it used the generic 9-op form it was giving back most of what the saved memory traffic won. Precision cost of double rounding is negligible and was verified on hardware: loopback THD, noise floor, and flat-path residual byte-identical to the contracted build, filter responses within 0.01 dB, and host analysis puts the noise penalty at ~1.5 dB on a −137 dB re-signal error floor. The flag is per-file: other DSP translation units (loudness, psybass, leveller, crossfeed, upmix) still contract and are candidates for the same measure-then-decide treatment.

*Last updated: 2026-08-06*

**Memory impact:** Biquad struct grows from ~48 to ~68 bytes on RP2350. With 110 EQ biquads at the larger size: ~3 KB additional BSS. (Loudness no longer uses the full `Biquad` struct; since 2026-07-09 its per-output shelf state is a separate minimal array, `loudness_output_state`, 144 B on RP2350 / 80 B on RP2040.)

**RP2040:** Completely unaffected. All SVF code is inside `#if PICO_RP2350` blocks.

### Per-Band Bypass
*Last updated: 2026-05-04*

Each EQ band has a user-controllable bypass flag in `EqParamPacket.bypass` (config.h). When set to exactly `1` it forces `Biquad.bypass = true` inside `dsp_compute_coefficients()`, which causes the audio inner loops in `dsp_pipeline.c`, `dsp_process_rp2040.S`, `audio_pipeline.c`, and `pdm_generator.c` to skip the band entirely (`if (bq->bypass) continue;`). User bypass and the existing auto-bypass for `FILTER_FLAT` / zero-gain peaking/shelf filters share the same `Biquad.bypass` flag — `channel_bypassed[ch]` (true when *every* band on the channel is bypassed) automatically benefits, allowing the audio path to skip the entire channel.

**0xFF safety:** the byte at offset 3 of `EqParamPacket` was previously named `reserved`. To stay backward-compatible with hosts that may zero-pad with `0xFF` rather than `0x00`, the firmware treats *only* the literal value `1` as bypass — every other value (0, 0xFF, 0x42, …) leaves the band active. Intake points (`REQ_SET_EQ_PARAM`, `REQ_SET_BAND_BYPASS`, bulk-params apply, flash preset load) normalize the byte to 0 or 1 so GET round-trips and the live recipe stay clean.

**Wire format:** `WireBandParams.reserved[3]` was split into `bypass` (1 byte) + `reserved[2]` (still zero-padded). Same byte layout, no `WIRE_FORMAT_VERSION` bump needed.

**Persistence:** `filter_recipes` is `memcpy`'d into `PresetSlot.filter_recipes` (flash_storage.c:500/625) so the bypass byte rides through preset save/load with no `SLOT_DATA_VERSION` bump. Legacy presets had `0` in that byte → they load as "active", preserving original behavior. `apply_slot_to_live()` re-normalizes after `memcpy` for defense-in-depth.

**Factory-default channel/band fix (2026-05-04):** `dsp_init_default_filters()` now writes `.channel = ch` and `.band = b` into every `filter_recipes` slot. Previously these fields stayed at BSS-zero, which caused `REQ_SET_BAND_BYPASS` to silently misroute writes to slot (0,0) since the handler copies the recipe (carrying the stale `.channel`/`.band`) into `pending_packet`, and the main-loop apply does `filter_recipes[p.channel][p.band] = p`. Symptom: bypass toggle didn't take effect on factory-defaulted bands until `REQ_SET_EQ_PARAM` rewrote the recipe with correct channel/band. `apply_slot_to_live()` (`flash_storage.c`) also normalizes `.channel` / `.band` after the recipe `memcpy` so old presets saved before the fix don't carry the bug into live state.

**Factory defaults are now flat (2026-05-04):** `dsp_init_default_filters()` no longer installs the 80 Hz highpass on output channels nor the 80 Hz lowpass on the PDM sub. Every band starts as `FILTER_FLAT` with `freq = 1000`, `Q = 0.707`, `gain_db = 0`, `bypass = 0`. Crossover/sub configuration is now an explicit user choice via the host, not a baked-in default that the user has to discover and override.

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

NUM_DELAY_CHANNELS = NUM_OUTPUT_CHANNELS (platform-dependent).

| Platform | Channels | Type | Max samples | Max delay (48kHz) | RAM usage |
|----------|----------|------|-------------|--------------------|-----------|
| RP2350 | 9 | float | 2048 | 42 ms | 72 KB |
| RP2040 | 5 | int32_t | 2048 | 42 ms | 40 KB |

Circular buffer: `delay_lines[ch][(write_idx - delay_samples) & MAX_DELAY_MASK]`

PDM sub gets automatic alignment compensation: +SUB_ALIGN_SAMPLES (128 samples = 2.67ms).

---

## Crossover Filters
*Last updated: 2026-08-05*

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

- `xover_filters[NUM_CHANNELS][MAX_XOVER_BANDS]` — designed biquad cascades
- `xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS]` — user-supplied recipe (EqParamPacket)
- `channel_xover_bypassed[NUM_CHANNELS]` — fast-path flag; the stage is skipped entirely for a channel when all 4 bands are bypassed (the default)

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

Every default band: `type=FILTER_FLAT, freq=1000.0, Q=0.707, gain_db=0, bypass=0, band=XOVER_BAND_BASE+i`. Because FLAT is not a crossover type, the design routine produces a bypassed cascade and `channel_xover_bypassed[*] = true`. Zero per-sample cost until the user picks a real crossover type.

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
*Last updated: 2026-03-19*

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

2-instruction NRZI encoder running on PIO0. Clock divider automatically adjusted for 44.1/48/96 kHz.

### Instance State

```c
typedef struct audio_spdif_instance {
    PIO pio;
    uint8_t pio_sm, dma_channel, dma_irq, pin;
    bool enabled;
    uint8_t subframe_position;  // 0-191: position in IEC 60958-1 192-frame audio block
    audio_buffer_pool_t *consumer_pool;
    audio_buffer_t silence_buffer;
    // ... format, connection details
} audio_spdif_instance_t;
```

### Buffer Configuration

- Producer pool: 8 buffers × 192 samples × 2ch × 4 bytes = 12,288 bytes per pool
- Producer format: `AUDIO_BUFFER_FORMAT_PCM_S32` (24-bit audio in lower 24 bits of int32)
- Consumer pool: 16 buffers × 48 samples (`SPDIF_CONSUMER_BUFFER_COUNT` × `PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT`)
- Consumer format: `AUDIO_BUFFER_FORMAT_PIO_SPDIF` (pre-encoded NRZI subframes)
- DMA transfer granularity: 48 samples (1 ms at 48 kHz), down from 192 samples (4 ms)
- Total consumer capacity: 16 × 48 = 768 samples (same as previous 4 × 192)
- Fill target: 8 buffers (50%), latency jitter: ±1 buffer = ±1 ms (was ±4 ms with 192-sample buffers)

### IEC 60958-1 Block Position Tracking

Each 192-frame audio block carries channel status bits and a Z preamble at frame 0. With 48-sample DMA transfers, block boundaries no longer align to buffer boundaries. A per-instance `subframe_position` counter (0-191) tracks the current position within the 192-frame block across buffer boundaries:

*Last updated: 2026-03-23*

- **Init:** Each consumer buffer is pre-initialized with preambles and channel status via `init_spdif_buffer(buffer, start_pos)`. These are treated as templates — runtime fixup corrects them before each DMA transfer.
- **Runtime:** `subframe_position` advances by `PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT` (48) **unconditionally** after each DMA completion (including silence), maintaining correct 192-frame alignment across silence/audio transitions. Wraps at 192 using a branch (no modulo — avoids expensive division on M0+).
- **Preamble + channel status stamping:** The consumer pool free list is LIFO, so buffers may return in a different order than initialized. `audio_start_dma_transfer()` stamps the correct Z/X preamble on the first L-channel subframe **and** corrects all channel status bits (IEC 60958-3 C bit at h[29]) to match the current `subframe_position`. When the C bit must flip, both C (bit 29) and parity P (bit 31) are XOR'd together, maintaining even subframe parity without recomputation. Applied to all buffers including the silence buffer.
- **Static assert:** `PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT % PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT == 0` enforced at compile time.

### 24-bit Output Encoding
*Last updated: 2026-05-19*

The USB input supports both 16-bit and 24-bit PCM via two alternate settings on the Audio Streaming interface. The host OS selects the desired bit depth; a runtime variable (`usb_input_bit_depth`) tracks the active format and branches the input conversion accordingly. With 24-bit input and 24-bit SPDIF output, the full precision signal path is maintained end-to-end. The DSP pipeline operates at >16-bit precision internally (float on RP2350, Q28 fixed-point on RP2040).

**Alt-change safety (2026-04-18):** With `TUP_DCD_EDPT_ISO_ALLOC` enabled on RP2040/RP2350, TinyUSB's `usbd_edpt_close()` is a no-op — it does not clear the `busy` flag left by the previous alt's in-flight iso xfer. On the next `usbd_edpt_xfer()` this trips `TU_ASSERT(busy == 0)` and crashes the device. `uac1_open_stream_eps()` therefore calls `usbd_edpt_clear_stall()` on both stream endpoints after `usbd_edpt_iso_activate()` to force-clear the stale busy bit (the same workaround TinyUSB's stock audio class driver applies at `audio_device.c:1871`). `uac1_apply_alt()` also flushes the USB audio ring whenever `usb_input_bit_depth` changes so queued pre-switch packets aren't re-decoded under the new bytes/frame assumption. See "Sample-rate & Bit-depth Switching" for the full mute/resync flow that brackets every format change.

**Input conversion (24-bit):**
- **RP2350:** 3-byte little-endian → sign-extended int32 → float via `÷ 8388608.0f`
- **RP2040:** 3-byte little-endian → Q28 via left-justify and `>> 2` (net `<< 6`); same full-scale as 16-bit (`<< 14`)
- **SPDIF RX:** extracted 24-bit samples follow the same internal full-scale convention: RP2350 sign-extends to int32 full-scale and divides by `2147483648.0f`; RP2040 shifts the sign-extended full-scale word by `>> 2` into Q28, matching the USB 24-bit `sample << 6` path so unity processing survives the later Q28 `>> 6` output conversion.

- **RP2350:** `float → int32_t` via `(int32_t)(sample * 8388607.0f)` (24-bit full-scale)
- **RP2040:** `Q28 → int24` via `clip_s24((sample + (1 << 5)) >> 6)` (shift right 6 with rounding)
- **Encoding:** `spdif_update_subframe()` encodes 3 bytes through the NRZI lookup table (was 2 for 16-bit)
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

All instances share DMA IRQ 1 via `irq_add_shared_handler()`. Reference-counted enable/disable. Handler iterates registered instances to find interrupt source.

### Synchronized Start

`audio_spdif_enable_sync()` starts all 4 PIO state machines on the same clock cycle using `pio_enable_sm_mask_in_sync()`.

---


In I2S clock-slave mode the synchronized start is further gated on an external LRCLK falling edge: `complete_pipeline_reset()` Phase 2 and `enable_outputs_in_sync()` first run the prepare-only halves (`audio_spdif_enable_sync_prepare()` / `audio_i2s_enable_sync_prepare()`), then wait for the edge (bounded ~35 us, skipped when no output slot is I2S), then start every slot of BOTH output types in one `pio_enable_sm_mask_in_sync` write. Together with the external-clock I2S program's one-frame discard this makes the SPDIF-vs-I2S inter-slot offset reset-invariant. *Last updated: 2026-07-06*
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
- Core 1 maintains TARGET_LEAD (256 samples) ahead of DMA read pointer

### Input Limiting

Hard clip at ±90% modulation (PDM_CLIP_THRESH = 29500) to prevent sigma-delta instability.

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
low  = LP2(x)                          // 2nd-order lowpass split at cutoff (Butterworth, Q=0.707)
even = |low|                           // full-wave rectifier: even harmonics, level-proportional
odd  = softclip(drive * low)           // cubic soft clipper (1.5d - 0.5d^3): odd harmonics
h    = (1 - t)*even + t*odd            // character blend t (0 = even/warm, 1 = odd/aggressive)
h    = OP4fc(HP2(h))                   // HP2 at cutoff kills DC + fundamental; one-pole LP at 4x cutoff caps brightness
out  = x + (g_orig - 1)*low + g_harm*h // original low band already split out, so its level control is free
```

The full-wave rectified (even) path tracks program dynamics; the driven cubic (odd) path adds bite. Blending both yields the consecutive 2f/3f/4f series that pitches the missing fundamental. The harmonic highpass at the cutoff removes DC and the fundamental itself (leaving only the reproducible harmonics); the one-pole lowpass at `4 * cutoff` (`PSYBASS_HARM_LP_RATIO`) is a gentle 6 dB/oct rolloff mimicking natural harmonic decay. The original low band is split out separately, so `original_db` attenuates it independently (`orig_delta = 10^(orig/20) - 1`, range -1..0) without touching the rest of the signal.

**Zero added latency.** The effect is pure IIR (no delay lines, identical `sample_count` on every output), so inter-output-slot sample alignment is untouched by construction (the CLAUDE.md inviolable guarantee).

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

## Stereo Upmixer
*Last updated: 2026-08-01 (centre engine OFF mode)*

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

- **Double-buffered publish.** `upmix_apply_config()` computes one shared `UpmixCoeffs` into the inactive buffer (`um_coeff_bufs[2]`) and atomically publishes `volatile const UpmixCoeffs *current_upmix_coeffs` (**NULL = disabled**), never mutating the buffer the pipeline reads. Vendor SET handlers write `upmix_config` and raise `upmix_update_pending`; the main loop consumes the flag and recomputes. Coefficients are also recomputed on rate change (`perform_rate_change()` raises the flag). Initial setup runs once in `core0_init()`.
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

- **Wire format V25 (presence byte V26; centre OFF V27):** `WireUpmixParams` (44 bytes, layout-identical to `UpmixConfigPacket`) is tail-appended to `WireBulkParams` (total 5944 bytes). V26 claims the section's reserved byte for `presence_q1` (int8, dB * 2, 0.5 dB steps; no size change). V27 changes no layout at all; it exists so hosts can detect that `center_mode` accepts 2 (OFF). Bulk collect/apply copy it straight to/from `upmix_config` and raise the pending flag; the section is zeroed on collect and ignored on apply on RP2040.
- **Preset slot V33 (presence byte V34):** the config is tail-appended to `PresetSlot` (struct grows 44 bytes; `SLOT_DATA_SIZE_V33`), gated on `slot->version >= 33` in `apply_slot_to_live()` (RP2350). V34 claims the upmix reserved byte for `upmix_presence_q1` (no size change; `SLOT_DATA_SIZE_V34` = `SLOT_DATA_SIZE_V33`); V33 slots always wrote 0 there, which decodes to the 0 dB default, so no version gate is needed for the byte. Older slots have no upmix data and load the disabled defaults; V21..V33 slots still validate via `slot_data_size_for_version()`. Factory reset sets it disabled with `UPMIX_DEFAULT_*` values. RP2040 round-trips the fields as zeros and never applies them.
- **Vendor commands:** `0x4A-0x4E`; RP2350 only (SETs STALL on RP2040, GETs return zeros). `0x4A` SET_CONFIG takes the 44-byte `UpmixConfigPacket` (byte 3 = `presence_q1`, int8 dB * 2); `0x4B` GET_CONFIG returns it; `0x4C`/`0x4D` SET/GET_PARAM address a single field by `wValue` (`UPMIX_PARAM_*` 0..13; 13 = `UPMIX_PARAM_PRESENCE`, plain float dB) as a 4-byte float; `0x4E` GET_STATUS returns the 16-byte `UpmixStatus`. `0x4F` is reserved. See the Vendor Command Reference table.

**Matrix row-sharing consequence.** Source rows 2..4 are dual-purpose: they carry multichannel inputs 3..5 in multichannel input modes and the Upmix C / Ls / Rs derived channels in stereo upmix mode. The matrix crosspoints for those rows are the same storage in both cases, so a host application must present them contextually (real input vs upmix-derived) based on the active input mode.

---

## Volume Leveller
*Last updated: 2026-07-10 (crossfeed no longer stereo-only; runs per output pair)*

### Purpose

Automatic volume levelling via a feedforward, channel-linked, single-band RMS compressor applied to the active input channels (2 to 8 on RP2350; always 2 on RP2040), pre-matrix, after per-input EQ (PASS 2.5 in the pipeline). Runs at all input counts; no longer bypassed in multichannel (the earlier stereo-only restriction is gone). Crossfeed now runs per output pair post-matrix (`output_pair_mask`-selected), not on the stereo input bus, so it too works in every input mode (see "Crossfeed"); loudness likewise runs per output (post-gain, `loudness_output_mask`-selected), so it is no longer stereo-only (see "Loudness Compensation").

### Algorithm

- **Topology:** Feedforward upward compressor with soft knee — boosts content below the threshold, leaves content above the threshold completely untouched (no makeup gain needed)
- **Channel linking:** A single shared gain drives every selected channel. The link is the loudest selected detector envelope (max over `detector_mask` channels), preserving the mix balance; for stereo this is exactly the old "louder of L/R" behavior
- **Channel masks:** `detector_mask` (which inputs feed the RMS detector) and `apply_mask` (which inputs receive the shared gain), each a `uint8_t` bit-per-input, default 0xFF (all channels). Both are ANDed with the active-input set at runtime; CPU is only spent on selected channels. Example uses: dialog boost = both masks on the center channel; night mode = all channels
- **Envelope:** Asymmetric attack/release smoothing on the per-channel RMS envelopes
- **Lookahead:** Optional 5 ms (240-sample) lookahead, one ring per input channel. When enabled, EVERY active input is delayed through its ring (applied channels through the gain stage, others as a plain delay), so inter-channel and inter-output-slot alignment is preserved exactly and mask changes cause zero time shift. Rings of newly activated inputs are cleared via the `active_prev` field
- **Gain computation:** Upward compression curve: content below threshold is boosted by `(threshold - x_db) * (1 - 1/ratio)`, content above threshold + knee/2 passes at unity (0 dB gain), with soft knee transition between
- **Limiter:** Gain-reduction style at -6 dBFS ceiling (instant attack, 100ms release) — computes gain reduction rather than hard clipping, rarely engages since loud content is untouched
- **Gate:** User-configurable silence gate prevents noise amplification when input is below the gate threshold
- **Glitch-free masks:** Writing masks (0xDE) recomputes coefficients but does NOT reset state; the rings keep running so a mask change is glitch-free

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
*Last updated: 2026-07-09 (per-output processing + output mask)*

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
*Last updated: 2026-07-25 (selective NVIC blackout: the output DMA IRQ lines stay live through erase/program, so every slot keeps clocking framed silence instead of freezing mid-frame; PDM ring silenced and re-anchored). Previously, 2026-07-23 (every runtime flash write now completes via complete_flash_write_operation_full; light completion path removed)*

### Flash Operation Safety

Flash erase/program requires quiescing XIP (execute-in-place): only the erase/program windows forbid flash fetches; ordinary XIP execution is legal even with IRQs disabled. `flash_write_sector()` and `preset_delete()` run under a selective interrupt blackout (below) and park Core 1 via a guarded `multicore_lockout` when Core 1 is a registered victim:

- **Guard condition:** `multicore_lockout_victim_is_initialized(1) && (__get_current_exception() == 0)`. The SDK function handles first-boot (Core 1 not launched) and launch-to-init race windows. The exception check skips lockout from IRQ context (USB vendor handler), where SDK lock internals are unsafe. IRQ-context saves that skip the lockout are safe because everything Core 1 can execute is RAM-resident (enforced by `scripts/check_ram_placement.py` Check B / B2), so a parked-or-not Core 1 never fetches from flash during the erase/program window. The audio consequence of a flash write is unchanged (muted window, feedback re-seed).
- **Core 1 victim init:** `multicore_lockout_victim_init()` called at the start of `pdm_core1_entry()`.
- **Interrupt blackout:** ~45 ms for sector erase + program. Since 2026-07-25 this is a *selective* blackout, not PRIMASK (see below). The existing mute strategy (`preset_loading` + `preset_mute_counter`) and feedback reseed still cover the audio gap.

#### Selective NVIC blackout — outputs keep clocking through the window
*Last updated: 2026-07-25 (RP2350: NVIC word 1 now masked too — flash-resident UART/I2C control ISRs could fire mid-window; restore ordering word 0 then word 1)*

Historically the erase/program window ran under `save_and_disable_interrupts()`. With every IRQ masked the output DMA completion handlers could not re-arm: each slot's DMA finished its in-flight 48-sample transfer, the PIO TX FIFO drained within tens of microseconds, and the output pin then held a DC level for the rest of the window. AC-coupled outputs turned that step into a pop, SPDIF receivers saw a dead line and unlocked, and BCK-PLL DACs (PCM5102 with SCK grounded) could exit the halt mis-locked.

`flash_irq_blackout_begin()` / `flash_irq_blackout_end()` (`flash_storage.c`, both RAM-resident) replace it:

- **Thread context** (the only production path — every runtime flash write is deferred to the main loop): save NVIC `ISER` word 0, then write `ICER` to disable everything *except* `DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ` (DMA_IRQ_0, the I2S TX handler line) and `DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ` (DMA_IRQ_1, the SPDIF TX handler line). `__dsb(); __isb();` orders the mask before the ROM flash calls; on exit `ISER` is restored.
- `ICER`/`ISER` are written directly rather than through `irq_set_mask_n_enabled()`, because the SDK helper clears pending bits on re-enable; direct writes preserve IRQs latched during the window (USB, timer, UART/I2C), matching the old PRIMASK semantics. Register shapes differ per platform (RP2040 scalar `iser`/`icer`, RP2350 `iser[2]`/`icer[2]`); both DMA IRQ numbers are below 32 on both platforms (static-asserted), so the keep mask lives entirely in word 0.
- **RP2350 NVIC word 1 is masked wholesale.** RP2350 interrupt numbers run past 31: SPI1 (32), UART0/1 (33/34), ADC (35), I2C0/1 (36/37) live in `iser[1]`/`icer[1]`, which the initial implementation never touched. The UART and I2C control-interface ISRs (`uart_ctrl_irq`, `i2c_ctrl_slave_handler`) are flash-resident, so a byte arriving on an active control link mid-window vectored into quiesced XIP — a hard fault, or an intermittent one when the XIP cache happened to hold the handler. The old PRIMASK blackout covered these; the selective blackout now saves/clears word 1 alongside word 0. RP2040 has no IRQ above 31, so word 0 covers everything there. Control-link bytes arriving during the window can still overflow the 32-byte UART RX FIFO (~45 ms at 115200 baud is ~520 byte times); that loss is identical to the PRIMASK era and the frame CRC + status-response protocol already recovers from it.
- **Restore ordering (defensive).** An IRQ enabled by an `ISER` write can preempt at the next instruction boundary — the trailing `__dsb(); __isb();` order the writes, they do not gate preemption. The end path therefore restores word 0 first, then word 1, so a pending UART/I2C interrupt that preempts mid-restore finds the core IRQs (timer, USB) already enabled; the begin path masks in the reverse order. XIP is back before `flash_irq_blackout_end()` runs, so this is consistency hygiene, not a crash guard.
- **IRQ context** (defensive only): falls back to the PRIMASK blackout, since NVIC masking cannot guarantee the DMA handlers preempt the current exception frame.

With those two lines live, the handlers keep chaining buffers for the whole window: real (already faded-to-zero) pool content while the pools last, then each instance's own silence buffer. The SPDIF handler re-stamps the Z/X preamble and corrects channel-status bits on every buffer including silence and keeps advancing `subframe_position`, so the 192-frame IEC block structure is continuous across the audio→silence→audio transitions; I2S slots emit the `I2S_PAD_PATTERN` dither so DAC zero-detect mutes do not chatter. Every slot delivers exactly 48 samples per DMA period throughout, so **inter-slot alignment during and after a flash window is identical to steady-state streaming** — nothing is aborted, no slot misses a period.

Why the handlers are safe while flash is unavailable: the vector table is the SDK's RAM copy (VTOR), the shared-IRQ chain slots live in `.data`, `audio_spdif_dma_irq_handler` / `audio_i2s_dma_irq_handler` and their whole call graph (`audio_start_dma_transfer`, the pool primitives) are `__isr __time_critical_func`, and the IEC channel-status table plus the per-instance silence buffers are RAM statics. `scripts/check_ram_placement.py` enforces it: Check A (hot symbols in RAM), Check B (no branch from the RAM closure into flash), and Check B2, whose `FLASH_WINDOW_ROOTS` scan walks both handlers' call graphs for flash-range literal-pool words and hard-fails if either root is not RAM-resident. Pool primitives take their spin locks with `spin_lock_blocking()`, which disables interrupts on the holding core, so Core 1 can never be parked mid-critical-section and deadlock the now-live Core 0 handler.

What stays masked: the timer IRQ (so `pico_spdif_rx`'s decode-timeout alarm cannot fire on the blackout edge, which historically crashed the core), USB (the host just NAKs; the ring is drained and the feedback controller re-seeded afterwards), the RP2350 word-1 peripherals (UART/I2C control interfaces, ADC, SPI1 — see above), and everything else. SPDIF RX and I2S RX are stopped by the caller's flash bracket anyway; `spdif_rx_dma_irq_handler` shares DMA_IRQ_1 with SPDIF TX, and with its channels masked only its RAM-resident skeleton can execute.

**PDM through the window.** PDM's ring DMA is IRQ-less and free-running, so it keeps clocking regardless, but Core 1 (its producer) is parked, so it would loop the last ~45 ms of modulator output. `pdm_flash_silence()` fills the ring with the modulator's `0xAAAAAAAA` 50 % duty silence pattern and sets `pdm_force_reanchor`; content only, so the DMA, PIO and every pointer are untouched and PDM phase stays continuous. On resume the processing loop unconditionally re-seats its write lead to `read + TARGET_LEAD`: the ring lapped an unknown number of times while Core 1 was parked, and the loop's modulo write-read delta cannot distinguish a multi-lap underrun from a valid lead (the `delta > half-ring` test misses roughly half the outcomes), so without this PDM could resume with an arbitrary wrong lead. It is called only when the lockout actually parked Core 1 — a still-running Core 1 keeps filling the ring itself, and a forced re-anchor would then be a gratuitous phase jump.

**ADAT bulk output through the window (RP2350).** Its data/control DMA chain is IRQ-less and free-running, so the lightpipe stays framed — that was already true before this change, since ADAT never depended on an IRQ. The ring (896 frames, ~18.7 ms at 48 kHz) drains past the write pointer because the main loop, its producer, is blocked for ~45 ms; it keeps replaying valid pre-encoded frames whose audio content is muted (the fade ran first), so the receiver stays locked on silence. The offset is then re-canonicalized by `adat_output_resync()` — from `complete_pipeline_reset()` Phase 6, from the input prefill's `enable_outputs_in_sync()`, or from the lap detector.

What the selective blackout *does* change for ADAT is the starvation mirror. Slot 0 now keeps completing DMA transfers through the window and counts a starvation per silence buffer (~45 at 48 kHz) where it previously counted ~1 (a single latched completion serviced after re-enable). `adat_output_task()`'s 1:1 slaved insertion must not mirror that backlog: ADAT was not behind by 45 buffers — it emitted a frame per sample clock for the same span — so inserting ~2160 frames of silence would push the ADAT-to-slot offset the wrong way (and overflow the 896-frame ring, forcing a slip). `complete_flash_write_operation_full()` therefore calls `adat_output_rebaseline_starvations()` first, dropping the backlog without insertion; the resync that follows re-establishes the canonical `ADAT_ALIGN_LEAD_FRAMES` cushion. On the USB path Phase 6's resync would have re-baselined anyway (`adat_output_resync()` re-samples the counter); the explicit call is what covers the SPDIF path, whose completion returns early and hands the restart to the re-lock prefill.

**Completion policy (2026-07-23): every runtime flash write ends in a synchronized output restart.** All flash-write call sites complete via `complete_flash_write_operation_full()`: preset save/delete and the legacy save path as before, and now also every metadata-only directory write (preset rename, startup policy, output-config mode/save, master-volume mode/save, UART/I2C control-interface config, Control Surface save, DAC-mute config). The former `complete_flash_write_operation_light()`, which skipped the output rebuild because DSP/output topology was unchanged, was removed. Rationale: during the ~45 ms IRQ blackout the output DMA handlers could not re-arm, the PIO TX FIFOs drained within tens of microseconds of the in-flight buffer completing, and every output SM stalled with BCK/LRCLK/DATA frozen mid-frame; the light path then resumed those clocks mid-frame with no DAC mute cycle and no restart. External DACs that derive their system clock from BCK (e.g. PCM5102 with SCK grounded, BCK PLL mode) can exit that halt mis-locked and render all subsequent, perfectly correct samples as full-scale distortion until the clocks are stopped and restarted cleanly. Field-reported (2026-07-22) as persistent "full scale noise resembling the music" after a preset rename and after a Control Surface save (Win10 USB source, I2S output to a PCM5102); pressing preset save fixed it because that path already ran the full completion. **The clock halt itself is gone as of 2026-07-25** (selective NVIC blackout, above), so that failure mode no longer exists; the unified completion is nevertheless retained, because it is what refills the drained pools from a deterministic synchronized state and is the shared owner of the feedback reset and the DAC mute release. It preserves the inviolable inter-slot alignment: `complete_pipeline_reset()` restarts every SM in sync, and for SPDIF/I2S/ADAT sources the input's prefill/re-lock handshake performs the equivalent synchronized restart. Dropping the restart for topology-unchanged writes is separate future work (no-teardown completions). Output type switches (`process_type_switches()`) and output data-pin changes (`process_pin_changes()`) need no equivalent treatment: they perform no flash writes and already end in a synchronized restart; their persistence commands (output-config save/mode) are among the metadata writes covered above.

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

### Preset Directory Fields (Version 19)
*Last updated: 2026-08-12 (V19 appends the Control Surfaces display config and page table)*

`DIR_VERSION_CURRENT` = 19. V4 renamed the former `include_pins` byte to
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
platform-dependent and validate at apply / render time). See
`Documentation/Features/output_config_independent_load.md`,
`Documentation/Features/control_interfaces_spec.md`,
`Documentation/Features/control_surfaces_spec.md`, and
`Documentation/Features/control_surfaces_display_spec.md`.

| Field | Description |
|-------|-------------|
| startup_mode | 0 = load specified default, 1 = load last active |
| default_slot | Slot to load in "specified default" mode (0-9) |
| last_active_slot | Last slot loaded/saved (always 0-9) |
| output_config_mode | Physical IO persistence mode (was `include_pins`): 1 = with preset (default), 0 = independent (device-global). Governs output pins/types, I2S MCK/BCK, SPDIF RX pin. |
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

### Preset Slot Data (Version 12)
*Last updated: 2026-07-19 (upmixer presence byte, slot V34; `SLOT_DATA_VERSION` now 34)*

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

**Observed fade-out before any disruptive work (2026-07-25).** Arming the mute envelope does not make the wire silent. The envelope (`update_preset_mute_envelope()`, `audio_pipeline.c`) only advances when a packet is processed, its gain is applied *ahead* of the per-output delay lines, and the resulting samples still have to cross the consumer queues before the DMA plays them. The bracket used to arm the mute and tear down in the same main-loop iteration, so everything in flight was discarded and the clocks stopped on a full-level sample — the click the bracket exists to prevent. `main.c` now runs a shared, restartable settle state machine, `pipeline_fade_to_silence_poll()`, which reports complete only once (1) a processed packet has rendered the envelope down to zero and (2) those zeros have had time to clear the longest active output delay line (`pipeline_max_active_delay_samples()`) plus a full consumer pool and a couple of producer blocks. The dwell is computed in samples and converted at the live rate, so 44.1 / 48 / 96 kHz all wait the same amount of audio.

The machine drives the envelope through a second, independent countdown (`pipeline_request_soft_mute()`), never through `preset_loading`: that flag doubles as the SPDIF/I2S/ADAT prefill handshake, and setting it early would make the main-loop lock blocks tear the outputs down at the wrong moment. The request is refreshed by the waiter each iteration and expires on its own (`PIPELINE_FADE_REQUEST_MS`, 40 ms of audio), so a fade that is armed and then abandoned unmutes instead of sticking. Two call paths use it:

- `pipeline_reset_ready()`, the non-blocking pre-gate the deferred handlers already poll, fades first and asserts the DAC hardware mute only afterwards. An analog mute engaged at full level would truncate the very ramp it exists to cover, and the step reappeared when the pin deasserted.
- `prepare_pipeline_reset()` spins on the same machine while servicing the active producer (`pipeline_settle_to_silence()`), so callers that do not pre-gate (boot, rate changes driven by an input's lock machinery, source switches, the flash bracket) still fade. It exits on the first test when the caller already settled, and immediately when no producer is running — in that case the envelope is latched at zero (`pipeline_latch_mute_silence()`) because no packet will ever arrive to advance it. `process_type_switches()` runs the settle *before* it masks the USB IRQ, since the fade needs the ring to keep refilling. The whole settle is bounded by `PIPELINE_FADE_CAP_US` (200 ms) for a producer that stalls after being declared live.

**Fade back up.** `complete_pipeline_reset()` Phase 3.5 holds the envelope at zero for `PIPELINE_POST_RESET_MUTE_MS` (24 ms) plus any configured `dac_hw_mute_release_ms()` after the synchronized restart, then the normal 8 ms ramp runs. Without the dwell the ramp starts against pools that are still empty (gaps in the first audio) or completes while the analog mute is still asserted (a step when it releases). It is a floor, never a shortening: the SPDIF/I2S/ADAT prefill handshakes clear the counter when they enable outputs and fade up through the audio they prefilled instead. `prepare_pipeline_reset()` applies the same floor to its own mute counter so a 5 ms `PRESET_MUTE_SAMPLES` cannot expire mid-operation and start fading up into the disruptive window. A zero-length packet no longer snaps the envelope to its target — it renders nothing, so it must not skip the ramp.

Alignment is unaffected: the settle only adds ordinary shared-pipeline packets (every slot, PDM, delay index and DSP state advances by the same sample count), the envelope is one gain shared by both cores, and the teardown/synchronized-restart sequence is unchanged.

All preset operations (load, save, delete) are **deferred from the USB IRQ to the main loop** via pending flags (`preset_load_pending`, `preset_save_pending`, `preset_delete_pending` in `usb_audio.c`). This avoids running flash operations inside the USB ISR (which would cause a ~45ms interrupt blackout inside an interrupt handler) and allows proper pipeline reset bracketing.

**SPDIF-RX suspend across disruptive state swaps (required when SPDIF is the active input).** Any deferred handler that mutates the live DSP state (coefficients, matrix, delays, output types/pins) or tears down output slots must first stop the SPDIF receiver and restart it afterward. While RX runs, `pico_spdif_rx`'s decode-timeout alarm fires on a separate timer IRQ and can touch PIO/DMA state mid-mutation; left running, this races the swap and faults the core, which the 8-second `watchdog_enable()` then resets (RAM is re-initialised, so `buffer_stats_sequence` resets to ~0 — the signature of this reboot). `preset_load_pending`, `process_type_switches`, and `process_pin_changes` have always bracketed RX this way. The `bulk_params_pending` (`REQ_SET_ALL_PARAMS` / 0xA1) and `factory_reset_pending` (`REQ_FACTORY_RESET` / 0x53) handlers were **missing this bracket** and would reboot the device when applied with SPDIF input active (intermittent, ~4/6 under repeated `0xA1`). Both now mirror `preset_load`: after `prepare_pipeline_reset()`, `if (active_input_source == INPUT_SOURCE_SPDIF && spdif_input_get_state() != SPDIF_INPUT_INACTIVE) { spdif_input_stop(); spdif_prefilling = false; }`, and at the end restart via `spdif_input_start()` (guarded by `!input_source_change_pending`, since a bulk-applied source change defers RX management to that handler). `process_type_switches` leaves caller-stopped RX alone (`rx_was_running` check), so the change-mask path doesn't double-start. Verified: 0/10 reboots on `0xA1` with SPDIF locked after the fix (was 4/6).

The main loop handler for each operation follows the pattern:
1. `usb_audio_drain_ring()` — process in-flight audio packets
2. `prepare_pipeline_reset(PRESET_MUTE_SAMPLES)` — fade the wire to silence, wait for Core 1 idle, engage mute
3. Execute the preset operation (`preset_load/save/delete`)
4. `complete_pipeline_reset()` — drain stale consumer buffers, resync outputs, reset USB feedback

**Delay line zeroing:** `preset_load()` clears all delay line buffers (`memset(delay_lines, 0, ...)`) after `dsp_update_delay_samples()` to prevent stale audio from the previous preset's delay configuration bleeding through.

**Feedback recovery:** `flash_write_sector()` (called during save/delete) reseeds the feedback controller at nominal after the ~45ms interrupt blackout. `complete_pipeline_reset()` (called after load/delete) also resets feedback state.

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
*Last updated: 2026-07-15 (universal PIN_RESET_TO_DEFAULT 0xFF escape hatch across every single-pin SET command)*

Vendor commands `REQ_SET_OUTPUT_PIN` (0x7C) / `REQ_GET_OUTPUT_PIN` (0x7D).

**Constraints:** Valid GPIO range, not reserved (23-25 power/LED; GPIO 16/17 are no longer reserved now that the debug UART is gone), and not in use by another output, the I2S BCK/LRCLK or MCK pin, the SPDIF RX pin, the DAC hardware-mute pin, or a live UART/I2C control-interface pin (`is_pin_in_use`).

**Reset-to-default escape hatch (`PIN_RESET_TO_DEFAULT`, 0xFF):** every single-pin SET command — `REQ_SET_OUTPUT_PIN` (0x7C), `REQ_SET_I2S_BCK_PIN` (0xC2, per role), `REQ_SET_MCK_PIN` (0xC6), `REQ_SET_ADAT_PIN` (0xCC), `REQ_SET_SPDIF_RX_PIN` (0xE4, per index), `REQ_SET_ADAT_INPUT_PIN` (0x6A), `REQ_SET_I2S_RX_PIN` (0xF1, per pair) — maps a pin byte of 0xFF to the platform default for the addressed target before running its normal validation chain, so a reset can still fail (e.g. PIN_IN_USE if the default has been claimed). 0xFF is not a valid GPIO on either platform, so pin byte 0 always means GPIO 0; the previous `REQ_SET_ADAT_PIN` behavior of treating 0 as "reset to default 12" is gone (0 now addresses GPIO 0 like everywhere else). ADAT input has no free default GPIO, so its "default" is unset: 0xFF clears the pin (only while the input is disabled, since an enabled input with no pin would be unselectable). The persistence layers are unchanged: flash/bulk zero-fill "0 = default/absent" decoding is a migration artifact of zero-filled struct tails, not a wire semantic of the SET commands.

**S/PDIF and I2S slots:** Deferred to the main loop, not applied in the USB ISR. `REQ_SET_OUTPUT_PIN` writes the target into `output_pins[out_idx]` (RAM-only, like `spdif_rx_pin`) and sets a bit in `output_pin_change_mask`; the main-loop gate (shared `pipeline_reset_ready()` hold) then runs `process_pin_changes(mask)`. That helper mirrors `process_type_switches`: `prepare_pipeline_reset()` (soft mute + Core 1 fence + DAC hardware-mute assert) → suspend SPDIF RX if running → `drain_and_disable_outputs()` → `audio_spdif_change_pin()` / `audio_i2s_change_data_pin()` on each flagged slot while disabled → `complete_pipeline_reset()` for the **synchronized** restart of all slots → restart RX. The synchronized restart is the point: a moved slot re-enters in phase with the others, preserving the inviolable inter-slot sample alignment. (The prior implementation restarted only the changed slot live in the ISR, which clicked and left that slot phase-misaligned until the next full reset.) `change_pin` masks the channel's DMA IRQ before aborting the DMA so the handler can't start a conflicting transfer during PIO SM reinit, and clears any stale completion flag before unmasking. Back-to-back requests accumulate in the mask and apply in one batch. Persistence follows `output_config_mode`: in with-preset mode the pin travels with the preset (`REQ_PRESET_SAVE` captures it, applied on load); in independent mode it is device-global (`REQ_SAVE_OUTPUT_CONFIG` persists it, applied at boot).

**PDM:** Applied inline (must be disabled first; rebuilds PIO config). PDM has no running audio to realign at change time, so it needs no deferral or synchronized restart.

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
*Last updated: 2026-08-06 (input capture arena row; EQ cascade row: SVF pairs now fuse too)*

### Hardware

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| CPU | Dual Cortex-M0+ @ 133 MHz (OC to 307.2 MHz) | Dual Cortex-M33 @ 150 MHz (OC to 307.2 MHz) |
| SRAM | 264 KB | 520 KB |
| FPU | None (software float) | Single-precision VFP |
| DCP | N/A | Double-precision coprocessor |
| VREG | 1.20V (for OC) | 1.10V |
| UART + I2C external control | Yes (identical) | Yes (identical) |
| Control Surfaces nouns (caps v8) | 52 in table, `ADAT_ACTIVE` + the 6 upmixer nouns unusable (empty action mask) | 52, all usable |
| Control Surfaces IR sub-slots | 16 (`CS_MAX_IR_COMMANDS`) | 16 (identical) |
| Binary type | `default` (XIP) | `default` (XIP) |
| Cold code location (control paths, storage, coeff design, init) | Flash XIP | Flash XIP |
| RAM code+rodata+data (.data) | 44,376 B (was 108,692 under copy_to_ram) | 48,688 B (was 147,332 under copy_to_ram) |
| Free RAM | ~80,596 B (was ~10,476) | ~182,228 B (was ~75,540) |
| Custom XIP linker script | `memmap_dspi_rp2040_xip.ld` (+divider/int64/bit-ops IN_RAM defines) | `memmap_dspi_rp2350_xip.ld` |

### DSP Processing

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Data type | Q28 fixed-point | IEEE 754 float |
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
| USB input bit depth | 16-bit or 24-bit (alt) | 16/24-bit (stereo) or 16-bit (multichannel) |
| AS alt settings | 0, 1 (16-bit), 2 (24-bit) | 0, 1, 2, 3 (4ch), 4 (6ch), 5 (8ch) |
| Wire / slot version | V28 / V35 | V28 / V35 |
| S/PDIF bit depth | 24-bit | 24-bit |
| S/PDIF input conversion | 24-bit sign-extended full-scale → Q28 via `>> 2` (equivalent to `sample << 6`) | 24-bit sign-extended full-scale → float via `÷ 2147483648.0f` |
| S/PDIF output conversion | Q28 >> 6 → int24 | float × 8388607 → int24 |
| Volume leveller | Q28 envelope + float gain; 2 channels, 2 lookahead rings; max boost clamped to 18 dB (Q28 gain range) | Float throughout; 2 to 8 active channels, mask-driven, 8 lookahead rings; full 35 dB max boost |
| Loudness | Per output, post-gain: 2 Q28 shelf biquads; `loudness_output_mask` (5 outputs) | Per output, post-gain: 2 SVF shelves; `loudness_output_mask` (9 outputs). Both platforms: volume-keyed, works in stereo and multichannel input modes |
| Crossfeed | Per output pair, post-matrix (PASS 4.5); 2 pairs; `output_pair_mask` (default pair 1) | Per output pair, post-matrix (PASS 4.5); 4 pairs; `output_pair_mask` (default pair 1). Both platforms: shared coeffs, per-pair state, works in every input mode |
| Psychoacoustic bass | Per output, pre-crossover; RBJ Q28 biquads (with pre-drive low-band clamp) | Per output, pre-crossover; TPT SVF float. Both platforms: missing-fundamental NLD, `output_mask`, zero added latency |
| Stereo upmixer | Not available (compiled out; matrix untouched) | Stereo input only: derives C/Ls/Rs into matrix rows 2..4 (passive/adaptive/off centre; off/passive/adaptive surround). Zero-latency steering; deliberate per-row surround Haas delay |
| EQ channels | 7 (NUM_CHANNELS) | 11 (NUM_CHANNELS) |

### Delay Lines

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Channels | 5 | 9 |
| Type | int32_t | float |
| Max samples | 2048 | 2048 |
| Max delay (48kHz) | 42 ms | 42 ms |
| RAM usage | 40 KB | 72 KB |

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
*Last updated: 2026-07-13 (ADAT input RX ring on CH15, RP2350)*

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Priority | Global bus priority bits | Per-channel high-priority flag |
| Output TX channels (SPDIF **or** I2S, per slot) | 0-1 | 0-3 |
| SPDIF TX IRQ | DMA_IRQ_1 | DMA_IRQ_1 |
| I2S TX IRQ | DMA_IRQ_0 | DMA_IRQ_0 |
| SPDIF RX channels | CH4, CH5 | CH5, CH6 |
| SPDIF RX IRQ | DMA_IRQ_1 (shared with SPDIF TX) | DMA_IRQ_1 (shared with SPDIF TX) |
| I2S RX channels | CH4, CH5 (1 pair; shared with SPDIF RX) | CH5/6 + 7/8 + 9/10 + 11/12 (up to 4 pairs; pair 0 shared with SPDIF RX) |
| I2S RX IRQ | IRQ-less (chained ring) | IRQ-less (chained rings) |
| PDM channel | Dynamic (`dma_claim_unused_channel`) | Dynamic (`dma_claim_unused_channel`) |
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
DMA channels (RP2350: 7-15 after PDM=CH4 and RX=CH5/6) for input use — notably
multi-channel I2S input. S/PDIF TX and I2S TX deliberately sit on **different**
DMA IRQ lines (IRQ_1 vs IRQ_0); sharing a channel is still correct because each
library independently masks/unmasks its own `(dma_irq, channel)` enable bit and
de-registers from its instance registry on teardown, so only the active type's
handler ever services the channel. The brief windows where a channel is
unclaimed during a retype cannot race a `dma_claim_unused_channel` consumer
(PDM): retype runs synchronously on the main loop with both DMA IRQ lines
masked, and PDM claims its channel once at init.

### Clock Configuration

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| 48 kHz family | 307.2 MHz (VCO 1536 MHz / 5) | 307.2 MHz (VCO 1536 MHz / 5) |
| 44.1 kHz family | 264.6 MHz (VCO 1058.4 MHz / 4) | 264.6 MHz (auto PLL) |
| PLL config | Manual (`set_sys_clock_pll()`) | Automatic (`set_sys_clock_hz()`) |
| I2S BCK/LRCLK default (master + unified pair) | GPIO 14/15 | GPIO 14/15 |
| I2S slave-pair BCK/LRCLK default (SPLIT clock-pin mode; `PICO_I2S_BCK_PIN_SLAVE`) | GPIO 12/13 | GPIO 26/27 |

---

## Memory Layout
*Last updated: 2026-08-12 (Control Surfaces display: +~750 B BSS both platforms, ~12 KB flash; preset directory now 3035 B at V19)*

> **Input capture arena (2026-08-06).** The `pico_spdif_rx` FIFO (12 KB), the I2S
> RX rings (4 KB RP2040 / 32 KB RP2350) and the ADAT RX ring (8 KB, RP2350 only)
> are mutually exclusive and now overlay one BSS union, `input_capture_arena`
> (`.bss.input_capture_arena`, first in the alignment-sorted `.bss` cluster).
> Size is the largest member: **12,288 B on RP2040** (4096-aligned) and
> **32,768 B on RP2350** (8192-aligned), reclaiming 4,092 B and 20,480 B of BSS
> respectively. The explicit section keeps it out of COMMON, where its alignment
> padding would eat most of the saving. Ownership is a runtime-checked invariant;
> see "Input Capture Arena" under Audio Input Source System.

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
> The ADAT engine adds a fixed 28 KB BSS frame ring (896 frames x 32 bytes,
> `adat_ring` in adat_output.c) plus small state; the ring is sized to cover
> the alignment cushion (96 frames) plus the blocking-give fill cap (16 x 48
> samples) so overwrite of unplayed frames is structurally impossible. The
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

> **Static consumer pools (2026-05-31).** The per-output-slot consumer buffer pool
> is now a single statically-allocated (BSS) pool per slot, **shared by the slot's
> S/PDIF and I2S instances and reused across output-type switches** — sized for the
> largest type (S/PDIF, stride `PICO_AUDIO_SPDIF_CONSUMER_FRAME_BYTES` = 16; I2S
> under-fills each 768-byte block). Previously each output-type instance malloc'd
> (and the I2S side freed) its own pool, so a slot that had been both types held two
> pools and a both-slots-I2S config on RP2040 overran the ~60 KB heap (`malloc`→NULL
> crash). The shared static pool removes the second allocation, makes the footprint
> deterministic and link-time-budgeted (a BSS overflow now fails the build, not the
> field), and eliminates retype heap churn/fragmentation. Built via
> `audio_consumer_pool_init_static()` at boot and re-pointed per connect via
> `audio_consumer_pool_reformat()` (pico_audio); `*_connect_extra()` now take the pool
> as a parameter. Producer pools remain heap (allocated once at boot, never churn).
> Per-instance silence buffers are likewise static (embedded in the instance struct).

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
> (cancelled/re-armed per block by the SPDIF RX ISR; `clock_get_hz` and the MCK
> divider sit on the clock-servo path), the notify ring (drained from the USB
> ISR while parameters change), the float/double/mem shims (`PICO_FLOAT_IN_RAM`,
> `PICO_DOUBLE_IN_RAM`, `PICO_MEM_IN_RAM` both platforms; `PICO_DIVIDER_IN_RAM`,
> `PICO_INT64_OPS_IN_RAM`, `PICO_BITS_IN_RAM` RP2040 only), and the entire
> Core-1 execution set.
> Newly attribute-marked hot functions include `fb_ctrl_sof_update` /
> `fb_ctrl_get_10_14` (per-SOF ISR), `spdif_input_update_clock_servo` and the
> SPDIF RX stable/lost callbacks, the full PDM Core-1 set (`pdm_core1_entry`,
> `pdm_processing_loop`, `pdm_push_sample`, `pdm_get_dma_fill_pct`,
> `pdm_get_ring_fill_pct`), `usb_audio_drain_ring` / `usb_audio_flush_ring`,
> `update_buffer_watermarks` / `get_slot_consumer_fill`, and the pico-extras
> spdif_rx DMA-ISR callees (`_check_block`, `_dma_done_and_restart`,
> `spdif_rx_callback_func`). The pico-extras `pico_audio` `AUDIO_TIME_CRITICAL`,
> `pico_audio_spdif_multi` `SPDIF_TIME_CRITICAL`, and `pico_audio_i2s_multi`
> `I2S_TIME_CRITICAL` attributes now apply on both platforms (all were
> RP2350-only, leaving the per-block producer/consumer and sample-encode
> functions in flash on RP2040); `audio_i2s_mck_set_divider` (clock-servo path)
> is also `I2S_TIME_CRITICAL`. The RP2040
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

### RP2040 (264 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (5 × 2048 × 4) | 40 KB |
| Output buffers (5 × 192 × 4 + 2 × 192 × 4) | ~5.25 KB |
| Filters + recipes (7 channels) | ~8 KB |
| Crossover filters + recipes (7 × 4 × Biquad + recipes) | ~4.1 KB |
| Loudness tables (2 × 61 × 2 × ~13B) | ~3 KB |
| Loudness per-output state (5 × 16 B) | 80 B |
| Preset system (dir_cache + slot_buf + write_buf) | ~6 KB |
| Bulk param buffer (8 KB aligned, holds V15 = 4128 B) | ~8 KB |
| `notify_rebaseline` static scratch (V15 WireBulkParams) | ~4.1 KB |
| USB audio ring buffer (4 × 578) | ~2.3 KB |
| Channel names (7 × 32) | ~224 B |
| Leveller state + lookahead (2 rings × 240 × 4) | ~1.9 KB |
| Per-channel preamp + master volume | ~48 B |
| Consumer pools + silence (static, 2 slots × 16 × 48 × 16, shared SPDIF/I2S) | ~27 KB |
| I2S RX DMA ring (1024 × 4, 4 KB aligned) | 4 KB |
| Other BSS | ~20 KB |
| **Total BSS** | **~132 KB** (measured: 134,932 B; unchanged by the XIP migration) |
| RAM code+rodata+data (.data section, hot set only) | 44,376 B (was 108,692 under copy_to_ram) |
| Flash-resident code (.text + .rodata + boot2, XIP) | ~98 KB |
| Free RAM | ~80,596 B (was ~10,476 under copy_to_ram) |
| SPDIF producer pools (heap, 2 × 8 × 192 × 8) | ~24 KB |
| Stack + remaining heap | drawn from the free-RAM pool above |

### RP2350 (520 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (9 × 2048 × 4) | 72 KB |
| Filters + recipes | ~18 KB |
| Crossover filters + recipes (11 × 4 × Biquad + recipes) | ~15 KB |
| Output buffers (9 × 192 × 4) | ~7 KB |
| Input buffers (buf_l + buf_r + buf_in_ext[6][192], 8-channel USB) | ~6 KB |
| Preset system (dir_cache + slot_buf + write_buf) | ~7 KB |
| Bulk param buffer (8 KB aligned, holds V15 = 4128 B) | ~8 KB |
| `notify_rebaseline` static scratch (V15 WireBulkParams) | ~4.1 KB |
| USB audio ring buffer (4 × 794, 8-channel packets) | ~3.2 KB |
| Channel names (11 × 32) | ~352 B |
| Leveller state + lookahead (8 rings × 240 × 4) | ~7.7 KB |
| Per-channel preamp (8 ch) + master volume | ~120 B |
| Matrix mixer (8 × 9 crosspoints + outputs) | ~1.3 KB |
| Loudness tables + per-output state (2 × 61 × 2 coeffs + 9 × 16 B) | ~3 KB |
| Consumer pools + silence (static, 4 slots × 16 × 48 × 16, shared SPDIF/I2S) | ~55 KB |
| I2S RX DMA ring (2048 × 4, 8 KB aligned) | 8 KB |
| ADAT RX ring (`adat_rx_ring`, 2048 × 4, 8 KB aligned) | 8 KB |
| Stereo upmixer state (Haas 2 × 1024 + allpass 2 × 512 floats + estimators + double-buffered coeffs) | ~12.3 KB |
| Other BSS | ~35 KB |
| **Total BSS** | **~346 KB** (measured 353,884 B after the stereo upmixer, + ~12.3 KB over the prior build. RP2040 unchanged: the feature is compiled out and the matrix is untouched) |
| RAM code+rodata+data (.data section, hot set only) | 84,360 B (was 147,332 under copy_to_ram; +5,256 B for the fused SVF pair arms, 2026-08-06). Over the 73,728 B `check_ram_placement.py` budget, which now fails by 10,632 B; the budget has not been re-cut since the ADAT input receiver raised it to 72 KB |
| Flash-resident code (.text + .rodata + boot2, XIP) | ~98 KB |
| Free RAM | 103,964 B (per scripts/check_ram_placement.py, includes vector table + 2 KB heap reserve accounting) |
| SPDIF producer pools (heap, 4 × 8 × 192 × 8) | ~48 KB |
| Stack + remaining heap | drawn from the free-RAM pool above |

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
| S/PDIF DMA transfer | 48 samples (1 ms at 48 kHz) |
| S/PDIF consumer pool | 16 buffers × 48 samples per output pair |
| S/PDIF producer pool | 8 buffers × 192 samples per output pair |
| PDM DMA ring | 2048 words |
| PDM sample ring | 256 entries (Core 0 → Core 1) |

### Latency (at 48 kHz)

| Path | Latency |
|------|---------|
| USB → S/PDIF | ~8 ms mean (16 × 48-sample buffers at 50% fill) |
| S/PDIF latency jitter | ±1 ms (±1 buffer of 48 samples) |
| S/PDIF → PDM alignment | +2.67 ms (+128 samples) |
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

## Channel Metering
*Last updated: 2026-03-01*

Full peak metering for all input and output channels. Peak values are `uint16_t` in Q15 format (0–32767 maps to 0.0–1.0 full scale). Per-channel clip detection via sticky `clip_flags` bitmask.

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
*Last updated: 2026-03-01*

`global_status.clip_flags` is a `uint16_t` bitmask — one bit per channel (bit position = channel index). A bit is **set** when the block peak exceeds the clip threshold (`CLIP_THRESH_F` = 1.001f on RP2350, `CLIP_THRESH_Q28` = (1<<28)+268 on RP2040). The threshold includes ~+0.01 dB headroom above unity to avoid false positives from float precision noise when 0 dBFS signals pass through biquad filters. Bits are **sticky**: once set, they remain set until explicitly cleared by the host via `REQ_CLEAR_CLIPS` (0x83). The firmware never autonomously clears clip flags.

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
*Last updated: 2026-08-24 (input-source stepping skips unselectable sources; caps v13: display level bars; caps v12: per-LED PWM brightness ceiling; caps v11: display line alignment and edit markers; caps v10: I2C display component, IR group support, nouns 53-56, commands 0x27-0x2B, directory V19)*

User-wired push buttons, toggle switches, potentiometers, quadrature rotary
encoders, plain indicator LEDs, PWM-dimmed LEDs, an IR remote receiver, and an
I2C character/OLED display on
spare GPIOs, configured over vendor commands `0x84`-`0x87`, `0x8B`/`0x8C`
(per-slot names), `0x8D`-`0x8F` (IR commands and learn), `0x9D`/`0x9E`
(save/revert), `0x20`-`0x26` (target groups and macros), and `0x27`-`0x2B`
(display config, pages and status). A binding attaches
one component (`CsType`)
to one firmware parameter (`CsNoun`) through one operation (`CsAction`), on one
or two GPIOs. The full integrator spec is
`Documentation/Features/control_surfaces_spec.md`, with groups and macros
specified in `Documentation/Features/control_surfaces_groups_macros_spec.md`
and the display in `Documentation/Features/control_surfaces_display_spec.md`.

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
channel are rejected at apply time (`CS_STATUS_PWM_CONFLICT`).

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
the directory cache (`control_surfaces_revert`: clears every slot through the
normal release/claim path, then re-runs the boot loader `cs_load_stored`,
which also reloads the stored names) with no flash write. A reboot is an
implicit revert.

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
*Last updated: 2026-08-24 (alignment-aware edit marker placement; caps v13: level bars; caps v11: per-line alignment, edit markers)*

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
merged line nor a bar row takes the configured alignment.

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

### Persistence and platform placement

The binding table is device-global in the preset directory (388-byte
`CsFlashConfig`: version + 16x 24-byte `CsBinding`), board-level like
`dac_hw_mute` and the control-interface config; it survives preset changes and
factory reset and is not part of `WireBulkParams`. The per-slot names live
next to it (V10, `cs_names[16][32]`) and the IR command table follows (V11,
132-byte `CsIrConfig`: version + 8x 16-byte `IrCommand`), with the group table
(V18, 324-byte `CsGroupConfig`), macro table (V18, 1060-byte `CsMacroConfig`)
and display blob (V19, 80-byte `CsDisplayFlash`) appended last, all with the
same lifetime. On RP2040
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
RP2040 like the rest of the engine.

---

## Vendor Command Reference
*Last updated: 2026-09-01 (REQ_GET_BUILD_INFO 0x80 added: 64-byte git/date build stamp)*

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
| REQ_GET_STATUS | 0x50 | IN | Get all channel peaks + CPU load (see Channel Metering) |
| REQ_SAVE_PARAMS | 0x51 | OUT | Save all params to flash |
| REQ_SAVE_OUTPUT_CONFIG | 0x52 | IN | Persist live physical IO config to the directory's device-global block (independent mode; was the deprecated REQ_LOAD_PARAMS) |
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
| REQ_GET_PLATFORM | 0x7F | IN | Get platform ID, fw version (legacy nibbles + full-width minor/patch), output count; 6 bytes |
| REQ_GET_BUILD_INFO | 0x80 | IN | Get 64-byte build stamp: git describe [0..47] + build date [48..59]; provenance only, never gated on |
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
| SET_MCK_ENABLE | 0xC4 | OUT | Set MCK enable |
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
| REQ_SET_I2S_RX_PIN | 0xF1 | IN* | Set I2S RX data GPIO pin (wValue=(pair<<8)\|pin, returns status; pin 0xFF = reset that pair to its default) |
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
*Last updated: 2026-08-02 (wire V28: input-config `spdif_rx_pin_ext` grows to 3 entries for SPDIF input 4; section and total size unchanged)*

Transfers the complete DSP state in a single USB control transfer (3664 bytes at V11/V12), replacing dozens of individual vendor requests.

**Wire format:** `WireBulkParams` (`bulk_params.h`, `WIRE_FORMAT_VERSION` 25, total 5944 bytes); packed struct with header, global params, crossfeed, legacy channel gains, delays, matrix crosspoints, matrix outputs, pin config, EQ bands, channel names, I2S config, leveller config, preamp config (`WirePreampConfig`, 16 bytes), master volume config (`WireMasterVolume`, 16 bytes), input source config (`WireInputConfig`, 16 bytes), LG Sound Sync (`WireLgSoundSync`, 16 bytes), user volume/mute (`WireUserVolume`, 16 bytes), DAC hardware mute (`WireDacHwMute`, 16 bytes, V10+), and **crossover bands** (`WireCrossoverConfig`, 704 bytes = 11 × 4 × `WireBandParams`, V11+). V12 claims two reserved bytes inside `WireInputConfig` for `i2s_rx_pin` and `i2s_input_rate` (enum 0=44100, 1=48000, 2=96000); V12 payloads are byte-identical in size to V11. All arrays sized at platform maximums (RP2350: 11 channels, 9 outputs, 5 pins, 12 PEQ bands, 4 crossover bands per channel). Unused entries zero-padded; for crossover, master rows (channel < `CH_OUT_1`) are zeroed on collect and skipped on apply. **V20** repurposes the `WireCrossfeedParams` reserved byte (offset 3) as `output_pair_mask` (bit p = crossfeed on output pair p); struct sizes are unchanged. **V22** carries the Linkwitz-Transform target `Q` in the EQ `WireBandParams.reserved[2]` bytes (`uint16` LE, `Q*512`; zero for non-LT types), so struct sizes stay unchanged. (V21 claimed one `WireInputConfig` reserved byte for the I2S clock master/slave mode, also size-neutral.) **V23** tail-appends the 24-byte `WirePsybassParams` (psychoacoustic bass: `enabled` + `output_mask` + five floats), bringing the total to 5900 bytes. **V24** claims three `WireInputConfig` reserved bytes for the ADAT input (`adat_input_pin`, `adat_input_enabled_p1`, `adat_clock_mode_p1`, each 0 = absent/keep-live); struct sizes and the 5900-byte total are unchanged. **V25** tail-appends the 44-byte `WireUpmixParams` (RP2350 stereo upmixer: enabled + centre/surround modes + reserved + ten floats; layout-identical to `UpmixConfigPacket`), bringing the total to 5944 bytes; the section is zeroed on collect and ignored on apply on RP2040. **V28** widens `WireInputConfig.spdif_rx_pin_ext` from 2 to 3 entries (SPDIF input 4), consuming that section's last reserved byte and shifting `spdif_rx_enabled_ext_p1`, `i2s_clock_mode` and the ADAT input fields down one byte; the section stays 16 bytes and the 5944-byte total and every later section offset are unchanged. The input-config section now has no reserved bytes left.

**Per-version size anchors** live in `bulk_params.h` (`WIRE_BULK_PARAMS_V{N}_SIZE`, N=2..12). Each legacy-section apply gate inside `bulk_params_apply()` compares `payload_length` against its own version's anchor, NOT against `sizeof(WireBulkParams)`. Without this discipline, growing the struct would silently lock older payloads out of the very tail sections they own (e.g. a V10 payload would stop applying its DAC-mute section the moment V11 was added). V<11 payloads leave crossover state untouched on apply; V<12 payloads leave the I2S input pin/rate untouched.

**Transport:** Multi-packet USB EP0 control transfers using `usb_stream_transfer` from pico-extras. Packets are 64 bytes. No modifications to `usb_device.c` required — uses only public API (`usb_stream_setup_transfer`, `usb_start_transfer`, `usb_start_empty_transfer`).

**GET (0xA0):** `bulk_params_collect()` snapshots live state into `bulk_param_buf`, then streams it out in 64-byte packets via `usb_stream_transfer`. ZLP appended if total length is a multiple of 64.

**SET (0xA1):** Incoming data accumulated into `bulk_param_buf` via `usb_stream_transfer`. On completion, `bulk_params_pending` flag is set (after status-phase ACK). Main loop processes deferred: snapshots `output_types[]`, waits for Core 1 idle, mutes audio (256 samples), calls `bulk_params_apply()` with pin application gated on `output_config_mode` (applied only in with-preset mode; independent mode leaves device-global IO to `REQ_SAVE_OUTPUT_CONFIG`), recalculates all filters and delays, transitions Core 1 mode to match the new output enable state, then diffs the new `output_types[]` against the snapshot. If any slot's type changed, dispatches `process_type_switches()` to reconfigure SPDIF/I2S hardware (mirrors the `preset_load_pending` pattern); otherwise calls `complete_pipeline_reset()` to resync output streams (or `reset_usb_feedback_loop()` when SPDIF input is active, to avoid disrupting the prefill handshake).

**Buffer:** 4 KB aligned static buffer in `usb_audio.c`, shared between GET and SET. Platform validation rejects mismatched `platform_id` or `num_channels`.

### Buffer Statistics
*Last updated: 2026-03-19*

Real-time buffer fill level monitoring for SPDIF consumer (DMA-side) pools and PDM buffers, accessible via USB vendor commands. Enables host applications to diagnose audio glitches, near-miss underruns, and pipeline health. Producer (USB-side) pool stats are not tracked because `producer_pool_blocking_give` returns buffers synchronously — the producer pool is always fully free between USB packets.

**Wire format:** `BufferStatsPacket` (44 bytes, fits in a single 64-byte USB control transfer). Contains per-instance SPDIF consumer stats (`SpdifBufferStats` x4, 8 bytes each), PDM stats (`PdmBufferStats`, 8 bytes), instance count, flags (PDM active, audio streaming), and a monotonic sequence counter.

**SPDIF stats per instance:** consumer free/prepared/playing counts with fill percentage and min/max watermarks (DMA-side pool).

**PDM stats:** DMA circular buffer fill percentage and software ring buffer fill percentage, each with min/max watermarks.

**Fill percentage formulas:**
- SPDIF consumer: `(prepared + playing) * 100 / SPDIF_CONSUMER_BUFFER_COUNT` — healthy: 25-75%
- PDM DMA: `((write_idx - read_idx) & (PDM_DMA_BUFFER_SIZE-1)) * 100 / PDM_DMA_BUFFER_SIZE` — healthy: ~12.5%
- PDM ring: `((head - tail) & 0xFF) * 100 / RING_SIZE` — healthy: 0-10%

**Producer fill formula:** `(capacity - free) * 100 / capacity` — measures in-flight + prepared buffers, since `prepared` alone is always near zero (DMA IRQ drains it on-demand via the connection).

**Watermark tracking:** Consumer watermarks updated once per USB audio packet (~1ms) in `process_audio_packet()`. Overhead ~1-2us (consumer pool list traversals under spinlock). Reset via `REQ_RESET_BUFFER_STATS` (0xB1, wValue bit 0).

**Implementation:** `audio_buffer_list_count()` inline in `pico/audio.h` for read-only list traversal. `pdm_stats_write_idx` volatile in `pdm_generator.c` exposes Core 1 write position to Core 0 (atomic on ARM). Helper functions in `usb_audio.c`: `count_pool_free()`, `count_pool_prepared()`, `update_buffer_watermarks()`, `reset_buffer_watermarks()`.

**BSS impact:** ~18 bytes total (watermark arrays + sequence counter + pdm_stats_write_idx).

---

## I2S Output Support
*Last updated: 2026-04-09*

### Overview

Each output slot can be independently configured as S/PDIF or I2S at runtime via vendor commands. A new `pico_audio_i2s_multi` library mirrors the proven `pico_audio_spdif_multi` patterns. The S/PDIF library is completely unchanged.

### Architecture

- **PIO0:** Both S/PDIF (4 instructions) and I2S (8 instructions) programs coexist in instruction memory (12/32 slots). Each SM's side-set pins are independent — S/PDIF side-set = data pin, I2S side-set = BCK/LRCLK.
- **MCK:** Generated by hardware **CLK_GPOUTn** (clock peripheral output), not PIO. No state machine consumed — see "Master Clock (CLK_GPOUTn)" below. **PIO1 SM1 is now free** (previously the MCK toggle program); reserved for future use.
- **OutputSlot abstraction** in `usb_audio.c` manages per-slot type, holding either a SPDIF or I2S instance.
- **DMA IRQ:** S/PDIF TX uses DMA_IRQ_1 (dedicated). I2S TX uses DMA_IRQ_0 (shared with SPDIF RX when active). Both register via `irq_add_shared_handler()`.
- **Producer pools** are format-identical (PCM_S32, stride 8). The I2S library's connection callback left-shifts samples by 8 for MSB-first I2S framing, then ORs `I2S_PAD_PATTERN` (0x01) into the unused bottom 8 padding bits — the 24-bit audio at [31:8] stays bit-perfect, but the 32-bit wire word is never zero. This defeats DAC zero-detect mute on chips configured for 32-bit input (PCM5102, ES9018/9038, CS43198, AKM in 32-bit mode), eliminating the auto-mute click on quiet content / stream gaps. 24-bit-mode DAC configs ignore the padding bits and are unaffected. The I2S silence buffer (substituted on consumer-pool underrun) and the consumer-pool pre-fill at `audio_i2s_connect_extra()` are filled with the same pattern so the DAC sees non-zero frames during DMA underruns and on first stream start. *Last updated: 2026-05-06*
- **No audio callback changes.** Core 1 remains output-type-agnostic.
- **Pipeline reset API** (`main.c`): two-phase `prepare_pipeline_reset()` / `complete_pipeline_reset()` brackets any disruptive output work. `complete_pipeline_reset()` is structured to keep the IRQ-disabled critical section tiny — only the `audio_*_enable_sync()` calls need atomicity (so all output slots start their PIO SMs on the same clock cycle, preserving CLAUDE.md's slot-alignment invariant). Per-slot teardown (`teardown_output_slot()` helper) runs with main interrupts ENABLED: the per-instance `enabled` flag is set false first (the shared DMA IRQ handler skips disabled instances), the channel's DMA IRQ is masked, and the pool/abort operations only touch this instance's state. The USB audio class ISR can continue to drain packets into the SPSC `audio_ring` throughout the teardown, eliminating the prior ~1 ms USB starvation that compounded I2S DAC pops during input-source switches. Core 1 is held idle by `prepare_pipeline_reset()` (spin-wait for `work_done`) and `preset_loading=true` (blocks new dispatch). `drain_and_disable_outputs()` shares the same `teardown_output_slot()` helper. I2S→S/PDIF switch restores the SPDIF connection before zeroing the I2S instance to prevent a dangling `producer_pool->connection` pointer. *Last updated: 2026-05-12*
- **Boot-time I2S restoration:** `core0_init()` inspects `output_types[]` after `preset_boot_load()` and converts preset-saved I2S slots from the default SPDIF instances created by `usb_sound_card_init()`. For each I2S slot: disables SPDIF, unclaims the PIO SM, calls `audio_i2s_setup()` + `audio_i2s_connect_extra()`, and enables the I2S instance. MCK is started if any I2S slot exists and `i2s_mck_enabled` is set.
- **MCK pin migration on preset apply:** Preset and bulk-params apply paths validate the loaded `i2s_mck_pin` against `GPIO_TO_GPOUT_CLOCK_HANDLE()`. If the stored pin has no GPOUTn mapping on the current platform (typical case: an RP2040 board loading a preset saved with `mck_pin = 13`, which is GPOUTn-capable on RP2350 only), the pin falls back to `PICO_I2S_MCK_PIN` (platform default) and `i2s_mck_enabled` is forced to `false`. Defaults: GPIO 13 on RP2350, GPIO 21 on RP2040 (the only board-friendly GPOUTn pin on RP2040 — GPIOs 23–25 are also GPOUTn-capable but reserved for control / SMPS / LED).
- **MCK enable order:** `process_type_switches()` writes the MCK clkdiv via `audio_i2s_mck_update_frequency()` *before* calling `audio_i2s_mck_set_enabled(true)`, so the CLK_GPOUTn block starts at the correct frequency rather than briefly running at whatever DIV value the block previously held (which would cause a transient PLL-relock chirp on connected DACs). Matches the `REQ_SET_MCK_ENABLE` vendor command order.
- **SPDIF RX is suspended across `process_type_switches()`:** the function shares `DMA_IRQ_1` between SPDIF TX and `pico_spdif_rx`. Forcing the IRQ off mid-transition would silence RX DMA completion handling for the duration; RX decode-timeout alarms (separate timer IRQ) could also fire and access PIO/DMA state being mutated. The function snapshots RX state at entry, calls `spdif_input_stop()` if it was running, and restarts it after `complete_pipeline_reset()` finishes — guarded by `active_input_source == INPUT_SOURCE_SPDIF && !input_source_change_pending` so a deferred input-source switch isn't pre-empted. Callers that have already stopped RX (e.g. `preset_load_pending` across the flash blackout) see no double-stop because `state == INACTIVE` triggers the no-op path; those callers are responsible for their own restart, which `preset_load_pending` now does once at the end of its block rather than before `process_type_switches`.

*Last updated: 2026-05-12*

### Master Clock (CLK_GPOUTn)
*Last updated: 2026-05-09*

- **No PIO state machine consumed.** MCK is generated by one of the four hardware **CLK_GPOUTn** clock outputs (`clk_gpout0..3`). The previous 2-instruction PIO toggle program (`audio_mck.pio`) is gone; PIO1 SM1 is free for future use.
- **Library API** (`audio_i2s_multi.c`, MCK section): `audio_i2s_mck_setup(pin)` — record pin only, no hardware effect; `audio_i2s_mck_set_enabled(bool)` — enable routes pad mux + loads divider via `clock_gpio_init_int_frac8(pin, AUXSRC=clk_sys, int, frac8)`, disable disconnects pad mux (generator continues running internally; no public SDK API stops it); `audio_i2s_mck_update_frequency(Fs, mult)` — recomputes 24.8 divider, hot-loads if running; `audio_i2s_mck_change_pin(pin)` — pure book-keeping (asserts `!mck_running`); `audio_i2s_mck_set_divider(div_24_8)` — raw divider write for SPDIF clock servo.
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
*Last updated: 2026-07-13 (LUT-driven NRZI encoder; single float->S24 conversion point shared with the slots)*

RP2350 only. Streams all 8 main output channels (post-matrix, post per-output
EQ/gain/delay/mute, the same finalized samples the SPDIF/I2S slots receive) as
one ADAT lightpipe signal on a single GPIO (default 12), fully concurrent with
the four output slots and PDM. 44.1/48 kHz only; the stream auto-suspends at
higher rates and auto-resumes when the rate returns. RP2040 compiles the
feature out (2 output channel pairs only).

**Engine** (`adat_output.c/h`): NRZI is encoded on the CPU, so the PIO program
is a single `out pins, 1` at 256 x Fs and the clkdiv is the IDENTICAL value the
S/PDIF TX SMs run. In USB/I2S modes both use the nominal `ceil(sys / Fs)`; in
SPDIF-input mode the clock servo (`spdif_input_update_clock_servo`) writes ADAT
the same servoed divider it writes the SPDIF slots (`adat_output_servo_divider`,
sanity-bounded against nominal; resync pulls `spdif_input_current_tx_divider()`),
so ADAT is rate-locked to the slots with zero long-term drift in every input
mode. Frames (256 bits: `[1][10x0][1][u3..u0]` then 8 x 6 x `[1][nibble]`, user
bits 0) are encoded on Core 0 in `process_input_block()` after the slot gives
and written into a 896-frame (28 KB BSS) ring drained by DMA CH13, with CH14 as
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
rejected (the servo dithers between adjacent possibly-odd dividers, and a
two-mode encoder would need mid-stream resync); the encode LUT is intentionally
not shared with the planned ADAT input, whose NRZI decode is `x ^ (x >> 1)` and
whose destuffing is two masks, needing no LUT.

**Alignment:** pushing after the blocking gives makes the ring lead track the
slot-0 consumer fill plus a constant 96-frame cushion; the blocking-give cap
(16 x 48 samples) bounds the lead under the ring size, so overwrite of unplayed
frames is impossible. The 96-sample (2 ms at 48 kHz) ADAT-to-slot offset is
re-established at every synchronized output restart (`complete_pipeline_reset`
Phase 6, `enable_outputs_in_sync`). During host underruns silence frames are
inserted slaved 1:1 to slot 0's DMA starvation counter (SPDIF or I2S; the I2S
multi library gained matching counters), so the offset survives underruns while
a USB stream is open; counter resets are detected by delta bounds and only
re-baseline. The existing slots' sync-start machinery is untouched: ADAT is a
PDM-style independent consumer, never a member of the Phase 2 sync start.

A flash window is the one case where mirroring is the wrong recovery: the slots
keep clocking (and counting starvations) through the erase/program, but ADAT's
free-running ring also kept emitting a frame per sample clock, so it is not
behind by the starvation count. `complete_flash_write_operation_full()` calls
`adat_output_rebaseline_starvations()` to drop that backlog without inserting
for it, and the following resync re-establishes the canonical cushion. See
"Selective NVIC blackout" under Flash Storage. *Last updated: 2026-07-25*

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
*Last updated: 2026-08-06 (input capture arena + receiver exclusivity invariant; see "Input Capture Arena")*

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

This reclaims 4,092 B of BSS on RP2040 and 20,480 B on RP2350. Each module aliases its old symbol name onto its member with a macro, so all other references are unchanged, and each static-asserts its own ring constants against the arena geometry. The arena's alignment equals one I2S row, which is what keeps every row inside its own DMA write-address wrap region; the ADAT ring wraps on the whole 8 KB member. The SPDIF FIFO is chained ping-pong DMA (no address wrap) and needs only 4-byte alignment. Buffers deliberately **not** in the arena: `adat_ring` (ADAT *output*, runs with any input), the consumer pools, `pdm_dma_buffer`, and the USB `audio_ring`.

**Ownership invariant.** At most one receiver runs, and only the one the active source owns. `input_arena_claim()` panics if the arena is already held by a different owner; each `*_input_start()` claims (USB claims nothing) and each `*_input_stop()` releases. Two enforcement points keep the invariant true rather than merely asserted:

- `stop_foreign_input_receivers(keep)` (main.c) stops every receiver that is *actually running* but is not `keep`, keyed on running state rather than on the old source enum. It runs once per main-loop iteration, at the top of the deferred source switch (with the incoming source as `keep`, making the per-branch stops no-ops), and at the restart points of preset load, factory reset and bulk apply.
- Those three paths matter because `apply_factory_defaults()` writes `active_input_source = INPUT_SOURCE_USB` directly, bypassing the deferred switch handler; before this, a factory reset / empty-slot preset load / active-slot preset delete while ADAT was live left the ADAT receiver running (ENDLESS DMA into its ring) with no path that would ever stop it, since the switch handler's stop chain keys on the old source. SPDIF and I2S were already interlocked by their shared DMA channels and PIO SM; ADAT has its own (DMA 15, PIO1 SM2), so only convention protected it.

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

`audio_set_volume()` (`usb_audio.c:350`) always records the host's last-set value into `audio_state.volume` so GET_CUR round-trips correctly, but bails out before touching `audio_state.vol_mul` or `current_loudness_coeffs` when source isn't USB. Mute application is gated symmetrically in the audio pipeline: `audio_pipeline.c:197` (RP2350 float) and `audio_pipeline.c:499` (RP2040 Q15) both guard `audio_state.mute` with `active_input_source == INPUT_SOURCE_USB`. Result: Windows volume slider and mute key have no audible effect during SPDIF playback.

The SPDIF→USB transition in the deferred input-source switch handler (`main.c:1597-1604`) calls `audio_set_volume(audio_state.volume)` to thaw the cached host volume into the live gain path so Windows' last-seen slider position takes effect immediately when the user switches back.

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
*Last updated: 2026-05-19*

**Library**: Forked from `elehobica/pico_spdif_rx` v0.9.3 at `firmware/pico-extras/src/rp2_common/pico_spdif_rx/`.

**DSPi library patches:**
- PIO2 support for RP2350
- Clock constants: 307.2 MHz sys_clk, 122.88 MHz PIO clock (divider 2.5 exact)
- Removed `pio_clear_instruction_memory()` (destroys shared PIO programs)
- Removed `irq_set_enabled(DMA_IRQ_x, false)` (disables entire shared IRQ line)
- Replaced `irq_has_shared_handler()` with private `irq_handler_registered` flag (prevents handler registration when other libraries share the IRQ line)
- Added `irq_remove_handler()` in `spdif_rx_end()` for clean lifecycle
- Added `save_and_disable_interrupts()` in `_spdif_rx_common_end()` to prevent re-entrant teardown

**PIO Allocation**:
- RP2350: PIO2 SM0 (dedicated block, no conflicts)
- RP2040: PIO1 SM2 (SM0=PDM occupies SM0 when active; SM1 was MCK pre-GPOUTn refactor and is now free — the patched `pio_clear_instruction_memory()` removal in the SPDIF RX library is still required because PDM and the I2S libraries share PIO program memory regardless of the MCK move)

**Clock**: sys_clk 307.2 MHz → PIO clock 122.88 MHz (divider 2.5, exact). At 122.88 MHz: cy=20 (48kHz), cy=10 (96kHz), cy=5 (192kHz) — identical to original library values, zero error.

**DMA**: Channels 5+6 (RP2350) or 4+5 (RP2040) on DMA_IRQ_0 (shared with I2S TX when active). DMA_IRQ_1 is dedicated to SPDIF TX only. This isolates SPDIF RX from SPDIF TX, avoiding shared handler conflicts.

**State Machine**: `spdif_input.h/c`
- INACTIVE → ACQUIRING → LOCKED → RELOCKING (on signal loss) → LOCKED (on re-lock)
- Lock: ~64ms library internal stability + firmware debounce polls
- Loss: 10ms timeout
- Audio extraction: FIFO → 24-bit decode → per-channel preamp → buf_l/buf_r → process_input_block(). RP2040 scales decoded samples into Q28 with the same `sample << 6` full-scale convention as USB 24-bit input; RP2350 scales to float full-scale.

**Clock Servo**: PI controller in `spdif_input_update_clock_servo()` adjusts all output PIO dividers based on FIFO fill level (target 50%). Gains: KP=0.0005, KI=0.000005, deadband ±2 blocks. MCK divider is servoed alongside I2S data SM dividers when MCK is enabled, using `audio_i2s_mck_set_divider()` to keep master clock frequency-locked to the servoed output rate. The divider-math and PIO/MCK-write actuation body (all output slots, the ADAT output, and MCK, with a proportional fill trim) is now factored out into `input_servo.c` (`input_servo_apply()` / `input_servo_reset()` / `input_servo_current_divider()`) and shared verbatim by SPDIF input and ADAT slave-clock mode; callers own their own lock gating, rate limiting, and input-rate measurement. I2S slave mode keeps its own servo variant (`i2s_slave_update_clock_servo()`), referenced to the first SPDIF-type slot rather than a consumer FIFO. *Last updated: 2026-07-13*

**Output Prefill**: On SPDIF lock acquisition, outputs are disabled and consumer buffers drained via `drain_and_disable_outputs()`. The pipeline then feeds real audio into consumer buffers while outputs are stopped. Once slot 0 consumer fill reaches 50% (8 of 16 buffers), outputs are started in sync via `enable_outputs_in_sync()`. This eliminates initial underruns after lock acquisition. Controlled by `spdif_prefilling` flag in `main.c`. *Last updated: 2026-04-12*

**Files**: `spdif_input.h` (API + status struct), `spdif_input.c` (lifecycle, audio extraction, clock servo, status queries)

### I2S Input
*Last updated: 2026-07-15 (pin byte 0xFF on the pin SETs = reset the addressed target to its default)*

In the default MASTER clock mode (`i2s_clock_mode` = 0) I2S input keeps the device as the clock authority: the external source slaves to our BCK/LRCLK, the same shared clock pair the I2S outputs use (`i2s_bck_pin` / +1; master clocking always uses this pair regardless of clock-pin mode). Because the input is then synchronous to the device clock domain, there is **no clock servo, no rate detection and no lock state machine**; the subsystem is structurally a much simpler sibling of SPDIF RX. State is just INACTIVE / RUNNING. In SLAVE clock mode (`i2s_clock_mode` = 1) an EXTERNAL master drives BCK/LRCLK instead; that mode adds rate detection, a lock state machine and an output clock servo, described in the "I2S Clock-Slave Input Mode" section below. Everything in THIS section describes master mode unless noted.

**Multichannel input (RP2350).** The receiver fans out to up to `I2S_RX_MAX_PAIRS` (4) stereo pairs — **2/4/6/8 channels**, selected by `i2s_input_channels` — each one PIO state machine + one IRQ-less DMA ring + one independently-configurable data pin (`i2s_rx_pin[0..3]`), all sharing the single BCK/LRCLK. In `i2s_input.c` each pair is an `I2sRxPair` descriptor and every lifecycle step loops over `i2s_n_pairs` (= channels/2), so the single-pair path is just `n_pairs == 1` with no special-casing. In the clock-master role pair 0's SM runs `clkmaster` (drives BCK/LRCLK + reads pair 0) and pairs 1.. run the `slave` program waiting on the *same* pads pair 0 drives; in the slave role every pair runs `slave` against the external clock. All pairs are enabled on one cycle via `pio_enable_sm_mask_in_sync()` and advance in lockstep on the one shared clock. In the **slave role** every pair runs the same wait preamble, so all latch the same first frame. In the **master role** there is a deterministic one-frame asymmetry: pair 0 (`clkmaster`) starts sampling immediately and captures from frame 0, while the slave-program pairs cannot detect the start of frame 0 (LRCLK is already low at enable) and lock on the next LRCLK fall, capturing from frame 1. `i2s_input_start()` corrects this by advancing pair 0's read pointer one stereo frame (2 words) when `clock_master && pairs > 1`, so every pair's read pointer lands on the same physical frame (cycle-exact per the clkmaster/slave PIO start timing, independent of ADC settling). Either way the 2/4/6/8 channels are sample-aligned (the inviolable inter-channel guarantee), exactly as the TX path's `audio_*_enable_sync()` aligns the outputs. The poll consumes the per-pair minimum available and reads the same frame index from every ring, deinterleaving pair `p` into input channels `2p`/`2p+1`; the pipeline's `n_active_inputs` for the I2S source is `i2s_input_channels`, so the 8×9 matrix applies automatically (identical to 8-ch USB input); the leveller runs over all active inputs, and loudness and crossfeed run per output (loudness post-gain, crossfeed per output pair post-matrix) rather than being bypassed. A live channel-count *raise* is applied to `i2s_input_channels` immediately but the extra pairs are not allocated until the deferred restart fires, so for the intervening poll cycles the poll zeroes the not-yet-filled `buf_in_ext` rows up to `active_input_channel_count()` — the matrix sees silence on the new channels, never stale samples (preserving the no-leak invariant). A count *drop* needs nothing (the matrix just stops reading the surplus rows). RP2040 is stereo-only (`I2S_RX_MAX_PAIRS` = 1). **Config:** vendor `REQ_SET/GET_I2S_INPUT_CHANNELS` (0xF3/0xF4) and `REQ_SET/GET_I2S_RX_PIN` (0xF1/0xF2, now `wValue = (pair<<8)|gpio`); a channel-count or higher-pair pin change restarts the input (`i2s_input_restart_pending`) so every pair re-syncs (pair-0 stereo keeps its lighter hot-swap). The four pair pins are kept mutually distinct AND clear of the I2S clocks: `check_i2s_rx_pin()` rejects a pin that is invalid, a clock pin (BCK/LRCLK), used by a fixed peripheral, or already on another pair. The clock check is **unconditional** — an I2S RX data pin coexists with BCK/LRCLK whenever the input runs, so it must avoid them even while I2S is inactive (closing the gap where `is_pin_in_use()` only reserves the clocks while they run; the inactive placeholders still never block their default GPIO 2/3/4 in stereo). The four data-pin defaults are the contiguous block GPIO 1/2/3/4 (pairs 0/1/2/3), all clear of the default peripheral assignments so enabling 4/6/8-channel input out of the box never self-collides. `REQ_SET_I2S_RX_PIN` and the count-*increase* path use it per-pin. The bulk and preset/flash **restore paths** validate the proposed pin set as a unit (`i2s_rx_pin_set_acceptable()`, against the effective BCK that transfer installs) and reject an inconsistent pushed/stored I2S config rather than apply it — so two state machines can never come up on one GPIO or on a clock pin regardless of how the config arrived. The restore paths also validate the incoming **BCK** itself before installing it (`i2s_bck_pin_acceptable()`: BCK + LRCLK valid GPIOs, no fixed-peripheral collision), keeping the live pin on failure — BCK/LRCLK are push-pull clock outputs, so an invalid GPIO could fault `pio_gpio_init()` and a collision with an output pin would be driver contention. (The vendor `REQ_SET_I2S_BCK_PIN` path keeps its additional "reject while an I2S output is active" guard; a restore omits it since it installs the output config and clock pair together.) **Persistence:** `WireInputConfig` carries the count + 3 ext pins in reserved bytes (`0 = absent`, no `WIRE_FORMAT_VERSION` break); per-preset `PresetSlot` V22 tail-append; device-global `FlashOutputConfig` via a directory V4→V5 migration.

**Files**: `i2s_input.h` (API), `i2s_input.c` (lifecycle, DMA ring, poll), `i2s_input.pio` (all three RX PIO programs).

**Clock-master election.** The input SM has two roles, decided by `i2s_input_should_be_master()` in `main.c`:

- **Slave** (at least one output slot is I2S): the lowest-index I2S output slot remains the clock master as before; the input SM runs a wait-driven program (`audio_i2s_rx_slave`, 7 instructions, divider 1.0) that samples the data pin against the BCK/LRCLK pads. Both slave variants are authored with placeholder GPIO indices in their `wait gpio` instructions (0 = BCK, 1 = LRCLK); `load_slave_program()` copies the selected variant to RAM and rewrites the 5-bit GPIO index of every WAIT-source-GPIO opcode because `i2s_bck_pin` is user-configurable. The external-clock role loads a different, CHECKED variant (`audio_i2s_rx_slave_checked`, 24 instructions) that re-verifies LRCLK framing every frame; see "I2S Clock-Slave Input Mode" below.
- **Clock master** (no output slot is I2S): the input SM runs `audio_i2s_rx_clkmaster` (12 instructions), driving BCK/LRCLK via side-set with the exact TX-master divider (`sys_clk * 2 / Fs`, 24.8 ceiling) while shifting data in. Cross-PIO-block sync is never needed: input-as-master only happens when zero I2S outputs exist, and I2S TX slaves are divider-locked within their own block.

All programs: in_base = data pin, IN shift left, autopush 32, RX FIFO joined, 24-bit audio MSB-aligned in 32-bit frames, standard I2S 1-bit delay (LRCLK transitions during the last bit cell, matching `audio_i2s_clkout.pio`). The first word pushed after any (re)start is always a LEFT word, and pushes stay in strict L,R alternation through every path (including the checked variant's resync), so a ring word's index parity fixes its channel permanently.

**Resources.** Reuses the SPDIF RX footprint (free whenever SPDIF input is inactive) and, for multichannel, the DMA channels the SPDIF/I2S TX DMA-sharing work freed: PIO1 SM2 (RP2040, 1 pair) / PIO2 SM0..3 (RP2350); per pair `p`, DMA `I2S_RX_DMA_BASE + 2p` (data) and `+2p+1` (reload) — pair 0 = `PICO_SPDIF_RX_DMA_CH0/CH1` (4/5 RP2040, 5/6 RP2350), pairs 1..3 = 7/8, 9/10, 11/12 on RP2350 (a `_Static_assert` guards the channel budget). All pairs' SMs and DMA channels are claimed in `i2s_input_start()` and unclaimed in `i2s_input_stop()` so the SDK claim table stays consistent across input switches.

**IRQ-less DMA ring.** Channel A moves PIO RX FIFO words into a power-of-2-aligned word ring (write-address wrap, transfer count = ring words) and chains to channel B, which rewrites A's `al2_write_addr_trig` with the ring base, retriggering it forever. Zero IRQs, so capture survives IRQ-disabled windows. **Teardown must be race-free**: the two channels re-trigger each other (A chains to B; B writes A's trigger), so aborting them with two sequential `dma_channel_abort()` calls lets one re-arm the other in the gap, intermittently hanging the abort's busy-wait (watchdog reset) or leaving a channel live after unclaim. `stop_all_dma_rings()` instead disarms every data channel's chain via the non-triggering CTRL alias, then aborts all channels in a single `dma_hw->abort` write (bounded busy-wait guard). Every input stop (input-source switch, output type switch, flash bracket, pin hot-swap) goes through this path. `i2s_input_poll()` derives the fill level from the DMA write address, consumes whole stereo pairs only (capped at 192 frames per poll), masks the low byte, applies preamp (same Q28/float conversion conventions as SPDIF RX), and feeds `process_input_block()`. **It batches to a 48-frame minimum** (`I2S_INPUT_MIN_BLOCK`): the ring's write address advances per word, so without batching the fast main loop would feed `process_input_block()` a handful of frames at a time, and the budget-based CPU meter (`busy_us / (frames / Fs)`) would amortize the fixed per-block cost (Core 1 EQ-worker handshake, pipeline setup) over too few samples and read ~4x inflated (measured cpu0 66% vs USB 16% before batching). 48 frames matches the USB packet / consumer-buffer granularity, so I2S CPU tracks USB; the input is continuous so the threshold never starves, and it adds ~1 ms of latency at 48 kHz. (SPDIF RX does not need this: its library FIFO already advances in DMA-block chunks.) Because the ring length is even and the read pointer only moves in pairs, a word's ring position fixes its channel permanently; even a writer-laps-reader overrun garbles audio momentarily but can never swap L/R.

**Slave resync invariant.** `complete_pipeline_reset()` and `enable_outputs_in_sync()` rewind the I2S TX clock master to its PIO entry point, resetting LRCLK phase. A running slave-role input SM would misframe permanently, so both functions end with `i2s_input_resync()`: a no-op unless the input is RUNNING in the slave role, otherwise disable SM, drain the RX FIFO into the ring, re-anchor the read pointer at the DMA write address, restart the SM at its sync preamble. This makes the invariant structural; no call site needs to remember it.

**Startup prefill (50%).** I2S input uses the same prefill handshake as SPDIF so outputs start against a half-full consumer pool instead of whatever low fill (~18%) the startup transient leaves; the extra margin absorbs main-loop scheduling jitter (a delayed `i2s_input_poll()` under load would otherwise risk an underrun). The handshake lives in the main-loop I2S block and is gated on `preset_loading` (set by every disruptive op via `prepare_pipeline_reset`, which also clears the `i2s_prefilling` flag so the handshake restarts cleanly): once the DAC-mute hold has elapsed it calls `drain_and_disable_outputs()`, clears `preset_loading`, sets `i2s_prefilling`; then on each later iteration, when `get_slot_consumer_fill(0)` reaches 50% it calls `enable_outputs_in_sync()` and releases the DAC mute. Unlike SPDIF there is no lock to wait for, so the bring-up helper `i2s_input_bringup_prefill()` (used by the input-source switch and boot) applies the rate and MCK, starts the input, and deliberately does NOT enable outputs; it leaves that to the main-loop block. When the selected rate differs from the live rate, `perform_rate_change()` still runs its full reset (so SPDIF TX dividers reprogram via instance teardown/restart), briefly enabling outputs that emit muted silence before the block re-drains and prefills.

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
| 0xF1 | REQ_SET_I2S_RX_PIN | IN* | Set I2S RX data pin (wValue=(pair<<8)\|pin, returns status byte; pin 0xFF = reset that pair to its default) |
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
*Last updated: 2026-08-02 (fourth SPDIF input persisted: wire V28, slot V35, directory V16)*

- `SLOT_DATA_VERSION` 13 adds `input_source` (uint8_t) to `PresetSlot`
- Slots with version < 13 leave input source at its current value (USB by default)
- Factory reset sets `active_input_source = INPUT_SOURCE_USB`
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

**Clock domains.** I2S output slots are edge-slaved with a dedicated wait-driven PIO program (`audio_i2s_dataout_extclk.pio`, 26 instructions, divider 1.0, `external_clock` instance flag in `pico_audio_i2s_multi`): an LRCLK preamble with a BCK skew guard, then one bit out per external BCK falling edge with per-frame LRCLK framing verification (see "Framing-slip watchdog" below). Like the RX slave programs it is authored with placeholder GPIO indices (wait gpio 0 = BCK, 1 = LRCLK) patched at load (`i2s_extclk_load_program()`, reloaded only on a BCK pin change). Because the checked program is too large to share PIO0's 32-slot instruction memory with the master-clocking programs (clkout 8 + dataout 8 + SPDIF 4), the two clocking modes evict each other's programs at load time (`i2s_extclk_evict_program()` and its mirror in the clkout/dataout load paths); safe because clocking mode is global and a mode rebuild tears down every I2S SM first.

**Framing-slip watchdog.** Real-world external masters (USB-I2S bridges such as the Amanero Combo384) glitch or re-frame BCK/LRCLK around stream stop/start and 44.1/48-family rate switches. With free-running slave programs, one runt BCK pulse or LRCLK phase jump would shift the 32-bit word window one bit for the rest of the session; the sign bit lands mid-word (full-scale wrapped garbage) and the word RATE is unchanged, so the rate watchdog can never detect it. Both external-clock programs (`audio_i2s_rx_slave_checked` for RX, `audio_i2s_dataout_extclk` for TX) therefore move/sample the 64 bits per frame by count (deterministic 24-in-32 alignment, autopush/autopull 32) and then verify the LRCLK level at the TWO ADJACENT cells straddling the frame boundary (bit 63 = cell R31 must read HIGH, bit 64 = cell R32 must read LOW, both sampled half a BCK after a rising edge so the pad synchronizers are settled). The adjacent-pair check catches every sub-frame offset (a fixed slip of k cells passes both checks only for k = 0 mod 64); a slip of exactly a whole frame is inherently undetectable from LRCLK (documented spec limit). On a mismatch the program raises **PIO irq flag 7** on its block and re-frames itself at the next LRCLK falling edge, bounding the garbage to ~2 frames; the RX resync path pads the aborted right word (`in null, 1` -> autopush) and the TX path flushes the unsent right-word remainder (`out null, 32`), so L/R word alternation - and therefore channel identity - survives every slip. `i2s_slave_poll()` reads-and-clears both flags (`i2s_slave_slip_check()`: RX block directly, TX blocks via `audio_i2s_extclk_framing_slipped()`) and treats a slip exactly like a clock loss: increment `slip_count` (surfaced in `I2sSlaveStatusPacket`, claims a reserved byte), `i2s_slave_drop_lock()`, and let the main loop's RELOCKING path restart the receiver and re-frame every output through the prefill's gated synchronized start - necessary because a slipped pair/slot is no longer sample-aligned with its peers (inter-slot alignment invariant) and only a full synchronized restart re-establishes that. Stale flags are consumed at every receiver start and by `audio_i2s_enable_sync_prepare()` for extclk instances (the restart IS the handling). The on-chip slave role keeps the plain free-running RX program: our own TX master never glitches, and the checked variant would not fit alongside the clkmaster program. SPDIF and ADAT stay on sys_clk dividers, rate-matched by `i2s_slave_update_clock_servo()` in `i2s_input.c`: a rate loop from the measured external rate plus the SPDIF-servo-style consumer-fill trim, but referenced to the FIRST SPDIF-type slot (an edge-locked I2S slot consumes at exactly the external rate, so its fill can never expose SPDIF divider error). ADAT receives the identical divider (`adat_output_servo_divider()`; `adat_output_resync()` falls back to `i2s_slave_current_tx_divider()`). PDM stays unservoed (parity with SPDIF input mode) and MCK output is forced off (a local MCK would be asynchronous to the external clocks); it resumes per its stored config on leaving slave mode.

**Rate detection and lock.** All measurement derives from pair 0's DMA write pointer (2 words per external frame). A ~32 ms fast window snaps to 44100/48000/96000 (2% tolerance); two agreeing windows lock (`I2sSlaveState`: INACTIVE / ACQUIRING / RELOCKING / LOCKED, mirrored to hosts via `REQ_GET_I2S_SLAVE_STATUS` 0x8A and NOTIFY 0x09). A dual-anchor 8-16 s long window refines the servo reference to ~0.1 ppm so ADAT holds without a SPDIF fill reference. 5 ms without words = clock loss (mute and wait; no internal-clock fallback by design); a rate change is a lock drop + re-acquire, feeding the standard deferred `pending_rate` mechanism via `i2s_slave_check_rate_change()`. A poll-gap watchdog (4 ms) re-anchors measurement across long main-loop stalls instead of measuring through a possible ring wrap, and flash brackets suspend/re-arm the input as before.

**Main-loop flow (slave branch of the I2S block).** Mirrors the SPDIF lock-gated flow: `i2s_input_poll()` itself is gated on LOCKED (pre-lock audio may be at the wrong rate and would drain the soft-mute counter early; the lock transition re-anchors every pair's read pointer to a fresh shared frame boundary so the lapped acquisition backlog is never played), prefill is armed only when LOCKED (real input audio fills the pools; the external clocks never stop during the drain, so no silence synthesis is needed), `enable_outputs_in_sync()` + DAC-mute release at 50% fill, and a RELOCKING event mutes (unless already muted) and ALWAYS stops/starts the receiver, including while preset_loading is set, so RX state machines that stalled mid-word re-frame when clocks return (the next gated enable re-frames the extclk TX SMs the same way). The prefill enable is additionally gated on LOCKED so a mid-prefill clock loss cannot re-enable against a misframed receiver.

**Alignment.** In slave mode the synchronized output start is LRCLK-gated with prepare/start split enable (see "Synchronized Start" under SPDIF Output System); the extclk program discards exactly one frame (`out null, 32` twice) so the edge-slaved slots land on the same sample index as the SPDIF slots started at the gate edge, making the inter-type offset reset-invariant.

**Role election and transitions.** `i2s_input_should_be_master()` returns false in slave mode and `i2s_input_start()` derives the external role itself (`i2s_role_extclk`); `i2s_input_resync()` is a no-op for the external role (output restarts never glitch the external LRCLK). `process_type_switches()` waits out a freshly armed DAC hardware-mute hold after its own mute assert (instant for pre-gated callers; protects rebuilds that run after a completed reset already released the mute) and treats a same-type I2S slot whose live `clock_master`/`external_clock`/`clock_pin_base` state mismatches the target as a change, which is how clock-mode and input-source transitions rebuild slot clocking without a type diff (`rebuild_i2s_output_clocking()`); its MCK block enforces the slave-mode force-off. Mode changes are deferred (`pending_i2s_clock_mode` + `i2s_clock_mode_change_pending`, handler placed before the input-source switch handler): dormant applies just record the global; a live apply mutes, restarts the input, rebuilds I2S output clocking, restores the selected rate + MCK policy when returning to master mode, and emits the apply-time `PARAM_CHANGED`. The input-source switch handler rebuilds I2S slot clocking whenever a switch crosses the slave-clocked I2S boundary, and the preset-load / preset-delete / factory-reset / bulk-apply flows call the same rebuild after applying IO config, covering restores that install a new `i2s_bck_pin`, slave pin, or clock-pin mode with an unchanged type map (the mismatch detection compares the instance's `clock_pin_base` against `i2s_effective_bck_pin()`, so a UNIFIED↔SPLIT flip under slave clocking rebuilds the slots onto the newly effective pair). In slave mode the stored master-mode `i2s_input_rate` is fully dormant: the vendor SET, the source-switch target rate, and the preset/bulk restart paths all skip arming it, leaving the detected external rate as the only authority. The restart-path gates check the EFFECTIVE mode (`pending_i2s_clock_mode` when a flip is deferred, else the live global), since a restore can defer a master-to-slave flip in the same apply; the mode handler additionally drops any pending rate change that raced the flip into slave.

**Hardware notes.** The external master must run BCK = 64 x Fs with standard LRCLK polarity (no runtime format detection). Whenever the device is NOT in slave-clocked I2S operation it drives BCK/LRCLK itself if any I2S output or I2S input is active, so a hard-wired external master and the device can transiently both drive the pins across boots and mode/source transitions (documented user constraint; the firmware minimizes the windows). SPLIT clock-pin mode addresses this directly: because master (and every non-slave) role drives `i2s_bck_pin` while the external master feeds the separate `i2s_bck_pin_slave` pair, the two never contend on the same GPIOs; the dual-driver caveat applies only in UNIFIED mode where both share one pair.

### ADAT Input (RP2350 only)
*Last updated: 2026-07-15*

`INPUT_SOURCE_ADAT` (3) is an 8-channel, 24-bit ADAT lightpipe input from one TOSLINK receiver, 44.1/48 kHz only (no SMUX/96k). Disabled by default; selectable only when `adat_input_enabled != 0` AND `adat_input_pin != 0xFF`, and only on RP2350 (RP2040 keeps the config state for wire/preset round-trips but has no PIO/DMA budget for the receiver). Files: `adat_input.c/h`, `adat_input.pio`.

**Receiver architecture.** PIO1 SM2 runs the `adat_rx` NRZI decoder (`adat_input.pio`, 15 instructions) at clock divider 1.0 (full sys_clk, zero divider jitter). Each wire bit cell is counted by a 2-cycle poll loop whose length is set per sample rate via the Y register (cell = 2Y+5 sys cycles: 27 at 44.1 kHz, 25 at 48 kHz at the 307.2 MHz sys clock); no transition within a cell decodes a 0, a transition decodes a 1 and re-anchors the cell grid within 2 sys cycles in both directions, so clock offset of either sign cannot accumulate (an earlier 8-PIO-cycles-per-cell fractional-divider design had a one-way ratchet that deleted bits at 44.1 kHz; see tools/adat_rx_test/adat_rx_bitdiff.c, which models both designs cycle-accurately and proves the current one bit-exact across at least +-1000 ppm source offsets at both rates). The SM emits the decoded bitstream MSB-first, autopushed every 32 bits. DMA channel 15 (the one permanently free channel) streams the words into an 8 KB / 2048-word ring in ENDLESS transfer-count mode with a hardware write-address wrap: a free-running ring with no IRQ and no reload channel. Frame handling is entirely CPU-side in the main-loop poll. ADAT input requires the 307.2 MHz sys clock; at the 150 MHz fallback clock the integer cell quantization is too coarse and the input does not lock.

**Frame sync and loss detection.** The sync header's 10-zero run cannot occur in channel data (ADAT forces a 1 every 5th bit, bounding data runs to 4), so `adat_rx_scan()` searches fresh decoded bits for the 12-bit structural pattern `[1][10x0][1]` (`0x801`) to find the frame boundary. Once found, frames sit at a fixed bit offset (edge resync in the PIO absorbs clock offset, so exactly 256 bits arrive per frame) and `adat_rx_decode_frame()` unstuffs the 8 channel fields (exact inverse of `adat_encode_frame` in adat_output.c) into int32 full-scale. Each frame's header is verified before its samples are trusted; header verification doubles as the loss detector, since a dark or unplugged line decodes as zeros and never matches the header. An isolated bad frame is skipped while the lock holds; `ADAT_HDR_FAIL_LIMIT` (2) consecutive bad headers drop the lock (`slip_count`++).

**Lock machine** (`AdatInputState`): INACTIVE (hardware stopped) -> ACQUIRING (slave: probing exact 48/44.1 kHz decoder timing; master: waiting for a valid device rate) -> SYNCING (candidate header-proven; completing the lock transition) -> LOCKED (decoding audio) -> RELOCKING (signal or rate lost; outputs muted exactly like a SPDIF lock loss). Slave acquisition starts with the 48 kHz cell timing, requires `ADAT_SYNC_VERIFY_FRAMES` = 8 consecutive valid headers at the fixed eight-word frame stride, and alternates to the 44.1 kHz timing after `ADAT_RX_PROBE_DWELL_US` = 10 ms without a valid run (continuing to alternate while absent/unsupported). Re-lock tries the last valid family first. This makes decoded frame structure the family-rate authority, not the DMA rate of a deliberately corrupt wrong-cell stream. The main loop reacts to RELOCKING with the same mute/drain/prefill flow as SPDIF, so the output slots remain aligned through every probe/re-lock transition.

**Clock modes** (`adat_clock_mode`, default MASTER):
- **MASTER:** the far end locks to DSPi's ADAT output, so the return stream is already in our clock domain; no rate detection, no servo. The device is the rate authority via `REQ_SET_INPUT_RATE` (shared with I2S master mode). Above 48 kHz the input parks with `rate_ok = false` (outputs stay muted through a never-completing prefill), mirroring the ADAT output's suspension above 48k. Parking is master-only: `rate_ok == false` always means "master mode above the device-rate ceiling".
- **SLAVE:** external gear owns the clock. Exact-timing header probes select the 44.1/48 kHz family as described above. Only after a candidate is proven does the receiver arm DMA-word measurement: 32 ms fast windows validate the family and supply the initial fine-rate estimate, while a dual-anchor 8-16 s long window supplies a ~0.1 ppm servo reference. All outputs (SPDIF/I2S/ADAT TX dividers + MCK) are servoed to it via `input_servo_apply()`, exactly like SPDIF input. `adat_input_check_rate_change()` feeds the standard deferred `pending_rate` mechanism when the detected rate differs from `audio_state.freq`. Slave mode is never parked (`rate_ok` always true): if the device rate is above 48 kHz at switch-in, acquisition still reaches LOCKED and the deferred rate change retunes the pipeline under the switch-in mute before the synchronized prefill enables the output slots; the servo holds off until `adat_rx_detected_rate == audio_state.freq`, so no output clock is slewed during that muted window.

**Prefill flow.** On lock the poll batches whole frames (`ADAT_INPUT_MIN_BLOCK` = 48, ~1 ms at 48k, capped at 192/poll), decodes each into the pipeline input buffers (`buf_l`/`buf_r` + `buf_in_ext[0..5]`) with per-channel preamp, and calls `process_input_block()`. A lap guard skips whole frames (preserving frame phase, since the ring holds an exact number of frames) before the reader can be overwritten.

**Loopback self-test.** The RX pin may deliberately equal the ADAT output pin: `adat_input_start()` only sets the pad's input enable and never touches funcsel, so the PIO reads the pad the ADAT TX drives; this is a zero-hardware loopback self-test. `adat_input_stop()` clears input-enable only, safe on the shared pin.

**RP2350 pad isolation.** RP2350 pads reset isolated (`ISO=1` in `PADS_BANK0`), and `gpio_set_input_enabled()` never clears the isolation latch; only `gpio_set_function()` does. A dedicated external RX pad would therefore stay electrically isolated and the PIO would see a static level (same-pin loopback worked only because the ADAT TX `pio_gpio_init()` unisolated the shared pad). `adat_input_start()` clears `ISO` directly with `hw_clear_bits()` under `#if HAS_PADS_BANK0_ISOLATION`, keeping funcsel untouched so the loopback self-test still works even if RX and TX ever sit on different PIO blocks. *Last updated: 2026-07-15*

**Main-loop integration** (`main.c`): the poll + prefill block, the source-switch branches, the deferred clock-mode handler (`adat_clock_mode_change_pending`), boot-into-ADAT, the `adat_input_restart_pending` handler (enable/pin change while ADAT is the live source), and the `perform_rate_change` hook (`adat_input_on_rate_change`) mirror the SPDIF/I2S input plumbing. Status is surfaced by `REQ_GET_ADAT_INPUT_STATUS` (0x6E, 20-byte `AdatInputStatusPacket`) and `NOTIFY_EVT_ADAT_INPUT_STATE` (0x0B) on every state change.

**Flash-write bracket.** `prepare_flash_write_operation()` treats a LOCKED ADAT input as a live source: it appears in the pre-mute drain, the streaming test, and the fade-settle loop (which calls `adat_input_poll()` until both the settle interval and the DAC hardware-mute hold have elapsed), matching the guarantees given to USB/SPDIF/I2S (see "DAC Hardware Mute"). Unlike SPDIF and I2S, ADAT is deliberately NOT stopped for the ~45 ms blackout: its RX is an IRQ-less free-running DMA ring with no decode-timeout alarms to race the blackout edge, and the poll re-acquires frame sync after the ring laps; `preset_loading` (still set from `prepare_pipeline_reset()`) then re-runs the drain/prefill/enable handshake once LOCKED returns.

**Resources.** PIO1 SM2 (PIO1 SM0 = PDM, SM1 = ADAT TX); DMA channel 15 (RX ring, ENDLESS mode, no IRQ). The 8 KB `adat_rx_ring` lives in the shared `input_capture_arena` (see "Input Capture Arena"); the decode hot path (`adat_input_poll`, `adat_rx_header_ok`, `adat_rx_decode_frame`, `adat_rx_rate_machine`, `adat_input_update_clock_servo`) is `DSP_TIME_CRITICAL` and RAM-resident (the acquisition probe/header scan stays cold in flash, since it runs only while muted). See "Memory Layout".

**Persistence.** ADAT input config persists like the optional SPDIF inputs (disabled by default, pin `0xFF` = unset, GPIO claimed only while ADAT is the active source). `WireInputConfig` (V24) gains `adat_input_pin`, `adat_input_enabled_p1` (enable + 1; 0 absent), and `adat_clock_mode_p1` (mode + 1; 0 absent, 1 master, 2 slave). The fields also ride `PresetSlot` (`SLOT_DATA_VERSION` 32) and the device-global `FlashOutputConfig` (`DIR_VERSION` 15), honoring `output_config_mode` exactly like the other physical-IO config. Vendor commands: `REQ_SET/GET_ADAT_INPUT_ENABLE` (0x68/0x69), `REQ_SET/GET_ADAT_INPUT_PIN` (0x6A/0x6B), `REQ_SET/GET_ADAT_INPUT_CLOCK_MODE` (0x6C/0x6D), `REQ_GET_ADAT_INPUT_STATUS` (0x6E); 0x6F reserved.

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

**SPDIF lock-acquisition path also releases the mute.** The USB→SPDIF switch (and boot-into-SPDIF, and SPDIF rx-pin hot-swap, and SPDIF re-lock after lock loss) does NOT call `complete_pipeline_reset()` — output must stay muted until SPDIF achieves lock and the consumer pool prefills. The lock-acquisition flow in the main loop replicates the relevant phases (`drain_and_disable_outputs()` → wait for lock + prefill → `enable_outputs_in_sync()`), then calls `dac_hw_mute_release()` directly to mirror Phase 4. If `release_ms > 0`, the pin remains asserted while `spdif_input_poll()` continues feeding the output buffers; without this release path, the XSMT pin asserted by the earlier `prepare_pipeline_reset()` would stay asserted indefinitely and the DAC's analog stage would never un-mute.

**`preset_loading` is floored to outlast a pending hold.** The deferrals above all hang off `preset_loading`, but that flag is not a plain latch: the soft-mute envelope (`update_preset_mute_envelope()`, `audio_pipeline.c`) auto-clears it once `preset_mute_counter` samples have been processed, and samples keep processing while a deferred consumer of the flag waits out the hardware-mute hold (`hold_ms` may be up to `DAC_HW_MUTE_HOLD_MS_MAX` = 500 ms). Two arming points enforce the floor (2026-07-14), both gated on `!dac_hw_mute_hold_elapsed()` and sized at `hold_ms` (via the `dac_hw_mute_hold_ms()` getter) + `PRESET_MUTE_HOLD_MARGIN_MS` (120 ms, `flash_storage.h`):
- `prepare_pipeline_reset()` (`main.c`) — closes the re-lock window: the SPDIF/ADAT/I2S-slave RELOCKING handlers arm only `PRESET_MUTE_SAMPLES` = 256 (~5 ms), and after a fast re-lock (or a pin-swap input restart) the source's poll keeps feeding the pipeline while the prefill drain waits on `dac_hw_mute_hold_elapsed()`; with a long hold the flag expired before the drain could run, the prefill handshake never fired, and `dac_hw_mute_release()` was never called (stuck mute). The floor engages only on a fresh hold; the synchronous reset handlers pre-gate on `pipeline_reset_ready()`, so their hold has already elapsed when their body runs prepare, and USB-path mute durations are unchanged.
- `flash_mute_hold_samples()` (`flash_storage.c`) — closes the non-streaming flash window: the flash brackets' fade-settle loop waits the hold out before the blackout, but only when the source is streaming; a flash write with the source dark leaves the hold pending at completion, and `flash_write_sector()`'s trailing re-arm (the last writer of `preset_mute_counter` before completion, ~10 ms un-floored) would let a subsequent fast lock burn the flag before the prefill block could run. The `preset_delete()` active-slot re-arm routes through the same helper. Note the streaming flash path never depended on the prepare-time floor: the settle loop's hold-wait plus the per-write re-arm already guarantee `preset_loading` is set with the hold elapsed at completion.

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

**Pump.** When the generator is running and no source is streaming, the main loop
calls `siggen_pump()` (`audio_pipeline.c`), feeding zero-input blocks through the
full pipeline paced by the slot-0 consumer fill level (top-up to half full). It
stands down while a source streams, an input-source change or output-type switch is
pending, or the producer pool is unallocated, so a real stream seamlessly takes
over pacing.

**Lifecycle.** Transient only: off at boot, never persisted; `siggen_stop_immediate
(SIGGEN_STOP_PRESET)` hard-stops it on preset load and factory reset. `main.c`
calls `siggen_service()` + `siggen_pump()` each main-loop iteration;
`siggen_service()` applies staged configs after a fade-out (SET/START while
running restarts) and pushes the deferred `NOTIFY_EVT_SIGGEN_STATE` (0x07)
start/stop/completion notification. Vendor commands `0xA4..0xA8` cover
SET/GET config, CONTROL (start/stop/stop-now), GET status, and self-describing GET
caps; `0xA9..0xAF` are reserved.

**Memory / CPU.** Small BSS (under 1 KB) plus RAM-resident render kernels
(`DSP_TIME_CRITICAL`); control/config code stays cold in flash. The only cross-core
datum is `siggen_raw_mask`, written by Core 0 between blocks.
