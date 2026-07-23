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
*Last updated: 2026-07-22*

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
| `soft_vcxo.c` | Soft VCXO: DMA-pulsed pll_sys fbdiv dither giving ppm-level sys_clk trim (the single actuator for input clock tracking) |
| `soft_vcxo.h` | Soft VCXO API (`soft_vcxo_init` / `soft_vcxo_set_ppm` / `soft_vcxo_current_ppm`) |
| `input_servo.c` | Shared clock servo for externally clocked inputs: ppm feed-forward + deadbanded P + centering integrator, actuates the soft VCXO |
| `input_servo.h` | Input servo API (`input_servo_apply` / `input_servo_reset`) |
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
*Last updated: 2026-07-05*

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
*Last updated: 2026-07-23 (consumer pools replaced by per-slot rings)*

Any host-driven format change (SET_INTERFACE between AS alts for a bit-depth switch, or SET_CUR on the endpoint sampling_freq control for a rate switch) must land on a muted, drained pipeline. Otherwise old-rate/old-bit-depth audio still queued in the slot rings plays out against the new PIO divider or gets decoded under the wrong bytes-per-frame assumption, producing an audible pitch shift or byte-scramble burst.

**`uac1_apply_alt()` (usb_audio.c):**
- **Idempotent early-return.** `SET_INTERFACE(alt=current)` is common from host driver probes and used to tear down / re-open iso endpoints for no reason. Now skipped.
- **Bit-depth switch (alt 1↔2) is treated the same as a cold start (alt 0→>0).** Both paths engage the mute envelope inline (`preset_mute_counter = PRESET_MUTE_SAMPLES`, `preset_loading = true`) so any packet decoded between the SETUP ack and the main-loop's `complete_pipeline_reset()` is silenced. Both paths also raise `stream_restart_resync_pending`, reset the feedback controller (`fb_ctrl_stream_stop` + `feedback_10_14 = nominal_feedback_10_14`), and clear `sync_started` / `total_samples_produced` so gap detection and the feedback loop resume from a deterministic baseline.
- The pre-existing ring flush on `bit_depth_changed` still runs; combined with the mute, stale packets cannot reach the DSP pipeline in the wrong format.

**SET_CUR sampling_freq validation:** unsupported rates are now stalled at EP0 rather than silently committed. Previously any 24-bit value was accepted — `audio_state.freq` would store garbage that `perform_rate_change()` later coerced to 44100, so a subsequent GET_CUR returned a rate the device never actually applied. The accepted set is {44100, 48000, 96000} to match the Type-I format descriptors on both alts.

**USB rate ownership:** `usb_selected_rate` is the UAC1 playback endpoint's host-selected rate, while `audio_state.freq` is the rate actually applied to the live DSP pipeline. SET_CUR and the fixed-48-kHz multichannel alts always update the USB selection, but arm a live rate change only while USB is the active input; GET_CUR returns the retained USB selection. A later source switch into USB applies that selection, so decorative USB control traffic cannot interrupt S/PDIF or I2S playback and the host does not need to resend its rate when USB becomes active. Two races are closed inside the source-switch handler: `rate_change_pending` is cleared right after `active_input_source` flips (a SET_CUR armed for the old source in the same loop iteration would otherwise fire under the new source; mirrors the clock-mode flip handler's clear), and the switching-to-USB tail re-checks the retained selection against `audio_state.freq` and re-arms if a SET_CUR landed in USB IRQ context between the branch reading the selection and the source flip.

**`perform_rate_change()` (main.c):** bracketed with `prepare_pipeline_reset(PRESET_MUTE_SAMPLES)` / `complete_pipeline_reset()`. Without the bracket the new PIO divider would apply while old-rate audio still sits in each slot ring, playing out at the new bit-clock; audible pitch wobble for ~16 ms. `complete_pipeline_reset()` aborts DMA on every enabled slot, resets each slot ring, and restarts all outputs in sync at the new divider. The I2S `audio_i2s_update_all_frequencies()` call inside `perform_rate_change()` still runs for its own divider+clkdiv_restart pass; the subsequent `complete_pipeline_reset()` re-aborts/re-enables idempotently and costs only microseconds.

**Nominal SPDIF divider restore (`restore_nominal_spdif_dividers()`, main.c):** since the soft-VCXO overhaul the input clock servos no longer trim any output divider; they pull `sys_clk` in ppm instead (see "Soft VCXO" and "Clock Servo"), so every SPDIF TX SM always runs its nominal `ceil(sys/f)` divider and the old trim-outlives-its-servo hazard (a servoed divider left displaced after the input stopped, dragging ADAT out of alignment with the slots it mirrors) is gone by construction. `restore_nominal_spdif_dividers()` is retained as one of the nominal-divider owner paths: `perform_rate_change()` still calls it (via `audio_spdif_apply_pio_frequency()`, a public wrapper in the SPDIF library) to reprogram every SPDIF-type slot to the new rate's nominal divider, and the two equal-rate paths that skip `perform_rate_change()` (the source-switch-away-from-I2S branch and the slave-to-master clock-mode flip) call it too; it is idempotent when the dividers already sit at nominal. All SPDIF slots receive the same divider, so inter-slot alignment is unaffected. *Last updated: 2026-07-22*

### Clock Diagnostics (bench instrument)
*Last updated: 2026-07-23*

`clock_diag.c` is a serial-console instrument for isolating DAC-cutout failure classes while the soft VCXO tracks an input (sys_clk excursion, servo oscillation, transient consumer starvation, TX/main-loop stall, wedged VCXO DMA loop). It exposes the same once-per-second values two ways: the `CLKDIAG` console `printf` dump, and a read-only `REQ_GET_CLOCK_DIAG` (0x82) vendor GET that returns the latched `ClockDiagPacket` snapshot (used when no console backend is enabled; this firmware ships with USB CDC and UART stdio both off, so the printf output is otherwise discarded). Gated entirely by `DSPI_CLOCK_DIAG` (config.h, default 1 for bench; set 0 for release, which compiles everything to no-op inlines and makes `REQ_GET_CLOCK_DIAG` return a zero-length response). `clock_diag_sample()` runs once per main-loop pass (services a non-blocking fc0 frequency counter, tracks the max loop gap, per-slot min consumer fill, and PWM-pacer counter movement); `clock_diag_poll()` prints once per second and resets the windowed aggregates.

Two A/B switches (config.h, default 0, honored only under the flag): `SOFT_VCXO_DIAG_FORCE_PARK` makes `soft_vcxo_set_ppm()` clamp every command to 0 (the servo still computes and its intent is still logged, but the PLL never moves; persistent cutouts then exonerate the VCXO), and `SOFT_VCXO_DIAG_FREEZE_INTEGRATOR` makes `input_servo.c` skip integrator accumulation (isolates integrator windup/oscillation).

The dump is six-to-eight lines, each prefixed `CLKDIAG ` for grepping:

- `sysclk:` fc0-measured sys_clk `hz`/`min`/`max`, derived `ppm` offset vs 307.2 MHz `[min..max]`, sample count `n`, PLL `lock` bit. (fc0 is otherwise unused in the tree.)
- `vcxo:` last commanded `ppm[min..max]`, `park` state, commanded `fbdiv0` (127/128/129), effective `pulseHz`, PWM `div`/`wrap`, cumulative `set`/`clamp`/`sflip` (sign-flip) counts, live `fbdivLive` (pll_sys `fbdiv_int`), PLL `cs` lock.
- `vcxoloop:` PULSE and PACE DMA channel `busy`/`en`/`tcr` (transfer count remaining, from the debug registers)/PULSE `rd` addr/PACE `ctdreq` (DREQ counter), plus the PWM pacer `pwmctr` and its `moves` count since last dump (loop-alive evidence).
- `servo:` measured input `in` Hz, `ff` feed-forward ppm, total `tot[min..max]` ppm, integrator `int` ppm, `ferr` fill error, `slot` used, `ticks`.
- `slots:` per output slot: type (spd/i2s), current `fill`/`min` since dump, `starv` (per-instance underrun count, now in **frames** of silence exposed to the wire, from the ring's always-on underrun accounting), `dwc` (delta words consumed = TX DMA throughput, derived from the unwrapped ring read pointer; drops to 0 on a stalled channel).
- `pdm:` (when PDM enabled) ring/dma fill, `ringur` (ring-empty silence insertions) / `dmaur` (write-behind-read) underruns.
- `input:` SPDIF RX `st` state, `rate`, `lk`/`ls` lock/loss counts, `par` parity errors, `fifo` fill %, estimator `hzL`/`hzS` (long-window count vs IIR bridge) and which is in `use`, long-window `span`.
- `loop:` worst main-loop `gapmax` since last dump.

Starvation counters are the authoritative instantaneous-empty signal (H3). With the ring model they count **frames** of silence exposed to the wire (ring underrun accounting is always on, no monitor bool, no IRQ path). All `clock_diag.*`, plus the diagnostic getters in `soft_vcxo.c`/`input_servo.c`/`spdif_input.c`, live in flash and run only from the main loop.

The `REQ_GET_CLOCK_DIAG` (0x82) snapshot is a packed little-endian `ClockDiagPacket` (`clock_diag.h`, version byte = 1) carrying every dump field: ppm values are centi-ppm (ppm times 100), frequencies flagged `_mhz` are milli-Hz, per-slot arrays are sized `NUM_SPDIF_INSTANCES` with a `num_slots` count. `clock_diag_poll()` latches it each second next to the printf; `clock_diag_get_snapshot()` copies it for the handler. Tearing is acceptable (both the poll and the control transfer run on the core 0 main loop). Host decoder: `tools`-adjacent `clkdiag_poll.py` polls it at 1 Hz. *Last updated: 2026-07-22.*

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
*Last updated: 2026-07-23 (consumed words now from the unwrapped ring read pointer at SOF)*

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

**Ring consumed-words used by feedback (2026-07-23):** with the free-running rings there is no `words_consumed` DMA-IRQ counter. At SOF the feedback loop reads `audio_spdif_ring_consumed_words()` (mirror `audio_i2s_*`), which unwraps the DMA read pointer to a monotonic total DMA words consumed since the last ring reset; this is sample-exact and needs no in-flight-transfer size term. The unwrap must be sampled at least once per ring period (~21 ms), which the main loop's fill checks and the per-SOF read both guarantee.

**IRQ safety:** The SOF handler runs inside `isr_usbctrl`. Output DMA raises no IRQ, so it cannot preempt the SOF read; the read-pointer unwrap is done under a brief IRQ-off section inside the getter. The DMA/USB IRQ priority pinning is retained for the S/PDIF RX path (the only remaining DMA IRQ user).

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
5. **Ring write**: the blocking give hands each finished producer block to the writer, which encodes it directly into the slot ring for the free-running DMA

The `process_input_block()` function reads from `buf_l[]`/`buf_r[]` arrays (extern, defined in `audio_pipeline.c`, filled by the input decode stage). This separation enables future alternative input sources (S/PDIF, I2S) to fill the same buffers and call `process_input_block()` directly. Buffer statistics helpers (`get_slot_consumer_fill()`, `get_slot_consumer_stats()`, `reset_buffer_watermarks()`) also live in `audio_pipeline.c`.

### RP2350 Float Pipeline
*Last updated: 2026-07-19 (upmixer presence bell added to the centre engine)*

All processing in IEEE 754 single-precision float. Hybrid SVF/biquad EQ filtering (SVF for bands below Fs/7.5, TDF2 biquad above).

| Stage | Description |
|-------|-------------|
| Input conversion | USB int16/24-bit or SPDIF RX 24-bit → float full-scale, per-channel preamp gain (`global_preamp_linear[ch]`) |
| Per-Input EQ + metering | Per active input: block-based `dsp_process_channel_block()`, 10 bands, hybrid SVF/biquad; then peak/clip into `peaks[k]`/`clip_flags` |
| Volume Leveller | Upward RMS compressor over active inputs (2 to 8), mask-driven detector/apply link, gain-reduction limiter, 5 ms per-channel lookahead (float throughout) |
| Stereo Upmixer | Stereo input only: derives Centre/Ls/Rs into matrix source rows 2..4 and removes centre from L/R; raises the matrix source count to 3 or 5. Parks in multichannel modes. Zero-latency steering; deliberate identical-per-row surround Haas delay (see "Stereo Upmixer") |
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

**Performance advantage of block-based processing:** Biquad coefficients are loaded once per biquad per block instead of once per sample. For a 2-band output channel with 192-sample blocks, this reduces coefficient loads from 384 to 2.

---

## USB Audio Loopback Capture (DSPI_LOOPBACK)
*Last updated: 2026-07-05 (XIP: RP2040 no longer RAM-tight; loopback ELFs are accepted by check_ram_placement.py)*

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
*Last updated: 2026-07-12*

**Types:** Flat (bypass), Peaking, Low Shelf, High Shelf, Low Pass, High Pass, Notch, All-Pass (2nd-order RBJ), First-Order All-Pass (`FILTER_ALLPASS1`), First-Order Low Shelf (`FILTER_LOWSHELF1`), First-Order High Shelf (`FILTER_HIGHSHELF1`), Linkwitz Transform (`FILTER_LINKWITZ_TRANSFORM`)

**Coefficient computation:** RBJ Audio-EQ-Cookbook formulas for biquad path, Cytomic SVF equations for SVF path (RP2350 only), both in `dsp_compute_coefficients()`

**First-order types (`FILTER_ALLPASS1`, added 2026-06-17; `FILTER_LOWSHELF1` / `FILTER_HIGHSHELF1`, added 2026-06-20):** Genuine 1st-order sections. The all-pass has flat magnitude with phase 0° → -180° (-90° at the corner), single parameter `freq` (`Q`/`gain_db` unused). The shelves are gentle 6 dB/oct shelves (monotonic, no `Q`); the first-order shelf prewarps `g` by `A` (not `sqrt(A)` like the 2nd-order shelves). On RP2350 these follow the **same hybrid rule as every other type** (changed 2026-06-20): a one-pole TPT SVF below `Fs/7.5` (`bq->svf_first_order`), a degenerate TDF2 biquad (`b2 = a2 = 0`) above. The one-pole SVF folds the `1/(1+g)` reciprocal into a coefficient so its inner loop is multiply-only. On RP2040 (no SVF) they always run as a Q28 degenerate biquad through the existing assembly kernel. Both realizations produce the identical RBJ-cookbook response, verified across the `Fs/7.5` boundary by `tools/filter_tester`.

**Linkwitz Transform (`FILTER_LINKWITZ_TRANSFORM` = 11, added 2026-07-12):** A pole/zero bass-extension biquad that replaces a driver's measured 2nd-order rolloff (`f0`, `Q0`) with a target alignment (`fp`, `Qp`): `H(s) = (s^2 + s*w0/Q0 + w0^2) / (s^2 + s*wp/Qp + wp^2)`. Unlike every other PEQ type it needs **four** parameters, so it borrows the wire packet unconventionally: `EqParamPacket.freq` = `f0`, `.Q` = `Q0`, `.gain_db` carries `fp` in Hz (`gain_db` is otherwise unused, and the front-panel gain noun is a no-op on LT bands), and the fourth parameter `Qp` rides a parallel array `peq_qp_x512[NUM_CHANNELS][MAX_BANDS]` (`dsp_pipeline.c`) as a `uint16` LE `Q*512` (`0` = the 0.707 default). Clamps: `f0` and `fp` to `[10, 0.15*Fs]` (tighter than the generic 0.45*Fs PEQ clamp; bounds every normalized coefficient inside the RP2040 Q28 range), `Q0` and `Qp` to `[0.1, 20]`; `fp <= 0` (or `freq <= 0`) is treated as flat. On RP2350 it runs on the hybrid path (see below); on RP2040 it is the same exact-bilinear TDF2 biquad in Q28 with both corners independently prewarped. The sidecar `Qp` is carried through the wire (`WireBandParams.reserved[2]`), flash (`PresetSlot.peq_qp_x512` tail-append), the `REQ_SET_EQ_PARAM` optional 18-byte payload, and `REQ_GET_EQ_PARAM` param code 5; these bumped `WIRE_FORMAT_VERSION` to 22 and `SLOT_DATA_VERSION` to 30 (V21..V29 slots still load, `qp` defaulting to 0 = 0.707). Full host-facing spec: [`Documentation/Features/peq_filters.md`](Features/peq_filters.md).

**Filter type value space (`enum FilterType`):** PEQ types occupy 0–7 plus the first-order all-pass at 8, the first-order low/high shelves at 9–10 (added 2026-06-20), and the Linkwitz Transform at 11 (added 2026-07-12), with 12–31 reserved for future PEQ types; crossover types occupy 32–63 (see [Crossover Filters](#crossover-filters)). `filter_is_peq_type()` (config.h) is the single classifier (any value below `FILTER_XOVER_FIRST`); `is_filter_flat()` uses it to flatten a crossover type that lands in a PEQ band slot. The crossover types were renumbered from the old 8–39 range on 2026-06-17 to open the contiguous PEQ block, which bumped `SLOT_DATA_VERSION` to 18 (pre-V18 presets migrated on load via `remap_filter_type_pre_v18()`) and `WIRE_FORMAT_VERSION` to 13. Adding the first-order shelves (new enum values only; no renumber, no on-disk layout change, no migration) bumped `SLOT_DATA_VERSION` to 19 and `WIRE_FORMAT_VERSION` to 14. The Linkwitz Transform adds a per-band target-`Q` sidecar (`peq_qp_x512`), tail-appended to `PresetSlot` and carried in the `WireBandParams.reserved[2]` bytes, which bumped `SLOT_DATA_VERSION` to 30 and `WIRE_FORMAT_VERSION` to 22 (V21..V29 slots still load with `qp` defaulting to 0 = 0.707).

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
*Last updated: 2026-07-12*

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

**State reset:** When a band changes filter topology — crossing the SVF/biquad boundary (e.g. due to sample rate change) or switching between the one-pole and 2nd-order SVF (`bq->svf_first_order` flips) — both biquad state (`s1`, `s2`) and SVF state (`svic1eq`, `svic2eq`) are reset to zero to prevent transients.

**Input validation:** Frequency clamped to [10 Hz, 0.45×Fs], Q clamped to [0.1, 20].

**FPU configuration (RP2350):** Both cores set FPSCR flush-to-zero (FZ) and default-NaN (DN) bits at startup. This prevents denormalized floats from causing performance penalties as SVF integrator and biquad states decay toward zero after silence.

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
*Last updated: 2026-06-20*

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

Kernel reuses the existing per-section TDF2 (RP2040) and SVF/TDF2 (RP2350) inner loops. RP2040 calls a new assembly entry point `dsp_process_band_cascade_block` that shares the inner-loop body with `dsp_process_channel_block` via local labels — only the band-loop terminator differs (parameter-supplied `num_sections` vs `channel_band_counts[channel]` lookup).

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
*Last updated: 2026-07-23 (pre-write recovery fill snapshot added)*

```c
typedef struct audio_spdif_instance {
    PIO pio;
    uint8_t pio_sm, dma_channel, dma_reload_channel, dma_irq, pin, instance_index;
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
    // ... producer format + embedded connection (retained pending direct write)
} audio_spdif_instance_t;
```

### Buffer Configuration
*Last updated: 2026-07-23 (PDM/slot recovery made block-size exact)*

The consumer buffer pool is gone. Each slot now drains a single **encoded ring** filled by the writer and read by a free-running DMA with **zero output IRQs**:

- One static 16 KB ring per slot (`slot_ring_store[NUM_SPDIF_INSTANCES][PICO_AUDIO_SPDIF_RING_BYTES]` in usb_audio.c), caller-owned and passed via `config.ring_base`. The same store backs the slot's S/PDIF and I2S instances across output-type switches. `PICO_AUDIO_SPDIF_RING_FRAMES = 1024` frames (21.3 ms at 48 kHz), 2 channels × 8 bytes per frame; I2S uses the identical geometry (`_Static_assert`ed equal).
- The writer encodes directly into the ring at give time (`audio_spdif_ring_write_s32`), stamping IEC preamble and channel-status per frame from the absolute frame index; the producer pools are retained for now (the blocking-give connection still drives the writer) pending a direct-write follow-up.
- Erase-ahead silence margin: the writer keeps `PICO_AUDIO_SPDIF_ERASE_AHEAD` = 192 frames (one IEC block) of valid silence stamped ahead of real audio, so a stalled producer plays silence rather than a loop of stale ring content.
- Free-running DMA: **ENDLESS** transfer count with hardware read-address wrap on RP2350 (power-of-two ring, no reload channel); on RP2040 a data + reload channel pair (`dma_reload_channel = OUTPUT_SLOT_RELOAD_DMA_BASE + slot`, base 6) rewrites the read address at the ring end. No DMA completes, so there is no output DMA IRQ on either platform.
- Fill, consumed-words, and underrun accounting are pointer arithmetic (always on). `wr_frames − consumed` gives fill. The DMA address supplies the exact position modulo the 1,024-frame ring; for calls separated by more than one lap, elapsed microseconds and the configured sample rate select the nearest whole-lap count, so the ~45 ms flash blackout cannot disappear from recovery accounting. Time is only the lap selector; the hardware pointer remains the sample-granular remainder. A coherent produced/consumed **snapshot pair** (`snap_produced_frames` / `snap_consumed_frames`) is latched at the end of each block write, giving the clock servo a constant-production-phase observation free of the block-arrival sawtooth; the fractional fill getter returns that snapshot difference (cumulative-count differencing). A separate `recovery_prewrite_fill_frames` snapshot captures fill immediately **before** each successful write (`UINT32_MAX` until the first write after reset). The separation is load-bearing: the servo wants post-write fill, while underrun recovery must exclude the final real block that PDM consumes before inserting zeros. The USB feedback loop consumes the same unwrapped ring read pointer at SOF (`audio_spdif_ring_consumed_words()`), which is sample-exact (see "Asynchronous Feedback Endpoint"). Fill target and prefill thresholds are unchanged in semantics; `REQ_GET_BUFFER_STATS` and the prefill checks keep byte/semantics compatibility via a 48-frames-per-bucket mapping (a ring fill of N frames maps to N/48 legacy "buffers").
- **Underrun recovery** is a group re-anchor: `output_rings_pre_give()` (audio_pipeline.c) selects the worst enabled slot and applies one identical `*_ring_skip()` to every enabled slot, so inter-slot sample alignment is preserved exactly. Its skip is `deficit + recovery_prewrite_fill_frames`, with no bucket rounding. Algebraically this is the missing-source interval after the final successfully written block; it therefore equals the number of zeros PDM inserts after draining that same real block, preserving PDM-to-slot content alignment for arbitrary block sizes. A valid zero pre-write fill remains distinguishable from the reset sentinel. The starvation counters (`audio_spdif_get_dma_starvations_instance()` / I2S mirror) return frames of silence exposed to the wire, not silence-buffer DMA starts.
Since 2026-07-23 each per-slot record also carries `fill_centi_pct` (uint16, formerly the two pad bytes): sample-granular fill in 0.01% units of the same 16-bucket scale (5000 = 50.00%; values above 10000 are legal, the ring holds more than the legacy capacity). Older firmware reports 0 in this field; the coarse bucket fields are unchanged, so existing Console builds keep working.

### IEC 60958-1 Block Phase
*Last updated: 2026-07-23*

Each 192-frame audio block carries channel status bits and a Z preamble at frame 0. With the ring, the block phase is simply the **absolute frame index mod 192**: `audio_spdif_ring_write_s32()` stamps the correct Z/X preamble and channel-status C bit (IEC 60958-3, h[29], with parity P at h[31] XOR'd alongside) per frame from the running write count, and the erase-ahead silence carries the same phase. No per-buffer template fixup or LIFO free-list correction is needed. `_Static_assert`: `PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT % PICO_AUDIO_SPDIF_ERASE_AHEAD == 0` style invariants keep the block/margin aligned at compile time.

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
*Last updated: 2026-07-23*

Output S/PDIF and I2S generate **no DMA IRQs** at all: the rings are drained by free-running DMA (ENDLESS on RP2350, data+reload pair on RP2040). The `dma_irq` field and the shared-refcount enable/disable API are retained solely for the S/PDIF RX hold on `DMA_IRQ_1`, which is unchanged and is now the only user of that refcount.

### Synchronized Start

Every output-slot start in the firmware goes through one shared helper, `start_output_slots_in_sync()` in `main.c` (used by both `complete_pipeline_reset()` Phase 2 and `enable_outputs_in_sync()`; there is deliberately no other start path). It runs the prepare-only halves of both output libraries (`audio_spdif_enable_sync_prepare()` / `audio_i2s_enable_sync_prepare()`), then starts every slot of BOTH output types in a single `pio_enable_sm_mask_in_sync()` write (valid because both types share `PICO_AUDIO_SPDIF_PIO`). The single mask write starts all SM clocks in the same sys_clk cycle and resets their fractional clkdiv accumulators together, so mixed-type configs get the same cycle-exact start alignment single-type configs always had. Before 2026-07-21 the non-slave path issued two group starts (`audio_spdif_enable_sync()` then `audio_i2s_enable_sync()`) with the I2S DMA priming between them, leaving a fixed few-microsecond inter-type start skew and unsynchronized divider phase at fractional rates; unifying the paths also eliminated a persistent 1-2 buffer slot-0 consumer-fill deficit observed on hardware after every reset. Callers hold interrupts disabled across the helper so nothing can complete a primed DMA transfer for one type before the other type's clocks run.

In I2S clock-slave mode the same helper additionally gates on an external LRCLK falling edge between priming and the mask write (bounded ~35 us, only when an extclk I2S slot exists). Together with the external-clock I2S program's one-frame discard this makes the SPDIF-vs-I2S inter-slot offset reset-invariant. *Last updated: 2026-07-21*
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
*Last updated: 2026-07-19*

### Purpose

Derives Centre / Left-Surround / Right-Surround virtual source channels from a stereo program so a two-channel input can drive a multi-speaker layout. It runs **only on RP2350** (the feature is compiled out on RP2040), **only when the active input is the plain stereo pair** (`active_input_channel_count() == 2`), and **only at sample rates of 48 kHz or below** (the delay rings are sized for 48 kHz; above that the upmixer parks, ADAT-style, and `upmix_apply_config` publishes NULL). In multichannel input modes those matrix rows carry real inputs and the upmixer parks the same way. Module: `firmware/DSPi/upmix.c/h`. Status: **HW-untested**.

### Signal Flow

The pass sits between the leveller and the matrix (PASS 3 in the RP2350 pipeline). It reads the post-EQ/leveller stereo bus (`buf_l`/`buf_r`), writes the derived channels into the otherwise-idle multichannel input rows (`buf_in_ext[0..2]` = matrix source rows 2..4 = `UPMIX_ROW_C`/`_LS`/`_RS`), and applies centre removal to L/R in place. The matrix then treats the derived channels as ordinary sources: routing, crosspoint gains, and the whole per-output chain (PEQ, crossover, delay, gain, loudness) are reused unchanged. The matrix loop bound is generalized from `n_active_inputs` to `n_matrix_sources` (2 + 1 for centre-only, 2 + 3 with surround), so only the rows actually carrying derived audio are summed. The derived rows have **no input PEQ** (they are synthesized after the per-input stage) but DO get peak/clip metering into `global_status.peaks[2..4]` while the upmixer runs; the extracted centre can legitimately reach +3 dBFS on hot correlated content, so hosts must watch these meters. Steering telemetry (correlation, balance, live gains) is exposed through `REQ_UPMIX_GET_STATUS`.

Two independent engines feed the derived rows.

**Centre engine** (always produces row 2 when enabled):
- `PASSIVE`: `C = 0.7071 * (L + R)`, fixed constant-power sum.
- `ADAPTIVE`: a running normalized cross-correlation and L/R balance (one-pole estimators on a bass-cut, one-pole-HP detector path) drive the centre gain through a threshold gate with renormalization above the knee, then attack/release ballistics applied per block (packet-size independent) and per-sample gain ramps. Only genuinely centre-panned correlated content is extracted, so the image does not pump; long-wavelength content is excluded by the detector HP (industry-standard bass-steering mitigation).

Extracted centre energy is subtracted from L/R scaled by strength and by centre-width (`0.5 * (1 - width)` removal), so a physical centre speaker and the L/R phantom do not comb-filter. Constant-power conventions: `0.7071` centre extraction, `0.5` removal.

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
| center_mode | 0/1 | ADAPTIVE (1) | Centre engine: 0 PASSIVE, 1 ADAPTIVE |
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

- **Wire format V25 (presence byte V26):** `WireUpmixParams` (44 bytes, layout-identical to `UpmixConfigPacket`) is tail-appended to `WireBulkParams` (total 5944 bytes). V26 claims the section's reserved byte for `presence_q1` (int8, dB * 2, 0.5 dB steps; no size change). Bulk collect/apply copy it straight to/from `upmix_config` and raise the pending flag; the section is zeroed on collect and ignored on apply on RP2040.
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
*Last updated: 2026-07-05*

### Flash Operation Safety

Flash erase/program requires quiescing XIP (execute-in-place): only the erase/program windows forbid flash fetches; ordinary XIP execution is legal even with IRQs disabled. `flash_write_sector()` and `preset_delete()` run with IRQs disabled and park Core 1 via a guarded `multicore_lockout` when Core 1 is a registered victim:

- **Guard condition:** `multicore_lockout_victim_is_initialized(1) && (__get_current_exception() == 0)`. The SDK function handles first-boot (Core 1 not launched) and launch-to-init race windows. The exception check skips lockout from IRQ context (USB vendor handler), where SDK lock internals are unsafe. IRQ-context saves that skip the lockout are safe because everything Core 1 can execute is RAM-resident (enforced by `scripts/check_ram_placement.py` Check B / B2), so a parked-or-not Core 1 never fetches from flash during the erase/program window. The audio consequence of a flash write is unchanged (muted window, feedback re-seed).
- **Core 1 victim init:** `multicore_lockout_victim_init()` called at the start of `pdm_core1_entry()`.
- **Interrupt blackout:** ~45ms for sector erase + program. All interrupts disabled on Core 0 during this window. The existing mute strategy (`preset_loading` + `preset_mute_counter`) and feedback reseed cover the audio gap.

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

### Preset Directory Fields (Version 11)
*Last updated: 2026-07-07 (V11 appends the Control Surfaces IR command table)*

`DIR_VERSION_CURRENT` = 11. V4 renamed the former `include_pins` byte to
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
longer persist). This growth makes
`sizeof(PresetDirectory)` 1433 bytes (1301 at V10, 789 at V9), still within
the single 4 KB directory sector. See
`Documentation/Features/output_config_independent_load.md`,
`Documentation/Features/control_interfaces_spec.md`, and
`Documentation/Features/control_surfaces_spec.md`.

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
| cs_ir | Control Surfaces IR command table (V11+, 132-byte `CsIrConfig`: version + 8x 16-byte `IrCommand`; all-zero = every sub-slot empty = idle; board-level, survives factory reset) |

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
*Last updated: 2026-05-31 (bulk-apply & factory-reset now suspend SPDIF RX across the swap)*

All preset operations (load, save, delete) are **deferred from the USB IRQ to the main loop** via pending flags (`preset_load_pending`, `preset_save_pending`, `preset_delete_pending` in `usb_audio.c`). This avoids running flash operations inside the USB ISR (which would cause a ~45ms interrupt blackout inside an interrupt handler) and allows proper pipeline reset bracketing.

**SPDIF-RX suspend across disruptive state swaps (required when SPDIF is the active input).** Any deferred handler that mutates the live DSP state (coefficients, matrix, delays, output types/pins) or tears down output slots must first stop the SPDIF receiver and restart it afterward. While RX runs, `pico_spdif_rx`'s decode-timeout alarm fires on a separate timer IRQ and can touch PIO/DMA state mid-mutation; left running, this races the swap and faults the core, which the 8-second `watchdog_enable()` then resets (RAM is re-initialised, so `buffer_stats_sequence` resets to ~0 — the signature of this reboot). `preset_load_pending`, `process_type_switches`, and `process_pin_changes` have always bracketed RX this way. The `bulk_params_pending` (`REQ_SET_ALL_PARAMS` / 0xA1) and `factory_reset_pending` (`REQ_FACTORY_RESET` / 0x53) handlers were **missing this bracket** and would reboot the device when applied with SPDIF input active (intermittent, ~4/6 under repeated `0xA1`). Both now mirror `preset_load`: after `prepare_pipeline_reset()`, `if (active_input_source == INPUT_SOURCE_SPDIF && spdif_input_get_state() != SPDIF_INPUT_INACTIVE) { spdif_input_stop(); spdif_prefilling = false; }`, and at the end restart via `spdif_input_start()` (guarded by `!input_source_change_pending`, since a bulk-applied source change defers RX management to that handler). `process_type_switches` leaves caller-stopped RX alone (`rx_was_running` check), so the change-mask path doesn't double-start. Verified: 0/10 reboots on `0xA1` with SPDIF locked after the fix (was 4/6).

The main loop handler for each operation follows the pattern:
1. `usb_audio_drain_ring()` — process in-flight audio packets
2. `prepare_pipeline_reset(PRESET_MUTE_SAMPLES)` — wait for Core 1 idle, engage mute
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
*Last updated: 2026-07-19 (upmixer presence bell; wire/slot version row now V26 / V34)*

### Hardware

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| CPU | Dual Cortex-M0+ @ 133 MHz (OC to 307.2 MHz) | Dual Cortex-M33 @ 150 MHz (OC to 307.2 MHz) |
| SRAM | 264 KB | 520 KB |
| FPU | None (software float) | Single-precision VFP |
| DCP | N/A | Double-precision coprocessor |
| VREG | 1.20V (for OC) | 1.10V |
| UART + I2C external control | Yes (identical) | Yes (identical) |
| Control Surfaces nouns (caps v4) | 49 in table, `ADAT_ACTIVE` + the 6 upmixer nouns unusable (empty action mask) | 49, all usable |
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
| USB input bit depth | 16-bit or 24-bit (alt) | 16/24-bit (stereo) or 16-bit (multichannel) |
| AS alt settings | 0, 1 (16-bit), 2 (24-bit) | 0, 1, 2, 3 (4ch), 4 (6ch), 5 (8ch) |
| Wire / slot version | V26 / V34 | V26 / V34 |
| S/PDIF bit depth | 24-bit | 24-bit |
| S/PDIF input conversion | 24-bit sign-extended full-scale → Q28 via `>> 2` (equivalent to `sample << 6`) | 24-bit sign-extended full-scale → float via `÷ 2147483648.0f` |
| S/PDIF output conversion | Q28 >> 6 → int24 | float × 8388607 → int24 |
| Volume leveller | Q28 envelope + float gain; 2 channels, 2 lookahead rings; max boost clamped to 18 dB (Q28 gain range) | Float throughout; 2 to 8 active channels, mask-driven, 8 lookahead rings; full 35 dB max boost |
| Loudness | Per output, post-gain: 2 Q28 shelf biquads; `loudness_output_mask` (5 outputs) | Per output, post-gain: 2 SVF shelves; `loudness_output_mask` (9 outputs). Both platforms: volume-keyed, works in stereo and multichannel input modes |
| Crossfeed | Per output pair, post-matrix (PASS 4.5); 2 pairs; `output_pair_mask` (default pair 1) | Per output pair, post-matrix (PASS 4.5); 4 pairs; `output_pair_mask` (default pair 1). Both platforms: shared coeffs, per-pair state, works in every input mode |
| Psychoacoustic bass | Per output, pre-crossover; RBJ Q28 biquads (with pre-drive low-band clamp) | Per output, pre-crossover; TPT SVF float. Both platforms: missing-fundamental NLD, `output_mask`, zero added latency |
| Stereo upmixer | Not available (compiled out; matrix untouched) | Stereo input only: derives C/Ls/Rs into matrix rows 2..4 (passive or adaptive centre; OFF/passive/adaptive surround). Zero-latency steering; deliberate per-row surround Haas delay |
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
| Crossover-stage availability when PDM enabled | Single-core (Core 0 only) | Single-core (Core 0 only) |

### DMA
*Last updated: 2026-07-23 (output rings free-running: zero output DMA IRQs; RP2040 adds reload channels 6-7)*

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Priority | Global bus priority bits | Per-channel high-priority flag |
| Output TX channels (SPDIF **or** I2S, per slot) | 0-1 data + 6-7 reload (`OUTPUT_SLOT_RELOAD_DMA_BASE` 6 + slot) | 0-3 (ENDLESS, no reload) |
| Output TX IRQ (SPDIF + I2S) | None (free-running ring) | None (free-running ring) |
| SPDIF RX channels | CH4, CH5 | CH5, CH6 |
| SPDIF RX IRQ | DMA_IRQ_1 (refcount hold; now the only user) | DMA_IRQ_1 (refcount hold; now the only user) |
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
deliberately sit on **different**
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
| sys_clk (all rate families) | 307.2 MHz (VCO 1536 MHz / 5) | 307.2 MHz (VCO 1536 MHz / 5) |
| 44.1 kHz family | Fractional 16.8 PIO divider at fixed 307.2 MHz sys_clk (no clock switching) | Same |
| PLL config | Manual (`set_sys_clock_pll()`) | Automatic (`set_sys_clock_hz()`) |
| I2S BCK/LRCLK default (master + unified pair) | GPIO 14/15 | GPIO 14/15 |
| I2S slave-pair BCK/LRCLK default (SPLIT clock-pin mode; `PICO_I2S_BCK_PIN_SLAVE`) | GPIO 12/13 | GPIO 26/27 |
| Input clock tracking | Soft VCXO (ppm sys_clk trim; see "Soft VCXO") | Soft VCXO (ppm sys_clk trim; see "Soft VCXO") |
| Soft VCXO PWM pacer slice (no GPIO) | Slice 7 (reserved) | Slice 8 (reserved) |

---

## Memory Layout
*Last updated: 2026-07-23 (output-ring recovery snapshots added)*

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

> **ADAT bulk input (2026-07-13, acquisition state updated 2026-07-15; RP2350 only).** The ADAT receiver adds a fixed
> **8 KB BSS ring** (`adat_rx_ring`, 2048 x uint32, aligned to its own size for the
> DMA write-address wrap) plus a few dozen bytes of state (including the current
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
> and link-time-budgeted (a BSS overflow fails the build). The producer pools remain
> heap-resident (allocated once at boot, never churn); they still drive the writer
> via the blocking-give connection and are retained pending the direct-write
> follow-up that will encode into the ring without an intermediate pool. Each
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
| Output slot rings (static, 2 × 16 KB, shared SPDIF/I2S) + up to 16 KB align pad | ~32-48 KB |
| I2S RX DMA ring (1024 × 4, 4 KB aligned) | 4 KB |
| Other BSS | ~20 KB |
| **Total BSS** | **~140 KB** (rings replaced the ~27 KB consumer pools) |
| RAM code+rodata+data (.data section, hot set only) | 44,376 B (was 108,692 under copy_to_ram) |
| Flash-resident code (.text + .rodata + boot2, XIP) | ~98 KB |
| Free RAM | ~36 KB |
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
| Output slot rings (static, 4 × 16 KB, shared SPDIF/I2S) + up to 16 KB align pad | ~64-80 KB |
| I2S RX DMA ring (2048 × 4, 8 KB aligned) | 8 KB |
| ADAT RX ring (`adat_rx_ring`, 2048 × 4, 8 KB aligned) | 8 KB |
| Stereo upmixer state (Haas 2 × 1024 + allpass 2 × 512 floats + estimators + double-buffered coeffs) | ~12.3 KB |
| Other BSS | ~35 KB |
| **Total BSS** | **~346 KB** (measured 353,884 B after the stereo upmixer, + ~12.3 KB over the prior build. RP2040 unchanged: the feature is compiled out and the matrix is untouched) |
| RAM code+rodata+data (.data section, hot set only) | 70,520 B (was 147,332 under copy_to_ram) |
| Flash-resident code (.text + .rodata + boot2, XIP) | ~98 KB |
| Free RAM | ~71 KB (rings replaced the ~55 KB consumer pools) |
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
| S/PDIF DMA transfer | Free-running (whole ring; no fixed transfer quantum) |
| S/PDIF slot ring | 1024 frames per slot (21.3 ms at 48 kHz); target fill ~50% |
| S/PDIF producer pool | 8 buffers × 192 samples per output pair (retained, feeds the ring writer) |
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
*Last updated: 2026-07-04*

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
`bulk_param_buf` and holds the bulk lock until the caller releases it.

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
*Last updated: 2026-07-22 (PWM LED bind rejects the soft-VCXO reserved slice; caps v4 unchanged)*

User-wired push buttons, toggle switches, potentiometers, quadrature rotary
encoders, plain indicator LEDs, PWM-dimmed LEDs, and an IR remote receiver on
spare GPIOs, configured over vendor commands `0x84`-`0x87`, `0x8B`/`0x8C`
(per-slot names), `0x8D`-`0x8F` (IR commands and learn), and `0x9D`/`0x9E`
(save/revert). A binding attaches one component (`CsType`)
to one firmware parameter (`CsNoun`) through one operation (`CsAction`), on one
or two GPIOs. The full integrator spec is
`Documentation/Features/control_surfaces_spec.md`.

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

### File layout

- `control_surfaces.c` / `.h`: the engine and the wire/flash data model
  (`CsBinding` 24 B, `CsFlashConfig` 388 B, `IrCommand` 16 B, `CsIrConfig`
  132 B, `CsCapsHeader` 40 B, `CsNounDesc` 12 B, `CsStatusPacket` 32 B, with a
  `uint16_t active_mask` for the 16 slots and a `uint8_t ir_active_mask` for
  the 8 IR sub-slots).
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
  with the toggle bit masked, and a stable FNV-1a timing-signature hash for
  everything else, surfacing press / repeat / release events plus the learn
  state machine (arm, 10 s window, capture-first-press).
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
Q step in octaves (default one-twelfth octave). Deferred nouns (`CS_NDF_DEFERRED`)
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
buttons are up to 8 `IrCommand` sub-slots in a separate table: button-subset
noun/action records (INC/DEC/TOGGLE/SET/TRIGGER/MOMENTARY, WRAP/REPEAT flags)
fired by a learned `{protocol, code}` pair instead of a GPIO edge, validated
against the same caps masks and dispatched through the same `cs_noun_dispatch`
path with per-command `CsOpState` (BUSY retry, deferred-noun shadow,
momentary restore). Multiple commands may share one code (one button, several
actions). Hold semantics mirror physical buttons: a NEC repeat frame or a
re-transmission of the same code within 250 ms extends the hold (REPEAT
events, gated to the button feel of 400 ms delay then 12.5 Hz); 250 ms of
silence releases (restoring MOMENTARY). Learn (`REQ_CS_IR_LEARN`: wValue 1
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

### Persistence and platform placement

The binding table is device-global in the preset directory (388-byte
`CsFlashConfig`: version + 16x 24-byte `CsBinding`), board-level like
`dac_hw_mute` and the control-interface config; it survives preset changes and
factory reset and is not part of `WireBulkParams`. The per-slot names live
next to it (V10, `cs_names[16][32]`) and the IR command table follows (V11,
132-byte `CsIrConfig`: version + 8x 16-byte `IrCommand`), all with the same
lifetime. On RP2040
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
op state ~400 B).

---

## Vendor Command Reference
*Last updated: 2026-07-18 (stereo upmixer commands 0x4A-0x4E added, RP2350 only)*

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
| REQ_GET_PLATFORM | 0x7F | IN | Get platform ID (0=RP2040, 1=RP2350) |
| REQ_GET_CLOCK_DIAG | 0x82 | IN | Get packed ClockDiagPacket clock/audio-path snapshot (DSPI_CLOCK_DIAG builds; zero-length otherwise). See Clock Diagnostics |
| REQ_CLEAR_CLIPS | 0x83 | IN | Read-then-clear clip flags (see Clip Detection) |
| REQ_SET_CS_BINDING | 0x84 | OUT | Set a Control Surfaces binding (wValue=slot 0-15, payload=24-byte CsBinding, required; short payload = INVALID_VALUE); apply-live-only preview, deferred, poll 0x87; persist via REQ_CS_SAVE (see Control Surfaces) |
| REQ_GET_CS_BINDING | 0x85 | IN | Get the live 24-byte CsBinding for a slot (wValue=slot) |
| REQ_GET_CS_CAPS | 0x86 | IN | Get capability tables (wValue=0xFFFF: 40-byte header+type table+max_ir_commands, caps v4; wValue=noun: 12-byte CsNounDesc) |
| REQ_GET_CS_STATUS | 0x87 | IN | Get 32-byte CsStatusPacket (last SET result, dirty flag, active_mask, per-slot status, ir_active_mask, learn state, per-sub-slot IR status) |
| REQ_SET_CS_NAME | 0x8B | OUT | Set a Control Surfaces slot name (wValue=slot 0-15, payload=1-32 bytes; one NUL byte clears); apply-live-only preview (persist via 0x9D), poll 0x87 for result |
| REQ_GET_CS_NAME | 0x8C | IN | Get a Control Surfaces slot name (wValue=slot, returns 32 bytes NUL-terminated, live) |
| REQ_SET_CS_IR_CMD | 0x8D | OUT | Set a Control Surfaces IR remote command (wValue=sub-slot 0-7, payload=16-byte IrCommand); apply-live-only, deferred; poll 0x87 (last_slot = 0x80\|sub) |
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
| REQ_CS_SAVE | 0x9D | IN | Persist the whole live Control Surfaces config (bindings + IR commands) in one directory write; deferred, poll 0x87 (last_slot=0xFF); clears the dirty flag |
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
| REQ_SET_INPUT_SOURCE | 0xE0 | OUT | Set active input source (0=USB, 1=SPDIF, 2=I2S, 3=ADAT, 4=SPDIF2, 5=SPDIF3). SPDIF2/3 rejected unless enabled; ADAT rejected unless enabled with a pin set (RP2350) |
| REQ_GET_INPUT_SOURCE | 0xE1 | IN | Get active input source |
| REQ_GET_SPDIF_RX_STATUS | 0xE2 | IN | Get SPDIF RX status (16-byte SpdifRxStatusPacket) |
| REQ_GET_SPDIF_RX_CH_STATUS | 0xE3 | IN | Get IEC 60958 channel status (24 bytes) |
| REQ_SET_SPDIF_RX_PIN | 0xE4 | IN* | Set a SPDIF input's RX GPIO (wValue = (index<<8)\|pin, index 0..2; old hosts sending just a pin target index 0; pin 0xFF = reset that input to its default). Enabled inputs conflict-check the new pin; a disabled 2/3 stores it as a preference. Returns status byte |
| REQ_GET_SPDIF_RX_PIN | 0xE5 | IN | Get a SPDIF input's RX GPIO (wValue = index 0..2) |
| REQ_SET_LG_SOUND_SYNC_ENABLE | 0xE6 | OUT | Set LG Sound Sync enable flag (per-preset; live until REQ_SAVE_PRESET) |
| REQ_GET_LG_SOUND_SYNC_ENABLE | 0xE7 | IN | Get LG Sound Sync enable flag |
| REQ_GET_LG_SOUND_SYNC_STATUS | 0xE8 | IN | Get 16-byte LgSoundSyncStatus (enabled/present/volume/muted + reserved) |
| REQ_SET_SPDIF_INPUT_ENABLE | 0xE9 | IN* | Enable/disable optional SPDIF input 2 or 3 (wValue = (index<<8)\|enable, index 1..2). Enabling validates the configured pin (PIN_IN_USE on conflict); disabling the live/pending source is rejected. RAM-only; persist via REQ_PRESET_SAVE. Returns PIN_CONFIG_* status byte |
| REQ_SET_INPUT_RATE | 0xED | OUT | Set I2S input sample rate (uint32_t Hz: 44100/48000/96000; applied live when I2S input active in master clock mode; stored only in slave mode, where the rate is auto-detected) |
| REQ_GET_INPUT_RATE | 0xEE | IN | Returns 8 bytes: 2x uint32_t {current pipeline Hz, selected I2S input Hz} |
| REQ_GET_SPDIF_INPUT_CONFIG | 0xEF | IN | Get 5 bytes: input count (3), enable mask over all inputs (bit 0 = input 1, always set), then GPIOs for inputs 1..3. Lets a host build its source list data-driven |
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
*Last updated: 2026-07-18 (wire V25: 44-byte stereo upmixer section appended; total 5944 bytes)*

Transfers the complete DSP state in a single USB control transfer (3664 bytes at V11/V12), replacing dozens of individual vendor requests.

**Wire format:** `WireBulkParams` (`bulk_params.h`, `WIRE_FORMAT_VERSION` 25, total 5944 bytes); packed struct with header, global params, crossfeed, legacy channel gains, delays, matrix crosspoints, matrix outputs, pin config, EQ bands, channel names, I2S config, leveller config, preamp config (`WirePreampConfig`, 16 bytes), master volume config (`WireMasterVolume`, 16 bytes), input source config (`WireInputConfig`, 16 bytes), LG Sound Sync (`WireLgSoundSync`, 16 bytes), user volume/mute (`WireUserVolume`, 16 bytes), DAC hardware mute (`WireDacHwMute`, 16 bytes, V10+), and **crossover bands** (`WireCrossoverConfig`, 704 bytes = 11 × 4 × `WireBandParams`, V11+). V12 claims two reserved bytes inside `WireInputConfig` for `i2s_rx_pin` and `i2s_input_rate` (enum 0=44100, 1=48000, 2=96000); V12 payloads are byte-identical in size to V11. All arrays sized at platform maximums (RP2350: 11 channels, 9 outputs, 5 pins, 12 PEQ bands, 4 crossover bands per channel). Unused entries zero-padded; for crossover, master rows (channel < `CH_OUT_1`) are zeroed on collect and skipped on apply. **V20** repurposes the `WireCrossfeedParams` reserved byte (offset 3) as `output_pair_mask` (bit p = crossfeed on output pair p); struct sizes are unchanged. **V22** carries the Linkwitz-Transform target `Q` in the EQ `WireBandParams.reserved[2]` bytes (`uint16` LE, `Q*512`; zero for non-LT types), so struct sizes stay unchanged. (V21 claimed one `WireInputConfig` reserved byte for the I2S clock master/slave mode, also size-neutral.) **V23** tail-appends the 24-byte `WirePsybassParams` (psychoacoustic bass: `enabled` + `output_mask` + five floats), bringing the total to 5900 bytes. **V24** claims three `WireInputConfig` reserved bytes for the ADAT input (`adat_input_pin`, `adat_input_enabled_p1`, `adat_clock_mode_p1`, each 0 = absent/keep-live); struct sizes and the 5900-byte total are unchanged. **V25** tail-appends the 44-byte `WireUpmixParams` (RP2350 stereo upmixer: enabled + centre/surround modes + reserved + ten floats; layout-identical to `UpmixConfigPacket`), bringing the total to 5944 bytes; the section is zeroed on collect and ignored on apply on RP2040.

**Per-version size anchors** live in `bulk_params.h` (`WIRE_BULK_PARAMS_V{N}_SIZE`, N=2..12). Each legacy-section apply gate inside `bulk_params_apply()` compares `payload_length` against its own version's anchor, NOT against `sizeof(WireBulkParams)`. Without this discipline, growing the struct would silently lock older payloads out of the very tail sections they own (e.g. a V10 payload would stop applying its DAC-mute section the moment V11 was added). V<11 payloads leave crossover state untouched on apply; V<12 payloads leave the I2S input pin/rate untouched.

**Transport:** Multi-packet USB EP0 control transfers using `usb_stream_transfer` from pico-extras. Packets are 64 bytes. No modifications to `usb_device.c` required — uses only public API (`usb_stream_setup_transfer`, `usb_start_transfer`, `usb_start_empty_transfer`).

**GET (0xA0):** `bulk_params_collect()` snapshots live state into `bulk_param_buf`, then streams it out in 64-byte packets via `usb_stream_transfer`. ZLP appended if total length is a multiple of 64.

**SET (0xA1):** Incoming data accumulated into `bulk_param_buf` via `usb_stream_transfer`. On completion, `bulk_params_pending` flag is set (after status-phase ACK). Main loop processes deferred: snapshots `output_types[]`, waits for Core 1 idle, mutes audio (256 samples), calls `bulk_params_apply()` with pin application gated on `output_config_mode` (applied only in with-preset mode; independent mode leaves device-global IO to `REQ_SAVE_OUTPUT_CONFIG`), recalculates all filters and delays, transitions Core 1 mode to match the new output enable state, then diffs the new `output_types[]` against the snapshot. If any slot's type changed, dispatches `process_type_switches()` to reconfigure SPDIF/I2S hardware (mirrors the `preset_load_pending` pattern); otherwise calls `complete_pipeline_reset()` to resync output streams (or `reset_usb_feedback_loop()` when SPDIF input is active, to avoid disrupting the prefill handshake).

**Buffer:** 4 KB aligned static buffer in `usb_audio.c`, shared between GET and SET. Platform validation rejects mismatched `platform_id` or `num_channels`.

### Buffer Statistics
*Last updated: 2026-07-23 (consumer pool replaced by the slot ring; stats via a 48-frames-per-bucket mapping)*

Real-time buffer fill level monitoring for the SPDIF slot rings (DMA-side) and PDM buffers, accessible via USB vendor commands. Enables host applications to diagnose audio glitches, near-miss underruns, and pipeline health. Producer (USB-side) pool stats are not tracked because `producer_pool_blocking_give` returns buffers synchronously; the producer pool is always fully free between USB packets.

**Ring-to-bucket mapping (2026-07-23).** The wire format and thresholds are unchanged: each slot's ring fill is mapped to legacy "buffer" buckets of 48 frames (a ring fill of N frames reports N/48 buckets, capped at `SPDIF_CONSUMER_BUFFER_COUNT` = 16), so `REQ_GET_BUFFER_STATS`, the fill percentages, and every prefill threshold keep byte- and semantics-compatibility with the old consumer-pool counts.

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
- **Pipeline reset API** (`main.c`): two-phase `prepare_pipeline_reset()` / `complete_pipeline_reset()` brackets any disruptive output work. `complete_pipeline_reset()` is structured to keep the IRQ-disabled critical section tiny; only the synchronized start (`start_output_slots_in_sync()`, single combined SM mask for both output types since 2026-07-21; see "Synchronized Start") needs atomicity (so all output slots start their PIO SMs on the same clock cycle, preserving CLAUDE.md's slot-alignment invariant). Per-slot teardown (`teardown_output_slot()` helper) runs with main interrupts ENABLED: the per-instance `enabled` flag is set false first (the shared DMA IRQ handler skips disabled instances), the channel's DMA IRQ is masked, and the pool/abort operations only touch this instance's state. The USB audio class ISR can continue to drain packets into the SPSC `audio_ring` throughout the teardown, eliminating the prior ~1 ms USB starvation that compounded I2S DAC pops during input-source switches. Core 1 is held idle by `prepare_pipeline_reset()` (spin-wait for `work_done`) and `preset_loading=true` (blocks new dispatch). `drain_and_disable_outputs()` shares the same `teardown_output_slot()` helper. I2S→S/PDIF switch restores the SPDIF connection before zeroing the I2S instance to prevent a dangling `producer_pool->connection` pointer. *Last updated: 2026-05-12*
- **Boot-time I2S restoration:** `core0_init()` inspects `output_types[]` after `preset_boot_load()` and converts preset-saved I2S slots from the default SPDIF instances created by `usb_sound_card_init()`. For each I2S slot: disables SPDIF, unclaims the PIO SM, calls `audio_i2s_setup()` + `audio_i2s_connect_extra()`, and enables the I2S instance. MCK is started if any I2S slot exists and `i2s_mck_enabled` is set.
- **MCK pin migration on preset apply:** Preset and bulk-params apply paths validate the loaded `i2s_mck_pin` against `GPIO_TO_GPOUT_CLOCK_HANDLE()`. If the stored pin has no GPOUTn mapping on the current platform (typical case: an RP2040 board loading a preset saved with `mck_pin = 13`, which is GPOUTn-capable on RP2350 only), the pin falls back to `PICO_I2S_MCK_PIN` (platform default) and `i2s_mck_enabled` is forced to `false`. Defaults: GPIO 13 on RP2350, GPIO 21 on RP2040 (the only board-friendly GPOUTn pin on RP2040 — GPIOs 23–25 are also GPOUTn-capable but reserved for control / SMPS / LED).
- **MCK enable order:** `process_type_switches()` writes the MCK clkdiv via `audio_i2s_mck_update_frequency()` *before* calling `audio_i2s_mck_set_enabled(true)`, so the CLK_GPOUTn block starts at the correct frequency rather than briefly running at whatever DIV value the block previously held (which would cause a transient PLL-relock chirp on connected DACs). Matches the `REQ_SET_MCK_ENABLE` vendor command order.
- **SPDIF RX is suspended across `process_type_switches()`:** the function shares `DMA_IRQ_1` between SPDIF TX and `pico_spdif_rx`. Forcing the IRQ off mid-transition would silence RX DMA completion handling for the duration; RX decode-timeout alarms (separate timer IRQ) could also fire and access PIO/DMA state being mutated. The function snapshots RX state at entry, calls `spdif_input_stop()` if it was running, and restarts it after `complete_pipeline_reset()` finishes — guarded by `active_input_source == INPUT_SOURCE_SPDIF && !input_source_change_pending` so a deferred input-source switch isn't pre-empted. Callers that have already stopped RX (e.g. `preset_load_pending` across the flash blackout) see no double-stop because `state == INACTIVE` triggers the no-op path; those callers are responsible for their own restart, which `preset_load_pending` now does once at the end of its block rather than before `process_type_switches`.

*Last updated: 2026-05-12*

### Master Clock (CLK_GPOUTn)
*Last updated: 2026-05-09*

- **No PIO state machine consumed.** MCK is generated by one of the four hardware **CLK_GPOUTn** clock outputs (`clk_gpout0..3`). The previous 2-instruction PIO toggle program (`audio_mck.pio`) is gone; PIO1 SM1 is free for future use.
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
*Last updated: 2026-07-13 (LUT-driven NRZI encoder; single float->S24 conversion point shared with the slots)*

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
rejected (a two-mode encoder would need mid-stream resync); the encode LUT is intentionally
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
*Last updated: 2026-07-13 (ADAT input added as source 3, RP2350 only; see "ADAT Input")*

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
} InputSource;
```

`input_source_valid()` accepts every value 0..5 (ADAT = 3 is structurally valid on both platforms so presets round-trip; RP2040 keeps only the config state and can never select it); `input_source_selectable()` additionally requires that a SPDIF 2/3 be enabled, or that ADAT be enabled with a pin set on RP2350, before it is offered. `input_source_is_spdif()`, `spdif_index_for_source()` (0..2), and `spdif_source_for_index()` map between the two SPDIF representations.

### Multiple Selectable SPDIF Inputs
*Last updated: 2026-07-08*

Up to `SPDIF_RX_NUM_INPUTS` (3) SPDIF inputs share the single RX PIO state machine and DMA pair; only the active one ever claims its GPIO. SPDIF input 1 (`INPUT_SOURCE_SPDIF`, default GPIO 5) is always enabled and behaves exactly as before. The optional inputs 2 and 3 (`INPUT_SOURCE_SPDIF2` / `INPUT_SOURCE_SPDIF3`, default GPIOs 20 and 21) are **disabled by default**.

- **Enable model.** `spdif_rx_enabled_ext` is a 2-bit mask (bit 0 = SPDIF2, bit 1 = SPDIF3). While an input is disabled it is not offered by `REQ_SET_INPUT_SOURCE`, and its configured pin is a stored preference only: invisible to pin-conflict validation, so other functions may use GPIO 20/21 freely. Enabling validates the configured pin via `spdif_input_enable_acceptable()` (valid GPIO, not claimed by any other function) and is rejected with `PIN_IN_USE` on a conflict. Disabling the input that is the active source (or the target of a pending source switch) is rejected; the host must switch away first.
- **GPIO lifecycle.** `spdif_rx_pin_for_index()` resolves an input's pin (`spdif_rx_pin` for index 0, `spdif_rx_pin_ext[]` for 1..2) and `spdif_rx_active_pin()` gives the GPIO the RX library should run on for the current source. `spdif_input_stop()` now resets the RX GPIO to `GPIO_FUNC_NULL` (the vendored `pico_spdif_rx` library never did); it releases the pin it recorded at start time, not the current config, so a pin change or an input switch actually frees the previous pad.
- **Switching between SPDIF inputs** reuses the existing deferred source-switch machinery (full stop, pipeline reset, restart on the new GPIO, outputs muted until lock). Inter-slot output alignment is preserved by the same complete-reset path as every other source switch.
- **Vendor commands.** `REQ_SET_SPDIF_RX_PIN` (0xE4) and `REQ_GET_SPDIF_RX_PIN` (0xE5) carry the input index (`wValue = (index<<8)|GPIO` on SET, `wValue = index` on GET); a bare pin from an old host targets index 0. `REQ_SET_SPDIF_INPUT_ENABLE` (0xE9, `wValue = (index<<8)|enable`, index 1..2) toggles the mask; `REQ_GET_SPDIF_INPUT_CONFIG` (0xEF) returns the input count, the enable mask over all inputs (bit 0 always set), and all three GPIOs so a host can build its source list data-driven.
- **Default channel names.** SPDIF input 1 keeps "SPDIF L/R"; inputs 2/3 produce "SPDIF 2 L/R" / "SPDIF 3 L/R" so the host can tell them apart.
- **Platform-independent:** identical on RP2040 and RP2350.

### Switching Behavior

- Source switching is deferred to the main loop via `input_source_change_pending` / `pending_input_source` flags (same pattern as output type switching)
- On switch: drain USB ring, `prepare_pipeline_reset()`, update `active_input_source`, `complete_pipeline_reset()`
- When input is not USB, `usb_audio_drain_ring()` is skipped — USB enumeration stays active but audio data is silently dropped
- SPDIF RX hardware only runs when SPDIF is the selected input source; I2S RX hardware only runs when I2S is the selected input source. The two share the same PIO SM and DMA channels, which is safe because inputs are switched, never mixed. The three SPDIF inputs likewise share the single RX SM/DMA pair; switching between them runs the same full stop/restart so only the active input's GPIO is ever claimed
- Switching to I2S applies the selected `i2s_input_rate` (via `perform_rate_change()` when it differs from the current rate), then runs the same drain/prefill-to-50%/enable-in-sync handshake as SPDIF (minus the lock wait, since the synchronous input runs as soon as it is started). See the I2S Input prefill note below

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
- Configurable via `REQ_SET_SPDIF_RX_PIN` (0xE4) / `REQ_GET_SPDIF_RX_PIN` (0xE5); both now carry the input index (0..2) in `wValue`. Enabled inputs conflict-check a new pin; a disabled 2/3's pin is a stored preference validated at enable time (`REQ_SET_SPDIF_INPUT_ENABLE`, 0xE9).
- **On-flash layout:** `spdif_rx_pin` lives in one byte that V13 originally reserved as `input_source_padding[0]`. Reusing that byte keeps the `PresetSlot` size unchanged, so existing V13 presets remain CRC-valid (their padding bytes were zero-initialised, which fails GPIO validity and falls through to the live default — same observable behaviour as before this change).
- **Boot-time bootstrap.** `preset_boot_load` still reads the directory's legacy `spdif_rx_pin` field as the initial live value. This means users upgrading from auto-flush firmware keep their previously-configured pin until they save a preset under the new firmware. After that, the slot's value drives behaviour and the directory field is no longer consulted on subsequent boots that load the same slot.
- **Hot-swap supported.** Pin changes (from vendor command, bulk params apply, or preset load) while SPDIF input is active set `spdif_rx_pin_change_pending`; the main-loop deferred handler runs `spdif_input_stop()` → `prepare_pipeline_reset()` → `spdif_input_start()` so the running RX library picks up the new GPIO. Deferred to main loop because the `pico_spdif_rx` library's teardown (program removal, IRQ handler removal, DMA channel unclaim) is not safe to perform from USB ISR context.
- **`bulk_params_apply` integration.** `WireInputConfig.spdif_rx_pin` is applied on bulk SET when `apply_pins == true`, mirroring how `output_pins[]` is applied. If the new pin differs from the current one and SPDIF input is active, the hot-swap fires.

### SPDIF RX Implementation
*Last updated: 2026-07-20*

**Library**: Forked from `elehobica/pico_spdif_rx` v0.9.3 at `firmware/pico-extras/src/rp2_common/pico_spdif_rx/`.

**DSPi library patches:**
- PIO2 support for RP2350
- Clock constants: 307.2 MHz sys_clk, 122.88 MHz PIO clock (divider 2.5 exact)
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

**Clock**: sys_clk 307.2 MHz → PIO clock 122.88 MHz (divider 2.5, exact). At 122.88 MHz: cy=20 (48kHz), cy=10 (96kHz), cy=5 (192kHz) — identical to original library values, zero error.

**DMA**: Channels 5+6 (RP2350) or 4+5 (RP2040) on DMA_IRQ_0 (shared with I2S TX when active). DMA_IRQ_1 is dedicated to SPDIF TX only. This isolates SPDIF RX from SPDIF TX, avoiding shared handler conflicts.

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

`input_servo_reset()` zeroes the integrator and parks the VCXO; it is called on tracking start, (re)lock, and every stop/disarm path, so USB and I2S-master modes always run at exactly nominal sys_clk. **Servo gating**: the SPDIF and I2S-slave servos hold off until the detected rate equals `audio_state.freq` (the deferred rate change has landed; otherwise the feed-forward would be computed against the wrong divider). A signal-loss (RELOCKING) event deliberately **holds the last ppm** rather than parking, so relock re-converges from where it was; full stop/disarm/source-switch paths park. **I2S slave** now shares `input_servo_apply()` (its old private P-only divider-writing servo, `i2s_slave_update_clock_servo()`, is gone in substance), passing the first SPDIF-type slot as fill reference or -1 when none exists (rate term alone, ~0.1 ppm long-window); the edge-locked I2S TX slots follow the external BCK directly and need no servo, while PDM and SPDIF slots track via sys_clk. **Divider-ratio coupling moved to the nominal path**: the old `i2s_div = 2 * spdif_div` servo coupling is gone; a common sys_clk trim can never correct a divider *ratio* error, so the coupling lives in the nominal dividers themselves; `i2s_compute_divider()` (and the matching RX clock-master divider `rx_master_divider_24_8()`) now derive the I2S divider as exactly `2 * ceil(sys/f)` instead of `ceil(2*sys/f)` (identical at 44.1/48/96 kHz, future-proofs rates like 176.4 kHz where they differ by one LSB / ~287 ppm). Verified on RP2350 hardware at 48 kHz SPDIF input; the standalone fbdiv-dither mechanism is bench-proven, but the full-firmware servo soak (SPDIF, ADAT slave, I2S slave at 44.1/48/96 kHz on both platforms) and RP2040 fbdiv dithering are HW-untested. *Last updated: 2026-07-22*

**Rate measurement (long-window estimator)**: this estimator supplies the `actual_freq` argument to `input_servo_apply()` (feeding the ppm feed-forward above). The servo's rate term does not feed the library's `spdif_rx_get_samp_freq_actual()` straight in: that value is an 8-block moving average of DMA IRQ intervals which telescopes to two IRQ timestamps only ~32 ms apart (16 ms at 96 kHz), so a few microseconds of IRQ latency jitter is 100+ ppm of rate noise (which under the former divider-actuated servo slammed the 16.8 output dividers across several LSBs every tick, bounced the fill, and FM'd the outgoing SPDIF carrier; the soft VCXO would likewise wobble sys_clk). `spdif_input.c` mirrors the ADAT slave dual-anchor scheme: it reads the library's coherent `{decoded block count, IRQ timestamp}` pair (`spdif_rx_get_block_ref()`, exact 192-frame blocks while stable), keeps two anchors rotated every 8 s (steady-state span 8-16 s, endpoint jitter well under 1 ppm), and trusts the count-based rate once the span reaches 1 s (~10 ppm noise, well inside the servo's authority). The first second after lock is bridged by an IIR-smoothed (alpha 0.25) library estimate. Anchors re-arm on every lock acquisition, so counts never span the library's `block_count` reset on relock. *Last updated: 2026-07-22*

**Note (superseded by the soft VCXO)**: the earlier "slew-limit the applied divider to ±1 LSB per call" idea is moot now that the servo actuates a ppm sys_clk trim rather than a PIO divider. The soft VCXO's own control quantum is one pulse (~1.17 sys cycles of phase, ~3.8 ns), far finer than a 156 ppm divider LSB, and the long-window estimator keeps the rate term sub-ppm, so output clock wander is already bounded without an explicit slew limit. *Last updated: 2026-07-22*

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

**Resources.** Reuses the SPDIF RX footprint (free whenever SPDIF input is inactive) and, for multichannel, the DMA channels the SPDIF/I2S TX DMA-sharing work freed: PIO1 SM2 (RP2040, 1 pair) / PIO2 SM0..3 (RP2350). DMA channel allocation now differs by platform. **RP2350**: one ENDLESS-mode data channel per pair, `I2S_RX_DMA_BASE + p` (pair 0 = `PICO_SPDIF_RX_DMA_CH0` = CH5, pairs 1..3 = CH6/7/8), no reload channels; this freed CH9-12 for the spare pool and the soft VCXO (see DMA map above). **RP2040** (no ENDLESS mode): pair 0 uses a data+reload channel pair `I2S_RX_DMA_BASE + 2p` / `+2p+1` = CH4/5 (`PICO_SPDIF_RX_DMA_CH0/CH1`), stereo-only. A `_Static_assert` in `i2s_input.c` pins the I2S RX range below `SOFT_VCXO_DMA_PULSE` on both platforms. All pairs' SMs and DMA channels are claimed in `i2s_input_start()` and unclaimed in `i2s_input_stop()` so the SDK claim table stays consistent across input switches.

**IRQ-less DMA ring.** Each pair's data channel moves PIO RX FIFO words into a power-of-2-aligned word ring (write-address wrap, transfer count = ring words). Zero IRQs, so capture survives IRQ-disabled windows. The re-arming mechanism is platform-specific. **RP2350** uses a single ENDLESS-mode channel per pair (transfer count auto-reloads, write address wraps in hardware), with no reload/chain channel; **RP2040** (no ENDLESS mode) pairs each data channel with a reload channel: the data channel chains to it, and it rewrites the data channel's `al2_write_addr_trig` with the ring base, retriggering it forever. **Teardown must be race-free.** On RP2040 the two chained channels re-trigger each other, so aborting them with two sequential `dma_channel_abort()` calls lets one re-arm the other in the gap, intermittently hanging the abort's busy-wait (watchdog reset) or leaving a channel live after unclaim. `stop_all_dma_rings()` instead disarms every data channel's chain via the non-triggering CTRL alias, then aborts all channels in a single grouped `dma_hw->abort` write (bounded busy-wait guard); on RP2350 the ENDLESS channels have no chain loop, so the grouped abort alone suffices, followed by a transfer-count=0 restore so a stale encoded count cannot confuse a later restart. Every input stop (input-source switch, output type switch, flash bracket, pin hot-swap) goes through this path. `i2s_input_poll()` derives the fill level from the DMA write address, consumes whole stereo pairs only (capped at 192 frames per poll), masks the low byte, applies preamp (same Q28/float conversion conventions as SPDIF RX), and feeds `process_input_block()`. **It batches to a 48-frame minimum** (`I2S_INPUT_MIN_BLOCK`): the ring's write address advances per word, so without batching the fast main loop would feed `process_input_block()` a handful of frames at a time, and the budget-based CPU meter (`busy_us / (frames / Fs)`) would amortize the fixed per-block cost (Core 1 EQ-worker handshake, pipeline setup) over too few samples and read ~4x inflated (measured cpu0 66% vs USB 16% before batching). 48 frames matches the USB packet / consumer-buffer granularity, so I2S CPU tracks USB; the input is continuous so the threshold never starves, and it adds ~1 ms of latency at 48 kHz. (SPDIF RX does not need this: its library FIFO already advances in DMA-block chunks.) Because the ring length is even and the read pointer only moves in pairs, a word's ring position fixes its channel permanently; even a writer-laps-reader overrun garbles audio momentarily but can never swap L/R.

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
| 0xE0 | REQ_SET_INPUT_SOURCE | OUT | Set active input source (uint8_t payload; 2 = I2S, 3 = ADAT, 4 = SPDIF2, 5 = SPDIF3; SPDIF2/3 rejected unless enabled, ADAT rejected unless enabled + pin set on RP2350) |
| 0xE1 | REQ_GET_INPUT_SOURCE | IN | Get active input source (returns uint8_t) |
| 0xE2 | REQ_GET_SPDIF_RX_STATUS | IN | Get SPDIF RX status (16-byte SpdifRxStatusPacket) |
| 0xE3 | REQ_GET_SPDIF_RX_CH_STATUS | IN | Get IEC 60958 channel status (24 bytes) |
| 0xE4 | REQ_SET_SPDIF_RX_PIN | IN* | Set a SPDIF input's RX pin (wValue = (index<<8)\|pin, index 0..2; old hosts sending a bare pin target index 0; pin 0xFF = reset that input to its default). Returns status byte |
| 0xE5 | REQ_GET_SPDIF_RX_PIN | IN | Get a SPDIF input's RX pin (wValue = index 0..2, returns uint8_t) |
| 0xE9 | REQ_SET_SPDIF_INPUT_ENABLE | IN* | Enable/disable optional SPDIF input 2/3 (wValue = (index<<8)\|enable, index 1..2). Returns PIN_CONFIG_* status byte |
| 0x88 | REQ_SET_I2S_CLOCK_MODE | OUT | Set clock mode (0=master, 1=slave); deferred apply |
| 0x89 | REQ_GET_I2S_CLOCK_MODE | IN | Get live clock mode (returns uint8_t) |
| 0x8A | REQ_GET_I2S_SLAVE_STATUS | IN | Get 16-byte I2sSlaveStatusPacket |
| 0xC2 | REQ_SET_I2S_BCK_PIN | IN* | Set a BCK pin; `wValue = (role<<8)|GPIO` (role 0 = master/unified pair, 1 = slave pair; GPIO 0xFF = reset that role's pair to its default) |
| 0xC3 | REQ_GET_I2S_BCK_PIN | IN | Get a BCK pin; `wValue = role` (0 = master/unified, 1 = slave) |
| 0xFE | REQ_SET_I2S_CLOCK_PIN_MODE | IN* | Set clock-pin mode (wValue = 0 unified / 1 split; returns `PIN_CONFIG_*`); SPLIT enable always validates the slave pair, live apply, deferred input restart when the effective pair moves |
| 0xFF | REQ_GET_I2S_CLOCK_PIN_MODE | IN | Get live clock-pin mode (returns uint8_t: 0=unified, 1=split) |
| 0xED | REQ_SET_INPUT_RATE | OUT | Set I2S input rate (uint32_t Hz: 44100/48000/96000; master mode only; stored-not-applied in slave mode) |
| 0xEE | REQ_GET_INPUT_RATE | IN | Returns 2x uint32_t {current pipeline Hz, selected I2S Hz} |
| 0xEF | REQ_GET_SPDIF_INPUT_CONFIG | IN | Returns 5 bytes: input count (3), enable mask (bit 0 = input 1, always set), GPIOs for inputs 1..3 |
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
*Last updated: 2026-07-13 (ADAT input config persisted: wire V24, slot V32, directory V15)*

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


### I2S Clock-Slave Input Mode
*Last updated: 2026-07-11 (clock-pin mode added: UNIFIED shares one BCK/LRCLK pair, SPLIT gives slave clocking its own `i2s_bck_pin_slave` pair; `i2s_effective_bck_pin()` resolves the active pair)*

`i2s_clock_mode` (0 = master, 1 = slave; persisted, see Persistence above) selects who owns BCK/LRCLK while I2S is the input source. In SLAVE mode an external master drives both pins (they become plain inputs; `gpio_init` + input enable, nothing on-chip drives them) and the device only moves data. The mode is dormant for other input sources. Full spec: `Documentation/Features/i2s_slave_input_spec.md`.

**Clock-pin mode (unified vs split).** A second setting, `i2s_clock_pin_mode` (`I2sClockPinMode` in `audio_input.h`), controls WHICH pins the two clock modes use:

- **UNIFIED (0, default, legacy behavior):** master and slave clock modes share one BCK/LRCLK pair (`i2s_bck_pin` / +1, default GPIO 14/15). The slave pair is fully dormant.
- **SPLIT (1):** master clock mode still drives `i2s_bck_pin`, while slave clock mode listens on a separate `i2s_bck_pin_slave` pair (LRCLK = BCK + 1 in both modes). Defaults: GPIO 12/13 on RP2040, GPIO 26/27 on RP2350 (`PICO_I2S_BCK_PIN_SLAVE`). This lets a board dedicate one pair to its own clock outputs and a different pair to the external master's clock inputs, so a hard-wired external master never contends with the device-driven pair.

The helper `i2s_effective_bck_pin()` resolves the pair the active clock mode actually uses (slave pin only when SPLIT **and** slave clock mode, else `i2s_bck_pin`). Every hardware consumer reads it rather than `i2s_bck_pin` directly: TX `clock_pin_base` in `process_type_switches()` (and its same-type-rebuild change detection), the RX start snapshot in `i2s_input.c` (`i2s_active_bck_pin = i2s_effective_bck_pin()`), and `i2s_input_active_bck_pin()`. In SPLIT mode BOTH pairs count as clock-claimed for RX-data, control-interface, and DAC-mute pin validation (`i2s_clock_pin_claimed()`), because the dormant pair is one deferred mode flip away from being driven or listened on; the two pairs are kept mutually distinct by the SET/apply validators. `REQ_SET_I2S_CLOCK_PIN_MODE` (0xFE) applies the mode live (no pending mechanism). Entering SPLIT always validates the slave pair (distinct from the master pair and free of other owners) even as a dormant store, because a later clock-mode flip adopts the pair without rechecking; leaving SPLIT needs no pin checks (the master pair is clock-claimed in every mode). A live slave-clocked set is additionally rejected with OUTPUT_ACTIVE if I2S output slots run, and otherwise arms a deferred input restart onto the new pair. `REQ_SET_I2S_BCK_PIN` (0xC2) takes `wValue = (role << 8) | GPIO` to address either pair (role 0 = master/unified, role 1 = slave); it restarts the input only when the pair being moved is the one the running input currently uses.

**Clock domains.** I2S output slots are edge-slaved with a dedicated wait-driven PIO program (`audio_i2s_dataout_extclk.pio`, 26 instructions, divider 1.0, `external_clock` instance flag in `pico_audio_i2s_multi`): an LRCLK preamble with a BCK skew guard, then one bit out per external BCK falling edge with per-frame LRCLK framing verification (see "Framing-slip watchdog" below). Like the RX slave programs it is authored with placeholder GPIO indices (wait gpio 0 = BCK, 1 = LRCLK) patched at load (`i2s_extclk_load_program()`, reloaded only on a BCK pin change). Because the checked program is too large to share PIO0's 32-slot instruction memory with the master-clocking programs (clkout 8 + dataout 8 + SPDIF 4), the two clocking modes evict each other's programs at load time (`i2s_extclk_evict_program()` and its mirror in the clkout/dataout load paths); safe because clocking mode is global and a mode rebuild tears down every I2S SM first.

**Framing-slip watchdog.** Real-world external masters (USB-I2S bridges such as the Amanero Combo384) glitch or re-frame BCK/LRCLK around stream stop/start and 44.1/48-family rate switches. With free-running slave programs, one runt BCK pulse or LRCLK phase jump would shift the 32-bit word window one bit for the rest of the session; the sign bit lands mid-word (full-scale wrapped garbage) and the word RATE is unchanged, so the rate watchdog can never detect it. Both external-clock programs (`audio_i2s_rx_slave_checked` for RX, `audio_i2s_dataout_extclk` for TX) therefore move/sample the 64 bits per frame by count (deterministic 24-in-32 alignment, autopush/autopull 32) and then verify the LRCLK level at the TWO ADJACENT cells straddling the frame boundary (bit 63 = cell R31 must read HIGH, bit 64 = cell R32 must read LOW, both sampled half a BCK after a rising edge so the pad synchronizers are settled). The adjacent-pair check catches every sub-frame offset (a fixed slip of k cells passes both checks only for k = 0 mod 64); a slip of exactly a whole frame is inherently undetectable from LRCLK (documented spec limit). On a mismatch the program raises **PIO irq flag 7** on its block and re-frames itself at the next LRCLK falling edge, bounding the garbage to ~2 frames; the RX resync path pads the aborted right word (`in null, 1` -> autopush) and the TX path flushes the unsent right-word remainder (`out null, 32`), so L/R word alternation - and therefore channel identity - survives every slip. `i2s_slave_poll()` reads-and-clears both flags (`i2s_slave_slip_check()`: RX block directly, TX blocks via `audio_i2s_extclk_framing_slipped()`) and treats a slip exactly like a clock loss: increment `slip_count` (surfaced in `I2sSlaveStatusPacket`, claims a reserved byte), `i2s_slave_drop_lock()`, and let the main loop's RELOCKING path restart the receiver and re-frame every output through the prefill's gated synchronized start - necessary because a slipped pair/slot is no longer sample-aligned with its peers (inter-slot alignment invariant) and only a full synchronized restart re-establishes that. Stale flags are consumed at every receiver start and by `audio_i2s_enable_sync_prepare()` for extclk instances (the restart IS the handling). The on-chip slave role keeps the plain free-running RX program: our own TX master never glitches, and the checked variant would not fit alongside the clkmaster program. SPDIF, ADAT, and PDM all derive from `sys_clk`, and `i2s_slave_update_clock_servo()` in `i2s_input.c` rate-matches them by calling the shared `input_servo_apply()` (soft-VCXO ppm trim; see "Clock Servo"), referenced to the FIRST SPDIF-type slot as fill reference (an edge-locked I2S slot consumes at exactly the external rate, so its fill can never expose sys_clk error), or the rate term alone (`fill_slot = -1`) when no SPDIF-type slot exists. There are no per-output divider writes and no ADAT servo-divider handoff any more; ADAT and PDM (which formerly did not track in this mode) now follow sys_clk automatically. The I2S slave also gains the centering integrator it never had. MCK output is still forced off in slave mode (a local MCK would be asynchronous to the external clocks); it resumes per its stored config on leaving slave mode.

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
- **SLAVE:** external gear owns the clock. Exact-timing header probes select the 44.1/48 kHz family as described above. Only after a candidate is proven does the receiver arm DMA-word measurement: 32 ms fast windows validate the family and supply the initial fine-rate estimate, while a dual-anchor 8-16 s long window supplies a ~0.1 ppm servo reference. All outputs (SPDIF, I2S, ADAT TX, MCK, PDM) track it via `input_servo_apply()`, exactly like SPDIF input; the servo pulls `sys_clk` in ppm (soft VCXO) rather than writing any output divider, so every derived output clock moves together. `adat_input_check_rate_change()` feeds the standard deferred `pending_rate` mechanism when the detected rate differs from `audio_state.freq`. Slave mode is never parked (`rate_ok` always true): if the device rate is above 48 kHz at switch-in, acquisition still reaches LOCKED and the deferred rate change retunes the pipeline under the switch-in mute before the synchronized prefill enables the output slots; the servo holds off until `adat_rx_detected_rate == audio_state.freq`, so no output clock is slewed during that muted window.

**Prefill flow.** On lock the poll batches whole frames (`ADAT_INPUT_MIN_BLOCK` = 48, ~1 ms at 48k, capped at 192/poll), decodes each into the pipeline input buffers (`buf_l`/`buf_r` + `buf_in_ext[0..5]`) with per-channel preamp, and calls `process_input_block()`. A lap guard skips whole frames (preserving frame phase, since the ring holds an exact number of frames) before the reader can be overwritten.

**Loopback self-test.** The RX pin may deliberately equal the ADAT output pin: `adat_input_start()` only sets the pad's input enable and never touches funcsel, so the PIO reads the pad the ADAT TX drives; this is a zero-hardware loopback self-test. `adat_input_stop()` clears input-enable only, safe on the shared pin.

**RP2350 pad isolation.** RP2350 pads reset isolated (`ISO=1` in `PADS_BANK0`), and `gpio_set_input_enabled()` never clears the isolation latch; only `gpio_set_function()` does. A dedicated external RX pad would therefore stay electrically isolated and the PIO would see a static level (same-pin loopback worked only because the ADAT TX `pio_gpio_init()` unisolated the shared pad). `adat_input_start()` clears `ISO` directly with `hw_clear_bits()` under `#if HAS_PADS_BANK0_ISOLATION`, keeping funcsel untouched so the loopback self-test still works even if RX and TX ever sit on different PIO blocks. *Last updated: 2026-07-15*

**Main-loop integration** (`main.c`): the poll + prefill block, the source-switch branches, the deferred clock-mode handler (`adat_clock_mode_change_pending`), boot-into-ADAT, the `adat_input_restart_pending` handler (enable/pin change while ADAT is the live source), and the `perform_rate_change` hook (`adat_input_on_rate_change`) mirror the SPDIF/I2S input plumbing. Status is surfaced by `REQ_GET_ADAT_INPUT_STATUS` (0x6E, 20-byte `AdatInputStatusPacket`) and `NOTIFY_EVT_ADAT_INPUT_STATE` (0x0B) on every state change.

**Flash-write bracket.** `prepare_flash_write_operation()` treats a LOCKED ADAT input as a live source: it appears in the pre-mute drain, the streaming test, and the fade-settle loop (which calls `adat_input_poll()` until both the settle interval and the DAC hardware-mute hold have elapsed), matching the guarantees given to USB/SPDIF/I2S (see "DAC Hardware Mute"). Unlike SPDIF and I2S, ADAT is deliberately NOT stopped for the ~45 ms blackout: its RX is an IRQ-less free-running DMA ring with no decode-timeout alarms to race the blackout edge, and the poll re-acquires frame sync after the ring laps; `preset_loading` (still set from `prepare_pipeline_reset()`) then re-runs the drain/prefill/enable handshake once LOCKED returns.

**Resources.** PIO1 SM2 (PIO1 SM0 = PDM, SM1 = ADAT TX); DMA channel 15 (RX ring, ENDLESS mode, no IRQ). The 8 KB `adat_rx_ring` is fixed BSS; the decode hot path (`adat_input_poll`, `adat_rx_header_ok`, `adat_rx_decode_frame`, `adat_rx_rate_machine`, `adat_input_update_clock_servo`) is `DSP_TIME_CRITICAL` and RAM-resident (the acquisition probe/header scan stays cold in flash, since it runs only while muted). See "Memory Layout".

**Persistence.** ADAT input config persists like the optional SPDIF inputs (disabled by default, pin `0xFF` = unset, GPIO claimed only while ADAT is the active source). `WireInputConfig` (V24) gains `adat_input_pin`, `adat_input_enabled_p1` (enable + 1; 0 absent), and `adat_clock_mode_p1` (mode + 1; 0 absent, 1 master, 2 slave). The fields also ride `PresetSlot` (`SLOT_DATA_VERSION` 32) and the device-global `FlashOutputConfig` (`DIR_VERSION` 15), honoring `output_config_mode` exactly like the other physical-IO config. Vendor commands: `REQ_SET/GET_ADAT_INPUT_ENABLE` (0x68/0x69), `REQ_SET/GET_ADAT_INPUT_PIN` (0x6A/0x6B), `REQ_SET/GET_ADAT_INPUT_CLOCK_MODE` (0x6C/0x6D), `REQ_GET_ADAT_INPUT_STATUS` (0x6E); 0x6F reserved.

---

## Soft VCXO
*Last updated: 2026-07-22*

The single actuator behind input clock tracking (SPDIF input, ADAT slave-clock mode, I2S slave). When the device follows an external audio clock its output clocks must match the source's rate to keep the consumer buffers centred. Previously each output type's PIO divider (and MCK, and ADAT TX) was trimmed independently; that had coarse steps (one 16.8 fractional LSB is 156 ppm at 48 kHz), many actuators to coordinate, and left PDM untracked. The soft VCXO replaces all of it with one knob: it pulls the system PLL by ppm amounts, exactly like warping a crystal. Every output clock (SPDIF TX, internally clocked I2S TX, MCK, ADAT TX, PDM) derives from `sys_clk`, so they all move together and inter-slot sample alignment is preserved by construction; there is no per-output write to get wrong and no divider rounding to coordinate. Files: `soft_vcxo.c/h`. Full spec: `Documentation/Features/soft_vcxo_spec.md`. Status: RP2350 mechanism bench-proven standalone; full-firmware behavior HW-untested; RP2040 fbdiv dithering HW-untested.

**Mechanism.** `sys_clk` = 307.2 MHz (XOSC 12 MHz, refdiv 1, FBDIV 128 = VCO 1536 MHz, postdiv 5×1). The PLL has no fractional divider, so one is synthesized by pulse-density modulation of FBDIV: DMA briefly writes FBDIV = 127 or 129, then restores 128, at a controlled repetition rate. Each ±1 displacement injects a fixed, deterministic slice of VCO phase that the loop filter smooths; the average frequency offset is proportional to the pulse rate. The analytic constant is `SOFT_VCXO_HZ_PER_PPM = 262.144` (~1 ppm per 262.144 Hz of pulse rate); a standalone RP2350 bench measured the tune ratio within 1% of analytic, and since the constant sits inside the servo loop the residual gain error is absorbed by the fill terms (no calibration pass). The static resolution floor is ~0.11 ppm; below that the VCXO parks (FBDIV excursion set to the nominal 128 so the pulses become harmless no-op writes, loop never stopped).

**Topology (zero IRQs, zero steady-state CPU).** Two DMA channels, DMA timer 0, and one PWM slice used purely as a DREQ pacer with no GPIO driven. A PACE channel (DREQ = PWM wrap) re-points and triggers a PULSE channel (DREQ = DMA timer 0), which streams `{excursion, nominal}` into `pll_sys_hw->fbdiv_int` and chains back to PACE; mutual re-arming keeps the loop running forever (both platforms; deliberately avoids the RP2350-only ENDLESS trans-count mode so the code is identical on RP2040). `fbdivs[]` and its address pointer are non-const RAM statics so the pulse train keeps running through flash blackout windows (a DMA read from XIP during a flash op would stall).

**Command path.** `soft_vcxo_set_ppm(float ppm)` (RAM-resident, `DSP_TIME_CRITICAL`) clamps to ±`SOFT_VCXO_MAX_PPM` (300), parks below the floor, sets the FBDIV excursion sign (129 for ppm > 0, 127 for < 0), and programs the PWM pacer (smallest power-of-two clkdiv whose wrap fits 16 bits, `wrap = round(307.2e6 / clkdiv / freq)`). The authority clamp is ±300 ppm; the mechanism could reach ~1000 ppm by raising the clamp and max PWM rate if ever needed.

**Init and bus access.** `soft_vcxo_init()` (called near the end of `core0_init()`) claims the DMA channels / DMA timer / PWM slice and bootstraps the loop. It verifies `clock_get_hz(clk_sys) == 307200000` and stays inert otherwise (covers the RP2350 150 MHz boot fallback, where tracked inputs fall back to untracked nominal outputs). On RP2350 the DMA master cannot reach PLL_SYS by default (ACCESSCTRL resets it to core-only), so init writes `accessctrl_hw->pll_sys = 0xACCE0000 | 0xFF` to admit it; RP2040 has no ACCESSCTRL (APB is open to the DMA master). DMA-to-PLL writes on RP2040 are architecturally fine but not yet hardware-verified.

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
*Last updated: 2026-07-14 (preset_loading now floored to outlast a pending hold: prepare_pipeline_reset() sizes preset_mute_counter to hold_ms + margin when the hold has not yet elapsed, closing the long-hold expiry cases below. Previously, 2026-07-13: Phase 4 deferral extended to ADAT and SPDIF inputs 2/3; flash-write fade-settle loop services a locked ADAT input)*

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

`prepare_pipeline_reset()` arms the soft envelope (`preset_loading + preset_mute_counter`) then calls `dac_hw_mute_assert()`. Order: software mute state is visible before disruptive work begins; hardware mute second gives the DAC's analog stage time to ramp down before clocks stop. The two layers cover different failure modes — data-path discontinuity and analog DC-step on clock cessation.

**The hold is asynchronous — the main loop never busy-waits.** `dac_hw_mute_assert()` only arms a deadline; the hold is enforced at the *clock-stop boundary*, which splits into two cases:
- **Synchronous reset handlers** (preset load, factory reset, bulk params, rate change, stream restart, output-type switch, input-source switch) stop clocks in the same iteration they start. Each gates its body on `pipeline_reset_ready()` — a thin helper = `dac_hw_mute_assert()` + `dac_hw_mute_hold_elapsed()`. While the hold is incomplete the body is skipped and the pending flag is left set, so the loop falls through and keeps servicing audio; the handler retries next iteration. The idempotent assert (no hold re-arm) makes calling the gate every iteration safe. **The gate engages only the hardware mute, never `preset_loading`:** `preset_loading` also triggers the earlier SPDIF lock-acquisition block, and holding it true across the wait would make that block run `drain_and_disable_outputs()` on the same iteration the body re-enables outputs — a double `enable_outputs_in_sync()` with no teardown between, breaking slot alignment. The body's own `prepare_pipeline_reset()` sets `preset_loading` at the proper time (right before teardown), preserving the SPDIF block's original ordering (it reacts on the *next* iteration, after the body's `complete`).
- **SPDIF lock/prefill path** already defers `drain_and_disable_outputs()` (its clock-stop) to a post-lock iteration; it adds `&& dac_hw_mute_hold_elapsed()` to that block's condition so even an instant re-lock still honors the hold. (These Category-B sites assert via `prepare_pipeline_reset()` directly, so they do set `preset_loading` — the block's trigger.)

`perform_rate_change()` and `process_type_switches()` keep their internal `prepare_pipeline_reset()` calls; reached from an already-gated handler these are harmless idempotent re-engages, and their teardown runs after the hold. Flash writes (inherently ≈45 ms IRQ-off blocking) fold the hold into their existing fade-settle loop rather than gating: `prepare_flash_write_operation()` runs its settle loop until BOTH the fade-settle interval and `dac_hw_mute_hold_elapsed()` are satisfied, servicing the active input (USB ring drain, `spdif_input_poll()`, `i2s_input_poll()`, or `adat_input_poll()`) throughout so the output pools stay fed with muted samples. The loop is entered whenever the active source is live: USB `sync_started`, SPDIF LOCKED, I2S RUNNING, or ADAT LOCKED. (ADAT was originally omitted from this streaming test, the pre-mute drain, and the loop body, so with a locked ADAT input the settle loop was skipped and the blackout could begin before the hold elapsed and before any muted samples reached the outputs — exposing the clock-stop click on preset saves; fixed 2026-07-13.) The boot-time `process_type_switches()` (before the main loop, no audio) proceeds without a hold.

Previously the hold was a `time_us_64()` busy-wait inside `dac_hw_mute_assert()`; it stalled the main loop for up to `hold_ms`, starving the SPDIF in-to-out path and delaying boot-into-SPDIF and input-source switches. The async barrier removes that stall while preserving the exact teardown/synchronized-restart sequence (inter-slot phase alignment unchanged).

`complete_pipeline_reset()` adds a Phase 4: after `reset_usb_feedback_loop()`, calls `dac_hw_mute_release()`. Clocks restarted first (Phase 2), then release begins. With `release_ms == 0` (the default), the mute pin deasserts immediately. With `release_ms > 0`, the pin remains asserted until `dac_hw_mute_tick()` observes the deadline; the main loop keeps draining USB/SPDIF audio during the hold, so consumer buffers do not pile up behind a busy-wait.

**I2S, SPDIF, and ADAT inputs defer Phase 4 to their prefill blocks.** Phase 4 is skipped when `preset_loading` is set and the active source is I2S, any SPDIF input (`input_source_is_spdif()`, covering inputs 2/3), or ADAT — each of those sources has a main-loop prefill handshake (gated on `preset_loading`) that, after the reset returns, DRAINS and disables the outputs again, re-enables them in sync, and owns the `dac_hw_mute_release()` (right after its `enable_outputs_in_sync()`). Releasing in Phase 4 would un-mute before that drain stops the clocks; with the default `release_ms == 0` the pin deasserts immediately, fully exposing the clock-stop click. The prefill blocks always run while their source is active and `preset_loading` is set (SPDIF and ADAT hold the mute until lock, the intended mute-until-lock behavior), so the mute is never left stuck asserted. History: the guard originally listed only I2S and SPDIF input 1, so a `complete_pipeline_reset()` with ADAT or SPDIF 2/3 active and `preset_loading` set (full flash-write completion, output pin/type switch, preset or bulk apply) released the mute one iteration before the prefill block's `drain_and_disable_outputs()` stopped the clocks; fixed 2026-07-13 by matching the prefill gates (`input_source_is_spdif()` plus `INPUT_SOURCE_ADAT`, the latter defined on both platforms but never active on RP2040). The companion case is `perform_rate_change()`, which gains a `defer_output_to_input_prefill` argument: callers pass it for I2S bring-up, runtime I2S rate change, and the source-switch *into* I2S, ADAT, or SPDIF (where `active_input_source` is still the old source, so the Phase-4 guard alone could not catch it) to skip `complete_pipeline_reset()` entirely and let the target's prefill block own the restart. USB is unaffected (it has no such prefill drain and releases in Phase 4 as before).

**SPDIF lock-acquisition path also releases the mute.** The USB→SPDIF switch (and boot-into-SPDIF, and SPDIF rx-pin hot-swap, and SPDIF re-lock after lock loss) does NOT call `complete_pipeline_reset()` — output must stay muted until SPDIF achieves lock and the consumer pool prefills. The lock-acquisition flow in the main loop replicates the relevant phases (`drain_and_disable_outputs()` → wait for lock + prefill → `enable_outputs_in_sync()`), then calls `dac_hw_mute_release()` directly to mirror Phase 4. If `release_ms > 0`, the pin remains asserted while `spdif_input_poll()` continues feeding the output buffers; without this release path, the XSMT pin asserted by the earlier `prepare_pipeline_reset()` would stay asserted indefinitely and the DAC's analog stage would never un-mute.

**`preset_loading` is floored to outlast a pending hold.** The deferrals above all hang off `preset_loading`, but that flag is not a plain latch: the soft-mute envelope (`update_preset_mute_envelope()`, `audio_pipeline.c`) auto-clears it once `preset_mute_counter` samples have been processed, and samples keep processing while a deferred consumer of the flag waits out the hardware-mute hold (`hold_ms` may be up to `DAC_HW_MUTE_HOLD_MS_MAX` = 500 ms). Two arming points enforce the floor (2026-07-14), both gated on `!dac_hw_mute_hold_elapsed()` and sized at `hold_ms` (via the `dac_hw_mute_hold_ms()` getter) + `PRESET_MUTE_HOLD_MARGIN_MS` (120 ms, `flash_storage.h`):
- `prepare_pipeline_reset()` (`main.c`) — closes the re-lock window: the SPDIF/ADAT/I2S-slave RELOCKING handlers arm only `PRESET_MUTE_SAMPLES` = 256 (~5 ms), and after a fast re-lock (or a pin-swap input restart) the source's poll keeps feeding the pipeline while the prefill drain waits on `dac_hw_mute_hold_elapsed()`; with a long hold the flag expired before the drain could run, the prefill handshake never fired, and `dac_hw_mute_release()` was never called (stuck mute). The floor engages only on a fresh hold; the synchronous reset handlers pre-gate on `pipeline_reset_ready()`, so their hold has already elapsed when their body runs prepare, and USB-path mute durations are unchanged.
- `flash_mute_hold_samples()` (`flash_storage.c`) — closes the non-streaming flash window: the flash brackets' fade-settle loop waits the hold out before the blackout, but only when the source is streaming; a flash write with the source dark leaves the hold pending at completion, and `flash_write_sector()`'s trailing re-arm (the last writer of `preset_mute_counter` before completion, ~10 ms un-floored) would let a subsequent fast lock burn the flag before the prefill block could run. The `preset_delete()` active-slot re-arm routes through the same helper. Note the streaming flash path never depended on the prepare-time floor: the settle loop's hold-wait plus the per-write re-arm already guarantee `preset_loading` is set with the hold elapsed at completion.

The flash blackout itself costs nothing against the counter (IRQs off, no samples processed).

**Flash-write completion paths release the mute too.** Both `complete_flash_write_operation_full()` (preset save/delete) and `complete_flash_write_operation_light()` (metadata-only writes: preset rename, startup policy, output-config mode/save, master-volume mode/save, DAC-mute config) assert the mute via `prepare_flash_write_operation()` → `prepare_pipeline_reset()` and must release it. Both follow the same source split, whose single canonical home is the `release_hw_mute_if_outputs_live()` helper (`main.c`): for **USB input** the mute deasserts now (full path implicitly inside `complete_pipeline_reset()`; light path via `release_hw_mute_if_outputs_live()`, since the light path never tore down outputs, so PIO clocks ran continuously through the blackout and the deassert is clock-safe); for **SPDIF input** both skip the release and let the lock-acquisition prefill path own it after RX re-locks; for **I2S input** both likewise skip it — the light/metadata path stops the input and leaves `preset_loading` set, so the main-loop I2S prefill block then drains, re-enables outputs in sync, and owns the release exactly like SPDIF (deasserting in the completion would un-mute BEFORE that drain stops the output clocks, exposing the clock-stop click the mute suppresses); for **ADAT input** the full path does call `complete_pipeline_reset()` (ADAT is deliberately not stopped for the blackout; its IRQ-less RX ring survives it), but Phase 4's deferral covers it, so the ADAT lock/prefill block owns the release there too. `release_hw_mute_if_outputs_live()` therefore releases for **USB input only**. Omitting the light-path release would leave the DAC silent indefinitely after a metadata-only write while EMC is enabled on USB input — the helper exists partly to keep that easy-to-forget release (the bug fixed in git 833a51a) in one named, documented place; the full path's SPDIF skip and the lock-acquisition release are the other two expressions of the same rule and cross-reference it.

### Mute-carry invariant
*Last updated: 2026-07-04 (fix: config re-apply un-muted the DAC mid-flash-write, causing full-volume noise on hold/release-timer edits while streaming)*

`dac_hw_mute_set_config()` is invoked from *inside* the flash-write brackets: the deferred `REQ_SET_DAC_HW_MUTE_CONFIG` handler (`main.c`) and the bulk `REQ_SET_ALL_PARAMS` apply (`bulk_params_apply()` → `dac_hw_mute_set_config()`) both run `prepare_flash_write_operation()` / `prepare_pipeline_reset()` first — which has already **asserted** the mute and waited out `hold_ms` — and only then apply the new config, whose `preset_set_dac_hw_mute()` → `dir_flush()` runs a ~45 ms IRQ-off flash blackout. Historically `dac_hw_mute_init()` unconditionally released the pin and re-drove it to the *un-muted* level (and cleared `s_lifecycle_asserted`) on every re-apply, so the DAC un-muted for that blackout; with audio streaming, the output producer pool underruns during the IRQ-off window and the SPDIF/I²S DMA loops stale buffer content, which the now-live DAC plays as full-volume noise. The trailing `dac_hw_mute_release()` was then a no-op (latch already cleared). Symptoms: noise only while music plays (pool has content to loop), only on DAC-mute-config or bulk edits (other directory setters don't call `dac_hw_mute_init()`), gone after closing/reopening the console (no re-apply). Fix: `dac_hw_mute_init()` carries the asserted state across the re-apply (see the module bullet above), so the DAC stays muted through the config change and its flash write, and the completion path (`complete_pipeline_reset()` Phase 4 / `complete_flash_write_operation_*()`) owns the release exactly as for any other reset.

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
