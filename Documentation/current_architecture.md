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
19. [Memory Layout](#memory-layout)
20. [Performance Characteristics](#performance-characteristics)

---

## System Overview
*Last updated: 2026-02-15*

DSPi is a USB Audio Class 1 (UAC1) digital signal processor built on the Raspberry Pi Pico (RP2040) and Pico 2 (RP2350). It receives stereo PCM audio over USB and routes it through a configurable DSP pipeline to multiple output channels via PIO-based S/PDIF and PDM.

- **RP2350:** 9 output channels — 4 S/PDIF stereo pairs (8 channels) + 1 PDM sub
- **RP2040:** 5 output channels — 2 S/PDIF stereo pairs (4 channels) + 1 PDM sub

**Key capabilities:**
- Per-channel input preamp (independent gain per USB input channel)
- Parametric EQ: 11 channels on RP2350 (2 master + 9 outputs), 7 channels on RP2040 (2 master + 5 outputs)
- Matrix mixer with per-output gain, mute, phase invert, and delay
- Master volume: device-side attenuation-only ceiling on all outputs (post-output-gain)
- BS2B crossfeed for headphone listening
- ISO 226:2003 loudness compensation
- Per-output configurable delay lines
- Runtime pin reconfiguration
- Full parameter persistence to flash
- Vendor control interface (WinUSB/WCID) for real-time parameter control

**Firmware binary:** `copy_to_ram` — entire firmware executes from SRAM for deterministic latency.

---

## Source File Map
*Last updated: 2026-04-18*

### Core Firmware (`firmware/DSPi/`)

| File | Purpose |
|------|---------|
| `main.c` | Entry point, initialization, main event loop |
| `usb_audio.c` | USB audio input decode (`process_audio_packet`), custom UAC1 TinyUSB class driver (`uac1_driver_*`), output slots, volume, init |
| `usb_audio.h` | USB audio interface API, AudioState struct, extern declarations for shared state |
| `tusb_config.h` | TinyUSB configuration (disables built-in classes; UAC1 handled by custom driver) |
| `audio_pipeline.c` | Input-agnostic DSP pipeline (`process_input_block`): loudness, EQ, leveller, crossfeed, matrix mixer, per-output EQ/gain/delay, output encoding, buffer stats |
| `audio_pipeline.h` | Pipeline entry point, shared buffer declarations (`buf_l`/`buf_r`/`buf_out`), buffer stats API |
| `vendor_commands.c` | Vendor USB control request handlers (GET/SET dispatch, pin/MCK helpers, diagnostics). Public entry `vendor_control_xfer_cb(rhport, stage, req)` is invoked from `usb_audio.c`'s UAC1 class driver. |
| `vendor_commands.h` | Vendor handler declarations, system stats and pin helper prototypes. |
| `dsp_pipeline.c` | Biquad coefficient computation, filter management |
| `dsp_pipeline.h` | Filter storage declarations, delay line API |
| `dsp_process_rp2040.S` | RP2040-only: hand-optimized ARM assembly biquad (per-sample + block-based) |
| `pdm_generator.c` | 2nd-order sigma-delta PDM modulator, Core 1 PDM mode |
| `pdm_generator.h` | PDM API, ring buffer communication |
| `crossfeed.c` | BS2B crossfeed filter (lowpass + allpass for ILD/ITD) |
| `crossfeed.h` | Crossfeed API, presets, state structs |
| `loudness.c` | ISO 226:2003 loudness curve computation, double-buffered tables |
| `loudness.h` | Loudness API, coefficient structs |
| `leveller.c` | Volume leveller (feedforward RMS compressor) |
| `leveller.h` | Volume leveller API, state/config structs |
| `flash_storage.c` | Parameter save/load to last 4KB flash sector |
| `flash_storage.h` | Flash storage API |
| `bulk_params.c` | Bulk parameter collect/apply (wire format ↔ live state) |
| `bulk_params.h` | Wire format structs (`WireBulkParams`), buffer size defines |
| `config.h` | Global config, data structures, vendor command IDs, channel defs |
| `usb_descriptors.c` | Hand-rolled UAC1 configuration descriptor as a packed byte array + TinyUSB `tud_descriptor_*_cb()` callbacks |
| `usb_descriptors.h` | Descriptor declarations, UAC1 request opcodes, per-alt EP descriptor pointer externs |
| `dcp_inline.h` | RP2350 DCP (Double Coprocessor) inline assembly wrappers |

### LUFA Compatibility (`firmware/DSPi/lufa/`)

**Not on Phase 1 include path.** Folder retained on disk for Phase 2 reference; the UAC1 descriptor is now hand-rolled in `usb_descriptors.c` against TinyUSB's `AUDIO_*` constants.

### SPDIF Library (`firmware/pico-extras/src/rp2_common/pico_audio_spdif_multi/`)

Multi-instance S/PDIF output library (PIO-based, converted from pico-extras singleton).

---

## Build System
*Last updated: 2026-04-18*

### CMake Configuration

**Build file:** `firmware/DSPi/CMakeLists.txt`

**Binary type:** `copy_to_ram` (entire firmware in SRAM)

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

**Vendor commands** were temporarily excluded in Phase 1 and re-added in Phase 2; `vendor_commands.c` / `vendor_commands.h` are now back in `add_executable()`.

**Build commands:**
```bash
cmake --build build-rp2040 --clean-first   # RP2040 build
cmake --build build-rp2350 --clean-first   # RP2350 build
```

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

### Sample-rate & Bit-depth Switching
*Last updated: 2026-04-18*

Any host-driven format change — SET_INTERFACE between AS alts (bit-depth switch) or SET_CUR on the endpoint sampling_freq control (rate switch) — must land on a muted, drained pipeline. Otherwise old-rate/old-bit-depth audio still queued in the consumer pools plays out against the new PIO divider or gets decoded under the wrong bytes-per-frame assumption, producing an audible pitch shift or byte-scramble burst.

**`uac1_apply_alt()` (usb_audio.c):**
- **Idempotent early-return.** `SET_INTERFACE(alt=current)` is common from host driver probes and used to tear down / re-open iso endpoints for no reason. Now skipped.
- **Bit-depth switch (alt 1↔2) is treated the same as a cold start (alt 0→>0).** Both paths engage the mute envelope inline (`preset_mute_counter = PRESET_MUTE_SAMPLES`, `preset_loading = true`) so any packet decoded between the SETUP ack and the main-loop's `complete_pipeline_reset()` is silenced. Both paths also raise `stream_restart_resync_pending`, reset the feedback controller (`fb_ctrl_stream_stop` + `feedback_10_14 = nominal_feedback_10_14`), and clear `sync_started` / `total_samples_produced` so gap detection and the feedback loop resume from a deterministic baseline.
- The pre-existing ring flush on `bit_depth_changed` still runs; combined with the mute, stale packets cannot reach the DSP pipeline in the wrong format.

**SET_CUR sampling_freq validation:** unsupported rates are now stalled at EP0 rather than silently committed. Previously any 24-bit value was accepted — `audio_state.freq` would store garbage that `perform_rate_change()` later coerced to 44100, so a subsequent GET_CUR returned a rate the device never actually applied. The accepted set is {44100, 48000, 96000} to match the Type-I format descriptors on both alts.

**`perform_rate_change()` (main.c):** bracketed with `prepare_pipeline_reset(PRESET_MUTE_SAMPLES)` / `complete_pipeline_reset()`. Without the bracket, the SPDIF `wrap_consumer_take` callback updates the PIO divider lazily on the next buffer-take, so old-rate audio already queued in each consumer pool plays out at the new bit-clock — audible pitch wobble for ~16 ms. `complete_pipeline_reset()` aborts DMA on every enabled slot, drains the consumer pool back to the free list, and restarts all outputs in sync at the new divider. The I2S `audio_i2s_update_all_frequencies()` call inside `perform_rate_change()` still runs for its own divider+clkdiv_restart pass; the subsequent `complete_pipeline_reset()` re-aborts/re-enables idempotently and costs only microseconds.

### Notification Endpoint (device→host push)
*Last updated: 2026-04-19*

The vendor interface carries one **bulk IN** endpoint (EP 0x83, wMaxPacketSize = 64) for out-of-band device→host notifications. The transport runs two protocol versions in parallel: v1 (8-byte `MASTER_VOLUME` packets, kept for existing host apps) and v2 (generic `PARAM_CHANGED` + discrete events, the primary protocol going forward). `USB_BCD_DEVICE = 0x0201` so Windows re-reads descriptors after the 8→64 byte EP bump.

See `Documentation/Features/notification_protocol_v2_spec.md` for the full protocol specification.

**Why bulk rather than interrupt:** an earlier draft used an interrupt IN endpoint at 4 ms polling. Under heavy EP0 control-transfer traffic (rapid `REQ_SET_MASTER_VOLUME` from a slider drag in the host app) the RP2040/2350 USB controller crashed after ~20–40 s and the device re-enumerated. Switching the EP to bulk IN eliminates the crash: bulk uses opportunistic host scheduling rather than a fixed bInterval poll cadence. The v2 protocol preserves the bulk transport.

**v2 core design (`notify.c/notify.h`):** every parameter is identified by its `offsetof` into `WireBulkParams`. A single event ID (`NOTIFY_EVT_PARAM_CHANGED = 0x02`) carries `(wire_offset, wire_size, source, value)`. Host dispatch is a flat lookup on offset, not a hand-written switch — adding a parameter requires zero wire-format changes.

**Subsystem state:**
- `param_shadow`: mirror of `WireBulkParams` (2912 B BSS). `notify_param_write` compares writes against it; notifications only fire on real byte-level changes.
- `notify_ring[32]`: SPSC ring of pending events (1920 B BSS). Coalesces PARAM_CHANGED entries on `(event_id, offset, size)` — a swept knob generates one queued entry, not hundreds.
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

**Emit hookpoints:** `update_master_volume` emits both v1 (`notify_push_master_volume_v1`) and v2 (`notify_param_write`). `update_preamp` emits v2. Direct-write setters in `vendor_commands.c` (delays, gain/mute, loudness, crossfeed, leveller, matrix, pins, I2S, MCK, SPDIF RX pin, channel names) each call `notify_param_write` after the live-state write. Deferred setters (EQ band, input source) emit at apply time in `main.c`.

**Bulk operations** (preset load, factory reset, bulk SET): wrapped in `notify_begin_bulk(source)` / `notify_end_bulk()`. Per-field writes don't flood the ring; the host sees one `BULK_INVALIDATED` and reads `REQ_GET_ALL_PARAMS` for the full state. Preset load also emits `NOTIFY_EVT_PRESET_LOADED(slot)` before the bulk opens.

**Drain:** `usb_notify_drain` (usb_audio.c) claims EP 0x83 via `usbd_edpt_claim`, calls `notify_peek_next` to format the next packet into the stable TX buffer, and submits via `usbd_edpt_xfer`. On success, `notify_commit_pop` advances the ring tail. On xfer rejection, the entry stays queued; the next tick retries.

**Initialisation:** `notify_init()` is called from `core0_init()` after `preset_boot_load()` so `bulk_params_collect(&param_shadow)` sees a fully-populated live state. The USB reset path (`uac1_driver_reset`) calls `notify_reset_queue()` to drop stale events.

**v1 back-compat:** `update_master_volume` still pushes an 8-byte `MASTER_VOLUME` (0x01) event into the ring. Existing v1 host apps that only recognise byte 0 = 0x01 continue to work; v2 hosts receive the parallel PARAM_CHANGED event and dispatch by offset.

### Volume & Mute

**Volume range:** -60 dB to 0 dB (1 dB resolution, 61 steps). Bottom step is fully silent. USB Audio Class 8.8 fixed-point dB encoding. Q15 lookup table (`db_to_vol[61]`) maps dB index to linear multiplier; index 0 = 0x0000 (silent), index 60 = 0x7FFF (unity).

**Mute:** UAC1 Feature Unit MUTE control. When `audio_state.mute` is set, `vol_mul` is forced to zero in the audio callback (RP2350: 0.0f, RP2040: 0), silencing all outputs immediately.

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

**Deferred flash SET commands:** `REQ_PRESET_SET_NAME`, `REQ_PRESET_SET_STARTUP`, `REQ_PRESET_SET_INCLUDE_PINS` use separate pending flags per command type. Main loop copies payload under brief interrupt-off (~1µs) to prevent ISR/thread race, then drains ring and executes the flash write. GET-style flash commands (SAVE/LOAD/DELETE) remain synchronous in the vendor handler with real result codes.

### Packet Flow
*Last updated: 2026-04-18*

`uac1_driver_xfer_cb(EP 0x01)` → `usb_audio_ring_push()` → (main loop) → `usb_audio_drain_ring()` → `process_audio_packet(data, len)` [usb_audio.c] → `process_input_block(sample_count)` [audio_pipeline.c]

1. **Ring push (task context)** — Copy raw packet into SPSC ring, detect arrival gaps
2. **Ring drain (main loop)** — Peek/process/consume loop, highest priority in main loop
3. **USB decode (`process_audio_packet` in `usb_audio.c`)** — Gap detection, sync tracking, USB byte decode (16/24-bit) with per-channel preamp into `buf_l[]`/`buf_r[]`
4. **DSP pipeline (`process_input_block` in `audio_pipeline.c`)** — Input-agnostic: buffer acquisition, preset mute envelope, loudness, EQ, leveller, crossfeed, matrix mixer, per-output EQ/gain/delay, output encoding, buffer return, CPU metering
5. **Buffer return** — Give completed buffers to consumer pools for DMA

The `process_input_block()` function reads from `buf_l[]`/`buf_r[]` arrays (extern, defined in `audio_pipeline.c`, filled by the input decode stage). This separation enables future alternative input sources (S/PDIF, I2S) to fill the same buffers and call `process_input_block()` directly. Buffer statistics helpers (`get_slot_consumer_fill()`, `get_slot_consumer_stats()`, `reset_buffer_watermarks()`) also live in `audio_pipeline.c`.

### RP2350 Float Pipeline
*Last updated: 2026-04-11*

All processing in IEEE 754 single-precision float. Hybrid SVF/biquad EQ filtering (SVF for bands below Fs/7.5, TDF2 biquad above).

| Stage | Description |
|-------|-------------|
| Input conversion | int16 → float, per-channel preamp gain (`global_preamp_mul[ch]`) |
| Loudness | 2 SVF shelf filters (low shelf + high shelf), volume-dependent |
| Master EQ | Block-based `dsp_process_channel_block()`, 10 bands per channel, hybrid SVF/biquad |
| Volume Leveller | Upward RMS compressor on master L/R with gain-reduction limiter (float throughout) |
| Crossfeed | BS2B lowpass + allpass (ILD + ITD) |
| Matrix mixing | Block-based: 2 inputs × 9 outputs with gain/phase |
| Output EQ | Block-based, 10 bands per output (Core 0: outputs 0-1, Core 1: outputs 2-7) |
| Output gain | Per-output gain × host volume × master volume |
| Delay | Float circular buffers, 2048 samples max (42ms at 48kHz) |
| SPDIF output | Float → int16 conversion, 4 stereo pairs |
| PDM output | Float → Q28 for sigma-delta modulation |

### RP2040 Fixed-Point Pipeline
*Last updated: 2026-04-11*

Block-based two-phase architecture with dual-core EQ processing, all in Q28 fixed-point (28 fractional bits). 2 S/PDIF stereo pairs + 1 PDM sub (5 output channels).

**Phase 1 (Core 0, block-based where possible):**

| Stage | Description |
|-------|-------------|
| Input conversion | int16 → Q28 (shift left 14), per-channel preamp via `fast_mul_q28()` (`global_preamp_mul[ch]`) — block loop to `buf_l[192]`, `buf_r[192]` |
| Loudness | 2 biquads per-sample via `fast_mul_q28()` (Q28 coefficients, state coupling) |
| Master EQ | **Block-based** `dsp_process_channel_block()`, 10 bands per channel |
| Volume Leveller | Upward RMS compressor on master L/R with gain-reduction limiter (Q28 envelope + float gain) |
| Crossfeed | BS2B per-sample via `fast_mul_q28()` (Q28 coefficients, stereo coupling) |
| Matrix mixing | Q15 gains via `fast_mul_q15()` (16-bit partial products), 2 inputs × 5 outputs → `buf_out[5][192]` |

**Phase 2 (per-output block, dual-core or single-core):**

| Stage | Description |
|-------|-------------|
| Output EQ | **Block-based** `dsp_process_channel_block()`, 10 bands per output |
| Output gain + volume | Combined Q15 multiply via `fast_mul_q15()` (output gain × host volume × master volume) |
| Delay | int32 circular buffers, 2048 samples max (42ms at 48kHz) |
| SPDIF output | Q28 → int16 (shift right 14 with rounding), 2 stereo pairs |
| PDM output | Q28 direct to sigma-delta modulator (single-core fallback only) |

**Dual-core mode:** Core 0 handles input pipeline + matrix mix + SPDIF pair 1 (outputs 0-1), Core 1 handles SPDIF pair 2 (outputs 2-3) — both cores process per-output EQ, gain, delay, and S/PDIF conversion in parallel. PDM sub (output 4) runs on Core 1 in PDM mode; PDM and EQ worker outputs (2-3) are mutually exclusive.

**Performance advantage of block-based processing:** Biquad coefficients are loaded once per biquad per block instead of once per sample. For a 2-band output channel with 192-sample blocks, this reduces coefficient loads from 384 to 2.

---

## DSP Processing Engine
*Last updated: 2026-04-11*

### Channel Layout

**RP2350 (11 channels):**

| Channel | Index | Description |
|---------|-------|-------------|
| CH_MASTER_LEFT | 0 | Master EQ left |
| CH_MASTER_RIGHT | 1 | Master EQ right |
| CH_OUT_1 | 2 | S/PDIF 1 Left |
| CH_OUT_2 | 3 | S/PDIF 1 Right |
| CH_OUT_3 | 4 | S/PDIF 2 Left |
| CH_OUT_4 | 5 | S/PDIF 2 Right |
| CH_OUT_5 | 6 | S/PDIF 3 Left |
| CH_OUT_6 | 7 | S/PDIF 3 Right |
| CH_OUT_7 | 8 | S/PDIF 4 Left |
| CH_OUT_8 | 9 | S/PDIF 4 Right |
| CH_OUT_9_PDM | 10 | PDM Subwoofer |

**RP2040 (7 channels):**

| Channel | Index | Description |
|---------|-------|-------------|
| CH_MASTER_LEFT | 0 | Master EQ left |
| CH_MASTER_RIGHT | 1 | Master EQ right |
| CH_OUT_1 | 2 | S/PDIF 1 Left |
| CH_OUT_2 | 3 | S/PDIF 1 Right |
| CH_OUT_3 | 4 | S/PDIF 2 Left |
| CH_OUT_4 | 5 | S/PDIF 2 Right |
| CH_OUT_5_PDM | 6 | PDM Subwoofer |

### Biquad Filter

**Types:** Flat (bypass), Peaking, Low Shelf, High Shelf, Low Pass, High Pass

**Coefficient computation:** RBJ Audio-EQ-Cookbook formulas for biquad path, Cytomic SVF equations for SVF path (RP2350 only), both in `dsp_compute_coefficients()`

**RP2350 biquad (hybrid SVF/biquad):**
```c
{ float b0, b1, b2, a1, a2; float s1, s2;
  float sva1, sva2, sva3; float svm0, svm1, svm2;
  float svic1eq, svic2eq; uint32_t svf_type;
  bool use_svf; bool bypass; }
```
Single-precision throughout. Per-band SVF or TDF2 biquad path selected at coefficient computation time. See [Hybrid SVF/Biquad Filtering](#hybrid-svfbiquad-filtering-rp2350) for details.

**RP2040 biquad:**
```c
{ int32_t b0, b1, b2, a1, a2, s1, s2; bool bypass; }
```
Q28 fixed-point. Both per-sample and block-based biquad processing implemented in hand-optimized ARM assembly (`dsp_process_rp2040.S`). Block-based `dsp_process_channel_block()` keeps s1/s2 state in high registers across the entire sample loop, shares operand decompositions across multiply groups, and uses r12 for intermediate saves — eliminating per-sample struct access, function call overhead, and redundant decompositions vs the C `fast_mul_q28()` version.

### Hybrid SVF/Biquad Filtering (RP2350)
*Last updated: 2026-03-02*

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

**State reset:** When a band crosses the SVF/biquad boundary (e.g. due to sample rate change), both biquad state (`s1`, `s2`) and SVF state (`svic1eq`, `svic2eq`) are reset to zero to prevent transients.

**Input validation:** Frequency clamped to [10 Hz, 0.45×Fs], Q clamped to [0.1, 20].

**FPU configuration (RP2350):** Both cores set FPSCR flush-to-zero (FZ) and default-NaN (DN) bits at startup. This prevents denormalized floats from causing performance penalties as SVF integrator and biquad states decay toward zero after silence.

**Memory impact:** Biquad struct grows from ~48 to ~68 bytes on RP2350. With 114 total biquads (110 EQ + 4 loudness): ~3 KB additional BSS.

**RP2040:** Completely unaffected. All SVF code is inside `#if PICO_RP2350` blocks.

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

## Matrix Mixer
*Last updated: 2026-02-15*

### Architecture

2 inputs (USB L/R) × NUM_OUTPUT_CHANNELS outputs (5 on RP2040, 9 on RP2350) with per-crosspoint control:

```c
typedef struct {
    MatrixCrosspoint crosspoints[2][NUM_OUTPUT_CHANNELS];
    OutputChannel outputs[NUM_OUTPUT_CHANNELS];
} MatrixMixer;
```

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

The USB input supports both 16-bit and 24-bit PCM via two alternate settings on the Audio Streaming interface. The host OS selects the desired bit depth; a runtime variable (`usb_input_bit_depth`) tracks the active format and branches the input conversion accordingly. With 24-bit input and 24-bit SPDIF output, the full precision signal path is maintained end-to-end. The DSP pipeline operates at >16-bit precision internally (float on RP2350, Q28 fixed-point on RP2040).

**Alt-change safety (2026-04-18):** With `TUP_DCD_EDPT_ISO_ALLOC` enabled on RP2040/RP2350, TinyUSB's `usbd_edpt_close()` is a no-op — it does not clear the `busy` flag left by the previous alt's in-flight iso xfer. On the next `usbd_edpt_xfer()` this trips `TU_ASSERT(busy == 0)` and crashes the device. `uac1_open_stream_eps()` therefore calls `usbd_edpt_clear_stall()` on both stream endpoints after `usbd_edpt_iso_activate()` to force-clear the stale busy bit (the same workaround TinyUSB's stock audio class driver applies at `audio_device.c:1871`). `uac1_apply_alt()` also flushes the USB audio ring whenever `usb_input_bit_depth` changes so queued pre-switch packets aren't re-decoded under the new bytes/frame assumption. See "Sample-rate & Bit-depth Switching" for the full mute/resync flow that brackets every format change.

**Input conversion (24-bit):**
- **RP2350:** 3-byte little-endian → sign-extended int32 → float via `÷ 8388608.0f`
- **RP2040:** 3-byte little-endian → Q28 via left-justify and `>> 2` (net `<< 6`); same full-scale as 16-bit (`<< 14`)

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
*Last updated: 2026-02-14*

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

---

## Volume Leveller
*Last updated: 2026-04-04*

### Purpose

Automatic volume levelling via a feedforward, stereo-linked, single-band RMS compressor applied to the master L/R channels. Sits in the signal chain after Master EQ and before Crossfeed (PASS 2.5 in the pipeline).

### Algorithm

- **Topology:** Feedforward upward compressor with soft knee — boosts content below the threshold, leaves content above the threshold completely untouched (no makeup gain needed)
- **Stereo linking:** Stereo-linked — RMS envelope is computed from the louder of L/R channels, and the same gain is applied to both channels to preserve the stereo image
- **Envelope:** Asymmetric attack/release smoothing on the RMS envelope
- **Lookahead:** Optional 10ms lookahead delay buffer (less critical with upward compression since loud content receives 0 dB gain, reducing overshoot risk)
- **Gain computation:** Upward compression curve: content below threshold is boosted by `(threshold - x_db) * (1 - 1/ratio)`, content above threshold + knee/2 passes at unity (0 dB gain), with soft knee transition between
- **Limiter:** Gain-reduction style at -6 dBFS ceiling (instant attack, 100ms release) — computes gain reduction rather than hard clipping, rarely engages since loud content is untouched
- **Gate:** User-configurable silence gate prevents noise amplification when input is below the gate threshold

### Parameters

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| enabled | bool | 0/1 | false | Enable/disable the leveller |
| amount | float | 0.0–100.0 | 50.0 | Compression strength % (ratio = 1 + amount/100 * 19) |
| speed | uint8_t | 0/1/2 | 0 (Slow) | Envelope speed: 0 = Slow, 1 = Medium, 2 = Fast |
| max_gain_db | float | 0.0–35.0 | 15.0 | Maximum boost gain in dB |
| lookahead | bool | 0/1 | true | Enable 10ms lookahead delay buffer |
| gate_threshold_db | float | -96.0–0.0 | -96.0 | Silence gate level in dBFS (below = no boost) |

### Signal Chain Position

```
USB Input → Per-Ch Preamp → Loudness → Master EQ → Volume Leveller → Crossfeed → Matrix Mix → Output EQ → Output Gain × Host Vol × Master Vol → Delay → Output
                                                     (PASS 2.5)
```

### Platform Implementation

- **RP2350:** Float throughout — RMS envelope, gain computation, and gain application all in single-precision float
- **RP2040:** Q28 fixed-point for the RMS envelope accumulator, float for gain computation and smoothing. Gain applied to Q28 audio samples

### Files

| File | Purpose |
|------|---------|
| `leveller.c` | RMS envelope tracking, gain computation, soft-knee curve, lookahead buffer |
| `leveller.h` | Public API, state struct, configuration struct |

### Vendor Commands (0xB4–0xBF)

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
| 0xBC | REQ_SET_LEVELLER_LOOKAHEAD | OUT | Enable/disable 10ms lookahead |
| 0xBD | REQ_GET_LEVELLER_LOOKAHEAD | IN | Get lookahead state |
| 0xBE | REQ_SET_LEVELLER_GATE | OUT | Set silence gate threshold (-96.0–0.0 dBFS, float) |
| 0xBF | REQ_GET_LEVELLER_GATE | IN | Get silence gate threshold |

---

## Loudness Compensation
*Last updated: 2026-03-02*

### Purpose

ISO 226:2003 equal-loudness contour compensation to maintain perceived frequency balance at low listening volumes.

### Filter Architecture

2 shelf filters per stereo channel:
1. Low shelf (200 Hz, Q=0.707) — bass boost at low volume
2. High shelf (6000 Hz, Q=0.707) — treble boost at low volume

**RP2350:** SVF shelf filters (Cytomic "SvfLinearTrapAllOutputs" with `k = 1/Q` for exact RBJ matching). Both loudness filters are always below the SVF crossover frequency at all supported sample rates, so SVF is used unconditionally. Coefficients are SVF integrator + mix coefficients (`sva1-3`, `svm0-2`); state is minimal `LoudnessSvfState` (`ic1eq`, `ic2eq`).

**RP2040:** Q28 fixed-point RBJ biquad coefficients with `fast_mul_q28()` processing.

### Parameters

- **Reference SPL:** 40-100 dB (default 83 dB)
- **Intensity:** 0-200% (default 100%)

### Table Architecture

Double-buffered for glitch-free updates:
```c
LoudnessCoeffs loudness_tables[2][61][2];  // [buffer][volume_step][biquad]
```

- 61 volume steps: -60 dB to 0 dB (1 dB increments), index 0 = silent
- Background computation writes inactive buffer, atomic pointer swap activates

---

## Flash Storage
*Last updated: 2026-04-09*

### Flash Operation Safety

Flash erase/program requires quiescing XIP (execute-in-place). `flash_write_sector()` and `preset_delete()` use a guarded `multicore_lockout` to safely park Core 1 in RAM during the operation:

- **Guard condition:** `multicore_lockout_victim_is_initialized(1) && (__get_current_exception() == 0)`. The SDK function handles first-boot (Core 1 not launched) and launch-to-init race windows. The exception check skips lockout from IRQ context (USB vendor handler), where SDK lock internals are unsafe. IRQ callers rely on the `copy_to_ram` build for XIP safety.
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

### Preset Directory Fields

| Field | Description |
|-------|-------------|
| startup_mode | 0 = load specified default, 1 = load last active |
| default_slot | Slot to load in "specified default" mode (0-9) |
| last_active_slot | Last slot loaded/saved (always 0-9) |
| include_pins | Whether preset load/save includes pin config (0/1, default 0) |
| slot_occupied | 16-bit bitmask (bit N = slot N has valid data) |
| master_volume_mode | 0 = independent (default, mode 0 saved-to-directory), 1 = with preset (was include_master_volume) |
| spdif_rx_pin | Device-level SPDIF RX GPIO pin |
| master_volume_db | Independent master volume (mode 0 source); float, default -20 dB |
| slot_names[10][32] | 32-byte NUL-terminated names per slot |

### Preset Slot Data (Version 12)
*Last updated: 2026-04-09*

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
| Crossfeed | enabled, preset, ITD, custom fc/feed |
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
- `REQ_LOAD_PARAMS` (0x52): reloads the active preset slot
- `REQ_FACTORY_RESET` (0x53): resets live state to defaults, active slot unchanged

### Preset-Switch Mute & Pipeline Reset
*Last updated: 2026-04-01*

All preset operations (load, save, delete) are **deferred from the USB IRQ to the main loop** via pending flags (`preset_load_pending`, `preset_save_pending`, `preset_delete_pending` in `usb_audio.c`). This avoids running flash operations inside the USB ISR (which would cause a ~45ms interrupt blackout inside an interrupt handler) and allows proper pipeline reset bracketing.

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

---

## Pin Configuration
*Last updated: 2026-02-15*

### Default Assignments

**RP2350:**

| GPIO | Function | Output |
|------|----------|--------|
| 6 | S/PDIF 1 | Outputs 1-2 |
| 7 | S/PDIF 2 | Outputs 3-4 |
| 8 | S/PDIF 3 | Outputs 5-6 |
| 9 | S/PDIF 4 | Outputs 7-8 |
| 10 | PDM Sub | Output 9 |
| 12 | UART TX | Debug |
| 25 | LED | Heartbeat |

**RP2040:**

| GPIO | Function | Output |
|------|----------|--------|
| 6 | S/PDIF 1 | Outputs 1-2 |
| 7 | S/PDIF 2 | Outputs 3-4 |
| 10 | PDM Sub | Output 5 |
| 12 | UART TX | Debug |
| 25 | LED | Heartbeat |

### Runtime Reconfiguration
*Last updated: 2026-02-15*

Vendor commands `REQ_SET_OUTPUT_PIN` (0x7C) / `REQ_GET_OUTPUT_PIN` (0x7D).

**Constraints:** Valid GPIO range, not in use by another output, not reserved (12, 23-25).

**S/PDIF:** Can change while enabled — live pin swap via disable → `audio_spdif_change_pin()` → re-enable. The change_pin function masks DMA IRQ generation for the channel before aborting the DMA, preventing the high-priority DMA IRQ handler from starting a new transfer that would conflict with the PIO SM reinitialization. Any stale DMA completion flag is cleared before unmasking.

**PDM:** Must be disabled first, rebuilds PIO config.

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

Core 0 processes input pipeline + matrix mix, then dispatches per-output work to Core 1.
Core 1 processes assigned SPDIF outputs in parallel: EQ, gain, delay, and S/PDIF conversion.

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
    float vol_mul;
    int16_t *spdif_out[3];
#else
    int32_t (*buf_out)[192];
    int32_t vol_mul;          // Q15 master volume
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
*Last updated: 2026-04-28*

### Hardware

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| CPU | Dual Cortex-M0+ @ 133 MHz (OC to 307.2 MHz) | Dual Cortex-M33 @ 150 MHz (OC to 307.2 MHz) |
| SRAM | 264 KB | 520 KB |
| FPU | None (software float) | Single-precision VFP |
| DCP | N/A | Double-precision coprocessor |
| VREG | 1.20V (for OC) | 1.10V |

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
| Max biquads | 70 | 110 |
| Matrix outputs | 5 | 9 |
| S/PDIF outputs | 2 pairs | 4 pairs |
| USB input bit depth | 16-bit or 24-bit (alt setting) | 16-bit or 24-bit (alt setting) |
| S/PDIF bit depth | 24-bit | 24-bit |
| S/PDIF output conversion | Q28 >> 6 → int24 | float × 8388607 → int24 |
| Volume leveller | Q28 envelope + float gain | Float throughout |
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
| EQ worker data type | int32_t Q28, block-based | float, block-based, hybrid SVF/biquad |

### DMA

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Priority | Global bus priority bits | Per-channel high-priority flag |
| SPDIF TX channels | 0-1 (hardcoded) | 0-3 (hardcoded) |
| SPDIF TX IRQ | DMA_IRQ_1 (dedicated) | DMA_IRQ_1 (dedicated) |
| SPDIF RX channels | CH4, CH5 | CH5, CH6 |
| SPDIF RX IRQ | DMA_IRQ_0 (shared with I2S TX) | DMA_IRQ_0 (shared with I2S TX) |
| PDM channel | Dynamic (claim) | Dynamic (claim) |

### Clock Configuration

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| 48 kHz family | 307.2 MHz (VCO 1536 MHz / 5) | 307.2 MHz (VCO 1536 MHz / 5) |
| 44.1 kHz family | 264.6 MHz (VCO 1058.4 MHz / 4) | 264.6 MHz (auto PLL) |
| PLL config | Manual (`set_sys_clock_pll()`) | Automatic (`set_sys_clock_hz()`) |

### SPDIF Input ASRC (when `SPDIF_USE_ASRC = 1`)

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Inter-phase blend | Linear (2 phases) | Cubic-Hermite / Catmull-Rom (4 phases) |
| Coefficient format | Q30 int32 | float |
| Phases × taps | 256 × 32 | 256 × 64 |
| Pass-band edge | 0.70 · input_Nyquist (~16.8 kHz @ 48k) | 0.85 · input_Nyquist (~20.4 kHz @ 48k) |
| Measured stop-band | -104 dB | -106 dB |
| Pass-band ripple | 0.03 dB | 0.03 dB |
| ROM size | 32 KB (RAM-placed `.time_critical`) | 64 KB (flash, XIP-cached) |
| Inner-loop multiply | Software 32×32→32 (M0+ has only `MULS`) | M33 single-cycle FMA |
| Core 0 cost @ 48 kHz | ~30–45 % (verify on bench) | ~20 % |
| Optional fallback | `SPDIF_ASRC_RP2040_LOW_PRECISION 1` (Q15 single-cycle MULS, ~85 dB) | n/a |

---

## Memory Layout
*Last updated: 2026-04-28*

### RP2040 (264 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (5 × 2048 × 4) | 40 KB |
| Output buffers (5 × 192 × 4 + 2 × 192 × 4) | ~5.25 KB |
| Filters + recipes (7 channels) | ~8 KB |
| Loudness tables (2 × 61 × 2 × ~13B) | ~3 KB |
| Preset system (dir_cache + slot_buf + write_buf) | ~6 KB |
| Bulk param buffer (4 KB aligned) | ~4 KB |
| USB audio ring buffer (4 × 578) | ~2.3 KB |
| Channel names (7 × 32) | ~224 B |
| Leveller state + lookahead | ~2 KB |
| Per-channel preamp + master volume | ~48 B |
| ASRC state + I/O staging (only when SPDIF_USE_ASRC=1) | ~3.5 KB |
| ASRC Q31 coefficient table (RAM, only when SPDIF_USE_ASRC=1) | 32 KB |
| Other BSS | ~20 KB |
| **Total BSS** | **~90 KB** (ASRC=0) / **~94 KB** (ASRC=1) |
| Code in RAM (.text copy_to_ram) | ~72 KB (ASRC=0) / ~106 KB (ASRC=1) |
| SPDIF producer pools (heap, 2 × 8 × 192 × 8) | ~24 KB |
| SPDIF consumer pools (heap, 2 × 16 × 48 × 16) | ~24 KB |
| Stack + remaining heap | ~42 KB |

Measured `arm-none-eabi-size` totals: 197 KB (ASRC=0) → 235 KB (ASRC=1) — comfortable headroom in 264 KB SRAM.

### RP2350 (520 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (9 × 2048 × 4) | 72 KB |
| Filters + recipes | ~18 KB |
| Output buffers (9 × 192 × 4) | ~7 KB |
| Preset system (dir_cache + slot_buf + write_buf) | ~7 KB |
| Bulk param buffer (4 KB aligned) | ~4 KB |
| USB audio ring buffer (4 × 578) | ~2.3 KB |
| Channel names (11 × 32) | ~352 B |
| Leveller state + lookahead | ~2 KB |
| Per-channel preamp + master volume | ~48 B |
| ASRC state + I/O staging (only when SPDIF_USE_ASRC=1) | ~3.5 KB |
| Other BSS | ~24 KB |
| **Total BSS** | **~154 KB** (ASRC=0) / **~158 KB** (ASRC=1) |
| Code in RAM (.time_critical + copy_to_ram) | ~68 KB (+ ASRC float coefs in flash, see below) |
| SPDIF producer pools (heap, 4 × 8 × 192 × 8) | ~48 KB |
| SPDIF consumer pools (heap, 4 × 16 × 48 × 16) | ~48 KB |
| Stack + remaining heap | ~200 KB |

ASRC RP2350 uses a 64 KB float coefficient table that lives in **flash** (XIP cache holds one phase comfortably; no RAM impact). Measured `arm-none-eabi-size`: 258 KB total (ASRC=0) → 329 KB (ASRC=1, +65 KB text from coef table in flash).

### Flash Layout

| Region | RP2040 (2 MB) | RP2350 (4 MB) |
|--------|---------------|---------------|
| Firmware code | ~68 KB | ~66 KB |
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

## Vendor Command Reference
*Last updated: 2026-04-09*

| Command | Code | Direction | Description |
|---------|------|-----------|-------------|
| REQ_SET_EQ_PARAM | 0x42 | OUT | Set EQ band parameters |
| REQ_GET_EQ_PARAM | 0x43 | IN | Get EQ band parameters |
| REQ_SET_PREAMP | 0x44 | OUT | Set preamp gain (legacy: sets all input channels) |
| REQ_GET_PREAMP | 0x45 | IN | Get preamp gain (legacy: returns channel 0) |
| REQ_SET_BYPASS | 0x46 | OUT | Set master EQ bypass |
| REQ_GET_BYPASS | 0x47 | IN | Get master EQ bypass state |
| REQ_SET_DELAY | 0x48 | OUT | Set channel delay |
| REQ_GET_DELAY | 0x49 | IN | Get channel delay |
| REQ_GET_STATUS | 0x50 | IN | Get all channel peaks + CPU load (see Channel Metering) |
| REQ_SAVE_PARAMS | 0x51 | OUT | Save all params to flash |
| REQ_LOAD_PARAMS | 0x52 | OUT | Load params from flash |
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
| REQ_SET_OUTPUT_PIN | 0x7C | OUT | Set output GPIO pin |
| REQ_GET_OUTPUT_PIN | 0x7D | IN | Get output GPIO pin |
| REQ_GET_SERIAL | 0x7E | IN | Get unique board serial |
| REQ_GET_PLATFORM | 0x7F | IN | Get platform ID (0=RP2040, 1=RP2350) |
| REQ_CLEAR_CLIPS | 0x83 | IN | Read-then-clear clip flags (see Clip Detection) |
| REQ_PRESET_SAVE | 0x90 | IN | Save live state to preset slot (wValue=slot) |
| REQ_PRESET_LOAD | 0x91 | IN | Load preset slot to live state (wValue=slot) |
| REQ_PRESET_DELETE | 0x92 | IN | Delete preset slot (wValue=slot) |
| REQ_PRESET_GET_NAME | 0x93 | IN | Get 32-byte preset name (wValue=slot) |
| REQ_PRESET_SET_NAME | 0x94 | OUT | Set preset name (wValue=slot, payload=32 bytes) |
| REQ_PRESET_GET_DIR | 0x95 | IN | Get directory summary (7 bytes: byte 6 = master_volume_mode) |
| REQ_PRESET_SET_STARTUP | 0x96 | OUT | Set startup mode + default slot (2 bytes) |
| REQ_PRESET_GET_STARTUP | 0x97 | IN | Get startup config (3 bytes) |
| REQ_PRESET_SET_INCLUDE_PINS | 0x98 | OUT | Set include-pins flag (1 byte) |
| REQ_PRESET_GET_INCLUDE_PINS | 0x99 | IN | Get include-pins flag (1 byte) |
| REQ_PRESET_GET_ACTIVE | 0x9A | IN | Get active preset slot (1 byte, always 0-9) |
| REQ_SET_CHANNEL_NAME | 0x9B | OUT | Set channel name (wValue=channel, payload=1-32 bytes) |
| REQ_GET_CHANNEL_NAME | 0x9C | IN | Get channel name (wValue=channel, returns 32 bytes) |
| REQ_GET_ALL_PARAMS | 0xA0 | IN | Get complete DSP state (~2832 bytes, multi-packet control transfer) |
| REQ_SET_ALL_PARAMS | 0xA1 | OUT | Set complete DSP state (~2832 bytes, multi-packet control transfer) |
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
| REQ_SET_LEVELLER_LOOKAHEAD | 0xBC | OUT | Enable/disable leveller 10ms lookahead |
| REQ_GET_LEVELLER_LOOKAHEAD | 0xBD | IN | Get leveller lookahead state |
| REQ_SET_LEVELLER_GATE | 0xBE | OUT | Set leveller silence gate threshold (-96.0–0.0 dBFS, float) |
| REQ_GET_LEVELLER_GATE | 0xBF | IN | Get leveller silence gate threshold |
| SET_OUTPUT_TYPE | 0xC0 | OUT | Set output slot type (S/PDIF or I2S) |
| GET_OUTPUT_TYPE | 0xC1 | IN | Get output slot type |
| SET_I2S_BCK_PIN | 0xC2 | OUT | Set I2S BCK pin |
| GET_I2S_BCK_PIN | 0xC3 | IN | Get I2S BCK pin |
| SET_MCK_ENABLE | 0xC4 | OUT | Set MCK enable |
| GET_MCK_ENABLE | 0xC5 | IN | Get MCK enable |
| SET_MCK_PIN | 0xC6 | OUT | Set MCK pin |
| GET_MCK_PIN | 0xC7 | IN | Get MCK pin |
| SET_MCK_MULTIPLIER | 0xC8 | OUT | Set MCK multiplier (0=128x, 1=256x) |
| GET_MCK_MULTIPLIER | 0xC9 | IN | Get MCK multiplier |
| REQ_SET_PREAMP_CH | 0xD0 | OUT | Set per-channel preamp gain (wValue=channel) |
| REQ_GET_PREAMP_CH | 0xD1 | IN | Get per-channel preamp gain (wValue=channel) |
| REQ_SET_MASTER_VOLUME | 0xD2 | OUT | Set master volume (-128 to 0 dB, -128=mute) |
| REQ_GET_MASTER_VOLUME | 0xD3 | IN | Get master volume |
| REQ_SET_MASTER_VOLUME_MODE | 0xD4 | OUT | Set master volume persistence mode (0=independent, 1=with preset) |
| REQ_GET_MASTER_VOLUME_MODE | 0xD5 | IN | Get master volume persistence mode |
| REQ_SAVE_MASTER_VOLUME | 0xD6 | IN | Persist live master volume to directory's independent field (mode 0 source) |
| REQ_GET_SAVED_MASTER_VOLUME | 0xD7 | IN | Get the directory's independent master volume |
| REQ_SET_INPUT_SOURCE | 0xE0 | OUT | Set active input source (0=USB, 1=SPDIF) |
| REQ_GET_INPUT_SOURCE | 0xE1 | IN | Get active input source |
| REQ_GET_SPDIF_RX_STATUS | 0xE2 | IN | Get SPDIF RX status (16-byte SpdifRxStatusPacket) |
| REQ_GET_SPDIF_RX_CH_STATUS | 0xE3 | IN | Get IEC 60958 channel status (24 bytes) |
| REQ_SET_SPDIF_RX_PIN | 0xE4 | IN* | Set SPDIF RX GPIO pin (wValue=pin, returns status) |
| REQ_GET_SPDIF_RX_PIN | 0xE5 | IN | Get SPDIF RX GPIO pin |

### Bulk Parameter Transfer
*Last updated: 2026-04-09*

Transfers the complete DSP state in a single USB control transfer (~2832 bytes), replacing dozens of individual vendor requests.

**Wire format:** `WireBulkParams` (`bulk_params.h`, `WIRE_FORMAT_VERSION` 7) — packed struct with header, global params, crossfeed, legacy channel gains, delays, matrix crosspoints, matrix outputs, pin config, EQ bands, channel names, I2S config, leveller config, preamp config (`WirePreampConfig`, 16 bytes), master volume config (`WireMasterVolume`, 16 bytes), and input source config (`WireInputConfig`, 16 bytes). All arrays sized at platform maximums (RP2350: 11 channels, 9 outputs, 5 pins, 12 bands). Unused entries zero-padded.

**Transport:** Multi-packet USB EP0 control transfers using `usb_stream_transfer` from pico-extras. Packets are 64 bytes. No modifications to `usb_device.c` required — uses only public API (`usb_stream_setup_transfer`, `usb_start_transfer`, `usb_start_empty_transfer`).

**GET (0xA0):** `bulk_params_collect()` snapshots live state into `bulk_param_buf`, then streams it out in 64-byte packets via `usb_stream_transfer`. ZLP appended if total length is a multiple of 64.

**SET (0xA1):** Incoming data accumulated into `bulk_param_buf` via `usb_stream_transfer`. On completion, `bulk_params_pending` flag is set (after status-phase ACK). Main loop processes deferred: waits for Core 1 idle, mutes audio (256 samples), calls `bulk_params_apply()` with `include_pins` from preset directory, recalculates all filters and delays, then transitions Core 1 mode to match the new output enable state.

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
- **PIO1 SM1:** MCK generator (2-instruction toggle), independent of data path.
- **OutputSlot abstraction** in `usb_audio.c` manages per-slot type, holding either a SPDIF or I2S instance.
- **DMA IRQ:** S/PDIF TX uses DMA_IRQ_1 (dedicated). I2S TX uses DMA_IRQ_0 (shared with SPDIF RX when active). Both register via `irq_add_shared_handler()`.
- **Producer pools** are format-identical (PCM_S32, stride 8). The I2S library's connection callback left-shifts samples by 8 for MSB-first I2S framing.
- **No audio callback changes.** Core 1 remains output-type-agnostic.
- **Pipeline reset API** (`main.c`): two-phase `prepare_pipeline_reset()` / `complete_pipeline_reset()` brackets any disruptive output work. `complete_pipeline_reset()` runs the entire drain → enable_sync → feedback reset sequence with interrupts disabled to prevent inter-slot fill offsets. I2S→S/PDIF switch restores the SPDIF connection before zeroing the I2S instance to prevent a dangling `producer_pool->connection` pointer.
- **Boot-time I2S restoration:** `core0_init()` inspects `output_types[]` after `preset_boot_load()` and converts preset-saved I2S slots from the default SPDIF instances created by `usb_sound_card_init()`. For each I2S slot: disables SPDIF, unclaims the PIO SM, calls `audio_i2s_setup()` + `audio_i2s_connect_extra()`, and enables the I2S instance. MCK is started if any I2S slot exists and `i2s_mck_enabled` is set.

*Last updated: 2026-03-26*

### Clock Math at 307.2 MHz / 48 kHz

| Signal | Frequency | PIO Divider | Jitter |
|--------|-----------|-------------|--------|
| I2S BCK (Fs×64) | 3.072 MHz | 50.0 | Zero |
| MCK 128× | 6.144 MHz | 25.0 | Zero |
| MCK 256× | 12.288 MHz | 12.5 | Fractional |

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
- `WIRE_FORMAT_VERSION` = 3: adds `WireI2SConfig` (16 bytes) to `WireBulkParams`
- `WIRE_FORMAT_VERSION` = 4: adds `WireLevellerConfig` (16 bytes) to `WireBulkParams` (total 2864 bytes)
- `WIRE_FORMAT_VERSION` = 5: changes `mck_multiplier` wire encoding in `WireI2SConfig` from raw value to enum-style (0 = 128x, 1 = 256x)
- `WIRE_FORMAT_VERSION` = 6: adds `WirePreampConfig` (16 bytes) and `WireMasterVolume` (16 bytes) to `WireBulkParams`
- Backward compatible: V<9 slots default to all-S/PDIF; V9-V10 slots use old MCK encoding; V<12 slots use single preamp value for all channels, default master volume 0 dB; older wire payloads accepted without new fields

### BSS Impact

| Platform | Delta |
|----------|-------|
| RP2040 | +292 bytes |
| RP2350 | +528 bytes |

Full specification: `Documentation/Features/i2s_output_spec.md`

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
*Last updated: 2026-04-26*

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
- Factory default master volume = `MASTER_VOL_DEFAULT_DB` (-20 dB) — applied at boot when the directory is fresh, on factory reset, and on legacy migration
- `apply_master_volume_db()` in `flash_storage.c` delegates to `update_master_volume()` so all paths emit host notifications
- Slots with version < 12 default to 0 dB master volume (unity, no attenuation)
- `WireMasterVolume` (16 bytes) section in `WireBulkParams` V6+

---

## Audio Input Source System
*Last updated: 2026-04-11*

Abstraction layer enabling selection between multiple audio input sources. Currently supports USB (default) and SPDIF. Designed for future extensibility to I2S and ADAT inputs without restructuring.

### Files

- `audio_input.h` — `InputSource` enum, globals, constants
- `audio_input.c` — Global definitions

### Input Source Enum

```c
typedef enum {
    INPUT_SOURCE_USB   = 0,
    INPUT_SOURCE_SPDIF = 1,
    // Future: INPUT_SOURCE_I2S = 2, INPUT_SOURCE_ADAT = 3
} InputSource;
```

### Switching Behavior

- Source switching is deferred to the main loop via `input_source_change_pending` / `pending_input_source` flags (same pattern as output type switching)
- On switch: drain USB ring, `prepare_pipeline_reset()`, update `active_input_source`, `complete_pipeline_reset()`
- When input is not USB, `usb_audio_drain_ring()` is skipped — USB enumeration stays active but audio data is silently dropped
- SPDIF RX hardware only runs when SPDIF is the selected input source

### SPDIF RX Pin

- Default: GPIO 11 (`PICO_SPDIF_RX_PIN_DEFAULT`)
- Device-level setting stored in `PresetDirectory` (not per-preset)
- Configurable via `REQ_SET_SPDIF_RX_PIN` (0xE4) / `REQ_GET_SPDIF_RX_PIN` (0xE5)
- Pin change rejected when SPDIF input is active

### SPDIF RX Implementation

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
- RP2040: PIO1 SM2 (SM0=PDM, SM1=MCK occupied — patched to not clear instruction memory)

**Clock**: sys_clk 307.2 MHz → PIO clock 122.88 MHz (divider 2.5, exact). At 122.88 MHz: cy=20 (48kHz), cy=10 (96kHz), cy=5 (192kHz) — identical to original library values, zero error.

**DMA**: Channels 5+6 (RP2350) or 4+5 (RP2040) on DMA_IRQ_0 (shared with I2S TX when active). DMA_IRQ_1 is dedicated to SPDIF TX only. This isolates SPDIF RX from SPDIF TX, avoiding shared handler conflicts.

**State Machine**: `spdif_input.h/c`
- INACTIVE → ACQUIRING → LOCKED → RELOCKING (on signal loss) → LOCKED (on re-lock)
- Lock: ~64ms library internal stability + firmware debounce polls
- Loss: 10ms timeout
- Audio extraction: FIFO → 24-bit decode → per-channel preamp → buf_l/buf_r → process_input_block()

**Clock Servo**: PI controller in `spdif_input_update_clock_servo()` adjusts all output PIO dividers based on FIFO fill level (target 50%). Gains: KP=0.0005, KI=0.000005, deadband ±2 blocks. MCK divider is servoed alongside I2S data SM dividers when MCK is enabled, using `audio_i2s_mck_set_divider()` to keep master clock frequency-locked to the servoed output rate. *(See "SPDIF Input ASRC Mode" below for the alternative topology.) Last updated: 2026-04-28*

**Output Prefill**: On SPDIF lock acquisition, outputs are disabled and consumer buffers drained via `drain_and_disable_outputs()`. The pipeline then feeds real audio into consumer buffers while outputs are stopped. Once slot 0 consumer fill reaches 50% (8 of 16 buffers), outputs are started in sync via `enable_outputs_in_sync()`. This eliminates initial underruns after lock acquisition. Controlled by `spdif_prefilling` flag in `main.c`. *Last updated: 2026-04-12*

**Files**: `spdif_input.h` (API + status struct), `spdif_input.c` (lifecycle, audio extraction, clock servo, status queries)

### SPDIF Input ASRC Mode (optional)
*Last updated: 2026-04-28*

A polyphase-FIR asynchronous sample-rate converter is selectable at build time via `#define SPDIF_USE_ASRC 1` in `config.h` (default: 0 — legacy PIO-divider servo unchanged). When enabled, output PIO dividers are pinned at the nominal value for `audio_state.freq` and the conversion ratio is driven by the same kind of feedforward + fill-trim servo used by the USB feedback controller, with the **ASRC ratio as the actuator** instead of a USB feedback word.

**Why opt in:** decouples all four output instances from the SPDIF transmitter's clock — eliminating output-divider slewing as a jitter source — and removes any source of slot-relative phase perturbation from clock recovery. Drift-tracking only: pipeline Fs continues to follow the detected SPDIF rate via the existing rate-change machinery, and the ASRC ratio stays within ±5000 ppm of 1.0 (typical ±50 ppm in steady state).

**Files**: `asrc.h`, `asrc.c`, `asrc_coeffs_rp2350.h` (256×64 float, β=11.5, 64 KB flash), `asrc_coeffs_rp2040.h` (256×32 Q30, β=10.0, 32 KB RAM-placed via `.time_critical`), `scripts/gen_asrc_coeffs.py` (offline coefficient generator).

**Algorithm:** Kaiser-windowed sinc polyphase prototype, per-phase normalized to unity DC gain. Inter-phase blending is platform-specific:
- **RP2350** (single-cycle FMA on M33): cubic-Hermite (Catmull-Rom) blend of 4 adjacent phases. Basis functions: `h_-1 = -s/2 + s² - s³/2`, `h_0 = 1 - 5s²/2 + 3s³/2`, `h_+1 = s/2 + 2s² - 3s³/2`, `h_+2 = -s²/2 + s³/2` (sum identically 1 for all s — get any of these wrong and DC pass-through is broken). Measured stop-band -106 dB, pass-band ripple 0.03 dB to 0.85·input_Nyquist.
- **RP2040** (no FPU, no SMULL on M0+): linear blend of 2 adjacent phases. **Q30 coefficients and samples** with software 32×32→32 multiply (`fast_mul_q30`, four `MULS` halves + shift). Q30 instead of Q31 keeps the cross-term sum (`ah·bl + al·bh`) inside int32 — at full Q31 inputs the sum reaches ±4·10⁹ and silently wraps, which produces constant metallic distortion on near-full-scale audio; halving to Q30 caps the sum at ±2.14·10⁹ and costs ~1 bit of precision (~−180 dB, negligible). Internal Q30↔Q31 conversion is a single shift at the asrc.c I/O boundary; the rest of the firmware sees Q31 left-justified int32. Measured stop-band -104 dB, pass-band ripple 0.03 dB to 0.70·input_Nyquist.

The narrower RP2040 pass edge is dictated by the M0+ compute budget — the inner loop is the high-cost item on RP2040 (Verification §4 of the design plan).

**Servo:** Rate feedforward `R_ff = Fs_in_meas / Fs_pipe_nom` (using the SPDIF library's 32 ms-averaged actual rate) plus a proportional fill trim (`KP_FILL = 6.25e-6` per buffer × `consumer_fill - 8`, deadband ±1, ±5000 ppm anti-windup clamp). Both pass through a single-pole LPF on the committed ratio with α = 1/64 per ~20 ms tick → τ ≈ 1.3 s. Pitch slew worst case ≈ 0.07 cents/s, three orders of magnitude below FM audibility (5–10 cents at 4–8 Hz). Mirrors the USB feedback controller's holdoff, deadband and clamp idioms.

**Phase accumulator:** Q32.32 in int64. Per output sample, advance phase by R; on 32-bit wrap, consume one extra input sample. For ratio ≈ 1.0 + ppm, this is exactly one input per output ~99.99 % of the time.

**Hot path placement:** `asrc_process()` runs on Core 0 inside `spdif_input_poll()`, between FIFO read and the existing format-conversion + preamp helper. Marked `__not_in_flash_func`. No Core 1 changes — the EQ-worker / PDM modes continue exactly as today; ASRC is upstream of `process_input_block()`.

**Lifecycle:** `asrc_reset()` is invoked at SPDIF lock loss / lock acquire / rate change. `asrc_init(audio_state.freq)` re-arms after rate-change machinery has updated the pipeline rate (detected via a cached-rate compare in the poll loop). After reset, the first `ASRC_TAPS` input samples populate the kernel without producing output (priming); the servo's first valid cycle then lifts the mute.

**Vendor commands:** `0xE6 REQ_GET_ASRC_RATIO_PPM` (int32), `0xE7 REQ_GET_ASRC_LOCK_STATE` (uint8 bitfield), `0xE8 REQ_GET_ASRC_OVERRUN` (uint32). All GET-only; advisory. Resolve to constants in non-ASRC builds (asrc.c provides empty stubs) so the host surface is stable.

**Hard-constraint compliance:** Output slot alignment is preserved by construction — all output instances are written from `buf_out[ch][n]` inside a single `process_input_block(N)` call, and ASRC only varies `N` on the input side, identically across all slots. PIO dividers never move in ASRC mode.

### Vendor Commands

| Code | Command | Direction | Description |
|------|---------|-----------|-------------|
| 0xE0 | REQ_SET_INPUT_SOURCE | OUT | Set active input source (uint8_t payload) |
| 0xE1 | REQ_GET_INPUT_SOURCE | IN | Get active input source (returns uint8_t) |
| 0xE2 | REQ_GET_SPDIF_RX_STATUS | IN | Get SPDIF RX status (16-byte SpdifRxStatusPacket) |
| 0xE3 | REQ_GET_SPDIF_RX_CH_STATUS | IN | Get IEC 60958 channel status (24 bytes) |
| 0xE4 | REQ_SET_SPDIF_RX_PIN | IN* | Set SPDIF RX pin (wValue=pin, returns status byte) |
| 0xE5 | REQ_GET_SPDIF_RX_PIN | IN | Get SPDIF RX pin (returns uint8_t) |
| 0xE6 | REQ_GET_ASRC_RATIO_PPM | IN | (R−1.0)·1e6 as int32, advisory — meaningful only when SPDIF_USE_ASRC=1 |
| 0xE7 | REQ_GET_ASRC_LOCK_STATE | IN | uint8 bitfield: bit0 primed, bit1 muted (returns 0 in non-ASRC builds) |
| 0xE8 | REQ_GET_ASRC_OVERRUN | IN | Cumulative ASRC input-underrun count (uint32) |

*0xE4 uses the immediate-response SET pattern (same as `REQ_SET_I2S_BCK_PIN`).

### Persistence

- `SLOT_DATA_VERSION` 13 adds `input_source` (uint8_t) to `PresetSlot`
- Slots with version < 13 leave input source at its current value (USB by default)
- Factory reset sets `active_input_source = INPUT_SOURCE_USB`
- `WireInputConfig` (16 bytes) section in `WireBulkParams` V7+
- SPDIF RX pin stored in `PresetDirectory` (consumed existing padding byte, no directory format change)

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
- `REQ_GET_ALL_PARAMS` / `REQ_SET_ALL_PARAMS` (~2912 bytes) now use `tud_control_xfer()`'s native EP0 chunking — the old `usb_stream_setup_transfer` / `_vendor_stream` / `_vendor_*_complete` plumbing is gone.
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
