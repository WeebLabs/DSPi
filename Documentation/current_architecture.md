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
17. [Memory Layout](#memory-layout)
18. [Performance Characteristics](#performance-characteristics)

---

## System Overview
*Last updated: 2026-02-15*

DSPi is a USB Audio Class 1 (UAC1) digital signal processor built on the Raspberry Pi Pico (RP2040) and Pico 2 (RP2350). It receives stereo PCM audio over USB and routes it through a configurable DSP pipeline to multiple output channels via PIO-based S/PDIF and PDM.

- **RP2350:** 9 output channels — 4 S/PDIF stereo pairs (8 channels) + 1 PDM sub
- **RP2040:** 5 output channels — 2 S/PDIF stereo pairs (4 channels) + 1 PDM sub

**Key capabilities:**
- Parametric EQ: 11 channels on RP2350 (2 master + 9 outputs), 7 channels on RP2040 (2 master + 5 outputs)
- Matrix mixer with per-output gain, mute, phase invert, and delay
- BS2B crossfeed for headphone listening
- ISO 226:2003 loudness compensation
- Per-output configurable delay lines
- Runtime pin reconfiguration
- Full parameter persistence to flash
- Vendor control interface (WinUSB/WCID) for real-time parameter control

**Firmware binary:** `copy_to_ram` — entire firmware executes from SRAM for deterministic latency.

---

## Source File Map
*Last updated: 2026-04-04*

### Core Firmware (`firmware/DSPi/`)

| File | Purpose |
|------|---------|
| `main.c` | Entry point, initialization, main event loop |
| `usb_audio.c` | USB audio packet processing, DSP pipeline orchestration |
| `usb_audio.h` | USB audio interface API, AudioState struct |
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
| `usb_descriptors.c` | USB device/config/interface/endpoint descriptors (UAC1 + vendor) |
| `usb_descriptors.h` | Descriptor declarations |
| `dcp_inline.h` | RP2350 DCP (Double Coprocessor) inline assembly wrappers |

### LUFA Compatibility (`firmware/DSPi/lufa/`)

| File | Purpose |
|------|---------|
| `AudioClassCommon.h` | UAC1 descriptor type definitions |
| `StdDescriptors.h` | Standard USB descriptor structures |

### SPDIF Library (`firmware/pico-extras/src/rp2_common/pico_audio_spdif_multi/`)

Multi-instance S/PDIF output library (PIO-based, converted from pico-extras singleton).

---

## Build System
*Last updated: 2026-02-14*

### CMake Configuration

**Build file:** `firmware/DSPi/CMakeLists.txt`

**Binary type:** `copy_to_ram` (entire firmware in SRAM)

**Optimization levels:**
- General code: `-O2`
- DSP-critical files (`dsp_pipeline.c`, `usb_audio.c`, `crossfeed.c`, `loudness.c`): `-O3`

**Platform-specific sources:**
- RP2040: Includes `dsp_process_rp2040.S` (hand-coded biquad assembly)
- RP2350: Pure C with DCP inline assembly in `dcp_inline.h`

**Key compile definitions:**
- `AUDIO_FREQ_MAX=48000`
- `PICO_AUDIO_SPDIF_PIO=0`
- `PICO_AUDIO_SPDIF_DMA_IRQ=1`
- `PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1`

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
*Last updated: 2026-03-29*

**Library:** pico-extras `usb_device` (UAC1)

**Error handling:** The pico-extras USB IRQ handler (`usb_device.c`) receives `USB_INTS_ERROR_BITS` interrupts for CRC errors, bit stuff errors, RX overflow, RX timeout, and data sequence errors. All error types are handled by clearing the corresponding SIE status bits and incrementing per-type diagnostic counters — no bus reset or re-enumeration. The host retransmits automatically per USB spec. Counters are readable via `REQ_GET_USB_ERROR_STATS` (0xB2) and resettable via `REQ_RESET_USB_ERROR_STATS` (0xB3).

**Interfaces:**
1. **Audio Control (AC)** — Interface 0
2. **Audio Streaming (AS)** — Interface 1
   - Alt 0: Zero-bandwidth (idle)
   - Alt 1: 16-bit PCM, 2 channels (44.1/48/96 kHz), wMaxPacketSize=384
   - Alt 2: 24-bit PCM, 2 channels (44.1/48/96 kHz), wMaxPacketSize=576
   - EP OUT (isochronous): Audio data (44-49 samples/packet at 48 kHz)
   - EP IN (isochronous): Feedback (10.14 fixed-point rate)
3. **Vendor (WinUSB/WCID)** — Interface 2 (EP0 control transfers only)

### Volume & Mute

**Volume range:** -60 dB to 0 dB (1 dB resolution, 61 steps). Bottom step is fully silent. USB Audio Class 8.8 fixed-point dB encoding. Q15 lookup table (`db_to_vol[61]`) maps dB index to linear multiplier; index 0 = 0x0000 (silent), index 60 = 0x7FFF (unity).

**Mute:** UAC1 Feature Unit MUTE control. When `audio_state.mute` is set, `vol_mul` is forced to zero in the audio callback (RP2350: 0.0f, RP2040: 0), silencing all outputs immediately.

### Asynchronous Feedback Endpoint
*Last updated: 2026-03-29*

The device declares itself as a USB asynchronous sink, meaning it drives the audio clock from its own crystal oscillator rather than locking to the host's SOF timing. The feedback endpoint (`_as_sync_packet()`) reports the actual device sample rate to the host in 10.14 fixed-point format (samples per USB frame), allowing the host to adjust its packet sizes to match.

**Architecture:** Q16.16 dual-loop controller (`usb_feedback_controller.c/h`) with 10.14 wire serialization. All internal math uses Q16.16 fixed-point with rounded updates; only the endpoint-facing value is quantized to 10.14.

- **SOF handler** (`usb_sof_irq()` in `main.c`): Runs at each USB Start-of-Frame (1 kHz). Reads the DMA transfer counter of slot 0 (SPDIF or I2S) and combines with `words_consumed` to get a sub-buffer-precise word total. Calls `fb_ctrl_sof_update()` which performs the 4-SOF decimated measurement and control update.
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
*Last updated: 2026-03-27*

The DSP pipeline is decoupled from the USB IRQ via a lock-free SPSC ring buffer (`usb_audio_ring.h`). The USB ISR pushes raw packets into the ring (~5µs); the main loop drains the ring and runs the full DSP pipeline in thread context. This prevents the USB stack from being blocked for hundreds of microseconds per packet and eliminates ISR-context spinlock contention.

**Ring buffer:** 4 fixed-size slots × 578 bytes (576 payload + 2 length). ~2.3KB BSS. Placed in RAM (`__not_in_flash`) for flash-operation safety. Peek/consume pattern (zero-copy consumer).

**Memory barriers:** `__dmb()` at publish/acquire points. Redundant on RP2040 (Cortex-M0+ in-order single-bus) but required on RP2350 (Cortex-M33 write buffer).

**Gap detection:** USB packet arrival gap measurement runs in the ISR (at actual arrival time, not main-loop processing time) using file-scope `audio_ring_last_push_us`, reset on stream lifecycle transitions in `as_set_alternate()` and `usb_audio_flush_ring()`.

**Ring overruns:** Separate `audio_ring.overrun_count` counter (distinct from `spdif_overruns`). Queryable via `REQ_GET_STATUS` wValue=22.

**Deferred flash SET commands:** `REQ_PRESET_SET_NAME`, `REQ_PRESET_SET_STARTUP`, `REQ_PRESET_SET_INCLUDE_PINS` use separate pending flags per command type. Main loop copies payload under brief interrupt-off (~1µs) to prevent ISR/thread race, then drains ring and executes the flash write. GET-style flash commands (SAVE/LOAD/DELETE) remain synchronous in the vendor handler with real result codes.

### Packet Flow
*Last updated: 2026-03-27*

`_as_audio_packet()` → `usb_audio_ring_push()` → (main loop) → `usb_audio_drain_ring()` → `process_audio_packet(data, len)`

1. **Ring push (USB ISR)** — Copy raw packet into SPSC ring, detect arrival gaps
2. **Ring drain (main loop)** — Peek/process/consume loop, highest priority in main loop
3. **Buffer acquisition** — Get audio buffers from S/PDIF/I2S producer pools
4. **DSP processing** — Platform-specific pipeline (see below)
5. **Buffer return** — Give completed buffers to consumer pools for DMA

### RP2350 Float Pipeline
*Last updated: 2026-04-04*

All processing in IEEE 754 single-precision float. Hybrid SVF/biquad EQ filtering (SVF for bands below Fs/7.5, TDF2 biquad above).

| Stage | Description |
|-------|-------------|
| Input conversion | int16 → float, preamp gain |
| Loudness | 2 SVF shelf filters (low shelf + high shelf), volume-dependent |
| Master EQ | Block-based `dsp_process_channel_block()`, 10 bands per channel, hybrid SVF/biquad |
| Volume Leveller | Upward RMS compressor on master L/R with gain-reduction limiter (float throughout) |
| Crossfeed | BS2B lowpass + allpass (ILD + ITD) |
| Matrix mixing | Block-based: 2 inputs × 9 outputs with gain/phase |
| Output EQ | Block-based, 10 bands per output (Core 0: outputs 0-1, Core 1: outputs 2-7) |
| Output gain | Per-output gain × master volume |
| Delay | Float circular buffers, 8192 samples max |
| SPDIF output | Float → int16 conversion, 4 stereo pairs |
| PDM output | Float → Q28 for sigma-delta modulation |

### RP2040 Fixed-Point Pipeline
*Last updated: 2026-04-04*

Block-based two-phase architecture with dual-core EQ processing, all in Q28 fixed-point (28 fractional bits). 2 S/PDIF stereo pairs + 1 PDM sub (5 output channels).

**Phase 1 (Core 0, block-based where possible):**

| Stage | Description |
|-------|-------------|
| Input conversion | int16 → Q28 (shift left 14), preamp via `fast_mul_q28()` — block loop to `buf_l[192]`, `buf_r[192]` |
| Loudness | 2 biquads per-sample via `fast_mul_q28()` (Q28 coefficients, state coupling) |
| Master EQ | **Block-based** `dsp_process_channel_block()`, 10 bands per channel |
| Volume Leveller | Upward RMS compressor on master L/R with gain-reduction limiter (Q28 envelope + float gain) |
| Crossfeed | BS2B per-sample via `fast_mul_q28()` (Q28 coefficients, stereo coupling) |
| Matrix mixing | Q15 gains via `fast_mul_q15()` (16-bit partial products), 2 inputs × 5 outputs → `buf_out[5][192]` |

**Phase 2 (per-output block, dual-core or single-core):**

| Stage | Description |
|-------|-------------|
| Output EQ | **Block-based** `dsp_process_channel_block()`, 10 bands per output |
| Output gain + volume | Combined Q15 multiply via `fast_mul_q15()` (output gain × master volume) |
| Delay | int32 circular buffers, 4096 samples max (software-capped at 50ms) |
| SPDIF output | Q28 → int16 (shift right 14 with rounding), 2 stereo pairs |
| PDM output | Q28 direct to sigma-delta modulator (single-core fallback only) |

**Dual-core mode:** Core 0 handles input pipeline + matrix mix + SPDIF pair 1 (outputs 0-1), Core 1 handles SPDIF pair 2 (outputs 2-3) — both cores process per-output EQ, gain, delay, and S/PDIF conversion in parallel. PDM sub (output 4) runs on Core 1 in PDM mode; PDM and EQ worker outputs (2-3) are mutually exclusive.

**Performance advantage of block-based processing:** Biquad coefficients are loaded once per biquad per block instead of once per sample. For a 2-band output channel with 192-sample blocks, this reduces coefficient loads from 384 to 2.

---

## DSP Processing Engine
*Last updated: 2026-03-02*

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
| RP2350 | 9 | float | 8192 | 170 ms | 288 KB |
| RP2040 | 5 | int32_t | 4096 | 50 ms (software cap) | 80 KB |

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
USB Input → Preamp → Loudness → Master EQ → Volume Leveller → Crossfeed → Matrix Mix → Output EQ → ...
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
*Last updated: 2026-03-27*

### Flash Operation Safety

Flash erase/program requires quiescing XIP (execute-in-place). `flash_write_sector()` and `preset_delete()` use a guarded `multicore_lockout` to safely park Core 1 in RAM during the operation:

- **Guard condition:** `multicore_lockout_victim_is_initialized(1) && (__get_current_exception() == 0)`. The SDK function handles first-boot (Core 1 not launched) and launch-to-init race windows. The exception check skips lockout from IRQ context (USB vendor handler), where SDK lock internals are unsafe. IRQ callers rely on the `copy_to_ram` build for XIP safety.
- **Core 1 victim init:** `multicore_lockout_victim_init()` called at the start of `pdm_core1_entry()`.
- **Interrupt blackout:** ~45ms for sector erase + program. All interrupts disabled on Core 0 during this window. The existing mute strategy (`preset_loading` + `preset_mute_counter`) and feedback reseed cover the audio gap.

### Preset System (replaces single-sector storage)

The firmware uses a 10-slot preset system. A preset is always active — there is no "no preset" state. Each slot can be either configured (has user data in flash) or unconfigured (loads factory defaults). Presets are stored in individual 4 KB flash sectors with a separate directory sector for metadata. Slot 0 has the default name "Default".
*Last updated: 2026-03-07*

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
| slot_names[10][32] | 32-byte NUL-terminated names per slot |

### Preset Slot Data (Version 8)
*Last updated: 2026-03-10*

| Field | Description |
|-------|-------------|
| Magic | 0x44535033 ("DSP3") |
| Version | 8 |
| slot_index | Sanity-check slot number |
| CRC32 | Integrity check over data section |
| EQ recipes | NUM_CHANNELS x 12 bands |
| Preamp | gain_db |
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

- Core 0: Idle-time based EMA filter, reported via `global_status.cpu0_load`
- Core 1: Separate tracking for PDM vs EQ worker modes (both platforms)

---

## RP2040 vs RP2350 Comparison
*Last updated: 2026-04-04*

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
| Max samples | 4096 | 4096 |
| Max delay (48kHz) | 85 ms | 85 ms |
| RAM usage | 80 KB | 144 KB |

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
| SPDIF channels | 0-1 (hardcoded) | 0-3 (hardcoded) |
| PDM channel | Dynamic (claim) | Dynamic (claim) |

### Clock Configuration

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| 48 kHz family | 307.2 MHz (VCO 1536 MHz / 5) | 307.2 MHz (VCO 1536 MHz / 5) |
| 44.1 kHz family | 264.6 MHz (VCO 1058.4 MHz / 4) | 264.6 MHz (auto PLL) |
| PLL config | Manual (`set_sys_clock_pll()`) | Automatic (`set_sys_clock_hz()`) |

---

## Memory Layout
*Last updated: 2026-04-04*

### RP2040 (264 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (5 × 4096 × 4) | 80 KB |
| Output buffers (5 × 192 × 4 + 2 × 192 × 4) | ~5.25 KB |
| Filters + recipes (7 channels) | ~8 KB |
| Loudness tables (2 × 61 × 2 × ~13B) | ~3 KB |
| Preset system (dir_cache + slot_buf + write_buf) | ~6 KB |
| Bulk param buffer (4 KB aligned) | ~4 KB |
| USB audio ring buffer (4 × 578) | ~2.3 KB |
| Channel names (7 × 32) | ~224 B |
| Leveller state + lookahead | ~2 KB |
| Other BSS | ~20 KB |
| **Total BSS** | **~90 KB** |
| Code in RAM (.text copy_to_ram) | ~72 KB |
| SPDIF producer pools (heap, 2 × 8 × 192 × 8) | ~24 KB |
| SPDIF consumer pools (heap, 2 × 16 × 48 × 16) | ~24 KB |
| Stack + remaining heap | ~42 KB |

### RP2350 (520 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (9 × 4096 × 4) | 144 KB |
| Filters + recipes | ~18 KB |
| Output buffers (9 × 192 × 4) | ~7 KB |
| Preset system (dir_cache + slot_buf + write_buf) | ~7 KB |
| Bulk param buffer (4 KB aligned) | ~4 KB |
| USB audio ring buffer (4 × 578) | ~2.3 KB |
| Channel names (11 × 32) | ~352 B |
| Leveller state + lookahead | ~2 KB |
| Other BSS | ~24 KB |
| **Total BSS** | **~210 KB** |
| Code in RAM (.time_critical + copy_to_ram) | ~68 KB |
| SPDIF producer pools (heap, 4 × 8 × 192 × 8) | ~48 KB |
| SPDIF consumer pools (heap, 4 × 16 × 48 × 16) | ~48 KB |
| Stack + remaining heap | ~200 KB |

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
*Last updated: 2026-04-04*

| Command | Code | Direction | Description |
|---------|------|-----------|-------------|
| REQ_SET_EQ_PARAM | 0x42 | OUT | Set EQ band parameters |
| REQ_GET_EQ_PARAM | 0x43 | IN | Get EQ band parameters |
| REQ_SET_PREAMP | 0x44 | OUT | Set preamp gain |
| REQ_GET_PREAMP | 0x45 | IN | Get preamp gain |
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
| REQ_PRESET_GET_DIR | 0x95 | IN | Get directory summary (6 bytes) |
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

### Bulk Parameter Transfer
*Last updated: 2026-03-11*

Transfers the complete DSP state in a single USB control transfer (~2832 bytes), replacing dozens of individual vendor requests.

**Wire format:** `WireBulkParams` (`bulk_params.h`, `WIRE_FORMAT_VERSION` 2) — packed struct with header, global params, crossfeed, legacy channel gains, delays, matrix crosspoints, matrix outputs, pin config, EQ bands, and channel names. All arrays sized at platform maximums (RP2350: 11 channels, 9 outputs, 5 pins, 12 bands). Unused entries zero-padded.

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
*Last updated: 2026-03-23*

### Overview

Each output slot can be independently configured as S/PDIF or I2S at runtime via vendor commands. A new `pico_audio_i2s_multi` library mirrors the proven `pico_audio_spdif_multi` patterns. The S/PDIF library is completely unchanged.

### Architecture

- **PIO0:** Both S/PDIF (4 instructions) and I2S (8 instructions) programs coexist in instruction memory (12/32 slots). Each SM's side-set pins are independent — S/PDIF side-set = data pin, I2S side-set = BCK/LRCLK.
- **PIO1 SM1:** MCK generator (2-instruction toggle), independent of data path.
- **OutputSlot abstraction** in `usb_audio.c` manages per-slot type, holding either a SPDIF or I2S instance.
- **DMA IRQ:** Both libraries register on the same DMA IRQ line via `irq_add_shared_handler()`. Each iterates its own instance array.
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
- `WIRE_FORMAT_VERSION` = 3: adds `WireI2SConfig` (16 bytes) to `WireBulkParams` (total 2848 bytes)
- Backward compatible: V<9 slots default to all-S/PDIF; V2 bulk payloads accepted without I2S changes

### BSS Impact

| Platform | Delta |
|----------|-------|
| RP2040 | +292 bytes |
| RP2350 | +528 bytes |

Full specification: `Documentation/Features/i2s_output_spec.md`
