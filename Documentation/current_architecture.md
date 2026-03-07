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
11. [Loudness Compensation](#loudness-compensation)
12. [Flash Storage](#flash-storage)
13. [Pin Configuration](#pin-configuration)
14. [Core 1 Architecture](#core-1-architecture)
15. [RP2040 vs RP2350 Comparison](#rp2040-vs-rp2350-comparison)
16. [Memory Layout](#memory-layout)
17. [Performance Characteristics](#performance-characteristics)

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
*Last updated: 2026-02-14*

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
| `flash_storage.c` | Parameter save/load to last 4KB flash sector |
| `flash_storage.h` | Flash storage API |
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
*Last updated: 2026-02-14*

Defined in `main.c`, function `core0_init()`:

1. **GPIO setup** — LED (GPIO 25), status pin (GPIO 23)
2. **Clock configuration** — PLL set to 288 MHz (48 kHz family) or 264.6 MHz (44.1 kHz family)
   - RP2350: `set_sys_clock_hz()`, VREG 1.10V
   - RP2040: Manual PLL (`set_sys_clock_pll()`), VREG 1.20V (overclock)
3. **Bus priority** — DMA gets highest system bus priority
4. **USB + SPDIF init** — Must happen BEFORE PDM (SPDIF requires DMA channel 0)
5. **Flash parameter load** — Restore saved EQ, delays, preamp (all three globals: `global_preamp_db`, `global_preamp_mul`, `global_preamp_linear`), pin config from last 4KB flash sector
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
*Last updated: 2026-02-22*

### USB Stack

**Library:** pico-extras `usb_device` (UAC1)

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
*Last updated: 2026-02-16*

The device declares itself as a USB asynchronous sink, meaning it drives the audio clock from its own crystal oscillator rather than locking to the host's SOF timing. The feedback endpoint (`_as_sync_packet()`) reports the actual device sample rate to the host in 10.14 fixed-point format (samples per USB frame), allowing the host to adjust its packet sizes to match.

**Measurement method:** DMA word-level counting with IIR-filtered SOF measurement.

- **SOF handler** (`usb_sof_irq()`): Runs at each USB Start-of-Frame (1 kHz). Reads the DMA transfer counter of S/PDIF instance 0 and combines with `words_consumed` (total completed DMA words) to get a sub-buffer-precise total. Every 4 SOFs (matching `bRefresh=2`, i.e. 2^2=4 ms), computes the delta in DMA words and converts to 10.14 format: `raw = delta_words << 10`.
- **IIR filter:** First-order low-pass with α≈0.125 (K=3 shift), giving a ~32 ms time constant. Smooths jitter while tracking crystal drift. Effective resolution after filtering is ~160 ppm, sufficient for typical 50 ppm crystal oscillators.
- **Rate change:** On sample rate switch, `perform_rate_change()` pre-computes `nominal_feedback_10_14 = (freq << 14) / 1000` using 64-bit arithmetic and resets the IIR accumulator via `feedback_reset_value`, providing immediate correct feedback for the new rate.
- **Fallback:** If `feedback_10_14` is zero (not yet measured), `_as_sync_packet()` falls back to `nominal_feedback_10_14`.

**Key variables (defined in `main.c`, externed in `config.h`):**
| Variable | Type | Description |
|----------|------|-------------|
| `feedback_10_14` | `volatile uint32_t` | SOF-measured feedback value (10.14 fixed-point) |
| `nominal_feedback_10_14` | `volatile uint32_t` | Pre-computed nominal feedback for current rate |
| `feedback_reset_value` | `static volatile uint32_t` | Non-zero triggers IIR reset in SOF handler |

**S/PDIF library additions (`audio_spdif_instance_t`):**
| Field | Type | Description |
|-------|------|-------------|
| `words_consumed` | `volatile uint32_t` | Total DMA words completed (incremented in DMA IRQ) |
| `current_transfer_words` | `uint32_t` | Size of current in-flight DMA transfer |

**IRQ safety:** The SOF handler runs inside `isr_usbctrl`. The DMA IRQ handler (`audio_spdif_dma_irq_handler`) has the same default priority on Cortex-M0+/M33. Same-priority interrupts cannot preempt each other, so reading both `words_consumed` and `transfer_count` is atomic with respect to DMA completion.

### Packet Flow

`_as_audio_packet()` → `process_audio_packet(data, len)`

1. **Buffer acquisition** — Get audio buffers from 4 S/PDIF producer pools
2. **Gap detection** — Reset sync state if >50ms between packets
3. **Pre-fill** — Insert 2 silent buffers on restart to prevent underrun
4. **DSP processing** — Platform-specific pipeline (see below)
5. **Buffer return** — Give completed buffers to S/PDIF consumer pools for DMA

### RP2350 Float Pipeline
*Last updated: 2026-03-02*

All processing in IEEE 754 single-precision float. Hybrid SVF/biquad EQ filtering (SVF for bands below Fs/7.5, TDF2 biquad above).

| Stage | Description |
|-------|-------------|
| Input conversion | int16 → float, preamp gain |
| Loudness | 2 SVF shelf filters (low shelf + high shelf), volume-dependent |
| Master EQ | Block-based `dsp_process_channel_block()`, 10 bands per channel, hybrid SVF/biquad |
| Crossfeed | BS2B lowpass + allpass (ILD + ITD) |
| Matrix mixing | Block-based: 2 inputs × 9 outputs with gain/phase |
| Output EQ | Block-based, 10 bands per output (Core 0: outputs 0-1, Core 1: outputs 2-7) |
| Output gain | Per-output gain × master volume |
| Delay | Float circular buffers, 8192 samples max |
| SPDIF output | Float → int16 conversion, 4 stereo pairs |
| PDM output | Float → Q28 for sigma-delta modulation |

### RP2040 Fixed-Point Pipeline
*Last updated: 2026-02-15*

Block-based two-phase architecture with dual-core EQ processing, all in Q28 fixed-point (28 fractional bits). 2 S/PDIF stereo pairs + 1 PDM sub (5 output channels).

**Phase 1 (Core 0, block-based where possible):**

| Stage | Description |
|-------|-------------|
| Input conversion | int16 → Q28 (shift left 14), preamp via `fast_mul_q28()` — block loop to `buf_l[192]`, `buf_r[192]` |
| Loudness | 2 biquads per-sample via `fast_mul_q28()` (Q28 coefficients, state coupling) |
| Master EQ | **Block-based** `dsp_process_channel_block()`, 10 bands per channel |
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
*Last updated: 2026-02-22*

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
    audio_buffer_pool_t *consumer_pool;
    audio_buffer_t silence_buffer;
    // ... format, connection details
} audio_spdif_instance_t;
```

### Buffer Configuration

- Producer pool: 8 buffers × 192 samples × 2ch × 4 bytes = 12,288 bytes per pool
- Producer format: `AUDIO_BUFFER_FORMAT_PCM_S32` (24-bit audio in lower 24 bits of int32)
- Consumer pool: 4 buffers (half watermark)
- Consumer format: `AUDIO_BUFFER_FORMAT_PIO_SPDIF` (pre-encoded NRZI subframes)

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
*Last updated: 2026-02-15*

### Location

Last 4 KB sector of flash: `FLASH_SIZE - 4096`

### Format (Version 7)

| Field | Description |
|-------|-------------|
| Magic | 0x44535031 ("DSP1") |
| Version | 7 |
| CRC32 | Integrity check over data |
| EQ recipes | NUM_CHANNELS × 12 bands |
| Preamp | gain_db |
| Bypass | master bypass flag |
| Delays | NUM_CHANNELS delay values |
| Legacy gain/mute | 3 channels (backward compatibility) |
| Loudness | enabled, reference SPL, intensity |
| Crossfeed | enabled, preset, ITD, custom fc/feed |
| Matrix mixer | V5: crosspoints + output channels |
| Pin config | V6/V7: NUM_PIN_OUTPUTS pin assignments (3 on RP2040, 5 on RP2350) |

**Note:** V7 uses platform-dependent array sizes. Existing V6 flash data will fail the version check and trigger factory reset (acceptable for this major refactor).

### Operations

**Save:** Build struct → CRC32 → disable interrupts → erase sector → program → verify

**Load:** Read via XIP → check magic + version → verify CRC → copy to globals → recompute derived values

**Factory reset:** Erase sector → restore default filters (80 Hz HP on outputs 1-8, LP on PDM sub)

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
*Last updated: 2026-03-02*

### Hardware

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| CPU | Dual Cortex-M0+ @ 133 MHz (OC to 288 MHz) | Dual Cortex-M33 @ 150 MHz (OC to 288 MHz) |
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
| EQ channels | 7 (NUM_CHANNELS) | 11 (NUM_CHANNELS) |

### Delay Lines

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Channels | 5 | 9 |
| Type | int32_t | float |
| Max samples | 4096 | 8192 |
| Max delay (48kHz) | 50 ms (software cap) | 170 ms |
| RAM usage | 80 KB | 288 KB |

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
| 48 kHz family | 288 MHz (VCO 1152 MHz / 4) | 288 MHz (auto PLL) |
| 44.1 kHz family | 264.6 MHz (VCO 1058.4 MHz / 4) | 264.6 MHz (auto PLL) |
| PLL config | Manual (`set_sys_clock_pll()`) | Automatic (`set_sys_clock_hz()`) |

---

## Memory Layout
*Last updated: 2026-03-02*

### RP2040 (264 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (5 × 4096 × 4) | 80 KB |
| Output buffers (5 × 192 × 4 + 2 × 192 × 4) | ~5.25 KB |
| Filters + recipes (7 channels) | ~8 KB |
| Loudness tables (2 × 61 × 2 × ~13B) | ~3 KB |
| Other BSS | ~20 KB |
| **Total BSS** | **~116 KB** |
| Code in RAM (.text copy_to_ram) | ~64 KB |
| SPDIF producer pools (heap, 2 × 8 × 192 × 8) | ~24 KB |
| Stack + remaining heap | ~60 KB |

### RP2350 (520 KB SRAM)

| Section | Size (approx) |
|---------|---------------|
| Delay lines (9 × 8192 × 4) | 288 KB |
| Filters + recipes | ~18 KB |
| Output buffers (9 × 192 × 4) | ~7 KB |
| Other BSS | ~30 KB |
| **Total BSS** | **~341 KB** |
| Code in RAM (.time_critical + copy_to_ram) | ~63 KB |
| SPDIF producer pools (heap, 4 × 8 × 192 × 8) | ~48 KB |
| Stack + remaining heap | ~72 KB |

---

## Performance Characteristics
*Last updated: 2026-02-15*

### Buffer Sizes

| Buffer | Size |
|--------|------|
| USB packet | 44-49 samples (~1 ms at 48 kHz) |
| S/PDIF block | 192 samples (IEC60958 standard) |
| S/PDIF pool | 8 buffers per output pair |
| PDM DMA ring | 2048 words |
| PDM sample ring | 256 entries (Core 0 → Core 1) |

### Latency (at 48 kHz)

| Path | Latency |
|------|---------|
| USB → S/PDIF | 4-8 ms (buffer watermark + processing) |
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
*Last updated: 2026-03-01*

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
