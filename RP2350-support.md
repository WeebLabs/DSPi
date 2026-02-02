# DSPi: RP2350 Support & Migration Guide

This document covers adding RP2350 support to the DSPi firmware while maintaining backward compatibility with the RP2040.

---

## Overview

The RP2350 (Cortex-M33) provides hardware floating-point and DSP instructions that allow DSPi to replace its Q28 fixed-point audio pipeline with a float pipeline, gaining precision, headroom, and code simplicity. The firmware can support both chips from a single codebase using compile-time target selection.

---

## What RP2350 Brings

| Feature | RP2040 (Cortex-M0+) | RP2350 (Cortex-M33) | Impact |
|---------|----------------------|----------------------|--------|
| **Float (32-bit)** | Software emulated | Hardware FPU (FPv5), 1-2 cycles | ~6x faster |
| **Double (64-bit)** | Software emulated | Hardware DCP, 2-3 cycles | ~35x faster |
| **DSP MAC** | No dedicated instructions | Armv8-M DSP extension, single-cycle 32×32→64 MAC | Biquad acceleration |
| **SRAM** | 264 KB | 520 KB | Room for larger delay lines, more filters |
| **Stock Clock** | 133 MHz | 150 MHz | Overclock headroom TBD |
| **PIO** | 2 blocks | 3 blocks | Unchanged usage |

---

## Audio Pipeline Comparison

### Current Pipeline (RP2040 — Q28 Fixed-Point)

```
USB 16-bit PCM
  → int32_t (<<14 to Q28)
  → Preamp (fast_mul_q28)
  → Master EQ: 10 biquads/ch (int32 coefficients, int64 multiply-accumulate, >>28 shift)
  → Output EQ: 2 biquads/ch
  → Per-channel gain/mute (int64 multiply, >>15 shift)
  → Master volume (int64 multiply, >>15 shift)
  → Delay lines (int32_t[3][8192])
  → 16-bit PCM output (>>14 to SPDIF)
  → int32_t to PDM delta-sigma modulator
```

**Internal headroom:** ~+12 dB above 0 dBFS. A 16-bit input shifted left 14 bits occupies 29 of 31 available magnitude bits, leaving 2 bits before `INT32_MAX` overflow. A +12 dB boost on a full-scale signal clips internally.

**Precision:** ~168 dB theoretical dynamic range (28 fractional bits), but fixed-point accumulation through 10 cascaded biquad stages compounds rounding error.

### Proposed Pipeline (RP2350 — Float with Double Accumulators)

```
USB 16-bit PCM
  → float (÷32768.0f, normalized to ±1.0)
  → Preamp (float multiply)
  → Master EQ: 10 biquads/ch (float coefficients, double state accumulators)
  → Output EQ: 8 biquads/ch
  → Per-channel gain/mute (float multiply / zero)
  → Master volume (float multiply)
  → Delay lines (float[3][8192])
  → 16-bit PCM output (×32767.0f, clamp to ±1.0, to SPDIF)
  → float to PDM delta-sigma modulator
```

**Internal headroom:** Effectively unlimited. IEEE 754 float represents values up to ~3.4 × 10^38. The signal can be boosted +100 dB internally without overflow. Clipping only occurs at the final 16-bit output conversion (clamp to ±1.0).

**Precision:** 144 dB SNR (24-bit mantissa) at any magnitude. The double-precision accumulators (53-bit mantissa, ~318 dB) prevent rounding noise from building up through cascaded filter stages.

### Headroom Summary

| Pipeline | Internal Headroom | Precision per Stage | Accumulator Precision |
|----------|-------------------|--------------------|-----------------------|
| RP2040 Q28 | +12 dB (hard clip) | ~168 dB (fixed) | 64-bit integer (multiply only) |
| RP2350 Float | Unlimited (no overflow) | 144 dB (at any level) | 318 dB (double state) |

---

## Expanded Output EQ (2 → 8 Bands)

The RP2350's processing headroom allows expanding output channels from 2 to 8 PEQ bands per channel.

### Processing Budget

At 240 MHz and 48 kHz sample rate: **5,000 cycles per sample** on Core 0.

| Configuration | Total Biquads | Estimated Cycles | Core 0 Usage |
|---------------|---------------|------------------|--------------|
| Current (RP2040, 2 output bands) | 26 | ~1,300 (software Q28) | ~26% |
| RP2350, 2 output bands | 26 | ~780 (hardware float) | ~16% |
| RP2350, 8 output bands | 44 | ~1,320 (hardware float) | ~26% |

44 biquads × ~30 cycles each = 1,320 cycles, leaving ~3,680 cycles for preamp, gain, mute, metering, USB, and overhead.

The RP2350 with 8 output bands uses roughly the same CPU percentage as the current RP2040 with 2 output bands, because hardware float is ~3x faster per operation than software fixed-point.

### Memory Impact

| Component | Current | With 8 Output Bands |
|-----------|---------|---------------------|
| Biquad state | 26 × 28 B = 728 B | 44 × 36 B = 1,584 B |
| Filter recipes | 26 × 16 B = 416 B | 44 × 16 B = 704 B |
| **Total** | **1,144 B** | **2,288 B** |

Negligible on either chip.

### Channel Model (Updated)

| Channel | ID | Current Bands | RP2350 Bands | Role |
|---------|----|--------------|--------------|------|
| Master L | 0 | 10 | 10 | USB Input EQ |
| Master R | 1 | 10 | 10 | USB Input EQ |
| Out L | 2 | 2 | 8 | SPDIF Output EQ |
| Out R | 3 | 2 | 8 | SPDIF Output EQ |
| Sub | 4 | 2 | 8 | PDM Subwoofer EQ |

---

## Dual-Target Compatibility

### Approach

The Pico SDK defines `PICO_RP2040` and `PICO_RP2350` preprocessor macros based on the build target. The firmware uses `#if` / `#else` blocks at the points where the two chips diverge, keeping all shared code unconditional.

**Build commands:**
```bash
# RP2040
cmake -DPICO_BOARD=pico ..

# RP2350
cmake -DPICO_BOARD=pico2 ..
```

Same source tree, different binary output.

### What's Shared (No Conditionals Needed)

- USB descriptor structure and vendor request protocol
- Control request handler logic (request codes 0x42–0x57)
- Channel model and band indexing
- PIO programs (SPDIF encoding, PDM bit-bang) — binary compatible
- DMA configuration
- Core allocation (Core 0: USB/DSP, Core 1: PDM)
- Multicore launch and ring buffer mechanism
- Flash storage recipe format (`EqParamPacket` is already float)
- Filter coefficient math (already uses float for computation)

### What Diverges (Conditional Compilation)

**1. Data Types (`config.h`)**
```c
#if PICO_RP2350
typedef struct {
    float b0, b1, b2, a1, a2;   // float coefficients
    double s1, s2;               // 64-bit state accumulators
} Biquad;                        // 36 bytes
#else
typedef struct {
    int32_t b0, b1, b2, a1, a2; // Q28 coefficients
    int32_t s1, s2;              // 32-bit state
} Biquad;                        // 28 bytes
#endif
```

**2. Band Counts (`config.h`)**
```c
#if PICO_RP2350
static const uint8_t channel_band_counts[5] = {10, 10, 8, 8, 8};  // 44 total
#else
static const uint8_t channel_band_counts[5] = {10, 10, 2, 2, 2};  // 26 total
#endif
```

**3. DSP Processing (`dsp_pipeline.c`)**

The biquad processing function has two implementations:

```c
#if PICO_RP2350
float dsp_process_channel(Biquad *biquads, float input, uint8_t channel) {
    float sample = input;
    for (int i = 0; i < channel_band_counts[channel]; i++) {
        Biquad *bq = &biquads[i];
        if (bq->a1 == 0.0f && bq->a2 == 0.0f) continue;  // skip flat

        double result = (double)bq->b0 * sample + bq->s1;
        bq->s1 = (double)bq->b1 * sample - (double)bq->a1 * result + bq->s2;
        bq->s2 = (double)bq->b2 * sample - (double)bq->a2 * result;

        sample = (float)result;
    }
    return sample;
}
#else
int32_t dsp_process_channel(Biquad *biquads, int32_t input, uint8_t channel) {
    int32_t sample = input;
    for (int i = 0; i < channel_band_counts[channel]; i++) {
        Biquad *bq = &biquads[i];
        if (bq->a1 == 0 && bq->a2 == 0) continue;

        int64_t result = (int64_t)bq->b0 * sample + ((int64_t)bq->s1 << FILTER_SHIFT);
        int32_t out = (int32_t)(result >> FILTER_SHIFT);
        bq->s1 = (int32_t)(((int64_t)bq->b1 * sample - (int64_t)bq->a1 * out) >> FILTER_SHIFT) + bq->s2;
        bq->s2 = (int32_t)(((int64_t)bq->b2 * sample - (int64_t)bq->a2 * out) >> FILTER_SHIFT);

        sample = out;
    }
    return sample;
}
#endif
```

**4. Coefficient Storage (`dsp_pipeline.c`)**

```c
void dsp_compute_coefficients(Biquad *bq, uint8_t type, float freq, float Q, float gain_db) {
    // ... shared float coefficient math (b0_f, b1_f, b2_f, a0_f, a1_f, a2_f) ...

#if PICO_RP2350
    bq->b0 = b0_f / a0_f;
    bq->b1 = b1_f / a0_f;
    bq->b2 = b2_f / a0_f;
    bq->a1 = a1_f / a0_f;
    bq->a2 = a2_f / a0_f;
#else
    // Quantize to Q28
    float inv_a0 = 1.0f / a0_f;
    bq->b0 = (int32_t)(b0_f * inv_a0 * (1 << FILTER_SHIFT));
    bq->b1 = (int32_t)(b1_f * inv_a0 * (1 << FILTER_SHIFT));
    // ... etc
#endif
}
```

**5. Audio Sample Processing (`usb_audio.c`)**

```c
#if PICO_RP2350
    float sample_l = (float)raw_l / 32768.0f;
    float sample_r = (float)raw_r / 32768.0f;
    // ... float processing ...
    int16_t out_l = (int16_t)(fmaxf(-1.0f, fminf(1.0f, sample_l)) * 32767.0f);
#else
    int32_t sample_l = (int32_t)raw_l << 14;
    int32_t sample_r = (int32_t)raw_r << 14;
    // ... Q28 processing ...
    int16_t out_l = (int16_t)(sample_l >> 14);
#endif
```

**6. PLL Configuration (`main.c`)**

```c
#if PICO_RP2350
    // RP2350 PLL parameters — values TBD, require characterization
    set_sys_clock_pll(/* RP2350 VCO/divider for 240 MHz */);
#else
    set_sys_clock_pll(1440000000, 6, 1);  // 240 MHz on RP2040
#endif
```

**7. Delay Lines (`usb_audio.c`)**

```c
#if PICO_RP2350
static float delay_lines[3][MAX_DELAY_SAMPLES];
#else
static int32_t delay_lines[3][MAX_DELAY_SAMPLES];
#endif
```

**8. Flash Storage Version (`flash_storage.c`)**

```c
#if PICO_RP2350
#define FLASH_VERSION 3
#else
#define FLASH_VERSION 2
#endif
```

Each target loads its own flash version. An RP2040 flash image won't be loaded on RP2350 and vice versa (version mismatch triggers factory reset to safe defaults).

### What This Means Practically

The conditional blocks are concentrated in:
- `config.h` — type definitions, band counts
- `dsp_pipeline.c` — biquad processing and coefficient storage
- `usb_audio.c` — sample conversion, gain/volume math, delay line types
- `main.c` — PLL parameters
- `flash_storage.c` — version number

Everything else compiles identically for both targets. The `#if PICO_RP2350` / `#else` pattern is readable and explicit — there's no abstraction layer to obscure what each chip is actually doing.

---

## Changes Per File

### `config.h`
- Conditional `Biquad` struct (float+double vs int32)
- Conditional band counts (8 vs 2 for output channels)
- Remove `FILTER_SHIFT`, `clip_s32()`, `clip_s64_to_s32()` under RP2350 path (keep for RP2040)

### `dsp_pipeline.c`
- Conditional `dsp_process_channel()` — float biquad vs Q28 biquad
- Conditional coefficient storage in `dsp_compute_coefficients()` — direct float vs Q28 quantization
- RP2350 path deletes `fast_mul_q28()` usage entirely

### `usb_audio.c`
- Conditional sample type (`float` vs `int32_t`) throughout audio callback
- Conditional input conversion (÷32768 vs <<14)
- Conditional preamp/gain/volume application (float multiply vs Q28 multiply)
- Conditional delay line type
- Conditional output conversion (×32767 clamp vs >>14)
- Conditional peak metering (`fabsf()` vs `abs()`)

### `pdm_generator.c`
- PDM delta-sigma modulator stays integer-based on both targets (it operates in the 1-bit domain)
- Input conversion changes: RP2350 receives float from ring buffer, converts to int32 at PDM input
- Inner loop (256× oversampling, 32 bits per word) stays bit-manipulation on both targets

### `flash_storage.c`
- Conditional `FLASH_VERSION` (3 vs 2)
- Storage struct unchanged in format (recipes are already float, gain is already float)
- Load function: version check gates which fields are read (backward compat on each target)

### `main.c`
- Conditional PLL configuration for 48 kHz and 44.1 kHz clock paths
- Conditional `vreg_set_voltage()` if needed for RP2350 overclocking

### `CMakeLists.txt`
- Target selection via `PICO_BOARD` variable (set at cmake invocation, not hardcoded)
- Pico SDK 2.2.0 already present (supports both RP2040 and RP2350)
- Float ABI flags (`-mfloat-abi=hard -mfpu=fpv5-sp-d16`) auto-set by SDK for RP2350
- Verify `pico_audio_spdif` library compatibility with RP2350

### `usb_descriptors.c`
- No changes needed (USB protocol is the same on both chips)

---

## DSPi Console App Changes

The macOS companion app needs updates to support the expanded output EQ:

- **Channel band count**: Update the `Channel.bandCount` property to return 8 for output channels when connected to an RP2350-based device (or make it unconditional if RP2040 firmware is also updated to support the band count query).
- **UI layout**: Output channel pages need space for 8 PEQ bands instead of 2.
- **Frequency response graph**: Already loop-driven, will render additional bands automatically.
- **Import/export**: Filter file format already supports arbitrary band counts per channel.
- **Device detection**: Could query firmware version or a new vendor request to determine channel capabilities, or simply always send 8 bands (unused bands stay flat/bypassed on RP2040).

No USB protocol changes are required — the existing `REQ_SET_EQ_PARAM` / `REQ_GET_EQ_PARAM` requests already encode channel and band indices, and band indices up to 9 are already supported.

---

## Memory Budget

| Component | RP2040 (26 bands, int32) | RP2350 (44 bands, float) | Notes |
|-----------|--------------------------|--------------------------|-------|
| Delay lines | 96 KB | 96 KB | float same size as int32 |
| Biquad state | 728 B | 1,584 B | double accumulators |
| Filter recipes | 416 B | 704 B | More bands |
| PDM DMA buffer | 8 KB | 8 KB | Unchanged |
| Audio buffers | 6 KB | 6 KB | SPDIF pool |
| **Total** | **~111 KB / 264 KB** | **~112 KB / 520 KB** | Fits easily on both |

---

## Hardware Risks

### PLL Configuration (Highest Risk)

The current firmware switches between two PLL configs:
- **48 kHz path**: `set_sys_clock_pll(1440000000, 6, 1)` → 240 MHz
- **44.1 kHz path**: `set_sys_clock_pll(1236000000, 7, 1)` → 176.57 MHz

RP2350 has different VCO range limits. These exact VCO/post-divider combinations may not be valid. Finding equivalent configurations that produce correct S/PDIF and PDM bit rates requires PLL parameter characterization (possibly with an oscilloscope or logic analyzer).

### Overclocking

The current firmware runs at 240 MHz (overclock from 133 MHz stock). RP2350 stock is 150 MHz. Community reports suggest 300+ MHz is achievable, but the PLL parameter space is different and needs characterization. `vreg_set_voltage()` may need different values.

### PIO Compatibility

PIO instruction set is binary-compatible between RP2040 and RP2350. The S/PDIF and PDM PIO programs should work unchanged. GPIO pad electrical characteristics have minor differences on RP2350 — output signal integrity should be verified.

### Pico SDK Version

The repo already includes **Pico SDK 2.2.0** at `pico-sdk/`, which supports RP2350. No SDK update is needed.

---

## pico_audio_spdif Library Compatibility

**Location:** `firmware/pico-extras/src/rp2_common/pico_audio_spdif/`

**Verdict: No changes needed.** The library is chip-agnostic by design.

### Analysis

The library has **zero** `#ifdef PICO_RP2040` or `PICO_RP2350` conditionals. All hardware interaction goes through generic pico-sdk abstractions (DMA, PIO, GPIO, IRQ) that handle chip differences transparently. The only RP2350-aware code is an auto-generated `PICO_PIO_VERSION` guard in the `.pio.h` output, which pioasm produces automatically for both targets.

### PIO Program

The SPDIF encoder is 4 instructions implementing NRZI encoding:

```asm
.program audio_spdif
.side_set 1
output_low:
    out x, 1            side 0
    jmp !x, output_low  side 0
output_high:
    out x, 1            side 1
    jmp !x, output_high side 1
```

All standard PIO instructions, binary-compatible on both chips. No RP2350-specific opcodes.

### DMA Usage

The library uses generic DMA APIs throughout:
- `dma_channel_claim()`, `dma_channel_configure()`, `dma_channel_transfer_from_buffer_now()`
- `dma_irqn_get_channel_status()`, `dma_irqn_acknowledge_channel()`, `dma_irqn_set_channel_enabled()`

No chip-specific register accesses. DMA channel 0 is hardcoded in the firmware config, and SPDIF must initialize before PDM to claim it — this constraint is the same on both chips.

### Sample Format

The library accepts **PCM_S16** (signed 16-bit) stereo samples and handles SPDIF subframe encoding internally via a 256-entry lookup table (`spdif_lookup[]`). The encoding is purely algorithmic — bit-packing and parity calculation with no hardware dependencies.

For the float pipeline, conversion happens at the handoff point in `usb_audio.c`:
```c
// Float pipeline → int16 for SPDIF library
int16_t out = (int16_t)(fmaxf(-1.0f, fminf(1.0f, sample)) * 32767.0f);
```

The library never touches the DSP path. It just takes finished int16 samples and encodes them.

### Firmware API Surface

The firmware uses exactly 6 functions from the library:

| Function | Purpose |
|----------|---------|
| `audio_new_producer_pool()` | Creates 8-buffer pool (192 samples each, fixed by S/PDIF spec) |
| `audio_spdif_setup()` | Configures PIO state machine, DMA channel, GPIO pin 20 |
| `audio_spdif_connect_extra()` | Links producer pool to SPDIF consumer with watermark at 4 buffers |
| `audio_spdif_set_enabled()` | Starts/stops PIO state machine and DMA transfers |
| `take_audio_buffer()` | Gets free buffer from pool to fill with processed samples |
| `give_audio_buffer()` | Returns filled buffer for DMA transfer to PIO |

### Configuration

```c
// From config.h / CMakeLists.txt
#define PICO_AUDIO_SPDIF_PIN 20          // GPIO pin for SPDIF output
#define PICO_AUDIO_SPDIF_DMA_IRQ 1       // DMA IRQ 1
struct audio_spdif_config config = {
    .pin = PICO_AUDIO_SPDIF_PIN,
    .dma_channel = 0,                    // Hardcoded — must init before PDM
    .pio_sm = 0
};
```

### pico-extras Version Note

The pico-extras library is bundled at `firmware/pico-extras/` as a separate copy from the main SDK. Its version should be verified to be aligned with SDK 2.2.0 — if it predates RP2350 support, the auto-generated PIO headers may need regeneration via `pioasm`.

---

## What Doesn't Change

Regardless of target chip:
- USB descriptor structure and control protocol
- Vendor request codes (0x42–0x57) and packet formats
- `EqParamPacket` format (already float)
- PIO programs (S/PDIF encoding, PDM bit-bang)
- DMA configuration pattern
- Core allocation (Core 0: USB/audio/DSP, Core 1: PDM modulator)
- Multicore launch and ring buffer mechanism
- Channel model (5 channels, gain/mute on outputs)

---

## Migration Order

### Phase 1: Build Infrastructure
1. ~~Update Pico SDK to 2.0.0+~~ — Already at 2.2.0, no action needed
2. Modify `CMakeLists.txt` to accept `PICO_BOARD` without hardcoding
3. Verify the existing RP2040 firmware still builds and runs unchanged

### Phase 2: Boot on RP2350
4. Build with `-DPICO_BOARD=pico2`, zero code changes
5. Verify PIO/DMA/USB enumerate correctly on RP2350
6. Characterize and fix PLL configurations for both clock paths

### Phase 3: Float Pipeline
7. Add conditional `Biquad` struct and band counts in `config.h`
8. Add conditional biquad processing in `dsp_pipeline.c`
9. Add conditional sample processing in `usb_audio.c`
10. Add conditional delay line types
11. Update `pdm_generator.c` input conversion

### Phase 4: Integration
12. Bump flash version for RP2350 target
13. Verify save/load/factory-reset on both targets
14. Update DSPi Console app for 8 output bands
15. Test end-to-end on both chips

**Phase 2** carries the most risk (PLL/hardware compatibility unknowns). The SDK is already compatible (2.2.0). **Phases 3–4** are straightforward code changes — the DSP conversion is mostly simplification (deleting fixed-point machinery on the RP2350 path).

### Estimated Scope

- ~400–500 lines of new/changed code across 6–7 firmware files
- PLL characterization work (may require test equipment)
- Console app: UI layout changes for 8 output bands per channel page
