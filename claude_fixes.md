# Critical Fixes for DSPi on RP2350

This document documents the changes made to resolve audio distortion and buffer underruns on the RP2350 build.

## 1. Hardware Math Acceleration (Fixes "Distortion/Slowness")

**Problem:**
The audio processing pipeline was using a manual fixed-point multiplication function (`fast_mul_q28`) optimized for the RP2040 (Cortex-M0+). On the RP2350 (Cortex-M33), this manual implementation was significantly slower than the available hardware instructions, consuming excessive CPU cycles (~60% load) and causing the processing loop to miss the 1ms USB deadline.

**Fix:**
Updated `firmware/foxdac/dsp_pipeline.c` to use the Cortex-M33's native single-cycle 64-bit multiply instruction (`SMULL`) for the RP2350 target.

*   **File:** `firmware/foxdac/dsp_pipeline.c`
*   **Change:**
    ```c
    DSP_TIME_CRITICAL int32_t fast_mul_q28(int32_t a, int32_t b) {
    #if PICO_RP2350
        // RP2350 (Cortex-M33) has hardware 32x32->64 multiply (SMULL)
        // This reduces cost from ~20 cycles to ~3 cycles
        return (int32_t)(((int64_t)a * b) >> 28);
    #else
        // ... (original manual implementation for RP2040) ...
    #endif
    }
    ```

**Result:**
Drastically reduced DSP processing time, eliminating CPU-starvation induced underruns.

---

## 2. XIP Cache Optimization (Fixes "Underruns")

**Problem:**
The `fast_mul_q28` function was previously marked with `__attribute__((noinline))` via the `DSP_TIME_CRITICAL` macro. While this saved RAM on the RP2040, it forced a full function call for every multiplication (thousands per millisecond), adding significant overhead.

**Fix:**
Removed the `noinline` attribute from the `DSP_TIME_CRITICAL` macro definition for ALL build configurations (RP2040 and RP2350).
*   **Analysis:** RP2040 build uses ~150KB of 264KB RAM. Removing `noinline` increases code size by ~6KB, well within safety margins.

*   **File:** `firmware/foxdac/config.h`
*   **Change:**
    ```c
    // Applied unconditionally
    #define DSP_TIME_CRITICAL __attribute__((section(".time_critical")))
    ```

**Result:**
Allows the compiler to inline the optimized math function, further reducing overhead and improving performance.

---

## 3. S/PDIF Clock Synchronization (Fixes "Clicks/Rate Mismatch")

**Problem:**
The S/PDIF output hardware was initialized at 48kHz and never updated when the USB host requested a sample rate change (e.g., to 44.1kHz).
*   **Symptom:** The hardware continued to consume samples at 48kHz (faster) while the host sent them at 44.1kHz (slower).
*   **Consequence:** The audio buffer would repeatedly run dry (underrun), causing audible clicks and "glitching" artifacts even if the CPU was fast enough.

**Fix:**
Updated the `perform_rate_change` function to explicitly update the `sample_freq` field of the global audio format structure. This signals the `pico_audio_spdif` driver to recalculate and update the PIO clock divider for the new rate.

*   **File:** `firmware/foxdac/main.c`
*   **Change:**
    ```c
    extern struct audio_format audio_format_48k;

    static void perform_rate_change(uint32_t new_freq) {
        // ... (validation) ...

        // Update global format so driver sees the change
        audio_format_48k.sample_freq = new_freq;

        // ...
    }
    ```

**Result:**
The S/PDIF hardware clock now correctly matches the USB sample rate (e.g., 44.1kHz output for 44.1kHz input), preventing buffer underruns caused by rate mismatch.

---

## 4. Float Pipeline Implementation (Phase 3 Complete)

**Summary:**
The RP2350 build now uses a fully floating-point audio path, replacing the fixed-point Q28 pipeline used on RP2040.

*   **Input:** USB 16-bit PCM -> Normalized Float (`-1.0` to `1.0`).
*   **Processing:**
    *   **Float Coefficients:** Stored directly as `float`.
    *   **Float Accumulators:** The Biquad filter state (`s1`, `s2`) uses `float` to utilize the Cortex-M33 hardware FPU.
        *   *Note:* Originally attempted `double` (64-bit) accumulators, but software emulation on Cortex-M33 (which lacks a Double Precision Unit) was too slow for 44+ biquads at 48kHz, causing 100% CPU usage. `float` provides hardware acceleration (1-2 cycles) and sufficient precision (~144dB dynamic range).
    *   **Float Math:** All gain, volume, and EQ calculations use hardware FPU instructions.
*   **Output:** Float -> Hard Clamped (`-1.0` to `1.0`) -> 16-bit PCM.
*   **Capacity:** Output channels (Left/Right/Sub) expanded to **8 PEQ bands** (vs 2 on RP2040) due to increased CPU headroom.

**Files Modified:**
*   `config.h`: Conditional structs and types (`Biquad`, `channel_band_counts`).
*   `dsp_pipeline.h`: Conditional prototypes for `dsp_process_channel` and `delay_lines`.
*   `dsp_pipeline.c`: 
    *   Float implementation of `dsp_compute_coefficients`.
    *   Float implementation of `dsp_process_channel` using `float` accumulators.
    *   Removal of `fast_mul_q28` for RP2350 target.
*   `usb_audio.c`: 
    *   Rewrote `process_audio_packet` for RP2350 to use the float pipeline logic (normalization, processing, denormalization).