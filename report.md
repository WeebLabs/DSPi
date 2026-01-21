# Code Audit Report: DSPi Firmware (FoxDAC)

**Date:** 2026-01-21
**Target:** RP2040 Audio DSP Firmware
**Auditor:** Senior QA Developer

## Executive Summary

The DSPi firmware demonstrates a solid understanding of USB Audio and DSP principles on the RP2040 platform. The architecture using Core 0 for USB/Control and Core 1 for PDM generation is sound. However, the current implementation contains **critical stability hazards** related to Flash memory operations that will cause audio glitches or hardware-damaging DC transients. Additionally, there are significant performance bottlenecks in the PDM generation and interrupt handling logic that threaten the robustness of the system at high sample rates or CPU loads.

## Critical Issues (Must Fix)

### 1. Flash XIP Stall & Subwoofer DC Risk
**Severity: CRITICAL**
**Location:** `flash_storage.c` (flash_save_params) and `pdm_generator.c` (pdm_core1_entry)

When `flash_save_params()` is called, it executes `flash_range_erase()` and `flash_range_program()`. On the RP2040, writing to internal Flash halts all XIP (Execute-In-Place) reads. This means any code executing from Flash will stall (freeze) until the write operation completes.

The function `pdm_core1_entry()` runs on Core 1 and is **not** marked with `__not_in_flash_func`. Therefore, it resides in Flash. When Core 0 saves parameters, Core 1 will freeze.
*   **Consequence:** The subwoofer PDM output (GPIO 10) will stop toggling and hold its last state (Logic 1 or 0) for the duration of the flash erase (tens of milliseconds). This outputs a massive DC signal to the subwoofer amplifier, resulting in a loud "thump" and potential speaker damage.
*   **Fix:** Apply `__not_in_flash_func` to `pdm_core1_entry`, `pdm_push_sample`, `fast_rand`, and any other functions called by the Core 1 loop. This ensures they run from RAM and continue generating silence/audio during Flash writes.

### 2. Interrupt Latency & Jitter
**Severity: HIGH**
**Location:** `main.c` (`dsp_compute_coefficients`, `dsp_recalculate_all_filters`)

The `dsp_compute_coefficients` function utilizes heavy software-emulated floating-point math (`sinf`, `cosf`, `powf`) to calculate biquad parameters.
*   In `main.c` loop, `save_and_disable_interrupts()` is called around `dsp_compute_coefficients`. While this protects shared state, blocking interrupts for the duration of trigonometric calculations (thousands of cycles) introduces significant jitter to the USB interrupt servicing.
*   `dsp_recalculate_all_filters` (triggered on rate change or load) calculates ~60 filters sequentially. If this runs inside a lock (or if it delays the main loop significantly while USB interrupts are fighting for time), it risks buffer underruns.
*   **Fix:** Calculate new coefficients into a temporary structure *outside* the critical section. Only disable interrupts for the brief moment required to `memcpy` the new coefficients into the active `filters` array.

### 3. PDM Generator Inefficiency
**Severity: MEDIUM**
**Location:** `pdm_generator.c` (`pdm_core1_entry`)

The Sigma-Delta modulator uses a nested loop structure that is computationally expensive:
```c
for (int k = 0; k < 32; k++) { ... }
```
This runs 32 iterations * 8 chunks * 48000 samples = ~12.3 million times per second. The bit-wise logic inside is inefficient.
*   **Consequence:** High CPU usage on Core 1, generating heat and preventing future feature expansion (e.g., higher order modulators or higher sample rates).
*   **Fix:** Unroll the inner loop 32x or use inline assembly/lookup tables to compute the PDM stream.

### 4. Race Conditions in Ring Buffer
**Severity: MEDIUM**
**Location:** `pdm_generator.c` (`pdm_ring`, `pdm_push_sample`)

The `pdm_ring` buffer stores `pdm_msg_t` structs (8 bytes). Copying a struct is not an atomic operation.
*   **Scenario:** Core 0 writes the first 4 bytes of the struct, then is interrupted or Core 1 pre-empts (unlikely on separate cores, but memory visibility applies).
*   While the index `pdm_head` is updated *after* the write, the compiler may reorder these stores without a memory barrier.
*   **Fix:** Add `__dmb()` (Data Memory Barrier) before updating `pdm_head` (producer) and `pdm_tail` (consumer) to ensure data writes are committed before the index changes.

## Code Quality & Architecture

### 1. Fixed Point Arithmetic (Q4.28)
The use of Q4.28 provides high precision but low headroom (+/- 8.0). High-Q filters or bass boosts could easily overflow intermediate buffers.
*   **Recommendation:** Ensure `fast_mul_q28` handles saturation correctly, or migrate to Q8.24 if more headroom is needed (at cost of noise floor).

### 2. USB Feedback Logic
The feedback drift correction in `usb_audio.c` uses a simple proportional controller: `correction = (drift * 50) / 1000`.
*   **Observation:** This is crude but functional. However, the gain (0.05) is magic. If the host PC drifts wildly, this might oscillate.
*   **Recommendation:** Implement a PID controller or at least smooth the feedback value to prevent jitter in the host's rate estimation.

### 3. Memory Usage
*   `delay_lines` consumes ~96KB of RAM.
*   `pdm_dma_buffer` consumes 8KB.
*   RP2040 RAM is striped. Ensure the linker places these large arrays without conflict. The current setup seems safe given the total 264KB available.

### 4. Volatile Usage
The code heavily relies on `volatile` global variables. While often necessary in embedded systems, it makes tracking state flow difficult.
*   **Recommendation:** Encapsulate audio state into a single protected structure or use atomic accessors.

## Recommendations for Professional Robustness

1.  **RAM Placement:** Immediately move `pdm_core1_entry` to RAM (`__not_in_flash_func`).
2.  **Locking Strategy:** Refactor `eq_update_pending` handling to compute-then-commit.
3.  **Watchdog:** The watchdog is enabled (`watchdog_enable(8000, 1)`), which is good practice. Ensure the main loop (Core 0) and the PDM loop (Core 1) both have mechanisms to prevent hanging (though currently only Core 0 pets the dog). If Core 1 hangs, Core 0 might continue running, leaving the sub dead. Consider a mechanism where Core 0 checks Core 1's heartbeat before petting the dog.

## Conclusion
The DSPi firmware is a functional prototype but is not yet "production ready" due to the Flash/XIP stall issue. Fixing the RAM placement of the PDM generator is a mandatory prerequisite for release. Optimizing the interrupt locking strategy will significantly improve audio stability under load.
