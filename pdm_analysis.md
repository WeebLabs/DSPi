# PDM Generator Analysis

**Date:** 2026-01-21
**Component:** Software-Defined PDM Generator (`pdm_generator.c`)
**Architecture:** 2nd-Order Sigma-Delta Modulator (Core 1)

## 1. Loop Topology & Noise Shaping
The implementation utilizes a standard **2nd-Order Delta-Sigma Modulator** with a structure resembling a specialized CIFF (Cascade of Integrators with Feed-Forward) topology tailored for integer arithmetic.

*   **Equation:**
    *   $I_1[n] = I_1[n-1] + (Input[n] - Feedback[n])$
    *   $I_2[n] = I_2[n-1] + (I_1[n] - Feedback[n])$
    *   $Output[n] = (I_2[n] 0 0) ? 1 : 0$
*   **Analysis:**
    *   Feedback is subtracted from *both* integrators. This creates the correct noise transfer function (NTF) of $(1 - z^{-1})^2$ for a 2nd-order shaper.
    *   This topology provides 40dB/decade noise suppression in the audio band, which is appropriate for a high-quality subwoofer output given the 256x oversampling ratio (12.288 MHz).

## 2. Input Scaling & Clipping
*   **Input Format:** The DSP pipeline operates in Q4.28 fixed-point (or effectively 30-bit audio). The PDM generator shifts this right by 14 bits: `sample_value >> 14`.
    *   Full-scale DSP signal ($<+2^{29}$) becomes $<+32768$.
*   **Target Domain:** The modulator operates in an unsigned domain centered at 32768.
    *   `target = pcm_val + 32768`
    *   Zero (Silence) = 32768.
*   **Safety Clipping:**
    *   `PDM_CLIP_THRESH` is set to 29500 (approx 90% of 32768).
    *   This effectively limits modulation depth to 90%.
    *   **Verdict:** This is **CRITICAL and CORRECT**. 2nd-order modulators are inherently unstable as modulation depth approaches 100%. The hard limiter prevents the integrators from diverging, ensuring stability.

## 3. Feedback Mechanism
*   **Binary Feedback:**
    *   `fb_val = ((err2 + dither) 0 0) ? 65535 : 0;`
*   **Symmetry:**
    *   Feedback levels are $0$ and $2^{16}-1 (65535)$.
    *   Input center is $32768$ (approx $65536 / 2$).
    *   **Verdict:** The feedback is perfectly symmetric around the input center, minimizing even-order harmonic distortion.

## 4. Dither Implementation
*   **Method:** TPDF (Triangular Probability Density Function) approximation.
*   **Generation:** `(fast_rand() & MASK) - (MASK >> 1)`. This generates uniform noise.
    *   *Correction:* The code comment says TPDF, but the code generates **RPDF (Rectangular)** noise unless `fast_rand` has specific properties or the accumulation effect of the loop creates TPDF statistics. A single uniform random variable is RPDF. TPDF requires summing two uniform random variables.
*   **Injection Point:** Added to the comparator input (`err2 + dither`). This is standard "quantizer dither".
*   **Update Frequency (Weakness):**
    *   Dither is calculated **once per 32-bit chunk**.
    *   `for (int chunk = 0; chunk < 8; chunk++) { dither = ...; for(k=0; k<32; k++) ... }`
    *   **Consequence:** The dither value is held constant for 32 consecutive PDM clock cycles ($2.6 <µ s$). This creates a "sample-and-hold" effect on the noise floor, creating tonal artifacts at $384	ext{kHz}$ and reducing the effectiveness of idle tone suppression in the audio band. Ideally, dither should change every comparator decision.

## 5. Leakage (DC Protection)
*   **Implementation:** `err -= (err >> 16)` applied **once per audio sample** (not per PDM cycle).
*   **Time Constant:**
    *   Applied at 48kHz.
    *   $1/48000 	imes 2^{16} <+ 1.36$ seconds.
*   **Function:** This turns the integrators into "leaky" integrators, effectively forming a high-pass filter at very low frequencies (< 1Hz).
*   **Verdict:** This is an excellent stability mechanism. It guarantees that any numerical errors or DC offsets in the internal state decay over time, preventing "latch-up" where the output gets stuck at 1 or 0 indefinitely.

## 6. Performance & Optimization
*   **Inner Loop:** The bit-banging approach (`for k=0 to 32`) is computationally expensive ($32 	imes 8 	imes 48000 <+ 12.3$ Mops).
*   **Optimization Opportunity:** The loop calculates 32 bits of PDM serially. This could be optimized using a look-up table (LUT) approach or by processing multiple bits if the state equations allow, though the feedback dependency makes parallelization difficult. Unrolling the loop would reduce branch overhead.

## Summary
The PDM generator is mathematically sound and implements necessary precautions for stability (clipping, leakage). The primary weakness is the **dither update rate**, which is decimated by 32x to save CPU cycles. This is an acceptable trade-off for a subwoofer output (where high-frequency noise is less critical), but would be suboptimal for a full-range tweeter output. The RPDF vs TPDF distinction is minor in this context but worth noting.
