.syntax unified
.cpu cortex-m0plus
.thumb

// Offsets (struct has bool bypass at end, padded to 32 bytes)
#define OFF_B0 0
#define OFF_B1 4
#define OFF_B2 8
#define OFF_A1 12
#define OFF_A2 16
#define OFF_S1 20
#define OFF_S2 24
#define OFF_BYPASS 28
#define SIZE_BQ 32

// ============================================================================
// Per-sample biquad processing (legacy, kept for compatibility)
// ============================================================================

.section .time_critical.dsp_process_channel, "ax"
.global dsp_process_channel
.type dsp_process_channel, %function

// int32_t dsp_process_channel(Biquad *biquads, int32_t input, uint8_t channel)
// r0: biquads pointer
// r1: input sample (x)
// r2: channel index

dsp_process_channel:
    push {r4-r7, lr}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    push {r4-r7}

    // Load count
    ldr r3, =channel_band_counts
    ldrb r3, [r3, r2]
    cmp r3, #0
    beq .Ldone
    mov r8, r3          // r8 = loop count

    // We keep x in r1.
    // We use r9, r10, r11 for s1, s2, result storage to free low regs for math.

.Lloop:
    // Bypass check - load single byte flag
    ldrb r2, [r0, #OFF_BYPASS]
    cmp r2, #0
    bne .Lskip

    // Load s1, s2 to high regs to free r0-r7
    ldr r2, [r0, #OFF_S1]
    mov r9, r2          // r9 = s1
    ldr r3, [r0, #OFF_S2]
    mov r10, r3         // r10 = s2

    // Pre-decompose x (r1) into r6 (xh), r7 (xl)
    asrs r6, r1, #16
    uxth r7, r1

    // --- T0 = b0 * x ---
    ldr r2, [r0, #OFF_B0]
    // Inline Mul (r2 = coeff, r6=xh, r7=xl)
    asrs r3, r2, #16    // ch
    uxth r2, r2         // cl

    mov r4, r3
    muls r4, r6         // high = ch * xh
    lsls r4, r4, #4

    muls r3, r7         // mid1 = ch * xl
    muls r2, r6         // mid2 = cl * xh
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r2, r4, r3     // r2 = T0

    // result = T0 + s1
    mov r3, r9          // s1
    adds r2, r2, r3
    mov r11, r2         // r11 = result

    // --- T1 = b1 * x ---
    ldr r2, [r0, #OFF_B1]
    asrs r3, r2, #16
    uxth r2, r2

    mov r4, r3
    muls r4, r6
    lsls r4, r4, #4

    muls r3, r7
    muls r2, r6
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r2, r4, r3     // r2 = T1

    // Save T1 in r9 (reuse s1 register as temp)
    mov r9, r2

    // --- T3 = b2 * x ---
    // (Doing T3 before T2/T4 to keep using r6/r7 xh/xl)
    ldr r2, [r0, #OFF_B2]
    asrs r3, r2, #16
    uxth r2, r2

    mov r4, r3
    muls r4, r6
    lsls r4, r4, #4

    muls r3, r7
    muls r2, r6
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r2, r4, r3     // r2 = T3

    // Save T3 in r10 (reuse s2 register as temp)
    mov r10, r2

    // Now we need multiplies involving 'result' (r11).
    // Decompose result: r6 (rh), r7 (rl)
    mov r1, r11         // Move result to r1
    asrs r6, r1, #16
    uxth r7, r1

    // --- T2 = a1 * result ---
    ldr r2, [r0, #OFF_A1]
    asrs r3, r2, #16
    uxth r2, r2

    mov r4, r3
    muls r4, r6         // ch * rh
    lsls r4, r4, #4

    muls r3, r7         // ch * rl
    muls r2, r6         // cl * rh
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r2, r4, r3     // r2 = T2

    // Update s1 = T1 - T2 + old_s2
    // T1 is in r9. old_s2 needs to be loaded again from memory (we overwrote r10)
    mov r3, r9          // T1
    subs r3, r3, r2     // T1 - T2
    ldr r4, [r0, #OFF_S2] // Reload old s2 from memory
    adds r3, r3, r4
    str r3, [r0, #OFF_S1] // Store new s1

    // --- T4 = a2 * result ---
    ldr r2, [r0, #OFF_A2]
    asrs r3, r2, #16
    uxth r2, r2

    mov r4, r3
    muls r4, r6
    lsls r4, r4, #4

    muls r3, r7
    muls r2, r6
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r2, r4, r3     // r2 = T4

    // Update s2 = T3 - T4
    // T3 is in r10.
    mov r3, r10
    subs r3, r3, r2
    str r3, [r0, #OFF_S2] // Store new s2

    // x = result (already in r1)

.Lskip:
    adds r0, #SIZE_BQ
    mov r3, r8
    subs r3, #1
    mov r8, r3
    bne .Lloop

.Ldone:
    mov r0, r1          // Return result
    pop {r4-r7}
    mov r11, r7
    mov r10, r6
    mov r9, r5
    mov r8, r4
    pop {r4-r7, pc}


// ============================================================================
// Block-based biquad processing — optimized for RP2040 (Cortex-M0+)
//
// Processes all samples through each biquad before moving to the next band.
// Key optimizations vs C fast_mul_q28 version:
//   - s1/s2 kept in high registers across entire sample loop (no struct access)
//   - x decomposition (xh/xl) shared across b0, b1, b2 multiplies
//   - y decomposition (yh/yl) shared across a1, a2 multiplies
//   - No function call overhead for fast_mul_q28 (inlined 5 times)
//   - b2*x saved in r12, avoiding need to reload s2 from struct
// ============================================================================

.section .time_critical.dsp_process_channel_block, "ax"
.global dsp_process_channel_block
.type dsp_process_channel_block, %function

// void dsp_process_channel_block(Biquad *biquads, int32_t *samples,
//                                uint32_t count, uint8_t channel)
// r0: biquads pointer
// r1: samples pointer
// r2: sample count
// r3: channel index
//
// Register allocation (inner sample loop):
//   r0  = biquad pointer (constant — coefficient loads)
//   r1  = current sample value (x in, y out)
//   r2-r5 = scratch for multiply
//   r6  = xh/yh (high half of decomposed operand)
//   r7  = xl/yl (low half of decomposed operand)
//   r8  = inner loop counter (samples remaining)
//   r9  = s1 (biquad state, carried across samples)
//   r10 = s2 (biquad state, carried across samples)
//   r11 = samples pointer (advancing by 4 each iteration)
//   r12 = temp save for b2*x

dsp_process_channel_block:
    push {r4-r7, lr}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    push {r4-r7}

    // Load band count from channel_band_counts[channel]
    ldr r4, =channel_band_counts
    ldrb r3, [r4, r3]
    cmp r3, #0
    beq .Lblk_done

    // Save loop-invariant values on stack for outer band loop
    // sp+0 = samples_ptr, sp+4 = sample_count, sp+8 = band_count
    push {r1, r2, r3}

.Lblk_band_loop:
    // r0 = current biquad pointer
    // Check bypass flag
    ldrb r1, [r0, #OFF_BYPASS]
    cmp r1, #0
    bne .Lblk_band_next

    // Load s1, s2 into high registers (carried across all samples)
    ldr r1, [r0, #OFF_S1]
    mov r9, r1
    ldr r1, [r0, #OFF_S2]
    mov r10, r1

    // Setup inner sample loop
    ldr r1, [sp, #0]       // samples base pointer
    mov r11, r1
    ldr r1, [sp, #4]       // sample count
    mov r8, r1

    // ---- Inner sample loop ----
.Lblk_sample_loop:
    // Load x = samples[i]
    mov r5, r11
    ldr r1, [r5, #0]

    // Decompose x: r6 = xh (signed), r7 = xl (unsigned)
    asrs r6, r1, #16
    uxth r7, r1

    // --- y = b0 * x + s1 ---
    ldr r2, [r0, #OFF_B0]
    asrs r3, r2, #16
    uxth r2, r2
    mov r4, r3
    muls r4, r6             // ch * xh
    lsls r4, r4, #4
    muls r3, r7             // ch * xl
    muls r2, r6             // cl * xh
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r2, r4, r3         // r2 = b0*x
    mov r3, r9              // s1
    adds r1, r2, r3         // r1 = y = b0*x + s1

    // --- T1 = b1 * x --- (xh=r6, xl=r7 still valid)
    ldr r2, [r0, #OFF_B1]
    asrs r3, r2, #16
    uxth r2, r2
    mov r4, r3
    muls r4, r6
    lsls r4, r4, #4
    muls r3, r7
    muls r2, r6
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r5, r4, r3         // r5 = b1*x (saved for s1 update)

    // --- T3 = b2 * x --- (xh=r6, xl=r7 still valid)
    ldr r2, [r0, #OFF_B2]
    asrs r3, r2, #16
    uxth r2, r2
    mov r4, r3
    muls r4, r6
    lsls r4, r4, #4
    muls r3, r7
    muls r2, r6
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r4, r4, r3         // r4 = b2*x
    mov r12, r4             // Save b2*x in r12

    // Decompose y: r6 = yh, r7 = yl (r1 = y)
    asrs r6, r1, #16
    uxth r7, r1

    // --- T2 = a1 * y ---
    ldr r2, [r0, #OFF_A1]
    asrs r3, r2, #16
    uxth r2, r2
    mov r4, r3
    muls r4, r6
    lsls r4, r4, #4
    muls r3, r7
    muls r2, r6
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r2, r4, r3         // r2 = a1*y

    // s1_new = b1*x - a1*y + s2
    subs r5, r5, r2         // b1*x - a1*y
    mov r3, r10             // s2
    adds r5, r5, r3
    mov r9, r5              // r9 = new s1

    // --- T4 = a2 * y --- (yh=r6, yl=r7 still valid)
    ldr r2, [r0, #OFF_A2]
    asrs r3, r2, #16
    uxth r2, r2
    mov r4, r3
    muls r4, r6
    lsls r4, r4, #4
    muls r3, r7
    muls r2, r6
    adds r3, r3, r2
    asrs r3, r3, #12
    adds r2, r4, r3         // r2 = a2*y

    // s2_new = b2*x - a2*y
    mov r3, r12             // b2*x
    subs r3, r3, r2
    mov r10, r3             // r10 = new s2

    // Store y to samples[i] and advance pointer
    mov r4, r11
    str r1, [r4, #0]
    adds r4, #4
    mov r11, r4

    // Decrement sample counter
    mov r3, r8
    subs r3, #1
    mov r8, r3
    bne .Lblk_sample_loop

    // ---- End inner sample loop ----

    // Write back s1, s2 to biquad struct
    mov r1, r9
    str r1, [r0, #OFF_S1]
    mov r1, r10
    str r1, [r0, #OFF_S2]

.Lblk_band_next:
    // Advance to next biquad
    adds r0, #SIZE_BQ

    // Decrement band counter
    ldr r1, [sp, #8]
    subs r1, #1
    str r1, [sp, #8]
    bne .Lblk_band_loop

    // Clean up stack (samples_ptr, sample_count, band_count)
    add sp, #12

.Lblk_done:
    pop {r4-r7}
    mov r11, r7
    mov r10, r6
    mov r9, r5
    mov r8, r4
    pop {r4-r7, pc}
