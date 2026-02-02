.syntax unified
.cpu cortex-m0plus
.thumb

.section .time_critical.dsp_process_channel, "ax"
.global dsp_process_channel
.type dsp_process_channel, %function

// int32_t dsp_process_channel(Biquad *biquads, int32_t input, uint8_t channel)
// r0: biquads pointer
// r1: input sample (x)
// r2: channel index

// Offsets
#define OFF_B0 0
#define OFF_B1 4
#define OFF_B2 8
#define OFF_A1 12
#define OFF_A2 16
#define OFF_S1 20
#define OFF_S2 24
#define SIZE_BQ 28

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
    // Bypass check
    ldr r2, [r0, #OFF_A1]
    ldr r3, [r0, #OFF_A2]
    orrs r2, r3
    beq .Lskip

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