/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"

.syntax unified
.cpu cortex-m0plus
.thumb

#define AUDIO_UPSAMPLE_SCALE_BITS 12
.align 2
.section .time_critical.audio_upsample
.global audio_upsample
.type audio_upsample,%function
// step is fraction of an input sample per output sample * (1 << AUDIO_UPSAMPLE_SCALE_BITS) and should be < (1 << AUDIO_UPSAMPLE_SCALE_BITS) ... i.e. we we are upsampling (otherwise results are undefined)
// void audio_upsample(int16_t *input, int16_t *output, int count, uint32_t step)
.thumb_func
audio_upsample:
    push    {r4, r5, r6, r7, lr}
    lsls    r2, #1
    mov     ip, r1
    add     ip, r2
    ldr     r6, =#SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET
    // interp_configure_with_signed_and_blend
    ldr     r4, =# ((AUDIO_UPSAMPLE_SCALE_BITS - 1) << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB) | (1 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) | ((24 - AUDIO_UPSAMPLE_SCALE_BITS) << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB) | SIO_INTERP0_CTRL_LANE0_BLEND_BITS
    str     r4, [r6, #SIO_INTERP0_CTRL_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    // interp_configure_with_signed_and_cross_input
    ldr     r4, =# ((AUDIO_UPSAMPLE_SCALE_BITS - 8) << SIO_INTERP0_CTRL_LANE1_SHIFT_LSB) | (0 << SIO_INTERP0_CTRL_LANE1_MASK_LSB_LSB) | (7 << SIO_INTERP0_CTRL_LANE1_MASK_MSB_LSB) | SIO_INTERP0_CTRL_LANE1_SIGNED_BITS | SIO_INTERP0_CTRL_LANE1_CROSS_INPUT_BITS
    str     r4, [r6, #SIO_INTERP0_CTRL_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    str     r0, [r6, #SIO_INTERP0_BASE2_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    movs    r0, #0
    str     r0, [r6, #SIO_INTERP0_ACCUM0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    mov     r7, r0 // last_offset = 0 (invalid)
    movs    r2, #2

    // r0 0
    // r1 output
    // r2 2
    // r3 step
    // r4 temp
    // r5 temp
    // r6 interp_hw
    // r7 last_offset
    // ip end
    b       4f

1: // aligned
    ldr     r5, [r4]
    str     r5, [r6, #SIO_INTERP0_BASE_1AND0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
2: // unchanged sample ptr
    ldr     r4, [r6, #SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1]
    str     r3, [r6, #SIO_INTERP0_ACCUM0_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    add     r1, r2
    cmp     r1, ip
    beq     5f
3: // next sample
    ldr     r4, [r6, #SIO_INTERP0_PEEK_FULL_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    cmp     r4, r7
    beq     2b
    mov     r7, r4
    tst     r4, r2
    beq     1b
    ldrsh   r5, [r4, r0]
    str     r5, [r6, #SIO_INTERP0_BASE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    ldrsh   r4, [r4, r2]
    str     r4, [r6, #SIO_INTERP0_BASE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    ldr     r4, [r6, #SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1]
    str     r3, [r6, #SIO_INTERP0_ACCUM0_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    add     r1, r2
4:
    cmp     r1, ip
    bne     3b
5:
    pop     {r4, r5, r6, r7, pc}

.align 2
.section .time_critical.audio_upsample_words
.global audio_upsample_words
.type audio_upsample_words,%function
// step is fraction of an input sample per output sample * (1 << AUDIO_UPSAMPLE_SCALE_BITS) and should be < (1 << AUDIO_UPSAMPLE_SCALE_BITS) ... i.e. we we are upsampling (otherwise results are undefined)
// void audio_upsample(int16_t *input, int16_t *output_aligned, int output_word_count, uint32_t step)
.thumb_func
audio_upsample_words:
    push    {r4, r5, r6, r7, lr}
    lsls    r2, #2
    mov     ip, r1
    add     ip, r2
    ldr     r6, =#SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET
    // interp_configure_with_blend
    ldr     r4, =# ((AUDIO_UPSAMPLE_SCALE_BITS - 1) << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB) | (1 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) | ((24 -AUDIO_UPSAMPLE_SCALE_BITS) << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB) | SIO_INTERP0_CTRL_LANE0_BLEND_BITS
    str     r4, [r6, #SIO_INTERP0_CTRL_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    // interp_configure_with_signed_and_cross_input
    ldr     r4, =# ((AUDIO_UPSAMPLE_SCALE_BITS - 8) << SIO_INTERP0_CTRL_LANE1_SHIFT_LSB) | (0 << SIO_INTERP0_CTRL_LANE1_MASK_LSB_LSB) | (7 << SIO_INTERP0_CTRL_LANE1_MASK_MSB_LSB) | SIO_INTERP0_CTRL_LANE1_SIGNED_BITS | SIO_INTERP0_CTRL_LANE1_CROSS_INPUT_BITS
    str     r4, [r6, #SIO_INTERP0_CTRL_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    str     r0, [r6, #SIO_INTERP0_BASE2_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    movs    r0, #0
    str     r0, [r6, #SIO_INTERP0_ACCUM0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    mov     r7, r0 // last_offset = 0 (invalid)
    movs    r2, #2

    // r0 0
    // r1 output
    // r2 2
    // r3 step
    // r4 temp
    // r5 temp
    // r6 interp_hw
    // r7 last_offset
    // ip end
    b       4f

1: // aligned A
    ldr     r5, [r4]
    str     r5, [r6, #SIO_INTERP0_BASE_1AND0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
2: // unchanged sample ptr A
    ldr     r4, [r6, #SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    // output A
    strh    r4, [r1]
    str     r3, [r6, #SIO_INTERP0_ACCUM0_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]

    // next sample B
    ldr     r4, [r6, #SIO_INTERP0_PEEK_FULL_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    cmp     r4, r7
    beq     6f

    mov     r7, r4
    tst     r4, r2
    bne     7f

8:
    // aligned B
    ldr     r5, [r4]
    str     r5, [r6, #SIO_INTERP0_BASE_1AND0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]

6:  // unchanged sample ptr B
    ldr     r4, [r6, #SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1, r2]
    str     r3, [r6, #SIO_INTERP0_ACCUM0_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    adds    r1, #4
    cmp     r1, ip
    beq     5f

3: // next sample A
    ldr     r4, [r6, #SIO_INTERP0_PEEK_FULL_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    cmp     r4, r7
    beq     2b
    mov     r7, r4
    tst     r4, r2
    beq     1b
    ldrsh   r5, [r4, r0]
    str     r5, [r6, #SIO_INTERP0_BASE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    ldrsh   r4, [r4, r2]
    str     r4, [r6, #SIO_INTERP0_BASE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    ldr     r4, [r6, #SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1]

    str     r3, [r6, #SIO_INTERP0_ACCUM0_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]

    // next sample B
    ldr     r4, [r6, #SIO_INTERP0_PEEK_FULL_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    cmp     r4, r7
    beq     6b
    mov     r7, r4
    tst     r4, r2
    beq     8b
7:  // unalignedb
    ldrsh   r5, [r4, r0]
    str     r5, [r6, #SIO_INTERP0_BASE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    ldrsh   r4, [r4, r2]
    str     r4, [r6, #SIO_INTERP0_BASE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    ldr     r4, [r6, #SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1, r2]
    str     r3, [r6, #SIO_INTERP0_ACCUM0_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    adds    r1, #4

4:
    cmp     r1, ip
    bne     3b

5:
    pop     {r4, r5, r6, r7, pc}

.global audio_upsample_double
.type audio_upsample_double,%function
// step is fraction of an input sample per output sample * (1 << AUDIO_UPSAMPLE_SCALE_BITS) and should be < (1 << AUDIO_UPSAMPLE_SCALE_BITS) ... i.e. we we are upsampling (otherwise results are undefined)
// void audio_upsample(int16_t *input, int16_t *output, int count, uint32_t step)
.thumb_func
audio_upsample_double:
    push    {r4, r5, r6, r7, lr}
    lsls    r2, #2
    mov     ip, r1
    add     ip, r2
    ldr     r6, =#SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET
    // interp_configure_with_signed_and_blend
    ldr     r4, =# ((AUDIO_UPSAMPLE_SCALE_BITS - 1) << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB) | (1 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) | ((24 - AUDIO_UPSAMPLE_SCALE_BITS) << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB) | SIO_INTERP0_CTRL_LANE0_BLEND_BITS
    str     r4, [r6, #SIO_INTERP0_CTRL_LANE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    // interp_configure_with_signed_and_cross_input
    ldr     r4, =# ((AUDIO_UPSAMPLE_SCALE_BITS - 8) << SIO_INTERP0_CTRL_LANE1_SHIFT_LSB) | (0 << SIO_INTERP0_CTRL_LANE1_MASK_LSB_LSB) | (7 << SIO_INTERP0_CTRL_LANE1_MASK_MSB_LSB) | SIO_INTERP0_CTRL_LANE1_SIGNED_BITS | SIO_INTERP0_CTRL_LANE1_CROSS_INPUT_BITS
    str     r4, [r6, #SIO_INTERP0_CTRL_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    str     r0, [r6, #SIO_INTERP0_BASE2_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    movs    r0, #0
    str     r0, [r6, #SIO_INTERP0_ACCUM0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    mov     r7, r0 // last_offset = 0 (invalid)
    movs    r2, #2

    // r0 0
    // r1 output
    // r2 2
    // r3 step
    // r4 temp
    // r5 temp
    // r6 interp_hw
    // r7 last_offset
    // ip end
    b       4f

1: // aligned
    ldr     r5, [r4]
    str     r5, [r6, #SIO_INTERP0_BASE_1AND0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
2: // unchanged sample ptr
    ldr     r4, [r6, #SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1]
    str     r3, [r6, #SIO_INTERP0_ACCUM0_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1, #2]
    add     r1, r2
    add     r1, r2
    cmp     r1, ip
    beq     5f
3: // next sample
    ldr     r4, [r6, #SIO_INTERP0_PEEK_FULL_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    cmp     r4, r7
    beq     2b
    mov     r7, r4
    tst     r4, r2
    beq     1b
    ldrsh   r5, [r4, r0]
    str     r5, [r6, #SIO_INTERP0_BASE0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    ldrsh   r4, [r4, r2]
    str     r4, [r6, #SIO_INTERP0_BASE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    ldr     r4, [r6, #SIO_INTERP0_PEEK_LANE1_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1]
    str     r3, [r6, #SIO_INTERP0_ACCUM0_ADD_OFFSET - SIO_INTERP0_ACCUM0_OFFSET]
    strh    r4, [r1, #2]
    add     r1, r2
    add     r1, r2
4:
    cmp     r1, ip
    bne     3b
5:
    pop     {r4, r5, r6, r7, pc}
