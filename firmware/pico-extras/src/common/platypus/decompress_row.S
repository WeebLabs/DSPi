#include "hardware/regs/addressmap.h"
#include "hardware/regs/sio.h"
.syntax unified
.cpu cortex-m0plus
.thumb

#ifndef VIDEO_DBI
#define FIRST_TEST_SHIFT 16
#define SECOND_TEST_SHIFT 8
#else
#define FIRST_TEST_SHIFT 6
#define SECOND_TEST_SHIFT 11
#endif

#define r_output r0
#define r_interps r1
#define r_input r2
#define r_top r3
#define r_bottom r4
#define r_tmp3 r5
#define r_tmp2 r6
#define r_tmp1 r7
#define r_output_end r8
#define r_save r12
#define r_row_delta r14

#define INTERP_OFFSET0(x) (x - SIO_INTERP0_ACCUM0_OFFSET)
#define INTERP_OFFSET1(x) (INTERP_OFFSET0(x) + SIO_INTERP1_ACCUM0_OFFSET - SIO_INTERP0_ACCUM0_OFFSET)

// input r_top     -- -- AB CD
//
// output r_top    AB CD AB CD
//        r_bottom AB CD AB CD
.macro duplicate_4_pixels tmp_a
    lsls  \tmp_a, r_top, #16
    uxth r_top, r_top
    orrs  r_top, \tmp_a
    mov  r_bottom, r_top
.endm

.macro do_222
#ifndef VIDEO_DBI
    lsls r_tmp1, #3
#else
    // done in interp
    // lsrs r_tmp1, #9
#endif
    str r_tmp1, [r_interps, #INTERP_OFFSET1(SIO_INTERP0_ACCUM1_OFFSET)]
    ldr r_tmp3, [r_interps, #INTERP_OFFSET1(SIO_INTERP0_PEEK_LANE1_OFFSET)]
    ldmia r_tmp3!, {r_tmp2, r_tmp1}
    add r_top, r_tmp2
    add r_bottom, r_tmp1
.endm

.macro do_555
#ifndef VIDEO_DBI
    rev16 r_tmp1, r_tmp1
    lsls r_tmp1, #3
#else
    // done in interp
    // lsrs r_tmp1, #8
#endif
    str r_tmp1, [r_interps, #INTERP_OFFSET1(SIO_INTERP0_ACCUM0_OFFSET)]
    str r_tmp1, [r_interps, #INTERP_OFFSET0(SIO_INTERP0_ACCUM0_OFFSET)]
    ldr r_tmp3, [r_interps, #INTERP_OFFSET1(SIO_INTERP0_PEEK_LANE0_OFFSET)]
    ldmia r_tmp3!, {r_tmp2, r_tmp1}
    add r_top, r_tmp2
    add r_bottom, r_tmp1
    ldr r_tmp3, [r_interps, #INTERP_OFFSET0(SIO_INTERP0_PEEK_LANE0_OFFSET)]
    ldmia r_tmp3!, {r_tmp2, r_tmp1}
#ifndef VIDEO_DBI
    lsls r_tmp2, #5
    lsls r_tmp1, #5
#else
    lsls r_tmp2, #6
    lsls r_tmp1, #6
#endif
    add r_top, r_tmp2
    add r_bottom, r_tmp1
    ldr r_tmp3, [r_interps, #INTERP_OFFSET0(SIO_INTERP0_PEEK_LANE1_OFFSET)]
    ldmia r_tmp3!, {r_tmp2, r_tmp1}
#ifndef VIDEO_DBI
    lsls r_tmp2, #10
    lsls r_tmp1, #10
#else
    lsls r_tmp2, #11
    lsls r_tmp1, #11
#endif
    add r_top, r_tmp2
    add r_bottom, r_tmp1
.endm

.macro shuffle_7_bytes_to_8 tmp_a tmp_b
#ifndef VIDEO_DBI
    ldr \tmp_b, =#0xff018401
    lsls \tmp_a, \tmp_b, #16
    ands \tmp_b, r_bottom
    ands \tmp_a, r_top
    eors r_bottom, \tmp_b
    eors r_top, \tmp_a

    lsrs \tmp_a, \tmp_a, #15
    orrs \tmp_b, \tmp_a
    lsrs \tmp_a, \tmp_b, #10
    lsls \tmp_b, #27
    lsls \tmp_a, #24
    orrs \tmp_b, \tmp_a
    orrs r_bottom, \tmp_b
#else
    ldr \tmp_b, =#0xff210821
    lsls \tmp_a, \tmp_b, #16
    ands \tmp_b, r_bottom
    ands \tmp_a, r_top
    eors r_bottom, \tmp_b
    eors r_top, \tmp_a

    // todo can we shave 1 more cycles to make cycles in the DBI code? a challenge to anyone who reads this!
    lsrs \tmp_a, #13
    orrs \tmp_b, \tmp_a
    lsrs \tmp_a, \tmp_b, #10
    eors \tmp_b, \tmp_a
    lsrs \tmp_a, \tmp_b, #12
    adcs \tmp_b, \tmp_b
    lsls \tmp_b, #24
    orrs r_bottom, \tmp_b
#endif
.endm

.macro write_output tmp_a
    mov   \tmp_a, r_row_delta
    str   r_bottom, [r_output, \tmp_a]
    stmia r_output!, {r_top}
.endm

.macro decompressor name data_section_prefix code_section_prefix

.section \data_section_prefix\().\name\().data

.global \name\()_5_table
\name\()_5_table:
.word    0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x00010000, 0x00010001, 0x00010000, 0x00000001
.word    0x00010001, 0x00000001, 0x00010000, 0x00000000, 0x00000000, 0x00010001, 0x00010000, 0x00010002
.word    0x00000000, 0x00010002, 0x00020000, 0x00010001, 0x00010000, 0x00020002, 0x00010000, 0x00010000
.word    0x00000001, 0x00000001, 0x00010001, 0x00000002, 0x00010001, 0x00000000, 0x00020001, 0x00000001
.word    0x00020000, 0x00020001, 0x00000001, 0x00000002, 0x00020002, 0x00000001, 0x00020001, 0x00000000
.word    0x00000000, 0x00000002, 0x00000001, 0x00010002, 0x00010000, 0x00020001, 0x00020000, 0x00010000
.word    0x00020000, 0x00020002, 0x00000000, 0x00020002, 0x00010002, 0x00000002, 0x00010000, 0x00020003
.word    0x00000000, 0x00020003, 0x00010000, 0x00030003, 0x00010000, 0x00000002, 0x00000001, 0x00010001

.global \name\()_222_table
\name\()_222_table:
#ifndef VIDEO_DBI
.word    0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x00010000, 0x00010001, 0x00010000, 0x00000001
.word    0x00000000, 0x00000020, 0x00000000, 0x00000021, 0x00010000, 0x00010021, 0x00010000, 0x00000021
.word    0x00200000, 0x00200020, 0x00200000, 0x00200021, 0x00210000, 0x00210021, 0x00210000, 0x00200021
.word    0x00200000, 0x00000020, 0x00200000, 0x00000021, 0x00210000, 0x00010021, 0x00210000, 0x00000021
.word    0x00000000, 0x00000400, 0x00000000, 0x00000401, 0x00010000, 0x00010401, 0x00010000, 0x00000401
.word    0x00000000, 0x00000420, 0x00000000, 0x00000421, 0x00010000, 0x00010421, 0x00010000, 0x00000421
.word    0x00200000, 0x00200420, 0x00200000, 0x00200421, 0x00210000, 0x00210421, 0x00210000, 0x00200421
.word    0x00200000, 0x00000420, 0x00200000, 0x00000421, 0x00210000, 0x00010421, 0x00210000, 0x00000421
.word    0x04000000, 0x04000400, 0x04000000, 0x04000401, 0x04010000, 0x04010401, 0x04010000, 0x04000401
.word    0x04000000, 0x04000420, 0x04000000, 0x04000421, 0x04010000, 0x04010421, 0x04010000, 0x04000421
.word    0x04200000, 0x04200420, 0x04200000, 0x04200421, 0x04210000, 0x04210421, 0x04210000, 0x04200421
.word    0x04200000, 0x04000420, 0x04200000, 0x04000421, 0x04210000, 0x04010421, 0x04210000, 0x04000421
.word    0x04000000, 0x00000400, 0x04000000, 0x00000401, 0x04010000, 0x00010401, 0x04010000, 0x00000401
.word    0x04000000, 0x00000420, 0x04000000, 0x00000421, 0x04010000, 0x00010421, 0x04010000, 0x00000421
.word    0x04200000, 0x00200420, 0x04200000, 0x00200421, 0x04210000, 0x00210421, 0x04210000, 0x00200421
.word    0x04200000, 0x00000420, 0x04200000, 0x00000421, 0x04210000, 0x00010421, 0x04210000, 0x00000421
#else
.word    0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x00010000, 0x00010001, 0x00010000, 0x00000001
.word    0x00000000, 0x00000040, 0x00000000, 0x00000041, 0x00010000, 0x00010041, 0x00010000, 0x00000041
.word    0x00400000, 0x00400040, 0x00400000, 0x00400041, 0x00410000, 0x00410041, 0x00410000, 0x00400041
.word    0x00400000, 0x00000040, 0x00400000, 0x00000041, 0x00410000, 0x00010041, 0x00410000, 0x00000041
.word    0x00000000, 0x00000800, 0x00000000, 0x00000801, 0x00010000, 0x00010801, 0x00010000, 0x00000801
.word    0x00000000, 0x00000840, 0x00000000, 0x00000841, 0x00010000, 0x00010841, 0x00010000, 0x00000841
.word    0x00400000, 0x00400840, 0x00400000, 0x00400841, 0x00410000, 0x00410841, 0x00410000, 0x00400841
.word    0x00400000, 0x00000840, 0x00400000, 0x00000841, 0x00410000, 0x00010841, 0x00410000, 0x00000841
.word    0x08000000, 0x08000800, 0x08000000, 0x08000801, 0x08010000, 0x08010801, 0x08010000, 0x08000801
.word    0x08000000, 0x08000840, 0x08000000, 0x08000841, 0x08010000, 0x08010841, 0x08010000, 0x08000841
.word    0x08400000, 0x08400840, 0x08400000, 0x08400841, 0x08410000, 0x08410841, 0x08410000, 0x08400841
.word    0x08400000, 0x08000840, 0x08400000, 0x08000841, 0x08410000, 0x08010841, 0x08410000, 0x08000841
.word    0x08000000, 0x00000800, 0x08000000, 0x00000801, 0x08010000, 0x00010801, 0x08010000, 0x00000801
.word    0x08000000, 0x00000840, 0x08000000, 0x00000841, 0x08010000, 0x00010841, 0x08010000, 0x00000841
.word    0x08400000, 0x00400840, 0x08400000, 0x00400841, 0x08410000, 0x00410841, 0x08410000, 0x00400841
.word    0x08400000, 0x00000840, 0x08400000, 0x00000841, 0x08410000, 0x00010841, 0x08410000, 0x00000841
#endif

.section \code_section_prefix\().\name\().code, "ax"

.global \name
.type \name,%function
.thumb_func
//const uint8_t* \name(uint32_t *d0, uint32_t *d1, const uint8_t *s, uint32_t w);
\name:
    push {r4, r5, r6, r7, lr}
    mov r4, r8
    push {r4}
    lsls r3, #1
    adds r3, r0
    // done (near)
    bcs 0f
    // todo assert r_output is r0
    subs r1, r_output
    mov r_row_delta, r1
    // todo assert r_interps is r1
    ldr r_interps, =#SIO_BASE + SIO_INTERP0_ACCUM0_OFFSET
    // todo assert r_source is r2
    mov r_output_end, r3

\name\()_rem_0: // r_top has 0 valid bytes
    ldmia r_input!, {r_top}

\name\()_rem_4: // r_top has 4 valid bytes LSB first
    lsrs  r_tmp1, r_top, #FIRST_TEST_SHIFT
    bcs  2f
    mov r_save, r_top // todo we could skip this (in case we hit 4 byte path anyway) if we have spare reg, or just move duplicate_4_pixels into both branches
    duplicate_4_pixels r_tmp2
    lsrs  r_tmp2, r_tmp1, #SECOND_TEST_SHIFT
    bcc  1f
    do_555
    write_output r_tmp2
    mov  r_top, r_save
    cmp   r_output, r_output_end
    blt  \name\()_rem_0
0:
    b    \name\()_done
1:
    do_222
    write_output r_tmp2
    mov  r_top, r_save
    cmp   r_output, r_output_end
    blt  \name\()_rem_1
    b    \name\()_done
2:
    //bkpt #0
    ldmia r_input!, {r_bottom}
    mov  r_save, r_bottom
    shuffle_7_bytes_to_8 r_tmp2, r_tmp3
    write_output r_tmp2
    mov r_top, r_save
    cmp   r_output, r_output_end
    bge   \name\()_done
    // fall thru

\name\()_rem_1: // r_remaining_bits has 1 byte remaining (in the MSB)
    ldmia r_input!, {r_tmp2}
    lsrs r_top, #24
    lsls r_tmp1, r_tmp2, #8
    orrs r_top, r_tmp1

    lsrs  r_tmp1, r_top, #FIRST_TEST_SHIFT
    bcs  2f
    mov  r_save, r_tmp2
    duplicate_4_pixels r_tmp2
    lsrs  r_tmp2, r_tmp1, #SECOND_TEST_SHIFT
    bcc  1f
    do_555
    write_output r_tmp2
    mov r_top, r_save
    cmp   r_output, r_output_end
    blt \name\()_rem_1
    b    \name\()_done
1:
    do_222
    write_output r_tmp2
    mov r_top, r_save
    cmp   r_output, r_output_end
    blt \name\()_rem_2
    b    \name\()_done
2:
    //bkpt #0
    // r_top has 4, r_save has 1
    ldmia r_input!, {r_tmp1}
    lsrs r_bottom, r_tmp2, #24
    lsls r_tmp2, r_tmp1, #8
    orrs r_bottom, r_tmp2

    shuffle_7_bytes_to_8 r_tmp2, r_tmp3
    write_output r_tmp2

    mov r_top, r_tmp1
    cmp   r_output, r_output_end
    blt \name\()_rem_2

\name\()_done:
    pop {r3, r4, r5, r6, r7}
    mov r8, r3
    pop {pc}

\name\()_rem_2: // r_remaining_bits has 2 bytes remaining (in the MSB)
    ldmia r_input!, {r_tmp2}
    lsrs r_top, #16
    lsls r_tmp1, r_tmp2, #16
    orrs r_top, r_tmp1

    lsrs  r_tmp1, r_top, #FIRST_TEST_SHIFT
    bcs  2f
    mov  r_save, r_tmp2
    duplicate_4_pixels r_tmp2
    lsrs  r_tmp2, r_tmp1, #SECOND_TEST_SHIFT
    bcc  1f
    do_555
    write_output r_tmp2
    mov r_top, r_save
    cmp   r_output, r_output_end
    blt \name\()_rem_2
    b \name\()_done
1:
    do_222
    write_output r_tmp2
    mov r_top, r_save
    cmp   r_output, r_output_end
    blt \name\()_rem_3
    b \name\()_done
2:
    //bkpt #0
    ldmia r_input!, {r_tmp1}
    lsrs r_bottom, r_tmp2, #16
    lsls r_tmp2, r_tmp1, #16
    orrs r_bottom, r_tmp2

    shuffle_7_bytes_to_8 r_tmp2, r_tmp3
    write_output r_tmp2
    mov r_top, r_tmp1
    cmp   r_output, r_output_end
    bge \name\()_done
    // fall thru

\name\()_rem_3: // r_remaining_bits has 3 bytes remaining (in the MSB)
    ldmia r_input!, {r_tmp2}
    lsrs r_top, #8
    lsls r_tmp1, r_tmp2, #24
    orrs r_top, r_tmp1

    lsrs  r_tmp1, r_top, #FIRST_TEST_SHIFT
    bcs  2f
    mov  r_save, r_tmp2
    duplicate_4_pixels r_tmp2
    lsrs  r_tmp2, r_tmp1, #SECOND_TEST_SHIFT
    bcc  1f
    do_555
    write_output r_tmp2
    mov r_top, r_save
    cmp   r_output, r_output_end
    blt \name\()_rem_3
    b \name\()_done
1:
    do_222
    write_output r_tmp2
    mov r_top, r_save
    cmp   r_output, r_output_end
    bge \name\()_done2
    b \name\()_rem_4
2:
    //bkpt #0
    ldmia r_input!, {r_tmp1}
    lsrs r_bottom, r_tmp2, #8
    lsls r_tmp2, r_tmp1, #24
    orrs r_bottom, r_tmp2

    shuffle_7_bytes_to_8 r_tmp2, r_tmp3
    write_output r_tmp2
    mov r_top, r_tmp1
    cmp   r_output, r_output_end
    bge \name\()_done2
    b \name\()_rem_4
\name\()_done2:
    pop {r3, r4, r5, r6, r7}
    mov r8, r3
    mov r0, r_input
    pop {pc}

.endm

// put one decompressor in each scratch bank for use by each core
decompressor platypus_decompress_row_asm_a .scratch_x, .scratch_x
decompressor platypus_decompress_row_asm_b .scratch_y, .scratch_y