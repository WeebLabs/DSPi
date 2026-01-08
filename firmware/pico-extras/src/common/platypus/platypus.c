#include <stdio.h>

#include "pico.h"
#include "platypus.h"
#include "pico/scanvideo.h"
#if PICO_ON_DEVICE
#include "hardware/interp.h"
#endif

#ifdef PLATYPUS_565
#define PIXEL_RSHIFT 0
#define PIXEL_GSHIFT 6
#define PIXEL_BSHIFT 11
#else
#define PIXEL_RSHIFT 0
#define PIXEL_GSHIFT 5
#define PIXEL_BSHIFT 10
#endif
#if PICO_NO_HARDWARE

const static uint32_t row_5_table[] = {
        0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x00010000, 0x00010001, 0x00010000, 0x00000001,
        0x00010001, 0x00000001, 0x00010000, 0x00000000, 0x00000000, 0x00010001, 0x00010000, 0x00010002,
        0x00000000, 0x00010002, 0x00020000, 0x00010001, 0x00010000, 0x00020002, 0x00010000, 0x00010000,
        0x00000001, 0x00000001, 0x00010001, 0x00000002, 0x00010001, 0x00000000, 0x00020001, 0x00000001,
        0x00020000, 0x00020001, 0x00000001, 0x00000002, 0x00020002, 0x00000001, 0x00020001, 0x00000000,
        0x00000000, 0x00000002, 0x00000001, 0x00010002, 0x00010000, 0x00020001, 0x00020000, 0x00010000,
        0x00020000, 0x00020002, 0x00000000, 0x00020002, 0x00010002, 0x00000002, 0x00010000, 0x00020003,
        0x00000000, 0x00020003, 0x00010000, 0x00030003, 0x00010000, 0x00000002, 0x00000001, 0x00010001,
};

#ifndef PLATYPUS_565
const static uint32_t row_222_table[] = {
        0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x00010000, 0x00010001, 0x00010000, 0x00000001,
        0x00000000, 0x00000020, 0x00000000, 0x00000021, 0x00010000, 0x00010021, 0x00010000, 0x00000021,
        0x00200000, 0x00200020, 0x00200000, 0x00200021, 0x00210000, 0x00210021, 0x00210000, 0x00200021,
        0x00200000, 0x00000020, 0x00200000, 0x00000021, 0x00210000, 0x00010021, 0x00210000, 0x00000021,
        0x00000000, 0x00000400, 0x00000000, 0x00000401, 0x00010000, 0x00010401, 0x00010000, 0x00000401,
        0x00000000, 0x00000420, 0x00000000, 0x00000421, 0x00010000, 0x00010421, 0x00010000, 0x00000421,
        0x00200000, 0x00200420, 0x00200000, 0x00200421, 0x00210000, 0x00210421, 0x00210000, 0x00200421,
        0x00200000, 0x00000420, 0x00200000, 0x00000421, 0x00210000, 0x00010421, 0x00210000, 0x00000421,
        0x04000000, 0x04000400, 0x04000000, 0x04000401, 0x04010000, 0x04010401, 0x04010000, 0x04000401,
        0x04000000, 0x04000420, 0x04000000, 0x04000421, 0x04010000, 0x04010421, 0x04010000, 0x04000421,
        0x04200000, 0x04200420, 0x04200000, 0x04200421, 0x04210000, 0x04210421, 0x04210000, 0x04200421,
        0x04200000, 0x04000420, 0x04200000, 0x04000421, 0x04210000, 0x04010421, 0x04210000, 0x04000421,
        0x04000000, 0x00000400, 0x04000000, 0x00000401, 0x04010000, 0x00010401, 0x04010000, 0x00000401,
        0x04000000, 0x00000420, 0x04000000, 0x00000421, 0x04010000, 0x00010421, 0x04010000, 0x00000421,
        0x04200000, 0x00200420, 0x04200000, 0x00200421, 0x04210000, 0x00210421, 0x04210000, 0x00200421,
        0x04200000, 0x00000420, 0x04200000, 0x00000421, 0x04210000, 0x00010421, 0x04210000, 0x00000421,
};
#else
const static uint32_t row_222_table[] = {
        0x00000000, 0x00000000, 0x00000000, 0x00000001, 0x00010000, 0x00010001, 0x00010000, 0x00000001,
        0x00000000, 0x00000040, 0x00000000, 0x00000041, 0x00010000, 0x00010041, 0x00010000, 0x00000041,
        0x00400000, 0x00400040, 0x00400000, 0x00400041, 0x00410000, 0x00410041, 0x00410000, 0x00400041,
        0x00400000, 0x00000040, 0x00400000, 0x00000041, 0x00410000, 0x00010041, 0x00410000, 0x00000041,
        0x00000000, 0x00000800, 0x00000000, 0x00000801, 0x00010000, 0x00010801, 0x00010000, 0x00000801,
        0x00000000, 0x00000840, 0x00000000, 0x00000841, 0x00010000, 0x00010841, 0x00010000, 0x00000841,
        0x00400000, 0x00400840, 0x00400000, 0x00400841, 0x00410000, 0x00410841, 0x00410000, 0x00400841,
        0x00400000, 0x00000840, 0x00400000, 0x00000841, 0x00410000, 0x00010841, 0x00410000, 0x00000841,
        0x08000000, 0x08000800, 0x08000000, 0x08000801, 0x08010000, 0x08010801, 0x08010000, 0x08000801,
        0x08000000, 0x08000840, 0x08000000, 0x08000841, 0x08010000, 0x08010841, 0x08010000, 0x08000841,
        0x08400000, 0x08400840, 0x08400000, 0x08400841, 0x08410000, 0x08410841, 0x08410000, 0x08400841,
        0x08400000, 0x08000840, 0x08400000, 0x08000841, 0x08410000, 0x08010841, 0x08410000, 0x08000841,
        0x08000000, 0x00000800, 0x08000000, 0x00000801, 0x08010000, 0x00010801, 0x08010000, 0x00000801,
        0x08000000, 0x00000840, 0x08000000, 0x00000841, 0x08010000, 0x00010841, 0x08010000, 0x00000841,
        0x08400000, 0x00400840, 0x08400000, 0x00400841, 0x08410000, 0x00410841, 0x08410000, 0x00400841,
        0x08400000, 0x00000840, 0x08400000, 0x00000841, 0x08410000, 0x00010841, 0x08410000, 0x00000841,
};

#endif

const uint32_t* platypus_decompress_row(uint32_t *d0, uint32_t *d1, const uint32_t *s4, uint32_t w) {
    const uint8_t *s = (const uint8_t *)s4;
    for(uint x = 0; x < w; x += 2)
    {
        uint32_t row0, row1;
#ifndef PLATYPUS_565
        if (s[1] & 0x80) {
#else
        if (s[0] & 0x20) {
#endif
            uint c0 = s[0] | (s[1] << 8);
            uint c1 = s[2] | (s[3] << 8);
#ifndef PLATYPUS_565
            if (s[3] & 0x80) {
#else
            if (s[2] & 0x20) {
#endif
                // todo this is unused right now
                row0 = (c1 << 16) | c0;
                row1 = (s[7] << 24) | (s[6] << 16) | (s[5] << 8) | s[4];
                s+=8;
            } else {
                row0 = (c1 << 16) | c0;
                row1 = (s[6] << 16) | (s[5] << 8) | s[4];
#ifndef PLATYPUS_565
                uint32_t mask = 0xff018401; // note ff at top saves us a mask of 0x00ffffff first for row1
                uint32_t hi_bits = row1 & mask;
                row1 ^= hi_bits;
                uint32_t lo_bits = row0 & (mask << 16u);
                row0 ^= lo_bits;
                hi_bits |= lo_bits >> 15u;
                row1 |= (hi_bits >> 10u) << 24u;
                row1 |= hi_bits << 27u;
#else
                uint32_t mask = 0xff210821;
                uint32_t hi_bits = row1 & mask;
                row1 ^= hi_bits;
                uint32_t lo_bits = row0 & (mask << 16u);
                row0 ^= lo_bits;
                lo_bits >>= 13u;
                hi_bits |= lo_bits;
                hi_bits ^= (hi_bits >> 10u);
                hi_bits = (hi_bits * 2) + ((hi_bits >> 11u) & 1u);
                row1 |= hi_bits << 24u;
#endif
                s+=7;
//                row0=row1=0;
            }
        } else {
            row0 = s[0] | (s[1] << 8);
            row1 = row0 = row0 | (row0 << 16);
            uint v = s[2];
#ifndef PLATYPUS_565
            if (v & 0x80u) {
                v = (v << 8u) | s[3];
#else
            if (v & 0x01u) {
                v = (v >> 1u) | (s[3] << 7u);
#endif
                // do Rs
#if PICO_ON_DEVICE
                v <<= 11u;
                interp1->accum[0] = v;
                uint32_t *p = (uint32_t *)(interp1->peek[0]);
                row0 += p[0];
                row1 += p[1];
#else
                // *2 for interleave
                v <<= 1u;
                row0 += row_5_table[v&0x3e] << PIXEL_RSHIFT;
                row1 += row_5_table[(v&0x3e)+1] << PIXEL_RSHIFT;
#endif
                // do Gs and Bs
#if PICO_ON_DEVICE
                interp0->accum[0] = v;
                uint32_t *p0 = (uint32_t *)(interp0->peek[0]);
                uint32_t *p1 = (uint32_t *)(interp0->peek[1]);

                row0 += p0[0] << PIXEL_GSHIFT;
                row1 += p0[1] << PIXEL_GSHIFT;
                row0 += p1[0] << PIXEL_BSHIFT;
                row1 += p1[1] << PIXEL_BSHIFT;
#else
                row0 += row_5_table[(v>>5u)&0x3eu] << PIXEL_GSHIFT;
                row1 += row_5_table[1 + ((v>>5u)&0x3eu)] << PIXEL_GSHIFT;
                row0 += row_5_table[(v>>10u)&0x3eu] << PIXEL_BSHIFT;
                row1 += row_5_table[1 + ((v>>10u)&0x3eu)] << PIXEL_BSHIFT;
#endif
                s += 4;
            } else {
#ifdef PLATYPUS_565
                v >>= 2;
#endif
#if PICO_ON_DEVICE
                interp1->accum[1] = v << 12u;
                uint32_t *p = (uint32_t *)(interp1->peek[1]);
                row0 += p[0];
                row1 += p[1];
#else
                row0 += row_222_table[v*2];
                row1 += row_222_table[v*2 + 1];
#endif
                s += 3;
            }
        }
        *d0++ = row0;
        *d1++ = row1;
    }
    s += (4u - ((uintptr_t)s) & 3u) & 3u;
    // round up to word boundary
    return (uint32_t *)s;
}

#else
void platypus_decompress_configure_interp(bool is_b) {
    extern uint32_t platypus_decompress_row_asm_a_222_table, platypus_decompress_row_asm_a_5_table;
    extern uint32_t platypus_decompress_row_asm_b_222_table, platypus_decompress_row_asm_b_5_table;
    uint32_t row_5 = (uintptr_t)(is_b?&platypus_decompress_row_asm_b_5_table:&platypus_decompress_row_asm_a_5_table);
    interp0->base[0] = row_5;
    interp0->base[1] = row_5;
    interp1->base[0] = row_5;
    interp1->base[1] = (uintptr_t)(is_b?&platypus_decompress_row_asm_b_222_table:&platypus_decompress_row_asm_a_222_table);
#ifndef VIDEO_DBI
    const uint es_555 = 0;
    const uint es_222 = 0;
#else
    const uint es_555 = 8;
    const uint es_222 = 9;
#endif
    interp_config c = interp_default_config();
    interp_config_set_shift(&c, 5 + es_555);
    interp_config_set_mask(&c, 3, 7);
    interp_set_config(interp0, 0, &c);

    interp_config_set_shift(&c, es_555);
    interp_set_config(interp1, 0, &c);

    interp_config_set_mask(&c, 3, 8);
    interp_config_set_shift(&c, es_222);
    interp_set_config(interp1, 1, &c);

    interp_config_set_mask(&c, 3, 7);
    interp_config_set_shift(&c, 10 + es_555);
    interp_config_set_cross_input(&c, true);
    interp_set_config(interp0, 1, &c);

//    interp_configure_none(interp0, 0, 5 + es_555, 3, 7);
//    interp_configure_with_cross_input(interp0, 1, 10 + es_555, 3, 7);
//    interp_configure_none(interp1, 0, es_555, 3, 7);
//    interp_configure_none(interp1, 1, es_222, 3, 8);

//    interp_add_force_bits(interp0, 0, 2);
//    interp_add_force_bits(interp0, 1, 2);
//    interp_add_force_bits(interp1, 0, 2);
//    interp_add_force_bits(interp1, 1, 2);
}
#endif