#ifndef _PLATYPUS_H
#define _PLATYPUS_H

#include <stdint.h>

#ifdef VIDEO_DBI
#undef PLATYPUS_565
#define PLATYPUS_565
#endif

#if PICO_ON_DEVICE
extern const uint32_t* platypus_decompress_row_asm_a(uint32_t *d0, uint32_t *d1, const uint32_t *s, uint32_t w);
extern const uint32_t* platypus_decompress_row_asm_b(uint32_t *d0, uint32_t *d1, const uint32_t *s, uint32_t w);
extern void platypus_decompress_configure_interp(bool is_b);
#define platypus_decompress_row_a platypus_decompress_row_asm_a
#define platypus_decompress_row_b platypus_decompress_row_asm_b
#else
extern const uint32_t* platypus_decompress_row(uint32_t *d0, uint32_t *d1, const uint32_t *s, uint32_t w);
#define platypus_decompress_row_a platypus_decompress_row
#define platypus_decompress_row_b platypus_decompress_row
#endif

#endif

