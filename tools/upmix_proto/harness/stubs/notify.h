#pragma once
#include <stdint.h>
/* Records every notify_param_write so the harness can check offsets against WireBulkParams. */
static uint16_t h_ntf_off[64], h_ntf_len[64];
static int h_ntf_n;
static inline void notify_param_write(uint16_t off, uint16_t len, const void *p) {
    (void)p;
    if (h_ntf_n < 64) { h_ntf_off[h_ntf_n] = off; h_ntf_len[h_ntf_n] = len; h_ntf_n++; }
}
