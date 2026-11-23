#pragma once
#include <stdint.h>
/* Records notify offsets so the harness can check them against WireBulkParams. */
static uint16_t h_ntf_off[32], h_ntf_len[32];
static int h_ntf_n;
static inline void notify_param_write(uint16_t off, uint16_t len, const void *p) {
    (void)p;
    if (h_ntf_n < 32) { h_ntf_off[h_ntf_n] = off; h_ntf_len[h_ntf_n] = len; h_ntf_n++; }
}
