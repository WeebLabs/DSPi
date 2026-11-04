/* Host-test stand-in for the ring-loan part of firmware/DSPi/adat_output.h. */
#ifndef STUB_ADAT_OUTPUT_H
#define STUB_ADAT_OUTPUT_H
#include <stdint.h>
#include <stdbool.h>
bool  adat_output_ring_loanable(void);
void *adat_output_ring_loan(uint32_t *bytes, void (*evict)(void));
void  adat_output_ring_return(void);
#endif
