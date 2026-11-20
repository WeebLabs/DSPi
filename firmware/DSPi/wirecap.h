/*
 * wirecap.h: bench wire-capture rig (DSPI_WIRECAP builds only). A PIO pin
 * sampler and an I2S clock generator for clock-slave tests, driven from the
 * DIAG page. Design, commands and limits: Documentation/Features/wirecap_test_rig.md.
 */

#ifndef WIRECAP_H
#define WIRECAP_H

#ifdef DSPI_WIRECAP

#include <stdint.h>
#include <stdbool.h>

// Board-internal on the Pico and Pico 2 (VBUS sense and LED), so no header wiring can see them.
#define WIRECAP_CLK_BCK_PIN   24u
#define WIRECAP_CLK_LRCK_PIN  25u

#define WIRECAP_STATUS_LEN    32u
#define WIRECAP_CLKGEN_LEN    24u

typedef enum {
    WIRECAP_OK = 0,
    WIRECAP_ERR_STATE,      // busy, or the request does not fit the current state
    WIRECAP_ERR_VALUE,      // a parameter is out of range
    WIRECAP_ERR_RESOURCE,   // no free state machine, DMA channel or instruction space
} WirecapResult;

// One-shot capture of pin_count pins from pin_base (1, 2, 4, 8, 16 or 32), bytes deep.
WirecapResult wirecap_arm(uint8_t pin_base, uint8_t pin_count, uint16_t div_int,
                          uint8_t div_frac, uint32_t bytes);
void wirecap_status(uint8_t out[WIRECAP_STATUS_LEN]);
// Copies up to len captured bytes from offset; returns the count copied.
uint16_t wirecap_read(uint32_t offset, uint8_t *dst, uint16_t len);

WirecapResult wirecap_clkgen_set(bool enable, uint32_t rate);
void wirecap_clkgen_status(uint8_t out[WIRECAP_CLKGEN_LEN]);
WirecapResult wirecap_clkgen_glitch(uint8_t kind, uint8_t cells);

// Output rate request through the host's deferred path (USB input only).
WirecapResult wirecap_request_rate(uint32_t rate);

// Main loop: releases the sampler's resources once a capture completes.
void wirecap_poll(void);

#endif // DSPI_WIRECAP
#endif // WIRECAP_H
