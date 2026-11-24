/*
 * rgb_led.h; the XIAO RP2350's on-board WS2812 on PIO1 SM3.
 */
#ifndef RGB_LED_H
#define RGB_LED_H
#include <stdint.h>
void rgb_led_init(void);
void rgb_led_set(uint8_t r, uint8_t g, uint8_t b);   // pushes one pixel; no-op if unchanged
#endif
