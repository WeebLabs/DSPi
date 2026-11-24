/*
 * ssd1331.h; SSD1331 96x64 RGB OLED over SPI with a DMA framebuffer flush.
 *
 * Assumes the 0.95" colour OLED breakout uses the SSD1331 controller (the
 * common one at this size and resolution); confirm against the module's
 * silkscreen/listing before first power-up.
 *
 * Bring-up never blocks: ssd1331_service() advances a small state machine
 * (reset pulse, init commands, black frame, display on) and returns true once
 * the panel is on. Core 0 only.
 */
#ifndef SSD1331_H
#define SSD1331_H

#include <stdbool.h>
#include <stdint.h>

void ssd1331_begin(void);                 // claim pins/SPI/DMA, start bring-up
bool ssd1331_service(void);               // advance bring-up and flush completion; true when ready
bool ssd1331_ready(void);
bool ssd1331_busy(void);                  // a flush is in flight
bool ssd1331_flush(const uint16_t *fb);   // start a full-frame flush; false if busy/not ready
void ssd1331_set_level(uint8_t level);    // 0..3 brightness
void ssd1331_power(bool on);              // display off/on (sleep), keeps RAM

#endif
