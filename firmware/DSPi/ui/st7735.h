/*
 * st7735.h; ST7735 1.8" 160x128 colour TFT over SPI with a DMA framebuffer
 * flush.  Landscape, RGB565.
 *
 * Bring-up never blocks: st7735_service() advances a small state machine
 * (reset pulse, init commands with their settle times, black frame) and
 * returns true once the panel is on. Core 0 only.
 *
 * The backlight is not driven (on this build it is wired on), so "levels" and
 * "power off" are done in the picture instead: gfx scales colours for the
 * level, and power off sends a black frame (switching an ST7735 off shows
 * white on these normally-white panels).
 */
#ifndef ST7735_H
#define ST7735_H

#include <stdbool.h>
#include <stdint.h>

void st7735_begin(void);                 // claim pins/SPI/DMA, start bring-up
bool st7735_service(void);               // advance bring-up and flush completion; true when ready
bool st7735_ready(void);
bool st7735_busy(void);                  // a flush (or blanking) is in flight
bool st7735_flush(const uint16_t *fb);   // start a full-frame flush; false if busy/not ready
void st7735_power(bool on);              // off = black frame; on = next flush redraws

#endif
