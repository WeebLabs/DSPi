/*
 * ui.h; front-panel UI for the XIAO RP2350 build (OLED, RGB LED, one button).
 *
 * Core 0 main loop only. ui_init() runs once after control_surfaces_init();
 * ui_service() runs every main-loop pass and is cheap when nothing is due.
 * Drawing runs as a budgeted background job (BG_JOB_UI); the display flush is
 * DMA. Everything is read and changed through DSPi's control-surface noun API,
 * the same path a hardware knob or the host app takes.
 */
#ifndef UI_H
#define UI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef DSPI_UI
void ui_init(void);
void ui_service(void);
bool ui_owns_pin(uint8_t gpio);   // pins the UI uses; DSPi's pin validator refuses them
#else
static inline void ui_init(void) {}
static inline void ui_service(void) {}
static inline bool ui_owns_pin(uint8_t gpio) { (void)gpio; return false; }
#endif

#endif
