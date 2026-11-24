/*
 * ui_hw.h; button and user-LED hardware shim for the XIAO RP2350.
 */
#ifndef UI_HW_H
#define UI_HW_H
#include <stdbool.h>
#include <stdint.h>

void ui_hw_init(void);
// Sampled at most every 5 ms; between samples the last value is returned.
bool ui_hw_button_pressed(uint32_t now_ms);
void ui_hw_user_led(bool on);   // yellow LED, active low on the board

#endif
