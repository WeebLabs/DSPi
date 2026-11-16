/*
 * vc_pins.h: GPIO validation and conflict checks (vc_pins.c). Status
 * returns are PIN_CONFIG_* from config.h.
 */

#ifndef VC_PINS_H
#define VC_PINS_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// A pin's validity for one function. Without DSPI_PLATFORM_PINS the function is
// ignored; a platform build applies its own rule (platform_pins.h).
#ifdef DSPI_PLATFORM_PINS
#define PIN_VALID_FOR(fn, pin) dspi_platform_pin_valid(fn, pin)
#else
#define PIN_VALID_FOR(fn, pin) is_valid_gpio_pin(pin)
#endif

// Platform default GPIO of each pin output, in output_pins[] order.
extern const uint8_t default_output_pins[NUM_PIN_OUTPUTS];

bool is_valid_gpio_pin(uint8_t pin);
bool is_pin_in_use(uint8_t pin, uint8_t exclude);

// Claimed by a fixed peripheral (outputs, SPDIF RX, ADAT, DAC mute, control
// pins), optionally MCK. I2S clock and RX data pins are the callers' business.
bool pin_used_by_fixed_peripheral(uint8_t pin, uint8_t exclude_output);
bool pin_used_by_fixed_peripheral_except_mck(uint8_t pin, uint8_t exclude_output);

// True if `pin` is the data pin of one of the first n_pairs I2S RX pairs other
// than exclude_pair. Any path that raises the active count must re-check.
bool i2s_rx_pair_pin_taken(uint8_t pin, uint8_t exclude_pair, uint8_t n_pairs);

// PIN_CONFIG_* for `pin` on I2S RX `pair` with n_pairs active. The BCK/LRCLK
// pair counts as always claimed, so a clash cannot first surface on the switch to I2S.
uint8_t check_i2s_rx_pin(uint8_t pin, uint8_t pair, uint8_t n_pairs);

// Full acceptability check for a UART/I2C control-interface GPIO. Returns PIN_CONFIG_*.
uint8_t ctrl_iface_check_pin(uint8_t pin);

// Also defined in vc_pins.c, declared in lightweight headers so the restore
// paths avoid TinyUSB: adat_pin_acceptable() in adat_output.h; the
// spdif_input_enable, i2s_rx_pin_set, i2s_bck(_slave), i2s_mck and
// adat_input *_acceptable() checks in audio_input.h.

#endif // VC_PINS_H
