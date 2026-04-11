/*
 * vendor_commands.h — Vendor USB control request handlers for DSPi
 *
 * Extracted from usb_audio.c: GET/SET dispatch, pin/MCK helpers,
 * system diagnostics (temperature, voltage), and ring buffer accessor.
 */

#ifndef VENDOR_COMMANDS_H
#define VENDOR_COMMANDS_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>
#include "hardware/vreg.h"

// Forward declarations to avoid pulling in heavy headers
struct usb_interface;
struct usb_setup_packet;

// GET/SET vendor request dispatch (assigned to vendor_interface.setup_request_handler)
bool vendor_setup_request_handler(struct usb_interface *interface, struct usb_setup_packet *setup);

// USB control IN helper — also called by device_setup_request_handler() in usb_audio.c
void vendor_send_response(const void *data, uint len);

// Core 1 mode derivation from current output enable state
Core1Mode derive_core1_mode(void);

// System statistics helpers
uint16_t vreg_voltage_to_mv(enum vreg_voltage voltage);
int16_t read_temperature_cdeg(void);

// Pin validation helpers
bool is_valid_gpio_pin(uint8_t pin);
bool is_pin_in_use(uint8_t pin, uint8_t exclude);

// MCK encode/decode for wire and flash persistence
uint8_t  mck_encode(uint16_t val);
uint16_t mck_decode(uint8_t raw);

// MCK multiplier validation and clamping
bool is_mck_multiplier_supported_for_rate(uint16_t mult, uint32_t sample_rate_hz);
void sanitize_mck_multiplier_for_rate(uint32_t sample_rate_hz);

// Ring buffer accessor (audio_ring is static in usb_audio.c)
uint32_t usb_audio_ring_overrun_count(void);

#endif // VENDOR_COMMANDS_H
