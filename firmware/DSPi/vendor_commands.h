/*
 * vendor_commands.h — Vendor USB control request handlers for DSPi
 *
 * Phase 2: adapted for TinyUSB.  The previous pico-extras `struct usb_interface`
 * / `struct usb_setup_packet` API is replaced by TinyUSB's stage-based
 * `tud_control_xfer` flow.  The DSPi UAC1 class driver (`usb_audio.c`) routes
 * vendor-class vendor-interface requests here via `vendor_control_xfer_cb`.
 */

#ifndef VENDOR_COMMANDS_H
#define VENDOR_COMMANDS_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>
#include "hardware/vreg.h"
#include "tusb.h"

// Overrides TinyUSB's weak tud_vendor_control_xfer_cb (usbd.c:82).  TinyUSB
// routes ALL vendor-type control transfers here directly (usbd.c:727-730),
// bypassing class drivers, regardless of wIndex/recipient.
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req);

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
