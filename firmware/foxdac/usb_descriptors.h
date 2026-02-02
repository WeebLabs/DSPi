/*
 * TinyUSB USB Descriptors Header for DSPi Audio Device
 */

#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include "tusb.h"

// ----------------------------------------------------------------------------
// DESCRIPTOR STRINGS
// ----------------------------------------------------------------------------

// Expose serial string so main.c can write to it
extern char descriptor_str_serial[17];

// ----------------------------------------------------------------------------
// MICROSOFT WCID SUPPORT
// ----------------------------------------------------------------------------

// Handle Microsoft OS descriptor vendor requests
// Called from tud_vendor_control_xfer_cb()
bool handle_ms_vendor_request(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request);

#endif // USB_DESCRIPTORS_H
