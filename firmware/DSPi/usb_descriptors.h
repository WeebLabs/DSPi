/*
 * USB Descriptors for DSPi — TinyUSB / UAC1
 *
 * Phase 1: audio-only composite (AC + AS).  Vendor interface removed pending
 * Phase 2.  Configuration descriptor is a packed byte array built at compile
 * time; no LUFA structs.
 */

#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include <stdint.h>

#include "tusb.h"
#include "class/audio/audio.h"

// ----------------------------------------------------------------------------
// USB IDs
// ----------------------------------------------------------------------------

#define USB_VENDOR_ID   0x2E8A
#define USB_PRODUCT_ID  0xFEAA
#define USB_BCD_DEVICE  0x0200

// ----------------------------------------------------------------------------
// ENDPOINT ADDRESSES
// ----------------------------------------------------------------------------

#define AUDIO_OUT_ENDPOINT  0x01U
#define AUDIO_IN_ENDPOINT   0x82U
#define AUDIO_EP_MAX_PKT    582U   // Sized for 24-bit stereo 96 kHz + 1 jitter sample

// ----------------------------------------------------------------------------
// INTERFACE NUMBERS
// ----------------------------------------------------------------------------

#define ITF_NUM_AUDIO_CONTROL   0
#define ITF_NUM_AUDIO_STREAMING 1
#define ITF_NUM_TOTAL           2

// ----------------------------------------------------------------------------
// UAC1 ENTITY IDs
// ----------------------------------------------------------------------------

#define UAC1_INPUT_TERMINAL_ID   1
#define UAC1_FEATURE_UNIT_ID     2
#define UAC1_OUTPUT_TERMINAL_ID  3

// ----------------------------------------------------------------------------
// UAC1 REQUEST OPCODES (not exposed by TinyUSB — UAC2 constants are UAC2-only)
// ----------------------------------------------------------------------------

#define UAC1_REQ_SET_CUR    0x01
#define UAC1_REQ_GET_CUR    0x81
#define UAC1_REQ_GET_MIN    0x82
#define UAC1_REQ_GET_MAX    0x83
#define UAC1_REQ_GET_RES    0x84

// UAC1 feature unit control selectors
#define UAC1_FU_CTRL_MUTE   0x01
#define UAC1_FU_CTRL_VOLUME 0x02

// UAC1 endpoint control selector
#define UAC1_EP_CTRL_SAMPLING_FREQ 0x01

// ----------------------------------------------------------------------------
// STRING INDICES
// ----------------------------------------------------------------------------

#define STRID_LANGID        0
#define STRID_MANUFACTURER  1
#define STRID_PRODUCT       2
#define STRID_SERIAL        3

// Exported for main.c — populated from chip unique ID at boot.
extern char usb_descriptor_str_serial[17];

// Full configuration descriptor as packed bytes.  Defined in usb_descriptors.c.
extern const uint8_t usb_config_descriptor[];
extern const uint16_t usb_config_descriptor_len;

// Alt-setting endpoint descriptor pointers — resolved at link time so the
// UAC1 class driver can call usbd_edpt_iso_activate() without re-walking the
// config on every SET_INTERFACE.  Indexed [alt-1]: [0] = alt 1 (16-bit),
// [1] = alt 2 (24-bit).
extern const uint8_t *const usb_audio_data_ep_desc[2];
extern const uint8_t *const usb_audio_fb_ep_desc[2];

#endif // USB_DESCRIPTORS_H
