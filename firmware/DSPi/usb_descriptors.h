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
// Bump on descriptor-affecting changes so Windows re-reads instead of using
// its cached descriptor.  0x0200 → 0x0201 for notification EP max-packet
// bump (8 → 64 bytes) introduced with the v2 notification protocol.
#define USB_BCD_DEVICE  0x0201

// ----------------------------------------------------------------------------
// ENDPOINT ADDRESSES
// ----------------------------------------------------------------------------

#define AUDIO_OUT_ENDPOINT  0x01U
#define AUDIO_IN_ENDPOINT   0x82U
#define AUDIO_EP_MAX_PKT    582U   // Sized for 24-bit stereo 96 kHz + 1 jitter sample

// Bulk IN endpoint on the vendor interface — device→host notifications
// (master volume changes, future knob events, etc.).
//
// We switched this from INTERRUPT to BULK after observing an RP2040/2350
// DCD-level crash when an interrupt IN endpoint under continuous host
// polling ran alongside rapid EP0 control transfers.  Bulk IN uses
// opportunistic host scheduling rather than fixed-interval polling, so the
// crash trigger (poll-timed IRQ cadence interacting with EP0 SETUP IRQs)
// is dodged.  bInterval is ignored for bulk on full-speed devices.
#define NOTIFY_IN_ENDPOINT      0x83U
// 64 bytes is the USB 2.0 full-speed bulk ceiling; it fits every current
// WireBulkParams field in a single transaction.  See notification_protocol_v2_spec.md.
#define NOTIFY_EP_MAX_PKT       64U
#define NOTIFY_EP_INTERVAL_MS   0U

// Notification event type constants are now defined in notify.h
// (NOTIFY_EVT_IDLE, NOTIFY_EVT_MASTER_VOLUME, NOTIFY_EVT_PARAM_CHANGED, ...).
// Legacy aliases retained for any code still using the old names.
#define NOTIFY_EVENT_IDLE          0x00
#define NOTIFY_EVENT_MASTER_VOLUME 0x01

// ----------------------------------------------------------------------------
// INTERFACE NUMBERS
// ----------------------------------------------------------------------------

#define ITF_NUM_AUDIO_CONTROL   0
#define ITF_NUM_AUDIO_STREAMING 1
#define ITF_NUM_VENDOR          2
#define ITF_NUM_TOTAL           3

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
