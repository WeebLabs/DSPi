/*
 * USB Descriptors for DSPi — TinyUSB / UAC1 + Vendor + MS OS 2.0
 *
 * Composite device with three interfaces: Audio Control (0) + Audio
 * Streaming (1) under an IAD bound to the OS audio class driver, plus a
 * standalone Vendor interface (2, class 0xFF) auto-bound to WinUSB on
 * Windows 8.1+ via Microsoft OS 2.0 Platform Capability descriptors.
 *
 * The configuration descriptor is a packed byte array built at compile
 * time. The BOS descriptor and MS OS 2.0 descriptor set live in
 * usb_descriptors.c and are returned via tud_descriptor_bos_cb and the
 * vendor SETUP handler in vendor_commands.c.
 */

#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include <stddef.h>  // size_t (for desc_ms_os_20_len)
#include <stdint.h>

#include "tusb.h"
#include "class/audio/audio.h"

// ----------------------------------------------------------------------------
// USB IDs
// ----------------------------------------------------------------------------

#define USB_VENDOR_ID   0x2E8B
#define USB_PRODUCT_ID  0xFEAA
// Bump on descriptor-affecting changes so Windows re-reads instead of using
// its cached descriptor.  0x0200 → 0x0201 for notification EP max-packet
// bump (8 → 64 bytes) introduced with the v2 notification protocol.
// 0x0201 → 0x0202 for the RP2350 8-channel input alt (alt 3) + widened input
// terminal/feature unit + larger iso OUT max-packet.
// 0x0202 → 0x0203 for the 4ch/6ch input alts + the unified channel model
// (per-input EQ/metering, wire V16 / slot V21).
// 0x0203 → 0x0204 adds RP2350 alt 6 (4ch / 24-bit / 48 kHz).
#define USB_BCD_DEVICE  0x0204

// ----------------------------------------------------------------------------
// ENDPOINT ADDRESSES
// ----------------------------------------------------------------------------

#define AUDIO_OUT_ENDPOINT  0x01U
#define AUDIO_IN_ENDPOINT   0x82U
#if PICO_RP2350
// RP2350 also serves an 8-channel/48 kHz/16-bit alt (5): 48 frames × 8 ch ×
// 2 B = 768 B, + 1 jitter frame (16 B) = 784, rounded up to a 4-byte-aligned
// 788.  This single value sizes the iso OUT EP for all alts (the EP buffer is
// allocated once at the max), the receive scratch buffer, and the USB ring
// slot.  Comfortably under the 1023-byte full-speed isochronous ceiling.
#define AUDIO_EP_MAX_PKT    788U
#else
#define AUDIO_EP_MAX_PKT    582U   // Sized for 24-bit stereo 96 kHz + 1 jitter sample
#endif

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
#ifdef DSPI_LOOPBACK
// Loopback capture function (debug build only): a second, self-contained UAC1
// audio function appended after the vendor interface.  Existing interface
// numbers 0/1/2 are unchanged so normal-build descriptors are unaffected.
#define ITF_NUM_LOOPBACK_AC     3   // capture AudioControl
#define ITF_NUM_LOOPBACK_AS     4   // capture AudioStreaming
#define ITF_NUM_TOTAL           5
#else
#define ITF_NUM_TOTAL           3
#endif

// ----------------------------------------------------------------------------
// UAC1 ENTITY IDs
// ----------------------------------------------------------------------------

#define UAC1_INPUT_TERMINAL_ID   1
#define UAC1_FEATURE_UNIT_ID     2
#define UAC1_OUTPUT_TERMINAL_ID  3

// ----------------------------------------------------------------------------
// LOOPBACK CAPTURE FUNCTION (DSPI_LOOPBACK, debug build only)
//
// A second UAC1 audio function exposing output slot 0 as a 2-ch 24-bit
// isochronous IN (recording) endpoint.  Rates are capped at DSPi's operating
// range (44.1/48 kHz) which also keeps the RP2040 USB DPRAM budget small.
// ----------------------------------------------------------------------------
#ifdef DSPI_LOOPBACK
#define LOOPBACK_IN_ENDPOINT        0x81U  // isochronous async IN (capture)

#define LOOPBACK_N_CHANNELS         2
#define LOOPBACK_BYTES_PER_SAMPLE   3      // 24-bit
#define LOOPBACK_BYTES_PER_FRAME    (LOOPBACK_N_CHANNELS * LOOPBACK_BYTES_PER_SAMPLE)  // 6
// Servo never emits more than this per USB frame; sizes the iso IN EP buffer.
// At 48 kHz the servo's hard max is floor(frac<1 + nominal 48 + SERVO_MAX_CORR 2)
// = 50 frames; 52 leaves a 2-frame margin.  Sized at 52 (312 B) instead of 64
// (384 B) so the host's full-speed iso reservation for input (788) + this
// capture EP + feedback (3) = 1103 B/frame stays under the FS periodic ceiling
// (input + the old 384 = 1175 over-subscribed and dropped output frames while
// the capture stream was active).  Unlike the input OUT EP, this is the loopback
// driver's own endpoint: wMaxPacketSize, usbd_edpt_iso_alloc(), and g_pkt all
// derive from LOOPBACK_EP_IN_SIZE, so there is no maxpacket/queue mismatch.
#define LOOPBACK_MAX_FRAMES_PER_PACKET  52
#define LOOPBACK_EP_IN_SIZE         (LOOPBACK_MAX_FRAMES_PER_PACKET * LOOPBACK_BYTES_PER_FRAME)  // 312

// Capture-function entity IDs (distinct from the playback function's 1/2/3).
#define LOOPBACK_INPUT_TERMINAL_ID   4   // internal: output slot 0
#define LOOPBACK_OUTPUT_TERMINAL_ID  5   // USB streaming
#endif

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
// config on every SET_INTERFACE.  Indexed [alt-1]: alts 1/2 = stereo 16/24-bit;
// RP2350 adds alts 3/4/5 = 4/6/8ch 16-bit and alt 6 = 4ch 24-bit.
extern const uint8_t *const usb_audio_data_ep_desc[];
extern const uint8_t *const usb_audio_fb_ep_desc[];

// MS OS 2.0 descriptor set (178 bytes).  Returned to the host on a vendor
// SETUP request bRequest=MS_VENDOR_CODE wIndex=7 to advertise WinUSB binding
// + DeviceInterfaceGUIDs for the vendor function.  Defined in usb_descriptors.c.
extern const uint8_t desc_ms_os_20[];
extern const size_t desc_ms_os_20_len;

#endif // USB_DESCRIPTORS_H
