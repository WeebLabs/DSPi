/*
 * USB Descriptors for DSPi — TinyUSB / UAC1
 *
 * Hand-rolled UAC1 config descriptor as a packed byte array.  TinyUSB's
 * TUD_AUDIO_DESC_* macros emit UAC2-shaped descriptors, so they cannot be
 * used here.
 */

#include <string.h>

#include "tusb.h"
#include "class/audio/audio.h"
#include "pico/unique_id.h"

#include "usb_descriptors.h"

// ----------------------------------------------------------------------------
// STRINGS
// ----------------------------------------------------------------------------

char usb_descriptor_str_serial[17] = "0123456789ABCDEF";

static const char *const string_table[] = {
    "",                    // 0 — placeholder; langid returned separately
    "GitHub.com/WeebLabs", // 1 — manufacturer
    "Weeb Labs DSPi",      // 2 — product
    usb_descriptor_str_serial, // 3 — serial (populated at boot)
};

// ----------------------------------------------------------------------------
// DEVICE DESCRIPTOR
// ----------------------------------------------------------------------------

static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VENDOR_ID,
    .idProduct          = USB_PRODUCT_ID,
    .bcdDevice          = USB_BCD_DEVICE,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 0x01,
};

// ----------------------------------------------------------------------------
// CONFIGURATION DESCRIPTOR (UAC1 with IAD)
//
// The Interface Association Descriptor (IAD) is required so TinyUSB's
// process_set_config() binds BOTH the AC and AS interfaces to our custom UAC1
// class driver.  Without an IAD, TinyUSB would only bind the first interface
// (AC) per the bInterfaceCount==1 default, and SET_INTERFACE requests on the
// AS interface would fail to route to our driver (no EP opens, no audio).
//
// Layout (offsets from start of usb_config_descriptor[]):
//
//   0  Config descriptor                               (9 bytes)
//   9  IAD (covers AC + AS)                            (8 bytes)
//  17  AC std interface (itf 0, 0 EPs, UAC1)           (9 bytes)
//  26  AC CS header                                    (9 bytes)
//  35  AC CS input terminal (ID 1, USB streaming)      (12 bytes)
//  47  AC CS feature unit (ID 2, mute+volume master)   (10 bytes)
//  57  AC CS output terminal (ID 3, speaker)           (9 bytes)
//  66  AS std interface alt 0 (zero-bw)                (9 bytes)
//  75  AS std interface alt 1 (16-bit)                 (9 bytes)
//  84  AS CS general (wFormatTag=PCM)                  (7 bytes)
//  91  AS CS format type I (16-bit, 3 rates)           (17 bytes)
// 108  Std iso EP OUT 0x01                             (9 bytes)
// 117  CS iso data EP (sampling freq control)          (7 bytes)
// 124  Std iso feedback EP IN 0x82                     (9 bytes)
// 133  AS std interface alt 2 (24-bit)                 (9 bytes)
// 142  AS CS general                                   (7 bytes)
// 149  AS CS format type I (24-bit, 3 rates)           (17 bytes)
// 166  Std iso EP OUT 0x01                             (9 bytes)
// 175  CS iso data EP                                  (7 bytes)
// 182  Std iso feedback EP IN 0x82                     (9 bytes)
// 191  Vendor std interface (itf 2, class 0xFF, 1 EP)  (9 bytes)
// 200  Std interrupt EP IN 0x83 (notifications, 4 ms)  (7 bytes)
// 207  total
//
// The vendor interface sits OUTSIDE the AC+AS IAD — it is its own USB
// function.  TinyUSB's process_set_config() will call our driver's open()
// a second time with this interface descriptor; we claim it, open the
// notification EP, and handle all class/vendor requests in control_xfer_cb().
// ----------------------------------------------------------------------------

#define CONFIG_TOTAL_LEN 207
#define AC_CS_TOTAL_LEN  40   // CS AC: header(9) + input(12) + FU(10) + output(9)

#define OFFSET_ALT1_DATA_EP 108
#define OFFSET_ALT1_FB_EP   124
#define OFFSET_ALT2_DATA_EP 166
#define OFFSET_ALT2_FB_EP   182
#define OFFSET_NOTIFY_EP    200   // Std interrupt EP descriptor, 7 bytes

// Sample rate little-endian expansion
#define RATE_LE(r) ((r) & 0xFF), (((r) >> 8) & 0xFF), (((r) >> 16) & 0xFF)

const uint8_t usb_config_descriptor[CONFIG_TOTAL_LEN] = {
    // --- 0: Config descriptor ---------------------------------------------
    9,                                  // bLength
    TUSB_DESC_CONFIGURATION,            // bDescriptorType
    U16_TO_U8S_LE(CONFIG_TOTAL_LEN),    // wTotalLength
    ITF_NUM_TOTAL,                      // bNumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0x80,                               // bmAttributes (bus-powered)
    0x32,                               // bMaxPower (100 mA)

    // --- 9: IAD grouping AC + AS into one audio function ------------------
    // Required so TinyUSB's process_set_config() binds both interfaces to our
    // class driver (bInterfaceCount = 2).
    8,                                  // bLength
    TUSB_DESC_INTERFACE_ASSOCIATION,    // 0x0B
    ITF_NUM_AUDIO_CONTROL,              // bFirstInterface (= 0)
    0x02,                               // bInterfaceCount (AC + AS)
    TUSB_CLASS_AUDIO,                   // bFunctionClass
    AUDIO_SUBCLASS_CONTROL,             // bFunctionSubClass (per USB-IF IAD convention for audio)
    0x00,                               // bFunctionProtocol (UAC1)
    0x00,                               // iFunction

    // --- 17: AC std interface (itf 0, 0 EPs, UAC1 protocol 0x00) ---------
    9,                                  // bLength
    TUSB_DESC_INTERFACE,                // bDescriptorType
    ITF_NUM_AUDIO_CONTROL,              // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,                   // bInterfaceClass
    AUDIO_SUBCLASS_CONTROL,             // bInterfaceSubClass
    0x00,                               // bInterfaceProtocol (UAC1 = 0x00)
    0x00,                               // iInterface

    // --- 18: AC CS header ------------------------------------------------
    9,                                  // bLength
    TUSB_DESC_CS_INTERFACE,             // bDescriptorType
    AUDIO_CS_AC_INTERFACE_HEADER,       // bDescriptorSubtype (0x01)
    U16_TO_U8S_LE(0x0100),              // bcdADC (UAC1.0)
    U16_TO_U8S_LE(AC_CS_TOTAL_LEN),     // wTotalLength (CS AC header+input+FU+output)
    0x01,                               // bInCollection
    ITF_NUM_AUDIO_STREAMING,            // baInterfaceNr[0]

    // --- 27: AC CS input terminal (ID 1, USB streaming) ------------------
    12,                                 // bLength
    TUSB_DESC_CS_INTERFACE,             // bDescriptorType
    AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL, // bDescriptorSubtype (0x02)
    UAC1_INPUT_TERMINAL_ID,             // bTerminalID
    U16_TO_U8S_LE(AUDIO_TERM_TYPE_USB_STREAMING),  // wTerminalType (0x0101)
    0x00,                               // bAssocTerminal
    0x02,                               // bNrChannels
    U16_TO_U8S_LE(0x0003),              // wChannelConfig (L|R)
    0x00,                               // iChannelNames
    0x00,                               // iTerminal

    // --- 39: AC CS feature unit (ID 2, master mute+volume, 2 logical ch) -
    10,                                 // bLength (7 + (2+1)*1)
    TUSB_DESC_CS_INTERFACE,             // bDescriptorType
    AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, // bDescriptorSubtype (0x06)
    UAC1_FEATURE_UNIT_ID,               // bUnitID
    UAC1_INPUT_TERMINAL_ID,             // bSourceID
    0x01,                               // bControlSize
    0x03,                               // bmaControls[0] master: MUTE|VOLUME
    0x00,                               // bmaControls[1] ch 1
    0x00,                               // bmaControls[2] ch 2
    0x00,                               // iFeature

    // --- 49: AC CS output terminal (ID 3, speaker) -----------------------
    9,                                  // bLength
    TUSB_DESC_CS_INTERFACE,             // bDescriptorType
    AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, // bDescriptorSubtype (0x03)
    UAC1_OUTPUT_TERMINAL_ID,            // bTerminalID
    U16_TO_U8S_LE(AUDIO_TERM_TYPE_OUT_GENERIC_SPEAKER),  // wTerminalType (0x0301)
    0x00,                               // bAssocTerminal
    UAC1_FEATURE_UNIT_ID,               // bSourceID
    0x00,                               // iTerminal

    // --- 58: AS std interface alt 0 (zero-bw) ----------------------------
    9,                                  // bLength
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING,
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,                               // bInterfaceProtocol (UAC1)
    0x00,                               // iInterface

    // --- 67: AS std interface alt 1 (16-bit) -----------------------------
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING,
    0x01,                               // bAlternateSetting
    0x02,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // --- 76: AS CS general alt 1 -----------------------------------------
    7,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_AS_GENERAL,   // 0x01
    UAC1_INPUT_TERMINAL_ID,             // bTerminalLink
    0x01,                               // bDelay
    U16_TO_U8S_LE(0x0001),              // wFormatTag = PCM

    // --- 83: AS CS format type I alt 1 (16-bit, discrete 44.1/48/96) ----
    17,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,  // 0x02
    0x01,                               // bFormatType = TYPE_I
    0x02,                               // bNrChannels
    0x02,                               // bSubFrameSize
    16,                                 // bBitResolution
    0x03,                               // bSamFreqType (3 discrete)
    RATE_LE(44100),
    RATE_LE(48000),
    RATE_LE(96000),

    // --- 100: Std iso EP OUT 0x01, alt 1 ---------------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_OUT_ENDPOINT,                 // bEndpointAddress
    0x05,                               // bmAttributes: iso, async
    U16_TO_U8S_LE(AUDIO_EP_MAX_PKT),    // wMaxPacketSize = 582
    0x01,                               // bInterval
    0x00,                               // bRefresh
    AUDIO_IN_ENDPOINT,                  // bSynchAddress (feedback EP)

    // --- 109: CS iso data EP alt 1 ---------------------------------------
    7,
    TUSB_DESC_CS_ENDPOINT,              // 0x25
    AUDIO_CS_EP_SUBTYPE_GENERAL,        // 0x01
    0x01,                               // bmAttributes: sampling freq control
    0x00,                               // bLockDelayUnits
    U16_TO_U8S_LE(0x0000),              // wLockDelay

    // --- 116: Std iso feedback EP IN 0x82, alt 1 -------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_IN_ENDPOINT,
    0x11,                               // bmAttributes: iso, feedback
    U16_TO_U8S_LE(3),                   // wMaxPacketSize
    0x01,                               // bInterval
    0x02,                               // bRefresh (2^2 = 4 ms)
    0x00,                               // bSynchAddress

    // --- 125: AS std interface alt 2 (24-bit) ----------------------------
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING,
    0x02,                               // bAlternateSetting
    0x02,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // --- 134: AS CS general alt 2 ----------------------------------------
    7,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_AS_GENERAL,
    UAC1_INPUT_TERMINAL_ID,
    0x01,
    U16_TO_U8S_LE(0x0001),

    // --- 141: AS CS format type I alt 2 (24-bit, discrete 44.1/48/96) ---
    17,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,
    0x01,
    0x02,
    0x03,                               // bSubFrameSize
    24,                                 // bBitResolution
    0x03,
    RATE_LE(44100),
    RATE_LE(48000),
    RATE_LE(96000),

    // --- 158: Std iso EP OUT 0x01, alt 2 ---------------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_OUT_ENDPOINT,
    0x05,
    U16_TO_U8S_LE(AUDIO_EP_MAX_PKT),
    0x01,
    0x00,
    AUDIO_IN_ENDPOINT,

    // --- 167: CS iso data EP alt 2 ---------------------------------------
    7,
    TUSB_DESC_CS_ENDPOINT,
    AUDIO_CS_EP_SUBTYPE_GENERAL,
    0x01,
    0x00,
    U16_TO_U8S_LE(0x0000),

    // --- 174: Std iso feedback EP IN 0x82, alt 2 -------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_IN_ENDPOINT,
    0x11,
    U16_TO_U8S_LE(3),
    0x01,
    0x02,
    0x00,

    // --- 191: Vendor std itf 2 (class 0xFF, 1 EP) -----------------------
    9,                                  // bLength
    TUSB_DESC_INTERFACE,
    ITF_NUM_VENDOR,                     // bInterfaceNumber (= 2)
    0x00,                               // bAlternateSetting
    0x01,                               // bNumEndpoints (interrupt IN)
    0xFF,                               // bInterfaceClass (vendor specific)
    0x00,                               // bInterfaceSubClass
    0x00,                               // bInterfaceProtocol
    0x00,                               // iInterface

    // --- 200: Std bulk EP IN 0x83 --------------------------------------
    7,                                  // bLength
    TUSB_DESC_ENDPOINT,
    NOTIFY_IN_ENDPOINT,                 // bEndpointAddress (0x83)
    TUSB_XFER_BULK,                     // bmAttributes (0x02)
    U16_TO_U8S_LE(NOTIFY_EP_MAX_PKT),   // wMaxPacketSize (8)
    NOTIFY_EP_INTERVAL_MS,              // bInterval (ignored for bulk on FS)
};

const uint16_t usb_config_descriptor_len = CONFIG_TOTAL_LEN;

// Per-alt EP descriptor pointers consumed by the UAC1 class driver.
const uint8_t *const usb_audio_data_ep_desc[2] = {
    &usb_config_descriptor[OFFSET_ALT1_DATA_EP],
    &usb_config_descriptor[OFFSET_ALT2_DATA_EP],
};
const uint8_t *const usb_audio_fb_ep_desc[2] = {
    &usb_config_descriptor[OFFSET_ALT1_FB_EP],
    &usb_config_descriptor[OFFSET_ALT2_FB_EP],
};

// ----------------------------------------------------------------------------
// TinyUSB descriptor callbacks
// ----------------------------------------------------------------------------

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&device_descriptor;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return usb_config_descriptor;
}

static uint16_t string_response[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    if (index == STRID_LANGID) {
        string_response[0] = (TUSB_DESC_STRING << 8) | 4;
        string_response[1] = 0x0409;  // English (US)
        return string_response;
    }

    if (index >= TU_ARRAY_SIZE(string_table)) return NULL;
    const char *str = string_table[index];
    if (!str) return NULL;

    size_t len = strlen(str);
    if (len > TU_ARRAY_SIZE(string_response) - 1) len = TU_ARRAY_SIZE(string_response) - 1;
    for (size_t i = 0; i < len; i++) {
        string_response[1 + i] = str[i];
    }
    string_response[0] = (TUSB_DESC_STRING << 8) | (uint8_t)(2 * len + 2);
    return string_response;
}
