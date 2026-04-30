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
#include "config.h"  // MS_VENDOR_CODE

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
    // bcdUSB = 0x0210 (USB 2.10) — required for Windows to query the BOS
    // descriptor and discover our MS OS 2.0 Platform Capability descriptor.
    // 0x0201 is the spec-minimum that triggers the BOS query; 0x0210 is the
    // conventional value used in canonical TinyUSB examples and is what we
    // standardise on.
    .bcdUSB             = 0x0210,
    // IAD signaling triplet (0xEF, 0x02, 0x01).  Per the USB-IF IAD ECN
    // and the Microsoft "Building Composite USB Devices" guidance, any
    // device that uses an Interface Association Descriptor MUST advertise
    // this triplet at the device level so the host classifies the device
    // as composite (Miscellaneous + Common + IAD).  On Windows, this is
    // what causes usbccgp.sys to spawn a composite-device parent and
    // apply per-function driver binding — including the per-function
    // WinUSB binding from our MS OS 2.0 Function Subset Header.  Without
    // it, Windows would inspect interface 0 (Audio Control) and treat
    // the whole device as Audio, breaking vendor-interface binding.
    .bDeviceClass       = TUSB_CLASS_MISC,        // 0xEF
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,   // 0x02
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,      // 0x01
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

    // --- 26: AC CS header ------------------------------------------------
    9,                                  // bLength
    TUSB_DESC_CS_INTERFACE,             // bDescriptorType
    AUDIO_CS_AC_INTERFACE_HEADER,       // bDescriptorSubtype (0x01)
    U16_TO_U8S_LE(0x0100),              // bcdADC (UAC1.0)
    U16_TO_U8S_LE(AC_CS_TOTAL_LEN),     // wTotalLength (CS AC header+input+FU+output)
    0x01,                               // bInCollection
    ITF_NUM_AUDIO_STREAMING,            // baInterfaceNr[0]

    // --- 35: AC CS input terminal (ID 1, USB streaming) ------------------
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

    // --- 47: AC CS feature unit (ID 2, master mute+volume, 2 logical ch) -
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

    // --- 57: AC CS output terminal (ID 3, speaker) -----------------------
    9,                                  // bLength
    TUSB_DESC_CS_INTERFACE,             // bDescriptorType
    AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, // bDescriptorSubtype (0x03)
    UAC1_OUTPUT_TERMINAL_ID,            // bTerminalID
    U16_TO_U8S_LE(AUDIO_TERM_TYPE_OUT_GENERIC_SPEAKER),  // wTerminalType (0x0301)
    0x00,                               // bAssocTerminal
    UAC1_FEATURE_UNIT_ID,               // bSourceID
    0x00,                               // iTerminal

    // --- 66: AS std interface alt 0 (zero-bw) ----------------------------
    9,                                  // bLength
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING,
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,                               // bInterfaceProtocol (UAC1)
    0x00,                               // iInterface

    // --- 75: AS std interface alt 1 (16-bit) -----------------------------
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING,
    0x01,                               // bAlternateSetting
    0x02,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // --- 84: AS CS general alt 1 -----------------------------------------
    7,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_AS_GENERAL,   // 0x01
    UAC1_INPUT_TERMINAL_ID,             // bTerminalLink
    0x01,                               // bDelay
    U16_TO_U8S_LE(0x0001),              // wFormatTag = PCM

    // --- 91: AS CS format type I alt 1 (16-bit, discrete 44.1/48/96) ----
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

    // --- 108: Std iso EP OUT 0x01, alt 1 ---------------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_OUT_ENDPOINT,                 // bEndpointAddress
    0x05,                               // bmAttributes: iso, async
    U16_TO_U8S_LE(AUDIO_EP_MAX_PKT),    // wMaxPacketSize = 582
    0x01,                               // bInterval
    0x00,                               // bRefresh
    AUDIO_IN_ENDPOINT,                  // bSynchAddress (feedback EP)

    // --- 117: CS iso data EP alt 1 ---------------------------------------
    7,
    TUSB_DESC_CS_ENDPOINT,              // 0x25
    AUDIO_CS_EP_SUBTYPE_GENERAL,        // 0x01
    0x01,                               // bmAttributes: sampling freq control
    0x00,                               // bLockDelayUnits
    U16_TO_U8S_LE(0x0000),              // wLockDelay

    // --- 124: Std iso feedback EP IN 0x82, alt 1 -------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_IN_ENDPOINT,
    0x11,                               // bmAttributes: iso, feedback
    U16_TO_U8S_LE(3),                   // wMaxPacketSize
    0x01,                               // bInterval
    0x02,                               // bRefresh (2^2 = 4 ms)
    0x00,                               // bSynchAddress

    // --- 133: AS std interface alt 2 (24-bit) ----------------------------
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING,
    0x02,                               // bAlternateSetting
    0x02,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // --- 142: AS CS general alt 2 ----------------------------------------
    7,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_AS_GENERAL,
    UAC1_INPUT_TERMINAL_ID,
    0x01,
    U16_TO_U8S_LE(0x0001),

    // --- 149: AS CS format type I alt 2 (24-bit, discrete 44.1/48/96) ---
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

    // --- 166: Std iso EP OUT 0x01, alt 2 ---------------------------------
    9,
    TUSB_DESC_ENDPOINT,
    AUDIO_OUT_ENDPOINT,
    0x05,
    U16_TO_U8S_LE(AUDIO_EP_MAX_PKT),
    0x01,
    0x00,
    AUDIO_IN_ENDPOINT,

    // --- 175: CS iso data EP alt 2 ---------------------------------------
    7,
    TUSB_DESC_CS_ENDPOINT,
    AUDIO_CS_EP_SUBTYPE_GENERAL,
    0x01,
    0x00,
    U16_TO_U8S_LE(0x0000),

    // --- 182: Std iso feedback EP IN 0x82, alt 2 -------------------------
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
    U16_TO_U8S_LE(NOTIFY_EP_MAX_PKT),   // wMaxPacketSize (64 — FS bulk max)
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
// MS OS 2.0 — auto-bind WinUSB to the vendor interface (no Zadig)
//
// On Windows 8.1+, advertising a Microsoft OS 2.0 Platform Capability
// descriptor in the BOS lets us tell the OS to auto-load winusb.sys for the
// vendor function (interface 2 only — audio interfaces 0+1 stay on the OS
// audio class driver).  Windows queries the BOS, sees the platform UUID
// for MS OS 2.0, then sends a vendor SETUP request (bmRequestType=0xC0,
// bRequest=MS_VENDOR_CODE, wIndex=7) to fetch the descriptor set below.
//
// The host application then uses the published DeviceInterfaceGUID to
// locate DSPi via SetupDiGetClassDevs / WinUsb_Initialize.  This GUID is
// product-line-scoped — the host app must use the SAME GUID we publish here.
//
// Reference: pico-sdk/lib/tinyusb/examples/device/webusb_serial/.
// ----------------------------------------------------------------------------

// MS OS 2.0 descriptor set — 178 bytes (0xB2):
//   Set Header (10) + Configuration Subset Header (8)
// + Function Subset Header (8) + Compatible ID Feature Descriptor (20)
// + Registry Property Feature Descriptor (132).
#define MS_OS_20_DESC_LEN  0xB2

// DSPi DeviceInterfaceGUID (generated 2026-04-30 via uuidgen).
// Hard-coded into the registry-property descriptor below (UTF-16LE).
// The host app MUST use this exact GUID when calling SetupDiGetClassDevs.
//
//     {9D9B8609-E6D1-4FF0-92AF-403119CB7692}
//
// If this GUID is ever changed, the host app's discovery code must change
// in lockstep, and Windows clients with a cached Container ID may need to
// re-enumerate the device (uninstall via Device Manager, replug).
const uint8_t desc_ms_os_20[] = {
    // ----- Set Header (10 bytes) -----
    U16_TO_U8S_LE(0x000A),                                   // wLength
    U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),           // wDescriptorType (0)
    U32_TO_U8S_LE(0x06030000),                               // dwWindowsVersion (Win 8.1)
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN),                        // wTotalLength

    // ----- Configuration Subset Header (8 bytes) -----
    U16_TO_U8S_LE(0x0008),                                   // wLength
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),     // wDescriptorType (1)
    0,                                                        // bConfigurationValue (0-based)
    0,                                                        // bReserved
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),                 // wTotalLength

    // ----- Function Subset Header (8 bytes) -----
    // Scopes the WinUSB compatible ID + DeviceInterfaceGUID below to
    // ITF_NUM_VENDOR ONLY.  Audio interfaces (0+1) are unaffected.
    U16_TO_U8S_LE(0x0008),                                   // wLength
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),          // wDescriptorType (2)
    ITF_NUM_VENDOR,                                           // bFirstInterface (= 2)
    0,                                                        // bReserved
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),          // wSubsetLength

    // ----- Compatible ID Feature Descriptor (20 bytes) -----
    // Tells Windows to bind winusb.sys to the function starting at
    // bFirstInterface above.  Note TinyUSB's enum is misspelt
    // "COMPATBLE" (no second 'I'); match it.
    U16_TO_U8S_LE(0x0014),                                   // wLength
    U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),            // wDescriptorType (3)
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,                // CompatibleID (8 bytes ASCII)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,          // SubCompatibleID (8 zero bytes)

    // ----- Registry Property Feature Descriptor (132 bytes) -----
    // Publishes "DeviceInterfaceGUIDs" (REG_MULTI_SZ) so the host app can
    // find DSPi via SetupDiGetClassDevs(&guid, ..., DIGCF_DEVICEINTERFACE).
    U16_TO_U8S_LE(0x0084),                                   // wLength (132)
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),            // wDescriptorType (4)
    U16_TO_U8S_LE(0x0007),                                   // wPropertyDataType (REG_MULTI_SZ)
    U16_TO_U8S_LE(0x002A),                                   // wPropertyNameLength (42)
    // PropertyName: UTF-16LE "DeviceInterfaceGUIDs\0" (42 bytes)
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00,
    'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00,
    't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00,
    'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00,
    0x00, 0x00,
    U16_TO_U8S_LE(0x0050),                                   // wPropertyDataLength (80)
    // PropertyData: UTF-16LE "{9D9B8609-E6D1-4FF0-92AF-403119CB7692}\0\0"
    // (38 chars + NUL terminator + extra NUL for REG_MULTI_SZ list end = 80 bytes)
    '{', 0x00, '9', 0x00, 'D', 0x00, '9', 0x00,
    'B', 0x00, '8', 0x00, '6', 0x00, '0', 0x00,
    '9', 0x00, '-', 0x00, 'E', 0x00, '6', 0x00,
    'D', 0x00, '1', 0x00, '-', 0x00, '4', 0x00,
    'F', 0x00, 'F', 0x00, '0', 0x00, '-', 0x00,
    '9', 0x00, '2', 0x00, 'A', 0x00, 'F', 0x00,
    '-', 0x00, '4', 0x00, '0', 0x00, '3', 0x00,
    '1', 0x00, '1', 0x00, '9', 0x00, 'C', 0x00,
    'B', 0x00, '7', 0x00, '6', 0x00, '9', 0x00,
    '2', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00,
};

const size_t desc_ms_os_20_len = sizeof(desc_ms_os_20);
_Static_assert(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN,
               "MS OS 2.0 descriptor set must be exactly 178 bytes");

// ----------------------------------------------------------------------------
// BOS Descriptor — 33 bytes (5 header + 28 Platform Capability for MS OS 2.0)
//
// Built from TinyUSB helper macros so the Microsoft platform-capability UUID
// is in the canonical mixed-endian byte order without hand-rolled GUIDs.
// See pico-sdk/lib/tinyusb/src/device/usbd.h for the macro expansions.
// ----------------------------------------------------------------------------

#define BOS_TOTAL_LEN  (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, /*bNumDeviceCaps*/ 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, MS_VENDOR_CODE)
};

uint8_t const *tud_descriptor_bos_cb(void) {
    return desc_bos;
}

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
