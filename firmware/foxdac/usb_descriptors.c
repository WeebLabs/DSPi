/*
 * TinyUSB USB Descriptors for DSPi Audio Device
 * UAC2 Audio Class with Vendor Interface (WinUSB/WCID)
 */

#include "tusb.h"
#include "usb_descriptors.h"
#include "config.h"
#include <string.h>

// ----------------------------------------------------------------------------
// DESCRIPTOR STRINGS
// ----------------------------------------------------------------------------

static char descriptor_str_vendor[] = "GitHub.com/WeebLabs";
static char descriptor_str_product[] = "Weeb Labs DSPi";
char descriptor_str_serial[17] = "0123456789ABCDEF";

// String descriptor indices
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
};

// ----------------------------------------------------------------------------
// USB IDS
// ----------------------------------------------------------------------------

#define USB_VID   0x2e8au
#define USB_PID   0xfeaau

// ----------------------------------------------------------------------------
// ENDPOINT ADDRESSES
// ----------------------------------------------------------------------------

#define EPNUM_AUDIO_OUT     0x01
#define EPNUM_AUDIO_FB      0x81

// ----------------------------------------------------------------------------
// INTERFACE NUMBERS
// ----------------------------------------------------------------------------

#define ITF_NUM_AUDIO_CONTROL   0
#define ITF_NUM_AUDIO_STREAMING 1
#define ITF_NUM_VENDOR          2
#define ITF_NUM_TOTAL           3

// ----------------------------------------------------------------------------
// UAC2 ENTITY IDS
// ----------------------------------------------------------------------------

#define UAC2_ENTITY_CLOCK           0x04
#define UAC2_ENTITY_INPUT_TERMINAL  0x01
#define UAC2_ENTITY_FEATURE_UNIT    0x02
#define UAC2_ENTITY_OUTPUT_TERMINAL 0x03

// ----------------------------------------------------------------------------
// DEVICE DESCRIPTOR
// ----------------------------------------------------------------------------

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    // Use IAD for composite device
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_EP0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0200,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 1
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

// ----------------------------------------------------------------------------
// UAC2 AUDIO CONFIGURATION
// ----------------------------------------------------------------------------

// Audio parameters
#define AUDIO_SAMPLE_RATE       48000
#define AUDIO_N_CHANNELS        2
#define AUDIO_N_BYTES_PER_SAMPLE 2
#define AUDIO_N_BITS_PER_SAMPLE 16

// Endpoint sizes
#define AUDIO_EP_SIZE   TUD_AUDIO_EP_SIZE(AUDIO_SAMPLE_RATE, AUDIO_N_BYTES_PER_SAMPLE, AUDIO_N_CHANNELS)
#define AUDIO_FB_EP_SIZE 4

// UAC2 Stereo Speaker with Feedback Descriptor Length
// We build this manually since TinyUSB only has mono speaker macro
#define TUD_AUDIO_SPEAKER_STEREO_FB_DESC_LEN ( \
    TUD_AUDIO_DESC_IAD_LEN + \
    TUD_AUDIO_DESC_STD_AC_LEN + \
    TUD_AUDIO_DESC_CS_AC_LEN + \
    TUD_AUDIO_DESC_CLK_SRC_LEN + \
    TUD_AUDIO_DESC_INPUT_TERM_LEN + \
    TUD_AUDIO_DESC_OUTPUT_TERM_LEN + \
    TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN + \
    TUD_AUDIO_DESC_STD_AS_INT_LEN + \
    TUD_AUDIO_DESC_STD_AS_INT_LEN + \
    TUD_AUDIO_DESC_CS_AS_INT_LEN + \
    TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN + \
    TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN + \
    TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN + \
    TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN \
)

// Configuration total length
#define CONFIG_TOTAL_LEN ( \
    TUD_CONFIG_DESC_LEN + \
    TUD_AUDIO_SPEAKER_STEREO_FB_DESC_LEN + \
    9 /* Vendor Interface */ \
)

static uint8_t const desc_configuration[] = {
    // Configuration Descriptor
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // ==========================================================================
    // UAC2 AUDIO FUNCTION
    // ==========================================================================

    // Interface Association Descriptor
    TUD_AUDIO_DESC_IAD(ITF_NUM_AUDIO_CONTROL, 2, 0x00),

    // Standard AC Interface Descriptor
    TUD_AUDIO_DESC_STD_AC(ITF_NUM_AUDIO_CONTROL, 0x00, 0x00),

    // Class-Specific AC Interface Header Descriptor
    TUD_AUDIO_DESC_CS_AC(
        0x0200,  // bcdADC (UAC 2.0)
        AUDIO_FUNC_DESKTOP_SPEAKER,
        TUD_AUDIO_DESC_CLK_SRC_LEN + TUD_AUDIO_DESC_INPUT_TERM_LEN +
        TUD_AUDIO_DESC_OUTPUT_TERM_LEN + TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN,
        AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS
    ),

    // Clock Source Descriptor (4.7.2.1)
    // Use INT_PRO_CLK since we support multiple sample rates (44.1k, 48k)
    TUD_AUDIO_DESC_CLK_SRC(
        UAC2_ENTITY_CLOCK,                                        // bClockID
        AUDIO_CLOCK_SOURCE_ATT_INT_PRO_CLK,                       // bmAttributes - internal programmable clock
        (AUDIO_CTRL_RW << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS),   // bmControls - sample rate RW
        UAC2_ENTITY_INPUT_TERMINAL,                               // bAssocTerminal
        0x00                                                      // iClockSource
    ),

    // Input Terminal Descriptor (4.7.2.4) - USB Streaming input
    TUD_AUDIO_DESC_INPUT_TERM(
        UAC2_ENTITY_INPUT_TERMINAL,                               // bTerminalID
        AUDIO_TERM_TYPE_USB_STREAMING,                            // wTerminalType
        0x00,                                                     // bAssocTerminal
        UAC2_ENTITY_CLOCK,                                        // bCSourceID
        AUDIO_N_CHANNELS,                                         // bNrChannels
        AUDIO_CHANNEL_CONFIG_NON_PREDEFINED,                      // bmChannelConfig
        0x00,                                                     // iChannelNames
        0x0000,                                                   // bmControls
        0x00                                                      // iTerminal
    ),

    // Output Terminal Descriptor (4.7.2.5) - Speaker output
    TUD_AUDIO_DESC_OUTPUT_TERM(
        UAC2_ENTITY_OUTPUT_TERMINAL,                              // bTerminalID
        AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER,                      // wTerminalType
        UAC2_ENTITY_INPUT_TERMINAL,                               // bAssocTerminal
        UAC2_ENTITY_FEATURE_UNIT,                                 // bSourceID
        UAC2_ENTITY_CLOCK,                                        // bCSourceID
        0x0000,                                                   // bmControls
        0x00                                                      // iTerminal
    ),

    // Feature Unit Descriptor (4.7.2.8) - Mute + Volume, 2 channels
    TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL(
        UAC2_ENTITY_FEATURE_UNIT,                                 // bUnitID
        UAC2_ENTITY_INPUT_TERMINAL,                               // bSourceID
        (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS),  // Master
        (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS),  // Ch1
        (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS),  // Ch2
        0x00                                                      // iFeature
    ),

    // ==========================================================================
    // Audio Streaming Interface (Alt 0 - Zero Bandwidth)
    // ==========================================================================
    TUD_AUDIO_DESC_STD_AS_INT(ITF_NUM_AUDIO_STREAMING, 0x00, 0x00, 0x00),

    // ==========================================================================
    // Audio Streaming Interface (Alt 1 - Operational)
    // ==========================================================================
    TUD_AUDIO_DESC_STD_AS_INT(ITF_NUM_AUDIO_STREAMING, 0x01, 0x02, 0x00),

    // Class-Specific AS Interface Descriptor (4.9.2)
    TUD_AUDIO_DESC_CS_AS_INT(
        UAC2_ENTITY_INPUT_TERMINAL,                               // bTerminalLink
        AUDIO_CTRL_NONE,                                          // bmControls
        AUDIO_FORMAT_TYPE_I,                                      // bFormatType
        AUDIO_DATA_FORMAT_TYPE_I_PCM,                             // bmFormats
        AUDIO_N_CHANNELS,                                         // bNrChannels
        AUDIO_CHANNEL_CONFIG_NON_PREDEFINED,                      // bmChannelConfig
        0x00                                                      // iChannelNames
    ),

    // Type I Format Type Descriptor (2.3.1.6)
    TUD_AUDIO_DESC_TYPE_I_FORMAT(AUDIO_N_BYTES_PER_SAMPLE, AUDIO_N_BITS_PER_SAMPLE),

    // Standard AS Isochronous Audio Data Endpoint Descriptor (4.10.1.1)
    TUD_AUDIO_DESC_STD_AS_ISO_EP(
        EPNUM_AUDIO_OUT,
        (uint8_t)(TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA),
        AUDIO_EP_SIZE,
        0x01                                                      // bInterval
    ),

    // Class-Specific AS Isochronous Audio Data Endpoint Descriptor (4.10.1.2)
    TUD_AUDIO_DESC_CS_AS_ISO_EP(
        AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,            // bmAttributes
        AUDIO_CTRL_NONE,                                          // bmControls
        AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC,         // bLockDelayUnits
        0x0001                                                    // wLockDelay = 1ms
    ),

    // Standard AS Isochronous Feedback Endpoint Descriptor
    TUD_AUDIO_DESC_STD_AS_ISO_FB_EP(EPNUM_AUDIO_FB, AUDIO_FB_EP_SIZE, 1),

    // ==========================================================================
    // INTERFACE 2: Vendor-Specific (WinUSB - Control Only)
    // ==========================================================================
    9,                                      // bLength
    TUSB_DESC_INTERFACE,                    // bDescriptorType
    ITF_NUM_VENDOR,                         // bInterfaceNumber
    0,                                      // bAlternateSetting
    0,                                      // bNumEndpoints
    TUSB_CLASS_VENDOR_SPECIFIC,             // bInterfaceClass
    0,                                      // bInterfaceSubClass
    0,                                      // bInterfaceProtocol
    0                                       // iInterface
};

// Compile-time verification that descriptor lengths match tusb_config.h
_Static_assert(TUD_AUDIO_SPEAKER_STEREO_FB_DESC_LEN == CFG_TUD_AUDIO_FUNC_1_DESC_LEN,
               "Audio descriptor length mismatch! Update CFG_TUD_AUDIO_FUNC_1_DESC_LEN in tusb_config.h");
_Static_assert(sizeof(desc_configuration) == CONFIG_TOTAL_LEN,
               "Configuration descriptor size mismatch!");

// Debug counter for configuration descriptor requests
volatile uint32_t usb_config_requests = 0;

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    usb_config_requests++;
    return desc_configuration;
}

// ----------------------------------------------------------------------------
// STRING DESCRIPTORS
// ----------------------------------------------------------------------------

static char const* string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: Language ID (English)
    descriptor_str_vendor,       // 1: Manufacturer
    descriptor_str_product,      // 2: Product
    descriptor_str_serial        // 3: Serial
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }

        const char* str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}

// ----------------------------------------------------------------------------
// MICROSOFT WCID / OS DESCRIPTORS
// ----------------------------------------------------------------------------

#define MS_OS_STRING_DESC_LEN 18
static const uint8_t ms_os_string_descriptor[MS_OS_STRING_DESC_LEN] = {
    MS_OS_STRING_DESC_LEN,
    TUSB_DESC_STRING,
    'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
    MS_VENDOR_CODE,
    0x00
};

#define MS_COMPAT_ID_DESC_LEN 40
static const uint8_t ms_compat_id_descriptor[MS_COMPAT_ID_DESC_LEN] = {
    U32_TO_U8S_LE(MS_COMPAT_ID_DESC_LEN),
    U16_TO_U8S_LE(0x0100),
    U16_TO_U8S_LE(0x0004),
    0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ITF_NUM_VENDOR,
    0x01,
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define MS_EXT_PROP_DESC_LEN 142
static const uint8_t ms_ext_prop_descriptor[MS_EXT_PROP_DESC_LEN] = {
    U32_TO_U8S_LE(MS_EXT_PROP_DESC_LEN),
    U16_TO_U8S_LE(0x0100),
    U16_TO_U8S_LE(0x0005),
    U16_TO_U8S_LE(0x0001),
    U32_TO_U8S_LE(132),
    U32_TO_U8S_LE(1),
    U16_TO_U8S_LE(40),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 0x00, 0x00,
    U32_TO_U8S_LE(78),
    '{', 0x00, '8', 0x00, '8', 0x00, 'B', 0x00, 'A', 0x00, 'E', 0x00, '0', 0x00, '3', 0x00,
    '2', 0x00, '-', 0x00, '5', 0x00, 'A', 0x00, '8', 0x00, '1', 0x00, '-', 0x00, '4', 0x00,
    '9', 0x00, 'F', 0x00, '0', 0x00, '-', 0x00, 'B', 0x00, 'C', 0x00, '3', 0x00, 'D', 0x00,
    '-', 0x00, 'A', 0x00, '4', 0x00, 'F', 0x00, 'F', 0x00, '1', 0x00, '3', 0x00, '8', 0x00,
    '2', 0x00, '1', 0x00, '6', 0x00, 'D', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00
};

bool handle_ms_vendor_request(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->bRequest != MS_VENDOR_CODE) return false;

    switch (request->wIndex) {
        case 0x0004:
            return tud_control_xfer(rhport, request, (void*)ms_compat_id_descriptor,
                                    TU_MIN(request->wLength, MS_COMPAT_ID_DESC_LEN));
        case 0x0005:
            return tud_control_xfer(rhport, request, (void*)ms_ext_prop_descriptor,
                                    TU_MIN(request->wLength, MS_EXT_PROP_DESC_LEN));
    }

    return false;
}
