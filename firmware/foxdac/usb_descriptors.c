/*
 * USB Descriptors for DSPi
 * UAC1 Device, Configuration, and MS OS descriptor data
 */

#include "usb_descriptors.h"

// ----------------------------------------------------------------------------
// STRING DESCRIPTORS
// ----------------------------------------------------------------------------

static char descriptor_str_serial_buf[17] = "0123456789ABCDEF";
char *usb_descriptor_str_serial = descriptor_str_serial_buf;

char *descriptor_strings[DESCRIPTOR_STRING_COUNT] = {
    "GitHub.com/WeebLabs",
    "Weeb Labs DSPi",
    descriptor_str_serial_buf
};

// ----------------------------------------------------------------------------
// CONFIGURATION DESCRIPTOR (LUFA-style UAC1)
// ----------------------------------------------------------------------------

const struct audio_device_config audio_device_config = {
    .descriptor = {
        .bLength             = sizeof(audio_device_config.descriptor),
        .bDescriptorType     = DTYPE_Configuration,
        .wTotalLength        = sizeof(audio_device_config),
        .bNumInterfaces      = ITF_NUM_TOTAL,
        .bConfigurationValue = 0x01,
        .iConfiguration      = 0x00,
        .bmAttributes        = 0x80,
        .bMaxPower           = 0x32,
    },
    .ac_interface = {
        .bLength            = sizeof(audio_device_config.ac_interface),
        .bDescriptorType    = DTYPE_Interface,
        .bInterfaceNumber   = ITF_NUM_AUDIO_CONTROL,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 0x00,
        .bInterfaceClass    = AUDIO_CSCP_AudioClass,
        .bInterfaceSubClass = AUDIO_CSCP_ControlSubclass,
        .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
        .iInterface         = 0x00,
    },
    .ac_audio = {
        .core = {
            .bLength = sizeof(audio_device_config.ac_audio.core),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Header,
            .bcdADC = VERSION_BCD(1, 0, 0),
            .wTotalLength = sizeof(audio_device_config.ac_audio),
            .bInCollection = 1,
            .bInterfaceNumbers = ITF_NUM_AUDIO_STREAMING,
        },
        .input_terminal = {
            .bLength = sizeof(audio_device_config.ac_audio.input_terminal),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_InputTerminal,
            .bTerminalID = 1,
            .wTerminalType = AUDIO_TERMINAL_STREAMING,
            .bAssocTerminal = 0,
            .bNrChannels = 2,
            .wChannelConfig = AUDIO_CHANNEL_LEFT_FRONT | AUDIO_CHANNEL_RIGHT_FRONT,
            .iChannelNames = 0,
            .iTerminal = 0,
        },
        .feature_unit = {
            .bLength = sizeof(audio_device_config.ac_audio.feature_unit),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Feature,
            .bUnitID = 2,
            .bSourceID = 1,
            .bControlSize = 1,
            .bmaControls = {AUDIO_FEATURE_MUTE | AUDIO_FEATURE_VOLUME, 0, 0},
            .iFeature = 0,
        },
        .output_terminal = {
            .bLength = sizeof(audio_device_config.ac_audio.output_terminal),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,
            .bTerminalID = 3,
            .wTerminalType = AUDIO_TERMINAL_OUT_SPEAKER,
            .bAssocTerminal = 0,
            .bSourceID = 2,
            .iTerminal = 0,
        },
    },
    .as_zero_interface = {
        .bLength            = sizeof(audio_device_config.as_zero_interface),
        .bDescriptorType    = DTYPE_Interface,
        .bInterfaceNumber   = ITF_NUM_AUDIO_STREAMING,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 0x00,
        .bInterfaceClass    = AUDIO_CSCP_AudioClass,
        .bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass,
        .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
        .iInterface         = 0x00,
    },
    .as_op_interface = {
        .bLength            = sizeof(audio_device_config.as_op_interface),
        .bDescriptorType    = DTYPE_Interface,
        .bInterfaceNumber   = ITF_NUM_AUDIO_STREAMING,
        .bAlternateSetting  = 0x01,
        .bNumEndpoints      = 0x02,
        .bInterfaceClass    = AUDIO_CSCP_AudioClass,
        .bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass,
        .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
        .iInterface         = 0x00,
    },
    .as_audio = {
        .streaming = {
            .bLength = sizeof(audio_device_config.as_audio.streaming),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_General,
            .bTerminalLink = 1,
            .bDelay = 1,
            .wFormatTag = 1, // PCM
        },
        .format = {
            .core = {
                .bLength = sizeof(audio_device_config.as_audio.format),
                .bDescriptorType = AUDIO_DTYPE_CSInterface,
                .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_FormatType,
                .bFormatType = 1,
                .bNrChannels = 2,
                .bSubFrameSize = 2,
                .bBitResolution = 16,
                .bSampleFrequencyType = count_of(audio_device_config.as_audio.format.freqs),
            },
            .freqs = {
                AUDIO_SAMPLE_FREQ(44100),
                AUDIO_SAMPLE_FREQ(48000),
                AUDIO_SAMPLE_FREQ(96000),
            },
        },
    },
    .ep1 = {
        .core = {
            .bLength          = sizeof(audio_device_config.ep1.core),
            .bDescriptorType  = DTYPE_Endpoint,
            .bEndpointAddress = AUDIO_OUT_ENDPOINT,
            .bmAttributes     = 5,        // Isochronous, async
            .wMaxPacketSize   = 384,      // 96kHz * 2ch * 2bytes / 1000
            .bInterval        = 1,
            .bRefresh         = 0,
            .bSyncAddr        = AUDIO_IN_ENDPOINT,
        },
        .audio = {
            .bLength = sizeof(audio_device_config.ep1.audio),
            .bDescriptorType = AUDIO_DTYPE_CSEndpoint,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSEndpoint_General,
            .bmAttributes = 1,            // Sampling frequency control
            .bLockDelayUnits = 0,
            .wLockDelay = 0,
        },
    },
    .ep2 = {
        .bLength          = sizeof(audio_device_config.ep2),
        .bDescriptorType  = 0x05,
        .bEndpointAddress = AUDIO_IN_ENDPOINT,
        .bmAttributes     = 0x11,         // Isochronous, feedback
        .wMaxPacketSize   = 3,
        .bInterval        = 0x01,
        .bRefresh         = 2,
        .bSyncAddr        = 0,
    },
    .vendor_interface = {
        .bLength            = sizeof(audio_device_config.vendor_interface),
        .bDescriptorType    = DTYPE_Interface,
        .bInterfaceNumber   = ITF_NUM_VENDOR,
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = 0x00,
        .bInterfaceClass    = 0xFF,       // Vendor specific
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
        .iInterface         = 0x00,
    },
};

// ----------------------------------------------------------------------------
// DEVICE DESCRIPTOR
// ----------------------------------------------------------------------------

const struct usb_device_descriptor boot_device_descriptor = {
    .bLength            = 18,
    .bDescriptorType    = 0x01,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 0x40,
    .idVendor           = VENDOR_ID,
    .idProduct          = PRODUCT_ID,
    .bcdDevice          = 0x0200,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

// ----------------------------------------------------------------------------
// MICROSOFT WCID / OS DESCRIPTORS
// ----------------------------------------------------------------------------

const uint8_t ms_os_string_descriptor[MS_OS_STRING_DESC_LEN] = {
    MS_OS_STRING_DESC_LEN,
    0x03, // DTYPE_String
    'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
    MS_VENDOR_CODE,
    0x00
};

const uint8_t ms_compat_id_descriptor[MS_COMPAT_ID_DESC_LEN] = {
    // Header
    0x28, 0x00, 0x00, 0x00, // dwLength = 40
    0x00, 0x01,             // bcdVersion = 1.0
    0x04, 0x00,             // wIndex = 4 (compat ID)
    0x01,                   // bCount = 1
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved
    // Function section
    ITF_NUM_VENDOR,         // bFirstInterfaceNumber
    0x01,                   // reserved (must be 0x01 per MS spec)
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00, // compatibleID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // subCompatibleID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // reserved
};

const uint8_t ms_ext_prop_descriptor[MS_EXT_PROP_DESC_LEN] = {
    // Header
    0x8e, 0x00, 0x00, 0x00, // dwLength = 142
    0x00, 0x01,             // bcdVersion = 1.0
    0x05, 0x00,             // wIndex = 5 (ext props)
    0x01, 0x00,             // wCount = 1
    // Property section
    0x84, 0x00, 0x00, 0x00, // dwSize = 132
    0x01, 0x00, 0x00, 0x00, // dwPropertyDataType = 1 (REG_SZ)
    0x28, 0x00,             // wPropertyNameLength = 40
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 0x00, 0x00,
    0x4e, 0x00, 0x00, 0x00, // dwPropertyDataLength = 78
    '{', 0x00, '8', 0x00, '8', 0x00, 'B', 0x00, 'A', 0x00, 'E', 0x00, '0', 0x00, '3', 0x00,
    '2', 0x00, '-', 0x00, '5', 0x00, 'A', 0x00, '8', 0x00, '1', 0x00, '-', 0x00, '4', 0x00,
    '9', 0x00, 'F', 0x00, '0', 0x00, '-', 0x00, 'B', 0x00, 'C', 0x00, '3', 0x00, 'D', 0x00,
    '-', 0x00, 'A', 0x00, '4', 0x00, 'F', 0x00, 'F', 0x00, '1', 0x00, '3', 0x00, '8', 0x00,
    '2', 0x00, '1', 0x00, '6', 0x00, 'D', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00
};
