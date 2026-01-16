#include "usb_descriptors.h"
#include "config.h"
#include <string.h>

// ----------------------------------------------------------------------------
// DESCRIPTOR STRINGS
// ----------------------------------------------------------------------------

static char descriptor_str_vendor[] = "H3 & astanoev.com";
static char descriptor_str_product[] = "Pico DSP 2.1 (WinUSB)";

// Removed 'static' so main.c can write the board ID
char descriptor_str_serial[17] = "0123456789ABCDEF";

static char *descriptor_strings[] = {
    descriptor_str_vendor,
    descriptor_str_product,
    descriptor_str_serial
};

const char *_get_descriptor_string(uint index) {
    if (index >= 1 && index <= 3) {
        return descriptor_strings[index - 1];
    }
    return "";
}

// ----------------------------------------------------------------------------
// USB IDS
// ----------------------------------------------------------------------------

#define VENDOR_ID   0x2e8au
#define PRODUCT_ID  0xfeaau

// Endpoint addresses
#define AUDIO_OUT_ENDPOINT  0x01U
#define AUDIO_IN_ENDPOINT   0x82U

// ----------------------------------------------------------------------------
// DEVICE DESCRIPTOR
// ----------------------------------------------------------------------------

const struct usb_device_descriptor boot_device_descriptor = {
    .bLength = 18,
    .bDescriptorType = 0x01,        // DEVICE
    .bcdUSB = 0x0200,               // USB 2.0 (needed for WCID)
    .bDeviceClass = 0x00,           // Defined at interface level
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 0x40,
    .idVendor = VENDOR_ID,
    .idProduct = PRODUCT_ID,
    .bcdDevice = 0x0200,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

// ----------------------------------------------------------------------------
// CONFIGURATION DESCRIPTOR (includes all interfaces)
// ----------------------------------------------------------------------------

const struct audio_device_config audio_device_config = {
    // Configuration Descriptor
    .descriptor = {
        .bLength = sizeof(audio_device_config.descriptor),
        .bDescriptorType = DTYPE_Configuration,
        .wTotalLength = sizeof(audio_device_config),
        .bNumInterfaces = 3,        // AC + AS + Vendor
        .bConfigurationValue = 0x01,
        .iConfiguration = 0x00,
        .bmAttributes = 0x80,       // Bus-powered
        .bMaxPower = 0x32,          // 100mA
    },
    
    // ========================================================================
    // INTERFACE 0: Audio Control
    // ========================================================================
    .ac_interface = {
        .bLength = sizeof(audio_device_config.ac_interface),
        .bDescriptorType = DTYPE_Interface,
        .bInterfaceNumber = 0x00,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x00,
        .bInterfaceClass = AUDIO_CSCP_AudioClass,
        .bInterfaceSubClass = AUDIO_CSCP_ControlSubclass,
        .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
        .iInterface = 0x00,
    },
    .ac_audio = {
        .core = {
            .bLength = sizeof(audio_device_config.ac_audio.core),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Header,
            .bcdADC = VERSION_BCD(1, 0, 0),
            .wTotalLength = sizeof(audio_device_config.ac_audio),
            .bInCollection = 1,
            .bInterfaceNumbers = 1  // AS interface number
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
            .iTerminal = 0
        },
        .feature_unit = {
            .bLength = sizeof(audio_device_config.ac_audio.feature_unit),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Feature,
            .bUnitID = 2,
            .bSourceID = 1,
            .bControlSize = 1,
            .bmaControls = {AUDIO_FEATURE_MUTE | AUDIO_FEATURE_VOLUME, 0, 0},
            .iFeature = 0
        },
        .output_terminal = {
            .bLength = sizeof(audio_device_config.ac_audio.output_terminal),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,
            .bTerminalID = 3,
            .wTerminalType = AUDIO_TERMINAL_OUT_SPEAKER,
            .bAssocTerminal = 0,
            .bSourceID = 2,
            .iTerminal = 0
        },
    },
    
    // ========================================================================
    // INTERFACE 1: Audio Streaming (Alt 0 = zero bandwidth)
    // ========================================================================
    .as_zero_interface = {
        .bLength = sizeof(audio_device_config.as_zero_interface),
        .bDescriptorType = DTYPE_Interface,
        .bInterfaceNumber = 0x01,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x00,
        .bInterfaceClass = AUDIO_CSCP_AudioClass,
        .bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass,
        .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
        .iInterface = 0x00
    },
    
    // ========================================================================
    // INTERFACE 1: Audio Streaming (Alt 1 = operational)
    // ========================================================================
    .as_op_interface = {
        .bLength = sizeof(audio_device_config.as_op_interface),
        .bDescriptorType = DTYPE_Interface,
        .bInterfaceNumber = 0x01,
        .bAlternateSetting = 0x01,
        .bNumEndpoints = 0x02,
        .bInterfaceClass = AUDIO_CSCP_AudioClass,
        .bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass,
        .bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
        .iInterface = 0x00
    },
    .as_audio = {
        .streaming = {
            .bLength = sizeof(audio_device_config.as_audio.streaming),
            .bDescriptorType = AUDIO_DTYPE_CSInterface,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_General,
            .bTerminalLink = 1,
            .bDelay = 1,
            .wFormatTag = 1  // PCM
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
                .bSampleFrequencyType = 2  // 2 discrete frequencies
            },
            .freqs = {
                AUDIO_SAMPLE_FREQ(44100),
                AUDIO_SAMPLE_FREQ(48000)
            },
        },
    },
    // Audio OUT endpoint (isochronous)
    .ep1 = {
        .core = {
            .bLength = sizeof(audio_device_config.ep1.core),
            .bDescriptorType = DTYPE_Endpoint,
            .bEndpointAddress = AUDIO_OUT_ENDPOINT,
            .bmAttributes = 5,      // Isochronous, async
            .wMaxPacketSize = 384,
            .bInterval = 1,
            .bRefresh = 0,
            .bSyncAddr = AUDIO_IN_ENDPOINT
        },
        .audio = {
            .bLength = sizeof(audio_device_config.ep1.audio),
            .bDescriptorType = AUDIO_DTYPE_CSEndpoint,
            .bDescriptorSubtype = AUDIO_DSUBTYPE_CSEndpoint_General,
            .bmAttributes = 1,      // Sampling frequency control
            .bLockDelayUnits = 0,
            .wLockDelay = 0
        }
    },
    // Audio sync feedback endpoint (isochronous IN)
    .ep2 = {
        .bLength = sizeof(audio_device_config.ep2),
        .bDescriptorType = 0x05,    // ENDPOINT
        .bEndpointAddress = AUDIO_IN_ENDPOINT,
        .bmAttributes = 0x11,       // Isochronous, feedback
        .wMaxPacketSize = 3,
        .bInterval = 0x01,
        .bRefresh = 2,
        .bSyncAddr = 0
    },
    
    // ========================================================================
    // INTERFACE 2: Vendor-Specific (WinUSB - Control Only)
    // ========================================================================
    .vendor_interface = {
        .bLength = sizeof(audio_device_config.vendor_interface),
        .bDescriptorType = DTYPE_Interface,
        .bInterfaceNumber = VENDOR_INTERFACE_NUMBER,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x00,      // Zero endpoints
        .bInterfaceClass = 0xFF,    // Vendor-specific
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
        .iInterface = 0x00
    }
};
