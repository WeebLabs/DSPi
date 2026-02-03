/*
 * USB Descriptors for DSPi
 * UAC1 Device, Configuration, and MS OS descriptor definitions
 */

#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include "pico/usb_device.h"
#include "lufa/AudioClassCommon.h"
#include "config.h"

// ----------------------------------------------------------------------------
// USB IDs
// ----------------------------------------------------------------------------

#define VENDOR_ID   0x2e8au
#define PRODUCT_ID  0xfeaau

// ----------------------------------------------------------------------------
// ENDPOINT ADDRESSES
// ----------------------------------------------------------------------------

#define AUDIO_OUT_ENDPOINT  0x01U
#define AUDIO_IN_ENDPOINT   0x82U

// ----------------------------------------------------------------------------
// INTERFACE NUMBERS
// ----------------------------------------------------------------------------

#define ITF_NUM_AUDIO_CONTROL   0
#define ITF_NUM_AUDIO_STREAMING 1
#define ITF_NUM_VENDOR          2
#define ITF_NUM_TOTAL           3

// ----------------------------------------------------------------------------
// AUDIO SAMPLE FREQUENCY MACRO (for descriptor byte encoding)
// ----------------------------------------------------------------------------

#undef AUDIO_SAMPLE_FREQ
#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

// ----------------------------------------------------------------------------
// CONFIGURATION DESCRIPTOR STRUCT (3 interfaces: AC, AS, Vendor)
// ----------------------------------------------------------------------------

struct audio_device_config {
    struct usb_configuration_descriptor descriptor;
    struct usb_interface_descriptor ac_interface;
    struct __packed {
        USB_Audio_StdDescriptor_Interface_AC_t core;
        USB_Audio_StdDescriptor_InputTerminal_t input_terminal;
        USB_Audio_StdDescriptor_FeatureUnit_t feature_unit;
        USB_Audio_StdDescriptor_OutputTerminal_t output_terminal;
    } ac_audio;
    struct usb_interface_descriptor as_zero_interface;
    struct usb_interface_descriptor as_op_interface;
    struct __packed {
        USB_Audio_StdDescriptor_Interface_AS_t streaming;
        struct __packed {
            USB_Audio_StdDescriptor_Format_t core;
            USB_Audio_SampleFreq_t freqs[3];
        } format;
    } as_audio;
    struct __packed {
        struct usb_endpoint_descriptor_long core;
        USB_Audio_StdDescriptor_StreamEndpoint_Spc_t audio;
    } ep1;
    struct usb_endpoint_descriptor_long ep2;
    struct usb_interface_descriptor vendor_interface;
};

// ----------------------------------------------------------------------------
// DESCRIPTOR INSTANCES
// ----------------------------------------------------------------------------

extern const struct audio_device_config audio_device_config;
extern const struct usb_device_descriptor boot_device_descriptor;

// ----------------------------------------------------------------------------
// STRING DESCRIPTORS
// ----------------------------------------------------------------------------

#define DESCRIPTOR_STRING_COUNT 3
extern char *descriptor_strings[DESCRIPTOR_STRING_COUNT];
extern char *usb_descriptor_str_serial;

// ----------------------------------------------------------------------------
// MICROSOFT OS / WCID DESCRIPTORS
// ----------------------------------------------------------------------------

#define MS_OS_STRING_DESC_LEN 18
#define MS_COMPAT_ID_DESC_LEN 40
#define MS_EXT_PROP_DESC_LEN  142

extern const uint8_t ms_os_string_descriptor[MS_OS_STRING_DESC_LEN];
extern const uint8_t ms_compat_id_descriptor[MS_COMPAT_ID_DESC_LEN];
extern const uint8_t ms_ext_prop_descriptor[MS_EXT_PROP_DESC_LEN];

#endif // USB_DESCRIPTORS_H
