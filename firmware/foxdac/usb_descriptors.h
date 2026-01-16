#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include "pico/stdlib.h"
#include "pico/usb_device.h"
#include "lufa/AudioClassCommon.h"
#include "config.h"

// ----------------------------------------------------------------------------
// DESCRIPTOR STRINGS
// ----------------------------------------------------------------------------
const char *_get_descriptor_string(uint index);

// Expose serial string so main.c can write to it
extern char descriptor_str_serial[17];

// ----------------------------------------------------------------------------
// AUDIO DEVICE CONFIG (with added vendor interface)
// ----------------------------------------------------------------------------

struct __attribute__((packed)) audio_device_config {
    struct usb_configuration_descriptor descriptor;
    
    // Interface 0: Audio Control
    struct usb_interface_descriptor ac_interface;
    struct __attribute__((packed)) {
        USB_Audio_StdDescriptor_Interface_AC_t core;
        USB_Audio_StdDescriptor_InputTerminal_t input_terminal;
        USB_Audio_StdDescriptor_FeatureUnit_t feature_unit;
        USB_Audio_StdDescriptor_OutputTerminal_t output_terminal;
    } ac_audio;
    
    // Interface 1: Audio Streaming (Alt 0 = zero-bandwidth)
    struct usb_interface_descriptor as_zero_interface;
    
    // Interface 1: Audio Streaming (Alt 1 = operational)
    struct usb_interface_descriptor as_op_interface;
    struct __attribute__((packed)) {
        USB_Audio_StdDescriptor_Interface_AS_t streaming;
        struct __attribute__((packed)) {
            USB_Audio_StdDescriptor_Format_t core;
            USB_Audio_SampleFreq_t freqs[2];
        } format;
    } as_audio;
    struct __attribute__((packed)) {
        struct usb_endpoint_descriptor_long core;
        USB_Audio_StdDescriptor_StreamEndpoint_Spc_t audio;
    } ep1;
    struct usb_endpoint_descriptor_long ep2;
    
    // Interface 2: Vendor-Specific (WinUSB - Control Only)
    // NOTE: Endpoints removed for Control-Only implementation
    struct usb_interface_descriptor vendor_interface;
};

extern const struct audio_device_config audio_device_config;
extern const struct usb_device_descriptor boot_device_descriptor;

#endif // USB_DESCRIPTORS_H
