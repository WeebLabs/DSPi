/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USB_DEVICE_PRIVATE_H
#define _USB_DEVICE_PRIVATE_H

#include "usb/usb_common.h"
#include <stdbool.h>

struct usb_transfer {
    // prototype
    const struct usb_transfer_type *type;
    usb_transfer_completed_func on_complete;
    // total number of buffers (packets) that still need to be handed over to the hardware
    // during the remaining course of the transfer (with data for IN, empty for data for out)
    uint32_t remaining_packets_to_submit;
    // total number of buffers when we will expect to receive IRQ/handle_buffer for during
    // the remaining course of the transfer
    uint32_t remaining_packets_to_handle;
#ifdef GENERAL_SIZE_HACKS
    union {
        struct {
#endif
    // number of packets which require usb_packet_done()
    bool outstanding_packet;
    // received a packet which we couldn't deliver because there was one outstanding
    bool packet_queued;
    bool started;
    bool completed;
#ifdef GENERAL_SIZE_HACKS
    };
    uint32_t all_flags;
};
#endif
};

struct usb_interface {
    const struct usb_interface_descriptor *descriptor;
    struct usb_endpoint *const *endpoints;
    uint8_t endpoint_count;
    bool (*setup_request_handler)(struct usb_interface *interface, struct usb_setup_packet *setup);
#if !PICO_USBDEV_NO_INTERFACE_ALTERNATES
    bool (*set_alternate_handler)(struct usb_interface *interface, uint alt);
    uint8_t alt;
#endif
};

struct usb_configuration {
    const struct usb_configuration_descriptor *descriptor;
    struct usb_interface *const *interfaces;
#ifndef PICO_USBDEV_FIXED_INTERFACE_COUNT
    uint8_t interface_count;
#endif
};
#ifdef PICO_USBDEV_FIXED_INTERFACE_COUNT
#define _usb_interface_count(config) PICO_USBDEV_FIXED_INTERFACE_COUNT
#else
#define _usb_interface_count(config) config->interface_count
#endif

struct usb_device {
    const struct usb_device_descriptor *descriptor;
#if !PICO_USBDEV_NO_DEVICE_SETUP_HANDLER
    bool (*setup_request_handler)(struct usb_device *dev, struct usb_setup_packet *setup);
#endif
    void (*on_configure)(struct usb_device *dev, bool configured);
    const char *(*get_descriptor_string)(uint index);
    // only support one config for now
    struct usb_configuration config;
    uint8_t current_address; // 0 if unaddressed
    uint8_t current_config_num; // 0 if unconfigured
    uint8_t pending_address; // address to set on completion of SET_ADDRESS CSW
    uint16_t next_buffer_offset;
//    bool started;
};

enum usb_halt_state {
    HS_NONE = 0,
    HS_NON_HALT_STALL = 1, // just stalled
    HS_HALTED = 2, // halted or command halted
    HS_HALTED_ON_CONDITION = 3 // halted that cannot be simply cleared by CLEAR_FEATURE
};

struct usb_endpoint {
    const struct usb_endpoint_descriptor *descriptor;
    struct usb_transfer *default_transfer;
    struct usb_transfer *current_transfer;
    struct usb_transfer *chain_transfer;
    void (*on_stall_change)(struct usb_endpoint *ep);
#if !PICO_USBDEV_NO_ENDPOINT_SETUP_HANDLER
    bool (*setup_request_handler)(struct usb_endpoint *ep, struct usb_setup_packet *setup);
#endif
    uint16_t dpram_buffer_offset;
    uint16_t buffer_size; // for an individual buffer
    struct usb_buffer current_hw_buffer;
#if !PICO_USBDEV_BULK_ONLY_EP1_THRU_16
    uint16_t buffer_stride;
#endif
    uint8_t num;
    uint8_t next_pid;
    uint8_t buffer_bit_index;
    uint8_t owned_buffer_count;
    uint8_t current_give_buffer;
    uint8_t current_take_buffer;
    uint8_t halt_state;
    bool first_buffer_after_reset;
    bool double_buffered;
    bool in;
};

static inline uint usb_endpoint_number(struct usb_endpoint *ep) {
    assert(ep);
    return ep->descriptor ? ep->descriptor->bEndpointAddress & 0xfu : 0;
}

static inline bool usb_is_endpoint_stalled(struct usb_endpoint *endpoint) {
    return endpoint->halt_state != HS_NONE;
}

const char *usb_endpoint_dir_string(struct usb_endpoint *ep);

static inline struct usb_endpoint *usb_get_control_in_endpoint() {
    extern struct usb_endpoint usb_control_in;
    return &usb_control_in;
}

static inline struct usb_endpoint *usb_get_control_out_endpoint() {
    extern struct usb_endpoint usb_control_out;
    return &usb_control_out;
}

#endif //_USB_DEVICE_PRIVATE_H
