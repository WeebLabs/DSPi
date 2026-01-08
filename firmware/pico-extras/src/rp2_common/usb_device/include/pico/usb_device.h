/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USB_USB_DEVICE_H
#define _USB_USB_DEVICE_H

#include "usb/usb_common.h"
#include "pico/assert.h"

#ifndef PICO_USBDEV_ENABLE_DEBUG_TRACE
#define PICO_USBDEV_ENABLE_DEBUG_TRACE 0
#endif

#ifndef PICO_USBDEV_ASSUME_ZERO_INIT
#define PICO_USBDEV_ASSUME_ZERO_INIT 0
#endif

#ifndef PICO_USBDEV_MAX_ENDPOINTS
#define PICO_USBDEV_MAX_ENDPOINTS USB_NUM_ENDPOINTS
#endif
static_assert(PICO_USBDEV_MAX_ENDPOINTS >= 1 && PICO_USBDEV_MAX_ENDPOINTS <= 16, "");

#ifndef PICO_USBDEV_MAX_DESCRIPTOR_SIZE
#define PICO_USBDEV_MAX_DESCRIPTOR_SIZE 64
#endif

// Enabling configuration items can reduce the size of the runtime code at the cost of some functionality
// or improve speed at the cost of some flexibility

// no custom per device setup packet handler
#ifndef PICO_USBDEV_NO_DEVICE_SETUP_HANDLER
#define PICO_USBDEV_NO_DEVICE_SETUP_HANDLER 0
#endif

// no custom per endpoint setup packet handlers
#ifndef PICO_USBDEV_NO_ENDPOINT_SETUP_HANDLER
#define PICO_USBDEV_NO_ENDPOINT_SETUP_HANDLER 0
#endif

// if all endpoints are bulk, then it allows simplification of some code
#ifndef PICO_USBDEV_BULK_ONLY_EP1_THRU_16
#define PICO_USBDEV_BULK_ONLY_EP1_THRU_16 0
#endif

// our interfaces are zero based number in the order they appear on the device - require that
#ifndef PICO_USBDEV_USE_ZERO_BASED_INTERFACES
#define PICO_USBDEV_USE_ZERO_BASED_INTERFACES 0
#endif

// no_init method for transfer
#ifndef PICO_USBDEV_NO_TRANSFER_ON_INIT_METHOD
#define PICO_USBDEV_NO_TRANSFER_ON_INIT_METHOD 0
#endif

// do on_cancel method for transfer
#ifndef PICO_USBDEV_NO_TRANSFER_ON_CANCEL_METHOD
#define PICO_USBDEV_NO_TRANSFER_ON_CANCEL_METHOD 0
#endif

// todo this needs to be part of configuration
#ifndef PICO_USBDEV_NO_INTERFACE_ALTERNATES
#define PICO_USBDEV_NO_INTERFACE_ALTERNATES 0
#endif

// todo this needs to be part of configuration
#ifndef PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE
#define PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE 0
#endif

static_assert((PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE >= 0) && (PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE < 4),
              "");

// don't zero out most structures (since we do so globablly for BSS)
//#define USB_SKIP_COMMON_INIT

// only 16 bytes saved to not set a sense code
//#define USB_SILENT_FAIL_ON_EXCLUSIVE

struct usb_transfer;
struct usb_endpoint;

typedef void (*usb_transfer_func)(struct usb_endpoint *ep);
typedef void (*usb_transfer_completed_func)(struct usb_endpoint *ep, struct usb_transfer *transfer);

struct usb_buffer {
    uint8_t *data;
    uint16_t data_len; // uint8_t -> uint16_t to fit 96kHz audio packets
    uint16_t data_max; // ^
    // then...
    bool valid; // aka user owned
};

#include "usb_device_private.h"

struct usb_transfer_type {
    // for IN transfers this is called to setup new packet buffers
    // for OUT transfers this is called with packet data
    //
    // In any case usb_packet_done must be called if this function has handled the buffer
    usb_transfer_func on_packet;
#if !PICO_USBDEV_NO_TRANSFER_ON_INIT_METHOD
    usb_transfer_func on_init;
#endif
#if !PICO_USBDEV_NO_TRANSFER_ON_CANCEL_METHOD
    usb_transfer_func on_cancel;
#endif
    uint8_t initial_packet_count;
};

struct usb_interface *usb_interface_init(struct usb_interface *interface, const struct usb_interface_descriptor *desc,
                                         struct usb_endpoint *const *endpoints, uint endpoint_count,
                                         bool double_buffered);
struct usb_device *usb_device_init(const struct usb_device_descriptor *desc,
                                   const struct usb_configuration_descriptor *config_desc,
                                   struct usb_interface *const *interfaces, uint interface_count,
                                   const char *(*get_descriptor_string)(uint index));

void usb_device_start();
void usb_device_stop();

// explicit stall
void usb_halt_endpoint_on_condition(struct usb_endpoint *ep);
void usb_halt_endpoint(struct usb_endpoint *endpoint);
void usb_clear_halt_condition(struct usb_endpoint *ep);
static inline bool usb_is_endpoint_stalled(struct usb_endpoint *endpoint);
void usb_set_default_transfer(struct usb_endpoint *ep, struct usb_transfer *transfer);
void usb_reset_transfer(struct usb_transfer *transfer, const struct usb_transfer_type *type,
                        usb_transfer_completed_func on_complete);
void usb_start_transfer(struct usb_endpoint *ep, struct usb_transfer *transfer);
void usb_reset_and_start_transfer(struct usb_endpoint *ep, struct usb_transfer *transfer,
                                  const struct usb_transfer_type *type, usb_transfer_completed_func on_complete);
void usb_chain_transfer(struct usb_endpoint *ep, struct usb_transfer *transfer);
void usb_grow_transfer(struct usb_transfer *transfer, uint packet_count);
void usb_start_default_transfer_if_not_already_running_or_halted(struct usb_endpoint *ep);

typedef void (*usb_transfer_func)(struct usb_endpoint *ep);

struct usb_buffer *usb_current_in_packet_buffer(struct usb_endpoint *ep);
struct usb_buffer *usb_current_out_packet_buffer(struct usb_endpoint *ep);
uint8_t *usb_get_single_packet_response_buffer(struct usb_endpoint *ep, uint len);

// call during (or asynchronously after) on_packet to mark the packet as done
void usb_packet_done(struct usb_endpoint *ep);

extern const struct usb_transfer_type usb_current_packet_only_transfer_type;

static inline struct usb_endpoint *usb_get_control_in_endpoint();
static inline struct usb_endpoint *usb_get_control_out_endpoint();

void usb_start_empty_control_in_transfer(usb_transfer_completed_func on_complete);
void usb_start_empty_control_in_transfer_null_completion();
void usb_start_tiny_control_in_transfer(uint32_t data, uint len);
void usb_start_single_buffer_control_in_transfer();
void usb_start_control_out_transfer(const struct usb_transfer_type *type);
void usb_start_empty_transfer(struct usb_endpoint *endpoint, struct usb_transfer *transfer,
                              usb_transfer_completed_func on_complete);

void usb_soft_reset_endpoint(struct usb_endpoint *ep);
void usb_hard_reset_endpoint(struct usb_endpoint *ep);

#if PICO_USBDEV_ENABLE_DEBUG_TRACE
void usb_dump_trace(void);
void usb_reset_trace(void);
#else

static inline void usb_dump_trace() {}

static inline void usb_reset_trace() {}

#endif

#define usb_warn(format, args...) ({printf("WARNING: "); printf(format, ## args); })
#if false && !defined(NDEBUG)
#define usb_debug(format,args...) printf(format, ## args)
#else
#define usb_debug(format, ...) ((void)0)
#endif

#if false && !defined(NDEBUG)
#define usb_trace(format,args...) printf(format, ## args)
#else
#define usb_trace(format, ...) ((void)0)
#endif

#endif
