/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "hardware/structs/usb.h"
#include "pico/usb_device.h"

#if PICO_USBDEV_MAX_DESCRIPTOR_SIZE > 64
#include "pico/usb_stream_helper.h"
#include "pico/fix/rp2040_usb_device_enumeration.h"
#endif

// -------------------------------------------------------------------------------------------------------------
// Note this is a small code size focused USB device abstraction, which also avoids using any mutable static
// data so it is easy to include in bootrom.
// -------------------------------------------------------------------------------------------------------------

//#define USB_SINGLE_BUFFERED

CU_REGISTER_DEBUG_PINS(usb_irq)
//CU_SELECT_DEBUG_PINS(usb_irq)

#if PICO_USBDEV_ENABLE_DEBUG_TRACE
static uint32_t debug_trace[128][2];
static volatile uint32_t trace_i;
#endif

// note we treat all errors the same (we just ignore)
#define USB_INTS_ERROR_BITS ( \
    USB_INTS_ERROR_DATA_SEQ_BITS      |  \
    USB_INTS_ERROR_BIT_STUFF_BITS     |  \
    USB_INTS_ERROR_CRC_BITS           |  \
    USB_INTS_ERROR_RX_OVERFLOW_BITS   |  \
    USB_INTS_ERROR_RX_TIMEOUT_BITS)

// define some macros so we implement different allocation schemes (right now we use bootrom which is no-alloc and assume zero)
#if PICO_USBDEV_ASSUME_ZERO_INIT
#define usb_init_clear_deref(x) ((void)0)
#else
#define usb_init_clear_deref(x) memset(x, 0, sizeof(*(x)))
#endif
#define usb_common_init(ep) ({assert(ep); usb_init_clear_deref(ep);  ep; })

#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

void _usb_transfer_current_packet_only(struct usb_endpoint *ep);

const struct usb_transfer_type usb_current_packet_only_transfer_type = {
        .on_packet = _usb_transfer_current_packet_only,
        .initial_packet_count = 1,
};

/**
 * Public ep 0 IN/OUT
 */
struct usb_endpoint usb_control_in, usb_control_out;

#if PICO_USBDEV_MAX_DESCRIPTOR_SIZE > 64
static struct usb_stream_transfer _control_in_stream_transfer;
#define _control_in_transfer _control_in_stream_transfer.core
#else
static struct usb_transfer _control_in_transfer;
#endif
static struct usb_transfer _control_out_transfer;

static struct usb_device _device;
static struct usb_endpoint *_endpoints[PICO_USBDEV_MAX_ENDPOINTS];

static inline const char *_in_out_string(bool in) {
    return in ? "IN" : "OUT";
}

/**
 * @param ep
 * @return a 32 bit pointer to both buffer control registers for an endpoint
 */
static io_rw_32 *_usb_buf_ctrl_wide(const struct usb_endpoint *ep) {
    return ep->in ? &usb_dpram->ep_buf_ctrl[ep->num].in : &usb_dpram->ep_buf_ctrl[ep->num].out;
}

/**
 * @param ep
 * @param which 0 or 1 double-buffer index
 * @return a 16 bit pointer to the specified (1 of 2) buffer control register for an endpoint
 */
static io_rw_16 *_usb_buf_ctrl_narrow(const struct usb_endpoint *ep, uint which) {
    return &((io_rw_16 *) _usb_buf_ctrl_wide(ep))[which];
}

static uint _ep_buffer_count(const struct usb_endpoint *ep) {
    return ep->double_buffered ? 2 : 1;
}

#ifndef NDEBUG
static void _usb_dump_eps(void)
{
    printf("\n");
    for (int num = 1; num < PICO_USBDEV_MAX_ENDPOINTS; num++) {
        for(int b = 0; b < 2; b++)
        {
            struct usb_endpoint *ep = _endpoints[num];
            uint16_t ctrl = (uint16_t) *_usb_buf_ctrl_narrow(ep, b);
            uint8_t pid = (ctrl & USB_BUF_CTRL_DATA1_PID) ? 1 : 0;
            printf("ep %d %s <= 0x%04x (DATA%d", ep->num,  usb_endpoint_dir_string(ep), ctrl, pid);
            if (ctrl & USB_BUF_CTRL_FULL)
            { printf(", FULL"); }
            if (ctrl & USB_BUF_CTRL_LAST)
            { printf(", LAST"); }
            if (ctrl & USB_BUF_CTRL_SEL)
            { printf(", SEL"); }
            printf(", LEN = %04x)\n", ctrl & USB_BUF_CTRL_LEN_MASK);
        }
    }
    usb_reset_trace();
}
#endif

/**
 * Reset the buffers for an endpoint to CPU ownership, aborting the buffers if necessary
 * @param ep
 */
static void _usb_reset_buffers(struct usb_endpoint *ep) {
    if ((USB_BUF_CTRL_AVAIL * 0x10001) & *_usb_buf_ctrl_wide((ep))) {
        usb_debug("Must abort buffers %d %s owned=%d %08x!!!\n", ep->num, usb_endpoint_dir_string(ep),
                  ep->owned_buffer_count, (uint) *_usb_buf_ctrl_wide(
                ep));
        // if the hardware owns 1 buffer, then when we reset we toggle the pid (in double-buffer mode it could own two)
        if (!ep->double_buffered || ep->owned_buffer_count == 1) {
            usb_debug("Toggling PID as buffers restored");
            ep->next_pid ^= 1u;
        }
        usb_dump_trace();
        uint32_t mask = 1u << ep->buffer_bit_index;
        usb_hw_clear->abort_done = mask;
        usb_hw_set->abort = mask;
        int count = 100000;
        while (!(usb_hw->abort_done & mask) && --count) {
            usb_hw_set->abort = mask;
        }
        if (!count) {
            usb_warn("**** FAILED TO ABORT %d %s: %08x %08x\n", ep->num, usb_endpoint_dir_string(ep),
                     (uint) usb_hw->abort, (uint) usb_hw->abort_done);
        }
        usb_hw_clear->abort = mask;
        usb_hw_clear->abort_done = mask;
    }
    *_usb_buf_ctrl_wide(ep) = 0;
    ep->owned_buffer_count = _ep_buffer_count(ep);
    usb_debug("clear current buffer %d %s\n", ep->num, usb_endpoint_dir_string(ep));
    ep->current_give_buffer = ep->current_take_buffer = 0;
    ep->first_buffer_after_reset = true;
}

/**
 * Stall the given endpoint
 *
 * @param ep
 * @param hs
 */
static void _usb_stall_endpoint(struct usb_endpoint *ep, enum usb_halt_state hs) {
    assert(hs);
    __unused enum usb_halt_state old_hs = ep->halt_state;
    if (!ep->halt_state) {
        if (ep->num == 0) {
            // A stall on EP0 has to be armed so it can be cleared on the next setup packet
            usb_hw_set->ep_stall_arm = ep->in ? USB_EP_STALL_ARM_EP0_IN_BITS : USB_EP_STALL_ARM_EP0_OUT_BITS;
        }
        *_usb_buf_ctrl_wide(ep) |= USB_BUF_CTRL_STALL;
        ep->halt_state = hs;
        if (ep->on_stall_change) ep->on_stall_change(ep);
    } else {
        // we should be stalled
        assert(USB_BUF_CTRL_STALL & *_usb_buf_ctrl_wide(ep));
        if (hs > ep->halt_state) ep->halt_state = hs;
    }
    usb_debug("Stall %d %s %d->%d\n", ep->num, usb_endpoint_dir_string(ep), old_hs, hs);
}

/**
 * Initialize any endpoint (0 or user defined)
 * @param ep
 * @param num
 * @param in
 * @param max_buffer_size
 * @param double_buffered
 * @return
 */
static __noinline struct usb_endpoint *_usb_endpoint_init_internal(struct usb_endpoint *ep,
                                                                   uint num,
                                                                   bool in,
                                                                   uint max_buffer_size,
                                                                   bool double_buffered) {
    // for some inling of memset reason, removing this makes the code larger!
    usb_common_init(ep);
    ep->num = num;
    ep->in = in;
    ep->buffer_size = max_buffer_size;
#ifndef USB_SINGLE_BUFFERED
    ep->double_buffered = double_buffered;
#endif
#if !PICO_USBDEV_BULK_ONLY_EP1_THRU_16
    ep->buffer_stride = 64;
#endif
    ep->buffer_bit_index = ((num * 2u) + (in ? 0u : 1u));
    return ep;
}

static uint32_t _usb_endpoint_stride(__unused struct usb_endpoint *ep) {
#if !PICO_USBDEV_BULK_ONLY_EP1_THRU_16
    return ep->buffer_stride;
#else
    return 64;
#endif
}

static void _usb_endpoint_hw_init(struct usb_endpoint *ep, __unused uintptr_t data) {
    uint ep_num = usb_endpoint_number(ep);
    usb_dpram->ep_buf_ctrl[ep_num].in = 0;
    usb_dpram->ep_buf_ctrl[ep_num].out = 0;
    ep->dpram_buffer_offset = _device.next_buffer_offset;
    usb_debug("endpoint %d %s buf at %04x %04xx%d\n", ep_num, usb_endpoint_dir_string(ep), ep->dpram_buffer_offset,
              ep->buffer_size, _ep_buffer_count(ep));
    uint32_t stride = _usb_endpoint_stride(ep);
    if (ep->double_buffered) stride <<= 1u;
    _device.next_buffer_offset += stride;
    assert(_device.next_buffer_offset <= USB_DPRAM_MAX);
    if (ep_num) {
        uint32_t reg = EP_CTRL_ENABLE_BITS
                       | (ep->double_buffered ? EP_CTRL_DOUBLE_BUFFERED_BITS : 0u)
                       | EP_CTRL_INTERRUPT_PER_BUFFER
                       //| EP_CTRL_INTERRUPT_ON_NAK
                       //                       | EP_CTRL_INTERRUPT_ON_STALL
                       | ep->dpram_buffer_offset
                       #if !PICO_USBDEV_BULK_ONLY_EP1_THRU_16
                       | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB);
#else
        | (USB_TRANSFER_TYPE_BULK << EP_CTRL_BUFFER_TYPE_LSB);
        assert(ep->descriptor->bmAttributes == USB_TRANSFER_TYPE_BULK);
#endif
        // todo coordinate with buff control
        if (ep->in) {
            usb_dpram->ep_ctrl[ep_num - 1].in = reg;
            usb_dpram->ep_ctrl[ep_num - 1].out = 0;
        } else {
            usb_dpram->ep_ctrl[ep_num - 1].in = 0;
            usb_dpram->ep_ctrl[ep_num - 1].out = reg;
        }
    }
}

typedef void (*endpoint_callback)(struct usb_endpoint *endpoint, uintptr_t data);

static void _usb_for_each_endpoint(endpoint_callback callback, bool include_control, uintptr_t data) {
    // note order is important here as the buffers are allocated in enumeration order
    if (include_control) {
        callback(&usb_control_in, data);
        callback(&usb_control_out, data);
    }
    for (uint i = 1; i < count_of(_endpoints); i++) {
        if (_endpoints[i]) {
            callback(_endpoints[i], data);
        }
    }
}

void _usb_transfer_current_packet_only(struct usb_endpoint *ep) {
//    printf("usb_transfer_current_packet_only %d %s\n", ep->num, usb_endpoint_dir_string(ep));
    if (ep->in) {
        assert(usb_current_in_packet_buffer(ep)->data_len <
               ep->buffer_size); // must not be buffer_size or we'd need two
    }
    usb_packet_done(ep);
}

static void _usb_reset_endpoint(struct usb_endpoint *ep, bool hard) {
    // ok we need to update the packet
#if !PICO_USBDEV_NO_TRANSFER_ON_CANCEL_METHOD
    if (ep->current_transfer && ep->current_transfer->type->on_cancel) {
        ep->current_transfer->type->on_cancel(ep);
    }
#endif
    ep->current_transfer = NULL;
    _usb_reset_buffers(ep); // hopefully a no-op
    if (hard) {
        // must be done after reset buffers above
        if (ep->next_pid) {
            usb_debug("Reset pid to 0 %d %s\n", ep->num, usb_endpoint_dir_string(ep));
        }
        ep->next_pid = 0;
    }
    ep->current_hw_buffer.valid = false;
    if (ep->halt_state) {
        ep->halt_state = HS_NONE;
        if (ep->on_stall_change) ep->on_stall_change(ep);
    }
    // note on_stall_change might have started a transfer
    if (_device.current_config_num && ep->default_transfer && !ep->current_transfer) {
        usb_debug("start default %d %s, nextpid = %d\n", ep->num, usb_endpoint_dir_string(ep), ep->next_pid);
        usb_reset_and_start_transfer(ep, ep->default_transfer, ep->default_transfer->type, 0);
    }
}

static void _usb_hard_reset_endpoint_callback(struct usb_endpoint *ep, __unused uintptr_t data) {
    usb_hard_reset_endpoint(ep);
}

static void _usb_handle_set_address(uint addr) {
    assert(!_device.current_config_num); // we expect to be unconfigured
    _device.current_address = addr;
    usb_hw->dev_addr_ctrl = addr;
}

static void _usb_handle_set_config(uint config_num) {
    _device.current_config_num = config_num;
    _usb_for_each_endpoint(_usb_hard_reset_endpoint_callback, false, 0);
    if (_device.on_configure) {
        _device.on_configure(&_device, config_num != 0);
    }
}

static void _usb_handle_bus_reset() {
#if PICO_USBDEV_ENABLE_DEBUG_TRACE
    usb_dump_trace();
    usb_reset_trace();
#endif

    // downgrade to unconfigured state
    _usb_handle_set_config(0);
    // downgrade to unaddressed state
    _usb_handle_set_address(0);

    // Clear buf status + sie status
    usb_hw_clear->buf_status = 0xffffffff;
    usb_hw_clear->sie_status = 0xffffffff;
//    // todo?
//    //usb_hw->abort = 0xffffffff;
}

#define should_handle_setup_request(e, s) (!(e)->setup_request_handler || !(e)->setup_request_handler(e, s))

struct usb_buffer *usb_current_packet_buffer(struct usb_endpoint *ep) {
    struct usb_buffer *packet = &ep->current_hw_buffer;
//    usb_debug("cpb %d %s\n", ep->num, usb_endpoint_dir_string(ep));
    if (!packet->valid) {
        packet->data_max = ep->buffer_size;
        uint which = ep->in ? ep->current_give_buffer : ep->current_take_buffer;
        if (ep->in) {
            assert(!(USB_BUF_CTRL_FULL & *_usb_buf_ctrl_narrow(ep, which)));
        } else {
            assert((USB_BUF_CTRL_FULL & *_usb_buf_ctrl_narrow(ep, which)));
        }
        packet->data = ((uint8_t *) (USBCTRL_DPRAM_BASE + ep->dpram_buffer_offset +
                                     (which ? _usb_endpoint_stride(ep) : 0)));
//        usb_debug("%d %s which %d len %08x\n", ep->num, usb_endpoint_dir_string(ep), which, (uint)*_usb_buf_ctrl_wide(ep));
        packet->data_len = ep->in ? 0 : (USB_BUF_CTRL_LEN_MASK & *_usb_buf_ctrl_narrow(ep, which));
        //usb_debug("getting buffer for endpoint %02x %s %p: buf_ctrl %d -> %04x\n", usb_endpoint_number(ep), usb_endpoint_dir_string(ep), packet->data, which, *_usb_buf_ctrl_narrow(ep, which));
        packet->valid = true;
    }
    return packet;
}

void _usb_give_buffer(struct usb_endpoint *ep, uint32_t len) {
    assert(ep->owned_buffer_count);
    assert(ep->current_transfer);
    assert(!ep->halt_state);
    ep->halt_state = HS_NONE; // best effort recovery

    assert(len < 1023);
    uint32_t val = len | USB_BUF_CTRL_AVAIL;

    if (ep->first_buffer_after_reset) {
        assert(!ep->current_give_buffer);
        val |= USB_BUF_CTRL_SEL;
        ep->first_buffer_after_reset = false;
    }

    assert(len <= ep->buffer_size);
    if (ep->in) val |= USB_BUF_CTRL_FULL;
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;
#if PICO_USBDEV_ENABLE_DEBUG_TRACE
    debug_trace[trace_i][0] = (uint32_t) _usb_buf_ctrl_narrow(ep, ep->current_give_buffer);
    debug_trace[trace_i][1] = val;
    trace_i++;
    if (trace_i == 128) {
        trace_i = 0;
    }
#endif

#if !PICO_USBDEV_BULK_ONLY_EP1_THRU_16
    if (ep->current_give_buffer) {
        val |= PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE
                << 11u; // 11 + 16 = 27 - which is where stride bits go (and only relevant on buffer 1)
    }
#endif

    *_usb_buf_ctrl_narrow(ep, ep->current_give_buffer) = val;
    if (ep->in) {
        // if there is a buffer len, then it must have been accessed to fill it with data
        assert(!len || ep->current_hw_buffer.valid);
    }

    ep->current_hw_buffer.valid = false;
    ep->owned_buffer_count--;
    ep->current_transfer->remaining_packets_to_submit--;
    if (ep->double_buffered) {
        ep->current_give_buffer ^= 1u;
//        usb_debug("toggle current give buffer %d %s to %d\n", ep->num, usb_endpoint_dir_string(ep), ep->current_give_buffer);
    }
}

static void _usb_call_on_packet(struct usb_endpoint *ep) {
    struct usb_transfer *current_transfer = ep->current_transfer;
    assert(current_transfer);
    assert(!current_transfer->outstanding_packet);
    current_transfer->outstanding_packet = true;
    current_transfer->type->on_packet(ep);
}

// If we own buffers, we try and transfer them to the hardware (either by filling packets via on_packet for
// IN or by passing empty buffers for out)
void _usb_give_as_many_buffers_as_possible(struct usb_endpoint *ep) {
    while (ep->current_transfer && ep->current_transfer->remaining_packets_to_submit && ep->owned_buffer_count &&
           !ep->halt_state) {
        if (ep->in) {
            uint old = ep->owned_buffer_count;
            _usb_call_on_packet(ep);
            if (old == ep->owned_buffer_count) {
                // on_packet did not yet submit anything
                break;
            }
        } else {
            if (ep->current_transfer->outstanding_packet) {
                usb_warn("untested? give buffer with outstanding packet %d %s owned %d\n", ep->num,
                         usb_endpoint_dir_string(ep), ep->owned_buffer_count);
            }
            _usb_give_buffer(ep, ep->buffer_size);
        }
    }
}

static void __noinline _usb_check_for_transfer_completion(struct usb_endpoint *ep) {
    struct usb_transfer *transfer = ep->current_transfer;
    assert(transfer);
    if (ep->halt_state || !(transfer->remaining_packets_to_handle || transfer->outstanding_packet)) {
        assert(!transfer->completed);
        transfer->completed = true;
        ep->current_transfer = NULL;
        if (ep->halt_state) {
            if (transfer->on_complete) {
                usb_warn("untested? stall of transfer with on_complete set %d %s %p\n", ep->num,
                         usb_endpoint_dir_string(ep), transfer->on_complete);
            }
            transfer->remaining_packets_to_submit = transfer->remaining_packets_to_handle = 0;
            return;
        }
        if (transfer->on_complete) {
            assert(!ep->chain_transfer);
            usb_debug("calling on complete\n");
            transfer->on_complete(ep, transfer);
        } else if (ep->chain_transfer) {
            usb_debug("chaining transfer\n");
            usb_start_transfer(ep, ep->chain_transfer);
        }
    } else if (!transfer->remaining_packets_to_handle) {
        usb_debug("outstanding packet %d on %d %s\n", transfer->outstanding_packet, ep->num,
                  usb_endpoint_dir_string(ep));
    }
}

static void _usb_handle_transfer(uint ep_num, bool in, uint which) {
    struct usb_endpoint *ep;
    assert(ep_num < PICO_USBDEV_MAX_ENDPOINTS);
    if (ep_num) {
        ep = _endpoints[ep_num];
    } else {
        ep = in ? &usb_control_in : &usb_control_out;
    }
    assert(ep); // "Received buffer IRQ for unknown EP");
    assert(!ep->halt_state);
    ep->owned_buffer_count++;
    struct usb_transfer *transfer = ep->current_transfer;
    if (!transfer) {
        usb_warn("received unexpected packet on %d %s\n", ep->num, usb_endpoint_dir_string(ep));
        return usb_halt_endpoint(ep);
    }
    assert(transfer->remaining_packets_to_handle);
    if (transfer->outstanding_packet) {
        usb_debug("re-enter %d %s which=%d\n", ep->num, usb_endpoint_dir_string(ep), which);
        assert(ep->double_buffered);
        assert(which != ep->current_take_buffer);
        transfer->packet_queued = true;
    } else {
        ep->current_take_buffer = which;
        // we only called on_packet for submit-able packets for an in transfer
        if (!ep->in || transfer->remaining_packets_to_submit) {
            _usb_call_on_packet(ep);
        }
        // transfer might already be completed during on_packet() if we stalled.
        if (!transfer->completed) {
            assert(transfer->remaining_packets_to_handle);
            --transfer->remaining_packets_to_handle;
            _usb_check_for_transfer_completion(ep);
        }
    }
}

void usb_packet_done(struct usb_endpoint *ep) {
    struct usb_buffer *buffer = &ep->current_hw_buffer;
    assert(buffer == &ep->current_hw_buffer);
    struct usb_transfer *transfer = ep->current_transfer;
    assert(transfer);
    assert(transfer->outstanding_packet);
    transfer->outstanding_packet = false;
    _usb_check_for_transfer_completion(ep);
    if (!transfer->completed) {
        //    usb_debug("buffer done for endpoint %02x %s %d/%d\n", usb_endpoint_number(ep), usb_endpoint_dir_string(ep),
        //              buffer->data_len, buffer->data_max);
        if (ep->in) {
            assert(buffer->valid);
            assert(buffer->data_len <= ep->buffer_size);
            _usb_give_buffer(ep, buffer->data_len);
        }
        ep->current_hw_buffer.valid = false;

        if (transfer->packet_queued) {
            assert(ep->double_buffered);
            usb_debug("Toggling current take buffer to %d and sending deferred packet %d %s\n",
                      ep->current_take_buffer ^ 1u, ep->num,
                      usb_endpoint_dir_string(ep));
            transfer->packet_queued = false;
            ep->owned_buffer_count--; // todo this is a bit of a hack because the function increments it a second time - maybe pass a param
            _usb_handle_transfer(ep->num, ep->in, ep->current_take_buffer ^ 1u);
        } else {
            // we may now need to top up double buffer;
            // note this call may cause recursion back into this function
            _usb_give_as_many_buffers_as_possible(ep);
        }
    }
}

void usb_set_default_transfer(struct usb_endpoint *ep, struct usb_transfer *transfer) {
    assert(!ep->default_transfer);
    ep->default_transfer = transfer;
}

void usb_start_transfer(struct usb_endpoint *ep, struct usb_transfer *transfer) {
    assert(!ep->current_transfer);
    ep->current_transfer = transfer;
    ep->chain_transfer = NULL;
    assert(transfer);
    assert(!transfer->started);
    transfer->started = true;
    assert(transfer->type->on_packet);
    // currently we explicitly disallow these rather than ending immediately.
    assert(transfer->remaining_packets_to_submit);
    assert(transfer->remaining_packets_to_handle);
#if !PICO_USBDEV_NO_TRANSFER_ON_INIT_METHOD
    if (transfer->type->on_init) {
        transfer->type->on_init(ep);
    }
#endif
    _usb_give_as_many_buffers_as_possible(ep);
}

void usb_chain_transfer(struct usb_endpoint *ep, struct usb_transfer *transfer) {
    assert(ep->current_transfer);
    assert(!ep->current_transfer->completed);
    assert(!ep->current_transfer->on_complete);
    ep->chain_transfer = transfer;
}

void __noinline usb_reset_transfer(struct usb_transfer *transfer, const struct usb_transfer_type *type,
                                   usb_transfer_completed_func on_complete) {
    memset(transfer, 0, sizeof(struct usb_transfer));
    transfer->type = type;
    transfer->on_complete = on_complete;
    transfer->remaining_packets_to_submit = transfer->remaining_packets_to_handle = type->initial_packet_count;
}

void usb_reset_and_start_transfer(struct usb_endpoint *ep, struct usb_transfer *transfer,
                                  const struct usb_transfer_type *type, usb_transfer_completed_func on_complete) {
    usb_reset_transfer(transfer, type, on_complete);
    usb_start_transfer(ep, transfer);
}

void usb_stall_control_pipe(__unused struct usb_setup_packet *setup) {
    // NOTE: doing this inside of usb_stall_endpoint which might seem reasonable allows a RACE with the host
    //  whereby it may send a new SETUP packet in response to one STALL before we have gotten to clearing
    //  the second buffer (yes I see this with the USB 2 Command Verifier!)
    _usb_reset_buffers(&usb_control_in);
    _usb_reset_buffers(&usb_control_out);

    _usb_stall_endpoint(&usb_control_in, HS_NON_HALT_STALL);
    _usb_stall_endpoint(&usb_control_out, HS_NON_HALT_STALL);
}

static void _tf_send_control_in_ack(__unused struct usb_endpoint *endpoint, __unused struct usb_transfer *transfer) {
    assert(endpoint == &usb_control_in);
    assert(transfer == &_control_in_transfer);
    usb_debug("_tf_setup_control_ack\n");
    usb_start_empty_transfer(&usb_control_out, &_control_out_transfer, 0);
}

static void _tf_send_control_out_ack(__unused struct usb_endpoint *endpoint, __unused struct usb_transfer *transfer) {
    assert(endpoint == &usb_control_out);
    assert(transfer == &_control_out_transfer);
    usb_debug("_tf_setup_control_ack\n");
    usb_start_empty_transfer(&usb_control_in, &_control_in_transfer, 0);
}

static void _tf_set_address(__unused struct usb_endpoint *endpoint, __unused struct usb_transfer *transfer) {
    assert(endpoint == &usb_control_in);
    usb_debug("_tf_set_address %d\n", _device.pending_address);
    _usb_handle_set_address(_device.pending_address);
}

static struct usb_configuration *_usb_get_current_configuration() {
    if (_device.current_config_num) return &_device.config;
    return NULL;
}

static struct usb_configuration *_usb_find_configuration(uint num) {
    if (_device.config.descriptor->bConfigurationValue == num) {
        return &_device.config;
    }
    return NULL;
}

static int _usb_prepare_string_descriptor(uint8_t *buf, __unused uint buf_len, const char *str) {
    int len = 2;
    uint8_t c;
    while (0 != (c = *str++)) {
        assert(len < buf_len);
        *(uint16_t *) (buf + len) = c;
        len += 2;
    }
    buf[0] = len;
    buf[1] = 3; // bDescriptorType
    return len;
}

static int _usb_handle_get_descriptor(uint8_t *buf, uint buf_len, struct usb_setup_packet *setup) {
    int len = -1;
    const uint8_t *src = NULL;
    buf = __builtin_assume_aligned(buf, 4);
    switch (setup->wValue >> 8u) {
        case USB_DT_DEVICE: {
            usb_trace("GET DEVICE DESCRIPTOR\n");
            len = sizeof(*_device.descriptor);
            src = (const uint8_t *) _device.descriptor;
            break;
        }
        case USB_DT_CONFIG: {
            usb_trace("GET CONFIG DESCRIPTOR %d\n", (uint8_t) setup->wValue);
            if (!(uint8_t) setup->wValue) {
                len = _device.config.descriptor->wTotalLength;
                src = (const uint8_t *) _device.config.descriptor;
            }
            break;
        }
        case USB_DT_STRING: {
            uint8_t index = setup->wValue;
            usb_trace("GET STRING DESCRIPTOR %d\n", index);
            if (index == 0) {
                // todo for now english only
                static const uint8_t lang_descriptor[] =
                        {
                                4, // bLength
                                0x03, // bDescriptorType == String Descriptor
                                0x09, 0x04 // language id = us english
                        };
                len = 4;
                src = lang_descriptor;
            } else {
                assert(_device.get_descriptor_string);
                const char *descriptor_string = _device.get_descriptor_string(index);
                assert(descriptor_string);
                len = _usb_prepare_string_descriptor(buf, buf_len, descriptor_string);
            }
            break;
        }
    }
    if (src && len > 0) {
        assert(len <= buf_len);
        memcpy(buf, src, len);
    }
    return len;
}

static void _usb_default_handle_device_setup_request(struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);
    if (!(setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        if (setup->bmRequestType & USB_DIR_IN) {
            struct usb_buffer *in_packet = usb_current_in_packet_buffer(&usb_control_in);
            uint8_t *buf = in_packet->data;
            uint buf_len = in_packet->data_max;
            int len = -1;

            switch (setup->bRequest) {
                case USB_REQUEST_GET_STATUS: {
                    usb_debug("DEVICE GET_STATUS\n");
                    *((uint16_t *) in_packet->data) = 0;
                    len = 2;
                    break;
                }
                case USB_REQUEST_GET_DESCRIPTOR: {
                    usb_debug("DEVICE GET_DESCRIPTOR\n");
#if PICO_USBDEV_MAX_DESCRIPTOR_SIZE > 64
                    static __aligned(4) uint8_t descriptor_buf[PICO_USBDEV_MAX_DESCRIPTOR_SIZE];
                    static struct usb_stream_transfer_funcs control_stream_funcs = {
                            .on_chunk = usb_stream_noop_on_chunk,
                            .on_packet_complete = usb_stream_noop_on_packet_complete
                    };
                    len = _usb_handle_get_descriptor(descriptor_buf, sizeof(descriptor_buf), setup);
                    if (len != -1)
                    {
                        len = MIN(len, setup->wLength);
                        usb_stream_setup_transfer(&_control_in_stream_transfer, &control_stream_funcs, descriptor_buf,
                                                  sizeof(descriptor_buf), len, _tf_send_control_in_ack);

                        _control_in_stream_transfer.ep = &usb_control_in;
                        return usb_start_transfer(&usb_control_in, &_control_in_stream_transfer.core);
                    } else {
                        //usb_warn("Didn't find requested device descriptor\n");
                    }
#else
                    len = _usb_handle_get_descriptor(buf, buf_len, setup);
#endif
                    break;
                }
                case USB_REQUEST_GET_CONFIGURATION: {
                    usb_debug("DEVICE GET_CONFIGURATION\n");
                    *((uint8_t *) buf) = _device.current_config_num;
                    len = 1;
                    break;
                }
            }
            if (len >= 0) {
                assert(len < buf_len); // a bit late
                in_packet->data_len = MIN(len, setup->wLength);
                return usb_start_single_buffer_control_in_transfer();
            }
            usb_warn("Unhandled device IN setup request %02x\n", setup->bRequest);
        } else {
            switch (setup->bRequest) {
                case USB_REQUEST_SET_FEATURE: {
                    assert(false);
                    break;
                }
                case USB_REQUEST_SET_ADDRESS: {
                    uint8_t addr = setup->wValue;
                    if (addr && addr <= 127) {
                        usb_debug("SET ADDRESS %02x\n", addr);
                        _device.pending_address = addr;
                        return usb_start_empty_control_in_transfer(_tf_set_address);
                    }
                    break;
                }
                case USB_REQUEST_SET_DESCRIPTOR: {
                    assert(false);
                    break;
                }
                case USB_REQUEST_SET_CONFIGURATION: {
                    uint8_t config_num = setup->wValue;
                    usb_debug("SET CONFIGURATION %02x\n", config_num);
                    if (!config_num || _usb_find_configuration(config_num)) {
                        // graham 1/3/20 removed this:
                        // USB 2.0 9.4.7: "If the specified configuration value matches the configuration value from a
                        // configuration descriptor, then that configuration is selected and the device remains in
                        // the Configured state"
                        // USB 2.0 9.4.5: "The Halt feature is reset to zero after either a SetConfiguration() or SetInterface() request even if the
                        // requested configuration or interface is the same as the current configuration or interface."
                        //
                        // Since there isn't a particularly clean way to unset a STALL, i'm taking this to mean that we should just do regular config setting tuff
                        //                    if (config_num != device.current_config_num)
                        //                    {
                        _usb_handle_set_config(config_num);
                        //                    }
                        return usb_start_empty_control_in_transfer_null_completion();
                    }
                    break;
                }
            }
            usb_warn("Unhandled device OUT setup request %02x\n", setup->bRequest);
        }
    }
    // default
    return usb_stall_control_pipe(setup);
}

static void _usb_default_handle_interface_setup_request(struct usb_setup_packet *setup,
                                                        __unused struct usb_interface *interface) {
    // check for valid class request
    if (!(setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) && !(setup->wIndex >> 8u)) {
        if (setup->bmRequestType & USB_DIR_IN) {
            switch (setup->bRequest) {
                case USB_REQUEST_GET_STATUS: {
                    usb_debug("DEVICE GET_STATUS\n");
                    return usb_start_tiny_control_in_transfer(0, 2);
                }
#if !PICO_USBDEV_NO_INTERFACE_ALTERNATES
                case USB_REQUEST_GET_INTERFACE: {
                    if (!setup->wValue && setup->wLength == 1) {
                        return usb_start_tiny_control_in_transfer(interface->alt, 1);
                    }
                }
#endif
            }
        } else {
            switch (setup->bRequest) {
                case USB_REQUEST_SET_INTERFACE: {
#if !PICO_USBDEV_NO_INTERFACE_ALTERNATES
                    if (interface->set_alternate_handler) {
                        if (interface->set_alternate_handler(interface, setup->wValue)) {
                            interface->alt = setup->wValue;
                            return usb_start_empty_control_in_transfer_null_completion();
                        }
                    }
#endif
                    // todo should we at least clear all HALT? - i guess given that we don't support this is fine
                    usb_warn("(ignored) set interface %d (alt %d)\n", setup->wIndex, setup->wValue);
                    break;
                }
            }
        }
    }
    usb_warn("Unhandled interface %02x setup request %02x bmRequestType %02x\n",
             interface->descriptor->bInterfaceNumber, setup->bRequest, setup->bmRequestType);
    // default
    return usb_stall_control_pipe(setup);
}

static void _usb_default_handle_endpoint_setup_request(struct usb_setup_packet *setup, struct usb_endpoint *ep) {
    if (!(setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        if (setup->bmRequestType & USB_DIR_IN) {
            switch (setup->bRequest) {
                case USB_REQUEST_GET_STATUS: {
                    if (!setup->wValue && setup->wLength == 2) {
                        // HALT FEATURE is not set for control stall
                        return usb_start_tiny_control_in_transfer(ep->halt_state > HS_NON_HALT_STALL ? 1 : 0, 2);
                    }
                    break;
                }
            }
            usb_warn("Unhandled ep %02x %s IN setup request %02x\n", ep->num, usb_endpoint_dir_string(ep),
                     setup->bRequest);
        } else {
            switch (setup->bRequest) {
                case USB_REQUEST_CLEAR_FEATURE: {
                    if (setup->wValue == USB_FEAT_ENDPOINT_HALT) {
                        if (ep->halt_state < HS_HALTED_ON_CONDITION) {
                            usb_debug("Request unhalt EP %d %s\n", ep->num, usb_endpoint_dir_string(ep));
                            usb_hard_reset_endpoint(ep);
                        } else {
                            ep->next_pid = 0; // must always reset data toggle
                            usb_debug("Skipped unhalt EP %d %s halt_state = %d\n", ep->num, usb_endpoint_dir_string(ep),
                                      ep->halt_state);
                        }
                        return usb_start_empty_control_in_transfer_null_completion();
                    }
                    break;
                }
                case USB_REQUEST_SET_FEATURE: {
                    if (setup->wValue == USB_FEAT_ENDPOINT_HALT) {
                        usb_debug("Request halt EP %d %s\n", ep->num, usb_endpoint_dir_string(ep));
                        _usb_stall_endpoint(ep, HS_HALTED);
                        return usb_start_empty_control_in_transfer_null_completion();
                    }
                    break;
                }
            }
            usb_warn("Unhandled ep %02x %s OUT setup request %02x\n", ep->num, usb_endpoint_dir_string(ep),
                     setup->bRequest);
        }
    } else {
        usb_warn("Unhandled endpoint %d %s setup request %02x bmRequestType %02x\n", ep->num,
                 usb_endpoint_dir_string(ep), setup->bRequest, setup->bmRequestType);
    }
    // default
    return usb_stall_control_pipe(setup);
}

// returns null if device not configured
static struct usb_interface *_usb_find_interface(uint num) {
    struct usb_configuration *config = _usb_get_current_configuration();
    if (config) {
#if PICO_USBDEV_USE_ZERO_BASED_INTERFACES
        if (num < _usb_interface_count(config)) {
            return config->interfaces[num];
        }
#else
        for (uint i = 0; i < _usb_interface_count(config); i++) {
            if (config->interfaces[i]->descriptor->bInterfaceNumber == num) {
                return config->interfaces[i];
            }
        }
#endif
    }
    return NULL;
}

// returns null if device not configured
static struct usb_endpoint *_usb_find_endpoint(uint num) {
    if (!num) {
        return &usb_control_out;
    } else if (num == USB_DIR_IN) {
        return &usb_control_in;
    }
    if (_usb_get_current_configuration()) {
        for (uint i = 1; i < count_of(_endpoints); i++) {
            if (_endpoints[i]->descriptor->bEndpointAddress == num) {
                return _endpoints[i];
            }
        }
    }
    return NULL;
}

static void _usb_handle_setup_packet(struct usb_setup_packet *setup) {
    usb_debug("Setup packet\n");
    // a setup packet is always accepted, so reset anything in progress
    usb_soft_reset_endpoint(&usb_control_in);
    usb_soft_reset_endpoint(&usb_control_out);
    usb_control_in.next_pid = usb_control_out.next_pid = 1;
    switch (setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) {
        case USB_REQ_TYPE_RECIPIENT_DEVICE: {
#if !PICO_USBDEV_NO_DEVICE_SETUP_HANDLER
            if (!should_handle_setup_request(&_device, setup)) return;
#endif
            return _usb_default_handle_device_setup_request(setup);
        }
        case USB_REQ_TYPE_RECIPIENT_INTERFACE: {
            struct usb_interface *interface = _usb_find_interface(
                    setup->wIndex & 0xffu); // todo interface is only one byte; high byte seems to be used for entity
            usb_debug("Interface request %d %p\n", setup->wIndex, interface);
            if (interface) {
                if (!should_handle_setup_request(interface, setup)) return;
                return _usb_default_handle_interface_setup_request(setup, interface);
            }
            usb_warn("Setup request %04x for unknown interface %04x\n", setup->bRequest, setup->wIndex);
            break;
        }
        case USB_REQ_TYPE_RECIPIENT_ENDPOINT: {
            struct usb_endpoint *endpoint = _usb_find_endpoint(setup->wIndex);
            if (endpoint) {
#if !PICO_USBDEV_NO_ENDPOINT_SETUP_HANDLER
                if (!should_handle_setup_request(endpoint, setup)) return;
#endif
                return _usb_default_handle_endpoint_setup_request(setup, endpoint);
            }
            usb_warn("Setup packet %04x for unknown endpoint %04x\n", setup->wValue, setup->wIndex);
            break;
        }
    }
    usb_warn("Unhandled setup packet - stalling control pipe\n");
    // default
    usb_stall_control_pipe(setup);
}

static void _usb_handle_buffer() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    if (!buffers) {
        usb_debug("_usb_handle_buffer called without any buffers set\n");
    }

    // do this for now could be smarter
    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < PICO_USBDEV_MAX_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            uint which = (usb_hw->buf_cpu_should_handle & bit) ? 1 : 0;
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            _usb_handle_transfer(i >> 1u, !(i & 1u), which);
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
    if (remaining_buffers) {
        usb_debug("Ignoring buffer event for impossible mask %08x\n", (uint) remaining_buffers);
        usb_hw_clear->buf_status = remaining_buffers;
    }
}

void __isr __used isr_usbctrl(void) {
    uint32_t status = usb_hw->ints;
    DEBUG_PINS_SET(usb_irq, 1);

    uint32_t handled = 0;
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        _usb_handle_setup_packet(remove_volatile_cast(struct usb_setup_packet *, &usb_dpram->setup_packet));
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
    }

    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        _usb_handle_buffer();
        // Interrupt is cleared when buff flag is cleared
    }

    if (status & USB_INTS_BUS_RESET_BITS) {
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_debug("Bus Reset\n");
        _usb_handle_bus_reset();
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        rp2040_usb_device_enumeration_fix();
    }

    if (status & USB_INTS_DEV_SOF_BITS) {
        handled |= USB_INTS_DEV_SOF_BITS;
        volatile uint unused = usb_hw->sof_rd; // clear SOF interrupt
        extern void __not_in_flash_func(usb_sof_irq)(void);
        usb_sof_irq();
    }

    if (status & USB_INTS_ERROR_BITS) {
        handled |= (status & USB_INTS_ERROR_BITS);
#ifndef NDEBUG
        _usb_dump_eps();
#endif
        //uint32_t errs = usb_hw->sie_status;
        usb_warn("Error 0x%lx (sie status 0x%lx)\n", (status & USB_INTS_ERROR_BITS), usb_hw->sie_status);
        if (usb_hw->sie_status & USB_SIE_STATUS_DATA_SEQ_ERROR_BITS) {
            usb_dump_trace();
            usb_warn("Data seq error\n");
            usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
        } else {
            // Assume we have been unplugged
            usb_debug("Assuming unplugged\n");
            _usb_handle_bus_reset();
        }
    }

    if (status ^ handled) {
        usb_warn("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }

    DEBUG_PINS_CLR(usb_irq, 1);
}

#if PICO_USBDEV_ENABLE_DEBUG_TRACE
void usb_dump_trace(void)
{
    usb_debug("\n");
    for (int i = 0; i < trace_i; i++) {
        uint16_t ctrl = (uint16_t)debug_trace[i][1];
        uint8_t pid = (ctrl & USB_BUF_CTRL_DATA1_PID) ? 1 : 0;
        int ep = -1, b = -1, d = -1;
        for(int e=0;e<USB_NUM_ENDPOINTS;e++) {
            if (debug_trace[i][0] == (uintptr_t)&usb_dpram->ep_buf_ctrl[e].in)
            {
                ep = e;
                b = 0;
                d = 0;
            } else if (debug_trace[i][0] == 2 + (uintptr_t)&usb_dpram->ep_buf_ctrl[e].in) {
                ep = e;
                b = 1;
                d = 0;
            } else if (debug_trace[i][0] == (uintptr_t)&usb_dpram->ep_buf_ctrl[e].out) {
                ep = e;
                b = 0;
                d = 1;
            } else if (debug_trace[i][0] == 2 + (uintptr_t)&usb_dpram->ep_buf_ctrl[e].out) {
                ep = e;
                b = 1;
                d = 1;
            }
        }
        usb_debug("0x%lx (ep %d, b %d, d %d) <= 0x%x (DATA%d", debug_trace[i][0], ep, b, d, ctrl, pid);
        if (debug_trace[i][0] & 0b100) {
            usb_debug(", OUT");
        } else {
            usb_debug(", IN ");
        }
        if (ctrl & USB_BUF_CTRL_FULL) { usb_debug(", FULL"); }
        if (ctrl & USB_BUF_CTRL_LAST) { usb_debug(", LAST"); }
        if (ctrl & USB_BUF_CTRL_SEL) { usb_debug(", SEL"); }
        usb_debug(", LEN = %d)\n", ctrl & USB_BUF_CTRL_LEN_MASK);
    }
    usb_reset_trace();
}

void usb_reset_trace(void)
{
    trace_i = 0;
}
#endif

const char *usb_endpoint_dir_string(struct usb_endpoint *ep) {
    return _in_out_string(ep->in);
}

static const void *usb_next_descriptor(const void *d, uint8_t type) {
    const struct usb_descriptor *desc = (const struct usb_descriptor *) d;
    do {
        desc = (const struct usb_descriptor *) (((const uint8_t *) desc) + desc->bLength);
    } while (desc->bDescriptorType != type);
    return desc;
}

/**
 * Initialize the runtime data structures for an interface, and all its endpoints
 * @param interface
 * @param desc
 * @param endpoints
 * @param endpoint_count
 * @param double_buffered
 * @return
 */
struct usb_interface *usb_interface_init(struct usb_interface *interface, const struct usb_interface_descriptor *desc,
                                         struct usb_endpoint *const *endpoints, uint endpoint_count,
                                         bool double_buffered) {
    assert(desc->bLength == sizeof(struct usb_interface_descriptor));
    assert(desc->bNumEndpoints == endpoint_count);
    interface = usb_common_init(interface);
    interface->descriptor = desc;
    interface->endpoints = endpoints;
    interface->endpoint_count = endpoint_count;
    const void *p = (const void *) desc;
    for (uint i = 0; i < endpoint_count; i++) {
        p = usb_next_descriptor(p, USB_DESCRIPTOR_TYPE_ENDPOINT);
        const struct usb_endpoint_descriptor *ep_desc = (const struct usb_endpoint_descriptor *) p;
        assert(ep_desc->bLength >= sizeof(struct usb_endpoint_descriptor));
        assert(ep_desc->bDescriptorType == USB_DESCRIPTOR_TYPE_ENDPOINT);
        uint8_t ep_num = ep_desc->bEndpointAddress & 0xfu;
        assert(ep_num && ep_num < PICO_USBDEV_MAX_ENDPOINTS);
        _usb_endpoint_init_internal(endpoints[i], ep_num, ep_desc->bEndpointAddress & USB_DIR_IN,
                                    ep_desc->wMaxPacketSize,
                                    double_buffered);
        endpoints[i]->descriptor = ep_desc;
#if !PICO_USBDEV_BULK_ONLY_EP1_THRU_16
        if (USB_TRANSFER_TYPE_ISOCHRONOUS == (ep_desc->bmAttributes & USB_TRANSFER_TYPE_BITS)) {
            endpoints[i]->buffer_stride = 128 << PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE;
        } else {
            endpoints[i]->buffer_stride = 64;
        }
        assert(ep_desc->wMaxPacketSize <= endpoints[i]->buffer_stride);
#endif
    }
    return interface;
}

struct usb_device *usb_device_init(const struct usb_device_descriptor *desc,
                                   const struct usb_configuration_descriptor *config_desc,
                                   struct usb_interface *const *interfaces, uint interface_count,
                                   const char *(*get_descriptor_string)(uint index)) {
    usb_debug("-------------------------------------------------------------------------------\n");
    assert(desc->bLength == sizeof(struct usb_device_descriptor));
    assert(desc->bNumConfigurations ==
           1); // all that is supported right now (otherwise we must handle GET/SET_CONFIGURATION better
    assert(config_desc->bNumInterfaces == interface_count);
    _device.descriptor = desc;
    _device.config.descriptor = config_desc;
    _device.config.interfaces = interfaces;
#ifndef PICO_USBDEV_FIXED_INTERFACE_COUNT
    _device.config.interface_count = interface_count;
#endif
    _device.get_descriptor_string = get_descriptor_string;

    _usb_endpoint_init_internal(&usb_control_in, 0, true, 64, false);
    _usb_endpoint_init_internal(&usb_control_out, 0, false, 64, false);
    usb_init_clear_deref(&_endpoints);
    for (uint i = 0; i < interface_count; i++) {
        for (uint e = 0; e < interfaces[i]->endpoint_count; e++) {
            struct usb_endpoint *ep = interfaces[i]->endpoints[e];
            uint ep_num = usb_endpoint_number(ep);
            assert(ep_num && ep_num < count_of(_endpoints));
            _endpoints[ep_num] = ep;
        }
    }
#if PICO_USBDEV_USE_ZERO_BASED_INTERFACES
    for (uint i = 0; i < interface_count; i++) {
        assert(interfaces[i]->descriptor->bInterfaceNumber == i);
    }
#endif

    _device.next_buffer_offset = 0x100;
    _usb_endpoint_hw_init(&usb_control_in, 0);
    _device.next_buffer_offset = 0x100;
    _usb_endpoint_hw_init(&usb_control_out, 0);
    _device.next_buffer_offset = 0x180;
    _usb_for_each_endpoint(_usb_endpoint_hw_init, false, 0);
    return &_device;
}

void usb_grow_transfer(struct usb_transfer *transfer, uint packet_count) {
    transfer->remaining_packets_to_submit += packet_count;
    transfer->remaining_packets_to_handle += packet_count;
}

void usb_soft_reset_endpoint(struct usb_endpoint *ep) {
    _usb_reset_endpoint(ep, false);
}

void usb_hard_reset_endpoint(struct usb_endpoint *ep) {
    _usb_reset_endpoint(ep, true);
}

void usb_halt_endpoint(struct usb_endpoint *ep) {
    _usb_stall_endpoint(ep, HS_HALTED);
};

void usb_halt_endpoint_on_condition(struct usb_endpoint *ep) {
    _usb_stall_endpoint(ep, HS_HALTED_ON_CONDITION);
};

void usb_clear_halt_condition(struct usb_endpoint *ep) {
    if (ep->halt_state == HS_HALTED_ON_CONDITION) {
        ep->halt_state = HS_HALTED; // can be reset by regular unstall
    }
}

void usb_device_start() {
    // At least on FPGA we don't know the previous state
    // so clean up registers. Should be fine not clearing DPSRAM
    io_rw_32 *reg = &usb_hw->dev_addr_ctrl;
    // Don't touch phy trim
    while (reg != &usb_hw->phy_trim)
        *reg++ = 0;

    // Start setup
#if PICO_USBDEV_ENABLE_DEBUG_TRACE
    trace_i = 0;
#endif

    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    // Reset various things to default state
    _usb_handle_bus_reset();

    // Pull up starts the show. Enable IRQ for EP0 buffer done
    usb_hw->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS | USB_SIE_CTRL_EP0_INT_1BUF_BITS;
    // Present pull up before enabling bus reset irq
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS |
                   USB_INTS_ERROR_BITS | USB_INTS_DEV_SOF_BITS;// | USB_INTS_EP_STALL_NAK_BITS;

    irq_set_enabled(USBCTRL_IRQ, true);
}

void usb_device_stop(__unused struct usb_device *device) {
    assert(false);
}

void usb_start_tiny_control_in_transfer(uint32_t data, uint len) {
    assert(len <= 4);
    struct usb_buffer *buffer = usb_current_in_packet_buffer(&usb_control_in);
    // little endian so this works for any len
    *(uint32_t *) buffer->data = data;
    buffer->data_len = len;
    return usb_start_single_buffer_control_in_transfer();
}

void usb_start_single_buffer_control_in_transfer() {
    assert(usb_current_in_packet_buffer(&usb_control_in)->data_len <
           64); // we don't want to have to send an extra packet
    usb_reset_and_start_transfer(&usb_control_in, &_control_in_transfer, &usb_current_packet_only_transfer_type,
                                 _tf_send_control_in_ack);
}

void usb_start_control_out_transfer(const struct usb_transfer_type *type) {
    usb_reset_and_start_transfer(&usb_control_out, &_control_out_transfer, type, _tf_send_control_out_ack);
}

void usb_start_empty_transfer(struct usb_endpoint *endpoint, struct usb_transfer *transfer,
                              usb_transfer_completed_func on_complete) {
    if (endpoint->in) usb_current_in_packet_buffer(endpoint)->data_len = 0;
    usb_reset_and_start_transfer(endpoint, transfer, &usb_current_packet_only_transfer_type, on_complete);
}

void usb_start_empty_control_in_transfer(usb_transfer_completed_func on_complete) {
    usb_start_empty_transfer(&usb_control_in, &_control_in_transfer, on_complete);
}

void usb_start_empty_control_in_transfer_null_completion() {
    usb_start_empty_control_in_transfer(0);
}

// this is provided as a wrapper to catch coding errors
struct usb_buffer *usb_current_in_packet_buffer(struct usb_endpoint *ep) {
    assert(ep->in);
    return usb_current_packet_buffer(ep);
}

// this is provided as a wrapper to catch coding errors
struct usb_buffer *usb_current_out_packet_buffer(struct usb_endpoint *ep) {
    assert(!ep->in);
    return usb_current_packet_buffer(ep);
}

void usb_start_default_transfer_if_not_already_running_or_halted(struct usb_endpoint *ep) {
    // if we are in halt state we will do this again later; defensively check against current transfer already in place
    if (!ep->halt_state && ep->current_transfer != ep->default_transfer) {
        usb_reset_and_start_transfer(ep, ep->default_transfer, ep->default_transfer->type, 0);
    }
}
