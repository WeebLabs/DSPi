/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include "pico/usb_stream_helper.h"

static uint32_t _usb_stream_chunk_offset(struct usb_stream_transfer *transfer) {
    return transfer->offset & (transfer->chunk_size - 1);
}

void usb_stream_packet_handler_complete(struct usb_stream_transfer *transfer) {
    struct usb_buffer *buffer;
    struct usb_endpoint *ep = transfer->ep;
#ifndef NDEBUG
    assert(transfer->packet_handler_complete_expected);
    transfer->packet_handler_complete_expected = false;
#endif
    assert(ep);
    if (ep->in) {
        buffer = usb_current_in_packet_buffer(ep);
        assert(buffer);
        assert(buffer->data_max == 64);
        uint chunk_offset = _usb_stream_chunk_offset(transfer);
        uint data_len = 64;
        if (transfer->offset + 64 > transfer->transfer_length) {
            data_len = transfer->transfer_length - transfer->offset;
        }
        buffer->data_len = data_len;
        memcpy(buffer->data, transfer->chunk_buffer + chunk_offset, data_len);
    } else {
        buffer = usb_current_out_packet_buffer(ep);
        assert(buffer);
        assert(buffer->data_len);
    }
    transfer->offset += buffer->data_len;
    if (ep->num > 2) usb_debug("  %d transfer_offset %d\n", ep->num, (uint) transfer->offset);
    assert(transfer->funcs && transfer->funcs->on_packet_complete);
    transfer->funcs->on_packet_complete(transfer);
#ifdef USE_BOOTROM_GPIO
    gpio_clr_mask(usb_activity_gpio_pin_mask);
#endif
    usb_packet_done(ep);
}

void usb_stream_chunk_done(struct usb_stream_transfer *transfer) {
    usb_stream_packet_handler_complete(transfer);
}

void _usb_stream_packet_packet_handler(struct usb_endpoint *ep) {
#ifdef USE_BOOTROM_GPIO
    gpio_set_mask(usb_activity_gpio_pin_mask);
#endif
    // todo assert type
    struct usb_stream_transfer *transfer = (struct usb_stream_transfer *) ep->current_transfer;
    uint chunk_offset = _usb_stream_chunk_offset(transfer);
    uint chunk_len = 0; // set to non zero to call on_chunk
    if (ep->in) {
        if (!chunk_offset) {
            // we are at the beginning of a chunk so want to call on_chunk
            chunk_len = (transfer->offset + transfer->chunk_size) > transfer->transfer_length ?
                        transfer->transfer_length - transfer->offset : transfer->chunk_size;
            if (ep->num > 2)
                usb_warn("chunko %d len %05x offset %08x size %04x transfer %08x\n", ep->num, chunk_len, chunk_offset,
                         (uint) transfer->chunk_size, (uint) transfer->transfer_length);
        }
    } else {
        //    usb_debug("write packet %04x %d\n", (uint)transfer->offset, ep->current_take_buffer);
        struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
        assert(buffer);
        // note we only set chunk_len if this is the end of a chunk
        if (transfer->offset + 64 >= transfer->transfer_length) {
            // we have ended the transfer (possibly mid-chunk)
            chunk_len = transfer->transfer_length & (transfer->chunk_size - 1);
            if (chunk_len) {
                usb_warn(">> Truncated %08x\n", chunk_len);
            } else {
                chunk_len = transfer->chunk_size;
            }
        } else if (chunk_offset + 64 >= transfer->chunk_size) {
            // end of regular chunk
            chunk_len = transfer->chunk_size;
        }
        assert(chunk_len || buffer->data_len == 64);
//        if (!(!chunk_len || buffer->data_len == ((chunk_len & 63u) ? (chunk_len & 63u) : 64u))) {
//            usb_warn("ooh off=%08x len=%08x chunk_off=%04x chunk_len=%04x data_len=%04x\n", (uint)transfer->offset, (uint)transfer->transfer_length, chunk_offset, chunk_len, buffer->data_len);
//        }
        assert(!chunk_len || buffer->data_len == ((chunk_len & 63u) ? (chunk_len & 63u) : 64u));
        // zero buffer when we start a new buffer, so that the chunk callback never sees data it shouldn't (for partial chunks)
        if (!chunk_offset) {
            memset(transfer->chunk_buffer, 0, transfer->chunk_size);
        }
        memcpy(transfer->chunk_buffer + chunk_offset, buffer->data, buffer->data_len); // always safe to copy all
    }
#ifndef NDEBUG
    transfer->packet_handler_complete_expected = true;
#endif

    // todo i think this is reasonable since 0 length chunk does nothing
    if (chunk_len) {
        assert(transfer->funcs && transfer->funcs->on_chunk);
        if (transfer->funcs->on_chunk(chunk_len, transfer))
            return;
    }
    usb_stream_packet_handler_complete(transfer);
}

static const struct usb_transfer_type _usb_stream_transfer_type = {
        .on_packet = _usb_stream_packet_packet_handler
};

void usb_stream_setup_transfer(struct usb_stream_transfer *transfer, const struct usb_stream_transfer_funcs *funcs,
                               uint8_t *chunk_buffer, uint32_t chunk_size, uint32_t transfer_length,
                               usb_transfer_completed_func on_complete) {
    transfer->funcs = funcs;
    transfer->chunk_buffer = chunk_buffer;
    assert(!(chunk_size & 63u)); // buffer should be a multiple of USB packet buffer size
    transfer->chunk_size = chunk_size;
    transfer->offset = 0;
    // todo combine with residue?
    transfer->transfer_length = transfer_length;
    usb_reset_transfer(&transfer->core, &_usb_stream_transfer_type, on_complete);
    usb_grow_transfer(&transfer->core, (transfer_length + 63) / 64);
}

void usb_stream_noop_on_packet_complete(__unused struct usb_stream_transfer *transfer) {

}

bool usb_stream_noop_on_chunk(uint32_t size, __unused struct usb_stream_transfer *transfer) {
    return false;
}



