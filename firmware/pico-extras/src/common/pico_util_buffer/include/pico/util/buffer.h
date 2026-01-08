/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_UTIL_BUFFER_H
#define _PICO_UTIL_BUFFER_H

#include "pico/types.h"

/** \file buffer.h
 * \defgroup util_buffer buffer
 * \brief Buffer management
 * \ingroup pico_util
 */

#ifdef PICO_BUFFER_USB_ALLOC_HACK
#include "hardware/address_mapped.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DEBUG_MALLOC
#include <stdio.h>
#endif

#include <stdlib.h>

/** \struct mem_buffer
 *  \ingroup util_buffer
 *  \brief Wrapper structure around a memory buffer
 * 
 *   Wrapper could be around static or allocated memory
 * 
 * \todo This module needs to be checked - think there are issues with the free function
 */
typedef struct mem_buffer {
    size_t size;
    uint8_t *bytes;
    uint8_t flags;
} mem_buffer_t;

#ifdef PICO_BUFFER_USB_ALLOC_HACK
#if !defined(USB_DPRAM_MAX) || (USB_DPRAM_MAX > 0)
#include "hardware/structs/usb.h"
#else
#define USB_DPRAM_SIZE 4096
#endif
#endif

inline static bool pico_buffer_alloc_in_place(mem_buffer_t *buffer, size_t size) {
#ifdef PICO_BUFFER_USB_ALLOC_HACK
    extern uint8_t *usb_ram_alloc_ptr;
    if ((usb_ram_alloc_ptr + size) <= (uint8_t *)USBCTRL_DPRAM_BASE + USB_DPRAM_SIZE) {
        buffer->bytes = usb_ram_alloc_ptr;
        buffer->size = size;
#ifdef DEBUG_MALLOC
        printf("balloc %d %p->%p\n", size, buffer->bytes, ((uint8_t *)buffer->bytes) + size);
#endif
        usb_ram_alloc_ptr += size;
        return true;
    }
#endif    // inline for now
    buffer->bytes = (uint8_t *) calloc(1, size);
    if (buffer->bytes) {
        buffer->size = size;
        return true;
    }
    buffer->size = 0;
    return false;
}

inline static mem_buffer_t *pico_buffer_wrap(uint8_t *bytes, size_t size) {
    mem_buffer_t *buffer = (mem_buffer_t *) malloc(sizeof(mem_buffer_t));
    if (buffer) {
        buffer->bytes = bytes;
        buffer->size = size;
    }
    return buffer;
}

inline static mem_buffer_t *pico_buffer_alloc(size_t size) {
    mem_buffer_t *b = (mem_buffer_t *) malloc(sizeof(mem_buffer_t));
    if (b) {
        if (!pico_buffer_alloc_in_place(b, size)) {
            free(b);
            b = NULL;
        }
    }
    return b;
}

#ifdef __cplusplus
}
#endif
#endif
