/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/util/buffer.h"

#ifdef PICO_BUFFER_USB_ALLOC_HACK
#include <string.h>

uint8_t *usb_ram_alloc_ptr = (uint8_t *)(USBCTRL_DPRAM_BASE + USB_DPRAM_MAX);

static void __attribute__((constructor)) _clear_usb_ram() {
    memset(usb_ram_alloc_ptr, 0, USB_DPRAM_SIZE - USB_DPRAM_MAX);
}
#endif
