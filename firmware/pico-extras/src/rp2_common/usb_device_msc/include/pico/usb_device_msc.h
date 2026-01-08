/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USB_MSC_H
#define _USB_MSC_H

#define SECTOR_SIZE 512u

bool msc_setup_request_handler(struct usb_interface *interface, struct usb_setup_packet *setup);
void msc_on_configure(__unused struct usb_device *device, bool configured);
//struct usb_endpoint msc_in, msc_out;
extern struct usb_endpoint msc_endpoints[2];

// provided by the hosting code
uint32_t msc_get_serial_number32();
void msc_eject();

#endif