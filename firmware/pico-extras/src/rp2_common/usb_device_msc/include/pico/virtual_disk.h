/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _VIRTUAL_DISK_H
#define _VIRTUAL_DISK_H

#include "usb_device_msc.h"

#define USE_INFO_UF2

void vd_init();
void vd_reset();

// return true for async operation
bool vd_read_block(uint32_t token, uint32_t lba, uint8_t *buf, uint32_t buf_size);
bool vd_write_block(uint32_t token, uint32_t lba, uint8_t *buf, uint32_t buf_size);

// give us ourselves 16M which should strictly be the minimum for FAT16 - Note Win10 doesn't like FAT12 - go figure!
// upped to 64M which allows us to download a 32M UF2
#define CLUSTER_UP_SHIFT 0u
#define CLUSTER_UP_MUL (1u << CLUSTER_UP_SHIFT)
#define VOLUME_SIZE (CLUSTER_UP_MUL * 128u * 1024u * 1024u)

#define SECTOR_COUNT (VOLUME_SIZE / SECTOR_SIZE)

#ifndef GENERAL_SIZE_HACKS

static inline uint32_t vd_sector_count() {
    return SECTOR_COUNT;
}

#else
// needs to be a compile time constant
#define vd_sector_count() SECTOR_COUNT
#endif

void vd_async_complete(uint32_t token, uint32_t result);
#endif
