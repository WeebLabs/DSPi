/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_SD_CARD_H
#define _PICO_SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico.h"

#define SD_OK (0)
#define SD_ERR_STUCK (-1)
#define SD_ERR_BAD_RESPONSE (-2)
#define SD_ERR_CRC (-3)
#define SD_ERR_BAD_PARAM (-4)

#ifndef PICO_SD_CLK_PIN
#define PICO_SD_CLK_PIN 23
#endif

#ifndef PICO_SD_CMD_PIN
#define PICO_SD_CMD_PIN 24
#endif

#ifndef PICO_SD_DAT0_PIN
#define PICO_SD_DAT0_PIN 19
#endif

// todo for now
#define PICO_SD_MAX_BLOCK_COUNT 32
// todo buffer pool
int sd_init_4pins();
int sd_init_1pin();
#define SD_SECTOR_SIZE 512
int sd_readblocks_sync(uint32_t *buf, uint32_t block, uint block_count);
int sd_readblocks_async(uint32_t *buf, uint32_t block, uint block_count);
int sd_readblocks_scatter_async(uint32_t *control_words, uint32_t block, uint block_count);
void sd_set_byteswap_on_read(bool swap);
bool sd_scatter_read_complete(int *status);
int sd_writeblocks_async(const uint32_t *data, uint32_t sector_num, uint sector_count);
bool sd_write_complete(int *status);
int sd_read_sectors_1bit_crc_async(uint32_t *sector_buf, uint32_t sector, uint sector_count);
int sd_set_wide_bus(bool wide);
int sd_set_clock_divider(uint div);

#endif

#ifdef __cplusplus
}
#endif