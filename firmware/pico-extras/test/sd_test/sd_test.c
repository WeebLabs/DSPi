/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/sd_card.h"

static uint8_t sector_data[1024];

#define ENABLE_4_PIN 0

int main(void) {
    set_sys_clock_48mhz();
    stdio_init_all();

    printf("SD Card test\n");

    int i = 0;

    // 0-2 for SD_CLK, SD_CMD, SD_DAT

    int sd_pin_base = 25;

//    gpio_init(0);
//    gpio_init(5);
//    gpio_init(10);
//    gpio_set_dir_out_masked(0x421);

#if ENABLE_4_PIN
    if (sd_init_4pin() < 0)
#else
    if (sd_init_1pin() < 0)
#endif
    {
        panic("doh");
    }

    static int block_base = 0;
    #define BLOCK_COUNT 2

#define STREAMING

#ifdef STREAMING
    static uint32_t b[BLOCK_COUNT * 128];
    for(int div = 4; div >= 1; div--)
    {
        uint8_t *buf = (uint8_t *)b;
        printf("-----------------------\n");
        printf("SPEED %uMB/s\n", 12/div);
        sd_set_clock_divider(div);
        printf("1 bit no crc\n");
        sd_set_wide_bus(false);
        memset(buf, 0xaa, 512);
        sd_readblocks_sync(b, block_base, BLOCK_COUNT);
        for(int byte = 0; byte < 512; byte += 16)
        {
            printf("%08x ", i * 512 + byte);
            for(int j = 0; j < 16; j++) printf("%02x ", buf[byte + j]);
            for(int j = 0; j < 16; j++) putchar(isprint(buf[byte + j]) ? buf[byte + j] : '.');
            printf("\n");
        }
#if ENABLE_4_PIN
        memset(buf, 0xaa, 512);
        printf("4 bit no crc\n");
        sd_set_wide_bus(true);
        sd_readblocks_sync(b, block_base, BLOCK_COUNT);
        for(int byte = 0; byte < 512; byte += 16)
        {
            printf("%08x ", i * 512 + byte);
            for(int j = 0; j < 16; j++) printf("%02x ", buf[byte + j]);
            for(int j = 0; j < 16; j++) putchar(isprint(buf[byte + j]) ? buf[byte + j] : '.');
            printf("\n");
        }
#endif
        memset(buf, 0xaa, 512);
        printf("1 bit crc\n");
        sd_read_sectors_1bit_crc_async(b, block_base, BLOCK_COUNT);
        int status = 0;
        while (!sd_scatter_read_complete(&status));
        printf("Status: %d\n", status);
#endif
        for(i = 0; i < BLOCK_COUNT; i++)
        {
#ifndef STREAMING
            uint8_t *buf = sd_readblock(i);
#endif
            //if (i == BLOCK_COUNT-1)
            for(int byte = 0; byte < 512; byte += 16)
            {
                printf("%08x ", i * 512 + byte);
                for(int j = 0; j < 16; j++) printf("%02x ", buf[byte + j]);
                for(int j = 0; j < 16; j++) putchar(isprint(buf[byte + j]) ? buf[byte + j] : '.');
                printf("\n");
            }
            printf("\n");
#ifdef STREAMING
            buf += 512;
#endif
        }
    }

#if 0
    strcpy(sector_data, "fish And Hello there zif squiffy!");
    sector_data[511] = 0xaa;
    sd_writeblocks_async((uint32_t*)sector_data, 0, 1);
    static int timeout = 10;
    int rc;
    while (!sd_write_complete(&rc)) {
        printf("Waiting for completion\n");
        if (!--timeout) break;
    }
    printf("Done %d!\n", rc);
    strcpy(sector_data, "vasil fleplic yoeville frentucky arrivant sklim avary ron giblet And Hello there zif squiffy!");
    sector_data[511] = 0x55;
    strcpy(sector_data + 512, "and this is sector 2 folks");
    sd_writeblocks_async((uint32_t*)sector_data, 0, 2);
    timeout = 10;
    while (!sd_write_complete(&rc)) {
        printf("Waiting for completion\n");
        if (!--timeout) break;
    }
#endif
    printf("Done!\n");
    __breakpoint();
}
