/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SCSI_IR_H
#define _SCSI_IR_H

// NOTE THIS IS IN A SEPARATE HEADER AS IT IS COMPRESSED WHEN USING COMPRESS_TEXT
typedef unsigned char uint8_t;

struct scsi_inquiry_response {
    uint8_t pdt;
    uint8_t rmb;
    uint8_t spc_version;
    uint8_t rdf;
    uint8_t additional_length;
    uint8_t inquiry5;
    uint8_t inquiry6;
    uint8_t inquiry7;
    char vendor[8];
    char product[16];
    char version[4];
} __packed;

#ifndef COMPRESS_TEXT
static const struct scsi_inquiry_response scsi_ir = {
        .rmb = 0x80,
        .spc_version = 2,
        .rdf = 2,
        .additional_length = sizeof(struct scsi_inquiry_response) - 4,
        .vendor  = "RPI     ",
        .product = "RP2             ",
        .version = "1   ",
};
#endif
#endif