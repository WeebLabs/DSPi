/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SCSI_H
#define _SCSI_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#define CBW_SIG 0x43425355
struct __aligned(4) __packed scsi_cbw {
    uint32_t sig;
    uint32_t tag;
    uint32_t data_transfer_length;
    uint8_t flags;
    uint8_t lun;
    uint8_t cb_length;
    uint8_t cb[16];
};

#define CSW_SIG 0x53425355
struct __packed scsi_csw {
    uint32_t sig;
    uint32_t tag;
    uint32_t residue;
    uint8_t status;
};

struct __packed scsi_capacity {
    uint32_t lba; // last block addr
    uint32_t block_len; // probably 512
};

struct __packed scsi_read_cb {
    uint8_t opcode;
    uint8_t flags;
    uint32_t lba;
    uint8_t reserved;
    uint16_t blocks;
    uint8_t control;
};

enum csw_status {
    CSW_STATUS_COMMAND_PASSED = 0x00,
    CSW_STATUS_COMMAND_FAILED = 0x01,
    CSW_STATUS_PHASE_ERROR = 0x02,
};

enum scsi_cmd {
    INQUIRY = 0x12,
    MODE_SELECT_6 = 0x15,
    MODE_SELECT_10 = 0x55,
    MODE_SENSE_6 = 0x1a,
    MODE_SENSE_10 = 0x5a,
    PREVENT_ALLOW_MEDIUM_REMOVAL = 0x1e,
    READ_6 = 0x08,
    READ_10 = 0x28,
    READ_12 = 0xa8,
    READ_FORMAT_CAPACITIES = 0x23,
    READ_CAPACITY_10 = 0x25,
    REPORT_LUNS = 0xa0,
    REQUEST_SENSE = 0x03,
    SEND_DIAGNOSTIC = 0x1d,
    START_STOP_UNIT = 0x1b,
    SYNCHRONIZE_CACHE = 0x35,
    TEST_UNIT_READY = 0x00,
    VERIFY = 0x2f,
    WRITE_6 = 0x0a,
    WRITE_10 = 0x2a,
    WRITE_12 = 0xaa,
};

enum scsi_sense_key {
    SK_OK = 0x00,
    SK_NOT_READY = 0x02,
    SK_ILLEGAL_REQUEST = 0x05,
    SK_UNIT_ATTENTION = 0x06,
    SK_DATA_PROTECT = 0x07
};

enum scsi_additional_sense_code {
    ASC_NONE = 0x00,
    ASC_INVALID_COMMAND_OPERATION_CODE = 0x20,
    ASC_PERIPHERAL_DEVICE_WRITE_FAULT = 0x03,
    ASC_ACCESS_DENIED = 0x20,
    ASC_LBA_OUT_OF_RANGE = 0x21,
    ASC_WRITE_PROTECTED = 0x27,
    ASC_NOT_READY_TO_READY_CHANGE = 0x28,
    ASC_MEDIUM_NOT_PRESENT = 0x3a,
};

enum scsi_additional_sense_code_qualifier {
    ASCQ_NA = 0x00,
};
#endif