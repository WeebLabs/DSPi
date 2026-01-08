/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "hardware/sync.h"
#include "pico/usb_device.h"
#include "pico/usb_device_msc.h"
#include "pico/scsi.h"
#include "pico/scsi_ir.h"
#include "pico/virtual_disk.h"
#include "pico/usb_stream_helper.h"

static __attribute__((aligned(4))) uint8_t _sector_buf[SECTOR_SIZE];

struct __packed scsi_request_sense_response {
    uint8_t code;
    uint8_t _pad;
    uint8_t key;
    uint32_t _info;
    uint8_t additonal_sense_len;
    uint32_t _cmd_specific;
    uint8_t asc;
    uint8_t ascq;
    uint8_t _fruc;
    uint8_t _sense_specific[3];
};
static_assert(sizeof(struct scsi_request_sense_response) == 18, "");

enum scsi_direction {
    SCSI_DIR_NONE = 0,
    SCSI_DIR_IN = 1,
    SCSI_DIR_OUT = 2,
};

static struct msc_state {
    struct scsi_csw csw;
    struct scsi_request_sense_response request_sense;
    uint32_t data_phase_length;
    uint8_t stall_direction_before_csw;
    bool send_csw_on_unstall;
    bool ejected;
} _msc_state;

// not part of _msc_state since we never reset it
static uint32_t _msc_async_token;

void _msc_cmd_packet(struct usb_endpoint *ep);

static const struct usb_transfer_type _msc_cmd_transfer_type = {
        .on_packet = _msc_cmd_packet,
        .initial_packet_count = 1,
};

// note we need these to be adjacent, so rather than relying on the fact just making them into an array which seems to produce the same code otherwise
//struct usb_endpoint msc_in, msc_out;

struct usb_endpoint msc_endpoints[2];
#define msc_in msc_endpoints[0]
#define msc_out msc_endpoints[1]

static struct usb_transfer _msc_cmd_transfer;
static struct usb_transfer _msc_cmd_response_transfer;

static void _tf_wait_command(__unused struct usb_endpoint *ep, __unused struct usb_transfer *transfer) {
    assert(ep == &msc_in);
    usb_debug("_tf_wait_command\n");
    assert(msc_out.default_transfer);
#ifndef GENERAL_SIZE_HACKS
    usb_start_default_transfer_if_not_already_running_or_halted(&msc_out);
#else
    assert(&msc_out == &msc_in + 1);
    usb_start_default_transfer_if_not_already_running_or_halted(ep + 1);
#endif
}

static __noinline void _msc_reset_and_start_cmd_response_transfer(usb_transfer_completed_func func) {
    usb_reset_and_start_transfer(&msc_in, &_msc_cmd_response_transfer, &usb_current_packet_only_transfer_type, func);
}

static void _msc_send_csw() {
    _msc_state.send_csw_on_unstall = false;
    uint8_t *buffer = usb_get_single_packet_response_buffer(&msc_in, sizeof(_msc_state.csw));
    memcpy(buffer, &_msc_state.csw, sizeof(_msc_state.csw));
    _msc_reset_and_start_cmd_response_transfer(_tf_wait_command);
}

static void _msc_set_csw_failed(enum scsi_sense_key sk, enum scsi_additional_sense_code asc,
                                enum scsi_additional_sense_code_qualifier ascq) {
    _msc_state.csw.status = CSW_STATUS_COMMAND_FAILED;
    _msc_state.request_sense.key = sk;
    _msc_state.request_sense.asc = asc;
    _msc_state.request_sense.ascq = ascq;
}

static void _msc_data_phase_complete() {
    if (_msc_state.stall_direction_before_csw == SCSI_DIR_IN) {
        _msc_state.stall_direction_before_csw = SCSI_DIR_NONE;
        _msc_state.send_csw_on_unstall = true;
        usb_debug("Stalling in\n");
        usb_halt_endpoint(&msc_in);
    } else {
        if (_msc_state.stall_direction_before_csw == SCSI_DIR_OUT) {
            _msc_state.stall_direction_before_csw = SCSI_DIR_NONE;
            usb_debug("Stalling out\n");
            usb_halt_endpoint(&msc_out);
        }
        _msc_send_csw();
    }
}

#ifndef GENERAL_SIZE_HACKS

static void _tf_data_phase_complete(__unused struct usb_endpoint *endpoint, __unused struct usb_transfer *transfer) {
    assert(endpoint == &msc_in || endpoint == &msc_out);
    usb_debug("_tf_data_phase_complete\n");
    _msc_data_phase_complete();
}

#else
#define _tf_data_phase_complete ((usb_transfer_completed_func)_msc_data_phase_complete)
#endif

// noinline here saves us 4 bytes; go figure
static enum scsi_direction _scsi_dir(const struct scsi_cbw *cbw) {
    return (cbw->flags & USB_DIR_IN) ? SCSI_DIR_IN : SCSI_DIR_OUT;
}

static void _msc_init_for_dn(const struct scsi_cbw *cbw) {
    _msc_state.stall_direction_before_csw = SCSI_DIR_NONE;
    if (cbw->data_transfer_length) {
        enum scsi_direction cbw_dir = _scsi_dir(cbw);
        _msc_state.stall_direction_before_csw = cbw_dir;
    }
    _msc_data_phase_complete();
}

static bool _msc_init_for_di_or_do(const struct scsi_cbw *cbw, uint32_t expected_length, enum scsi_direction dir) {
    _msc_state.stall_direction_before_csw = SCSI_DIR_NONE;
    _msc_state.data_phase_length = 0;
    enum scsi_direction cbw_dir = _scsi_dir(cbw);
    if (cbw_dir != dir) {
        usb_debug("Will stall because direction wrong\n");
        _msc_state.stall_direction_before_csw = cbw_dir;
        _msc_state.csw.status = CSW_STATUS_PHASE_ERROR;
    } else {
        if (expected_length != cbw->data_transfer_length) {
            _msc_state.stall_direction_before_csw = dir;
        }
        if (expected_length > cbw->data_transfer_length) {
            _msc_state.csw.status = CSW_STATUS_PHASE_ERROR;
        }
        _msc_state.data_phase_length = MIN(expected_length, cbw->data_transfer_length);
    }
    usb_debug("_msc_init_for_di exp = %d tran = %d stall = %d status = %d length = %d\n", (uint) expected_length,
              (uint) cbw->data_transfer_length,
              _msc_state.stall_direction_before_csw, _msc_state.csw.status, (uint) _msc_state.data_phase_length);
    if (!_msc_state.data_phase_length) {
        _msc_data_phase_complete();
        return false;
    }
    return true;
}

static void _scsi_fail_cmd(const struct scsi_cbw *cbw, enum scsi_sense_key sk, enum scsi_additional_sense_code asc,
                           enum scsi_additional_sense_code_qualifier ascq) {
    _msc_set_csw_failed(sk, asc, ascq);
    // this handily takes care of the STALLing/CSW based on us not intending to send data
    _msc_init_for_dn(cbw);
}

static void _scsi_standard_response(const struct scsi_cbw *cbw) {
    struct usb_buffer *buffer = usb_current_in_packet_buffer(&msc_in);
    assert(buffer->data_len);
    if (_msc_init_for_di_or_do(cbw, MIN(buffer->data_len, cbw->data_transfer_length), SCSI_DIR_IN)) {
        assert(_msc_state.data_phase_length <= buffer->data_len);
        buffer->data_len = _msc_state.data_phase_length; // truncate buffer
        _msc_state.csw.residue -= buffer->data_len;
        _msc_reset_and_start_cmd_response_transfer(_tf_data_phase_complete);
    }
}

static_assert(sizeof(struct scsi_inquiry_response) == 36, "");

static void _scsi_handle_inquiry_response(struct scsi_cbw *cbw) {
    uint8_t *buf = usb_get_single_packet_response_buffer(&msc_in, sizeof(struct scsi_inquiry_response));
    memcpy(buf, &scsi_ir, sizeof(scsi_ir));
    _scsi_standard_response(cbw);
}

static struct msc_sector_transfer {
    struct usb_stream_transfer stream;
    uint32_t lba;
} _msc_sector_transfer;

static void _msc_on_sector_stream_packet_complete(__unused struct usb_stream_transfer *transfer) {
    assert(transfer == &_msc_sector_transfer.stream);
    _msc_state.csw.residue -= 64;
}

bool _msc_on_sector_stream_chunk(__unused uint32_t chunk_len, __unused struct usb_stream_transfer *transfer) {
    assert(transfer == &_msc_sector_transfer.stream);
    assert(chunk_len == SECTOR_SIZE);
    bool (*vd_read_or_write)(uint32_t token, uint32_t lba, uint8_t *buf, uint32_t buf_size);
    vd_read_or_write = _msc_sector_transfer.stream.ep->in ? vd_read_block : vd_write_block;
    return vd_read_or_write(++_msc_async_token, _msc_sector_transfer.lba++, _sector_buf, SECTOR_SIZE);
}

static const struct usb_stream_transfer_funcs _msc_sector_funcs = {
        .on_packet_complete = _msc_on_sector_stream_packet_complete,
        .on_chunk = _msc_on_sector_stream_chunk
};

// note that this may be called during regular vd_operation
void vd_async_complete(uint32_t token, uint32_t result) {
    usb_debug("complete token %d\n", (int) token);
    // note that this USB library is not thread safe, however this is the only function called
    // from non IRQ handler code after usb_device_start; therefore we just disable IRQs for this call
    uint32_t save = save_and_disable_interrupts();
    if (token == _msc_async_token) {
        if (result) {
            // if we error, we'll just abort and send csw
            // todo does it matter what we send? - we have a residue - prefer to send locked or write error
#ifndef USB_SILENT_FAIL_ON_EXCLUSIVE
            _msc_set_csw_failed(SK_DATA_PROTECT, ASC_ACCESS_DENIED, 2); // no access rights
#endif
            _msc_state.stall_direction_before_csw = SCSI_DIR_OUT;
            _msc_data_phase_complete();
        }
        usb_stream_chunk_done(&_msc_sector_transfer.stream);
    } else {
        usb_warn("async complete for incorrect token %d != %d\n", (int) token, (int) _msc_async_token);
    }
    restore_interrupts(save);
}

static void _scsi_read_or_write_blocks(const struct scsi_cbw *cbw, uint32_t lba, uint32_t blocks,
                                       enum scsi_direction dir) {
    assert(dir);
    _msc_sector_transfer.stream.ep = (dir == SCSI_DIR_IN) ? &msc_in : &msc_out;
    _msc_sector_transfer.lba = lba;
    uint32_t expected_length = blocks * SECTOR_SIZE;
    if (_msc_init_for_di_or_do(cbw, expected_length, dir)) {
        assert(_msc_state.data_phase_length <= expected_length);
        expected_length = _msc_state.data_phase_length /
                          64; // round down... this means we may send less than dwTransferLength, but residue will be correct
        // todo we could remove the if if start_transfer allows empty transfers
        if (expected_length) {
            _msc_async_token++;
            // transfer length is exact multiple of 64 as per above rounding comment
            usb_stream_setup_transfer(&_msc_sector_transfer.stream, &_msc_sector_funcs, _sector_buf, SECTOR_SIZE,
                                      expected_length * 64,
                                      _tf_data_phase_complete);
            if (dir == SCSI_DIR_IN) {
                usb_start_transfer(&msc_in, &_msc_sector_transfer.stream.core);
            } else {
                usb_chain_transfer(&msc_out, &_msc_sector_transfer.stream.core);
            }
        } else {
            _msc_data_phase_complete();
        }
    }
}

static void _scsi_handle_test_unit_ready(const struct scsi_cbw *cbw) {
    if (_msc_state.ejected) {
        return _scsi_fail_cmd(cbw, SK_NOT_READY, ASC_MEDIUM_NOT_PRESENT, ASCQ_NA);
    }
    return _msc_init_for_dn(cbw);
}

void msc_eject() {
    _msc_state.ejected = true;
}

static void _scsi_handle_start_stop_unit(const struct scsi_cbw *cbw) {
    if (2u == (cbw->cb[4] & 3u)) {
        usb_warn("EJECT immed %02x\n", cbw->cb[1]);
        msc_eject();
    }
    return _msc_init_for_dn(cbw);
}

static void _scsi_handle_read_or_write_command(const struct scsi_cbw *cbw, enum scsi_direction dir) {
    const struct scsi_read_cb *cb = (const struct scsi_read_cb *) &cbw->cb[0];
    uint32_t lba = __builtin_bswap32(cb->lba);
    uint16_t blocks = __builtin_bswap16(cb->blocks);
    usb_debug(dir == SCSI_DIR_IN ? "Read %d blocks starting at lba %ld\n" :
              "Write %d blocks starting at lba %ld\n",
              blocks, lba);
    _scsi_read_or_write_blocks(cbw, lba, blocks, dir);
}

static void _scsi_memcpy_response(const struct scsi_cbw *cbw, uint8_t *data, uint len) {
    memcpy(usb_get_single_packet_response_buffer(&msc_in, len), data, len);
    _scsi_standard_response(cbw);
}

static void _scsi_handle_read_capacity(const struct scsi_cbw *cbw) {
    struct scsi_capacity _resp = {
            .lba = __builtin_bswap32(vd_sector_count() - 1),
            .block_len = __builtin_bswap32(SECTOR_SIZE)
    };
    _scsi_memcpy_response(cbw, (uint8_t *) &_resp, sizeof(_resp));
}

struct __packed scsi_read_format_capacity_response {
    uint8_t _pad[3];
    uint8_t descriptors_size;
    uint32_t descriptor_1_block_count_msb;
    uint32_t descriptor_1_type_and_block_size;
};

static void _scsi_handle_read_format_capacities(const struct scsi_cbw *cbw) {
    struct scsi_read_format_capacity_response _resp = {
            .descriptor_1_block_count_msb = __builtin_bswap32(vd_sector_count() - 1),
            .descriptor_1_type_and_block_size = 2u | // formatted
                                                __builtin_bswap32(SECTOR_SIZE)
    };
    _scsi_memcpy_response(cbw, (uint8_t *) &_resp, sizeof(_resp));
}

static void _scsi_handle_request_sense(const struct scsi_cbw *cbw) {
    uint8_t *buf = usb_get_single_packet_response_buffer(&msc_in, sizeof(_msc_state.request_sense));
//    printf("RS %d\n", scsi.request_sense.key);
    memcpy(buf, &_msc_state.request_sense, sizeof(_msc_state.request_sense));
    _msc_state.request_sense.key = SK_OK;
    _msc_state.request_sense.asc = 0;
    _msc_state.request_sense.ascq = 0;
    _scsi_standard_response(cbw);
}

static void _scsi_handle_mode_sense(const struct scsi_cbw *cbw) {
    uint8_t *buf = usb_get_single_packet_response_buffer(&msc_in, 4);
    *(uint32_t *) buf = 3;
    _scsi_standard_response(cbw);
}

static void _msc_in_on_stall_change(struct usb_endpoint *ep) {
    usb_debug("Stall change in stalled %d send csw %d \n", usb_is_endpoint_stalled(ep), _msc_state.send_csw_on_unstall);
    if (!usb_is_endpoint_stalled(ep) && ep == &msc_in) {
        // todo we need to clear this on the ep cancel
        if (_msc_state.send_csw_on_unstall) {
            usb_debug("Sending CSW on unstall\n");
            _msc_send_csw();
        }
    }
}

static void _msc_reset(void) {
    static bool one_time;
    if (!one_time) {
        _msc_cmd_transfer.type = &_msc_cmd_transfer_type;
        usb_set_default_transfer(&msc_out, &_msc_cmd_transfer);
        msc_in.on_stall_change = _msc_in_on_stall_change;
        vd_init();
        one_time = true;
    }
    memset(&_msc_state, 0, sizeof(_msc_state));
    _msc_state.request_sense.code = 0x70;
    _msc_state.request_sense.additonal_sense_len = 0xa;
    vd_reset();
    usb_soft_reset_endpoint(&msc_in);
    usb_soft_reset_endpoint(&msc_out);
}

static void _msc_cmd_halt() {
    usb_halt_endpoint_on_condition(&msc_in);
    usb_halt_endpoint_on_condition(&msc_out);
}

static void _msc_cmd_packet_internal(struct usb_endpoint *ep) {
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
    uint len = buffer->data_len;

    struct scsi_cbw *cbw = (struct scsi_cbw *) buffer->data;
    if (len == 31u && cbw->sig == CBW_SIG && !cbw->lun && !(cbw->flags & 0x7fu) && cbw->cb_length &&
        cbw->cb_length <= 16) {
        // todo we need to validate CBW sizes
        _msc_state.csw.sig = CSW_SIG;
        _msc_state.csw.tag = cbw->tag;
        _msc_state.csw.residue = cbw->data_transfer_length;
        usb_debug("SCSI: ");
        enum scsi_cmd cmd = cbw->cb[0];
        if (cmd != REQUEST_SENSE) {
            _msc_state.request_sense.key = SK_OK;
            _msc_state.request_sense.asc = 0;
            _msc_state.request_sense.ascq = 0;
        }
        _msc_state.csw.status = CSW_STATUS_COMMAND_PASSED;
        switch (cmd) {
            case INQUIRY:
                usb_debug("INQUIRY\n");
                return _scsi_handle_inquiry_response(cbw);
            case MODE_SENSE_6:
                usb_debug("MODESENSE(6)\n");
                return _scsi_handle_mode_sense(cbw);
            case PREVENT_ALLOW_MEDIUM_REMOVAL:
                usb_debug("PREVENT ALLOW MEDIUM REMOVAL\n");// %d\n", buf[4] & 3u);
                // Nothing to do just reply success
                return _msc_init_for_dn(cbw);
            case READ_10:
                usb_debug("READ(10)\n");
                return _scsi_handle_read_or_write_command(cbw, SCSI_DIR_IN);
            case WRITE_10:
                usb_debug("WRITE(10)\n");
                return _scsi_handle_read_or_write_command(cbw, SCSI_DIR_OUT);
            case READ_FORMAT_CAPACITIES:
                usb_debug("READ FORMAT_CAPACITIES\n");
                return _scsi_handle_read_format_capacities(cbw);
            case READ_CAPACITY_10:
                usb_debug("READ CAPACITY(10)\n");
                return _scsi_handle_read_capacity(cbw);
            case REQUEST_SENSE:
                usb_debug("REQUEST SENSE\n");
                return _scsi_handle_request_sense(cbw);
            case TEST_UNIT_READY:
                usb_debug("TEST UNIT READY\n");
                return _scsi_handle_test_unit_ready(cbw);
            case START_STOP_UNIT:
                usb_debug("START STOP UNIT\n");
                return _scsi_handle_start_stop_unit(cbw);
            case SYNCHRONIZE_CACHE:
                usb_debug("SYNCHRONIZE CACHE(10)\n");
                return _msc_init_for_dn(cbw);
            case VERIFY:
                usb_debug("VERIFY\n");
                return _msc_init_for_dn(cbw);
            default:
                usb_debug("cmd %02x\n", cbw->cb[0]);
                break;
        }
        return _scsi_fail_cmd(cbw, SK_ILLEGAL_REQUEST, ASC_INVALID_COMMAND_OPERATION_CODE, ASCQ_NA);
    } else {
        usb_debug("invalid cbw\n");
        return _msc_cmd_halt();
    }
}

void _msc_cmd_packet(struct usb_endpoint *ep) {
    _msc_cmd_packet_internal(ep);
    usb_packet_done(ep);
}

bool msc_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        if (setup->bmRequestType & USB_DIR_IN) {
            if (setup->bRequest == USB_REQUEST_MSC_GET_MAX_LUN) {
                if (!setup->wValue && setup->wLength) {
                    usb_debug("GET_MAX_LUN\n");
                    struct usb_buffer *buffer = usb_current_in_packet_buffer(usb_get_control_in_endpoint());
                    buffer->data[0] = 0;
                    buffer->data_len = 1;
                    usb_start_single_buffer_control_in_transfer();
                    return true;
                } else {
                    usb_debug("INVALID GET_MAX_LUN\n");
                }
            }
        } else {
            if (setup->bRequest == USB_REQUEST_MSC_RESET) {
                if (!setup->wValue && !setup->wLength) {
                    usb_debug("MSC_RESET\n");
                    // doesn't unstall, but allows CLEAR_HALT to proceed
                    usb_clear_halt_condition(&msc_in);
                    usb_clear_halt_condition(&msc_out);
                    _msc_reset();
                    usb_start_empty_control_in_transfer_null_completion();
                    return true;
                } else {
                    usb_debug("INVALID MSC_RESET\n");
                }
            }
        }
    }
    return false;
}


void msc_on_configure(__unused struct usb_device *device, bool configured) {
    if (configured) {
        _msc_reset();
    }
}
