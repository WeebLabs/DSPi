/*
 * control_transport.c — I2C/UART wrappers for the shared control executor
 *
 * Non-USB transports carry the same logical fields as USB vendor requests:
 *
 *   request:  "D" "C" 0x01 dir bRequest wValueLE wIndexLE wLengthLE payload...
 *   response: "D" "C" 0x81 status lengthLE payload...
 *
 * `wIndex` is preserved in the frame and intentionally ignored by the current
 * executor, matching the USB vendor callback's compatibility behavior.
 */

#include "control_transport.h"

#include "bulk_params.h"
#include "config.h"
#include "control_executor.h"

#include "hardware/sync.h"
#include "pico/stdlib.h"

#if DSPI_CONTROL_I2C_PRIORITY >= 0
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/regs/i2c.h"
#endif

#if DSPI_CONTROL_UART_PRIORITY >= 0
#include "hardware/irq.h"
#include "hardware/uart.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CONTROL_MAGIC0 'D'
#define CONTROL_MAGIC1 'C'
#define CONTROL_REQUEST_VERSION  0x01
#define CONTROL_RESPONSE_VERSION 0x81
#define CONTROL_REQUEST_HEADER_LEN 11
#define CONTROL_RESPONSE_HEADER_LEN 6

typedef enum {
    CONTROL_LINK_DISABLED = 0,
    CONTROL_LINK_IDLE,
    CONTROL_LINK_RECEIVING,
    CONTROL_LINK_ACCEPTED,
    CONTROL_LINK_EXECUTING,
    CONTROL_LINK_RESPONSE_READY,
    CONTROL_LINK_DRAINING,
    CONTROL_LINK_ERROR,
} ControlLinkState;

typedef enum {
    CONTROL_STATUS_OK = 0,
    CONTROL_STATUS_BUSY = 1,
    CONTROL_STATUS_STALL = 2,
    CONTROL_STATUS_BAD_FRAME = 3,
    CONTROL_STATUS_OVERFLOW = 4,
} ControlWireStatus;

typedef struct {
    const char *name;
    int8_t priority;
    volatile ControlLinkState state;
    uint8_t rx[CONTROL_REQUEST_HEADER_LEN + WIRE_BULK_BUF_SIZE];
    uint16_t rx_len;
    uint16_t expected_len;
    uint8_t tx[CONTROL_RESPONSE_HEADER_LEN + WIRE_BULK_BUF_SIZE];
    uint16_t tx_len;
    uint16_t tx_pos;
    uint8_t busy_pos;
    volatile bool bootloader_due;
    bool bootloader_after_response;
} ControlLink;

static const uint8_t busy_response[CONTROL_RESPONSE_HEADER_LEN] = {
    CONTROL_MAGIC0, CONTROL_MAGIC1, CONTROL_RESPONSE_VERSION,
    CONTROL_STATUS_BUSY, 0, 0
};

static uint16_t read_le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void write_le16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)(value >> 8);
}

static void link_reset_to_idle(ControlLink *link) {
    link->rx_len = 0;
    link->expected_len = 0;
    link->tx_len = 0;
    link->tx_pos = 0;
    link->busy_pos = 0;
    link->bootloader_after_response = false;
    link->state = CONTROL_LINK_IDLE;
    __dmb();
}

static void link_build_response(ControlLink *link, ControlWireStatus status,
                                const uint8_t *payload, uint16_t payload_len,
                                bool enter_bootloader) {
    if (payload_len > WIRE_BULK_BUF_SIZE) {
        payload_len = WIRE_BULK_BUF_SIZE;
        status = CONTROL_STATUS_OVERFLOW;
    }

    link->tx[0] = CONTROL_MAGIC0;
    link->tx[1] = CONTROL_MAGIC1;
    link->tx[2] = CONTROL_RESPONSE_VERSION;
    link->tx[3] = (uint8_t)status;
    write_le16(&link->tx[4], payload_len);
    if (payload_len > 0 && payload) {
        memcpy(&link->tx[CONTROL_RESPONSE_HEADER_LEN], payload, payload_len);
    }

    link->tx_len = CONTROL_RESPONSE_HEADER_LEN + payload_len;
    link->tx_pos = 0;
    link->bootloader_after_response = enter_bootloader;
    link->state = CONTROL_LINK_RESPONSE_READY;
    __dmb();
}

static void link_build_error(ControlLink *link, ControlWireStatus status) {
    link_build_response(link, status, NULL, 0, false);
}

static void link_accept(ControlLink *link) {
    link->state = CONTROL_LINK_ACCEPTED;
    __dmb();
}

static void link_rx_byte(ControlLink *link, uint8_t byte) {
    ControlLinkState state = link->state;
    if (state == CONTROL_LINK_DISABLED ||
        state == CONTROL_LINK_ACCEPTED ||
        state == CONTROL_LINK_EXECUTING ||
        state == CONTROL_LINK_RESPONSE_READY ||
        state == CONTROL_LINK_DRAINING) {
        return;
    }

    if (state == CONTROL_LINK_IDLE || state == CONTROL_LINK_ERROR) {
        link->rx_len = 0;
        link->expected_len = 0;
        link->state = CONTROL_LINK_RECEIVING;
    }

    if (link->rx_len == 0 && byte != CONTROL_MAGIC0) {
        link->state = CONTROL_LINK_IDLE;
        return;
    }
    if (link->rx_len == 1 && byte != CONTROL_MAGIC1) {
        link->rx_len = 0;
        link->state = CONTROL_LINK_IDLE;
        return;
    }
    if (link->rx_len == 2 && byte != CONTROL_REQUEST_VERSION) {
        link_build_error(link, CONTROL_STATUS_BAD_FRAME);
        return;
    }
    if (link->rx_len >= sizeof(link->rx)) {
        link_build_error(link, CONTROL_STATUS_OVERFLOW);
        return;
    }

    link->rx[link->rx_len++] = byte;

    if (link->rx_len == CONTROL_REQUEST_HEADER_LEN) {
        uint8_t dir = link->rx[3];
        uint16_t payload_len = read_le16(&link->rx[9]);
        if (dir != CONTROL_DIR_OUT && dir != CONTROL_DIR_IN) {
            link_build_error(link, CONTROL_STATUS_BAD_FRAME);
            return;
        }
        if (payload_len > WIRE_BULK_BUF_SIZE) {
            link_build_error(link, CONTROL_STATUS_OVERFLOW);
            return;
        }

        link->expected_len = CONTROL_REQUEST_HEADER_LEN;
        if (dir == CONTROL_DIR_OUT) {
            link->expected_len += payload_len;
        }
        if (link->expected_len == link->rx_len) {
            link_accept(link);
        }
    } else if (link->expected_len != 0 && link->rx_len == link->expected_len) {
        link_accept(link);
    }
}

static void link_end_write(ControlLink *link) {
    if (link->state == CONTROL_LINK_RECEIVING) {
        link_build_error(link, CONTROL_STATUS_BAD_FRAME);
    }
}

static bool link_has_tx(const ControlLink *link) {
    return link->state == CONTROL_LINK_RESPONSE_READY ||
           link->state == CONTROL_LINK_DRAINING;
}

static void link_finish_drain(ControlLink *link, bool complete) {
    bool boot = complete && link->bootloader_after_response;
    link_reset_to_idle(link);
    if (boot) {
        link->bootloader_due = true;
        __dmb();
    }
}

static bool link_take_tx_byte(ControlLink *link, uint8_t *byte) {
    if (!link_has_tx(link)) return false;
    link->state = CONTROL_LINK_DRAINING;

    if (link->tx_pos < link->tx_len) {
        *byte = link->tx[link->tx_pos++];
        return true;
    }

    link_finish_drain(link, true);
    return false;
}

static uint8_t link_read_byte(ControlLink *link) {
    uint8_t byte;
    if (link_take_tx_byte(link, &byte)) {
        return byte;
    }

    byte = busy_response[link->busy_pos % CONTROL_RESPONSE_HEADER_LEN];
    link->busy_pos++;
    return byte;
}

static void link_end_read(ControlLink *link) {
    link->busy_pos = 0;
    if (link->state == CONTROL_LINK_DRAINING) {
        link_finish_drain(link, link->tx_pos >= link->tx_len);
    }
}

static ControlRequest link_request(const ControlLink *link) {
    ControlRequest request = {
        .direction = (ControlDirection)link->rx[3],
        .bRequest = link->rx[4],
        .wValue = read_le16(&link->rx[5]),
        .wIndex = read_le16(&link->rx[7]),
        .wLength = read_le16(&link->rx[9]),
    };
    return request;
}

static void link_execute(ControlLink *link) {
    if (link->state != CONTROL_LINK_ACCEPTED) return;

    link->state = CONTROL_LINK_EXECUTING;
    ControlRequest request = link_request(link);

    if (request.direction == CONTROL_DIR_IN) {
        ControlResponse response;
        if (!control_executor_execute_in(&request, &response)) {
            link_build_error(link, CONTROL_STATUS_STALL);
            return;
        }
        link_build_response(link, CONTROL_STATUS_OK, response.data,
                            response.length, response.enter_bootloader);
        return;
    }

    ControlOutBuffer out;
    if (!control_executor_prepare_out(&request, &out)) {
        link_build_error(link, CONTROL_STATUS_STALL);
        return;
    }

    const uint8_t *payload = &link->rx[CONTROL_REQUEST_HEADER_LEN];
    if (!out.immediate_status && out.length > 0 && out.data) {
        memcpy(out.data, payload, out.length);
        control_executor_execute_out(&request, out.data, out.length);
    }
    control_executor_commit_out(&request);
    link_build_response(link, CONTROL_STATUS_OK, NULL, 0, false);
}

static void link_check_bootloader(ControlLink *link) {
    if (!link->bootloader_due) return;
    link->bootloader_due = false;
    __dmb();
    control_executor_enter_bootloader();
}

#if DSPI_CONTROL_I2C_PRIORITY >= 0

#if DSPI_CONTROL_I2C_INSTANCE == 0
#define DSPI_CONTROL_I2C_HW i2c0
#define DSPI_CONTROL_I2C_IRQ I2C0_IRQ
#else
#define DSPI_CONTROL_I2C_HW i2c1
#define DSPI_CONTROL_I2C_IRQ I2C1_IRQ
#endif

static ControlLink i2c_link = {
    .name = "i2c",
    .priority = DSPI_CONTROL_I2C_PRIORITY,
    .state = CONTROL_LINK_IDLE,
};
static volatile bool i2c_saw_rx;
static volatile bool i2c_saw_read;

static void control_i2c_irq_handler(void) {
    i2c_hw_t *hw = i2c_get_hw(DSPI_CONTROL_I2C_HW);
    uint32_t status = hw->intr_stat;

    while (hw->status & I2C_IC_STATUS_RFNE_BITS) {
        uint8_t byte = (uint8_t)hw->data_cmd;
        i2c_saw_rx = true;
        link_rx_byte(&i2c_link, byte);
    }

    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        (void)hw->clr_rd_req;
        i2c_saw_read = true;
        hw->data_cmd = link_read_byte(&i2c_link);
    }

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        (void)hw->clr_tx_abrt;
    }

    if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        (void)hw->clr_stop_det;
        if (i2c_saw_read) {
            link_end_read(&i2c_link);
        } else if (i2c_saw_rx) {
            link_end_write(&i2c_link);
        }
        i2c_saw_rx = false;
        i2c_saw_read = false;
    }
}

static void control_i2c_init(void) {
    i2c_init(DSPI_CONTROL_I2C_HW, DSPI_CONTROL_I2C_BAUD);
    gpio_set_function(DSPI_CONTROL_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DSPI_CONTROL_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DSPI_CONTROL_I2C_SDA_PIN);
    gpio_pull_up(DSPI_CONTROL_I2C_SCL_PIN);
    i2c_set_slave_mode(DSPI_CONTROL_I2C_HW, true, DSPI_CONTROL_I2C_ADDR);

    i2c_hw_t *hw = i2c_get_hw(DSPI_CONTROL_I2C_HW);
    hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS |
                    I2C_IC_INTR_MASK_M_RD_REQ_BITS |
                    I2C_IC_INTR_MASK_M_TX_ABRT_BITS |
                    I2C_IC_INTR_MASK_M_STOP_DET_BITS;

    irq_set_exclusive_handler(DSPI_CONTROL_I2C_IRQ, control_i2c_irq_handler);
    irq_set_enabled(DSPI_CONTROL_I2C_IRQ, true);
}

#endif

#if DSPI_CONTROL_UART_PRIORITY >= 0

#if DSPI_CONTROL_UART_INSTANCE == 0
#define DSPI_CONTROL_UART_HW uart0
#define DSPI_CONTROL_UART_IRQ UART0_IRQ
#else
#define DSPI_CONTROL_UART_HW uart1
#define DSPI_CONTROL_UART_IRQ UART1_IRQ
#endif

static ControlLink uart_link = {
    .name = "uart",
    .priority = DSPI_CONTROL_UART_PRIORITY,
    .state = CONTROL_LINK_IDLE,
};

static void control_uart_irq_handler(void) {
    while (uart_is_readable(DSPI_CONTROL_UART_HW)) {
        link_rx_byte(&uart_link, uart_getc(DSPI_CONTROL_UART_HW));
    }

    while (uart_is_writable(DSPI_CONTROL_UART_HW)) {
        uint8_t byte;
        if (!link_take_tx_byte(&uart_link, &byte)) break;
        uart_putc_raw(DSPI_CONTROL_UART_HW, (char)byte);
    }

    uart_set_irq_enables(DSPI_CONTROL_UART_HW, true, link_has_tx(&uart_link));
}

static void control_uart_init(void) {
    uart_init(DSPI_CONTROL_UART_HW, DSPI_CONTROL_UART_BAUD);
    gpio_set_function(DSPI_CONTROL_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DSPI_CONTROL_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(DSPI_CONTROL_UART_HW, true);
    uart_set_irq_enables(DSPI_CONTROL_UART_HW, true, false);

    irq_set_exclusive_handler(DSPI_CONTROL_UART_IRQ, control_uart_irq_handler);
    irq_set_enabled(DSPI_CONTROL_UART_IRQ, true);
}

static void control_uart_kick_tx(void) {
    if (link_has_tx(&uart_link)) {
        uart_set_irq_enables(DSPI_CONTROL_UART_HW, true, true);
    }
}

#endif

void control_transports_init(void) {
#if DSPI_CONTROL_I2C_PRIORITY >= 0
    control_i2c_init();
#endif
#if DSPI_CONTROL_UART_PRIORITY >= 0
    control_uart_init();
#endif
}

void control_transports_tick(void) {
    ControlLink *best = NULL;

#if DSPI_CONTROL_I2C_PRIORITY >= 0
    if (i2c_link.state == CONTROL_LINK_ACCEPTED) {
        best = &i2c_link;
    }
#endif
#if DSPI_CONTROL_UART_PRIORITY >= 0
    if (uart_link.state == CONTROL_LINK_ACCEPTED &&
        (!best || uart_link.priority > best->priority)) {
        best = &uart_link;
    }
#endif

    if (best) {
        link_execute(best);
    }

#if DSPI_CONTROL_UART_PRIORITY >= 0
    control_uart_kick_tx();
#endif
#if DSPI_CONTROL_I2C_PRIORITY >= 0
    link_check_bootloader(&i2c_link);
#endif
#if DSPI_CONTROL_UART_PRIORITY >= 0
    link_check_bootloader(&uart_link);
#endif
}
