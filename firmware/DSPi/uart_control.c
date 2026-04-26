#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include "bulk_params.h"
#include "config.h"
#include "uart_control.h"
#include "usb_audio.h"

#if DSPi_UART_CONTROL_ENABLE

typedef enum {
    UART_RX_IDLE = 0,
    UART_RX_LINE,
    UART_RX_OVERFLOW,
} UartRxState;

static char uart_line[DSPi_UART_CONTROL_LINE_MAX];
static uint16_t uart_line_len = 0;
static uint32_t uart_last_rx_us = 0;
static UartRxState uart_rx_state = UART_RX_IDLE;
static uint8_t uart_payload[64];

static uart_inst_t *control_uart(void) {
#if DSPi_UART_CONTROL_UART == 1
    return uart1;
#else
    return uart0;
#endif
}

static void uart_write_str(const char *s) {
    uart_puts(control_uart(), s);
}

static char hex_digit(uint8_t v) {
    v &= 0x0F;
    return (v < 10) ? (char)('0' + v) : (char)('A' + (v - 10));
}

static void uart_write_hex(const uint8_t *data, uint16_t len) {
    char pair[3] = {0};
    for (uint16_t i = 0; i < len; i++) {
        pair[0] = hex_digit(data[i] >> 4);
        pair[1] = hex_digit(data[i]);
        uart_write_str(pair);
    }
}

static void uart_ok_hex(const uint8_t *data, uint16_t len) {
    uart_write_str("OK");
    if (len) {
        uart_write_str(" ");
        uart_write_hex(data, len);
    }
    uart_write_str("\r\n");
}

static void uart_ok_text(const char *text) {
    uart_write_str("OK");
    if (text && text[0]) {
        uart_write_str(" ");
        uart_write_str(text);
    }
    uart_write_str("\r\n");
}

static void uart_err(const char *code) {
    uart_write_str("ERR ");
    uart_write_str(code);
    uart_write_str("\r\n");
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool decode_hex(const char *hex, uint8_t *out, uint16_t out_capacity,
                       uint16_t *out_len) {
    size_t n = strlen(hex);
    if ((n & 1u) != 0) return false;
    if ((n / 2u) > out_capacity) return false;

    for (size_t i = 0; i < n; i += 2) {
        int hi = hex_value(hex[i]);
        int lo = hex_value(hex[i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i / 2u] = (uint8_t)((hi << 4) | lo);
    }
    *out_len = (uint16_t)(n / 2u);
    return true;
}

static bool parse_u16(const char *token, uint16_t *out) {
    if (!token || !token[0]) return false;
    char *end = NULL;
    unsigned long value = strtoul(token, &end, 0);
    if (*end != '\0' || value > 0xFFFFul) return false;
    *out = (uint16_t)value;
    return true;
}

static bool parse_u8(const char *token, uint8_t *out) {
    uint16_t value;
    if (!parse_u16(token, &value) || value > 0xFFu) return false;
    *out = (uint8_t)value;
    return true;
}

static void uppercase_token(char *token) {
    for (; *token; token++) {
        *token = (char)toupper((unsigned char)*token);
    }
}

static const char *control_result_code(ControlResult result) {
    switch (result) {
        case CONTROL_RESULT_OK: return "OK";
        case CONTROL_RESULT_ACCEPTED: return "ACCEPTED";
        case CONTROL_RESULT_UNSUPPORTED: return "UNSUPPORTED";
        case CONTROL_RESULT_INVALID_REQUEST: return "REQ";
        case CONTROL_RESULT_INVALID_VALUE: return "VALUE";
        case CONTROL_RESULT_INVALID_LENGTH: return "LEN";
        case CONTROL_RESULT_BUFFER_TOO_SMALL: return "BUFFER";
    }
    return "REQ";
}

static char *next_token(char **cursor) {
    char *s = *cursor;
    while (*s && isspace((unsigned char)*s)) s++;
    if (!*s) {
        *cursor = s;
        return NULL;
    }
    char *start = s;
    while (*s && !isspace((unsigned char)*s)) s++;
    if (*s) *s++ = '\0';
    *cursor = s;
    return start;
}

static void handle_get(char *cursor) {
    uint8_t request;
    uint16_t w_value = 0;
    uint16_t w_length = 64;
    uint8_t response[64];
    uint16_t response_len = 0;

    char *req_token = next_token(&cursor);
    char *value_token = next_token(&cursor);
    char *length_token = next_token(&cursor);
    if (!parse_u8(req_token, &request)) {
        uart_err("PARSE");
        return;
    }
    if (value_token && !parse_u16(value_token, &w_value)) {
        uart_err("PARSE");
        return;
    }
    if (length_token && !parse_u16(length_token, &w_length)) {
        uart_err("PARSE");
        return;
    }

    ControlResult result = dspi_control_in(request, w_value, w_length,
                                           response, sizeof(response),
                                           &response_len);
    if (result == CONTROL_RESULT_OK || result == CONTROL_RESULT_ACCEPTED) {
        uart_ok_hex(response, response_len);
    } else {
        uart_err(control_result_code(result));
    }
}

static void handle_set(char *cursor) {
    uint8_t request;
    uint16_t w_value = 0;
    uint16_t payload_len = 0;

    char *req_token = next_token(&cursor);
    char *value_token = next_token(&cursor);
    char *hex_token = next_token(&cursor);
    if (!parse_u8(req_token, &request)) {
        uart_err("PARSE");
        return;
    }
    if (!hex_token) {
        hex_token = value_token;
    } else {
        if (!parse_u16(value_token, &w_value)) {
            uart_err("PARSE");
            return;
        }
    }
    if (!hex_token) hex_token = "";
    if (!decode_hex(hex_token, uart_payload, sizeof(uart_payload), &payload_len)) {
        uart_err("HEX");
        return;
    }

    ControlResult result = dspi_control_out(request, w_value, uart_payload, payload_len);
    if (result == CONTROL_RESULT_OK || result == CONTROL_RESULT_ACCEPTED) {
        uart_ok_text(result == CONTROL_RESULT_ACCEPTED ? "ACCEPTED" : "");
    } else {
        uart_err(control_result_code(result));
    }
}

static void handle_bulk_get(void) {
    uint16_t len = 0;
    ControlResult result = dspi_control_bulk_get(bulk_param_buf, WIRE_BULK_BUF_SIZE, &len);
    if (result != CONTROL_RESULT_OK) {
        uart_err(control_result_code(result));
        return;
    }
    uart_write_str("OK ");
    uart_write_hex(bulk_param_buf, len);
    uart_write_str("\r\n");
}

static void handle_bulk_set(char *cursor) {
    char *hex_token = next_token(&cursor);
    if (!hex_token) {
        uart_err("PARSE");
        return;
    }
    size_t hex_len = strlen(hex_token);
    if (hex_len != sizeof(WireBulkParams) * 2u) {
        uart_err("LEN");
        return;
    }
    uint16_t payload_len = 0;
    if (!decode_hex(hex_token, bulk_param_buf, WIRE_BULK_BUF_SIZE, &payload_len)) {
        uart_err("HEX");
        return;
    }
    ControlResult result = dspi_control_bulk_set(bulk_param_buf, payload_len);
    if (result == CONTROL_RESULT_OK || result == CONTROL_RESULT_ACCEPTED) {
        uart_ok_text("ACCEPTED");
    } else {
        uart_err(control_result_code(result));
    }
}

static void handle_line(char *line) {
    char *cursor = line;
    char *cmd = next_token(&cursor);
    if (!cmd || cmd[0] == '#') return;
    uppercase_token(cmd);

    if (strcmp(cmd, "PING") == 0) {
        uart_ok_text("PONG");
    } else if (strcmp(cmd, "G") == 0 || strcmp(cmd, "GET") == 0) {
        handle_get(cursor);
    } else if (strcmp(cmd, "S") == 0 || strcmp(cmd, "SET") == 0) {
        handle_set(cursor);
    } else if (strcmp(cmd, "BGET") == 0) {
        handle_bulk_get();
    } else if (strcmp(cmd, "BSET") == 0) {
        handle_bulk_set(cursor);
    } else if (strcmp(cmd, "HELP") == 0) {
        uart_ok_text("PING|G req [wValue] [len]|S req [wValue] hex|BGET|BSET hex");
    } else {
        uart_err("CMD");
    }
}

void uart_control_init(void) {
    uart_inst_t *uart = control_uart();
    uart_init(uart, DSPi_UART_CONTROL_BAUD);
    gpio_set_function(DSPi_UART_CONTROL_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DSPi_UART_CONTROL_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(uart, true);

#if DSPi_UART_CONTROL_CTS_PIN >= 0
    gpio_set_function(DSPi_UART_CONTROL_CTS_PIN, GPIO_FUNC_UART);
#endif
#if DSPi_UART_CONTROL_RTS_PIN >= 0
    gpio_set_function(DSPi_UART_CONTROL_RTS_PIN, GPIO_FUNC_UART);
#endif
    uart_set_hw_flow(uart, DSPi_UART_CONTROL_CTS_PIN >= 0, DSPi_UART_CONTROL_RTS_PIN >= 0);

    uart_line_len = 0;
    uart_rx_state = UART_RX_IDLE;
    uart_last_rx_us = time_us_32();
    uart_ok_text("DSPi UART READY");
}

void uart_control_poll(void) {
    uart_inst_t *uart = control_uart();
    uint32_t now = time_us_32();

    if (uart_rx_state == UART_RX_LINE && uart_line_len > 0 &&
        (uint32_t)(now - uart_last_rx_us) > DSPi_UART_CONTROL_TIMEOUT_US) {
        uart_line_len = 0;
        uart_rx_state = UART_RX_IDLE;
        uart_err("TIMEOUT");
    }

    while (uart_is_readable(uart)) {
        char c = (char)uart_getc(uart);
        uart_last_rx_us = time_us_32();

        if (c == '\r' || c == '\n') {
            if (uart_rx_state == UART_RX_OVERFLOW) {
                uart_rx_state = UART_RX_IDLE;
                uart_line_len = 0;
                uart_err("OVERSIZE");
            } else if (uart_line_len > 0) {
                uart_line[uart_line_len] = '\0';
                handle_line(uart_line);
                uart_line_len = 0;
                uart_rx_state = UART_RX_IDLE;
            }
            continue;
        }

        if (uart_rx_state == UART_RX_OVERFLOW) continue;

        uart_rx_state = UART_RX_LINE;
        if (uart_line_len + 1u >= sizeof(uart_line)) {
            uart_rx_state = UART_RX_OVERFLOW;
            uart_line_len = 0;
            continue;
        }
        uart_line[uart_line_len++] = c;
    }
}

#else

void uart_control_init(void) {}
void uart_control_poll(void) {}

#endif
