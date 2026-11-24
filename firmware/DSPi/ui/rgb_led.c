/*
 * rgb_led.c; see rgb_led.h
 */
#include "rgb_led.h"
#include "ui_board.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

static PIO      led_pio = pio1;
static bool     ready;
static uint32_t last = 0xFFFFFFFFu;

void rgb_led_init(void) {
    if (!pio_sm_is_claimed(led_pio, UI_RGB_PIO_SM)) pio_sm_claim(led_pio, UI_RGB_PIO_SM);
    if (!pio_can_add_program(led_pio, &ws2812_program)) return;   // PIO1 instruction memory full: LED stays off
    uint off = pio_add_program(led_pio, &ws2812_program);
    ws2812_program_init(led_pio, UI_RGB_PIO_SM, off, UI_PIN_RGB_DATA, 800000.0f);
    ready = true;
}

void rgb_led_set(uint8_t r, uint8_t g, uint8_t b) {
    if (!ready) return;
    uint32_t px = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;   // WS2812 wants GRB
    if (px == last) return;
    if (pio_sm_is_tx_fifo_full(led_pio, UI_RGB_PIO_SM)) return;     // retry next call
    pio_sm_put(led_pio, UI_RGB_PIO_SM, px << 8);
    last = px;
}
