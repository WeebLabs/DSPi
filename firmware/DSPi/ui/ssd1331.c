/*
 * ssd1331.c; see ssd1331.h
 */
#include "ssd1331.h"
#include "ui_board.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

enum { S_OFF, S_RST_LOW, S_RST_HIGH, S_INIT, S_BLACK, S_ON_WAIT, S_READY };

static uint8_t  state = S_OFF;
static uint32_t t_mark;
static bool     flushing;
static uint8_t  level = 2;

// Master current (0x87) per level; contrast cmds stay at their init values.
static const uint8_t level_cur[4] = { 0x01, 0x03, 0x06, 0x0F };

static inline void cs(bool active) { gpio_put(UI_PIN_DISP_CS, !active); }

static void cmd(const uint8_t *b, int n) {
    gpio_put(UI_PIN_DISP_DC, 0);
    cs(true);
    spi_write_blocking(UI_DISP_SPI_INST, b, n);
    cs(false);
}
static void cmd1(uint8_t c)                         { cmd(&c, 1); }
static void cmd2(uint8_t c, uint8_t a)              { uint8_t b[2] = { c, a }; cmd(b, 2); }

static bool wait_us(uint32_t us) { return (int32_t)(time_us_32() - t_mark) >= (int32_t)us; }

static void send_init(void) {
    static const uint8_t seq[] = {
        0xAE,                    // display off
        0xA0, UI_DISP_REMAP,     // remap / colour depth
        0xA1, 0x00,              // start line
        0xA2, 0x00,              // display offset
        0xA4,                    // normal display
        0xA8, 0x3F,              // multiplex 64
        0xAD, 0x8E,              // master config: external VCC
        0xB0, 0x0B,              // power save off
        0xB1, 0x31,              // phase 1 / 2 periods
        0xB3, 0xF0,              // clock divider / oscillator
        0x8A, 0x64, 0x8B, 0x78, 0x8C, 0x64,   // second pre-charge A/B/C
        0xBB, 0x3A,              // pre-charge level
        0xBE, 0x3E,              // VCOMH
        0x87, 0x06,              // master current
        0x81, 0x91, 0x82, 0x50, 0x83, 0x7D,   // contrast A/B/C
    };
    cmd(seq, sizeof seq);
}

static void window(void) {
    static const uint8_t w[] = { 0x15, 0, UI_DISP_W - 1, 0x75, 0, UI_DISP_H - 1 };
    cmd(w, sizeof w);
}

static void dma_start(const uint16_t *fb) {
    dma_channel_config c = dma_channel_get_default_config(UI_DISP_DMA_CH);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(UI_DISP_SPI_INST, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    gpio_put(UI_PIN_DISP_DC, 1);
    cs(true);
    dma_channel_configure(UI_DISP_DMA_CH, &c, &spi_get_hw(UI_DISP_SPI_INST)->dr, fb,
                          UI_DISP_W * UI_DISP_H * 2, true);
    flushing = true;
}

void ssd1331_begin(void) {
    gpio_init(UI_PIN_DISP_CS);  gpio_set_dir(UI_PIN_DISP_CS, GPIO_OUT);  gpio_put(UI_PIN_DISP_CS, 1);
    gpio_init(UI_PIN_DISP_DC);  gpio_set_dir(UI_PIN_DISP_DC, GPIO_OUT);  gpio_put(UI_PIN_DISP_DC, 0);
    gpio_init(UI_PIN_DISP_RST); gpio_set_dir(UI_PIN_DISP_RST, GPIO_OUT); gpio_put(UI_PIN_DISP_RST, 0);
    spi_init(UI_DISP_SPI_INST, UI_DISP_SPI_HZ);
    spi_set_format(UI_DISP_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(UI_PIN_DISP_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(UI_PIN_DISP_MOSI, GPIO_FUNC_SPI);
    dma_channel_claim(UI_DISP_DMA_CH);
    t_mark = time_us_32();
    state = S_RST_LOW;
}

bool ssd1331_ready(void) { return state == S_READY; }

bool ssd1331_busy(void) { return flushing; }

static void flush_poll(void) {
    if (!flushing) return;
    if (dma_channel_is_busy(UI_DISP_DMA_CH)) return;
    if (spi_get_hw(UI_DISP_SPI_INST)->sr & SPI_SSPSR_BSY_BITS) return;   // last byte still shifting
    cs(false);
    flushing = false;
}

bool ssd1331_service(void) {
    flush_poll();
    switch (state) {
    case S_RST_LOW:
        if (wait_us(5000)) { gpio_put(UI_PIN_DISP_RST, 1); t_mark = time_us_32(); state = S_RST_HIGH; }
        break;
    case S_RST_HIGH:
        if (wait_us(20000)) { state = S_INIT; }
        break;
    case S_INIT:
        send_init();
        {   // clear GDDRAM with the controller's clear-window command (0x25), no DMA needed
            static const uint8_t clr[] = { 0x25, 0, 0, UI_DISP_W - 1, UI_DISP_H - 1 };
            cmd(clr, sizeof clr);
        }
        t_mark = time_us_32();
        state = S_BLACK;
        break;
    case S_BLACK:
        if (wait_us(2000)) { cmd1(0xAF); t_mark = time_us_32(); state = S_ON_WAIT; }
        break;
    case S_ON_WAIT:
        if (wait_us(110000)) { ssd1331_set_level(level); state = S_READY; }
        break;
    default: break;
    }
    return state == S_READY;
}

bool ssd1331_flush(const uint16_t *fb) {
    if (state != S_READY || flushing) return false;
    window();
    dma_start(fb);
    return true;
}

void ssd1331_set_level(uint8_t l) {
    if (l > 3) l = 3;
    level = l;
    if (state >= S_ON_WAIT) cmd2(0x87, level_cur[l]);
}

void ssd1331_power(bool on) {
    if (state == S_READY && !flushing) cmd1(on ? 0xAF : 0xAE);
}
