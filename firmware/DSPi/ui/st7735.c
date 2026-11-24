/*
 * st7735.c; see st7735.h
 *
 * Init is the widely used ST7735R/S "red tab" sequence. The panel options
 * that differ between modules are in ui_board.h: orientation/colour order
 * (UI_DISP_MADCTL) and the RAM offset of "green tab" modules, whose 132x162
 * controller RAM puts the glass at column 1, row 2 in landscape
 * (UI_DISP_X_OFS / UI_DISP_Y_OFS).
 */
#include "st7735.h"
#include "ui_board.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

enum { S_OFF, S_RST_LOW, S_RST_HIGH, S_INIT, S_BLACK, S_BLACK_WAIT, S_READY };

static uint8_t  state = S_OFF;
static uint32_t t_mark;
static uint32_t wait_for;
static uint16_t init_pos;
static bool     flushing;
static bool     blank_pending;
static const uint16_t s_zero = 0;     // DMA source for black frames (no read increment)

static inline void cs(bool active) { gpio_put(UI_PIN_DISP_CS, !active); }

static void spi_idle(void) {
    while (spi_is_busy(UI_DISP_SPI_INST)) { }
    while (spi_is_readable(UI_DISP_SPI_INST)) (void)spi_get_hw(UI_DISP_SPI_INST)->dr;
}

// One command and its argument bytes; at most 17 bytes, a few microseconds.
static void cmd(uint8_t c, const uint8_t *args, int n) {
    cs(true);
    gpio_put(UI_PIN_DISP_DC, 0);
    spi_write_blocking(UI_DISP_SPI_INST, &c, 1);
    if (n) {
        gpio_put(UI_PIN_DISP_DC, 1);
        spi_write_blocking(UI_DISP_SPI_INST, args, (size_t)n);
    }
    cs(false);
}

static bool waited(void) { return (int32_t)(time_us_32() - t_mark) >= (int32_t)wait_for; }
static void wait_us(uint32_t us) { t_mark = time_us_32(); wait_for = us; }

// cmd, n | DELAY, args..., [delay in ms when flagged]
#define DELAY 0x80
#define END   0xFF
static const uint8_t init_seq[] = {
    0x01, DELAY, 150,                            // SWRESET
    0x11, DELAY, 150,                            // SLPOUT
    0xB1, 3, 0x01, 0x2C, 0x2D,                   // FRMCTR1
    0xB2, 3, 0x01, 0x2C, 0x2D,                   // FRMCTR2
    0xB3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D, // FRMCTR3
    0xB4, 1, 0x07,                               // INVCTR: no inversion
    0xC0, 3, 0xA2, 0x02, 0x84,                   // PWCTR1
    0xC1, 1, 0xC5,                               // PWCTR2
    0xC2, 2, 0x0A, 0x00,                         // PWCTR3
    0xC3, 2, 0x8A, 0x2A,                         // PWCTR4
    0xC4, 2, 0x8A, 0xEE,                         // PWCTR5
    0xC5, 1, 0x0E,                               // VMCTR1
    0x20, 0,                                     // INVOFF
    0x36, 1, UI_DISP_MADCTL,                     // MADCTL: orientation, colour order
    0x3A, 1, 0x05,                               // COLMOD: RGB565
    0xE0, 16, 0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
              0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,   // GMCTRP1
    0xE1, 16, 0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
              0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,   // GMCTRN1
    0x13, DELAY, 10,                             // NORON
    END,
};

static void window(void) {
    const uint8_t x0 = UI_DISP_X_OFS, y0 = UI_DISP_Y_OFS;
    const uint8_t ca[4] = { 0, x0, 0, (uint8_t)(x0 + UI_DISP_W - 1) };
    const uint8_t ra[4] = { 0, y0, 0, (uint8_t)(y0 + UI_DISP_H - 1) };
    cmd(0x2A, ca, 4);    // CASET
    cmd(0x2B, ra, 4);    // RASET
}

// Stream a full frame: from fb, or black when fb is NULL.
static void dma_start(const uint16_t *fb) {
    window();
    cs(true);
    gpio_put(UI_PIN_DISP_DC, 0);
    const uint8_t ramwr = 0x2C;
    spi_write_blocking(UI_DISP_SPI_INST, &ramwr, 1);
    spi_idle();
    gpio_put(UI_PIN_DISP_DC, 1);

    dma_channel_config c = dma_channel_get_default_config(UI_DISP_DMA_CH);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(UI_DISP_SPI_INST, true));
    channel_config_set_read_increment(&c, fb != NULL);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(UI_DISP_DMA_CH, &c, &spi_get_hw(UI_DISP_SPI_INST)->dr,
                          fb ? (const void *)fb : (const void *)&s_zero,
                          UI_DISP_W * UI_DISP_H * 2, true);
    flushing = true;
}

void st7735_begin(void) {
    gpio_init(UI_PIN_DISP_CS);  gpio_set_dir(UI_PIN_DISP_CS, GPIO_OUT);  gpio_put(UI_PIN_DISP_CS, 1);
    gpio_init(UI_PIN_DISP_DC);  gpio_set_dir(UI_PIN_DISP_DC, GPIO_OUT);  gpio_put(UI_PIN_DISP_DC, 1);
    gpio_init(UI_PIN_DISP_RST); gpio_set_dir(UI_PIN_DISP_RST, GPIO_OUT); gpio_put(UI_PIN_DISP_RST, 0);
    spi_init(UI_DISP_SPI_INST, UI_DISP_SPI_HZ);
    spi_set_format(UI_DISP_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(UI_PIN_DISP_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(UI_PIN_DISP_MOSI, GPIO_FUNC_SPI);
    dma_channel_claim(UI_DISP_DMA_CH);
    wait_us(10000);           // reset low for 10 ms
    state = S_RST_LOW;
}

bool st7735_ready(void) { return state == S_READY; }

bool st7735_busy(void) { return flushing || blank_pending; }

static void flush_poll(void) {
    if (!flushing) return;
    if (dma_channel_is_busy(UI_DISP_DMA_CH)) return;
    if (spi_is_busy(UI_DISP_SPI_INST)) return;   // last byte still shifting
    cs(false);
    while (spi_is_readable(UI_DISP_SPI_INST)) (void)spi_get_hw(UI_DISP_SPI_INST)->dr;
    spi_get_hw(UI_DISP_SPI_INST)->icr = SPI_SSPICR_RORIC_BITS;
    flushing = false;
}

bool st7735_service(void) {
    flush_poll();
    switch (state) {
    case S_RST_LOW:
        if (waited()) { gpio_put(UI_PIN_DISP_RST, 1); wait_us(120000); state = S_RST_HIGH; }
        break;
    case S_RST_HIGH:
        if (waited()) { init_pos = 0; wait_for = 0; state = S_INIT; }
        break;
    case S_INIT:
        // One command per call, so each main-loop pass stays short.
        if (!waited()) break;
        if (init_seq[init_pos] == END) { state = S_BLACK; break; }
        {
            uint8_t c = init_seq[init_pos];
            uint8_t hdr = init_seq[init_pos + 1];
            uint8_t n = hdr & 0x7F;
            cmd(c, &init_seq[init_pos + 2], n);
            init_pos = (uint16_t)(init_pos + 2 + n);
            if (hdr & DELAY) wait_us((uint32_t)init_seq[init_pos++] * 1000u);
            else             wait_us(0);
        }
        break;
    case S_BLACK:
        // The panel RAM powers up as noise: wipe it before the display goes on.
        if (!flushing) { dma_start(NULL); state = S_BLACK_WAIT; }
        break;
    case S_BLACK_WAIT:
        if (!flushing) { cmd(0x29, NULL, 0); state = S_READY; }   // DISPON
        break;
    case S_READY:
        if (blank_pending && !flushing) { blank_pending = false; dma_start(NULL); }
        break;
    default: break;
    }
    return state == S_READY;
}

bool st7735_flush(const uint16_t *fb) {
    if (state != S_READY || flushing || blank_pending) return false;
    dma_start(fb);
    return true;
}

void st7735_power(bool on) {
    if (!on) blank_pending = true;
}
