/*
 * ui_board.h; Seeed XIAO RP2350 pin map for the DSPi front-panel UI
 *
 * Header pin -> GPIO (XIAO RP2350): D0=26 D1=27 D2=28 D3=5 D4=6 D5=7 D6=0
 * D7=1 D8=2 D9=4 D10=3.  On-board: yellow user LED GPIO25 (active low),
 * WS2812 RGB LED GPIO22 with its power enable on GPIO23 (core0_init already
 * drives GPIO23 high), BOOT button on the QSPI_SS line.
 *
 * Audio (set as DSPi's defaults in config.h for this board):
 *   PCM5102 BCK  = GPIO26 (D0)    LRCK = GPIO27 (D1, always BCK+1)
 *   PCM5102 DIN  = GPIO28 (D2)
 * Display (ST7735 1.8" 160x128 TFT, SPI0; BL wired on, not driven):
 *   CS = GPIO1 (D7)  SCK = GPIO2 (D8)  DC = GPIO4 (D9)  MOSI = GPIO3 (D10)
 *   RES = GPIO5 (D3)
 * Left free: D4 (GPIO6) and D5 (GPIO7) are the idle S/PDIF slot pins DSPi
 * brings up at boot, so nothing should be wired there; D6 (GPIO0) is free and
 * is the optional external button pin.
 */
#ifndef UI_BOARD_H
#define UI_BOARD_H

#include <stdint.h>

#define UI_FACTORY_I2S_DATA_PIN 28   // PCM5102 DIN (D2); slot 0 factory default in this build

#define UI_DISP_W 160      // landscape
#define UI_DISP_H 128

#define UI_DISP_SPI_INST    spi0
#define UI_PIN_DISP_CS      1
#define UI_PIN_DISP_SCK     2
#define UI_PIN_DISP_MOSI    3
#define UI_PIN_DISP_DC      4
#define UI_PIN_DISP_RST     5
#ifndef UI_DISP_SPI_HZ
#define UI_DISP_SPI_HZ      26000000u  // highest clk_peri divide at or below this; verified on this panel. 13 MHz for long wiring
#endif
// MADCTL: 0xA0 = MY|MV, landscape, RGB colour order: verified on this
// module (with the BGR bit, 0x08, orange and red showed as blue). 0x60
// (MX|MV) rotates 180 degrees; add 0x08 for BGR-order modules.
#ifndef UI_DISP_MADCTL
#define UI_DISP_MADCTL      0xA0
#endif
// Green-tab modules: 132x162 controller RAM with the glass at (1, 2) in
// landscape; without this, noise shows along the bottom and right edges.
// Set both to 0 for red/black-tab modules.
#ifndef UI_DISP_X_OFS
#define UI_DISP_X_OFS       1
#endif
#ifndef UI_DISP_Y_OFS
#define UI_DISP_Y_OFS       2
#endif
#define UI_DISP_DMA_CH      9          // spare on RP2350 (DSPi map: outputs 0-3, PDM 4, RX 5-8, spare 9-10)

#define UI_PIN_USER_LED     25         // yellow, active low
#define UI_PIN_RGB_DATA     22         // WS2812
#define UI_RGB_PIO_SM       3          // pio1 SM3: DSPi uses SM0 PDM, SM1 ADAT TX, SM2 ADAT RX

// Button source: the BOOT button (default) or an external switch to GND.
#define UI_BTN_SRC_BOOTSEL  1
#define UI_BTN_SRC_GPIO     2
#ifndef UI_BTN_SOURCE
#define UI_BTN_SOURCE       UI_BTN_SRC_GPIO
#endif
#define UI_PIN_BTN_GPIO     0          // D6, active low with internal pull-up

// GPIOs the UI owns; DSPi's pin validator rejects these for its own functions.
#if UI_BTN_SOURCE == UI_BTN_SRC_GPIO
#define UI_OWNED_PIN_MASK ((1u<<1)|(1u<<2)|(1u<<3)|(1u<<4)|(1u<<5)|(1u<<22)|(1u<<0))
#else
#define UI_OWNED_PIN_MASK ((1u<<1)|(1u<<2)|(1u<<3)|(1u<<4)|(1u<<5)|(1u<<22))
#endif

#endif
