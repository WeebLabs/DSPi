/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "platform_defs_c.h"
#include "tft_driver.h"
#include "gpio.h"
#include "pio.h"
#include "control.pio.h"
#include "dma.h"
#include "debug.h"

CU_REGISTER_DEBUG_PINS(cmds)
//CU_SELECT_DEBUG_PINS(cmds)

#define MAKE_CMD(x, jmp) ((x) | ((jmp)<<11u))

//#define NO_SETUP

static struct __packed {
#ifndef NO_SETUP
    uint16_t exec_csrs_l;       //
    uint16_t wr_1_cmd;          //
    uint16_t caset16;           //

    uint16_t exec_rs_h;
    uint16_t wr_4_cmd;          //
    uint16_t x0_h;
    uint16_t x0_l;
    uint16_t x1_h;
    uint16_t x1_l;

    uint16_t exec_csrs_l_2;     //
    uint16_t wr_1_cmd_2;        //
    uint16_t paset16;           //

    uint16_t exec_rs_h_2;       //
    uint16_t wr_4_cmd_2;        //
    uint16_t y0_h;
    uint16_t y0_l;
    uint16_t y1_h;
    uint16_t y1_l;

    uint16_t exec_csrs_l_3;     //
    uint16_t wr_1_cmd_3;        //
    uint16_t ramwr16;           //

    uint16_t exec_rs_h_3;       //
#else
    uint16_t exec_rs_h;
#endif
    uint16_t skip_cmd;          //
    uint16_t exec_set_irq4;     //
    uint16_t clk_n_cmd;         //

    uint16_t exec_csrs_h;       //
    uint16_t skip_cmd_2;        //
    uint16_t exec_set_irq0;       //
    uint16_t skip_cmd_3;        //

#ifndef NO_SETUP
    // to align on word boundary
    uint16_t exec_skip;         //
#endif
} __aligned(4) scanline_control_sequence;

static struct __packed {
    uint16_t exec_csrs_l;       //
    uint16_t wr_1_cmd;          //

//    uint16_t ptlar16;           //
//    uint16_t exec_rs_h;
//
//    uint16_t wr_4_cmd;          //
//    uint16_t y0_h;
//
//    uint16_t y0_l;
//    uint16_t y1_h;
//
//    uint16_t y1_l;
//    uint16_t exec_csrs_l_2;       //
//
//    uint16_t wr_1_cmd_2;          //
    uint16_t vscrsaddr16;           //

    uint16_t exec_rs_h_2;
    uint16_t wr_2_cmd;          //

    uint16_t y_h;
    uint16_t y_l;

    uint16_t exec_csrs_h;
    uint16_t skip_cmd;

    uint16_t exec_skip;
} __aligned(4) switch_buffer_control_sequence;

//These define the ports and port bits used for the write, chip select (CS) and data/command (RS) lines
#define WR_L gpio_put(WR_PIN, false)

// We need a slower write strobe for the ILI9488
#ifdef ILI9486
#define ILI9481
  #define WR_H ({gpio_put(WR_PIN, false); gpio_put(WR_PIN, true);})
  #define WR_STB ({gpio_put(WR_PIN, false); gpio_put(WR_PIN, false); gpio_put(WR_PIN, true);})
#else
#define WR_H gpio_put(WR_PIN, true);
#define WR_STB gpio_put(WR_PIN, false); gpio_put(WR_PIN, true);
#endif

// Chip select must be toggled during setup
#define SETUP_CS_H gpio_put(CS_PIN, true)
#define SETUP_CS_L gpio_put(CS_PIN, false)

// Chip select can optionally be kept low after setup
#ifndef KEEP_CS_LOW
#define CS_H gpio_put(CS_PIN, true)
#define CS_L gpio_put(CS_PIN, false)
#else
#define CS_H // We do not define this so CS will not be set high
  #define CS_L gpio_put(CS_PIN, false)
#endif

// If pin 4 is hard wired to pin 38 we benefit from all controls on PORTG
#ifdef FAST_RS
#define RS_L gpio_put(RS_PIN, false);
  #define RS_H gpio_put(RS_PIN, true);
#else
#define RS_L gpio_put(RS_PIN, false)
#define RS_H gpio_put(RS_PIN, true)
#endif


static inline void digitalWrite(uint pin, bool value) {
    gpio_put(pin, value);
}

static inline void set_data_pins16(uint16_t data) {
    gpio_put_masked(0xffff, data);
}

static inline void set_data_pins8(uint8_t data) {
    gpio_put_masked(0xff, data);
}

void writecommand(uint8_t c)
{
    DEBUG_PINS_SET(cmds, 1);
    SETUP_CS_L;
    RS_L;
    set_data_pins16(c);
    WR_STB;
    RS_H;
    SETUP_CS_H;
    DEBUG_PINS_CLR(cmds, 1);
}


/***************************************************************************************
** Function name:           writedata
** Description:             Send a 8 bit data value to the TFT
***************************************************************************************/
void writedata(uint8_t c)
{
    SETUP_CS_L;
    set_data_pins16(c);
    WR_STB;
    SETUP_CS_H;
}

#ifdef NO_SETUP
void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    //if((x1 >= _width) || (y1 >= _height)) return;
    CS_L;
    RS_L;  set_data_pins8(HX8357_CASET); WR_STB; RS_H;
    set_data_pins8(x0>>8); WR_STB;
    set_data_pins8(x0); WR_STB;
    set_data_pins8(x1>>8); WR_STB;
    set_data_pins8(x1); WR_STB;
    RS_L; set_data_pins8(HX8357_PASET); WR_STB; RS_H;
    set_data_pins8(y0>>8); WR_STB;
    set_data_pins8(y0); WR_STB;
    set_data_pins8(y1>>8); WR_STB;
    set_data_pins8(y1); WR_STB;
    RS_L; set_data_pins8(HX8357_RAMWR); WR_STB; RS_H;
}
#endif


void tft_panel_init() {
    // toggle RST low to reset
    gpio_put(RST_PIN, true);
    sleep_ms(50);
    gpio_put(RST_PIN, false);
    sleep_ms(10);
    gpio_put(RST_PIN, true);
    sleep_ms(10);

#ifndef HX8357C

#ifdef ILI9486
    writecommand(0x01);
    writedata(0x00);
    sleep_ms(50);

    writecommand(0x28);
    writedata(0x00);

    writecommand(0xC0);        // Power Control 1
    writedata(0x0d);
    writedata(0x0d);

    writecommand(0xC1);        // Power Control 2
    writedata(0x43);
    writedata(0x00);

    writecommand(0xC2);        // Power Control 3
    writedata(0x00);

    writecommand(0xC5);        // VCOM Control
    writedata(0x00);
    writedata(0x48);

    writecommand(0xB6);        // Display Function Control
    writedata(0x00);
    writedata(0x22);           // 0x42 = Rotate display 180 deg.
    writedata(0x3B);

    writecommand(0xE0);        // PGAMCTRL (Positive Gamma Control)
    writedata(0x0f);
    writedata(0x24);
    writedata(0x1c);
    writedata(0x0a);
    writedata(0x0f);
    writedata(0x08);
    writedata(0x43);
    writedata(0x88);
    writedata(0x32);
    writedata(0x0f);
    writedata(0x10);
    writedata(0x06);
    writedata(0x0f);
    writedata(0x07);
    writedata(0x00);

    writecommand(0xE1);        // NGAMCTRL (Negative Gamma Control)
    writedata(0x0F);
    writedata(0x38);
    writedata(0x30);
    writedata(0x09);
    writedata(0x0f);
    writedata(0x0f);
    writedata(0x4e);
    writedata(0x77);
    writedata(0x3c);
    writedata(0x07);
    writedata(0x10);
    writedata(0x05);
    writedata(0x23);
    writedata(0x1b);
    writedata(0x00); 

    writecommand(0x20);        // Display Inversion OFF, 0x21 = ON

    writecommand(0x36);        // Memory Access Control
//    writedata(0x0A);
    writedata(0x02); // rgb -> bgr

    writecommand(0x3A);        // Interface Pixel Format
    writedata(0x55);

//    // scroll
//    writecommand( 0x37);
//    writedata(0);
//    writedata(120);
//
    writecommand( 0x30);
    writedata(0);
    writedata(0);
    writedata(0);
    writedata(240);

    // partial mode
    writecommand( 0x12);

    writecommand(0x11);

    sleep_ms(150);

    writecommand(0x29);
    sleep_ms(25);

#else
// Configure HX8357-B display
    writecommand(0x11);
    sleep_ms(20);
    writecommand(0xD0);
    writedata(0x07);
    writedata(0x42);
    writedata(0x18);

    writecommand(0xD1);
    writedata(0x00);
    writedata(0x07);
    writedata(0x10);

    writecommand(0xD2);
    writedata(0x01);
    writedata(0x02);

    writecommand(0xC0);
    writedata(0x10);
    writedata(0x3B);
    writedata(0x00);
    writedata(0x02);
    writedata(0x11);

    writecommand(0xC5);
    writedata(0x08);

    writecommand(0xC8);
    writedata(0x00);
    writedata(0x32);
    writedata(0x36);
    writedata(0x45);
    writedata(0x06);
    writedata(0x16);
    writedata(0x37);
    writedata(0x75);
    writedata(0x77);
    writedata(0x54);
    writedata(0x0C);
    writedata(0x00);

    writecommand(0x36);
    //writedata(0x0a);
    writedata(0x02); // rgb -> bgr

    writecommand(0x3A);
    writedata(0x55);

    writecommand(0x2A);
    writedata(0x00);
    writedata(0x00);
    writedata(0x01);
    writedata(0x3F);

    writecommand(0x2B);
    writedata(0x00);
    writedata(0x00);
    writedata(0x01);
    writedata(0xDF);

    sleep_ms(120);
    writecommand(0x29);

    sleep_ms(25);
// End of HX8357-B display configuration
#endif

#else

    // HX8357-C display initialisation

    writecommand(0xB9); // Enable extension command
    writedata(0xFF);
    writedata(0x83);
    writedata(0x57);
    sleep_ms(50);
    
    writecommand(0xB6); //Set VCOM voltage
    writedata(0x2C);    //0x52 for HSD 3.0"
    
    writecommand(0x11); // Sleep off
    sleep_ms(200);
    
    writecommand(0x35); // Tearing effect on
    writedata(0x00);    // Added parameter

    writecommand(0x3A); // Interface pixel format
    writedata(0x55);    // 16 bits per pixel

    //writecommand(0xCC); // Set panel characteristic
    //writedata(0x09);    // S960>S1, G1>G480, R-G-B, normally black

    //writecommand(0xB3); // RGB interface
    //writedata(0x43);
    //writedata(0x00);
    //writedata(0x06);
    //writedata(0x06);

    writecommand(0xB1); // Power control
    writedata(0x00);
    writedata(0x15);
    writedata(0x0D);
    writedata(0x0D);
    writedata(0x83);
    writedata(0x48);
    
    
    writecommand(0xC0); // Does this do anything?
    writedata(0x24);
    writedata(0x24);
    writedata(0x01);
    writedata(0x3C);
    writedata(0xC8);
    writedata(0x08);
    
    writecommand(0xB4); // Display cycle
    writedata(0x02);
    writedata(0x40);
    writedata(0x00);
    writedata(0x2A);
    writedata(0x2A);
    writedata(0x0D);
    writedata(0x4F);
    
    writecommand(0xE0); // Gamma curve
    writedata(0x00);
    writedata(0x15);
    writedata(0x1D);
    writedata(0x2A);
    writedata(0x31);
    writedata(0x42);
    writedata(0x4C);
    writedata(0x53);
    writedata(0x45);
    writedata(0x40);
    writedata(0x3B);
    writedata(0x32);
    writedata(0x2E);
    writedata(0x28);
    
    writedata(0x24);
    writedata(0x03);
    writedata(0x00);
    writedata(0x15);
    writedata(0x1D);
    writedata(0x2A);
    writedata(0x31);
    writedata(0x42);
    writedata(0x4C);
    writedata(0x53);
    writedata(0x45);
    writedata(0x40);
    writedata(0x3B);
    writedata(0x32);
    
    writedata(0x2E);
    writedata(0x28);
    writedata(0x24);
    writedata(0x03);
    writedata(0x00);
    writedata(0x01);

    writecommand(0x36); // MADCTL Memory access control
    writedata(0x48);
    sleep_ms(20);

    writecommand(0x21); //Display inversion on
    sleep_ms(20);

    writecommand(0x29); // Display on
    
    sleep_ms(120);
#endif

#ifdef NO_SETUP
    setAddrWindow(0, 0, 320, 240);
#endif

#ifdef KEEP_CS_LOW
    SETUP_CS_L;
#endif
}

void tft_driver_init()
{
    for(int i = 0; i < 16; i++)
    {
        gpio_init(i);
    }
    gpio_init(RS_PIN);
    gpio_init(CS_PIN);
//    gpio_init(_fcs);
    gpio_init(WR_PIN);
    gpio_init(RST_PIN);

    gpio_set_dir(RST_PIN, true);
    gpio_put(RST_PIN, true);

    gpio_set_dir(RS_PIN, true);
    gpio_set_dir(CS_PIN, true);
    gpio_set_dir(WR_PIN, true);

    gpio_put(RS_PIN, true);

#ifndef KEEP_CS_LOW
    gpio_put(CS_PIN, true);
#else
    gpio_put(CS_PIN, false);
#endif
    
    gpio_put(WR_PIN, true);

    //  DDRA = 0xFF; // Set direction for the 2 8 bit data ports
    //  DDRC = 0xFF;
    gpio_set_dir_masked(0xffff, 0xffff);

#define EXEC_CSL_RSL pio_encode_with_sideset_opt(pio_encode_set_pins(0), 2, 1)
#define EXEC_CSL_RSH pio_encode_with_sideset_opt(pio_encode_set_pins(1), 2, 1)
#define EXEC_CSH_RSH pio_encode_with_sideset_opt(pio_encode_set_pins(3), 2, 1)
#define EXEC_SET_IRQ(n) pio_encode_with_sideset_opt(pio_encode_irq_set((n), false), 2, 1)
#define DOH_PROGRAM_OFFSET 16
#define EXEC_SKIP pio_encode_with_sideset_opt(pio_encode_jmp(DOH_PROGRAM_OFFSET + video_dbi_control_offset_new_state_wait), 2, 1)

#define WR_CMD(n) MAKE_CMD( (n)-1, DOH_PROGRAM_OFFSET + video_dbi_control_offset_data_run_out)
#define SKIP_CMD MAKE_CMD( 0, DOH_PROGRAM_OFFSET + video_dbi_control_offset_new_state_wait)
#define CLOCK_CMD(n) MAKE_CMD((w)-1, DOH_PROGRAM_OFFSET + video_dbi_control_offset_clock_run)
#ifndef NO_SETUP
    scanline_control_sequence.exec_csrs_l = EXEC_CSL_RSL;
    scanline_control_sequence.wr_1_cmd = WR_CMD(1);
    scanline_control_sequence.caset16 = HX8357_CASET;
    scanline_control_sequence.wr_4_cmd = WR_CMD(4);
    scanline_control_sequence.exec_csrs_l_2 = EXEC_CSL_RSL;
    scanline_control_sequence.wr_1_cmd_2 = WR_CMD(1);
    scanline_control_sequence.paset16 = HX8357_PASET;
    scanline_control_sequence.exec_rs_h_2 = EXEC_CSL_RSH;
    scanline_control_sequence.wr_4_cmd_2 = WR_CMD(4);
    scanline_control_sequence.exec_csrs_l_3 = EXEC_CSL_RSL;
    scanline_control_sequence.wr_1_cmd_3 = WR_CMD(1);
    scanline_control_sequence.ramwr16 = HX8357_RAMWR;
    scanline_control_sequence.exec_rs_h_3 = EXEC_CSL_RSH;
#endif
    scanline_control_sequence.skip_cmd = SKIP_CMD;
    scanline_control_sequence.exec_rs_h = EXEC_CSL_RSH;
    scanline_control_sequence.exec_set_irq4 = EXEC_SET_IRQ(4);
    scanline_control_sequence.exec_csrs_h = EXEC_CSH_RSH;
    scanline_control_sequence.exec_set_irq0 = EXEC_SET_IRQ(0);
    scanline_control_sequence.skip_cmd_2 = SKIP_CMD;
    scanline_control_sequence.skip_cmd_3 = SKIP_CMD;
    scanline_control_sequence.exec_skip = EXEC_SKIP;
    scanline_control_sequence.x0_h = 0;
    scanline_control_sequence.x0_l = 0;

    switch_buffer_control_sequence.exec_csrs_l = EXEC_CSL_RSL;       //
    switch_buffer_control_sequence.wr_1_cmd = WR_CMD(1);          //
//    switch_buffer_control_sequence.ptlar16 = 0x30;           //
//    switch_buffer_control_sequence.exec_rs_h = EXEC_CSL_RSH;
//    switch_buffer_control_sequence.wr_4_cmd = WR_CMD(4);          //
//
//    switch_buffer_control_sequence.exec_csrs_l_2 = EXEC_CSL_RSL;       //
//    switch_buffer_control_sequence.wr_1_cmd_2 = WR_CMD(1);          //
    switch_buffer_control_sequence.vscrsaddr16 = 0x37;           //
    switch_buffer_control_sequence.exec_rs_h_2 = EXEC_CSL_RSH;
    switch_buffer_control_sequence.wr_2_cmd = WR_CMD(2);          //

    switch_buffer_control_sequence.exec_csrs_h = EXEC_CSH_RSH;
    switch_buffer_control_sequence.skip_cmd = SKIP_CMD;
    switch_buffer_control_sequence.exec_skip = EXEC_SKIP;
    tft_panel_init();
}

#define video_pio pio0

// todo duplicate defines from elsewhere
#define SM_DATA 0
#define SM_CONTROL 3
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL 0

uint32_t *get_control_sequence(uint w, uint y, uint *count, bool buffer) {
   y += buffer ? 240 : 0;
#ifndef NO_SETUP
    scanline_control_sequence.x1_h = w>>8;
    scanline_control_sequence.x1_l = w&0xff;
    scanline_control_sequence.y0_h = y>>8;
    scanline_control_sequence.y0_l = y&0xff;
    scanline_control_sequence.y1_h = (y+1)>>8;
    scanline_control_sequence.y1_l = (y+1)&0xff;
#endif
    scanline_control_sequence.clk_n_cmd = CLOCK_CMD(w);

    static_assert(!(sizeof(scanline_control_sequence) & 3u), "");
    *count = sizeof(scanline_control_sequence) / 4;
    return (uint32_t *)&scanline_control_sequence;
}

extern uint32_t *get_switch_buffer_sequence(uint *count, bool buffer) {
//    switch_buffer_control_sequence.y0_h = 0;
//    switch_buffer_control_sequence.y0_l = buffer?240:0;
//    switch_buffer_control_sequence.y1_h = buffer?(480>>8):0;
//    switch_buffer_control_sequence.y1_l = buffer?(480&0xff):240;
    switch_buffer_control_sequence.y_h = 0;
    switch_buffer_control_sequence.y_l = buffer?240:0;
    *count = sizeof(switch_buffer_control_sequence) / 4;
    return (uint32_t *)&switch_buffer_control_sequence;
}