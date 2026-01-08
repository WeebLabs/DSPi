/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// TODO NOTE THIS CODE IS A HACKED TOGETHER PROTOTYPE ATM
//  there is dead code, ugliness and zero error handling... it is very much in a prove it can work state (which it does)

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sd_card.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "sd_card.pio.h"
#include "crc7.h"
#include "crc-itu-t.h"
#include "pico/binary_info.h"

#define SD_CLK_SM 0u
#define SD_CMD_SM 1u
#define SD_DAT_SM 2u
// pins are hard-coded

// todo this is very much a WIP - lots of hacked together stuff and test code that needs to be teased into an actual sensible library with error handling

// todo note there is a lot of crud in here right now

#if 0
#define sd_debug(format, args...) printf(format, ## args)
#else
#define sd_debug(format,args...) (void)0
#endif

#define CMD(n) ((n)+0x40)

static inline uint32_t sd_pio_cmd(uint cmd, uint32_t param)
{
    assert(cmd <= sd_cmd_or_dat_program.length);
    assert(param <= 0xffff);
    return (pio_encode_jmp(cmd) << 16u) | param;
}

#define SD_PIO_CMD(a, b)
PIO sd_pio = pio1;

static uint sd_dat_pin_base; // todo remove me
// todo struct these
static uint8_t rca_high, rca_low;
static enum bus_width {bw_unknown, bw_narrow, bw_wide} bus_width;

const int sd_cmd_dma_channel = 11;
const int sd_data_dma_channel = 10;
const int sd_chain_dma_channel = 9;
const int sd_pio_dma_channel = 8;

static bool allow_four_data_pins;
static bool bytes_swap_on_read = false;

struct message {
    int len;
    uint8_t msg[8];
};

static inline void check_pio_debug(const char *s) {
#ifndef NDEBUG
    static int counter = 0;
    counter++;
    uint32_t debug = sd_pio->fdebug & 0xffffff;
    if (debug) {
        printf("AWOOGA: %d %s %08x\n", counter, s, (uint)debug);
        sd_pio->fdebug = debug;
    }
#endif
}

static inline uint64_t sd_make_command(uint8_t cmd, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    cmd |= 0x40u;
    uint8_t crc = 0;
    crc = crc7_table[crc ^ cmd];
    crc = crc7_table[crc ^ b0];
    crc = crc7_table[crc ^ b1];
    crc = crc7_table[crc ^ b2];
    crc = crc7_table[crc ^ b3];

    uint64_t rc = b3;
    rc = (rc << 8u) | crc | 1u; // crc and stop bit
    rc = (rc << 16u) | sd_cmd_or_dat_offset_no_arg_state_wait_high;
    rc = (rc << 8u) | cmd;
    rc = (rc << 8u) | b0;
    rc = (rc << 8u) | b1;
    rc = (rc << 8u) | b2;
    return rc;
}

inline static int safe_wait_tx_empty(pio_hw_t *pio, uint sm) {
    int wooble = 0;
    while (!pio_sm_is_tx_fifo_empty(pio, sm)) {
        wooble++;
        if (wooble > 1000000) {
            check_pio_debug("stuck");
            printf("stuck %d @ %d\n", sm, (int)pio->sm[sm].addr);
            __breakpoint();
            return SD_ERR_STUCK;
        }
    }
    return SD_OK;
}

inline static int safe_wait_tx_not_full(pio_hw_t *pio, uint sm) {
    int wooble = 0;
    while (pio_sm_is_tx_fifo_full(pio, sm)) {
        wooble++;
        if (wooble > 1000000) {
            check_pio_debug("stuck");
            printf("stuck %d @ %d\n", sm, (int)pio->sm[sm].addr);
            __breakpoint();
            return SD_ERR_STUCK;
        }
    }
    return SD_OK;
}

inline static int safe_dma_wait_for_finish(pio_hw_t *pio, uint sm, uint chan) {
    int wooble = 0;
    while (dma_channel_is_busy(chan)) {
        wooble++;
        if (wooble > 8000000) {
            check_pio_debug("stuck dma");
            printf("stuck dma channel %d rem %08x %d @ %d\n", chan, (uint)dma_hw->ch[chan].transfer_count, sm, (int)pio->sm[sm].addr);
            __breakpoint();
            return SD_ERR_STUCK;
        }
    }
    return SD_OK;
}


static inline int acquiesce_sm(int sm) {
    check_pio_debug("ac1");
    int rc = safe_wait_tx_empty(sd_pio, sm);
    if (rc) return rc;
    check_pio_debug("ac2");
    uint32_t foo = 0;
    uint32_t timeout = 1000000;
    while (--timeout) {
        uint32_t addr = sd_pio->sm[sm].addr;
        foo |= 1<<addr;
        if (addr == sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd) {
            break;
        }
        // todo not forever
    }
    if (!timeout) return SD_ERR_STUCK;
    check_pio_debug("ac3");
    return SD_OK;
}

static int __time_critical_func(start_single_dma)(uint dma_channel, uint sm, uint32_t *buf, uint byte_length, bool bswap, bool sniff) {
    gpio_set_mask(1);
    uint word_length = (byte_length + 3) / 4;
    dma_channel_config c = dma_channel_get_default_config(dma_channel);
    channel_config_set_bswap(&c, bswap);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_PIO1_RX0 + sm);
    dma_channel_configure(
            dma_channel,
            &c,
            buf,                  // dest
            &sd_pio->rxf[sm],        // src
            word_length,
            false
    );
    if (sniff)
    {
        assert(sm == SD_DAT_SM);
        dma_sniffer_enable(dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, true);
        dma_hw->sniff_data = 0;
    }
    dma_channel_start(dma_channel);
    gpio_clr_mask(1);
    return SD_OK;
}

static void __time_critical_func(start_chain_dma_read_with_address_size_only)(uint sm, uint32_t *buf, bool bswap, bool sniff) {
    assert(!sniff); // for now
    dma_channel_config c = dma_channel_get_default_config(sd_data_dma_channel);
    channel_config_set_bswap(&c, bswap);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_PIO1_RX0 + sm);
    channel_config_set_chain_to(&c, sd_chain_dma_channel); // individual buffers chain back to master
    channel_config_set_irq_quiet(&c, true);

    dma_channel_configure(
            sd_data_dma_channel,
            &c,
            0,                    // dest
            &sd_pio->rxf[sm],        // src
            0,
            false
    );

    c = dma_channel_get_default_config(sd_chain_dma_channel);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, 1, 3);  // wrap the write at 8 bytes (so each transfer writes the same 2 word ctrl registers)
    dma_channel_configure(
            sd_chain_dma_channel,
            &c,
            &dma_channel_hw_addr(sd_data_dma_channel)->al1_write_addr,                    // dest
            buf,        // src
            2, // send 2 words to ctrl block of data chain per transfer
            false
    );

    gpio_set_mask(1);
//    if (sniff)
//    {
//        dma_enable_sniffer(sd_data_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC16);
//        dma_hw->sniff_data = 0;
//    }
    dma_channel_start(sd_chain_dma_channel);
    gpio_clr_mask(1);
}

static void __time_critical_func(start_chain_dma_read_with_full_cb)(uint sm, uint32_t *buf) {
    dma_channel_config c = dma_get_channel_config(sd_data_dma_channel);
    channel_config_set_chain_to(&c, sd_chain_dma_channel); // todo isn't this the case already
    channel_config_set_irq_quiet(&c, true); // todo isn't this the case already
    dma_channel_set_config(sd_data_dma_channel, &c, false);

    c = dma_channel_get_default_config(sd_chain_dma_channel);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, 1, 4);  // wrap the write at 16 bytes
    dma_channel_configure(
            sd_chain_dma_channel,
            &c,
            &dma_channel_hw_addr(sd_data_dma_channel)->read_addr,  // ch DMA config (target "ring" buffer size 16) - this is (read_addr, write_addr, transfer_count, ctrl),                    // dest
            buf,        // src
            4, // send 4 words to ctrl block of data chain per transfer
            false
    );
    gpio_set_mask(1);
    dma_channel_start(sd_chain_dma_channel);
    gpio_clr_mask(1);
}
static __attribute__((used)) __noinline void spoop() {
    int dma_channel = 3;
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, DREQ_SPI0_RX);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    dma_channel_set_config(dma_channel, &config, false);

    *(volatile uint32_t *)(DMA_BASE + DMA_CH3_AL1_CTRL_OFFSET) = 0x00089831;
}
static int __time_critical_func(start_read)(int sm, uint32_t *buf, uint byte_length, bool enable)
{
    spoop();
    int rc;
    gpio_set_mask(1);
    assert(!(3u & (uintptr_t)buf)); // in all fairness we should receive into a buffer from the pool
    uint bit_length = byte_length * 8;
    if (sm == SD_DAT_SM) {
        assert(!(bit_length & 31u));
        bit_length += bus_width == bw_wide ? 64 : 16;
    }
    rc = safe_wait_tx_not_full(sd_pio, sm);
    if (rc) return rc;
    check_pio_debug("sd_repsone_dma");
    if (bus_width == bw_wide && sm != SD_CMD_SM)
    {
        pio_sm_put(sd_pio, sm, sd_pio_cmd(sd_cmd_or_dat_offset_state_receive_bits, bit_length / 4 - 1));
        pio_sm_set_wrap(sd_pio, sm, sd_cmd_or_dat_offset_wrap_target_for_4bit_receive,
                        sd_cmd_or_dat_offset_wrap_for_4bit_receive -
                        1); // note -1since wrap values are on the last instruction before wrap
    }
    else
    {
        pio_sm_put(sd_pio, sm, sd_pio_cmd(sd_cmd_or_dat_offset_state_receive_bits, bit_length - 1));
        pio_sm_set_wrap(sd_pio, sm, sd_cmd_or_dat_wrap_target, sd_cmd_or_dat_wrap);
    }
    gpio_clr_mask(1);
    gpio_set_mask(1);
    if (enable) pio_sm_set_enabled(sd_pio, sm, true);
    if (bit_length & 31u)
    {
        rc = safe_wait_tx_not_full(sd_pio, sm);
        if (rc) return rc;
        pio_sm_put(sd_pio, sm, sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction,
                                    pio_encode_in(pio_null, 32 - (bit_length & 31u))));
    }
    // now go back to wait state
    rc = safe_wait_tx_not_full(sd_pio, sm);
    if (rc) return rc;
    pio_sm_put(sd_pio, sm, sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction, pio_encode_jmp(
            sm == SD_DAT_SM ? sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd
                            : sd_cmd_or_dat_offset_no_arg_state_wait_high)));
    gpio_clr_mask(1);
    return SD_OK;
}

static int __time_critical_func(finish_read)(uint dma_channel, int sm, uint16_t *suffixed_crc, uint16_t *sniffed_crc)
{
    gpio_set_mask(1);
    int rc = safe_dma_wait_for_finish(sd_pio, sm, dma_channel);
    if (rc) return rc;
    if (sniffed_crc) {
        *sniffed_crc = (uint16_t)dma_hw->sniff_data;
    }
    if (sm == SD_DAT_SM) {
        // todo not forever
        while (pio_sm_is_rx_fifo_empty(sd_pio, SD_DAT_SM));
        uint32_t w = sd_pio->rxf[SD_DAT_SM];
        if (suffixed_crc) *suffixed_crc = w >> 16u;
        if (bus_width == bw_wide) {
            while (pio_sm_is_rx_fifo_empty(sd_pio, SD_DAT_SM));
            sd_pio->rxf[SD_DAT_SM];
        }
    }
    assert(pio_sm_is_rx_fifo_empty(sd_pio, sm));
    gpio_clr_mask(1);
    return SD_OK;
}

int __time_critical_func(sd_response_dma)(uint dma_channel, uint sm, uint32_t *buf, uint byte_length, bool bswap, uint16_t *suffixed_crc, uint16_t *sniffed_crc, bool first, bool last, bool enable)
{
    int rc = SD_OK;
    if (first) {
        rc = start_single_dma(dma_channel, sm, buf, byte_length, bswap, sniffed_crc != 0);
        if (!rc) rc = start_read(sm, buf, byte_length, enable);
    }

    if (!rc) rc =  finish_read(dma_channel, sm, suffixed_crc, sniffed_crc);

    if (!last && !rc) {
        if (!rc) rc = start_single_dma(dma_channel, sm, buf, byte_length, bswap, sniffed_crc != 0);
        if (!rc) rc = start_read(sm, buf, byte_length, enable);
    }
    return rc;
}

int __noinline sd_command(uint64_t packed_command, uint32_t *receive_buf, uint byte_length)
{
    int rc = acquiesce_sm(SD_CMD_SM);
    if (rc) return rc;
    sd_debug("SD command %d\n", 0x3fu & (((uint)packed_command) >> 24u));
    // disable SM so we don't have a race on us filling the FIFO - we must not stall or we will lose sync with clock
    pio_sm_set_enabled(sd_pio, SD_CMD_SM, false);
    pio_sm_put(sd_pio, SD_CMD_SM, sd_pio_cmd(sd_cmd_or_dat_offset_state_send_bits, 48 - 1));
    pio_sm_put(sd_pio, SD_CMD_SM, (uint32_t) packed_command);
    pio_sm_put(sd_pio, SD_CMD_SM, (uint32_t) (packed_command >> 32u));
    // todo we know the recvlen based on the command
    if (byte_length)
    {
        rc = sd_response_dma(sd_cmd_dma_channel, SD_CMD_SM, receive_buf, byte_length, false, NULL, NULL, true, true, true);
        if (!rc)
        {
            uint32_t cmd = ((uint32_t) packed_command) >> 24u;
            cmd &= 63u;
            uint32_t w0 = receive_buf[0] >> 1u;
            uint32_t w1 = (receive_buf[1] >> 1u) | (receive_buf[0] & 1u ? 0x80000000u : 0u);
            bool ok = true;
            switch (cmd)
            {
                case 2:
                    break;
                case 41:
                    ok = (w0 & 0xff1e0000) == 0x3f000000;
                    ok &= (w1 >> 16) == 0xff;
                    break;
                default:
                {
                    if (cmd != w0 >> 24u)
                    {
                        printf("tsk\n");
                    }
                    uint8_t crc = crc7_table[w0 >> 24u];
                    crc = crc7_table[crc ^ (uint8_t) (w0 >> 16u)];
                    crc = crc7_table[crc ^ (uint8_t) (w0 >> 8u)];
                    crc = crc7_table[crc ^ (uint8_t) w0];
                    crc = crc7_table[crc ^ (uint8_t) (w1 >> 24u)];
                    if ((crc | 1u) != (uint8_t) (w1 >> 16u))
                    {
                        panic("bad crc %02x != %02x\n", crc | 1u, (uint8_t) (w1 >> 16u));
                        ok = false;
                    }
                }
            }
            if (!ok)
            {
                printf("bad response from card\n");
                return SD_ERR_BAD_RESPONSE;
            }
        }
    } else {
        pio_sm_set_enabled(sd_pio, SD_CMD_SM, true);
    }
    sd_debug("SD command done %d\n", 0x3fu & (((uint)packed_command) >> 24u));
    return SD_OK;
}

int sd_wait()
{
    int rc = acquiesce_sm(SD_DAT_SM);
    if (!rc)
    {
        pio_sm_put(sd_pio, SD_DAT_SM, sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction, pio_encode_set(pio_pindirs, 0)));
        pio_sm_put(sd_pio, SD_DAT_SM,
                sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction, pio_encode_wait_pin(true, 0)));
        pio_sm_put(sd_pio, SD_DAT_SM, sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction,
                                           pio_encode_jmp(sd_cmd_or_dat_offset_no_arg_state_wait_high)));
        rc = acquiesce_sm(SD_DAT_SM);
    }
    return rc;
}

static inline void fixup_cmd_response_48(uint32_t *buf)
{
    uint32_t b0 = buf[0];
    buf[1] = __builtin_bswap32((buf[1] >> 1u) | ((b0 & 1u) << 31u));
    buf[0] = __builtin_bswap32(b0 >> 1u);
}

const char *states[] = {
        "idle", "ready", "ident", "stby", "tran", "data", "rcv", "prg", "dis", "(9)", "(a)", "(b)", "(c)", "(d)", "(e)", "(f)"
};

void print_status(uint32_t *response_buffer, bool needs_fixup) {
    uint32_t r[2];
    const uint8_t *b;
    if (needs_fixup) {
        r[0] = response_buffer[0];
        r[1] = response_buffer[1];
        fixup_cmd_response_48(r);
        b = (const uint8_t *)r;
    } else {
        b = (const uint8_t *)response_buffer;
    }
    if (b[1]&0x80) printf(" ORANGE");
    if (b[1]&0x40) printf(" ADDRESS");
    if (b[1]&0x20) printf(" BLEN");
    if (b[1]&0x10) printf(" ESEQ");
    if (b[1]&0x8) printf(" EPARM");
    if (b[1]&0x4) printf(" WPV");
    if (b[1]&0x2) printf(" LOCKED");
    if (b[1]&0x1) printf(" UNLOCK");
    if (b[2]&0x80) printf(" CRC");
    if (b[2]&0x40) printf(" ILLEGAL");
    if (b[2]&0x20) printf(" ECC");
    if (b[2]&0x10) printf(" INTERNAL");
    if (b[2]&0x8) printf(" << ERRORS: ");
    if (b[3]&0x80) printf(" era_skip");
    if (b[3]&0x40) printf(" ecc_dis");
    if (b[3]&0x20) printf(" era_reset");
    printf(" %s", states[(b[3]>>1u)&0xfu]);
    printf((b[3] & 1) ? " ready" : " not-ready");
    if (b[4] & 0x20) printf(" ACMD...");
    printf("\n");
}

void read_status(bool dump)
{
    uint32_t response_buffer[5];

    int not_ready_retries = 3;
    while (not_ready_retries--) {
      // let's see the status
      sd_command(sd_make_command(13, rca_high, rca_low, 0, 0), response_buffer, 6);
      fixup_cmd_response_48(response_buffer);
      uint8_t *b = (uint8_t *)response_buffer;
      printf("%02x %02x %02x %02x : %02x %02x\n", b[0], b[1], b[2], b[3], b[4], b[5]);
      if (dump) {
        print_status(response_buffer, false);
      }

      // Break if ready
      if (b[3] & 1) break;

      // Wait if not ready and try again
      sleep_ms(1);
    }
}

int sd_set_wide_bus(bool wide)
{
  printf("Set bus width: %d\n", (wide ? 4 : 1));
    if (bus_width == bw_unknown || bus_width == (wide ? bw_narrow : bw_wide)) {
        if (wide && !allow_four_data_pins) {
            printf("May not select wide pus without 4 data pins\n");
            return SD_ERR_BAD_PARAM;
        }
        uint32_t response_buffer[5];
        int rc = sd_command(sd_make_command(55, rca_high, rca_low, 0, 0), response_buffer, 6);
        if (!rc)
        {
            rc = sd_command(sd_make_command(6, 0, 0, 0, wide ? 2 : 0), response_buffer, 6);
        }
        if (!rc) {
            bus_width = wide ? bw_wide : bw_narrow;
        } else {
            bus_width = bw_unknown;
        }
        return rc;
    }
    return SD_OK;
}

int sd_set_clock_divider(uint div) {
#ifdef PICO_SD_CARD_EXTRA_CLOCK_DIVIDER
    div *= PICO_SD_CARD_EXTRA_CLOCK_DIVIDER;
#endif
    pio_sm_set_clkdiv_int_frac(sd_pio, SD_CLK_SM, div, 0);
    pio_sm_set_clkdiv_int_frac(sd_pio, SD_CMD_SM, div, 0);
    pio_sm_set_clkdiv_int_frac(sd_pio, SD_DAT_SM, div, 0);
    pio_clkdiv_restart_sm_mask(sd_pio, (1u << SD_CMD_SM) | (1u << SD_CLK_SM) | (1u << SD_DAT_SM));
    return SD_OK;
}

// todo fixup error handling
static int sd_init( bool _allow_four_data_pins)
{
    bi_decl_if_func_used(bi_2pins_with_names(PICO_SD_CLK_PIN, "SDIO clock", PICO_SD_CMD_PIN, "SDIO cmd"));
    bus_width = bw_unknown;
    int sd_clk_pin = PICO_SD_CLK_PIN;
    int sd_cmd_pin = PICO_SD_CMD_PIN;
    sd_dat_pin_base = PICO_SD_DAT0_PIN;
    // todo #define for four allowing four pins
    gpio_set_function(sd_clk_pin, GPIO_FUNC_PIO1);
    gpio_set_function(sd_cmd_pin, GPIO_FUNC_PIO1);
    gpio_set_function(sd_dat_pin_base, GPIO_FUNC_PIO1);
    gpio_set_pulls(sd_clk_pin, false, true);
    gpio_set_pulls(sd_cmd_pin, true, false);
    gpio_set_pulls(sd_dat_pin_base, true, false);
    allow_four_data_pins = _allow_four_data_pins;

    // Have to set pulls on other pins regardless otherwise SD card fails to
    // initialise in 1 bit mode
    gpio_set_function(sd_dat_pin_base+1, GPIO_FUNC_PIO1);
    gpio_set_function(sd_dat_pin_base+2, GPIO_FUNC_PIO1);
    gpio_set_function(sd_dat_pin_base+3, GPIO_FUNC_PIO1);
    gpio_set_pulls(sd_dat_pin_base+1, true, false);
    gpio_set_pulls(sd_dat_pin_base+2, true, false);
    gpio_set_pulls(sd_dat_pin_base+3, true, false);

    static bool added; // todo this is a temporary hack as we don't free
    static uint cmd_or_dat_offset;
    static uint clk_program_offset;

    if (!added) {
        cmd_or_dat_offset = pio_add_program(sd_pio, &sd_cmd_or_dat_program);
        assert(!cmd_or_dat_offset); // we don't add this later because it is assumed to be 0
        clk_program_offset = pio_add_program(sd_pio, &sd_clk_program);
        added = true;
    }

    pio_sm_config c = sd_clk_program_get_default_config(clk_program_offset);
    sm_config_set_sideset_pins(&c, sd_clk_pin);
    pio_sm_init(sd_pio, SD_CLK_SM, clk_program_offset, &c);

    c = sd_cmd_or_dat_program_get_default_config(cmd_or_dat_offset);
    sm_config_set_out_pins(&c, sd_cmd_pin, 1);
    sm_config_set_set_pins(&c, sd_cmd_pin, 1);
    sm_config_set_in_pins(&c, sd_cmd_pin);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_out_shift(&c, false, true, 32);
    pio_sm_init(sd_pio, SD_CMD_SM, cmd_or_dat_offset, &c);

    c = sd_cmd_or_dat_program_get_default_config(cmd_or_dat_offset);
    uint num_dat_pins = allow_four_data_pins ? 4 : 1;
    sm_config_set_out_pins(&c, sd_dat_pin_base, num_dat_pins);
    sm_config_set_set_pins(&c, sd_dat_pin_base, num_dat_pins);
    sm_config_set_in_pins(&c, sd_dat_pin_base);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_out_shift(&c, false, true, 32);
    pio_sm_init(sd_pio, SD_DAT_SM, cmd_or_dat_offset, &c);

    sd_set_clock_divider(50);
//    int div = 50;
//    pio_sm_set_clkdiv_int_frac(pio, SD_CLK_SM, div, 0); // run at 240 kHz as required initially
//    pio_sm_set_clkdiv_int_frac(pio, SD_CMD_SM, div, 0); // run at 240 kHz as required initially
//    pio_sm_set_clkdiv_int_frac(pio, SD_DAT_SM, div, 0); // run at 240 kHz as required initially
// //   pio_clkdiv_restart_mask(pio, (1u << SD_CMD_SM) | (1u << SD_CLK_SM) | (1u << SD_DAT_SM));

    // set later anyway
//    pio_set_wrap(pio, SD_DAT_SM, sd_cmd_or_dat_wrap_target, sd_cmd_or_dat_wrap);


    pio_sm_exec(sd_pio, SD_CMD_SM, pio_encode_jmp(sd_cmd_or_dat_offset_no_arg_state_wait_high));
    pio_sm_exec(sd_pio, SD_DAT_SM, pio_encode_jmp(sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd));

    uint32_t dat_pin_mask = allow_four_data_pins ? 0xfu : 0x1u;
    uint32_t all_pin_mask = (dat_pin_mask << sd_dat_pin_base) | (1u << sd_cmd_pin) | (1u << sd_clk_pin);
    pio_sm_set_pindirs_with_mask(sd_pio, SD_CLK_SM, all_pin_mask, all_pin_mask);
    pio_sm_exec(sd_pio, SD_DAT_SM, pio_encode_set(pio_pins, dat_pin_mask));

    //pio_sm_enable(pio, SD_CLK_SM, 1);
//    pio_sm_put(sd_pio, SD_CMD_SM, sd_pio_cmd(sd_offset_state_send_bits, 72 - 1));
    // we use 80 rather than 72 so we can keep our 16 bit instruction stream aligned
    pio_sm_put(sd_pio, SD_CMD_SM, sd_pio_cmd(sd_cmd_or_dat_offset_state_send_bits, 80 - 1));
    pio_sm_put(sd_pio, SD_CMD_SM, 0xffffffff);
    pio_sm_put(sd_pio, SD_CMD_SM, 0xffffffff);
//    pio_sm_put(sd_pio, SD_CMD_SM, 0xff000000 | (sd_offset_dump_osr_then_wait_high << 16u));
    pio_sm_put(sd_pio, SD_CMD_SM, 0xffff0000 | pio_encode_jmp(sd_cmd_or_dat_offset_no_arg_state_wait_high));
    pio_enable_sm_mask_in_sync(sd_pio, (1u<<SD_CMD_SM)|(1u<<SD_CLK_SM)|(1u<<SD_DAT_SM));

    uint32_t response_buffer[5];
    sd_command(sd_make_command(0, 0, 0, 0, 0), response_buffer, 0);
    sd_command(sd_make_command(8, 0, 0, 1, 0xa5), response_buffer, 6); // VHS=b0001

    uint8_t *byte_buf = (uint8_t *) response_buffer;
    fixup_cmd_response_48(response_buffer);
    if (byte_buf[4] != 0xa5)
    {
        __breakpoint();
        printf("R7 check pattern doesn't match sent\r\n");
        return -1;
    }

    do
    {
        sd_command(sd_make_command(55, 0, 0, 0, 0), response_buffer, 6);
        sd_command(sd_make_command(41, 0x40, 0x10, 0, 0), response_buffer, 6); // HCS=1, 3.2-3.3V only
        fixup_cmd_response_48(response_buffer);
        assert(byte_buf[0] == 0x3f);
    }
    while (!(byte_buf[1] & 0x80u)); // repeat while nbusy bit is low
    printf("Card ready\r\n");

    sd_command(sd_make_command(2, 0, 0, 0, 0), response_buffer, 17);
    sd_command(sd_make_command(3, 0, 0, 0, 0), response_buffer, 6);
    fixup_cmd_response_48(response_buffer);
    rca_high = byte_buf[1];
    rca_low = byte_buf[2];
    sd_command(sd_make_command(7, rca_high, rca_low, 0, 0), response_buffer, 6);

    // wait for not busy after CMD7
    sd_wait();

    int rc = sd_set_wide_bus(allow_four_data_pins);
    if (!rc)
    {
        rc = sd_set_clock_divider(1); // as fast as possible please
    }
    return rc;
}

int sd_init_1pin() {
    bi_decl_if_func_used(bi_1pin_with_name(PICO_SD_DAT0_PIN, "SDIO data"));
    return sd_init(false);
}

int sd_init_4pins() {
    // note names are | separated, and repeat if not specified; i.e. this names all pins the same
    bi_decl_if_func_used(bi_pin_mask_with_names(0xfu << PICO_SD_DAT0_PIN, "SDIO data 0-3"));
    return sd_init(true);
}

void sd_set_byteswap_on_read(bool swap)
{
    bytes_swap_on_read = swap;
}

static uint32_t crcs[PICO_SD_MAX_BLOCK_COUNT * 2];
static uint32_t ctrl_words[(PICO_SD_MAX_BLOCK_COUNT + 1) * 4];
static uint32_t pio_cmd_buf[PICO_SD_MAX_BLOCK_COUNT * 3];

int sd_readblocks_async(uint32_t *buf, uint32_t block, uint block_count)
{
    assert(block_count <= PICO_SD_MAX_BLOCK_COUNT);

    uint32_t *p = ctrl_words;
    uint crc_words = bus_width == bw_wide ? 2 : 1;
    for(int i = 0; i < block_count; i++)
    {
        *p++ = (uintptr_t) (buf + i * 128);
        *p++ = 128;
        // for now we read the CRCs also
        *p++ = (uintptr_t)(crcs + i * crc_words);
        *p++ = crc_words;
    }
    *p++ = 0;
    *p++ = 0;
    return sd_readblocks_scatter_async(ctrl_words, block, block_count);
}

int sd_readblocks_sync(uint32_t *buf, uint32_t block, uint block_count)
{
    assert(block_count <= PICO_SD_MAX_BLOCK_COUNT);

    uint32_t *p = ctrl_words;
    uint crc_words = bus_width == bw_wide ? 2 : 1;
    for(int i=0;i<block_count;i++)
    {
        *p++ = (uintptr_t)(buf + i * 128);
        *p++ = 128;
        // for now we read the CRCs also
        *p++ = (uintptr_t)(crcs + i * crc_words);
//        printf("%08x\n", (uint)(uint32_t)(crcs + i * crc_words));
        *p++ = crc_words;
    }
    *p++ = 0;
    *p++ = 0;
    int rc = sd_readblocks_scatter_async(ctrl_words, block, block_count);
    if (!rc)
    {
//        printf("waiting for finish\n");
        while (!sd_scatter_read_complete(&rc))
        {
            tight_loop_contents();
        }
//        for(int i=0;i<block_count;i++)
//        {
//            printf("y %08x\n", (uint) crcs[i * crc_words]);
//        }

        //printf("finished\n");
    }
    return rc;
}

static uint32_t *start_read_to_buf(int sm, uint32_t *buf, uint byte_length, bool first)
{
    uint bit_length = byte_length * 8;
    if (sm == SD_DAT_SM) {
        assert(!(bit_length & 31u));
        bit_length += bus_width == bw_wide ? 64 : 16;
    }
    if (bus_width == bw_wide && sm != SD_CMD_SM)
    {
        *buf++ = sd_pio_cmd(sd_cmd_or_dat_offset_state_receive_bits, bit_length / 4 - 1);
        if (first) pio_sm_set_wrap(sd_pio, sm, sd_cmd_or_dat_offset_wrap_target_for_4bit_receive, sd_cmd_or_dat_offset_wrap_for_4bit_receive - 1); // note -1since wrap values are on the last instruction before wrap
    } else {
        *buf++ = sd_pio_cmd(sd_cmd_or_dat_offset_state_receive_bits, bit_length - 1);
        if (first) pio_sm_set_wrap(sd_pio, sm, sd_cmd_or_dat_wrap_target, sd_cmd_or_dat_wrap);
    }
    // add zero padding to word boundary if necessary
    if (bit_length & 31u)
    {
        *buf++ = sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction, pio_encode_in(pio_null, 32 - (bit_length & 31u)));
    }
    *buf++ = sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction, pio_encode_jmp(sm == SD_DAT_SM?sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd:sd_cmd_or_dat_offset_no_arg_state_wait_high));
    return buf;
}

// note caller must make space for CRC (2 word) in 4 bit mode
int sd_readblocks_scatter_async(uint32_t *control_words, uint32_t block, uint block_count)
{
    uint32_t response_buffer[5];

    assert(pio_sm_is_rx_fifo_empty(sd_pio, SD_DAT_SM));
    uint32_t total = 0;
    uint32_t *p = control_words;
    while (p[0]) {
//        printf("%p %08x %08x\n", p, (uint)p[0], (uint)p[1]);
        assert(p[1]);
        total += p[1];
        p += 2;
    }

    // todo further state checks
    while (sd_pio->sm[SD_DAT_SM].addr != sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd) {
        printf("oops %d\n", (uint)sd_pio->sm[SD_DAT_SM].addr);
    }
    assert(sd_pio->sm[SD_DAT_SM].addr == sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd);
    assert(pio_sm_is_rx_fifo_empty(sd_pio, SD_DAT_SM));
    assert(block_count <= PICO_SD_MAX_BLOCK_COUNT);

    assert(total == block_count * (128 + (bus_width == bw_wide ? 2 : 1)));
    start_chain_dma_read_with_address_size_only(SD_DAT_SM, control_words, !bytes_swap_on_read, false);
    uint32_t *buf = pio_cmd_buf;
    for(int i=0;i<block_count;i++) {
        buf = start_read_to_buf(SD_DAT_SM, buf, 512, !i);
    }

    dma_channel_config c = dma_channel_get_default_config(sd_pio_dma_channel);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_PIO1_TX0 + SD_DAT_SM);
    dma_channel_configure(
            sd_pio_dma_channel,
            &c,
            &sd_pio->txf[SD_DAT_SM],                  // dest
            pio_cmd_buf,        // src
            buf - pio_cmd_buf,
            true
    );
    // todo decide timing of this - as long as dat lines are hi, this is fine. (note this comment now applies to the trigger true in the dma_channel_configure)
    // dma_channel_start(sd_pio_dma_channel);
    assert(block_count);
    int rc;
    if (block_count == 1) {
        rc = sd_command(sd_make_command(17, block >> 24, block >> 16, block >> 8, block & 0xffu), response_buffer, 6);
    } else
    {
//        read_status(true);
        // todo can we expect support for 23?
        rc = sd_command(sd_make_command(23, block_count >> 24, block_count >> 16, block_count >> 8, block_count & 0xffu),
                        response_buffer, 6);
        if (!rc) rc = sd_command(sd_make_command(18, block >> 24, block >> 16, block >> 8, block & 0xffu), response_buffer, 6);
    }
    return rc;
}

int check_crc_count;

bool sd_scatter_read_complete(int *status) {
//    printf("%d:%d %d:%d %d:%d %d\n", dma_busy(sd_chain_dma_channel), (uint)dma_hw->ch[sd_chain_dma_channel].transfer_count,
//           dma_busy(sd_data_dma_channel), (uint)dma_hw->ch[sd_data_dma_channel].transfer_count,
//           dma_busy(sd_pio_dma_channel), (uint)dma_hw->ch[sd_pio_dma_channel].transfer_count, (uint)pio->sm[SD_DAT_SM].addr);
    // this is a bit half arsed atm
    bool rc;
    if (dma_channel_is_busy(sd_chain_dma_channel) || dma_channel_is_busy(sd_data_dma_channel) || dma_channel_is_busy(sd_pio_dma_channel)) {
        rc = false;
    } else {
        rc = (sd_pio->sm[SD_DAT_SM].addr == sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd &&
              pio_sm_is_tx_fifo_empty(sd_pio, SD_DAT_SM));
    }
    int s = SD_OK;
    if (rc) {
//        read_status(true);
        for(int i=0;i<check_crc_count;i++) {
            if ((crcs[i*2] >> 16u) != crcs[i*2+1]) {
                printf("CRC error on block %d\n", i);
                s = SD_ERR_CRC;
                break;
            }
        }
        check_crc_count = 0;
    }
    if (status) *status = s;
    return rc;
}

static void __time_critical_func(start_chain_dma_write)(uint sm, uint32_t *buf) {
    dma_channel_config c = dma_get_channel_config(sd_data_dma_channel);
    channel_config_set_chain_to(&c, sd_chain_dma_channel);
    channel_config_set_irq_quiet(&c, true);
    dma_channel_set_config(sd_data_dma_channel, &c, false);

    c = dma_channel_get_default_config(sd_chain_dma_channel);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, 1, 4);  // wrap the write at 16 bytes
    dma_channel_configure(
            sd_chain_dma_channel,
            &c,
            &dma_channel_hw_addr(sd_data_dma_channel)->read_addr,  // ch DMA config (target "ring" buffer size 16) - this is (read_addr, write_addr, transfer_count, ctrl),                    // dest
            buf,        // src
            4, // send 4 words to ctrl block of data chain per transfer
            false
    );
    gpio_set_mask(1);
    dma_channel_start(sd_chain_dma_channel);
    gpio_clr_mask(1);
}

static __unused uint8_t flapulent[1024];
static __unused uint32_t flap_count;
uint32_t zeroes;
uint32_t start_bit = 0xfffffffe;

static uint32_t dma_ctrl_for(enum dma_channel_transfer_size size, bool src_incr, bool dst_incr, uint dreq,
                                    uint chain_to, bool ring_sel, uint ring_size, bool enable) {
    dma_channel_config c = dma_channel_get_default_config(0); // channel doesn't matter as we set chain_to later (it is just use to pre-populate that)
    channel_config_set_transfer_data_size(&c, size);
    channel_config_set_read_increment(&c, src_incr);
    channel_config_set_write_increment(&c, dst_incr);
    channel_config_set_dreq(&c, dreq);
    channel_config_set_chain_to(&c, chain_to);
    channel_config_set_ring(&c, ring_sel, ring_size);
    channel_config_set_enable(&c, enable);
    return c.ctrl;
}
//#define CRC_FIRST
// note caller must make space for CRC (2 word) in 4 bit mode
int sd_writeblocks_async(const uint32_t *data, uint32_t sector_num, uint sector_count)
{
    uint32_t response_buffer[5];

#ifdef CRC_FIRST
    // lets crc the first sector
    dma_channel_config c = dma_channel_get_default_config(sd_data_dma_channel);
    if (true)
    {
        channel_config_set_bswap(&c, true);
        dma_sniffer_enable(sd_data_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, true);
        dma_sniffer_set_byte_swap_enabled(true);
    } else {
        dma_sniffer_enable(sd_data_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, true);
    }
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_FORCE);
    dma_channel_configure(
            sd_data_dma_channel,
            &c,
            flapulent,                  // dest
            data,        // src
            128,
            false
    );
    hw_set_bits(&dma_hw->ch[sd_data_dma_channel].al1_ctrl, DMA_CH0_CTRL_TRIG_BSWAP_BITS);
    dma_hw->sniff_data = 0;
    dma_channel_start(sd_data_dma_channel);
    dma_channel_wait_for_finish_blocking(sd_data_dma_channel);
    printf("Sniff raw %08x, word %04x\n", (uint)dma_hw->sniff_data, __bswap16(dma_hw->sniff_data));
    // todo we need to be able to reset the sniff data correctly
    crcs[0] = __bswap16(dma_hw->sniff_data);
#endif

    uint32_t *buf = pio_cmd_buf;
    for(int i=0; i < sector_count; i++)
    {
        // we send an extra word even though the CRC is only 16 bits to make life easy... the receiver doesn't care
        // todo that would need to work anyway for inline CRC (which can't include a pio_cmd)
        *buf++ = sd_pio_cmd(sd_cmd_or_dat_offset_state_send_bits, 512 * 8 + 32 + 32 - 1);
    }
    *buf++ = sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction, pio_encode_jmp(sd_cmd_or_dat_offset_no_arg_state_wait_high));

    if (sector_count > (PICO_SD_MAX_BLOCK_COUNT - 1) / 4) {
        panic("too many blocks for now");
    }

    assert(pio_sm_is_tx_fifo_empty(sd_pio, SD_DAT_SM));

    uint32_t *p = ctrl_words;
//#define SEND_TO_BUFFER
#ifdef SEND_TO_BUFFER
    uint32_t *output_buffer = flapulent;
    uint32_t offset = 0;
    #define build_transfer(src, words, size, flags) \
        *p++ = (uintptr_t)(src); \
        *p++ = output_buffer + offset; \
        *p++ = words; \
        offset += words; \
        *p++ = dma_ctrl_for(size, true, true, DREQ_FORCE, sd_chain_dma_channel, 0, 0, true) | (flags);
#else
#define build_transfer(src, words, size, flags) \
        *p++ = (uintptr_t)(src); \
        *p++ = (uintptr_t)(&sd_pio->txf[SD_DAT_SM]); \
        *p++ = words; \
        *p++ = dma_ctrl_for(size, true, false, DREQ_PIO1_TX0 + SD_DAT_SM, sd_chain_dma_channel, 0, 0, true) | (flags);

#endif
    for(int i=0;i<sector_count;i++) {
        // first cb - zero out sniff data
#ifndef CRC_FIRST
        *p++ = (uintptr_t)&zeroes;
        *p++ = (uintptr_t)(&dma_hw->sniff_data);
        *p++ = 1;
        *p++ = dma_ctrl_for(DMA_SIZE_32, false, false, DREQ_FORCE, sd_chain_dma_channel, 0, 0, true);
#endif
        // second cb - send bits command
        build_transfer(pio_cmd_buf + i, 1, DMA_SIZE_32, 0);
        build_transfer(&start_bit, 1, DMA_SIZE_32, 0);
        // third cb - 128 words of sector data
        build_transfer(data + i * 128, 128, DMA_SIZE_32, DMA_CH0_CTRL_TRIG_BSWAP_BITS | DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS);
//        // fourth cb - transfer sniff
#ifdef CRC_FIRST
        build_transfer( crcs, 1, DMA_SIZE_32, DMA_CH0_CTRL_TRIG_BSWAP_BITS);
#else
        // note offset of 2, since we bswap the data
        build_transfer( (uintptr_t)&dma_hw->sniff_data, 1, DMA_SIZE_16, 0);//DMA_CH0_CTRL_TRIG_BSWAP_BITS);
#endif
    }
    // final cb - return to wait state
    build_transfer(pio_cmd_buf + sector_count, 1, DMA_SIZE_32, 0);
#ifdef SEND_TO_BUFFER
    flap_count = offset;
#endif
    *p++ = 0;
    *p++ = 0;

    // todo further state checks
    while (sd_pio->sm[SD_DAT_SM].addr != sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd) {
        printf("oops %d\n", (uint)sd_pio->sm[SD_DAT_SM].addr);

    }
    assert(sd_pio->sm[SD_DAT_SM].addr == sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd);
    assert(pio_sm_is_tx_fifo_empty(sd_pio, SD_DAT_SM));
    pio_sm_put(sd_pio, SD_DAT_SM, sd_pio_cmd(sd_cmd_or_dat_offset_state_inline_instruction, pio_encode_jmp(sd_cmd_or_dat_offset_no_arg_state_wait_high)));
    while (sd_pio->sm[SD_DAT_SM].addr != sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd) {
        printf("reps %d\n", (uint)sd_pio->sm[SD_DAT_SM].addr);

    }

    assert(sector_count);
    int rc = sd_set_wide_bus(false); // use 1 bit writes for now
    if (rc) return rc;

    if (sector_count == 1) {
        rc = sd_command(sd_make_command(24, sector_num >> 24, (sector_num >> 16) & 0xff, (sector_num >> 8) & 0xff, sector_num & 0xffu), response_buffer, 6);
    } else
    {
        // todo this is only writing the first sector on SanDisk EDGE 16G right now - probably need a delay between sectors... works fine on a SAMSUNG EVO 32G

        // todo can we expect support for 23?
        rc = sd_command(sd_make_command(23, sector_count >> 24, sector_count >> 16, sector_count >> 8, sector_count & 0xffu),
                        response_buffer, 6);
        if (!rc) rc = sd_command(sd_make_command(25, sector_num >> 24, sector_num >> 16, sector_num >> 8, sector_num & 0xffu), response_buffer, 6);
    }
    read_status(true);
    if (!rc)
    {
        pio_sm_set_enabled(sd_pio, SD_DAT_SM, false);
        dma_sniffer_enable(sd_data_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, true);
        dma_sniffer_set_byte_swap_enabled(true);
        start_chain_dma_write(SD_DAT_SM, ctrl_words);
        pio_sm_set_enabled(sd_pio, SD_DAT_SM, true);
        printf("dma chain data (rem %04x @ %08x) data (rem %04x @ %08x) pio data (rem %04x @ %08x) datsm @ %d\n",
               (uint) dma_hw->ch[sd_chain_dma_channel].transfer_count,
               (uint) dma_hw->ch[sd_chain_dma_channel].read_addr,
               (uint) dma_hw->ch[sd_data_dma_channel].transfer_count, (uint) dma_hw->ch[sd_data_dma_channel].read_addr,
               (uint) dma_hw->ch[sd_pio_dma_channel].transfer_count, (uint) dma_hw->ch[sd_pio_dma_channel].read_addr,
               (int) sd_pio->sm[SD_DAT_SM].addr);

    }
    return rc;
}

bool sd_write_complete(int *status) {
    printf("dma chain data (rem %04x @ %08x) data (rem %04x @ %08x) datsm @ %d\n",
           (uint)dma_hw->ch[sd_chain_dma_channel].transfer_count, (uint)dma_hw->ch[sd_chain_dma_channel].read_addr,
           (uint)dma_hw->ch[sd_data_dma_channel].transfer_count, (uint)dma_hw->ch[sd_data_dma_channel].read_addr,
           (int)sd_pio->sm[SD_DAT_SM].addr);
    // this is a bit half arsed atm
    bool rc;
    if (dma_channel_is_busy(sd_chain_dma_channel) || dma_channel_is_busy(sd_data_dma_channel)) rc = false;
    else rc = sd_pio->sm[SD_DAT_SM].addr == sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd;
    if (rc) {
        read_status(true);
        printf("sniffo %08x\n", (uint)dma_hw->sniff_data);
    }
    if (status) *status = SD_OK;
    return rc;
}

#if 1
// note caller must make space for CRC (2 word) in 4 bit mode
int sd_read_sectors_1bit_crc_async(uint32_t *sector_buf, uint32_t sector, uint sector_count)
{
    uint32_t response_buffer[5];

    sd_set_wide_bus(false);
    assert(pio_sm_is_rx_fifo_empty(sd_pio, SD_DAT_SM));

    if (sector_count > (PICO_SD_MAX_BLOCK_COUNT - 1) / 4) {
        panic("too many blocks for now");
    }

    check_crc_count = sector_count;
    uint32_t *p = ctrl_words;
    for(int i=0;i<sector_count;i++) {
        // first cb - zero out sniff data
        *p++ = (uintptr_t)&zeroes;
        *p++ = (uintptr_t)(&dma_hw->sniff_data);
        *p++ = 1;
        *p++ = dma_ctrl_for(DMA_SIZE_32, false, false, DREQ_FORCE, sd_chain_dma_channel, 0, 0, true);
        // second cb - 128 words of sector data
        *p++ = (uintptr_t)(&sd_pio->rxf[SD_DAT_SM]);
        *p++ = (uintptr_t)(sector_buf + i * 128);
        *p++ = 128;
        *p++ = dma_ctrl_for(DMA_SIZE_32, false, true, DREQ_PIO1_RX0 + SD_DAT_SM, sd_chain_dma_channel, 0, 0, true) | DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS | DMA_CH0_CTRL_TRIG_BSWAP_BITS;
        // third crc from stream
        *p++ = (uintptr_t)(&sd_pio->rxf[SD_DAT_SM]);
        *p++ = (uintptr_t)(crcs + i * 2);
        *p++ = 1;
        *p++ = dma_ctrl_for(DMA_SIZE_32, false, false, DREQ_PIO1_RX0 + SD_DAT_SM, sd_chain_dma_channel, 0, 0, true);
        // fourth crc from sniff
        *p++ = (uintptr_t)&dma_hw->sniff_data;
        *p++ = (uintptr_t)(crcs + i * 2 + 1);
        *p++ = 1;
        *p++ = dma_ctrl_for(DMA_SIZE_32, false, false, DREQ_FORCE, sd_chain_dma_channel, 0, 0, true);
    }
    *p++ = 0;
    *p++ = 0;

    // todo further state checks
    while (sd_pio->sm[SD_DAT_SM].addr != sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd) {
        printf("oops %d\n", (uint)sd_pio->sm[SD_DAT_SM].addr);
    }
    assert(sd_pio->sm[SD_DAT_SM].addr == sd_cmd_or_dat_offset_no_arg_state_waiting_for_cmd);
    assert(pio_sm_is_rx_fifo_empty(sd_pio, SD_DAT_SM));
    assert(sector_count <= PICO_SD_MAX_BLOCK_COUNT);

    dma_sniffer_enable(sd_data_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, false);
    //dma_enable_sniffer_byte_swap(true);
    start_chain_dma_read_with_full_cb(SD_DAT_SM, ctrl_words);
    uint32_t *buf = pio_cmd_buf;
    for(int i=0;i<sector_count;i++) {
        buf = start_read_to_buf(SD_DAT_SM, buf, 512, !i);
    }
    dma_channel_config c = dma_channel_get_default_config(sd_pio_dma_channel);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_PIO1_TX0 + SD_DAT_SM);
    dma_channel_configure(
            sd_pio_dma_channel,
            &c,
            &sd_pio->txf[SD_DAT_SM],                  // dest
            pio_cmd_buf,        // src
            buf - pio_cmd_buf,
            false
    );
    // todo decide timing of this - as long as dat lines are hi, this is fine.
    dma_channel_start(sd_pio_dma_channel);
    assert(sector_count);
    int rc;
    if (sector_count == 1) {
        rc = sd_command(sd_make_command(17, sector >> 24, sector >> 16, sector >> 8, sector & 0xffu), response_buffer, 6);
    } else
    {
//        read_status(true);
        // todo can we expect support for 23?
        rc = sd_command(sd_make_command(23, sector_count >> 24, sector_count >> 16, sector_count >> 8, sector_count & 0xffu),
                        response_buffer, 6);
        if (!rc) rc = sd_command(sd_make_command(18, sector >> 24, sector >> 16, sector >> 8, sector & 0xffu), response_buffer, 6);
    }
    return rc;
}

#endif
