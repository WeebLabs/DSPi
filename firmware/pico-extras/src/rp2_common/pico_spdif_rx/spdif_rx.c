/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "spdif_rx.h"

#include <stdio.h>
#include <string.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "spdif_rx_capture.pio.h"
#include "spdif_rx_48000.pio.h"
#include "spdif_rx_96000.pio.h"
#include "spdif_rx_192000.pio.h"

// sample words to detect is equivalent to 64*4+8 symbols (2 frames + sync) at 44.1 KHz @ 125 MHz clock
// because at least one Sync M Code has to be included in sampled words (-> need 2 frames considering when head block hits)
#define SPDIF_RX_CAPTURE_SIZE (((SPDIF_RX_PIO_CLK_FREQ / SAMP_FREQ_44100 / 128 + 1) * (64 * 4 + 8) + 31) / 32)

#define spdif_rx_pio __CONCAT(pio, PICO_SPDIF_RX_PIO)
#define DREQ_PIOx_RX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_SPDIF_RX_PIO), _RX0)

#define DMA_IRQ_x __CONCAT(DMA_IRQ_, PICO_SPDIF_RX_DMA_IRQ)

#define NUM_AVE (8)
#define NUM_STABLE_FREQ (16) // 1 ~ 31

static spdif_rx_config_t gcfg;
static alarm_id_t alarm_id;

typedef enum _spdif_rx_pio_program_id_t  {
    SPDIF_RX_PIO_CAPTURE = 0,
    SPDIF_RX_PIO_DECODE_48000,
    SPDIF_RX_PIO_DECODE_96000,
    SPDIF_RX_PIO_DECODE_192000,
} spdif_rx_pio_program_id_t;

typedef struct _spdif_rx_pio_program_t {
    const spdif_rx_pio_program_id_t id;
    const pio_program_t* program;
    uint offset;
    uint entry_point;
    pio_sm_config (*get_default_config)(uint);
} spdif_rx_pio_program_t;

static spdif_rx_pio_program_t program_sets[] = {
    {SPDIF_RX_PIO_CAPTURE,           &spdif_rx_capture_program,    0, spdif_rx_capture_offset_entry_point,    spdif_rx_capture_program_get_default_config},
    {SPDIF_RX_PIO_DECODE_48000,      &spdif_rx_48000_program,      0, spdif_rx_48000_offset_entry_point,      spdif_rx_48000_program_get_default_config},
    {SPDIF_RX_PIO_DECODE_96000,      &spdif_rx_96000_program,      0, spdif_rx_96000_offset_entry_point,      spdif_rx_96000_program_get_default_config},
    {SPDIF_RX_PIO_DECODE_192000,     &spdif_rx_192000_program,     0, spdif_rx_192000_offset_entry_point,     spdif_rx_192000_program_get_default_config}
};

typedef enum _spdif_rx_samp_freq_id_t {
    SAMP_FREQ_ID_44100 = 0,
    SAMP_FREQ_ID_48000,
    SAMP_FREQ_ID_88200,
    SAMP_FREQ_ID_96000,
    SAMP_FREQ_ID_176400,
    SAMP_FREQ_ID_192000,
    _NUM_SAMP_FREQ_IDS_
} spdif_rx_samp_freq_id_t;

typedef struct _spdif_rx_samp_freq_info_set_t {
    const spdif_rx_samp_freq_id_t id;
    const spdif_rx_samp_freq_t sf;
    const int samp_freq_lower;
    const int samp_freq_upper;
    const int min_edge_interval_lower;
    const int min_edge_interval_upper;
    const int max_edge_interval_lower;
    const int max_edge_interval_upper;
} spdif_rx_samp_freq_info_set_t;

#define GEN_SF_INFO_SET(sf) \
    { \
        SAMP_FREQ_ID_##sf, \
        SAMP_FREQ_##sf, \
        SAMP_FREQ_##sf *  99 / 100, \
        SAMP_FREQ_##sf * 101 / 100, \
        (SPDIF_RX_PIO_CLK_FREQ / SAMP_FREQ_##sf * 2 / 128) *  91 / 100, \
        (SPDIF_RX_PIO_CLK_FREQ / SAMP_FREQ_##sf * 2 / 128) * 111 / 100, \
        (SPDIF_RX_PIO_CLK_FREQ / SAMP_FREQ_##sf * 6 / 128) *  97 / 100, \
        (SPDIF_RX_PIO_CLK_FREQ / SAMP_FREQ_##sf * 6 / 128) * 104 / 100 \
    }
// -9%, +11% is the minimum range which makes at least +- 1 cycles at 192000 Hz in 2 symbol length
// -3%, +4% is the maximum range which doesn't overlap between near-by frequencies in 6 symbol length

static const spdif_rx_samp_freq_info_set_t sf_info_sets[] = {
    GEN_SF_INFO_SET(44100),
    GEN_SF_INFO_SET(48000),
    GEN_SF_INFO_SET(88200),
    GEN_SF_INFO_SET(96000),
    GEN_SF_INFO_SET(176400),
    GEN_SF_INFO_SET(192000)
};

typedef enum _spdif_rx_samp_freq_criteria_check_t  {
    CHK_SAMP_FREQ = 0,
    CHK_SAMP_FREQ_LOWER_ONLY,
    CHK_SAMP_FREQ_UPPER_ONLY,
    CHK_MIN_EDGE_INTERVAL,
    CHK_MIN_EDGE_INTERVAL_LOWER_ONLY,
    CHK_MIN_EDGE_INTERVAL_UPPER_ONLY,
    CHK_MAX_EDGE_INTERVAL,
    CHK_MAX_EDGE_INTERVAL_LOWER_ONLY,
    CHK_MAX_EDGE_INTERVAL_UPPER_ONLY
} spdif_rx_samp_freq_criteria_check_t;

// Sync codes which appears the output of decode PIO program
static const uint32_t SYNC_B = 0b1111;
static const uint32_t SYNC_M = 0b1011;
static const uint32_t SYNC_W = 0b0111;

// timer parameters
static const uint32_t capture_retry_interval_ms = 100;
static const uint32_t capture_timeout_us        = 100 + 300; // us (300 us to give more time margin before cancelling timeout from spdif_rx_dma_irq_handler by _clear_timer() since sdk-2.0.0 corresponding to alarm_pool usage)
static const uint32_t decode_wait_stable_ms     = 200;
static const uint32_t decode_timeout_ms         = 10;

static spdif_rx_state_t state;

static void (*on_stable_func)(spdif_rx_samp_freq_t samp_freq) = NULL;
static void (*on_lost_stable_func)() = NULL;

static uint32_t fifo_buff[SPDIF_RX_FIFO_SIZE];
static uint32_t buff_wr_pre_ptr = 0;
static uint32_t buff_wr_done_ptr = 0;
static uint32_t buff_rd_ptr = 0;

static dma_channel_config dma_config0;
static dma_channel_config dma_config1;

static bool setup_done = false;
static bool stable_done = false;
static bool irq_handler_registered = false;  // DSPi patch: track our own handler registration
static spdif_rx_pio_program_t* current_pg;
static int block_count;
static uint64_t prev_time_us = 0;
static uint32_t waiting_start_time_ms = 0;
static uint32_t block_interval[NUM_AVE];
static bool block_aligned;
static int block_align_count;
static float samp_freq_actual;
static spdif_rx_samp_freq_t samp_freq;
static uint32_t stable_freq_history;
static bool stable_freq_flg;
static uint trans_count = SPDIF_BLOCK_SIZE;
static uint8_t c_bits[SPDIF_BLOCK_SIZE / 16];
static uint32_t parity_err_count;

// prototype declaration
static int64_t _spdif_rx_capture_retry(alarm_id_t /*id*/, void* /*user_data*/);
void __isr __time_critical_func(spdif_rx_dma_irq_handler)();

static inline uint64_t _micros(void)
{
    return to_us_since_boot(get_absolute_time());
}

static inline uint32_t _millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}

// === Internal functions ===

static inline void _spdif_rx_program_init(PIO pio, uint sm, uint offset, uint entry_point, pio_sm_config (*get_default_config)(uint), uint pin, bool inverted)
{
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
    pio_gpio_init(pio, pin);
    //gpio_pull_down(pin);
    if (inverted) {
        gpio_set_inover(pin, GPIO_OVERRIDE_INVERT);
    } else {
        gpio_set_inover(pin, GPIO_OVERRIDE_NORMAL);
    }

    pio_sm_config sm_config = (*get_default_config)(offset);

    sm_config_set_jmp_pin(&sm_config, pin);
    sm_config_set_in_pins(&sm_config, pin); // PINCTRL_IN_BASE for wait
    sm_config_set_in_shift(&sm_config, true, false, 32); // shift_right, no autopush, 32bit
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
    const uint32_t div_int   = SPDIF_RX_SYS_CLK_FREQ / SPDIF_RX_PIO_CLK_FREQ;
    const uint8_t  div_frac8 = (uint8_t) (((uint64_t) SPDIF_RX_SYS_CLK_FREQ * 256) / SPDIF_RX_PIO_CLK_FREQ - 256);
    sm_config_set_clkdiv_int_frac8(&sm_config, div_int, div_frac8);
    pio_sm_init(pio, sm, offset, &sm_config);
    pio_sm_set_pins(pio, sm, 0); // clear pins
    pio_sm_clear_fifos(pio, sm);
    pio_sm_drain_tx_fifo(pio, sm);

    // set OSR (use as fixed value)
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_exec(pio, sm, pio_encode_set(pio_x, 0x0)); // only for copying to osr
    pio_sm_exec(pio, sm, pio_encode_mov_not(pio_osr, pio_x)); // osr = 0xFFFFFFFF
    pio_sm_set_enabled(pio, sm, true);

    pio_sm_exec(pio, sm, pio_encode_jmp(offset + entry_point));
}

static inline uint32_t _ptr_inc(uint32_t ptr, uint32_t count)
{
    return (ptr + count) % (SPDIF_RX_FIFO_SIZE * 2);
}

static inline uint32_t* _to_buff_ptr(uint32_t ptr)
{
    return &fifo_buff[ptr % SPDIF_RX_FIFO_SIZE];
}

// default spdif_rx block callback function (you may override at external)
__attribute__((weak))
void spdif_rx_callback_func(uint32_t* buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err)
{
    (void) buff;
    (void) sub_frame_count;
    (void) c_bits;
    (void) parity_err;
    return;
}

static int _check_block(uint32_t buff[SPDIF_BLOCK_SIZE])
{
    uint pos_syncB = 0;
    uint32_t block_parity_err_count = 0;
    uint8_t c_bits_raw[SPDIF_BLOCK_SIZE / 16];
    uint8_t c_bit_pos = 0x1; // start with LSB
    uint8_t c_bits_byte = 0;

    for (int i = 0; i < SPDIF_BLOCK_SIZE; i++) {
        uint32_t sync = buff[i] & 0xf;
        if (sync == SYNC_B) {
            block_aligned = (i == 0);
            pos_syncB = i;
        }
        if (block_aligned) {
            if ((i % 2 == 0 && (sync != SYNC_B && sync != SYNC_M)) || (i % 2 != 0 && sync != SYNC_W)) {
                block_aligned = false;
                break;
            }
            if (gcfg.flags == SPDIF_RX_FLAGS_NONE) {
                return true;
            }
            // VUCP handling
            if (gcfg.flags & SPDIF_RX_FLAG_C_BITS) {
                // C bits (whole 192 bits)
                if (i % 2 == 0) { // using even sub frame of each block
                    if (buff[i] & (0x1 << 30)) {
                        c_bits_byte |= c_bit_pos;
                    }
                    if (c_bit_pos >> 7) { // top boundary bit of each byte
                        c_bits_raw[i / 16] = c_bits_byte;
                        c_bits_byte = 0;
                        c_bit_pos = 0x1; // restart with LSB
                    } else {
                        c_bit_pos <<= 1;
                    }
                }
            }
            if (gcfg.flags & SPDIF_RX_FLAG_CHECK_PARITY) {
                // Parity (28 bits of every sub frame)
                uint32_t v = buff[i] & 0xFFFFFFF0; // excluding sync
                // bithack by Compute parity of word with a multiply, faster than __builtin_parity(), suggested by IDC-Dragon
                v ^= v >> 1;
                v ^= v >> 2;
                v = (v & 0x11111111) * 0x11111111;
                if (v & (1 << 28)) { // parity is now in bit 28
                    block_parity_err_count++;
                }
            }
        }
    }
    parity_err_count += block_parity_err_count;
    // block align adjustment
    if (block_aligned) {
        memcpy(c_bits, c_bits_raw, sizeof(c_bits)); // copy what's been collected
        trans_count = SPDIF_BLOCK_SIZE;
        if (spdif_rx_get_samp_freq() != SAMP_FREQ_NONE) {
            if (gcfg.flags & SPDIF_RX_FLAG_CALLBACKS) {
                spdif_rx_callback_func(buff, trans_count, c_bits, block_parity_err_count > 0);
            }
        }
    } else {
        if (pos_syncB != 0 && block_align_count == 0) {
            // dispose pos_syncB to align because fifo_buff[SPDIF_BLOCK_SIZE-1] was (SPDIF_BLOCK_SIZE - pos_syncB)'th sub frame
            // it takes 3 blocks to align because coming 2 transfers already issued
            trans_count = (pos_syncB > SPDIF_BLOCK_SIZE / 2) ? pos_syncB : SPDIF_BLOCK_SIZE / 2; // workaround: small number of trans_count can make DMA overun
            block_align_count = 3;
        } else {
            trans_count = SPDIF_BLOCK_SIZE;
            if (block_align_count > 0) { block_align_count--; }
        }
    }

    return block_aligned;
}

static uint32_t _dma_done_and_restart(uint8_t dma_channel, dma_channel_config* dma_config)
{
    uint32_t done_ptr = buff_wr_done_ptr;
    if (spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
        if (spdif_rx_get_fifo_count() + SPDIF_BLOCK_SIZE > SPDIF_RX_FIFO_SIZE) {
            //printf("spdif_rx fifo overflow\n");
            buff_rd_ptr = _ptr_inc(buff_rd_ptr, SPDIF_BLOCK_SIZE); // dispose overflow data
        }
        buff_wr_done_ptr = _ptr_inc(done_ptr, SPDIF_BLOCK_SIZE);
    } else { // if status is not ready, fifo should be empty
        buff_wr_done_ptr = _ptr_inc(done_ptr, SPDIF_BLOCK_SIZE);
        buff_rd_ptr = buff_wr_done_ptr;
    }
    dma_channel_configure(
        dma_channel,
        dma_config,
        _to_buff_ptr(buff_wr_pre_ptr), // dest
        &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
        trans_count, // count
        false // trigger
    );
    buff_wr_pre_ptr = _ptr_inc(buff_wr_pre_ptr, SPDIF_BLOCK_SIZE);
    return done_ptr;
}

static spdif_rx_pio_program_id_t _spdif_rx_get_decode_pio_promgam_id(spdif_rx_samp_freq_t samp_freq)
{
    spdif_rx_pio_program_id_t pio_program_id;
    switch (samp_freq) {
        case SAMP_FREQ_44100: // fallthrough
        case SAMP_FREQ_48000:
            pio_program_id = SPDIF_RX_PIO_DECODE_48000;
            break;
        case SAMP_FREQ_88200: // fallthrough
        case SAMP_FREQ_96000:
            pio_program_id = SPDIF_RX_PIO_DECODE_96000;
            break;
        case SAMP_FREQ_176400: // fallthrough
        case SAMP_FREQ_192000:
            pio_program_id = SPDIF_RX_PIO_DECODE_192000;
            break;
        default:
            pio_program_id = SPDIF_RX_PIO_DECODE_48000;
            break;
    }
    return pio_program_id;
}

static inline void _clear_timer()
{
    alarm_pool_cancel_alarm(gcfg.alarm_pool, alarm_id);
}

static inline void _set_timer_after_by_us(alarm_callback_t callback, uint32_t after_us)
{
    _clear_timer();
    alarm_id = alarm_pool_add_alarm_in_us(gcfg.alarm_pool, (uint64_t) after_us, callback, NULL, false);
}

static inline void _set_timer_after_by_ms(alarm_callback_t callback, uint32_t after_ms)
{
    _clear_timer();
    alarm_id = alarm_pool_add_alarm_in_ms(gcfg.alarm_pool, after_ms, callback, NULL, false);
}

static void _spdif_rx_capture_start()
{
    // DMA0
    dma_channel_claim(gcfg.dma_channel0);
    dma_config0 = dma_channel_get_default_config(gcfg.dma_channel0);
    channel_config_set_transfer_data_size(&dma_config0, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config0, false);
    channel_config_set_write_increment(&dma_config0, true);
    channel_config_set_dreq(&dma_config0, DREQ_PIOx_RX0 + gcfg.pio_sm);
    channel_config_set_chain_to(&dma_config0, gcfg.dma_channel0);

    dma_channel_configure(
        gcfg.dma_channel0,
        &dma_config0,
        fifo_buff, // dest
        &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
        SPDIF_RX_CAPTURE_SIZE, // count
        false // trigger
    );

    // DMA IRQ
    // DSPi patch: use our own flag instead of irq_has_shared_handler() which
    // returns true if ANY handler is on this IRQ (e.g. SPDIF TX), preventing
    // our RX handler from ever being registered.
    if (!irq_handler_registered) {
        irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_handler_registered = true;
    }
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0, true);
    irq_set_enabled(DMA_IRQ_x, true);

    // Start DMA
    dma_channel_start(gcfg.dma_channel0);

    // === PIO configuration ===
    pio_sm_claim(spdif_rx_pio, gcfg.pio_sm);
    current_pg = &program_sets[SPDIF_RX_PIO_CAPTURE];
    current_pg->offset = pio_add_program(spdif_rx_pio, current_pg->program);
    spdif_rx_capture_program_init(
        spdif_rx_pio,
        gcfg.pio_sm,
        current_pg->offset,
        current_pg->entry_point,
        current_pg->get_default_config,
        gcfg.data_pin
    );
}

static void _spdif_rx_common_end()
{
    // DSPi patch: disable interrupts during teardown to prevent re-entrancy.
    // The DMA IRQ (highest priority) can preempt the timer alarm callback
    // (default priority), causing double-teardown of DMA/PIO resources which
    // panics on unclaim of already-unclaimed channels.
    uint32_t save = save_and_disable_interrupts();

    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0, false);
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1, false);
    // DSPi patch: removed irq_set_enabled(DMA_IRQ_x, false) — it disables the
    // ENTIRE DMA IRQ line, killing SPDIF TX output DMA handlers that share the
    // same IRQ. The per-channel disable above is sufficient.
    dma_channel_abort(gcfg.dma_channel0);
    dma_channel_wait_for_finish_blocking(gcfg.dma_channel0);
    dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0);
    if (dma_channel_is_claimed(gcfg.dma_channel0)) {
        dma_channel_cleanup(gcfg.dma_channel0);
        dma_channel_unclaim(gcfg.dma_channel0);
    }
    dma_channel_abort(gcfg.dma_channel1);
    dma_channel_wait_for_finish_blocking(gcfg.dma_channel1);
    dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1);
    if (dma_channel_is_claimed(gcfg.dma_channel1)) {
        dma_channel_cleanup(gcfg.dma_channel1);
        dma_channel_unclaim(gcfg.dma_channel1);
    }
    pio_sm_set_enabled(spdif_rx_pio, gcfg.pio_sm, false);
    pio_sm_clear_fifos(spdif_rx_pio, gcfg.pio_sm);
    pio_sm_drain_tx_fifo(spdif_rx_pio, gcfg.pio_sm);
    pio_remove_program(spdif_rx_pio, current_pg->program, current_pg->offset);
    // DSPi patch: removed pio_clear_instruction_memory() — it wipes ALL programs
    // on the PIO block, destroying PDM/MCK/SPDIF-TX programs on shared PIO blocks.
    if (pio_sm_is_claimed(spdif_rx_pio, gcfg.pio_sm)) {
        pio_sm_unclaim(spdif_rx_pio, gcfg.pio_sm);
    }

    restore_interrupts(save);
}

static int64_t _spdif_rx_capture_timeout(alarm_id_t id, void* user_data)
{
    (void) id;
    (void) user_data;

    // DSPi patch: disable interrupts for the entire callback to prevent the
    // DMA IRQ (higher priority) from preempting mid-teardown and causing
    // double resource claim/unclaim panics.
    uint32_t save = save_and_disable_interrupts();
    _clear_timer();
    _spdif_rx_common_end();
    _set_timer_after_by_ms(_spdif_rx_capture_retry, capture_retry_interval_ms);
    restore_interrupts(save);
    return 0;
}

static int64_t _spdif_rx_capture_retry(alarm_id_t id, void* user_data)
{
    (void) id;
    (void) user_data;

    uint32_t save = save_and_disable_interrupts();
    _clear_timer();
    _spdif_rx_capture_start();
    _set_timer_after_by_us(_spdif_rx_capture_timeout, capture_timeout_us);
    restore_interrupts(save);
    return 0;
}

static bool _spdif_rx_check_criteria(int samp_freq_id, int value, spdif_rx_samp_freq_criteria_check_t check_type)
{
    const spdif_rx_samp_freq_info_set_t* sf_info = &sf_info_sets[samp_freq_id];
    switch (check_type) {
        case CHK_SAMP_FREQ:
            return value >= sf_info->samp_freq_lower && value <= sf_info->samp_freq_upper;
        case CHK_SAMP_FREQ_LOWER_ONLY:
            return value >= sf_info->samp_freq_lower;
        case CHK_SAMP_FREQ_UPPER_ONLY:
            return value <= sf_info->samp_freq_upper;
        case CHK_MIN_EDGE_INTERVAL:
            return value >= sf_info->min_edge_interval_lower && value <= sf_info->min_edge_interval_upper;
        case CHK_MIN_EDGE_INTERVAL_LOWER_ONLY:
            return value >= sf_info->min_edge_interval_lower;
        case CHK_MIN_EDGE_INTERVAL_UPPER_ONLY:
            return value <= sf_info->min_edge_interval_upper;
        case CHK_MAX_EDGE_INTERVAL:
            return value >= sf_info->max_edge_interval_lower && value <= sf_info->max_edge_interval_upper;
        case CHK_MAX_EDGE_INTERVAL_LOWER_ONLY:
            return value >= sf_info->max_edge_interval_lower;
        case CHK_MAX_EDGE_INTERVAL_UPPER_ONLY:
            return value <= sf_info->max_edge_interval_upper;
        default:
            break;
    }
    return false;
}

static int _spdif_count_leading_zeros(uint32_t v)
{
    // to avoid the optimization issue, the separate wrapper function is needed for the care of the zero input case of __builtin_clz()
    return v == 0 ? 32 : __builtin_clz(v);
}

static int _spdif_rx_analyze_capture(spdif_rx_samp_freq_t* samp_freq, bool* inverted)
{
    // sampled data analysis to calculate min_edge_interval, max_edge_interval for both 0 and 1
    int edge_pos[2] = {0, 0}; // unknown
    int max_edge_interval[2] = {0, 0};
    int min_edge_interval = 0xFFFF;
    {
        //for (int i = 0; i < SPDIF_RX_CAPTURE_SIZE; i++) printf("data[%3d] = %032b\n", i, fifo_buff[i]);
        int pos = 0; // current bit position in buffer
        int cur = 1; // edge toggle, PIO sample started with a rising edge
        int num_check_words = SPDIF_RX_CAPTURE_SIZE;
        uint32_t shift_reg = fifo_buff[0];
        int bit_pos = 0; // position within current dword
        int word_idx = 0;
        while (true) {
            int r = _spdif_count_leading_zeros((cur) ? ~shift_reg : shift_reg); // leading zeros
            if (r + bit_pos <= 31) { // found within remaining part of 32bit dword?
                pos += r; // go to the found edge position
                cur = 1 - cur; // toggle to other edge index
                if (edge_pos[cur]) { // valid previous edge?
                    int distance = pos - edge_pos[cur];
                    if (max_edge_interval[cur] < distance) {
                        max_edge_interval[cur] = distance;
                    }
                    if (min_edge_interval > distance) {
                        min_edge_interval = distance;
                        // early termination by min_edge_interval (the higher frequency, the fewer samples for equivalent number of frames)
                        if (!_spdif_rx_check_criteria(SAMP_FREQ_ID_192000, min_edge_interval, CHK_MIN_EDGE_INTERVAL_LOWER_ONLY)) {
                            break; // no hope in this case, force termination
                        } else if (_spdif_rx_check_criteria(SAMP_FREQ_ID_176400, min_edge_interval, CHK_MIN_EDGE_INTERVAL_UPPER_ONLY)) {
                            num_check_words = SPDIF_RX_CAPTURE_SIZE / 4;
                        } else if (_spdif_rx_check_criteria(SAMP_FREQ_ID_88200, min_edge_interval, CHK_MIN_EDGE_INTERVAL_UPPER_ONLY)) {
                            num_check_words = SPDIF_RX_CAPTURE_SIZE / 2;
                        }
                    }
                }
                edge_pos[cur] = pos; // other edge
                shift_reg <<= r;
                bit_pos += r;
            } else { // otherwise go first bit in next 32bit
                pos += 32 - bit_pos;
                bit_pos = 0;
                word_idx++;
                if (word_idx >= num_check_words) { // end of analysis
                    break;
                }
                shift_reg = fifo_buff[word_idx];
            }
        }
        //printf("min_edge = %d, max_edge0 = %d, max_edge1 = %d\n", min_edge_interval, max_edge_interval[0], max_edge_interval[1]);

        if (word_idx < num_check_words) { // force termination
            return 0;
        }
    }

    // evaluate analysis result to confirm if it meets criteria of sampling frequencies
    // use the interval of same edges which is free from signal transition difference between rising and falling
    for (int sf_id = 0; sf_id < _NUM_SAMP_FREQ_IDS_; sf_id++) {
        // Judge frequency: thanks to great idea by IDC-Dragon
        // focusing on Sync Code M which appears in L sub-frame (except for head frame in the block)
        //                                                                            |<---->|
        // normal polarity case,   min interval of rising edge = 2  (0 -> 1 1 1 0 0 0 1 0 -> 1)
        // inverted polarity case, min interval of falling edge = 2 (1 -> 0 0 0 1 1 1 0 1 -> 0)
        // Sync Code B of head frame also has minimum edge interval, however it will not hit during 2 frames of capture period
        if (_spdif_rx_check_criteria(sf_id, min_edge_interval, CHK_MIN_EDGE_INTERVAL)) {
            // Judge polarity (+ frequency): thanks to great idea by IDC-Dragon
            // focusing on Sync Code M which appears in L sub-frame (except for head frame in the block)
            //                                                                      |<--------->|
            // if max interval of rising edge = 6,  then polarity is normal   (0 -> 1 1 1 0 0 0 1 0 -> 1)
            // if max interval of falling edge = 6, then polarity is inverted (1 -> 0 0 0 1 1 1 0 1 -> 0)
            if (_spdif_rx_check_criteria(sf_id, max_edge_interval[1], CHK_MAX_EDGE_INTERVAL)) { // normal bit stream
                *samp_freq = sf_info_sets[sf_id].sf;
                *inverted = false;
                return 1;
            } else if (_spdif_rx_check_criteria(sf_id, max_edge_interval[0], CHK_MAX_EDGE_INTERVAL)) { // inverted bit stream
                *samp_freq = sf_info_sets[sf_id].sf;
                *inverted = true;
                return 1;
            }
        }
    }
    return 0;
}

static int64_t _spdif_rx_decode_timeout(alarm_id_t id, void* user_data)
{
    (void) id;
    (void) user_data;

    // DSPi patch: disable interrupts for the entire callback (same reason
    // as _spdif_rx_capture_timeout — prevent DMA IRQ re-entrancy).
    uint32_t save = save_and_disable_interrupts();
    state = SPDIF_RX_STATE_NO_SIGNAL;
    if ((gcfg.flags & SPDIF_RX_FLAG_CALLBACKS) && on_lost_stable_func != NULL) {
        (*on_lost_stable_func)();
    }
    _clear_timer();
    _spdif_rx_common_end();
    _set_timer_after_by_ms(_spdif_rx_capture_retry, capture_retry_interval_ms);
    restore_interrupts(save);
    return 0;
}

static void _spdif_rx_decode_start(spdif_rx_samp_freq_t samp_freq, bool inverted)
{
    stable_done = false;
    buff_wr_pre_ptr = 0;
    buff_wr_done_ptr = 0;
    buff_rd_ptr = 0;
    block_count = 0;
    block_aligned = false;
    block_align_count = 0;
    stable_freq_history = 0;
    stable_freq_flg = false;
    trans_count = SPDIF_BLOCK_SIZE;
    memset(c_bits, 0, sizeof(c_bits));
    parity_err_count = 0;

    // === DMA configuration ===
    __mem_fence_release();
    dma_channel_claim(gcfg.dma_channel0);
    dma_channel_claim(gcfg.dma_channel1);

    // DMA0
    dma_config0 = dma_channel_get_default_config(gcfg.dma_channel0);
    channel_config_set_transfer_data_size(&dma_config0, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config0, false);
    channel_config_set_write_increment(&dma_config0, true);
    channel_config_set_dreq(&dma_config0, DREQ_PIOx_RX0 + gcfg.pio_sm);
    channel_config_set_chain_to(&dma_config0, gcfg.dma_channel1);

    dma_channel_configure(
        gcfg.dma_channel0,
        &dma_config0,
        _to_buff_ptr(buff_wr_pre_ptr), // dest
        &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
        SPDIF_BLOCK_SIZE, // count
        false // trigger
    );
    buff_wr_pre_ptr = _ptr_inc(buff_wr_pre_ptr, SPDIF_BLOCK_SIZE);

    // DMA1
    dma_config1 = dma_channel_get_default_config(gcfg.dma_channel1);
    channel_config_set_transfer_data_size(&dma_config1, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config1, false);
    channel_config_set_write_increment(&dma_config1, true);
    channel_config_set_dreq(&dma_config1, DREQ_PIOx_RX0 + gcfg.pio_sm);
    channel_config_set_chain_to(&dma_config1, gcfg.dma_channel0);

    dma_channel_configure(
        gcfg.dma_channel1,
        &dma_config1,
        _to_buff_ptr(buff_wr_pre_ptr), // dest
        &spdif_rx_pio->rxf[gcfg.pio_sm],  // src
        SPDIF_BLOCK_SIZE, // count
        false // trigger
    );
    buff_wr_pre_ptr = _ptr_inc(buff_wr_pre_ptr, SPDIF_BLOCK_SIZE);

    // DMA IRQ
    // DSPi patch: use our own flag instead of irq_has_shared_handler() which
    // returns true if ANY handler is on this IRQ (e.g. SPDIF TX), preventing
    // our RX handler from ever being registered.
    if (!irq_handler_registered) {
        irq_add_shared_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_handler_registered = true;
    }
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0, true);
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1, true);
    irq_set_enabled(DMA_IRQ_x, true);

    // Start DMA
    dma_channel_start(gcfg.dma_channel0);

    // === PIO configuration ===
    pio_sm_claim(spdif_rx_pio, gcfg.pio_sm);
    // PIO start
    spdif_rx_pio_program_id_t pio_program_id = _spdif_rx_get_decode_pio_promgam_id(samp_freq);
    current_pg = &program_sets[pio_program_id];
    current_pg->offset = pio_add_program(spdif_rx_pio, current_pg->program);
    _spdif_rx_program_init(
        spdif_rx_pio,
        gcfg.pio_sm,
        current_pg->offset,
        current_pg->entry_point,
        current_pg->get_default_config,
        gcfg.data_pin,
        inverted
    );
}

// irq handler for DMA
void __isr __time_critical_func(spdif_rx_dma_irq_handler)()
{
    _clear_timer(); // timeout must not happen while isr
    bool proc_dma0 = false;
    bool proc_dma1 = false;
    uint64_t now_us = _micros();
    uint32_t now_ms = _millis();
    if (dma_irqn_get_channel_status(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0)) {
        dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel0);
        proc_dma0 = true;
    } else if (dma_irqn_get_channel_status(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1)) {
        dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, gcfg.dma_channel1);
        proc_dma1 = true;
    }
    // state transition in capture operation case
    if (state == SPDIF_RX_STATE_NO_SIGNAL) {
        spdif_rx_samp_freq_t samp_freq;
        bool inverted;
        _spdif_rx_common_end();
        if (!_spdif_rx_analyze_capture(&samp_freq, &inverted)) {
            _set_timer_after_by_ms(_spdif_rx_capture_retry, capture_retry_interval_ms);
            return;
        }
        state = SPDIF_RX_STATE_WAITING_STABLE;
        _spdif_rx_decode_start(samp_freq, inverted);
        waiting_start_time_ms = _millis();
        setup_done = true;
        _set_timer_after_by_ms(_spdif_rx_decode_timeout, decode_timeout_ms);
        return;
    }
    // decode operation below here
    { // Calculate samp_freq and check if it's stable
        block_interval[block_count % NUM_AVE] = (uint32_t) (now_us - prev_time_us);
        uint32_t accum = 0;
        for (int i = 0; i < NUM_AVE; i++) {
            accum += block_interval[i];
        }
        float ave_block_interval = (float) accum / NUM_AVE;
        float bitrate_16b = (float) SPDIF_BLOCK_SIZE * 2 * 8 * 1e6 / ave_block_interval;
        samp_freq_actual = bitrate_16b / 32.0;
        spdif_rx_samp_freq_t sf = SAMP_FREQ_NONE;
        for (int sf_id = 0; sf_id < _NUM_SAMP_FREQ_IDS_; sf_id++) {
            if (_spdif_rx_check_criteria(sf_id, (int) (samp_freq_actual + 0.5), CHK_SAMP_FREQ)) {
                sf = sf_info_sets[sf_id].sf;
                break;
            }
        }
        stable_freq_history = (stable_freq_history << 1) | (sf != SAMP_FREQ_NONE && sf == samp_freq);
        uint32_t stable_mask = ((1ul << NUM_STABLE_FREQ) - 1);
        stable_freq_flg = (stable_freq_history & stable_mask) == stable_mask;
        if (setup_done && stable_freq_flg && block_aligned) {
            state = SPDIF_RX_STATE_STABLE;
        } else if (setup_done && stable_done && (!stable_freq_flg || !block_aligned)) {
            state = SPDIF_RX_STATE_NO_SIGNAL;
        }
        samp_freq = sf;
    }
    block_count++;
    if (proc_dma0) {
        uint32_t done_ptr = _dma_done_and_restart(gcfg.dma_channel0, &dma_config0);
        _check_block(_to_buff_ptr(done_ptr));
    } else if (proc_dma1) {
        uint32_t done_ptr = _dma_done_and_restart(gcfg.dma_channel1, &dma_config1);
        _check_block(_to_buff_ptr(done_ptr));
    }
    // state transition in decode operation case
    if (state == SPDIF_RX_STATE_STABLE) {
        if (!stable_done) {
            if ((gcfg.flags & SPDIF_RX_FLAG_CALLBACKS) && on_stable_func != NULL) {
                (*on_stable_func)(samp_freq);
            }
        }
        stable_done = true;
        _set_timer_after_by_ms(_spdif_rx_decode_timeout, decode_timeout_ms);
    } else if (state == SPDIF_RX_STATE_WAITING_STABLE && now_ms < waiting_start_time_ms + decode_wait_stable_ms) {
        _set_timer_after_by_ms(_spdif_rx_decode_timeout, decode_timeout_ms);
    } else {
        _spdif_rx_decode_timeout(-1, NULL); // call decode timeout target directly
    }
    prev_time_us = now_us;
}

// === Public functions ===

void spdif_rx_start(const spdif_rx_config_t* config)
{
    state = SPDIF_RX_STATE_NO_SIGNAL;
    memmove(&gcfg, config, sizeof(spdif_rx_config_t)); // copy to gcfg
    _spdif_rx_capture_retry(-1, NULL); // at first, call capture retry timeout target directly
}

void spdif_rx_end()
{
    _spdif_rx_common_end();
    // DSPi patch: remove our shared handler and reset flag so it gets
    // re-registered on next spdif_rx_start().
    if (irq_handler_registered) {
        irq_remove_handler(DMA_IRQ_x, spdif_rx_dma_irq_handler);
        irq_handler_registered = false;
    }
    state = SPDIF_RX_STATE_NO_SIGNAL;
}

void spdif_rx_set_callback_on_stable(void (*func)(spdif_rx_samp_freq_t samp_freq))
{
    on_stable_func = func;
}

void spdif_rx_set_callback_on_lost_stable(void (*func)())
{
    on_lost_stable_func = func;
}

spdif_rx_state_t spdif_rx_get_state()
{
    return state;
}

float spdif_rx_get_samp_freq_actual()
{
    return samp_freq_actual;
}

spdif_rx_samp_freq_t spdif_rx_get_samp_freq()
{
    return samp_freq;
}

void spdif_rx_get_c_bits(void* ptr, size_t size, uint32_t offset)
{
    uint32_t save = save_and_disable_interrupts(); // to avoid getting incomplete set of c_bits
    uint8_t* bptr = (uint8_t*) ptr;
    for (size_t i = 0; i < size; i++) {
        if (offset + i >= SPDIF_BLOCK_SIZE / 16) {
            break;
        }
        bptr[i] = c_bits[offset + i];
    }
    restore_interrupts(save);
}

uint32_t spdif_rx_get_parity_err_count()
{
    return parity_err_count;
}

uint32_t spdif_rx_get_fifo_count()
{
    uint32_t save = save_and_disable_interrupts();
    uint32_t fifo_count;
    if (buff_wr_done_ptr >= buff_rd_ptr) {
        fifo_count = buff_wr_done_ptr - buff_rd_ptr;
    } else {
        fifo_count = buff_wr_done_ptr + SPDIF_RX_FIFO_SIZE * 2 - buff_rd_ptr;
    }
    restore_interrupts(save);
    return fifo_count;
}

uint32_t spdif_rx_read_fifo(uint32_t** buff, uint32_t req_count)
{
    uint32_t save = save_and_disable_interrupts();
    uint32_t get_count = req_count;
    uint32_t fifo_count = spdif_rx_get_fifo_count();
    if (get_count > fifo_count) {
        get_count = fifo_count;
    }
    if ((buff_rd_ptr % SPDIF_RX_FIFO_SIZE) + get_count <= SPDIF_RX_FIFO_SIZE) { // cannot take due to the end of fifo_buff
        *buff = _to_buff_ptr(buff_rd_ptr);
        buff_rd_ptr = _ptr_inc(buff_rd_ptr, get_count);
    } else {
        get_count = SPDIF_RX_FIFO_SIZE - (buff_rd_ptr % SPDIF_RX_FIFO_SIZE);
        *buff = _to_buff_ptr(buff_rd_ptr);
        buff_rd_ptr = _ptr_inc(buff_rd_ptr, get_count);
    }
    restore_interrupts(save);
    return get_count;
}
