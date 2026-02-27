/*
 * S/PDIF Input Receiver — RP2350 only
 *
 * Always-running receiver: starts at boot, scans for signal continuously.
 * State machine: NO_SIGNAL → ACQUIRING → LOCKED
 * PIO2 SM0 captures/decodes; DMA CH6/CH7 double-buffered.
 * Architecture follows elehobica/pico_spdif_rx (reference only, not imported).
 */

#if PICO_RP2350

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "pico/audio_spdif.h"

#include "config.h"
#include "usb_audio.h"
#include "spdif_input.h"

// ---------------------------------------------------------------------------
// PIO Programs (embedded instruction arrays — compiled from PIO assembly)
// ---------------------------------------------------------------------------
//
// Capture program: 7 instructions. Samples pin as raw bitstream at PIO clock.
// Entry point waits for 32 edges (ensures signal present), then samples
// continuously via autopush.
//
// Decode programs: 21 instructions each. BMC decoder with parametric delay.
// Three rate families at 96 MHz PIO clock:
//   48k family (cy=16, lp=8):  44.1k / 48k
//   96k family (cy=8,  lp=4):  88.2k / 96k
//  192k family (cy=4,  lp=2): 176.4k / 192k
//
// Output format per 32-bit word:
//   [31] P (parity)  [30] C (channel status)  [29] U (user)  [28] V (validity)
//   [27:4] 24-bit audio (signed, MSB = bit 27)
//   [3:0]  Sync: B=0xF, M=0xB, W=0x7

// Capture program (wrap_target=6, wrap=6, entry=0)
static const uint16_t pio_capture_prog[] = {
    0xe03f,  // 0: set x, 31
    0x2020,  // 1: wait 0 pin 0
    0x20a0,  // 2: wait 1 pin 0
    0x0041,  // 3: jmp x-- 1
    0x2020,  // 4: wait 0 pin 0
    0x20a0,  // 5: wait 1 pin 0
    0x4001,  // 6: in pins, 1         .wrap_target / .wrap
};

// Decode program — 48k family (cy=16, lp=8) (wrap_target=1, wrap=11, entry=0)
static const uint16_t pio_decode_48k[] = {
    0x2020,  //  0: wait 0 pin 0                  entry_point
    0x2fa0,  //  1: wait 1 pin 0 [23]             wait1 (wrap_target)
    0x00c5,  //  2: jmp pin 5                     symbol_1x
    0x40e1,  //  3: in osr, 1                     (emit 1)
    0x0001,  //  4: jmp 1                         → wait1
    0xae42,  //  5: nop [14]                      symbol_11x
    0x00ce,  //  6: jmp pin 14                    → sync1110?
    0x4061,  //  7: in null, 1                    (emit 0)
    0x0d0a,  //  8: jmp 10 [13]                   → symbol_0x
    0x2f20,  //  9: wait 0 pin 0 [23]             wait0
    0x00cc,  // 10: jmp pin 12                    symbol_0x
    0x4061,  // 11: in null, 1                    (emit 0, wrap)
    0x40e1,  // 12: in osr, 1                     symbol_01
    0x0009,  // 13: jmp 9                         → wait0
    0x8020,  // 14: push block                    sync1110
    0x40e2,  // 15: in osr, 2                     (emit sync 11)
    0x2f20,  // 16: wait 0 pin 0 [23]
    0x0ed3,  // 17: jmp pin 19 [14]               → sync1xxx?
    0x000a,  // 18: jmp 10                        → symbol_0x
    0x40e2,  // 19: in osr, 2                     sync1xxx (emit sync 11)
    0x0000,  // 20: jmp 0                         → entry_point
};

// Decode program — 96k family (cy=8, lp=4) (same structure, different delays)
static const uint16_t pio_decode_96k[] = {
    0x2020, 0x2ba0, 0x00c5, 0x40e1, 0x0001, 0xa642, 0x00ce, 0x4061,
    0x050a, 0x2b20, 0x00cc, 0x4061, 0x40e1, 0x0009, 0x8020, 0x40e2,
    0x2b20, 0x06d3, 0x000a, 0x40e2, 0x0000,
};

// Decode program — 192k family (cy=4, lp=2)
static const uint16_t pio_decode_192k[] = {
    0x2020, 0x25a0, 0x00c5, 0x40e1, 0x0001, 0xa242, 0x00ce, 0x4061,
    0x010a, 0x2520, 0x00cc, 0x4061, 0x40e1, 0x0009, 0x8020, 0x40e2,
    0x2520, 0x02d3, 0x000a, 0x40e2, 0x0000,
};

// PIO program descriptors (pio_program_t for SDK pio_add_program)
static const pio_program_t capture_program = {
    .instructions = pio_capture_prog,
    .length = 7,
    .origin = -1,
};

static const pio_program_t decode_program_48k = {
    .instructions = pio_decode_48k,
    .length = 21,
    .origin = -1,
};

static const pio_program_t decode_program_96k = {
    .instructions = pio_decode_96k,
    .length = 21,
    .origin = -1,
};

static const pio_program_t decode_program_192k = {
    .instructions = pio_decode_192k,
    .length = 21,
    .origin = -1,
};

// Sync codes from decode PIO output [3:0]
#define SYNC_B  0x0F
#define SYNC_M  0x0B
#define SYNC_W  0x07

// ---------------------------------------------------------------------------
// Rate detection criteria (at 96 MHz PIO clock)
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t sample_rate;
    int min_edge_lower;   // minimum edge interval lower bound
    int min_edge_upper;   // minimum edge interval upper bound
    int max_edge_lower;   // max edge interval lower bound
    int max_edge_upper;   // max edge interval upper bound
    const pio_program_t *decode_prog;
} RateInfo;

// PIO clock = 96 MHz. Symbol period = PIO_CLK / (sample_rate * 128)
// Min edge interval = 2 symbols, Max edge interval = 6 symbols
// Tolerance: ±10% for min, ±5% for max
#define PIO_CLK 96000000

#define RATE_ENTRY(sr, prog) { \
    .sample_rate = sr, \
    .min_edge_lower = (int)((float)PIO_CLK / sr * 2 / 128 * 0.91f), \
    .min_edge_upper = (int)((float)PIO_CLK / sr * 2 / 128 * 1.11f), \
    .max_edge_lower = (int)((float)PIO_CLK / sr * 6 / 128 * 0.97f), \
    .max_edge_upper = (int)((float)PIO_CLK / sr * 6 / 128 * 1.04f), \
    .decode_prog = &prog, \
}

static const RateInfo rate_table[] = {
    RATE_ENTRY( 44100, decode_program_48k),
    RATE_ENTRY( 48000, decode_program_48k),
    RATE_ENTRY( 88200, decode_program_96k),
    RATE_ENTRY( 96000, decode_program_96k),
    RATE_ENTRY(176400, decode_program_192k),
    RATE_ENTRY(192000, decode_program_192k),
};
#define NUM_RATES (sizeof(rate_table) / sizeof(rate_table[0]))

// ---------------------------------------------------------------------------
// Capture buffer size
// ---------------------------------------------------------------------------

// Need at least 2 frames worth of data at the lowest rate (44.1k) for analysis.
// At 96 MHz, 44.1k has symbol period ~17 cycles. One frame = 64 symbols.
// 2 frames + sync = 2*64+8 = 136 symbols * ~17 cycles = ~2312 bits → ~73 words.
// Use 160 words for margin.
#define CAPTURE_SIZE  160

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static volatile SpdifInState rx_state = SPDIF_IN_NO_SIGNAL;
static volatile AudioSource current_source = AUDIO_SOURCE_USB;
static volatile uint32_t detected_rate = 0;
static volatile uint32_t parity_err_count = 0;
static volatile bool signal_inverted = false;

// C bits accumulation (24 bytes = 192 bits, we expose first 5)
static uint8_t c_bits_raw[24];

// FIFO ring buffer
static uint32_t fifo_buf[SPDIF_RX_FIFO_SIZE];
static volatile uint32_t fifo_wr_pre = 0;   // next DMA write target
static volatile uint32_t fifo_wr_done = 0;  // last completed DMA write
static volatile uint32_t fifo_rd = 0;       // consumer read pointer
#define FIFO_WRAP (SPDIF_RX_FIFO_SIZE * 2)

static inline uint32_t fifo_ptr_inc(uint32_t ptr, uint32_t count) {
    return (ptr + count) % FIFO_WRAP;
}
static inline uint32_t *fifo_to_buf(uint32_t ptr) {
    return &fifo_buf[ptr % SPDIF_RX_FIFO_SIZE];
}

// DMA configs
static dma_channel_config dma_cfg0, dma_cfg1;

// PIO state
static uint8_t rx_pin;
static PIO rx_pio;
static uint rx_sm;
static const pio_program_t *current_prog = NULL;
static uint current_prog_offset = 0;

// Timer/alarm state
static alarm_id_t rx_alarm_id = -1;

// Block validation
static bool block_aligned = false;
static int block_align_count = 0;
static uint32_t trans_count;

// Stability tracking
#define NUM_STABLE_REQUIRED 16
static uint32_t stable_freq_history = 0;
static bool stable_done = false;
static uint32_t waiting_start_ms = 0;
static uint32_t block_count = 0;
static uint64_t prev_block_time_us = 0;
static float actual_freq = 0.0f;
static uint32_t matched_rate = 0;

// PI servo state
static int32_t servo_pi_integral = 0;
static uint8_t sd_sub_accum = 0;            // sigma-delta sub-frac accumulator
static uint16_t servo_nominal_div_int = 0;  // PIO divider integer part
static uint32_t servo_nominal_fine = 0;     // nominal frac in fine units (frac << 8)
static volatile uint32_t servo_fine_target = 0; // current target (nominal ± correction)
static volatile bool servo_active = false;
static volatile bool tx_buffer_needed = false;

#define SERVO_KP_Q8          32   // proportional gain (Q8): 0.125 fine_units per sample-pair error
#define SERVO_KI               4   // integral gain: per update
#define SERVO_FIFO_TARGET     (SPDIF_RX_FIFO_SIZE / 2)  // 1536 subframes
#define SERVO_INTEGRAL_CLAMP  256  // ±167 ppm max integral
#define SERVO_HIGH_MARK       ((SPDIF_RX_FIFO_SIZE * 85) / 100)
#define SERVO_LOW_MARK        ((SPDIF_RX_FIFO_SIZE * 15) / 100)

// Hold concealment (last good samples)
static int32_t last_good_l = 0, last_good_r = 0;

// Deferred state transition flags (set in IRQ, handled in poll)
static volatile bool lock_pending = false;
static volatile bool lost_pending = false;
static volatile uint32_t lock_pending_rate = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

static void capture_start(void);
static void decode_start(uint32_t rate, bool inverted);
static void common_end(void);
static void __isr __time_critical_func(rx_dma_irq_handler)(void);
static int64_t capture_retry_cb(alarm_id_t id, void *data);
static int64_t decode_timeout_cb(alarm_id_t id, void *data);

// External: SPDIF TX instances for clock feedback
extern audio_spdif_instance_t *spdif_instance_ptrs[];

// External: process_audio_packet trigger
extern void process_audio_packet(const uint8_t *data, uint16_t data_len);

// ---------------------------------------------------------------------------
// PIO clock divider helper
// ---------------------------------------------------------------------------

static void rx_set_pio_clkdiv(void) {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t div_int = sys_clk / PIO_CLK;
    uint8_t div_frac = (uint8_t)(((uint64_t)sys_clk * 256) / PIO_CLK - div_int * 256);
    pio_sm_set_clkdiv_int_frac(rx_pio, rx_sm, div_int, div_frac);
}

// ---------------------------------------------------------------------------
// Timer helpers
// ---------------------------------------------------------------------------

static void clear_alarm(void) {
    if (rx_alarm_id >= 0) {
        cancel_alarm(rx_alarm_id);
        rx_alarm_id = -1;
    }
}

static void set_alarm_ms(alarm_callback_t cb, uint32_t ms) {
    clear_alarm();
    rx_alarm_id = add_alarm_in_ms(ms, cb, NULL, false);
}

static void set_alarm_us(alarm_callback_t cb, uint32_t us) {
    clear_alarm();
    rx_alarm_id = add_alarm_in_us(us, cb, NULL, false);
}

// ---------------------------------------------------------------------------
// PIO program management
// ---------------------------------------------------------------------------

static void load_program(const pio_program_t *prog) {
    if (current_prog) {
        pio_remove_program(rx_pio, current_prog, current_prog_offset);
        current_prog = NULL;
    }
    current_prog_offset = pio_add_program(rx_pio, prog);
    current_prog = prog;
}

// Configure SM for capture mode
static void sm_init_capture(void) {
    load_program(&capture_program);

    pio_sm_set_consecutive_pindirs(rx_pio, rx_sm, rx_pin, 1, false);
    pio_gpio_init(rx_pio, rx_pin);
    gpio_set_inover(rx_pin, GPIO_OVERRIDE_NORMAL);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, current_prog_offset + 6, current_prog_offset + 6);
    rx_set_pio_clkdiv();
    sm_config_set_clkdiv_int_frac(&c,
        clock_get_hz(clk_sys) / PIO_CLK,
        (uint8_t)(((uint64_t)clock_get_hz(clk_sys) * 256) / PIO_CLK
                  - (clock_get_hz(clk_sys) / PIO_CLK) * 256));
    sm_config_set_jmp_pin(&c, rx_pin);
    sm_config_set_in_pins(&c, rx_pin);
    sm_config_set_in_shift(&c, false, true, 32);  // shift_left, autopush, 32-bit
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    pio_sm_init(rx_pio, rx_sm, current_prog_offset, &c);
    pio_sm_set_pins(rx_pio, rx_sm, 0);
    pio_sm_clear_fifos(rx_pio, rx_sm);
    pio_sm_set_enabled(rx_pio, rx_sm, true);
    // Jump to entry point
    pio_sm_exec(rx_pio, rx_sm, pio_encode_jmp(current_prog_offset + 0));
}

// Configure SM for decode mode
static void sm_init_decode(const pio_program_t *prog, bool inverted) {
    load_program(prog);

    pio_sm_set_consecutive_pindirs(rx_pio, rx_sm, rx_pin, 1, false);
    pio_gpio_init(rx_pio, rx_pin);
    gpio_set_inover(rx_pin, inverted ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, current_prog_offset + 1, current_prog_offset + 11);
    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t div_int = sys_clk / PIO_CLK;
    uint8_t div_frac = (uint8_t)(((uint64_t)sys_clk * 256) / PIO_CLK - div_int * 256);
    sm_config_set_clkdiv_int_frac(&c, div_int, div_frac);
    sm_config_set_jmp_pin(&c, rx_pin);
    sm_config_set_in_pins(&c, rx_pin);
    sm_config_set_in_shift(&c, true, false, 32);  // shift_right, no autopush, 32-bit
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    pio_sm_init(rx_pio, rx_sm, current_prog_offset, &c);
    pio_sm_set_pins(rx_pio, rx_sm, 0);
    pio_sm_clear_fifos(rx_pio, rx_sm);
    pio_sm_drain_tx_fifo(rx_pio, rx_sm);

    // Pre-load OSR with 0xFFFFFFFF (used by decode program for emitting 1s)
    pio_sm_set_enabled(rx_pio, rx_sm, false);
    pio_sm_exec(rx_pio, rx_sm, 0xe020);  // set x, 0
    pio_sm_exec(rx_pio, rx_sm, 0xa0e9);  // mov osr, ~x  (osr = 0xFFFFFFFF)
    pio_sm_set_enabled(rx_pio, rx_sm, true);

    // Jump to entry point
    pio_sm_exec(rx_pio, rx_sm, pio_encode_jmp(current_prog_offset + 0));
}

// ---------------------------------------------------------------------------
// Capture phase (rate detection)
// ---------------------------------------------------------------------------

static void capture_start(void) {
    // DMA channel 0 only — single buffer capture
    dma_cfg0 = dma_channel_get_default_config(SPDIF_RX_DMA_CH0);
    channel_config_set_transfer_data_size(&dma_cfg0, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg0, false);
    channel_config_set_write_increment(&dma_cfg0, true);
    channel_config_set_dreq(&dma_cfg0, pio_get_dreq(rx_pio, rx_sm, false));
    channel_config_set_chain_to(&dma_cfg0, SPDIF_RX_DMA_CH0);  // self-chain to stop

    dma_channel_configure(SPDIF_RX_DMA_CH0, &dma_cfg0,
                          fifo_buf,                       // dest
                          &rx_pio->rxf[rx_sm],            // src
                          CAPTURE_SIZE,                    // count
                          false);                          // don't start yet

    // Enable IRQ for channel 0
    dma_irqn_set_channel_enabled(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH0, true);
    irq_set_enabled(DMA_IRQ_0 + SPDIF_RX_DMA_IRQ, true);
    dma_channel_start(SPDIF_RX_DMA_CH0);

    // Start capture PIO
    sm_init_capture();

    // Timeout: if no signal detected within 200us, retry
    set_alarm_us(decode_timeout_cb, 500);
}

static int _clz(uint32_t v) {
    return v == 0 ? 32 : __builtin_clz(v);
}

// Analyze captured bitstream to determine rate and polarity
static bool analyze_capture(uint32_t *samp_rate, bool *inverted) {
    int edge_pos[2] = {0, 0};
    int max_edge[2] = {0, 0};
    int min_edge = 0xFFFF;
    int cur = 1;  // PIO capture starts after rising edge
    int pos = 0;
    int word_idx = 0;
    uint32_t shift_reg = fifo_buf[0];
    int bit_pos = 0;

    while (word_idx < CAPTURE_SIZE) {
        int r = _clz(cur ? ~shift_reg : shift_reg);
        if (r + bit_pos <= 31) {
            pos += r;
            cur = 1 - cur;
            if (edge_pos[cur]) {
                int dist = pos - edge_pos[cur];
                if (max_edge[cur] < dist) max_edge[cur] = dist;
                if (dist < min_edge) min_edge = dist;
            }
            edge_pos[cur] = pos;
            shift_reg <<= r;
            bit_pos += r;
        } else {
            pos += 32 - bit_pos;
            bit_pos = 0;
            word_idx++;
            if (word_idx < CAPTURE_SIZE) shift_reg = fifo_buf[word_idx];
        }
    }

    if (min_edge == 0xFFFF) return false;

    // Match against rate table
    for (int i = 0; i < (int)NUM_RATES; i++) {
        if (min_edge >= rate_table[i].min_edge_lower &&
            min_edge <= rate_table[i].min_edge_upper) {
            // Check polarity via max edge interval
            if (max_edge[1] >= rate_table[i].max_edge_lower &&
                max_edge[1] <= rate_table[i].max_edge_upper) {
                *samp_rate = rate_table[i].sample_rate;
                *inverted = false;
                return true;
            } else if (max_edge[0] >= rate_table[i].max_edge_lower &&
                       max_edge[0] <= rate_table[i].max_edge_upper) {
                *samp_rate = rate_table[i].sample_rate;
                *inverted = true;
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Decode phase
// ---------------------------------------------------------------------------

static const pio_program_t *get_decode_prog(uint32_t rate) {
    for (int i = 0; i < (int)NUM_RATES; i++) {
        if (rate_table[i].sample_rate == rate)
            return rate_table[i].decode_prog;
    }
    return &decode_program_48k;
}

static void decode_start(uint32_t rate, bool inverted) {
    const pio_program_t *prog = get_decode_prog(rate);

    // Reset FIFO state
    fifo_wr_pre = 0;
    fifo_wr_done = 0;
    fifo_rd = 0;
    block_count = 0;
    block_aligned = false;
    block_align_count = 0;
    stable_freq_history = 0;
    stable_done = false;
    trans_count = SPDIF_BLOCK_SUBFRAMES;
    memset(c_bits_raw, 0, sizeof(c_bits_raw));
    parity_err_count = 0;
    prev_block_time_us = 0;
    matched_rate = rate;

    // Configure double-buffered DMA (ping-pong: CH0 chains to CH1, CH1 chains to CH0)
    dma_cfg0 = dma_channel_get_default_config(SPDIF_RX_DMA_CH0);
    channel_config_set_transfer_data_size(&dma_cfg0, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg0, false);
    channel_config_set_write_increment(&dma_cfg0, true);
    channel_config_set_dreq(&dma_cfg0, pio_get_dreq(rx_pio, rx_sm, false));
    channel_config_set_chain_to(&dma_cfg0, SPDIF_RX_DMA_CH1);

    dma_channel_configure(SPDIF_RX_DMA_CH0, &dma_cfg0,
                          fifo_to_buf(fifo_wr_pre), &rx_pio->rxf[rx_sm],
                          SPDIF_BLOCK_SUBFRAMES, false);
    fifo_wr_pre = fifo_ptr_inc(fifo_wr_pre, SPDIF_BLOCK_SUBFRAMES);

    dma_cfg1 = dma_channel_get_default_config(SPDIF_RX_DMA_CH1);
    channel_config_set_transfer_data_size(&dma_cfg1, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg1, false);
    channel_config_set_write_increment(&dma_cfg1, true);
    channel_config_set_dreq(&dma_cfg1, pio_get_dreq(rx_pio, rx_sm, false));
    channel_config_set_chain_to(&dma_cfg1, SPDIF_RX_DMA_CH0);

    dma_channel_configure(SPDIF_RX_DMA_CH1, &dma_cfg1,
                          fifo_to_buf(fifo_wr_pre), &rx_pio->rxf[rx_sm],
                          SPDIF_BLOCK_SUBFRAMES, false);
    fifo_wr_pre = fifo_ptr_inc(fifo_wr_pre, SPDIF_BLOCK_SUBFRAMES);

    // Enable IRQs for both channels
    dma_irqn_set_channel_enabled(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH0, true);
    dma_irqn_set_channel_enabled(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH1, true);
    irq_set_enabled(DMA_IRQ_0 + SPDIF_RX_DMA_IRQ, true);

    // Start first DMA transfer
    dma_channel_start(SPDIF_RX_DMA_CH0);

    // Start decode PIO
    sm_init_decode(prog, inverted);

    waiting_start_ms = to_ms_since_boot(get_absolute_time());
    set_alarm_ms(decode_timeout_cb, 10);
}

static void common_end(void) {
    // Stop DMA
    dma_irqn_set_channel_enabled(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH0, false);
    dma_irqn_set_channel_enabled(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH1, false);
    dma_channel_abort(SPDIF_RX_DMA_CH0);
    dma_channel_abort(SPDIF_RX_DMA_CH1);
    dma_channel_wait_for_finish_blocking(SPDIF_RX_DMA_CH0);
    dma_channel_wait_for_finish_blocking(SPDIF_RX_DMA_CH1);
    dma_irqn_acknowledge_channel(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH0);
    dma_irqn_acknowledge_channel(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH1);

    // Stop PIO
    pio_sm_set_enabled(rx_pio, rx_sm, false);
    pio_sm_clear_fifos(rx_pio, rx_sm);

    // Remove program
    if (current_prog) {
        pio_remove_program(rx_pio, current_prog, current_prog_offset);
        current_prog = NULL;
    }
}

// ---------------------------------------------------------------------------
// Block validation (called from DMA IRQ for each completed block)
// ---------------------------------------------------------------------------

static bool check_block(uint32_t *buf) {
    uint32_t pos_syncB = 0;
    uint32_t block_parity_errs = 0;
    uint8_t c_bit_pos = 0x01;
    uint8_t c_bits_byte = 0;
    uint8_t c_bits_tmp[24];

    for (int i = 0; i < (int)SPDIF_BLOCK_SUBFRAMES; i++) {
        uint32_t sync = buf[i] & 0x0F;
        if (sync == SYNC_B) {
            block_aligned = (i == 0);
            pos_syncB = i;
        }
        if (!block_aligned) continue;

        // Validate sync pattern
        if ((i % 2 == 0 && sync != SYNC_B && sync != SYNC_M) ||
            (i % 2 != 0 && sync != SYNC_W)) {
            block_aligned = false;
            break;
        }

        // C bits (from even subframes = channel A)
        if (i % 2 == 0) {
            if (buf[i] & (1u << 30)) c_bits_byte |= c_bit_pos;
            if (c_bit_pos >> 7) {
                c_bits_tmp[i / 16] = c_bits_byte;
                c_bits_byte = 0;
                c_bit_pos = 0x01;
            } else {
                c_bit_pos <<= 1;
            }
        }

        // Parity check (bits 4-31)
        uint32_t v = buf[i] & 0xFFFFFFF0;
        v ^= v >> 1;
        v ^= v >> 2;
        v = (v & 0x11111111) * 0x11111111;
        if (v & (1u << 28)) block_parity_errs++;
    }

    parity_err_count += block_parity_errs;

    if (block_aligned) {
        memcpy(c_bits_raw, c_bits_tmp, sizeof(c_bits_raw));
        trans_count = SPDIF_BLOCK_SUBFRAMES;
    } else {
        // Attempt block alignment
        if (pos_syncB != 0 && block_align_count == 0) {
            trans_count = (pos_syncB > SPDIF_BLOCK_SUBFRAMES / 2)
                        ? pos_syncB : SPDIF_BLOCK_SUBFRAMES / 2;
            block_align_count = 3;
        } else {
            trans_count = SPDIF_BLOCK_SUBFRAMES;
            if (block_align_count > 0) block_align_count--;
        }
    }
    return block_aligned;
}

// DMA done: restart DMA for next block, return pointer to completed block
static uint32_t dma_done_restart(uint8_t ch, dma_channel_config *cfg) {
    uint32_t done_ptr = fifo_wr_done;

    // Advance FIFO write pointer
    if (rx_state == SPDIF_IN_LOCKED) {
        // If FIFO full, advance read pointer (discard oldest)
        if (spdif_input_get_fifo_count() + SPDIF_BLOCK_SUBFRAMES > SPDIF_RX_FIFO_SIZE) {
            fifo_rd = fifo_ptr_inc(fifo_rd, SPDIF_BLOCK_SUBFRAMES);
        }
    }
    fifo_wr_done = fifo_ptr_inc(done_ptr, SPDIF_BLOCK_SUBFRAMES);
    if (rx_state != SPDIF_IN_LOCKED) {
        // When not locked, keep FIFO empty
        fifo_rd = fifo_wr_done;
    }

    // Configure next DMA transfer
    dma_channel_configure(ch, cfg,
                          fifo_to_buf(fifo_wr_pre), &rx_pio->rxf[rx_sm],
                          trans_count, false);
    fifo_wr_pre = fifo_ptr_inc(fifo_wr_pre, SPDIF_BLOCK_SUBFRAMES);

    return done_ptr;
}

// ---------------------------------------------------------------------------
// Timer callbacks
// ---------------------------------------------------------------------------

static int64_t capture_retry_cb(alarm_id_t id, void *data) {
    (void)id; (void)data;
    clear_alarm();
    capture_start();
    return 0;
}

static int64_t decode_timeout_cb(alarm_id_t id, void *data) {
    (void)id; (void)data;

    if (rx_state == SPDIF_IN_LOCKED) {
        // Signal lost while locked
        lost_pending = true;
    }

    rx_state = SPDIF_IN_NO_SIGNAL;
    detected_rate = 0;
    actual_freq = 0;
    stable_done = false;

    clear_alarm();
    common_end();
    set_alarm_ms(capture_retry_cb, 100);
    return 0;
}

// ---------------------------------------------------------------------------
// DMA IRQ handler
// ---------------------------------------------------------------------------

static void __isr __time_critical_func(rx_dma_irq_handler)(void) {
    clear_alarm();

    bool ch0_done = dma_irqn_get_channel_status(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH0);
    bool ch1_done = dma_irqn_get_channel_status(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH1);

    if (ch0_done) dma_irqn_acknowledge_channel(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH0);
    if (ch1_done) dma_irqn_acknowledge_channel(SPDIF_RX_DMA_IRQ, SPDIF_RX_DMA_CH1);

    if (!ch0_done && !ch1_done) return;

    // === Capture phase: analyze and transition to decode ===
    if (rx_state == SPDIF_IN_NO_SIGNAL) {
        uint32_t rate;
        bool inv;
        common_end();
        if (!analyze_capture(&rate, &inv)) {
            set_alarm_ms(capture_retry_cb, 100);
            return;
        }
        rx_state = SPDIF_IN_ACQUIRING;
        detected_rate = rate;
        signal_inverted = inv;
        decode_start(rate, inv);
        return;
    }

    // === Decode phase: validate blocks, track stability ===
    uint64_t now_us = to_us_since_boot(get_absolute_time());
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // Measure block interval for rate estimation
    if (prev_block_time_us > 0) {
        uint32_t interval_us = (uint32_t)(now_us - prev_block_time_us);
        if (interval_us > 0 && interval_us < 100000) {
            // bitrate = SPDIF_BLOCK_SUBFRAMES * 2 * 8 / interval (bits/sec, 16-bit symbols)
            float bitrate = (float)SPDIF_BLOCK_SUBFRAMES * 2.0f * 8.0f * 1e6f / (float)interval_us;
            actual_freq = bitrate / 32.0f;

            // Check if measured frequency matches detected rate (±1%)
            bool freq_ok = (actual_freq >= matched_rate * 0.99f &&
                           actual_freq <= matched_rate * 1.01f);
            stable_freq_history = (stable_freq_history << 1) | (freq_ok ? 1 : 0);
        }
    }
    prev_block_time_us = now_us;

    // Process completed block
    if (ch0_done) {
        uint32_t ptr = dma_done_restart(SPDIF_RX_DMA_CH0, &dma_cfg0);
        check_block(fifo_to_buf(ptr));
    } else if (ch1_done) {
        uint32_t ptr = dma_done_restart(SPDIF_RX_DMA_CH1, &dma_cfg1);
        check_block(fifo_to_buf(ptr));
    }
    block_count++;

    // Check stability
    uint32_t stable_mask = (1u << NUM_STABLE_REQUIRED) - 1;
    bool freq_stable = (stable_freq_history & stable_mask) == stable_mask;

    if (rx_state == SPDIF_IN_ACQUIRING) {
        if (freq_stable && block_aligned) {
            rx_state = SPDIF_IN_LOCKED;
            detected_rate = matched_rate;
            stable_done = true;
            lock_pending = true;
            lock_pending_rate = matched_rate;
        } else if (now_ms > waiting_start_ms + 200) {
            // Timeout: failed to lock
            decode_timeout_cb(-1, NULL);
            return;
        }
    } else if (rx_state == SPDIF_IN_LOCKED) {
        if (!freq_stable || !block_aligned) {
            // Lost lock
            decode_timeout_cb(-1, NULL);
            return;
        }
    }

    // Reset watchdog
    set_alarm_ms(decode_timeout_cb, 100);
}

// ---------------------------------------------------------------------------
// PI Servo + Sigma-Delta Dithering
// ---------------------------------------------------------------------------

static void compute_servo_nominal(uint32_t sample_rate) {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t divider_raw = sys_clk / sample_rate +
                           (sys_clk % sample_rate != 0 ? 1 : 0);
    servo_nominal_div_int = (uint16_t)(divider_raw >> 8);
    uint8_t nominal_frac = (uint8_t)(divider_raw & 0xFF);
    servo_nominal_fine = (uint32_t)nominal_frac << 8;
    servo_fine_target = servo_nominal_fine;
    servo_pi_integral = 0;
    sd_sub_accum = 0;
}

// Sigma-delta dithered TX PIO divider update (runs in DMA IRQ context)
static void __time_critical_func(apply_tx_clkdiv_sd)(void) {
    uint32_t ft = servo_fine_target;  // snapshot volatile
    uint8_t base_frac = (uint8_t)(ft >> 8);
    uint8_t sub_frac  = (uint8_t)(ft & 0xFF);
    uint8_t prev = sd_sub_accum;
    sd_sub_accum += sub_frac;
    uint8_t carry = (sd_sub_accum < prev) ? 1 : 0;
    uint8_t frac_out = base_frac + carry;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (spdif_instance_ptrs[i])
            pio_sm_set_clkdiv_int_frac(spdif_instance_ptrs[i]->pio,
                                        spdif_instance_ptrs[i]->pio_sm,
                                        servo_nominal_div_int, frac_out);
    }
}

// DMA completion callback — sets flag for main loop (runs in DMA IRQ context)
static void __isr __time_critical_func(tx_dma_done_handler)(void) {
    if (servo_active) apply_tx_clkdiv_sd();
    tx_buffer_needed = true;
    __sev();
}

// PI controller — called from main loop when tx_buffer_needed is set
void spdif_input_servo_update(void) {
    if (!servo_active || rx_state != SPDIF_IN_LOCKED) return;

    uint32_t level = spdif_input_get_fifo_count();

    // Emergency backstop: sample-slip at extreme thresholds
    if (level > SERVO_HIGH_MARK) {
        fifo_rd = fifo_ptr_inc(fifo_rd, 2);
    } else if (level < SERVO_LOW_MARK && level >= 2) {
        fifo_rd = (fifo_rd + FIFO_WRAP - 2) % FIFO_WRAP;
    }

    int32_t error_pairs = ((int32_t)level - (int32_t)SERVO_FIFO_TARGET) / 2;

    int32_t p_term = (SERVO_KP_Q8 * error_pairs) >> 8;

    servo_pi_integral += SERVO_KI * error_pairs;
    if (servo_pi_integral >  SERVO_INTEGRAL_CLAMP) servo_pi_integral =  SERVO_INTEGRAL_CLAMP;
    if (servo_pi_integral < -SERVO_INTEGRAL_CLAMP) servo_pi_integral = -SERVO_INTEGRAL_CLAMP;

    int32_t correction = p_term + servo_pi_integral;
    // FIFO overfull → positive correction → decrease divider → speed up TX
    int32_t new_fine = (int32_t)servo_nominal_fine - correction;
    if (new_fine < 0) new_fine = 0;
    if (new_fine > 65535) new_fine = 65535;
    servo_fine_target = (uint32_t)new_fine;
}

bool spdif_input_is_pull_active(void) {
    return servo_active;
}

bool spdif_input_pull_process_needed(void) {
    if (tx_buffer_needed) {
        tx_buffer_needed = false;
        return true;
    }
    return false;
}

void spdif_input_recompute_servo(uint32_t sample_rate) {
    if (servo_active) compute_servo_nominal(sample_rate);
}

// Restore TX PIO dividers to nominal (used when switching back to USB)
static void restore_tx_nominal_divider(void) {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    // Recalculate from current audio frequency
    extern volatile AudioState audio_state;
    uint32_t divider_raw = sys_clk / audio_state.freq +
                           (sys_clk % audio_state.freq != 0 ? 1 : 0);
    uint16_t div_int = (uint16_t)(divider_raw >> 8);
    uint8_t div_frac = (uint8_t)(divider_raw & 0xFF);
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (spdif_instance_ptrs[i])
            pio_sm_set_clkdiv_int_frac(spdif_instance_ptrs[i]->pio,
                                        spdif_instance_ptrs[i]->pio_sm,
                                        div_int, div_frac);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void spdif_input_init(uint8_t data_pin) {
    rx_pin = data_pin;
    rx_pio = pio2;
    rx_sm = SPDIF_RX_SM_CAPTURE;  // SM0

    // Pull-down prevents noise on floating RX pin (no cable connected)
    gpio_init(rx_pin);
    gpio_pull_down(rx_pin);

    // Claim SM and DMA channels
    pio_sm_claim(rx_pio, rx_sm);
    dma_channel_claim(SPDIF_RX_DMA_CH0);
    dma_channel_claim(SPDIF_RX_DMA_CH1);

    // Install DMA IRQ handler
    irq_add_shared_handler(DMA_IRQ_0 + SPDIF_RX_DMA_IRQ, rx_dma_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    // Start scanning immediately
    rx_state = SPDIF_IN_NO_SIGNAL;
    capture_start();
}

void spdif_input_poll(void) {
    // Handle deferred lock notification
    if (lock_pending) {
        lock_pending = false;
        // When SPDIF source is active and rate changes, reconfigure DSP
        if (current_source == AUDIO_SOURCE_SPDIF) {
            extern volatile bool rate_change_pending;
            extern volatile uint32_t pending_rate;
            extern volatile AudioState audio_state;
            if (audio_state.freq != lock_pending_rate) {
                pending_rate = lock_pending_rate;
                rate_change_pending = true;
            }
        }
    }

    // Handle deferred signal loss
    if (lost_pending) {
        lost_pending = false;
        // If we were on SPDIF input, mute has already been handled by
        // process_audio_packet seeing no data in the FIFO
    }
}

SpdifInState spdif_input_get_state(void) {
    return rx_state;
}

uint32_t spdif_input_get_sample_rate(void) {
    return detected_rate;
}

void spdif_input_get_status(SpdifInStatus *out) {
    out->state = (uint32_t)rx_state;
    out->sample_rate = detected_rate;
    out->parity_err_count = parity_err_count;
    memcpy(out->c_bits, c_bits_raw, 5);
    memset(out->_pad, 0, 3);
}

uint32_t spdif_input_get_fifo_count(void) {
    uint32_t wr = fifo_wr_done;
    uint32_t rd = fifo_rd;
    return (wr - rd + FIFO_WRAP) % FIFO_WRAP;
}

uint32_t spdif_input_read_samples(float *buf_l, float *buf_r, uint32_t max_samples) {
    uint32_t avail = spdif_input_get_fifo_count();
    // Need 2 subframes per stereo sample (L + R)
    uint32_t pairs = avail / 2;
    if (pairs > max_samples) pairs = max_samples;
    if (pairs == 0) return 0;

    const float inv_scale = 1.0f / 8388608.0f;

    for (uint32_t i = 0; i < pairs; i++) {
        uint32_t *p = fifo_to_buf(fifo_rd);
        uint32_t sf_l = p[0];  // Left subframe
        uint32_t sf_r = p[1];  // Right subframe
        fifo_rd = fifo_ptr_inc(fifo_rd, 2);

        // Extract 24-bit audio: sign-extend bits [27:4]
        int32_t raw_l = ((int32_t)(sf_l << 4)) >> 8;
        int32_t raw_r = ((int32_t)(sf_r << 4)) >> 8;

        // Validity check (bit 28): hold concealment if invalid
        if (sf_l & (1u << 28)) {
            raw_l = last_good_l;
        } else {
            last_good_l = raw_l;
        }
        if (sf_r & (1u << 28)) {
            raw_r = last_good_r;
        } else {
            last_good_r = raw_r;
        }

        buf_l[i] = (float)raw_l * inv_scale;
        buf_r[i] = (float)raw_r * inv_scale;
    }

    return pairs;
}

AudioSource spdif_input_get_source(void) {
    return current_source;
}

bool spdif_input_switch_source(AudioSource new_source) {
    if (new_source == current_source) return true;
    if (new_source > AUDIO_SOURCE_SPDIF) return false;

    if (new_source == AUDIO_SOURCE_SPDIF) {
        // USB → SPDIF
        if (rx_state != SPDIF_IN_LOCKED) return false;

        // Mute outputs briefly
        extern volatile AudioState audio_state;
        bool was_muted = audio_state.mute;
        audio_state.mute = true;
        busy_wait_ms(5);

        // Flush RX FIFO
        fifo_rd = fifo_wr_done;
        last_good_l = 0;
        last_good_r = 0;

        // Reconfigure TX clocks to match RX rate
        extern void perform_rate_change(uint32_t);
        if (audio_state.freq != detected_rate) {
            current_source = AUDIO_SOURCE_SPDIF;  // Set before rate change
            perform_rate_change(detected_rate);
        } else {
            current_source = AUDIO_SOURCE_SPDIF;
        }

        // Compute servo nominal for detected rate
        compute_servo_nominal(detected_rate);

        // Wait for FIFO to reach ~50% fill (100ms timeout)
        uint32_t timeout_ms = to_ms_since_boot(get_absolute_time()) + 100;
        while (spdif_input_get_fifo_count() < SERVO_FIFO_TARGET) {
            if (to_ms_since_boot(get_absolute_time()) > timeout_ms) break;
            tight_loop_contents();
        }

        // Reset CPU metering so pre-fill calls don't contaminate EMA
        cpu0_metering_reset();

        // Pre-fill consumer pools: call process_audio_packet 3 times
        // to prime the TX DMA pipeline with audio data
        for (int i = 0; i < 3; i++) {
            process_audio_packet(NULL, 0);
        }

        // Enable servo and DMA callback
        servo_active = true;
        tx_buffer_needed = false;
        audio_spdif_set_dma_callback(tx_dma_done_handler);

        // Unmute
        audio_state.mute = was_muted;
        return true;

    } else {
        // SPDIF → USB
        extern volatile AudioState audio_state;
        bool was_muted = audio_state.mute;
        audio_state.mute = true;
        busy_wait_ms(5);

        // Disable DMA callback and servo
        audio_spdif_set_dma_callback(NULL);
        servo_active = false;

        // Restore TX PIO dividers to nominal
        restore_tx_nominal_divider();

        // Reset PI state
        servo_pi_integral = 0;
        sd_sub_accum = 0;
        servo_fine_target = servo_nominal_fine;

        current_source = AUDIO_SOURCE_USB;

        // Reset CPU metering for clean USB-mode baseline
        cpu0_metering_reset();

        // Unmute
        audio_state.mute = was_muted;
        return true;
    }
}

#endif // PICO_RP2350
