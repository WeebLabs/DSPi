// ----------------------------------------------------------------------------
// ADAT lightpipe / TDM8 bulk output (RP2350 only)
//
// This module drives one of two wire formats for the 8 post-gain outputs,
// selected at runtime with bulk_output_set_format().  They are mutually
// exclusive and share the PIO SM, the DMA channel pair, the ring storage and
// the alignment/underrun machinery; only frame encoding and pin routing differ.
// The TDM8 format is described after the ADAT notes below.
//
// One 256-bit ADAT frame per sample carries all 8 post-gain output channels:
//   [1][10x0][1][u3..u0]  then 8 x ( 6 x [1][nibble] ), nibbles MSB-first,
// NRZI on the wire (1 = level transition).  NRZI is encoded on the CPU so
// the PIO is a single out-pins instruction at 1 cycle/bit: PIO clock =
// 256*Fs, making the ADAT clkdiv IDENTICAL to the S/PDIF TX divider.  The
// SPDIF-input clock servo therefore rate-locks ADAT by writing it the same
// divider it writes the slots (see spdif_input.c); in USB/I2S modes both run
// the same nominal divider.  Either way ADAT can never drift against the
// output slots.
//
// Encoding is LUT-driven: each PCM byte stuffs to a fixed 10-bit token
// [1][hi nibble][1][lo nibble], and NRZI is linear (the entry-level-1 line
// pattern is the complement of the entry-level-0 pattern), so a 256 x 16-bit
// table of pre-NRZI'd tokens plus one XOR mask per byte replaces the old
// separate stuff + prefix-XOR passes.  Samples arrive already converted to
// S24 in place in buf_out (see output_s24.h); the pipeline's single
// float->S24 pass feeds both the S/PDIF slots and this encoder.
//
// Buffering: a single 896-frame ring (28 KB BSS) drained by a free-running
// DMA data channel; a chained control channel rewrites the read address at
// the ring end (no IRQ, no power-of-2 size).  Frames are pushed after the
// slot gives, so the ring lead equals the slot-0 consumer fill plus a fixed
// ADAT_ALIGN_LEAD_FRAMES cushion.  The blocking give caps the fill at
// 16 x 48 samples, which bounds the lead below the ring size; overwrite is
// structurally impossible while the DMA runs.
//
// Underruns: silence frames are inserted slaved 1:1 to slot 0's DMA
// starvation counter, so ADAT shifts by exactly the amount the slots shift
// and the constant ADAT-to-slot offset survives host underruns.  While the
// host stream is stopped (starvation counting disabled) the cushion is
// simply kept topped up with silence.
//
// TDM8 (TAD5242 / TI "TDM" bus mode, Fs <= 48 kHz): FSYNC is a one-BCLK pulse
// that coincides with the MSB of slot 0 (no I2S-style 1-bit delay); 8 slots of
// 32 bits (24-bit sample MSB-first, 8 zero pad bits), 256 BCLK per frame.  The
// DAC samples DIN/FSYNC on the falling BCLK edge, so the PIO changes both on
// the rising edge.  Three GPIOs: DATA = pin, FSYNC = pin + 1 (both driven by
// one `out pins, 2`), BCLK = its own pin (side-set).  The program is two
// instructions (BCLK high + out, BCLK low) for 2 PIO cycles per BCLK, so the
// PIO clock is 512*Fs and every BCLK costs 2 bits of ring data: bit0 = DATA,
// bit1 = FSYNC.  A frame is therefore 16 words (64 B).  Daisy-chained
// TAD5242s (MD0 = TDM) each consume the last two slots and forward the rest
// on MD6, so one DATA line feeds four devices (outputs 1-2 at the far end).
// ----------------------------------------------------------------------------

#include "adat_output.h"

#if PICO_RP2350

#include <string.h>
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "usb_audio.h"
#include "spdif_input.h"
#include "i2s_input.h"
#include "adat_input.h"
#include "notify.h"

#define ADAT_PIO                pio1   // PDM owns SM 0 on this block
#define ADAT_SM                 1
#define ADAT_DMA_DATA_CH        13     // 13/14 are unused in every config:
#define ADAT_DMA_CTRL_CH        14     // outputs 0-3, PDM 4, input 5-12
#define ADAT_FRAME_WORDS        8      // 256 bits
#define TDM_FRAME_WORDS         16     // 256 BCLK x 2 bits (DATA, FSYNC)
#define ADAT_RING_FRAMES        896    // > cushion + blocking-give cap (96 + 768)
#define ADAT_ALIGN_LEAD_FRAMES  96     // constant ADAT-vs-slot lead; underrun cushion
#define ADAT_SYNC_HEADER        0x8010u // [1][10x0][1][0000]; user bits always 0

// The wire data is pre-NRZI-encoded line levels, so the program is a bare
// 1 cycle/bit shifter (same as the PDM program): out pins, 1.
static const uint16_t adat_pio_instr[] = { 0x6001 };
static const struct pio_program adat_pio_program = {
    .instructions = adat_pio_instr, .length = 1, .origin = -1,
};

// Sized for the larger TDM frame (57 KB); ADAT uses the first half.
static uint32_t adat_ring[ADAT_RING_FRAMES * TDM_FRAME_WORDS];
// Read by the control DMA channel to re-point the data channel at wrap.
static uint32_t adat_ring_base_holder;

// NRZI-encoded silence frames, one per line-entry level, plus each variant's
// exit level (entry XOR the frame's ones-parity).
static uint32_t adat_silence_nrzi[2][ADAT_FRAME_WORDS];
static uint32_t adat_silence_exit[2];
static uint32_t adat_line_level;   // line level after the last ring write

// TDM8: out pins,2 side 1 / mov y,y side 0 (1 side-set bit, 4-bit delay field).
static const uint16_t tdm_pio_instr[] = { 0x7002, 0xA042 };
static const struct pio_program tdm_pio_program = {
    .instructions = tdm_pio_instr, .length = 2, .origin = -1,
};
static uint     tdm_pio_offset;
static uint16_t tdm_spread_lut[256];     // byte -> 8 BCLK pairs, DATA in bit 0 of each
static uint32_t tdm_silence[TDM_FRAME_WORDS];

static volatile uint8_t bulk_cfg_format;   // BULK_FORMAT_*
static volatile uint8_t tdm_cfg_bclk = PICO_TDM_BCLK_PIN;
static uint8_t  bulk_hw_format;            // format the routed pins belong to
static uint8_t  tdm_hw_bclk = 0xFF;
static uint32_t bulk_frame_words = ADAT_FRAME_WORDS;   // words per ring frame
static uint32_t bulk_frame_shift = 5;                  // log2(bytes per frame)

static uint32_t adat_wr_frame;
// Mutated only from Core 0 main-loop context (resync/stop paths).  The DSP
// pipeline's per-packet finalize_s24 snapshot (audio_pipeline.c) relies on
// this staying constant across one process_input_block() call; never flip it
// from an ISR or Core 1.
static bool     adat_running;
static bool     adat_hw_init_done;
static uint8_t  adat_hw_pin = 0xFF;       // GPIO currently routed to the PIO
static uint     adat_pio_offset;
static uint32_t adat_cur_freq = 48000;
static bool     adat_rate_ok = true;
static bool     adat_stream_active_mode;  // starvation counting is live
static uint32_t adat_slot0_starv_seen;
static bool     adat_need_local_resync;

static volatile uint8_t adat_cfg_enabled;
static volatile uint8_t adat_cfg_pin = PICO_ADAT_PIN;
static volatile uint8_t tdm_cfg_pin  = PICO_TDM_DATA_PIN;   // FSYNC = +1
volatile bool adat_output_config_dirty;

// Servo-supplied divider (0 = none); same 16.8 value the SPDIF TX SMs run.
static volatile uint32_t adat_servo_div;

static uint16_t adat_resync_count;
static uint16_t adat_slip_count;
static uint32_t adat_last_notified_state = 0xFFFFFFFF;

extern uint8_t output_types[];  // usb_audio.c; slot 0 type selects starvation source

// ---------------------------------------------------------------------------
// Frame construction
// ---------------------------------------------------------------------------

// Pre-NRZI'd 10-bit line tokens for every PCM byte, entry level 0: the wire
// levels of the stuffed pattern [1][b7..b4][1][b3..b0], MSB transmitted
// first.  Entry level 1 is the complement (XOR 0x3FF).  Built at init; BSS
// so the hot path never touches flash.
static uint16_t adat_token_lut[256];

// NRZI'd 16-bit frame header [1][10x0][1][u3..u0] per entry level, with the
// exit level each variant leaves the line at.
static uint16_t adat_hdr_nrzi[2];
static uint32_t adat_hdr_exit[2];

// Encode one frame (header + 8 x S24) into 8 pre-NRZI'd transmit words,
// w[0] MSB first (the SM shifts left; bit budget 16 + 8*30 = 256).  entry is
// the line level (0/1) at the frame start; returns the exit level.  The
// level chains through the tokens as an XOR mask m (0 or 0x3FF): each
// token's LSB is the line level during its last bit.
static inline uint32_t adat_encode_frame(uint32_t *w, const int32_t s24[8],
                                         uint32_t entry) {
    uint32_t hdr = adat_hdr_nrzi[entry];
    uint32_t m = (0u - adat_hdr_exit[entry]) & 0x3FFu;
    uint32_t c[8];
    for (int ch = 0; ch < 8; ch++) {
        uint32_t s = (uint32_t)s24[ch];
        uint32_t t2 = adat_token_lut[(s >> 16) & 0xFFu] ^ m;
        m = (0u - (t2 & 1u)) & 0x3FFu;
        uint32_t t1 = adat_token_lut[(s >> 8) & 0xFFu] ^ m;
        m = (0u - (t1 & 1u)) & 0x3FFu;
        uint32_t t0 = adat_token_lut[s & 0xFFu] ^ m;
        m = (0u - (t0 & 1u)) & 0x3FFu;
        c[ch] = (t2 << 20) | (t1 << 10) | t0;
    }
    w[0] = (hdr << 16)  | (c[0] >> 14);
    w[1] = (c[0] << 18) | (c[1] >> 12);
    w[2] = (c[1] << 20) | (c[2] >> 10);
    w[3] = (c[2] << 22) | (c[3] >> 8);
    w[4] = (c[3] << 24) | (c[4] >> 6);
    w[5] = (c[4] << 26) | (c[5] >> 4);
    w[6] = (c[5] << 28) | (c[6] >> 2);
    w[7] = (c[6] << 30) |  c[7];
    return m & 1u;
}

// One TDM8 frame: 8 slots x (24-bit sample, 8 zero bits) = 16 words, two words
// per slot, 16 BCLK (32 bits) each, first BCLK in the lowest bit pair.  FSYNC
// (bit 1 of the first pair) marks the MSB of slot 0.
static inline void tdm_encode_frame(uint32_t *w, const int32_t s24[8]) {
    for (int ch = 0; ch < 8; ch++) {
        uint32_t s = (uint32_t)s24[ch];
        w[2 * ch]     = (uint32_t)tdm_spread_lut[(s >> 16) & 0xFFu]
                      | ((uint32_t)tdm_spread_lut[(s >> 8) & 0xFFu] << 16);
        w[2 * ch + 1] = tdm_spread_lut[s & 0xFFu];
    }
    w[0] |= 2u;
}

// ---------------------------------------------------------------------------
// Ring bookkeeping
// ---------------------------------------------------------------------------

static inline uint32_t adat_rd_frame(void) {
    uint32_t off = (uint32_t)(dma_hw->ch[ADAT_DMA_DATA_CH].read_addr
                              - (uintptr_t)adat_ring);
    // At the wrap instant read_addr can equal ring end for one control-channel
    // transfer; the modulo folds it to 0.
    return (off >> bulk_frame_shift) % ADAT_RING_FRAMES;
}

static inline uint32_t adat_lead_frames(void) {
    return (adat_wr_frame + ADAT_RING_FRAMES - adat_rd_frame()) % ADAT_RING_FRAMES;
}

static void adat_write_silence(uint32_t frames) {
    uint32_t wf = adat_wr_frame;
    uint32_t lvl = adat_line_level;
    const bool tdm = (bulk_hw_format == BULK_FORMAT_TDM8);
    for (uint32_t i = 0; i < frames; i++) {
        if (tdm) {
            memcpy(&adat_ring[wf * TDM_FRAME_WORDS], tdm_silence,
                   sizeof(tdm_silence));
        } else {
            memcpy(&adat_ring[wf * ADAT_FRAME_WORDS], adat_silence_nrzi[lvl],
                   ADAT_FRAME_WORDS * sizeof(uint32_t));
            lvl = adat_silence_exit[lvl];
        }
        if (++wf == ADAT_RING_FRAMES) wf = 0;
    }
    adat_line_level = lvl;
    adat_wr_frame = wf;
}

// ---------------------------------------------------------------------------
// Hardware bring-up / teardown
// ---------------------------------------------------------------------------

// Nominal 16.8 clkdiv for PIO clock = 256*Fs: the SAME formula and value as
// the S/PDIF TX library, so ADAT consumes at the identical rate in every
// mode.  Computed at resync (adat_update_divider must stay flash-free for
// the servo path).
static uint32_t adat_nom_div;

DSP_TIME_CRITICAL
static void adat_update_divider(void) {
    // While the SPDIF-input clock servo is trimming the slot dividers it
    // supplies the servoed value; sanity-bounded against nominal so a stale
    // value from a previous rate can never be applied.
    uint32_t div = adat_nom_div;
    uint32_t servo = adat_servo_div;
    // The servo value is for a 256*Fs PIO clock; TDM8 runs 512*Fs.
    if (bulk_hw_format == BULK_FORMAT_TDM8) servo >>= 1;
    if (servo && servo > div - div / 128 && servo < div + div / 128)
        div = servo;
    pio_sm_set_clkdiv_int_frac(ADAT_PIO, ADAT_SM,
                               (uint16_t)(div >> 8), (uint8_t)(div & 0xFF));
}

static void adat_hw_init_once(void) {
    if (adat_hw_init_done) return;
    pio_sm_claim(ADAT_PIO, ADAT_SM);
    adat_pio_offset = pio_add_program(ADAT_PIO, &adat_pio_program);
    tdm_pio_offset  = pio_add_program(ADAT_PIO, &tdm_pio_program);
    dma_channel_claim(ADAT_DMA_DATA_CH);
    dma_channel_claim(ADAT_DMA_CTRL_CH);
    adat_ring_base_holder = (uint32_t)(uintptr_t)adat_ring;
    adat_hw_init_done = true;
}

static void adat_sm_configure(void) {
    pio_sm_config c = pio_get_default_sm_config();
    uint offset;
    if (bulk_hw_format == BULK_FORMAT_TDM8) {
        offset = tdm_pio_offset;
        sm_config_set_wrap(&c, offset, offset + 1);
        sm_config_set_out_pins(&c, adat_hw_pin, 2);      // DATA, FSYNC = DATA + 1
        sm_config_set_set_pins(&c, adat_hw_pin, 2);
        sm_config_set_sideset(&c, 1, false, false);
        sm_config_set_sideset_pins(&c, tdm_hw_bclk);
        sm_config_set_out_shift(&c, true, true, 32);     // shift right: first BCLK = low bits
    } else {
        offset = adat_pio_offset;
        sm_config_set_wrap(&c, offset, offset);
        sm_config_set_out_pins(&c, adat_hw_pin, 1);
        sm_config_set_set_pins(&c, adat_hw_pin, 1);      // lets stop force the pin low
        sm_config_set_out_shift(&c, false, true, 32);    // shift left: MSB first
    }
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(ADAT_PIO, ADAT_SM, offset, &c);
    adat_update_divider();                            // pio_sm_init resets the divider
}

static void adat_stop_hw(void) {
    if (!adat_hw_init_done) return;
    // Break the retrigger chain first: a data completion racing the aborts
    // would otherwise fire the control channel and re-arm the data channel.
    dma_channel_config dc = dma_get_channel_config(ADAT_DMA_DATA_CH);
    channel_config_set_chain_to(&dc, ADAT_DMA_DATA_CH);
    dma_channel_set_config(ADAT_DMA_DATA_CH, &dc, false);
    dma_channel_abort(ADAT_DMA_CTRL_CH);
    dma_channel_abort(ADAT_DMA_DATA_CH);
    pio_sm_set_enabled(ADAT_PIO, ADAT_SM, false);
    pio_sm_clear_fifos(ADAT_PIO, ADAT_SM);
    // Force only OUR pin low via the SM's SET mapping; pio_sm_set_pins()
    // would transiently drive every GPIO muxed to this PIO block (PDM).
    if (adat_hw_pin != 0xFF) {
        pio_sm_exec(ADAT_PIO, ADAT_SM, pio_encode_set(pio_pins, 0));
        if (bulk_hw_format == BULK_FORMAT_TDM8)   // BCLK low (side-set 0)
            pio_sm_exec(ADAT_PIO, ADAT_SM,
                        pio_encode_nop() | pio_encode_sideset(1, 0));
    }
    adat_running = false;
}

static void bulk_release_gpio(uint8_t pin) {
    // Only release if still ours: another peripheral may have re-claimed the
    // pin between a deferred disable and this service (its mux is not PIO1).
    if (gpio_get_function(pin) == GPIO_FUNC_PIO1) {
        gpio_set_function(pin, GPIO_FUNC_NULL);
        gpio_set_dir(pin, GPIO_IN);
    }
}

static void adat_release_pin(void) {
    if (adat_hw_pin == 0xFF) return;
    bulk_release_gpio(adat_hw_pin);
    if (bulk_hw_format == BULK_FORMAT_TDM8) {
        bulk_release_gpio((uint8_t)(adat_hw_pin + 1));
        bulk_release_gpio(tdm_hw_bclk);
        tdm_hw_bclk = 0xFF;
    }
    adat_hw_pin = 0xFF;
}

static void adat_arm_dma(void) {
    // Control channel: on data completion, rewrite the data channel's read
    // address (trigger alias) back to the ring base.  TRANS_COUNT reloads
    // from its shadow on every trigger, so this runs forever without an IRQ.
    dma_channel_config cc = dma_channel_get_default_config(ADAT_DMA_CTRL_CH);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    dma_channel_configure(ADAT_DMA_CTRL_CH, &cc,
                          &dma_hw->ch[ADAT_DMA_DATA_CH].al3_read_addr_trig,
                          &adat_ring_base_holder, 1, false);

    dma_channel_config dc = dma_channel_get_default_config(ADAT_DMA_DATA_CH);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(ADAT_PIO, ADAT_SM, true));
    channel_config_set_chain_to(&dc, ADAT_DMA_CTRL_CH);
    channel_config_set_high_priority(&dc, true);
    dma_channel_configure(ADAT_DMA_DATA_CH, &dc, &ADAT_PIO->txf[ADAT_SM],
                          adat_ring, ADAT_RING_FRAMES * bulk_frame_words, true);
}

static inline uint32_t adat_slot0_starvations(void) {
    return (output_types[0] == OUTPUT_TYPE_I2S)
        ? audio_i2s_get_dma_starvations_instance(0)
        : audio_spdif_get_dma_starvations_instance(0);
}

static void adat_notify_if_changed(void) {
    // Whole tuple, so enable/pin changes made while rate-suspended (active
    // unchanged) still produce an event.
    uint32_t state = ((uint32_t)bulk_cfg_format << 24)
                   | ((uint32_t)(bulk_cfg_format ? tdm_cfg_pin : adat_cfg_pin) << 16)
                   | ((uint32_t)adat_cfg_enabled << 8)
                   | (adat_running ? 1u : 0u);
    if (state == adat_last_notified_state) return;
    adat_last_notified_state = state;
    notify_push_adat_state(adat_cfg_enabled, adat_running ? 1 : 0,
                           bulk_cfg_format ? tdm_cfg_pin : adat_cfg_pin);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void adat_output_init(void) {
    // Token LUT: NRZI wire levels (entry level 0) of the stuffed byte
    // [1][b7..b4][1][b3..b0].  0x210 = the two forced '1' bits.
    for (uint32_t b = 0; b < 256; b++) {
        uint32_t stuffed = 0x210u | ((b & 0xF0u) << 1) | (b & 0x0Fu);
        uint32_t t = 0, l = 0;
        for (int k = 9; k >= 0; k--) {
            l ^= (stuffed >> k) & 1u;
            t = (t << 1) | l;
        }
        adat_token_lut[b] = (uint16_t)t;
    }
    // Header variants per entry level.
    for (uint32_t lvl = 0; lvl < 2; lvl++) {
        uint32_t t = 0, l = lvl;
        for (int k = 15; k >= 0; k--) {
            l ^= (ADAT_SYNC_HEADER >> k) & 1u;
            t = (t << 1) | l;
        }
        adat_hdr_nrzi[lvl] = (uint16_t)t;
        adat_hdr_exit[lvl] = l;
    }
    // TDM8: spread each byte MSB-first onto the DATA bit of 8 BCLK pairs.
    for (uint32_t b = 0; b < 256; b++) {
        uint32_t t = 0;
        for (int k = 0; k < 8; k++)
            t |= ((b >> (7 - k)) & 1u) << (2 * k);
        tdm_spread_lut[b] = (uint16_t)t;
    }
    {
        const int32_t zero[8] = {0};
        tdm_encode_frame(tdm_silence, zero);
    }
    // Silence frames per entry level, via the same encoder as live audio.
    const int32_t zero[8] = {0};
    for (uint32_t lvl = 0; lvl < 2; lvl++)
        adat_silence_exit[lvl] =
            adat_encode_frame(adat_silence_nrzi[lvl], zero, lvl);
}

void adat_output_set_config(bool enabled, uint8_t pin) {
    if (adat_cfg_enabled == (uint8_t)enabled && adat_cfg_pin == pin) return;
    adat_cfg_enabled = (uint8_t)enabled;
    adat_cfg_pin = pin;
    adat_output_config_dirty = true;
}

void bulk_output_set_format(uint8_t format) {
    format = (format == BULK_FORMAT_TDM8) ? BULK_FORMAT_TDM8 : BULK_FORMAT_ADAT;
    if (bulk_cfg_format == format) return;
    bulk_cfg_format = format;
    adat_output_config_dirty = true;
}

uint8_t bulk_output_format(void) { return bulk_cfg_format; }

void tdm_output_set_pins(uint8_t data_pin, uint8_t bclk_pin) {
    if (tdm_cfg_pin == data_pin && tdm_cfg_bclk == bclk_pin) return;
    tdm_cfg_pin = data_pin;
    tdm_cfg_bclk = bclk_pin;
    adat_output_config_dirty = true;
}

uint8_t tdm_output_data_pin(void) { return tdm_cfg_pin; }
uint8_t tdm_output_bclk_pin(void) { return tdm_cfg_bclk; }

bool bulk_output_owns_pin(uint8_t pin) {
    if (!adat_cfg_enabled) return false;
    if (bulk_cfg_format == BULK_FORMAT_TDM8)
        return pin == tdm_cfg_pin || pin == (uint8_t)(tdm_cfg_pin + 1)
            || pin == tdm_cfg_bclk;
    return pin == adat_cfg_pin;
}

bool    adat_output_config_enabled(void) { return adat_cfg_enabled != 0; }
uint8_t adat_output_pin(void)            { return adat_cfg_pin; }

// Sampled per packet by the DSP pipeline to pick the slot finalization mode
// (see output_s24.h); must stay RAM-resident like its caller.
DSP_TIME_CRITICAL
bool    adat_output_is_active(void)      { return adat_running; }

void adat_output_get_status(AdatStatus *out) {
    out->enabled      = adat_cfg_enabled;
    out->active       = adat_running ? 1 : 0;
    out->pin          = bulk_cfg_format ? tdm_cfg_pin : adat_cfg_pin;
    out->rate_ok      = adat_rate_ok ? 1 : 0;
    out->resync_count = adat_resync_count;
    out->slip_count   = adat_slip_count;
}

void adat_output_on_rate_change(uint32_t freq) {
    adat_cur_freq = freq;
    adat_rate_ok = (freq <= 48000);
    adat_servo_div = 0;   // stale for the new rate; servo re-pushes if locked
    // Caller (perform_rate_change) holds the mute; the following
    // complete_pipeline_reset() restarts the stream via resync if valid.
    if (!adat_rate_ok && adat_running) adat_stop_hw();
}

// Called from the RAM-pinned SPDIF-input clock servo; keep it RAM-resident.
DSP_TIME_CRITICAL
void adat_output_servo_divider(uint32_t div_16_8) {
    adat_servo_div = div_16_8;
    if (adat_running && div_16_8) adat_update_divider();
}

void adat_output_stream_stop(void) {
    adat_stop_hw();
}

void adat_output_resync(void) {
    adat_stop_hw();
    adat_output_config_dirty = false;
    adat_need_local_resync = false;

    // Re-derive from the live rate: a boot preset can put the system at
    // 44.1 kHz without perform_rate_change() ever running.
    adat_cur_freq = audio_state.freq;
    adat_rate_ok = (adat_cur_freq <= 48000);

    if (!adat_cfg_enabled || !adat_rate_ok) {
        adat_release_pin();
        adat_notify_if_changed();
        return;
    }

    adat_hw_init_once();
    const uint8_t fmt  = bulk_cfg_format;
    const uint8_t data = (fmt == BULK_FORMAT_TDM8) ? tdm_cfg_pin : adat_cfg_pin;
    const uint8_t bclk = tdm_cfg_bclk;
    if (adat_hw_pin != data || bulk_hw_format != fmt ||
        (fmt == BULK_FORMAT_TDM8 && tdm_hw_bclk != bclk)) {
        adat_release_pin();   // releases the pins of the OLD format
        bulk_hw_format = fmt;
        adat_hw_pin = data;
        pio_gpio_init(ADAT_PIO, data);
        if (fmt == BULK_FORMAT_TDM8) {
            tdm_hw_bclk = bclk;
            pio_gpio_init(ADAT_PIO, (uint)data + 1);
            pio_gpio_init(ADAT_PIO, bclk);
            pio_sm_set_consecutive_pindirs(ADAT_PIO, ADAT_SM, data, 2, true);
            pio_sm_set_consecutive_pindirs(ADAT_PIO, ADAT_SM, bclk, 1, true);
        } else {
            pio_sm_set_consecutive_pindirs(ADAT_PIO, ADAT_SM, data, 1, true);
        }
    }
    bulk_frame_words = (fmt == BULK_FORMAT_TDM8) ? TDM_FRAME_WORDS : ADAT_FRAME_WORDS;
    bulk_frame_shift = (fmt == BULK_FORMAT_TDM8) ? 6 : 5;
    // Pick up the active input servo's current divider (0 when no servo is
    // locked; at most one of the two can be nonzero since input sources are
    // exclusive); adat_update_divider() sanity-bounds it against nominal.
    {
        uint32_t sys = clock_get_hz(clk_sys);
        // PIO clock: 256*Fs for ADAT, 512*Fs for TDM8 (two cycles per BCLK).
        uint32_t fs = adat_cur_freq * (fmt == BULK_FORMAT_TDM8 ? 2u : 1u);
        adat_nom_div = sys / fs + (sys % fs != 0);
    }
    adat_servo_div = spdif_input_current_tx_divider();
    if (adat_servo_div == 0) adat_servo_div = i2s_slave_current_tx_divider();
    if (adat_servo_div == 0) adat_servo_div = adat_input_current_tx_divider();
    adat_sm_configure();

    // Line starts low: force the pin and seed the NRZI encoder to match.
    pio_sm_exec(ADAT_PIO, ADAT_SM, pio_encode_set(pio_pins, 0));
    if (fmt == BULK_FORMAT_TDM8)
        pio_sm_exec(ADAT_PIO, ADAT_SM,
                    pio_encode_nop() | pio_encode_sideset(1, 0));
    adat_line_level = 0;

    // Alignment cushion: the constant ADAT-vs-slot lead, re-established at
    // every synchronized output restart.
    adat_wr_frame = 0;
    adat_write_silence(ADAT_ALIGN_LEAD_FRAMES);

    adat_slot0_starv_seen = adat_slot0_starvations();
    adat_arm_dma();
    pio_sm_set_enabled(ADAT_PIO, ADAT_SM, true);
    adat_running = true;
    adat_resync_count++;
    adat_notify_if_changed();
}

void adat_output_set_stream_active(bool active) {
    if (active && !adat_stream_active_mode)
        adat_slot0_starv_seen = adat_slot0_starvations();  // re-baseline
    adat_stream_active_mode = active;
}

void adat_output_rebaseline_starvations(void) {
    adat_slot0_starv_seen = adat_slot0_starvations();
}

void adat_output_task(void) {
    if (adat_need_local_resync) {
        adat_slip_count++;
        adat_output_resync();
        return;
    }
    if (!adat_running) return;

    if (adat_stream_active_mode) {
        // Slaved insertion: mirror slot 0's silence fallbacks 1:1 so the
        // ADAT-to-slot offset survives underruns exactly.
        uint32_t starv = adat_slot0_starvations();
        uint32_t pending = starv - adat_slot0_starv_seen;
        if (pending > 256) {
            // Impossible as a real backlog: the counter was reset (stream
            // (re)start, stats reset).  Re-baseline without inserting.
            adat_slot0_starv_seen = starv;
        } else if (pending) {
            if (pending > 8) pending = 8;   // bound per pass; catch up next call
            adat_write_silence(pending * PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT);
            adat_slot0_starv_seen += pending;
        }
    } else {
        // Host stream stopped: no gives, no starvation counting.  Keep the
        // cushion topped up; alignment re-canonicalizes at the next resync.
        uint32_t lead = adat_lead_frames();
        if (lead < ADAT_ALIGN_LEAD_FRAMES)
            adat_write_silence(ADAT_ALIGN_LEAD_FRAMES - lead);
    }

    // The DMA passing the write pointer means the cushion is gone (stalled
    // main loop or dead DMA); restart cleanly instead of playing the ring.
    if (adat_lead_frames() == 0) adat_need_local_resync = true;
}

DSP_TIME_CRITICAL
void adat_output_push_block(const out_s24_t (*bufs)[192], uint32_t sample_count) {
    if (!adat_running) return;

    // Structurally unreachable while the blocking give caps the slot fill;
    // reaching it means the DMA stalled.  Skip and restart rather than
    // overwrite unplayed frames.
    if (adat_lead_frames() + sample_count >= ADAT_RING_FRAMES) {
        adat_need_local_resync = true;
        return;
    }

    uint32_t wf = adat_wr_frame;
    uint32_t lvl = adat_line_level;
    const bool tdm = (bulk_hw_format == BULK_FORMAT_TDM8);
    for (uint32_t i = 0; i < sample_count; i++) {
        int32_t s[8];
        for (int ch = 0; ch < 8; ch++) s[ch] = bufs[ch][i];
        if (tdm)
            tdm_encode_frame(&adat_ring[wf * TDM_FRAME_WORDS], s);
        else
            lvl = adat_encode_frame(&adat_ring[wf * ADAT_FRAME_WORDS], s, lvl);
        if (++wf == ADAT_RING_FRAMES) wf = 0;
    }
    adat_line_level = lvl;
    adat_wr_frame = wf;
}

#endif // PICO_RP2350
