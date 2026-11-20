/*
 * wirecap.c: bench wire-capture rig, DSPI_WIRECAP builds only.
 * Resources, commands and limits: Documentation/Features/wirecap_test_rig.md.
 */

#ifdef DSPI_WIRECAP

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/address_mapped.h"
#include "wirecap.h"
#include "audio_clock_div.h"
#include "audio_input.h"
#include "usb_audio.h"

#if PICO_RP2350
// About 120 KB lies free between .bss and .slot_rings; keep some of it for the heap.
#define WIRECAP_BUF_BYTES   (96u * 1024u)
#define GEN_FRAMES          128u
#define GEN_RING_BITS       13u      // 128 frames x 64 bytes
#else
#define WIRECAP_BUF_BYTES   (16u * 1024u)
#define GEN_FRAMES          64u
#define GEN_RING_BITS       12u
#endif
#define GEN_WORDS_PER_FRAME 16u      // 64 BCK x 4 quarter cells x 2 pins = 512 bits
#define GEN_QUARTERS        256u
#define GEN_GLITCH_CELL     10u      // well clear of both LRCLK edges
#define GEN_GLITCH_LEAD     8u       // frames between the DMA read position and the glitch

typedef struct { PIO pio; uint8_t sm; } SmSlot;

// Never PIO0: the output slots own it. PIO1 SM0-2 are PDM and ADAT; PIO2 SM0-3 are the receivers.
#if PICO_RP2350
static const SmSlot k_cap_sms[] = { { pio2, 3 }, { pio1, 3 }, { pio2, 2 } };
static const SmSlot k_gen_sms[] = { { pio1, 3 }, { pio2, 3 }, { pio2, 2 } };
// 9 and 10 are never claimed by fixed users; 13/14 belong to ADAT TX, which the rig must not run beside.
static const uint8_t k_dma_pool[] = { 9, 10, 14, 13 };
#else
static const SmSlot k_cap_sms[] = { { pio1, 3 } };
static const SmSlot k_gen_sms[] = { { pio1, 3 } };
static const uint8_t k_dma_pool[] = { 9, 8, 3 };
#endif

// ---- Sampler ----

typedef enum { CAP_IDLE = 0, CAP_RUNNING = 1, CAP_DONE = 2, CAP_ABORTED = 3 } CapState;

static uint32_t s_buf[WIRECAP_BUF_BYTES / 4] __attribute__((aligned(4)));
static uint16_t s_cap_instr[1];
static pio_program_t s_cap_prog = { .instructions = s_cap_instr, .length = 1, .origin = -1 };

static struct {
    volatile uint8_t state;
    uint8_t  flags;              // bit0 RXSTALL (a sample was delayed), bit1 RXUNDER
    uint8_t  base, count;
    uint16_t div_int;
    uint8_t  div_frac;
    int8_t   offset;
    PIO      pio;
    uint8_t  sm, dma_data, dma_ctrl;
    uint32_t clk_hz, bytes, seq;
    uint32_t stop_mask;          // the SM's enable bit, written to the CTRL clear alias by dma_ctrl
} s_cap;

// ---- Clock generator ----

static uint32_t s_ring[GEN_FRAMES * GEN_WORDS_PER_FRAME]
    __attribute__((aligned(GEN_FRAMES * GEN_WORDS_PER_FRAME * 4)));
static uint16_t s_gen_instr[1];
static pio_program_t s_gen_prog = { .instructions = s_gen_instr, .length = 1, .origin = -1 };

static struct {
    bool     on;
    PIO      pio;
    uint8_t  sm, dma;
    int8_t   offset;
    uint32_t rate, div;          // div is 16.8
    uint32_t phase;              // quarter-cell phase of the ring pattern, 0..255
    uint8_t  last_kind;
    uint32_t glitches, late;
} s_gen;

// ---- Pad ownership of the generator pins ----

void __real_gpio_set_function(uint gpio, gpio_function_t fn);
void __real_gpio_init(uint gpio);

enum { REQ_NONE = 0, REQ_FUNC, REQ_INIT };
static bool    s_held;
static uint8_t s_orig_fn[2];
static uint8_t s_req[2], s_req_fn[2];

static inline int held_index(uint gpio) {
    if (!s_held) return -1;
    if (gpio == WIRECAP_CLK_BCK_PIN) return 0;
    if (gpio == WIRECAP_CLK_LRCK_PIN) return 1;
    return -1;
}

// Linked with --wrap: while the generator drives 24/25, other owners' mux
// changes are deferred to its release, so slave mode's gpio_init cannot cut it off.
void __wrap_gpio_set_function(uint gpio, gpio_function_t fn) {
    int i = held_index(gpio);
    if (i >= 0 && fn != (gpio_function_t)(GPIO_FUNC_PIO0 + pio_get_index(s_gen.pio))) {
        s_req[i] = REQ_FUNC;
        s_req_fn[i] = (uint8_t)fn;
        return;
    }
    __real_gpio_set_function(gpio, fn);
}

void __wrap_gpio_init(uint gpio) {
    int i = held_index(gpio);
    if (i >= 0) { s_req[i] = REQ_INIT; return; }
    __real_gpio_init(gpio);
}

static uint8_t pad_funcsel(uint gpio) {
    return (uint8_t)((io_bank0_hw->io[gpio].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS)
                     >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB);
}

static void pins_hold(void) {
    const uint pins[2] = { WIRECAP_CLK_BCK_PIN, WIRECAP_CLK_LRCK_PIN };
    for (int i = 0; i < 2; i++) {
        s_orig_fn[i] = pad_funcsel(pins[i]);
        s_req[i] = REQ_NONE;
    }
    s_held = true;
    for (int i = 0; i < 2; i++)
        __real_gpio_set_function(pins[i], (gpio_function_t)(GPIO_FUNC_PIO0 + pio_get_index(s_gen.pio)));
}

// Hands the pins to whoever asked for them while held, else back to their owner at hold time.
static void pins_release(void) {
    const uint pins[2] = { WIRECAP_CLK_BCK_PIN, WIRECAP_CLK_LRCK_PIN };
    s_held = false;
    for (int i = 0; i < 2; i++) {
        if (s_req[i] == REQ_INIT) __real_gpio_init(pins[i]);
        else if (s_req[i] == REQ_FUNC) __real_gpio_set_function(pins[i], (gpio_function_t)s_req_fn[i]);
        else __real_gpio_set_function(pins[i], (gpio_function_t)s_orig_fn[i]);
        s_req[i] = REQ_NONE;
    }
}

// ---- Shared resource helpers ----

static bool sm_taken(const SmSlot *s, PIO other_pio, uint8_t other_sm, bool other_on) {
    if (other_on && s->pio == other_pio && s->sm == other_sm) return true;
    return pio_sm_is_claimed(s->pio, s->sm);
}

static int claim_dma(void) {
    for (unsigned i = 0; i < sizeof(k_dma_pool); i++) {
        if (!dma_channel_is_claimed(k_dma_pool[i])) {
            dma_channel_claim(k_dma_pool[i]);
            return k_dma_pool[i];
        }
    }
    return -1;
}

// ---- Sampler ----

static void cap_release(void) {
    pio_sm_set_enabled(s_cap.pio, s_cap.sm, false);
    dma_channel_abort(s_cap.dma_data);
    dma_channel_abort(s_cap.dma_ctrl);
    pio_remove_program(s_cap.pio, &s_cap_prog, (uint)s_cap.offset);
    pio_sm_unclaim(s_cap.pio, s_cap.sm);
    dma_channel_unclaim(s_cap.dma_data);
    dma_channel_unclaim(s_cap.dma_ctrl);
}

static void cap_finish_if_done(void) {
    if (s_cap.state != CAP_RUNNING) return;
    if (dma_channel_is_busy(s_cap.dma_data) || dma_channel_is_busy(s_cap.dma_ctrl)) return;
    // The ctrl channel stopped the SM within a few cycles of the last word; the
    // joined 8-deep FIFO absorbs those, so RXSTALL means a sample really waited.
    uint32_t fd = s_cap.pio->fdebug;
    s_cap.flags = (uint8_t)(((fd >> (PIO_FDEBUG_RXSTALL_LSB + s_cap.sm)) & 1u) |
                            (((fd >> (PIO_FDEBUG_RXUNDER_LSB + s_cap.sm)) & 1u) << 1));
    cap_release();
    s_cap.state = CAP_DONE;
}

WirecapResult wirecap_arm(uint8_t pin_base, uint8_t pin_count, uint16_t div_int,
                          uint8_t div_frac, uint32_t bytes) {
    cap_finish_if_done();
    if (bytes == 0) {
        if (s_cap.state == CAP_RUNNING) { cap_release(); s_cap.state = CAP_ABORTED; }
        return WIRECAP_OK;
    }
    if (s_cap.state == CAP_RUNNING) return WIRECAP_ERR_STATE;
    if (pin_count == 0 || pin_count > 32 || (pin_count & (pin_count - 1)) ||
        pin_base >= 32 || div_int == 0 || (bytes & 3u) || bytes > WIRECAP_BUF_BYTES)
        return WIRECAP_ERR_VALUE;

    const SmSlot *slot = NULL;
    for (unsigned i = 0; i < sizeof(k_cap_sms) / sizeof(k_cap_sms[0]); i++) {
        if (!sm_taken(&k_cap_sms[i], s_gen.pio, s_gen.sm, s_gen.on)) { slot = &k_cap_sms[i]; break; }
    }
    if (!slot) return WIRECAP_ERR_RESOURCE;
    s_cap_instr[0] = pio_encode_in(pio_pins, pin_count);
    if (!pio_can_add_program(slot->pio, &s_cap_prog)) return WIRECAP_ERR_RESOURCE;
    int d0 = claim_dma();
    int d1 = d0 >= 0 ? claim_dma() : -1;
    if (d1 < 0) {
        if (d0 >= 0) dma_channel_unclaim((uint)d0);
        return WIRECAP_ERR_RESOURCE;
    }

    pio_sm_claim(slot->pio, slot->sm);
    s_cap.pio = slot->pio;
    s_cap.sm = slot->sm;
    s_cap.dma_data = (uint8_t)d0;
    s_cap.dma_ctrl = (uint8_t)d1;
    s_cap.offset = (int8_t)pio_add_program(s_cap.pio, &s_cap_prog);
    s_cap.base = pin_base;
    s_cap.count = pin_count;
    s_cap.div_int = div_int;
    s_cap.div_frac = div_frac;
    s_cap.bytes = bytes;
    s_cap.flags = 0;
    s_cap.clk_hz = clock_get_hz(clk_sys);
    s_cap.seq++;

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, (uint)s_cap.offset, (uint)s_cap.offset);
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac8(&c, div_int, div_frac);
    pio_sm_init(s_cap.pio, s_cap.sm, (uint)s_cap.offset, &c);
    s_cap.pio->fdebug = (1u << (PIO_FDEBUG_RXSTALL_LSB + s_cap.sm)) |
                        (1u << (PIO_FDEBUG_RXUNDER_LSB + s_cap.sm));

    s_cap.stop_mask = 1u << (PIO_CTRL_SM_ENABLE_LSB + s_cap.sm);
    dma_channel_config cc = dma_channel_get_default_config(s_cap.dma_ctrl);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    dma_channel_configure(s_cap.dma_ctrl, &cc, hw_clear_alias(&s_cap.pio->ctrl),
                          &s_cap.stop_mask, 1, false);

    dma_channel_config dc = dma_channel_get_default_config(s_cap.dma_data);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(s_cap.pio, s_cap.sm, false));
    channel_config_set_chain_to(&dc, s_cap.dma_ctrl);
    memset(s_buf, 0, bytes);
    dma_channel_configure(s_cap.dma_data, &dc, s_buf, &s_cap.pio->rxf[s_cap.sm], bytes / 4u, true);

    s_cap.state = CAP_RUNNING;
    pio_sm_set_enabled(s_cap.pio, s_cap.sm, true);
    return WIRECAP_OK;
}

void wirecap_status(uint8_t out[WIRECAP_STATUS_LEN]) {
    cap_finish_if_done();
    uint32_t done = 0;
    if (s_cap.state == CAP_RUNNING) {
        uint32_t left = dma_hw->ch[s_cap.dma_data].transfer_count & 0x0FFFFFFFu;
        done = s_cap.bytes - 4u * left;
    } else if (s_cap.state == CAP_DONE) {
        done = s_cap.bytes;
    }
    memset(out, 0, WIRECAP_STATUS_LEN);
    out[0] = s_cap.state;
    out[1] = s_cap.flags;
    out[2] = s_cap.base;
    out[3] = s_cap.count;
    memcpy(out + 4, &s_cap.clk_hz, 4);
    memcpy(out + 8, &s_cap.div_int, 2);
    out[10] = s_cap.div_frac;
    out[11] = s_cap.seq ? (uint8_t)pio_get_index(s_cap.pio) : 0xFF;
    memcpy(out + 12, &s_cap.bytes, 4);
    memcpy(out + 16, &done, 4);
    out[20] = s_cap.sm;
    out[21] = s_cap.dma_data;
    out[22] = s_cap.dma_ctrl;
    uint32_t cap = WIRECAP_BUF_BYTES;
    memcpy(out + 24, &cap, 4);
    memcpy(out + 28, &s_cap.seq, 4);
}

uint16_t wirecap_read(uint32_t offset, uint8_t *dst, uint16_t len) {
    cap_finish_if_done();
    if (s_cap.state != CAP_DONE || offset >= s_cap.bytes) return 0;
    if (len > s_cap.bytes - offset) len = (uint16_t)(s_cap.bytes - offset);
    memcpy(dst, (const uint8_t *)s_buf + offset, len);
    return len;
}

// ---- Clock generator ----

// Quarter q of a frame: BCK low then high per cell, LRCLK low (left) for cells 0-31.
static inline uint32_t gen_state(uint32_t q) {
    uint32_t cell = (q >> 2) & 63u;
    return ((q & 3u) >= 2u ? 1u : 0u) | (cell >= 32u ? 2u : 0u);
}

static void gen_frame_words(uint32_t *w, uint32_t phase, int glitch_kind) {
    uint8_t st[GEN_QUARTERS];
    for (uint32_t q = 0; q < GEN_QUARTERS; q++) st[q] = (uint8_t)gen_state((q + phase) & 255u);
    if (glitch_kind == 1 || glitch_kind == 2) {
        // Cell GEN_GLITCH_CELL in stream terms; phase is always whole cells.
        uint32_t p = (GEN_GLITCH_CELL * 4u - phase) & 255u;
        uint8_t lr = st[p] & 2u;
        static const uint8_t runt[4] = { 0, 1, 0, 1 }, none[4] = { 0, 0, 0, 0 };
        const uint8_t *b = glitch_kind == 1 ? runt : none;
        for (int i = 0; i < 4; i++) st[p + i] = (uint8_t)(lr | b[i]);
    }
    for (uint32_t j = 0; j < GEN_WORDS_PER_FRAME; j++) {
        uint32_t v = 0;
        for (uint32_t i = 0; i < 16; i++) v |= (uint32_t)st[j * 16 + i] << (2 * i);
        w[j] = v;
    }
}

static void gen_fill(uint32_t phase) {
    uint32_t w[GEN_WORDS_PER_FRAME];
    gen_frame_words(w, phase, 0);
    for (uint32_t f = 0; f < GEN_FRAMES; f++) memcpy(&s_ring[f * GEN_WORDS_PER_FRAME], w, sizeof(w));
}

static uint32_t gen_frame_now(void) {
    uint32_t a = dma_hw->ch[s_gen.dma].read_addr - (uint32_t)(uintptr_t)s_ring;
    return (a / (GEN_WORDS_PER_FRAME * 4u)) % GEN_FRAMES;
}

static void gen_stop(void) {
    pio_sm_set_enabled(s_gen.pio, s_gen.sm, false);
    dma_channel_abort(s_gen.dma);
    pio_sm_set_consecutive_pindirs(s_gen.pio, s_gen.sm, WIRECAP_CLK_BCK_PIN, 2, false);
    pins_release();
    pio_remove_program(s_gen.pio, &s_gen_prog, (uint)s_gen.offset);
    pio_sm_unclaim(s_gen.pio, s_gen.sm);
    dma_channel_unclaim(s_gen.dma);
    s_gen.on = false;
}

WirecapResult wirecap_clkgen_set(bool enable, uint32_t rate) {
    if (!enable) {
        if (s_gen.on) gen_stop();
        return WIRECAP_OK;
    }
    if (rate != 44100u && rate != 48000u && rate != 96000u) return WIRECAP_ERR_VALUE;
    uint32_t div = audio_base_divider_16_8(rate);   // the output slots' exact 256 x Fs divider
    if (s_gen.on) {
        pio_sm_set_clkdiv_int_frac8(s_gen.pio, s_gen.sm, div >> 8, (uint8_t)div);
        s_gen.rate = rate;
        s_gen.div = div;
        return WIRECAP_OK;
    }

    const SmSlot *slot = NULL;
    for (unsigned i = 0; i < sizeof(k_gen_sms) / sizeof(k_gen_sms[0]); i++) {
        if (!sm_taken(&k_gen_sms[i], s_cap.pio, s_cap.sm, s_cap.state == CAP_RUNNING)) { slot = &k_gen_sms[i]; break; }
    }
    if (!slot) return WIRECAP_ERR_RESOURCE;
    s_gen_instr[0] = pio_encode_out(pio_pins, 2);
    if (!pio_can_add_program(slot->pio, &s_gen_prog)) return WIRECAP_ERR_RESOURCE;
    int d = claim_dma();
    if (d < 0) return WIRECAP_ERR_RESOURCE;

    pio_sm_claim(slot->pio, slot->sm);
    s_gen.pio = slot->pio;
    s_gen.sm = slot->sm;
    s_gen.dma = (uint8_t)d;
    s_gen.offset = (int8_t)pio_add_program(s_gen.pio, &s_gen_prog);
    s_gen.rate = rate;
    s_gen.div = div;
    s_gen.phase = 0;
    gen_fill(0);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, (uint)s_gen.offset, (uint)s_gen.offset);
    sm_config_set_out_pins(&c, WIRECAP_CLK_BCK_PIN, 2);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, div >> 8, (uint8_t)div);
    pio_sm_init(s_gen.pio, s_gen.sm, (uint)s_gen.offset, &c);
    pio_sm_set_pins_with_mask(s_gen.pio, s_gen.sm, 0, 3u << WIRECAP_CLK_BCK_PIN);
    pio_sm_set_consecutive_pindirs(s_gen.pio, s_gen.sm, WIRECAP_CLK_BCK_PIN, 2, true);
    pins_hold();

    dma_channel_config dc = dma_channel_get_default_config(s_gen.dma);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_ring(&dc, false, GEN_RING_BITS);
    channel_config_set_dreq(&dc, pio_get_dreq(s_gen.pio, s_gen.sm, true));
#if PICO_RP2350
    uint32_t count = dma_encode_endless_transfer_count();
#else
    uint32_t count = 0xFFFFFFFFu;   // RP2040 has no endless mode: about 90 minutes at 48 kHz
#endif
    dma_channel_configure(s_gen.dma, &dc, &s_gen.pio->txf[s_gen.sm], s_ring, count, true);
    pio_sm_set_enabled(s_gen.pio, s_gen.sm, true);
    s_gen.on = true;
    return WIRECAP_OK;
}

void wirecap_clkgen_status(uint8_t out[WIRECAP_CLKGEN_LEN]) {
    memset(out, 0, WIRECAP_CLKGEN_LEN);
    out[0] = s_gen.on;
    out[1] = s_gen.on ? (uint8_t)pio_get_index(s_gen.pio) : 0xFF;
    out[2] = s_gen.sm;
    out[3] = s_gen.dma;
    memcpy(out + 4, &s_gen.rate, 4);
    memcpy(out + 8, &s_gen.div, 4);
    memcpy(out + 12, &s_gen.glitches, 4);
    memcpy(out + 16, &s_gen.late, 4);
    out[20] = (uint8_t)s_gen.phase;
    out[22] = s_gen.last_kind;
    out[23] = (uint8_t)((s_req[0] ? 1u : 0u) | (s_req[1] ? 2u : 0u));
}

// Kinds: 1 runt BCK pulse, 2 missing BCK pulse, 3 LRCLK edge k cells early, 4 k cells late.
WirecapResult wirecap_clkgen_glitch(uint8_t kind, uint8_t cells) {
    if (!s_gen.on) return WIRECAP_ERR_STATE;
    if (kind < 1 || kind > 4) return WIRECAP_ERR_VALUE;
    if (kind >= 3 && (cells < 1 || cells > 31)) return WIRECAP_ERR_VALUE;

    uint32_t g = (gen_frame_now() + GEN_GLITCH_LEAD) % GEN_FRAMES;
    uint32_t w[GEN_WORDS_PER_FRAME];
    if (kind <= 2) {
        gen_frame_words(w, s_gen.phase, kind);
        memcpy(&s_ring[g * GEN_WORDS_PER_FRAME], w, sizeof(w));
    } else {
        // Every frame from g on gets the shifted phase; the repair below rewrites the rest.
        uint32_t d = 4u * cells;
        s_gen.phase = (kind == 3 ? s_gen.phase + d : s_gen.phase + GEN_QUARTERS - d) & 255u;
        gen_frame_words(w, s_gen.phase, 0);
        for (uint32_t i = 0; i < GEN_FRAMES - GEN_GLITCH_LEAD - 3u; i++) {
            uint32_t f = (g + i) % GEN_FRAMES;
            memcpy(&s_ring[f * GEN_WORDS_PER_FRAME], w, sizeof(w));
        }
    }
    // Wait until the glitch frame has gone out (about 0.2 ms), then repair the ring here:
    // a main-loop repair runs late whenever the slip restarts the pipeline.
    uint32_t t0 = time_us_32();
    uint32_t wait_us = (uint32_t)(((uint64_t)(GEN_GLITCH_LEAD + 2u) * 1000000u) / s_gen.rate) + 1u;
    while (time_us_32() - t0 < wait_us) tight_loop_contents();
    gen_fill(s_gen.phase);
    uint32_t frames = (uint32_t)(((uint64_t)(time_us_32() - t0) * s_gen.rate) / 1000000u);
    if (frames >= GEN_FRAMES - 3u) s_gen.late++;
    s_gen.last_kind = kind;
    s_gen.glitches++;
    return WIRECAP_OK;
}

WirecapResult wirecap_request_rate(uint32_t rate) {
    if (rate != 44100u && rate != 48000u && rate != 96000u) return WIRECAP_ERR_VALUE;
    if (active_input_source != INPUT_SOURCE_USB) return WIRECAP_ERR_STATE;
    wirecap_usb_set_selected_rate(rate);
    return WIRECAP_OK;
}

void wirecap_poll(void) {
    cap_finish_if_done();
}

#endif // DSPI_WIRECAP
