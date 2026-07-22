/*
 * i2s_input.c - I2S receiver integration for DSPi
 *
 * Structure mirrors spdif_input.c with everything clock-related removed:
 * the input is synchronous to the device's own clock domain, so there is
 * no servo, no rate detection and no lock handling.
 *
 * Multichannel: the receiver fans out to up to I2S_RX_MAX_PAIRS stereo pairs
 * (2/4/6/8 channels on RP2350; always 1 pair / 2 channels on RP2040).  Each
 * pair is one PIO state machine + one IRQ-less DMA ring + one serial-data pin,
 * all sharing a single BCK/LRCLK:
 *
 *   clock master (no output slot is I2S): pair 0's SM runs the clkmaster
 *     program, driving BCK/LRCLK via side-set while sampling its data pin;
 *     pairs 1.. run the wait-driven slave program against the SAME pads that
 *     pair 0 drives.
 *   slave (at least one output slot is I2S): every pair runs the slave
 *     program against the BCK/LRCLK pads driven by the I2S TX clock master.
 *
 * Hardware:
 *   - PIO: PICO_SPDIF_RX_PIO, SM I2S_RX_SM_BASE + pair (SM 2 on RP2040,
 *     SMs 0..3 on RP2350), claimed only while running.  Free whenever SPDIF
 *     RX is inactive (inputs are switched, never mixed).
 *   - DMA: an IRQ-less infinite ring per pair, moving PIO RX FIFO words
 *     into a power-of-2-aligned ring (write-address wrap).  RP2350: one
 *     ENDLESS-mode data channel per pair (I2S_RX_DMA_BASE + p); the wrap
 *     makes it self-sustaining, no reload channel (mirrors the ADAT RX
 *     ring).  RP2040 (no ENDLESS mode): pair 0 uses I2S_RX_DMA_BASE (data)
 *     and +1 (reload); the data channel chains to the reload channel,
 *     which rewrites its write address and retriggers it.  Zero IRQs, so
 *     capture survives IRQ-disabled windows.  Pair 0 reuses the SPDIF RX
 *     channels (free whenever SPDIF input is inactive).
 *
 * Inter-channel alignment: all pairs are enabled on the same cycle
 * (pio_enable_sm_mask_in_sync), so every pair latches the same first frame
 * and the rings advance in lockstep on the one shared BCK/LRCLK.  The poll
 * reads the same frame index from every pair, so the 2/4/6/8 channels are
 * sample-aligned (the firmware's inviolable inter-channel alignment
 * guarantee).  This mirrors the TX path's audio_*_enable_sync().
 *
 * L/R framing: the PIO programs guarantee the first word pushed after a
 * (re)start is a LEFT word, and each ring holds an even number of words,
 * so a word's position in a ring fixes its channel permanently.  Even a
 * writer-laps-reader overrun garbles audio momentarily but can never
 * swap channels.
 */

#include "i2s_input.h"
#include "audio_input.h"
#include "audio_pipeline.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"
#include "notify.h"
#include "input_servo.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"

#include "i2s_input.pio.h"

#include <stdio.h>
#include <string.h>

// ============================================================================
// RESOURCES (shared with SPDIF RX, mutually exclusive by input switching)
// ============================================================================

#define i2s_rx_pio __CONCAT(pio, PICO_SPDIF_RX_PIO)

// First state machine used.  RP2350 has a dedicated PIO block (SMs 0..3 all
// available); RP2040 shares PIO1 where SM0 is PDM, so the input starts at SM2.
#if PICO_RP2350
#define I2S_RX_SM_BASE      0
#else
#define I2S_RX_SM_BASE      2
#endif

// DMA channels.  RP2350: pair p uses one ENDLESS-mode channel (base + p),
// so pairs 0..3 sit on 5..8 and 9..12 stay free (11/12 are the soft
// VCXO's).  RP2040: pair 0 uses (base) data + (base + 1) reload.  Pair 0
// reuses the SPDIF RX channels (free whenever SPDIF input is inactive).
#define I2S_RX_DMA_BASE     PICO_SPDIF_RX_DMA_CH0
#if PICO_RP2350
_Static_assert(I2S_RX_DMA_BASE + I2S_RX_MAX_PAIRS <= SOFT_VCXO_DMA_PULSE,
               "I2S RX DMA channels collide with the soft VCXO pair");
#else
_Static_assert(I2S_RX_DMA_BASE + 2 * I2S_RX_MAX_PAIRS <= SOFT_VCXO_DMA_PULSE,
               "I2S RX DMA channels collide with the soft VCXO pair");
#endif

// Ring sizing (per pair): must be a power of 2 (DMA address wrap) and even
// (L/R parity).  At 96 kHz stereo (192k words/s) this is ~5 ms of headroom on
// RP2040 and ~10 ms on RP2350; main-loop stalls longer than that only happen
// around flash operations, which suspend the input anyway.
#if PICO_RP2350
#define I2S_RX_RING_WORDS   2048u
#define I2S_RX_RING_BITS    13u                     // log2(ring bytes)
#else
#define I2S_RX_RING_WORDS   1024u
#define I2S_RX_RING_BITS    12u
#endif
#define I2S_RX_RING_BYTES   (I2S_RX_RING_WORDS * 4u)

// Minimum stereo frames to accumulate before feeding the pipeline.
//
// The DMA ring's write address advances per word, so the main loop (which
// polls far faster than samples arrive) would otherwise call
// process_input_block() with only a handful of frames each time.  The CPU
// meter is budget-based (busy_us / (frames / Fs)), so the fixed per-block
// cost (Core 1 EQ-worker handshake, pipeline setup) divided by a tiny frame
// count reads as a large inflation.  Batching to 48 frames matches the USB
// packet / consumer-buffer granularity, bringing I2S CPU in line with USB.
// At 48 kHz this adds ~1 ms of input latency; the consumer pool (50% prefill)
// absorbs the resulting sub-buffer fill ripple.
#define I2S_INPUT_MIN_BLOCK 48u

// Per-pair ring storage.  Each row is aligned to its own byte size so the DMA
// write-address wrap (channel_config_set_ring) stays within the pair's ring.
static uint32_t __attribute__((aligned(I2S_RX_RING_BYTES)))
    i2s_rx_ring[I2S_RX_MAX_PAIRS][I2S_RX_RING_WORDS];

// ============================================================================
// PER-PAIR STATE
// ============================================================================

// One receiver stereo pair = one SM + one DMA ring + one data pin.  Every
// lifecycle operation (start/stop/resync/poll) iterates i2s_n_pairs of these,
// so the single-pair path is simply n_pairs == 1 with no special-casing.
typedef struct {
    uint8_t   sm;          // PIO state machine index
    uint8_t   data_pin;    // this pair's serial-data GPIO (captured at start)
    uint8_t   dma_data;    // PIO RXF -> ring
#if !PICO_RP2350
    uint8_t   dma_reload;  // re-arms dma_data (RP2040 has no ENDLESS mode)
#endif
    uint32_t *ring;        // -> i2s_rx_ring[p]
    uintptr_t ring_base;   // ring address anchor (RP2040: reload source word)
    uint32_t  rd_word;     // software read index into this pair's ring (whole pairs)
} I2sRxPair;

static I2sRxPair i2s_pairs[I2S_RX_MAX_PAIRS];

// Active pairs this session = i2s_input_channels / 2 (clamped). Captured at
// start; stop()/poll() use it, never the live i2s_input_channels global (which
// the host may change between start and the deferred restart).
static uint8_t i2s_n_pairs;

// ============================================================================
// STATE
// ============================================================================

static volatile I2sInputState i2s_state = I2S_INPUT_INACTIVE;
static bool i2s_role_master = false;

// External-clock slave role (i2s_clock_mode == I2S_CLOCK_MODE_SLAVE at
// start): an external master drives BCK/LRCLK, both pads are inputs, and
// the i2s_slave_* section below owns rate detection and the output servo.
static bool i2s_role_extclk = false;

// Loaded PIO program offsets (-1 = not loaded).  The clkmaster program is
// loaded only in the master role (pair 0); the slave program is shared by all
// slave-role SMs (master role pairs 1.., or every pair in the slave role).
static int i2s_clkmaster_offset = -1;
static int i2s_slave_offset = -1;

// BCK pin captured at start.  stop() must release what was actually
// configured, NOT the live global: the hot-swap handlers update i2s_bck_pin
// before the deferred stop runs, so releasing the global would strand the old
// clock pins on the input PIO function.  (Data pins are captured per pair.)
static uint8_t i2s_active_bck_pin;

// RAM copy of the loaded slave program variant with the BCK/LRCLK GPIO
// numbers patched into the wait instructions (i2s_bck_pin is runtime
// configurable).  Sized for the larger (checked) variant; .length is set
// at load time to the variant actually loaded so stop()'s
// pio_remove_program frees exactly what was added.
#define I2S_RX_SLAVE_CHECKED_LEN \
    (sizeof(audio_i2s_rx_slave_checked_program_instructions) / sizeof(uint16_t))
static uint16_t i2s_slave_prog_ram[I2S_RX_SLAVE_CHECKED_LEN];
static struct pio_program i2s_slave_prog = {
    .instructions = i2s_slave_prog_ram,
    .length = 0,
    .origin = -1,
};

// PIO irq flag raised by the checked slave program on a framing slip.
// Block-local: nothing else on the SPDIF/I2S RX PIO raises PIO irq flags.
#define I2S_RX_SLIP_IRQ 7u

// ============================================================================
// HELPERS
// ============================================================================

// 24.8 fixed-point divider for the clock-master role; identical math to
// the I2S TX library (2x the SPDIF ceiling divider, see
// i2s_compute_divider) so input BCK matches output BCK exactly.
static uint32_t rx_master_divider_24_8(uint32_t sample_freq) {
    uint32_t sys = clock_get_hz(clk_sys);
    return 2u * (sys / sample_freq + (sys % sample_freq != 0));
}

// Patch the 5-bit GPIO index field of a `wait gpio` instruction.
static inline uint16_t patch_wait_gpio(uint16_t instr, uint8_t pin) {
    return (uint16_t)((instr & ~0x1Fu) | (pin & 0x1Fu));
}

// Current DMA write position (in words) within a pair's ring.
static inline uint32_t pair_write_word(const I2sRxPair *pr) {
    return (uint32_t)((dma_hw->ch[pr->dma_data].write_addr - (uint32_t)pr->ring_base) / 4u) %
           I2S_RX_RING_WORDS;
}

// Wait (bounded) for DMA to drain one SM's PIO RX FIFO.  Used before
// re-anchoring a read pointer so no in-flight words land after the anchor.
static void drain_rx_fifo(uint8_t sm) {
    for (uint32_t spin = 0; spin < 10000; spin++) {
        if (pio_sm_get_rx_fifo_level(i2s_rx_pio, sm) == 0) break;
        tight_loop_contents();
    }
}

// Patch and load one of the two wait-driven slave program variants (BCK and
// LRCLK pins are runtime config).  checked = the external-clock variant with
// per-frame LRCLK framing verification; plain = the minimal free-running
// variant for on-chip clocks (master role pairs 1..), which must stay small
// enough to share program memory with the clkmaster program.
//
// Both variants are authored with placeholder GPIO indices in their `wait
// gpio` instructions (0 = BCK, 1 = LRCLK); every WAIT-source-GPIO opcode
// gets its 5-bit index rewritten here.
static void load_slave_program(bool checked) {
    const uint16_t *src = checked ? audio_i2s_rx_slave_checked_program_instructions
                                  : audio_i2s_rx_slave_program_instructions;
    uint8_t len = checked ? audio_i2s_rx_slave_checked_program.length
                          : audio_i2s_rx_slave_program.length;
    memcpy(i2s_slave_prog_ram, src, (size_t)len * sizeof(uint16_t));
    for (uint8_t i = 0; i < len; i++) {
        uint16_t instr = i2s_slave_prog_ram[i];
        if ((instr >> 13) != 0x1u) continue;          // not a WAIT
        if (((instr >> 5) & 0x3u) != 0u) continue;    // WAIT source not GPIO
        uint8_t pin = (instr & 0x1Fu) ? (uint8_t)(i2s_active_bck_pin + 1)
                                      : i2s_active_bck_pin;
        i2s_slave_prog_ram[i] = patch_wait_gpio(instr, pin);
    }
    i2s_slave_prog.length = len;
    i2s_slave_offset = pio_add_program(i2s_rx_pio, &i2s_slave_prog);
}

// Set up one pair's IRQ-less ring.
static void start_pair_dma_ring(const I2sRxPair *pr) {
#if PICO_RP2350
    // Data channel: PIO RX FIFO -> ring, write-address wrap + ENDLESS
    // transfer count; self-sustaining with no reload channel (mirrors the
    // ADAT RX ring in adat_input.c).
    dma_channel_config ca = dma_channel_get_default_config(pr->dma_data);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_32);
    channel_config_set_read_increment(&ca, false);
    channel_config_set_write_increment(&ca, true);
    channel_config_set_ring(&ca, true, I2S_RX_RING_BITS);
    channel_config_set_dreq(&ca, pio_get_dreq(i2s_rx_pio, pr->sm, false));
    dma_channel_set_config(pr->dma_data, &ca, false);
    dma_channel_set_read_addr(pr->dma_data, &i2s_rx_pio->rxf[pr->sm], false);
    dma_channel_set_write_addr(pr->dma_data, pr->ring, false);
    dma_channel_set_trans_count(pr->dma_data,
                                dma_encode_endless_transfer_count(), true);
#else
    // Reload channel: one word, no increments, chain-to-self.  Each completion
    // of the data channel chains here; this writes the ring base into the data
    // channel's write-address trigger alias, which also reloads its transfer
    // count.  Runs forever, zero IRQs.
    dma_channel_config cb = dma_channel_get_default_config(pr->dma_reload);
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_32);
    channel_config_set_read_increment(&cb, false);
    channel_config_set_write_increment(&cb, false);
    channel_config_set_chain_to(&cb, pr->dma_reload);
    dma_channel_configure(pr->dma_reload, &cb,
                          &dma_hw->ch[pr->dma_data].al2_write_addr_trig,
                          &pr->ring_base, 1, false);

    // Data channel: PIO RX FIFO -> ring with write-address wrap.
    dma_channel_config ca = dma_channel_get_default_config(pr->dma_data);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_32);
    channel_config_set_read_increment(&ca, false);
    channel_config_set_write_increment(&ca, true);
    channel_config_set_ring(&ca, true, I2S_RX_RING_BITS);
    channel_config_set_dreq(&ca, pio_get_dreq(i2s_rx_pio, pr->sm, false));
    channel_config_set_chain_to(&ca, pr->dma_reload);
    dma_channel_configure(pr->dma_data, &ca,
                          pr->ring, &i2s_rx_pio->rxf[pr->sm],
                          I2S_RX_RING_WORDS, true);
#endif
}

// Race-free teardown of every pair's ring.
//
// RP2350: ENDLESS-mode data channels only; a single grouped abort is enough.
//
// RP2040: each data channel chains to its reload channel, and the reload
// channel re-triggers the data channel by writing its al2_write_addr_trig.
// Aborting these naively (sequential dma_channel_abort calls) lets one
// channel re-arm the other in the gap, so dma_channel_abort's `while (BUSY)`
// spin never returns (watchdog reset) or a channel is left live after
// unclaim.  Break the loops deterministically: first disarm every data
// channel's chain via the non-triggering CTRL alias (point chain_to at
// itself), then abort all channels in a single dma_hw->abort write so none
// can re-arm another.  The bounded guard degrades a hardware quirk to a
// clean stop rather than a hang.
static void stop_all_dma_rings(void) {
    uint32_t abort_mask = 0;
    for (uint8_t p = 0; p < i2s_n_pairs; p++) {
        const I2sRxPair *pr = &i2s_pairs[p];
#if PICO_RP2350
        // ENDLESS channels have no chain loop; a plain abort suffices.
        abort_mask |= (1u << pr->dma_data);
#else
        hw_write_masked(&dma_hw->ch[pr->dma_data].al1_ctrl,
                        (uint32_t)pr->dma_data << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
                        DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
        abort_mask |= (1u << pr->dma_data) | (1u << pr->dma_reload);
#endif
    }

    dma_hw->abort = abort_mask;

    uint32_t guard = 1000000u;
    while (guard--) {
        bool any_busy = false;
        for (uint8_t p = 0; p < i2s_n_pairs; p++) {
            if ((dma_hw->ch[i2s_pairs[p].dma_data].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS)
#if !PICO_RP2350
                || (dma_hw->ch[i2s_pairs[p].dma_reload].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS)
#endif
                ) {
                any_busy = true;
                break;
            }
        }
        if (!any_busy) break;
        tight_loop_contents();
    }

#if PICO_RP2350
    // Leave ENDLESS mode so a stale encoded count can't confuse a future
    // NORMAL-mode reuse of these channels (mirrors adat_input_stop).
    for (uint8_t p = 0; p < i2s_n_pairs; p++)
        dma_channel_hw_addr(i2s_pairs[p].dma_data)->transfer_count = 0;
#endif
}

// Build the SM enable bitmask for the active pairs.
static inline uint32_t i2s_sm_mask(void) {
    uint32_t mask = 0;
    for (uint8_t p = 0; p < i2s_n_pairs; p++) mask |= (1u << i2s_pairs[p].sm);
    return mask;
}

// ============================================================================
// PUBLIC API
// ============================================================================

void i2s_input_init(void) {
    i2s_state = I2S_INPUT_INACTIVE;
    // Ring addresses are fixed for the life of the program; ring_base
    // anchors the write-pointer math (and, on RP2040, is the stable source
    // word the reload DMA channel reads).
    for (uint8_t p = 0; p < I2S_RX_MAX_PAIRS; p++) {
        i2s_pairs[p].ring = i2s_rx_ring[p];
        i2s_pairs[p].ring_base = (uintptr_t)i2s_rx_ring[p];
    }
}

// Forward decl: (re)arm the slave-mode measurement state (defined in the
// external-clock slave section below).
static void i2s_slave_arm(void);
static void i2s_slave_disarm(void);

void i2s_input_start(bool clock_master) {
    // Guard against double-start (would panic on resource re-claim).
    if (i2s_state != I2S_INPUT_INACTIVE) return;

    // External-clock slave role overrides the master election: an external
    // device owns BCK/LRCLK, so the input SM never generates clocks (the
    // caller already passes clock_master == false; this is defensive).
    i2s_role_extclk = (i2s_clock_mode == I2S_CLOCK_MODE_SLAVE);
    if (i2s_role_extclk) clock_master = false;

    // Number of stereo pairs = active channels / 2, clamped to the platform
    // maximum.  Each pair claims one SM, its DMA channel(s) and one data pin.
    uint8_t pairs = (uint8_t)(i2s_input_channels / 2u);
    if (pairs < 1) pairs = 1;
    if (pairs > I2S_RX_MAX_PAIRS) pairs = I2S_RX_MAX_PAIRS;
    i2s_n_pairs = pairs;
    i2s_role_master = clock_master;
    // Effective pair: SPLIT pin mode routes the external-clock role onto its
    // own BCK/LRCLK pair; every other role uses the master/unified pair.
    i2s_active_bck_pin = i2s_effective_bck_pin();

    // Assign and claim per-pair resources.
    for (uint8_t p = 0; p < pairs; p++) {
        I2sRxPair *pr = &i2s_pairs[p];
        pr->sm         = (uint8_t)(I2S_RX_SM_BASE + p);
        pr->data_pin   = i2s_rx_pin[p];
#if PICO_RP2350
        pr->dma_data   = (uint8_t)(I2S_RX_DMA_BASE + p);
#else
        pr->dma_data   = (uint8_t)(I2S_RX_DMA_BASE + 2u * p);
        pr->dma_reload = (uint8_t)(I2S_RX_DMA_BASE + 2u * p + 1u);
#endif
        pr->rd_word    = 0;

        pio_sm_claim(i2s_rx_pio, pr->sm);
        dma_channel_claim(pr->dma_data);
#if !PICO_RP2350
        dma_channel_claim(pr->dma_reload);
#endif

        // Defensive: SPDIF RX shares the low channels on DMA_IRQ_1; clear any
        // stale per-channel IRQ enables since these rings run IRQ-less.
        dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, pr->dma_data, false);
        dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, pr->dma_data);
#if !PICO_RP2350
        dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, pr->dma_reload, false);
        dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, pr->dma_reload);
#endif

        // Data pin: input path only.  pio_gpio_init sets pad IE and clears the
        // RP2350 pad isolation latch; the SM never drives it (pindir input).
        pio_gpio_init(i2s_rx_pio, pr->data_pin);
    }

    if (clock_master) {
        // We own BCK/LRCLK: route them to this PIO block and enable their input
        // buffers so pairs 1.. (slave program) can read the pads pair 0 drives.
        pio_gpio_init(i2s_rx_pio, i2s_active_bck_pin);
        pio_gpio_init(i2s_rx_pio, i2s_active_bck_pin + 1);
        gpio_set_input_enabled(i2s_active_bck_pin, true);
        gpio_set_input_enabled(i2s_active_bck_pin + 1, true);

        // Pair 0: clock-generating program + matched 24.8 divider.
        i2s_clkmaster_offset = pio_add_program(i2s_rx_pio, &audio_i2s_rx_clkmaster_program);
        audio_i2s_rx_clkmaster_program_init(i2s_rx_pio, i2s_pairs[0].sm,
                                            (uint)i2s_clkmaster_offset,
                                            i2s_pairs[0].data_pin, i2s_active_bck_pin);
        uint32_t div = rx_master_divider_24_8(audio_state.freq);
        pio_sm_set_clkdiv_int_frac(i2s_rx_pio, i2s_pairs[0].sm,
                                   (uint16_t)(div >> 8u), (uint8_t)(div & 0xFFu));

        // Pairs 1..: wait-driven slave program against the driven BCK/LRCLK
        // pads.  Plain (unchecked) variant: on-chip clocks are glitch-free,
        // and the checked variant would not fit alongside the clkmaster
        // program in the block's instruction memory.
        if (pairs > 1) {
            load_slave_program(false);
            for (uint8_t p = 1; p < pairs; p++) {
                audio_i2s_rx_slave_program_init(i2s_rx_pio, i2s_pairs[p].sm,
                                                (uint)i2s_slave_offset,
                                                i2s_pairs[p].data_pin);
                pio_sm_set_clkdiv_int_frac(i2s_rx_pio, i2s_pairs[p].sm, 1, 0);
            }
        }
    } else {
        // Clocks come from elsewhere: the I2S TX clock master (another PIO
        // block), or an EXTERNAL master in the extclk role.  In the extclk
        // role nothing on-chip owns the pads yet, so configure them as plain
        // inputs first (SIO function clears the RP2350 pad isolation latch,
        // direction stays input; idempotent with the extclk TX output setup).
        if (i2s_role_extclk) {
            gpio_init(i2s_active_bck_pin);
            gpio_init(i2s_active_bck_pin + 1);
        }
        // Make sure the BCK/LRCLK input buffers are on (belt and braces in
        // the on-chip slave role), then run the slave program on every pair.
        //
        // Variant selection: the external-clock role runs the CHECKED
        // program (per-frame LRCLK framing verification + slip flag) because
        // real-world external masters glitch and re-frame their clocks; the
        // on-chip slave role keeps the plain free-running program (our own
        // TX master never glitches, and explicit i2s_input_resync() calls
        // already re-phase it around output restarts).
        gpio_set_input_enabled(i2s_active_bck_pin, true);
        gpio_set_input_enabled(i2s_active_bck_pin + 1, true);

        load_slave_program(i2s_role_extclk);
        for (uint8_t p = 0; p < pairs; p++) {
            if (i2s_role_extclk) {
                audio_i2s_rx_slave_checked_program_init(i2s_rx_pio, i2s_pairs[p].sm,
                                                        (uint)i2s_slave_offset,
                                                        i2s_pairs[p].data_pin,
                                                        (uint)(i2s_active_bck_pin + 1));
            } else {
                audio_i2s_rx_slave_program_init(i2s_rx_pio, i2s_pairs[p].sm,
                                                (uint)i2s_slave_offset,
                                                i2s_pairs[p].data_pin);
            }
            pio_sm_set_clkdiv_int_frac(i2s_rx_pio, i2s_pairs[p].sm, 1, 0);
        }
    }

    // DMA rings before enabling the SMs (the SMs are disabled with clean FIFOs
    // after pio_sm_init), so the first pushed word lands at each ring base =
    // LEFT word.
    for (uint8_t p = 0; p < pairs; p++) {
        i2s_pairs[p].rd_word = 0;
        start_pair_dma_ring(&i2s_pairs[p]);
    }

    // Master pair 0 start procedure: preload the bit counter and enter at the
    // start of a left frame.  Done while disabled; the sync-enable below does
    // not touch the PC or registers, so this survives.
    if (clock_master) {
        pio_sm_exec(i2s_rx_pio, i2s_pairs[0].sm, pio_encode_set(pio_x, 29));
        pio_sm_exec(i2s_rx_pio, i2s_pairs[0].sm,
                    pio_encode_jmp((uint)i2s_clkmaster_offset +
                                   audio_i2s_rx_clkmaster_wrap_target));
    }
    // Slave SMs: pio_sm_init left the PC at the program entry point.

    // A stale framing-slip flag from a previous session would trip the slip
    // watchdog immediately after the next lock; this (re)start IS the slip
    // handling, so consume it.
    if (i2s_role_extclk) pio_interrupt_clear(i2s_rx_pio, I2S_RX_SLIP_IRQ);

    // Enable every pair on the SAME cycle so the rings advance in lockstep on
    // the one shared BCK/LRCLK.  (Mirrors audio_*_enable_sync() on the TX path.)
    pio_enable_sm_mask_in_sync(i2s_rx_pio, i2s_sm_mask());

    // Master-role multichannel frame-skew correction.
    //
    // Pair 0 runs the clkmaster program: it generates BCK/LRCLK and begins
    // sampling immediately, so its ring index 0 is the FIRST frame it drives
    // ("frame 0").  The other pairs run the wait-driven slave program; because
    // LRCLK is already low at enable, their preamble cannot detect the start of
    // frame 0 and instead locks on the next LRCLK fall, making their first
    // sample frame 1 (ring index 0 = "frame 1").  At equal ring indices the
    // slaves therefore lead pair 0 by exactly one frame.  Advancing pair 0's
    // read pointer by one stereo frame (2 words) lands every pair's read
    // pointer on the same physical frame; from there all rings advance in
    // lockstep, so the 2/4/6/8 channels stay sample-aligned (the inviolable
    // inter-channel guarantee).  The offset is cycle-exact per the clkmaster vs
    // slave PIO start timing and is independent of ADC settling latency (a
    // garbage frame 0 is simply discarded by the skip).  Single-pair master
    // has no peer to align with, and the slave role is already symmetric (every
    // pair, including pair 0, runs the same preamble), so neither needs it.
    if (clock_master && pairs > 1)
        i2s_pairs[0].rd_word = 2;

    // Slave role: arm the rate measurement / lock state machine against the
    // freshly anchored pair-0 ring.
    if (i2s_role_extclk) i2s_slave_arm();

    i2s_state = I2S_INPUT_RUNNING;
    printf("I2S RX: started %u channel(s) on GPIO", (unsigned)(pairs * 2u));
    for (uint8_t p = 0; p < pairs; p++) printf(" %u", i2s_pairs[p].data_pin);
    printf(" (%s)\n", clock_master ? "clock master"
                                   : (i2s_role_extclk ? "external-clock slave" : "slave"));
}

void i2s_input_stop(void) {
    if (i2s_state == I2S_INPUT_INACTIVE) return;

    // Disable all SMs, then tear down every ring race-free (see
    // stop_all_dma_rings) before unclaiming anything.
    for (uint8_t p = 0; p < i2s_n_pairs; p++) {
        pio_sm_set_enabled(i2s_rx_pio, i2s_pairs[p].sm, false);
    }
    stop_all_dma_rings();

    // Remove the loaded program(s).
    if (i2s_clkmaster_offset >= 0) {
        pio_remove_program(i2s_rx_pio, &audio_i2s_rx_clkmaster_program,
                           (uint)i2s_clkmaster_offset);
        i2s_clkmaster_offset = -1;
    }
    if (i2s_slave_offset >= 0) {
        pio_remove_program(i2s_rx_pio, &i2s_slave_prog, (uint)i2s_slave_offset);
        i2s_slave_offset = -1;
    }

    // Release BCK/LRCLK to high-Z (master role only).  If an output master is
    // taking over it re-initializes them on its own PIO block immediately after.
    if (i2s_role_master) {
        gpio_set_function(i2s_active_bck_pin, GPIO_FUNC_NULL);
        gpio_set_dir(i2s_active_bck_pin, GPIO_IN);
        gpio_set_function(i2s_active_bck_pin + 1, GPIO_FUNC_NULL);
        gpio_set_dir(i2s_active_bck_pin + 1, GPIO_IN);
    }

    // Release each pair's data pin and free its SM + DMA channels.
    for (uint8_t p = 0; p < i2s_n_pairs; p++) {
        I2sRxPair *pr = &i2s_pairs[p];
        gpio_set_function(pr->data_pin, GPIO_FUNC_NULL);
        gpio_set_dir(pr->data_pin, GPIO_IN);
        pio_sm_unclaim(i2s_rx_pio, pr->sm);
        dma_channel_unclaim(pr->dma_data);
#if !PICO_RP2350
        dma_channel_unclaim(pr->dma_reload);
#endif
    }

    if (i2s_role_extclk) i2s_slave_disarm();

    i2s_state = I2S_INPUT_INACTIVE;
    printf("I2S RX: stopped\n");
}

void i2s_input_resync(void) {
    // Only running ON-CHIP slaves need re-phasing: the TX clock master
    // restarts from its PIO entry point during synchronized output starts,
    // which resets LRCLK phase under our bit counters.  The master role
    // generates its own clocks; the external-clock role's LRCLK never
    // glitches on an output restart.  Both are unaffected.
    if (i2s_state != I2S_INPUT_RUNNING || i2s_role_master || i2s_role_extclk) return;

    // Disable every pair, let DMA drain what each SM already pushed, then
    // anchor each read pointer at its current write position: the next word the
    // re-entered program pushes is a LEFT word.
    for (uint8_t p = 0; p < i2s_n_pairs; p++) {
        pio_sm_set_enabled(i2s_rx_pio, i2s_pairs[p].sm, false);
    }
    for (uint8_t p = 0; p < i2s_n_pairs; p++) {
        I2sRxPair *pr = &i2s_pairs[p];
        drain_rx_fifo(pr->sm);
        pio_sm_restart(i2s_rx_pio, pr->sm);   // clears ISR shift counter
        pr->rd_word = pair_write_word(pr);
        pio_sm_exec(i2s_rx_pio, pr->sm,
                    pio_encode_jmp((uint)i2s_slave_offset +
                                   audio_i2s_rx_slave_offset_entry_point));
    }

    // Re-enable all pairs on the same cycle so they re-acquire the same frame.
    pio_enable_sm_mask_in_sync(i2s_rx_pio, i2s_sm_mask());
}

// ============================================================================
// MAIN-LOOP POLL
// ============================================================================

DSP_TIME_CRITICAL
uint32_t i2s_input_poll(void) {
    if (i2s_state != I2S_INPUT_RUNNING) return 0;

    // Frames to consume this poll = the minimum available across all pairs.
    // The pairs fill in lockstep on the shared clock, so this is the common
    // frame count; taking the minimum keeps every channel on the same frame
    // index even under a word of DMA timing jitter between rings.
    uint32_t frames = 192u;   // buf capacity cap
    for (uint8_t p = 0; p < i2s_n_pairs; p++) {
        uint32_t avail = (pair_write_word(&i2s_pairs[p]) - i2s_pairs[p].rd_word) %
                         I2S_RX_RING_WORDS;
        uint32_t f = avail / 2u;
        if (f < frames) frames = f;
    }
    // Batch: wait for a pipeline-sized block (see I2S_INPUT_MIN_BLOCK).  The
    // input is continuous, so the minimum always climbs to the threshold.
    if (frames < I2S_INPUT_MIN_BLOCK) return 0;

    // Deinterleave each pair into its two pipeline input channels (2p, 2p+1).
    // Inputs 0/1 are the shared stereo bus (buf_l/buf_r); 2.. are buf_in_ext.
    for (uint8_t p = 0; p < i2s_n_pairs; p++) {
        I2sRxPair *pr = &i2s_pairs[p];
        int ch_l = 2 * p, ch_r = 2 * p + 1;

#if PICO_RP2350
        float *out_l = (ch_l == 0) ? buf_l : buf_in_ext[ch_l - NUM_STEREO_INPUTS];
        float *out_r = (ch_r == 1) ? buf_r : buf_in_ext[ch_r - NUM_STEREO_INPUTS];
        float preamp_l = global_preamp_linear[ch_l];
        float preamp_r = global_preamp_linear[ch_r];
        const float inv_2147483648 = 1.0f / 2147483648.0f;
#else
        // RP2040 is single-pair (channels 0/1 only); Q28 path.
        int32_t *out_l = buf_l;
        int32_t *out_r = buf_r;
        int32_t preamp_l = global_preamp_mul[ch_l];
        int32_t preamp_r = global_preamp_mul[ch_r];
#endif

        uint32_t idx = pr->rd_word;
        for (uint32_t i = 0; i < frames; i++) {
            // 24-bit audio in bits [31:8]; mask the don't-care low byte.
            int32_t raw_l = (int32_t)(pr->ring[idx] & 0xFFFFFF00u);
            idx = (idx + 1u) % I2S_RX_RING_WORDS;
            int32_t raw_r = (int32_t)(pr->ring[idx] & 0xFFFFFF00u);
            idx = (idx + 1u) % I2S_RX_RING_WORDS;

#if PICO_RP2350
            out_l[i] = (float)raw_l * inv_2147483648 * preamp_l;
            out_r[i] = (float)raw_r * inv_2147483648 * preamp_r;
#else
            // Q28: int32 full-scale >> 2 -> Q28, then preamp; matches the
            // SPDIF RX and USB 24-bit paths so output unity gain holds.
            out_l[i] = fast_mul_q28(raw_l >> 2, preamp_l);
            out_r[i] = fast_mul_q28(raw_r >> 2, preamp_r);
#endif
        }
        pr->rd_word = idx;
    }

#if PICO_RP2350
    // Live channel-count RAISE guard.  When the host raises i2s_input_channels
    // while I2S is the running source, the pipeline immediately sees the larger
    // active count, but the extra pairs are not allocated/filled until the
    // deferred restart (main loop) fires.  This poll has filled only the
    // currently-running pairs (i2s_n_pairs), so zero the extra input rows the
    // matrix will read (it iterates active_input_channel_count()) — it then sees
    // silence, never stale buf_in_ext content, until the restart fills them.
    // (A count DROP needs nothing: the matrix simply stops reading the surplus
    // rows.)  RP2040 is single-pair, so this never triggers; it is RP2350-only
    // because buf_in_ext exists only there.
    uint32_t active = active_input_channel_count();
    for (uint32_t ch = 2u * (uint32_t)i2s_n_pairs; ch < active; ch++)
        memset(buf_in_ext[ch - NUM_STEREO_INPUTS], 0,
               frames * sizeof(buf_in_ext[0][0]));
#endif

    process_input_block(frames);
    return frames;
}

// Push one silent block through the DSP pipeline to prefill the output consumer
// pools when the input cannot supply samples itself.
//
// This is only used during a SLAVE-role prefill.  In slave mode the input is
// clocked by the I2S output clock master, so draining the outputs to prefill
// (the SPDIF-style handshake) also stops the input's BCK/LRCLK and no input
// samples arrive; the pools could never reach the 50% target and outputs would
// never re-enable.  Synthesizing silence fills the pools deterministically;
// real audio resumes after enable_outputs_in_sync() restarts the clock master
// and re-phases the input rings.  Master-role prefill uses real input audio via
// i2s_input_poll() and never calls this.
void i2s_input_prefill_silence(uint32_t frames) {
    if (frames == 0) return;
    if (frames > 192u) frames = 192u;   // buf capacity

    // Zero every active input channel so the matrix sees silence on all of
    // them (stale buf_in_ext content must not leak through in multichannel).
    uint8_t pairs = (uint8_t)(i2s_input_channels / 2u);
    if (pairs < 1) pairs = 1;
    if (pairs > I2S_RX_MAX_PAIRS) pairs = I2S_RX_MAX_PAIRS;
    memset(buf_l, 0, frames * sizeof(buf_l[0]));
    memset(buf_r, 0, frames * sizeof(buf_r[0]));
#if PICO_RP2350
    for (uint8_t p = 1; p < pairs; p++) {
        memset(buf_in_ext[2 * p - NUM_STEREO_INPUTS], 0, frames * sizeof(buf_in_ext[0][0]));
        memset(buf_in_ext[2 * p + 1 - NUM_STEREO_INPUTS], 0, frames * sizeof(buf_in_ext[0][0]));
    }
#endif

    process_input_block(frames);
}

// ============================================================================
// EXTERNAL-CLOCK SLAVE MODE: rate measurement, lock state machine, servo
// ============================================================================
//
// The external master owns BCK/LRCLK, so the device must (a) discover the
// sample rate, (b) notice the clocks stopping or changing, and (c) keep the
// sys_clk-domain outputs (SPDIF, ADAT) rate-matched to the external clock
// domain.  All three derive from one observable: pair 0's DMA write pointer,
// which advances at exactly 2 words per frame of external LRCLK.
//
// Fast window (~32 ms): rate snap and lock/loss decisions, ~30 ppm precision.
// Long window (8-16 s dual anchor): servo rate reference, ~0.1 ppm; precise
// enough that ADAT stays rate-locked even with no SPDIF slot fill to observe.
// Fill trim: from the first SPDIF-type slot's consumer fill (shared terms
// in input_servo.c).  Edge-locked I2S slots consume at exactly the external
// rate, so their fill carries no clock information; slot 0 alone is NOT a
// valid reference here, unlike the SPDIF input servo.

#define I2S_SLAVE_WINDOW_US         32000u     // fast measurement window
#define I2S_SLAVE_CLOCK_TIMEOUT_US  5000u      // no words this long = clocks gone
#define I2S_SLAVE_LOCK_WINDOWS      2          // agreeing windows needed to lock
#define I2S_SLAVE_LONG_HALF_US      8000000ull // long-window anchor rotation
#define I2S_SLAVE_SERVO_INTERVAL    1000       // main-loop iterations (~20 ms)

static volatile I2sSlaveState i2s_slave_state = I2S_SLAVE_INACTIVE;
static uint8_t  i2s_slave_lock_count = 0;      // cumulative since boot
static uint8_t  i2s_slave_loss_count = 0;
static uint8_t  i2s_slave_slip_count = 0;      // framing slips, cumulative
static uint32_t i2s_slave_detected_rate = 0;   // snapped Hz, valid when LOCKED

// Word accumulation (pair 0)
static uint32_t meas_last_word;
static uint64_t meas_total_words;
static uint64_t meas_last_advance_us;

// Fast window
static uint64_t meas_win_us;
static uint64_t meas_win_words;
static float    meas_hz_smooth;                // EMA of fast windows
static uint32_t meas_measured_hz;              // last fast-window result
static uint32_t lock_candidate;
static uint8_t  lock_agree;

// Long window: [0] = older anchor, [1] = newer; rate measured [0] -> now
static uint64_t long_us[2];
static uint64_t long_words[2];
static bool     long_valid;
static float    meas_hz_long;

// Servo
static uint32_t i2s_slave_servo_skip = 0;

// Poll-gap watchdog: a main-loop stall longer than the ring-fill time can
// wrap the DMA write pointer a whole ring between polls, silently losing
// words from the accumulator (flash ops suspend the input, but a heavy
// bulk apply or preset load does not).  Above this gap, re-anchor instead
// of measuring through it.  4 ms is under the worst ring fill (~5.3 ms:
// RP2040 ring at 96 kHz stereo).
#define I2S_SLAVE_POLL_GAP_US  4000u
static uint64_t meas_last_poll_us;

// (Re)arm measurement against the freshly anchored pair-0 ring.  Lock/loss
// counters deliberately persist (cumulative diagnostics, like SPDIF's).
static void i2s_slave_arm(void) {
    uint64_t now = time_us_64();
    meas_last_word = pair_write_word(&i2s_pairs[0]);
    meas_total_words = 0;
    meas_last_advance_us = now;
    meas_last_poll_us = now;
    meas_win_us = now;
    meas_win_words = 0;
    meas_hz_smooth = 0.0f;
    meas_measured_hz = 0;
    i2s_slave_detected_rate = 0;
    lock_candidate = 0;
    lock_agree = 0;
    long_valid = false;
    i2s_slave_servo_skip = 0;
    input_servo_reset();
    i2s_slave_state = I2S_SLAVE_ACQUIRING;
    notify_push_i2s_slave_state(I2S_SLAVE_ACQUIRING, 0);
}

static void i2s_slave_disarm(void) {
    i2s_slave_state = I2S_SLAVE_INACTIVE;
    i2s_slave_detected_rate = 0;
    meas_measured_hz = 0;
    input_servo_reset();   // park the VCXO; no longer tracking
    notify_push_i2s_slave_state(I2S_SLAVE_INACTIVE, 0);
}

// Snap a measured rate to a supported one (2% tolerance; supported rates
// are >8% apart so the windows can never overlap). 0 = no match.
static uint32_t i2s_slave_snap_rate(float hz) {
    static const uint32_t rates[3] = {44100, 48000, 96000};
    for (int i = 0; i < 3; i++) {
        float r = (float)rates[i];
        float d = hz - r;
        if (d < 0) d = -d;
        if (d <= r * 0.02f) return rates[i];
    }
    return 0;
}

// Common lock-drop bookkeeping (clock loss or rate change while LOCKED)
static void i2s_slave_drop_lock(void) {
    i2s_slave_state = I2S_SLAVE_RELOCKING;
    if (i2s_slave_loss_count < 255) i2s_slave_loss_count++;
    i2s_slave_detected_rate = 0;
    long_valid = false;
    lock_candidate = 0;
    lock_agree = 0;
    notify_push_i2s_slave_state(I2S_SLAVE_RELOCKING, 0);
}

// Read and clear the framing-slip flags from both PIO blocks: the RX SMs
// (checked slave program) and the external-clock I2S output SMs.
DSP_TIME_CRITICAL
static bool i2s_slave_slip_check(void) {
    bool slip = false;
    if (pio_interrupt_get(i2s_rx_pio, I2S_RX_SLIP_IRQ)) {
        pio_interrupt_clear(i2s_rx_pio, I2S_RX_SLIP_IRQ);
        slip = true;
    }
    if (audio_i2s_extclk_framing_slipped()) slip = true;
    return slip;
}

DSP_TIME_CRITICAL
void i2s_slave_poll(void) {
    if (i2s_slave_state == I2S_SLAVE_INACTIVE || i2s_state != I2S_INPUT_RUNNING)
        return;

    // Framing-slip watchdog.  The checked RX/TX programs verify LRCLK phase
    // at every frame boundary; a BCK glitch or LRCLK phase jump from the
    // external master (Amanero-style re-clocking around stream stop/start
    // or rate switches) sets a PIO irq flag and the program re-frames
    // itself.  A slip leaves the word RATE unchanged, so the rate watchdog
    // below can never see one; and a slipped pair/slot is no longer
    // sample-aligned with its peers, which only a full restart can fix.
    // Treat the flag exactly like a clock loss: drop the lock and let the
    // main loop's RELOCKING path restart the receiver and re-frame every
    // output via the prefill's gated synchronized start.
    if (i2s_slave_slip_check()) {
        if (i2s_slave_slip_count < 255) i2s_slave_slip_count++;
        if (i2s_slave_state != I2S_SLAVE_RELOCKING) {
            i2s_slave_drop_lock();
            meas_measured_hz = 0;
        }
        return;
    }

    uint64_t now = time_us_64();
    uint32_t w = pair_write_word(&i2s_pairs[0]);

    // Poll-gap watchdog: after a stall the word delta is ambiguous (the DMA
    // pointer may have lapped the ring), so re-anchor everything and skip
    // this poll's judgements rather than risk a spurious lock drop.  The
    // long window is word-count based, so it must reset too; the servo
    // falls back to the fast-window EMA until it re-fills (~8 s).
    uint64_t poll_gap = now - meas_last_poll_us;
    meas_last_poll_us = now;
    if (poll_gap > I2S_SLAVE_POLL_GAP_US) {
        meas_last_word = w;
        meas_last_advance_us = now;
        meas_win_us = now;
        meas_win_words = meas_total_words;
        long_us[0] = long_us[1] = now;
        long_words[0] = long_words[1] = meas_total_words;
        long_valid = false;
        return;
    }

    uint32_t delta = (w + I2S_RX_RING_WORDS - meas_last_word) % I2S_RX_RING_WORDS;
    meas_last_word = w;
    meas_total_words += delta;
    if (delta) meas_last_advance_us = now;

    // Clock-presence timeout: even 44.1 kHz pushes a word every ~11 us, so
    // 5 ms of silence is unambiguous loss.  The main loop reacts to
    // RELOCKING exactly like a SPDIF lock loss (mute, drain, wait).
    if (i2s_slave_state == I2S_SLAVE_LOCKED &&
        (now - meas_last_advance_us) > I2S_SLAVE_CLOCK_TIMEOUT_US) {
        i2s_slave_drop_lock();
        meas_measured_hz = 0;
        meas_win_us = now;              // fresh window after the gap
        meas_win_words = meas_total_words;
        return;
    }

    uint64_t span = now - meas_win_us;
    if (span < I2S_SLAVE_WINDOW_US) return;

    // Fast-window rate: words/2 frames over the elapsed span
    float hz = (float)(uint32_t)(meas_total_words - meas_win_words) *
               (1e6f / 2.0f) / (float)span;
    meas_win_us = now;
    meas_win_words = meas_total_words;
    meas_measured_hz = (uint32_t)(hz + 0.5f);

    uint32_t snapped = i2s_slave_snap_rate(hz);

    if (i2s_slave_state == I2S_SLAVE_LOCKED) {
        if (snapped != i2s_slave_detected_rate) {
            // Rate changed or the window was disturbed (glitch burst):
            // drop the lock and re-acquire.
            i2s_slave_drop_lock();
            return;
        }
        meas_hz_smooth += 0.25f * (hz - meas_hz_smooth);

        // Long dual-anchor window: rotate the anchor every 8 s, measure
        // from the older anchor (8-16 s span).
        if (now - long_us[1] >= I2S_SLAVE_LONG_HALF_US) {
            long_us[0] = long_us[1];       long_words[0] = long_words[1];
            long_us[1] = now;              long_words[1] = meas_total_words;
        }
        uint64_t long_span = now - long_us[0];
        if (long_span >= I2S_SLAVE_LONG_HALF_US) {
            meas_hz_long = (float)(meas_total_words - long_words[0]) *
                           (1e6f / 2.0f) / (float)long_span;
            long_valid = true;
        }
    } else {
        // ACQUIRING / RELOCKING: need consecutive agreeing windows
        if (snapped && snapped == lock_candidate) {
            if (++lock_agree >= I2S_SLAVE_LOCK_WINDOWS) {
                // Re-anchor every pair's read pointer to the freshest frame
                // boundary before audio processing starts: the poll is
                // lock-gated, so the rings free-ran (and lapped) during
                // acquisition and the stale backlog would front-load the
                // prefill with lap-garbled audio.  One shared anchor keeps
                // the pairs on the same frame index (even index = LEFT);
                // a pair a word behind the anchor self-corrects via the
                // modulo avail math within a word-time.
                uint32_t anchor = pair_write_word(&i2s_pairs[0]) & ~1u;
                for (uint8_t p = 0; p < i2s_n_pairs; p++)
                    i2s_pairs[p].rd_word = anchor;
                i2s_slave_state = I2S_SLAVE_LOCKED;
                i2s_slave_detected_rate = snapped;
                if (i2s_slave_lock_count < 255) i2s_slave_lock_count++;
                meas_hz_smooth = hz;
                long_us[0] = long_us[1] = now;
                long_words[0] = long_words[1] = meas_total_words;
                long_valid = false;
                input_servo_reset();   // fresh integrator for the new lock
                notify_push_i2s_slave_state(I2S_SLAVE_LOCKED, snapped);
            }
        } else {
            lock_candidate = snapped;
            lock_agree = snapped ? 1 : 0;
        }
    }
}

bool i2s_slave_check_rate_change(void) {
    if (i2s_slave_state != I2S_SLAVE_LOCKED) return false;
    uint32_t rate = i2s_slave_detected_rate;
    if (rate == 0) return false;
    if (rate != audio_state.freq) {
        // Same deferred mechanism as USB/SPDIF rate changes
        pending_rate = rate;
        __dmb();
        rate_change_pending = true;
        return true;
    }
    return false;
}

DSP_TIME_CRITICAL
void i2s_slave_update_clock_servo(void) {
    if (i2s_slave_state != I2S_SLAVE_LOCKED) return;

    // Hold off until the deferred rate change has landed: the servo's ppm
    // feed-forward is computed against the current pipeline rate's nominal
    // divider (see input_servo.c).
    if (i2s_slave_detected_rate == 0 ||
        i2s_slave_detected_rate != audio_state.freq) return;

    if (++i2s_slave_servo_skip < I2S_SLAVE_SERVO_INTERVAL) return;
    i2s_slave_servo_skip = 0;

    float actual = long_valid ? meas_hz_long : meas_hz_smooth;

    // Fill reference: first SPDIF-type slot (see section comment). All
    // SPDIF slots consume in lockstep, so one slot's fill represents them
    // all.  With no SPDIF slot (-1) the long-window rate term alone holds
    // the sys_clk-domain outputs (~0.1 ppm).  Edge-locked I2S output slots
    // follow the external clock directly and need no servo at all.
    extern uint8_t output_types[];
    extern struct audio_spdif_instance *spdif_instance_ptrs[];
    int fill_slot = -1;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_SPDIF && spdif_instance_ptrs[i]) {
            fill_slot = i;
            break;
        }
    }

    input_servo_apply(actual, fill_slot);
}

I2sSlaveState i2s_slave_get_state(void) {
    return i2s_slave_state;
}

uint32_t i2s_slave_get_detected_rate(void) {
    return i2s_slave_detected_rate;
}

void i2s_slave_get_status(I2sSlaveStatusPacket *out) {
    memset(out, 0, sizeof(*out));
    out->state = (uint8_t)i2s_slave_state;
    out->clock_mode = i2s_clock_mode;
    out->lock_count = i2s_slave_lock_count;
    out->loss_count = i2s_slave_loss_count;
    out->detected_rate = i2s_slave_detected_rate;
    out->measured_hz = meas_measured_hz;
    out->slip_count = i2s_slave_slip_count;
}

// ============================================================================
// STATUS
// ============================================================================

I2sInputState i2s_input_get_state(void) {
    return i2s_state;
}

bool i2s_input_is_clock_master(void) {
    return (i2s_state == I2S_INPUT_RUNNING) && i2s_role_master;
}

uint8_t i2s_input_active_bck_pin(void) {
    // The pin the running session was actually configured with; falls back
    // to the effective pin when the input is stopped (next start will use it).
    return (i2s_state == I2S_INPUT_RUNNING) ? i2s_active_bck_pin
                                            : i2s_effective_bck_pin();
}
