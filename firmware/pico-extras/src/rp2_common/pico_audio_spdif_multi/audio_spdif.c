/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Per-block producer path; must stay in RAM under XIP builds on both
// platforms (reached via connection function pointers).
#define SPDIF_TIME_CRITICAL __attribute__((noinline, section(".time_critical")))

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "pico/audio_spdif.h"
#include <pico/audio_spdif/sample_encoding.h>
#include "audio_spdif.pio.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

// ---------------------------------------------------------------------------
// Ring output model
//
// One statically allocated encoded ring per slot, drained by free-running
// DMA into the PIO TX FIFO: ENDLESS transfer count on RP2350, a data +
// reload channel pair on RP2040 (the reload rewrites the data channel's
// transfer count each ring pass; read address wraps in hardware on both).
// Zero DMA IRQs; fill, consumed-words and underrun accounting are pointer
// arithmetic. The writer encodes producer blocks directly into the ring,
// stamping IEC preamble/channel-status from the absolute frame index
// (phase = frames mod 192, which cannot live in ring position because 192
// does not divide a power of two), and keeps ERASE_AHEAD frames of valid
// silence stamped ahead of real audio: a stalled producer plays clean
// silence, never a loop of stale ring content.
//
// Underrun: when the read pointer overtakes the writer, the DSPi give path
// applies an identical audio_spdif_ring_skip() to every enabled slot (both
// output types), preserving inter-slot alignment deterministically.
// ---------------------------------------------------------------------------

#ifndef container_of
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

_Static_assert(PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT % PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT == 0,
    "DMA sample count must divide block sample count (192) evenly");
_Static_assert((PICO_AUDIO_SPDIF_RING_FRAMES & (PICO_AUDIO_SPDIF_RING_FRAMES - 1)) == 0,
    "ring frame count must be a power of two");

#define RING_FRAMES      PICO_AUDIO_SPDIF_RING_FRAMES
#define RING_MASK        (RING_FRAMES - 1u)
#define RING_WORDS       (RING_FRAMES * 4u)
#define RING_BYTES       PICO_AUDIO_SPDIF_RING_BYTES
// Writable window: capacity minus the silence margin the writer maintains.
#define WRITABLE_FRAMES  (RING_FRAMES - PICO_AUDIO_SPDIF_ERASE_AHEAD)

#if !PICO_RP2350
// Reload source word for the RP2040 free-running pair. Deliberately
// non-const: the DMA reads it from RAM during flash blackout windows
// where an XIP fetch would stall the ring.
static uint32_t spdif_ring_words_const = RING_WORDS;
#endif

// ---------------------------------------------------------------------------
// Global shared state
// ---------------------------------------------------------------------------

// NRZI lookup table -- shared, built once
uint32_t spdif_lookup[256];
static bool spdif_lookup_initialized = false;

// PIO program offset per PIO block -- loaded once per block
static int pio_program_offset[3] = {-1, -1, -1};

// Instance registry (starvation getters and group operations)
static audio_spdif_instance_t *spdif_instances[PICO_AUDIO_SPDIF_MAX_INSTANCES];
static uint spdif_instance_count = 0;

// Reference count for DMA IRQ enable/disable. The ring TX path uses no DMA
// IRQs; this survives solely for external holders (SPDIF RX keeps line 1
// alive across pipeline resets via audio_spdif_irq_refcount_adjust).
static uint8_t irq_enable_count[2] = {0, 0};

// ---------------------------------------------------------------------------
// S/PDIF constants
// ---------------------------------------------------------------------------

#define PREAMBLE_X 0b11001001
#define PREAMBLE_Y 0b01101001
#define PREAMBLE_Z 0b00111001

// IEC 60958-3 consumer channel status (5 bytes = 40 bits)
// Byte values verified against Linux kernel include/sound/asoundef.h
static uint8_t spdif_channel_status[5] = {
    0x04,  // Byte 0: consumer, PCM, copy permitted
    0x00,  // Byte 1: general category
    0x00,  // Byte 2: source/channel
    0x00,  // Byte 3: sample rate (set dynamically via update_pio_frequency)
    0x0B,  // Byte 4: max=24bit, word length=24bit (0x01 | 0x0A)
};

static inline uint get_channel_status_bit(uint subframe_idx) {
    if (subframe_idx >= 40) return 0;
    return (spdif_channel_status[subframe_idx / 8] >> (subframe_idx % 8)) & 1u;
}

// Encode one stereo frame at ring position `p` with IEC phase `phase`.
static inline void stamp_frame(spdif_subframe_t *p, uint phase,
                               int32_t l, int32_t r) {
    uint c = get_channel_status_bit(phase);
    p[0].l = (phase == 0) ? PREAMBLE_Z : PREAMBLE_X;
    p[0].h = 0x55000000u | (c << 29u);
    spdif_update_subframe(&p[0], l);
    p[1].l = PREAMBLE_Y;
    p[1].h = 0x55000000u | (c << 29u);
    spdif_update_subframe(&p[1], r);
}

// ---------------------------------------------------------------------------
// Consumed-side accounting
// ---------------------------------------------------------------------------

SPDIF_TIME_CRITICAL
uint32_t audio_spdif_ring_consumed_words(audio_spdif_instance_t *inst) {
    // Forward-unwrap the DMA read pointer. Correct as long as calls are no
    // further apart than one ring period (~21 ms); the main loop's fill
    // checks guarantee that while producing. A longer gap under-counts by
    // whole ring multiples, which is benign: playback position is masked,
    // the re-anchor deficit errs small, and every reset re-zeroes it.
    uint32_t save = save_and_disable_interrupts();
    uint32_t addr = dma_channel_hw_addr(inst->dma_channel)->read_addr;
    uint32_t delta = (addr - inst->last_read_addr) & (RING_BYTES - 1u);
    inst->last_read_addr = addr;
    inst->consumed_words += delta / 4u;
    uint32_t out = inst->consumed_words;
    restore_interrupts(save);
    return out;
}

SPDIF_TIME_CRITICAL
uint32_t audio_spdif_ring_fill_frames(audio_spdif_instance_t *inst) {
    return inst->wr_frames - audio_spdif_ring_consumed_words(inst) / 4u;
}

void audio_spdif_ring_reset(audio_spdif_instance_t *inst) {
    // DMA must be stopped. Re-anchors all accounting to ring position 0.
    inst->wr_frames = 0;
    inst->erased_frames = 0;
    inst->consumed_words = 0;
    inst->last_read_addr = (uint32_t)(uintptr_t)inst->ring;
    inst->snap_produced_frames = 0;
    inst->snap_consumed_frames = 0;
    // Re-anchor the HARDWARE read pointer too: the abort left it frozen
    // mid-ring, and any consumed/fill read before the next arm would
    // otherwise count a bogus base-to-stale delta into consumed_words,
    // skewing this slot's accounting (and its re-arm offset) against the
    // others. DMA is stopped here per the contract, so the write is safe.
    dma_channel_set_read_addr(inst->dma_channel, inst->ring, false);
}

SPDIF_TIME_CRITICAL
void audio_spdif_ring_skip(audio_spdif_instance_t *inst, uint32_t frames) {
    inst->underrun_frames += frames;
    inst->wr_frames += frames;
    // Restamp silence through the new margin: the reader immediately
    // traverses the skipped span, which beyond the old erase-ahead may
    // still hold stale audio from the previous lap.
    spdif_subframe_t *ring = (spdif_subframe_t *)inst->ring;
    uint32_t target = inst->wr_frames + PICO_AUDIO_SPDIF_ERASE_AHEAD;
    uint32_t e = inst->erased_frames;
    if ((int32_t)(target - e) > (int32_t)RING_FRAMES) e = target - RING_FRAMES;
    while ((int32_t)(target - e) > 0) {
        stamp_frame(&ring[(e & RING_MASK) * 2u], e % 192u, 0, 0);
        e++;
    }
    inst->erased_frames = e;
}

// Extend the silence region toward one full ring ahead of the reader so a
// stalled producer can never loop stale content. Never touches unplayed
// audio (the stamped span is exactly the free region). Bounded per call.
SPDIF_TIME_CRITICAL
void audio_spdif_ring_silence_pump(audio_spdif_instance_t *inst, uint32_t max_frames) {
    uint32_t consumed = audio_spdif_ring_consumed_words(inst) / 4u;
    uint32_t target = consumed + RING_FRAMES;
    uint32_t e = inst->erased_frames;
    if ((int32_t)(e - inst->wr_frames) < 0) e = inst->wr_frames;
    int32_t need = (int32_t)(target - e);
    if (need <= 0) return;
    uint32_t n = ((uint32_t)need > max_frames) ? max_frames : (uint32_t)need;
    spdif_subframe_t *ring = (spdif_subframe_t *)inst->ring;
    while (n--) {
        stamp_frame(&ring[(e & RING_MASK) * 2u], e % 192u, 0, 0);
        e++;
    }
    inst->erased_frames = e;
}

// ---------------------------------------------------------------------------
// Ring writer
// ---------------------------------------------------------------------------

SPDIF_TIME_CRITICAL
void audio_spdif_ring_write_s32(audio_spdif_instance_t *inst,
                                const int32_t *lr, uint32_t frames) {
    uint32_t consumed = audio_spdif_ring_consumed_words(inst) / 4u;
    uint32_t fill = inst->wr_frames - consumed;

    // fill > RING_FRAMES (unsigned) also covers the underrun case where the
    // reader has overtaken the writer: writable becomes 0, the block is
    // dropped and counted, and the DSPi give path re-anchors via skip().
    uint32_t writable = (fill < WRITABLE_FRAMES) ? (WRITABLE_FRAMES - fill) : 0;
    if (frames > writable) {
        inst->drop_frames += frames - writable;
        frames = writable;
    }

    // Phase = w mod 192. Known limit: at the uint32 wrap (~24.8 h of
    // uninterrupted streaming, and any ring reset restarts the clock) the
    // phase steps by 64 for one block; receivers resync on the next Z.
    spdif_subframe_t *ring = (spdif_subframe_t *)inst->ring;
    uint32_t w = inst->wr_frames;
    for (uint32_t i = 0; i < frames; i++) {
        stamp_frame(&ring[(w & RING_MASK) * 2u], w % 192u,
                    lr[i * 2u], lr[i * 2u + 1u]);
        w++;
    }
    inst->wr_frames = w;

    // Extend the erase-ahead silence margin.
    uint32_t e = inst->erased_frames;
    if (e < w) e = w;
    uint32_t target = w + PICO_AUDIO_SPDIF_ERASE_AHEAD;
    while (e < target) {
        stamp_frame(&ring[(e & RING_MASK) * 2u], e % 192u, 0, 0);
        e++;
    }
    inst->erased_frames = e;

    // Latch the coherent produced/consumed pair (constant production phase;
    // the clock servo's cumulative-count observable).
    inst->snap_produced_frames = w;
    inst->snap_consumed_frames = consumed;
}

// ---------------------------------------------------------------------------
// Free-running DMA arm/stop
// ---------------------------------------------------------------------------

static void ring_dma_arm(audio_spdif_instance_t *inst) {
    // Drain stale words from a previous run so the first output sample
    // after a (re)start is the ring's, not leftovers (audible click).
    pio_sm_clear_fifos(inst->pio, inst->pio_sm);

    uint32_t offset = (inst->consumed_words * 4u) & (RING_BYTES - 1u);
    uint8_t *start = (uint8_t *)inst->ring + offset;
    // Keep the unwrap anchor exactly in step with the address programmed
    // below so the first consumed read after (re)arm sees a zero delta.
    inst->last_read_addr = (uint32_t)(uintptr_t)start;

    dma_channel_config c = dma_channel_get_default_config(inst->dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, __builtin_ctz(RING_BYTES));
    channel_config_set_dreq(&c, pio_get_dreq(inst->pio, inst->pio_sm, true));
    channel_config_set_high_priority(&c, true);

#if PICO_RP2350
    dma_channel_set_config(inst->dma_channel, &c, false);
    dma_channel_set_write_addr(inst->dma_channel, &inst->pio->txf[inst->pio_sm], false);
    dma_channel_set_read_addr(inst->dma_channel, start, false);
    // Triggering pre-fills the PIO FIFO and stalls on DREQ until the SM starts.
    dma_channel_set_trans_count(inst->dma_channel,
                                dma_encode_endless_transfer_count(), true);
#else
    // Reload partner: one un-paced transfer per chain trigger rewrites the
    // data channel's transfer count, restarting the next ring pass.
    dma_channel_config r = dma_channel_get_default_config(inst->dma_reload_channel);
    channel_config_set_transfer_data_size(&r, DMA_SIZE_32);
    channel_config_set_read_increment(&r, false);
    channel_config_set_write_increment(&r, false);
    dma_channel_configure(inst->dma_reload_channel, &r,
                          &dma_hw->ch[inst->dma_channel].al1_transfer_count_trig,
                          &spdif_ring_words_const, 1, false);

    channel_config_set_chain_to(&c, inst->dma_reload_channel);
    dma_channel_set_config(inst->dma_channel, &c, false);
    dma_channel_set_write_addr(inst->dma_channel, &inst->pio->txf[inst->pio_sm], false);
    dma_channel_set_read_addr(inst->dma_channel, start, false);
    dma_channel_set_trans_count(inst->dma_channel, RING_WORDS, true);
#endif
}

static void ring_dma_stop(audio_spdif_instance_t *inst) {
#if !PICO_RP2350
    // Break the chain loop before aborting so the reload cannot re-arm the
    // data channel mid-abort (same hazard class as the RX ring teardown).
    hw_write_masked(&dma_hw->ch[inst->dma_channel].al1_ctrl,
                    (uint32_t)inst->dma_channel << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
                    DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
    dma_hw->abort = (1u << inst->dma_channel) | (1u << inst->dma_reload_channel);
    while ((dma_hw->ch[inst->dma_channel].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) ||
           (dma_hw->ch[inst->dma_reload_channel].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS))
        tight_loop_contents();
#else
    dma_channel_abort(inst->dma_channel);
    // Leave ENDLESS mode so a future NORMAL-mode reuse of the channel
    // cannot inherit the encoded count (mirrors adat_input_stop).
    dma_channel_hw_addr(inst->dma_channel)->transfer_count = 0;
#endif
}

// ---------------------------------------------------------------------------
// audio_spdif_setup
// ---------------------------------------------------------------------------

const audio_format_t *audio_spdif_setup(audio_spdif_instance_t *inst,
                                        const audio_format_t *intended_audio_format,
                                        const audio_spdif_config_t *config) {
    assert(spdif_instance_count < PICO_AUDIO_SPDIF_MAX_INSTANCES);

    // Build NRZI lookup table once
    if (!spdif_lookup_initialized) {
        for(uint i=0;i<256;i++) {
            uint32_t v = 0x5555;
            uint p = 0;
            for(uint j = 0; j<8; j++) {
                if (i & (1<<j)) {
                    p ^= 1;
                    v |= (2<<(j*2));
                }
            }
            spdif_lookup[i] = v | (p << 16u);
        }
        spdif_lookup_initialized = true;
    }

    // Store hardware config into instance
    inst->pio = NULL;
    switch (config->pio) {
        case 0: inst->pio = pio0; break;
        case 1: inst->pio = pio1; break;
#if NUM_PIOS > 2
        case 2: inst->pio = pio2; break;
#endif
        default: panic("Invalid PIO index %d", config->pio);
    }
    inst->pio_sm = config->pio_sm;
    inst->dma_channel = config->dma_channel;
    inst->dma_reload_channel = config->dma_reload_channel;
    inst->dma_irq = config->dma_irq;
    inst->pin = config->pin;
    inst->freq = 0;
    inst->enabled = false;
    // Stable across teardown/re-setup: the DMA channel is this slot's
    // permanent identity, so the stats index never collides across
    // output-type switches.
    inst->instance_index = config->dma_channel;
    // Caller-owned slot ring (shared with the slot's I2S instance).
    assert(config->ring_base != NULL);
    assert(((uintptr_t)config->ring_base & (RING_BYTES - 1u)) == 0);
    inst->ring = (uint32_t *)config->ring_base;
    inst->underrun_frames = 0;
    inst->drop_frames = 0;
    audio_spdif_ring_reset(inst);

    // This instance struct may be reused across output-type switches.
    memset(&inst->connection, 0, sizeof(inst->connection));

    // GPIO init for this PIO block
    pio_gpio_init(inst->pio, config->pin);

    // Claim SM
    pio_sm_claim(inst->pio, inst->pio_sm);

    // Load PIO program once per PIO block
    if (pio_program_offset[config->pio] < 0) {
        pio_program_offset[config->pio] = pio_add_program(inst->pio, &audio_spdif_program);
    }
    uint offset = (uint)pio_program_offset[config->pio];

    spdif_program_init(inst->pio, inst->pio_sm, offset, config->pin);

    __mem_fence_release();

    // Claim DMA; configuration happens at each arm.
    dma_channel_claim(inst->dma_channel);
#if !PICO_RP2350
    dma_channel_claim(inst->dma_reload_channel);
#endif

    // Register instance
    spdif_instances[spdif_instance_count++] = inst;

    return intended_audio_format;
}

// ---------------------------------------------------------------------------
// update_pio_frequency
// ---------------------------------------------------------------------------

static void update_pio_frequency(audio_spdif_instance_t *inst, uint32_t sample_freq) {
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);

    // ceil the divider; must stay bit-identical to the input servo's
    // feed-forward reference (input_servo.c) and the I2S 2x derivation.
    uint32_t divider = system_clock_frequency / sample_freq + (system_clock_frequency % sample_freq != 0);
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(inst->pio, inst->pio_sm, divider >> 8u, divider & 0xffu);
    inst->freq = sample_freq;

    // Update IEC 60958-3 channel status byte 3 (sample rate)
    switch (sample_freq) {
        case 44100: spdif_channel_status[3] = 0x00; break;  // IEC958_AES3_CON_FS_44100
        case 48000: spdif_channel_status[3] = 0x02; break;  // IEC958_AES3_CON_FS_48000
        case 96000: spdif_channel_status[3] = 0x0A; break;  // IEC958_AES3_CON_FS_96000
        default:    spdif_channel_status[3] = 0x01; break;  // not indicated
    }
}

// Public eager variant: rate-change paths restore the nominal divider
// explicitly (the ring path has no lazy consumer-take hook).
void audio_spdif_apply_pio_frequency(audio_spdif_instance_t *inst, uint32_t sample_freq) {
    update_pio_frequency(inst, sample_freq);
}

// ---------------------------------------------------------------------------
// Connection callback (until the direct ring-write migration removes the
// producer pools)
// ---------------------------------------------------------------------------

SPDIF_TIME_CRITICAL static void wrap_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    struct producer_pool_blocking_give_connection *pbc =
        (struct producer_pool_blocking_give_connection *)connection;
    audio_spdif_instance_t *inst =
        container_of(pbc, audio_spdif_instance_t, connection);

    assert(buffer->format->format->format == AUDIO_BUFFER_FORMAT_PCM_S32);

    // Lazy rate follow (replaces the old wrap_consumer_take hook).
    if (connection->producer_pool->format->sample_freq != inst->freq) {
        update_pio_frequency(inst, connection->producer_pool->format->sample_freq);
    }

    audio_spdif_ring_write_s32(inst, (const int32_t *)buffer->buffer->bytes,
                               buffer->sample_count);
    queue_free_audio_buffer(connection->producer_pool, buffer);
}

// ---------------------------------------------------------------------------
// audio_spdif_connect_*
// ---------------------------------------------------------------------------

bool audio_spdif_connect_extra(audio_spdif_instance_t *inst,
                               audio_buffer_pool_t *producer,
                               bool buffer_on_give, audio_buffer_pool_t *consumer_pool,
                               audio_connection_t *connection) {
    (void)buffer_on_give;
    (void)consumer_pool;   // ring path: the shared static pool is unused
    printf("Connecting PIO S/PDIF audio (ring)\n");

    assert(producer->format->format == AUDIO_BUFFER_FORMAT_PCM_S32);
    inst->consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_SPDIF;
    inst->consumer_format.sample_freq = producer->format->sample_freq;
    inst->consumer_format.channel_count = 2;
    inst->consumer_buffer_format.format = &inst->consumer_format;
    inst->consumer_buffer_format.sample_stride = 2 * sizeof(spdif_subframe_t);

    update_pio_frequency(inst, producer->format->sample_freq);

    __mem_fence_release();

    if (!connection) {
        inst->connection.core.consumer_pool_take = NULL;
        inst->connection.core.consumer_pool_give = NULL;
        inst->connection.core.producer_pool_take = producer_pool_take_buffer_default;
        inst->connection.core.producer_pool_give = wrap_producer_give;
        connection = &inst->connection.core;
    }
    // Manual link: there is no consumer pool to complete a connection with.
    connection->producer_pool = producer;
    connection->consumer_pool = NULL;
    producer->connection = connection;
    return true;
}

// ---------------------------------------------------------------------------
// audio_spdif_change_pin
// ---------------------------------------------------------------------------

void audio_spdif_change_pin(audio_spdif_instance_t *inst, uint new_pin) {
    assert(!inst->enabled);

    ring_dma_stop(inst);

    // Release old pin from PIO mux -> high-Z
    gpio_set_function(inst->pin, GPIO_FUNC_NULL);
    gpio_set_dir(inst->pin, GPIO_IN);

    // Claim new pin for PIO
    pio_gpio_init(inst->pio, new_pin);

    // Reinitialize SM with new pin using cached program offset
    uint pio_idx = pio_get_index(inst->pio);
    assert(pio_program_offset[pio_idx] >= 0);
    uint offset = (uint)pio_program_offset[pio_idx];
    spdif_program_init(inst->pio, inst->pio_sm, offset, new_pin);

    // Restore clock divider (pio_sm_init resets it to default)
    if (inst->freq != 0) {
        update_pio_frequency(inst, inst->freq);
    }

    inst->pin = new_pin;
}

// ---------------------------------------------------------------------------
// audio_spdif_set_enabled
// ---------------------------------------------------------------------------

void audio_spdif_set_enabled(audio_spdif_instance_t *inst, bool enabled) {
    if (enabled != inst->enabled) {
        if (enabled) {
            ring_dma_arm(inst);
            pio_sm_set_enabled(inst->pio, inst->pio_sm, true);
        } else {
            pio_sm_set_enabled(inst->pio, inst->pio_sm, false);
            ring_dma_stop(inst);
        }
        inst->enabled = enabled;
    }
}

// ---------------------------------------------------------------------------
// audio_spdif_teardown -- full resource release for output type switching
// ---------------------------------------------------------------------------

void audio_spdif_teardown(audio_spdif_instance_t *inst) {
    if (inst->enabled) {
        audio_spdif_set_enabled(inst, false);
    } else {
        ring_dma_stop(inst);
    }

    // Break the producer connection link.
    if (inst->connection.core.producer_pool &&
        inst->connection.core.producer_pool->connection == &inst->connection.core) {
        inst->connection.core.producer_pool->connection = NULL;
    }
    inst->connection.core.producer_pool_take = NULL;
    inst->connection.core.producer_pool_give = NULL;
    inst->connection.core.consumer_pool_take = NULL;
    inst->connection.core.consumer_pool_give = NULL;
    inst->connection.core.producer_pool = NULL;
    inst->connection.core.consumer_pool = NULL;

    // Release the pin to high-Z.
    gpio_set_function(inst->pin, GPIO_FUNC_NULL);
    gpio_set_dir(inst->pin, GPIO_IN);

    // Unclaim DMA channel(s) and PIO SM so this slot's I2S instance can
    // claim them.
    dma_channel_unclaim(inst->dma_channel);
#if !PICO_RP2350
    dma_channel_unclaim(inst->dma_reload_channel);
#endif
    pio_sm_unclaim(inst->pio, inst->pio_sm);

    // Remove from the instance registry.
    uint32_t save = save_and_disable_interrupts();
    for (uint i = 0; i < spdif_instance_count; i++) {
        if (spdif_instances[i] == inst) {
            for (uint j = i; j < spdif_instance_count - 1; j++) {
                spdif_instances[j] = spdif_instances[j + 1];
            }
            spdif_instance_count--;
            break;
        }
    }
    restore_interrupts(save);

    inst->enabled = false;

    printf("S/PDIF teardown: SM%d, GPIO %d (instances remaining: %u)\n",
           inst->pio_sm, inst->pin, spdif_instance_count);
}

// ---------------------------------------------------------------------------
// audio_spdif_enable_sync -- synchronized start for multiple instances
// ---------------------------------------------------------------------------

// Prepare-only half of the synchronized start: arm every instance's DMA
// (pre-fills the PIO FIFO, then stalls on DREQ) WITHOUT starting the SMs.
// Returns the SM mask; the caller performs the pio_enable_sm_mask_in_sync
// so both output types start on one cycle.
uint32_t audio_spdif_enable_sync_prepare(audio_spdif_instance_t *instances[], uint count) {
    assert(count > 0 && count <= PICO_AUDIO_SPDIF_MAX_INSTANCES);

    for (uint i = 0; i < count; i++) {
        ring_dma_arm(instances[i]);
    }

    uint32_t sm_mask = 0;
    for (uint i = 0; i < count; i++) {
        assert(instances[i]->pio == instances[0]->pio);
        sm_mask |= (1u << instances[i]->pio_sm);
    }

    for (uint i = 0; i < count; i++) {
        instances[i]->enabled = true;
    }
    return sm_mask;
}

void audio_spdif_enable_sync(audio_spdif_instance_t *instances[], uint count) {
    uint32_t sm_mask = audio_spdif_enable_sync_prepare(instances, count);

    uint32_t save = save_and_disable_interrupts();
    pio_enable_sm_mask_in_sync(instances[0]->pio, sm_mask);
    restore_interrupts(save);
}

// ---------------------------------------------------------------------------
// Starvation / diagnostics API (unit change: counts are now FRAMES of
// silence exposed to the wire, not silence-buffer DMA starts)
// ---------------------------------------------------------------------------

void audio_spdif_set_starvation_monitoring(bool enabled) {
    (void)enabled;   // ring underrun accounting is always on
}

void audio_spdif_reset_dma_starvations(void) {
    for (uint i = 0; i < spdif_instance_count; i++) {
        spdif_instances[i]->underrun_frames = 0;
        spdif_instances[i]->drop_frames = 0;
    }
}

uint32_t audio_spdif_get_dma_starvations(void) {
    uint32_t total = 0;
    for (uint i = 0; i < spdif_instance_count; i++) {
        total += spdif_instances[i]->underrun_frames;
    }
    return total;
}

uint32_t audio_spdif_get_dma_starvations_instance(uint index) {
    for (uint i = 0; i < spdif_instance_count; i++) {
        if (spdif_instances[i]->instance_index == index) {
            return spdif_instances[i]->underrun_frames;
        }
    }
    return 0;
}

void audio_spdif_irq_refcount_adjust(uint dma_irq, int delta) {
    assert(dma_irq <= 1);
    if (delta > 0) {
        if (irq_enable_count[dma_irq]++ == 0)
            irq_set_enabled(DMA_IRQ_0 + dma_irq, true);
    } else if (delta < 0 && irq_enable_count[dma_irq] > 0) {
        if (--irq_enable_count[dma_irq] == 0)
            irq_set_enabled(DMA_IRQ_0 + dma_irq, false);
    }
}
