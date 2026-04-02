/*
 * Copyright (c) 2026 WeebLabs
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Multi-instance I2S audio output library for RP2040/RP2350.
 *
 * Architecture mirrors pico_audio_spdif_multi: instance-based API, shared
 * DMA IRQ handler with reference-counted enable/disable, per-PIO-block
 * program caching, and synchronized multi-instance start.
 *
 * Key differences from SPDIF:
 *   - PIO program: 8-instruction I2S master (vs 4-instruction SPDIF NRZI)
 *   - Consumer stride: 8 bytes/sample (2 × int32) vs 16 bytes (2 × subframe)
 *   - DMA words per sample: 2 (vs 4 for SPDIF)
 *   - Encoding: simple left-shift by 8 (vs NRZI lookup table)
 *   - No preambles, channel status, or block structure
 *   - BCK/LRCLK shared across instances (side-set), data pin independent
 *   - Includes MCK generator on a separate PIO SM
 */

// RP2350: Force time-critical functions into RAM to avoid XIP cache misses
#if PICO_RP2350
#define I2S_TIME_CRITICAL __attribute__((noinline, section(".time_critical")))
#else
#define I2S_TIME_CRITICAL
#endif

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "pico/audio_i2s_multi.h"
#include "audio_i2s_24.pio.h"
#include "audio_mck.pio.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

#ifndef container_of
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

// ---------------------------------------------------------------------------
// Global shared state
// ---------------------------------------------------------------------------

// PIO program offset per PIO block — loaded once per block
static int i2s_pio_program_offset[3] = {-1, -1, -1};

// Instance registry for DMA IRQ handler
static audio_i2s_instance_t *i2s_instances[PICO_AUDIO_I2S_MAX_INSTANCES];
static uint i2s_instance_count = 0;

// Track whether shared IRQ handler is installed per DMA IRQ line
static bool i2s_irq_handler_installed[2] = {false, false};

// Reference count for DMA IRQ enable/disable
static uint8_t i2s_irq_enable_count[2] = {0, 0};

// ---------------------------------------------------------------------------
// MCK generator state
// ---------------------------------------------------------------------------

static PIO mck_pio = NULL;
static uint mck_sm = 0;
static uint8_t mck_pin_current = 0;
static bool mck_running = false;
static int mck_program_offset = -1;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

static void __isr __time_critical_func(audio_i2s_dma_irq_handler)(void);
static void __time_critical_func(i2s_audio_start_dma_transfer)(audio_i2s_instance_t *inst);

// ---------------------------------------------------------------------------
// PIO block index helper
// ---------------------------------------------------------------------------

static PIO i2s_pio_block_from_index(uint8_t idx) {
    switch (idx) {
        case 0: return pio0;
        case 1: return pio1;
#if NUM_PIOS > 2
        case 2: return pio2;
#endif
        default: panic("Invalid PIO index %d", idx);
    }
}

// ---------------------------------------------------------------------------
// Clock divider calculation
// ---------------------------------------------------------------------------

// I2S timing:
//   Each audio bit takes 2 PIO clocks (out + jmp instruction pair).
//   32 bits/channel × 2 channels = 64 bits/frame × 2 clocks/bit = 128 PIO
//   clocks per stereo sample.
//
//   PIO_clk = sample_freq × 128
//
//   divider (24.8 fixed-point) = sys_clk × 256 / PIO_clk
//                               = sys_clk × 256 / (sample_freq × 128)
//                               = sys_clk × 2 / sample_freq
//
//   At 307.2 MHz / 48 kHz: divider = 12800 → integer 50, fractional 0
//   (zero PIO clock jitter)

static void i2s_update_pio_frequency(audio_i2s_instance_t *inst, uint32_t sample_freq) {
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);

    // divider in 24.8 fixed-point = sys_clk * 2 / sample_freq
    // Ceiling division to match SPDIF library rounding.  Truncation makes I2S
    // ~3 Hz fast at 44.1kHz/307.2MHz; if the feedback endpoint tracks this I2S
    // slot, all SPDIF outputs accumulate the surplus and fill over minutes.
    uint64_t num = (uint64_t)system_clock_frequency * 2;
    uint32_t divider = (uint32_t)((num + sample_freq - 1) / sample_freq);

    printf("I2S[SM%d] clock divider 0x%x/256 (%u.%u) for %d Hz\n",
           inst->pio_sm, (uint)divider,
           (uint)(divider >> 8), (uint)(divider & 0xff),
           (int)sample_freq);
    assert(divider < 0x1000000);

    pio_sm_set_clkdiv_int_frac(inst->pio, inst->pio_sm,
                                divider >> 8u, divider & 0xffu);
    inst->freq = sample_freq;
}

// ---------------------------------------------------------------------------
// Connection callbacks
// ---------------------------------------------------------------------------

I2S_TIME_CRITICAL
static audio_buffer_t *i2s_wrap_consumer_take(audio_connection_t *connection, bool block) {
    // Recover the instance from the embedded connection
    audio_i2s_instance_t *inst = container_of(
        (struct producer_pool_blocking_give_connection *)connection,
        audio_i2s_instance_t, connection);

    // Support dynamic frequency shifting
    if (connection->producer_pool->format->sample_freq != inst->freq) {
        i2s_update_pio_frequency(inst, connection->producer_pool->format->sample_freq);
    }
    return consumer_pool_take_buffer_default(connection, block);
}

// ---------------------------------------------------------------------------
// I2S sample encoding: left-justify 24-bit audio into 32-bit I2S frames
//
// Producer buffer: interleaved int32_t L/R pairs, 24-bit audio in bits [23:0]
// Consumer buffer: interleaved int32_t L/R pairs, 24-bit audio in bits [31:8]
//
// The PIO shifts MSB-first, so audio must be in the upper 24 bits.
// The lower 8 bits are zero (standard I2S padding).
//
// This is dramatically simpler than SPDIF BMC encoding — just a left-shift.
// ---------------------------------------------------------------------------

// I2S producer-give callback.
//
// Follows the same pattern as SPDIF's stereo_to_spdif_producer_give_s32():
//   1. Take a free consumer buffer (blocking — waits for DMA to return one)
//   2. Convert producer samples into consumer format
//   3. Return producer buffer to free list via queue_free_audio_buffer()
//      (NOT give_audio_buffer, which would re-trigger this callback → recursion)
//   4. Give consumer buffer to prepared list for DMA pickup
//
I2S_TIME_CRITICAL
static void i2s_wrap_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    // Mirrors producer_pool_blocking_give(): consume producer samples into
    // free consumer buffers, then queue completed consumer buffers to prepared.
    struct producer_pool_blocking_give_connection *pbc =
        (struct producer_pool_blocking_give_connection *)connection;
    uint32_t pos = 0;

    while (pos < buffer->sample_count) {
        if (!pbc->current_consumer_buffer) {
            pbc->current_consumer_buffer = get_free_audio_buffer(connection->consumer_pool, true);
            pbc->current_consumer_buffer_pos = 0;
        }

        uint32_t in_remaining = buffer->sample_count - pos;
        uint32_t out_remaining = pbc->current_consumer_buffer->max_sample_count -
                                 pbc->current_consumer_buffer_pos;
        uint32_t sample_count = in_remaining < out_remaining ? in_remaining : out_remaining;

        int32_t *src = ((int32_t *)buffer->buffer->bytes) + (pos * 2);
        int32_t *dst = ((int32_t *)pbc->current_consumer_buffer->buffer->bytes) +
                       (pbc->current_consumer_buffer_pos * 2);

        // Left-shift each channel sample by 8 to place 24-bit audio at MSB.
        for (uint32_t i = 0; i < sample_count * 2; i++) {
            dst[i] = src[i] << 8;
        }

        pos += sample_count;
        pbc->current_consumer_buffer_pos += sample_count;

        if (pbc->current_consumer_buffer_pos ==
            pbc->current_consumer_buffer->max_sample_count) {
            pbc->current_consumer_buffer->sample_count =
                pbc->current_consumer_buffer->max_sample_count;
            queue_full_audio_buffer(connection->consumer_pool,
                                    pbc->current_consumer_buffer);
            pbc->current_consumer_buffer = NULL;
        }
    }

    // Return producer buffer directly to free list (avoids recursive callback).
    queue_free_audio_buffer(connection->producer_pool, buffer);
}

// ---------------------------------------------------------------------------
// audio_i2s_setup
// ---------------------------------------------------------------------------

const audio_format_t *audio_i2s_setup(audio_i2s_instance_t *inst,
                                       const audio_format_t *intended_audio_format,
                                       const audio_i2s_config_t *config) {
    assert(i2s_instance_count < PICO_AUDIO_I2S_MAX_INSTANCES);

    // Store hardware config into instance
    inst->pio = i2s_pio_block_from_index(config->pio);
    inst->pio_sm = config->pio_sm;
    inst->dma_channel = config->dma_channel;
    inst->dma_irq = config->dma_irq;
    inst->data_pin = config->data_pin;
    inst->clock_pin_base = config->clock_pin_base;
    inst->playing_buffer = NULL;
    inst->freq = 0;
    inst->enabled = false;
    inst->words_consumed = 0;
    inst->current_transfer_words = 0;

    // Assert all instances share the same DMA IRQ line
    if (i2s_instance_count > 0) {
        assert(inst->dma_irq == i2s_instances[0]->dma_irq);
    }

    // GPIO init for data pin
    pio_gpio_init(inst->pio, config->data_pin);

    // GPIO init for clock pins (BCK + LRCLK)
    pio_gpio_init(inst->pio, config->clock_pin_base);
    pio_gpio_init(inst->pio, config->clock_pin_base + 1);

    // Claim SM
    pio_sm_claim(inst->pio, inst->pio_sm);

    // Load I2S PIO program once per PIO block
    if (i2s_pio_program_offset[config->pio] < 0) {
        i2s_pio_program_offset[config->pio] = pio_add_program(inst->pio, &audio_i2s_24_program);
    }
    uint offset = (uint)i2s_pio_program_offset[config->pio];

    // Initialize the PIO state machine
    audio_i2s_24_program_init(inst->pio, inst->pio_sm, offset,
                               config->data_pin, config->clock_pin_base);

    // Initialize per-instance silence buffer (DMA-sized, all zeros)
    inst->consumer_buffer_format.format = &inst->consumer_format;
    inst->silence_buffer.sample_count = PICO_AUDIO_I2S_DMA_SAMPLE_COUNT;
    inst->silence_buffer.max_sample_count = PICO_AUDIO_I2S_DMA_SAMPLE_COUNT;
    inst->silence_buffer.format = &inst->consumer_buffer_format;

    // I2S silence: 48 samples × 2 channels × 4 bytes = 384 bytes
    inst->silence_buffer.buffer = pico_buffer_alloc(
        PICO_AUDIO_I2S_DMA_SAMPLE_COUNT * 2 * sizeof(int32_t));
    memset(inst->silence_buffer.buffer->bytes, 0,
           PICO_AUDIO_I2S_DMA_SAMPLE_COUNT * 2 * sizeof(int32_t));

    __mem_fence_release();

    // DMA setup
    dma_channel_claim(inst->dma_channel);

    dma_channel_config dma_config = dma_channel_get_default_config(inst->dma_channel);
    channel_config_set_dreq(&dma_config, pio_get_dreq(inst->pio, inst->pio_sm, true));

#if PICO_RP2350
    // RP2350 requires explicit high priority for DMA to prevent audio underruns
    channel_config_set_high_priority(&dma_config, true);
#endif

    dma_channel_configure(inst->dma_channel,
                          &dma_config,
                          &inst->pio->txf[inst->pio_sm],  // dest: PIO TX FIFO
                          NULL,   // src: set per transfer
                          0,      // count: set per transfer
                          false   // don't trigger yet
    );

    // Install shared IRQ handler once per DMA IRQ line
    if (!i2s_irq_handler_installed[inst->dma_irq]) {
        irq_add_shared_handler(DMA_IRQ_0 + inst->dma_irq, audio_i2s_dma_irq_handler,
                               PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        i2s_irq_handler_installed[inst->dma_irq] = true;
    }

    dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, 1);

    // Register instance
    i2s_instances[i2s_instance_count++] = inst;

    return intended_audio_format;
}

// ---------------------------------------------------------------------------
// audio_i2s_connect_extra
// ---------------------------------------------------------------------------

bool audio_i2s_connect_extra(audio_i2s_instance_t *inst,
                              audio_buffer_pool_t *producer,
                              bool buffer_on_give, uint buffer_count,
                              audio_connection_t *connection) {
    printf("Connecting PIO I2S audio (SM%d, data GPIO %d, BCK GPIO %d)\n",
           inst->pio_sm, inst->data_pin, inst->clock_pin_base);

    assert(producer->format->format == AUDIO_BUFFER_FORMAT_PCM_S32);

    // Consumer format: raw PCM for I2S (2 × int32 per stereo sample)
    inst->consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_I2S;
    inst->consumer_format.sample_freq = producer->format->sample_freq;
    inst->consumer_format.channel_count = 2;
    inst->consumer_buffer_format.format = &inst->consumer_format;
    inst->consumer_buffer_format.sample_stride = 2 * sizeof(int32_t);  // 8 bytes

    // Create consumer pool: buffer_count buffers × DMA_SAMPLE_COUNT samples
    inst->consumer_pool = audio_new_consumer_pool(&inst->consumer_buffer_format,
                                                   buffer_count,
                                                   PICO_AUDIO_I2S_DMA_SAMPLE_COUNT);

    // Zero-fill all consumer buffers (I2S silence is just zeros)
    for (audio_buffer_t *buffer = inst->consumer_pool->free_list; buffer; buffer = buffer->next) {
        memset(buffer->buffer->bytes, 0,
               PICO_AUDIO_I2S_DMA_SAMPLE_COUNT * 2 * sizeof(int32_t));
    }

    i2s_update_pio_frequency(inst, producer->format->sample_freq);

    __mem_fence_release();

    if (!connection) {
        printf("I2S stereo 24-bit at %d Hz\n", (int)producer->format->sample_freq);

        // Initialize the embedded connection callbacks
        inst->connection.core.consumer_pool_take = i2s_wrap_consumer_take;
        inst->connection.core.consumer_pool_give = consumer_pool_give_buffer_default;
        inst->connection.core.producer_pool_take = producer_pool_take_buffer_default;
        inst->connection.core.producer_pool_give = i2s_wrap_producer_give;
        connection = &inst->connection.core;
    }
    audio_complete_connection(connection, producer, inst->consumer_pool);
    return true;
}

// ---------------------------------------------------------------------------
// DMA transfer
// ---------------------------------------------------------------------------

static void __time_critical_func(i2s_audio_start_dma_transfer)(audio_i2s_instance_t *inst) {
    assert(!inst->playing_buffer);
    audio_buffer_t *ab = take_audio_buffer(inst->consumer_pool, false);

    inst->playing_buffer = ab;
    if (!ab) {
        // Play silence on underrun
        ab = &inst->silence_buffer;

        extern int overruns;
        extern volatile bool preset_loading;
        if (!preset_loading)
            overruns++;
    }

    // I2S: 2 DMA words per stereo sample (1 int32 L + 1 int32 R)
    uint32_t transfer_words = ab->sample_count * 2;
    inst->current_transfer_words = transfer_words;
    dma_channel_transfer_from_buffer_now(inst->dma_channel, ab->buffer->bytes, transfer_words);
}

// ---------------------------------------------------------------------------
// DMA IRQ handler — iterates all registered I2S instances
// ---------------------------------------------------------------------------

void __isr __time_critical_func(audio_i2s_dma_irq_handler)(void) {
    for (uint i = 0; i < i2s_instance_count; i++) {
        audio_i2s_instance_t *inst = i2s_instances[i];
        if (dma_irqn_get_channel_status(inst->dma_irq, inst->dma_channel)) {
            dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);

            // Track total DMA words consumed (for USB feedback endpoint)
            inst->words_consumed += inst->current_transfer_words;

            // Free the buffer we just finished playing
            if (inst->playing_buffer) {
                extern volatile uint32_t pio_samples_dma;
                pio_samples_dma++;

                give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
                inst->playing_buffer = NULL;
            }
            i2s_audio_start_dma_transfer(inst);
        }
    }
}

// ---------------------------------------------------------------------------
// audio_i2s_set_enabled
// ---------------------------------------------------------------------------

void audio_i2s_set_enabled(audio_i2s_instance_t *inst, bool enabled) {
    if (enabled != inst->enabled) {
#ifndef NDEBUG
        if (enabled) {
            puts("Enabling PIO I2S audio\n");
            printf("(on core %d)\n", get_core_num());
        }
#endif
        if (enabled) {
            if (i2s_irq_enable_count[inst->dma_irq]++ == 0)
                irq_set_enabled(DMA_IRQ_0 + inst->dma_irq, true);
            i2s_audio_start_dma_transfer(inst);
            pio_sm_set_enabled(inst->pio, inst->pio_sm, true);
        } else {
            pio_sm_set_enabled(inst->pio, inst->pio_sm, false);
            if (--i2s_irq_enable_count[inst->dma_irq] == 0)
                irq_set_enabled(DMA_IRQ_0 + inst->dma_irq, false);
        }
        inst->enabled = enabled;
    }
}

// ---------------------------------------------------------------------------
// audio_i2s_change_data_pin
// ---------------------------------------------------------------------------

void audio_i2s_change_data_pin(audio_i2s_instance_t *inst, uint new_pin) {
    assert(!inst->enabled);

    // Mask DMA IRQ for this channel during reinit
    dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, false);

    // Abort any stale DMA transfer
    dma_channel_abort(inst->dma_channel);

    // Return in-flight buffer to consumer pool
    if (inst->playing_buffer != NULL) {
        give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
        inst->playing_buffer = NULL;
    }

    // Release old data pin from PIO mux -> high-Z
    gpio_set_function(inst->data_pin, GPIO_FUNC_NULL);
    gpio_set_dir(inst->data_pin, GPIO_IN);

    // Claim new data pin for PIO
    pio_gpio_init(inst->pio, new_pin);

    // Reinitialize SM with new data pin (clock pins unchanged)
    uint pio_idx = pio_get_index(inst->pio);
    assert(i2s_pio_program_offset[pio_idx] >= 0);
    uint offset = (uint)i2s_pio_program_offset[pio_idx];
    audio_i2s_24_program_init(inst->pio, inst->pio_sm, offset,
                               new_pin, inst->clock_pin_base);

    // Restore clock divider (pio_sm_init resets it to default)
    if (inst->freq != 0) {
        i2s_update_pio_frequency(inst, inst->freq);
    }

    // Clear any stale DMA completion flag, then unmask
    dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
    dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, true);

    inst->data_pin = new_pin;
}

// ---------------------------------------------------------------------------
// audio_i2s_enable_sync — synchronized start for multiple instances
// ---------------------------------------------------------------------------

void audio_i2s_enable_sync(audio_i2s_instance_t *instances[], uint count) {
    assert(count > 0 && count <= PICO_AUDIO_I2S_MAX_INSTANCES);

    // Enable DMA IRQ and prime DMA for all instances
    for (uint i = 0; i < count; i++) {
        audio_i2s_instance_t *inst = instances[i];
        if (i2s_irq_enable_count[inst->dma_irq]++ == 0)
            irq_set_enabled(DMA_IRQ_0 + inst->dma_irq, true);
        i2s_audio_start_dma_transfer(inst);
    }

    // Build per-PIO-block SM bitmasks
    uint32_t pio_sm_mask[3] = {0, 0, 0};
    for (uint i = 0; i < count; i++) {
        audio_i2s_instance_t *inst = instances[i];
        for (uint p = 0; p < 3; p++) {
            PIO block = i2s_pio_block_from_index(p);
            if (inst->pio == block) {
                pio_sm_mask[p] |= (1u << inst->pio_sm);
                break;
            }
        }
    }

    // Disable interrupts briefly and start all PIO blocks synchronously
    uint32_t save = save_and_disable_interrupts();
    for (uint p = 0; p < 3; p++) {
        if (pio_sm_mask[p]) {
            pio_enable_sm_mask_in_sync(i2s_pio_block_from_index(p), pio_sm_mask[p]);
        }
    }
    restore_interrupts(save);

    // Mark all instances enabled
    for (uint i = 0; i < count; i++) {
        instances[i]->enabled = true;
    }
}

// ---------------------------------------------------------------------------
// audio_i2s_teardown — full cleanup for output type switching
// ---------------------------------------------------------------------------

void audio_i2s_teardown(audio_i2s_instance_t *inst) {
    // Disable if still running
    if (inst->enabled) {
        audio_i2s_set_enabled(inst, false);
    }

    // Mask DMA IRQ and abort
    dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, false);
    dma_channel_abort(inst->dma_channel);

    // Return in-flight buffer
    if (inst->playing_buffer != NULL) {
        give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
        inst->playing_buffer = NULL;
    }

    // Release data pin
    gpio_set_function(inst->data_pin, GPIO_FUNC_NULL);
    gpio_set_dir(inst->data_pin, GPIO_IN);

    // Note: BCK/LRCLK pins are NOT released here — they may still be in use
    // by other I2S instances on the same PIO block.

    // Unclaim DMA channel and PIO SM
    dma_channel_unclaim(inst->dma_channel);
    pio_sm_unclaim(inst->pio, inst->pio_sm);

    // Remove from instance registry
    for (uint i = 0; i < i2s_instance_count; i++) {
        if (i2s_instances[i] == inst) {
            // Shift remaining entries down
            for (uint j = i; j < i2s_instance_count - 1; j++) {
                i2s_instances[j] = i2s_instances[j + 1];
            }
            i2s_instance_count--;
            break;
        }
    }

    printf("I2S teardown: SM%d, data GPIO %d (instances remaining: %d)\n",
           inst->pio_sm, inst->data_pin, i2s_instance_count);
}

// ---------------------------------------------------------------------------
// MCK generator functions
// ---------------------------------------------------------------------------

void audio_i2s_mck_setup(PIO pio, uint sm, uint pin) {
    mck_pio = pio;
    mck_sm = sm;
    mck_pin_current = pin;

    pio_gpio_init(pio, pin);
    pio_sm_claim(pio, sm);

    if (mck_program_offset < 0) {
        mck_program_offset = pio_add_program(pio, &audio_mck_program);
    }
    audio_mck_program_init(pio, sm, (uint)mck_program_offset, pin);

    printf("MCK setup: PIO%d SM%d, GPIO %d\n", pio_get_index(pio), sm, pin);
}

void audio_i2s_mck_set_enabled(bool enabled) {
    if (mck_pio == NULL) return;
    pio_sm_set_enabled(mck_pio, mck_sm, enabled);
    mck_running = enabled;
    printf("MCK %s\n", enabled ? "enabled" : "disabled");
}

// MCK frequency calculation:
//   MCK = sample_freq × multiplier
//   PIO clock = MCK × 2 (toggle program: 2 instructions per cycle)
//   divider (24.8) = sys_clk × 256 / (MCK × 2)
//                   = sys_clk × 256 / (sample_freq × multiplier × 2)
//                   = sys_clk × 128 / (sample_freq × multiplier)
//
//   At 307.2 MHz, 48 kHz, 128×: divider = 307200000 × 128 / (48000 × 128)
//                                        = 307200000 / 48000 = 6400 → 25.0
//
//   At 307.2 MHz, 48 kHz, 256×: divider = 307200000 × 128 / (48000 × 256)
//                                        = 307200000 / 96000 = 3200 → 12.5

void audio_i2s_mck_update_frequency(uint32_t sample_freq, uint32_t multiplier) {
    if (mck_pio == NULL) return;

    uint32_t sys_clk = clock_get_hz(clk_sys);

    // divider_24.8 = sys_clk × 128 / (sample_freq × multiplier)
    // Use 64-bit intermediate to avoid overflow
    uint64_t num = (uint64_t)sys_clk * 128;
    uint64_t den = (uint64_t)sample_freq * multiplier;
    uint32_t divider = (uint32_t)(num / den);

    printf("MCK divider 0x%x/256 (%u.%u) for %d Hz × %u = %u Hz\n",
           (uint)divider, (uint)(divider >> 8), (uint)(divider & 0xff),
           (int)sample_freq, (uint)multiplier,
           (uint)(sample_freq * multiplier));

    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(mck_pio, mck_sm, divider >> 8u, divider & 0xffu);
}

void audio_i2s_mck_change_pin(uint new_pin) {
    if (mck_pio == NULL) return;
    assert(!mck_running);  // MCK must be disabled before changing pin

    // Release old pin
    gpio_set_function(mck_pin_current, GPIO_FUNC_NULL);
    gpio_set_dir(mck_pin_current, GPIO_IN);

    // Claim new pin and reinit
    pio_gpio_init(mck_pio, new_pin);
    assert(mck_program_offset >= 0);
    audio_mck_program_init(mck_pio, mck_sm, (uint)mck_program_offset, new_pin);

    mck_pin_current = new_pin;
    printf("MCK pin changed to GPIO %d\n", new_pin);
}

uint8_t audio_i2s_mck_get_pin(void) {
    return mck_pin_current;
}

bool audio_i2s_mck_is_enabled(void) {
    return mck_running;
}
