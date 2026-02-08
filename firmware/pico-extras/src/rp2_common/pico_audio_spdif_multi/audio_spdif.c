/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// RP2350: Force time-critical functions into RAM to avoid XIP cache misses
#if PICO_RP2350
#define SPDIF_TIME_CRITICAL __attribute__((noinline, section(".time_critical")))
#else
#define SPDIF_TIME_CRITICAL
#endif

#include <stdio.h>
#include <stddef.h>
#include "pico/audio_spdif.h"
#include <pico/audio_spdif/sample_encoding.h>
#include "audio_spdif.pio.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"


CU_REGISTER_DEBUG_PINS(audio_timing)

// ---- select at most one ---
//CU_SELECT_DEBUG_PINS(audio_timing)

#ifndef container_of
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

// ---------------------------------------------------------------------------
// Global shared state
// ---------------------------------------------------------------------------

// NRZI lookup table -- shared, built once
uint32_t spdif_lookup[256];
static bool spdif_lookup_initialized = false;

// PIO program offset per PIO block -- loaded once per block
static int pio_program_offset[3] = {-1, -1, -1};

// Instance registry for DMA IRQ handler
static audio_spdif_instance_t *spdif_instances[PICO_AUDIO_SPDIF_MAX_INSTANCES];
static uint spdif_instance_count = 0;

// Track whether shared IRQ handler is installed per DMA IRQ line
static bool irq_handler_installed[2] = {false, false};

// Reference count for DMA IRQ enable/disable
static uint8_t irq_enable_count[2] = {0, 0};

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

static void __isr __time_critical_func(audio_spdif_dma_irq_handler)();
static void __time_critical_func(audio_start_dma_transfer)(audio_spdif_instance_t *inst);

// ---------------------------------------------------------------------------
// S/PDIF constants
// ---------------------------------------------------------------------------

#define SR_44100 0
#define SR_48000 1

#define PREAMBLE_X 0b11001001
#define PREAMBLE_Y 0b01101001
#define PREAMBLE_Z 0b00111001

#define SPDIF_CONTROL_WORD (\
    0x4 | /* copying allowed */ \
    0x20 | /* PCM encoder/decoder */ \
    (SR_44100 << 24) /* todo is this required */ \
    )

// ---------------------------------------------------------------------------
// Buffer initialization
// ---------------------------------------------------------------------------

// each buffer is pre-filled with data
static void init_spdif_buffer(audio_buffer_t *buffer) {
    assert(buffer->max_sample_count == PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT);
    spdif_subframe_t *p = (spdif_subframe_t *)buffer->buffer->bytes;
    for(uint i=0;i<PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT;i++) {
        uint c_bit = i < 32 ? (SPDIF_CONTROL_WORD >> i) & 1u: 0;

        p->l = (i ? PREAMBLE_X : PREAMBLE_Z) | 0b10101010101010100000000; // 0-7 preamble, 8-15 aux samp, 16 - 56 sample
        p->h = 0x55000000u | (c_bit << 29u);                              // 0-24 sample low, 25 26 valid, 27 28 user data, 29 30 status
        p++;
        p->l = PREAMBLE_Y | 0b10101010101010100000000;
        p->h = 0x55000000u | (c_bit << 29u); // 25 -> 29
        p++;
    }
}

// ---------------------------------------------------------------------------
// PIO block index helper
// ---------------------------------------------------------------------------

static PIO pio_block_from_index(uint8_t idx) {
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
    inst->pio = pio_block_from_index(config->pio);
    inst->pio_sm = config->pio_sm;
    inst->dma_channel = config->dma_channel;
    inst->dma_irq = config->dma_irq;
    inst->pin = config->pin;
    inst->playing_buffer = NULL;
    inst->freq = 0;
    inst->enabled = false;

    // Assert all instances share the same DMA IRQ line
    if (spdif_instance_count > 0) {
        assert(inst->dma_irq == spdif_instances[0]->dma_irq);
    }

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

    // Initialize per-instance silence buffer
    inst->consumer_buffer_format.format = &inst->consumer_format;
    inst->silence_buffer.sample_count = PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT;
    inst->silence_buffer.max_sample_count = PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT;
    inst->silence_buffer.format = &inst->consumer_buffer_format;

    inst->silence_buffer.buffer = pico_buffer_alloc(PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT * 2 * sizeof(spdif_subframe_t));
    init_spdif_buffer(&inst->silence_buffer);
    spdif_subframe_t *sf = (spdif_subframe_t *)inst->silence_buffer.buffer->bytes;
    for(uint i=0;i<inst->silence_buffer.sample_count;i++) {
        spdif_update_subframe(sf++, 0);
        spdif_update_subframe(sf++, 0);
    }

    __mem_fence_release();

    // DMA setup
    dma_channel_claim(inst->dma_channel);

    dma_channel_config dma_config = dma_channel_get_default_config(inst->dma_channel);
    channel_config_set_dreq(&dma_config, pio_get_dreq(inst->pio, inst->pio_sm, true));

#if PICO_RP2350
    // RP2350 requires explicit high priority for DMA to prevent audio underruns
    // when USB/CPU activity creates bus contention
    channel_config_set_high_priority(&dma_config, true);
#endif

    dma_channel_configure(inst->dma_channel,
                          &dma_config,
                          &inst->pio->txf[inst->pio_sm],  // dest
                          NULL, // src
                          0, // count
                          false // trigger
    );

    // Install shared IRQ handler once per DMA IRQ line
    if (!irq_handler_installed[inst->dma_irq]) {
        irq_add_shared_handler(DMA_IRQ_0 + inst->dma_irq, audio_spdif_dma_irq_handler,
                               PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_handler_installed[inst->dma_irq] = true;
    }

    dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, 1);

    // Register instance
    spdif_instances[spdif_instance_count++] = inst;

    return intended_audio_format;
}

// ---------------------------------------------------------------------------
// update_pio_frequency
// ---------------------------------------------------------------------------

static void update_pio_frequency(audio_spdif_instance_t *inst, uint32_t sample_freq) {
    printf("setting pio freq %d\n", (int) sample_freq);
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);

    // ceil the divider (gets us a closer hit to 44100 at 176.57142857142858 MHz)
    uint32_t divider = system_clock_frequency / sample_freq + (system_clock_frequency % sample_freq != 0);

    printf("System clock at %u, S/PDIF clock divider 0x%x/256\n", (uint) system_clock_frequency, (uint)divider);
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(inst->pio, inst->pio_sm, divider >> 8u, divider & 0xffu);
    inst->freq = sample_freq;
}

// ---------------------------------------------------------------------------
// Connection callbacks
// ---------------------------------------------------------------------------

SPDIF_TIME_CRITICAL static audio_buffer_t *wrap_consumer_take(audio_connection_t *connection, bool block) {
    // Recover the instance from the embedded connection
    audio_spdif_instance_t *inst = container_of(
        (struct producer_pool_blocking_give_connection *)connection,
        audio_spdif_instance_t, connection);

    // support dynamic frequency shifting
    if (connection->producer_pool->format->sample_freq != inst->freq) {
        update_pio_frequency(inst, connection->producer_pool->format->sample_freq);
    }
    return consumer_pool_take_buffer_default(connection, block);
}

SPDIF_TIME_CRITICAL static void wrap_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
    if (buffer->format->format->format == AUDIO_BUFFER_FORMAT_PCM_S16) {
#if PICO_AUDIO_SPDIF_MONO_INPUT
        mono_to_spdif_producer_give(connection, buffer);
#else
        stereo_to_spdif_producer_give(connection, buffer);
#endif
    } else {
        panic_unsupported();
    }
}

// ---------------------------------------------------------------------------
// audio_spdif_connect_*
// ---------------------------------------------------------------------------

bool audio_spdif_connect_thru(audio_spdif_instance_t *inst,
                              audio_buffer_pool_t *producer,
                              audio_connection_t *connection) {
    return audio_spdif_connect_extra(inst, producer, true, 2, connection);
}

bool audio_spdif_connect(audio_spdif_instance_t *inst, audio_buffer_pool_t *producer) {
    return audio_spdif_connect_thru(inst, producer, NULL);
}

bool audio_spdif_connect_extra(audio_spdif_instance_t *inst,
                               audio_buffer_pool_t *producer,
                               bool buffer_on_give, uint buffer_count,
                               audio_connection_t *connection) {
    printf("Connecting PIO S/PDIF audio\n");

    assert(producer->format->format == AUDIO_BUFFER_FORMAT_PCM_S16);
    inst->consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_SPDIF;
    inst->consumer_format.sample_freq = producer->format->sample_freq;
    inst->consumer_format.channel_count = 2;
    inst->consumer_buffer_format.format = &inst->consumer_format;
    inst->consumer_buffer_format.sample_stride = 2 * sizeof(spdif_subframe_t);

    inst->consumer_pool = audio_new_consumer_pool(&inst->consumer_buffer_format, buffer_count, PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT);
    for (audio_buffer_t *buffer = inst->consumer_pool->free_list; buffer; buffer = buffer->next) {
        init_spdif_buffer(buffer);
    }

    update_pio_frequency(inst, producer->format->sample_freq);

    // todo cleanup threading
    __mem_fence_release();

    if (!connection) {
        if (producer->format->channel_count == 2) {
#if PICO_AUDIO_SPDIF_MONO_INPUT
            panic("need to merge channels down\n");
#else
            printf("Copying stereo to stereo at %d Hz\n", (int) producer->format->sample_freq);
#endif
            printf("TODO... not completing stereo audio connection properly!\n");
        } else {
            printf("Converting mono to stereo at %d Hz\n", (int) producer->format->sample_freq);
        }
        // Initialize the embedded connection callbacks
        inst->connection.core.consumer_pool_take = wrap_consumer_take;
        inst->connection.core.consumer_pool_give = consumer_pool_give_buffer_default;
        inst->connection.core.producer_pool_take = producer_pool_take_buffer_default;
        inst->connection.core.producer_pool_give = wrap_producer_give;
        connection = &inst->connection.core;
    }
    audio_complete_connection(connection, producer, inst->consumer_pool);
    return true;
}

// ---------------------------------------------------------------------------
// DMA transfer
// ---------------------------------------------------------------------------

static void __time_critical_func(audio_start_dma_transfer)(audio_spdif_instance_t *inst) {
    assert(!inst->playing_buffer);
    audio_buffer_t *ab = take_audio_buffer(inst->consumer_pool, false);

    inst->playing_buffer = ab;
    if (!ab) {
        // just play some silence
        ab = &inst->silence_buffer;

        extern int overruns;
        overruns++;
    } else {
        gpio_put(18, 0);
    }

    dma_channel_transfer_from_buffer_now(inst->dma_channel, ab->buffer->bytes, ab->sample_count * 4);
}

// ---------------------------------------------------------------------------
// DMA IRQ handler -- iterates all registered instances
// ---------------------------------------------------------------------------

void __isr __time_critical_func(audio_spdif_dma_irq_handler)() {
#if PICO_AUDIO_SPDIF_NOOP
    assert(false);
#else
    for (uint i = 0; i < spdif_instance_count; i++) {
        audio_spdif_instance_t *inst = spdif_instances[i];
        if (dma_irqn_get_channel_status(inst->dma_irq, inst->dma_channel)) {
            dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
            DEBUG_PINS_SET(audio_timing, 4);
            // free the buffer we just finished
            if (inst->playing_buffer) {
                extern volatile uint32_t pio_samples_dma;
                pio_samples_dma++;

                give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
                inst->playing_buffer = NULL;
            }
            audio_start_dma_transfer(inst);
            DEBUG_PINS_CLR(audio_timing, 4);
        }
    }
#endif
}

// ---------------------------------------------------------------------------
// audio_spdif_set_enabled
// ---------------------------------------------------------------------------

void audio_spdif_set_enabled(audio_spdif_instance_t *inst, bool enabled) {
    if (enabled != inst->enabled) {
#ifndef NDEBUG
        if (enabled) {
            puts("Enabling PIO S/PDIF audio\n");
            printf("(on core %d\n", get_core_num());
        }
#endif
        if (enabled) {
            if (irq_enable_count[inst->dma_irq]++ == 0)
                irq_set_enabled(DMA_IRQ_0 + inst->dma_irq, true);
            audio_start_dma_transfer(inst);
            pio_sm_set_enabled(inst->pio, inst->pio_sm, true);
        } else {
            pio_sm_set_enabled(inst->pio, inst->pio_sm, false);
            if (--irq_enable_count[inst->dma_irq] == 0)
                irq_set_enabled(DMA_IRQ_0 + inst->dma_irq, false);
        }
        inst->enabled = enabled;
    }
}

// ---------------------------------------------------------------------------
// audio_spdif_enable_sync -- synchronized start for multiple instances
// ---------------------------------------------------------------------------

void audio_spdif_enable_sync(audio_spdif_instance_t *instances[], uint count) {
    assert(count > 0 && count <= PICO_AUDIO_SPDIF_MAX_INSTANCES);

    // Enable DMA IRQ and prime DMA for all instances
    for (uint i = 0; i < count; i++) {
        audio_spdif_instance_t *inst = instances[i];
        if (irq_enable_count[inst->dma_irq]++ == 0)
            irq_set_enabled(DMA_IRQ_0 + inst->dma_irq, true);
        audio_start_dma_transfer(inst);
    }

    // Build per-PIO-block SM bitmasks
    uint32_t pio_sm_mask[3] = {0, 0, 0};
    for (uint i = 0; i < count; i++) {
        audio_spdif_instance_t *inst = instances[i];
        for (uint p = 0; p < 3; p++) {
            PIO block = pio_block_from_index(p);
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
            pio_enable_sm_mask_in_sync(pio_block_from_index(p), pio_sm_mask[p]);
        }
    }
    restore_interrupts(save);

    // Mark all instances enabled
    for (uint i = 0; i < count; i++) {
        instances[i]->enabled = true;
    }
}
