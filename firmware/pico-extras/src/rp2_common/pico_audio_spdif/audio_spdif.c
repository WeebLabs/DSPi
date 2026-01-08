/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/audio_spdif.h"
#include <pico/audio_spdif/sample_encoding.h>
#include "audio_spdif.pio.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"


CU_REGISTER_DEBUG_PINS(audio_timing)

// ---- select at most one ---
//CU_SELECT_DEBUG_PINS(audio_timing)


#define audio_pio __CONCAT(pio, PICO_AUDIO_SPDIF_PIO)
#define GPIO_FUNC_PIOx __CONCAT(GPIO_FUNC_PIO, PICO_AUDIO_SPDIF_PIO)
#define DREQ_PIOx_TX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_AUDIO_SPDIF_PIO), _TX0)

struct {
    audio_buffer_t *playing_buffer;
    uint32_t freq;
    uint8_t pio_sm;
    uint8_t dma_channel;
} shared_state;

static audio_format_t pio_spdif_consumer_format;
audio_buffer_format_t pio_spdif_consumer_buffer_format = {
        .format = &pio_spdif_consumer_format,
};

static audio_buffer_t silence_buffer = {
        .sample_count =  PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT,
        .max_sample_count =  PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT,
        .format = &pio_spdif_consumer_buffer_format
};

static void __isr __time_critical_func(audio_spdif_dma_irq_handler)();

const audio_spdif_config_t audio_spdif_default_config = {
    .pin = PICO_AUDIO_SPDIF_PIN,
    .pio_sm = 0,
    .dma_channel = 0,
};

#define SR_44100 0
#define SR_48000 1

#define PREAMBLE_X 0b11001001
#define PREAMBLE_Y 0b01101001
#define PREAMBLE_Z 0b00111001

//#define SPDIF_CONTROL_WORD 0x4

#define SPDIF_CONTROL_WORD (\
    0x4 | /* copying allowed */ \
    0x20 | /* PCM encoder/decoder */ \
    (SR_44100 << 24) /* todo is this required */ \
    )

// each buffer is pre-filled with data
static void init_spdif_buffer(audio_buffer_t *buffer) {
    // BIT DESCRIPTIONS:
    //    0–3 	Preamble 	                A synchronisation preamble (biphase mark code violation) for audio blocks, frames, and subframes.
    //    4–7 	Auxiliary sample (optional) A low-quality auxiliary channel used as specified in the channel status word, notably for producer talkback or recording studio-to-studio communication.
    //    8–27, or 4–27 	                Audio sample. One sample stored with most significant bit (MSB) last. If the auxiliary sample is used, bits 4–7 are not included. Data with smaller sample bit depths always have MSB at bit 27 and are zero-extended towards the least significant bit (LSB).
    //    28 	Validity (V) 	            Unset if the audio data are correct and suitable for D/A conversion. During the presence of defective samples, the receiving equipment may be instructed to mute its output. It is used by most CD players to indicate that concealment rather than error correction is taking place.
    //    29 	User data (U) 	            Forms a serial data stream for each channel (with 1 bit per frame), with a format specified in the channel status word.
    //    30 	Channel status (C) 	        Bits from each frame of an audio block are collated giving a 192-bit channel status word. Its structure depends on whether AES3 or S/PDIF is used.
    //    31 	Parity (P)

    // We want to pre-encode (in our fixed length 192 buffers), the
    //   * Preamble
    //   * Aux (0)
    //   * Low4 (0)
    //
    //   * V(0)
    //   * U(0)
    //   * C(0) (or from SPDIF_CONTROL_WORD in the first 32)

    // note everything is encoded in NRZI
    // regular data bits are encoded
    // 0 -> 10 (LSB first)
    // 1 -> 11

    assert(buffer->max_sample_count == PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT);
    spdif_subframe_t *p = (spdif_subframe_t *)buffer->buffer->bytes;
    for(uint i=0;i<PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT;i++) {
        uint c_bit = i < 32 ? (SPDIF_CONTROL_WORD >> i) & 1u: 0;
//        p->l = (i ? PREAMBLE_X : PREAMBLE_Z) | 0b10101010101010100000000 | 0x55000000;
//        p->h = 0x55000000u | (c_bit << 25u) | 0x0055555555;
//        p++;
//        p->l = PREAMBLE_Y | 0b10101010101010100000000 | 0x55000000;
//        p->h = 0x55000000u | (c_bit << 25u) | 0x0055555555;
//        p++;

        p->l = (i ? PREAMBLE_X : PREAMBLE_Z) | 0b10101010101010100000000; // 0-7 preamble, 8-15 aux samp, 16 - 56 sample
        p->h = 0x55000000u | (c_bit << 29u);                              // 0-24 sample low, 25 26 valid, 27 28 user data, 29 30 status
        p++;
        p->l = PREAMBLE_Y | 0b10101010101010100000000;
        p->h = 0x55000000u | (c_bit << 29u); // 25 -> 29
        p++;

    }
}

uint32_t spdif_lookup[256];

const audio_format_t *audio_spdif_setup(const audio_format_t *intended_audio_format,
                                               const audio_spdif_config_t *config) {
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
    uint func = GPIO_FUNC_PIOx;
    gpio_set_function(config->pin, func);

    uint8_t sm = shared_state.pio_sm = config->pio_sm;
    pio_sm_claim(audio_pio, sm);

    uint offset = pio_add_program(audio_pio, &audio_spdif_program);

    spdif_program_init(audio_pio, sm, offset, config->pin);

    silence_buffer.buffer = pico_buffer_alloc(PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT * 2 * sizeof(spdif_subframe_t));
    init_spdif_buffer(&silence_buffer);
    spdif_subframe_t *sf = (spdif_subframe_t *)silence_buffer.buffer->bytes;
    for(uint i=0;i<silence_buffer.sample_count;i++) {
        spdif_update_subframe(sf++, 0);
        spdif_update_subframe(sf++, 0);
    }

    __mem_fence_release();
    uint8_t dma_channel = config->dma_channel;
    dma_channel_claim(dma_channel);

    shared_state.dma_channel = dma_channel;


    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);

    channel_config_set_dreq(&dma_config,
                            DREQ_PIOx_TX0 + sm
    );

    dma_channel_configure(dma_channel,
                          &dma_config,
                          &audio_pio->txf[sm],  // dest
                          NULL, // src
                          0, // count
                          false // trigger
    );

    irq_set_exclusive_handler(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, audio_spdif_dma_irq_handler);
    //irq_add_shared_handler(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, audio_spdif_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    dma_irqn_set_channel_enabled(PICO_AUDIO_SPDIF_DMA_IRQ, dma_channel, 1);
    return intended_audio_format;
}

static audio_buffer_pool_t *audio_spdif_consumer;

static void update_pio_frequency(uint32_t sample_freq) {
    printf("setting pio freq %d\n", (int) sample_freq);
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    // coincidentally * 256 (for 8 bit fraction) / 2 (channels) * 32 (bits) * 2 (time periods) * 2 cycles per time period)
    //uint32_t divider = system_clock_frequency / sample_freq;

    // ceil the divider instead (gets us a closer hit to 44100 at 176.57142857142858 MHz: 44098.7583844)
    uint32_t divider = system_clock_frequency / sample_freq + (system_clock_frequency % sample_freq != 0);

    printf("System clock at %u, S/PDIF clock divider 0x%x/256\n", (uint) system_clock_frequency, (uint)divider);
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(audio_pio, shared_state.pio_sm, divider >> 8u, divider & 0xffu);
    shared_state.freq = sample_freq;
}

static audio_buffer_t *wrap_consumer_take(audio_connection_t *connection, bool block) {
    // support dynamic frequency shifting
    if (connection->producer_pool->format->sample_freq != shared_state.freq) {
        update_pio_frequency(connection->producer_pool->format->sample_freq);
    }
    return consumer_pool_take_buffer_default(connection, block);
}

static void wrap_producer_give(audio_connection_t *connection, audio_buffer_t *buffer) {
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

static struct producer_pool_blocking_give_connection m2s_audio_spdif_connection = {
        .core = {
                .consumer_pool_take = wrap_consumer_take,
                .consumer_pool_give = consumer_pool_give_buffer_default,
                .producer_pool_take = producer_pool_take_buffer_default,
                .producer_pool_give = wrap_producer_give,
        }
};

bool audio_spdif_connect_thru(audio_buffer_pool_t *producer, audio_connection_t *connection) {
    return audio_spdif_connect_extra(producer, true, 2, connection);
}

bool audio_spdif_connect(audio_buffer_pool_t *producer) {
    return audio_spdif_connect_thru(producer, NULL);
}

bool audio_spdif_connect_extra(audio_buffer_pool_t *producer, bool buffer_on_give, uint buffer_count,
                               audio_connection_t *connection) {
    printf("Connecting PIO S/PDIF audio\n");

    assert(producer->format->format == AUDIO_BUFFER_FORMAT_PCM_S16);
    pio_spdif_consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_SPDIF;
    pio_spdif_consumer_format.sample_freq = producer->format->sample_freq;
    pio_spdif_consumer_format.channel_count = 2;
    pio_spdif_consumer_buffer_format.sample_stride = 2 * sizeof(spdif_subframe_t);

    audio_spdif_consumer = audio_new_consumer_pool(&pio_spdif_consumer_buffer_format, buffer_count, PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT);
    for (audio_buffer_t *buffer = audio_spdif_consumer->free_list; buffer; buffer = buffer->next) {
        init_spdif_buffer(buffer);
    }

    update_pio_frequency(producer->format->sample_freq);

    // todo cleanup threading
    __mem_fence_release();

    if (!connection) {
        if (producer->format->channel_count == 2) {
#if PICO_AUDIO_SPDIF_MONO_INPUT
            panic("need to merge channels down\n");
#else
            printf("Copying stereo to stereo at %d Hz\n", (int) producer->format->sample_freq);
#endif
            // todo we should support pass thru option anyway
            printf("TODO... not completing stereo audio connection properly!\n");
        } else {
            printf("Converting mono to stereo at %d Hz\n", (int) producer->format->sample_freq);
        }
        connection = &m2s_audio_spdif_connection.core;
    }
    audio_complete_connection(connection, producer, audio_spdif_consumer);
    return true;
}

static struct buffer_copying_on_consumer_take_connection m2s_audio_spdif_connection_s8 = {
        .core = {
                .consumer_pool_take = wrap_consumer_take,
                .consumer_pool_give = consumer_pool_give_buffer_default,
                .producer_pool_take = producer_pool_take_buffer_default,
                .producer_pool_give = wrap_producer_give,
        }
};

bool audio_spdif_connect_s8(audio_buffer_pool_t *producer) {
    panic_unsupported(); // needs fixing up
    printf("Connecting PIO S/PDIF audio (U8)\n");

    // todo we need to pick a connection based on the frequency - e.g. 22050 can be more simply upsampled to 44100
    assert(producer->format->format == AUDIO_BUFFER_FORMAT_PCM_S8);
    pio_spdif_consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_SPDIF;
    // todo we could do mono
    // todo we can't match exact, so we should return what we can do
    pio_spdif_consumer_format.sample_freq = producer->format->sample_freq;
    pio_spdif_consumer_format.channel_count = 2;
    pio_spdif_consumer_buffer_format.sample_stride = 2 * sizeof(spdif_subframe_t);

    // we do this on take so should do it quickly...
    uint samples_per_buffer = 256;
    // todo with take we really only need 1 buffer
    audio_spdif_consumer = audio_new_consumer_pool(&pio_spdif_consumer_buffer_format, 2, samples_per_buffer);
    // todo we need a method to calculate this in clocks
    uint32_t system_clock_frequency = 48000000;
//    uint32_t divider = system_clock_frequency * 256 / producer->format->sample_freq * 16 * 4;
    uint32_t divider = system_clock_frequency * 4 / producer->format->sample_freq; // avoid arithmetic overflow
    pio_sm_set_clkdiv_int_frac(audio_pio, shared_state.pio_sm, divider >> 8u, divider & 0xffu);

    // todo cleanup threading
    __mem_fence_release();

    audio_connection_t *connection;
    if (producer->format->channel_count == 2) {
        // todo we should support pass thru option anyway
        printf("TODO... not completing stereo audio connection properly!\n");
        connection = &m2s_audio_spdif_connection_s8.core;
    } else {
        printf("Converting mono to stereo at %d Hz\n", (int) producer->format->sample_freq);
        connection = &m2s_audio_spdif_connection_s8.core;
    }
    audio_complete_connection(connection, producer, audio_spdif_consumer);
    return true;
}

static void __time_critical_func(audio_start_dma_transfer)() {
    assert(!shared_state.playing_buffer);
    audio_buffer_t *ab = take_audio_buffer(audio_spdif_consumer, false);

    shared_state.playing_buffer = ab;
    if (!ab) {

        //DEBUG_PINS_XOR(audio_timing, 1);
        //DEBUG_PINS_XOR(audio_timing, 2);
        //DEBUG_PINS_XOR(audio_timing, 1);
        //DEBUG_PINS_XOR(audio_timing, 2);
        // just play some silence
        ab = &silence_buffer;

        extern int overruns;
        overruns++;
    } else {
        gpio_put(18, 0);

    }

    //assert(ab->sample_count);
    // todo better naming of format->format->format!!
    //assert(ab->format->format->format == AUDIO_BUFFER_FORMAT_PIO_SPDIF);
    //assert(ab->format->format->channel_count == 2);
    //assert(ab->format->sample_stride == 2 * sizeof(spdif_subframe_t));

    dma_channel_transfer_from_buffer_now(shared_state.dma_channel, ab->buffer->bytes, ab->sample_count * 4);
}

// irq handler for DMA
void __isr __time_critical_func(audio_spdif_dma_irq_handler)() {
#if PICO_AUDIO_SPDIF_NOOP
    assert(false);
#else
    uint dma_channel = shared_state.dma_channel;
    if (dma_irqn_get_channel_status(PICO_AUDIO_SPDIF_DMA_IRQ, dma_channel)) {
        dma_irqn_acknowledge_channel(PICO_AUDIO_SPDIF_DMA_IRQ, dma_channel);
//gpio_put(28, 1);
        DEBUG_PINS_SET(audio_timing, 4);
        // free the buffer we just finished
        if (shared_state.playing_buffer) {
            extern volatile uint32_t pio_samples_dma;
            pio_samples_dma++;

            give_audio_buffer(audio_spdif_consumer, shared_state.playing_buffer);
#ifndef NDEBUG
            shared_state.playing_buffer = NULL;
#endif
        }
        audio_start_dma_transfer();
        DEBUG_PINS_CLR(audio_timing, 4);
//gpio_put(28, 0);
    }
#endif
}

static bool audio_enabled;

void audio_spdif_set_enabled(bool enabled) {
    if (enabled != audio_enabled) {
#ifndef NDEBUG
        if (enabled)
        {
            puts("Enabling PIO S/PDIF audio\n");
            printf("(on core %d\n", get_core_num());
        }
#endif
        irq_set_enabled(DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ, enabled);

        if (enabled) {
            audio_start_dma_transfer();
        }

        pio_sm_set_enabled(audio_pio, shared_state.pio_sm, enabled);

        audio_enabled = enabled;
    }
}

