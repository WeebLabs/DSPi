/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "pico/audio_pwm.h"

#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "pico/audio_pwm/sample_encoding.h"

#include "audio_pwm.pio.h"

// TODO: add noise shaped fixed dither


#define audio_pio __CONCAT(pio, PICO_AUDIO_PWM_PIO)
#define GPIO_FUNC_PIOx __CONCAT(GPIO_FUNC_PIO, PICO_AUDIO_PWM_PIO)
#define DREQ_PIOx_TX0 __CONCAT(__CONCAT(DREQ_PIO, PICO_AUDIO_PWM_PIO), _TX0)

// ======================
// == DEBUGGING =========

#define ENABLE_PIO_AUDIO_PWM_ASSERTIONS

CU_REGISTER_DEBUG_PINS(audio_timing, audio_underflow)

// ---- select at most one ---
//CU_SELECT_DEBUG_PINS(audio_timing)

// ======================

#ifdef ENABLE_PIO_AUDIO_PWM_ASSERTIONS
#define audio_assert(x) assert(x)
#else
#define audio_assert(x) (void)0
#endif

#define _UNDERSCORE(x, y) x ## _ ## y
#define _CONCAT(x, y) _UNDERSCORE(x,y)
#define audio_program _CONCAT(program_name,program)
#define audio_program_get_default_config _CONCAT(program_name,program_get_default_config)
#define audio_entry_point _CONCAT(program_name,offset_entry_point)

static bool audio_enabled;
static bool push_queuing_to_core1;

static void __isr __time_critical_func(audio_pwm_dma_irq_handler)();

static struct {
    audio_buffer_pool_t *playback_buffer_pool[PICO_AUDIO_PWM_MAX_CHANNELS];
    audio_buffer_t *playing_buffer[PICO_AUDIO_PWM_MAX_CHANNELS];
    // ----- begin protected by free_list_spin_lock -----
    uint8_t pio_sm[PICO_AUDIO_PWM_MAX_CHANNELS];
    uint8_t dma_channel[PICO_AUDIO_PWM_MAX_CHANNELS];
    int channel_count;
} shared_state;

const audio_pwm_channel_config_t default_left_channel_config =
        {
                .core = {
                        .base_pin = PICO_AUDIO_PWM_L_PIN,
                        .pio_sm = 0,
                        .dma_channel = 0
                },
                .pattern = 1,
        };

const audio_pwm_channel_config_t default_right_channel_config =
        {
                .core = {
                        .base_pin = PICO_AUDIO_PWM_R_PIN,
                        .pio_sm = 1,
                        .dma_channel = 1
                },
                .pattern = 1,
        };

const audio_pwm_channel_config_t default_mono_channel_config =
        {
                .core = {
                        .base_pin = PICO_AUDIO_PWM_MONO_PIN,
                        .pio_sm = 0,
                        .dma_channel = 0
                },
                .pattern = 3,
        };

static audio_buffer_t silence_buffer;

static inline void audio_start_dma_transfer(int ch)
{
#if PICO_AUDIO_PWM_NOOP
    assert(false);
#else
    assert(!shared_state.playing_buffer[ch]);
    audio_buffer_t *ab = take_audio_buffer(shared_state.playback_buffer_pool[ch], false);

    shared_state.playing_buffer[ch] = ab;
    DEBUG_PINS_SET(audio_underflow, 4);
    if (!ab)
    {
        DEBUG_PINS_XOR(audio_underflow, 2);
        // just play some silence
        ab = &silence_buffer;
//        static int foo;
//        printf("underflow %d\n", foo++);
        DEBUG_PINS_XOR(audio_underflow, 2);
    }
    DEBUG_PINS_CLR(audio_underflow, 4);
    assert(ab->sample_count);
    // todo better naming of format->format->format!!
    assert(ab->format->format->format == NATIVE_BUFFER_FORMAT);
    assert(ab->format->format->channel_count == 1);
    assert(ab->format->sample_stride == sizeof(pwm_cmd_t));
    dma_channel_transfer_from_buffer_now(shared_state.dma_channel[ch], ab->buffer->bytes,
                                         ab->sample_count * sizeof(pwm_cmd_t) / 4);
}

semaphore_t sem_transfer_buffer_fill, sem_transfer_buffer_drain;
void *volatile transfer_buffer;
int32_t transfer_buffer_sample_count;

// irq handler for DMA
static void __isr __time_critical_func(audio_pwm_dma_irq_handler)()
{
#if PICO_AUDIO_PWM_NOOP
    assert(false);
#else
    // todo better DMA channel handling? (should we combine to keep channels in sync?)
    //  (pico_audio - sync should be maintained by source of pico_audio buffers, though we need to be able to insert
    //  the correct amount of silence to re-align)
    for(int ch = 0; ch < shared_state.channel_count; ch++)
    {
        uint dma_channel = shared_state.dma_channel[ch];
        if (dma_irqn_get_channel_status(PICO_AUDIO_PWM_DMA_IRQ, dma_channel)) {
            dma_irqn_acknowledge_channel(PICO_AUDIO_PWM_DMA_IRQ, dma_channel);
            DEBUG_PINS_SET(audio_timing, 4);
            // free the buffer we just finished
            if (shared_state.playing_buffer[ch])
            {
                give_audio_buffer(shared_state.playback_buffer_pool[ch], shared_state.playing_buffer[ch]);
#ifndef NDEBUG
                shared_state.playing_buffer[ch] = 0;
#endif
            }
            audio_start_dma_transfer(ch);
            DEBUG_PINS_CLR(audio_timing, 4);
        }
    }
#endif
}

audio_format_t pwm_consumer_format;
audio_buffer_format_t pwm_consumer_buffer_format = {
        .format = &pwm_consumer_format,
        .sample_stride = sizeof(pwm_cmd_t)
};
audio_buffer_pool_t *pwm_consumer_pool;

const audio_format_t *audio_pwm_setup(const audio_format_t *intended_audio_format, int32_t max_latency_ms,
                                               const audio_pwm_channel_config_t *channel_config0, ...)
{
    va_list args;

    assert(max_latency_ms == -1); // not implemented yet
    __builtin_memset(&shared_state, 0, sizeof(shared_state));
    // init non zero members
#if !PICO_AUDIO_PWM_NOOP

    shared_state.channel_count = intended_audio_format->channel_count;
#if !PICO_AUDIO_PWM_ENABLE_NOISE_SHAPING
    pwm_consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_PWM_CMD1;
    pwm_consumer_format.channel_count = 1;
#else
    pwm_consumer_format.format = AUDIO_BUFFER_FORMAT_PIO_PWM_CMD3;
    pwm_consumer_format.channel_count = 1;
#endif
#ifndef AUDIO_HALF_FREQ
    pwm_consumer_format.sample_freq = 22058;
#else
    pwm_consumer_format.sample_freq = 11029;
#endif

    for(int i = 0; i < shared_state.channel_count; i++)
    {
        shared_state.playback_buffer_pool[i] = audio_new_consumer_pool(&pwm_consumer_buffer_format,
                                                                       PICO_AUDIO_PWM_BUFFERS_PER_CHANNEL,
                                                                       PICO_AUDIO_PWM_BUFFER_SAMPLE_LENGTH);
    }
    __mem_fence_release();

    silence_buffer.buffer = pico_buffer_alloc(PICO_AUDIO_PWM_SILENCE_BUFFER_SAMPLE_LENGTH * sizeof(silence_cmd));
    for(int i = 0; i < PICO_AUDIO_PWM_SILENCE_BUFFER_SAMPLE_LENGTH; i++)
    {
        __builtin_memcpy((void *) (silence_buffer.buffer->bytes + i * sizeof(silence_cmd)), &silence_cmd,
                         sizeof(silence_cmd));
    }
    silence_buffer.sample_count = PICO_AUDIO_PWM_SILENCE_BUFFER_SAMPLE_LENGTH;
    silence_buffer.format = &pwm_consumer_buffer_format;

    va_start(args, channel_config0);
    uint offset = pio_add_program(audio_pio, &audio_program);

    const audio_pwm_channel_config_t *config = channel_config0;

    irq_add_shared_handler(DMA_IRQ_0 + PICO_AUDIO_PWM_DMA_IRQ, audio_pwm_dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    for(int ch = 0; ch < shared_state.channel_count; ch++)
    {
        if (!config)
        {
            config = va_arg(args, const struct audio_pwm_channel_config *);
        }

        gpio_set_function(config->core.base_pin, GPIO_FUNC_PIOx);

        uint8_t sm = shared_state.pio_sm[ch] = config->core.pio_sm;
        pio_sm_claim(audio_pio, sm);

        pio_sm_config sm_config = audio_program_get_default_config(offset);
        sm_config_set_out_pins(&sm_config, config->core.base_pin, 1);
        sm_config_set_sideset_pins(&sm_config, config->core.base_pin);
        // disable auto-pull for !OSRE (which doesn't work with auto-pull)
        static_assert(CYCLES_PER_SAMPLE <= 18, "");
        sm_config_set_out_shift(&sm_config, true, false, CMD_BITS + CYCLES_PER_SAMPLE);
        pio_sm_init(audio_pio, sm, offset, &sm_config);

        pio_sm_set_consecutive_pindirs(audio_pio, sm, config->core.base_pin, 1, true);
        pio_sm_set_pins(audio_pio, sm, 0);

        // todo this should be part of sm_init
        pio_sm_exec(audio_pio, sm, pio_encode_jmp(offset + audio_entry_point)); // jmp to ep

        uint8_t dma_channel = config->core.dma_channel;
        dma_channel_claim(dma_channel);

        shared_state.dma_channel[ch] = dma_channel;

        dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);

        channel_config_set_dreq(&dma_config, DREQ_PIOx_TX0 + sm);
        dma_channel_configure(dma_channel,
                              &dma_config,
                              &audio_pio->txf[sm],  // dest
                              NULL, // src
                              0, // count
                              false // trigger
        );
        dma_irqn_set_channel_enabled(PICO_AUDIO_PWM_DMA_IRQ, dma_channel, 1);
        config = 0;
    }

    va_end(args);
#endif
#ifndef NDEBUG
    puts("PicoAudio: initialized\n");
#endif
    pwm_consumer_pool = (shared_state.playback_buffer_pool[0]); // forcing channel 0 to be consumer for now
    // todo we need to update this to what is exact
    return intended_audio_format;
}

void audio_pwm_set_enabled(bool enabled)
{
    if (enabled != audio_enabled)
    {
#ifndef NDEBUG
        if (enabled)
        {
            puts("Enabling PIO PWM audio\n");
        }
#endif
#if !PICO_AUDIO_PWM_NOOP
        irq_set_enabled(DMA_IRQ_0 + PICO_AUDIO_PWM_DMA_IRQ, enabled);

        if (enabled)
        {
            // todo this is wrong
            for(int ch = 0; ch < shared_state.channel_count; ch++)
            {
                audio_start_dma_transfer(ch);
            }
        }

        // todo need to start them in sync - need WAIT in program
        for(int ch = 0; ch < shared_state.channel_count; ch++)
        {
            pio_sm_set_enabled(audio_pio, shared_state.pio_sm[ch], enabled);
        }
#endif

        audio_enabled = enabled;
    }
}

#pragma GCC push_options
#ifdef __arm__
// seems uber keen to inline audio_queue_samples which is large
#pragma GCC optimize("O1")
#endif

void core1_worker()
{
    while (true)
    {
        sem_acquire_blocking(&sem_transfer_buffer_drain);
//        audio_queue_samples(0, transfer_buffer, transfer_buffer_sample_count, 1, true);
        sem_release(&sem_transfer_buffer_fill);
    }
    __builtin_unreachable();
}

#pragma GCC pop_options

bool audio_start_queue_work_on_core_1()
{
    if (!push_queuing_to_core1)
    {
        puts("In the spirit of the season, core 1 is helping out too...\n");
        sem_init(&sem_transfer_buffer_drain, 0, 1);
        // one fill is implicitly owned by the client application as it has a buffer
        // (note the count here is actually the number of buffers the client has)
        sem_init(&sem_transfer_buffer_fill, 2, 1);
        multicore_launch_core1(core1_worker);
        push_queuing_to_core1 = true;
    }
    return true;
}

#endif

static struct producer_pool_blocking_give_connection producer_pool_blocking_give_connection_singleton = {
        .core = {
                .consumer_pool_take = consumer_pool_take_buffer_default,
                .consumer_pool_give = consumer_pool_give_buffer_default,
                .producer_pool_take = producer_pool_take_buffer_default,
        }
        // rest 0 initialized
};

bool audio_pwm_default_connect(audio_buffer_pool_t *producer_pool, bool dedicate_core_1)
{
    if (!dedicate_core_1)
    {
        printf("Connecting PIO PWM audio via 'blocking give'\n");
        assert(pwm_consumer_pool);
        assert(pwm_consumer_pool->format->channel_count == 1); // for now
        // todo oops this is pulling in everything!
        switch (producer_pool->format->format) {
            case AUDIO_BUFFER_FORMAT_PCM_S16:
                producer_pool_blocking_give_connection_singleton.core.producer_pool_give = producer_pool_blocking_give_to_pwm_s16;
                break;
            case AUDIO_BUFFER_FORMAT_PCM_S8:
                producer_pool_blocking_give_connection_singleton.core.producer_pool_give = producer_pool_blocking_give_to_pwm_s8;
                break;
            case AUDIO_BUFFER_FORMAT_PCM_U16:
                producer_pool_blocking_give_connection_singleton.core.producer_pool_give = producer_pool_blocking_give_to_pwm_s16;
                break;
            case AUDIO_BUFFER_FORMAT_PCM_U8:
                producer_pool_blocking_give_connection_singleton.core.producer_pool_give = producer_pool_blocking_give_to_pwm_s8;
                break;
            default:
                return false;
        }
        audio_complete_connection(&producer_pool_blocking_give_connection_singleton.core, producer_pool,
                                  pwm_consumer_pool);
        return true;
    }
    else
    {
        assert(false);
    }
    return false;
}
