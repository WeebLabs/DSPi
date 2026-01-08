/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>

#include "pico/sample_conversion.h"
#include "pico/audio_pwm/sample_encoding.h"
#include "pico/audio_pwm.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"

CU_REGISTER_DEBUG_PINS(encoding)
//CU_SELECT_DEBUG_PINS(encoding)

#ifndef PICO_AUDIO_PWM_DEFAULT_CORRECTION_MODE
#define PICO_AUDIO_PWM_DEFAULT_CORRECTION_MODE dither
#endif

static enum audio_correction_mode audio_correction_mode = PICO_AUDIO_PWM_DEFAULT_CORRECTION_MODE;

struct FmtPWM  : public FmtDetails<pwm_cmd_t> {
};

bool audio_pwm_set_correction_mode(enum audio_correction_mode mode)
{
    if (mode == none || mode == dither || mode == fixed_dither)
    {
        audio_correction_mode = mode;
        return true;
    }
#ifdef ENABLE_NOISE_SHAPING
    if (mode == noise_shaped_dither) {
        audio_correction_mode = mode;
        return true;
    }
#endif
    return false;
}

enum audio_correction_mode audio_pwm_get_correction_mode()
{
    return audio_correction_mode;
}

template <typename FromFmt> void
    __no_inline_not_in_flash_func(encode_samples_none)(int s_count, const typename FromFmt::sample_t *s, pwm_cmd_t *encoded)
{
    const typename FromFmt::sample_t *s_end = s + s_count * FromFmt::channel_count;
    // hacky cast to allow use to DITHER_BITS > 1
    uint32_t *e = (uint32_t *) encoded;

#if !PIO_AUDIO_PWM_NO_INTERP_SAVE
    interp_hw_save_t saver;
    interp_save(interp0, &saver);
#endif

//    interp_configure_with_signed(interp0, 0, FRACTIONAL_BITS, 0, QUANTIZED_BITS - 1);
    interp_config config = interp_default_config();
    interp_config_set_signed(&config, true);
    interp_config_set_shift(&config, FRACTIONAL_BITS);
    interp_config_set_mask(&config, 0, QUANTIZED_BITS - 1);
    interp_set_config(interp0, 0, &config);
    interp0->base[0] = 0x8000u >> FRACTIONAL_BITS;

    while (s < s_end)
    {
        // accum = signed_sample_16
        interp0->accum[0] = sample_converter<FmtS16, FromFmt>::convert_sample(*s);
        // quant = ((0x8000 + signed_sample_16) >> FRACTIONAL_BITS) & QUANTIZED_MASK
        uint32_t quant = interp0->pop[0];

        assert(quant >= 0 && quant <= 127);
        uint32_t cmd = MAKE_CMD(quant);
        for(uint k = 0; k < OUTER_LOOP_COUNT; k++)
        {
            *e++ = cmd;
        }
        s += FromFmt::channel_count;
    }

#if !PIO_AUDIO_PWM_NO_INTERP_SAVE
    interp_restore(interp0, &saver);
#endif
}

//void gen_fixed_dither()
//{
//    for(int i = 0; i < 16; i++)
//    {
//        printf("0b");
//        int e = 0;
//        for(int j = 0; j < 15; j++)
//        {
//            e += i;
//            if (e >= 16)
//            {
//                printf("1");
//                e -= 16;
//            }
//            else
//            {
//                printf("0");
//            }
//        }
//        printf("\n");
//    }
//
//    for(int i = 0; i < 16; i++)
//    {
//        printf("0b");
//        int e = 0;
//        for(int j = 0; j < 15; j++)
//        {
//            e += i;
//            if (e >= 16)
//            {
//                printf("100");
//                e -= 16;
//            }
//            else
//            {
//                printf("000");
//            }
//            if (j == 4 || j == 9)
//            {
//                printf(", 0b");
//            }
//        }
//        printf("\n");
//    }

#if DITHER_BITS == 3
#define FIXED_DITHER_SHIFT 2
// note 4 not 3 which wastes 16 bytes, but allows interpolator to handle address
static uint32_t fixed_dither_table[16*(1<<FIXED_DITHER_SHIFT)] = {
        0b000000000000000 << CMD_BITS, 0b000000000000000 << CMD_BITS, 0b000000000000000 << CMD_BITS, 0 << CMD_BITS,
        0b000000000000000 << CMD_BITS, 0b000000000000000 << CMD_BITS, 0b000000000000000 << CMD_BITS, 0 << CMD_BITS,
        0b000000000000000 << CMD_BITS, 0b000000100000000 << CMD_BITS, 0b000000000000000 << CMD_BITS, 0 << CMD_BITS,
        0b000000000000000 << CMD_BITS, 0b100000000000000 << CMD_BITS, 0b100000000000000 << CMD_BITS, 0 << CMD_BITS,
        0b000000000100000 << CMD_BITS, 0b000000100000000 << CMD_BITS, 0b000100000000000 << CMD_BITS, 0 << CMD_BITS,
        0b000000000100000 << CMD_BITS, 0b000100000000100 << CMD_BITS, 0b000000100000000 << CMD_BITS, 0 << CMD_BITS,
        0b000000100000000 << CMD_BITS, 0b100000100000000 << CMD_BITS, 0b100000000100000 << CMD_BITS, 0 << CMD_BITS,
        0b000000100000100 << CMD_BITS, 0b000100000000100 << CMD_BITS, 0b000100000100000 << CMD_BITS, 0 << CMD_BITS,
        0b000100000100000 << CMD_BITS, 0b100000100000100 << CMD_BITS, 0b000100000100000 << CMD_BITS, 0 << CMD_BITS,
        0b000100000100000 << CMD_BITS, 0b100000100100000 << CMD_BITS, 0b100000100000100 << CMD_BITS, 0 << CMD_BITS,
        0b000100000100100 << CMD_BITS, 0b000100100000100 << CMD_BITS, 0b000100100000100 << CMD_BITS, 0 << CMD_BITS,
        0b000100100000100 << CMD_BITS, 0b100000100100000 << CMD_BITS, 0b100100000100100 << CMD_BITS, 0 << CMD_BITS,
        0b000100100100000 << CMD_BITS, 0b100100100000100 << CMD_BITS, 0b100100000100100 << CMD_BITS, 0 << CMD_BITS,
        0b000100100100100 << CMD_BITS, 0b000100100100100 << CMD_BITS, 0b000100100100100 << CMD_BITS, 0 << CMD_BITS,
        0b000100100100100 << CMD_BITS, 0b100100100000100 << CMD_BITS, 0b100100100100100 << CMD_BITS, 0 << CMD_BITS,
        0b000100100100100 << CMD_BITS, 0b100100100100100 << CMD_BITS, 0b100100100100100 << CMD_BITS, 0 << CMD_BITS,
};
#elif DITHER_BITS == 1
#define FIXED_DITHER_SHIFT 0
static uint32_t fixed_dither_table[16*(1<<FIXED_DITHER_SHIFT)] = {
    0b000000000000000 << CMD_BITS,
    0b000000000000000 << CMD_BITS,
    0b000000010000000 << CMD_BITS,
    0b000001000010000 << CMD_BITS,
    0b000100010001000 << CMD_BITS,
    0b000100100100100 << CMD_BITS,
    0b001001010010010 << CMD_BITS,
    0b001010100101010 << CMD_BITS,
    0b010101010101010 << CMD_BITS,
    0b010101011010101 << CMD_BITS,
    0b010110110101101 << CMD_BITS,
    0b011011011011011 << CMD_BITS,
    0b011101110111011 << CMD_BITS,
    0b011110111101111 << CMD_BITS,
    0b011111110111111 << CMD_BITS,
    0b011111111111111 << CMD_BITS,
};
#else
#error
#endif
template <typename FromFmt> void
__no_inline_not_in_flash_func(encode_samples_fixed_dither)(int s_count, const typename FromFmt::sample_t *s, pwm_cmd_t *encoded)
{
    const typename FromFmt::sample_t *s_end = s + s_count * FromFmt::channel_count;
    // hacky cast to allow use to DITHER_BITS > 1
    uint32_t *e = (uint32_t *) encoded;

#if !PIO_AUDIO_PWM_NO_INTERP_SAVE
    interp_hw_save_t saver;
    interp_save(interp0, &saver);
#endif

    //interp_configure_with_signed(interp0, 0, FRACTIONAL_BITS, 0, QUANTIZED_BITS - 1);
    interp_config config = interp_default_config();
    interp_config_set_shift(&config, FRACTIONAL_BITS);
    interp_config_set_mask(&config, 0, QUANTIZED_BITS - 1);
    interp_config_set_signed(&config, true);
    interp_set_config(interp0, 0, &config);

//    interp_configure_with_cross_input(interp0, 1, FRACTIONAL_BITS - FIXED_DITHER_SHIFT - 6, FIXED_DITHER_SHIFT + 2, FIXED_DITHER_SHIFT + 5);
    config = interp_default_config();
    interp_config_set_shift(&config, FRACTIONAL_BITS - FIXED_DITHER_SHIFT - 6);
    interp_config_set_mask(&config, FIXED_DITHER_SHIFT + 2, FIXED_DITHER_SHIFT + 5);
    interp_config_set_cross_input(&config, true);
    interp_set_config(interp0, 1, &config);

    interp0->base[0] = 0x8000u >> FRACTIONAL_BITS;
    interp0->base[1] = (uintptr_t)fixed_dither_table;

    static int16_t error = 0;
    while (s < s_end)
    {
        // accum = signed_sample_16
        interp0->accum[0] = sample_converter<FmtS16, FromFmt>::convert_sample(*s) + error;
        uint32_t *fdt = (uint32_t *)interp0->peek[1];
        uint32_t quant = interp0->pop[0];

        assert(quant >= 0 && quant <= 127);
        uint32_t cmd = MAKE_CMD(quant);
        for(uint k = 0; k < OUTER_LOOP_COUNT; k++)
        {
            *e++ = cmd | fdt[k];
        }
        s += FromFmt::channel_count;
    }

#if !PIO_AUDIO_PWM_NO_INTERP_SAVE
    interp_restore(interp0, &saver);
#endif
}

template <typename FromFmt> void
    __no_inline_not_in_flash_func(encode_samples_dither)(int s_count, const typename FromFmt::sample_t *s, pwm_cmd_t *encoded)
{
    static_assert(DITHER_BITS > 0 && DITHER_BITS <= 3, "");
    const typename FromFmt::sample_t *s_end = s + s_count * FromFmt::channel_count;
    // hacky cast to allow us to DITHER_BITS > 1
    uint32_t *e = (uint32_t *) encoded;

#if PICO_AUDIO_PWM_INTERP_SAVE
    interp_hw_save_t saver;
    interp_save(interp0, &saver);
#endif

//    interp_configure_with_signed_and_cross_result(interp0, 0, FRACTIONAL_BITS, 0, QUANTIZED_MASK - 1);
    interp_config config = interp_default_config();
    interp_config_set_shift(&config, FRACTIONAL_BITS);
    interp_config_set_mask(&config, 0, QUANTIZED_BITS - 1);
    interp_config_set_signed(&config, true);
    interp_config_set_cross_result(&config, true);
    interp_set_config(interp0, 0, &config);
//    interp_configure_with_cross_input(interp0, 1, 0, 0, FRACTIONAL_BITS - 1);
    config = interp_default_config();
    interp_config_set_mask(&config, 0, FRACTIONAL_BITS - 1);
    interp_config_set_cross_input(&config, true);
    interp_set_config(interp0, 1, &config);

    interp0->base[0] = 0;

    int32_t last_sample_error = 0;
    static uint32_t saved_error = 0;

    // accum 0 is the error
    interp0->accum[0] = saved_error;

    while (s < s_end)
    {
        uint32_t sample = sample_converter<FmtU16, FromFmt>::convert_sample(*s);

        // we will be adding this sample error to accumulated error each time in the super sample loop
        uint32_t sample_error = sample & FRACTIONAL_MASK;
        interp0->base[1] = sample_error;

        // adjust the accumulated_error from (last_sample_error + error) to (sample_error + error)
        // because the error is one cycle ahead of the loop (i.e. we need to have added the sample error once before
        // the first loop)
        interp0->add_raw[0] = sample_error - last_sample_error;
        last_sample_error = sample_error;

        uint32_t quant0 =
                (sample >> FRACTIONAL_BITS) & QUANTIZED_MASK; // can't use interp here since accumulator has error in it
        assert(quant0 >= 0 && quant0 <= QUANTIZED_MAX);
        for(uint k = 0; k < OUTER_LOOP_COUNT; k++)
        {
            uint32_t cmd = MAKE_CMD(quant0);
            uint32_t bit = CMD_BITS + DITHER_BITS - 1;

            for(uint j = 0; j < CYCLES_PER_WORD; j++)
            {
                // accumulated_error (accum[0]) = previous_accumulated_error + sample_error - note this was done ahead of this iteration
                // quant (result[0]) = (accumulated_error) >> FRACTIONAL_BITS) & QUANTIZED_MASK;
                // accumulated_error (result[1]->accum[0]) = (previous_accumulated_error + sample_error) & FRACTIONAL_MASK;
                uint32_t quant = interp0->pop[0];
                // we can only dither 0 or +1
                assert(quant == 0 || quant == 1);
                if (!!quant)
                    cmd |= 1 << bit;
                bit += DITHER_BITS;
            }
            *e++ = cmd;
        }
        s += FromFmt::channel_count;
    }
    saved_error = interp0->accum[0] - last_sample_error;

#if PICO_AUDIO_PWM_INTERP_SAVE
    interp_restore(interp0, &saver);
#endif
}

#ifdef ENABLE_NOISE_SHAPING
static uint8_t shape_bits[4] = { 0b000, 0b100, 0b110, 0b111 };

template <typename FromFmt> void
    __no_inline_not_in_flash_func(encode_samples_noise_shaped_dither)(int s_count, const typename FromFmt::sample_t *s, pwm_cmd_t *encoded) {

    static_assert(DITHER_BITS <= 3, "");
    const typename FromFmt::sample_t *s_end = s + s_count * FromFmt::channel_count;
    // hacky cast to allow us to DITHER_BITS > 1
    uint32_t * e = (uint32_t *)encoded;
#if PICO_AUDIO_PWM_INTERP_SAVE
    interp_hw_save_t saver;
    interp_save(interp0, &saver);
#endif

    interp_configure_with_signed_and_cross_result(interp0, 0, FRACTIONAL_BITS, 0, QUANTIZED_BITS - 1);
    interp_configure_with_cross_input(interp0, 1, 0, 0, FRACTIONAL_BITS - 1);

    // we offset one to the quantized result because the error is between -1 and 2, giving us a range of 0 to 3
    // todo i keep being tempted to make this zero and remove the -1 offset from quant0 but this does not work
    //  so i need to comment why... I believe this corrects something we later double to be zero based not -1 based
    interp0->base[0] = 1;

    int32_t last_sample_error = 0;
    static uint32_t saved_error = 0, previous_accumulated_error = 0;
    interp0->accum[0] = saved_error;

    while (s < s_end)
    {
        uint32_t sample = sample_converter<Mono<FmtU16>, FromFmt>::convert_sample(s);

        // we will be adding this sample error to accumulated error each time in the super sample loop
        uint32_t sample_error = sample & FRACTIONAL_MASK;
        interp0->base[1] = sample_error;

        // adjust the accumulated_error from (last_sample_error + error) to (sample_error + error)
        // because the error is one cycle ahead of the loop (i.e. we need to have added the sample error once before
        // the first loop)
        interp0->add_raw[0] = sample_error - last_sample_error;
        last_sample_error = sample_error;

        uint32_t quant0 = ((sample >> FRACTIONAL_BITS) - 1u) & QUANTIZED_MASK; // can't use interp here since accumulator has error in it
        // todo clearly this could be a problem for too high a volume!
        assert(quant0 >= 0 && quant0 <= QUANTIZED_MAX);
        for(uint k=0; k < OUTER_LOOP_COUNT; k++) {
            uint32_t cmd = MAKE_CMD(quant0);
            uint base_bit = CMD_BITS;
            for (uint j = 0; j < CYCLES_PER_WORD; j++) {
                // accumulated_error (accum[0]) = previous_accumulated_error + sample_error - note this was done ahead of this iteration
                uint32_t accumulated_error = interp0->add_raw[1];
                // quant (result[0]) = 1 + (accumulated_error) >> FRACTIONAL_BITS) & QUANTIZED_MASK;
                // accumulated_error (result[1]->accum[0]) = (previous_accumulated_error + sample_error) & FRACTIONAL_MASK;
                uint32_t quant = interp0->pop[0];
                // accumulated_error += accumulated_error - previous_accumulated_error
                // i.e. accumulated_error = ((previous_accumulated_error + sample_error) & FRACTIONAL_MASK) * 2 - previous_accumulated_error
                // which is the noise shaping
                interp0->add_raw[0] = accumulated_error - previous_accumulated_error;
                previous_accumulated_error = accumulated_error;

                assert (quant >=0 && quant <= 3);
                cmd |= shape_bits[quant] << base_bit;
                base_bit += DITHER_BITS;
            }
            *e++ = cmd;
        }
        s += FromFmt::channel_count;
    }
    saved_error = interp0->accum[0] - last_sample_error;

#if PICO_AUDIO_PWM_INTERP_SAVE
    interp_restore(interp0, &saver);
#endif
}
#endif

// encoding converter
template<typename FromFmt> struct converting_copy<FmtPWM,FromFmt> {
    static void copy(typename FmtPWM::sample_t *dest, const typename FromFmt::sample_t *src, uint sample_count) {
        DEBUG_PINS_SET(encoding, 1);
        switch (audio_correction_mode)
        {
            case dither:
                encode_samples_dither<FromFmt>(sample_count, src, dest);
                break;
            case fixed_dither:
                encode_samples_fixed_dither<FromFmt>(sample_count, src, dest);
                break;
#ifdef ENABLE_NOISE_SHAPING
            case noise_shaped_dither:
                encode_samples_noise_shaped_dither<FromFmt>(sample_count, src, dest);
                break;
#endif
            default:
                encode_samples_none<FromFmt>(sample_count, src, dest);
                break;
        }
        DEBUG_PINS_CLR(encoding, 1);
    }
};

void producer_pool_blocking_give_to_pwm_s16(audio_connection_t *connection, audio_buffer_t *buffer)
{
    producer_pool_blocking_give<FmtPWM, FmtS16>(connection, buffer);
}

void producer_pool_blocking_give_to_pwm_s8(audio_connection_t *connection, audio_buffer_t *buffer)
{
    producer_pool_blocking_give<FmtPWM, FmtS8>(connection, buffer);
}

void producer_pool_blocking_give_to_pwm_u16(audio_connection_t *connection, audio_buffer_t *buffer)
{
    producer_pool_blocking_give<FmtPWM, FmtU16>(connection, buffer);
}

void producer_pool_blocking_give_to_pwm_u8(audio_connection_t *connection, audio_buffer_t *buffer)
{
    producer_pool_blocking_give<FmtPWM, FmtU8>(connection, buffer);
}