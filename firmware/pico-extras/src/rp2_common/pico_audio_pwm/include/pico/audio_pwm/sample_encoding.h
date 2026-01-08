/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_PWM_SAMPLE_ENCODING_H
#define _PICO_AUDIO_PWM_SAMPLE_ENCODING_H

#ifdef __cplusplus
extern "C" {
#endif
// todo some if not all of this can go in sample_encoding.cpp

#define FRACTIONAL_BITS 9u
#define QUANTIZED_BITS 7u

static_assert(FRACTIONAL_BITS + QUANTIZED_BITS == 16, "");

#ifndef ENABLE_NOISE_SHAPING
const uint32_t audio_carrier_freq = 350364;
#define program_name pwm_one_bit_dither
#define NATIVE_BUFFER_FORMAT AUDIO_BUFFER_FORMAT_PIO_PWM_CMD1
#else
#define program_name pwm_two_bit_dither
#define NATIVE_BUFFER_FORMAT AUDIO_BUFFER_FORMAT_PIO_PWM_CMD3
#endif

static_assert(QUANTIZED_BITS == 7, ""); // required by make_cmd below
#define MAKE_CMD(q) (((q)) | (127u - (q)) << 7u)
#define CMD_BITS (QUANTIZED_BITS * 2)
#define SILENCE_LEVEL 0x40u
#define SILENCE_CMD MAKE_CMD(SILENCE_LEVEL)

#ifdef ENABLE_NOISE_SHAPING
#define DITHER_BITS 3u
// this needs to be divisible by dither bits
#define CYCLES_PER_SAMPLE 15
typedef struct {
    uint32_t a;
    uint32_t b;
    uint32_t c;
#ifdef AUDIO_HALF_FREQ
    uint32_t d, e, f;
#endif
} pwm_cmd_t; // what we send to PIO for each sample
const pwm_cmd_t silence_cmd = {SILENCE_CMD, SILENCE_CMD, SILENCE_CMD,
#ifdef AUDIO_HALF_FREQ
        SILENCE_CMD, SILENCE_CMD, SILENCE_CMD,
#endif
};
#else
#define CYCLES_PER_SAMPLE 16
#ifndef AUDIO_HALF_FREQ
typedef uint32_t pwm_cmd_t; // what we send to PIO for each sample
const pwm_cmd_t silence_cmd = SILENCE_CMD;
#else
typedef struct {
    uint32_t a;
    uint32_t b;
} pwm_cmd_t; // what we send to PIO for each sample
const pwm_cmd_t silence_cmd = { SILENCE_CMD, SILENCE_CMD };
#endif
#define DITHER_BITS 1u
#endif

static_assert(CYCLES_PER_SAMPLE % DITHER_BITS == 0, "");
#define CYCLES_PER_WORD (CYCLES_PER_SAMPLE / DITHER_BITS)
#ifndef AUDIO_HALF_FREQ
#define OUTER_LOOP_COUNT DITHER_BITS
#else
#define OUTER_LOOP_COUNT DITHER_BITS*2
#endif
#define FRACTIONAL_LSB 0u
#define FRACTIONAL_MSB (FRACTIONAL_LSB + FRACTIONAL_BITS - 1u)
#define FRACTIONAL_MASK ((1u << FRACTIONAL_BITS) - 1u)
#define QUANTIZED_LSB FRACTION_BITS
#define QUANTIZED_MSB (QUANTIZED_LSB + QUANTIZED_BITS - 1u)
#define QUANTIZED_MAX ((1u << QUANTIZED_BITS) - 1u)
#define QUANTIZED_MASK QUANTIZED_MAX


void producer_pool_blocking_give_to_pwm_s16(audio_connection_t *connection, audio_buffer_t *buffer);
void producer_pool_blocking_give_to_pwm_s8(audio_connection_t *connection, audio_buffer_t *buffer);
void producer_pool_blocking_give_to_pwm_u16(audio_connection_t *connection, audio_buffer_t *buffer);
void producer_pool_blocking_give_to_pwm_u8(audio_connection_t *connection, audio_buffer_t *buffer);

#ifdef __cplusplus
}
#endif

#endif //SOFTWARE_SAMPLE_ENCODING_H
