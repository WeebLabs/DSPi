/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>
#include <iostream>
#include <sstream>
#include "pico/stdlib.h"
#include "pico/bit_ops.h"
#include "pico/sample_conversion.h"

typedef int (*sample_converter_fn)(int);

int u16_to_u16(int s) { return (uint16_t) s; }

int s16_to_u16(int s) { return (uint16_t) (s ^ 0x8000u); }

int u8_to_u16(int s) { return (uint16_t) (s << 8); }

int s8_to_u16(int s) { return (uint16_t) ((s << 8u) ^ 0x8000u); }

int u16_to_s16(int s) { return (int16_t) (s ^ 0x8000u); }

int s16_to_s16(int s) { return (int16_t) s; }

int u8_to_s16(int s) { return (int16_t) ((s << 8u) ^ 0x8000u); }

int s8_to_s16(int s) { return (int16_t) (s << 8u); }

int u16_to_u8(int s) { return (uint8_t) (s >> 8u); }

int s16_to_u8(int s) { return (uint8_t) ((s ^ 0x8000u) >> 8u); }

int u8_to_u8(int s) { return (uint8_t) s; }

int s8_to_u8(int s) { return (uint8_t) (s ^ 0x80); }

int u16_to_s8(int s) { return (int8_t) ((s ^ 0x8000u) >> 8u); }

int s16_to_s8(int s) { return (int8_t) (s >> 8u); }

int u8_to_s8(int s) { return (int8_t) (s ^ 0x80); }

int s8_to_s8(int s) { return (int8_t) s; }

template<typename Fmt>
typename Fmt::sample_t random_sample() {
    return (typename Fmt::sample_t) rand();
}

void check_sample(int from, int expected, int actual) {
    if (expected != actual) {
        printf("Failed converting %04x to %04x (got %04x)\n", from, expected, actual);
        assert(false);
    }
}

template<typename ToFmt, typename FromFmt>
void check_conversion(sample_converter_fn converter_fn) {
    uint length = 256 + rand() & 0xffu;
    typename ToFmt::sample_t to_buffer[length * ToFmt::channel_count];
    typename FromFmt::sample_t from_buffer[length * FromFmt::channel_count];
    for (uint i = 0; i < length * FromFmt::channel_count; i++) {
        from_buffer[i] = random_sample<FromFmt>();
    }
    converting_copy<ToFmt, FromFmt>::copy(to_buffer, from_buffer, length);
    if (ToFmt::channel_count == FromFmt::channel_count) {
        for (uint i = 0; i < length * ToFmt::channel_count; i++) {
            check_sample(from_buffer[i], converter_fn(from_buffer[i]), to_buffer[i]);
        }
    } else if (ToFmt::channel_count == 2 & FromFmt::channel_count == 1) {
        // mono -> stereo duplicates
        for (uint i = 0; i < length; i++) {
            check_sample(from_buffer[i], converter_fn(from_buffer[i]), to_buffer[i * 2]);
            check_sample(from_buffer[i], converter_fn(from_buffer[i]), to_buffer[i * 2 + 1]);
        }
    } else if (ToFmt::channel_count == 1 & FromFmt::channel_count == 2) {
        // stereo -> mono averages
        for (uint i = 0; i < length; i++) {
            // can't represent both samples
            check_sample(0xf00d, converter_fn((from_buffer[i * 2] + from_buffer[i * 2 + 1]) / 2), to_buffer[i]);
        }
    } else {
        assert(false);
    }
}

template<class ToFmt, class FromFmt>
void check_conversions(sample_converter_fn converter_fn) {
    // for a given format check conversions to and from
    check_conversion<Mono<ToFmt>, Mono<FromFmt>>(converter_fn);
    check_conversion<Stereo<ToFmt>, Mono<FromFmt>>(converter_fn);
    check_conversion<Mono<ToFmt>, Stereo<FromFmt>>(converter_fn);
    check_conversion<Stereo<ToFmt>, Stereo<FromFmt>>(converter_fn);
}

int main() {
    // On FPGA, pins 28 and 29 are connected to the VC707 board USB-UART
    uart_init(uart0, 115200);
    gpio_set_function(28, GPIO_FUNC_UART);
    gpio_set_function(29, GPIO_FUNC_UART);

    // check all permutations of supported formats

    check_conversions<FmtU16, FmtU16>(u16_to_u16);
    check_conversions<FmtS16, FmtU16>(u16_to_s16);
    check_conversions<FmtU8, FmtU16>(u16_to_u8);
    check_conversions<FmtS8, FmtU16>(u16_to_s8);

    check_conversions<FmtU16, FmtS16>(s16_to_u16);
    check_conversions<FmtS16, FmtS16>(s16_to_s16);
    check_conversions<FmtU8, FmtS16>(s16_to_u8);
    check_conversions<FmtS8, FmtS16>(s16_to_s8);

    check_conversions<FmtU16, FmtU8>(u8_to_u16);
    check_conversions<FmtS16, FmtU8>(u8_to_s16);
    check_conversions<FmtU8, FmtU8>(u8_to_u8);
    check_conversions<FmtS8, FmtU8>(u8_to_s8);

    check_conversions<FmtU16, FmtS8>(s8_to_u16);
    check_conversions<FmtS16, FmtS8>(s8_to_s16);
    check_conversions<FmtU8, FmtS8>(s8_to_u8);
    check_conversions<FmtS8, FmtS8>(s8_to_s8);

    printf("OK\n");
}

