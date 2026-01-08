/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "video.h"

// todo support for inverted-y (probably belongs in the scanline generators, as would inverted x)
const uint32_t video_clock_freq = 24000000;

extern const video_pio_program_t video_24mhz_composable;
const video_timing_t vga_timing_640x480_60_default =
{
    .clock_freq = 24000000,

    .h_active = 640,
    .v_active = 480,

    .h_front_porch = 16,
    .h_pulse = 64,
    .h_total = 800,
    .h_sync_polarity = 1,

    .v_front_porch = 1,
    .v_pulse = 2,
    .v_total = 500,
    .v_sync_polarity = 1,

    .enable_clock = 0,
    .clock_polarity = 0,

    .enable_den = 0
};

const video_timing_t vga_timing_640x240_60_default =
        {
                .clock_freq = 24000000,

                .h_active = 640,
                .v_active = 240,

                .h_front_porch = 16,
                .h_pulse = 64,
                .h_total = 800,
                .h_sync_polarity = 1,

                .v_front_porch = 1,
                .v_pulse = 2,
                .v_total = 250,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };


const video_timing_t vga_timing_648x480_60_alt1 =
{

    .clock_freq = 24000000,

    .h_active = 640,
    .v_active = 480,

    .h_front_porch = 16,
    .h_pulse = 48,
    .h_total = 768,
    .h_sync_polarity = 1,

    .v_front_porch = 10,
    .v_pulse = 2,
    .v_total = 523,
    .v_sync_polarity = 1,

    .enable_clock = 0,
    .clock_polarity = 0,

    .enable_den = 0
};

const video_timing_t vga_timing_648x480_50ish =
        {

                .clock_freq = 24000000,

                .h_active = 640,
                .v_active = 480,

                .h_front_porch = 56,
                .h_pulse = 72,
                .h_total = 896,
                .h_sync_polarity = 1,

                .v_front_porch = 30,
                .v_pulse = 2,
                .v_total = 536,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const video_timing_t vga_timing_648x480_50ish2 =
        {

                .clock_freq = 24000000,

                .h_active = 640,
                .v_active = 480,

                .h_front_porch = 32,
                .h_pulse = 64,
                .h_total = 832,
                .h_sync_polarity = 1,

                .v_front_porch = 27,
                .v_pulse = 2,
                .v_total = 577,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const video_timing_t vga_timing_648x480_50ish3 =
        {

                .clock_freq = 24000000,

                .h_active = 640,
                .v_active = 480,

                .h_front_porch = 72,
                .h_pulse = 96,
                .h_total = 928,
                .h_sync_polarity = 1,

                .v_front_porch = 8,
                .v_pulse = 2,
                .v_total = 518,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

#define actual_vga_timing_50 vga_timing_648x480_50ish3

const video_mode_t vga_mode_160x120_60 =
{
    .default_timing = &vga_timing_640x480_60_default,
    .pio_program = &video_24mhz_composable,
    .width = 160,
    .height = 120,
    .xscale = 4,
    .yscale = 4,
};

const video_mode_t vga_mode_213x160_60 =
{
    .default_timing = &vga_timing_640x480_60_default,
    .pio_program = &video_24mhz_composable,
    .width = 213,
    .height = 160,
    .xscale = 3,
    .yscale = 3,
};

const video_mode_t vga_mode_320x240_60 =
{
    .default_timing = &vga_timing_640x240_60_default,
    .pio_program = &video_24mhz_composable,
    .width = 320,
    .height = 240,
    .xscale = 2,
    .yscale = 1,
};

const video_mode_t vga_mode_320x480_60 =
        {
                .default_timing = &vga_timing_640x480_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 320,
                .height = 480,
                .xscale = 2,
                .yscale = 2,
        };


const video_mode_t vga_mode_640x480_60 =
{
    .default_timing = &vga_timing_640x480_60_default,
    .pio_program = &video_24mhz_composable,
    .width = 640,
    .height = 480,
    .xscale = 1,
    .yscale = 1,
};


const video_mode_t vga_mode_640x480_50 =
{
    .default_timing = &actual_vga_timing_50,
    .pio_program = &video_24mhz_composable,
    .width = 640,
    .height = 480,
    .xscale = 1,
    .yscale = 1,
};

const video_mode_t vga_mode_320x240_50 =
{
    .default_timing = &actual_vga_timing_50,
    .pio_program = &video_24mhz_composable,
    .width = 320,
    .height = 240,
    .xscale = 2,
    .yscale = 2,
};

/* this is 50 hz */
const video_timing_t vga_timing_wide_480_50 =
{
    .clock_freq = 24000000,

    .h_active = 800,
    .v_active = 480,

    .h_front_porch = 32,
    .h_pulse = 48,
    .h_total = 960,
    .h_sync_polarity = 0,

    .v_front_porch = 1,
    .v_pulse = 2,
    .v_total = 500,
    .v_sync_polarity = 0,

    .enable_clock = 1,
    .clock_polarity = 0,

    .enable_den = 1
};

const video_mode_t vga_mode_tft_800x480_50 =
{
    .default_timing = &vga_timing_wide_480_50,
    .pio_program = &video_24mhz_composable,
    .width = 800,
    .height = 480,
    .xscale = 1,
    .yscale = 1,
};

const video_mode_t vga_mode_tft_400x240_50 =
{
    .default_timing = &vga_timing_wide_480_50,
    .pio_program = &video_24mhz_composable,
    .width = 400,
    .height = 240,
    .xscale = 2,
    .yscale = 2,
};

const video_timing_t vga_timing_512x576_50_attempt1 =
        {
                .clock_freq = 24000000,

                .h_active = 512,
                .v_active = 576,

                .h_front_porch = 64,
                .h_pulse = 64,
                .h_total = 768,
                .h_sync_polarity = 1,

                .v_front_porch = 30,
                .v_pulse = 2,
                .v_total = 612,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const video_timing_t vga_timing_512x576_60_attempt1 =
        {
                .clock_freq = 24000000,

                .h_active = 512,
                .v_active = 576,

                .h_front_porch = 64,
                .h_pulse = 64,
                .h_total = 768,
                .h_sync_polarity = 1,

                .v_front_porch = 30,
                .v_pulse = 2,
                .v_total = 612,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const video_mode_t vga_mode_256x192_50 =
        {
                .default_timing = &vga_timing_512x576_50_attempt1,
                .pio_program = &video_24mhz_composable,
                .width = 256,
                .height = 192,
                .xscale = 2,
                .yscale = 3,
        };

const video_timing_t vga_timing_800x600_38 =
        {
                .clock_freq = 24000000,

                .h_active = 800,
                .v_active = 600,

                .h_front_porch = 24,
                .h_pulse = 80,
                .h_total = 1008,
                .h_sync_polarity = 1,

                .v_front_porch = 3,
                .v_pulse = 4,
                .v_total = 621,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const video_mode_t vga_mode_800x600_38 =
        {
                .default_timing = &vga_timing_800x600_38,
                .pio_program = &video_24mhz_composable,
                .width = 800,
                .height = 600,
                .xscale = 1,
                .yscale = 1,
        };

