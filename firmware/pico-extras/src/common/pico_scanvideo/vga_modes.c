/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/scanvideo.h"

// todo support for inverted-y (probably belongs in the scanline generators, as would inverted x)

#if PICO_SCANVIDEO_48MHZ
const scanvideo_timing_t vga_timing_640x480_60_default =
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

const scanvideo_timing_t vga_timing_800x600_54_default =
        {
                .clock_freq = 24000000, // wrong, but ok for now

                .h_active = 800,
                .v_active = 600,

                .h_front_porch = 3 * 8,
                .h_pulse = 10 * 8,
                .h_total = 126 * 8,
                .h_sync_polarity = 0,

                .v_front_porch = 1,
                .v_pulse = 3,
                .v_total = 619,
                .v_sync_polarity = 0,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const scanvideo_timing_t vga_timing_800x600_60_default =
        {
                .clock_freq = 38400000,

                .h_active = 800,
                .v_active = 600,

                .h_front_porch = 4 * 8,
                .h_pulse = 10 * 8,
                .h_total = 128 * 8,
                .h_sync_polarity = 0,

                .v_front_porch = 1,
                .v_pulse = 3,
                .v_total = 625,
                .v_sync_polarity = 0,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const scanvideo_timing_t vga_timing_1024x768_63_default =
        {
                .clock_freq = 24000000, // wrong, but ok for now

                .h_active = 1024,
                .v_active = 768,

                .h_front_porch = 7 * 8,
                .h_pulse = 13 * 8,
                .h_total = 168 * 8,
                .h_sync_polarity = 0,

                .v_front_porch = 1,
                .v_pulse = 3,
                .v_total = 797,
                .v_sync_polarity = 0,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const scanvideo_timing_t vga_timing_1280x1024_40_default =
        {
                .clock_freq = 24000000, // wrong, but ok for now

                .h_active = 1280,
                .v_active = 1024,

                .h_front_porch = 7 * 8,
                .h_pulse = 16 * 8,
                .h_total = 206 * 8,
                .h_sync_polarity = 0,

                .v_front_porch = 1,
                .v_pulse = 3,
                .v_total = 1048,
                .v_sync_polarity = 0,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const scanvideo_timing_t vga_timing_648x480_60_alt1 =
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

const scanvideo_timing_t vga_timing_648x480_50ish =
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

const scanvideo_timing_t vga_timing_648x480_50ish2 =
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

const scanvideo_timing_t vga_timing_648x480_50ish3 =
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

/* this is 50 hz */
const scanvideo_timing_t vga_timing_wide_480_50 =
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

const scanvideo_mode_t vga_mode_tft_800x480_50 =
        {
                .default_timing = &vga_timing_wide_480_50,
                .pio_program = &video_24mhz_composable,
                .width = 800,
                .height = 480,
                .xscale = 1,
                .yscale = 1,
        };

const scanvideo_mode_t vga_mode_tft_400x240_50 =
        {
                .default_timing = &vga_timing_wide_480_50,
                .pio_program = &video_24mhz_composable,
                .width = 400,
                .height = 240,
                .xscale = 2,
                .yscale = 2,
        };

const scanvideo_timing_t vga_timing_512x576_50_attempt1 =
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

const scanvideo_timing_t vga_timing_512x576_60_attempt1 =
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

const scanvideo_mode_t vga_mode_256x192_50 =
        {
                .default_timing = &vga_timing_512x576_50_attempt1,
                .pio_program = &video_24mhz_composable,
                .width = 256,
                .height = 192,
                .xscale = 2,
                .yscale = 3,
        };

const scanvideo_timing_t vga_timing_800x600_38 =
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

const scanvideo_mode_t vga_mode_800x600_38 =
        {
                .default_timing = &vga_timing_800x600_38,
                .pio_program = &video_24mhz_composable,
                .width = 800,
                .height = 600,
                .xscale = 1,
                .yscale = 1,
        };

const scanvideo_mode_t vga_mode_800x600_54 =
        {
                .default_timing = &vga_timing_800x600_54_default,
                .pio_program = &video_24mhz_composable,
                .width = 800,
                .height = 600,
                .xscale = 1,
                .yscale = 1,
        };

const scanvideo_mode_t vga_mode_800x600_60 =
        {
                .default_timing = &vga_timing_800x600_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 800,
                .height = 600,
                .xscale = 1,
                .yscale = 1,
        };


const scanvideo_mode_t vga_mode_1024x768_63 =
        {
                .default_timing = &vga_timing_1024x768_63_default,
                .pio_program = &video_24mhz_composable,
                .width = 1024,
                .height = 768,
                .xscale = 1,
                .yscale = 1,
        };

const scanvideo_mode_t vga_mode_1280x1024_40 =
        {
                .default_timing = &vga_timing_1280x1024_40_default,
                .pio_program = &video_24mhz_composable,
                .width = 1280,
                .height = 1024,
                .xscale = 1,
                .yscale = 1,
        };


const scanvideo_mode_t vga_mode_640x480_50 =
        {
                .default_timing = &actual_vga_timing_50,
                .pio_program = &video_24mhz_composable,
                .width = 640,
                .height = 480,
                .xscale = 1,
                .yscale = 1,
        };

const scanvideo_mode_t vga_mode_320x240_50 =
        {
                .default_timing = &actual_vga_timing_50,
                .pio_program = &video_24mhz_composable,
                .width = 320,
                .height = 240,
                .xscale = 2,
                .yscale = 2,
        };
#else
const scanvideo_timing_t vga_timing_640x480_60_default =
        {
                .clock_freq = 25000000,

                .h_active = 640,
                .v_active = 480,

                .h_front_porch = 16,
                .h_pulse = 64,
                .h_total = 800,
                .h_sync_polarity = 1,

                .v_front_porch = 1,
                .v_pulse = 2,
                .v_total = 523,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };
#endif

const scanvideo_mode_t vga_mode_160x120_60 =
        {
                .default_timing = &vga_timing_640x480_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 160,
                .height = 120,
                .xscale = 4,
                .yscale = 4,
        };

const scanvideo_mode_t vga_mode_213x160_60 =
        {
                .default_timing = &vga_timing_640x480_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 213,
                .height = 160,
                .xscale = 3,
                .yscale = 3,
        };

const scanvideo_mode_t vga_mode_320x240_60 =
        {
                .default_timing = &vga_timing_640x480_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 320,
                .height = 240,
                .xscale = 2,
                .yscale = 2,
        };

const scanvideo_mode_t vga_mode_640x480_60 =
        {
                .default_timing = &vga_timing_640x480_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 640,
                .height = 480,
                .xscale = 1,
                .yscale = 1,
        };

/* Requires 130Mhz system clock, but standard XGA mode */
const scanvideo_timing_t vga_timing_1024x768_60_default =
        {
                .clock_freq = 65000000,

                .h_active = 1024,
                .v_active = 768,

                .h_front_porch = 24,
                .h_pulse = 136,
                .h_total = 1344,
                .h_sync_polarity = 0,

                .v_front_porch = 3,
                .v_pulse = 6,
                .v_total = 806,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const scanvideo_mode_t vga_mode_1024x768_60 =
        {
                .default_timing = &vga_timing_1024x768_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 1024,
                .height = 768,
                .xscale = 1,
                .yscale = 1,
        };

const scanvideo_timing_t vga_timing_1280x720_60_default =
        {
                .clock_freq = 74250000,

                .h_active = 1280,
                .v_active = 720,

                .h_front_porch = 110,
                .h_pulse = 40,
                .h_total = 1650,
                .h_sync_polarity = 1,

                .v_front_porch = 5,
                .v_pulse = 5,
                .v_total = 750,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };


const scanvideo_mode_t vga_mode_720p_60 =
        {
                .default_timing = &vga_timing_1280x720_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 1280,
                .height = 720,
                .xscale = 1,
                .yscale = 1,
        };

const scanvideo_timing_t vga_timing_1920x1080_60_default =
        {
                .clock_freq = 148500000,

                .h_active = 1920,
                .v_active = 1080,

                .h_front_porch = 88,
                .h_pulse = 44,
                .h_total = 2200,
                .h_sync_polarity = 1,

                .v_front_porch = 4,
                .v_pulse = 5,
                .v_total = 1125,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };


const scanvideo_mode_t vga_mode_1080p_60 =
        {
                .default_timing = &vga_timing_1920x1080_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 1920,
                .height = 1080,
                .xscale = 1,
                .yscale = 1,
        };

const scanvideo_timing_t vga_timing_1280x1024_60_default =
        {
                .clock_freq = 108000000,

                .h_active = 1280,
                .v_active = 1024,

                .h_front_porch = 48,
                .h_pulse = 112,
                .h_total = 1688,
                .h_sync_polarity = 0,

                .v_front_porch = 1,
                .v_pulse = 3,
                .v_total = 1066,
                .v_sync_polarity = 0,
};

const scanvideo_mode_t vga_mode_1280x1024_60 =
        {
                .default_timing = &vga_timing_1920x1080_60_default,
                .pio_program = &video_24mhz_composable,
                .width = 1280,
                .height = 1024,
                .xscale = 1,
                .yscale = 1,
        };
