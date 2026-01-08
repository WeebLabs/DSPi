/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SCANVIDEO_COMPOSABLE_SCANLINE_H_
#define SCANVIDEO_COMPOSABLE_SCANLINE_H_

#include "pico/types.h"
#include "scanvideo.pio.h"

// PICO_CONFIG: PICO_SCANVIDEO_USE_RAW1P_2CYCLE, Enable/disable SVideo use raw  1P 2 cycle, type=bool, default=0, group=video+-
#ifndef PICO_SCANVIDEO_USE_RAW1P_2CYCLE
#define PICO_SCANVIDEO_USE_RAW1P_2CYCLE 0
#endif

#if !PICO_SCANVIDEO_USE_RAW1P_2CYCLE
#define video_24mhz_composable_prefix video_24mhz_composable_default
#else
#define video_24mhz_composable_prefix video_24mhz_composable_raw1p_2cycle
#endif

// seems needed on some platforms
#define __EXTRA_CONCAT(x, y) __CONCAT(x,y)
#define video_24mhz_composable_program_extern(x) __EXTRA_CONCAT( __EXTRA_CONCAT(video_24mhz_composable_prefix, _offset_), x)
#define __DVP_JMP(x) ((unsigned)video_24mhz_composable_program_extern(x))
#define COMPOSABLE_COLOR_RUN __DVP_JMP(color_run)
#define COMPOSABLE_EOL_ALIGN __DVP_JMP(end_of_scanline_ALIGN)
#define COMPOSABLE_EOL_SKIP_ALIGN __DVP_JMP(end_of_scanline_skip_ALIGN)
#define COMPOSABLE_RAW_RUN __DVP_JMP(raw_run)
#define COMPOSABLE_RAW_1P __DVP_JMP(raw_1p)
#define COMPOSABLE_RAW_2P __DVP_JMP(raw_2p)
#if !PICO_SCANVIDEO_USE_RAW1P_2CYCLE
#define COMPOSABLE_RAW_1P_SKIP_ALIGN __DVP_JMP(raw_1p_skip_ALIGN)
#else
#define COMPOSABLE_RAW_1P_2CYCLE __DVP_JMP(raw_1p_2cycle)
#endif

#endif
