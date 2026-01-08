/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef _VIDEO_H
#define _VIDEO_H

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif
    
// == CONFIG ============
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA 1
#endif
#if PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA) || PICO_SCANVIDEO_PLANE2_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA 1
#endif
#if PICO_SCANVIDEO_PLANE3_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE3_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE3_FRAGMENT_DMA 1
#endif

#define ENABLE_VIDEO_CLOCK
#define ENABLE_VIDEO_DEN
// todo make multi plane play nicely with mode swapping;
//  today we have hard coded blank/empty lines

//#define ENABLE_VIDEO_PLANE2
//#define ENABLE_VIDEO_PLANE3
//#define PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA 1
//#define PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA 1

#ifndef PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS
#define PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS 128
#endif
#ifndef PICO_SCANVIDEO_MAX_SCANLINE_BUFFER2_WORDS
#define PICO_SCANVIDEO_MAX_SCANLINE_BUFFER2_WORDS 16
#endif
#ifndef PICO_SCANVIDEO_MAX_SCANLINE_BUFFER3_WORDS
#define PICO_SCANVIDEO_MAX_SCANLINE_BUFFER3_WORDS 16
#endif

//extern struct semaphore vmode_updated;

// note by default we allow for alpha mask (and lose a bit of green)
// todo make this configurable
#define PICO_SCANVIDEO_ALPHA_MASK 0x0020
#define PICO_SCANVIDEO_PIXEL_RSHIFT 0
#define PICO_SCANVIDEO_PIXEL_GSHIFT 6
#define PICO_SCANVIDEO_PIXEL_BSHIFT 11
#define PICO_SCANVIDEO_PIXEL_FROM_RGB8(r, g, b) ((((b)>>3)<<PICO_SCANVIDEO_PIXEL_BSHIFT)|(((g)>>3)<<PICO_SCANVIDEO_PIXEL_GSHIFT)|(((r)>>3)<<PICO_SCANVIDEO_PIXEL_RSHIFT))
#define PICO_SCANVIDEO_PIXEL_FROM_RGB5(r, g, b) (((b)<<PICO_SCANVIDEO_PIXEL_BSHIFT)|((g)<<PICO_SCANVIDEO_PIXEL_GSHIFT)|((r)<<PICO_SCANVIDEO_PIXEL_RSHIFT))

// ======================

#define BPP 16

// most likely 24000000
extern const uint32_t vga_clock_freq;

// todo pragma pack?
struct video_timing
{
    uint32_t clock_freq;

    uint16_t h_active;
    uint16_t v_active;

    uint16_t h_front_porch;
    uint16_t h_pulse;
    uint16_t h_total;
    uint8_t h_sync_polarity;

    uint16_t v_front_porch;
    uint16_t v_pulse;
    uint16_t v_total;
    uint8_t v_sync_polarity;

    uint8_t enable_clock;
    uint8_t clock_polarity;

    uint8_t enable_den;
};

struct pio_program;

// todo we need to handle blank data correctly (perhaps DMA should just not start for that scanline,
//  though obviously this is slightly more complicated with multiple playfields, or perhaps worse with
//  just one
struct video_mode
{
    const struct video_timing *default_timing;
    const struct video_pio_program *pio_program;

    uint16_t width;
    uint16_t height;
    uint8_t xscale; // 1 == normal, 2 == double wide etc. up to what pio timing allows (not sure I have an assert based on delays)
    uint8_t yscale; // same for y scale (except any yscale is possible)
};

extern bool video_setup(const struct video_mode *mode);
extern bool video_setup_with_timing(const struct video_mode *mode, const struct video_timing *timing);
extern void video_timing_enable(bool enable);
// these take effect after the next vsync
extern void video_display_enable(bool enable);
// doesn't exist yet!
// extern void video_set_display_mode(const struct video_mode *mode);

// --- scanline management ---

struct scanline_buffer
{
    uint32_t scanline_id;
    uint32_t *data;
    uint16_t data_used;
    uint16_t data_max;
#if PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA
    uint16_t fragment_words;
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 1
    uint32_t *data2;
    uint16_t data2_used;
    uint16_t data2_max;
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    uint32_t *data3;
    uint16_t data3_used;
    uint16_t data3_max;
#endif
#endif
    // useful to track state between the buffer being passed to
    // video_end_scanline_generation, and when the buffer is no longer
    // in use by the video code and is returned to a subsequent caller
    // via video_begin_scanline_generation
    // todo we caould add a callback to begin scanline generation to enuerate
    //  all already discarded buffers early - not clear this would be useful in general
    //  because it only saves you space if stuff is running with low buffer utilization
    void *user_data;
    uint8_t status;
};

enum
{
    SCANLINE_OK = 1,
    SCANLINE_ERROR,
    SCANLINE_SKIPPED
};

// note frame numbers wrap
static inline uint16_t frame_number(uint32_t scanline_id)
{
    return (uint16_t) (scanline_id >> 16u);
}

static inline uint16_t scanline_number(uint32_t scanline_id)
{
    return (uint16_t) scanline_id;
}

/**
 * @return the current vga mode (if there is one)
 */
extern struct video_mode video_get_mode();

/**
 * @return the next scanline_id to be displayed (may be from the next frame)
 */
extern uint32_t video_get_next_scanline_id();

/**
 * @return true if in the vblank interval
 */
extern bool video_in_vblank();
/**
 * @return true if in the hblank interval (or more accurately scanline data is not currently being sent to the PIO, which roughly corresponds, but is not exact). Note also that in
 * yscale-d modes, there are height * yscale hblank intervals per frame.
 */
extern bool video_in_hblank();

extern void video_wait_for_vblank();

extern uint32_t video_wait_for_scanline_complete(uint32_t scanline_id);
/**
 * Acquire a scanline that needs generating. The scanline_id field indicates which scanline is required.
 *
 * This method may be called concurrently
 *
 * @param block true to block if the vga system is not ready to generate a new scanline
 * @return the scanline_buffer or NULL if block is false, and the vga system is not ready
 */
struct scanline_buffer *video_begin_scanline_generation(bool block);

/**
 * Return a scanline that has been generated / or at least the client is done with.
 *
 * The status field indicates whether the scanline was actually generated OK
 *
 * This method may be called concurrently (for different buffers)
 *
 * @param scanline_buffer \todo
 */
void video_end_scanline_generation(struct scanline_buffer *scanline_buffer);

extern const struct video_timing vga_timing_640x480_60_default;
extern const struct video_timing vga_timing_wide_480_50;
extern const struct video_timing vga_timing_648x480_60_alt1;

extern const struct video_mode vga_mode_160x120_60; // 3d monster maze anyone :-)
extern const struct video_mode vga_mode_213x160_60;
extern const struct video_mode vga_mode_320x240_60;
extern const struct video_mode vga_mode_640x480_60;
extern const struct video_mode vga_mode_320x480_60;

extern const struct video_mode vga_mode_tft_800x480_50;
extern const struct video_mode vga_mode_tft_400x240_50;

#ifndef NDEBUG
// todo this is only for vga composable 24... should exist behind mode impl
extern void validate_scanline(const uint32_t *dma_data, uint dma_data_size, uint max_pixels, uint expected_width);
#endif


// mode implementation

pio_hw_t;

struct video_pio_program
{
#if !PICO_NO_HARDWARE
    const uint16_t *program;
    const int program_size;
    const int entry_point;
    bool (*adapt_for_mode)(const struct video_pio_program *program, const struct video_mode *mode,
                           struct scanline_buffer *missing_scanline_buffer, uint16_t *buffer, uint buffer_max);
    void (*configure_pio)(pio_hw_t *pio, uint sm);
#else
    const char *id;
#endif
};

extern void video_default_configure_pio(pio_hw_t *pio, uint sm, uint wrap_trarget, uint wrap, bool overlay);

#include "video.pio.h"
#if !PICO_SCANVIDEO_USE_RAW1P_2CYCLE
#define video_24mhz_composable_prefix video_24mhz_composable_default
#else
#define video_24mhz_composable_prefix video_24mhz_composable_raw1p_2cycle
#endif
// yuk... extra __P needed for native on some platforms
#define video_24mhz_composable_program_extern(x) __SAFE_CONCAT(__SAFE_CONCAT(video_24mhz_composable_prefix, _offset_), x)
#define __DVP_JMP(x) ((unsigned)video_24mhz_composable_program_extern(x))
#define COMPOSABLE_COLOR_RUN __DVP_JMP(color_run)
#define COMPOSABLE_EOL_ALIGN __DVP_JMP(end_of_scanline_ALIGN)
#define COMPOSABLE_EOL_SKIP_ALIGN __DVP_JMP(end_of_scanline_skip_word_ALIGN)
#define COMPOSABLE_RAW_RUN __DVP_JMP(raw_run)
#define COMPOSABLE_RAW_1P __DVP_JMP(raw_1p)
#define COMPOSABLE_RAW_2P __DVP_JMP(raw_2p)
#if !PICO_SCANVIDEO_USE_RAW1P_2CYCLE
#define COMPOSABLE_RAW_1P_SKIP_ALIGN __DVP_JMP(raw_1p_skip_word_ALIGN)
#else
#define COMPOSABLE_RAW_1P_2CYCLE __DVP_JMP(raw_1p_2cycle)
#endif

#ifdef __cplusplus
}
#endif
#endif //_VIDEO_H
