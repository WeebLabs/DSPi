/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SCANVIDEO_scanvideo_H_
#define SCANVIDEO_scanvideo_H_

#include "pico/types.h"

#if !PICO_NO_HARDWARE

#include "hardware/pio.h"

#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \file scanvideo_base.h
 *  \defgroup pico_scanvideo pico_scanvideo
 *
 * Common Scan-out Video API
 */
// == CONFIG ============
#ifndef PICO_SCANVIDEO_PLANE_COUNT
#define PICO_SCANVIDEO_PLANE_COUNT 1
#endif

#ifndef PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT
#define PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT 8
#endif

#ifndef PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS
#define PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS 0
#endif

#ifndef PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA 0
#endif

#ifndef PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA 0
#endif

#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA 1
#endif

#ifndef PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA 0
#endif

#ifndef PICO_SCANVIDEO_PLANE2_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE2_FIXED_FRAGMENT_DMA 0
#endif

#if PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE2_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA 1
#endif

#ifndef PICO_SCANVIDEO_PLANE3_VARIABLE_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE3_VARIABLE_FRAGMENT_DMA 0
#endif

#ifndef PICO_SCANVIDEO_PLANE3_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE3_FIXED_FRAGMENT_DMA 0
#endif

#if PICO_SCANVIDEO_PLANE3_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE3_FIXED_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE3_FRAGMENT_DMA 1
#endif

#ifndef PICO_SCANVIDEO_ENABLE_CLOCK_PIN
#define PICO_SCANVIDEO_ENABLE_CLOCK_PIN 0
#endif

#ifndef PICO_SCANVIDEO_ENABLE_DEN_PIN
#define PICO_SCANVIDEO_ENABLE_DEN_PIN 0
#endif

#ifndef PICO_SCANVIDEO_COLOR_PIN_BASE
#define PICO_SCANVIDEO_COLOR_PIN_BASE 0
#endif

#ifndef PICO_SCANVIDEO_COLOR_PIN_COUNT
#define PICO_SCANVIDEO_COLOR_PIN_COUNT 16
#endif

#ifndef PICO_SCANVIDEO_SYNC_PIN_BASE
#define PICO_SCANVIDEO_SYNC_PIN_BASE (PICO_SCANVIDEO_COLOR_PIN_BASE + PICO_SCANVIDEO_COLOR_PIN_COUNT)
#endif

#ifndef PICO_SCANVIDEO_ENABLE_VIDEO_CLOCK_DOWN
#define PICO_SCANVIDEO_ENABLE_VIDEO_CLOCK_DOWN 0
#endif

// todo make multi plane play nicely with mode swapping;
//  today we have hard coded blank/empty lines

//#define PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA 1
//#define PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA 1

#ifndef PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS
#define PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS 180
#endif
#ifndef PICO_SCANVIDEO_MAX_SCANLINE_BUFFER2_WORDS
#define PICO_SCANVIDEO_MAX_SCANLINE_BUFFER2_WORDS PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS
#endif
#ifndef PICO_SCANVIDEO_MAX_SCANLINE_BUFFER3_WORDS
#define PICO_SCANVIDEO_MAX_SCANLINE_BUFFER3_WORDS PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS
#endif

//extern struct semaphore vmode_updated;


// ======================

#define BPP 16

// most likely 24000000
extern const uint32_t video_clock_freq;

// todo pragma pack?
typedef struct scanvideo_timing {
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
} scanvideo_timing_t;

typedef struct scanvideo_pio_program scanvideo_pio_program_t;

// todo we need to handle blank data correctly (perhaps DMA should just not start for that scanline,
//  though obviously this is slightly more complicated with multiple playfields, or perhaps worse with
//  just one
typedef struct scanvideo_mode {
    const scanvideo_timing_t *default_timing;
    const scanvideo_pio_program_t *pio_program;

    uint16_t width;
    uint16_t height;
    uint8_t xscale; // 1 == normal, 2 == double wide etc. up to what pio timing allows (not sure I have an assert based on delays)
    uint16_t yscale; // same for y scale (except any yscale is possible)
    // if > 1 then yscale is divided by this to provide the effective yscale;
    // note that yscale must be > yscale_denominator; i.e. only stretching is supported
    uint16_t yscale_denominator;
} scanvideo_mode_t;

extern bool scanvideo_setup(const scanvideo_mode_t *mode);
extern bool scanvideo_setup_with_timing(const scanvideo_mode_t *mode, const scanvideo_timing_t *timing);
extern void scanvideo_timing_enable(bool enable);
// these take effect after the next vsync
extern void scanvideo_display_enable(bool enable);
// doesn't exist yet!
// extern void video_set_display_mode(const struct scanvideo_mode *mode);

// --- scanline management ---

typedef struct scanvideo_scanline_buffer {
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
    void *user_data;
#if PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS
    struct scanvideo_scanline_buffer *link;
    uint8_t link_after;
#endif
    uint8_t status;
} scanvideo_scanline_buffer_t;

enum {
    SCANLINE_OK = 1,
    SCANLINE_ERROR,
    SCANLINE_SKIPPED
};

// note frame numbers wrap
static inline uint16_t scanvideo_frame_number(uint32_t scanline_id) {
    return (uint16_t) (scanline_id >> 16u);
}

static inline uint16_t scanvideo_scanline_number(uint32_t scanline_id) {
    return (uint16_t) scanline_id;
}

/**
 * @return the current vga mode (if there is one)
 */
extern scanvideo_mode_t scanvideo_get_mode();

/**
 * @return the next scanline_id to be displayed (may be from the next frame)
 */
extern uint32_t scanvideo_get_next_scanline_id();

/**
 * @return true if in the vblank interval
 */
extern bool scanvideo_in_vblank();
/**
 * @return true if in the hblank interval (or more accurately scanline data is not currently being sent to the PIO, which roughly corresponds, but is not exact). Note also that in
 * yscale-d modes, there are height * yscale hblank intervals per frame.
 */
extern bool scanvideo_in_hblank();

extern void scanvideo_wait_for_vblank();

extern uint32_t scanvideo_wait_for_scanline_complete(uint32_t scanline_id);
/**
 * Acquire a scanline that needs generating. The scanline_id field indicates which scanline is required.
 *
 * This method may be called concurrently
 *
 * @param block true to block if the vga system is not ready to generate a new scanline
 * @return the scanline_buffer or NULL if block is false, and the vga system is not ready
 */
scanvideo_scanline_buffer_t *scanvideo_begin_scanline_generation(bool block);
scanvideo_scanline_buffer_t *scanvideo_begin_scanline_generation2(scanvideo_scanline_buffer_t **second, bool block);
#if PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS
scanvideo_scanline_buffer_t *scanvideo_begin_scanline_generation_linked(uint n, bool block);
#endif

/**
 * Return a scanline that has been generated / or at least the client is done with.
 *
 * The status field indicates whether the scanline was actually generated OK
 *
 * This method may be called concurrently (for different buffers)
 *
 * @param scanline_buffer \todo
 */
void scanvideo_end_scanline_generation(scanvideo_scanline_buffer_t *scanline_buffer);

typedef uint (*scanvideo_scanline_repeat_count_fn)(uint32_t scanline_id);
void scanvideo_set_scanline_repeat_fn(scanvideo_scanline_repeat_count_fn fn);

extern const scanvideo_timing_t vga_timing_640x480_60_default;
extern const scanvideo_timing_t vga_timing_1280x1024_60_default;
extern const scanvideo_timing_t vga_timing_wide_480_50;
extern const scanvideo_timing_t vga_timing_648x480_60_alt1;

extern const scanvideo_mode_t vga_mode_160x120_60; // 3d monster maze anyone :-)
extern const scanvideo_mode_t vga_mode_213x160_60;
extern const scanvideo_mode_t vga_mode_320x240_60;
extern const scanvideo_mode_t vga_mode_640x480_60;
extern const scanvideo_mode_t vga_mode_800x600_54;
extern const scanvideo_mode_t vga_mode_800x600_60;
extern const scanvideo_mode_t vga_mode_1024x768_63;
extern const scanvideo_mode_t vga_mode_1280x1024_40;
extern const scanvideo_mode_t vga_mode_1024x768_60;
extern const scanvideo_mode_t vga_mode_1280x1024_60;
extern const scanvideo_mode_t vga_mode_720p_60;
extern const scanvideo_mode_t vga_mode_1080p_60;

extern const scanvideo_mode_t vga_mode_tft_800x480_50;
extern const scanvideo_mode_t vga_mode_tft_400x240_50;

#ifndef NDEBUG
// todo this is only for vga composable 24... should exist behind mode impl
extern void validate_scanline(const uint32_t *dma_data, uint dma_data_size, uint max_pixels, uint expected_width);
#endif

// mode implementation

struct scanvideo_pio_program {
#if !PICO_NO_HARDWARE
    const pio_program_t *program;
    const uint8_t entry_point;
    // modifiable_instructions is of size program->length
    bool (*adapt_for_mode)(const scanvideo_pio_program_t *program, const scanvideo_mode_t *mode,
                           scanvideo_scanline_buffer_t *missing_scanline_buffer, uint16_t *modifiable_instructions);
    pio_sm_config (*configure_pio)(pio_hw_t *pio, uint sm, uint offset);
#else
    const char *id;
#endif
};

extern const scanvideo_pio_program_t video_24mhz_composable;

#if !PICO_NO_HARDWARE
extern void scanvideo_default_configure_pio(pio_hw_t *pio, uint sm, uint offset, pio_sm_config *config, bool overlay);
#endif

#ifndef PICO_SPINLOCK_ID_VIDEO_SCANLINE_LOCK
#define PICO_SPINLOCK_ID_VIDEO_SCANLINE_LOCK 2
#endif

#ifndef PICO_SPINLOCK_ID_VIDEO_FREE_LIST_LOCK
#define PICO_SPINLOCK_ID_VIDEO_FREE_LIST_LOCK 3
#endif

#ifndef PICO_SPINLOCK_ID_VIDEO_DMA_LOCK
#define PICO_SPINLOCK_ID_VIDEO_DMA_LOCK 4
#endif

#ifndef PICO_SPINLOCK_ID_VIDEO_IN_USE_LOCK
#define PICO_SPINLOCK_ID_VIDEO_IN_USE_LOCK 5
#endif

// note this is not necessarily an absolute gpio pin mask, it is still shifted by PICO_SCANVIDEO_COLOR_PIN_BASE
#define PICO_SCANVIDEO_ALPHA_MASK (1u << PICO_SCANVIDEO_ALPHA_PIN)

#ifndef PICO_SCANVIDEO_PIXEL_FROM_RGB8
#define PICO_SCANVIDEO_PIXEL_FROM_RGB8(r, g, b) ((((b)>>3u)<<PICO_SCANVIDEO_PIXEL_BSHIFT)|(((g)>>3u)<<PICO_SCANVIDEO_PIXEL_GSHIFT)|(((r)>>3u)<<PICO_SCANVIDEO_PIXEL_RSHIFT))
#endif

#ifndef PICO_SCANVIDEO_PIXEL_FROM_RGB5
#define PICO_SCANVIDEO_PIXEL_FROM_RGB5(r, g, b) (((b)<<PICO_SCANVIDEO_PIXEL_BSHIFT)|((g)<<PICO_SCANVIDEO_PIXEL_GSHIFT)|((r)<<PICO_SCANVIDEO_PIXEL_RSHIFT))
#endif

#ifndef PICO_SCANVIDEO_R5_FROM_PIXEL
#define PICO_SCANVIDEO_R5_FROM_PIXEL(p) (((p)>>PICO_SCANVIDEO_PIXEL_RSHIFT)&0x1f)
#endif

#ifndef PICO_SCANVIDEO_G5_FROM_PIXEL
#define PICO_SCANVIDEO_G5_FROM_PIXEL(p) (((p)>>PICO_SCANVIDEO_PIXEL_GSHIFT)&0x1f)
#endif

#ifndef PICO_SCANVIDEO_B5_FROM_PIXEL
#define PICO_SCANVIDEO_B5_FROM_PIXEL(p) (((p)>>PICO_SCANVIDEO_PIXEL_BSHIFT)&0x1f)
#endif

#ifdef __cplusplus
}
#endif

#endif //_VIDEO_H
