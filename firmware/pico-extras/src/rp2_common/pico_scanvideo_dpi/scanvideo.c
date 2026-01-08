/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma GCC push_options

#if !PICO_SCANVIDEO_DEBUG_IMPL
#undef PARAM_ASSERTIONS_DISABLE_ALL
#define PARAM_ASSERTIONS_DISABLE_ALL 1
#pragma GCC optimize("O3")
#endif

#include <stdlib.h>
#include <stdio.h>
#include "pico/platform.h"
#include "pico/sem.h"
#include "pico/util/buffer.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "timing.pio.h"
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "hardware/structs/bus_ctrl.h"
#include "pico/binary_info.h"

#if PICO_SCANVIDEO_PLANE_COUNT > 3
#error only up to 3 planes supported
#endif

// PICO_CONFIG: PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY, Enable/disable video recovery,type=bool, default=1, group=video
#ifndef PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
#define PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY 1
#endif

// PICO_CONFIG: PICO_SCANVIDEO_ADJUST_BUS_PRIORITY, Enable/disable adjust bus priority, type=bool, default=0, group=video
#ifndef PICO_SCANVIDEO_ADJUST_BUS_PRIORITY
#define PICO_SCANVIDEO_ADJUST_BUS_PRIORITY 0
#endif

// PICO_CONFIG: PICO_SCANVIDEO_ENABLE_SCANLINE_ASSERTIONS, Enable/disable scanline assertions, type=bool, default=0, group=video
#ifndef PICO_SCANVIDEO_ENABLE_SCANLINE_ASSERTIONS
#define PICO_SCANVIDEO_ENABLE_SCANLINE_ASSERTIONS 0
#endif

// PICO_CONFIG: PICO_SCANVIDEO_DEBUG_IMPL, Enable/disable debug implementation, type=bool, default=0, group=video
#ifndef PICO_SCANVIDEO_DEBUG_IMPL
#define PICO_SCANVIDEO_DEBUG_IMPL 0
#endif

// PICO_CONFIG: PICO_SCANVIDEO_NO_DMA_TRACKING, Enable/disable DMA tracking, type=bool, default=0, group=video
#ifndef PICO_SCANVIDEO_NO_DMA_TRACKING
#define PICO_SCANVIDEO_NO_DMA_TRACKING 0
#endif

#define PICO_SCANVIDEO_SCANLINE_SM 0u
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL 0u

// PICO_CONFIG: PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA, Enable/disable plane 1 DMA fragments, type=bool, default=0, group=video
#ifndef PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA 0
#endif

#if PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
#define PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL 3u
#endif

#define PICO_SCANVIDEO_TIMING_SM 3u
#if PICO_SCANVIDEO_PLANE_COUNT > 1
#define PICO_SCANVIDEO_SCANLINE_SM2 1u
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2 1u

// PICO_CONFIG: PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA, Enable/disable plane 2 DMA fragments, type=bool, default=0, group=video
#ifndef PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA
#define PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA 0
#endif

#if PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA
#define PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2 4u
#endif

#if PICO_SCANVIDEO_PLANE_COUNT > 2
#define PICO_SCANVIDEO_SCANLINE_SM3 2u
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3 2u
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNELS_MASK ((1u << PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL) | ( 1u << PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2) | ( 1u << PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3))
#else
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNELS_MASK ((1u << PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL) | ( 1u << PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2))
#endif
#else
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNELS_MASK (1u << PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL)
#endif

// todo add ability to shift scanline back a bit (we already have the timing, but it should be a post mode set adjustment)
//  we can use this to allow some initial work in the scanline b4 the first pixel (e.g. a dummy black pixel)

// todo bad state recovery
//      - stress test with pause/unpause
//      - bad state should cause SCANLINE_ASSERTION_ERROR
//      - possible orphaned in_use - perhaps clean up when error state is detected
//      - if PIO is not in the right place, pause/clear FIFO join-unjoin/jmp/resume
//      - dma may need to be cancelled
// todo dma chaining support


// == DEBUGGING =========

// note that this is very very important if you see things going wrong with the display,
// however beware, because it will also cause visual artifiacts if we are pushing the edge of the envelope
// since it itself uses cycles that are in short supply! This is why it is off by default
//
// todo note, it should eventually be difficult to get the display into a bad state (even
//  with things like runaway scanline program; incomplete DMA etc.. which currently break it).

CU_REGISTER_DEBUG_PINS(video_timing, video_dma_buffer, video_irq, video_dma_completion, video_generation,
                       video_recovery, video_in_use, video_link)

// ---- select at most one ---
//CU_SELECT_DEBUG_PINS(video_recovery)
//CU_SELECT_DEBUG_PINS(video_generation)
//CU_SELECT_DEBUG_PINS(video_in_use)
//CU_SELECT_DEBUG_PINS(scanvideo_timing)
//CU_SELECT_DEBUG_PINS(video_irq)
//CU_SELECT_DEBUG_PINS(video_dma_buffer)
//CU_SELECT_DEBUG_PINS(video_dma_completion)
//CU_SELECT_DEBUG_PINS(video_link)

// ======================

#if PICO_SCANVIDEO_ENABLE_SCANLINE_ASSERTIONS
// we want some sort of assertion even in release builds
#ifndef NDEBUG
#define scanline_assert(x) assert(x)
#else
#define scanline_assert(x) hard_assert(x)
#endif
#else
#define scanline_assert(x) (void)0
#endif

#define video_pio pio0

#if PICO_SCANVIDEO_ADJUST_BUS_PRIORITY
#define VIDEO_ADJUST_BUS_PRIORITY_VAL (BUSCTRL_BUS_PRIORITY_PROC0_BITS | BUSCTRL_BUS_PRIORITY_PROC1_BITS)
#endif

#ifdef VIDEO_MOST_TIME_CRITICAL_CODE_SECTION
#define __video_most_time_critical_func(x) __attribute__((section(__XSTRING(VIDEO_MOST_TIME_CRITICAL_CODE_SECTION) "." x))) x
#else
#define __video_most_time_critical_func(x) __not_in_flash_func(x)
#endif

#ifdef VIDEO_TIME_CRITICAL_CODE_SECTION
#define __video_time_critical_func(x) __attribute__((section(__XSTRING(VIDEO_TIME_CRITICAL_CODE_SECTION) "." x))) x
#else
#define __video_time_critical_func(x) __not_in_flash_func(x)
#endif
// --- video_24mhz_composable ---

#define video_24mhz_composable_program __CONCAT(video_24mhz_composable_prefix, _program)
#define video_24mhz_composable_wrap_target __CONCAT(video_24mhz_composable_prefix, _wrap_target)
#define video_24mhz_composable_wrap __CONCAT(video_24mhz_composable_prefix, _wrap)

bool video_24mhz_composable_adapt_for_mode(const scanvideo_pio_program_t *program, const scanvideo_mode_t *mode,
                                           scanvideo_scanline_buffer_t *missing_scanline_buffer,
                                           uint16_t *modifiable_instructions);

pio_sm_config video_24mhz_composable_configure_pio(pio_hw_t *pio, uint sm, uint offset);

const scanvideo_pio_program_t video_24mhz_composable = {
        .program = &video_24mhz_composable_program,
        .entry_point = video_24mhz_composable_program_extern(entry_point),
        .adapt_for_mode = video_24mhz_composable_adapt_for_mode,
        .configure_pio = video_24mhz_composable_configure_pio
};

#define PIO_WAIT_IRQ4 pio_encode_wait_irq(1, false, 4)
static uint8_t video_htiming_load_offset;
static uint8_t video_program_load_offset;

// --- video timing stuff

// 4 possible instructions; index into program below
enum {
    SET_IRQ_0 = 0u,
    SET_IRQ_1 = 1u,
    SET_IRQ_SCANLINE = 2u,
    CLEAR_IRQ_SCANLINE = 3u,
};

static struct {
    int32_t v_active;
    int32_t v_total;
    int32_t v_pulse_start;
    int32_t v_pulse_end;
    // todo replace with plain polarity
    uint32_t vsync_bits_pulse;
    uint32_t vsync_bits_no_pulse;

    uint32_t a, a_vblank, b1, b2, c, c_vblank;
    uint32_t vsync_bits;
    uint16_t dma_state_index;
    int32_t timing_scanline;
} timing_state;

#define DMA_STATE_COUNT 4
static uint32_t dma_states[DMA_STATE_COUNT];

// todo get rid of this altogether
#undef PICO_SCANVIDEO_ENABLE_VIDEO_CLOCK_DOWN
#define PICO_SCANVIDEO_ENABLE_VIDEO_CLOCK_DOWN 1

#if PICO_SCANVIDEO_ENABLE_VIDEO_CLOCK_DOWN
static uint16_t video_clock_down_times_2;
#endif

semaphore_t vblank_begin;

// --- scanline stuff
// private representation of scanline buffer (adds link for one list this scanline buffer is currently in)
typedef struct full_scanline_buffer {
    scanvideo_scanline_buffer_t core;
    struct full_scanline_buffer *next;
} full_scanline_buffer_t;

// each scanline_buffer should be in exactly one of the shared_state lists below
// (unless we don't have USE_SCANLINE_DEBUG in which case we don't keep the generating list,
// in which case the scanline is entirely trusted to the client when generating)
full_scanline_buffer_t scanline_buffers[PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT];

// This state is sensitive as it it accessed by either core, and multiple IRQ handlers which may be re-entrant
// Nothing in here should be touched except when protected by the appropriate spin lock.
//
// The separations by spin lock (other than the need for spin locks to protect state consistency) is to allow
// safe concurrent operation by both cores, client, IRQ and nested IRQ (pre-emption) where desirable due
// to timing concerns.
static struct {
    struct {
        spin_lock_t *lock;
        // note in_use is a list as we are lazy in removing buffers from it
        full_scanline_buffer_t *in_use_ascending_scanline_id_list;
        // pointer to the tail element of the list for making appending by ascending scanline id quick
        full_scanline_buffer_t *in_use_ascending_scanline_id_list_tail;
    } in_use;

    struct {
        spin_lock_t *lock;
        full_scanline_buffer_t *current_scanline_buffer;
        uint32_t last_scanline_id;
        uint32_t next_scanline_id;
        // 0 based index of y repeat... goes 0, 0, 0 in non scaled mode, 0, 1, 0, 1 in doubled etc.
        uint16_t y_repeat_index;
        uint16_t y_repeat_target;
        bool in_vblank;
        // This generated list is in this struct because it is accessed together in fsb latching
        // and the only other place it is used in scanvideo_end_scanline_generation which needs no other
        // locks (i.e. we are saving an extra lock in the latch case by not placing in a separate struct)
        full_scanline_buffer_t *generated_ascending_scanline_id_list;
        full_scanline_buffer_t *generated_ascending_scanline_id_list_tail;
#if PICO_SCANVIDEO_ENABLE_SCANLINE_ASSERTIONS
        full_scanline_buffer_t *generating_list;
#endif
    } scanline;

    struct {
        spin_lock_t *lock;
        full_scanline_buffer_t *free_list;
    } free_list;

    // This is access by DMA IRQ and by SM IRQs
    struct {
        spin_lock_t *lock;
#if !PICO_SCANVIDEO_NO_DMA_TRACKING
        // bit mask of completed DMA scanline channels
        uint32_t dma_completion_state;
#endif
        // number of buffers to release (may be multiple due to interrupt pre-emption)
        // todo combine these two fields
        uint8_t buffers_to_release;
        bool scanline_in_progress;
    } dma;

    // these are not updated, so not locked
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
    int scanline_program_wait_index;
#endif
} shared_state;

// PICO_CONFIG: PICO_SCANVIDEO_MISSING_SCANLINE_COLOR, Define colour used for missing scanlines, default=PICO_SVIDEO_PIXEL_FROM_RGB8(0,0,255), group=video
#ifndef PICO_SCANVIDEO_MISSING_SCANLINE_COLOR
#define PICO_SCANVIDEO_MISSING_SCANLINE_COLOR PICO_SCANVIDEO_PIXEL_FROM_RGB8(0,0,255)
#endif
static uint32_t _missing_scanline_data[] =
        {
                COMPOSABLE_COLOR_RUN | (PICO_SCANVIDEO_MISSING_SCANLINE_COLOR << 16u),
                /*width-3*/ 0u | (COMPOSABLE_RAW_1P << 16u),
                0u | (COMPOSABLE_EOL_ALIGN << 16u)
        };

#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
static uint32_t variable_fragment_missing_scanline_data_chain[] = {
    count_of(_missing_scanline_data),
    0, // missing_scanline_data,
    0,
    0,
};
#endif

#if PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE2_FIXED_FRAGMENT_DMA
static uint32_t fixed_fragment_missing_scanline_data_chain[] = {
    0, // missing_scanline_data,
    0,
};
#endif

#if PICO_SCANVIDEO_PLANE_COUNT > 1
uint32_t missing_scanline_data_overlay[] = {
    // blank line
    0u                   | (COMPOSABLE_EOL_ALIGN << 16u)
};
#endif

static full_scanline_buffer_t _missing_scanline_buffer;

static inline bool is_scanline_after(uint32_t scanline_id1, uint32_t scanline_id2) {
    return ((int32_t) (scanline_id1 - scanline_id2)) > 0;
}

static void prepare_for_active_scanline_irqs_enabled();

static void scanline_dma_complete_irqs_enabled();

static void setup_sm(int sm, uint offset);

// -- MISC stuff
static scanvideo_mode_t video_mode;
static bool video_timing_enabled = false;
static bool display_enabled = true;

static scanvideo_scanline_repeat_count_fn _scanline_repeat_count_fn;

inline static void list_prepend(full_scanline_buffer_t **phead, full_scanline_buffer_t *fsb) {
    scanline_assert(fsb);
    scanline_assert(fsb->next == NULL);
    scanline_assert(fsb != *phead);
    fsb->next = *phead;
    *phead = fsb;
}

inline static void list_prepend_all(full_scanline_buffer_t **phead, full_scanline_buffer_t *to_prepend) {
    full_scanline_buffer_t *fsb = to_prepend;

    // todo should this be assumed?
    if (fsb) {
        while (fsb->next) {
            fsb = fsb->next;
        }

        fsb->next = *phead;
        *phead = to_prepend;
    }
}

inline static full_scanline_buffer_t *list_remove_head(full_scanline_buffer_t **phead) {
    full_scanline_buffer_t *fsb = *phead;

    if (fsb) {
        *phead = fsb->next;
        fsb->next = NULL;
    }

    return fsb;
}

inline static full_scanline_buffer_t *list_remove_head_ascending(full_scanline_buffer_t **phead,
                                                                      full_scanline_buffer_t **ptail) {
    full_scanline_buffer_t *fsb = *phead;

    if (fsb) {
        *phead = fsb->next;

        if (!fsb->next) {
            scanline_assert(*ptail == fsb);
            *ptail = NULL;
        } else {
            fsb->next = NULL;
        }
    }

    return fsb;
}

inline static void list_remove(full_scanline_buffer_t **phead, full_scanline_buffer_t *fsb) {
    scanline_assert(*phead);
    full_scanline_buffer_t *prev = *phead;

    if (prev == fsb) {
        *phead = fsb->next;
    } else {
        while (prev->next && prev->next != fsb) {
            prev = prev->next;
        }

        scanline_assert(prev->next == fsb);
        prev->next = fsb->next;
    }

    // todo do we need this without assertions?
    fsb->next = NULL;
}

static inline uint32_t scanline_id_after(uint32_t scanline_id) {
    uint32_t tmp = scanline_id & 0xffffu;

    if (tmp < video_mode.height - 1) {
        return scanline_id + 1;
    } else {
        return scanline_id + 0x10000u - tmp;
    }
}

// todo add a tail for these already sorted lists as we generally insert on the end
inline static void list_insert_ascending(full_scanline_buffer_t **phead, full_scanline_buffer_t **ptail,
                                         full_scanline_buffer_t *fsb) {
    scanline_assert(fsb->next == NULL);
    scanline_assert(fsb != *phead);
    scanline_assert(fsb != *ptail);

    if (!*phead || !is_scanline_after(fsb->core.scanline_id, (*phead)->core.scanline_id)) {
        if (!*phead) {
            scanline_assert(!*ptail);
            *ptail = fsb;
        }

        // insert at the beginning
        list_prepend(phead, fsb);
    } else {
        if (is_scanline_after(fsb->core.scanline_id, (*ptail)->core.scanline_id)) {
            // insert at end
            (*ptail)->next = fsb;
            *ptail = fsb;
        } else {
            // not after
            full_scanline_buffer_t *prev = *phead;

            while (prev->next && is_scanline_after(fsb->core.scanline_id, prev->next->core.scanline_id)) {
                prev = prev->next;
            }

            scanline_assert(prev != *ptail); // we should have already inserted at the end in this case
            fsb->next = prev->next;
            prev->next = fsb;
        }
    }
}

inline static void free_local_free_list_irqs_enabled(full_scanline_buffer_t *local_free_list) {
    if (local_free_list) {
        uint32_t save = spin_lock_blocking(shared_state.free_list.lock);
        DEBUG_PINS_SET(video_timing, 4);
        list_prepend_all(&shared_state.free_list.free_list, local_free_list);
        DEBUG_PINS_CLR(video_timing, 4);
        spin_unlock(shared_state.free_list.lock, save);
        // note also this is useful for triggering scanvideo_wait_for_scanline_complete check
        __sev();
    }
}

// Caller must own scanline_state_spin_lock
inline static full_scanline_buffer_t *scanline_locked_try_latch_fsb_if_null_irqs_disabled(
        full_scanline_buffer_t **local_free_list) {
    // note this just checks that someone owns it not necessarily this core.
    scanline_assert(is_spin_locked(shared_state.scanline.lock));
    full_scanline_buffer_t *fsb = shared_state.scanline.current_scanline_buffer;

    if (!fsb) {
        // peek the head
        while (NULL != (fsb = shared_state.scanline.generated_ascending_scanline_id_list)) {
            if (!is_scanline_after(shared_state.scanline.next_scanline_id, fsb->core.scanline_id)) {
                if (shared_state.scanline.next_scanline_id == fsb->core.scanline_id) {
                    full_scanline_buffer_t __unused *dbg = list_remove_head_ascending(
                            &shared_state.scanline.generated_ascending_scanline_id_list,
                            &shared_state.scanline.generated_ascending_scanline_id_list_tail);
                    scanline_assert(dbg == fsb);
                    spin_lock_unsafe_blocking(shared_state.in_use.lock);
                    DEBUG_PINS_SET(video_timing, 2);
                    DEBUG_PINS_XOR(video_in_use, 1);
                    list_insert_ascending(&shared_state.in_use.in_use_ascending_scanline_id_list,
                                          &shared_state.in_use.in_use_ascending_scanline_id_list_tail, fsb);
                    DEBUG_PINS_CLR(video_timing, 2);
                    spin_unlock_unsafe(shared_state.in_use.lock);
                    shared_state.scanline.current_scanline_buffer = fsb;
                } else {
                    fsb = NULL;
                }

                break;
            } else {
                // scanline is in the past
                full_scanline_buffer_t __unused *dbg = list_remove_head_ascending(
                        &shared_state.scanline.generated_ascending_scanline_id_list,
                        &shared_state.scanline.generated_ascending_scanline_id_list_tail);
                scanline_assert(dbg == fsb);
                list_prepend(local_free_list, fsb);
#if PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS
                full_scanline_buffer_t *fsb2;
                while(NULL != (fsb2 = (full_scanline_buffer_t *)fsb->core.link)) {
                    fsb->core.link = NULL;
                    DEBUG_PINS_SET(video_link, 2);
                    list_prepend(local_free_list, fsb2);
                    DEBUG_PINS_CLR(video_link, 2);
                    fsb = fsb2;
                }
#endif
            }
        }
    }

    return fsb;
}

static inline void release_scanline_irqs_enabled(int buffers_to_free_count,
                                                 full_scanline_buffer_t **local_free_list) {
    if (buffers_to_free_count) {
        uint32_t save = spin_lock_blocking(shared_state.in_use.lock);
        while (buffers_to_free_count--) {
            DEBUG_PINS_SET(video_dma_buffer, 2);
            // We always discard the head which is the oldest
            DEBUG_PINS_XOR(video_in_use, 2);
            full_scanline_buffer_t *fsb = list_remove_head_ascending(
                    &shared_state.in_use.in_use_ascending_scanline_id_list,
                    &shared_state.in_use.in_use_ascending_scanline_id_list_tail);
            list_prepend(local_free_list, fsb);
#if PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS
            full_scanline_buffer_t *fsb2;
            while(NULL != (fsb2 = (full_scanline_buffer_t *)fsb->core.link)) {
                fsb->core.link = NULL;
                DEBUG_PINS_SET(video_link, 2);
                list_prepend(local_free_list, fsb2);
                DEBUG_PINS_CLR(video_link, 2);
                fsb = fsb2;
            }
#endif
            DEBUG_PINS_CLR(video_dma_buffer, 2);
        }
        spin_unlock(shared_state.in_use.lock, save);
    }
}

static inline bool update_dma_transfer_state_irqs_enabled(bool cancel_if_not_complete,
                                                          int *scanline_buffers_to_release) {
    uint32_t save = spin_lock_blocking(shared_state.dma.lock);
    if (!shared_state.dma.scanline_in_progress) {
#if !PICO_SCANVIDEO_NO_DMA_TRACKING
        assert(!shared_state.dma.dma_completion_state);
#endif
        assert(!shared_state.dma.buffers_to_release);
        spin_unlock(shared_state.dma.lock, save);
        return true;
    }
#if !PICO_SCANVIDEO_NO_DMA_TRACKING
    uint32_t old_completed = shared_state.dma.dma_completion_state;
    uint32_t new_completed;
    while (0 != (new_completed = dma_hw->ints0 & PICO_SCANVIDEO_SCANLINE_DMA_CHANNELS_MASK)) {
        scanline_assert(!(old_completed & new_completed));
        // clear interrupt flags
        dma_hw->ints0 = new_completed;
        DEBUG_PINS_SET(video_dma_completion, new_completed);
        DEBUG_PINS_CLR(video_dma_completion, new_completed);
        new_completed |= old_completed;
        if (new_completed == PICO_SCANVIDEO_SCANLINE_DMA_CHANNELS_MASK) {
            // tell caller to free these buffers... note it is safe to release any outstanding ones
            // as only one DMA transfer can be logically in process and we have just finished that
            // if the number is > 1 this is due to IRQ / preemption (todo this comment is out of date)
            *scanline_buffers_to_release = shared_state.dma.buffers_to_release;
            // we have taken ownership of releasing all the current ones
            shared_state.dma.buffers_to_release = 0;
            if (*scanline_buffers_to_release) {
                // now that ISR clearing is protected by lock and also done by the active_scanline start
                // we cannot have nesting
                scanline_assert(*scanline_buffers_to_release == 1);
                DEBUG_PINS_SET(video_dma_completion, 1);
                DEBUG_PINS_CLR(video_dma_completion, 1);
            }
            shared_state.dma.dma_completion_state = shared_state.dma.scanline_in_progress = 0;
            spin_unlock(shared_state.dma.lock, save);
            return true;
        } else {
            DEBUG_PINS_SET(video_dma_completion, 2);
            DEBUG_PINS_CLR(video_dma_completion, 2);
            shared_state.dma.dma_completion_state = old_completed = new_completed;
        }
    }
    // can't cancel yet, note if dma_buffers_to_release = 0 then completion DID happen (todo is this ever the case)
    if (cancel_if_not_complete) {
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
        if (shared_state.dma.buffers_to_release) {
            shared_state.dma.dma_completion_state = shared_state.dma.scanline_in_progress = 0;
            *scanline_buffers_to_release = shared_state.dma.buffers_to_release;
            shared_state.dma.buffers_to_release = 0;
            DEBUG_PINS_XOR(video_in_use, 4);
        }
        dma_channel_abort(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL);
#if PICO_SCANVIDEO_PLANE_COUNT > 1
        dma_channel_abort(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2);
#if PICO_SCANVIDEO_PLANE_COUNT > 2
        dma_channel_abort(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3);
#endif
#endif
#else
        panic("need VIDEO_RECOVERY");
#endif
    }
    spin_unlock(shared_state.dma.lock, save);
    return cancel_if_not_complete;
#else
    if (shared_state.dma.buffers_to_release) {
        shared_state.dma.scanline_in_progress = 0;
        *scanline_buffers_to_release = shared_state.dma.buffers_to_release;
        shared_state.dma.buffers_to_release = 0;
        dma_abort(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL);
#if PICO_SCANVIDEO_PLANE_COUNT > 1
        dma_abort(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2);
#if PICO_SCANVIDEO_PLANE_COUNT > 2
        dma_channel_abort(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3);
#endif
#endif
    }
    spin_unlock(shared_state.dma.lock, save);
    return false;
#endif
}

#if !PICO_SCANVIDEO_NO_DMA_TRACKING

static inline void scanline_dma_complete_irqs_enabled() {
    // The DMA interrupt may be pre-empted by SM IRQ interrupt at any point, so it is possible even in non multi plane
    // that this IRQ handler is not the one that is responsible for dealing with the end of the transfer.
    // In the multi plane case, there are to different DMAs to worry about which may or may not both be complete
    // by the time we get here with one having completed.
    DEBUG_PINS_SET(video_dma_completion, 4);
    int buffers_to_free_count = 0;
    bool is_completion_trigger = update_dma_transfer_state_irqs_enabled(false, &buffers_to_free_count);
    full_scanline_buffer_t *local_free_list = NULL;
    if (is_completion_trigger) {
        uint32_t save = spin_lock_blocking(shared_state.scanline.lock);
        DEBUG_PINS_SET(video_timing, 1);
        // We make an early attempt to latch a scanline buffer to save time later in the PIO SM IRQ handler
        // Of course there may not be a buffer ready yet, or we may have been pre-empty by the PIO SM IRQ handler
        // already, in which case fsb will be non null
        scanline_locked_try_latch_fsb_if_null_irqs_disabled(&local_free_list); // do an early attempt to latch
        DEBUG_PINS_CLR(video_timing, 1);
        spin_unlock(shared_state.scanline.lock, save);
    }

    // because IRQs are enabled, we may obviously be pre-empted before or between either of these
    release_scanline_irqs_enabled(buffers_to_free_count, &local_free_list);
    free_local_free_list_irqs_enabled(local_free_list);
    DEBUG_PINS_CLR(video_dma_completion, 4);
}

#endif

static void set_next_scanline_id(uint32_t scanline_id) {
    shared_state.scanline.next_scanline_id = scanline_id;
    shared_state.scanline.y_repeat_target = _scanline_repeat_count_fn(scanline_id) * video_mode.yscale;
}

void __video_most_time_critical_func(prepare_for_active_scanline_irqs_enabled)() {
    // note we are now only called in active display lines..
    DEBUG_PINS_SET(video_timing, 1);
    full_scanline_buffer_t *local_free_list = NULL;
    int buffers_to_free_count = 0;
    uint32_t save = spin_lock_blocking(shared_state.scanline.lock);
    // VERY IMPORTANT: THIS CODE CAN ONLY TAKE ABOUT 4.5 us BEFORE LAUNCHING DMA...
    // ... otherwise our scanline will be shifted over (because we will have started display)
    //
    // to alleviate this somewhat, we let the dma_complete alsu do a check for current_scanline_buffer == null, and look for a completed scanlines
    // In ideal case the dma complete IRQ handler will have been able to set current_scanline_buffer for us (or indeed this is a y scaled mode
    // and we are repeating a line)... in either case we will come in well under time budget
    full_scanline_buffer_t *fsb = scanline_locked_try_latch_fsb_if_null_irqs_disabled(&local_free_list);

    spin_unlock(shared_state.scanline.lock, save);
    DEBUG_PINS_CLR(video_timing, 1);
    if (fsb) {
        if (fsb->core.scanline_id != shared_state.scanline.next_scanline_id) {
            // removed to allow for other video modes; not worth abstracting that far...
            // also; we basically never see this color anyway!
//            ((uint16_t *) (missing_scanline_data))[1] = 0x03e0;
            // note: this should be in the future
            fsb = &_missing_scanline_buffer;
        }
    } else {
        // removed to allow for other video modes; not worth abstracting that far...
//        ((uint16_t *)(missing_scanline_data))[1] = 0x001f;
        // this is usually set by latch
        fsb = &_missing_scanline_buffer;
    }

    update_dma_transfer_state_irqs_enabled(true, &buffers_to_free_count);

//    DEBUG_PINS_SET(video_irq, 2);
    // bit of overkill (to reset src_addr) for y scale repeat lines, but then again those should already have data. but this is now
    // required in case current_scanline_buffer was set by the dma complete handler, in which case current_scanline_buffer was null when we got to the test above

    // don't need to reset anything put the CB pointer to start a reload? as we have already configured the rest
    // note DMA should already be aborted by here.
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
    if (!pio_sm_is_tx_fifo_empty(video_pio, PICO_SCANVIDEO_SCANLINE_SM)) {
        pio_sm_clear_fifos(video_pio, PICO_SCANVIDEO_SCANLINE_SM);
    }
    if (video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM].instr != PIO_WAIT_IRQ4) {
        // hmm the problem here is we don't know if we should wait or not, because that is purely based on timing..
        // - if irq not posted, and we wait: GOOD
        // - if irq not posted and we don't wait: BAD. early line
        // - if irq already posted, and we wait: BAD. blank line
        // - id irq already posted, and we don't wait: GOOD
        pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM, pio_encode_wait_irq(1, false, 4));
        if (pio_sm_is_exec_stalled(video_pio, PICO_SCANVIDEO_SCANLINE_SM)) {
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM, pio_encode_jmp(shared_state.scanline_program_wait_index));
        } else {
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM,
                        pio_encode_jmp(shared_state.scanline_program_wait_index + 1));
        }
    }
#if PICO_SCANVIDEO_PLANE_COUNT > 1
    if (!pio_sm_is_tx_fifo_empty(video_pio, PICO_SCANVIDEO_SCANLINE_SM2)) {
        pio_sm_clear_fifos(video_pio, PICO_SCANVIDEO_SCANLINE_SM2);
    }
    if (video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM2].instr != PIO_WAIT_IRQ4) {
        // hmm the problem here is we don't know if we should wait or not, because that is purely based on timing..
        // - if irq not posted, and we wait: GOOD
        // - if irq not posted and we don't wait: BAD. early line
        // - if irq already posted, and we wait: BAD. blank line
        // - id irq already posted, and we don't wait: GOOD
        pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM2, pio_encode_wait_irq(1, false, 4));
        if (pio_sm_is_exec_stalled(video_pio, PICO_SCANVIDEO_SCANLINE_SM2)) {
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM2, pio_encode_jmp(shared_state.scanline_program_wait_index));
        } else {
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM2,
                        pio_encode_jmp(shared_state.scanline_program_wait_index + 1));
        }
    }
    #endif
#endif
#if PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
#if PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA
    dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL)->al3_transfer_count = fsb->core.fragment_words;
#endif
    //dma_channel_transfer_from_buffer_now(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL, (uintptr_t)fsb->core.data, (uint32_t) fsb->core.data_used);
    dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL)->al3_read_addr_trig = (uintptr_t)fsb->core.data;
#else
    dma_channel_transfer_from_buffer_now(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL, fsb->core.data,
                                         (uint32_t) fsb->core.data_used);
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 1
#if PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA
    dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2)->al3_read_addr_trig = (uintptr_t)fsb->core.data2;
#else
    dma_channel_transfer_from_buffer_now(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2, fsb->core.data2, (uint32_t) fsb->core.data2_used);
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    dma_channel_transfer_from_buffer_now(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3, fsb->core.data3, (uint32_t) fsb->core.data3_used);
//    scanline_assert(video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM3].addr == video_24mhz_composable_offset_end_of_scanline_ALIGN);
#endif
//    scanline_assert(video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM2].addr == video_24mhz_composable_offset_end_of_scanline_ALIGN);
#endif
//    scanline_assert(video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM].addr == video_24mhz_composable_offset_end_of_scanline_ALIGN);
//    DEBUG_PINS_CLR(video_irq, 2);

    save = spin_lock_blocking(shared_state.scanline.lock);
    DEBUG_PINS_SET(video_timing, 1);
    shared_state.scanline.in_vblank = false;
    bool was_correct_scanline = (fsb != &_missing_scanline_buffer);
    bool free_scanline = false;
    shared_state.scanline.y_repeat_index += video_mode.yscale_denominator;
    if (shared_state.scanline.y_repeat_index >= shared_state.scanline.y_repeat_target) {
        // pick up a new scanline next time around if we had the right one
        if (was_correct_scanline) {
            free_scanline = true;
        }

        shared_state.scanline.y_repeat_index -= shared_state.scanline.y_repeat_target;
        set_next_scanline_id(scanline_id_after(shared_state.scanline.next_scanline_id));
        shared_state.scanline.current_scanline_buffer = NULL;
    } else if (!was_correct_scanline) {
        // not at the the end of yscale, but the wrong (or missing) scanline anyway, so clear that
        shared_state.scanline.current_scanline_buffer = NULL;
#if PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS
    } else if (fsb->core.link_after && !--fsb->core.link_after) {
        assert(fsb->core.link);
        spin_lock_unsafe_blocking(shared_state.in_use.lock);
        DEBUG_PINS_SET(video_link, 1);
        full_scanline_buffer_t *fsb2 = (full_scanline_buffer_t *)fsb->core.link;
        fsb->core.link = NULL; // the linkee scanline is now tracked on its own, so shouldn't be freed with the linker
        // we need to insert after the current item in the list, which is after fsb
        fsb2->core.scanline_id = fsb->core.scanline_id;
        fsb2->next = fsb->next;
        fsb->next = fsb2;
        if (fsb == shared_state.in_use.in_use_ascending_scanline_id_list_tail) {
            shared_state.in_use.in_use_ascending_scanline_id_list_tail = fsb2;
        }
        DEBUG_PINS_CLR(video_link, 1);
        spin_unlock_unsafe(shared_state.in_use.lock);
        shared_state.scanline.current_scanline_buffer = fsb2;
        free_scanline = true;
#endif
    }
    // safe to nest dma lock we never nest the other way
    spin_lock_unsafe_blocking(shared_state.dma.lock);
    shared_state.dma.scanline_in_progress = 1;
    if (free_scanline) {
        scanline_assert(!shared_state.dma.buffers_to_release);
        shared_state.dma.buffers_to_release++;
    }
    spin_unlock_unsafe(shared_state.dma.lock);
    DEBUG_PINS_CLR(video_timing, 1);
    spin_unlock(shared_state.scanline.lock, save);

    // because IRQs are enabled, we may obviously be pre-empted before or between either of these
    release_scanline_irqs_enabled(buffers_to_free_count, &local_free_list);
    free_local_free_list_irqs_enabled(local_free_list);
}

static void __video_time_critical_func(prepare_for_vblank_scanline_irqs_enabled)() {
    bool signal = false;

    // To simplify logic below, clean up any active scanlines now

    // note we only really need to do this on the first time in vsync, however as a defensive (potential recovery)
    // move we do it every time as it is cheap if nothing to do.
    int buffers_to_free_count = 0;
    update_dma_transfer_state_irqs_enabled(true, &buffers_to_free_count);

    uint32_t save = spin_lock_blocking(shared_state.scanline.lock);
    DEBUG_PINS_SET(video_timing, 1);
    full_scanline_buffer_t *local_free_list = NULL;

    if (!shared_state.scanline.in_vblank) {
        shared_state.scanline.in_vblank = true;
        shared_state.scanline.y_repeat_index = 0;

        // generally this should already have wrapped, but may not have just after a sync
        if (scanvideo_scanline_number(shared_state.scanline.next_scanline_id) != 0) {
            // set up for scanline 0 of the next frame when we come out of vblank
            shared_state.scanline.next_scanline_id =
                    (scanvideo_frame_number(shared_state.scanline.next_scanline_id) + 1u) << 16u;
            shared_state.scanline.y_repeat_target = _scanline_repeat_count_fn(shared_state.scanline.next_scanline_id);
        }


        signal = true;
    }

    if (!shared_state.scanline.current_scanline_buffer || is_scanline_after(shared_state.scanline.next_scanline_id,
                                                                            shared_state.scanline.current_scanline_buffer->core.scanline_id)) {
        // if we had a scanline buffer still (which was in the past, unset it and make sure it will be freed
        // before we attempt to relatch which only does something when csb == NULL)
        if (shared_state.scanline.current_scanline_buffer) {
            buffers_to_free_count++; // make sure it gets removed from in_use list
            shared_state.scanline.current_scanline_buffer = NULL;
        }
        // this will probably succeed, because we are buffering ahead of the actual beam position
        scanline_locked_try_latch_fsb_if_null_irqs_disabled(&local_free_list);
    }

    DEBUG_PINS_CLR(video_timing, 1);
    spin_unlock(shared_state.scanline.lock, save);

    // because IRQs are enabled, we may obviously be pre-empted before or between either of these
    release_scanline_irqs_enabled(buffers_to_free_count, &local_free_list);

    free_local_free_list_irqs_enabled(local_free_list);

    if (signal) {
        sem_release(&vblank_begin);
    }
}

#define setup_dma_states_vblank() if (true) { dma_states[0] = timing_state.a_vblank; dma_states[1] = timing_state.b1; dma_states[2] = timing_state.b2; dma_states[3] = timing_state.c_vblank; } else __builtin_unreachable()
#define setup_dma_states_no_vblank() if (true) { dma_states[0] = timing_state.a; dma_states[1] = timing_state.b1; dma_states[2] = timing_state.b2; dma_states[3] = timing_state.c; } else __builtin_unreachable()

static inline void top_up_timing_pio_fifo() {
    // todo better irq reset ... we are seeing irq get set again, handled in this loop, then we re-enter here when we don't need to
    // keep filling until SM3 TX is full
    while (!(video_pio->fstat & (1u << (PICO_SCANVIDEO_TIMING_SM + PIO_FSTAT_TXFULL_LSB)))) {
        DEBUG_PINS_XOR(video_irq, 1);
        DEBUG_PINS_XOR(video_irq, 1);
        pio_sm_put(video_pio, PICO_SCANVIDEO_TIMING_SM, dma_states[timing_state.dma_state_index] | timing_state.vsync_bits);
        // todo simplify this now we have a1, a2, b, c
        // todo display enable (only goes positive on start of screen)

        // todo right now we are fixed... make this generic for timing and improve
        if (++timing_state.dma_state_index >= DMA_STATE_COUNT) {
            timing_state.dma_state_index = 0;
            timing_state.timing_scanline++;

            // todo check code and put these in a current state struct
            if (timing_state.timing_scanline >= timing_state.v_active) {
                if (timing_state.timing_scanline >= timing_state.v_total) {
                    timing_state.timing_scanline = 0;
                    // active display - gives irq 0 and irq 4
                    setup_dma_states_no_vblank();
                } else if (timing_state.timing_scanline <= timing_state.v_pulse_end) {
                    if (timing_state.timing_scanline == timing_state.v_active) {
                        setup_dma_states_vblank();
                    } else if (timing_state.timing_scanline == timing_state.v_pulse_start) {
                        timing_state.vsync_bits = timing_state.vsync_bits_pulse;
                    } else if (timing_state.timing_scanline == timing_state.v_pulse_end) {
                        timing_state.vsync_bits = timing_state.vsync_bits_no_pulse;
                    }
                }
            }
        }
    }
}

void __isr __video_most_time_critical_func(isr_pio0_0)() {
#if PICO_SCANVIDEO_ADJUST_BUS_PRIORITY
    bus_ctrl_hw->priority = VIDEO_ADJUST_BUS_PRIORITY_VAL;
#endif

    // handler for explicit PIO_IRQ0 from PICO_SCANVIDEO_TIMING_SM at a good time to start a DMA for a scanline
    // this called once per scanline during non vblank
    if (video_pio->irq & 1u) {
        video_pio->irq = 1;
        DEBUG_PINS_SET(video_irq, 1);
        if (display_enabled) {
            prepare_for_active_scanline_irqs_enabled();
        }
        DEBUG_PINS_CLR(video_irq, 1);
    }
#if PICO_SCANVIDEO_ADJUST_BUS_PRIORITY
    bus_ctrl_hw->priority = 0;
#endif
    // handler for explicit PIO_IRQ1 from PICO_SCANVIDEO_TIMING_SM at a good time to prepare for a scanline
    // this is only called once per scanline during vblank
    if (video_pio->irq & 2u) {
//        video_pio->irq = 2;
        video_pio->irq = 3; // we clear irq1 for good measure, in case we had gotten out of sync
        DEBUG_PINS_SET(video_irq, 2);
        prepare_for_vblank_scanline_irqs_enabled();
        DEBUG_PINS_CLR(video_irq, 2);
    }
}

// irq for PIO FIFO
void __isr __video_most_time_critical_func(isr_pio0_1)() {
    DEBUG_PINS_SET(video_irq, 4);
    top_up_timing_pio_fifo();
    DEBUG_PINS_CLR(video_irq, 4);
}

#if !PICO_SCANVIDEO_NO_DMA_TRACKING

// DMA complete
void __isr __video_time_critical_func(isr_dma_0)() {
#if PICO_SCANVIDEO_ADJUST_BUS_PRIORITY
    bus_ctrl_hw->priority = VIDEO_ADJUST_BUS_PRIORITY_VAL;
#endif
    DEBUG_PINS_SET(video_irq, 4);
    scanline_dma_complete_irqs_enabled();
    DEBUG_PINS_CLR(video_irq, 4);
#if PICO_SCANVIDEO_ADJUST_BUS_PRIORITY
    bus_ctrl_hw->priority = 0;
#endif
}

#endif

static inline bool is_scanline_sm(int sm) {
#if PICO_SCANVIDEO_PLANE_COUNT > 1
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    return sm == PICO_SCANVIDEO_SCANLINE_SM || sm == PICO_SCANVIDEO_SCANLINE_SM2 || sm == PICO_SCANVIDEO_SCANLINE_SM3;
#else
    return sm == PICO_SCANVIDEO_SCANLINE_SM || sm == PICO_SCANVIDEO_SCANLINE_SM2;
#endif
#else
    return sm == PICO_SCANVIDEO_SCANLINE_SM;
#endif
}

void setup_sm(int sm, uint offset) {
#ifndef NDEBUG
    printf("Setting up SM %d\n", sm);
#endif

    pio_sm_config config = is_scanline_sm(sm) ? video_mode.pio_program->configure_pio(video_pio, sm, offset) :
                           video_htiming_program_get_default_config(offset);

#if PICO_SCANVIDEO_ENABLE_VIDEO_CLOCK_DOWN
    sm_config_set_clkdiv_int_frac(&config, video_clock_down_times_2 / 2, (video_clock_down_times_2 & 1u) << 7u);
#endif

    if (!is_scanline_sm(sm)) {
        // enable auto-pull
        sm_config_set_out_shift(&config, true, true, 32);
        const uint BASE = PICO_SCANVIDEO_SYNC_PIN_BASE; // hsync and vsync are +0 and +1, clock is +2
        uint pin_count;
#if PICO_SCANVIDEO_ENABLE_DEN_PIN
        pin_count = 3;
        // 3 OUT pins and maybe 1 sideset pin following them
#else
        // 2 OUT pins and 1 sideset pin following them
        pin_count = 2;
#endif
        sm_config_set_out_pins(&config, BASE, pin_count);
#if PICO_SCANVIDEO_ENABLE_DEN_PIN
        // side set pin as well
        sm_config_set_sideset_pins(&config, BASE + pin_count);
        pin_count++;
#endif
        pio_sm_set_consecutive_pindirs(video_pio, sm, BASE, pin_count, true);
    }

    pio_sm_init(video_pio, sm, offset, &config); // now paused
}

scanvideo_mode_t scanvideo_get_mode() {
    return video_mode;
}

extern uint32_t scanvideo_get_next_scanline_id() {
    return *(volatile uint32_t *) &shared_state.scanline.next_scanline_id;
}

extern bool scanvideo_in_hblank() {
    // this is a close estimate
    //    return !*(volatile bool *) &shared_state.dma.scanline_in_progress;

    // better we can see if the PIO is waiting on IRQ 4 (which it is almost all of the time it isn't drawing pixels)
    //
    // note that currently we require even custom PIO scanline programs to use WAIT IRQ 4 to sync with start of scanline
    return video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM].instr == PIO_WAIT_IRQ4;
}

extern bool scanvideo_in_vblank() {
    return *(volatile bool *) &shared_state.scanline.in_vblank;
}


static uint default_scanvideo_scanline_repeat_count_fn(uint32_t scanline_id) {
    return 1;
}

extern scanvideo_scanline_buffer_t *__video_time_critical_func(scanvideo_begin_scanline_generation)(
        bool block) {
    full_scanline_buffer_t *fsb;

    DEBUG_PINS_SET(video_link, 1);
    DEBUG_PINS_SET(video_generation, 1);
    do {
        uint32_t save = spin_lock_blocking(shared_state.free_list.lock);
//        DEBUG_PINS_SET(video_timing, 4);
        fsb = list_remove_head(&shared_state.free_list.free_list);
//        DEBUG_PINS_CLR(video_timing, 4);
        spin_unlock(shared_state.free_list.lock, save);

        if (fsb) {
            save = spin_lock_blocking(shared_state.scanline.lock);
            DEBUG_PINS_SET(video_timing, 1);
#if PICO_SCANVIDEO_ENABLE_SCANLINE_ASSERTIONS
            list_prepend(&shared_state.scanline.generating_list, fsb);
#endif
            // todo improve this algorithm... how far ahead should we be
            // todo i.e. should we skip ahead a bit further if we are perpetually behind - doesn't really help because we'd
            // todo be skipping some scanlines anyway; doesn't really matter which ones at that point
            uint32_t scanline_id = shared_state.scanline.next_scanline_id;

            if (!is_scanline_after(scanline_id, shared_state.scanline.last_scanline_id)) {
                // we are buffering ahead of the display
                scanline_id = scanline_id_after(shared_state.scanline.last_scanline_id);
            }

            fsb->core.scanline_id = shared_state.scanline.last_scanline_id = scanline_id;
            DEBUG_PINS_CLR(video_timing, 1);
            spin_unlock(shared_state.scanline.lock, save);
            break;
        }

        if (block) {
            __wfe();
        }
    } while (block);

    DEBUG_PINS_CLR(video_link, 1);
    DEBUG_PINS_CLR(video_generation, 1);
#if PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS
    fsb->core.link_after = 0;
#endif
    return (scanvideo_scanline_buffer_t *) fsb;
}

// todo remove this in favor of using scanvideo_begin_scanline_generation_link
scanvideo_scanline_buffer_t *__video_time_critical_func(scanvideo_begin_scanline_generation2)(scanvideo_scanline_buffer_t **second, bool block)
{
    full_scanline_buffer_t *fsb;

    DEBUG_PINS_SET(video_generation, 1);
    do
    {
        uint32_t save = spin_lock_blocking(shared_state.free_list.lock);
        DEBUG_PINS_SET(video_timing, 4);
        fsb = shared_state.free_list.free_list;
        full_scanline_buffer_t *fsb2;
        if (fsb) {
            fsb2 = fsb->next;
            if (fsb2) {
                // unlink both
                shared_state.free_list.free_list = fsb->next->next;
                fsb->next = NULL;
                fsb2->next = NULL;
            } else {
                fsb = NULL;
            }
        } else {
            fsb = NULL;
        }
        DEBUG_PINS_CLR(video_timing, 4);
        spin_unlock(shared_state.free_list.lock, save);
        if (fsb)
        {
            save = spin_lock_blocking(shared_state.scanline.lock);
            DEBUG_PINS_SET(video_timing, 1);
#ifdef ENABLE_SCANLINE_ASSERTIONS
            list_prepend(&shared_state.scanline.generating_list, fsb);
            list_prepend(&shared_state.scanline.generating_list, fsb2);
#endif
            // todo improve this algorithm... how far ahead should we be
            // todo i.e. should we skip ahead a bit further if we are perpetually behind - doesn't really help because we'd
            // todo be skipping some scanlines anyway; doesn't really matter which ones at that point
            uint32_t scanline_id = shared_state.scanline.next_scanline_id;

            if (!is_scanline_after(scanline_id, shared_state.scanline.last_scanline_id))
            {
                // we are buffering ahead of the display
                scanline_id = scanline_id_after(shared_state.scanline.last_scanline_id);
            }

            fsb->core.scanline_id = scanline_id;
            scanline_id = scanline_id_after(scanline_id);
            fsb2->core.scanline_id = shared_state.scanline.last_scanline_id = scanline_id;
            DEBUG_PINS_CLR(video_timing, 1);
            *second = &fsb2->core;
            spin_unlock(shared_state.scanline.lock, save);
            break;
        }

        if (block)
        {
            DEBUG_PINS_SET(video_generation, 4);
            __wfe();
            DEBUG_PINS_CLR(video_generation, 4);
        }
    }
    while (block);

    DEBUG_PINS_CLR(video_generation, 1);
    return (scanvideo_scanline_buffer_t *) fsb;
}

#if PICO_SCANVIDEO_LINKED_SCANLINE_BUFFERS
scanvideo_scanline_buffer_t *__video_time_critical_func(scanvideo_begin_scanline_generation_linked)(uint n,
        bool block) {
    full_scanline_buffer_t *fsb;

    DEBUG_PINS_SET(video_link, 1);
    DEBUG_PINS_SET(video_generation, 1);
    do {
        uint32_t save = spin_lock_blocking(shared_state.free_list.lock);
//        DEBUG_PINS_SET(video_timing, 4);
        fsb = shared_state.free_list.free_list;
        if (fsb) {
            full_scanline_buffer_t *fsb_tail = fsb;
            for(uint i=1; i<n && fsb_tail; i++) {
                fsb_tail = fsb_tail->next;
            }
            if (fsb_tail) {
                shared_state.free_list.free_list = fsb_tail->next;
                fsb_tail->next = NULL;
            } else {
                fsb = NULL;
            }
        }
//        DEBUG_PINS_CLR(video_timing, 4);
        spin_unlock(shared_state.free_list.lock, save);

        if (fsb) {
            full_scanline_buffer_t *fsb_tail = fsb;
            while (fsb_tail) {
                full_scanline_buffer_t *fsb_next = fsb_tail->next;
                fsb_tail->core.link = fsb_next ? &fsb_next->core : NULL;
                fsb_tail->next = NULL;
                fsb_tail = fsb_next;
            }
            save = spin_lock_blocking(shared_state.scanline.lock);
            DEBUG_PINS_SET(video_timing, 1);
#if PICO_SCANVIDEO_ENABLE_SCANLINE_ASSERTIONS
            list_prepend(&shared_state.scanline.generating_list, fsb);
#endif
            // todo improve this algorithm... how far ahead should we be
            // todo i.e. should we skip ahead a bit further if we are perpetually behind - doesn't really help because we'd
            // todo be skipping some scanlines anyway; doesn't really matter which ones at that point
            uint32_t scanline_id = shared_state.scanline.next_scanline_id;

            if (!is_scanline_after(scanline_id, shared_state.scanline.last_scanline_id)) {
                // we are buffering ahead of the display
                scanline_id = scanline_id_after(shared_state.scanline.last_scanline_id);
            }

            shared_state.scanline.last_scanline_id = scanline_id;
            scanvideo_scanline_buffer_t *fsb_core = &fsb->core;
            while (fsb_core) {
                fsb_core->scanline_id = scanline_id;
                fsb_core = fsb_core->link;
            }
            DEBUG_PINS_CLR(video_timing, 1);
            spin_unlock(shared_state.scanline.lock, save);
            break;
        }

        if (block) {
            __wfe();
        }
    } while (block);

    DEBUG_PINS_CLR(video_link, 1);
    DEBUG_PINS_CLR(video_generation, 1);
    fsb->core.link_after = 0;
    return (scanvideo_scanline_buffer_t *) fsb;
}
#endif

extern void __video_time_critical_func(scanvideo_end_scanline_generation)(
        scanvideo_scanline_buffer_t *scanline_buffer) {
    DEBUG_PINS_SET(video_generation, 2);
    full_scanline_buffer_t *fsb = (full_scanline_buffer_t *) scanline_buffer;
    uint32_t save = spin_lock_blocking(shared_state.scanline.lock);
#if PICO_SCANVIDEO_ENABLE_SCANLINE_ASSERTIONS
    list_remove(&shared_state.scanline.generating_list, fsb);
#endif
    list_insert_ascending(&shared_state.scanline.generated_ascending_scanline_id_list,
                          &shared_state.scanline.generated_ascending_scanline_id_list_tail, fsb);
    spin_unlock(shared_state.scanline.lock, save);
    DEBUG_PINS_CLR(video_generation, 2);
}

//#pragma GCC pop_options

void scanvideo_set_scanline_repeat_fn(scanvideo_scanline_repeat_count_fn fn) {
    _scanline_repeat_count_fn = fn ? fn : default_scanvideo_scanline_repeat_count_fn;
}

bool scanvideo_setup(const scanvideo_mode_t *mode) {
    return scanvideo_setup_with_timing(mode, mode->default_timing);
}

static pio_program_t copy_program(const pio_program_t *program, uint16_t *instructions,
                                       uint32_t max_instructions) {
    assert(max_instructions >= program->length);
    pio_program_t copy = *program;
    __builtin_memcpy(instructions, program->instructions, MIN(program->length, max_instructions) * sizeof(uint16_t));
    copy.instructions = instructions;
    return copy;
}

bool scanvideo_setup_with_timing(const scanvideo_mode_t *mode, const scanvideo_timing_t *timing) {
    __builtin_memset(&shared_state, 0, sizeof(shared_state));
    // init non zero members
    // todo pass scanline buffers and size, or allow client to allocate
    shared_state.scanline.lock = spin_lock_init(PICO_SPINLOCK_ID_VIDEO_SCANLINE_LOCK);
    shared_state.dma.lock = spin_lock_init(PICO_SPINLOCK_ID_VIDEO_DMA_LOCK);
    shared_state.free_list.lock = spin_lock_init(PICO_SPINLOCK_ID_VIDEO_FREE_LIST_LOCK);
    shared_state.in_use.lock = spin_lock_init(PICO_SPINLOCK_ID_VIDEO_IN_USE_LOCK);
    shared_state.scanline.last_scanline_id = 0xffffffff;

    video_mode = *mode;
    video_mode.default_timing = timing;

    static_assert(BPP == 16, ""); // can't do 8 bit now because of pixel count
    // this is no longer necessary
    //assert(!(mode->width & 1));
    if (!video_mode.yscale_denominator) video_mode.yscale_denominator = 1;
    // todo is this still necessary?
    //invalid_params_if(SCANVIDEO_DPI, (timing->v_active % mode->yscale));
    ((uint16_t *) (_missing_scanline_data))[2] = mode->width / 2 - 3;
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
    variable_fragment_missing_scanline_data_chain[1] = native_safe_hw_ptr(_missing_scanline_data);
#endif
#if PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE2_FIXED_FRAGMENT_DMA
    fixed_fragment_missing_scanline_data_chain[0] = native_safe_hw_ptr(_missing_scanline_data);
#endif

    sem_init(&vblank_begin, 0, 1);

    for (int i = 0; i < PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT; i++) {
        mem_buffer_t b;
        pico_buffer_alloc_in_place(&b, PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS * sizeof(uint32_t));
        scanline_buffers[i].core.data = (uint32_t *)b.bytes;// calloc(PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS, sizeof(uint32_t));
        scanline_buffers[i].core.data_max = PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS;
#if PICO_SCANVIDEO_PLANE_COUNT > 1
        scanline_buffers[i].core.data2 = (uint32_t *) calloc(PICO_SCANVIDEO_MAX_SCANLINE_BUFFER2_WORDS, sizeof(uint32_t));
        scanline_buffers[i].core.data2_max = PICO_SCANVIDEO_MAX_SCANLINE_BUFFER2_WORDS;
#if PICO_SCANVIDEO_PLANE_COUNT > 2
        scanline_buffers[i].core.data3 = (uint32_t *) calloc(PICO_SCANVIDEO_MAX_SCANLINE_BUFFER3_WORDS, sizeof(uint32_t));
        scanline_buffers[i].core.data3_max = PICO_SCANVIDEO_MAX_SCANLINE_BUFFER3_WORDS;
#endif
#endif
        scanline_buffers[i].next = i != PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT - 1 ? &scanline_buffers[i + 1] : NULL;
    }

    shared_state.free_list.free_list = &scanline_buffers[0];
    // shared state init complete - probably overkill
    __mem_fence_release();

    uint pin_mask = 3u << PICO_SCANVIDEO_SYNC_PIN_BASE;
    bi_decl_if_func_used(bi_2pins_with_names(PICO_SCANVIDEO_SYNC_PIN_BASE, "HSync",
                                               PICO_SCANVIDEO_SYNC_PIN_BASE + 1, "VSync"));

#if PICO_SCANVIDEO_ENABLE_DEN_PIN
    bi_decl_if_func_used(bi_1pin_with_name(PICO_SCANVIDEO_SYNC_PIN_BASE + 2, "Display Enable"));
    pin_mask |= 4u << PICO_SCANVIDEO_SYNC_PIN_BASE;
#endif
#if PICO_SCANVIDEO_ENABLE_CLOCK_PIN
    bi_decl_if_func_used(bi_1pin_with_name(PICO_SCANVIDEO_SYNC_PIN_BASE + 3, "Pixel Clock"));
    pin_mask |= 8u << PICO_SCANVIDEO_SYNC_PIN_BASE;
#endif
    static_assert(PICO_SCANVIDEO_PIXEL_RSHIFT + PICO_SCANVIDEO_PIXEL_RCOUNT <= PICO_SCANVIDEO_COLOR_PIN_COUNT, "red bits do not fit in color pins");
    static_assert(PICO_SCANVIDEO_PIXEL_GSHIFT + PICO_SCANVIDEO_PIXEL_GCOUNT <= PICO_SCANVIDEO_COLOR_PIN_COUNT, "green bits do not fit in color pins");
    static_assert(PICO_SCANVIDEO_PIXEL_BSHIFT + PICO_SCANVIDEO_PIXEL_BCOUNT <= PICO_SCANVIDEO_COLOR_PIN_COUNT, "blue bits do not fit in color pins");
#define RMASK ((1u << PICO_SCANVIDEO_PIXEL_RCOUNT) - 1u)
#define GMASK ((1u << PICO_SCANVIDEO_PIXEL_GCOUNT) - 1u)
#define BMASK ((1u << PICO_SCANVIDEO_PIXEL_BCOUNT) - 1u)
    pin_mask |= RMASK << (PICO_SCANVIDEO_COLOR_PIN_BASE + PICO_SCANVIDEO_PIXEL_RSHIFT);
    pin_mask |= GMASK << (PICO_SCANVIDEO_COLOR_PIN_BASE + PICO_SCANVIDEO_PIXEL_GSHIFT);
    pin_mask |= BMASK << (PICO_SCANVIDEO_COLOR_PIN_BASE + PICO_SCANVIDEO_PIXEL_BSHIFT);
    bi_decl_if_func_used(bi_pin_mask_with_name(RMASK << (PICO_SCANVIDEO_COLOR_PIN_BASE + PICO_SCANVIDEO_PIXEL_RSHIFT), RMASK == 1 ? "Red" : ("Red 0-" __XSTRING(PICO_SCANVIDEO_PIXEL_GCOUNT))));
    bi_decl_if_func_used(bi_pin_mask_with_name(GMASK << (PICO_SCANVIDEO_COLOR_PIN_BASE + PICO_SCANVIDEO_PIXEL_GSHIFT), GMASK == 1 ? "Green" : ("Green 0-" __XSTRING(PICO_SCANVIDEO_PIXEL_GCOUNT))));
    bi_decl_if_func_used(bi_pin_mask_with_name(BMASK << (PICO_SCANVIDEO_COLOR_PIN_BASE + PICO_SCANVIDEO_PIXEL_BSHIFT), BMASK == 1 ? "Blue" : ("Blue 0-" __XSTRING(PICO_SCANVIDEO_PIXEL_BCOUNT))));

    for(uint8_t i = 0; pin_mask; i++, pin_mask>>=1u) {
        if (pin_mask & 1) gpio_set_function(i, GPIO_FUNC_PIO0);
    }

#if !PICO_SCANVIDEO_ENABLE_VIDEO_CLOCK_DOWN
    valid_params_if(SCANVIDEO_DPI, timing->clock_freq == video_clock_freq);
#else
    uint sys_clk = clock_get_hz(clk_sys);
    video_clock_down_times_2 = sys_clk / timing->clock_freq;
#if PICO_SCANVIDEO_ENABLE_CLOCK_PIN
    if (video_clock_down_times_2 * timing->clock_freq != sys_clk) {
        panic("System clock (%d) must be an integer multiple of 2 times the requested pixel clock (%d).", sys_clk, timing->clock_freq);
    }
#else
    if (video_clock_down_times_2 * timing->clock_freq != sys_clk) {
        panic("System clock (%d) must be an integer multiple of the requested pixel clock (%d).", sys_clk, timing->clock_freq);
    }
#endif
#endif

    valid_params_if(SCANVIDEO_DPI, mode->width * mode->xscale <= timing->h_active);
    valid_params_if(SCANVIDEO_DPI, mode->height * mode->yscale <= timing->v_active * video_mode.yscale_denominator);

    uint16_t instructions[32];
    pio_program_t modified_program = copy_program(mode->pio_program->program, instructions,
                                                       count_of(instructions));

    if (!mode->pio_program->adapt_for_mode(mode->pio_program, mode, &_missing_scanline_buffer.core, instructions)) {
        valid_params_if(SCANVIDEO_DPI, false);
    }
    valid_params_if(SCANVIDEO_DPI, _missing_scanline_buffer.core.data && _missing_scanline_buffer.core.data_used);
    video_program_load_offset = pio_add_program(video_pio, &modified_program);

#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
    int program_wait_index = -1;
#endif
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY || PARAM_ASSERTIONS_ENABLED(SCANVIDEO_DBI)
    for (int i = 0; i < mode->pio_program->program->length; i++) {
        if (instructions[i] == PIO_WAIT_IRQ4) {
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
            valid_params_if(SCANVIDEO_DPI, program_wait_index == -1);
            program_wait_index = i;
#endif
        }
    }
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
    valid_params_if(SCANVIDEO_DPI, program_wait_index != -1);
    shared_state.scanline_program_wait_index = program_wait_index;
#endif
#endif

#if PICO_SCANVIDEO_PLANE_COUNT > 1
    valid_params_if(SCANVIDEO_DPI,_missing_scanline_buffer.core.data2 && _missing_scanline_buffer.core.data2_used);
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    valid_params_if(SCANVIDEO_DPI,_missing_scanline_buffer.core.data3 && _missing_scanline_buffer.core.data3_used);
#endif
#endif
    _missing_scanline_buffer.core.status = SCANLINE_OK;

    setup_sm(PICO_SCANVIDEO_SCANLINE_SM, video_program_load_offset);
#if PICO_SCANVIDEO_PLANE_COUNT > 1
    setup_sm(PICO_SCANVIDEO_SCANLINE_SM2, video_program_load_offset);
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    setup_sm(PICO_SCANVIDEO_SCANLINE_SM3, video_program_load_offset);
#endif
#endif

    uint32_t side_set_xor = 0;
    modified_program = copy_program(&video_htiming_program, instructions, count_of(instructions));

    if (timing->clock_polarity) {
        side_set_xor = 0x1000; // flip the top side set bit

        for (uint i = 0; i < video_htiming_program.length; i++) {
            instructions[i] ^= side_set_xor;
        }
    }

    video_htiming_load_offset = pio_add_program(video_pio, &modified_program);

    setup_sm(PICO_SCANVIDEO_TIMING_SM, video_htiming_load_offset);

    // make this highest priority
#if PICO_DEFAULT_IRQ_PRIORITY < 0x40
#warning pico_scanvideo_dpi may not always function correctly without PIO_IRQ_0 at a higher priority than other interrupts.
    irq_set_priority(PIO0_IRQ_1, 0x40); // lower priority by 1
    irq_set_priority(DMA_IRQ_0, 0x80); // lower priority by 2
#else
    irq_set_priority(PIO0_IRQ_0, 0); // highest priority
    irq_set_priority(PIO0_IRQ_1, 0x40); // lower priority by 1
#endif

    dma_claim_mask(PICO_SCANVIDEO_SCANLINE_DMA_CHANNELS_MASK);
    dma_set_irq0_channel_mask_enabled(PICO_SCANVIDEO_SCANLINE_DMA_CHANNELS_MASK, true);

    // todo reset DMA channels

    dma_channel_config channel_config = dma_channel_get_default_config(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL);
    channel_config_set_dreq(&channel_config, DREQ_PIO0_TX0 +
                                     PICO_SCANVIDEO_SCANLINE_SM);  // Select scanline dma dreq to be PICO_SCANVIDEO_SCANLINE_SM TX FIFO not full
#if PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
    channel_config_set_chain_to(&channel_config, PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL);
    channel_config_set_irq_quiet(&channel_config, true);
#endif
    dma_channel_configure(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL,
                          &channel_config,
                          &video_pio->txf[PICO_SCANVIDEO_SCANLINE_SM],
                          NULL, // set later
                          0, // set later
                          false);
#if PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
    dma_channel_config chain_config = dma_channel_get_default_config(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL);
    channel_config_set_write_increment(&chain_config, true);
    // configure write ring
    channel_config_set_ring(&chain_config, true,
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA
            3 // wrap the write at 8 bytes (so each transfer writes the same 2 word ctrl registers)
#else
            2 // wrap the write at 4 bytes (so each transfer writes the same ctrl register)
#endif
    );
    dma_channel_configure(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL,
                  &chain_config,
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA
                  &dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL)->al3_transfer_count,  // ch DMA config (target "ring" buffer size 8) - this is (transfer_count, read_addr trigger)
#else
                  &dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL)->al3_read_addr_trig,  // ch DMA config (target "ring" buffer size 4) - this is (read_addr trigger)
#endif
                  NULL, // set later
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA
                   2, // send 2 words to ctrl block of data chain per transfer
#else
                  1,
#endif
                  false);
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 1
    channel_config = dma_channel_get_default_config(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2);
    channel_config_set_dreq(&channel_config, DREQ_PIO0_TX0 + PICO_SCANVIDEO_SCANLINE_SM2);  // Select scanline dma dreq to be PICO_SCANVIDEO_SCANLINE_SM TX FIFO not full
#if PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA
    channel_config_set_chain_to(&channel_config, PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2);
    channel_config_set_irq_quiet(&channel_config, true);
#endif
    dma_channel_configure(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2,
                  &channel_config,
                  (void *)&video_pio->txf[PICO_SCANVIDEO_SCANLINE_SM2],
                  NULL, // set later
                  0, // set later
                  false);
#if PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA
    dma_channel_config chain_config2 = dma_channel_get_default_config(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2);
    channel_config_set_write_increment(&chain_config2, true);
    // configure write ring
    channel_config_set_ring(&chain_config2, true,
#if PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
            3 // wrap the write at 8 bytes (so each transfer writes the same 2 word ctrl registers)
#else
            2 // wrap the write at 4 bytes (so each transfer writes the same ctrl register)
#endif
    );
    dma_channel_configure(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2,
                  &chain_config2,
#if PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
                  &dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2)->al3_transfer_count,  // ch DMA config (target "ring" buffer size 8) - this is (transfer_count, read_addr trigger)
#else
                  &dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2)->al3_read_addr_trig,  // ch DMA config (target "ring" buffer size 4) - this is (read_addr trigger)
#endif
                  NULL, // set later
#if PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
                   2, // send 2 words to ctrl block of data chain per transfer
#else
                  1,
#endif
                  false);
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 2
#if PICO_SCANVIDEO_PLANE3_FRAGMENT_DMA
    static_assert(false);
#endif
    channel_config = dma_channel_get_default_config(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3);
    channel_config_set_dreq(&channel_config, DREQ_PIO0_TX0 + PICO_SCANVIDEO_SCANLINE_SM3);  // Select scanline dma dreq to be PICO_SCANVIDEO_SCANLINE_SM TX FIFO not full
    dma_channel_configure(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3,
                  &channel_config,
                  (void *)&video_pio->txf[PICO_SCANVIDEO_SCANLINE_SM3],
                  NULL, // set later
                  0, // set later
                  false);
#endif
#endif

    // clear scanline irq
    pio_sm_exec(video_pio, PICO_SCANVIDEO_TIMING_SM, video_htiming_states_program.instructions[CLEAR_IRQ_SCANLINE]);

    // todo there are probably some restrictions :-)
    //assert(timing->v_front_porch == 1);
    //assert(timing->v_pulse == 2);
    //assert(timing->v_total == 500);

    timing_state.v_total = timing->v_total;
    timing_state.v_active = timing->v_active;
    timing_state.v_pulse_start = timing->v_active + timing->v_front_porch;
    timing_state.v_pulse_end = timing_state.v_pulse_start + timing->v_pulse;
    const uint32_t vsync_bit = 0x40000000;
    timing_state.vsync_bits_pulse = timing->v_sync_polarity ? 0 : vsync_bit;
    timing_state.vsync_bits_no_pulse = timing->v_sync_polarity ? vsync_bit : 0;

    // these are read bitwise backwards (lsb to msb) by PIO pogram

    // we can probably do smaller
#define HTIMING_MIN 8

#define TIMING_CYCLE 3u
#define timing_encode(state, length, pins) ((video_htiming_states_program.instructions[state] ^ side_set_xor)| (((uint32_t)(length) - TIMING_CYCLE) << 16u) | ((uint32_t)(pins) << 29u))
#define A_CMD SET_IRQ_0
#define A_CMD_VBLANK SET_IRQ_1
#define B1_CMD CLEAR_IRQ_SCANLINE
#define B2_CMD CLEAR_IRQ_SCANLINE
#define C_CMD SET_IRQ_SCANLINE
#define C_CMD_VBLANK CLEAR_IRQ_SCANLINE

    int h_sync_bit = timing->h_sync_polarity ? 0 : 1;
    timing_state.a = timing_encode(A_CMD, 4, h_sync_bit);
    static_assert(HTIMING_MIN >= 4, "");
    timing_state.a_vblank = timing_encode(A_CMD_VBLANK, 4, h_sync_bit);
    int h_back_porch = timing->h_total - timing->h_front_porch - timing->h_pulse - timing->h_active;

    valid_params_if(SCANVIDEO_DPI, timing->h_pulse - 4 >= HTIMING_MIN);
    timing_state.b1 = timing_encode(B1_CMD, timing->h_pulse - 4, h_sync_bit);

    // todo decide on what these should be - we should really be asserting the timings
    //
    // todo note that the placement of the active scanline IRQ from the timing program is super important.
    //  if it gets moved too much (or indeed at all) it may be that there are problems with DMA/SM IRQ
    //  overlap, which may require the addition of a separate timing state for the prepare for scanline
    //  (separate from the needs of setting the hsync pulse)
    valid_params_if(SCANVIDEO_DPI, timing->h_active >= HTIMING_MIN);
    //assert(timing->h_front_porch >= HTIMING_MIN);
    valid_params_if(SCANVIDEO_DPI, h_back_porch >= HTIMING_MIN);
    valid_params_if(SCANVIDEO_DPI, (timing->h_total - h_back_porch - timing->h_pulse) >= HTIMING_MIN);
    timing_state.b2 = timing_encode(B2_CMD, h_back_porch, !h_sync_bit);
    timing_state.c = timing_encode(C_CMD, timing->h_total - h_back_porch - timing->h_pulse, 4 | !h_sync_bit);
    timing_state.c_vblank = timing_encode(C_CMD_VBLANK, timing->h_total - h_back_porch - timing->h_pulse, !h_sync_bit);

    // this is two scanlines in vblank
    setup_dma_states_vblank();
    timing_state.vsync_bits = timing_state.vsync_bits_no_pulse;
    scanvideo_set_scanline_repeat_fn(NULL);
    return true;
}

bool video_24mhz_composable_adapt_for_mode(const scanvideo_pio_program_t *program, const scanvideo_mode_t *mode,
                                           scanvideo_scanline_buffer_t *missing_scanline_buffer,
                                           uint16_t *modifiable_instructions) {
    int delay0 = 2 * mode->xscale - 2;
    int delay1 = delay0 + 1;
    valid_params_if(SCANVIDEO_DPI, delay0 <= 31);
    valid_params_if(SCANVIDEO_DPI, delay1 <= 31);

    // todo macro-ify this
    modifiable_instructions[video_24mhz_composable_program_extern(delay_a_1)] |= (unsigned) delay1 << 8u;
    modifiable_instructions[video_24mhz_composable_program_extern(delay_b_1)] |= (unsigned) delay1 << 8u;
    modifiable_instructions[video_24mhz_composable_program_extern(delay_c_0)] |= (unsigned) delay0 << 8u;
    modifiable_instructions[video_24mhz_composable_program_extern(delay_d_0)] |= (unsigned) delay0 << 8u;
    modifiable_instructions[video_24mhz_composable_program_extern(delay_e_0)] |= (unsigned) delay0 << 8u;
    modifiable_instructions[video_24mhz_composable_program_extern(delay_f_1)] |= (unsigned) delay1 << 8u;
#if !PICO_SCANVIDEO_USE_RAW1P_2CYCLE
    modifiable_instructions[video_24mhz_composable_program_extern(delay_g_0)] |= (unsigned) delay0 << 8u;
#else
    int delay_half = mode->xscale - 2;
    modifiable_instructions[video_24mhz_composable_program_extern(delay_g_0)] |= (unsigned)delay_half << 8u;
#endif
    modifiable_instructions[video_24mhz_composable_program_extern(delay_h_0)] |= (unsigned) delay0 << 8u;

#if !PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
    missing_scanline_buffer->data = _missing_scanline_data;
    missing_scanline_buffer->data_used = missing_scanline_buffer->data_max = sizeof(_missing_scanline_data) / 4;
#else
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA
    missing_scanline_buffer->data = variable_fragment_missing_scanline_data_chain;
    missing_scanline_buffer->data_used = missing_scanline_buffer->data_max = sizeof(variable_fragment_missing_scanline_data_chain) / 4;
#else
    missing_scanline_buffer->data = fixed_fragment_missing_scanline_data_chain;
    missing_scanline_buffer->data_used = missing_scanline_buffer->data_max = sizeof(fixed_fragment_missing_scanline_data_chain) / 4;
#endif
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 1
#if !PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA
    missing_scanline_buffer->data2 = missing_scanline_data_overlay;
    missing_scanline_buffer->data2_used = missing_scanline_buffer->data2_max = sizeof(missing_scanline_data_overlay) / 4;
#else
#if PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
    missing_scanline_buffer->data2 = variable_fragment_missing_scanline_data_chain;
    missing_scanline_buffer->data2_used = missing_scanline_buffer->data2_max = sizeof(variable_fragment_missing_scanline_data_chain) / 4;
#else
    missing_scanline_buffer->data2 = fixed_fragment_missing_scanline_data_chain;
    missing_scanline_buffer->data2_used = missing_scanline_buffer->data2_max = sizeof(fixed_fragment_missing_scanline_data_chain) / 4;
#endif
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    missing_scanline_buffer->data3 = missing_scanline_data_overlay;
    missing_scanline_buffer->data3_used = missing_scanline_buffer->data3_max = sizeof(missing_scanline_data_overlay) / 4;
#endif
#endif
    return true;
}

bool video_default_adapt_for_mode(const scanvideo_pio_program_t *program, const scanvideo_mode_t *mode,
                                  uint16_t *modifiable_instructions) {
    return true;
}

void scanvideo_default_configure_pio(pio_hw_t *pio, uint sm, uint offset, pio_sm_config *config, bool overlay) {
    pio_sm_set_consecutive_pindirs(pio, sm, PICO_SCANVIDEO_COLOR_PIN_BASE, PICO_SCANVIDEO_COLOR_PIN_COUNT, true);
    sm_config_set_out_pins(config, PICO_SCANVIDEO_COLOR_PIN_BASE, PICO_SCANVIDEO_COLOR_PIN_COUNT);
    sm_config_set_out_shift(config, true, true, 32); // autopull
    sm_config_set_fifo_join(config, PIO_FIFO_JOIN_TX);
    if (overlay) {
        sm_config_set_out_special(config, 1, 1, PICO_SCANVIDEO_ALPHA_PIN);
    } else {
        sm_config_set_out_special(config, 1, 0, 0);
    }
}

pio_sm_config video_24mhz_composable_configure_pio(pio_hw_t *pio, uint sm, uint offset) {
    pio_sm_config config = video_24mhz_composable_default_program_get_default_config(offset);
    scanvideo_default_configure_pio(pio, sm, offset, &config, sm != PICO_SCANVIDEO_SCANLINE_SM);
    return config;
}

void scanvideo_timing_enable(bool enable) {
    // todo we need to protect our state here... this can't be frame synced obviously (at least turning on)
    // todo but we should make sure we clear out state when we turn it off, and probably reset scanline counter when we turn it on
    if (enable != video_timing_enabled) {
        // todo should we disable these too? if not move to scanvideo_setup
        pio_set_irq0_source_mask_enabled(video_pio, (1u << pis_interrupt0) | (1u << pis_interrupt1), true);
        pio_set_irq1_source_enabled(video_pio, pis_sm0_tx_fifo_not_full + PICO_SCANVIDEO_TIMING_SM, true);
        irq_set_mask_enabled((1u << PIO0_IRQ_0)
                              | (1u << PIO0_IRQ_1)
                              #if !PICO_SCANVIDEO_NO_DMA_TRACKING
                              | (1u << DMA_IRQ_0)
#endif
                , enable);
        uint32_t sm_mask = (1u << PICO_SCANVIDEO_SCANLINE_SM) | 1u << PICO_SCANVIDEO_TIMING_SM;
#if PICO_SCANVIDEO_PLANE_COUNT > 1
        sm_mask |= 1u << PICO_SCANVIDEO_SCANLINE_SM2;
#if PICO_SCANVIDEO_PLANE_COUNT > 2
        sm_mask |= 1u << PICO_SCANVIDEO_SCANLINE_SM3;
#endif
#endif
        pio_claim_sm_mask(video_pio, sm_mask);
        pio_set_sm_mask_enabled(video_pio, sm_mask, false);
#if PICO_SCANVIDEO_ENABLE_VIDEO_CLOCK_DOWN
        pio_clkdiv_restart_sm_mask(video_pio, sm_mask);
#endif

        if (enable) {
            uint jmp = video_program_load_offset + pio_encode_jmp(video_mode.pio_program->entry_point);
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM, jmp);
#if PICO_SCANVIDEO_PLANE_COUNT > 1
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM2, jmp);
#if PICO_SCANVIDEO_PLANE_COUNT > 2
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM3, jmp);
#endif
#endif
            // todo we should offset the addresses for the SM
            pio_sm_exec(video_pio, PICO_SCANVIDEO_TIMING_SM,
                        pio_encode_jmp(video_htiming_load_offset + video_htiming_offset_entry_point));
            pio_set_sm_mask_enabled(video_pio, sm_mask, true);
        }
        video_timing_enabled = enable;
    }
}

uint32_t scanvideo_wait_for_scanline_complete(uint32_t scanline_id) {
    // next_scanline_id is potentially the scanline_id in progress, so we need next_scanline_id to
    // be more than the scanline_id after the passed one
    scanline_id = scanline_id_after(scanline_id);
    uint32_t frame = scanvideo_frame_number(scanline_id);
    uint32_t next_scanline_id;
    // scanline_id > scanvideo_get_next_scanline_id() but with wrapping support
    while (0 < (scanline_id - (next_scanline_id = scanvideo_get_next_scanline_id()))) {
        // we may end up waiting for the next scanline while in vblank; the one we are waiting for is clearly done
        if (scanvideo_in_vblank() && (scanvideo_frame_number(next_scanline_id) - frame) >= 1)
            break;
        assert(video_timing_enabled); // todo should we just return
        __wfe();
    }
    return next_scanline_id;
}

void scanvideo_wait_for_vblank() {
    sem_acquire_blocking(&vblank_begin);
}

#ifndef NDEBUG
// todo this is for composable only atm
void validate_scanline(const uint32_t *dma_data, uint dma_data_size,
                       uint max_pixels, uint expected_width) {
    const uint16_t *it = (uint16_t *)dma_data;
    assert(!(3u&(uintptr_t)dma_data));
    const uint16_t *const dma_data_end = (uint16_t *)(dma_data + dma_data_size);
    uint16_t *pixel_buffer = 0;
    const uint16_t *const pixels_end = (uint16_t *)(pixel_buffer + max_pixels);
    uint16_t *pixels = pixel_buffer;
    bool ok = false;
    bool done = false;
    bool had_black = false;
    do {
        uint16_t cmd = *it++;
        switch (cmd) {
            case video_24mhz_composable_program_extern(end_of_scanline_skip_ALIGN):
                it++;
                // fall thru
            case video_24mhz_composable_program_extern(end_of_scanline_ALIGN):
                done = ok = true;
                break;
            case video_24mhz_composable_program_extern(color_run):
            {
                it++;
                uint16_t len = *it++;
                for(int i=0; i<len+3; i++) {
                    assert(pixels < pixels_end);
                    pixels++;
                }
                break;
            }
            case video_24mhz_composable_program_extern(raw_run):
            {
                assert(pixels < pixels_end);
                pixels++; it++;
                uint16_t len = *it++;
                for(int i=0; i<len+2; i++) {
                    assert(pixels < pixels_end);
                    pixels++; it++;
                }
                break;
            }
            case video_24mhz_composable_program_extern(raw_2p):
                assert(pixels < pixels_end);
                pixels++; it++;
                // fall thru
            case video_24mhz_composable_program_extern(raw_1p):
                if (pixels == pixels_end) {
                    assert(!had_black);
                    uint c = *it++;
                    assert(!c); // must end with black
                    had_black = true;
                } else {
                    assert(pixels < pixels_end);
                    pixels++; it++;
                }
                break;
#if !PICO_SCANVIDEO_USE_RAW1P_2CYCLE
            case video_24mhz_composable_program_extern(raw_1p_skip_ALIGN):
                assert(pixels < pixels_end);
                pixels++; it++;
                break;
#else
            case video_24mhz_composable_program_extern(raw_1p_2cycle):
            {
                assert(pixels < pixels_end);
                uint c = *it++;
                had_black= !c;
                break;
            }
#endif
            default:
                assert(false);
                done = true;
        }
    } while (!done);
    assert(ok);
    assert(it == dma_data_end);
    assert(!(3u&(uintptr_t)(it))); // should end on dword boundary
    assert(!expected_width || pixels == pixel_buffer + expected_width); // with the correct number of pixels (one more because we stick a black pixel on the end)
    assert(had_black);
}
#endif

#pragma GCC pop_options
