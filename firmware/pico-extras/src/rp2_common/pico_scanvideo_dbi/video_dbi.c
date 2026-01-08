/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <stdlib.h>
#include "platform.h"
#include "debug.h"
#include "video.h"
#include "gpio.h"
#include "dma.h"
#include "dreq.h"
#include "pio.h"
#include "tft_driver.h"
#include "control.pio.h"

// todo add ability to shift scanline back a bit (we already have the timing, but it should be a post mode set adjustment)
//  we can use this to allow some initial work in the scanline b4 the first pixel (e.g. a dummy black pixel)

// todo bad state recovery
//      - stress test with pause/unpause
//      - bad state should cause SCANLINE_ASSERTION_ERROR
//      - possible orphaned in_use - perhaps clean up when error state is detected
//      - if PIO is not in the right place, pause/clear FIFO join-unjoin/jmp/resume
//      - dma may need to be cancelled
// todo dma chaining support

//#define ENABLE_VIDEO_CLOCK_DOWN

#define GO_AT_USER_SPEED

#pragma GCC push_options
//#ifdef __arm__
//#pragma GCC optimize("O3")
//#endif

// == CONFIG ============

#define PICO_SCANVIDEO_SCANLINE_SM 0u
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL 0u
#define TIMING_DMA_CHANNEL 6u

#if PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
#define PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL 3u
#endif
#define TIMING_SM 3u
#if PICO_SCANVIDEO_PLANE_COUNT > 1
#define PICO_SCANVIDEO_SCANLINE_SM2 1u
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2 1u
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
#if PICO_SCANVIDEO_PLANE_COUNT > 2
#fatal must have ENABLE_VIDEO_PLANE2 for ENABLE_VIDEO_PLANE3
#endif
#define PICO_SCANVIDEO_SCANLINE_DMA_CHANNELS_MASK (1u << PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL)
#endif

// == DEBUGGING =========

// note that this is very very important if you see things going wrong with the display,
// however beware, because it will also cause visual artifiacts if we are pushing the edge of the envelope
// since it itself uses cycles that are in short supply! This is why it is off by default
//
// todo note, it should eventually be difficult to get the display into a bad state (even
//  with things like runaway scanline program; incomplete DMA etc.. which currently break it).
//#define ENABLE_SCANLINE_ASSERTIONS

//#define PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY

CU_REGISTER_DEBUG_PINS(sequence, video_timing, video_dma_buffer, video_irq, video_dma_completion, video_generation, video_recovery
)

// ---- select at most one ---
//CU_SELECT_DEBUG_PINS(video_recovery)
//CU_SELECT_DEBUG_PINS(video_generation)
CU_SELECT_DEBUG_PINS(video_timing)
//CU_SELECT_DEBUG_PINS(video_irq)
//CU_SELECT_DEBUG_PINS(video_dma_buffer)
//CU_SELECT_DEBUG_PINS(video_dma_completion)
//CU_SELECT_DEBUG_PINS(sequence)

// ======================

// todo this needs to come from somwehere useful
#define DISABLE_VIDEO_ASSERTIONS

#ifndef DISABLE_VIDEO_ASSERTIONS
#define video_assert(x) assert(x)
#else
#define video_assert(x)  (void)0
#endif

#ifdef ENABLE_SCANLINE_ASSERTIONS
#define scanline_assert(x) assert(x)
#else
#define scanline_assert(x) (void)0
#endif

#define video_pio pio0

#define VIDEO_ADJUST_BUS_PRIORITY_VAL (BUSCTRL_BUS_PRIORITY_PROC0_BITS | BUSCTRL_BUS_PRIORITY_PROC1_BITS)

#ifdef VIDEO_MOST_TIME_CRITICAL_CODE_SECTION
#define __video_most_time_critical(x) __attribute__((section(__XSTRING(VIDEO_MOST_TIME_CRITICAL_CODE_SECTION) "." x)))
#else
#define __video_most_time_critical(x) __not_in_flash("video.mostcrit." x)
#endif

#ifdef VIDEO_TIME_CRITICAL_CODE_SECTION
#define __video_time_critical(x) __attribute__((section(__XSTRING(VIDEO_TIME_CRITICAL_CODE_SECTION) "." x)))
#else
#define __video_time_critical(x) __not_in_flash("video.crit." x)
#endif
// --- video_24mhz_composable ---

#define video_24mhz_composable_program __CONCAT(video_24mhz_composable_prefix, _program)
#define video_24mhz_composable_wrap_target __CONCAT(video_24mhz_composable_prefix, _wrap_target)
#define video_24mhz_composable_wrap __CONCAT(video_24mhz_composable_prefix, _wrap)

bool video_24mhz_composable_adapt_for_mode(const struct video_pio_program *program, const struct video_mode *mode,
                                           struct scanline_buffer *missing_scanline_buffer, uint16_t *buffer,
                                           uint buffer_max);
void video_24mhz_composable_configure_pio(pio_hw_t *pio, uint sm);

const struct video_pio_program video_24mhz_composable = {
        .program = video_24mhz_composable_program,
        .program_size = count_of(video_24mhz_composable_program),
        .entry_point = video_24mhz_composable_program_extern(entry_point),
        .adapt_for_mode = video_24mhz_composable_adapt_for_mode,
        .configure_pio = video_24mhz_composable_configure_pio
};

#define PIO_WAIT_IRQ4 pio_encode_wait_irq(1, 4)

static const uint16_t video_dbi_control_load_offset = 16;

// --- video timing stuff

static struct {
    uint16_t v_active;
    uint16_t v_total;
    uint16_t v_pulse_start;
    uint16_t v_pulse_end;

} timing_state;

#ifdef ENABLE_VIDEO_CLOCK_DOWN
static uint16_t video_clock_down;
#endif

struct semaphore vblank_begin;

// --- scanline stuff
// private representation of scanline buffer (adds link for one list this scanline buffer is currently in)
struct full_scanline_buffer {
    struct scanline_buffer core;
    struct full_scanline_buffer *next;
};

#ifndef PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT
#define PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT 8
#endif
// each scanline_buffer should be in exactly one of the shared_state lists below
// (unless we don't have USE_SCANLINE_DEBUG in which case we don't keep the generating list,
// in which case the scanline is entirely trusted to the client when generating)
struct full_scanline_buffer scanline_buffers[PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT];

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
        struct full_scanline_buffer *in_use_ascending_scanline_id_list;
        // pointer to the tail element of the list for making appending by ascending scanline id quick
        struct full_scanline_buffer *in_use_ascending_scanline_id_list_tail;
    } in_use;

    struct {
        spin_lock_t *lock;
        struct full_scanline_buffer *current_scanline_buffer;
        uint32_t last_scanline_id;
        uint32_t next_scanline_id;
        // 0 based index of y repeat... goes 0, 0, 0 in non scaled mode, 0, 1, 0, 1 in doubled etc.
        uint y_repeat_index;
        bool in_vblank;
        // This generated list is in this struct because it is accessed together in fsb latching
        // and the only other place it is used in video_end_scanline_generation which needs no other
        // locks (i.e. we are saving an extra lock in the latch case by not placing in a separate struct)
        struct full_scanline_buffer *generated_ascending_scanline_id_list;
        struct full_scanline_buffer *generated_ascending_scanline_id_list_tail;
        bool vblank_pending;
        bool need_prepare_for_active_scanline;
#ifdef ENABLE_SCANLINE_ASSERTIONS
        struct full_scanline_buffer *generating_list;
#endif
    } scanline;

    struct {
        spin_lock_t *lock;
        struct full_scanline_buffer *free_list;
    } free_list;

    // This is access by DMA IRQ and by SM IRQs
    struct {
        spin_lock_t *lock;
        // bit mask of completed DMA scanline channels
        uint32_t dma_completion_state;
        // number of buffers to release (may be multiple due to interrupt pre-emption)
        // todo combine these two fields
        uint8_t buffers_to_release;
        bool scanline_in_progress;
    } dma;

    bool which_buffer;
    // these are not updated, so not locked
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
    int scanline_program_wait_index;
#endif
} shared_state;

static uint32_t missing_scanline_data[] =
        {
                COMPOSABLE_COLOR_RUN | (PICO_SCANVIDEO_PIXEL_FROM_RGB8(255, 0, 0) << 16u), /* color */
                /*width-3*/ 0u | (COMPOSABLE_RAW_1P << 16u),
                0u | (COMPOSABLE_EOL_ALIGN << 16u)
        };

#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
static uint32_t variable_fragment_missing_scanline_data_chain[] = {
    count_of(missing_scanline_data),
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

static struct full_scanline_buffer missing_scanline_buffer;

static inline bool is_scanline_after(uint32_t scanline_id1, uint32_t scanline_id2) {
    return ((int32_t)(scanline_id1 - scanline_id2)) > 0;
}

static void prepare_for_active_scanline_irqs_enabled();

static void setup_sm(int sm);

// -- MISC stuff
static struct video_mode video_mode;
static bool video_timing_enabled = false;
static bool display_enabled = true;

inline static void list_prepend(struct full_scanline_buffer **phead, struct full_scanline_buffer *fsb) {
    scanline_assert(fsb->next == NULL);
    scanline_assert(fsb != *phead);
    fsb->next = *phead;
    *phead = fsb;
}

inline static void list_prepend_all(struct full_scanline_buffer **phead, struct full_scanline_buffer *to_prepend) {
    struct full_scanline_buffer *fsb = to_prepend;

    // todo should this be assumed?
    if (fsb) {
        while (fsb->next) {
            fsb = fsb->next;
        }

        fsb->next = *phead;
        *phead = to_prepend;
    }
}

inline static struct full_scanline_buffer *list_remove_head(struct full_scanline_buffer **phead) {
    struct full_scanline_buffer *fsb = *phead;

    if (fsb) {
        *phead = fsb->next;
        fsb->next = NULL;
    }

    return fsb;
}

inline static struct full_scanline_buffer *list_remove_head_ascending(struct full_scanline_buffer **phead,
                                                                      struct full_scanline_buffer **ptail) {
    struct full_scanline_buffer *fsb = *phead;

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

inline static void list_remove(struct full_scanline_buffer **phead, struct full_scanline_buffer *fsb) {
    scanline_assert(*phead);
    struct full_scanline_buffer *prev = *phead;

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
inline static void list_insert_ascending(struct full_scanline_buffer **phead, struct full_scanline_buffer **ptail,
                                         struct full_scanline_buffer *fsb) {
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
            struct full_scanline_buffer *prev = *phead;

            while (prev->next && is_scanline_after(fsb->core.scanline_id, prev->next->core.scanline_id)) {
                prev = prev->next;
            }

            scanline_assert(prev != *ptail); // we should have already inserted at the end in this case
            fsb->next = prev->next;
            prev->next = fsb;
        }
    }
}

inline static void free_local_free_list_irqs_enabled(struct full_scanline_buffer *local_free_list) {
    if (local_free_list) {
        uint32_t save = spin_lock_blocking(shared_state.free_list.lock);
//        DEBUG_PINS_SET(video_timing, 4);
        list_prepend_all(&shared_state.free_list.free_list, local_free_list);
//        DEBUG_PINS_CLR(video_timing, 4);
        spin_unlock(shared_state.free_list.lock, save);
        // note also this is useful for triggering video_wait_for_scanline_complete check
        __sev();
    }
}

// Caller must own scanline_state_spin_lock
inline static struct full_scanline_buffer *scanline_locked_try_latch_fsb_if_null_irqs_disabled(
        struct full_scanline_buffer **local_free_list) {
    // note this just checks that someone owns it not necessarily this core.
    scanline_assert(is_spin_locked(shared_state.scanline.lock));
    struct full_scanline_buffer *fsb = shared_state.scanline.current_scanline_buffer;

    if (!fsb) {
        // peek the head
        while (NULL != (fsb = shared_state.scanline.generated_ascending_scanline_id_list)) {
            if (!is_scanline_after(shared_state.scanline.next_scanline_id, fsb->core.scanline_id)) {
                if (shared_state.scanline.next_scanline_id == fsb->core.scanline_id) {
                    int c1 = 0;
                    for (struct full_scanline_buffer *x = shared_state.scanline.generated_ascending_scanline_id_list; x; x = x->next) c1++;
                    struct full_scanline_buffer __unused
                    *dbg = list_remove_head_ascending(&shared_state.scanline.generated_ascending_scanline_id_list,
                                                      &shared_state.scanline.generated_ascending_scanline_id_list_tail);
                    scanline_assert(dbg == fsb);
                    int c2 = 0;
                    for (struct full_scanline_buffer *x = shared_state.scanline.generated_ascending_scanline_id_list; x; x = x->next) c2++;
                    unprotected_spin_lock(shared_state.in_use.lock);
//                    DEBUG_PINS_SET(video_timing, 2);
                    list_insert_ascending(&shared_state.in_use.in_use_ascending_scanline_id_list,
                                          &shared_state.in_use.in_use_ascending_scanline_id_list_tail, fsb);
//                    DEBUG_PINS_CLR(video_timing, 2);
                    spin_unlock_unsafe(shared_state.in_use.lock);
                }

                shared_state.scanline.current_scanline_buffer = fsb;
                break;
            } else {
                // scanline is in the past
                struct full_scanline_buffer __unused
                *dbg = list_remove_head_ascending(&shared_state.scanline.generated_ascending_scanline_id_list,
                                                  &shared_state.scanline.generated_ascending_scanline_id_list_tail);
                scanline_assert(dbg == fsb);
                list_prepend(local_free_list, fsb);
            }
        }
    }

    return fsb;
}

static inline void release_scanline_irqs_enabled(int buffers_to_free_count,
                                                 struct full_scanline_buffer **local_free_list) {
    if (buffers_to_free_count) {
        uint32_t save = spin_lock_blocking(shared_state.in_use.lock);
        while (buffers_to_free_count--) {
            DEBUG_PINS_SET(video_dma_buffer, 2);
            // We always discard the head which is the oldest
            struct full_scanline_buffer *fsb = list_remove_head_ascending(
                    &shared_state.in_use.in_use_ascending_scanline_id_list,
                    &shared_state.in_use.in_use_ascending_scanline_id_list_tail);
            scanline_assert(fsb);
            list_prepend(local_free_list, fsb);
            DEBUG_PINS_CLR(video_dma_buffer, 2);
        }
        spin_unlock(shared_state.in_use.lock, save);
    }
}

static inline bool update_dma_transfer_state_irqs_enabled(bool cancel_if_not_complete,
                                                          int *scanline_buffers_to_release) {
    uint32_t save = spin_lock_blocking(shared_state.dma.lock);
    if (!shared_state.dma.scanline_in_progress) {
        assert(!shared_state.dma.dma_completion_state);
        assert(!shared_state.dma.buffers_to_release);
        spin_unlock(shared_state.dma.lock, save);
        return true;
    }
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
        }
        dma_abort(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL);
#else
        panic("need VIDEO_RECOVERY");
#endif
    }
    spin_unlock(shared_state.dma.lock, save);
    return false;
}

void __video_most_time_critical("irq")

prepare_for_active_scanline_irqs_enabled() {
    // note we are now only called in active display lines..
    DEBUG_PINS_SET(video_timing, 1);
    struct full_scanline_buffer *local_free_list = NULL;
    int buffers_to_free_count = 0;
    uint32_t save = spin_lock_blocking(shared_state.scanline.lock);
    // VERY IMPORTANT: THIS CODE CAN ONLY TAKE ABOUT 4.5 us BEFORE LAUNCHING DMA...
    // ... otherwise our scanline will be shifted over (because we will have started display)
    //
    // to alleviate this somewhat, we let the dma_complete alsu do a check for current_scanline_buffer == null, and look for a completed scanlines
    // In ideal case the dma complete IRQ handler will have been able to set current_scanline_buffer for us (or indeed this is a y scaled mode
    // and we are repeating a line)... in either case we will come in well under time budget
    struct full_scanline_buffer *fsb = scanline_locked_try_latch_fsb_if_null_irqs_disabled(&local_free_list);
#ifdef GO_AT_USER_SPEED
    bool not_ready = false;
    if (!fsb || fsb->core.scanline_id != shared_state.scanline.next_scanline_id) {
        shared_state.scanline.need_prepare_for_active_scanline = true;
        shared_state.scanline.current_scanline_buffer = 0;
        not_ready = true;
    }
#endif
    spin_unlock(shared_state.scanline.lock, save);
#ifdef GO_AT_USER_SPEED
    if (not_ready) return;
#else
    if (fsb)
    {
        if (fsb->core.scanline_id != shared_state.scanline.next_scanline_id) {
            // removed to allow for other video modes; not worth abstracting that far...
            // also; we basically never see this color anyway!
//            ((uint16_t *) (missing_scanline_data))[1] = 0x03e0;
            // note: this should be in the future
            fsb = &missing_scanline_buffer;
            fsb->core.scanline_id = shared_state.scanline.next_scanline_id; // used for y position so we must update
        }
    }
    else
    {
        // removed to allow for other video modes; not worth abstracting that far...
//        ((uint16_t *)(missing_scanline_data))[1] = 0x001f;
        // this is usually set by latch
        fsb = &missing_scanline_buffer;
        fsb->core.scanline_id = shared_state.scanline.next_scanline_id; // used for y position so we must update
    }
#endif

    update_dma_transfer_state_irqs_enabled(true, &buffers_to_free_count);

//    DEBUG_PINS_SET(video_irq, 2);
    // bit of overkill (to reset src_addr) for y scale repeat lines, but then again those should already have data. but this is now
    // required in case current_scanline_buffer was set by the dma complete handler, in which case current_scanline_buffer was null when we got to the test above

    // don't need to reset anything put the CB pointer to start a reload? as we have already configured the rest
    // note DMA should already be aborted by here.
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
    if (!pio_tx_empty(video_pio, PICO_SCANVIDEO_SCANLINE_SM)) {
        pio_fifo_join(video_pio, PICO_SCANVIDEO_SCANLINE_SM, PIO_FIFO_JOIN_NONE);
        pio_fifo_join(video_pio, PICO_SCANVIDEO_SCANLINE_SM, PIO_FIFO_JOIN_TX);
    }
    if (video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM].instr != PIO_WAIT_IRQ4) {
        // hmm the problem here is we don't know if we should wait or not, because that is purely based on timing..
        // - if irq not posted, and we wait: GOOD
        // - if irq not posted and we don't wait: BAD. early line
        // - if irq already posted, and we wait: BAD. blank line
        // - id irq already posted, and we don't wait: GOOD
        pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM, pio_encode_wait_irq(0, 4));
        if (pio_sm_exec_stalled(video_pio, PICO_SCANVIDEO_SCANLINE_SM)) {
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM, pio_encode_jmp(shared_state.scanline_program_wait_index));
        } else {
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM, pio_encode_jmp(shared_state.scanline_program_wait_index+1));
        }
    }
#endif
#if PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
#if PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA
    dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL)->al3_transfer_count = fsb->core.fragment_words;
#endif
    //dma_transfer_from_buffer_now(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL, (uintptr_t)fsb->core.data, (uint32_t) fsb->core.data_used);
    dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL)->al3_read_addr_trig = (uintptr_t)fsb->core.data;
#else
    assert(!dma_busy(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL));
    dma_transfer_from_buffer_now(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL, (uintptr_t) fsb->core.data,
                                 (uint32_t) fsb->core.data_used);
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 1
#if PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA
    dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2)->al3_read_addr_trig = (uintptr_t)fsb->core.data2;
#else
    dma_transfer_from_buffer_now(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2, (uintptr_t)fsb->core.data2, (uint32_t) fsb->core.data2_used);
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    dma_transfer_from_buffer_now(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3, (uintptr_t)fsb->core.data3, (uint32_t) fsb->core.data3_used);
//    scanline_assert(video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM3].addr == video_24mhz_composable_offset_end_of_scanline_ALIGN);
#endif
//    scanline_assert(video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM2].addr == video_24mhz_composable_offset_end_of_scanline_ALIGN);
#endif

    //. send the control signals which will send irq 4
    // todo precheck width is even
    DEBUG_PINS_SET(sequence, 4);
    uint count;
    assert(!dma_busy(TIMING_DMA_CHANNEL));
    uint32_t *control = get_control_sequence(video_mode.width & ~1u, scanline_number(fsb->core.scanline_id), &count,
                                             shared_state.which_buffer);
//    printf("pants %p %d\n", control, count);
    DEBUG_PINS_SET(video_dma_buffer, 3);
    dma_transfer_from_buffer_now(TIMING_DMA_CHANNEL, (uintptr_t) control, count);
//    while (dma_busy(TIMING_DMA_CHANNEL)) {
//        printf("  %d\n", dma_channel_hw_addr(TIMING_DMA_CHANNEL)->transfer_count);
//    }
    DEBUG_PINS_CLR(video_dma_buffer, 3);
    DEBUG_PINS_CLR(sequence, 4);

//    scanline_assert(video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM].addr == video_24mhz_composable_offset_end_of_scanline_ALIGN);
//    DEBUG_PINS_CLR(video_irq, 2);

    save = spin_lock_blocking(shared_state.scanline.lock);
    DEBUG_PINS_SET(video_timing, 2);
    shared_state.scanline.in_vblank = false;
    bool was_correct_scanline = (fsb != &missing_scanline_buffer);
    bool free_scanline = false;
    if (++shared_state.scanline.y_repeat_index >= video_mode.yscale) {
        // pick up a new scanline next time around if we had the right one
        if (was_correct_scanline) {
            free_scanline = true;
        }

        shared_state.scanline.next_scanline_id = scanline_id_after(shared_state.scanline.next_scanline_id);
        if (!scanline_number(shared_state.scanline.next_scanline_id)) {
            shared_state.scanline.vblank_pending = true;
        }
        shared_state.scanline.y_repeat_index = 0;
        shared_state.scanline.current_scanline_buffer = NULL;
    } else if (!was_correct_scanline) {
        // not at the the end of yscale, but the wrong (or missing) scanline anyway, so clear that
        shared_state.scanline.current_scanline_buffer = NULL;
    }
    // safe to nest dma lock we never nest the other way
    unprotected_spin_lock(shared_state.dma.lock);
    shared_state.dma.scanline_in_progress = 1;
    if (free_scanline) {
        scanline_assert(!shared_state.dma.buffers_to_release);
        shared_state.dma.buffers_to_release++;
    }
    spin_unlock_unsafe(shared_state.dma.lock);
    DEBUG_PINS_CLR(video_timing, 3);
    spin_unlock(shared_state.scanline.lock, save);

    // because IRQs are enabled, we may obviously be pre-empted before or between either of these
    release_scanline_irqs_enabled(buffers_to_free_count, &local_free_list);
    free_local_free_list_irqs_enabled(local_free_list);
}

void __isr __video_most_time_critical("irq") isr_pio0_0() {
#if PICO_SCANVIDEO_ADJUST_BUS_PRIORITY
    bus_ctrl_hw->priority = VIDEO_ADJUST_BUS_PRIORITY_VAL;
#endif
    if (video_pio->irq & 1u) {
        video_pio->irq = 1;
        DEBUG_PINS_SET(video_timing, 4);
        scanline_assert(!dma_busy(TIMING_DMA_CHANNEL));
        scanline_assert(video_pio->sm[TIMING_SM].addr ==
                        video_dbi_control_load_offset + video_dbi_control_offset_new_state_wait);

        bool signal = false;
        uint32_t save = spin_lock_blocking(shared_state.scanline.lock);
        signal = shared_state.scanline.vblank_pending;
        shared_state.scanline.vblank_pending = false;
        spin_unlock(shared_state.scanline.lock, save);
        if (signal) {
            uint count;
            uint32_t *control = get_switch_buffer_sequence(&count, shared_state.which_buffer);
            shared_state.which_buffer = !shared_state.which_buffer;
            dma_transfer_from_buffer_now(TIMING_DMA_CHANNEL, (uintptr_t) control, count);
            while (dma_busy(TIMING_DMA_CHANNEL));
            sem_release(&vblank_begin);
        }

        // we are called at then end of a scanline transfer (from the timing SM)
        if (display_enabled) {
            if (!scanline_number(shared_state.scanline.next_scanline_id)) {
            }
            bool too_soon = false;
            if (!too_soon) {
                prepare_for_active_scanline_irqs_enabled();
            } else {
                assert(false); // not handled yet
            }
        }
        DEBUG_PINS_CLR(video_timing, 4);
    }
}

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

void setup_sm(int sm) {
#ifndef NDEBUG
    printf("Setting up SM %d\n", sm);
#endif
    pio_sm_init(video_pio, sm); // now paused

    if (is_scanline_sm(sm)) {
        video_mode.pio_program->configure_pio(video_pio, sm);
    } else if (sm == TIMING_SM) {
        // don't join as we want to read too
//        pio_fifo_join(video_pio, sm, PIO_FIFO_JOIN_TX); // give the program as much time as we can

        pio_set_consecutive_pindirs(video_pio, TIMING_SM, WR_PIN, 1, true);
        //pio_set_consecutive_pindirs(video_pio, TIMING_SM, 0, 16, true);
        pio_set_consecutive_pindirs(video_pio, TIMING_SM, RS_PIN, 2, true);
        pio_setup_pinctrl(video_pio, sm, 0, 16, RS_PIN, 2, WR_PIN, 0);

        pio_setup_pinctrl(video_pio, sm, 0, 16, RS_PIN, 2, WR_PIN, 0);
        pio_setup_sideset(video_pio, sm, 1, false, 0);
        pio_setup_out_special(video_pio, sm, 1, 0, 0); // need to be sticky as we output control info

        // init pins
        pio_sm_exec(video_pio, sm, pio_encode_set_pins(3));
        pio_set_wrap(video_pio, sm, video_dbi_control_load_offset + video_dbi_control_wrap_target,
                     video_dbi_control_load_offset + video_dbi_control_wrap);
    }

#ifdef ENABLE_VIDEO_CLOCK_DOWN
    pio_set_clkdiv_int_frac(video_pio, sm, video_clock_down, 0);
#endif
    // enable auto-pull
    pio_setup_shiftctrl(video_pio, sm, SHIFT_TO_RIGHT, SHIFT_TO_RIGHT, 0, 1, 32, 32);
}

//extern bool video_get_mode(struct video_mode *mode) {
//    // todo if initialized
//    *mode = video_mode;
//    return true;
//}

extern uint32_t video_get_next_scanline_id() {
    return *(volatile uint32_t *) &shared_state.scanline.next_scanline_id;
}

extern bool video_in_hblank() {
    // this is a close estimate
    //    return !*(volatile bool *) &shared_state.dma.scanline_in_progress;

    // better we can see if the PIO is waiting on IRQ 4 (which it is almost all of the time it isn't drawing pixels)
    //
    // note that currently we require even custom PIO scanline programs to use WAIT IRQ 4 to sync with start of scanline
    return video_pio->sm[PICO_SCANVIDEO_SCANLINE_SM].instr == PIO_WAIT_IRQ4;
}

extern bool video_in_vblank() {
    return *(volatile bool *) &shared_state.scanline.in_vblank;
}

extern struct scanline_buffer __video_time_critical("begin_scanline") *
video_begin_scanline_generation(bool
block)
{
struct full_scanline_buffer *fsb;

DEBUG_PINS_SET(video_generation,
1);
do
{
uint32_t save = spin_lock_blocking(shared_state.free_list.lock);
//        DEBUG_PINS_SET(video_timing, 4);
fsb = list_remove_head(&shared_state.free_list.free_list);
//        DEBUG_PINS_CLR(video_timing, 4);
spin_unlock(shared_state
.free_list.lock, save);

if (fsb)
{
save = spin_lock_blocking(shared_state.scanline.lock);
#ifdef ENABLE_SCANLINE_ASSERTIONS
list_prepend(&shared_state.scanline.generating_list, fsb);
#endif
// todo improve this algorithm... how far ahead should we be
// todo i.e. should we skip ahead a bit further if we are perpetually behind - doesn't really help because we'd
// todo be skipping some scanlines anyway; doesn't really matter which ones at that point
uint32_t scanline_id = shared_state.scanline.next_scanline_id;

if (!
is_scanline_after(scanline_id, shared_state
.scanline.last_scanline_id))
{
// we are buffering ahead of the display
scanline_id = scanline_id_after(shared_state.scanline.last_scanline_id);
}

fsb->core.
scanline_id = shared_state.scanline.last_scanline_id = scanline_id;
spin_unlock(shared_state
.scanline.lock, save);
break;
}

if (block)
{
DEBUG_PINS_SET(video_generation,
4);
__wfe();
DEBUG_PINS_CLR(video_generation,
4);
}
}
while (block);

DEBUG_PINS_CLR(video_generation,
1);
return (struct scanline_buffer *)
fsb;
}

extern void __video_time_critical("end_scanline")

video_end_scanline_generation(struct scanline_buffer *scanline_buffer) {
    DEBUG_PINS_SET(video_generation, 2);
    struct full_scanline_buffer *fsb = (struct full_scanline_buffer *) scanline_buffer;
    uint32_t save = spin_lock_blocking(shared_state.scanline.lock);
#ifdef ENABLE_SCANLINE_ASSERTIONS
    list_remove(&shared_state.scanline.generating_list, fsb);
#endif
    list_insert_ascending(&shared_state.scanline.generated_ascending_scanline_id_list,
                          &shared_state.scanline.generated_ascending_scanline_id_list_tail, fsb);
    bool prepare = shared_state.scanline.need_prepare_for_active_scanline;
    shared_state.scanline.need_prepare_for_active_scanline = false;
    spin_unlock(shared_state.scanline.lock, save);
    if (prepare) {
        prepare_for_active_scanline_irqs_enabled();
    }
    DEBUG_PINS_CLR(video_generation, 2);
}

#pragma GCC pop_options

bool video_setup(const struct video_mode *mode) {
    return video_setup_with_timing(mode, mode->default_timing);
}

bool video_setup_with_timing(const struct video_mode *mode, const struct video_timing *timing) {
    __builtin_memset(&shared_state, 0, sizeof(shared_state));
    // init non zero members
    // todo pass scanline buffers and size, or allow client to allocate
    shared_state.scanline.lock = spin_lock_init(PICO_SPINLOCK_ID_VIDEO_SCANLINE_LOCK);
    shared_state.dma.lock = spin_lock_init(PICO_SPINLOCK_ID_VIDEO_DMA_LOCK);
    shared_state.free_list.lock = spin_lock_init(PICO_SPINLOCK_ID_VIDEO_FREE_LIST_LOCK);
    shared_state.in_use.lock = spin_lock_init(PICO_SPINLOCK_ID_VIDEO_IN_USE_LOCK);
    shared_state.scanline.last_scanline_id = 0xffffffff;
    shared_state.scanline.need_prepare_for_active_scanline = true;

    video_mode = *mode;
    video_mode.default_timing = timing;

    static_assert(BPP == 16, ""); // can't do 8 bit now because of pixel count
    // this is no longer necessary
    //assert(!(mode->width & 1));
    // todo is this still necessary?
    video_assert(!(timing->v_active % mode->yscale));
    ((uint16_t * )(missing_scanline_data))[2] = mode->width / 2 - 3;
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
    variable_fragment_missing_scanline_data_chain[1] = native_safe_hw_ptr(missing_scanline_data);
#endif
#if PICO_SCANVIDEO_PLANE1_FIXED_FRAGMENT_DMA || PICO_SCANVIDEO_PLANE2_FIXED_FRAGMENT_DMA
    fixed_fragment_missing_scanline_data_chain[0] = native_safe_hw_ptr(missing_scanline_data);
#endif

    sem_init(&vblank_begin, 0, 1);

    for (int i = 0; i < PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT; i++) {
        scanline_buffers[i].core.data = (uint32_t *) calloc(PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS, sizeof(uint32_t));
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

#ifndef ENABLE_VIDEO_CLOCK_DOWN
    video_assert(timing->clock_freq == video_clock_freq);
#else
    video_clock_down = video_clock_freq / timing->clock_freq;
    video_assert( video_clock_down * timing->clock_freq == video_clock_freq);
#endif

    setup_sm(PICO_SCANVIDEO_SCANLINE_SM);
#if PICO_SCANVIDEO_PLANE_COUNT > 1
    setup_sm(PICO_SCANVIDEO_SCANLINE_SM2);
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    setup_sm(PICO_SCANVIDEO_SCANLINE_SM3);
#endif
#endif
    setup_sm(TIMING_SM);

    video_assert(mode->width * mode->xscale <= timing->h_active);
    video_assert(mode->height * mode->yscale <= timing->v_active);

    uint16_t program[32];
    if (!mode->pio_program->adapt_for_mode(mode->pio_program, mode, &missing_scanline_buffer.core, program,
                                           count_of(program))) {
        video_assert(false);
    }
    video_assert(missing_scanline_buffer.core.data && missing_scanline_buffer.core.data_used);
#if PICO_SCANVIDEO_PLANE_COUNT > 1
    video_assert(missing_scanline_buffer.core.data2 && missing_scanline_buffer.core.data2_used);
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    video_assert(missing_scanline_buffer.core.data3 && missing_scanline_buffer.core.data3_used);
#endif
#endif
    missing_scanline_buffer.core.status = SCANLINE_OK;

#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
    int program_wait_index = -1;
#endif
#if defined(PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY) || !defined(DISABLE_VIDEO_ASSERTIONS)
    for(int i = 0; i < mode->pio_program->program_size; i++) {
        if (program[i] == PIO_WAIT_IRQ4) {
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
            video_assert(program_wait_index == -1);
            program_wait_index = i;
#endif
        }
    }
#if PICO_SCANVIDEO_ENABLE_VIDEO_RECOVERY
    video_assert(program_wait_index != -1);
    shared_state.scanline_program_wait_index = program_wait_index;
#endif
#endif

    pio_load_program(video_pio, program, mode->pio_program->program_size, 0);

    uint32_t side_set_xor = 0;
    static_assert(count_of(video_dbi_control_program) <= count_of(program), "too big");
    __builtin_memcpy(program, video_dbi_control_program, count_of(video_dbi_control_program) * sizeof(uint16_t));

    if (timing->clock_polarity) {
        side_set_xor = 0x1000; // flip the top side set bit

        for (int i = 0; i < count_of(program); i++) {
            program[i] ^= side_set_xor;
        }
    }

    pio_load_program(video_pio, program, count_of(video_dbi_control_program), video_dbi_control_load_offset);

    // todo priorities should be correct anyway...
    irq_set_priority(PIO0_IRQ_1, 0x40); // lower priority by 1
    irq_set_priority(DMA_IRQ_0, 0x80); // lower priority by 2

    // todo merge these calls
    dma_enable_irq0(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL, true);
#if PICO_SCANVIDEO_PLANE_COUNT > 1
    dma_enable_irq0(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2, true);
#if PICO_SCANVIDEO_PLANE_COUNT > 2
    dma_enable_irq0(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3, true);
#endif
#endif
    // also done in video_timing_enable
//    video_pio->inte1 = 1u << (TIMING_SM + PIO_IRQ1_INTE_SM0_TXNFULL_LSB);

    // todo reset DMA channels

    dma_configure(
            PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL,
            0,                        // src
            (uint32_t) & video_pio->txf[PICO_SCANVIDEO_SCANLINE_SM],  // dest
            SIZE_32,
            DMA_INCR,
            DMA_NOINCR,
            0, // len (set later)
            DREQ_PIO0_TX0 +
            PICO_SCANVIDEO_SCANLINE_SM // Select scanline dma dreq to be PICO_SCANVIDEO_SCANLINE_SM TX FIFO not full
    );
#if PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
    dma_chain_to(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL, PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL); // individual buffers chain back to master
    dma_set_quiet(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL, true);
    dma_configure_full(
            0, // src
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA
            (uintptr_t) &dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL)->al3_transfer_count,  // ch DMA config (target "ring" buffer size 8) - this is (transfer_count, read_addr trigger)
#else
            (uintptr_t) &dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL)->al3_read_addr_trig,  // ch DMA config (target "ring" buffer size 4) - this is (read_addr trigger)
#endif
            SIZE_32,
            DMA_INCR,
            DMA_INCR,
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA
            2, // send 2 words to ctrl block of data chain per transfer
#else
            1,
#endif
            DREQ_FORCE,
            PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL, // no chain as we trigger the data channel via _trig reg
            1,
#if PICO_SCANVIDEO_PLANE1_VARIABLE_FRAGMENT_DMA
            3, // wrap the write at 8 bytes (so each transfer writes the same 2 word ctrl registers)
#else
            2, // wrap the write at 4 bytes (so each transfer writes the same ctrl register)
#endif
            true, // enable
            dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL)
    );
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 1
    dma_configure(
            PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2,
            0,                        // src
            (uint32_t)&video_pio->txf[PICO_SCANVIDEO_SCANLINE_SM2],  // dest
            SIZE_32,
            DMA_INCR,
            DMA_NOINCR,
            0, // len
            DREQ_PIO0_TX0 + PICO_SCANVIDEO_SCANLINE_SM2// Select scanline2 dma dreq to be PICO_SCANVIDEO_SCANLINE_SM2 TX FIFO not full
    );
#if PICO_SCANVIDEO_PLANE2_FRAGMENT_DMA
#if !PICO_SCANVIDEO_PLANE2_VARIABLE_FRAGMENT_DMA
    static_assert(false);
#endif
    dma_chain_to(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2, PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2); // individual buffers chain back to master
    dma_set_quiet(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2, true);
    dma_configure_full(
            0, // src
            (uintptr_t) &dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL2)->al3_transfer_count,  // ch DMA config (target "ring" buffer size 8) - this is (transfer_count, read_addr trigger)
            SIZE_32,
            DMA_INCR,
            DMA_INCR,
            2, // send 2 words to ctrl block of data chain per transfer
            DREQ_FORCE,
            PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2, // no chain as we trigger the data channel via _trig reg
            1,
            3, // wrap the write at 8 bytes (so each transfer writes the same 2 word ctrl registers)
            true, // enable
            dma_channel_hw_addr(PICO_SCANVIDEO_SCANLINE_DMA_CB_CHANNEL2)
    );
#endif
#if PICO_SCANVIDEO_PLANE_COUNT > 2
#if PICO_SCANVIDEO_PLANE3_FRAGMENT_DMA
    static_assert(false);
#endif
    dma_configure(
            PICO_SCANVIDEO_SCANLINE_DMA_CHANNEL3,
            0,                        // src
            (uint32_t)&video_pio->txf[PICO_SCANVIDEO_SCANLINE_SM3],  // dest
            SIZE_32,
            DMA_INCR,
            DMA_NOINCR,
            0, // len
            DREQ_PIO0_TX0 + PICO_SCANVIDEO_SCANLINE_SM3// Select scanline3 dma dreq to be PICO_SCANVIDEO_SCANLINE_SM3 TX FIFO not full
    );
#endif
#endif

    dma_configure(
            TIMING_DMA_CHANNEL,
            0,                        // src
            (uint32_t) & video_pio->txf[TIMING_SM],  // dest
            SIZE_32,
            DMA_INCR,
            DMA_NOINCR,
            0, // len (set later)
            DREQ_PIO0_TX0 + TIMING_SM // Select scanline dma dreq to be PICO_SCANVIDEO_SCANLINE_SM TX FIFO not full
    );

    // clear scanline irq
    pio_sm_exec(video_pio, TIMING_SM, pio_encode_irq_clear(4, false));

    timing_state.v_total = timing->v_total;
    timing_state.v_active = timing->v_active;
    timing_state.v_pulse_start = timing->v_active + timing->v_front_porch;
    timing_state.v_pulse_end = timing_state.v_pulse_start + timing->v_pulse;

    tft_driver_init();
    gpio_set_mask(7u << 19u);
    for (uint i = 0; i < 16; i++) {
        gpio_funcsel(i, GPIO_FUNC_PIO0);
    }
    gpio_funcsel(WR_PIN, GPIO_FUNC_PIO0);
    gpio_funcsel(CS_PIN, GPIO_FUNC_PIO0);
    gpio_funcsel(RS_PIN, GPIO_FUNC_PIO0);
    gpio_clr_mask(7u << 19u);
//    gpio_funcsel(RST_PIN, GPIO_FUNC_PIO0);
    return true;
}

bool video_24mhz_composable_adapt_for_mode(const struct video_pio_program *program, const struct video_mode *mode,
                                           struct scanline_buffer *missing_scanline_buffer, uint16_t *buffer,
                                           uint buffer_max) {
    int delay0 = 2 * mode->xscale - 2;
    int delay1 = delay0 + 1;
    video_assert(delay0 <= 31);
    video_assert(delay1 <= 31);

    video_assert(buffer_max >= program->program_size);
    __builtin_memcpy(buffer, program->program, program->program_size * sizeof(uint16_t));

    // todo macro-ify this
    buffer[video_24mhz_composable_program_extern(delay_a_1)] |= (unsigned) delay1 << 8u;
    buffer[video_24mhz_composable_program_extern(delay_b_1)] |= (unsigned) delay1 << 8u;
    buffer[video_24mhz_composable_program_extern(delay_c_0)] |= (unsigned) delay0 << 8u;
    buffer[video_24mhz_composable_program_extern(delay_d_0)] |= (unsigned) delay0 << 8u;
    buffer[video_24mhz_composable_program_extern(delay_e_0)] |= (unsigned) delay0 << 8u;
    buffer[video_24mhz_composable_program_extern(delay_f_1)] |= (unsigned) delay1 << 8u;
#if !PICO_SCANVIDEO_USE_RAW1P_2CYCLE
    buffer[video_24mhz_composable_program_extern(delay_g_0)] |= (unsigned) delay0 << 8u;
#else
    int delay_half = mode->xscale - 2;
    buffer[video_24mhz_composable_program_extern(delay_g_0)] |= (unsigned)delay_half << 8u;
#endif
    buffer[video_24mhz_composable_program_extern(delay_h_0)] |= (unsigned) delay0 << 8u;

#if !PICO_SCANVIDEO_PLANE1_FRAGMENT_DMA
    missing_scanline_buffer->data = missing_scanline_data;
    missing_scanline_buffer->data_used = missing_scanline_buffer->data_max = sizeof(missing_scanline_data) / 4;
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

bool video_default_adapt_for_mode(const struct video_pio_program *program, const struct video_mode *mode,
                                  uint16_t *buffer, uint buffer_max) {
    if (buffer_max >= program->program_size) {
        __builtin_memcpy(buffer, program->program, program->program_size * sizeof(uint16_t));
        return true;
    }
    return false;
}

void video_default_configure_pio(pio_hw_t *pio, uint sm, uint wrap_trarget, uint wrap, bool overlay) {
    const uint BASE = 0; //sm == PICO_SCANVIDEO_SCANLINE_SM ? 0 : 5;
    pio_set_consecutive_pindirs(pio, sm, BASE, 16, true);
    pio_setup_pinctrl(pio, sm, BASE, 16, 0, 0, 0, 0);
    pio_sm_exec(pio, sm, pio_encode_set_pins(0));
    pio_fifo_join(pio, sm, PIO_FIFO_JOIN_TX); // give the program as much time as we can
    pio_set_wrap(pio, sm, wrap_trarget, wrap);
    if (overlay) {
        pio_setup_out_special(pio, sm, 1, 1, 5);
    } else {
        pio_setup_out_special(pio, sm, 1, 0, 0);
    }
}

void video_24mhz_composable_configure_pio(pio_hw_t *pio, uint sm) {
    video_default_configure_pio(pio, sm, video_24mhz_composable_wrap_target, video_24mhz_composable_wrap,
                                sm != PICO_SCANVIDEO_SCANLINE_SM);
}

void video_timing_enable(bool enable) {
    // todo we need to protect our state here... this can't be frame synced obviously (at least turning on)
    // todo but we should make sure we clear out state when we turn it off, and probably reset scanline counter when we turn it on
    if (enable != video_timing_enabled) {
        // todo should we disable these too? if not move to video_setup
        video_pio->inte0 = PIO_IRQ0_INTE_SM0_BITS;// | PIO_IRQ0_INTE_SM1_BITS;
//        video_pio->inte1 = (1u << (TIMING_SM + PIO_IRQ1_INTE_SM0_TXNFULL_LSB));
        irq_enable_mask((1u << PIO0_IRQ_0)
                //|(1u << PIO0_IRQ_1)
//                                  |(1u << DMA_IRQ_0)
                , enable);
        uint32_t sm_mask = (1u << PICO_SCANVIDEO_SCANLINE_SM) | 1u << TIMING_SM;
#if PICO_SCANVIDEO_PLANE_COUNT > 1
        sm_mask |= 1u << PICO_SCANVIDEO_SCANLINE_SM2;
#if PICO_SCANVIDEO_PLANE_COUNT > 2
        sm_mask |= 1u << PICO_SCANVIDEO_SCANLINE_SM3;
#endif
#endif
        pio_sm_enable_mask(video_pio, sm_mask, false);

        if (enable) {
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM, pio_encode_jmp(video_mode.pio_program->entry_point));
#if PICO_SCANVIDEO_PLANE_COUNT > 1
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM2, pio_encode_jmp(video_mode.pio_program->entry_point));
#if PICO_SCANVIDEO_PLANE_COUNT > 2
            pio_sm_exec(video_pio, PICO_SCANVIDEO_SCANLINE_SM3, pio_encode_jmp(video_mode.pio_program->entry_point));
#endif
#endif
            pio_sm_exec(video_pio, TIMING_SM,
                        pio_encode_jmp(video_dbi_control_load_offset + video_dbi_control_offset_entry_point));
            pio_sm_enable_mask(video_pio, sm_mask, true);
        }
        video_timing_enabled = enable;
    }
}

uint32_t video_wait_for_scanline_complete(uint32_t scanline_id) {
//    // next_scanline_id is potentially the scanline_id in progress, so we need next_scanline_id to
//    // be more than the scanline_id after the passed one
//    scanline_id = scanline_id_after(scanline_id);
//    uint32_t frame = frame_number(scanline_id);
//    uint32_t next_scanline_id;
//    // scanline_id > video_get_next_scanline_id() but with wrapping support
//    while (0 < (scanline_id - (next_scanline_id = video_get_next_scanline_id()))) {
//        // we may end up waiting for the next scanline while in vblank; the one we are waiting for is clearly done
//        if (video_in_vblank() && (frame_number(next_scanline_id) - frame) >= 1)
//            break;
//        assert(video_timing_enabled); // todo should we just return
//        __wfe();
//    }
//    return next_scanline_id;
    assert(false);
}

void video_wait_for_vblank() {
    sem_acquire(&vblank_begin);
}

#ifndef NDEBUG

// todo this is for composable only atm
void validate_scanline(const uint32_t *dma_data, uint dma_data_size,
                       uint max_pixels, uint expected_width) {
    const uint16_t *it = (uint16_t *) dma_data;
    assert(!(3u & (uintptr_t) dma_data));
    const uint16_t *const dma_data_end = (uint16_t * )(dma_data + dma_data_size);
    uint16_t *pixel_buffer = 0;
    const uint16_t *const pixels_end = (uint16_t * )(pixel_buffer + max_pixels);
    uint16_t *pixels = pixel_buffer;
    bool ok = false;
    bool done = false;
    bool had_black = false;
    do {
        uint16_t cmd = *it++;
        switch (cmd) {
            case video_24mhz_composable_program_extern(end_of_scanline_skip_word_ALIGN):
                it++;
                // fall thru
            case video_24mhz_composable_program_extern(end_of_scanline_ALIGN):
                done = ok = true;
                break;
            case video_24mhz_composable_program_extern(color_run): {
                it++;
                uint16_t len = *it++;
                for (int i = 0; i < len + 3; i++) {
                    assert(pixels < pixels_end);
                    pixels++;
                }
                break;
            }
            case video_24mhz_composable_program_extern(raw_run): {
                assert(pixels < pixels_end);
                pixels++;
                it++;
                uint16_t len = *it++;
                for (int i = 0; i < len + 2; i++) {
                    assert(pixels < pixels_end);
                    pixels++;
                    it++;
                }
                break;
            }
            case video_24mhz_composable_program_extern(raw_2p):
                assert(pixels < pixels_end);
                pixels++;
                it++;
                // fall thru
            case video_24mhz_composable_program_extern(raw_1p):
                if (pixels == pixels_end) {
                    assert(!had_black);
                    uint c = *it++;
                    assert(!c); // must end with black
                    had_black = true;
                } else {
                    assert(pixels < pixels_end);
                    pixels++;
                    it++;
                }
                break;
#if !PICO_SCANVIDEO_USE_RAW1P_2CYCLE
            case video_24mhz_composable_program_extern(raw_1p_skip_word_ALIGN):
                assert(pixels < pixels_end);
                pixels++;
                it++;
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
    assert(!(3u & (uintptr_t)(it))); // should end on dword boundary
    assert(!expected_width || pixels == pixel_buffer +
                                        expected_width); // with the correct number of pixels (one more because we stick a black pixel on the end)
    assert(had_black);
}

#endif
