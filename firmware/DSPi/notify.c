/*
 * notify.c — Device→host notification subsystem (v2 protocol).
 *
 * See Documentation/Features/notification_protocol_v2_spec.md for design.
 *
 * Internal layout:
 *   - param_shadow: a live copy of WireBulkParams used to detect byte-level
 *     changes on every param_write call.  Populated once at init, kept in
 *     sync by param_write, re-baselined on bulk operations.
 *   - notify_ring: SPSC-style FIFO of pending events.  Producer is the main
 *     thread via notify_* push functions; consumer is usb_audio.c's drain
 *     (also main thread, but can interleave with xfer_cb on USB IRQ).
 *     Short interrupt-disabled critical sections guard head/tail/entry
 *     mutation.
 *
 * Concurrency model:
 *   - Single producer (main thread).  If this ever changes (e.g. Core 1
 *     writes parameters), promote notify_current_source to per-core and
 *     add a lock around push, or switch to an atomic ring.
 *   - Single consumer (drain in usb_audio.c).  Called from the main loop
 *     tick AND from xfer_cb on EP 0x83 completion; the xfer_cb path runs
 *     on the TinyUSB task, which is the main thread in our cooperative
 *     scheduler — so still effectively single-consumer.
 */

#include "notify.h"
#include "bulk_params.h"
#include "config.h"

#include <string.h>
#include "hardware/sync.h"
#include "pico/platform.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Ring depth.  Must be a power of two.  Spec recommends 32 to absorb bursts
// from preset-load flushes (though those are coalesced into BULK_INVALIDATED
// so the typical peak is small).
#define NOTIFY_RING_SIZE      32
#define NOTIFY_RING_MASK      (NOTIFY_RING_SIZE - 1)

// Maximum PARAM_CHANGED value payload.  Covers every WireBulkParams field
// (largest is WireChannelNames row at 32 B).  Total packet = 12 + this.
#define NOTIFY_MAX_PAYLOAD    52

// Maximum wire-level packet size.  Must match NOTIFY_EP_MAX_PKT in
// usb_descriptors.h.  Sized at 64 (USB 2.0 full-speed bulk cap).
#define NOTIFY_MAX_PACKET     64

// ---------------------------------------------------------------------------
// Ring entry
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  event_id;      // NOTIFY_EVT_*
    uint8_t  source;        // ParamSource
    uint16_t wire_offset;   // For PARAM_CHANGED; unused for others
    uint16_t wire_size;     // For PARAM_CHANGED; unused for others
    uint8_t  value[NOTIFY_MAX_PAYLOAD];  // Payload (or 1-byte slot for PRESET_LOADED)
} NotifyRingEntry;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

// Shadow mirror of live state in wire format.  param_write compares new
// writes against this to decide whether a notification is needed.
// 2912 B on RP2350 / RP2040 alike.  Placed in RAM so reads don't stall on XIP.
static __attribute__((aligned(4))) WireBulkParams param_shadow;

// Event ring.  head = next-free slot, tail = next-to-send.
// When head == tail → empty; when (head+1)&MASK == tail → full.
static NotifyRingEntry notify_ring[NOTIFY_RING_SIZE];
static volatile uint8_t notify_head = 0;
static volatile uint8_t notify_tail = 0;

// Monotonic sequence counter stamped onto every v2 packet.
static volatile uint8_t notify_seq = 0;

// Active source tag for the current setter call.  Defaults to UNKNOWN;
// caller sets this inside a scoped bracket.
static volatile uint8_t notify_current_source = PARAM_SRC_UNKNOWN;

// Nesting depth for bulk operations.  While > 0, per-field pushes are
// suppressed (shadow still updates).  Last end() pushes one BULK_INVALIDATED.
static volatile uint8_t notify_bulk_depth = 0;
static volatile uint8_t notify_bulk_source = PARAM_SRC_UNKNOWN;

// Diagnostics
volatile uint32_t notify_overflow_count = 0;
volatile uint32_t notify_drops_count = 0;

// ---------------------------------------------------------------------------
// Ring primitives (all callers hold interrupts disabled)
// ---------------------------------------------------------------------------

static inline bool ring_empty_locked(void) {
    return notify_head == notify_tail;
}

static inline bool ring_full_locked(void) {
    return ((uint8_t)(notify_head + 1) & NOTIFY_RING_MASK) == notify_tail;
}

static inline uint8_t ring_count_locked(void) {
    return (uint8_t)(notify_head - notify_tail) & NOTIFY_RING_MASK;
}

// Find an unsent PARAM_CHANGED entry matching (offset, size).  Returns the
// index or 0xFF if not found.  Only scans occupied slots.
static uint8_t ring_find_coalesce_locked(uint16_t offset, uint16_t size) {
    uint8_t i = notify_tail;
    while (i != notify_head) {
        NotifyRingEntry *e = &notify_ring[i];
        if (e->event_id == NOTIFY_EVT_PARAM_CHANGED &&
            e->wire_offset == offset &&
            e->wire_size == size) {
            return i;
        }
        i = (i + 1) & NOTIFY_RING_MASK;
    }
    return 0xFF;
}

// Find an unsent BULK_INVALIDATED entry.  At most one is ever useful.
static uint8_t ring_find_invalidated_locked(void) {
    uint8_t i = notify_tail;
    while (i != notify_head) {
        if (notify_ring[i].event_id == NOTIFY_EVT_BULK_INVALIDATED) return i;
        i = (i + 1) & NOTIFY_RING_MASK;
    }
    return 0xFF;
}

// Append entry.  Caller must ensure the ring is not full.
static void ring_push_locked(const NotifyRingEntry *e) {
    notify_ring[notify_head] = *e;
    notify_head = (notify_head + 1) & NOTIFY_RING_MASK;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void notify_init(void) {
    notify_head = 0;
    notify_tail = 0;
    notify_seq  = 0;
    notify_current_source = PARAM_SRC_UNKNOWN;
    notify_bulk_depth = 0;
    notify_bulk_source = PARAM_SRC_UNKNOWN;
    notify_overflow_count = 0;
    notify_drops_count = 0;
    bulk_params_collect(&param_shadow);
}

void notify_rebaseline(void) {
    // Collect into a scratch buffer first so we don't clobber shadow state
    // if collect() reads from any volatile globals racily.  Our collect is
    // synchronous with live state, so this is defensive rather than strictly
    // necessary.
    WireBulkParams tmp;
    bulk_params_collect(&tmp);
    uint32_t flags = save_and_disable_interrupts();
    memcpy(&param_shadow, &tmp, sizeof(param_shadow));
    restore_interrupts(flags);
}

void notify_set_source(ParamSource src) {
    notify_current_source = (uint8_t)src;
}

void notify_begin_bulk(ParamSource src) {
    uint32_t flags = save_and_disable_interrupts();
    if (notify_bulk_depth == 0) {
        notify_bulk_source = (uint8_t)src;
    }
    notify_bulk_depth++;
    restore_interrupts(flags);
}

void notify_end_bulk(void) {
    uint32_t flags = save_and_disable_interrupts();
    if (notify_bulk_depth > 0) notify_bulk_depth--;
    bool last = (notify_bulk_depth == 0);
    uint8_t src = notify_bulk_source;
    restore_interrupts(flags);

    if (!last) return;

    // Outermost bulk closed: rebaseline shadow, then push BULK_INVALIDATED.
    notify_rebaseline();
    notify_push_bulk_invalidated((ParamSource)src);
}

void notify_param_write(uint16_t wire_offset,
                        uint16_t size,
                        const void *src) {
    if (size == 0 || size > NOTIFY_MAX_PAYLOAD) return;
    if ((uint32_t)wire_offset + size > sizeof(WireBulkParams)) return;

    uint8_t *shadow_p = (uint8_t *)&param_shadow + wire_offset;

    uint32_t flags = save_and_disable_interrupts();

    bool changed = (memcmp(shadow_p, src, size) != 0);
    if (changed) {
        memcpy(shadow_p, src, size);
    }
    bool suppress = (notify_bulk_depth > 0);
    uint8_t source = notify_current_source;

    if (!changed || suppress) {
        restore_interrupts(flags);
        return;
    }

    // Look for a coalesce target.  If found, overwrite its value in place.
    uint8_t idx = ring_find_coalesce_locked(wire_offset, size);
    if (idx != 0xFF) {
        NotifyRingEntry *e = &notify_ring[idx];
        memcpy(e->value, src, size);
        e->source = source;   // latest source wins
        restore_interrupts(flags);
        return;
    }

    if (ring_full_locked()) {
        notify_overflow_count++;
        notify_drops_count++;
        restore_interrupts(flags);
        return;
    }

    NotifyRingEntry e = {
        .event_id    = NOTIFY_EVT_PARAM_CHANGED,
        .source      = source,
        .wire_offset = wire_offset,
        .wire_size   = size,
    };
    memcpy(e.value, src, size);
    ring_push_locked(&e);

    restore_interrupts(flags);
}

void notify_push_master_volume_v1(float db) {
    uint32_t flags = save_and_disable_interrupts();

    // Coalesce: find an unsent MASTER_VOLUME entry (there is at most one).
    uint8_t i = notify_tail;
    while (i != notify_head) {
        if (notify_ring[i].event_id == NOTIFY_EVT_MASTER_VOLUME) {
            memcpy(notify_ring[i].value, &db, sizeof(float));
            restore_interrupts(flags);
            return;
        }
        i = (i + 1) & NOTIFY_RING_MASK;
    }

    if (ring_full_locked()) {
        notify_overflow_count++;
        notify_drops_count++;
        restore_interrupts(flags);
        return;
    }

    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_MASTER_VOLUME,
        .source   = PARAM_SRC_UNKNOWN,
        .wire_offset = 0,
        .wire_size   = 0,
    };
    memcpy(e.value, &db, sizeof(float));
    ring_push_locked(&e);

    restore_interrupts(flags);
}

void notify_push_preset_loaded(uint8_t slot) {
    uint32_t flags = save_and_disable_interrupts();
    if (ring_full_locked()) {
        notify_overflow_count++;
        notify_drops_count++;
        restore_interrupts(flags);
        return;
    }
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_PRESET_LOADED,
        .source   = PARAM_SRC_PRESET,
    };
    e.value[0] = slot;
    ring_push_locked(&e);
    restore_interrupts(flags);
}

void notify_push_bulk_invalidated(ParamSource src) {
    uint32_t flags = save_and_disable_interrupts();

    // Coalesce: if an unsent invalidation is already queued, just refresh
    // its source.  One invalidation is sufficient regardless of cause.
    uint8_t idx = ring_find_invalidated_locked();
    if (idx != 0xFF) {
        notify_ring[idx].source = (uint8_t)src;
        restore_interrupts(flags);
        return;
    }

    // If the ring is full, displace the oldest unsent entry (spec §4.3).
    // BULK_INVALIDATED is always deliverable because the host cannot
    // recover state without knowing it happened.
    if (ring_full_locked()) {
        notify_ring[notify_tail].event_id = NOTIFY_EVT_BULK_INVALIDATED;
        notify_ring[notify_tail].source   = (uint8_t)src;
        notify_ring[notify_tail].wire_offset = 0;
        notify_ring[notify_tail].wire_size   = 0;
        notify_overflow_count++;
        // Do not advance tail; just mutate in place so it'll be next out.
        restore_interrupts(flags);
        return;
    }

    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_BULK_INVALIDATED,
        .source   = (uint8_t)src,
    };
    ring_push_locked(&e);
    restore_interrupts(flags);
}

// ---------------------------------------------------------------------------
// Drain
// ---------------------------------------------------------------------------

bool notify_has_pending(void) {
    return notify_head != notify_tail;
}

void notify_reset_queue(void) {
    uint32_t flags = save_and_disable_interrupts();
    notify_head = 0;
    notify_tail = 0;
    notify_seq  = 0;
    notify_bulk_depth = 0;
    notify_current_source = PARAM_SRC_UNKNOWN;
    restore_interrupts(flags);
}

uint16_t notify_peek_next(uint8_t *out_buf, uint16_t max_len) {
    if (out_buf == NULL) return 0;

    uint32_t flags = save_and_disable_interrupts();

    if (notify_head == notify_tail) {
        restore_interrupts(flags);
        return 0;
    }

    NotifyRingEntry e = notify_ring[notify_tail];
    uint8_t seq = ++notify_seq;

    restore_interrupts(flags);

    // Format the packet.  Every code path writes a bounded number of bytes
    // and returns the length.
    switch (e.event_id) {
        case NOTIFY_EVT_MASTER_VOLUME: {
            // v1 legacy packet: 8 bytes.
            if (max_len < 8) return 0;
            out_buf[0] = NOTIFY_EVT_MASTER_VOLUME;
            out_buf[1] = 0;
            out_buf[2] = 0;
            out_buf[3] = 0;
            memcpy(&out_buf[4], e.value, 4);
            return 8;
        }

        case NOTIFY_EVT_PARAM_CHANGED: {
            // v2 generic: 12 + size bytes.
            uint16_t len = (uint16_t)(12 + e.wire_size);
            if (len > max_len) return 0;
            out_buf[0]  = NOTIFY_V2_VERSION;
            out_buf[1]  = NOTIFY_EVT_PARAM_CHANGED;
            out_buf[2]  = 0;                   // flags
            out_buf[3]  = seq;
            out_buf[4]  = (uint8_t)(e.wire_offset & 0xFF);
            out_buf[5]  = (uint8_t)(e.wire_offset >> 8);
            out_buf[6]  = (uint8_t)(e.wire_size & 0xFF);
            out_buf[7]  = (uint8_t)(e.wire_size >> 8);
            out_buf[8]  = e.source;
            out_buf[9]  = 0;
            out_buf[10] = 0;
            out_buf[11] = 0;
            memcpy(&out_buf[12], e.value, e.wire_size);
            return len;
        }

        case NOTIFY_EVT_BULK_INVALIDATED: {
            // 8 bytes.
            if (max_len < 8) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_BULK_INVALIDATED;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.source;
            out_buf[5] = 0;
            out_buf[6] = 0;
            out_buf[7] = 0;
            return 8;
        }

        case NOTIFY_EVT_PRESET_LOADED: {
            // 8 bytes: slot in byte 4.
            if (max_len < 8) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_PRESET_LOADED;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.value[0];
            out_buf[5] = 0;
            out_buf[6] = 0;
            out_buf[7] = 0;
            return 8;
        }

        default:
            // Unknown event (shouldn't happen).  Skip it by committing the
            // pop without emitting a packet; fall through returns 0.
            notify_commit_pop();
            return 0;
    }
}

void notify_commit_pop(void) {
    uint32_t flags = save_and_disable_interrupts();
    if (notify_head != notify_tail) {
        notify_tail = (notify_tail + 1) & NOTIFY_RING_MASK;
    }
    restore_interrupts(flags);
}
