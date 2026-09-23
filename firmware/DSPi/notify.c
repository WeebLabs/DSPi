/*
 * notify.c — Device→host notification subsystem (v2 protocol).
 *
 * See Documentation/Features/notification_protocol_v2_spec.md for design.
 *
 * Internal layout:
 *   - param_shadow: a live copy of WireBulkParams used to detect byte-level
 *     changes on every param_write call.  Populated once at init, kept in
 *     sync by param_write, re-baselined on bulk operations.
 *   - notify_ring: FIFO of pending events with one producer and multiple
 *     independent consumers (USB EP 0x83 drain, UART notification frames),
 *     each owning its own tail.  Short interrupt-disabled critical sections
 *     guard head/tails/entry mutation.
 *
 * Concurrency model:
 *   - Single producer (main thread).  If this ever changes (e.g. Core 1
 *     writes parameters), promote notify_current_source to per-core and
 *     add a lock around push, or switch to an atomic ring.
 *   - All consumers drain from main-thread context (usb_notify_tick and
 *     the EP 0x83 xfer_cb both run on the TinyUSB task = main thread;
 *     uart_ctrl_poll is the main loop).  A lagging consumer never blocks
 *     the producer or another consumer: the producer force-drops the
 *     laggard's oldest entry to make room, and the consumer detects the
 *     loss as a sequence gap and re-reads full state.
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
    uint8_t  seq;           // Stamped at push for v2 events (v1 entries carry
                            // no seq and do not consume one), so every
                            // consumer can detect its own drops as a gap.
    uint16_t wire_offset;   // For PARAM_CHANGED; unused for others
    uint16_t wire_size;     // For PARAM_CHANGED; unused for others
    uint8_t  value[NOTIFY_MAX_PAYLOAD];  // Payload (or 1-byte slot for PRESET_LOADED)
} NotifyRingEntry;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

// Shadow mirror of live state in wire format.  param_write compares new
// writes against this to decide whether a notification is needed.
// Same size on both platforms (the wire format is platform-independent).
static __attribute__((aligned(4))) WireBulkParams param_shadow;

// Event ring.  head = next-free slot; each consumer owns a tail
// (next-to-send for that consumer).  head == tail_c → empty for c.  The
// producer never fails a push: if advancing head would collide with an
// active consumer's tail, that tail is force-advanced first (the lagging
// consumer drops its oldest entry, counted per consumer).
static NotifyRingEntry notify_ring[NOTIFY_RING_SIZE];
static volatile uint8_t notify_head = 0;
static volatile uint8_t notify_tails[NOTIFY_CONSUMER_COUNT];
static volatile bool    notify_active[NOTIFY_CONSUMER_COUNT];
// Ring index captured by the last peek, per consumer, so commit can detect
// that the producer force-dropped past it (0xFF = no peek outstanding).
static volatile uint8_t notify_peek_idx[NOTIFY_CONSUMER_COUNT];

// Monotonic sequence counter stamped onto every v2 entry at PUSH time (not
// at drain time: a re-peeked entry must not burn a number, and per-consumer
// gap detection needs the seq to identify the event, not the transmission).
static volatile uint8_t notify_seq = 0;

// Active source tag for the current setter call.  Defaults to UNKNOWN;
// caller sets this inside a scoped bracket.
static volatile uint8_t notify_current_source = PARAM_SRC_UNKNOWN;

// Nesting depth for bulk operations.  While > 0, per-field pushes are
// suppressed (shadow still updates).  Last end() pushes one BULK_INVALIDATED.
static volatile uint8_t notify_bulk_depth = 0;
static volatile uint8_t notify_bulk_source = PARAM_SRC_UNKNOWN;

// Diagnostics
volatile uint32_t notify_consumer_drops[NOTIFY_CONSUMER_COUNT];
volatile uint32_t notify_overflow_count = 0;
volatile uint32_t notify_drops_count = 0;

// ---------------------------------------------------------------------------
// Ring primitives (all callers hold interrupts disabled)
// ---------------------------------------------------------------------------

// Start index of the fully-unsent window: entries no ACTIVE consumer has
// consumed yet.  That is the fastest consumer's tail (fewest pending).
// Coalescing may only mutate entries in this window; touching an entry a
// faster consumer already sent would make that consumer miss the update.
static uint8_t ring_coalesce_start_locked(void) {
    uint8_t min_pending = NOTIFY_RING_SIZE;   // > any real count
    for (int c = 0; c < NOTIFY_CONSUMER_COUNT; c++) {
        if (!notify_active[c]) continue;
        uint8_t pending = (uint8_t)(notify_head - notify_tails[c]) & NOTIFY_RING_MASK;
        if (pending < min_pending) min_pending = pending;
    }
    if (min_pending == NOTIFY_RING_SIZE) return notify_head;   // no active consumer
    return (uint8_t)(notify_head - min_pending) & NOTIFY_RING_MASK;
}

// Find a fully-unsent PARAM_CHANGED entry matching (offset, size).  Returns
// the index or 0xFF if not found.
static uint8_t ring_find_coalesce_locked(uint16_t offset, uint16_t size) {
    uint8_t i = ring_coalesce_start_locked();
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

// Find a fully-unsent BULK_INVALIDATED entry.  At most one is ever useful.
static uint8_t ring_find_invalidated_locked(void) {
    uint8_t i = ring_coalesce_start_locked();
    while (i != notify_head) {
        if (notify_ring[i].event_id == NOTIFY_EVT_BULK_INVALIDATED) return i;
        i = (i + 1) & NOTIFY_RING_MASK;
    }
    return 0xFF;
}

// Append entry.  Never fails: any active consumer whose tail would collide
// with the new head is force-advanced first (it drops its oldest entry).
// v2 entries are stamped with a push-time sequence number; the v1 legacy
// master-volume entry carries no seq byte on the wire and must not consume
// one (a skipped v1 entry would otherwise read as a drop to the UART
// consumer on every volume change).
static void ring_push_locked(NotifyRingEntry *e) {
    uint8_t next = (uint8_t)(notify_head + 1) & NOTIFY_RING_MASK;
    for (int c = 0; c < NOTIFY_CONSUMER_COUNT; c++) {
        if (notify_active[c] && notify_tails[c] == next) {
            // Dropping a v1 entry a non-USB consumer would have skipped
            // anyway is not an observable loss; keep the counter honest so
            // it always corresponds to a real seq gap for that consumer.
            bool observable =
                (notify_ring[notify_tails[c]].event_id != NOTIFY_EVT_MASTER_VOLUME ||
                 c == NOTIFY_CONSUMER_USB);
            notify_tails[c] = (uint8_t)(notify_tails[c] + 1) & NOTIFY_RING_MASK;
            if (observable) {
                notify_consumer_drops[c]++;
                notify_overflow_count++;
                notify_drops_count++;
            }
        }
    }
    e->seq = (e->event_id == NOTIFY_EVT_MASTER_VOLUME) ? 0 : ++notify_seq;
    notify_ring[notify_head] = *e;
    notify_head = next;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void notify_init(void) {
    notify_head = 0;
    notify_seq  = 0;
    for (int c = 0; c < NOTIFY_CONSUMER_COUNT; c++) {
        notify_tails[c] = 0;
        notify_peek_idx[c] = 0xFF;
        notify_active[c] = false;
        notify_consumer_drops[c] = 0;
    }
    notify_active[NOTIFY_CONSUMER_USB] = true;   // USB drain always runs
    notify_current_source = PARAM_SRC_UNKNOWN;
    notify_bulk_depth = 0;
    notify_bulk_source = PARAM_SRC_UNKNOWN;
    notify_overflow_count = 0;
    notify_drops_count = 0;
    bulk_params_collect(&param_shadow);
}

void notify_consumer_set_active(NotifyConsumer c, bool active) {
    if (c >= NOTIFY_CONSUMER_COUNT) return;
    uint32_t flags = save_and_disable_interrupts();
    if (active && !notify_active[c]) {
        notify_tails[c] = notify_head;   // see events from now on, no backlog
        notify_peek_idx[c] = 0xFF;
    }
    notify_active[c] = active;
    restore_interrupts(flags);
}

void notify_rebaseline(void) {
    // Collects in place with no scratch copy or IRQ lock.  Safe only while
    // every param_write runs on the Core 0 main loop (see header comment).
    bulk_params_collect(&param_shadow);
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

    // Look for a coalesce target (fully-unsent entries only).  If found,
    // overwrite its value in place; its push-time seq stands, since only
    // one packet will ever go out for it.
    uint8_t idx = ring_find_coalesce_locked(wire_offset, size);
    if (idx != 0xFF) {
        NotifyRingEntry *e = &notify_ring[idx];
        memcpy(e->value, src, size);
        e->source = source;   // latest source wins
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

    // Coalesce: find a fully-unsent MASTER_VOLUME entry (at most one).
    uint8_t i = ring_coalesce_start_locked();
    while (i != notify_head) {
        if (notify_ring[i].event_id == NOTIFY_EVT_MASTER_VOLUME) {
            memcpy(notify_ring[i].value, &db, sizeof(float));
            restore_interrupts(flags);
            return;
        }
        i = (i + 1) & NOTIFY_RING_MASK;
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
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_PRESET_LOADED,
        .source   = PARAM_SRC_PRESET,
    };
    e.value[0] = slot;
    ring_push_locked(&e);
    restore_interrupts(flags);
}

void notify_push_input_format(uint8_t channels) {
    uint32_t flags = save_and_disable_interrupts();
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_INPUT_FORMAT,
        .source   = PARAM_SRC_INTERNAL,
    };
    e.value[0] = channels;
    ring_push_locked(&e);
    restore_interrupts(flags);
}

// Implemented unconditionally so it links on RP2040 (where ADAT is absent and
// this is never called).
void notify_push_adat_state(uint8_t enabled, uint8_t active, uint8_t pin) {
    uint32_t flags = save_and_disable_interrupts();
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_ADAT_STATE,
        .source   = PARAM_SRC_INTERNAL,
    };
    e.value[0] = enabled;
    e.value[1] = active;
    e.value[2] = pin;
    ring_push_locked(&e);
    restore_interrupts(flags);
}

// Implemented unconditionally so it links on both platforms.
void notify_push_i2s_slave_state(uint8_t state, uint32_t rate_hz) {
    uint32_t flags = save_and_disable_interrupts();
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_I2S_SLAVE_STATE,
        .source   = PARAM_SRC_INTERNAL,
    };
    e.value[0] = state;
    memcpy(&e.value[1], &rate_hz, 4);
    ring_push_locked(&e);
    restore_interrupts(flags);
}

// Implemented unconditionally so it links on RP2040 (where ADAT input is
// absent and this is never called).
void notify_push_adat_input_state(uint8_t state, uint32_t rate_hz,
                                  uint8_t clock_mode) {
    uint32_t flags = save_and_disable_interrupts();
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_ADAT_INPUT_STATE,
        .source   = PARAM_SRC_INTERNAL,
    };
    e.value[0] = state;
    memcpy(&e.value[1], &rate_hz, 4);
    e.value[5] = clock_mode;
    ring_push_locked(&e);
    restore_interrupts(flags);
}

void notify_push_siggen_state(uint8_t state, uint8_t reason,
                              uint8_t signal_type, uint8_t channel) {
    uint32_t flags = save_and_disable_interrupts();
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_SIGGEN_STATE,
        .source   = PARAM_SRC_INTERNAL,
    };
    e.value[0] = state;
    e.value[1] = reason;
    e.value[2] = signal_type;
    e.value[3] = channel;
    ring_push_locked(&e);
    restore_interrupts(flags);
}

void notify_push_cs_ir_learn(uint8_t state, uint8_t protocol, uint32_t code) {
    uint32_t flags = save_and_disable_interrupts();
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_CS_IR_LEARN,
        .source   = PARAM_SRC_INTERNAL,
    };
    e.value[0] = state;
    e.value[1] = protocol;
    e.value[2] = (uint8_t)(code & 0xFF);
    e.value[3] = (uint8_t)((code >> 8) & 0xFF);
    e.value[4] = (uint8_t)((code >> 16) & 0xFF);
    e.value[5] = (uint8_t)((code >> 24) & 0xFF);
    ring_push_locked(&e);
    restore_interrupts(flags);
}

// Source is the caller's, not INTERNAL: an aux can be driven by a bound
// control as well as by a host or transport, and the host filters on it.
void notify_push_cs_aux(uint8_t slot, uint8_t state, uint16_t level_q8, ParamSource src) {
    uint32_t flags = save_and_disable_interrupts();
    NotifyRingEntry e = {
        .event_id = NOTIFY_EVT_CS_AUX,
        .source   = (uint8_t)src,
    };
    e.value[0] = slot;
    e.value[1] = state;
    e.value[2] = (uint8_t)(level_q8 & 0xFF);
    e.value[3] = (uint8_t)(level_q8 >> 8);
    ring_push_locked(&e);
    restore_interrupts(flags);
}

void notify_push_bulk_invalidated(ParamSource src) {
    uint32_t flags = save_and_disable_interrupts();

    // Coalesce: if a fully-unsent invalidation is already queued, just
    // refresh its source.  One invalidation covers any number of causes.
    // (The pre-multi-consumer code also had a displace-oldest-on-full path
    // here; ring_push_locked's force-advance now guarantees delivery
    // without it, at the cost of the laggard's oldest entry.)
    uint8_t idx = ring_find_invalidated_locked();
    if (idx != 0xFF) {
        notify_ring[idx].source = (uint8_t)src;
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

bool notify_has_pending_for(NotifyConsumer c) {
    if (c >= NOTIFY_CONSUMER_COUNT || !notify_active[c]) return false;
    return notify_head != notify_tails[c];
}

void notify_reset_queue(void) {
    // USB bus reset: drop the USB consumer's backlog (the host re-syncs
    // with a full REQ_GET_ALL_PARAMS on reconnect) and clear the global
    // source/bulk brackets.  Other consumers' backlogs and the sequence
    // counter are untouched; resetting seq would fake a wrap-around gap
    // for a mid-stream UART consumer.
    uint32_t flags = save_and_disable_interrupts();
    notify_tails[NOTIFY_CONSUMER_USB] = notify_head;
    notify_peek_idx[NOTIFY_CONSUMER_USB] = 0xFF;
    notify_bulk_depth = 0;
    notify_current_source = PARAM_SRC_UNKNOWN;
    restore_interrupts(flags);
}

uint16_t notify_peek_next_for(NotifyConsumer c, uint8_t *out_buf, uint16_t max_len) {
    if (out_buf == NULL || c >= NOTIFY_CONSUMER_COUNT) return 0;

  next_entry:;
    uint32_t flags = save_and_disable_interrupts();

    if (!notify_active[c] || notify_head == notify_tails[c]) {
        restore_interrupts(flags);
        return 0;
    }

    uint8_t idx = notify_tails[c];
    NotifyRingEntry e = notify_ring[idx];
    notify_peek_idx[c] = idx;

    restore_interrupts(flags);

    // The v1 legacy master-volume packet exists only for pre-v2 USB hosts;
    // every v1 event has a v2 PARAM_CHANGED twin, so other consumers skip
    // it (it carries no seq, so skipping never reads as a drop).
    if (e.event_id == NOTIFY_EVT_MASTER_VOLUME && c != NOTIFY_CONSUMER_USB) {
        notify_commit_pop_for(c);
        goto next_entry;
    }

    uint8_t seq = e.seq;

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

        case NOTIFY_EVT_INPUT_FORMAT: {
            // 8 bytes: active input channel count in byte 4.
            if (max_len < 8) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_INPUT_FORMAT;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.value[0];
            out_buf[5] = 0;
            out_buf[6] = 0;
            out_buf[7] = 0;
            return 8;
        }

        case NOTIFY_EVT_ADAT_STATE: {
            // 8 bytes: enabled, active, data pin in bytes 4..6.
            if (max_len < 8) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_ADAT_STATE;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.value[0];
            out_buf[5] = e.value[1];
            out_buf[6] = e.value[2];
            out_buf[7] = 0;
            return 8;
        }

        case NOTIFY_EVT_I2S_SLAVE_STATE: {
            // 9 bytes: state in byte 4, detected rate (Hz, LE) in bytes 5..8.
            if (max_len < 9) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_I2S_SLAVE_STATE;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.value[0];
            memcpy(&out_buf[5], &e.value[1], 4);
            return 9;
        }

        case NOTIFY_EVT_SIGGEN_STATE: {
            // 8 bytes: state, stop reason, signal type, walk channel.
            if (max_len < 8) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_SIGGEN_STATE;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.value[0];
            out_buf[5] = e.value[1];
            out_buf[6] = e.value[2];
            out_buf[7] = e.value[3];
            return 8;
        }

        case NOTIFY_EVT_ADAT_INPUT_STATE: {
            // 10 bytes: state, detected rate (Hz, LE), clock mode.
            if (max_len < 10) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_ADAT_INPUT_STATE;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.value[0];
            memcpy(&out_buf[5], &e.value[1], 4);
            out_buf[9] = e.value[5];
            return 10;
        }

        case NOTIFY_EVT_CS_IR_LEARN: {
            // 12 bytes: learn state, protocol, pad, pad, learned code LE.
            if (max_len < 12) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_CS_IR_LEARN;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.value[0];
            out_buf[5] = e.value[1];
            out_buf[6] = 0;
            out_buf[7] = 0;
            out_buf[8]  = e.value[2];
            out_buf[9]  = e.value[3];
            out_buf[10] = e.value[4];
            out_buf[11] = e.value[5];
            return 12;
        }

        case NOTIFY_EVT_CS_AUX: {
            // 9 bytes: slot, state, level (8.8 LE), source.
            if (max_len < 9) return 0;
            out_buf[0] = NOTIFY_V2_VERSION;
            out_buf[1] = NOTIFY_EVT_CS_AUX;
            out_buf[2] = 0;
            out_buf[3] = seq;
            out_buf[4] = e.value[0];
            out_buf[5] = e.value[1];
            out_buf[6] = e.value[2];
            out_buf[7] = e.value[3];
            out_buf[8] = e.source;
            return 9;
        }

        default:
            // Unknown event (shouldn't happen).  Skip it and try the next
            // entry rather than returning 0 with data still pending.
            notify_commit_pop_for(c);
            goto next_entry;
    }
}

void notify_commit_pop_for(NotifyConsumer c) {
    if (c >= NOTIFY_CONSUMER_COUNT) return;
    uint32_t flags = save_and_disable_interrupts();
    // Advance only if the producer's force-drop has not already moved this
    // consumer's tail past the entry the peek captured.
    if (notify_head != notify_tails[c] &&
        notify_tails[c] == notify_peek_idx[c]) {
        notify_tails[c] = (uint8_t)(notify_tails[c] + 1) & NOTIFY_RING_MASK;
    }
    notify_peek_idx[c] = 0xFF;
    restore_interrupts(flags);
}
