#ifndef NOTIFY_H
#define NOTIFY_H

/*
 * notify.h — Device→host notification subsystem.
 *
 * See Documentation/Features/notification_protocol_v2_spec.md for design.
 *
 * The protocol identifies every parameter by its offset into WireBulkParams.
 * A single generic event (PARAM_CHANGED) covers every parameter change; the
 * host dispatches on offset rather than a hand-written switch.  Adding a
 * new parameter requires no wire-format changes and no host-side changes
 * beyond a handler registration.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Wire-level event IDs
// ---------------------------------------------------------------------------

// Idle keep-alive (v1-compatible single-byte packet). Reserved.
#define NOTIFY_EVT_IDLE              0x00

// v1 legacy master-volume packet (8 bytes: [0x01, 0, 0, 0, float_db_LE]).
// Emitted in addition to v2 PARAM_CHANGED so existing v1 hosts keep working.
#define NOTIFY_EVT_MASTER_VOLUME     0x01

// v2 generic parameter-change event.
// Packet: [ver=2, evt=0x02, flags=0, seq, off_LE, size_LE, src, 0, 0, 0, value...]
#define NOTIFY_EVT_PARAM_CHANGED     0x02

// v2 bulk-invalidation event.  Host should re-read REQ_GET_ALL_PARAMS.
// Packet: [ver=2, evt=0x03, flags=0, seq, src, 0, 0, 0]
#define NOTIFY_EVT_BULK_INVALIDATED  0x03

// v2 preset-loaded event (always followed by BULK_INVALIDATED).
// Packet: [ver=2, evt=0x04, flags=0, seq, slot, 0, 0, 0]
#define NOTIFY_EVT_PRESET_LOADED     0x04

// Reserved for future use.
// 0x05 — error / overflow report
// 0x06 — batched PARAM_CHANGED (see spec §10.5)

// v2 protocol version byte (first byte of every v2 packet)
#define NOTIFY_V2_VERSION            0x02

// ---------------------------------------------------------------------------
// Source tags
// ---------------------------------------------------------------------------

typedef enum {
    PARAM_SRC_UNKNOWN  = 0,  // Default; origin not explicitly marked
    PARAM_SRC_HOST_SET = 1,  // EP0 REQ_SET_* from host
    PARAM_SRC_BULK_SET = 2,  // REQ_SET_ALL_PARAMS
    PARAM_SRC_PRESET   = 3,  // Preset slot applied
    PARAM_SRC_FACTORY  = 4,  // Factory reset
    PARAM_SRC_GPIO     = 5,  // Hardware control (knobs, encoders, pads)
    PARAM_SRC_INTERNAL = 6,  // Firmware-initiated (clamp, auto-recalc)
} ParamSource;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialise the subsystem.  Call once after live DSP state is set up but
// before any param_write calls.  Populates param_shadow via bulk_params_collect.
void notify_init(void);

// Re-baseline the shadow from live state.  Call after any wholesale state
// rewrite (preset load, factory reset, bulk SET).
void notify_rebaseline(void);

// Scoped source tag.  A setter called inside a source bracket will attach
// that tag to its notification.  The tag is a single global (see spec §4.4).
void notify_set_source(ParamSource src);

// Bulk-operation bracket.  While an outer bulk is active, per-field
// notify_param_write calls only update the shadow — they do not push
// PARAM_CHANGED events.  The matching end() pushes exactly one
// BULK_INVALIDATED when the outermost bulk closes.
//
// Nests safely; use paired begin/end.
void notify_begin_bulk(ParamSource src);
void notify_end_bulk(void);

// Write a parameter's wire value and, if bytes differ from the shadow, push
// a PARAM_CHANGED event.  `wire_offset` must be offsetof(WireBulkParams, ...)
// and `size` must match the field's sizeof.  Size must be <= 52.
//
// Caller is responsible for having applied the live-state update first
// (this function only handles shadow + notification).
void notify_param_write(uint16_t wire_offset,
                        uint16_t size,
                        const void *src);

// Push a v1-compatible master-volume event (8-byte legacy packet).  Keeps
// existing v1 host apps working during the transition.  Called from
// update_master_volume() alongside notify_param_write().
void notify_push_master_volume_v1(float db);

// Push discrete v2 events.
void notify_push_preset_loaded(uint8_t slot);
void notify_push_bulk_invalidated(ParamSource src);

// Drain interface used by usb_audio.c.
//
// peek: format the next pending packet into out_buf (max max_len bytes).
//       Returns the packet length (always > 0 if pending, 0 if ring empty).
//       Does NOT advance the tail — call notify_commit_pop() on successful
//       xfer submission.
uint16_t notify_peek_next(uint8_t *out_buf, uint16_t max_len);

// Commit the last peek — advances the ring tail.  Must be called iff the
// packet returned by peek was successfully submitted to the USB stack.
void notify_commit_pop(void);

// True when at least one entry is pending delivery.  Lock-free fast path.
bool notify_has_pending(void);

// Clear all pending state.  Called on USB reset.
void notify_reset_queue(void);

// Debug / diagnostics.
extern volatile uint32_t notify_overflow_count;
extern volatile uint32_t notify_drops_count;

#endif // NOTIFY_H
