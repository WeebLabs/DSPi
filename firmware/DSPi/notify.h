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

// v2 input-format event: the active USB input channel count changed (host
// switched the USB alt / format).  The app re-lays-out its mixer/sidebar.
// Packet: [ver=2, evt=0x05, flags=0, seq, channels, 0, 0, 0]
#define NOTIFY_EVT_INPUT_FORMAT      0x05

// Reserved for future use.
// 0x06 — batched PARAM_CHANGED (see spec §10.5)

// v2 test-signal-generator state event (start, stop, completion).
// Packet: [ver=2, evt=0x07, flags=0, seq, state, reason, signal_type, channel]
// state = SiggenState, reason = SIGGEN_STOP_*, channel = walk channel or 0xFF.
#define NOTIFY_EVT_SIGGEN_STATE      0x07

// v2 ADAT bulk-output state event (RP2350): stream started/stopped, including
// rate-policy auto-suspend/resume (see adat_output.h).
// Packet: [ver=2, evt=0x08, flags=0, seq, enabled, active, pin, 0]
#define NOTIFY_EVT_ADAT_STATE        0x08

// v2 I2S clock-slave lock-state event: acquiring, relocking, locked, inactive.
// Payload: state byte + detected rate (Hz, 0 until LOCKED).
// Packet (9 bytes): [ver=2, evt=0x09, flags=0, seq, state, rate_LE(4)]
#define NOTIFY_EVT_I2S_SLAVE_STATE   0x09

// v2 Control Surfaces IR learn completion: capture or timeout.
// Packet: [ver=2, evt=0x0A, flags=0, seq, state, protocol, 0, 0, code_LE32]
// state = CS_IR_LEARN_DONE or CS_IR_LEARN_TIMEOUT; protocol/code are valid
// only for DONE (CS_IR_PROTO_* / the learned code).
#define NOTIFY_EVT_CS_IR_LEARN       0x0A

// v2 ADAT input lock-state event (RP2350): acquiring, syncing, locked,
// relocking, inactive (see adat_input.h).
// Packet (10 bytes): [ver=2, evt=0x0B, flags=0, seq, state, rate_LE(4),
// clock_mode]; rate = 0 unless LOCKED.
#define NOTIFY_EVT_ADAT_INPUT_STATE  0x0B

// v2 Control Surfaces aux output changed (REQ_SET_CS_AUX_STATE / _LEVEL from
// any source, including a bound control).  Both values are sent so a host
// refreshes the slot in one step; src identifies the origin.
// Packet (9 bytes): [ver=2, evt=0x0C, flags=0, seq, slot, state,
// level_q8_LE(2), src]; level is 8.8 percent, 0 on a CS_TYPE_AUX_OUT slot.
#define NOTIFY_EVT_CS_AUX            0x0C

// v2 spectrum analyser applied a staged REQ_RTA_SET_CONFIG, changed or not.
// Never pushed for frame restarts (preset, rate, input, auto-off/start).
// Packet (17 bytes): [ver=2, evt=0x0D, flags=0, seq, RtaConfig as applied
// (12), src]; src is the source of the SET that staged it.
#define NOTIFY_EVT_RTA_CONFIG        0x0D

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
    PARAM_SRC_UAC1     = 7,  // UAC1 Feature Unit SET_CUR (OS volume slider, mute key)
    PARAM_SRC_UART     = 8,  // External UART control interface
    PARAM_SRC_I2C      = 9,  // External I2C target control interface
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
// Active USB input channel count changed (2/4/6/8) — host switched the alt.
void notify_push_input_format(uint8_t channels);
// Test-signal generator started/stopped/completed (see siggen.h).
void notify_push_siggen_state(uint8_t state, uint8_t reason,
                              uint8_t signal_type, uint8_t channel);
// ADAT bulk-output stream state changed (RP2350; see adat_output.h).
void notify_push_adat_state(uint8_t enabled, uint8_t active, uint8_t pin);
// I2S clock-slave lock state changed (see i2s_input.h).
void notify_push_i2s_slave_state(uint8_t state, uint32_t rate_hz);
// ADAT input lock state changed (RP2350; see adat_input.h).
void notify_push_adat_input_state(uint8_t state, uint32_t rate_hz,
                                  uint8_t clock_mode);
// Control Surfaces IR learn finished (state = CS_IR_LEARN_DONE / _TIMEOUT).
void notify_push_cs_ir_learn(uint8_t state, uint8_t protocol, uint32_t code);

// Aux output change (NOTIFY_EVT_CS_AUX).  Safe from any dispatch context.
void notify_push_cs_aux(uint8_t slot, uint8_t state, uint16_t level_q8, ParamSource src);

// Analyser config applied (NOTIFY_EVT_RTA_CONFIG).  cfg is the 12-byte
// RtaConfig as applied.  Safe from any dispatch context.
#define NOTIFY_RTA_CONFIG_LEN        12
void notify_push_rta_config(const void *cfg, ParamSource src);

// ---------------------------------------------------------------------------
// Consumers
//
// The ring supports multiple independent consumers, each with its own tail:
// USB (EP 0x83 drain in usb_audio.c) and the UART control transport's
// notification frames.  One producer path feeds all of them.  A lagging
// consumer never delays the producer or the other consumers: when the
// producer catches a consumer's tail, that consumer's oldest entry is
// force-dropped (counted in notify_consumer_drops) and the push proceeds.
// Consumers detect their drops from the packet sequence-number gap and
// recover with a full REQ_GET_ALL_PARAMS read.
// ---------------------------------------------------------------------------

typedef enum {
    NOTIFY_CONSUMER_USB  = 0,   // always active
    NOTIFY_CONSUMER_UART = 1,   // active while the UART transport is live
                                // with notifications enabled
    NOTIFY_CONSUMER_COUNT
} NotifyConsumer;

// Activate / deactivate a consumer.  Activation snaps the consumer's tail
// to the current head (it sees events from now on, never a stale backlog).
// Inactive consumers are ignored by the producer's room-making logic.
void notify_consumer_set_active(NotifyConsumer c, bool active);

// peek: format consumer `c`'s next pending packet into out_buf (max max_len
// bytes).  Returns the packet length, or 0 when nothing is pending.  Skips
// entries not meant for this consumer (the v1 legacy master-volume packet
// is USB-only; every v1 event has a v2 twin).  Does NOT advance the tail;
// call notify_commit_pop_for() once the packet is safely handed off.
uint16_t notify_peek_next_for(NotifyConsumer c, uint8_t *out_buf, uint16_t max_len);

// Commit the last peek for consumer `c`, advancing its tail.  A no-op if
// the producer force-dropped past the peeked entry in the meantime.
void notify_commit_pop_for(NotifyConsumer c);

// True when consumer `c` is active and has at least one entry pending.
bool notify_has_pending_for(NotifyConsumer c);

// Reset the USB consumer's view (drop its pending backlog) and the global
// source/bulk brackets.  Called on USB bus reset.  Other consumers'
// backlogs and the global sequence counter are deliberately untouched.
void notify_reset_queue(void);

// Debug / diagnostics.  notify_consumer_drops counts force-dropped entries
// per consumer; the two legacy aggregates are kept for existing tooling.
extern volatile uint32_t notify_consumer_drops[NOTIFY_CONSUMER_COUNT];
extern volatile uint32_t notify_overflow_count;
extern volatile uint32_t notify_drops_count;

#endif // NOTIFY_H
