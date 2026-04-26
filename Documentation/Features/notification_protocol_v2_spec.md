# Notification Protocol v2 Specification

*Status: draft — pending review*
*Last updated: 2026-04-19*

## 1. Motivation

The v1 notification protocol (see `interrupt_endpoint_spec.md`) defines a per-parameter event catalog: each parameter that needs to notify the host gets its own event ID, pending flag, and dispatch branch. Only `NOTIFY_EVT_MASTER_VOLUME` is wired up today. Scaling v1 to the rest of the DSP state — especially with upcoming GPIO-driven parameter control — would require hand-classifying every parameter in three places (firmware emit site, firmware event ID enum, host dispatch switch).

**Goal:** a notification protocol where adding a parameter requires zero wire-format or host-dispatch changes beyond the single bulk-parameters schema the host already consumes.

**Non-goals:**
- Replacing v1 entirely. v1 stays for back-compat until every host app is updated.
- Notifying every sample of audio state (meters, levels). This protocol is for **parameter** changes only.
- Replacing `REQ_GET_ALL_PARAMS` / bulk SET. Bulk transfer is still the right tool for initial sync and wholesale restore.

## 2. Design Principles

1. **`WireBulkParams` is the single source of truth for "what is a parameter."** Every field in `WireBulkParams` is, by definition, a notifiable parameter. Its `offsetof` + `sizeof` is its stable identity. Adding a field to `WireBulkParams` automatically makes it notifiable.
2. **One generic event type covers all parameter changes.** `NOTIFY_EVT_PARAM_CHANGED` carries `(wire_offset, wire_size, source, value_bytes)`. Host dispatch is a flat lookup on offset, not a hand-written switch.
3. **Write-site notification, not poll-based diffing.** Every setter calls one helper (`param_write`) that updates both the live state and a shadow copy of `WireBulkParams`, then enqueues a notification only if bytes actually changed. No periodic diff scanner; no jitter.
4. **Source tagging lets the host suppress its own echoes.** The emit path stamps each notification with its origin (host SET, preset load, GPIO, factory reset, etc.). Suppression is a host-side policy decision, not a firmware config.
5. **Coalescing keys on `(offset, size)`.** A sweeping knob produces one queued notification per target parameter, not hundreds.
6. **Bulk invalidation is a separate event.** Operations that touch dozens of parameters at once (preset load, factory reset, bulk SET) emit a single `NOTIFY_EVT_BULK_INVALIDATED` and the host does one `REQ_GET_ALL_PARAMS`.

## 3. Wire Format v2

### 3.1 Endpoint Changes

The current EP 0x83 descriptor uses `NOTIFY_EP_MAX_PKT = 8`. That is too small for addressed notifications (header + offset + size + value exceeds 8 for any non-trivial field). **We bump it to 64 bytes** in the descriptor.

64 is the hard ceiling: USB 2.0 caps full-speed bulk `wMaxPacketSize` at 8/16/32/64, and DSPi is a full-speed device. Going larger would require fragmenting across multiple transactions on the wire, breaking the "one ring pop = one xfer" invariant and adding host-side reassembly. 64 fits every current parameter (largest: `WireChannelNames[i]` at 32 B payload + 12 B header = 44 B); anything bigger falls back to `BULK_INVALIDATED`.

Constraints we preserve:
- **Keep the EP as BULK IN**, not interrupt — the existing comment in `usb_descriptors.h` documents a real RP2xxx DCD crash when an interrupt IN endpoint polls continuously alongside rapid EP0 SETUPs. Do not switch back to interrupt.
- **Keep the "always-armed" invariant.** The endpoint must never NAK (the same DCD bug is the reason). When the ring is empty, arm with a 1-byte `NOTIFY_EVT_IDLE` packet, same as today. The host sees short packets when idle, full packets when events arrive.

This is a descriptor change. Implications:
- Bump `bcdDevice` so Windows re-reads the descriptor instead of using its cached version.
- Existing v1 hosts reading 8-byte transfers still work (bulk IN allows short reads).

Proposed: `NOTIFY_EP_MAX_PKT = 64`.

### 3.2 Packet Layout

Every v2 packet shares this 4-byte header:

```
Offset  Size  Field       Description
------  ----  ----------  ---------------------------------------------
0       1     version     2 for v2. 1 packets still recognized (back-compat).
1       1     event_id    See §3.3.
2       1     flags       Reserved. Must be 0 in v2. Host ignores unknown bits.
3       1     seq         Monotonic 8-bit counter, wraps at 256. Gap = loss.
```

Bytes 4..N are event-specific. Host MUST size its read by `actual_length`, not by header.

### 3.3 Event IDs

| ID   | Name                          | Class        | Description |
|------|-------------------------------|--------------|-------------|
| 0x00 | `NOTIFY_EVT_IDLE`             | Idle         | Keep-alive. Host discards. Exists only to keep EP armed. |
| 0x01 | `NOTIFY_EVT_MASTER_VOLUME`    | v1 legacy    | Payload: `float db` (4 bytes). Emitted **in addition to** v2 for hosts that only speak v1. |
| 0x02 | `NOTIFY_EVT_PARAM_CHANGED`    | Coalesceable | See §3.4. The primary v2 event. |
| 0x03 | `NOTIFY_EVT_BULK_INVALIDATED` | Discrete     | See §3.5. Host should `REQ_GET_ALL_PARAMS`. |
| 0x04 | `NOTIFY_EVT_PRESET_LOADED`    | Discrete     | Payload: `uint8_t slot` (byte 4), followed by `NOTIFY_EVT_BULK_INVALIDATED`. |
| 0x05 | `NOTIFY_EVT_ERROR`            | Discrete     | Reserved for future use (ring overflow report, etc.). |
| 0x06..0x7F | reserved (coalesceable) |              | |
| 0x80..0xFF | reserved (discrete)     |              | |

### 3.4 PARAM_CHANGED Layout

```
Offset  Size      Field         Description
------  --------  ------------  -----------------------------------------------
0       1         version       2
1       1         event_id      0x02
2       1         flags         0 in v2
3       1         seq           monotonic
4       2         wire_offset   offsetof into WireBulkParams (little-endian)
6       2         wire_size     sizeof the field (little-endian); see §3.6
8       1         source        ParamSource enum (see §3.7)
9       3         reserved      zero
12      N         value         wire_size bytes, same encoding as bulk
```

Total packet length = `12 + wire_size`. Max `wire_size` supported in a single packet = 52 bytes (fits `WireBulkParams.channel_names[i]` at 32 B, `WireCrosspoint` at 8 B, `WireBandParams` at 16 B, etc.). Fields larger than 52 B are not notified individually — they fall back to `BULK_INVALIDATED`.

### 3.5 BULK_INVALIDATED Layout

```
Offset  Size  Field        Description
------  ----  -----------  ----------------------------------------------
0       1     version      2
1       1     event_id     0x03
2       1     flags        0
3       1     seq          monotonic
4       1     source       ParamSource enum
5       3     reserved     zero
```

Total = 8 bytes. Emitted when the host should do a full re-sync rather than apply per-field deltas.

**When to emit:**
- Preset load (always — plus `NOTIFY_EVT_PRESET_LOADED` first so the host can show a toast)
- Factory reset
- Bulk SET via `REQ_SET_ALL_PARAMS`
- Firmware-side mass edits (e.g., auto-tune completes, future batch operations)

### 3.6 Addressing Rule

`wire_offset` is strictly `offsetof(WireBulkParams, field)` for scalar fields, or `offsetof(WireBulkParams, array) + index * sizeof(element)` for array elements.

Examples:
- `WireMasterVolume.master_volume_db` → `offsetof(WireBulkParams, master_volume.master_volume_db)`, size 4
- `WireBandParams` for channel 3 band 7 → `offsetof(WireBulkParams, eq) + (3*12+7)*sizeof(WireBandParams)`, size 16
- `WireCrosspoint` at input 0 output 5 → `offsetof(WireBulkParams, crosspoints) + (0*9+5)*sizeof(WireCrosspoint)`, size 8

**The host does not need a table.** It already has the `WireBulkParams` struct definition; it interprets the payload by casting at the offset.

### 3.7 Source Enum

```c
typedef enum : uint8_t {
    PARAM_SRC_UNKNOWN    = 0,
    PARAM_SRC_HOST_SET   = 1,  // REQ_SET_* via EP0
    PARAM_SRC_BULK_SET   = 2,  // REQ_SET_ALL_PARAMS
    PARAM_SRC_PRESET     = 3,  // Preset load
    PARAM_SRC_FACTORY    = 4,  // Factory reset
    PARAM_SRC_GPIO       = 5,  // Hardware control (knobs, encoders, pads)
    PARAM_SRC_INTERNAL   = 6,  // Firmware-initiated (clamp, auto-recalc)
} ParamSource;
```

The source is set by a scoped bracket in the caller (analogous to today's `notify_master_vol_host_initiated`). See §4.4.

## 4. Firmware Architecture

### 4.1 The `param_write()` Helper

One helper replaces all ad-hoc notify emit calls:

```c
// Write `size` bytes from `src` into the live shadow at `wire_offset`.
// If bytes differ from the current shadow, enqueue a PARAM_CHANGED
// notification with the given source tag.
//
// MUST be called AFTER the live DSP state has been updated — param_write
// only handles the shadow + notification, not the DSP side effects.
void param_write(uint16_t wire_offset,
                 uint16_t size,
                 const void *src,
                 ParamSource source);
```

Example integration in `update_master_volume`:

```c
void update_master_volume(float db) {
    /* existing clamp + live-state updates unchanged */
    master_volume_db = db;
    /* ... */

    param_write(offsetof(WireBulkParams, master_volume.master_volume_db),
                sizeof(float),
                &db,
                current_param_source);
}
```

`current_param_source` is a file-scope `volatile ParamSource` with a default of `PARAM_SRC_UNKNOWN`, overridden inside scoped brackets (§4.4).

### 4.2 Shadow Buffer

A single file-scope `WireBulkParams param_shadow` kept in sync with live state. Cost: `sizeof(WireBulkParams)` = 2912 B of BSS.

**Initialisation:** at boot, after `apply_factory_defaults()` and any preset load, call `bulk_params_collect(&param_shadow)`. This means first-boot and every preset transition starts with shadow == live; subsequent `param_write` calls compare against a truthful baseline.

**Re-sync hooks:** any code path that changes live state without going through a setter (unlikely, but possible) must call `bulk_params_collect(&param_shadow)` afterwards — or, preferably, route through a setter.

**Placement:** `__not_in_flash("param_shadow")` so reads from the audio callback never stall on XIP.

### 4.3 Notification Ring

Replace the single-slot `notify_master_vol_pending` with a fixed-size ring:

```c
#define NOTIFY_RING_SIZE  32    // power of two

typedef struct {
    uint8_t  event_id;
    uint8_t  source;
    uint16_t wire_offset;
    uint16_t wire_size;
    uint8_t  value[52];   // max PARAM_CHANGED payload
} NotifyRingEntry;

static NotifyRingEntry notify_ring[NOTIFY_RING_SIZE];
static volatile uint8_t notify_head, notify_tail;
static volatile uint32_t notify_overflow_count;
```

**Push rule (called from `param_write` and other emit sites):**
1. Disable interrupts (short critical section, single-producer assumption is safer if core 1 can also emit).
2. For coalesceable events (PARAM_CHANGED): scan unsent entries in the ring for a match on `(event_id, wire_offset, wire_size)`. If found, overwrite `value` in place and return.
3. Otherwise, append at `head`. If the ring is full, increment `notify_overflow_count` and drop the event. Exception: `BULK_INVALIDATED` displaces the oldest unsent entry when the ring is full (it is always deliverable).

**Drain rule (called from `usb_notify_tick` and `xfer_cb`):**
1. If no entry is pending, arm with the 1-byte `NOTIFY_EVT_IDLE` packet (preserves the always-armed invariant — §3.1).
2. Otherwise, claim EP via `usbd_edpt_claim`.
3. Pop entry from `tail`, format into `notify_buf`, stamp `seq = ++notify_seq`, fire `usbd_edpt_xfer`.
4. On `xfer_cb`, immediately re-arm (idle or next entry).

**Claim failure / xfer rejection** re-inserts the entry at `tail` (push-front) so ordering is preserved.

### 4.4 Source Tag Propagation

A scoped bracket sets `current_param_source` for the duration of an operation. Pattern already exists for `notify_master_vol_host_initiated`; generalise it:

```c
// In vendor_commands.c around a SET dispatch:
current_param_source = PARAM_SRC_HOST_SET;
__dmb();
dispatch_vendor_set(request);
__dmb();
current_param_source = PARAM_SRC_UNKNOWN;
```

Equivalent brackets wrap preset load (`PARAM_SRC_PRESET`), factory reset (`PARAM_SRC_FACTORY`), bulk SET (`PARAM_SRC_BULK_SET`), and GPIO handlers (`PARAM_SRC_GPIO`). Because v1 `update_master_volume()` runs inside these brackets, the source is attached at emit time without per-setter plumbing.

**Concurrency:** `current_param_source` is file-scope and not thread-safe. Core 1 must not issue parameter writes concurrently with Core 0 — today it doesn't, and we preserve that invariant. If that changes, promote the source to a per-core variable.

### 4.5 Bulk Operations

**Preset load** (`preset_apply()`):
1. Before: push `NOTIFY_EVT_PRESET_LOADED` with the slot index.
2. During the load: `current_param_source = PARAM_SRC_PRESET`. Setters still call `param_write`, but we **skip** the ring push when a bulk operation is in progress (see below).
3. After: call `bulk_params_collect(&param_shadow)` to re-baseline, then push one `NOTIFY_EVT_BULK_INVALIDATED`.

Rationale: emitting 100+ PARAM_CHANGED events for every preset load floods the ring and the USB bandwidth. One `BULK_INVALIDATED` + host-side bulk GET is strictly better.

Implementation: a `notify_suppress_count` that emit paths check. `preset_apply`, `apply_factory_defaults`, `bulk_params_apply` increment at entry, decrement at exit, and emit `BULK_INVALIDATED` on the last exit.

**Host bulk SET** (`REQ_SET_ALL_PARAMS`): same pattern with `PARAM_SRC_BULK_SET`.

**Factory reset**: same pattern with `PARAM_SRC_FACTORY`.

### 4.6 Flash Blackout

Unchanged. The ring absorbs events pushed during the blackout; the main loop drains them after. `notify_overflow_count` may tick up if a preset-load + reset sequence lands entirely inside the blackout; `BULK_INVALIDATED`'s displacement rule guarantees at least one is delivered.

## 5. Host Migration

### 5.1 Version Negotiation

The first byte of every packet is the protocol version.

- `version == 1`: parse as v1 (`NOTIFY_EVT_MASTER_VOLUME` only).
- `version == 2`: parse as v2.
- Anything else: discard.

The firmware emits both during the transition period: `update_master_volume` pushes a v1 master-volume packet **and** a v2 PARAM_CHANGED for the same field. This lets an unmodified v1 host keep working unchanged while new hosts see everything via v2. After one release cycle, the v1 emit can be removed.

### 5.2 Host Dispatch Table

```c
typedef void (*ParamHandler)(const void *value, uint16_t size, uint8_t source);

static const struct {
    uint16_t offset;
    uint16_t size;
    ParamHandler handler;
} PARAM_TABLE[] = {
    { offsetof(WireBulkParams, master_volume.master_volume_db), 4, on_master_volume },
    { offsetof(WireBulkParams, global.preamp_gain_db),          4, on_preamp_gain   },
    /* ... one row per parameter the host cares about ... */
};
```

Unknown offsets are ignored (forward compatibility). The host can also not register a handler at all and just call a generic UI sync function that reads the offset into its own copy of `WireBulkParams`.

### 5.3 Echo Suppression

Per-handler policy based on `source`:

```c
static void on_master_volume(const void *v, uint16_t n, uint8_t source) {
    if (source == PARAM_SRC_HOST_SET) return;   // host already updated its UI
    float db; memcpy(&db, v, 4);
    ui_update_master_volume(db);
}
```

For GPIO-driven changes, `source == PARAM_SRC_GPIO` — the host always wants these and updates the slider.

## 6. GPIO Integration (Illustrative)

A GPIO poll loop calling the existing setters needs **zero** notification-specific code:

```c
// In a GPIO handler running on Core 0:
void on_knob_tick(int knob_idx, int32_t delta) {
    if (knob_idx == KNOB_MASTER_VOL) {
        current_param_source = PARAM_SRC_GPIO;
        float db = master_volume_db + 0.5f * delta;
        update_master_volume(db);  // already calls param_write internally
        current_param_source = PARAM_SRC_UNKNOWN;
    }
}
```

`update_master_volume` already calls `param_write`; the host sees a PARAM_CHANGED event with `source = PARAM_SRC_GPIO` and updates its slider.

## 7. BSS / Flash Impact

| Component                       | Size      | Notes |
|---------------------------------|-----------|-------|
| `param_shadow`                  | 2912 B    | Baseline for change detection |
| `notify_ring` (32 × 60 B)       | 1920 B    | 52-byte payload + 8 B metadata |
| `notify_buf` (64 B)             | +56 B     | Grows from 8 B |
| Code delta (`param_write`, ring mgmt, drain rewrite) | ~1–1.5 KB | Estimated |
| **Total BSS**                   | **~4.9 KB** | Both platforms |

RP2040 BSS is currently ~125 KB; headroom is fine. RP2350 currently ~210 KB; same.

## 8. Testing Plan

### 8.1 Firmware Unit Tests

- `param_write` with identical bytes does not enqueue.
- `param_write` with differing bytes enqueues exactly one entry.
- Coalescing: two writes to the same offset before drain → one entry with the latest value.
- Ring overflow: 33 discrete events produce `notify_overflow_count == 1` and 32 delivered.
- `BULK_INVALIDATED` always delivered even when ring full.
- `notify_suppress_count` correctly gates individual emits during preset load.

### 8.2 Integration Tests

- Sweep master volume via EP0 → host sees v1 + v2 events; source = HOST_SET.
- Load a preset with 100+ changed fields → host sees one `PRESET_LOADED`, one `BULK_INVALIDATED`, no per-field PARAM_CHANGED.
- Simulate GPIO turning a knob → host sees PARAM_CHANGED with source = GPIO.
- Rapid knob sweep (1000 writes/sec) → ring never overflows, host sees latest value.
- Flash write during a knob sweep → no gap in sequence numbers after the blackout lifts (events coalesced and delivered late).

### 8.3 DCD Stress

- Rapid EP0 SETUP train (1 kHz) concurrent with PARAM_CHANGED emits at 100 Hz for 5 minutes.
- No DCD crash; no enumerate failure.
- This is the known RP2xxx failure mode that originally motivated bulk-vs-interrupt; regression surface here is real.

## 9. Rollout Steps

1. Write `param_shadow` + `param_write` + ring into `usb_audio.c` (or a new `notify.c` if clean separation is preferred).
2. Bump `NOTIFY_EP_MAX_PKT` to 64; bump `bcdDevice`.
3. Convert `update_master_volume` to emit both v1 and v2 (shim layer).
4. Instrument all other setters (`update_preamp`, crossfeed, matrix, EQ band set, delays, leveller, loudness, I2S config, pin config, channel names, preamp per-ch, input source).
5. Add scoped source brackets in `vendor_commands.c`, `flash_storage.c` (preset), `bulk_params.c` (bulk SET).
6. Add `notify_suppress_count` gate for bulk operations.
7. Host app: add v2 parser, PARAM_TABLE, echo-suppression policy. Remove v1 handler once stable.
8. Regression: run the full DCD stress matrix before merging.

## 10. Open Questions

1. ~~**Do we want a max packet size > 64?**~~ **Resolved: 64.** USB 2.0 caps full-speed bulk `wMaxPacketSize` at 8/16/32/64. 128 is not a legal value; 512 requires high-speed. Anything larger would require wire fragmentation, which breaks the one-xfer-per-event invariant. Every current parameter fits.
2. ~~**Do we want per-core source tagging?**~~ **Resolved: no.** Keep `current_param_source` as a single global. Core 1 does not call setters today; if that changes, promote to a per-core variable then.
3. ~~**Should `REQ_GET_NOTIFY_STATS` be added?**~~ **Resolved: no.** `notify_overflow_count` stays as a firmware-internal counter for debug builds; no vendor command.
4. ~~**Do v2 hosts receive `NOTIFY_EVT_IDLE`?**~~ **Resolved: idle is always `{0x00}`, one byte.** Treated as a protocol-neutral keep-alive by both v1 and v2 parsers. Version byte is not used in idle packets; the host distinguishes idle by length (1 byte) or by `event_id == 0x00`.
5. ~~**Do we want batched PARAM_CHANGED?**~~ **Resolved: defer.** Event ID 0x06 is reserved for a future `NOTIFY_EVT_PARAM_CHANGED_BATCH` that would pack multiple `(offset, size, value)` tuples into one 64 B packet. Implement only if measurement shows PARAM_CHANGED bursts are a bottleneck. The v2 base protocol does not depend on it.

---

## Appendix A: v1 → v2 Event Mapping

| v1 event                   | v2 equivalent |
|----------------------------|---------------|
| `NOTIFY_EVENT_IDLE`        | Same (0x00). Version-neutral keep-alive. |
| `NOTIFY_EVENT_MASTER_VOLUME` | `PARAM_CHANGED(offset=master_volume.master_volume_db, size=4)` |

During the transition, the firmware emits both. After one release, the v1 master-volume emit can be removed (the descriptor string still advertises bulk IN so v1 clients can at least see they're getting unknown `version` bytes and fall back to polling).

## Appendix B: Packet Examples

**Master volume → -12.0 dB, from GPIO:**
```
02 02 00 5C  D4 0A 04 00  05 00 00 00  00 00 40 C1
^vr ^ev ^fl ^seq
            ^offset=0x0AD4  ^size=4   ^src=GPIO  ^value=-12.0f LE
```
Total: 16 bytes.

**Preset 3 loaded:**
```
02 04 00 5D  03 00 00 00
02 03 00 5E  03 00 00 00
```
Two packets, 8 bytes each: `PRESET_LOADED(3)` then `BULK_INVALIDATED(source=PRESET)`.

**EQ band change (ch=2, band=5, type=PK, 1kHz, Q=1.0, +3 dB):**
```
02 02 00 5F
offset = offsetof(eq) + (2*12+5)*16  (little-endian uint16)
size   = 16
source = HOST_SET
value  = { type=2, reserved[3], freq=1000.0, q=1.0, gain=3.0 }
```
Total: 28 bytes.
