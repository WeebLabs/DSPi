# Control Surfaces: Target Groups and Macros

*Spec version 1; caps v9, directory V18. Companion to
`control_surfaces_spec.md`, which remains authoritative for everything a
single-target binding already did.*

Groups let one physical control (or IR command via a macro, or macro step)
address a named set of channels as a unit: mute a speaker group, trim gain or
delay on a stereo pair, light an LED when any (or every) member satisfies a
condition. Macros let a button-shaped event fire a short, optionally delayed
sequence of parameter changes: select an input and load a preset, switch
monitor A/B, run a timed mute sequence.

Both features ride the existing engine unchanged in kind: every resulting
parameter change still dispatches the same vendor command a host would send,
through `vendor_dispatch_set/get` with `CTRL_SOURCE_GPIO`. Nothing in this
feature adds a new path to the audio pipeline, so preset loads, input
switches and output-config changes triggered from a macro inherit the
existing deferred, pipeline-safe apply machinery. Output slot alignment is
untouched by construction.

## 1. Concepts

### 1.1 Groups

A group is `{target_kind, member_mask, name}` in one of 8 device-global
slots. `target_kind` uses the existing `CS_TARGET_*` channel spaces
(`INPUT_CH`, `OUTPUT_CH`, `DSP_CH`); bit N of `member_mask` = channel N of
that space. Groups are pure configuration: they are created and edited by
the host (Console), persisted with the rest of the CS config, and referenced
by bindings, and macro steps, through a flag.

A binding references a group by setting `CS_FLAG_GROUP` (0x20) in `flags`;
`target` then carries the group index instead of a channel index. The
binding's noun must be targeted, and the group's kind must be compatible
with the noun's target kind:

| Noun `target_kind`  | Accepted group kind |
|---------------------|---------------------|
| `CS_TARGET_INPUT_CH`  | `CS_TARGET_INPUT_CH`  |
| `CS_TARGET_OUTPUT_CH` | `CS_TARGET_OUTPUT_CH` |
| `CS_TARGET_DSP_CH`    | `CS_TARGET_DSP_CH`    |
| `CS_TARGET_DSP_BAND`  | `CS_TARGET_DSP_CH` (binding `index` = band, validated per member) |

IR commands do not carry the GROUP flag in this revision (`IrCommand.flags`
still accepts only WRAP and REPEAT); a remote button reaches groups by
firing a macro whose steps use them.

### 1.2 Link laws (continuous nouns)

Two laws govern how members of a group relate under a continuous control:

- **Offset-preserving (default).** Relative ops (encoder `STEP`, button
  `INC`/`DEC`) step every member from its own value, so a balance trim
  between members survives. Absolute ops from a pot (`ADJUST`) move the
  *group mean* to the pot position while maintaining each member's offset
  from the mean, captured at the start of the gesture.
- **Absolute-identical (`CS_FLAG_LINK_ABS`, 0x40).** `ADJUST` drives every
  member to the same value. Only valid on `ADJUST` (it is the only action
  where the two laws differ; `SET` and `MOMENTARY` are inherently
  absolute-identical, `STEP`/`INC`/`DEC` inherently relative).

Clamping is per member against the noun's range: spinning far enough
compresses the members against the rail and the offset is progressively
lost, exactly like ganged faders on a console. Within one gesture session
(see 4.2) winding back restores the offsets; a new session captures whatever
the values then are.

Bool and enum nouns under a group use the **anchor rule**: the
lowest-numbered member is read as the anchor, the action is computed against
it (`TOGGLE` inverts the anchor, `INC`/`DEC`/`STEP` step it), and every
member is driven to the anchor's new value. A half-muted group therefore
becomes coherent on the first press.

### 1.3 Grouped indicators

`IND_EQUALS` / `IND_ABOVE` on a group evaluate the condition per member and
combine with OR by default (lit when *any* member matches);
`CS_FLAG_GROUP_ALL` (0x80) selects AND (*all* members). TON/TOF
`on_delay`/`off_delay` filtering applies to the combined condition.
`IND_LEVEL` (PWM brightness) follows the **maximum** member value; GROUP_ALL
is invalid there.

### 1.4 Macros

A macro is `{name, step_count, steps[8]}` in one of 8 device-global slots. A
step is a stripped binding: `{noun, action, flags, target, index, value,
step, pre_delay}` with the action subset `SET` / `TOGGLE` / `INC` / `DEC` /
`TRIGGER`, flags `WRAP` / `GROUP`, and a `pre_delay` in 10 ms
units (0..65535 = up to ~10.9 min) that elapses *before* the step executes.
Steps may target groups; steps may not fire macros (noun `CS_NOUN_MACRO` is
rejected in a step; no nesting).

Macros are fired through a new noun, `CS_NOUN_MACRO` (52): an enum of
`max_macros` positions accepting `SET` (fire macro `value`) and `IND_EQUALS`
(LED lit while macro `value` is running). Any button gesture, or IR command
can therefore fire a macro with no structural change to `CsBinding` or
`IrCommand`. `REQ_CS_MACRO_FIRE` gives hosts and external MCUs the same
trigger. The live read of the noun is the running macro's index, or 255
when idle (matches no `IND_EQUALS` comparand).

**Sequencer semantics.** One macro runs at a time. Firing a macro while one
is running cancels the running one at its current step boundary (remaining
steps are dropped, the in-flight step is not rolled back) and starts the new
one; firing the *same* macro restarts it. An empty macro (`step_count` 0) is
a no-op. Steps execute in order on the 1 kHz CS tick; each step's dispatch
honours the BUSY-retry machinery, so a step landing during a flash write
waits rather than being lost. Steps are re-validated at fire time; a step
that no longer validates (e.g. its group was since emptied or re-kinded) is
skipped, not fatal.

## 2. Wire reference

All multi-byte fields little-endian, as everywhere in the vendor protocol.

### 2.1 `CsGroup` (40 bytes)

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 0 | 1 | `target_kind` | `CS_TARGET_INPUT_CH`/`OUTPUT_CH`/`DSP_CH`; 0 = slot empty |
| 1 | 3 | `reserved` | write 0 |
| 4 | 4 | `member_mask` | bit N = channel N; 32-bit (RP2350 DSP space is 17 channels) |
| 8 | 32 | `name` | NUL-terminated, same convention as slot names |

Empty slot = all-zero record (strict, like binding clears). A non-empty
group must have a valid kind, a non-zero mask, and no mask bits at or above
the platform's channel count for that kind.

### 2.2 `CsGroupConfig` (324 bytes, flash)

`{uint8 version (=1); uint8 reserved[3]; CsGroup groups[8]}`. All-zero =
no groups; a fresh directory needs no seeding.

### 2.3 `CsMacroStep` (12 bytes)

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 0 | 1 | `noun` | any noun accepting the step's action; not `CS_NOUN_MACRO` |
| 1 | 1 | `action` | `SET`/`TOGGLE`/`INC`/`DEC`/`TRIGGER` only |
| 2 | 1 | `flags` | `CS_FLAG_WRAP` \| `CS_FLAG_GROUP` |
| 3 | 1 | `target` | channel, or group index when GROUP set |
| 4 | 1 | `index` | band for `DSP_BAND` nouns, else 0 |
| 5 | 1 | `reserved` | write 0 |
| 6 | 2 | `value` | as `CsBinding.value` |
| 8 | 2 | `step` | as `CsBinding.step` (INC/DEC size; 0 = default) |
| 10 | 2 | `pre_delay` | 10 ms units before this step runs |

An all-zero record is an empty step (skipped). `LINK_ABS` is meaningless in
the step action subset and rejected.

### 2.4 `CsMacro` (132 bytes) and `CsMacroConfig` (1060 bytes, flash)

`CsMacro` = `{char name[32]; uint8 step_count; uint8 reserved[3];
CsMacroStep steps[8]}`. Execution uses `steps[0..step_count-1]`.
`CsMacroConfig` = `{uint8 version (=1); uint8 reserved[3]; CsMacro
macros[8]}`. All-zero = no macros.

### 2.5 `CsMacroHeaderWire` (36 bytes)

`{char name[32]; uint8 step_count; uint8 reserved[3]}`; the SET payload for
a macro's name and length (steps are SET individually; the 64-byte vendor
control buffer cannot carry a whole 132-byte macro).

### 2.6 `CsExtStatusPacket` (24 bytes)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `max_groups` (8) |
| 1 | 1 | `max_macros` (8) |
| 2 | 1 | `max_macro_steps` (8) |
| 3 | 1 | `macro_running` (index, 0xFF = idle) |
| 4 | 1 | `macro_step` (current step index while running) |
| 5 | 3 | `reserved` |
| 8 | 8 | `group_status[8]` (stored-record validity, `CS_STATUS_*`/0) |
| 16 | 8 | `macro_status[8]` (worst step validity, `CS_STATUS_*`/0) |

`CsStatusPacket` (41 bytes) and every pre-existing GET response are
**byte-identical to caps v8**; external clients doing exact-length readback
(the ESP32 front panel) are unaffected until they opt into the new
commands.

### 2.7 Caps v9

`CsCapsHeader` stays 40 bytes; the three former `reserved` bytes after
`max_ir_commands` become `max_groups`, `max_macros`, `max_macro_steps` (a
pre-v9 host read them as zeros). `caps_version` = 9. `noun_count` grows to
53 (`CS_NOUN_MACRO`).

## 3. Commands (`0x20`-`0x26`)

All follow the existing CS conventions: SETs are deferred single-deep
handoffs applied on the main loop with results in `cs_last_status` /
`cs_last_slot`; applies are live-only previews; `REQ_CS_SAVE` persists the
whole CS config (now including groups and macros) in one directory write and
`REQ_CS_REVERT` restores the stored one; the shared dirty flag covers all of
it. `cs_last_slot` tags: `0x40 | group` for group SETs, `0x60 | macro` for
macro header and step SETs (IR keeps `0x80 | sub`).

| Cmd | Name | Dir | wValue | Payload / response |
|-----|------|-----|--------|--------------------|
| 0x20 | `REQ_SET_CS_GROUP` | SET | group 0-7 | 40-byte `CsGroup`; all-zero clears |
| 0x21 | `REQ_GET_CS_GROUP` | GET | group 0-7 | 40-byte live `CsGroup` |
| 0x22 | `REQ_SET_CS_MACRO` | SET | macro 0-7 | 36-byte `CsMacroHeaderWire` |
| 0x23 | `REQ_GET_CS_MACRO` | GET | macro 0-7 | 132-byte live `CsMacro` |
| 0x24 | `REQ_SET_CS_MACRO_STEP` | SET | `(step << 8) \| macro` | 12-byte `CsMacroStep`; all-zero clears |
| 0x25 | `REQ_CS_MACRO_FIRE` | GET-style | macro 0-7; 0xFFFF = cancel | 1 status byte |
| 0x26 | `REQ_GET_CS_EXT_STATUS` | GET | 0 | 24-byte `CsExtStatusPacket` |

Recommended edit order for hosts: write steps first, header (with the final
`step_count`) last, so a concurrently-fired macro never sees a length that
exceeds its written steps. Fire-time revalidation makes a torn edit
skip-safe, not crash-prone, regardless.

New status codes: `CS_STATUS_INVALID_GROUP` 0x1F (bad/empty/mismatched
group reference), `CS_STATUS_INVALID_MACRO` 0x20 (bad macro index or
`step_count`), `CS_STATUS_INVALID_STEP` 0x21 (step record invalid).

## 4. Engine semantics

### 4.1 Group fan-out and BUSY retry

A grouped op computes every member's absolute target up front, then
dispatches member-by-member in the same tick. Members whose dispatch
reports BUSY stay in a per-slot pending mask and retry each tick. New
detents arriving while members are pending fold into the same op (the
accumulated delta updates every member's target and re-queues all members),
so no input is lost and no member double-steps. Grouped filter-noun ops
share the single EQ pending packet, so members trickle out one per EQ apply
cycle; this is milliseconds and invisible at human speed.

### 4.2 Gesture sessions (replacing per-member shadows)

Relative grouped ops and offset-preserving `ADJUST` run in a *session*: at
the first event after idle (or after `CS_GROUP_SESSION_TICKS` = 500 ms
without events) the engine captures each member's live value as its base
(and, for `ADJUST`, the group mean at capture), then accumulates one delta
for the whole session; targets are always `clamp(base_m + delta)` (log
units: `base_m * 2^delta`). This keeps rapid detents from coalescing
against stale live values of deferred-apply nouns (the job the single-target
shadow does), without per-member shadow state.

### 4.3 Momentary on a group

`MOMENTARY` on a grouped bool captures every member's value at the press,
drives all members to `value`, and restores each member on release; the
restore also runs when the binding is torn down mid-hold (rebind, revert).
As part of this change the single-target teardown path gained the same
best-effort restore (previously only IR commands had it), so no momentary
of any kind can strand a parameter at its held value.

### 4.4 Group edits and dependents

Applying a group SET re-validates every active binding that references any
group. A binding that no longer validates (group emptied, kind changed,
band index invalid for a new member) is deactivated with its failure in
`slot_status`, exactly like a stored binding that fails at boot; it
reactivates automatically when a later group SET makes it valid again.
There is no in-use refusal. A running macro is not cancelled by a group
edit; its later steps simply revalidate at execution.

### 4.5 Macro execution

The sequencer owns one op-state and one group context. Per step: wait out
`pre_delay`, revalidate, resolve the action exactly as a button press on an
equivalent binding would (anchor rule, group fan-out included), then wait
for the step's dispatches to leave the BUSY-pending state before starting
the next step's delay. A queued preset load, factory reset or bulk apply
also counts as unfinished work until it lands after the pipeline fade, and
no step fires while one is queued, so later steps are never overwritten by
it. Cancellation (new fire, `REQ_CS_MACRO_FIRE` 0xFFFF,
or revert) drops remaining steps; anything already dispatched stands.

## 5. Persistence (directory V18)

`PresetDirectory` gains `cs_groups` (324 B) then `cs_macros` (1060 B)
appended after `cs_ir`, total directory ~2955 bytes of the 4096-byte
sector. `DIR_VERSION_CURRENT` = 18. The V17 layout is byte-identical to the
V18 prefix, so the V17 to V18 migration is a prefix copy with zeroed new
blobs (= no groups, no macros), following the V16 to V17 pattern, with a
frozen `PresetDirectory_v17` snapshot and static asserts pinning the
geometry. Sanitizers bound-check the blob versions, `target_kind` and
`step_count` on load, zeroing implausible entries; `member_mask` range and
step records are platform-dependent and validate at apply / fire time
instead.

Boot order: groups and macros load before bindings (bindings validate
against groups). Factory reset and preset load/save do not touch them;
like all CS config they are device-global.

## 6. Validation summary

Binding (`cs_validate` additions):
- `CS_FLAG_GROUP`: noun must be targeted; `target` < 8; group non-empty;
  kind compatible per the 1.1 table; for `DSP_BAND` nouns, `index` must be a
  valid band for **every** member; `TRIGGER` never grouped (no targeted
  trigger nouns exist).
- `CS_FLAG_LINK_ABS`: requires GROUP + `ADJUST` + continuous noun.
- `CS_FLAG_GROUP_ALL`: requires GROUP + `IND_EQUALS`/`IND_ABOVE`.
- Without GROUP, the three new bits must be 0 (strict, as ever).

Group SET: kind in {1,2,3} or all-zero clear; mask non-zero and inside the
platform channel count; reserved bytes 0.

Macro header SET: `step_count` <= 8; reserved 0. Step SET: action in the
step subset; noun accepts it (per the caps noun mask, button column);
noun != `CS_NOUN_MACRO`; flags subset WRAP|GROUP; target/index valid
(channel or group, same rules as bindings); value/step bounds per
`cs_validate_values`. Steps also revalidate at fire time and skip on
failure.

## 7. RAM and platform notes

Live tables (groups 324 B + macros 1060 B), per-slot group contexts
(pending mask, session base per DSP channel), and the macro sequencer add
roughly 3 KB BSS on RP2350 and 2.5 KB on RP2040 (fewer channels). Both
platforms get the full feature; the only platform difference is channel
counts, which flow from the existing noun descriptors.
