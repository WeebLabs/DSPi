# Control Surfaces (User-Wired Physical Controls and Indicators)

*Firmware capability format version: 9*
*Config (flash) version: 2; IR config version: 2*
*Directory version: 18*

This document is the complete, self-contained specification for the DSPi
Control Surfaces feature: user-wired push buttons, toggle switches,
potentiometers, quadrature rotary encoders, indicator LEDs, PWM-dimmed
LEDs, and an IR remote receiver on spare GPIOs, configured over vendor
commands `0x84`-`0x87`, `0x8B`/`0x8C` (per-slot names), `0x8D`-`0x8F`
(IR remote commands and learn), and `0x9D`/`0x9E` (save / revert). The caps v9
target-group and macro commands `0x20`-`0x26` are specified in the companion
document `control_surfaces_groups_macros_spec.md`. It is
written for a host-app developer (e.g. DSPi Console) or an LLM adding Control
Surfaces support to an app, or extending the firmware feature itself. No DSPi
source is required to implement a host client.

Format v2 supersedes the v1 (16-byte binding, 8 slots, 9 nouns) format
described by earlier revisions of this document; see section 11 for the
compatibility story.

Caps v3 adds two things on top of v2 (section 11.6):

- **The IR remote component** (`CS_TYPE_IR`): one binding slot holds a
  demodulating IR receiver on one GPIO, and up to 8 learned remote-button
  **commands** live in a separate sub-slot table, each dispatching through
  the same noun/action machinery as a physical button. Codes are learned
  from any remote by pressing its button at the device. Sections 2.7, 3.6,
  6.8, 7.5.
- **The Apply / Save / Revert preview model**: binding and IR command SETs
  now apply live only; `REQ_CS_SAVE` persists the whole live config in one
  flash write and `REQ_CS_REVERT` reloads the stored one. Section 3.5.

Caps v4 adds 14 nouns and one unit on top of v3, with no structure or
stored-config changes (section 11.5): the stereo upmixer (35-40, RP2350
only), psychoacoustic bass (41-46), per-output delay (47, using the new
`CS_UNIT_MS`), and a preset-reload trigger (48).

Caps v6 doubles the IR command table from 8 to 16 sub-slots (`CsIrConfig`
format v2, `CsStatusPacket` 41 bytes); hosts must size the list from
`max_ir_commands`, not assume 8.

Caps v7 appends the two remaining loudness parameters as nouns 49-50
(reference SPL and intensity), with no structure or stored-config changes.

Caps v8 adds indicator condition timing: `CsBinding` gains `on_delay` and
`off_delay` (uint16, 0.1 s units, carved from the former `reserved2[6]`;
struct still 24 bytes), legal only on LED types with `IND_EQUALS`/`IND_ABOVE`
(sections 2.2 and 6.5). It also appends noun 51, `INPUT_LEVEL_MAX` (read-only
dB, the loudest channel of the active input), for signal-presence sensing
such as amplifier trigger outputs. Pre-v8 configs carry zeros in the new
fields, which means no delay; nothing else changes.

Caps v9 adds target groups (8 named channel sets a binding addresses as a unit
via `CS_FLAG_GROUP`) and macros (8 sequences of up to 8 steps, fired through
the appended noun 52, `CS_NOUN_MACRO`), commands `0x20`-`0x26`, and directory
V18. `CsBinding`, `IrCommand` and `CsStatusPacket` are byte-identical to v8;
the caps header's three bytes after `max_ir_commands` become
`max_groups`/`max_macros`/`max_macro_steps`. The authority for all of it,
including every new struct and status code, is
`control_surfaces_groups_macros_spec.md`.

Caps v10 and v11 are the I2C display bundle and its line alignment; both are
specified in `control_surfaces_display_spec.md`. Neither changes `CsBinding`.

Caps v12 adds `CsBinding.base_bright`, a per-LED brightness ceiling for
`CS_TYPE_LED_PWM` carved from the former `reserved` byte at offset 9 (struct
still 24 bytes; sections 2.2 and 6.6). Pre-v12 configs carry 0 there, which
means full brightness, so nothing migrates and no directory version changes.

Writing style note: this doc avoids em-dashes per project convention.

---

## 1. Overview and design rationale

### 1.1 The Verb-Type-Noun-Parameter-GPIO user model

A **binding** attaches one physical component to one firmware parameter through
one operation, on one or two GPIOs. The user thinks in five parts:

| Concept | Meaning | Example |
|---------|---------|---------|
| **Verb** | What the user does (turn, press, flip, slide) | "turn" |
| **Type** | The component wired up (`CsType`) | rotary encoder |
| **Noun** | The firmware parameter driven or shown (`CsNoun`) | master volume |
| **Parameter** | The operation applied (`CsAction`) plus its event/value/step/range/target | +1 dB per detent |
| **GPIO** | The pin(s) the component occupies | GPIO 27/28 |

The **Verb is not carried on the wire**; it is derived from the **Type**. A
potentiometer can only "adjust", an encoder can only "step", a switch can only
"follow", and so on. Encoding the verb separately would let a host send an
impossible combination (a pot that "toggles"); deriving it from the type makes
those states unrepresentable. The wire therefore carries Type, Noun, Action,
flags, event, target/index, pins, and numeric operands; the host builds its
verb-oriented UI from the capability tables (section 4).

v2 adds four orthogonal concepts to this model:

- **Events** (buttons only): one physical button can carry several bindings,
  distinguished by gesture: short press, long press (>= 500 ms), double press
  (two taps within 350 ms). Section 6.1.
- **Targets**: nouns that address a channel (per-input preamp, per-output
  gain/mute/enable, per-channel clip/level) or a channel plus a filter band
  (per-filter frequency/gain/Q/type/bypass) carry the address in the binding's
  `target` / `index` bytes. Section 4.4.
- **Units**: continuous nouns declare a unit (dB, Hz, Q, percent, ms). Frequency
  and Q step and map logarithmically; dB, percent, and ms linearly. Section 2.1.
- **Acceleration and repeat**: encoders can accelerate on fast rotation;
  INC/DEC buttons can auto-repeat while held. Sections 6.3, 6.1.

### 1.2 Why apply goes through the shared vendor dispatcher

Every control action is applied by dispatching **the same vendor command a host
would send**, through `vendor_dispatch_set` / `vendor_dispatch_get` with source
`CTRL_SOURCE_GPIO`. Turning the master-volume encoder one detent calls exactly
the `REQ_SET_MASTER_VOLUME` handler that a USB or UART host would reach. This
buys three things for free:

- **Zero duplication.** No parameter-apply logic is re-implemented in the
  Control Surfaces engine. Validation, clamping, deferred pipeline-safe apply
  (e.g. preset load stops SPDIF RX and fences Core 1), and output-slot alignment
  all come from the existing handler. The engine only resolves the target value.
- **Free host notifications.** The dispatch is tagged `PARAM_SRC_GPIO` (value 5
  in the notify `ParamSource` enum), so any change a knob makes emits the normal
  `PARAM_CHANGED` notification to connected hosts through the existing notify
  protocol (USB EP `0x83`, UART type-`0x40` frames), letting a UI track a
  physical control in real time with no extra path.
- **Transport-neutral safety.** The engine runs from main-loop context and gets
  the same `CTRL_DISPATCH_BUSY` back-pressure a USB control SET in flight would
  cause, so it never races the USB stack.

Per-filter nouns use a read-modify-write: the engine copies the live filter
recipe, replaces the one field the binding drives, and dispatches the full
packet through `REQ_SET_EQ_PARAM`. The EQ handlers share a single deferred
apply slot, so the engine additionally treats "an EQ apply is still owed" as
BUSY and retries next tick rather than overwriting a pending write.

### 1.3 Why the config is device-global (not per-preset)

The binding table is persisted **device-global** in the preset directory (V9),
not inside any preset. Wiring is a **board-level** fact: which physical control
sits on which GPIO does not change when the listener switches EQ profiles.
Consequently the config:

- **Survives preset changes.** Loading preset 3 does not rewire the knobs.
- **Survives factory reset** (the audio state resets; the wiring does not),
  exactly like `dac_hw_mute` and the UART/I2C control-interface config.
- Is **not** part of `WireBulkParams`; the bulk wire-format version is unchanged
  by this feature.

An all-zero blob (every slot `CS_TYPE_NONE`) is the idle state; a fresh or
factory-reset device needs no special seeding.

The per-slot **names** (section 3.4) follow the same reasoning and live in the
same directory (V10), as slot metadata alongside the binding table rather than
inside it: a name describes the physical control ("Sub Level"), so it survives
binding edits and slot clears, and may be set before a binding exists.

The **IR command table** (section 2.8) lives in the same directory (V11),
beside the binding table. It is board-level for the same reason: which remote
buttons the device answers to does not change with the listening profile.

### 1.4 Why IR is one component slot with sub-slots

Binding slots are sized and validated around **GPIO occupancy**: one slot, one
physical component, one or two pins. All of a remote's buttons arrive on one
receiver pin, so modelling each button as a binding would burn a slot's worth
of bookkeeping per button for no extra hardware. Instead `CS_TYPE_IR` occupies
**one** binding slot (the receiver: pin plus idle sense) and its buttons are
**commands** in a separate table of `CS_MAX_IR_COMMANDS` (16) sub-slots. A
command is semantically a button-shaped binding (same nouns, same actions
`INC`/`DEC`/`TOGGLE`/`SET`/`TRIGGER`/`MOMENTARY`, same value/step rules) fired
by a learned code instead of a GPIO edge, and it dispatches through exactly
the same shared vendor-command path (section 1.2). The slot's name (section
3.4) then naturally names the component ("Living Room Remote").

One IR component may be live at a time (a second `CS_TYPE_IR` binding is
rejected with `CS_STATUS_IR_IN_USE`): one receiver, one capture engine, one
un-partitioned command table.

---

## 2. Wire reference (byte-by-byte)

All structures are `__attribute__((packed))`, little-endian, no padding. The
same `CsBinding` bytes appear on the wire (the `REQ_SET`/`GET_CS_BINDING`
payload) and in flash.

### 2.1 Units and fixed-point conventions

Every continuous noun declares a **unit** in its descriptor (`CsNounDesc.unit`).
The unit fixes both the wire encoding of `value`/`range_min`/`range_max` and
the stepping law:

| Unit | Value | Encoding of value/range | Stepping law | `step` field encoding | Default step | Pot/LED mapping |
|------|-------|-------------------------|--------------|------------------------|--------------|-----------------|
| `CS_UNIT_NONE` | 0 | plain integer (bool 0/1, enum 0..N-1) | one position | (unused) | 1 position | n/a |
| `CS_UNIT_DB` | 1 | signed 8.8 dB (1 dB = 256) | linear | 8.8 dB | 1 dB | linear |
| `CS_UNIT_HZ` | 2 | plain integer Hz | logarithmic | 8.8 octaves | 1/12 octave | logarithmic |
| `CS_UNIT_Q` | 3 | 8.8 Q (Q 0.707 = 181) | logarithmic | 8.8 octaves | 1/12 octave | logarithmic |
| `CS_UNIT_PERCENT` | 4 | 8.8 percent (1 % = 256) | linear | 8.8 percent | 1 % | linear |
| `CS_UNIT_MS` | 5 | 8.8 ms (1 ms = 256) | linear | 8.8 ms | 0.1 ms | linear |

`CS_UNIT_MS` is a caps-v4 addition (currently only `OUTPUT_DELAY`); its
default step is 0.1 ms rather than the one-unit default of the other linear
units, because whole-ms detents are too coarse for delay alignment.

Logarithmic stepping multiplies: one detent scales the value by
`2^(step_octaves)`, so a frequency knob moves in musically even ratios and a
Q knob sweeps 0.1 to 10 in even multiplicative steps. `step = 256` is one
octave per detent; `step = 0` selects the default 1/12 octave.

Pots and dispatch quantization: linear units quantize to **half a unit**
(half-dB, half-percent, half-ms); log units quantize to **1/24 octave** above
the noun's minimum. A pot only re-dispatches when the quantized value changes,
giving smooth, jitter-free knob behavior without flooding the dispatcher.

### 2.2 `CsBinding` (24 bytes)

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `type` | `CsType` (0-7); `0` = slot cleared |
| 1 | 1 | `noun` | `CsNoun` (0-51) |
| 2 | 1 | `action` | `CsAction` (0-11) |
| 3 | 1 | `flags` | `CS_FLAG_*` bitfield (see 2.2.1); unknown bits are rejected with `CS_STATUS_INVALID_VALUE` |
| 4 | 1 | `gpio[0]` | primary GPIO |
| 5 | 1 | `gpio[1]` | second GPIO (encoders); `0xFF` (`CS_GPIO_UNUSED`) otherwise |
| 6 | 1 | `event` | `CsEvent` (buttons: 0 = press, 1 = long, 2 = double); MUST be 0 for other types |
| 7 | 1 | `target` | channel address for targeted nouns (section 4.4); 0 otherwise |
| 8 | 1 | `index` | filter band for `CS_TARGET_DSP_BAND` nouns; 0 otherwise |
| 9 | 1 | `base_bright` | caps v12: brightness ceiling, percent 1-100; `0` = unset = full. `CS_TYPE_LED_PWM` only, rejected non-zero elsewhere |
| 10 | 2 | `value` (int16) | `SET`/`MOMENTARY` target, `IND_EQUALS`/`IND_ABOVE` comparand; unit-encoded (2.1) |
| 12 | 2 | `step` (int16) | `STEP`/`INC`/`DEC` size; `0` = the unit default (2.1) |
| 14 | 2 | `range_min` (int16) | pot / `IND_LEVEL` span low end; both range fields `0` = the noun's full range |
| 16 | 2 | `range_max` (int16) | pot / `IND_LEVEL` span high end |
| 18 | 2 | `on_delay` (uint16) | caps v8: raw condition must hold true this long before the LED turns on; 0.1 s units, 0 = immediate. LED types with `IND_EQUALS`/`IND_ABOVE` only; rejected non-zero elsewhere |
| 20 | 2 | `off_delay` (uint16) | caps v8: raw condition must hold false this long before the LED turns off; same rules |
| 22 | 2 | `reserved2[2]` | write 0 (rejected non-zero); earmarked for an LED-extras flags byte, since `flags` has no free bit |

#### 2.2.1 Flags

| Bit | Name | Applies to | Meaning |
|-----|------|-----------|---------|
| `0x01` | `CS_FLAG_INVERT` | inputs | active-high with pull-down (default is active-low with pull-up) |
| | | LEDs | drive low = lit (PWM LED: inverted duty) |
| `0x02` | `CS_FLAG_REVERSE` | pot/encoder | invert direction |
| `0x04` | `CS_FLAG_WRAP` | enum STEP/INC/DEC | wrap around the ends (INC + WRAP = a "cycle" button) |
| `0x08` | `CS_FLAG_ACCEL` | encoder only | fast rotation multiplies the step (section 6.3) |
| `0x10` | `CS_FLAG_REPEAT` | button INC/DEC, press event only | auto-repeat while held (section 6.1) |

`CS_FLAG_ACCEL` on a non-encoder and `CS_FLAG_REPEAT` on anything but a
button INC/DEC are rejected with `CS_STATUS_INVALID_VALUE`.

### 2.3 `CsFlashConfig` (388 bytes)

The device-global persisted blob. Not sent by any single command; it is what
the firmware stores and what `REQ_GET_ALL_PARAMS` does **not** contain.

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `version` | `CS_CONFIG_VERSION` (2) |
| 1 | 3 | `reserved[3]` | 0 |
| 4 | 384 | `bindings[16]` | sixteen 24-byte `CsBinding` records |

### 2.4 `CsTypeDesc` (4 bytes) and `CsCapsHeader` (40 bytes)

`CsTypeDesc`:

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 2 | `actions` (uint16) | `CS_ACT_BIT` mask this component can drive |
| 2 | 1 | `pin_count` | GPIOs consumed (1 or 2) |
| 3 | 1 | `pin_class` | `CS_PINCLASS_ANY` (0) or `CS_PINCLASS_ADC` (1) |

`CsCapsHeader` (returned by `REQ_GET_CS_CAPS` with `wValue = 0xFFFF`):

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `caps_version` | capability format version (12) |
| 1 | 1 | `max_bindings` | `CS_MAX_BINDINGS` (16) |
| 2 | 1 | `type_count` | `CS_TYPE_COUNT` (9); the type table has this many entries, indexed by `CsType` |
| 3 | 1 | `noun_count` | `CS_NOUN_COUNT` (57) |
| 4 | 32 | `types[8]` | eight `CsTypeDesc`, one per `CsType` including index 0 (`NONE`, all-zero) |
| 36 | 1 | `max_ir_commands` | `CS_MAX_IR_COMMANDS` (16) |
| 37 | 3 | `reserved[3]` | 0 |

Total `4 + 4*8 + 4 = 40` bytes. The v3 tail fields sit **after** the
variable-length type table; a host locates them at offset
`4 + 4*type_count`, so a future type-table growth does not move them
relative to the table end.

The `CS_TYPE_IR` type descriptor's action mask describes what its
**commands** may do (the button action set); the container binding itself
carries `noun = action = 0` (section 2.7).

### 2.5 `CsNounDesc` (12 bytes)

Returned by `REQ_GET_CS_CAPS` with `wValue = noun index` (0-51).

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `kind` | `CS_KIND_CONTINUOUS` (0), `CS_KIND_BOOL` (1), `CS_KIND_ENUM` (2) |
| 1 | 1 | `enum_count` | valid enum values `0..enum_count-1` (`CS_KIND_ENUM` only) |
| 2 | 2 | `actions` (uint16) | `CS_ACT_BIT` mask this noun accepts; **0 = noun unavailable on this platform** (e.g. `ADAT_ACTIVE` on RP2040) |
| 4 | 2 | `min_q` (int16) | continuous range low end, unit-encoded (2.1) |
| 6 | 2 | `max_q` (int16) | continuous range high end |
| 8 | 1 | `unit` | `CS_UNIT_*` (2.1) |
| 9 | 1 | `target_kind` | `CS_TARGET_*` (4.4) |
| 10 | 1 | `target_count` | valid `target` values `0..target_count-1`; 0 when untargeted |
| 11 | 1 | `dflags` | `CS_NDF_DEFERRED` (`0x01`): apply is deferred; the engine steps from a target shadow (7.2) |

### 2.6 `CsStatusPacket` (41 bytes)

Returned by `REQ_GET_CS_STATUS`.

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `last_status` | result of the most recent deferred CS SET of any kind: binding, name, IR command, save, revert (`PIN_CONFIG_*` / `CS_STATUS_*`) |
| 1 | 1 | `last_slot` | slot the last SET targeted; `0x80 \| n` = IR sub-slot n, `0xFF` = save/revert |
| 2 | 1 | `max_bindings` | `CS_MAX_BINDINGS` (16) |
| 3 | 1 | `dirty` | 1 = the live config differs from flash (an unsaved preview; section 3.5) |
| 4 | 2 | `active_mask` (uint16) | bit N = binding N is live |
| 6 | 16 | `slot_status[16]` | per-slot apply status; `PIN_CONFIG_SUCCESS` (0) when live, else the failure code |
| 22 | 2 | `ir_active_mask` (uint16) | bit N = IR command N is live (valid, learned, and the IR component is up) |
| 24 | 1 | `ir_learn_state` | `CS_IR_LEARN_*`: 0 idle, 1 armed (listening), 2 done (result available), 3 timeout |
| 25 | 16 | `ir_cmd_status[16]` | per-sub-slot apply status, same codes as `slot_status` |

### 2.7 `IrCommand` (16 bytes)

One learned remote-button command; identical on the wire (the
`REQ_SET`/`GET_CS_IR_CMD` payload) and in flash. Semantically a button-shaped
binding: the noun / action / target / value / step fields follow exactly the
`CsBinding` rules of the same names (sections 2.1, 2.2, 4), fired by the
learned `code` instead of a GPIO edge.

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `noun` | `CsNoun` (0-48) |
| 1 | 1 | `action` | `CsAction`; button subset only: `INC`, `DEC`, `TOGGLE`, `SET`, `TRIGGER`, `MOMENTARY` |
| 2 | 1 | `flags` | `CS_FLAG_WRAP` and `CS_FLAG_REPEAT` only (REPEAT: INC/DEC only); any other bit is `CS_STATUS_INVALID_VALUE` |
| 3 | 1 | `target` | channel address for targeted nouns (section 4.4); 0 otherwise |
| 4 | 1 | `index` | filter band for `CS_TARGET_DSP_BAND` nouns; 0 otherwise |
| 5 | 1 | `protocol` | `CS_IR_PROTO_*` (see below); `0` (`NONE`) = empty sub-slot |
| 6 | 2 | `value` (int16) | `SET`/`MOMENTARY` target; unit-encoded (2.1) |
| 8 | 2 | `step` (int16) | `INC`/`DEC` size; `0` = the unit default (2.1) |
| 10 | 2 | `reserved[2]` | write 0 (rejected non-zero) |
| 12 | 4 | `code` (uint32) | the learned code, little-endian; `0` = never learned (rejected for an occupied sub-slot; no decoder can produce 0) |

An **empty** sub-slot is protocol `CS_IR_PROTO_NONE` with every other byte 0
(a non-zero remainder is rejected, like the binding reserved fields). Clear a
sub-slot by sending 16 zero bytes.

Protocols and code encodings (a host treats `protocol`+`code` as an opaque
pair; the encodings are documented for display and diagnostics):

| Protocol | Value | Code encoding |
|----------|-------|---------------|
| `CS_IR_PROTO_NONE` | 0 | (empty sub-slot) |
| `CS_IR_PROTO_NEC` | 1 | The 32 data bits in transmission order, bit 0 first: byte 0 = address, byte 1 = ~address (or address high for extended NEC), byte 2 = command, byte 3 = ~command. NEC repeat frames drive hold-to-repeat (7.5). |
| `CS_IR_PROTO_RC5` | 2 | The 14-bit frame in bits 13..0 (S1 at bit 13, always 1; S2/RC5X field bit at 12; address bits 10..6; command bits 5..0), with the toggle bit (bit 11) forced to 0 so a learned button matches every press. |
| `CS_IR_PROTO_RC6` | 3 | `(mode + 1) << 16 \| control << 8 \| information` for RC6 mode 0 (mode+1 keeps address 0 / command 0 away from the 0 sentinel); the toggle bit is checked for shape and excluded. |
| `CS_IR_PROTO_HASH` | 4 | An opaque FNV-1a hash of the frame's timing signature. Stable per button per remote; matches any remote whose protocol is not decoded above (repeat works by re-transmission). |

Two occupied sub-slots may carry the **same** protocol+code: one remote
button then fires several commands at once (e.g. set a preset and switch the
input). This is deliberate.

### 2.8 `CsIrConfig` (260 bytes)

The device-global persisted IR command table (directory V11+; this format
v2 layout since directory V17), beside
`CsFlashConfig`. Not sent by any single command.

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `version` | `CS_IR_CONFIG_VERSION` (2) |
| 1 | 3 | `reserved[3]` | 0 |
| 4 | 256 | `cmds[16]` | sixteen 16-byte `IrCommand` records |

All-zero = every sub-slot empty = feature idle; a fresh directory needs no
seeding. Like the binding table it survives preset changes and factory reset
and is not part of `WireBulkParams`.

---

## 3. Command reference (`0x84`-`0x87`, `0x8B`-`0x8F`, `0x9D`-`0x9E`)

The caps v9 group and macro commands `0x20`-`0x26` follow the same conventions
but are documented in `control_surfaces_groups_macros_spec.md`; `REQ_CS_SAVE`
and `REQ_CS_REVERT` below cover their config too.

| Command | Code | Dir | wValue | wLength / payload | Response |
|---------|------|-----|--------|-------------------|----------|
| `REQ_SET_CS_BINDING` | `0x84` | OUT | slot (0-15) | 24-byte `CsBinding` | none (deferred; poll `0x87`) |
| `REQ_GET_CS_BINDING` | `0x85` | IN | slot (0-15) | - | 24-byte `CsBinding` (live) |
| `REQ_GET_CS_CAPS` | `0x86` | IN | `0xFFFF` = header+types; else noun index | - | 40-byte `CsCapsHeader` or 12-byte `CsNounDesc` |
| `REQ_GET_CS_STATUS` | `0x87` | IN | - | - | 41-byte `CsStatusPacket` |
| `REQ_SET_CS_NAME` | `0x8B` | OUT | slot (0-15) | 1-32 byte name | none (deferred; poll `0x87`) |
| `REQ_GET_CS_NAME` | `0x8C` | IN | slot (0-15) | - | 32-byte NUL-terminated name (live) |
| `REQ_SET_CS_IR_CMD` | `0x8D` | OUT | sub-slot (0-15) | 16-byte `IrCommand` | none (deferred; poll `0x87`) |
| `REQ_GET_CS_IR_CMD` | `0x8E` | IN | sub-slot (0-15) | - | 16-byte `IrCommand` (live) |
| `REQ_CS_IR_LEARN` | `0x8F` | IN | 1 = arm, 0 = cancel, 2 = read result | - | 1 ack byte (arm/cancel) or 8-byte result (3.6.1) |
| `REQ_CS_SAVE` | `0x9D` | IN | - | - | 1 ack byte (deferred; poll `0x87`) |
| `REQ_CS_REVERT` | `0x9E` | IN | - | - | 1 ack byte (deferred; poll `0x87`) |

(`0x88`-`0x8A` are reserved for another feature branch.)

### 3.1 Transport note (UART / I2C)

Every one of these commands works identically over USB, UART, and I2C (the
transport-neutral dispatcher). On the external transports, the direction bit
matters: `0x85`, `0x86`, `0x87`, `0x8C`, `0x8E`, `0x8F`, `0x9D`, `0x9E` are IN
(GET-type) frames; `0x84`, `0x8B`, and `0x8D` are OUT (SET-type) frames
carrying their payload. `0x8F`, `0x9D`, and `0x9E` are GET-type action
commands in the mold of `REQ_SIGGEN_CONTROL`: the "GET" carries the action in
`wValue` and returns an acknowledgement byte.

### 3.2 Deferred-apply model (`REQ_SET_CS_BINDING`, `REQ_SET_CS_IR_CMD`)

`REQ_SET_CS_BINDING` does **not** apply synchronously. The USB/transport handler
only validates the slot index and payload length and latches the 24-byte binding
into a pending buffer, then returns immediately (no response payload). The main
loop then:

1. Runs `control_surfaces_apply_binding` (releases the slot's old pins, validates
   the new binding, claims the new pins, seeds runtime state).
2. Records the result in `last_status` / `last_slot`.
3. **Marks the live config dirty on `PIN_CONFIG_SUCCESS`.** Nothing is written
   to flash; the apply is a live preview until `REQ_CS_SAVE` (section 3.5).

`REQ_SET_CS_IR_CMD` follows the identical model for a 16-byte `IrCommand`
into a sub-slot, reported in `last_slot` as `0x80 | sub-slot`. A rejected SET
of either kind leaves the previous binding/command intact and the dirty flag
untouched.

On accept, the handler immediately records `CS_STATUS_PENDING` (`0x16`) in
`last_status` / `last_slot`; the main-loop apply then overwrites it with the
real result. The host learns the outcome by polling `REQ_GET_CS_STATUS` until
`last_status != CS_STATUS_PENDING` for the slot named in `last_slot`. A short
poll (a few ms) is ample; the apply is GPIO/engine work only.

If the slot index in `wValue` is out of range (binding slot `>= 16`, IR
sub-slot `>= 16`), the handler records `CS_STATUS_INVALID_SLOT` immediately (no
main-loop round trip) and no apply is queued. A payload shorter than the
record size (24 or 16 bytes) records `CS_STATUS_INVALID_VALUE` immediately. A
SET arriving while a previous SET of the same kind is still queued for the
main-loop apply is dropped and records `CS_STATUS_BUSY` (`0x1B`); a host that
polls for the PENDING result before sending the next SET (as this section
prescribes) never sees it.

Because binding SETs, IR command SETs, name SETs, save, and revert all report
through the single `last_status` / `last_slot` channel, a host should let each
deferred operation resolve (PENDING clears) before issuing the next one of
**any** kind; otherwise the later operation's result overwrites the earlier
one's in the shared channel (the per-slot `slot_status[]` / `ir_cmd_status[]`
arrays still hold each slot's verdict).

### 3.3 Status codes

`0x00`-`0x05` reuse the shared `PIN_CONFIG_*` namespace (config.h); Control
Surfaces extends it from `0x10`.

| Code | Name | Meaning |
|------|------|---------|
| `0x00` | `PIN_CONFIG_SUCCESS` | applied (or slot cleared) |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | pin out of range, or an encoder's two pins are equal |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | pin already claimed by another peripheral or a non-shareable binding |
| `0x03` | `PIN_CONFIG_INVALID_OUTPUT` | (shared namespace; not produced by CS) |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | (shared namespace; not produced by CS) |
| `0x05` | `PIN_CONFIG_INVALID_PARAM` | (shared namespace; not produced by CS) |
| `0x10` | `CS_STATUS_INVALID_SLOT` | slot index >= 16 |
| `0x11` | `CS_STATUS_INVALID_TYPE` | `type` >= `CS_TYPE_COUNT` |
| `0x12` | `CS_STATUS_INVALID_NOUN` | `noun` >= `CS_NOUN_COUNT` |
| `0x13` | `CS_STATUS_INVALID_ACTION` | `action` invalid, or not allowed for this type+noun (including a platform-disabled noun, whose action mask is 0) |
| `0x14` | `CS_STATUS_INVALID_VALUE` | `value` / `step` / `range` out of bounds, unknown `flags` bits, misused ACCEL/REPEAT flag, non-zero reserved bytes, or INVERT mismatch on a shared button pin |
| `0x15` | `CS_STATUS_PIN_NOT_ADC` | a pot was assigned a non-ADC GPIO (not 26-28) |
| `0x16` | `CS_STATUS_PENDING` | SET accepted; the main-loop apply has not run yet (poll again) |
| `0x17` | `CS_STATUS_INVALID_TARGET` | `target`/`index` out of range for the noun (or non-zero on an untargeted noun) |
| `0x18` | `CS_STATUS_INVALID_EVENT` | bad `event` value, an event on a non-button, or MOMENTARY/REPEAT bound to a non-press event |
| `0x19` | `CS_STATUS_PWM_CONFLICT` | PWM LED collides with another PWM LED on the same PWM slice+channel |
| `0x1A` | `CS_STATUS_EVENT_IN_USE` | another button binding already has this GPIO+event pair |
| `0x1B` | `CS_STATUS_BUSY` | SET dropped; a previous SET of the same kind is still queued for apply (retry after polling); also a save/revert while one is already queued |
| `0x1C` | `CS_STATUS_FLASH_ERROR` | the directory persist failed (`REQ_CS_SAVE`) |
| `0x1D` | `CS_STATUS_IR_IN_USE` | another slot already holds the IR component (one receiver per device) |
| `0x1E` | `CS_STATUS_NO_IR` | learn was armed with no live `CS_TYPE_IR` binding |

### 3.4 Per-slot names (`REQ_SET_CS_NAME` / `REQ_GET_CS_NAME`)

Each of the 16 slots carries a device-persistent user label (`CS_NAME_LEN` =
32 bytes, NUL-terminated; same convention as preset and channel names), so an
app on any host, or an external MCU on UART/I2C, can show what a given control
is for ("Sub Level", "Mute All") without out-of-band knowledge.

Semantics:

- **Names are slot metadata, independent of the binding.** Setting or clearing
  a binding leaves the slot's name untouched, and a name may be set for a slot
  that has no binding yet. Clear a name explicitly by sending a payload of one
  NUL byte.
- **Persistence** is device-global in the preset directory (V10), next to the
  binding table; saved names survive preset changes and factory reset (an
  unsaved live name is preview state and is lost on reboot, section 3.5).
  All-zero = unnamed, so a fresh directory needs no seeding.
- `REQ_SET_CS_NAME` uses the **same deferred model** as the binding SET
  (section 3.2): the handler validates the slot, latches the name, records
  `CS_STATUS_PENDING` in `last_status`/`last_slot`, and the main loop applies
  it to the live name table and overwrites the status with
  `PIN_CONFIG_SUCCESS`. Binding and name SETs share the single status
  channel; poll `REQ_GET_CS_STATUS` until the PENDING result resolves before
  sending the next SET of either kind. A name SET arriving while a previous
  name SET is still queued records `CS_STATUS_BUSY`; an empty payload records
  `CS_STATUS_INVALID_VALUE`. Payloads longer than 31 characters are
  truncated.
- `REQ_GET_CS_NAME` is synchronous and reads the live name table; it always
  returns 32 bytes with guaranteed NUL termination. While `dirty` is set the
  returned name may be an unsaved preview.
- Names are **not** part of `WireBulkParams` and emit **no notification**;
  a host that cares about cross-host renames re-reads the 16 names on
  connect (16 GETs, one round trip each).
- Names are **inside the Apply/Save/Revert preview** (section 3.5), exactly
  like bindings and IR commands: a name SET is live-only and sets `dirty`,
  `REQ_CS_SAVE` persists the names with the rest of the config, and
  `REQ_CS_REVERT` restores the stored names.

### 3.5 Apply / Save / Revert (`REQ_CS_SAVE` / `REQ_CS_REVERT`)

Binding, IR command, and slot-name SETs are **live-only previews**: they
claim pins and start behaving immediately, but nothing is written to flash.
This lets a host wire up a panel or a remote, try it, and only then commit:

- **Apply** = `REQ_SET_CS_BINDING` / `REQ_SET_CS_IR_CMD` / `REQ_SET_CS_NAME`.
  Each successful apply sets the `dirty` flag in the status packet.
- **Save** = `REQ_CS_SAVE`: persists the **whole live config** (the 16
  bindings, the 8 IR commands, and the 16 slot names together) in one
  directory-sector flash write, then clears `dirty`. Deferred like a SET: the handler acknowledges
  with one status byte, records `CS_STATUS_PENDING` with `last_slot = 0xFF`,
  and the main loop overwrites it with `PIN_CONFIG_SUCCESS` or
  `CS_STATUS_FLASH_ERROR`.
- **Revert** = `REQ_CS_REVERT`: discards the preview by re-applying the
  stored config from the directory cache (releases and reclaims GPIOs, PWM
  slices, and the IR receiver exactly as at boot), then clears `dirty`. Same
  deferred acknowledgement shape (`last_slot = 0xFF`); no flash write.
- A save or revert issued while either is still queued records
  `CS_STATUS_BUSY` and does nothing.
- **A reboot is an implicit revert**: unsaved changes do not survive power
  loss. That is the preview semantics, not a defect.

Hosts should surface `dirty` ("unsaved changes") and offer Save / Revert
whenever it is set. Serialize SETs against save/revert per section 3.2.

### 3.6 IR commands and learning (`0x8D`-`0x8F`)

Prerequisite: a live `CS_TYPE_IR` binding (the receiver; sections 2.7, 6.8).
Commands may be SET before the component exists; they validate and store, and
activate the moment the component comes up (`ir_active_mask` tracks this).

`REQ_SET_CS_IR_CMD` / `REQ_GET_CS_IR_CMD` write and read one 16-byte
`IrCommand` sub-slot (deferred model, section 3.2). A command whose
`protocol` is `NONE` (all-zero record) clears the sub-slot.

#### 3.6.1 Learn flow (`REQ_CS_IR_LEARN`)

1. Host sends `GET 0x8F, wValue = 1` (arm). With no live IR component the
   request STALLs (USB) / returns ERROR (UART/I2C) and records
   `CS_STATUS_NO_IR`; otherwise it acknowledges with one byte and the
   firmware listens on the receiver for up to **10 seconds**.
2. The user presses the desired button on the remote. The first cleanly
   decoded press is captured (NEC repeat frames and undecodable noise are
   ignored); normal command dispatch is suppressed while armed, and a button
   held into the arm is released first, so teaching a code never actuates
   anything.
3. Completion is pushed on the notification stream as event `0x0A`
   (section 7.5.1) with the state (`2` = done, `3` = timeout), the protocol,
   and the code. Hosts without the notify stream (e.g. I2C) poll
   `GET 0x8F, wValue = 2`, which returns 8 bytes:
   `{state, protocol, 0, 0, code_le32}`; `ir_learn_state` in the status
   packet carries the same state.
4. The host writes the captured protocol+code into an `IrCommand` along with
   the chosen noun/action and sends it via `REQ_SET_CS_IR_CMD`, then
   `REQ_CS_SAVE` when the user commits.

`GET 0x8F, wValue = 0` cancels an armed learn (state returns to idle, no
notification). Re-arming replaces a previous result. Removing the IR
component while armed aborts the learn as a timeout (with the notification),
since it can never complete without a receiver.

---

## 4. Capability / validity model

A binding is valid iff its `action` bit is set in **both** the type's action mask
and the noun's action mask, its `event` obeys the button rules, its
`target`/`index` address an existing channel/band, its numeric operands are in
range for the noun's kind and unit, and its pins pass the conflict checks.
Validation order: type -> noun -> action-in-range -> flags/reserved ->
action-allowed-by-type-AND-noun -> event -> target/index -> value/step/range ->
pins -> PWM slice.

> **Hosts MUST build their UI from `REQ_GET_CS_CAPS` at runtime, not from
> hardcoded tables.** The firmware serves these exact tables, so a host that
> reads them can never disagree with the device about which combinations are
> legal, and new types/nouns/actions in a future firmware appear in the UI with
> no host change. The tables below are the current (version 2) values, shown so
> an integrator can reason about them; treat them as illustrative, not as a
> substitute for reading the caps at connect time.

### 4.1 Reading the caps responses

- `REQ_GET_CS_CAPS`, `wValue = 0xFFFF` -> 40-byte `CsCapsHeader`: version, limits,
  the 8-entry type table (indexed by `CsType`), and the v3 tail
  (`max_ir_commands`).
- `REQ_GET_CS_CAPS`, `wValue = 0..51` (a noun index) -> 12-byte `CsNounDesc` for
  that noun. An out-of-range noun STALLs (USB) / returns ERROR (UART/I2C).
- A noun whose `actions == 0` is unavailable on this platform (currently only
  `ADAT_ACTIVE` on RP2040); grey it out.

### 4.2 Type table (`CsType` -> allowed actions, pins, class)

| `CsType` | Value | Allowed actions | `actions` mask | Pins | Pin class |
|----------|-------|-----------------|----------------|------|-----------|
| `CS_TYPE_NONE` | 0 | (none; clears the slot) | `0x0000` | 0 | ANY |
| `CS_TYPE_BUTTON` | 1 | `INC`, `DEC`, `TOGGLE`, `SET`, `TRIGGER`, `MOMENTARY` | `0x02BC` | 1 | ANY |
| `CS_TYPE_SWITCH` | 2 | `FOLLOW` | `0x0040` | 1 | ANY |
| `CS_TYPE_POT` | 3 | `ADJUST` | `0x0001` | 1 | **ADC** |
| `CS_TYPE_ENCODER` | 4 | `STEP` | `0x0002` | 2 | ANY |
| `CS_TYPE_LED` | 5 | `IND_EQUALS`, `IND_ABOVE` | `0x0500` | 1 | ANY |
| `CS_TYPE_LED_PWM` | 6 | `IND_EQUALS`, `IND_ABOVE`, `IND_LEVEL` | `0x0D00` | 1 | ANY |
| `CS_TYPE_IR` | 7 | `INC`, `DEC`, `TOGGLE`, `SET`, `TRIGGER`, `MOMENTARY` (its **commands**; the container binding carries `noun = action = 0`) | `0x02BC` | 1 | ANY |

Action bit positions (`CS_ACT_BIT(a) = 1 << a`): `ADJUST`=0, `STEP`=1, `INC`=2,
`DEC`=3, `TOGGLE`=4, `SET`=5, `FOLLOW`=6, `TRIGGER`=7, `IND_EQUALS`=8,
`MOMENTARY`=9, `IND_ABOVE`=10, `IND_LEVEL`=11.

The `CS_TYPE_IR` container binding itself is validated differently from every
other type: `noun`, `action`, `event`, `target`, `index`, `value`, `step`,
and both range fields must all be 0 (`CS_STATUS_INVALID_VALUE` otherwise),
`CS_FLAG_INVERT` is the only legal flag, and a second live IR binding is
`CS_STATUS_IR_IN_USE`. Its action mask exists for the host UI and for
validating its **commands**: an `IrCommand` is valid iff its action bit is in
both this mask and the noun's mask, exactly like a button binding (the noun
table, targets, units, and value/step bounds of sections 4.3, 4.4, and 2.1
apply unchanged).

### 4.3 Noun table (`CsNoun` -> kind, unit, range, target, allowed actions)

Action-mask groups used below:

- **CONT-RW** `0x0C2F` = `ADJUST | STEP | INC | DEC | SET | IND_ABOVE | IND_LEVEL`
- **BOOL-RW** `0x0370` = `TOGGLE | SET | FOLLOW | MOMENTARY | IND_EQUALS`
- **ENUM-RW** `0x012E` = `STEP | INC | DEC | SET | IND_EQUALS`
- **BOOL-RO / ENUM-RO** `0x0100` = `IND_EQUALS`
- **CONT-RO** `0x0C00` = `IND_ABOVE | IND_LEVEL`

| `CsNoun` | Value | Kind | Unit | Range / enum_count | Target | Actions |
|----------|-------|------|------|--------------------|--------|---------|
| `USER_VOLUME` | 0 | CONT | DB | -60..0 dB | - | CONT-RW |
| `MASTER_VOLUME` | 1 | CONT | DB | -127..0 dB | - | CONT-RW |
| `USER_MUTE` | 2 | BOOL | - | - | - | BOOL-RW |
| `LOUDNESS` | 3 | BOOL | - | - | - | BOOL-RW |
| `CROSSFEED` | 4 | BOOL | - | - | - | BOOL-RW |
| `LEVELLER` | 5 | BOOL | - | - | - | BOOL-RW |
| `PRESET` | 6 | ENUM | - | 10 | - | ENUM-RW, deferred |
| `INPUT_SOURCE` | 7 | ENUM | - | 3 (USB/SPDIF/I2S) | - | ENUM-RW, deferred |
| `CLIP` | 8 | BOOL | - | - | - | `TRIGGER \| IND_EQUALS` (`0x0180`) |
| `EQ_BYPASS` | 9 | BOOL | - | 1 = bypassed | - | BOOL-RW |
| `LG_SYNC` | 10 | BOOL | - | - | - | BOOL-RW |
| `CROSSFEED_PRESET` | 11 | ENUM | - | 4 (default/chu-moy/meier/custom) | - | ENUM-RW |
| `CROSSFEED_ITD` | 12 | BOOL | - | - | - | BOOL-RW |
| `LEVELLER_AMOUNT` | 13 | CONT | PERCENT | 0..100 % | - | CONT-RW |
| `LEVELLER_SPEED` | 14 | ENUM | - | 3 (slow/medium/fast) | - | ENUM-RW |
| `LEVELLER_LOOKAHEAD` | 15 | BOOL | - | - | - | BOOL-RW |
| `PREAMP` | 16 | CONT | DB | -24..+24 dB | INPUT_CH | CONT-RW |
| `OUTPUT_GAIN` | 17 | CONT | DB | -60..+12 dB | OUTPUT_CH | CONT-RW |
| `OUTPUT_MUTE` | 18 | BOOL | - | - | OUTPUT_CH | BOOL-RW |
| `OUTPUT_ENABLE` | 19 | BOOL | - | - | OUTPUT_CH | BOOL-RW |
| `FILTER_FREQ` | 20 | CONT | HZ | 20..20000 Hz | DSP_BAND | CONT-RW, deferred |
| `FILTER_GAIN` | 21 | CONT | DB | -24..+24 dB | DSP_BAND (PEQ only) | CONT-RW, deferred |
| `FILTER_Q` | 22 | CONT | Q | 0.1..10 | DSP_BAND (PEQ only) | CONT-RW, deferred |
| `FILTER_TYPE` | 23 | ENUM | - | 11 (PEQ `FilterType` 0-10) | DSP_BAND (PEQ only) | ENUM-RW, deferred |
| `FILTER_BYPASS` | 24 | BOOL | - | 1 = bypassed | DSP_BAND | BOOL-RW, deferred |
| `SIGGEN` | 25 | BOOL | - | 1 = running | - | BOOL-RW |
| `DAC_MUTE_TEST` | 26 | BOOL | - | - | - | `TRIGGER` (`0x0080`) |
| `CLIP_CH` | 27 | BOOL | - | read-only | DSP_CH | BOOL-RO |
| `LEVEL` | 28 | CONT | DB | -60..0 dB, read-only | DSP_CH | CONT-RO |
| `SPDIF_LOCK` | 29 | BOOL | - | read-only | - | BOOL-RO |
| `SAMPLE_RATE` | 30 | ENUM | - | 3 (0=44.1k, 1=48k, 2=96k), read-only | - | ENUM-RO |
| `USB_STREAMING` | 31 | BOOL | - | read-only | - | BOOL-RO |
| `ADAT_ACTIVE` | 32 | BOOL | - | read-only; **RP2350 only** (mask 0 on RP2040) | - | BOOL-RO |
| `LG_PRESENT` | 33 | BOOL | - | read-only | - | BOOL-RO |
| `LG_MUTED` | 34 | BOOL | - | read-only | - | BOOL-RO |
| `UPMIX` | 35 | BOOL | - | **RP2350 only** (mask 0 on RP2040) | - | BOOL-RW |
| `UPMIX_CENTER_MODE` | 36 | ENUM | - | 3 (Sinner/Logician/Off); **RP2350 only** | - | ENUM-RW |
| `UPMIX_SURROUND_MODE` | 37 | ENUM | - | 3 (Off/Sinner/Logician); **RP2350 only** | - | ENUM-RW |
| `UPMIX_STRENGTH` | 38 | CONT | PERCENT | 0..100 %; **RP2350 only** | - | CONT-RW |
| `UPMIX_WIDTH` | 39 | CONT | PERCENT | 0..100 %; **RP2350 only** | - | CONT-RW |
| `UPMIX_PRESENCE` | 40 | CONT | DB | -12..+12 dB; **RP2350 only** | - | CONT-RW |
| `PSYBASS` | 41 | BOOL | - | - | - | BOOL-RW |
| `PSYBASS_CUTOFF` | 42 | CONT | HZ | 30..300 Hz | - | CONT-RW |
| `PSYBASS_HARMONICS` | 43 | CONT | DB | -24..+12 dB | - | CONT-RW |
| `PSYBASS_DRIVE` | 44 | CONT | DB | 0..+18 dB | - | CONT-RW |
| `PSYBASS_CHARACTER` | 45 | CONT | PERCENT | 0..100 % | - | CONT-RW |
| `PSYBASS_ORIGINAL` | 46 | CONT | DB | -60..0 dB | - | CONT-RW |
| `OUTPUT_DELAY` | 47 | CONT | MS | 0..21 ms (RP2040) / 0..42 ms (RP2350) | OUTPUT_CH | CONT-RW |
| `PRESET_RELOAD` | 48 | BOOL | - | - | - | `TRIGGER` (`0x0080`) |
| `LOUDNESS_SPL` | 49 | CONT | DB | 40..100 dB SPL | - | CONT-RW |
| `LOUDNESS_INTENSITY` | 50 | CONT | PERCENT | 0..127 % | - | CONT-RW |
| `INPUT_LEVEL_MAX` | 51 | CONT | DB | -60..0 dB | - | CONT-RO |

The *effective* legal action set for a (type, noun) pair is the bitwise AND of
its two masks. Example: an encoder (`STEP` only) on `USER_MUTE` (bool, no
`STEP`) has an empty intersection and is rejected with
`CS_STATUS_INVALID_ACTION`.

### 4.4 Targets

`CsNounDesc.target_kind` says what the binding's `target` byte addresses;
`target_count` gives the valid range (it reflects the platform: RP2350 has 8
input channels and 9 outputs, RP2040 has 2 and 5).

| `target_kind` | Value | `target` means | `index` means |
|---------------|-------|----------------|----------------|
| `CS_TARGET_NONE` | 0 | (must be 0) | (must be 0) |
| `CS_TARGET_INPUT_CH` | 1 | input channel 0..N-1 | (must be 0) |
| `CS_TARGET_OUTPUT_CH` | 2 | output channel 0..N-1 | (must be 0) |
| `CS_TARGET_DSP_CH` | 3 | DSP channel (inputs first, then outputs) | (must be 0) |
| `CS_TARGET_DSP_BAND` | 4 | DSP channel | filter band |

Valid bands for `CS_TARGET_DSP_BAND`: PEQ bands `0..channel_band_counts-1`
(currently 10 per channel), plus, for `FILTER_FREQ` and `FILTER_BYPASS` only,
the crossover bands `20..23` **on output channels only** (matching the EQ
handler's addressing; see crossover_filters_spec.md). `FILTER_GAIN`, `FILTER_Q`
and `FILTER_TYPE` are PEQ-only. Anything else is `CS_STATUS_INVALID_TARGET`.

---

## 5. Nouns reference

Each noun maps to an existing vendor command; the engine resolves an absolute
target and dispatches it.

| Noun | Underlying command | Value semantics |
|------|--------------------|-----------------|
| `USER_VOLUME` | `REQ_SET_USER_VOLUME` (`0xDA`, float dB) | The user/OS-slider volume, shared with UAC1; range -60..0 dB. |
| `MASTER_VOLUME` | `REQ_SET_MASTER_VOLUME` (`0xD2`, float dB) | Device output ceiling; range -127..0 dB. The `-128 dB` mute sentinel is **not reachable** from a pot or encoder; use a mute noun. |
| `USER_MUTE` | `REQ_SET_USER_MUTE` (`0xDC`, uint8 0/1) | User mute, shared with UAC1. |
| `LOUDNESS` | `REQ_SET_LOUDNESS` (`0x58`) | Loudness compensation enable. |
| `CROSSFEED` | `REQ_SET_CROSSFEED` (`0x5E`) | Crossfeed enable. |
| `LEVELLER` | `REQ_SET_LEVELLER_ENABLE` (`0xB4`) | Volume leveller enable. |
| `PRESET` | `REQ_PRESET_LOAD` (`0x91`, GET, wValue = slot) | Active preset 0-9, deferred pipeline-safe load. **Stepping moves across OCCUPIED slots only.** |
| `INPUT_SOURCE` | `REQ_SET_INPUT_SOURCE` (`0xE0`) | 0 = USB, 1 = SPDIF, 2 = I2S. |
| `CLIP` | `REQ_CLEAR_CLIPS` (`0x83`, GET) | Any-channel clip latch. LED (`IND_EQUALS`, value 1): lit while any clip bit is set. Button (`TRIGGER`): clears all clip flags. |
| `EQ_BYPASS` | `REQ_SET_BYPASS` (`0x46`) | Master EQ bypass; value 1 = EQ bypassed. |
| `LG_SYNC` | `REQ_SET_LG_SOUND_SYNC_ENABLE` (`0xE6`) | LG Sound Sync enable. |
| `CROSSFEED_PRESET` | `REQ_SET_CROSSFEED_PRESET` (`0x60`) | Crossfeed voicing 0-3; INC+WRAP makes a preset-cycle button. |
| `CROSSFEED_ITD` | `REQ_SET_CROSSFEED_ITD` (`0x66`) | Crossfeed inter-aural time delay enable. |
| `LEVELLER_AMOUNT` | `REQ_SET_LEVELLER_AMOUNT` (`0xB6`, float) | Leveller strength 0..100 %. |
| `LEVELLER_SPEED` | `REQ_SET_LEVELLER_SPEED` (`0xB8`) | 0 slow, 1 medium, 2 fast. |
| `LEVELLER_LOOKAHEAD` | `REQ_SET_LEVELLER_LOOKAHEAD` (`0xBC`) | Lookahead enable (toggling resets the leveller delay line; that is the handler's documented behavior, not a CS side effect). |
| `PREAMP` | `REQ_SET_PREAMP_CH` (`0xD0`, wValue = `target`, float dB) | Per-input-channel preamp, -24..+24 dB bindable span. |
| `OUTPUT_GAIN` | `REQ_SET_OUTPUT_GAIN` (`0x74`, wValue = `target`, float dB) | Per-output gain, -60..+12 dB bindable span. |
| `OUTPUT_MUTE` | `REQ_SET_OUTPUT_MUTE` (`0x76`, wValue = `target`) | Per-output soft mute. |
| `OUTPUT_ENABLE` | `REQ_SET_OUTPUT_ENABLE` (`0x72`, wValue = `target`) | Per-output enable. Subject to the PDM/EQ-worker Core 1 interlock; a blocked enable is silently skipped by the handler (the switch/LED will show the true state). |
| `FILTER_FREQ` | `REQ_SET_EQ_PARAM` (`0x42`, RMW) | Per-filter frequency, 20..20000 Hz, log stepping. Valid on PEQ bands and (outputs only) crossover bands 20-23; a sub-crossover frequency knob is `target = <sub output>`, `index = 20`. |
| `FILTER_GAIN` | `REQ_SET_EQ_PARAM` (RMW) | Per-filter gain, -24..+24 dB. PEQ bands only. |
| `FILTER_Q` | `REQ_SET_EQ_PARAM` (RMW) | Per-filter Q, 0.1..10, log stepping. PEQ bands only. |
| `FILTER_TYPE` | `REQ_SET_EQ_PARAM` (RMW) | PEQ `FilterType` 0-10 (flat, peaking, low-shelf, high-shelf, low-pass, high-pass, notch, all-pass, all-pass-1, low-shelf-1, high-shelf-1). PEQ bands only. |
| `FILTER_BYPASS` | `REQ_SET_BAND_BYPASS` (`0xD8`, wValue = `(target << 8) \| index`) | Per-band user bypass; value 1 = bypassed. PEQ and (outputs only) crossover bands. |
| `SIGGEN` | `REQ_SIGGEN_CONTROL` (`0xA6`, GET, wValue = start/stop) | Test signal generator run state (uses the applied `SiggenConfig`). 1 = running (any non-idle state). `MOMENTARY` gives hold-to-test. |
| `DAC_MUTE_TEST` | `REQ_TEST_DAC_HW_MUTE` (`0xEC`, GET) | `TRIGGER` pulses the DAC hardware mute for ~1 s so an installer can verify pin and polarity by ear. |
| `CLIP_CH` | (read-only) | Per-channel clip latch bit (`target` = DSP channel). Clearing is global via the `CLIP` noun. |
| `LEVEL` | (read-only) | Per-channel peak meter in dB (-60..0), `target` = DSP channel. Drives `IND_ABOVE` (signal-present LED) and `IND_LEVEL` (PWM meter LED). |
| `INPUT_LEVEL_MAX` | (read-only) | Loudest channel of the active input in dB (-60..0), untargeted; the max over `peaks[0..active_input_channel_count())`, gated on `pipeline_producer_is_streaming()` so a stopped stream reads -60 (the silence floor) instead of the last frozen peak. Signal presence for the whole source; with `IND_ABOVE` and `off_delay` it drives an amplifier trigger output (8.2g). |
| `SPDIF_LOCK` | (read-only) | 1 while the SPDIF receiver is locked. |
| `SAMPLE_RATE` | (read-only) | Enum for `IND_EQUALS`: LED lit while the pipeline rate equals 44.1/48/96 kHz. An unrecognised rate matches nothing. |
| `USB_STREAMING` | (read-only) | 1 while USB is the active input and the host stream is running. |
| `ADAT_ACTIVE` | (read-only, RP2350) | 1 while the ADAT output is streaming at a supported rate. |
| `LG_PRESENT` | (read-only) | 1 while an LG Sound Sync source is detected. |
| `LG_MUTED` | (read-only) | 1 while an LG source is present and reports muted. |
| `UPMIX` | `REQ_UPMIX_SET_PARAM` (`0x4C`, wValue = 0, float) | Stereo upmixer enable (RP2350 only, mask 0 on RP2040; likewise the five nouns below). |
| `UPMIX_CENTER_MODE` | `REQ_UPMIX_SET_PARAM` (wValue = 1) | Centre engine: 0 = Passive (fixed 0.7071 sum; displayed "Sinner"), 1 = Adaptive (correlation-steered extraction; displayed "Logician"), 2 = Off (no centre output, L/R untouched; surrounds keep working). INC+WRAP cycles. |
| `UPMIX_SURROUND_MODE` | `REQ_UPMIX_SET_PARAM` (wValue = 2) | Surround engine: 0 = Off, 1 = Passive (difference feed; displayed "Sinner"), 2 = Adaptive (Dolby low-complexity steering; displayed "Logician"). INC+WRAP cycles. |
| `UPMIX_STRENGTH` | `REQ_UPMIX_SET_PARAM` (wValue = 3) | Centre extraction strength 0..100 %. |
| `UPMIX_WIDTH` | `REQ_UPMIX_SET_PARAM` (wValue = 4) | Centre width 0..100 % (0 = full removal from L/R, 100 = phantom kept). |
| `UPMIX_PRESENCE` | `REQ_UPMIX_SET_PARAM` (wValue = 13) | Centre presence bell -12..+12 dB (both centre modes). |
| `PSYBASS` | `REQ_SET_PSYBASS` (`0x30`, uint8 0/1) | Psychoacoustic bass enable. |
| `PSYBASS_CUTOFF` | `REQ_SET_PSYBASS_CUTOFF` (`0x32`, float Hz) | Speaker LF limit 30..300 Hz, log stepping. |
| `PSYBASS_HARMONICS` | `REQ_SET_PSYBASS_HARMONICS` (`0x34`, float dB) | Generated-harmonics mix level -24..+12 dB. |
| `PSYBASS_DRIVE` | `REQ_SET_PSYBASS_DRIVE` (`0x36`, float dB) | Odd-path clipper drive 0..18 dB. |
| `PSYBASS_CHARACTER` | `REQ_SET_PSYBASS_CHARACTER` (`0x38`, float %) | Even<->odd harmonic blend 0..100 % (warm to aggressive). |
| `PSYBASS_ORIGINAL` | `REQ_SET_PSYBASS_ORIGINAL` (`0x3A`, float dB) | Original low-band level -60..0 dB. |
| `OUTPUT_DELAY` | `REQ_SET_OUTPUT_DELAY` (`0x78`, wValue = `target`, float ms) | Per-output delay; bindable span is the full delay ring at 48 kHz (21 ms RP2040 / 42 ms RP2350). At 96 kHz the pipeline clamps in samples, exactly as for a host-set delay; the ms value round-trips unclamped. |
| `PRESET_RELOAD` | `REQ_PRESET_LOAD` (`0x91`, GET, wValue = active slot) | `TRIGGER` reloads the currently active preset from flash via the deferred pipeline-safe path, discarding unsaved live edits. Device-global state (master volume in independent mode, output config, CS bindings) is untouched. |
| `LOUDNESS_SPL` | `REQ_SET_LOUDNESS_REF` (`0x5A`, float dB SPL) | Reference listening level 40..100 dB SPL: the level at which the ISO 226 compensation reads flat. Lower it and the curve engages sooner as volume drops. |
| `LOUDNESS_INTENSITY` | `REQ_SET_LOUDNESS_INTENSITY` (`0x5C`, float %) | Compensation depth, 100 % = the full ISO 226 contour difference. The vendor command accepts 0..200 %, but 8.8 percent caps the bindable span at 0..127 %. |

### 5.1 Enum stepping detail

For `STEP`/`INC`/`DEC` on enum nouns, `CS_FLAG_WRAP` makes the ends wrap; without
it the value clamps at 0 and `enum_count-1`. **`INC` + `WRAP` is the canonical
"cycle" button** (cycle inputs, cycle crossfeed presets, cycle presets). For
`PRESET`, stepping searches outward in the requested direction for the next
occupied slot (honoring wrap), so a "next preset" button skips empty slots.

### 5.2 Continuous stepping / adjust detail

- `INC`/`DEC`/`STEP` on a continuous noun move by `step` (unit default when 0),
  linearly for dB/percent and multiplicatively for Hz/Q, clamped to the noun
  range. Encoder acceleration multiplies the number of steps (6.3).
- `ADJUST` (pot) maps the knob position across the noun's full range, or across
  `[range_min, range_max]` when either field is non-zero (a custom span,
  e.g. a volume knob limited to -30..0 dB, or a sub-crossover knob limited to
  40..200 Hz). Log-unit nouns map exponentially so knob travel is perceptually
  even.

### 5.3 Deferred nouns and the target shadow

Nouns flagged `CS_NDF_DEFERRED` (`PRESET`, `INPUT_SOURCE`, all `FILTER_*`)
apply asynchronously in the firmware (a preset load takes tens of ms; EQ writes
land on the next main-loop pass). The engine steps such nouns from a **target
shadow**, the last dispatched target, until the live value confirms it (or a
500 ms timeout drops it), so spinning an encoder several detents advances
several steps rather than re-targeting from a stale read.

The `FILTER_*` nouns additionally treat "an EQ apply is still owed" as BUSY
(the EQ handlers share one deferred packet slot); the engine retries next tick,
so rapid detents on two different filter knobs cannot overwrite each other.

---

## 6. Electrical / wiring reference

### 6.1 Buttons (1 GPIO): gestures, sharing, repeat

- **Default wiring is active-low with the internal pull-up**: wire the button
  between the GPIO and GND. **`CS_FLAG_INVERT`** selects active-high with the
  internal pull-down.
- **Debounce**: a level must be stable for **10 ms** before it is accepted.
- **Events.** Each button binding names the gesture that fires it
  (`CsBinding.event`): `PRESS` (0), `LONG` (1, held >= 500 ms), `DOUBLE` (2,
  two taps with the second press within 350 ms of the first release).
- **Pin sharing.** Several button bindings may share one GPIO, one per event
  (the same GPIO+event pair twice is `CS_STATUS_EVENT_IN_USE`). All bindings
  on a shared pin must agree on `CS_FLAG_INVERT` (`CS_STATUS_INVALID_VALUE`
  otherwise). Sharing with any non-button binding is `PIN_CONFIG_PIN_IN_USE`.
- **Gesture disambiguation** (per pin, automatic from which events are bound):
  - Only `PRESS` bound: fires on the press edge (immediate).
  - `LONG` also bound: `PRESS` fires on release before the 500 ms threshold;
    `LONG` fires once at the threshold while still held.
  - `DOUBLE` also bound: a second press inside the window fires `DOUBLE`; a
    lone tap fires `PRESS` when the window expires (the unavoidable cost of
    double-press detection is that latency on single presses).
  - A long hold of the second tap of a double does not additionally fire `LONG`.
- **Hold-to-repeat** (`CS_FLAG_REPEAT`, `INC`/`DEC` on the `PRESS` event):
  after 400 ms held, the action repeats at 12.5 Hz. Runtime-suppressed on pins
  that also carry `LONG` or `DOUBLE` bindings (the hold belongs to the gesture
  decoder there).
- **`MOMENTARY`** (press event only): on press the noun is set to `value`; on
  release it is restored to the value captured at the press. Both directions
  are absolute dispatches (7.1). Engages on the press edge regardless of other
  gestures on the pin; combining it with `LONG`/`DOUBLE` bindings on the same
  pin is legal but rarely what you want.
- A button held at boot (or when its binding is applied) is a sync, not a
  press; nothing fires, including `LONG`, until it is released and pressed
  again.

### 6.2 Switches (1 GPIO)

- `FOLLOW` only: the bound bool tracks the switch position, **including at
  boot** (boot-sync): the first stable read is dispatched as an absolute value,
  so the firmware state matches the physical switch immediately.
- Wiring/debounce as buttons; no events.

### 6.3 Rotary encoders (2 GPIOs)

- **2-pin incremental quadrature.** `gpio[0]` = channel A, `gpio[1]` = channel B;
  the two pins must differ. Both pins use the internal pull-up (or pull-down
  under `CS_FLAG_INVERT`), so wire the common terminal to GND (or 3V3).
- **One detent = 4 quarter-steps** (a full A/B cycle), decoded with a standard
  16-entry transition table; invalid two-bit jumps decode as no movement.
- **`CS_FLAG_REVERSE`** flips the direction.
- **Acceleration** (`CS_FLAG_ACCEL`, encoders only): the per-detent step count
  multiplies with rotation speed, measured as the gap between detents:
  >= 128 ms x1, < 128 ms x2, < 64 ms x4, < 32 ms x8. Enum nouns never
  accelerate (always one position per detent). Recommended for volume and
  frequency encoders; leave off for fine-trim duties.

### 6.4 Potentiometers / faders (ADC GPIO only)

- **Only GPIO 26, 27, 28** (ADC channels 0-2, on both RP2040 and RP2350). GPIO 29
  is the board's VSYS/3 monitor and is excluded. A pot on any other GPIO is
  rejected with `CS_STATUS_PIN_NOT_ADC`.
- **Full-range or custom span.** With `range_min == range_max == 0`, the knob
  spans the noun's full range; otherwise it spans `[range_min, range_max]`
  (unit-encoded, must satisfy `range_min < range_max` and lie inside the noun
  range).
- **`CS_FLAG_REVERSE`** inverts the direction (CW = down).
- **Conditioning**: the 12-bit ADC reading is smoothed with an EMA (shift 3),
  gated by a **12-count deadband** (~0.3%) so electrical jitter does not
  re-dispatch, and quantized per the noun's unit (half-dB / half-percent /
  1/24 octave).
- **Immediate takeover with boot sync.** At claim the engine seeds the filter
  and waits ~50 pot-service rounds for the EMA to settle (~50 ms with one pot;
  scaled by the round-robin, so ~150 ms with three), then takes the knob's
  physical position as the value (an absolute dispatch). There is no
  "pick-up"/"catch" behavior: the knob is authoritative as soon as it settles.
  **The knob owns its parameter**: shortly after boot (or binding apply) it
  overrides whatever value the preset or a host loaded, so wire pots only to
  parameters the panel should own.

### 6.5 LEDs (1 GPIO)

- Driven **active-high** by default: the GPIO is an output, high = lit.
  **`CS_FLAG_INVERT`** = active-low (drive low = lit), for LEDs wired to 3V3
  through a resistor.
- Actions: `IND_EQUALS` (lit while noun value == `value`) and `IND_ABOVE`
  (lit while noun value >= `value`, continuous nouns; e.g. a signal-present
  LED is `LEVEL` + `IND_ABOVE` with `value = -45 dB`).
- The pin is initialized to "off" at claim and only re-driven on change.
  Indicator evaluation is decimated to **every 8 ms**, staggered across slots.
- **Condition timing (caps v8):** `on_delay`/`off_delay` (0.1 s units) filter
  the raw condition like a PLC TON/TOF timer: it must hold continuously true
  for `on_delay` before the LED lights, and continuously false for
  `off_delay` before it goes out; any blip restarts the pending edge. 0 (the
  default) = immediate. The filter acts on the logical condition;
  `CS_FLAG_INVERT` then only flips drive polarity, so the delays keep their
  meaning for active-low wiring: `on_delay` always gates the
  condition-becoming-true edge, whichever pin level that maps to.
  Timing is wall-clock (`time_us_64`-derived), so a slow or stalled main
  loop (e.g. a flash-write blackout) can only make the LED follow an edge
  late, never early, and cannot stretch a minute-scale delay; a fresh
  binding starts "off" and counts `on_delay` from activation if its
  condition is already true. Delays on `IND_LEVEL` or any non-LED type are
  rejected with `CS_STATUS_INVALID_VALUE`.

### 6.6 PWM LEDs (1 GPIO, hardware PWM)

- `CS_TYPE_LED_PWM` drives the LED from the pin's hardware PWM slice
  (wrap 4095 at sysclk/16, a ~2 kHz carrier). Nothing else in the firmware
  uses PWM, so all slices are free; still, two PWM LEDs must not land on the
  same **slice+channel output** (e.g. GPIO 0 and GPIO 16 are both slice 0
  channel A on RP2040); that is rejected with `CS_STATUS_PWM_CONFLICT`.
  Two PWM LEDs on the same slice but different channels (e.g. GPIO 0 and 1)
  are fine.
- Actions: `IND_LEVEL` (brightness follows the noun value across the noun
  range or the custom `[range_min, range_max]` span, with a squared
  perceptual curve; a per-channel VU-style meter LED is `LEVEL` +
  `IND_LEVEL`), plus `IND_EQUALS`/`IND_ABOVE` as full-on/off.
- **Brightness ceiling (caps v12):** `base_bright` (percent 1-100, 0 = full)
  scales the computed duty, so a meter keeps its whole sweep and only its top
  end moves; it applies to the full-on/off actions too, which is how a panel
  of indicators is dimmed as a set. The scale is **linear in duty** (the
  perceptual curve belongs to the `IND_LEVEL` mapping, and squaring here
  would round a 1% ceiling to fully off at wrap 4095). It is applied before
  `CS_FLAG_INVERT`, so an active-low LED keeps a full-rail off state.
  A future global Panel Brightness noun is intended to multiply into this
  per-LED value for LEDs that opt in, via a flags byte from `reserved2`.
- **`CS_FLAG_INVERT`** inverts the duty cycle for active-low wiring.
- Refresh is decimated like plain LEDs (8 ms); brightness changes are applied
  only when the computed level changes.
- `on_delay`/`off_delay` (6.5) apply to the `IND_EQUALS`/`IND_ABOVE` full-on/off
  actions; they are rejected on `IND_LEVEL` (a continuous level has no boolean
  edge to time).

### 6.7 Poll budget

The tick runs at **1 kHz**. A detented encoder needs 4 samples per detent, so the
poll comfortably tracks a fast hand-spin (~250 quarter-steps/s -> ~60 detents/s).
Pots are read **one ADC conversion per tick, round-robin** across active pots, so
with the maximum useful three pots each is sampled every ~3 ms; ample for a fader.
Buttons debounce per **pin group**, not per binding, so sharing costs nothing.

### 6.8 IR remote receiver (1 GPIO)

- **Use a demodulating IR receiver module** (e.g. TSOP38238 or any 36-40 kHz
  AGC receiver): VCC to 3V3, GND to GND, OUT to the chosen GPIO. These
  modules idle **high** and pull the line **low during a mark** (carrier
  present); that is the default sense. **`CS_FLAG_INVERT`** selects an
  idle-low / active-high receiver. The firmware enables the internal pull-up
  (pull-down under INVERT), so an open-collector output needs no external
  resistor.
- Any GPIO works (`CS_PINCLASS_ANY`); the receiver is the only Control
  Surfaces component captured by interrupt rather than polled. Both edges on
  the pin are timestamped by a tiny, lowest-priority, RAM-resident interrupt
  handler into a private ring; all decoding still happens on the 1 kHz tick.
  The handler only records timestamps, so it cannot disturb the audio path,
  and IR reception has no polling cost when the line is idle.
- **Carrier frequency tolerance.** A 38 kHz receiver demodulates 36-40 kHz
  remotes with reduced range; virtually every consumer remote works. The
  decoders match on demodulated mark/space timing with about 25 % tolerance.
- **Protocols.** NEC and extended NEC decode natively, including the
  dedicated repeat frame (so hold-to-repeat volume works exactly like a held
  button). RC5 and RC6 mode 0 (Philips and MCE-style remotes) decode
  natively with the **toggle bit masked**; without that, every second press
  of the same button would carry a different code and a learned button would
  only work half the time. Every other protocol falls back to a stable
  timing-signature hash, so **any remote can be learned**; such remotes
  repeat by re-transmission, which the hold model treats as repeats
  (section 7.5).

---

## 7. Runtime behavior details

### 7.1 BUSY retry latch (absolute targets, no double-toggle)

Actions resolve to an **absolute target** at event time, never a relative "toggle
again". If a dispatch returns `CTRL_DISPATCH_BUSY` (a USB control SET is
mid-flight, or a shared deferred-apply slot is owed), the engine latches the
resolved target and retries it on the next tick, before sampling new events.
Because the target is absolute, a retry can never double-apply (e.g. a toggle
cannot flip twice; a `MOMENTARY` release restores exactly the captured value).
Rapid encoder detents accumulate correctly across a BUSY stall: the base for
the next step is the latched pending target, not the stale live value.

### 7.2 Boot bring-up order and pin collisions

`control_surfaces_init()` runs **last** in `core0_init()`, after `preset_boot_load`,
all audio/output pin claims, `notify_init`, and the UART/I2C control interfaces,
so pin-conflict checks are truthful and dispatched writes see initialized state.
For each stored non-empty binding it calls `control_surfaces_apply_binding`. If a
binding's pins now collide with something claimed earlier (a moved output pin, an
enabled control interface, etc.):

- The binding is **kept down** (inactive) but its config is **preserved** in the
  live table, so it round-trips unchanged and re-activates once the conflict
  clears.
- The failure code appears in that slot's `slot_status[]` (readable via
  `REQ_GET_CS_STATUS`); the `active_mask` bit stays 0.

Pot and switch boot-sync dispatches do not fire from `init`; they happen from the
first main-loop ticks.

### 7.3 Notification behavior

- **Parameter changes a control makes DO notify.** Each dispatch is tagged
  `PARAM_SRC_GPIO` (5), so hosts see the normal `PARAM_CHANGED` event and can
  reflect a physical knob live.
- **Binding-config changes do NOT notify.** `REQ_SET_CS_BINDING` only does
  GPIO/flash work; there is no `PARAM_CHANGED` for the binding table itself. A
  host that just wrote a binding should **re-read** it (`REQ_GET_CS_BINDING`) and
  poll `REQ_GET_CS_STATUS` for the result; it will not receive a push.

### 7.4 Concurrent writers

A physical filter knob and a host editing the same filter both funnel through
the shared EQ handler; the last write wins per full packet. The engine's RMW
reads the live recipe at event time and its BUSY guard prevents it from
overwriting a not-yet-applied host write, but interleaved host and knob edits
of two different fields of the same band within one apply window resolve to
the later packet. This matches the existing multi-host behavior of the EQ
command surface.

### 7.5 IR runtime model (capture, hold, repeat, momentary)

Capture and decode: the receiver's edges are timestamped by the interrupt
handler; on each 1 kHz tick the engine drains them, and a space longer than
10 ms terminates a frame. The frame is tried against NEC (a 3-period frame
matching 9 ms / 2.25 ms / 560 us is the NEC repeat frame), then RC5, then
RC6 mode 0, then the hash fallback (frames of fewer than 8 periods that
match no protocol are discarded as ambient noise). The decoded
protocol+code fires every live command learned to it.

**Hold model (one button at a time).** A decoded frame marks its code as
held. The hold extends while frames keep arriving within **250 ms** of each
other: a NEC repeat frame, or a full re-transmission of the same code (how
RC5/RC6/Sony and most hash-protocol remotes signal a held button). Silence
past 250 ms is the release. A different code releases the old button and
presses the new one.

- **Press actions** (`INC`, `DEC`, `TOGGLE`, `SET`, `TRIGGER`) fire once on
  the initial frame.
- **Hold-to-repeat** (`CS_FLAG_REPEAT` on `INC`/`DEC`): repeats gate on the
  same feel as a physical button: nothing for the first **400 ms** of the
  hold, then at most one step per **80 ms** (12.5 Hz), clocked by the
  remote's own repeat cadence (NEC ~108 ms, RC5 ~114 ms). Remotes that send
  a fixed burst per tap (e.g. Sony SIRC transmits 3 frames per press) fall
  inside the 400 ms delay and do not spuriously step.
- **`MOMENTARY`** engages on the press and restores on the release, which is
  detected by the 250 ms silence, so an IR momentary releases about a
  quarter-second after the finger leaves the button. Removing or rebinding
  the IR component (including a revert) restores any engaged momentary
  first.
- Every dispatch is the same absolute-target, BUSY-retried, shadow-stepped
  path as a physical button (7.1, 5.3), tagged `PARAM_SRC_GPIO`.

While a **learn** is armed (section 3.6.1), decoded frames are captured
instead of dispatched, and the 250 ms hold state is force-released at arm
time so nothing stays engaged into the learn.

#### 7.5.1 Learn notification (event `0x0A`)

Learn completion is pushed on the notification stream (USB EP `0x83` and
UART type-`0x40` frames; see notification_protocol_v2_spec.md):

```
[ver=2, evt=0x0A, flags=0, seq, state, protocol, 0, 0, code_LE32]   (12 bytes)
```

`state` = 2 (`CS_IR_LEARN_DONE`, protocol/code valid) or 3
(`CS_IR_LEARN_TIMEOUT`, protocol/code zero). Exactly one event is pushed per
armed learn that completes; a cancel pushes nothing. Binding and IR command
**config** changes still push nothing (7.3); parameter changes an IR command
makes push the normal `PARAM_CHANGED` with `source = PARAM_SRC_GPIO`.

---

## 8. App integration patterns

All multi-byte fields little-endian. `slot` is 0-15. Field offsets per 2.2:
`type,noun,action,flags @0-3`, `gpio @4-5`, `event,target,index,rsv @6-9`,
`value @10`, `step @12`, `range_min @14`, `range_max @16`, `reserved2 @18-23`.

### 8.1 Enumerate capabilities (do this at connect)

1. `GET 0x86, wValue=0xFFFF` -> 32-byte `CsCapsHeader`. Read `type_count`,
   `noun_count`, `max_bindings`, and the seven `CsTypeDesc` entries.
2. For each noun `n` in `0 .. noun_count-1`: `GET 0x86, wValue=n` -> 12-byte
   `CsNounDesc`. Cache kind, unit, enum_count, range, target kind/count,
   dflags, and the accepted-action mask. Skip nouns with `actions == 0`.
3. Build the picker UI: for each type, offer the nouns whose action mask
   intersects the type's action mask; offer only the intersecting actions.
   For targeted nouns, offer a channel (and band) picker sized by
   `target_count` (and the device's band layout).

### 8.2a Rotary encoder on GPIO 27/28, master volume, 1 dB/detent, accelerated

`type=ENCODER(4)`, `noun=MASTER_VOLUME(1)`, `action=STEP(1)`,
`flags=CS_FLAG_ACCEL(0x08)`, `gpio={27,28}`, `step=256`.

```
04 01 01 08 1B 1C 00 00 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00
```

Send `SET 0x84, wValue=<slot>, payload=<24 bytes>`, then poll `GET 0x87` until
`last_slot == slot` and `last_status != 0x16` (expect `0x00`).

### 8.2b LED on GPIO 20 indicating loudness is on

`type=LED(5)`, `noun=LOUDNESS(3)`, `action=IND_EQUALS(8)`, `gpio={20,0xFF}`,
`value=1`.

```
05 03 08 00 14 FF 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### 8.2c One button, three functions (GPIO 16): mute / next preset / input cycle

Three bindings sharing GPIO 16, distinguished by event:

Slot A, short press toggles user mute (`event=PRESS`):
```
01 02 04 00 10 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```
Slot B, long press steps to the next occupied preset with wrap
(`event=LONG(1)`, `action=INC(2)`, `flags=WRAP(0x04)`):
```
01 06 02 04 10 FF 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```
Slot C, double press cycles the input source
(`event=DOUBLE(2)`, `action=INC(2)`, `flags=WRAP(0x04)`, `noun=INPUT_SOURCE(7)`):
```
01 07 02 04 10 FF 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### 8.2d Sub-crossover frequency pot on GPIO 26, 40..200 Hz

RP2350 example: sub output is DSP channel 12 (8 inputs + output index 4),
crossover band 20. `type=POT(3)`, `noun=FILTER_FREQ(20=0x14)`,
`action=ADJUST(0)`, `target=12(0x0C)`, `index=20(0x14)`,
`range_min=40(0x0028)`, `range_max=200(0x00C8)`.

```
03 14 00 00 1A FF 00 0C 14 00 00 00 00 00 28 00 C8 00 00 00 00 00 00 00
```

### 8.2e PWM meter LED on GPIO 21 following output 1's level

RP2350 example: output 1 is DSP channel 8. `type=LED_PWM(6)`,
`noun=LEVEL(28=0x1C)`, `action=IND_LEVEL(11=0x0B)`, `target=8`.

```
06 1C 0B 00 15 FF 00 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### 8.2f Hold-to-test: momentary siggen button on GPIO 17

`type=BUTTON(1)`, `noun=SIGGEN(25=0x19)`, `action=MOMENTARY(9)`, `value=1`.

```
01 19 09 00 11 FF 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00
```

Configure the signal first (`REQ_SIGGEN_SET_CONFIG`); the button starts and
stops whatever config is applied.

### 8.2g Amplifier trigger on GPIO 22: on with signal, off after 10 min idle

The GPIO (through a driver transistor) feeds an amplifier's 12 V trigger
input. `type=LED(5)`, `noun=INPUT_LEVEL_MAX(51=0x33)`,
`action=IND_ABOVE(10=0x0A)`, `value=-50 dB(0xCE00)`,
`off_delay=6000(0x1770)` = 600 s. The pin goes high the moment any channel
of the active input crosses -50 dB and drops only after 10 unbroken minutes
below it.

```
05 33 0A 00 16 FF 00 00 00 00 00 CE 00 00 00 00 00 00 00 00 70 17 00 00
```

Two integration cautions. Keep the threshold above -60 dB: the meter floor
is -60 (silence and "no stream" both read as it), and `IND_ABOVE` at exactly
-60 matches everything. And any rebind, `REQ_CS_REVERT`, or boot re-apply of
the slot briefly releases the GPIO and restarts the filter from "off"; for a
real amplifier trigger that is a power cycle, so reconfigure the panel with
the amp path muted or expect the amp to drop out.

### 8.3 Clear a binding

Send a binding with `type = 0` (`CS_TYPE_NONE`) to the slot; the rest of the
bytes are ignored. This releases the slot's pins and marks it idle.

```
SET 0x84, wValue=<slot>, payload = 24 x 00
```

`last_status` returns `PIN_CONFIG_SUCCESS`.

### 8.4 Read back and display live status

- `GET 0x85, wValue=<slot>` -> the live 24-byte `CsBinding` for editing/display.
- `GET 0x87` -> `CsStatusPacket`: `active_mask` (uint16) tells you which slots
  are live; `slot_status[slot]` gives per-slot health (0 = ok, else a failure
  code from section 3.3, e.g. a boot pin collision).
- To reflect live control activity (a knob being turned), subscribe to the
  notification stream and watch for `PARAM_CHANGED` events with
  `source == PARAM_SRC_GPIO` (5).

### 8.5 IR remote walk-through (component, learn, commands, save)

The full Console flow for "volume up/down plus mute on the living-room
remote", receiver on GPIO 22:

1. **Create the component.** Binding with `type=IR(7)`, `gpio[0]=22`,
   everything else zero, to a free slot:
   ```
   07 00 00 00 16 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
   ```
   `SET 0x84, wValue=<slot>`, poll `0x87` until not PENDING. Optionally name
   the slot ("Living Room Remote") via `0x8B`.
2. **Learn the first button.** `GET 0x8F, wValue=1`, then prompt the user to
   press volume-up on the remote. Wait for notify event `0x0A` (or poll
   `GET 0x8F, wValue=2` until `state != 1`). Suppose the result is
   `protocol=1 (NEC), code=0xE718FF00`.
3. **Write the command.** Sub-slot 0: `noun=USER_VOLUME(0)`, `action=INC(2)`,
   `flags=REPEAT(0x10)` so holding the button ramps, `step=0x0100` (1 dB):
   ```
   00 02 10 00 00 01 00 00 00 01 00 00 00 FF 18 E7
   ```
   `SET 0x8D, wValue=0`, poll `0x87` (`last_slot` reads `0x80`).
4. **Repeat** for volume-down (`action=DEC`) into sub-slot 1 and mute
   (`action=TOGGLE`, `noun=USER_MUTE`) into sub-slot 2, learning each button
   first.
5. The commands are already live (that is the Apply); the user tries the
   remote. When happy: `GET 0x9D` (**Save**), poll `0x87` until not PENDING
   with `last_slot=0xFF`. To abandon instead: `GET 0x9E` (**Revert**).

To display the component later: `GET 0x85` per slot to find `type=7`,
`GET 0x8E` per sub-slot for the commands (protocol `0` = empty),
`GET 0x87` for `ir_active_mask` / `ir_cmd_status` health, and `GET 0x8C`
for the component name.

---

## 9. Extension guide (firmware)

The engine (control_surfaces.c) is noun-agnostic; parameter-specific code lives
in control_surfaces_nouns.c. Hosts read caps at runtime, so adding capability
does not break existing hosts.

### 9.1 Add a new noun

1. Append a value to `CsNoun` (never renumber; `CS_NOUN_COUNT` grows).
2. Add a `CsNounDesc` row in `cs_noun_table[]` (kind, unit, action mask, range,
   target kind/count, dflags) in control_surfaces_nouns.c.
3. Add a `case` in `cs_noun_get()` (live read, natural units) and in
   `cs_noun_dispatch()` (map to the underlying vendor command). For a
   deferred/pipeline-safe apply, dispatch through `vendor_dispatch_get` like
   `PRESET` does, and set `CS_NDF_DEFERRED` so the engine uses a target shadow.
   If the apply shares a pending slot with other commands (like the EQ packet),
   return `false` (BUSY) while that slot is owed.
4. If the noun is targeted, extend `cs_noun_validate_target()` if the generic
   kinds do not cover it.
5. Document it in section 5 here.

### 9.2 Add a new action

1. Append to `CsAction` (never renumber; `CS_ACT_COUNT` grows). Bit position must
   stay `< 16` (the masks are `uint16`).
2. Add the bit to the relevant type masks (`s_caps.types[]` in
   control_surfaces.c) and the noun action groups (control_surfaces_nouns.c).
3. Implement the behavior in the tick/gesture handlers and any value-bounds
   check in `cs_validate()`.

### 9.3 Add a new component type (e.g. a character display)

1. Append to `CsType` (never renumber; `CS_TYPE_COUNT` grows). Add a `CsTypeDesc`
   row: its action mask, pin count, and pin class.
2. Handle it in `cs_claim_pins`, `cs_release_pins`, `cs_seed_runtime`, and the
   `control_surfaces_tick()` dispatch switch.
3. No wire-format change is needed: `CsBinding` already carries
   type/pins/event/target/value, and hosts pick up the new type from the caps
   header automatically.

`CS_TYPE_IR` is the template for a **container** type whose events arrive
out-of-band rather than by polling its pin: the binding validates with its
own rules (`cs_validate_ir_container`), claim/release attach and detach a
capture module (control_surfaces_ir.c), and per-event work happens in a
dedicated tick hook (`cs_tick_ir`) that funnels into the shared op helpers
(`cs_button_press` and friends) through a binding-shaped view of each
command. New decode protocols slot into `ir_frame_complete` in
control_surfaces_ir.c: append a `CS_IR_PROTO_*` value (never renumber; codes
are flash-persistent) and try the decoder before the hash fallback.

### 9.4 Versioning rules

- **Append-only** enum values; **never renumber** an existing `CsType`, `CsNoun`,
  `CsAction`, or `CsEvent` (they are wire- and flash-persistent).
- Bump `CS_CONFIG_VERSION` **only** when the `CsFlashConfig` byte layout changes
  (adding an enum value does not). A stored blob with a higher version than the
  firmware understands is ignored (feature stays idle) rather than misread.
- Bump `caps_version` when the descriptor semantics change so hosts can adapt.
- A `CsFlashConfig` **layout** change also needs a directory version bump: add a
  new `PresetDirectory_vN` snapshot, extend `load_directory()` with an
  N-1 -> N migration, bump `DIR_VERSION_CURRENT`, and extend
  `dir_sanitize_cs_config()` if new fields need bounds checks (see
  flash_storage.c for the V8 -> V9 pattern, which is exactly this feature's
  v1 -> v2 config migration).

---

## 10. Limits and constraints

- **16 bindings** (`CS_MAX_BINDINGS`). Slots are independent; a slot holds one
  component (or one gesture of a shared button). The IR component occupies
  one slot and carries its own **16 command sub-slots** (`CS_MAX_IR_COMMANDS`);
  one IR component per device.
- **Pin conflicts** use the shared `ctrl_iface_check_pin()`: a pin must be a valid
  GPIO, not already claimed by an output, MCK, SPDIF/I2S RX, ADAT, DAC
  hardware-mute, a UART/I2C control interface, or another live binding (the one
  sanctioned overlap is button-to-button sharing per 6.1). The **I2S clock pair
  is always treated as reserved**, matching the other control-interface pin
  checks. `control_surfaces_owns_pin()` is wired into
  `pin_used_by_fixed_peripheral()`, so no other subsystem can take a live CS pin.
- **One ADC conversion per tick** (round-robin), so pots share the single ADC
  fairly without stalling the 1 kHz loop.
- **No PIO** is used. The engine is main-loop polling with one exception: the
  IR receiver's edges are timestamped by the firmware's only GPIO interrupt
  (IO_IRQ_BANK0, lowest priority, RAM-resident, core 0); decoding still runs
  on the tick. The engine is a cheap no-op when no binding is active. PWM
  LEDs use the otherwise unused hardware PWM slices.
- **Indicator evaluation is decimated** to 8 ms and staggered across slots, so
  many LEDs (including status-struct reads and the meter dB conversion) stay
  cheap on the RP2040's software float.
- **RP2040 XIP placement**: control_surfaces.c, control_surfaces_nouns.c, and
  the decode side of control_surfaces_ir.c execute from flash (XIP); they
  contain no `DSP_TIME_CRITICAL` code and never run inside the IRQs-off
  flash-write window. The IR edge interrupt handler is RAM-resident
  (`__not_in_flash_func`) and touches only registers and its own RAM ring,
  so it survives the flash blackout; edges arriving while interrupts are
  disabled during a flash write are lost and the partial frame is discarded
  by the decoder (press the button again).
- **Platform differences** are carried entirely by the caps tables:
  `target_count` reflects the platform's channel counts, and `ADAT_ACTIVE`
  has an empty action mask on RP2040. Everything else is identical on RP2040
  and RP2350.
- **Output-slot alignment is unaffected.** Every apply goes through the existing,
  known-safe vendor handlers (e.g. preset load's deferred pipeline-safe path), so
  the inviolable inter-slot phase-alignment guarantee is preserved; the Control
  Surfaces engine adds no new audio-path or reset behavior of its own.

---

## 11. Compatibility

### 11.1 v7 -> v8 (caps version 8, directory unchanged)

- **No structure size or stored-config change.** `CsBinding` stays 24 bytes:
  `on_delay` (offset 18) and `off_delay` (offset 20) are carved from the
  former `reserved2[6]`, leaving `reserved2[2]`. Stored and in-flight v7
  bindings carry zeros there, which decode as "no delay", i.e. exactly the
  v7 behavior; the directory does not migrate.
- The delays are accepted only on `CS_TYPE_LED`/`CS_TYPE_LED_PWM` with
  `IND_EQUALS`/`IND_ABOVE`; every other type/action combination (and the IR
  container) must still write 0 there or the binding is rejected with
  `CS_STATUS_INVALID_VALUE`, so a v7 host that zero-fills reserved bytes
  keeps working unchanged.
- One appended noun: `INPUT_LEVEL_MAX` (51, `CS_UNIT_DB`, -60..0, read-only,
  untargeted). `noun_count` reads 52.
- **Downgrade:** a v8 config whose LED binding carries a nonzero delay fails
  v7 firmware's reserved2 all-zero check, so that slot loads inactive
  (`CS_STATUS_INVALID_VALUE` in `slot_status`) rather than reverting to
  undelayed behavior; clear the delays before downgrading if the binding
  must survive.

### 11.2 v6 -> v7 (caps version 7, directory unchanged)

- **No stored-config change and no structure size change.** The directory stays
  V17 and both caps and status structures keep their sizes; nothing migrates.
- The bump signals two appended nouns: `LOUDNESS_SPL` (49, `CS_UNIT_DB`,
  40..100 dB SPL) and `LOUDNESS_INTENSITY` (50, `CS_UNIT_PERCENT`, 0..127 %).
  `noun_count` reads 51. No new units or actions, so a v6 host that enumerates
  nouns from `noun_count` picks them up with no code change.
- The intensity noun's bindable span stops at 127 % even though
  `REQ_SET_LOUDNESS_INTENSITY` accepts 0..200 %: 8.8 percent cannot encode a
  larger value in the int16 wire fields. A host wanting 128..200 % must set it
  over the vendor command directly.

### 11.3 v5 -> v6 (caps version 6, directory V17)

- **Stored configs migrate automatically.** A V16 directory migrates to V17 on
  first boot: `cs_ir` is the last directory member, so every earlier field keeps
  its offset, the eight learned commands carry over verbatim, and sub-slots 8-15
  start empty. Bindings and names are untouched.
- **Response sizes grew.** `REQ_GET_CS_STATUS` returns 41 bytes: `ir_active_mask`
  is now a uint16 at offset 22, pushing `ir_learn_state` to 24 and
  `ir_cmd_status[16]` to 25. `REQ_GET_CS_CAPS` is unchanged at 40 bytes, but
  `max_ir_commands` now reads 16.
- Hosts must size the IR command list from `max_ir_commands` rather than assume
  8, and must not read the status packet at the v5 offsets.
- Firmware downgrade: an older firmware reading a V17 directory fails the version
  check and rebuilds a fresh directory (presets and CS config are lost); it does
  not misparse.

### 11.4 v4 -> v5 (caps version 5, directory unchanged)

- **No stored-config change and no structure size change.** The directory stays
  V11 and both caps and status structures keep their sizes; nothing migrates.
- The bump signals one thing: `UPMIX_CENTER_MODE` now has three values instead
  of two, the third being `Off` (2). Hosts that read `enum_count` from the noun
  descriptor need no change at all; the bump exists for hosts that hard-code
  the mode labels.
- Note the centre enum puts `Off` last while `UPMIX_SURROUND_MODE` puts it
  first. `Off` was appended rather than renumbered because the vendor interface
  has no per-command version negotiation, so moving 0/1 would have silently
  remapped existing hosts and saved presets. A front panel is free to cycle the
  modes in any order it likes.

### 11.5 v3 -> v4 (caps version 4, directory unchanged)

- **No stored-config change.** The directory stays V11; bindings, IR commands,
  and names are byte-identical, so no migration runs and firmware
  up/downgrades across this boundary keep the stored CS config.
- **No structure size change.** `REQ_GET_CS_CAPS` still returns 40 bytes and
  `REQ_GET_CS_STATUS` 32 bytes. The `caps_version` bump to 4 signals the 14
  new nouns (35-48: stereo upmixer, psychoacoustic bass, per-output delay,
  preset reload) and the new `CS_UNIT_MS` unit (5) that `OUTPUT_DELAY` uses.
- A v3 host keeps working unchanged for nouns 0-34; before offering the new
  nouns it must learn `CS_UNIT_MS` (8.8 ms, linear, default step 0.1 ms).
- The six upmixer nouns are RP2350-only: on RP2040 their action masks read 0
  (unavailable), the same convention as `ADAT_ACTIVE`.

### 11.6 v2 -> v3 (caps version 3, directory V11)

- **Stored configs migrate automatically.** A V10 directory migrates to V11
  on first boot by appending the (empty) IR command table; bindings and
  names are untouched. All v2 types, nouns, actions, flags, and status
  codes kept their numeric values.
- **Response sizes grew.** `REQ_GET_CS_CAPS` (`wValue = 0xFFFF`) now returns
  40 bytes (8 type entries plus the v3 tail) and `REQ_GET_CS_STATUS` returns
  32 bytes. Hosts must read `caps_version` (3) and use the v3 sizes; the
  first 22 status bytes kept their v2 layout except that the reserved byte
  at offset 3 became `dirty`.
- **`REQ_SET_CS_BINDING` semantics changed: it no longer persists.** A v2
  host that writes bindings and never sends `REQ_CS_SAVE` will find them
  gone after a reboot. This is the one breaking behavioral change of v3;
  hosts must add Save (and should surface `dirty` / offer Revert).
- Firmware downgrade: an older firmware reading a V11 directory fails the
  version check and rebuilds a fresh directory (presets and CS config are
  lost); it does not misparse.

### 11.7 v1 -> v2

- **Stored configs migrate automatically.** A device with a V8 directory (v1
  16-byte bindings, 8 slots) migrates on first boot: the 8 bindings carry
  over with `event = PRESS`, `target = index = 0`; slots 8-15 are empty. All
  v1 nouns, actions, flags, and status codes kept their numeric values.
  A V9 directory migrates to V10 by appending the (empty) name block; the
  bindings are untouched.
- **The control wire format is NOT backward compatible.** `REQ_SET_CS_BINDING`
  requires 24 bytes (a 16-byte payload gets `CS_STATUS_INVALID_VALUE`);
  `REQ_GET_CS_BINDING`, `REQ_GET_CS_CAPS`, and `REQ_GET_CS_STATUS` responses
  grew. Hosts must read `caps_version` and use the current sizes.
- v1's `IND_EQUALS`-only LED bindings behave identically after migration; the
  `CLIP` noun's semantics are unchanged.
