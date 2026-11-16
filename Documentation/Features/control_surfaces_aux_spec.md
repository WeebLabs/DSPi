# Control Surfaces: Auxiliary Outputs

*Spec version 2; caps v18, directory V21. Companion to
`control_surfaces_spec.md`, which remains authoritative for bindings,
components, actions and the validity model.*

*Last updated: 2026-11-16 (section 3 and the command names on the paged protocol of firmware 1.1.7)*

An auxiliary output is a GPIO the firmware attaches no audio meaning to. It
exists so a front panel can switch and dim things the firmware does not know
about. An amplifier trigger, a speaker relay, a panel lamp, a fan. Two new
component types put the output in a binding slot of its own. `CS_TYPE_AUX_OUT`
is a plain on/off pin, and `CS_TYPE_AUX_PWM` is a dimmable pin on a hardware
PWM slice. The slot owns the pin, the name, the invert sense, the delays and
the boot values, so an aux output is configured exactly the way a button or an
LED is.

Controls reach it by noun. A button, switch, encoder, pot, IR key, macro step
or display page addresses `CS_NOUN_AUX` (68) or `CS_NOUN_AUX_LEVEL` (69) with
`target` set to the slot holding the output.

## 1. Concepts

### 1.1 What an aux output is

A component in one of the 16 binding slots. It carries two live values, an
on/off flag and an 8.8 percent level. Both live in RAM only, change instantly,
and are never written to flash on change. They exist only while the slot is up
(active); a slot held down by a pin conflict reads 0 and refuses writes,
because a value set while the output is down would be lost to the boot fields
when it comes back.

| Type | Value | Pin | What it does |
|------|-------|-----|--------------|
| `CS_TYPE_AUX_OUT` | 9 | 1 GPIO, driven as an output | follows the on/off flag |
| `CS_TYPE_AUX_PWM` | 10 | 1 GPIO on a hardware PWM slice | emits the level while the flag is on |

Both are **container** types, like `CS_TYPE_IR` and `CS_TYPE_DISPLAY`. Their
caps action mask is `0x0000` and the container binding carries no noun, action
or target of its own.

### 1.2 The two nouns

Both use target kind `CS_TARGET_AUX` (5), where `target` is the binding slot
holding the aux component and `index` must be 0. `target_count` reads
`CS_MAX_BINDINGS` (16).

| Noun | Value | Kind | Unit | Actions | Needs |
|------|-------|------|------|---------|-------|
| `CS_NOUN_AUX` | 68 | BOOL | - | BOOL-RW (`TOGGLE`, `SET`, `FOLLOW`, `MOMENTARY`, `IND_EQUALS`) | either aux type in the target slot |
| `CS_NOUN_AUX_LEVEL` | 69 | CONT | PERCENT (0..100 %) | CONT-RW (`ADJUST`, `STEP`, `INC`, `DEC`, `SET`, `IND_ABOVE`, `IND_LEVEL`) | `CS_TYPE_AUX_PWM` in the target slot |

On a PWM output the two nouns compose. The flag gates the pin and the level
sets how bright it is when the flag is on, so a lamp can be switched by a
button and dimmed by an encoder without either control fighting the other.

### 1.3 What can drive one

Anything that already drives a noun. A button (`TOGGLE`, `MOMENTARY`), a
switch (`FOLLOW`), an encoder or pot on the level, an IR command, a macro
step, or a display page in edit mode. Hosts write the values directly with the
commands in section 3. Every one of those paths goes through the same command
surface, so a panel press and a USB write are indistinguishable to the engine
and produce the same notification.

Ordinary LED bindings may still *follow* an aux output as extra indicators. A
`CS_TYPE_LED` with `IND_EQUALS` and `value = 1` on noun 68 lights while the
output is on, and a `CS_TYPE_LED_PWM` with `IND_LEVEL` on noun 69 tracks its
brightness. That is optional decoration now, not the mechanism.

### 1.4 What aux nouns do not do

- **No groups.** A binding that sets `CS_FLAG_GROUP` on an aux noun is
  rejected with `CS_STATUS_INVALID_GROUP`, and a `CsGroup` created with
  `target_kind = 5` is rejected the same way. Groups exist for channel spaces.
- **No `CYCLE_ALL` display pages.** That mode walks untargeted nouns only, so
  aux outputs appear on the display as explicit pages.

---

## 2. Wire reference

### 2.1 An aux slot's `CsBinding` (24 bytes)

The layout is the ordinary `CsBinding` (`control_surfaces_spec.md` 2.2),
unchanged in size and field order. This is what the fields mean on an aux
slot.

| Off | Size | Field | Meaning on an aux slot | `AUX_OUT` | `AUX_PWM` |
|----|------|-------|------------------------|-----------|-----------|
| 0 | 1 | `type` | component type | 9 | 10 |
| 1 | 1 | `noun` | unused | 0 | 0 |
| 2 | 1 | `action` | unused | 0 | 0 |
| 3 | 1 | `flags` | `CS_FLAG_INVERT` (`0x01`) makes the pin active-low | 0 or `0x01` | 0 or `0x01` |
| 4 | 1 | `gpio[0]` | the output pin | any free GPIO | any free GPIO with a free PWM slice output |
| 5 | 1 | `gpio[1]` | unused | `0xFF` | `0xFF` |
| 6 | 1 | `event` | unused | 0 | 0 |
| 7 | 1 | `target` | unused | 0 | 0 |
| 8 | 1 | `index` | unused | 0 | 0 |
| 9 | 1 | `base_bright` | PWM duty ceiling, percent 1-100, 0 = full | 0 | 0-100 |
| 10 | 2 | `value` (int16) | boot level, 8.8 percent | 0 | 0..25600 |
| 12 | 2 | `step` (int16) | unused | 0 | 0 |
| 14 | 2 | `range_min` (int16) | unused | 0 | 0 |
| 16 | 2 | `range_max` (int16) | unused | 0 | 0 |
| 18 | 2 | `on_delay` (uint16) | TON filter on the on/off flag, 0.1 s units, 0 = immediate | any | any |
| 20 | 2 | `off_delay` (uint16) | TOF filter on the on/off flag, same units | any | any |
| 22 | 1 | `extras` | `CS_AUX_X_*` bits (2.2) | `0x01`, `0x02` | `0x01`, `0x02`, `0x04` |
| 23 | 1 | `reserved2` | write 0 | 0 | 0 |

Byte 22 was `reserved2[0]` before caps v18. It is now a type-extras flags byte
and must still be 0 on every type except the two aux types, so a pre-v18
config stays valid unchanged.

The output's name is the ordinary per-slot name, set and read with
`CS.SLOT_NAME`. There is no separate
aux name.

### 2.2 `extras` bits

| Bit | Name | Applies to | Meaning |
|-----|------|-----------|---------|
| `0x01` | `CS_AUX_X_BOOT_ON` | both | the output boots on (clear = boots off) |
| `0x02` | `CS_AUX_X_BOOT_SAVED` | both | `CS.SAVE` folds the live flag and level into `CS_AUX_X_BOOT_ON` and `value` before writing |
| `0x04` | `CS_AUX_X_LINEAR` | `AUX_PWM` only | linear duty instead of the squared perceptual curve; rejected on `AUX_OUT` |
| | `CS_AUX_X_MASK` = `0x07` | | any other bit set is `CS_STATUS_INVALID_VALUE` |

`CS_AUX_X_LINEAR` is for loads whose response is already linear, a fan or a
heater rather than a lamp. Leave it clear for anything the eye judges.

### 2.3 Caps v18

`caps_version` reads 18. `CS_TYPE_COUNT` is 11, so `type_count` reads 11 and
the header is `4 + 4*11 + 4 = 52` bytes. Hosts must keep locating the v3 tail
at `4 + 4*type_count`, as always. Both new type descriptors read
`{ actions = 0x0000, pin_count = 1, pin_class = CS_PINCLASS_ANY }`.

No header field is added or moved, and `CsBinding`, `IrCommand`,
`CsStatusPacket` and `CsFlashConfig` all keep their sizes.

---

## 3. Commands (`CS` page 0x0A)

The address is the binding slot, 0-15. All three work over USB, UART and I2C,
and the two SETs are the same commands the engine itself dispatches
(section 6.1).

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `CS.AUX_STATE` | 0x0A | 0x60 | GET, SET | slot 0-15 | GET 1, SET 1 | Non-zero = on. SET applies immediately |
| `CS.AUX_LEVEL` | 0x0A | 0x61 | GET, SET | slot 0-15 | GET 2, SET 2 | 8.8 percent LE. SET applies immediately |
| `CS.AUX_ALL` | 0x0A | 0x62 | GET | none (0) | GET 48 | Every slot's state and level (3.3) |

The caps v17 aux config commands no longer exist.

### 3.1 Runtime SET (`CS.AUX_STATE`, `CS.AUX_LEVEL`)

Applied in the handler, with no flash write, no dirty flag and no deferral.
`CS.AUX_STATE` treats any non-zero byte as on. `CS.AUX_LEVEL` takes 8.8
percent little-endian and clamps above 100 % (25600). A write that leaves the value unchanged is
accepted silently and sends no notification, matching how parameter writes
behave elsewhere.

A write to a slot that is not an aux output that is up (or a level write to an
`AUX_OUT` slot) STALLs with `VC_ERR_STATE` and also records
`CS_STATUS_INVALID_AUX` in the shared Control Surfaces status channel, where
`cs_last_slot` is the plain binding slot. There is no `0x70 | aux` tag any
more. A bad slot is `VC_ERR_ADDRESS` and a wrong payload length is
`VC_ERR_LENGTH`. Neither touches the status channel. A write during a revert
is `VC_ERR_BUSY` and leaves the status channel alone. UART and I2C return the
same code in the response status byte, and every transport can read the
reason from `SYS.LAST_RESULT`. `CS.STATUS` and `slot_status` report an aux
slot like any other binding slot.

### 3.2 Runtime GET (`CS.AUX_STATE`, `CS.AUX_LEVEL`)

`CS.AUX_STATE` returns one state byte. `CS.AUX_LEVEL` returns two bytes of 8.8
percent, which read 0 on an `AUX_OUT` slot because a digital output has no
level. Either STALLs with `VC_ERR_STATE` if the slot is not an aux output that
is up.

### 3.3 Bulk state read (`CS.AUX_ALL`)

Returns **48 bytes**. Sixteen `state` bytes, one per binding slot, followed by
sixteen little-endian uint16 `level_q8` values in the same slot order. Slots
that are not up aux outputs read zero in both halves. One transfer gives a
host its whole initial aux picture, after which the notification in section 5
keeps it current.

### 3.4 Validation table

| Fault | Status |
|-------|--------|
| `target` >= 16, or `index` != 0, on a binding / IR command / macro step / display page | `CS_STATUS_INVALID_TARGET` (`0x17`) |
| noun 68 whose target slot holds neither aux type | `CS_STATUS_INVALID_AUX` (`0x26`) |
| noun 69 whose target slot is not `CS_TYPE_AUX_PWM` | `CS_STATUS_INVALID_AUX` (`0x26`) |
| a binding on noun 68 / 69 whose target is the slot it is being written to | `CS_STATUS_INVALID_TARGET` (`0x17`) |
| `CS.AUX_STATE` / `CS.AUX_LEVEL` SET on a slot that is not an aux output that is up | `CS_STATUS_INVALID_AUX` (`0x26`), SET refused with `VC_ERR_STATE` |
| `CS.AUX_LEVEL` SET on an `AUX_OUT` slot | `CS_STATUS_INVALID_AUX` (`0x26`), SET refused with `VC_ERR_STATE` |
| `CS.AUX_STATE` / `CS.AUX_LEVEL` SET payload other than 1 / 2 bytes | no status record, SET refused with `VC_ERR_LENGTH` |
| `CS.AUX_STATE` / `CS.AUX_LEVEL` GET on a slot that is not an aux output that is up | no status record, GET refused with `VC_ERR_STATE` |
| `CS_FLAG_GROUP` on an aux noun, or a `CsGroup` with `target_kind = 5` | `CS_STATUS_INVALID_GROUP` (`0x1F`) |
| aux container with a non-zero `noun`, `action`, `event`, `target`, `index`, `step` or range field | `CS_STATUS_INVALID_VALUE` (`0x14`) |
| `extras` outside `CS_AUX_X_MASK`, or non-zero on a non-aux type | `CS_STATUS_INVALID_VALUE` (`0x14`) |
| `CS_AUX_X_LINEAR`, `base_bright` or a non-zero `value` on `AUX_OUT` | `CS_STATUS_INVALID_VALUE` (`0x14`) |
| `value` outside 0..25600 on `AUX_PWM` | `CS_STATUS_INVALID_VALUE` (`0x14`) |
| two PWM components (aux or LED) sharing one slice output | `CS_STATUS_PWM_CONFLICT` (`0x19`) |

`CS_STATUS_INVALID_AUX` (`0x26`) now means "the target slot is not an aux
output, or the level noun targets a non-PWM aux slot". Its caps v17 meaning
(an aux index out of range) is gone.

---

## 4. Boot, save and revert

**Boot.** The flag comes from `CS_AUX_X_BOOT_ON` and the level from `value`,
whether or not `CS_AUX_X_BOOT_SAVED` is set. That bit changes only what a save
writes into those two fields, never how they are read at boot. The delay
filter does not apply to the boot value; the pin
comes up at it directly. Aux slots are applied **before** every other slot
and before the display pages load, so a control or page that targets one
validates whatever order the slots happen to sit in. One consequence of that
order is worth knowing. When two stored slots claim the same GPIO, the aux
slot now wins and the other slot comes up `PIN_CONFIG_PIN_IN_USE`, where a
lower-numbered non-aux slot used to win.

**Claim order.** The pin is driven to its initial value *before* it is
switched to an output (or, for PWM, before the pin is muxed to the slice), so
a relay never sees a glitch through the off state on the way up.

**Save (`CS.SAVE`).** `control_surfaces_aux_prepare_save()` runs first.
For every up aux slot carrying `CS_AUX_X_BOOT_SAVED` it copies the live flag
into `CS_AUX_X_BOOT_ON` and, on `AUX_PWM`, the live level into `value`. The
whole Control Surfaces config then persists in the one directory write.
Toggling an output never writes flash and never sets the dirty flag.

**Revert (`CS.REVERT`).** Reloads the stored config, but carries the live
flag and level of every up aux slot across to the slot that reloads with the
same type. A revert therefore never snaps a relay to its boot value. The pin
is released and reclaimed during the reload, a gap of microseconds, exactly as
for any LED.

**Same-type re-apply.** A host `CS.BINDING` SET on an aux slot that keeps
the same type (a rename, a delay change, a ceiling change, a boot-field edit)
also keeps the live value. Changing the type resets the slot to its boot
value.

**Dirty flag caveat.** Because live values are not configuration, they are not
part of the dirty flag. One consequence follows for `BOOT_SAVED` slots. A save
can change the stored boot fields while the flag reads clear, and a later
revert restores those newly saved values rather than the ones from before the
save. Hosts that show "unsaved changes" should treat `BOOT_SAVED` slots as
always potentially different from flash.

**Never on change.** There is deliberately no "remember on every change" mode.
A flash write freezes the audio clocks for roughly 44 ms, so a mode that
persisted on every toggle would put a flash write behind a front-panel button.
`CS_AUX_X_BOOT_SAVED` covers the same intent at save time instead.

---

## 5. Notification (`NOTIFY_EVT_CS_AUX`, `0x0C`)

Every `CS.AUX_STATE` and `CS.AUX_LEVEL` SET that changes a value pushes a **9-byte**
packet, from any source.

```
[ver=2, evt=0x0C, flags=0, seq, slot, state, level_q8_LE(2), src]
```

| Off | Size | Field |
|----|------|-------|
| 0 | 1 | `ver` = 2 |
| 1 | 1 | `evt` = `0x0C` |
| 2 | 1 | `flags` = 0 |
| 3 | 1 | `seq` |
| 4 | 1 | `slot` (binding slot 0-15) |
| 5 | 1 | `state` (0/1) |
| 6 | 2 | `level_q8` (uint16 LE, 8.8 percent; 0 on an `AUX_OUT` slot) |
| 8 | 1 | `src` (`ParamSource`) |

Both values are carried on every event, so a host never has to read back to
learn which of the pair moved. `src` is the `ParamSource` of the dispatch.
`PARAM_SRC_GPIO` (5) means a bound control, IR command or macro step,
`PARAM_SRC_HOST_SET` (1) a USB write, and `PARAM_SRC_UART` (8) /
`PARAM_SRC_I2C` (9) the external transports. This is how the Console learns
that a panel button changed an aux output.

Binding changes push nothing new; an aux slot follows the existing
poll-the-status model of every other Control Surfaces SET.

---

## 6. Engine and display behaviour

### 6.1 Dispatch

`CS_NOUN_AUX` and `CS_NOUN_AUX_LEVEL` go through
`vc_set(CTRL_SOURCE_GPIO, ...)` with `CS.AUX_STATE` / `CS.AUX_LEVEL`, exactly
like every other noun. Nothing about aux outputs bypasses the command surface,
which is why a panel toggle produces the same notification a host write does,
tagged as GPIO. The level is 8.8 percent in RAM, on the wire and in the noun
value, so any `step` is valid and there is no rounding nudge. The caps v17
whole-percent step restriction is gone.

### 6.2 Pin drive

An `AUX_OUT` pin follows the on/off flag through the `on_delay` / `off_delay`
TON/TOF filter, exactly as an indicator LED follows its condition, with
`CS_FLAG_INVERT` applied last.

An `AUX_PWM` pin emits its level only while the flag is on, so off is 0 % duty
and there is no half-lit state. The duty is `(level/100)^2` by default, or
linear with `CS_AUX_X_LINEAR`, then scaled by `base_bright`, then inverted if
`CS_FLAG_INVERT` is set. The slice runs at wrap 4095 and sysclk/16, the same
carrier as a PWM LED, and shares the same slice-output conflict rule
(`CS_STATUS_PWM_CONFLICT`).

The tick runs at the indicator decimation, every 8 ms per slot, and writes the
pin only when the computed value changes.

### 6.3 Dependents

When an aux slot appears, vanishes or changes kind, every non-grouped binding,
IR command and macro step on nouns 68 or 69 targeting that slot is
re-validated. Active bindings that no longer validate go down with
`CS_STATUS_INVALID_AUX` and release their pins; down ones are resurrected if
they validate again. This mirrors the group-edit rule.

Display pages are **not** re-validated. A page whose target stops being an aux
simply shows `Off` or `0 %`.

### 6.4 Display

An aux page's label is the slot's name when one is set, otherwise `Aux N` with
N = slot + 1. A level page appends ` Level` (` Lvl` on the tight two-row bar
layout) so the two pages of one output stay distinguishable. Values render as
`On` / `Off` for the bool noun and `NN%` for the level, which supports the
caps v13 level bar. Both nouns are writable, so a page can be edited from the
panel through `DISPLAY_EDIT` / `PAGE_VALUE`, and a host-driven change pops the
event overlay like any other watched page.

---

## 7. Wiring examples

### 7.1 Amp trigger relay on a button

| Slot | Type | Noun | Action | Fields |
|------|------|------|--------|--------|
| 0 | `BUTTON` (GPIO 16) | `AUX` (68) | `TOGGLE` | `target = 1`, `event = CS_EVT_PRESS` |
| 1 | `AUX_OUT` (GPIO 21) | - | - | `CS_FLAG_INVERT` for an active-low relay board, `extras = 0` so it boots off |

Name slot 1 "Amp" with `CS.SLOT_NAME`. Leaving `CS_AUX_X_BOOT_ON` clear is
what keeps the amp from waking up with the device. Add an `off_delay` on slot
1 if the relay should hold in through a quick power blip.

### 7.2 Panel lamp dimmer on an encoder

| Slot | Type | Noun | Action | Fields |
|------|------|------|--------|--------|
| 2 | `ENCODER` (GPIO 18/19) | `AUX_LEVEL` (69) | `STEP` | `target = 3`, `step = 5 %` (1280 in 8.8) |
| 3 | `AUX_PWM` (GPIO 22) | - | - | `extras = CS_AUX_X_BOOT_SAVED`, `base_bright` to cap the top end |
| 4 | `BUTTON` (GPIO 17) | `AUX` (68) | `TOGGLE` | `target = 3`, `event = CS_EVT_PRESS` |

The encoder dims the lamp and the button switches it. That switch is the part
caps v17 could not do without a second indicator binding; the flag now gates
the same pin the level drives. `CS_AUX_X_BOOT_SAVED` brings the lamp back at
the brightness and on/off position it had at the last save.

### 7.3 Aux as a menu item

Add a display page with `noun = 68`, `target = 3` and a second with
`noun = 69`, `target = 3`, then bind a button to `PAGE_VALUE` with `TOGGLE`
(or an encoder with `STEP`). The pages read "Lamp On" and "Lamp Level 40%" if
slot 3 is named "Lamp", and the same control edits whichever page is shown. An
unnamed slot 3 reads "Aux 4", since the fallback label is `Aux N` with
N = slot + 1.

---

## 8. Electrical notes

A Pico GPIO is a 3.3 V pin that can source a few milliamps. It can drive an
LED through a resistor and the gate of a small MOSFET, and nothing else.
Anything real (a relay coil, an amplifier trigger, a lamp, a fan) needs a
logic-level MOSFET, a transistor with a flyback diode, or an opto-isolated
relay module between the pin and the load. Most such modules are active-low,
which is what `CS_FLAG_INVERT` is for.

Two further cautions belong in a host UI next to these controls:

- A PWM-dimmed load sharing the 3.3 V rail can couple switching noise into
  the DAC. Power the load from its own supply and use the GPIO only as a
  control signal.
- Amplifier trigger inputs vary. Some expect 12 V, some a dry contact. A
  3.3 V pin is not a trigger output on its own.

### 8.1 Slot budget

One aux output costs one binding slot. A button or encoder driving it costs
another, the same as any other control. A switched-and-dimmed lamp with a
display page is therefore three slots of the sixteen.

---

## 9. Persistence and migration (directory V21)

Aux components persist inside `CsFlashConfig`, the 388-byte bindings blob,
like every other slot. Nothing else was added to flash. The blob is
board-level, so it survives preset changes and a factory reset, and it is not
part of `WireBulkParams`.

`DIR_VERSION_CURRENT` is 21, and **V21 is byte-identical to V19** (3035
bytes). The caps v17 model had appended a 292-byte `CsAuxConfig` at V20, and
V21 removes it again.

- **V19 -> V21** copies the whole V19 data block unchanged. `PresetDirectory_v19`
  stays frozen at 3035 bytes, pinned by a `_Static_assert` alongside the
  `cs_display` offset check.
- **V20 -> V21** verifies the V20 CRC, then copies the V20 data block up to
  (and excluding) the 292-byte aux blob and discards the blob. Any aux output
  configured on a V20 build is lost and must be recreated as a binding-slot
  component. `PresetDirectory_v20` is kept as a frozen 3327-byte layout for
  this migration alone, with the dropped block declared as an opaque
  `uint8_t[292]`.
- Every older directory version migrates straight to V21.

RAM cost is roughly **130 B of BSS** on both platforms, a 4-byte carry entry
per slot for the revert / re-apply value stash plus 3 bytes per slot of
runtime record. The caps v17 model's roughly 650 B is gone. No new code sits
on a hot path, and the audio path and output slot alignment are untouched.

---

## 10. What changed from spec version 1

Spec version 1 (caps v17, directory V20) was never shipped to any device or
host. Everything below replaces it rather than extending it.

- Auxiliary outputs are **components in binding slots**, not a separate table
  of eight pinless values. `CS_TYPE_AUX_OUT` (9) and `CS_TYPE_AUX_PWM` (10)
  are new; `CS_TYPE_COUNT` is 11 and the caps header is 52 bytes.
- The slot owns the pin. A second LED or PWM LED binding is no longer needed
  to reach hardware, and LED follows are now optional extra indicators.
- `CsAuxCfg` (36 B) and `CsAuxConfig` (292 B) are gone, and with them
  the aux config commands (then opcodes 0x02 and 0x03).
- Boot mode became two `extras` bits, `CS_AUX_X_BOOT_ON` and
  `CS_AUX_X_BOOT_SAVED`, on `CsBinding` byte 22 (formerly `reserved2[0]`).
  `CS_AUX_X_LINEAR` was added for non-perceptual loads.
- The name is the ordinary per-slot name (`CS.SLOT_NAME`); there is no
  separate aux name.
- `target` is now the **binding slot** 0-15, not an aux index 0-7, and
  `target_count` reads 16. Validation is strict at bind time.
- `CS_STATUS_INVALID_AUX` (`0x26`) changed meaning from "index out of range"
  to "target slot is not an aux output, or the level noun targets a non-PWM
  aux slot".
- The level is 8.8 percent everywhere, not whole percent. The level SET takes
  two bytes, the level GET returns two, the bulk read (now `CS.AUX_ALL`) is 48
  bytes rather than 16,
  and the whole-percent `step` restriction and its dispatch nudge are gone.
- `NOTIFY_EVT_CS_AUX` grew from 8 to 9 bytes, carrying the slot and a 16-bit
  level.
- The `0x70 | aux` `cs_last_slot` tag is gone; an aux slot reports as a plain
  binding slot.
- Directory V21 restores the V19 layout. V20 is frozen for migration only.
