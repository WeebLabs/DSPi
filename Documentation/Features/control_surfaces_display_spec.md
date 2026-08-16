# Control Surfaces: I2C Character Displays (caps v10 bundle)

*Spec version 2, implemented at caps v10 / directory V19. Companion to
`control_surfaces_spec.md` and `control_surfaces_groups_macros_spec.md`.
Caps v10 bundles three additions: the display component, IR remote group
support, and four new nouns (CPU_LOAD, DISPLAY_PAGE, DISPLAY_EDIT,
PAGE_VALUE). Caps v11 adds per-line horizontal alignment and the bracketed
edit markers (s5.1); no structure size changes.*

A display is a new Control Surfaces component: a small I2C character or
OLED module wired to two spare GPIOs that shows what the device is doing.
The content model is deliberately hi-fi-shaped: an **event overlay** pops
the parameter that just changed (volume, preset, input, or whatever knob
the user is turning) for a hold time, then the screen falls back to a
**home page**: either one fixed page, a rotation through a user-chosen
page set, or a rotation through everything, with a configurable dwell.

Displayable items are not a new catalog: a page is `{noun, target}` from
the existing noun table, rendered generically by the noun's kind and unit.
Anything a control can bind to, a display can show, including groups
("Fronts  -12.0 dB"), and future nouns appear on displays automatically.

## 1. Concepts

### 1.1 The component

`CS_TYPE_DISPLAY` (8) occupies one binding slot and is a container like the
IR receiver: one live display per device, `gpio[0]` = SDA, `gpio[1]` = SCL,
`index` = model, `value` = 7-bit I2C address (0 = the model's conventional
default), every other field 0. The device is I2C **master** on whichever
hardware I2C instance those pins map to; that instance must not be the one
the external control interface (I2C target mode) occupies. Validation
enforces: SDA on an even GPIO, SCL on an odd GPIO, both mapping to the same
instance per the RP2040/RP2350 pin mux (GPIO bit 1 selects i2c0/i2c1, bit 0
selects SDA/SCL), instance clear of the control interface
(`i2c_ctrl_live_instance()`), plus the standard pin-conflict checks. The
display module owns the pin mux, the internal pull-ups and the peripheral;
the engine only calls attach/detach.

### 1.2 Models (v1)

| Model | Module | Text geometry | Default addr | Bus |
|---|---|---|---|---|
| 1 | LCD1602 (HD44780 + PCF8574 backpack) | 16x2 | 0x27 | 100 kHz; 3.3 V wiring note in s7 |
| 2 | LCD2004 (HD44780 + PCF8574 backpack) | 20x4 | 0x27 | 100 kHz |
| 3 | Character OLED 16x2 (US2066/RW1063) | 16x2 | 0x3C | 400 kHz, native I2C, 3.3 V |
| 4 | Character OLED 20x2 (US2066/RW1063) | 20x2 | 0x3C | 400 kHz |
| 5 | Character OLED 20x4 (US2066/RW1063) | 20x4 | 0x3C | 400 kHz |
| 6 | SSD1306 128x64 | 21x8 small font (12 LARGE columns) | 0x3C | 400 kHz |
| 7 | SSD1306 128x32 | 21x4 small font (12 LARGE columns) | 0x3C | 400 kHz |
| 8 | SH1106 128x64 | 21x8 small font, 2-column panel offset | 0x3C | 400 kHz |

Model 0 marks the enum origin (no display); ST7032/AiP31068 modules are a
candidate model family for a later revision. Bus speed is fixed per model
(the PCF8574 tops out at 100 kHz; everything else runs 400 kHz). Each model
also fixes which physical row carries the label and which carries the value,
so 4-row modules render centred (label on row 1, value on row 2) rather than
against the top edge.

### 1.3 Pages

Up to 16 page slots, device-global, each 4 bytes: `{noun, target, index,
flags}`. Flags: bit0 = ACTIVE (all-zero record = empty slot), bit1 = GROUP
(`target` is a group index, same rules as bindings), bit2 = LARGE (render the
value pixel-doubled on graphic OLEDs; ignored on character modules).
Rendering is generic: the label line is the item's name, the value line its
value formatted by unit (dB, Hz, percent, ms, on/off, enum name). Labels
come from a flash string table per noun; targeted pages prefix the channel
("Out3 Gain") or the group's user-assigned name ("Fronts Gain"). Enum nouns
with fixed value sets render names from per-noun label tables: INPUT_SOURCE,
SAMPLE_RATE, CROSSFEED_PRESET (Default/Chu Moy/Meier/Custom), LEVELLER_SPEED
(Slow/Medium/Fast), FILTER_TYPE, and the upmix centre/surround modes
(passive = "Sinner", adaptive = "Logician"). FILTER_TYPE covers all PEQ
types 0-13 (including the host-only Linkwitz and first-order low/high pass)
with slope-based names: PEQ low/high pass display as High Cut / Low Cut, and
shelves and cuts carry their slope ("Low Shelf 12dB", "High Cut 6dB"). A
value rendered in the LARGE font swaps to an abbreviated table sized for its
12 columns ("LS 12dB/oct", "HC 6dB/oct"); character modules always use the
long names. Special cases: a PRESET page
shows the active preset's stored name, a MACRO page the macro's name, a
grouped page whose group is empty renders `--`, and an enum value with no
table entry (out-of-range or unrecognised values) falls back to `#N`.

The first time a display component attaches with an all-empty page table, it
seeds pages 0-3 as volume (LARGE), preset, input source and sample rate, and
fills in any unset timing defaults (overlay hold 2 s, dwell 3 s, edit timeout
10 s). Seeding marks the CS config dirty, so it is a live preview like any
other edit until `REQ_CS_SAVE`; every seeded page is editable or clearable
like any other page.

### 1.4 Modes and the event overlay

`mode`: 0 = HOME_FIXED (show `home_page`), 1 = CYCLE_SELECTED (rotate the
active pages at `dwell`), 2 = CYCLE_ALL (rotate the untargeted,
platform-available nouns at `dwell`, no page table needed; MACRO and the
three display nouns are skipped, and a rotation always renders in the small
font).

The overlay is orthogonal to mode and enabled by default: when a monitored
item changes, its page is shown for `overlay_hold`, then the home content
returns. An overlay always renders its value LARGE on graphic models.
`overlay_hold` = 0 disables the overlay entirely. Two triggers:

- **Any CS dispatch** (physical control, IR, macro step): the engine's op
  initiators call `cs_display_note_adjust()` with `{noun, target, index,
  grouped}`; the overlay shows the matching configured page, or a synthesized
  generic page when the adjusted item has none and `CS_DCFG_OVERLAY_ANY` is
  set. A knob always gives feedback, even for items the user never
  configured. MACRO, DISPLAY_PAGE, DISPLAY_EDIT and PAGE_VALUE never pop as
  themselves; a PAGE_VALUE edit pops the item it resolved to, because that
  dispatch goes through the ordinary op path.
- **Host-side changes**: a compare-poll every 128 ticks (~8 Hz) over the
  active pages' items catches Console or UAC volume changes, with a 0.05
  natural-unit threshold. Read-only items are excluded from poll-triggered
  pops (meters churn constantly) but remain valid home pages.

`CS_NOUN_DISPLAY_PAGE` (see s4) lets any button, encoder, or remote key flip
through the active pages manually; a manual selection dismisses the overlay
and restarts the dwell timer.

### 1.5 Editing from the display (DISPLAY_EDIT / PAGE_VALUE)

Two further nouns make the display a front-panel editor. `DISPLAY_EDIT`
(bool) arms editing of the shown page, from a button, remote key, or
momentary hold, with an inactivity timeout (`edit_timeout`) so the panel
never stays hot; an LED can indicate the armed state (`IND_EQUALS`) and the
display brackets the value line with `>` and `<` while armed, or `!` on both
sides when the shown item is read-only (s5.1). `PAGE_VALUE` (virtual) resolves the currently shown page's
`{noun, target, index}` at event time and applies STEP / INC / DEC / TOGGLE
through the normal dispatch path using the resolved noun's unit, step law,
and range: continuous items step, bools switch (a TOGGLE flips; an encoder
detent sets on for up and off for down), grouped pages fan out through the
group engine, read-only pages no-op.

`CsDisplayCfg` flag bit1 (`CS_DCFG_EDIT_GATED`) gates `PAGE_VALUE` behind
edit mode; while gated and edit is OFF, `PAGE_VALUE` steps navigate pages
instead and a `PAGE_VALUE` toggle does nothing (the edit button owns
arming). One encoder plus one button therefore forms the classic browse /
arm / adjust front panel with no additional machinery. ADJUST (pots) is
deliberately excluded: an absolute control against a changing target would
slam values on every page change. A `PAGE_VALUE` binding must carry zeroed
value/step/range fields (nothing static to validate them against); stepping
uses the resolved noun's defaults plus encoder acceleration, and the op and
group-session state reset whenever the resolved item changes (a per-binding
and per-IR-command resolved-item key).

Buttons, encoders and IR commands may bind `PAGE_VALUE`. **Macro steps may
not**: it is rejected at step validation exactly like `CS_NOUN_MACRO`,
because a stored sequence that edits whatever happens to be on screen is
non-deterministic.

### 1.6 Layers (forward compatibility)

The overlay keys off what was *dispatched*, not which physical control
moved, so future layered controls (one encoder, several nouns via a layer
switch) display correctly with no changes here. A layer-switch event would
simply become one more poppable page.

## 2. Wire reference

### 2.1 Display binding (existing 24-byte CsBinding)

`type` = CS_TYPE_DISPLAY (8), `gpio[0]` = SDA, `gpio[1]` = SCL, `index` =
model (1..8), `value` = I2C address (0, or 0x08-0x77) for the model default,
all other fields 0 (strict, like the IR container). Second display slot
rejected with `CS_STATUS_DISPLAY_IN_USE`.

### 2.2 `CsDisplayCfg` (12 bytes)

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | `mode` | 0 FIXED / 1 CYCLE_SELECTED / 2 CYCLE_ALL |
| 1 | 1 | `home_page` | page slot shown in FIXED mode |
| 2 | 2 | `dwell` | rotation period, 0.1 s units (min 10 in cycle modes) |
| 4 | 2 | `overlay_hold` | pop-up hold, 0.1 s units (0 = overlay disabled) |
| 6 | 1 | `brightness` | 0-255; OLED contrast, applied by the init script (0 = the driver default) |
| 7 | 1 | `flags` | bit0 = overlay includes unconfigured dispatches; bit1 = PAGE_VALUE gated behind edit mode (steps navigate while unarmed); bits 3:2 = label alignment, bits 5:4 = value alignment (0 left, 1 centre, 2 right; 3 reserved and rejected) |
| 8 | 2 | `edit_timeout` | edit-mode inactivity auto-disarm, 0.1 s units (0 = manual only) |
| 10 | 2 | `reserved` | write 0 |

Brightness reaches the panel through the init script, so a change takes
effect on the next attach or after an I2C fault re-init; character modules
ignore it (their backlight line is driven on unconditionally).

### 2.3 `CsDisplayPage` (4 bytes)

`{noun, target, index, flags}` as in s1.3. All-zero clears the slot.

### 2.4 Commands (0x27-0x2B)

| Cmd | Name | Dir | wValue | Payload / response |
|---|---|---|---|---|
| 0x27 | `REQ_SET_CS_DISPLAY_CFG` | SET | 0 | 12-byte `CsDisplayCfg` |
| 0x28 | `REQ_GET_CS_DISPLAY_CFG` | GET | 0 | 16 bytes: `{max_pages, model_count, reserved[2]}` + `CsDisplayCfg` |
| 0x29 | `REQ_SET_CS_DISPLAY_PAGE` | SET | page 0-15 | 4-byte `CsDisplayPage` |
| 0x2A | `REQ_GET_CS_DISPLAY_PAGE` | GET | page 0-15 | 4-byte `CsDisplayPage` |
| 0x2B | `REQ_GET_CS_DISPLAY_STATUS` | GET | 0 | 8-byte `CsDisplayStatus` |

`CsDisplayStatus`: `init_state` (0 down / 1 initializing / 2 live / 3
error-backoff), `current_page` (0xFF = none or a synthesized overlay),
`flags` (bit0 overlay showing, bit1 edit armed), `model`, `nak_count`
(uint16, saturating count of I2C aborts), `reserved[2]`.

All SETs are deferred single-deep handoffs in the established shape,
apply-live-only previews under the shared dirty flag; `REQ_CS_SAVE` /
`REQ_CS_REVERT` cover the display config and pages. `cs_last_slot` tags:
`0x50` bare for the config, `0x50 | page` for a page (0x50 = display
namespace, distinct from 0x40 groups and 0x60 macros). New status codes:
`CS_STATUS_DISPLAY_IN_USE` 0x22, `CS_STATUS_PIN_NOT_I2C` 0x23,
`CS_STATUS_I2C_IN_USE` 0x24, `CS_STATUS_INVALID_PAGE` 0x25.

**Caps discovery.** The fixed `CsCapsHeader` fields are unchanged and
`caps_version` becomes 10, but adding `CS_TYPE_DISPLAY` grows the caps
**type table** by one 4-byte `CsTypeDesc` row, so the `REQ_GET_CS_CAPS`
header response goes from 40 to 44 bytes. This is the self-describing
mechanism documented since caps v3: hosts read `type_count` and locate the
post-table fields (`max_ir_commands`, `max_groups`, `max_macros`,
`max_macro_steps`) at offset `4 + 4 * type_count`, never at a hard-coded
offset. Every other structure (`CsBinding`, `IrCommand`, `CsStatusPacket`,
`CsExtStatusPacket`, `CsNounDesc`) is byte-identical to caps v9. This is the
one pre-existing GET whose length changes at caps v10; external clients that
do exact-length readback (see the ESP32 front-panel client) must be updated
before shipping this firmware to them. Display limits are served separately
by `REQ_GET_CS_DISPLAY_CFG`'s response header.

## 3. IR remote group support (bundled)

`IrCommand.flags` gains `CS_FLAG_GROUP` (0x20): `target` becomes a group
index under the same validation as bindings (kind compatibility, non-empty
group, per-member band checks). All button-subset actions compose,
including hold-to-repeat on grouped INC/DEC and grouped MOMENTARY
engage/restore, which closes the v9 gap where grouped volume from a remote
needed press-per-step macro firing. The 16-byte `IrCommand` record and the
IR config blob are unchanged; pre-v10 firmware rejects a grouped record at
validation (slot loads inactive), the same graceful downgrade as every prior
caps bump.

Runtime cost is bounded by physics: an IR receiver delivers one key at a
time, so the IR path gets a pool of **2** shared group contexts (one may be
pinned by a held grouped momentary while another key fires), not one per
command slot. Pool exhaustion (only reachable via a held momentary plus
interleaved grouped keys) drops the new op for that press. The tick pumps
BUSY members, ages sessions, and frees an idle context back to the pool;
receiver teardown restores any engaged grouped momentary first.

IR commands may also bind `PAGE_VALUE` (INC / DEC / TOGGLE), so a remote can
drive the browse/arm/adjust panel. Each command carries its own resolved-item
key, and a grouped resolved page borrows a context from the same pool.

## 4. New nouns

| Noun | Value | Kind | Notes |
|---|---|---|---|
| `CS_NOUN_CPU_LOAD` | 53 | continuous percent 0-100, read-only | `global_status.cpu0_load`, the same source as `REQ_GET_STATUS` wValue 9; bindable to IND_ABOVE / IND_LEVEL (PWM meter LED) and displayable |
| `CS_NOUN_DISPLAY_PAGE` | 54 | enum 0-15 | STEP/INC/DEC/SET (+WRAP) select the shown page; read = current page (0xFF when none); steps skip empty page slots exactly like PRESET skips empty presets |
| `CS_NOUN_DISPLAY_EDIT` | 55 | bool | arm editing of the shown page (TOGGLE/SET/MOMENTARY; IND_EQUALS for an armed LED); auto-disarms after `edit_timeout` |
| `CS_NOUN_PAGE_VALUE` | 56 | virtual | STEP/INC/DEC/TOGGLE applied to the shown page's item, resolved at event time (s1.5); value/step/range fields must be 0; no ADJUST; not usable as a macro step or as a page noun |

All three display nouns are no-ops (and read idle) without a live display
component, so they are valid to configure in advance.

## 5. Fonts and rendering (graphic OLEDs)

One font in flash, none in RAM: 5x8 (ASCII 32-126, 95 x 5 B = ~480 B) giving
21 columns at a 6 px pitch. There is no second font table: LARGE text is the
same font **pixel-doubled at render time** through a 16-entry nibble LUT,
producing 10x16 glyphs at a 10 px pitch, so 12 large columns fit across
128 px and the value occupies two page rows (`value_row` and `value_row + 1`).
Rendering is column-on-the-fly from the font into the transmit ring; there is
no pixel framebuffer and no staging bitmap. Character modules ignore fonts
entirely (glyphs in module ROM); no CGRAM glyphs are uploaded in this
revision.

### 5.1 Line layout and edit markers (caps v11)

Each logical line is laid out across its physical width after formatting and
before the change diff, so alignment costs one pass over a 21-byte buffer at
the render decimation and never adds a byte of I2C traffic (a dirty row
already rewrites every column). The label uses the model's column count; the
value uses 12 when LARGE on a graphic panel and the model's count otherwise.
Alignment comes from the two `CsDisplayCfg.flags` fields (s2.2) and applies
to every view, including the overlay, CYCLE_ALL, and the idle "DSPi" line.

While edit is armed the value line reserves its outer two columns for the
markers, `>` and its mirror `<` (or `!` on both sides for a read-only item),
and the value is centred, flushed or ranged within the span between them,
truncated to that span if it is too long. Reserving the columns
unconditionally is what keeps arming from moving the value: with centre
alignment the value lands on exactly the same columns armed and unarmed, for
any width and any string length. Left and right alignment necessarily shift
the value one column inward when armed, since the marker takes the edge the
value is anchored to.

Character modules draw the markers from module ROM and graphic panels from
the 5x8 table, so no CGRAM glyph upload is needed; a filled-triangle marker
would require one and is deliberately out of scope.

## 6. Driver architecture

One byte-budget state machine on the existing 1 kHz tick, per the
established rule that nothing on the main loop may block:

- Content is two logical text lines (label and value, 21 chars max) plus the
  last-transmitted copy of each; a render pass diffs them and sets a
  per-physical-row dirty bit. There is no full-screen shadow buffer.
- All bus traffic goes through a 64-entry uint16 TX ring whose words are the
  DW `data_cmd` encoding verbatim (low 8 bits data, bit 9 STOP), plus
  `0x8000 | ms` delay sentinels. Per tick the producer (init script, then
  dirty rows) fills the ring while space allows and the pump drains it into
  the I2C TX FIFO while the FIFO has room; the hardware transmits in the
  background. A dirty row lands in a few tens of ticks, well under
  perception.
- HD44780 long commands (clear/home) are delay sentinels in the ring, never
  busy-waits; the four expander writes that clock one HD44780 byte ride a
  single I2C transaction.
- Init is a per-model scripted sequence (HD44780 4-bit bring-up, US2066
  3.3 V sequence, SSD1306/SH1106 page-addressing setup) run in bounded
  bursts. A TX abort (NAK, arbitration loss) increments the saturating NAK
  counter, flushes the ring, parks the component in the ERROR state and
  re-runs init after ~1 s, so an unplugged or wedged module keeps retrying
  hot-plug style without disturbing the rest of the tick.
- Render runs on `s_render_now` or every 32 ticks; the host-change compare
  poll every 128 ticks.
- Flash blackouts: the I2C master finishes its FIFO on its own while the
  loop is stalled; the state machine resumes where it left off. Font reads
  are XIP but only ever happen from the tick, which does not run during a
  blackout.

The event hook is a call at each op initiator recording the last-adjusted
`{noun, target, index, grouped}`; the compare poll walks only active pages'
items with quantized comparison.

## 7. Electrical notes

PCF8574 backpacks are 5 V modules; neither MCU has 5 V-tolerant GPIOs.
Users need a proper level shifter (bus is open-drain, so a MOSFET shifter
board works) or a module verified at 3.3 V (contrast often suffers). The
character OLEDs and SSD1306/SH1106 boards are 3.3 V native and are the
recommended pairing. The driver enables the internal pull-ups, but they are
not sufficient for I2C at 400 kHz with module capacitance; external
2.2-4.7 k pull-ups to 3.3 V are assumed and documented, not enforced.

## 8. Persistence (directory V19)

`CsDisplayFlash` (version + reserved[3] + `CsDisplayCfg` 12 B +
`CsDisplayPage[16]` 64 B = 80 B) joins the directory as one versioned blob
appended after `cs_macros`; the V18 prefix stays byte-identical (frozen
`PresetDirectory_v18` snapshot plus `_Static_assert`s), so the V18->V19
migration is a prefix copy with the new blob zero-filled. A migrated device
shows nothing until configured: the display feature is idle and page seeding
(s1.3) happens only when a display binding is first applied, never at
migration. `dir_sanitize_cs_display()` resets the whole blob on an
implausible version or dirty reserved bytes, clamps `mode` and `home_page`,
and clears any page carrying unknown flag bits; noun validity is
platform-dependent and checked at apply/render time. The display binding
itself lives in the ordinary binding table as today, and `preset_set_cs_all`
persists the blob in the same single directory write as bindings, names, IR
commands, groups and macros.

## 9. Budget

| Resource | Cost |
|---|---|
| RAM | ~750 B BSS on both platforms: ~370 B in the display module (80 B live blob, 128 B TX ring, 2 x 22 B last-transmitted line buffers, 64 B poll cache, driver state) plus the engine-side PAGE_VALUE keys (16 binding + 16 IR) and the 2-entry grouped-IR context pool |
| Flash | ~12 KB: per-model drivers and init scripts, the ~480 B font table, the noun label table, plus the `dir_cache` mirror of the 80 B blob |
| Directory | 80 B, taking the directory to 3035 of its 4096-byte sector at V19 |
| Main loop | a few microseconds per tick while redrawing; zero when idle |

## 10. Validation summary

Display binding: model 1..8, address 0 or 0x08-0x77, SDA even / SCL odd on
the same free I2C instance, one display per device, all other binding fields
0. Config: mode 0-2, `home_page` < 16, `dwell` >= 10 when a cycle mode is
set, no unknown flag bits, reserved bytes 0.
Page: ACTIVE implies the noun exists on this platform (non-empty action
mask); DISPLAY_PAGE, DISPLAY_EDIT and PAGE_VALUE are rejected as page nouns
(`CS_STATUS_INVALID_PAGE`); a non-ACTIVE record must be all-zero;
target/index/group rules identical to bindings, and GROUP pages follow group
edits exactly as grouped bindings do (a page whose group empties renders
`--`). PAGE_VALUE bindings: actions STEP/INC/DEC/TOGGLE only, untargeted
(target/index 0), value/step/range 0; PAGE_VALUE IR commands additionally
require value/step 0; PAGE_VALUE is rejected in macro steps. DISPLAY_EDIT
and PAGE_VALUE both require a live display component to act (no-op without
one, valid to configure in advance, matching the IR-commands-before-receiver
rule).
