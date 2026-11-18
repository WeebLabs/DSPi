# Pin Availability Query Specification

*Last updated: 2026-11-18 (implemented; section 17 records where the code differs from or settles the design). Status: implemented, HW-untested.*

This document specifies how a host asks the DSPi which GPIOs a function may
use, and how the firmware guarantees that the answer matches what it will
later accept. It covers the firmware refactor that makes this possible, the
new vendor commands, the notification, the rules for bulk and preset
restores, and the host integration. It complements
`Documentation/Features/clock_pins_spec.md`,
`Documentation/Features/control_interfaces_spec.md`,
`Documentation/Features/control_surfaces_spec.md` and
`Documentation/Features/vendor_protocol_migration_guide.md`.

---

## 1. Summary

The firmware refuses a pin assignment that collides with another function.
The DSPi Console keeps its own copy of those rules so that it can grey out
pins before the user picks one. The two copies drift. A pin the Console
offers can be refused by the device, and a pin the device would accept can
be hidden by the Console.

The fix is that the Console stops computing availability. It asks the
device. For each pin function the device returns bit masks of the GPIOs
that function could take, plus a short status that says whether the
function is blocked for a reason unrelated to any one pin. A second query
returns the list of current pin claims so the UI can say which function
holds a pin.

The answer is only trustworthy if the query and the real assignment run the
same code. Today they could not, because the rules are spread over a dozen
sites that disagree with each other in small ways. So the larger part of
this work is a refactor. Every path that decides whether a pin is
acceptable calls one set of per-function checks. That covers single SETs,
the deferred applies on the main loop, the bulk blob, preset load, boot
restore and the new query.

One rule governs all of it. A function holds its pins only while it is in
use. A function that is switched off keeps its pin as a stored preference
that blocks nothing. The conflict check runs when the function is switched
on, and a conflict there is a clean refusal.

A generation counter tells the host when its cached answers are stale. One
notification announces a change.

---

## 2. Problem

### 2.1 Drift between the Console and the firmware

Mirroring raw ownership to the host is not enough. Acceptance has
function-specific exceptions, and those are what drift. The exceptions in
the current firmware are listed below.

- ADAT RX may share the ADAT TX pin while TX is enabled (`vendor/vc_pins.c:175`).
- MCK ignores its own claim (`vendor/vc_pins.c:160`).
- The I2S clock pair is always claimed against control interface pins, CS pins and I2S RX data pins, but only while clocks run against everything else (`vendor/vc_pins.c:74`, `:100`, `:189`).
- SPLIT clock-pin mode reserves the dormant slave pair.
- A disabled optional S/PDIF input claims nothing.
- Several button bindings may share one GPIO.

This specification removes the third and fourth exceptions (section 4.6).

### 2.2 Drift inside the firmware

The rules have already drifted between firmware sites. These are the cases
found during the review.

- DAC mute keeps a private collision check that omits Control Surfaces pins, I2S RX data pins and the clock pair when I2S is the input source (`dac_hw_mute.c:159`). Its range check admits GPIO 23 to 25 and pins up to 47 on RP2350 (`dac_hw_mute.c:141`).
- The bulk blob applies output and S/PDIF RX pins with a range check and a control-interface check only. There is no cross-conflict check (`bulk_params.c:476`).
- The restore check for the BCK pair ignores running clocks and RX data pins. The SET checks both (`vendor/vc_pins.c:135`, `vendor/vc_page_clock.c:54`).
- The MCK SET requires a GPOUT-capable pin. The restore check does not (`vendor/vc_page_clock.c:162`, `vendor/vc_pins.c:160`).
- Conflict checks read the live input source, output types and clock mode. A queued change is invisible until the main loop applies it (`vendor/vc_pins.c:79`).
- Switching an output slot to I2S starts the clock pair with no pin check at all (`vendor/vc_page_output.c:30`).
- A switched-off function is treated three different ways. A disabled S/PDIF input stores any valid pin. A disabled MCK or ADAT port has its pin SET conflict-checked. PDM holds its pin whether it runs or not.

### 2.3 Why a simple mask of "what the SET accepts" is not enough

Roughly half the pin SET sites are not a synchronous single-pin accept or
reject. A design that models them that way gives wrong answers.

- Three SET families answer `VC_OK` and validate ownership later on the main loop. They are `XPORT.UART_CONFIG` and `XPORT.I2C_CONFIG` (result in `XPORT.STATUS`), `OUTPUT.DAC_MUTE_CONFIG` (read back to confirm) and the Control Surfaces preview SETs (result in `CS.STATUS`).
- Some functions take two pins that are not adjacent. UART needs TX on a GPIO with `pin % 4 == 0` and RX on one with `pin % 4 == 1`, both on the same UART instance (`uart_control.c:546`). I2C and the CS display need SDA even and SCL odd on the same I2C instance.
- A switched-off function accepts a pin that it could not use if it were switched on now.
- Some functions refuse every pin for a state reason, such as MCK while it is enabled.

---

## 3. Design overview

| Part | What it is |
|---|---|
| Pin state snapshot | One struct that holds every value a pin decision depends on |
| Per-function checks | One check per pin function, run against a snapshot |
| Start checks | The conflict check run at every point where a function begins to use its pins |
| Unit validators | Whole-configuration validation for bulk, preset load and boot, built from the same checks |
| Availability GETs | One read-only command beside each pin SET, returning masks and a status |
| Claim list | One read-only command listing every pin claim, for display |
| Generation counter | A number in every reply that changes when any answer could have changed |
| `PIN_MAP_CHANGED` notification | One event, raised from a single main-loop point |

---

## 4. Firmware: one set of checks

### 4.1 Pin functions

Each pin function gets a wire-stable identifier. The existing
`DSPI_PIN_FN_*` enum cannot serve, because its values are implicit, it is
only compiled into platform builds and it has no Control Surfaces entry
(`platform_pins.h:12`). The new enum lives in `vendor/vc_pins.h`, has
explicit values and is append only.

| ID | Name | Index | Roles |
|---|---|---|---|
| 1 | `PIN_FN_OUTPUT` | output slot | data |
| 2 | `PIN_FN_PDM` | 0 | data |
| 3 | `PIN_FN_I2S_CLK` | 0 master or unified, 1 slave | BCK, LRCLK (BCK + 1) |
| 4 | `PIN_FN_MCK` | 0 | clock |
| 5 | `PIN_FN_ADAT_TX` | 0 | data |
| 6 | `PIN_FN_ADAT_RX` | 0 | data |
| 7 | `PIN_FN_SPDIF_RX` | input index | data |
| 8 | `PIN_FN_I2S_RX` | stereo pair | data |
| 9 | `PIN_FN_DAC_MUTE` | 0 | mute |
| 10 | `PIN_FN_UART_CTRL` | 0 | TX, RX |
| 11 | `PIN_FN_I2C_CTRL` | 0 | SDA, SCL |
| 12 | `PIN_FN_CS` | binding slot | per binding type |

A platform build maps these to its `DSPI_PIN_FN_*` hooks in one place.

### 4.2 The snapshot

`PinState` holds every input to a pin decision. That means the pins of
every function, each function's enable state, the output types, the input
source, the I2S RX pair count, the clock mode, the clock-pin mode and the
Control Surfaces bindings.

A snapshot is built in one of two ways.

- `pin_state_capture()` reads the running device. It is used by SET handlers, deferred applies and the query.
- A restore path builds a snapshot from the configuration it is about to apply (section 7).

The checks never read globals. They read a snapshot. That is what lets one
check serve both a live SET and a proposed configuration.

### 4.3 Effective state is the union of live and pending

Several settings that decide claims are applied later by the main loop.
They are the output type, the input source, the I2S clock mode, the UART
and I2C configurations, the DAC mute configuration and the Control Surfaces
preview.

`pin_state_capture()` records a claim if it holds in the live state or in
the pending state. Pending does not replace live. With a switch away from
I2S queued, the clock pair is still driven until the apply runs, and an
immediate SET must not land on it. The union is also safe when the apply
later drops the pending change.

The union also closes the gap after an accepted start. Once a handler has
queued a start, the function's pins count as claimed, so no later SET can
take them before the apply runs.

### 4.4 The check

```c
uint8_t pin_check(const PinState *st, const PinRequest *req, PinCheckMode mode);
```

`PinRequest` names the function, its index and all of its pins as one unit.
For a Control Surfaces binding it also carries the binding record. The
return value is the status byte the firmware already uses, where
`PIN_CONFIG_*` occupies 0x00 to 0x05 and `CS_STATUS_*` starts at 0x10.

The check has two layers, applied in this order.

1. Pin validity for the function. This holds the platform range, the GPOUT rule for MCK, and the mux and pairing rules for UART and I2C.
2. Conflicts against every active claim in the snapshot, with the function's sharing exceptions.

`mode` selects how far the check goes. `PIN_CHECK_STORE` stops after layer
1. `PIN_CHECK_CLAIM` runs both.

Function state is a separate call, `pin_fn_status()`, because it reads live
hardware state a proposed restore config does not have. It returns
`PIN_CONFIG_OUTPUT_ACTIVE` when the function refuses every pin for now. A SET
handler calls it first through `pin_check_set()`. Control Surfaces keeps its
type rules (ADC class, PWM slices, the display pair, button sharing) in one
helper of its own, `cs_pin_status()`, which calls `pin_conflicts()` for every
function outside its family.

Each surviving rule in section 2.1 becomes a clause of the owning
function's check and exists nowhere else.

### 4.5 Self-inclusion

A function's own claim never blocks it. The check ignores claims owned by
the same function and index. This replaces four ad hoc mechanisms.

- The output SET's `exclude` index (`vendor/vc_page_output.c:57`).
- DAC mute's private collision helper.
- The deferral of control-interface ownership checks, which exists because the live interface's own pins look taken (`vendor/vc_page_xport.c:25`).
- The early return for an unchanged pin in most handlers.

One consequence is that moving the BCK pair by one pin becomes legal. It
is refused today because the new BCK lands on the pair's own LRCLK.

### 4.6 A function claims its pins only while in use

This is the central rule. A function that is not in use holds no pin. Its
pin is a stored preference. A stored pin never blocks another function,
and it never blocks a sibling pin of the same function family.

The table says when each function is in use. "Live or pending" follows
section 4.3.

| Function | In use when |
|---|---|
| Output slot | Always. A slot has no off state |
| PDM | PDM is enabled or running |
| I2S clock pair | Any output slot is I2S, or I2S is the input source. Only the pair the clock mode selects is in use |
| MCK | Enabled |
| ADAT TX, ADAT RX | Enabled |
| S/PDIF input 1 | Always. See below |
| S/PDIF inputs 2 to 4 | Enabled |
| I2S RX pair | I2S is the input source and the pair is within the channel count |
| DAC mute | Enabled |
| UART and I2C control | Enabled and live |
| CS binding | The binding is active |

Three reservations in the current firmware are removed by this rule.

- The I2S clock pair is no longer held while no clock runs. That includes the special rule that held it against control interface pins, CS pins and I2S RX data pins.
- The dormant pair in SPLIT clock-pin mode is no longer held.
- PDM no longer holds its pin while it is off.

I2S RX data pins are also released while I2S is not the input source. The
I2S input cannot run without its clock pair. With the clock pair released,
holding the data pin alone would guarantee nothing.

**S/PDIF input 1 keeps its claim.** The firmware defines input 1 as always
enabled, with no off state. It is treated like an enabled optional input
that is not the selected source, which also holds its pin. Input 1 does not
get an enable flag in this work. That decision may be revisited later, and
an enable flag would then bring input 1 under the same rule as inputs 2 to 4.

#### Pin SET on a function that is not in use

The SET runs `PIN_CHECK_STORE`. It refuses a GPIO the function could never
use and accepts a GPIO another function currently holds.

#### Start checks

Every point where a function begins to use its pins runs
`PIN_CHECK_CLAIM` on the pins it is about to take. A conflict is refused
with `VC_ERR_PIN_IN_USE` in the handler. Nothing is queued, nothing is
restarted and the function stays off.

| Start site | Pins checked |
|---|---|
| S/PDIF input enable | That input's pin |
| MCK enable, ADAT TX enable, ADAT RX enable | The function's pin |
| DAC mute, UART and I2C configuration with enable set | The function's pins, at the deferred apply |
| CS binding apply | The binding's pins |
| PDM start | The PDM pin |
| `OUTPUT.TYPE` to I2S, when no clock runs yet | The clock pair the clock mode selects |
| `INPUT.SOURCE` to I2S | The clock pair if no clock runs yet, plus the data pins of every pair within the count |
| `INPUT.I2S_RX_CHANNELS` rise while I2S is the source | The data pins of the added pairs |
| `CLOCK.I2S_CLOCK_MODE` or `CLOCK.I2S_CLOCK_PIN_MODE` change that moves running clocks to the other pair | The pair being adopted |

The I2S start is checked as one unit. The clock pair and the data pins
must be free of other claims and must not collide with each other.

A start that comes from somewhere other than a host command runs the same
check. That covers a Control Surfaces binding, an IR command or a macro
that switches the input source or an output type. A refusal there is a
no-op.

#### Pin SET on a function that is in use

The SET runs `PIN_CHECK_CLAIM`. A BCK pin SET for the pair that is not
currently selected is a SET on a function that is not in use.

### 4.7 Callers

| Caller | When | Snapshot | Mode |
|---|---|---|---|
| Pin SET, function in use | In the handler | Captured | Claim |
| Pin SET, function not in use | In the handler | Captured | Store |
| Start sites (4.6) | In the handler | Captured | Claim |
| Deferred applies (UART, I2C, DAC mute, CS) | On the main loop | Captured at apply time | Claim if enabling, else Store |
| Availability GETs | In the handler | Captured | Both |
| `STORE.ALL_PARAMS` SET, preset load, factory reset | On the main loop | Proposed (section 7) | Per function |
| Boot restore | Before the outputs start | Proposed (section 7) | Per function |

The deferred SET handlers keep their current split. They refuse scalar
errors and never-usable GPIOs at once and leave ownership to the apply.

### 4.8 Slot alignment

This work adds no restart path and removes none. A pin move still goes
through `process_pin_changes()` and the synchronized output restart.

Every new refusal in section 4.6 happens before any state changes. A
refused start leaves every output slot untouched. An accepted start takes
the same restart path it takes today.

---

## 5. Availability masks

### 5.1 Two masks

Each pin role returns two masks.

- **`valid`.** The GPIOs the function could ever use in this role. It holds the range, mux, GPOUT, ADC and PWM rules. It does not depend on what other functions hold.
- **`free`.** The subset of `valid` that no other function claims right now, after the function's sharing exceptions.

Both are produced by running `pin_check()` over every GPIO the platform
has, once in each mode. Neither is computed by separate code.

`free` always includes the function's current pin when the function is in
use, by section 4.5.

### 5.2 What the host can rely on

Flag bit 0 of the reply says whether the function is in use.

| Function state | A pin SET accepts | The function can start when |
|---|---|---|
| In use | Pins in `free` | n/a |
| Not in use | Pins in `valid` | Its stored pins are all in `free` (flag bit 3) |

A UI can therefore show three kinds of pin for a switched-off function.
They are pins that are free, pins that can be stored but are held by
something else, and pins the function can never use.

One limit applies to the I2S family. Flag bit 3 on an I2S reply covers
that reply's own pins. Whether the clock pair and the data pins also
avoid each other is decided by the start check, and the claim list shows
the stored pins of each.

### 5.3 Function status

Every reply carries a status byte for the function as a whole. A zero mask
cannot tell "disable it first" from "nothing is free", so the status says
which.

| Status | Meaning |
|---|---|
| `0x00` | A pin SET can proceed |
| `VC_ERR_STATE` (`0x18`) | The SET would be refused whatever pin is sent. Examples are MCK while enabled, PDM while active and the BCK pair that I2S outputs are running on |
| `VC_ERR_BUSY` (`0x17`) | The answer cannot be computed now. Retry |

When the status is `VC_ERR_STATE`, the masks are still filled in. They
show what would be acceptable once the block is cleared.

Control Surfaces replies use `CS_STATUS_*` in this byte (section 6).

### 5.4 Roles and groups

A multi-pin function returns its masks per pin role.

Some roles must come from the same hardware instance. The reply expresses
that with groups. A pair is acceptable when both pins are set in their
role masks within the same group. The host needs no mux knowledge.

| Function | Groups | Roles | Pairing |
|---|---|---|---|
| Single-pin functions | 1 | 1 | none |
| I2S clock pair | 1 | 1 (BCK) | LRCLK is BCK + 1. A set bit means both pins pass |
| UART control | 2 (UART0, UART1) | 2 (TX, RX) | same group |
| I2C control | 2 (I2C0, I2C1) | 2 (SDA, SCL) | same group |
| CS display | 2 (I2C0, I2C1) | 2 (SDA, SCL) | same group |
| CS encoder | 1 | 2 (A, B) | pins must differ |

The I2C control interface and the CS display cannot use the same I2C
instance, even on different pins (`i2c_control.c:377`,
`control_surfaces.c:1662`). The instance the other one holds has all-zero
`free` masks. Its `valid` masks are unchanged.

### 5.5 Mask width

Every mask is a little-endian u64, with bit n for GPIO n. Thirty-two bits
would be enough for the Pico boards but not for RP2350B packages or
platform builds. `SYS.PIN_STATE` reports the GPIO count and the mask of
pins that exist.

---

## 6. Control Surfaces

Control Surfaces pins are where the Console most needs this, so they are
part of the same work.

### 6.1 Addressing

Acceptance depends on the binding type, so the query is addressed by slot
and type. All roles come back in one reply.

A new address kind `VC_A_CS_SLOT_TYPE` is appended to `VcAddrKind`. Its
`wValue` is `(type << 8) | slot`. A type of `CS_TYPE_NONE` or one the
build does not have is `VC_ERR_ADDRESS`.

### 6.2 Three masks

The reply carries the `valid` and `free` masks of section 5 and one more.

- **`valid`.** The pin fits the type. That covers ADC pins for a pot and the PWM slice the soft VCXO paces on.
- **`free`.** The pin is unclaimed for this type in this slot. A PWM slice and channel another binding uses is excluded here.
- **Button-shareable.** The pin is held only by other button bindings. A button binding may join them if its event and invert sense fit. The final answer comes from the apply, which can still return `CS_STATUS_EVENT_IN_USE`.

A pin is never set in both `free` and the shareable mask. The shareable
mask is zero for every type other than `CS_TYPE_BUTTON`.

### 6.3 Status

The status byte is in `CS_STATUS_*` space.

| Status | Meaning |
|---|---|
| `0x00` | A binding of this type can be staged in this slot |
| `CS_STATUS_IR_IN_USE` | Another slot holds the IR receiver |
| `CS_STATUS_DISPLAY_IN_USE` | Another slot holds the display |
| `CS_STATUS_BUSY` | A revert is running. Retry after `CS.STATUS` leaves pending |

### 6.4 Which bindings count

Only active bindings claim pins. A binding that failed to come up holds
nothing.

The check reads the same view the apply reads. That is the active
bindings, plus the stored bindings a running revert is about to reclaim
(`control_surfaces.c:2338`). The query answers `CS_STATUS_BUSY` during a
revert, as the preview SETs do.

---

## 7. Bulk, preset and boot restore

### 7.1 One validator

`STORE.ALL_PARAMS` SET, preset load, factory reset and boot restore all
apply many pin fields at once. They call one unit validator that takes a
proposed configuration and returns a corrected one. The validator calls
`pin_check()` for each function. No restore path keeps a pin rule of its
own.

The existing unit checks (`i2s_rx_pin_set_acceptable()`,
`i2s_bck_pin_acceptable()`, `i2s_mck_pin_acceptable()`,
`adat_pin_acceptable()`, `adat_input_pin_acceptable()`,
`spdif_input_enable_acceptable()` and `io_pin_valid()`) are removed.

### 7.2 Functions that are off in the proposed configuration

A function that the proposed configuration leaves off is checked for
validity only. It takes no part in conflict checks, it blocks nothing and
a conflict never changes it.

A pin that fails validity can never be used, so it is replaced by the
platform default.

### 7.3 Fallback rule for functions that are in use

A bad pin does not reject the whole blob. The validator processes the
functions that are in use in a fixed priority order. Each is checked
against the claims already accepted for higher-priority functions.

What happens on a failure depends on whether the function has an enable
flag.

**A function with an enable flag.** The proposed pin is kept as the stored
preference and the function is forced disabled. The validator does not
move it to another pin, since that would drive a pin the user did not
choose. A pin that is invalid rather than taken can never work, so it is
replaced by the default, and the function stays on if the default claims.
An ADAT input enabled with no pin is switched off.
This applies to PDM (its enable is the matrix output enable), MCK, ADAT TX,
ADAT RX, the optional S/PDIF inputs and DAC mute. The UART and I2C control
interfaces are fixed claims at run time and are judged after the audio pins
at boot.

**A function with no off state.** The validator tries these in order, and
checks each candidate.

1. The platform default for that function.
2. The base value. At run time that is the live value. At boot it is the factory default.

This applies to output slots, S/PDIF input 1, and the I2S clock pair and I2S
RX pairs when the proposed outputs use I2S. When only a proposed switch of
the input to I2S needs them, the switch is dropped instead.

If neither candidate passes, the whole pin section is abandoned and the
base configuration is kept. A boot keeps the stored input source even then,
since the factory pin set serves any source. At run time the base is the live
configuration, which is consistent. At boot it is the factory default
set, which is consistent by construction and is covered by a build-time
test.

Because each function is checked only against higher-priority claims, two
functions that exchange pins in one blob both pass.

### 7.4 Priority order

Only functions that are in use take part. Claims that the blob does not
carry are fixed and come first. The rest follow the order the restore
code uses today.

1. Live UART and I2C control interface pins (run time only).
2. Active Control Surfaces bindings and the display.
3. Output slots by index, then PDM.
4. S/PDIF input 1.
5. MCK.
6. The I2S clock pair in use.
7. Optional S/PDIF inputs by index.
8. I2S RX pairs by index.
9. DAC mute.
10. ADAT TX, then ADAT RX.

Rule 1 keeps the existing self-lockout guard. A pushed configuration never
takes the pins of a control link, since over UART or I2C that would cut
the link doing the push.

At boot the order differs in one way. The control interfaces and Control
Surfaces are not up yet, so stored audio pins are validated first. The
control interface configurations and the CS bindings are then validated
against them. A colliding control configuration stays down and shows in
`XPORT.STATUS`, as it does today.

### 7.5 Telling the host

A bulk apply or preset load already ends in `BULK_INVALIDATED`. The host
reads the pin fields back after it. `SYS.PIN_STATE` also carries a
substitution mask with one bit per pin function ID. A set bit means the
last restore replaced that function's pins or forced it disabled.

---

## 8. Vendor commands

All commands are read-only GETs with flags 0. They work on USB, UART and
I2C. Each availability command sits on the page of the SET it mirrors and
uses that SET's address kind, so `SYS.PAGE_COMMANDS` reports support and
platform conditions with no extra capability bit.

The opcodes below are free in the current `vendor/pages/*.def` tables.

### 8.1 Command list

| Command | ID | Address | GET length | Build |
|---|---|---|---|---|
| `OUTPUT.PIN_AVAIL` | `0x0412` | `PIN_OUTPUT` | 24 | all |
| `OUTPUT.ADAT_TX_PIN_AVAIL` | `0x0433` | none | 24 | `PICO_RP2350` |
| `OUTPUT.DAC_MUTE_PIN_AVAIL` | `0x0442` | none | 24 | all |
| `INPUT.SPDIF_RX_PIN_AVAIL` | `0x0325` | `SPDIF_RX` | 24 | all |
| `INPUT.I2S_RX_PIN_AVAIL` | `0x0333` | `I2S_PAIR` | 24 | all |
| `INPUT.ADAT_RX_PIN_AVAIL` | `0x0344` | none | 24 | `PICO_RP2350` |
| `CLOCK.I2S_BCK_PIN_AVAIL` | `0x0C14` | `ROLE` | 24 | all |
| `CLOCK.MCK_PIN_AVAIL` | `0x0C23` | none | 24 | all |
| `XPORT.UART_PIN_AVAIL` | `0x0B13` | none | 72 | all |
| `XPORT.I2C_PIN_AVAIL` | `0x0B14` | none | 72 | all |
| `CS.BINDING_PIN_AVAIL` | `0x0A22` | `CS_SLOT_TYPE` | 80 | all |
| `SYS.PIN_STATE` | `0x0150` | none | 24 | all |
| `SYS.PIN_CLAIMS` | `0x0151` | none | variable | all |

### 8.2 Availability reply

Every availability reply starts with the same 8-byte header.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | Function status (5.3, or 6.3 for CS) |
| 1 | u8 | Flags |
| 2 | u8 | Group count G |
| 3 | u8 | Role count R |
| 4 | u32 | Generation (section 9) |
| 8 | 2 x u64 x G x R | Mask pairs, group-major. Each pair is `valid` then `free` |

The order is group 0 role 0, group 0 role 1, group 1 role 0, and so on.

| Flag bit | Meaning |
|---|---|
| 0 | The function is in use (4.6), live or pending |
| 1 | The function has an implied second pin at role 0 + 1 |
| 2 | The function has no pin assigned |
| 3 | The function's current pins are all in `free`. A start would not be refused on pins |

The 24-byte replies have G = 1 and R = 1. `CLOCK.I2S_BCK_PIN_AVAIL` sets
flag bit 1. The two `XPORT` replies have G = 2 and R = 2.

`CS.BINDING_PIN_AVAIL` always returns 80 bytes. Bytes 8 to 71 hold four
mask pairs laid out for G = 2 and R = 2, with unused masks zero. The
header gives the real G and R for the type. Bytes 72 to 79 hold the
button-shareable mask.

### 8.3 `SYS.PIN_STATE`

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | Generation |
| 4 | u8 | GPIO count of the platform |
| 5 | u8 | Claim count `SYS.PIN_CLAIMS` would return |
| 6 | u16 | Substitution mask of the last restore, bit n for pin function ID n (7.5) |
| 8 | u64 | GPIOs that exist and are assignable to at least one function |
| 16 | u64 | Reserved, 0 |

A host without notifications polls this command and compares the
generation.

### 8.4 `SYS.PIN_CLAIMS`

The reply is an 8-byte header followed by one 4-byte entry per claim.

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | Generation |
| 4 | u8 | Entry count N |
| 5 | u8[3] | Reserved, 0 |
| 8 | 4 x N | Entries |

| Entry byte | Field |
|---|---|
| 0 | GPIO |
| 1 | Pin function ID (4.1) |
| 2 | Index within the function |
| 3 | Bits 1:0 claim state, bits 3:2 role, bits 7:4 CS binding type (0 for other functions) |

| Claim state | Value | Meaning |
|---|---|---|
| Active | 0 | The function is in use, live or pending, and holds the pin |
| Stored | 1 | The stored pin of a function that is not in use. It blocks nothing |

A GPIO can appear more than once. Buttons that share a pin, ADAT RX on
the ADAT TX pin, and any number of stored pins are simply separate
entries. Entries are sorted by GPIO.

The list is for display only. A host must never decide availability from
it.

**Capacity.** `VC_REPLY_MAX` is 256 bytes, which allows 62 entries. A
static assert derives the worst case from the build constants.

```
NUM_PIN_OUTPUTS + 2 (ADAT, RP2350 only) + SPDIF_RX_NUM_INPUTS
  + I2S_RX_MAX_PAIRS + 4 (two clock pairs) + 1 (MCK) + 1 (DAC mute)
  + 2 (UART) + 2 (I2C) + 2 * CS_MAX_BINDINGS
```

That is 57 on RP2350 and 50 on RP2040. The margin on RP2350 is five
entries. A new multi-pin function has to add paging, and the assert makes
that impossible to miss.

While a Control Surfaces revert runs, the bindings it is about to reclaim are
listed too, and the list can outgrow the reply. The command then answers
`VC_ERR_BUSY` until the revert completes.

### 8.5 Registry

The commands are added to the page tables, so
`Documentation/protocol/dspi_commands.json` and the generated host
constants pick them up. `Documentation/commands.md` and
`Documentation/current_architecture.md` are updated when the firmware
lands.

---

## 9. Generation counter and notification

### 9.1 The counter

The generation is a u32 that starts at 0 on every boot. It increases when
any availability answer or the claim list could have changed.

The firmware does not try to bump it at each site that changes a claim.
There are too many, and a missed site would be silent. One function,
`pin_generation_sync()`, captures the snapshot, hashes it together with the
live status of PDM, MCK and both clock pairs, and compares the hash with the
last one. A difference bumps the counter.

`pin_generation_sync()` is called from two places.

- One point on the main loop, at most every 50 ms. This is the only place that pushes the notification, including for a change a GET saw first.
- Each availability and claim GET, before it answers. The generation in a reply therefore always describes the state the reply was computed from.

This catches every cause with no per-site code. That includes pin SETs,
enable and mode changes, queued changes, deferred applies, preset load,
bulk apply, factory reset and a Control Surfaces revert.

### 9.2 The notification

| Field | Value |
|---|---|
| Event | `NOTIFY_EVT_PIN_MAP_CHANGED` = `0x11` |
| Payload | u32 generation |

`0x11` is the next free event. `0x06` and `0x0E` are reserved.

The event is coalesced. At most one unsent `PIN_MAP_CHANGED` entry exists,
and it carries the newest generation.

It is sent only on transports that have notifications enabled. That is
the USB notification endpoint, and UART when its `notify_enable` is set.

The implementation needs both halves. It needs the push, and it needs a
`notify_peek_next_for` case. Without the second the formatter drops the
event and still consumes a sequence number.

### 9.3 Host rules

- Query on every connect. The counter restarts at boot, so a remembered generation means nothing.
- On `PIN_MAP_CHANGED`, or when a reply carries a generation the host has not seen, discard cached masks and query again.
- After a deferred SET, read the setting back as today, then query again.

---

## 10. RP2040 and RP2350

| Item | RP2040 | RP2350 |
|---|---|---|
| Assignable GPIOs | 0 to 22, 26 to 28 | 0 to 22, 26 to 29 |
| Pin outputs | 3 | 5 |
| ADAT commands | absent | present |
| I2S RX pairs | 1 | 4 |
| Worst-case claims | 49 | 57 |
| Soft VCXO PWM slice | excluded from CS PWM masks | no bondable GPIO maps to it |

A command a build lacks is absent from `SYS.PAGE_COMMANDS`. A platform
build (`DSPI_PLATFORM_PINS`) supplies its own validity and pairing rules
through the existing hooks, and the masks reflect them with no host
change.

---

## 11. Behaviour changes

These are visible to hosts and belong in the release notes.

**Pins that are released.**

1. The I2S clock pair is not held while no clock runs. A control interface pin, a CS pin or any other function can take it.
2. The dormant pair in SPLIT clock-pin mode is not held. Entering SPLIT no longer requires the slave pair to be unclaimed.
3. I2S RX data pins are not held while I2S is not the input source.
4. The PDM pin is not held, driven or muxed while PDM is off. Its PIO output direction is set only while it runs, since PDM shares PIO1 with receivers.
5. A pin SET on a disabled MCK, ADAT TX or ADAT RX, or on an I2S RX pair beyond the channel count, accepts a pin another function holds. Today those SETs are conflict-checked.

**New refusals.**

6. `OUTPUT.TYPE` to I2S, `INPUT.SOURCE` to I2S, `CLOCK.I2S_CLOCK_MODE`, `CLOCK.I2S_CLOCK_PIN_MODE` and the PDM start can answer `VC_ERR_PIN_IN_USE` when the pins they need are taken. `OUTPUT.TYPE` has no pin check today.
7. DAC mute, when enabled, is refused on a Control Surfaces pin, an I2S RX data pin in use, a clock pair in use, GPIO 23 to 25 and any GPIO above the platform limit. A stored configuration that passes today can be refused after the update, and the function then stays disabled.
8. A pin SET sees queued changes. A pin on the clock pair is refused as soon as a switch to I2S is queued, not only once it has been applied.
9. A disabled UART or I2C configuration must satisfy the mux and pairing rules. Today only the GPIO range is checked while disabled.

**Restore.**

10. The bulk blob and preset load no longer accept pins that collide with each other among functions that are in use. Colliding fields are handled by the fallback rule.
11. The fallback rule is uniform. Today a bad output pin falls back to the default and a bad S/PDIF RX pin keeps the live pin. Both now try the default first and the live pin second.
12. The restore path refuses an MCK pin that has no GPOUT function.

**Other.**

13. The BCK pair can move by one pin while its own LRCLK is the only conflict.
14. The ADAT transmitter may now move onto the pin an enabled ADAT receiver listens on, as the receiver could already move onto the transmitter's.
15. `XPORT.UART_CONFIG` and `XPORT.I2C_CONFIG` refuse a pin pair that breaks the mux rules with `VC_ERR_PIN_INVALID` at once, instead of through `XPORT.STATUS` after the apply.
16. A DAC mute config with an invalid pin is refused even while disabled.
18. A stored, disabled UART or I2C config whose pins break the mux rules now shows `PIN_CONFIG_INVALID_PIN` in `XPORT.STATUS` after boot. It stays down, as before.
17. A re-sent unchanged pin is accepted as before, including while PDM runs or I2S outputs use the BCK pair.

The optional S/PDIF inputs do not change. A disabled input already stores
any valid pin and is checked at enable.

Existing GET replies do not change. Strict clients that check reply
lengths are unaffected.

---

## 12. Host integration

### 12.1 DSPi Console

The Console reads `SYS.PAGE_COMMANDS` at connect, as the migration guide
already requires. When the availability command for a function is present,
the Console uses the device's masks and does not run its own rules. When
it is absent, the Console falls back to its current logic. The fallback is
removed when support for that firmware is dropped.

The Console shows the function status as the reason a control is disabled,
and uses `SYS.PIN_CLAIMS` to label taken pins.

For a function that is off, the Console offers the pins in `valid` and
marks those outside `free` as held by another function. It uses flag bit 3
to warn that the function cannot be switched on until its pin is freed.

The Console must handle `VC_ERR_PIN_IN_USE` from the start sites in
section 4.6, including `OUTPUT.TYPE` and `INPUT.SOURCE`.

### 12.2 Swapping pins

Two functions that are both in use cannot exchange pins with two plain
SETs, because each sees the other's pin as taken. The firmware does not
add a swap command. The host has two ways to do it.

- Move one function to a free pin, move the other, then move the first back.
- Send one bulk blob. The unit validator checks the end state as a whole (7.3).

### 12.3 UART and I2C clients

The commands are additive. A client that does not use them needs no
change. A client that parses notifications strictly must tolerate event
`0x11`, which is only sent when the client has asked for notifications.

A client that switches the input source or an output type must expect the
new refusals in section 11 item 6.

---

## 13. Testing

### 13.1 Consistency tests in `dspi_test` (`paged_pins`)

The tests compare the masks with real outcomes and restore what they touch.

- **Shapes.** `SYS.PIN_STATE` and `SYS.PIN_CLAIMS` agree, entries are sorted and well formed, every availability reply has its documented length, groups and roles, `free` lies inside `valid`, and `valid` inside the GPIO mask. A CS button's free and shareable masks never overlap.
- **Free masks avoid active claims.** No free bit names a GPIO the claim list shows another function holding, the ADAT loopback share excepted.
- **Masks match real SETs.** For every one-pin function, a function not in use must accept exactly its `valid` GPIOs. One in use must refuse every valid GPIO outside `free` with `VC_ERR_PIN_IN_USE` and every invalid one with `VC_ERR_PIN_INVALID`. A live function is never moved to a free GPIO by the test, since that would restart outputs.
- **Refused starts.** A stored held pin is accepted, flag bit 3 is clear, and the start is refused with the function left as it was. This is covered for an optional S/PDIF input, MCK, `INPUT.SOURCE` to I2S, `OUTPUT.TYPE` to I2S and the PDM output enable.
- **Generation.** Moving a stored pin changes the generation.

Existing tests that need a free or owned GPIO (`gpio_owners()`, `busy_gpios()`) read `SYS.PIN_CLAIMS`, so the host suite carries no copy of the pin rules. Firmware without the command falls back to the older reading of every config.

### 13.2 Not yet covered

- **Bulk and preset restores.** The validator cases (colliding output pins replaced with the substitution bit set, an enabled MCK on a taken pin switched off with its pin kept, disabled functions on taken pins left alone, two output pins swapped) need the output pin offsets inside the bulk blob, which host tooling does not have yet.
- **Deferred families.** `XPORT`, the DAC mute and Control Surfaces report through `XPORT.STATUS`, the DAC mute readback and `CS.STATUS`; their masks are not yet compared with those channels.
- **The notification** is not yet captured by the suite.

### 13.3 Bench

- Slot alignment after pin moves that the new rules newly allow, and after each refused start.
- A control interface placed on the idle clock pair, followed by a refused switch to I2S. The control link must stay up.
- The ESP32 front panel against a build that emits `PIN_MAP_CHANGED`.

---

## 14. Implementation phasing

1. **Checks and snapshot.** Add `PinState`, `pin_check()` with its two modes and the function enum. Migrate the synchronous pin SET handlers. No wire change.
2. **In-use rule and start checks.** Apply section 4.6. Add the start checks, including the new ones on `OUTPUT.TYPE`, `INPUT.SOURCE`, the clock mode SETs and the PDM start. Add the live-or-pending union.
3. **Deferred applies.** Migrate the UART, I2C and DAC mute applies.
4. **Unit validator.** Replace the bulk, preset and boot pin code with the validator and the fallback rule.
5. **Control Surfaces.** Move the pin section of binding validation into the check. Route CS, IR and macro actions that start a function through the start check.
6. **Wire.** Add the availability GETs, `SYS.PIN_STATE`, `SYS.PIN_CLAIMS`, the generation sync and the notification. Regenerate the registry and update the documentation.
7. **Hosts and tests.** Add the `dspi_test` cases, switch the Console over and run the bench checks.

Phases 1 to 5 change accepted and refused requests. They can ship before
phase 6, but the behaviour changes in section 11 arrive with them.

---

## 15. Sites to migrate

| Site | File | What changes |
|---|---|---|
| `output_type_set` | `vendor/vc_page_output.c:30` | Gains the I2S start check |
| `output_pin_set` | `vendor/vc_page_output.c:49` | Calls `pin_check()` for `OUTPUT` or `PDM`, in the mode the PDM state selects |
| `output_adat_tx_enable_set`, `output_adat_tx_pin_set` | `vendor/vc_page_output.c:150`, `:176` | Enable is a start check. Pin SET mode follows the enable state |
| `dac_hw_mute_set_config` | `dac_hw_mute.c:319` | `gpio_in_range()` and `collides_with_other_subsystem()` removed |
| Input source SET | `vendor/vc_page_input.c:40` | Gains the I2S start check |
| `input_spdif_rx_pin_set`, `input_spdif_rx_enable_set` | `vendor/vc_page_input.c:89`, `:115` | Same behaviour, through the shared check |
| `input_i2s_rx_pin_set`, `input_i2s_rx_channels_set` | `vendor/vc_page_input.c:156`, `:185` | `check_i2s_rx_pin()` removed. Conflict checks only while I2S is the source |
| `input_adat_rx_enable_set`, `input_adat_rx_pin_set` | `vendor/vc_page_input.c:252`, `:278` | `adat_rx_pin_taken()` removed |
| `clock_bck_pin_set`, `clock_pin_mode_set`, `clock_i2s_mode_set` | `vendor/vc_page_clock.c:35`, `:76`, `:107` | Conflict checks only for the pair in use. Mode SETs gain the start check |
| `clock_mck_enable_set`, `clock_mck_pin_set` | `vendor/vc_page_clock.c:133`, `:156` | GPOUT rule moves into the check. Pin SET while disabled is store mode |
| PDM start paths | `main.c:1828`, `:3088`, `:3229` | Gain the start check |
| `uart_ctrl_validate` | `uart_control.c:529` | Mux, instance and ownership rules move into the check |
| `i2c_ctrl_validate` | `i2c_control.c:357` | Same, plus the display instance rule |
| CS binding and display validation | `control_surfaces.c:1640`, `:1775` | Pin section moves into the check. Non-pin validation stays |
| Bulk apply | `bulk_params.c:476` to `:780` | Replaced by the unit validator |
| Preset and boot restore | `flash_storage.c:3066`, `:3320` to `:3565` | Replaced by the unit validator |
| Ownership helpers | `vendor/vc_pins.c` | `is_pin_in_use()`, `pin_used_by_fixed_peripheral*()`, `ctrl_iface_check_pin()`, `i2s_clock_pin_claimed()` and the `*_acceptable()` family removed |

Each module reports its own claims to the claim list: `uart_ctrl_live_config()`,
`i2c_ctrl_live_config()`, `dac_hw_mute_get_config()` with `dac_hw_mute_owns_pin()`,
and `control_surfaces_add_pin_claims()`. The UART, I2C and Control Surfaces
`*_owns_pin()` functions are gone.

---

## 16. Open items

1. **ESP32 front panel.** Whether it tolerates an unknown notification event is unverified. It also needs checking against the new refusals on source and output type switches. Bench check before release.
2. **Priority order.** The order in 7.4 follows the current restore sequence. It needs a review against real board layouts before it is frozen, since it decides which function loses a collision.
3. **Stored configurations that become invalid.** Items 7, 12 and 16 of section 11 can disable a function on an existing unit after the update. The release notes must say so.
4. **Hardware bench.** Slot alignment after the newly allowed moves and the refused starts (13.3), and the PDM start and stop with the pin muxed only while running.
5. **Test coverage.** Section 13.2 lists what the suite does not cover yet.
6. **Generation tick cost.** The 50 ms main-loop tick rebuilds the claim list from flash-resident code. Measure it with the loop profiler build on RP2040; if it costs more than a few tens of microseconds, RAM-pin it or slow it further.

## 17. Settled during implementation

*Added 2026-11-18.*

- **PDM.** The user decided that a stopped PDM must not drive its pin. Boot no longer starts the PDM SM or DMA. The Core 1 start path sets the pin's PIO direction and mux with the SM stopped, and both stop paths clear them after stopping it, so a stored PDM pin never drives a pin another PIO1 state machine listens on. The start check sits in `ROUTE.OUTPUT_ENABLE` for the PDM output and in the restore validator.
- **S/PDIF input 1.** It keeps its claim. The user decided against an enable flag for it for now.
- **Start paths outside host commands.** Control Surfaces bindings, IR commands and macros reach the input source and output type through `vc_set()`, so they run the same handlers and start checks. Preset load, bulk SET and factory reset go through the restore validator.
- **Overlapping clock pairs.** `process_type_switches()` releases every outgoing pin, the old clock pair included, before it claims any, and the I2S input stops before it restarts on the new pair. Overlapping pairs therefore swap safely, and the two pairs are one function in the conflict check.
- **Control interface defaults.** The factory UART pins 16/17 and I2C pins 18/19 pass the mux rules.
- **Dispatch context.** UART and I2C requests are dispatched from the main loop, like USB requests from `tud_task`, so the GETs may read the live state.
- **Start checks see queued switches.** A start check judges the change on top of every queued output-type, input-source and clock-mode switch, so raising the I2S channel count while a switch to I2S is queued is checked too.
- **Hardware claims.** The live claim list also holds pins the hardware still drives after its config has let go of them: the clock pair a dormant clock-mode change has not moved yet, an ADAT reroute not yet applied, and a PDM fade-out.
