# I2S Clock-Pin Mode Specification (Unified / Split BCK+LRCLK Pairs)

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7). 2026-07-10: initial version*

This document specifies the I2S clock-pin mode feature: what it does, the
complete host-facing control surface (vendor commands, validation rules,
notifications, bulk-parameter and persistence formats), and the integration
patterns an app developer or LLM needs to support it. It complements
`Documentation/Features/i2s_slave_input_spec.md` (the I2S clock-slave input
mode this feature extends) and `Documentation/Features/i2s_output_spec.md`.

---

## 1. Concept

The DSPi has two I2S *clock modes* (`CLOCK.I2S_CLOCK_MODE`):

- MASTER (0, default): the device drives BCK/LRCLK as outputs.
- SLAVE (1): an external master drives BCK/LRCLK; the device's pins are inputs.

Historically both clock modes used one GPIO pair. The *clock-pin mode*
selects whether that stays true:

| Pin mode | Value | Master clock mode uses | Slave clock mode uses |
|---|---|---|---|
| UNIFIED (default, legacy) | 0 | `i2s_bck_pin` (+1 for LRCLK) | the same pair |
| SPLIT | 1 | `i2s_bck_pin` (+1) | `i2s_bck_pin_slave` (+1) |

UNIFIED is convenient when one connector serves both roles. SPLIT exists for
boards that route the two roles to different physical connectors, e.g. a test
PCB that must exercise master-mode outputs and an external-master input
without rewiring; it also removes the dual-driver contention window described
in `i2s_slave_input_spec.md` section 2 (with SPLIT, an always-wired external
master never shares a pad with the device-driven master pair).

Key invariants:

- **LRCLK is always BCK + 1** in both pairs (PIO side-set hardware
  constraint). Each pair is stored/configured as a single BCK GPIO.
- The slave pair is **fully dormant in UNIFIED mode**: it is stored and
  reported, but nothing ever drives, reads, or reserves it.
- In SPLIT mode **both pairs are reserved**: validation refuses to assign RX
  data pins, control-interface pins, or the DAC-mute pin onto either pair,
  and the two pairs must not overlap each other.
- The firmware resolves the pair the hardware actually uses through one
  helper (`i2s_effective_bck_pin()`): the slave pair if and only if
  `pin mode == SPLIT && clock mode == SLAVE`, else the master pair.

Identical on RP2040 and RP2350 except the defaults below.

## 2. Default pins

| | BCK | LRCLK |
|---|---|---|
| Master/unified pair (both platforms) | GPIO 14 | GPIO 15 |
| Slave pair, RP2040 | GPIO 12 | GPIO 13 |
| Slave pair, RP2350 | GPIO 26 | GPIO 27 |

The slave defaults are the nearest sequential GPIO pair to 14/15 that no
other feature defaults to on that platform. On RP2040, 12/13 are free (ADAT
is RP2350-only and the RP2040 MCK default is GPIO 21). On RP2350 everything
adjacent to 14/15 is a default for some feature (13 MCK, 12 ADAT, 11 DAC
mute, 10 PDM, 9-6 SPDIF out, 5 SPDIF RX, 4-1 I2S RX data, 16/17 UART, 18/19
I2C, 20/21 optional SPDIF inputs 2/3), so 26/27 is the closest clean pair.

## 3. Vendor commands

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `CLOCK.I2S_CLOCK_PIN_MODE` | 0x0C | 0x11 | GET, SET | none (0) | GET 1, SET 1 | 0 UNIFIED, 1 SPLIT |
| `CLOCK.I2S_BCK_PIN` | 0x0C | 0x10 | GET, SET | role 0-1 | GET 1, SET 1 | BCK GPIO of the pair. LRCLK is that value + 1 |
| `CLOCK.I2S_CLOCK_MODE` | 0x0C | 0x12 | GET, SET | none (0) | GET 1, SET 1 | 0 MASTER, 1 SLAVE. Deferred |

All commands use the paged protocol (page in the `wIndex` high byte, command in `bRequest`, see `../commands.md`) and work over USB vendor control transfers and the UART/I2C control interfaces. Every SET carries its value in a 1-byte payload. There is no status byte: success is an ACK, and a rejected SET STALLs with a `VC_*` result readable from `SYS.LAST_RESULT` (UART and I2C return the code in the response status byte).

### `CLOCK.I2S_CLOCK_PIN_MODE` SET

Payload: 0 (UNIFIED) or 1 (SPLIT).

| Result | When |
|---|---|
| `VC_OK` | Applied (or no-op: already in this mode) |
| `VC_ERR_PIN_IN_USE` | Entering SPLIT while the slave pair overlaps the master pair or either slave GPIO is owned by another subsystem (RX data pin, fixed peripheral, control interface, DAC mute, Control Surfaces binding) |
| `VC_ERR_STATE` | The change would move the live effective pair (clock mode is, or is pending, SLAVE) while at least one output slot is I2S |
| `VC_ERR_VALUE` | Payload > 1 |

Semantics:

- Entering SPLIT is validated **even when dormant** (clock mode MASTER):
  the slave pair must be conflict-free at enable time, because a later
  clock-mode flip (`CLOCK.I2S_CLOCK_MODE`) adopts it without rechecking. If the enable is
  rejected with `VC_ERR_PIN_IN_USE`, move the slave pair first (`CLOCK.I2S_BCK_PIN` role 1) or free
  the conflicting assignment, then retry.
- Leaving SPLIT needs no pin checks (the master pair is reserved in every
  mode, so nothing else can occupy it).
- If the change moves the live effective pair with only the I2S *input*
  running (no I2S output slots), it applies immediately and the input
  restarts on the new pair via the usual deferred muted restart.
- If clock mode is MASTER the set is a pure stored-state change; nothing
  live moves (master clocking uses the master pair in both pin modes).
- With I2S output slots active and slave clocking in force the set is
  rejected (`VC_ERR_STATE`). Workaround sequence: switch the I2S slots to
  SPDIF (`OUTPUT.TYPE`), change the pin mode, switch back.

### `CLOCK.I2S_CLOCK_PIN_MODE` GET

Returns the live pin mode (0/1). The SET applies synchronously, so unlike
`CLOCK.I2S_CLOCK_MODE` there is no deferred-apply reporting window.

### `CLOCK.I2S_BCK_PIN` SET

`wValue` = role, payload = BCK GPIO:

- role 0 = master/unified pair.
- role 1 = slave pair. Storable at any time (including while UNIFIED, where
  it is dormant); useful to stage the pair before enabling SPLIT.
- A role above 1 is `VC_ERR_ADDRESS`.
- GPIO 0xFF (`PIN_RESET_TO_DEFAULT`) restores the role's default pair (section 2).

| Result | When |
|---|---|
| `VC_OK` | Applied or no-op (pin already set to that value) |
| `VC_ERR_PIN_INVALID` | GPIO or GPIO+1 invalid (out of range, or 23-25 reserved) |
| `VC_ERR_ADDRESS` | role > 1 |
| `VC_ERR_STATE` | The pair being moved is the one live I2S output slots clock against: role 0 while any slot is I2S (except in SPLIT + slave clocking, where the master pair is dormant and the set is allowed); role 1 while any slot is I2S in SPLIT + slave clocking |
| `VC_ERR_PIN_IN_USE` | GPIO or GPIO+1 owned by another subsystem, or the pair would overlap the other pair (role 1 always checks against the master pair; role 0 checks against the slave pair only in SPLIT mode, so the dormant slave default never constrains UNIFIED-mode hosts) |

If the input is running on the pair being changed, the set arms a deferred
muted input restart on the new pins.

### `CLOCK.I2S_BCK_PIN` GET

`wValue` = role (0 = master/unified pair, 1 = slave pair). Returns that
pair's BCK GPIO; LRCLK is that value + 1. A role above 1 is `VC_ERR_ADDRESS`.

### Interactions with existing commands

| Command | Behavior with clock-pin modes |
|---|---|
| `CLOCK.I2S_CLOCK_MODE` | In SPLIT mode a master/slave flip also swaps the effective pair; the deferred main-loop transition rebuilds input and output clocking on the correct pair automatically. |
| `OUTPUT.TYPE` | Unchanged. I2S slots always clock on the effective pair at (re)build time. |
| `INPUT.I2S_RX_PIN`, `INPUT.I2S_RX_CHANNELS` | Validation rejects data pins on either clock pair while SPLIT (both pairs claimed), only the master pair while UNIFIED. |
| `XPORT.UART_CONFIG`, `XPORT.I2C_CONFIG`, `OUTPUT.DAC_MUTE_CONFIG`, Control Surfaces bindings | Same rule: their pin validation treats both pairs as claimed in SPLIT mode. |
| `CLOCK.MCK_ENABLE`, `CLOCK.MCK_PIN`, `CLOCK.MCK_MULTIPLIER` | Unaffected; MCK is independent of the clock pairs (and forced off in slave clocking as before). |

## 4. Notifications

Both settings are mirrored into the bulk-params shadow, so applying them
emits standard `PARAM_CHANGED` (event 0x02) notifications carrying the wire
offset and 1-byte value:

- Pin mode: offset of `i2s_config.clock_pin_mode_p1`, value is the **+1
  encoding** (1 = unified, 2 = split), matching what `STORE.ALL_PARAMS` returns.
- Slave BCK: offset of `i2s_config.bck_pin_slave`, plain GPIO value.
- Master BCK (role 0): offset of `i2s_config.bck_pin`.

Within the `WireI2SConfig` section the byte positions are: `output_types[4]`
at 0-3, `bck_pin` 4, `mck_pin` 5, `mck_enabled` 6, `mck_multiplier` 7,
`clock_pin_mode_p1` 8, `bck_pin_slave` 9, reserved 10-15. Hosts using the
SDK's `WireBulkParams` struct get the absolute offsets from `offsetof`.

## 5. Bulk parameters (`STORE.ALL_PARAMS`)

`WIRE_FORMAT_VERSION` is **unchanged** (18): the two fields claim former
reserved bytes of the 16-byte `WireI2SConfig` section with absent-value
sentinels, so payloads from older hosts (zeros there) apply cleanly.

| Field | Encoding | SET semantics |
|---|---|---|
| `clock_pin_mode_p1` | 0 = absent (keep live), 1 = unified, 2 = split | Installed live when valid; decoded value must be <= 1 |
| `bck_pin_slave` | 0 = absent (keep live), else BCK GPIO | Installed only if the GPIO pair is valid, collision-free against fixed peripherals, and non-overlapping with the master pair installed by the same payload; otherwise the live pin is kept (same keep-live-on-reject model as `bck_pin`) |

The RX data-pin set inside the same payload is validated against the clock
pairs that will actually be installed (including these two fields), not the
pre-apply live values. If the effective input pair moves, the apply arms the
usual deferred muted input restart.

**Feature detection:** every firmware that speaks the paged protocol
(1.1.7 and later) has both commands. A host may check their bits in
`SYS.PAGE_COMMANDS` for page 0x0C. Firmware 1.1.6 and older does not answer
the paged protocol (`vendor_command_overhaul_plan.md` section 5.14). In a
bulk snapshot, `clock_pin_mode_p1 != 0` still marks firmware with the feature.

## 6. Persistence

Both settings are part of the physical IO block and follow the existing
**output-config mode** selection (`STORE.OUTPUT_CONFIG_MODE`):

- WITH_PRESET (default): saved into the active preset by `STORE.PRESET_SAVE`
  and restored on preset load, with the device-global directory value as the
  baseline for presets that predate the fields.
- INDEPENDENT: the live values survive preset loads; snapshot them
  device-globally with `STORE.OUTPUT_CONFIG_SAVE`; restored at boot.

Formats:

- **Preset slot V29** (`SLOT_DATA_VERSION` 29) tail-appends two bytes:
  `i2s_clock_pin_mode` (plain 0/1; 0 = unified) and `i2s_bck_pin_slave`
  (0 = unset, falls back to the device baseline then the platform default).
  Pre-V29 slots load with unified + default, byte-identical to their old
  behavior.
- **Directory V14** grows the device-global `FlashOutputConfig` from 29 to
  31 bytes with the same two fields; a V13 to V14 migration (and the whole
  older chain) fills them with 0 = unified/unset automatically.
- A preset/bulk restore that changes these fields while I2S is live applies
  them through the same deferred muted transitions as every other IO field;
  inter-slot output alignment is preserved by the synchronized restart paths.

Stored values are validated on apply exactly like the master BCK pin
(invalid or conflicting stored pins are rejected and the live/known-good
value is kept), so a corrupted or cross-platform image cannot install a
broken clock pair.

## 7. App integration patterns

### Enabling SPLIT mode

1. Optionally GET `CLOCK.I2S_BCK_PIN` role 1 to show the current slave pair; move it first
   with a `CLOCK.I2S_BCK_PIN` role 1 SET if the default conflicts with your wiring.
2. SET `CLOCK.I2S_CLOCK_PIN_MODE` to 1. Handle `VC_ERR_PIN_IN_USE` by prompting the
   user to free the conflicting pin or choose another pair; handle
   `VC_ERR_STATE` by offering to temporarily switch I2S output slots to SPDIF.
3. Persist: `STORE.PRESET_SAVE` (WITH_PRESET) or `STORE.OUTPUT_CONFIG_SAVE`
   (INDEPENDENT), as for any IO change.

### UI guidance

- Present the mode as a radio choice ("Shared clock pins" / "Separate pins
  per clock mode") with the slave-pair picker enabled only in SPLIT mode.
- Always display LRCLK as read-only BCK+1 (both pickers).
- When SPLIT is active, annotate both pairs as reserved in any pin-map UI;
  when UNIFIED, show the slave pair as stored-but-inactive.
- After a SET, update state from the PARAM_CHANGED notifications (or re-GET
  `CLOCK.I2S_CLOCK_PIN_MODE` / `CLOCK.I2S_BCK_PIN`) rather than assuming success; every SET can be rejected with
  a STALL and a result code.

### Sequencing detail

`CLOCK.I2S_CLOCK_PIN_MODE` and `CLOCK.I2S_BCK_PIN` apply their stored value synchronously (the
ACK or STALL is authoritative); only the hardware restart is deferred by a few main-loop
iterations. This differs from `CLOCK.I2S_CLOCK_MODE`, whose GET reflects the
change only after the deferred apply.

## 8. Edge cases and known limitations

- **Both pairs must be sequential GPIO pairs** (LRCLK = BCK + 1). There is
  no independent LRCLK assignment in either mode.
- **The pairs may not overlap** (including LRCLK adjacency: bases exactly 1
  apart collide). Enforced when setting the slave pin (always), the master
  pin (in SPLIT mode), enabling SPLIT, and on every stored-config apply.
- **In UNIFIED mode the dormant slave pin constrains nothing.** A
  UNIFIED-mode host can freely place the master pair or other functions on GPIOs 12/13
  (RP2040) / 26/27 (RP2350); the conflict, if any, is caught when SPLIT is
  enabled.
- **GPIO 26/27 are ADC-capable pins** (RP2350 default slave pair). Boards
  using them for analog must move the slave pair.
- **Rejection while outputs run is by design** (same rule as the legacy BCK
  pin): moving a pair that live I2S output slots clock against would break
  inter-slot sample alignment, which is inviolable. Use the
  SPDIF-switch-change-switch-back sequence.
- **The `CLOCK.I2S_CLOCK_MODE` flip validates nothing new**: it adopts whatever
  pair is effective. That is safe because every path that could make the
  slave pair effective (`CLOCK.I2S_CLOCK_PIN_MODE` enable, `CLOCK.I2S_BCK_PIN` role 1, stored-config apply)
  validated it first.

## 9. Implementation map (for firmware developers)

| Piece | Location |
|---|---|
| Mode/pin globals (`i2s_clock_pin_mode`, `i2s_bck_pin_slave`) | `firmware/DSPi/usb_audio.c` (defs), `audio_input.h` (externs, enum) |
| `i2s_effective_bck_pin()`, `i2s_clock_pin_claimed()` | `firmware/DSPi/audio_input.h` (inline) |
| Defaults (`PICO_I2S_BCK_PIN_SLAVE`) | `firmware/DSPi/config.h` |
| Command table (`CLOCK` page) | `firmware/DSPi/vendor/pages/clock.def` |
| Vendor handlers (`CLOCK.I2S_BCK_PIN` role indexing, `CLOCK.I2S_CLOCK_PIN_MODE`) | `firmware/DSPi/vendor/vc_page_clock.c` |
| Effective-pair consumers: TX `clock_pin_base`, rebuild change detection | `firmware/DSPi/main.c` (`process_type_switches`) |
| Effective-pair consumers: RX start snapshot, active-pin query | `firmware/DSPi/i2s_input.c` |
| Pin-claim validation (both pairs in SPLIT) | `firmware/DSPi/vendor/vc_pins.c` (`is_pin_in_use`, `check_i2s_rx_pin`, `ctrl_iface_check_pin`, `i2s_rx_pin_set_acceptable`), `dac_hw_mute.c` |
| Persistence (slot V29, directory V14, frozen v13 structs, migration) | `firmware/DSPi/flash_storage.c` |
| Wire fields + collect/apply | `firmware/DSPi/bulk_params.h` / `bulk_params.c` |
