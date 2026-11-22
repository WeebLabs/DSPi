# Input On/Off and Count: Implementation Plan

*Last updated: 2026-11-19*

Status: firmware, protocol registry, device tests and docs implemented on release/v1.1.7 (uncommitted). DSPi Console and DSPi Link follow-ups and the bench pass are still owed. Target is firmware 1.1.7.

## 1. Summary

Today the three wired inputs follow three different rules for when they hold their pins. This plan gives every input block the same shape. Each block has a main switch. Two blocks also have a count under the switch. A block that is on holds exactly the pins its count needs, whether or not it is the selected source. A block that is off holds none.

The result is that a source switch can never fail on a pin clash. Every clash shows up when the user turns a block on or raises its count, which is where the pin availability queries can help.

Breaking changes to commands, wire format and stored config are accepted for this release.

## 2. User-facing model

| Block | Switch | Count | Pins held while on |
|---|---|---|---|
| USB | None, always present | None | None |
| S/PDIF | On/off | Instances, 1 to 4 | The data pin of inputs 1 to N |
| I2S | On/off | Channels, 2, 4, 6 or 8 (2 only on RP2040) | Channels / 2 data pins, plus the clock pair the clock mode selects |
| ADAT (RP2350) | On/off | None | The ADAT input pin |

Rules that apply to every block.

- The count is remembered while the block is off.
- Off holds no pins. The stored pin numbers stay as stored claims.
- A source is selectable only if its block is on and, for S/PDIF, its index is within the count.
- Turning a block off while it is the active or pending source is refused with `VC_ERR_STATE`. So is lowering the S/PDIF count below the active or pending S/PDIF source. The host must switch source first. This is the rule S/PDIF 2 to 4 and ADAT already follow (`vendor/vc_page_input.c:129`, `:266`).
- Lowering the I2S channel count while I2S is selected stays allowed, as it is today.

Defaults for a fresh unit are S/PDIF on with count 1, I2S off with count 2, and ADAT off. That matches what a fresh unit holds today. See decision 1. A factory reset run while the device listens to another input keeps that input's block on, since the restore cannot cut a live source.

## 3. What changes from today

| Input | Today | After |
|---|---|---|
| S/PDIF input 1 | Always held, no off state | Held while the S/PDIF block is on |
| S/PDIF inputs 2 to 4 | Held per input by an enable mask (`spdif_rx_enabled_ext`) | Held while the block is on and the count reaches them |
| I2S data pins | Held only while I2S is the source | Held while the I2S block is on |
| I2S clock pair | Held while I2S is the source or an output slot is I2S | Held while the I2S block is on or an output slot is I2S |
| ADAT | Held while enabled with a pin | Unchanged |

The only cases where `INPUT.SOURCE` can return `VC_ERR_PIN_IN_USE` today are the I2S data pins and the clock pair. Both go away.

## 4. Commands (page INPUT, 0x03)

| ID | Name | Change |
|---|---|---|
| 0x23 | (retired) | The old per-input enable. The ID is left unassigned so an old client gets an unknown-command error |
| 0x27 | `SPDIF_RX_ENABLE` | New. The S/PDIF block switch. No index. Value 0 or 1 |
| 0x26 | `SPDIF_RX_COUNT` | New. Value 1 to 4. Settable while the block is off |
| 0x24 | `SPDIF_RX_CONFIG` | Reply becomes input count limit, block switch, instance count, then one GPIO per input |
| 0x34 | `I2S_RX_ENABLE` | New. Value 0 or 1 |
| 0x31 | `I2S_RX_CHANNELS` | Same values. Settable while the block is off. The pin check now bites whenever the block is on |
| 0x40 | `ADAT_RX_ENABLE` | Unchanged |
| 0x10 | `SOURCE` | No longer runs a pin check. Refuses a source whose block is off or out of count with `VC_ERR_STATE` |

0x26, 0x27 and 0x34 are free in `vendor/pages/input.def` today.

Error behaviour for the switch and count SETs.

- Out-of-range value returns `VC_ERR_VALUE`.
- A SET that would need a pin another function holds returns the pin error from `vc_from_pin_status()`, and nothing changes.
- A SET that would remove the active or pending source returns `VC_ERR_STATE`.
- A SET of the current value returns `VC_OK`.

Flags. The I2S switch and count need `VC_F_DEFERRED | VC_F_RESTART` when they change what the running clock generator drives. The S/PDIF and ADAT switches claim or release pins without touching running audio, as `SPDIF_RX_ENABLE` and `ADAT_RX_ENABLE` do today. Confirm in Phase 2 whether turning the I2S block on with no I2S output slot must start the clock generator or may only reserve its pins. Any path that starts or stops clocks must go through the existing full restart so output slot alignment is preserved.

Notifications. Every switch and count SET calls `notify_param_write()` on its wire field so that `PARAM_CHANGED` (0x02) reaches DSPi Link hubs and other clients. Bulk SET, preset load and factory reset stay inside the bulk bracket and emit one `BULK_INVALIDATED` (0x03). `PIN_MAP_CHANGED` (0x11) follows from the pin generation changing and needs no new code.

## 5. Wire format

`WireInputConfig` is 16 bytes with no spare byte (`bulk_params.h:204`). The plan reuses two bytes so the size does not change.

| Byte today | Becomes |
|---|---|
| `spdif_rx_enabled_ext_p1` | `spdif_rx_count`, 1 to 4 |
| `adat_input_enabled_p1` | `input_enable`, a bit mask. Bit 0 S/PDIF, bit 1 I2S, bit 2 ADAT |

`i2s_input_channels` keeps its place and meaning. `WireBulkParams` stays at 6136 bytes, so the size assert in `bulk_params.c:61` and the length in `store.def` hold. `WIRE_FORMAT_VERSION` moves from 32 to the next free number. Check the unmerged branches for collisions before picking it.

`bulk_params_apply()` treats a count outside its range as "keep the live value". It then applies the same guard as today, which is that the active or pending source cannot be switched off by a push (`bulk_params.c:502`). That guard grows to cover I2S and the S/PDIF count.

## 6. Pin rules in `vendor/vc_pins.c`

`PinConfig` (`vendor/vc_pins.h:90`) loses `spdif_rx_enabled_ext` and gains `spdif_rx_enabled`, `spdif_rx_count` and `i2s_rx_enabled`.

`cfg_in_use()` becomes the following.

| Function | In use when |
|---|---|
| `PIN_FN_SPDIF_RX` idx | `spdif_rx_enabled && idx < spdif_rx_count` |
| `PIN_FN_I2S_RX` idx | `i2s_rx_enabled && idx < i2s_rx_pairs` |
| `PIN_FN_ADAT_RX` | Unchanged |
| `PIN_FN_I2S_CLK` idx | Unchanged expression, but `cfg_clocks_run()` tests `i2s_rx_enabled` in place of `input_source == INPUT_SOURCE_I2S` |

The clock rule applies in every clock mode. `cfg_clock_pair()` already picks the slave pair in SPLIT slave mode, so that case needs no special handling.

Start checks. `start_with()` already builds a before and after view and checks every function that becomes in use in the after view. A count raise from 1 to 4 is therefore all-or-nothing as long as it goes through one `start_with()` call. Add edit functions and wrappers for the S/PDIF switch, the S/PDIF count and the I2S switch, next to `pin_start_check_i2s_rx_pairs()`. Never loop per input.

`pin_start_check_source()` loses its purpose. A source switch no longer brings any function into use. Remove the call from `input_source_set` and delete the function if nothing else uses it.

`input_source_selectable()` (`audio_input.h:268`) gains the block and count tests for S/PDIF and I2S.

Availability queries need no format change. An input above the count, or in a block that is off, reports as a stored claim and without `PIN_AVAIL_IN_USE`. I2S pairs within the count report as active while the block is on, even when I2S is not selected.

### Restore policy

A restore is a boot, a preset load or a bulk SET, all through `pin_restore_apply()`. The policy for the new model is below.

1. Before validation, the callers force the selected and the live source to be valid. The block of that source is turned on, and the S/PDIF count is raised to reach it. This replaces the mask-forcing code in `bulk_params.c:502` and `flash_storage.c:3313`, and it now covers I2S.
2. S/PDIF inputs are claimed in ascending order through the existing enable-flag rule (`claim_or_off()`). A taken pin is kept as the stored preference and the count is cut to the last input that claimed. Only a pin that can never work is replaced by its default. The `PIN_FN_SPDIF_RX` bit is set in `substituted` either way.
3. The count is never cut below the live S/PDIF source. If input 1 cannot claim, the block is switched off and the same bit is set. If the cut or the switch-off would remove the live source, the restore is abandoned to the base config through the existing abandon path.
4. A restored config that selects a source the cut removed has its source reverted and `source_reverted` set, as I2S does today.
5. I2S stays one unit of clock pair plus counted data pins (`claim_i2s_set()`). If the unit cannot claim, the I2S block is switched off, the count is kept, and the `PIN_FN_I2S_RX` bit is set. When I2S is the kept source it cannot switch off, so each of its pins tries its default and then the base pin, and only if one still fails is the restore abandoned. If I2S was the newly selected source, the source reverts. `is_live_input()` gains an I2S case so a restore cannot switch off I2S while the device is listening to it.
6. ADAT is unchanged.

The host reads the outcome through `pin_restore_substituted()` as it does now.

## 7. Single record or not

Each of these settings exists in six places today. They are the subsystem global, `PinConfig`, `FlashOutputConfig`, `PresetSlot`, `WireInputConfig` and the notify shadow.

Recommendation: do not make `PinConfig` the single record in this work. `PinConfig` is a value type that the checks copy and edit to compare before and after. The globals are read directly by the audio code (`i2s_input.c`, `audio_pipeline.c`, `adat_input.c`). Flash and wire need their own fixed layouts whatever the live record looks like. Collapsing the globals into one live `PinConfig` would touch audio code that this change otherwise leaves alone, and would only remove one of the six copies.

What this work should do instead is keep one bridge. `pin_config_capture()` and `pin_config_install()` stay the only functions that translate between globals and `PinConfig`. The new handlers write the new globals in one small setter each, so no handler open-codes the state change.

## 8. Persistence and migration

New fields go in both `FlashOutputConfig` (directory) and `PresetSlot`. `DIR_VERSION_CURRENT` (25) and `SLOT_DATA_VERSION` (39) each move to the next free number. Check unmerged branches for collisions.

Mapping from an existing record.

| Old state | New state |
|---|---|
| S/PDIF input 1, always on | S/PDIF block on |
| Enable mask over inputs 2 to 4 | Count = highest enabled input. No bits set gives count 1 |
| `i2s_input_channels` | Kept as the count |
| I2S selected as source in the slot | I2S block on |
| I2S not selected | I2S block off |
| ADAT enable and pin | Unchanged |

Points to watch.

- A sparse mask, for example only input 3 enabled, becomes count 3. That newly claims input 2's pin. If it clashes, restore rule 2 cuts the count and reports it. If input 3 is the selected source, rule 1 and rule 3 keep it or abandon to base.
- The directory record holds no input source. A unit whose slot selects I2S but whose pin config comes from the directory gets the I2S block turned on by restore rule 1, not by the directory migration.
- A unit that had I2S selected and an I2S data pin now shared with another function cannot exist today, so that migration cannot clash.
- A unit with I2S not selected migrates to I2S off, which holds no pins. No migration can take a pin from a working function.
- Directory migration is one more version branch in `dir_load_cache()`. Slot migration is a version gate in `io_config_from_slot()` and `apply_slot_to_live()`, plus a new size in `slot_data_size_for_version()`.

## 9. Follow-ups outside the firmware

DSPi Console (`/Users/weeblabs/DSPi Console`).

- `setSpdifInputCount` (`DSPi_ConsoleApp.swift:9651`) loops per-input enables today. It becomes one `SPDIF_RX_COUNT` SET.
- Add a main switch to the S/PDIF block and to the I2S block. The count picker sits under the switch and stays usable while off.
- Update `VendorCommands.swift`, the `SPDIF_RX_CONFIG` parser, `PinIOApplyDevice.swift`, and the bulk decoder for the two reused bytes.
- Update `SpdifInputWireTests.swift` and `PinMapLiveTests.swift`.
- Firmware and Console ship as a matched pair, so this lands in the same Console release.

DSPi Link (`/Users/weeblabs/DSPi-Link`).

- Add rows to `policy/commands.json` for `INPUT.SPDIF_RX_COUNT` (0x0326), `INPUT.SPDIF_RX_ENABLE` (0x0327) and `INPUT.I2S_RX_ENABLE` (0x0334), and remove the 0x0323 row, with `get` as `read` and `set` as `config`, like the existing `SPDIF_RX_ENABLE` row. A command with no row is treated as config class on both GET and SET, so bridges would refuse even the GET to non-admin clients.
- Regenerate `link_policy_table.c` with `bridge/esp32/tools/gen_policy.py`.
- Update the fake device in the host tests, and any decoder that reads the two reused bytes.
- Link's payload limit is 8192 bytes (`LINK_MAX_PAYLOAD_BYTES`). The number 6136 is the bulk blob size and appears in Link tests, docs and `BULK_PARAMS_BYTES`. This plan keeps the blob at 6136. Wire V35 then grew it to 6252 bytes for the upmixer section, so those values must change to 6252, which still fits the payload limit.

Other clients. The ESP32 front panel client checks values strictly and will need the new commands. Add the change to `Documentation/Features/vendor_protocol_client_update.md`.

## 10. Docs to update

- `Documentation/current_architecture.md`. Sections "Pin Ownership and Availability" (in-use table), "Restore Order in `io_config_apply()` and `bulk_params_apply()`", "Input Source Enum", "Multiple Selectable SPDIF Inputs", "I2S Input", "ADAT Input", the memory or storage version notes, and the vendor command reference.
- `Documentation/Features/pin_availability_query_spec.md`. Section 4.6 (in-use table and the "S/PDIF input 1 keeps its claim" paragraph), 7.3, 7.4, and a line under section 17.
- `Documentation/protocol/dspi_commands.json`. Regenerate with `scripts/gen_vendor_protocol.py` after editing `input.def` and `store.def`.
- `Documentation/Features/vendor_protocol_client_update.md`.

## 11. Test plan

Device tests in `tools/dspi_test/tests/`.

- `paged_input.py`. Replace the per-input enable tests with switch and count round trips for S/PDIF and I2S. Cover count set while off, no-op SETs, and value errors leaving state untouched.
- `paged_pins.py`. Replace `source_switch_refused_on_held_rx_pin` with a test that a source switch never returns a pin error. Add I2S enable refused on a held data pin and on a held clock pin, in unified and in split slave mode. Add a count raise from 1 to 4 with input 3's pin held, and check that the count and all claims are unchanged afterwards.
- `inputs.py`. Source switch refused with `VC_ERR_STATE` when the block is off or the index is above the count. Switch-off and under-count refused while the source is active and while it is pending.
- Notifications. Each new SET produces `PARAM_CHANGED` on the right offset. A bulk SET produces one `BULK_INVALIDATED`.
- Availability. I2S pairs report in use while the block is on and I2S is not selected. Inputs above the count report as stored. The generation changes on every switch and count change.
- Restore. Bulk SET with a clashing S/PDIF pin cuts the count and sets the substituted bit. A push that would switch off the live source keeps it. An I2S unit that cannot claim switches the block off.
- Migration. Load directory and slot images saved by 1.1.6 for four cases. They are defaults, S/PDIF 2 to 4 enabled, a sparse mask, and I2S selected with 8 channels.
- Update `golden/*.json`, `crosscut.py`, `helpers.py` and `selftest.py` for the new commands.

Host tests. Update `scripts/tests/gen_vendor_protocol/`, the Console wire tests and the Link conformance tests.

Bench. With S/PDIF, I2S and ADAT all on, switch between every pair of sources and confirm no refusal. Confirm inter-slot output alignment is unchanged across each switch and across turning the I2S block on and off with an I2S output slot running.

Build both platforms and run `scripts/check_ram_placement.py`.

## 12. Phases

1. **Pin rules.** New `PinConfig` fields, `cfg_in_use()`, `cfg_clocks_run()`, capture and install, the new start checks, and the restore policy. Globals added with their setters. No command changes yet, so behaviour is reachable only through tests of the pin layer.
2. **Commands.** `input.def` changes, handlers, `input_source_selectable()`, removal of the source pin check, notifications. Settle the I2S clock generator question from section 4.
3. **Wire and storage.** `WireInputConfig`, `bulk_params_apply()`, directory and slot fields, version bumps, migration branches.
4. **Tests and docs.** Device tests, goldens, regenerated catalogue, architecture doc, pin availability spec, client update guide.
5. **Console and Link.** Console UI and wire code, Link policy rows and regenerated table.
6. **Bench pass** on RP2040 and RP2350, including the migration images and the alignment check.

Phases 1 to 4 land together in the firmware. Phase 5 must ship with the same release.

## 13. Later: output slots

Out of scope here. If an off state for output slots is added later, off means the slot keeps running in the synchronized start with its pin detached. It is not a third output type, and it must not change inter-slot alignment.

## 14. Decisions

*Last updated: 2026-11-19*

These were open in the first draft and are now settled.

1. **Default for the S/PDIF block on a fresh unit.** On, with count 1. This keeps today's behaviour.
2. **Sparse mask migration.** "Only input 3 enabled" becomes count 3. Inputs are never renumbered.
3. **Restore corner case.** When an S/PDIF input below the live source cannot claim its pin or its default, the restore is abandoned to the base config.
4. **I2S block on with no I2S output slot.** The pins are only reserved. The clocks start when I2S is selected, as they do today. The switch therefore needs no restart.
5. **ID 0x23.** Retired. The S/PDIF block switch takes the fresh ID 0x27, so an old client fails loudly.
