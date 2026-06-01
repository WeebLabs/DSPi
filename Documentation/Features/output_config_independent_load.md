# Physical IO / Output Config — Independent vs. With-Preset Persistence

*Last updated: 2026-06-01*

## Summary

This note documents how the device's **physical IO/output configuration** —
output pins, output types (SPDIF/I2S), I2S MCK/BCK, and the SPDIF RX pin — is
persisted relative to presets. A single directory flag,
`output_config_mode` (`OUTPUT_CONFIG_MODE_*`), selects between two behaviors,
mirroring the existing **master-volume** independent/with-preset mechanism.

This **repurposes the former `include_pins` flag**, which previously only gated
whether `output_pins[]` + `spdif_rx_pin` were restored on preset load (output
*types*, I2S MCK/BCK, and input source always travelled with presets, whether or
not that was wanted). The flag is now a *mode* governing the whole IO block.

> **Status:** implemented on RP2040/RP2350. The byte keeps its directory offset
> and 1:1 value mapping (`1 → WITH_PRESET`, `0 → INDEPENDENT`), so existing
> devices are unaffected (default stays with-preset) and no value migration is
> needed — only a directory format bump (V3 → V4) that appends the device-global
> `FlashOutputConfig` block. **Input source (USB vs SPDIF) is NOT part of this
> block** — it stays per-preset, as it is a listening choice, not wiring.

## The contract

Like master volume, the IO config follows the active preset *context*:

> "Output configuration is either part of each preset (with-preset) or stored on
> the device independently and applied at boot (independent). In independent
> mode, **loading a preset never changes your wiring**; an explicit
> `REQ_SAVE_OUTPUT_CONFIG` persists the live IO to the device."

| Context | Independent (mode 0) | With-preset (mode 1, default) |
|---|---|---|
| **Boot** | apply saved directory block | slot value, or firmware defaults for an empty/legacy slot |
| **Runtime preset load** | **leave live IO untouched** | slot value, or firmware defaults for an empty/legacy slot |
| **Active-slot delete** | **leave live IO untouched** | firmware defaults — active context is now empty |
| **Factory reset** | **leave live IO untouched** (device-global wiring preserved) | reset IO to firmware defaults |

The factory-reset row differs slightly from master volume (which leaves the
ceiling intact in *both* modes): a DSP factory reset *does* reset IO to defaults
in with-preset mode (matching the prior behavior where `apply_factory_defaults()`
reset pins/types), but in independent mode the device-global wiring is left
intact — a DSP reset shouldn't wipe device-level routing.

## Design

`apply_output_config_from_mode(slot_or_null, is_boot)` is the single owner of the
live IO config across preset-context changes — the exact analog of
`apply_master_volume_from_mode()`:

- **with-preset:** a V6+/V9+ slot supplies the IO (`io_config_from_slot()`); any
  context without one (empty slot, NULL) gets firmware defaults
  (`io_config_defaults()`).
- **independent:** applies the saved directory block (`dir_cache.output_config`)
  **only when `is_boot`**; runtime is a no-op so the live IO survives.

IO application was **extracted out of** `apply_slot_to_live()` (the old
`include_pins` block + the I2S-config block) and `apply_factory_defaults()` (the
pin reset + I2S reset) so the owner function is the only place IO is sourced —
the same discipline master volume uses. A single `io_config_apply()` validates
pins, checks MCK GPOUT-capability, and schedules the SPDIF RX hot-swap when the
pin changes while RX is the active input; it is shared by both the per-slot and
device-global sources so they cannot diverge. `io_config_from_live()` snapshots
the live globals for `REQ_SAVE_OUTPUT_CONFIG`.

Context callers each invoke the helper right beside their existing
`apply_master_volume_from_mode()` call: `preset_load`, `preset_delete` (active
slot, NULL context), `preset_boot_load` (`is_boot=true`), and
`flash_factory_reset` (`NULL`, not boot → with-preset resets, independent no-op).

## Persistence (explicit save)

Per the chosen model, INDEPENDENT mode uses **explicit save** (like master
volume): live IO changes via the existing per-field commands
(`REQ_SET_OUTPUT_TYPE`, `REQ_SET_OUTPUT_PIN`, MCK setters, `REQ_SET_SPDIF_RX_PIN`)
take effect immediately but only survive reboot after a
**`REQ_SAVE_OUTPUT_CONFIG` (0x52)** — `preset_save_output_config()` snapshots the
live IO into `dir_cache.output_config` and flushes. There is no `get-saved`
command: the device-global block equals live IO right after a save, and the app
reads current IO via the existing per-field getters / bulk-params GET.

## Vendor commands (zero net-new opcodes)

- `REQ_SET_OUTPUT_CONFIG_MODE` (0x98) / `REQ_GET_OUTPUT_CONFIG_MODE` (0x99) —
  renamed from `REQ_PRESET_SET/GET_INCLUDE_PINS`; payload/return is the 0/1 mode
  byte (clamped `>1 → independent`). The mode is also byte [5] of the
  `REQ_PRESET_GET_DIR` summary.
- `REQ_SAVE_OUTPUT_CONFIG` (0x52) — **repurposed** from the dead, deprecated
  `REQ_LOAD_PARAMS`. No payload; 1-byte `PRESET_OK` status; flash write deferred
  to the main loop. Hosts use `REQ_PRESET_LOAD` (0x91) for "revert to saved".

## Directory format V3 → V4

`PresetDirectory` renames `include_pins` → `output_config_mode` (same byte) and
appends a packed `FlashOutputConfig` block (20 bytes, fixed/platform-independent;
only the first `NUM_PIN_OUTPUTS`/`NUM_SPDIF_INSTANCES` entries are meaningful).
`DIR_VERSION_CURRENT` is 4; `dir_load_cache()` migrates V1/V2/V3 forward, seeding
`output_config` from firmware IO defaults and carrying the device-level SPDIF RX
pin into the block. A device that had the non-default `include_pins=0` lands in
independent mode and boots to default routing until it re-saves via
`REQ_SAVE_OUTPUT_CONFIG` (output_config is unused for the default with-preset
majority).

## Inviolable constraint

This change alters only *when / from where* IO config is sourced (slot vs.
directory vs. skip), never *how* it is realized into hardware — the
retype / pin-reassign / synchronized-enable path (`process_type_switches`,
`complete_pipeline_reset`) is untouched, so inter-slot sample alignment is
preserved. In independent mode, runtime preset loads skip IO entirely (strictly
less disruption); boot applies the device-global IO via the same path a slot's
IO uses.

## Verification

Built clean on both platforms (RP2350 BSS +20 bytes = the `FlashOutputConfig`
added to the RAM-cached directory). On connected RP2350: V3→V4 migration
preserved `output_config_mode` (1, from `include_pins=1`); `output_config_save`
(0x52) acks `PRESET_OK`; `output_config_mode_roundtrip` (0x98/0x99 + clamp) and
the behavioral `output_config_independent_isolation` test pass — the latter
proving that, with the MCK multiplier as a non-disruptive marker, a preset load
leaves live IO untouched in independent mode and applies the slot's IO in
with-preset mode.
