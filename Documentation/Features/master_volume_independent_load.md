# Master Volume on Preset Load — Independent-Mode Behavior

*Last updated: 2026-05-27*

## Summary

This note documents a behavioral discrepancy in how **master volume** is handled
when a preset is loaded in **independent** mode (`MASTER_VOLUME_MODE_INDEPENDENT`),
and a fix that has been applied to the **STM32H723 port** but **not** to this
(RP2040/RP2350) firmware.

> **Status on RP:** unchanged — RP still exhibits the behavior described under
> "Current RP behavior". This is a known divergence, recorded here so the two
> codebases don't silently drift without explanation. Decide deliberately
> whether to port the STM32 fix back.

## The contract

The DSPi Console describes independent mode to users as:

> "Master volume is stored on the device independently of presets and applied at
> boot. **Loading a preset never changes it.**"

So the intended semantics are:

| Context | Independent (mode 0) | With-preset (mode 1) |
|---|---|---|
| **Boot** | apply saved directory value | apply slot value (fallback: directory) |
| **Runtime preset load** | **leave live value untouched** | apply slot value |
| **Active-slot delete / factory reset** | **leave live value untouched** | (n/a) |

## Current RP behavior (the bug)

`apply_master_volume_from_mode()` in `firmware/DSPi/flash_storage.c` is called
unconditionally on every load path, including runtime `preset_load()`
(around line 1099) and inside `apply_factory_defaults()`. In independent mode
that helper applies `dir_cache.master_volume_db` — the last value persisted by
`REQ_SAVE_MASTER_VOLUME` (0xD6).

Net effect: **every runtime preset load (empty or occupied) snaps the live
master volume to the last-saved directory value**, even though the user may have
changed the live value since. This violates "loading a preset never changes it".

Confirmed on STM32 hardware before the fix (raw USB, console not involved):
saved baseline −5 dB, live set to −25 dB (unsaved), load any preset → live
reverted to −5 dB. Boot restore itself was correct; only runtime loads misbehaved.

## The STM32 fix

In `firmware/STM32/Core/Src/flash_storage.c`:

1. `apply_master_volume_from_mode()` gained an `is_boot` parameter. In
   independent mode it now applies the saved directory value **only when
   `is_boot` is true**; runtime calls are a no-op (live value survives). In
   with-preset mode it always applies (slot value, or directory fallback).
2. `apply_factory_defaults()` no longer touches master volume at all (it runs on
   runtime empty-load, active-slot delete, and the factory-reset command — all
   of which must leave the live value untouched).
3. Boot paths in `preset_boot_load()` call
   `apply_master_volume_from_mode(slot_or_null, /*is_boot=*/true)` explicitly so
   boot restore is unchanged.

Verified on hardware: independent mode leaves live volume untouched across empty
and filled preset loads; with-preset mode still restores the slot's saved value;
boot restore still works.

## Porting back to RP

The same three changes apply almost verbatim — `apply_master_volume_from_mode`,
`apply_factory_defaults`, and the `preset_boot_load` call sites are structurally
identical between the two trees. The one extra consideration on RP is the
deferred/main-loop preset machinery (`preset_load` is reached via the pending-
flag path in `main.c`), but the master-volume application points are the same
functions, so the fix is mechanical. Factory-reset semantics ("leave master
volume untouched") were chosen deliberately and should be preserved if ported.
