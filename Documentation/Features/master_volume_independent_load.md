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

The governing principle: **master volume follows the active preset *context*.**
A context switch (preset load, active-slot delete, boot) re-derives master volume
from `(mode, slot)`; a factory reset is *not* a context switch (it clears the DSP
chain but keeps the active preset selected), so it leaves the ceiling intact.

So the intended semantics are:

| Context | Independent (mode 0) | With-preset (mode 1) |
|---|---|---|
| **Boot** | apply saved directory value | slot value, or factory default (−20 dB) for an empty/legacy slot |
| **Runtime preset load** | **leave live value untouched** | slot value, or factory default (−20 dB) for an empty/legacy slot |
| **Active-slot delete** | **leave live value untouched** | factory default (−20 dB) — active context is now empty |
| **Factory reset** | **leave live value untouched** | **leave live value untouched** (not a context switch) |

Note the with-preset empty-slot case: because an empty preset loads factory
defaults for *everything* (EQ flat, delays zero, …), its master volume is the
power-on default (`MASTER_VOL_DEFAULT_DB`, −20 dB), **not** the directory's
independent value (which is a mode-0 concept and has no meaning in mode 1).

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

In `firmware/STM32/Core/Src/flash_storage.c`, `apply_master_volume_from_mode()`
is the single source of truth for "what master volume becomes when the preset
context changes", and every context-changing path calls it consistently:

1. `apply_master_volume_from_mode(slot_or_null, is_boot)`:
   - with-preset: a V12+ slot uses its own value; any context without one (empty
     slot **or** legacy pre-V12 preset) is "factory defaults" and gets
     `MASTER_VOL_DEFAULT_DB` (−20 dB).
   - independent: applies the saved directory value **only when `is_boot`**;
     runtime is a no-op (live value survives).
2. `apply_factory_defaults()` no longer touches master volume at all — it resets
   only the DSP processing chain.
3. The context callers each call the helper after loading: `preset_load()` and
   `preset_boot_load()` are structurally identical (load slot *or* factory
   defaults, then `apply_master_volume_from_mode(loaded_slot_or_null, is_boot)`);
   `preset_delete()` of the active slot calls it with `NULL` (now-empty context).
   `flash_factory_reset()` deliberately does **not** call it (not a context
   switch → ceiling left intact).

Verified on hardware: with-preset load of a configured slot restores the slot
value; with-preset load of an empty slot (and active-slot delete) gives −20 dB;
independent-mode loads leave the live value untouched; boot restore still works.

## Porting back to RP

The changes apply almost verbatim — `apply_master_volume_from_mode`,
`apply_factory_defaults`, `preset_load`, `preset_delete`, and the
`preset_boot_load` call sites are structurally identical between the two trees.
The one extra consideration on RP is the deferred/main-loop preset machinery
(`preset_load` is reached via the pending-flag path in `main.c`), but the
master-volume application points are the same functions, so the fix is
mechanical. The deliberate policy decisions to preserve when porting: factory
reset leaves master volume untouched, and an empty/legacy slot in with-preset
mode uses the −20 dB factory default (not the directory value).
