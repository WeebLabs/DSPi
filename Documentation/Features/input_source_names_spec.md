# Input Source Names

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7)*
*Status: implemented, HW-untested*
*Platforms: RP2040 + RP2350*

Users can give each input source (USB, S/PDIF 1-4, I2S, ADAT) a name of their own, such as "Turntable" or "TV". The device stores one name per source. Every host reads the same names, and the front panel display shows them.

This feature names input *sources*. It is separate from channel names (`ROUTE.CHANNEL_NAME`), which label individual input and output channels and travel with presets. The board's own name (`device_name_spec.md`) shares this feature's staging table and main-loop write.

---

## 1. Model

- **One name per source ID.** The table is indexed by the `InputSource` enum (`audio_input.h`). It has 8 entries so a future source fits without a layout change. IDs 0..`INPUT_SOURCE_MAX` are valid today.
- **Device-global.** A source name describes what is plugged into a jack, so it does not change when a preset loads. Names live in the preset directory beside the other board-level settings. Preset save, preset load and factory reset all leave them alone, the same as Control Surfaces names.
- **Empty means default.** An all-zero entry means the user has not named that source. The firmware never invents a default string for the host. Each host supplies its own built-in label (the Console shows "S/PDIF 1" when optional inputs are enabled, for example). The front panel display falls back to its built-in label table.
- **Length.** A name is up to 31 bytes plus a NUL terminator, in a 32-byte field (`INPUT_SOURCE_NAME_LEN`, equal to `PRESET_NAME_LEN`). The firmware treats the bytes as opaque. Hosts should send printable ASCII, because the front panel font only covers characters 32-126 and renders anything else as a space.
- **Any structurally valid source can be named.** That includes ADAT on RP2040 and disabled optional S/PDIF inputs. This matches `input_source_valid()`, so a name set before an input is enabled is still there afterwards.

## 2. Vendor commands

In firmware 1.1.6 the feature used opcodes 0x54 and 0x55, which had previously carried a legacy three-channel gain and mute that no audio path read (see section 6). The board name is the separate command `SYS.DEVICE_NAME`.

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `INPUT.SOURCE_NAME` | 0x03 | 0x12 | GET, SET | source ID 0..`INPUT_SOURCE_MAX` | GET 32, SET 1-32 | GET returns 32 bytes, NUL-terminated, all zeros when the source is unnamed. SET takes 1-32 bytes of name. A payload whose first byte is NUL clears the name. Bytes past 31 are dropped. |

- An invalid source ID STALLs either direction with `VC_ERR_ADDRESS`. A `wValue` with a non-zero high byte counts as invalid. In 1.1.6 the SET was a silent no-op.
- An empty or over-long SET payload STALLs with `VC_ERR_LENGTH`. The reason for any STALL is readable from `SYS.LAST_RESULT` (see `../commands.md`).
- A SET is applied on the main loop (section 4). A GET issued after the SET returns the new name straight away, even if the flash write has not happened yet.
- The command works over every control transport (USB, UART, I2C) through the paged dispatcher (`vc_dispatch()`). A 32-byte GET fits all of them.

### Detecting support

Every firmware that speaks the paged protocol (1.1.7 and later) has `INPUT.SOURCE_NAME`. A host that gates features on the command table checks the command's GET bit in `SYS.PAGE_COMMANDS` for page 0x03. Firmware 1.1.6 and older does not answer the paged protocol at all (`vendor_command_overhaul_plan.md` section 5.14), so a host never sends this command to it.

## 3. Persistence

Names live in a new block at the end of the preset directory, which moves to directory version 24.

```c
char input_source_names[INPUT_SOURCE_NAME_SLOTS][INPUT_SOURCE_NAME_LEN];  // 8 x 32 = 256 bytes
```

- **Migration.** V23 migrates by copying its whole data block. The new block stays zero, which reads as "every source unnamed". Older versions already migrate through a zeroed cache and pick up the same result.
- **Sanitize on load.** The current-version load forces a NUL into the last byte of each entry, so a corrupt sector can never hand an unterminated string to a GET or the display.
- **Sector budget.** The directory grows from 3151 to 3407 bytes, inside the single 4096-byte sector.
- **Downgrade.** Older firmware has no V24 migration and treats a V24 directory as missing. Flashing a pre-V24 build after this one therefore resets every directory-stored setting (startup preset, Control Surfaces, control interfaces, independent IO config and the rest), not only the names. The V23 bump had the same property.
- **Branch collision.** The `overclock` and `ac3-dts-decode` branches each also claim directory version 23 for their own blocks. Whichever of them merges next has to renumber its version and stack its block after V25 (the board name, `device_name_spec.md`).

## 4. Apply timing

The SET handler runs in USB IRQ context, which cannot write flash. It copies the name into a per-source pending buffer, records the dispatch source (USB, UART or I2C), and sets that source's bit in a pending mask. Each source has its own buffer, so renaming several sources in quick succession loses nothing.

The main loop takes a snapshot of every pending entry with interrupts disabled, clears the mask, and writes all of them in a single directory flush. That flush runs inside the usual `prepare_flash_write_operation()` / `complete_flash_write_operation_full()` bracket, like every other directory write. The full completion restarts all outputs together, so inter-slot alignment is preserved. A rename does cost the same brief output restart as a preset rename.

A SET that matches the stored name exactly is still written. Hosts are expected to send a SET only when the user commits a change.

If the directory write fails, the new name stays in the RAM cache and is still announced, so GETs and the display show it until the next reboot. Preset renames behave the same way.

## 5. Notification

After the flush, the firmware pushes one `NOTIFY_EVT_INPUT_SOURCE_NAME` (0x0F) per renamed source. It sends 0x0F rather than 0x0E because the `ac3-dts-decode` branch holds 0x0E.

```
[ver=2, evt=0x0F, flags=0, seq, source_id, param_src, 0, 0, name[32]]   // 40 bytes
```

`param_src` is the `ParamSource` of the SET that staged the name. A host can ignore events it caused itself. The name is the stored value, all zeros when cleared, so a host can update without a follow-up GET.

## 6. Retirement of the legacy gain and mute

In firmware older than this feature, opcodes 0x54 to 0x57 set and read `channel_gain_db[3]`, `channel_gain_mul[3]`, `channel_gain_linear[3]` and `channel_mute[3]`. Nothing in the audio path read those values, and the Console never sent the commands. The variables are deleted.

The `legacy` block of `WireBulkParams` and the matching fields of the preset slot keep their layout, because both formats are versioned and frozen. The firmware now always writes 0.0 dB and unmuted into them, and ignores whatever a bulk SET or a stored preset puts there. Neither format version changes.

## 7. Front panel display

On a display page for `CS_NOUN_INPUT_SOURCE`, the value line shows the custom name when one is set and the built-in label otherwise. The line is truncated to the panel's column count. The display re-renders on its normal cadence, so a rename appears without any extra hook.

## 8. DSPi Console

- Probe support once per connection (section 2) and read the name of every valid source.
- `inputSourceTitle()` returns the custom name when it is set and the built-in label otherwise. The source picker, the Control Surfaces value text and the unsaved-changes list all go through it. The notification log keeps raw built-in labels on purpose.
- The Inputs settings page ends with a "Source Names" section. Each source the device can select gets a row with its built-in label, where it physically comes in (for example "Receiver on GPIO 11") and a name field. A field keeps only printable ASCII up to 31 characters as you type. Clearing it, or its clear button, restores the built-in label.
- Typing only edits a draft. Edited names join the Settings unsaved-changes bar, which writes each changed name on Save and drops the drafts on Revert. They also count toward the pending-changes warning on a device switch. An orange dot marks a row whose name is not saved yet.
- Right-clicking the source picker in the main window sidebar opens a context menu with "Rename ..." for the selected source, plus "Use Built-in Name" when it has a custom name. Rename opens a small sheet with the same filtering as the settings field. Confirming the sheet writes at once and replaces any unsaved Settings draft for that source.
- Handle `NOTIFY_EVT_INPUT_SOURCE_NAME` by updating the cached name for that source.

## 9. Test coverage

`tools/dspi_test` replaces the legacy gain and mute tests with a source-name test. It checks a round trip on USB, S/PDIF 1 and S/PDIF 4, truncation of a 32-byte payload to 31 bytes, clearing with a leading NUL, a STALL on an invalid source ID, that a SET with a high `wValue` byte changes nothing, and that a name survives a preset load. Because a GET returns a staged name at once, the suite cannot prove the flash write itself. That needs a power cycle on the bench.
