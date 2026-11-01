# Device Name

*Last updated: 2026-11-01*
*Status: implemented, HW-untested*
*Platforms: RP2040 + RP2350*

Users can give a board a name of its own, such as "Living room" or "Rack left". The board stores the name, so it keeps it wherever it is plugged in: any DSPi Link hub, or straight into any DSPi Console. Before this feature the name lived only in the hub's preferences, keyed by serial, and was lost when the board moved.

This is the board-level sibling of input source names (`input_source_names_spec.md`). It shares that feature's rules for length, staging, persistence and notification, and the two share one staging table in firmware.

---

## 1. Model

- **One name per board.** It is device-global. Preset save, preset load and factory reset leave it alone, as they do source names. Bulk params (0xA0/0xA1) do not carry it.
- **Empty means default.** An all-zero name means the user has not named the board. Each host supplies its own fallback (the Console shows "DSPi (serial)"; a hub uses its stored name or "DSPi" plus the serial tail, section 5).
- **Length.** Up to 31 bytes plus a NUL terminator, in a 32-byte field (`DEVICE_NAME_LEN`). Hosts should send printable ASCII, because the front panel font only covers characters 32-126.

## 2. Vendor commands

| Opcode | Name | Dir | wValue | Payload / response |
|---|---|---|---|---|
| 0x56 | `REQ_SET_DEVICE_NAME` | OUT | 0 | 1-32 bytes of name. A payload whose first byte is NUL clears the name. Bytes past 31 are dropped. |
| 0x57 | `REQ_GET_DEVICE_NAME` | IN | 0 | 32 bytes, NUL-terminated. All zeros when the board is unnamed. |

- A non-zero wValue STALLs the GET and makes the SET a no-op.
- Both commands work over USB, UART and I2C through the shared vendor dispatcher.
- A GET issued after a SET returns the new name at once, even before the flash write.

### Detecting support

Probe 0x57 on its own. Passing the 0x55 source-name probe does not imply support: firmware from 2026-10-31 to 2026-11-01 has source names but STALLs 0x57, and older firmware answers 0x57 with the 1-byte legacy channel mute. A host sends a 32-byte GET and treats a STALL or any reply shorter than 32 bytes as "not supported". It must never send 0x56 to firmware that fails the probe.

## 3. Persistence

The name is a 32-byte block appended to the preset directory at version 25 (`char device_name[32]`, after `input_source_names`).

- **Migration.** V24 migrates by copying its whole data block (`PRESET_DIRECTORY_V24_SIZE`), and V23 and older keep their existing paths. Both leave the new block zero, which reads as unnamed.
- **Why not fold into V24.** The current-version loader CRCs `sizeof(PresetDirectory)` bytes. A unit that ran the first V24 build holds a shorter V24 image, so a grown V24 would fail the CRC and drop every directory setting.
- **Sanitize on load.** The load forces a NUL into the last byte.
- **Sector budget.** The directory grows from 3407 to 3439 bytes of the 4096-byte sector.
- **Downgrade.** Older firmware treats a V25 directory as missing, which resets every directory-stored setting, as earlier bumps did.

## 4. Apply timing and notification

A SET is staged in the same per-entry table as the source names (an internal entry after the eight source entries) and flushed by the same main-loop pass, `input_source_names_apply_pending()`. That pass writes every staged name, source and board alike, in one directory flush inside `prepare_flash_write_operation()` / `complete_flash_write_operation_full()`. The full completion restarts every output together, so slot alignment is preserved. A rename costs a brief output restart.

A host that saves a board name and source names together should send the SETs back to back. If they all land before the main loop runs its pass, the user hears one restart instead of several. This is best effort, not a guarantee.

After the flush the firmware pushes `NOTIFY_EVT_DEVICE_NAME` (0x10), in the same 40-byte layout as 0x0F with byte 4 zero:

```
[ver=2, evt=0x10, flags=0, seq, 0, param_src, 0, 0, name[32]]   // 40 bytes
```

The event fires even when the directory write fails. In that case the name lives in RAM until reboot, as it does for source names. I2C clients have no notification consumer and must re-read.

## 5. DSPi Link hubs

The board is the single authority for its name on firmware that passes the probe. Hubs treat it like this:

- **Inventory.** The hub reads 0x57 while reading identity. The `name` in `device.list` is the board's name when set, else the hub-stored name, else the serial default. The hub applies that order every time a board comes online, not only on first sight.
- **Notify is the only writer.** On capable firmware the hub sets the registry name from `NOTIFY_EVT_DEVICE_NAME` and nothing else, so a rename from any source is seen: `device.rename`, a raw 0x56 through `cmd`, the hub's own local UI, a UART client or the front panel.
- **`device.rename`.** On capable firmware the hub writes 0x56 as the requesting session and replies `ok` once the board accepts the write. `device.changed` follows from the notify. A capable board that is not attached (offline within its grace window) answers `no_device`, because a stored name would be overridden by the board's own name when it returns. An unknown handle also answers `no_device`. The hub stores what the board can hold: it drops characters outside printable ASCII, closes the gaps they leave, trims the ends and caps the name at 31 characters, so "Café – Rack" becomes "Caf Rack". On old firmware the hub updates its stored name and emits `device.changed` itself, as before.
- **Probe at attach.** The hub reads 0x57 with the identity and retries once, so a transient miss does not treat a capable board as old firmware for the whole attachment.
- **One-time adoption.** A hub that already stored a name for a capable board that reports an empty name writes the stored name to the board once. The hub deletes its stored entry as soon as the board accepts that write, whenever it reads a non-empty name from the board, and on any 0x10 for that serial. Deleting on acceptance keeps adoption to one attempt even if the confirming notify is lost, so a later deliberate clear is never undone. The cost is that a directory write failing on the board loses the adopted name at the next reboot. The adoption write is an unrequested flash write with a brief output restart at attach time. It happens at most once per board per hub, and only for boards already named on that hub.

## 6. Front panel display

When no display page is current, the label line shows the board name instead of "DSPi", truncated to the panel width.

## 7. DSPi Console

- Probe support once per connection and read the name.
- The Global Parameters settings page has a "Device Name" field. Like the source-name fields, it drops unsupported characters and stops at 31 as you type, so the field never shows more than will be stored. Typing edits a draft that joins the Settings unsaved-changes bar, the same as source names. Save writes it, after any source names, in one burst of SETs.
- A `NOTIFY_EVT_DEVICE_NAME` for the connected board updates the cached name. A draft that now matches it no longer counts as unsaved.
- The device picker shows the board name for a locally connected board once it has been read. A board reached through a hub keeps the hub's inventory name, which on capable firmware is the board's own name.
- The field appears only on firmware that passes the probe. Remote boards rename through the same field, because 0x56 travels through the hub's `cmd` passthrough and the hub learns the result from the notify.

## 8. Test coverage

`tools/dspi_test` checks a round trip, truncation of a 32-byte payload to 31 bytes, clearing with a leading NUL, a STALL on a non-zero wValue, and that the name survives a preset load. The flash write itself needs a power cycle on the bench.
