# Firmware Versioning Specification

## TL;DR

- Firmware versions are three plain numbers: major.minor.patch (e.g. 1.1.6). No beta/rc suffixes, ever.
- The device can only report those three numbers over USB; a suffix like `-beta2` is invisible to the host, so two builds sharing a version cannot be told apart.
- Every build that anyone outside the bench can install gets its own patch number. Private test builds get a build label instead; a label is never compared by software.
- The macOS Console app and the firmware ship as a matched pair with the same version, because the app gates features on the version the device reports.
- The old wire encoding packed minor and patch into half a byte each, capping both at 15. `REQ_GET_PLATFORM` now appends two full-width bytes so patch numbers can run past 15.
- Old hosts keep working unchanged: they ask for 4 bytes and get exactly the same 4 bytes as before.

## Why beta suffixes are gone

`REQ_GET_PLATFORM` (0x7F) is the only way a host learns the running firmware version, and it carries exactly three numbers. A `1.1.6-beta1` and a `1.1.6-beta2` both report `1.1.6`; the Console's feature gating and any auto-updater see them as the same build. An updater cannot decide whether to update, and a bug report saying "1.1.6" is ambiguous. So suffixes are banned:

- **Published builds** (anything a user can install, including test releases): bump the patch number. 1.1.6, 1.1.7, 1.1.8, and so on. Patch numbers are cheap; burn them freely.
- **Private test builds** (never leaves the bench): keep the version of the release they are based on and mark them with a build label (e.g. a date or short hash in the filename or release notes). Nothing compares a label; it exists only for humans. Never encode test status as a version suffix.

## App and firmware are a matched pair

DSPi Console reads the reported version and gates features on it (wire-format versions, capability versions, and per-feature `firmwareSupportsX >= N` checks all key off what 0x7F returns). A firmware version bump therefore always ships together with the matching app release; bumping one side alone breaks the gating contract. The app also bundles the matching `.uf2` images for its updater.

## Version macros

`config.h` defines the version:

```c
#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            1
#define FW_VERSION_PATCH            6
#define FW_VERSION_PACKED           ((FW_VERSION_MAJOR << 8) | (FW_VERSION_MINOR << 4) | FW_VERSION_PATCH)
```

`FW_VERSION_PACKED` (formerly misnamed `FW_VERSION_BCD`) is plain nibble packing, not BCD: minor and patch each occupy 4 bits with no carry handling, so each caps at 15. A patch of 16 would silently corrupt the minor field. The packed form exists only to feed the legacy bytes of `REQ_GET_PLATFORM`; nothing else may use it.

## Wire encoding: REQ_GET_PLATFORM (0x7F)

**Direction:** Device to Host (GET)
**wValue:** 0 (unused)
**wIndex:** Vendor interface number
**wLength:** 6 (new hosts) or 4 (legacy hosts)

### Response (6 bytes)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `platform` | `0` = RP2040, `1` = RP2350 |
| 1 | 1 | `fw_major` | `FW_VERSION_MAJOR` |
| 2 | 1 | `fw_minor_patch_legacy` | `(FW_VERSION_MINOR << 4) \| FW_VERSION_PATCH`, each nibble caps at 15 |
| 3 | 1 | `num_outputs` | Compile-time `NUM_OUTPUT_CHANNELS` |
| 4 | 1 | `fw_minor` | `FW_VERSION_MINOR`, full width |
| 5 | 1 | `fw_patch` | `FW_VERSION_PATCH`, full width |

Bytes 0-3 are byte-for-byte identical to the historical 4-byte response. Bytes 4-5 are the same minor and patch as byte 2, but as plain `uint8_t` each, so they keep working past 15. Once patch (or minor) exceeds 15, byte 2's corresponding nibble is no longer trustworthy; only bytes 4-5 are.

### Old 4-byte encoding and why it capped at 15

The original response was 4 bytes with minor and patch sharing byte 2 as one nibble each. Four bits hold 0-15, and the packing macro has no carry logic, so 15 was a hard ceiling on both fields. With suffixed betas, few patch numbers were ever burned and the cap was distant; with every published build taking a patch number, it would be reached quickly. Hence the widening.

### Short reads

The firmware always offers 6 bytes; the transfer layer clamps the data stage to the host's `wLength` (TinyUSB `tud_control_xfer` clamps via `tu_min16(len, request->wLength)`; the UART/I2C dispatch path applies the same cap). A legacy host requesting 4 bytes receives exactly the old 4-byte response.

## Compatibility matrix

| Host | Firmware | Behavior |
|------|----------|----------|
| Old (asks 4) | Old (offers 4) | Unchanged: 4 bytes, nibble decode. |
| Old (asks 4) | New (offers 6) | Transfer clamps to 4; host sees the identical legacy bytes and decodes nibbles. Correct as long as minor and patch are both <= 15. |
| New (asks 6) | Old (offers 4) | Transfer completes short with 4 bytes; host falls back to nibble decode. |
| New (asks 6) | New (offers 6) | Host receives 6 bytes and uses bytes 4-5. |

**Fallback rule for hosts:** request 6 bytes. If at least 6 arrive, take minor from byte 4 and patch from byte 5. If fewer than 6 arrive, decode minor as `byte[2] >> 4` and patch as `byte[2] & 0x0F`. Never mix the two decodes.

## Host implementation

```c
uint8_t info[6];
int n = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    0x7F /* REQ_GET_PLATFORM */,
    0, VENDOR_INTF,
    info, 6, 1000);
if (n < 4) { /* identification unavailable */ }
int major = info[1];
int minor = (n >= 6) ? info[4] : (info[2] >> 4);
int patch = (n >= 6) ? info[5] : (info[2] & 0x0F);
```

See `Documentation/Features/device_identification_spec.md` for the full `REQ_GET_PLATFORM` and `REQ_GET_SERIAL` reference.
