# Firmware Versioning Specification

## TL;DR

- A firmware version is three plain numbers, such as 1.1.6. There are never any beta or rc suffixes.
- The device can only report those three numbers over USB. A suffix like `-beta2` never reaches the host, so two builds that share a version cannot be told apart.
- Every build that anyone outside the bench can install gets its own patch number. Private test builds keep the version they are based on and get a build label instead. No software ever compares a label.
- The macOS Console app and the firmware ship as a matched pair with the same version. The app turns features on and off based on the version the device reports.
- Legal ranges are major 0 to 255, minor 0 to 15, patch 0 to 255. The minor limit is a real trap and is explained under "Legal ranges" below.
- Old hosts keep working without any change. They ask for 4 bytes and get exactly the same 4 bytes they always got.
- Separately from the version, every binary carries an automatic git stamp readable via `REQ_GET_BUILD_INFO` (0x80) and `picotool info`. The version is the contract; the stamp is provenance. See "Build provenance" below.

## Why beta suffixes are gone

`REQ_GET_PLATFORM` (0x7F) is the only way a host learns which firmware is running, and it carries exactly three numbers. Both `1.1.6-beta1` and `1.1.6-beta2` report themselves as `1.1.6`. The Console's feature gating and any auto-updater see one build, not two. An updater then cannot tell whether it needs to update, and a bug report that says "1.1.6" does not say which build it means. Suffixes are therefore banned.

- **Published builds.** This means anything a user can install, test releases included. Bump the patch number. 1.1.6, then 1.1.7, then 1.1.8, and so on. Patch numbers are cheap, so burn them freely.
- **Private test builds.** These never leave the bench. Keep the version of the release they are based on and add a build label such as a date or a short hash in the filename or the release notes. A label exists only for humans to read. Never turn test status into a version suffix.

## App and firmware are a matched pair

DSPi Console reads the version the device reports and uses it to decide which features to offer. Wire-format versions, capability versions, and the per-feature `firmwareSupportsX >= N` checks all key off the answer to 0x7F. A firmware version bump must therefore ship alongside the matching app release. Bumping only one side breaks the gating contract. The app also bundles the matching `.uf2` images for its updater.

## Version macros

`config.h` defines the version.

```c
#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            1
#define FW_VERSION_PATCH            6
#define FW_VERSION_PACKED           ((FW_VERSION_MAJOR << 8) | (FW_VERSION_MINOR << 4) | FW_VERSION_PATCH)
```

`FW_VERSION_PACKED` was once called `FW_VERSION_BCD`, which was a misleading name. It is plain nibble packing rather than BCD. Minor and patch each get 4 bits, and the macro does no range checking at all, so anything that does not fit simply overlaps its neighbour. The packed form exists only to build the legacy bytes of `REQ_GET_PLATFORM`. Nothing else may use it.

## Legal ranges

| Field | Legal range | What breaks past it |
|-------|-------------|---------------------|
| Major | 0 to 255 | Byte 1 is a plain `uint8_t`, so 256 wraps to 0. |
| Minor | 0 to 15 | The reported **major** number changes. See below. |
| Patch | 0 to 255 | Legacy 4-byte hosts read the wrong patch. New 6-byte hosts stay correct. |

A patch number above 15 is safe for current hosts. It overflows its nibble in byte 2, which corrupts the legacy encoding, but byte 5 still carries the true value and any host built after this change reads byte 5. This is the tradeoff the widening was made for.

A minor number above 15 is not safe, and this is the trap. Byte 1 is `FW_VERSION_PACKED >> 8`, and the macro ORs `FW_VERSION_MINOR << 4` into that same 16-bit word. A minor of 16 sets bit 8, which is the low bit of the major byte. Version 2.16.0 would therefore report itself as major 3 to every host, old and new alike, because there is no separate full-width major byte to fall back on.

If the firmware ever needs a minor number above 15, mask the packed macro first so the spill cannot happen.

```c
#define FW_VERSION_PACKED  ((FW_VERSION_MAJOR << 8) | ((FW_VERSION_MINOR & 0xF) << 4) | (FW_VERSION_PATCH & 0xF))
```

The bulk parameter header carries the version separately as two `uint16_t` fields, so it is not affected by any of this. `REQ_GET_PLATFORM` is the only place the limits bite.

## Wire encoding: REQ_GET_PLATFORM (0x7F)

**Direction:** Device to Host (GET)
**wValue:** 0 (unused)
**wIndex:** Vendor interface number
**wLength:** any length. New hosts ask for 6, old hosts ask for 4, and the firmware truncates to whatever was asked for.

### Response (6 bytes)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `platform` | `0` = RP2040, `1` = RP2350 |
| 1 | 1 | `fw_major` | `FW_VERSION_MAJOR` |
| 2 | 1 | `fw_minor_patch_legacy` | `(FW_VERSION_MINOR << 4) \| FW_VERSION_PATCH`, each nibble caps at 15 |
| 3 | 1 | `num_outputs` | Compile-time `NUM_OUTPUT_CHANNELS` |
| 4 | 1 | `fw_minor` | `FW_VERSION_MINOR`, full width |
| 5 | 1 | `fw_patch` | `FW_VERSION_PATCH`, full width |

Bytes 0 to 3 are byte-for-byte identical to the historical 4-byte response. Bytes 4 and 5 repeat the minor and patch from byte 2, but each gets a whole `uint8_t`, so they stay correct past 15. Once patch goes above 15 the low nibble of byte 2 is no longer trustworthy and only byte 5 should be believed.

### Why the old 4-byte encoding capped at 15

The original response was 4 bytes, and minor and patch shared byte 2 with one nibble each. Four bits hold 0 to 15 and the packing macro never checked for overflow, so 15 was a hard ceiling on both numbers. Back when beta suffixes existed, almost no patch numbers were ever spent and that ceiling was far away. Now that every published build spends a patch number, the ceiling would arrive within a year or two, which is why bytes 4 and 5 were added.

### Short reads

The firmware always offers 6 bytes and the transfer layer cuts the data stage down to the host's `wLength`. TinyUSB does this in `tud_control_xfer` with `tu_min16(len, request->wLength)`, and the UART and I2C dispatch paths apply the same cap. A legacy host that asks for 4 bytes therefore receives exactly the old 4-byte response.

## Compatibility matrix

| Host | Firmware | Behavior |
|------|----------|----------|
| Old (asks 4) | Old (offers 4) | Unchanged. 4 bytes, nibble decode. |
| Old (asks 4) | New (offers 6) | The transfer is cut to 4 bytes. The host sees the same legacy bytes as before and decodes the nibbles. This is correct as long as minor and patch are both 15 or below. |
| New (asks 6) | Old (offers 4) | The transfer completes short with 4 bytes and the host falls back to the nibble decode. |
| New (asks 6) | New (offers 6) | The host receives 6 bytes and uses bytes 4 and 5. |

**Fallback rule for hosts.** Always ask for 6 bytes. If 6 or more arrive, take minor from byte 4 and patch from byte 5. If fewer than 6 arrive, decode minor as `byte[2] >> 4` and patch as `byte[2] & 0x0F`. Never mix the two decodes within one read.

## Host implementation

```c
uint8_t info[6];
int n = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    0x7F /* REQ_GET_PLATFORM */,
    0, VENDOR_INTF,
    info, 6, 1000);
if (n < 4) {
    return -1;  /* identification unavailable, do not read info[] */
}
int major = info[1];
int minor = (n >= 6) ? info[4] : (info[2] >> 4);
int patch = (n >= 6) ? info[5] : (info[2] & 0x0F);
```

## Build provenance: REQ_GET_BUILD_INFO (0x80)

The version number answers "what does this build promise to hosts". It cannot answer "which exact source built this binary", because two bench builds and a release can all legally report the same three numbers. The build stamp answers that second question.

Every build automatically embeds the output of `git describe --always --dirty` plus the build date. Nobody types it and nobody can forget it: `scripts/gen_build_info.cmake` runs on every build (build time, not configure time, so it never goes stale) and generates `build_info.h` in the build directory. A string like `v1.1.7-3-g1a2b3c4` means "3 commits past tag v1.1.7, exact commit 1a2b3c4". A `-dirty` suffix means the tree had uncommitted changes at build time, so the hash is approximate and the build should not be trusted as reproducible.

The stamp is readable three ways.

- `REQ_GET_BUILD_INFO` (0x80) returns a 64-byte blob from the running device. Bytes 0 to 47 are the describe string and bytes 48 to 59 are the build date `YYYY-MM-DD`, both ASCII and NUL-padded. Old firmware STALLs the request.
- `picotool info <file>.uf2` reads the same string from the UF2's binary info, identifying a build file with no device attached.
- `tools/dspi_test` prints it in the profile summary and checks it in the identity tests.

Two rules keep it honest. First, the stamp is for humans only; no software may gate, compare, or parse it, exactly as with build labels. Second, a release build must be made from a clean checkout of the tagged commit, so a published binary reports the bare tag with no offset and no `-dirty`. A release stamp that says anything else means the binary was not built from the tag.

Wire layout details live in `Documentation/Features/device_identification_spec.md`.

See `Documentation/Features/device_identification_spec.md` for the full `REQ_GET_PLATFORM` and `REQ_GET_SERIAL` reference.
