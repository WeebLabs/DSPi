# Firmware Versioning Specification

*Last updated: 2026-11-16 (version read through `SYS.PLATFORM` on the paged protocol of firmware 1.1.7, probe order for older firmware)*

## TL;DR

- A firmware version is three plain numbers plus a beta ordinal, such as 1.1.6 beta 2. A final release is beta 0 and drops the word entirely.
- The device reports all four numbers over USB, so two betas of the same patch can be told apart by software. This is new; before it, a suffix like `-beta2` never reached the host and beta suffixes were banned outright.
- Betas of one patch share that patch number and count upward: 1.1.6 beta 1, beta 2, beta 3, then 1.1.6 final. A final release always sorts above every beta of the same patch.
- Private test builds keep the version they are based on and get a build label instead. No software ever compares a label.
- The macOS Console app and the firmware ship as a matched pair with the same version. The app turns features on and off based on the version the device reports.
- Legal ranges are major 0 to 255, minor 0 to 15, patch 0 to 255. The minor limit is a real trap and is explained under "Legal ranges" below.
- Firmware 1.1.7 reports the version through `SYS.PLATFORM` on the paged protocol, with the same 7 bytes the old opcode 0x7F returned. A host written for 1.1.6 cannot read it, because 1.1.7 refuses every request on page 0. A current host probes `SYS.PROTOCOL_INFO` first and falls back to opcode 0x7F only on older firmware (see "Reading the version" below).
- Separately from the version, every binary carries an automatic git stamp readable via `SYS.BUILD_INFO` (1.1.6 opcode 0x80) and `picotool info`. The version is the contract; the stamp is provenance. See "Build provenance" below.

## Beta builds

`SYS.PLATFORM` (1.1.6 opcode 0x7F) is the only way a host learns which firmware is running. It used to carry exactly three numbers, so both `1.1.6-beta1` and `1.1.6-beta2` reported themselves as `1.1.6`. The Console's feature gating and any auto-updater saw one build, not two, and a bug report that said "1.1.6" did not say which build it meant. That is why beta suffixes were banned.

Byte 6 fixes the cause rather than the symptom. The beta ordinal rides on the wire alongside the three numbers, so the host can tell betas apart and betas no longer have to burn a patch number each.

- **Betas.** Set `FW_VERSION_BETA` to 1 for the first beta of a patch and count upward. The patch number stays put across the whole beta run.
- **Final releases.** Set `FW_VERSION_BETA` back to 0 in the same commit that gets tagged. A final release is strictly newer than every beta of its patch, so 1.1.6 final supersedes 1.1.6 beta 3.
- **Private test builds.** These never leave the bench. Keep the version of the release they are based on, beta ordinal included, and add a build label such as a date or a short hash in the filename or the release notes. A label exists only for humans to read. Never turn test status into a version suffix.

Tags carry the ordinal as a suffix (`v1.1.6-beta2`) because git tags are for humans. Nothing parses it; the device reports the ordinal itself.

## App and firmware are a matched pair

DSPi Console reads the version the device reports and uses it to decide which features to offer. Wire-format versions, capability versions, and the per-feature `firmwareSupportsX >= N` checks all key off the answer to `SYS.PLATFORM`. On paged firmware a host can also gate a feature on the command bits of `SYS.PAGE_COMMANDS` or the page revisions in `SYS.PAGE_LIST`, which `vendor_command_overhaul_plan.md` section 5.14 prefers to version compares. A firmware version bump must therefore ship alongside the matching app release, beta ordinal included. The app reads the whole thing, ordinal and all, from its own `MARKETING_VERSION`, which carries the tag spelling during a beta run (`1.1.6-beta2`). Bumping only one side breaks the gating contract. The app also bundles the matching `.uf2` images for its updater.

## Version macros

`config.h` defines the version.

```c
#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            1
#define FW_VERSION_PATCH            7
#define FW_VERSION_PACKED           ((FW_VERSION_MAJOR << 8) | (FW_VERSION_MINOR << 4) | FW_VERSION_PATCH)
#define FW_VERSION_BETA             1
```

`FW_VERSION_BETA` is independent of the packed macro and never touches the legacy bytes. It reaches the host only as byte 6.

`FW_VERSION_PACKED` was once called `FW_VERSION_BCD`, which was a misleading name. It is plain nibble packing rather than BCD. Minor and patch each get 4 bits, and the macro does no range checking at all, so anything that does not fit simply overlaps its neighbour. The packed form exists only to build the legacy bytes of `SYS.PLATFORM`. Nothing else may use it.

## Legal ranges

| Field | Legal range | What breaks past it |
|-------|-------------|---------------------|
| Major | 0 to 255 | Byte 1 is a plain `uint8_t`, so 256 wraps to 0. |
| Minor | 0 to 15 | The reported **major** number changes. See below. |
| Patch | 0 to 255 | Legacy 4-byte hosts read the wrong patch. New 6-byte hosts stay correct. |
| Beta | 0 to 255 | Nothing. It is a whole byte of its own with no packing. |

A patch number above 15 is safe for current hosts. It overflows its nibble in byte 2, which corrupts the legacy encoding, but byte 5 still carries the true value and any host built after this change reads byte 5. This is the tradeoff the widening was made for.

A minor number above 15 is not safe, and this is the trap. Byte 1 is `FW_VERSION_PACKED >> 8`, and the macro ORs `FW_VERSION_MINOR << 4` into that same 16-bit word. A minor of 16 sets bit 8, which is the low bit of the major byte. Version 2.16.0 would therefore report itself as major 3 to every host, old and new alike, because there is no separate full-width major byte to fall back on.

If the firmware ever needs a minor number above 15, mask the packed macro first so the spill cannot happen.

```c
#define FW_VERSION_PACKED  ((FW_VERSION_MAJOR << 8) | ((FW_VERSION_MINOR & 0xF) << 4) | (FW_VERSION_PATCH & 0xF))
```

The bulk parameter header carries the version separately as two `uint16_t` fields, so it is not affected by any of this. `SYS.PLATFORM` is the only place the limits bite.

## Wire encoding: `SYS.PLATFORM`

| Command | Page | Cmd | Dir | Address (`wValue`) | Size |
|---|---|---|---|---|---|
| `SYS.PLATFORM` | 0x01 | 0x20 | GET | none (0) | GET 7 |

**Direction:** Device to Host (GET, `bmRequestType` 0xC1, `bRequest` 0x20)
**wValue:** 0. Any other value STALLs with `VC_ERR_ADDRESS`
**wIndex:** `0x0102`, page 0x01 in the high byte and the vendor interface number in the low byte
**wLength:** any length. Current hosts ask for 7, and the firmware truncates to whatever was asked for.

Firmware 1.1.6 and older answered the same bytes to opcode 0x7F with `wIndex` 0 or 2. That is the request the compatibility notes below call the legacy read.

### Response (7 bytes)

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | `platform` | `0` = RP2040, `1` = RP2350 |
| 1 | 1 | `fw_major` | `FW_VERSION_MAJOR` |
| 2 | 1 | `fw_minor_patch_legacy` | `(FW_VERSION_MINOR << 4) \| FW_VERSION_PATCH`, each nibble caps at 15 |
| 3 | 1 | `num_outputs` | Compile-time `NUM_OUTPUT_CHANNELS` |
| 4 | 1 | `fw_minor` | `FW_VERSION_MINOR`, full width |
| 5 | 1 | `fw_patch` | `FW_VERSION_PATCH`, full width |
| 6 | 1 | `fw_beta` | `FW_VERSION_BETA`; 0 = final release, 1 to 255 = beta N |

Bytes 0 to 3 are byte-for-byte identical to the historical 4-byte response. Bytes 4 and 5 repeat the minor and patch from byte 2, but each gets a whole `uint8_t`, so they stay correct past 15. Once patch goes above 15 the low nibble of byte 2 is no longer trustworthy and only byte 5 should be believed.

Byte 6 has no legacy counterpart. Every paged firmware answers all 7 bytes. Older firmware that answers opcode 0x7F with fewer than 7 bytes predates the ordinal, and every such build was a final release, so a short reply decodes as beta 0 rather than as unknown.

### Why the old 4-byte encoding capped at 15

The original response was 4 bytes, and minor and patch shared byte 2 with one nibble each. Four bits hold 0 to 15 and the packing macro never checked for overflow, so 15 was a hard ceiling on both numbers. Back when a beta run spent no patch numbers at all, that ceiling was far away. It came into reach during the period when every published build, betas included, had to burn its own patch number, which is why bytes 4 and 5 were added. Byte 6 has since given the betas their own counter again, but the widening stands: patch numbers are still cheap and still spent freely.

### Short reads

The firmware always offers 7 bytes and the transfer layer cuts the data stage down to the host's `wLength`. On paged firmware the USB front end in `firmware/DSPi/vendor/vc_usb.c` caps the reply at `wLength`, and `ctrl_dispatch_get()` applies the same cap for UART and I2C. A legacy host that asks for 4 bytes therefore receives exactly the old 4-byte response, and one that asks for 6 receives exactly the pre-beta 6-byte response.

## Reading the version

Follow the start-up order of `vendor_command_overhaul_plan.md` section 5.14.

1. Send GET `SYS.PROTOCOL_INFO` (page 0x01, command 0x10). A reply that starts with the magic `"DSPi"` means paged firmware. Read `SYS.PLATFORM`.
2. A STALL means firmware 1.1.6 or older. Read the legacy opcode 0x7F with `wIndex` 2 and decode it with the fallback rule below. Offer the update, and send that device no paged command, because old firmware ignores `wIndex` and would run a paged number as the old opcode with that number.

Firmware 1.1.7 refuses opcode 0x7F on page 0 like every other page 0 request, so a host built for 1.1.6 cannot read the version of a 1.1.7 device.

## Compatibility matrix

This matrix covers the legacy 0x7F read on firmware 1.1.6 and older, and `SYS.PLATFORM` on paged firmware, which always offers 7.

| Host | Firmware | Behavior |
|------|----------|----------|
| Asks 4 | Offers 4 | Unchanged. 4 bytes, nibble decode. |
| Asks 4 | Offers 7 | The transfer is cut to 4 bytes. The host sees the same legacy bytes as before and decodes the nibbles. This is correct as long as minor and patch are both 15 or below. |
| Asks 6 | Offers 4 | The transfer completes short with 4 bytes and the host falls back to the nibble decode. |
| Asks 6 | Offers 7 | The transfer is cut to 6 bytes. The host reads the full-width minor and patch and never learns the beta ordinal, so it treats every build of a patch as the same build. |
| Asks 7 | Offers 4 or 6 | The transfer completes short. The host decodes what arrived and takes beta as 0. |
| Asks 7 | Offers 7 | The host receives all four numbers. |

**Fallback rule for hosts.** Always ask for 7 bytes. If 6 or more arrive, take minor from byte 4 and patch from byte 5. If fewer than 6 arrive, decode minor as `byte[2] >> 4` and patch as `byte[2] & 0x0F`. Never mix the two decodes within one read. Take beta from byte 6 when 7 or more arrive, and 0 otherwise.

**Ordering rule for hosts.** A beta precedes the final release of the same patch, which is the opposite of what a plain numeric compare on the four fields gives, because final is encoded as 0. Compare `(major, minor, patch, beta == 0 ? 256 : beta)`.

## Host implementation

```c
uint8_t info[7];
/* SYS.PLATFORM on paged firmware. After a failed SYS.PROTOCOL_INFO probe, send
   bRequest 0x7F with wIndex VENDOR_INTF instead (firmware 1.1.6 and older). */
int n = libusb_control_transfer(handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
    paged ? 0x20 : 0x7F,
    0, paged ? ((0x01 << 8) | VENDOR_INTF) : VENDOR_INTF,
    info, 7, 1000);
if (n < 4) {
    return -1;  /* identification unavailable, do not read info[] */
}
int major = info[1];
int minor = (n >= 6) ? info[4] : (info[2] >> 4);
int patch = (n >= 6) ? info[5] : (info[2] & 0x0F);
int beta  = (n >= 7) ? info[6] : 0;
int rank  = beta ? beta : 256;   /* final outranks every beta of the patch */
```

## Build provenance: `SYS.BUILD_INFO`

The version number answers "what does this build promise to hosts". It cannot answer "which exact source built this binary", because two bench builds and a release can all legally report the same three numbers. The build stamp answers that second question.

Every build automatically embeds the output of `git describe --always --dirty` plus the build date. Nobody types it and nobody can forget it: `scripts/gen_build_info.cmake` runs on every build (build time, not configure time, so it never goes stale) and generates `build_info.h` in the build directory. A string like `v1.1.7-3-g1a2b3c4` means "3 commits past tag v1.1.7, exact commit 1a2b3c4". A `-dirty` suffix means the tree had uncommitted changes at build time, so the hash is approximate and the build should not be trusted as reproducible.

The stamp is readable three ways.

- `SYS.BUILD_INFO` (page 0x01, command 0x22, 1.1.6 opcode 0x80) returns a 64-byte blob from the running device. Bytes 0 to 47 are the describe string and bytes 48 to 59 are the build date `YYYY-MM-DD`, both ASCII and NUL-padded. Firmware older than the stamp STALLs opcode 0x80.
- `picotool info <file>.uf2` reads the same string from the UF2's binary info, identifying a build file with no device attached.
- `tools/dspi_test` prints it in the profile summary and checks it in the identity tests.

Two rules keep it honest. First, the stamp is for humans only; no software may gate, compare, or parse it, exactly as with build labels. Second, a release build must be made from a clean checkout of the tagged commit, so a published binary reports the bare tag with no offset and no `-dirty`. A release stamp that says anything else means the binary was not built from the tag.

Wire layout details live in `Documentation/Features/device_identification_spec.md`.

See `Documentation/Features/device_identification_spec.md` for the full `SYS.PLATFORM` and `SYS.SERIAL` reference.
