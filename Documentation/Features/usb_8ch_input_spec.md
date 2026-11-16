# 8-Channel USB Audio Input (RP2350)

*Last updated: 2026-06-24*

*Note (2026-11-16): the vendor command names and numbers in this document are those of firmware 1.1.6. Firmware 1.1.7 uses the paged protocol described in `../commands.md`. The old to new mapping is in `vendor_protocol_migration_notes.md` and `../protocol/legacy_map.json`.*

> **Superseded by the unified channel model (firmware wire V16 / slot V21).** This spec
> describes the original 8-channel-only design (alt 3 = 8ch, tail-append persistence V15/V20,
> "master EQ" channels 0/1). The current firmware instead exposes **2/4/6/8-channel** input
> alts, makes **every input a first-class channel with its own PEQ + metering** (no "master"),
> and uses a **direct** V16 wire / V21 slot layout. For the authoritative description see
> `Documentation/current_architecture.md` → "Multichannel USB Input + Per-Input EQ/Metering",
> and the app-side guide `Documentation/Features/8-channel-usb-input.md`. The alt/format-menu
> mechanics and 48 kHz lock below still apply (now generalized to 4/6/8 channels).

## Summary

RP2350 builds expose an optional **8-channel USB audio input** format alongside the
existing stereo formats. The host selects it the normal USB way — as another playback
format on the DSPi audio device. The full set of selectable formats on RP2350:

| Format | Channels | Bit depth | Sample rates | AS alt |
|--------|----------|-----------|--------------|--------|
| Stereo 16-bit | 2 | 16-bit | 44.1 / 48 / 96 kHz | 1 |
| Stereo 24-bit | 2 | 24-bit | 44.1 / 48 / 96 kHz | 2 |
| **8-channel** | **8** | **16-bit** | **48 kHz only** | **3** |

RP2040 is **stereo-only** (alts 0-2); it has just 2 SPDIF instances (4 output channels)
and cannot emit 8 discrete channels, so its descriptor is unchanged.

The 8 USB input channels feed a widened, fully re-routable matrix mixer into the four
SPDIF pairs (8 output channels), each with its own per-output EQ, gain, and delay. This
makes the device an 8-channel DSP DAC suitable for active multi-way crossovers, multichannel
room correction, and surround processing.

## Host behavior

- Selecting the 8-channel / 48 kHz format activates AS alternate setting 3. The device
  locks the sample rate to 48 kHz while alt 3 is active (a `SET_CUR` sampling-frequency
  request for 44.1 or 96 kHz is stalled at EP0).
- The stereo formats are unaffected; the host can switch between any of the formats at will.
- Channel order follows the declared 7.1 channel config (`wChannelConfig = 0x063F`:
  FL FR FC LFE BL BR SL SR). USB input channel *i* maps to matrix input *i*.

### Host-compatibility note

The device declares a single Input Terminal (ID 1) and Feature Unit sized for the channel
**superset** (`bNrChannels = 8`), while each AudioStreaming alt carries its own Format Type I
`bNrChannels`. This is legal UAC1 and is the simplest topology that lets a host which keys
channel count off the terminal expose all formats. Some hosts may key channel count off the
terminal rather than the per-alt format; **this should be validated on real Windows 10/11 and
macOS hardware** (confirm both 2-channel and 8-channel formats appear, that selecting 8ch
activates alt 3 + 48 kHz, and that stereo playback is not forced to 8 channels). If a host
mishandles the shared terminal, the contingency is a dedicated second Input Terminal +
Feature Unit + Output Terminal for the 8-channel path.

## DSP behavior

- In 8-channel mode the inherently-stereo **master chain is bypassed**: loudness
  compensation, Master EQ (CH_MASTER_LEFT/RIGHT), the volume leveller, crossfeed, and
  master-peak metering do **not** run. The 8 inputs flow straight into the matrix.
- Per-output processing is unchanged: each of the 8 SPDIF output channels keeps its own
  EQ bank, gain, delay, and crossover, applied after the matrix. With a 1:1 matrix routing
  this is effectively per-input-channel processing.
- Per-input preamp applies to all 8 channels (`REQ_SET_PREAMP_CH`, channel index 0-7).
- **Default routing is unchanged** (stereo pass-through to SPDIF 1). To use all 8 channels,
  the host app configures the matrix (e.g. a 1:1 input→output mapping) and saves it in a
  preset; the firmware does not auto-populate an 8-channel routing.
- Master-peak / clip reporting (`global_status.peaks`) is not produced in 8-channel mode
  (documented limitation; the metering is a stereo-bus measurement).

## Vendor commands

No new vendor commands. On RP2350 the existing matrix and preamp commands accept the wider
input range:

- `REQ_SET_MATRIX_ROUTE` / `REQ_GET_MATRIX_ROUTE` — input index **0-7** (was 0-1).
- `REQ_SET_PREAMP_CH` / `REQ_GET_PREAMP_CH` — channel index **0-7** (was 0-1).

## Persistence & wire format (host-app dependency)

The widened matrix (8 inputs) and per-channel preamp (8 channels) are persisted and synced
via **append-only tail sections**, so all existing offsets are stable and old presets / old
host apps keep working.

- **Flash preset:** `SLOT_DATA_VERSION` 19 → **20**. Inputs 0/1 remain in the base layout;
  inputs 2-7 (matrix crosspoints + preamp) are an appended tail. Pre-V20 presets load with
  inputs 2-7 defaulted (disabled / 0 dB). On RP2040 the ext section is compiled out, so a V20
  slot is byte-identical to V19.
- **Wire format (`WireBulkParams`):** `WIRE_FORMAT_VERSION` 14 → **15**. A new
  `WireInputExtConfig` tail section (464 bytes: `crosspoints[6][9]` + `preamp_db[6]`) carries
  inputs 2-7. The packet grows from 3664 to 4128 bytes, so `WIRE_BULK_BUF_SIZE` doubled to
  8192. Header `num_input_channels` reports 8 on RP2350.
  - Legacy (V14-and-earlier) payloads decode with all existing offsets intact; the
    `REQ_SET_ALL_PARAMS` path applies inputs 2-7 only when the payload is full V15 size,
    leaving them untouched otherwise.

**Host app:** to drive/read 8-channel routing, the desktop/web app must mirror the V15
`WireBulkParams` layout (including the new tail section), enlarge its transfer buffer to
8192 bytes, and present matrix-input indices 0-7 / preamp channels 0-7 on RP2350. The
firmware stays forward-compatible for the existing stereo fields, so an un-updated app
continues to work for 2-channel control.

## USB / electrical

- Iso OUT endpoint max packet (`AUDIO_EP_MAX_PKT`) rises to **788** on RP2350
  (48 frames × 8 ch × 2 B = 768 + one jitter frame), under the 1023-byte full-speed
  isochronous ceiling. The USB ring slot (`USB_RING_MAX_PKT`) matches. RP2040 stays at 582.
- `USB_BCD_DEVICE` bumped (0x0201 → 0x0202) so Windows re-reads the descriptor.
- Asynchronous feedback (EP 0x82, 10.14 format) is unchanged — it is rate-based and
  independent of channel count.

## CPU

In 8-channel mode all 8 SPDIF output channels can carry independent audio simultaneously
(vs stereo where outputs 2-7 are often mirrors or disabled), so Core 0 (matrix up to 8×9,
partly offset by the skipped master chain) and the Core 1 EQ worker (outputs 2-7) both see
higher load. At the locked 48 kHz this is the relaxed rate case and is expected to fit, but
**should be confirmed on hardware** via the CPU-load metering (`cpu0_load_q8`) and Core 1
timing before release.

## Verification

1. Build both platforms clean; the descriptor and struct-size `_Static_assert`s gate the
   build. RP2350 `usb_config_descriptor` is 265 bytes; RP2040 is 207 (unchanged).
2. Enumerate (`lsusb -v` / USBView) on RP2350: confirm alts 0-3, alt 3 Format Type I =
   8ch/16-bit/48 kHz only, Input Terminal `bNrChannels=8` / `wChannelConfig=0x063F`, iso OUT
   max-packet 788. Diff the RP2040 descriptor against the pre-change dump to confirm it is
   byte-identical (apart from `bcdDevice`).
3. Host (Windows + macOS): both 2-channel and 8-channel formats appear; selecting 8ch sets
   alt 3 and locks 48 kHz; stereo formats unaffected.
4. Drive an 8-channel test signal, route each input to a distinct output via the matrix, and
   confirm correct per-input routing/gain/phase, **inter-output sample alignment** across the
   SPDIF instances, and clean 2ch↔8ch transitions (no stale audio, no slot drift). Re-verify
   alignment after preset/rate switches while in 8-channel mode.
5. Persistence: save/load a V20 preset in 8-channel mode; load a pre-existing V19 stereo
   preset and confirm inputs 2-7 are silent with inputs 0/1 correct; round-trip
   `REQ_GET/SET_ALL_PARAMS` at V15 with the updated host app.
