# DSPi AC-3 Decoding Specification

*Version 1.0, 2026-10-22. Firmware: wire format V33, preset slot V40, directory V23, vendor commands 0x02, 0xCF and notify event 0x0E. Decoding is RP2350 only; RP2040 detects and mutes bitstreams and round-trips the settings. Hardware-untested with a real AC-3 source at the time of writing (host-verified bit-exact against upstream liba52 and ffmpeg).*

This document contains what an application developer needs to integrate AC-3 decoding: the concept, the signal path, every command byte layout, the status packet, the notify event, persistence behaviour and recommended UX. The firmware design is in `Documentation/current_architecture.md` under "IEC 61937 Bitstream Detection" and "AC-3 Decoding"; the engineering history is in `ac3_dts_decoding_plan.md`.

---

## 1. What it does

An AC-3 (Dolby Digital compatible) bitstream arriving on the S/PDIF input is detected, decoded and delivered to the DSP pipeline as six input channels in this fixed order:

| Input | Channel |
|---|---|
| 0 | L |
| 1 | R |
| 2 | C |
| 3 | LFE |
| 4 | Ls |
| 5 | Rs |

A stream that carries fewer channels (2/0 stereo, 3/0, 2/1, and so on) fills the inputs it has and leaves the others silent, so a matrix configured for a stereo source keeps working when a 5.1 stream arrives, and a 5.1 matrix plays a stereo stream on L and R only. A single surround channel (2/1, 3/1) is delivered to both Ls and Rs. Dual mono and mono streams land on C.

While decoding, the S/PDIF input reports six active inputs; otherwise two. Every input keeps its own preamp, PEQ and metering, and the leveller, matrix and everything downstream see the channels exactly as they see an eight-channel USB or ADAT source.

The decode adds a fixed delay of one AC-3 frame, 1536 samples (32 ms at 48 kHz, 34.8 ms at 44.1 kHz), relative to the carrier. The delay is the same on every channel and every output, so inter-channel and inter-output alignment are unaffected.

Any other bitstream (DTS, pause bursts with no data, unknown types) and any AC-3 stream while decoding is disabled are muted: the input is zero-filled so an encoded stream never plays as noise. A plain PCM signal on the same input passes through unchanged.

### What it does not do

- DTS, E-AC-3 and the HDMI-only formats are not decoded (DTS is planned).
- AC-3 at 32 kHz is not decoded (the receiver does not lock at that rate).
- There is no pass-through of the raw bitstream to an output.
- Full scale is the AC-3 coded level; the LFE channel is delivered as coded, without the +10 dB playback offset, which the matrix gain provides if wanted.

## 2. Settings

Three settings, one command. All are per-device settings that follow `output_config_mode` (section 5).

| Setting | Values | Default | Meaning |
|---|---|---|---|
| `enabled` | 0, 1 | 1 | 0 mutes AC-3 like any other bitstream |
| `drc_mode` | 0 off, 1 full, 2 half | 1 | How the stream's dynamic range words (`dynrng`) are applied: ignored, as sent, or half the gain change in dB |
| `dialnorm_mode` | 0 ignore, 1 apply | 1 | Whether the stream's dialnorm field normalises dialogue to -31 dBFS. A stream coded at dialnorm -27 is attenuated 4 dB when applied |

Dialnorm applied is what consumer decoders do, so levels match an AV receiver and stay consistent across programmes. Ignoring it plays the coded level, which is what a measurement setup usually wants. The `dialnorm` byte of the status packet (section 3) reports the stream's field either way.

DRC "full" is the AC-3 default (Dolby "line mode"). "Half" is the common custom setting. The `compr` (RF mode, heavy) word is never applied.

## 3. Commands

All commands use the vendor interface like every other DSPi command (`bmRequestType` 0x40 for SET with a data stage, 0xC0 for GET; `wIndex` = vendor interface). They work on every control transport (USB, UART, I2C).

### 3.1 REQ_AC3_CONFIG (0x02)

**SET**: 4-byte payload. Applies immediately. Out-of-range values take the defaults. Each field change is announced through `NOTIFY_EVT_PARAM_CHANGED` against the wire offsets in section 4.

```
byte 0  enabled         0 or 1
byte 1  drc_mode        0 off, 1 full, 2 half
byte 2  dialnorm_mode   0 ignore, 1 apply
byte 3  reserved        0
```

**GET**: returns the same 4 bytes (reserved reads 0). Available on RP2040 too, where the settings round-trip without effect.

### 3.2 REQ_GET_SPDIF_BITSTREAM_STATUS (0xCF)

GET, 16 bytes, little-endian. Reflects the S/PDIF input whatever the active source is (all zero while the receiver is inactive).

```
byte 0      format          0 PCM, 1 AC-3, 2 DTS, 3 other bitstream
byte 1      data_type       IEC 61937 Pc bits 4:0 of the last burst; 0xFF before any burst
byte 2      muted           1 while the input is zero-filled (a bitstream that is not decoded)
byte 3      flags           bit 0 channel-status non-audio bit set
                            bit 1 last burst's Pc error flag set
                            bit 2 AC-3 decode active (inputs 0-5 carry decoded audio)
bytes 4-5   burst_length    Pd of the last burst (bits for AC-3)
bytes 6-7   burst_period    carrier frames between the last two preambles (1536 for AC-3; 0 unknown)
bytes 8-11  burst_count     preambles seen since the receiver last locked
byte 12     decode_cpu_pct  decode time as a percent of one core, 1 s window
byte 13     dialnorm        the stream's dialnorm field, 0-31 (0 means -31 dB; n means -n dB)
byte 14     acmod_flags     liba52 output mode (bits 3:0: 0 dual mono, 1 mono, 2 stereo, 3 3/0,
                            4 2/1, 5 3/1, 6 2/2, 7 3/2, 10 Dolby Surround) | 0x10 when LFE present
byte 15     late_blocks     blocks decoded after their playback slot, saturating at 255
```

`format` is PCM whenever no bitstream is present. While AC-3 decodes, `format` is 1, `muted` is 0 and flags bit 2 is set. A DTS stream reads `format` 2, `muted` 1.

### 3.3 Other relevant commands

- `REQ_GET_SPDIF_RX_STATUS` (0xE2) is unchanged: lock state, rate, FIFO.
- `REQ_SET_INPUT_SOURCE` (0xE0) selects the S/PDIF input as before; decoding needs no mode switch.
- `REQ_GET_STATUS` (0x50) reports the six decoded inputs' peaks in their input slots like any multichannel source.

## 4. Wire format (bulk parameters)

Wire format V33 appends an 8-byte section at offset 6136, total 6144 bytes:

```
offset  size  field               encoding
6136    1     enabled_p1          enabled + 1 (1 off, 2 on)
6137    1     drc_mode_p1         drc_mode + 1 (1 off, 2 full, 3 half)
6138    1     dialnorm_mode_p1    dialnorm_mode + 1 (1 ignore, 2 apply)
6139    5     reserved            0
```

Every field is stored plus one so that a host built before V33 that pushes zeros here leaves the live settings alone (0 = absent). On apply the section follows `output_config_mode`: it is applied only when the push applies pins (with-preset mode), exactly like the output limiter section. Out-of-range values take the defaults.

`NOTIFY_EVT_PARAM_CHANGED` events for these settings carry the offsets above with size 1.

## 5. Persistence

The settings are stored in two places and sourced by `output_config_mode`, the same rule the output limiter follows:

| Mode | Loads from | Saves to |
|---|---|---|
| With preset (1) | the preset slot (V40+; older slots give the defaults) on every load; defaults on an empty slot or factory reset | `REQ_PRESET_SAVE` writes the live settings into the slot |
| Independent (0) | the directory block at boot only; preset load, active-slot delete and factory reset leave the live settings alone | `REQ_SAVE_OUTPUT_CONFIG` (0x52) snapshots the live settings into the directory |

A preset save always stores the live settings in the slot whatever the mode, so a later switch to with-preset mode sees them.

Directory V23 appends a 4-byte block (`version`, `enabled`, `drc_mode`, `dialnorm_mode`); a V22 directory migrates by a prefix copy and the new block reads as "never saved", which seeds the defaults. Preset slot V40 appends 3 bytes; V21 to V39 slots still load.

## 6. Notify events

- `NOTIFY_EVT_SPDIF_FORMAT` (0x0E, 8 bytes, see `notification_protocol_v2_spec.md` section 3.10): pushed when the input's format, mute or decode state changes. A host that shows a source badge ("PCM", "AC-3 5.1", "DTS (muted)") needs nothing else; `data_type`, `muted` and `bs_flags` bit 2 are in the packet, and `acmod_flags` is one 0xCF read away.
- `NOTIFY_EVT_PARAM_CHANGED` (0x02) for each config field, as in section 4.
- `NOTIFY_EVT_INPUT_FORMAT` (0x05) is **not** pushed when the input count changes between 2 and 6 for decode; it remains the USB alt event. Hosts should derive the S/PDIF channel count from 0xCF flags bit 2.

## 7. Recommended UX

- Show the detected format next to the S/PDIF input selector, from `NOTIFY_EVT_SPDIF_FORMAT`, with `acmod_flags` turned into a layout string ("5.1", "2.0", "3/1 + LFE").
- Offer the three settings in the S/PDIF input section. Dialnorm and DRC deserve a one-line explanation each; most users should leave both at the defaults.
- While decode is active, lay the mixer out with six inputs, as for a six-channel USB alt, and name them L, R, C, LFE, Ls, Rs unless the preset carries channel names.
- When `muted` is 1 and `format` is 2 or 3, say that the stream is a format the device does not decode rather than showing a silent meter.
- `late_blocks` and `decode_cpu_pct` are diagnostics; a non-zero late count after a preset save is expected (flash writes stall the decoder for about one frame).

## 8. Limits and gotchas

- Decode latency is one frame; for lip-sync, the device's input delay is 32 ms longer on AC-3 than on PCM. Hosts that expose a lip-sync offset should mention this.
- The first decoded audio starts 1536 carrier frames after the first preamble, and the mute releases 4096 frames (85 ms at 48 kHz) after the last preamble when a stream ends.
- A source that sends byte-swapped AC-3 words (sync word 0x770B) is accepted.
- The decoder's dither for zero-bit mantissas differs from other decoders', so a bit-exact comparison against an AV receiver's output is not meaningful; levels and content are.
