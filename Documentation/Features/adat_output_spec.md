# ADAT Bulk Output

*Last updated: 2026-10-11*

## Overview

The ADAT bulk output streams all 8 main output channels (post-matrix, post
per-output EQ/crossover, post gain/volume/mute, post delay) as a single ADAT
lightpipe signal on one GPIO. It is intended as a "bulk" monitoring/expansion
tap of the finalized output channels, running concurrently with the four
SPDIF/I2S output slots and PDM; enabling it changes nothing about the
existing outputs.

| Property | Value |
|---|---|
| Platform | RP2350 only (RP2040 lacks 8 output channels) |
| Channels | 8 (output channels 1..8, the SPDIF slot channels) |
| Sample rates | 44.1 kHz and 48 kHz only |
| Bit depth | 24-bit (same clamp + scale as the SPDIF outputs) |
| Rates above 48 kHz | Stream auto-suspends; auto-resumes when the rate returns |
| ADAT user bits | Always 0 (no timecode/MIDI/S-MUX signalling) |
| S/MUX | Not supported (future work) |
| Default GPIO | 12 (`PICO_ADAT_PIN`) |
| Signal | 3.3 V logic NRZI at 256 x Fs; drive an optical TOSLINK transmitter rated for ~12.3 Mbit/s |

## Signal path and alignment

The tap point is after every per-output processing stage and after the same
gain composite the SPDIF/I2S slots receive (host volume x master volume x
preset mute x per-output matrix gain, including mute and delay). What goes to
ADAT channel N is bit-identical in level to what goes to slot output N.

Alignment guarantees:

- ADAT cannot drift against the output slots. Its PIO clock is 256 x Fs, the
  same as SPDIF TX, and it always runs the identical nominal divider. Input
  clock tracking trims `sys_clk` itself (the soft VCXO), which moves ADAT and
  every slot together.
- ADAT runs a constant 96-sample lead cushion (2 ms at 48 kHz) relative to the
  slot outputs, counted in DMA words consumed. On the wire the offset is 95
  frames behind an S/PDIF slot 0 or 93 behind an I2S slot 0, because each
  counter also includes the 8 words prefetched into its PIO FIFO (1 ADAT
  frame, 2 S/PDIF frames, 4 I2S frames). It is constant per slot-0 type, so
  nothing drifts. Every output restart anchors it against slot 0's measured
  ring fill, exact to within one frame (ADAT is on another PIO block and
  cannot join the slots' synchronized start).
- Every slot underrun re-anchor is mirrored into ADAT in the same call, before
  the next block is pushed, so the offset survives underruns exactly, whatever
  the input source or USB stream state. A keep-alive tops ADAT up with silence
  while the slots are dry, including through flash writes. The accounting is in
  `current_architecture.md`, "ADAT Bulk Output".

## Vendor commands

Commands follow the standard vendor EP0 conventions (also reachable over the
UART and I2C control transports through the shared dispatcher). All SET
commands carry their payload in `wValue` and return a single status byte from
the shared `PIN_CONFIG_*` set (config.h): `0x00` SUCCESS, `0x01` INVALID_PIN,
`0x02` PIN_IN_USE, `0x03` INVALID_OUTPUT, `0x05` INVALID_PARAM.

| Command | ID | Direction | wValue | Response |
|---|---|---|---|---|
| `REQ_SET_ADAT_ENABLE` | 0xCA | SET | 0 = disable, 1 = enable (low byte) | 1 status byte |
| `REQ_GET_ADAT_ENABLE` | 0xCB | GET | - | 1 byte: configured enable (0/1) |
| `REQ_SET_ADAT_PIN` | 0xCC | SET | GPIO number (low byte) | 1 status byte |
| `REQ_GET_ADAT_PIN` | 0xCD | GET | - | 1 byte: configured GPIO |
| `REQ_GET_ADAT_STATUS` | 0xCE | GET | - | 8-byte `AdatStatus` |

Behavioral notes:

- `REQ_SET_ADAT_ENABLE` with the current value is a no-op success. Enabling
  validates the configured pin first (INVALID_PIN / PIN_IN_USE on conflict).
  Disabling always succeeds.
- `REQ_SET_ADAT_PIN` validates the GPIO (23..25 and out-of-range rejected,
  conflicts with any owned pin rejected) and may be issued while enabled; the
  stream is re-routed under a muted restart. `wValue = 0xFF`
  (`PIN_RESET_TO_DEFAULT`, the escape hatch shared by every single-pin SET
  command) resets to the platform default pin; 0 means GPIO 0. (The
  flash/bulk "0 = default" decoding is a persistence-layer migration
  convention only and is unchanged.)
- Config changes apply from the main loop inside the same muted
  pipeline-restart bracket used by output type/pin switches; expect the brief
  global mute-and-resync that any output reconfiguration produces.
- On RP2040 both SETs return `0x03` (INVALID_OUTPUT); the GETs return zeros.
- While ADAT is enabled its GPIO is owned: every other pin-assignment command
  rejects that pin with PIN_IN_USE, and vice versa.

### `AdatStatus` (REQ_GET_ADAT_STATUS, 8 bytes, packed, little-endian)

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `enabled` | Configured enable (persisted intent) |
| 1 | u8 | `active` | Stream currently running |
| 2 | u8 | `pin` | Configured data GPIO |
| 3 | u8 | `rate_ok` | Current sample rate is 44.1/48 kHz |
| 4 | u16 | `resync_count` | Stream restarts since boot (every pipeline reset increments this; a growing number during steady playback is normal only if resets occur) |
| 6 | u16 | `slip_count` | Emergency local resyncs since boot; should stay 0, nonzero indicates a stalled main loop or DMA fault |

`enabled=1, active=0, rate_ok=0` is the auto-suspended state (rate above
48 kHz); the stream resumes automatically, no host action needed.

## Notify event

`NOTIFY_EVT_ADAT_STATE` (0x08), pushed on every stream start/stop including
rate-policy auto-suspend/resume. 8-byte v2 packet:

```
[0] 0x02 (version)   [1] 0x08 (event)   [2] 0 (flags)   [3] seq
[4] enabled          [5] active         [6] pin         [7] 0
```

Apps that track ADAT state should consume this instead of polling 0xCE.
Config changes made via 0xCA/0xCC additionally produce the usual
`PARAM_CHANGED` events for the `adat_config` wire fields.

## Bulk params (REQ_GET/SET_ALL_PARAMS)

`WIRE_FORMAT_VERSION` is now **17** (was 16). Hosts must fetch the new layout;
apply rejects any other version. The new section is the final member of
`WireBulkParams`:

```c
typedef struct __attribute__((packed)) {
    uint8_t enabled;      // 0/1 configured enable
    uint8_t pin;          // data GPIO; 0 = platform default (12)
    uint8_t reserved[6];  // future (user bits, S/MUX); write 0
} WireAdatConfig;         // 8 bytes; sizeof(WireBulkParams) = 5872
```

Collect returns the live config (zeros on RP2040). Apply clamps `enabled`,
maps `pin == 0` to the default, rejects invalid/conflicting pins by keeping
the live pin, and defers the hardware apply exactly like the vendor commands.

## Persistence

ADAT config persists identically to the other physical IO config (output
types, I2S BCK/MCK, SPDIF RX pin) and honors `output_config_mode`
(REQ 0x98/0x99):

- **WITH_PRESET (1, default):** `adat_enabled`/`adat_pin` are stored in each
  preset slot (`SLOT_DATA_VERSION` 23) and applied on preset load. Slots saved
  before V23 leave the current device-level ADAT config untouched.
- **INDEPENDENT (0):** ADAT config lives in the device-global directory block
  (`PresetDirectory` V8, `FlashOutputConfig`), is applied at boot, and preset
  load / factory reset never touch it. `REQ_SAVE_OUTPUT_CONFIG` (0x52) stores
  the live ADAT config with the rest of the IO block.

`adat_pin == 0` in flash means "unset, use the platform default". Old
firmware versions' presets and directories migrate automatically; a V23
preset loads on old firmware as its version-appropriate prefix (ADAT fields
ignored).

## App integration patterns

Enable flow:

1. (Optional) `REQ_SET_ADAT_PIN` if the hardware uses a non-default GPIO.
2. `REQ_SET_ADAT_ENABLE` wValue=1; check the status byte.
3. Wait for `NOTIFY_EVT_ADAT_STATE` with `active=1` (or poll 0xCE). If
   `rate_ok=0` the device is above 48 kHz; the stream starts when the rate
   drops, with another 0x08 event.
4. Persist: in WITH_PRESET mode save the preset (0x91); in INDEPENDENT mode
   send `REQ_SAVE_OUTPUT_CONFIG` (0x52).

Rate handling: apps do not need to manage the 48 kHz restriction. Selecting
96 kHz suspends the stream (event with `active=0, enabled=1`); returning to
44.1/48 kHz resumes it automatically.

Platform detection: check the platform ID (RP2350 = 1) from the device ident
before exposing ADAT UI, or treat `0xCA` returning INVALID_OUTPUT as "not
available".

Level mapping: ADAT sample N is the same 24-bit value the SPDIF outputs
carry: float clamp to [-1, +1] then scale by 8388607.

## Wire format (for receiver/analyzer implementers)

Each sample period is one 256-bit frame, NRZI encoded (a 1 is a level
transition), bit clock 256 x Fs (12.288 MHz at 48 kHz):

```
[1] [0000000000] [1] [u3 u2 u1 u0]          16-bit sync pad, user bits = 0
then for each channel 0..7 (30 bits each):
[1][b23..b20] [1][b19..b16] [1][b15..b12] [1][b11..b8] [1][b7..b4] [1][b3..b0]
```

Samples are MSB-first, two's complement 24-bit. The 10-zero run is the only
place the stream is transition-free for more than 4 bits, which is what
receivers lock onto. The encoder was verified by exhaustive host-side
decode/NRZI/zero-run analysis (4096-frame corpus including both full-scale
extremes); no hardware ADAT receiver has confirmed interop yet. The 2026-07-13
LUT rewrite (below) was proven bit-identical to that original encoder by a
host-side differential test: 4 million streamed frames (random plus edge-case
samples, chained NRZI line level across frames), both silence-template entry
levels, all 256 byte tokens in all 3 byte positions at both entry levels, and
the float clamp/convert path including +/-1.0, denormals and infinities.

## Firmware internals (summary)

- Engine: `firmware/DSPi/adat_output.c`; NRZI is encoded on the CPU (line-level
  state carried across frames), so the PIO program is a single `out pins, 1`
  at 256 x Fs and the clkdiv is identical to the SPDIF TX divider. DMA
  channels 13 (data) + 14 (control) run a free-running 960-frame (30 KB) ring
  with an IRQ-less chained wrap.
- Encoder (2026-07-13, LUT-driven; bit-identical to the original stuff +
  prefix-XOR implementation): each PCM byte maps to a fixed 10-bit stuffed
  token `[1][hi nibble][1][lo nibble]`, and NRZI is linear, so the
  entry-level-1 line pattern is the bitwise complement of the entry-level-0
  pattern. `adat_token_lut[256]` (uint16, BSS, built in `adat_output_init()`)
  holds the pre-NRZI'd entry-level-0 token per byte; `adat_encode_frame()`
  XORs each token with a level mask (0 or 0x3FF) chained through the token
  LSB (the line level during the token's last bit), then packs the header +
  eight 30-bit chunks with the same fixed shifts as before. The 16-bit sync
  header is precomputed per entry level (`adat_hdr_nrzi[2]`/`adat_hdr_exit[2]`).
  Table cost: 528 bytes BSS. Encoder codegen (Cortex-M33, -O3): 311 -> 231
  instructions per frame, and the per-frame float clamp/convert is gone
  entirely (see next bullet). Silence templates are built at init through the
  same encoder.
- Single S24 conversion point, gated on ADAT activity: while ADAT is active
  the pipeline converts `buf_out[0..7]` float -> clamped S24 **in place**
  once per block (`output_s24.h`, `output_block_to_s24_inplace()`), after
  the last float consumer (EQ, gain, loudness, delay, peak metering) and
  before the S/PDIF slot interleave. Both the slot interleave
  (`output_pair_interleave_s24()`, pure integer) and the ADAT encoder read
  the integer form (`out_s24_t`, a may_alias int32), eliminating the
  previous duplicate conversion (8 channels x Fs = 384,000 extra
  clamp/scale/convert per second at 48 kHz) with zero additional RAM. While
  ADAT is inactive the staging pass is skipped entirely and each slot pair
  runs the original fused convert+interleave
  (`output_pair_convert_interleave()`, rows stay float), so the ADAT-off
  steady state pays no extra memory pass. Both modes produce bit-identical
  slot bytes. The mode is snapshotted once per packet on Core 0
  (`adat_output_is_active()`, RAM-resident) and shared with Core 1 via
  `core1_eq_work.finalize_s24`, and the same snapshot gates the ADAT push,
  so the two cores and the encoder can never disagree within a packet
  (ADAT resync runs on the same thread as the pipeline). In dual-core mode
  Core 0 converts rows 0-1 and Core 1 converts rows 2-7; ADAT reads them
  only after the `work_done` handshake. Conversion runs even when a slot
  pool is starved (no buffer taken), so ADAT always sees converted samples.
- The DSP pushes encoded frames from `process_input_block()` after the slot
  buffer gives; the blocking give bounds the ring lead, so overwrite of
  unplayed frames is structurally impossible while the DMA runs.
- Rate policy hooks in `perform_rate_change()`; stream restarts in
  `complete_pipeline_reset()` Phase 6 and `enable_outputs_in_sync()`.
- RP2040 compiles the engine out entirely (zero RAM/flash cost).

## Planned ADAT input: what this engine shares (design notes, 2026-07-13)

Decisions made while optimizing the output encoder, recorded so the future
input implementation does not re-derive them:

- The encode token LUT is deliberately **not** shared with decode, because
  decode does not need a LUT at all: NRZI decode of a captured word is
  `x ^ (x >> 1)` (plus the carried entry bit), and destuffing a decoded
  10-bit token back to a byte is two masks and a shift
  (`((t >> 1) & 0xF0) | (t & 0x0F)`). A 1024-entry reverse table would cost
  RAM to save nothing.
- What the input should reuse directly: the frame geometry and sync header
  (`ADAT_SYNC_HEADER`, 16 + 8x30 = 256 bits per frame, token layout above),
  the 256 x Fs bit-clock relationship, and the divider plumbing patterns.
  In slave mode, follow the SPDIF-input model (RX SM + DMA, rate detect,
  output clock servo trimming the TX dividers, which already rate-locks this
  ADAT output via `adat_output_servo_divider()`). In master mode the device
  clock is nominal and the existing nominal-divider path already applies.
- Hybrid PIO NRZI (encoding NRZI in the PIO program for nominal-clock modes)
  was evaluated and rejected: the servo dithers between adjacent, possibly
  odd, divider values, PIO NRZI at 2 cycles/bit would halve divider
  resolution, and switching encode modes mid-stream would need a resync or a
  ring re-encode. With NRZI baked into the token LUT the CPU cost of NRZI is
  already zero, so a two-mode encoder has no remaining upside.
