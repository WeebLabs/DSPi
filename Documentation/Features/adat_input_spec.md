# ADAT Input

*Last updated: 2026-11-16 (vendor commands on the paged protocol of firmware 1.1.7)*

## Overview

The ADAT input is a selectable 8-channel input source: one TOSLINK receiver
delivers ADAT lightpipe audio into the DSP pipeline, where the 8 channels
become input channels 0..7 of the unified channel model (per-input PEQ,
metering, and the full 8x9 routing matrix all apply, exactly as they do for
8-channel USB input). Together with the ADAT output it turns DSPi plus an
outboard ADC/DAC box (for example a Behringer ADA8200) into a complete
analog-in to analog-out processor.

| Property | Value |
|---|---|
| Platform | RP2350 only (RP2040 lacks the channels, PIO and DMA budget) |
| Input source value | 3 (`INPUT_SOURCE_ADAT`, set with `INPUT.SOURCE`) |
| Channels | 8 (input channels 0..7) |
| Sample rates | 44.1 kHz and 48 kHz only |
| Bit depth | 24-bit |
| S/MUX (96 kHz) | Not supported; an S/MUX stream never locks |
| Default GPIO | None free; pin ships unset (0xFF) and must be assigned |
| Enable state | Disabled by default; must be enabled before it is selectable |
| Signal | 3.3 V logic NRZI at 256 x Fs from a TOSLINK receiver (~12.3 Mbit/s capable) |

Like the optional SPDIF inputs 2/3, a disabled ADAT input reserves nothing:
its stored pin is invisible to pin-conflict validation and the GPIO is only
used while ADAT is enabled. Input sources are exclusive; selecting ADAT
replaces USB/SPDIF/I2S as the audio source (USB control/config always works).

## Clock modes

The receiver hardware is identical in both modes; the modes differ only in
who owns the sample rate and whether the output clock servo runs.

### MASTER (0, default)

DSPi is the clock master of the whole chain. The intended wiring:

```
DSPi ADAT OUT  ->  ADC/DAC box ADAT IN   (box syncs to this stream)
DSPi ADAT IN   <-  ADC/DAC box ADAT OUT  (returns at the same clock)
```

Because the box locks to DSPi's ADAT output, the stream returning into the
ADAT input is already in DSPi's own clock domain. There is no rate
detection, no clock servo, and no drift by construction. The device is the
sample-rate authority via `INPUT.I2S_RX_RATE`, shared with I2S
master mode (same stored rate). Only 44.1/48 kHz are usable: selecting a
higher device rate parks the input (`rate_ok = 0` in the status packet,
outputs muted) until the rate returns to 44.1/48 kHz; this mirrors the ADAT
output's auto-suspend and needs no host recovery action.

### SLAVE (1)

External gear owns the clock (for example DSPi hangs off an audio
interface's ADAT output). Acquisition probes the exact 48 kHz and 44.1 kHz
decoder cell timings in turn (10 ms dwell each) and accepts a candidate only
after eight consecutive valid frame headers at the fixed frame stride. This
avoids inferring the source rate from a corrupt wrong-cell decoded stream.
Once locked, 32 ms DMA-word timing windows measure the fine source-clock
offset; the pipeline follows the detected family through the standard
deferred rate-change path, and every output (SPDIF and I2S slot dividers, the
ADAT output divider, and MCK) is servo rate-matched to it, exactly like SPDIF
input. An 8-16 s dual-anchor long window gives the servo a ~0.1 ppm reference.

Slave mode is never parked: `rate_ok` is always 1 (parking is a master-only
concept). If the device rate is above 48 kHz when ADAT slave becomes the
active source (the source switch keeps the previous rate), acquisition still
runs to LOCKED; the deferred rate change then retunes the pipeline to the
detected 44.1/48 kHz rate while the switch-in mute holds, and only afterwards
does the synchronized prefill enable every output slot together. The output
servo also holds off until the pipeline rate matches the detected rate, so no
output clock is slewed during that muted window.

Clock-slaving to ADAT is only in force while ADAT is the active input
source; switching to any other source restores nominal output dividers.

## Lock states

`state` in the status packet and in every 0x0B notify event:

| Value | State | Meaning |
|---|---|---|
| 0 | INACTIVE | Hardware stopped (ADAT is not the active source) |
| 1 | ACQUIRING | Slave: probing 48/44.1 kHz cell timing and verifying headers. Master: parked, waiting for a valid (<= 48 kHz) device rate |
| 2 | SYNCING | Rate known; searching for / verifying the frame sync header |
| 3 | LOCKED | Decoding audio; outputs enabled after the prefill completes |
| 4 | RELOCKING | Signal lost or rate changed; outputs muted, re-acquiring |

Loss detection is header-based: every frame's sync header is verified, and
two consecutive mismatches drop the lock (a dark or unplugged line decodes
as zeros and never matches). Lock acquisition after a loss is automatic.
Audio is muted in every state except LOCKED; on each transition into LOCKED
the outputs are drained, prefilled to 50% with real input audio, and started
in sync (the same handshake USB/SPDIF/I2S use), so inter-output-slot sample
alignment is preserved through every ADAT event.

## Vendor commands

The commands use the paged protocol of `../commands.md` and are also
reachable over the UART and I2C control transports. Each SET carries its
value in a 1-byte payload. There is no status byte: success is ACK, and a
refused SET STALLs with a result code readable from `SYS.LAST_RESULT` (UART
and I2C return the code in the response status byte). All four commands are
RP2350 only.

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `INPUT.ADAT_RX_ENABLE` | 0x03 | 0x40 | GET, SET | none (0) | GET 1, SET 1 | 0 = disable, 1 = enable |
| `INPUT.ADAT_RX_PIN` | 0x03 | 0x41 | GET, SET | none (0) | GET 1, SET 1 | GPIO, or 0xFF to clear. GET 0xFF = unset |
| `INPUT.ADAT_RX_CLOCK_MODE` | 0x03 | 0x42 | GET, SET | none (0) | GET 1, SET 1 | 0 = master, 1 = slave. GET returns the live mode |
| `INPUT.ADAT_RX_STATUS` | 0x03 | 0x43 | GET | none (0) | GET 20 | `AdatInputStatusPacket` |

| Result | Meaning |
|---|---|
| `VC_ERR_PIN_INVALID` | GPIO not usable for ADAT input, or enable with no pin set |
| `VC_ERR_PIN_IN_USE` | GPIO owned by another function |
| `VC_ERR_STATE` | Disable while ADAT is the live or pending source, or clear the pin while enabled |
| `VC_ERR_VALUE` | Enable or clock mode above 1 |
| `VC_ERR_UNKNOWN` | RP2040: the commands do not exist |

Commands 0x44-0x4F of the INPUT page are free for future ADAT input use.
Source selection itself is `INPUT.SOURCE` with value 3.

Behavioral notes:

- Order matters: set the pin (`INPUT.ADAT_RX_PIN`) BEFORE enabling
  (`INPUT.ADAT_RX_ENABLE`). Enabling with no pin returns `VC_ERR_PIN_INVALID`.
  Selecting source 3 while disabled or pin-less is refused by `INPUT.SOURCE`
  with `VC_ERR_STATE`.
- `INPUT.ADAT_RX_PIN` validates the GPIO and rejects conflicts with any owned
  pin (`VC_ERR_PIN_IN_USE`), with one deliberate exception: the pin MAY equal the ADAT
  output's pin. The receiver only listens (input-enable, never drives), so
  jumpering or simply sharing the TX GPIO is the supported zero-hardware
  loopback self-test (see below). Setting the current pin again is a no-op
  success. `0xFF` (clear) is accepted only while disabled, and is
  `VC_ERR_STATE` otherwise. If ADAT is the
  live source, a pin change re-routes under a brief muted restart.
- Disable through `INPUT.ADAT_RX_ENABLE` is REFUSED (`VC_ERR_STATE`) while ADAT is the active source or
  the target of a pending source switch; switch away first. Enable
  re-validates the stored pin.
- An `INPUT.ADAT_RX_CLOCK_MODE` SET is deferred: the main loop applies it,
  instantly when ADAT is not the active source, or under a muted receiver
  restart when it is. The GET reports the LIVE mode, so a just-sent change may read back stale for a few
  milliseconds; the apply emits the PARAM_CHANGED notify for the wire field.
- While ADAT input is enabled its GPIO counts as owned: other pin-assignment
  commands reject it with `VC_ERR_PIN_IN_USE` (the ADAT output keeps its own claim;
  the sharing exception is one-directional, input onto output).
- Rate selection in master mode uses `INPUT.I2S_RX_RATE`, the same stored
  rate I2S master mode uses. 96000 is accepted but parks the ADAT input
  (`rate_ok = 0`); hosts should grey out 96 kHz while source 3 is active in
  master mode. In slave mode the SET only stores the rate and does not apply
  it to ADAT (the wire rate is the authority).
- On RP2040 the four ADAT input commands do not exist and answer
  `VC_ERR_UNKNOWN`, and `INPUT.SOURCE` refuses source 3 with
  `VC_ERR_UNSUPPORTED`. 1.1.6 still round-tripped the stored clock mode
  there. Presets and bulk params carry the stored fields on both platforms
  as before.

### `AdatInputStatusPacket` (`INPUT.ADAT_RX_STATUS`, 20 bytes, packed, little-endian)

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `state` | Lock state (table above) |
| 1 | u8 | `clock_mode` | Live clock mode: 0 master, 1 slave |
| 2 | u8 | `enabled` | Configured enable |
| 3 | u8 | `pin` | Configured RX GPIO (0xFF = unset) |
| 4 | u8 | `rate_ok` | 0 = master mode parked because the device rate is above 48 kHz; always 1 in slave mode |
| 5 | u8 | `lock_count` | Locks since input start (saturates at 255) |
| 6 | u8 | `loss_count` | Lock losses since input start (saturates at 255) |
| 7 | u8 | `slip_count` | Losses caused by header verification failure (bit slips / signal loss) |
| 8 | u16 | `header_err` | Cumulative header mismatches, including isolated single-frame errors that did not drop the lock (wraps) |
| 10 | u16 | `reserved` | 0 |
| 12 | u32 | `detected_rate` | Hz. Slave: the snapped wire rate (valid from SYNCING). Master: the device rate. 0 when unknown/parked |
| 16 | u32 | `measured_hz` | Slave: raw measured wire rate from the last fast window; 0 in master mode |

A healthy locked link shows `header_err` static and `slip_count = 0`. A
slowly growing `header_err` with the lock held indicates marginal optical
signal (isolated bit errors); growing `slip_count`/`loss_count` indicates
signal dropouts or a rate change upstream.

## Notify event

`NOTIFY_EVT_ADAT_INPUT_STATE` (0x0B), pushed on every lock-state transition.
10-byte v2 packet:

```
[0] 0x02 (version)  [1] 0x0B (event)  [2] 0 (flags)  [3] seq
[4] state           [5..8] detected rate, Hz LE32 (0 unless LOCKED)
[9] clock_mode
```

Apps that track ADAT input state should consume this instead of polling
`INPUT.ADAT_RX_STATUS`. Config changes (enable/pin/clock mode) additionally produce the usual
`PARAM_CHANGED` events for the `WireInputConfig` fields below.

## Bulk params (`STORE.ALL_PARAMS`)

`WIRE_FORMAT_VERSION` is **24** (was 23). The three fields claim reserved
bytes of `WireInputConfig` (Section 15), so the total wire size is unchanged
(5900 bytes). All three use the 0 = "absent, keep live value" convention so
zeros pushed by an old host change nothing:

| Field | Encoding |
|---|---|
| `adat_input_pin` | 0 = absent/keep, else raw GPIO (0xFF is never on the wire; an unset pin collects as 0) |
| `adat_input_enabled_p1` | 0 = absent, 1 = disabled, 2 = enabled |
| `adat_clock_mode_p1` | 0 = absent, 1 = master, 2 = slave |

Apply validates the pin (invalid or conflicting pins keep the live value;
the ADAT-output-pin sharing exception applies), refuses to disable the live
source, and routes the clock mode through the same deferred apply as
`INPUT.ADAT_RX_CLOCK_MODE`.

## Persistence

Same dual-layer model as the other physical IO config:

- Per-preset: `SLOT_DATA_VERSION` **32** adds `adat_input_pin`,
  `adat_input_enabled`, `adat_input_clock_mode` to each preset slot. Slots
  saved before V32 read as pin unset / disabled / master, leaving the
  device-level config in charge.
- Device-level: directory version **15** mirrors the same three fields;
  applied at boot; upgrades from older directories seed the defaults.
- On preset load/boot, a stored pin that now conflicts falls back to unset
  (with enable forced off) instead of stealing the GPIO; a stored clock mode
  applies dormantly when ADAT is not the live source.

## App integration patterns

### Bring-up (typical, master mode with an ADA8200-class box)

1. `INPUT.ADAT_RX_PIN` with the wired GPIO. A STALL means it was refused.
2. `INPUT.ADAT_RX_CLOCK_MODE` with payload 0 (master, the default).
3. Configure the ADAT OUTPUT (`OUTPUT.ADAT_TX_PIN`, `OUTPUT.ADAT_TX_ENABLE`,
   see adat_output_spec.md) and connect it to the box's ADAT input; set the
   box to sync to ADAT.
4. `INPUT.ADAT_RX_ENABLE` with payload 1. A STALL means it was refused.
5. `INPUT.SOURCE` with payload 3.
6. Wait for `NOTIFY_EVT_ADAT_INPUT_STATE` with state 3 (LOCKED), or poll
   `INPUT.ADAT_RX_STATUS`. Audio flows once the prefill completes
   (milliseconds after lock).
7. Route the 8 input channels through the matrix (`ROUTE.MATRIX_ROUTE`)
   as with 8-channel USB; per-input PEQ and metering work unchanged.
8. Persist by saving the preset.

### Slave mode (DSPi hangs off an interface's ADAT out)

Same flow with `INPUT.ADAT_RX_CLOCK_MODE` payload 1. After selecting the source, the device
detects the wire rate (well under a second), retunes the pipeline if
needed, locks, and servoes all outputs to the external clock. Rate changes
at the source are followed automatically (brief mute + relock). Unplugging
the cable produces state 4 (RELOCKING) and mute; replugging relocks
automatically.

### Zero-hardware loopback self-test

Because the RX pin may equal the ADAT output pin, a board with ADAT output
enabled can validate the entire input path with no cabling:

1. Enable ADAT output (`OUTPUT.ADAT_TX_ENABLE`) on its pin.
2. Set the ADAT input pin (`INPUT.ADAT_RX_PIN`) to THE SAME GPIO; enable
   (`INPUT.ADAT_RX_ENABLE`); master
   mode; select source 3.
3. The receiver locks onto the device's own transmitted frames. Play audio
   into the outputs (USB is not the source, so use the signal generator,
   `MEAS.SIGGEN_CONFIG` and `MEAS.SIGGEN_START`, injected on output channels) and observe the input
   meters; assert state LOCKED, `header_err` static, `slip_count` 0.

The same test with a physical jumper between two GPIOs exercises real pad
transitions; with clock mode slave it exercises rate detection and servo
convergence (the servo settles at the nominal divider since both ends share
the clock).

### Detection / capability probing

Treat a missing `INPUT.ADAT_RX_ENABLE` bit in `SYS.PAGE_COMMANDS` for page
0x03 (or the platform ID = RP2040) as "ADAT input unavailable". There is no free default GPIO, so a host UI must always
collect a pin from the user before offering enable.

### Channel names and metering

While source 3 is active the default input channel names are "ADAT 1" ..
"ADAT 8" (custom names persist as usual). Input metering (`MEAS.LEVELS`)
covers all 8 channels, which is what the loopback self-test reads to
verify channel ordering end to end.

## Signal path and latency

Decoded samples pass through per-channel preamp, per-input PEQ, the
leveller, and the 8x9 matrix, then the per-output chain; identical to
8-channel USB input. Input batching adds about 1-4 ms (48-frame minimum
block, 5.3 ms ring); the output consumer pools add their usual 50% prefill.
The path is bit-transparent up to the preamp: a 24-bit sample arrives as the
identical 24-bit value the far end transmitted.

## Wire format (receiver side)

See adat_output_spec.md "Wire format" for the frame layout; the input
implements the exact inverse. Reception details:

- The PIO program (`adat_input.pio`, PIO1 SM2, 15 instructions) is an NRZI
  decoder running at clock divider 1.0 (full 307.2 MHz, so zero divider
  jitter). Each bit cell is counted by a 2-cycle poll loop whose length is
  set per rate via the Y register (27 sys cycles at 44.1 kHz, 25 at 48 kHz);
  a detected transition emits a 1 and re-anchors the cell grid within 2 sys
  cycles in BOTH directions, so clock offset of either sign never
  accumulates. Verified bit-exact by a cycle-accurate program model across
  at least +-1000 ppm of source offset at both rates
  (tools/adat_rx_test/adat_rx_bitdiff.c). The cell length is set once per
  rate and never servoed.
- Slave acquisition does not measure a wrong-cell decoded stream. It tries
  the 48 kHz timing first, alternates to 44.1 kHz after 10 ms without a valid
  eight-header run, and continues alternating while the signal is absent or
  unsupported. On re-lock it tries the last valid family first. Fine DMA-rate
  measurement and the output servo start only after header-proven lock.
- DMA channel 15 streams decoded bits into an 8 KB ring using the RP2350
  ENDLESS transfer-count mode plus hardware address wrap: a free-running
  ring with no IRQ and no reload channel.
- Frame sync is found on the CPU: the header's 10-zero run cannot occur in
  channel data (a forced 1 every 5th bit bounds data runs to 4 zeros), so a
  single scan fixes the frame boundary; after that, frames sit at a constant
  bit offset and only the 12 structural header bits are checked per frame
  (user bits from external gear are ignored, not assumed zero).
- The decoder was verified bit-exact against the firmware's own ADAT
  encoder by a host-side round trip at all 32 possible bit offsets
  (tools/adat_rx_test/adat_rx_roundtrip.c). Hardware interop with a real
  ADAT transmitter (e.g. ADA8200) is untested as of 2026-07-13.

## Firmware internals (summary)

- Engine: `firmware/DSPi/adat_input.c` (all `#if PICO_RP2350`); RP2040
  keeps only the four config globals for round-trips.
- Main-loop poll decodes in >= 48-frame batches (cap 192) into
  `buf_l`/`buf_r`/`buf_in_ext[0..5]` and calls `process_input_block()`;
  nibble unstuffing is shift/mask (no LUTs, no RAM tables).
- Slave-mode servo actuation is the shared `input_servo_apply()`
  (input_servo.c), extracted verbatim from the SPDIF input servo; the ADAT
  output picks the servoed divider up on resync via
  `adat_input_current_tx_divider()`.
- Rate policy hooks in `perform_rate_change()` (master-mode retune / park);
  source switching, prefill, clock-mode apply, and pin hot-swap all ride the
  existing deferred main-loop handlers.
- RAM cost: 8 KB ring + ~130 B state + ~3 KB RAM-pinned hot code (the
  RP2350 RAM-image budget in scripts/check_ram_placement.py was raised from
  64 KB to 72 KB for this).
- No flash-write or output-type-switch suspension is needed: the receiver
  has no IRQs, the ring survives stalls (whole-frame lap skip preserves
  frame phase), and header verification catches anything else.
