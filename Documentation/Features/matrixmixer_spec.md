# DSPi Matrix Mixer - Developer Reference

*Last updated: 2026-11-16 (commands on the paged protocol of firmware 1.1.7)*

This document provides complete specifications for implementing control software for the DSPi matrix mixer. All communication occurs via USB vendor control transfers on interface 2.

---

## Architecture Overview

```
USB Stereo Input (L/R)
         │
         ▼
    ┌─────────┐
    │ Preamp  │  (global gain, -60dB to +20dB)
    └────┬────┘
         ▼
    ┌──────────┐
    │ Loudness │  (Fletcher-Munson compensation)
    └────┬─────┘
         ▼
   ┌───────────┐
   │ Master EQ │  (CH_MASTER_LEFT/RIGHT, 10 bands each)
   └─────┬─────┘
         ▼
   ┌───────────┐
   │ Crossfeed │  (stereo imaging)
   └─────┬─────┘
         ▼
╔═══════════════════════════════════════════════════════╗
║              MATRIX MIXER (2 × 9)                     ║
║                                                       ║
║  Input 0 (L) ──┬──►[xp]──►[xp]──►[xp]──► ... ──►[xp] ║
║                │                                      ║
║  Input 1 (R) ──┴──►[xp]──►[xp]──►[xp]──► ... ──►[xp] ║
║                     │      │      │            │     ║
║                     ▼      ▼      ▼            ▼     ║
║                   Out0   Out1   Out2   ...   Out8    ║
╚═══════════════════════════════════════════════════════╝
         │
         ▼ (per output)
    ┌─────────┐
    │Output EQ│  (10 bands per output channel)
    └────┬────┘
         ▼
    ┌─────────┐
    │  Gain   │  (per-output, -60dB to +12dB)
    └────┬────┘
         ▼
    ┌─────────┐
    │  Delay  │  (per-output, 0-170ms RP2350 / 0-85ms RP2040)
    └────┬────┘
         ▼
    ┌─────────┐
    │  Mute   │  (per-output soft mute)
    └────┬────┘
         ▼
   Physical Outputs (S/PDIF × 4 + PDM × 1)
```

---

## Physical Output Mapping

| Output Index | Channel Pair | GPIO Pin | Output Type | Description |
|--------------|--------------|----------|-------------|-------------|
| 0 | Out 1 (L) | GPIO 20 | S/PDIF 1 | First stereo pair, left |
| 1 | Out 2 (R) | GPIO 20 | S/PDIF 1 | First stereo pair, right |
| 2 | Out 3 (L) | GPIO 21 | S/PDIF 2 | Second stereo pair, left |
| 3 | Out 4 (R) | GPIO 21 | S/PDIF 2 | Second stereo pair, right |
| 4 | Out 5 (L) | GPIO 22 | S/PDIF 3 | Third stereo pair, left |
| 5 | Out 6 (R) | GPIO 22 | S/PDIF 3 | Third stereo pair, right |
| 6 | Out 7 (L) | GPIO 23 | S/PDIF 4 | Fourth stereo pair, left |
| 7 | Out 8 (R) | GPIO 23 | S/PDIF 4 | Fourth stereo pair, right |
| 8 | Out 9 | GPIO 10 | PDM | Mono subwoofer output |

---

## EQ Channel Mapping

| Channel Index | Constant | Description | Bands |
|---------------|----------|-------------|-------|
| 0 | `CH_MASTER_LEFT` | Master left EQ (pre-matrix) | 10 |
| 1 | `CH_MASTER_RIGHT` | Master right EQ (pre-matrix) | 10 |
| 2 | `CH_OUT_1` | Output 1 EQ (S/PDIF 1 L) | 10 (RP2350) / 2 (RP2040) |
| 3 | `CH_OUT_2` | Output 2 EQ (S/PDIF 1 R) | 10 / 2 |
| 4 | `CH_OUT_3` | Output 3 EQ (S/PDIF 2 L) | 10 / 2 |
| 5 | `CH_OUT_4` | Output 4 EQ (S/PDIF 2 R) | 10 / 2 |
| 6 | `CH_OUT_5` | Output 5 EQ (S/PDIF 3 L) | 10 / 2 |
| 7 | `CH_OUT_6` | Output 6 EQ (S/PDIF 3 R) | 10 / 2 |
| 8 | `CH_OUT_7` | Output 7 EQ (S/PDIF 4 L) | 10 / 2 |
| 9 | `CH_OUT_8` | Output 8 EQ (S/PDIF 4 R) | 10 / 2 |
| 10 | `CH_OUT_9_PDM` | Output 9 EQ (PDM Sub) | 10 / 2 |

---

## USB Control Transfer Format

All vendor commands use the paged protocol of firmware 1.1.7 (see `../commands.md`), as USB control transfers with:
- **bmRequestType**: `0x41` (OUT, vendor, interface) for a SET or `0xC1` (IN, vendor, interface) for a GET
- **wIndex**: `(page << 8) | 2`. The high byte is the page, the low byte the interface number (2)
- **bRequest**: Command number within the page (see the tables below)
- **wValue**: The address (input/output, output, or channel/band). Zero for a command with no address
- **wLength**: Data length. A SET must send exactly the documented size
- **Data**: Command-specific payload, little-endian

A rejected SET STALLs. The reason is readable from `SYS.LAST_RESULT` as a `VC_*` result code. A GET for a bad address also STALLs (`VC_ERR_ADDRESS`).

---

## Matrix Mixer Commands

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `ROUTE.MATRIX_ROUTE` | 0x05 | 0x10 | GET, SET | `(input << 8) \| output` | GET 8, SET 8 | One crosspoint |
| `ROUTE.OUTPUT_ENABLE` | 0x05 | 0x20 | GET, SET | output 0..NUM_OUTPUT_CHANNELS-1 | GET 1, SET 1 | 0 = disabled, non-zero = enabled |
| `ROUTE.OUTPUT_GAIN` | 0x05 | 0x21 | GET, SET | output | GET 4, SET 4 | f32 dB |
| `ROUTE.OUTPUT_MUTE` | 0x05 | 0x22 | GET, SET | output | GET 1, SET 1 | 0 = unmuted, 1 = muted |
| `ROUTE.OUTPUT_DELAY` | 0x05 | 0x23 | GET, SET | output | GET 4, SET 4 | f32 ms |
| `ROUTE.OUTPUT_ENABLE_CONFLICT` | 0x05 | 0x24 | GET | output | GET 1 | 1 when enabling this output would collide with the Core 1 mode |

### `ROUTE.MATRIX_ROUTE`: one crosspoint

Configure or read a single crosspoint in the matrix. The crosspoint is addressed by `wValue` in both directions.
- High byte: input channel (0..NUM_INPUT_CHANNELS-1)
- Low byte: output channel (0..NUM_OUTPUT_CHANNELS-1)

The SET payload and the GET reply share one 8-byte layout:

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 1 | uint8_t | enabled | Route enabled (0=off, 1=on) |
| 1 | 1 | uint8_t | phase_invert | Polarity invert (0=normal, 1=inverted) |
| 2 | 2 | uint16_t | reserved | Send 0 |
| 4 | 4 | float | gain_db | Crosspoint gain in dB (-60.0 to +12.0) |

A NaN or infinite gain is refused with `VC_ERR_VALUE` and stores nothing. An input or output past the end is `VC_ERR_ADDRESS`.

**Example - Route input 0 (Left) to Output 0 at -3dB:**
```c
uint8_t packet[8] = {0};
packet[0] = 1;              // enabled
packet[1] = 0;              // no phase invert
float gain = -3.0f;
memcpy(&packet[4], &gain, 4);
// SET: bmRequestType=0x41, bRequest=0x10, wIndex=(0x05 << 8) | 2,
//      wValue=(0 << 8) | 0, data=packet (8 bytes)
```

**Example - Read crosspoint for input 0 to Output 3:**
```c
// GET: bmRequestType=0xC1, bRequest=0x10, wIndex=0x0502,
//      wValue=(0 << 8) | 3 = 0x0003, wLength=8
// The reply is the 8-byte layout above. It does not echo input and output.
```

```python
# Same with the test harness (tools/dspi_test/device.py)
dev.pset("ROUTE.MATRIX_ROUTE", struct.pack("<BBHf", 1, 0, 0, -3.0), addr=(0 << 8) | 0)
enabled, phase, _, gain = struct.unpack("<BBHf", dev.pget("ROUTE.MATRIX_ROUTE", addr=(0 << 8) | 3))
```

---

### `ROUTE.OUTPUT_ENABLE`: enable or disable an output

Enable or disable an output channel. Disabled outputs consume no CPU. Payload and reply are one byte (0=disabled, 1=enabled). An enable that would collide with the Core 1 mode (PDM and the Core 1 EQ worker outputs are mutually exclusive) is refused with `VC_ERR_STATE`. `ROUTE.OUTPUT_ENABLE_CONFLICT` reports in advance whether an enable would collide.

---

### `ROUTE.OUTPUT_GAIN`: output gain

Set or read the gain for an output channel. Payload and reply are a float, gain in dB (-60.0 to +12.0). A NaN or infinite gain is refused with `VC_ERR_VALUE`.

---

### `ROUTE.OUTPUT_MUTE`: output mute

Mute or unmute an output channel. Muted outputs are silenced but still consume CPU (unlike disable). Payload and reply are one byte (0=unmuted, 1=muted).

---

### `ROUTE.OUTPUT_DELAY`: output delay

Set or read the delay for an output channel. Payload and reply are a float, delay in milliseconds. A NaN or infinite value is refused with `VC_ERR_VALUE`. A negative value is clamped to 0.

**Delay Limits:**
- RP2350: 0 to 170ms (8192 samples at 48kHz)
- RP2040: 0 to 85ms (4096 samples at 48kHz)

An `OUTPUT_DELAY` write also updates the output's entry in `ROUTE.CHANNEL_DELAY`. A `ROUTE.CHANNEL_DELAY` write to an output channel does not change what `OUTPUT_DELAY` reads back, so set output delays through `OUTPUT_DELAY`.

---

## Per-Output EQ Commands

Output channels have parametric EQ available via the standard band command. Use the output channel indices (`CH_OUT_1` and above).

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `FILTER.BAND` | 0x06 | 0x10 | GET, SET | `(channel << 8) \| band` | GET 16, SET 16 | One whole band, both directions |
| `FILTER.BAND_BYPASS` | 0x06 | 0x11 | GET, SET | `(channel << 8) \| band` | GET 1, SET 1 | Bypass one band |

**`FILTER.BAND` payload (16 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 1 | uint8_t | type | Filter type (see below) |
| 1 | 1 | uint8_t | bypass | 1 = bypassed, anything else = active (`band_bypass_spec.md`) |
| 2 | 2 | uint16_t | qp_x512 | Linkwitz Transform only (`peq_filters.md`). Send 0 otherwise |
| 4 | 4 | float | freq | Center/corner frequency (Hz) |
| 8 | 4 | float | Q | Q factor (0.1 to 10.0) |
| 12 | 4 | float | gain_db | Gain in dB (-24.0 to +24.0) |

A GET returns the whole band in the same layout. There is no per-field read. A band write that arrives while an earlier one is still waiting for the main loop is refused with `VC_ERR_BUSY`, so retry after a short delay. The full filter type list and the band index map are in `peq_filters.md` and `crossover_filters_spec.md`.

**Filter Types:**

| Value | Type | Description |
|-------|------|-------------|
| 0 | FILTER_FLAT | Bypass (no processing) |
| 1 | FILTER_PEAKING | Parametric EQ bell |
| 2 | FILTER_LOWSHELF | Low shelf |
| 3 | FILTER_HIGHSHELF | High shelf |
| 4 | FILTER_LOWPASS | 2nd-order lowpass |
| 5 | FILTER_HIGHPASS | 2nd-order highpass |

---

## Persistence Commands

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `STORE.PRESET_SAVE` | 0x02 | 0x10 | SET (zero-length) | slot 0-9 | SET 0 | Save the live state, matrix included, to a slot |
| `STORE.PRESET_ACTIVE` | 0x02 | 0x14 | GET | none (0) | GET 1 | The active slot |
| `STORE.FACTORY_RESET` | 0x02 | 0x30 | SET (zero-length) | none (0) | SET 0 | Reset the live state to defaults |

Matrix mixer state is saved with the rest of the live state in a user preset slot (`user_presets_spec.md`). The 1.1.6 "save to the active slot" request (opcode 0x51) is retired. To get the same effect, read `STORE.PRESET_ACTIVE` and send `STORE.PRESET_SAVE` to that slot. There was never a separate "load from flash" request: opcode 0x52, listed here as one in earlier revisions of this document, was the output-config save, now `STORE.OUTPUT_CONFIG_SAVE`. Loading a preset is `STORE.PRESET_LOAD`.

`STORE.FACTORY_RESET` is a zero-length SET with no status byte. Success is the ACK. The reset runs on the main loop after the ACK. It resets all live settings to factory defaults and does NOT erase flash (save to a preset slot with `STORE.PRESET_SAVE` afterwards to persist).

**Factory Default Matrix Configuration:**
- Input L → Output 0 at 0dB (enabled)
- Input R → Output 1 at 0dB (enabled)
- Outputs 0-1 enabled, 2-8 disabled
- All gains at 0dB, no mutes, no delays
- No phase inversions
- Default EQ: 80Hz highpass on outputs 0-7, 80Hz lowpass on output 8 (PDM sub)

---

## Common Routing Examples

### Stereo Pass-Through (Default)

```
L → Out0 (0dB), R → Out1 (0dB)
Outputs 0,1 enabled; 2-8 disabled
```

### Mono Subwoofer (L+R Sum)

```
L → Out8 (-6dB), R → Out8 (-6dB)  // -6dB each to prevent clipping on sum
Output 8 enabled
EQ: 80Hz lowpass on channel 10 (CH_OUT_9_PDM)
```

### 2.1 System (Stereo + Sub)

```
L → Out0 (0dB), R → Out1 (0dB)          // Main L/R
L → Out8 (-6dB), R → Out8 (-6dB)        // Sub (mono sum)
Outputs 0,1,8 enabled
EQ: 80Hz highpass on channels 2,3; 80Hz lowpass on channel 10
```

### Bi-Amped Stereo (4 outputs)

```
L → Out0 (0dB), R → Out1 (0dB)          // Tweeters (highpassed)
L → Out2 (0dB), R → Out3 (0dB)          // Woofers (lowpassed)
Outputs 0,1,2,3 enabled
EQ: 2kHz highpass on channels 2,3; 2kHz lowpass on channels 4,5
```

### Phase-Inverted Subwoofer

```
L → Out8 (-6dB, phase_invert=1)
R → Out8 (-6dB, phase_invert=1)
```

---

## Data Type Summary

| Type | Size | Range | Notes |
|------|------|-------|-------|
| Input index | uint8_t | 0-1 | 0=Left, 1=Right |
| Output index | uint8_t | 0-8 | 0-7=S/PDIF, 8=PDM |
| EQ channel | uint8_t | 0-10 | 0-1=Master, 2-10=Outputs |
| EQ band | uint8_t | 0-9 | 10 bands per channel (RP2040: 2 for outputs) |
| Filter type | uint8_t | 0-5 | See filter types table |
| Enable/Mute | uint8_t | 0-1 | 0=off/unmuted, 1=on/muted |
| Gain (dB) | float | -60.0 to +12.0 | IEEE 754 single precision |
| Frequency (Hz) | float | 20.0 to 20000.0 | |
| Q factor | float | 0.1 to 10.0 | |
| Delay (ms) | float | 0.0 to 170.0 | RP2040 max: 85.0 |

---

## Error Handling

- A rejected SET STALLs, and `SYS.LAST_RESULT` gives the reason.
  - `VC_ERR_ADDRESS`: input, output, channel or band index out of range (GETs too)
  - `VC_ERR_LENGTH`: payload size other than the documented one
  - `VC_ERR_VALUE`: NaN or infinite float (gain, delay, EQ freq/Q/gain)
  - `VC_ERR_STATE`: output enable that collides with the Core 1 mode
  - `VC_ERR_BUSY`: EQ band write while an earlier one is still pending
- Gain values are internally converted to linear multipliers: `linear = 10^(dB/20)`
- Delay values exceeding the maximum are clamped
- Negative delays are clamped to 0
- Preset saves and the factory reset are deferred and return no flash result code. Read `STORE.DIRECTORY` to confirm a save. A factory reset is announced by a bulk-invalidated notification (`notification_protocol_v2_spec.md`)

---

## Platform Differences

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| Max delay | 85ms | 170ms |
| Output EQ bands | 2 per channel | 10 per channel |
| DSP precision | 32-bit fixed-point | 32-bit float (64-bit accumulators) |
| RAM for delays | 144KB (shared) | 295KB |

---

## Command Reference Quick Table

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `ROUTE.MATRIX_ROUTE` | 0x05 | 0x10 | GET, SET | `(input << 8) \| output` | GET 8, SET 8 | Crosspoint routing |
| `ROUTE.OUTPUT_ENABLE` | 0x05 | 0x20 | GET, SET | output | GET 1, SET 1 | Enable/disable output |
| `ROUTE.OUTPUT_GAIN` | 0x05 | 0x21 | GET, SET | output | GET 4, SET 4 | Output gain (dB) |
| `ROUTE.OUTPUT_MUTE` | 0x05 | 0x22 | GET, SET | output | GET 1, SET 1 | Mute/unmute output |
| `ROUTE.OUTPUT_DELAY` | 0x05 | 0x23 | GET, SET | output | GET 4, SET 4 | Output delay (ms) |
| `ROUTE.OUTPUT_ENABLE_CONFLICT` | 0x05 | 0x24 | GET | output | GET 1 | Core 1 collision check |
| `FILTER.BAND` | 0x06 | 0x10 | GET, SET | `(channel << 8) \| band` | GET 16, SET 16 | EQ band |
| `STORE.PRESET_SAVE` | 0x02 | 0x10 | SET (zero-length) | slot 0-9 | SET 0 | Save to a preset slot |
| `STORE.FACTORY_RESET` | 0x02 | 0x30 | SET (zero-length) | none (0) | SET 0 | Reset to defaults |

---

## Implementation Notes

1. **Atomic Updates**: Changes to crosspoints, gains, and mutes take effect immediately (within one audio buffer period, ~4ms).

2. **Delay Changes**: Modifying delay values calls `dsp_update_delay_samples()` which recalculates sample counts for the current sample rate.

3. **EQ Changes**: EQ parameter changes trigger coefficient recalculation. For smooth transitions, consider ramping gains rather than abrupt changes.

4. **CPU Optimization**: Disable unused outputs to save CPU. Each enabled output with active EQ consumes approximately 5-10% CPU.

5. **Phase Coherence**: All S/PDIF outputs are synchronized via PIO. The PDM output has automatic latency compensation.

6. **Persistence**: Matrix mixer state is part of every user preset slot. Use `STORE.PRESET_SAVE` to persist configuration across power cycles.
