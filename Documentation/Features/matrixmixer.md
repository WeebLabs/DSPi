# DSPi Matrix Mixer - Developer Reference

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

All vendor commands use USB control transfers with:
- **bmRequestType**: `0x40` (OUT, vendor, device) or `0xC0` (IN, vendor, device)
- **wIndex**: Interface number (2)
- **bRequest**: Command code (see tables below)
- **wValue**: Command-specific parameter
- **wLength**: Data length
- **Data**: Command-specific payload

---

## Matrix Mixer Commands

### REQ_SET_MATRIX_ROUTE (0x70) - Set Crosspoint

Configure a single crosspoint in the matrix.

| Field | Value |
|-------|-------|
| bmRequestType | 0x40 (OUT) |
| bRequest | 0x70 |
| wValue | 0 (unused) |
| wIndex | 2 |
| wLength | 9 |
| Data | MatrixRoutePacket |

**MatrixRoutePacket Structure (9 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 1 | uint8_t | input | Input channel (0=Left, 1=Right) |
| 1 | 1 | uint8_t | output | Output channel (0-8) |
| 2 | 1 | uint8_t | enabled | Route enabled (0=off, 1=on) |
| 3 | 1 | uint8_t | phase_invert | Polarity invert (0=normal, 1=inverted) |
| 4 | 4 | float | gain_db | Crosspoint gain in dB (-60.0 to +12.0) |

**Example - Route Left input to Output 0 at -3dB:**
```c
uint8_t packet[9];
packet[0] = 0;              // input = Left
packet[1] = 0;              // output = 0
packet[2] = 1;              // enabled
packet[3] = 0;              // no phase invert
float gain = -3.0f;
memcpy(&packet[4], &gain, 4);
// Send control transfer with bRequest=0x70, wValue=0, data=packet
```

---

### REQ_GET_MATRIX_ROUTE (0x71) - Get Crosspoint

Read the current state of a crosspoint.

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x71 |
| wValue | (input << 8) \| output |
| wIndex | 2 |
| wLength | 9 |
| Data | MatrixRoutePacket (returned) |

**wValue encoding:**
- High byte: input channel (0-1)
- Low byte: output channel (0-8)

**Example - Read crosspoint for Left→Output 3:**
```c
// wValue = (0 << 8) | 3 = 0x0003
MatrixRoutePacket result;
// Control IN transfer with bRequest=0x71, wValue=0x0003
// Result contains current crosspoint state
```

---

### REQ_SET_OUTPUT_ENABLE (0x72) - Enable/Disable Output

Enable or disable an output channel. Disabled outputs consume no CPU.

| Field | Value |
|-------|-------|
| bmRequestType | 0x40 (OUT) |
| bRequest | 0x72 |
| wValue | output (0-8) |
| wIndex | 2 |
| wLength | 1 |
| Data | uint8_t (0=disabled, 1=enabled) |

---

### REQ_GET_OUTPUT_ENABLE (0x73) - Get Output Enable State

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x73 |
| wValue | output (0-8) |
| wIndex | 2 |
| wLength | 1 |
| Data | uint8_t (0=disabled, 1=enabled) |

---

### REQ_SET_OUTPUT_GAIN (0x74) - Set Output Gain

Set the gain for an output channel.

| Field | Value |
|-------|-------|
| bmRequestType | 0x40 (OUT) |
| bRequest | 0x74 |
| wValue | output (0-8) |
| wIndex | 2 |
| wLength | 4 |
| Data | float (gain in dB, -60.0 to +12.0) |

---

### REQ_GET_OUTPUT_GAIN (0x75) - Get Output Gain

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x75 |
| wValue | output (0-8) |
| wIndex | 2 |
| wLength | 4 |
| Data | float (gain in dB) |

---

### REQ_SET_OUTPUT_MUTE (0x76) - Set Output Mute

Mute or unmute an output channel. Muted outputs are silenced but still consume CPU (unlike disable).

| Field | Value |
|-------|-------|
| bmRequestType | 0x40 (OUT) |
| bRequest | 0x76 |
| wValue | output (0-8) |
| wIndex | 2 |
| wLength | 1 |
| Data | uint8_t (0=unmuted, 1=muted) |

---

### REQ_GET_OUTPUT_MUTE (0x77) - Get Output Mute State

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x77 |
| wValue | output (0-8) |
| wIndex | 2 |
| wLength | 1 |
| Data | uint8_t (0=unmuted, 1=muted) |

---

### REQ_SET_OUTPUT_DELAY (0x78) - Set Output Delay

Set the delay for an output channel.

| Field | Value |
|-------|-------|
| bmRequestType | 0x40 (OUT) |
| bRequest | 0x78 |
| wValue | output (0-8) |
| wIndex | 2 |
| wLength | 4 |
| Data | float (delay in milliseconds) |

**Delay Limits:**
- RP2350: 0 to 170ms (8192 samples at 48kHz)
- RP2040: 0 to 85ms (4096 samples at 48kHz)

---

### REQ_GET_OUTPUT_DELAY (0x79) - Get Output Delay

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x79 |
| wValue | output (0-8) |
| wIndex | 2 |
| wLength | 4 |
| Data | float (delay in milliseconds) |

---

## Per-Output EQ Commands

Output channels have parametric EQ available via the standard EQ commands. Use EQ channel indices 2-10 (CH_OUT_1 through CH_OUT_9_PDM).

### REQ_SET_EQ_PARAM (0x42) - Set EQ Band Parameter

| Field | Value |
|-------|-------|
| bmRequestType | 0x40 (OUT) |
| bRequest | 0x42 |
| wValue | 0 |
| wIndex | 2 |
| wLength | 16 |
| Data | EqParamPacket |

**EqParamPacket Structure (16 bytes):**

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 1 | uint8_t | channel | EQ channel (0-10) |
| 1 | 1 | uint8_t | band | Band index (0-9) |
| 2 | 1 | uint8_t | type | Filter type (see below) |
| 3 | 1 | uint8_t | reserved | Set to 0 |
| 4 | 4 | float | freq | Center/corner frequency (Hz) |
| 8 | 4 | float | Q | Q factor (0.1 to 10.0) |
| 12 | 4 | float | gain_db | Gain in dB (-24.0 to +24.0) |

**Filter Types:**

| Value | Type | Description |
|-------|------|-------------|
| 0 | FILTER_FLAT | Bypass (no processing) |
| 1 | FILTER_PEAKING | Parametric EQ bell |
| 2 | FILTER_LOWSHELF | Low shelf |
| 3 | FILTER_HIGHSHELF | High shelf |
| 4 | FILTER_LOWPASS | 2nd-order lowpass |
| 5 | FILTER_HIGHPASS | 2nd-order highpass |

### REQ_GET_EQ_PARAM (0x43) - Get EQ Band Parameter

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x43 |
| wValue | (channel << 8) \| (band << 4) \| param |
| wIndex | 2 |
| wLength | 4 |
| Data | uint32_t or float (see below) |

**wValue encoding:**
- Bits 15-8: channel (0-10)
- Bits 7-4: band (0-9)
- Bits 3-0: parameter (0=type, 1=freq, 2=Q, 3=gain_db)

---

## Persistence Commands

### REQ_SAVE_PARAMS (0x51) - Save to Flash

Save all current settings (including matrix mixer state) to flash memory.

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x51 |
| wValue | 0 |
| wIndex | 2 |
| wLength | 1 |
| Data | int8_t result (0=success) |

### REQ_LOAD_PARAMS (0x52) - Load from Flash

Load settings from flash memory.

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x52 |
| wValue | 0 |
| wIndex | 2 |
| wLength | 1 |
| Data | int8_t result (0=success, -1=no data, -2=CRC error) |

### REQ_FACTORY_RESET (0x53) - Reset to Defaults

Reset all settings to factory defaults. Does NOT erase flash (use SAVE_PARAMS after to persist).

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (IN) |
| bRequest | 0x53 |
| wValue | 0 |
| wIndex | 2 |
| wLength | 1 |
| Data | int8_t result (0=success) |

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

- Invalid input/output indices are silently ignored
- Gain values are internally converted to linear multipliers: `linear = 10^(dB/20)`
- Delay values exceeding the maximum are clamped
- Negative delays are clamped to 0
- Flash operations return result codes:
  - `0` (FLASH_OK): Success
  - `-1` (FLASH_ERR_NO_DATA): No saved data found
  - `-2` (FLASH_ERR_CRC): CRC validation failed
  - `-3` (FLASH_ERR_WRITE): Write verification failed

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

| Code | Name | Direction | wValue | Data Size | Description |
|------|------|-----------|--------|-----------|-------------|
| 0x70 | SET_MATRIX_ROUTE | OUT | 0 | 9 | Set crosspoint routing |
| 0x71 | GET_MATRIX_ROUTE | IN | (in<<8)\|out | 9 | Get crosspoint state |
| 0x72 | SET_OUTPUT_ENABLE | OUT | output | 1 | Enable/disable output |
| 0x73 | GET_OUTPUT_ENABLE | IN | output | 1 | Get output enable state |
| 0x74 | SET_OUTPUT_GAIN | OUT | output | 4 | Set output gain (dB) |
| 0x75 | GET_OUTPUT_GAIN | IN | output | 4 | Get output gain |
| 0x76 | SET_OUTPUT_MUTE | OUT | output | 1 | Mute/unmute output |
| 0x77 | GET_OUTPUT_MUTE | IN | output | 1 | Get mute state |
| 0x78 | SET_OUTPUT_DELAY | OUT | output | 4 | Set output delay (ms) |
| 0x79 | GET_OUTPUT_DELAY | IN | output | 4 | Get output delay |
| 0x42 | SET_EQ_PARAM | OUT | 0 | 16 | Set EQ band |
| 0x43 | GET_EQ_PARAM | IN | encoded | 4 | Get EQ parameter |
| 0x51 | SAVE_PARAMS | IN | 0 | 1 | Save to flash |
| 0x52 | LOAD_PARAMS | IN | 0 | 1 | Load from flash |
| 0x53 | FACTORY_RESET | IN | 0 | 1 | Reset to defaults |

---

## Implementation Notes

1. **Atomic Updates**: Changes to crosspoints, gains, and mutes take effect immediately (within one audio buffer period, ~4ms).

2. **Delay Changes**: Modifying delay values calls `dsp_update_delay_samples()` which recalculates sample counts for the current sample rate.

3. **EQ Changes**: EQ parameter changes trigger coefficient recalculation. For smooth transitions, consider ramping gains rather than abrupt changes.

4. **CPU Optimization**: Disable unused outputs to save CPU. Each enabled output with active EQ consumes approximately 5-10% CPU.

5. **Phase Coherence**: All S/PDIF outputs are synchronized via PIO. The PDM output has automatic latency compensation.

6. **Persistence**: Matrix mixer state is included in flash storage format V5. Use SAVE_PARAMS to persist configuration across power cycles.
