# Volume Leveller Specification

## 1. Overview

The Volume Leveller is a feedforward, stereo-linked, single-band RMS dynamic range compressor. Its purpose is to maintain a consistent perceived volume across content with varying loudness levels -- for example, preventing quiet dialogue from being drowned out by loud action scenes, or normalizing volume differences between music tracks.

### Key characteristics

- **Upward compression:** Boosts content below the threshold while leaving content above the threshold completely untouched. No makeup gain needed -- the upward compression IS the boost.
- **RMS-based detection:** Uses root-mean-square envelope tracking, which correlates with perceived loudness better than peak detection.
- **Soft-knee compression:** Gradual transition between full boost and unity gain for transparent, artifact-free gain control.
- **Stereo-linked:** The louder of the two channels determines gain for both, preserving the stereo image.
- **Optional lookahead:** A 10ms delay buffer allows the compressor to "see" transitions before they arrive. Less critical with upward compression since loud content receives 0 dB gain (no overshoot to anticipate), but still available for marginally smoother transitions.
- **Gain-reduction limiter:** Safety limiter at -6 dBFS ceiling uses gain reduction (instant attack, 100ms release) rather than hard clipping, avoiding waveform distortion. Rarely engages since loud content passes through at unity.

### Signal chain position

The Volume Leveller sits at **PASS 2.5** in the DSP pipeline:

```
USB Audio In
    |
PASS 1: Preamp + Volume
    |
PASS 2: Master EQ (per-channel biquad/SVF filters)
    |
PASS 2.5: Volume Leveller  <-- HERE
    |
PASS 3: Crossfeed + Master Peak Metering
    |
PASS 4: Matrix Mixing (fan-out to output channels)
    |
PASS 5: Per-Output EQ + Delay + Gain
    |
Output Encoding (S/PDIF, I2S, PDM)
```

The leveller processes only the **master L/R input pair** (channels 0 and 1). It operates on audio blocks in-place, modifying `buf_l` and `buf_r` before they reach the crossfeed stage and matrix mixer.

### Independence from loudness compensation

The Volume Leveller is completely independent of the Loudness Compensation feature. Loudness compensation applies ISO 226 equal-loudness contour correction driven by the USB volume knob position. The Volume Leveller instead performs dynamic gain adjustment based on the signal's RMS level. Both can be enabled simultaneously without conflict.

---

## 2. Parameters

### 2.1 enabled

| Property | Value |
|----------|-------|
| **Type** | `bool` (uint8_t on wire) |
| **Range** | 0 (off) or 1 (on) |
| **Default** | 0 (disabled) |
| **SET command** | `0xB4` (`REQ_SET_LEVELLER_ENABLE`) |
| **GET command** | `0xB5` (`REQ_GET_LEVELLER_ENABLE`) |
| **Payload** | 1 byte: `0x00` = disabled, `0x01` = enabled |

Controls whether the Volume Leveller is active in the signal chain. When disabled, the leveller is completely bypassed with zero CPU cost (checked via a fast boolean flag before the processing function is called). Toggling this parameter triggers a full state reset (envelopes zeroed, gain returned to unity, lookahead buffer cleared).

### 2.2 amount

| Property | Value |
|----------|-------|
| **Type** | `float` (IEEE 754 single-precision) |
| **Range** | 0.0 to 100.0 |
| **Default** | 50.0 |
| **SET command** | `0xB6` (`REQ_SET_LEVELLER_AMOUNT`) |
| **GET command** | `0xB7` (`REQ_GET_LEVELLER_AMOUNT`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Controls the upward compression strength as a percentage. This parameter determines the compression ratio:

- **0%:** ratio = 1:1 (no compression, passthrough)
- **50%:** ratio = 10.5:1 (moderate levelling, default)
- **100%:** ratio = 20:1 (aggressive levelling)

The formula is: `ratio = 1.0 + (amount / 100.0) * 19.0`

With upward compression, higher amounts increase the boost applied to quiet content. There is no makeup gain -- the upward compression directly provides the boost, while loud content above the threshold is left untouched. A good starting point for most content is 40-60%.

Values outside the valid range are clamped by the firmware.

### 2.3 speed

| Property | Value |
|----------|-------|
| **Type** | `uint8_t` |
| **Range** | 0, 1, or 2 |
| **Default** | 0 (Slow) |
| **SET command** | `0xB8` (`REQ_SET_LEVELLER_SPEED`) |
| **GET command** | `0xB9` (`REQ_GET_LEVELLER_SPEED`) |
| **Payload** | 1 byte |

Selects a speed preset that controls the attack time, release time, and RMS window length. Each preset is tuned for a different content type:

| Value | Name | Attack | Release | RMS Window | Use case |
|-------|------|--------|---------|------------|----------|
| 0 | Slow | 100ms | 2000ms | 400ms | Music, orchestral, wide dynamic range (default) |
| 1 | Medium | 50ms | 1000ms | 200ms | General purpose |
| 2 | Fast | 20ms | 500ms | 100ms | Speech, dialogue, podcasts |

- **Attack time:** How quickly the compressor reacts when loud content appears (gain decreasing from boosted state). With upward compression, attack controls how fast the boost is removed when the signal rises above the threshold.
- **Release time:** How quickly gain recovers (boost resumes) after the signal drops back below the threshold. Shorter = faster recovery, but too fast can cause "pumping" artifacts.
- **RMS window:** The averaging time for level detection. Longer windows give smoother, more musical compression; shorter windows track rapid level changes.

Invalid values (>= 3) are rejected silently by the firmware (the SET command has no effect).

### 2.4 max_gain_db

| Property | Value |
|----------|-------|
| **Type** | `float` (IEEE 754 single-precision) |
| **Range** | 0.0 to 35.0 |
| **Default** | 15.0 |
| **SET command** | `0xBA` (`REQ_SET_LEVELLER_MAX_GAIN`) |
| **GET command** | `0xBB` (`REQ_GET_LEVELLER_MAX_GAIN`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Sets the maximum boost (in dB) that the upward compressor can apply to quiet content. This prevents the compressor from amplifying very quiet passages (or near-silence) by an excessive amount, which would raise the noise floor.

- **0 dB:** No boost allowed; the leveller passes audio through without gain change.
- **12 dB:** Up to 4x voltage gain for quiet content.
- **15 dB:** Up to ~5.6x voltage gain (default).
- **24 dB:** Up to 16x voltage gain.
- **35 dB:** Up to ~56x voltage gain (maximum; use with caution, may amplify noise).

Values outside the valid range are clamped by the firmware.

### 2.5 lookahead

| Property | Value |
|----------|-------|
| **Type** | `bool` (uint8_t on wire) |
| **Range** | 0 (off) or 1 (on) |
| **Default** | 1 (enabled) |
| **SET command** | `0xBC` (`REQ_SET_LEVELLER_LOOKAHEAD`) |
| **GET command** | `0xBD` (`REQ_GET_LEVELLER_LOOKAHEAD`) |
| **Payload** | 1 byte: `0x00` = disabled, `0x01` = enabled |

Enables a 480-sample (10ms at 48kHz) lookahead delay. When enabled, the audio signal is delayed by 10ms while the level detection operates on the non-delayed signal. This allows the compressor to anticipate level transitions before they arrive.

With upward compression, lookahead is less critical than with traditional downward compression because loud content receives 0 dB gain (there is no overshoot to anticipate). However, it still provides marginally smoother transitions when the signal crosses the compression threshold.

**Trade-off:** Enabling lookahead adds 10ms of latency to the audio path. This is inaudible for music playback but may be noticeable in real-time monitoring scenarios.

Toggling this parameter triggers a full state reset (lookahead buffer cleared, envelopes zeroed, gain returned to unity) to prevent clicks or artifacts.

### 2.6 gate_threshold_db

| Property | Value |
|----------|-------|
| **Type** | `float` (IEEE 754 single-precision) |
| **Range** | -96.0 to 0.0 |
| **Default** | -96.0 |
| **SET command** | `0xBE` (`REQ_SET_LEVELLER_GATE`) |
| **GET command** | `0xBF` (`REQ_GET_LEVELLER_GATE`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Sets the silence gate threshold in dBFS. When the RMS level of the signal falls below this threshold, the leveller applies unity gain (0 dB adjustment) instead of boosting. This prevents the upward compressor from amplifying background noise or near-silence, which would raise the noise floor ("noise pumping").

- **-96.0 dBFS:** Default. Gate is effectively disabled (only true digital silence triggers it).
- **-70.0 dBFS:** Prevents boosting of typical background noise floors.
- **-40.0 dBFS:** Aggressive gating; quiet passages will not be boosted.
- **0.0 dBFS:** Gate is always active (leveller never boosts; equivalent to disabling the leveller).

Values outside the valid range are clamped by the firmware.

### Parameter summary

| Parameter | Type | Range | Default | SET | GET | Payload |
|-----------|------|-------|---------|-----|-----|---------|
| enabled | bool | 0/1 | 0 | 0xB4 | 0xB5 | 1 byte |
| amount | float | 0.0-100.0 | 50.0 | 0xB6 | 0xB7 | 4 bytes (LE float) |
| speed | uint8_t | 0-2 | 0 | 0xB8 | 0xB9 | 1 byte |
| max_gain_db | float | 0.0-35.0 | 15.0 | 0xBA | 0xBB | 4 bytes (LE float) |
| lookahead | bool | 0/1 | 1 | 0xBC | 0xBD | 1 byte |
| gate_threshold_db | float | -96.0-0.0 | -96.0 | 0xBE | 0xBF | 4 bytes (LE float) |

---

## 3. Vendor Command Reference

All commands use USB EP0 vendor control transfers on the vendor interface (interface 2). SET commands are host-to-device (OUT) with data in the payload. GET commands are device-to-host (IN) with the response in the data phase. The `wValue` and `wIndex` fields are not used by leveller commands (ignored by firmware).

### 3.1 REQ_SET_LEVELLER_ENABLE (0xB4)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xB4` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Payload** | 1 byte: `0x00` = disable, nonzero = enable |

**Firmware behavior:**
1. Reads byte 0 from the vendor receive buffer.
2. Sets `leveller_config.enabled` to `true` if nonzero, `false` if zero.
3. Sets `leveller_update_pending = true` and `leveller_reset_pending = true`.
4. On the next main loop iteration, coefficients are recomputed, state is reset (envelopes zeroed, gain to unity, lookahead buffer cleared), and the bypass flag is updated.

**Validation:** Minimum payload length = 1 byte. Shorter payloads are silently ignored.

**Example:** Enable the leveller:
```
bRequest = 0xB4, wValue = 0x0000, wIndex = 0x0000, wLength = 1
Payload: [0x01]
```

### 3.2 REQ_GET_LEVELLER_ENABLE (0xB5)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xB5` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Response** | 1 byte: `0x00` = disabled, `0x01` = enabled |

**Example response** (leveller enabled):
```
[0x01]
```

### 3.3 REQ_SET_LEVELLER_AMOUNT (0xB6)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xB6` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Payload** | 4 bytes: IEEE 754 float, little-endian |

**Firmware behavior:**
1. Reads 4 bytes from vendor receive buffer as a float via `memcpy`.
2. Clamps value to [0.0, 100.0].
3. Sets `leveller_config.amount` and `leveller_update_pending = true`.
4. On the next main loop iteration, coefficients (ratio, max gain) are recomputed. State is NOT reset -- the envelope continues tracking smoothly.

**Validation:** Minimum payload length = 4 bytes. Shorter payloads are silently ignored. Out-of-range values are clamped (not rejected).

**Example:** Set amount to 75.0%:
```
bRequest = 0xB6, wValue = 0x0000, wIndex = 0x0000, wLength = 4
Payload: [0x00, 0x00, 0x96, 0x42]   (75.0f in little-endian IEEE 754)
```

### 3.4 REQ_GET_LEVELLER_AMOUNT (0xB7)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xB7` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Response** | 4 bytes: IEEE 754 float, little-endian |

**Example response** (amount = 50.0):
```
[0x00, 0x00, 0x48, 0x42]   (50.0f)
```

### 3.5 REQ_SET_LEVELLER_SPEED (0xB8)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xB8` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Payload** | 1 byte: `0x00` = Slow, `0x01` = Medium, `0x02` = Fast |

**Firmware behavior:**
1. Reads byte 0 from vendor receive buffer.
2. If the value is < 3 (valid), sets `leveller_config.speed` and `leveller_update_pending = true`.
3. If the value is >= 3, the command is silently ignored (no change).

**Validation:** The value must be 0, 1, or 2. Invalid values are rejected (not clamped).

**Example:** Set speed to Fast (speech):
```
bRequest = 0xB8, wValue = 0x0000, wIndex = 0x0000, wLength = 1
Payload: [0x02]
```

### 3.6 REQ_GET_LEVELLER_SPEED (0xB9)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xB9` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Response** | 1 byte: `0x00`, `0x01`, or `0x02` |

**Example response** (Medium):
```
[0x01]
```

### 3.7 REQ_SET_LEVELLER_MAX_GAIN (0xBA)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xBA` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Payload** | 4 bytes: IEEE 754 float, little-endian |

**Firmware behavior:**
1. Reads 4 bytes from vendor receive buffer as a float via `memcpy`.
2. Clamps value to [0.0, 35.0].
3. Sets `leveller_config.max_gain_db` and `leveller_update_pending = true`.
4. Coefficients are recomputed on the next main loop iteration. State is NOT reset.

**Validation:** Minimum payload length = 4 bytes. Shorter payloads are silently ignored. Out-of-range values are clamped.

**Example:** Set max gain to 18.0 dB:
```
bRequest = 0xBA, wValue = 0x0000, wIndex = 0x0000, wLength = 4
Payload: [0x00, 0x00, 0x90, 0x41]   (18.0f in little-endian IEEE 754)
```

### 3.8 REQ_GET_LEVELLER_MAX_GAIN (0xBB)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xBB` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Response** | 4 bytes: IEEE 754 float, little-endian |

**Example response** (max_gain_db = 15.0):
```
[0x00, 0x00, 0x70, 0x41]   (15.0f)
```

### 3.9 REQ_SET_LEVELLER_LOOKAHEAD (0xBC)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xBC` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Payload** | 1 byte: `0x00` = disable, nonzero = enable |

**Firmware behavior:**
1. Reads byte 0 from the vendor receive buffer.
2. Sets `leveller_config.lookahead` to `true` if nonzero, `false` if zero.
3. Sets `leveller_update_pending = true` and `leveller_reset_pending = true`.
4. On the next main loop iteration, coefficients are recomputed and state is reset (including clearing the circular delay buffer).

The state reset on toggle is intentional: switching lookahead on/off changes the effective signal path, and stale data in the delay buffer would produce clicks.

**Validation:** Minimum payload length = 1 byte. Shorter payloads are silently ignored.

**Example:** Enable lookahead:
```
bRequest = 0xBC, wValue = 0x0000, wIndex = 0x0000, wLength = 1
Payload: [0x01]
```

### 3.10 REQ_GET_LEVELLER_LOOKAHEAD (0xBD)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xBD` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 1 |
| **Response** | 1 byte: `0x00` = disabled, `0x01` = enabled |

**Example response** (lookahead disabled):
```
[0x00]
```

### 3.11 REQ_SET_LEVELLER_GATE (0xBE)

| Field | Value |
|-------|-------|
| **Direction** | Host -> Device (OUT) |
| **bRequest** | `0xBE` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Payload** | 4 bytes: IEEE 754 float, little-endian |

**Firmware behavior:**
1. Reads 4 bytes from vendor receive buffer as a float via `memcpy`.
2. Clamps value to [-96.0, 0.0].
3. Sets `leveller_config.gate_threshold_db` and `leveller_update_pending = true`.
4. Coefficients are recomputed on the next main loop iteration. State is NOT reset.

**Validation:** Minimum payload length = 4 bytes. Shorter payloads are silently ignored. Out-of-range values are clamped.

**Example:** Set gate threshold to -60.0 dBFS:
```
bRequest = 0xBE, wValue = 0x0000, wIndex = 0x0000, wLength = 4
Payload: [0x00, 0x00, 0x70, 0xC2]   (-60.0f in little-endian IEEE 754)
```

### 3.12 REQ_GET_LEVELLER_GATE (0xBF)

| Field | Value |
|-------|-------|
| **Direction** | Device -> Host (IN) |
| **bRequest** | `0xBF` |
| **wValue** | Unused (0) |
| **wIndex** | Unused (0) |
| **wLength** | 4 |
| **Response** | 4 bytes: IEEE 754 float, little-endian |

**Example response** (gate_threshold_db = -96.0):
```
[0x00, 0x00, 0xC0, 0xC2]   (-96.0f)
```

---

## 4. Wire Format (Bulk Parameters)

The Volume Leveller configuration is included in the bulk parameter transfer as `WireLevellerConfig`, a 16-byte packed struct. This allows the entire leveller state to be read or written as part of a single USB control transfer alongside all other DSP parameters.

### WireLevellerConfig struct (16 bytes)

| Byte offset | Size | Type | Field | Description |
|-------------|------|------|-------|-------------|
| 0 | 1 | uint8_t | `enabled` | 0 = disabled, 1 = enabled |
| 1 | 1 | uint8_t | `speed` | 0 = Slow, 1 = Medium, 2 = Fast |
| 2 | 1 | uint8_t | `lookahead` | 0 = off, 1 = on |
| 3 | 1 | uint8_t | `reserved` | Must be 0 |
| 4 | 4 | float | `amount` | 0.0 - 100.0 (compression strength %) |
| 8 | 4 | float | `max_gain_db` | 0.0 - 35.0 (max boost in dB) |
| 12 | 4 | float | `gate_threshold_db` | -96.0 - 0.0 (silence gate in dBFS) |

All multi-byte fields are little-endian. Float fields are IEEE 754 single-precision at 4-byte-aligned offsets.

### Position in WireBulkParams

The `WireLevellerConfig` is the last section in the `WireBulkParams` struct:

| Section | Size (bytes) | Cumulative offset |
|---------|-------------|-------------------|
| WireHeader | 16 | 0 |
| WireGlobalParams | 16 | 16 |
| WireCrossfeedParams | 16 | 32 |
| WireLegacyChannels | 16 | 48 |
| WireChannelDelays | 44 | 64 |
| WireCrosspoint[2][9] | 144 | 108 |
| WireOutputChannel[9] | 108 | 252 |
| WirePinConfig | 8 | 360 |
| WireBandParams[11][12] | 2112 | 368 |
| WireChannelNames | 352 | 2480 |
| WireI2SConfig | 16 | 2832 |
| **WireLevellerConfig** | **16** | **2848** |
| **Total** | **2864** | |

### Version compatibility

The `WireLevellerConfig` section was introduced in `WIRE_FORMAT_VERSION` 4.

| Wire format version | Leveller section present | Behavior on SET |
|---------------------|--------------------------|-----------------|
| V2 | No | Leveller defaults applied |
| V3 | No | Leveller defaults applied |
| V4 | Yes | Leveller config parsed and applied |

When the firmware receives a SET (0xA1) with `format_version` < 4, the leveller is reset to factory defaults:
- enabled = false
- amount = 50.0
- speed = 0 (Slow)
- max_gain_db = 15.0
- lookahead = true
- gate_threshold_db = -96.0

When the firmware sends a GET (0xA0), the `WireLevellerConfig` section is always populated with current values and `format_version` is set to 4.

### Bulk transfer commands

| Command | Code | Direction | Description |
|---------|------|-----------|-------------|
| `REQ_GET_ALL_PARAMS` | 0xA0 | Device -> Host | Returns full `WireBulkParams` (2864 bytes) |
| `REQ_SET_ALL_PARAMS` | 0xA1 | Host -> Device | Applies full `WireBulkParams` to live state |

After a bulk SET, the firmware sets `leveller_update_pending = true` and `leveller_reset_pending = true`, triggering a full coefficient recompute and state reset on the next main loop iteration.

---

## 5. Preset Persistence

Volume Leveller parameters are saved and restored as part of the user preset system.

### Flash storage version

The leveller fields were added to the `PresetSlot` struct at `SLOT_DATA_VERSION` 10 (previously 9 for I2S config).

### PresetSlot fields

The following fields are appended to the `PresetSlot` struct after the I2S configuration fields:

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `leveller_enabled` | uint8_t | 1 | 0 = disabled, 1 = enabled |
| `leveller_speed` | uint8_t | 1 | 0/1/2 speed preset index |
| `leveller_lookahead` | uint8_t | 1 | 0 = off, 1 = on |
| `leveller_padding` | uint8_t | 1 | Alignment padding (must be 0) |
| `leveller_amount` | float | 4 | Compression strength 0.0-100.0 |
| `leveller_max_gain_db` | float | 4 | Max boost 0.0-35.0 dB |
| `leveller_gate_threshold_db` | float | 4 | Silence gate -96.0-0.0 dBFS |

Total: 16 bytes added to the preset slot.

### Save behavior

When `REQ_PRESET_SAVE` (0x90) is issued, the current `leveller_config` fields are copied into the slot:

```
slot->leveller_enabled          = leveller_config.enabled ? 1 : 0;
slot->leveller_speed            = leveller_config.speed;
slot->leveller_lookahead        = leveller_config.lookahead ? 1 : 0;
slot->leveller_amount           = leveller_config.amount;
slot->leveller_max_gain_db      = leveller_config.max_gain_db;
slot->leveller_gate_threshold_db = leveller_config.gate_threshold_db;
```

### Load behavior

When `REQ_PRESET_LOAD` (0x91) is issued:

- **Slot version >= 10:** Leveller parameters are restored from the slot data.
- **Slot version < 10:** Factory defaults are applied (the slot predates leveller support).
- **Unconfigured (empty) slot:** Factory defaults are applied.

In all cases, `leveller_update_pending` and `leveller_reset_pending` are set to `true`, triggering coefficient recomputation and a full state reset.

### Factory defaults

Applied when loading an empty slot, on factory reset (`REQ_FACTORY_RESET` / 0x53), or when loading a pre-V10 slot:

| Parameter | Default value |
|-----------|---------------|
| enabled | false (0) |
| amount | 50.0 |
| speed | 0 (Slow) |
| max_gain_db | 15.0 |
| lookahead | true (1) |
| gate_threshold_db | -96.0 |

### Boot behavior

On device startup:
1. The preset system loads the startup slot (per the startup policy).
2. If the slot contains V10+ data, leveller parameters are restored.
3. If the slot is empty or pre-V10, factory defaults are applied.
4. `leveller_compute_coefficients()` is called with the loaded config and an initial sample rate of 48000 Hz.
5. `leveller_reset_state()` clears all runtime state.
6. `leveller_bypassed` is set based on `leveller_config.enabled`.

---

## 6. Algorithm Details

This section describes the internal DSP algorithm for advanced users and developers who want to understand what the leveller is doing to the audio signal.

### 6.1 Signal flow (per block)

Each audio block (up to 192 samples of stereo audio) is processed in four stages:

1. **Per-sample RMS envelope update** -- Track the signal level.
2. **Per-block gain computation** -- Determine how much gain adjustment is needed.
3. **Per-block gain smoothing** -- Smooth the gain change to avoid artifacts.
4. **Per-sample gain application** -- Apply the smoothed gain with interpolation, optional lookahead delay, and safety limiting.

### 6.2 RMS envelope (Stage 1)

The RMS level is estimated using a one-pole IIR filter on the squared input signal. This is computed independently for L and R channels:

```
env = alpha_rms * env + (1 - alpha_rms) * (sample * sample)
```

Where `alpha_rms` is the retention coefficient computed from the speed preset's RMS window time constant. The convention is "Form A" (retention form):

```
alpha = exp(-ln(10) / (Fs * T))
```

Where `T` is the 0%-to-90% step response time and `Fs` is the sample rate.

- `alpha` near 1.0 = slow (retains previous value, long averaging window)
- `alpha` near 0.0 = fast (tracks input immediately)

The envelope value `env` represents the mean of the squared signal (proportional to RMS squared). Denormal values below 1e-30 are flushed to zero to prevent floating-point performance degradation.

### 6.3 Gain computer (Stage 2)

After envelope update, the stereo-linked RMS level is computed by taking the maximum of the L and R envelopes:

```
rms_sq = max(env_sq_l, env_sq_r)
rms_db = 10 * log10(rms_sq + 1e-30)
```

The gain computer then determines the upward compression boost in dB using a soft-knee curve. Unlike a traditional downward compressor (which reduces loud content), this is an upward compressor that boosts quiet content while leaving loud content untouched.

**Fixed internal parameters:**
| Parameter | Value |
|-----------|-------|
| Threshold | -20 dBFS |
| Knee width | 6 dB |
| Limiter ceiling | -6 dBFS (0.50119 linear) |

**Silence gate:** If `rms_db < gate_threshold_db`, the gain adjustment is 0 dB (unity). This prevents the compressor from boosting near-silence, which would amplify background noise ("noise pumping"). The gate threshold is a user-configurable parameter (see section 2.6); the default is -96.0 dBFS (effectively disabled).

**Upward compression curve:**

The knee region spans from `threshold - knee_width/2` to `threshold + knee_width/2`, i.e., from -23 dBFS to -17 dBFS.

| Input level region | Gain computation |
|-------------------|------------------|
| Above knee (> -17 dBFS) | 0 dB (loud content untouched) |
| Within knee (-23 to -17 dBFS) | Quadratic blend: `(1 - 1/ratio) * d^2 / (2 * knee_width)` where `d = threshold + knee_width/2 - x_db` |
| Below knee (< -23 dBFS) | Full upward compression: `(threshold - x_db) * (1 - 1/ratio)` |

There is no makeup gain. The upward compression directly provides the boost for quiet content. Loud content above the threshold receives 0 dB gain (passes through at unity).

After the upward compression curve, the gain is clamped to `max_gain_db`.

### 6.4 Gain smoothing (Stage 3)

The target gain from Stage 2 is smoothed using an asymmetric one-pole filter with separate attack and release time constants:

```
if target_gain < current_smooth_gain:
    alpha = alpha_attack     (gain decreasing = loud signal detected)
else:
    alpha = alpha_release    (gain recovering = signal getting quieter)

gain_smooth_db = alpha * gain_smooth_db + (1 - alpha) * target_gain
```

With upward compression, "attack" corresponds to the gain decreasing (removing boost when the signal rises above the threshold) and "release" corresponds to the gain increasing (restoring boost when the signal drops back below the threshold). This asymmetry sounds natural and avoids "pumping."

The smoothed dB gain is then converted to a linear multiplier:

```
gain_linear = 10^(gain_smooth_db / 20)
```

### 6.5 Gain application with interpolation (Stage 4)

The gain multiplier changes once per block (e.g., every 192 samples = 4ms at 48kHz). Applying a step change in gain would produce audible clicks. To prevent this, the gain is linearly interpolated from the previous block's gain to the current block's gain across all samples in the block:

```
gain_step = (gain_current - gain_previous) / (block_size - 1)
gain = gain_previous
for each sample:
    output = input * gain
    gain += gain_step
```

### 6.6 Lookahead delay

When lookahead is enabled, a 480-sample (10ms at 48kHz) circular delay buffer inserts a delay in the audio path. The RMS envelope is computed from the *non-delayed* signal, but the gain is applied to the *delayed* signal. This means the compressor "sees" level transitions 10ms before they pass through, allowing it to pre-emptively adjust gain.

With upward compression, lookahead is less critical than with traditional downward compression. Since loud content receives 0 dB gain, there is no overshoot to anticipate. However, lookahead still provides marginally smoother transitions when the signal crosses between the boosted and unity-gain regions. Lookahead is enabled by default.

The delay buffer is implemented as a simple circular write/read:

```
for each sample:
    delayed_sample = lookahead_buf[write_idx]       // read old
    lookahead_buf[write_idx] = current_sample       // write new
    write_idx = (write_idx + 1) % 480
    output = delayed_sample * gain
```

The buffer is always allocated in BSS (both channels, 480 samples each) but only used when `cfg->lookahead` is true.

### 6.7 Gain-reduction limiter

A gain-reduction style safety limiter at -6 dBFS ceiling (`LEVELLER_LIMITER_CEIL = 0.50119` linear) prevents downstream clipping without waveform distortion.

**Operation:**
1. Scan the current block for predicted output peaks (input sample * leveller gain).
2. If the peak exceeds the ceiling, compute the gain reduction: `limiter_gain = ceiling / peak`.
3. Smooth the limiter gain with instant attack (0ms) and 100ms release using a per-block one-pole IIR.
4. Multiply each output sample by the smoothed limiter gain.

Unlike a hard clipper, this approach scales the entire waveform down when peaks would exceed the ceiling, preserving the waveform shape. The limiter gain is always <= 1.0 (unity = no limiting).

With upward compression, the limiter rarely engages because loud content (which is most likely to clip) receives 0 dB gain from the compressor. The limiter primarily acts as a safety net for edge cases where the upward boost of quiet content combines with subsequent gain stages to approach the ceiling.

---

## 7. App Integration Guide

This section walks through how to add Volume Leveller support to a host application that communicates with the DSPi firmware over USB.

### Step 1: Read the current leveller state

On app startup or when connecting to a device, read all six leveller parameters. You have two options:

**Option A: Individual GET commands (6 transfers)**

```
GET 0xB5 -> enabled          (1 byte)
GET 0xB7 -> amount           (4 bytes, float)
GET 0xB9 -> speed            (1 byte)
GET 0xBB -> max_gain_db      (4 bytes, float)
GET 0xBD -> lookahead        (1 byte)
GET 0xBF -> gate_threshold_db (4 bytes, float)
```

**Option B: Bulk parameter GET (1 transfer, recommended)**

Issue `REQ_GET_ALL_PARAMS` (0xA0). The response is a `WireBulkParams` struct (2864 bytes). Parse the `WireLevellerConfig` at byte offset 2848 (the last 16 bytes of the payload).

### Step 2: Build the UI

Recommended UI controls:

| Parameter | UI control | Notes |
|-----------|-----------|-------|
| enabled | Toggle switch | On/off for the entire leveller |
| amount | Slider (0-100) | Label as percentage. Consider showing 0% as "Off" and 100% as "Maximum". |
| speed | 3-option selector | Labels: "Slow (Music)", "Medium", "Fast (Speech)" |
| max_gain_db | Slider (0-35) | Label in dB. Consider showing 0 as "No boost" |
| lookahead | Toggle switch | Add "(+10ms latency)" label. On by default; less critical with upward compression. |
| gate_threshold_db | Slider (-96 to 0) | Label in dBFS. Lower values = less gating. Consider "Advanced" placement. |

Suggested layout: Place the enable toggle prominently. Gray out or disable the other controls when the leveller is disabled. The amount slider is the primary control most users will interact with.

### Step 3: Send SET commands on user interaction

When the user changes a control, send the corresponding SET command immediately:

```
// User drags amount slider to 75.0
float amount = 75.0f;
uint8_t payload[4];
memcpy(payload, &amount, 4);  // little-endian on ARM/x86
usb_vendor_out(0xB6, payload, 4);

// User toggles enable on
uint8_t enable = 1;
usb_vendor_out(0xB4, &enable, 1);

// User selects "Fast" speed
uint8_t speed = 2;
usb_vendor_out(0xB8, &speed, 1);
```

**Important:** The firmware applies SET commands asynchronously. The vendor receive handler stores the value and sets a pending flag. The main loop processes the pending update on its next iteration (typically within 1-2ms). There is no acknowledgment -- if the transfer completes successfully at the USB level, the parameter was accepted.

### Step 4: Handle preset load events

When the user loads a preset (via `REQ_PRESET_LOAD` / 0x91), all DSP parameters change, including the leveller. After a preset load, re-read all leveller parameters to update the UI:

```
// After preset load completes:
GET 0xB5 -> update enable toggle
GET 0xB7 -> update amount slider
GET 0xB9 -> update speed selector
GET 0xBB -> update max gain slider
GET 0xBD -> update lookahead toggle
GET 0xBF -> update gate threshold slider
```

Or use a single bulk GET (0xA0) to refresh all parameters at once. This is the preferred approach since a preset load affects all DSP parameters, not just the leveller.

### Step 5: Handle bulk transfers

If your app uses bulk parameter transfers for configuration backup/restore:

**Reading (GET 0xA0):**
1. Parse the full `WireBulkParams` response.
2. Check `header.format_version`. If < 4, the `WireLevellerConfig` section is not present; use factory defaults for display.
3. If >= 4, read `WireLevellerConfig` at offset 2848.

**Writing (SET 0xA1):**
1. Populate `WireLevellerConfig` in the `WireBulkParams` struct.
2. Set `header.format_version = 4`.
3. Send the complete struct.
4. The firmware will validate the header and apply all parameters including the leveller.
5. After a bulk SET, re-read the state (or trust that the values you sent are now active).

### Step 6: Endianness

All numeric values in the wire protocol are **little-endian**, which is the native byte order on both ARM (the device) and x86/x64 (most host platforms). On little-endian hosts, you can use `memcpy` directly between float variables and byte buffers without conversion. On big-endian hosts (rare), byte-swap is required for float and multi-byte integer fields.

---

## 8. Platform Notes

### Both platforms supported

The Volume Leveller is available on both RP2040 and RP2350. The vendor commands, wire format, and preset storage are identical across platforms. The algorithm produces equivalent results, though the internal numeric representation differs.

### RP2350 (float pipeline)

- RMS envelope tracking: single-precision float (`float env_sq`).
- Gain computation: single-precision float throughout.
- Gain application: float multiply (`sample *= gain`).
- Lookahead buffer: `float[2][480]` -- 3840 bytes.
- Safety limiter: clamp to [-1.0f, +1.0f].

### RP2040 (Q28 fixed-point pipeline)

- RMS envelope tracking: Q28 fixed-point via `fast_mul_q28()` for the IIR update.
- Gain computation: **float** -- the per-block gain computer (log, exp, soft knee) runs in float using the Pico SDK's ROM float routines. This is acceptable because it runs once per block (~4ms), not per sample.
- Gain application: Q28 fixed-point via `fast_mul_q28()`.
- Linear interpolation: 64-bit intermediate to avoid overflow.
- Lookahead buffer: `int32_t[2][480]` -- 3840 bytes.
- Safety limiter: clamp to [-(1<<28), +(1<<28)].

### CPU impact

Less than 1% on both platforms. The per-sample work is a single multiply-accumulate (envelope) plus a multiply (gain application). The expensive math (log10, pow, exp) runs only once per block.

### BSS (RAM) impact

Approximately 3.9 KB on both platforms, dominated by the lookahead buffer:

| Component | Size | Notes |
|-----------|------|-------|
| `LevellerState` | ~3864 bytes | Includes lookahead buffer (2 x 480 x 4 = 3840 bytes), always allocated |
| `LevellerCoeffs` | 36 bytes | 9 float coefficients |
| `LevellerConfig` | ~16 bytes | 6 fields with padding |
| **Total** | **~3.9 KB** | |

The lookahead buffer is always allocated in BSS regardless of whether lookahead is enabled. This avoids dynamic allocation and ensures deterministic memory usage.

### Sample rate handling

Leveller coefficients (alpha values for envelope and gain smoothing) are computed relative to the current sample rate. When the sample rate changes (e.g., switching from 44.1 kHz to 48 kHz), `leveller_update_pending` is set, and coefficients are recomputed on the next main loop iteration.

The lookahead delay is fixed at 480 samples. At 48 kHz this is exactly 10ms; at 44.1 kHz it is approximately 10.9ms.

### Coefficient update flow

All coefficient updates are deferred to the main loop to avoid modifying shared state from the USB interrupt context:

```
USB vendor handler (interrupt context):
    leveller_config.param = new_value
    leveller_update_pending = true
    (optionally) leveller_reset_pending = true

Main loop (non-interrupt context):
    if leveller_update_pending:
        leveller_update_pending = false
        leveller_compute_coefficients(&coeffs, &config, sample_rate)
        if leveller_reset_pending:
            leveller_reset_pending = false
            leveller_reset_state(&state)
        leveller_bypassed = !config.enabled
```

The `leveller_bypassed` flag is a fast boolean checked in the audio callback before calling `leveller_process_block()`. When the leveller is disabled, there is zero overhead in the audio path.
