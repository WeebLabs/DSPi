# Loudness Compensation Specification

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7)*

## 1. Overview

Loudness Compensation counteracts the ear's loss of bass and treble sensitivity at low listening levels. Human hearing is not flat: as playback volume drops, perceived low-frequency and high-frequency content falls away faster than the midrange (the ISO 226:2003 equal-loudness contours). This feature applies a volume-keyed pair of shelving filters that boost the spectral extremes by exactly the amount the contours predict, so a mix turned down for late-night listening keeps its perceived tonal balance.

The compensation is keyed to the **user volume** (the same value the OS volume slider drives). At the reference level it applies no correction; as the user turns the volume down, the correction grows automatically. No user interaction is needed beyond enabling it.

### Key characteristics

- **ISO 226:2003 derived:** Correction amounts are computed from the standard's equal-loudness contour equations at 50 Hz (bass) and 10 kHz (treble), not from an arbitrary "smile" curve.
- **Volume-keyed:** A precomputed 61-step coefficient table maps every user-volume position (-60..0 dB, 1 dB steps) to a filter pair. Volume changes re-select coefficients instantly; no computation happens in the audio path.
- **Per-output processing with an output mask:** The filters run independently on each *output* channel selected by `loudness_output_mask`. Compensate headphones but not speakers, all 9 outputs of a 7.1 system, or any other combination. CPU is spent only on selected outputs.
- **Works in every input mode:** Stereo, S/PDIF, I2S, and multichannel USB (4/6/8 channels). Earlier firmware bypassed loudness entirely when a multichannel USB alt was active; that restriction is gone.
- **Two shelving filters per output:** Low shelf at 200 Hz, high shelf at 6 kHz, both Q = 0.707. Filter gains vary with volume; corner frequencies do not.
- **Zero added latency:** The shelves are minimum-phase IIR filters with no sample delay. Inter-output sample alignment is untouched in every mode and transition.
- **Free when idle:** At or near the reference volume the computed shelf gains round to 0 dB and the filters self-bypass; an enabled output then costs almost nothing.

### Signal chain position

Loudness runs **per output channel**, after the output gain stage and before the per-output delay, inside PASS 5-7 of the pipeline:

```
Input source (USB / S/PDIF / I2S)
    |
PASS 1:   Preamp + per-input volume
    |
PASS 2:   Per-input EQ + metering
    |
PASS 2.5: Volume Leveller (input-side, mask-driven)
    |
PASS 3:   Crossfeed (stereo input modes only)
    |
PASS 4:   Matrix Mixing (fan-out to output channels)
    |
PASS 5-7: Per-output crossover + EQ
              |
          Output gain (user vol x master vol x matrix gain, ramped)
              |
          Loudness Compensation  <-- HERE (masked outputs only)
              |
          Per-output delay
              |
          Peak metering + output encoding (S/PDIF / I2S / PDM)
```

Post-gain placement is deliberate:

1. **Headroom (RP2040):** Loudness only boosts when the volume is low, i.e. immediately after the signal has been heavily attenuated by the gain stage. Boosting the attenuated signal keeps the Q28 fixed-point path far from saturation. (On earlier firmware the filters ran pre-gain on potentially full-scale input.)
2. **Truthful meters:** The per-output peak/clip meters run after loudness, so a boost that approaches clipping is visible to the host.
3. **Equivalence:** The shelves, crossover, EQ, and delay are all LTI, so applying loudness after the crossover produces the same response as applying it before; nothing is lost by the placement.

Because processing happens after the matrix, an output derived from several inputs is compensated exactly once, and outputs excluded by the mask are exactly untouched.

### Platform summary

| | RP2040 | RP2350 |
|---|---|---|
| Output channels | 5 (2 stereo slots + PDM) | 9 (4 stereo slots + PDM) |
| Filter implementation | Direct Form 2 biquads, Q28 fixed point | Cytomic SVF shelves, float |
| Mask default | 0xFFFF (all outputs) | 0xFFFF (all outputs) |
| Multichannel inputs | n/a (stereo only) | supported (loudness unaffected by input count) |
| Per-output state RAM | 16 B/output (80 B total) | 16 B/output (144 B total) |

---

## 2. Theory of operation

### 2.1 ISO 226:2003 model

For a frequency with tabulated constants (threshold `Tf`, exponent `af`, transfer level `Lu`), the standard gives the SPL required to reach a loudness of `Ln` phon:

```
B   = 0.4 * 10^((Tf + Lu)/10 - 9)
Af  = 4.47e-3 * (10^(0.025*Ln) - 1.15) + B^af
Lp  = (10/af) * log10(Af) - Lu + 94
```

The firmware evaluates this at two frequencies only:

| Frequency | Tf | af | Lu | Drives |
|-----------|-----|------|------|--------|
| 50 Hz | 44.0 | 0.432 | -15.9 | low shelf gain |
| 10 kHz | 13.9 | 0.271 | -10.7 | high shelf gain |

### 2.2 Compensation amount

For a volume-reduced listening level, the required correction at each frequency is the amount by which perceived level at that frequency drops *beyond* the flat volume reduction:

```
effective_phon = clamp(ref_spl + volume_db, 20, ref_spl)   // volume_db is <= 0
flat_change    = effective_phon - ref_spl                   // the volume knob's effect at 1 kHz
freq_change    = Lp(f, effective_phon) - Lp(f, ref_spl)     // its effect at 50 Hz / 10 kHz
gain_db        = (freq_change - flat_change) * intensity_pct / 100
```

At `volume_db = 0` the correction is zero. As volume drops the correction grows, more steeply at 50 Hz than at 10 kHz (matching the contours). The 20-phon floor stops the correction growing without bound at extreme attenuation.

### 2.3 The volume-keyed coefficient table

The firmware precomputes filter coefficients for all 61 user-volume steps (`index 0..60` = `-60..0 dB`) into a double-buffered RAM table:

- `loudness_recompute_table()` fills the inactive buffer, then swaps the active-table pointer atomically. The audio path never sees a half-written table.
- Each entry holds the coefficients for both shelves at that volume step, plus a per-filter `bypass` flag set when the computed gain is below 0.01 dB.
- Recomputes are deferred to the main loop (`loudness_recompute_pending`) and are triggered by: boot, `SET_LOUDNESS_REF`, `SET_LOUDNESS_INTENSITY`, sample-rate change, preset load, bulk-parameter apply, and factory reset. A recompute finishes within one main-loop pass (milliseconds); audio continues on the old table until the swap.
- Volume changes do **not** recompute anything; they just re-point `current_loudness_coeffs` at a different row (done in lock-step with the gain change by `apply_vol_index_to_audio()`, so compensation always matches the gain actually applied).

The volume that keys the table is the **user-perceived volume**: the UAC1 host slider, the `ROUTE.USER_VOLUME` vendor command, and LG Sound Sync all drive the same underlying index. Master volume (`ROUTE.MASTER_VOLUME`), per-channel preamp, and matrix gains do NOT key loudness; they are treated as fixed system calibration.

### 2.4 Filters

| Filter | Corner | Q | Gain source |
|--------|--------|-----|-------------|
| Low shelf | 200 Hz | 0.707 | ISO 226 correction at 50 Hz |
| High shelf | 6 kHz | 0.707 | ISO 226 correction at 10 kHz |

Both filters follow the RBJ Audio-EQ-Cookbook shelf response. RP2350 realizes them as Cytomic (Simper) SVF shelves in single-precision float; RP2040 as Direct Form 2 biquads in Q28 fixed point. The two implementations are matched in response; coefficients are recomputed for the active sample rate.

---

## 3. Per-output processing and the output mask

### 3.1 Mask semantics

`loudness_output_mask` is a 16-bit value; **bit k enables compensation on output channel k**:

| Bit | RP2350 output | RP2040 output |
|-----|---------------|---------------|
| 0 | Slot 1 L (output 1) | Slot 1 L (output 1) |
| 1 | Slot 1 R (output 2) | Slot 1 R (output 2) |
| 2 | Slot 2 L (output 3) | Slot 2 L (output 3) |
| 3 | Slot 2 R (output 4) | Slot 2 R (output 4) |
| 4 | Slot 3 L (output 5) | PDM subwoofer |
| 5 | Slot 3 R (output 6) | ignored |
| 6 | Slot 4 L (output 7) | ignored |
| 7 | Slot 4 R (output 8) | ignored |
| 8 | PDM subwoofer | ignored |
| 9-15 | ignored | ignored |

"Slot" is an output slot in the S/PDIF/I2S sense (each slot carries a stereo pair and may be S/PDIF or I2S). Ignored bits are stored and returned by GET exactly as written, but have no effect. Default is `0xFFFF` (everything compensated), which reproduces the pre-mask behavior.

The ADAT bulk output (RP2350) mirrors the finalized output buffers, so ADAT channels inherit whatever compensation their source outputs received. There are no separate ADAT mask bits.

### 3.2 When an output is actually processed

An output's filters run for a packet only when ALL of the following hold:

1. Loudness is enabled and the coefficient table is ready (`current_loudness_coeffs != NULL`).
2. The output's mask bit is set.
3. The output is enabled in the matrix mixer.
4. The output's composite gain is not zero for the whole packet (muted outputs are skipped, so a muted output can never emit a decaying filter tail).
5. The output is not carrying a RAW test signal (`siggen` RAW mask), so measurement signals stay bit-exact.

Whenever any of those conditions fails, that output's filter state is **cleared** instead. This makes re-entry deterministic: enabling a mask bit (or unmuting, or re-enabling an output) always starts the filters from silence, never from stale state. Within one filter run, a shelf whose table entry is flagged `bypass` (0 dB at this volume) is skipped and its state cleared too.

### 3.3 Consistency and alignment guarantees

- **Sample alignment (hard guarantee):** the filters are in-place IIR with zero sample delay. Masked and unmasked outputs stay sample-aligned in every mode and across every transition (mask writes, enable toggles, preset loads, rate changes). Nothing about this feature moves audio in time.
- **Phase:** shelving filters are minimum-phase and impose a small phase shift near their corners. Outputs that sum acoustically (e.g. a sub crossed over against mains) should share the same mask setting, otherwise the summed response through the crossover region will have a level and phase step. This is a configuration guideline, not a firmware constraint; see the recipes in section 8.
- **Packet-consistent view:** the coefficient pointer and mask are snapshotted once per audio packet and shared with both CPU cores, so a mask or volume write can never split a packet's outputs across two settings.
- **Dual-core:** outputs are statically partitioned between the cores (Core 0: outputs 0-1; Core 1: the rest). Each output's filter state is owned by exactly one core; there is no locking in the audio path.

### 3.4 Timing of changes

| Change | Takes effect |
|--------|--------------|
| `SET_LOUDNESS` (enable/disable) | immediately (next packet); coefficients re-keyed to the live volume index in the handler |
| `SET_LOUDNESS_MASK` | next audio packet (sub-millisecond); no recompute, no mute, no pipeline reset |
| `SET_LOUDNESS_REF` / `SET_LOUDNESS_INTENSITY` | next main-loop pass (table recompute + atomic swap, milliseconds) |
| Volume change | same packet the new gain applies (coefficient row re-selected in lock-step) |

Mask and enable changes switch the correction in or out as a step (there is no crossfade), exactly like the pre-existing enable toggle. At typical late-night correction levels this is a small tonal step, not a click; state clearing guarantees no transient from stale filter memory.

---

## 4. Parameters

### 4.1 enabled

| Property | Value |
|----------|-------|
| **Type** | bool (1 byte on wire) |
| **Range** | 0 (off) or 1 (on) |
| **Default** | 0 (disabled) |
| **Command** | `DYN.LOUDNESS_ENABLE` (page 0x07, command 0x30), GET and SET |
| **Payload** | exactly 1 byte: `0x00` = disabled, nonzero = enabled |

Master switch. Disabling costs zero CPU (a per-packet pointer check). Enabling immediately re-selects coefficients for the *currently active* volume index (whether that volume came from the USB slider, the vendor channel, or LG Sound Sync), so the correction is right from the first packet.

### 4.2 ref_spl (reference SPL)

| Property | Value |
|----------|-------|
| **Type** | float (IEEE 754 single, little-endian) |
| **Range** | 40.0 to 100.0 (finite values outside are clamped by the firmware, NaN and infinity are refused with `VC_ERR_VALUE`) |
| **Default** | 87.0 |
| **Command** | `DYN.LOUDNESS_REF` (page 0x07, command 0x31), GET and SET |
| **Payload** | exactly 4 bytes: little-endian IEEE 754 float |

The calibration anchor: the acoustic SPL (in dB, treated as phon at 1 kHz) that the listener experiences at **0 dB user volume**. At that level no correction is applied; every dB of volume reduction below it is compensated per the contours. 87 dB SPL corresponds to a fairly loud home playback level; users with quieter systems should lower it. GET returns the clamped, stored value.

### 4.3 intensity_pct

| Property | Value |
|----------|-------|
| **Type** | float (IEEE 754 single, little-endian) |
| **Range** | 0.0 to 200.0 (finite values outside are clamped by the firmware, NaN and infinity are refused with `VC_ERR_VALUE`) |
| **Default** | 100.0 |
| **Command** | `DYN.LOUDNESS_INTENSITY` (page 0x07, command 0x32), GET and SET |
| **Payload** | exactly 4 bytes: little-endian IEEE 754 float |

Scales the computed correction. 100% applies the ISO-derived amount exactly; 50% halves it (in dB); 200% doubles it for listeners who prefer an exaggerated contour. 0% is equivalent to disabled (but still burns the bypass check). GET returns the clamped, stored value.

### 4.4 output_mask

| Property | Value |
|----------|-------|
| **Type** | uint16 (little-endian) |
| **Range** | any 16-bit value; bits above the platform's output count are ignored |
| **Default** | 0xFFFF (all outputs compensated) |
| **Command** | `DYN.LOUDNESS_MASK` (page 0x07, command 0x33), GET and SET |
| **Payload** | exactly 2 bytes: little-endian uint16, bit k = output channel k (see section 3.1) |

Selects which output channels the volume-keyed filters process. Only selected outputs cost CPU. Changing the mask does not trigger a table recompute, mute, or pipeline reset; it applies from the next packet. Newly selected outputs start from cleared filter state.

Firmware support detection: the mask is in the bulk payload from wire format **V19** (bulk header `format_version >= 19`). Every firmware that speaks the paged protocol (1.1.7 and later) has `DYN.LOUDNESS_MASK`. Firmware 1.1.6 and older does not answer the paged protocol at all (`vendor_command_overhaul_plan.md` section 5.14).

---

## 5. Vendor protocol details

The four parameters are four commands in the `DYN` page (0x07) of the paged protocol (see `../commands.md`). Each command is one number for both directions: writes are `bmRequestType 0x41`, reads are `bmRequestType 0xC1`. `bRequest` is the command number, `wIndex` is `(0x07 << 8) | 2`, `wValue` is 0 and the data stage is the payload or the response. The same commands work over the UART and I2C control transports (no USB-only gating).

| Command | Page | Cmd | Dir | Address (`wValue`) | Size |
|---|---|---|---|---|---|
| `DYN.LOUDNESS_ENABLE` | 0x07 | 0x30 | GET, SET | none (0) | GET 1, SET 1 |
| `DYN.LOUDNESS_REF` | 0x07 | 0x31 | GET, SET | none (0) | GET 4, SET 4 |
| `DYN.LOUDNESS_INTENSITY` | 0x07 | 0x32 | GET, SET | none (0) | GET 4, SET 4 |
| `DYN.LOUDNESS_MASK` | 0x07 | 0x33 | GET, SET | none (0) | GET 2, SET 2 |

A SET with the wrong payload length is `VC_ERR_LENGTH`, a non-zero `wValue` is `VC_ERR_ADDRESS`, and a NaN or infinite float is `VC_ERR_VALUE` (nothing is stored). A rejected SET STALLs on USB and the reason is readable from `SYS.LAST_RESULT`.

Byte-level examples (page 0x07):

```
Enable loudness:                 SET 0x30, payload [01]
Set reference to 80 dB SPL:      SET 0x31, payload [00 00 A0 42]      (80.0f LE)
Set intensity to 75%:            SET 0x32, payload [00 00 96 42]      (75.0f LE)
Headphones on outputs 1-2 only:  SET 0x33, payload [03 00]            (mask 0x0003)
Stereo mains + PDM sub (RP2350): SET 0x33, payload [03 01]            (mask 0x0103)
Everything:                      SET 0x33, payload [FF FF]            (mask 0xFFFF)
Read mask back:                  GET 0x33 -> 2 bytes LE
```

### 5.1 Notifications

Every SET emits a `PARAM_WRITE` notification (see `notification_protocol_v2_spec.md`) so other attached controllers can stay in sync. The offsets are into `WireBulkParams` and are stable across wire versions (the `global` section directly follows the 16-byte header):

| Parameter | Offset | Length |
|-----------|-------:|-------:|
| loudness_enabled | 21 | 1 |
| loudness_output_mask | 22 | 2 |
| loudness_ref_spl | 24 | 4 |
| loudness_intensity_pct | 28 | 4 |

### 5.2 Bulk parameters

The `global` section of `WireBulkParams` carries all four parameters:

```
offset 16: f32 preamp_gain_db
offset 20: u8  bypass
offset 21: u8  loudness_enabled
offset 22: u16 loudness_output_mask     (V19+; reserved bytes before V19)
offset 24: f32 loudness_ref_spl
offset 28: f32 loudness_intensity_pct
```

The mask occupies what were reserved bytes, so the section (and the whole payload) did not change size at V19. A `STORE.ALL_PARAMS` SET applies the mask along with everything else, and its GET returns the live value.

---

## 6. Persistence

- **User presets (flash):** all four parameters are stored per preset. The mask was tail-appended at slot version **V26**; presets saved by older firmware (V21-V25) load with the mask defaulted to 0xFFFF. Preset save and load commands: `STORE.PRESET_SAVE` and `STORE.PRESET_LOAD` (see `user_presets_spec.md`).
- **Factory reset:** enabled = 0, ref_spl = 87.0, intensity = 100.0, mask = 0xFFFF.
- Like every other parameter, the mask takes effect immediately on write but persists only when the user saves a preset.

---

## 7. Interactions with other features

| Feature | Interaction |
|---------|-------------|
| User volume (`ROUTE.USER_VOLUME`) / UAC1 slider / LG Sound Sync | Keys the compensation. All three drive the same volume index, so compensation tracks whichever is in control. |
| Master volume (`ROUTE.MASTER_VOLUME`) | Does NOT key compensation; it is calibration, not listening level. Set `ref_spl` with master volume at its normal value. |
| Volume Leveller | Independent (input-side vs output-side). The leveller's detector sees the *uncompensated* signal, so loudness boost cannot pump the leveller. Both can be enabled together. |
| Crossfeed | Independent; crossfeed runs input-side in stereo modes. |
| Per-output crossover / EQ / delay | Commute with loudness (all LTI); the audible result is identical to loudness-before-crossover. |
| Signal generator | Outputs in the siggen RAW mask bypass loudness so measurement signals are unmodified. Non-RAW generator output is compensated like normal audio. |
| Output mute / disable | Skips processing and clears state; a muted output emits true silence, never a filter tail. |
| ADAT output (RP2350) | Mirrors compensated outputs 1-8; no separate mask. |
| Peak/clip metering | Per-output meters read post-loudness; watch them when running high intensity at very low volume. |
| Preset mute / pipeline resets | Loudness adds no state that survives incorrectly; filter state is cleared on any skip and the coefficient table swap is atomic. |

---

## 8. Configuration recipes

**Headphones only (headphones on slot 1, speakers on slot 2):**
mask = 0x0003. Speakers stay untouched; the headphone pair gets the full contour correction. This is the canonical "compensate my low-level listening chain only" setup.

**7.1 night mode (RP2350, 8 outputs + none on PDM):**
mask = 0x00FF. All eight main channels are compensated identically, preserving the surround mix balance at low volume. Add bit 8 (mask 0x01FF) if the sub feed is the PDM output.

**Stereo mains + subwoofer via crossover:**
Include the sub's output bit alongside the mains (e.g. mask 0x0103 for outputs 1-2 plus PDM on RP2350). If the sub were excluded, the low-shelf boost would stop abruptly at the crossover frequency and the summed response would step there; including it keeps the transition coherent (identical filters on both sides of a crossover commute through it).

**Zone feed that must stay flat (recording tap, second room with its own volume):**
Clear that output's bit. It receives the mix at unit-calibrated level regardless of the main listening correction.

---

## 9. CPU and memory cost

- **CPU:** two shelf filters per selected output per sample; roughly the cost of two extra PEQ bands per output, on the order of 1% core load per output at 48 kHz (double at 96 kHz). Cost scales linearly with the number of mask bits set on enabled outputs and is zero for cleared bits. At or near reference volume, both shelves self-bypass and an enabled output costs only the loop check.
- **RAM:** per-output filter state is 16 bytes per output (80 B on RP2040, 144 B on RP2350). The coefficient tables (double-buffered, 61 steps x 2 filters) predate this feature and are unchanged: about 5.9 KB on RP2040, 6.8 KB on RP2350.
- **Flash/wire:** +2 bytes in the preset slot (V26); no change to the bulk payload size (V19 reused reserved bytes).

---

## 10. Host application implementation guide

A minimal, correct integration:

1. **Detect support.** Read the bulk header (`STORE.ALL_PARAMS` GET, first 16 bytes) and check `format_version`. `>= 19` means the output mask exists; older firmware supports only enable/ref/intensity (and applies loudness to everything, stereo modes only).
2. **Read state on connect.** Either parse the `global` section of the bulk payload (offsets in section 5.2) or issue the four GETs (`DYN.LOUDNESS_ENABLE`, `DYN.LOUDNESS_REF`, `DYN.LOUDNESS_INTENSITY`, `DYN.LOUDNESS_MASK`).
3. **Build the UI.**
   - Enable toggle.
   - Reference SPL slider, 40-100 dB, default 87, step 1 dB. Label it as "SPL at full volume" and consider a calibration hint (play pink noise at 0 dB volume, measure, enter the number).
   - Intensity slider, 0-200%, default 100.
   - One checkbox per output channel for the mask, labeled with the user's output names (readable via `ROUTE.CHANNEL_NAME`) and gated to the platform's output count (`SYS.PLATFORM` or the bulk header's `num_output_channels`). Warn or auto-group when the user splits a bass-managed pair (see section 8).
4. **Write changes.** Fire the SET for the parameter that changed; no ordering constraints, no busy window. Read back with a GET of the same command if you want confirmation, or rely on the notification.
5. **Stay in sync.** Subscribe to `PARAM_WRITE` notifications and patch your model at the offsets in section 5.1, so changes made by another controller are reflected live. A preset load does not renotify individual parameters; it emits `PRESET_LOADED` / bulk-invalidated events, after which the app should re-read the bulk payload (or the four GETs).
6. **Persistence.** Remind the user that settings live in presets; call `STORE.PRESET_SAVE` to persist.

Common mistakes to avoid:

- Do not key any UI logic off master volume; loudness ignores it by design.
- Do not assume mask bits above the output count read back as zero; they are stored verbatim. Mask them off in the app when displaying.
- A STALL from a loudness SET is a real refusal (bad length, `wValue` or non-finite float). Read `SYS.LAST_RESULT` for the reason.
- Send the mask as little-endian bytes (`[low, high]`); it is not a bitfield of pairs, it is one bit per output channel.

---

## 11. Version history

| Date | Change |
|------|--------|
| 2026-11-16 | Firmware 1.1.7: the four parameters moved to the paged protocol as `DYN.LOUDNESS_ENABLE`, `DYN.LOUDNESS_REF`, `DYN.LOUDNESS_INTENSITY` and `DYN.LOUDNESS_MASK`. |
| 2026-07-09 | Per-output rework: loudness moved from the stereo input bus (pre-matrix, stereo-input modes only) to per-output processing after the output gain; added `loudness_output_mask` (1.1.6 opcodes 0xFA/0xFB, now `DYN.LOUDNESS_MASK`, wire V19, preset slot V26); multichannel input modes now supported; per-output state with deterministic clearing. |
| earlier | Stereo-only implementation: two shelves on the input bus, enable/ref/intensity (1.1.6 opcodes 0x58-0x5D), skipped when a multichannel USB alt was active. |
