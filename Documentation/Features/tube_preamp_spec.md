# Tube Preamp Emulation Specification

## 1. Overview

The tube preamp ("tube") adds valve-style harmonic colour, compression and, optionally, output-transformer character to any selected output channel. It is a per-output effect module in the same family as Psychoacoustic Bass and the Subharmonic Synthesizer: one global configuration, a 16-bit output mask, shared double-buffered coefficients, per-output filter state, zero added latency.

The emulation is a characterful approximation, not a circuit simulation. A single static waveshaper with an adjustable operating point (bias), separate positive and negative knees (asymmetry), a knee-hardness blend, and a slow supply-sag envelope reproduces the audible differences between the popular preamp and power tubes. A tube-type selector loads a row of those four character parameters; the row is a preset, never a hidden term in the audio path, so the kernel cost is identical for every type.

Both platforms are supported. RP2350 runs the kernel in float; RP2040 runs the same kernel in Q28 fixed point through `fast_mul_q28`.

### Key characteristics

- **Per-output processing with one global parameter set.** Every output selected by `output_mask` runs the same configuration. Different settings per speaker are handled by masking and presets, as for loudness and psybass.
- **Single shaper shape, sixteen tube styles.** Triodes, pentodes, push-pull and single-ended power stages differ in this model only by bias, knee asymmetry, knee hardness and sag depth. Selecting a tube type copies its row into those four parameters and the user's drive, mix and transformer settings are left alone.
- **Level-dependent even harmonics.** The bias term shifts the operating point on the curve, so second-harmonic content rises with signal level exactly as it does in a real single-ended stage. Knee asymmetry adds further even-order content at heavy drive.
- **Supply sag.** A one-pole envelope of how far into the knee the stage is being driven pulls the drive down slowly. A rectifier selector presets the sag depth scale and attack and release times.
- **Transformer stage.** A one-pole low-band split at a settable corner, a soft saturator on that low band only, and a one-pole high-frequency rolloff. Because transformer core saturation scales with voltage over frequency, a 6 dB per octave split is the physically correct slope.
- **Zero added latency.** Every stage is memoryless or a one-pole IIR. The dry path is never delayed. Inter-output-slot sample alignment is untouched whether the effect is on, off, or masked per channel.
- **Saturation meter.** A per-output decaying peak of the knee drive (0 = linear, full scale = fully clipped) is readable by the host.

### Signal flow (per selected output channel)

```
   x ----------------------------------------------------------+ dry
   |                                                            |
   +--> [sag] m_eff = m * (1 - depth * env)                     |
   |                                                            |
   +--> t = m_eff * x + b                                       |
        t < 0: t *= ratio_n                                     |
        t = clamp(t, -1, +1)                                    |
        v = p(t) * (t >= 0 ? s_p : s_n)     p = blended poly    |
        v -= v0                            static bias offset   |
        y = DC_block(v)                    one-pole HP at 5 Hz  |
        [transformer, optional]                                 |
          low  = LP1(y, xfmr_lf_hz)                             |
          high = y - low                                        |
          y    = high + sat(low)           cubic soft clip      |
          y    = LP1(y, xfmr_hf_hz)                             |
   out = (1 - mix) * x + mix * trim * y  <-----------------------+

   env   = one-pole follower of |t| (attack / release from rectifier row)
   meter = max(|t|, meter * decay)      300 ms decay
```

- `m = 10^(drive_db/20)`. The positive knee is fixed at t = 1, so drive alone sets how hard the stage is driven and full-scale input at 0 dB drive just reaches the knee.
- `b = bias_pct / 200` (range -0.5 .. +0.5 in knee units).
- `ratio_n = 10^(-asym_db/20)`: a positive `asym_db` makes the negative half clip later (softer cut-off side, harder grid-conduction side).
- `p(t) = t * (c1 + t^2 * (c3 + t^2 * c5))` with `h = hardness_pct / 100`, `c1 = 1.5 + 0.375 h`, `c3 = -0.5 - 0.75 h`, `c5 = 0.375 h`. At h = 0 this is the cubic `1.5 t - 0.5 t^3`; at h = 1 it is the quintic `(15 t - 10 t^3 + 3 t^5) / 8`. Both reach exactly 1 at t = 1 with zero slope, so the clamp is continuous in value and slope for every hardness.
- `s_p = 1 / c1` and `s_n = 10^(asym_db/20) / c1` normalise the small-signal gain to unity so hardness does not change the level of clean material.
- `v0 = p(b) * s` is the shaper's output for silence, subtracted so enabling the effect produces no step. The DC blocker removes the level-dependent offset asymmetric clipping creates.
- Sag: `depth = clamp(sag_pct / 100 * rect_scale, 0, 0.9)`; `env += (|t| - env) * (|t| > env ? a_att : a_rel)`. Solid-state rectifier disables sag entirely.
- Transformer saturator: `sat(low) = 1.5 u - 0.5 u^3` with `u = clamp(low / ks, -1, 1)`, scaled by `ks / 1.5`; `ks = 10^(-0.18 * xfmr_sat_pct / 20)` so 100 % puts the low-band knee at -18 dBFS.
- `mix = mix_pct / 100`, `trim = 10^(trim_db/20)`.

### Signal chain position

Tube runs **per output channel, post-matrix, after psybass, pre-crossover, pre-output-EQ**:

```
PASS 4:   Matrix Mixing (fan-out to output channels)
PASS 4.5: Crossfeed (per output pair)
          Subharmonic Synthesizer (per output, masked)
          Psychoacoustic Bass (per output, masked)
             |
          Tube Preamp   <-- HERE (per output, masked)
             |
          Crossover -> Per-Output PEQ -> Gain/Volume -> Loudness -> Delay
             |
          Output Encoding (S/PDIF, I2S, ADAT, PDM)
```

Pre-crossover placement puts the saturator where a real preamp sits, ahead of an active crossover: a subwoofer output saturates the full-band program and then low-passes the result, a tweeter output keeps the harmonics the bass generated. Because tube runs pre-gain its character does not change with volume, and per-output PEQ shapes its output like everything else.

---

## 2. Parameters

Every parameter is addressed by a small integer index through one indexed SET/GET pair (section 3). On the wire every parameter is a little-endian IEEE 754 float32, including the boolean, mask and enumerated ones. Booleans are non-zero-is-true. The mask and the enums are rounded to the nearest integer and then clamped. Floats are clamped to their range. A NaN is ignored and the SET reports success. A GET after a SET returns the stored value.

| Index | Name | Type | Range | Default | Recompute |
|---|---|---|---|---|---|
| 0 | `enabled` | bool | 0 / 1 | 0 | yes |
| 1 | `output_mask` | uint16 | 0x0000 .. 0xFFFF | 0xFFFF | no (read live) |
| 2 | `tube_type` | enum | 0 .. 16 | 1 (12AX7) | yes (loads a row) |
| 3 | `drive_db` | float | 0 .. 24 dB | 6 | yes |
| 4 | `bias_pct` | float | -100 .. +100 % | 30 | yes |
| 5 | `asym_db` | float | -12 .. +12 dB | 3 | yes |
| 6 | `hardness_pct` | float | 0 .. 100 % | 40 | yes |
| 7 | `sag_pct` | float | 0 .. 100 % | 30 | yes |
| 8 | `rectifier` | enum | 0 .. 3 | 1 (GZ34) | yes |
| 9 | `xfmr_enabled` | bool | 0 / 1 | 0 | yes |
| 10 | `xfmr_lf_hz` | float | 20 .. 300 Hz | 80 | yes |
| 11 | `xfmr_sat_pct` | float | 0 .. 100 % | 30 | yes |
| 12 | `xfmr_hf_hz` | float | 2000 .. 20000 Hz | 20000 | yes |
| 13 | `mix_pct` | float | 0 .. 100 % | 100 | yes |
| 14 | `trim_db` | float | -12 .. +12 dB | 0 | yes |

Defaults for indices 4 to 7 are the 12AX7 row.

### 2.1 enabled

Master switch. When off, `current_tube_coeffs` is NULL and every output's state is reset; the audio path costs nothing beyond the per-packet pointer check.

### 2.2 output_mask

Bit k selects output channel k (RP2350: 0..7 S/PDIF, 8 PDM; RP2040: 0..3 S/PDIF, 4 PDM). Bits above the platform's output count are ignored. Read live each packet, no recompute. A masked-off, muted, disabled or RAW-signal-generator output has its state reset so it re-enters cleanly.

### 2.3 tube_type

| Value | Tube | Style | bias_pct | asym_db | hardness_pct | sag_pct |
|---|---|---|---|---|---|---|
| 0 | Custom | Knobs as set, no row applied | | | | |
| 1 | 12AX7 / ECC83 | High-gain preamp triode; the default | 30 | 3 | 40 | 30 |
| 2 | 5751 | Cooler 12AX7 | 25 | 3 | 35 | 25 |
| 3 | 12AT7 / ECC81 | Medium-gain driver, more odd-order | 15 | 2 | 55 | 20 |
| 4 | 12AY7 | Tweed front end, gentle | 20 | 4 | 25 | 30 |
| 5 | 12AU7 / ECC82 | Clean line stage | 15 | 5 | 20 | 15 |
| 6 | 6SN7 | Octal hi-fi line stage, sweet | 20 | 6 | 15 | 20 |
| 7 | 6SL7 | Octal high-mu, rounder knee | 30 | 3 | 30 | 30 |
| 8 | 6DJ8 / ECC88 / 6922 | Clean, hard when pushed | 10 | 2 | 60 | 10 |
| 9 | EF86 / 6267 | Pentode preamp, symmetric bite | 5 | 0 | 75 | 25 |
| 10 | 6SJ7 | Octal pentode, softer than EF86 | 8 | 1 | 65 | 30 |
| 11 | EL84 / 6BQ5 | Push-pull power, chimey | 0 | 0 | 50 | 45 |
| 12 | EL34 | Push-pull power, mid crunch, deep sag | 0 | 0 | 60 | 55 |
| 13 | 6L6 / 5881 | Push-pull power, tight | 0 | 0 | 55 | 35 |
| 14 | 6V6 | Push-pull power, early breakup, heavy sag | 0 | 0 | 35 | 60 |
| 15 | KT88 / 6550 | Push-pull hi-fi power, near linear | 0 | 0 | 45 | 20 |
| 16 | 300B / 2A3 | Single-ended DHT, pure even harmonics | 35 | 6 | 10 | 25 |

**Apply semantics.** Setting `tube_type` to 1..16 copies that row into `bias_pct`, `asym_db`, `hardness_pct` and `sag_pct`, stores the type, and raises the recompute flag. Each changed field emits its own change notification. Setting it to 0 stores 0, leaves the four knobs untouched, and still raises the recompute flag (a harmless no-op recompute). Setting any of indices 4..7 to a value different from the current one resets `tube_type` to 0 (Custom) and notifies that byte; a SET that lands on the value already stored leaves the type alone. Bulk apply and preset load restore the stored fields verbatim with no row lookup, so a preset saved as "12AX7" reloads as 12AX7 even if the row table changes in a later firmware.

Push-pull power-tube styles have zero bias and asymmetry because a push-pull stage cancels even harmonics by construction; their character comes from hardness and sag, and from the transformer stage, which the host should suggest enabling for them.

### 2.4 drive_db

Linear gain ahead of the shaper. At 0 dB a full-scale input just reaches the knee. On RP2040 the drive coefficient is stored in Q24 rather than Q28 so the full 24 dB range fits the fixed-point ceiling. The kernel clamps the shaper input to +/-4.0 (+12 dBFS) first so the drive product cannot wrap; the dry path is never clamped. The clamp is lossless except for inputs beyond +12 dBFS at 0 dB drive with maximum bias and asymmetry, where the negative half saturates slightly early.

### 2.5 bias_pct

Operating-point offset in knee units, `b = bias_pct / 200`. Positive values produce the classic "warm" second harmonic that grows with level. Negative values give the same magnitude with opposite polarity of the even-order products, which matters only when mixing with the dry signal.

### 2.6 asym_db

Where the negative half clips relative to the positive half. `+6` means the negative half reaches its knee 6 dB later. Zero is symmetric.

### 2.7 hardness_pct

Blend from the cubic soft knee (0) to the quintic hard knee (100). Small-signal gain is normalised so this never changes clean level.

### 2.8 sag_pct

Depth of the supply-sag compression before the rectifier scale is applied. Effective depth is clamped to 0.9 so gain never falls to zero.

### 2.9 rectifier

| Value | Style | Depth scale | Attack | Release |
|---|---|---|---|---|
| 0 | Solid state | 0 (sag off) | | |
| 1 | GZ34 / 5AR4 | 0.6 | 5 ms | 120 ms |
| 2 | 5U4 | 1.0 | 8 ms | 200 ms |
| 3 | 5Y3 | 1.3 | 10 ms | 300 ms |

### 2.10 xfmr_enabled

Enables the transformer stage. When off, the transformer filters are skipped entirely and their state is zero.

### 2.11 xfmr_lf_hz

Corner of the one-pole split that feeds the low-band saturator.

### 2.12 xfmr_sat_pct

Low-band saturation amount. Maps linearly to a knee from 0 dBFS (0 %) to -18 dBFS (100 %). The -18 dB floor is the Q28 ceiling for the reciprocal knee coefficient on RP2040 and is shared by both platforms. At 0 % the low band still passes through the cubic below its full-scale knee, so it is gently shaped rather than bit-exact; turn `xfmr_enabled` off for a linear low band.

### 2.13 xfmr_hf_hz

One-pole high-frequency rolloff after the saturator. 20000 Hz is treated as bypass (coefficient exactly 1.0).

### 2.14 mix_pct

Dry/wet blend. The dry path is the untouched input, sample-aligned with the wet path.

### 2.15 trim_db

Output level applied to the wet path only.

### 2.16 Saturation meter (read-only)

Per-output decaying peak of `|t|` after the clamp, on the status-packet scale 0..32767 where 32767 means the stage was fully clipped. 300 ms decay, so a meter widget can be polled at 10 to 20 Hz. Runtime only: no wire, slot or notification presence.

---

## 3. Vendor Command Transport

### USB (primary transport)

Vendor control requests on the DSPi vendor interface, as for every other module. UART and I2C reach the same handlers through the shared dispatcher with no per-command table to update.

### Command summary

| Command | Code | Dir | wValue | Payload / Response |
|---|---|---|---|---|
| `REQ_SET_TUBE_PARAM` | 0x3E | OUT | parameter index (0..14) | 4-byte float32 LE |
| `REQ_GET_TUBE_PARAM` | 0x3F | IN | parameter index (0..14) | 4-byte float32 LE |
| `REQ_GET_TUBE_METER` | 0x81 | IN | 0 | `NUM_OUTPUT_CHANNELS` x uint16 LE (18 B RP2350, 10 B RP2040) |

The index travels in the low byte of wValue on both SET and GET, matching the existing indexed pin commands. An out-of-range index makes the SET a no-op and the GET STALL. A SET shorter than 4 bytes is a no-op.

### Apply semantics

SET handlers clamp, write `tube_config`, and raise `tube_update_pending`. The main loop recomputes coefficients into the inactive buffer and publishes the pointer; the audio path only ever reads the published snapshot. `output_mask` is read live and does not raise the flag. Sample-rate changes raise the flag like every other module.

### Change notifications

Every SET calls `notify_param_write` at each field it writes, at that field's `WireBulkParams` offset; the notify layer compares against its shadow and only queues a NOTIFY_EVT_PARAM_CHANGED for a real byte change. A tube-type SET writes five fields (four knobs plus the type byte); a knob SET that resets the type to Custom writes two. Only the Custom reset itself is gated on the knob value actually changing.

---

## 4. Bulk Wire Format (GET/SET_ALL_PARAMS 0xA0/0xA1)

Wire format version **31** appends `WireTubeParams` (48 bytes) after the subharm section at offset 5980, for a total of 6028 bytes.

```c
typedef struct __attribute__((packed)) {
    uint8_t  enabled;        // 0 / 1
    uint8_t  tube_type;      // 0 = custom, 1..16 per section 2.3
    uint8_t  rectifier;      // 0..3 per section 2.9
    uint8_t  xfmr_enabled;   // 0 / 1
    uint16_t output_mask;    // bit k = output k
    uint8_t  reserved[2];    // zero
    float    drive_db;
    float    bias_pct;
    float    asym_db;
    float    hardness_pct;
    float    sag_pct;
    float    xfmr_lf_hz;
    float    xfmr_sat_pct;
    float    xfmr_hf_hz;
    float    mix_pct;
    float    trim_db;
} WireTubeParams;             // 48 bytes
```

Bulk apply copies fields verbatim, clamps the enums, and raises the recompute flag. It never runs the tube-type row lookup.

---

## 5. Persistence

Preset slot data version **38** appends the same fifteen values to `PresetSlot` (48 bytes, laid out as the wire struct). Loading a slot written before V38 applies the defaults from section 2 with `enabled = 0`. Factory reset applies the same defaults. The recompute flag is raised on both branches so a load that turns the effect off unpublishes its coefficients.

---

## 6. App Integration Patterns

### Startup / reconnect sync

Read the bulk parameter block (V31 or later) and populate the UI from the tube section. There is no need to issue fifteen GETs.

### Live control

Send `REQ_SET_TUBE_PARAM` per knob change. The firmware clamps and notifies, so a second host or a front panel stays in sync from the notification stream.

### Typical UI

- Enable toggle, tube-type picker, drive knob, mix knob, output-mask checkboxes.
- "Character" group (bias, asymmetry, hardness, sag, rectifier) shown as the values the selected type loaded. Editing any of the first four flips the picker to Custom; the firmware does this itself, the UI just follows the notification.
- Transformer group with its own enable.
- Per-output saturation meters from `REQ_GET_TUBE_METER` at 10 to 20 Hz.

### Suggested starting points

- Warm hi-fi: 12AU7 or 6SN7, drive 3 to 6 dB, mix 100, transformer off.
- Single-ended sweetness: 300B, drive 6 dB, transformer on at 80 Hz, 30 %.
- Guitar-amp style: 12AX7, drive 12 to 18 dB, rectifier 5U4, transformer on with `xfmr_hf_hz` around 6 kHz.
- Push-pull power styles (EL84, EL34, 6L6, 6V6, KT88) are meant to be used with the transformer on.

### Feature detection

Firmware wire format version >= 31, or a non-STALL response to `REQ_GET_TUBE_PARAM` index 0.

### Control Surfaces

Caps version 19 adds four nouns so panels and IR remotes can drive the effect: `CS_NOUN_TUBE` (bool), `CS_NOUN_TUBE_DRIVE` (continuous dB 0..24), `CS_NOUN_TUBE_TYPE` (enum, 17 values), `CS_NOUN_TUBE_MIX` (continuous percent 0..100). Each maps to the indexed SET with the parameter index in wValue.

---

## 7. Interactions and Edge Cases

- **Slot alignment.** Nothing in the module delays a sample. Masking the effect per output changes the phase response of that output only through one-pole IIR stages, the same category as a PEQ band, and never its sample alignment.
- **Headroom.** The shaper output is bounded by `max(s_p, s_n)`, at most 2.65 (+8.5 dBFS) with the negative knee 12 dB later, and the DC blocker can double that on a transient. So a heavily driven, strongly asymmetric setting can push the wet path above 0 dBFS even at 0 dB trim; the host should watch the output clip flags and use `trim_db` to bring it back. Symmetric settings stay under +1 dBFS. The transformer saturator is bounded by its knee. Small-signal gain is unity at every hardness.
- **Q28 ceilings (RP2040).** `fast_mul_q28` splits each operand into 16-bit halves and sums the two cross products in a 32-bit integer, so its real constraint is that the two operand magnitudes sum to below 8.0, not that their product does. The kernel is budgeted on that rule. Drive up to 15.85 is carried in Q24 along with the bias and sag terms so the drive product lands in a "/16" domain; the shaper input is clamped to +/-4.0 and the driven value to +/-4 knee units before the negative-knee ratio multiply (see 2.4 for the one lossy corner). The DC-blocker output is bounded by twice the shaper peak (5.3), so a copy of it is clamped to +/-3.4 (`TUBE_Q28_Y_LIM`) while the filter state keeps the true value; the transformer output is clamped to +/-3.4 (`TUBE_Q28_Y2_LIM`) ahead of its HF one-pole so each one-pole difference stays under 6.8. The transformer low band is clamped to +/-1.0 before its knee multiply, with the reciprocal knee (up to 7.94) carried in Q26 and shifted back. The wet signal is clamped to `wet_lim = clamp((7.5 - 4 dry_w) / wet_w, 0, 3.4)` and the dry term uses the +/-4.0-clamped input, so the final sum never exceeds 7.5. None of these clamps can bite while the wet signal is within +10 dBFS pre-trim. A host model that emulates the multiply helper exactly reports zero integer overflows over 404 parameter combinations, both transformer states, with +6 dBFS and realistic stimuli, and a worst in-range difference from the float kernel of about -73 dBFS, which is the helper's own truncation floor.
- **Bypass cost.** With `enabled = 0` the published pointer is NULL and the per-output loop skips the call. With the transformer off the transformer stages are skipped inside the kernel by a hoisted flag, not evaluated with pass-through coefficients.
- **Psybass and subharm ordering.** Both run before tube so the stage saturates the enhanced bass rather than the other way round.
- **RAW signal generator outputs** bypass the effect and reset its state, as for psybass.
- **Aliasing.** The shaper runs at the native rate with no oversampling. Harmonic content is bounded to fifth order below the clamp, so aliasing is audible only on bright material at heavy drive at 44.1/48 kHz and is negligible at 96 kHz.

---

## 8. Implementation Summary (firmware reference)

- **Files:** `tube.h`, `tube.c` (config, coefficient computation, tube and rectifier tables, per-output kernel, meter accessor).
- **Kernel:** one `DSP_TIME_CRITICAL` non-inline function `tube_process_output_block()` called from the six per-output loops (four in `audio_pipeline.c`, two in `pdm_generator.c`), so its RAM text is paid once. `tube.c` compiles with `-O3 -ffp-contract=off` like `subharm.c`.
- **Snapshot:** `Core1EqWork` gains `tube_coeffs` and `tube_mask` so both cores apply one view per packet.
- **State:** `TubeOutputState` per output: sag envelope, DC-blocker input and output, transformer low-band and high-frequency one-pole states, meter. 24 bytes per output (216 B RP2350, 120 B RP2040).
- **Coefficients:** 23 values including the RP2040-only `wet_lim`, double-buffered.
- **Measured footprint (2026-09-18 clean builds):** RP2350 kernel 858 B of RAM text, `.data` 91,496 of the 92,160 B budget, BSS +552 B, free RAM 77,320 B. RP2040 kernel 1,220 B of RAM text, `.data` 64,464 of 65,536, BSS +452 B, free RAM 46,748 B. Neither placement budget needed raising.
- **Cost estimate:** about 45 FP ops per sample per output on RP2350 (0.45 % CPU per output at 48 kHz, 307.2 MHz). On RP2040, 11 `fast_mul_q28` per sample base, plus one with sag on, one on the negative half, and six with the transformer on (13 to 19 typical). Both to be confirmed on the CPU meter; hardware-untested as of this writing.
- **Versions:** wire V31, slot V38, Control Surfaces caps v19.
- **Vendor commands:** 0x3E, 0x3F, 0x81.
