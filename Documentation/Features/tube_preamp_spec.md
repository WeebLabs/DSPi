# Tube Preamp Emulation Specification

## 1. Overview

The tube preamp ("tube") adds valve-style harmonic colour, compression and, optionally, the output-stage response of a tube amplifier driving a loudspeaker to any selected output channel. It is a per-output effect module in the same family as Psychoacoustic Bass and the Subharmonic Synthesizer: one global configuration, a 16-bit output mask, shared double-buffered coefficients, per-output filter state, zero added latency.

The emulation is a characterful approximation, not a circuit simulation. A single static waveshaper with an adjustable operating point (bias), separate positive and negative knees (asymmetry), a knee-hardness blend, and a slow supply-sag envelope reproduces the audible differences between the popular preamp and power tubes. A tube-type selector loads a row of those four character parameters; the row is a preset, never a hidden term in the audio path, so the kernel cost is identical for every type.

Both platforms are supported. RP2350 runs the kernel in float; RP2040 runs the same kernel in Q28 fixed point through `fast_mul_q28`.

### Key characteristics

- **Per-output processing with one global parameter set.** Every output selected by `output_mask` runs the same configuration. Different settings per speaker are handled by masking and presets, as for loudness and psybass.
- **Subtle by default, level-neutral.** The shaper carries `1 / drive` makeup gain so small-signal level never changes, and the default knee sits 12 dB above full scale. Enabling the module at default adds about 0.23 % second-harmonic-led distortion at -12 dBFS and about 0.1 dB of gain change. Drive is the control that brings the character up, and its -30 dB floor is close to transparent.
- **Single shaper shape, sixteen tube styles.** Triodes, pentodes, push-pull and single-ended power stages differ in this model only by bias, knee asymmetry, knee hardness and sag depth. Selecting a tube type copies its row into those four parameters and the user's drive, mix and transformer settings are left alone.
- **Level-dependent even harmonics.** The bias term shifts the operating point on the curve, so second-harmonic content rises with signal level exactly as it does in a real single-ended stage. Knee asymmetry adds further even-order content at heavy drive.
- **Supply sag.** A one-pole envelope of how far into the knee the stage is being driven pulls the drive down slowly. A rectifier selector presets the sag depth scale and attack and release times.
- **Output stage.** A tube amplifier's high source impedance lets the speaker's own impedance curve shape the response: a broad bump at the woofer resonance and a small lift where voice-coil inductance raises impedance at the top. The stage models that with one damping-factor control and a resonance frequency, as a low-Q bell plus a first-order top shelf. Nothing in it is nonlinear.
- **Zero added latency.** Every stage is memoryless or a one-pole IIR. The dry path is never delayed. Inter-output-slot sample alignment is untouched whether the effect is on, off, or masked per channel.

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
        y = DC_block(v)                    one-pole HP at 2.5 Hz|
        [output stage, optional]                                |
          y = bell(y, xfmr_res_hz, +G_bump)   Q 0.707           |
          y = shelf(y, 2.5 kHz, +G_top)       first order       |
   out = (1 - mix) * x + mix * trim * y  <-----------------------+

   env   = one-pole follower of |t| (attack / release from rectifier row)
```

- `m = 10^(drive_db/20)`. The positive knee is fixed at t = 1, so drive alone sets how hard the stage is driven: a full-scale input reaches the knee at 0 dB drive, at the -12 dB default the knee sits 12 dB above full scale, and at the -30 dB floor it sits 30 dB above.
- `b = bias_pct / 200` (range -0.5 .. +0.5 in knee units).
- `ratio_n = 10^(-asym_db/20)`: a positive `asym_db` makes the negative half clip later (softer cut-off side, harder grid-conduction side).
- `p(t) = t * (c1 + t^2 * (c3 + t^2 * c5))` with `h = hardness_pct / 100`, `c1 = 1.5 + 0.375 h`, `c3 = -0.5 - 0.75 h`, `c5 = 0.375 h`. At h = 0 this is the cubic `1.5 t - 0.5 t^3`; at h = 1 it is the quintic `(15 t - 10 t^3 + 3 t^5) / 8`. Both reach exactly 1 at t = 1 with zero slope, so the clamp is continuous in value and slope for every hardness.
- `s_p = 1 / (c1 m)` and `s_n = 10^(asym_db/20) / (c1 m)` normalise the small-signal gain to unity, so neither hardness nor drive changes the level of clean material. Drive moves the knee, not the loudness; `trim_db` is the only level control.
- `v0 = p(b) * s` is the shaper's output for silence, subtracted so enabling the effect produces no step. The DC blocker removes the level-dependent offset asymmetric clipping creates.
- Sag: `depth = clamp(sag_pct / 100 * rect_scale, 0, 0.9)`; `env += (|t| - env) * (|t| > env ? a_att : a_rel)`. Solid-state rectifier disables sag entirely.
- Output stage: with damping factor `df` the source impedance is `Zn / df`. Against a speaker whose impedance rises to 4 x nominal at resonance and 2 x nominal at the top, the terminal voltage lifts by `G_bump = 4 (df + 1) / (4 df + 1)` at the bell and `G_top = 2 (df + 1) / (2 df + 1)` on the shelf. The bell is a Cytomic TPT SVF peaking section with `A = sqrt(G_bump)`, `k = 1 / (0.707 A)`, mix `k (A^2 - 1)`; the shelf is `y += (G_top - 1) * HP1(y)` with a one-pole at 2.5 kHz.
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

Every parameter is addressed by a small integer index (0 to 13) through one indexed SET/GET pair (section 3). On the wire every parameter is a little-endian IEEE 754 float32, including the boolean, mask and enumerated ones. Booleans are non-zero-is-true. The mask and the enums are rounded to the nearest integer and then clamped. Floats are clamped to their range. A NaN is ignored and the SET reports success. A GET after a SET returns the stored value.

| Index | Name | Type | Range | Default | Recompute |
|---|---|---|---|---|---|
| 0 | `enabled` | bool | 0 / 1 | 0 | yes |
| 1 | `output_mask` | uint16 | 0x0000 .. 0xFFFF | 0xFFFF | no (read live) |
| 2 | `tube_type` | enum | 0 .. 16 | 1 (12AX7) | yes (loads a row) |
| 3 | `drive_db` | float | -30 .. 24 dB | -12 | yes |
| 4 | `bias_pct` | float | -100 .. +100 % | 10 | yes |
| 5 | `asym_db` | float | -12 .. +12 dB | 3 | yes |
| 6 | `hardness_pct` | float | 0 .. 100 % | 40 | yes |
| 7 | `sag_pct` | float | 0 .. 100 % | 15 | yes |
| 8 | `rectifier` | enum | 0 .. 3 | 1 (GZ34) | yes |
| 9 | `xfmr_enabled` | bool | 0 / 1 | 0 | yes |
| 10 | `xfmr_damping` | float | 1 .. 20 | 2 | yes |
| 11 | `xfmr_res_hz` | float | 30 .. 150 Hz | 85 | yes |
| 12 | `mix_pct` | float | 0 .. 100 % | 100 | yes |
| 13 | `trim_db` | float | -12 .. +12 dB | 0 | yes |

Defaults for indices 4 to 7 are the 12AX7 row. The defaults are chosen to be subtle rather than an obvious effect. With the knee 12 dB above full scale and a 10 % bias, enabling the module at default is close to level-neutral and adds a small, second-harmonic-led colour that grows with level. Host-model figures for a 100 Hz sine on the default settings (the same row at the old -6 dB default measured 0.50 % THD at -12 dBFS and 3.6 % at 0 dBFS, and at the -30 dB floor 0.029 % and 0.12 %):

| Input level | H2 | H3 | THD |
|---|---|---|---|
| -20 dBFS | -61 dBc | -82 dBc | 0.092 % |
| -12 dBFS | -53 dBc | -66 dBc | 0.23 % |
| -6 dBFS | -46 dBc | -55 dBc | 0.50 % |
| 0 dBFS | -39 dBc | -44 dBc | 1.25 % |

Below the knee the second harmonic scales as about `0.5 x bias x drive x level` and the third as `(drive x level)^2 / 12`, both relative to the fundamental, so drive raises both and bias raises only the even-order part. Hardness and asymmetry also act below the knee. Hardness raises the cubic term from 1/3 to 2/3 of the linear term, and asymmetry scales the negative half's cubic term by `10^(-asym_db/10)`, which adds even-order content. Both scale with `(drive x level)^2`, so at low drive they fade out with the rest of the distortion.

### 2.1 enabled

Master switch. When off, `current_tube_coeffs` is NULL and every output's state is reset; the audio path costs nothing beyond the per-packet pointer check.

### 2.2 output_mask

Bit k selects output channel k (RP2350: 0..7 S/PDIF, 8 PDM; RP2040: 0..3 S/PDIF, 4 PDM). Bits above the platform's output count are ignored. Read live each packet, no recompute. A masked-off, muted, disabled or RAW-signal-generator output has its state reset so it re-enters cleanly.

### 2.3 tube_type

| Value | Tube | Style | bias_pct | asym_db | hardness_pct | sag_pct |
|---|---|---|---|---|---|---|
| 0 | Custom | Knobs as set, no row applied | | | | |

Rows are scaled for a clean default. Asymmetry and hardness carry each tube's identity and only act at the knee, so turning drive up brings the character out; bias and sag are kept low so the default is transparent.
| 1 | 12AX7 / ECC83 | High-gain preamp triode; the default | 10 | 3 | 40 | 15 |
| 2 | 5751 | Cooler 12AX7 | 8 | 3 | 35 | 12 |
| 3 | 12AT7 / ECC81 | Medium-gain driver, more odd-order | 5 | 2 | 55 | 10 |
| 4 | 12AY7 | Tweed front end, gentle | 7 | 4 | 25 | 15 |
| 5 | 12AU7 / ECC82 | Clean line stage | 5 | 5 | 20 | 8 |
| 6 | 6SN7 | Octal hi-fi line stage, sweet | 7 | 6 | 15 | 10 |
| 7 | 6SL7 | Octal high-mu, rounder knee | 10 | 3 | 30 | 15 |
| 8 | 6DJ8 / ECC88 / 6922 | Clean, hard when pushed | 3 | 2 | 60 | 5 |
| 9 | EF86 / 6267 | Pentode preamp, symmetric bite | 2 | 0 | 75 | 12 |
| 10 | 6SJ7 | Octal pentode, softer than EF86 | 3 | 1 | 65 | 15 |
| 11 | EL84 / 6BQ5 | Push-pull power, chimey | 0 | 0 | 50 | 25 |
| 12 | EL34 | Push-pull power, mid crunch, deep sag | 0 | 0 | 60 | 30 |
| 13 | 6L6 / 5881 | Push-pull power, tight | 0 | 0 | 55 | 18 |
| 14 | 6V6 | Push-pull power, early breakup, heavy sag | 0 | 0 | 35 | 30 |
| 15 | KT88 / 6550 | Push-pull hi-fi power, near linear | 0 | 0 | 45 | 10 |
| 16 | 300B / 2A3 | Single-ended DHT, pure even harmonics | 12 | 6 | 10 | 12 |

**Apply semantics.** Setting `tube_type` to 1..16 copies that row into `bias_pct`, `asym_db`, `hardness_pct` and `sag_pct`, stores the type, and raises the recompute flag. Each changed field emits its own change notification. Setting it to 0 stores 0, leaves the four knobs untouched, and still raises the recompute flag (a harmless no-op recompute). Setting any of indices 4..7 to a value different from the current one resets `tube_type` to 0 (Custom) and notifies that byte; a SET that lands on the value already stored leaves the type alone. Bulk apply and preset load restore the stored fields verbatim with no row lookup, so a preset saved as "12AX7" reloads as 12AX7 even if the row table changes in a later firmware.

Push-pull power-tube styles have zero bias and asymmetry because a push-pull stage cancels even harmonics by construction; their character comes from hardness and sag, and from the output stage, which the host should suggest enabling for them. The output-stage settings are not part of the row: damping and resonance describe the amplifier and speaker, not the tube, so the user owns them.

### 2.4 drive_db

Sets how far into the knee the signal is driven. Small-signal gain is unity at every drive because the shaper output is scaled by `1 / m`, so drive changes distortion, not loudness, and the clip ceiling of the wet path falls as drive rises (`1 / (c1 m)` on the positive half). At 0 dB a full-scale input just reaches the knee; at the -12 dB default the knee is 12 dB above full scale; at the -30 dB floor it is 30 dB above; at +24 dB a -24 dBFS input reaches it. Third-order distortion falls about 12 dB and second-order about 6 dB for every 6 dB of drive removed. The -30 dB floor is set by the RP2040 fixed-point budget. The makeup scales `s_p + s_n` reach 105 there with +12 dB asymmetry, which is the most the shaper-output domain at `s_shift` 4 can hold (section 7). The kernel clamps the shaper input to +/-4.0 (+12 dBFS) first so the drive product cannot wrap; the dry path is never clamped. The clamp is lossless except for inputs beyond +12 dBFS at low drive with maximum bias and asymmetry, where the negative half saturates slightly early.

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

Enables the output stage. When off, the bell and shelf are skipped entirely at block level and their state is zero.

### 2.11 xfmr_damping

Damping factor of the modelled amplifier, the speaker's nominal impedance divided by the amplifier's source impedance. It sets both the bell boost and the top lift:

| Damping factor | Bell at resonance | Top lift |
|---|---|---|
| 1 | +4.1 dB | +2.5 dB |
| 2 (default) | +2.5 dB | +1.6 dB |
| 4 | +1.4 dB | +0.8 dB |
| 10 | +0.6 dB | +0.3 dB |
| 20 | +0.3 dB | +0.2 dB |

A single-ended triode amplifier without feedback sits around 2 to 3; a push-pull pentode amplifier with feedback around 8 to 15. The default of 2 gives the recognisable "big bottom" as soon as the stage is enabled; 20 is close to flat.

### 2.12 xfmr_res_hz

The loudspeaker's resonance in its enclosure, which is where the bell sits. Q is fixed at 0.707, so the bump is broad. 85 Hz suits a typical small to medium woofer; larger drivers sit lower.

### 2.13 mix_pct (index 12)

Dry/wet blend. The dry path is the untouched input, sample-aligned with the wet path.

### 2.14 trim_db (index 13)

Output level applied to the wet path only.

---

## 3. Vendor Command Transport

### USB (primary transport)

Vendor control requests on the DSPi vendor interface, as for every other module. UART and I2C reach the same handlers through the shared dispatcher with no per-command table to update.

### Command summary

| Command | Code | Dir | wValue | Payload / Response |
|---|---|---|---|---|
| `REQ_SET_TUBE_PARAM` | 0x3E | OUT | parameter index (0..13) | 4-byte float32 LE |
| `REQ_GET_TUBE_PARAM` | 0x3F | IN | parameter index (0..13) | 4-byte float32 LE |

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
    float    xfmr_damping;
    float    xfmr_res_hz;
    float    mix_pct;
    float    trim_db;
    float    reserved_f;     // zero
} WireTubeParams;             // 48 bytes; reserved_f keeps the V31 size after the output-stage rework
```

Bulk apply copies fields verbatim, clamps the enums, and raises the recompute flag. It never runs the tube-type row lookup.

---

## 5. Persistence

Preset slot data version **38** appends the same fourteen values plus the reserved float to `PresetSlot` (48 bytes, laid out as the wire struct). Loading a slot written before V38 applies the defaults from section 2 with `enabled = 0`. Factory reset applies the same defaults. The recompute flag is raised on both branches so a load that turns the effect off unpublishes its coefficients.

---

## 6. App Integration Patterns

### Startup / reconnect sync

Read the bulk parameter block (V31 or later) and populate the UI from the tube section. There is no need to issue fourteen GETs.

### Live control

Send `REQ_SET_TUBE_PARAM` per knob change. The firmware clamps and notifies, so a second host or a front panel stays in sync from the notification stream.

### Typical UI

- Enable toggle, tube-type picker, drive knob, mix knob, output-mask checkboxes.
- "Character" group (bias, asymmetry, hardness, sag, rectifier) shown as the values the selected type loaded. Editing any of the first four flips the picker to Custom; the firmware does this itself, the UI just follows the notification.
- Output-stage group with its own enable, a damping-factor slider and a resonance frequency. The existing output meters cover level monitoring; the module has no meter of its own.

### Suggested starting points

- Default: 12AX7, drive -12 dB, mix 100, output stage off. About 0.1 dB of level change, 0.23 % THD at -12 dBFS and 1.25 % at 0 dBFS.
- Near transparent: any row, drive -30 dB. The 12AX7 row gives 0.03 % THD at -12 dBFS.
- Warm hi-fi: 12AU7 or 6SN7, drive -3 to 0 dB, mix 100, output stage off or damping 10 and above.
- Single-ended sweetness: 300B, drive 0 to 3 dB, output stage on, damping 2, resonance matched to the speaker.
- Guitar-amp style: 12AX7, drive 12 to 18 dB, rectifier 5U4, output stage on with damping 1 to 3 and resonance around 100 Hz.
- Push-pull power styles (EL84, EL34, 6L6, 6V6, KT88) are meant to be used with the output stage on, damping 4 to 10.

### Feature detection

Firmware wire format version >= 31, or a non-STALL response to `REQ_GET_TUBE_PARAM` index 0.

### Control Surfaces

Caps version 19 adds four nouns so panels and IR remotes can drive the effect: `CS_NOUN_TUBE` (bool), `CS_NOUN_TUBE_DRIVE` (continuous dB -30..24, taken from `TUBE_DRIVE_MIN` and `TUBE_DRIVE_MAX`), `CS_NOUN_TUBE_TYPE` (enum, 17 values), `CS_NOUN_TUBE_MIX` (continuous percent 0..100). Each maps to the indexed SET with the parameter index in wValue.

---

## 7. Interactions and Edge Cases

- **Slot alignment.** Nothing in the module delays a sample. Masking the effect per output changes the phase response of that output only through one-pole IIR stages, the same category as a PEQ band, and never its sample alignment.
- **Headroom.** With makeup gain the positive-half ceiling is `1 / (c1 m)`: 0.67 at 0 dB drive, falling 1 dB per dB of drive above that, and rising 1 dB per dB below it to 21 (+26 dBFS) at -30 dB. The negative half's ceiling is `kn` times higher, and the DC blocker can double a transient. Below about -3.5 dB drive the ceiling sits above full scale, so the wet path simply follows an in-range input. A strongly asymmetric setting driven hard can push the wet path above 0 dBFS at 0 dB trim, so the host should watch the output clip flags and use `trim_db`. With symmetric settings and a full-scale input, the wet path peaks at the lower of the input level and `1 / (c1 m)`. The output stage adds at most +4.1 dB. Small-signal gain is unity at every hardness and drive.
- **Q28 ceilings (RP2040).** `fast_mul_q28` splits each operand into 16-bit halves and sums the two cross products in a 32-bit integer, so its real constraint is that the two operand magnitudes sum to below 8.0, not that their product does. The kernel is budgeted on that rule. Two coefficient-set shifts keep every product inside that rule across the whole drive range. `t_shift` (1..4) is the smallest shift with `m < 1.75 x 2^t_shift`. The drive, bias and sag terms are stored in Q(28 - `t_shift`), so the drive product `4 m + |b|` stays under 7.5. It never goes below 1, because the driven value is pre-clamped to +/-4 knee units before the negative-knee ratio multiply and 4 x 3.98 would overflow at shift 0. `s_shift` (0..4) is the smallest shift with `s_p + s_n < 7.5 x 2^s_shift`. The makeup scales and `v0` are stored in Q(28 - `s_shift`), so the shaper product and its `v0` subtraction stay under 7.5 even though `s_n` reaches 84 at -30 dB drive. From -6 dB drive up, `s_shift` is 0 as in the original kernel. `t_shift` is 1 below about +10.9 dB drive and reaches the original 4 only from about +22.9 dB, which gives the driven value more resolution than the fixed Q24 domain did. The shaper input is clamped to +/-4.0 (see 2.4 for the one lossy corner). The shaper output `v` is clamped to +/-3.4 (`TUBE_Q28_Y_LIM`) in its own domain and then shifted back to Q28 before the DC blocker, because the blocker's output is bounded by twice its input, which must stay under 8.0; a copy of the blocker output is clamped to the same limit while the filter state keeps the true value; the bell input is clamped to +/-2.5 (`TUBE_Q28_BELL_IN`) so the SVF difference term stays under 6.5, and the bell output to +/-3.4 (`TUBE_Q28_Y2_LIM`) ahead of the shelf so the shelf's one-pole difference stays under 6.8. Every bell and shelf coefficient is below 1.0. The wet signal is clamped to `wet_lim = clamp((7.5 - 4 dry_w) / wet_w, 0, 3.4)` and the dry term uses the +/-4.0-clamped input, so the final sum never exceeds 7.5. None of these clamps can bite while the wet signal is within +8 dBFS pre-trim. A host model that emulates the multiply helper exactly reports zero integer overflows over 23,760 parameter combinations (drive -30 to +24 dB, every bias, asymmetry, hardness, sag, mix and trim extreme), both output-stage states, with +6 dBFS sine, +12 dBFS square and in-range stimuli (2026-09-28). The worst in-range difference from the float kernel is about -72 dBFS at extreme corners, with a median of about -91 dBFS. That is the helper's own truncation floor, and the kernel before the drive-floor change measured -73.6 dBFS on the same sweep.
- **Bypass cost.** With `enabled = 0` the published pointer is NULL and the per-output loop skips the call. With the output stage off its arm is compiled out of the loop body the kernel runs, not evaluated with pass-through coefficients.
- **Psybass and subharm ordering.** Both run before tube so the stage saturates the enhanced bass rather than the other way round.
- **RAW signal generator outputs** bypass the effect and reset its state, as for psybass.
- **Aliasing.** The shaper runs at the native rate with no oversampling. Harmonic content is bounded to fifth order below the clamp, so aliasing is audible only on bright material at heavy drive at 44.1/48 kHz and is negligible at 96 kHz.

---

## 8. Implementation Summary (firmware reference)

- **Files:** `tube.h`, `tube.c` (config, coefficient computation, tube and rectifier tables, per-output kernel).
- **Kernel:** one `DSP_TIME_CRITICAL` non-inline function `tube_process_output_block()` called from the six per-output loops (four in `audio_pipeline.c`, two in `pdm_generator.c`), so its RAM text is paid once. `tube.c` compiles with `-O3 -ffp-contract=off` like `subharm.c`; a hardware A/B on 2026-09-19 found no measurable difference either way for this file.
- **Branch-free float body (RP2350).** The first build wrote every clamp and sign-dependent select as `if` or `?:`. On the M33 each float comparison is a VCMP followed by a VMRS that moves FPU flags into the core and stalls the pipeline, and every taken branch flushes because the core has no branch predictor. That build had 16 compare pairs and 26 branches per loop and metered about 2 % CPU per output at 48 kHz, roughly four times the arithmetic-only estimate. The kernel now writes each select as an `fmaxf`/`fminf` split (`t = fmaxf(t,0) + fminf(t,0) * ratio_n`, and likewise for the half scale, the sag attack/release and the meter), which compiles to VMAXNM/VMINNM with no flag transfer and is bit-exact with the branchy form. The output-stage arm is a literal parameter to an always-inline body so the compiler emits two loops. Result: zero compares, 5 branches, 660 B of RAM text.
- **Snapshot:** `Core1EqWork` gains `tube_coeffs` and `tube_mask` so both cores apply one view per packet.
- **State:** `TubeOutputState` per output: sag envelope, DC-blocker input and output, two bell integrators, shelf one-pole state. 24 bytes per output (216 B RP2350, 120 B RP2040).
- **Coefficients:** 22 values including the RP2040-only `wet_lim`, double-buffered.
- **Measured footprint (2026-09-20 builds):** RP2350 kernel 660 B of RAM text, `.data` 91,288 of the 92,160 B budget, BSS +560 B, free RAM 77,520 B. RP2040 kernel 1,212 B of RAM text (1,196 B after the 2026-09-28 drive-floor change), `.data` 64,448 of 65,536, BSS +460 B, free RAM 46,756 B. Neither placement budget needed raising.
- **Cost:** the branchy first build metered about 2 % CPU per output at 48 kHz on RP2350 (307.2 MHz); the branch-free build meters just over 1 % per output under the same conditions (2026-09-19). Arithmetic alone is about 30 FP ops per sample per output with the output stage off and about 50 with it on (the bell is 13 ops, the shelf 6). On RP2040, 10 `fast_mul_q28` per sample base, plus one with sag on, one on the negative half, and seven with the output stage on (12 to 19 typical), unmeasured.
- **Versions:** wire V31, slot V38, Control Surfaces caps v19.
- **Vendor commands:** 0x3E, 0x3F.
