# Psychoacoustic Bass Enhancement Specification

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7)*

## 1. Overview

Psychoacoustic Bass ("psybass") makes small, excursion-limited speakers convey bass that they cannot physically reproduce. It exploits the missing fundamental effect: when the ear hears a harmonic series (2f, 3f, 4f, ...), the brain perceives the pitch of the fundamental f even when f itself is absent. The feature extracts the low band below a configurable cutoff (the speaker's low-frequency limit), synthesizes harmonics from it with a nonlinear device (NLD), band-limits those harmonics into the speaker's reproducible range, and mixes them back into the signal. Optionally it also attenuates or removes the original un-reproducible fundamental, which reduces driver excursion and distortion at no perceptual cost.

Both platforms are supported: RP2350 uses a float TPT state-variable-filter implementation, RP2040 uses Q28 fixed-point RBJ biquads.

### Key characteristics

- **Per-output processing with one global parameter set.** A single configuration (cutoff, harmonic level, drive, character, original bass level) applies to every output channel selected by a 16-bit output mask, exactly like loudness compensation. Different speakers on different outputs that need different settings should be handled by masking and preset switching.
- **Consecutive harmonic series.** The NLD blends a full-wave rectifier (even harmonics: 2f, 4f, ...) with a driven cubic soft clipper (odd harmonics: 3f, 5f, ...). The blend produces the consecutive 2f/3f/4f series that pitches the missing fundamental correctly; either family alone tends to sound octave-shifted or hollow.
- **Level-tracking dynamics.** The rectifier path is exactly level-proportional, so harmonic level follows program bass level and the effect breathes with the music instead of pumping a constant drone.
- **Zero added latency.** The processing is pure IIR and in place; the dry signal path is never delayed. Inter-output-slot sample alignment (the firmware's hard invariant) is untouched whether the effect is on, off, or masked per channel.
- **Free original-bass control.** The low band is already split out for harmonic generation, so attenuating the original fundamental costs nothing extra.

### Signal flow (per selected output channel)

```
              +--> LP2 @ cutoff --+--> x (g_orig - 1) ------------------+
              |     (low band)    |                                     |
   in --------+                   +--> NLD --> HP2 @ cutoff             v
              |                        --> LP1 @ 4x cutoff --> x g_harm +--> out
              |                                                         ^
              +---------------------------------------------------------+
                                     (dry path, untouched)

   NLD:  even = |low|                                (full-wave rectifier)
         odd  = softclip(drive * low)                (cubic: 1.5d - 0.5d^3)
         h    = (1 - t) * even + t * odd             (t = character / 100)
```

- LP2 / HP2 are 2nd-order Butterworth (Q = 0.7071) at the cutoff frequency.
- The highpass removes the DC produced by rectification and any residual fundamental; the one-pole lowpass at 4x cutoff rolls the harmonic series off gently (6 dB/oct) so it mimics natural harmonic decay instead of sounding buzzy.
- `g_orig = 10^(original_db/20)`, `g_harm = 10^(harmonics_db/20)`.

### Signal chain position

Psybass runs **per output channel, post-matrix, pre-crossover, pre-output-EQ** (PASS 5-7 entry):

```
PASS 4:   Matrix Mixing (fan-out to output channels)
PASS 4.5: Crossfeed (per output pair)
             |
          Psychoacoustic Bass   <-- HERE (per output, masked)
             |
          Crossover -> Per-Output PEQ -> Gain/Volume -> Loudness -> Delay
             |
          Output Encoding (S/PDIF, I2S, ADAT, PDM)
```

Pre-crossover placement is deliberate: the effect must see the low band before a high-pass crossover removes it. The classic small-speaker protection recipe is a high-pass crossover at the same frequency as the psybass cutoff; psybass synthesizes the harmonics from the bass, then the crossover removes the fundamental the speaker cannot play. In that configuration leave `original_db` at 0 (the crossover already does the removal), or skip the crossover and use `original_db` instead.

Because psybass runs pre-gain, its character does not change with volume; and because it runs pre-output-EQ, per-output PEQ shapes the generated harmonics along with everything else.

---

## 2. Parameters

All floats on the wire are little-endian IEEE 754 single-precision. All finite SET values are clamped by the firmware to the documented range; a GET after a SET returns the clamped value. A NaN or infinite float is refused with `VC_ERR_VALUE`.

### 2.1 enabled

| Property | Value |
|----------|-------|
| **Type** | `bool` (uint8_t on wire) |
| **Range** | 0 (off) or 1 (on) |
| **Default** | 0 (disabled) |
| **Command** | `FX.PSYBASS_ENABLE` (page 0x08, command 0x40), GET and SET |
| **Payload** | 1 byte: `0x00` = disabled, `0x01` = enabled |

Master enable. When disabled the coefficient pointer is unpublished and the per-output processing is skipped entirely (zero per-sample CPU cost); per-output filter states are cleared so re-enabling starts transient-free.

### 2.2 cutoff_hz

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | 30.0 to 300.0 (Hz) |
| **Default** | 80.0 |
| **Command** | `FX.PSYBASS_CUTOFF` (page 0x08, command 0x41), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

The speaker's low-frequency limit. Content below this frequency feeds the harmonic generator; generated harmonics occupy roughly cutoff to 4x cutoff. Set it to the frequency below which the target speaker's output collapses (typical values: 50-60 Hz for bookshelf speakers, 80-120 Hz for small Bluetooth speakers, 150-250 Hz for laptop/phone speakers).

### 2.3 harmonics_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | -24.0 to +12.0 (dB) |
| **Default** | 0.0 |
| **Command** | `FX.PSYBASS_HARMONICS` (page 0x08, command 0x42), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Level of the generated harmonics mixed into the output. This is the primary "amount of effect" control. 0 dB injects the shaped harmonic band at unity relative to the NLD output; negative values make the effect subtle, positive values emphasize it. The +12 dB ceiling protects fixed-point headroom on RP2040 and applies to both platforms for parity.

### 2.4 drive_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | 0.0 to 18.0 (dB) |
| **Default** | 6.0 |
| **Command** | `FX.PSYBASS_DRIVE` (page 0x08, command 0x43), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Pre-gain into the odd-harmonic soft clipper. Higher drive pushes quieter bass into the clipper's nonlinear region, producing a denser, more saturated odd-harmonic series and making the effect audible on quieter passages. At 0 dB only near-full-scale bass generates odd harmonics. Drive does not affect the even (rectifier) path, so at low `character` values this control does little.

### 2.5 character_pct

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | 0.0 to 100.0 (%) |
| **Default** | 50.0 |
| **Command** | `FX.PSYBASS_CHARACTER` (page 0x08, command 0x44), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Even-to-odd harmonic blend. 0% = rectifier only (even harmonics: warm, smooth, octave-flavored); 100% = clipper only (odd harmonics: harder, growlier, more "electric bass"). The 50% default gives the consecutive series that produces the strongest missing-fundamental pitch perception. Present this in a UI as a "warm ... aggressive" tone control.

### 2.6 original_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | -60.0 to 0.0 (dB) |
| **Default** | 0.0 |
| **Command** | `FX.PSYBASS_ORIGINAL` (page 0x08, command 0x45), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Level of the original low band (below cutoff) in the output. 0 dB leaves the original bass untouched (harmonics are added on top). Negative values attenuate the fundamental the speaker cannot reproduce anyway, freeing driver excursion and amplifier headroom; -60 dB is effectively full removal. Implemented as `out += (g_orig - 1) * low`, so it is exact complementary attenuation of the LP2 band, not a shelf approximation.

### 2.7 output_mask

| Property | Value |
|----------|-------|
| **Type** | `uint16_t` |
| **Range** | bit k = process output channel k; bits above the platform's channel count are ignored |
| **Default** | 0xFFFF (all outputs) |
| **Command** | `FX.PSYBASS_MASK` (page 0x08, command 0x46), GET and SET |
| **Payload** | 2 bytes: little-endian uint16 |

Selects which output channels are processed. Output channel indexing:

| Platform | Bits 0-7 | PDM sub bit |
|----------|----------|-------------|
| RP2350 | outputs 0-7 (S/PDIF or I2S slots 1-4, L/R interleaved) | bit 8 |
| RP2040 | bits 0-3: outputs 0-3 (slots 1-2) | bit 4 |

Masked-off outputs cost zero per-sample CPU and have their filter state cleared each packet, so toggling a mask bit on is always transient-free. Recommendation: mask off the PDM subwoofer output (and any full-range/sub outputs), since synthesizing harmonics on a channel that can reproduce real bass is counterproductive. The all-outputs default is safe but not optimal; a typical app sets the mask to exactly the small-speaker outputs.

Mask changes take effect on the next audio packet without a coefficient recompute.

---

## 3. Vendor Command Transport

Psybass uses the standard DSPi vendor command surface, the `FX` page (0x08) of the paged protocol (see `../commands.md`), so it is reachable over every control transport (USB EP0, UART, I2C target, control surfaces engine) with the same commands. Each parameter is one command, and the transfer direction selects GET or SET.

**Control Surfaces** (caps v4+): six front-panel nouns map onto these commands, so physical knobs/buttons/IR commands can drive the effect: `PSYBASS` (41, enable), `PSYBASS_CUTOFF` (42, log stepping), `PSYBASS_HARMONICS` (43), `PSYBASS_DRIVE` (44), `PSYBASS_CHARACTER` (45), `PSYBASS_ORIGINAL` (46). The output mask stays host-only. See `control_surfaces_spec.md` sections 4.3 and 5.

### USB (primary transport)

- **SET**: control transfer, `bmRequestType = 0x41` (vendor, interface, host-to-device), `bRequest = <command>`, `wValue = 0`, `wIndex = (0x08 << 8) | 2`, data stage = payload as documented per parameter.
- **GET**: control transfer, `bmRequestType = 0xC1` (vendor, interface, device-to-host), `bRequest = <command>`, `wValue = 0`, `wIndex = (0x08 << 8) | 2`, `wLength` = response size; the device returns the payload in the data stage.

### Command summary

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Meaning |
|---|---|---|---|---|---|---|
| `FX.PSYBASS_ENABLE` | 0x08 | 0x40 | GET, SET | none (0) | GET 1, SET 1 | Enable (u8 bool) |
| `FX.PSYBASS_CUTOFF` | 0x08 | 0x41 | GET, SET | none (0) | GET 4, SET 4 | Cutoff (Hz, clamps 30-300) |
| `FX.PSYBASS_HARMONICS` | 0x08 | 0x42 | GET, SET | none (0) | GET 4, SET 4 | Harmonic level (dB, clamps -24..+12) |
| `FX.PSYBASS_DRIVE` | 0x08 | 0x43 | GET, SET | none (0) | GET 4, SET 4 | Drive (dB, clamps 0..18) |
| `FX.PSYBASS_CHARACTER` | 0x08 | 0x44 | GET, SET | none (0) | GET 4, SET 4 | Character (%, clamps 0..100) |
| `FX.PSYBASS_ORIGINAL` | 0x08 | 0x45 | GET, SET | none (0) | GET 4, SET 4 | Original bass level (dB, clamps -60..0) |
| `FX.PSYBASS_MASK` | 0x08 | 0x46 | GET, SET | none (0) | GET 2, SET 2 | Output mask (u16 LE) |

Floats are 4-byte IEEE 754 LE. A SET payload of any other length is `VC_ERR_LENGTH`, a non-zero `wValue` is `VC_ERR_ADDRESS`, and a NaN or infinite float is `VC_ERR_VALUE` (nothing is stored). A rejected SET STALLs on USB and the reason is readable from `SYS.LAST_RESULT`.

### Apply semantics

- Every SET updates live state immediately. Enable and the five value parameters raise an internal recompute flag; the firmware main loop rebuilds the coefficient set (double-buffered, glitch-free) and publishes it, typically within a few milliseconds. No stream interruption, no click, no alignment disturbance.
- The mask SET takes effect on the next audio packet directly (no recompute needed).
- SETs are **not persisted** to flash by themselves. Persistence happens when the user saves a preset (`STORE.PRESET_SAVE`), following the same convention as loudness/crossfeed/leveller. The 1.1.6 save-to-active-slot opcode is retired. A host saves to the active slot by reading `STORE.PRESET_ACTIVE` and sending `STORE.PRESET_SAVE` to that slot.

### Change notifications

Each SET emits a parameter-write notification on the notification endpoint (see `notification_protocol_v2_spec.md`) whose offset/length identify the changed field inside the bulk wire structure (section 4). A second host UI listening to notifications can therefore mirror psybass changes live, exactly as for the other DSP features.

---

## 4. Bulk Wire Format (`STORE.ALL_PARAMS`)

Psybass appears in `WireBulkParams` from **wire format version 23** as the final section. `WIRE_FORMAT_VERSION` mismatches are rejected wholesale by the firmware, so a host must speak V23 to use bulk transfer with this firmware (total packet size 5900 bytes).

`WirePsybassParams`, 24 bytes, at byte offset **5876** within `WireBulkParams`:

| Offset | Size | Type | Field |
|--------|------|------|-------|
| +0 | 1 | uint8 | enabled (0/1) |
| +1 | 1 | uint8 | reserved (write 0) |
| +2 | 2 | uint16 LE | output_mask |
| +4 | 4 | float LE | cutoff_hz |
| +8 | 4 | float LE | harmonics_db |
| +12 | 4 | float LE | drive_db |
| +16 | 4 | float LE | character_pct |
| +20 | 4 | float LE | original_db |

On a bulk SET, all psybass fields are applied and coefficients recompute automatically. On a bulk GET, the section reflects live state including clamping.

---

## 5. Persistence

- **Preset slots (flash):** psybass fields are stored per preset from `SLOT_DATA_VERSION` 31. Presets saved by older firmware (V21-V30) load with psybass defaults (disabled, mask 0xFFFF, cutoff 80, harmonics 0, drive 6, character 50, original 0); no data is lost or misread. Preset save (`STORE.PRESET_SAVE`) captures the live psybass state; preset load (`STORE.PRESET_LOAD`) restores it and recomputes coefficients.
- **Factory reset (`STORE.FACTORY_RESET`):** restores the defaults above.
- **Startup:** the boot preset (or factory defaults) determines the psybass state at power-on; the effect is fully initialized before audio starts.

---

## 6. App Integration Patterns

### Startup / reconnect sync

1. Read `STORE.ALL_PARAMS` and parse the psybass section at offset 5876 (verify `format_version` in the header first), **or** issue the seven individual GETs (`FX.PSYBASS_ENABLE` to `FX.PSYBASS_MASK`, commands 0x40-0x46).
2. Populate the UI from the returned values. Do not assume defaults; the device may have loaded a preset.

### Live control

- Sliders should send SETs on change (they are cheap and glitch-free); read-back is unnecessary except to reflect clamping, since the firmware clamps silently. If the app enforces the documented ranges itself, its state and the device's state stay identical.
- There is no need to disable/re-enable around parameter changes; coefficient updates are seamless.

### Typical UI

A minimal UI is a single enable switch plus a "bass boost" slider mapped to `harmonics_db` (-24..+12). A full UI exposes: enable; cutoff (log slider, 30-300 Hz); harmonics (slider, dB); drive (slider, dB); character (warm-to-aggressive slider, %); original bass (slider, dB, labelled "speaker protection" or "excursion control"); per-output checkboxes building the mask.

### Suggested starting points

| Use case | cutoff | harmonics | drive | character | original |
|----------|--------|-----------|-------|-----------|----------|
| Bookshelf speakers, gentle help | 60 | 0 | 6 | 50 | 0 |
| Small Bluetooth speaker | 100 | +3 | 9 | 40 | -12 |
| Laptop / tablet speakers | 180 | +6 | 12 | 50 | -24 |
| Headphone "more bass feel" | 45 | -3 | 6 | 30 | 0 |

### Feature detection

Every firmware that speaks the paged protocol (1.1.7 and later) has psybass. A host that gates features on the command table checks the GET bit of `FX.PSYBASS_ENABLE` in `SYS.PAGE_COMMANDS` for page 0x08, or the FX page revision in `SYS.PAGE_LIST`. Firmware 1.1.6 and older does not answer the paged protocol (`vendor_command_overhaul_plan.md` section 5.14).

---

## 7. Interactions and Edge Cases

- **Crossover:** psybass runs before the per-output crossover, so a high-pass crossover does not starve it of bass. Pairing a HP crossover at the psybass cutoff with `original_db = 0` is equivalent to (and steeper than) using `original_db = -60` without a crossover.
- **Loudness compensation:** independent and compatible. Loudness runs post-gain; psybass pre-gain. Both have their own output masks.
- **Volume leveller / crossfeed:** run earlier in the chain (input side / per pair); psybass processes their output like any other content.
- **Test signals (signal generator):** outputs carrying a RAW test signal bypass psybass (as they bypass all per-output processing), so measurement signals are never colored.
- **Muted or disabled outputs:** skipped entirely; state is cleared so unmuting is transient-free.
- **Sample rate changes:** coefficients recompute automatically for 44.1/48/96 kHz; no host action needed.
- **Multichannel USB (RP2350 4/6/8-channel alts):** fully supported; psybass is input-count agnostic since it runs post-matrix.
- **Clipping headroom:** the effect adds energy. With `harmonics_db` high, hot content, and `original_db = 0`, the summed output can clip (the output meters and clip flags at channels CH_OUT_* report it). Reducing `original_db` usually buys back more headroom than the harmonics consume; otherwise lower the output gain or harmonics level.
- **CPU cost:** roughly 1.5-2x the cost of loudness compensation per enabled output. On RP2040, enabling all outputs at 96 kHz is the worst case; monitor `cpu0_load`/`cpu1_load` in `SYS.SYSTEM_STATUS` (bytes 12 and 13) if you enable it broadly. Masked-off outputs cost nothing.

---

## 8. Implementation Summary (firmware reference)

| Aspect | Detail |
|--------|--------|
| Module | `firmware/DSPi/psybass.h` / `psybass.c` |
| Filters (RP2350) | TPT SVF (Cytomic) LP2/HP2 at cutoff, one-pole LP at 4x cutoff, float |
| Filters (RP2040) | RBJ LP2/HP2 biquads (TDF2, Q28), one-pole LP, `fast_mul_q28` |
| NLD | `|x|` (even) blended with cubic soft clip `1.5d - 0.5d^3` of drive-scaled, clamped input (odd) |
| Fixed-point safety | Low band clamped to +/-1.0 before the drive multiply so no Q28 wrap at any legal setting |
| Coefficients | One global set, double-buffered, pointer-published (`current_psybass_coeffs`, NULL = off), rebuilt in the main loop on parameter/rate change |
| State | `psybass_output_state[NUM_OUTPUT_CHANNELS]`, 5 words per output, owned by the core that owns the output, reset whenever the output is skipped |
| Dual-core | Coefficient pointer + mask snapshotted once per packet into `Core1EqWork` so both cores apply one consistent view |
| Latency | 0 samples added; dry path untouched; inter-slot alignment preserved by construction |
| Versions | Vendor commands `FX.PSYBASS_ENABLE` to `FX.PSYBASS_MASK` (page 0x08, 0x40-0x46); wire format V23; preset slot V31 |
| RAM | ~340 B (RP2040) / ~420 B (RP2350) total |
| Status | Implemented and audited; hardware listening test pending |
