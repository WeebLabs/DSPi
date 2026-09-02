# Subharmonic Synthesizer Specification

## 1. Overview

The subharmonic synthesizer ("subharm") is a dbx 120A style octave divider. It listens to the bass already in the program (48 to 112 Hz), synthesizes a new note exactly one octave below it (24 to 56 Hz), and mixes that note back in at a user-set level. It is the opposite of psychoacoustic bass: where psybass adds harmonics above the bass so a small speaker can imply a fundamental it cannot play, subharm adds a real fundamental below the bass for systems that can reproduce it (subwoofers, large full-range systems, club and cinema playback).

Both platforms are supported. The kernel is written once against a small number-type abstraction: RP2350 runs it in single-precision float, RP2040 in Q28 fixed point through `fast_mul_q28`.

### Key characteristics

- **Two fixed bands with independent levels, as on the dbx.** Program content in 48 to 72 Hz produces a 24 to 36 Hz sub; content in 72 to 112 Hz produces a 36 to 56 Hz sub. Each band has its own level control, so the synthesized response can be tuned to the system.
- **Waveform tracking.** The divider does not generate a tone. It flips the polarity of the band-passed program signal once per cycle, so the synthesized sub has the same amplitude envelope as the bass that produced it. Bass that decays produces a sub that decays with it.
- **Dual dividers for polyphony.** Each band has its own divider, so a kick drum in the lower band and a bass note in the upper band are divided independently. A single divider over the whole band would lose lock on that mixture.
- **History-independent level.** The two bands' subs are held in phase quadrature by a small phase-alignment allpass, so the level of a note near the band boundary does not depend on the polarity state the previous note left behind. Without it the same note could come back up to 4.7 dB different.
- **LF boost bell.** A gentle bell centred on 70 Hz, applied after the subs are summed, fills the gap between the synthesized sub and the program's mid-bass. This mirrors the dbx LF Boost.
- **Exact headroom reading.** The firmware computes the worst-case gain of the current configuration and reports it as the preamp headroom a host must free. The effect is amplitude-linear, so lowering the preamp by the reported amount is an exact fix, not an approximation.
- **Zero added latency.** Pure IIR, in place. Inter-output-slot sample alignment is untouched by construction.

### Signal flow (per selected output channel)

```
   in ----+------------------------------------------------------------+
          |                                                            |
          +--> HP2 48 Hz --> LP2 112 Hz --+--> LP2 72 Hz --> band 0    |
                                          |                  (48-72)   v
                                          +--> HP2 72 Hz --> band 1   sum --> bell 70 Hz --> out
                                                             (72-112)  ^
   band 0 --> divider --> LP2 40 Hz ---------------------> x g_low ----+
   band 1 --> divider --> LP2 62 Hz --> AP1 160 Hz ------> x g_high ---+

   divider: env    = max(|s|, env * decay)            peak follower, 40 ms
            armed  = 1 once s < -env/4                 hysteresis
            on the first s >= 0 while armed: flip polarity, clear armed
            d      = polarity ? -s : s
```

- Every filter is a topology-preserving-transform (TPT) state-variable filter. The band split and the post-divider lowpasses are Butterworth (Q = 0.7071). The 72 Hz split is one SVF whose lowpass output is band 0 and whose highpass output is band 1, so the whole split costs three filters.
- The divider flips at the zero crossing itself, so the divided waveform is continuous. Arming on an envelope-relative threshold (a quarter of the band's peak) is what stops beating partials and noise from re-triggering inside one cycle, and it keeps the divider working at any signal level.
- A band sine of frequency f, polarity-flipped every cycle, has components at f/2 (amplitude 0.849 of the band), 3f/2 (0.509), 5f/2 (0.121) and so on, and nothing at f. The post lowpass keeps f/2 and strips the rest.
- The allpass on band 1 matches the phase lag of the 62 Hz lowpass to that of the 40 Hz lowpass over 24 to 56 Hz (within about 5 degrees). Highpass and lowpass outputs of one SVF are 180 degrees apart at every frequency, which puts the two dividers' square waves a quarter of a sub cycle apart, so with matched post-filter phase the two subs add in quadrature and their sum has the same magnitude whichever way the flip-flops happen to be set.
- `g_low = 10^(low_db/20)`, `g_high = 10^(high_db/20)`; a band at the -30 dB floor is off and its divider and lowpass are skipped. The bell is the Cytomic form: `out = x + k (A^2 - 1) v1` with `A = 10^(boost_db/40)`, `k = 1/(Q A)`, Q = 0.9.

### Signal chain position

Subharm runs **per output channel, post-matrix, pre-crossover, ahead of psybass** (PASS 5-7 entry):

```
PASS 4:   Matrix Mixing (fan-out to output channels)
PASS 4.5: Crossfeed (per output pair)
             |
          Subharmonic Synthesizer   <-- HERE (per output, masked)
             |
          Psychoacoustic Bass -> Crossover -> Per-Output PEQ -> Gain/Volume -> Loudness -> Delay
             |
          Output Encoding (S/PDIF, I2S, ADAT, PDM)
```

Pre-crossover placement means a subwoofer output with a lowpass crossover still passes the synthesized sub, and a satellite output with a highpass crossover has the sub removed again by that crossover. Running ahead of psybass means the divider sees the program bass rather than harmonics psybass synthesized from it. Because subharm runs pre-gain its character does not change with volume.

---

## 2. Parameters

All floats on the wire are little-endian IEEE 754 single-precision. All SET values are clamped by the firmware to the documented range; a GET after a SET returns the clamped value.

### 2.1 enabled

| Property | Value |
|----------|-------|
| **Type** | `bool` (uint8_t on wire) |
| **Range** | 0 (off) or 1 (on) |
| **Default** | 0 (disabled) |
| **SET command** | `0x10` (`REQ_SET_SUBHARM`) |
| **GET command** | `0x11` (`REQ_GET_SUBHARM`) |
| **Payload** | 1 byte: `0x00` = disabled, `0x01` = enabled |

Master enable. When disabled the coefficient pointer is unpublished and per-output processing is skipped entirely (zero per-sample CPU cost); per-output state is cleared so re-enabling starts transient-free.

### 2.2 low_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | -30.0 to +6.0 (dB); -30.0 = band off |
| **Default** | 0.0 |
| **SET command** | `0x12` (`REQ_SET_SUBHARM_LOW`) |
| **GET command** | `0x13` (`REQ_GET_SUBHARM_LOW`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Level of the 24 to 36 Hz sub, synthesized from program content in 48 to 72 Hz. At 0 dB the sub's fundamental is 0.85 of the band amplitude (the divider's natural gain). The floor value turns the band off and skips its processing.

### 2.3 high_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | -30.0 to +6.0 (dB); -30.0 = band off |
| **Default** | 0.0 |
| **SET command** | `0x14` (`REQ_SET_SUBHARM_HIGH`) |
| **GET command** | `0x15` (`REQ_GET_SUBHARM_HIGH`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Level of the 36 to 56 Hz sub, synthesized from program content in 72 to 112 Hz. Same semantics as `low_db`.

### 2.4 boost_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | 0.0 to +6.0 (dB) |
| **Default** | 0.0 |
| **SET command** | `0x16` (`REQ_SET_SUBHARM_BOOST`) |
| **GET command** | `0x17` (`REQ_GET_SUBHARM_BOOST`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Gain of the LF boost bell (70 Hz, Q 0.9, roughly 40 to 120 Hz at half gain), applied to the whole output after the subs are summed. 0 dB skips the stage. The +6 dB ceiling keeps the RP2040 Q28 sum inside its representable range at every legal setting; the boost is meant to be gentle, as on the dbx.

### 2.5 output_mask

| Property | Value |
|----------|-------|
| **Type** | `uint16_t` |
| **Range** | bit k = process output channel k; bits above the platform's channel count are ignored |
| **Default** | 0xFFFF (all outputs) |
| **SET command** | `0x18` (`REQ_SET_SUBHARM_MASK`) |
| **GET command** | `0x19` (`REQ_GET_SUBHARM_MASK`) |
| **Payload** | 2 bytes: little-endian uint16 |

Selects which output channels are processed. Output channel indexing:

| Platform | Bits 0-7 | PDM sub bit |
|----------|----------|-------------|
| RP2350 | outputs 0-7 (S/PDIF or I2S slots 1-4, L/R interleaved) | bit 8 |
| RP2040 | bits 0-3: outputs 0-3 (slots 1-2) | bit 4 |

Masked-off outputs cost zero per-sample CPU and have their state cleared each packet. The all-outputs default is safe (the effect ships disabled) but a typical app masks exactly the subwoofer or full-range outputs. Mask changes take effect on the next audio packet without a coefficient recompute.

### 2.6 headroom (read-only)

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | 0.0 upward (dB); 0.0 while disabled |
| **GET command** | `0x1A` (`REQ_GET_SUBHARM_HEADROOM`) |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

The worst-case gain of the current configuration: the amount by which a full-scale input could exceed full scale after the effect, and therefore the preamp headroom a host must free so the effect cannot clip. It is computed from the live configuration on every GET (no cached value can race the coefficient recompute after a SET), so a host can SET and immediately GET.

The figure is a steady-state tone bound scanned on an eighth-octave grid from 16 to 256 Hz:

```
peak(f)   = bell(f) + sum over bands b of  g_b * band_b(f) * sum over n in {1,3,5,7} of  c_n * lp_b(n f/2) * bell(n f/2)
headroom  = 20 log10( max over f of peak(f) )

band_0(f) = hp2(f/48) lp2(f/112) lp2(f/72)      band_1(f) = hp2(f/48) lp2(f/112) hp2(f/72)
c_n       = 0.849, 0.509, 0.121, 0.057          (divided-waveform Fourier amplitudes)
lp2, hp2  = 2nd-order Butterworth magnitudes    bell = the LF boost magnitude (1 when off)
```

Components at different frequencies are summed as amplitudes (worst-case phase), so the bound is conservative by 1 to 2 dB on real tones. Offline verification against a model of the kernel: one band at 0 dB reads 4.2 dB (measured worst 3.3 dB); both bands at 0 dB read 6.3 dB (measured 4.4 dB); everything at +6 dB reads 13.5 dB (measured 11.2 dB); the bell alone at +6 dB reads 6.0 dB (measured 6.0 dB). The bound never reads below the measured peak.

Because the divider preserves the band amplitude and every other stage is linear, a preamp cut of X dB ahead of the effect lowers the synthesized sub by exactly X dB as well. Lowering the per-channel preamp on the masked outputs' sources by the reported amount is therefore an exact correction, unlike psybass whose drive and clipper respond nonlinearly to level.

---

## 3. Vendor Command Transport

Subharm uses the standard DSPi vendor command surface, so it is reachable over every control transport (USB EP0, UART, I2C target, control surfaces engine) with the same command bytes. These are the first application commands allocated inside 0x00 to 0x1F; 0x01 remains the Microsoft OS descriptor vendor code and is intercepted before the dispatcher.

**Control Surfaces** (caps v14+): four front-panel nouns map onto these commands: `SUBHARM` (57, enable), `SUBHARM_LOW` (58), `SUBHARM_HIGH` (59), `SUBHARM_BOOST` (60). The output mask and the headroom reading stay host-only. See `control_surfaces_spec.md` sections 4.3 and 5.

### USB (primary transport)

- **SET**: control transfer, `bmRequestType = 0x40` (vendor, host-to-device), `bRequest = <command>`, `wValue = 0`, `wIndex = 0`, data stage = payload as documented per parameter.
- **GET**: control transfer, `bmRequestType = 0xC0` (vendor, device-to-host), `bRequest = <command>`, `wLength` >= response size; the device returns the payload in the data stage.

### Command summary

| Command | Direction | Payload | Meaning |
|---------|-----------|---------|---------|
| 0x10 | SET | 1 byte bool | Enable/disable |
| 0x11 | GET | 1 byte bool | Enabled state |
| 0x12 | SET | 4-byte float | 24-36 Hz band level (dB, clamps -30..+6; -30 = off) |
| 0x13 | GET | 4-byte float | 24-36 Hz band level |
| 0x14 | SET | 4-byte float | 36-56 Hz band level (dB, clamps -30..+6; -30 = off) |
| 0x15 | GET | 4-byte float | 36-56 Hz band level |
| 0x16 | SET | 4-byte float | LF boost (dB, clamps 0..+6) |
| 0x17 | GET | 4-byte float | LF boost |
| 0x18 | SET | 2-byte uint16 LE | Output mask |
| 0x19 | GET | 2-byte uint16 LE | Output mask |
| 0x1A | GET | 4-byte float | Headroom to free (dB, 0 while disabled) |

### Apply semantics

- Every SET updates live state immediately. Enable and the three level parameters raise an internal recompute flag; the firmware main loop rebuilds the coefficient set (double-buffered, glitch-free) and publishes it, typically within a few milliseconds. No stream interruption, no click, no alignment disturbance.
- The mask SET takes effect on the next audio packet directly.
- SETs are **not persisted** to flash by themselves. Persistence happens when the user saves a preset (`REQ_PRESET_SAVE` 0x90) or via `REQ_SAVE_PARAMS` (0x51), following the same convention as psybass.

### Change notifications

Each SET emits a parameter-write notification on the notification endpoint whose offset/length identify the changed field inside the bulk wire structure (section 4). A second host UI can mirror subharm changes live and re-read the headroom (0x1A) after any of them.

---

## 4. Bulk Wire Format (GET/SET_ALL_PARAMS 0xA0/0xA1)

Subharm appears in `WireBulkParams` from **wire format version 29** as the final section (total packet size 5960 bytes).

`WireSubharmParams`, 16 bytes, at byte offset **5944** within `WireBulkParams`:

| Offset | Size | Type | Field |
|--------|------|------|-------|
| +0 | 1 | uint8 | enabled (0/1) |
| +1 | 1 | uint8 | reserved (write 0) |
| +2 | 2 | uint16 LE | output_mask |
| +4 | 4 | float LE | low_db |
| +8 | 4 | float LE | high_db |
| +12 | 4 | float LE | boost_db |

On bulk SET (0xA1), all subharm fields are applied and coefficients recompute automatically. On bulk GET (0xA0), the section reflects live state including clamping. The headroom reading is not part of the wire structure; read it with 0x1A.

---

## 5. Persistence

- **Preset slots (flash):** subharm fields are stored per preset from `SLOT_DATA_VERSION` 36 (16 bytes tail-appended). Presets saved by older firmware (V21 to V35) load with subharm defaults (disabled, mask 0xFFFF, both bands 0 dB, boost 0 dB); no data is lost or misread. Preset save (0x90) captures the live state; preset load (0x91) restores it and recomputes coefficients.
- **Factory reset (0x53):** restores the defaults above.
- **Startup:** the boot preset (or factory defaults) determines the state at power-on; the effect is fully initialized before audio starts.

---

## 6. App Integration Patterns

### Startup / reconnect sync

1. Read `GET_ALL_PARAMS` (0xA0) and parse the subharm section at offset 5944 (verify `format_version == 29` first), **or** issue the five individual GETs (0x11, 0x13, 0x15, 0x17, 0x19).
2. Read the headroom (0x1A) and show it next to the effect.

### Live control

- Sliders send SETs on change; the firmware clamps silently, so an app that enforces the documented ranges keeps identical state.
- After any SET, re-read 0x1A. Offer a "free headroom" action that lowers the per-channel preamp (`REQ_SET_PREAMP_CH`) on the inputs feeding the masked outputs by the reported amount. Show the reading as a requirement, not a suggestion: the effect is amplitude-linear, so the number is exact.
- The headroom reading does not depend on the mask or the sample rate, only on enable, the two band levels and the boost.

### Typical UI

Enable; two sliders labelled "24-36 Hz" and "36-56 Hz" (-30 to +6 dB, floor shown as "Off"); an "LF Boost" slider (0 to +6 dB); per-output checkboxes building the mask; a headroom readout with an apply button.

### Suggested starting points

| Use case | low | high | boost |
|----------|-----|------|-------|
| Subwoofer feed, subtle weight | -6 | -6 | 0 |
| Club / large system | 0 | 0 | +3 |
| Thin recordings, add fundamental | 0 | -6 | +3 |
| Cinema LFE emphasis | +3 | -12 | 0 |

### Feature detection

There is no capability bit. Detect support by firmware version, by `format_version >= 29` in the bulk header, or by issuing `REQ_GET_SUBHARM` (0x11) and treating a failed control transfer as "unsupported".

---

## 7. Interactions and Edge Cases

- **Polyphony.** Two notes in different bands (kick at 55 Hz plus bass at 100 Hz) are divided independently and cleanly (spurious content below the subs at -21 dB in the offline model). Two notes inside one band confuse that band's divider; this is inherent to any octave divider and matches the dbx.
- **Crossover.** Subharm runs before the per-output crossover. A subwoofer output with a lowpass crossover keeps the sub; a satellite output with a highpass crossover loses it again, so mask satellites off to save CPU rather than relying on the crossover.
- **Psychoacoustic bass.** Independent and compatible; subharm runs first. Enabling both on one output is unusual (they pull in opposite directions) but legal.
- **Loudness, leveller, crossfeed.** Independent; loudness runs post-gain, the leveller and crossfeed earlier in the chain.
- **Test signals.** Outputs carrying a RAW signal-generator signal bypass subharm, as they bypass all per-output processing.
- **Muted or disabled outputs.** Skipped; state cleared so unmuting is transient-free.
- **Sample rate changes.** Coefficients recompute automatically for 44.1/48/96 kHz.
- **Very quiet input.** The envelope-relative threshold keeps the divider working at any level. Below the noise floor the divider toggles on noise, but its output is that noise and is inaudible.
- **Sub-48 Hz program content.** The 48 Hz highpass keeps real sub-bass and DC out of the dividers, so a 30 Hz note does not produce a 15 Hz sub-sub.
- **RP2040 fixed point.** Each band signal is clamped to +/-1.0 before its divider and the boost ceiling is +6 dB, so no legal setting can wrap `fast_mul_q28` past +/-8.0 on inputs up to 0 dBFS. Inputs driven above 0 dBFS by preamp and matrix gain are in the same regime as the PEQ.
- **CPU cost.** Six SVFs, two dividers and one allpass per masked output per sample: about 1.7x psybass. Masked-off outputs and skipped bands or bell cost nothing beyond a state clear. Enable it only on the outputs that need it.

---

## 8. Implementation Summary (firmware reference)

| Aspect | Detail |
|--------|--------|
| Module | `firmware/DSPi/subharm.h` / `subharm.c` |
| Kernel | One source, two number types: `sh_num_t` is float (RP2350) or Q28 int32 (RP2040) with `sh_mul` / `sh_twice` / `sh_quarter` / `sh_abs` / `sh_band_limit` helpers; a single RAM-resident (`DSP_TIME_CRITICAL`) out-of-line function shared by both cores |
| Filters | TPT SVF throughout: HP2 48 Hz, LP2 112 Hz, split SVF 72 Hz (LP out = band 0, HP out = band 1), post LP2 40 / 62 Hz, AP1 160 Hz on band 1, bell 70 Hz Q 0.9 |
| Divider | Peak follower (40 ms), arm below -env/4, flip at the next non-negative sample, conditional negate |
| Coefficients | One global set, double-buffered, pointer-published (`current_subharm_coeffs`, NULL = off), rebuilt in the main loop on parameter/rate change |
| State | `subharm_output_state[NUM_OUTPUT_CHANNELS]`, 68 bytes per output, owned by the core that owns the output, reset whenever the output is skipped; skipped bands and bell keep zeroed state |
| Dual-core | Coefficient pointer + mask snapshotted once per packet into `Core1EqWork` (`subharm_coeffs` / `subharm_mask`) |
| Headroom | `subharm_headroom_db()`: pure function of the config, analog-prototype magnitudes on a 33-point eighth-octave grid, computed on each 0x1A GET |
| Latency | 0 samples added; dry path untouched; inter-slot alignment preserved by construction |
| Versions | Vendor commands 0x10-0x1A; wire format V29; preset slot V36; control surfaces caps v14 (nouns 57-60) |
| RAM | about 600 B (RP2040) / 900 B (RP2350) of state and coefficient buffers, plus one RAM kernel copy of roughly 1 KB |
| Status | Implemented, verified against an offline model of the kernel (float and Q28 bit-for-bit in behaviour); hardware listening test pending |
