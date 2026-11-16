# Subharmonic Synthesizer Specification

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7)*

## 1. Overview

The subharmonic synthesizer ("subharm") is a dbx 120A style octave divider, extended. It listens to the bass already in the program (48 to 160 Hz in three bands), synthesizes a new note exactly one octave below it (24 to 80 Hz), and mixes that note back in at a user-set level. It is the opposite of psychoacoustic bass: where psybass adds harmonics above the bass so a small speaker can imply a fundamental it cannot play, subharm adds a real fundamental below the bass for systems that can reproduce it (subwoofers, large full-range systems, club and cinema playback).

Both platforms are supported. The kernel is written once against a small number-type abstraction: RP2350 runs it in single-precision float, RP2040 in Q28 fixed point through `fast_mul_q28`.

### Key characteristics

- **Three fixed bands with independent levels.** Program content in 48 to 72 Hz produces a 24 to 36 Hz sub; content in 72 to 112 Hz produces a 36 to 56 Hz sub; content in 112 to 160 Hz produces a 56 to 80 Hz sub. The first two mirror the dbx; the third (off by default) serves systems whose subwoofer cannot reach the lowest octave. Each band has its own level control.
- **Waveform tracking.** The divider does not generate a tone. It flips the polarity of the band-passed program signal once per cycle, so the synthesized sub has the same amplitude envelope as the bass that produced it. Bass that decays produces a sub that decays with it.
- **One divider per band for polyphony.** Each band has its own divider, so a kick drum in the lower band and a bass note in the upper band are divided independently. A single divider over the whole range would lose lock on that mixture.
- **History-independent level.** Adjacent bands' subs are held in phase quadrature by small phase-alignment allpasses, and the outer bands (0 and 2) are parity-slaved when the same note drives both, so the level of a note near a band boundary does not depend on the polarity state the previous note left behind. Without these the same note could come back up to 4.7 dB (two bands) or 7 dB (three bands) different; with them the spread is under 0.8 dB with two bands and under 1.1 dB with three.
- **Selectivity.** An optional per-band gate weights the sub toward percussive material (a short burst after each attack) or toward sustained material (opens only after the band has rung for a hold time), so a kick can be extended without extending a bass line, or the reverse.
- **Sub ceiling.** An optional soft limiter caps the synthesized sub at an absolute level before it is mixed in, which protects a subwoofer and lets the host free only the ceiling's worth of headroom.
- **Linked stereo pairs.** Each S/PDIF output pair can synthesize one sub from its mono sum and feed it to both channels, as the dbx does. This prevents a panned event from leaving the two channels' dividers in opposite polarity, which would put a centred bass note's sub out of phase between the speakers.
- **Solo and meter.** A runtime solo removes the program signal from the masked outputs so the synthesized sub can be heard or measured on its own, and a per-output meter reports the sub's peak level.
- **Decimated core.** Everything below the bell works at about 8 kHz. That cuts the cost per sample by more than half against the full-rate kernel and raises the RP2040 fixed-point precision by about 13 dB, because the filter corners sit six times higher relative to the rate.
- **LF boost bell.** A gentle bell centred on 70 Hz, applied after the subs are summed, fills the gap between the synthesized sub and the program's mid-bass. This mirrors the dbx LF Boost.
- **Exact headroom reading.** The firmware computes the worst-case gain of the current configuration and reports it as the preamp headroom a host must free. The effect is amplitude-linear, so lowering the preamp by the reported amount is an exact fix, not an approximation.
- **Alignment-safe.** Pure IIR, in place; the dry signal is untouched and the sub lags it by one low-rate period (0.125 ms, under 4 degrees at 80 Hz) identically on every processed output. Inter-output-slot sample alignment is untouched by construction.

### Signal flow (per selected output channel)

```
   in --+--> LP2 160 Hz (full rate, anti-alias) --> [decimate by D] --> HP2 48 Hz --> s
        |                                                                            |
        |     s --> SVF 112 Hz --+--> LP out --> SVF 72 Hz --+--> LP out = band 0 (48-72)
        |                        |                           +--> HP out = band 1 (72-112)
        |                        +--> HP out = band 2 (112-160)
        |
        |     band 0 --> divider --> [gate] --> LP2 40 Hz -----------------> x g_low  --+
        |     band 1 --> divider --> [gate] --> LP2 62 Hz --> AP1 160/105 --> x g_high --+--> [ceiling] --> sub
        |     band 2 --> divider --> [gate] --> LP2 80 Hz --> AP1 130 -----> x g_top  --+       (meter)
        |
        +--> dry ------------------------------------------------------------------+
                                                                                    v
             sub --> [interpolate x D back to full rate] ---------------------> sum --> bell 70 Hz --> out

   divider: env    = max(|s|, env * decay)            peak follower, 40 ms
            armed  = 1 once s < -env/4                 hysteresis
            on the first s >= 0 while armed: flip polarity (or take band 0's, see below), clear armed
            d      = polarity ? -s : s
```

- Every filter is a topology-preserving-transform (TPT) state-variable filter. The band split and the post-divider lowpasses are Butterworth (Q = 0.7071). Each split SVF yields two bands at once (lowpass output and highpass output), so the three-way split costs three filters plus the 48 Hz highpass.
- The 160 Hz lowpass runs at the full sample rate and is both the top of band 2 and the anti-alias filter for the decimation. D = round(fs / 8000): 6 at 44.1 and 48 kHz, 12 at 96 kHz. Folded content lands at least 68 dB down. The sub is brought back to full rate by linear interpolation, whose images sit below -99 dB.
- The divider flips at the zero crossing itself, so the divided waveform is continuous. Arming on an envelope-relative threshold (a quarter of the band's peak) is what stops beating partials and noise from re-triggering inside one cycle, and it keeps the divider working at any signal level.
- A band sine of frequency f, polarity-flipped every cycle, has components at f/2 (amplitude 0.849 of the band), 3f/2 (0.509), 5f/2 (0.121) and so on, and nothing at f. The post lowpass keeps f/2 and strips the rest.
- Highpass and lowpass outputs of one SVF are 180 degrees apart at every frequency, which puts two adjacent dividers' square waves a quarter of a sub cycle apart. The allpass on band 1 then matches its post-lowpass lag to band 0's (160 Hz with two bands, 105 Hz with three) and the allpass on band 2 does the same against band 1 (130 Hz), so adjacent subs add in quadrature and the sum has the same magnitude whichever way the flip-flops happen to be set.
- Bands 0 and 2 are not adjacent and no allpass can hold them in quadrature, yet with second-order edges a 90 Hz note leaks into both at about -6 dB. When their measured flip periods agree within 1/16 (the same note is driving both), band 2's flip-flop takes its polarity from the sign of band 0's divided signal at the flip instant instead of toggling. Setting the polarity exactly at a zero crossing is click-free. Different notes in the two bands have different periods, so they are never slaved. Offline model, all eight parity states: spread under 1.1 dB across 56 to 128 Hz with three bands, under 0.8 dB with two.
- The selectivity gate multiplies each band's divided signal before its post lowpass, so gate steps are smoothed by the lowpass. See section 2.8.
- `g_low = 10^(low_db/20)`, `g_high = 10^(high_db/20)`, `g_top = 10^(top_db/20)`; a band at the -30 dB floor is off and its divider, gate and lowpass are skipped. The bell is the Cytomic form: `out = x + k (A^2 - 1) v1` with `A = 10^(boost_db/40)`, `k = 1/(Q A)`, Q = 0.9.

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

With `link_pairs` on (the default), an S/PDIF pair whose two outputs are both eligible (masked, enabled, not muted, not RAW) is processed as one unit: the sub is synthesized from the pair's mono sum and mixed into both outputs, and each output keeps its own bell. A pair with only one eligible output, and the PDM output, are processed singly.

---

## 2. Parameters

All floats on the wire are little-endian IEEE 754 single-precision. All finite SET values are clamped by the firmware to the documented range; a GET after a SET returns the clamped value. A NaN or infinite float is refused with `VC_ERR_VALUE`.

### 2.1 enabled

| Property | Value |
|----------|-------|
| **Type** | `bool` (uint8_t on wire) |
| **Range** | 0 (off) or 1 (on) |
| **Default** | 0 (disabled) |
| **Command** | `FX.SUBHARM_ENABLE` (page 0x08, command 0x20), GET and SET |
| **Payload** | 1 byte: `0x00` = disabled, `0x01` = enabled |

Master enable. When disabled the coefficient pointer is unpublished and per-output processing is skipped entirely (zero per-sample CPU cost); per-output state is cleared so re-enabling starts transient-free.

### 2.2 low_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | -30.0 to +12.0 (dB); -30.0 = band off |
| **Default** | 0.0 |
| **Command** | `FX.SUBHARM_LOW` (page 0x08, command 0x21), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Level of the 24 to 36 Hz sub, synthesized from program content in 48 to 72 Hz. At 0 dB the sub's fundamental is 0.85 of the band amplitude (the divider's natural gain). The floor value turns the band off and skips its processing.

### 2.3 high_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | -30.0 to +12.0 (dB); -30.0 = band off |
| **Default** | 0.0 |
| **Command** | `FX.SUBHARM_HIGH` (page 0x08, command 0x22), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Level of the 36 to 56 Hz sub, synthesized from program content in 72 to 112 Hz. Same semantics as `low_db`.

### 2.4 boost_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | 0.0 to +6.0 (dB) |
| **Default** | 0.0 |
| **Command** | `FX.SUBHARM_BOOST` (page 0x08, command 0x24), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Gain of the LF boost bell (70 Hz, Q 0.9, roughly 40 to 120 Hz at half gain), applied to the whole output after the subs are summed. 0 dB skips the stage. The +6 dB ceiling keeps the RP2040 Q28 sum inside its representable range at every legal setting; the boost is meant to be gentle, as on the dbx.

### 2.5 output_mask

| Property | Value |
|----------|-------|
| **Type** | `uint16_t` |
| **Range** | bit k = process output channel k; bits above the platform's channel count are ignored |
| **Default** | 0xFFFF (all outputs) |
| **Command** | `FX.SUBHARM_MASK` (page 0x08, command 0x25), GET and SET |
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
| **Command** | `FX.SUBHARM_HEADROOM` (page 0x08, command 0x2E), GET only |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

The worst-case gain of the current configuration: the amount by which a full-scale input could exceed full scale after the effect, and therefore the preamp headroom a host must free so the effect cannot clip. It is computed from the live configuration on every GET (no cached value can race the coefficient recompute after a SET), so a host can SET and immediately GET.

The figure is a steady-state tone bound scanned on an eighth-octave grid from 16 to 362 Hz:

```
sub(f)    = sum over bands b of  g_b * band_b(f) * sum over n in {1,3,5,7} of  c_n * lp_b(n f/2) * bell(n f/2)
            scaled down to the ceiling when the same sum without the bell exceeds it (ceiling on)
peak(f)   = bell(f) + sub(f)
headroom  = 20 log10( max over f of peak(f) )

pre(f)    = hp2(f/48) lp2(f/160)
band_0(f) = pre(f) lp2(f/112) lp2(f/72)   band_1(f) = pre(f) lp2(f/112) hp2(f/72)   band_2(f) = pre(f) hp2(f/112)
c_n       = 0.849, 0.509, 0.121, 0.057    (divided-waveform Fourier amplitudes)
lp2, hp2  = 2nd-order Butterworth magnitudes; bell = the LF boost magnitude (1 when off)
```

With the ceiling on, the reading is the smallest headroom H such that an input at -H dBFS cannot exceed full scale, found by bisection on the input level (the ceiling is an absolute level, so it only bites once the sub actually reaches it; simply capping the sub in the relative bound would under-report). Selectivity and the pair link can only lower the sub (gate weights are at most 1, and the mono sum is at most the larger channel), so they never raise the bound. The ceiling's few-millisecond onset overshoot is not in the bound; it is covered by the 1 to 2 dB conservatism below. Host check with the C kernel: freeing the reported H and sweeping 24 to 300 Hz at 0 to -40 dBFS never exceeds full scale at any ceiling.

Components at different frequencies are summed as amplitudes (worst-case phase), so the bound is conservative by 1 to 2 dB on real tones. Band levels reach +12 dB: at that setting the synthesized octave sits well above the bass that produced it, which is the intent for club and cinema use and for driving the ceiling as a sub maximizer, and the headroom reading grows accordingly (about 19 dB with everything at +12 dB and the bell at +6 dB). Offline verification against a model of the kernel: one band at 0 dB reads 4.2 dB (measured worst 3.3 dB); both bands at 0 dB read 6.3 dB (measured 4.4 dB); everything at +6 dB reads 13.5 dB (measured 11.2 dB); the bell alone at +6 dB reads 6.0 dB (measured 6.0 dB). The bound never reads below the measured peak.

Because the divider preserves the band amplitude and every other stage is linear, a preamp cut of X dB ahead of the effect lowers the synthesized sub by exactly X dB as well. Lowering the per-channel preamp on the masked outputs' sources by the reported amount is therefore an exact correction, unlike psybass whose drive and clipper respond nonlinearly to level.

### 2.7 top_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | -30.0 to +12.0 (dB); -30.0 = band off |
| **Default** | -30.0 (off) |
| **Command** | `FX.SUBHARM_TOP` (page 0x08, command 0x23), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

Level of the 56 to 80 Hz sub, synthesized from program content in 112 to 160 Hz. Same semantics as `low_db` and `high_db`. It ships at the floor because the third band reaches up into the range where a divided sub starts to compete with the program's own fundamentals.

### 2.8 select_mode

| Property | Value |
|----------|-------|
| **Type** | `uint8_t` |
| **Range** | 0 = all, 1 = percussive, 2 = sustained; values above 2 clamp to 2 |
| **Default** | 0 (all) |
| **Command** | `FX.SUBHARM_SELECT` (page 0x08, command 0x26), GET and SET |
| **Payload** | 1 byte |

Chooses which kind of bass material gets a synthesized sub. `all` treats every band signal alike. `percussive` favours short bursts after an attack. `sustained` favours notes that have already been ringing. `select_depth` sets how strongly the unfavoured material is suppressed and `select_hold_ms` sets the time span the choice is made over. This is a clamp, not a drop: a mode byte of 7 reads back as 2.

### 2.9 select_depth

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | 0.0 to 100.0 (percent) |
| **Default** | 100.0 |
| **Command** | `FX.SUBHARM_DEPTH` (page 0x08, command 0x27), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

How far the material `select_mode` does not favour is gated down. 0 % makes the selectivity inaudible whatever the mode; 100 % is full gating. Ignored when `select_mode` is 0.

### 2.10 select_hold_ms

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | 50.0 to 400.0 (ms) |
| **Default** | 150.0 |
| **Command** | `FX.SUBHARM_HOLD` (page 0x08, command 0x28), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

The time span the selectivity decision is made over. In `percussive` mode it is the burst length after an attack; in `sustained` mode it is how long a band must ring before the sub opens. Ignored when `select_mode` is 0.

### 2.11 ceiling_db

| Property | Value |
|----------|-------|
| **Type** | `float` |
| **Range** | -40.0 to 0.0 (dBFS); 0.0 = off |
| **Default** | 0.0 (off) |
| **Command** | `FX.SUBHARM_CEILING` (page 0x08, command 0x29), GET and SET |
| **Payload** | 4 bytes: little-endian IEEE 754 float |

A soft limit on the synthesized sub just before it is mixed back into the output. It caps how far the sub can push a driver without touching the program signal. A ceiling at full scale limits nothing useful, so 0 dBFS means the stage is off.

### 2.12 link_pairs

| Property | Value |
|----------|-------|
| **Type** | `bool` (uint8_t on wire) |
| **Range** | 0 or 1 |
| **Default** | 1 (linked) |
| **Command** | `FX.SUBHARM_LINK` (page 0x08, command 0x2A), GET and SET |
| **Payload** | 1 byte |

When set, each output pair's sub is synthesized from the pair's mono sum rather than from each channel on its own, so the two channels of a pair always receive the same sub. Bass is near-mono in most material and two independent dividers can otherwise land on opposite polarities. Like the mask, the pipeline reads this live each packet; no coefficient recompute is involved.

### 2.13 solo (runtime only)

| Property | Value |
|----------|-------|
| **Type** | `bool` (uint8_t on wire) |
| **Range** | 0 or 1 |
| **Default** | 0 (off) |
| **Command** | `FX.SUBHARM_SOLO` (page 0x08, command 0x2B), GET and SET |
| **Payload** | 1 byte |

A monitoring aid: masked outputs carry only the synthesized sub, with the program signal removed, so the effect can be heard on its own while it is being set up. It is **never** written to a preset slot and **never** appears in the bulk wire format, so no saved configuration and no bulk apply can leave a device playing with the program signal missing. A preset load leaves it untouched; a factory reset clears it. Read live each packet like the mask, so it needs no recompute. A host that offers solo should make it a momentary control and clear it when the panel closes.

### 2.14 sub meter (read-only)

| Property | Value |
|----------|-------|
| **Type** | array of `uint16_t` LE, one per output channel |
| **Range** | 0 to 32767 per entry |
| **Command** | `FX.SUBHARM_METER` (page 0x08, command 0x2F), GET only |
| **Payload** | `NUM_OUTPUT_CHANNELS` x 2 bytes: 18 on RP2350, 10 on RP2040 |

A decaying peak of the synthesized sub that was mixed into each output, on the same 0 to 32767 scale as the `MEAS.LEVELS` peaks, so one host meter widget can be driven from either source. Entry k is output channel k in the same order the output mask uses. The reading is of the sub alone, not the output, which is what makes it useful while dialling the band levels in.

---

## 3. Vendor Command Transport

Subharm uses the standard DSPi vendor command surface, the `FX` page (0x08) of the paged protocol (see `../commands.md`), so it is reachable over every control transport (USB EP0, UART, I2C target, control surfaces engine) with the same commands. Each parameter is one command, and the transfer direction selects GET or SET. The block occupies commands 0x20 to 0x2F of the page (0x2C and 0x2D are unused).

**Control Surfaces** (caps v15+): eleven front-panel nouns map onto these commands: `SUBHARM` (57, enable), `SUBHARM_LOW` (58), `SUBHARM_HIGH` (59), `SUBHARM_BOOST` (60), `SUBHARM_TOP` (61), `SUBHARM_SELECT` (62), `SUBHARM_DEPTH` (63), `SUBHARM_HOLD` (64), `SUBHARM_CEILING` (65), `SUBHARM_LINK` (66), `SUBHARM_SOLO` (67). Nouns 57-60 are caps v14; 61-67 are caps v15; caps v16 widens the band-level nouns to +12 dB. The output mask, the headroom reading and the sub meter stay host-only. The hold noun's front-panel span stops at 127 ms because the caps table encodes ranges as signed 8.8 fixed point; the command itself still accepts the full 50 to 400 ms. See `control_surfaces_spec.md` sections 4.3 and 5.

### USB (primary transport)

- **SET**: control transfer, `bmRequestType = 0x41` (vendor, interface, host-to-device), `bRequest = <command>`, `wValue = 0`, `wIndex = (0x08 << 8) | 2`, data stage = payload as documented per parameter.
- **GET**: control transfer, `bmRequestType = 0xC1` (vendor, interface, device-to-host), `bRequest = <command>`, `wValue = 0`, `wIndex = (0x08 << 8) | 2`, `wLength` = response size; the device returns the payload in the data stage.

### Command summary

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Meaning |
|---|---|---|---|---|---|---|
| `FX.SUBHARM_ENABLE` | 0x08 | 0x20 | GET, SET | none (0) | GET 1, SET 1 | Enable (u8 bool) |
| `FX.SUBHARM_LOW` | 0x08 | 0x21 | GET, SET | none (0) | GET 4, SET 4 | 24-36 Hz band level (dB, clamps -30..+12; -30 = off) |
| `FX.SUBHARM_HIGH` | 0x08 | 0x22 | GET, SET | none (0) | GET 4, SET 4 | 36-56 Hz band level (dB, clamps -30..+12; -30 = off) |
| `FX.SUBHARM_TOP` | 0x08 | 0x23 | GET, SET | none (0) | GET 4, SET 4 | 56-80 Hz band level (dB, clamps -30..+12; -30 = off) |
| `FX.SUBHARM_BOOST` | 0x08 | 0x24 | GET, SET | none (0) | GET 4, SET 4 | LF boost (dB, clamps 0..+6) |
| `FX.SUBHARM_MASK` | 0x08 | 0x25 | GET, SET | none (0) | GET 2, SET 2 | Output mask (u16 LE) |
| `FX.SUBHARM_SELECT` | 0x08 | 0x26 | GET, SET | none (0) | GET 1, SET 1 | Selectivity mode (u8, clamps to 0..2) |
| `FX.SUBHARM_DEPTH` | 0x08 | 0x27 | GET, SET | none (0) | GET 4, SET 4 | Selectivity depth (%, clamps 0..100) |
| `FX.SUBHARM_HOLD` | 0x08 | 0x28 | GET, SET | none (0) | GET 4, SET 4 | Selectivity hold (ms, clamps 50..400) |
| `FX.SUBHARM_CEILING` | 0x08 | 0x29 | GET, SET | none (0) | GET 4, SET 4 | Sub ceiling (dBFS, clamps -40..0; 0 = off) |
| `FX.SUBHARM_LINK` | 0x08 | 0x2A | GET, SET | none (0) | GET 1, SET 1 | Link output pairs to their mono sum (u8 bool) |
| `FX.SUBHARM_SOLO` | 0x08 | 0x2B | GET, SET | none (0) | GET 1, SET 1 | Solo the synthesized sub (u8 bool, runtime only) |
| `FX.SUBHARM_HEADROOM` | 0x08 | 0x2E | GET | none (0) | GET 4 | Headroom to free (dB float, 0 while disabled) |
| `FX.SUBHARM_METER` | 0x08 | 0x2F | GET | none (0) | GET var | Per-output sub peak (u16 per output, 0..32767 each) |

Floats are 4-byte IEEE 754 LE. A SET payload of any other length is `VC_ERR_LENGTH`, a non-zero `wValue` is `VC_ERR_ADDRESS`, and a NaN or infinite float is `VC_ERR_VALUE` (nothing is stored). A rejected SET STALLs on USB and the reason is readable from `SYS.LAST_RESULT`.

### Apply semantics

- Every SET updates live state immediately. Enable, the four level parameters, the selectivity settings and the ceiling raise an internal recompute flag; the firmware main loop rebuilds the coefficient set (double-buffered, glitch-free) and publishes it, typically within a few milliseconds. No stream interruption, no click, no alignment disturbance.
- The mask, the pair link and solo take effect on the next audio packet directly, with no recompute.
- SETs are **not persisted** to flash by themselves. Persistence happens when the user saves a preset (`STORE.PRESET_SAVE`), following the same convention as psybass. The 1.1.6 save-to-active-slot opcode is retired. A host saves to the active slot by reading `STORE.PRESET_ACTIVE` and sending `STORE.PRESET_SAVE` to that slot.

### Change notifications

Each SET emits a parameter-write notification on the notification endpoint whose offset/length identify the changed field inside the bulk wire structure (section 4). A second host UI can mirror subharm changes live and re-read the headroom (`FX.SUBHARM_HEADROOM`) after any of them. Solo is the exception: it has no wire offset, so it emits no notification and a second UI must poll `FX.SUBHARM_SOLO` if it wants to show it.

---

## 4. Bulk Wire Format (`STORE.ALL_PARAMS`)

Subharm appears in `WireBulkParams` from **wire format version 29** as the final section. **Wire format version 30** grows the section from 16 to 36 bytes by tail-appending, taking the total packet size from 5960 to **5980 bytes**.

`WireSubharmParams`, 36 bytes, at byte offset **5944** within `WireBulkParams`:

| Offset | Size | Type | Field |
|--------|------|------|-------|
| +0 | 1 | uint8 | enabled (0/1) |
| +1 | 1 | uint8 | reserved (write 0) |
| +2 | 2 | uint16 LE | output_mask |
| +4 | 4 | float LE | low_db |
| +8 | 4 | float LE | high_db |
| +12 | 4 | float LE | boost_db |
| +16 | 4 | float LE | top_db (V30+) |
| +20 | 4 | float LE | select_depth (V30+) |
| +24 | 4 | float LE | select_hold_ms (V30+) |
| +28 | 4 | float LE | ceiling_db (V30+) |
| +32 | 1 | uint8 | select_mode (V30+) |
| +33 | 1 | uint8 | link_pairs (V30+) |
| +34 | 2 | uint8[2] | reserved (write 0) |

On a bulk SET, all subharm fields are applied and coefficients recompute automatically. On a bulk GET, the section reflects live state including clamping. Two readings are not part of the wire structure. Read the headroom with `FX.SUBHARM_HEADROOM` and the sub meter with `FX.SUBHARM_METER`. `solo` is deliberately absent as well, so a bulk apply never changes it.

---

## 5. Persistence

- **Preset slots (flash):** subharm fields are stored per preset from `SLOT_DATA_VERSION` 36 (16 bytes tail-appended). Presets saved by older firmware (V21 to V35) load with subharm defaults (disabled, mask 0xFFFF, both bands 0 dB, boost 0 dB); no data is lost or misread. Preset save (`STORE.PRESET_SAVE`) captures the live state; preset load (`STORE.PRESET_LOAD`) restores it and recomputes coefficients.
- **Preset slots, V37:** the third band, the selectivity settings, the ceiling and the pair link are stored per preset from `SLOT_DATA_VERSION` 37 (a further 20 bytes tail-appended). V36 slots load the defaults for those fields (top -30 dB, mode 0, depth 100 %, hold 150 ms, ceiling 0 dB, link on). `solo` is not stored, and a preset load leaves it exactly as it was.
- **Factory reset (`STORE.FACTORY_RESET`):** restores the defaults above, and is the only operation that clears `solo`.
- **Startup:** the boot preset (or factory defaults) determines the state at power-on; the effect is fully initialized before audio starts.

---

## 6. App Integration Patterns

### Startup / reconnect sync

1. Read `STORE.ALL_PARAMS` and parse the subharm section at offset 5944 (verify `format_version` first), **or** issue the individual GETs (`FX.SUBHARM_ENABLE` to `FX.SUBHARM_LINK`, commands 0x20 to 0x2A).
2. Read the headroom (`FX.SUBHARM_HEADROOM`) and show it next to the effect.
3. Read solo (`FX.SUBHARM_SOLO`) separately; it is not in the bulk blob.

### Live control

- Sliders send SETs on change; the firmware clamps silently, so an app that enforces the documented ranges keeps identical state.
- After any SET, re-read `FX.SUBHARM_HEADROOM`. Offer a "free headroom" action that lowers the per-channel preamp (`ROUTE.PREAMP`) on the inputs feeding the masked outputs by the reported amount. Show the reading as a requirement, not a suggestion: the effect is amplitude-linear, so the number is exact.
- The headroom reading does not depend on the mask or the sample rate, only on enable, the two band levels and the boost.

### Typical UI

Enable; two sliders labelled "24-36 Hz" and "36-56 Hz" (-30 to +12 dB, floor shown as "Off"); an "LF Boost" slider (0 to +6 dB); per-output checkboxes building the mask; a headroom readout with an apply button.

### Suggested starting points

| Use case | low | high | boost |
|----------|-----|------|-------|
| Subwoofer feed, subtle weight | -6 | -6 | 0 |
| Club / large system | 0 | 0 | +3 |
| Thin recordings, add fundamental | 0 | -6 | +3 |
| Cinema LFE emphasis | +3 | -12 | 0 |

### Feature detection

Every firmware that speaks the paged protocol (1.1.7 and later) has every subharm command in this document. A host that gates features on the command table checks the GET bit of `FX.SUBHARM_ENABLE` in `SYS.PAGE_COMMANDS` for page 0x08, or the FX page revision in `SYS.PAGE_LIST`. Firmware 1.1.6 and older does not answer the paged protocol (`vendor_command_overhaul_plan.md` section 5.14).

---

## 7. Interactions and Edge Cases

- **Polyphony.** Two notes in different bands (kick at 55 Hz plus bass at 100 Hz, or 55 plus 130 Hz with the third band on) are divided independently and cleanly (spurious content below the subs at -21 dB in the offline model). Two notes inside one band confuse that band's divider; this is inherent to any octave divider and matches the dbx.
- **Selectivity limits.** The gate decides per band from time behaviour, so it cannot separate two sources that sound at the same instant in the same band. Percussive mode is robust: a bass note under a kick gets a short sub burst at each kick and nothing between. Sustained mode ducks a held note's sub for the hold time at every kick that lands in the same band, which is audible pumping on a four-on-the-floor bass line; use `select_depth` and a short hold to soften it. In sustained mode the first hold period of every note has no sub, so staccato bass lines get little.
- **Ceiling onset.** The limiter's gain follows the sub's peak with a 3 ms attack, so the first few milliseconds of a loud onset overshoot the ceiling (about 7 dB on a full-scale 60 Hz step in the model) before settling. In steady state the sub sits at the ceiling with no added distortion.
- **Third band and the 112 Hz edge.** Turning the third band on moves band 1's alignment allpass from 160 to 105 Hz and enables the band 0 / band 2 parity slaving; band 1's response is unchanged. The 160 Hz anti-alias lowpass lowers band 1's response at 112 Hz by about 0.9 dB compared with the original 48 kHz two-band kernel.
- **Sub lag.** The sub is computed at the low rate and interpolated back, so it lags the dry signal by one low-rate period (6 samples at 48 kHz) on every processed output alike. The dry signal itself is not delayed.
- **Solo.** Removes the dry signal and skips the bell on the masked outputs; unmasked outputs are unaffected. Runtime only: never persisted, never in the bulk wire format, cleared by a factory reset, untouched by a preset load. Hosts should drive it from a momentary control.
- **Crossover.** Subharm runs before the per-output crossover. A subwoofer output with a lowpass crossover keeps the sub; a satellite output with a highpass crossover loses it again, so mask satellites off to save CPU rather than relying on the crossover.
- **Psychoacoustic bass.** Independent and compatible; subharm runs first. Enabling both on one output is unusual (they pull in opposite directions) but legal.
- **Loudness, leveller, crossfeed.** Independent; loudness runs post-gain, the leveller and crossfeed earlier in the chain.
- **Test signals.** Outputs carrying a RAW signal-generator signal bypass subharm, as they bypass all per-output processing.
- **Muted or disabled outputs.** Skipped; state cleared so unmuting is transient-free. When an output re-enters processing and its S/PDIF pair partner is running, it takes the partner's divider state (bell aside) instead of starting from zero. Two dividers restarted independently on the same program land in opposite polarity half the time and never resynchronize, which would cancel the sub between the two speakers. For the same reason a linked pair mirrors its state into the odd output every block, so switching the link off continues both dividers in step.
- **Decimation phase.** One phase per packet, derived from an absolute sample counter on Core 0 and handed to Core 1, so the sub path has the same one-period lag on every output at every rate. A per-output phase would have let an output that re-entered processing sit up to D-1 samples away from its neighbours.
- **Sample rate changes.** Coefficients recompute automatically for 44.1/48/96 kHz.
- **Very quiet input.** The envelope-relative threshold keeps the divider working at any level. Below the noise floor the divider toggles on noise, but its output is that noise and is inaudible.
- **Sub-48 Hz program content.** The 48 Hz highpass keeps real sub-bass and DC out of the dividers, so a 30 Hz note does not produce a 15 Hz sub-sub.
- **RP2040 fixed point.** The sub path's input is clamped to +/-3.0, each band signal to +/-1.0 before its divider, each band's scaled sub to +/-2.5, and the sub sum to +/-2.0, with the boost ceiling at +6 dB, so no stored value can wrap `fast_mul_q28` past +/-8.0 on inputs up to +9.5 dBFS even with three bands at +12 dB. The dry path is not clamped. Inputs driven harder than that by preamp and matrix gain are in the same regime as the PEQ.
- **CPU cost.** Per processed output: one SVF, the interpolator and the bell at full rate, and the split, dividers, gates and ceiling at one sixth of the rate. About 12 multiplies per sample with the default two bands and roughly 16 with three bands, selectivity and ceiling on, against 25 for the original full-rate kernel. A linked pair costs one kernel plus one extra bell. Masked-off outputs and skipped bands, gate or bell cost nothing beyond a state clear.

---

## 8. Implementation Summary (firmware reference)

| Aspect | Detail |
|--------|--------|
| Module | `firmware/DSPi/subharm.h` / `subharm.c` |
| Kernel | One source, two number types: `sh_num_t` is float (RP2350) or Q28 int32 (RP2040) with `sh_mul` / `sh_half` / `sh_twice` / `sh_quarter` / `sh_abs` / `sh_band_limit` / `sh_ratio` helpers; RAM-resident (`DSP_TIME_CRITICAL`) out-of-line functions shared by both cores: `subharm_process_block` (full-rate loop), `subharm_low_rate_tick`, `sh_band_tick`, `sh_select_gain`, and the per-packet pass `subharm_process_outputs` |
| Rates | Full rate: LP2 160 Hz anti-alias, linear interpolator, bell. Low rate (D = round(fs/8000)): everything else. Decimation phase and interpolator state persist in the output state across blocks |
| Filters | TPT SVF throughout: LP2 160 Hz, HP2 48 Hz, split SVF 112 Hz (HP out = band 2), split SVF 72 Hz (LP out = band 0, HP out = band 1), post LP2 40 / 62 / 80 Hz, AP1 160 or 105 Hz on band 1 and 130 Hz on band 2, bell 70 Hz Q 0.9 |
| Divider | Peak follower (40 ms), arm below -env/4, flip at the next non-negative sample, conditional negate; flip period measured per band; band 2 takes band 0's parity when the periods match within 1/16 |
| Gate | Per band: slow follower (400 ms, 30 ms rise), attack = fast env above 2x slow, alive floor -60 dBFS, hold counter; percussive opens at once and closes over 100 ms, sustained opens over 30 ms and shuts at once on an attack; applied before the post lowpass |
| Ceiling | Peak follower on the sub sum (200 ms release), gain = ceiling / peak above the ceiling, 3 ms attack and 200 ms release on the gain |
| Coefficients | One global set, double-buffered, pointer-published (`current_subharm_coeffs`, NULL = off), rebuilt in the main loop on parameter/rate change |
| State | `subharm_output_state[NUM_OUTPUT_CHANNELS]`, 160 bytes per output, owned by the core that owns the output, reset whenever the output is skipped (ceiling gain idles at unity); a fresh output is seeded from a running pair partner; skipped bands, gate, ceiling and bell keep zeroed state (the bell also while soloed); a linked pair mirrors its state into the odd output, which keeps only its own bell |
| Pass | `subharm_process_outputs(coeffs, mask, flags, first, last, ...)` runs once per packet per core range before the per-output loop: eligibility per output, linked pairs when both outputs of an S/PDIF pair are eligible, singles otherwise, resets for the rest |
| Dual-core | Coefficient pointer + mask + flags (link, solo) + decimation phase snapshotted once per packet into `Core1EqWork` (`subharm_coeffs` / `subharm_mask` / `subharm_flags` / `subharm_phase`) |
| Headroom | `subharm_headroom_db()`: pure function of the config, analog-prototype magnitudes on a 37-point eighth-octave grid (16 to 362 Hz); with a ceiling, bisection on the input level for the smallest safe headroom; computed on each `FX.SUBHARM_HEADROOM` GET |
| Latency | Dry path untouched; the sub lags by one low-rate period (6 samples at 48 kHz) on every processed output alike; inter-slot alignment preserved by construction |
| Versions | Vendor commands `FX.SUBHARM_*` (page 0x08, 0x20-0x2B, 0x2E, 0x2F); wire format V30; preset slot V37; control surfaces caps v16 (nouns 57-67; band levels to +12 dB) |
| RAM | about 1.2 KB (RP2040) / 1.9 KB (RP2350) of state and coefficient buffers, plus about 3 KB of RAM-resident code |
| Status | Implemented, verified against an offline model of the kernel (the C float kernel matches the model to 6e-7, output is bit-identical across block sizes, Q28 error about -102 dBFS); hardware listening test pending |
