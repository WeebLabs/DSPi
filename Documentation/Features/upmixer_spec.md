# DSPi Stereo Upmixer Specification

*Version 2.0, 2026-11-22. Status: upmixer v2 (pair engine, two profiles, module routing, back pair) is implemented through phase 5 of `upmixer_v2_plan.md`, and the hardware bench pass (phase 6) is pending. RP2350 only.*

This document contains everything an application developer needs to integrate the DSPi upmixer: the concept, profiles and activation, the routing model, every command byte layout, parameter semantics with ranges and defaults, status telemetry, persistence, and recommended UX patterns. The design rationale lives in `upmixer_v2_spec.md`. Where the two differ, this document describes the firmware as built.

---

## 1. What it does

The upmixer takes a pair of input channels, the **source pair** (source A and source B), and splits it into seven generated channels.

| g | Name | Content |
|---|---|---|
| 0 | A' | Source A after centre removal. A bit-exact copy of source A when the centre mode is OFF or the centre width is 100 |
| 1 | B' | Source B after centre removal |
| 2 | C | Extracted centre: the correlated content of the pair |
| 3 | SA | Side ambience, A side |
| 4 | SB | Side ambience, B side |
| 5 | BA | Back ambience, A side. Arrives after the sides (section 1.2) |
| 6 | BB | Back ambience, B side |

The names are role-neutral because the source pair can be front L/R, side Ls/Rs or any other two inputs. Source A plays the role of L and source B the role of R in every engine equation. With the default sources (inputs 0 and 1) and a stereo input, A' and B' are the processed fronts, C is a centre speaker feed, SA/SB feed the side surrounds and BA/BB the back surrounds. Apps should label the channels per use case ("Front L", "Back centre", and so on).

Each generated channel is routed by the upmixer itself to any set of outputs, with a trim and a polarity flag (section 3). The matrix does not grow and no matrix rows are borrowed.

All generated channels are full range or band-limited ambience. The upmixer has **no LFE output and no crossover**. Its layouts are 3.0, 5.0 and 7.0. A ".1" system is an x.0 upmix plus a sub fed from the matrix, with ordinary bass management on the outputs (section 3.5).

### 1.1 Engines

Two independent engines, each with a quality/CPU mode, run on the source pair.

| Engine | Modes | Algorithm |
|---|---|---|
| Centre | `PASSIVE` (0), `ADAPTIVE` (1), `OFF` (2) | Passive: fixed C = 0.7071(A+B). Adaptive: running normalized cross-correlation plus A/B balance steer the centre gain through a threshold gate with attack/release ballistics. Off: extraction gain forced to zero, so C is silent and A'/B' are bit-exact copies of the sources. |
| Surround | `OFF` (0), `PASSIVE` (1), `ADAPTIVE` (2) | Passive: difference feed (A-B). Adaptive: prediction-residual feed (each surround keeps what its own side cannot predict from the other), a balance gate that closes the surround on the dominant side, front/back dominance and a transient duck, all steered from the band-limited signal ahead of the surround delay (section 12). |

The extracted centre is (configurably) removed from A' and B', so a real centre speaker and the A'/B' phantom image do not comb-filter. That removal is the only thing the upmixer does to the pair. The surround engine reads the sources but never changes A' or B'. There are two ways to keep the mains exactly as recorded, and both leave the surrounds fully operational.

- **Centre mode `OFF`**: no centre at all, A'/B' bit-exact. The surrounds-only setup.
- **`center_width_pct` = 100**: the centre is still extracted and routable, A'/B' bit-exact, phantom centre kept. Expect combing if a real centre speaker also plays.

Both surround modes feed a built-in conditioning chain for each ambience channel. It consists of the **ambience band** (a 2nd-order Butterworth high-pass and low-pass, 12 dB/oct each, default 300 Hz to 7 kHz), a Haas delay, and a Schroeder allpass decorrelator. The ambience band shapes the surround content. It is not an LFE crossover. Its high-pass can be set as low as 20 Hz, and then the side and back outputs need their own output high-passes for bass management again.

The centre enum orders its `OFF` last, unlike the surround enum. `OFF` was appended as value 2 so that existing hosts and presets kept their meaning for 0 and 1. Hosts may present the modes in any order. Only the wire values are fixed.

### 1.2 Timing

- **A', B' and C** have zero added latency (pure gain steering).
- **SA and SB** arrive after the surround delay plus 10 ms (the main tap of their 10 ms allpass).
- **BA and BB** read the same Haas delay line at a deeper tap and pass through their own allpass of about 5.23 ms. The firmware places that tap so the backs arrive `back_offset_ms` (0 to 10 ms, default 2) after the sides. The tap is held inside the 1024-sample delay line (about 21.3 ms at 48 kHz), so at the default 12 ms surround delay the usable offset tops out near 4.5 ms. More back delay comes from the per-output delay on the back outputs, which carry only ambience.

SA and SB share one tap, and BA and BB share one tap, so a pair routed to a pair of outputs has matched timing. The upmixer adds no latency to any output path, and every output advances by the same sample count with the same sample index, so inter-slot alignment is never disturbed.

### 1.3 Back pair

The back pair uses the same steering as the sides (same residual weights and per-sample gains), so no second engine is needed for 7.0. Its decorrelator uses the side gain with mirrored signs, so a same-side SA and BA differ from their first tap. The backs have no separate trim, band or delay settings. Route trim covers level and the ambience band is shared with the sides.

The back pair runs only while BA or BB is routed to an enabled output (or is still fading out) and the surround mode is not OFF. When routing begins, its allpass clears and, in ADAPTIVE mode, it fades in over 10 ms from the moment its delayed audio first emerges.

---

## 2. Profiles and activation

### 2.1 Two profiles

The upmixer holds two **config profiles**, 0 and 1. Each is a complete configuration: every engine parameter, the source pair, the input counts it serves (`active_counts`), the back offset and the routes of all seven generated channels. One preset can therefore upmix stereo music one way and extend 6-channel film audio another. There is one engine, so at most one profile runs at a time.

`active_counts` is a bit mask over the active input channel count.

| Bit | Input channel count |
|---|---|
| 0 (0x01) | 2 |
| 1 (0x02) | 4 |
| 2 (0x04) | 6 |
| 3 (0x08) | 8 |

The mask is required, not optional. A stereo-to-5.0 profile reads inputs 0 and 1, which exist in every mode, so without the mask it would also run on a real 6-channel stream and replace real centre and surround content.

### 2.2 Which profile runs

The **winning profile** is the lowest-numbered profile that is enabled and whose `active_counts` contains the active input channel count, while the sample rate is 48 kHz or below. The engine runs on the winning profile when both of these also hold.

1. Both of its sources are present (each source index is below the active input count).
2. At least one of its routes targets an enabled output.

Otherwise the upmixer parks, and every output carries the plain matrix mix. The checks run in that order. A winning profile whose source is absent, or which routes nothing to an enabled output, does not hand over to the other profile. Overlapping `active_counts` are legal and never refused. With an overlap, profile 0 wins and profile 1 is inert for that count. An app should warn about overlaps.

`FX.UPMIX_STATUS` reports whether the engine is running, which profile it holds, and why it is parked (section 5).

Defaults for both profiles are sources 0 and 1, `active_counts` 0x01 (stereo only), disabled, no routes.

### 2.3 Starting, stopping and switching

- **Start.** The engine starts from a reset state and fades in over 10 ms. The ambience channels start from silence and appear once the delayed audio emerges.
- **Disable, losing the win, last route removed.** These fade out over 10 ms on the last coefficient set, then the engine parks. Disabling is no longer a hard stop.
- **Profile change or source edit.** When a live edit (enable, `active_counts`, or the running profile's `src_a`/`src_b`) changes what should run, the change is sequential, never a crossfade. The old profile fades out, the state resets, and the new configuration fades in.
- **Forced parks are hard.** Input source switches, input channel count changes and sample rate changes pass through the pipeline reset, which parks the engine at once with every fade at zero. It starts again with a fade-in. Only a pipeline reset can click, and it already mutes the outputs.
- **No audio.** While no producer feeds the pipeline, the main loop parks the engine hard. The next start fades in cleanly.

Parameter edits to the running profile apply within a few ms without restarting it (section 4).

---

## 3. Routing

### 3.1 Routes

Each generated channel of each profile has one route.

| Field | Type | Range | Default | Meaning |
|---|---|---|---|---|
| `out_mask` | u16 | bits 0 to 8 (RP2350 has 9 outputs) | 0 | Bit k routes this channel to output k |
| `trim_q1` | i8 | -80..+12 (-40.0 to +6.0 dB) | 0 | Gain in 0.5 dB steps, clamped |
| `flags` | u8 | bit 0 | 0 | Bit 0 inverts polarity. The other bits are reserved and must be 0 |

A generated channel may go to several outputs, and several generated channels may land on one output, where they are summed. There is no per-destination gain. Level differences between outputs belong in the per-output gain.

Routing a channel to a disabled output is accepted. It is silent until that output is enabled, and it does not count as a route for activation. An app should warn when a route targets a disabled output.

### 3.2 Claims: replace the source pair, keep everything else

Claims act on the input side of the matrix. On any output that at least one generated channel of the running profile is routed to, the profile's **source-pair rows** are dropped from that output's matrix sum, and every other matrix row still sums there. Such an output is **claimed**. Examples follow.

- Stereo input, Front L output fed by row 0. Route A' to it. The output now carries A' instead of L.
- Stereo input, a centre output with no matrix feed. Route C to it. The output carries C.
- 6-channel input, profile on inputs 4 and 5. The side outputs fed by rows 4 and 5 now carry A'/B'. Any other rows routed to those outputs remain.
- A sub output fed by rows 0 and 1 that has no route keeps raw L+R. That is the original bass, independent of centre steering, so it is the correct sub feed.

This is "replace", not "add". Adding A' to a matrix feed that still carried L would put the extracted centre back and double the level. Both source rows are dropped on a claimed output whichever generated channels are routed there. Routing only C to an output fed by row 0 replaces its L with C.

The matrix under a claim is the **parked mix**. When the engine parks or is disabled, claimed outputs return to their plain matrix feed. Apps and templates should therefore leave the source-row crosspoints in place on the fronts. An output with no matrix feed, such as a dedicated centre, is silent while parked unless the matrix also feeds it. Apps should mark matrix cells whose row is claimed on that output ("taken by Upmixer") rather than hide them.

A silent generated channel still claims. C when the centre mode is OFF, and SA/SB/BA/BB when the surround mode is OFF, keep their outputs claimed, so a mode switch never makes matrix rows pop back in.

### 3.3 Ramps

- Every route contribution slews toward its target at a fixed rate of full scale per 10 ms and lands exactly on it. Trim edits, mask edits, polarity changes, start and stop all use this slew, and a second edit during a slew simply retargets it. A large trim step takes proportionally longer (a +6 dB step about 20 ms).
- Each output's claim weight slews the same way. The dropped source rows fade out as the claim fades in, and back in as it fades out.
- In steady state, an output claimed by a single generated channel at unity trim with no other matrix rows carries that channel bit-exact.
- A disabled output gets no contributions. When it is enabled again, its routes fade in.

### 3.4 Level conventions

- C uses the constant-power convention. Full extraction of a 0 dBFS mono signal yields C at +3 dBFS and silence on A'/B'. **C can exceed 0 dBFS by up to 3 dB.** Watch its peak and clip bit in the status (section 5), and set the C trim to -3 dB if the downstream chain has no headroom.
- Passive surround peaks at 0.7071 x (A-B). Adaptive surround gains reach at most 0.75 and the residual feed `A' - w*B'` has `w` in [0.5623, 4], so full-scale anti-phase content reaches about +1.4 dBFS (0.75 x 1.56). A transient where the weight still sits high from correlated material while the inputs swing apart can momentarily exceed that, in theory up to 0.75 x 5. BA/BB follow SA/SB.
- Ambience level relative to the fronts is simply the route trim.

### 3.5 Example setups

All setups below use profile 0 with sources 0 and 1 and `active_counts` 0x01, unless stated.

- **3.0 (LCR).** Surround mode OFF. Route A' to Front L, B' to Front R, C to the centre output (trim -3 dB to taste). Keep rows 0 and 1 on Front L/R in the matrix as the parked mix. Set `center_width_pct` to taste (25, the default, keeps a little centre energy in A'/B').
- **5.0.** As 3.0, plus surround mode ADAPTIVE and SA to side L, SB to side R, trims around -3 to -6 dB. The built-in delay, ambience band and decorrelation need no further setup. Per-output delay can still be added on the side outputs for distance compensation.
- **7.0.** As 5.0, plus BA to back L and BB to back R. Use `back_offset_ms` and the back outputs' per-output delay for extra back distance.
- **x.0 plus a matrix sub feed (3.1, 5.1, 7.1 systems).** Leave the sub output with no upmixer route, so it keeps the raw L+R matrix feed of rows 0 and 1. Put a low-pass on the sub output and matching high-passes on the front and centre outputs, as ordinary bass management with the output crossovers. With the default 300 Hz ambience band the side and back outputs need no high-pass. If the ambience band's high-pass is lowered toward 20 Hz, give them one too.
- **Surrounds only (fronts exactly as recorded).** Centre mode OFF, surround mode PASSIVE or ADAPTIVE, SA/SB (and optionally BA/BB) to the rear outputs. The fronts need no route. Their matrix rows 0 and 1 reach them unchanged. Strength, width, threshold, attack, release, detector HPF and presence are all inert here.
- **6-channel input: backs from the side surrounds.** Profile 1, sources = the input channels carrying Ls and Rs, `active_counts` 0x0C (6 and 8 channels). Route A'/B' to the side outputs and SA/SB to the back outputs, or C to both back outputs for a back centre. The input's LFE channel is a real input row and reaches the sub through the matrix as usual.

The DSPi Console offers these and other use cases as templates (for example centre fill, dialogue lift, vocal reduction). The firmware has no notion of templates. A typical "stereo plus 6-channel" preset uses profile 0 for 2-channel input and profile 1 for 6- and 8-channel input.

---

## 4. Parameters

Every parameter exists once per profile. All are persisted in presets and carried in bulk params. Floats are IEEE 754 single precision little-endian. Float fields are stored as sent and clamped only when the coefficients are computed, so a GET returns the written value. A NaN or infinite float is refused with `VC_ERR_VALUE` and stores nothing. The Command column names the per-parameter command (page 0x08, section 6.6).

| # | Field | Range | Default | Meaning | Command |
|---|---|---|---|---|---|
| 0 | enabled | 0/1 | 0 | Profile enable | `FX.UPMIX_ENABLE` (0x90) |
| 1 | center_mode | 0-2 | 1 (ADAPTIVE) | Centre engine mode. 2 = OFF: C silent, A'/B' bit-exact, surrounds unaffected. Values above 2 fall back to ADAPTIVE (not up to OFF, so a corrupt byte cannot silently disable the centre) | `FX.UPMIX_CENTER_MODE` (0x91) |
| 2 | surround_mode | 0-2 | 2 (ADAPTIVE) | Surround engine mode. Values above 2 become 2 | `FX.UPMIX_SURROUND_MODE` (0x92) |
| 3 | strength_pct | 0-100 | 100 | Centre extraction strength. Scales both C and the removal from A'/B'. **Both centre modes**. In passive mode it is the fixed centre gain | `FX.UPMIX_STRENGTH` (0x93) |
| 4 | center_width_pct | 0-100 | 25 | How much extracted centre stays in A'/B'. 0 = full removal (discrete centre), 100 = A'/B' untouched (C is additive). **Both centre modes** | `FX.UPMIX_CENTER_WIDTH` (0x94) |
| 5 | corr_threshold_pct | 0-95 | 30 | Correlation gate. Centre presence below this extracts nothing. Above it, extraction scales smoothly to full. **Adaptive centre only** | `FX.UPMIX_THRESHOLD` (0x95) |
| 6 | attack_ms | 1-500 | 10 | Centre gain rise time. **Adaptive centre only** (in passive mode it still smooths strength and mode changes) | `FX.UPMIX_ATTACK` (0x96) |
| 7 | release_ms | 5-2000 | 100 | Centre gain fall time. **Adaptive centre only** (same passive-mode caveat) | `FX.UPMIX_RELEASE` (0x97) |
| 8 | detector_hpf_hz | 20-1000 | 200 | Detector bass cut. Content below this corner is ignored by the detector. The audio itself is not filtered. **Adaptive centre only** | `FX.UPMIX_DET_HPF` (0x98) |
| 9 | surround_delay_ms | 0-20 | 12 | Haas delay of the ambience channels. Rule of thumb about 1 ms per foot of listener distance | `FX.UPMIX_SUR_DELAY` (0x99) |
| 10 | surround_hpf_hz | 20-2000 | 300 | Ambience band high-pass (12 dB/oct). Not an LFE crossover | `FX.UPMIX_SUR_HPF` (0x9A) |
| 11 | surround_lpf_hz | 1000-20000 | 7000 | Ambience band low-pass (12 dB/oct). Also held below 0.45 x the sample rate | `FX.UPMIX_SUR_LPF` (0x9B) |
| 12 | decorr_pct | 0-100 | 90 | Decorrelator amount. Allpass gain G = 0.5 x pct/100. 0 disables decorrelation | `FX.UPMIX_DECORR` (0x9C) |
| 13 | presence_db | -12..+12 | 0 | Centre presence bell gain at a fixed 3 kHz, Q 0.6. Negative moves voices back, positive brings them forward. **Both centre modes**. In the config packet, bulk and presets it travels in 0.5 dB steps | `FX.UPMIX_PRESENCE` (0x9D) |
| 14 | src_a | 0-7 | 0 | Source A input channel. Refused above 7 on a vendor SET | `FX.UPMIX_SOURCE_A` (0x9E) |
| 15 | src_b | 0-7 | 1 | Source B input channel. `src_a == src_b` is legal and degenerate (fully correlated) | `FX.UPMIX_SOURCE_B` (0x9F) |
| 16 | active_counts | 0x0-0xF | 0x01 | Input counts this profile serves (section 2.1). Bits above 3 are refused on a vendor SET | `FX.UPMIX_ACTIVE_COUNTS` (0xA0) |
| 17 | back_offset_ms | 0-10 | 2 | Back pair arrival after the sides (section 1.2) | `FX.UPMIX_BACK_OFFSET` (0xA1) |

Routes are not numbered parameters. They have their own command, `FX.UPMIX_ROUTE` (section 6.4).

**Per-mode applicability (UI guidance).** When the centre mode is `OFF`, grey out the whole centre group (3, 4, 5, 6, 7, 8, 13). None of it has any effect. When the centre mode is `PASSIVE`, keep Strength (3), Width (4) and Presence (13) accessible, because they are the passive mode's working controls. Grey out only Threshold (5), Attack (6), Release (7) and Detector HPF (8), which drive the adaptive steering. The ambience parameters (9 to 12) apply in both surround modes and should be greyed only when the surround mode is `OFF`. Back offset (17) matters only while BA or BB is routed and the surround mode is not `OFF`. Sources and counts (14 to 16) always apply.

**Fixed internals** are not configurable. The correlation estimator time constant is 100 ms, the front/back dominance smoother 40 ms and the residual-weight estimators 40 ms. The duck envelopes are 2 ms and 60 ms with a 60 ms release. The side decorrelator is 10 ms and the back decorrelator about 5.23 ms. The back tap sits 4.8 ms plus `back_offset_ms` beyond the side tap. The activation fade is 10 ms and the routing slew is full scale per 10 ms. The presence bell is fixed at 3 kHz, Q 0.6 (TPT SVF bell, boost/cut symmetric). The adaptive surround detector uses the ambience band, and its lookahead equals the surround delay.

**Live changes.** Parameter changes take effect within a few ms (main-loop coefficient recompute, double-buffered publish, no audio interruption). Mode and parameter changes while running are click-safe. Gains ramp per sample, and a surround OFF to ON transition clears the ambience state and fades the ambience in. Changing between PASSIVE and ADAPTIVE surround while running clears the ambience state and restarts it from silence after the Haas delay, because the delay line holds different signals in the two modes. Switching the centre engine to OFF ramps the extraction and removal to zero across one packet (about 1 ms at 48 kHz) and then holds exact zero, so A'/B' become bit-exact again.

---

## 5. Metering and status

Generated channels are not input channels, so `MEAS.LEVELS` no longer reports them. Its input peaks and clip flags for channels 2 to 4 are always real inputs 3 to 5. The levels of the generated channels are in `FX.UPMIX_STATUS`. For a spectrum view of generated content, use the RTA output tap of the output it is routed to.

### 5.1 UpmixStatus (32 bytes)

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | active | 1 while the engine is running. Also 1 while it is parked only for want of audio, when it would start with the next packet. 0 while fading out |
| 1 | u8 | parked_reason | 0 = running, 1 = no profile enabled, 2 = no enabled profile serves the active input count, 3 = sample rate above 48 kHz, 4 = a source of the winning profile is absent, 5 = fading out, 6 = the winning profile routes nothing to an enabled output |
| 2 | i16 | corr_q14 | Smoothed source-pair correlation, Q14: value/16384 in [-1, +1]. Zero in passive centre mode |
| 4 | i16 | balance_q14 | Smoothed level balance, Q14: 0 = centred, 16384 = fully one-sided |
| 6 | u16 | center_gain_q15 | Live centre extraction gain, value/32767 in [0, 1] (includes strength, gate, ballistics and fade) |
| 8 | u16 | ls_gain_q15 | Live SA steering gain, Q15. Adaptive: side gate x front/back term x duck x fade, excluding the residual weight. Passive: the fade only |
| 10 | u16 | rs_gain_q15 | Live SB steering gain, Q15, same meaning |
| 12 | u8[4] | reserved | |
| 16 | u8 | active_profile | Profile running, fading out or about to run (0 or 1). 0xFF for none |
| 17 | u8 | gen_clip | Bit g is set while generated channel g passed 0 dBFS, before its trim, within the last second. No read clears it, so several pollers see the same bits |
| 18 | u16[7] | gen_peak | Peak of each generated channel over the last packet, before its trim, 32767 = 0 dBFS, saturating. Order A', B', C, SA, SB, BA, BB |

When the engine is not running, `parked_reason` gives the first reason that applies in the order 5, 1, 3, 4, 6, 2. Reason 2 is v1's "input not stereo" for a v1-style profile. The engine telemetry fields (2 to 10) and the peaks read zero while parked. The live fields describe the profile the engine holds. A host that reads only the first 16 bytes gets the original v1 fields.

Poll at 5 to 20 Hz for a responsive display (correlation meter, extraction gain bar, ambience activity, generated-channel meters). Values are telemetry snapshots and need no locking.

---

## 6. Vendor commands

The upmixer commands live in the `FX` page (0x08) of the paged protocol (see `../commands.md` section 13.5), on USB EP0 vendor requests and on the UART and I2C control transports, which share one dispatcher. Each command is one number for both directions. Multi-byte values are little-endian.

On **RP2040** the feature does not exist. None of these commands is in the RP2040 command table, so every request answers `VC_ERR_UNKNOWN` (a STALL on USB, the status byte on UART and I2C).

**Feature detection.** The upmixer exists when `SYS.PAGE_COMMANDS` for page 0x08 has the GET bit of `FX.UPMIX_CONFIG`. The v2 interface described here exists when it also has the GET bit of `FX.UPMIX_CONFIG_EXT` (0x83), and `SYS.PAGE_LIST` reports FX revision 2 or later.

### 6.1 Command table

| Command | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|
| `FX.UPMIX_CONFIG` | 0x80 | GET, SET | profile (0, 1) | GET 44, SET 44 | Config packet bytes 0 to 43 (6.2) |
| `FX.UPMIX_STATUS` | 0x81 | GET | none (0) | GET 32 | `UpmixStatus` (5.1) |
| `FX.UPMIX_ROUTE` | 0x82 | GET, SET | (profile << 8) \| g | GET 4, SET 4 | `UpmixRoute` of one generated channel (6.4) |
| `FX.UPMIX_CONFIG_EXT` | 0x83 | GET, SET | profile (0, 1) | GET 36, SET 36 | Config packet bytes 44 to 79 (6.3) |
| `FX.UPMIX_ENABLE` to `FX.UPMIX_PRESENCE` | 0x90 to 0x9D | GET, SET | profile (0, 1) | GET 4, SET 4 | Parameters 0 to 13, float32 (6.5) |
| `FX.UPMIX_SOURCE_A` | 0x9E | GET, SET | profile (0, 1) | GET 4, SET 4 | Parameter 14, float32 |
| `FX.UPMIX_SOURCE_B` | 0x9F | GET, SET | profile (0, 1) | GET 4, SET 4 | Parameter 15, float32 |
| `FX.UPMIX_ACTIVE_COUNTS` | 0xA0 | GET, SET | profile (0, 1) | GET 4, SET 4 | Parameter 16, float32 holding the mask |
| `FX.UPMIX_BACK_OFFSET` | 0xA1 | GET, SET | profile (0, 1) | GET 4, SET 4 | Parameter 17, float32 ms |

The profile-addressed commands were address-less before FX revision 2. Address 0 keeps its old meaning, so a host that always sends `wValue` 0 still reads and writes profile 0. A profile of 2 or more is `VC_ERR_ADDRESS`. `FX.UPMIX_STATUS` takes `wValue` 0 because there is one engine.

### 6.2 UpmixConfigPacket (80 bytes, sent in two parts)

A profile's configuration is one 80-byte image. A non-bulk SET is at most 64 bytes, and every GET must be a valid SET, so the image travels in two commands. `FX.UPMIX_CONFIG` carries bytes 0 to 43 both ways and `FX.UPMIX_CONFIG_EXT` carries bytes 44 to 79 both ways. Each SET leaves the other part unchanged. Bulk parameters and presets carry the whole image.

Bytes 0 to 43, `FX.UPMIX_CONFIG`:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | enabled (0/1) |
| 1 | u8 | center_mode (0 = PASSIVE, 1 = ADAPTIVE, 2 = OFF) |
| 2 | u8 | surround_mode (0 = OFF, 1 = PASSIVE, 2 = ADAPTIVE) |
| 3 | i8 | presence_q1: presence bell gain in 0.5 dB steps (dB x 2, -24..+24; 0 = flat) |
| 4 | f32 | strength_pct |
| 8 | f32 | center_width_pct |
| 12 | f32 | corr_threshold_pct |
| 16 | f32 | attack_ms |
| 20 | f32 | release_ms |
| 24 | f32 | detector_hpf_hz |
| 28 | f32 | surround_delay_ms |
| 32 | f32 | surround_hpf_hz |
| 36 | f32 | surround_lpf_hz |
| 40 | f32 | decorr_pct |

A SET of any other length is `VC_ERR_LENGTH`. A NaN or infinite float field is `VC_ERR_VALUE` and nothing is stored. Mode bytes out of range are sanitized as in section 4, and `enabled` is a non-zero test.

### 6.3 Bytes 44 to 79, `FX.UPMIX_CONFIG_EXT`

Packet offsets are given. On the wire the EXT payload starts at packet offset 44, so subtract 44 for the payload offset.

| Packet offset | EXT offset | Type | Field |
|---|---|---|---|
| 44 | 0 | u8 | src_a (input channel, 0 to 7) |
| 45 | 1 | u8 | src_b (input channel, 0 to 7) |
| 46 | 2 | u8 | active_counts (bits 0 to 3) |
| 47 | 3 | u8 | reserved, 0 |
| 48 | 4 | f32 | back_offset_ms |
| 52 | 8 | 7 x `UpmixRoute` | routes for g = 0 to 6 (A', B', C, SA, SB, BA, BB), 4 bytes each (6.4) |

A SET of any other length is `VC_ERR_LENGTH`. A source above 7, an `active_counts` bit above 3, a non-finite `back_offset_ms`, or any route that `FX.UPMIX_ROUTE` would refuse makes the whole SET `VC_ERR_VALUE`, and nothing is stored. Route trims are clamped.

### 6.4 UpmixRoute (4 bytes), `FX.UPMIX_ROUTE`

`wValue` = profile x 256 + generated channel g (0 A', 1 B', 2 C, 3 SA, 4 SB, 5 BA, 6 BB). A profile above 1 or a channel above 6 is `VC_ERR_ADDRESS`.

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | out_mask, bit k = output k. A bit above bit 8 (the last output on RP2350) is `VC_ERR_VALUE` |
| 2 | i8 | trim_q1, dB x 2, clamped to -80 (-40 dB) to +12 (+6 dB) |
| 3 | u8 | flags: bit 0 inverts polarity. Any other bit is `VC_ERR_VALUE`, so a later host can detect firmware that lacks a new flag |

The GET returns the stored route, with the trim as clamped. Use `FX.UPMIX_ROUTE` for live routing edits and `FX.UPMIX_CONFIG_EXT` to write all seven routes of a profile at once.

### 6.5 Per-parameter commands

Each of the 18 parameters has its own command, in parameter order (command = 0x90 + parameter number), addressed by profile. Everything travels as a 4-byte float, including enable, modes, sources and the count mask. Enable is a non-zero test. Modes, sources and the count mask are rounded to the nearest integer. A source outside 0 to 7 or a count mask outside 0 to 15 is `VC_ERR_VALUE`. A SET payload other than 4 bytes is `VC_ERR_LENGTH`, and a NaN or infinite value is `VC_ERR_VALUE`.

Prefer the per-parameter SETs and `FX.UPMIX_ROUTE` for live controls, because they carry no read-modify-write race against other controllers. Use `FX.UPMIX_CONFIG` plus `FX.UPMIX_CONFIG_EXT` to apply a whole profile.

### 6.6 Suggested error handling

- A STALL on RP2350 is a refusal with its reason in `SYS.LAST_RESULT`. `VC_ERR_LENGTH` (wrong length), `VC_ERR_VALUE` (non-finite float, out-of-range source, count, mask or flag) and `VC_ERR_ADDRESS` (profile above 1, channel above 6, non-zero `wValue` on the status) all indicate a client bug.
- `VC_ERR_UNKNOWN` on a device you have identified as RP2040 is expected. Hide the feature.

### 6.7 Change notifications

Every accepted upmixer SET emits a `NOTIFY_EVT_PARAM_CHANGED` event on the notification endpoint for each field of that profile whose stored wire value actually changed. The event's offset is the field's position in `WireBulkParams` (section 7), its size is the field's size, and its payload is the new wire value. A SET that changes nothing emits nothing. Edits to the profile that is not running notify too.

A per-parameter or route SET normally emits one event. A `FX.UPMIX_CONFIG` SET emits up to 14 and a `FX.UPMIX_CONFIG_EXT` SET up to 11. A route is one 4-byte event at the route's offset. This covers every route into these commands: USB, the UART and I2C control transports, and the Control Surfaces nouns. The event's source byte tells them apart (`PARAM_SRC_HOST_SET`, `_UART`, `_I2C` or `_GPIO`). Preset load, factory reset and bulk SET emit one bulk-invalidation event instead of per-field events.

| Packet offset | Size | Field | Profile 0 wire offset | Profile 1 wire offset |
|---|---|---|---|---|
| 0 | 1 | enabled | 5900 | 6172 |
| 1 | 1 | center_mode | 5901 | 6173 |
| 2 | 1 | surround_mode | 5902 | 6174 |
| 3 | 1 | presence_q1 (int8, dB x 2) | 5903 | 6175 |
| 4 | 4 | strength_pct | 5904 | 6176 |
| 8 | 4 | center_width_pct | 5908 | 6180 |
| 12 | 4 | corr_threshold_pct | 5912 | 6184 |
| 16 | 4 | attack_ms | 5916 | 6188 |
| 20 | 4 | release_ms | 5920 | 6192 |
| 24 | 4 | detector_hpf_hz | 5924 | 6196 |
| 28 | 4 | surround_delay_ms | 5928 | 6200 |
| 32 | 4 | surround_hpf_hz | 5932 | 6204 |
| 36 | 4 | surround_lpf_hz | 5936 | 6208 |
| 40 | 4 | decorr_pct | 5940 | 6212 |
| 44 | 1 | src_a | 6136 | 6216 |
| 45 | 1 | src_b | 6137 | 6217 |
| 46 | 1 | active_counts | 6138 | 6218 |
| 48 | 4 | back_offset_ms | 6140 | 6220 |
| 52 + 4g | 4 | route of channel g | 6144 + 4g | 6224 + 4g |

Presence is the one field whose notified value can differ from the `FX.UPMIX_PRESENCE` GET. Its SET stores the float dB exactly, but the wire byte carries it in 0.5 dB steps. A SET of 3.3 dB therefore notifies `presence_q1` = 7 (3.5 dB), and a further SET that rounds to the same step emits nothing. Float fields are notified as stored, before the compute-time clamp, which matches what the config GETs and bulk GET return.

### 6.8 Control Surfaces

Six front-panel nouns dispatch to the matching per-parameter command, so physical knobs, buttons and IR commands can drive the upmixer. They are `UPMIX` (35, enable), `UPMIX_CENTER_MODE` (36), `UPMIX_SURROUND_MODE` (37), `UPMIX_STRENGTH` (38), `UPMIX_WIDTH` (39) and `UPMIX_PRESENCE` (40). They always edit **profile 0**, whatever the input, so one binding stays deterministic. All six are RP2350-only (empty action mask on RP2040). The front-panel mode labels call the PASSIVE engine modes "Sinner" and the ADAPTIVE engine modes "Logician". The wire values are identical. See `control_surfaces_spec.md` sections 4.3 and 5.

---

## 7. Bulk parameters (wire V35)

`STORE.ALL_PARAMS` (chunked `STORE.ALL_PARAMS_CHUNK`) carries both profiles in two sections of `WireBulkParams`.

| Wire offset | Size | Section | Content |
|---|---|---|---|
| 5900 | 44 | `upmix` (section 22, V25+) | Profile 0, packet bytes 0 to 43 |
| 6136 | 36 | `upmix2.p0_ext` (section 26, V35+) | Profile 0, packet bytes 44 to 79 |
| 6172 | 44 | `upmix2.p1` | Profile 1, packet bytes 0 to 43 |
| 6216 | 36 | `upmix2.p1_ext` | Profile 1, packet bytes 44 to 79 |

The total at `WIRE_FORMAT_VERSION` 35 is **6252 bytes**. Profile 0's first 44 bytes have not moved since V25.

- On RP2040 both sections are present but zero on GET and ignored on SET (platform-independent wire format).
- Version discipline is strict. The header's format_version must match exactly, so a client built for an older version is refused. Clients must send 35.
- A bulk SET applies both profiles like a config SET, with deferred coefficient recompute and no audio glitch. It sanitizes rather than refuses. A source above 7 falls back to the default (0 for A, 1 for B), count bits above 3 and route mask bits above the last output are dropped, reserved flag bits are cleared, trims are clamped, and mode bytes are sanitized as in section 4.

---

## 8. Persistence

- **Preset slots.** The upmixer layout is final at `SLOT_DATA_VERSION` 42. Profile 0's first 44 bytes stay where V33 put them (presence byte V34). V42 tail-appends 116 bytes with the same layout as the bulk section 26 (profile 0's bytes 44 to 79, then profile 1 whole). Preset save and load round-trip every field of both profiles.
- **Factory reset.** Both profiles at defaults: disabled, sources 0 and 1, `active_counts` 0x01, back offset 2 ms, no routes.
- **RP2040 slots.** The fields are stored as zeros for layout uniformity and never applied.
- After any preset load or bulk SET, re-read both profiles (`FX.UPMIX_CONFIG` and `FX.UPMIX_CONFIG_EXT` for profiles 0 and 1) rather than trusting a cached copy. The bulk-invalidation notification is the refresh trigger.

### 8.1 Migrating older presets

- **Slot V32 or older.** No upmixer data. Both profiles load at defaults, disabled.
- **Slot V33 to V41.** Profile 0 gets the stored v1 fields. Its sources, counts and back offset are the defaults (0, 1, 0x01, 2 ms), which reproduce v1 activation. Profile 1 loads at defaults, disabled.
- **Routes for every slot older than V42**, with the upmixer on or off, are derived from the preset's matrix, so turning the upmixer on later sounds as it did in v1. A disabled profile claims nothing, so the routes are inert until then. For each enabled output, the firmware collects the enabled crosspoints of rows 0 to 4 whose gain is above the matrix mute (-120 dB).
  - If exactly one row r is found, the output's bit is set on the generated channel that row carried in v1 (0 to A', 1 to B', 2 to C, 3 to SA, 4 to SB). Rows 3 and 4 map even when the surround mode is OFF, so turning surrounds on later behaves as it did in v1.
  - If two or more rows are found, the output gets no route and keeps its matrix feed.
  - The trim is the crosspoint gain rounded to 0.5 dB and clamped to the trim range, and the polarity is the crosspoint's phase. If one row feeds several single-row outputs at different gains, the largest gain and its phase win.
- Crosspoints are left in place. Rows 2 to 4 are real inputs in multichannel mode, and rows 0 and 1 become the parked mix.

The migration is lossy in four cases.

- An output fed by several rows, such as a sub on rows 0 and 1, now receives raw L+R instead of v1's L'+R'. This is the more correct sub feed.
- A phantom fold such as row 0 plus row 2 on one output now receives raw L, which already contains the centre.
- Outputs fed by one row at different gains share one trim.
- A crosspoint gain above +6 dB or below -40 dB is clamped to the trim range, which changes that output's level.

### 8.2 Host compatibility

Console builds made for v1 must not be used with this firmware. They route the upmixer through matrix rows 2 to 4, which are now always real inputs, so their routing gives no upmix output. They also know neither profiles nor routes, and their bulk SET carries an older format version and is refused. Firmware and Console ship as a matched pair.

A v1 host that only reads and writes `FX.UPMIX_CONFIG` and the per-parameter commands with `wValue` 0 still works on profile 0. A 16-byte `FX.UPMIX_STATUS` read returns the original fields, with the reason codes of section 5.1.

---

## 9. Behaviour notes and edge cases

- **Sample rates.** 48 kHz and below (44.1 and 48 kHz in practice). Above 48 kHz the upmixer parks (`parked_reason` 3). A rate change parks it hard, recomputes every time-based coefficient, and restarts it with a fade-in.
- **Input switching.** Any input source switch passes through the pipeline reset, which parks the engine hard. It restarts with a fade-in if a profile wins for the new input. A change of input channel count also re-selects the profile.
- **Disabled outputs.** They are zeroed by the matrix, get no claim, and their routes are silent. When re-enabled, the routes fade in.
- **Signal generator.** Generator injection happens after the upmixer, so a generator channel replaces the whole output, upmixer contribution included. Generator RAW outputs bypass output processing as usual.
- **CPU.** Measured on RP2350 at 307.2 MHz with 48-sample packets, before module routing and the back pair existed, the engine cost 19,374 to 20,409 cycles per packet in adaptive mode, 13,333 to 14,482 in passive mode and 4,176 to 5,076 with the centre only. Total core 0 load on a loopback build was about 12 % adaptive and 11 % passive with a stereo-to-5.0 routing plus a matrix sub, and about 13 % and 11 % with the back pair routed as well. There is no cost when no profile runs.
- **RAM.** v1's engine state was about 12.4 KB of BSS. v2 phases 1 to 4 added about 4 KB of BSS (back allpass rings, render chunk buffer, routing and claim state, second profile). None on RP2040.
- **Mono compatibility.** With width 0 and full extraction, each main channel reconstructs exactly for the extracted component as A' + 0.7071 x C = A (and likewise for B). Folding the centre back into the mains at the standard -3 dB convention is therefore lossless. Equivalently, A' + B' + 1.4142 x C = A + B.
- **What the upmixer never does.** It never writes the input channels, adds latency to A', B' or C, shifts inter-slot alignment, runs two profiles at once, runs above 48 kHz, produces an LFE channel, or processes while no profile wins.

---

## 10. Quick-start recipe (app pseudocode)

```
if FX.UPMIX_CONFIG_EXT (0x83) not in SYS.PAGE_COMMANDS(0x08):
    hide upmixer UI (RP2040, or firmware with the v1 interface); stop.

// One-click "stereo to 5.0" in profile 0 (input channels 0/1, stereo input only).
P = 0
ext = GET FX.UPMIX_CONFIG_EXT wValue=P         // 36 bytes
ext.src_a = 0; ext.src_b = 1
ext.active_counts = 0x01                       // 2-channel input
ext.routes[A']  = { out_mask = bit(frontL), trim_q1 = 0,  flags = 0 }
ext.routes[B']  = { out_mask = bit(frontR), trim_q1 = 0,  flags = 0 }
ext.routes[C]   = { out_mask = bit(centre), trim_q1 = -6, flags = 0 }   // -3 dB
ext.routes[SA]  = { out_mask = bit(sideL),  trim_q1 = -9, flags = 0 }   // -4.5 dB
ext.routes[SB]  = { out_mask = bit(sideR),  trim_q1 = -9, flags = 0 }
SET FX.UPMIX_CONFIG_EXT wValue=P ext           // routes first, so the start is complete

cfg = GET FX.UPMIX_CONFIG wValue=P             // 44 bytes
cfg.enabled = 1
cfg.center_mode = ADAPTIVE                     // 1
cfg.surround_mode = ADAPTIVE                   // 2
SET FX.UPMIX_CONFIG wValue=P cfg
// Leave matrix rows 0/1 on frontL/frontR (parked mix). Leave the sub on rows 0+1
// with no route, low-pass it, and high-pass frontL/frontR/centre.

// Add backs for 7.0
SET FX.UPMIX_ROUTE wValue=(P << 8) | 5  { bit(backL), -9, 0 }   // BA
SET FX.UPMIX_ROUTE wValue=(P << 8) | 6  { bit(backR), -9, 0 }   // BB

// Live UI loop (10 Hz)
st = GET FX.UPMIX_STATUS                       // 32 bytes
show corr meter    = st.corr_q14 / 16384.0
show centre gauge  = st.center_gain_q15 / 32767.0
show channel meter = st.gen_peak[g] / 32767.0, clip LED = st.gen_clip bit g
switch st.parked_reason:
    1: banner("Upmixer off")
    2: banner("Upmixer idle: no profile for this input channel count")
    3: banner("Upmixer idle: sample rate above 48 kHz")
    4: banner("Upmixer idle: a source channel is not present")
    6: banner("Upmixer idle: nothing routed to an enabled output")

// Save with the rest of the preset
SET STORE.PRESET_SAVE as usual. Both profiles ride along (slot V42).
```

---

## 11. Version summary

| Item | Value |
|---|---|
| Vendor commands | `FX.UPMIX_CONFIG` (0x80), `FX.UPMIX_STATUS` (0x81), `FX.UPMIX_ROUTE` (0x82), `FX.UPMIX_CONFIG_EXT` (0x83), `FX.UPMIX_ENABLE` to `FX.UPMIX_PRESENCE` (0x90 to 0x9D), `FX.UPMIX_SOURCE_A`, `FX.UPMIX_SOURCE_B`, `FX.UPMIX_ACTIVE_COUNTS`, `FX.UPMIX_BACK_OFFSET` (0x9E to 0xA1), all in page 0x08 |
| FX page revision | 2. 0x80 and 0x90 to 0x9D gained the profile address, 0x82, 0x83 and 0x9E to 0xA1 were added, and the status grew from 16 to 32 bytes |
| Wire format | V35. Profile 0 bytes 0 to 43 at offset 5900 (section 22, since V25), section 26 `upmix2` at offset 6136 (116 bytes). Total 6252 B |
| Preset slot | V42 (116-byte tail-append). V33 to V41 slots load into profile 0 with routes migrated from matrix rows 0 to 4 (section 8.1) |
| Change notifications | One `PARAM_CHANGED` per changed field of either profile on every SET (section 6.7) |
| Platforms | RP2350 only (RP2040: the commands do not exist, `VC_ERR_UNKNOWN`) |
| Profiles | 2, lowest enabled profile serving the input count wins |
| Generated channels | 7: A', B', C, SA, SB, BA, BB. Routed per channel with claim semantics. No matrix rows |
| Status telemetry | `FX.UPMIX_STATUS`, 32 B, poll 5 to 20 Hz. Generated-channel peaks and clip bits are here, not in `MEAS.LEVELS` |

---

## 12. Adaptive surround design (2026-11-20)

*Terms updated 2026-11-22 for v2.* In this section L and R mean sources A and B, and Ls and Rs mean SA and SB. The back pair BA/BB uses the same feed, weights and gains at its deeper tap (section 1.3), with its own fade that starts when the back audio first emerges.

**Why it changed.** The earlier ADAPTIVE mode followed the Dolby low-complexity decoder (WO2007067320A2). Its dominance detector, a gain of 1024 plus a hard clip, is close to a smoothed sign bit. A hard-left source reads as neutral on the front/back axis, so it reached Ls at 0.75 x 0.871, about -3.7 dB re its front level. Uncorrelated hard-L and hard-R sources reached the rears at about -6 dB. The fixed 0.871 / -0.4898 feed leaked centre at -8.4 dB whenever the rears were open. The detector ran full-band, so bass steered a 300 Hz to 7 kHz surround band. An offline model (`tools/upmix_proto`) reproduced all four figures before anything changed.

**Signal flow per sample.** The surround HP/LP band-limits the raw L/R, before centre removal. Band-limited L'/R' follow by linearity (`bL' = bL - rem * (bL + bR)`) and go into the Haas rings. Every detector reads the undelayed band signals: the gate and dominance use the raw pair, because removal subtracts the same mid from both channels and would make L'/R' look balanced and anti-correlated. The steered feed is applied to the delayed band signals, then the Schroeder allpass runs as before. The delay moved rather than grew: Ls and Rs keep exactly the Haas plus allpass latency of the passive path at every delay setting.

**Terms.**
- *Residual feed.* `Ls = L' - wl * R'`, `wl = lr' / (rr' + 1e-3 (ll' + rr'))`, and mirrored for Rs. The primed moments come once per block from the raw 40 ms band moments through the removal matrix (`L' = a*L + b*R`, `a = 1 - rem`, `b = -rem`). A single panned source is predicted from the other side and cancels. The weight is clamped to [0.5623, 4]. The floor is the PLII ratio, so while the estimator lags a centre onset the feed is never worse than the old fixed decode. Anti-phase content (w would be negative) passes. The ceiling bounds the noise gain of `-w*R'`.
- *Floor relaxation, disabled.* `UPMIX_FLOOR_Q` lets the floor fall with balance on the weight that feeds the dominant channel into the opposite surround. In the model this cleaned the quiet side of a hard-panned source, but measured per side it let centre content (a kick, a voice) into that surround: the kick reached Rs 7.4 dB above the old engine. It ships at 0, a fixed floor.
- *Side-weighted gate.* With `d = (ll - rr)/(ll + rr)` on the raw moments, the dominant side's gain scales by `1 - |d|` and the quiet side's by `(1 - |d|)^0.75` (`UPMIX_GATE_K`). A fully one-sided gate kept the most ambience, but it swung the ambience between the rears by 13 to 16 dB (std) when sources alternated sides. Softer quiet-side exponents leaked centre into the quiet surround: K 0.5 put a centre kick 1.5 to 2 dB above the old engine there. K 0.75 is the softest value that keeps every centre stem within 1 dB of the old engine in each surround, worst case 0.73 dB, and it keeps about 3 dB more quiet-side ambience than the symmetric gate (K 1) on a hard-panned mix.
- *Front/back dominance, F/B axis only.* The patent's front/back term still closes the rears on front-dominant material, covering residual-estimator lag on centre onsets, and opens them for back (anti-phase) content. It reads the raw band pair, so centre removal does not move it. The left/right axis was removed: it duplicated the gate and cost selectivity.
- *Rear fade.* The rears fade in over 10 ms from exactly zero, starting at the first block at or after the point where delayed audio first emerges from the Haas ring, on unpark and on every surround (re)start.
- *Transient duck.* A 2 ms / 60 ms power-envelope ratio on each side's residual. Above 4 the gain becomes `sqrt(4/ratio)`, releasing over 60 ms. Because control leads audio by the Haas delay, the duck lands before the onset reaches the rears.

**Measured** (compiled firmware source on the host, stem-through; dB re the stem's front level through the same band limit; default settings unless stated). "Net" is the leak change minus the ambience change versus the old engine, so a plain gain cut scores 0. Negative is better. `tools/upmix_proto/harness` also checks every direct and centre stem per side across centre ADAPTIVE / PASSIVE / OFF at width 0, 25 and 100. None is more than 1 dB worse than the old engine in either surround, and total ambience is never more than 2 dB above it.

| Case | Old engine | New engine |
|---|---|---|
| Hard-left source, Ls / Rs | -3.7 / -64.8 | below -200 / below -200 |
| Uncorrelated hard L + hard R, Ls / Rs of the left one | -5.8 / -11.7 | -9.0 / -13.4 |
| Single source at pan 0 / 0.25 / 0.5 (total rears) | -72.8 / -58.2 / -51.4 | -116.5 / -89.4 / -73.1 |
| Centre voice with hard-panned guitar, Ls / Rs | -15.5 / -16.4 | -17.6 / -16.6 |
| Centre kick with hard-panned keys, Ls / Rs | -18.8 / -21.2 | -24.4 / -22.0 |
| Centre voice with hard-panned hats, Ls / Rs | -21.0 / -23.3 | -29.9 / -25.6 |
| Hard-panned guitar, Ls / Rs (net total) | -7.0 / -13.4 | -9.1 / -13.3 (-1.0) |
| Hard-panned keys, Ls / Rs (net total) | -10.0 / -18.1 | -21.8 / -22.8 (-4.0) |
| Hard-panned hats, Ls / Rs (net total) | -9.8 / -15.9 | -15.3 / -16.9 (-2.1) |
| Centre PASSIVE width 0: hard-panned keys, Ls / Rs | -12.1 / -15.3 | -23.9 / -20.1 |
| Panned snare onset (first 50 ms) | -7.8 | -29.4 |
| Anti-phase retention | 0 | +1.2 |
| Diffuse ambience retention | 0 | +0.6 |
| Ambience level motion when a hard-panned part alternates sides (std, Ls / Rs) | 5.6 / 5.2 | 4.8 / 5.6 |

The last row is the one trade. With any balance gate strong enough to give the hard-pan improvements above, the Rs ambience level moves about 0.4 dB (std) more than under the old engine when a hard-panned part alternates sides, while Ls moves 0.8 dB less. The harness accepts up to 1 dB more per side, the same tolerance as the stem rule.

The limit is fundamental to broadband gain steering. A hard-panned source and decorrelated ambience look the same statistically at this resolution. The rears are cleaner of direct sound, but they cannot be made clean of a hard-panned instrument without also losing the ambience that shares its time and band. Multiband steering (2 or 3 bands) bought 1 to 3 dB more net selectivity in the model for roughly twice the ops, and was not ported.

**Tunables** (`upmix.h`): `UPMIX_W_TAU_MS` 40, `UPMIX_W_FLOOR` 0.5623, `UPMIX_W_CEIL` 4, `UPMIX_W_EPS` 1e-3, `UPMIX_GATE_K` 0.75 (quiet-side exponent), `UPMIX_FLOOR_Q` 0, `UPMIX_DUCK_THR` 4, `UPMIX_DUCK_DEPTH` 0.5, `UPMIX_DUCK_FAST_MS` 2, `UPMIX_DUCK_SLOW_MS` 60, `UPMIX_DUCK_REL_MS` 60, `UPMIX_FB_NEUTRAL` 0.75. `UPMIX_GATE_K = 1` with `UPMIX_FLOOR_Q = 0` gives the symmetric gate and fixed floor. The sensitive values are the quiet-side K and the floor Q, which trade centre leak into the quiet surround against kept ambience. The 1 dB centre limit holds for K 0.75 and 1 with Q 0, and for K 1 with Q 0.5 right at the limit. It fails for every K at Q 1 and for K 0.5 at any Q. Hardware-untested.
