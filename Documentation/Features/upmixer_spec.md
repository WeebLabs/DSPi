# DSPi Stereo Upmixer Specification

*Version 1.6, 2026-11-20 (ADAPTIVE surround reworked: residual feed, side-specific balance gate, transient duck and lookahead; design and measurements in section 12; no host interface change). Previously 1.5, 2026-11-16 (host interface on the paged protocol of firmware 1.1.7, section 6). Previously 1.4, 2026-10-10 (change notifications, section 6.5). Firmware: upmixer wire layout final at V27 (current wire format V32), upmixer slot layout final at V34 (current preset slot V39), vendor commands `FX.UPMIX_CONFIG`, `FX.UPMIX_STATUS` and `FX.UPMIX_*` (page 0x08, commands 0x80, 0x81 and 0x90-0x9D). RP2350 only. Hardware-untested at time of writing (implemented and audited on the bench build only).*

This document contains everything an application developer needs to integrate the DSPi stereo upmixer: the concept, the routing model, every command byte layout, parameter semantics with ranges and defaults, status telemetry, persistence behavior, and recommended UX patterns.

---

## 1. What it does

The upmixer derives up to three additional channels from a stereo input:

- **Centre (C)**: centre-panned correlated content (vocals, dialogue, solo instruments) extracted so a physical centre speaker can anchor the image; the phantom centre no longer shifts as the listener moves off-axis.
- **Left Surround / Right Surround (Ls/Rs)**: ambience and decorrelated content steered to rear speakers, with a built-in delay, band-limit, and decorrelation chain so surrounds work with no manual setup.

The extracted centre energy is (configurably) removed from L/R so a real centre speaker and the L/R phantom image do not comb-filter.

That removal is the **only** thing the upmixer does to the stereo pair; the surround engine reads L/R but never writes them (the Dolby patent's front-channel gain riding is deliberately omitted). So there are two ways to run the upmixer with the mains as recorded, and both leave the surrounds fully operational:

- **Centre mode `OFF`** (see below): no centre channel at all, L/R bit-exact. The surrounds-only setup.
- **`center_width_pct` = 100**: centre still extracted and routable on row 2, L/R bit-exact, phantom centre kept. Expect combing if a real centre speaker also plays.

Two independent engines, each with a quality/CPU mode:

| Engine | Modes | Algorithm |
|---|---|---|
| Centre | `PASSIVE` (0), `ADAPTIVE` (1), `OFF` (2) | Passive: fixed C = 0.7071(L+R). Adaptive: running normalized cross-correlation plus L/R balance steer the centre gain through a threshold gate with attack/release ballistics. Off: extraction gain forced to zero, so row 2 is silent and L/R pass through untouched. |
| Surround | `OFF` (0), `PASSIVE` (1), `ADAPTIVE` (2) | Passive: difference feed (L-R). Adaptive: prediction-residual feed (each surround keeps what its own side cannot predict from the other), a balance gate that closes the surround on the dominant side, front/back dominance and a transient duck, all steered from the band-limited signal ahead of the surround delay (section 12). |

Both surround modes feed a built-in conditioning chain per rear channel: Butterworth high-pass and low-pass band-limit, Haas delay, and mirrored Schroeder allpass decorrelators.

Note the centre enum orders its `OFF` last, unlike the surround enum. `OFF` was appended as value 2 rather than renumbering `PASSIVE`/`ADAPTIVE` to match the surround layout, because the vendor interface has no per-command version negotiation and moving 0/1 would have silently remapped every existing host and saved preset. Hosts should present the modes in whatever order suits the UI (Off first is fine); only the wire values are fixed.

**Latency:** zero added latency on C/L/R (pure gain steering). The surround delay is a deliberate psychoacoustic feature (precedence effect), identical for every output slot fed from the same source, so inter-output-slot sample alignment is never disturbed.

---

## 2. Activation model

The upmixer processes audio only when **all** of these hold:

1. `enabled` is true (config).
2. The active input is a plain stereo pair: `active_input_channel_count() == 2`. True for S/PDIF inputs, 2-channel USB alt, 2-channel I2S. False for 4/6/8-channel USB alts, multichannel I2S, and ADAT (always 8-channel).
3. The sample rate is 48 kHz or below (44.1/48 kHz). Above 48 kHz the upmixer parks with `parked_reason` 3, mirroring the ADAT rate policy; it resumes automatically when the rate returns to 48 kHz or below.

Otherwise the upmixer **parks**: processing state (delay rings, estimators, envelopes) is reset and the derived rows are not exposed to the matrix. When it re-activates, a 10 ms fade-in plus estimator warm-up prevents clicks.

`FX.UPMIX_STATUS` reports `active` and a `parked_reason` (see section 6.5) so the app can explain why upmixing is not running.

**Disable behavior:** disabling while routed is a hard stop (derived rows vanish from the mix within one packet). If click-free disable matters in your UX, mute or ramp the relevant output slots first.

---

## 3. Routing model: virtual matrix sources

The upmixer does not have its own routing. The derived channels become ordinary **matrix source rows**, routed with the same crosspoint commands the app already uses:

| Matrix source row | Stereo input mode, upmixer active | Multichannel input mode |
|---|---|---|
| 0 | Input L (post-EQ, post-leveller, post centre removal) | Input 1 |
| 1 | Input R (same) | Input 2 |
| 2 | **Upmix C** | Input 3 |
| 3 | **Upmix Ls** (only when surround mode != OFF) | Input 4 |
| 4 | **Upmix Rs** (only when surround mode != OFF) | Input 5 |
| 5-7 | inactive | Inputs 6-8 |

Everything downstream applies unchanged per output slot: crosspoint gain/phase, output PEQ, crossover, per-output delay, gain, mute, loudness, psybass.

**Row-sharing caveat (important for app UI):** rows 2-4 use the same crosspoint storage as multichannel inputs 3-5. A preset that routes "input 4" to an output will, in stereo-plus-upmix mode, route "Upmix Ls" to that output. The app must label these rows contextually based on the active input mode and upmixer state. This is by design (it keeps the wire format and preset layout unchanged) and mirrors how the hardware treats the rows.

**When surround mode is OFF**, rows 3-4 are not summed into any output even if their crosspoints are enabled; only row 2 (C) is exposed.

**When centre mode is OFF**, row 2 stays exposed but silent. It is deliberately not withdrawn: dropping it would renumber Ls/Rs onto rows 2-3 and silently repoint existing matrix routing. Apps should show the C row greyed or at -inf rather than hiding it.

### 3.1 Level conventions

- C is produced at the constant-power convention: full extraction of a 0 dBFS mono signal yields C = +3 dBFS on row 2 and silence on rows 0/1. **The centre row can exceed 0 dBFS by up to 3 dB.** Watch its meter (section 5) and set the C crosspoint gain to -3 dB if the downstream chain has no headroom.
- Passive surround peaks at 0.7071 x (L-R). Adaptive surround gains reach at most 0.75 and the residual feed `L' - w*R'` has `w` in [0.5623, 4], so full-scale anti-phase content reaches about +1.4 dBFS (0.75 x 1.56). A transient where the weight still sits high from correlated material while the inputs swing apart can momentarily exceed that, in theory up to 0.75 x 5. Watch rows 3-4 on the meters. Rear level trim is simply the crosspoint gain.

### 3.2 Example setups

3.0 stereo (stable centre, no rears): surround mode OFF; route row 0 to Front L slot, row 1 to Front R slot, row 2 to Centre slot (crosspoint gain 0 or -3 dB to taste). Set `center_width_pct` to taste (25 default keeps a little centre energy in L/R).

5.0-style music: as above plus surround mode ADAPTIVE, route row 3 to the Ls slot and row 4 to the Rs slot, rear crosspoint gains around -3 to -6 dB. The built-in delay/band-limit/decorrelation needs no further setup; per-output delay can still be added on the rear slots for room distance compensation.

Surrounds only (stereo front kept exactly as recorded): centre mode OFF, surround mode PASSIVE or ADAPTIVE; route row 0 to Front L, row 1 to Front R, rows 3-4 to the rear slots. L/R are bit-identical to the input through the whole pass, so the front image is untouched and the rears add ambience only. Strength, width, threshold, attack, release, detector HPF and presence are all inert here.

3.1 / 5.1 bass management: route rows into a sub output with the existing crossover on that slot, exactly as with any other source.

---

## 4. Parameters

All persisted in presets (slot V34+) and carried in bulk params (wire V27+). Floats are IEEE 754 single precision little-endian. Out-of-range floats are clamped by the firmware when coefficients are computed (the stored config keeps the written value). A NaN or infinite float is refused with `VC_ERR_VALUE` and stores nothing. The Command column names the per-parameter command (page 0x08) of section 6.2.

| # | Param id | Field | Range | Default | Meaning | Command |
|---|---|---|---|---|---|---|
| 0 | `UPMIX_PARAM_ENABLED` | enabled | 0/1 | 0 | Master enable | `FX.UPMIX_ENABLE` (0x90) |
| 1 | `UPMIX_PARAM_CENTER_MODE` | center_mode | 0-2 | 1 (ADAPTIVE) | Centre engine mode. 2 = OFF: no C output, L/R bit-exact, surrounds unaffected. Values above 2 fall back to the ADAPTIVE default (not clamped up to OFF, so a corrupt byte cannot silently disable the centre) | `FX.UPMIX_CENTER_MODE` (0x91) |
| 2 | `UPMIX_PARAM_SURROUND_MODE` | surround_mode | 0-2 | 2 (ADAPTIVE) | Surround engine mode | `FX.UPMIX_SURROUND_MODE` (0x92) |
| 3 | `UPMIX_PARAM_STRENGTH` | strength_pct | 0-100 | 100 | Centre extraction strength; scales both the C output and the removal from L/R. **Both centre modes**; in passive mode it is the fixed centre gain (the primary passive control) | `FX.UPMIX_STRENGTH` (0x93) |
| 4 | `UPMIX_PARAM_CENTER_WIDTH` | center_width_pct | 0-100 | 25 | How much extracted centre stays in L/R. 0 = full removal (discrete centre), 100 = L/R untouched (C is additive; expect combing if a real centre speaker plays). **Both centre modes** | `FX.UPMIX_CENTER_WIDTH` (0x94) |
| 5 | `UPMIX_PARAM_THRESHOLD` | corr_threshold_pct | 0-95 | 30 | Correlation gate. Centre presence below this extracts nothing; above it, extraction scales smoothly to full. Raise to extract only strongly-correlated content. **Adaptive centre mode only**; no effect in passive | `FX.UPMIX_THRESHOLD` (0x95) |
| 6 | `UPMIX_PARAM_ATTACK` | attack_ms | 1-500 | 10 | Centre gain rise time. **Adaptive centre mode only** (in passive mode it still smooths strength/mode changes, but has no steady-state effect) | `FX.UPMIX_ATTACK` (0x96) |
| 7 | `UPMIX_PARAM_RELEASE` | release_ms | 5-2000 | 100 | Centre gain fall time. **Adaptive centre mode only** (same passive-mode caveat as attack) | `FX.UPMIX_RELEASE` (0x97) |
| 8 | `UPMIX_PARAM_DET_HPF` | detector_hpf_hz | 20-1000 | 200 | Detector bass-cut corner. Bass correlates by coincidence and would pump the steering; content below this corner is ignored by the detector (the audio itself is not filtered). **Adaptive centre mode only**; no effect in passive | `FX.UPMIX_DET_HPF` (0x98) |
| 9 | `UPMIX_PARAM_SUR_DELAY` | surround_delay_ms | 0-20 | 12 | Haas delay on Ls/Rs. Rule of thumb ~1 ms per foot of listener distance | `FX.UPMIX_SUR_DELAY` (0x99) |
| 10 | `UPMIX_PARAM_SUR_HPF` | surround_hpf_hz | 20-2000 | 300 | Surround band-limit high-pass (keeps rumble out of the rears) | `FX.UPMIX_SUR_HPF` (0x9A) |
| 11 | `UPMIX_PARAM_SUR_LPF` | surround_lpf_hz | 1000-20000 | 7000 | Surround band-limit low-pass (classic 7 kHz surround voicing; raise for full-band rears) | `FX.UPMIX_SUR_LPF` (0x9B) |
| 12 | `UPMIX_PARAM_DECORR` | decorr_pct | 0-100 | 90 | Decorrelator amount; allpass gain G = 0.5 x pct/100. 0 disables decorrelation | `FX.UPMIX_DECORR` (0x9C) |
| 13 | `UPMIX_PARAM_PRESENCE` | presence_db | -12..+12 | 0 | Centre presence bell gain (dB) at a fixed 3 kHz, Q 0.6. Negative moves voices back, positive brings them forward (Syn-style presence). **Both centre modes.** Via `FX.UPMIX_PRESENCE` the value is a plain float dB; in the config packet and presets it is carried in 0.5 dB steps | `FX.UPMIX_PRESENCE` (0x9D) |

**Per-mode applicability (UI guidance):** when the centre mode is `OFF`, grey out the whole centre group (3, 4, 5, 6, 7, 8, 13); none of it has any effect. When the centre mode is `PASSIVE`, keep **Strength** (3) and **Width** (4) accessible; they are the passive mode's working controls. Keep **Presence** (13) accessible in both centre modes as well. Grey out only **Threshold** (5), **Attack** (6), **Release** (7), and **Detector HPF** (8); those drive the adaptive steering and are inert in passive mode. Do not disable the whole centre group on mode. The surround conditioning params (9-12) apply in both `PASSIVE` and `ADAPTIVE` surround modes and should be greyed only when the surround mode is `OFF`.

Fixed internals (not configurable): correlation estimator time constant 100 ms, front/back dominance smoother 40 ms, residual-weight estimators 40 ms, duck envelopes 2 ms / 60 ms with 60 ms release, decorrelator delay 10 ms, activation fade 10 ms, presence bell corner 3 kHz / Q 0.6 (TPT SVF bell, boost/cut symmetric). The adaptive surround detector band follows `surround_hpf_hz` / `surround_lpf_hz`, and its lookahead equals `surround_delay_ms`.

Parameter changes take effect within a few ms (main-loop coefficient recompute, double-buffered publish, no audio interruption). Mode and parameter changes while running are click-safe: gains ramp per sample, and a live surround OFF to ON transition clears the rear conditioning state and fades the rears in. Switching the centre engine to OFF ramps the extraction and removal gains linearly to zero across one packet (about 1 ms at 48 kHz) and then holds exact zero; it deliberately bypasses the release ballistic, which would only ever decay asymptotically toward zero and never let L/R become bit-exact again.

---

## 5. Metering

While the upmixer is active, the standard channel meters (the `MEAS.LEVELS` peaks array and clip flags) report the derived rows:

- `peaks[2]` = Upmix C, `peaks[3]` = Upmix Ls, `peaks[4]` = Upmix Rs (uint16, 32767 = 0 dBFS; the meter saturates at 0 dBFS even though C can internally reach +3 dBFS).
- `clip_flags` bits 2-4 flag derived-row overloads (threshold just above 0 dBFS).

When parked or disabled, these rows read zero, exactly as idle multichannel inputs do. Rows 3-4 read zero when surround mode is OFF.

---

## 6. Vendor commands

The upmixer commands live in the `FX` page (0x08) of the paged protocol (see `../commands.md`), on USB EP0 vendor requests and on the UART and I2C control transports, which share the same dispatcher. Each command is one number for both directions, and none takes an address, so `wValue` is always 0. Multi-byte values are little-endian.

On **RP2040** the feature does not exist. None of these commands is in the RP2040 command table, so every request answers `VC_ERR_UNKNOWN` (a STALL on USB, the status byte on UART and I2C). A host detects the feature from the GET bit of `FX.UPMIX_CONFIG` in `SYS.PAGE_COMMANDS` for page 0x08, or from the platform in `SYS.PLATFORM`, before showing upmixer UI.

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `FX.UPMIX_CONFIG` | 0x08 | 0x80 | GET, SET | none (0) | GET 44, SET 44 | `UpmixConfigPacket` (6.1) |
| `FX.UPMIX_STATUS` | 0x08 | 0x81 | GET | none (0) | GET 16 | `UpmixStatus` (6.3) |
| `FX.UPMIX_ENABLE` | 0x08 | 0x90 | GET, SET | none (0) | GET 4, SET 4 | Parameter 0, float32 |
| `FX.UPMIX_CENTER_MODE` | 0x08 | 0x91 | GET, SET | none (0) | GET 4, SET 4 | Parameter 1, float32 |
| `FX.UPMIX_SURROUND_MODE` | 0x08 | 0x92 | GET, SET | none (0) | GET 4, SET 4 | Parameter 2, float32 |
| `FX.UPMIX_STRENGTH` | 0x08 | 0x93 | GET, SET | none (0) | GET 4, SET 4 | Parameter 3, float32 |
| `FX.UPMIX_CENTER_WIDTH` | 0x08 | 0x94 | GET, SET | none (0) | GET 4, SET 4 | Parameter 4, float32 |
| `FX.UPMIX_THRESHOLD` | 0x08 | 0x95 | GET, SET | none (0) | GET 4, SET 4 | Parameter 5, float32 |
| `FX.UPMIX_ATTACK` | 0x08 | 0x96 | GET, SET | none (0) | GET 4, SET 4 | Parameter 6, float32 |
| `FX.UPMIX_RELEASE` | 0x08 | 0x97 | GET, SET | none (0) | GET 4, SET 4 | Parameter 7, float32 |
| `FX.UPMIX_DET_HPF` | 0x08 | 0x98 | GET, SET | none (0) | GET 4, SET 4 | Parameter 8, float32 |
| `FX.UPMIX_SUR_DELAY` | 0x08 | 0x99 | GET, SET | none (0) | GET 4, SET 4 | Parameter 9, float32 |
| `FX.UPMIX_SUR_HPF` | 0x08 | 0x9A | GET, SET | none (0) | GET 4, SET 4 | Parameter 10, float32 |
| `FX.UPMIX_SUR_LPF` | 0x08 | 0x9B | GET, SET | none (0) | GET 4, SET 4 | Parameter 11, float32 |
| `FX.UPMIX_DECORR` | 0x08 | 0x9C | GET, SET | none (0) | GET 4, SET 4 | Parameter 12, float32 |
| `FX.UPMIX_PRESENCE` | 0x08 | 0x9D | GET, SET | none (0) | GET 4, SET 4 | Parameter 13, float32 |

**Control Surfaces** (caps v4+; v5 for the three-value centre mode): six front-panel nouns dispatch through
`vc_set()` to the matching per-parameter command, so physical knobs/buttons/IR commands can drive the
upmixer: `UPMIX` (35, enable), `UPMIX_CENTER_MODE` (36), `UPMIX_SURROUND_MODE`
(37), `UPMIX_STRENGTH` (38), `UPMIX_WIDTH` (39), `UPMIX_PRESENCE` (40). All
six are RP2350-only (empty action mask on RP2040). The front-panel mode
labels call the PASSIVE engine modes "Sinner" and the ADAPTIVE engine modes
"Logician"; the wire values are identical. See `control_surfaces_spec.md` sections 4.3 and 5.

### 6.1 UpmixConfigPacket (44 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | enabled (0/1) |
| 1 | u8 | center_mode (0 = PASSIVE, 1 = ADAPTIVE, 2 = OFF; V27+ for OFF) |
| 2 | u8 | surround_mode (0 = OFF, 1 = PASSIVE, 2 = ADAPTIVE) |
| 3 | i8 | presence_q1: presence bell gain in 0.5 dB steps (dB x 2, -24..+24; V26+, was reserved; 0 = flat) |
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

A `FX.UPMIX_CONFIG` SET with any other length is refused with `VC_ERR_LENGTH`. A NaN or infinite float field is refused with `VC_ERR_VALUE` and nothing is stored. Mode bytes out of range are clamped by the firmware.

### 6.2 Per-parameter commands

Each of the 14 parameters has its own command, `FX.UPMIX_ENABLE` (0x90) to `FX.UPMIX_PRESENCE` (0x9D), in `UPMIX_PARAM_*` order (command = 0x90 + parameter number). Everything travels as a 4-byte float, including the enable and mode params (the firmware rounds the modes to integers; enable is a plain "nonzero = on" test). A SET payload other than 4 bytes is `VC_ERR_LENGTH`, and a NaN or infinite value is `VC_ERR_VALUE`. Prefer the per-parameter SETs for live sliders (no read-modify-write race against other controllers). Use `FX.UPMIX_CONFIG` for atomic apply of a whole panel.

### 6.3 UpmixStatus (16 bytes)

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | active | 1 = processing audio right now |
| 1 | u8 | parked_reason | 0 = active, 1 = disabled, 2 = input not stereo, 3 = sample rate above 48 kHz |
| 2 | i16 | corr_q14 | Smoothed L/R correlation, Q14: value/16384 in [-1, +1]. Zero in passive centre mode |
| 4 | i16 | balance_q14 | Smoothed level balance, Q14: 0 = centred, 16384 = fully one-sided |
| 6 | u16 | center_gain_q15 | Live centre extraction gain, value/32767 in [0, 1] (includes strength, gate, ballistics, fade) |
| 8 | u16 | ls_gain_q15 | Live Ls steering gain, Q15. Adaptive: side gate x front/back term x duck x fade, excluding the residual weight. Passive: the fade only |
| 10 | u16 | rs_gain_q15 | Live Rs steering gain, Q15, same meaning |
| 12 | u8[4] | reserved | |

Poll at 5-20 Hz for a responsive "what is the upmixer doing" display (correlation meter, extraction gain bar, surround activity). Values are telemetry snapshots; no locking needed.

### 6.4 Suggested error handling

- A STALL on RP2350 is a refusal with its reason in `SYS.LAST_RESULT`: `VC_ERR_LENGTH` (wrong length), `VC_ERR_VALUE` (NaN or infinite float) or `VC_ERR_ADDRESS` (non-zero `wValue`). Treat as a client bug.
- `VC_ERR_UNKNOWN` on a device you have identified as RP2040: expected. Hide the feature.

### 6.5 Change notifications

*Added 2026-10-10. Firmware before this date sent no notification for the config or parameter SETs (1.1.6 opcodes 0x4A and 0x4C).*

Every accepted upmixer SET emits a `NOTIFY_EVT_PARAM_CHANGED` event on the notification endpoint for each field whose stored wire value actually changed. The event's offset is the field's position in `WireBulkParams` (section 7), its size is the field's size, and its payload is the new wire value. A SET that changes nothing emits nothing. A per-parameter SET normally emits one event. A `FX.UPMIX_CONFIG` SET emits one event per changed field, up to 14.

This covers every route into these commands: USB, the UART and I2C control transports, and the Control Surfaces nouns. The event's source byte tells them apart (`PARAM_SRC_HOST_SET`, `_UART`, `_I2C` or `_GPIO`). Preset load, factory reset and bulk SET are unchanged. They emit one bulk-invalidation event, not per-field events (section 8).

| Wire offset | Size | Field |
|---|---|---|
| 5900 | 1 | enabled |
| 5901 | 1 | center_mode |
| 5902 | 1 | surround_mode |
| 5903 | 1 | presence_q1 (int8, dB x 2) |
| 5904 | 4 | strength_pct |
| 5908 | 4 | center_width_pct |
| 5912 | 4 | corr_threshold_pct |
| 5916 | 4 | attack_ms |
| 5920 | 4 | release_ms |
| 5924 | 4 | detector_hpf_hz |
| 5928 | 4 | surround_delay_ms |
| 5932 | 4 | surround_hpf_hz |
| 5936 | 4 | surround_lpf_hz |
| 5940 | 4 | decorr_pct |

Presence is the one field where the notified value can differ from the `FX.UPMIX_PRESENCE` GET. Its SET stores the float dB exactly, but the wire byte carries it in 0.5 dB steps. A SET of 3.3 dB therefore notifies `presence_q1` = 7 (3.5 dB), and a further SET that rounds to the same step emits nothing. Float fields are notified as stored, before the downstream range clamp, which matches what the `FX.UPMIX_CONFIG` GET and bulk GET return.

---

## 7. Bulk parameters (wire V25-V27, current V32)

`STORE.ALL_PARAMS` (chunked `STORE.ALL_PARAMS_CHUNK`) carries the upmixer as a section of `WireBulkParams`:

- The section arrived at `WIRE_FORMAT_VERSION` 25 as the final section (total 5944 bytes, was 5900 at V24). Later versions appended the subharmonic, tube and limiter sections after it, so the current V32 total is **6136 bytes**. The upmix offset has not moved.
- The upmix section is a `WireUpmixParams` at **byte offset 5900**, byte-identical to `UpmixConfigPacket` (section 6.1).
- On RP2040 the section is present but zero on GET and ignored on SET (platform-independent wire format).
- Version discipline is strict: the header's format_version must match exactly, so a V26 client's SET is rejected against V27 firmware even though V27 changed no layout at all (it only widened the centre-mode enum). Clients must send the firmware's current format_version (32 at the time of writing).

A bulk SET applies the upmix section exactly like a `FX.UPMIX_CONFIG` SET (deferred coefficient recompute, no audio glitch).

---

## 8. Persistence

- **Preset slots:** the upmix layout is final at `SLOT_DATA_VERSION` 34 (current firmware writes 39; later versions append other features' data after it). The upmix config is a 44-byte tail-append to the slot (V33); V34 claims the reserved byte for presence_q1 with no size change. Presets saved by older firmware (V21-V32) load with the upmixer **disabled at defaults**; V33 presets load with presence 0 dB (the byte was always written 0). Preset save/load round-trips all 14 fields. Loading a preset that disables the upmixer unpublishes it within one main-loop pass.
- **Factory reset:** upmixer disabled, all params to defaults.
- **RP2040 slots:** fields are stored (zeros) for layout uniformity but never applied.
- After any preset load or bulk SET, re-read the upmix config (`FX.UPMIX_CONFIG`) rather than assuming your cached copy; the standard bulk-invalidation notification (notify endpoint) fires as usual and is your refresh trigger.

---

## 9. Behavior notes and edge cases

- **Sample rates:** 44.1 and 48 kHz only. Above 48 kHz the upmixer parks (`parked_reason` 3) and the derived rows go silent; it resumes with the activation fade when the rate drops back. All time-based parameters (delays, ballistics, corners) are recomputed on every rate change; expect a brief (tens of ms) surround transient at a 44.1 <-> 48 kHz change (stale ring content), on par with the existing per-output delay behavior.
- **Input switching:** switching between stereo sources (USB 2ch, S/PDIF, I2S 2ch) keeps the upmixer running (state continues through the pipeline reset). Switching to a multichannel source parks it; switching back re-activates with the fade-in.
- **Signal generator:** siggen injection happens post-matrix and is unaffected. Siggen RAW outputs bypass output processing as usual regardless of source rows.
- **CPU:** about 160 float ops per sample fully active (adaptive centre + adaptive surround; was about 135 before the 2026-11-20 rework, counting add, sub, mul and abs as 1 and a clamp as 2), plus about 70 ops, 7 divides, 7 square roots and 2 `powf` per 1 ms block. Unmeasured on the CPU meter. Zero measurable cost when disabled (one pointer compare per packet).
- **Surround mode switch:** changing between PASSIVE and ADAPTIVE while running clears the rear state and restarts the rears from silence after the Haas delay, because the delay rings hold different signals in the two modes.
- **RAM:** ~12.4 KB BSS on RP2350 (delay/decorrelator rings sized for 48 kHz). None on RP2040.
- **Mono compatibility:** with width 0 and full extraction, each main channel reconstructs exactly for the extracted component as L' + 0.7071 x C = L (and likewise for R), so folding the centre back into the mains at the standard -3 dB convention is lossless. Equivalently, L' + R' + 1.4142 x C = L + R.
- **What the upmixer never does:** add latency to C/L/R, shift inter-slot alignment, run on multichannel inputs, run above 48 kHz, or process when disabled.

---

## 10. Quick-start recipe (app pseudocode)

```
if FX.UPMIX_CONFIG not in SYS.PAGE_COMMANDS(0x08): hide upmixer UI; stop.

// One-click "3.0 stereo" setup
cfg = GET FX.UPMIX_CONFIG           // read current 44-byte config
cfg.enabled = 1
cfg.center_mode = ADAPTIVE          // 1
cfg.surround_mode = OFF             // 0
SET FX.UPMIX_CONFIG cfg
enable crosspoint(row=2, out=centerSlot, gain_db=-3)

// One-click "add surrounds"
SET FX.UPMIX_SURROUND_MODE 2.0f    // ADAPTIVE
enable crosspoint(row=3, out=lsSlot, gain_db=-4.5)
enable crosspoint(row=4, out=rsSlot, gain_db=-4.5)

// Live UI loop (10 Hz)
st = GET FX.UPMIX_STATUS
show corr meter   = st.corr_q14 / 16384.0
show center gauge = st.center_gain_q15 / 32767.0
if st.parked_reason == 2: banner("Upmixer idle: input is not stereo")
if st.parked_reason == 3: banner("Upmixer idle: sample rate above 48 kHz")

// Save with the rest of the preset
SET STORE.PRESET_SAVE as usual; upmix config rides along (slot V34)
```

---

## 11. Version summary

| Item | Value |
|---|---|
| Vendor commands | `FX.UPMIX_CONFIG` (0x80), `FX.UPMIX_STATUS` (0x81), `FX.UPMIX_ENABLE` to `FX.UPMIX_PRESENCE` (0x90-0x9D), all in page 0x08 |
| Wire format | Upmix layout final at V27 (`WireUpmixParams` at offset 5900; V27 widened the centre-mode enum only, no layout change). Current firmware V32, total 6136 B |
| Change notifications | One `PARAM_CHANGED` per changed field on every config or per-parameter SET (section 6.5) |
| Preset slot | V34 (44-byte tail-append; presence byte claimed from reserved, size unchanged). Centre `OFF` needed no slot bump: firmware predating it clamps the unknown byte to the ADAPTIVE default |
| Platforms | RP2350 only (RP2040: the commands do not exist, `VC_ERR_UNKNOWN`) |
| Matrix rows | 2 = C, 3 = Ls, 4 = Rs (shared with multichannel inputs 3-5) |
| Status telemetry | `FX.UPMIX_STATUS`, 16 B, poll 5-20 Hz |
| Derived-row meters | `peaks[2..4]` + clip flags bits 2-4 while active |

---

## 12. Adaptive surround design (2026-11-20)

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
