# DSPi Stereo Upmixer Specification

*Version 1.5, 2026-11-16 (host interface on the paged protocol of firmware 1.1.7, section 6). Previously 1.4, 2026-10-10 (change notifications, section 6.5). Firmware: upmixer wire layout final at V27 (current wire format V32), upmixer slot layout final at V34 (current preset slot V39), vendor commands `FX.UPMIX_CONFIG`, `FX.UPMIX_STATUS` and `FX.UPMIX_*` (page 0x08, commands 0x80, 0x81 and 0x90-0x9D). RP2350 only. Hardware-untested at time of writing (implemented and audited on the bench build only).*

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
| Surround | `OFF` (0), `PASSIVE` (1), `ADAPTIVE` (2) | Passive: difference feed (L-R). Adaptive: Dolby low-complexity matrix decoder steering (WO2007067320A2) with Pro Logic II decode coefficients; front dominance ducks the surrounds, left/right dominance steers between Ls and Rs. |

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
- Ls/Rs adaptive steering gains reach at most 1.0; passive surround peaks at 0.7071 x (L-R). Rear level trim is simply the crosspoint gain.

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

Fixed internals (not configurable): correlation estimator time constant 100 ms, dominance smoothers 40 ms, decorrelator delay 10 ms, activation fade 10 ms, presence bell corner 3 kHz / Q 0.6 (TPT SVF bell, boost/cut symmetric).

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
| 8 | u16 | ls_gain_q15 | Live Ls steering gain, Q15 |
| 10 | u16 | rs_gain_q15 | Live Rs steering gain, Q15 |
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
- **CPU:** roughly 90 float ops/sample when fully active (adaptive centre + adaptive surround); well under 1 percent of the RP2350 budget. Zero measurable cost when disabled (one pointer compare per packet).
- **RAM:** ~12.3 KB BSS on RP2350 (delay/decorrelator rings sized for 48 kHz). None on RP2040.
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
