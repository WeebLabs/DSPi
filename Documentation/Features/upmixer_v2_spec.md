# DSPi Upmixer v2: Pair Engine with Module Routing

*Design spec, 2026-11-22 (revised the same day: one engine with a built-in back pair and two config profiles replaces the two-instance design). Status: agreed design, phases 0 to 3 implemented (2026-11-22). Supersedes sections 2, 3, 5 and parts of 6 to 9 of `upmixer_spec.md` once it lands; the centre and surround algorithms (including section 12) are unchanged. RP2350 only. Implementation plan: `upmixer_v2_plan.md`.*

---

## 1. Summary

Today the upmixer only turns a stereo input into C/Ls/Rs. It borrows idle matrix input rows 2 to 4 to deliver them, so it cannot run when a multichannel source is active.

v2 makes it a general **pair engine**. The user picks any two input channels as its source. The engine splits them into seven generated channels: two processed fronts, a centre, a side ambience pair and a back ambience pair. Each generated channel is routed by the module itself to one or more outputs, with a trim and a polarity flag. The routing matrix does not grow.

There is one engine. It holds two **config profiles**, each tied to the input channel counts it serves, so one preset can upmix stereo music one way and extend 5.1 films another. Only one profile runs at a time.

The firmware stays use-case neutral. The Console offers templates (section 9) that set sources, routing and tuning for a named use case.

## 2. What changes and what does not

Unchanged:
- Centre engine (PASSIVE / ADAPTIVE / OFF), presence bell, surround engine (OFF / PASSIVE / ADAPTIVE), surround conditioning chain, and all 14 existing parameters with their ranges and defaults.
- The 48 kHz rate gate.
- Zero latency on A', B' and C.
- With the back pair unrouted and default sources, the generated A', B', C, SA and SB are bit-identical to v1.

Changed:
- Sources are selectable instead of fixed to inputs 0/1.
- The engine runs after the matrix and writes nothing into input rows, including the in-place centre removal v1 does on L/R today.
- Delivery is by per-channel output masks, not matrix rows. Rows 2 to 4 are always real inputs 3 to 5.
- Activation follows a per-profile input-count mask instead of the fixed "input is stereo" test.
- A back ambience pair (BA/BB) is added.
- Two config profiles share the one engine.

Removed:
- Row sharing and the contextual row labels it required.
- Input meters `peaks[2..4]` and RTA input taps carrying upmix rows. Generated levels move to the status packet (section 7.4). Spectrum views of generated content use the RTA output tap of the output they land on.

Given up compared with a two-engine design, by choice:
- Two pairs upmixed at the same time (for example 5.1 to 9.1, or a second zone).
- Independent band-limit tuning for the back pair.

Stereo widening is **not** part of the upmixer. See section 10.

## 3. Engine and profiles

### 3.1 One engine

- The engine runs on Core 0 after the matrix (PASS 4) and before `siggen_render()` (section 6.3). Its sources are the post-EQ, post-leveller input buffers, which the matrix reads but never writes, so the engine sees exactly what v1 saw in PASS 3.
- The engine state, coefficient double buffer and published-coefficient pointer stay singletons. Rings: Haas 2 x 1024 and side allpass 2 x 512 (unchanged), plus the new back allpass 2 x 256.

### 3.2 Two profiles

- Profiles 0 and 1 are complete configs: every v1 parameter, sources, `active_counts`, `back_offset_ms` and routes.
- **Running profile:** the lowest-numbered profile that is enabled and whose `active_counts` contains the current active input count. Overlapping `active_counts` are legal and never refused. Refusing them would make SET order matter (swapping counts between profiles would pass through an illegal state) and would break bulk and preset loads. With an overlap, profile 0 wins and profile 1 is inert for that count. The Console warns.
- **Profile changes while audio runs.** An input-count change already goes through `complete_pipeline_reset()`, so the old profile stops hard and the new one fades in. A live edit (enable or `active_counts`) can also change the winner, and an edit of the running profile's `src_a` or `src_b` is treated the same way. That change is sequential, never a crossfade. The old profile fades out on its held coefficients and routes, the state resets, and the new profile fades in. The audio thread latches the running profile separately from the selected one.
- Control Surfaces nouns always edit profile 0. This keeps one binding deterministic whatever the input is, and it matches migrated presets.

## 4. Sources and activation

Per profile:
- `src_a`, `src_b`: input channel indices 0 to 7. `src_a` plays the role of L and `src_b` the role of R in every engine equation. `src_a == src_b` is legal and degenerate (fully correlated).
- `active_counts`: bit mask over active input channel counts. Bit 0 = 2 channels, bit 1 = 4, bit 2 = 6, bit 3 = 8.

The engine runs when a profile wins (section 3.2), both of its sources are below the active input count, and the rate is 48 kHz or below. Otherwise it parks.

`active_counts` is required, not optional. A stereo-to-5.1 profile reads inputs 0 and 1, which exist in every mode, so a "source present" test alone would run it on a real 5.1 stream and overwrite real centre and surround outputs.

Defaults for both profiles: sources 0 and 1, `active_counts` = 0x01 (stereo only), profile 1 disabled. This reproduces v1 activation.

## 5. Generated channels

| g | Name | Content |
|---|---|---|
| 0 | A' | `src_a` after centre removal (bit-exact copy when centre is OFF or width is 100) |
| 1 | B' | `src_b` after centre removal |
| 2 | C | Extracted centre (correlated content of the pair) |
| 3 | SA | Side ambience, A side (surround engine output after band-limit, Haas delay, 10 ms decorrelator) |
| 4 | SB | Side ambience, B side |
| 5 | BA | Back ambience, A side (section 5.1) |
| 6 | BB | Back ambience, B side |

The names are role-neutral because the source can be L/R, Ls/Rs or any other pair. The Console labels them per template ("Front L", "Back centre", and so on).

Level conventions from `upmixer_spec.md` section 3.1 still hold, and BA/BB follow SA/SB. C can reach +3 dBFS, so the default C trim in templates is -3 dB.

When the centre mode is OFF, C is silent. When the surround mode is OFF, SA, SB, BA and BB are silent. A silent generated channel still claims its outputs (section 6.2), so a mode switch never makes matrix rows pop back in.

### 5.1 Back pair

The back pair derives a second ambience pair from the same steering, so stereo to 7.1 needs no second engine.

- **Second tap.** BA/BB read the existing Haas ring at a later tap `d_back`. They form the feed with the same block weights and per-sample steering gains as the sides. The steering control therefore leads the back audio by `d_back` rather than `d_side`. This is acceptable. The estimators run at 40 ms and longer, so a stationary panned source still cancels. The transient duck keeps at least about 86 % of its attenuation at the largest extra lead. A source that just stopped can trail in the backs for a few ms longer.
- **Timing.** A Schroeder allpass's main tap sits at its delay length. The sides therefore arrive at `d_side + 10 ms` and the backs at `d_back + 5.2 ms`. The user sets `back_offset_ms` (0 to 10, default 2), the back pair's arrival relative to the sides. The firmware computes `d_back = d_side + 4.8 ms + back_offset_ms` and clamps it to the ring (1023 samples, about 21.3 ms at 48 kHz) when coefficients are computed. It never refuses the SET. At the default 12 ms side delay the effective offset tops out near 4.5 ms. More back delay comes from the per-output delay on the back outputs, which carry only ambience.
- **Decorrelator.** Each back channel has its own Schroeder allpass with about a 5.2 ms delay (about 251 samples at 48 kHz) in a 256-sample ring, at the same gain as the sides. Exactly 5 ms is avoided because it is half the side length and the comb patterns would coincide. The signs are mirrored opposite to the sides (BA takes -g, BB takes +g), so that same-side SA and BA differ in their first tap. Both the length and the signs are validated on the `tools/upmix_proto` harness. If 5.2 ms decorrelates audibly worse in the surround band, the back ring grows to 512 at 7 ms.
- **Fade.** In ADAPTIVE mode the backs have their own fade envelope, started when `fill` reaches `d_back`, applied as one extra ramped multiply. `fill` counts on to `d_back`. Without this, the back audio would emerge with the side gain already open, a hard onset. In PASSIVE mode the activation ramp is applied before the ring write, so it is already baked into the ring and the second tap needs nothing.
- **Cost gating.** The back pair runs only while BA or BB is routed. When routing begins, its allpass state clears and its envelope restarts.
- No separate trim, band-limit or delay chain for the backs. Routing trim covers level and the band is shared with the sides.

## 6. Routing

### 6.1 Per generated channel

| Field | Type | Range | Default | Meaning |
|---|---|---|---|---|
| `out_mask` | u16 | bits 0 to 8 | 0 | Bit k routes this channel to output k. Bits above `NUM_OUTPUT_CHANNELS - 1` are refused with `VC_ERR_VALUE` |
| `trim_q1` | i8 | -80..+12 (-40.0 to +6.0 dB) | 0 | Gain in 0.5 dB steps, clamped |
| `flags` | u8 | bit 0 | 0 | Bit 0 = polarity invert. Other bits are reserved and refused with `VC_ERR_VALUE`, so a later host can detect firmware that lacks them |

A generated channel may go to several outputs. Several generated channels may land on one output and are summed there.

Per-destination gain was considered and rejected. A 7 x 9 gain table with phase is the crosspoint matrix by another name. Level differences between outputs belong in the existing per-output gain.

### 6.2 Claim semantics: replace the source pair, keep everything else

An output is **claimed** when any generated channel of the running profile has that output's bit set, while the engine is active or fading. A claimed output no longer receives the profile's source rows from the matrix. All other matrix contributions to that output stay. A profile with nothing routed does not run the engine; it starts with its first route. Examples:
- Stereo input, Front L output fed by row 0. Assign A' to it. The output now carries A' instead of L.
- 6-channel input, profile on Ls/Rs (inputs 4 and 5). The side outputs fed by rows 4 and 5 now carry A'/B'. Any other rows routed to those outputs remain.
- A sub output fed by rows 0 and 1 that is not assigned keeps raw L+R. That is the original bass and does not depend on centre steering, which makes it the correct sub feed. Assign nothing to the sub.

This is "replace", not "add". Adding A' to a matrix feed that already carries L would put the extracted centre back and double the level.

The matrix under a claim is the **parked mix**. When the engine parks or is disabled, claimed outputs return to the plain matrix feed. Templates must therefore leave the source-row crosspoints in place on fronts.

### 6.3 Processing

All of it runs on Core 0 after PASS 4 and before `siggen_render()`, so Core 1 only sees finished `buf_out`. No per-output latency is added and every output advances by the same `sample_count`.

The engine never materialises whole-packet generated buffers. It is split into two calls.
- `upmix_begin(n)` runs once per packet. It does exactly what the top of v1's `upmix_process_block()` does today (control update, fade and fill logic, ramp setup across n). The ramp cursors move from locals into the engine state (about 50 bytes).
- `upmix_render(chunk_n)` runs the per-sample loop for the next chunk of up to 32 samples into one shared 7 x 32 float chunk buffer. The injection step below then accumulates that chunk into the claimed outputs.

The control update still happens once per packet, and the per-sample state does not depend on where a chunk ends. So the engine's output is identical at any chunk size, and bit-identical to v1 for the five v1 channels. Generated-channel peaks for the status packet are the maximum across chunks.

1. **Snapshot once per packet.** Read the running profile's masks and the claim weights. An enabled output is claimed if any routed bit targets it and its claim weight is above 0 at block start or block end. Its omitted rows are the profile's source rows (one row if `src_a == src_b`).
2. **PASS 4 omission.** While building a claimed output's crosspoint list, skip the omitted rows. Store each skipped row's (source pointer, signed gain) in a per-output side list. The injection step uses this captured list and never re-reads `matrix_mixer`, which vendor SETs may change mid-packet.
3. **Injection, per chunk.** For each enabled output with a claim record (it can exist with an empty side list, for example a back output that the matrix feeds nothing):
   `dst[n] += sum over routed g of ( e[g][out][n] * gen[g][n] ) + ( 1 - w[out][n] ) * sum over skipped rows r of ( gain_r * src_r[n] )`
   - `e` is the slewed contribution gain (trim and polarity, times the fade).
   - When `w[out]` is exactly 1.0 at both block ends, the restore term is skipped. With unity trim and no other rows, the output is then bit-exact `gen`.
4. Disabled outputs are skipped. The matrix already zeroes them and they get no claim record.

### 6.4 Ramps

- Each contribution gain `e[g][out]` stores only its current value. Every sample it slews toward its target at a fixed rate of full scale per 10 ms (`UPMIX_FADE_MS`) and lands exactly on the target. The target is the trim and polarity if the channel is routed and the engine is active, otherwise 0. This covers trim edits, mask edits, invert changes and activation with no stored step and no edit queue, so a second edit during a fade simply retargets. A large trim step takes proportionally longer (a +6 dB step about 20 ms).
- The claim weight `w[out]` slews the same way, 0 to 1 when the output becomes claimed and 1 to 0 when it stops. Weights clamp to exactly 0.0 and 1.0 at the ends.
- **Disable fades out.** When the running profile is disabled, loses the win, or loses its last routed bit on an output, the engine keeps running on the last published coefficient set until the fade completes, then parks. This replaces v1's documented hard stop on disable.
- **Parks forced by the pipeline are hard.** Input source switches, active-count changes and rate changes already pass through `complete_pipeline_reset()`, which parks the engine (`upmix_hard_park()`); it starts again from a fade-in. Count and rate changes are also caught per packet.
- **No audio, no fades.** Fades advance per packet, so while no producer feeds the pipeline the main loop parks the engine hard. Nothing is playing, so nothing is heard, and the next start fades in cleanly instead of finishing a stale fade or replaying the Haas ring.

## 7. Host interface

All commands stay in page `FX` (0x08). The FX page revision increments.

### 7.1 Profile addressing

The existing config and per-parameter commands (0x80 and 0x90 to 0x9D) change address kind from `NONE` to `INDEX`, with `wValue` = profile (0 or 1). Every existing host already sends `wValue` 0, which keeps addressing profile 0. The handlers return `VC_ERR_ADDRESS` for a profile of 2 or more, because the dispatcher does not range-check `INDEX`. `FX.UPMIX_STATUS` (0x81) keeps address `NONE`, because there is one engine.

`commands.md` needs two edits:
- Section 3 gains a clause allowing a `NONE` command to gain an address in a later page revision, provided address 0 keeps its meaning.
- The FX page preamble ("none takes an address") is corrected.

### 7.2 New commands

| Command | Cmd | Dir | Address | Size | Notes |
|---|---|---|---|---|---|
| `FX.UPMIX_ROUTE` | 0x82 | G, S | `(profile << 8) \| g`, g 0 to 6 | 4 / 4 | {u16 out_mask, i8 trim_q1, u8 flags} |
| `FX.UPMIX_SOURCE_A` | 0x9E | G, S | profile | 4 / 4 | Parameter 14, f32, rounded. Out of range is `VC_ERR_VALUE` |
| `FX.UPMIX_SOURCE_B` | 0x9F | G, S | profile | 4 / 4 | Parameter 15 |
| `FX.UPMIX_ACTIVE_COUNTS` | 0xA0 | G, S | profile | 4 / 4 | Parameter 16, f32 holding the 4-bit mask. Bits above 3 are `VC_ERR_VALUE` |
| `FX.UPMIX_BACK_OFFSET` | 0xA1 | G, S | profile | 4 / 4 | Parameter 17, f32 ms, 0 to 10, clamped |

Parameter numbering continues the v1 rule (command = 0x90 + parameter number).

### 7.3 UpmixConfigPacket grows from 44 to 80 bytes

Bytes 0 to 43 are unchanged. Appended:

| Offset | Type | Field |
|---|---|---|
| 44 | u8 | src_a |
| 45 | u8 | src_b |
| 46 | u8 | active_counts |
| 47 | u8 | reserved (0) |
| 48 | f32 | back_offset_ms |
| 52 | 7 x {u16 out_mask, i8 trim_q1, u8 flags} | routes for g = 0 to 6 |

A 44-byte SET is still accepted (rule 8) and leaves bytes 44 to 79 unchanged. It does not reset them to defaults.

### 7.4 UpmixStatus grows from 16 to 32 bytes

`parked_reason` gains codes. 2 now means "no profile matches the active input count" (for a v1-style config this is exactly v1's "input not stereo"). 4 means "a source channel of the running profile is not present". 5 means "fading out". 6 means "the winning profile has nothing routed". `active` reads 1 while running and also while parked only for want of audio, when the profile would start with the next packet. The live gain fields report the running profile.

Appended:

| Offset | Type | Field |
|---|---|---|
| 16 | u8 | active_profile: running, fading or about to run (0, 1, or 0xFF for none) |
| 17 | u8 | Bit g set while generated channel g passed 0 dBFS before its trim within the last second. No read clears it, so several pollers see the same bits |
| 18 | u16 x 7 | Peak per generated channel, 32767 = 0 dBFS, saturating |

### 7.5 Notifications

Every new field of both profiles gets per-field `NOTIFY_EVT_PARAM_CHANGED` with its `offsetof(WireBulkParams, ...)` offset, through the shared config-to-packet encoder, so the notify shadow matches what bulk GET returns. Edits to the profile that is not running notify too. Any new SET path calls `upmix_notify_config(profile)`.

### 7.6 Control Surfaces

The six existing nouns edit profile 0. No new nouns in v2.

## 8. Persistence

- **Wire bulk.** Profile 0's 44-byte block stays at offset 5900. A new section is appended at the end of `WireBulkParams`, holding profile 0's 36-byte extension (bytes 44 to 79) followed by profile 1's full 80-byte packet (116 bytes). `WIRE_FORMAT_VERSION` takes the next free number at implementation time (V35, checked across all branches on 2026-11-22; V33 is held by the AC-3 branch). Bulk SET is strict version equality, so no wire migration is needed.
- **Preset slot.** The same 116 bytes are tail-appended. `SLOT_DATA_VERSION` takes the next free number (V42, checked the same way; V40 is held by the AC-3 branch), following the per-version CRC size table idiom.
- **Factory reset.** Both profiles at defaults, disabled, with no routes.

### 8.1 Migrating older presets

A slot older than the v2 version with the upmixer enabled loads into profile 0 with sources 0 and 1, `active_counts` = 0x01, default `back_offset_ms`, and routes derived from the matrix. Profile 1 loads disabled at defaults.

Route derivation, for each enabled output:
- Collect the enabled, non-zero crosspoints among rows 0 to 4.
- If exactly one row r is found, set the output bit on the generated channel r maps to (0 to A', 1 to B', 2 to C, 3 to SA, 4 to SB). Rows 3 and 4 are mapped even when the surround mode is OFF, so turning surrounds on later behaves as it did in v1.
- If two or more rows are found, the output stays unassigned on the matrix.
- Trim comes from the crosspoint gain, clamped to the trim range and rounded to 0.5 dB, and invert comes from its phase. If one row feeds several single-row outputs at different gains, the largest gain (and its phase) wins.

Crosspoints are left in place. Rows 2 to 4 are real inputs in multichannel mode, and rows 0 and 1 are the parked mix.

The lossy cases, to be stated in the release notes:
- A multi-row output such as a sub fed by rows 0 and 1 now receives raw L+R instead of v1's L'+R'. This is the more correct sub feed.
- A phantom fold such as row 0 plus row 2 on one output now receives raw L, which already contains the centre.
- Outputs fed by one row at different gains get one trim.

## 9. Use-case templates (Console)

Templates live in the Console. The firmware has no notion of them. Each template fills one profile. A "Stereo + 5.1" preset typically uses profile 0 for 2-channel input and profile 1 for 6/8-channel input.

| Template | Profile setup | Routing |
|---|---|---|
| Stereo to 3.0 (LCR) | src 0/1, counts 2ch, surround OFF | A' to FL, B' to FR, C to centre |
| Stereo to 5.1 | as above, surround ADAPTIVE | plus SA to SL, SB to SR. Sub left on the matrix (L+R) |
| Stereo to 7.1 | as 5.1 | plus BA to BL, BB to BR. Extra back distance via per-output delay |
| 5.1 to 7.1 (backs from surrounds) | src = input Ls/Rs, counts 6ch and 8ch | A'/B' to side L/R; C to both backs (back centre, EX style), or SA/SB to the backs |
| Centre / mono fill | src 0/1, surround OFF, width 100 | C alone to the fill output |
| Dialogue lift | src 0/1, width 100, presence up | A' to FL, B' to FR, C to both FL and FR at a negative trim |
| Vocal reduction | src 0/1, width 0, strength 100 | A' to FL, B' to FR, C unrouted |

The Console should mark matrix cells whose row is claimed on that output ("taken by Upmixer"), not hide them. It warns when the two profiles' `active_counts` overlap.

## 10. Stereo width: a separate module

Widening was discussed as an upmixer mode and rejected for the upmixer. In mid/side terms, M = (L+R)/2 and S = (L-R)/2. A centred source has S = 0, so plain widening never moves it, and the adaptive centre detector adds nothing. The detector also cannot separate a hard-panned source from ambience (section 12 of `upmixer_spec.md`), so "widen only the ambience" is not something this engine can do.

Width will be its own per-output-pair stage, specified separately:
- Algorithm: mid/side width with a Blumlein/Gerzon-style shuffler low shelf on the side signal (width 0 to 200 %, bass width, bass corner).
- Post-matrix, pre-EQ, immediately before crossfeed (width treats the programme, crossfeed models the transducer).
- Both platforms, following the crossfeed idiom (pair mask, per-pair state, process call on the core that owns the pair, zero latency).
- Not mutually exclusive with crossfeed in firmware. The Console warns.
- Its spec must state the level cost. At 200 % width a hard-left source gives +3.5 dB on L and anti-phase crosstalk at -6 dB on R.

## 11. Costs

- **RAM.** Growth over v1 on RP2350 (v1 engine state measured at 12,444 B).

  | Item | Bytes |
  |---|---|
  | Back allpass ring 2 x 256 floats, back envelope and state, latched running-profile data | about 2.1 KB |
  | Shared chunk buffer (7 x 32 floats) and ramp cursors | about 0.95 KB |
  | Routing gains, current value only (7 x 9), claim weights and side lists | about 0.45 KB |
  | Profile 1 config and the new fields of profile 0 | about 0.15 KB |
  | RAM-resident code (routing, injection, begin/render split, back pair) | about 2 KB |
  | Bulk and slot buffers (116 B section; more if a notify shadow duplicates it) | about 0.2 to 0.6 KB |
  | **Total** | **about 6 KB** |

  The first two-instance estimate was about 23 KB. Rejected along the way: whole-packet generated buffers (replaced by the chunked render), a second engine (replaced by the back pair and profiles), int16 or fp16 ring storage (noise floor on content that can exceed 0 dBFS), a shared delay pool (a cross-setting UX rule), and dropping the back decorrelator (the backs would become a delayed copy of the sides).
- **CPU.** About 160 float ops per sample with both engines ADAPTIVE (the v1 figure, never measured on hardware), plus about 15 to 20 per sample while the back pair is routed, plus the injection pass (a few multiply-adds per routed contribution per sample). One hardware measurement goes into this spec. It is no longer a gate.

## 12. Invariants

- No per-output latency is introduced. Every output advances by the same `sample_count` with the same sample index.
- SA and SB share one Haas tap. BA and BB share one Haas tap. Templates that route a pair to a pair of outputs get matched timing.
- Source buffers are never written by the upmixer.
- A claimed output is bit-exact in steady state when its claim is a single generated channel at unity trim with no other rows.
- A parked engine claims nothing. Its outputs carry the plain matrix mix.
- At most one profile runs. Profile changes are sequential fades, never a mix of two profiles.
