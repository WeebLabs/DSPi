# Upmixer v2 Implementation Plan

*2026-11-22 (revised the same day for the one-engine design). Implements `upmixer_v2_spec.md`. Phases 0 to 5 done. RP2350 only, except phase 7.*

Each phase builds on both platforms, keeps the tree releasable, and ends with the `tools/upmix_proto` harness passing where the engine is touched. Architecture doc updates follow `CLAUDE.md`.

## Phase 0: Prerequisites (done 2026-11-22)

- The engine is committed. The adaptive surround rework and the `tools/upmix_proto` harness landed in 3bc9b93 on `release/v1.1.7`, which matches `origin/release/v1.1.7` with no uncommitted changes to tracked files (checked at 806a61b).
- Next free versions, checked across every local and remote branch: `WIRE_FORMAT_VERSION` 35 and `SLOT_DATA_VERSION` 42. Current is wire 34 and slot 41 on `release/v1.1.7`, and `ac3-dts-decode` holds wire 33 and slot 40. The directory version (26) is unaffected.
- Collision to watch: the locked worktree `agent-ae9cd744540d7c58e` (wirecap rig, uncommitted) edits `commands.md`, `dspi_commands.json` and `scripts/gen_vendor_protocol.py`. Phase 5 touches the same generated files, so land or rebase that work before phase 5.
- Command definitions live in `firmware/DSPi/vendor/pages/fx.def`. The `commands.md` tables and `dspi_commands.json` are generated from it by `scripts/gen_vendor_protocol.py`, so phase 5 edits `fx.def` and regenerates rather than hand-editing the tables.

## Phase 1: Engine split (no behaviour change; done 2026-11-22, a6144b1)

Result: `split_check.py` passes 1,500 runs bit for bit against HEAD, chunk sizes 1, 7, 32 and the whole packet (and reports mismatches against the pre-rework engine at `3bc9b93~1`, so the check does bite). The full `harness.py` passes too: zero difference from HEAD, zero from the Python model, no failing acceptance cell. Both platforms build. RP2350 `.bss` +304 B and `.data` -472 B, because `upmix_begin()` is compiled `-Os` (it was 2 KB of RAM text at `-O3` and pushed `.data` 360 B over its budget). `upmix_render()` writes to caller pointers, so the bridge stages only A'/B'. The bridge sits inside `upmix_process_block()`, so `audio_pipeline.c` did not change. Hardware (RP2350, 307.2 MHz, 48-sample packets, cycle counter around the upmix call): 19,374 to 20,409 cycles per packet adaptive, 13,333 to 14,482 passive, 4,176 to 5,076 centre only. Against the single function before the split that adds about 0.3 % of core 0 while the upmixer runs (the whole upmixer is about 6.5 %). Phase 3 should recover the bridge share.

- Split `upmix_process_block()` into `upmix_begin(n)` (once per packet: control update, fades, ramp setup) and `upmix_render(chunk_n)` (per-sample loop into a shared chunk buffer). Move the ramp cursors from locals into engine state.
- Sources become read-only pointers, and the engine no longer writes them in place.
- Keep v1 delivery as a temporary bridge inside this phase only. Still call the engine at the v1 hook and copy each chunk to rows 0 to 4, so behaviour and wire are identical. Phase 3 deletes the bridge in the same release.
- Harness: extend `tools/upmix_proto/harness` to the new calls. Every v1 acceptance case must stay bit-identical at chunk sizes 1, 7, 32 and the whole packet.

## Phase 2: Profiles, sources and activation (done 2026-11-22, 9293bc8)

- `upmix_config` became `upmix_profiles[2]`, each with `src_a`, `src_b` and `active_counts` (defaults 0, 1, 0x01; both disabled), and each with its own published coefficient set (`current_upmix_coeffs[]`, double-buffered). `upmix_profile_defaults()` replaces the copied default blocks in preset load and factory reset.
- `upmix_select(n)` returns the lowest published profile whose `active_counts` holds the input count. Coefficient sets carry their profile index, and `upmix_begin()` resets the engine when a set from a different profile arrives, so the new profile fades in from reset.
- `parked_reason` 2 now means no profile serves the input count, and 4 means a source channel is absent. Reason 5 (fading out) arrives with phase 3.
- Scope moved to phase 3, because the row bridge cannot carry it: running above stereo (the bridge writes rows 2..4, which hold real inputs there, so `upmix_bridge_coeffs()` keeps the stereo gate) and the fade-out on a live profile change (that fade is the claim weights). Until then a profile change is a hard reset plus fade-in, as an input-count change already is.
- Not reachable from a host until phase 5: profile 1 and the new fields have no commands and are not persisted. Every vendor, bulk, preset and Control Surfaces path uses profile 0, and a preset load resets the rest to defaults. Behaviour with the defaults is v1's.
- Result: `split_check.py` adds profile selection (12 cases: disjoint and overlapping counts, lowest-wins, absent sources, source clamp, rate gate, park reasons) and a profile-switch test (the output after a switch equals the new profile run alone from reset; a mutant without the reset fails it). 1,500 engine runs stay bit-identical to HEAD and the full `harness.py` passes. RP2350 `.data` 99,600 to 99,840 B, `.bss` +268 B (second coefficient double buffer). Hardware: core 0 load unchanged from phase 1.

## Phase 3: Module routing and claim semantics (done 2026-11-22, 17ede0d)

Result: the bridge is gone. `upmix_claim()` runs before the matrix and holds the running profile on a copied coefficient set, so disable, a lost win and a removed last route fade out before the engine parks; count and rate changes park hard. PASS 4 omits the source rows on claimed outputs into a side list, and `upmix_run()` renders and injects after the matrix, before the generator. Pulled forward from phase 5 so hardware could exercise the routing: `FX.UPMIX_ROUTE` (0x82, no notification or persistence yet) and the 32-byte status, FX revision 2. BA/BB have route and status slots but render silence until phase 4. Host: `split_check.py` adds a mini pipeline against an independent float64 model of the claim and fade rules (three scenes, packet sizes 7 to 192): claims agree on every packet, worst error under 1e-7, settled unity claims bit-exact, and six seeded faults each fail it. The engine stays bit-identical to 9293bc8 over 1,500 runs and the full `harness.py` passes. Builds: RP2350 `.data` 99,840 to 101,000 B, so the RP2350 budget rose 2 KB to 102,400 B (covers phase 4); `.bss` +1,324 B; RP2040 unchanged. Hardware (RP2350 loopback build, 48 and 44.1 kHz): A'/B' routed over L/R are bit-exact with the centre off, a mono tone gives A' -12.04 dB and C +3.01 dB as computed, the matrix is bit-exact again after a disable, and route GET/SET, clamps and refusals behave. Core 0 with a 5.1 routing: adaptive 12.2 %, passive 11.0 %, off 5.0 % (phase 2 without routes: 12, 10, 5). A v1 preset that used rows 2..4 gives no upmix output until the phase 5 migration. Fable audit the same day (no blockers): the engine now parks hard while no producer feeds the pipeline and on every pipeline reset, a source edit fades like a profile change, a profile with nothing routed does not run, clip bits hold for a second instead of clearing on read (protocol rule 2), reserved route flags are refused, and the host check gained a two-row side list, rate and source-edit cases, time-placed edits, an independent fade and trim model, status checks and `harness/mutants.py` (11 seeded faults, all caught).

**Release gate.** No beta is cut between phase 3 and phase 5. Until phase 5, routes have no change notification, no persistence, and a preset load or factory reset clears them, and FX revision 2 changes shape again in phase 5. Check the ESP32 front panel's `FX.UPMIX_STATUS` request length before the release.


- Add the per-generated-channel route fields (mask, trim, flags) to each profile.
- PASS 4: build the per-packet claim snapshot from the running profile, omit its source rows from claimed outputs, and capture the omitted (source, gain) pairs in a per-output side list.
- Move the engine call after PASS 4. Call `upmix_begin()` once, then `upmix_render()` and injection chunk by chunk, all before `siggen_render()`. Injection covers slewed contribution gains (current value only), per-output claim weights, the restore-term skip at w = 1, and the disabled-output skip.
- Disable fade-out. Hold the last coefficient set until the claim weights reach 0, then park. Pipeline-forced parks stay hard.
- Remove the phase 1 bridge, the row-sharing code path, `n_matrix_sources` growth, derived-row input metering and RTA input taps. Add generated-channel peaks, clip bits and `active_profile` to status.
- Host-side test: a C model of the injection pass against a float64 reference. Cover single and multiple claims, `src_a == src_b`, a mask edit mid-fade, a second edit during a fade, a crosspoint SET between snapshot and injection, and a live profile change. Check steady-state bit-exactness for unity single claims.

## Phase 4: Back pair (done 2026-11-22, fc5cb65)

Result: as specified, with the 5.23 ms / 256-sample back allpass kept. `harness/back_check.py`: chunk invariance on all seven channels; arrival 0.02 / 2.02 / 4.54 ms after the sides at offsets 0 / 2 / 10 ms (clamped tap); hard-left leak below -80 dB on SA, SB, BA and BB; back onset over bed +15.1 dB against +14.0 dB at the side; side-to-back peak correlation 0.63 against 0.80 with no back allpass. The mirrored signs and the 7 ms / 512 fallback both measure 0.63 too, so the signs are neutral on this metric and the fallback is not needed. A fade started at the side tap gives a -9 dB first 5 ms against -19 dB, and the check catches it. `split_check.py` adds a stereo to 7.1 scene (back routes removed mid-run, a +6 dB back fade-out with no back route left, restart, mode switch, offset change), and `mutants.py` adds a back that stops before its fade (12 caught). Builds: RP2350 `.data` 101,760 B (640 B left), `.bss` +2,128 B. Hardware: BA 2.02 ms (48 kHz) and 2.04 ms (44.1 kHz) after SA, peak correlation 0.63; core 0 with 7.1 routing 13 % adaptive, 11 % passive. `back_offset_ms` has no command until phase 5.

- Second Haas tap at `d_back = d_side + 4.8 ms + back_offset_ms`, clamped to the ring at compute time.
- Back allpass pair (256 ring, about 5.2 ms, signs mirrored against the sides), own ADAPTIVE fade envelope keyed to `fill >= d_back`, with `fill` counting on to `d_back`.
- Runs only while BA or BB is routed. Clear and restart on routing start.
- Harness: back-tap hard-pan leak and snare onset compared with the side tap. Side-to-back decorrelation in the 300 Hz to 7 kHz band at 5.2 ms, with fallback to a 512 ring at 7 ms. Validate the mirrored signs.

## Phase 5: Host interface and persistence (done 2026-11-22)

Result: as planned, with one change. `VC_SET_MAX` is 64 bytes (reported in `SYS.PROTOCOL_INFO`, sizing every transport's receive buffer) and rule 7 needs a GET to be a valid SET, so the 80-byte packet cannot be one command. `FX.UPMIX_CONFIG` stays 44 bytes both ways and the new `FX.UPMIX_CONFIG_EXT` (0x83) carries bytes 44 to 79. Wire V35 (6252 bytes, section at 6136) and slot V42 (116-byte tail) carry the 80-byte image of both profiles; `upmix_packet_to_config()` is the one sanitizing decoder for bulk, flash and vendor writes, and `upmix_notify_config(p)` notifies every field including routes. `upmix_routes_from_v1()` migrates V33 to V41 slots. Host: `split_check.py` adds 12 migration and codec cases (5.1, LCR with inverted copies, phantom fold, sub on two rows, clamps, muted and disabled crosspoints, round trip, sanitizing, v1-only decode). Hardware (loopback build): all nine upmixer tests pass, including new profile-addressing, `UPMIX_CONFIG_EXT` and refusal checks; a whole-struct bulk GET and SET round-trips profile 1 and restores byte for byte; a 5.1 preset saved as slot V41 on the phase 4 build loads on V42 with exactly the expected routes, and both profiles survive a V42 save and load. The chunked bulk tests could not run because DSPi Console held the chunk session. Builds: `.bss` +232 B on both platforms (bulk buffer and notify shadow), no RAM code. `upmixer_spec.md` rewritten for v2. Fable audit the same day (two rounds, no blockers): every pre-V42 slot now migrates whether the upmixer was on or off; the decoder works by parts (`UPMIX_PKT_HEAD/TAIL/ALL`), so an EXT SET no longer re-quantizes presence, and it replaces non-finite floats with defaults (the slot path for profile 0 now goes through it too); the legacy map pins `UPMIX_SET/GET_CONFIG` to address 0; stale bulk-size statements across the docs and the generator are fixed. New tests: a flash-tier `presets/upmix_profiles_persist`, notify offsets checked on the host against the real `bulk_params.h`, and more migration and codec cases.

**Before the beta.** Run `paged_store` (the chunked bulk path, which Windows uses) with DSPi Console closed. Recapture the golden protocol reference on the phase 6 pass (its baseline blob is V32, so `compare` cannot run until then). Check the ESP32 front panel for an exact `FX.UPMIX_STATUS` length of 16 or an exact 6136-byte bulk read. Ship the matching Console, since bulk SET requires V35.

- Commands, in `vendor/pages/fx.def` then regenerated: change the address kind of 0x80 and 0x90 to 0x9D to `INDEX` (profile) with handler range checks. 0x81 stays `NONE`. Add 0x9E to 0xA1. Grow `UpmixConfigPacket` to 80 bytes (a 44-byte SET keeps the tail). 0x82 `UPMIX_ROUTE`, the 32-byte `UpmixStatus` and the FX revision bump to 2 landed in phase 3; the revision stays 2 for the rest of this release.
- Notifications for every new field of both profiles via the shared encoder, routes included (0x82 has none until then).
- Wire bulk: append the 116-byte section and bump `WIRE_FORMAT_VERSION`.
- Slot: tail-append 116 bytes, bump `SLOT_DATA_VERSION`, extend the CRC size table, and add the legacy route migration of spec 8.1 as one function used by slot load.
- Factory reset defaults for both profiles.
- Docs: `commands.md` (rule 3 clause, FX preamble, command rows), `upmixer_spec.md` rewritten from the v2 spec, and `current_architecture.md` (pipeline order, memory layout, vendor command table).
- Test: save on v1 firmware, load on v2, and compare the migrated routes with the expected table for representative presets (LCR, 5.1, surrounds only, sub on L+R, phantom fold).

## Phase 6: Hardware bench pass

- Stereo to 5.1, stereo to 7.1 (back pair), 5.1 to 7.1 (6-channel USB), LCR, centre fill, dialogue lift, and a two-profile preset switching between stereo and 6-channel sources.
- Inter-slot alignment check with the wirecap rig across enable, disable, mask edits, profile changes, source switches and preset loads.
- Click check on every fade path, including the back-pair start. Listen at stream start too: the main loop parks the engine while `pipeline_producer_is_streaming()` is false, so blocks must not flow audibly in that window (USB before `sync_started`, I2S before RUNNING). Confirm pipeline-forced parks only click where a pipeline reset already does.
- Preset migration on real v1 presets.
- CPU meter measurement of the engine with and without the back pair, recorded in the spec. Measure RAM growth from the ELF against the spec's 6 KB estimate.

## Phase 7: Stereo width stage (separate module, both platforms)

- Write `stereo_width_spec.md` first: mid/side width with a shuffler low shelf on the side signal, pair mask, ranges, level cost, position before crossfeed.
- Implement following the crossfeed idiom in float (RP2350) and Q28 (RP2040), with its own wire and slot section.
- Not coupled to the upmixer and not mutually exclusive with crossfeed in firmware.

## Cross-repo follow-ups

- **DSPi Console.** Profile tabs with their input-count selection, per-generated-channel output pickers with trim and invert, the template set of spec section 9, claimed-cell marking in the matrix, an overlap warning between profiles, generated meters from status, and removal of the row-sharing labels. The Console must ship with the matching firmware, because an old Console routing rows 2 to 4 on v2 firmware gets silence.
- **ESP32 front panel.** Check that its upmix GETs request the length they know (44 and 16) so the longer replies truncate cleanly.
- **Control Surfaces.** No change. Nouns edit profile 0.

## Risks

- Claim bookkeeping and profile changes are the subtlest code in the change. Both get host tests before hardware.
- The back pair's decorrelation at 5.2 ms is unproven until the harness runs.
- Migration is lossy for multi-row and multi-gain outputs. The cases are listed in spec 8.1 and go in the release notes.
