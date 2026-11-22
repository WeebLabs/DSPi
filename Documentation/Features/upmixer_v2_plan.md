# Upmixer v2 Implementation Plan

*2026-11-22 (revised the same day for the one-engine design). Implements `upmixer_v2_spec.md`. Not started. RP2350 only, except phase 7.*

Each phase builds on both platforms, keeps the tree releasable, and ends with the `tools/upmix_proto` harness passing where the engine is touched. Architecture doc updates follow `CLAUDE.md`.

## Phase 0: Prerequisites (done 2026-11-22)

- The engine is committed. The adaptive surround rework and the `tools/upmix_proto` harness landed in 3bc9b93 on `release/v1.1.7`, which matches `origin/release/v1.1.7` with no uncommitted changes to tracked files (checked at 806a61b).
- Next free versions, checked across every local and remote branch: `WIRE_FORMAT_VERSION` 35 and `SLOT_DATA_VERSION` 42. Current is wire 34 and slot 41 on `release/v1.1.7`, and `ac3-dts-decode` holds wire 33 and slot 40. The directory version (26) is unaffected.
- Collision to watch: the locked worktree `agent-ae9cd744540d7c58e` (wirecap rig, uncommitted) edits `commands.md`, `dspi_commands.json` and `scripts/gen_vendor_protocol.py`. Phase 5 touches the same generated files, so land or rebase that work before phase 5.
- Command definitions live in `firmware/DSPi/vendor/pages/fx.def`. The `commands.md` tables and `dspi_commands.json` are generated from it by `scripts/gen_vendor_protocol.py`, so phase 5 edits `fx.def` and regenerates rather than hand-editing the tables.

## Phase 1: Engine split (no behaviour change; done 2026-11-22, a6144b1)

Result: `split_check.py` passes 1,500 runs bit for bit against HEAD, chunk sizes 1, 7, 32 and the whole packet (and reports mismatches against the pre-rework engine at `3bc9b93~1`, so the check does bite). The full `harness.py` passes too: zero difference from HEAD, zero from the Python model, no failing acceptance cell. Both platforms build. RP2350 `.bss` +304 B and `.data` -472 B, because `upmix_begin()` is compiled `-Os` (it was 2 KB of RAM text at `-O3` and pushed `.data` 360 B over its budget). `upmix_render()` writes to caller pointers, so the bridge stages only A'/B'. The bridge sits inside `upmix_process_block()`, so `audio_pipeline.c` did not change. Hardware (RP2350, 307.2 MHz, 48-sample packets, cycle counter around the upmix call): 19,374 to 20,409 cycles per packet adaptive, 13,333 to 14,482 passive, 4,176 to 5,076 centre only, about 0.3 % of core 0 while the upmixer runs. Phase 3 should recover the bridge share.

- Split `upmix_process_block()` into `upmix_begin(n)` (once per packet: control update, fades, ramp setup) and `upmix_render(chunk_n)` (per-sample loop into a shared chunk buffer). Move the ramp cursors from locals into engine state.
- Sources become read-only pointers, and the engine no longer writes them in place.
- Keep v1 delivery as a temporary bridge inside this phase only. Still call the engine at the v1 hook and copy each chunk to rows 0 to 4, so behaviour and wire are identical. Phase 3 deletes the bridge in the same release.
- Harness: extend `tools/upmix_proto/harness` to the new calls. Every v1 acceptance case must stay bit-identical at chunk sizes 1, 7, 32 and the whole packet.

## Phase 2: Profiles, sources and activation (done 2026-11-22)

- `upmix_config` became `upmix_profiles[2]`, each with `src_a`, `src_b` and `active_counts` (defaults 0, 1, 0x01; both disabled), and each with its own published coefficient set (`current_upmix_coeffs[]`, double-buffered). `upmix_profile_defaults()` replaces the copied default blocks in preset load and factory reset.
- `upmix_select(n)` returns the lowest published profile whose `active_counts` holds the input count. Coefficient sets carry their profile index, and `upmix_begin()` resets the engine when a set from a different profile arrives, so the new profile fades in from reset.
- `parked_reason` 2 now means no profile serves the input count, and 4 means a source channel is absent. Reason 5 (fading out) arrives with phase 3.
- Scope moved to phase 3, because the row bridge cannot carry it: running above stereo (the bridge writes rows 2..4, which hold real inputs there, so `upmix_bridge_coeffs()` keeps the stereo gate) and the fade-out on a live profile change (that fade is the claim weights). Until then a profile change is a hard reset plus fade-in, as an input-count change already is.
- Not reachable from a host until phase 5: profile 1 and the new fields have no commands and are not persisted. Every vendor, bulk, preset and Control Surfaces path uses profile 0, and a preset load resets the rest to defaults. Behaviour with the defaults is v1's.
- Result: `split_check.py` adds profile selection (12 cases: disjoint and overlapping counts, lowest-wins, absent sources, source clamp, rate gate, park reasons) and a profile-switch test (the output after a switch equals the new profile run alone from reset; a mutant without the reset fails it). 1,500 engine runs stay bit-identical to HEAD and the full `harness.py` passes. RP2350 `.data` 99,600 to 99,840 B, `.bss` +268 B (second coefficient double buffer). Hardware: core 0 load unchanged from phase 1.

## Phase 3: Module routing and claim semantics

- Add the per-generated-channel route fields (mask, trim, flags) to each profile.
- PASS 4: build the per-packet claim snapshot from the running profile, omit its source rows from claimed outputs, and capture the omitted (source, gain) pairs in a per-output side list.
- Move the engine call after PASS 4. Call `upmix_begin()` once, then `upmix_render()` and injection chunk by chunk, all before `siggen_render()`. Injection covers slewed contribution gains (current value only), per-output claim weights, the restore-term skip at w = 1, and the disabled-output skip.
- Disable fade-out. Hold the last coefficient set until the claim weights reach 0, then park. Pipeline-forced parks stay hard.
- Remove the phase 1 bridge, the row-sharing code path, `n_matrix_sources` growth, derived-row input metering and RTA input taps. Add generated-channel peaks, clip bits and `active_profile` to status.
- Host-side test: a C model of the injection pass against a float64 reference. Cover single and multiple claims, `src_a == src_b`, a mask edit mid-fade, a second edit during a fade, a crosspoint SET between snapshot and injection, and a live profile change. Check steady-state bit-exactness for unity single claims.

## Phase 4: Back pair

- Second Haas tap at `d_back = d_side + 4.8 ms + back_offset_ms`, clamped to the ring at compute time.
- Back allpass pair (256 ring, about 5.2 ms, signs mirrored against the sides), own ADAPTIVE fade envelope keyed to `fill >= d_back`, with `fill` counting on to `d_back`.
- Runs only while BA or BB is routed. Clear and restart on routing start.
- Harness: back-tap hard-pan leak and snare onset compared with the side tap. Side-to-back decorrelation in the 300 Hz to 7 kHz band at 5.2 ms, with fallback to a 512 ring at 7 ms. Validate the mirrored signs.

## Phase 5: Host interface and persistence

- Commands, in `vendor/pages/fx.def` then regenerated: change the address kind of 0x80 and 0x90 to 0x9D to `INDEX` (profile) with handler range checks. 0x81 stays `NONE`. Add 0x82 `UPMIX_ROUTE` and 0x9E to 0xA1. Grow `UpmixConfigPacket` to 80 bytes (a 44-byte SET keeps the tail) and `UpmixStatus` to 32. Bump the FX page revision.
- Notifications for every new field of both profiles via the shared encoder.
- Wire bulk: append the 116-byte section and bump `WIRE_FORMAT_VERSION`.
- Slot: tail-append 116 bytes, bump `SLOT_DATA_VERSION`, extend the CRC size table, and add the legacy route migration of spec 8.1 as one function used by slot load.
- Factory reset defaults for both profiles.
- Docs: `commands.md` (rule 3 clause, FX preamble, command rows), `upmixer_spec.md` rewritten from the v2 spec, and `current_architecture.md` (pipeline order, memory layout, vendor command table).
- Test: save on v1 firmware, load on v2, and compare the migrated routes with the expected table for representative presets (LCR, 5.1, surrounds only, sub on L+R, phantom fold).

## Phase 6: Hardware bench pass

- Stereo to 5.1, stereo to 7.1 (back pair), 5.1 to 7.1 (6-channel USB), LCR, centre fill, dialogue lift, and a two-profile preset switching between stereo and 6-channel sources.
- Inter-slot alignment check with the wirecap rig across enable, disable, mask edits, profile changes, source switches and preset loads.
- Click check on every fade path, including the back-pair start. Confirm pipeline-forced parks only click where a pipeline reset already does.
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
