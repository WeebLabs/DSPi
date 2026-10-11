# Merge Plan: spdif_rx_overhaul into release/v1.1.6

*Written 2026-10-10. Status: plan only, nothing merged.*

## TL;DR

The overhaul branch carries three upgrades. The first is the soft VCXO: every output clock follows the input by trimming the system PLL by parts per million, instead of nudging each output's divider. The second is output rings: each output slot plays from a free-running DMA ring with no interrupts, instead of a pool of buffers fed by an interrupt per buffer. The third is a better SPDIF input rate estimator with word-by-word delivery. Producer pools were not removed. The pipeline still writes into producer pools, and the rings are filled from those.

The textual merge is small: 13 files conflict, mostly in a handful of short hunks. The real work is semantic. Since the overhaul was written in July, release/v1.1.6 rebuilt the output lifecycle (pin restores, type switches, synchronized starts, silent flash writes, divider alignment) on top of the pool design. That code compiles against things the overhaul deletes, or quietly assumes interrupts and pools that no longer exist. The plan below keeps release's lifecycle and correctness fixes and swaps their plumbing to rings and the VCXO. Release's divider-alignment machinery is deleted outright, because the VCXO never writes output dividers.

It fits in RAM without removing producer pools, but only just on RP2040, so the ring store must stop paying alignment padding. Producer-pool removal is the natural next step and is planned as its own phase after the merged build is proven on hardware.

Five pre-existing overhaul defects were found during this review and get fixed during the merge. They are a sub-frame SPDIF restart skew, an SOF interrupt race with ring reset, mis-framed SPDIF silence when the ring laps during a stall, ADAT offset after a stall on non-USB inputs, and loopback capturing dropped blocks. The overhaul's RP2040 ring path has never run on hardware.

## Starting state

| | release/v1.1.6 | spdif_rx_overhaul |
|---|---|---|
| Commits since merge base 9f2e8ae | 90 | 14 |
| Output path | producer pools, consumer pools, output DMA IRQs | producer pools, per-slot DMA rings, no output IRQs |
| Clock tracking | PIO divider servo (SPDIF, I2S, ADAT out, MCK) | soft VCXO, sys PLL ppm trim, all outputs together |
| Flash writes | selective NVIC blackout keeps output DMA IRQs alive (285b89d) | full blackout plus full-ring silence prestamp |
| Free heap, RP2040 | 49,936 B | 30,728 B |
| Free heap, RP2350 | 94,180 B | 63,640 B |

Free heap is measured as `__HeapLimit - end` before runtime allocation. Producer pools are allocated from that heap at boot (about 12.8 KB per slot, so about 25 KB on RP2040 and 51 KB on RP2350).

The overhaul's ring store costs about 5 KB (RP2040) and 10 KB (RP2350) more than the consumer pools it replaces. Most of the remaining heap loss on the overhaul branch is alignment padding. `slot_ring_store` must be aligned to its 16 KB size for DMA ring wrap on both platforms, and the linker leaves a gap of about 8 KB in front of it plus further gaps behind other aligned objects.

The 4-way merge estimate for RP2040 is about 36 KB free before producer pools, so about 11 KB after them. That is workable but must be measured, not assumed.

Not in scope: the selectable system clock (307.2/384/480 MHz) lives on the `overclock` branch, not on release. Phase 2g makes the VCXO ready for it so it does not become debt.

## Resolution rules

These rules decide every conflict and every semantically affected auto-merge.

1. **Release owns the lifecycle.** `process_type_switches`, `apply_restored_outputs`, `process_pin_changes`, MCK policy, pin release-before-claim, the fade and DAC-mute ordering, and the full-path flash completion (773a453) all come from release.
2. **The overhaul owns the mechanism.** Ring libraries, group admission, idle pump, underrun re-anchor, frames-native counters, `start_output_slots_in_sync` (1490a8e), `teardown_output_slot`, `drain_and_disable_outputs`, input servo, soft VCXO, SPDIF RX estimator, I2S RX ENDLESS rings and USB feedback Loop B all come from the overhaul.
3. **Nothing ever trims an output divider.** Every output divider is nominal at all times. Any release code whose purpose was to replay, align or restore a trimmed divider is deleted, not adapted.
4. **Every output side effect sits behind admission.** SPDIF and I2S ring writes, both PDM push loops, the ADAT mirror push and the loopback push all happen only for admitted blocks. A block is either on every output or on none.
5. **No release fix is dropped silently.** Each one in the table below gets either a ring-world counterpart or a written reason it no longer applies.

## Phase 0: Preparation

1. Create an integration branch from release/v1.1.6 (proposed name `merge/spdif-rx-overhaul`). Nothing touches release/v1.1.6 until the end.
2. Record the baseline on release/v1.1.6 HEAD. That means clean builds of both platforms, `arm-none-eabi-size`, free heap, `scripts/check_ram_placement.py`, and a loopback suite run at 48 and 44.1 kHz. Every later phase is compared against this.

## Phase 1: The merge commit

Run a real `git merge` so the overhaul's history stays reachable (its commit hashes are referenced in docs and notes). The merge commit resolves every conflict and every compile error on both platforms. It does not yet attempt the semantic fixes of Phase 2, which go in as separate reviewable commits.

| File | Resolution |
|---|---|
| `firmware/DSPi/CMakeLists.txt` | Union. Keep release's RTA sources and add `soft_vcxo.c/h` and `clock_diag.c/h`. |
| `scripts/check_ram_placement.py` | Keep release's budgets for now. Re-measure in 2i. `FLASH_WINDOW_ROOTS` is rewritten in 2a, because both root symbols are deleted by the overhaul and the check hard-fails. |
| `adat_output.c/.h` | Take the overhaul (no `adat_output_servo_divider`). Keep release's `adat_output_rebaseline_starvations` declaration until 2b replaces it. |
| `audio_pipeline.c` | Keep release's `rta_tap` calls. Put both PDM push loops inside `if (rings_admit)`. The tap observes the DSP output and runs regardless of admission. |
| `i2s_input.c`, `input_servo.c/.h` | Take the overhaul. Delete `i2s_slave_reapply_clock_servo`, `input_servo_reapply`, `write_slot_dividers` and `write_mck_divider`. |
| `pdm_generator.c` | Union. Keep release's pin getters, `pdm_pin_needs_move`, `pdm_hw_is_active` and the hold handshake, and add the overhaul's `pdm_ring_free_slots`. |
| `main.c` | Release's lifecycle functions with ring plumbing swapped in (details below). The main loop calls both `rta_service()` and `output_rings_idle_pump()`. |
| `audio_spdif.c`, `audio_i2s_multi.c` and headers | Overhaul ring core plus the release APIs ported onto it (details below). |
| `Documentation/current_architecture.md` | Union of both sides in the merge commit, then rewritten for the merged design in 2j. |

**main.c changes inside the merge commit:**
- Delete `align_output_clocks()` and both calls (in `complete_pipeline_reset` and `enable_outputs_in_sync`).
- Delete `restore_nominal_output_clocks()` and its callers' divider work. Its `input_servo_reset()` call must **not** survive anywhere on the reset path, because under the VCXO it parks the PLL and wipes the integrator mid-lock on every preset load, save, type switch and pin change. Parking belongs to the input stop paths (`adat_input_stop`, `i2s_slave_disarm`, SPDIF stop), which already do it.
- Real rate changes keep their nominal divider recompute through `perform_rate_change`.
- Replace release's per-slot quiesce block in `process_type_switches` with `set_enabled(false)`. A raw `dma_channel_abort` is wrong on RP2040, where the reload channel re-triggers the data channel.
- Add `.ring_base = slot_ring_store[i]` and, on RP2040, `.dma_reload_channel = OUTPUT_SLOT_RELOAD_DMA_BASE + i` to the setup configs. Drop `slot_consumer_pools` and the consumer-pool arguments.
- Replace every `inst->consumer_pool` "slot is set up" check (43 sites) with `inst->ring`.
- Drop the overhaul's boot pin block. It re-enables a moved SPDIF slot on its own, which is exactly the misalignment 528b170 fixed. Release's `apply_restored_outputs` covers boot.
- Use the overhaul's `start_output_slots_in_sync` inside release's `complete_pipeline_reset` and `enable_outputs_in_sync`. Keep release's Phase 3.5 dwell, Phase 4 mute rule, `i2s_input_resync` and `adat_output_resync`.

**Library changes inside the merge commit:**
- Port `audio_spdif_attach_pin` and `audio_i2s_attach_data_pin`. Each is the overhaul's pin change minus the old-pin release, keeping `ring_dma_stop()` and, for I2S, the clock divider restart.
- Port release's guarded pin release (`spdif_release_pin`, `i2s_release_pin`, the MCK pad check). The overhaul releases pins unconditionally and can take back a GPIO another subsystem claimed.
- Port `audio_i2s_apply_pio_frequency` as a wrapper over the overhaul's static frequency update.
- Do not port `audio_i2s_mck_get_divider`. Its only caller was the MCK servo.
- Clear `inst->ring` in both teardowns. Otherwise a SPDIF instance still looks live after its slot switches to I2S.

**Gate:** both platforms build clean from scratch. Measure free heap. If RP2040 cannot allocate its producer pools, do 2i before anything else.

## Phase 2: Semantic integration

Each item below is its own commit on the integration branch, and every commit builds on both platforms.

### 2a. Stalls and flash writes

With rings, outputs keep clocking through any stall, because nothing in the output path needs an interrupt. The problem moves to what the DMA replays. A 1024-frame ring laps about twice in a 45 ms flash write at 48 kHz, and four times at 96 kHz. SPDIF frames carry their IEC block position from their absolute frame number, and 1024 is not a multiple of 192. Replayed silence is therefore 64 frames out of block step on every lap. That is the receiver-unlock class of fault that 285b89d set out to remove.

- **Keep release's selective NVIC blackout**, including the RP2350 word-1 handling from 9776c2f, but change what it keeps alive. Instead of the deleted output DMA handlers, keep one RAM-resident timer alarm that stamps correctly framed silence ahead of each reader while a stall is open. This keeps the IEC block structure intact for any stall length.
- Wrap it in one stall bracket API (begin and end), used by the flash window, the DAC-hold spin in `pipeline_reset_ready`, and `pdm_hold_hw` waits. Release has several main-loop stalls that do not pump, and all of them need the same treatment. The alarm runs only inside a bracket, where the producer is stopped by construction, so it never races the main-loop writer.
- Replace the pool-sized `PIPELINE_FADE_DRAIN_SAMPLES` dwell with a measured mark. At the first admitted, fully zero block, record `mark = wr_frames + pipeline_max_active_delay_samples()`. Silence is observed once every enabled slot's consumed frame counter has passed the mark (signed compare). The overhaul's full-ring prestamp becomes the bracket's first pump.
- Re-derive the Phase 3.5 post-restart dwell from the ring fill target instead of pool refill.
- Rewrite `FLASH_WINDOW_ROOTS` in `check_ram_placement.py`. The new roots are the stall alarm handler and everything it calls. Add placement checks that every word a DMA reads during the window is in RAM. That covers the slot rings, `spdif_ring_words_const` and `i2s_ring_words_const` (RP2040 reload), `adat_ring_base_holder`, and the soft VCXO's `fbdivs` and `fbdivs_addr`.

### 2b. ADAT output after a skip

Ring starvation is counted late. The group skip happens at the first admission after a stall, not during it. Release's `adat_output_rebaseline_starvations()` therefore runs before the skip exists and does nothing, and the overhaul's 1:1 mirroring is wrong once ADAT's own 896-frame ring has lapped.

- Mirror a skip into ADAT 1:1 only when its deficit is below `ADAT_ALIGN_LEAD_FRAMES` (96), because then ADAT never ran dry.
- Otherwise re-anchor ADAT to `ADAT_ALIGN_LEAD_FRAMES` plus slot 0's fill after the skip, reading both pointers in one short interrupt-off section. Today's `adat_output_resync()` assumes slot fill is zero, which only holds right after a synchronized start.
- Delete `adat_output_rebaseline_starvations()`.
- Check whether framed ADAT silence can be stamped by the 2a alarm too, which would make stalls seamless on ADAT as well.

### 2c. PDM

- Keep release's `pdm_flash_silence` and forced lead re-anchor, the hold handshake and the pin move.
- Gate PDM's queue capacity as an admission input only while PDM is actually draining. The condition is `core1_mode == CORE1_MODE_PDM && pdm_enabled && !pdm_hold && pdm_hw_is_active()`. Without the hold and active terms, a held PDM whose queue fills makes every output drop blocks.
- Prove that the overhaul's skip recovery (384-frame default reference after reset) and release's re-anchor (`TARGET_LEAD`) put PDM at the same offset against the slots. This was not verifiable by reading. It needs a bench measurement.

### 2d. Synchronized start and ring reset races

- **SPDIF rewind.** The overhaul's ring arm clears the TX FIFO but does not restart the state machine. A stopped SPDIF SM keeps 0 to 31 half-shifted bits in its output register and plays them before frame 0, so slots restart up to a quarter frame apart. Port release's rewind loop (clear FIFOs, `pio_sm_restart`, jump to `output_low`) into `audio_spdif_enable_sync_prepare`, **before** the ring arm loop. If the clear runs after the arm, it throws away prefilled words already counted as consumed, which is a two-frame skew. I2S already rewinds in this order.
- **SOF race.** `uac1_driver_sof` reads slot 0's consumed words whenever no type switch is in progress, including during Phase 1 of `complete_pipeline_reset` with interrupts enabled. If it lands inside `ring_reset` between the anchor write and the DMA read address write, it counts bogus words and the next arm starts slot 0 mid-frame. Make the reset atomic with interrupts off, or have SOF skip slots that are not enabled.
- **Loopback.** The `DSPI_LOOPBACK` push happens before admission, so it captures blocks the wire drops. Move it behind admission (rule 4).

### 2e. Servo exits

- Confirm that every path out of a tracked input parks the VCXO through its stop function. That covers ADAT slave to USB, ADAT master/slave flips, I2S slave exit, SPDIF unlock and source switches, factory defaults (11a1cf7 already routes this through the handler), and bulk apply.
- Confirm that nothing else calls `input_servo_reset()`.
- The I2S-against-SPDIF drift that 3272007 fixed cannot recur. A single PLL trim moves every output, and the I2S divider is always exactly twice the SPDIF one.

### 2f. Fixes worth keeping from the stash

The stash "SPDIF non-ring servo port" is otherwise superseded. Four of its fixes apply to the overhaul's code:
- Make `block_count` a `uint32_t`. The overhaul's `static int` overflows (undefined behaviour) after about 99 days at 48 kHz.
- Take `now_us` before `_clear_timer()` to remove alarm-cancel jitter from the block timestamp.
- `_fifo_live_count_locked` returns 0 unless the library state is STABLE.
- `spdif_input_poll` re-checks stable-then-lost at the lock edge, so it never enters LOCKED on a dead receiver.

After the merge lands, the stash can be dropped. That needs your go-ahead.

### 2g. VCXO parameterization

Read the nominal `fbdiv_int`, the system clock and the ppm-per-pulse rate from the PLL at init instead of hardcoding 307.2 MHz and fbdiv 128. Widen the PWM pacer divider so the documented minimum pulse rate is actually reachable (today's floor is 36.6 Hz against a 30 Hz design floor).

This costs a few lines now, and it is exactly what the `overclock` branch would otherwise have to retrofit. A runtime clock switch will additionally need `soft_vcxo_stop()` before `set_sys_clock_pll()`, because the pulse DMA keeps writing the old fbdiv. That belongs to the overclock merge, but the stop function should exist now.

Also make Loop A's divider reference use one shared helper, not a private `ceil(sys/freq)`.

### 2h. Clock diagnostics

- `DSPI_CLOCK_DIAG` is hardcoded to 1 on the overhaul and its own comment says to set it to 0 for release. Turn it into a CMake option, off by default.
- Delete the two A/B bench switches (`SOFT_VCXO_DIAG_FORCE_PARK`, `SOFT_VCXO_DIAG_FREEZE_INTEGRATOR`). They tested hypotheses that are now closed.
- Fix the stale "0x80/0x81 stay free" comment, since both are taken on release. 0x82 itself is free.
- Document that `REQ_GET_CLOCK_DIAG` is USB-only. Its packet of about 190 bytes is truncated by the UART and I2C transports.

### 2i. RAM placement

- Place `slot_ring_store` in its own explicitly aligned linker section at the start of RAM bss, so it stops paying up to 16 KB of padding.
- Re-measure both platforms, set `DATA_BUDGET` from the measured numbers, and record free heap after producer pools in the arch doc.

### 2k. Output DMA IRQ plumbing

Nothing but SPDIF RX uses a DMA interrupt once the rings land. Remove the output libraries' `dma_irq` fields and IRQ refcount, the output-line priority setup in `usb_audio.c`, and the IRQ mask in `process_type_switches`. Move anything SPDIF RX still needs into SPDIF RX.

### 2j. Documentation

Update `current_architecture.md` for the merged design as CLAUDE.md requires. That covers these sections:
- Clock servo, rewritten for the VCXO.
- Output path, rewritten for rings.
- Silent state changes, rewritten for the stall bracket.
- Memory Layout.
- The RP2040 vs RP2350 comparison (DMA channel map, reload channels, PWM pacer slice).
- The Vendor Command Reference (0x82).
- `GET_STATUS` values 17 to 21, whose starvation counters change from silence buffers to frames.

Also mark sections 5.2 and 5.3 of `silent_state_changes_spec.md` as unimplemented. They describe no-teardown completion code that does not exist. Rings make it achievable, but it is out of scope here.

## Phase 3: Producer-pool removal (follow-up)

Do this after Phase 4 has proven the merged build on hardware. Removal is behaviour-neutral, so it can be verified against a known-good baseline.

The steps are as follows:
- Point all four pipeline variants and Core 1's `spdif_out` pointers at per-slot staging.
- On admission, write rings directly with `audio_*_ring_write_s32`.
- Move the lazy rate follow out of `wrap_producer_give`.
- Delete the `connect_extra` and connection plumbing.
- Give the libraries a rate source other than `audio_format_48k`.

It frees about 25 KB of heap on RP2040 and about 51 KB on RP2350, minus any staging, and removes one copy per sample per slot. If 2i still leaves RP2040 without comfortable headroom, this phase moves ahead of Phase 4.

## Phase 4: Verification

1. Clean builds of both platforms, `check_ram_placement.py` with 0 FAIL, and the full loopback suite at 48 and 44.1 kHz with no regressions against the Phase 0 baseline.
2. **Alignment matrix on hardware, both platforms.** Measure inter-slot phase across SPDIF slots, I2S slots, PDM and ADAT out after each of these operations: boot, preset load, preset save, active preset delete, factory reset, bulk SET, every metadata flash write, input source switch, rate change, output type switch, output pin change, MCK change, USB alt change, and an I2S slave or ADAT slave unlock and relock. Any drift of even one sample is a failure.
3. **Clock tracking.** Use `tools/clkdiag_poll.py` with a diagnostic build. Cover SPDIF in at 44.1, 48 and 96 kHz, ADAT slave, and I2S slave, on both platforms. RP2040 PLL dithering has never run on hardware.
4. **Flash stress.** Run repeated renames and Control Surface saves while playing into an external SPDIF receiver and a PCM5102 on I2S. Pass means no receiver unlock and no wedged DAC.
5. **Soak.** Run continuous playback past 6.2 hours, the old counter-coherence failure point, with the fill steady and zero starvation.
6. **RP2040 ring path end to end.** This is the largest unknown in the whole merge.

## Phase 5: Landing

- Fast-forward release/v1.1.6 to the integration branch only when you say so. The same applies to every commit and push along the way.
- **DSPi Console** must know four things. The starvation counters in `GET_STATUS` 17 to 21 are now frames. Those counters also climb at the sample rate whenever the outputs are enabled but nothing is producing (host stream stopped, source gap): the idle re-anchor counts every frame of silence it skips, where the pool-era counters were gated on an active USB stream. The Console must not present a climbing counter alone as an underrun fault. `fill_centi_pct` is available in buffer stats. RP2040 Control Surface bindings on GPIO 14 and 15 now report `CS_STATUS_PWM_CONFLICT`, because PWM slice 7 is the VCXO pacer.
- **ESP32 front panel**: no scalar GET it validates changes length or range, so no action is expected. Confirm it does not read buffer stats before release.
- Release notes should describe the user-visible effects. Flash writes no longer interrupt any output, clock tracking is smoother on every input, and PDM is now clock-tracked.

## Release fixes and their ring-world counterparts

| Release change | After merge |
|---|---|
| 285b89d keep mask for output DMA IRQs | Keep mask now holds the stall silence alarm (2a). |
| 285b89d fade-to-observed-silence | Kept. Drain measured by ring mark (2a). |
| 285b89d PDM silence plus re-anchor | Kept (2c). |
| 285b89d ADAT starvation rebaseline | Replaced by the skip-size rule (2b). |
| 9776c2f NVIC word 1 | Kept unchanged. |
| 773a453 full-path completion | Kept. |
| 528b170 unified pin transition | Kept, on ring libraries (Phase 1). |
| 528b170 SPDIF SM rewind | Ported, ordered before ring arm (2d). |
| 528b170 PDM hold handshake | Kept. Joins the stall bracket and the admission gate (2a, 2c). |
| 4537294 pin restore order | Untouched by the overhaul. Kept. |
| 3272007 one divider for every output | Deleted. The VCXO makes it structural. |
| 4bf2084 input capture arena | Auto-merges. The overhaul's I2S RX ENDLESS rows are self-aligned in the arena. |
| 11a1cf7 factory-default USB switch via handler | Kept. It is part of what guarantees the VCXO parks (2e). |

## Decisions

*Decided 2026-10-10.*

1. Stall handling keeps the selective blackout with a RAM silence alarm (2a).
2. Producer-pool removal is Phase 3, after hardware proof.
3. Clock diagnostics become an off-by-default CMake option (2h).

## Progress

### Phase 0 results

*Last updated: 2026-10-10*

The integration branch is `merge/spdif-rx-overhaul`, created from release/v1.1.6 at 3272007.

Clean builds of both platforms finished with zero warnings. `check_ram_placement.py` reports 0 FAIL on both, with 51 WARN on RP2040 and 52 on RP2350, all pre-existing veneer and cold-path warnings.

| | RP2040 | RP2350 |
|---|---|---|
| text / bss (`arm-none-eabi-size`) | 283,604 / 140,720 | 324,548 / 334,124 |
| `end` | 0x20033cf0 | 0x2006901c |
| Free heap (`__HeapLimit - end`) | 49,936 B | 94,180 B |
| `.data` against `DATA_BUDGET` | 67,488 of 67,584 | 94,160 of 94,208 |
| free_ram (check script) | 53,936 B | 96,004 B |

The `.data` margin is 96 B on RP2040 and 48 B on RP2350. The overhaul's RAM-resident code (ring writers, VCXO, estimator) will exceed both budgets, so 2i raises them from measured numbers.

Baseline images are saved in `custom-builds/merge-baseline-3272007/`. There is a normal and a `DSPI_LOOPBACK` UF2 for each platform, plus the normal ELFs.

**Pending:** the loopback audio suite at 48 and 44.1 kHz. No DSPi was connected when Phase 0 ran. Run it on the baseline loopback images before Phase 1 results are compared on hardware.

### Phase 1 results

*Last updated: 2026-10-10*

The merge is resolved and staged on `merge/spdif-rx-overhaul`, but not committed. Clean builds of both platforms finish with zero compiler warnings.

Work pulled forward from Phase 2, because the conflicts forced the decision:
- **2d SPDIF rewind** is done. It sits in `audio_spdif_enable_sync_prepare` before the ring arm loop.
- **2i ring placement** is done, because without it RP2040 could not allocate its producer pools (14,344 B of heap against about 25 KB needed). `slot_ring_store` was a tentative definition in COMMON, which dragged the COMMON block and then all of `.bss` onto 16 KB boundaries. It now lives in a NOLOAD `.slot_rings` section at the top of RAM in both linker scripts. `__HeapLimit` ends below it, and an ASSERT ties its size to `SLOT_RINGS_BYTES`. It is not zeroed at boot, which is safe because every slot setup stamps its whole ring.
- **Dead MCK servo API removed.** `audio_i2s_mck_set_divider` and `audio_i2s_mck_get_divider` had no callers left.

main.c servo cleanup:
- `restore_nominal_output_clocks` and `align_output_clocks` are deleted.
- `apply_nominal_output_rate` programs S/PDIF slots and MCK at a real rate change, without `input_servo_reset`.
- The equal-rate restore branches in both clock-mode flips and both source-switch exits are gone. Their input stop functions park the VCXO.

| | RP2040 | RP2350 |
|---|---|---|
| Free heap before producer pools | 44,852 B (baseline 49,936) | 84,408 B (baseline 94,180) |
| Free heap after producer pools (estimate) | about 19 KB | about 33 KB |
| `.data` against `DATA_BUDGET` | 70,064 of 67,584 (FAIL, see 2i) | 97,008 of 94,208 (FAIL, see 2i) |

`check_ram_placement.py` fails three checks per platform, all expected. Two are the deleted flash-window roots (2a), and one is the `.data` budget (2i).

New items found during Phase 1:
- **2a.** Remove the stale `audio_spdif_dma_irq_handler` / `audio_i2s_dma_irq_handler` entries from the script's `COMMON` list along with `FLASH_WINDOW_ROOTS`. Add the ring writer and admission functions to the hot list.
- **2i.** `check_ram_placement.py` computes free RAM from `.data + .bss + .heap` and does not count `.slot_rings`, so its free RAM figure now over-reports by 32 KB / 64 KB.
- **2k (new).** Remove the remaining output DMA IRQ plumbing. Nothing but SPDIF RX uses a DMA IRQ now. That covers the `dma_irq` config fields, the TX library's IRQ refcount, the priority setup for the output lines in `usb_audio.c`, and the type-switch IRQ mask in `process_type_switches`. That mask is harmless today, because SPDIF RX's own stop and start re-enable its line. Whatever SPDIF RX still needs should move to SPDIF RX itself.
- **2j.** About 15 comments still describe consumer pools, some already stale on the overhaul branch.

The merge was committed as `0c3a762` on 2026-10-10.

### Phase 2a results

*Last updated: 2026-10-10*

2a is implemented and builds clean on both platforms, but is not committed.

**Stall bracket** (`audio_pipeline.c`):
- `output_rings_stall_init()` claims a spare hardware alarm at boot.
- While a bracket is open, its RAM-resident handler re-runs `output_rings_stamp_silence_ahead()` every 2 ms.
- Brackets nest.

**Where brackets open:**
- In both flash primitives (`flash_program_staged()`, `preset_delete()`), around Core 1 lockout, PDM silence and erase/program.
- Around the DAC-hold spin in `process_type_switches()`.
- Around the PDM hold handshake, through a new `pdm_hold_for_pin_move()` helper used at both call sites.
- Not across the whole flash prepare-to-complete span, because four handlers run `apply_restored_outputs()` inside it. The full-ring stamp moved to the very end of `prepare_flash_write_operation()` to cover the short gap before the first primitive.

**Selective blackout:** it now keeps only the stall alarm's NVIC line. The DMA-IRQ static assert and keep mask are gone, along with the now-unused `hardware/irq.h` include.

**Fade drain:**
- A measured silence mark (`output_rings_silence_mark_arm()` / `_reached()`) replaced `PIPELINE_FADE_DRAIN_SAMPLES` and its 4 ms margin.
- A ring reset after arming counts as silent.

**Post-restart hold:**
- Phase 3.5 now holds `OUTPUT_RING_TARGET_FILL_FRAMES` (384) samples plus the DAC release time, instead of 24 ms.
- The 24 ms constant survives under an honest name, `PIPELINE_OP_MUTE_FLOOR_MS`, for its other job: the minimum operation mute in `prepare_pipeline_reset()`.
- `OUTPUT_RING_TARGET_FILL_FRAMES` also replaced the `8u * 48u` literal in the underrun re-anchor.

**RAM check:**
- `FLASH_WINDOW_ROOTS` is now `output_rings_stall_irq`, which is also in the Check A/B hot list.
- The stale output DMA handler entries are gone.
- The ring writers, admission and pumps are listed.
- Placement checks now cover every DMA-read word: `slot_ring_store`, `fbdivs`, `fbdivs_addr`, `adat_ring_base_holder` (RP2350), and `spdif_ring_words_const` and `i2s_ring_words_const` (RP2040).
- All pass. The only failure left is the `.data` budget (2i): 70,512 on RP2040 and 97,456 on RP2350.

**Docs:**
- New "Output Ring Stall Bracket" section in `current_architecture.md`.
- "Selective NVIC blackout" and "Preset-Switch Mute & Pipeline Reset" are updated.
- `silent_state_changes_spec.md` section 4 points at the new design.

**Not yet verified on hardware:**
- That an external SPDIF receiver stays locked through repeated flash writes.
- Alarm timing at 96 kHz.

**Left for 2b:** the ADAT paragraphs in "Selective NVIC blackout" still describe the pool-era starvation rebaseline.

2a was committed as `60fc3af`.

### Phase 2b results

*Last updated: 2026-10-11*

2b is implemented and builds clean on both platforms, but is not committed. It replaced the plan's two-case rule ("mirror below the cushion, otherwise re-anchor by measurement") with exact accounting. Three defects inherited from the overhaul went with it:
- The slot skip was mirrored late, from the main-loop task, after the next block had already been pushed. ADAT played the skip's silence after that block instead of before it.
- `adat_output_resync()` always seeded 96 frames and ignored slot 0's fill, so the ADAT-to-slot offset varied with each reset and prefill.
- The 896-frame ADAT ring could not hold the cushion plus the rings' 832-frame fill cap.

The new design:
- **Invariant.** ADAT lead = 96 + slot 0 signed ring fill + `adat_excess_frames`.
- **Skip mirror.** `output_rings_pre_give()` calls `adat_output_on_slot_skip(S)` right after skipping the slots, before the block is pushed. ADAT writes S minus its excess.
- **Keep-alive.** `adat_output_keepalive()` (main-loop task and stall alarm) tops the lead up to 96 from slot 0's counters while slot 0 is dry, and records what it wrote as excess. It is exact even after the reader overtook the writer.
- **Resync.** It fills the whole ring with level-neutral silence. Then, with interrupts off, it places the write pointer at 96 + slot 0's measured fill, which is exact to within one frame because ADAT cannot join the PIO0 synchronized start.
- **Removed.** The starvation-counter slaving is gone: `adat_output_set_stream_active`, `adat_output_rebaseline_starvations`, the `lead == 0` local resync, and their callers in `usb_audio.c` and `main.c`.
- **Ring size.** `ADAT_RING_FRAMES` is now 960, static-asserted against the cushion plus `SPDIF_RING_FRAMES - ERASE_AHEAD`. This costs 2 KB of BSS on RP2350, whose heap before pools is now 82,340 B.
- **Alarm period.** The stall alarm period went from 2 ms to 1 ms, because ADAT's 96-frame cushion lasts 2 ms at 48 kHz.
- **New helper.** `output_ring_fill_signed()` is RP2350-only, so RP2040 pays nothing.
- **RAM check.** It lists `adat_output_keepalive`, `adat_output_on_slot_skip` and `output_ring_fill_signed`. All pass, and only the 2i `.data` budget fails (RP2350 97,960).
- **Docs.** `current_architecture.md` ("ADAT Bulk Output" alignment rewritten, flash-window ADAT paragraph, stall bracket period, Memory Layout) and `adat_output_spec.md` are updated.

**Possible follow-up:** RP2350 can start state machines on several PIO blocks in one cycle (`pio_enable_sm_multi_mask_in_sync`). Joining ADAT to the slots' synchronized start would make its anchor exact instead of within one frame. This is not done yet.

**Not verified on hardware:**
- That the ADAT-to-slot offset stays constant across underruns, flash writes and resets.
- That an ADAT receiver stays locked through flash writes.

2b was committed as `e32c87c`.

### Phase 2c results

*Last updated: 2026-10-11*

2c is implemented and builds clean on both platforms, but is not committed.

- **Admission gate.** PDM's push queue counts as a ring admission input only while `pdm_queue_is_draining()` holds (new, RAM-resident): `core1_mode == CORE1_MODE_PDM && pdm_enabled && !pdm_hold && pdm_hw_active`. The old gate ignored a held PDM, whose filled queue would refuse every block for every output.
- **Kept as they were.** `pdm_flash_silence()` with its forced re-anchor, the hold handshake, and the pin move.
- **RAM check.** `pdm_ring_free_slots` and `pdm_queue_is_draining` are in the hot list. All pass, and only the 2i budget fails.
- **The plan's offset question, answered by reading.** The skip recovery and the flash re-anchor agree. After any pipeline reset, and after a flash window, PDM sits at `TARGET_LEAD` and the slots re-anchor to 384 frames at the first block. The underrun skip preserves whatever offset exists.

**Found, and pre-existing in release (open, needs your decision).** The PDM offset to the slots is miscalibrated and non-deterministic:
- `SUB_ALIGN_SAMPLES` assumes PDM latency is its whole 256-sample DMA ring. Core 1 actually writes about 32 samples (`TARGET_LEAD` is 256 words) ahead of the reader, so by this reading the sub plays about 200 samples (about 4 ms at 48 kHz) ahead of the mains.
- Late blocks make Core 1 insert zeros, which ratchet PDM later. Bursts re-seat it earlier.

This is documented under "Known alignment defect" in `current_architecture.md`, with the corrected `TARGET_LEAD` units. It needs a scope measurement (siggen burst, PDM pin against an I2S data line) to confirm the magnitude. The proper fix is to give PDM the slots' ring accounting, which overlaps the planned PDM-as-slot overhaul.

2c was committed as `e63e6e9`. On 2026-10-11 the PDM offset was deferred to the PDM overhaul (recorded in the to-do list).

### Phase 2d results

*Last updated: 2026-10-11*

2d is implemented and builds clean (both platforms, plus both loopback-test builds), but is not committed. The SPDIF rewind half of 2d already landed in the merge commit.

- **SOF race.** `audio_spdif_ring_reset()` / `audio_i2s_ring_reset()` and both ring arms now set the software read anchor and the DMA read address inside one interrupt-disabled section. The race was worse than the review predicted. A start-of-frame interrupt landing between the two writes counted the stale DMA address and then its wrap back, adding exactly one ring (1024 frames) to slot 0's consumed count. Slot 0's accounting then sat a full ring ahead of its physically aligned DMA, and admission and the underrun skip would fight over the phantom deficit until the next reset. The firmware arms only inside the IRQ-off synchronized start, but the arm is guarded too, because the libraries' public `set_enabled(true)` reaches it.
- **Loopback.** The `DSPI_LOOPBACK` capture tap now runs only for admitted blocks.

2d was committed as `91b93cb`.

### Phase 2e results

*Last updated: 2026-10-11*

2e is implemented and builds clean on both platforms, but is not committed.

**Audit result.** Only the three tracked input modules call `input_servo_reset()`. Nothing on the pipeline-reset path does: the Phase 1 merge removed the old `restore_nominal_output_clocks` call, and `i2s_input_resync()` returns at once for the external-clock role.

Every exit from a tracked input reaches a stop that parks the VCXO:
- **Source switches** go through the source-switch handler.
- **Factory defaults** reach USB through the same handler (11a1cf7).
- **I2S and ADAT clock-mode flips** stop the input explicitly first.
- **Flash suspends** stop the input and restart it, which parks again.
- **Leaked receivers** are caught by `stop_foreign_input_receivers()`.

The servo applies only while locked and only once the detected rate equals `audio_state.freq`. The I2S/SPDIF divider ratio can no longer drift, because one PLL trim moves every output.

**Inconsistency fixed.** `soft_vcxo_spec.md` says a lock loss holds the last-good trim so relock re-converges from where it was, and SPDIF does that. ADAT and the I2S slave parked the VCXO and zeroed the integrator at every lock. Their lock-time reset descends from release's ADAT "force a full divider rewrite on first servo" call, whose meaning changed from "rewrite the divider" to "park the PLL" under the VCXO.
- Both lock-time resets are removed.
- `adat_input_start()` now parks explicitly at tracking start, like `spdif_input_start()` and `i2s_slave_arm()`.
- The `input_servo.h` comment and the architecture doc state the rule: reset only at tracking start and stop, never on relock, never on the reset path.

**Not verified on hardware:** relock behaviour on ADAT and I2S slave after a cable pull.

2e was committed as `593968a`.

### Phase 2f results

*Last updated: 2026-10-11*

2f is implemented and builds clean on both platforms, but is not committed.

**Applied whole.** The stash's `spdif_rx.c` and both `spdif_rx.h` copies differ from the merged tree only by these fixes and by shorter comments that meet the four-line comment limit. The overhaul's versions ran to six or seven lines.
- `block_count` is `uint32_t`.
- `now_us` is taken before `_clear_timer()`.
- `_fifo_live_count_locked()` returns 0 unless STABLE.

**Ported.** In `spdif_input.c`, the lock-edge recheck: a stable-then-lost flag pair between polls goes to RELOCKING instead of LOCKED.

**Not ported, already present or obsolete:**
- The overhaul already has the loss-pending servo guard and the 1 ms batching.
- The first-apply preload, the divider-domain PI, the 2 s nominal-rate hold and the ADAT div/32 bound belong to the divider servo the VCXO replaced.

**The stash itself** (`stash@{0}`, "SPDIF non-ring servo port") is now fully accounted for. It can be dropped once you approve.

2f was committed as `e60fee3`. The stash was dropped on 2026-10-11; its commit was `b4b1ae62398d7d935163fc9586bc530391767ff4`, recoverable from git's internal records while they last.

### Phase 2g results

*Last updated: 2026-10-11*

2g is implemented and builds clean on both platforms, but is not committed.

**Soft VCXO:**
- `soft_vcxo_init()` reads refdiv, the feedback divider N and both post-dividers from `pll_sys_hw`. It stays inert unless they reproduce `clock_get_hz(clk_sys)`.
- It computes the pulse rate per ppm at runtime as sys_MHz × N / `SOFT_VCXO_PULSE_CYCLES`, which is 262.144 at 307.2 MHz.
- `SOFT_VCXO_FBDIV_NOM`, `SOFT_VCXO_SYS_HZ` and `SOFT_VCXO_HZ_PER_PPM` are gone.
- The pacer now picks the smallest integer clkdiv up to 255, not powers of two up to 128. The old floor could not reach the 30 Hz park threshold, so commands between 30 and 36.6 Hz were clamped. Park is now a ppm threshold (`SOFT_VCXO_PARK_PPM` 0.115), reachable for any N of 79 or more.

**Clock diagnostics.** `clock_diag.c` measures the sys_clk ppm offset against `clock_get_hz(clk_sys)` instead of a hardcoded 307.2 MHz.

**`soft_vcxo_stop()` not added.** The plan called for it, but nothing would call it until the overclock merge, so it would be dead code. The header and the architecture doc instead state that a runtime sys_clk change must stop the pulse train and re-run init.

**Shared divider.** `firmware/DSPi/audio_clock_div.h` holds `audio_base_divider_16_8()`, with the same name and the same round-to-nearest as the `overclock` branch. Every output divider now derives from it: S/PDIF TX, I2S TX (×2), the I2S RX clock master (×2), ADAT TX, PDM (previously a float division), MCK (×256/mult), and the input servo's feed-forward. At 307.2 MHz every supported rate's slot divider is unchanged.

**Fixed along the way.** MCK truncated its own divider, which put it one LSB (about 144 ppm) off BCK at 44.1 kHz on the nominal path. It now gets 6966 instead of 6965 at 256×, and 13932 instead of 13931 at 128×.

**Docs.** There is a new "Shared output divider" section, and "Soft VCXO" and the MCK notes are updated.

2g was committed as `3d0a091`.

### Phase 2h results

*Last updated: 2026-10-11*

2h is implemented and builds clean, but is not committed. Default builds and `-DDSPI_CLOCK_DIAG=ON` builds both compile on both platforms.

- **Build option.** `DSPI_CLOCK_DIAG` is now a CMake option, OFF by default, following the `DSPI_LOOPBACK` pattern. `config.h` only defaults it to 0. Release images contain no diagnostic code, which saves about 500 B of RAM code on RP2040 and 630 B on RP2350 (`.data` is now 70,064 and 97,560).
- **A/B switches deleted.** `SOFT_VCXO_DIAG_FORCE_PARK` and `SOFT_VCXO_DIAG_FREEZE_INTEGRATOR` are gone, along with their `#if` blocks in `soft_vcxo.c` and `input_servo.c`.
- **Transport guard.** 0x82 answers with zero bytes when the requesting transport's response buffer cannot hold the packet (UART/I2C, about 190 B against `_ext_resp_copy`), instead of being silently truncated.
- **Comments and docs.** The stale "0x80/0x81 stay free" comment is fixed. "Clock Diagnostics", the vendor table row and `tools/clkdiag_poll.py` now describe the option.

**Note for Phase 4:** the clock-tracking checks need a diagnostics build. Configure a separate build dir with `-DDSPI_CLOCK_DIAG=ON`.

2h was committed as `56e1fc6`.

### Phase 2i results

*Last updated: 2026-10-11*

2i is implemented, but is not committed. The ring placement half landed in the merge commit. **`check_ram_placement.py` now passes on both platforms (0 FAIL)** for the first time since the merge, and so do both loopback-test builds.

- **Budgets.** `DATA_BUDGET` is now 72,704 B (RP2040) and 100,352 B (RP2350), about 2.5 KB above the measured 70,064 B / 97,560 B. The loopback builds measure 70,816 B / 98,352 B. The budget comment points to Memory Layout instead of narrating history.
- **Free RAM accounting.** `free_ram` now counts `.slot_rings`. A new `heap_free` line reports `__HeapLimit - end`, the real arena the producer pools allocate from: 45,256 B and 82,804 B. That leaves about 19 KB and 31 KB after the pools. The loopback builds have 36,640 B and 65,992 B.
- **Docs.** The Memory Layout section has a dated merge note. Its RP2040 and RP2350 tables are re-measured, with slot rings at the top of RAM, the input capture arena row, the ADAT TX ring row, `.bss` plus `.slot_rings`, `.data`, free heap and pool sizes. The comparison table's RAM rows are updated too.

2i was committed as `219e076`.

### Phase 2k results

*Last updated: 2026-10-11*

2k is implemented and builds clean, but is not committed. The default, diagnostics and loopback builds all compile, and the RAM check passes.

**Removed:**
- The `dma_irq` field from both libraries' config and instance structs, and its six initializers in `main.c` and `usb_audio.c`.
- The SPDIF library's `irq_enable_count` refcount and `audio_spdif_irq_refcount_adjust()`.
- Both no-op `*_set_starvation_monitoring()` functions, with their callers in `usb_audio.c` and `clock_diag.c`.
- The I2S setup's IRQ-bit clear, kept for "a prior pool-mode user" of the channel, which no longer exists.
- The `PICO_AUDIO_SPDIF_DMA_IRQ` / `PICO_AUDIO_I2S_DMA_IRQ` header defaults and CMake defines.
- The output IRQ priority setup in `usb_audio.c`.
- The `USBCTRL <= DMA` priority asserts in `main.c`. They guarded an SOF read the pool-era DMA handlers raced, and that read is now IRQ-safe on its own.
- The type-switch IRQ mask in `process_type_switches()`.

**Caught during this step:** SPDIF TX had been on `DMA_IRQ_1` with SPDIF RX (`PICO_AUDIO_SPDIF_DMA_IRQ=1` in CMake), so the removed priority call was what made the RX capture IRQ highest priority. The receiver relies on that to preempt its decode-timeout alarm. `spdif_input_init()` now sets it explicitly (confirmed in the disassembly).

**SPDIF RX owns its line.** `pico_spdif_rx`'s teardown again disables `DMA_IRQ_1`. That was upstream behaviour, removed only because SPDIF TX shared the line. It matches what the deleted refcount did at stop.

**Docs.** The architecture doc's "IRQ Handling", compile definitions, instance struct, the channel-sharing paragraph, the comparison table row, the I2S output notes and the SPDIF input DMA line are updated. The last of these had wrongly said `DMA_IRQ_0`.

2k was committed as `5a2a7e5`.

### Phase 2j results

*Last updated: 2026-10-11*

2j is done but not committed. It closes Phase 2.

**Code comments.** These no longer describe consumer pools or output DMA IRQs:
- About 10 comments in `main.c`, including the teardown and flash-completion rationales. Those were rewritten as present-tense hazards instead of history.
- Comments in `i2s_input.c`, `input_servo.h` and `vendor_commands.c` (the type-switch deferral reason).
- Both output libraries' file headers and the connect-function docs. Their `consumer_pool` parameter is documented as unused until Phase 3 deletes the functions.
- In `config.h`, `SPDIF_CONSUMER_BUFFER_COUNT` is now described as the 48-frame bucket scale. The sub-alignment block points at the PDM known-defect section.

**Dead pool-era constants removed.**
- `PICO_AUDIO_SPDIF_CONSUMER_FRAME_BYTES`, `PICO_AUDIO_I2S_CONSUMER_FRAME_BYTES` and the static assert that used it.
- `PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT` and its assert.
- `PICO_AUDIO_I2S_DMA_SAMPLE_COUNT` and `PICO_AUDIO_SPDIF_BUFFERS_PER_CHANNEL`.
- The ring code's `% 192u` literals now use `PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT`.

**Specs:**
- `silent_state_changes_spec.md` section 5 (Pillar B) is marked not implemented. None of its functions exist; the rings make it achievable.
- `soft_vcxo_spec.md` now describes the runtime clock-plan read and the merge status.
- `current_architecture.md` has the remaining pool references corrected and an `audio_clock_div.h` file-table row.

**Phase 2 is complete.** Commits from `0c3a762` (merge) to `5a2a7e5` (2k), plus this one. Every build variant compiles clean and `check_ram_placement.py` passes on both platforms. Nothing has run on hardware yet.

**Next:** Phase 4 (hardware verification) comes before Phase 3 (producer-pool removal), as planned.

2j was committed as `006b3de`.

### Phase 4 preparation

*Last updated: 2026-10-11*

The bench images are built clean from `f223724` (the post-merge audit fixes; the earlier `006b3de` set in `custom-builds/merge-bench-006b3de/` is superseded). Each reports `v1.1.6-beta4-37-gf223724`, without the dirty flag. They are saved in `custom-builds/merge-bench-f223724/`: a normal, a loopback and a clock-diagnostics UF2 for each chip, plus the normal ELFs. The step-by-step checklist is `spdif_rx_overhaul_bench_checklist.md`. Its first section is the still-owed Phase 0 baseline run.

### Post-merge audit

*Last updated: 2026-10-11*

A read-through audit of the whole branch after Phase 2j found no alignment defect. Three code fixes landed from it, all after `006b3de`, so the bench images predate them and must be rebuilt before Phase 4:
- **Heap bound.** Both linker scripts set `__StackLimit` to the `.slot_rings` address. The SDK's `_sbrk` and `pico_malloc` bound the heap by that symbol, not `__HeapLimit`, so the merge's `__HeapLimit` move alone left malloc free to grow into the rings. Not reachable today (the producer pools are the only heap users), but silent corruption if it ever were.
- **SPDIF lost-then-stable.** `spdif_input_poll()` read the rate from the IRQ-written copy that its own loss branch had just zeroed, so a loss and relock between two polls held RELOCKING, muted, until the next loss. The lock branch now reads the library's rate. Pre-existing on release.
- **ADAT lead bound.** `ADAT_MAX_LEAD_FRAMES` (928) bounds the resync write pointer (clamped, `slip_count` incremented) and the skip mirror (local resync instead of lapping the reader). Only reachable through a broken slot reading; without it that reading became an out-of-ring write.

Doc corrections from the same audit: the I2S slave parks on every loss (its RELOCKING path is a receiver restart), the ADAT wire offset is 95 or 93 frames once FIFO prefetch is counted, and the Console note above about idle starvation counters.
