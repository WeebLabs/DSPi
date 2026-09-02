# Silent State Changes

*Last updated: 2026-07-19 (live fade completion is observed; stopped producers retain a 130 ms drain floor)*
*Status: implemented; hardware verification pending (see section 9)*

## 1. Purpose

Preset loads, preset saves and deletes, bulk parameter sets, factory resets,
metadata flash writes, and USB stream resyncs used to produce audible pops.
This feature makes those operations silent by construction, without stopping
any output clock or DMA. External mute control (the DAC hardware mute) is now
effectively reserved for the operations that genuinely require a hardware
reconfiguration: output type switches, pin changes, sample-rate changes, and
I2S clock-mode program swaps.

This spec supersedes the exploratory `glitchfree_blackout_spec.md`
(2026-05-13). That document proposed a DMA silence self-chain with an atomic
hot-swap; the mechanism as specified was unimplementable (a single RP2040/
RP2350 DMA channel cannot loop a buffer via `CHAIN_TO == self`; `READ_ADDR`
never self-reloads, and the SDK treats self-chain as "chaining disabled"),
and its supporting machinery (pool drain and re-prime, `MULTI_CHAN_TRIGGER`
hot-swap, 192-frame silence blocks) turned out to be unnecessary. The
implemented design reaches the same goal with far less code and near-zero
RAM.

## 2. The two failure modes that caused pops

**Failure mode A: flash writes stalled the output DMA.** Flash erase and
program make XIP unavailable for roughly 45 ms per write (a preset save does
two: slot then directory). Historically the whole window ran under
`save_and_disable_interrupts()`. With every IRQ masked, the SPDIF/I2S output
DMA handlers could not re-arm; each slot's DMA finished its in-flight
48-sample transfer, the PIO TX FIFO drained in tens of microseconds, and the
output pin held a DC level for the rest of the window. AC-coupled outputs
turned that step into a pop, and SPDIF receivers saw a dead line and
unlocked.

**Failure mode B: the recovery teardown was itself destructive.** After the
operation, `complete_pipeline_reset()` aborted every output DMA, stopped and
restarted every PIO SM, and drained the pools. The restart preserved
inter-slot alignment (synchronized SM start) but was a per-slot discontinuity
on the wire: BMC encoder restart, LRCLK frame restart, receiver unlock and
relock. This teardown also ran for operations that only mutate RAM state
(preset load apply, bulk params, factory reset, stream resync), where no
hardware reconfiguration was needed at all.

## 3. Design summary

Two independent mechanisms, both built on machinery that already existed:

**Pillar A, selective flash IRQ blackout.** During flash erase/program, mask
interrupts at the NVIC instead of via PRIMASK, keeping only the audio output
DMA IRQ lines enabled. The output DMA handlers are fully RAM-resident, so
they keep re-arming through the window. The consumer pools starve within
about 16 ms and the handlers fall back to the existing per-instance silence
buffers, which carry correct IEC 60958 preambles and channel status. Every
slot keeps clocking real, correctly framed silence; nothing is ever aborted;
receivers never unlock.

**Pillar B, no-teardown completions.** Operations whose output topology is
unchanged (no PIO program, clkdiv, or pin change) no longer call
`complete_pipeline_reset()`. They fade fully out on the wire first, apply
their changes, and fade back in, with the outputs clocking continuously
throughout. Inter-slot alignment is preserved by construction because no
output is ever stopped.

## 4. Pillar A: selective flash IRQ blackout

### 4.1 Mechanism

`flash_irq_blackout_begin()` / `flash_irq_blackout_end()` in
`flash_storage.c` replace `save_and_disable_interrupts()` around both flash
critical sections (`flash_write_sector()` and `preset_delete()`'s slot
erase):

- Thread context (the only production path; every runtime flash write is
  deferred to the main loop): save the NVIC ISER state, then write ICER to
  disable everything except
  `DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ` (DMA_IRQ_0, the I2S TX handler line)
  and `DMA_IRQ_0 + PICO_AUDIO_SPDIF_DMA_IRQ` (DMA_IRQ_1, the SPDIF TX
  handler line, shared with SPDIF RX). `__dsb(); __isb();` orders the mask
  before the ROM flash calls. On exit, restore ISER.
- ICER/ISER are written directly rather than through
  `irq_set_mask_n_enabled()`, because the SDK helper clears pending bits on
  re-enable; direct writes preserve IRQs latched during the window (USB,
  timer), matching the old PRIMASK semantics.
- IRQ context (defensive only): fall back to the old PRIMASK blackout,
  since NVIC masking cannot guarantee the DMA handlers preempt the current
  exception frame.
- The register shapes differ per platform (RP2040 scalar `iser`/`icer`,
  RP2350 `iser[2]`/`icer[2]`); both DMA IRQ numbers are below 32 on both
  platforms, so only word 0 needs the keep mask.

Ordering around the window (both call sites):

1. `multicore_lockout_start_blocking()` parks Core 1 (RAM-resident).
2. `pdm_flash_silence()` fills the PDM DMA ring with the modulator's
   0xAAAAAAAA 50% duty silence pattern, so the free-running PDM ring DMA
   outputs true silence instead of looping the last (already faded) audio.
   Content only; the DMA, PIO, and pointers are untouched, so PDM phase is
   continuous.  It also sets `pdm_force_reanchor`: the ring laps an unknown
   number of times while Core 1 is parked, and the loop's modulo write-read
   delta cannot distinguish a multi-lap underrun from a valid lead (the
   `delta > half-ring` test misses roughly half the outcomes), so on resume
   the processing loop unconditionally re-seats its write lead to
   `read + TARGET_LEAD`.  Without this, PDM could resume with an arbitrary
   wrong lead, shifting PDM output by up to ~1024 samples relative to the
   slots.  This hole predates the silent-state-changes work (the old
   teardown path never touched PDM and the `pdm_msg_t.reset` flag is dead
   code); it is fixed here because PDM is in the inviolable alignment set.
3. `flash_irq_blackout_begin()`.
4. `dspi_flash_range_erase()` / `dspi_flash_range_program()`.
5. `flash_irq_blackout_end()`, `multicore_lockout_end_blocking()`.
6. The pre-existing tail re-seeds the USB feedback controller and re-arms
   the mute counter (`flash_mute_hold_samples()`).

### 4.2 Why the handlers are safe while flash is unavailable

Everything reachable from the two output DMA IRQ handlers is RAM-resident
and reads no flash data:

- `audio_spdif_dma_irq_handler` and `audio_i2s_dma_irq_handler` are
  `__isr __time_critical_func`, as are `audio_start_dma_transfer` and the
  pool primitives (`take_audio_buffer`, `give_audio_buffer`, queue ops).
- The IEC channel-status table (`spdif_channel_status[]`) and the silence
  buffers live in RAM (statics inside the instance structs).
- `scripts/check_ram_placement.py` enforces this: Check A (hot symbols in
  RAM), Check B (no branch from the RAM closure into flash), and Check B2,
  which was extended with `FLASH_WINDOW_ROOTS` (the two output handlers) so
  their whole call graphs are also scanned for flash-range literal-pool
  words. B2 additionally fails hard if either flash-window root is ever not
  found in RAM.
- `spdif_rx_dma_irq_handler` shares DMA_IRQ_1 but is deliberately not a B2
  root: SPDIF RX is torn down and its channels masked before every runtime
  flash write (see 4.4), so only the handler's skeleton (RAM code, register
  reads) can execute during the window.
- The timer IRQ stays masked, so the SPDIF RX decode-timeout alarm, whose
  firing on the blackout edge historically crashed the core, cannot fire
  mid-window; it is serviced after restore in a coherent state.
- NVIC masking is per-core; Core 1's lockout FIFO handshake is unaffected.
- `dspi_flash_range_erase/program` do not touch the interrupt state
  themselves; the caller owns it, which is exactly what the selective mask
  relies on.

### 4.3 What the wire carries during a flash window

Per slot and per 48-sample DMA period: real (muted, faded-to-zero) pool
content while the pool lasts, then the per-instance silence buffer. The
handler stamps the Z/X preamble and corrects channel-status bits on every
buffer, including silence, and `subframe_position` keeps advancing, so the
192-frame IEC block structure is continuous across the audio-silence-audio
transitions. I2S slots emit the `I2S_PAD_PATTERN` dither so DAC zero-detect
mutes do not chatter. Every slot emits exactly 48 samples per period
throughout, so inter-slot sample alignment is untouched.

### 4.4 Inputs during a flash window

Unchanged from before this feature:

- SPDIF RX is stopped by `prepare_flash_write_operation()` (or the preset
  load / factory reset / bulk handlers' local suspension) and restarted
  after; the lock-acquisition prefill block owns recovery.
- I2S input is stopped and restarted likewise.
- ADAT input keeps running; its IRQ-less ring laps during the window and the
  poll re-anchors (stall detection), then re-locks.
- USB keeps NAKing; the iso ring is drained after; the feedback controller
  is re-seeded to nominal (SOF was masked).

### 4.5 ADAT bulk output during a flash window (RP2350)

The ADAT bulk output's data/control DMA chain is IRQ-less and free-running,
so it keeps clocking through the window. Two cases:

- Its ring (~18.7 ms at 48 kHz) drains past the write pointer because the
  main loop (its producer) is blocked for ~45 ms. The ring keeps replaying
  stale frames; these are valid pre-encoded NRZI frames whose audio content
  is muted (the settle in 5.1 ran first), so the lightpipe stays framed and
  silent. On resume, the pre-existing lap detection (`adat_need_local_
  resync`) performs a deliberate `adat_output_resync()`.
- While the operation runs with the USB stream active, slot-0 starvations
  are counted (the starvation monitors are enabled whenever the USB alt is
  active) and `adat_output_task` inserts exactly matching silence, keeping
  the ADAT stream slaved to slot time.

Semantic note (from the post-implementation audit): because slot 0 is no
longer restarted on USB silent operations, the ADAT-to-slot phase after a
lap-triggered resync is re-established against slot 0's running stream
rather than against a fresh synchronized start. The offset remains constant
after each resync (1:1 starvation mirroring) but its value is not canonical
across operations. The inviolable alignment constraint (SPDIF instances, I2S
slots, PDM) is unaffected. ADAT receiver interop is hardware-untested in
general; section 9 includes a bench check for this.

## 5. Pillar B: no-teardown silent operations

### 5.1 prepare_silent_operation()

Factored out of `prepare_flash_write_operation()` (which now calls it and
then adds the flash-only input suspensions). It:

1. Drains the active input once (USB ring or input poll).
2. Arms only the soft-mute envelope via `arm_pipeline_soft_mute()`
   (`FLASH_WRITE_PREMUTE_MS` worth of samples, with a Core 1 fence).
3. Services the selected input unconditionally until
   `pipeline_preset_mute_is_silent()` proves a live producer completed the
   100 ms fade, then continues for a 30 ms consumer-queue drain. A stopped
   producer uses the 130 ms queue-drain floor; a 500 ms defensive cap covers a
   malformed producer. The preset/state envelope is an independent final gain
   after every output delay line; this prevents up to 42 ms of already-unfaded
   delayed audio from shortening the fade when the preset handler resets delay
   state.
4. Calls `pipeline_latch_preset_mute_silence()` to pin the packet-driven final
   envelope at zero. Elapsed wall time cannot advance it when no input block
   arrives; the latch guarantees the first post-apply block is muted after the
   old queue has drained.
5. Only after the digital output is silent, asserts the DAC hardware mute,
   floors the soft-mute counter past its attack hold, and continues servicing
   the producer until that hold elapses.

The ordering in step 5 is essential. Previously preset load, factory reset,
and bulk SET pre-gated on `pipeline_reset_ready()`, and
`prepare_pipeline_reset()` asserted hardware mute again when the software
fade was merely armed. A configured DAC therefore began its analog mute at
full signal level, truncating the fade before its first packet; the resulting
discontinuity was exposed when hardware mute released at fade-up. These
handlers no longer pre-gate. Silent preparation owns the hardware hold after
the fade, while still protecting a later clock stop if the restored state
changes output topology.

The signal generator counts as a producer: whenever it is running and no
input source is delivering blocks (any selected source, not just USB), its
idle pump is what fills the pools.  The settle therefore includes a
`siggen_pumping` case and calls `siggen_pump()` in its loop, so generator
audio fades out on the wire like any other source; without this the
envelope would never advance and unfaded old-state generator audio would
sit queued into the operation.  In USB mode the pump already runs through
the mute by design; for non-USB sources it normally refuses while
`preset_loading` is set (that flag doubles as the prefill handshake
signal), so the settle scopes `siggen_settle_pump_override()` around both
loops. The rate-aware pre-mute counter covers the fade phase and is re-floored
for the hardware-mute hold, so settle pumping cannot clear
`preset_loading`.
Preset load and factory reset stop the generator
(`siggen_stop_immediate`) AFTER the settle for the same reason: the stop
then cuts silence, not live tone.  Bulk SET leaves the generator running;
it fades out, the state swaps, and it fades back in with the new state.
On a non-USB source that never (re)locks, the generator stays muted after
the operation until lock or a source change; that is pre-existing
semantics, unchanged here.

Callers then re-fence Core 1 (`core1_eq_fence()`, factored from the
previously duplicated spin-wait) because the settle loop keeps dispatching
EQ work, and only then mutate DSP state.

### 5.2 complete_silent_operation()

The shared completion for operations that kept every slot clocking. Per
active source:

- SPDIF input: `reset_usb_feedback_loop()` only. The pools must not be
  drained; the lock-acquisition prefill block owns the refill and the DAC
  mute release after re-lock.
- USB input: `reset_usb_feedback_loop()` plus
  `release_hw_mute_if_outputs_live()`. Before either, deterministic muted
  fill recovery (5.3) rebuilds consumer lead without a pipeline reset.
- I2S or ADAT input: `complete_pipeline_reset()` is retained. Their bringup
  and prefill machinery still performs a synchronized output restart after
  disruptive operations; migrating them is deferred scope (see section 8).

Bench testing localized a remaining preset-switch click to the beginning of
fade-up. Three independent state discontinuities were present and are now
closed:

1. Hardware mute was asserted before the software fade had processed a
   packet, cutting the analog output at full level. It is now asserted only
   after the observed fade completion and queue drain, and the preset/factory/bulk handlers no
   longer pre-gate on hardware mute.
2. A rate-matched USB host replaces data only as fast as the outputs consume
   it, so the fill gate could wait indefinitely without rebuilding useful
   consumer lead. `usb_silent_fill_recovery()` now extends the mute floor and
   primes zero-input blocks through the complete shared pipeline.
3. Whole-preset coefficient replacement preserved PEQ/crossover recursive
   state and published loudness/crossfeed/psybass/subharm/upmix/leveller coefficients
   on the next main-loop pass. Old recursive energy was therefore interpreted
   by new coefficients, while one post-apply block could see a mixed DSP
   context. Whole-context recalculation now clears PEQ/crossover state, and
   `apply_pending_dsp_updates(true)` publishes and resets every derived
   processor synchronously under the existing mute/Core-1 fence. Direct live
   single-band edits retain their state-preserving behavior.

`process_input_block()` is marked `__attribute__((noclone))`: its new
constant-argument recovery call would otherwise cause GCC to emit a roughly
6.5 KB constprop clone that the XIP RAM closure places in `.data`.

`complete_flash_write_operation_full()` now restarts what the flash bracket
suspended and delegates to `complete_silent_operation()`, so preset save,
preset delete, and the legacy save path inherit the behavior without their
call sites changing.

### 5.3 Fill-gated fade-in

After a flash window or a stream restart the consumer pools refill from
empty while the outputs bridge on the silence fallback. If the envelope
faded in against a near-empty pool, jitter would punch audible gaps into
real audio. A gate alone is insufficient for a rate-matched USB source:
steady-state production and consumption are both 1:1, and the feedback servo
raises fill only slowly.

USB no-teardown completion therefore calls `usb_silent_fill_recovery()` while
`preset_loading` still holds the composite output gain at zero. It extends the
mute floor by two producer blocks plus at least
`USB_SILENT_POST_PRIME_MUTE_MS` (12 ms), then calls
`pipeline_prime_muted_silence()` for at most two 192-sample zero-input blocks,
stopping early when slot 0 reaches `UNMUTE_MIN_FILL`. Each producer block
creates four 48-sample consumer buffers per slot, so two blocks can raise an
empty pool to 8 of 16. The prime uses `process_input_block()` rather than
writing any individual pool; every SPDIF/I2S slot, PDM, delay index, envelope,
and recursive DSP state advances by the same number of samples. Exact
inter-output alignment is therefore preserved.

For preset load specifically, after recovery/priming and any output clocking
rebuild have completed, a continuing USB source floors
`preset_mute_counter` to `PRESET_LOAD_FADE_UP_DELAY_MS` (1000 ms). The
final post-delay gain remains exactly zero for that additional dwell, after which
the normal 8 ms fade-up begins. Presets that change input source skip this
floor because the destination source's prefill handshake owns unmute timing.

The original main-loop gate remains as a bounded fallback. It holds
`preset_mute_counter` at a ~4 ms floor while:

- `preset_loading` is set, and
- the active source is USB with `sync_started`, and
- slot 0's consumer fill is below `UNMUTE_MIN_FILL` (6 of 16 buffers),

bounded per low-fill episode by `UNMUTE_FILL_TIMEOUT_US` (500 ms) so a
struggling stream can never leave the device muted. The gate is a floor
only: it never lowers the counter, never sets `preset_loading`, and cannot
re-mute after the envelope has faded in. Non-USB sources are excluded
because their prefill blocks own their unmute; when the signal generator's
idle pump is the producer it tops fill to 50% itself and passes the gate.

### 5.4 Input servo interaction

`input_servo_apply()` holds its fill-trim (Loop B) at the last value while
`preset_loading` is true, instead of integrating against a starved or
refilling pool; the rate loop and every divider write continue unchanged,
using the frozen trim, so SPDIF/I2S/MCK/ADAT dividers stay mutually
consistent. `input_servo_reset()` clears the held trim.

## 6. Per-operation behavior

| Operation | Before | Now |
|---|---|---|
| Preset save / delete / legacy save | Full teardown + restart; DMA stalled mid-flash | Settle to silence; selective blackout; no teardown (USB/SPDIF source); deterministic USB muted prime plus fill-gated fallback |
| Metadata writes (names, startup, config modes, control surfaces, DAC mute config) | Light path, but DMA stalled mid-flash | Same light path; selective blackout keeps slots clocking; deterministic USB muted prime before release |
| Preset load (no type change) | Immediate apply under a just-armed mute (unfaded audio cut), then full teardown | Settle to silence, fence, reset/publish the whole DSP context, prime USB lead, `complete_silent_operation()` |
| Factory reset (no type change) | Same as preset load | Same as preset load |
| Bulk param SET (no type change) | Same as preset load | Same as preset load (no flash window at all) |
| USB stream resync (alt toggles, format changes) | Full teardown + restart per toggle | Drain, flush, 12 ms envelope re-arm, feedback reset, mute release; no teardown |
| Output type switch, pin change, rate change, I2S clock-mode swap | Full teardown | Unchanged (full teardown, EMC territory) |
| Preset load / factory / bulk WITH type change | Full teardown via `process_type_switches` | Unchanged, but entered with the wire already settled to silence |

The stream-resync change also silences the periodic pops caused by macOS
coreaudiod alt-setting toggles (host IO overload recoveries).

## 7. Alignment argument

The inviolable constraint is sample-level alignment across the SPDIF
instances, I2S slots, and PDM.

- Pillar A never stops or aborts anything. Each output DMA delivers exactly
  48 samples per period (audio or silence) at the shared, phase-locked PIO
  rate; PDM's ring DMA free-runs with only its content rewritten. Alignment
  during and after a flash window is therefore identical to steady-state
  streaming.
- Pillar B removes teardown/restart cycles; it adds no operation that
  touches PIO state, DMA state, clkdivs, or per-slot timing. Muted fill
  recovery runs the ordinary shared block path, advancing every slot, PDM,
  delay index, and DSP state by the same sample count; it never injects into
  an individual output pool. The paths that do touch output hardware
  (type/pin/rate/clock-mode) still use the synchronized teardown/restart
  machinery unchanged.
- The servo hold (5.4) keeps the single divider trim applied uniformly to
  all output types, unchanged from the existing invariant that I2S divider
  = 2x SPDIF divider exactly.

## 8. Deferred scope

- Same-Fs input source switches still run the full rate-change/teardown
  path; migrating them requires reworking the per-source prefill semantics
  (fill-by-silence-injection instead of fill-while-stopped) and the servo
  trim restoration, and was deliberately deferred until this core is
  hardware-proven.
- Flash operations while a non-USB input is live still end in that input's
  re-lock handshake, which restarts outputs; same deferral.
- SPDIF RX teardown across flash could likely be dropped now that the timer
  IRQ is masked through the window (the decode-timeout alarm can no longer
  fire on the edge); kept for now to minimize changed behavior per step.

## 9. Hardware verification checklist

All of this is bench work; the implementation is static-checked only
(builds on all four variants, `check_ram_placement.py` PASS, Opus audit).

1. Preset save x50 while playing a 1 kHz sine over USB: no audible click on
   SPDIF, I2S, or PDM outputs; SPDIF receiver never loses lock (watch the
   receiver's unlock counter, not just audio).
2. Preset load, factory reset, bulk params x50 while playing: no click,
   especially at the beginning of fade-up.
3. Rapid preset cycling (several per second) for 60 s: no accumulated
   artifacts, no stuck mute, envelope always returns to unity.
4. macOS alt-toggle scenario (or manual 2ch/8ch alt flapping): no pops.
5. Inter-slot skew after 100 mixed operations: capture two slots and verify
   0 samples of drift (the starvation counters and `words_consumed` deltas
   can corroborate from `tools/dspi_test`).
6. RP2350 with ADAT output active: verify the lightpipe stays framed
   through preset saves, and measure the ADAT-to-slot-0 offset across
   several saves to characterize the post-resync offset behavior (section
   4.5). Decide then whether a canonical re-anchor is required.
7. SPDIF/ADAT input flash ops: verify re-lock behavior is unchanged.
8. Metadata saves (name set, CS save) while playing: silence throughout.
9. Long-session soak with EMC configured: DAC hardware mute asserts and
   releases correctly around silent operations (no stuck analog mute).

## 10. Cost

- RAM: no new audio buffers; muted recovery reuses `input_bufs` and the
  existing producer/consumer pools. `process_input_block()` remains a single
  RAM-resident body via `noclone`. Standard-build `.data` is 57,184 bytes on
  RP2040 (61,440-byte budget) and 70,600 bytes on RP2350 (73,728-byte budget).
- Flash/code: the recovery and synchronous DSP-state helpers plus comments;
  all state storage already existed.
- Both standard builds pass `check_ram_placement.py` with 0 FAIL. Hardware
  verification remains pending.

## 11. File inventory

| File | Change |
|---|---|
| `firmware/DSPi/flash_storage.c` | `flash_irq_blackout_begin/end()`, both critical sections converted, PDM silence calls |
| `firmware/DSPi/pdm_generator.c/.h` | `pdm_flash_silence()` |
| `firmware/DSPi/input_servo.c` | fill-trim hold while `preset_loading` |
| `firmware/DSPi/main.c` | `core1_eq_fence()`, silent prepare/complete, deterministic USB fill recovery, synchronous derived-DSP publication/reset, resync/preset-load/factory/bulk migrations |
| `firmware/DSPi/audio_pipeline.c/.h` | alignment-safe zero-block priming through the shared pipeline; `process_input_block()` kept as one RAM-resident body |
| `firmware/DSPi/dsp_pipeline.c`, `crossover.c/.h` | clear recursive state on whole-context recalculation while preserving state for direct live edits |
| `scripts/check_ram_placement.py` | `FLASH_WINDOW_ROOTS` in Check B2, missing-root hard fail |
| `Documentation/current_architecture.md` | new "Silent State Changes" section plus updated flash/PDM/USB sections |
