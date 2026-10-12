# Glitch-Free Blackout — Implementation Plan

*Last updated: 2026-05-13*

> **Status (2026-10-12): superseded.** This is a historical design record. It was never implemented as written (see `silent_state_changes_spec.md`), and the pool-based output path it builds on no longer exists. Outputs now play from per-slot free-running DMA rings fed directly by the pipeline. The current design is in `Documentation/current_architecture.md`.

## 1. Executive summary

This document specifies a glitch-free "blackout" mechanism that eliminates
audible pops during preset loads, preset saves, flash directory writes,
input source switches (USB ↔ SPDIF), USB stream resyncs, factory resets,
and any other RAM-only DSP reconfiguration. The mechanism keeps every
PIO state machine, every DMA channel, and every output clock running
continuously while the operation executes — sample-aligned across all
output slots from start to finish.

The current firmware already handles output-type retypes (SPDIF ↔ I2S)
with a synchronized teardown/restart in `complete_pipeline_reset()` and
relies on `audio_spdif_enable_sync()` / `audio_i2s_enable_sync()` to
preserve inter-slot phase alignment. That path is correct for retypes
because the PIO program itself must change, but it is overkill — and
audibly imperfect — for the much larger class of operations that only
mutate DSP state (filter coefficients, matrix routing, delay setpoints,
channel names, etc.) plus an interval where the audio callback cannot
run (flash write).

The proposal here is to add a parallel, opt-in path:

- A **DMA self-chain primitive** that keeps each output's DMA looping
  silence buffers in hardware, with no CPU or IRQ involvement, for the
  entire duration of an operation.
- A **blackout coordinator** (`blackout_enter()` / `blackout_exit()`) that
  fades audio to silence at the DSP layer, hands the DMAs over to the
  self-chain, freezes the audio callback, releases the operation to run,
  then synchronously hands the DMAs back to the producer pool and fades
  audio back in.
- A **state-preservation discipline** that guarantees every piece of
  audio-affecting state (filter IIR states, leveller envelope, delay
  write pointer, master volume ramp endpoint, etc.) survives the
  blackout coherently, with no audible artifact on resume.

Output-type retype keeps its existing path; everything else moves to the
new path. The result: a verifiable, sample-aligned, click-free transition
through every routine reconfiguration the firmware performs at runtime.

The hard constraint from `CLAUDE.md` — **output slot alignment is
inviolable** — is preserved by construction: the DMAs are never
aborted, the PIO clocks never stop, and the synchronized swap-back from
silence to audio uses the same atomic `MULTI_CHAN_TRIGGER` mechanism
that the existing `pio_enable_sm_mask_in_sync()` exploits for clock-cycle
exactness.

---

## 2. Goals and non-goals

### 2.1 Goals

1. **Zero audible artifact** on the seven routine operations enumerated in
   §3.3 (preset load, preset save, preset delete, directory flush, input
   source switch, USB stream resync, factory reset).
2. **Sample-exact inter-slot alignment** preserved across every blackout,
   on both RP2040 and RP2350, for every combination of output types
   (all-SPDIF, all-I2S, mixed, with or without PDM).
3. **Clean, opt-in library extension**: pico-extras audio libraries gain
   a new `audio_*_blackout_enter()` / `audio_*_blackout_exit()` API. The
   existing `audio_*_set_enabled()` / `audio_*_enable_sync()` paths are
   unchanged. Callers that don't opt in keep the current behaviour.
4. **Single-responsibility application layer**: a new `blackout.c` /
   `blackout.h` in `firmware/DSPi/` encapsulates all coordination logic.
   `main.c` and `usb_audio.c` call into it; no blackout state leaks into
   the DSP pipeline or vendor command handlers.
5. **No tech debt**: state ownership is explicit, no globals threaded
   through unrelated code paths, no special-case branches in hot paths.
   Every new field on a struct is documented; every new global has a
   comment explaining its lifecycle.
6. **Backward-compatible**: output-type retype keeps the existing
   teardown/restart path unchanged. Any future operation that needs a
   true teardown (rate change with `clkdiv` discontinuity, master/slave
   role flip, etc.) continues to use `complete_pipeline_reset()`.

### 2.2 Non-goals

1. **Output-type retype**: SPDIF ↔ I2S retype keeps its current path. The
   PIO program changes; some discontinuity is unavoidable on the slot
   being retyped. (A future enhancement could pre-load both PIO programs
   and hot-pivot a single slot's program counter; that is out of scope
   for this spec but called out in §13.)
2. **Rate change**: changes to the sample rate (host SET_INTERFACE,
   SPDIF lock to a different Fs) still go through `perform_rate_change()`.
   PIO `clkdiv` updates require all SMs to be restarted in sync. Blackout
   is invoked around the rate change to soften it, but the SM restart is
   still performed.
3. **Glitch-free DAC analog stage**: this proposal addresses digital-domain
   continuity. AC-coupled analog outputs (line-level RCA) will still see
   a small DC settling time on cold-boot or DAC mute release. The DAC
   hardware-mute pin (already wired) covers that case.
4. **Source-side silence**: this proposal does not attempt to mask
   genuine USB stream silence (host pause, stream stop). Those are
   normal silence transitions; the leveller and other compensation
   stages handle them already.

---

## 3. Current state analysis

### 3.1 What works today

| Mechanism | File:Line | Purpose |
|---|---|---|
| `prepare_pipeline_reset(N)` | main.c:504 | Spin-wait Core 1 EQ worker, latch `preset_loading=true`, set mute counter, assert DAC hw mute |
| `update_preset_mute_envelope()` | audio_pipeline.c:126 | 8 ms linear fade-out / fade-in tied to `preset_loading` |
| `complete_pipeline_reset()` | main.c:642 | Phase-1 per-slot teardown, Phase-2 IRQ-disabled `enable_sync` PIO restart, Phase-3 feedback reset, Phase-4 DAC mute release |
| `audio_spdif_enable_sync()` | audio_spdif.c:520 | Multi-instance synchronized PIO start using `pio_enable_sm_mask_in_sync()` |
| `audio_i2s_enable_sync()` | audio_i2s_multi.c:621 | Same for I2S |
| Per-instance `silence_buffer` | audio_spdif.h:113, audio_i2s_multi.h:85 | Pre-formatted underrun fallback used when consumer pool is empty (only effective with IRQs enabled) |

### 3.2 What's broken (the audible problem)

Two distinct failure modes contribute to today's pops:

**Failure mode A — interrupt-disabled DMA stall during flash writes.**
`flash_write_sector()` (flash_storage.c:387–439) parks Core 1 via
`multicore_lockout_start_blocking()`, then enters a ~45 ms critical
section with all interrupts disabled. During that window:

1. The audio callback (`process_audio_packet()`, main-loop function)
   cannot run — Core 0 is executing the flash routine in RAM.
2. The DMA IRQ handler cannot fire — `irq_set_enabled()` state is
   irrelevant because the global `PRIMASK` is set.
3. Each output's DMA completes its current 48-sample transfer (~1 ms at
   48 kHz) and stalls. The IRQ handler is the only thing that re-arms
   the next transfer; without it, the DMA never restarts.
4. After the DMA stalls, the PIO TX FIFO (4 words deep) drains in
   ~21 µs (SPDIF, 4 bytes/word) to ~83 µs (I2S 32-bit). Then the PIO SM
   blocks on its next `out pins, N` instruction, holding the output pin
   at its last value.
5. The held DC level on the SPDIF / I2S pin lasts ~45 ms until interrupts
   re-enable. AC-coupled outputs (TOSLINK, line-level) generate a step
   transient; DACs see a long zero-frame run that may trigger their
   internal mute-detect.

The existing `silence_buffer` underrun fallback in both libraries does
*not* help here, because that fallback only triggers when the IRQ
handler runs and finds the pool empty. With IRQs disabled, the handler
never runs.

**Failure mode B — `complete_pipeline_reset()` itself is destructive.**
Phase 1 calls `dma_channel_abort()` and `pio_sm_set_enabled(false)` on
every output. Phase 2 calls `pio_enable_sm_mask_in_sync()` to restart.
Even though Phase 2's atomic enable preserves *inter-slot* alignment,
the moment of restart is a discontinuity for *each* slot relative to
its own pre-reset state: the BMC encoder restarts from a fresh
subframe-position-0 boundary, the I2S frame counter restarts from the
LRCLK left edge, and the consumer pool was drained.

For a downstream SPDIF receiver this manifests as a brief unlock-relock
cycle on some chips; for I2S DACs it's a glitch in the bit stream that
may trigger internal de-glitch logic; for line-level analog it's a
discontinuity that the DAC's reconstruction filter smears into an
audible click.

The current DAC hardware-mute pin masks the analog-side click, but
nothing covers the digital-side discontinuity. SPDIF receivers see it.

### 3.3 Operations that need blackout coverage

From the call-site audit (Appendix A), seven operations currently route
through `prepare_pipeline_reset()` / `complete_pipeline_reset()` despite
not requiring PIO teardown:

| # | Operation | File:Line | Why it doesn't need teardown |
|---|---|---|---|
| 1 | Preset load (no type change) | main.c:1412, 1483 | Mutates RAM-only DSP state (coefficients, routing, delays); flash read |
| 2 | Preset save | main.c:1382 area, vendor_commands.c:1312 | Flash erase + program; no DSP changes |
| 3 | Preset delete | vendor_commands.c (similar) | Directory rewrite; no DSP changes |
| 4 | Directory flush | flash_storage.c:565 | Flash erase + program; no DSP changes |
| 5 | Bulk param SET (no rate/type change) | main.c:1625, 1716 | RAM-only DSP state |
| 6 | Input source switch USB→SPDIF or SPDIF→USB (same Fs) | main.c:1731, 1787 | Routing change; PIO `clkdiv` unchanged when both sides agree on Fs |
| 7 | Factory reset (no type change) | main.c:1569 | Restores defaults; same surface as preset load |
| 8 | USB stream resync | main.c:1382 | Producer pool drain; no DSP changes |

Four operations *do* require the existing teardown and remain unchanged
by this proposal:

| Operation | File:Line | Why teardown is required |
|---|---|---|
| Output-type retype (SPDIF ↔ I2S) | main.c:280, 452 | PIO program must change |
| Rate change | main.c:110, 162 | All PIO `clkdiv` must update atomically; SM restart enforces phase reset to new Fs |
| Input source switch crossing Fs | main.c:1731 (via `perform_rate_change`) | Composition of #6 + rate change |
| Cold boot / USB enumeration | usb_audio.c:1568 | First-time PIO+DMA setup |

---

## 4. Architecture overview

### 4.1 Conceptual model

```
                  ┌─────────────────────────────┐
                  │   Audio callback (Core 0)    │
                  │   process_audio_packet()     │
                  │   - DSP pipeline             │
                  │   - Fade gain stage          │ ← preset_mute_smooth_gain
                  │   - Producer pool writes     │
                  └────────────┬────────────────┘
                               │ writes to
                               ▼
                  ┌─────────────────────────────┐
                  │  Per-slot producer pool      │
                  │  (audio buffer ring)         │
                  └────────────┬────────────────┘
                               │
                               │  (Normal path)
                               ▼
              ┌────────────────────────────────────┐
              │  Per-slot DMA channel              │
              │  - Reads buffer from consumer pool │
              │  - IRQ-driven re-arm on completion │
              │  - chain_to = NULL                 │
              └────────────────────┬───────────────┘
                                   │  writes to PIO TX FIFO
                                   ▼
              ┌────────────────────────────────────┐
              │  PIO state machine (SPDIF / I2S)   │
              │  - DREQ-paced from TX FIFO         │
              │  - Identical clkdiv across slots   │
              │  - Phase-locked to sys_clk         │
              └────────────────────┬───────────────┘
                                   │
                                   ▼
                          GPIO output pin
```

The blackout transition replaces only the middle of this stack — the
DMA channel — and only for the duration of the blackout. Source/sink
(producer pool ⇄ PIO ⇄ pin) is unchanged.

### 4.2 The DMA self-chain trick

The RP2040/RP2350 DMA controller supports a `CHAIN_TO` field in each
channel's `CTRL` register. When a channel finishes a transfer (its
`TRANS_COUNT` reaches zero), hardware automatically triggers the channel
named in `CHAIN_TO`. If `CHAIN_TO == channel_index` (self-chain), the
channel re-triggers from its alias-3 register set, which auto-reloads
`TRANS_COUNT` and `READ_ADDR` from previously-stashed values.

This means: with `CHAIN_TO = self`, `READ_ADDR = silence_buffer`, and
`TRANS_COUNT = silence_words`, a DMA channel will continuously loop the
silence buffer with **no CPU intervention**. The IRQ handler doesn't
need to run. Interrupts can be disabled for arbitrary durations. The
PIO TX FIFO is fed forever from sys_clk-paced DREQ.

This is the entire mechanism. Everything else in this spec is plumbing
around it: how to switch the DMA *into* this self-chain state at
blackout entry, how to switch it *out* at exit, how to keep all output
slots aligned across both switches, and how to manage the surrounding
DSP state.

### 4.3 Three-stage transition

```
Time  Stage    PIO/DMA state              DSP state              Audio
──────────────────────────────────────────────────────────────────────────
 t=0  Stage A  Audio running              fade gain ramps 1→0    Audio fading
                                                                  out (~8 ms)
 t=8  Stage B  All DMAs hot-swap to       Audio callback paused  Silence ring
      ms      silence self-chain          OR allowed to run      output (sys_clk
              (atomic, sample-aligned)    with frozen state       paced, no IRQ
                                                                  needed)
              <Blackout operation runs here. Flash write,
               preset apply, source switch, etc. Can be
               40-100+ ms. Interrupts may be disabled
               throughout. DMAs are self-sustaining.>
              
              All DMAs hot-swap back      Producer pool          Fade-in audio
      ~50    to producer pool             pre-filled with        starts
      -100  (atomic, sample-aligned)     fade-in audio
      ms
 t=     Stage C  Audio running              fade gain ramps 0→1    Audio fading
 ~108              IRQ re-arm normal                                in (~8 ms)
 ms
──────────────────────────────────────────────────────────────────────────
```

Stage A and Stage C reuse the existing `preset_mute_smooth_gain`
envelope (audio_pipeline.c:117). Stage B is new.

### 4.4 Synchronized DMA hot-swap

Both transitions — Stage A→B (audio → silence) and Stage B→C (silence →
audio) — must be sample-exact across all output slots. The mechanism is
the same in both directions:

1. Build a bitmask of every slot's DMA channel number.
2. Inside `save_and_disable_interrupts()`:
   - Write `dma_hw->abort = mask` and spin until cleared. All channels
     abort simultaneously. The PIO TX FIFO (4 words deep) provides
     ~21–83 µs of slack while the swap happens.
   - Reconfigure each channel's alias-3 registers (`READ_ADDR`,
     `TRANS_COUNT`, `CTRL`) for the new state (silence self-chain or
     audio with IRQ re-arm).
   - Write `dma_hw->multi_channel_trigger = mask`. All channels start in
     the same sys_clk cycle.
3. `restore_interrupts()`.

Sample alignment is preserved because:

- All DMAs are aborted in the same cycle.
- All DMAs are triggered in the same cycle.
- The PIO SMs were already phase-locked (they were running and never
  stopped); they continue to consume from their respective TX FIFOs at
  identical rates.
- The first word each DMA delivers to its PIO TX FIFO after the
  re-trigger corresponds to the same logical sample index N across all
  slots.

The PIO TX FIFO depth (4 words) buffers the brief ~1 µs swap window —
PIO never stalls.

### 4.5 Why this is safer than `complete_pipeline_reset()`

| Aspect | `complete_pipeline_reset()` | Blackout (this spec) |
|---|---|---|
| PIO SM running across transition? | No — stopped and restarted | Yes — never stops |
| BMC encoder state reset? | Yes — restarts at subframe 0 | No — continuous |
| I2S frame counter reset? | Yes — restarts at LRCLK edge | No — continuous |
| Producer pool drained? | Yes | No — frozen, restored |
| Consumer pool drained? | Yes | No — DMA buffers remain valid |
| DAC sees discontinuity? | Yes (digital) | No |
| Window where DMA stalls? | Sub-millisecond | None |
| Window where PIO is idle? | ~1 ms | None |
| Inter-slot sample alignment | Preserved via `enable_sync` | Preserved via `multi_channel_trigger` |
| USB feedback reset? | Yes | Optional (typically no) |

---

## 5. Component designs

This section specifies each module separately. All new application-layer
code lives in `firmware/DSPi/blackout.c` and `firmware/DSPi/blackout.h`.
Library extensions live in the existing `pico_audio_spdif_multi` and
`pico_audio_i2s_multi` libraries.

### 5.1 Silence-buffer format

#### 5.1.1 SPDIF silence buffer

Per-instance, allocated by `audio_spdif_setup()` (already exists as
`inst->silence_buffer`). Content must be valid IEC 60958-1 subframes:

- Audio data: a small non-zero pattern to defeat receiver mute-detect.
  Recommended: alternating `+1, -1` (24-bit), encoded BMC, generating a
  Nyquist-frequency signal at ~144 dBFS — completely inaudible but
  guarantees the data field is non-zero in every subframe.
- Preamble: rotated correctly by the IRQ handler's existing logic at
  audio_spdif.c:388–401, which stamps the Z/X preamble and channel
  status bits based on `inst->subframe_position`. **This is already
  correct in the existing fallback path** and must remain correct on
  the self-chain path.
- Validity (V), User (U), Channel status (C), Parity (P) bits: must
  match the IEC 60958-1 channel-status frame. Already handled by
  `init_spdif_buffer()` (audio_spdif.c:322) at pool-init time.

**Challenge with self-chain**: when the DMA self-chains, the IRQ handler
does *not* run. The Z/X preamble re-stamp logic at audio_spdif.c:388 does
not execute. So the same 48-sample buffer is replayed repeatedly without
the per-block preamble correction.

**Mitigation**: pre-construct a special silence buffer whose length is
exactly one IEC 60958-1 block (192 samples = 384 subframes). Stamp the
Z preamble at subframe 0 and X preambles at subframes 1..191, and
populate the full 192-frame channel status sequence statically. When
the DMA loops this buffer, every iteration starts at subframe 0 (Z
preamble), so the block alignment is naturally preserved across
arbitrary loop counts. The `inst->subframe_position` counter does not
advance during blackout (no IRQ); on exit we resume from position 0
which matches the silence buffer's terminal state.

This buffer is 192 × 8 = 1536 bytes per SPDIF instance. With 2
instances on RP2040 and 4 on RP2350, total cost is 3 KB / 6 KB. We
allocate it once at setup, statically. It replaces the existing
`inst->silence_buffer` (which today is just 48 samples and shares the
generic underrun-fallback role).

#### 5.1.2 I2S silence buffer

Per-instance. Content: I2S frames with the 24-bit audio data set to
the dither pattern `0x000001 / 0xFFFFFF` alternating, OR'd with the
existing `I2S_PAD_PATTERN (0x01)` low byte. This produces a sample at
±1 LSB, well below audibility.

Length: a single 48-sample DMA buffer is fine for I2S — there's no
192-frame block structure to preserve. The DMA self-chains on the same
48-sample buffer indefinitely.

Memory cost: 48 × 8 = 384 bytes per instance. With up to 4 instances,
total ~1.5 KB.

#### 5.1.3 PDM "silence" handling

PDM already runs in DMA ring mode (pdm_generator.c:144–151) with
`transfer_count = 0xFFFFFFFF` and a 256-bit ring buffer continuously
filled by Core 1's sigma-delta encoder. During blackout:

- Core 1 is parked (multicore lockout for flash ops; gated by the same
  blackout flag for non-flash ops).
- The 256-bit PDM ring is no longer refreshed.
- The DMA continues to cycle the existing buffer contents — which is
  the last non-silence output Core 1 produced.

This is a **bug today** as well: PDM during flash will replay the last
~5 ms of audio in a loop. The fix is to pre-fill the PDM ring with
silence (zero bits at the 1-bit modulator output) before parking Core 1.

A clean fix: extend `blackout_enter()` to write zeros into the PDM ring
buffer (a single 32-byte memset). On exit, Core 1 resumes filling the
ring with real audio.

### 5.2 DMA hot-swap primitive

#### 5.2.1 Library API additions

In `pico_audio_spdif_multi/include/pico/audio_spdif.h` and
`pico_audio_i2s_multi/include/pico/audio_i2s_multi.h`, add the
following symmetric API:

```c
/** \brief Hot-swap a set of instances' DMAs into silence self-chain mode.
 *
 * Atomically aborts each instance's current DMA, reconfigures it to loop
 * `inst->silence_buffer` via CHAIN_TO=self, and re-triggers all channels
 * in the same sys_clk cycle using MULTI_CHAN_TRIGGER.
 *
 * Sample alignment across all listed instances is preserved by the atomic
 * abort + trigger sequence under save_and_disable_interrupts().
 *
 * The instances' PIO state machines MUST remain running. Do NOT call
 * pio_sm_set_enabled(false) on these SMs around this call.
 *
 * The DMA IRQ for these channels is masked by this function. On exit,
 * audio_*_blackout_exit() unmasks it.
 *
 * After this call, the instances are in "blackout" state. The producer
 * pool and consumer pool are not touched; the audio callback may keep
 * running and writing to the producer pool, or may pause — either is
 * valid. The DMA is no longer pulling from the consumer pool.
 *
 * \param instances Array of pointers to instances (all must be enabled)
 * \param count     Number of instances
 */
void audio_spdif_blackout_enter(audio_spdif_instance_t *instances[], uint count);

/** \brief Hot-swap a set of instances' DMAs back to producer-pool mode.
 *
 * Symmetric counterpart to blackout_enter. Atomically aborts the silence
 * self-chain, reconfigures each DMA to consume from the consumer pool
 * (initial transfer pre-armed), unmasks the DMA IRQ, and triggers all
 * channels in the same sys_clk cycle.
 *
 * On entry the consumer pool MUST contain at least one prepared buffer
 * per instance. Callers achieve this by enabling the audio callback for
 * a few packets with fade-in gain == 0 before calling this function (the
 * preset_mute envelope makes those packets silent), so the DMA's first
 * post-blackout transfer is muted audio.
 *
 * \param instances Array of pointers to instances (all in blackout state)
 * \param count     Number of instances
 */
void audio_spdif_blackout_exit(audio_spdif_instance_t *instances[], uint count);

/* Same signatures in audio_i2s_multi.h for I2S */
void audio_i2s_blackout_enter(audio_i2s_instance_t *instances[], uint count);
void audio_i2s_blackout_exit(audio_i2s_instance_t *instances[], uint count);
```

#### 5.2.2 Implementation: `audio_spdif_blackout_enter()`

Pseudocode (real implementation in audio_spdif.c). The I2S version is
nearly identical with the I2S `silence_buffer` and word counts.

```c
void audio_spdif_blackout_enter(audio_spdif_instance_t *instances[], uint count) {
    assert(count > 0 && count <= PICO_AUDIO_SPDIF_MAX_INSTANCES);

    // Build the channel bitmask for atomic abort + trigger.
    uint32_t mask = 0;
    for (uint i = 0; i < count; i++) {
        mask |= (1u << instances[i]->dma_channel);
    }

    // Disable interrupts for the swap. The PIO TX FIFO depth (4 words)
    // gives us ~21 µs of slack before PIO would underflow; we need only
    // a few hundred ns to do the reconfig + retrigger.
    uint32_t save = save_and_disable_interrupts();

    // Mask each channel's DMA IRQ so the handler does not see the abort
    // as a completion and try to re-arm. The handler doesn't read this
    // mask directly — it filters via dma_irqn_get_channel_status — but
    // masking here is belt-and-braces and matches teardown_output_slot.
    for (uint i = 0; i < count; i++) {
        audio_spdif_instance_t *inst = instances[i];
        dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, false);
    }

    // Atomic abort: all listed channels stop in the same cycle.
    // dma_hw->abort is a write-1-to-trigger register; spin until the
    // hardware ack clears the bits.
    dma_hw->abort = mask;
    while (dma_hw->abort & mask) tight_loop_contents();

    // Return any in-flight playing_buffer to the consumer pool. This
    // mirrors teardown_output_slot's logic so we don't leak buffers.
    // It is safe because the DMA is now stopped and the IRQ is masked.
    for (uint i = 0; i < count; i++) {
        audio_spdif_instance_t *inst = instances[i];
        if (inst->playing_buffer) {
            give_audio_buffer(inst->consumer_pool, inst->playing_buffer);
            inst->playing_buffer = NULL;
        }
        // Ack any pending IRQ flag so the handler, on resume, doesn't
        // see a stale completion from the abort.
        dma_irqn_acknowledge_channel(inst->dma_irq, inst->dma_channel);
    }

    // Reconfigure each channel for silence self-chain.
    // - READ_ADDR: silence_buffer start
    // - TRANS_COUNT: silence_buffer_words (one full 192-frame block)
    // - CTRL: chain_to = self, ring mode off (we use chain re-trigger
    //   instead so we can reload TRANS_COUNT and READ_ADDR via alias-3)
    for (uint i = 0; i < count; i++) {
        audio_spdif_instance_t *inst = instances[i];
        dma_channel_config c = dma_channel_get_default_config(inst->dma_channel);
        channel_config_set_dreq(&c, pio_get_dreq(inst->pio, inst->pio_sm, true));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        // chain_to = self: on completion, hardware re-triggers this same
        // channel and reloads READ_ADDR + TRANS_COUNT from the alias-3
        // shadow registers, which auto-reload from our last-write values.
        channel_config_set_chain_to(&c, inst->dma_channel);

        // Use alias-3 to set CTRL last (the trigger):
        //   AL3_READ_ADDR   = silence buffer start
        //   AL3_TRANS_COUNT = silence words
        //   AL3_CTRL_TRIG   = config | trigger (written last to start)
        // But we're not triggering yet — that's done atomically below
        // via multi_channel_trigger. So write CTRL via AL3_CTRL (no trig).
        dma_channel_hw_addr(inst->dma_channel)->al3_read_addr =
            (uintptr_t)inst->silence_buffer.buffer->bytes;
        dma_channel_hw_addr(inst->dma_channel)->al3_transfer_count =
            inst->silence_buffer.sample_count * 4;  // 4 DMA words per stereo sample
        // Write CTRL last, but to non-trigger alias.
        dma_channel_hw_addr(inst->dma_channel)->al1_ctrl = channel_config_get_ctrl_value(&c);
    }

    // Atomic multi-channel trigger: all channels start in the same cycle.
    dma_hw->multi_channel_trigger = mask;

    restore_interrupts(save);

    // Mark instances as in-blackout for diagnostic and bookkeeping.
    for (uint i = 0; i < count; i++) {
        instances[i]->in_blackout = true;  // new field, see §5.2.4
    }
}
```

Key correctness points (each commented in the real code):

1. **Why `save_and_disable_interrupts()` is required**: ensures the DMA
   IRQ handler cannot fire mid-swap and re-arm with stale state.
2. **Why we mask the per-channel DMA IRQ inside the critical section**:
   teardown_output_slot's comment at main.c:533–558 explains the same
   race; we follow that pattern.
3. **Why we use `al3_read_addr`/`al3_transfer_count` and `al1_ctrl`**:
   alias-3 is the auto-reload set used by `chain_to`. When the DMA
   completes and chain_to triggers itself, hardware reads from alias-3
   to reload, so we want our silence values there. `al1_ctrl` is the
   non-triggering alias for CTRL — writing it does not start the DMA.
   We then trigger via `multi_channel_trigger`.
4. **Why we ack the per-channel DMA IRQ flag**: a final completion from
   the aborted transfer may have set the `ints` bit; we clear it so the
   handler doesn't see a phantom completion on exit.
5. **Inter-slot alignment proof**: all channels' final triggers happen
   in the same sys_clk cycle via `multi_channel_trigger`. The PIO SMs are
   already phase-locked. The PIO TX FIFO drain rate is identical
   across slots (same clkdiv, same encoding width). Therefore all
   slots' silence-buffer playback starts on the same logical sample
   boundary.

#### 5.2.3 Implementation: `audio_spdif_blackout_exit()`

Symmetric:

```c
void audio_spdif_blackout_exit(audio_spdif_instance_t *instances[], uint count) {
    assert(count > 0 && count <= PICO_AUDIO_SPDIF_MAX_INSTANCES);
    for (uint i = 0; i < count; i++) {
        assert(instances[i]->in_blackout);
    }

    uint32_t mask = 0;
    for (uint i = 0; i < count; i++) {
        mask |= (1u << instances[i]->dma_channel);
    }

    uint32_t save = save_and_disable_interrupts();

    // Atomic abort the silence self-chain.
    dma_hw->abort = mask;
    while (dma_hw->abort & mask) tight_loop_contents();

    // Reconfigure each DMA to consume from the consumer pool.
    // Caller has guaranteed at least one prepared buffer per instance.
    for (uint i = 0; i < count; i++) {
        audio_spdif_instance_t *inst = instances[i];

        // Take the first audio buffer for playback.
        audio_buffer_t *ab = take_audio_buffer(inst->consumer_pool, false);
        if (!ab) {
            // Defensive: fall back to silence_buffer for this slot.
            // This indicates the caller did not pre-fill — log and
            // continue. The fade-in envelope will hide one cycle of
            // silence regardless.
            ab = &inst->silence_buffer;
        }
        inst->playing_buffer = (ab == &inst->silence_buffer) ? NULL : ab;

        // Re-stamp preamble + channel status for this buffer's
        // subframe position. Mirrors audio_start_dma_transfer.
        stamp_spdif_buffer(inst, ab);  // factored out — see §5.2.5

        // Configure DMA back to normal mode: chain_to = NONE,
        // IRQ-driven re-arm.
        dma_channel_config c = dma_channel_get_default_config(inst->dma_channel);
        channel_config_set_dreq(&c, pio_get_dreq(inst->pio, inst->pio_sm, true));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        // chain_to = self == disable chain (per SDK convention).
        channel_config_set_chain_to(&c, inst->dma_channel);

        uint32_t transfer_words = ab->sample_count * 4;
        inst->current_transfer_words = transfer_words;

        dma_channel_hw_addr(inst->dma_channel)->al3_read_addr =
            (uintptr_t)ab->buffer->bytes;
        dma_channel_hw_addr(inst->dma_channel)->al3_transfer_count =
            transfer_words;
        dma_channel_hw_addr(inst->dma_channel)->al1_ctrl =
            channel_config_get_ctrl_value(&c);

        // Unmask the per-channel DMA IRQ so the handler will re-arm
        // subsequent buffers normally.
        dma_irqn_set_channel_enabled(inst->dma_irq, inst->dma_channel, true);
    }

    // Atomic trigger — all channels start fade-in audio in the same cycle.
    dma_hw->multi_channel_trigger = mask;

    restore_interrupts(save);

    for (uint i = 0; i < count; i++) {
        instances[i]->in_blackout = false;
    }
}
```

#### 5.2.4 Instance-struct additions

Each library's instance struct gets one new field:

```c
typedef struct audio_spdif_instance {
    /* ... existing fields ... */

    /* In-blackout state.  Set by audio_spdif_blackout_enter, cleared by
     * audio_spdif_blackout_exit.  Diagnostic only — the DMA hardware
     * state (chain_to, read_addr, etc.) is the actual source of truth.
     * Used by the DMA IRQ handler to short-circuit if a stale completion
     * fires after blackout entry but before the channel IRQ mask takes
     * effect. */
    bool in_blackout;
} audio_spdif_instance_t;
```

The IRQ handler at audio_spdif.c:412 gains one check:

```c
void __isr __time_critical_func(audio_spdif_dma_irq_handler)() {
    for (uint i = 0; i < spdif_instance_count; i++) {
        audio_spdif_instance_t *inst = spdif_instances[i];
        if (inst->in_blackout) continue;          // <-- new
        if (dma_irqn_get_channel_status(inst->dma_irq, inst->dma_channel)) {
            /* ... existing handler body ... */
        }
    }
}
```

This is defensive — the per-channel IRQ should already be masked during
blackout — but it provides a clean second line of defence against any
stale-completion race.

#### 5.2.5 Silence buffer construction

Factor the existing per-buffer stamping logic at audio_spdif.c:388–401
into a helper `stamp_spdif_buffer(inst, ab)`. Then construct the
silence buffer at `audio_spdif_setup()` time:

```c
// In audio_spdif_setup, after consumer_pool allocation:
//
// The silence buffer is a single 192-sample IEC 60958-1 audio block,
// containing alternating +1/-1 24-bit audio data, BMC-encoded into
// subframes with valid Z/X preambles and channel-status bits.  The
// DMA self-chains this buffer indefinitely during blackout.
//
// Why one full 192-sample block: when the DMA loops, every loop
// iteration starts at subframe 0 (Z preamble).  Block-position
// coherency is naturally preserved across arbitrary loop counts.
// This is critical because the IRQ handler does NOT run during
// blackout to re-stamp preambles on a smaller buffer.

inst->silence_buffer.sample_count = PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT;  // 192
inst->silence_buffer.buffer = audio_new_buffer(2 * sizeof(spdif_subframe_t),
                                                 PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT);
inst->silence_buffer.format = &inst->consumer_buffer_format;

// Populate every subframe with audio data = ±1 (Nyquist square at 1 LSB).
for (uint i = 0; i < PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT; i++) {
    int32_t s = (i & 1) ? 1 : -1;  // 24-bit ±1
    spdif_subframe_t *sf = &((spdif_subframe_t *)inst->silence_buffer.buffer->bytes)[i * 2];
    encode_subframe(sf, /*channel=*/0, /*audio=*/s, /*frame_idx=*/i);
    encode_subframe(sf + 1, /*channel=*/1, /*audio=*/s, /*frame_idx=*/i);
}
// Stamp Z preamble at subframe 0, X preamble at 1..191 (existing
// helper handles this when subframe_position == 0).
```

#### 5.2.6 Rationale for `MULTI_CHAN_TRIGGER`

The RP2040 datasheet §2.5 documents `MULTI_CHAN_TRIGGER` at offset 0x430
of the DMA block: "Trigger one or more channels simultaneously". Writing
a bitmask sets the corresponding `BUSY` bits in the same cycle, starting
all listed channels on the same sys_clk edge. The RP2350 datasheet
preserves this register at the same offset.

This is the same mechanism the SDK's `dma_channel_start()` uses
internally, just for multiple channels at once.

### 5.3 Blackout coordinator API

New file: `firmware/DSPi/blackout.h`:

```c
#ifndef DSPI_BLACKOUT_H
#define DSPI_BLACKOUT_H

#include <stdint.h>
#include <stdbool.h>

/* The blackout coordinator drives the DSP fade envelope, parks the audio
 * callback, hot-swaps the DMA hardware to silence self-chain, and on
 * exit hands the DMAs back to the producer pool with a fade-in.
 *
 * Lifecycle:
 *
 *     blackout_enter(BLACKOUT_REASON_PRESET_LOAD);
 *     ... operation (flash write, source switch, RAM-only mutation) ...
 *     blackout_exit();
 *
 * Both calls block until the corresponding transition completes:
 *
 *   blackout_enter() blocks for ~8 ms — the duration of the DSP
 *   fade-out envelope — before returning.  This guarantees the
 *   producer pool's most recent buffers are sample-by-sample fading
 *   toward zero when the DMA hot-swap to silence happens.
 *
 *   blackout_exit() blocks for ~8 ms — the fade-in envelope — after
 *   the DMA hot-swap back to producer-pool mode.  This guarantees the
 *   first buffers consumed from the producer pool are sample-by-sample
 *   fading from zero, so there is no step at the swap boundary.
 *
 * Idempotency / nesting:
 *
 *   Blackouts are NOT nestable.  Calling blackout_enter() while
 *   already in blackout asserts.  Callers must ensure single-threaded
 *   access (today: all callers are on Core 0 main loop).
 *
 *   blackout_exit() without a prior blackout_enter() asserts.
 *
 * Reason enum is informational, used for diagnostic logging and to
 * select reason-specific tuning (e.g. longer mute hold for flash ops).
 */

typedef enum {
    BLACKOUT_REASON_PRESET_LOAD       = 0,
    BLACKOUT_REASON_PRESET_SAVE       = 1,
    BLACKOUT_REASON_PRESET_DELETE     = 2,
    BLACKOUT_REASON_DIR_FLUSH         = 3,
    BLACKOUT_REASON_BULK_PARAMS       = 4,
    BLACKOUT_REASON_SOURCE_SWITCH     = 5,
    BLACKOUT_REASON_STREAM_RESYNC     = 6,
    BLACKOUT_REASON_FACTORY_RESET     = 7,
    BLACKOUT_REASON_COUNT
} blackout_reason_t;

/** Enter blackout.  Blocks for the fade-out duration before returning.
 *  On return, all output DMAs are looping silence in hardware; the
 *  audio callback is in a known-safe state (DSP fade gain at zero);
 *  Core 1 is parked (work_ready clear, EQ worker idle).
 *
 *  The caller may now perform any RAM-only mutation or interrupt-
 *  disabled critical section.  PIO clocks stay running and DMAs
 *  stay fed in hardware throughout.
 *
 *  Asserts: !blackout_is_active().
 *
 *  \param reason  Informational, used for diagnostics and tuning.
 */
void blackout_enter(blackout_reason_t reason);

/** Exit blackout.  Pre-fills the consumer pool with fade-in audio,
 *  hot-swaps DMAs back to producer-pool mode synchronously across all
 *  slots, then blocks until the fade-in envelope completes.
 *
 *  On return, the firmware is back in normal operation.
 *
 *  Asserts: blackout_is_active().
 */
void blackout_exit(void);

/** Query the current blackout state.  Used by code that needs to take
 *  a different path while a blackout is in progress (e.g. a vendor
 *  command handler that should defer processing). */
bool blackout_is_active(void);

/** Initialise blackout-coordinator state.  Called from main() once at
 *  boot, after all output instances are set up but before the audio
 *  callback runs. */
void blackout_init(void);

#endif
```

### 5.4 Blackout coordinator implementation sketch

`firmware/DSPi/blackout.c` (real implementation expanded inline; this
sketch shows the structure and the comments that go in the file):

```c
#include "blackout.h"
#include "audio_pipeline.h"
#include "flash_storage.h"
#include "pdm_generator.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "config.h"

// Per-reason fade-out / fade-in tuning.  Most operations use the
// default; flash operations extend the fade slightly (one extra packet)
// so the hardware DAC mute can engage before the DMA hot-swap.
typedef struct {
    uint32_t fade_out_samples;
    uint32_t fade_in_samples;
} blackout_tuning_t;

static const blackout_tuning_t blackout_tuning[BLACKOUT_REASON_COUNT] = {
    [BLACKOUT_REASON_PRESET_LOAD]   = { .fade_out_samples = 384, .fade_in_samples = 384 },
    [BLACKOUT_REASON_PRESET_SAVE]   = { .fade_out_samples = 480, .fade_in_samples = 384 },
    /* ... etc ... */
};

static volatile bool s_blackout_active = false;
static blackout_reason_t s_blackout_reason;

bool blackout_is_active(void) { return s_blackout_active; }

void blackout_init(void) {
    s_blackout_active = false;
}

void blackout_enter(blackout_reason_t reason) {
    assert(!s_blackout_active);
    s_blackout_reason = reason;

    // ===== Stage A: DSP fade-out =====
    //
    // Engage the existing preset-mute envelope.  This sets
    // preset_loading=true (audio_pipeline.c reads it) and seeds the
    // mute counter so the smoothing envelope ramps from 1.0 to 0.0
    // over PRESET_MUTE_TRANSITION_MS (8 ms).
    //
    // We use prepare_pipeline_reset() so the spin-wait for the Core 1
    // EQ worker happens here, and so the DAC hardware mute asserts
    // alongside the soft envelope — covering both the digital fade-out
    // and any DAC analog pop.
    prepare_pipeline_reset(blackout_tuning[reason].fade_out_samples);

    // Wait for the fade-out envelope to actually reach zero before we
    // hot-swap the DMA.  If we swap before the fade reaches zero, the
    // last few samples in the consumer pool are non-silent, and the
    // moment of swap shows a step from "fading audio" to "silence
    // pattern".  By waiting we guarantee the swap happens between two
    // genuinely silent samples.
    //
    // The envelope runs in the audio callback context, decrementing
    // preset_mute_counter by sample_count per packet.  When counter
    // reaches zero, preset_loading is cleared.  But we WANT
    // preset_loading to stay true throughout the blackout, so we
    // observe the envelope's gain directly instead.
    //
    // Implementation: poll preset_mute_smooth_gain until it reads
    // <= 0.001f.  Bounded by ~16 ms in the worst case (8 ms ideal +
    // 8 ms slack for a stuck audio callback).
    blackout_wait_fade_out_complete();  // see §5.4.1

    // ===== Stage B: DMA hot-swap to silence =====
    //
    // Build the per-type instance arrays from output_types[].
    //
    // The current configuration may be mixed (some slots SPDIF, some
    // I2S).  We hot-swap each type separately via its library's
    // blackout_enter call.  Inter-slot alignment IS preserved across
    // types because (a) the PIO SMs remain running and are sys_clk-
    // locked, and (b) each library's blackout_enter uses MULTI_CHAN_
    // TRIGGER atomically within its instance set.
    //
    // The CROSS-type alignment edge: SPDIF and I2S DMAs do not share
    // a multi_channel_trigger write.  But their PIO SMs were started
    // together (via complete_pipeline_reset's combined enable_sync)
    // and have not stopped since.  Their TX FIFOs are sys_clk-paced
    // and identical-rate.  The hot-swap is a per-channel abort +
    // trigger pair, lasting a few hundred ns, buffered by the 4-deep
    // TX FIFO.  The cross-type trigger writes happen at different
    // sys_clk cycles (because they're separate save_and_disable_
    // interrupts blocks), but within the TX FIFO's drain budget
    // (~21 µs for SPDIF, ~83 µs for I2S).  So the worst-case skew is
    // bounded by the time between the two trigger writes — which we
    // can drive to a few hundred ns by doing them back-to-back
    // without releasing interrupts between them.
    //
    // To enforce this, we wrap BOTH library calls in a single outer
    // save_and_disable_interrupts() block (see §5.4.2).

    blackout_hot_swap_to_silence();

    // Pre-zero the PDM ring so PDM output is genuine silence during
    // the blackout (otherwise it loops the last ~5 ms of audio).
    pdm_ring_silence();

    // ===== Stage B (cont.): Park audio processing =====
    //
    // For flash operations, the caller will disable interrupts and
    // park Core 1 via multicore_lockout_start_blocking() after we
    // return.  We don't need to park Core 1 here — the lockout takes
    // care of that.
    //
    // For non-flash operations (RAM-only mutation), the audio callback
    // may continue to run.  Producer pool keeps getting filled with
    // (silent, fade-gain-zero) buffers.  Those buffers are queued but
    // not consumed by DMA (which is on silence self-chain).  This is
    // fine — the consumer pool's prepared list will fill up; eventually
    // take_audio_buffer() will return NULL and the callback will count
    // it as an overrun.  Overruns during blackout are benign and
    // should not be counted; the callback already checks preset_loading
    // before incrementing the overrun counter (audio_pipeline.c:203).
    //
    // So we just leave the callback running (or not) and return.

    s_blackout_active = true;
}

void blackout_exit(void) {
    assert(s_blackout_active);

    // ===== Stage C: Hot-swap back, then fade-in =====
    //
    // The producer pool may contain stale buffers from before the
    // blackout (if the audio callback ran during the blackout, they
    // are fade-gain-zero silence) OR may be empty.  Either way, we
    // need to ensure at least one prepared buffer per output slot
    // before calling blackout_exit on the libraries, because the
    // libraries' blackout_exit takes the first buffer from the
    // consumer pool to load into the DMA.
    //
    // Easiest, most predictable approach: clear the consumer pool's
    // prepared list (drop any stale buffers), reset preset_mute_counter
    // to inject a fresh fade-in, then run one or two audio callback
    // iterations to populate the consumer pool with genuine fade-in-
    // gain-zero buffers.
    //
    // Note: at this point, preset_loading is still true (we have not
    // expired the counter).  So the next audio callback iteration
    // produces fade-gain-zero output.  After we hand the DMA back,
    // we re-arm the counter for the fade-in transition, and the
    // envelope ramps 0→1 over PRESET_MUTE_TRANSITION_MS.

    blackout_prime_consumer_pools();  // §5.4.3

    blackout_hot_swap_to_audio();     // calls audio_*_blackout_exit

    // Trigger fade-in: clear preset_loading and let the envelope
    // ramp up.  We don't immediately set preset_loading=false because
    // process_audio_packet() reads it at the top of each packet; we
    // arrange for the next packet to see it false and start ramping.
    //
    // Implementation: zero the mute counter so the next call to
    // update_preset_mute_envelope() sees preset_mute_counter==0 and
    // (with mute_active_for_packet==true, the latched state) clears
    // preset_loading and starts the fade-in.
    blackout_trigger_fade_in();

    // Block until the fade-in envelope reaches 1.0 so callers can
    // be sure normal operation has resumed before they return to
    // their caller (e.g. a vendor command handler that wants to ACK
    // only after audio is restored).
    blackout_wait_fade_in_complete();

    // Release DAC hardware mute now that audio is at full gain.
    // (dac_hw_mute_release is idempotent.)
    dac_hw_mute_release();

    s_blackout_active = false;
}
```

#### 5.4.1 `blackout_wait_fade_out_complete()`

Watch the existing `preset_mute_smooth_gain` static in audio_pipeline.c.
Two options for visibility:

- **Option A**: make `preset_mute_smooth_gain` non-static and export it
  via a getter in audio_pipeline.h: `float audio_pipeline_get_mute_gain(void)`.
- **Option B**: spin on `preset_mute_counter == 0` (already public in
  flash_storage.h:118) plus a small fixed delay to let the final
  packet's envelope settle.

Option A is cleaner. We add:

```c
// In audio_pipeline.h:
float audio_pipeline_get_mute_gain(void);

// In audio_pipeline.c:
float audio_pipeline_get_mute_gain(void) {
    return preset_mute_smooth_gain;
}
```

Then in blackout.c:

```c
static void blackout_wait_fade_out_complete(void) {
    // Bound at 32 ms — much longer than the envelope, but a safety
    // net in case the audio callback has stalled (e.g. USB just
    // disconnected and we're awaiting a packet that won't arrive).
    absolute_time_t deadline = make_timeout_time_ms(32);
    while (audio_pipeline_get_mute_gain() > 0.001f) {
        if (time_reached(deadline)) break;
        tight_loop_contents();
    }
}
```

#### 5.4.2 `blackout_hot_swap_to_silence()` — combined SPDIF + I2S

```c
static void blackout_hot_swap_to_silence(void) {
    // Collect per-type instance arrays from the active configuration.
    audio_spdif_instance_t *spdif_set[NUM_SPDIF_INSTANCES];
    audio_i2s_instance_t *i2s_set[NUM_SPDIF_INSTANCES];
    uint spdif_count = 0;
    uint i2s_count = 0;

    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (inst && inst->enabled) i2s_set[i2s_count++] = inst;
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (inst && inst->enabled) spdif_set[spdif_count++] = inst;
        }
    }

    // Wrap BOTH library calls in a single outer save_and_disable_
    // interrupts.  The libraries each have their own inner save_and_
    // disable around the multi_channel_trigger write, but the OUTER wrap
    // brackets the SPDIF↔I2S boundary so the two trigger writes happen
    // a handful of sys_clk cycles apart — well within the PIO TX FIFO
    // drain budget of either type.  Without this outer wrap, an
    // interrupt between the SPDIF and I2S calls could let one type's
    // DMA complete an extra transfer before the other's swap, breaking
    // inter-type alignment.
    //
    // This mirrors the design of complete_pipeline_reset (main.c:666)
    // which wraps its two enable_sync calls in the same way.
    uint32_t save = save_and_disable_interrupts();
    if (spdif_count) audio_spdif_blackout_enter(spdif_set, spdif_count);
    if (i2s_count)   audio_i2s_blackout_enter(i2s_set, i2s_count);
    restore_interrupts(save);
}
```

#### 5.4.3 `blackout_prime_consumer_pools()`

```c
static void blackout_prime_consumer_pools(void) {
    // Drop any stale buffers in each instance's consumer pool prepared
    // list.  These would be buffers the audio callback wrote during
    // the blackout (fade-gain-zero silence) plus any buffers that were
    // in the playing slot before blackout entry.  All of them are
    // either zero-gain or pre-blackout audio — neither is what we
    // want for the fade-in.  Drop them and let the audio callback
    // refill from a clean slate.
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            audio_i2s_instance_t *inst = i2s_instance_ptrs[i];
            if (inst) i2s_reset_consumer_pipeline(inst);  // existing helper
        } else {
            audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
            if (inst) spdif_reset_consumer_pipeline(inst);
        }
    }

    // Re-seed the preset_mute_counter so the audio callback continues
    // to apply fade-gain-zero output across the next few packets.
    // We need enough packets to fill the consumer pool to its prepared-
    // list depth (typically 2-3 buffers).  At 1 ms per packet, 4 packets
    // is enough margin.
    preset_mute_counter = 4 * 48;  // 4 ms at 48 kHz, sample-count form

    // Run the audio callback synchronously for the required number of
    // packets.  Each call drains one USB packet from the audio_ring
    // and processes it.  If the ring is empty (USB stream stopped),
    // we instead synthesise zero-input packets — see §5.4.4.
    blackout_prime_packets(4);
}
```

#### 5.4.4 `blackout_prime_packets()`

```c
// Synchronously drive the audio callback for N packets, then return.
// Used by blackout_exit to pre-fill the consumer pool with fade-in-
// gain-zero audio before handing the DMA back.
//
// If the USB audio_ring has packets queued, we drain them normally —
// the host has been streaming throughout the blackout and we have a
// backlog.  If the ring is empty, we synthesise zero-input packets
// of the standard 48-sample length so the consumer pool fills with
// silent buffers.
static void blackout_prime_packets(uint32_t packets) {
    extern void process_audio_packet(const audio_packet_t *pkt);
    extern void process_audio_packet_silent(uint32_t sample_count);
    /* the silent variant is a thin wrapper that calls process_input_block
       with zero buf_l/buf_r — see §5.5.1. */

    for (uint32_t i = 0; i < packets; i++) {
        audio_packet_t pkt;
        if (usb_audio_ring_peek(&pkt)) {
            process_audio_packet(&pkt);
            usb_audio_ring_pop();
        } else {
            process_audio_packet_silent(48);
        }
    }
}
```

### 5.5 State preservation discipline

#### 5.5.1 Filter states (master EQ, per-output EQ, loudness, crossfeed)

These IIR states accumulate per-sample. The blackout duration is
unbounded (typically 8–100 ms), so they will not advance during a
blackout that pauses the audio callback. On resume, the states hold
their pre-blackout value.

**Risk**: feeding the first post-blackout sample through a filter whose
state is from 100 ms ago creates a 1–2 sample transient (the filter
"jumps" to the new equilibrium). For a 10-tap biquad at 48 kHz with
Q≈0.7, the transient settles in ~5–10 samples (≈0.2 ms). The 8 ms
fade-in envelope completely masks this — the first samples are at gain
0 anyway.

**Mitigation**: none required. The fade-in envelope handles it.

**Verification**: listening test with EQ enabled, preset cycle 100×,
no audible click. Plus null-test recording vs. baseline.

#### 5.5.2 Volume leveller

The leveller has three pieces of state:
- RMS envelope (`env_sq_l`, `env_sq_r`)
- Smoothed gain (`gain_smooth_db`, `gain_linear`, `gain_prev_linear`)
- Lookahead ring buffer + write index

If the audio callback runs during blackout (non-flash case), the
leveller processes the actual USB input. The fade gain is applied
*after* the leveller, so the leveller sees real audio and its state
stays current. No artifact on resume.

If the audio callback does *not* run during blackout (flash case), the
leveller doesn't get called. State freezes at its pre-blackout value.
On resume, the next call processes real audio with that state as the
initial condition — same scenario as filter states. The fade-in masks
any transient.

**No leveller-specific code needed.** The agent's "trap" about RMS
decay applies only if we feed silence *through* the leveller during
blackout — which we don't.

#### 5.5.3 Delay lines and matrix mixer

Delay lines (`delay_lines[]`, `delay_write_idx`) are not advanced when
the audio callback doesn't run. On resume, the write index continues
from its pre-blackout value. The delay history is fully preserved.

For non-flash blackouts where the callback continues to run, the
delays do advance during the blackout — but the audio they store is
fade-gain-zero, so the delayed output is also fade-gain-zero. When the
fade-in begins, the most recent N samples in the delay line are
silence and the older samples are pre-blackout audio. As the fade-in
ramps up, the delayed output transitions from silence to a brief
period of pre-blackout audio (if the delay tap is longer than the
blackout duration). This is the correct, audible behaviour — exactly
what a real long-delay device would produce.

The matrix mixer has no per-sample state; coefficient changes are
applied at packet boundaries. A coefficient change during blackout
(from a preset load) takes effect at the first post-blackout packet,
which is fade-gain-zero. The transition is muted; the new coefficients
apply from the fade-in onward.

**No delay/mixer-specific code needed.** The data flow is naturally
correct.

#### 5.5.4 Master volume ramp endpoint

`vol_mul_master_prev` (audio_pipeline.c:97) holds the ramp's start
point for each packet. If the audio callback pauses during blackout
and resumes 100 ms later, `vol_mul_master_prev` is from 100 ms ago.
The first post-blackout packet's ramp goes from the old value to the
new (fade-gain-zero) value — but since `vol_mul_master_target *=
preset_mute_gain` and `preset_mute_gain == 0` at this point, the ramp
target is 0, and the ramp interpolates from old-value to 0 over the
packet. That's still a step in `vol_mul_master_target` between the
last pre-blackout packet and the first post-blackout packet.

**Mitigation**: at blackout exit, force `vol_mul_master_prev = 0` so
the fade-in starts from 0. Add a helper in audio_pipeline.h:

```c
void audio_pipeline_reset_volume_ramp(void);  // sets vol_mul_master_prev = 0
```

Call from `blackout_prime_consumer_pools()`.

#### 5.5.5 USB feedback controller

The feedback controller's state (rate estimator, fill error, SOF
counter) is updated only on USB SOF, in a different code path from the
audio callback. If we leave the USB SOF ISR running through blackout
(which is what happens on non-flash blackouts), feedback updates
continue and the controller stays current.

For flash blackouts, the SOF ISR is disabled (interrupts off). The
controller's state freezes. On resume, the first SOF after
re-enabling interrupts produces a delta that crosses the blackout
duration — potentially a 100 ms delta value. This propagates a large
sample-count error through `fb_ctrl_sof_update`, which clamps and IIR-
filters it. The clamp (`FB_OUTER_CLAMP_Q16` per main.c:633) bounds the
short-term feedback error.

The existing `reset_usb_feedback_loop()` (called from `complete_pipeline_
reset()` Phase 3) handles this for the existing teardown path. For
the new blackout path:

- **Non-flash blackouts**: do nothing; the SOF ISR keeps the controller
  current.
- **Flash blackouts**: call `reset_usb_feedback_loop()` from
  `blackout_exit()` after the DMA hot-swap back. This is the same
  function the existing path uses.

The reason ticket from `blackout_enter(reason)` lets us key this
behaviour:

```c
void blackout_exit(void) {
    /* ... */
    blackout_hot_swap_to_audio();

    if (blackout_reason_requires_feedback_reset(s_blackout_reason)) {
        reset_usb_feedback_loop();
    }

    blackout_trigger_fade_in();
    /* ... */
}

static bool blackout_reason_requires_feedback_reset(blackout_reason_t r) {
    switch (r) {
        case BLACKOUT_REASON_PRESET_SAVE:
        case BLACKOUT_REASON_PRESET_DELETE:
        case BLACKOUT_REASON_DIR_FLUSH:
        case BLACKOUT_REASON_FACTORY_RESET:
            return true;
        default:
            return false;
    }
}
```

#### 5.5.6 Core 1 EQ worker handshake

Two distinct cases:

**Non-flash blackout**: Core 1 keeps running. The audio callback
continues to dispatch EQ work (with fade-gain-zero output gain folded
into `vol_mul`). Core 1 processes the work normally. No special
handling needed.

**Flash blackout**: `flash_write_sector()` calls
`multicore_lockout_start_blocking()` which parks Core 1. Before this
call, `prepare_pipeline_reset()` already spin-waits for any in-flight
Core 1 work to complete (main.c:505–509). So Core 1 is idle when the
lockout starts. After the flash op, `multicore_lockout_end_blocking()`
releases Core 1; it returns to its `__wfe()` loop. The next audio
callback after blackout exit will dispatch the first fade-in packet,
which Core 1 picks up normally.

**No Core 1 code changes needed.**

#### 5.5.7 SPDIF input clock servo / resampler

If SPDIF input is active and the audio source, blackout operations
that don't disturb the SPDIF RX state (most of them) leave the servo
and resampler running. The clock servo is on a different code path
(`spdif_input_update_clock_servo`, called from main loop on a timer)
and is not blocked by `preset_loading`. The resampler (when enabled)
tracks the input rate independently.

For non-flash blackouts (preset load, source switch *to* SPDIF, bulk
params), the SPDIF RX state is preserved. The audio callback may pause
briefly while the operation runs; on resume, the resampler's ring has
buffered samples ready.

For flash blackouts, the SPDIF RX state is preserved across the
interrupt-disabled window because (a) the SPDIF PIO clock recovery is
hardware, not interrupt-driven, and (b) the resampler ring is in RAM
and not affected. The DSPi `main.c:830–832` already suspends the
SPDIF RX briefly across flash to avoid a decode-timeout race; that
behaviour is preserved.

**No SPDIF input code changes needed** beyond what's already there.

### 5.6 PDM coordination

Already covered in §5.1.3. The change is:

- Add `void pdm_ring_silence(void);` to `pdm_generator.h`, which
  zero-fills the PDM 256-bit ring buffer.
- Call from `blackout_hot_swap_to_silence()` *after* the audio DMA
  swap is complete and Core 1 has been parked (or is otherwise
  guaranteed not to write to the ring).

Implementation:

```c
// pdm_generator.c
void pdm_ring_silence(void) {
    // Zero the entire ring.  The DMA is in ring mode reading 32-bit
    // words from this buffer; once zeroed it will output a constant
    // zero stream (true silence at the PDM output pin).
    extern uint32_t pdm_ring[];  // existing static, expose via the .h
    memset(pdm_ring, 0, sizeof(uint32_t) * (1 << PDM_DMA_RING_BITS));
}
```

### 5.7 USB feedback continuity

Covered in §5.5.5. Brief: for non-flash blackouts no feedback action
is needed; for flash blackouts call `reset_usb_feedback_loop()` from
`blackout_exit()`.

---

## 6. Sync preservation strategy

This section restates the inter-slot alignment guarantees, since they
are the most load-bearing invariant in the CLAUDE.md constraints.

### 6.1 What's locked together by hardware

- All PIO SMs across all blocks share the same root clock (sys_clk).
- Each output type (SPDIF / I2S) uses an identical `clkdiv` across its
  instances.
- PIO TX FIFO drain is DREQ-paced from the SM.

Once any two SMs are *started in the same sys_clk cycle*, they remain
phase-locked forever — until one is stopped. This is the guarantee
that `pio_enable_sm_mask_in_sync()` provides.

### 6.2 What this means for blackout

The blackout never calls `pio_sm_set_enabled(false)`. The SMs are
running at blackout entry and are still running at blackout exit. They
remain phase-locked across the entire blackout, by construction.

The DMAs feed their respective PIO TX FIFOs. The hot-swap at entry and
exit is an abort+reconfigure+retrigger of the DMA, *not* the SM. The
PIO TX FIFO depth (4 words) buffers the brief ~1 µs swap window:

- SPDIF: 4 words × 4 bytes/word × 8 bits/byte ÷ 6.144 Mbits/s = ~21 µs slack
- I2S 32-bit: 4 words × 32 bits ÷ 3.072 Mbits/s = ~42 µs slack
  (numbers approximate, depend on actual sample rate)

The swap completes in <2 µs (a handful of register writes plus the
abort spin-wait). PIO never starves.

### 6.3 Cross-type alignment

When the configuration mixes SPDIF and I2S slots, the two type-specific
swap operations happen in sequence. They are wrapped in a single outer
`save_and_disable_interrupts()` block (§5.4.2), so the second swap
follows the first within a handful of sys_clk cycles. Both fit inside
either type's TX FIFO slack budget, so both types remain phase-locked
across the cross-type boundary.

This mirrors the existing `complete_pipeline_reset()` structure
(main.c:666), which wraps the SPDIF and I2S `enable_sync` calls in the
same outer block for the same reason. The comment at main.c:617–627
explicitly documents why the outer wrap is required, and that comment
applies verbatim to the new blackout path.

### 6.4 PDM alignment

PDM is on its own SM and its own DMA, in ring mode. It does not
participate in the multi-channel trigger. But its DMA never stops —
ring mode is endless. PDM phase relative to SPDIF/I2S is therefore
preserved by the same sys_clk-locked SM argument.

Pre-zeroing the PDM ring during blackout introduces a brief glitch
*at the PDM output pin* (Core 1's last ~5 ms of output stops being
looped; pure zero starts). This glitch is at the PDM modulator's
sigma-delta output, which feeds a hardware filter (R-C or LC) and
loudspeaker. Whether it's audible depends on the speaker's response
to a DC step. The DSP-side fade-out already attenuates the PDM input
to zero before the swap (Core 1 sees the fade-gain-zero output from
Core 0); the ring contains progressively quieter content as the
fade-out completes. By the time we zero the ring, the loop content
is already near-silent. The transition from "near-silent loop" to
"true zero" is small and below audibility on any reasonable speaker.

### 6.5 What we don't do

- We do NOT call `pio_sm_restart()` during blackout.
- We do NOT call `pio_sm_clear_fifos()` during blackout.
- We do NOT call `pio_sm_exec()` to redirect SM PC.
- We do NOT update `clkdiv` during blackout.
- We do NOT change PIO programs.

Any of those would risk inter-slot phase drift. The blackout is
strictly a DMA-layer operation.

---

## 7. Call-site migration plan

The eight operations listed in §3.3 are migrated. Each one keeps the
*operation* logic intact and only swaps the surrounding mute /
teardown calls. Four operations are unchanged.

### 7.1 Migration template

**Before** (current pattern):
```c
prepare_pipeline_reset(N);          // soft mute via DSP
... operation ...
complete_pipeline_reset();          // teardown + sync start
```

**After** (blackout pattern):
```c
blackout_enter(REASON);             // fade-out + DMA → silence
... operation ...
blackout_exit();                    // DMA → audio + fade-in
```

### 7.2 Per-call-site changes

| # | Operation | Current call site | After migration |
|---|---|---|---|
| 1 | Preset load (no type change) | main.c:1412, 1483 — `prepare_pipeline_reset(N)` ... `complete_pipeline_reset()` | `blackout_enter(BLACKOUT_REASON_PRESET_LOAD)` ... `blackout_exit()` |
| 2 | Preset save | flash_storage.c:387 (`flash_write_sector`) is called from preset_save which is wrapped at vendor_commands.c. Wrap there: `blackout_enter(BLACKOUT_REASON_PRESET_SAVE)` ... preset_save() ... `blackout_exit()` |
| 3 | Preset delete | Same pattern as #2, REASON=PRESET_DELETE |
| 4 | Directory flush | Often piggybacks on save/delete; if standalone, wrap the dir_flush call with REASON=DIR_FLUSH |
| 5 | Bulk param SET (no rate/type change) | main.c:1625, 1716 — wrap the apply call with REASON=BULK_PARAMS; if the bulk includes a rate change or type change, fall through to the existing path |
| 6 | Input source switch (same Fs) | main.c:1731 area — split: if Fs unchanged, use blackout; if Fs changes, use existing rate-change path with blackout wrapped around it |
| 7 | Factory reset (no type change) | main.c:1569 area — same as #5 |
| 8 | USB stream resync | main.c:1382 — wrap with REASON=STREAM_RESYNC |

For operations 5–7, the decision logic is: "does this operation require
PIO teardown?" If yes (rate change or type change pending), use the
existing `complete_pipeline_reset()` path. If no, use `blackout_*`.

We add a helper to make the decision explicit and well-tested:

```c
// In main.c or a new helper:
typedef enum {
    PIPELINE_PATH_BLACKOUT,    // RAM-only mutation; use blackout
    PIPELINE_PATH_FULL_RESET,  // PIO must be torn down (type or Fs change)
} pipeline_path_t;

static pipeline_path_t classify_pending_apply(const apply_context_t *ctx);
```

`apply_context_t` is a small struct carrying flags like
`type_change_pending`, `rate_change_pending`, etc. The classifier
inspects those and returns the correct path. Each migrated call site
goes through `classify_pending_apply` so the choice is centralised
and testable.

### 7.3 Operations that keep `complete_pipeline_reset()`

- Rate change (main.c:110, 162)
- Output-type retype (main.c:280, 452)
- Boot / USB init (usb_audio.c:1568)
- Composition of rate + type changes

These wrap themselves with `blackout_enter(...)` / `blackout_exit()`
**around** the `complete_pipeline_reset()` call, so the user-visible
fade-out / fade-in still happens — but the PIO teardown happens
*inside* the blackout window:

```c
blackout_enter(BLACKOUT_REASON_RATE_CHANGE);
... reset PIO clkdivs, etc. ...
complete_pipeline_reset();           // PIO teardown + sync restart
blackout_exit();
```

The blackout's fade gain keeps the audio at zero during the PIO
restart, so even the residual digital click from BMC/frame restart is
inaudible (it happens between two genuinely silent samples).

This is a net improvement on top of today's behaviour: rate changes
and type changes also gain the protection of the soft fade envelope
around the unavoidable hardware reset.

### 7.4 Call-site migration: ordering and back-compat

The migration is staged (§12) so that the existing path keeps working
throughout. The decision logic in `classify_pending_apply` defaults to
the existing path; the blackout path is opt-in per call site. We
migrate one site at a time, verify with audible testing, then move to
the next.

---

## 8. State preservation audit

This audit consolidates the §5.5 analysis plus the broader state map
from the investigation. Each item is categorised as one of:

- **FREEZE** — state must not advance during blackout
- **NATURAL** — state freezes automatically when the audio callback
  doesn't run; no special action needed
- **PRE-ROLL** — state must be advanced through silence before unmute
- **RESET** — state must be cleared on exit
- **PRESERVE** — state is fine across blackout (don't touch)

| Subsystem | State | Action | Notes |
|---|---|---|---|
| Master EQ biquad/SVF | per-band IIR state | NATURAL | Callback pause freezes state; fade-in masks 1–2 sample transient |
| Per-output EQ | per-band IIR state | NATURAL | Same |
| Loudness | SVF state per band per ch | NATURAL | Same |
| Crossfeed | LP + AP state | NATURAL | Same |
| Volume leveller | RMS, gain smooth, lookahead | NATURAL | See §5.5.2 |
| Matrix mixer | crosspoint coefficients | PRESERVE | No per-sample state |
| Delay lines | ring buffer contents | PRESERVE | Audio history |
| Delay lines | `delay_write_idx` | NATURAL | Doesn't advance if callback paused |
| Master volume ramp | `vol_mul_master_prev` | RESET (to 0) | At blackout exit; see §5.5.4 |
| Preset mute envelope | `preset_loading`, counter, smooth gain | Drive explicitly | The blackout coordinator owns this |
| USB feedback servo | rate, fill, SOF counter | RESET (flash ops only) | §5.5.5 |
| Host volume / mute | scalars | PRESERVE | Snapshotted each packet |
| Per-ch preamp | linear/mul/dB | PRESERVE | Constants |
| Core 1 handshake | work_ready, work_done | NATURAL | Spin-wait at entry guarantees idle; on exit, Core 0 dispatches fresh work |
| Core 1 delay_write_idx snapshot | in Core1EqWork | NATURAL | Re-captured per packet during fade-in |
| PDM ring buffer | 256-bit sigma-delta data | RESET (silence) | §5.1.3, §5.6 |
| Producer pools | head/tail | RESET (drained) | At blackout exit before audio swap |
| Consumer pools | prepared list | RESET (drained) | At blackout exit |
| SPDIF RX | clock recovery, ASRC | PRESERVE | Independent path |

The blackout coordinator's job is to enforce this matrix at the
boundaries of `blackout_enter()` and `blackout_exit()`. Items marked
NATURAL require nothing beyond not running the audio callback (or
running it with fade-gain-zero, which has the same per-state effect).
Items marked RESET / PRE-ROLL / explicit-drive are handled by the
coordinator code in §5.4.

---

## 9. Recovery procedure (Stage C in detail)

The most subtle stage. Listed step by step with rationale:

1. **Drop stale consumer-pool buffers.** Call `*_reset_consumer_pipeline()`
   on each instance. These buffers contain either pre-blackout audio
   (potentially several packets back, with old preset state) or
   fade-gain-zero silence. Both are wrong for a clean fade-in start;
   drop them.

2. **Reset master volume ramp endpoint.** Set `vol_mul_master_prev = 0`
   so the first fade-in packet ramps from 0 (not from a stale
   pre-blackout value).

3. **Re-seed the mute counter for fade-in.** The mute counter is still
   counting down from the entry's value, but we want to extend it so
   the audio callback continues producing fade-gain-zero packets for
   long enough to fill the consumer pool. Set it to a value that
   covers ~4 packet's worth.

4. **Prime the consumer pool.** Run `process_audio_packet()`
   synchronously for ~4 packets. Each call produces a fade-gain-zero
   buffer and pushes it to the consumer pool's prepared list. If USB
   has no packets queued, synthesise zero-input packets.

5. **Hot-swap the DMA back to producer-pool mode.** Call
   `audio_spdif_blackout_exit()` and `audio_i2s_blackout_exit()` under
   a single outer `save_and_disable_interrupts()` block (mirroring the
   entry side). DMAs synchronously start consuming the prepared fade-
   gain-zero buffers.

6. **For flash blackouts only**, call `reset_usb_feedback_loop()` to
   reseed the feedback controller from the post-blackout DMA word
   count.

7. **Trigger the fade-in envelope.** Set `preset_mute_counter = 0` so
   `update_preset_mute_envelope()` flips `preset_loading = false` on
   the next packet and starts ramping `preset_mute_smooth_gain` from
   0 toward 1 over `PRESET_MUTE_TRANSITION_MS` (8 ms).

8. **Wait for fade-in to complete.** Poll `audio_pipeline_get_mute_
   gain() >= 0.999f` with a 32 ms timeout. On timeout, log a warning
   and return — the firmware is in a known state, just audibly less
   smooth.

9. **Release DAC hardware mute.** `dac_hw_mute_release()` is
   idempotent and safe at any point; we call it last so the DAC's
   analog stage is muted across the entire digital fade-in transition.

10. **Mark blackout complete.** `s_blackout_active = false`.

---

## 10. Memory budget

| Item | RP2040 | RP2350 |
|---|---|---|
| SPDIF silence buffer (192-frame block) per instance | 1.5 KB × 2 = 3 KB | 1.5 KB × 4 = 6 KB |
| I2S silence buffer (48-frame) per instance | 384 B × up to 2 = 768 B | 384 B × up to 4 = 1.5 KB |
| `in_blackout` flag in each instance struct | ~1 B × ~4 | ~1 B × ~8 |
| Blackout coordinator state (blackout.c statics) | ~32 B | ~32 B |
| **Total BSS increase** | **~3.8 KB** | **~7.5 KB** |

Current BSS usage (from memory): RP2040 ~127 KB, RP2350 ~205 KB.
RP2040 SRAM is 264 KB; RP2350 SRAM is 520 KB. The increase is
comfortably within budget on both platforms.

Code size: estimated ~1.5 KB per platform (blackout.c plus the
library extensions in audio_spdif.c / audio_i2s_multi.c).

---

## 11. Testing and validation plan

### 11.1 Static / build tests

1. **Both platform builds clean.** No warnings. `cmake --build
   build-rp2040 --clean-first` and `cmake --build build-rp2350 --clean-first`
   succeed.
2. **BSS within budget.** `arm-none-eabi-size build-rp2040/DSPi/DSPi.elf`
   shows < 264 KB SRAM consumption.
3. **No new functions placed in flash by mistake.** All DSP-time-critical
   functions retain `__time_critical_func` or `__not_in_flash_func`
   attributes.

### 11.2 Unit-level tests (instrumented firmware)

Add temporary instrumentation (toggled via a vendor command) that:

- Counts DMA IRQs received during a blackout (must be zero).
- Counts PIO TX FIFO underruns during a blackout (must be zero).
- Records `inst->words_consumed` deltas across each phase, then
  computes inter-slot skew at blackout exit (must be zero samples).

Run with each blackout reason in turn, verify all three counters.

### 11.3 Audible tests

A real-world acceptance test list. Each is performed 50+ times
consecutively on both platforms with a high-end headphone amp / DAC
chain and a spectrum analyser on the SPDIF / line-level output.

| # | Test | Pass criterion |
|---|---|---|
| 1 | Preset load while playing 1 kHz sine | No audible click. Spectrum shows no transient. |
| 2 | Preset save while playing pink noise | No audible click. |
| 3 | Factory reset while playing music | No audible click. |
| 4 | Input source USB→SPDIF while SPDIF source is silent | No click; signal locks cleanly. |
| 5 | Input source SPDIF→USB | No click. |
| 6 | Bulk params set (whole DSP state via vendor command) | No click. |
| 7 | Stream resync (USB renegotiation) | No click. |
| 8 | Rapid preset cycling (10 changes / second for 60 s) | No accumulated artifacts. |
| 9 | Mixed output config (SPDIF slots + I2S slots) — preset load | No click on either output type. |
| 10 | Inter-slot sample alignment after 100 preset cycles | Captured outputs measure 0 sample skew. |

### 11.4 Edge cases

| Edge case | Expected behaviour |
|---|---|
| `blackout_enter()` called from inside an audio callback | Asserted out (not allowed). The coordinator is main-loop only. |
| Blackout while USB is unplugged | `blackout_prime_packets()` synthesises zero-input packets; no hang. |
| Blackout duration > consumer pool capacity × stale buffer lifetime | The silence self-chain runs indefinitely; no underrun. Validated by leaving blackout open for 10 s on a test build. |
| `blackout_exit()` called twice | Asserted out. |
| Power loss during blackout | Same as any other power loss; no special handling. |

### 11.5 Listening test in mixed configurations

Run a 30-minute play loop with random preset switches every 5 s and
random source switches every 30 s. Recording done with:

- TOSLINK SPDIF capture
- Line-level capture from I2S DAC
- Headphone capture from PDM PWM

Spectrogram of each capture should show no transients at the blackout
boundaries.

---

## 12. Phased implementation order

The work is divided into 8 self-contained phases. Each phase is a
small, reviewable, individually-testable change. We pause and verify
after each phase before moving to the next.

### Phase 0 — Specification and review
- This document.
- Review by the human user; iterate until approved.

### Phase 1 — Silence buffer construction
- Build the 192-frame SPDIF silence buffer at `audio_spdif_setup()`
  time, replacing today's smaller `silence_buffer`. Verify the
  existing IRQ-fallback path (`audio_start_dma_transfer` with empty
  pool) still works and now plays the correct ±1 LSB pattern.
- Same for I2S (48-frame is fine for I2S; pattern is `±1` shifted
  into the high 24 bits, OR'd with `I2S_PAD_PATTERN`).
- **Test**: simulate consumer-pool underrun (vendor command that
  pauses the audio callback for 100 ms). Verify the output is silent
  with no pop, and that SPDIF receivers stay locked.

### Phase 2 — DMA self-chain primitive (library)
- Add `audio_spdif_blackout_enter` / `audio_spdif_blackout_exit` to
  `pico_audio_spdif_multi`.
- Add `audio_i2s_blackout_enter` / `audio_i2s_blackout_exit` to
  `pico_audio_i2s_multi`.
- Add `in_blackout` field to each instance struct; defensive check in
  IRQ handlers.
- **Test**: vendor command that enters blackout for 100 ms then exits.
  Verify with the instrumentation in §11.2 that DMA never underruns,
  PIO never stalls, and inter-slot skew remains zero.

### Phase 3 — Blackout coordinator skeleton
- Create `firmware/DSPi/blackout.c` and `blackout.h` with
  `blackout_init`, `blackout_is_active`, `blackout_enter`,
  `blackout_exit`. First version uses Reason=BLACKOUT_REASON_PRESET_LOAD
  for all calls.
- Add `audio_pipeline_get_mute_gain()` and
  `audio_pipeline_reset_volume_ramp()` to audio_pipeline.h.
- **Test**: vendor command that triggers a 50 ms blackout. Confirm
  audible fade-out, silence, fade-in.

### Phase 4 — Migrate preset load
- Replace `prepare_pipeline_reset` / `complete_pipeline_reset` at
  main.c:1412, 1483 with `blackout_enter(BLACKOUT_REASON_PRESET_LOAD)`
  / `blackout_exit()`.
- Branch when the preset includes a type or rate change: fall back to
  the existing path inside the blackout (the blackout wraps the
  full reset).
- **Test**: 100 preset cycles while playing. Listen for clicks.

### Phase 5 — Migrate preset save / delete / directory flush
- Wrap the flash-write call sites in `vendor_commands.c` with
  `blackout_enter(BLACKOUT_REASON_PRESET_SAVE)` etc. Move the existing
  hold time into the blackout coordinator's tuning table.
- Carefully preserve the multicore lockout + interrupt disable inside
  the blackout window — these are still needed for the actual flash
  write, but now operate while DMA is self-sustaining.
- **Test**: 100 preset saves while playing. Listen for clicks. Verify
  flash write completes correctly (post-save load round-trip).

### Phase 6 — Migrate input source switch (same Fs)
- Split the source-switch logic at main.c:1731 into two branches:
  same-Fs and cross-Fs. Same-Fs uses blackout; cross-Fs uses the
  existing rate-change path wrapped in blackout.
- **Test**: USB↔SPDIF switching at 48 kHz with a steady source playing
  on each side.

### Phase 7 — Migrate remaining call sites
- Bulk params (main.c:1625, 1716): wrap with classify_pending_apply
  to choose blackout vs full reset.
- Factory reset (main.c:1569): same pattern.
- Stream resync (main.c:1382): wrap with BLACKOUT_REASON_STREAM_RESYNC.

### Phase 8 — Wrap retype and rate change in blackout
- `process_type_switches()` and `perform_rate_change()` already use
  `complete_pipeline_reset()`. Wrap their entire bodies in
  `blackout_enter()` / `blackout_exit()` so the unavoidable PIO
  restart happens inside a silent window.
- **Test**: rate change USB 44.1→48 while playing. Type switch
  SPDIF→I2S on a single slot while other slots keep playing.

### Phase 9 — Cleanup and finalise
- Remove any temporary instrumentation.
- Update `Documentation/current_architecture.md` to add a "Blackout
  coordinator" section and update the "Memory Layout" section with
  the new BSS figures.
- Update `MEMORY.md` index with a one-line pointer to this spec.

Each phase ends with a build on both platforms, BSS check, and the
relevant subset of the listening tests in §11.

---

## 13. Open questions and future work

### 13.1 Hot-pivot output type retype

This spec keeps output-type retype on the existing path. A future
enhancement could pre-load both SPDIF and I2S PIO programs in the same
block (they fit: 4 + 8 = 12 instructions vs. 32-slot capacity) and
hot-pivot a single SM's program counter via `pio_sm_exec(jmp ...)` plus
`clkdiv` update plus DMA reconfig. This would eliminate the discontinuity
on type-switching individual slots.

The plumbing (program co-residence, sync IRQ rendezvous, etc.) was
discussed in earlier conversation but is non-trivial. Defer until the
core blackout work is proven.

### 13.2 PDM analog softening

PDM's transition from "near-silent loop" to "true zero" during
blackout is below audibility on most speakers but not provably zero.
A future enhancement could ramp the PDM ring content via Core 1 over
~1 ms (rather than a memset) so the transition is smooth at the
modulator output. Low priority.

### 13.3 Reducing blackout duration

Today's tunings are conservative — 8 ms fade-out, 4-packet prime,
8 ms fade-in, plus the operation itself. For routine preset loads
this means ~25 ms of audible silence. Some users may want sub-10 ms
transitions. The fade durations are tunable via the reason table;
the prime depth could be reduced to 1–2 packets if validated. Not
needed for v1.

### 13.4 Asynchronous blackout

Today's API is synchronous (`blackout_enter` blocks for the fade-out;
`blackout_exit` blocks for the fade-in). A future revision could
provide `blackout_enter_async()` / `blackout_check_ready()` for
callers that can do useful work during the fade. Not needed today.

### 13.5 Dither pattern selection

The proposed ±1 LSB Nyquist square is one of several valid silence
patterns. Alternatives: triangular dither at LSB amplitude (better
spectral profile but more complex to construct); fixed `0x000001`
(simpler but DC-biased). The Nyquist square is chosen because (a) it
guarantees every subframe is non-zero, (b) it has zero DC content,
(c) it's perfectly above audibility. Re-evaluate if any downstream
chip is sensitive to the pattern.

---

## 14. Appendix A — detailed call-site inventory

(Reproduced from the investigation pass for traceability. file:line
references are exact.)

### Operations migrating to blackout (RAM-only or flash-write)

- **Preset load (no type change)** — `process_pending_preset_load`
  area, main.c:1412, 1483.
- **Preset save / delete / dir flush** — `flash_write_sector` in
  flash_storage.c:387–439; called from `preset_save`, `preset_delete`,
  `dir_flush` (each ~45 ms with IRQs disabled).
- **Input source switch (same Fs)** — `perform_input_source_switch` at
  main.c:1731, 1787. Today this unconditionally goes through
  `perform_rate_change()` even when Fs doesn't change; we split.
- **USB stream resync** — `process_stream_resync` at main.c:1382.
- **Bulk params SET** — main.c:1625 (set), 1716 (apply).
- **Factory reset** — main.c:1569 area.

### Operations remaining on `complete_pipeline_reset()` path

- **Output-type retype** — `process_type_switches` at main.c:280, 452.
- **Rate change** — `perform_rate_change` at main.c:110, 162.

Both of these will be *wrapped* in blackout entries/exits (Phase 8) so
the PIO teardown happens inside the silent window, but the teardown
itself is unchanged.

### Code paths called from inside blackout but unchanged

- `flash_range_erase` / `flash_range_program` (in flash_storage.c).
- `multicore_lockout_start_blocking` / `multicore_lockout_end_blocking`.
- `prepare_pipeline_reset()` (called from inside `blackout_enter`).
- `reset_usb_feedback_loop()` (called from `blackout_exit` for flash
  reasons only).

---

## 15. Appendix B — library API surface changes

### `pico_audio_spdif_multi`

**New public API** (`audio_spdif.h`):
```c
void audio_spdif_blackout_enter(audio_spdif_instance_t *instances[], uint count);
void audio_spdif_blackout_exit(audio_spdif_instance_t *instances[], uint count);
```

**New instance field** (`audio_spdif.h`):
```c
typedef struct audio_spdif_instance {
    /* ...existing... */
    bool in_blackout;  // diagnostic + IRQ-handler defensive check
} audio_spdif_instance_t;
```

**Existing API unchanged**: `audio_spdif_setup`, `audio_spdif_connect_*`,
`audio_spdif_set_enabled`, `audio_spdif_change_pin`,
`audio_spdif_enable_sync`, starvation API.

**Internal changes**:
- `audio_spdif_setup()` allocates the 192-frame silence buffer and
  initialises it with the dither pattern.
- The DMA IRQ handler at audio_spdif.c:412 gains an
  `if (inst->in_blackout) continue;` guard.
- `stamp_spdif_buffer()` factored out of `audio_start_dma_transfer` for
  reuse in `audio_spdif_blackout_exit`.

### `pico_audio_i2s_multi`

Same shape as SPDIF: new `audio_i2s_blackout_enter` / `_exit`, new
`in_blackout` field, internal silence buffer build, IRQ-handler
guard. No I2S-specific subtleties.

### `pdm_generator`

**New public API** (`pdm_generator.h`):
```c
void pdm_ring_silence(void);  // zero the 256-bit ring
```

### `audio_pipeline`

**New public API** (`audio_pipeline.h`):
```c
float audio_pipeline_get_mute_gain(void);   // exposes preset_mute_smooth_gain
void  audio_pipeline_reset_volume_ramp(void); // sets vol_mul_master_prev = 0
```

Both are simple getters/setters and require no other code changes.

### `blackout` (new module)

`firmware/DSPi/blackout.h`:
```c
typedef enum { /* BLACKOUT_REASON_* */ } blackout_reason_t;
void blackout_init(void);
void blackout_enter(blackout_reason_t reason);
void blackout_exit(void);
bool blackout_is_active(void);
```

`firmware/DSPi/blackout.c` — implementation (~400 lines including
extensive comments).

---

## 16. Documentation updates

After implementation completes (per CLAUDE.md):

- Update `Documentation/current_architecture.md`:
  - Add new "Glitch-free blackout" section under audio outputs.
  - Update "Memory Layout" section with new BSS figures.
  - Update "Vendor Command Reference" (none new in this work, but
    confirm).
- Add one-line index entry in `MEMORY.md` pointing to this spec.

No other documentation needs to change for this feature.

---

*End of specification.*
