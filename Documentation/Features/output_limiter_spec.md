# Output Limiter Specification

## 1. Overview

The output limiter is a brickwall peak limiter on every output channel. Each output has its own on/off switch, threshold and release time, and outputs can be linked into groups that share one gain reduction. It exists to protect whatever sits after the device (amplifiers, drivers, a downstream converter) and to stop hard clipping at the output converter.

The limiter looks ahead. It delays the audio by a fixed 32 samples so the gain has already come down by the time a peak reaches the output. With the lookahead, no sample leaves a limited output above its threshold, and the gain never jumps. It always ramps.

Both platforms are supported. RP2350 runs the limiter in float. RP2040 runs it in fixed point with one hardware divide per block.

### Key characteristics

- **Per output, all settings independent.** Every output has its own enable, threshold, release and link group.
- **No overshoot.** A sample on a limited output never exceeds its threshold, including right after its limiter is switched on or off (section 7).
- **Linked groups.** Outputs with the same non-zero link group all apply the deepest gain reduction any member needs. A stereo image cannot shift, and a pair of woofers stays matched.
- **Zero cost when unused.** While no output has its limiter enabled, the lookahead delay is removed from the signal path entirely. No CPU is spent and no latency is added.
- **Alignment preserved.** While any limiter is enabled, every output carries the same 32-sample delay, including outputs whose own limiter is off. Inter-slot sample alignment never changes. Switching the delay in or out happens only under a short fade to silence (section 2.3).

### Signal chain position

```
matrix -> crossfeed -> subharm -> psybass -> tube -> crossover -> PEQ
       -> output gain (matrix gain x volume x master) -> loudness
       -> LIMITER -> output delay -> metering -> output encoding
```

The limiter runs after all gain stages, so its threshold is the absolute level leaving the device in dBFS. It runs before the per-output delay line, which leaves the user's time-alignment delays unchanged.

## 2. Algorithm

### 2.1 Block-rate gain with lookahead

The limiter decides its gain once per 16-sample block (B = 16) and delays the audio by two blocks (D = 32 samples). Block boundaries sit on one stream-wide sample counter shared by every output, so all outputs decide on the same samples.

For each output, when input block `j` completes:

```
p_j = max |x| over block j
h_j = min(1, T / p_j)                       hard target: the gain that holds block j at T
E_j = min(E_(j-1) * r, 1, h_(j-1), h_j)     envelope: release, then attack to both blocks
```

The output emitted next is block `j-1`, because the delay is two blocks. Its gain ramps linearly from `G_(j-1)` to `G_j`, where `G` is `E` for an unlinked output or the group minimum of `E` for a linked one (section 2.2).

**Why there is no overshoot.** Block `k` is emitted with a gain that ramps from `G_k` to `G_(k+1)`. `G_k` contains `h_k` as its "newest block" term, and `G_(k+1)` contains `h_k` as its "previous block" term. Both ends are at or below `T / p_k`, so every point on the straight line between them is too, and every sample in block `k` stays at or below `T`.

**Why the delay is two blocks.** The ramp for block `k` ends at `G_(k+1)`, which needs the peak of block `k+1`. That is known only once block `k+1` has fully arrived, so the output must lag the input by two blocks.

**Attack** is therefore one block (16 samples: 0.33 ms at 48 kHz, 0.17 ms at 96 kHz), and it always completes before the peak arrives. Attack is fixed. Any other attack would need a different delay, and every output must share one delay.

**Release** multiplies the envelope by a constant `r` per block, so the gain recovers at a constant rate in dB. `r = exp(B / (tau * fs))`, where `tau` is the release time. The gain recovers 8.69 dB (a factor of e) per release time, which is the classic peak-envelope-follower definition.

### 2.2 Link groups

Each output's `E` is computed from its own threshold, its own release and its own signal. For a linked output, the gain actually applied at each boundary is the minimum `E` over all participating members of its group. Every member stays at or below its own threshold, because the minimum is never above its own `E`. Members with different release times need no special rule.

The dual-core pipeline splits the outputs between cores. On RP2350, Core 0 owns outputs 0-1 and Core 1 owns outputs 2-7. On RP2040, Core 0 owns 0-1 and Core 1 owns 2-3. When an active group has members on both cores, the cores meet once per packet. Each core measures its own outputs, the cores wait for each other, and then each applies the group gains to its own outputs. The meeting is skipped for any packet where no group spans the cores, and in single-core mode.

### 2.3 Engaging and releasing the lookahead delay

The 32-sample delay is present only while at least one output has its limiter enabled. When the first limiter is switched on, or the last one is switched off, the delay has to appear or disappear on every output at once. Adding or dropping 32 samples mid-stream would click, so the change is made in silence.

1. The main loop sees that the published configuration wants the delay in a different state. It holds the pipeline's soft mute (the same 8 ms fade used by preset and flash operations) with a 40 ms refresh. If the last 32 samples were already silent it skips the fade, because the switch will happen on the next packet anyway.
2. The pipeline counts consecutive samples whose output gain is exactly zero. Once 32 or more have passed, the delay rings hold only zeros.
3. At the next packet boundary the pipeline switches the delay in or out on every output together, with the rings cleared. The switch is inaudible because both sides of it are silent.
4. The main loop stops holding the mute, and audio fades back up.

When nothing is streaming, the main loop switches the delay directly, because there is nothing on the wire to disturb. The same applies when a producer counts as streaming but no packet has been processed for 50 ms of main-loop time, for example a USB host that stopped sending without changing the alt setting. A user who is already muted gets the switch with no extra fade.

The whole operation affects every output identically in one packet, so inter-slot alignment holds before, during and after.

## 3. Parameters

All parameters are per output. Output indices follow the output channel numbering used everywhere else (RP2350: 0-7 S/PDIF or I2S, 8 PDM; RP2040: 0-3, 4 PDM).

| Index | Name | Range | Default | Notes |
|-------|------|-------|---------|-------|
| 0 | `enabled` | 0 / 1 | 0 | Any non-zero value enables |
| 1 | `threshold_db` | -30.0 .. 0.0 dBFS | -1.0 | Ceiling. 1 dB of margin covers inter-sample overshoot in a DAC |
| 2 | `release_ms` | 10 .. 1000 ms | 100 | Gain recovers 8.69 dB per release time |
| 3 | `link_group` | 0 .. 4 | 0 | 0 = unlinked. Rounded to the nearest integer |

Out-of-range values are clamped. NaN is ignored and never stored.

**Full scale.** 0 dBFS is digital full scale of the S/PDIF, I2S and ADAT outputs on both platforms. On RP2350 that is float 1.0. On RP2040 it is Q28 raw 2^29, which is the value that maps to 24-bit full scale through the Q28 `>> 6` output conversion. The PDM output keeps the pipeline's existing PDM scaling, which is not referenced to the same full scale on the two platforms.

## 4. Vendor Command

One opcode carries every limiter operation. The transfer direction selects SET or GET, so the limiter costs a single top-level command.

| Name | Code | Dir | Description |
|------|------|-----|-------------|
| `REQ_LIMITER` | 0x81 | OUT | Set one parameter. `wValue = (output << 8) \| index`, payload = float32 LE. `output = 0xFF` sets that parameter on every output |
| `REQ_LIMITER` | 0x81 | IN | Get one parameter as float32 LE, or a read-only block (below) |

GET indices:

| Index | Returns |
|-------|---------|
| 0-3 | The parameter for `output`, as float32 LE (4 bytes) |
| 0x80 | Gain-reduction meter, all outputs: `NUM_OUTPUT_CHANNELS` x uint16 LE, gain reduction in 0.01 dB (0 = none). The output byte is ignored |
| 0x81 | Status, 4 bytes: `engaged` (1 = the lookahead delay is in the signal path), `lookahead_samples` (32), `block_samples` (16), `num_outputs` |

A bad output or index STALLs a GET. A bad output or index on a SET is a silent no-op, matching the other indexed parameter commands. A host can feature-detect the limiter by reading index 0x81.

**Meter semantics.** Each output reports the deepest gain applied during the most recent audio packet. The limiter's own release smooths it, so a poll at 10-20 Hz sees any limiting event whose release is 50 ms or longer. Outputs that are not limiting report 0.

**Latency.** When `engaged` is 1, every output is 32 samples later than when it is 0. A host doing time-aligned measurements should read the status block.

### Apply semantics

A SET writes the live configuration and raises a main-loop recompute flag. The main loop recomputes the coefficients and publishes them. The pipeline picks them up at the next packet boundary. Changes to threshold, release and link group take effect within one packet and do not interrupt audio. Enabling the first limiter or disabling the last one triggers the silent delay switch in section 2.3.

### Change notifications

Every SET emits `notify_param_write` at the parameter's offset in `WireBulkParams.limiter`, one notification per output changed (nine for an `output = 0xFF` SET on RP2350).

## 5. Bulk Wire Format and Persistence

**Wire format V32** appends a 108-byte `limiter` section to `WireBulkParams` at offset 6028 (total 6136 bytes).

```c
typedef struct __attribute__((packed)) {
    uint8_t  enabled;        // 0/1
    uint8_t  link_group;     // 0..4
    uint8_t  reserved[2];    // zero
    float    threshold_db;   // -30..0
    float    release_ms;     // 10..1000
} WireLimiterOutput;         // 12 bytes

typedef struct __attribute__((packed)) {
    WireLimiterOutput outputs[WIRE_MAX_OUTPUT_CHANNELS];   // 9 x 12
} WireLimiterParams;         // 108 bytes
```

Entries past `num_output_channels` are zero on GET and ignored on SET.

**Preset slot V39** appends the same per-output record, one per `NUM_OUTPUT_CHANNELS`. Older slots (V21..V38) load with every limiter disabled at the defaults. Factory reset restores the defaults.

## 6. CPU and Memory

The per-sample work is small and fixed.

| | RP2350 (float) | RP2040 (Q28) |
|---|---|---|
| Measure pass, per sample | `|x|` and a running max | integer abs and compare |
| Apply pass, per sample | ring read and write, one multiply, one add | ring read and write, two 16-bit multiplies |
| Per 16-sample block | one divide, three min/max, one multiply | one hardware divide, one 64-bit multiply |
| Delay-only output (limiter off, delay engaged) | ring read and write, no arithmetic | ring read and write, no arithmetic |

Estimated cost at 48 kHz and 307.2 MHz is about 0.1 % of a core per limited output on RP2350 and 0.4 to 0.5 % on RP2040. This is an estimate from operation counts. It has not been measured on the CPU meter yet.

Static RAM: RP2350 about 2.1 KB of BSS (nine 128-byte delay rings, per-output state, the per-packet envelope table) and 2.2 KB of RAM-resident code. RP2040 about 1.2 KB of BSS and 2.4 KB of code.

## 7. Interactions and Edge Cases

- **Switching one output's limiter on while audio plays.** The two blocks already in its ring were never measured. On joining, the limiter scans the ring once and holds the output at the gain the ring's peak needs until block decisions take over, so nothing overshoots. The gain can step down once at that moment.
- **Switching one output's limiter off while it is reducing gain.** The output drains. Its ring was measured, so it keeps its own envelope with no threshold and releases to unity at its release rate. No sample overshoots and the gain never steps. Once it reaches exact unity it becomes a plain delay.
- **Muted and matrix-disabled outputs.** A disabled output is not processed. Its ring is cleared and its limiter state reset, so it re-enters cleanly. A muted output keeps running through the delay with silence.
- **Signal generator RAW outputs.** A RAW test signal is limited like any other signal on an output whose limiter is on. RAW skips the crossover, so a full-range sweep can reach a tweeter, which is when protection matters most. Signals below the threshold pass bit-exact. For an unaltered full-scale measurement, switch that output's limiter off first.
- **Preset load, factory reset.** Delay rings and limiter state are cleared with the other delay lines. A preset whose limiter configuration changes the engaged state goes through the section 2.3 switch.
- **Sample-rate change.** Release coefficients are recomputed. The lookahead stays 32 samples, so the attack time in milliseconds scales with the rate.
- **Fade-to-silence accounting.** While the delay is engaged, `pipeline_max_active_delay_samples()` includes the 32 lookahead samples, so flash and reset brackets wait for the fade to drain through the limiter as well as the delay lines.
- **Sample peaks, not true peaks.** The limiter holds sample values at or below the threshold. Reconstruction in a DAC can overshoot between samples by up to about 1 dB on worst-case material, which is why the default threshold is -1 dBFS.
- **Not thermal protection.** This is a peak limiter. It does not model voice-coil heating. A slow RMS power limiter would be a separate mode.

## 8. Implementation Summary

- `limiter.h` / `limiter.c`: configuration, indexed parameter access, coefficient publish, engage state machine, and the measure/link/apply kernel shared by both cores.
- `audio_pipeline.c`: `limiter_packet_begin()` on Core 0 before the Core 1 dispatch, `limiter_process_outputs()` for each core's outputs after loudness and before the delay lines, and `limiter_packet_end()` after both cores finish.
- `pdm_generator.c`: `limiter_process_outputs()` for Core 1's outputs in the EQ worker.
- `main.c`: coefficient recompute on the pending flag and the engage service (soft-mute hold, or a direct switch when nothing is streaming).
- `vendor_commands.c`: `REQ_LIMITER` in both the SET and GET dispatchers.
- `bulk_params.c`, `flash_storage.c`: wire V32 section and preset slot V39.
