# S/PDIF Output: Biphase-Mark Encoding in the PIO

*Status: implemented 2026-10-11 as a hybrid (PIO encoder by default, software encoder only when the I2S external-clock program needs the block). Waveform equivalence proven in simulation (`tools/spdif_pio_bmc_sim.py`) and the assembled program checked against the simulated one. Not yet verified on hardware.*

## Summary

Before this change each enabled S/PDIF output slot cost about 1.3% of core 0 at 307.2 MHz, almost all of it the software biphase-mark encoder in `audio_spdif_ring_write_s32()`: three table lookups plus parity per subframe, a modulo-192 phase per frame, and a block of silence restamped behind every block. The PIO now does that encoding. A 16-instruction program (`audio_spdif_bmc.pio`) fed with raw 24-bit samples produces a waveform identical, cycle for cycle, to the original 4-instruction NRZI program fed with software-encoded words: the same 256 PIO cycles per frame, the same four ring words per frame, the same DMA and consumed-word accounting, and the same two-cycle start lead-in. The CPU side is four stores and two masks per frame.

The constraint that made this a hybrid is PIO instruction memory. The slot block (PIO0) holds the S/PDIF program plus either the I2S master-clock programs (8 + 8) or the checked I2S external-clock program (26). The 16-instruction encoder fits beside the master-clock programs with nothing spare, and does not fit beside the external-clock program. So the firmware selects the encoder per configuration.

## Selection rule

The software encoder is used if and only if `i2s_slave_mode_active()` is true **and** at least one output slot is typed I2S. In every other configuration, including an external I2S master driving the input while every output slot is S/PDIF, the PIO encoder is loaded. The rule lives in `process_type_switches()` (main.c), which already computes `want_extclk` for the I2S slots.

| Configuration | PIO0 residents | S/PDIF encoder |
|---|---|---|
| No I2S output slots (any clock mode) | S/PDIF 16 | PIO |
| I2S slots, internal clocking | S/PDIF 16 + clkout 8 + dataout 8 = 32 | PIO |
| I2S slots, external master (clock slave) | S/PDIF 4 + extclk 26 = 30 | software |

### How the selection is applied

- The library keeps one requested encoder and one loaded encoder per PIO block (`audio_spdif_select_encoder()`, `audio_spdif_loaded_encoder()`). The requested value is applied the next time the block loads a program, which is the first `audio_spdif_setup()` after the block's last `audio_spdif_teardown()`. The last teardown on a block now releases the program; the I2S library does the same for its three programs when its last instance on the block goes, so the 16-slot encoder always has room when it is wanted.
- Each instance records the format of the block's loaded program in `pio_bmc` at setup, before its ring reset stamps silence. All S/PDIF slots on a block share one program and therefore one word format.
- `process_type_switches()` computes the wanted encoder from the target configuration before any teardown or setup, and treats a mismatch with the loaded encoder as a transition (`spdif_encoder_flip`). On a flip every S/PDIF slot that is staying S/PDIF is torn down ahead of Pass 1 and re-set-up in Pass 2 alongside the retyped slots, so the program swap and the ring restamp happen inside the existing stopped-and-restarted output rebuild. Boot sets the slots up with the default (PIO) encoder and the boot-time `apply_restored_outputs()` corrects it through the same path.
- If a program ever fails to fit, both libraries panic with a message naming the program and block rather than storing a negative offset.
- Instruction memory cannot fragment. Every encoder flip tears down all S/PDIF slots and (through the clocking mismatch) all I2S slots, so the block is empty before the new set loads, and within internal-clock mode the three programs are 16, 8 and 8 slots, which pack into 32 in any load order under the SDK's top-down placement.

## Why it is alignment-safe

Slot alignment depends on the frame clock, the start sequence and the ring accounting, none of which change.

- **Frame timing is unchanged.** Every subframe is exactly 128 PIO cycles (32 time slots, two half-cells each, two cycles per half-cell), so the 256 Fs PIO clock and `audio_base_divider_16_8()` are untouched.
- **Ring geometry is unchanged.** Still 16 bytes per frame (two 32-bit words per subframe), so `RING_WORDS`, `consumed_word_rem`, the DMA ring wrap, `ring_dma_arm()`, `audio_spdif_ring_consumed_words()`, the FIFO prefill accounting and `audio_spdif_ring_skip()` keep their arithmetic. Only the meaning of the two words changes.
- **Start is unchanged.** The synchronized start execs a jump to the program's entry instruction and enables all SMs in one mask write. Both entry points emit two low cycles before the first preamble bit, so S/PDIF slots keep their phase relative to I2S slots and PDM whichever program is loaded.
- **Underrun behaviour is unchanged.** A `pull` on an empty FIFO stalls with the pin held, exactly as the autopull `out` does. The ring's silence margin makes that unreachable in normal operation either way.
- **Parity cannot drift.** The PIO derives the parity bit from the line state, so every subframe ends at the same level and the preamble polarity is fixed by construction.
- **Encoder swaps only happen inside a full output rebuild**, with every slot stopped, the rings reset and all slots restarted from one synchronized start.

## Wire format in the ring (PIO encoder)

Each subframe is two words, pulled in order.

| Word | Bits | Content |
|---|---|---|
| A (control) | [2:0] | lead-low count: 4 for X, 2 for Y, 0 for Z |
| A | [5:3] | trail-low count: 1 for X, 3 for Y, 5 for Z |
| B (data) | [23:0] | 24-bit sample, LSB first on the wire (time slots 4 to 27) |
| B | [24], [25] | V and U, both 0 |
| B | [26] | channel-status bit for this frame's phase |
| B | [31:27] | ignored (threshold 27, no autopull) |

Control word constants (`BMC_CTRL_X/Y/Z` in audio_spdif.c): X = 0x0C, Y = 0x1A, Z = 0x28. The preamble is generated as two low runs around a fixed high run, so the three preambles differ only in those two counts.

CPU work per frame is two constant stores, two `(sample & 0xFFFFFF) | (c << 26)` stores, and a running IEC phase. Silence frames are the same stores with the sample at zero. The software path (`stamp_frame()`, `stamp_silence_frame()`, the lookup table) is retained unchanged for the external-clock configuration; `stamp_audio_run()` and `stamp_silence_run()` select the format once per run from `pio_bmc`. Both paths now carry a running phase counter instead of a per-frame modulo.

## The program

Shift right, no autopull, pull threshold 27, one side-set pin on the output GPIO. Each data bit is four cycles: a forced transition then a conditional one, with the level tracked by which copy of the loop is executing. After 27 data bits (sample, V, U, C) the `jmp !osre` falls through, and which copy it falls out of says what the parity bit must be.

```
.program audio_spdif_bmc
.side_set 1
.wrap_target
L_bit:    out x, 1          side 1          ; bit half 1, line high
          jmp !x H_high2    side 1          ; bit 0: half 2 stays high
L_low2:   jmp !osre L_bit   side 0 [1]      ; bit 1: half 2 low. 27 bits done at low: P=1
          nop               side 1 [1]      ;   P half 1
          pull              side 0 [1]      ;   P half 2, OSR = word A      <- sync-start entry
common:   nop               side 1 [5]      ; preamble: high x6
          out y, 3          side 0          ; lead low count
lead:     jmp y-- lead      side 0          ; low x (y+1), total lead = y+2
          out y, 3          side 1          ; high x2, trail count
          pull              side 1          ;   OSR = word B
trail:    jmp y-- trail     side 0          ; low x (y+1), then wrap to L_bit
.wrap
H_bit:    out x, 1          side 0          ; bit half 1, line low
          jmp !x L_low2     side 0          ; bit 0: half 2 stays low
H_high2:  jmp !osre H_bit   side 1 [1]      ; bit 1: half 2 high. 27 bits done at high: P=0
          pull              side 0 [2]      ;   P both halves low (3 of 4 cycles), OSR = word A
          jmp common        side 0          ;   4th cycle
```

Cycle budget per subframe. The preamble is 6 + lead + 2 + trail = 16 cycles with lead + trail = 8, the 27 data bits take 108, and the parity bit takes 4, for 128 in total.

`spdif_bmc_program_init()` configures `sm_config_set_out_shift(&c, true, false, 27)` and jumps to `entry_point` (offset + 4); `audio_spdif_enable_sync_prepare()` jumps to the same entry through `spdif_entry_offset()`.

## Verification

**Simulation.** `tools/spdif_pio_bmc_sim.py` models the PIO subset both programs use (OUT, JMP including `!osre` and `y--`, PULL, side-set, delays, wrap, OSR shift count against the pull threshold, autopull versus manual pull). It feeds the original program with words from a port of `stamp_frame()` and the new program with the raw words above, and compares the pin level on every cycle. Over 1000 frames on five random seeds, including zero, full-scale, negative and sign-extended samples and five wraps of the 192-frame channel-status cycle, the two waveforms are identical, and the first eight cycles after enable match. The pioasm output in the build tree was decoded back into the simulator's instruction model and matches it instruction for instruction, with the same wrap points and entry offset.

**Build.** Both platforms build clean; `scripts/check_ram_placement.py` passes on both images (the stampers are forced inline into the RAM-resident writer, skip and pump).

**Bench items still open.**

1. `JMP !OSRE` evaluates against the pull threshold with autopull disabled, so the data loop exits after exactly 27 bits. The datasheet defines OSRE this way; a scope check that every subframe is 128 cycles settles it.
2. Receiver lock on all slots at 44.1, 48 and 96 kHz, including the fractional-divider rates.
3. Every encoder-flip transition: slave mode on and off with I2S slots present, the last I2S slot retyped to S/PDIF under slave mode, the first I2S slot added under slave mode, and a boot into a slave-mode preset with I2S slots.
4. With PIO0 at 32 of 32 in I2S master-clock mode, slot retypes in every order (the lazy program loads must always find room).

## Known limits

- PIO0 has no spare instruction in the internal-clock I2S configuration, so any future PIO0 program needs this encoder off or the master-clock programs merged (slaves could run clkout with side-set on the clock pins, freeing 8).
- The software encoder stays in the build for the external-clock configuration. The dead producer-give variants in `sample_encoding.cpp` were already unreferenced before this change and remain a cleanup candidate.
