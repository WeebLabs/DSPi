# S/PDIF Output: Biphase-Mark Encoding in the PIO

*Status: implemented. First version (two words per subframe, software NRZI fallback) 2026-10-11. Replaced 2026-11-20 by a 14-instruction one-word-per-subframe encoder; the NRZI fallback is deleted. Waveform equivalence proven in simulation (`tools/spdif_pio_bmc_sim.py`). Not yet verified on hardware.*

*Last updated: 2026-11-20*

## Summary

The PIO does all biphase-mark encoding for S/PDIF output. A 14-instruction program (`audio_spdif_bmc.pio`) is fed one 32-bit word per subframe holding the preamble selector, the 24-bit sample and V/U/C, and produces a waveform identical, cycle for cycle, to the original 4-instruction NRZI program fed with software-encoded transition words. Each frame is 256 PIO cycles as before, with the same two-cycle start lead-in, but the ring now holds **2 words per frame** (8 bytes), the same as the I2S rings.

CPU work per frame is two loads, a shift/mask/or per sample, and two stores. For 152 of every 192 frames (IEC phase 40 and up) the channel-status bit is zero and the preamble is the constant X/Y pair, so the writer runs those frames through a branch-free inner loop. Silence there is two constant stores per frame.

There is one ring word format. The software NRZI encoder, its lookup table, the `pio_bmc` instance flag, `audio_spdif_select_encoder()` / `audio_spdif_loaded_encoder()` and the encoder-flip rebuild in `process_type_switches()` are gone.

## PIO0 instruction budget

The slot block (PIO0) holds the S/PDIF program next to whichever I2S programs are loaded.

| Configuration | PIO0 residents |
|---|---|
| No I2S output slots | S/PDIF 14 |
| I2S slots, internal clocking | S/PDIF 14 + clkout 8 + dataout 8 = 30 |
| I2S slots, external master (clock slave) | S/PDIF 14 + extclk program (must be 18 or fewer) |

The extclk row depends on the I2S external-clock program being shrunk to 17 instructions (separate work). Until it is, that configuration does not fit and `audio_spdif_setup()` or the I2S setup panics at program load. Both libraries release their programs when their last instance on the block goes, so a full rebuild always starts from an empty block.

## Wire format in the ring

One word per subframe, OSR shift right, autopull off, pull threshold 31.

| Bits | Content |
|---|---|
| [1:0] | lead low-loop count: X = 2, Y = 1, Z = 0 |
| [3:2] | trail low-loop count: X = 0, Y = 1, Z = 2 |
| [27:4] | 24-bit sample, LSB first on the wire (time slots 4 to 27) |
| [28], [29] | V and U, both 0 |
| [30] | channel-status bit for this frame's phase |
| [31] | ignored (the PIO generates parity from the line level) |

Constants: X = 0x2, Y = 0x5, Z = 0x8 (`SPDIF_WORD_PRE_X/Y/Z`). Frame at IEC phase p with samples l, r and channel-status bit c:

- word 0 = `((l & 0xFFFFFF) << 4) | (c << 30) | (p == 0 ? Z : X)`
- word 1 = `((r & 0xFFFFFF) << 4) | (c << 30) | Y`

Silence is the same with l = r = 0.

## The program

```
.program audio_spdif_bmc
.side_set 1
.wrap_target
L_bit:    out x, 1          side 1          ; bit half 1, line high
          jmp !x H_high2    side 1          ; bit 0: half 2 stays high
L_low2:   jmp !osre L_bit   side 0 [1]      ; bit 1: half 2 low. 31 bits done at low: P=1
          nop               side 1 [1]      ;   P half 1
          pull              side 0 [1]      ;   P half 2, next word   <- sync-start entry
common:   out y, 2          side 1 [5]      ; preamble high x6, lead count
lead:     jmp y-- lead      side 0 [1]      ; low 2(y+1)
          out y, 2          side 1 [1]      ; high x2, trail count
trail:    jmp y-- trail     side 0 [1]      ; low 2(y+1), then wrap to L_bit
.wrap
H_bit:    out x, 1          side 0          ; bit half 1, line low
          jmp !x L_low2     side 0          ; bit 0: half 2 stays low
H_high2:  jmp !osre H_bit   side 1 [1]      ; bit 1: half 2 high. 31 bits done at high: P=0
          pull              side 0 [2]      ;   P both halves low (3 of 4 cycles)
          jmp common        side 0          ;   4th cycle
```

Cycle budget per subframe: preamble 6 + 2(lead+1) + 2 + 2(trail+1) = 16 with lead + trail = 2, then 27 data bits (sample, V, U, C) at 4 cycles = 108, then parity 4, for 128. The 4 preamble bits plus 27 data bits make 31 shifted bits, which is where OSRE trips with threshold 31.

`spdif_bmc_program_init()` sets `sm_config_set_out_shift(&c, true, false, 31)` and jumps to `entry_point` (offset + 4); `audio_spdif_enable_sync_prepare()` jumps to the same entry through `spdif_entry_offset()`.

## Ring writer

The stamping loops live in `include/pico/audio_spdif/ring_stamp.h` (plain C, no SDK dependency) so the host test compiles the real code. `spdif_stamp_audio()` and `spdif_stamp_silence()` split every run at the ring wrap and at IEC phases 0, 40 and 192. Segments at phase 40 and up use the constant X/Y words; segments below 40 read the channel-status bit per frame and use Z at phase 0. Both are always inline into the RAM-resident writer, skip and pump (the silence path runs during flash blackout). Absolute-frame phase, erase-ahead, all-or-nothing admission and the `wr_frames` / `erased_frames` / `consumed_frames` semantics are unchanged.

## Why it is alignment-safe

- **Frame timing is unchanged.** Every subframe is exactly 128 PIO cycles, so the 256 Fs PIO clock and `audio_base_divider_16_8()` are untouched.
- **Start is unchanged.** The synchronized start execs a jump to the entry instruction and enables all SMs in one mask write. The entry emits two low cycles before the first preamble bit, as the NRZI program did, so S/PDIF slots keep their phase relative to I2S slots and PDM.
- **Ring accounting is per frame.** All fill, deficit and skip math uses `consumed_frames` and `wr_frames`; only the words-to-frames divisor changed (4 to 2).
- **FIFO depth.** The joined 8-word TX FIFO now holds 4 frames instead of 2, the same as I2S slots. The DMA read pointer, and so `consumed_frames`, leads the wire by 4 frames on both slot types. Wire alignment never depended on this (every slot starts from the same ring offset on the same cycle). The only effect is that S/PDIF and I2S fill readings no longer differ by the 2-frame FIFO term.
- **Underrun.** A `pull` on an empty FIFO stalls with the pin held; the ring's silence margin makes that unreachable in normal operation.
- **Parity cannot drift.** The PIO derives the parity bit from the line state.

## Verification

**Simulation.** `tools/spdif_pio_bmc_sim.py` runs the retired NRZI program (the golden model, with a Python port of the old software encoder) and the 14-instruction program on the words above, and compares the pin level every cycle on five seeds, including zero, full-scale and sign-extended samples, channel-status wraps and the first eight cycles after enable.

**Stamping.** `tools/spdif_ring_stamp_test/run.sh` compiles `ring_stamp.h` with the host compiler and checks both loops against a naive per-frame reference for 200,000 random runs (random and boundary-targeted start positions, lengths up to a full ring, ring wrap, phases 0, 40 and 192, uint32 counter wrap).

**Bench items still open.**

1. `JMP !OSRE` with autopull disabled exits after exactly 31 bits; a scope check that every subframe is 128 cycles settles it.
2. Receiver lock on all slots at 44.1, 48 and 96 kHz, including the fractional-divider rates.
3. Mixed S/PDIF and I2S slots in both I2S clock modes, checking sample alignment across slot types and PDM.
