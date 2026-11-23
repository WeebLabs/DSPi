# DSPi Stereo Spatializer

*Design spec, 2026-11-23, revision 5: revision 4 as agreed with Fable, then the implementation changes in section 6 (10 ms gain ramps, XTC gains from 0 after a reset, mid-path tail and warm ring) and 7.3 (tap order). Status: implemented, host- and loopback-tested on RP2350; the RP2040 gate of 7.3 is unmeasured. Both platforms (RP2350 float, RP2040 Q28). Supersedes section 10 of `upmixer_v2_spec.md` (width as a separate stage), widening it to a module with several modes.*

---

## 1. Summary

The Spatializer changes the apparent width of a stereo image on selected output pairs. It is its own stage, separate from the upmixer and from crossfeed. It runs per output pair after the matrix and immediately before crossfeed, adds no latency, and runs on both platforms.

Two modes, each from published work:
- **Width.** Mid/side width with a Blumlein/Gerzon shuffler: the side signal gets one gain above a corner frequency and another below it. The mono sum is untouched. The safe default.
- **Crosstalk cancellation (XTC).** The recursive loudspeaker crosstalk canceller of Atal and Schroeder (1962) and Cooper and Bauck (1989), in the band-limited recursive form Glasgal and Miller published as RACE (AES 123, paper 7301, 2007): an inverted, delayed and attenuated copy of each output is fed to the other, within about 250 Hz to 5 kHz. It widens the image beyond the speakers for a centred listener, with a narrow sweet spot. A mid-path amount trades true cancellation for an untouched centre (section 3.3).

A mono-to-stereo mode (Lauridsen/Schroeder complementary combs) is deferred (section 11).

## 2. What it is not

- Not a headphone process. Crossfeed (BS2B) stays as it is, with its own commands and presets. The two are not mutually exclusive in firmware; the Console warns when both are on, since XTC is for speakers and crossfeed for headphones.
- Not part of the upmixer. It acts on whatever the matrix and upmixer deliver to its pairs.
- Not HRTF virtualization or Haas widening: the first needs convolution, the second comb-filters and is not mono-safe.

## 3. Signal flow

### 3.1 Delta form

Per selected output pair (outputs 2p and 2p+1; the mono PDM sub is never a pair), with M = (L + R)/2 and S = (L - R)/2:

```
L_out = (L + f * (dM + dS)) * T
R_out = (R + f * (dM - dS)) * T
```

The kernel never rebuilds L and R from M and S. It adds a mid delta dM and a side delta dS to the untouched inputs, so a zero delta is exact passthrough on both platforms. The recursions of section 3.3 run on the unfaded deltas (M' = M + dM, S' = S + dS); only the deltas added to the outputs are scaled by f and T. f is the pair's fade gain (section 6) and T = 1 + f * (t - 1) with t the linear trim. At f = 0, or with both deltas zero and trim 0 dB, the output is bit-exact with the input; the multiply by T is skipped at t = 1.

Every output sample is the input sample of the same index plus terms built from that index and older ones. The delayed terms are feedback only. No output shifts in time and both outputs of a pair advance by the same samples (CLAUDE.md alignment rule).

### 3.2 Width

- dM = 0.
- dS = (w - 1) * S + (wb - w) * LP(S), with LP a one-pole low-pass at the shelf corner fc, so the side gain is w above the corner and wb below it.
- w = width / 100, wb = bass width / 100 (each 0 to 2).

At w = wb = 1 both coefficients are exactly zero and the stage is bit-exact. L_out + R_out = L + R in every setting: exactly in Q28, and within one rounding step in float.

### 3.3 Crosstalk cancellation

The crosstalk path is H(z) = B(z) * z^-D. B is a first-order low-pass at 5 kHz and a first-order high-pass at 250 Hz, normalised by k so its peak magnitude is 1 (k = 0.999 / computed peak, so Q28 rounding cannot exceed 1). The chain is linear, so its order is free; the kernel uses the interpolated tap, then g * k, then the low-pass, then the high-pass, which keeps every multiplied value small on RP2040 (section 7.3). D is the interaural delay for the speaker half-angle theta, from Woodworth's spherical head, less the low-pass's own low-frequency phase delay:

```
d       = (a / c) * (theta + sin theta) * fs        a = 8.75 cm, c = 343 m/s
x       = exp(-2 * pi * 5000 / fs)                  the low-pass pole
D       = max(1.0, d - x / (1 - x))                 samples, at least one whole sample
```

At 48 kHz, d is 2.1 samples at 5 degrees, 4.3 at 10, 6.4 at 15 and 12.5 at 30; the low-pass removes 1.08. The worst case is 30 degrees at 96 kHz: d = 25.1 samples. The one-sample floor applies at 5 degrees and 44.1 kHz (1.96 - 0.96). The fractional part of D uses linear interpolation (0.47 dB loss at 5 kHz at half a sample, inside B's own roll-off). The tap is read first and B applied after it, so B smooths the interpolation, the delay slew and the g ramp.

The recursion in L/R is RACE's: L_out = L - g * H(R_out), R_out = R - g * H(L_out). Its sum and difference are exactly the mid/side shuffler:

```
S' = S / (1 - g * H)    -> dS = +g  * H(S')
M' = M / (1 + gm * H)   -> dM = -gm * H(M')
```

- g = 10^(-atten / 20), the RACE attenuation per pass: atten 2.5 dB to 20 dB, default 3 dB (g = 0.708). G_MAX = 0.75 (2.5 dB).
- gm = g * mid / 100, with mid 0 to 100 %, default 100 (the full canceller). At 100 % both ears get exactly the source channel (section 9's null test). At 0 % the mid path is untouched: the centre stays electrically clean and the stage becomes a side-only widener, cancelling a hard-panned source's crosstalk by at most 6 dB. Values between trade centre colouration against cancellation.

Stability: |g * B| <= 0.75 < 1 at every frequency, so both recursions are stable for any delay by the small-gain argument, including while g, gm or D ramp (section 6), since they stay within the same bound. The interpolator has gain at most 1.

## 4. Parameters

One global configuration drives every selected pair (crossfeed's model). Per-pair settings would need new commands (section 8.4).

| Field | Type | Range | Default | Meaning |
|---|---|---|---|---|
| enabled | u8 | 0/1 | 0 | On or off |
| mode | u8 | 0 width, 1 XTC | 0 | Algorithm |
| pair_mask | u8 | bits 0..N-1 | 0x01 | Output pairs it runs on (N = 2 on RP2040, 4 on RP2350); higher bits are masked off |
| width_pct | f32 | 0 to 200 | 125 | Side gain above the corner (width mode) |
| bass_width_pct | f32 | 0 to 200 | 150 | Side gain below the corner (width mode); Gerzon lifts the side most at low frequencies |
| shelf_hz | f32 | 100 to 1500 | 700 | Width-mode corner (Gerzon's shuffler corner is near 700 Hz) |
| span_deg | f32 | 5 to 30 | 10 | Speaker half-angle (XTC); 10 degrees is 89 us, inside RACE's 60 to 100 us. A setup value: it suits a narrow RACE layout and is wrong for a standard 30 degree triangle, so the Console should ask for it |
| atten_db | f32 | 2.5 to 20 | 3 | XTC attenuation per pass (g = 10^(-atten/20)); the 3 dB default is a DSPi choice for stability margin and Q28 headroom, not a published RACE figure |
| mid_pct | f32 | 0 to 100 | 100 | XTC mid-path amount (100 = full canceller, 0 = side only) |
| trim_db | f32 | -12 to 0 | 0 | Output gain of the stage, both modes |

- A single SET clamps and stores the clamped value, and the GET returns it (commands.md rule 14). Bulk and flash loads store as loaded and clamp when coefficients are computed. A non-finite float is refused on SET and replaced by the default on load.
- An unknown mode from newer firmware loads as mode 0 with the stage off.
- The band edges (250 Hz, 5 kHz) are fixed constants: they are the published values, and accurate delay matters more than band tuning.

## 5. Level cost

The figures are for the stage alone with trim at 0 dB. The output limiter catches peaks on RP2350; on RP2040 the stage saturates its own internal values first (section 7.3).

- **Width.** The worst case is anti-phase content (L = -R): L_out = w * L, so 200 % is +6.0 dB. A hard-left source gives L = (1 + w)/2 and R = (1 - w)/2: +3.5 dB on L and an anti-phase -6 dB on R at 200 %. Below 100 % the side only shrinks.
- **XTC.** Both paths are combs between 1/(1 + g|B|) and 1/(1 - g|B|), with the mid path scaled by mid_pct. The bound is -4.9 dB to +12.0 dB at G_MAX = 0.75. At the default (3 dB, g = 0.708) it is -4.7 dB to +10.7 dB. On real spans the peaks sit where B is below 1: at the default span the side path's main lift is about +9 dB near 500 Hz and the mid path's about +8 dB in the upper midrange, where it lands on vocals. Exact figures per span come from the float64 model (section 9). Lowering mid_pct reduces the mid lift.
- The steady-state sine bounds above are not peak bounds: the recursion's impulse-response L1 norm is larger. Section 9 measures it at every limit.

## 6. Ramps and switching

- **Fades.** Each pair state holds `running_mode` and a fade gain f counted in samples (fade length `round(0.010 * fs)`). A pair fades up when it should run and down when it should not. A pair whose running mode differs from the target mode fades to 0, adopts the target, resets its mode state and fades up. After any reset, XTC also starts its loop gains g and gm at 0 and ramps them to target over the same 10 ms, because its ring and band-pass start empty and a cold onset at full loop gain measured about 20 dB above a warm fade. The target enable and mode come from the published coefficient set, which always holds both modes' coefficients, so a run of SETs inside one fade never strands a pair: every packet just reads the newest target. The pair mask is read live, as crossfeed's is: the pipeline snapshots it from the config once per packet next to the coefficient pointer and hands both to core 1 in `core1_eq_work`, so pair 0 on core 0 and the pairs on core 1 agree.
- **Idle cost.** An idle pair (target off and f = 0) costs one test per packet. Its state carries a dirty flag and is cleared once, when it goes idle, not every packet (the subharm precedent; crossfeed clears every packet, which is fine at its 16 B but not at about 300 B).
- **Gain ramps.** When any target among w, wb, g, gm and t moves, every one of them ramps linearly, per sample, from the value in use to its new target over one fade length (10 ms). A further change restarts the ramp from wherever it is. The result does not depend on packet size. (Revision 4 ramped across one packet, the master-volume convention; on the loopback bench a 48-sample ramp of a 3 dB trim step read 9 dB above an ideal 10 ms fade, and the host model put every parameter there.)
- **Delay slew.** D moves toward its target at a fixed rate of 1/64 sample of delay per output sample, independent of packet size, so a full 5 to 30 degree move takes about 17 ms at 96 kHz. The interpolator moves the tap smoothly, and B is applied after the tap. The slew limit keeps the effective loop gain under 0.75 x 1.01 (a shrinking delay can raise energy by roughly 1/sqrt(1 - slew)), so the loop stays stable while it moves.
- **Corner.** The width low-pass coefficient ramps with the gains over 10 ms. Its state is continuous, but an instant coefficient jump kinks the filter's slope, which measured up to -59 dBFS above 5 kHz on the loopback bench depending on the signal phase at the change.
- **Mid path.** gm ramps like g. The mid recursion keeps running for one fade length after gm settles at 0, so its band-pass tail decays instead of being cut, then is skipped and its two filter states are cleared. Its ring always receives M + dM (M while skipped), which is the recursion's true past output, so gm ramps back in from a warm state. On the host model both transitions measure at or below an ideal 10 ms crossfade above 5 kHz.
- **Hard skips.** A pair that carries a RAW signal-generator output, or that does not have both outputs enabled, is skipped at once with its state reset and f forced to 0, so its return fades in. This keeps two existing guarantees: a RAW output is never processed, and a disabled output's zeroed buffer never receives bleed (see `crossfeed_process_pairs`).
- **Rate change.** The published set carries the sample rate it was computed for. A pair whose stored rate differs resets its state and fades in.

## 7. Pipeline and resources

### 7.1 Placement

PASS 4.5, immediately before crossfeed, on the core that owns the pair (core 0 for pair 0, core 1 for the rest in dual-core mode), from one per-packet snapshot of the published coefficient pointer handed to core 1 in `core1_eq_work` as crossfeed's is. The publish model is crossfeed's: compute into the inactive buffer of two, then swap the pointer; the pointer is never NULL while the stage exists, because a fading pair still needs coefficients.

### 7.2 State and RAM

- Per pair: running mode, fade gain, the last-used ramp values; width one low-pass state; XTC per path two band-pass states and a 32-sample ring (for D up to 25.1 samples plus the interpolation tap). About 300 B per pair, 600 B on RP2040 and 1.2 KB on RP2350 of `.bss`.
- Two kernels (width, XTC), so the default mode stays small. `DSP_TIME_CRITICAL`. Expected RAM code: 0.8 to 1.2 KB on RP2040 and 0.5 to 0.8 KB on RP2350, against 684 B and 640 B left in the `.data` budgets. **A budget raise is required on both platforms**: measure, raise `DATA_BUDGET` in `scripts/check_ram_placement.py`, and record it in the Memory Layout section. Measured (revision 5): RAM code 3,124 B on RP2040 and 2,070 B on RP2350; `.bss` 368 B per pair on both (736 B and 1,472 B), plus 96 B of coefficient buffers; `DATA_BUDGET` raised from 72,704 to 77,312 B and from 102,400 to 105,472 B.

### 7.3 RP2040 (Q28)

Q28 holds +-8.0 in int32 and `fast_mul_q28` does not saturate (it is exact for a multiply by 1.0). Its intermediate `mid1 + mid2` also overflows unless |a| + |b| < 8.0, giving a 2^-8 error, so **every `fast_mul_q28` call must have |a| + |b| < 8.0**; the rows below list the operands where that is not obvious. The input L, R to the stage can exceed 1.0 (two full-scale matrix rows sum to 2.0). Every intermediate value is bounded so none can wrap; psybass's pre-drive clamp is the precedent:

| Value | Formed as | Clamp | Bound |
|---|---|---|---|
| M, S | `(L >> 1) +- (R >> 1)` | +-2.0 | 2.0 (the direct path L, R is not clamped) |
| Ring write S', M' (XTC) | S + dS, M + dM | +-3.9 | 3.9 (at most 4.9 before the clamp) |
| Scaled tap (XTC) | g_eff times the interpolated tap a(1 - fr) + b fr, g_eff = g * k <= 0.79 | none | 3.1; operands 3.9 and 1.0, then 3.9 and 0.79 |
| Low-pass, 5 kHz (XTC) | one-pole on the scaled tap, either form | none | 3.1; `x - y` at most 6.2 |
| High-pass, 250 Hz (XTC) | v - LP(v), never multiplied afterwards | none | 6.2 |
| dS, dM (XTC) | the high-pass output (gm's share as its own weights) | +-2.9 | 2.9; dM + dS at most 5.8 |
| Width low-pass | one-pole on S, either form | none | 2.0; `x - y` at most 4.0 |
| dS (width) | (w - 1) S + (wb - w) LP(S); operands 1 and 2, 2 and 2 | +-5.8 | 5.8 (at most 6.0 before the clamp) |
| Faded delta | f * (dM + dS); operands 1.0 and 5.8 | none | 5.8 |
| Output, trim 0 dB | L + faded delta, saturating | +-7.9 | 7.9; the T multiply is skipped |
| Output, trim below 0 dB | L + faded delta, saturating, then times T | +-6.9, then T | 6.9; operands 6.9 and 1.0 |

Per path that is an M or S clamp, a ring-write clamp and a delta clamp in XTC (one delta clamp in width), plus one saturating add per output. On normal programme (inputs within full scale) none of the clamps acts; they only stop a wrap or a multiply error from being recirculated.
- Cost: XTC full is 12 `fast_mul_q28` per sample per pair (per path three for the scaled tap and two one-poles, then two fade multiplies), 15 with trim, against crossfeed's 8. Savings: skip trim at 0 dB, skip the mid path at gm = 0. Static estimate from the RP2040 disassembly: about 480 cycles per sample per pair, 540 with trim, so both pairs at 96 kHz take about 15 % of each core at 307.2 MHz in dual-core mode, and about 30 % of core 0 in single-core mode (core 1 running PDM). Unmeasured, and a lower bound: the multiplies are out-of-line calls. **Gate**: XTC on both RP2040 pairs at 96 kHz must fit in the measured core budget; if it does not, XTC is limited to one pair on RP2040 and the spec records that.
- The output limiter should link the two outputs of a pair running XTC, since unlinked gain reduction breaks the cancellation. That is a setup note for the Console, not firmware behaviour.

## 8. Host interface and persistence

### 8.1 Commands (page FX, revision 2 to 3)

| Cmd | Name | Dir | Size | Notes |
|---|---|---|---|---|
| 0xB0 | `SPATIAL_CONFIG` | G, S | 32 | The whole section (8.2). A SET validates every field first (finite floats, mode 0 or 1) and changes nothing on refusal; it then clamps floats and masks the pair mask. |
| 0xB1 | `SPATIAL_ENABLE` | G, S | 1 | u8 bool |
| 0xB2 | `SPATIAL_MODE` | G, S | 1 | u8; above 1 is `VC_ERR_VALUE` |
| 0xB3 | `SPATIAL_PAIRS` | G, S | 1 | u8 mask, masked to the pair count (as `FX.CROSSFEED_OUTPUTS` is); read live, not deferred |
| 0xB4 | `SPATIAL_WIDTH` | G, S | 4 | f32 percent, clamped |
| 0xB5 | `SPATIAL_BASS_WIDTH` | G, S | 4 | f32 percent, clamped |
| 0xB6 | `SPATIAL_SHELF` | G, S | 4 | f32 Hz, clamped |
| 0xB7 | `SPATIAL_SPAN` | G, S | 4 | f32 degrees, clamped |
| 0xB8 | `SPATIAL_ATTEN` | G, S | 4 | f32 dB, clamped |
| 0xB9 | `SPATIAL_MID` | G, S | 4 | f32 percent, clamped |
| 0xBA | `SPATIAL_TRIM` | G, S | 4 | f32 dB, clamped |

All take `wValue` 0. Enable, mode and the floats are `VC_F_DEFERRED`. 0xBB to 0xCF stay free for the deferred mono-to-stereo mode. The FX page revision goes to 3 (no build reporting 2 has shipped, but a bench Console built against 2 can then tell them apart).

### 8.2 Wire section (32 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | enabled |
| 1 | u8 | mode |
| 2 | u8 | pair_mask |
| 3 | u8 | reserved (zero on GET, ignored on SET) |
| 4 | f32 | width_pct |
| 8 | f32 | bass_width_pct |
| 12 | f32 | shelf_hz |
| 16 | f32 | span_deg |
| 20 | f32 | atten_db |
| 24 | f32 | mid_pct |
| 28 | f32 | trim_db |

### 8.3 Persistence and notifications

- Wire V36 appends the section to `WireBulkParams` (6252 to 6284 bytes). Bulk apply masks the pair mask to the platform's pair count, so an RP2350 preset loads on RP2040.
- Slot V43 tail-appends the same 32 bytes. Older slots load the defaults with the stage off.
- Factory reset: defaults, stage off.
- Every field notifies through one shared encoder at its `WireBulkParams` offset (the upmixer and tube pattern).

### 8.4 Later per-pair settings

commands.md rule 4 lets a command gain an address only if address 0 keeps its meaning, and "global" cannot later mean "pair 0". Per-pair settings would therefore need new commands.

### 8.5 Follow-ons in the same change

Regenerate `dspi_commands.json` and the commands.md tables; add commands.md section 13.6; record the longer bulk length for third-party clients (ESP32 front panel); architecture doc sections.

## 9. Testing

**Host (C harness against a float64 model, both kernels; the Q28 kernel within a stated error):**
- The acoustic null: model the crosstalk path as A = g * B * z^-D and check that at mid 100 % each ear receives exactly its source channel (eL = L, eR = R), and at mid 0 % the side ear signal is exact (eS = S).
- Bit-exact passthrough at 100 % / 100 % width and trim 0 dB, and at fade gain 0. Mono-sum invariance in width mode: exact in Q28, within one rounding step in float.
- Chunk independence within a packet for every path; packet-size independence for the delay slew and the fades (gain ramps follow packet boundaries by design, section 6).
- Q28 range: impulse-response decay, and a full-scale sine at the resonant peak at G_MAX, every span limit and all three rates, with trim at 0 dB and below 0 dB, and with inputs at +-2.0 and at +-7.9, compared against a saturating int64 reference of the same kernel. That catches both a true wrap and the 2^-8 multiply error.
- Delay range: the 30-degree span at 96 kHz fits the ring; the one-sample floor at 5 degrees and 44.1 kHz.
- The fade and switch logic: mode switch, two mode SETs inside one fade, mask edits, RAW start on a running pair, and an output disabled on a running pair (the disabled buffer stays exactly zero). The dispatcher over core 1's pair range runs on the host, since the loopback taps only one pair.
- Persistence: a V42 slot loads the defaults with the stage off (the `flash_storage.c` version gate is checked by inspection only; no test builds a V42 slot); an RP2350 mask loads on RP2040. A V35 bulk payload is refused, as every bulk version other than the current one is.

**Hardware loopback (RP2350 loopback build, pair 0):**
- Width gains on panned, centred and anti-phase tones against section 3.2.
- XTC side and mid responses against the model at a few frequencies, and zero lag between the two outputs.
- Clicks under signal for each live change: span sweep, mid change, corner sweep, mode switch, enable, and mask, measured against an ideal 10 ms fade and a hard-toggle control (the upmixer bench's method).
- A measured CPU figure per mode, and the RP2040 gate of section 7.3 on an RP2040 build.

## 10. Name

"Spatializer" was the trademark of Spatializer Audio Laboratories in the 1990s. The firmware calls the stage `spatial` internally; the user-facing name is the Console's call.

## 11. Deferred

- Mono-to-stereo (Lauridsen complementary combs). It needs a 10 to 20 ms delay per pair: fine on RP2350, tight on RP2040 across pairs.
- Console templates (desktop speakers, wide stereo, narrow for mono-heavy mixes).
- Per-pair settings (section 8.4).
- Control Surfaces nouns for enable and width.
