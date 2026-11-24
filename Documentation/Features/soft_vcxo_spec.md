# Soft VCXO: Fractional-N System PLL Trim for Input Clock Tracking

*Last updated: 2026-11-24 (section 8: shaped actuator prototype, `DSPI_VCXO_SHAPED`, RP2350 only). Previously 2026-11-16 (clock diagnostic command renamed for the paged protocol); 2026-10-11 (clock plan read from pll_sys at init)*
*Status: implemented, merged into release/v1.1.6 through `merge/spdif-rx-overhaul` (2026-10-11); RP2350 mechanism bench-proven standalone; full-firmware behavior HW-untested; RP2040 fbdiv dithering HW-untested.*

## 1. Purpose

When the DSPi tracks an external audio clock (SPDIF input, ADAT input in
slave clock mode, I2S slave mode), the output clocks must follow the
source's rate to keep the consumer buffers centred. Previously this was
done by trimming each output type's PIO clock divider. That approach had
three structural problems:

1. **Coarse steps.** The PIO divider is 16.8 fixed point. At 48 kHz the
   SPDIF TX divider is 25.0 (stored as 6400), so one fractional LSB is
   1/256 / 25 = 156 ppm. The servo had to dither between adjacent LSBs to
   fake sub-LSB rates, stepping every output clock by 156 ppm each time;
   unkind to downstream DAC PLLs.
2. **Many actuators.** SPDIF TX, I2S TX (forced to exactly 2x the SPDIF
   divider to avoid independent-rounding drift), MCK (CLK_GPOUT divider),
   and ADAT TX each needed their own coordinated write.
3. **PDM was untracked.** The PDM output's clock was never servoed at all.

The soft VCXO replaces all of that with a single actuator: the system PLL
itself is pulled by ppm amounts, exactly like warping the crystal of a
VCXO. Every output clock (SPDIF TX, internally clocked I2S TX, MCK, ADAT
TX, PDM) derives from sys_clk, so they all move together and inter-slot
sample alignment is preserved **by construction**; there is no per-output
write to get wrong and no divider rounding to coordinate.

## 2. Mechanism

### 2.1 Clock plan (identical on both platforms)

Default plan: sys_clk = 307.2 MHz from XOSC 12 MHz ref, refdiv 1, FBDIV 128
(VCO 1536 MHz), postdiv 5 x 1. Since 2026-10-11 `soft_vcxo_init()` reads
refdiv, FBDIV and both post-dividers from `pll_sys_hw` and stays inert unless
they reproduce `clock_get_hz(clk_sys)` (covers the RP2350 150 MHz boot
fallback), so any clock plan works.

### 2.2 Pulse-density modulation of FBDIV

The PLL has no fractional divider, so one is synthesized: DMA briefly
writes FBDIV = 127 or 129, then restores 128, at a controlled repetition
rate. While FBDIV is displaced by +-1 for a time t, the locked loop
accumulates exactly `f_ref * t` extra (or missing) VCO cycles; the loop
filter smooths the step so sys_clk sees only a tiny phase ramp. The
average frequency offset is therefore proportional to the pulse rate and
completely deterministic:

```
pulse width      t   = 150 sys cycles = 150 / 307.2e6 s      (~0.49 us)
VCO cycles/pulse     = f_ref * t = 12e6 * 150 / 307.2e6 = 5.859375
sys cycles/pulse     = 5.859375 / postdiv1(5)           = 1.171875
pulse rate for 1 ppm = 307.2 / 1.171875                 = 262.144 Hz
```

In general the rate is sys_MHz x FBDIV / `SOFT_VCXO_PULSE_CYCLES` (150) Hz
per ppm, computed at init; 262.144 at the default plan. A standalone RP2350 bench test measured
0.003843 ppm/Hz against the analytic 0.003815 ppm/Hz (within 1%); since
the constant sits inside the input servo's feedback loop, residual gain
error is absorbed by the fill terms and **no calibration pass is needed**
(the bench prototype's `calibrate()` was deliberately dropped).

The control quantum is one pulse = 1.17 sys cycles of phase (~3.8 ns),
versus the old servo's 156 ppm frequency steps. At the minimum pulse rate
the offset resolution floor is ~0.11 ppm; below that the VCXO parks.

### 2.3 DMA/PWM topology (zero IRQs, zero steady-state CPU)

Two DMA channels, DMA timer 0, and one PWM slice used purely as a DREQ
pacer (no GPIO is driven):

```
PACE  channel: trans_count 1, DREQ = PWM wrap.
               Writes &fbdivs[0] into PULSE's AL3_READ_ADDR_TRIG
               (re-points the read address AND triggers PULSE).
PULSE channel: trans_count 2, DREQ = DMA timer 0 (sys * 1/150 = 2.048 MHz).
               Streams fbdivs[] = {excursion, nominal} into
               pll_sys_hw->fbdiv_int, then CHAIN_TO PACE.
```

Endless operation uses mutual re-arming: a channel's transfer count
reloads from its last programmed value on every trigger (both platforms),
so PULSE's completion chain re-arms PACE for the next PWM wrap and PACE's
write re-arms PULSE. `dma_channel_start(PACE)` bootstraps the loop once at
init. This deliberately avoids the RP2350-only ENDLESS trans-count mode so
the code is identical on RP2040.

The pulse width is one DMA-timer period: after PACE triggers PULSE, the
first transfer waits for the next timer DREQ edge (writes the excursion
value), the second waits one full period (restores nominal). Timing
jitter on the trigger does not matter; the inserted phase per pulse is
fixed and the rate is set by the PWM.

When parked (|ppm| below ~0.11), `fbdivs[0]` is set to the nominal 128 so
the pulses become harmless no-op writes; the loop is never stopped.

`fbdivs[]` and `fbdivs_addr` are non-const statics (.data, RAM): the pulse
train keeps running through flash blackout windows, and DMA reads from
XIP during a flash op would stall.

### 2.4 Command path

`soft_vcxo_set_ppm(float ppm)` (RAM-resident, `DSP_TIME_CRITICAL`):

1. Clamp to +-`SOFT_VCXO_MAX_PPM` (1000, the IEC 60958-3 Level II tolerance).
2. `freq = |ppm| * 262.144`; below 30 Hz, park (see above).
3. Set `fbdivs[0]` = 129 (ppm > 0) or 127 (ppm < 0).
4. Program the PWM pacer: pick the smallest power-of-two clkdiv (1..128)
   whose wrap fits 16 bits, then `wrap = round(307.2e6 / clkdiv / freq)`.
   Reachable rate span: ~18 Hz to well past the 262 kHz needed at
   1000 ppm (duty at 1000 ppm is ~13%, so pulses never merge).

A sign flip or rate change mid-pulse can at worst mis-sign one pulse
(~0.004 ppm-seconds of phase); the servo loop absorbs it. Changing the
PWM wrap while the counter is above the new top lets the counter run to
0xFFFF once (one long period); equally negligible.

### 2.5 Bus access

RP2350: ACCESSCTRL resets PLL_SYS to core-only access, so
`soft_vcxo_init()` writes `accessctrl_hw->pll_sys = 0xACCE0000 | 0xFF`
to admit the DMA master. (`hardware/pll.h` macro-aliases `pll_sys`, which
must be `#undef`'d first; see soft_vcxo.c.)
RP2040: no ACCESSCTRL; the DMA master can already reach APB peripherals.
DMA-to-PLL writes on RP2040 are architecturally fine but not yet
hardware-verified.

## 3. Servo integration

### 3.1 Structure (input_servo.c)

`input_servo_apply(float actual_freq, int fill_slot)` is shared by all
three tracked input modes. Per ~20 ms tick:

```
div    = ceil(sys_nominal / audio_state.freq)        16.8-as-integer, ==
                                                     the TX library's own
                                                     update_pio_frequency()
ppm_ff = (actual_freq * div / sys_nominal - 1) * 1e6
```

Referencing the **programmed divider** (not the nominal rate) folds the
divider's ceiling-rounding residual into the feed-forward, so the
integrator never has to hold it. `sys_nominal` is `clock_get_hz(clk_sys)`,
which by convention keeps reporting 307.2 MHz; all nominal-divider math
everywhere stays nominal, only the physical clock moves.

Fill terms (only when `fill_slot >= 0`), in ppm convention (overfull
consumer = outputs too slow = positive ppm = speed sys_clk up).

Tuning history: the original divider-servo gains, expressed on a
continuous actuator, limit-cycled the integrator between its clamp rails
(-15..+50 ppm, ~10 ppm/s slews) on zero true error because the fill was
quantized to whole buffers; the external DAC receiver PLL unlocked on
the slews ("fill locked at 50-56% but the DAC cuts out"). A first retune
(2026-07-22) bounded the cycle with an error-gated integrator, a
+-10 ppm clamp, and a slew limiter; hardware confirmed carrier motion
dropped to ~0.25 ppm/s and the DAC held lock, with the bounded +-10 ppm
wander remaining as predicted.

Current form (2026-07-23): the quantizer itself was removed. Loop B
reads `get_slot_consumer_fill_frac()`, a sample-granular fill (queued
whole buffers plus the producer staging position plus the DMA in-flight
remainder, in buffer units), IIR-smoothed (alpha 0.125, ~160 ms). On a
continuous error a plain PI converges to a fixed point, so the
integrator gate is gone and the fill centres on 8.0 exactly:

- P: 2 ppm per buffer of smoothed error (correction bandwidth ~0.002
  rad/s against the ~0.001 buffer/s-per-ppm plant at 48 kHz), plus a
  second 4 ppm/buffer slope beyond +-2 buffers for real displacements.
- I: 5e-5 ppm/tick per buffer (zero below the P crossover for damping),
  clamp +-3 ppm; it only holds the static residual (estimator bias,
  tune-ratio gain error).
- Global slew limiter: unchanged, 0.5 ppm per ~20 ms tick (~25 ppm/s).
  `input_servo_reset()` zeroes the limiter and integrator and re-seeds
  the fill IIR at target.
- An invalid fill reading (negative: type switch in progress) holds the
  previous command for that tick instead of actuating on garbage.

A guard skips the tick when `|ppm_ff| > 2000`: that only happens if a
caller applied against the wrong pipeline rate.

`input_servo_reset()` zeroes the integrator and parks the VCXO. It is
called on tracking start, (re)lock, and every stop/disarm path, so USB
and I2S-master modes always run at exactly nominal sys_clk.

### 3.2 Per-mode notes

- **SPDIF input** (spdif_input.c): unchanged estimator (long-window block
  count, IIR bridge). New gate: servo holds off until
  `spdif_rx_detected_rate == audio_state.freq`, i.e. until the deferred
  rate change lands (previously it slewed dividers toward the wire rate
  during the muted window; with feed-forward computed against the current
  rate's divider that would briefly command garbage). Rate whitelists
  match: `is_supported_rate()` and `perform_rate_change()` both accept
  exactly {44100, 48000, 96000}.
- **ADAT slave** (adat_input.c): already gated on rate match; passes fill
  slot 0 like SPDIF.
- **I2S slave** (i2s_input.c): now shares `input_servo_apply()`; its old
  private servo (P-only, divider writes to SPDIF slots + ADAT) is gone.
  It passes the first SPDIF-type slot as fill reference, or -1 when no
  SPDIF-type slot exists (rate term alone, ~0.1 ppm long-window). The
  edge-locked I2S TX slots follow the external BCK directly and need no
  servo; PDM and SPDIF slots now track via sys_clk (PDM previously did
  not track at all in this mode). The I2S slave gains an integrator it
  never had, fixing the park-at-deadband-edge behavior class.
- **Divider forcing moved to the nominal path**: the old
  `i2s_div = 2 * spdif_div` servo coupling existed to stop independent
  rounding from drifting output types apart; a common sys_clk trim can
  never correct a divider RATIO error, so the coupling must live in the
  nominal dividers themselves. `i2s_compute_divider()` in the I2S TX
  library (and the matching RX clock-master divider) now derives the I2S
  divider as exactly `2 * ceil(sys/f)` instead of `ceil(2*sys/f)`. The two
  formulas are identical at every supported rate (44.1/48/96 k); the
  change future-proofs rates like 176.4 kHz where they differ by one LSB
  (~287 ppm ratio error).

### 3.3 What was deleted

- All divider writes from servo code (`pio_sm_set_clkdiv` trims, MCK
  divider trims, `audio_i2s_mck_set_divider` servo calls).
- `servo_cache_base_dividers()` and the cached base dividers.
- `adat_output_servo_divider()`, `adat_servo_div`, and the resync-time
  servo-divider pickup in adat_output.c; ADAT TX now always runs the
  nominal divider.
- `spdif_input_current_tx_divider()`, `i2s_slave_current_tx_divider()`,
  `adat_input_current_tx_divider()`, `input_servo_current_divider()`
  (their only consumer was the ADAT pickup above).

Nominal dividers are owned exclusively by the existing rate-change paths:
`restore_nominal_spdif_dividers()`, `audio_i2s_update_all_frequencies()`,
`audio_i2s_mck_update_frequency()`, and the ADAT resync.

## 4. Resource map

| Resource | RP2040 | RP2350 | Notes |
|---|---|---|---|
| DMA PULSE channel | 10 | 11 | explicit `dma_channel_claim` at boot |
| DMA PACE channel | 11 | 12 | explicit claim at boot |
| DMA timer | 0 | 0 | first use of DMA timers in the firmware |
| PWM slice | 7 | 8 | pacer only, no GPIO |
| RAM | ~24 B statics | ~24 B statics | plus ~0.3 kB code (set_ppm in RAM) |

- **RP2350 slice 8** has no bondable GPIO on RP2350A, so it can never
  collide with a control-surface PWM LED. **RP2040 slice 7** covers GPIOs
  14/15; the CS LED bind path now rejects pins on the reserved slice with
  `CS_STATUS_PWM_CONFLICT` (checked on both platforms for uniformity).
- To free RP2350 channels, the I2S RX input rings were converted from
  data+reload channel pairs (5..12 for 4 pairs) to single ENDLESS-mode
  channels with write-address wrap (5..8), the same proven pattern as the
  ADAT RX ring. Channels 9/10 are now spare; 11/12 are the VCXO's.
  RP2040 has no ENDLESS mode and enough free channels, so it keeps the
  original data+reload pair (4/5); its behavior is unchanged.
- `_Static_assert`s in i2s_input.c pin the I2S RX range below
  `SOFT_VCXO_DMA_PULSE` on both platforms.

Resulting worst-case channel occupancy (RP2350): outputs 0-3, PDM 4,
I2S RX 5-8, spare 9-10, VCXO 11-12, ADAT out 13-14, ADAT in 15.

## 5. Interactions and invariants

- **Slot alignment**: unaffected by design; sys_clk trim scales every
  output clock identically and no start/stop sequencing changed.
- **USB**: clk_usb comes from pll_usb; host streaming is unaffected. In
  tracked modes the async feedback loop keeps following the device's
  consumption as before.
- **Timers / rate estimators**: the system timer ticks from clk_ref
  (XOSC), so `time_us_*` based rate measurement is independent of the
  trim; the servo loop is well-posed.
- **clk_peri consumers** (UART) and the flash clkdiv shift by the trim
  (< 1000 ppm); orders of magnitude inside tolerance.
- **`clock_get_hz(clk_sys)` stays nominal.** This is intentional; do not
  "fix" call sites to account for the trim. The trim is a closed-loop
  correction, not a reported frequency.
- **Flash blackouts**: the pulse train is pure DMA/PWM from RAM sources
  and keeps the PLL tracking through flash operations, unlike the old
  servo which simply paused.
- **Watchdog/reset**: hardware reset returns FBDIV to the SDK-programmed
  128 before `soft_vcxo_init()` runs again; no persistence.
- **Signal loss (RELOCKING)**: deliberate hold. A transient lock loss
  stops servo updates but does NOT park the VCXO; sys_clk keeps the
  last-good ppm so relock re-converges from where it was instead of
  slewing from nominal. Full stop/disarm/source-switch paths do park.
  Since 2026-11-05 every S/PDIF lock also steps the trim straight to the
  measured rate before the outputs start (`input_servo_seed()`; see
  "Lock-time seed" in current_architecture.md). S/PDIF stop-and-restart
  suspends (flash writes, reconfigurations) keep the trim too, through
  `spdif_input_suspend()`; only a switch away parks.
- **RP2350 low-clock boot fallback** (150 MHz, only if the 307.2 MHz PLL
  setup ever failed): the VCXO stays inert and tracked inputs fall back
  to untracked (nominal outputs, buffers eventually slip). The fallback
  is defensive dead code in practice; noted for completeness.

## 6. Limits

- Authority: +-1000 ppm (clamp), the full IEC 60958-3 Level II
  tolerance; RP2040 bench-verified linear to the clamp on 2026-11-05.
  Sources further off-spec behave like before: fill runs off and the
  existing overrun/underrun handling applies.
- Resolution floor: ~0.11 ppm static (park threshold); the integrator
  dithers across it, so the average is finer.
- 44.1 kHz still uses a fractional nominal divider (307.2e6/44100 is not
  integer); that is a fixed value whose residual the feed-forward cancels.
  The PIO fractional-divider jitter at 44.1 k (one sys cycle) is inherent
  to the divider and unchanged by this work.

## 7. Verification status

- Build: RP2040, RP2350, and both loopback variants compile clean;
  `scripts/check_ram_placement.py` passes all four ELFs (0 FAIL);
  `soft_vcxo_set_ppm` and `input_servo_apply` are RAM-resident.
- Bench (pre-integration): the DMA/PWM fbdiv-dither mechanism was
  validated standalone on RP2350 at 307.2 MHz across +-0.25..128 ppm with
  a frequency counter; measured tune ratio within 1% of analytic.
- Audit: an independent review confirmed topology, math, signs, gating,
  the ENDLESS conversion, per-mode DMA budgets, and cleanup completeness.
  Its one mechanism concern (DMA-timer DREQ credits accumulating while
  PULSE idles, which would collapse the pulse width) is refuted by the
  bench data: stale credits would have shrunk the measured tune ratio
  ~100x, and it matched analytic within 1%, consistent with per-channel
  DREQ credits being cleared on channel trigger. It stays on the RP2040
  bench checklist since that platform has not been measured.
- First full-firmware hardware capture (2026-07-22, RP2350, SPDIF input
  48 k, 130 s via 1.1.6 opcode 0x82, now `DIAG.CLOCK_DIAG`): PLL locked throughout, measured
  sys_clk tracks the commanded ppm 1:1, VCXO DMA/PWM loop alive, zero
  slot starvation at 192 kwords/s, input locked with zero parity errors,
  main-loop gaps under 0.5 ms. The same capture exposed the integrator
  limit cycle described in 3.1 (fixed by the retune); the mechanism
  itself performed exactly as designed.
- **Pending hardware tests**: post-retune soak confirming the DAC holds
  lock (commanded ppm should sit near the feed-forward, integrator
  within +-10 ppm, slews <= 0.5 ppm/tick); RP2040 fbdiv dithering (bench
  was RP2350); tracking soak at 44.1/48/96 k for SPDIF, ADAT slave, and
  I2S slave on both platforms; RP2350 8-channel I2S input (ENDLESS ring
  conversion); CS PWM LED conflict rejection on RP2040 slice 7.

## 8. Shaped actuator: crystal-paced, noise-shaped pulses (RP2350 prototype)

*Last updated: 2026-11-24 (section 8.8: first hardware run in the full firmware). Previously 2026-11-24 (byte-fed window 44..57, default d 50, 240-byte start and resync lead, overtaken counter). Status: bench-tested on one RP2350 board in the full firmware under USB input with a forced trim; the servo path (S/PDIF, ADAT, I2S slave) is untested on hardware.*

This section describes an alternative actuator behind the same API
(`soft_vcxo_init`, `soft_vcxo_set_ppm`, `soft_vcxo_current_ppm`), built
by the CMake option `DSPI_VCXO_SHAPED` (default OFF). It is RP2350 only.
`config.h` derives `SOFT_VCXO_SHAPED`, and an RP2040 build with the
option on keeps the PWM-paced actuator of sections 2 to 7. That
fallback lives in one `#if` in `config.h`, and CMake prints a warning for
it. With the option OFF both platforms build exactly what they built
before. Source: `soft_vcxo_shaped.c` and `soft_vcxo_pacer.pio`.
`soft_vcxo.c` compiles to nothing in a shaped build.

### 8.1 Why

The PWM-paced pulse of section 2 is timed from the system clock, so the
phase it inserts depends on where its register writes land against the
crystal. Under bus load 1 to 5 percent of such pulses miscount, and the
pulse train itself is one frequency of pulses with no noise shaping.
Pulses timed from a crystal edge at a clean delay counted exactly in
243 million bench pulses under heavy load. Driving them from a
second-order sigma-delta pushes the timing error out of the audio band.
Simulation gives about 0.075 ns rms in 20 Hz to 20 kHz at 10 ppm,
against about 1.1 ns for the PWM-paced actuator.

### 8.2 Mechanism and constants

Clock plan (required; anything else leaves the actuator inert):
XOSC 12 MHz, refdiv 1, FBDIV N = 128, VCO 1536 MHz, postdiv 5 x 1,
sys_clk 307.2 MHz. A *slot* is 5 XOSC periods (128 sys cycles). A
*pulse* is FBDIV = 129 (or 127) for exactly one slot, then 128 again. It
inserts (or removes) exactly 5 VCO cycles, 3.26 ns, but only if its two
register writes land at a clean phase against the crystal.

```
pacer:    wait 0 pin 0 / wait 1 pin 0 / mov y, x [31] / dly: jmp y-- dly
          out isr, 8 / push block [31]                    (PIO1 SM3, 6 instructions)
in pin:   GPIO 25 = CLK_GPOUT3 = XOSC, 12 mA, fast slew, input enabled
X:        pacer delay d; the FBDIV write lands d + 35 sys cycles after a rising edge
writer:   DMA 11, PIO1 RX3 -> pll_sys fbdiv_int, 32-bit, ENDLESS, high priority
feed:     DMA 12, byte ring -> PIO1 TX3, 8-bit, read ring wrap 8 KB, ENDLESS, high priority
ring:     vcxo_ring, 8192 B, 8192-aligned, .bss.aligned.vcxo_ring (RAM)
decision: one per 6 slots = 400 k/s; one byte of 127/128/129, then five 128s
u:        pulses per decision, Q24; u = ppm * 1e-6 * 128 * 6 = ppm * 7.68e-4
full scale (a pulse every decision): 1e6 / 768 = 1302.08 ppm
clamp:    +-1000 ppm (SOFT_VCXO_MAX_PPM unchanged), 0.768 of full scale
park:     |ppm| < 0.001 (in practice only the servo's explicit 0)
```

The pacer waits for a rising edge, delays, takes one byte from its OSR
(autopull, threshold 8) into the ISR and pushes it. `out isr, 8` writes
the byte zero-extended into the ISR whatever the shift direction, so
FBDIV receives exactly the ring byte. The next `wait` starts d + 67
cycles after the detected edge, which by the cycle count catches edge 5
of the slot for d from about 37 to 61. Outside that range a slot is 4 or 6
periods long, and a 4-period excursion is in the range that resets the
chip. The firmware accepts d from 40 to 59 (`SOFT_VCXO_D_MIN` and
`SOFT_VCXO_D_MAX`). Measured clean windows: 44..57 for this byte-fed
path (quiet and heavy load; 120 M pulses, 0 miscounts at d = 50), and
45..58 for the earlier 32-bit bench variant (same across 8 boots and a
cold power cycle). The default is 50. The clean window was measured at 12 mA,
fast slew on GPIO 25; the constant depends on the pad settings.

The modulator is second order with a three-level quantizer:
`y = sign(i2) if |i2| > 0.5 else 0`, `i1 += u - y`, `i2 += i1 - y`, all
in Q24. The integrators saturate at +-8 and +-16 pulses (`SSAT` 28 and
29 bits). A host simulation of the exact arithmetic over 200,000
decisions per point shows the peaks at 1000 ppm are 1.65 and 2.75
pulses, so the clamps never engage within +-1000 ppm; at 1250 ppm they
are 1.8 and 9.9; at full scale the integrators sit on the clamps and the
output saturates to a pulse every decision. The long-run mean equals
`u` to the quantization of the window in every case. Two decisions are
written as three aligned words, `0x808080yy`, `0x80yy8080` and
`0x80808080`, so a write never splits a word and never wraps inside one.

### 8.3 Hazards and how the code meets them

1. **A short FBDIV = 127 excursion resets the chip** (about 77 to 105
   sys cycles). Every slot is 128 sys cycles by PIO timing, so no ring
   content can shorten one. A starved TX FIFO can: the SM stalls before a
   pulse write and resumes off phase, then ends the pulse on the next
   crystal edge. The feed is therefore ENDLESS with read-address wrap,
   at high priority, so it can never run dry. If the CPU is late the
   DMA replays an old lap of valid bytes. The SM is enabled only after
   the feed has filled the TX FIFO (bounded 100 us wait; on timeout
   nothing has reached FBDIV and the actuator goes inert). The ring
   starts with 240 bytes of 128, so any ragged first slot while GPOUT3
   starts only rewrites 128.
2. **Writing 0 or garbage to `fbdiv_int` kills sys_clk.** Every ring byte
   is 127, 128 or 129 at all times: init and every start and stop fill
   it with 128 before any DMA reads it, and the writer only stores
   encoded decisions. The pacer's X is loaded through the TX FIFO with
   autopull off, then the OSR is emptied with `out null, 32` and the
   FIFOs cleared before autopull is enabled, so the delay value can
   never be the first byte written. The CPU writes only 128.
3. **Two aborts are not simultaneous.** Stopping fills the ring with
   128s first, waits until only 128s can be in flight, then stops the SM,
   then clears EN on and aborts both channels (the RP2350-E5 order),
   then writes 128 from the CPU. When the channels stop, both carry 128,
   so their order does not matter.
4. **RP2350 TRANS_COUNT is 28 bits** (under two minutes at 2.4 M/s).
   Both channels use ENDLESS mode.
5. **Flash blackouts** park both cores with IRQs masked while DMA keeps
   running. The ring is in RAM (`.bss.aligned`), and
   `scripts/check_ram_placement.py` requires `vcxo_ring` and `vx_fill`
   in RAM in any ELF that defines `vcxo_ring` (and drops the PWM
   actuator's `fbdivs` words there). During a blackout the last lap
   replays: the trim holds near its value (a lap is 1365 decisions, so
   its mean is within a few pulses of `u`, about 2 ppm at worst) with a
   tone at the lap rate (293 Hz); audio is muted for flash writes anyway.
6. **ACCESSCTRL.** Init grants the DMA master access to PLL_SYS, as the
   PWM actuator does.

### 8.4 Start and park

`soft_vcxo_set_ppm()` only clamps and records the command. A state
machine in `soft_vcxo_service()` does the hardware work:

- **PARKED** (after init): SM disabled, both channels idle, GPOUT3
  disabled, FBDIV exactly 128, no DMA traffic. A non-park command starts
  the pacer.
- **Start** (one call, about 10 to 20 us): fill the ring with 128, reset
  the integrators, `pio_sm_init` with autopull off, load X, empty the
  OSR, enable autopull, clear the FIFOs, start the writer, start the
  feed, wait for a full TX FIFO, fill up to 1 ms of decisions, enable
  GPOUT3, enable the SM. State RUNNING.
- **RUNNING**: refill every 200 us or later (below). A park command (or,
  in diag builds, a new delay d) begins the stop.
- **Stop, step 1**: fill the ring with 128 and note the time. State
  DRAINING. Nothing blocks.
- **Stop, step 2** (the first call 50 us or more later, by which time
  120 slots of 128 have passed against about 10 bytes in flight): disable
  the SM, clear EN and abort the feed and the writer, write 128 to
  `fbdiv_int`, clear the FIFOs, disable GPOUT3. State PARKED. A new
  command starts it again on a later call.

A park therefore takes one or two delivery passes, and the audio path is
never blocked for more than the start's few microseconds.

### 8.5 Filler cadence and ring size

The ring holds 3.41 ms of slots. `soft_vcxo_service()` runs at the end of
every `pipeline_service_producer()` call in `main.c`: the four main-loop
delivery points, and every blocking spin (fades, flash-write settle),
so the same cadence that feeds the output rings feeds this one, in all
input modes. Measured worst delivery gaps in soaks are 0.4 to 0.6 ms
(current_architecture.md, "Delivery points"), against a lead kept
between about 3.1 and 3.4 ms. A refill runs when 200 us or more have
passed since the last one (480 bytes, 40 two-decision steps) and is
capped at 1 ms of slots per call. Each refill reads the feed's read
address. If the time since the last refill is longer than the lead that
refill left, or the measured lead grew, the DMA has replayed stale
bytes: the replay is counted and the writer restarts 240 bytes (100 us)
ahead of the DMA. After each fill loop the feed's read address is read
again; if the DMA passed the first byte that pass wrote (a slow or
preempted fill), `overtaken` counts it (the replay counter in release
builds) and the next call's replay logic resyncs. This is detection only. Replays should happen only across flash blackouts and similar
long stalls.

Cost (RP2350, release build, from the generated code): the two-decision
loop is 38 instructions, about 20 cycles per decision, so about
8 M cycles/s while running. With the per-call overheads the estimate is
about 3 percent of core 0 at 307.2 MHz while a trim runs, and a few
cycles per delivery while parked. Diag builds measure the real figure
(`fill_cycles` over `decisions`).

### 8.6 Resources

| Resource | Shaped actuator (RP2350) |
|---|---|
| PIO | PIO1 SM3, 6 instructions (`pio_add_program` at init, before PDM and ADAT load theirs) |
| DMA | CH11 writer, CH12 feed (the PWM actuator's PULSE and PACE numbers) |
| GPIO | 25 as CLK_GPOUT3; the heartbeat LED is compiled out |
| DMA timer, PWM slice 8 | unused; the Control Surfaces slice guard stays and is a no-op on RP2350 |
| RAM | 8,192 B ring (`.bss_aligned`), a few dozen bytes of statics, 692 B of RAM code in a release build (`.data` +416 B net; free heap 137,520 to 128,804 B) |

`config.h` refuses `DSPI_VCXO_SHAPED` together with `DSPI_WIRECAP`
(the rig uses GPIO 25 and PIO1 SM3) or `DSPI_PLATFORM_PINS` (no platform
pin map reserves GPIO 25). In the RP2350 release configuration nothing
else can reach GPIO 25 (`is_valid_gpio_pin()` rejects 23 to 25 for every
user pin) or PIO1 SM3 (PDM is SM0, ADAT TX SM1, ADAT RX SM2, all claimed
by number).

### 8.7 Bench hooks (`DSPI_VCXO_SHAPED` and `DSPI_CLOCK_DIAG`)

Three DIAG commands, absent from every other build:

| Cmd | Name | Dir | Payload |
|---|---|---|---|
| 0x60 | `VCXO_STATUS` | GET 104 B | `SoftVcxoShapedDiag` (below) |
| 0x61 | `VCXO_FORCE` | GET / SET 5 B | u8 enable, f32 ppm (+-1000). While enabled the modulator runs this trim and ignores the servo; 0 ppm parks |
| 0x62 | `VCXO_DELAY` | GET / SET 1 B | u8 d, 40 to 59 (`VC_ERR_VALUE` outside), default 50. A running pacer parks and restarts on it |

`SoftVcxoShapedDiag`, packed little-endian (`soft_vcxo.h`, static-asserted
to 104 bytes; `tools/vcxo_shaped_bench.py` decodes it): u8 version (1),
state (0 inert, 1 parked, 2 running, 3 draining), d, forced; f32
force_ppm, servo_ppm, run_ppm; i32 u_q24; u32 starts, stops, replays,
fill_calls; u64 decisions; i64 net_pulses; u64 fill_cycles (core 0 DWT);
u32 lead_min_bytes, fill_gap_max_us; i32 i1_peak, i2_peak (Q24); u32
sat_hits; u16 fbdiv_live; u8 sm_enabled, dma_busy (bit 0 writer, bit 1
feed); u32 ring_bytes; u64 fc0_sum; u32 fc0_gates; u32 overtaken. `lead_min_bytes`,
`fill_gap_max_us` and the integrator peaks restart on every read; the
rest are cumulative, so the host differences two reads.

`fc0_sum` and `fc0_gates` are the clock-diag fc0 results of clk_sys
against the crystal, summed since boot (1/32 kHz units, about 30 gates
per second, 0.1 ppm per gate). Their ratio over a window of N seconds
averages about 30 N gates. `net_pulses` over `decisions` gives the
trim the modulator actually emitted, so the measured trim can be
compared with both the command and the pulse count.

`tools/vcxo_shaped_bench.py` (`status`, `force`, `delay`, `measure`,
`sweep`, `linearity`) drives these over USB.

### 8.8 Hardware results in the full firmware (2026-11-24)

One RP2350 board, `build-rp2350-vcxoshaped` (clock diag on), USB input at
48 kHz, trims forced with `DIAG.VCXO_FORCE`, measured by averaging fc0
gates (about 15.6 gates per second). Raw logs and scripts:
`bench_results/vcxo_shaped_fw/`.

- **fc0 resolution.** Eight parked 10 s windows: standard deviation
  0.005 ppm; at +10 ppm, 0.009 ppm. Parked readings carry a constant
  +0.05 to +0.066 ppm fc0 offset (FBDIV is exactly 128 then), so every
  trim is measured against interleaved parked baselines.
- **Bring-up.** 400 random force on/off cycles (0.02 to 1000 ppm, 0 to
  30 ms dwell): 401 starts, 401 stops, FBDIV 128 and SM and DMA idle
  after every check, no reset. 446 starts and stops over the session.
- **Realised against commanded trim, d = 50.** From ±0.02 to ±1000 ppm
  every point is within 0.015 ppm of the command (gain 1.00000 at
  ±1000, ±300, ±600 ppm). At ±0.02 and ±0.05 ppm the error is the
  resolution limit.
- **Delay window.** Idle and under the heavy profile with USB
  streaming: d = 40, 41 and 42 miscount (net 0.1 to 1.2 ppm at
  ±1000 ppm, worst at 42), d = 43 to 59 show no net miscount above the
  noise. fc0 sees only the net of miscounts. At ±1000 ppm (307 k pulses/s,
  one sign) its 0.015 ppm floor is about 25 net miscounts per second, a
  fraction of about 8e-5; at ±100 ppm about 1e-3. Miscounts that cancel
  in sign, or that are rarer than that, are not detectable this way.
- **Load.** The latency_bench 4-way profile plus crossfeed, spatializer,
  subharmonic, psychoacoustic bass, tube, leveller, loudness and the
  limiter on all outputs (core 0 54 %, core 1 44 %), USB streaming. No
  ring underrun, refused block, USB overrun, USB gap or host underflow
  appeared in any of 20 A/B windows of 20 to 25 s (6 parked, 14 running
at ±10, ±100 and ±1000 ppm) across the default and heavy configurations.
- **Filler.** Across about 60 minutes of running (1.44 G decisions):
  0 overtaken, 0 saturation hits, 0 replays outside flash writes. The
  worst gap between refills under load was 939 us and the least unread
  ring content 5,874 B (about 2,250 B consumed). A 4 KB ring would have
  held with about 0.7 ms of margin; 2 KB would not. Integrator peaks
  1.0 and 1.5 at ±10 ppm, 1.65 and 2.75 at ±1000 ppm, as simulated.
  Fill cost measured by DWT in this diag build: 28.2 cycles per
  decision, 3.67 % of core 0 (the estimate was about 20 and 3 %).
- **Flash blackouts.** Nine flash writes at +100 ppm (preset save,
  preset name, device name): filler gaps 43 to 123 ms, 1 or 2 replays
  each, state stayed RUNNING, no reset, trim correct afterwards
  (+100.015 ppm).
- **Soak.** 10 minutes at +10 ppm: mean +9.999 ppm (sd 0.003 over
  1-minute windows); at -10 ppm: -10.008 (sd 0.003). No replay,
  overtaken, start or stop. One macOS coreaudiod IO stop (EndWriting
  error, about 100 ms without packets) in each soak; the same event
  happened in a 10-minute parked control run, so it is host-side.
- **Not tested.** No S/PDIF source is connected (receiver stays
  ACQUIRING), so the servo path is untested on hardware; ADAT and I2S
  slave likewise. A release (non-diag) shaped build was not flashed.

