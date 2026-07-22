# Soft VCXO: Fractional-N System PLL Trim for Input Clock Tracking

*Last updated: 2026-07-22*
*Status: implemented on branch `spdif_rx_overhaul`; RP2350 mechanism bench-proven standalone; full-firmware behavior HW-untested; RP2040 fbdiv dithering HW-untested.*

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

sys_clk = 307.2 MHz: XOSC 12 MHz ref, refdiv 1, FBDIV 128 (VCO 1536 MHz),
postdiv 5 x 1. `soft_vcxo_init()` verifies `clock_get_hz(clk_sys) ==
307200000` and stays inert otherwise (covers the RP2350 150 MHz boot
fallback).

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

`SOFT_VCXO_HZ_PER_PPM = 262.144`. A standalone RP2350 bench test measured
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

1. Clamp to +-`SOFT_VCXO_MAX_PPM` (300).
2. `freq = |ppm| * 262.144`; below 30 Hz, park (see above).
3. Set `fbdivs[0]` = 129 (ppm > 0) or 127 (ppm < 0).
4. Program the PWM pacer: pick the smallest power-of-two clkdiv (1..128)
   whose wrap fits 16 bits, then `wrap = round(307.2e6 / clkdiv / freq)`.
   Reachable rate span: ~18 Hz to well past the 78.6 kHz needed at
   300 ppm (duty at 300 ppm is ~4%, so pulses never merge).

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

Fill terms (only when `fill_slot >= 0`), same plant and gains as before,
signs flipped to ppm convention (overfull consumer = outputs too slow =
positive ppm = speed sys_clk up):

- Deadbanded P: `|fill_error| > 2` buffers, KP = 5e-4 (as a dimensionless
  rate trim; x1e6 = ppm).
- Centering integrator: always active, KI = 2e-7/tick/buffer, clamp
  +-5e-5 (+-50 ppm). The old +-2-LSB (+-320 ppm) clamp existed to cancel
  divider rounding; the feed-forward now does that exactly, so the clamp
  shrank to bound windup from transient bogus fill reads.

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
  (< 300 ppm); orders of magnitude inside tolerance.
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
- **RP2350 low-clock boot fallback** (150 MHz, only if the 307.2 MHz PLL
  setup ever failed): the VCXO stays inert and tracked inputs fall back
  to untracked (nominal outputs, buffers eventually slip). The fallback
  is defensive dead code in practice; noted for completeness.

## 6. Limits

- Authority: +-300 ppm (clamp). Sources further off-spec behave like
  before: fill runs off and the existing overrun/underrun handling
  applies. The mechanism itself could reach ~1000 ppm by raising the
  clamp and max PWM rate if ever needed.
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
- **Pending hardware tests**: RP2040 fbdiv dithering (bench was RP2350);
  tracking soak at 44.1/48/96 k for SPDIF, ADAT slave, and I2S slave on
  both platforms; RP2350 8-channel I2S input (ENDLESS ring conversion);
  CS PWM LED conflict rejection on RP2040 slice 7.
