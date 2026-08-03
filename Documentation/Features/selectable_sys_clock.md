# Selectable System Clock and Core Voltage

*Written: 2026-08-03. Status: implemented on release/v1.1.5, hardware-untested.*

## Overview

The firmware historically ran a fixed 307.2 MHz system clock at 1.15 V, chosen
because the 48 kHz audio family gets integer PIO dividers there. This feature
makes the system clock selectable between three modes on **both platforms**,
with a matching core-voltage selection, persisted in the flash directory and
applied both at boot and live at runtime:

| Mode | sys_clk | PLL (VCO / pd1 / pd2) | Default vreg | 48k-family dividers |
|------|---------|----------------------|--------------|---------------------|
| 0 | 307.2 MHz | 1536 MHz / 5 / 1 | `VREG_VOLTAGE_1_15` | Integer |
| 1 | 384 MHz   | 1536 MHz / 4 / 1 | `VREG_VOLTAGE_1_20` | Exact 8.8 fractional |
| 2 | 480 MHz   | 1440 MHz / 3 / 1 | `VREG_VOLTAGE_1_30` | Exact 8.8 fractional |

The higher modes exist to buy DSP headroom (more EQ bands, upmixer, leveller
etc. per block). They are overclocks: 480 MHz especially is well beyond the
RP2040's rating and beyond many individual chips at any voltage. The design
therefore treats "the configured clock does not run on this chip" as a
first-class case (see Crash fallback).

Module: `firmware/DSPi/sys_clock.c` / `sys_clock.h`.
Divider derivation: `firmware/DSPi/audio_clock_div.h`.

## Divider architecture: the coherent family base

### The problem

The PIO/GPOUT clock dividers are 16.8/24.8 fixed point (fraction in 1/256).
At 307.2 MHz the 44.1k family is *already* fractional (no exact representation
exists), but every consumer's independent rounding happened to land on the same
relative error (-1.95 ppm), so all output slots ran at identical rates. At 384
and 480 MHz that luck ends: independently rounded dividers disagree per output
(at 384 MHz / 44.1 kHz: SPDIF -59.4 ppm vs I2S -0.5 ppm vs PDM +55.5 ppm),
which would make output slots drift apart continuously; a violation of the
inviolable inter-slot alignment constraint.

### The fix

`audio_base_divider_16_8(fs)` computes ONE base divider per rate family:
the 256×Fs divider in 16.8, rounded to nearest:

```
base = (clock_get_hz(clk_sys) + fs/2) / fs        // value is div * 256
```

Every audio output clock derives from it by exact power-of-two scaling:

| Consumer | Derivation | Applied in |
|----------|-----------|------------|
| SPDIF TX (PIO clk 256×Fs) | base | `audio_spdif.c update_pio_frequency()` |
| ADAT out (PIO clk 256×Fs) | base | `adat_output.c` (`adat_nom_div`) |
| I2S TX (PIO clk 128×Fs, 24.8) | 2 × base | `audio_i2s_multi.c i2s_compute_divider()` |
| I2S RX clock-master | 2 × base | `i2s_input.c rx_master_divider_24_8()` |
| MCK 256× (GPOUTn, 24.8) | base | `audio_i2s_multi.c mck_update_frequency()` |
| MCK 128× | 2 × base | same |
| PDM (PIO clk 256×Fs) | base (int+frac API) | `pdm_generator.c pdm_update_clock()` |
| SPDIF servo sentinel cache | base / 2×base | `spdif_input.c servo_cache_base_dividers()` |

Because every slot's divider is an exact multiple of one rounded value, every
slot shares a single ppm error and inter-slot ratios are exact by construction,
at any sys clock. The servo trim paths (`input_servo.c`, the I2S-slave servo in
`i2s_input.c`) still compute float dividers from *measured* input rates by
design; they already derive I2S = 2× SPDIF, ADAT = SPDIF, and (since this work)
MCK = 1×/2× the trimmed SPDIF divider, so trims move all slots coherently too.

Side effect fixed: the old MCK computation truncated where I2S ceiled, leaving
MCK ~150 ppm off an exact multiple of BCK at 44.1 kHz. Both the nominal path
and the servo path now derive MCK from the same value as BCK.

### Resulting absolute accuracy (free-running outputs)

| Family | 307.2 MHz | 384 MHz | 480 MHz |
|--------|-----------|---------|---------|
| 48k / 96k / 192k | exact | exact | exact |
| 44.1k | -1.95 ppm | +55.5 ppm | +32.5 ppm |
| 88.2k | -1.95 ppm | -59.4 ppm | +32.5 ppm |

These offsets matter only when nothing disciplines the rate (signal generator /
RAW use with no USB stream): USB feedback slaves the host to the device, and
the SPDIF/ADAT/I2S-slave input servos track measured input rates. All values
are far inside SPDIF Level II (±1000 ppm) but the 44.1k family at 384 MHz
(±55.5 ppm) exceeds AES3 Grade 1 / Level I (±50 ppm); do not recommend 384 MHz
as a free-running 44.1k master where Grade 1 matters.

### Jitter caveat

A fractional divider dithers between adjacent integer divisions: deterministic
edge jitter of up to one sys period (~2.6 ns at 384, ~2.1 ns at 480) on BCK,
MCK, and SPDIF symbol edges. SPDIF receivers reclock and PLL-equipped DACs
don't care; a jitter-sensitive MCK-synchronous DAC may measure worse in the
overclocked modes than at 307.2 MHz with its integer 48k dividers. Inherent to
the feature.

## Input-side clocking

**SPDIF RX** (`pico_spdif_rx`): the decode thresholds and the per-rate PIO
programs are calibrated to a 122.88 MHz PIO clock. The former compile-time
`SPDIF_RX_SYS_CLK_FREQ` (307.2 MHz) is gone; the SM divider is computed at
program init from `clock_get_hz(clk_sys)` targeting `SPDIF_RX_PIO_CLK_FREQ`
exactly: 2.5 / 3.125 / 3.90625 in the three modes, all exact in 16.8. The
capture (debug) PIO variant got the same treatment.

**ADAT RX** (`adat_input.c`): the NRZI decoder counts sys cycles per wire bit
cell and requires an odd cell count (its poll loop steps 2 cycles). Rather than
re-deriving cells per mode (384 MHz / 44.1 kHz rounds to an even 34, a lock
failure), the RX SM runs at an exact-fraction divider that pins its *effective*
clock at 307.2 MHz in every mode (`adat_rx_div_256()`: 1.0 / 1.25 / 1.5625).
Cells are computed against the effective clock and stay 27 (44.1k) / 25 (48k)
with margins identical to the original design everywhere. Divider jitter is
≤1 sys cycle against an 8-effective-cycle poll granularity, and the decoder
re-anchors at every transition. `tools/adat_rx_test/adat_rx_bitdiff.c` accepts
the sys clock as an argv for offline re-validation.

## Persistence

Two bytes appended to the device-global `FlashOutputConfig` inside
`PresetDirectory` (directory version 16 → 17, frozen `FlashOutputConfig_v16` /
`PresetDirectory_v16` prefix structs, standard migration arm seeding
mode 0 / vreg 0xFF):

- `sys_clock_mode`: 0..2
- `sys_clock_vreg`: raw platform vreg enum, `0xFF` = "use the mode's default"

The setting is device-global only: preset save/load never touches it
(`io_config_from_live()` snapshots and restores the bytes around its memset,
because its destination aliases the directory cache). `dir_load_cache()`
sanitizes the bytes on **every** load path (an invalid mode becomes 0; a
voltage below the mode's default or above 1.30 V becomes 0xFF), covering the
zero-fill that pre-V17 migration arms leave behind. Accessors:
`preset_set_sys_clock()` / `preset_get_sys_clock()`; the getter is safe before
the clock is configured (XIP reads plus at most one migration flush).

## Vendor interface

**`REQ_SET_SYS_CLOCK` (0x40, OUT, 2 bytes)**: `{mode, vreg_sel}`.
Validation before staging: `mode < 3`, and `vreg_sel` either 0xFF or a vreg
enum **at or above the mode's default** and at most the platform ceiling:
`VREG_VOLTAGE_1_50` on RP2350 (the firmware disables the POWMAN voltage limit
automatically when a value above 1.30 V is applied), `VREG_VOLTAGE_1_30` on
RP2040 (regulator hardware maximum). Anything else STALLs. Undervolting is
rejected by design: the voltage knob exists to trade voltage up for stability
on marginal silicon, and a below-default value is a guaranteed-unstable
persisted setting. Note that 1.35 to 1.50 V sit above Raspberry Pi's
sanctioned operating range: expect substantially more heat (the on-chip LDO
dissipation grows with both voltage and clock), and treat them as bench tools
rather than shipping configurations. Staged via
`sys_clock_req_mode/vreg` + `sys_clock_set_pending` (declared in `sys_clock.h`)
and applied from the main loop. Allowed on all transports.

**`REQ_GET_SYS_CLOCK` (0x41, IN, 8 bytes)**:
`[0]` active mode, `[1]` stored mode, `[2]` stored vreg_sel (0xFF preserved),
`[3]` live vreg enum, `[4]` fallback-active flag, `[5..7]` zero. Active and
stored differ exactly when a fallback boot is in force. Live sys clock in Hz
and vreg in mV also remain readable via `REQ_GET_STATUS` sub-indices 13/14.

## Runtime switch sequence

Handled in the main loop's deferred-flash section (`sys_clock_set_pending`):

**Voltage-only** (requested mode == active mode): flash bracket around the
directory write only, then `sys_clock_apply_vreg_only()`; a glitch-free vreg
step with audio running. No PLL relock, no teardown.

**Full switch:**
1. `prepare_flash_write_operation()`: mute, drain, stop SPDIF RX and I2S input.
2. RP2350: `adat_input_stop()` if ADAT is the active source (its SM divider and
   cell timing are clock-derived).
3. `preset_set_sys_clock()`: persist **before** the PLL step, so a crash
   mid-switch reboots into the stored mode with the breadcrumb armed, which is
   exactly the boot-fallback path.
4. `sys_clock_apply()`: arms the breadcrumb, sequences vreg/PLL (voltage rises
   before the frequency step and falls after it).
5. Nominal divider rebuild at the new clock, mirroring `perform_rate_change`'s
   divider block: `audio_i2s_update_all_frequencies()`,
   `restore_nominal_spdif_dividers()`, MCK update, `pdm_update_clock()`,
   `adat_output_on_rate_change()` (which now refreshes and applies
   `adat_nom_div` immediately, because the SPDIF-input resync can be
   unboundedly late), ADAT input restart.
6. `i2c_ctrl_reclock()`: the I2C target IP counts timing in clk_sys cycles.
   UART needs nothing (clk_peri runs from the 48 MHz USB PLL).
7. `complete_flash_write_operation_full()`: restarts SPDIF RX / I2S input
   (their dividers are set at program init) and restores outputs. Under USB
   input this runs `complete_pipeline_reset()`; under SPDIF input it returns
   early with outputs muted and the lock prefill's `enable_outputs_in_sync()`
   performs the synchronized all-slot restart, so no audio ever plays before
   all slots are re-aligned at the new clock.

USB stays enumerated throughout (the USB PLL is untouched); the isochronous
stream sees a brief muted gap and the feedback loop is reset.

## Crash fallback (breadcrumb + latch)

A persisted mode this particular chip cannot run would otherwise soft-brick the
device (it never enumerates, so no vendor command can revert it). Protection:

- **Watchdog first.** `watchdog_enable(8000, 1)` moved *before* `core0_init()`
  in `main()`, so a hang anywhere in boot still reboots.
- **Breadcrumb.** `watchdog_hw->scratch[1]` (scratch 4..7 are SDK reboot
  vectors; scratch 2..3 carry bootrom reboot/usb-boot parameters; scratch
  clears on power-on reset and survives warm resets). Armed with
  `0xC1C0DE00 | mode` whenever the boot config is non-default (mode != 0 OR a
  custom voltage; an undervolt crash-loops like an overclock) and on every
  runtime apply.
- **Confirm window.** `sys_clock_confirm_tick()` runs every main-loop iteration
  and clears the breadcrumb only after **5 s** of fed-watchdog uptime since it
  was armed. Overclock failures surface under enumeration and DSP load, not on
  the first loop iteration.
- **Fallback + latch.** A watchdog-caused reboot with the breadcrumb armed
  boots at mode 0 / default vreg, sets the fallback flag (GET byte 4), writes a
  LATCH value (`0xC1C0FA11`) into scratch[1], and marks itself confirmed so the
  tick never clears the latch. Every subsequent warm reboot stays on the safe
  default; a power cycle (scratch cleared by POR) or a new `REQ_SET_SYS_CLOCK`
  (apply overwrites the latch with a fresh breadcrumb) retries. The persisted
  setting itself is never rewritten, so a marginal-voltage user can bump the
  voltage and retry without reconfiguring.

Residual risk, accepted: a chip that survives the 5 s window but crashes later
alternates between the stored mode and fallback (one 8 s watchdog dwell per
attempt); a chip that crashes only under rare peak load may stay on the stored
mode. BOOTSEL remains the last-resort recovery.

## Hardware validation TODO (feature is HW-untested)

- **Flash timing.** The QSPI divider is per-mode on RP2350
  (`flash_clkdiv.c`): 6 at 307.2/384 MHz (51.2/64 MHz flash) and 8 at 480 MHz
  (60 MHz flash). Bench status 2026-08-03: 384 MHz ran clean; 480 MHz with
  div 6 (80 MHz flash) crashed immediately and the breadcrumb fallback
  recovered the device as designed. The div-8 change is the discriminating
  test between flash timing and core silicon limits; if 480 still crashes
  with div 8 the chip itself cannot run 480 MHz at 1.30 V. The sys-clock
  switch applies the divider for the faster of old/new clocks before the PLL
  moves; the divider is RAM-cached because the flash-op wrappers run with XIP
  down. RP2040 keeps boot2's fixed div 6 (80 MHz at 480). If div 8 proves to
  be the fix, an RXDELAY bump is the alternative should the cold-path cost of
  div 8 ever matter.
- Overclock stability screening per platform (384 and especially 480 MHz on
  RP2040 will not run on all silicon even at 1.30 V).
- SPDIF RX lock and ADAT RX lock at both rates in the overclocked modes
  (fractional-divider sampling jitter margins).
- DAC behaviour on fractional BCK/MCK (jitter-sensitive MCK-synchronous DACs).
- 44.1k interop at +55.5 ppm (384 MHz) with external gear that enforces
  Level I tolerance.
- The 5 s confirm window against real enumeration times on slow hosts.

## RAM cost

6 bytes of statics in `sys_clock.c` plus 2 bytes inside the existing directory
cache. The mode table is const (flash). No RAM-pinned code was added; the
sys_clock functions are cold-path XIP.
