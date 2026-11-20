# rta_test: host harness for the spectrum analyser

Compiles the analyser kernel (`firmware/DSPi/rta_fft.c`) natively with clang
in both sample formats (float for RP2350, Q15 for RP2040), drives it from
Python through ctypes, and checks it against numpy for orders 8 to 10 (256 to
1024 points) at 44.1, 48 and 96 kHz. A second script compiles the real engine
(`rta.c`) against stubs and exercises the protocol end to end. Design and
acceptance criteria: `Documentation/Features/spectrum_analyser_spec.md`,
section 4.2.

```
./run.sh
```

Needs `clang` and `python3` with numpy and scipy. `test_rta.py` builds the shared
libraries into `build/`; `test_engine.py` builds two throwaway executables.
All print PASS or FAIL and exit non-zero on failure.

## What test_rta.py checks

One row per (format, order, rate):

| column | meaning | tolerance |
|---|---|---|
| bins | worst bin level error against a numpy Blackman-Harris FFT | 0.35 dB float, 0.75 dB Q15 |
| fs-bnd | full-scale sine level error in its band | 0.5 dB |
| -60bnd | -60 dBFS sine level error in its band | 0.2 dB float; 1.25 dB Q15 |
| edgepr | sine on a band edge, error of the two-band sum | 0.5 dB |
| edge1 | same tone, the better single band | never below -3.05 dB |
| pinkfl | pink noise spread across bands of 12 bins or more | 1.0 dB |
| pinkor | pink band powers against the oracle | same as bins |
| floor | loudest bin more than 3 bins from a -100 dBFS sine | below -120 dBFS float, -80 dBFS Q15 |
| dyn-rng | loudest bin more than 3 bins from a full-scale sine | reported, not graded |

Before the rows it also grades bins 0 to 5 and the top 5 bins on every bin
against the oracle, and checks that finish chunk sizes 1, 3 and 4 give the same
bytes. Those are the bins whose window taps wrap through slot 0 or the Nyquist
mirror, which the finish handles on a separate path from the interior.

Before the rows it asserts that every band table in `rta_tables.h` matches
`scripts/gen_rta_tables.py`, which both this harness and the firmware treat as
the single source of truth for the band layout. Every dB comparison against a
level byte carries half a wire step (0.25 dB) of quantisation on top of the
stated tolerance.

## Measured figures worth knowing

- **Q15 dynamic range is 78.5 dB**, not the 80 dB the first spec draft asked
  for. `RtaCaps.dynamic_range_db` reports 78. The -100 dBFS floor test passes
  in Q15 only because a 0.33 LSB tone quantises to silence; `dyn-rng` is the
  honest number.
- **The FFT window is 4-term Blackman-Harris.** Hann's slow sidelobe decay let
  loud bass leak into the first FFT bands above the bass bank as a fixed hump.
  The wider main lobe reads a centred tone 3.02 dB low in a one-bin band.
- **A tone on a band edge reads up to 3.01 dB low in each band.** The window main
  lobe splits across the two bands, so the pair sums correctly but no single
  band can. A client that wants a tone's level should sum the pair.
- **A -60 dBFS sine reads up to 1.24 dB high in Q15 at order 10**, the
  largest size, 0.01 dB inside tolerance. The band sums the per-bin floor across
  all its bins, and the floor is closer at higher orders because every stage
  halves. Order 9, the RP2040 default, stays within 1.08 dB.
- **Two real samples packed into one complex slot give sqrt(2) full scale**,
  so the Q15 path prescales by 1/sqrt(2) and the split pass restores it. That
  is why the Q15 range is 3 dB short of a plain 16-bit floor.

## What test_engine.py checks

The engine compiled with stubbed hardware and time, in both formats: config
version and order rejection, caps, all-channel rotation cadence on a tone, the
bin frame layout and seq head/tail, recovery when a channel goes dead
mid-fill, restart, a switch to the 1024-point ceiling, and auto-off after 5 s without a
read.


## Continuous bass and V3 integration (2026-09-12)

`test_bass.py` compiles the actual streaming kernel in both formats and checks
all 14 centres at 44.1/48/96 kHz against a separate float64 scipy signal-chain
oracle. It also checks -60/-70 dBFS, full-scale and hot inputs, DC rejection,
alias rejection, band selectivity (at least 40 dB one octave from a tone and
60 dB two octaves away), irregular packet sizes, channel isolation and step response.
The response thresholds exclude user averaging and FFT publication scheduling.

`test_engine.py` additionally runs undefined-behavior sanitization and checks
V2 rejection/V3 layout, identical continuous histories for every selected
channel during FFT rotation, input/output tap selection, sample immutability,
liveness resets, averaging changes, RESET_AVG, rate restart and stop/start.
No host test establishes an on-device CPU percentage or physical output timing.

Generate coefficients with `python3 scripts/gen_rta_bass.py`; generate the
37-band FFT tables with `python3 scripts/gen_rta_tables.py --out
firmware/DSPi/rta_tables.h`. Each bass band is a sixth-order Butterworth
bandpass calibrated for tones at its exact centre.
