# ac3_test

Host verification and device bench for the ported liba52 AC-3 decoder in
`firmware/DSPi/ac3/` (Phase 1 of `Documentation/Features/ac3_dts_decoding_plan.md`).

## Host verification

```
tools/ac3_test/run.sh [workdir]
```

Builds `ac3_test` from the firmware sources with `-ffp-contract=off` (the
device build uses the same flag, so both round identically), checks the
generated IMDCT tables against a recomputation, generates AC-3 vectors with
ffmpeg's encoder, decodes them with both decoders and compares them with
`compare.py`. Tones agree with ffmpeg to 75 dB and better. Noisy sources only
agree to 25 to 45 dB because each decoder substitutes its own random dither
for zero-bit mantissas, so those vectors use a loose threshold and are checked
bit-exact against upstream instead:

```
git clone https://code.videolan.org/videolan/liba52.git /tmp/liba52
tools/ac3_test/run_stock.sh /tmp/liba52 [workdir]
```

`run_stock.sh` compiles the unmodified liba52 and compares every output float
bit for bit. Needs the vectors from a previous `run.sh` in the same workdir.

Requirements: ffmpeg, a C compiler, python3 (no numpy).

## Device bench

Configure a bench build (RP2350 only):

```
cmake -S firmware -B build-rp2350-ac3bench -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350-arm-s -DCMAKE_BUILD_TYPE=Release -DDSPI_AC3_BENCH=ON -DDSPI_AC3_RAM_HOT=ON
cmake --build build-rp2350-ac3bench
```

`DSPI_AC3_RAM_HOT=OFF` builds the XIP-only variant. `DSPI_AC3_OPT` sets the
decoder's optimisation level (default `-O2`). The build embeds
`vectors/bench_51_640k.ac3` (eight 5.1 frames at 640 kbps, the worst-case
bitrate) and the checksum from its `.sum` sidecar.

Flash the UF2, then:

```
tools/ac3_test/bench_poll.py --passes 50
```

The run blocks the main loop for its duration (about 0.5 s at 50 passes) and
reports microseconds per frame, the header/block split, the share of one core
that real-time decode would take at 48 kHz, and whether the device decode
matched the host checksum. Needs pyusb, as `tools/dspi_poke` does.

## Files

- `ac3_test.c`: decode, table check and host timing.
- `compare.py`, `strip_channels.py`: float32 comparison helpers.
- `gen_imdct_tables.py`: regenerates `firmware/DSPi/ac3/imdct_tables.h`.
- `gen_bench_frames.py`: embeds the bench vector at build time.
- `stock_test.c`, `run_stock.sh`: bit-exact check against upstream.
- `vectors/`: the committed bench vector and its checksum sidecar. To change
  it, write the new file, run `ac3_test decode <file>` and store the printed
  checksum in `<file>.sum`.
