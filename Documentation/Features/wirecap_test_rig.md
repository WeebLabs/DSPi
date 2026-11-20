# Wire-Capture Test Rig (DSPI_WIRECAP)

*Last updated: 2026-11-20*

A bench build that watches the output pins themselves. It exists to prove, on real silicon, that the S/PDIF and I2S wire signals and the inter-slot alignment do not change when the output PIO programs change. The USB loopback build (`DSPI_LOOPBACK`) taps samples before the ring writer and the PIO, so it cannot see encoder or clocking bugs. This rig can.

It has three parts:

- **Pin sampler.** A logic analyser built from one spare PIO state machine and two DMA channels. It samples up to 32 GPIOs into a RAM buffer, one shot at a time, and reports its exact sample clock and whether a sample was ever delayed.
- **I2S clock generator.** A BCK/LRCLK master on GPIO 24/25, so the firmware's I2S clock-slave mode can lock with no external equipment. It can inject clock glitches to trigger the framing-slip watchdog.
- **Host tool.** `tools/wirecap/wirecap.py` decodes the captures, runs a test matrix, writes a JSON result file and a summary, and compares two result files.

Release builds leave the flag off. With it off, every line of rig code is compiled out and the firmware image is byte-identical to a build without the rig.

## Building

```
cmake -S firmware -B build-rp2350-wirecap -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 \
      -DPICO_SDK_PATH=<sdk> -DCMAKE_BUILD_TYPE=Release -DDSPI_WIRECAP=ON
cmake --build build-rp2350-wirecap
```

RP2040 builds with the flag (`-DPICO_PLATFORM=rp2040 -DPICO_BOARD=pico`) but has only been compiled, never run. Its buffers are much smaller (16 KB capture, 4 KB generator ring) and it has a single spare state machine (PIO1 SM3), so the sampler and the generator cannot run at the same time there.

## Resources

| Resource | RP2350 | RP2040 |
|---|---|---|
| Sampler state machine | first free of PIO2 SM3, PIO1 SM3, PIO2 SM2 | PIO1 SM3 |
| Generator state machine | first free of PIO1 SM3, PIO2 SM3, PIO2 SM2 | PIO1 SM3 |
| DMA channels | first free of 9, 10, 14, 13 (sampler 2, generator 1) | first free of 9, 8, 3 |
| Capture buffer | 96 KB BSS | 16 KB |
| Generator ring | 8 KB (128 frames), 8 KB aligned | 4 KB (64 frames) |
| Instruction memory | one instruction each | same |

PIO0 is never touched. The output slots own it, and their programs sit at fixed offsets (the S/PDIF encoder at 0..13, the I2S programs from 14) that leave at most two slots free. PIO1 SM0-2 belong to PDM and ADAT, and PIO2 SM0-3 to the S/PDIF and I2S receivers (only SM0 in the configurations the matrix uses). Everything is claimed through the SDK at arm or enable time and released afterwards, and a claimed resource is skipped. Channels 13 and 14 are ADAT TX's fixed channels, so never enable ADAT output while the rig holds them.

RAM is the binding limit on RP2350. On the 2026-11-20 baseline about 120 KB lay free between `.bss` and `.slot_rings`. The rig takes about 104 KB, which left roughly 14 KB of heap. Recheck that gap with `arm-none-eabi-objdump -h` after any change to the output rings.

## Pin sampler

The state machine runs one instruction, `in pins, N`, with autopush at 32 bits and the RX FIFO joined to 8 entries. Sample `i` of a word lands in bits `[i*N, i*N+N)`, so word bit `k` of a 32-pin capture is GPIO `(base + k) % 32`. The DMA channel that drains the FIFO chains to a second channel, which writes the state machine's enable bit into the PIO `CTRL` clear alias. The state machine therefore stops within a few cycles of the last word, and the 8-entry FIFO absorbs those cycles. A set `FDEBUG.RXSTALL` bit afterwards means a sample really waited for space, so the capture's timeline has a gap. The host treats such a capture as invalid and retakes it.

The sampler never changes a pad's function, direction or input enable. A pin reads meaningfully only while something has it muxed, which every pin under test is.

**Sample rates (RP2350 at 307.2 MHz, measured, 5 captures each):**

| Pins | Fastest stall-free divider | Rate |
|---|---|---|
| 32 | 2 | 153.6 MS/s (divider 1 stalls every time) |
| 4 | 1 | 307.2 MS/s |
| 1 | 1 | 307.2 MS/s |

The matrix uses three regimes. **Fine** is 32 pins at divider 2: 6.5 ns resolution over 160 us. **Coarse** is 32 pins at divider 8: 26 ns over 640 us, about 30 frames at 48 kHz, long enough to match samples across lines. **Slow** is one pin at divider 4: 10.2 ms, which holds at least two Z preambles at 44.1 kHz.

## I2S clock generator

The generator state machine runs `out pins, 2` (GPIO 24 = BCK, GPIO 25 = LRCLK) at four states per BCK period. An endless DMA ring of 128 identical frames feeds it, each frame 64 BCK periods long with LRCLK low (left) for the first 32 and changing on the BCK falling edge. Its divider is `audio_base_divider_16_8()`, the output slots' own 256 x Fs divider. The generator clock and the S/PDIF slot clocks therefore have an exact ratio, and the slave servo sees zero rate error at every rate.

**Glitches** go into the frame 8 frames ahead of the DMA read position. The handler waits until that frame has played (about 0.2 ms) and then repairs the ring itself. A main-loop repair arrives late, because the slip it provokes restarts the pipeline.

| Kind | Effect |
|---|---|
| 1 runt | one extra short BCK pulse in cell 10 of a frame |
| 2 missing | cell 10 has no BCK pulse |
| 3 early | the next LRCLK edge comes `k` BCK periods early; every later edge keeps the new phase |
| 4 late | the next LRCLK edge comes `k` BCK periods late |

Kinds 3 and 4 shorten or lengthen one LRCLK half period, not the whole stream.

### Pad muxing and pin exceptions

On the Pico and Pico 2, GPIO 24 is VBUS sense and GPIO 25 is the LED, so no header wiring can reach them. The firmware does not use GPIO 24 at all. It drives GPIO 25 only as the LED heartbeat through SIO, which has no effect while another function owns the pad. In this build only:

- `single_valid()` (`vendor/vc_pins.c`) accepts 24 and 25 for the I2S clock pair, so `CLOCK.I2S_BCK_PIN` can be set to 24. The device then drives its own master-mode BCK/LRCLK there too.
- The link wraps `gpio_set_function()` and `gpio_init()`. While the generator runs, a request to move GPIO 24/25 to any other function is recorded instead of applied. Slave-mode setup calls `gpio_init()` on the clock pads, which would otherwise disconnect the generator. The slave programs read the pads with `wait gpio` and `jmp pin`, which see the pad whatever its function. When the generator stops, the last recorded request is applied, or the pad returns to the function it had before the generator started. `CLKGEN` status byte 23 shows which pins have a recorded request.

Order matters. Stop the generator only after the device has left slave mode; the deferred master-mode mux is applied at that moment. The matrix does this.

## DIAG commands (page 0xF0)

All exist only in a `DSPI_WIRECAP` build and appear in `Documentation/protocol/dspi_commands.json`. Little-endian throughout.

| Cmd | Name | Dir | Payload / reply |
|---|---|---|---|
| 0x50 | `WIRECAP_ARM` | SET 10 B | u8 pin base, u8 pin count (1, 2, 4, 8, 16 or 32), u16 divider integer, u8 divider fraction /256, u8 0, u32 bytes (multiple of 4, up to the buffer size). Bytes 0 aborts a running capture. `VC_ERR_STATE` while one runs, `VC_ERR_BUSY` when no state machine, DMA channel or instruction space is free. |
| 0x51 | `WIRECAP_STATUS` | GET 32 B | u8 state (0 idle, 1 running, 2 done, 3 aborted), u8 flags (bit 0 RXSTALL, bit 1 RXUNDER), u8 base, u8 count, u32 clk_sys Hz at arm, u16 divider integer, u8 fraction, u8 PIO index, u32 bytes requested, u32 bytes captured, u8 SM, u8 data DMA, u8 stop DMA, u8 0, u32 buffer size, u32 capture sequence number |
| 0x52 | `WIRECAP_READ` | GET var, wValue = chunk | 256 bytes of the finished capture from byte `256 * wValue`; `VC_ERR_STATE` before a capture is done |
| 0x53 | `WIRECAP_CLKGEN` | SET 5 B / GET 24 B | SET: u8 enable, u32 rate (44100, 48000 or 96000). A rate change while running only reloads the divider. GET: u8 on, u8 PIO, u8 SM, u8 DMA, u32 rate, u32 divider (16.8), u32 glitches injected, u32 late repairs, u8 phase in quarter BCK periods, u8 0, u8 last glitch kind, u8 deferred-mux pin mask |
| 0x54 | `WIRECAP_CLKGEN_GLITCH` | SET 2 B | u8 kind (1 to 4, above), u8 cells (1 to 31, kinds 3 and 4). `VC_ERR_STATE` if the generator is off. |
| 0x55 | `WIRECAP_RATE` | SET 4 B | u32 rate (44100, 48000, 96000). Takes the same deferred path as a host `SET_CUR` on the USB endpoint, for use while no audio stream runs. USB input only (`VC_ERR_STATE` otherwise). |

`WIRECAP_RATE` exists because CoreAudio refuses to set the DSPi output endpoint to anything but 48 kHz.

## Host tool

`tools/wirecap/` holds three files. `rig.py` sends the DIAG commands and reads the health counters. `decode.py` holds the S/PDIF and I2S decoders and the alignment measurement. `wirecap.py` is the command line. It needs pyusb and numpy and reads command names from the registry.

```
cd tools/wirecap
python3 wirecap.py rates                      # stall-free sampler dividers
python3 wirecap.py matrix -o results.json     # whole matrix, writes results_summary.txt too
python3 wirecap.py matrix --only glitch master/mixed -o part.json
python3 wirecap.py compare baseline.json new.json
```

### What it checks

**S/PDIF**, per line:

- the biphase-mark half-cell runs quantize to 1, 2 or 3 half cells, and the bit-cell timing statistics are recorded
- every preamble is X, Y or Z, with X/Z and Y alternating
- a transition falls at every bit-cell boundary
- even parity holds on every subframe, and V is 0
- Z comes exactly every 192 frames (slow capture)
- the channel-status rate code matches the rate

**I2S**, per line:

- exactly 64 BCK rising edges per LRCLK period, 32 per half
- LRCLK and data change within one sampler tick of a BCK falling edge
- 24-bit samples are read MSB first, one BCK after the LRCLK edge

The firmware sends 32 significant bits, so a nonzero low byte is recorded but not judged.

**Alignment.** The signal generator plays the same white noise at -40 dBFS (RAW, outputs 0-7) on every output, and master volume stays as the user left it. For each pair of slots, a sample value seen on line A is found on line B. The time between the two frame starts (the X/Z preamble for S/PDIF, the LRCLK falling edge for I2S) gives a whole-frame offset (the sample-index offset) and a residual inside one frame. The fine capture measures the residual to 6.5 ns. The pair offsets form the configuration's **alignment signature**.

**Matrix.** It has 27 cases:

- Clock master: all S/PDIF, mixed (slots 1 and 3 I2S) and all I2S, each at 44.1, 48 and 96 kHz. The input is USB, which is idle, and the signal generator drives the pipeline.
- Clock slave on the generator: the same 9 combinations. The input is I2S with its data pin on GPIO 11. GPIO 1, the default, is the bench unit's UART control RX.
- Events, mixed, 48 kHz master. The signature is captured before and after each of: slot 1 to S/PDIF and back; a rate change to 96 kHz and back; master to slave to master; and `STORE.ALL_PARAMS` read and written back unchanged. A preset load writes flash (it records the active slot), so the bulk re-apply stands in for it: it takes the same deferred full restart without a flash write.
- Glitches, mixed, 48 kHz slave: runt, missing, LRCLK 1 cell early, 1 cell late, 5 cells early. Each must raise the slip count, relock, and restore the signature.

Every measurement records the change in `SYS.PIPELINE_STATS` underruns and drops, the `SYS.SPDIF_TX_STATS` starvation counters, and the slave lock, loss and slip counts.

### Tolerances

- **Sample-index offsets** must match exactly, always.
- **Residuals between lines on the same clock** (every pair in master mode, and S/PDIF-to-S/PDIF or I2S-to-I2S pairs in slave mode) must match within 2 fine ticks (13 ns). Each line's edge is quantized to one tick, so a difference can move by two.
- **S/PDIF-to-I2S residuals in slave mode** must match within 2 us. In slave mode the S/PDIF slots run from the sys clock while the I2S slots follow the external BCK. The synchronized start waits for an LRCLK edge in software, then writes the enable. The residual is constant within one lock: 12 readings over 25 s stayed within one fine tick. Each relock lands it anew, though. Over 36 relocks on the baseline (12 at each rate) it took values from -0.18 to -1.92 us, with clusters near the master-mode value and about 0.7 us earlier. 2 us covers that spread and is still under a quarter frame at 96 kHz, so it cannot hide a whole-sample error. The sample-index offset is still compared exactly.

### Comparing two builds

`compare` matches cases by name. A case passes when:

- B's own checks pass
- every signature it holds (before and after, for events) matches A's within the tolerances above
- no S/PDIF line's channel-status bytes changed
- no wire check that passed in A fails in B

The exit status is 1 if any case fails.

## Limits

- The generator is derived from the same PLL as everything else, so slave mode is exercised at an exact frequency ratio, never with a truly asynchronous master. Servo behaviour against a drifting clock is not tested.
- RP2040 is compile-only.
- The coarse window is about 30 frames, so the sample-index offset is found only up to about plus or minus 12 frames. Large output delays would put matching samples outside the window. The matrix leaves output delays as configured (all zero on the bench unit).
- Captures happen while the output is static. Transient behaviour during a switch is not recorded. Only the state before and after is.
- PDM and ADAT output are not observed.
- The rig assumes the clock-pin mode is UNIFIED and MCK is off, and refuses to start otherwise.
