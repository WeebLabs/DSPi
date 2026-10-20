# Spectrum Analyser (RTA / FFT) Specification

*Status: implemented, hardware-untested*
*Last updated: 2026-09-12 (shared elliptic b0; Blackman-Harris FFT window; sixth-order bass bands; protocol V3, continuous 10–200 Hz bass bank plus FFT up to 1024 points)*

## 1. Overview

The onboard **spectrum analyser** ("RTA") is a single FFT engine that can be
pointed at any set of channels on either side of the processing chain. It
produces two products from every frame: a hybrid third-octave **band table** small
enough for any transport, and the **raw magnitude bins** for the Console's
FFT view. It exists so that S/PDIF, ADAT, and I2S inputs, every output, and
non-USB clients (the ESP32 front panel, control-surface OLEDs, UART and I2C
bridges) can all see a spectrum. For USB playback the host already holds the
samples and can transform them itself.

### Key characteristics

- **One engine, selectable tap.** The tap is either the input side (after the
  per-input PEQ, before the matrix) or the output side (after gain and delay,
  before encoding, which is exactly what the slot transmits).
- **Any set of channels.** One FFT rotates through the selected channels at
  constant aggregate transform cost. Each selected channel also has a continuous
  bass filter bank: its small CPU cost scales with selected channel count.
- **Bounded audio work.** The callback copies the current FFT channel and updates
  the selected bass banks. FFT work remains resumable in the main loop. Bass work
  is synchronous audio work and must fit the packet deadline on hardware.
- **Alignment-neutral by construction.** The tap is read-only and changes no
  sample count, so it cannot move any output slot relative to another.
- **Small.** The transform runs in place in the capture buffer, and every
  constant table lives in flash.
- **Configurable cost.** FFT size (256 to 1024 points) and averaging let the
  user trade CPU and refresh rate for resolution.
- **Transient only.** Never persisted, off at boot, not part of presets or the
  bulk-params blob, and it switches itself off when nobody is reading it.
- **Platform parity.** Identical wire protocol on both platforms. RP2350 uses a
  float kernel, RP2040 a Q15 kernel with a higher numerical noise floor. The bass bank uses float on RP2350 and Q27 states/Q28 coefficients with 64-bit accumulators on RP2040.

### Signal-chain position

```
process_input_block():
    PASS 2: per-input PEQ + metering          >>> INPUT TAP (per-input meter loop)
    PASS 2.5 / 3: leveller, upmixer
    PASS 4: matrix mix -> buf_out[]
    siggen injection
    PASS 5-7: crossover + PEQ, trim, master volume, mute, delay
              per-output metering              >>> OUTPUT TAP (per-output meter loop,
              S24 finalize / encode                 on whichever core owns the row)
```

Both taps sit inside the existing peak-meter loops, which already run once per
block per channel on the core that owns that channel, after every sample-value
change and before any conversion. The output tap therefore sees the same values
the slot transmits, and it runs before `output_block_to_s24_inplace()` so it
never reads a row that has already been converted while ADAT is active.

## 2. Products

### 2.1 Band table

Third-octave bands with IEC 61260 nominal centres from 10 Hz upward. The band
count depends on the sample rate:

| Sample rate | Bands | Top band centre |
|-------------|-------|-----------------|
| 44.1 kHz | 34 | 20 kHz |
| 48 kHz | 34 | 20 kHz |
| 96 kHz | 37 | 40 kHz |

`RTA_MAX_BANDS` is 37. Above 200 Hz, a band's level is the sum of the windowed bin powers
inside the band's edges, normalised so that a full-scale sine reads 0 dBFS in
the band that contains it. Pink noise therefore reads flat across bands, and
white noise rises 3 dB per octave, which is standard RTA behaviour.

The following FFT resolution rules apply above the bass bank; its 14 bands are always populated at supported rates. The DC bin belongs to no band. A band that contains no bin at the current
size and rate is **empty** and reads the floor (0); it is never faked from a
neighbouring bin. `RtaStatus.first_band` is 0 for supported layouts because the bass bank reaches 10 Hz. Clients must still use the FFT geometry to identify empty bands above 200 Hz at small FFT sizes/high sample rates. A band that holds only one
or two bins captures only part of a tone's window main lobe and reads a centred
tone up to 3.02 dB low (one bin) or 0.84 dB low (two bins); wideband levels are
unaffected.

Lowest FFT bin-containing band per size, at 48 kHz (raw FFT only):

| Points | Bin width | First resolved band | Frame time |
|--------|-----------|---------------------|------------|
| 256 | 187.5 Hz | 200 Hz | 5 ms |
| 512 | 93.75 Hz | 100 Hz | 11 ms |
| 1024 | 46.9 Hz | 50 Hz | 21 ms |

At 44.1 kHz each FFT size reaches one band lower; at 96 kHz one octave
higher. The 1024-point ceiling and its buffer are unchanged. Bass band levels
come from the separate continuous bank described below, not from interpolated
or zero-padded FFT bins.

### 2.1.1 Continuous bass bank

Fourteen bands use exact base-10 centres 10 Hz through 199.526 Hz. The nominal
caps table starts `10, 13, 16, 20, 25, 32, ...`; the whole-Hz wire format rounds
12.589 and 31.623 Hz to 13 and 32. Each selected/live channel is fed on every
packet, including FFT/FINISH phases and other channels' capture turns.

The analyzer-only branch is CIC3 decimation by 8 at 44.1/48 kHz or 16 at
96 kHz, an eighth-order elliptic low-pass (225 Hz pass edge, 0.1 dB ripple,
85 dB stop specification), then decimation by 8. Final rate is 689.0625 or
750 Hz. The CIC uses defined unsigned 32-bit modular accumulators on both
platforms; input quantisation is Q18 at 44.1/48 kHz, Q15 at 96 kHz, clipped at
+/-4 FS. Prefix gain normalisation of the four low-pass sections prevents
internal fixed-point overload. Each elliptic section's zeros lie on the unit
circle, so b2 equals b0. The generator stores b0 once and the kernel reuses the
b0*x product. Coefficients are generated by
`scripts/gen_rta_bass.py` and copied to RAM before streaming.

Each band is a sixth-order Butterworth bandpass (three `b0*(1 - z^-2)`
sections, lowest-Q first, every cascade prefix normalised to at most unity
peak gain) followed by a continuous power EMA. A single biquad per band was
rejected because its 6 dB/octave skirts made one bass tone light every band
in the bank.
The EMA time constant is the longer of one centre-frequency period and
`avg_ms`. It directly supplies the published average, avoiding duplicate state
and a second averaging delay. Peak hold remains updated at each channel's FFT
publication cadence. Per-centre power correction compensates decimator and bandpass gain.
The fixed detector uses Q48 power in a 64-bit word and clamps detection above
+12 dBFS; float uses a float power state. `RESET_AVG` clears filter history too.

At `avg_ms=0`, host tests measure 90% power response in 590 ms at 10 Hz,
300 ms at 19.953 Hz, and 50 ms at 199.526 Hz. Publication adds up to a normal
FFT rotation interval; it does not multiply the filter settling time by the
number of channels. Larger `avg_ms` deliberately slows response.

These are **tone-calibrated RTA bands**, not certified IEC 61260 measurement
filters or additional raw FFT bins. Host tests require a tone to read at least
40 dB down one octave from its band and 60 dB down two octaves away (measured
worst case 42.4 dB at one octave). Adjacent bands still cross about 18 dB down
at the neighbouring centre. Numerical frequency-response integration predicts
pink-noise levels 0.1–0.2 dB above ideal disjoint third-octave integration. `bass_dynamic_range_db=70` is a conservative separate capability; the
FFT's 78/120 dB capability does not describe the bass path.

Two arrays are kept per channel: the **averaged** level and the **peak-hold**
level. Above 200 Hz, averaging is an exponential moving average in the power domain (not the
dB domain, which under-reads noise by up to 2.5 dB in single-bin bands). Its
coefficient is derived from `avg_ms` and the channel's actual refresh interval
so the time constant stays in real time regardless of how many channels share
the rotation. Peak hold decays at `peak_decay_db_s`, with the decay time
carried between publishes so short intervals still decay.

### 2.2 Raw bins

The most recent frame's magnitude bins, `N/2` values, normalised so a full-scale
sine at a bin centre reads 0 dBFS. Only the latest frame is kept, tagged with
its channel and a sequence number, because a bin set per channel would cost
more RAM than the whole feature. A host that wants per-channel bins for a
multichannel set polls at the frame rate; the intended use is single-channel.

### 2.3 Level encoding

Every level on the wire is one byte in 0.5 dB steps:

```
level_dBFS = (v - 243) * 0.5        v = 255 -> +6.0 dBFS
                                    v = 243 ->  0.0 dBFS
                                    v = 0   -> -121.5 dBFS or below (floor)
```

The 6 dB headroom exists because upmix-derived rows and hot EQ can legitimately
exceed 0 dBFS. `RTA_LEVEL_ZERO_DBFS` = 243 is reported in the caps so clients
never hard-code it.

## 3. The engine

### 3.1 Frame state machine

```
IDLE  --start-->  FILL  --buffer full (audio callback)-->  FFT
FFT  --main loop, one step per call-->  FINISH  -->  publish  -->  advance
advance: next channel in the selected set, wr = 0, back to FILL
```

- **FILL.** Each block, the owning core's tap copies up to the remaining
  buffer space from the tapped row and writes back how many samples it
  copied. On RP2350 the samples are copied as float. On RP2040 the Q28 samples
  are converted to saturated Q15 (`>> 13`). When the buffer fills mid-block
  the tail of that block is dropped.
- **FFT.** `rta_service()` runs from the main loop next to `siggen_service()`.
  Each call performs one bounded unit of the transform and returns: the
  prescale, one butterfly stage, the permutation, or the real-split pass. No
  unit exceeds roughly 250 µs on RP2040 at the largest size, so the main loop's
  USB and transport duties never stall.
- **FINISH.** One call applies the window, sums bin powers into bands, writes
  the bin levels, publishes, and advances the rotation. This is the longest
  single step and is charged to `last_frame_us` so the bench can see it.

Because the transform runs in place, capture is off while it runs. The gap is
the transform time, a fraction of a millisecond on RP2350 and a couple of
milliseconds on RP2040, against a 21 ms frame at 1024 points and 48 kHz.

### 3.2 Channel rotation

The selected set is `channel_mask` at the chosen tap, intersected with what is
actually live: active input rows for the input tap (including upmix-derived
rows while the upmixer runs), enabled outputs for the output tap. The rotation
walks the set in ascending bit order. Bits for channels that are not live are
skipped, so disabled outputs do not slow the refresh of the others.

Per-channel refresh interval = (fill time + transform time) x number of live
channels in the set.

| Points | Rate | Fill time | 1 channel | 2 channels | 8 channels |
|--------|------|-----------|-----------|------------|------------|
| 1024 | 48 kHz | 21.3 ms | 21 ms | 43 ms | 171 ms |
| 512 | 48 kHz | 10.7 ms | 11 ms | 21 ms | 85 ms |
| 1024 | 96 kHz | 10.7 ms | 11 ms | 21 ms | 85 ms |

The engine advances by what the tap actually copied each packet, and the main
loop moves on if the channel being filled stops being live (a USB alt change,
a disabled output, the upmixer parking), so a frame can never stall.

### 3.3 Cross-core tap

Output rows 2 and above are processed by Core 1 in EQ worker mode, so the tap
for those rows runs on Core 1. Core 0 writes the tap descriptor into
`rta_view` (`RtaPacketView`, rta.h) in `rta_packet_begin()` at the top of
`process_input_block()`, ahead of the `work_ready` barrier, so both cores use
one view per packet. This follows the same single-view rule as the loudness,
crossfeed, and subharm snapshots in `Core1EqWork`, just in its own struct.

Only the core that owns the row copies samples this packet, and it copies to
the snapshotted index. It writes back one field, `produced`, which Core 0
reads in `rta_packet_end()` after the `work_done` join. Each owning core additionally updates its own bass channel state and per-packet timing slot, selected by a snapshotted bass mask. Core 0 clears channels entering/leaving the mask before dispatch and reads bank powers/timings only after the existing join. Core 0 alone advances
the index and changes phase, so the two cores never touch the same
bookkeeping. The capture buffer is written by at most one core per packet and
read by the main loop only after the fill completes, so no lock is needed
beyond the existing handshake barriers.

Every Core 0 entry into the pipeline (the USB ring drain, the S/PDIF, I2S,
and ADAT polls, the signal-generator pump) runs from the main loop, as do the
vendor handlers, so `rta_service()` and `process_input_block()` never
pre-empt each other.

### 3.4 Window, power, and normalisation

The window is the 4-term minimum Blackman-Harris (0.35875, 0.48829, 0.14128,
0.01168), applied in the frequency domain after the transform and scaled to a
0.5 centre tap:

```
Xw[k] = 0.5 * X[k] - 0.340271777 * (X[k-1] + X[k+1])
      + 0.098452962 * (X[k-2] + X[k+2]) - 0.008139373 * (X[k-3] + X[k+3])
```

Q15 uses taps 16384, 11150, 3226 and 267. Bins past DC and Nyquist come from
conjugate symmetry. The scaling keeps the Hann-era bin normalisation (a
bin-centred full-scale sine reads 0 dBFS). Band sums divide by 0.125272059,
the sum of squared taps over 4.

Hann was replaced because its sidelobes fall away slowly. With the bass bank
reading 70-90 dB down around 200 Hz, a loud 20-100 Hz tone leaked into the
first FFT bands at -40 to -55 dB and drew a fixed hump whose position moved
with FFT size. Engine simulation at 1024 points now puts that leakage at
-98 dB or lower in float. The cost is a wider main lobe. At 512 and 256 points
the lowest FFT band sits within two or three bins of loud bass and reads it
higher than Hann did; bands from 500 Hz up are clean at 512 points.

Applying the window after the transform removes the per-sample multiply and
the window table. Powers are summed
per band in the linear domain and converted to dB once per band; the bin
product converts once per bin using a fast log2 on the float exponent (RP2350)
or a leading-zero count (RP2040).

### 3.5 Restart conditions

The current frame is discarded and capture restarts from index 0 on: a sample
rate change, an input source switch, a preset load or factory reset, and any
config change that alters tap, mask, or size. `rta_service()` also restarts
on its own if it sees the device rate differ from the rate it armed with.
Averaging and peak-hold state are cleared on tap, mask, or rate change and by
`RTA_CTL_RESET_AVG`. The soft-mute envelope is not a restart condition; the
tap simply sees the faded samples.

### 3.6 Auto-off

The analyser records the time of the last band or bin read from any transport.
If `RTA_IDLE_TIMEOUT_MS` (5000) passes without one, it stops, releases the
tap, and returns to IDLE. Any band or bin read starts it again, so a client
that opens an RTA view and polls simply works, and a device nobody is watching
spends nothing. A start counts as a read, so `RTA_CTL_START` is not undone by
the next service pass. With `RTA_FLAG_MANUAL` the run state belongs to
CONTROL entirely: no auto-start on read and no auto-off, so a manual client
that starts the analyser must also stop it.

## 4. Kernel

`rta_fft.h` exposes a platform-neutral kernel used by the engine and by the
host-side test harness. Both platforms implement the same interface:

```c
// Real FFT of n = 1 << order samples, in place, via an n/2-point complex
// transform plus a split pass.  Twiddles come from a flash table sized for
// the largest order; smaller orders stride through it.  Executes one
// bounded unit per call so the caller can limit main-loop latency.
//   returns true when the last stage has completed.
bool rta_fft_step(rta_sample_t *buf, uint8_t order, uint8_t *stage);

// Frequency-domain Blackman-Harris, bin power, band sums and bin magnitudes.
// bands[]/bins[] receive RTA level bytes (section 2.3).
void rta_fft_finish(const rta_sample_t *buf, uint8_t order,
                    const RtaBandTable *table, float *band_power,
                    uint8_t *bins);
```

`rta_sample_t` is `float` on RP2350 and `int16_t` on RP2040. The RP2040 kernel
scales by one half per stage and keeps bin power in 32-bit integers; its
measured dynamic range is 78.5 dB (reported as 78 in the caps). The RP2350
kernel's is 120 dB. Two real samples packed into one complex slot give a
sqrt(2) full-scale magnitude, so the Q15 path prescales by 1/sqrt(2) and the
split pass restores it; that costs 3 dB of Q15 range.

### 4.1 Generated tables

`scripts/gen_rta_tables.py` writes `firmware/DSPi/rta_tables.h`:

- complex twiddles for the largest supported transform, float and Q15;
- split-pass twiddles;
- band edge tables: for every (sample rate, order) pair, the first and last
  bin index of every band, with empty bands encoded as `lo > hi`;
- the log2 mantissa table and the band centre frequencies in Hz for the caps.

All tables are `const` and stay in flash. The generator is the single source of
truth for the band layout; the host test harness imports it.

### 4.2 Kernel acceptance (host, numpy oracle)

Compiled natively with clang and driven from Python (`tools/rta_test/run.sh`),
for every order and both sample formats:

- A full-scale sine at any frequency reads 0 dBFS within 0.5 dB in the band
  that contains it. On a band edge the window main lobe splits across two bands:
  the pair sums to 0 dBFS within 0.5 dB and the better single band never falls
  below -3.05 dB.
- A sine at -60 dBFS reads within 0.2 dB (RP2350) or 1.25 dB (RP2040; 1.5 dB
  where the band sums more of the Q15 floor). The measured worst case is
  1.24 dB at 1024 points, 0.01 dB inside the tolerance; the wider window sums
  more Q15 floor than Hann's 1.11 dB.
- With a -100 dBFS sine, every bin more than three bins away reads below the
  platform's dynamic-range floor.
- Pink noise reads flat across bands within 1 dB after averaging 64 frames.
- Bin magnitudes match a numpy Blackman-Harris FFT of the same input within 0.35 dB
  (RP2350) or 0.75 dB (RP2040) above the floor.

`tools/rta_test/test_engine.py` additionally compiles the real engine against
stubs and checks the protocol, all-channel cadence, bin frame, liveness
recovery, restart, config rejection, and auto-off in both formats.

## 5. Wire protocol

### 5.1 Commands

Vendor commands 0x08 to 0x0F. All GETs are `bmRequestType` 0xC1, all SETs
0x41, following the shared dispatcher, so UART and I2C get them for free.

| Code | Name | Dir | wValue | Payload / response |
|------|------|-----|--------|--------------------|
| 0x08 | `REQ_RTA_SET_CONFIG` | SET | 0 | `RtaConfig` (12 B). STALL on invalid |
| 0x09 | `REQ_RTA_GET_CONFIG` | GET | 0 | `RtaConfig` (12 B) applied config |
| 0x0A | `REQ_RTA_GET_CAPS` | GET | 0 | `RtaCaps` (16 B); 1..n = band centre table chunk |
| 0x0B | `REQ_RTA_GET_BANDS` | GET | channel | `RtaBandFrame` (82 B) for that channel at the current tap |
| 0x0C | `REQ_RTA_GET_BINS` | GET | byte offset | chunk of the bin frame, `wLength` bytes |
| 0x0D | `REQ_RTA_GET_STATUS` | GET | 0 | `RtaStatus` (24 B) |
| 0x0E | `REQ_RTA_CONTROL` | GET | action | one status byte, like `REQ_SIGGEN_CONTROL`. `RTA_CTL_*` |
| 0x0F | `REQ_RTA_GET_BANDS_ALL` | GET | 0 | every live channel's `RtaBandFrame` back to back, USB only |

`REQ_RTA_GET_BANDS`, `GET_BINS` and `GET_BANDS_ALL` count as reads for the
auto-off timer and auto-start the analyser. `GET_STATUS` does not.

Every applied `REQ_RTA_SET_CONFIG` is announced with `NOTIFY_EVT_RTA_CONFIG`
(0x0D), carrying the applied `RtaConfig` and the SET's source. See
`notification_protocol_v2_spec.md` §3.9 and section 7 below.

`REQ_RTA_GET_BANDS_ALL` is refused on UART and I2C the same way 0xA0 is; those
clients read per channel with 0x0B, which fits their 132-byte GET limit. On
USB it is built in `bulk_param_buf` under the bulk lock, acquired and released
at the same three sites as 0xA0 through `vendor_is_bulk_get()`.

`REQ_RTA_GET_BINS` follows the 0xA2 chunk pattern with `wValue` as the byte
offset into the bin frame. The frame is `16 + n_bins + 1` bytes: the header
carries the sequence number, and the same number sits in the final byte, so a
host that reads several chunks validates that both match and re-reads if the
frame turned over in between. The engine clears the tail to 0xFF before it
writes and sets it last; sequence numbers skip 0xFF. The layout is fixed for
the life of a config. No lock is taken. The bin frame is never larger than
529 bytes.

### 5.2 Structures

```c
#define RTA_CFG_VERSION      3
#define RTA_MAX_BANDS        37
#define RTA_LEVEL_ZERO_DBFS  243

#define RTA_TAP_INPUT   0     // after per-input PEQ, before matrix
#define RTA_TAP_OUTPUT  1     // after gain + delay, before encode

#define RTA_FLAG_MANUAL 0x01  // CONTROL owns run state: no auto-start, no auto-off

typedef struct __attribute__((packed)) {
    uint8_t  version;          // RTA_CFG_VERSION
    uint8_t  tap;              // RTA_TAP_*
    uint16_t channel_mask;     // bit i = channel i at that tap; 0 = STALL
    uint8_t  fft_order;        // 8..10 (256 to 1024 points)
    uint8_t  reserved0;
    uint16_t avg_ms;           // power-domain EMA time constant, 0 = none
    uint8_t  peak_decay_db_s;  // 0 = peak hold off, else dB per second
    uint8_t  flags;            // RTA_FLAG_*
    uint16_t reserved;
} RtaConfig;                   // 12 bytes

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  input_channels;   // NUM_INPUT_CHANNELS
    uint8_t  output_channels;  // NUM_OUTPUT_CHANNELS
    uint8_t  fft_order_min;    // 8
    uint8_t  fft_order_max;    // 10
    uint8_t  fft_order_default;// 10 on RP2350, 9 on RP2040
    uint8_t  bass_bands;       // 14 continuous bands from index 0
    uint8_t  max_bands;        // RTA_MAX_BANDS
    uint8_t  level_zero;       // RTA_LEVEL_ZERO_DBFS
    uint8_t  dynamic_range_db; // 78 on RP2040 (measured), 120 on RP2350
    uint16_t idle_timeout_ms;  // RTA_IDLE_TIMEOUT_MS
    uint16_t max_bin_frame;    // largest bin frame in bytes (529)
    uint16_t bass_dynamic_range_db; // 70, independent of FFT dynamic_range_db
} RtaCaps;                     // 16 bytes

// GET_CAPS wValue 1..: uint16 band centre frequencies in Hz, 32 per chunk.

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  channel;          // channel index at the current tap
    uint8_t  seq;              // increments per frame of this channel
    uint8_t  n_bands;          // bands valid at the current sample rate
    uint16_t age_ms;           // since this channel's last frame; 0xFFFF = never
    uint16_t reserved;
    uint8_t  avg[RTA_MAX_BANDS];
    uint8_t  peak[RTA_MAX_BANDS];
} RtaBandFrame;                // 82 bytes

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  channel;
    uint8_t  seq;              // frame sequence, repeated as the final byte
    uint8_t  fft_order;
    uint32_t sample_rate_hz;
    uint16_t n_bins;           // (1 << fft_order) / 2
    uint16_t reserved[3];
    // uint8_t bins[n_bins]; uint8_t seq_tail;
} RtaBinFrameHeader;           // 16 bytes

#define RTA_STATE_IDLE          0
#define RTA_STATE_CAPTURING     1
#define RTA_STATE_TRANSFORMING  2

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  state;            // RTA_STATE_*
    uint8_t  tap;              // applied tap
    uint8_t  channel;          // being captured / transformed, 0xFF = none
    uint8_t  reserved0;
    uint8_t  live_count;       // channels in the rotation right now
    uint16_t live_mask;        // selected AND live
    uint16_t frames_per_s;     // frames completed per second
    uint16_t busy_us_per_s;    // main-loop microseconds per second spent transforming (EMA)
    uint16_t last_frame_us;    // wall time of the last complete transform, for the bench
    uint16_t idle_ms;          // since the last data read; 0xFFFF = never
    uint32_t sample_rate_hz;
    uint8_t  first_band;       // 0 for supported bass layouts, else 0xFF
    uint8_t  reserved1;
    uint16_t bass_busy_us_per_s; // summed bass tap time, saturates at 65535
} RtaStatus;                   // 24 bytes

#define RTA_CTL_STOP       0
#define RTA_CTL_START      1
#define RTA_CTL_RESET_AVG  2   // clear averaging and peak hold, keep running
```

`busy_us_per_s` exists because the existing CPU load figure measures only the
packet callback and cannot see main-loop work. `bass_busy_us_per_s` separately
reports summed elapsed microseconds spent inside bass tap calls on both cores
normalised to one second from the last statistics window, saturating at 65535.
It excludes packet bookkeeping, and includes interrupt/preemption time during
those calls; it is not a cycle counter. The audio CPU meters also include this
work, so do not add the bass figure to them again. 10,000 us/s equals 1% of one
core's available time. Hardware CPU/deadline and output-alignment tests remain
required before release.

V3 is deliberately incompatible with V2 config/frame interpretation. Config,
caps, status and bin-header sizes stay 12/16/24/16 bytes; band frames grow from
80 to 82 bytes and their indices shift by three. Read `max_bands` and the centre
table, and reject unsupported versions. GET_BANDS_ALL uses 82-byte strides.
No command IDs were added. The separate Console repository needs a matching V3
client update; this firmware repository contains only the device test client.

### 5.3 Validation

`REQ_RTA_SET_CONFIG` STALLs on wrong version or length, unknown tap, an
`fft_order` outside the caps range, or an empty `channel_mask` after masking
to the tap's channel count. `avg_ms` is clamped to 0..10000 and
`peak_decay_db_s` to 0..100. A config that is valid but changes tap, mask, or
size restarts the frame (section 3.5). A change of `avg_ms`,
`peak_decay_db_s`, or `flags` alone applies at the next publish.

## 6. Cost

### 6.1 RAM

The 1024-point capture buffer stays 4,096 B on RP2350 / 2,048 B on RP2040;
the raw-bin frame stays 529 B. The shared bass coefficient block is 364 B.
Each bass channel is 228 B float or 288 B fixed (wider power state), while
removing the 14 duplicated float EMA states saves 56 B per channel.

Measured BSS increase over the preceding single-FFT firmware: **2,000 B
RP2350 / 1,580 B RP2040**, including the V3 band/frame changes and timing state.
Total analyzer BSS is consequently about 8.6 KB / 5.3 KB. RAM-resident bass
code is additional to BSS; use `scripts/check_ram_placement.py` for total RAM
accounting and the hot-call closure. No audio buffers, delay lines, output
slots, or pipeline reset sequencing are changed.

### 6.2 CPU

FFT throughput still shares one engine. Bass cost scales with selected/live
channels: at 48/96 kHz each channel executes 144,000 CIC integrator additions
at 48 kHz (288,000 at 96 kHz), 24,000 low-pass biquad updates and 10,500 bass
biquad/detector updates per second. Input conversion, comb differences and
bookkeeping are additional. RP2040 uses 64-bit fixed-point intermediates at
the decimated rates; RP2350 uses float there. No on-device CPU percentage is
claimed from host tests. Check both `busy_us_per_s` and
`bass_busy_us_per_s`, plus the existing audio CPU/deadline metrics, on hardware.
Unlike the background FFT, bass filtering is synchronous audio work.

### 6.3 Flash

Twiddle and band tables: about 7.5 KB on RP2350 and 4.5 KB on RP2040. Kernel,
engine, and handler code: about 6 KB. The tap and the packet begin/end
bookkeeping, live-mask helper and bass streaming kernel are `DSP_TIME_CRITICAL`, because they run inside the meter loops
on both cores during flash writes. The transform, the band sums, and all
control paths stay in flash.

## 7. Sharing between hosts

The device has one analyser and one configuration, but several hosts can
watch it at once: the USB host, any number of DSPi Link sessions through a
hub, and a UART or I2C bridge. Every band read from any of them keeps the
analyser running and any of them can push a configuration. The firmware does
not arbitrate, because every host reaches it through the same dispatcher and
it cannot tell sessions apart. It does three things that let hosts share
without fighting.

### 7.1 Announce every applied configuration

`apply_config()` pushes `NOTIFY_EVT_RTA_CONFIG` (0x0D) once per applied SET,
with the applied `RtaConfig` and the SET's dispatch source. The source is
captured when the SET is staged, because the apply happens later on the main
loop. On USB the event echoes back to the host that sent the SET and to any
Link session through the hub, which attributes it to the session that wrote
it. A host that sees a configuration it did not send knows another host has
taken the analyser, and a host that sees its own echo knows its push landed
without reading the configuration back.

### 7.2 Auto-off and auto-start are the ownership model

A host that stops watching simply stops reading. The analyser stops after
`RTA_IDLE_TIMEOUT_MS` without a read from anyone, and the next read from
anyone restarts it with the configuration still in place. `RTA_CTL_STOP` from
one host while another is reading is therefore only a glitch: the next read
restarts the engine, at the cost of one discarded frame and cleared averages.
Hosts that can see other watchers (a DSPi Link hub) must not forward a STOP
while another session is watching; see the DSPi Link spec, section 7.10. A
host that cannot see other watchers may send STOP, since the firmware repairs
it. `RTA_FLAG_MANUAL` disables this repair and is reserved for a single
dedicated client; a shared host must not set it.

### 7.3 Host rules

These are the rules the DSPi Console follows; any other host that shares the
analyser should follow the same ones.

1. **Last chooser drives.** Whoever pushed the most recent configuration owns
   the analyser. There is no reservation and no lock.
2. **A host follows a configuration it did not push.** On an `RTA_CONFIG`
   event (or a readback) whose tap, mask or FFT order differ from what the
   host asked for, and whose origin is not the host itself, the host adopts
   the device's configuration, shows the channels the device is analysing,
   and stops re-pushing its own. It tells the user it is following and offers
   to take control.
3. **A user action takes control.** Choosing a channel, a tap or an option,
   or pressing "take control", pushes the host's own configuration again. When
   the tap matches, the host asks for the union of its own channels and the
   channels the device is currently analysing, so the other host keeps its
   picture. The device analyses every channel in the mask anyway and
   `GET_BANDS_ALL` returns them all in one read.
4. **A superset is success.** A readback or event counts as the host's
   configuration applied when the tap and FFT order match and the device's
   mask includes every channel the host asked for. The host adopts the
   clamped `avg_ms` and `peak_decay_db_s` the device reports.
5. **A refusal is still a refusal.** A configuration the device never applied
   (no echo, readback never agrees) is reported as rejected after a bounded
   number of pushes, as before. Following is only entered when the device
   holds a valid configuration that someone else pushed.
6. **`RESET_AVG` stays shared.** It is momentary and intentional, like
   pressing reset on a shared meter, and it does not restart the frame.
7. **Without the event**, on firmware that predates it, the host keeps reading
   `REQ_RTA_GET_CONFIG` back. A readback that disagrees after an earlier one
   agreed means another host changed the configuration and the host follows
   it with an unknown origin. A readback that never agreed is the refusal
   path of rule 5.

## 8. Files

| File | Content |
|------|---------|
| `firmware/DSPi/rta.h`, `rta.c` | engine: state machine, rotation, tap, publish, averaging, service, auto-off, wire helpers |
| `firmware/DSPi/rta_fft.h`, `rta_fft.c` | kernel: FFT, split, window/power/bands (both platforms) |
| `firmware/DSPi/rta_tables.h` | generated twiddles and band tables |
| `firmware/DSPi/rta_bass.c`, `rta_bass.h`, `rta_bass_tables.h` | continuous bass bank, state, generated decimator/resonator coefficients |
| `scripts/gen_rta_bass.py` | SciPy bass coefficient generator |
| `scripts/gen_rta_tables.py` | table generator, single source of truth for band layout |
| `tools/rta_test/` | host harness: numpy oracle for the kernel, stubbed engine smoke test |
| `tools/dspi_test/tests/rta.py` | device tests (section 9) |

Touch points in existing files: `config.h` (command codes), `audio_pipeline.c`
and `pdm_generator.c` (tap calls inside the meter loops, packet begin/end),
`main.c` (`rta_init()`, `rta_service()` in the main loop, `rta_restart()` at
the existing reset sites), `vendor_commands.c` (handlers, `vendor_is_bulk_get`,
the SET's source passed to `rta_set_config()`), `notify.h`/`notify.c`
(`NOTIFY_EVT_RTA_CONFIG`, `notify_push_rta_config()`), `CMakeLists.txt`,
`scripts/check_ram_placement.py` (hot symbols).

## 9. Device tests

Using the signal generator on the output tap and USB playback on the input tap:

- A 1 kHz sine at -20 dBFS on one output reads -20 dBFS within 1 dB in the
  1 kHz band of that output and below -60 dBFS in every band two or more away.
- Pink noise on all outputs reads flat within 2 dB across resolved FFT bands after
  2 s of averaging, on every selected channel.
- With all outputs selected, `seq` advances on every one of them and `age_ms`
  never exceeds 1.5x the expected rotation interval.
- `busy_us_per_s` stays under budget at every order.
- With the analyser running, the existing output alignment tests pass
  unchanged.
- Stop polling for 6 s: `state` returns to IDLE. One band read restarts it.
- A chunked bin read validates seq head against tail and re-reads on mismatch.
- An empty mask or an out-of-range order is rejected.

## 10. Deferred

- Transfer-function mode (input versus output, live measured response).
- A third tap point between the matrix and the output EQ.
- A control-surface OLED spectrum page.
- Capture-buffer overlay onto `bulk_param_buf`.
- Block floating point in the Q15 kernel, which would recover up to 6 dB per
  bit on quiet captures (needs an out-parameter on `rta_fft_step`).
- DSPi Console RTA and FFT views (separate repository, matched release).
