# I2S Input Specification

## 1. Overview

DSPi supports three audio input sources: **USB** (0), **S/PDIF** (1), and **I2S** (2). The input source is selected at runtime via vendor USB control transfers. When I2S is the active source, received audio feeds through the identical DSP pipeline (preamp, loudness, master EQ, leveller, crossfeed, matrix mixer, per-output EQ, delay) as USB and S/PDIF audio.

I2S input is **Pico-master**: the RP2040/RP2350 generates BCK and LRCLK from `sys_clk` via PIO, and the external I2S source (typically a stereo ADC) clocks its data on those lines. Because the input and output PIO state machines are all driven by the same crystal through identical 24.8 fixed-point dividers, the input rate is **always exactly equal** to the output rate by construction. There is no clock servo, no resampler, and no fill trim — and no risk of input/output drift.

I2S input may coexist with any combination of I2S TX outputs (Phase 2.5 lifted the earlier mutual-exclusion restriction).

Three vendor commands are I2S-input-specific, and two more govern input source selection (shared with S/PDIF input):

| Code | Name | Direction | Description |
|------|------|-----------|-------------|
| 0xE0 | `REQ_SET_INPUT_SOURCE` | OUT | Switch among USB / S/PDIF / I2S |
| 0xE1 | `REQ_GET_INPUT_SOURCE` | IN | Query the current input source |
| 0xCA | `REQ_SET_I2S_DIN_PIN` | SET (immediate response) | Configure I2S DIN GPIO pin |
| 0xCB | `REQ_GET_I2S_DIN_PIN` | IN | Query current I2S DIN GPIO pin |
| 0xCC | `REQ_GET_I2S_INPUT_STATUS` | IN | Query 16-byte I2S receiver status struct |

The input source enum and related defaults are in `audio_input.h`:

```c
typedef enum {
    INPUT_SOURCE_USB   = 0,
    INPUT_SOURCE_SPDIF = 1,
    INPUT_SOURCE_I2S   = 2,
    // Future: INPUT_SOURCE_ADAT = 3
} InputSource;

#define INPUT_SOURCE_MAX            INPUT_SOURCE_I2S
#define PICO_I2S_DIN_PIN_DEFAULT    18
```

Related I2S clock pins (shared with I2S **output**, defined in `config.h`):

```c
#define PICO_I2S_BCK_PIN   14   // Bit clock; LRCLK = BCK + 1 = GPIO 15
#define PICO_I2S_MCK_PIN   13   // Master clock (optional, vendor-controlled)
```

---

## 2. State Machine

The I2S receiver state machine is defined in `i2s_input.h` as `I2sInputState`:

```c
typedef enum {
    I2S_INPUT_INACTIVE = 0,   // RX hardware stopped (input source != I2S)
    I2S_INPUT_STARTING = 1,   // SM started, waiting for prefill to reach 50%
    I2S_INPUT_ACTIVE   = 2,   // Streaming, output unmuted
} I2sInputState;
```

There is **no** RELOCKING / unsupported-rate state. Pico-master means the receiver cannot lose lock — if the source ADC is connected and powered, samples flow; if not, the DMA ring fills with silence (no transitions detected at LRCLK).

### State Transitions

```
                          i2s_input_start()
                    +------------------------------+
                    |                              v
             +-----------+                  +-----------+
             |           |  i2s_input_      |           |
             | INACTIVE  |---start()------->| STARTING  |
             |  (0)      |                  |   (1)     |
             +-----------+                  +-----+-----+
                  ^                               |
                  | i2s_input_stop()               | consumer slot 0
                  | (input source change)          | fill >= 50%
                  |                               | (main-loop polling
                  |                               |  block calls
                  |                               |  i2s_input_mark_active())
                  |                               v
                  |                         +-----------+
                  +-------------------------|  ACTIVE   |
                                            |   (2)     |
                                            +-----------+
```

### State Descriptions

- **INACTIVE (0):** RX hardware is stopped. This is the state when USB or S/PDIF is the active input, or the device has just booted with a non-I2S input source. No PIO or DMA resources are claimed by the I2S input subsystem (the I2S clock domain may still be active if an I2S TX output is registered).

- **STARTING (1):** `i2s_input_start()` has been called. The library has registered the RX instance and run its clock-domain election. If no I2S TX is registered, a "phantom" clocks-only master SM has been installed to drive BCK/LRCLK; if an I2S TX is already master, the RX simply attaches as observer. The RX SM has been enabled; it self-syncs on the next LRCLK low edge and DMA begins writing decoded subframes into the ring. Audio is **muted** during this state — outputs are drained and consumer pools are being prefilled with the new I2S audio.

- **ACTIVE (2):** Audio data is being extracted from the DMA ring and fed through the DSP pipeline. The state machine entered ACTIVE when consumer slot 0's fill reached 50% — at that point the main loop called `enable_outputs_in_sync()` and `i2s_input_mark_active()`. Output is unmuted and audio is flowing.

### Transitions

| From | To | Trigger | What happens |
|------|----|---------|-------------|
| INACTIVE | STARTING | `i2s_input_start()` (input source set to I2S) | Library elects clock domain (installs phantom if no TX), claims RX SM + DMA, arms ring, enables RX SM, outputs muted |
| STARTING | ACTIVE | Consumer slot 0 fill >= 50% (main-loop polling) | `enable_outputs_in_sync()`, `i2s_input_mark_active()`, output unmutes |
| Any | INACTIVE | `i2s_input_stop()` (input source changed away from I2S) | RX SM/DMA torn down, library uninstalls phantom if last I2S participant, ring read index reset |

### Timing

| Event | Duration |
|-------|----------|
| RX setup + SM start | < 1 ms |
| First LRCLK edge after SM enable | < 1 LRCLK period (~21 µs at 48 kHz) |
| `i2s_input_poll` batch threshold | 192 stereo pairs (~4 ms at 48 kHz) before `process_input_block` is called |
| Prefill to `I2S_PREFILL_BUFFER_COUNT` (default 6 of 16 consumer buffers) | ~6 ms at 48 kHz |
| Total USB-to-I2S switch time (`REQ_SET_INPUT_SOURCE`) | ~25-40 ms |
| I2S-to-USB switch time | < 10 ms (immediate stop + flush + unmute) |
| Steady-state input latency (DIN edge → DSP entry) | ~4 ms (one batch boundary) |
| Steady-state output latency (consumer pool oscillation) | ~4 ms (avg pool fill at I2S threshold) |

Compared to S/PDIF, I2S input switches **much** faster because there is no library lock-acquisition phase (~64 ms for SPDIF) and no firmware lock debounce period. Steady-state input latency matches SPDIF (one block / one batch ≈ 4 ms at 48 kHz). Steady-state output latency is **lower** than SPDIF (~4 ms vs ~6 ms) because I2S Pico-master needs less consumer-pool jitter margin — there's no clock servo correction or FIFO-jitter slack to absorb.

### Tuning the I2S prefill threshold

`I2S_PREFILL_BUFFER_COUNT` in `config.h` controls how many consumer buffers (out of 16 total) must be filled before outputs are enabled after switching to I2S input. The pool naturally oscillates ±4 buffers (one input batch = 192 samples = 4 buffers), so the prefill threshold sets where the oscillation is centred:

| `I2S_PREFILL_BUFFER_COUNT` | Pool oscillation | Avg latency | Jitter margin |
|----------------------------|------------------|-------------|---------------|
| 8 (50%) | 4-8 buffers | ~6 ms | 4 ms |
| **6 (37%, default)** | **2-6 buffers** | **~4 ms** | **2 ms** |
| 4 (25%) | 0-4 buffers | ~2 ms | 0 ms (silence buffer on any late batch) |

The default (6) trades 2 ms of latency for 2 ms of jitter margin — robust against typical main-loop work (vendor commands, USB control transfers, DSP processing for the prior batch). Reducing to 4 saves another 2 ms but leaves no margin: any main-loop delay ≥ 1 ms past the batch boundary causes the audio library to insert a silence buffer (~1 ms audible click). Test on target hardware before going below 6.

---

## 3. Vendor Commands

All commands use the standard DSPi vendor control transfer format:

- **bmRequestType:** `0x41` (Host-to-Device SET) or `0xC1` (Device-to-Host GET)
- **wIndex:** `2` (vendor interface number, `VENDOR_INTERFACE_NUMBER`)
- **Timeout:** 1000 ms recommended for all commands

---

### 3.1 REQ_SET_INPUT_SOURCE (0xE0)

Switch the active input source. Same command as for S/PDIF; the value `2` selects I2S.

**Direction:** Host -> Device (SET)
**bmRequestType:** `0x41`
**bRequest:** `0xE0`
**wValue:** `0` (unused)
**wLength:** `1`

#### Request payload (1 byte)

| Offset | Size | Type | Field | Values |
|--------|------|------|-------|--------|
| 0 | 1 | uint8_t | `source` | `0` = USB, `1` = S/PDIF, `2` = I2S |

#### Behavior

The command returns immediately (non-blocking). The actual source switch is deferred to the firmware main loop:

1. Firmware validates the source value (0-2) and checks it differs from the current source.
2. If valid and different, sets `pending_input_source = src` and `input_source_change_pending = true`.
3. Main loop detects the flag and runs the switch sequence:
   - Drains the USB audio ring (if old source was USB)
   - Mutes output for 256 samples (~5 ms at 48 kHz) via `prepare_pipeline_reset()`
   - Stops old source hardware:
     - If old source was SPDIF: `spdif_input_stop()`, `perform_rate_change(audio_state.freq)` to restore nominal output dividers (the SPDIF servo had been adjusting them)
     - If old source was I2S: `i2s_input_stop()` (no rate-change restore needed — Pico-master, dividers were already at nominal)
   - Updates `active_input_source` to the new value
   - Starts new source hardware:
     - I2S: `i2s_input_start()` — outputs stay muted until prefill (handled by the main-loop I2S polling block)
     - SPDIF: `spdif_input_start()` — outputs stay muted until lock acquisition
     - USB: flushes stale ring data, calls `complete_pipeline_reset()`
   - Calls `update_mck_for_i2s_state()` to enable/disable the MCK PIO SM if the I2S participant set changed
   - Emits a `notify_param_write` for `input_config.input_source` so the host gets a v2 notification

#### Error handling

| Condition | Firmware behavior |
|-----------|-------------------|
| `source > 2` (invalid) | Ignored silently, no action |
| `source == active_input_source` | Ignored silently, no action |
| No I2S clock signal connected | Switch proceeds; STARTING never advances to ACTIVE because no samples reach the ring |
| Source ADC running at a rate other than `audio_state.freq` | Switch proceeds; samples are decoded as-if the rate matched. **The host is responsible** for keeping the source ADC rate in sync with `audio_state.freq` (changes at USB rate-change events). See section 6. |

There is no error response. The command always ACKs successfully. Use `REQ_GET_I2S_INPUT_STATUS` to monitor the outcome.

#### Hex example

Switch to I2S input:
```
bmRequestType: 0x41
bRequest:      0xE0
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0001
Data:          02
```

---

### 3.2 REQ_GET_INPUT_SOURCE (0xE1)

Query the currently active input source.

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0xE1`
**wValue:** `0` (unused)
**wLength:** `1`

#### Response (1 byte)

| Offset | Size | Type | Field | Values |
|--------|------|------|-------|--------|
| 0 | 1 | uint8_t | `source` | `0` = USB, `1` = S/PDIF, `2` = I2S |

#### Notes

- Returns the current value of `active_input_source`.
- If a source switch is pending (deferred to main loop), this returns the source that was active **before** the switch.
- At boot, defaults to whatever `INPUT_SOURCE_*` value the loaded preset specifies (V13+ slots), or `0` (USB) on a fresh device.

---

### 3.3 REQ_SET_I2S_DIN_PIN (0xCA)

Set the GPIO pin used for I2S input (DIN — data input). This is a **device-level** setting stored in the preset directory (not per-preset).

**Direction:** Host -> Device (GET-style immediate response)
**bmRequestType:** `0xC1`
**bRequest:** `0xCA`
**wValue:** New GPIO pin number
**wLength:** `1`

Note: Despite being a "SET" command, this uses a GET-direction transfer (`0xC1`) because it returns an immediate status byte. The pin number is passed in `wValue`, not in the request body. This follows the same pattern as `REQ_SET_SPDIF_RX_PIN`, `REQ_SET_I2S_BCK_PIN`, and other pin-configuration commands.

#### Response (1 byte)

| Value | Name | Meaning |
|-------|------|---------|
| `0x00` | `PIN_CONFIG_SUCCESS` | Pin changed (or already set to this value) |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | Pin is out of range or reserved |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | Pin is already used by an output, BCK/LRCLK, MCK, SPDIF RX, or another reserved function |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | Cannot change pin while I2S input is active |

#### Validation logic (`vendor_commands.c:1709`)

```c
case REQ_SET_I2S_DIN_PIN: {
    uint8_t new_pin = (uint8_t)setup->wValue;
    uint8_t status;
    if (!is_valid_gpio_pin(new_pin)) {
        status = PIN_CONFIG_INVALID_PIN;
    } else if (new_pin == i2s_din_pin) {
        status = PIN_CONFIG_SUCCESS;             // No-op
    } else if (active_input_source == INPUT_SOURCE_I2S) {
        status = PIN_CONFIG_OUTPUT_ACTIVE;       // Can't change while RX active
    } else if (is_pin_in_use(new_pin, 0xFF)) {
        status = PIN_CONFIG_PIN_IN_USE;
    } else {
        i2s_din_pin = new_pin;
        flash_set_i2s_din_pin_pending = true;    // Deferred flash write
        status = PIN_CONFIG_SUCCESS;
        notify_param_write(offsetof(WireBulkParams, input_config.i2s_din_pin),
                           1, &i2s_din_pin);
    }
    resp_buf[0] = status;
    vendor_send_response(resp_buf, 1);
    return true;
}
```

#### GPIO pin constraints

Valid GPIO pins must satisfy ALL of the following:

- **Range:** 0-28 (RP2040) or 0-29 (RP2350)
- **Not reserved:** GPIO 12 (UART TX), GPIO 23-25 (power/LED) are always excluded
- **Not in use:** Must not conflict with any S/PDIF output data pin, I2S TX data pin, I2S BCK or LRCLK (BCK+1), MCK pin, S/PDIF RX pin, or PDM output pin
- **Not active:** I2S input must be INACTIVE (switch to USB or S/PDIF first before changing the pin)

The pin setting persists across reboots (stored in the `PresetDirectory` v3 sector of flash). The default pin is GPIO 18 (`PICO_I2S_DIN_PIN_DEFAULT`).

#### Hex example

Set I2S DIN pin to GPIO 19:
```
bmRequestType: 0xC1
bRequest:      0xCA
wValue:        0x0013   (19)
wIndex:        0x0002
wLength:       0x0001
Response:      00       (SUCCESS)
```

Attempt to set pin while I2S input is active:
```
bmRequestType: 0xC1
bRequest:      0xCA
wValue:        0x0013
wIndex:        0x0002
wLength:       0x0001
Response:      04       (OUTPUT_ACTIVE)
```

---

### 3.4 REQ_GET_I2S_DIN_PIN (0xCB)

Query the current GPIO pin configured for I2S input.

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0xCB`
**wValue:** `0` (unused)
**wLength:** `1`

#### Response (1 byte)

| Offset | Size | Type | Field | Values |
|--------|------|------|-------|--------|
| 0 | 1 | uint8_t | `pin` | GPIO pin number (default: 18) |

#### Hex example

```
bmRequestType: 0xC1
bRequest:      0xCB
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0001
Response:      12       (GPIO 18)
```

---

### 3.5 REQ_GET_I2S_INPUT_STATUS (0xCC)

Query the I2S receiver status. Returns a 16-byte packed struct (sized for symmetry with `SpdifRxStatusPacket`).

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0xCC`
**wValue:** `0` (unused)
**wLength:** `16`

#### Response: I2sInputStatusPacket (16 bytes)

Defined in `i2s_input.h`:

```c
typedef struct __attribute__((packed)) {
    uint8_t  state;              // I2sInputState (0-2)
    uint8_t  input_source;       // Active InputSource enum (0-2)
    uint8_t  din_pin;            // Current DIN GPIO pin
    uint8_t  reserved0;          // Always 0
    uint32_t sample_rate;        // Current Fs in Hz (== audio_state.freq when active)
    uint32_t ring_fill_words;    // Words available in ring (write_idx - read_idx)
    uint16_t ring_size_words;    // Total ring size in 32-bit words (always 2048)
    uint16_t reserved1;          // Always 0
} I2sInputStatusPacket;          // 16 bytes
```

All multi-byte fields are **little-endian** (native ARM Cortex-M0+/M33 byte order).

#### Byte layout

```
Offset: 00 01 02 03  04 05 06 07  08 09 0A 0B  0C 0D 0E 0F
Field:  ST IS DP r0  SR SR SR SR  RF RF RF RF  RS RS r1 r1

ST = state              IS = input_source     DP = din_pin
SR = sample_rate (LE)   RF = ring_fill_words (LE)
RS = ring_size_words (LE)  r0/r1 = reserved (zero)
```

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 1 | uint8_t | `state` | `I2sInputState` enum (0-2) |
| 1 | 1 | uint8_t | `input_source` | Current `InputSource` enum (`0`=USB, `1`=SPDIF, `2`=I2S) |
| 2 | 1 | uint8_t | `din_pin` | Current DIN GPIO pin (default 18) |
| 3 | 1 | uint8_t | `reserved0` | Always 0 |
| 4 | 4 | uint32_t LE | `sample_rate` | Current sample rate in Hz (mirrors `audio_state.freq` when active) |
| 8 | 4 | uint32_t LE | `ring_fill_words` | Words currently available in the DMA ring (write_idx - read_idx, mod ring size). 0 when INACTIVE. |
| 12 | 2 | uint16_t LE | `ring_size_words` | Ring size in 32-bit words (always `2048` = 8 KB) |
| 14 | 2 | uint16_t LE | `reserved1` | Always 0 |

#### State values

| Value | Name | Description |
|-------|------|-------------|
| 0 | `INACTIVE` | RX hardware stopped (not the active input source) |
| 1 | `STARTING` | RX SM running, prefilling consumer pools — output muted |
| 2 | `ACTIVE` | Streaming — output unmuted |

#### `ring_fill_words` interpretation

The DMA write-ring continuously writes 32-bit words (one per I2S subframe = one channel-sample). The consumer (`i2s_input_poll()`) reads at the same rate that DSP processing consumes, with the goal of keeping the ring nearly empty.

| `ring_fill_words` | Meaning |
|-------------------|---------|
| 0 | Ring just drained; ADC is silent or no source connected (and you're INACTIVE) |
| 2-32 | Healthy operation — consumer keeps up with producer |
| > 64 | Consumer falling behind (rare; only possible during heavy main-loop work) |
| 1024 (50% of ring) | Boundary case — should never reach this in normal operation |
| 2046+ | Imminent overrun — diagnostic of bug |

The ring size is power-of-two (2048 words = 8 KB) so `(write_idx - read_idx) & (ring_size - 1)` always yields the live fill count even after wrap.

#### Example response (hex)

ACTIVE at 48 kHz on default pin, ring fill 4 words:
```
02 02 12 00  80 BB 00 00  04 00 00 00  00 08 00 00
```

Breakdown:
- `02` = ACTIVE
- `02` = input_source is I2S
- `12` = DIN on GPIO 18
- `00` = reserved0
- `80 BB 00 00` = 48000 (0x0000BB80 LE)
- `04 00 00 00` = 4 words in ring
- `00 08` = 2048 ring size (0x0800 LE)
- `00 00` = reserved1

INACTIVE (USB is the active source):
```
00 00 12 00  80 BB 00 00  00 00 00 00  00 08 00 00
```

---

## 4. Audio Data Format

### Wire format (I2S frame)

Each I2S frame contains two 32-bit slots (left, right). Audio data is **24-bit, signed, MSB-first, left-justified** within each 32-bit slot. The bottom 8 bits of each slot are zero-padded.

```
Bit:   31  30  29  28  27 ... 9   8   7   6   5   4   3   2   1   0
       |<------ 24-bit signed audio sample (MSB-first) ------>|  |<-- pad bits = 0 -->|
```

This layout matches mainstream I2S ADCs (e.g. PCM1808, ES7243, AKM AK5720) configured for "I2S-standard" or "left-justified" mode with 24-bit data. It is *not* the same as DSP-mode or right-justified frame formats; the source ADC must be configured for I2S-standard timing.

### Sample timing

The PIO RX program (`audio_i2s_datain.pio`) waits for a falling edge of LRCLK to start a frame, then samples DIN on rising BCK edges. Each 32-bit slot is shifted in MSB-first. The library patches the `wait gpio` operands at runtime so any BCK/LRCLK pin pair is supported.

### Sample extraction (consumer side)

The consumer reads 32-bit words from the ring buffer:

```c
uint32_t word_l = i2s_rx_inst.ring[r];
uint32_t word_r = i2s_rx_inst.ring[(r + 1) & ring_mask];

// Mask off the bottom 8 zero-pad bits, leaving a 24-bit MSB-justified sample
// in the upper bits. Casting to int32_t sign-extends naturally.
int32_t raw_l = (int32_t)(word_l & 0xFFFFFF00u);
int32_t raw_r = (int32_t)(word_r & 0xFFFFFF00u);
```

The resulting `raw_l` / `raw_r` are signed 32-bit integers where:

- `+2,147,483,647` (`0x7FFFFF00` after mask) = positive full-scale (24-bit `+0x7FFFFF`)
- `-2,147,483,648` (`0x80000000` after mask) = negative full-scale (24-bit `-0x800000`)
- `0` = silence

### Firmware conversion to internal format

#### RP2350 (float)

```c
const float inv_2147483648 = 1.0f / 2147483648.0f;
buf_l[sample_idx] = (float)raw_l * inv_2147483648 * preamp_l;
buf_r[sample_idx] = (float)raw_r * inv_2147483648 * preamp_r;
```

Converts int32 full-scale to the [-1.0, +1.0] float range, then applies per-channel preamp (`global_preamp_linear[]`).

#### RP2040 (Q28 fixed-point)

```c
int32_t q28_l = raw_l >> 4;                          // int32 full-scale -> Q28
int32_t q28_r = raw_r >> 4;
buf_l[sample_idx] = fast_mul_q28(q28_l, preamp_l);   // apply preamp in Q28
buf_r[sample_idx] = fast_mul_q28(q28_r, preamp_r);
```

Right-shifts by 4 to convert from 32-bit full-scale to Q28 (28 fractional bits), then multiplies by the Q28 preamp gain (`global_preamp_mul[]`).

These are the **same** conversions used by the S/PDIF input path — the only difference is the source mask (S/PDIF uses `0x0FFFFFF0u << 4`; I2S uses `0xFFFFFF00u`).

---

## 5. Clocking (No Servo)

I2S input is **Pico-master**: BCK and LRCLK are generated by the Pico's PIO from `sys_clk` using the same divider math as the I2S TX outputs.

### Clock derivation

```
sys_clk (307.2 MHz)
   │
   ├─────┐ (clkdiv 24.8)
   │     │
   │  ┌──▼─────────────────────────┐
   │  │ I2S clkout SM (TX master   │
   │  │  or "phantom" clkout-only) │
   │  └──┬───────────────────┬─────┘
   │     │                   │
   │   BCK pin            LRCLK pin
   │   (=Fs × 64)         (=Fs)
   │
   ├─────┐ (same clkdiv 24.8)
   │     │
   │  ┌──▼─────────────────────────┐
   │  │ I2S RX SM (samples DIN     │
   │  │  on BCK edges)             │
   │  └────────────────────────────┘
   │
   └─────┐ (sys_clk × 128 / (Fs × multiplier))
         │
      ┌──▼──────────────────────┐
      │ MCK SM (optional)       │
      └─────────────────────────┘
```

- **BCK** = sample_rate × 64 (32 bits per slot × 2 slots stereo)
- **LRCLK** = sample_rate
- **MCK** = sample_rate × multiplier (128× or 256×, optional, governed by `REQ_SET_MCK_*` commands)
- **PIO clkdiv** (for the I2S clkout/clkout_only SM) = `sys_clk × 2 / Fs` = `307200000 × 2 / 48000` = `12800.000` → exact integer at 48 kHz; 96 kHz also exact. 44.1 kHz uses the fractional part of the 24.8 divider.

Because the input RX SM and output TX SMs (and any clkout-only phantom) all derive from the same `sys_clk` through identical `clkdiv` values, **input rate is locked to output rate by hardware**. There is no possibility of FIFO drift, overflow, or underflow.

### What this means for the host

- No clock servo to monitor.
- No FIFO-fill-level watching for drift.
- No "lock loss / re-lock" events to handle.
- The only way the input "loses lock" is if you call `i2s_input_stop()` (changing the input source).

### What this means for the source ADC

The external ADC must:

1. Be configured as an **I2S slave** (ADC accepts BCK/LRCLK from Pico, drives DIN out).
2. Be in **I2S-standard mode**, 24-bit data (not left-justified, not right-justified, not DSP mode — those have different LRCLK alignment).
3. Run at exactly the sample rate the Pico is generating (`audio_state.freq`).
4. Be powered up and ready before `REQ_SET_INPUT_SOURCE` switches DSPi into I2S mode (the prefill phase needs samples flowing).

Most modern audio ADCs (PCM1808, ES7243, AKM AK5720, etc.) auto-detect their input clock and adjust their internal divider, so all you need to do is route BCK/LRCLK from the configured Pico pins to the ADC. No host-side ADC control is required (DSPi does not currently support I2C/SPI control of external ADCs).

If the ADC is missing or misconfigured (e.g. running in left-justified mode), DSPi will still enter the ACTIVE state — the polling block only checks for prefill, not for valid audio. The audio will be silence or distorted depending on the misconfiguration. Diagnose by listening on a known-good output and checking `ring_fill_words` for steady non-zero values.

---

## 6. Sample Rate Handling

### Supported rates

| Rate | Supported | Notes |
|------|-----------|-------|
| 44,100 Hz | Yes | CD audio |
| 48,000 Hz | Yes | Standard (default for USB) |
| 96,000 Hz | Yes | High-resolution |

These are the same three rates supported elsewhere in DSPi. The PIO clkdiv is exact for 48 kHz (12800.000) and 96 kHz (6400.000) at the standard 307.2 MHz sys_clk; 44.1 kHz uses the fractional 24.8 divider.

### Rate change handling

The I2S input rate is **always** equal to `audio_state.freq` (the host-driven USB rate). When the USB host changes its sample rate (e.g. switching between 44.1 kHz and 48 kHz content):

1. USB stack signals a rate change via the `pending_rate` mechanism.
2. Main loop calls `perform_rate_change(new_freq)`:
   - Updates all output PIO dividers (SPDIF + I2S TX) to match `new_freq`
   - Calls `audio_i2s_clock_domain_clkout_only_set_frequency(new_freq)` if the phantom clkout-only master is active (i.e. I2S input is the only I2S participant)
   - Calls `i2s_input_set_frequency(new_freq)` to update the RX SM divider in lockstep with the BCK/LRCLK update
   - Recalculates all DSP filter coefficients for the new rate
3. The host's external I2S ADC must independently adjust its internal divider to match the new BCK rate. Most ADCs do this automatically (they slave to incoming BCK).

There is **no** rate-change notification to the I2S source; the source is expected to follow the master clock automatically.

If the I2S source cannot follow rate changes (e.g. it's a fixed-rate device hardwired at 48 kHz), the host application should ensure the USB rate stays at the same fixed rate. Mixing different USB and I2S rates is **not supported** — the source ADC must always run at the same rate as `audio_state.freq`.

---

## 7. Errors & Recovery

I2S input has far fewer failure modes than S/PDIF input because there is no signal-detection logic and no clock servo.

| Condition | Detection | State | Output | App recommendation |
|-----------|-----------|-------|--------|-------------------|
| **No I2S source connected** | `ring_fill_words` stays at 0 indefinitely after switch | Stuck in STARTING (1) | Muted (prefill never completes) | Show "Connecting I2S source...", warn after 5 s timeout |
| **ADC misconfigured (wrong frame format)** | ACTIVE reached but audio is distorted/silent | ACTIVE (2) | Distorted/silent | Verify ADC is in I2S-standard 24-bit mode |
| **Wiring fault on DIN pin** | Same as no source — `ring_fill_words` is 0 | Stuck in STARTING | Muted | Verify cable, GPIO assignment via `REQ_GET_I2S_DIN_PIN` |
| **Source ADC running at wrong rate** | Audio plays back pitch-shifted | ACTIVE | Pitch-shifted | Check ADC is slaved to BCK and not running at fixed rate |
| **Pin collision attempted via SET_DIN_PIN** | Status response = `0x02` (PIN_IN_USE) | Unchanged | N/A | Show error, surface conflicting use |
| **DIN pin change attempted while active** | Status response = `0x04` (OUTPUT_ACTIVE) | Unchanged | N/A | Switch source to USB first, then change pin, then switch back |

### Bring-up checklist for app developers

When a user reports "no audio on I2S input":

1. **Check `state`** via `REQ_GET_I2S_INPUT_STATUS`. If `INACTIVE`, the source switch hasn't completed — call `REQ_GET_INPUT_SOURCE` to confirm `active_input_source == 2`.
2. **Check `ring_fill_words`**. If 0 in STARTING/ACTIVE, the ADC is not delivering samples. Likely causes: ADC not powered, no BCK reaching ADC, ADC in slave mode but no DIN driver, wrong pin.
3. **Check `din_pin`** matches the user's wiring. Default is GPIO 18; set via `REQ_SET_I2S_DIN_PIN`.
4. **Check BCK pin**. Even though I2S input doesn't have its own BCK pin, it shares the same BCK pin as I2S TX outputs. Query via `REQ_GET_I2S_BCK_PIN` (0xC3). Default is GPIO 14, with LRCLK on GPIO 15.
5. **Check `sample_rate`** matches what the ADC is configured to expect (or what it is auto-detecting). DSPi sets this from the USB host rate.
6. **Listen on a known-good output**. The DSP pipeline is identical for all input sources, so if SPDIF/USB sound right and I2S is silent, the issue is upstream of the DSP (i.e. on the ADC).

---

## 8. GPIO Constraints

### DIN pin

The DIN (data input) pin is assignable by the host and follows the same validation rules as all DSPi GPIO pin assignments.

#### Valid pin range

| Platform | Valid range |
|----------|------------|
| RP2040 | GPIO 0-28 |
| RP2350 | GPIO 0-29 |

#### Always excluded

| GPIO | Reason |
|------|--------|
| 12 | UART TX (debug console) |
| 23 | Power control (SMPS mode) |
| 24 | Power control (VBUS detect) |
| 25 | On-board LED |

#### Must not conflict with (checked by `is_pin_in_use()`)

- Any S/PDIF or I2S TX **output data pin** (`output_pins[0..NUM_PIN_OUTPUTS-1]`)
- I2S **BCK** pin or **LRCLK** pin (BCK + 1) — these are reserved unconditionally even when no I2S participant is active, because the clock domain may install a phantom on them at any time
- I2S **MCK** pin, if MCK is enabled
- S/PDIF RX pin (`spdif_rx_pin`)
- The current I2S DIN pin itself (no-op SUCCESS if same)

The firmware checks all of these via `is_pin_in_use(new_pin, 0xFF)` before allowing the change.

#### Pin change restrictions

- The pin **cannot** be changed while I2S input is active (state != INACTIVE). Switch to USB or S/PDIF first, change the pin, then switch back to I2S.
- If the pin is already set to the requested value, the command returns `SUCCESS` as a no-op.

#### Persistence

The DIN pin is stored in the `PresetDirectory` (device-level, **not** per-preset). It persists across reboots and preset changes. On first boot (no directory), it defaults to GPIO 18.

### BCK / LRCLK pins

I2S input shares the same BCK/LRCLK pins as I2S **output** — they are jointly owned by the I2S "clock domain" inside the `pico_audio_i2s_multi` library. The library elects which PIO state machine drives BCK/LRCLK based on the registered participants:

| Registered I2S participants | BCK/LRCLK driver | Library program | Notes |
|-----------------------------|------------------|-----------------|-------|
| 1+ TX (any RX) | `i2s_instances[0]` (TX SM) | `audio_i2s_clkout` | Combined clocks-and-data SM |
| 0 TX, 1+ RX | "Phantom" clocks-only SM | `audio_i2s_clkout_only` | Installed automatically when I2S input is enabled standalone |
| 0 TX, 0 RX | None | — | BCK/LRCLK pins are released to high-Z |

The phantom is automatically installed when `i2s_input_start()` runs and no I2S TX is registered. It is automatically uninstalled when `i2s_input_stop()` runs and no I2S TX remains. This is internal library bookkeeping — the host has no direct control or visibility.

The BCK pin is configured via `REQ_SET_I2S_BCK_PIN` (0xC2). Default is GPIO 14 (with LRCLK on GPIO 15). It cannot be changed while any I2S participant (TX **or** RX) is active.

### MCK pin

The MCK (master clock) is optional. When enabled (`REQ_SET_MCK_ENABLE`), it generates an MCK signal on a dedicated PIO SM (PIO1 SM1) at sample_rate × 128 or × 256. MCK is independent of the data path but shares the I2S clock domain's enable lifecycle: if any I2S participant (TX or RX) is active, MCK is generated; if no I2S is active, MCK is disabled.

The MCK pin is configured via `REQ_SET_MCK_PIN` (0xC6). Default is GPIO 13.

---

## 9. Preset Integration

### Flash storage

The active input source is stored per-preset (`PresetSlot.input_source` at `SLOT_DATA_VERSION` 13+). Loading a preset that specifies I2S as its input source schedules an automatic input-source switch to I2S after the preset load completes.

The DIN pin is **device-level** — stored in the `PresetDirectory` (sector 0), not in per-preset slots:

```c
typedef struct __attribute__((packed)) {
    // ... existing fields ...
    // V3 additions
    uint8_t  i2s_din_pin;                    // I2S DIN GPIO pin, device-level
    uint8_t  v3_padding[3];                  // Reserved
} PresetDirectory;

#define DIR_VERSION_CURRENT  3
```

| Action | Behavior on `i2s_din_pin` | Behavior on `input_source` |
|--------|---------------------------|----------------------------|
| **Save preset** | Not stored in slot | Current `active_input_source` is stored in the slot |
| **Load preset** | Unchanged (device-level) | If slot version ≥ 13, defers an input source switch |
| **Factory reset** | Reset to default (GPIO 18) and persisted | Reset to `INPUT_SOURCE_USB` (0) |
| **Boot with no preset** | Loaded from directory; defaults to GPIO 18 | Defaults to USB |

### Directory migration

| From | To | Action |
|------|----|--------|
| v1 (legacy single-sector) | v3 | `i2s_din_pin = PICO_I2S_DIN_PIN_DEFAULT` |
| v2 (current up to commit `4e3a129`) | v3 | `i2s_din_pin = PICO_I2S_DIN_PIN_DEFAULT` (new field) |
| v3 | v3 | No migration |

Migration is automatic on first boot of post-stash firmware against an older directory. The directory is rewritten in v3 format and the original is overwritten (the old data is no longer accessible — there is no "downgrade" path).

### Wire format (`WIRE_FORMAT_VERSION` = 7)

The bulk parameter transfer's `WireInputConfig` section (16 bytes, V7+) carries the I2S DIN pin alongside the SPDIF RX pin and active input source:

```c
typedef struct __attribute__((packed)) {
    uint8_t  input_source;       // InputSource enum (0=USB, 1=SPDIF, 2=I2S)
    uint8_t  spdif_rx_pin;       // SPDIF RX GPIO pin (slot-level, applied on SET when apply_pins=true)
    uint8_t  i2s_din_pin;        // I2S DIN GPIO pin (device-level; informational on SET)
    uint8_t  reserved[13];       // Future expansion (pad to 16 bytes)
} WireInputConfig;               // 16 bytes
```

| Field | GET behavior | SET behavior |
|-------|--------------|--------------|
| `input_source` | Mirrors `active_input_source` | If different from current, schedules an input-source switch (deferred to main loop) |
| `spdif_rx_pin` | Mirrors `spdif_rx_pin` | Applied if `apply_pins == true` (slot-level pin convention since commit `4e3a129`) |
| `i2s_din_pin` | Mirrors `i2s_din_pin` (device-level) | **Not applied on SET** — informational only. To change the DIN pin via wire, use `REQ_SET_I2S_DIN_PIN` (the device-level setter). |

The `i2s_din_pin` field claims one byte from `WireInputConfig.reserved[]` without bumping `WIRE_FORMAT_VERSION` (older firmware reads it as zero, which the I2S input subsystem treats as "use device default").

### Mute during preset load

When a preset is loaded that switches the input source to I2S, the standard 256-sample (~5 ms) mute envelope applies during the pipeline reset. After `i2s_input_start()` brings up the RX hardware, additional silent time elapses during the prefill phase (~16 ms at 48 kHz). Total perceptible silence is typically ~25-50 ms.

---

## 10. Platform Differences

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| I2S input PIO block | PIO1 | PIO2 |
| I2S input PIO SM (RX) | SM2 | SM0 |
| Clocks-only "phantom" master PIO block | PIO0 | PIO2 |
| Clocks-only phantom PIO SM | SM2 | SM1 |
| RX DMA channel | 7 | 7 |
| RX DMA IRQ | DMA_IRQ_0 (shared with I2S TX) | DMA_IRQ_0 (shared with I2S TX) |
| Internal audio format | Q28 fixed-point (32-bit) | IEEE 754 float |
| Sample conversion | `(raw & 0xFFFFFF00) >> 4` then `fast_mul_q28()` | `(raw & 0xFFFFFF00) * inv_2147483648 * preamp` |
| Supported input rates | 44.1, 48, 96 kHz | 44.1, 48, 96 kHz |
| Input bit depth | 24-bit | 24-bit |
| Default DIN pin | GPIO 18 | GPIO 18 |
| Default BCK pin | GPIO 14 | GPIO 14 |
| Default LRCLK pin | GPIO 15 | GPIO 15 |
| Default MCK pin (when enabled) | GPIO 13 | GPIO 13 |

From `config.h`:

```c
#if PICO_RP2350
#define I2S_CLOCK_DOMAIN_PHANTOM_PIO_IDX  2u  // pio2/SM1
#define I2S_CLOCK_DOMAIN_PHANTOM_SM       1u
#else
#define I2S_CLOCK_DOMAIN_PHANTOM_PIO_IDX  0u  // pio0/SM2
#define I2S_CLOCK_DOMAIN_PHANTOM_SM       2u
#endif
```

Both platforms expose **identical** vendor command interfaces and status struct formats. Application code does not need to differentiate between platforms for I2S input functionality.

### Library context

The new I2S RX support and clock-domain abstraction live in `firmware/pico-extras/src/rp2_common/pico_audio_i2s_multi/`:

- `audio_i2s_multi.c/h` — multi-instance TX (existing) plus new RX API and clock-domain abstraction
- `audio_i2s_clkout.pio` — Combined clocks-and-data master program (existing, used by I2S TX)
- `audio_i2s_clkout_only.pio` — **New.** Clocks-only master program (used when I2S input runs with no I2S TX)
- `audio_i2s_dataout.pio` — Data-only program for slave TX SMs (existing)
- `audio_i2s_datain.pio` — **New.** Data-input program for the RX SM, with `wait gpio` operands patched at runtime to match BCK/LRCLK pins
- `audio_mck.pio` — MCK generator (existing)

---

## 11. Example App Integration

### C struct definitions (for parsing responses)

```c
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

// Input source identifiers
#define INPUT_SOURCE_USB    0
#define INPUT_SOURCE_SPDIF  1
#define INPUT_SOURCE_I2S    2

// I2S receiver states
#define I2S_STATE_INACTIVE  0
#define I2S_STATE_STARTING  1
#define I2S_STATE_ACTIVE    2

// Vendor command request codes
#define REQ_SET_INPUT_SOURCE       0xE0
#define REQ_GET_INPUT_SOURCE       0xE1
#define REQ_SET_I2S_DIN_PIN        0xCA
#define REQ_GET_I2S_DIN_PIN        0xCB
#define REQ_GET_I2S_INPUT_STATUS   0xCC

// Pin configuration status codes
#define PIN_CONFIG_SUCCESS         0x00
#define PIN_CONFIG_INVALID_PIN     0x01
#define PIN_CONFIG_PIN_IN_USE      0x02
#define PIN_CONFIG_OUTPUT_ACTIVE   0x04

// Vendor interface number
#define VENDOR_INTF  2

// I2S input status packet (matches firmware I2sInputStatusPacket exactly)
typedef struct __attribute__((packed)) {
    uint8_t  state;             // I2S_STATE_xxx (0=INACTIVE, 1=STARTING, 2=ACTIVE)
    uint8_t  input_source;      // INPUT_SOURCE_xxx
    uint8_t  din_pin;           // Current DIN GPIO
    uint8_t  reserved0;         // 0
    uint32_t sample_rate;       // Hz, little-endian
    uint32_t ring_fill_words;   // Words in ring (LE) — 0 when INACTIVE
    uint16_t ring_size_words;   // Always 2048 (LE)
    uint16_t reserved1;         // 0
} I2sInputStatusPacket;         // 16 bytes total
```

### Switch to I2S input

```c
int switch_to_i2s(libusb_device_handle *handle) {
    uint8_t source = INPUT_SOURCE_I2S;
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_SET_INPUT_SOURCE,   // 0xE0
        0,                      // wValue (unused)
        VENDOR_INTF,            // wIndex = 2
        &source, 1,             // 1 byte payload
        1000);                  // 1s timeout

    if (ret < 0) {
        fprintf(stderr, "Failed to switch input: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;  // Command accepted (switch is async — poll status to confirm)
}
```

### Poll I2S receiver status

```c
int get_i2s_status(libusb_device_handle *handle, I2sInputStatusPacket *out) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_I2S_INPUT_STATUS,   // 0xCC
        0, VENDOR_INTF,
        (uint8_t *)out, sizeof(I2sInputStatusPacket),
        1000);

    if (ret != (int)sizeof(I2sInputStatusPacket)) {
        fprintf(stderr, "Failed to get I2S status: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}

// Usage: polling loop for a status display (recommended: every 250 ms)
void update_i2s_status_display(libusb_device_handle *handle) {
    I2sInputStatusPacket status;
    if (get_i2s_status(handle, &status) != 0) return;

    const char *state_names[] = { "Inactive", "Starting", "Active" };
    printf("State: %s\n", state_names[status.state & 3]);
    printf("DIN pin: GPIO %u\n", status.din_pin);

    if (status.state != I2S_STATE_INACTIVE) {
        printf("Sample rate: %u Hz\n", status.sample_rate);
        printf("Ring fill: %u / %u words\n",
               status.ring_fill_words, status.ring_size_words);

        if (status.state == I2S_STATE_STARTING && status.ring_fill_words == 0) {
            printf("WARNING: No samples arriving — check ADC connection\n");
        }
    }
}
```

### Configure and query DIN pin

```c
int set_i2s_din_pin(libusb_device_handle *handle, uint8_t pin) {
    uint8_t status;
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_SET_I2S_DIN_PIN,    // 0xCA
        pin,                     // wValue = GPIO pin
        VENDOR_INTF,
        &status, 1, 1000);

    if (ret != 1) {
        fprintf(stderr, "Failed to set DIN pin: %s\n", libusb_error_name(ret));
        return -1;
    }

    switch (status) {
        case PIN_CONFIG_SUCCESS:
            printf("DIN pin set to GPIO %u\n", pin);
            return 0;
        case PIN_CONFIG_INVALID_PIN:
            fprintf(stderr, "GPIO %u is not a valid pin\n", pin);
            return -1;
        case PIN_CONFIG_PIN_IN_USE:
            fprintf(stderr, "GPIO %u is already in use by another function\n", pin);
            return -1;
        case PIN_CONFIG_OUTPUT_ACTIVE:
            fprintf(stderr, "Cannot change DIN pin while I2S input is active\n");
            return -1;
        default:
            fprintf(stderr, "Unknown status: 0x%02X\n", status);
            return -1;
    }
}

int get_i2s_din_pin(libusb_device_handle *handle, uint8_t *out_pin) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_I2S_DIN_PIN,    // 0xCB
        0, VENDOR_INTF,
        out_pin, 1, 1000);

    if (ret != 1) {
        fprintf(stderr, "Failed to get DIN pin: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}
```

### Complete example: switch, wait for ACTIVE, monitor

```c
#include <time.h>

void use_i2s_input(libusb_device_handle *handle) {
    // 1. Switch to I2S
    if (switch_to_i2s(handle) != 0) return;

    // 2. Poll until ACTIVE or timeout (3 seconds)
    bool reached_active = false;
    for (int attempt = 0; attempt < 12; attempt++) {
        I2sInputStatusPacket status;
        if (get_i2s_status(handle, &status) != 0) break;

        if (status.state == I2S_STATE_ACTIVE) {
            printf("I2S input active at %u Hz, ring fill = %u words\n",
                   status.sample_rate, status.ring_fill_words);
            reached_active = true;
            break;
        }

        if (status.state == I2S_STATE_STARTING && status.ring_fill_words == 0) {
            printf("Waiting for samples from ADC... (no data yet)\n");
        } else {
            printf("State: %u, ring fill: %u\n", status.state, status.ring_fill_words);
        }

        // Wait 250 ms between polls
        struct timespec ts = { .tv_nsec = 250000000 };
        nanosleep(&ts, NULL);
    }

    if (!reached_active) {
        printf("I2S input did not reach ACTIVE state. Check ADC connection.\n");
        printf("  Verify: ADC powered, BCK/LRCLK reaching ADC, DIN wired to GPIO %u\n",
               /* current din_pin from earlier query */ 18);
        return;
    }

    // 3. ... use I2S input as your audio source ...
}
```

### Switching DIN pin (with required source dance)

```c
int change_i2s_din_pin_safely(libusb_device_handle *handle, uint8_t new_pin) {
    // 1. Check current source
    uint8_t source;
    if (get_input_source(handle, &source) != 0) return -1;

    bool was_i2s = (source == INPUT_SOURCE_I2S);

    // 2. If currently on I2S, switch to USB first
    if (was_i2s) {
        uint8_t usb_src = INPUT_SOURCE_USB;
        libusb_control_transfer(handle,
            LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
            REQ_SET_INPUT_SOURCE, 0, VENDOR_INTF,
            &usb_src, 1, 1000);

        // Wait briefly for switch to complete (~50 ms is plenty)
        struct timespec ts = { .tv_nsec = 100000000 };  // 100 ms
        nanosleep(&ts, NULL);
    }

    // 3. Change the pin
    if (set_i2s_din_pin(handle, new_pin) != 0) {
        // Restore source if switch failed
        if (was_i2s) switch_to_i2s(handle);
        return -1;
    }

    // 4. Restore I2S source if originally active
    if (was_i2s) {
        return switch_to_i2s(handle);
    }

    return 0;
}
```

### Recommended polling interval

For responsive UI feedback, poll `REQ_GET_I2S_INPUT_STATUS` at **250 ms intervals**. There is no interrupt or notification mechanism — the host must poll.

- **Minimum interval:** 100 ms. Faster polling provides no additional information because the receiver state machine operates on ~10-25 ms timescales.
- **Maximum interval:** 1000 ms. Longer intervals may cause noticeable UI lag.
- **Idle optimization:** When `state == INACTIVE` (USB or S/PDIF is the active input), you can stop polling I2S status entirely and resume only when the user activates I2S.

The status query is lightweight (reads cached state variables and one DMA register, no audio interaction) and does not affect audio processing.

---

## 12. Backward Compatibility

### Older firmware

Firmware versions that do not implement I2S input will **STALL** on vendor requests `0xCA`-`0xCC` and reject `INPUT_SOURCE_I2S = 2` from `REQ_SET_INPUT_SOURCE`. This is standard USB behavior for unsupported vendor requests / values. Control software should handle the STALL gracefully:

```c
int ret = libusb_control_transfer(handle, ..., REQ_GET_I2S_INPUT_STATUS, ...);
if (ret == LIBUSB_ERROR_PIPE) {
    // Firmware does not support I2S input — hide UI controls
    i2s_input_supported = false;
}
```

You can also detect support by checking the response from `REQ_GET_INPUT_SOURCE_MAX` if implemented, or by attempting `REQ_SET_INPUT_SOURCE` with value `2` and observing whether `REQ_GET_INPUT_SOURCE` then reads back `2` (success) or stays at the prior value (silent rejection on older firmware).

### Older presets

Presets saved with `SLOT_DATA_VERSION < 13` do not contain an `input_source` field. When such a preset is loaded:
- The input source remains at whatever it was before the load.
- On a fresh boot (where `active_input_source` starts as USB), loading an old preset will leave USB as the active source.

Presets at `SLOT_DATA_VERSION 13` (added between `4e3a129` and the I2S-input feature) store `input_source` but were saved when only USB and SPDIF existed. Such a preset cannot have stored `INPUT_SOURCE_I2S = 2` — the firmware that wrote it had no way to set that value.

### Older directories

Pre-I2S firmware uses `PresetDirectory` v2, which has no `i2s_din_pin` field. On first boot of post-stash firmware:
1. `dir_load_cache()` detects v2 magic and version.
2. Reads with the v2 struct, validates v2 CRC.
3. Copies fields into the current v3 struct.
4. Defaults `i2s_din_pin` to `PICO_I2S_DIN_PIN_DEFAULT` (GPIO 18).
5. Flushes as v3.

Migration is one-way (no v3 → v2 downgrade). Once migrated, older firmware would treat the directory as invalid (CRC mismatch) and fall back to factory defaults.

### Wire format

Bulk parameter payloads with `format_version < 7` do not contain the `WireInputConfig` section. Older host applications that request fewer bytes than the full `WireBulkParams` size will receive a truncated response (the firmware respects `wLength`).

The `i2s_din_pin` field in `WireInputConfig` was added without bumping `WIRE_FORMAT_VERSION` — it consumes one byte from the existing reserved area. Older firmware reads it as zero, which the I2S input subsystem treats as "use device default".

No existing vendor commands are modified by the I2S input feature. All prior commands (EQ, matrix mixer, crossfeed, loudness, leveller, master volume, output type, pin config, presets, etc.) continue to function identically regardless of input source.

---

## Vendor Command Summary

| Code | Command | Direction | Data | Description |
|------|---------|-----------|------|-------------|
| `0xE0` | `REQ_SET_INPUT_SOURCE` | OUT (`0x41`) | 1 byte: source (0=USB, 1=SPDIF, 2=I2S) | Switch input source (deferred, non-blocking) |
| `0xE1` | `REQ_GET_INPUT_SOURCE` | IN (`0xC1`) | 1 byte: source | Query current input source |
| `0xCA` | `REQ_SET_I2S_DIN_PIN` | IN (`0xC1`)\* | wValue=pin, 1 byte response: status | Set DIN GPIO pin (immediate response) |
| `0xCB` | `REQ_GET_I2S_DIN_PIN` | IN (`0xC1`) | 1 byte: pin number (default 18) | Query current DIN GPIO pin |
| `0xCC` | `REQ_GET_I2S_INPUT_STATUS` | IN (`0xC1`) | 16 bytes: I2sInputStatusPacket | Query receiver state, rate, ring fill, pin |

\* `REQ_SET_I2S_DIN_PIN` uses IN direction (`0xC1` bmRequestType) with an immediate 1-byte status response. Status codes: `0x00`=success, `0x01`=invalid pin, `0x02`=pin in use, `0x04`=I2S input active (cannot change pin while in use).

---

## Related Specs

- [`SPDIF_input_spec.md`](SPDIF_input_spec.md) — sister spec for the S/PDIF input subsystem; shares the input-source switching machinery (`REQ_SET_INPUT_SOURCE` / `REQ_GET_INPUT_SOURCE`).
- [`i2s_output_spec.md`](i2s_output_spec.md) — I2S **output** support; shares the BCK/LRCLK pins and the same `pico_audio_i2s_multi` library and clock-domain election logic.
- [`input_switching_spec.md`](input_switching_spec.md) — overview of the input-source abstraction shared across USB / SPDIF / I2S.
- [`per_channel_preamp_spec.md`](per_channel_preamp_spec.md) — preamp gain applied per-channel during input decode for all input sources (USB, SPDIF, I2S).
