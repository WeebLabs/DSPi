# S/PDIF Input Specification

## 1. Overview

DSPi supports two audio input sources: **USB** (0) and **S/PDIF** (1). The input source is selected at runtime via vendor USB control transfers. When S/PDIF is the active source, received audio feeds through the identical DSP pipeline (preamp, loudness, master EQ, leveller, crossfeed, matrix mixer, per-output EQ, delay) as USB audio.

Six vendor commands control S/PDIF input functionality:

| Code | Name | Direction | Description |
|------|------|-----------|-------------|
| 0xE0 | `REQ_SET_INPUT_SOURCE` | OUT | Switch between USB and S/PDIF input |
| 0xE1 | `REQ_GET_INPUT_SOURCE` | IN | Query the current input source |
| 0xE2 | `REQ_GET_SPDIF_RX_STATUS` | IN | Query 16-byte receiver status struct |
| 0xE3 | `REQ_GET_SPDIF_RX_CH_STATUS` | IN | Query 24-byte raw IEC 60958 channel status |
| 0xE4 | `REQ_SET_SPDIF_RX_PIN` | SET (immediate response) | Configure SPDIF RX GPIO pin |
| 0xE5 | `REQ_GET_SPDIF_RX_PIN` | IN | Query current SPDIF RX GPIO pin |

The input source enum and related definitions are in `audio_input.h`:

```c
typedef enum {
    INPUT_SOURCE_USB   = 0,
    INPUT_SOURCE_SPDIF = 1,
} InputSource;

#define INPUT_SOURCE_MAX    INPUT_SOURCE_SPDIF
#define PICO_SPDIF_RX_PIN_DEFAULT  11
```

---

## 2. State Machine

The S/PDIF receiver state machine is defined in `spdif_input.h` as `SpdifInputState`:

```c
typedef enum {
    SPDIF_INPUT_INACTIVE   = 0,   // RX hardware stopped (not selected as input)
    SPDIF_INPUT_ACQUIRING  = 1,   // Waiting for initial signal lock
    SPDIF_INPUT_LOCKED     = 2,   // Receiving and processing audio
    SPDIF_INPUT_RELOCKING  = 3,   // Signal lost, waiting for re-lock
} SpdifInputState;
```

### State Transitions

```
                          spdif_input_start()
                    +------------------------------+
                    |                              v
             +-----------+                  +-----------+
             |           |  spdif_input_    |           |
             | INACTIVE  |-----start()----->| ACQUIRING |
             |  (0)      |                  |   (1)     |
             +-----------+                  +-----+-----+
                  ^                               |
                  | spdif_input_stop()             | on_stable callback
                  | (from any state)               | (supported rate)
                  |                               v
                  |                         +-----------+
                  |                         |           |<--------------+
                  +-------------------------|  LOCKED   |              |
                  |                         |   (2)     |------+       |
                  |                         +-----+-----+      |       |
                  |                               |            |       |
                  |                on_lost callback|   on_stable|callback
                  |                               |   (re-lock)|
                  |                               v            |
                  |                         +-----------+      |
                  +-------------------------| RELOCKING |------+
                                            |   (3)     |
                                            +-----------+
```

### State Descriptions

- **INACTIVE (0):** RX hardware is stopped. This is the state when USB is the active input or the device has just booted. No PIO or DMA resources are claimed. Entering this state calls `spdif_rx_end()`.

- **ACQUIRING (1):** `spdif_input_start()` has been called. The library is scanning for BMC-encoded transitions on the RX GPIO pin. The `on_stable_callback` is registered and will fire when the library locks to an incoming signal and identifies its sample rate.

- **LOCKED (2):** Audio data is being extracted from the FIFO and fed through the DSP pipeline. The clock servo is active. A lock debounce period (8 polls, ~a few ms) is enforced before audio processing begins, allowing the FIFO to build up. The sample rate is confirmed.

- **RELOCKING (3):** The `on_lost_stable_callback` fired, indicating signal loss. Audio output is muted. The receiver remains running and will transition back to LOCKED if the signal returns and a new `on_stable_callback` fires with a supported rate. If the signal returns at an unsupported rate, the state remains RELOCKING.

### Transitions

| From | To | Trigger | What happens |
|------|----|---------|-------------|
| INACTIVE | ACQUIRING | `spdif_input_start()` called (input source set to SPDIF) | PIO/DMA started, callbacks registered, counters reset, output muted |
| ACQUIRING | LOCKED | Library reports stable lock at a supported rate (44100/48000/96000) | Clock servo initialized, debounce counter starts |
| ACQUIRING | RELOCKING | Library reports stable lock at an unsupported rate | Output stays muted, `sample_rate` field is 0 |
| LOCKED | RELOCKING | Library reports signal loss (`on_lost_stable` callback) | Output muted immediately, servo reset, `loss_count` incremented |
| RELOCKING | LOCKED | Library reports stable lock at a supported rate | Servo re-initialized, debounce counter restarted |
| Any | INACTIVE | `spdif_input_stop()` called (input source changed away from SPDIF) | PIO/DMA stopped, all state cleared |

### Timing

| Event | Duration |
|-------|----------|
| Library lock acquisition (ACQUIRING period) | ~64 ms typical (16 consecutive valid SPDIF blocks) |
| Signal loss detection | ~10 ms (DMA activity watchdog) |
| Lock debounce (firmware, after library reports lock) | ~8 main-loop polls (a few ms) |
| SPDIF RX lock debounce constant (`SPDIF_RX_LOCK_DEBOUNCE_MS`) | 100 ms (firmware `#define`, not configurable via vendor command) |
| Total USB-to-SPDIF switch time | ~100-200 ms (mute + lock + debounce + FIFO fill) |
| SPDIF-to-USB switch time | < 10 ms (immediate stop + flush + unmute) |

---

## 3. Vendor Commands

All commands use the standard DSPi vendor control transfer format:

- **bmRequestType:** `0x41` (Host-to-Device SET) or `0xC1` (Device-to-Host GET)
- **wIndex:** `2` (vendor interface number, `VENDOR_INTERFACE_NUMBER`)
- **Timeout:** 1000 ms recommended for all commands

---

### 3.1 REQ_SET_INPUT_SOURCE (0xE0)

Switch the active input source.

**Direction:** Host -> Device (SET)
**bmRequestType:** `0x41`
**bRequest:** `0xE0`
**wValue:** `0` (unused)
**wLength:** `1`

#### Request payload (1 byte)

| Offset | Size | Type | Field | Values |
|--------|------|------|-------|--------|
| 0 | 1 | uint8_t | `source` | `0` = USB, `1` = S/PDIF |

#### Behavior

This command returns immediately (non-blocking). The actual source switch is deferred to the firmware main loop via a pending flag:

1. Firmware validates the source value and checks it differs from the current source.
2. If valid and different, sets `input_source_change_pending = true`.
3. Main loop detects the flag and executes the switch sequence:
   - Drains any pending USB audio data
   - Mutes output for 256 samples (~5 ms at 48 kHz)
   - Stops old source hardware (if SPDIF, calls `spdif_input_stop()`)
   - Updates `active_input_source`
   - Starts new source hardware (if SPDIF, calls `spdif_input_start()`)
   - For USB: flushes stale ring data and unmutes immediately
   - For SPDIF: output remains muted until lock is acquired (see state machine)

#### Error handling

| Condition | Firmware behavior |
|-----------|-------------------|
| `source > 1` (invalid) | Ignored silently, no action |
| `source == active_input_source` | Ignored silently, no action |
| No SPDIF signal connected | Switch proceeds; output stays muted in ACQUIRING state indefinitely |
| SPDIF signal at unsupported rate | Switch proceeds; output stays muted in RELOCKING state |

There is no error response. The command always ACKs successfully. Use `REQ_GET_SPDIF_RX_STATUS` to monitor the outcome.

#### Firmware implementation (vendor_commands.c)

```c
case REQ_SET_INPUT_SOURCE: {
    if (buffer->data_len >= 1) {
        uint8_t src = vendor_rx_buf[0];
        if (input_source_valid(src) && src != active_input_source) {
            pending_input_source = src;
            __dmb();
            input_source_change_pending = true;
        }
    }
    break;
}
```

#### Hex example

Switch to S/PDIF input:
```
bmRequestType: 0x41
bRequest:      0xE0
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0001
Data:          01
```

Switch to USB input:
```
bmRequestType: 0x41
bRequest:      0xE0
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0001
Data:          00
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
| 0 | 1 | uint8_t | `source` | `0` = USB, `1` = S/PDIF |

#### Notes

- Returns the current value of `active_input_source`.
- If a source switch is pending (deferred to main loop), this returns the source that was active **before** the switch. The switch completes asynchronously.
- At boot, defaults to `0` (USB) unless a preset with SPDIF input was loaded.

#### Hex example

```
bmRequestType: 0xC1
bRequest:      0xE1
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0001
Response:      01           (S/PDIF is active)
```

---

### 3.3 REQ_GET_SPDIF_RX_STATUS (0xE2)

Query the S/PDIF receiver status. Returns a 16-byte packed struct.

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0xE2`
**wValue:** `0` (unused)
**wLength:** `16`

#### Response: SpdifRxStatusPacket (16 bytes)

Defined in `spdif_input.h`:

```c
typedef struct __attribute__((packed)) {
    uint8_t  state;              // SpdifInputState enum (0-3)
    uint8_t  input_source;       // Current active InputSource enum (0-1)
    uint8_t  lock_count;         // Number of successful locks since activation
    uint8_t  loss_count;         // Number of lock losses since activation
    uint32_t sample_rate;        // Detected sample rate in Hz (0 if not locked)
    uint32_t parity_errors;      // Cumulative parity error count
    uint16_t fifo_fill_pct;      // RX FIFO fill percentage (0-100)
    uint16_t reserved;           // Debug: byte 14 = library state, byte 15 = callback counters
} SpdifRxStatusPacket;           // 16 bytes
```

All multi-byte fields are **little-endian** (native ARM Cortex-M0+/M33 byte order).

#### Byte layout

```
Offset: 00 01 02 03  04 05 06 07  08 09 0A 0B  0C 0D 0E 0F
Field:  ST IS LC LS  SR SR SR SR  PE PE PE PE  FF FF RR RR

ST = state           IS = input_source    LC = lock_count     LS = loss_count
SR = sample_rate     PE = parity_errors   FF = fifo_fill_pct  RR = reserved
```

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 1 | uint8_t | `state` | `SpdifInputState` enum (0-3) |
| 1 | 1 | uint8_t | `input_source` | Current `InputSource` enum (`0`=USB, `1`=SPDIF) |
| 2 | 1 | uint8_t | `lock_count` | Number of successful locks since activation (0-255, saturates) |
| 3 | 1 | uint8_t | `loss_count` | Number of lock losses since activation (0-255, saturates) |
| 4 | 4 | uint32_t LE | `sample_rate` | Detected sample rate in Hz, or `0` if not locked/unsupported |
| 8 | 4 | uint32_t LE | `parity_errors` | Cumulative parity error count |
| 12 | 2 | uint16_t LE | `fifo_fill_pct` | RX FIFO fill level as percentage (0-100) |
| 14 | 1 | uint8_t | `reserved[0]` | Temporary debug: library internal state (0=NO_SIGNAL, 1=WAITING_STABLE, 2=STABLE) |
| 15 | 1 | uint8_t | `reserved[1]` | Temporary debug: high nibble = on_stable callback count (0-15), low nibble = on_lost_stable callback count (0-15) |

#### State values

| Value | Name | Description |
|-------|------|-------------|
| 0 | `INACTIVE` | Receiver hardware is stopped (not the active input source) |
| 1 | `ACQUIRING` | Receiver started, waiting for initial signal lock |
| 2 | `LOCKED` | Locked and processing audio |
| 3 | `RELOCKING` | Signal lost, waiting for re-lock |

#### Sample rate values

| `sample_rate` | Meaning |
|---------------|---------|
| 0 | Not locked, unsupported rate, or not yet detected |
| 44100 | 44.1 kHz |
| 48000 | 48 kHz |
| 96000 | 96 kHz |

Rates 88200, 176400, and 192000 are detected by the library but are not supported by the DSP pipeline. When one of these rates is detected, `sample_rate` is reported as `0` and the state remains `RELOCKING`.

#### Counter behavior

- `lock_count` and `loss_count` reset to 0 when the receiver is started (`spdif_input_start()`).
- They saturate at 255 (do not wrap).
- `parity_errors` is a cumulative count from the library, queried via `spdif_rx_get_parity_err_count()`. Only populated when state is not INACTIVE.

#### FIFO fill percentage

- Calculated as `(fifo_word_count * 100) / SPDIF_RX_FIFO_SIZE`.
- A healthy locked connection hovers around 50% (the clock servo target).
- 0% when INACTIVE or no data arriving.
- Values significantly deviating from 50% while LOCKED suggest clock drift issues.

#### Example response (hex)

Locked at 48 kHz, 2 locks, 0 losses, no parity errors, FIFO at 48%, library STABLE, 1 on_stable callback:
```
02 01 02 00  80 BB 00 00  00 00 00 00  30 00 02 10
```

Breakdown:
- `02` = LOCKED
- `01` = input_source is SPDIF
- `02` = 2 locks since activation
- `00` = 0 losses
- `80 BB 00 00` = 48000 (0x0000BB80 LE)
- `00 00 00 00` = 0 parity errors
- `30 00` = 48% FIFO fill (0x0030 LE)
- `02` = reserved[0]: library state 2 (STABLE)
- `10` = reserved[1]: high nibble 1 = 1 on_stable callback, low nibble 0 = 0 on_lost_stable callbacks

---

### 3.4 REQ_GET_SPDIF_RX_CH_STATUS (0xE3)

Retrieve the raw IEC 60958-3 channel status bits from the received S/PDIF stream.

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0xE3`
**wValue:** `0` (unused)
**wLength:** `24`

#### Response (24 bytes)

The full 192-bit IEC 60958-3 channel status block, organized as 24 bytes. Byte 0 bit 0 corresponds to the first channel status bit. When the receiver is INACTIVE, all 24 bytes are zero.

These bytes are only meaningful when the receiver state is `LOCKED` (state 2).

#### Firmware implementation

```c
void spdif_input_get_channel_status(uint8_t *out_24_bytes) {
    if (spdif_state != SPDIF_INPUT_INACTIVE) {
        spdif_rx_get_c_bits(out_24_bytes, 24, 0);
    } else {
        memset(out_24_bytes, 0, 24);
    }
}
```

#### Key fields for application developers

| Byte | Bits | Field | Description |
|------|------|-------|-------------|
| 0 | 0 | Consumer/Professional | `0` = consumer (IEC 60958-3), `1` = professional (AES3) |
| 0 | 1 | Audio/Non-audio | `0` = PCM audio, `1` = non-audio (e.g., AC-3, DTS) |
| 0 | 2-4 | Emphasis | Pre-emphasis mode |
| 0 | 5 | Lock | `0` = locked (confusingly inverted in IEC 60958) |
| 1 | 0-7 | Category code | Source device category |
| 2 | 0-3 | Source number | Identifies source within a category |
| 2 | 4-7 | Channel number | Channel identification |
| 3 | 0-3 | Sample rate | Sample frequency code (see table below) |
| 3 | 4-5 | Clock accuracy | `00` = Level II (default), `01` = Level I, `10` = Level III |
| 4 | 0 | Max word length | `0` = 20-bit max, `1` = 24-bit max |
| 4 | 1-3 | Word length | Sample word length (see table below) |

#### Sample rate codes (byte 3, bits 0-3)

| Byte 3 & 0x0F | Sample rate |
|----------------|-------------|
| `0x00` | 44.1 kHz |
| `0x02` | 48 kHz |
| `0x03` | 32 kHz |
| `0x08` | 88.2 kHz |
| `0x0A` | 96 kHz |
| `0x0C` | 176.4 kHz |
| `0x0E` | 192 kHz |

#### Word length codes (byte 4, bits 0-3)

| Byte 4 & 0x0F | Meaning |
|----------------|---------|
| `0x00` | Not indicated (max 20-bit) |
| `0x02` | 16-bit (max 20-bit) |
| `0x04` | 20-bit (max 20-bit) |
| `0x08` | 17-bit (max 24-bit) |
| `0x0A` | 22-bit (max 24-bit) |
| `0x0B` | 24-bit (max 24-bit) |

#### Hex example

```
bmRequestType: 0xC1
bRequest:      0xE3
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0018  (24)

Response (24 bytes):
04 00 00 02 0B 00 00 00  00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00

Interpretation:
  Byte 0 = 0x04: consumer format, PCM audio, copy permitted
  Byte 3 = 0x02: 48 kHz
  Byte 4 = 0x0B: 24-bit audio
```

---

### 3.5 REQ_SET_SPDIF_RX_PIN (0xE4)

Set the GPIO pin used for S/PDIF input. This is a device-level setting stored in the preset directory (not per-preset).

**Direction:** Host -> Device (GET-style immediate response)
**bmRequestType:** `0xC1`
**bRequest:** `0xE4`
**wValue:** New GPIO pin number
**wLength:** `1`

Note: Despite being a "SET" command, this uses a GET-direction transfer (`0xC1`) because it returns an immediate status byte. The pin number is passed in `wValue`, not in the request body. This follows the same pattern as `REQ_SET_I2S_BCK_PIN` and other pin configuration commands.

#### Response (1 byte)

| Value | Name | Meaning |
|-------|------|---------|
| `0x00` | `PIN_CONFIG_SUCCESS` | Pin changed (or already set to this value) |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | Pin is out of range or reserved |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | Pin is already used by an output, I2S BCK/LRCLK, or MCK |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | Cannot change pin while S/PDIF input is active |

#### Validation logic (vendor_commands.c)

```c
case REQ_SET_SPDIF_RX_PIN: {
    uint8_t new_pin = (uint8_t)setup->wValue;
    uint8_t status;
    if (!is_valid_gpio_pin(new_pin)) {
        status = PIN_CONFIG_INVALID_PIN;
    } else if (new_pin == spdif_rx_pin) {
        status = PIN_CONFIG_SUCCESS;  // No-op
    } else if (active_input_source == INPUT_SOURCE_SPDIF) {
        status = PIN_CONFIG_OUTPUT_ACTIVE;  // Can't change while RX active
    } else if (is_pin_in_use(new_pin, 0xFF)) {
        status = PIN_CONFIG_PIN_IN_USE;
    } else {
        spdif_rx_pin = new_pin;
        flash_set_spdif_rx_pin_pending = true;
        status = PIN_CONFIG_SUCCESS;
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
- **Not in use:** Must not conflict with any SPDIF output data pin, I2S data pin, I2S BCK or LRCLK (BCK+1), or MCK pin
- **Not active:** The SPDIF input must be INACTIVE (switch to USB first before changing the pin)

The pin setting persists across reboots (stored in the preset directory sector of flash). The default pin is GPIO 11 (`PICO_SPDIF_RX_PIN_DEFAULT`).

#### Hex example

Set SPDIF RX pin to GPIO 10:
```
bmRequestType: 0xC1
bRequest:      0xE4
wValue:        0x000A  (10)
wIndex:        0x0002
wLength:       0x0001
Response:      00       (SUCCESS)
```

Attempt to set pin while SPDIF input is active:
```
bmRequestType: 0xC1
bRequest:      0xE4
wValue:        0x000A
wIndex:        0x0002
wLength:       0x0001
Response:      04       (OUTPUT_ACTIVE)
```

---

### 3.6 REQ_GET_SPDIF_RX_PIN (0xE5)

Query the current GPIO pin configured for S/PDIF input.

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0xE5`
**wValue:** `0` (unused)
**wLength:** `1`

#### Response (1 byte)

| Offset | Size | Type | Field | Values |
|--------|------|------|-------|--------|
| 0 | 1 | uint8_t | `pin` | GPIO pin number (default: 11) |

#### Hex example

```
bmRequestType: 0xC1
bRequest:      0xE5
wValue:        0x0000
wIndex:        0x0002
wLength:       0x0001
Response:      0B       (GPIO 11)
```

---

## 4. Audio Data Format

### Raw SPDIF subframe word (32-bit, from library FIFO)

The `pico_spdif_rx` library delivers 32-bit words from its circular FIFO. Each word represents one subframe (one channel, one sample):

```
Bit:   31  30  29  28  27  26  25  24  23  22  21  20 ... 5   4   3   2   1   0
       V   U   C   P   |<---------- 24-bit audio ---------->|  |<- sync code ->|

V = Validity bit (0 = valid audio)
U = User data bit
C = Channel status bit
P = Parity bit (even parity over bits 4-31)
Bits 27:4 = 24-bit signed audio sample (MSB at bit 27)
Bits 3:0  = Sync/preamble code
```

### Audio sample extraction

To extract a signed 32-bit full-scale sample from a raw FIFO word:

```c
uint32_t word = fifo_buffer[i];
int32_t sample = (int32_t)((word & 0x0FFFFFF0u) << 4);
```

This masks out the VUCP flags (top 4 bits) and sync code (bottom 4 bits), then shifts left 4 to sign-extend the 24-bit value into the full 32-bit range.

The resulting `sample` is a signed 32-bit integer where:
- `+2,147,483,647` (`0x7FFFFFF0` shifted) = positive full-scale
- `-2,147,483,648` (`0x80000000` after shift) = negative full-scale
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
int32_t q28_l = raw_l >> 4;                    // int32 full-scale -> Q28
int32_t q28_r = raw_r >> 4;
buf_l[sample_idx] = fast_mul_q28(q28_l, preamp_l);  // apply preamp in Q28
buf_r[sample_idx] = fast_mul_q28(q28_r, preamp_r);
```

Right-shifts by 4 to convert from 32-bit full-scale to Q28 (28 fractional bits), then multiplies by the Q28 preamp gain (`global_preamp_mul[]`).

---

## 5. Clock Servo

When receiving S/PDIF and simultaneously outputting audio (S/PDIF or I2S), the input and output clocks are asynchronous. The input follows the external source's clock, while the output uses the local oscillator. Without correction, the RX FIFO will slowly overflow or underflow.

The firmware implements a PI (Proportional-Integral) controller that adjusts output PIO clock dividers to track the input clock rate.

### Control loop

```
                    +----------------------+
                    |   SPDIF RX FIFO      |
  SPDIF input  --> |  (target: 50% full)  | --> DSP pipeline --> Output PIO
                    +----------+-----------+
                               |
                    error = fifo_count - target_fill
                               |
                    +----------v-----------+
                    |   PI Controller      |
                    |  P = error * KP      |
                    |  I += error * KI     |
                    |  adjust = -(P + I)   |
                    +----------+-----------+
                               |
                    +----------v-----------+
                    |  Output PIO dividers |
                    |  (all SPDIF + I2S)   |
                    +----------------------+
```

### Parameters (firmware `#define` constants)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `SERVO_KP` | 0.0005 | Proportional gain |
| `SERVO_KI` | 0.000005 | Integral gain |
| `SERVO_MAX_ADJUST` | 0.005 | Maximum fractional divider adjustment (0.5%) |
| `SERVO_DEADBAND` | 2 x `SPDIF_BLOCK_SIZE` = 768 subframes | Error magnitude below which no adjustment is made |

### Behavior

- **Target:** FIFO fill at 50% of `SPDIF_RX_FIFO_SIZE`.
- **Deadband:** Errors within +/-768 subframes of the target are ignored, preventing jitter from causing unnecessary adjustments.
- **Direction:** Positive error (FIFO filling up) means outputs are too slow. The controller reduces the PIO clock divider (speeds up outputs). Negative error means the reverse.
- **Scope:** The adjustment is applied to ALL active output PIO state machines (both S/PDIF and I2S).
- **Reset:** The integral term and base divider are reset whenever the receiver transitions out of LOCKED state.

### Servo limits

The maximum adjustment of 0.5% corresponds to approximately 5000 ppm, far exceeding the worst-case crystal oscillator drift (~50 ppm). In practice, adjustments are typically in the tens-of-ppm range.

---

## 6. Supported Sample Rates

| Rate | Supported | Notes |
|------|-----------|-------|
| 44,100 Hz | Yes | CD audio |
| 48,000 Hz | Yes | Standard (default for USB) |
| 96,000 Hz | Yes | High-resolution |
| 32,000 Hz | Detected only | Reported as unsupported (`sample_rate = 0`) |
| 88,200 Hz | Detected only | Reported as unsupported (`sample_rate = 0`) |
| 176,400 Hz | Detected only | Reported as unsupported (`sample_rate = 0`) |
| 192,000 Hz | Detected only | Reported as unsupported (`sample_rate = 0`) |

The `is_supported_rate()` function in `spdif_input.c` accepts only 44100, 48000, and 96000 Hz:

```c
static bool is_supported_rate(uint32_t rate_hz) {
    return (rate_hz == 44100 || rate_hz == 48000 || rate_hz == 96000);
}
```

When an unsupported rate is detected:
- The receiver stays in RELOCKING state (state 3).
- `sample_rate` in the status packet is 0.
- Output remains muted.
- The library continues monitoring. If the source switches to a supported rate, the receiver transitions to LOCKED and output unmutes.

### Rate change handling

If the S/PDIF source changes rate while locked (e.g., switching tracks from 44.1 kHz to 48 kHz content):

1. The library detects the change and fires the `on_lost_stable` callback.
2. Firmware transitions to RELOCKING, mutes output.
3. The library re-acquires at the new rate and fires `on_stable` with the new frequency.
4. Firmware transitions to LOCKED, checks for rate change via `spdif_input_check_rate_change()`.
5. If the new rate differs from the current operating rate, a rate change is triggered: all DSP filter coefficients are recalculated for the new sample rate.
6. After debounce and FIFO fill, output unmutes.

This entire sequence is automatic and requires no host intervention.

---

## 7. Errors & Recovery

| Condition | Detection | State | Output | App recommendation |
|-----------|-----------|-------|--------|-------------------|
| **No signal** | Stays in ACQUIRING, no callback fires | 1 | Muted | Show "No S/PDIF signal" |
| **Signal loss** (~10ms) | `on_lost_stable_callback` fires | 3 | Muted immediately | Show "Signal lost" |
| **Signal returns after loss** | `on_stable_callback` fires | 3 -> 2 | Unmuted after debounce | Show "Locked" |
| **Rate change mid-stream** | Brief loss + re-lock at new rate | 3 -> 2 | Brief mute (~100-200 ms) | Automatic, no action needed |
| **Unsupported rate** | `on_stable_callback` with unsupported rate | 3 | Muted | Show "Unsupported rate" |
| **Jittery source** | Rapid lock/loss cycling | Varies | Intermittent mute | Check `loss_count`; warn about signal quality |
| **Parity errors** | `parity_errors` field incrementing | 2 | Active (may click) | Warn user about cable/EMI |

### Interpreting lock_count and loss_count

A healthy connection shows `lock_count = 1` and `loss_count = 0` after initial activation. If `loss_count` is incrementing, the signal is intermittent. Common causes:

- Loose cable connection
- Source device power-cycling or changing modes
- EMI interference on long cable runs
- Source crystal oscillator instability

---

## 8. GPIO Constraints

The S/PDIF RX pin follows the same validation rules as all GPIO pin assignments in DSPi.

### Valid pin range

| Platform | Valid range |
|----------|------------|
| RP2040 | GPIO 0-28 |
| RP2350 | GPIO 0-29 |

### Always excluded

| GPIO | Reason |
|------|--------|
| 12 | UART TX (debug console) |
| 23 | Power control (SMPS mode) |
| 24 | Power control (VBUS detect) |
| 25 | On-board LED |

### Must not conflict with

- Any S/PDIF output data pin (slot 0 through `NUM_SPDIF_INSTANCES - 1`)
- I2S BCK pin or LRCLK pin (BCK + 1), if any slot is configured as I2S
- I2S MCK pin, if MCK is enabled
- PDM output pin

The firmware checks all of these via `is_pin_in_use()` before allowing the change.

### Pin change restrictions

- The pin **cannot** be changed while S/PDIF input is active (state != INACTIVE). Switch to USB first, change the pin, then switch back to SPDIF.
- If the pin is already set to the requested value, the command returns SUCCESS as a no-op.

### Persistence

The RX pin is stored in the preset directory (device-level, not per-preset). It persists across reboots and preset changes. On first boot (no directory), it defaults to GPIO 11.

---

## 9. Preset Integration

### Flash storage (SLOT_DATA_VERSION = 13)

The `PresetSlot` struct in `flash_storage.c` includes the input source at version 13:

```c
// Input source selection (V13)
uint8_t input_source;            // InputSource enum (0=USB, 1=SPDIF)
uint8_t input_source_padding[3]; // Pad to 4-byte boundary
```

| Action | Behavior |
|--------|----------|
| **Save preset** | Current `active_input_source` is stored in the slot |
| **Load preset** | If slot version >= 13, defers an input source switch. If < 13, input source is left unchanged. |
| **Factory reset** | Input source defaults to USB (`INPUT_SOURCE_USB = 0`) |
| **Boot with no preset** | Defaults to USB |

### RX pin in PresetDirectory

The RX pin is a **device-level** setting stored in the `PresetDirectory` struct, not in individual preset slots:

```c
typedef struct __attribute__((packed)) {
    // ...
    uint8_t  spdif_rx_pin;   // SPDIF RX GPIO pin, device-level (was padding[1])
    // ...
} PresetDirectory;
```

It is unaffected by preset save/load/delete operations. It only changes via `REQ_SET_SPDIF_RX_PIN` and persists until explicitly changed.

### Wire format (WIRE_FORMAT_VERSION = 7)

The bulk parameter transfer includes a `WireInputConfig` section (16 bytes) added in version 7:

```c
typedef struct __attribute__((packed)) {
    uint8_t  input_source;       // InputSource enum (0=USB, 1=SPDIF)
    uint8_t  spdif_rx_pin;      // SPDIF RX GPIO pin (informational, SET does not apply)
    uint8_t  reserved[14];       // Future expansion (pad to 16 bytes)
} WireInputConfig;               // 16 bytes
```

The `spdif_rx_pin` field in the wire format is **informational** on GET. On SET (`REQ_SET_ALL_PARAMS`), the bulk apply function restores the `input_source` but does not modify the RX pin (which is a device-level setting managed separately).

### Mute during preset load

When a preset is loaded that switches the input source, the standard 256-sample (~5 ms) mute applies during the pipeline reset. If switching to SPDIF, additional mute time occurs during lock acquisition.

---

## 10. Platform Differences

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| SPDIF RX PIO block | PIO1 | PIO2 |
| SPDIF RX PIO SM | SM2 (SM0=PDM, SM1=MCK occupied) | SM0 (dedicated PIO block) |
| RX DMA IRQ | DMA_IRQ_0 (shared with I2S TX) | DMA_IRQ_0 (shared with I2S TX) |
| RX DMA channels | CH4, CH5 | CH5, CH6 |
| Internal audio format | Q28 fixed-point (32-bit) | IEEE 754 float |
| Sample conversion | `raw >> 4` then `fast_mul_q28()` | `raw * inv_2147483648 * preamp` |
| Supported input rates | 44.1, 48, 96 kHz | 44.1, 48, 96 kHz |
| Input bit depth | 24-bit | 24-bit |
| Default RX pin | GPIO 11 | GPIO 11 |
| SPDIF output slots | 2 (4 channels) | 4 (8 channels) |

DMA IRQ assignment: SPDIF RX uses DMA_IRQ_0 (shared with I2S TX when active). DMA_IRQ_1 is dedicated to SPDIF TX only. This isolates SPDIF RX from SPDIF TX, avoiding shared handler conflicts.

From `config.h`:

```c
#if PICO_RP2350
#define PICO_SPDIF_RX_DMA_CH0      5
#define PICO_SPDIF_RX_DMA_CH1      6
#else
#define PICO_SPDIF_RX_DMA_CH0      4
#define PICO_SPDIF_RX_DMA_CH1      5
#endif
```

Both platforms expose identical vendor command interfaces and status struct formats. Application code does not need to differentiate between platforms for S/PDIF input functionality.

### Library Patches

The forked `pico_spdif_rx` library (from `elehobica/pico_spdif_rx` v0.9.3) has the following DSPi-specific patches:

| Patch | Reason |
|-------|--------|
| PIO2 support for RP2350 | RP2350 uses a dedicated PIO2 block for SPDIF RX |
| Clock constants: 307.2 MHz sys_clk, 122.88 MHz PIO clock (divider 2.5 exact) | Match DSPi's overclocked sys_clk |
| Removed `pio_clear_instruction_memory()` | Destroys shared PIO programs (PDM, MCK on same PIO block) |
| Removed `irq_set_enabled(DMA_IRQ_x, false)` | Disables entire shared IRQ line, breaking other DMA users |
| Replaced `irq_has_shared_handler()` with private `irq_handler_registered` flag | Prevents incorrect handler registration when other libraries share the IRQ line |
| Added `irq_remove_handler()` in `spdif_rx_end()` | Clean lifecycle — handler is properly deregistered on shutdown |
| Added `save_and_disable_interrupts()` in `_spdif_rx_common_end()` | Prevents re-entrant teardown during shutdown sequence |

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

// SPDIF receiver states
#define SPDIF_STATE_INACTIVE   0
#define SPDIF_STATE_ACQUIRING  1
#define SPDIF_STATE_LOCKED     2
#define SPDIF_STATE_RELOCKING  3

// Vendor command request codes
#define REQ_SET_INPUT_SOURCE        0xE0
#define REQ_GET_INPUT_SOURCE        0xE1
#define REQ_GET_SPDIF_RX_STATUS     0xE2
#define REQ_GET_SPDIF_RX_CH_STATUS  0xE3
#define REQ_SET_SPDIF_RX_PIN        0xE4
#define REQ_GET_SPDIF_RX_PIN        0xE5

// Pin configuration status codes
#define PIN_CONFIG_SUCCESS       0x00
#define PIN_CONFIG_INVALID_PIN   0x01
#define PIN_CONFIG_PIN_IN_USE    0x02
#define PIN_CONFIG_OUTPUT_ACTIVE 0x04

// Vendor interface number
#define VENDOR_INTF  2

// SPDIF RX status packet (matches firmware SpdifRxStatusPacket exactly)
typedef struct __attribute__((packed)) {
    uint8_t  state;           // SPDIF_STATE_xxx
    uint8_t  input_source;    // INPUT_SOURCE_xxx
    uint8_t  lock_count;      // Locks since activation (0-255)
    uint8_t  loss_count;      // Losses since activation (0-255)
    uint32_t sample_rate;     // Detected Hz (0/44100/48000/96000), little-endian
    uint32_t parity_errors;   // Cumulative parity error count, little-endian
    uint16_t fifo_fill_pct;   // 0-100, little-endian
    uint8_t  lib_state;       // Debug: library internal state (0-2)
    uint8_t  callback_counts; // Debug: high nibble = on_stable count, low = on_lost_stable count
} SpdifRxStatusPacket;        // 16 bytes total
```

### Switch to S/PDIF input

```c
int switch_to_spdif(libusb_device_handle *handle) {
    uint8_t source = INPUT_SOURCE_SPDIF;
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
    return 0;  // Command accepted (switch is async -- poll status to confirm)
}
```

### Switch to USB input

```c
int switch_to_usb(libusb_device_handle *handle) {
    uint8_t source = INPUT_SOURCE_USB;
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_SET_INPUT_SOURCE,
        0, VENDOR_INTF,
        &source, 1, 1000);

    if (ret < 0) {
        fprintf(stderr, "Failed to switch input: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}
```

### Query current input source

```c
int get_input_source(libusb_device_handle *handle, uint8_t *out_source) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_INPUT_SOURCE,   // 0xE1
        0, VENDOR_INTF,
        out_source, 1, 1000);

    if (ret != 1) {
        fprintf(stderr, "Failed to get input source: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}

// Usage:
// uint8_t source;
// if (get_input_source(handle, &source) == 0) {
//     printf("Current input: %s\n", source == INPUT_SOURCE_SPDIF ? "S/PDIF" : "USB");
// }
```

### Poll receiver status

```c
int get_spdif_status(libusb_device_handle *handle, SpdifRxStatusPacket *out) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_SPDIF_RX_STATUS,   // 0xE2
        0, VENDOR_INTF,
        (uint8_t *)out, sizeof(SpdifRxStatusPacket),
        1000);

    if (ret != (int)sizeof(SpdifRxStatusPacket)) {
        fprintf(stderr, "Failed to get SPDIF status: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}

// Usage: polling loop for a status display (recommended: every 200 ms)
void update_spdif_status_display(libusb_device_handle *handle) {
    SpdifRxStatusPacket status;
    if (get_spdif_status(handle, &status) != 0) return;

    const char *state_names[] = {
        "Inactive", "Acquiring", "Locked", "Relocking"
    };
    printf("State: %s\n", state_names[status.state & 3]);

    if (status.state == SPDIF_STATE_LOCKED) {
        printf("Sample rate: %u Hz\n", status.sample_rate);
        printf("FIFO fill: %u%%\n", status.fifo_fill_pct);
        if (status.parity_errors > 0) {
            printf("WARNING: %u parity errors detected\n", status.parity_errors);
        }
    }

    printf("Locks: %u, Losses: %u\n", status.lock_count, status.loss_count);
}
```

### Read and parse IEC 60958 channel status

```c
int get_spdif_channel_status(libusb_device_handle *handle, uint8_t out_24[24]) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_SPDIF_RX_CH_STATUS,   // 0xE3
        0, VENDOR_INTF,
        out_24, 24, 1000);

    if (ret != 24) {
        fprintf(stderr, "Failed to get channel status: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}

void print_spdif_source_info(libusb_device_handle *handle) {
    uint8_t cs[24];
    if (get_spdif_channel_status(handle, cs) != 0) return;

    // Print raw hex
    printf("Channel status (24 bytes): ");
    for (int i = 0; i < 24; i++) printf("%02X ", cs[i]);
    printf("\n");

    // Byte 0: format flags
    int professional = cs[0] & 0x01;
    int non_audio    = cs[0] & 0x02;
    int copy_ok      = cs[0] & 0x04;
    printf("Format: %s, %s%s\n",
           professional ? "Professional (AES3)" : "Consumer (IEC 60958-3)",
           non_audio ? "Non-audio (compressed)" : "PCM audio",
           copy_ok ? ", copy permitted" : "");

    // Byte 3: sample rate
    const char *rate_str = "Unknown";
    switch (cs[3] & 0x0F) {
        case 0x00: rate_str = "44.1 kHz"; break;
        case 0x02: rate_str = "48 kHz";   break;
        case 0x03: rate_str = "32 kHz";   break;
        case 0x08: rate_str = "88.2 kHz"; break;
        case 0x0A: rate_str = "96 kHz";   break;
        case 0x0C: rate_str = "176.4 kHz"; break;
        case 0x0E: rate_str = "192 kHz";  break;
    }
    printf("Channel status rate: %s\n", rate_str);

    // Byte 4: word length
    const char *wl_str = "Not indicated";
    switch (cs[4] & 0x0F) {
        case 0x02: wl_str = "16-bit"; break;
        case 0x04: wl_str = "20-bit"; break;
        case 0x0A: wl_str = "22-bit"; break;
        case 0x0B: wl_str = "24-bit"; break;
    }
    printf("Word length: %s\n", wl_str);
}
```

### Configure and query RX pin

```c
int set_spdif_rx_pin(libusb_device_handle *handle, uint8_t pin) {
    uint8_t status;
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_SET_SPDIF_RX_PIN,   // 0xE4
        pin,                     // wValue = GPIO pin
        VENDOR_INTF,
        &status, 1, 1000);

    if (ret != 1) {
        fprintf(stderr, "Failed to set RX pin: %s\n", libusb_error_name(ret));
        return -1;
    }

    switch (status) {
        case PIN_CONFIG_SUCCESS:
            printf("RX pin set to GPIO %u\n", pin);
            return 0;
        case PIN_CONFIG_INVALID_PIN:
            fprintf(stderr, "GPIO %u is not a valid pin\n", pin);
            return -1;
        case PIN_CONFIG_PIN_IN_USE:
            fprintf(stderr, "GPIO %u is already in use by another function\n", pin);
            return -1;
        case PIN_CONFIG_OUTPUT_ACTIVE:
            fprintf(stderr, "Cannot change pin while SPDIF input is active\n");
            return -1;
        default:
            fprintf(stderr, "Unknown status: 0x%02X\n", status);
            return -1;
    }
}

int get_spdif_rx_pin(libusb_device_handle *handle, uint8_t *out_pin) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        REQ_GET_SPDIF_RX_PIN,   // 0xE5
        0, VENDOR_INTF,
        out_pin, 1, 1000);

    if (ret != 1) {
        fprintf(stderr, "Failed to get RX pin: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}
```

### Complete example: switch, wait for lock, query status and channel info

```c
void monitor_spdif_input(libusb_device_handle *handle) {
    // Switch to SPDIF
    if (switch_to_spdif(handle) != 0) return;

    // Poll until locked or timeout (5 seconds)
    for (int attempt = 0; attempt < 20; attempt++) {
        SpdifRxStatusPacket status;
        if (get_spdif_status(handle, &status) != 0) break;

        if (status.state == SPDIF_STATE_LOCKED) {
            printf("Locked to S/PDIF at %u Hz (FIFO %u%%)\n",
                   status.sample_rate, status.fifo_fill_pct);

            // Read and display channel status
            print_spdif_source_info(handle);
            return;
        }
        printf("State: %u, waiting...\n", status.state);

        // Wait 250ms between polls
        struct timespec ts = { .tv_nsec = 250000000 };
        nanosleep(&ts, NULL);
    }

    printf("Failed to lock to S/PDIF signal within 5 seconds\n");

    // Switch back to USB
    switch_to_usb(handle);
}
```

### Recommended polling interval

For responsive UI feedback, poll `REQ_GET_SPDIF_RX_STATUS` at **200 ms intervals**. There is no interrupt or notification mechanism -- the host must poll.

- **Minimum interval:** 100 ms. Faster polling provides no additional information because the receiver state machine operates on ~50-100 ms timescales.
- **Maximum interval:** 500 ms. Longer intervals may cause noticeable UI lag when the receiver state changes.
- **Idle optimization:** When `state == INACTIVE` (USB is the active input), you can reduce the polling rate to 1000 ms or stop polling entirely.

The status query is lightweight (reads cached state variables, no hardware interaction) and does not affect audio processing.

---

## 12. Backward Compatibility

### Older firmware

Firmware versions that do not implement S/PDIF input will **STALL** on vendor requests 0xE0-0xE5. This is standard USB behavior for unsupported vendor requests. Control software should handle the STALL gracefully and hide S/PDIF input controls:

```c
int ret = libusb_control_transfer(handle, ..., REQ_GET_INPUT_SOURCE, ...);
if (ret == LIBUSB_ERROR_PIPE) {
    // Firmware does not support SPDIF input -- hide UI controls
    spdif_input_supported = false;
}
```

The S/PDIF input feature is available in firmware v1.1.0 and later. Use `REQ_GET_PLATFORM` (0x7F) to check the firmware version.

### Older presets

Presets saved with `SLOT_DATA_VERSION < 13` do not contain an `input_source` field. When such a preset is loaded:
- The input source remains at whatever it was before the load.
- On a fresh boot (where `active_input_source` starts as USB), loading an old preset will leave USB as the active source.

### Wire format

Bulk parameter payloads with `format_version < 7` do not contain the `WireInputConfig` section. The firmware checks the format version before accessing V7+ fields. Older host applications that request fewer bytes than the full `WireBulkParams` size will receive a truncated response (the firmware respects `wLength`).

No existing vendor commands are modified by the S/PDIF input feature. All prior commands (EQ, matrix mixer, crossfeed, loudness, pin config, presets, etc.) continue to function identically regardless of input source.

---

## Vendor Command Summary

| Code | Command | Direction | Data | Description |
|------|---------|-----------|------|-------------|
| `0xE0` | `REQ_SET_INPUT_SOURCE` | OUT (0x41) | 1 byte: source (0=USB, 1=SPDIF) | Switch input source (deferred, non-blocking) |
| `0xE1` | `REQ_GET_INPUT_SOURCE` | IN (0xC1) | 1 byte: source | Query current input source |
| `0xE2` | `REQ_GET_SPDIF_RX_STATUS` | IN (0xC1) | 16 bytes: SpdifRxStatusPacket | Query receiver state, rate, errors, FIFO fill |
| `0xE3` | `REQ_GET_SPDIF_RX_CH_STATUS` | IN (0xC1) | 24 bytes: IEC 60958 channel status | Raw channel status bits from received stream |
| `0xE4` | `REQ_SET_SPDIF_RX_PIN` | IN (0xC1)* | wValue=pin, 1 byte response: status | Set RX GPIO pin (immediate response) |
| `0xE5` | `REQ_GET_SPDIF_RX_PIN` | IN (0xC1) | 1 byte: pin number (default 11) | Query current RX GPIO pin |

\* `REQ_SET_SPDIF_RX_PIN` uses IN direction (`0xC1` bmRequestType) with an immediate 1-byte status response. Status codes: `0x00`=success, `0x01`=invalid pin, `0x02`=pin in use, `0x04`=RX active.
