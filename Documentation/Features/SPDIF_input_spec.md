# S/PDIF Input Specification

*Last updated: 2026-11-16 (sections 1, 2, 4, 9, 12, 13 and the command summary: host interface on the paged protocol of firmware 1.1.7)*

## 1. Overview

DSPi supports several audio input sources, selected at runtime via vendor USB control transfers: **USB** (0), up to **four S/PDIF inputs** (1, 4, 5, 6), and **I2S** (2, documented separately in `i2s_input_spec.md`). Value 3 is the ADAT input. This document covers the S/PDIF inputs. When any S/PDIF input is the active source, received audio feeds through the identical DSP pipeline (preamp, loudness, master EQ, leveller, crossfeed, matrix mixer, per-output EQ, delay) as USB audio.

S/PDIF input 1 is always present. The three optional inputs (S/PDIF 2, 3 and 4) are disabled by default; they are enabled per-configuration by the host and share the single RX PIO state machine and DMA pair with input 1. Only one S/PDIF input is ever active at a time. See section 2 for the multi-input model.

Seven vendor commands on the INPUT page control S/PDIF input functionality (section 4):

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `INPUT.SOURCE` | 0x03 | 0x10 | GET, SET | none (0) | GET 1, SET 1 | Select or query the active input source |
| `INPUT.STATUS` | 0x03 | 0x11 | GET | none (0) | GET 12 | Active source, active channel count, measured S/PDIF rate in mHz, VCXO trim |
| `INPUT.SPDIF_RX_STATUS` | 0x03 | 0x20 | GET | none (0) | GET 16 | Receiver status struct |
| `INPUT.SPDIF_RX_CH_STATUS` | 0x03 | 0x21 | GET | none (0) | GET 24 | Raw IEC 60958 channel status |
| `INPUT.SPDIF_RX_PIN` | 0x03 | 0x22 | GET, SET | S/PDIF input 0-3 | GET 1, SET 1 | GPIO of one input in the payload, 0xFF restores its default |
| `INPUT.SPDIF_RX_ENABLE` | 0x03 | 0x23 | GET, SET | S/PDIF input 1-3 | GET 1, SET 1 | Enable of one optional input |
| `INPUT.SPDIF_RX_CONFIG` | 0x03 | 0x24 | GET | none (0) | GET 6 | Input inventory: count, enable mask, one pin per input |

In firmware 1.1.6 these were opcodes 0xE0 and 0xE1 (source SET and GET), 0xE2, 0xE3, 0xE4 and 0xE5 (pin SET and GET), 0xE9 (enable SET) and 0xEF (inventory). `INPUT.STATUS` collects what were `GET_STATUS` selectors 23, 32 and 33.

The input source enum and related definitions are in `audio_input.h`:

```c
// Input source identifiers (extensible; leave gaps for future types)
typedef enum {
    INPUT_SOURCE_USB    = 0,
    INPUT_SOURCE_SPDIF  = 1,   // SPDIF input 1 (always enabled)
    INPUT_SOURCE_I2S    = 2,
    INPUT_SOURCE_ADAT   = 3,   // 8-channel ADAT input (RP2350 only)
    INPUT_SOURCE_SPDIF2 = 4,   // Optional SPDIF input 2 (disabled until enabled by host)
    INPUT_SOURCE_SPDIF3 = 5,   // Optional SPDIF input 3 (disabled until enabled by host)
    INPUT_SOURCE_SPDIF4 = 6,   // Optional SPDIF input 4 (disabled until enabled by host)
} InputSource;

#define INPUT_SOURCE_MAX    INPUT_SOURCE_SPDIF4   // Highest valid value

// Number of selectable SPDIF inputs (input 1 always present; 2/3 optional).
#define SPDIF_RX_NUM_INPUTS 4

#define PICO_SPDIF_RX_PIN_DEFAULT  5    // SPDIF input 1 default GPIO
#define PICO_SPDIF_RX_PIN2_DEFAULT 20   // SPDIF input 2 default GPIO
#define PICO_SPDIF_RX_PIN3_DEFAULT 21   // SPDIF input 3 default GPIO
```

`input_source_valid()` accepts any `src <= INPUT_SOURCE_MAX`; ADAT (3) is structurally valid on both platforms so presets round-trip, and `input_source_selectable()` is what gates what the host may actually select.

---

## 2. Multiple SPDIF Inputs

DSPi exposes up to four S/PDIF inputs. Input 1 (`INPUT_SOURCE_SPDIF`, index 0) is always present and enabled. Inputs 2, 3 and 4 (`INPUT_SOURCE_SPDIF2` = 4 through `INPUT_SOURCE_SPDIF4` = 6, indices 1..3) are optional and disabled by default. All four share a single RX PIO state machine and DMA channel pair; only one input is ever active, and only the active input's GPIO is claimed in hardware.

The optional sources are contiguous from `INPUT_SOURCE_SPDIF2`; `spdif_index_for_source()` / `spdif_source_for_index()` are arithmetic, and a `_Static_assert` in `audio_input.h` holds that layout.

### Enable model and defaults

| Index | InputSource | Default GPIO | Default state |
|-------|-------------|--------------|---------------|
| 0 | `INPUT_SOURCE_SPDIF` (1) | 5 (`PICO_SPDIF_RX_PIN_DEFAULT`) | Always enabled (not disableable) |
| 1 | `INPUT_SOURCE_SPDIF2` (4) | 20 (`PICO_SPDIF_RX_PIN2_DEFAULT`) | Disabled |
| 2 | `INPUT_SOURCE_SPDIF3` (5) | 21 (`PICO_SPDIF_RX_PIN3_DEFAULT`) | Disabled |
| 3 | `INPUT_SOURCE_SPDIF4` (6) | 22 (`PICO_SPDIF_RX_PIN4_DEFAULT`) | Disabled |

The enable state of the optional inputs is held in a bit mask `spdif_rx_enabled_ext` (bit 0 = SPDIF 2 .. bit 2 = SPDIF 4; `SPDIF_RX_ENABLED_EXT_MASK` is the valid-bit mask). Their GPIOs live in `spdif_rx_pin_ext[3]`. Input 1's pin remains `spdif_rx_pin`. The helpers `spdif_input_enabled(idx)`, `spdif_rx_pin_for_index(idx)`, `spdif_rx_pin_default_for_index(idx)`, `spdif_index_for_source(src)`, and `spdif_source_for_index(idx)` (all in `audio_input.h`) map between the two representations.

A disabled optional input is not offered in the source list: `INPUT.SOURCE` validates the requested source with `input_source_selectable()` and refuses a disabled optional input with `VC_ERR_STATE`. Enable an optional input with `INPUT.SPDIF_RX_ENABLE` before selecting it.

### GPIO ownership rules

Each optional input's pin has three possible ownership states:

- **Disabled:** the stored pin is a preference only. It is **invisible** to every pin-conflict check, so another function (an output, MCK, a control interface, etc.) may freely use that GPIO. `pin_used_by_fixed_peripheral()` in `vendor/vc_pins.c` (mirrored in `dac_hw_mute.c`) only reserves an optional input's pin while that input is enabled.
- **Enabled but not active:** the pin is reserved exactly like input 1's pin. No other function may claim it, but the hardware RX library is not running on it.
- **Enabled and active:** the pin is claimed in hardware by the running RX library (muxed to the PIO input).

Because a disabled input's pin is invisible, enabling an input is where the conflict check happens. `spdif_input_enable_acceptable(idx)` requires the configured GPIO to be a valid pin and not in use by any other function; the enable command and the bulk/preset restore paths share this single check.

### Shared RX resources and switching

All four inputs multiplex onto one RX PIO state machine (RP2040: PIO1 SM2; RP2350: PIO2 SM0) and one DMA channel pair. Only the active input's GPIO is muxed to the PIO; the others are not driven. Switching between two S/PDIF inputs uses the **same deferred switch** as a USB-to-S/PDIF switch (`INPUT.SOURCE`, handled in the main loop):

1. Drain pending audio, engage the output mute, fence Core 1.
2. `spdif_input_stop()` on the old input; its GPIO is released.
3. Pipeline reset (output PIO dividers restored to nominal, filter coefficients recalculated).
4. `spdif_input_start()` on the new input's pin (`spdif_rx_active_pin()` resolves the active source's GPIO).
5. Outputs stay muted until the new input locks (see the state machine in section 3).

### GPIO release on stop

Historically the vendored `pico_spdif_rx` library left the RX pad muxed to the PIO after teardown, so the old GPIO was never actually freed. `spdif_input_stop()` now records the pin it started on (`spdif_active_data_pin`, captured in `spdif_input_start()`) and, on stop, resets it to `GPIO_FUNC_NULL` with the direction set back to input (mirroring `i2s_input_stop`). This is what lets a pin change or an input switch actually free the previous GPIO for reuse.

### Default channel names

Input-channel default names follow each source (`get_default_channel_name()` in `usb_audio.c`). Input 1 keeps the historical bare names "SPDIF L" / "SPDIF R". The optional inputs are numbered so a host can tell them apart:

| Input | Left channel | Right channel |
|-------|--------------|---------------|
| S/PDIF 1 | `SPDIF L` | `SPDIF R` |
| S/PDIF 2 | `SPDIF 2 L` | `SPDIF 2 R` |
| S/PDIF 3 | `SPDIF 3 L` | `SPDIF 3 R` |
| S/PDIF 4 | `SPDIF 4 L` | `SPDIF 4 R` |

A custom channel name set by the host is preserved across a source switch (only names that still equal the old default are relabelled).

### RP2040 GPIO 21 caveat

The S/PDIF 3 default GPIO is 21 on both platforms. On RP2040, GPIO 21 is also the MCK default (`PICO_I2S_MCK_PIN`). If MCK is enabled on GPIO 21, enabling S/PDIF 3 at its default pin is rejected (`VC_ERR_PIN_IN_USE`) because the enable-time check sees MCK occupying that pad. Repin S/PDIF 3 (or MCK) first, then enable. On RP2350 the MCK default is GPIO 13, so no default clash exists.

---

## 3. State Machine

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

## 4. Vendor Commands

All commands use the standard DSPi vendor control transfer format, in the paged protocol of firmware 1.1.7 (`../commands.md`). They are on the INPUT page (0x03).

- **bmRequestType:** `0x41` (Host-to-Device SET) or `0xC1` (Device-to-Host GET)
- **bRequest:** the command number
- **wIndex:** `(page << 8) | 2` (vendor interface number, `VENDOR_INTERFACE_NUMBER`), so `0x0302`
- **wValue:** the address (S/PDIF input index) or 0
- **Result:** an accepted SET ACKs. A refused SET, or a GET for a bad address, STALLs, and the reason is readable from `SYS.LAST_RESULT`. UART and I2C return the reason in the response status byte.
- **Timeout:** 1000 ms recommended for all commands

The handlers are in `firmware/DSPi/vendor/vc_page_input.c`. The pin checks they share with the restore paths are in `firmware/DSPi/vendor/vc_pins.c`.

---

### 4.1 `INPUT.SOURCE` SET

Switch the active input source.

**Direction:** Host -> Device (SET)
**bmRequestType:** `0x41`
**bRequest:** `0x10`
**wIndex:** `0x0302`
**wValue:** `0`
**wLength:** `1`

#### Request payload (1 byte)

| Offset | Size | Type | Field | Values |
|--------|------|------|-------|--------|
| 0 | 1 | uint8_t | `source` | `0` = USB, `1` = S/PDIF 1, `2` = I2S, `3` = ADAT, `4` = S/PDIF 2, `5` = S/PDIF 3, `6` = S/PDIF 4 |

Values `4` to `6` are only accepted when the corresponding optional S/PDIF input is enabled. Value `3` (ADAT) needs an enabled ADAT input on RP2350 and is refused on RP2040.

#### Behavior

This command returns immediately (non-blocking). The actual source switch is deferred to the firmware main loop via a pending flag:

1. Firmware validates the source value with `input_source_valid()` and `input_source_selectable()`, and checks it differs from the current source.
2. If selectable and different, sets `input_source_change_pending = true`.
3. Main loop detects the flag and executes the switch sequence:
   - Drains any pending USB audio data
   - Mutes output for 256 samples (~5 ms at 48 kHz)
   - Stops old source hardware (if SPDIF, calls `spdif_input_stop()`, releasing its GPIO)
   - Updates `active_input_source`
   - Starts new source hardware (if SPDIF, calls `spdif_input_start()` on the new input's pin)
   - For USB: flushes stale ring data and unmutes immediately
   - For SPDIF: output remains muted until lock is acquired (see state machine)

A S/PDIF-to-S/PDIF switch takes the same stop/reset/start path (stop old input, pipeline reset, start on the new input's GPIO), with outputs muted until the new input locks; it is not special-cased.

#### Error handling

| Condition | Firmware behavior |
|-----------|-------------------|
| `source` above 6 | STALL, `VC_ERR_VALUE`. No action |
| `source` is ADAT on RP2040 | STALL, `VC_ERR_UNSUPPORTED` |
| `source` is a disabled optional S/PDIF input (or a disabled ADAT input) | STALL, `VC_ERR_STATE` (`input_source_selectable()` returns false) |
| Payload not exactly 1 byte | STALL, `VC_ERR_LENGTH` |
| `source == active_input_source` | ACK, no action |
| No SPDIF signal connected | Switch proceeds; output stays muted in ACQUIRING state indefinitely |
| SPDIF signal at unsupported rate | Switch proceeds; output stays muted in RELOCKING state |

Firmware 1.1.6 ACKed every one of these and silently dropped the invalid ones. An ACK still says only that the switch was queued. Use `INPUT.SPDIF_RX_STATUS` to monitor the outcome.

#### Hex example

Switch to S/PDIF input 1:
```
bmRequestType: 0x41
bRequest:      0x10
wValue:        0x0000
wIndex:        0x0302
wLength:       0x0001
Data:          01
```

Switch to S/PDIF input 2 (must be enabled first):
```
bmRequestType: 0x41
bRequest:      0x10
wValue:        0x0000
wIndex:        0x0302
wLength:       0x0001
Data:          04
```

Switch to USB input:
```
bmRequestType: 0x41
bRequest:      0x10
wValue:        0x0000
wIndex:        0x0302
wLength:       0x0001
Data:          00
```

---

### 4.2 `INPUT.SOURCE` GET

Query the currently active input source.

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0x10`
**wIndex:** `0x0302`
**wValue:** `0`
**wLength:** `1`

#### Response (1 byte)

| Offset | Size | Type | Field | Values |
|--------|------|------|-------|--------|
| 0 | 1 | uint8_t | `source` | `0` = USB, `1` = S/PDIF 1, `2` = I2S, `3` = ADAT, `4` = S/PDIF 2, `5` = S/PDIF 3, `6` = S/PDIF 4 |

#### Notes

- Returns the current value of `active_input_source`.
- If a source switch is pending (deferred to main loop), this returns the source that was active **before** the switch. The switch completes asynchronously.
- At boot, defaults to `0` (USB) unless a preset with SPDIF input was loaded.
- `INPUT.STATUS` (12 bytes) returns the same source at offset 0, plus the active input channel count (u8, offset 1), the measured S/PDIF rate in mHz (u32, offset 4) and the soft VCXO trim in ppm x100 (i32, offset 8). In firmware 1.1.6 the last three were `GET_STATUS` selectors 23, 32 and 33.

#### Hex example

```
bmRequestType: 0xC1
bRequest:      0x10
wValue:        0x0000
wIndex:        0x0302
wLength:       0x0001
Response:      01           (S/PDIF is active)
```

---

### 4.3 `INPUT.SPDIF_RX_STATUS`

Query the S/PDIF receiver status. Returns a 16-byte packed struct.

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0x20`
**wIndex:** `0x0302`
**wValue:** `0`
**wLength:** `16`

#### Response: SpdifRxStatusPacket (16 bytes)

Defined in `spdif_input.h`:

```c
typedef struct __attribute__((packed)) {
    uint8_t  state;              // SpdifInputState enum (0-3)
    uint8_t  input_source;       // Current active InputSource enum (0-6)
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
| 1 | 1 | uint8_t | `input_source` | Current `InputSource` enum (`0`=USB, `1`=S/PDIF 1, `2`=I2S, `3`=ADAT, `4`=S/PDIF 2, `5`=S/PDIF 3, `6`=S/PDIF 4) |
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

### 4.4 `INPUT.SPDIF_RX_CH_STATUS`

Retrieve the raw IEC 60958-3 channel status bits from the received S/PDIF stream.

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0x21`
**wIndex:** `0x0302`
**wValue:** `0`
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
bRequest:      0x21
wValue:        0x0000
wIndex:        0x0302
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

### 4.5 `INPUT.SPDIF_RX_PIN` SET

Set the GPIO pin used for one of the four S/PDIF inputs. The input index is the address in `wValue`. The GPIO number is the 1-byte payload, and `0xFF` (`PIN_RESET_TO_DEFAULT`) restores that input's default pin.

**Direction:** Host -> Device (SET)
**bmRequestType:** `0x41`
**bRequest:** `0x22`
**wIndex:** `0x0302`
**wValue:** index `0..3`
**wLength:** `1`
**Payload:** 1 byte, the GPIO

In firmware 1.1.6 this was opcode 0xE4, an IN transfer with `(index << 8) | GPIO` packed into `wValue` and a 1-byte status reply. In 1.1.7 it is an ordinary SET with no status byte. There is no bare-pin form, so a host always sends the index.

#### Result

| Result | Old status byte | Meaning |
|--------|-----------------|---------|
| ACK (`VC_OK`) | `0x00` `PIN_CONFIG_SUCCESS` | Pin changed, or already set to this value |
| `VC_ERR_PIN_INVALID` | `0x01` `PIN_CONFIG_INVALID_PIN` | Pin is out of range or board-reserved |
| `VC_ERR_PIN_IN_USE` | `0x02` `PIN_CONFIG_PIN_IN_USE` | Pin is in use by another function (only checked when the target input is enabled) |
| `VC_ERR_ADDRESS` | `0x03` `PIN_CONFIG_INVALID_OUTPUT` | Index is out of range (`index >= 4`), or `wValue` has a high byte set |
| `VC_ERR_LENGTH` | none | Payload not exactly 1 byte |

There is no `PIN_CONFIG_OUTPUT_ACTIVE` style refusal: the pin is **hot-swappable** while the input is active.

#### Behavior

- The pin is validated and stored in RAM (`spdif_rx_pin` for index 0, `spdif_rx_pin_ext[index-1]` for 1 to 3).
- `PIN_IN_USE` is only checked when the target input is **enabled**. A disabled optional input's pin is a stored preference; its conflicts are validated at enable time, not here, so you can preconfigure a disabled input's pin freely.
- If the changed index is the currently active source, the change is hot-swapped: the handler sets `spdif_rx_pin_change_pending`, and the main loop stops the RX library, runs a pipeline reset, and restarts on the new pin. Outputs are muted until the RX re-locks.
- A change pushes a `NOTIFY_EVT_PARAM_CHANGED` on the pin's `WireBulkParams` offset.

#### Persistence

The update is **RAM-only**. It is not written to flash until a preset save (`STORE.PRESET_SAVE`), which captures it slot-scoped (and, under `output_config_mode = INDEPENDENT`, into the device-global directory baseline, also written by `STORE.OUTPUT_CONFIG_SAVE`). This replaces the previous behavior where the pin was written to the preset directory immediately.

#### Validation logic (`vc_page_input.c`)

```c
static VcResult input_spdif_rx_pin_set(VcCtx *c) {
    uint8_t index = (uint8_t)c->addr;          // already range-checked by the dispatcher
    uint8_t new_pin = vc_in_u8(c, 0);
    if (new_pin == PIN_RESET_TO_DEFAULT) new_pin = spdif_rx_pin_default_for_index(index);
    if (!PIN_VALID_FOR(DSPI_PIN_FN_SPDIF_RX, new_pin)) return VC_ERR_PIN_INVALID;
    if (new_pin == spdif_rx_pin_for_index(index)) return VC_OK;
    // A disabled optional input stores its pin as a preference; enable re-checks it.
    if (spdif_input_enabled(index) && is_pin_in_use(new_pin, 0xFF)) return VC_ERR_PIN_IN_USE;

    if (index == 0) spdif_rx_pin = new_pin;
    else            spdif_rx_pin_ext[index - 1] = new_pin;
    if (input_source_is_spdif(active_input_source) &&
        spdif_index_for_source(active_input_source) == index)
        spdif_rx_pin_change_pending = true;    // hot swap on the main loop
    // ... notify_param_write of the changed wire offset ...
    return VC_OK;
}
```

#### GPIO pin constraints

See section 9 for the full pin rules. For an enabled input, the pin must be a valid GPIO (0-28 on RP2040, 0-29 on RP2350; 23-25 excluded) and not conflict with any output, MCK, an enabled S/PDIF input, the DAC hardware-mute, a control interface, active I2S clocks, or an active I2S RX data pin. Defaults: GPIO 5 / 20 / 21 / 22 for inputs 1 / 2 / 3 / 4.

#### Hex examples

Set S/PDIF input 1 (index 0) to GPIO 10:
```
bmRequestType: 0x41
bRequest:      0x22
wValue:        0x0000  (index 0)
wIndex:        0x0302
wLength:       0x0001
Data:          0A       (GPIO 10)
Result:        ACK
```

Set S/PDIF input 2 (index 1) to GPIO 20:
```
bmRequestType: 0x41
bRequest:      0x22
wValue:        0x0001  (index 1)
wIndex:        0x0302
wLength:       0x0001
Data:          14       (GPIO 20)
Result:        ACK
```

Restore S/PDIF input 1 to its default pin:
```
bmRequestType: 0x41
bRequest:      0x22
wValue:        0x0000  (index 0)
wIndex:        0x0302
wLength:       0x0001
Data:          FF
Result:        ACK
```

Bad index:
```
bmRequestType: 0x41
bRequest:      0x22
wValue:        0x0004  (index 4)
wIndex:        0x0302
wLength:       0x0001
Data:          0A
Result:        STALL, SYS.LAST_RESULT = VC_ERR_ADDRESS
```

---

### 4.6 `INPUT.SPDIF_RX_PIN` GET

Query the GPIO pin configured for one of the four S/PDIF inputs. The input is selected by `wValue` (index `0..3`).

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0x22`
**wIndex:** `0x0302`
**wValue:** index (`0..3`)
**wLength:** `1`

#### Response (1 byte)

| Offset | Size | Type | Field | Values |
|--------|------|------|-------|--------|
| 0 | 1 | uint8_t | `pin` | GPIO of the addressed input |

An out-of-range index (`>= 4`) STALLs with `VC_ERR_ADDRESS`. Firmware 1.1.6 returned `0` instead.

#### Hex example

Query S/PDIF input 2 (index 1) pin:
```
bmRequestType: 0xC1
bRequest:      0x22
wValue:        0x0001  (index 1)
wIndex:        0x0302
wLength:       0x0001
Response:      14       (GPIO 20)
```

---

### 4.7 `INPUT.SPDIF_RX_ENABLE`

Enable or disable one of the three optional S/PDIF inputs, or read whether it is enabled. Input 1 is always enabled and is not addressable.

**Direction:** Host -> Device (SET) or Device -> Host (GET)
**bmRequestType:** `0x41` (SET) or `0xC1` (GET)
**bRequest:** `0x23`
**wIndex:** `0x0302`
**wValue:** index `1..3`
**wLength:** `1`
**Payload / response:** 1 byte, 0 = disabled, non-zero = enabled (the GET returns 0 or 1)

In firmware 1.1.6 this was opcode 0xE9, an IN transfer with `(index << 8) | enable` in `wValue` and a 1-byte status reply. It had no GET. In 1.1.7 it is an ordinary SET with the enable in the payload, and the GET is new.

#### Result

| Result | Old status byte | Meaning |
|--------|-----------------|---------|
| ACK (`VC_OK`) | `0x00` `PIN_CONFIG_SUCCESS` | Enable/disable applied, or already in the requested state |
| `VC_ERR_PIN_INVALID` | `0x02` | Enable: the configured pin is not usable for S/PDIF RX |
| `VC_ERR_PIN_IN_USE` | `0x02` | Enable: the configured pin is taken by another function |
| `VC_ERR_STATE` | `0x02` | Disable: the input is the active source or a pending switch target |
| `VC_ERR_ADDRESS` | `0x03` (`0x00` for enabling input 1) | Index 0 (input 1) or index out of range, in either direction |
| `VC_ERR_LENGTH` | none | Payload not exactly 1 byte |

1.1.6 reported all three enable and disable refusals as `0x02`. The paged command tells them apart.

#### Behavior

| Case | Result |
|------|--------|
| index 0, either value or a GET | `VC_ERR_ADDRESS` (input 1 is always enabled and is not addressed by this command; 1.1.6 answered success for enabling it) |
| index `>= 4` | `VC_ERR_ADDRESS` |
| enable already matches current state | ACK, nothing changes |
| Enable, pin OK | Sets the mask bit; ACK |
| Enable, pin invalid | `VC_ERR_PIN_INVALID`, input left disabled |
| Enable, pin taken | `VC_ERR_PIN_IN_USE`, input left disabled |
| Disable, input is active source or pending switch target | `VC_ERR_STATE` (host must switch away first) |
| Disable, otherwise | Clears the mask bit; ACK |

Enabling validates the configured pin (valid GPIO for S/PDIF RX, and not otherwise in use), the same check the restore paths make with `spdif_input_enable_acceptable()`. The change is **RAM-only** and persisted on a preset save (`STORE.PRESET_SAVE`), exactly like `INPUT.SPDIF_RX_PIN`. A change pushes a `NOTIFY_EVT_PARAM_CHANGED` on `input_config.spdif_rx_enabled_ext_p1`.

#### Validation logic (`vc_page_input.c`)

```c
static VcResult input_spdif_rx_enable_set(VcCtx *c) {
    uint8_t index = (uint8_t)c->addr;
    uint8_t enable = vc_in_u8(c, 0) ? 1 : 0;
    if (index == 0) return VC_ERR_ADDRESS;
    if (enable == (spdif_input_enabled(index) ? 1 : 0)) return VC_OK;

    if (enable) {
        uint8_t pin = spdif_rx_pin_for_index(index);
        if (!PIN_VALID_FOR(DSPI_PIN_FN_SPDIF_RX, pin)) return VC_ERR_PIN_INVALID;
        if (is_pin_in_use(pin, 0xFF)) return VC_ERR_PIN_IN_USE;
        spdif_rx_enabled_ext |= (uint8_t)(1u << (index - 1));
    } else {
        // The host must switch the source away first.
        bool is_active = input_source_is_spdif(active_input_source) &&
                         spdif_index_for_source(active_input_source) == index;
        bool is_pending = input_source_change_pending &&
                          pending_input_source == spdif_source_for_index(index);
        if (is_active || is_pending) return VC_ERR_STATE;
        spdif_rx_enabled_ext &= (uint8_t)~(1u << (index - 1));
    }
    // ... notify_param_write(spdif_rx_enabled_ext_p1 ...) ...
    return VC_OK;
}
```

#### Hex examples

Enable S/PDIF input 2 (index 1):
```
bmRequestType: 0x41
bRequest:      0x23
wValue:        0x0001  (index 1)
wIndex:        0x0302
wLength:       0x0001
Data:          01
Result:        ACK
```

Attempt to disable S/PDIF input 2 while it is the active source:
```
bmRequestType: 0x41
bRequest:      0x23
wValue:        0x0001  (index 1)
wIndex:        0x0302
wLength:       0x0001
Data:          00
Result:        STALL, SYS.LAST_RESULT = VC_ERR_STATE (switch the source away first)
```

Attempt to disable input 1:
```
bmRequestType: 0x41
bRequest:      0x23
wValue:        0x0000  (index 0)
wIndex:        0x0302
wLength:       0x0001
Data:          00
Result:        STALL, SYS.LAST_RESULT = VC_ERR_ADDRESS
```

---

### 4.8 `INPUT.SPDIF_RX_CONFIG`

Query the whole S/PDIF input inventory in one 6-byte read, so a host can build its source list data-driven (which optional inputs exist, which are enabled, and each input's pin).

**Direction:** Device -> Host (GET)
**bmRequestType:** `0xC1`
**bRequest:** `0x24`
**wIndex:** `0x0302`
**wValue:** `0`
**wLength:** `6`

#### Response (6 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | `count` | Number of S/PDIF inputs (`SPDIF_RX_NUM_INPUTS` = 4) |
| 1 | 1 | `enable_mask` | Enable mask over all inputs; bit 0 = input 1 (always set), bit 1 = S/PDIF 2, bit 2 = S/PDIF 3, bit 3 = S/PDIF 4 |
| 2 | 1 | `pin[0]` | GPIO of input 1 |
| 3 | 1 | `pin[1]` | GPIO of input 2 |
| 4 | 1 | `pin[2]` | GPIO of input 3 |
| 5 | 1 | `pin[3]` | GPIO of input 4 |

The enable mask is `(spdif_rx_enabled_ext << 1) | 1`; bit 0 is hard-set because input 1 is always enabled.

#### Firmware implementation (`vc_page_input.c`)

```c
static VcResult input_spdif_rx_config_get(VcCtx *c) {
    c->out[0] = SPDIF_RX_NUM_INPUTS;
    c->out[1] = (uint8_t)((spdif_rx_enabled_ext << 1) | 1);
    for (uint8_t i = 0; i < SPDIF_RX_NUM_INPUTS; i++)
        c->out[2 + i] = spdif_rx_pin_for_index(i);
    return VC_OK;
}
```

#### Hex example

Factory defaults (only input 1 enabled; pins 5 / 20 / 21 / 22):
```
bmRequestType: 0xC1
bRequest:      0x24
wValue:        0x0000
wIndex:        0x0302
wLength:       0x0006
Response:      04 01 05 14 15 16

Interpretation:
  04 = 4 inputs
  01 = enable mask 0b0001 (input 1 only; bit 0 always set)
  05 = input 1 GPIO 5
  14 = input 2 GPIO 20
  15 = input 3 GPIO 21
  16 = input 4 GPIO 22
```

With S/PDIF 2 additionally enabled, byte 1 becomes `03` (0b0011).

---

## 5. Audio Data Format

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

## 6. Clock Servo

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

## 7. Supported Sample Rates

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

## 8. Errors & Recovery

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

## 9. GPIO Constraints

Each S/PDIF RX pin follows the same GPIO validation rules as the rest of DSPi, but with per-input semantics: an input's pin is only reserved when that input is enabled.

### Valid pin range

`is_valid_gpio_pin()` in `vendor/vc_pins.c` (S/PDIF RX goes through `PIN_VALID_FOR(DSPI_PIN_FN_SPDIF_RX, pin)`, which is this check on boards without a platform pin table):

| Platform | Valid range |
|----------|------------|
| RP2040 | GPIO 0-28 |
| RP2350 | GPIO 0-29 |

### Always excluded

| GPIO | Reason |
|------|--------|
| 23 | Power control (SMPS mode) |
| 24 | Power control (VBUS detect) |
| 25 | On-board LED |

Only GPIO 23-25 are board-reserved. GPIO 12 and GPIO 16/17 are **not** excluded (the debug UART was removed); when a control interface claims 16/17 at runtime, `is_pin_in_use()` covers it dynamically.

### Per-input reservation

- **Input 1** always reserves its pin (`pin_used_by_fixed_peripheral()` treats `spdif_rx_pin` as always claimed).
- **Optional inputs 2/3** reserve their pin **only while enabled**. A disabled input's stored pin is invisible to conflict checks, so another function may use that GPIO.

### Must not conflict with (for an enabled input)

- Any S/PDIF output data pin (slot 0 through `NUM_PIN_OUTPUTS - 1`)
- I2S BCK pin or LRCLK pin (BCK + 1), while any slot is I2S output or I2S is the active input
- I2S MCK pin, if MCK is enabled
- Any other enabled S/PDIF input's pin
- The DAC hardware-mute pin
- A live UART / I2C / Control-Surfaces pin
- An active I2S RX data pin

The firmware checks these via `is_pin_in_use()` (for the pin set on an enabled input) and `spdif_input_enable_acceptable()` (at enable time).

### Hot-swap behavior

The pin **can** be changed while the S/PDIF input is active. `INPUT.SPDIF_RX_PIN` accepts the change, and if the changed index is the active source the firmware hot-swaps it: stop RX, pipeline reset, restart on the new pin, outputs muted until re-lock. There is no "switch to USB first" restriction; the old `PIN_CONFIG_OUTPUT_ACTIVE` rejection was removed. If the pin already matches, the command ACKs and changes nothing.

### Persistence

Pin and enable changes are RAM-only until a preset save, `STORE.PRESET_SAVE` (slot-scoped, plus a device-global directory baseline under `output_config_mode = INDEPENDENT`). On first boot the defaults are GPIO 5 / 20 / 21 / 22 for inputs 1 / 2 / 3 / 4, with inputs 2..4 disabled. See section 10.

---

## 10. Preset Integration

The physical IO config (output pins/types, I2S clocks, and all S/PDIF RX pins/enables) travels through the `output_config_mode` mechanism. In `OUTPUT_CONFIG_MODE_WITH_PRESET` (the default) it lives in each preset slot; in `OUTPUT_CONFIG_MODE_INDEPENDENT` it is a device-global block in the directory, applied at boot. A single snapshot/apply path (`io_config_from_slot` / `io_config_from_live` / `io_config_apply` in `flash_storage.c`) serves both sources so they cannot diverge. The **input source** selection (USB vs S/PDIF vs I2S) is always per-preset, not part of that block.

### Flash storage (SLOT_DATA_VERSION = 35)

Relevant `PresetSlot` fields (`flash_storage.c`):

```c
uint8_t input_source;            // InputSource enum (V13; 0=USB, 1=SPDIF, ...)
uint8_t spdif_rx_pin;            // SPDIF RX 1 GPIO (V13; 0 = absent -> use default)
// ...
uint8_t spdif_rx_enabled_ext;    // Optional SPDIF enable mask (V24; bit0=SPDIF2 .. bit2=SPDIF4)
uint8_t spdif_rx_pin_ext[2];     // SPDIF RX 2/3 GPIOs (V24; 0 = unset -> defaults 20/21)
uint8_t spdif_rx_pin4;           // SPDIF RX 4 GPIO (V35 tail-append; 0 = unset -> default 22)
```

- `input_source` and `spdif_rx_pin` were added at V13.
- `spdif_rx_enabled_ext` + `spdif_rx_pin_ext[2]` were appended at V24 (slot grows by 3 bytes). Pre-V24 slots do not carry them; `io_config_from_slot()` gates the read on `slot->version >= 24`, so **older slots load with S/PDIF 2/3 disabled** (baselined from the device-global directory config). A slot pin of `0` means "unset, use the live/default pin".
- `spdif_rx_pin4` was appended at V35 (slot grows by 1 byte), gated on `slot->version >= 35`. S/PDIF 4's *enable* bit needed no new storage: it is bit 2 of the V24 `spdif_rx_enabled_ext` byte, which pre-V35 firmware always wrote as 0. Because the slot's mask fully overrides the device-global one, loading a V24..V34 preset disables S/PDIF 4 — the same rule that already applied to inputs 2/3.

| Action | Behavior |
|--------|----------|
| **Save preset** | Current `active_input_source` and the live pins/enable mask are captured into the slot |
| **Load preset** | Input source switch is deferred (via `input_source_selectable()`); pins/enables applied through `io_config_apply()`, with a hot-swap if the active input's pin changed |
| **Factory reset / boot with no preset** | USB input; pins 5/20/21/22; optional S/PDIF inputs disabled |

### Device-global IO config (INDEPENDENT mode) and directory V16

Under INDEPENDENT mode the IO config lives in the `FlashOutputConfig` block inside the directory. `spdif_rx_enabled_ext` + `spdif_rx_pin_ext[2]` were appended at directory V12 (25 -> 28 bytes) and `spdif_rx_pin4` at directory **V16** (34 -> 35 bytes, `DIR_VERSION_CURRENT = 16`). Input 4's pin is a tail byte rather than a third `spdif_rx_pin_ext` entry because the frozen `FlashOutputConfig_v11..v15` structs are strict prefixes that the older migrations copy forward; `cfg_spdif_ext_pin()` / `cfg_spdif_ext_pin_get()` in `flash_storage.c` index the split storage as one array. Older directories are read through their frozen structs and migrated forward, defaulting the optional inputs to disabled with pins 20/21/22.

### Wire format (WIRE_FORMAT_VERSION = 28)

The bulk parameter transfer includes a `WireInputConfig` section (16 bytes). The optional-SPDIF fields are claimed from its reserved bytes, so the section size is unchanged. Widening `spdif_rx_pin_ext` from 2 to 3 entries for input 4 consumed that section's last reserved byte and shifted the fields below it down one, which is why the format version is now 28; no later wire section moved:

```c
typedef struct __attribute__((packed)) {
    uint8_t  input_source;           // InputSource enum (0=USB, 1=SPDIF, 2=I2S)
    uint8_t  spdif_rx_pin;          // SPDIF RX GPIO pin (applied on SET when apply_pins=true)
    uint8_t  i2s_rx_pin;             // I2S RX data GPIO, stereo pair 0 (V12+)
    uint8_t  i2s_input_rate;         // I2S input rate enum: 0=44100, 1=48000, 2=96000 (V12+)
    uint8_t  i2s_input_channels;     // Active I2S input channels: 2/4/6/8 (0 = absent)
    uint8_t  i2s_rx_pin_ext[3];      // I2S RX data GPIOs for stereo pairs 1..3 (0 = unset)
    uint8_t  spdif_rx_pin_ext[3];    // SPDIF RX 2/3/4 GPIOs (0 = absent, keep live)
    uint8_t  spdif_rx_enabled_ext_p1;// SPDIF 2/3/4 enable mask + 1 (0 = absent;
                                     // 1 = all disabled, 2 = SPDIF2, 3 = 2+3, ...)
    uint8_t  i2s_clock_mode;         // I2S clock: 0=master, 1=slave (V21+)
    uint8_t  adat_input_pin;         // ADAT RX GPIO (V24+; 0 = absent, keep live)
    uint8_t  adat_input_enabled_p1;  // enable + 1 (0 absent, 1 disabled, 2 enabled)
    uint8_t  adat_clock_mode_p1;     // clock mode + 1 (0 absent, 1 master, 2 slave)
} WireInputConfig;                   // 16 bytes (full; no reserved bytes left)
```

Sentinel semantics on SET (`bulk_params_apply`, with `apply_pins`):

- `spdif_rx_pin` / `spdif_rx_pin_ext[i]` = `0` means "absent, keep the live pin"; a non-zero valid pin is applied (hot-swapped if it is the active input). Unlike the obsolete V7 behavior, `spdif_rx_pin` IS applied on SET when `apply_pins` is true.
- `spdif_rx_enabled_ext_p1` is the enable mask **plus one**: `0` = "field absent, keep the live mask"; otherwise `mask = enc - 1`. The +1 encoding lets old hosts (which push zeros) mean "absent" rather than "disable all".
- Bulk apply guards: the ext pins are applied before the enable mask (so a pin+enable pushed together validates against the new pin); a newly enabled bit is accepted only if `spdif_input_enable_acceptable()` passes; a newly disabled bit that names the **live source** is refused (a pushed config must not silently kill the running input).

### Mute during preset load

When a preset load switches the input source, the standard 256-sample (~5 ms) mute applies during the pipeline reset. Switching to any S/PDIF input adds mute time during lock acquisition.

---

## 11. Platform Differences

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| SPDIF RX PIO block | PIO1 | PIO2 |
| SPDIF RX PIO SM | SM2 (SM0=PDM, SM1=MCK occupied) | SM0 (dedicated PIO block) |
| RX DMA IRQ | DMA_IRQ_1 (shared with SPDIF TX) | DMA_IRQ_1 (shared with SPDIF TX) |
| RX DMA channels | CH4, CH5 | CH5, CH6 |
| Internal audio format | Q28 fixed-point (32-bit) | IEEE 754 float |
| Sample conversion | `raw >> 4` then `fast_mul_q28()` | `raw * inv_2147483648 * preamp` |
| Supported input rates | 44.1, 48, 96 kHz | 44.1, 48, 96 kHz |
| Input bit depth | 24-bit | 24-bit |
| Selectable SPDIF inputs | 3 (share one RX SM/DMA) | 3 (share one RX SM/DMA) |
| Default RX pins (inputs 1/2/3) | GPIO 5 / 20 / 21 | GPIO 5 / 20 / 21 |
| MCK default pin | GPIO 21 (clashes with S/PDIF 3 default) | GPIO 13 |
| SPDIF output slots | 2 (4 channels) | 4 (8 channels) |

DMA IRQ assignment: SPDIF RX shares DMA_IRQ_1 with SPDIF TX (`PICO_SPDIF_RX_DMA_IRQ = 1`, `PICO_AUDIO_SPDIF_DMA_IRQ = 1`); I2S TX uses DMA_IRQ_0. The library patches (private `irq_handler_registered` flag, removed `irq_set_enabled` in teardown, interrupt-safe `_spdif_rx_common_end`) let RX and SPDIF TX coexist safely on the same line.

**RP2040 GPIO 21 caveat:** S/PDIF 3's default GPIO (21) is also the RP2040 MCK default. If MCK is enabled there, enabling S/PDIF 3 at its default pin is rejected (`PIN_IN_USE`) until S/PDIF 3 (or MCK) is repinned. RP2350 has no such default clash (MCK default is GPIO 13).

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

## 12. Example App Integration

### C struct definitions (for parsing responses)

```c
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

// Input source identifiers
#define INPUT_SOURCE_USB     0
#define INPUT_SOURCE_SPDIF   1
#define INPUT_SOURCE_I2S     2
#define INPUT_SOURCE_SPDIF2  4
#define INPUT_SOURCE_SPDIF3  5
#define INPUT_SOURCE_SPDIF4  6

// SPDIF receiver states
#define SPDIF_STATE_INACTIVE   0
#define SPDIF_STATE_ACQUIRING  1
#define SPDIF_STATE_LOCKED     2
#define SPDIF_STATE_RELOCKING  3

// Paged protocol: page in the wIndex high byte, command in bRequest
#define VENDOR_INTF  2
#define PAGE_SYS     0x01
#define PAGE_INPUT   0x03
#define WINDEX(page) ((uint16_t)(((page) << 8) | VENDOR_INTF))

// INPUT page commands
#define INPUT_SOURCE              0x10   // GET, SET
#define INPUT_SPDIF_RX_STATUS     0x20   // GET
#define INPUT_SPDIF_RX_CH_STATUS  0x21   // GET
#define INPUT_SPDIF_RX_PIN        0x22   // GET, SET; wValue = input index 0..3
#define INPUT_SPDIF_RX_ENABLE     0x23   // GET, SET; wValue = input index 1..3
#define INPUT_SPDIF_RX_CONFIG     0x24   // GET
#define SYS_LAST_RESULT           0x14   // GET, 8 bytes

// Result codes (SYS.LAST_RESULT byte 3)
#define VC_OK                0x00
#define VC_ERR_LENGTH        0x12
#define VC_ERR_ADDRESS       0x13
#define VC_ERR_VALUE         0x14
#define VC_ERR_UNSUPPORTED   0x15
#define VC_ERR_STATE         0x18
#define VC_ERR_PIN_INVALID   0x19
#define VC_ERR_PIN_IN_USE    0x1A

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

// After a STALL, read why. Reply: u16 command ID, u8 is_get, u8 result, u16 wValue, 2 reserved.
// LAST_RESULT is per transport, so check the ID: another USB host may have overwritten it.
int last_result(libusb_device_handle *handle, uint16_t expect_id) {
    uint8_t r[8];
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        SYS_LAST_RESULT, 0, WINDEX(PAGE_SYS), r, sizeof(r), 1000);
    if (ret != 8 || (uint16_t)(r[0] | (r[1] << 8)) != expect_id) return -1;
    return r[3];
}
```

### Switch to S/PDIF input

```c
int switch_to_spdif(libusb_device_handle *handle) {
    uint8_t source = INPUT_SOURCE_SPDIF;
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        INPUT_SOURCE,           // 0x10
        0,                      // wValue (no address)
        WINDEX(PAGE_INPUT),     // wIndex = 0x0302
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
        INPUT_SOURCE,
        0, WINDEX(PAGE_INPUT),
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
        INPUT_SOURCE,           // 0x10, IN
        0, WINDEX(PAGE_INPUT),
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
        INPUT_SPDIF_RX_STATUS,     // 0x20
        0, WINDEX(PAGE_INPUT),
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
        INPUT_SPDIF_RX_CH_STATUS,     // 0x21
        0, WINDEX(PAGE_INPUT),
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
// index: 0 = SPDIF 1 .. 3 = SPDIF 4.  The index is the wValue address and the
// GPIO is the 1-byte payload (0xFF restores the default).  The pin is hot-swapped
// if it belongs to the active source; there is no "input active" rejection.
int set_spdif_rx_pin(libusb_device_handle *handle, uint8_t index, uint8_t pin) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        INPUT_SPDIF_RX_PIN,      // 0x22
        index,                   // wValue = input index
        WINDEX(PAGE_INPUT),
        &pin, 1, 1000);

    if (ret == 1) {
        printf("SPDIF input %u pin set to GPIO %u\n", index + 1, pin);
        return 0;
    }
    if (ret != LIBUSB_ERROR_PIPE) {
        fprintf(stderr, "Failed to set RX pin: %s\n", libusb_error_name(ret));
        return -1;
    }
    switch (last_result(handle, (PAGE_INPUT << 8) | INPUT_SPDIF_RX_PIN)) {
        case VC_ERR_PIN_INVALID:
            fprintf(stderr, "GPIO %u is not a valid pin\n", pin);
            return -1;
        case VC_ERR_PIN_IN_USE:
            fprintf(stderr, "GPIO %u is already in use by another function\n", pin);
            return -1;
        case VC_ERR_ADDRESS:
            fprintf(stderr, "No such SPDIF input index %u\n", index);
            return -1;
        default:
            fprintf(stderr, "RX pin refused\n");
            return -1;
    }
}

// index: 0..3.  An out-of-range index STALLs (VC_ERR_ADDRESS).
int get_spdif_rx_pin(libusb_device_handle *handle, uint8_t index, uint8_t *out_pin) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        INPUT_SPDIF_RX_PIN,      // 0x22
        index, WINDEX(PAGE_INPUT),
        out_pin, 1, 1000);

    if (ret != 1) {
        fprintf(stderr, "Failed to get RX pin: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}

// Enable (enable=1) or disable (enable=0) an optional SPDIF input.
// index: 1 = SPDIF 2 .. 3 = SPDIF 4.  Input 1 (index 0) is always enabled and not addressable.
int set_spdif_input_enable(libusb_device_handle *handle, uint8_t index,
                           uint8_t enable) {
    uint8_t value = enable ? 1 : 0;
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        INPUT_SPDIF_RX_ENABLE,   // 0x23
        index, WINDEX(PAGE_INPUT),
        &value, 1, 1000);

    if (ret == 1) {
        printf("SPDIF input %u %s\n", index + 1, enable ? "enabled" : "disabled");
        return 0;
    }
    if (ret != LIBUSB_ERROR_PIPE) {
        fprintf(stderr, "Failed to set SPDIF enable: %s\n", libusb_error_name(ret));
        return -1;
    }
    switch (last_result(handle, (PAGE_INPUT << 8) | INPUT_SPDIF_RX_ENABLE)) {
        case VC_ERR_PIN_INVALID:
        case VC_ERR_PIN_IN_USE:
            fprintf(stderr, "SPDIF input %u: configured pin invalid or already claimed\n",
                    index + 1);
            return -1;
        case VC_ERR_STATE:
            fprintf(stderr, "SPDIF input %u is the active source (switch away first)\n",
                    index + 1);
            return -1;
        case VC_ERR_ADDRESS:
            fprintf(stderr, "SPDIF input %u: bad index, or input 1 (always enabled)\n",
                    index + 1);
            return -1;
        default:
            fprintf(stderr, "SPDIF enable refused\n");
            return -1;
    }
}

// Read the whole SPDIF input inventory in one transfer.
// out_count, out_mask, out_pins[4] must be non-NULL (pins array length 4).
int get_spdif_input_config(libusb_device_handle *handle, uint8_t *out_count,
                           uint8_t *out_mask, uint8_t out_pins[4]) {
    uint8_t resp[6];
    int ret = libusb_control_transfer(handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        INPUT_SPDIF_RX_CONFIG,        // 0x24
        0, WINDEX(PAGE_INPUT),
        resp, sizeof(resp), 1000);

    if (ret != (int)sizeof(resp)) {
        fprintf(stderr, "Failed to get SPDIF config: %s\n", libusb_error_name(ret));
        return -1;
    }
    *out_count  = resp[0];          // 4
    *out_mask   = resp[1];          // bit0=input1 (always), bit1=SPDIF2 .. bit3=SPDIF4
    for (int i = 0; i < 4; i++) out_pins[i] = resp[2 + i];
    return 0;
}
```

### Populate a source dropdown

Combine `INPUT.SPDIF_RX_CONFIG` to learn which S/PDIF inputs are enabled with `INPUT.SOURCE` (GET) to mark the current selection:

```c
void build_source_menu(libusb_device_handle *handle) {
    static const uint8_t spdif_src[4] = { INPUT_SOURCE_SPDIF, INPUT_SOURCE_SPDIF2,
                                          INPUT_SOURCE_SPDIF3, INPUT_SOURCE_SPDIF4 };

    uint8_t count, mask, pins[4], current = INPUT_SOURCE_USB;
    if (get_spdif_input_config(handle, &count, &mask, pins) != 0) return;
    get_input_source(handle, &current);

    printf("%s USB\n", current == INPUT_SOURCE_USB ? "*" : " ");
    for (uint8_t i = 0; i < count; i++) {
        if (!(mask & (1u << i))) continue;   // skip disabled optional inputs
        printf("%s SPDIF %u (GPIO %u)\n",
               current == spdif_src[i] ? "*" : " ", i + 1, pins[i]);
    }
    // (Also add an I2S entry if the device reports I2S support.)
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

For responsive UI feedback, poll `INPUT.SPDIF_RX_STATUS` at **200 ms intervals**. The receiver status is not pushed as a notification, so the host must poll.

- **Minimum interval:** 100 ms. Faster polling provides no additional information because the receiver state machine operates on ~50-100 ms timescales.
- **Maximum interval:** 500 ms. Longer intervals may cause noticeable UI lag when the receiver state changes.
- **Idle optimization:** When `state == INACTIVE` (USB is the active input), you can reduce the polling rate to 1000 ms or stop polling entirely.

The status query is lightweight (reads cached state variables, no hardware interaction) and does not affect audio processing.

---

## 13. Backward Compatibility

### Older firmware

Every firmware that speaks the paged protocol (1.1.7 and later) has all seven commands. A host that gates features on the command table checks the command bits in `SYS.PAGE_COMMANDS` for page 0x03 (`vendor_command_overhaul_plan.md` section 5.14). Firmware 1.1.6 and older does not answer the paged protocol at all, and a host must not send it paged commands.

Within the 1.1.6 opcode space, the base S/PDIF input feature shipped in firmware v1.1.0 and the multiple-selectable-input feature (opcodes 0xE9 and 0xEF, the indexed 0xE4 and 0xE5, S/PDIF 2 and 3) in v1.1.5. A host that still supports 1.1.6 devices reads their version with the 1.1.6 `GET_PLATFORM` (0x7F, page 0) before choosing a protocol.

### Pin commands take an explicit index

`INPUT.SPDIF_RX_PIN` always takes the input index in `wValue` and the GPIO in the payload. The 1.1.6 bare-pin form, in which `wValue` carried only the GPIO and addressed input 1, no longer exists.

### Older presets

Presets saved with `SLOT_DATA_VERSION < 13` carry no `input_source`; loading one leaves the source unchanged (USB on a fresh boot). Presets saved with `SLOT_DATA_VERSION < 24` carry no optional-SPDIF fields; `io_config_from_slot()` gates them on `version >= 24`, so such presets **load with the optional S/PDIF inputs disabled** (baselined from the device-global directory config, defaults 20/21/22). Presets in the V24..V34 range carry the mask and the 2/3 pins but not input 4's pin (gated on `version >= 35`), and their mask has bit 2 clear, so they load with S/PDIF 4 disabled at the device-global pin.

### Wire format

The `WireInputConfig` section is still 16 bytes at `WIRE_FORMAT_VERSION = 28`, but the section is now full: input 4's pin took its last reserved byte and shifted `spdif_rx_enabled_ext_p1`, `i2s_clock_mode` and the ADAT input fields down one, which is what forced the version bump (a V27 host's bulk SET is rejected until rebuilt). Zeros in the pin/enable bytes still decode as "absent" (pins kept live; enable mask kept live, thanks to the +1 encoding of `spdif_rx_enabled_ext_p1`). Hosts that request fewer bytes than the full `WireBulkParams` size receive a truncated response (the firmware respects `wLength`).

All other vendor commands (EQ, matrix mixer, crossfeed, loudness, pin config, presets, etc.) function identically regardless of input source.

---

## Vendor Command Summary

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `INPUT.SOURCE` | 0x03 | 0x10 | GET, SET | none (0) | GET 1, SET 1 | Source (0=USB, 1=SPDIF 1, 2=I2S, 3=ADAT, 4=SPDIF 2, 5=SPDIF 3, 6=SPDIF 4). SET is deferred and non-blocking |
| `INPUT.STATUS` | 0x03 | 0x11 | GET | none (0) | GET 12 | u8 source, u8 active channels, u16 reserved, u32 measured S/PDIF rate mHz, i32 VCXO trim ppm x100 |
| `INPUT.SPDIF_RX_STATUS` | 0x03 | 0x20 | GET | none (0) | GET 16 | `SpdifRxStatusPacket`: receiver state, rate, errors, FIFO fill |
| `INPUT.SPDIF_RX_CH_STATUS` | 0x03 | 0x21 | GET | none (0) | GET 24 | Raw IEC 60958 channel status bits from received stream |
| `INPUT.SPDIF_RX_PIN` | 0x03 | 0x22 | GET, SET | S/PDIF input 0-3 | GET 1, SET 1 | GPIO in the payload, 0xFF restores the default. Hot-swappable |
| `INPUT.SPDIF_RX_ENABLE` | 0x03 | 0x23 | GET, SET | S/PDIF input 1-3 | GET 1, SET 1 | Enable of an optional S/PDIF input |
| `INPUT.SPDIF_RX_CONFIG` | 0x03 | 0x24 | GET | none (0) | GET 6 | Count, enable mask, pins[4] |

A refused SET STALLs. Its result code (`VC_ERR_*`, section 4) is readable from `SYS.LAST_RESULT`. In 1.1.6 the pin and enable SETs were IN transfers that returned a `PIN_CONFIG_*` status byte. Sections 4.5 and 4.7 map each old status to its result code.
