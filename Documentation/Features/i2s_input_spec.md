# I2S Input Specification

**Firmware version:** introduced 2026-06-11 (wire format V12, preset slot V17)
**Audience:** host application developers integrating I2S input support, plus firmware maintainers. Everything an app needs (commands, payloads, sequences, notifications, persistence, edge cases) is in this document; firmware internals are summarized at the end with pointers into the source.

---

## 1. Overview

I2S input adds a third selectable audio input source (`INPUT_SOURCE_I2S = 2`) alongside USB (0) and SPDIF (1). An external I2S source (typically an ADC or another digital audio device) feeds stereo 24-bit audio into the DSP pipeline through a GPIO data pin.

> **Clock mode note (2026-07-06):** this document describes the default
> MASTER clock mode (`REQ_GET_I2S_CLOCK_MODE` = 0). A SLAVE clock mode now
> exists in which an external master drives BCK/LRCLK and the rate is
> auto-detected; see `Documentation/Features/i2s_slave_input_spec.md`.
> Everything below applies unchanged while the mode is MASTER.

The defining property in master mode: **the DSPi is the I2S clock master.** The external source must be configured as an I2S *slave*; it receives BCK and LRCLK from the DSPi (the same shared clock pair the I2S outputs use) and returns data aligned to them. Consequences an app should internalize:

- The input is sample-synchronous with the outputs. There is no clock servo, no drift, and no resampling.
- There is **no lock state machine and no signal detection**. Unlike SPDIF, the device cannot tell whether a source is actually connected; if nothing drives the data pin, the input simply reads silence (or noise on a floating pin).
- The **device picks the sample rate**, not the source. The app selects 44100, 48000 or 96000 Hz via a vendor command (`REQ_SET_INPUT_RATE`); the external slave must follow the clocks it is given.
- Switching to/from I2S never disturbs USB enumeration. As with SPDIF, the USB audio interface stays fully alive and the host never re-enumerates; the switch is purely a DSP routing decision.

### Key characteristics

| Property | Value |
|---|---|
| Input source enum value | 2 (`INPUT_SOURCE_I2S`) |
| Audio format | Stereo, 24-bit in 32-bit frames, MSB first, standard I2S (1-bit delay) |
| Clock direction | DSPi drives BCK + LRCLK (and optionally MCK); source is slave |
| Sample rates | 44100 / 48000 / 96000 Hz, selected by the app |
| Data pin | Any free GPIO, default GPIO 4, runtime configurable |
| Clock pins | Shared `i2s_bck_pin` (default 14) and BCK+1 = LRCLK (default 15) |
| MCK | Optional, shared with I2S output MCK (CLK_GPOUT based) |
| Lock / status concept | None; selected = running |
| Platforms | RP2040 and RP2350, identical app-facing behavior |

---

## 2. Hardware Wiring (read this first)

Connect the external I2S source (slave mode) to the DSPi:

| Signal | Direction | DSPi GPIO (default) | Notes |
|---|---|---|---|
| BCK (bit clock) | DSPi → source | `i2s_bck_pin` (14) | Fs × 64 |
| LRCLK / WS (word clock) | DSPi → source | `i2s_bck_pin` + 1 (15) | = Fs; low = left |
| DATA | source → DSPi | `i2s_rx_pin` (4) | Sampled on BCK rising edge |
| MCK (optional) | DSPi → source | `i2s_mck_pin` (21 on RP2040, 13 on RP2350) | 128x or 256x Fs, only if enabled |

For a concrete Pico 2 wiring example using a PCM1808 ADC module, see [`Images/pcm1808_pico2_i2s_wiring.svg`](../../Images/pcm1808_pico2_i2s_wiring.svg). The PCM1808 should be strapped for slave, standard-I2S mode (`MD1 = LOW`, `MD0 = LOW`, `FMT = LOW`) and should receive `SCKI/MCLK` from DSPi MCK with the multiplier set to **256x**.

Electrical and format requirements:

- 3.3 V logic levels.
- Standard I2S framing: 32 BCK cycles per channel (64 per frame), data MSB first, first data bit one BCK period after each LRCLK transition (the standard I2S 1-bit delay). This matches the DSPi's own I2S outputs, so any DAC-compatible ADC framing works.
- The source must output 24-bit audio left-justified in the 32-bit slot (bits 31..8). The DSPi masks the low 8 bits, so status/zero padding there is ignored.
- If at least one output slot is configured as I2S, the input shares that output's BCK/LRCLK; the external source and the DACs hang off the same clock bus. If **no** output slot is I2S, the input state machine itself drives BCK/LRCLK on the same pins, so wiring does not change with output configuration.
- Sources that need MCK: enable the device MCK (`REQ_SET_MCK_ENABLE`, multiplier via `REQ_SET_MCK_MULTIPLIER`). When I2S input is active the firmware keeps MCK running even with zero I2S outputs.

---

## 3. Vendor Command Reference

All commands are EP0 control transfers on the vendor interface (wIndex = 2), same conventions as every other DSPi command:

- **bmRequestType** `0x41` (Host-to-Device, Vendor, Interface) for OUT commands with a payload.
- **bmRequestType** `0xC1` (Device-to-Host, Vendor, Interface) for IN commands, including the "immediate-response SET" pattern where the parameter rides in wValue and a status byte comes back.

### 3.1 Command summary

| Command | Code | Direction | wValue | wLength | Description |
|---|---|---|---|---|---|
| `REQ_SET_INPUT_SOURCE` | `0xE0` | OUT | 0 | 1 | Select input source; payload byte 2 = I2S |
| `REQ_GET_INPUT_SOURCE` | `0xE1` | IN | 0 | 1 | Returns active source (0/1/2) |
| `REQ_SET_INPUT_RATE` | `0xED` | OUT | 0 | 4 | Select I2S input sample rate (uint32 LE Hz) |
| `REQ_GET_INPUT_RATE` | `0xEE` | IN | 0 | 8 | Returns {current pipeline Hz, selected I2S Hz} |
| `REQ_SET_I2S_RX_PIN` | `0xF1` | IN* | new pin | 1 | Set I2S data pin; returns status byte |
| `REQ_GET_I2S_RX_PIN` | `0xF2` | IN | 0 | 1 | Returns current I2S data pin |
| `REQ_SET_I2S_BCK_PIN` | `0xC2` | IN* | new pin | 1 | Existing command; now also legal while I2S input is active with no I2S outputs |
| `REQ_SET_MCK_ENABLE` etc. | `0xC4`..`0xC9` | | | | Existing MCK commands, unchanged; relevant because the input source may need MCK |

*IN-direction with the value in wValue and a 1-byte status response (same shape as `REQ_SET_SPDIF_RX_PIN` 0xE4 and `REQ_SET_I2S_BCK_PIN` 0xC2).

### 3.2 Status codes (pin commands)

| Code | Name | Meaning |
|---|---|---|
| `0x00` | `PIN_CONFIG_SUCCESS` | Accepted (or no-op, already that pin) |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | Out of range or reserved GPIO |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | GPIO already used by an output, clock, RX pin or DAC mute |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | (BCK only) rejected because an output slot is I2S |

Reserved GPIOs (always rejected): 12 (UART TX), 23..25 (power/LED), above 28 (RP2040) / 29 (RP2350). The in-use check covers: all output data pins, BCK/LRCLK whenever any output slot is I2S **or I2S is the active input**, the MCK pin when MCK is enabled, the SPDIF RX pin, the I2S RX pin, and DAC hardware-mute pins.

### 3.3 REQ_SET_INPUT_SOURCE (0xE0), updated

Payload byte now accepts `2` (I2S). The switch is deferred to the firmware main loop; expect a brief full mute (soft envelope plus DAC hardware mute if configured) around the transition, exactly like USB/SPDIF switches. No response payload; confirm with `REQ_GET_INPUT_SOURCE` or the `input_source` PARAM_CHANGED notification, which is emitted **when the switch actually applies**, not when the request is received.

```
bmRequestType: 0x41   bRequest: 0xE0   wValue: 0   wIndex: 2   wLength: 1
payload: 02                       (INPUT_SOURCE_I2S)
```

Behavior specific to selecting I2S:

- If the selected I2S rate differs from the current pipeline rate, the firmware performs the rate change as part of the switch (one combined reset, not two).
- If MCK is enabled and no output slot is I2S, MCK is started for the external source (divider loaded before enable, so no frequency chirp).
- Input-channel default names regenerate to "I2S L" / "I2S R" (only if the user had not renamed them); expect channel-name PARAM_CHANGED notifications.
- Switching **away** from I2S stops the input hardware and, if no I2S output needs it, stops MCK.

### 3.4 REQ_SET_INPUT_RATE (0xED)

Sets the I2S input sample rate. Always stored (it persists with presets and applies on the next switch to I2S); applied immediately through a deferred rate change when I2S is currently the active source.

```
bmRequestType: 0x41   bRequest: 0xED   wValue: 0   wIndex: 2   wLength: 4
payload: 80 BB 00 00              (48000 little-endian)
```

- Valid values: 44100, 48000, 96000 (decimal Hz, uint32 little-endian). Anything else is silently ignored.
- When applied live, the device runs a full pipeline reset (brief mute) and retunes BCK/LRCLK/MCK; the external slave simply follows the new clocks.
- A PARAM_CHANGED notification fires for the `i2s_input_rate` wire byte (see section 5).
- There is no SET response; read back with `REQ_GET_INPUT_RATE`.

### 3.5 REQ_GET_INPUT_RATE (0xEE)

```
bmRequestType: 0xC1   bRequest: 0xEE   wValue: 0   wIndex: 2   wLength: 8
response: uint32 LE current_pipeline_hz, uint32 LE selected_i2s_hz
```

`current_pipeline_hz` is the live processing rate regardless of source (useful generally, valid for USB and SPDIF too). `selected_i2s_hz` is the stored I2S selection. While I2S input is active and settled the two are equal; immediately after a SET they may briefly differ until the deferred rate change lands.

### 3.6 REQ_SET_I2S_RX_PIN (0xF1)

```
bmRequestType: 0xC1   bRequest: 0xF1   wValue: <new_pin>   wIndex: 2   wLength: 1
response: 1 status byte (section 3.2)
```

- RAM-only update; persistence is explicit (section 6).
- Hot-swap supported: if I2S input is active, the firmware restarts the input on the new pin via a deferred handler (brief mute). Nothing else changes.
- Emits a PARAM_CHANGED for the `i2s_rx_pin` wire byte on success.

### 3.7 REQ_GET_I2S_RX_PIN (0xF2)

Returns 1 byte: the current data pin.

### 3.8 REQ_SET_I2S_BCK_PIN (0xC2), behavior change

Previously rejected with `PIN_CONFIG_OUTPUT_ACTIVE` whenever any output slot was I2S, and that rejection still stands. New: when I2S **input** is active with no I2S outputs (the input state machine is the clock master), the command is accepted and the input restarts on the new BCK/LRCLK pair via the same deferred handler. Both `pin` and `pin + 1` must be valid and free.

### 3.9 Things that do NOT exist (on purpose)

- No I2S RX status/lock command. `REQ_GET_INPUT_SOURCE == 2` means the input is running; there is nothing else to know. `REQ_GET_SPDIF_RX_STATUS` (0xE2) reports SPDIF state only and will show INACTIVE while I2S is selected.
- No input-format configuration. Frame format is fixed (24-in-32, standard I2S); rate is the only variable.

---

## 4. App Integration Patterns

### 4.1 Recommended switch-to-I2S sequence

```
1. (optional) REQ_SET_I2S_RX_PIN     if the board wiring differs from GPIO 4
2. (optional) REQ_SET_I2S_BCK_PIN    if BCK/LRCLK differ from 14/15 (only legal
                                     when no I2S output is active; otherwise the
                                     shared output clock pins are already fixed)
3. (optional) MCK setup              REQ_SET_MCK_PIN / REQ_SET_MCK_MULTIPLIER /
                                     REQ_SET_MCK_ENABLE if the source needs MCK
4. REQ_SET_INPUT_RATE                pick 44100/48000/96000
5. REQ_SET_INPUT_SOURCE = 2
6. Wait for the input_source PARAM_CHANGED notification (or poll 0xE1)
7. (optional) REQ_PRESET_SAVE        to persist (section 6)
```

Steps 1..4 are all safe while on another input; they only store configuration. Doing them first means the switch lands in one clean transition.

### 4.2 Live changes while I2S is active

All of these are legal while I2S input is running; each costs one brief mute cycle:

- `REQ_SET_INPUT_RATE`: full rate change (filters recalculated, clocks retuned).
- `REQ_SET_I2S_RX_PIN`: input restarts on the new data pin.
- `REQ_SET_I2S_BCK_PIN`: input restarts on the new clock pins (input-master mode only).
- Output type changes (`REQ_SET_OUTPUT_TYPE`): the firmware automatically re-elects the input's clock role. Switching the first output slot to I2S demotes the input to slave on the output's clocks; switching the last I2S output away promotes the input to clock master. The app does not manage this; clocks on the wire stay on the same GPIOs throughout.

### 4.3 UI guidance

- Expose the rate selector only for I2S input (USB rate belongs to the host OS; SPDIF rate belongs to the source). `REQ_GET_INPUT_RATE` byte 0..3 is still a fine generic "current rate" display for all sources.
- Do not look for a lock indicator; there is none. If you want a "signal present" hint, use the input channel peak meters (`REQ_GET_STATUS`, peaks[0..1]): a connected, playing source shows nonzero peaks.
- Input channel labels: after a source switch the default names become "I2S L" / "I2S R" (channel-name notifications fire). User-renamed channels are left alone.
- Volume model matches SPDIF: the host (Windows) volume slider and mute do not affect I2S playback; use `REQ_SET_USER_VOLUME` / `REQ_SET_USER_MUTE` and the master volume commands, which always apply.

### 4.4 Notifications to handle

Via the standard EP 0x83 notification protocol (see `notification_protocol_v2_spec.md`), the I2S input feature emits PARAM_CHANGED events for these `WireBulkParams` byte offsets (V12 layout; `WireInputConfig` starts at offset 2896):

| Field | Offset | Size | Emitted when |
|---|---|---|---|
| `input_config.input_source` | 2896 | 1 | A source switch applies (any direction) |
| `input_config.i2s_rx_pin` | 2898 | 1 | 0xF1 succeeds, bulk apply changes it, preset load changes it |
| `input_config.i2s_input_rate` | 2899 | 1 | 0xED accepts a rate (wire encoding 0/1/2, not Hz) |
| `i2s_config.bck_pin` | (see i2s_output spec) | 1 | 0xC2 succeeds |
| `channel_names.names[0..1]` | per name slot | 32 | Default input names regenerate on source switch |

PRESET_LOADED / BULK_INVALIDATED events follow the usual rule: re-read everything with `REQ_GET_ALL_PARAMS`.

### 4.5 Example code (libusb, C)

```c
#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <string.h>

#define REQ_SET_INPUT_SOURCE  0xE0
#define REQ_GET_INPUT_SOURCE  0xE1
#define REQ_SET_INPUT_RATE    0xED
#define REQ_GET_INPUT_RATE    0xEE
#define REQ_SET_I2S_RX_PIN    0xF1
#define REQ_GET_I2S_RX_PIN    0xF2
#define VENDOR_INTF           2
#define INPUT_SOURCE_I2S      2

#define OUT_REQ (LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE)
#define IN_REQ  (LIBUSB_ENDPOINT_IN  | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE)

// Select the I2S input rate (44100 / 48000 / 96000)
int dspi_set_input_rate(libusb_device_handle *h, uint32_t hz) {
    uint8_t payload[4];
    memcpy(payload, &hz, 4);  // little-endian host assumed; serialize explicitly if not
    return libusb_control_transfer(h, OUT_REQ, REQ_SET_INPUT_RATE,
                                   0, VENDOR_INTF, payload, 4, 1000);
}

// Read {current pipeline Hz, selected I2S Hz}
int dspi_get_input_rate(libusb_device_handle *h, uint32_t *current, uint32_t *selected) {
    uint8_t buf[8];
    int r = libusb_control_transfer(h, IN_REQ, REQ_GET_INPUT_RATE,
                                    0, VENDOR_INTF, buf, 8, 1000);
    if (r != 8) return -1;
    memcpy(current,  buf,     4);
    memcpy(selected, buf + 4, 4);
    return 0;
}

// Set the I2S RX data pin; returns PIN_CONFIG_* status or negative on USB error
int dspi_set_i2s_rx_pin(libusb_device_handle *h, uint8_t pin) {
    uint8_t status;
    int r = libusb_control_transfer(h, IN_REQ, REQ_SET_I2S_RX_PIN,
                                    pin, VENDOR_INTF, &status, 1, 1000);
    return (r == 1) ? status : -1;
}

// Full switch-to-I2S sequence
int dspi_select_i2s_input(libusb_device_handle *h, uint32_t hz) {
    if (dspi_set_input_rate(h, hz) < 0) return -1;
    uint8_t src = INPUT_SOURCE_I2S;
    if (libusb_control_transfer(h, OUT_REQ, REQ_SET_INPUT_SOURCE,
                                0, VENDOR_INTF, &src, 1, 1000) != 1) return -1;
    // Poll until the deferred switch applies (or watch EP 0x83 notifications)
    for (int i = 0; i < 50; i++) {
        uint8_t cur;
        if (libusb_control_transfer(h, IN_REQ, REQ_GET_INPUT_SOURCE,
                                    0, VENDOR_INTF, &cur, 1, 1000) == 1
            && cur == INPUT_SOURCE_I2S) return 0;
        // ~10 ms between polls; the switch typically lands within one or two
        struct timespec ts = {0, 10 * 1000 * 1000}; nanosleep(&ts, NULL);
    }
    return -2;
}
```

---

## 5. Wire Format (Bulk Parameters)

`WIRE_FORMAT_VERSION` is now **12**. V12 claims two previously-reserved bytes inside `WireInputConfig`; the total payload size is byte-identical to V11 (3664 bytes), so transport code needs no changes.

### 5.1 WireInputConfig (16 bytes, offset 2896 in WireBulkParams)

| Byte | Field | Description |
|---|---|---|
| 0 | `input_source` | 0 = USB, 1 = SPDIF, 2 = I2S |
| 1 | `spdif_rx_pin` | SPDIF RX GPIO |
| 2 | `i2s_rx_pin` | I2S RX data GPIO (V12+) |
| 3 | `i2s_input_rate` | Rate enum: 0 = 44100, 1 = 48000, 2 = 96000 (V12+) |
| 4..15 | reserved | Zero |

### 5.2 Apply semantics (REQ_SET_ALL_PARAMS, 0xA1)

- `input_source` (V7+): a differing value queues the standard deferred source switch.
- `i2s_rx_pin` (V12+): applied on the same gate as output pins (only when the device's output-config mode is "with preset"); value 0 or an invalid/reserved GPIO is ignored. If it changes while I2S input is active, the hot-swap restart fires.
- `i2s_input_rate` (V12+): always stored. If the bulk apply happens while I2S input is active and the rate differs from the live pipeline rate, the firmware defers a rate change after restarting the input.
- Payloads with `format_version < 12` leave both new fields untouched (their bytes were reserved/zero in older layouts).

### 5.3 Collect semantics (REQ_GET_ALL_PARAMS, 0xA0)

All four bytes reflect live state. `i2s_input_rate` uses the enum encoding, not Hz.

---

## 6. Preset Persistence

### 6.1 What is stored where

| Item | PresetSlot (V17) | Device-global block (`FlashOutputConfig`) | Applied by |
|---|---|---|---|
| `input_source` | yes (V13+) | no | preset load (deferred switch) |
| `i2s_rx_pin` | yes (V17+, 0 = unset) | yes (claimed reserved byte, 0 = unset) | output-config mode rules |
| `i2s_input_rate` | yes (V17+, enum) | yes (stored +1, 0 = unset) | output-config mode rules |

The I2S pin and rate follow the existing **output-config mode** (`REQ_SET_OUTPUT_CONFIG_MODE`, 0x98):

- **With-preset mode:** `REQ_PRESET_SAVE` captures the live pin/rate into the slot; preset load restores them.
- **Independent mode:** presets do not touch them; `REQ_SAVE_OUTPUT_CONFIG` (0x52) persists the live values device-globally and boot restores them.

Vendor SETs (0xED, 0xF1) are always RAM-live only; nothing hits flash until the user saves a preset or the output config. Apps should surface this the same way they do for output pins.

### 6.2 Flash format details

- `SLOT_DATA_VERSION` is now **17**: `i2s_rx_pin` and `i2s_input_rate` are appended after the V16 crossover block (struct grows 2 bytes). Older slots remain CRC-valid via the per-version size table; V16-and-older slots simply leave the live pin/rate alone on load.
- The device-global `FlashOutputConfig` stores the rate as **encoding + 1** so that directories written by older firmware (zeros in reserved bytes) read as "unset" instead of 44.1 kHz. Slot storage uses the plain enum because reads are gated on `version >= 17`.
- Factory reset returns the I2S pin/rate to defaults (GPIO 4, 48 kHz) per the normal factory-defaults flow, and queues the standard deferred switch to USB, which stops the input and applies the retained USB rate.

### 6.3 Loading a preset that selects I2S

Preset load applies DSP state, restarts the input with a freshly elected clock role, and, if the preset's stored rate differs from the live rate, performs a deferred rate change. If the preset also changes the input source, the standard deferred source-switch handler manages hardware start/stop. Boot into an I2S preset behaves identically (the source switch is deferred into the first main-loop iterations).

---

## 7. Behavior Details and Timing

### 7.1 Switch timing

A source switch to I2S is deferred to the firmware main loop and typically completes within a few milliseconds (one `pipeline_reset_ready()` gate plus one pipeline reset; plus one rate change if the rate differs). During the window: outputs soft-mute, the DAC hardware mute asserts if configured, and the pipeline restarts all output slots sample-aligned. Audio resumes through a short fade-in. Nothing about USB changes.

### 7.2 Clock behavior on the wire

- I2S input active, no I2S outputs: BCK/LRCLK are driven by the input state machine. They stop briefly during restarts (rate change, pin swap, preset load) and resume; a slave ADC just follows.
- I2S input active, at least one I2S output: BCK/LRCLK come from the output clock master, exactly as before this feature; the input listens on the same pads. Output reconfigurations briefly stop/restart the clocks as they always have.
- MCK (when enabled) runs whenever any I2S output exists **or** I2S input is active. It is glitchless across rate changes (divider hot-reload).

### 7.3 What the device cannot detect

No source-present detection, no rate verification (the source has no say in the rate), no error counters. A misconfigured source (wrong format, master instead of slave) produces garbled audio or silence with no firmware-side indication beyond the channel meters. Document this in app help text: "the external device must be configured as an I2S slave, 24-bit, 32-bit frame".

### 7.4 L/R integrity guarantee

The firmware guarantees the left/right word framing of the input can never silently swap: every input (re)start and every synchronized output restart re-phases the input against LRCLK, and the capture ring preserves word parity even through overruns. Apps never need to offer an "L/R swap" workaround toggle.

---

## 8. Edge Cases

| Case | Behavior |
|---|---|
| SET input source 2 while already I2S | Consumed as no-op, no mute, no notification |
| SET rate while on USB/SPDIF | Stored only; applies on next switch to I2S |
| SET invalid rate (e.g. 88200) | Silently ignored (no error response exists on OUT commands) |
| SET I2S RX pin to current pin | `PIN_CONFIG_SUCCESS`, no restart |
| SET I2S RX pin to a pin in use | `PIN_CONFIG_PIN_IN_USE`, state unchanged |
| SET BCK pin while an output slot is I2S | `PIN_CONFIG_OUTPUT_ACTIVE` (pre-existing rule) |
| SET BCK pin while I2S input active, no I2S outputs | Accepted; input restarts on new pins |
| Output type switched while I2S input active | Input suspends and restarts with re-elected clock role automatically |
| Preset save/load, factory reset, bulk apply while I2S active | Input suspended across the flash blackout / state swap and restarted after, same as SPDIF RX |
| Source pin floating (nothing connected) | Undefined samples (often noise); use peak meters to detect |
| Output pins changed (0x7C) while I2S input active | No input restart needed; firmware re-phases automatically |
| Windows volume slider / mute while on I2S | No audible effect (matches SPDIF policy); use user/master volume commands |

---

## 9. Firmware Internals (summary for maintainers)

App developers can stop reading here. Full detail lives in `Documentation/current_architecture.md` ("Audio Input Source System" section); this is the orientation map.

- **Files:** `firmware/DSPi/i2s_input.c/.h` (lifecycle, DMA ring, poll), `firmware/DSPi/i2s_input.pio` (two RX programs), election helper and all orchestration in `main.c`, commands in `vendor_commands.c`.
- **Resources:** reuses the SPDIF RX PIO state machine (PIO1 SM2 on RP2040, PIO2 SM0 on RP2350) and the two SPDIF RX DMA channels; legal because inputs are switched, never mixed. Claimed/unclaimed on start/stop.
- **Two PIO programs:** `audio_i2s_rx_clkmaster` (12 instructions, side-set drives BCK/LRCLK, divider = sys_clk * 2 / Fs, identical to the TX master) and `audio_i2s_rx_slave` (7 instructions, divider 1.0, wait-driven on the BCK/LRCLK pads; the `wait gpio` instructions are patched with the runtime pin numbers at load time). Both autopush 32-bit words, left word first.
- **Clock-role election:** input is master only when zero output slots are I2S (`i2s_input_should_be_master()`); `process_type_switches()` is the re-election point. Cross-PIO-block sync is never needed because input-as-master implies no I2S outputs.
- **Capture path:** IRQ-less two-channel DMA ring (data channel with write-address wrap chained to a reload channel that re-triggers it). `i2s_input_poll()` in the main loop converts and feeds `process_input_block()`, capped at 192 frames per call. 4 KB ring on RP2040, 8 KB on RP2350.
- **Slave resync invariant:** `complete_pipeline_reset()` and `enable_outputs_in_sync()` end with `i2s_input_resync()`, because restarting the I2S TX clock master resets LRCLK phase under a running slave input. Any new code path that restarts the TX master must go through those functions.
- **Rate handling:** `i2s_input_rate` global; applied via the standard `pending_rate` / `rate_change_pending` deferral. `perform_rate_change()` now also updates `audio_state.freq` (fixing a pre-existing stale-rate defect on SPDIF-driven changes) and brackets the input with stop/restart.
- **Suspension sites** mirror SPDIF RX: rate change, type switches, flash-write brackets (`resume_i2s_after_flash()`), preset load, factory reset, bulk apply, input switch, plus the deferred hot-swap handler for data-pin/BCK changes.

### Hardware test matrix

| Test | Expectation |
|---|---|
| Input master, zero I2S outputs, scope BCK/LRCLK | BCK = Fs x 64, LRCLK = Fs at 44.1/48/96 kHz; clean restart on rate change |
| Input slave with one I2S output | Correct L/R framing; trigger an output pin change and a preset save and verify framing survives (resync invariant) |
| USB -> I2S -> USB and SPDIF -> I2S switches | Clean mutes, no clicks, correct rate after each transition |
| Flash write during I2S playback (preset save) | Brief mute, input resumes, no L/R swap |
| 0xED rate change in both roles | One mute cycle, BCK retunes, audio resumes at new rate |
| Preset roundtrip | Pin + rate + source restored per output-config mode |
| V16 slot on V17 firmware | Loads cleanly, I2S fields fall back to live defaults |
| MCK with input master | MCK present and frequency-correct with zero I2S outputs |
