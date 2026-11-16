# I2S Input Specification

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7)*

**Firmware version:** introduced 2026-06-11 (wire format V12, preset slot V17)
**Audience:** host application developers integrating I2S input support, plus firmware maintainers. Everything an app needs (commands, payloads, sequences, notifications, persistence, edge cases) is in this document; firmware internals are summarized at the end with pointers into the source.

---

## 1. Overview

I2S input adds a third selectable audio input source (`INPUT_SOURCE_I2S = 2`) alongside USB (0) and SPDIF (1). An external I2S source (typically an ADC or another digital audio device) feeds stereo 24-bit audio into the DSP pipeline through a GPIO data pin.

> **Clock mode note (2026-07-06):** this document describes the default
> MASTER clock mode (`CLOCK.I2S_CLOCK_MODE` = 0). A SLAVE clock mode now
> exists in which an external master drives BCK/LRCLK and the rate is
> auto-detected; see `Documentation/Features/i2s_slave_input_spec.md`.
> Everything below applies unchanged while the mode is MASTER.

The defining property in master mode: **the DSPi is the I2S clock master.** The external source must be configured as an I2S *slave*; it receives BCK and LRCLK from the DSPi (the same shared clock pair the I2S outputs use) and returns data aligned to them. Consequences an app should internalize:

- The input is sample-synchronous with the outputs. There is no clock servo, no drift, and no resampling.
- There is **no lock state machine and no signal detection**. Unlike SPDIF, the device cannot tell whether a source is actually connected; if nothing drives the data pin, the input simply reads silence (or noise on a floating pin).
- The **device picks the sample rate**, not the source. The app selects 44100, 48000 or 96000 Hz via a vendor command (`INPUT.I2S_RX_RATE`); the external slave must follow the clocks it is given.
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
- Sources that need MCK: enable the device MCK (`CLOCK.MCK_ENABLE`, multiplier via `CLOCK.MCK_MULTIPLIER`). When I2S input is active the firmware keeps MCK running even with zero I2S outputs.

---

## 3. Vendor Command Reference

All commands are EP0 control transfers on the vendor interface using the paged protocol of firmware 1.1.7 (see `../commands.md`):

- **bmRequestType** `0x41` (Host-to-Device, Vendor, Interface) for SETs, `0xC1` (Device-to-Host, Vendor, Interface) for GETs.
- **bRequest** is the command number and **wIndex** is `(page << 8) | 2`.
- **wValue** is the address (the stereo pair for `INPUT.I2S_RX_PIN`, the role for `CLOCK.I2S_BCK_PIN`), otherwise 0.
- Values travel in the payload. There is no status byte: a SET is ACKed, or it STALLs and the `VC_*` reason is readable from `SYS.LAST_RESULT`.

### 3.1 Command summary

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Description |
|---|---|---|---|---|---|---|
| `INPUT.SOURCE` | 0x03 | 0x10 | GET, SET | none (0) | GET 1, SET 1 | Active input source; SET payload 2 selects I2S |
| `INPUT.I2S_RX_RATE` | 0x03 | 0x32 | GET, SET | none (0) | GET 4, SET 4 | Selected I2S input sample rate (uint32 LE Hz) |
| `SYS.SYSTEM_STATUS` | 0x01 | 0x40 | GET | none (0) | GET 16 | Current pipeline rate is the u32 at offset 4 |
| `INPUT.I2S_RX_PIN` | 0x03 | 0x30 | GET, SET | stereo pair (0 for stereo input) | GET 1, SET 1 | I2S data pin of the pair; 0xFF restores its default |
| `CLOCK.I2S_BCK_PIN` | 0x0C | 0x10 | GET, SET | role 0-1 | GET 1, SET 1 | BCK pin (LRCLK = BCK + 1); also legal while I2S input is active with no I2S outputs |

The MCK commands (`CLOCK.MCK_ENABLE`, `CLOCK.MCK_PIN`, `CLOCK.MCK_MULTIPLIER`, see `master_clock_spec.md`) are relevant because the input source may need MCK.

### 3.2 Result codes (pin commands)

| Code | Name | Meaning |
|---|---|---|
| `0x00` | `VC_OK` | Accepted (or no-op, already that pin) |
| `0x19` | `VC_ERR_PIN_INVALID` | Out of range or reserved GPIO |
| `0x1A` | `VC_ERR_PIN_IN_USE` | GPIO already used by an output, clock, RX pin or DAC mute |
| `0x18` | `VC_ERR_STATE` | (BCK only) rejected because an output slot is I2S |
| `0x13` | `VC_ERR_ADDRESS` | Pair or role out of range |

Reserved GPIOs (always rejected): 23..25 (power/LED), above 28 (RP2040) / 29 (RP2350). GPIO 12 is an ordinary pin (it was reserved for the debug UART TX until that UART was removed; updated 2026-11-15). The in-use check covers: all output data pins, BCK/LRCLK whenever any output slot is I2S **or I2S is the active input**, the MCK pin when MCK is enabled, the SPDIF RX pin, the I2S RX pin, and DAC hardware-mute pins.

### 3.3 `INPUT.SOURCE` SET

Payload byte `2` selects I2S. The switch is deferred to the firmware main loop; expect a brief full mute (soft envelope plus DAC hardware mute if configured) around the transition, exactly like USB/SPDIF switches. The ACK only means the request passed validation; confirm with an `INPUT.SOURCE` GET or the `input_source` PARAM_CHANGED notification, which is emitted **when the switch actually applies**, not when the request is received.

```
bmRequestType: 0x41   bRequest: 0x10   wValue: 0   wIndex: 0x0302   wLength: 1
payload: 02                       (INPUT_SOURCE_I2S)
```

Behavior specific to selecting I2S:

- If the selected I2S rate differs from the current pipeline rate, the firmware performs the rate change as part of the switch (one combined reset, not two).
- If MCK is enabled and no output slot is I2S, MCK is started for the external source (divider loaded before enable, so no frequency chirp).
- Input-channel default names regenerate to "I2S L" / "I2S R" (only if the user had not renamed them); expect channel-name PARAM_CHANGED notifications.
- Switching **away** from I2S stops the input hardware and, if no I2S output needs it, stops MCK.

### 3.4 `INPUT.I2S_RX_RATE` SET

Sets the I2S input sample rate. Always stored (it persists with presets and applies on the next switch to I2S); applied immediately through a deferred rate change when I2S is currently the active source.

```
bmRequestType: 0x41   bRequest: 0x32   wValue: 0   wIndex: 0x0302   wLength: 4
payload: 80 BB 00 00              (48000 little-endian)
```

- Valid values: 44100, 48000, 96000 (decimal Hz, uint32 little-endian). Anything else STALLs with `VC_ERR_VALUE` and changes nothing.
- When applied live, the device runs a full pipeline reset (brief mute) and retunes BCK/LRCLK/MCK; the external slave simply follows the new clocks.
- A PARAM_CHANGED notification fires for the `i2s_input_rate` wire byte (see section 5).
- Read back with an `INPUT.I2S_RX_RATE` GET.

### 3.5 Reading the rates

```
bmRequestType: 0xC1   bRequest: 0x32   wValue: 0   wIndex: 0x0302   wLength: 4
response: uint32 LE selected_i2s_hz                     (INPUT.I2S_RX_RATE)

bmRequestType: 0xC1   bRequest: 0x40   wValue: 0   wIndex: 0x0102   wLength: 16
response: offset 4 is uint32 LE current_pipeline_hz     (SYS.SYSTEM_STATUS)
```

`current_pipeline_hz` is the live processing rate regardless of source (useful generally, valid for USB and SPDIF too). `selected_i2s_hz` is the stored I2S selection. The 1.1.6 command returned both in one 8-byte reply; they are now two reads. While I2S input is active and settled the two are equal; immediately after a SET they may briefly differ until the deferred rate change lands.

### 3.6 `INPUT.I2S_RX_PIN` SET

```
bmRequestType: 0x41   bRequest: 0x30   wValue: 0 (stereo pair 0)   wIndex: 0x0302   wLength: 1
payload: <new_pin>                (0xFF restores the default)
outcome: ACK, or STALL with a result code (section 3.2)
```

- RAM-only update; persistence is explicit (section 6).
- Hot-swap supported: if I2S input is active, the firmware restarts the input on the new pin via a deferred handler (brief mute). Nothing else changes.
- Emits a PARAM_CHANGED for the `i2s_rx_pin` wire byte on success.

### 3.7 `INPUT.I2S_RX_PIN` GET

Returns 1 byte: the current data pin of the pair in `wValue`.

### 3.8 `CLOCK.I2S_BCK_PIN` SET, behavior change

Previously rejected whenever any output slot was I2S, and that rejection (`VC_ERR_STATE`) still stands. New: when I2S **input** is active with no I2S outputs (the input state machine is the clock master), the command is accepted and the input restarts on the new BCK/LRCLK pair via the same deferred handler. Both `pin` and `pin + 1` must be valid and free.

### 3.9 Things that do NOT exist (on purpose)

- No I2S RX status/lock command (in master clock mode). `INPUT.SOURCE` reading 2 means the input is running; there is nothing else to know. `INPUT.SPDIF_RX_STATUS` reports SPDIF state only and will show INACTIVE while I2S is selected.
- No input-format configuration. Frame format is fixed (24-in-32, standard I2S); rate is the only variable.

---

## 4. App Integration Patterns

### 4.1 Recommended switch-to-I2S sequence

```
1. (optional) INPUT.I2S_RX_PIN SET   if the board wiring differs from GPIO 4
2. (optional) CLOCK.I2S_BCK_PIN SET  if BCK/LRCLK differ from 14/15 (only legal
                                     when no I2S output is active; otherwise the
                                     shared output clock pins are already fixed)
3. (optional) MCK setup              CLOCK.MCK_PIN / CLOCK.MCK_MULTIPLIER /
                                     CLOCK.MCK_ENABLE if the source needs MCK
4. INPUT.I2S_RX_RATE SET             pick 44100/48000/96000
5. INPUT.SOURCE SET = 2
6. Wait for the input_source PARAM_CHANGED notification (or poll INPUT.SOURCE)
7. (optional) STORE.PRESET_SAVE      to persist (section 6)
```

Steps 1..4 are all safe while on another input; they only store configuration. Doing them first means the switch lands in one clean transition.

### 4.2 Live changes while I2S is active

All of these are legal while I2S input is running; each costs one brief mute cycle:

- `INPUT.I2S_RX_RATE`: full rate change (filters recalculated, clocks retuned).
- `INPUT.I2S_RX_PIN`: input restarts on the new data pin.
- `CLOCK.I2S_BCK_PIN`: input restarts on the new clock pins (input-master mode only).
- Output type changes (`OUTPUT.TYPE`): the firmware automatically re-elects the input's clock role. Switching the first output slot to I2S demotes the input to slave on the output's clocks; switching the last I2S output away promotes the input to clock master. The app does not manage this; clocks on the wire stay on the same GPIOs throughout.

### 4.3 UI guidance

- Expose the rate selector only for I2S input (USB rate belongs to the host OS; SPDIF rate belongs to the source). The sample rate in `SYS.SYSTEM_STATUS` (offset 4) is a fine generic "current rate" display for all sources.
- Do not look for a lock indicator; there is none. If you want a "signal present" hint, use the input channel peak meters (input channels 0 and 1 in `MEAS.LEVELS`): a connected, playing source shows nonzero peaks.
- Input channel labels: after a source switch the default names become "I2S L" / "I2S R" (channel-name notifications fire). User-renamed channels are left alone.
- Volume model matches SPDIF: the host (Windows) volume slider and mute do not affect I2S playback; use `ROUTE.USER_VOLUME` / `ROUTE.USER_MUTE` and the master volume commands, which always apply.

### 4.4 Notifications to handle

Via the standard EP 0x83 notification protocol (see `notification_protocol_v2_spec.md`), the I2S input feature emits PARAM_CHANGED events for these `WireBulkParams` byte offsets (V12 layout; `WireInputConfig` starts at offset 2896):

| Field | Offset | Size | Emitted when |
|---|---|---|---|
| `input_config.input_source` | 2896 | 1 | A source switch applies (any direction) |
| `input_config.i2s_rx_pin` | 2898 | 1 | `INPUT.I2S_RX_PIN` SET succeeds, bulk apply changes it, preset load changes it |
| `input_config.i2s_input_rate` | 2899 | 1 | `INPUT.I2S_RX_RATE` accepts a rate (wire encoding 0/1/2, not Hz) |
| `i2s_config.bck_pin` | (see i2s_output spec) | 1 | `CLOCK.I2S_BCK_PIN` SET succeeds |
| `channel_names.names[0..1]` | per name slot | 32 | Default input names regenerate on source switch |

PRESET_LOADED / BULK_INVALIDATED events follow the usual rule: re-read everything with `STORE.ALL_PARAMS`.

### 4.5 Example code (libusb, C)

```c
#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define VENDOR_INTF           2
#define PAGE_SYS              0x01
#define PAGE_INPUT            0x03
#define CMD_SYSTEM_STATUS     0x40   // SYS.SYSTEM_STATUS
#define CMD_SOURCE            0x10   // INPUT.SOURCE
#define CMD_I2S_RX_PIN        0x30   // INPUT.I2S_RX_PIN
#define CMD_I2S_RX_RATE       0x32   // INPUT.I2S_RX_RATE
#define INPUT_SOURCE_I2S      2
#define WINDEX(page)          (uint16_t)(((page) << 8) | VENDOR_INTF)

#define OUT_REQ (LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE)
#define IN_REQ  (LIBUSB_ENDPOINT_IN  | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE)

// Select the I2S input rate (44100 / 48000 / 96000). LIBUSB_ERROR_PIPE means refused.
int dspi_set_input_rate(libusb_device_handle *h, uint32_t hz) {
    uint8_t payload[4];
    memcpy(payload, &hz, 4);  // little-endian host assumed; serialize explicitly if not
    return libusb_control_transfer(h, OUT_REQ, CMD_I2S_RX_RATE,
                                   0, WINDEX(PAGE_INPUT), payload, 4, 1000);
}

// Read the current pipeline rate and the selected I2S rate
int dspi_get_input_rate(libusb_device_handle *h, uint32_t *current, uint32_t *selected) {
    uint8_t st[16], sel[4];
    if (libusb_control_transfer(h, IN_REQ, CMD_SYSTEM_STATUS,
                                0, WINDEX(PAGE_SYS), st, 16, 1000) != 16) return -1;
    if (libusb_control_transfer(h, IN_REQ, CMD_I2S_RX_RATE,
                                0, WINDEX(PAGE_INPUT), sel, 4, 1000) != 4) return -1;
    memcpy(current,  st + 4, 4);
    memcpy(selected, sel,    4);
    return 0;
}

// Set the data pin of stereo pair 0. Returns 1 on ACK, LIBUSB_ERROR_PIPE when refused
// (read SYS.LAST_RESULT for the reason), other negatives on USB error.
int dspi_set_i2s_rx_pin(libusb_device_handle *h, uint8_t pin) {
    return libusb_control_transfer(h, OUT_REQ, CMD_I2S_RX_PIN,
                                   0 /* pair */, WINDEX(PAGE_INPUT), &pin, 1, 1000);
}

// Full switch-to-I2S sequence
int dspi_select_i2s_input(libusb_device_handle *h, uint32_t hz) {
    if (dspi_set_input_rate(h, hz) < 0) return -1;
    uint8_t src = INPUT_SOURCE_I2S;
    if (libusb_control_transfer(h, OUT_REQ, CMD_SOURCE,
                                0, WINDEX(PAGE_INPUT), &src, 1, 1000) != 1) return -1;
    // Poll until the deferred switch applies (or watch EP 0x83 notifications)
    for (int i = 0; i < 50; i++) {
        uint8_t cur;
        if (libusb_control_transfer(h, IN_REQ, CMD_SOURCE,
                                    0, WINDEX(PAGE_INPUT), &cur, 1, 1000) == 1
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

### 5.2 Apply semantics (`STORE.ALL_PARAMS` SET)

- `input_source` (V7+): a differing value queues the standard deferred source switch.
- `i2s_rx_pin` (V12+): applied on the same gate as output pins (only when the device's output-config mode is "with preset"); value 0 or an invalid/reserved GPIO is ignored. If it changes while I2S input is active, the hot-swap restart fires.
- `i2s_input_rate` (V12+): always stored. If the bulk apply happens while I2S input is active and the rate differs from the live pipeline rate, the firmware defers a rate change after restarting the input.
- Payloads with `format_version < 12` leave both new fields untouched (their bytes were reserved/zero in older layouts).

### 5.3 Collect semantics (`STORE.ALL_PARAMS` GET)

All four bytes reflect live state. `i2s_input_rate` uses the enum encoding, not Hz.

---

## 6. Preset Persistence

### 6.1 What is stored where

| Item | PresetSlot (V17) | Device-global block (`FlashOutputConfig`) | Applied by |
|---|---|---|---|
| `input_source` | yes (V13+) | no | preset load (deferred switch) |
| `i2s_rx_pin` | yes (V17+, 0 = unset) | yes (claimed reserved byte, 0 = unset) | output-config mode rules |
| `i2s_input_rate` | yes (V17+, enum) | yes (stored +1, 0 = unset) | output-config mode rules |

The I2S pin and rate follow the existing **output-config mode** (`STORE.OUTPUT_CONFIG_MODE`):

- **With-preset mode:** `STORE.PRESET_SAVE` captures the live pin/rate into the slot; preset load restores them.
- **Independent mode:** presets do not touch them; `STORE.OUTPUT_CONFIG_SAVE` persists the live values device-globally and boot restores them.

Vendor SETs (`INPUT.I2S_RX_RATE`, `INPUT.I2S_RX_PIN`) are always RAM-live only; nothing hits flash until the user saves a preset or the output config. Apps should surface this the same way they do for output pins.

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
| SET invalid rate (e.g. 88200) | STALL with `VC_ERR_VALUE`, nothing stored |
| SET I2S RX pin to current pin | ACK, no restart |
| SET I2S RX pin to a pin in use | STALL with `VC_ERR_PIN_IN_USE`, state unchanged |
| SET BCK pin while an output slot is I2S | STALL with `VC_ERR_STATE` (pre-existing rule) |
| SET BCK pin while I2S input active, no I2S outputs | Accepted; input restarts on new pins |
| Output type switched while I2S input active | Input suspends and restarts with re-elected clock role automatically |
| Preset save/load, factory reset, bulk apply while I2S active | Input suspended across the flash blackout / state swap and restarted after, same as SPDIF RX |
| Source pin floating (nothing connected) | Undefined samples (often noise); use peak meters to detect |
| Output pins changed (`OUTPUT.PIN`) while I2S input active | No input restart needed; firmware re-phases automatically |
| Windows volume slider / mute while on I2S | No audible effect (matches SPDIF policy); use user/master volume commands |

---

## 9. Firmware Internals (summary for maintainers)

App developers can stop reading here. Full detail lives in `Documentation/current_architecture.md` ("Audio Input Source System" section); this is the orientation map.

- **Files:** `firmware/DSPi/i2s_input.c/.h` (lifecycle, DMA ring, poll), `firmware/DSPi/i2s_input.pio` (two RX programs), election helper and all orchestration in `main.c`, commands in `firmware/DSPi/vendor/vc_page_input.c` and `vc_page_clock.c`.
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
| `INPUT.I2S_RX_RATE` rate change in both roles | One mute cycle, BCK retunes, audio resumes at new rate |
| Preset roundtrip | Pin + rate + source restored per output-config mode |
| V16 slot on V17 firmware | Loads cleanly, I2S fields fall back to live defaults |
| MCK with input master | MCK present and frequency-correct with zero I2S outputs |
