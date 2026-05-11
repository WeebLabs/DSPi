# Master Clock (MCK) Specification

*Last updated: 2026-05-09*

## 1. Overview

The Master Clock (MCK) is an optional GPIO output that provides a clean, sample-rate-locked clock to an external DAC's master clock input. The DSPi firmware drives MCK directly from one of the chip's four hardware **CLK_GPOUTn** clock peripheral outputs — no PIO state machine is consumed, the divider acts on the clk_sys hardware clock domain, and most common configurations end up with **integer dividers** at the device's fixed 307.2 MHz system clock.

MCK is independent from the I2S data/word clocks (BCK/LRCLK). It can be enabled even if no slot is configured for I2S output, although in practice you typically only enable MCK when at least one slot is driving an external DAC over I2S.

### 1.1 Quick facts for app developers

| Property | Value |
|----------|-------|
| Vendor commands | `0xC4`–`0xC9` (six commands: SET/GET enable, pin, multiplier) |
| Default state | Disabled |
| Default pin (RP2350) | GPIO 13 (`clk_gpout0`) |
| Default pin (RP2040) | GPIO 21 (`clk_gpout0`) |
| Default multiplier | 128× |
| Supported multipliers | 128×, 256× |
| Persisted in preset | Yes (`SLOT_DATA_VERSION ≥ 9`) |
| In bulk-params wire format | Yes (`WireI2SConfig`, `WIRE_FORMAT_VERSION ≥ 3`) |
| Notification offsets (v2) | `WireBulkParams.i2s_config.{mck_pin,mck_enabled,mck_multiplier}` |
| Hardware backing | CLK_GPOUTn (no PIO state machine) |
| AUXSRC | `clk_sys` (fixed @ 307.2 MHz) |
| Divider precision | 24 bits integer + 8 bits fractional (1/256 step) |

### 1.2 Why CLK_GPOUTn (and not PIO toggle)?

Earlier firmware revisions generated MCK using a 2-instruction PIO program (`audio_mck.pio`) that toggled a GPIO via side-set. That approach worked, but:

1. **Halved divider precision.** A toggle program runs at `pio_clk = 2 × MCK`, so the same 24.8 fixed-point divider register is consumed twice as fast as it is for a hardware clock output. The result was that every 256× combination, plus 96 kHz × 128×, ended up with a fractional divider — visible as audible jitter on the DAC's master-clock input.
2. **Burned a state machine.** PIO1 SM1 was permanently allocated for MCK on both platforms. With CLK_GPOUTn that SM is now free for future features.
3. **Couldn't survive 96 kHz × 256×.** The 6.25 fractional divider in that combo was unstable on real hardware (lock loss / silence), so the firmware silently force-clamped 256× to 128× at 96 kHz +. That clamp has been removed.

The CLK_GPOUTn implementation produces integer dividers for **every common Fs × multiplier combination** at sys_clk = 307.2 MHz, except 96 kHz × 256× (still fractional, but stable on real hardware).

---

## 2. Hardware Backing

### 2.1 Source clock

MCK is derived from `clk_sys` via the hardware clock generator's 24.8 fixed-point divider:

```
MCK_freq  = clk_sys / divider
divider   = sys_clk × 256 / (Fs × multiplier)         (24.8 representation)
divider_int  = divider >> 8                            (integer part, written to DIV_INT)
divider_frac = divider & 0xFF                          (fractional part, 8-bit, written to DIV_FRAC)
```

At sys_clk = 307.2 MHz the divider table is:

| Fs    | Mult | MCK target | Divider (24.8) | Integer? |
|-------|------|------------|----------------|----------|
| 48 kHz | 128× | 6.144 MHz  | 50.0           | ✓ Yes    |
| 48 kHz | 256× | 12.288 MHz | 25.0           | ✓ Yes    |
| 96 kHz | 128× | 12.288 MHz | 25.0           | ✓ Yes    |
| 96 kHz | 256× | 24.576 MHz | 12.5           | ✗ No (.5 fractional) |
| 44.1 kHz | 128× | 5.6448 MHz | ≈ 54.422 | ✗ No (the 44.1k family always has fractional dividers at sys_clk = 307.2 MHz) |
| 44.1 kHz | 256× | 11.2896 MHz | ≈ 27.211 | ✗ No |

**Note:** at 44.1 kHz the firmware drops sys_clk to 264.6 MHz (PLL family that gives integer SPDIF dividers at 44.1 k). At that sys_clk, MCK at 128× gives a divider of `264.6e6 × 256 / (44100 × 128) ≈ 11975 → 46.78` — still fractional. The 44.1 kHz family was never integer for MCK and isn't expected to be; the design priority is the 48 kHz family where DACs care most.

### 2.2 Pin → CLK_GPOUTn mapping

The Pico SDK macro `GPIO_TO_GPOUT_CLOCK_HANDLE(pin, default)` maps GPIOs to clock-output blocks:

**RP2040** (only four CLK_GPOUTn-capable pins):

| GPIO | clock output |
|------|--------------|
| 21 | `clk_gpout0` |
| 23 | `clk_gpout1` |
| 24 | `clk_gpout2` |
| 25 | `clk_gpout3` |

**RP2350** (adds GPIO 13 / 15 in the RP2350-specific table, falls through to RP2040 mapping for higher GPIOs):

| GPIO | clock output |
|------|--------------|
| 13 | `clk_gpout0` |
| 15 | `clk_gpout1` |
| 21 | `clk_gpout0` |
| 23 | `clk_gpout1` |
| 24 | `clk_gpout2` |
| 25 | `clk_gpout3` |

**DSPi-friendly subset** (after removing pins blocked by `is_valid_gpio_pin()` for control / SMPS / LED):

| Platform | Valid MCK pins |
|----------|----------------|
| RP2040 | **GPIO 21 only** |
| RP2350 | GPIO 13, 15, 21 (15 conflicts with I2S LRCLK when any output slot is set to I2S) |

GPIOs 23, 24, 25 are CLK_GPOUTn-capable but are board-reserved (control signal / SMPS / LED on Pico boards) and rejected by `is_valid_gpio_pin()` — both for MCK and for any other output assignment.

### 2.3 Default pin (per platform)

```c
// firmware/DSPi/config.h
#if PICO_RP2350
#define PICO_I2S_MCK_PIN  13   // clk_gpout0 on RP2350
#else
#define PICO_I2S_MCK_PIN  21   // clk_gpout0 on RP2040 (only DSPi-friendly choice)
#endif
```

The default takes effect on factory reset, on first boot before any preset has been saved, and as the migration target when an existing preset's MCK pin is invalid for the running platform (see § 7).

---

## 3. Parameters

### 3.1 Enable (`i2s_mck_enabled`)

| Property | Value |
|----------|-------|
| Type | `bool` (uint8_t on the wire: 0/1) |
| SET command | `0xC4` `REQ_SET_MCK_ENABLE` |
| GET command | `0xC5` `REQ_GET_MCK_ENABLE` |
| Default | `false` |
| Persisted | Yes |
| Notify offset (v2) | `offsetof(WireBulkParams, i2s_config.mck_enabled)`, length 1 |

When `true`, the CLK_GPOUTn block is configured (AUXSRC = clk_sys, divider loaded from current Fs × multiplier) and the GPIO pad mux is routed to the clock-output function. When `false`, the pad mux is disconnected (pin floats); the underlying generator continues running internally because the SDK has no public API to stop it.

### 3.2 Pin (`i2s_mck_pin`)

| Property | Value |
|----------|-------|
| Type | `uint8_t` (GPIO index) |
| SET command | `0xC6` `REQ_SET_MCK_PIN` |
| GET command | `0xC7` `REQ_GET_MCK_PIN` |
| Default | `PICO_I2S_MCK_PIN` (13 on RP2350, 21 on RP2040) |
| Persisted | Yes |
| Notify offset (v2) | `offsetof(WireBulkParams, i2s_config.mck_pin)`, length 1 |
| Mutable while enabled? | **No** — must disable MCK first |

The pin must satisfy **all** of:

1. `is_valid_gpio_pin(pin)` — not GPIO 12 (UART TX), not in 23–25 (board-reserved), in range for the platform.
2. `GPIO_TO_GPOUT_CLOCK_HANDLE(pin, clk_sys) != clk_sys` — pin must map to a CLK_GPOUTn block on the current platform.
3. Not in use by any other output / RX feature (`is_pin_in_use(pin, 0xFF) == false`).
4. MCK currently disabled (`i2s_mck_enabled == false`).

If any check fails, the SET handler returns a non-zero status byte (see § 4.2).

### 3.3 Multiplier (`i2s_mck_multiplier`)

| Property | Value |
|----------|-------|
| Type | `uint16_t` runtime; `uint8_t` on wire/flash |
| Wire encoding | `0` = 128×, `1` = 256× |
| SET command | `0xC8` `REQ_SET_MCK_MULTIPLIER` |
| GET command | `0xC9` `REQ_GET_MCK_MULTIPLIER` |
| Default | `128` (wire `0`) |
| Persisted | Yes |
| Notify offset (v2) | `offsetof(WireBulkParams, i2s_config.mck_multiplier)`, length 1 |

The MCK frequency is `Fs × multiplier`. Both 128× and 256× are accepted at every supported sample rate (44.1 / 48 / 88.2 / 96 kHz). The earlier 96 kHz × 256× clamp has been removed — see § 1.2.

---

## 4. Vendor Commands

All MCK commands use the standard DSPi vendor-class control transfer pattern (`bmRequestType` recipient = vendor, type = vendor; routed via `tud_vendor_control_xfer_cb` in `vendor_commands.c`).

### 4.1 Command table

| Code | Name | Direction | wValue | wLength | Payload | Response |
|------|------|-----------|--------|---------|---------|----------|
| `0xC4` | `REQ_SET_MCK_ENABLE` | OUT-then-IN | 0 = disable, 1 = enable | 0 | (none) | 1 byte status (`PIN_CONFIG_*`) |
| `0xC5` | `REQ_GET_MCK_ENABLE` | IN | 0 | 1 | — | 1 byte: 0/1 |
| `0xC6` | `REQ_SET_MCK_PIN` | OUT-then-IN | new GPIO index | 0 | (none) | 1 byte status (`PIN_CONFIG_*`) |
| `0xC7` | `REQ_GET_MCK_PIN` | IN | 0 | 1 | — | 1 byte: GPIO index |
| `0xC8` | `REQ_SET_MCK_MULTIPLIER` | OUT-then-IN | 0 = 128×, 1 = 256× | 0 | (none) | 1 byte status (`PIN_CONFIG_*`) |
| `0xC9` | `REQ_GET_MCK_MULTIPLIER` | IN | 0 | 1 | — | 1 byte: 0 (128×) / 1 (256×) |

"OUT-then-IN" SET commands send the parameter in the setup packet's `wValue` and immediately return a 1-byte status byte over the IN data stage — there is no separate OUT data phase.

### 4.2 Status byte (`PIN_CONFIG_*`)

Returned by the three SET commands:

| Code | Name | Meaning |
|------|------|---------|
| `0x00` | `PIN_CONFIG_SUCCESS` | Operation completed; new state has been applied |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | (Pin commands only.) Pin failed `is_valid_gpio_pin()` *or* has no `clk_gpoutN` mapping on this platform *or* multiplier wValue out of range |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | (Pin commands only.) Pin is currently assigned to another output / SPDIF RX / I2S BCK or LRCLK |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | (Pin commands only.) Cannot change pin while MCK is enabled |

`PIN_CONFIG_INVALID_OUTPUT` (`0x03`) is shared with other vendor commands and is not returned by the MCK handlers.

### 4.3 Behavioural details per command

**`REQ_SET_MCK_ENABLE`** (0xC4):

1. If `wValue != 0` and MCK was previously off:
   - Calls `audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier)` to compute and stash the divider.
   - Calls `audio_i2s_mck_set_enabled(true)` — configures CLK_GPOUTn (AUXSRC, DIV, ENABLE) and routes the pad mux.
   - Sets `i2s_mck_enabled = true`.
2. If `wValue == 0` and MCK was on: disconnects pad mux, clears `i2s_mck_enabled`.
3. Always emits a v2 notification at offset `i2s_config.mck_enabled`.
4. Returns `PIN_CONFIG_SUCCESS` unconditionally.

**`REQ_SET_MCK_PIN`** (0xC6):

1. Validates against `is_valid_gpio_pin()` → returns `INVALID_PIN` on failure.
2. Validates against `GPIO_TO_GPOUT_CLOCK_HANDLE()` → returns `INVALID_PIN` if pin has no clock-output mapping.
3. If `i2s_mck_enabled` → returns `OUTPUT_ACTIVE` (must disable first).
4. If `is_pin_in_use(pin, 0xFF)` → returns `PIN_IN_USE`.
5. If new pin equals current pin → no-op `SUCCESS`.
6. Otherwise: calls `audio_i2s_mck_change_pin(pin)` (book-keeping only, no hardware effect; takes effect on next enable), updates `i2s_mck_pin`, emits notification, returns `SUCCESS`.

**`REQ_SET_MCK_MULTIPLIER`** (0xC8):

1. `wValue` must be 0 or 1 → otherwise returns `INVALID_PIN`.
2. Maps wValue to `i2s_mck_multiplier ∈ {128, 256}`.
3. If MCK is currently enabled, hot-loads the new divider via `audio_i2s_mck_update_frequency()`. The CLK_GPOUTn `DIV` register is reloaded glitchlessly (no pin transitions are dropped; ENABLE stays asserted).
4. Emits notification; returns `SUCCESS`.

**`REQ_GET_*`** commands (0xC5/0xC7/0xC9): single-byte read of current state, no validation, no side effects.

### 4.4 Common app patterns

#### 4.4.1 Enable MCK with a custom multiplier

```
1. control SET 0xC4 wValue=0          → status 0x00 (force MCK off, in case it was on)
2. control SET 0xC8 wValue=1          → status 0x00 (set 256×)
3. control SET 0xC4 wValue=1          → status 0x00 (enable; MCK starts at Fs × 256)
```

If you skip step 1 and MCK is already on, `REQ_SET_MCK_MULTIPLIER` will hot-update the divider without re-toggling the pad mux (no DAC PLL relock chirp).

#### 4.4.2 Move MCK to a different GPIO

```
1. control SET 0xC4 wValue=0          → status 0x00 (must disable first)
2. control SET 0xC6 wValue=21         → status 0x00 (or INVALID_PIN / PIN_IN_USE)
3. control SET 0xC4 wValue=1          → status 0x00 (re-enable on new pin)
```

Trying step 2 while MCK is on returns `OUTPUT_ACTIVE` (0x04). The same status is returned for changing the I2S BCK pin while any I2S slot is active — same pattern.

#### 4.4.3 Read full MCK state in one round trip

If your host SDK already uses `REQ_GET_ALL_PARAMS` (0xA0) for bulk state, the `WireI2SConfig` block already contains all three MCK fields. Prefer the bulk transfer over three separate GETs.

---

## 5. Lifecycle and Ordering Rules

### 5.1 Boot sequence

1. `usb_sound_card_init()` calls `audio_i2s_mck_setup(i2s_mck_pin)`. This **only records the pin** — no hardware effect, no clock generation. The default value of `i2s_mck_pin` is `PICO_I2S_MCK_PIN`.
2. `preset_boot_load()` runs and may overwrite `i2s_mck_pin` / `i2s_mck_enabled` / `i2s_mck_multiplier` with values from the active preset (or platform defaults if the preset's pin is invalid — see § 7).
3. `core0_init()` enables I2S slots that the preset configured. If `i2s_mck_enabled` is set and at least one slot is I2S, MCK is started here via the same divider-then-enable sequence as the vendor command.

### 5.2 Sample rate change

When the device negotiates a new sample rate (USB SOF feedback or SPDIF input lock acquisition), `perform_rate_change(new_freq)` runs:

1. Updates output PIO dividers (BCK / SPDIF / I2S TX).
2. **If MCK is enabled**, calls `audio_i2s_mck_update_frequency(new_freq, i2s_mck_multiplier)`. This hot-loads the new divider into CLK_GPOUTn — glitchless, no pad mux disturbance.

The host doesn't need to do anything here; the firmware handles the rate change transparently.

### 5.3 Output type switch (S/PDIF ↔ I2S)

`process_type_switches()` recomputes the MCK gating:

| Slot config after switch | MCK enabled bit | MCK actually running |
|---|---|---|
| At least one I2S slot AND `i2s_mck_enabled = true` | true | **Yes** (started if not already) |
| No I2S slots | (preserved) | **No** (stopped, regardless of `i2s_mck_enabled`) |

The user-facing `i2s_mck_enabled` bit is preserved across "no I2S slots" → "I2S slot added" transitions: enabling the first I2S slot will re-start MCK at the previous user setting.

### 5.4 Preset load / bulk-params apply

These paths apply persisted values to live state. The MCK pin is validated with `GPIO_TO_GPOUT_CLOCK_HANDLE()` — if the pin doesn't map to CLK_GPOUTn on the current platform, it is replaced with the platform default and `i2s_mck_enabled` is forced to `false`. See § 7.

After the apply, if MCK is still enabled, `audio_i2s_mck_update_frequency()` is called so the divider matches the current sample rate (preset load may have been triggered while playing audio at any rate).

### 5.5 Servo (SPDIF input clock-domain)

When SPDIF input is the active source, the resampler-mode clock servo applies ppm-level corrections to the MCK divider via `audio_i2s_mck_set_divider(div_24_8)`. This bypasses the Fs × multiplier math and writes a raw 24.8 divider directly. The DAC stays frequency-locked to whatever the SPDIF source is doing.

The host doesn't observe the servo'd divider — it doesn't change the multiplier or the nominal Fs. The vendor commands continue to report the user-set multiplier value, not the servoed running divider.

---

## 6. Wire Format and Persistence

### 6.1 `WireI2SConfig` (16 bytes)

Defined in `firmware/DSPi/bulk_params.h` Section 11. Used by both bulk parameter transfers (`REQ_GET_ALL_PARAMS` / `REQ_SET_ALL_PARAMS`) and v2 notifications (`notify_param_write`):

```c
typedef struct __attribute__((packed)) {
    uint8_t  output_types[WIRE_MAX_SPDIF_INSTANCES]; // 4 bytes (one per slot, 0=SPDIF, 1=I2S)
    uint8_t  bck_pin;                // 1 byte
    uint8_t  mck_pin;                // 1 byte
    uint8_t  mck_enabled;            // 1 byte: 0 = off, 1 = on
    uint8_t  mck_multiplier;         // 1 byte: 0 = 128×, 1 = 256×
    uint8_t  reserved[8];            // future expansion (must be 0)
} WireI2SConfig;                     // 16 bytes
```

The `WireI2SConfig` lives at `WireBulkParams.i2s_config`. Notification offsets:

| Field | offsetof(WireBulkParams, …) | Length |
|-------|-----------------------------|--------|
| `i2s_config.mck_pin` | depends on `WIRE_FORMAT_VERSION` (use `offsetof()`) | 1 |
| `i2s_config.mck_enabled` | (use `offsetof()`) | 1 |
| `i2s_config.mck_multiplier` | (use `offsetof()`) | 1 |

**Always derive offsets from the wire format struct** (`offsetof(WireBulkParams, i2s_config.mck_pin)`) — do not hard-code them. Newer `WIRE_FORMAT_VERSION`s can shuffle field positions.

### 6.2 Preset (`SLOT_DATA_VERSION`)

`PresetSlot` (in `flash_storage.c`) stores the same three values:

```c
uint8_t i2s_mck_pin;         // GPIO index
uint8_t i2s_mck_enabled;     // 0 / 1
uint8_t i2s_mck_multiplier;  // 0 = 128×, 1 = 256× (V11+); raw {128, 0} (V9-V10)
```

Version-dependent encoding (the firmware decodes both legacy and modern):

| `SLOT_DATA_VERSION` | `i2s_mck_multiplier` encoding |
|---------------------|-------------------------------|
| V9, V10 | Raw value: `128` = 128×, `0` = 256× |
| V11+ | Enum: `0` = 128×, `1` = 256× |

The CLK_GPOUTn migration is **value-only** — no `SLOT_DATA_VERSION` bump was required because the field layout did not change. Existing presets continue to load.

### 6.3 Notification protocol v2

Every successful SET handler emits a `notify_param_write(offset, length, ptr)` call so other USB hosts (or the same host, on re-poll) see the new state. For MCK:

- `REQ_SET_MCK_PIN` → emits 1 byte at `i2s_config.mck_pin`
- `REQ_SET_MCK_ENABLE` → emits 1 byte at `i2s_config.mck_enabled`
- `REQ_SET_MCK_MULTIPLIER` → emits 1 byte at `i2s_config.mck_multiplier`

See `Documentation/Features/notification_protocol_v2_spec.md` for the notification framing.

---

## 7. Migration: Existing Presets / Bulk Payloads

The CLK_GPOUTn implementation requires the MCK pin to be a CLK_GPOUTn-capable GPIO on the **running** platform. Because GPIO 13 is `clk_gpout0` on **RP2350 only**, an RP2040 board loading a preset that was saved with `mck_pin = 13` would have a non-functional MCK if applied verbatim.

The firmware applies the following migration during `apply_slot_to_live()` (preset load) and `bulk_params_apply()` (bulk SET):

```c
if (GPIO_TO_GPOUT_CLOCK_HANDLE(loaded_mck_pin, clk_sys) == clk_sys) {
    // Stored pin is not CLK_GPOUTn-capable on this platform.
    i2s_mck_pin     = PICO_I2S_MCK_PIN;   // platform default (13 RP2350 / 21 RP2040)
    i2s_mck_enabled = false;              // force off — user must re-enable on a valid pin
} else {
    i2s_mck_pin     = loaded_mck_pin;
    i2s_mck_enabled = (loaded_mck_enabled != 0);
}
```

This is silent at the wire / flash level — there is no error returned to the host. If you want to detect that migration occurred, compare the MCK fields before and after the bulk SET (or check the directory `mck_pin` against the platform default).

**Recommendation for app developers:** when porting a preset between platforms (export from RP2350, import on RP2040), call `REQ_GET_MCK_PIN` after import. If it equals `21` and you expected `13` (or vice versa), prompt the user that MCK was moved to the platform default and is currently disabled.

---

## 8. Host SDK Code Patterns

### 8.1 Enabling MCK with libusb / Python

```python
import usb.core, usb.util

DSPI_VENDOR  = 0x239A   # placeholder — see device descriptors
REQ_SET_MCK_ENABLE     = 0xC4
REQ_GET_MCK_ENABLE     = 0xC5
REQ_SET_MCK_PIN        = 0xC6
REQ_GET_MCK_PIN        = 0xC7
REQ_SET_MCK_MULTIPLIER = 0xC8
REQ_GET_MCK_MULTIPLIER = 0xC9

PIN_CONFIG_SUCCESS       = 0x00
PIN_CONFIG_INVALID_PIN   = 0x01
PIN_CONFIG_PIN_IN_USE    = 0x02
PIN_CONFIG_OUTPUT_ACTIVE = 0x04

def vendor_set(dev, request, wValue):
    """Returns the 1-byte status from a SET-then-IN command."""
    rsp = dev.ctrl_transfer(
        bmRequestType = 0xC2,   # IN | vendor | interface
        bRequest      = request,
        wValue        = wValue,
        wIndex        = 0,
        data_or_wLength = 1,
        timeout       = 1000)
    return rsp[0]

def vendor_get(dev, request, length=1):
    return dev.ctrl_transfer(0xC2, request, 0, 0, length, timeout=1000)

def enable_mck(dev, multiplier=256, pin=None):
    """Configure and enable MCK.  Disables first to allow pin/mult changes."""
    # Step 1: force MCK off so we can change pin / multiplier
    assert vendor_set(dev, REQ_SET_MCK_ENABLE, 0) == PIN_CONFIG_SUCCESS

    # Step 2: optionally change the pin
    if pin is not None:
        st = vendor_set(dev, REQ_SET_MCK_PIN, pin)
        if st != PIN_CONFIG_SUCCESS:
            raise RuntimeError(f"REQ_SET_MCK_PIN({pin}) failed: status 0x{st:02x}")

    # Step 3: set the multiplier (0 = 128×, 1 = 256×)
    wire_mult = 1 if multiplier == 256 else 0
    st = vendor_set(dev, REQ_SET_MCK_MULTIPLIER, wire_mult)
    if st != PIN_CONFIG_SUCCESS:
        raise RuntimeError(f"REQ_SET_MCK_MULTIPLIER({multiplier}) failed: status 0x{st:02x}")

    # Step 4: enable
    assert vendor_set(dev, REQ_SET_MCK_ENABLE, 1) == PIN_CONFIG_SUCCESS

def get_mck_state(dev):
    return {
        "enabled":    bool(vendor_get(dev, REQ_GET_MCK_ENABLE)[0]),
        "pin":        int(vendor_get(dev, REQ_GET_MCK_PIN)[0]),
        "multiplier": 256 if vendor_get(dev, REQ_GET_MCK_MULTIPLIER)[0] == 1 else 128,
    }
```

### 8.2 Validating a pin client-side

Saves a USB round trip when the user picks an MCK pin in the UI — replicate the firmware's GPIO_TO_GPOUT_CLOCK_HANDLE check:

```python
def is_mck_pin_valid(platform_is_rp2350: bool, pin: int) -> bool:
    if pin == 12 or 23 <= pin <= 25:
        return False                # reserved by is_valid_gpio_pin
    if pin in (21,):                # gpout0 on both
        return True
    if platform_is_rp2350 and pin in (13, 15):
        return True
    return False                    # any other pin has no GPOUTn mapping
```

The platform identity is exposed via `REQ_GET_DEVICE_INFO` (see `device_identification_spec.md`).

### 8.3 Error handling

| Status | Meaning | Recommended UX |
|--------|---------|----------------|
| `0x00` `SUCCESS` | Operation applied | Update UI from new GET values |
| `0x01` `INVALID_PIN` | Pin has no GPOUTn mapping or is otherwise unsuitable | Show the per-platform valid set (§ 2.2); refuse the choice |
| `0x02` `PIN_IN_USE` | Pin claimed by another output / RX | Suggest a different pin or first move the conflicting feature |
| `0x04` `OUTPUT_ACTIVE` | Tried to change pin while MCK enabled | Auto-issue `REQ_SET_MCK_ENABLE(0)` first, then retry |

App code that batches changes (e.g., importing a preset) should disable MCK first, apply pin + multiplier, then re-enable — this avoids the `OUTPUT_ACTIVE` status in the middle of the sequence.

---

## 9. Verification

### 9.1 Static checks (no hardware needed)

```sh
# Both platforms build cleanly, no warnings
cmake --build build-rp2040 --clean-first
cmake --build build-rp2350 --clean-first

# No stale PIO-MCK references remain
grep -rn "audio_mck_program\|mck_pio\b\|mck_sm\b\|mck_program_offset" firmware/
grep -rn "is_mck_multiplier_supported_for_rate\|sanitize_mck_multiplier_for_rate" firmware/
# Both should return only intentional comment-out lines.

# audio_mck.pio is gone
ls firmware/pico-extras/src/rp2_common/pico_audio_i2s_multi/audio_mck.pio
# expected: No such file or directory
```

### 9.2 Vendor command tests

| Test | Expected result |
|------|-----------------|
| `REQ_SET_MCK_PIN(13)` on RP2350 | `0x00 SUCCESS` |
| `REQ_SET_MCK_PIN(13)` on RP2040 | `0x01 INVALID_PIN` (no GPOUTn mapping) |
| `REQ_SET_MCK_PIN(17)` on either | `0x01 INVALID_PIN` |
| `REQ_SET_MCK_PIN(23)` on either | `0x01 INVALID_PIN` (board-reserved) |
| `REQ_SET_MCK_PIN(21)` on RP2040 | `0x00 SUCCESS` |
| `REQ_SET_MCK_PIN` while enabled | `0x04 OUTPUT_ACTIVE` |
| `REQ_SET_MCK_MULTIPLIER(1)` at 96 kHz | `0x00 SUCCESS` (was rejected pre-CLK_GPOUTn) |
| `REQ_SET_MCK_MULTIPLIER(2)` (out of range) | `0x01 INVALID_PIN` |

### 9.3 Frequency verification (oscilloscope on the MCK pin)

| Configuration | Expected frequency | Notes |
|---|---|---|
| 48 kHz × 128× | 6.144 MHz | Integer divider 50.0 |
| 48 kHz × 256× | 12.288 MHz | Integer divider 25.0 |
| 96 kHz × 128× | 12.288 MHz | Integer divider 25.0 |
| 96 kHz × 256× | 24.576 MHz | Fractional 12.5 — slightly noisier |
| Sample-rate change mid-playback | New frequency takes effect within one DMA wrap | No PLL-relock chirp on the DAC |

### 9.4 Preset migration test

1. On RP2350: enable MCK, save preset slot 0. Read back: `mck_pin = 13`, `mck_enabled = 1`.
2. Export preset (host-side dump of slot bytes).
3. Flash RP2040 firmware. Import the preset to slot 0.
4. Read back: **`mck_pin = 21`, `mck_enabled = 0`** (migration applied silently).
5. App should detect the difference and prompt the user.

### 9.5 SPDIF servo regression

With SPDIF RX locked and resampler enabled, monitor MCK divider updates over a 30-second window. The servo should make ppm-level corrections (visible as small DIV_FRAC oscillations); the DAC must remain locked across at least ±100 ppm of source-rate drift.

---

## 10. Implementation References

| Area | File | Where |
|------|------|-------|
| Library API (state + functions) | `firmware/pico-extras/src/rp2_common/pico_audio_i2s_multi/audio_i2s_multi.c` | "MCK generator state" / "MCK generator functions" sections |
| Library header | `firmware/pico-extras/src/rp2_common/pico_audio_i2s_multi/include/pico/audio_i2s_multi.h` | "MCK (Master Clock) generator API" section |
| Default pin | `firmware/DSPi/config.h` | `PICO_I2S_MCK_PIN` |
| Boot-time setup | `firmware/DSPi/usb_audio.c` | `usb_sound_card_init()` → `audio_i2s_mck_setup(i2s_mck_pin)` |
| Vendor command handlers | `firmware/DSPi/vendor_commands.c` | `REQ_SET_MCK_*` / `REQ_GET_MCK_*` cases |
| Pin validation | `firmware/DSPi/vendor_commands.c` | `is_valid_gpio_pin()`, `is_pin_in_use()` |
| Preset persistence | `firmware/DSPi/flash_storage.c` | `apply_slot_to_live()` I2S configuration block |
| Bulk wire format | `firmware/DSPi/bulk_params.h` / `bulk_params.c` | `WireI2SConfig`, `bulk_params_apply()` I2S section |
| Sample rate change hooks | `firmware/DSPi/main.c` | `perform_rate_change()`, `process_type_switches()`, preset_load_pending block, bulk_apply_pending block |
| SPDIF servo | `firmware/DSPi/spdif_input.c` | `spdif_input_update_clock_servo()` → `audio_i2s_mck_set_divider()` |
| SDK macro | `firmware/pico-sdk/src/rp2_common/hardware_clocks/include/hardware/clocks.h` | `GPIO_TO_GPOUT_CLOCK_HANDLE`, `clock_gpio_init_int_frac8` |

## 11. Future Extensions

The current implementation hard-codes `AUXSRC = clk_sys` and supports only 128× / 256× multipliers. Possible future extensions, listed in approximate order of likely value:

1. **384× / 512× multipliers** for higher-end DACs (ESS, AKM Velvet Sound). Requires sys_clk math: at 307.2 MHz, 48 kHz × 384× = 18.432 MHz → divider 16.667 (fractional); at 384 kHz × 128× the divider would also be fractional. Not a quick win.
2. **Alternate AUXSRC** (e.g., `pll_usb` for chip-isolated jitter). Would let the user pick a different clock domain for MCK independently of clk_sys. Easy to add — change the `CLK_GPOUT0_CTRL_AUXSRC_VALUE_*` constant and recompute divider math against `clock_get_hz(<new src>)`.
3. **Multiple MCK outputs** (use clk_gpout1/2/3 simultaneously to fan out to multiple DACs). The library would need per-pin state and a selector; the existing code is single-instance.
4. **Stop the GPOUTn generator on disable** (vs. just disconnecting the pad mux). Would require direct register writes to `clocks_hw->clk[].ctrl` since the SDK has no public API for this. Power impact is negligible (sub-µA), so not pursued in v1.
5. **Surface PIO1 SM1 as an explicitly-allocatable resource** for future features (extra PDM channel, GPIO output peripheral, etc.). Currently unused but unreserved.
