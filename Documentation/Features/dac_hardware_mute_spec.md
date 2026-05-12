# DAC Hardware Mute Support — Implementation Spec

*Last updated: 2026-05-12*
*Status: design — not yet implemented*
*Platforms: RP2040 + RP2350*

This spec defines hardware-mute integration for I²S DACs connected to DSPi, designed to eliminate the audible thump that some DAC chips produce when the I²S bit clock and LRCLK abruptly stop during pipeline reset (input source switch, preset load, sample-rate change, output type switch).

**Scope:** pin-based hardware mute only. DACs without a dedicated hardware mute pin (ES9038Q2M, CS43198, modern AKM/Cirrus parts that expose mute only via I²C/SPI registers) are **explicitly out of scope** — those chips ship with internal soft-mute and pop-suppression that mitigate the same problem from the chip side. Adding I²C/SPI for register-based mute would require a bus driver, per-chip register tables, and chip-detection logic that is disproportionate to the benefit. Users running register-mute DACs rely on the DAC's own internal protection.

---

## 1. Problem statement

The I²S DAC chips that DSPi drives via the `pico_audio_i2s_multi` library receive BCK, LRCLK, DATA, and optionally MCK from the RP2040/RP2350. During pipeline reset (`complete_pipeline_reset()` in `main.c`), the I²S state machine is stopped and the DMA aborted before being restarted in sync. This causes BCK / LRCLK to halt mid-cycle. Many DAC chips respond to clock cessation by:

1. The analog output stage holds whatever amplitude it was at, DC-stepping the output voltage.
2. AC-coupling capacitors at the DAC output charge to the held DC level.
3. When clocks resume and audio data flows again, the analog stage rapidly transitions to the new audio level — discharging the AC-coupling cap with an audible thump.

The existing software mitigations (the consumer-pool silence buffer with `I2S_PAD_PATTERN`, and the per-packet `preset_mute_gain` ramp) silence the *data* sent to the DAC, but they don't help during the brief window where clocks are stopped entirely.

A hardware mute provides a direct fix: assert the DAC's mute input *before* stopping the clocks. The DAC's analog output ramps to silence under its own internal control, then clock cessation has no audible consequence because the output is already silent.

---

## 2. Verified DAC mute behavior (in-scope chips only)

All times below assume **48 kHz** unless noted. Ramp time scales inversely with sample rate for sample-counted ramps.

| Chip | Mute pin | Polarity | Ramp formula | Time @ 48 kHz | Time @ 96 kHz | Time @ 44.1 kHz | Source |
|------|----------|----------|--------------|---------------|---------------|-----------------|--------|
| **PCM5102A** | XSMT (pin 17) | Active-low | 104 samples × (1/fs) | **2.17 ms** | 1.08 ms | 2.36 ms | TI datasheet, –1 dB per sample |
| **WM8741** | MUTEB | Active-low | 1022 × (4/fs) | **85.2 ms** | 42.6 ms | 92.7 ms | Wolfson datasheet |
| **AK4493** | SMUTE + ATS[1:0] | Active-high | 4096/2048/1024/512 × (1/fs) | **85 / 43 / 21 / 11 ms** | half | 1.09× | AKM datasheet, ATS selects divisor |

(For installations using AK4493 with non-default ATS, the user is responsible for matching `hold_ms` to the configured ATS divisor.)

**Out of scope** — these chips have no hardware mute pin and rely on register-only mute through I²C/SPI; they are not supported by this feature:

| Chip | Reason |
|------|--------|
| ES9038Q2M / Q8M / PRO | Register-only mute (Register 6 vol_rate). Internal soft-mute already handles clock-stop transients via DPLL behavior. |
| CS43198 | Register-only mute. Built-in Popguard® handles power and clock transients. |
| Modern AKM (AK4499 etc.) | Register-only mute. Internal soft-ramp adequate. |

---

## 3. Design goals

1. **Eliminate audible pop on I²S DACs with hardware mute pins during pipeline reset.**
2. **Single GPIO + minimal state.** No I²C / SPI / bus drivers, ever.
3. **Per-installation configuration via vendor commands.** User configures the GPIO, polarity, and hold time once; persists in directory.
4. **Single shared mute pin.** `complete_pipeline_reset()` is a global event that disables and re-enables ALL output slots together — there is no moment when one slot's clocks are stopping while another's are running. A per-slot mute array would be over-engineering with no behavioural benefit. Installations with multiple separate DACs wire their MUTE pins together to one RP2 GPIO externally; the firmware sees one pin regardless of how many physical chips it drives.
5. **Co-exists with existing soft-mute envelope.** The hardware mute is an *additional* layer, not a replacement. The existing `preset_loading` + `preset_mute_gain` envelope ramps the digital data to zero first; the new hardware mute then drives the DAC chip's mute pin. Together they give the DAC the best possible state before clock-stop.
6. **Slot-alignment invariant preserved.** Per CLAUDE.md, no firmware change may cause inter-slot phase drift. The hardware mute does not touch the I²S DMA / PIO start sequence — `audio_*_enable_sync()` remains the only mechanism for sample-aligned starts.
7. **Zero impact on platforms without a configured mute pin.** If `enabled = 0`, the audio path is byte-for-byte identical to today.
8. **No tech debt.** Self-contained module. Single responsibility. Tested at boundaries. No reserved fields for features we've explicitly excluded — `reserved1` exists only for the natural growth a 16-byte struct allows, not as a placeholder for register-mute hooks.
9. **Hard-tied to lifecycle events.** Integrates with the existing two-phase pipeline reset (`prepare_pipeline_reset()` / `complete_pipeline_reset()`), not a parallel state machine.

### Non-goals (and they stay non-goals)

- **Register-based mute via I²C/SPI.** Excluded permanently. Users with register-mute DACs rely on the DAC's internal protection.
- Per-channel mute (left vs. right).
- Automatic DAC chip detection.
- Soft-ramping the digital data through the hardware mute hold (the existing `preset_mute_gain` envelope already does this).
- A "DAC chip family" enum or per-chip configuration table.

---

## 4. Architecture

### 4.1 New module: `firmware/DSPi/dac_hw_mute.c/.h`

Self-contained module owning the hardware mute state and lifecycle. Same structural pattern as `lg_sound_sync.c`, `leveller.c`, `crossfeed.c` — domain logic out of `main.c`, one canonical home.

```c
// dac_hw_mute.h
#ifndef DAC_HW_MUTE_H
#define DAC_HW_MUTE_H

#include <stdint.h>
#include <stdbool.h>

#define DAC_HW_MUTE_PIN_NONE   0xFFu   // sentinel — no pin configured

// Per-installation configuration.  Lives in the flash directory sector
// (NOT per-preset) because the mute pin is a hardware-board attribute,
// not a listening-profile attribute.  Wire-stable 16-byte layout.
//
// Single shared GPIO drives the DAC's MUTE input.  See §3 for the
// rationale on why one pin is sufficient (pipeline reset is global —
// no per-slot granularity needed).
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                  // 0 = feature off, 1 = on
    uint8_t  active_low;               // 1 = assert pin LOW to mute, 0 = assert HIGH to mute
    uint8_t  pin;                      // GPIO; 0xFF = no pin
    uint8_t  reserved0;                // alignment for hold_ms
    uint16_t hold_ms;                  // [1..500] mute-attack hold before clock-stop
    uint16_t release_ms;               // [0..500] hold after un-mute before pipeline resumes
    uint8_t  reserved[8];              // Zero-fill; natural struct growth only — NOT reserved for
                                       // I²C/register mute (explicitly excluded by feature scope).
} DacHwMuteConfig;                     // 16 bytes

// One-time init.  Called from core0_init() after preset_boot_load().
// Reserves GPIOs and drives them un-muted per polarity.  Idempotent.
void dac_hw_mute_init(const DacHwMuteConfig *cfg);

// Validate + apply + persist.  Called by REQ_SET_DAC_HW_MUTE_CONFIG and
// bulk_params_apply().  Returns PIN_CONFIG_* status (config.h).
uint8_t dac_hw_mute_set_config(const DacHwMuteConfig *cfg);

// Snapshot current live config.  Called by REQ_GET_DAC_HW_MUTE_CONFIG
// and bulk_params_collect().
void dac_hw_mute_get_config(DacHwMuteConfig *out);

// Lifecycle hooks — called from main.c's pipeline reset.  Safe to call
// when feature disabled (no-op).
//
// _assert(): drive all configured mute pins to "muted" and busy-wait
// hold_ms so the DAC's internal soft ramp completes before clocks
// stop.  Called near the end of prepare_pipeline_reset() after the
// existing soft-mute envelope has fully attenuated.
void dac_hw_mute_assert(void);

// _release(): drive all configured mute pins to "un-muted" after
// clocks have restarted (so DAC analog ramp-up happens with clocks
// live).  Called at the end of complete_pipeline_reset(), after the
// synchronized PIO SM start.  If release_ms > 0, busy-waits that long
// before returning to let the DAC's analog stage ramp up before the
// audio pipeline un-mutes its own envelope.
void dac_hw_mute_release(void);

// Diagnostic: returns true if hardware mute is currently asserted.
bool dac_hw_mute_is_asserted(void);

// Test pulse: assert mute for ~1 s then release.  For installer
// verification (REQ_TEST_DAC_HW_MUTE).
uint8_t dac_hw_mute_run_test_pulse(void);

// Returns true iff `pin` is the configured mute pin.  Used by
// is_pin_in_use() so other pin-setting commands reject the mute pin.
bool dac_hw_mute_owns_pin(uint8_t pin);

#endif // DAC_HW_MUTE_H
```

### 4.2 Internal state (`dac_hw_mute.c`)

```c
// All writes happen on the main thread (init, vendor command apply,
// pipeline reset hooks).  No concurrent access — no volatile required.
static DacHwMuteConfig s_cfg;
static bool            s_asserted;
static bool            s_pin_claimed;     // pin gpio_init'd + driven OUT
```

### 4.3 Integration with pipeline reset

```c
// main.c — pseudocode showing where hooks land
static void prepare_pipeline_reset(uint32_t mute_samples) {
    // (existing) wait for Core 1 EQ worker to drain
    if (core1_mode == CORE1_MODE_EQ_WORKER) { ... }

    // (existing) engage soft-mute envelope
    preset_mute_counter = mute_samples;
    preset_loading = true;
    __dmb();

    // (NEW) assert hardware mute and busy-wait hold_ms.  Order
    // rationale: soft envelope engaged above → I²S DMA tail will
    // ramp toward zero on subsequent packets; hardware mute here →
    // DAC analog stage silences via its own internal ramp before
    // clocks stop.  Both layers protect against different failure
    // modes (data path vs. clock path).  The hardware mute hold is
    // sized to cover the DAC's internal ramp duration even if the
    // I²S DMA queue still contains non-silent samples — the DAC's
    // analog output is silent regardless of what data the chip is
    // receiving on its serial input.
    dac_hw_mute_assert();
}

static void complete_pipeline_reset(void) {
    // (existing) per-slot teardown — main IRQs enabled
    for (i = 0..N) teardown_output_slot(i);

    // (existing) tiny IRQ-disabled section
    flags = save_and_disable_interrupts();
    audio_spdif_enable_sync(...);
    audio_i2s_enable_sync(...);
    restore_interrupts(flags);

    // (existing) USB feedback reset
    reset_usb_feedback_loop();

    // (NEW) release hardware mute now that clocks are running again.
    // DAC sees clocks-then-release rather than release-then-clocks,
    // so the analog ramp-up happens with valid clocks.
    dac_hw_mute_release();

    // (existing) soft envelope ramps back up naturally when
    // preset_loading flips false at end of preset_mute_counter.
}
```

### 4.4 Wait helpers — busy-wait, not sleep

The `hold_ms` inside `_assert()` and the `release_ms` inside `_release()` use `time_us_64()`-based busy-wait, not `sleep_ms()`. Reasons:

- We're already inside a deferred pipeline-reset block. `sleep_ms()` would yield to other deferred work, which could trigger a nested pipeline reset (recursion).
- The waits are short (< 100 ms worst case).
- Audio is silent during these windows (hardware muted, or soft envelope at zero), so a USB packet pile-up on input doesn't produce audible artifacts.
- `tight_loop_contents()` is invoked inside the wait loops so the WFE hint can fire if the platform supports it.

Both helpers early-exit if the feature is disabled, so the cost is one byte load + branch when off.

### 4.5 Pin claim / validation

`dac_hw_mute_set_config()` validates:

- Each non-`PIN_NONE` pin is in the platform's valid GPIO range.
- No pin collides with: S/PDIF TX (`output_pins[]`), I²S BCK / LRCLK, I²S MCK, PDM, S/PDIF RX (`spdif_rx_pin`).
- No duplicate pins within the config itself.
- `hold_ms` ∈ [1, 500]; `release_ms` ∈ [0, 500].

Returns one of the existing `PIN_CONFIG_*` status codes from `config.h`. On success, releases previously-claimed pins (back to high-Z input) and claims the new ones (GPIO output, driven to un-muted state per polarity).

The pin-conflict check in `is_pin_in_use()` is extended to consult `dac_hw_mute_owns_pin()` so other pin-setting vendor commands reject the GPIO currently claimed by mute.

### 4.6 Why directory-stored, not per-preset

The hardware mute config is a **board-level attribute** — pin numbers, polarity, and DAC chip characteristics don't change between listening profiles. The directory sector (already host to startup mode, include_pins flag, master volume mode, master volume) is the right home.

Persistence:
- `PresetDirectory` struct gains a 16-byte `DacHwMuteConfig dac_hw_mute` field.
- Directory version bumped by 1.
- Pre-existing directories load the field as zero, which is `enabled = 0` → feature off → identical to old behavior. Backward-compatible without migration code.

---

## 5. Vendor commands

```c
// config.h additions — next free range after 0xE9
#define REQ_SET_DAC_HW_MUTE_CONFIG    0xEA  // payload = 16-byte DacHwMuteConfig
#define REQ_GET_DAC_HW_MUTE_CONFIG    0xEB  // returns 16-byte DacHwMuteConfig
#define REQ_TEST_DAC_HW_MUTE          0xEC  // no payload; asserts mute for ~1s
                                            // then releases, so installer can
                                            // verify pin + polarity audibly
```

**SET:** payload is the full struct. Firmware validates, persists to directory on success, applies live (re-init GPIOs). Returns `PIN_CONFIG_*` status byte.

**GET:** returns live config.

**TEST:** queues a deferred-to-main-loop test pulse: `_assert()` → 1-second wait → `_release()`. Returns immediately. Audible result: audio silences for ~1 s, then returns. Confirms pin number, polarity, and that the DAC is actually being muted (not just receiving zero data).

**Notify:** SET pushes a `notify_param_write()` on the `WireBulkParams.dac_hw_mute` field so other connected hosts re-render their UI.

---

## 6. Wire format additions

```c
// bulk_params.h — V10 addition
#define WIRE_FORMAT_VERSION  10   // V10: + WireDacHwMute

typedef struct __attribute__((packed)) {
    uint8_t  enabled;
    uint8_t  active_low;
    uint8_t  pin;
    uint8_t  reserved0;
    uint16_t hold_ms;
    uint16_t release_ms;
    uint8_t  reserved[8];
} WireDacHwMute;                    // 16 bytes — matches DacHwMuteConfig

typedef struct __attribute__((packed)) {
    // ... existing fields ...
    WireDacHwMute    dac_hw_mute;   // V10+
} WireBulkParams;                   // Total: 2928 bytes
```

`bulk_params_apply()` detects `format_version < 10` and skips the dac_hw_mute apply — old hosts' V9 payloads leave the config untouched (preserves whatever the directory had). New hosts on old firmware are gated by the dispatcher's payload-length check.

---

## 7. Edge cases

### 7.1 Feature toggle mid-playback

`REQ_SET_DAC_HW_MUTE_CONFIG` arriving during active audio: the SET handler is deferred to the main loop. Apply runs between audio packets. If the new config disables the feature, pins are released (driven to safe high-Z input state) and any held assert is cleared. If it changes pins, old pins are released and new pins are claimed with the un-muted level driven first to avoid an audible mute glitch on reconfiguration.

### 7.2 Reboot / factory reset

`apply_factory_defaults()` in `flash_storage.c` resets the live config to `{enabled = 0, ...}`. Pins are released to high-Z input. After factory reset the user re-configures the mute pin if desired.

### 7.3 Pin pre-empted by another feature

If `REQ_SET_OUTPUT_PIN` (or I²S BCK / MCK / SPDIF RX pin set) tries to claim a GPIO currently held by the mute config, the pin-setting command rejects with `PIN_CONFIG_PIN_IN_USE`. The user must `REQ_SET_DAC_HW_MUTE_CONFIG` to release the mute pin first. Convention matches every other pin conflict in the firmware — never silently steal.

### 7.4 Sample-rate dependency on `hold_ms`

PCM5102A and WM8741 ramps are sample-counted, so the actual chip ramp time is longer at lower sample rates. Recommendation: set `hold_ms` conservatively for the lowest supported rate. For PCM5102A: 5 ms covers 44.1 kHz (2.36 ms actual) with margin. For WM8741: 100 ms covers 44.1 kHz (92.7 ms actual). For AK4493 with default ATS=00: 100 ms.

The firmware does not auto-scale `hold_ms` with sample rate — keeps the audio path zero-cost and gives the user a single predictable number.

### 7.5 Multiple physical DACs

Not a software concern. The firmware has one mute pin. Installations with separate DACs on separate I²S slots wire their MUTE inputs together (or through external buffers if isolation is needed) so one RP2 GPIO drives all of them simultaneously — which is what we want anyway, since the pipeline reset disables every slot's clocks together.

### 7.6 No DAC connected (S/PDIF outputs only)

`enabled = 0`. No pin claim, no overhead. Audio path is byte-for-byte identical to today.

### 7.7 Mixed I²S / S/PDIF outputs

The mute pin fires regardless of slot output type — it's tied to the pipeline-reset lifecycle, not to any specific slot. An S/PDIF-only installation can still enable the mute pin if it's wired to an external mute switch or LED indicator; the firmware just drives the GPIO.

---

## 8. Performance & memory cost

| Item | Cost |
|------|------|
| `dac_hw_mute.c/.h` source | ~200 LOC |
| BSS (`s_cfg` + `s_asserted` + `s_pin_claimed`) | ~20 B |
| Flash directory growth | +16 B |
| Wire format growth | +16 B (V9 → V10) |
| Audio-path overhead (feature disabled or enabled) | **0 cycles, 0 branches** in the inner DSP loop |
| Pipeline-reset overhead (feature enabled) | + `hold_ms` busy-wait (typically 5–100 ms) + optional `release_ms` |

The audio-path zero-overhead is critical: the inner DSP loops never see this feature. All work happens in the pipeline-reset handler, which fires on lifecycle events only — never per-packet.

---

## 9. Test plan

1. **Feature disabled (default), baseline:** verify no audible regression on existing input-switch path. Same pops as today on PCM5102A; clean on S/PDIF.
2. **Feature enabled with valid pin, PCM5102A board:** verify input-switch pop eliminated. Scope on DAC AC-output cap should show ramp-to-silence over ~2 ms before BCK stops, instead of mid-amplitude cutoff.
3. **`hold_ms` = 0:** confirm pop returns (proves the hold is doing its job — minus one).
4. **`hold_ms` = 500 ms:** confirm input-switch takes 500+ ms but no audible artifacts beyond delay.
5. **Polarity inverted (`active_low` flipped):** confirm DAC stays muted permanently. Re-flip, confirm normal operation. (Proves polarity is honored.)
6. **Pin conflict rejection:** try setting mute pin to GPIO 6 (S/PDIF out slot 0). Expect `PIN_CONFIG_PIN_IN_USE`.
7. **Bulk apply:** send V10 bulk_params with dac_hw_mute section, confirm config takes effect identically to vendor command path.
8. **Old host on new firmware:** send V9 bulk_params, confirm dac_hw_mute config preserved (not zeroed).
9. **Factory reset:** confirm dac_hw_mute returns to `{enabled=0}`, pins released.
10. **Power-cycle:** confirm config persists across power loss.
11. **Inter-slot alignment with mute enabled:** play a phase-coherent test signal across all I²S slots, perform multiple input switches, verify sample-level alignment unchanged via loopback measurement.
12. **REQ_TEST_DAC_HW_MUTE:** confirm audio mutes for ~1 s, then returns. Confirm scope shows assertion transition matches configured polarity.
13. **`REQ_SET_OUTPUT_PIN` to currently-claimed mute pin:** confirm rejection with `PIN_CONFIG_PIN_IN_USE`.
14. **AK4493 with ATS=11 (11 ms ramp) and `hold_ms`=20:** verify no pop. Set `hold_ms`=5, verify pop returns.

---

## 10. Implementation steps

1. Create `firmware/DSPi/dac_hw_mute.c/.h` with full module API.
2. Add `DacHwMuteConfig` field to `PresetDirectory` in `flash_storage.c`; bump directory version.
3. Add `WireDacHwMute` section to `WireBulkParams`; bump `WIRE_FORMAT_VERSION` to 10.
4. Add bulk_params collect/apply integration (`bulk_params.c`).
5. Add vendor commands `REQ_SET_DAC_HW_MUTE_CONFIG` (0xEA), `REQ_GET_DAC_HW_MUTE_CONFIG` (0xEB), `REQ_TEST_DAC_HW_MUTE` (0xEC) in `vendor_commands.c`.
6. Add pin-conflict-check integration: extend the existing `is_pin_in_use()` (or equivalent) to consult `dac_hw_mute_owns_pin()`.
7. Add lifecycle hooks `dac_hw_mute_assert()` in `prepare_pipeline_reset()`; `_release()` in `complete_pipeline_reset()`. Both in `main.c`.
8. Add init call in `core0_init()` after `preset_boot_load()`.
9. Update `Documentation/current_architecture.md` with a new "DAC Hardware Mute" section.
10. Build both platforms (`build-rp2040`, `build-rp2350`). Verify BSS budget.
11. Hardware test against PCM5102A breakout (most common case): walk through test plan §9.

---

## 11. Open questions

- **Default `hold_ms` value:** the firmware accepts a minimum of 1; user is expected to set a value matching their DAC's published ramp time. Recommended starting points: 5 ms for PCM5102A, 100 ms for WM8741, 100 ms for AK4493 (ATS=00).
- **Should `REQ_TEST_DAC_HW_MUTE` be gated on `enabled = 1` and a valid pin?** Yes — testing while feature off or with no pin is meaningless. Returns `PIN_CONFIG_INVALID_OUTPUT` if either is missing.

---

## Sources

Mute timing data verified against:

- [PCM5102A datasheet (TI)](https://www.mouser.com/datasheet/2/405/pcm5102a-445759.pdf) — 104-sample soft ramp at 1 dB/sample.
- [AK4493 datasheet (Asahi Kasei)](https://static6.arrow.com/aropdfconversion/998f308f5eee03db15bcec9c8b34a5e254587fb0/ak4493seq-en-datasheet-myakm.pdf) — SMUTE pin + ATS[1:0] divisor selection.
- [WM8741 datasheet (Wolfson, now Cirrus)](https://manuals.lddb.com/DACs/Wolfson/WM8741.pdf) — MUTEB pin, ramp `1022 × (4/fs)`.
