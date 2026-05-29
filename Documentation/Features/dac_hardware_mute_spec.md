# DAC Hardware Mute Support — Implementation Spec

*Last updated: 2026-05-29*
*Status: implemented*
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
5. **Co-exists with existing soft-mute envelope.** The hardware mute is an *additional* layer, not a replacement. The existing `preset_loading` + `preset_mute_gain` envelope handles the digital data path; the hardware mute gives the DAC chip's analog stage time to ramp before clocks stop or resume. Together they give the DAC the best possible state around clock disruption.
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
    uint16_t release_ms;               // [0..500] post-clock-restart hold before unmute
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
// _assert(): drive the configured mute pin to "muted" and arm the hold
// deadline.  NON-BLOCKING — the caller polls _hold_elapsed() and defers
// the clock-stop until it returns true.  Idempotent: re-asserting does not
// re-arm (extend) the hold, so the main-loop gate may call it every loop.
void dac_hw_mute_assert(void);

// _hold_elapsed(): true once the hold armed by _assert() has elapsed (or
// the feature is off / no hold armed) — i.e. it is safe to stop clocks.
bool dac_hw_mute_hold_elapsed(void);

// _release(): begin release after clocks have restarted. If
// release_ms == 0, deasserts immediately. If release_ms > 0, leaves
// the pin asserted, records a deadline, and returns; dac_hw_mute_tick()
// deasserts later so the audio pipeline keeps draining.
void dac_hw_mute_release(void);

// Main-loop deadline service for delayed pipeline releases and
// diagnostic test pulses.
void dac_hw_mute_tick(void);

// Diagnostic: returns true if hardware mute is currently asserted.
bool dac_hw_mute_is_asserted(void);

// Test pulse: assert mute for ~1 s then release asynchronously.  For
// installer verification (REQ_TEST_DAC_HW_MUTE).
uint8_t dac_hw_mute_test_start(void);

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
static bool            s_pin_claimed;     // pin gpio_init'd + driven OUT
static bool            s_asserted;        // current physical/logical output
static bool            s_lifecycle_asserted;
static uint64_t        s_lifecycle_hold_us;     // pre-clock-stop hold deadline
static uint64_t        s_lifecycle_release_us;
static uint64_t        s_test_release_us;
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

    // (NEW) assert hardware mute and arm the hold deadline. NON-BLOCKING:
    // software mute state is visible before disruptive work begins; the
    // hardware mute gives the DAC analog stage time to ramp down, enforced
    // by the caller deferring the clock-stop (see gate below).
    dac_hw_mute_assert();
}

// Synchronous reset handlers (preset/factory/bulk/rate/stream/output-type/
// input-source) stop clocks in the SAME iteration they start, so they gate
// on this helper instead of busy-waiting the hold:
//
//   if (some_pending && pipeline_reset_ready()) {
//       some_pending = false;
//       ... usb_audio_drain_ring(); prepare_pipeline_reset(N); ...
//   }
//
// While the hold is incomplete the body is skipped and the pending flag is
// left set; the main loop keeps draining audio and the handler retries next
// iteration.  dac_hw_mute_assert() is idempotent (does not re-arm/extend the
// hold once asserted), so calling the gate every iteration is safe.
//
// CRITICAL: the gate engages ONLY the DAC hardware mute, NOT the soft-mute
// flag (preset_loading).  preset_loading also triggers the SPDIF lock-
// acquisition block, which runs EARLIER in the loop; holding it true across
// the wait would make that block fire drain_and_disable_outputs() on the very
// iteration the hold elapses — before the body's complete_pipeline_reset() —
// leaving spdif_prefilling set so the prefill path re-enables a second time
// with no teardown between, double-starting the SPDIF DMA and breaking inter-
// slot alignment.  The handler body's own prepare_pipeline_reset() engages the
// soft mute at the proper time (right before teardown), so the SPDIF block
// keeps its original ordering (reacts on the NEXT iteration, after complete).
static bool pipeline_reset_ready(void) {
    dac_hw_mute_assert();
    return dac_hw_mute_hold_elapsed();
}

// The SPDIF lock-acquisition path already defers its clock-stop
// (drain_and_disable_outputs) to a later iteration; it just adds the same
// barrier so a fast re-lock still honors the hold.  This guards the
// Category-B sites that assert via prepare_pipeline_reset() directly
// (boot-into-SPDIF, lock-loss, RX-pin swap), which DO set preset_loading:
//
//   if (LOCKED && preset_loading && !prefilling && dac_hw_mute_hold_elapsed())
//       drain_and_disable_outputs(); ...

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

    // (NEW) begin hardware-mute release now that clocks are running.
    // release_ms == 0 deasserts now; release_ms > 0 keeps the pin
    // asserted and lets dac_hw_mute_tick() deassert later.
    dac_hw_mute_release();

    // (existing) soft envelope ramps back up naturally when
    // preset_loading flips false at end of preset_mute_counter.
}
```

### 4.4 Hold/release timing — both asynchronous

Neither `hold_ms` nor `release_ms` is a busy-wait. Both are deadlines polled from the main loop, so audio servicing (USB ring drain, SPDIF poll, feedback) never stalls while the DAC ramps.

**Hold (`hold_ms`, pre-clock-stop):** `_assert()` drives the pin to muted, records `s_lifecycle_hold_us = now + hold_ms`, and returns. The barrier is enforced at the *clock-stop boundary*, which falls into two cases:

- **Synchronous handlers** (preset/factory/bulk/rate/stream/output-type/input-source) stop clocks in the same iteration they start. They gate their body on `pipeline_reset_ready()` (= `dac_hw_mute_assert()` + `dac_hw_mute_hold_elapsed()`); until the hold elapses the body is skipped, the pending flag stays set, and the loop keeps draining audio. The idempotent assert (no hold re-arm on re-assert) makes calling it every iteration safe — the deadline never slips out of reach. The gate engages **only** the hardware mute — it must NOT set `preset_loading`, because that flag also drives the earlier SPDIF lock-acquisition block; holding it true across the wait would make that block tear down outputs on the same iteration the body re-enables them (a double-enable that breaks slot alignment — see §4.3). The body's own `prepare_pipeline_reset()` engages the soft mute at the right moment (right before teardown), which also fences Core 1 after the body's final `usb_audio_drain_ring()`.
- **SPDIF lock/prefill path** (boot-into-SPDIF, USB→SPDIF, re-lock, RX-pin swap) already defers its clock-stop (`drain_and_disable_outputs()`) to a later iteration that waits for lock. These sites assert via `prepare_pipeline_reset()` directly (so they DO set `preset_loading`, the block's trigger); adding `&& dac_hw_mute_hold_elapsed()` to that block's condition guarantees the hold even on an instant re-lock, at the cost of a few extra non-blocking poll iterations.

During the hold the hardware mute pin is asserted (the DAC ramps to silence) while the rest of the pipeline keeps playing normally; the soft mute and clock-stop happen together when the body runs after the hold. This is correct because the hold exists solely to give the *DAC's analog stage* its head start — every other output is muted (soft) and torn down atomically by the body, exactly as without the feature.

A blocking busy-wait here was the previous design; it stalled the main loop for up to `hold_ms`, starving the SPDIF in-to-out path and delaying boot-into-SPDIF / input-source switches. Removing it is the whole point of this revision.

**Release (`release_ms`, post-clock-restart):** `_release()` runs after clocks restart; if `release_ms > 0` it keeps the pin asserted, records `s_lifecycle_release_us`, and returns. `dac_hw_mute_tick()` deasserts later, keeping the SPDIF in-to-out path draining during a long release hold.

**Flash writes** (`prepare_flash_write_operation()`) are inherently blocking (≈45 ms IRQ-off), so there is nothing to yield to. The hold is folded into the existing fade-settle loop, which runs until both the settle window and `dac_hw_mute_hold_elapsed()` are satisfied — adding no blocking beyond `max(settle, hold)` and keeping the pipeline fed throughout. When not streaming the loop is skipped (no audio ⇒ no clock-stop thump ⇒ no hold needed).

Both flash completion paths must release the mute they asserted. `complete_flash_write_operation_full()` (preset save/delete) and `complete_flash_write_operation_light()` (metadata-only writes — preset rename, startup policy, include_pins, master-volume mode/save, DAC-mute config) use the same source split: for **USB input** they release directly (full via `complete_pipeline_reset()`, light via a bare `dac_hw_mute_release()` — the light path never tears outputs down, so PIO clocks run continuously through the blackout and the deassert is clock-safe); for **SPDIF input** they skip the release and let the lock-acquisition prefill path own it after RX re-locks. The light path's release is easy to forget because, unlike the full path, it does not call `complete_pipeline_reset()` — omitting it leaves the DAC silent indefinitely after a metadata-only write while EMC is enabled on USB input.

**Boot output-type switch** (`process_type_switches()` from `core0_init()`, before the main loop) runs with no audio playing, so its clock-stop cannot thump; it proceeds without a hold. This is the one path where the hold is intentionally not enforced, and it is harmless because there is no signal to protect.

All lifecycle helpers early-exit (treating the hold as already elapsed) if the feature is disabled or no pin is claimed.

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

**TEST:** queues a deferred-to-main-loop asynchronous test pulse. `dac_hw_mute_test_start()` asserts the pin, records a ~1 s deadline, and returns; `dac_hw_mute_tick()` releases when the deadline expires. Audible result: audio silences for ~1 s, then returns. Confirms pin number, polarity, and that the DAC is actually being muted (not just receiving zero data).

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
| BSS (`s_cfg` + flags + async deadlines) | ~40 B |
| Flash directory growth | +16 B |
| Wire format growth | +16 B (V9 → V10) |
| Audio-path overhead (feature disabled or enabled) | **0 cycles, 0 branches** in the inner DSP loop |
| Pipeline-reset overhead (feature enabled) | `hold_ms` and `release_ms` are both asynchronous (deadline-polled); the main loop keeps draining audio throughout. Only inherently-blocking flash writes absorb the hold into their existing settle window. |

The audio-path zero-overhead is critical: the inner DSP loops never see this feature. All work happens in the pipeline-reset handler, which fires on lifecycle events only — never per-packet.

---

## 9. Test plan

1. **Feature disabled (default), baseline:** verify no audible regression on existing input-switch path. Same pops as today on PCM5102A; clean on S/PDIF.
2. **Feature enabled with valid pin, PCM5102A board:** verify input-switch pop eliminated. Scope on DAC AC-output cap should show ramp-to-silence over ~2 ms before BCK stops, instead of mid-amplitude cutoff.
3. **`hold_ms` = 0:** confirm pop returns (proves the hold is doing its job — minus one).
4. **`hold_ms` = 500 ms:** confirm the un-mute is delayed 500+ ms with no audible artifacts beyond the delay, **and that the main loop never stalls** — USB stays responsive (host control requests answered, no enumeration hiccup) and SPDIF input keeps draining throughout the hold (no consumer-fill growth / RX FIFO overflow). This is the non-blocking-hold regression check.
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
15. **`release_ms` with SPDIF input:** set a nonzero release hold, switch/relock SPDIF, and confirm the mute pin remains asserted for the configured period while `spdif_input_poll()` continues feeding output buffers without consumer fill growth.
16. **Non-blocking hold on SPDIF paths (`hold_ms` = 200):** boot into a preset whose default source is SPDIF, and separately switch USB→SPDIF at runtime. Scope the mute pin: assert→clock-stop gap must be ≥ `hold_ms` on every run, and the main loop must keep servicing USB/SPDIF during the hold (the reported symptom — boot/switch no longer blocks).
17. **Fast SPDIF re-lock honors the hold:** with SPDIF already present and stable, force a brief re-lock (e.g. momentary source toggle) so lock returns within a few ms. Confirm `drain_and_disable_outputs()` still waits for `dac_hw_mute_hold_elapsed()` — the assert→clock-stop gap stays ≥ `hold_ms` even though lock was near-instant.
18. **Metadata-only write releases the mute (USB input):** with EMC enabled, hold_ms = 100, and USB as the active input streaming audio, issue a preset rename (`REQ_PRESET_SET_NAME`). Confirm the mute pin asserts during the flash blackout and **deasserts within `release_ms` afterward** — i.e. audio returns. Repeat for startup-policy, include_pins, master-volume-mode, master-volume save, and a DAC-mute config write. Before this fix the pin stayed asserted indefinitely (DAC silent until the next reset). With SPDIF input, confirm the pin instead releases after RX re-locks and prefills (lock-acquisition path owns it).

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
