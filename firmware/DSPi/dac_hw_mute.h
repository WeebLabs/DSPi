/*
 * dac_hw_mute.h — Hardware mute pin support for I²S DACs.
 *
 * Drives a configurable GPIO line that connects to a DAC chip's
 * dedicated MUTE input (e.g. PCM5102A XSMT, WM8741 MUTEB, AK4493
 * SMUTE).  The pin is asserted before pipeline reset stops the I²S
 * bit clock and LR clock, giving the DAC time to ramp its analog
 * output to silence under its own internal control.  Without this,
 * the DAC sees BCK/LRCLK abruptly cease mid-cycle and its analog
 * output DC-steps, producing an audible thump on input-source
 * switches, preset loads, and other lifecycle events that disable
 * the I²S state machine.
 *
 * SCOPE: hardware pin only.  DACs without a dedicated mute pin (ES9038
 * family, CS43198, modern AKM with register-only mute) are explicitly
 * out of scope — those chips ship with internal soft-mute and
 * Popguard-style protection that mitigates the same problem from the
 * chip side.  Supporting register-based mute would require an I²C/SPI
 * bus driver, per-chip register tables, and chip detection, which is
 * disproportionate to the benefit.
 *
 * INTEGRATION SEAMS (main.c):
 *   - core0_init() → dac_hw_mute_init(&dir_cache.dac_hw_mute) once
 *     after preset_boot_load() so the loaded config applies at boot.
 *   - prepare_pipeline_reset() → dac_hw_mute_wait_soft_envelope() then
 *     dac_hw_mute_assert() AFTER engaging the existing soft-mute
 *     envelope.  Order rationale: soft envelope first so the DAC's
 *     last DMA-fed sample before assert is silent; hardware mute
 *     second so the DAC's internal soft-ramp starts from already-
 *     silent amplitude rather than mid-amplitude.  Both layers
 *     protect against different failure modes (data path vs. clock
 *     path).
 *   - complete_pipeline_reset() → dac_hw_mute_release() AFTER the
 *     synchronized PIO SM start so the DAC sees clocks restarted then
 *     mute released — analog ramp-up happens with valid clocks.
 *
 * ZERO-COST WHEN DISABLED: every public entry point early-exits if
 * cfg.enabled == 0.  The audio-path inner loops never call this
 * module — all work happens in the deferred pipeline-reset block
 * which only fires on lifecycle events.  Steady-state audio
 * performance is byte-for-byte identical to a build with this module
 * removed.
 *
 * THREAD MODEL: all entry points are main-thread.  No ISR access, no
 * cross-core access.  GPIO state writes are non-atomic relative to
 * the audio path but the audio path never reads them.  Configuration
 * struct is a plain global, not volatile.
 *
 * See Documentation/Features/dac_hardware_mute_spec.md for the full
 * design rationale, verified DAC mute timings, and persistence model.
 */

#ifndef DAC_HW_MUTE_H
#define DAC_HW_MUTE_H

#include <stdint.h>
#include <stdbool.h>

/* Sentinel for "no mute pin assigned".  0xFF cannot collide with any
 * real RP2040/RP2350 GPIO number (max 29 / 47 respectively).  Stored
 * in DacHwMuteConfig.pin to mean "feature configured but no GPIO" —
 * functionally equivalent to enabled=0 and treated the same way in
 * all hot paths. */
#define DAC_HW_MUTE_PIN_NONE     0xFFu

/* Default GPIO assigned to a fresh / factory-reset directory.  GPIO 11
 * sits inside the contiguous output-pin neighborhood (SPDIF outs on
 * 6-9, PDM on 10, I²S BCK/LRCLK on 14-15, MCK on 13) but is itself
 * unused by any output, so users who route DAC mute through a header
 * pin near the output bus get a sensible default.  The pin only takes
 * effect when the user explicitly sets `enabled=1` via vendor command;
 * at-boot behavior is unchanged for users who never enable the feature.
 * If GPIO 11 is in use by the time the user enables the feature
 * (e.g., a custom preset reassigned an output to 11 or moved
 * spdif_rx_pin to 11), the runtime pin-conflict check in
 * dac_hw_mute_set_config() rejects with PIN_CONFIG_PIN_IN_USE and the
 * user picks another. */
#define DAC_HW_MUTE_DEFAULT_PIN  11u

/* Reasonable defaults for the most common DAC chip (PCM5102A on
 * AliExpress / Adafruit breakouts) — active-low XSMT with a soft
 * ramp of ~2 ms.  5 ms covers the ramp at 44.1 kHz with margin.
 * Users with other DACs override these via the vendor command. */
#define DAC_HW_MUTE_DEFAULT_ACTIVE_LOW  1u
#define DAC_HW_MUTE_DEFAULT_HOLD_MS     5u
#define DAC_HW_MUTE_DEFAULT_RELEASE_MS  0u

/* Range clamps for the hold and release dwell times.  hold_ms must be
 * at least 1 when the feature is enabled — a 0-ms hold would defeat
 * the whole purpose (we'd stop clocks before the DAC's analog ramp
 * completed).  500 ms is well above any datasheet-published ramp.
 * release_ms 0 means "no dwell" — the audio pipeline's soft envelope
 * handles the un-mute ramp naturally as preset_loading clears. */
#define DAC_HW_MUTE_HOLD_MS_MIN      1u
#define DAC_HW_MUTE_HOLD_MS_MAX      500u
#define DAC_HW_MUTE_RELEASE_MS_MIN   0u
#define DAC_HW_MUTE_RELEASE_MS_MAX   500u

/*
 * Per-installation configuration.  Wire/flash-stable 16-byte layout.
 * Lives in the flash directory sector (NOT per-preset) because mute
 * pin assignment is a board-level attribute that does not change
 * between listening profiles.
 *
 * Single shared GPIO drives the DAC's MUTE input.  This is the only
 * topology the firmware needs to support: the pipeline-reset path is
 * a global event that disables and re-enables ALL output slots
 * together — there is never a moment when one slot's I²S clocks are
 * stopping while another's are running.  So a per-slot mute array
 * would be over-engineering with no behavioural benefit.
 * Installations with multiple separate DACs (one per slot) wire all
 * their MUTE pins together to a single RP2 GPIO externally — every
 * DAC mutes simultaneously, same as if they shared one chip.
 *
 * The `reserved` bytes are zero-fill and pad the struct to a clean
 * 16-byte size; they are NOT earmarked for future register-mute
 * support — that feature is explicitly out of scope (see header
 * comment).
 */
typedef struct __attribute__((packed)) {
    uint8_t  enabled;                  /* 0 = feature off, 1 = on */
    uint8_t  active_low;               /* 1 = assert pin LOW to mute, 0 = HIGH */
    uint8_t  pin;                      /* GPIO; DAC_HW_MUTE_PIN_NONE = no pin */
    uint8_t  reserved0;                /* alignment for hold_ms */
    uint16_t hold_ms;                  /* mute-attack hold before clock-stop */
    uint16_t release_ms;               /* dwell after un-mute before resume */
    uint8_t  reserved[8];              /* zero-fill */
} DacHwMuteConfig;

/* Compile-time sanity check — the wire format depends on this being
 * exactly 16 bytes.  Caught at build time if anyone changes the struct
 * without updating the wire format. */
_Static_assert(sizeof(DacHwMuteConfig) == 16,
               "DacHwMuteConfig must be 16 bytes for wire/flash stability");

/* One-time initialization.  Called from core0_init() after
 * preset_boot_load() has loaded the directory.  Reserves GPIOs
 * (gpio_init + gpio_set_dir to OUT) and drives each configured pin
 * to its un-muted level per polarity.  Idempotent: subsequent calls
 * release any previously-claimed pins before claiming the new ones,
 * so this also serves as the re-init entry point used by
 * dac_hw_mute_set_config() on a live config change. */
void dac_hw_mute_init(const DacHwMuteConfig *cfg);

/* Validate + apply + persist.  Called by REQ_SET_DAC_HW_MUTE_CONFIG
 * and by bulk_params_apply().  Validates:
 *   - enabled ∈ {0,1}
 *   - active_low ∈ {0,1}
 *   - each non-NONE pin is in the platform's valid GPIO range
 *   - no non-NONE pin collides with output_pins[], i2s_bck_pin (or
 *     its +1 LRCLK partner), i2s_mck_pin (if MCK enabled), or
 *     spdif_rx_pin
 *   - no duplicate pins within the config itself
 *   - hold_ms ∈ [DAC_HW_MUTE_HOLD_MS_MIN, _MAX]
 *   - release_ms ∈ [DAC_HW_MUTE_RELEASE_MS_MIN, _MAX]
 *
 * Returns one of the PIN_CONFIG_* status codes from config.h.
 * PIN_CONFIG_SUCCESS means the config has been applied live AND
 * persisted to the flash directory.  Any other return code means the
 * live state is unchanged (config was rejected before any pin was
 * touched).
 *
 * SAFETY: when this is called mid-playback (e.g. via vendor command
 * during active audio), there is a brief window during which the
 * previously-claimed pins are released and the new pins are claimed.
 * If a pipeline reset happens during this window, the
 * assert/release path will see whichever config was active at the
 * moment — never a torn state, because the apply path serializes
 * with the main loop (vendor commands deferred from USB ISR are
 * processed between audio packets, and pipeline reset is also
 * deferred to the main loop).  No need for explicit interlock. */
uint8_t dac_hw_mute_set_config(const DacHwMuteConfig *cfg);

/*
 * Validate + apply + persist (continued).  Validation rules:
 *   - enabled ∈ {0,1}
 *   - active_low ∈ {0,1}
 *   - pin is either DAC_HW_MUTE_PIN_NONE or in the platform's valid GPIO range
 *   - pin does not collide with output_pins[], i2s_bck_pin (or +1 LRCLK),
 *     i2s_mck_pin (if enabled), or spdif_rx_pin
 *   - hold_ms ∈ [DAC_HW_MUTE_HOLD_MS_MIN, _MAX]
 *   - release_ms ∈ [DAC_HW_MUTE_RELEASE_MS_MIN, _MAX]
 */

/* Snapshot the current live config into *out.  Used by
 * REQ_GET_DAC_HW_MUTE_CONFIG and bulk_params_collect().  Always
 * succeeds; out is fully populated even when the feature is
 * disabled. */
void dac_hw_mute_get_config(DacHwMuteConfig *out);

/* ----- Pipeline-reset lifecycle hooks ----- */

/* Drive all configured mute pins to "muted" per polarity, then busy-
 * wait hold_ms so the DAC's internal soft-ramp completes before the
 * caller stops the I²S clocks.  Safe to call when feature disabled
 * (no-op).  Idempotent — calling assert while already asserted is a
 * harmless re-drive of the same pin levels. */
void dac_hw_mute_assert(void);

/* Drive all configured mute pins to "un-muted" per polarity.  Called
 * from complete_pipeline_reset() AFTER the synchronized PIO SM start,
 * so the DAC sees valid clocks the moment its analog ramp-up begins.
 * If release_ms > 0, busy-waits before returning to give the DAC's
 * analog stage time to settle before the audio pipeline's own
 * envelope un-mutes (which happens naturally when preset_loading
 * clears).  Safe to call when feature disabled (no-op). */
void dac_hw_mute_release(void);

/* ----- Diagnostics / vendor test ----- */

/* Returns true iff hardware mute is currently asserted (pins driven
 * to the muted level).  Used by REQ_GET_DAC_HW_MUTE_CONFIG response
 * shaping if a future UI wants to display the live state, and by the
 * test helper below.  Cheap (one byte load). */
bool dac_hw_mute_is_asserted(void);

/* Begin an asynchronous ~1 second test mute pulse.  Asserts the
 * mute pin immediately and records a release deadline; returns
 * without blocking.  The release fires from dac_hw_mute_tick()
 * called by the main loop on its normal cadence — so the audio
 * pipeline keeps draining the USB ring and feeding the SPDIF/I²S
 * DMA throughout the pulse, and only the DAC's analog output
 * actually mutes.  A synchronous busy-wait here would starve the
 * producer pool (~48 ms depth at 48 kHz) and the unmuted outputs
 * would underrun audibly even though the firmware never asked them
 * to silence.
 *
 * Returns PIN_CONFIG_INVALID_OUTPUT if the feature is disabled or
 * no pin is configured, PIN_CONFIG_SUCCESS if the pulse has been
 * started.  If a previous test is still in flight, the call is a
 * no-op (returns PIN_CONFIG_SUCCESS) — the deadline is not
 * extended. */
uint8_t dac_hw_mute_test_start(void);

/* Main-loop tick.  Cheap when no test is active (one variable load +
 * branch).  When a test is in flight and the deadline has passed,
 * releases the mute pin.  Call once per main-loop iteration alongside
 * usb_notify_tick() / lg_sound_sync_tick().  Same threading model:
 * single-producer (main thread only). */
void dac_hw_mute_tick(void);

/* Returns true iff `pin` is currently claimed by this module.  Used
 * by the pin-conflict check (is_pin_in_use in vendor_commands.c) so
 * other pin-setting commands reject the mute pin. */
bool dac_hw_mute_owns_pin(uint8_t pin);

#endif /* DAC_HW_MUTE_H */
