/*
 * dac_hw_mute.c — Hardware mute pin support for I²S DACs.
 *
 * See dac_hw_mute.h for the public API and
 * Documentation/Features/dac_hardware_mute_spec.md for the design.
 *
 * Implementation notes:
 *
 *   - All state is plain (non-volatile).  Every public entry point
 *     runs on the main thread; no ISR access.  GPIO writes use the
 *     SDK's gpio_put() which compiles to a single SIO register
 *     store — atomic on both Cortex-M0+ and M33.
 *
 *   - "Un-muted level" is the polarity-inverse of "muted level":
 *     active_low=1 → un-muted = HIGH, muted = LOW; active_low=0 →
 *     un-muted = LOW, muted = HIGH.  Pins are driven to the un-muted
 *     level the moment they are claimed (gpio_init + set_dir), so
 *     there is no audible mute pulse during normal init.
 *
 *   - Pin release on config change drives the pin to high-Z input
 *     before re-init, not to a "safe" level — the GPIO is no longer
 *     owned by this module and may be repurposed by another
 *     subsystem.  Driving a level would briefly assert mute on a
 *     freshly-released pin which is wasteful.
 *
 *   - The pre-clock-stop hold uses time_us_64() with
 *     tight_loop_contents() so the ARM WFE hint can fire.  sleep_ms()
 *     is avoided because we run inside deferred pipeline-reset blocks
 *     where sleep_ms() could yield to other deferred work and
 *     recursively trigger a reset.
 *
 *   - The post-clock-restart release hold is asynchronous: the mute
 *     pin remains asserted, a deadline is recorded, and the main-loop
 *     dac_hw_mute_tick() deasserts later.  This keeps the SPDIF
 *     in-to-out path draining while the external DAC remains muted.
 *
 *   - Persistence is via flash_storage.c's directory cache.  This
 *     module does not directly touch flash — dac_hw_mute_set_config()
 *     writes the in-RAM dir_cache and triggers a deferred flash
 *     write via flash_storage_mark_directory_dirty() (the same
 *     mechanism master volume mode and include_pins use).
 */

#include "dac_hw_mute.h"

#include "config.h"            /* PIN_CONFIG_* status codes, NUM_SPDIF_INSTANCES */
#include "flash_storage.h"     /* dir_cache, flash_storage_mark_directory_dirty */
#include "notify.h"            /* notify_param_write */
#include "bulk_params.h"       /* WireBulkParams offsets */

#include "hardware/gpio.h"
#include "pico/time.h"         /* time_us_64 */
#include "pico/stdlib.h"       /* tight_loop_contents */

#include <stddef.h>
#include <string.h>

/* External pin-conflict state lives in vendor_commands.c / usb_audio.c.
 * Imported here only for validation; we never write these. */
extern uint8_t output_pins[];
extern uint8_t i2s_bck_pin;
extern uint8_t i2s_mck_pin;
extern bool    i2s_mck_enabled;
extern uint8_t spdif_rx_pin;

/* ----------------------------------------------------------------- */
/* File-scope state                                                   */
/* ----------------------------------------------------------------- */

/* Live config mirror.  Zero-initialized at BSS → enabled = 0 by
 * default; the audio path is byte-for-byte identical to a build with
 * this module removed until dac_hw_mute_init() copies in real values. */
static DacHwMuteConfig s_cfg;

/* Whether the configured GPIO is currently claimed (gpio_init'd +
 * direction set OUT).  Set during init/apply, cleared on release.
 * Lets the assert/release hot path branch on a single bool rather
 * than re-validating cfg.pin against DAC_HW_MUTE_PIN_NONE every
 * call. */
static bool s_pin_claimed = false;

/* Whether mute is currently asserted on the physical/logical output.
 * Used by dac_hw_mute_is_asserted(). */
static bool s_asserted = false;

/* Pipeline-reset lifecycle latch.  Set by dac_hw_mute_assert() and
 * cleared by dac_hw_mute_release().  While true, the pin remains
 * asserted indefinitely regardless of diagnostic-test deadlines. */
static bool s_lifecycle_asserted = false;

/* Asynchronous post-clock-restart release deadline.  0 = no delayed
 * lifecycle release in flight.  When non-zero, the pin remains muted
 * until dac_hw_mute_tick() observes the deadline has passed. */
static uint64_t s_lifecycle_release_us = 0;

/* Asynchronous test-pulse deadline.  0 = no test in flight.  When
 * non-zero, dac_hw_mute_tick() compares against time_us_64() each
 * iteration and releases the test hold once the deadline has passed.
 * Async design (rather than a blocking 1-second busy-wait) is
 * essential — a busy-wait in the test path would block the main
 * loop, starving usb_audio_drain_ring() and producing an audible
 * SPDIF/I²S output underrun on top of the intended DAC mute. */
#define DAC_HW_MUTE_TEST_DURATION_US   (1ull * 1000ull * 1000ull)  /* 1 s */
static uint64_t s_test_release_us = 0;

/* ----------------------------------------------------------------- */
/* Internal helpers                                                   */
/* ----------------------------------------------------------------- */

/* Compute the muted level for the configured polarity. */
static inline bool muted_level(void) {
    return s_cfg.active_low ? false : true;
}

/* Compute the un-muted (idle) level for the configured polarity. */
static inline bool unmuted_level(void) {
    return s_cfg.active_low ? true : false;
}

/* Platform-aware GPIO range check.  Mirrors is_valid_pin() in
 * vendor_commands.c without forcing a circular include. */
static inline bool gpio_in_range(uint8_t pin) {
#if PICO_RP2350
    return pin <= 47;
#else
    return pin <= 29;
#endif
}

/* Return true if `pin` collides with any non-mute pin currently in
 * use by another subsystem.  Mirrors the dependency graph in
 * vendor_commands.c's is_pin_in_use() but excludes mute pins (those
 * collisions are checked separately as part of the duplicate-check
 * inside the cfg's pin field).  Done as a private helper because the public
 * is_pin_in_use() includes mute pins, and using it here would
 * deadlock the validation logic (rejecting a pin we're about to
 * claim because we already claim it). */
static bool collides_with_other_subsystem(uint8_t pin) {
    /* SPDIF / I2S output pins */
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
        if (output_pins[i] == pin) return true;
    }
    /* I2S BCK + LRCLK partner if any slot is I2S */
    extern uint8_t output_types[];
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            if (pin == i2s_bck_pin || pin == (uint8_t)(i2s_bck_pin + 1)) return true;
            break;
        }
    }
    /* MCK pin (only when MCK enabled) */
    if (i2s_mck_enabled && pin == i2s_mck_pin) return true;
    /* SPDIF RX */
    if (pin == spdif_rx_pin) return true;
    return false;
}

/* Drive the configured mute pin to `level`.  Used by refresh_pin_assertion()
 * for both assert (level = muted_level()) and release (level =
 * unmuted_level()).  No-op when the pin isn't claimed. */
static inline void drive_pin(bool level) {
    if (!s_pin_claimed) return;
    gpio_put(s_cfg.pin, level);
}

/* Return a non-zero absolute deadline `delay_us` in the future.  Zero
 * is the "no deadline" sentinel, so avoid returning it on the
 * vanishingly unlikely 64-bit wrap boundary. */
static uint64_t make_deadline_us(uint64_t delay_us) {
    uint64_t deadline = time_us_64() + delay_us;
    return deadline == 0 ? 1 : deadline;
}

/* Apply the OR of every reason the mute pin should remain asserted.
 * This prevents a diagnostic test expiry from unmuting during a
 * pipeline reset, and prevents a pipeline release expiry from ending
 * an in-flight test pulse. */
static void refresh_pin_assertion(void) {
    if (s_cfg.enabled == 0 || !s_pin_claimed) {
        s_asserted = false;
        return;
    }

    bool should_assert = s_lifecycle_asserted ||
                         (s_lifecycle_release_us != 0) ||
                         (s_test_release_us != 0);
    drive_pin(should_assert ? muted_level() : unmuted_level());
    s_asserted = should_assert;
}

/* Busy-wait `wait_ms` milliseconds using time_us_64().  Inserted
 * tight_loop_contents() so the WFE hint can fire if the CPU supports
 * it.  Early-exit on wait_ms == 0 keeps callers branch-free.  Caller
 * is responsible for ensuring the wait runs in a context where
 * busy-waiting is acceptable (i.e. inside the deferred pipeline-reset
 * block where audio is muted regardless). */
static void mute_busy_wait_ms(uint32_t wait_ms) {
    if (wait_ms == 0) return;
    uint64_t deadline = time_us_64() + (uint64_t)wait_ms * 1000ull;
    while ((int64_t)(deadline - time_us_64()) > 0) {
        tight_loop_contents();
    }
}

/* Release the currently-claimed pin back to high-Z input state.
 * Used by init (when re-applying a new config) and by set_config when
 * the feature is being disabled.  Operates on s_cfg.pin only when
 * s_pin_claimed is true; safe to call on a freshly-zeroed module
 * (claim flag false → no-op). */
static void release_claimed_pin(void) {
    if (!s_pin_claimed) return;
    /* gpio_set_dir(IN) leaves the pin as input with no pull
     * (whatever the SDK's gpio_init default is).  Another subsystem
     * can then claim the pin for a peripheral function via
     * gpio_set_function() without conflict. */
    gpio_set_dir(s_cfg.pin, GPIO_IN);
    s_pin_claimed = false;
}

/* Claim the configured pin: gpio_init + set output value + set_dir(OUT).
 * Pre-setting the value before enabling output direction means the pin
 * transitions directly from input to output at the correct (un-muted)
 * level — never momentarily floats during the dir change, which
 * matters in cold-boot scenarios where the DAC may have power before
 * the RP2 finishes init.  Records the claim for later release. */
static void claim_pin(uint8_t pin) {
    gpio_init(pin);
    gpio_put(pin, unmuted_level());
    gpio_set_dir(pin, GPIO_OUT);
    s_pin_claimed = true;
}

/* ----------------------------------------------------------------- */
/* Public API — initialization & configuration                        */
/* ----------------------------------------------------------------- */

void dac_hw_mute_init(const DacHwMuteConfig *cfg) {
    /* Release any previously-claimed pin first so this function is
     * idempotent — set_config() calls back into us after a runtime
     * config change with potentially a different pin. */
    release_claimed_pin();

    /* Copy the new config into our live mirror.  If cfg is NULL or
     * the feature is off, leave s_cfg zeroed (BSS-zero state) so
     * subsequent assert/release calls are zero-cost no-ops. */
    if (!cfg || cfg->enabled == 0) {
        memset(&s_cfg, 0, sizeof(s_cfg));
        s_asserted = false;
        s_lifecycle_asserted = false;
        s_lifecycle_release_us = 0;
        s_test_release_us = 0;
        return;
    }
    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    /* Claim the pin if one is configured.  Validation is the caller's
     * responsibility (set_config() does it before calling us; the boot
     * path trusts whatever was persisted because it was validated when
     * it was written). */
    if (s_cfg.pin != DAC_HW_MUTE_PIN_NONE) {
        claim_pin(s_cfg.pin);
    }
    s_asserted = false;
    s_lifecycle_asserted = false;
    s_lifecycle_release_us = 0;
    s_test_release_us = 0;
}

uint8_t dac_hw_mute_set_config(const DacHwMuteConfig *cfg) {
    if (!cfg) return PIN_CONFIG_INVALID_OUTPUT;

    /* When disabling, just release and zero-out — no other validation
     * needed.  enabled==0 is always accepted. */
    if (cfg->enabled == 0) {
        dac_hw_mute_init(cfg);
        preset_set_dac_hw_mute(cfg);
        notify_param_write(offsetof(WireBulkParams, dac_hw_mute),
                           sizeof(DacHwMuteConfig), cfg);
        return PIN_CONFIG_SUCCESS;
    }

    /* Range checks on scalar fields */
    if (cfg->active_low > 1) return PIN_CONFIG_INVALID_OUTPUT;
    if (cfg->hold_ms < DAC_HW_MUTE_HOLD_MS_MIN ||
        cfg->hold_ms > DAC_HW_MUTE_HOLD_MS_MAX) {
        return PIN_CONFIG_INVALID_OUTPUT;
    }
    if (cfg->release_ms > DAC_HW_MUTE_RELEASE_MS_MAX) {
        return PIN_CONFIG_INVALID_OUTPUT;
    }

    /* Pin validation: in-range + no collision with other subsystems.
     * No duplicate check needed — there is only one pin to claim. */
    if (cfg->pin != DAC_HW_MUTE_PIN_NONE) {
        if (!gpio_in_range(cfg->pin)) return PIN_CONFIG_INVALID_PIN;
        if (collides_with_other_subsystem(cfg->pin)) return PIN_CONFIG_PIN_IN_USE;
    }

    /* All validation passed — apply live, persist to directory,
     * notify other hosts. */
    dac_hw_mute_init(cfg);
    preset_set_dac_hw_mute(cfg);
    notify_param_write(offsetof(WireBulkParams, dac_hw_mute),
                       sizeof(DacHwMuteConfig), cfg);
    return PIN_CONFIG_SUCCESS;
}

void dac_hw_mute_get_config(DacHwMuteConfig *out) {
    if (!out) return;
    memcpy(out, &s_cfg, sizeof(*out));
}

/* ----------------------------------------------------------------- */
/* Pipeline-reset lifecycle hooks                                     */
/* ----------------------------------------------------------------- */

void dac_hw_mute_assert(void) {
    if (s_cfg.enabled == 0 || !s_pin_claimed) return;
    /* Drive pin first, THEN wait — the DAC's ramp clock starts at
     * the edge of the pin transition, so any hold cycles spent
     * before the drive are wasted.  A new lifecycle assert supersedes
     * any previous delayed lifecycle release. */
    s_lifecycle_asserted = true;
    s_lifecycle_release_us = 0;
    refresh_pin_assertion();
    mute_busy_wait_ms(s_cfg.hold_ms);
}

void dac_hw_mute_release(void) {
    if (s_cfg.enabled == 0 || !s_pin_claimed) return;
    if (!s_lifecycle_asserted) return;

    s_lifecycle_asserted = false;
    if (s_cfg.release_ms == 0) {
        s_lifecycle_release_us = 0;
    } else {
        s_lifecycle_release_us =
            make_deadline_us((uint64_t)s_cfg.release_ms * 1000ull);
    }
    refresh_pin_assertion();
}

/* ----------------------------------------------------------------- */
/* Diagnostics                                                        */
/* ----------------------------------------------------------------- */

bool dac_hw_mute_is_asserted(void) {
    return s_asserted;
}

uint8_t dac_hw_mute_test_start(void) {
    /* Feature must be active and have a pin to test.  Otherwise the
     * test would be a no-op. */
    if (s_cfg.enabled == 0 || !s_pin_claimed) return PIN_CONFIG_INVALID_OUTPUT;

    /* If a test is already in flight, don't extend it — just return
     * success.  Re-issuing the command while muted is a host UI
     * mistake; the existing pulse will complete on schedule. */
    if (s_test_release_us != 0) return PIN_CONFIG_SUCCESS;

    /* Assert immediately, record release deadline.  dac_hw_mute_tick()
     * polls the deadline from the main loop. */
    s_test_release_us = make_deadline_us(DAC_HW_MUTE_TEST_DURATION_US);
    refresh_pin_assertion();
    return PIN_CONFIG_SUCCESS;
}

void dac_hw_mute_tick(void) {
    /* Hot path is "no deadline active" — two loads + branches. */
    if (s_test_release_us == 0 && s_lifecycle_release_us == 0) return;

    uint64_t now = time_us_64();
    bool changed = false;

    if (s_test_release_us != 0 &&
        (int64_t)(s_test_release_us - now) <= 0) {
        s_test_release_us = 0;
        changed = true;
    }

    if (s_lifecycle_release_us != 0 &&
        (int64_t)(s_lifecycle_release_us - now) <= 0) {
        s_lifecycle_release_us = 0;
        changed = true;
    }

    if (changed) refresh_pin_assertion();
}

bool dac_hw_mute_owns_pin(uint8_t pin) {
    if (!s_pin_claimed) return false;
    return s_cfg.pin == pin;
}
