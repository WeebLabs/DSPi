/*
 * lg_sound_sync.c — LG Sound Sync (optical) protocol decoder.
 *
 * See lg_sound_sync.h and Documentation/Features/lg_sound_sync_spec.md
 * for the protocol description and design rationale.  This file is a
 * straight implementation of §4 (Architecture) and §5 (Detection State
 * Machine) of that spec.
 *
 * Architecture summary:
 *   - One periodic tick (~50 ms) reads the locked SPDIF channel-status
 *     bytes and looks for a fixed 5-nibble LG signature.
 *   - Asymmetric hysteresis (3-poll rise, 10-poll fall) absorbs lock
 *     transients and bit errors without dropping vol_mul mid-listening.
 *   - On PRESENT, the decoded volume/mute is funneled through
 *     apply_vol_index_to_audio() — the same helper used by the USB host
 *     volume path — so loudness compensation tracks LG-driven changes.
 *   - On ABSENT (transition from PRESENT), vol_mul is restored to the
 *     value it would have had under USB-host control: the cached
 *     audio_state.volume mapped through the same arithmetic that
 *     audio_set_volume() uses.
 *
 * Concurrency:
 *   - All public entry points run on the main thread.  vol_mul writes
 *     are not torn-read-safe by themselves on RP2040 (16-bit field,
 *     single store per architecture), but the audio pipeline only
 *     samples vol_mul at packet boundaries through a per-packet ramp,
 *     so a brief inconsistency cannot produce a discontinuity.
 *   - Status reads (lg_sound_sync_get_status) snapshot via a small IRQ-
 *     disabled critical section so the host sees a coherent struct.
 */

#include "lg_sound_sync.h"

#include "audio_input.h"      /* active_input_source */
#include "bulk_params.h"      /* WireBulkParams offsets for notify */
#include "config.h"           /* CENTER_VOLUME_INDEX, INPUT_SOURCE_* */
#include "notify.h"           /* notify_param_write */
#include "spdif_input.h"      /* spdif_input_get_state, spdif_input_get_channel_status */
#include "usb_audio.h"        /* apply_vol_index_to_audio, audio_state, CENTER_VOLUME_INDEX */

#include "hardware/sync.h"    /* save_and_disable_interrupts */
#include "pico/time.h"        /* time_us_64 */

#include <stddef.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */

/* Channel-status poll cadence.  The library populates `c_bits` once per
 * 192-frame block (~32 ms at 48 kHz), so polling much faster than 50 ms
 * just re-reads identical bytes.  50 ms × 3-poll PRESENT threshold =
 * ~150 ms latency from "TV starts Sound Sync" to "DSPi takes over",
 * which is below human perceptual threshold for "instant". */
#define LG_POLL_INTERVAL_US    (50ULL * 1000ULL)

/* Asymmetric hysteresis — see spec §5.1.
 *   Rise (PRESENT):  3 polls × 50 ms = 150 ms.  Fast enough that the
 *                    user gets responsive control as soon as Sound Sync
 *                    starts.
 *   Fall (ABSENT):  10 polls × 50 ms = 500 ms.  Slow enough that a
 *                   single corrupted CS block or brief signal hiccup
 *                   doesn't snap vol_mul back to the cached USB value
 *                   mid-listening. */
#define LG_PRESENT_THRESHOLD     3u
#define LG_ABSENT_THRESHOLD     10u

/* Sentinel for "no LG volume has been observed since boot/init".  Used
 * in LgSoundSyncStatus.volume.  0xFF cannot collide with a real LG vol
 * (range 0..100), so it is unambiguous on the wire. */
#define LG_VOLUME_NEVER_SEEN    0xFFu

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

/* User-facing enable flag.  Set by lg_sound_sync_set_enabled() and read
 * lock-free by the tick.  No volatile-required because all writers and
 * readers are main-thread; the `volatile` is defensive in case a future
 * Core 1 reader is added. */
static volatile bool s_enabled = (LG_SOUND_SYNC_DEFAULT_ENABLED != 0);

/* Detection state.  Mutated only by the tick (main thread). */
static bool     s_present        = false;
static uint8_t  s_present_streak = 0;     /* consecutive polls with signature   */
static uint8_t  s_absent_streak  = 0;     /* consecutive polls without signature*/
static uint64_t s_last_poll_us   = 0;     /* throttle reference                 */

/* Last decoded values.  Held even after demotion to ABSENT — the host
 * UI gets to display "last known LG vol = 42, mute=on" rather than
 * blanks when Sound Sync drops out. */
static uint8_t  s_last_volume = LG_VOLUME_NEVER_SEEN;
static bool     s_last_muted  = false;

/* Last vol_index actually written through apply_vol_index_to_audio().
 * Used to coalesce no-op writes — the LG poll fires every 50 ms but
 * the user only changes volume on remote presses, so most polls
 * decode the same value as the last.  The pipeline ramp handles real
 * changes click-free; the coalesce just avoids redundant pointer
 * loads in loudness coefficient swap. */
static int16_t  s_last_applied_vol_index = -1;   /* -1 = none yet */

/* ------------------------------------------------------------------ */
/* Pure helpers                                                        */
/* ------------------------------------------------------------------ */

/* Map LG vol [0..100] → vol_index [0..CENTER_VOLUME_INDEX] proportionally
 * with round-half-up division.  Endpoints land cleanly:
 *   lg=0   → vol_index=0   (silent  — db_to_vol[0] = 0)
 *   lg=50  → vol_index=30  (~ -30 dB)
 *   lg=99  → vol_index=59  (~ -1 dB)   — round-half-up: 99×60+50 = 5990, /100 = 59
 *   lg=100 → vol_index=60  (unity gain — db_to_vol[60] = 0x8000)
 * Intermediate steps approximate 1 dB each (matches db_to_vol[]'s
 * ~1-dB-per-step shape), which matches what LG TVs themselves do
 * internally and what users expect from a TV remote. */
static inline uint8_t lg_vol_to_vol_index(uint8_t lg_vol) {
    if (lg_vol > 100u) lg_vol = 100u;
    /* +50 implements round-half-up: x.5 rounds up, x.49 rounds down.
     * Only lg=100 reaches vol_index=60; lg=99 lands on 59 (≈ -1 dB). */
    uint32_t scaled = (uint32_t)lg_vol * (uint32_t)CENTER_VOLUME_INDEX + 50u;
    uint8_t  idx    = (uint8_t)(scaled / 100u);
    if (idx > CENTER_VOLUME_INDEX) idx = CENTER_VOLUME_INDEX;
    return idx;
}

/* Decode the LG signature from a 24-byte channel-status array.  Returns
 * true iff bytes 16/17/18 carry the fixed F-04-8A pattern.  See spec §2.3
 * for the bit-level derivation; bottom line is that this 5-nibble pattern
 * is only present when an LG TV is broadcasting Sound Sync, has not been
 * observed on any non-LG source, and is robust against single-bit flips
 * because the hysteresis state machine requires LG_PRESENT_THRESHOLD
 * consecutive matches before declaring presence. */
static inline bool lg_signature_present(const uint8_t cs[24]) {
    return ((cs[16] & 0x0Fu) == 0x0Fu)
        && (cs[17] == 0x04u)
        && (cs[18] == 0x8Au);
}

/* Decode volume + mute from CS bytes 15 and 16.
 * vol_byte = (cs[15] low nibble) << 4 | (cs[16] high nibble), where bit 7
 * is the mute flag and bits 6:0 are the volume 0..100.  See spec §2.4
 * for the empirical verification table. */
static inline void lg_decode_volume(const uint8_t cs[24],
                                     uint8_t *out_vol, bool *out_muted) {
    uint8_t vol_byte = (uint8_t)(((cs[15] & 0x0Fu) << 4) |
                                 ((cs[16] & 0xF0u) >> 4));
    *out_muted = (vol_byte & 0x80u) != 0u;
    uint8_t v = (uint8_t)(vol_byte & 0x7Fu);
    if (v > 100u) v = 100u;   /* defensive — values >100 unobserved */
    *out_vol = v;
}

/* ------------------------------------------------------------------ */
/* Apply path                                                          */
/* ------------------------------------------------------------------ */

/* Recompute the vol_index that the cached USB host volume would map to
 * and apply it.  Matches the arithmetic in audio_set_volume() exactly
 * (kept in sync deliberately — see comment at the source).
 *
 * Used when transitioning from PRESENT to ABSENT to "hand vol_mul back"
 * to whatever the OS slider last set.  We can't just call
 * audio_set_volume(audio_state.volume) because that early-returns on
 * non-USB input — exactly what we don't want here. */
static void restore_host_volume_to_audio_path(void) {
    int16_t cached = (int16_t)audio_state.volume;
    /* Same shift as audio_set_volume: dB 0.5-resolution signed →
     * unsigned 8.8 with the slider's zero point at the centre. */
    int32_t v = (int32_t)cached + (int32_t)CENTER_VOLUME_INDEX * 256;
    if (v < 0) v = 0;
    int32_t hi = ((int32_t)CENTER_VOLUME_INDEX + 1) * 256 - 1;
    if (v > hi) v = hi;
    apply_vol_index_to_audio((uint8_t)((uint32_t)v >> 8u));
}

/* Apply LG-decoded state to the audio path.  Called only while
 * s_present is true.  Click-free transitions are guaranteed by the
 * audio pipeline's per-packet ramp — this function just writes the
 * target. */
static void apply_lg_state(uint8_t lg_vol, bool lg_mute) {
    uint8_t vol_index = lg_vol_to_vol_index(lg_vol);

    if (lg_mute) {
        /* Hard mute: drive vol_mul to zero directly.  The pipeline ramp
         * will glide from the current target down to 0 over the next
         * packet (~1 ms at 48 kHz) — click-free.  Loudness coefficients
         * are intentionally NOT zeroed: holding them at the previous
         * vol_index means unmuting from this state has no one-packet
         * coefficient-mismatch transient.  (Audio is silent during mute
         * anyway, so the held coefficients are not heard.)
         *
         * Sentinel `-1` forces the next non-mute call into the
         * "vol_index < 0" branch below, guaranteeing re-apply even when
         * the post-mute vol_index matches the pre-mute one.  Without
         * this, an LG mute → unmute at the same TV vol would coalesce
         * away the apply and leave vol_mul stuck at 0. */
        audio_state.vol_mul = 0;
        s_last_applied_vol_index = -1;
    } else if (vol_index != (uint8_t)s_last_applied_vol_index ||
               s_last_applied_vol_index < 0) {
        /* Coalesce: the LG poll re-reads the same byte while no remote
         * button is pressed.  apply_vol_index_to_audio is cheap (one
         * array load + a pointer swap) but skipping the no-op write
         * keeps the pipeline's prev/target ramp from spuriously re-
         * computing across no-change packets. */
        apply_vol_index_to_audio(vol_index);
        s_last_applied_vol_index = (int16_t)vol_index;
    }
}

/* Push a PARAM_CHANGED notification for any of `volume`, `muted`,
 * `present` that has changed since the previous tick.  Field-granular
 * because the notify v2 protocol coalesces by (offset, size) — fine-
 * grained UI updates without packet-count cost. */
static void notify_runtime_field(uint16_t field_offset, uint8_t value) {
    notify_param_write(field_offset, sizeof(uint8_t), &value);
}

/* Compute WireBulkParams field offsets for the runtime fields without
 * pulling the layout into this file.  bulk_params.h declares
 * WireBulkParams; lg_sound_sync field is added in Phase 4.  These
 * helpers are defined inline so a typo in offsetof(...) at any one
 * call site produces a single compile error rather than diffuse
 * runtime drift. */
static inline uint16_t off_enabled(void) {
    return (uint16_t)offsetof(WireBulkParams, lg_sound_sync.enabled);
}
static inline uint16_t off_present(void) {
    return (uint16_t)offsetof(WireBulkParams, lg_sound_sync.present);
}
static inline uint16_t off_volume(void) {
    return (uint16_t)offsetof(WireBulkParams, lg_sound_sync.volume);
}
static inline uint16_t off_muted(void) {
    return (uint16_t)offsetof(WireBulkParams, lg_sound_sync.muted);
}

/* ------------------------------------------------------------------ */
/* State transitions                                                   */
/* ------------------------------------------------------------------ */

static void enter_present(uint8_t lg_vol, bool lg_mute) {
    bool was_present = s_present;
    s_present = true;
    apply_lg_state(lg_vol, lg_mute);

    /* Order of notifications: volume + muted first, then present last.
     * This way a host UI that subscribes only to `present` and re-reads
     * all fields on the rising edge sees consistent values.  Hosts that
     * subscribe to individual fields naturally see them all updated by
     * the time `present=1` arrives. */
    if (lg_vol != s_last_volume) {
        s_last_volume = lg_vol;
        notify_runtime_field(off_volume(), lg_vol);
    }
    if (lg_mute != s_last_muted) {
        s_last_muted = lg_mute;
        notify_runtime_field(off_muted(), lg_mute ? 1u : 0u);
    }
    if (!was_present) {
        notify_runtime_field(off_present(), 1u);
    }
}

/* Demote PRESENT → ABSENT, optionally restoring the cached USB host
 * volume.  We don't always restore: when the input source has just
 * switched away from SPDIF, the input-source-change handler in main.c
 * has its own thaw path (audio_set_volume(audio_state.volume)) and
 * calling our restore here would either duplicate that work or fight
 * against it. */
static void leave_present(bool restore_host_vol) {
    if (!s_present) return;
    s_present = false;
    s_present_streak = 0;
    s_last_applied_vol_index = -1;
    if (restore_host_vol) {
        restore_host_volume_to_audio_path();
    }
    notify_runtime_field(off_present(), 0u);
    /* Note: we do NOT clear s_last_volume / s_last_muted on demote.
     * They reflect the last value the LG TV broadcast — useful info
     * for a UI that wants to show "TV last reported vol=42 (not
     * currently broadcasting)".  They are reset at lg_sound_sync_init()
     * (boot, factory reset). */
}

/* ------------------------------------------------------------------ */
/* Tick                                                                */
/* ------------------------------------------------------------------ */

void lg_sound_sync_tick(void) {
    /* Cheap exit path 1: feature disabled.  If we were previously
     * driving vol_mul, hand it back to the cached host volume now.
     * No throttle on this branch — the user expects the toggle to
     * take effect immediately, not 50 ms later. */
    if (!s_enabled) {
        leave_present(/*restore_host_vol=*/true);
        return;
    }

    /* Cheap exit path 2: not on SPDIF.  No restore here — the input-
     * source-change handler does its own thaw on USB transitions. */
    if (active_input_source != INPUT_SOURCE_SPDIF) {
        leave_present(/*restore_host_vol=*/false);
        return;
    }

    /* Throttle to LG_POLL_INTERVAL_US.  time_us_64() rather than a
     * counter so the cadence is robust against variable main-loop
     * pacing (heavy USB activity vs. idle). */
    uint64_t now_us = time_us_64();
    if ((now_us - s_last_poll_us) < LG_POLL_INTERVAL_US) return;
    s_last_poll_us = now_us;

    /* SPDIF must be locked for channel-status to be meaningful.
     * Treat any non-locked state as "no signature" so the absent
     * streak can build up to the demote threshold. */
    if (spdif_input_get_state() != SPDIF_INPUT_LOCKED) {
        s_present_streak = 0;
        if (s_absent_streak < 0xFFu) s_absent_streak++;
        if (s_present && s_absent_streak >= LG_ABSENT_THRESHOLD) {
            leave_present(/*restore_host_vol=*/true);
        }
        return;
    }

    /* Read the channel-status snapshot.  The library serializes the
     * read against its own DMA IRQ internally, so we do not need to
     * disable interrupts here. */
    uint8_t cs[24];
    spdif_input_get_channel_status(cs);

    if (lg_signature_present(cs)) {
        s_absent_streak = 0;
        if (s_present_streak < 0xFFu) s_present_streak++;

        /* Apply on every signature-positive poll once we're either past
         * the rising-edge threshold OR already present.  The "already
         * present" branch covers the case where streaks were reset
         * (e.g. by lg_sound_sync_on_preset_loaded()) but s_present
         * itself was preserved — without it we'd freeze vol_mul at the
         * last-applied value for ~150 ms after every preset load.
         * enter_present() is idempotent, so calling it on every poll is
         * cheap (one byte compare on volume + one bit compare on mute,
         * with the notify_param_write coalesce stage filtering the
         * identical-value case at the ring level). */
        if (s_present || s_present_streak >= LG_PRESENT_THRESHOLD) {
            uint8_t lg_vol; bool lg_mute;
            lg_decode_volume(cs, &lg_vol, &lg_mute);
            enter_present(lg_vol, lg_mute);
        }
    } else {
        s_present_streak = 0;
        if (s_absent_streak < 0xFFu) s_absent_streak++;
        if (s_present && s_absent_streak >= LG_ABSENT_THRESHOLD) {
            leave_present(/*restore_host_vol=*/true);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void lg_sound_sync_init(void) {
    /* Reset all RAM state.  Live `s_enabled` keeps its current value —
     * preset_boot_load() writes through lg_sound_sync_set_enabled() to
     * load the persisted flag, and that callsite controls the source
     * tag for the resulting notification. */
    s_present = false;
    s_present_streak = 0;
    s_absent_streak  = 0;
    s_last_poll_us   = 0;
    s_last_volume    = LG_VOLUME_NEVER_SEEN;
    s_last_muted     = false;
    s_last_applied_vol_index = -1;
}

void lg_sound_sync_set_enabled(bool en) {
    bool prev = s_enabled;
    s_enabled = en;
    if (prev == en) return;

    /* Push notification for the enable bit.  Source tag is whatever
     * the caller set on `notify_set_source()` — vendor command
     * dispatcher tags as PARAM_SRC_HOST_SET; preset apply path tags
     * as PARAM_SRC_PRESET. */
    uint8_t v = en ? 1u : 0u;
    notify_param_write(off_enabled(), sizeof(uint8_t), &v);

    /* If we just disabled the feature while presenting, hand vol_mul
     * back to the cached host volume immediately.  Don't wait for the
     * tick — the user expects the toggle to take audible effect now. */
    if (!en && s_present) {
        leave_present(/*restore_host_vol=*/true);
    } else if (en) {
        /* Just enabled.  Reset streaks so the next tick starts a fresh
         * acquisition — a stale present_streak from a previous enable
         * would otherwise cause a near-instant PRESENT declaration if
         * the signature happens to be on the wire.
         *
         * Also reset s_last_applied_vol_index to -1.  Today this is
         * already covered by the leave_present() path on the previous
         * disable transition, so the slot is always -1 on re-enable.
         * Resetting unconditionally here is defensive: it removes the
         * dependence on that prior-call invariant and keeps a future
         * "force re-arm" entry point (one that doesn't pass through
         * leave_present) from inheriting a stale apply-cache. */
        s_present_streak = 0;
        s_absent_streak  = 0;
        s_last_poll_us   = 0;   /* poll on next tick */
        s_last_applied_vol_index = -1;
    }
}

bool lg_sound_sync_get_enabled(void) {
    return s_enabled;
}

void lg_sound_sync_get_status(LgSoundSyncStatus *out) {
    if (!out) return;
    /* Brief IRQ disable for a coherent snapshot.  The fields are
     * 1-byte each and reads are individually atomic, but a host that
     * receives a torn struct (e.g., enabled=1, present=0 because
     * present was about to go 1) could mis-render the UI for one
     * frame.  Cost is negligible — four byte loads. */
    uint32_t flags = save_and_disable_interrupts();
    out->enabled = s_enabled ? 1u : 0u;
    out->present = s_present ? 1u : 0u;
    out->volume  = s_last_volume;
    out->muted   = s_last_muted ? 1u : 0u;
    memset(out->reserved, 0, sizeof(out->reserved));
    restore_interrupts(flags);
}

void lg_sound_sync_on_input_source_change(uint8_t new_source) {
    /* If the new source is not SPDIF, demote without restoring vol_mul
     * — the input-source-change handler already does the thaw on its
     * own USB→ branch.  Calling restore here would produce a double-
     * write and (if the handler thaws to a different value than our
     * cached read) a one-packet glitch. */
    if (new_source != INPUT_SOURCE_SPDIF) {
        leave_present(/*restore_host_vol=*/false);
    } else {
        /* USB → SPDIF: re-arm.  Streaks zero so we don't prematurely
         * declare PRESENT from stale state. */
        s_present_streak = 0;
        s_absent_streak  = 0;
        s_last_poll_us   = 0;
    }
}

void lg_sound_sync_on_preset_loaded(void) {
    /* A preset load may have changed the `enabled` bit.  Reset the
     * detection streaks so we re-evaluate from CS — stored volume/
     * muted from the prior preset are non-authoritative because the
     * TV may have changed state during the transition. */
    s_present_streak = 0;
    s_absent_streak  = 0;
    s_last_poll_us   = 0;
    s_last_applied_vol_index = -1;

    /* If the new preset disabled the feature while we were presenting,
     * demote and restore.  apply_factory_defaults() / preset_load()
     * both run inside the preset_loading mute envelope, so no audible
     * transition fires here regardless. */
    if (!s_enabled && s_present) {
        leave_present(/*restore_host_vol=*/true);
    }
}

void lg_sound_sync_invalidate_apply_cache(void) {
    /* See header for why this exists.  Single store, no IRQ disable
     * needed — the only reader is the next apply_lg_state() call from
     * the polling tick, which runs in the same main-thread context as
     * any caller of this hook.  The audio pipeline never reads
     * s_last_applied_vol_index. */
    s_last_applied_vol_index = -1;
}
