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

/* Last vol_index actually written through update_user_volume().
 * Used to coalesce no-op writes — the LG poll fires every 50 ms but
 * the user only changes volume on remote presses, so most polls decode
 * the same value as the last.  Coalescing avoids redundant
 * audio_state.volume writes and the per-write user_volume notify
 * (which would otherwise fire 20× per second of TV silence). */
static int16_t  s_last_applied_vol_index = -1;   /* -1 = none yet */

/* Tracks whether the *current* user_mute state was imposed by LG (vs.
 * set manually by the user via REQ_SET_USER_MUTE).  On leave_present
 * we clear user_mute only if LG owned it; otherwise the user's
 * deliberate mute is preserved across the demote.  Set true when LG
 * drives user_mute false→true, cleared when LG drives true→false or
 * when leave_present clears the LG-imposed mute. */
static bool     s_lg_imposed_mute = false;

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

/* LG model layouts.
 *
 * HiFiBerry's reference analysis (see spec §2.3) places the Sound Sync
 * signature at CS bytes 16/17/18 with the volume nibbles spanning
 * cs[15] low + cs[16] high.  This matches newer LG TVs.
 *
 * The 2017 LG B7 (and likely contemporaries) lay the same data at
 * positions mirrored around the middle of the 24-byte block:
 *   byte 18 ↔ byte 5,  byte 17 ↔ byte 6,  byte 16 ↔ byte 7,  byte 15 ↔ byte 8.
 * The nibble layout *within* each byte is unchanged; only the byte
 * positions are mirrored.  Empirically verified at TV vol = 3 and 26
 * (see spec §2.6).
 *
 * lg_match_layout() tries each layout in order and returns the first
 * match (or NULL).  The detection state machine treats "any layout
 * matched" as "signature present"; the matched layout is plumbed
 * through to the volume decoder so the right offsets are used.  The
 * matcher is cheap (3 byte comparisons per layout, short-circuit) and
 * runs only once per LG_POLL_INTERVAL_US, so the cost of supporting
 * multiple layouts is negligible. */
typedef struct {
    uint8_t sig_a;   /* index where (byte & 0x0F) == 0x0F */
    uint8_t sig_b;   /* index where byte == 0x04 */
    uint8_t sig_c;   /* index where byte == 0x8A */
    uint8_t vol_hi;  /* index whose low nibble is the volume byte's high nibble */
    uint8_t vol_lo;  /* index whose high nibble is the volume byte's low nibble */
} LgLayout;

/* HiFiBerry layout (newer LG models). */
static const LgLayout LG_LAYOUT_NEW = {
    .sig_a = 16, .sig_b = 17, .sig_c = 18,
    .vol_hi = 15, .vol_lo = 16,
};

/* B7-era layout (mirror around byte 11.5). */
static const LgLayout LG_LAYOUT_B7 = {
    .sig_a = 7, .sig_b = 6, .sig_c = 5,
    .vol_hi = 8, .vol_lo = 7,
};

static const LgLayout *const LG_LAYOUTS[] = { &LG_LAYOUT_NEW, &LG_LAYOUT_B7 };
#define LG_LAYOUT_COUNT  (sizeof(LG_LAYOUTS) / sizeof(LG_LAYOUTS[0]))

/* Return the matching layout (NULL if none).  Order is "newer first" so
 * a TV that happens to populate both layouts (none observed in the wild)
 * would prefer the layout HiFiBerry's analysis was based on. */
static inline const LgLayout *lg_match_layout(const uint8_t cs[24]) {
    for (size_t i = 0; i < LG_LAYOUT_COUNT; i++) {
        const LgLayout *L = LG_LAYOUTS[i];
        if (((cs[L->sig_a] & 0x0Fu) == 0x0Fu)
            && (cs[L->sig_b] == 0x04u)
            && (cs[L->sig_c] == 0x8Au)) {
            return L;
        }
    }
    return NULL;
}

/* Decode volume + mute using the offsets from the matched layout.
 * vol_byte = (cs[vol_hi] low nibble) << 4 | (cs[vol_lo] high nibble),
 * where bit 7 is the mute flag and bits 6:0 are the volume 0..100. */
static inline void lg_decode_volume(const uint8_t cs[24], const LgLayout *L,
                                     uint8_t *out_vol, bool *out_muted) {
    uint8_t vol_byte = (uint8_t)(((cs[L->vol_hi] & 0x0Fu) << 4) |
                                 ((cs[L->vol_lo] & 0xF0u) >> 4));
    *out_muted = (vol_byte & 0x80u) != 0u;
    uint8_t v = (uint8_t)(vol_byte & 0x7Fu);
    if (v > 100u) v = 100u;   /* defensive — values >100 unobserved */
    *out_vol = v;
}

/* ------------------------------------------------------------------ */
/* Apply path                                                          */
/* ------------------------------------------------------------------ */

/* Apply LG-decoded state to the audio path.  Called only while
 * s_present is true.  Drives BOTH the user-facing volume
 * (audio_state.volume + user_volume notify) AND the audio path
 * (vol_mul + loudness coeffs) through update_user_volume() — a single
 * funnel for every owner of the user-perceived volume.  Mute drives
 * user_mute (the vendor mute, gates audio without touching volume).
 *
 * This is "option 2" from the LG-volume design discussion: LG drives
 * the host's user-volume slider directly so the UI single-widget
 * display tracks TV remote presses.  Trade-off accepted: when SPDIF
 * input switches back to USB or LG demotes, audio_state.volume stays
 * wherever LG left it (no thaw cache).  The OS may re-issue UAC1
 * SET_CUR with its own remembered per-device volume on enumeration or
 * default-device change events; for DSPi-internal input switches the
 * user's slider position simply picks up from LG's last value.
 *
 * Click-free transitions are guaranteed by the audio pipeline's per-
 * packet ramp — this function just writes the target. */
static void apply_lg_state(uint8_t lg_vol, bool lg_mute) {
    uint8_t vol_index = lg_vol_to_vol_index(lg_vol);

    /* Volume.  Coalesce no-op writes — at 50 ms poll cadence with no
     * remote button presses, every poll decodes the same value.  Each
     * non-coalesced apply pushes a user_volume notify; without
     * coalescing we'd flood the host with 20 redundant notifies/sec.
     *
     * update_user_volume() invalidates s_last_applied_vol_index to -1
     * (so external vendor writes can break our cache).  We immediately
     * re-establish the cache to the value we just wrote so subsequent
     * matching polls coalesce. */
    if (vol_index != (uint8_t)s_last_applied_vol_index ||
        s_last_applied_vol_index < 0) {
        float db = (float)((int32_t)vol_index - (int32_t)CENTER_VOLUME_INDEX);
        update_user_volume(db);
        s_last_applied_vol_index = (int16_t)vol_index;
    }

    /* Mute.  Drives user_mute (the vendor mute, OR'd with audio_state.mute
     * in the audio pipeline).  Tracks `s_lg_imposed_mute` so leave_present
     * can clear LG-imposed mute on demote without clobbering a mute the
     * user set manually before LG took over.  When LG is present LG owns
     * mute on the SPDIF path: an LG-driven unmute clears whatever the
     * mute source was, mirroring how LG-driven volume overrides any
     * vendor SET_USER_VOLUME. */
    if (lg_mute != user_mute) {
        user_mute = lg_mute;
        s_lg_imposed_mute = lg_mute;
        uint8_t v = lg_mute ? 1u : 0u;
        notify_param_write(offsetof(WireBulkParams, user_volume.user_mute),
                           sizeof(uint8_t), &v);
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

/* ------------------------------------------------------------------ */
/* State transitions                                                   */
/* ------------------------------------------------------------------ */

static void enter_present(uint8_t lg_vol, bool lg_mute) {
    bool was_present = s_present;
    s_present = true;
    apply_lg_state(lg_vol, lg_mute);

    /* Track last seen LG vol/mute for REQ_GET_LG_SOUND_SYNC_STATUS, but
     * do not push per-field notifies.  The corresponding user-volume /
     * user-mute changes are notified by update_user_volume() inside
     * apply_lg_state() — pushing additional lg_sound_sync.volume and
     * lg_sound_sync.muted notifies would duplicate the same event in
     * different units, doubling wire chatter during TV remote presses.
     * Hosts that want the raw 0..100 LG vol can poll the status struct. */
    s_last_volume = lg_vol;
    s_last_muted  = lg_mute;

    if (!was_present) {
        notify_runtime_field(off_present(), 1u);
    }
}

/* Demote PRESENT → ABSENT.  Per the option-2 design, we do NOT
 * restore the previously-cached USB host volume — audio_state.volume
 * stays wherever LG last set it.  The OS may re-issue UAC1 SET_CUR
 * with its remembered per-device volume on enumeration or default-
 * device-change; for DSPi-internal input switches the user's slider
 * position simply picks up from LG's last value.  The
 * `restore_host_vol` parameter is retained on the signature for
 * source-compat with existing callers but is now ignored.
 *
 * We DO clear LG-imposed mute so the user isn't stuck silent when the
 * TV stops broadcasting Sound Sync (e.g., TV powers off, TOSLINK
 * unplugged).  s_lg_imposed_mute distinguishes mutes LG drove from
 * mutes the user set manually via REQ_SET_USER_MUTE — the latter are
 * preserved across the demote. */
static void leave_present(bool restore_host_vol) {
    (void)restore_host_vol;  /* vestigial — see comment above */
    if (!s_present) return;
    s_present = false;
    s_present_streak = 0;
    s_last_applied_vol_index = -1;

    if (s_lg_imposed_mute && user_mute) {
        user_mute = false;
        s_lg_imposed_mute = false;
        uint8_t v = 0u;
        notify_param_write(offsetof(WireBulkParams, user_volume.user_mute),
                           sizeof(uint8_t), &v);
    } else {
        s_lg_imposed_mute = false;
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

    const LgLayout *layout = lg_match_layout(cs);
    if (layout != NULL) {
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
            lg_decode_volume(cs, layout, &lg_vol, &lg_mute);
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
    s_lg_imposed_mute = false;
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
