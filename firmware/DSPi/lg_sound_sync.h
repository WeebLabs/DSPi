/*
 * lg_sound_sync.h — LG Sound Sync (optical) protocol decoder.
 *
 * LG TVs broadcast their volume and mute state in specific channel-status
 * bytes of the SPDIF (TOSLINK) stream they output.  When this feature is
 * enabled and an LG-Sound-Sync-marked SPDIF source is locked, this module
 * drives the device's host volume + loudness coefficients from the LG-
 * decoded values, so the TV remote acts as a volume control for DSPi.
 *
 * The protocol is observation-only — there is no back-channel; we just
 * watch the channel-status for a fixed signature and decode the volume
 * out of nibbles 30..31 + 32..35 of the 192-bit channel-status field.
 *
 * Why is this here and not in spdif_input.c?
 *   - spdif_input.c owns the RX hardware lifecycle (start/stop, FIFO
 *     drain, clock servo, lock state).  Coupling control-side decoding
 *     into it would make it harder to reason about either piece.
 *   - Different external sources can drive the same logical "TV remote
 *     controls device volume" abstraction (LG today, Samsung/Sony in
 *     future).  Keeping this in its own module lets us add per-protocol
 *     decoders without touching the audio path.
 *
 * Why does it drive host volume rather than master volume?
 *   - Loudness compensation tracks the *raw user-perceived* volume index
 *     (the same one db_to_vol[] indexes).  Master volume is a separate
 *     ceiling that does not feed loudness.  If LG vol drove master
 *     instead, lowering the TV vol would reduce SPL but loudness would
 *     keep its EQ at the original reference, over-compensating bass and
 *     treble.  Driving host volume keeps the SPL/loudness loop coherent.
 *
 * See Documentation/Features/lg_sound_sync_spec.md for the full design,
 * protocol details, and edge-case matrix.
 */

#ifndef LG_SOUND_SYNC_H
#define LG_SOUND_SYNC_H

#include <stdint.h>
#include <stdbool.h>

/* Default state for a fresh slot or pre-V14 slot read.  Defined here so
 * lg_sound_sync.c, flash_storage.c, and bulk_params.c all share one
 * source of truth.  This build defaults the feature ON, so LG TV users
 * get Sound Sync without any setup; a pre-V14 user preset loaded after
 * firmware update will also default to ON.  Disabling per-preset still
 * works via the vendor command + REQ_SAVE_PRESET. */
#define LG_SOUND_SYNC_DEFAULT_ENABLED  1

/*
 * Aggregate runtime status returned by REQ_GET_LG_SOUND_SYNC_STATUS and
 * mirrored in WireBulkParams.lg_sound_sync (when the wire format is
 * extended in Phase 4 of the implementation plan).
 *
 * Layout is fixed at 16 bytes for forward compatibility — extending the
 * struct in the future just shrinks `reserved`, leaving offsets stable.
 */
typedef struct __attribute__((packed)) {
    uint8_t enabled;        /* User-controlled gate (0 = off, 1 = on).
                             * Per-preset; persists with REQ_SAVE_PRESET. */
    uint8_t present;        /* Detection state (0 = absent, 1 = present).
                             * Read-only — set by the detection state
                             * machine.  Goes 1 only after the signature
                             * has been seen for LG_PRESENT_THRESHOLD
                             * consecutive polls. */
    uint8_t volume;         /* Last decoded LG volume (0..100).  0xFF
                             * sentinel means "never decoded since boot".
                             * Meaningful only when `present` is 1; held
                             * (not cleared) when `present` falls back. */
    uint8_t muted;          /* Last decoded LG mute (0/1).  Meaningful
                             * only when `present` is 1; held when
                             * `present` falls. */
    uint8_t reserved[12];   /* Pad to 16 bytes.  Future fields would go
                             * here without changing sizeof or any
                             * existing field's offset. */
} LgSoundSyncStatus;

/* One-time initialisation.  Resets all RAM state to "feature off,
 * absent, no volume seen".  Does NOT load the persisted enable flag —
 * that is loaded by flash_storage.c during preset_boot_load() (the
 * per-preset apply path), which writes through lg_sound_sync_set_enabled()
 * with the source tag set to PARAM_SRC_PRESET. */
void lg_sound_sync_init(void);

/* Main-loop tick.  Cheap when the feature is disabled or SPDIF is not
 * the active input.  Internally throttled to one channel-status poll
 * every LG_POLL_INTERVAL_MS regardless of how often it is called.  Safe
 * to call every iteration of the main loop. */
void lg_sound_sync_tick(void);

/* Set the user-facing enable flag.  Updates live state immediately and
 * (when transitioning enabled→disabled while present) demotes to absent
 * + restores host volume.  Pushes a PARAM_CHANGED notification on the
 * `enabled` field of WireLgSoundSync.  Does NOT write flash — flash
 * persistence happens on REQ_SAVE_PRESET. */
void lg_sound_sync_set_enabled(bool en);

/* Read the live enable flag. */
bool lg_sound_sync_get_enabled(void);

/* Snapshot the full runtime status into `out`.  16 bytes.  Safe to call
 * from any main-thread context; reads volatile state with a brief IRQ
 * disable to avoid torn reads of multi-byte fields. */
void lg_sound_sync_get_status(LgSoundSyncStatus *out);

/* Hook called by main.c after committing an input-source change.  Lets
 * the module demote to absent without touching vol_mul (the input-switch
 * handler itself thaws the cached host volume on USB transitions). */
void lg_sound_sync_on_input_source_change(uint8_t new_source);

/* Hook called by flash_storage.c immediately after a preset load
 * commits its slot data to live state.  Clears the detection streaks so
 * the next tick re-evaluates the signature against the freshly-loaded
 * `enabled` flag.  Does not push notifications — notify_push_preset_loaded()
 * + bulk_invalidated already drive the host to re-read all fields. */
void lg_sound_sync_on_preset_loaded(void);

#endif /* LG_SOUND_SYNC_H */
