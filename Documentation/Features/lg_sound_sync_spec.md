# LG Sound Sync (Optical) — Specification & Implementation Plan

*Status: design / pre-implementation — 2026-05-08*

## 1. Overview

LG Sound Sync is a one-way control side-channel that LG televisions multiplex onto their SPDIF (TOSLINK) optical output. It rides on top of the regular PCM audio stream by stuffing TV-volume and TV-mute information into specific *channel-status* bits of the IEC 60958 frame. The receiving device (a sound bar, AVR, or — in our case — DSPi) decodes those bits and applies the TV's volume to its own output.

This document specifies how DSPi will detect and act on Sound Sync, the wire-level protocol it observes, the runtime state machine it runs, the host-facing controls and notifications it exposes, and the integration points within the existing firmware architecture. The implementation goal is a self-contained module that adds the feature without altering any other audio path or constraint.

### 1.1 What "support" means here

DSPi will:

1. **Detect** when LG Sound Sync is being broadcast on the locked SPDIF input. This is a runtime-only observation (no configuration required to detect).
2. **Decode** the LG-reported volume (0–100) and mute state from the channel-status bytes.
3. **Optionally apply** that volume/mute to the device's audio output, gated by a vendor-controllable enable flag — so non-LG sources (or even an LG source where the user does not want Sound Sync to take over) cannot accidentally have their volume changed by stale or spurious channel-status bits.
4. **Expose** the enable flag, presence, decoded volume, and decoded mute via vendor commands, the WireBulkParams payload, and the v2 notification endpoint.

### 1.2 Why volume must drive *host volume*, not master volume

Sound Sync drives the same internal control as the USB host volume slider, **not** the device master volume. This is required so loudness compensation — which is keyed off the *raw* host volume index — continues to track perceived loudness correctly when the user changes TV volume. If Sound Sync drove master volume instead, loudness would be flat against TV-volume changes and would compensate the wrong reference point.

The relationship is:

| Input source | What sets `audio_state.vol_mul` (and loudness coeffs) |
|--------------|--------------------------------------------------------|
| USB         | `audio_set_volume()` from UAC1 SET_CUR (host slider)   |
| SPDIF + Sound Sync **inactive** | The cached USB host value (frozen at last set) |
| SPDIF + Sound Sync **active**   | `lg_sound_sync` writing the LG-decoded volume |

Master volume (the device-side ceiling, `0xD2`/`0xD3`) is not affected by Sound Sync at all.

---

## 2. Background — LG Protocol Findings

### 2.1 Source

Reverse-engineered by HiFiBerry: <https://github.com/hifiberry/hifiberry-dsp/blob/master/doc/reverse-engineering-lg.md>

The doc is a series of register dumps captured from an LG OLED55C9 TV connected via TOSLINK. Each dump is 24 bytes of IEC 60958 channel-status. By comparing dumps at different volumes / mute states, the encoding can be inferred.

### 2.2 Frame and bit ordering

S/PDIF channel-status (CS) is a per-channel 192-bit field assembled across 192 sub-frame headers (one CS bit per sub-frame). Standard IEC 60958-3 ordering is **LSB-first within each 8-bit byte**: CS bit 0 is the LSB of byte 0, CS bit 7 is the MSB of byte 0, CS bit 8 is the LSB of byte 1, and so on.

Both the HiFiBerry register dumps and DSPi's `pico_spdif_rx` library present the 24-byte CS array in the same convention. Specifically the library accumulates bits LSB-first into `c_bits[]` in `_check_block()` (`firmware/pico-extras/src/rp2_common/pico_spdif_rx/spdif_rx.c:227`):

```c
uint8_t c_bit_pos = 0x1;          // start with LSB
...
if (buff[i] & (0x1 << 30)) {       // CS bit set in this sub-frame
    c_bits_byte |= c_bit_pos;
}
if (c_bit_pos >> 7) {              // crossed top of byte
    c_bits_raw[i / 16] = c_bits_byte;
    c_bit_pos = 0x1;
} else {
    c_bit_pos <<= 1;
}
```

So the byte values DSPi reads via `spdif_rx_get_c_bits()` are directly comparable to the HiFiBerry dumps. No bit-reversal needed.

### 2.3 Sound Sync presence signature

The HiFiBerry doc identifies a constant 5-nibble pattern that is only present when an LG TV is broadcasting Sound Sync:

| 0-indexed byte | Test                          | Description                      |
|----------------|-------------------------------|----------------------------------|
| 16             | `(byte & 0x0F) == 0x0F`       | Low nibble of byte 16 is `F`     |
| 17             | `byte == 0x04`                | Whole byte is `0x04`             |
| 18             | `byte == 0x8A`                | Whole byte is `0x8A`             |

Reading these as concatenated nibbles starting at the low nibble of byte 16 yields `F-0-4-8-A` = **`0xF048A`**, the signature called out by HiFiBerry as "bytes 17.5/19" in their (1-indexed) notation.

Dumps with no SPDIF or with a non-LG SPDIF source carry zero or different bytes here, so the signature has very low false-positive risk over noise. We still apply temporal hysteresis (§4) to be safe against bit errors during lock acquisition.

### 2.4 Volume encoding

HiFiBerry's "byte 16.5" — the byte spanning the high nibble of (1-indexed) byte 16 and the low nibble of (1-indexed) byte 17, which translates to the high nibble of (0-indexed) byte 15 plus the high nibble of (0-indexed) byte 16 — encodes a 7-bit volume value (0–100) plus a mute flag in bit 7:

```c
// Reconstruct the LG volume byte from CS bytes 15 and 16.
uint8_t vol_byte = ((cs[15] & 0x0F) << 4) | ((cs[16] & 0xF0) >> 4);

bool    muted   = (vol_byte & 0x80) != 0;
uint8_t volume  = vol_byte & 0x7F;       // 0..100 (clamp; values >100 are unobserved)
```

Verification against every HiFiBerry sample:

| LG vol | cs[15] | cs[16] | reconstructed vol_byte | mute bit | volume |
|--------|--------|--------|------------------------|----------|--------|
| 0%     | `0x10` | `0x0F` | `0x00`                 | 0        | 0      |
| 50%    | `0x13` | `0x2F` | `0x32`                 | 0        | 50     |
| 51%    | `0x13` | `0x3F` | `0x33`                 | 0        | 51     |
| 52%    | `0x13` | `0x4F` | `0x34`                 | 0        | 52     |
| 53%    | `0x13` | `0x5F` | `0x35`                 | 0        | 53     |
| 80% (unmuted) | `0x05` | `0x0F` | `0x50`         | 0        | 80     |
| 80% (muted)   | `0x0D` | `0x0F` | `0xD0`         | 1        | 80     |
| 100%   | `0x16` | `0x4F` | `0x64`                 | 0        | 100    |

Byte 11 also varies with volume (HiFiBerry confirms volume is "encoded multiple times"), but the byte-15/16 location is sufficient and is what the doc recommends. We do not decode the redundant copies in the initial implementation.

### 2.5 Mute encoding

Mute is the high bit (bit 7) of the reconstructed `vol_byte`. Equivalently, it can be checked directly on `cs[15]`:

```c
bool muted = (cs[15] & 0x08) != 0;
```

The doc shows that mute also toggles bit 3 of `cs[10]` (a redundant copy at CS bit 83). Either location is sufficient; the implementation uses only the `cs[15]` test.

When the TV is muted, the volume value is preserved in the lower 7 bits — i.e., unmuting restores the same level. The implementation honors this by not destroying the held volume on mute.

### 2.6 What the protocol does *not* offer

- No back-channel — the TV cannot see whether DSPi acknowledged anything.
- No discrete pairing/handshake — Sound Sync presence is purely "did we see the signature".
- No timestamping, sequence numbers, or replay protection — every poll independently re-reads the current volume.
- No standardized "leave Sound Sync" announcement — when the TV switches to non-Sound-Sync, the signature simply disappears.

These limitations drive the detection design (§4): pure observation, with hysteresis to absorb bit errors and lock transients.

---

## 3. Design Goals & Hard Constraints

### 3.1 Goals

- **Modular.** Detection and application logic live in a new `lg_sound_sync.c/h` module. No existing module learns about Sound Sync internals; integration is via small, narrow seams.
- **Maintainable.** All non-obvious decisions (e.g., the byte-15/16 nibble unpack, the hysteresis thresholds, the choice to drive host volume not master volume) are documented inline at the call site. Comments explain *why*, not just *what*.
- **Extensible.** The feature is built so that future protocol extensions (decoding the redundant copies, adding Samsung's analogous protocol, exposing a configurable LG-volume → vol_index curve) can be added without rewriting the core.
- **Zero tech debt.** No transient flags, no scratch globals leaking into other modules, no commented-out half-features, no version-conditional shims that survive past the version they fix.
- **Observable.** Host apps see four pieces of state: feature `enabled`, runtime `present`, decoded `volume`, decoded `muted`. All four are reflected in WireBulkParams and notified on change.

### 3.2 Hard constraint: output slot alignment

CLAUDE.md mandates that nothing in the firmware may cause sample-level misalignment between output slots. Sound Sync's only effect on the audio path is to write `audio_state.vol_mul` (and conditionally `current_loudness_coeffs`). Both writes are read every audio packet by the existing pipeline ramp (see §5.3), which already handles asynchronous changes in a click-free, slot-symmetric way. No code added by this feature calls `complete_pipeline_reset()`, `prepare_pipeline_reset()`, or any DMA / PIO / pool reset. **Slot alignment is preserved across every Sound Sync transition** (enable, disable, presence change, volume change, mute, unmute, input source switch, feature toggle, preset load, factory reset, flash write, boot).

### 3.3 Soft constraint: USB host volume semantics unchanged

When Sound Sync is **inactive** (feature disabled, or feature enabled but not detected), the existing `audio_set_volume()` semantics on SPDIF input are preserved exactly: `audio_state.volume` records the host-set value, `vol_mul` does not respond to OS slider while on SPDIF, and the SPDIF→USB transition handler thaws the cached value. Sound Sync only takes over when it has positively detected the signature.

---

## 4. Architecture Overview

### 4.1 New module: `lg_sound_sync.c/h`

A self-contained module added to `firmware/DSPi/`. Public surface:

```c
// lg_sound_sync.h

typedef enum {
    LG_SS_ABSENT  = 0,   // Signature not seen (or feature disabled, or input not SPDIF)
    LG_SS_PRESENT = 1,   // Signature stable; LG volume is being applied
} LgSoundSyncState;

// Aggregate runtime status.  Returned by REQ_GET_LG_SOUND_SYNC_STATUS.
typedef struct __attribute__((packed)) {
    uint8_t enabled;   // User-controlled feature gate (0/1)
    uint8_t present;   // Detection state machine: 1 if PRESENT, else 0
    uint8_t volume;    // Last decoded LG volume (0..100; 0xFF if never decoded)
    uint8_t muted;     // Last decoded LG mute (0/1; meaningful only when present)
    uint8_t reserved[12];   // Pad to 16 bytes for forward compatibility
} LgSoundSyncStatus;

// Init + tick.
void lg_sound_sync_init(void);
void lg_sound_sync_tick(void);            // Called from main loop ~every iteration

// Vendor command surface.
void lg_sound_sync_set_enabled(bool en);  // Persists to directory + applies live
bool lg_sound_sync_get_enabled(void);
void lg_sound_sync_get_status(LgSoundSyncStatus *out);

// Lifecycle hooks (called by main.c / vendor_commands.c on relevant transitions).
void lg_sound_sync_on_input_source_change(uint8_t new_source);
```

Internally the module owns:

- The decoded last-known volume / mute / present state (single producer: main loop's tick).
- The rolling hysteresis counters (`present_streak`, `absent_streak`).
- A timestamp for the last poll (to throttle to the configured cadence).
- The cached vol_index that was last applied to `vol_mul` (for change detection — avoids redundant writes).

It does **not** own:

- The SPDIF channel-status buffer (lives in the library, accessed via `spdif_input_get_channel_status()`).
- The vol_mul / loudness multipliers (live in `usb_audio.c`, written via the new `apply_vol_index_to_audio()` helper — see §5.4).
- The flash-backed enable flag (lives in `PresetDirectory` in `flash_storage.c`; the module reads/writes via the directory API).

### 4.2 Integration seams

Five small seams; everything else is internal to the module:

1. **Channel-status read.** Reuses the existing `spdif_input_get_channel_status(uint8_t out[24])` function (`spdif_input.c:464`). No changes.
2. **Apply-only vol/loudness helper.** Refactor of `audio_set_volume()`: extract the `vol_mul` + `current_loudness_coeffs` write into a new `apply_vol_index_to_audio(uint8_t vol_index)` helper. Both `audio_set_volume()` and `lg_sound_sync` call this. No behavior change for the host-volume path.
3. **Input source change notification.** `main.c`'s input-source switch handler calls `lg_sound_sync_on_input_source_change(new_source)` so the module can demote to ABSENT and let the existing thaw path restore vol_mul.
4. **Vendor command dispatch.** `vendor_commands.c` adds three handlers (0xE6 / 0xE7 / 0xE8).
5. **Bulk params + notifications.** `bulk_params.c` reads/writes the new `WireLgSoundSync` section; `lg_sound_sync.c` uses `notify_param_write()` on state changes.

### 4.3 Signal chain effect (where vol_mul is consumed)

`vol_mul` (host volume in Q15-ish format) is multiplied into every output slot by `audio_pipeline.c`. The pipeline already runs a per-packet linear ramp from `vol_mul_master_prev` to a target derived from `audio_state.vol_mul × master_volume_linear`, so any Sound Sync write is consumed on the *next* audio packet boundary and ramped over `sample_count` samples — typically 48 samples = 1 ms at 48 kHz. This gives glitch-free transitions for free, with no module-side smoothing required.

Loudness coefficients (`current_loudness_coeffs`) are pointer-swapped at vol-index granularity. The pipeline reads the pointer atomically on each packet; an old-vs-new mismatch lasts at most one packet (~1 ms) which is well below the perceptual threshold for filter-coefficient swaps at the levels involved.

---

## 5. Detection State Machine

### 5.1 State

Two abstract states with hysteresis. Implemented as a pair of streak counters, not a true state enum, so transitions are simply threshold crossings.

```c
// Implementation sketch (private to lg_sound_sync.c)

#define LG_POLL_INTERVAL_MS    50    // Channel status reading cadence
#define LG_PRESENT_THRESHOLD    3    // ~150 ms of consistent signature → declare present
#define LG_ABSENT_THRESHOLD    10    // ~500 ms of missing signature → declare absent

static uint8_t  present_streak = 0;
static uint8_t  absent_streak  = 0;
static bool     present        = false;
static uint64_t last_poll_us   = 0;
static uint8_t  last_volume    = 0xFF;   // 0xFF sentinel: "never seen"
static bool     last_muted     = false;
static int8_t   last_applied_vol_index = -1;  // For change-coalescing
```

**Why two thresholds?** Asymmetric hysteresis. Present-rise is faster than present-fall:

- Quick rise (3 polls = 150 ms) means the user gets responsive volume control as soon as Sound Sync starts.
- Slow fall (10 polls = 500 ms) means a brief signal hiccup or single corrupted block does not cause vol_mul to suddenly rebound to the cached USB host value mid-listening.

### 5.2 Polling cadence

50 ms is the chosen cadence. Justification:

- The library's `c_bits` array is fully populated only after a 192-frame block aligns and decodes — about 32 ms at 48 kHz, longer at 44.1 kHz.
- LG TV remote presses can produce one CS update per (volume step), with multiple presses per second.
- Polling at 50 ms guarantees we see fresh CS data on every poll while keeping main-loop overhead negligible (3 short int compares + a 24-byte memcpy + ISR-disabled critical section in the library).
- 50 ms × 3-poll threshold = 150 ms latency from "TV starts Sound Sync" to "DSPi takes over volume" — comfortably below human perception of "instant".

The cadence is implemented with `time_us_64()` deltas, not a counter, so it is robust against variable main-loop pacing (e.g., main loop running faster during heavy USB activity).

### 5.3 Tick algorithm

```c
void lg_sound_sync_tick(void) {
    // Cheap exits — no allocation, no I/O, run every main-loop iteration.
    if (!enabled) {
        // Feature disabled.  If we were previously driving vol_mul, hand it back.
        if (present) demote_to_absent_and_restore_host_volume();
        return;
    }
    if (active_input_source != INPUT_SOURCE_SPDIF) {
        // Not on SPDIF — nothing to detect.
        if (present) demote_to_absent();   // No restore: input switch handles it
        return;
    }

    // Throttle to LG_POLL_INTERVAL_MS.  Library does its own block-buffering;
    // polling faster than the block rate just re-reads identical bytes.
    uint64_t now_us = time_us_64();
    if ((now_us - last_poll_us) < (LG_POLL_INTERVAL_MS * 1000ULL)) return;
    last_poll_us = now_us;

    // SPDIF must be locked for channel status to be meaningful.
    if (spdif_input_get_state() != SPDIF_INPUT_LOCKED) {
        bump_absent_streak();
        return;
    }

    uint8_t cs[24];
    spdif_input_get_channel_status(cs);

    // Signature: cs[16] low nibble = 0xF, cs[17] = 0x04, cs[18] = 0x8A.
    bool sig = ((cs[16] & 0x0F) == 0x0F) && (cs[17] == 0x04) && (cs[18] == 0x8A);

    if (sig) {
        absent_streak = 0;
        if (present_streak < 0xFF) present_streak++;
        if (present_streak >= LG_PRESENT_THRESHOLD) {
            // Decode volume and mute, apply to audio path.
            uint8_t vol_byte = (uint8_t)(((cs[15] & 0x0F) << 4) | ((cs[16] & 0xF0) >> 4));
            uint8_t lg_vol   = vol_byte & 0x7F;
            bool    lg_mute  = (vol_byte & 0x80) != 0;
            if (lg_vol > 100) lg_vol = 100;       // Defensive clamp; values >100 unobserved
            apply_lg_state(lg_vol, lg_mute);
            mark_present_if_new();
        }
    } else {
        present_streak = 0;
        if (absent_streak < 0xFF) absent_streak++;
        if (present && absent_streak >= LG_ABSENT_THRESHOLD) {
            demote_to_absent_and_restore_host_volume();
        }
    }
}
```

`mark_present_if_new()` and `demote_to_absent_and_restore_host_volume()` push notifications and update the `present` boolean. They are idempotent and cheap on the no-change path, so they can be called every poll without per-iteration cost.

### 5.4 What "demote to absent" does

When Sound Sync transitions from PRESENT to ABSENT, we must not leave `vol_mul` stuck at the last LG-decoded value. The desired behavior is to fall back to the same value `audio_set_volume()` would have produced for the cached USB host volume (`audio_state.volume`). This is implemented by calling `audio_set_volume(audio_state.volume)`, which (since SPDIF input causes the early return) writes the cached value back to `audio_state.volume` (no-op) and exits without touching `vol_mul`. To force a vol_mul update on demote we instead call the new helper directly:

```c
// Recompute the vol_index that the cached host volume would map to and apply it.
// This restores both vol_mul AND loudness coeffs in one shot.
static void restore_host_volume_to_audio_path(void) {
    int16_t cached = audio_state.volume;     // Last value the host wrote
    cached += CENTER_VOLUME_INDEX * 256;     // Same arithmetic as audio_set_volume()
    if (cached < 0) cached = 0;
    if (cached >= (CENTER_VOLUME_INDEX + 1) * 256) cached = (CENTER_VOLUME_INDEX + 1) * 256 - 1;
    apply_vol_index_to_audio((uint8_t)((uint16_t)cached >> 8));
}
```

This block is private to `lg_sound_sync.c`. The duplication of the arithmetic from `audio_set_volume()` is intentional: the rule "compute vol_index from a 16-bit signed dB-like value" is a thin slice we do not want to expose as another public helper, and adding a second public entry point would dilute the audio-set abstraction. The duplication is annotated as such in the code.

---

## 6. Volume / Mute Application

### 6.1 LG volume → vol_index mapping

The pipeline's volume table `db_to_vol[]` (`usb_audio.c:332`) has 61 entries indexed `0..CENTER_VOLUME_INDEX` (0..60). Index 0 is true silence (`0x0000`), index 60 is unity gain (`0x8000`), and intermediate steps approximate 1 dB each.

LG volume runs 0..100. We map proportionally with rounded division so endpoints land cleanly:

```c
// lg_vol ∈ [0, 100]; result ∈ [0, CENTER_VOLUME_INDEX = 60].
// Round-half-up division: vol_index = (lg_vol * 60 + 50) / 100.
//   lg=0   →  0   (silent — db_to_vol[0] = 0)
//   lg=1   →  1   (~ -60 dB)
//   lg=50  → 30   (~ -30 dB)
//   lg=99  → 59   (~ -1 dB; 99×60+50 = 5990, /100 = 59)
//   lg=100 → 60   (= 0 dB, unity)
// Only lg=100 reaches unity; lg=99 lands at -1 dB (vol_index 59).
static inline uint8_t lg_vol_to_vol_index(uint8_t lg_vol) {
    if (lg_vol > 100) lg_vol = 100;
    uint32_t scaled = (uint32_t)lg_vol * (uint32_t)CENTER_VOLUME_INDEX + 50u;
    uint8_t  idx    = (uint8_t)(scaled / 100u);
    if (idx > CENTER_VOLUME_INDEX) idx = CENTER_VOLUME_INDEX;
    return idx;
}
```

Properties:

- **Monotonic**, with at most ±1 step jitter near boundaries — fine for human perception.
- **No tunable parameters** in v1. A future extension could expose a configurable `min_db` or a curve type (linear-in-dB vs. perceptual), but the linear-in-vol-index mapping matches what LG TVs themselves do internally and matches the table's existing 1-dB-per-step characteristic.
- **Pure function**; no state. Easy to test.

### 6.2 Apply path — `apply_vol_index_to_audio()`

A new helper added to `usb_audio.c`, declared in `usb_audio.h`. Refactors the inner block of `audio_set_volume()`:

```c
// (usb_audio.h)
//
// Apply a vol_index in [0..CENTER_VOLUME_INDEX] to the live audio path:
// updates vol_mul (which the audio pipeline ramps to over the next packet)
// and refreshes the loudness compensation coefficient pointer when loudness
// is enabled.  Does NOT modify audio_state.volume — the caller decides
// whether the host's reported value is being changed or merely re-applied.
//
// Safe to call from the main loop.  NOT safe from ISR context — vol_mul is
// not a single 16-bit atomic on all targets.
void apply_vol_index_to_audio(uint8_t vol_index);
```

```c
// (usb_audio.c)
void apply_vol_index_to_audio(uint8_t vol_index) {
    // Defensive clamp: callers are trusted but a corrupt mapping should not
    // index outside db_to_vol[]'s 61-entry table.
    if (vol_index > CENTER_VOLUME_INDEX) vol_index = CENTER_VOLUME_INDEX;

    audio_state.vol_mul = db_to_vol[vol_index];

    // Loudness compensation tracks the *raw* user-perceived volume.  Whoever
    // owns the volume — host slider, Sound Sync, or future controllers — must
    // funnel through this helper so loudness stays consistent.
    if (loudness_enabled && loudness_active_table) {
        current_loudness_coeffs = loudness_active_table[vol_index];
    }
}
```

`audio_set_volume()` is then trimmed to:

```c
void audio_set_volume(int16_t volume) {
    audio_state.volume = volume;                       // Always cache for round-trip GET
    if (active_input_source != INPUT_SOURCE_USB) return;
    volume += CENTER_VOLUME_INDEX * 256;
    if (volume < 0) volume = 0;
    if (volume >= (CENTER_VOLUME_INDEX + 1) * 256) volume = (CENTER_VOLUME_INDEX + 1) * 256 - 1;
    apply_vol_index_to_audio((uint8_t)(((uint16_t)volume) >> 8));
}
```

Behavior is byte-identical to today's code path on USB input. SPDIF-input behavior is unchanged when Sound Sync is inactive.

### 6.3 Mute behavior

LG mute is communicated as bit 7 of the reconstructed vol_byte. When asserted we want true silence on the device, but we do **not** want to lose the held volume value so unmuting is instant. Implementation:

```c
static void apply_lg_state(uint8_t lg_vol, bool lg_mute) {
    uint8_t vol_index = lg_vol_to_vol_index(lg_vol);

    if (lg_mute) {
        // Hard mute via vol_mul = 0.  The next packet's pipeline ramp will
        // glide vol_mul from its current target down to 0 — click-free.
        // Do not touch loudness coeffs: holding the previous coefficients
        // is correct because the signal is silenced anyway, and unmuting
        // from this state needs the coeffs already in place to avoid a
        // one-packet mismatch glitch.
        audio_state.vol_mul = 0;
    } else if (vol_index != last_applied_vol_index) {
        // Coalesce: only re-apply when the resulting index actually changes.
        // db_to_vol[] is a flat lookup, but loudness coefficient pointer
        // chasing is a pointer-load per packet so skipping no-ops is real.
        apply_vol_index_to_audio(vol_index);
        last_applied_vol_index = (int8_t)vol_index;
    }

    if (lg_vol != last_volume || lg_mute != last_muted) {
        last_volume = lg_vol;
        last_muted  = lg_mute;
        push_status_change_notifications();   // §7.3
    }
}
```

The "hold loudness coefficients on mute" choice is documented inline. The alternative (zero them) would create a 1-packet (~1 ms) coefficient-mismatch transient on every unmute and is not worth the consistency.

### 6.4 Click-free transitions (delegated)

There is intentionally no slewing or smoothing inside `lg_sound_sync.c`. All transitions piggy-back on the audio pipeline's existing per-packet vol_mul ramp (`audio_pipeline.c:97`–`231` and parallel int32 path at `:573`). The pipeline:

- Reads `audio_state.vol_mul` and `master_volume_linear` once per packet.
- Computes a target multiplier and ramps from `vol_mul_master_prev` to that target linearly across `sample_count` samples.
- Stores the target back as the next packet's `prev`.

So: a Sound Sync write to `vol_mul` is consumed at the next packet boundary, fades cleanly over ~1 ms (at 48 kHz with 48-sample packets), and feeds Core 1 through `core1_eq_work.vol_mul_start/step` symmetrically with all other volume changes. There is no path by which Sound Sync can produce a sample-level discontinuity that other vol changes don't already smooth.

---

## 7. Vendor Commands & Notifications

### 7.1 Command IDs (config.h)

```c
// LG Sound Sync (optical) Commands
#define REQ_SET_LG_SOUND_SYNC_ENABLE   0xE6  // payload uint8_t (0=off, 1=on); persists to directory
#define REQ_GET_LG_SOUND_SYNC_ENABLE   0xE7  // returns uint8_t
#define REQ_GET_LG_SOUND_SYNC_STATUS   0xE8  // returns LgSoundSyncStatus (16 bytes)
```

`0xE6`/`0xE7`/`0xE8` are the next free IDs after the SPDIF input block (`0xE0`..`0xE5`), keeping all SPDIF-related commands clustered.

### 7.2 Wire format extension

Add a new fixed-size section to `WireBulkParams`. WIRE_FORMAT_VERSION bumps `7 → 8`.

```c
// bulk_params.h

// ============================================================================
// Section 16: LG Sound Sync (16 bytes) — V8+
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t enabled;       // User-controlled gate (0/1).  Read/write on bulk SET.
    uint8_t present;       // Detection state.  Read-only — bulk SET ignores.
    uint8_t volume;        // Last decoded LG volume (0..100; 0xFF if never).
    uint8_t muted;         // Last decoded LG mute (0/1; meaningful if present=1).
    uint8_t reserved[12];
} WireLgSoundSync;          // 16 bytes
```

`bulk_params_collect()` populates all four fields; `bulk_params_apply()` honors only `enabled` (the rest are runtime observations the host cannot meaningfully push). This asymmetry mirrors how `WireMasterVolume.master_volume_db` is honored but `WireBulkParams.input_config.input_source` is not (cross-checked with `apply_pins`-style read-only handling).

### 7.3 Notification events

Standard `NOTIFY_EVT_PARAM_CHANGED` events on the WireBulkParams offset of each changed field:

| Field changed | When                                  | Source tag              |
|---------------|---------------------------------------|--------------------------|
| `enabled`     | Vendor SET (0xE6) or bulk SET         | `PARAM_SRC_HOST_SET`     |
| `present`     | Detection state crosses threshold     | `PARAM_SRC_INTERNAL`     |
| `volume`      | LG-decoded value changes              | `PARAM_SRC_INTERNAL`     |
| `muted`       | LG-decoded mute toggles               | `PARAM_SRC_INTERNAL`     |

Since the four fields live in one wire-format struct, the host sees fine-grained PARAM_CHANGED events for each. Coalescing within the notify ring (`notify.c:113`) means rapid LG vol changes naturally collapse to a single in-flight packet per field.

When `present` flips to 1 and the same poll also produces a `volume` change (the common first-detect case), the module pushes the `volume` and `muted` events first, then `present` last — host UIs can treat the `present=1` event as "all fields are now live" without needing to re-read.

### 7.4 Behavior summary by command

| Command | Effect |
|---------|--------|
| `REQ_SET_LG_SOUND_SYNC_ENABLE` (1 byte payload) | Updates the live `lg_sound_sync_enabled` flag immediately. **No flash write fires** — the value persists only when the user saves the active preset (matches loudness/crossfeed/leveller toggle semantics). If transitioning enabled→disabled while PRESENT, demotes to ABSENT and restores host volume. If transitioning disabled→enabled while on SPDIF and locked, begins polling on next tick. Pushes a PARAM_CHANGED on the `enabled` offset. |
| `REQ_GET_LG_SOUND_SYNC_ENABLE` | Returns 1 byte of the live flag. |
| `REQ_GET_LG_SOUND_SYNC_STATUS` | Returns the 16-byte `LgSoundSyncStatus` struct (= the WireLgSoundSync section minus the bulk-only positioning). |

---

## 8. Persistence

### 8.1 Storage location

The `lg_sound_sync_enabled` flag is **per-preset**. It lives in `PresetSlot` alongside other per-preset behavior settings (loudness enable, crossfeed config, leveller config, channel names, EQ bands, …). The flag follows the same lifecycle as those settings:

- Live state is the source of truth — a single global `bool lg_sound_sync_enabled` in `usb_audio.c` (or the module — see §10).
- Vendor SET (`0xE6`) writes live state immediately. No flash write fires.
- On `REQ_SAVE_PRESET`, the live value is captured into the active `PresetSlot` and written to its sector.
- On `REQ_LOAD_PRESET`, the slot's stored value is restored to live state. `apply_factory_defaults()` resets it to the firmware default.
- Bulk SET (`REQ_SET_ALL_PARAMS`) writes the wire `enabled` byte directly to live state, same as any other live-write parameter.

**Why per-preset?** Different listening profiles legitimately want different Sound Sync behavior. A "Headphone" preset selected on a different output may not want the TV remote to drive its level; a "Mastering" preset may want a known-fixed volume independent of TV state; a "TV Listening" preset wants Sound Sync on. Putting the flag in the preset matches every other "what does the audio path do here" toggle in the firmware (loudness, leveller, crossfeed, master EQ bypass).

The default for a fresh slot or a legacy slot pre-V13 is **off** — non-LG users see no behavior change after FW update, and existing presets continue to behave as they always have until the user explicitly toggles Sound Sync on and re-saves.

### 8.2 Slot schema bump

`PresetSlot` gets a new byte. `SLOT_DATA_VERSION` bumps (currently 12 → 13; verify against `flash_storage.h` at implementation time — if any other in-flight feature has already bumped it, take the next number). The slot reader treats a slot whose stored version is < 13 as having `lg_sound_sync_enabled = 0` (firmware default), so existing user presets load cleanly without re-saving.

### 8.3 Wire-format bump

`WIRE_FORMAT_VERSION` bumps `7 → 8`. Bulk SET payloads from a pre-V8 host are rejected at the EP0 dispatcher's `wLength == sizeof(WireBulkParams)` gate (`vendor_commands.c`, in the `REQ_SET_ALL_PARAMS` setup-stage handler) — a V7 host sends `wLength = 2912` and the new `sizeof(WireBulkParams) = 2928`, so the dispatcher returns `false` and TinyUSB STALLs the transfer. The version-check guard inside `bulk_params_apply()` is a defense-in-depth backup that catches mismatches if the dispatcher gate is ever loosened (e.g. for forward-compat smaller payloads); the V8 LG-section apply has its own size assertion (`payload_length >= sizeof(WireBulkParams)`) so a relaxed dispatcher gate cannot result in a read past the transferred bytes.

### 8.4 Default value

`LG_SOUND_SYNC_DEFAULT_ENABLED` = `0` (defined in `config.h`). Used by:

- Power-on initialization (until `preset_boot_load` overwrites it).
- `apply_factory_defaults()` for the live state.
- Slot read fallback for pre-V13 stored slots.
- Bulk-params collect when called before `lg_sound_sync_init()` runs (defensive).

### 8.5 What is *not* persisted

- `present`, `volume`, `muted` — runtime observations, never written to flash.
- The hysteresis counters, the last-applied vol_index, and the last-poll timestamp — RAM only.

---

## 9. Lifecycle & Coordination

### 9.1 Boot

1. `flash_storage_init()` reads the directory, populates `lg_sound_sync_enabled` from the directory byte (default 0 if directory pre-V8).
2. `lg_sound_sync_init()` resets all RAM state: streaks=0, present=false, last_volume=0xFF, last_muted=false, last_applied_vol_index=-1, last_poll_us=0.
3. Main loop begins calling `lg_sound_sync_tick()` once per iteration.

### 9.2 Input source switch (SPDIF ↔ USB)

- `main.c`'s input-switch handler calls `lg_sound_sync_on_input_source_change(new_source)` *after* the new active source is committed but *before* it returns to the main loop.
- The hook updates the module's local copy of the active source (cheap atomic) and, if the new source is anything other than SPDIF, demotes to ABSENT without calling `restore_host_volume_to_audio_path()` — because the input-source change handler itself thaws the cached host volume on USB transitions, and on SPDIF→other-future-input there is no thaw to perform.

### 9.3 SPDIF lock loss

When `spdif_input_get_state() != SPDIF_INPUT_LOCKED`, the tick routine bumps `absent_streak` once per poll. After 10 polls (~500 ms) of unlocked, present demotes to absent. This delay is intentional — it avoids vol_mul snapping back to the cached USB value on every brief SPDIF-input glitch (e.g. cable unplug+replug, sample rate change).

### 9.4 Flash-write blackouts

`prepare_flash_write_operation()` (`main.c:681`) tears down SPDIF RX before flash writes. While the RX is down, `spdif_input_get_state()` returns `SPDIF_INPUT_INACTIVE` and our tick treats that the same as not-locked — bumps `absent_streak`. For the typical preset save (one write, ~50 ms blackout) we never reach the 500 ms absent threshold, so present persists across the write. For longer write sequences (factory reset, multi-sector preset save) we do demote to absent, vol_mul restores to host-cached, and on RX restart Sound Sync re-detects within ~150 ms.

The pipeline's `preset_loading` flag mutes audio across the blackout, so the user doesn't hear the volume transition.

### 9.5 Preset load

Preset load restores `lg_sound_sync_enabled` from the loaded slot. The mute envelope (`preset_loading`) is already in effect across the load, so any consequent vol_mul ramp is inaudible. After the load completes:

- If the loaded slot has `enabled=0`: the next tick (within ~50 ms) demotes to absent and restores `vol_mul` to the cached USB host value via `restore_host_volume_to_audio_path()`.
- If the loaded slot has `enabled=1` and the preset was saved while Sound Sync was active: the module needs to re-detect from scratch on the new tick stream. There is no shortcut; the streaks reset on every preset load. ~150 ms after the mute envelope opens, vol_mul resumes TV-driven control if the signature is still on the wire.

The streak reset is intentional. Stored `volume`/`muted` from the old preset are not authoritative because the TV may have changed state during the preset transition, and re-detecting from CS is the only correct source.

`notify_push_preset_loaded()` already drives the host to re-read `WireBulkParams`, so the host sees the new `enabled` value automatically. No per-field PARAM_CHANGED is needed during preset load — the bulk invalidate covers it (per the existing notify v2 protocol).

### 9.6 Factory reset

`apply_factory_defaults()` writes `LG_SOUND_SYNC_DEFAULT_ENABLED` (= 0) to live state and resets the streaks. The active slot's stored value is **not** rewritten — the policy "factory reset keeps the active slot unchanged" (per the User Preset System memory entry) is preserved. If the user wants the change persisted, they save the preset.

---

## 10. Files Touched

### 10.1 New files

| Path                                                  | Purpose                                  |
|-------------------------------------------------------|------------------------------------------|
| `firmware/DSPi/lg_sound_sync.h`                       | Public API                               |
| `firmware/DSPi/lg_sound_sync.c`                       | Implementation                           |
| `Documentation/Features/lg_sound_sync_spec.md`        | This document                            |

### 10.2 Modified files

| Path                                                | Changes                                                                                                                                                          |
|-----------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `firmware/DSPi/CMakeLists.txt`                      | Add `lg_sound_sync.c` to source list.                                                                                                                            |
| `firmware/DSPi/config.h`                            | Add `REQ_SET_LG_SOUND_SYNC_ENABLE` / `REQ_GET_LG_SOUND_SYNC_ENABLE` / `REQ_GET_LG_SOUND_SYNC_STATUS` (0xE6/E7/E8). Bump `WIRE_FORMAT_VERSION` from 7 to 8.        |
| `firmware/DSPi/usb_audio.h`                         | Declare `apply_vol_index_to_audio()`.                                                                                                                            |
| `firmware/DSPi/usb_audio.c`                         | Implement `apply_vol_index_to_audio()`. Refactor `audio_set_volume()` to call it. No behavior change on USB path.                                                |
| `firmware/DSPi/main.c`                              | Call `lg_sound_sync_init()` after flash/audio init. Call `lg_sound_sync_tick()` once per main loop iteration. Hook input-source-change to call `lg_sound_sync_on_input_source_change()`. |
| `firmware/DSPi/vendor_commands.c`                   | Add 0xE6/E7/E8 dispatch cases. The SET handler updates live state only — no deferred-flash dance, matching the loudness/crossfeed/leveller pattern.              |
| `firmware/DSPi/bulk_params.h`                       | Add `WireLgSoundSync` struct definition. Append `WireLgSoundSync lg_sound_sync;` to `WireBulkParams`. Total grows by 16 bytes (2912 → 2928). Bump `WIRE_BULK_PARAMS_SIZE` consumer comments. |
| `firmware/DSPi/bulk_params.c`                       | `bulk_params_collect()` populates all four fields from live module state. `bulk_params_apply()` honors only `enabled`; logs (or silently drops) writes to read-only fields. |
| `firmware/DSPi/flash_storage.h`                     | Add `uint8_t lg_sound_sync_enabled` to `PresetSlot`. Bump `SLOT_DATA_VERSION` (e.g. 12 → 13). Add `LG_SOUND_SYNC_DEFAULT_ENABLED` constant.                      |
| `firmware/DSPi/flash_storage.c`                     | `preset_save_to_slot()` captures the live flag; `preset_load_from_slot()` restores it (with default-on-legacy fallback for slot version < 13). `apply_factory_defaults()` writes the default to live state. |
| `Documentation/current_architecture.md`             | Append a "LG Sound Sync" subsection describing the module, polling, hysteresis, and integration seams. Stamp `*Last updated: 2026-05-08*`.                       |

### 10.3 Files explicitly NOT touched

- `audio_pipeline.c`, `audio_pipeline.h`, `dsp_pipeline.*`, `dsp_process_rp2040.S` — no DSP path changes.
- `spdif_input.c`, `spdif_input.h` — already exposes `spdif_input_get_channel_status()`.
- `pico_spdif_rx` library — channel status decoding already works.
- `usb_descriptors.c`, `tusb_config.h` — no USB stack changes.
- Notify ring or notification protocol — uses existing PARAM_CHANGED on a new offset.

---

## 11. Implementation Order / Phasing

A single PR is acceptable — the change is small, well-localized, and fully tested by toggling LG TV volume + power. But the work breaks naturally into four self-contained commits if a phased review is preferred:

### Phase 1 — `apply_vol_index_to_audio()` refactor

Smallest, highest-leverage. Extract the inner block of `audio_set_volume()` into the helper. No new behavior. Verify USB host volume path is byte-identical (regress test: change OS slider, observe vol_mul, confirm loudness coeff pointer updates).

**Files:** `usb_audio.h`, `usb_audio.c`.

### Phase 2 — Module skeleton + detection

Add `lg_sound_sync.c/h` with detection and decoding only — no application yet. Wire `lg_sound_sync_init()` and `lg_sound_sync_tick()` into `main.c`. Verify with a TV: confirm `present`, `volume`, `muted` reach correct values via a debug printf or temporary status command.

**Files:** new `.c`/`.h`, `CMakeLists.txt`, `main.c`.

### Phase 3 — Application + vendor commands

Connect detection to `apply_vol_index_to_audio()`. Add 0xE6/E7/E8 vendor commands writing live state only. Verify TV-volume changes drive DSPi `vol_mul`; verify `enabled=0` disables takeover. Persistence is not yet wired — the flag survives a reboot only if Phase 4 ships.

**Files:** `lg_sound_sync.c`, `vendor_commands.c`, `config.h`.

### Phase 4 — Persistence + wire format + notifications

Bundle three coupled changes:

1. **Slot persistence.** Add the `lg_sound_sync_enabled` byte to `PresetSlot`, bump `SLOT_DATA_VERSION`, hook capture/restore in `preset_save_to_slot()` / `preset_load_from_slot()`, hook reset in `apply_factory_defaults()`.
2. **Wire format.** Add `WireLgSoundSync`, bump `WIRE_FORMAT_VERSION`, wire `bulk_params_collect()` / `bulk_params_apply()`.
3. **Notifications.** Push PARAM_CHANGED on `enabled` (host SET, bulk SET) and on `present`/`volume`/`muted` (internal). Verify the host UI sees all four fields and updates promptly.

These three are bundled because shipping them separately is incoherent — splitting persistence from the wire format means the host can't see the per-preset state, and splitting notifications from either means the host has to poll. One commit, three logical sections.

**Files:** `flash_storage.h`/`.c`, `bulk_params.h`/`.c`, `lg_sound_sync.c` (notify pushes), `Documentation/current_architecture.md`.

Each phase builds + flashes cleanly without the next, making bisection trivial if a regression appears.

---

### 11.5 Schema-bump coordination

`SLOT_DATA_VERSION` and `WIRE_FORMAT_VERSION` should be bumped in the *same* commit (Phase 4) so that no built artifact has only one of the two version numbers advanced — bisecting between the bumps would otherwise be confusing.

If another in-flight branch has already bumped either version when this work merges, take the next available number and update this spec — the behavior described here does not depend on the exact integer.

---

## 12. Edge Cases & Failure Modes

| Scenario                                                | Behavior                                                                                                                                                  |
|---------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| LG TV powered off mid-playback                          | SPDIF lock loss → tick bumps absent_streak → after ~500 ms demote to absent → vol_mul restored to USB-cached.                                             |
| TV switches from Sound Sync app to a non-Sound-Sync app | Signature disappears, audio continues (PCM still flowing) → after ~500 ms demote to absent → vol_mul restored to USB-cached. No audio dropout.            |
| Bit error in one CS block corrupts signature            | Single signature miss; present_streak resets but absent_streak only +1; doesn't cross threshold; stays present. Volume holds last decoded value.          |
| Volume value corrupted (e.g., reads 200)                | Defensive clamp to 100 before mapping. Pipeline ramp absorbs the one-poll glitch over 50 ms.                                                              |
| Feature toggled disabled while present                  | Tick path's enabled-check fires → demote to absent → restore host volume. ~50 ms latency at most.                                                         |
| Feature toggled enabled while on SPDIF + signature live | Next tick (within 50 ms) sees signature, present_streak begins. Takes over within ~150 ms.                                                                |
| Input switched SPDIF→USB while present                  | `lg_sound_sync_on_input_source_change()` demotes to absent. The switch handler's existing `audio_set_volume(audio_state.volume)` thaw restores vol_mul.   |
| Input switched USB→SPDIF                                | Module re-arms; tick begins polling once SPDIF locks. Initial state is absent (vol_mul stays at USB-cached) until signature seen.                         |
| Loudness disabled mid-Sound-Sync                        | `apply_vol_index_to_audio()` no-ops on the loudness-coeff write. No glitch.                                                                               |
| Loudness enabled mid-Sound-Sync                         | Loudness module's enable handler should already re-fetch coeffs at the current `vol_mul`'s vol_index. (Verify against `loudness.c` at implementation; if not, trigger by calling `apply_vol_index_to_audio()` from the loudness-enable path.) |
| Bulk SET clears the wire enable flag                    | Module receives the writeback through `bulk_params_apply()` (which calls `lg_sound_sync_set_enabled()` internally) — same path as vendor SET. Same demote behavior. |
| Channel status stale (library hasn't refreshed yet)     | Same as no-signature-seen; absent_streak bumps. Self-corrects on next library block.                                                                      |
| Two TVs A/B-switched on the same TOSLINK                | Each TV brings its own signature. Demote-on-absent + re-detect-on-present handles the swap with ~650 ms transition.                                       |

---

## 13. Testing Plan

### 13.1 Unit-level (host-side, no hardware)

- `lg_vol_to_vol_index()` covers boundary inputs (0, 1, 50, 99, 100, 255).
- Signature detection function: synthetic 24-byte arrays with byte 16/17/18 patterns matching/non-matching.
- Volume/mute decoder: every HiFiBerry sample dump verified against expected `(volume, muted)` outputs (table from §2.4).
- Hysteresis: drive `process_cs_sample()` with synthetic streams of present/absent and verify state transitions cross thresholds at the right boundaries.

### 13.2 Hardware integration (LG TV)

| Test                                                                                                                                | Expected                                                                                                  |
|-------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| TV at 50%, Sound Sync enabled in TV settings, DSPi feature enabled                                                                 | DSPi `present=1`, `volume=50`, `muted=0`. Audio at perceptually-half volume.                              |
| TV vol+ pressed 5 times rapidly                                                                                                     | DSPi `volume` lands on the final TV vol within ~150 ms; no vol-jumps mid-press; no audible clicks.        |
| TV mute pressed                                                                                                                     | DSPi vol_mul drops to 0 within ~50 ms; pipeline ramp produces no click; loudness coeffs unchanged.        |
| TV mute pressed again                                                                                                               | DSPi vol_mul ramps back to held vol_index; instant audibility.                                            |
| TV powered off                                                                                                                      | DSPi present=0 within ~500 ms; vol_mul restored to USB-cached; no silence beyond the pipeline's ramp.     |
| Disable feature via vendor command while Sound Sync active                                                                          | Within one tick (~50 ms) DSPi present=0; vol_mul restored; volume/muted retain last-known values.         |
| Switch DSPi input to USB while Sound Sync active                                                                                    | DSPi present=0 immediately; vol_mul thawed to USB host slider value.                                      |
| Switch back to SPDIF                                                                                                                | Sound Sync re-detected within ~150 ms after SPDIF lock; vol_mul resumes TV-driven control.                |
| Save preset while Sound Sync active                                                                                                 | Audio mutes during write (preset_loading); current live `enabled=1` is captured into the slot; on resume, vol_mul re-applied within ~150 ms; no slot drift. |
| Load preset whose stored `enabled=0` while Sound Sync was active                                                                    | After preset_loading mute clears, next tick demotes to ABSENT and restores host-cached vol_mul. ~50 ms.   |
| Load preset whose stored `enabled=1` while Sound Sync was inactive                                                                  | Streaks reset; tick re-detects from CS over ~150 ms; vol_mul takes over once present_streak crosses threshold. |
| Load preset pre-V13 (stored before this feature shipped)                                                                            | Slot reader returns `enabled=0` (firmware default); behaves as if the user has never enabled Sound Sync.  |
| Factory reset                                                                                                                       | Live `lg_sound_sync_enabled` is reset to `LG_SOUND_SYNC_DEFAULT_ENABLED` (= 0). The active slot's stored value is **not** rewritten — user must save the preset to persist the reset value. |
| Connect a non-LG SPDIF source (CD player, computer)                                                                                 | DSPi present=0 indefinitely. vol_mul follows USB-cached / OS slider as today. No false detection.         |

### 13.3 Regression

Exhaustively re-run existing test plans for:

- Master Volume (`master_volume_spec.md`) — verify chain unaffected.
- USB volume + loudness compensation (`loudness.h` driven path) — verify USB-input behavior is byte-identical pre/post refactor.
- SPDIF input lock/unlock and input-source switching (`input_switching_spec.md`) — verify hooks don't introduce latency.
- Preset save/load + flash-blackout muting — verify no clicks added.

---

## 14. Open Questions / Future Extensions

These are explicitly out of scope for v1 and are listed for future work:

1. **Configurable LG-vol → vol_index curve.** Expose `min_db` and curve type (linear-in-dB vs. perceptual). Default unchanged.
2. **Decode redundant copies and cross-check.** Volume is encoded in multiple CS bytes (HiFiBerry doc §"Conclusions"). Cross-checking would let us reject single-bit flips before the present_streak hysteresis kicks in. Marginal robustness gain at the cost of code complexity; defer until field evidence shows it's needed.
3. **Samsung Anynet+ / Sony Bravia Sync.** Other TV brands have analogous one-way SPDIF control protocols. The same module shape (signature + decode + apply via `apply_vol_index_to_audio()`) supports them — would just become a per-protocol decoder behind a common detection layer.
4. **Channel-status diagnostics.** The 24 raw CS bytes are already exposed via `REQ_GET_SPDIF_RX_CH_STATUS` (0xE3). Add a debug overlay in the host app that highlights bytes 15–18 to make protocol bring-up easier.
5. **Device-wide override flag.** Add a directory-level "Sound Sync globally disabled" master switch that overrides every preset's per-preset flag. Useful for users who want to leave Sound Sync wired into their preferred presets but temporarily globally suppress it (e.g. while doing measurements). Trivial to layer on: an additional `&&` against the per-preset check in the tick path.

---

## 15. Glossary

| Term | Meaning |
|------|---------|
| **CS** | Channel Status — the 192-bit IEC 60958 metadata field carried in SPDIF sub-frames. |
| **vol_mul** | The 16-bit Q15-ish multiplier (`audio_state.vol_mul`) read once per packet by the audio pipeline. |
| **vol_index** | An 8-bit index into `db_to_vol[]` and `loudness_active_table[]`, range 0..CENTER_VOLUME_INDEX (60). |
| **CENTER_VOLUME_INDEX** | 60 (declared in `usb_audio.h`). The index for unity gain (0 dB). |
| **db_to_vol[]** | The 61-entry vol_index → Q15-ish multiplier lookup table in `usb_audio.c`. |
| **PRESENT** | The state where Sound Sync's signature is stably visible and DSPi is acting on the LG-decoded volume. |
| **ABSENT** | The state where the signature is missing or the feature is disabled / input is not SPDIF. |
| **Hysteresis streaks** | Counters of consecutive polls observing signature present/absent, used to filter spurious flips. |
