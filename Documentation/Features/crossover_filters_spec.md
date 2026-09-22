# Crossover Filters Specification

*Last updated: 2026-08-04*

## Purpose

DSPi provides per-output-channel high-order crossover filters as a dedicated stage in the DSP pipeline. Crossover filters band-limit each output to a specific frequency range — the standard pro-audio pattern for multi-driver active speaker systems (one output → woofer LP, another → tweeter HP, etc.). The crossover stage runs **after** the matrix mixer and **before** per-output PEQ, matching the active-monitor convention (input EQ → driver split → driver-correction EQ).

This document describes everything a host application needs to implement crossover support without reading firmware source.

---

## 1. User-visible model

- Every output channel has a **crossover stage** with up to **4 bands**.
- Each band is a single user-visible filter of a chosen family/order/shape.
- Each filter is internally a cascade of up to 4 biquad sections, but section count is implementation detail — the app sets one filter per band.
- Bands are independent; a typical 2-way speaker uses 1 LP on one channel and 1 HP on another (both at the same `freq`).
- Each band can be bypassed individually.

### Band-index map (per channel)

The band index is unified across PEQ and crossover for vendor command addressing:

| Band index | Meaning | Storage |
|---|---|---|
| 0 .. 9 | Active PEQ band | `filter_recipes[ch][band]` |
| 10 .. 19 | **Reserved**; rejected today (future PEQ-count growth) | n/a |
| 20 .. 23 | Crossover band 0 .. 3 | `xover_recipes[ch][band - 20]` |
| 24 .. 255 | Rejected | n/a |

`MAX_BANDS = 12` (PEQ storage; only 0–9 active today). `XOVER_BAND_BASE = 20` (crossover wire-index base). `MAX_XOVER_BANDS = 4`. The wide gap at 10–19 is intentional: future PEQ-count expansion can grow active PEQ (and `MAX_BANDS`) up to 20 bands without moving crossover indices. The base was widened from 12 to 20 in firmware revision dated 2026-06-04; see the version-history note about the `REQ_GET_EQ_PARAM` wValue re-encoding that accompanied it.

### Channels and the master-channel rule

Crossovers apply only to **output channels**. Master channels (`CH_MASTER_LEFT=0`, `CH_MASTER_RIGHT=1`) are pre-matrix-mixer and crossover is meaningless there.

- Vendor commands targeting crossover bands (band 20–23) on master channels (channel < `CH_OUT_1` = 2) are **rejected**.
- Bulk transfers send zeroed master rows for the crossover section on collect; bulk-apply skips master rows for the crossover section.
- The recipe array `xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS]` is symmetric with PEQ so apps, presets and the bulk wire format use the same channel indexing; its master rows are simply inert. The designed cascades (`xover_filters[NUM_OUTPUT_CHANNELS][MAX_XOVER_BANDS]`) and the `output_xover_bypassed` flags are output-indexed (`ch - CH_OUT_1`) and hold no master rows (since 2026-09-22).

---

## 2. Filter type enum

`FilterType` (defined in `config.h`) is shared between PEQ and crossover. The value space is partitioned (see the "FilterType value-space contract" comment in `config.h`):

- **0–7** core PEQ types: FLAT, PEAKING, LOWSHELF, HIGHSHELF, LOWPASS, HIGHPASS, NOTCH, ALLPASS (the second-order RBJ all-pass).
- **8** `FILTER_ALLPASS1` — first-order all-pass (a PEQ type). Flat magnitude, phase 0° → -180° (-90° at the corner freq); single parameter (`freq`), `Q` and `gain_db` unused.
- **9–10** `FILTER_LOWSHELF1`, `FILTER_HIGHSHELF1` — first-order shelves (PEQ types, added 2026-06-20). Gentle 6 dB/oct shelf, monotonic, no `Q`; parameters `freq` + `gain_db`.
- **11** `FILTER_LINKWITZ_TRANSFORM`: Linkwitz Transform (PEQ type, added 2026-07-12). Pole/zero bass-extension biquad; the only PEQ type needing four parameters, so it repurposes `gain_db` and carries a `Qp` sidecar. See [`peq_filters.md`](peq_filters.md).
- **12–13** `FILTER_LOWPASS1`, `FILTER_HIGHPASS1`: first-order low/high pass (PEQ types, added 2026-07-27). Single-pole 6 dB/oct rolloff, -3 dB at the corner, no resonance; single parameter (`freq`), `Q` and `gain_db` unused.
- **14–31** reserved for future PEQ types.
- **32–63** crossover types (table below).
- **64+** reserved for future crossover types; they must stay contiguous from `FILTER_XOVER_FIRST` because `xover_type_table[]` is indexed by `type - FILTER_XOVER_FIRST`.

> **Renumbering note (2026-06-17):** crossover types were shifted from the old 8–39 range to **32–63** to open a contiguous PEQ block (first-order all-pass added at 8, padding to 31). This is a breaking change to stored and wire type values: firmware bumped `SLOT_DATA_VERSION` to **18** (pre-V18 presets have their type values migrated on load) and `WIRE_FORMAT_VERSION` to **13**. A host must match its firmware version.

| Value | Symbol | Family | Order | Shape | Sections |
|---|---|---|---|---|---|
| 32 | `FILTER_LR2_LP` | Linkwitz-Riley | 2 | LP | 1 |
| 33 | `FILTER_LR2_HP` | Linkwitz-Riley | 2 | HP | 1 |
| 34 | `FILTER_LR4_LP` | Linkwitz-Riley | 4 | LP | 2 |
| 35 | `FILTER_LR4_HP` | Linkwitz-Riley | 4 | HP | 2 |
| 36 | `FILTER_LR6_LP` | Linkwitz-Riley | 6 | LP | 4 (2 first-order + 2 biquads) |
| 37 | `FILTER_LR6_HP` | Linkwitz-Riley | 6 | HP | 4 |
| 38 | `FILTER_LR8_LP` | Linkwitz-Riley | 8 | LP | 4 |
| 39 | `FILTER_LR8_HP` | Linkwitz-Riley | 8 | HP | 4 |
| 40 | `FILTER_BW1_LP` | Butterworth | 1 | LP | 1 (1st-order) |
| 41 | `FILTER_BW1_HP` | Butterworth | 1 | HP | 1 (1st-order) |
| 42 | `FILTER_BW2_LP` | Butterworth | 2 | LP | 1 |
| 43 | `FILTER_BW2_HP` | Butterworth | 2 | HP | 1 |
| 44 | `FILTER_BW3_LP` | Butterworth | 3 | LP | 2 (1st-order + biquad) |
| 45 | `FILTER_BW3_HP` | Butterworth | 3 | HP | 2 |
| 46 | `FILTER_BW4_LP` | Butterworth | 4 | LP | 2 |
| 47 | `FILTER_BW4_HP` | Butterworth | 4 | HP | 2 |
| 48 | `FILTER_BW5_LP` | Butterworth | 5 | LP | 3 (1st-order + 2 biquads) |
| 49 | `FILTER_BW5_HP` | Butterworth | 5 | HP | 3 |
| 50 | `FILTER_BW6_LP` | Butterworth | 6 | LP | 3 |
| 51 | `FILTER_BW6_HP` | Butterworth | 6 | HP | 3 |
| 52 | `FILTER_BW7_LP` | Butterworth | 7 | LP | 4 (1st-order + 3 biquads) |
| 53 | `FILTER_BW7_HP` | Butterworth | 7 | HP | 4 |
| 54 | `FILTER_BW8_LP` | Butterworth | 8 | LP | 4 |
| 55 | `FILTER_BW8_HP` | Butterworth | 8 | HP | 4 |
| 56 | `FILTER_BES2_LP` | Bessel | 2 | LP | 1 |
| 57 | `FILTER_BES2_HP` | Bessel | 2 | HP | 1 |
| 58 | `FILTER_BES4_LP` | Bessel | 4 | LP | 2 |
| 59 | `FILTER_BES4_HP` | Bessel | 4 | HP | 2 |
| 60 | `FILTER_BES6_LP` | Bessel | 6 | LP | 3 |
| 61 | `FILTER_BES6_HP` | Bessel | 6 | HP | 3 |
| 62 | `FILTER_BES8_LP` | Bessel | 8 | LP | 4 |
| 63 | `FILTER_BES8_HP` | Bessel | 8 | HP | 4 |

`FILTER_XOVER_FIRST` = 32, `FILTER_XOVER_LAST` = 63 (32 crossover filter types total). Any type outside this range written into a crossover band slot (bands 20–23) is treated as bypassed; the recipe round-trips the value so the app can detect the mismatch.

### Filter response conventions

- **Linkwitz-Riley** is the most common crossover family; LR2/LR4/LR6/LR8 sum to -6 dB at fc when paired (allpass-sum behavior). LR4 is the de-facto standard; LR6 is the canonical between LR4 and LR8 and is implemented as `(BW3)²` — its cascade includes two first-order sections plus two biquads, using the full 4-section per-band budget.
- **Butterworth** is maximally flat in passband; odd orders sum to allpass with the complementary HP/LP, even orders sum to +3 dB at fc.
- **Bessel** has approximately linear phase (small group-delay variation) but a wider crossover region. The pole tables use the -3 dB normalization convention (Rane / Williams reference) — cutoff frequency in vendor commands lands at the -3 dB point.

### LP → HP transform note (Bessel correctness)

The analog LP→HP transformation `s → ωc/s` inverts each pole through the unit circle. For a conjugate pair at `(-σ_n ± jω_n)` in the LP prototype, the corresponding HP pair is at `(-σ_n/r² ± jω_n/r²)` where `r² = σ_n² + ω_n²`. For Butterworth and Linkwitz-Riley, every prototype pole sits exactly on the unit circle (`r² = 1` by construction), so the reciprocal coincides with the original — only the numerator changes between LP and HP. For Bessel under the -3 dB normalization convention, `r²` is decidedly not 1 (it ranges roughly 1.6 .. 4.6 across orders 2-8 and across sections within an order), so the reciprocal scaling is mandatory: the HP version of a Bessel filter has different pole locations from the LP version, with different section natural frequencies and the same Qs. The firmware applies the reciprocal unconditionally in `section_emit_2nd_order()` and `section_emit_1st_order()` — it's a no-op for Butterworth/LR and the correct transform for Bessel.

### Bandpass

There is no "bandpass" filter type. A bandpass output uses two crossover bands on the same channel — one HP and one LP at different fc values.

---

## 3. Vendor commands

Crossover bands use the **existing band-addressing vendor commands**. No new commands are introduced. Apps that already implement PEQ band SET/GET work with crossover bands by passing band indices 20–23.

> **Wire-protocol change (2026-06-04):** crossover moved from bands 12–15 to **20–23**. Because `REQ_GET_EQ_PARAM` previously packed the band into a 4-bit `wValue` field (max 15), its `wValue` layout was widened to a 5-bit band field. See [3.2 GET commands](#32-get-commands). `REQ_SET_EQ_PARAM`, `REQ_SET_BAND_BYPASS`, `REQ_GET_BAND_BYPASS`, and the bulk transfer carry the band in a full 8-bit field and were unaffected.

### 3.1 SET commands

| Request | Code | wValue | Payload | Effect |
|---|---|---|---|---|
| `REQ_SET_EQ_PARAM` | 0x42 | (unused) | 16-byte `EqParamPacket` | Set full band recipe |
| `REQ_SET_BAND_BYPASS` | 0xD8 | (channel << 8) \| band | 1 byte: 1 = bypass, anything else = active | Toggle band bypass |

`EqParamPacket` layout (16 bytes, packed):

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | uint8_t | channel | 0..NUM_CHANNELS-1 |
| 1 | uint8_t | band | 0..9 (PEQ) or 20..23 (crossover) |
| 2 | uint8_t | type | `FilterType` enum |
| 3 | uint8_t | bypass | 1 = bypassed (strict); any other value = active |
| 4 | float | freq | Hz, clamped to [10, 0.45·Fs] |
| 8 | float | Q | 0.1..20.0 (ignored for crossover types) |
| 12 | float | gain_db | -120..+80 dB (ignored for crossover types) |

Validation rules at the SET handler:
- `channel >= NUM_CHANNELS` → reject
- `band` in active PEQ range (0..`channel_band_counts[ch]`-1) → accepted as PEQ
- `band` in crossover range (20..23) → accepted only if `channel >= CH_OUT_1` (== 2)
- `band` in reserved range (10..19) → reject silently
- `band` ≥ 24 → reject silently
- `bypass` byte normalized: strictly equal to 1 = bypassed, else active (defends against 0xFF padding from legacy hosts)
- For SET_BAND_BYPASS: the handler reads the current recipe from storage, overwrites the bypass byte, normalizes channel/band to the wire values, and queues the update

### 3.2 GET commands

| Request | Code | wValue | Returns |
|---|---|---|---|
| `REQ_GET_EQ_PARAM` | 0x43 | (channel << 8) \| (band << 3) \| param | 4 bytes (scalar value) |
| `REQ_GET_BAND_BYPASS` | 0xD9 | (channel << 8) \| band | 1 byte (0 or 1) |

**`REQ_GET_EQ_PARAM` wValue encoding (changed 2026-06-04):** the band field is now **5 bits** so crossover bands (20–23) remain addressable after the gap widened. The `param` selector is **3 bits** (only 5 values exist). Bit layout:

| Bits | Field | Range |
|---|---|---|
| 15..8 | channel | 0..NUM_CHANNELS-1 |
| 7..3 | band | 0..31 (PEQ 0..9, crossover 20..23) |
| 2..0 | param | 0..4 |

Build it as `wValue = (channel << 8) | (band << 3) | param`. (Previously band occupied bits 7..4 and param bits 3..0; hosts that addressed PEQ bands 0–9 with the old layout MUST update.) `REQ_GET_BAND_BYPASS` is unchanged: it carries the band in the low 8 bits.

`REQ_GET_EQ_PARAM` returns a single scalar selected by `param`:

| param | Returned scalar |
|---|---|
| 0 | `type` (uint32_t, zero-padded) |
| 1 | `freq` (float) |
| 2 | `Q` (float, irrelevant for crossover types) |
| 3 | `gain_db` (float, irrelevant for crossover types) |
| 4 | `bypass` (uint32_t, 0 or 1) |

Note: `REQ_GET_EQ_PARAM` does NOT round-trip the `band` field; the band is implicit in `wValue`. To verify the stored `band` field, use the bulk transfer.

GET validation mirrors SET: master channels with crossover bands return a USB protocol stall.

### 3.3 Vendor command example: configure a 2-way crossover at 2 kHz

For an output pair (CH_OUT_3 = 4 as woofer, CH_OUT_4 = 5 as tweeter):

```
// Woofer: LR4 LP at 2000 Hz, band 20 on channel 4
EqParamPacket woofer = { .channel=4, .band=20, .type=FILTER_LR4_LP, .bypass=0,
                         .freq=2000.0f, .Q=0.707f, .gain_db=0.0f };
SET REQ_SET_EQ_PARAM payload=woofer

// Tweeter: LR4 HP at 2000 Hz, band 20 on channel 5
EqParamPacket tweeter = { .channel=5, .band=20, .type=FILTER_LR4_HP, .bypass=0,
                          .freq=2000.0f, .Q=0.707f, .gain_db=0.0f };
SET REQ_SET_EQ_PARAM payload=tweeter
```

That's it. The crossover stage is now active. To disable, send `REQ_SET_BAND_BYPASS` with bypass=1.

---

## 4. Bulk transfer (`WireBulkParams` V11)

The bulk parameter transfer includes crossover state in a dedicated section appended to `WireBulkParams`. PEQ and crossover are kept in separate sections at the wire-format level (even though they share band-index addressing at the vendor-command level) so existing offsets in `eq[][]` and notification offsets remain stable.

### 4.1 V11 wire format additions

```c
#define WIRE_MAX_XOVER_BANDS  4
#define WIRE_FORMAT_VERSION   11

typedef struct __attribute__((packed)) {
    WireBandParams bands[WIRE_MAX_CHANNELS][WIRE_MAX_XOVER_BANDS];  // 11×4×16 = 704 bytes
} WireCrossoverConfig;

typedef struct __attribute__((packed)) {
    // ... existing V10 sections ...
    WireDacHwMute       dac_hw_mute;
    WireCrossoverConfig crossovers;  // NEW in V11 — 704 bytes
} WireBulkParams;
```

`WireBandParams` is the same 16-byte struct PEQ uses (type, bypass, reserved[2], freq, q, gain_db).

Total payload size:
- V10: 2960 bytes
- V11: 3664 bytes (+704)

### 4.2 Per-version size anchors

Apps that target multiple firmware versions read the host's `format_version` field and trust the payload sizes below:

| Version | Total payload size |
|---|---|
| V2 | `WIRE_BULK_PARAMS_V2_SIZE` (smallest accepted) |
| V3..V5 | V2_SIZE + WireI2SConfig + WireLevellerConfig |
| V6..V7 | V5_SIZE + WirePreampConfig + WireMasterVolume + WireInputConfig |
| V8 | V7_SIZE + WireLgSoundSync |
| V9 | V8_SIZE + WireUserVolume |
| V10 | V9_SIZE + WireDacHwMute = 2960 |
| V11 | V10_SIZE + WireCrossoverConfig = 3664 |

The firmware accepts any payload length between `WIRE_BULK_PARAMS_MIN_SIZE` (= V2 size) and `sizeof(WireBulkParams)` (V11) — older payloads simply leave the tail sections at current state.

### 4.3 V11 vs V<11 SET semantics

| Sender version | Crossover behavior on SET |
|---|---|
| V11 | Crossover section applied; master rows skipped |
| V<11 | Current crossover state preserved (V10 senders don't know crossover exists, so don't clobber) |

V<11 senders can still write all of their known sections (DAC mute, LG, etc.) — those gates now compare against per-version size anchors, not against the full `sizeof(WireBulkParams)`. This means a V10 app does not lose its DAC-mute section just because firmware grew to V11.

### 4.4 V11 GET payload

Firmware always returns V11 payload. The `crossovers` section is populated with live crossover state; master rows (channels 0..1) are **zeroed**. App rendering should hide crossover controls for master channels.

### 4.5 Live-edit notifications

When a host SET writes a crossover band, the firmware emits a `PARAM_CHANGED` notification (interrupt EP 0x83) whose `wire_offset` points into the `WireCrossoverConfig` section, NOT into `eq[]`. Apps that mirror the wire format use the offset to discriminate the two.

Offset formula for `crossovers.bands[ch][local_band]`:
```
off = offsetof(WireBulkParams, crossovers)
    + offsetof(WireCrossoverConfig, bands)
    + (ch * WIRE_MAX_XOVER_BANDS + local_band) * sizeof(WireBandParams)
```
where `local_band = band - XOVER_BAND_BASE` (i.e., 0..3 for wire band indices 20..23).

---

## 5. Persistence (`PresetSlot` V16)

### 5.1 V16 slot layout

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;       // SLOT_MAGIC = 'DSP3'
    uint16_t version;     // 16
    uint16_t slot_index;
    uint32_t crc32;
    // ... data section (V15 fields unchanged) ...
    uint8_t  user_vol_index;        // last V15 field
    EqParamPacket xover_recipes[NUM_CHANNELS][MAX_XOVER_BANDS];  // V16 addition
} PresetSlot;
```

### 5.2 Version-aware CRC

V16 grew the on-disk struct, so CRC validation cannot use `sizeof(PresetSlot)` for older versions. The validator looks up the right byte range per version:

```c
size_t slot_data_size_for_version(uint8_t version) {
    switch (version) {
        case 12: case 13: case 14: case 15:
            return sizeof(PresetSlot)
                 - offsetof(PresetSlot, filter_recipes)
                 - sizeof(((PresetSlot *)0)->xover_recipes);
        case 16:
            return sizeof(PresetSlot)
                 - offsetof(PresetSlot, filter_recipes);
        default:
            return 0;  // unsupported / unknown → invalidate
    }
}
```

V12–V15 all share the same on-disk size (V13/V14/V15 each consumed reserved bytes of V12 without growing the struct). Pre-V12 versions are already invalidated by current firmware (their on-disk size differs from V12's) and remain rejected by the new helper.

### 5.3 Behavior on load

- V16 slot → `xover_recipes` copied from slot. The `band` field is re-normalized to `XOVER_BAND_BASE+i` (20..23) on load defensively. Slots saved by firmware that used the old base (12..15) are harmless: the value is overwritten on load, and `xover_recipes` storage is indexed by array position, not by the stored band field.
- V12–V15 slot → `xover_recipes` initialized to defaults (FLAT type, fc=1000, Q=0.707, gain=0, band=XOVER_BAND_BASE+i) via `xover_init_default_filters()`.
- Pre-V12 slot → CRC invalidation; firmware falls back to factory defaults.

### 5.4 Legacy migration

The single-sector legacy magic format (`DSP1`) predates the directory-based preset system. The migration path (`migrate_legacy()`) produces a **V16 slot** — the migrated slot's `version` field is set to `SLOT_DATA_VERSION = 16` regardless of the legacy version that was on disk. This is required because the new CRC validator looks up byte range by version; producing a non-V16 slot would CRC-mismatch on next boot.

**Field-default discipline:** Because the migrated slot is tagged V16, every `slot->version >= N` gate in `apply_slot_to_live()` fires and reads the corresponding field directly from the slot — rather than falling through to its default-init branch. So migrate must populate *every* post-legacy field (V8–V16) with the value the V<N default branch would have produced. Specifically:

- **V8 channel names:** computed via `get_default_channel_name(ch, INPUT_SOURCE_USB, NULL, ...)`. Without this, names would memcpy as blanks.
- **V9 I2S config:** `i2s_bck_pin = PICO_I2S_BCK_PIN`, `i2s_mck_pin = PICO_I2S_MCK_PIN`, MCK disabled, 128× multiplier. Without this, MCK pin would be 0 (a fallback would re-default it, but the path is messier).
- **V10 leveller:** `LEVELLER_DEFAULT_*` for every field. Without this, leveller would appear disabled with zero amount — same effect, but unprincipled.
- **V12 preamp_db_per_ch / master_volume_db:** `preamp_db_per_ch[i] = slot->preamp_db` (the legacy single value applied to every channel — matches the V<12 apply path); master volume = 0 dB unity (dormant under the default master-volume-mode=independent setup).
- **V13 input_source / spdif_rx_pin:** USB / 0 (spdif_rx_pin=0 falls back to live default on apply).
- **V14 lg_sound_sync_enabled:** 0 (disabled).
- **V15 user_vol_index:** **CRITICAL** — must be `CENTER_VOLUME_INDEX`, not 0. The vol-index → dB mapping is `db = idx - CENTER_VOLUME_INDEX`, so idx=0 would slam the user volume to `-CENTER_VOLUME_INDEX` dB (effectively full mute) on the first boot after upgrade.
- **V16 xover_recipes:** FLAT type, fc=1000 Hz, Q=0.707, gain_db=0, **band = XOVER_BAND_BASE+i** (wire index 20..23) per the band-field-normalization invariant.

This discipline is enforced inline in `migrate_legacy()` — see the code for the canonical list. Future fields added past V16 must extend the same block.

### 5.5 Factory reset

`apply_factory_defaults()` calls `dsp_init_default_filters()`, which calls `xover_init_default_filters()`. All crossover bands return to defaults (FLAT, 1000 Hz, band-field = 20+i).

---

## 6. Pipeline position

Per-output processing order (both platforms, all dispatch paths):

```
input decode → preamp → loudness → master EQ → leveller → crossfeed
            → matrix mixer
            → CROSSOVER STAGE          ← new in V11
            → per-output PEQ
            → output gain ramp
            → delay line
            → SPDIF/I2S/PDM encoding
            → DMA output
```

The crossover stage is in-place on the existing `buf_out[output][]` arrays. It does not alter buffer sizes, sample counts, DMA configuration, PIO state, or any pacing variable — output slot sample alignment is preserved (the project's hard rule from `CLAUDE.md`).

Core 1 EQ worker (when active — see CPU section): runs crossover before PEQ on its assigned output range.

**Global EQ bypass (`REQ_SET_BYPASS`) intentionally does NOT bypass the crossover stage.** Crossover filters are speaker-protection critical: a tweeter's high-pass must survive a global "EQ off" toggle, or the comparison listen could damage the driver. Only per-band bypass (`REQ_SET_BAND_BYPASS` or the recipe's `bypass` byte) and the per-output `output_xover_bypassed` fast path disable crossover processing.

---

## 7. Defaults

| Property | Default value |
|---|---|
| All 4 bands per channel | `bypass = false`, `type = FILTER_FLAT`, `freq = 1000.0 Hz`, `Q = 0.707`, `gain_db = 0.0` |
| `band` field in each recipe | `XOVER_BAND_BASE + i` (i.e., 20, 21, 22, 23; the wire band index) |
| Per-output `output_xover_bypassed` flag | `true` (fast-path: stage is skipped entirely when no band is active) |

Because `FILTER_FLAT` is not in the crossover range, every default band is automatically bypassed by the design path. The user must explicitly pick a crossover filter type to engage the band.

---

## 8. Edge cases and validation

- **Frequency clamp:** `[10 Hz, 0.45 × Fs]`. Out-of-range values are clamped silently at coefficient computation time. Same convention as PEQ.
- **Non-crossover type in crossover slot:** if the app writes a PEQ type (0..7) into band 20..23, the recipe stores the value (round-trips on GET) but the design routine produces a bypassed section. The band has no audible effect.
- **Crossover type accidentally written to PEQ slot (band 0..9):** symmetric defense — `dsp_compute_coefficients()` treats any type outside the PEQ enum range (`> FILTER_ALLPASS`) as bypassed via the `is_filter_flat()` predicate. Without this guard, an unknown type would leave the RP2350 SVF output-mix coefficients zeroed (`svm0 = svm1 = svm2 = 0`) and the band would output silence at low fc. The recipe round-trips on GET so apps can detect the misconfiguration; the band is bypassed at compute time so audio passes through unaffected.
- **Type ≥ FILTER_XOVER_LAST + 1:** treated as bypassed.
- **`fc <= 0` or `Fs <= 0`:** treated as bypassed.
- **Bypass byte normalization:** the firmware accepts only `bypass == 1` as "bypassed". Any other value (including 0xFF from legacy hosts that fail to zero-init padding) is treated as active. Apps should always send `bypass = 0` or `1`.
- **Rate change:** all crossover sections are redesigned at the new Fs (in `dsp_recalculate_all_filters()`). Section state (`s1`, `s2`, `svic1eq`, `svic2eq`) is preserved across the redesign and reset only when a section's SVF/TDF2 path changes (e.g. fc crossing the Fs/7.5 gate at the new rate), when a section drops out of the new cascade, or when the band toggles bypass. A small transient from the coefficient step itself is still possible; the PEQ stage has the same behavior, and the preset-mute envelope does NOT cover rate change today (consistent with existing PEQ limitation).
- **Live edit (REQ_SET_EQ_PARAM with same band twice):** each edit redesigns just that band's sections, then recomputes `output_xover_bypassed[ch - CH_OUT_1]`. Section state is preserved across the redesign (matching the PEQ convention in `dsp_compute_coefficients()`), so dragging fc or switching order does not zero the cascade's memory; any residual transient comes from the coefficient step, as with PEQ.
- **Reserved band indices 10..19:** rejected at the vendor handler.
- **Crossover on master channels:** rejected. Use output channels (CH_OUT_1 and above) only.

---

## 9. CPU and BSS impact

### BSS

| Resource | RP2040 (7 channels) | RP2350 (11 channels) |
|---|---|---|
| `xover_filters[NUM_OUTPUT_CHANNELS][4]` (output-indexed since 2026-09-22) | 2.6 KB | 10.3 KB |
| `xover_recipes[NUM_CHANNELS][4]` | 0.45 KB | 0.70 KB |
| `notify_rebaseline` static scratch (V11 size growth) | 0.7 KB | 0.7 KB |
| **Total added** | **~5 KB** | **~15.4 KB** |

### CPU (worst case at 48 kHz)

| Configuration | RP2040 (200 MHz, 1 core after PDM dispatch) | RP2350 (150 MHz, 1 or 2 cores) |
|---|---|---|
| All bypassed (default) | ~0% | ~0% |
| 1 LR4 band on each output | ~10% | ~6% |
| Realistic 2-way active speaker config | ~15% | ~10% |
| **Worst case: 4 LR8 bands on every output** | ~58% (single-core, PDM-active) | ~35% (Core 0 alone) |

Add this to existing PEQ, master EQ, loudness, leveller, crossfeed, matrix mixer, output encoding loads — worst case on RP2040 with PDM active approaches the limit. **Recommendation:** when PDM is enabled on RP2040, limit each output to ≤ 2 LR8 bands or use lower orders (LR4, BW4) for tighter CPU budget.

### Core 1 dispatch

- **RP2350:** Core 1 EQ worker is active when no PDM output is enabled. Crossover runs on Core 1 for outputs `CORE1_EQ_FIRST_OUTPUT` (= 2) through `CORE1_EQ_LAST_OUTPUT` (= 7).
- **RP2040:** Same pattern, Core 1 owns outputs 2–3 when PDM is disabled.
- **When PDM is enabled (either platform):** Core 1 is in PDM mode; ALL crossover (and PEQ) runs on Core 0.

---

## 10. Known limitations

- **BW8 on RP2040 (Q28 fixed-point):** Cascaded 8th-order Butterworth sections can transiently push internal state past 2× unity for full-scale impulses (kick drum, snare hit). Theoretically representable in Q28 but borderline; avoid feeding 0 dBFS transients into a BW8 stage on RP2040, or use LR4 / BW4 which have lower internal-state gain peaks. The RP2350 float path has no such concern.
- **Bessel section frequency above Fs/7.5:** the SVF/TDF2 selection boundary is keyed on the user's fc, not on each section's effective frequency. Bessel sections have radii up to 2.157 (Bes8 outer pair), so at fc near Fs/7.5 some sections operate above the SVF "comfortable" band. They remain numerically correct; the SVF advantage over TDF2 just dissipates. Use BW or LR families if you want SVF-driven precision at high fc on RP2350.
- **Rate-change click:** crossover state resets across sample-rate changes (same as PEQ). The preset-mute envelope does not cover rate change today.
- **Live-edit click:** changing a band's `freq` or `type` mid-stream produces a small click as the section state mismatches the new coefficients. Same as live PEQ edits. Use SET-then-bypass-toggle if you need a click-free transition.
- **Master-channel rows:** stored but never processed. Apps shouldn't display crossover controls for CH_MASTER_*.

---

## 11. App-developer walkthrough

### 11.1 Discovering crossover support

Read the bulk transfer (`REQ_GET_ALL_PARAMS`). Inspect `header.format_version`:
- V11+ → device supports crossover; read `crossovers` section
- V<11 → device predates crossover; the section doesn't exist

### 11.2 Reading current crossover state

Two paths:

**Bulk read (preferred for app startup):** `REQ_GET_ALL_PARAMS` returns the full state in one transfer; index into `crossovers.bands[channel][local_band]` (`local_band = wire_band - 20`).

**Per-band scalar reads:** `REQ_GET_EQ_PARAM` with `wValue = (channel<<8) | (band<<4) | param_nibble` returns 4 bytes per call. Issue one call per parameter (type/freq/Q/gain_db/bypass) per band per channel — slower but useful for incremental UI updates.

### 11.3 Writing crossover state

**Per-band SET:** `REQ_SET_EQ_PARAM` with a 16-byte `EqParamPacket`. Set `channel`, `band` (20–23), `type` (one of `FILTER_*` crossover values), `freq`, and `bypass`. `Q` and `gain_db` can be 0; the firmware ignores them.

**Bypass-only SET:** `REQ_SET_BAND_BYPASS` with `wValue = (channel<<8) | band` and a 1-byte payload (0 = active, 1 = bypassed). Faster than a full SET for toggle UI elements.

**Bulk SET:** assemble a `WireBulkParams` payload at V11 size, fill the `crossovers` section, send via `REQ_SET_ALL_PARAMS`. Master rows in `crossovers` are skipped by the firmware so you can zero them.

### 11.4 Saving to a preset

`REQ_PRESET_SAVE` with `wValue = slot_index` captures the current live state (including crossover) into the named slot. Preset slots store all crossover state per channel per band.

### 11.5 Common configurations

**2-way active speaker (woofer + tweeter, 1 channel per driver):**
- Woofer (e.g., CH_OUT_3 = 4): LR4 LP at 2 kHz, band 20
- Tweeter (e.g., CH_OUT_4 = 5): LR4 HP at 2 kHz, band 20

**3-way active speaker:**
- Woofer: LR4 LP at 300 Hz, band 20
- Midrange: LR4 HP at 300 Hz, band 20 + LR4 LP at 3 kHz, band 21
- Tweeter: LR4 HP at 3 kHz, band 20

**Subwoofer integration (sub gets LP, mains get HP):**
- Sub output: LR4 LP at 80 Hz, band 20
- Main outputs: LR4 HP at 80 Hz, band 20

### 11.6 UI hints

- Hide crossover controls for master channels (CH_MASTER_LEFT / CH_MASTER_RIGHT).
- Hide bands 10–19 from the band picker (they're reserved).
- "Filter type" picker for crossover bands shows only the 32 crossover values (indices 32..63). PEQ bands should show only PEQ values (0..10, where 8 is the first-order all-pass and 9..10 are the first-order shelves).
- When loading a preset, the app can detect crossover state by checking `xover_recipes[ch][i].type != FILTER_FLAT && xover_recipes[ch][i].bypass != 1` for each band.

---

## 12. References

- **Filter design:** R. Bristow-Johnson, "Audio EQ Cookbook" (RBJ) for the bilinear-transform formulas used in PEQ. Crossover sections use a similar bilinear approach driven from analog pole locations.
- **Linkwitz-Riley:** S. Linkwitz, "Active Crossover Networks for Noncoincident Drivers", JAES 1976.
- **Butterworth pole locations:** standard formula `s_k = exp(j·π·(2k+N−1)/(2N))` for k=1..N (left half plane, conjugate-paired).
- **Bessel pole tables (-3 dB normalized):** Williams & Taylor, "Electronic Filter Design Handbook" 4th ed., cross-referenced against Analog Devices MT-201.
- **SVF (Cytomic TPT form):** A. Simper, "Linear Trapezoidal State Variable Filter" 2013 (used on RP2350 for sections where fc < Fs/7.5).
- **Pipeline order:** miniDSP digital-crossover application notes; Rane "Linkwitz-Riley Crossovers and Why" technical brief.

---

## 13. Version history

| Doc version | Date | Change |
|---|---|---|
| 1.0 | 2026-05-18 | Initial spec; V16 preset slot, V11 wire format, FilterType extensions 8..37 |
| 1.1 | 2026-06-04 | Crossover wire band base moved 12 → 20 (reserved gap widened to 10..19, `XOVER_BAND_BASE`); `REQ_GET_EQ_PARAM` wValue re-encoded to a 5-bit band field (`(channel<<8)\|(band<<3)\|param`). No change to wire-format size, persistence layout, or filter-type enum. |
