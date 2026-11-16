# Per-Band Bypass — Implementation Spec for Host Apps

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7)*
*Firmware version introduced: 1.1.4 (post-master-volume)*
*Both platforms (RP2040 and RP2350)*

This spec is for app/host developers who need to read, write, persist, and display the per-band bypass control. It is intentionally over-explanatory; if something here surprises you, the spec is wrong — file an issue.

---

## 1. What it is

Each EQ band on every channel has a user-controllable **bypass** flag. When set, that single band is removed from the signal path with sample-accurate transition (the audio inner loop simply skips it). When all bands on a channel are bypassed, the entire channel skips EQ processing.

Per-band bypass is **distinct** from:
- **Master EQ bypass** (`FILTER.EQ_BYPASS` / `bypass_master_eq`). It bypasses *all* EQ on *all* channels, a single global toggle.
- **Auto-bypass** of `FILTER_FLAT` filters or peaking/shelf filters with `|gain_db| < 0.01` — done internally by `dsp_compute_coefficients()` to save CPU. Auto-bypass and user-bypass share the same internal `Biquad.bypass` flag, so neither overrides the other: if either is true, the band is skipped.

The user's freq/Q/gain/type values are **preserved** when a band is bypassed — un-bypassing returns the band exactly to its previous response.

---

## 2. Wire formats

### 2.1 `FILTER.BAND` payload (vendor SET and GET)

The `FILTER.BAND` command carries one band in 16 bytes. The channel and band are in `wValue` as `(channel << 8) | band`, not in the payload.

```c
// FILTER.BAND SET payload and GET reply, 16 bytes, little-endian
uint8_t  type;       // FilterType enum
uint8_t  bypass;     // 1 = bypassed, anything else = active. SEE 0xFF SAFETY.
uint16_t qp_x512;    // Linkwitz Transform target Q (peq_filters.md), ignored by other types
float    freq;       // Hz
float    Q;          // 0.1 .. 20
float    gain_db;    // dB
```

The bypass byte sits at **offset 1**, the same place as in `WireBandParams` (2.2). The two layouts are byte-identical apart from the meaning of bytes 2-3.

### 2.1a `EqParamPacket` (firmware recipe, also stored in flash presets)

The firmware keeps each band as an `EqParamPacket`. It is not on the vendor wire any more, but it is the preset-slot storage format:

```c
typedef struct __attribute__((packed)) {
    uint8_t channel;
    uint8_t band;
    uint8_t type;       // FilterType enum
    uint8_t bypass;     // 1 = bypassed, anything else = active. SEE 0xFF SAFETY.
    float   freq;       // Hz
    float   Q;          // 0.1 .. 20
    float   gain_db;    // dB
} EqParamPacket;        // 16 bytes
```

Here the bypass byte sits at **offset 3**. This is the same byte that older firmware called `reserved`. See **§5 0xFF Safety** for the contract.

### 2.2 `WireBandParams` (bulk-params payload, `STORE.ALL_PARAMS`)

```c
typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  bypass;        // 1 = bypassed, anything else = active
    uint8_t  reserved[2];   // Must be zero
    float    freq;
    float    q;
    float    gain_db;
} WireBandParams;            // 16 bytes
```

Note that `WireBandParams` is **not** identical in layout to `EqParamPacket`:
- `EqParamPacket` carries `channel` + `band` (the recipe's coordinates) in bytes 0-1.
- `WireBandParams` omits those because the position in the bulk array (`eq[channel][band]`) implies them.

Both structs put `bypass` immediately before the floats. **Do not** assume one is a memcpy-equivalent of the other — they're not.

### 2.3 Position in the bulk-params block

`WireBulkParams.eq[ch][b]` is a `WIRE_MAX_CHANNELS × WIRE_MAX_BANDS` row-major array of `WireBandParams`:

```
WIRE_MAX_CHANNELS = 12
WIRE_MAX_BANDS    = 12
sizeof(WireBandParams)            = 16
offsetof(WireBulkParams, eq)      = 360
sizeof(eq[][]) total              = 2304   (12 × 12 × 16; firmware fills MAX_BANDS=12)
```

The byte offset of any single bypass byte inside the bulk packet is therefore:
```c
360 + (ch * 12 + b) * 16 + 1
```
That offset value is also exactly what the firmware sends on the **device→host notification endpoint** when a band changes (see §6.4).

**No `WIRE_FORMAT_VERSION` bump.** This change is *byte-for-byte* layout-compatible with V7. If you sent zero in the old `reserved[3]` you'll send zero in the new `bypass`+`reserved[2]` — no behavioral change. New hosts simply learn to populate the byte.

---

## 3. Vendor commands

All are paged control transfers on the vendor interface: the page goes in the `wIndex` high byte, the command in `bRequest`, and `wValue` is the address (see `../commands.md`). A rejected SET STALLs, and the reason is readable from `SYS.LAST_RESULT`.

| Command | Page | Cmd | Dir | Address (`wValue`) | Size | Notes |
|---|---|---|---|---|---|---|
| `FILTER.BAND` | 0x06 | 0x10 | GET, SET | `(channel << 8) \| band` | GET 16, SET 16 | Whole band (2.1). The `bypass` byte rides along. A GET returns bypass as 0 or 1 |
| `FILTER.BAND_BYPASS` | 0x06 | 0x11 | GET, SET | `(channel << 8) \| band` | GET 1, SET 1 | `1` = bypass, anything else = active. Preserves freq/Q/gain/qp. A GET returns 0 or 1 |
| `FILTER.EQ_BYPASS` | 0x06 | 0x12 | GET, SET | none (0) | GET 1, SET 1 | Master EQ bypass, non-zero bypasses |
| `STORE.ALL_PARAMS` | 0x02 | 0x40 | GET, SET | none (0) | GET 6136, SET 6136 | Bulk snapshot. GET bypass bytes are normalized 0/1. SET intake normalizes `!=1` to 0 |

A channel past the end, a band in the reserved gap, or a crossover band on an input channel is refused with `VC_ERR_ADDRESS` in both directions. `FILTER.BAND` and `FILTER.BAND_BYPASS` share one pending slot. A SET of either that arrives while an earlier one is still waiting for the main loop is refused with `VC_ERR_BUSY` and changes nothing, so retry after a short delay.

### 3.1 `FILTER.BAND` (recommended for full-band updates)

If your host already sends a full band for every UI change, just populate the `bypass` field. The firmware will:
1. Copy the band into `pending_packet`.
2. Normalize: `pending_packet.bypass = (bypass == 1) ? 1 : 0`.
3. Defer to the main loop, which calls `dsp_compute_coefficients()` and updates `channel_bypassed[ch]`.
4. Emit a notify-shadow update so other connected hosts see the new state.

### 3.2 `FILTER.BAND_BYPASS` SET (recommended for toggle UI)

Cheaper round-trip when the user is just clicking a "bypass" button:
- 1-byte payload vs. 16-byte packet.
- Preserves whatever freq/Q/gain/qp are currently in the firmware. There is no risk of clobbering stored parameters.
- The firmware reads its own current recipe, sets `bypass`, and dispatches through the same eq_update_pending path. All the same Core 1 synchronization, channel_bypassed recalc, and notify-shadow logic runs.

### 3.3 `FILTER.BAND_BYPASS` GET (explicit single-byte query)

When you only need to display the bypass state, this avoids a 16-byte transfer. Always returns exactly `0` or `1` (the firmware normalizes), so you can blindly cast to `bool`.

---

## 4. Preset persistence

Per-band bypass is **automatically saved and restored** with the active preset. There is **no opt-in flag** (unlike include_pins or master-volume mode) — it is always part of a preset, just like freq/Q/gain.

- The byte lives inside `PresetSlot.filter_recipes[ch][b]`, which is `memcpy`'d to/from flash.
- Legacy presets (any prior firmware version) had `0` in the corresponding byte → they load with no bands bypassed → behavior identical to old firmware.
- `apply_slot_to_live()` (flash_storage.c) explicitly re-normalizes the byte after `memcpy`, so a corrupted flash byte cannot cause unintended bypass.
- No `SLOT_DATA_VERSION` change required (V12 still).

### 4.1 Factory reset

Factory reset re-initializes `filter_recipes` from `dsp_init_default_filters()`, which writes every field of every band, `bypass = 0` included. Every path that falls back to defaults goes through it: factory reset, loading or deleting an empty active slot, and booting with no saved preset. All bands therefore come back active. Firmware up to and including v1.1.6-beta5 did not write `bypass` here, so a band bypassed earlier in the session stayed bypassed after any of those resets. If you maintain a host-side "preset" cache, reset your cached `bypass[]` to all-zeros on a factory-reset notification.

---

## 5. The 0xFF safety contract

**The contract:** the firmware treats the bypass byte as bypassed **if and only if** its value is exactly `0x01`. Every other value — `0x00`, `0xFF`, `0x42`, garbage from uninitialized stack, anything — leaves the band active.

### Why this matters

The bypass byte (offset 1 of the `FILTER.BAND` payload and of `WireBandParams`, offset 3 of `EqParamPacket`) was named `reserved` in earlier firmware. Some host implementations may:
- `memset(&pkt, 0xFF, sizeof(pkt))` and then populate fields, leaving `reserved` as `0xFF`.
- Send legacy structs over the wire that happen to have non-zero garbage at that offset.
- Pass through bytes from another protocol layer that uses `0xFF` as a "missing" sentinel.

If the firmware accepted "any non-zero value means bypass," any of those scenarios would silently bypass random bands the moment the host upgrades to a firmware that knows about the byte. The strict `== 1` rule makes the upgrade safe with no host changes required.

### What this means for your host code

**Reading bypass state:**
- `FILTER.BAND_BYPASS` GET always returns `0` or `1`. Treat as boolean.
- The bypass byte (offset 1) of a `FILTER.BAND` GET is always `0` or `1`.
- `WireBandParams.bypass` from `STORE.ALL_PARAMS` GET is always `0` or `1` (firmware normalizes during collect).

**Writing bypass state:**
- Write **`0`** for active and **`1`** for bypass. Never write `0xFF` or any other value to mean "bypass". The firmware will treat it as active and you'll get a confusing GET that returns `0`.
- When constructing a `FILTER.BAND` payload or a `WireBandParams`, **explicitly set the bypass byte** rather than relying on a memset. Example C:
  ```c
  uint8_t pkt[16] = {0};     // good: bypass starts at 0 (active)
  pkt[0] = FILTER_PEAKING;   // type
  pkt[1] = ui_state.bypassed ? 1 : 0;  // bypass, explicit
  // pkt[2..3] qp_x512, pkt[4..15] freq, Q, gain_db as f32 LE
  // wValue = (ch << 8) | b
  ...
  ```
  Avoid `memset(&pkt, 0xFF, sizeof(pkt))`. You'll then need to remember to clear `bypass` explicitly, and the safer pattern is just to start from zero.
- **Bulk SET (`STORE.ALL_PARAMS`):** always set `WireBandParams.reserved[2]` to `{0, 0}`, except that a Linkwitz Transform band carries its `qp` there (`peq_filters.md`).

**Cross-protocol guards (Python / Rust / TS examples):**
```python
def encode_bypass(b: bool) -> int:
    return 1 if b else 0      # never return any other value

def decode_bypass(byte: int) -> bool:
    return byte == 1           # mirrors firmware's strict ==1 rule
```

```rust
fn encode_bypass(b: bool) -> u8 { if b { 1 } else { 0 } }
fn decode_bypass(byte: u8) -> bool { byte == 1 }
```

```typescript
const encodeBypass = (b: boolean): number => (b ? 1 : 0);
const decodeBypass = (byte: number): boolean => byte === 1;
```

---

## 6. Host-app integration patterns

### 6.1 Initial load on connect

```
1. STORE.ALL_PARAMS GET  → parse into local cache.
2. For each (ch, b):
       cache.bands[ch][b].bypass = (wire.eq[ch][b].bypass == 1);
3. Render UI from cache.
```

### 6.2 User toggles a single band's bypass

```
1. cache.bands[ch][b].bypass = !cache.bands[ch][b].bypass;
2. FILTER.BAND_BYPASS SET  wValue=(ch<<8)|band  payload=cache ? 1 : 0
   (on VC_ERR_BUSY, retry shortly)
3. (Optional) wait for the notify echo to confirm; or trust the SET and update UI immediately.
```

The firmware will internally:
- Update `filter_recipes[ch][b].bypass`.
- Recompute `filters[ch][b]` (sets `Biquad.bypass = true`, zeros coefficients).
- Walk all bands on the channel and update `channel_bypassed[ch]`.
- Send a `WireBandParams` notify-shadow to every connected host (including the originator) so other UIs stay in sync.

### 6.3 User edits freq/Q/gain on an already-bypassed band

You have two valid choices:
- **Preserve bypass (recommended):** continue sending `bypass=1` in the `FILTER.BAND` payload. The user explicitly bypassed this band; their parameter edits are pre-staged for when they un-bypass.
- **Un-bypass on edit:** send `bypass=0`. Trade-off: the user can't tweak filters with the audio bypassed for A/B comparison.

DSPi's reference app uses preserve.

### 6.4 Listening for state changes from other hosts

The notification endpoint (bulk EP `0x83`) sends a `notify_param_write(offset, len, data)` whenever any param changes. For per-band bypass updates triggered by `FILTER.BAND` or `FILTER.BAND_BYPASS`, the firmware sends the **entire `WireBandParams`** (16 bytes) at offset:
```c
offsetof(WireBulkParams, eq) + (ch * WIRE_MAX_BANDS + b) * sizeof(WireBandParams)
= 360 + (ch * 12 + b) * 16
```

So your host's notify dispatcher should:
1. Identify the offset falls inside the `eq[][]` region.
2. Decode `(offset - 360) / 16` → flat index → `(ch, b) = (idx / 12, idx % 12)`.
3. Re-parse the 16 bytes as `WireBandParams` and update the cache.

### 6.5 Preset save/load UX

- Preset save (`STORE.PRESET_SAVE`): no special handling. Bypass state is captured automatically.
- Preset load (`STORE.PRESET_LOAD`): firmware emits a bulk-params changed notification and the new `WireBulkParams` becomes the truth. Re-render your EQ UI from the new wire data; in particular, **update bypass indicators** since they may have changed.

### 6.6 Display recommendations

- A bypassed band's frequency response curve should be drawn as flat (or hidden) in the EQ visualization, even though the freq/Q/gain values are still set. This matches what the audio actually does.
- Show a clear "bypassed" indicator on the band control (icon, dimmed handle, strikethrough on the gain readout, etc.) so users know why their adjustments don't change the curve.
- "Bypass all bands" / "Solo this band" are convenient bulk operations. Implement them client-side by sending one `FILTER.BAND_BYPASS` SET per band, retrying on `VC_ERR_BUSY`. There is no server-side bulk command.

---

## 7. Internal firmware data flow (for the curious)

```
host SET (one of three paths):
   FILTER.BAND            → vendor/vc_page_filter.c: build recipe from wValue + payload, normalize → pending_packet
   FILTER.BAND_BYPASS     → vendor/vc_page_filter.c: clone live recipe, set bypass, → pending_packet
   STORE.ALL_PARAMS       → bulk_params.c: per-band normalize → filter_recipes, then full recompute

main.c eq_update_pending handler:
   filter_recipes[ch][b] = pending_packet
   notify_param_write( WireBandParams shadow )
   if Core 1 owns this channel: spin until Core 1 idle
   IRQ-safe: dsp_compute_coefficients(p, &filters[ch][b], Fs)
       → if (p->bypass == 1 || is_filter_flat(p) || Fs == 0):
             bq->bypass = true; coeffs zeroed; return
   recompute channel_bypassed[ch] from filters[ch][*].bypass

audio path (per sample / per block, both cores, every channel):
   if (channel_bypassed[ch]) → skip channel entirely (existing optimization)
   else for each band:
       if (bq->bypass) continue;       // ← THIS LINE is what user-bypass exploits
       ...biquad/SVF math...
```

The single line `if (bq->bypass) continue;` already existed in every DSP inner loop in the firmware. User-bypass is purely a new way to set that flag — no new code paths in the audio engine.

### CPU cost

- **Audio path:** zero added cost. The `bq->bypass` check ran on every band of every channel of every sample before this feature; it still does. A bypassed band actually saves CPU because it skips the biquad/SVF math.
- **Coefficient recompute:** one extra byte comparison (`p->bypass == 1`) per band. Recomputes happen on user write or sample-rate change, never in the audio path.
- **Memory:** zero. The `bypass` byte already existed inside `Biquad` (auto-bypass for FLAT filters), and the recipe byte already existed as `EqParamPacket.reserved`.

### Memory cost

```
Δ BSS:        0 bytes
Δ flash code: < 100 bytes (one extra check + new opcode handlers + normalize loops)
Δ wire size:  0 bytes (re-uses padding)
Δ flash slot: 0 bytes (re-uses padding inside EqParamPacket)
```

---

## 8. Compatibility matrix

| Host firmware version | Knows about per-band bypass? | What happens |
|-----------------------|-----------------------------|--------------|
| **Old host + Old firmware** | No / No | Status quo. |
| **Old host + New firmware** | No / Yes | Host sends 0 (or 0xFF) in the `reserved` byte → firmware normalizes to 0 → bands stay active. Identical to old behavior. |
| **New host + Old firmware** | Yes / No | Host sends `bypass=1` → byte stored in `reserved` → old firmware ignores it. Bypass UI in the new host appears non-functional. Host should detect the firmware version (`GET_PLATFORM` on 1.1.6, `SYS.PLATFORM` on 1.1.7) and gray out the UI. |
| **New host + New firmware** | Yes / Yes | Works as designed. |

**Detecting firmware support from the host:** every firmware that speaks the paged protocol (1.1.7 and later) has `FILTER.BAND_BYPASS`. A host that gates features on the command table checks the command's GET bit in `SYS.PAGE_COMMANDS` for page 0x06. Firmware 1.1.6 and older does not answer the paged protocol at all (`vendor_command_overhaul_plan.md` section 5.14). Per-band bypass landed in 1.1.4, so the matrix above matters only to hosts that still speak the 1.1.6 protocol.

---

## 9. Things not to do

- **Don't write `0xFF` to mean bypass.** It will mean "active" with no error. See §5.
- **Don't write `2` thinking it might be a future "phase-invert + bypass" code.** Write `0` or `1`. Future firmware may extend semantics; the strict `== 1` rule is the only forward-compatible signal you can rely on today.
- **Don't assume `EqParamPacket` and `WireBandParams` are memcpy-compatible.** They differ in the leading bytes (`channel`/`band` vs. nothing). Marshal field-by-field. The `FILTER.BAND` payload, by contrast, has the `WireBandParams` layout.
- **Don't try to bypass the master EQ by setting all bands' bypass.** The user-bypass flag is per-band; there is a separate `FILTER.EQ_BYPASS` command for the master toggle. Setting all bands gives the same audio result but costs N×M control transfers and triggers N×M coefficient recomputes.
- **Don't change `WireBandParams.reserved[2]` to non-zero.** It's reserved for future extension; the firmware ignores it today, but a future version may interpret it. Future-you will thank you.
- **Don't poll bypass state.** Use the notify endpoint (§6.4) — the firmware pushes updates whenever any host changes a band.

---

## 10. Quick reference card

```
Toggle: FILTER.BAND_BYPASS  page 0x06 cmd 0x11  wValue=(ch<<8)|band  SET u8 (0 or 1), GET u8 (always 0 or 1)
Whole:  FILTER.BAND         page 0x06 cmd 0x10  wValue=(ch<<8)|band  16 B both ways, bypass at offset 1
Bulk:   STORE.ALL_PARAMS    page 0x02 cmd 0x40  wValue=0             WireBulkParams, 6136 B both ways

EqParamPacket.bypass     offset 3, 1 byte (firmware recipe and preset slots)
WireBandParams.bypass    offset 1, 1 byte
WireBulkParams.eq[ch][b].bypass  byte offset 360 + (ch*12 + b)*16 + 1

Encoding rule: 1 = bypass, anything else = active.
                       ^ the only value the firmware treats as bypass
```
