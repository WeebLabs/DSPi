# WebUSB Support — Implementation Spec

*Last updated: 2026-05-13*
*Status: design — not yet implemented*
*Platforms: RP2040 + RP2350*

*Note (2026-11-16): the vendor command names and numbers in this document are those of firmware 1.1.6. Firmware 1.1.7 uses the paged protocol described in `../commands.md`. The old to new mapping is in `vendor_protocol_migration_notes.md` and `../protocol/legacy_map.json`.*

This spec defines WebUSB-ready firmware additions for DSPi, letting browser-based control apps (Chrome, Edge, Opera on desktop + Android) talk to the device's existing vendor interface over the WebUSB API. The change is purely additive — no existing behavior, descriptor, or wire format is altered, and the change is verifiable as a no-op for users who never visit a WebUSB-enabled page.

---

## 1. Background

**WebUSB** (W3C draft, Chrome-implemented) lets a web page issue control / bulk / interrupt / isochronous USB transfers to a connected device, gated by an explicit user-permission prompt. The browser identifies a device as WebUSB-capable by reading a **Platform Capability descriptor** in the device's BOS (Binary Object Store).

The device-side requirements are minimal:
1. Include a **WebUSB Platform Capability** entry in the BOS (24 bytes).
2. Optionally serve a **URL descriptor** that points at a "landing page" the browser can suggest to the user when the device is plugged in.
3. Reserve one vendor `bRequest` code for the WebUSB GET_URL transfer.

That's the entire firmware surface. DSPi already has every other piece needed (vendor interface, control + bulk endpoints, custom vendor handler, BOS scaffolding for MS OS 2.0).

**What WebUSB cannot touch:** USB Audio Class interfaces. The OS owns kernel-class interfaces; WebUSB only sees the vendor interface. DSPi's audio behavior is unaffected.

**Browser support:**

| Platform | Chrome | Edge | Opera | Firefox | Safari |
|----------|--------|------|-------|---------|--------|
| Windows  | ✓      | ✓    | ✓     | ✗       | n/a    |
| macOS    | ✓      | ✓    | ✓     | ✗       | ✗      |
| Linux    | ✓      | ✓    | ✓     | ✗       | n/a    |
| Android  | ✓      | ✓    | ✓     | ✗       | n/a    |
| iOS      | ✗      | ✗    | ✗     | ✗       | ✗      |

Origin must be served over HTTPS (or be `localhost`).

---

## 2. Scope

### In scope
- WebUSB-ready descriptor additions (BOS Platform Capability + URL descriptor).
- Handler for the WebUSB `GET_URL` vendor request.
- Coexistence with the existing MS OS 2.0 Platform Capability (Windows WinUSB auto-binding).
- Zero impact on the existing native host app's vendor-interface usage.
- One landing-page URL (compile-time constant — see §6 on configurability).

### Out of scope
- Building a web-side control app (separate project; can reuse the wire-format spec from the existing native host).
- Runtime URL configurability via vendor command (see §6 — leaving this build-time-only to avoid security/persistence complexity that doesn't pay off in user value).
- Per-origin access policy. The spec allows declaring allowed origins in the URL descriptor; we don't restrict, leaving permission entirely to the user-prompt model.
- Custom WebUSB URL types beyond a single landing page. Some implementations advertise multiple URL indices (manual, support, store); DSPi advertises one.

---

## 3. Design goals + non-goals

### Goals

1. **Additive only.** No change to existing descriptors' offsets, no change to vendor opcode numbering, no change to MS OS 2.0 behavior, no change to audio. A diff between "without WebUSB" and "with WebUSB" should be inspectable as pure addition.
2. **Survives device-side reflash without web-side coordination.** The URL is a compile-time constant living in firmware. Updating the URL means reflashing the firmware. This is acceptable because: (a) the URL is meaningful only when a web app exists at that endpoint, and (b) changing it remotely without authentication would be a phishing vector.
3. **Zero overhead when not exercised.** Browsers ask for the URL exactly once per device-plug, then never again. The vendor handler's WebUSB case fires precisely on that single request and is unreachable through any other code path.
4. **Maintainable.** A single new constant (`WEBUSB_LANDING_URL`) is the customization point. Adding more URL indices later is a one-table extension. No "wait, where does the URL come from" archaeology.
5. **Documented inline.** Every WebUSB-spec-derived constant (UUID, descriptor type codes, bRequest values) carries a spec-section reference in the comment so future maintainers can verify against W3C without re-deriving.

### Non-goals
- Web-side library or example. Out of scope for firmware.
- Authentication on the WebUSB control channel. The vendor protocol DSPi exposes is the same one the native app uses — no escalated privileges via WebUSB.
- Defeating Chrome's permission prompt. The prompt is the user-consent mechanism; bypassing it is a browser-policy concern, not firmware.

---

## 4. Architecture

### 4.1 BOS descriptor extension

The existing BOS descriptor (in `firmware/DSPi/usb_descriptors.c`) contains:
- BOS header (5 bytes)
- MS OS 2.0 Platform Capability (28 bytes)

The new BOS will contain:
- BOS header (5 bytes) — `wTotalLength` updated
- MS OS 2.0 Platform Capability (28 bytes) — unchanged
- WebUSB Platform Capability (24 bytes) — new

**WebUSB Platform Capability layout** (W3C WebUSB §3.1.1):

| Offset | Field | Size | Value | Notes |
|--------|-------|------|-------|-------|
| 0 | bLength | 1 | 0x18 (24) | Descriptor size |
| 1 | bDescriptorType | 1 | 0x10 | USB DEVICE CAPABILITY |
| 2 | bDevCapabilityType | 1 | 0x05 | PLATFORM |
| 3 | bReserved | 1 | 0x00 | Spec-required zero |
| 4 | PlatformCapabilityUUID | 16 | `3408b638-09a9-47a0-8bfd-a0768815b665` | WebUSB UUID, little-endian per RFC 4122 §4.1.2 |
| 20 | bcdVersion | 2 | 0x0100 | WebUSB spec version 1.00 |
| 22 | bVendorCode | 1 | `WEBUSB_VENDOR_CODE` (= `0x02`) | DSPi vendor bRequest reserved for WebUSB transfers |
| 23 | iLandingPage | 1 | `0x01` | Index of the landing page URL descriptor (1-based; 0 = none) |

`bVendorCode = 0x02` is distinct from `MS_VENDOR_CODE = 0x01` (already used by MS OS 2.0). The two platform capabilities live side-by-side in the BOS; the browser and Windows each fetch their own descriptor via their own vendor code without ambiguity.

### 4.2 URL descriptor

**WebUSB URL descriptor layout** (W3C WebUSB §4.1):

| Offset | Field | Size | Value | Notes |
|--------|-------|------|-------|-------|
| 0 | bLength | 1 | 3 + N | N = URL string byte length |
| 1 | bDescriptorType | 1 | 0x03 | WEBUSB URL |
| 2 | bScheme | 1 | 0x01 | 0x00=http://, 0x01=https://, 0xFF=embedded in URL |
| 3 | URL | N | UTF-8 | URL without the scheme prefix |

DSPi's URL descriptor uses `bScheme = 0x01` (https) so the browser prepends `https://` automatically.

**Compile-time URL constant** (`usb_descriptors.h`):

```c
#define WEBUSB_LANDING_URL  "dspi.weeblabs.com/control"
```

Total URL descriptor size: 3 + 25 = 28 bytes. Well under the 255-byte descriptor limit.

### 4.3 Vendor-handler addition

The existing `tud_vendor_control_xfer_cb()` in `firmware/DSPi/vendor/vc_usb.c` already serves MS OS 2.0 GET_DESCRIPTOR transfers (`bRequest = MS_VENDOR_CODE = 0x01`, `wIndex = 0x07`) at SETUP, before the paged front end sees the request. The new WebUSB GET_URL handler is symmetric and sits beside it:

- `bmRequestType = 0xC0` (vendor IN device-to-host)
- `bRequest = WEBUSB_VENDOR_CODE = 0x02`
- `wValue = URL descriptor index` (we only support `0x01`)
- `wIndex = 0x02` (WEBUSB_REQUEST_GET_URL — W3C WebUSB §3.1.2)

Response: the URL descriptor bytes.

The handler validates `(bRequest, wIndex, wValue)` and responds via the existing `tud_control_xfer()` pattern used for MS OS 2.0.

### 4.4 Disambiguation between MS OS 2.0 and WebUSB requests

Both descriptors share the same control endpoint (EP0) and the same vendor handler. The two are distinguished by `bRequest`:

| `bRequest` | Origin | Handler action |
|------------|--------|----------------|
| `0x01` (MS_VENDOR_CODE) + `wIndex = 0x07` | Windows requesting MS OS 2.0 descriptor | Return existing MS OS 2.0 descriptor set |
| `0x02` (WEBUSB_VENDOR_CODE) + `wIndex = 0x02` | Browser requesting URL descriptor | Return URL descriptor |
| Anything else | DSPi vendor command (firmware 1.1.7 paged protocol) | Pass to the paged front end, `vc_dispatch()` |

No collision is possible. We assign both vendor codes in the BOS, so we control both sides. The paged protocol reserves `bRequest` 0x00-0x0F in every page, with 0x01 for MS OS 2.0 and 0x02 for WebUSB (`vendor_command_overhaul_plan.md` section 3.2, rule 11). A GET_URL request also carries 0 in the `wIndex` high byte, which is page 0, and page 0 holds no DSPi command.

### 4.5 File touchpoints

| File | Change |
|------|--------|
| `firmware/DSPi/usb_descriptors.h` | Add `WEBUSB_VENDOR_CODE`, `WEBUSB_LANDING_URL`, descriptor-type and request-index constants. |
| `firmware/DSPi/usb_descriptors.c` | Extend BOS descriptor with WebUSB Platform Capability. Define static URL descriptor byte array. Update `wTotalLength` in BOS header. Update `tud_descriptor_bos_cb()` if needed (TinyUSB usually serves BOS via a single array). |
| `firmware/DSPi/vendor/vc_usb.c` | Add WebUSB GET_URL case to `tud_vendor_control_xfer_cb()`, next to the MS OS 2.0 case. |
| `Documentation/current_architecture.md` | New "WebUSB" subsection under "USB Audio Pipeline" or as a sibling top-level section. |

No changes to: audio path, DSP, flash storage, bulk-params wire format, preset system, host app. Existing vendor opcodes unchanged.

---

## 5. Edge cases

### 5.1 Browser asks for URL index 0
Spec-reserved value. Return STALL — `tud_control_xfer(rhport, req, NULL, 0)` with the IN data stage refused. The browser will fall back to no-landing-page behavior, which is what `iLandingPage=0` would have signaled anyway.

### 5.2 Browser asks for URL index 2..255
DSPi advertises one URL (index 1). Indices outside the supported range must STALL. Browser falls back to no-landing-page.

### 5.3 Browser asks for URL with `wLength` < descriptor size
TinyUSB's `tud_control_xfer(rhport, req, urlbuf, descriptor_length)` correctly returns only `min(wLength, descriptor_length)` bytes — standard USB SETUP behavior. No special-case code needed.

### 5.4 Browser asks for URL with `wLength` >> descriptor size
Browser receives the descriptor and the host-side stack stops reading after `bLength`. No data leak — TinyUSB only sends the exact descriptor length.

### 5.5 MS OS 2.0 request collides with WebUSB on `wIndex`?
**No.** MS OS 2.0 uses `wIndex = 0x07` (MS_OS_20_DESCRIPTOR_INDEX); WebUSB GET_URL uses `wIndex = 0x02`. Distinct values. The `bRequest` is also distinct (0x01 vs 0x02). Disambiguation is unambiguous on both `bRequest` AND `wIndex`.

### 5.6 Windows fetches WebUSB descriptor "by accident"?
Windows ignores the WebUSB Platform Capability — it only reads MS OS 2.0 capabilities for driver binding. The WebUSB capability is invisible to Windows USB enumeration.

### 5.7 macOS / Linux behavior
Both OSes ignore both Platform Capability descriptors at enumeration time (no auto-driver-binding for vendor interfaces). The WebUSB descriptor only matters when a Chromium-family browser later opens the device.

### 5.8 Native host app vs. WebUSB page running concurrently
Both claim vendor interface #2. The OS gives the handle to whichever process opens it first; the other gets `LIBUSB_ERROR_BUSY` (or platform equivalent). This is unchanged by adding WebUSB — same one-claim-at-a-time semantics that already exist for the native host app on Windows (WinUSB) and on macOS/Linux (libusb).

**Documentation note:** the user-facing docs should say "close the native DSPi Console before using the web-based control panel, and vice versa."

### 5.9 Chrome permission re-prompts
By default Chrome prompts the user every session unless the origin is allow-listed via enterprise policy (`WebUsbAllowDevicesForUrls`). This is browser policy, not firmware. We don't try to defeat or work around it.

### 5.10 Firmware update via web app while WebUSB is open?
Firmware update uses `SYS.ENTER_BOOTLOADER` (page 0x01, command 0x30, a zero-length SET), which reboots into UF2. From the browser's perspective the device disappears mid-session; the browser closes the WebUSB handle cleanly. After reboot the device appears as a UF2 mass-storage drive. Browsers can't talk to that (no WebUSB descriptor). User drops the UF2, device reboots, web app reconnects. Standard flow.

### 5.11 URL length > 252 bytes
Descriptor `bLength` is a single byte and the URL must fit in `255 - 3 = 252` bytes. Compile-time `_Static_assert` on `sizeof(WEBUSB_LANDING_URL)` catches over-long URLs at build time.

### 5.12 URL contains non-ASCII characters
WebUSB URL descriptors are UTF-8 encoded. ASCII is a valid UTF-8 subset. If anyone ever needs IDN (internationalized domains), use punycode (xn--…) — keeps the URL ASCII-clean. Spec doc notes this; the firmware doesn't validate.

### 5.13 What if a future firmware needs a different vendor bRequest for an unrelated feature?
Since firmware 1.1.7 the vendor `bRequest` namespace is per page (the page travels in the `wIndex` high byte):
- `0x00` and `0x03..0x0F` are reserved for protocol-level use in every page.
- `0x01` = MS_VENDOR_CODE (MS OS 2.0).
- `0x02` = WEBUSB_VENDOR_CODE (this spec), held in every page.
- `0x10..0xFF` = DSPi commands within each page.

A future platform-style vendor code takes a number from `0x03..0x0F`. The allocation rules are in `vendor_command_overhaul_plan.md` section 3.2.

### 5.14 Cold-boot ordering with the host app
Browser opens device → reads BOS → sees WebUSB capability → asks for URL → firmware responds. This sequence happens entirely via control transfers on EP0, before any audio streaming starts. No interaction with audio init, presets, or anything else. Cold-boot ordering unchanged.

### 5.15 Device reset during a URL transfer
If the device resets mid-GET_URL transfer (extremely unlikely — control transfers are tens of microseconds), the browser sees a stall/timeout and the device re-enumerates. WebUSB enumeration resumes after re-enum. No special handling needed.

### 5.16 Per-origin allow-list (skipped)
WebUSB descriptors can include an "allowed origins" list. We don't use this — any origin can request access (the user consent prompt is the gate). Future enhancement if a real use case appears.

### 5.17 What if the host running the browser doesn't have HTTPS available?
WebUSB requires HTTPS for non-localhost origins. If our advertised URL is HTTP-only, the browser shows the URL but visiting it doesn't grant WebUSB access. Use HTTPS for the landing page. Compile-time choice; spec doc reminds the implementer.

### 5.18 Browser bug or spec evolution
The WebUSB spec is a W3C draft and Chrome's implementation may drift. We use only the stable subset (Platform Capability + URL descriptor + GET_URL), which has been stable since Chrome 61 (2017). Lower-level breakage would manifest as "WebUSB suddenly stops working in version N+1," requiring a firmware update — same risk as any browser-facing feature.

---

## 6. Should the URL be runtime-configurable?

Considered, rejected. Reasons:

1. **Phishing surface.** A malicious tool with vendor-command access could redirect users to a fake control page that captures inputs. Today the vendor protocol has no authentication; relying on URL trust requires either authentication (large scope) or a fixed URL (this design).
2. **Persistence cost.** Storing a configurable URL needs a flash sector, a length-checked string field in the directory, validation, and migration. Adds ~30 bytes of flash, ~30 bytes of BSS, and a wire-format extension — for what?
3. **No user demand.** Power users who want to point at their own control panel can already do so by building their own firmware with a different `WEBUSB_LANDING_URL`. That's a deliberate-friction path that matches the threat model.
4. **Cleaner update story.** When the official control page URL changes, a firmware update is the obvious time to update the URL. No drift between firmware version and advertised URL.

If a real use case appears later (e.g., enterprise deployments with private control panels), revisit. The Platform Capability descriptor doesn't change; just the URL descriptor would need to be runtime-served instead of compile-time-static. Migration path is clean.

---

## 7. Performance & memory cost

| Item | Cost |
|------|------|
| `usb_descriptors.c` BOS extension | +24 bytes flash (Platform Capability) + 28 bytes (URL descriptor) = **52 bytes flash** |
| `vendor/vc_usb.c` handler additions | ~30 lines of code, ~80 bytes flash |
| BSS | **0 bytes** — no module-level state; descriptor is `const` in flash |
| Audio path | **0 cycles** — never touched by audio path |
| USB enumeration time | Negligible (one extra 24-byte BOS read at descriptor-set fetch; happens once per plug) |
| Control-transfer rate | One additional GET_URL transfer per browser session, only when a WebUSB-aware page is visited |

---

## 8. Test plan

1. **No-WebUSB baseline.** Without browser involvement, all existing behavior (Windows WCID, native host app, audio class, vendor opcodes) works identically. Build, flash, sanity-check audio + native app.
2. **BOS descriptor dump.** Use `lsusb -v` (Linux) or `system_profiler SPUSBDataType` (macOS) to confirm the BOS contains both MS OS 2.0 and WebUSB platform capabilities, with correct UUIDs and sizes.
3. **WebUSB demo page.** Visit Chrome's [WebUSB explorer demo](https://wicg.github.io/webusb/demos/) → click "Request device" → DSPi appears in the chooser → confirm the landing page URL displays correctly in the browser's device-info panel.
4. **GET_URL transfer.** Use Chrome DevTools to call `device.controlTransferIn({...})` with `bRequest = WEBUSB_VENDOR_CODE`, `wIndex = WEBUSB_REQUEST_GET_URL`, `wValue = 1` → response decodes to the configured URL.
5. **Invalid URL indices.** Call `controlTransferIn` with `wValue = 0`, `wValue = 2`, `wValue = 255` → all STALL. Browser reports the failure cleanly.
6. **MS OS 2.0 unaffected.** Plug into Windows, verify WinUSB auto-binds, verify native host app's vendor calls work.
7. **Vendor opcode unaffected.** Native host app sends all the existing vendor commands (preset save/load, bulk get/set, etc.) — every one works as before.
8. **Concurrent claim.** With the native host app running, open the WebUSB demo page and try to claim the device → browser reports the device is busy (LIBUSB_ERROR_BUSY or platform equivalent). Close the native app, retry → succeeds. Confirms one-claim-at-a-time semantics are unchanged.
9. **Firefox / Safari.** Plug device, visit demo page — the page reports "WebUSB not supported" — confirms graceful degradation.
10. **Linux udev.** Without a udev rule, browser may prompt for permission to access the device. Add `SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8b", ATTRS{idProduct}=="feaa", MODE="0666"` to a udev rule file if needed. Documented but not required at firmware level.
11. **Long URL stress test.** Temporarily set `WEBUSB_LANDING_URL` to a 252-byte string, verify build succeeds. Set to 253 bytes, verify `_Static_assert` catches at build time.

---

## 9. Implementation steps

1. **`usb_descriptors.h`**: define constants
   - `WEBUSB_VENDOR_CODE` (0x02)
   - `WEBUSB_REQUEST_GET_URL` (0x02)
   - `WEBUSB_DESC_TYPE_URL` (0x03)
   - `WEBUSB_URL_SCHEME_HTTPS` (0x01)
   - `WEBUSB_LANDING_URL` (string)
   - `_Static_assert` to guard URL length ≤ 252.
2. **`usb_descriptors.c`**: extend BOS
   - Add 24-byte WebUSB Platform Capability struct to the BOS array.
   - Update `wTotalLength` in the BOS header.
   - Define the URL descriptor as a `static const uint8_t webusb_url_descriptor[]`.
3. **`vendor/vc_usb.c`**: extend `tud_vendor_control_xfer_cb`
   - Add a SETUP-stage case for `req->bRequest == WEBUSB_VENDOR_CODE && req->wIndex == WEBUSB_REQUEST_GET_URL`.
   - Validate `wValue == 1`; STALL otherwise.
   - Return the URL descriptor via `tud_control_xfer()`.
   - Comment thoroughly with W3C-spec section references.
4. **`Documentation/current_architecture.md`**: add a "WebUSB" subsection under USB Audio Pipeline.
   - Document the vendor `bRequest` namespace (0x01 = MS OS 2.0, 0x02 = WebUSB, 0x10 and up = DSPi commands in each page).
   - Note the one-claim-at-a-time interaction with the native host app.
5. **Build both platforms.** Verify size deltas are within spec (~130 bytes text).
6. **Manual test plan §8.**

---

## 10. Open questions

- **Should we advertise an "origins allow-list" in the WebUSB descriptor?** No — keeps the door open for any browser-based tooling without firmware involvement. The user prompt is the gate.
- **Should the test plan include automated Chrome WebUSB unit tests?** Out of scope. Browser tests are a separate project; firmware tests stay at the descriptor and protocol level.
- **Should `WEBUSB_LANDING_URL` go into a separate "branding" header so OEMs reskinning DSPi can override without editing usb_descriptors.h?** Probably yes if reskinning becomes a goal. Trivial later refactor (move one `#define` to a new header). Not blocking.

---

## Sources

- [W3C WebUSB API draft (Editor's Draft)](https://wicg.github.io/webusb/) — §3.1.1 Platform Capability descriptor, §3.1.2 vendor request codes, §4 URL descriptor format.
- [Chrome WebUSB introduction (Google Developers)](https://developer.chrome.com/docs/capabilities/usb) — high-level usage and browser support.
- [Microsoft USB Platform Capability descriptor](https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors) — for context on the MS OS 2.0 capability that already lives in DSPi's BOS.
- TinyUSB `tud_vendor_control_xfer_cb()` reference: `firmware/pico-sdk/lib/tinyusb/src/class/vendor/vendor_device.c`.
- Existing DSPi MS OS 2.0 implementation: `firmware/DSPi/usb_descriptors.c` (current BOS code) — pattern to mirror.
