# I2S Clock-Slave Input Mode Specification

*Last updated: 2026-11-16 (host interface on the paged protocol of firmware 1.1.7). 2026-07-11 (clock-pin mode: UNIFIED vs SPLIT slave pair, role-indexed BCK pin, new clock-pin mode command, slot V29 / directory V14; wire claims two `WireI2SConfig` reserved bytes with no version bump)*

This document specifies the I2S clock-slave input mode: behavior, wiring, the
complete host-facing control surface (vendor commands, notifications, bulk
parameter and persistence formats), and the integration patterns an app or
LLM needs to support the feature. It complements
`Documentation/Features/i2s_input_spec.md`, which describes the default
clock-master architecture; everything there still applies when the clock mode
is MASTER.

---

## 1. Concept

The DSPi has two I2S clock modes, selected by a single persisted setting:

| Mode | Value | Who drives BCK/LRCLK | Rate authority |
|---|---|---|---|
| MASTER (default) | 0 | DSPi (PIO side-set or I2S output clock master) | Host-selected via `INPUT.I2S_RX_RATE` |
| SLAVE | 1 | External device; DSPi's BCK/LRCLK pins become inputs | Auto-detected from the external LRCLK |

In SLAVE mode the DSPi handles audio data only: it clocks input data in and
output data out on clock edges supplied by the external master. This allows
superior external clocking and compatibility with I2S sources that only
operate as clock masters (many ADCs, USB-to-I2S interfaces).

The mode is only meaningful while the input source is I2S
(`INPUT.SOURCE` = 2). With any other input source the setting is
dormant: it is stored, reported by GET, and takes effect at the next switch
into I2S. **While a non-I2S source is selected, the DSPi reverts to driving
the clock pins whenever an I2S output slot exists** (see section 8).

Everything below is identical on RP2040 and RP2350 except where noted (ADAT
is RP2350-only).

### Clock domains in slave mode

| Output | Clocking in slave mode |
|---|---|
| I2S output slots | Edge-slaved: a wait-driven PIO program shifts data bits directly on the external BCK/LRCLK edges (bit-exact lock, no servo) |
| SPDIF output slots | Internally clocked, rate-matched to the external clock by a servo (measured-rate loop plus consumer-fill trim) |
| ADAT output (RP2350) | Same servo divider as SPDIF (identical 256 x Fs PIO clock) |
| PDM sub output | Nominal rate, not servoed (identical to SPDIF-input mode behavior) |
| MCK output | Forced off (a locally generated MCK would be asynchronous to the external BCK/LRCLK, which is invalid for downstream converters); re-enabled per its stored config when leaving slave mode |

Inter-slot sample alignment is preserved: synchronized output starts are
gated on an external LRCLK edge and all output slots (both types) start in a
single `pio_enable_sm_mask_in_sync` write, with the edge-slaved I2S program
discarding exactly one frame so I2S and SPDIF slots emit the same sample
index in the same frame window.

---

## 2. Hardware and wiring

### Pins

Which pins carry BCK/LRCLK is governed by `i2s_clock_pin_mode`, a persisted
setting with two values:

| Clock-pin mode | Value | Master clock mode uses | Slave clock mode uses |
|---|---|---|---|
| UNIFIED (default, legacy) | 0 | `i2s_bck_pin` / +1 | the SAME `i2s_bck_pin` / +1 pair |
| SPLIT | 1 | `i2s_bck_pin` / +1 | a SEPARATE `i2s_bck_pin_slave` / +1 pair |

- BCK = `i2s_bck_pin` (default GPIO 14), LRCLK = BCK + 1 (hardware constraint,
  default GPIO 15). This is the master/unified pair.
- Slave-pair BCK = `i2s_bck_pin_slave`, LRCLK = +1. Defaults: **GPIO 12/13 on
  RP2040, GPIO 26/27 on RP2350** (`PICO_I2S_BCK_PIN_SLAVE`). Only meaningful,
  and only claimed, in SPLIT mode; fully dormant in UNIFIED mode.

In UNIFIED mode slave clocking listens on the same pair master mode drives, so a
hard-wired external master shares GPIOs with the device's own clock outputs
whenever a non-slave role goes live (see the dual-driver warning below). SPLIT
mode removes that contention: the device drives `i2s_bck_pin` in every non-slave
role and only ever LISTENS on `i2s_bck_pin_slave`, so the two never fight over
the same GPIOs. The firmware resolves the active pair with a single helper
(`i2s_effective_bck_pin()`); every hardware consumer (TX clock base, RX start
snapshot, rebuild change-detection) reads it, and in SPLIT mode BOTH pairs count
as reserved for RX-data / control-interface / DAC-mute pin checks. The two pairs
are kept mutually distinct by validation.

**Setting the pins:**

- `CLOCK.I2S_BCK_PIN` SET: `wValue` = role, 1-byte payload = BCK GPIO. Role 0 =
  master/unified pair, role 1 = slave pair. Rejected with
  `VC_ERR_STATE` while I2S output slots run on the pair being moved,
  `VC_ERR_PIN_IN_USE` on overlap/collision, `VC_ERR_ADDRESS` for role > 1. The slave
  pin is storable at any time; a change restarts the input only when that pair
  is the one the running input currently uses (SPLIT + slave clocking).
- `CLOCK.I2S_BCK_PIN` GET: `wValue` = role (0 = master/unified,
  1 = slave); an invalid role is `VC_ERR_ADDRESS`.
- `CLOCK.I2S_CLOCK_PIN_MODE` SET: 1-byte payload, 0 unified / 1 split. Entering SPLIT always validates the slave pair
  (distinct from the master pair, both GPIOs free of other owners), even as a
  dormant store while the clock mode is master; a later `CLOCK.I2S_CLOCK_MODE` flip adopts the
  pair without rechecking, so the conflict must be caught here (`VC_ERR_PIN_IN_USE`).
  Leaving SPLIT needs no pin checks. A set while slave clocking is (or is
  pending) live is rejected with `VC_ERR_STATE` when I2S output slots exist;
  otherwise it applies live and arms a deferred input restart onto the new
  pair. `VC_ERR_VALUE` for a payload > 1.
- `CLOCK.I2S_CLOCK_PIN_MODE` GET: returns the live mode byte (0/1).

A rejected SET STALLs and its result code is readable from `SYS.LAST_RESULT`
(section 4). In slave mode the active clock pair pins are inputs. Input data pins
(`INPUT.I2S_RX_PIN`) and I2S output slot data pins
(`OUTPUT.PIN`) are unchanged.

### External master requirements

- **BCK must be 64 x Fs** (32 BCK per channel, 24-in-32 standard I2S with the
  1-bit delay). Masters running 32 x Fs or 48 x Fs frames produce garbage;
  there is no runtime format detection.
- **Standard LRCLK polarity** (low = left). Inverted-WS masters silently swap
  channels.
- Supported detected rates: **44100, 48000, 96000 Hz** (snap tolerance 2%).
  Other rates (32k, 88.2k, 176.4k, 192k) never lock; outputs stay muted and
  the status reports ACQUIRING with the raw measured rate.

### Usage example

External USB-to-I2S interface (master-only) feeding two I2S DACs and two
SPDIF DACs:

1. Interface BCK out -> DSPi GPIO 14 (BCK, now input); LRCLK out -> GPIO 15.
2. Interface DATA out -> DSPi I2S RX data pin (default GPIO 1).
3. Two output slots set to I2S; their data pins -> the I2S DACs' data inputs.
   The DACs' BCK/LRCLK inputs connect to the interface's master outputs
   (all devices share the one external clock pair).
4. Two output slots left as SPDIF -> the SPDIF DACs.
5. Enable slave mode, set the input source to I2S, play.

### Dual-driver warning (important)

Whenever the DSPi is NOT in slave-clocked I2S operation (input source is USB
or SPDIF, or clock mode is MASTER) and an I2S output slot exists or the I2S
input is active, the DSPi drives BCK/LRCLK itself. If the external master is
still wired and driving, both devices drive the same lines (bounded by pad
current limits, but not healthy). Apps should warn users to avoid selecting
other input sources or master mode while an external master is hard-wired,
or wire the clock lines through small series resistors. A short muted
transient of this kind also occurs at boot before a slave-mode startup
preset fully applies, and during live mode/preset transitions; the firmware
minimizes but cannot eliminate it while the user has both devices wired.

---

## 3. Runtime behavior

### Lock state machine

Slave mode adds a SPDIF-style lock state machine, exposed via
`CLOCK.I2S_SLAVE_STATUS` and NOTIFY event 0x09:

| State | Value | Meaning |
|---|---|---|
| INACTIVE | 0 | Not in slave role (mode is MASTER, input is not I2S, or the input is stopped, e.g. across a rate change or flash write) |
| ACQUIRING | 1 | Measuring the external clocks; no lock yet; outputs muted |
| RELOCKING | 2 | Clocks lost or rate changed; outputs muted, waiting |
| LOCKED | 3 | Locked to a supported rate; audio flowing (after prefill) |

Mechanics:

- The rate is measured from the RX DMA word rate (2 words per frame of
  external LRCLK) over ~32 ms windows. Two consecutive agreeing windows are
  required to lock, so lock takes ~64-100 ms after valid clocks appear.
- Clock loss is declared after 5 ms without data words. Behavior is **mute
  and wait**: outputs are muted and drained; nothing plays until valid
  clocks return, then the device re-locks, re-prefills, and resumes.
  Downstream SPDIF/ADAT receivers lose lock during the wait.
- An external rate change (e.g. the source switching 48 -> 96 kHz) is
  detected as a lock drop; the device re-acquires, retunes the whole
  pipeline to the new rate, and resumes automatically. Expect roughly
  100-300 ms of mute for the full re-lock plus prefill.
- A long-window (8-16 s) measurement refines the servo rate reference to
  ~0.1 ppm, which keeps ADAT rate-locked even with no SPDIF slot to observe.

### Framing-slip watchdog

Real-world external masters (USB-I2S bridges such as the Amanero Combo384)
glitch or re-frame BCK/LRCLK around stream stop/start, pause/seek, and
44.1/48-family rate switches. A short glitch (under the 5 ms clock-loss
timeout) does not change the word RATE, so the lock state machine alone
cannot see it; with naive free-running capture it would shift the bit
alignment for the rest of the session (the sign bit lands mid-word and
meters/audio show full-scale wrapped garbage).

Both external-clock PIO programs (RX capture and edge-slaved I2S TX)
therefore verify the LRCLK level at the two adjacent bit cells straddling
every frame boundary, which catches every sub-frame misalignment within one
frame. On a mismatch the program:

1. re-frames itself at the next LRCLK falling edge (bounding wrong data to
   ~2 frames, with L/R word order preserved so channels can never swap), and
2. raises a PIO irq flag that `i2s_slave_poll()` treats exactly like a
   clock loss: outputs mute, the receiver restarts, the device re-locks and
   re-prefills, and every output re-frames through the gated synchronized
   start (restoring inter-slot sample alignment, which a slip breaks at the
   wire level).

Each detected slip increments `slip_count` in the status packet (and
`loss_count`, via the relock it forces). A steadily climbing `slip_count`
with a stable master indicates signal-integrity problems on the BCK/LRCLK
wiring rather than source behavior.

### Rate reporting

- The current pipeline rate is the sample rate field of `SYS.SYSTEM_STATUS`
  (u32 Hz at offset 4). In slave mode it follows the detected rate once
  locked.
- `INPUT.I2S_RX_RATE` GET returns the selected I2S rate (u32 Hz), which in
  slave mode is the stored master-mode preference. Its SET in slave mode stores
  the value (and notifies) but does NOT change the live rate; the stored rate
  applies when the device returns to master mode.
- The status packet (`CLOCK.I2S_SLAVE_STATUS`) carries both the snapped locked rate and the raw
  measured rate.

### Output type switching

Output slots remain freely switchable between SPDIF and I2S (`OUTPUT.TYPE`) while
slave mode is active. The type switch rebuilds the affected slots with the
correct clocking (edge-slaved program for I2S) and re-runs the muted
prefill/synchronized start, exactly like a type switch in any other mode.

---

## 4. Vendor commands

| Command | Page | Cmd | Dir | Address (`wValue`) | Size |
|---|---|---|---|---|---|
| `CLOCK.I2S_CLOCK_MODE` | 0x0C | 0x12 | GET, SET | none (0) | GET 1, SET 1 |
| `CLOCK.I2S_SLAVE_STATUS` | 0x0C | 0x13 | GET | none (0) | GET 16 |

All three commands of this feature (these two and `CLOCK.I2S_CLOCK_PIN_MODE`,
section 2) use the paged protocol (see `../commands.md`) and work over USB
vendor control transfers and the UART/I2C control interfaces. A rejected SET
STALLs on USB and its `VC_*` result is readable from `SYS.LAST_RESULT`. UART
and I2C return the code in the response status byte.

### `CLOCK.I2S_CLOCK_MODE` SET

Payload: 1 byte, 0 = master, 1 = slave. A value > 1 is refused with `VC_ERR_VALUE`.

Deferred apply: the main loop performs the transition (input restart,
output-slot clocking rebuild, MCK policy). If the input source is not I2S
the value is recorded and applies at the next switch into I2S. Setting the
current mode again is a no-op. A `PARAM_CHANGED` notification for the
`input_config.i2s_clock_mode` wire field is emitted when the change is
actually applied, not at SET time.

### `CLOCK.I2S_CLOCK_MODE` GET

Returns 1 byte: the LIVE mode. A SET that has not yet been applied by the
main loop is not reflected (window is a few main-loop iterations; track the
apply via the PARAM_CHANGED notification if exact sequencing matters).

### `CLOCK.I2S_SLAVE_STATUS` GET

Returns the 16-byte `I2sSlaveStatusPacket` (little-endian):

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | state | I2sSlaveState (0-3, table above) |
| 1 | 1 | clock_mode | Live mode (0/1); lets one call answer both questions |
| 2 | 1 | lock_count | Locks since boot (saturates at 255) |
| 3 | 1 | loss_count | Losses since boot (saturates at 255) |
| 4 | 4 | detected_rate | Snapped Hz (44100/48000/96000); 0 unless LOCKED |
| 8 | 4 | measured_hz | Raw measured external rate, rounded Hz; 0 when no clocks |
| 12 | 1 | slip_count | Framing slips since boot (saturates at 255); each also increments loss_count via the relock it forces |
| 13 | 3 | reserved | Zero |

Poll this for diagnostics UI (e.g. 2-10 Hz while an I2S settings page is
open). For state transitions prefer NOTIFY event 0x09.

### Interactions with existing commands

| Command | Slave-mode behavior |
|---|---|
| `INPUT.SOURCE` | Unchanged; slave clocking engages/disengages when I2S becomes/stops being the source |
| `INPUT.I2S_RX_RATE` SET | Stored + notified only; no live effect until master mode |
| `CLOCK.I2S_BCK_PIN` SET | Role-indexed (`wValue` = role, payload = GPIO; role 0 = master/unified pair, 1 = slave pair). Rejected with `VC_ERR_STATE` only while I2S output slots run on the pair being moved; a change restarts the input when that pair is the one the running input uses. In UNIFIED mode role 0 is the slave pair too; in SPLIT mode slave clocking uses role 1 (`i2s_bck_pin_slave`, default 12/13 RP2040 / 26/27 RP2350) |
| `CLOCK.I2S_CLOCK_PIN_MODE` SET | Selects UNIFIED (0) vs SPLIT (1). Dormant store in master/non-I2S; live slave-clocked change rejected with `VC_ERR_STATE` if I2S outputs run, else validates the newly effective pair and arms a deferred input restart |
| `CLOCK.MCK_ENABLE` SET | Stored; MCK output remains forced off while slave mode is live and resumes per config on exit |
| `INPUT.I2S_RX_PIN`, `INPUT.I2S_RX_CHANNELS` | Unchanged (multichannel 4/6/8-ch input works in slave mode on RP2350; all pairs sample against the external clocks) |

---

## 5. Notifications

### NOTIFY_EVT_I2S_SLAVE_STATE (0x09)

Pushed on every slave lock-state transition. v2 packet, 9 bytes:

| Byte | Value |
|---|---|
| 0 | 0x02 (v2 version) |
| 1 | 0x09 (event id) |
| 2 | flags (0) |
| 3 | sequence number |
| 4 | state (I2sSlaveState) |
| 5-8 | detected rate, Hz, little-endian (0 unless the state is LOCKED) |

Note this packet is 9 bytes where most discrete v2 events are 8; parsers
must use per-event lengths (as with `PARAM_CHANGED`).

Expected sequences an app will observe:

- Enable + lock: `ACQUIRING -> LOCKED(rate)`
- Clock loss and recovery: `RELOCKING -> INACTIVE -> ACQUIRING -> LOCKED`
  (the INACTIVE blip is the internal receiver restart that re-frames the
  state machines; treat RELOCKING/INACTIVE/ACQUIRING uniformly as "waiting
  for clocks")
- External rate change: same as clock loss, then `LOCKED(new rate)`; a
  pipeline rate change also fires the usual recompute-driven notifications
- Leaving slave mode or switching source: `INACTIVE`

### PARAM_CHANGED

The mode itself is mirrored into the bulk-params shadow, so applying it
emits a standard `PARAM_CHANGED` (event 0x02) carrying the wire offset of
`input_config.i2s_clock_mode` with a 1-byte value. Rate changes ripple the
usual `INPUT_FORMAT` (0x05) and related events exactly as SPDIF rate changes
do.

---

## 6. Persistence and bulk parameters

### Wire format (`STORE.ALL_PARAMS`)

`WIRE_FORMAT_VERSION` = 18 when this feature landed; the byte offsets below
are the CURRENT (V28) ones. `WireInputConfig` is a 16-byte section throughout;
later versions claimed its reserved bytes rather than growing it:

| Byte | Field |
|---|---|
| 0 | input_source |
| 1 | spdif_rx_pin |
| 2 | i2s_rx_pin (pair 0) |
| 3 | i2s_input_rate (encoded: 0=44100, 1=48000, 2=96000) |
| 4 | i2s_input_channels (2/4/6/8; 0 = absent) |
| 5-7 | i2s_rx_pin_ext[3] (pairs 1-3) |
| 8-10 | spdif_rx_pin_ext[3] (SPDIF inputs 2/3/4; the third entry and the one-byte shift of everything below arrived with V28) |
| 11 | spdif_rx_enabled_ext_p1 (optional-SPDIF enable mask + 1) |
| 12 | **i2s_clock_mode (0=master, 1=slave; new in V21)** |
| 13-15 | adat_input_pin, adat_input_enabled_p1, adat_clock_mode_p1 (V24) |

Apply-side semantics match the vendor SET: validated (<= 1), deferred to the
main loop, no-op when unchanged. Pre-V21 payloads are rejected wholesale by
the existing strict version check, as with every wire bump.

The clock-pin mode + slave pair ride in a DIFFERENT section, `WireI2SConfig`
(the output/clock section), which claimed two of its reserved bytes with
**NO `WIRE_FORMAT_VERSION` bump (stays 18)**:

- `clock_pin_mode_p1`: +1 encoded (0 = absent/keep-live, 1 = unified,
  2 = split), so the mode's meaningful 0 (unified) is distinguishable from a
  zero-filled byte an old host sends.
- `bck_pin_slave`: slave-mode BCK GPIO (0 = absent/keep-live).

Apply validates the RX pin set against the values that will actually be
installed, installs the pin mode live and the slave pin only if acceptable and
non-overlapping with the master pair, and arms a deferred input restart if the
effective input pair moved.

### Preset slots

`SLOT_DATA_VERSION` = 29 appends `i2s_clock_pin_mode` (0=unified, 1=split) and
`i2s_bck_pin_slave` (slave-mode BCK GPIO, 0 = unset -> `PICO_I2S_BCK_PIN_SLAVE`)
to the slot tail, on top of the V28 `i2s_clock_mode` byte. Pre-V29 slots read
the missing bytes as 0 (= unified + unset), the correct defaults; V21..V28
slots still load via the per-version CRC-range mechanism (backward-compatible
tail-append). Both are saved by `STORE.PRESET_SAVE` like the rest of the IO
config and restored on preset load / boot, honoring `output_config_mode`
exactly like the other IO fields:

- WITH_PRESET: the slot value (device-global directory value as fallback).
- INDEPENDENT: the device-global directory value (saved via
  `STORE.OUTPUT_CONFIG_SAVE`).

Directory format V14 grew the device-global `FlashOutputConfig` by these two
bytes (29 -> 31 bytes) with an automatic V13 -> V14 migration (frozen
`FlashOutputConfig_v13` / `PresetDirectory_v13` snapshots; the new bytes default
to unified + unset). The V28 `i2s_clock_mode` byte grew the directory V12 -> V13
the same way. Unlike `i2s_clock_mode`, the clock-pin mode has no pending
mechanism: it is installed live during IO-config apply (after the master BCK,
before RX validation), arming a deferred input restart only if the effective
pair actually moved while I2S is the live source.

A preset load that changes `i2s_clock_mode` while I2S is live still applies it
through the same deferred main-loop transition (one extra muted reset after
the load).

---

## 7. App integration patterns

### Enabling slave mode

1. SET `CLOCK.I2S_CLOCK_MODE` to 1.
2. If not already: SET `INPUT.SOURCE` to 2.
3. Watch NOTIFY 0x09 (or poll `CLOCK.I2S_SLAVE_STATUS`) for LOCKED; display the detected rate.
4. Audio starts automatically after lock + prefill. No further host action.
5. Persist with `STORE.PRESET_SAVE` as usual.

### Status UI guidance

- Map ACQUIRING/RELOCKING/INACTIVE (while mode=slave and source=I2S) to a
  single "waiting for external clock" indication with the raw `measured_hz`
  as a diagnostic (0 means no clocks at all; a nonzero value that never
  locks means an unsupported rate or wrong BCK ratio).
- In slave mode, gray out the sample-rate selector (`INPUT.I2S_RX_RATE`) or relabel it
  "rate used in master mode"; show the detected rate as read-only.
- Hide/disable MCK controls while slave mode is live (values are kept but
  the output is forced off).
- The lock/loss counters are cumulative since boot; deltas indicate cable or
  source stability problems.

### Sequencing detail

A `CLOCK.I2S_CLOCK_MODE` GET reflects the mode only after the deferred apply; if the app needs a
positive confirmation, wait for the PARAM_CHANGED on
`input_config.i2s_clock_mode` (or the first 0x09 event) after the SET.

---

## 8. Edge cases and known limitations

- **Non-I2S sources re-drive the clock pins.** By design the slave clocking
  applies only while I2S is the input source (there is no way to keep other
  sources' outputs on the external clock domain without resampling). See the
  dual-driver warning in section 2.
- **Supported rates are 44.1/48/96 kHz** (the pipeline's global set). An
  88.2/176.4/192 kHz master never locks; outputs stay muted.
- **BCK must be 64 x Fs and LRCLK polarity standard.** No runtime detection
  of other framings.
- **Whole-frame slips are undetectable.** The framing-slip watchdog catches
  every sub-frame misalignment, but a glitch burst that adds or drops
  exactly 64 BCK edges (one whole frame) with every intermediate per-frame
  check passing leaves a validly framed stream shifted one frame in time,
  which no LRCLK-level check can distinguish. In practice a burst that
  large trips a check (or the 5 ms clock-loss timeout) mid-way.
- **Extreme DMA-interrupt outages slip edge-slaved I2S output framing, but
  it now self-heals.** A DMA service outage longer than the joined TX FIFO
  (roughly 80 microseconds at 48 kHz) stalls the TX program against the
  external clock; the per-frame LRCLK check catches the resulting
  misalignment at the next frame boundary and forces the standard
  relock/re-frame instead of leaving it garbled until the next reset.
  Flash operations already suspend and restart the pipeline around their
  blackout.
- **Mute-and-wait on clock loss** means downstream SPDIF/ADAT DACs lose lock
  while external clocks are absent (the deliberate design choice; there is
  no internal-clock fallback).
- **Servo residual on exit.** Returning to master mode or another source
  restores nominal clocking through the standard rate-change path; the
  ppm-level servo trim on SPDIF dividers normalizes exactly as it does when
  leaving SPDIF input mode.
- **RP2040 vs RP2350**: identical behavior; RP2040 is limited to the single
  stereo input pair and has no ADAT, as in master mode.

---

## 9. Implementation map (for firmware developers)

| Piece | Location |
|---|---|
| Mode globals, pending flags, `i2s_slave_mode_active()` | `firmware/DSPi/audio_input.h/.c` |
| RX external role, rate measurement, lock FSM, servo, status | `firmware/DSPi/i2s_input.c` (`i2s_slave_*`), `i2s_input.h` |
| Checked RX PIO program (per-frame LRCLK verification) | `firmware/DSPi/i2s_input.pio` (`audio_i2s_rx_slave_checked`) |
| Framing-slip watchdog (flag read/clear, relock trigger) | `i2s_input.c` (`i2s_slave_slip_check`), `audio_i2s_multi.c` (`audio_i2s_extclk_framing_slipped`) |
| Edge-slaved TX PIO program (with the same per-frame verification) | `firmware/pico-extras/src/rp2_common/pico_audio_i2s_multi/audio_i2s_dataout_extclk.pio` |
| Library external-clock plumbing, program patch/reload, two-phase `enable_sync_prepare` | `audio_i2s_multi.c/.h` (and `audio_spdif.c/.h` for the SPDIF prepare half) |
| LRCLK-gated combined output start, main-loop slave block, deferred mode-change handler, type-switch clocking rebuild | `firmware/DSPi/main.c` |
| ADAT servo handoff | `firmware/DSPi/adat_output.c` (`adat_output_servo_divider`, resync divider pull) |
| Vendor handlers: `CLOCK` page (clock mode, slave status, role-indexed BCK pin, clock-pin mode), `INPUT.I2S_RX_RATE` gate | `firmware/DSPi/vendor/vc_page_clock.c`, `vc_page_input.c`, `pages/clock.def` |
| Clock-pin mode globals + `i2s_effective_bck_pin()` / `i2s_clock_pin_claimed()` helpers, `PICO_I2S_BCK_PIN_SLAVE` default | `firmware/DSPi/audio_input.h`, `config.h`, `usb_audio.c` (globals) |
| NOTIFY event 0x09 (push + formatter) | `firmware/DSPi/notify.c/.h` |
| Persistence (slot V29, directory V14, wire V21 with two claimed `WireI2SConfig` bytes) | `firmware/DSPi/flash_storage.c`, `bulk_params.h/.c` |
