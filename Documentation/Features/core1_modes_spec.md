# Core 1 Dual-Mode: EQ Worker + PDM

## Overview

Core 1 operates in one of three modes on **both platforms**, selected automatically based on which outputs are enabled. When PDM is disabled and any outputs in the Core 1 range are enabled, Core 1 runs as an EQ worker that processes per-output equalization, gain, delay, and S/PDIF conversion in parallel with Core 0.

### Platform Support

| Feature | RP2040 | RP2350 |
|---------|--------|--------|
| CORE1_MODE_IDLE | Yes | Yes |
| CORE1_MODE_PDM | Yes | Yes |
| CORE1_MODE_EQ_WORKER | Yes (outputs 2-3) | Yes (outputs 2-7) |
| Dual-core EQ split | Yes | Yes |
| REQ_GET_CORE1_MODE | Yes | Yes |
| REQ_GET_CORE1_CONFLICT | Yes | Yes |
| EQ worker data type | int32_t (Q28) | float |

---

## Core 1 Modes

| Value | Name | Trigger | Core 1 Activity |
|-------|------|---------|-----------------|
| 0 | `CORE1_MODE_IDLE` | No Core 1 outputs or PDM enabled | Sleeps via `__wfe()`, cpu1_load = 0 |
| 1 | `CORE1_MODE_PDM` | PDM output enabled | Sigma-delta modulation on PIO1 |
| 2 | `CORE1_MODE_EQ_WORKER` | Any Core 1 EQ output enabled, PDM disabled | EQ + gain + delay + S/PDIF for Core 1 outputs |

The mode is **derived** from the `matrix_mixer.outputs[]` enable state after each `REQ_SET_OUTPUT_ENABLE` command. There is no explicit "set mode" command. The derivation priority is:

**RP2350:** (9 outputs, indices 0-8)
1. If output index 8 (PDM) is enabled -> `CORE1_MODE_PDM`
2. Else if any output in indices 2-7 is enabled -> `CORE1_MODE_EQ_WORKER`
3. Else -> `CORE1_MODE_IDLE`

**RP2040:** (5 outputs, indices 0-4)
1. If output index 4 (PDM) is enabled -> `CORE1_MODE_PDM`
2. Else if any output in indices 2-3 is enabled -> `CORE1_MODE_EQ_WORKER`
3. Else -> `CORE1_MODE_IDLE`

---

## Output Index Map

The firmware uses zero-based output indices throughout the matrix mixer and vendor commands.

### RP2350 (9 outputs)

| Output Index | Physical Output | S/PDIF Pair | EQ Channel | Core Assignment |
|:---:|---|:---:|---|---|
| 0 | Out 1 (S/PDIF 1 L) | 1 | CH_OUT_1 (2) | Always Core 0 |
| 1 | Out 2 (S/PDIF 1 R) | 1 | CH_OUT_2 (3) | Always Core 0 |
| 2 | Out 3 (S/PDIF 2 L) | 2 | CH_OUT_3 (4) | Core 1 when EQ_WORKER |
| 3 | Out 4 (S/PDIF 2 R) | 2 | CH_OUT_4 (5) | Core 1 when EQ_WORKER |
| 4 | Out 5 (S/PDIF 3 L) | 3 | CH_OUT_5 (6) | Core 1 when EQ_WORKER |
| 5 | Out 6 (S/PDIF 3 R) | 3 | CH_OUT_6 (7) | Core 1 when EQ_WORKER |
| 6 | Out 7 (S/PDIF 4 L) | 4 | CH_OUT_7 (8) | Core 1 when EQ_WORKER |
| 7 | Out 8 (S/PDIF 4 R) | 4 | CH_OUT_8 (9) | Core 1 when EQ_WORKER |
| 8 | Out 9 (PDM Sub) | - | CH_OUT_9_PDM (10) | Always Core 0 (data); Core 1 (modulation) |

### RP2040 (5 outputs)

| Output Index | Physical Output | S/PDIF Pair | EQ Channel | Core Assignment |
|:---:|---|:---:|---|---|
| 0 | Out 1 (S/PDIF 1 L) | 1 | CH_OUT_1 (2) | Always Core 0 |
| 1 | Out 2 (S/PDIF 1 R) | 1 | CH_OUT_2 (3) | Always Core 0 |
| 2 | Out 3 (S/PDIF 2 L) | 2 | CH_OUT_3 (4) | Core 1 when EQ_WORKER |
| 3 | Out 4 (S/PDIF 2 R) | 2 | CH_OUT_4 (5) | Core 1 when EQ_WORKER |
| 4 | Out 5 (PDM Sub) | - | CH_OUT_5_PDM (6) | Always Core 0 (data); Core 1 (modulation) |

The constants `CORE1_EQ_FIRST_OUTPUT` (2) and `CORE1_EQ_LAST_OUTPUT` define the Core 1 EQ worker range (7 on RP2350, 3 on RP2040).

---

## Mutual Exclusion Interlock

### Constraint

Core 1 EQ worker outputs and the PDM output **cannot be simultaneously enabled** on either platform. They share Core 1: EQ worker outputs use it for EQ/gain/delay/S/PDIF processing, while PDM uses it for sigma-delta modulation.

| Platform | Core 1 EQ range | PDM output | Always-available |
|----------|----------------|------------|-----------------|
| RP2350 | Indices 2-7 | Index 8 | Indices 0-1 |
| RP2040 | Indices 2-3 | Index 4 | Indices 0-1 |

### Firmware Behavior

The interlock is enforced in the `REQ_SET_OUTPUT_ENABLE` handler on **both platforms**:

- **Enabling PDM:** The firmware checks whether any output in the Core 1 EQ range is currently enabled. If so, the enable request is **silently refused**.
- **Enabling a Core 1 EQ output:** The firmware checks whether the PDM output is currently enabled. If so, the enable request is **silently refused**.
- **Disabling any output:** Always succeeds. No conflict check is needed.

The firmware does **not** return an error code for refused enables. The request completes normally via USB control transfer; the app must read back the output state to confirm.

### Required App Pattern

Applications **must** follow this pattern to safely enable conflicting outputs:

```
1. Check conflict BEFORE attempting enable:
   REQ_GET_CORE1_CONFLICT(wValue=output_index) -> conflict (0 or 1)

2. If conflict == 0, proceed with enable:
   REQ_SET_OUTPUT_ENABLE(wValue=output_index, data=0x01)

3. ALWAYS read back to confirm the enable took effect:
   REQ_GET_OUTPUT_ENABLE(wValue=output_index) -> enabled (0 or 1)
```

To **switch** from one conflicting group to the other (e.g., disable Core 1 EQ outputs and enable PDM):

```
1. Disable all conflicting outputs first:
   REQ_SET_OUTPUT_ENABLE(wValue=2, data=0x00)
   REQ_SET_OUTPUT_ENABLE(wValue=3, data=0x00)
   ... (repeat for all enabled outputs in Core 1 EQ range)

2. Confirm mode has transitioned (optional):
   REQ_GET_CORE1_MODE() -> should return 0 (IDLE)

3. Enable the new output:
   REQ_SET_OUTPUT_ENABLE(wValue=PDM_index, data=0x01)

4. Confirm:
   REQ_GET_OUTPUT_ENABLE(wValue=PDM_index) -> 1
   REQ_GET_CORE1_MODE() -> should return 1 (PDM)
```

### UI Guidance

The control app should **grey out or disable** conflicting outputs in the UI to prevent the user from attempting invalid combinations:

- When any Core 1 EQ outputs are enabled: PDM should be greyed out
- When PDM is enabled: Core 1 EQ outputs should be greyed out
- Outputs 1-2 are always available on both platforms
- Display a tooltip or message explaining the hardware constraint

The `REQ_GET_CORE1_CONFLICT` command enables the app to query the conflict state for any specific output without maintaining its own shadow state.

---

## Vendor Commands

All commands use EP0 control transfers on the vendor interface. SET commands use Host-to-Device direction; GET commands use Device-to-Host direction.

### REQ_SET_OUTPUT_ENABLE (0x72)

Enables or disables a matrix mixer output. Subject to the mutual exclusion interlock.

| Field | Value |
|-------|-------|
| bmRequestType | 0x40 (Vendor, Host-to-Device) |
| bRequest | 0x72 |
| wValue | Output index (0-8 on RP2350, 0-4 on RP2040) |
| wIndex | 0 |
| wLength | 1 |
| Data | `0x01` = enable, `0x00` = disable |

**Behavior:**
- If enabling and a conflict exists, the request is silently ignored (output stays disabled).
- After applying the change, the firmware re-derives the Core 1 mode and transitions Core 1 if needed.
- Mode transitions are immediate: Core 1 exits its current loop at the next iteration boundary.

### REQ_GET_OUTPUT_ENABLE (0x73)

Reads the current enable state of an output.

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (Vendor, Device-to-Host) |
| bRequest | 0x73 |
| wValue | Output index (0-8 on RP2350, 0-4 on RP2040) |
| wIndex | 0 |
| wLength | 1 |
| Response | 1 byte: `0x01` = enabled, `0x00` = disabled |

### REQ_GET_CORE1_MODE (0x7A)

Returns the current Core 1 operating mode.

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (Vendor, Device-to-Host) |
| bRequest | 0x7A |
| wValue | 0 |
| wIndex | 0 |
| wLength | 1 |
| Response | 1 byte: `0` = IDLE, `1` = PDM, `2` = EQ_WORKER |

### REQ_GET_CORE1_CONFLICT (0x7B)

Pre-flight check: would enabling a given output conflict with the current state?

| Field | Value |
|-------|-------|
| bmRequestType | 0xC0 (Vendor, Device-to-Host) |
| bRequest | 0x7B |
| wValue | Proposed output index (0-8 on RP2350, 0-4 on RP2040) |
| wIndex | 0 |
| wLength | 1 |
| Response | 1 byte: `0x00` = no conflict, `0x01` = would conflict |

**Conflict rules checked (platform-adaptive):**
- If proposed output is the PDM index (RP2350: 8, RP2040: 4): conflict if any Core 1 EQ output is enabled
- If proposed output is in the Core 1 EQ range (RP2350: 2-7, RP2040: 2-3): conflict if PDM is enabled
- If proposed output is index 0-1: never conflicts (always returns 0)

### REQ_GET_STATUS (0x50)

The existing status command includes Core 1 CPU load. The `cpu1_load` field reflects the active mode:

- **IDLE mode:** `cpu1_load = 0`
- **PDM mode:** percentage of time spent in sigma-delta modulation (per ~1ms sample window)
- **EQ_WORKER mode:** percentage of time spent processing EQ + gain (per ~48ms block window)

Both loads are reported as `uint8_t` values in the range 0-100.

**Combined status (wValue=9):** 12-byte response, bytes 10-11 are cpu0_load and cpu1_load respectively.

**Legacy status (wValue=2):** 4-byte response, bits 16-23 are cpu0_load, bits 24-31 are cpu1_load.

---

## Audio Processing Pipeline

The DSP pipeline processes audio in 7 passes per USB audio packet (~48 samples at 48kHz, arriving every ~1ms):

```
Pass 1: Input conversion + Preamp + Loudness
Pass 2: Master EQ (L/R)
Pass 3: Crossfeed + Master peaks
Pass 4: Matrix mixing (all outputs)
Pass 5: Per-output EQ + Gain + Delay + S/PDIF  <-- DUAL-CORE SPLIT HERE
Pass 6: Remaining delay + S/PDIF (Core 0 outputs)
Pass 7: PDM output (if enabled, single-core fallback)
```

### Pass 5 Dual-Core Split (EQ_WORKER Mode)

When `core1_mode == CORE1_MODE_EQ_WORKER`, Core 1 handles EQ, gain, delay, and S/PDIF conversion for its assigned outputs in parallel with Core 0:

**RP2350:** Core 0 does outputs 0-1 (pair 1), Core 1 does outputs 2-7 (pairs 2-4)
**RP2040:** Core 0 does outputs 0-1 (pair 1), Core 1 does outputs 2-3 (pair 2)

```
Core 0 (USB IRQ context)              Core 1 (eq_worker_loop)
================================       ================================
Writes work descriptor:
  - sample_count
  - vol_mul
  - work_done = false
  __dmb()
  work_ready = true
  __sev()  ---------------------------->  Wakes from __wfe()
                                          __dmb()
                                          Reads descriptor

EQ + gain + delay + S/PDIF             EQ + gain + delay + S/PDIF
  for Core 0 outputs                     for Core 1 outputs
                                         (dsp_process_channel_block per ch)

while (!work_done) __wfe()  <----------  work_ready = false
__dmb()                                  __dmb()
                                         work_done = true
                                         __sev()

Continue with remaining                  __wfe() (sleeping)
  outputs (PDM if enabled)
```

**When NOT in EQ_WORKER mode** (IDLE or PDM), all outputs are processed on Core 0 in a single loop -- identical to the pre-dual-core behavior.

### Data Ownership During Parallel Window

During the parallel processing window (between `work_ready=true` and `work_done=true`):

**RP2350:**

| Resource | Core 0 Owns | Core 1 Owns | Shared |
|----------|:-----------:|:-----------:|:------:|
| `buf_out[0]`, `buf_out[1]` | Write | - | - |
| `buf_out[2]` .. `buf_out[7]` | - | Write | - |
| `buf_out[8]` (PDM) | Write | - | - |
| `filters[CH_OUT_1]`, `filters[CH_OUT_2]` | Read | - | - |
| `filters[CH_OUT_3]` .. `filters[CH_OUT_8]` | - | Read/Write (state) | - |
| `filters[CH_OUT_9_PDM]` | Read | - | - |
| `spdif_out[0]` (pair 1) | Write | - | - |
| `spdif_out[1..3]` (pairs 2-4) | - | Write | - |

**RP2040:**

| Resource | Core 0 Owns | Core 1 Owns | Shared |
|----------|:-----------:|:-----------:|:------:|
| `buf_out[0]`, `buf_out[1]` | Write | - | - |
| `buf_out[2]`, `buf_out[3]` | - | Write | - |
| `buf_out[4]` (PDM) | Write | - | - |
| `filters[CH_OUT_1]`, `filters[CH_OUT_2]` | Read | - | - |
| `filters[CH_OUT_3]`, `filters[CH_OUT_4]` | - | Read/Write (state) | - |
| `filters[CH_OUT_5_PDM]` | Read | - | - |
| `spdif_out[0]` (pair 1) | Write | - | - |
| `spdif_out[1]` (pair 2) | - | Write | - |

Both platforms: `matrix_mixer.outputs[]` and `channel_bypassed[]` are read-only shared.

There is **zero overlap** in write regions. Core 0 regains full access to all buffers after the `work_done` handshake completes.

---

## Mode Transitions

Mode transitions always pass through the mode dispatcher loop in `pdm_core1_entry()`:

```
                    +-----------+
         +--------->|   IDLE    |<---------+
         |          +-----------+          |
         |           ^         ^           |
    disable all     |           |     disable all
    Core 1 EQ       |           |     + PDM
    outputs         |           |           |
         v          |           |           v
    +-----------+   |           |   +-----------+
    | EQ_WORKER |---+           +---| PDM       |
    +-----------+                   +-----------+
      enable any                     enable
      Core 1 EQ                      PDM
      output                         output
```

**Direct EQ_WORKER <-> PDM transitions are impossible** due to the mutual exclusion interlock. The interlock guarantees that one group must be fully disabled before the other can be enabled, which means the mode always passes through IDLE.

### Transition Mechanism

1. `REQ_SET_OUTPUT_ENABLE` handler applies the enable/disable (with interlock check)
2. `derive_core1_mode()` computes the new mode from output state
3. If mode changed, `core1_mode` is written (word-sized volatile = atomic on ARM)
4. `__sev()` wakes Core 1
5. Core 1's current mode loop checks `core1_mode` at its next iteration boundary:
   - `pdm_processing_loop()` checks `while (core1_mode == CORE1_MODE_PDM)`
   - `eq_worker_loop()` checks `while (core1_mode == CORE1_MODE_EQ_WORKER)` and also inside the inner `__wfe()` wait
6. Loop exits, control returns to the dispatcher, which enters the new mode

### Transition Timing

- **IDLE -> EQ_WORKER:** Immediate (next `__sev()` wakes Core 1 from `__wfe()`)
- **IDLE -> PDM:** Immediate (same mechanism, plus PDM hardware re-init)
- **EQ_WORKER -> IDLE:** At next audio packet boundary (Core 1 finishes current work, checks mode, exits)
- **PDM -> IDLE:** At next PDM sample boundary (hardware is cleanly shut down)

---

## EQ Coefficient Safety

When the host application updates EQ parameters for a channel processed by Core 1 (RP2350: CH_OUT_3 through CH_OUT_8, indices 4-9; RP2040: CH_OUT_3 through CH_OUT_4, indices 4-5), the firmware uses a two-phase guard to prevent concurrent access:

### Phase 1: Wait for Core 1 idle

```c
if (is_core1_channel && core1_mode == CORE1_MODE_EQ_WORKER) {
    while (core1_eq_work.work_ready && !core1_eq_work.work_done) {
        tight_loop_contents();
    }
    __dmb();
}
```

This spin-waits until Core 1 has either finished its current work (`work_done == true`) or has no work pending (`work_ready == false`). In either case, Core 1 is not actively accessing the filter coefficients.

### Phase 2: Prevent new work dispatch

```c
uint32_t flags = save_and_disable_interrupts();
dsp_compute_coefficients(&p, &filters[p.channel][p.band], sample_rate);
// ... update channel_bypassed[] ...
restore_interrupts(flags);
```

Disabling interrupts prevents the USB audio packet callback from running, which means no new work can be dispatched to Core 1 during the coefficient update. This creates a brief atomic window (microseconds) where the coefficients are safely updated.

### App Considerations

- EQ parameter updates are **safe at any time** from the app's perspective. The firmware handles all synchronization internally.
- There is no need for the app to check Core 1 mode before sending `REQ_SET_EQ_PARAM`.
- The coefficient update may cause a brief increase in `cpu0_load` due to the spin-wait, but this is negligible (sub-microsecond).

---

## Flash Persistence

The Core 1 mode is **not stored in flash**. It is derived from the output enable states which **are** stored as part of the matrix mixer configuration in the V5 flash format.

On power-up:
1. `flash_load_params()` restores output enables from flash
2. `core0_init()` derives `core1_mode` from the restored output state
3. Core 1 is launched and enters the derived mode

The mutual exclusion invariant is guaranteed during flash save because it is enforced at the point of every `REQ_SET_OUTPUT_ENABLE`. A valid flash image can never contain a state where both outputs 2-7 and output 9 are simultaneously enabled.

---

## Data Types

### Core1Mode (enum)

```c
typedef enum {
    CORE1_MODE_IDLE      = 0,
    CORE1_MODE_PDM       = 1,
    CORE1_MODE_EQ_WORKER = 2,
} Core1Mode;
```

Defined in `config.h`. The integer values are stable and used in the `REQ_GET_CORE1_MODE` response.

### Core1EqWork (struct)

Platform-conditional types: float on RP2350, int32_t (Q28/Q15) on RP2040.

```c
typedef struct {
    volatile bool     work_ready;     // Set by Core 0, cleared by Core 1
    volatile bool     work_done;      // Set by Core 1, cleared by Core 0
#if PICO_RP2350
    float           (*buf_out)[192];  // Pointer to shared output buffer
    float             vol_mul;        // Master volume multiplier (float)
    int16_t          *spdif_out[3];   // S/PDIF buffers for pairs 2-4
#else
    int32_t         (*buf_out)[192];  // Pointer to shared output buffer (Q28)
    int32_t           vol_mul;        // Master volume multiplier (Q15)
    int16_t          *spdif_out[1];   // S/PDIF buffer for pair 2
#endif
    uint32_t          sample_count;   // Samples in current block (typically 48)
    uint32_t          delay_write_idx;// Current delay line write index
} Core1EqWork;
```

Defined in `config.h`. Global instance: `core1_eq_work` in `pdm_generator.c`.

### Memory Ordering

All flag transitions use explicit ARM data memory barriers:

| Transition | Sequence |
|-----------|----------|
| Core 0 dispatches work | Write descriptor fields -> `__dmb()` -> `work_ready = true` -> `__sev()` |
| Core 1 accepts work | Wake from `__wfe()` -> `__dmb()` -> Read descriptor fields |
| Core 1 completes work | `work_ready = false` -> `__dmb()` -> `work_done = true` -> `__sev()` |
| Core 0 resumes | Wake from `__wfe()` -> `__dmb()` -> Access all buf_out[] freely |

The `__dmb()` (Data Memory Barrier) ensures that all preceding memory writes are visible to the other core before the flag is observed. Without these barriers, the ARM memory model permits reordering that could cause Core 1 to read stale descriptor values or Core 0 to read stale output buffers.

---

## Diagnostic Checklist

| Symptom | Likely Cause | Check |
|---------|-------------|-------|
| cpu1_load stuck at 0 with Core 1 outputs enabled | Mode not transitioning | `REQ_GET_CORE1_MODE` -- should be 2 |
| Enabling PDM has no effect | Core 1 EQ outputs still enabled (interlock) | `REQ_GET_CORE1_CONFLICT(wValue=PDM_index)` |
| Enabling Core 1 output has no effect | PDM enabled (interlock) | `REQ_GET_CORE1_CONFLICT(wValue=output_index)` |
| Audio glitches on Core 1 outputs after EQ change | Coefficient race (should not happen) | Verify firmware has the coefficient guard |
| cpu1_load reads impossibly high | Metering formula mismatch | Verify EQ worker uses `(us * 137) >> 16` |
| Core 1 output audio silent but cpu1_load > 0 | Matrix routes not configured | Check `REQ_GET_MATRIX_ROUTE` for those outputs |

---

## Example: Full 8-Channel S/PDIF Setup (RP2350)

```
// Enable stereo pass-through on all 4 S/PDIF pairs
// (Assuming matrix routes are already configured)

for output_index in 0..7:
    // Check for conflict (only relevant for indices 2-7)
    conflict = REQ_GET_CORE1_CONFLICT(wValue=output_index)
    if conflict:
        error("Cannot enable output -- PDM is active")
        break

    REQ_SET_OUTPUT_ENABLE(wValue=output_index, data=0x01)

    // Confirm
    enabled = REQ_GET_OUTPUT_ENABLE(wValue=output_index)
    assert(enabled == 1)

// Verify dual-core mode is active
mode = REQ_GET_CORE1_MODE()
assert(mode == 2)  // CORE1_MODE_EQ_WORKER

// Monitor load distribution
status = REQ_GET_STATUS(wValue=9)  // 12-byte combined status
cpu0 = status[10]  // Should show reduced load vs single-core
cpu1 = status[11]  // Should show > 0, processing outputs 3-8
```

## Example: Switch from 8-Channel to PDM Sub (RP2350)

```
// Disable outputs 3-8 (indices 2-7) to free Core 1
for output_index in 2..7:
    REQ_SET_OUTPUT_ENABLE(wValue=output_index, data=0x00)

// Confirm transition to IDLE
mode = REQ_GET_CORE1_MODE()
assert(mode == 0)  // CORE1_MODE_IDLE

// Now enable PDM (no conflict)
conflict = REQ_GET_CORE1_CONFLICT(wValue=8)
assert(conflict == 0)

REQ_SET_OUTPUT_ENABLE(wValue=8, data=0x01)
enabled = REQ_GET_OUTPUT_ENABLE(wValue=8)
assert(enabled == 1)

mode = REQ_GET_CORE1_MODE()
assert(mode == 1)  // CORE1_MODE_PDM
```
