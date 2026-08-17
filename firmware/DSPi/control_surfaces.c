/*
 * control_surfaces.c; runtime engine for user-wired controls and indicators.
 *
 * Main-loop only.  A self-throttled 1 kHz tick polls every active binding:
 * button GPIOs are debounced and decoded per-pin into press / long-press /
 * double-press / hold gestures (several bindings may share one button GPIO
 * with distinct events), switches level-follow, encoders are decoded with a
 * quadrature transition table (optionally rate-accelerated), pots are read
 * via the ADC (one per tick, round-robin) with EMA smoothing and unit-aware
 * quantization, and LEDs (on/off or PWM-dimmed) are re-evaluated against
 * their noun's live state on a decimated schedule.  Every resulting change
 * is applied by dispatching the corresponding vendor command through
 * vendor_dispatch_set/get with CTRL_SOURCE_GPIO, so validation,
 * deferred-apply safety, and PARAM_SRC_GPIO host notifications all come
 * from the existing command surface.
 *
 * The engine is unit- and noun-agnostic: value semantics (dB vs Hz vs Q,
 * linear vs logarithmic stepping, live reads, dispatch mapping, target
 * validation) live in control_surfaces_nouns.c behind cs_noun_get /
 * cs_noun_dispatch / cs_noun_validate_target and the cs_noun_table.
 *
 * No PIO resources are used; polling at 1 kHz comfortably tracks
 * hand-operated detented encoders (a fast spin is ~250 quarter-steps per
 * second, 4 samples per transition).  PWM LEDs use the otherwise-unused
 * hardware PWM slices.  The one interrupt user is the IR component:
 * control_surfaces_ir.c timestamps receiver edges from IO_IRQ_BANK0 into a
 * private ring, decoded here on the tick; remote buttons then act exactly
 * like physical buttons through the shared op helpers.
 *
 * Serialization assumption (load-bearing): this tick, the host command
 * handlers (tud_task), and the binding-apply handler all run on the core0
 * main loop.  The unguarded ADC muxing and the shared EQ pending packet
 * rely on that; nothing here may move to ISR or timer context.
 */

#include "control_surfaces.h"
#include "control_surfaces_ir.h"
#include "control_surfaces_display.h"
#include "config.h"
#include "vendor_commands.h"
#include "flash_storage.h"
#include "notify.h"
#include "i2c_control.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"

#include <math.h>
#include <string.h>

// Poll cadence and input conditioning
#define CS_TICK_INTERVAL_US   1000u
#define CS_DEBOUNCE_TICKS     10      // 10 ms stable before a level is accepted
#define CS_POT_SEED_TICKS     50      // EMA settle time before the boot sync
#define CS_POT_DEADBAND       12      // raw ADC counts (~0.3%) before remapping
#define CS_LEVEL_UNKNOWN      0xFF

// Gesture timing (ticks = ms)
#define CS_LONG_TICKS         500     // hold this long = long press
#define CS_DOUBLE_TICKS       350     // second tap within this = double press
#define CS_REPEAT_DELAY_TICKS 400     // hold before auto-repeat starts
#define CS_REPEAT_RATE_TICKS  80      // one repeat per this many ticks (12.5/s)

// Encoder acceleration (CS_FLAG_ACCEL): step multiplier by inter-detent gap.
// Only continuous nouns accelerate; enums always step one position.
#define CS_ACCEL_GAP_X2       128     // slower than this = no acceleration
#define CS_ACCEL_GAP_X4       64
#define CS_ACCEL_GAP_X8       32

#define CS_SHADOW_TIMEOUT_TICKS  500  // drop a never-confirmed target after 500 ms

// Indicator refresh decimation: LEDs re-evaluate every 8 ticks (125 Hz),
// staggered by slot, bounding the per-tick cost of the heavier live reads
// (meter dB conversion, status-struct copies) while staying visually instant.
#define CS_IND_DECIM          8

// PWM LEDs: wrap 4095 at sysclk/16 (~2 kHz carrier); flicker-free and slow
// enough that the slice config is identical on both platforms.
#define CS_PWM_WRAP           4095
#define CS_PWM_CLKDIV         16.0f

// Log-unit quantization: 1/24 octave; linear units quantize to half a unit
// (half-dB, half-percent), matching the v1 pot behavior.
#define CS_LOG_QUANT_STEPS    24.0f
#define CS_LIN_QUANT          0.5f

// ADC-capable pot pins: GPIO 26..28 = ADC channels 0..2 on both platforms.
// GPIO 29 is the board's VSYS/3 monitor on Pico and Pico 2, so it is excluded.
#define CS_ADC_PIN_FIRST      26
#define CS_ADC_PIN_LAST       28

// Deferred SET handoff (vendor_commands.c writes, main.c consumes)
volatile bool    cs_set_binding_pending = false;
uint8_t          cs_set_binding_slot = 0;
CsBinding        cs_set_binding_val;
volatile uint8_t cs_last_status = PIN_CONFIG_SUCCESS;
volatile uint8_t cs_last_slot = 0;

// Deferred slot-name SET handoff (persist is a directory flash write)
volatile bool    cs_set_name_pending = false;
uint8_t          cs_set_name_slot = 0;
char             cs_set_name_val[CS_NAME_LEN];

// Deferred IR command SET / save / revert handoffs
volatile bool    cs_set_ir_cmd_pending = false;
uint8_t          cs_set_ir_cmd_slot = 0;
IrCommand        cs_set_ir_cmd_val;
volatile bool    cs_save_pending = false;
volatile bool    cs_revert_pending = false;

// Deferred display SET handoffs (caps v10)
volatile bool    cs_set_disp_cfg_pending = false;
CsDisplayCfg     cs_set_disp_cfg_val;
volatile bool    cs_set_disp_page_pending = false;
uint8_t          cs_set_disp_page_slot = 0;
CsDisplayPage    cs_set_disp_page_val;

// Deferred group / macro SET handoffs (caps v9)
volatile bool    cs_set_group_pending = false;
uint8_t          cs_set_group_slot = 0;
CsGroup          cs_set_group_val;
volatile bool    cs_set_macro_hdr_pending = false;
uint8_t          cs_set_macro_hdr_slot = 0;
CsMacroHeaderWire cs_set_macro_hdr_val;
volatile bool    cs_set_macro_step_pending = false;
uint8_t          cs_set_macro_step_slot = 0;
uint8_t          cs_set_macro_step_idx = 0;
CsMacroStep      cs_set_macro_step_val;

// ---------------------------------------------------------------------------
// Type capability table; the per-noun half lives in control_surfaces_nouns.c.
// REQ_GET_CS_CAPS serves these verbatim, so host UIs and the firmware can
// never disagree about which type/noun/action combinations are legal.
// ---------------------------------------------------------------------------

static const CsCapsHeader s_caps = {
    .caps_version = 13,
    .max_bindings = CS_MAX_BINDINGS,
    .type_count   = CS_TYPE_COUNT,
    .noun_count   = CS_NOUN_COUNT,
    .types = {
        [CS_TYPE_NONE]    = { 0, 0, CS_PINCLASS_ANY },
        [CS_TYPE_BUTTON]  = { CS_ACT_BIT(CS_ACT_INC) | CS_ACT_BIT(CS_ACT_DEC) |
                              CS_ACT_BIT(CS_ACT_TOGGLE) | CS_ACT_BIT(CS_ACT_SET) |
                              CS_ACT_BIT(CS_ACT_TRIGGER) | CS_ACT_BIT(CS_ACT_MOMENTARY),
                              1, CS_PINCLASS_ANY },
        [CS_TYPE_SWITCH]  = { CS_ACT_BIT(CS_ACT_FOLLOW), 1, CS_PINCLASS_ANY },
        [CS_TYPE_POT]     = { CS_ACT_BIT(CS_ACT_ADJUST), 1, CS_PINCLASS_ADC },
        [CS_TYPE_ENCODER] = { CS_ACT_BIT(CS_ACT_STEP), 2, CS_PINCLASS_ANY },
        [CS_TYPE_LED]     = { CS_ACT_BIT(CS_ACT_IND_EQUALS) | CS_ACT_BIT(CS_ACT_IND_ABOVE),
                              1, CS_PINCLASS_ANY },
        [CS_TYPE_LED_PWM] = { CS_ACT_BIT(CS_ACT_IND_EQUALS) | CS_ACT_BIT(CS_ACT_IND_ABOVE) |
                              CS_ACT_BIT(CS_ACT_IND_LEVEL), 1, CS_PINCLASS_ANY },
        // The IR mask describes what its COMMANDS may do (button semantics);
        // the container binding itself carries noun/action 0.
        [CS_TYPE_IR]      = { CS_ACT_BIT(CS_ACT_INC) | CS_ACT_BIT(CS_ACT_DEC) |
                              CS_ACT_BIT(CS_ACT_TOGGLE) | CS_ACT_BIT(CS_ACT_SET) |
                              CS_ACT_BIT(CS_ACT_TRIGGER) | CS_ACT_BIT(CS_ACT_MOMENTARY),
                              1, CS_PINCLASS_ANY },
        // Display container: gpio = SDA/SCL, content via commands 0x27-0x2B.
        [CS_TYPE_DISPLAY] = { 0, 2, CS_PINCLASS_ANY },
    },
    .max_ir_commands = CS_MAX_IR_COMMANDS,
    .max_groups      = CS_MAX_GROUPS,
    .max_macros      = CS_MAX_MACROS,
    .max_macro_steps = CS_MAX_MACRO_STEPS,
};

// ---------------------------------------------------------------------------
// Live state
// ---------------------------------------------------------------------------

// Per-operation dispatch state, shared by binding slots and IR commands
// (both step, retry-on-BUSY, shadow deferred nouns and support momentary).
typedef struct {
    // deferred dispatch (retried while the command surface reports BUSY)
    bool     pending;
    float    value;
    // deferred-apply nouns (CS_NDF_DEFERRED): step from the last dispatched
    // target while the live value catches up, so rapid detents are not
    // coalesced against a stale current value
    bool     shadow_active;
    float    shadow;
    int32_t  shadow_q;      // quantized target, cached so the confirm check
                            // costs one cs_quantize, not two (RP2040 softfloat)
    uint16_t shadow_age;    // ticks since dispatch (timeout guard)
    // momentary
    bool     mom_engaged;
    float    mom_restore;   // value captured at press, restored at release
} CsOpState;

typedef struct {
    bool     active;
    // switch (level-follow; buttons debounce in the per-pin gesture group)
    uint8_t  sw_stable;
    uint8_t  sw_candidate;
    uint8_t  sw_debounce;
    // encoder
    uint8_t  enc_prev;      // previous AB state (2 bits)
    int8_t   enc_accum;     // quarter-steps toward a detent
    uint16_t enc_gap;       // ticks since the last detent (saturating; accel)
    // pot
    uint16_t pot_filt;      // EMA-filtered 12-bit ADC value
    uint16_t pot_settle;    // seed ticks remaining before the boot sync
    uint16_t pot_sent_raw;  // filtered value at last dispatch
    int32_t  pot_sent_q;    // quantized units at last dispatch
    // LEDs
    uint8_t  led_lit;       // CS_LEVEL_UNKNOWN forces the first write
    uint8_t  led_raw;       // last raw indicator condition (delay filter input)
    uint8_t  led_cond;      // delay-filtered condition the pin follows
    uint32_t led_edge_ms;   // wall-clock ms at the last raw-condition change
    uint16_t pwm_level;     // last written PWM compare level
    uint32_t pv_key;        // PAGE_VALUE resolved-item key (session reset)
    CsOpState op;
} CsRuntime;

// Per-pin gesture state for buttons.  All button bindings sharing a GPIO
// share one debouncer and one gesture decoder; events fan out to bindings
// by their `event` field.
typedef struct {
    uint8_t  pin;           // CS_GPIO_UNUSED = entry free
    uint8_t  invert;        // shared CS_FLAG_INVERT (validated identical)
    uint8_t  stable;        // debounced logical level; CS_LEVEL_UNKNOWN at claim
    uint8_t  candidate;
    uint8_t  debounce;
    bool     has_long;      // any CS_EVT_LONG binding on this pin
    bool     has_double;    // any CS_EVT_DOUBLE binding on this pin
    bool     consumed;      // long/double already fired for this hold
    uint16_t hold_ticks;    // while held
    bool     wait_double;   // released after a short tap, double window open
    uint16_t wait_ticks;
    bool     repeat_on;     // auto-repeat running for this hold
    uint16_t repeat_ticks;
} CsBtnGroup;

static CsFlashConfig s_cfg;                      // live config (flash on save)
static char          s_names[CS_MAX_BINDINGS][CS_NAME_LEN];  // live slot names
static CsRuntime     s_rt[CS_MAX_BINDINGS];
static CsBtnGroup    s_btn[CS_MAX_BINDINGS];
static uint8_t       s_slot_status[CS_MAX_BINDINGS];
static bool          s_any_active = false;
static uint8_t       s_pot_rr = 0;               // round-robin pot cursor
static uint32_t      s_tick_ct = 0;              // decimation phase
static uint64_t      s_last_tick_us = 0;
static bool          s_dirty = false;            // live config != flash

// IR component state.  One CS_TYPE_IR binding (s_ir_slot) owns the receiver;
// commands fire by learned code.  Hold bookkeeping mirrors the button
// auto-repeat feel: repeats gate on the same delay and rate.
static CsIrConfig    s_ir;                       // live IR command table
static CsOpState     s_ir_op[CS_MAX_IR_COMMANDS];
static uint8_t       s_ir_cmd_status[CS_MAX_IR_COMMANDS];
static uint32_t      s_ir_pv_key[CS_MAX_IR_COMMANDS];  // PAGE_VALUE session keys
static uint8_t       s_ir_slot = 0xFF;           // slot of the live IR binding
static bool          s_ir_hold = false;          // a remote button is down
static uint16_t      s_ir_hold_ticks = 0;
static uint16_t      s_ir_fire_gap = 0;

// Defined in the IR command section below; used by cs_release_pins.
static void cs_ir_release_momentary(uint8_t sub);

// ---------------------------------------------------------------------------
// Group / macro state (caps v9)
// ---------------------------------------------------------------------------

// One in-flight grouped operation.  base[] is indexed by member channel and
// double-duties as the gesture-session capture (REL/ADJ, log units stored in
// log2 domain) and the momentary restore set (natural units).
typedef struct {
    uint8_t  mode;          // CS_GOP_*
    uint32_t member_mask;   // members resolved at op/session start
    uint32_t pend_mask;     // members whose dispatch is still owed (BUSY)
    uint16_t age;           // ticks since the last event (session expiry)
    float    delta;         // REL/ADJ accumulated offset (octaves for log units)
    float    abs_value;     // ABS/engage target; ADJ stores the capture mean
    float    base[NUM_CHANNELS];
} CsGroupOp;

#define CS_GOP_IDLE         0
#define CS_GOP_REL          1   // step session: target = base + delta
#define CS_GOP_ADJ          2   // pot offset law: same math, delta = pot - mean
#define CS_GOP_ABS          3   // one value for every member
#define CS_GOP_MOM_ENGAGE   4   // momentary held: abs_value out, base[] captured
#define CS_GOP_MOM_RESTORE  5   // momentary released: base[] back out

#define CS_GROUP_SESSION_TICKS  500   // idle gap that ends a gesture session

static CsGroupConfig s_groups;                   // live group table
static CsMacroConfig s_macros;                   // live macro table
static uint8_t       s_group_status[CS_MAX_GROUPS];
static uint8_t       s_macro_status[CS_MAX_MACROS];
static CsGroupOp     s_gop[CS_MAX_BINDINGS];

// Macro sequencer: one macro at a time; a new fire cancels the old one at
// its current step boundary.
// Grouped IR ops share a 2-deep context pool: a receiver delivers one key
// at a time, so per-command contexts would buy nothing; two covers a held
// grouped momentary plus one interleaved grouped press.  Pool exhaustion
// drops the new press.
typedef struct {
    bool      used;
    uint8_t   sub;
    CsGroupOp gop;
} CsIrGop;
#define CS_IR_GOP_POOL  2
static CsIrGop s_ir_gop[CS_IR_GOP_POOL];

static uint8_t   s_macro_run = 0xFF;             // running macro (0xFF idle)
static uint8_t   s_macro_step = 0;
static uint32_t  s_macro_delay = 0;              // ticks left before the step runs
static bool      s_macro_fired = false;          // step dispatched, draining BUSY
static CsOpState s_macro_op;                     // single-target step dispatches
static CsGroupOp s_macro_gop;                    // grouped step dispatches
// Snapshot of the step taken at fire time.  BUSY retries dispatch from this,
// never the live table, so a host editing the draining step cannot retarget
// an in-flight value.
static CsMacroStep s_macro_cur;

// Standard quadrature transition table, indexed by (prev << 2) | curr.
// Invalid two-bit jumps decode as 0 (skipped sample, no movement credited).
static const int8_t s_enc_table[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

// ---------------------------------------------------------------------------
// Unit math.  Natural units are float (dB, Hz, Q, percent, ms); wire fields
// are int16 (8.8 for dB/Q/percent/ms, plain integer for Hz).  HZ and Q step
// and quantize logarithmically (in octaves), the rest linearly.
// ---------------------------------------------------------------------------

static bool cs_unit_is_log(uint8_t unit) {
    return unit == CS_UNIT_HZ || unit == CS_UNIT_Q;
}

static float cs_decode(uint8_t unit, int16_t raw) {
    return (unit == CS_UNIT_HZ) ? (float)raw : (float)raw * (1.0f / 256.0f);
}

// Step size in natural units (linear) or octaves (log units).  step == 0
// selects the default: 1 dB, 1 percent, 0.1 ms, or 1/12 octave.
static float cs_step_size(const CsBinding *b, uint8_t unit) {
    if (b->step > 0) return (float)b->step * (1.0f / 256.0f);
    if (unit == CS_UNIT_MS) return 0.1f;
    return cs_unit_is_log(unit) ? (1.0f / 12.0f) : 1.0f;
}

static float cs_clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// The binding's effective span: the noun's full range, or the custom
// [range_min, range_max] when either field is non-zero (pot / IND_LEVEL).
static void cs_span(const CsBinding *b, const CsNounDesc *nd, float *lo, float *hi) {
    if (b->range_min != 0 || b->range_max != 0) {
        *lo = cs_decode(nd->unit, b->range_min);
        *hi = cs_decode(nd->unit, b->range_max);
    } else {
        *lo = cs_decode(nd->unit, nd->min_q);
        *hi = cs_decode(nd->unit, nd->max_q);
    }
}

bool cs_noun_span(uint8_t noun, float *lo, float *hi) {
    if (noun >= CS_NOUN_COUNT) return false;
    const CsNounDesc *nd = &cs_noun_table[noun];
    if (nd->kind != CS_KIND_CONTINUOUS) return false;
    float a = cs_decode(nd->unit, nd->min_q);
    float b = cs_decode(nd->unit, nd->max_q);
    if (!(b > a)) return false;
    *lo = a;
    *hi = b;
    return true;
}

// Quantize a natural value: half-units for linear nouns, 1/24 octave above
// the noun minimum for log nouns.  Dispatches reconstruct from the quantized
// value so a pot's output is stable and jitter-free.  Live reads can surface
// a zeroed filter recipe (freq 0) or garbage; guard the log/round domain.
static int32_t cs_quantize(const CsNounDesc *nd, float v) {
    if (!isfinite(v)) return 0;
    if (cs_unit_is_log(nd->unit)) {
        if (v <= 0.0f) return 0;
        float lo = cs_decode(nd->unit, nd->min_q);
        return (int32_t)lroundf(log2f(v / lo) * CS_LOG_QUANT_STEPS);
    }
    return (int32_t)lroundf(v / CS_LIN_QUANT);
}

static float cs_unquantize(const CsNounDesc *nd, int32_t q) {
    if (cs_unit_is_log(nd->unit)) {
        float lo = cs_decode(nd->unit, nd->min_q);
        return lo * exp2f((float)q / CS_LOG_QUANT_STEPS);
    }
    return (float)q * CS_LIN_QUANT;
}

// True once the live value has caught up with a dispatched deferred target
// (within half a quantization step), so the shadow can retire.  Compares
// against the quantized target cached at dispatch time (op->shadow_q).
static bool cs_shadow_confirmed(const CsNounDesc *nd, float live,
                                const CsOpState *op) {
    if (nd->kind != CS_KIND_CONTINUOUS)
        return (int)live == (int)op->shadow;
    return cs_quantize(nd, live) == op->shadow_q;
}

// ---------------------------------------------------------------------------
// Dispatch plumbing
// ---------------------------------------------------------------------------

// The op-state helpers take (binding, op) pairs so binding slots and IR
// commands (which present a binding-shaped view) share them unchanged.

static void cs_queue_op(const CsBinding *b, CsOpState *op, float value) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    // Overlay hook: a control adjusted this item (retries do not re-note).
    cs_display_note_adjust(b->noun, b->target, b->index, false);
    if (nd->dflags & CS_NDF_DEFERRED) {
        op->shadow_active = true;
        op->shadow = value;
        op->shadow_q = cs_quantize(nd, value);
        op->shadow_age = 0;
    }
    if (cs_noun_dispatch(b->noun, b->target, b->index, value)) {
        op->pending = false;
    } else {
        op->pending = true;
        op->value = value;
    }
}

// Base value for relative actions: the latched (not-yet-dispatched) target
// when one is pending, else the target shadow for deferred-apply nouns,
// else the live value.  Keeps rapid detents accumulating correctly across
// BUSY retries and across a preset load / EQ apply still in flight.
static float cs_base_value(const CsBinding *b, const CsOpState *op) {
    if (op->pending) return op->value;
    if (op->shadow_active) return op->shadow;
    return cs_noun_get(b->noun, b->target, b->index);
}

// ---------------------------------------------------------------------------
// Group fan-out engine.  A grouped op resolves every member's absolute
// target up front, then dispatches member by member; BUSY members stay in
// pend_mask and retry each tick.  Sessions replace the single-target shadow:
// bases are captured once per gesture, so detents never coalesce against a
// stale live value of a deferred-apply noun.  See the feature spec, s4.
// ---------------------------------------------------------------------------

// Defined in the Actions section below; the anchor rule needs it here.
static int cs_enum_step(const CsBinding *b, const CsOpState *op, int dir);

static bool cs_binding_grouped(const CsBinding *b) {
    return (b->flags & CS_FLAG_GROUP) != 0;
}

// A group is usable by a live op iff its stored record validated and it has
// members inside the noun's channel space.
static uint32_t cs_group_members(const CsBinding *b) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    if (b->target >= CS_MAX_GROUPS) return 0;
    if (s_group_status[b->target] != PIN_CONFIG_SUCCESS) return 0;
    uint32_t lim = (nd->target_count >= 32) ? 0xFFFFFFFFu
                                            : ((1u << nd->target_count) - 1u);
    return s_groups.groups[b->target].member_mask & lim;
}

static uint8_t cs_mask_lowest(uint32_t m) {
    return (uint8_t)__builtin_ctz(m);
}

// Capture the session bases (log units in log2 domain so delta is octaves).
static void cs_gop_capture(const CsBinding *b, CsGroupOp *gop, uint32_t members) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    bool logu = cs_unit_is_log(nd->unit);
    gop->member_mask = members;
    for (uint32_t m = members; m; m &= m - 1) {
        uint8_t ch = cs_mask_lowest(m);
        float v = cs_noun_get(b->noun, ch, b->index);
        if (logu) v = (v > 0.0f) ? log2f(v) : 0.0f;
        gop->base[ch] = v;
    }
}

// Member target for the current op, clamped to the noun's range.
static float cs_gop_target(const CsBinding *b, const CsGroupOp *gop, uint8_t ch) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    switch (gop->mode) {
        case CS_GOP_REL:
        case CS_GOP_ADJ: {
            float v = gop->base[ch] + gop->delta;
            if (cs_unit_is_log(nd->unit)) v = exp2f(v);
            float lo = cs_decode(nd->unit, nd->min_q);
            float hi = cs_decode(nd->unit, nd->max_q);
            return cs_clampf(v, lo, hi);
        }
        case CS_GOP_MOM_RESTORE:
            return gop->base[ch];
        default:  // ABS / MOM_ENGAGE
            return gop->abs_value;
    }
}

// Dispatch every pending member once; survivors retry next tick.  Only a
// finished momentary restore idles here; REL/ADJ/ABS stay up as the live
// gesture session (the tick ages them out) so rapid events on deferred
// nouns fold against the session instead of a stale live read.
static void cs_gop_pump(const CsBinding *b, CsGroupOp *gop) {
    for (uint32_t m = gop->pend_mask; m; m &= m - 1) {
        uint8_t ch = cs_mask_lowest(m);
        if (cs_noun_dispatch(b->noun, ch, b->index, cs_gop_target(b, gop, ch)))
            gop->pend_mask &= ~(1u << ch);
    }
    if (gop->pend_mask == 0 && gop->mode == CS_GOP_MOM_RESTORE)
        gop->mode = CS_GOP_IDLE;
}

// One value to every member (SET / FOLLOW / anchor results / pot LINK_ABS).
static void cs_group_abs(const CsBinding *b, CsGroupOp *gop, float value) {
    uint32_t members = cs_group_members(b);
    if (!members) return;
    cs_display_note_adjust(b->noun, b->target, b->index, true);
    gop->mode = CS_GOP_ABS;
    gop->member_mask = members;
    gop->abs_value = value;
    gop->pend_mask = members;
    gop->age = 0;
    cs_gop_pump(b, gop);
}

// Relative step, offset-preserving: each member walks from its own value
// captured at session start.  Re-queues every member on each detent so a
// BUSY straggler never applies a stale target.
static void cs_group_step(const CsBinding *b, CsGroupOp *gop, int dir, int steps) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    if (nd->kind == CS_KIND_CONTINUOUS) {
        uint32_t members = cs_group_members(b);
        if (!members) return;
        cs_display_note_adjust(b->noun, b->target, b->index, true);
        if (gop->mode != CS_GOP_REL || gop->age >= CS_GROUP_SESSION_TICKS ||
            gop->member_mask != members) {
            gop->mode = CS_GOP_REL;
            gop->delta = 0.0f;
            cs_gop_capture(b, gop, members);
        }
        gop->delta += (float)(dir * steps) * cs_step_size(b, nd->unit);
        gop->age = 0;
        gop->pend_mask = members;
        cs_gop_pump(b, gop);
    } else {
        // Anchor rule: compute the step against the lowest member, then
        // drive the whole group to the anchor's new value so a mixed group
        // becomes coherent on the first event.  A live ABS session stands
        // in for the anchor's value the way the single-target shadow does,
        // so detents on a deferred noun are not coalesced away.
        uint32_t members = cs_group_members(b);
        if (!members) return;
        CsBinding av = *b;
        av.flags &= (uint8_t)~CS_FLAG_GROUP;
        av.target = cs_mask_lowest(members);
        CsOpState tmp;
        memset(&tmp, 0, sizeof(tmp));
        if (gop->mode == CS_GOP_ABS && gop->member_mask == members) {
            tmp.shadow_active = true;
            tmp.shadow = gop->abs_value;
        }
        int t = cs_enum_step(&av, &tmp, dir);
        if (t >= 0) cs_group_abs(b, gop, (float)t);
    }
}

// Momentary on a group: capture per-member restores at the press, drive all
// members to `value`, restore each member on release.
static void cs_group_momentary_engage(const CsBinding *b, CsGroupOp *gop) {
    uint32_t members = cs_group_members(b);
    if (!members) return;
    cs_display_note_adjust(b->noun, b->target, b->index, true);
    cs_gop_capture(b, gop, members);
    gop->mode = CS_GOP_MOM_ENGAGE;
    gop->abs_value = (float)b->value;
    gop->pend_mask = members;
    gop->age = 0;
    cs_gop_pump(b, gop);
}

static void cs_group_momentary_release(const CsBinding *b, CsGroupOp *gop) {
    if (gop->mode != CS_GOP_MOM_ENGAGE) return;
    gop->mode = CS_GOP_MOM_RESTORE;
    gop->pend_mask = gop->member_mask;
    cs_gop_pump(b, gop);
}

// Grouped button press dispatch (the grouped half of cs_button_press).
static void cs_group_press(const CsBinding *b, CsGroupOp *gop) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    switch (b->action) {
        case CS_ACT_INC: cs_group_step(b, gop, +1, 1); break;
        case CS_ACT_DEC: cs_group_step(b, gop, -1, 1); break;
        case CS_ACT_TOGGLE: {
            uint32_t members = cs_group_members(b);
            if (!members) return;
            // The live ABS session is the anchor's effective value (shadow
            // rule); without it a double-tap on a deferred bool re-reads the
            // stale live value and the second press is lost.
            float anchor = (gop->mode == CS_GOP_ABS && gop->member_mask == members)
                         ? gop->abs_value
                         : cs_noun_get(b->noun, cs_mask_lowest(members), b->index);
            cs_group_abs(b, gop, anchor >= 0.5f ? 0.0f : 1.0f);
            break;
        }
        case CS_ACT_SET: {
            float v = (nd->kind == CS_KIND_CONTINUOUS)
                    ? cs_decode(nd->unit, b->value) : (float)b->value;
            cs_group_abs(b, gop, v);
            break;
        }
        default: break;   // TRIGGER is never grouped (validation)
    }
}

// Combined indicator condition over the members: ANY by default, ALL with
// CS_FLAG_GROUP_ALL.  An empty/invalid group reads as condition off.
static uint8_t cs_group_ind_cond(const CsBinding *b, bool above) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    uint32_t members = cs_group_members(b);
    if (!members) return 0;
    bool all = (b->flags & CS_FLAG_GROUP_ALL) != 0;
    for (uint32_t m = members; m; m &= m - 1) {
        uint8_t ch = cs_mask_lowest(m);
        float v = cs_noun_get(b->noun, ch, b->index);
        uint8_t c = above ? (v >= cs_decode(nd->unit, b->value) ? 1 : 0)
                          : ((int)v == (int)b->value ? 1 : 0);
        if (all && !c) return 0;
        if (!all && c) return 1;
    }
    return all ? 1 : 0;
}

// Pot ADJUST on a group.  LINK_ABS drives every member to the pot value;
// the default offset law moves the group mean to the pot value while each
// member keeps its offset from the mean captured at the session start
// (log units work in log2 domain so offsets are ratios).
static void cs_group_adjust(const CsBinding *b, CsGroupOp *gop, float value) {
    if (b->flags & CS_FLAG_LINK_ABS) {
        cs_group_abs(b, gop, value);
        return;
    }
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    uint32_t members = cs_group_members(b);
    if (!members) return;
    cs_display_note_adjust(b->noun, b->target, b->index, true);
    if (gop->mode != CS_GOP_ADJ || gop->age >= CS_GROUP_SESSION_TICKS ||
        gop->member_mask != members) {
        gop->mode = CS_GOP_ADJ;
        cs_gop_capture(b, gop, members);
        float sum = 0.0f;
        int n = 0;
        for (uint32_t m = members; m; m &= m - 1) {
            sum += gop->base[cs_mask_lowest(m)];
            n++;
        }
        gop->abs_value = sum / (float)n;   // capture mean
    }
    float v = cs_unit_is_log(nd->unit) ? ((value > 0.0f) ? log2f(value) : 0.0f)
                                       : value;
    gop->delta = v - gop->abs_value;
    gop->age = 0;
    gop->pend_mask = members;
    cs_gop_pump(b, gop);
}

// IND_LEVEL over a group follows the loudest member.
static float cs_group_ind_level(const CsBinding *b) {
    uint32_t members = cs_group_members(b);
    if (!members) return -INFINITY;
    float best = -INFINITY;
    for (uint32_t m = members; m; m &= m - 1) {
        float v = cs_noun_get(b->noun, cs_mask_lowest(m), b->index);
        if (v > best) best = v;
    }
    return best;
}

// ---------------------------------------------------------------------------
// PAGE_VALUE: a control that edits whatever the display currently shows.
// The target resolves at event time; op state carries over only while the
// resolved item is unchanged (keyed reset, like the macro sequencer).
// ---------------------------------------------------------------------------

// Forward decls from the Actions and validation sections.
static void cs_apply_step(const CsBinding *b, CsOpState *op, int dir, int steps);
static void cs_button_press(const CsBinding *b, CsOpState *op);
static uint8_t cs_validate_group_ref(const CsBinding *b);

// Navigate to the next/previous active page (always wraps).  From the
// "no page shown" state the first step lands on the edge page, not past it.
static void cs_display_page_nav(int dir) {
    uint16_t mask = cs_display_page_mask();
    if (!mask) return;
    uint8_t cur = cs_display_current_page();
    uint8_t n = (cur != 0xFF) ? cur
              : (dir > 0) ? (CS_MAX_DISPLAY_PAGES - 1) : 0;
    for (uint8_t i = 0; i < CS_MAX_DISPLAY_PAGES; i++) {
        n = (uint8_t)((n + CS_MAX_DISPLAY_PAGES + dir) % CS_MAX_DISPLAY_PAGES);
        if (mask & (1u << n)) break;
    }
    cs_display_select_page(n);
}

// dir/steps for STEP/INC/DEC; action CS_ACT_TOGGLE for a toggle press.
static void cs_pagevalue_apply(const CsBinding *pv, CsOpState *op, CsGroupOp *gop,
                               uint32_t *key, int dir, int steps, uint8_t action) {
    CsDisplayPage pg;
    if (!cs_display_resolve_current(&pg)) return;
    if ((control_surfaces_display_flash()->cfg.flags & CS_DCFG_EDIT_GATED) &&
        !cs_display_edit_armed()) {
        // Unarmed while gated: steps browse pages, toggles do nothing (the
        // edit button owns arming).
        if (action != CS_ACT_TOGGLE && dir != 0) cs_display_page_nav(dir);
        return;
    }
    const CsNounDesc *nd = &cs_noun_table[pg.noun];
    const uint16_t writers = CS_ACT_BIT(CS_ACT_STEP) | CS_ACT_BIT(CS_ACT_INC) |
        CS_ACT_BIT(CS_ACT_DEC) | CS_ACT_BIT(CS_ACT_SET) | CS_ACT_BIT(CS_ACT_TOGGLE);
    if ((nd->actions & writers) == 0) return;   // read-only page: no-op

    CsBinding v;
    memset(&v, 0, sizeof(v));
    v.type   = CS_TYPE_ENCODER;   // ignored by the op helpers
    v.noun   = pg.noun;
    v.target = pg.target;
    v.index  = pg.index;
    v.flags  = (uint8_t)(((pg.flags & CS_DPAGE_GROUP) ? CS_FLAG_GROUP : 0) |
                         (pv->flags & CS_FLAG_WRAP));

    uint32_t k = ((uint32_t)pg.noun << 16) | ((uint32_t)pg.target << 8) |
                 pg.index | ((pg.flags & CS_DPAGE_GROUP) ? 0x01000000u : 0);
    if (*key != k) {
        memset(op, 0, sizeof(*op));
        memset(gop, 0, sizeof(*gop));
        *key = k;
    }
    if (action == CS_ACT_TOGGLE) {
        if (nd->kind != CS_KIND_BOOL) return;
        v.action = CS_ACT_TOGGLE;
        if (cs_binding_grouped(&v)) cs_group_press(&v, gop);
        else                        cs_button_press(&v, op);
        return;
    }
    if (nd->kind == CS_KIND_BOOL) {
        // Detents on a bool page: up = on, down = off.
        v.action = CS_ACT_SET;
        v.value = (dir > 0) ? 1 : 0;
        if (cs_binding_grouped(&v)) cs_group_press(&v, gop);
        else                        cs_button_press(&v, op);
        return;
    }
    if (cs_binding_grouped(&v)) cs_group_step(&v, gop, dir, steps);
    else                        cs_apply_step(&v, op, dir, steps);
}

// Exported wrapper for the display module's page validation (the group
// table and statuses are engine-owned statics).
uint8_t cs_validate_grouped_target(const CsBinding *b) {
    return cs_validate_group_ref(b);
}

// Retry/pump view for a PAGE_VALUE op: the parked work belongs to the
// RESOLVED item (encoded in the session key), not the virtual noun, or a
// BUSY dispatch would be retried through PAGE_VALUE and silently dropped.
static CsBinding cs_pv_effective(const CsBinding *b, uint32_t key) {
    if (b->noun != CS_NOUN_PAGE_VALUE) return *b;
    CsBinding v;
    memset(&v, 0, sizeof(v));
    v.type   = CS_TYPE_ENCODER;
    v.noun   = (uint8_t)(key >> 16);
    v.target = (uint8_t)(key >> 8);
    v.index  = (uint8_t)key;
    v.flags  = (uint8_t)(((key & 0x01000000u) ? CS_FLAG_GROUP : 0) |
                         (b->flags & CS_FLAG_WRAP));
    return v;
}

// ---------------------------------------------------------------------------
// Actions
// ---------------------------------------------------------------------------

// Step an enum noun by dir (+1/-1).  Presets step across OCCUPIED slots
// only; an empty device is a no-op.  Returns the target index or -1 for
// no movement.
static int cs_enum_step(const CsBinding *b, const CsOpState *op, int dir) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    int count = nd->enum_count;
    int cur = (int)cs_base_value(b, op);
    bool wrap = (b->flags & CS_FLAG_WRAP) != 0;

    if (b->noun == CS_NOUN_PRESET || b->noun == CS_NOUN_DISPLAY_PAGE) {
        // Both step across OCCUPIED slots only (presets / active pages).
        uint16_t occ;
        if (b->noun == CS_NOUN_PRESET) {
            uint8_t sm, ds, la, ocm, mvm;
            preset_get_directory(&occ, &sm, &ds, &la, &ocm, &mvm);
        } else {
            occ = cs_display_page_mask();
            // "No page shown" reads 255; re-enter the range from the edge
            // so a non-WRAP control is not stuck outside it forever.
            if (cur >= count) cur = (dir > 0) ? -1 : count;
        }
        if (!occ) return -1;
        for (int i = 1; i <= count; i++) {
            int cand = cur + dir * i;
            if (wrap) cand = ((cand % count) + count) % count;
            else if (cand < 0 || cand >= count) return -1;
            if (occ & (1u << cand)) return (cand == cur) ? -1 : cand;
        }
        return -1;
    }

    int t = cur + dir;
    if (wrap) t = ((t % count) + count) % count;
    else t = t < 0 ? 0 : (t >= count ? count - 1 : t);
    return (t == cur) ? -1 : t;
}

// Apply `steps` relative steps (encoder detents or INC/DEC presses).
// `steps` carries the acceleration multiplier for continuous nouns; enums
// always move one position per event.
static void cs_apply_step(const CsBinding *b, CsOpState *op, int dir, int steps) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    if (nd->kind == CS_KIND_CONTINUOUS) {
        float lo = cs_decode(nd->unit, nd->min_q);
        float hi = cs_decode(nd->unit, nd->max_q);
        float d = (float)(dir * steps) * cs_step_size(b, nd->unit);
        float base = cs_base_value(b, op);
        float v = cs_unit_is_log(nd->unit) ? base * exp2f(d) : base + d;
        cs_queue_op(b, op, cs_clampf(v, lo, hi));
    } else {
        int t = cs_enum_step(b, op, dir);
        if (t >= 0) cs_queue_op(b, op, (float)t);
    }
}

static void cs_button_press(const CsBinding *b, CsOpState *op) {
    switch (b->action) {
        case CS_ACT_INC:     cs_apply_step(b, op, +1, 1); break;
        case CS_ACT_DEC:     cs_apply_step(b, op, -1, 1); break;
        case CS_ACT_TOGGLE:  cs_queue_op(b, op, cs_base_value(b, op) >= 0.5f ? 0.0f : 1.0f); break;
        case CS_ACT_TRIGGER: cs_queue_op(b, op, 0.0f); break;
        case CS_ACT_SET: {
            const CsNounDesc *nd = &cs_noun_table[b->noun];
            float v = (nd->kind == CS_KIND_CONTINUOUS)
                    ? cs_decode(nd->unit, b->value) : (float)b->value;
            cs_queue_op(b, op, v);
            break;
        }
        default: break;
    }
}

static void cs_momentary_engage(const CsBinding *b, CsOpState *op) {
    op->mom_restore = cs_base_value(b, op);
    op->mom_engaged = true;
    cs_queue_op(b, op, (float)b->value);
}

static void cs_momentary_release(const CsBinding *b, CsOpState *op) {
    if (!op->mom_engaged) return;
    op->mom_engaged = false;
    cs_queue_op(b, op, op->mom_restore);
}

// ---------------------------------------------------------------------------
// Button gesture groups
// ---------------------------------------------------------------------------

static CsBtnGroup *cs_group_for_pin(uint8_t pin) {
    for (int i = 0; i < CS_MAX_BINDINGS; i++)
        if (s_btn[i].pin == pin) return &s_btn[i];
    return NULL;
}

// Rebuild the per-pin gesture groups from the active button bindings.
// Debounce/gesture state of pins that persist across the rebuild is kept
// so editing an unrelated slot cannot glitch a held button.
static void cs_rebuild_groups(void) {
    CsBtnGroup old[CS_MAX_BINDINGS];
    memcpy(old, s_btn, sizeof(old));
    for (int i = 0; i < CS_MAX_BINDINGS; i++) {
        s_btn[i].pin = CS_GPIO_UNUSED;
    }
    int n = 0;
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        if (!s_rt[s].active || s_cfg.bindings[s].type != CS_TYPE_BUTTON) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        CsBtnGroup *g = NULL;
        for (int i = 0; i < n; i++)
            if (s_btn[i].pin == b->gpio[0]) { g = &s_btn[i]; break; }
        if (!g) {
            g = &s_btn[n++];
            // Carry over live state if this pin already had a group.
            CsBtnGroup *prev = NULL;
            for (int i = 0; i < CS_MAX_BINDINGS; i++)
                if (old[i].pin == b->gpio[0]) { prev = &old[i]; break; }
            if (prev) {
                *g = *prev;
            } else {
                memset(g, 0, sizeof(*g));
                g->pin = b->gpio[0];
                g->stable = CS_LEVEL_UNKNOWN;
                g->candidate = CS_LEVEL_UNKNOWN;
            }
            g->invert = (b->flags & CS_FLAG_INVERT) ? 1 : 0;
            g->has_long = false;
            g->has_double = false;
        }
        if (b->event == CS_EVT_LONG)   g->has_long = true;
        if (b->event == CS_EVT_DOUBLE) g->has_double = true;
    }
    // If a LONG/DOUBLE binding was just added to a pin whose button is
    // physically held right now, consume the in-progress hold; otherwise the
    // new LONG could fire mid-hold, breaking the "sync, not a press" rule.
    for (int i = 0; i < CS_MAX_BINDINGS; i++) {
        CsBtnGroup *g = &s_btn[i];
        if (g->pin == CS_GPIO_UNUSED || g->stable != 1) continue;
        for (int j = 0; j < CS_MAX_BINDINGS; j++) {
            if (old[j].pin != g->pin) continue;
            if ((g->has_long && !old[j].has_long) ||
                (g->has_double && !old[j].has_double))
                g->consumed = true;
            break;
        }
    }
}

// Fire every non-momentary button binding on `pin` bound to `event`.
// Repeat deliveries only reach CS_FLAG_REPEAT bindings.
static void cs_group_fire(const CsBtnGroup *g, uint8_t event, bool is_repeat) {
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (!s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        if (b->type != CS_TYPE_BUTTON || b->gpio[0] != g->pin) continue;
        if (b->action == CS_ACT_MOMENTARY) continue;
        if (b->event != event) continue;
        if (is_repeat && !(b->flags & CS_FLAG_REPEAT)) continue;
        if (b->noun == CS_NOUN_PAGE_VALUE) {
            int dir = (b->action == CS_ACT_DEC) ? -1
                    : (b->action == CS_ACT_INC) ? +1 : 0;
            cs_pagevalue_apply(b, &s_rt[s].op, &s_gop[s], &s_rt[s].pv_key,
                               dir, 1, b->action == CS_ACT_TOGGLE
                                       ? CS_ACT_TOGGLE : CS_ACT_STEP);
        } else if (cs_binding_grouped(b)) {
            cs_group_press(b, &s_gop[s]);
        } else {
            cs_button_press(b, &s_rt[s].op);
        }
    }
}

static void cs_group_momentary(const CsBtnGroup *g, bool engage) {
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (!s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        if (b->type != CS_TYPE_BUTTON || b->gpio[0] != g->pin) continue;
        if (b->action != CS_ACT_MOMENTARY) continue;
        if (cs_binding_grouped(b)) {
            if (engage) cs_group_momentary_engage(b, &s_gop[s]);
            else        cs_group_momentary_release(b, &s_gop[s]);
        } else {
            if (engage) cs_momentary_engage(b, &s_rt[s].op);
            else        cs_momentary_release(b, &s_rt[s].op);
        }
    }
}

// Gesture decoding, one pin per call.  Semantics:
// - No LONG/DOUBLE bindings on the pin: PRESS fires on the press edge
//   (immediate, v1 behavior); REPEAT bindings auto-repeat while held.
// - LONG present: PRESS fires on release before the long threshold; LONG
//   fires once when the threshold is reached while held.
// - DOUBLE present: a second press inside the window fires DOUBLE at that
//   press edge; a lone tap fires PRESS when the window expires.
// - MOMENTARY bindings engage on the press edge and restore on release,
//   independent of the gesture outcome.
// The first debounced level after a claim is a sync, not a press.
static void cs_tick_button_group(CsBtnGroup *g) {
    uint8_t raw = gpio_get(g->pin) ? 1 : 0;
    uint8_t level = g->invert ? raw : (uint8_t)!raw;

    if (level != g->candidate) {
        g->candidate = level;
        g->debounce = 0;
    } else if (g->candidate != g->stable) {
        if (++g->debounce >= CS_DEBOUNCE_TICKS) {
            g->debounce = 0;
            bool first = (g->stable == CS_LEVEL_UNKNOWN);
            g->stable = g->candidate;
            if (first && g->stable == 1) {
                // Held at claim/boot: a sync, not a press.  Consume the
                // whole hold so LONG cannot fire from a pre-boot press.
                g->consumed = true;
            }
            if (!first && g->stable == 1) {
                // Press edge
                g->hold_ticks = 0;
                g->repeat_on = false;
                if (g->wait_double) {
                    g->wait_double = false;
                    g->consumed = true;      // second tap: whole hold consumed
                    cs_group_fire(g, CS_EVT_DOUBLE, false);
                } else {
                    g->consumed = false;
                    if (!g->has_long && !g->has_double) {
                        cs_group_fire(g, CS_EVT_PRESS, false);
                        g->repeat_on = true;
                        g->repeat_ticks = 0;
                    }
                }
                cs_group_momentary(g, true);
            } else if (!first && g->stable == 0) {
                // Release edge
                cs_group_momentary(g, false);
                g->repeat_on = false;
                if (!g->consumed && (g->has_long || g->has_double)) {
                    if (g->has_double) {
                        g->wait_double = true;   // tap; single fires on expiry
                        g->wait_ticks = 0;
                    } else {
                        cs_group_fire(g, CS_EVT_PRESS, false);
                    }
                }
                g->consumed = false;
            }
        }
    } else {
        g->debounce = 0;
    }

    if (g->stable == 1) {
        if (g->hold_ticks < 0xFFFF) g->hold_ticks++;
        if (g->has_long && !g->consumed && g->hold_ticks >= CS_LONG_TICKS) {
            g->consumed = true;
            cs_group_fire(g, CS_EVT_LONG, false);
        }
        if (g->repeat_on && g->hold_ticks >= CS_REPEAT_DELAY_TICKS) {
            if (++g->repeat_ticks >= CS_REPEAT_RATE_TICKS) {
                g->repeat_ticks = 0;
                cs_group_fire(g, CS_EVT_PRESS, true);
            }
        }
    } else if (g->wait_double) {
        if (++g->wait_ticks >= CS_DOUBLE_TICKS) {
            g->wait_double = false;
            cs_group_fire(g, CS_EVT_PRESS, false);
        }
    }
}

// ---------------------------------------------------------------------------
// Per-type tick handlers (switch / encoder / pot / LEDs)
// ---------------------------------------------------------------------------

// Logical level: 1 = pressed / on.  Default wiring is component-to-GND with
// the internal pull-up; CS_FLAG_INVERT selects active-high with pull-down.
static uint8_t cs_read_level(const CsBinding *b, uint8_t pin) {
    uint8_t raw = gpio_get(pin) ? 1 : 0;
    return (b->flags & CS_FLAG_INVERT) ? raw : (uint8_t)!raw;
}

static void cs_tick_switch(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    uint8_t level = cs_read_level(b, b->gpio[0]);

    if (level != rt->sw_candidate) {
        rt->sw_candidate = level;
        rt->sw_debounce = 0;
        return;
    }
    if (rt->sw_candidate == rt->sw_stable) { rt->sw_debounce = 0; return; }
    if (++rt->sw_debounce < CS_DEBOUNCE_TICKS) return;
    rt->sw_debounce = 0;
    rt->sw_stable = rt->sw_candidate;
    // A switch is authoritative, including at claim/boot (matches the
    // pot's immediate-takeover semantics).
    if (cs_binding_grouped(b)) cs_group_abs(b, &s_gop[slot], (float)rt->sw_stable);
    else                       cs_queue_op(b, &rt->op, (float)rt->sw_stable);
}

static void cs_tick_encoder(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    if (rt->enc_gap < 0xFFFF) rt->enc_gap++;
    uint8_t curr = (uint8_t)((gpio_get(b->gpio[0]) ? 2 : 0) |
                             (gpio_get(b->gpio[1]) ? 1 : 0));
    int8_t d = s_enc_table[(rt->enc_prev << 2) | curr];
    rt->enc_prev = curr;
    if (!d) return;
    rt->enc_accum += (b->flags & CS_FLAG_REVERSE) ? -d : d;
    // One detent = 4 quarter-steps (both channels through a full cycle).
    int dir = 0;
    if (rt->enc_accum >= 4)       { rt->enc_accum -= 4; dir = +1; }
    else if (rt->enc_accum <= -4) { rt->enc_accum += 4; dir = -1; }
    if (!dir) return;

    int steps = 1;
    if (b->flags & CS_FLAG_ACCEL) {
        uint16_t gap = rt->enc_gap;
        if      (gap < CS_ACCEL_GAP_X8) steps = 8;
        else if (gap < CS_ACCEL_GAP_X4) steps = 4;
        else if (gap < CS_ACCEL_GAP_X2) steps = 2;
    }
    rt->enc_gap = 0;
    if (b->noun == CS_NOUN_PAGE_VALUE)
        cs_pagevalue_apply(b, &rt->op, &s_gop[slot], &rt->pv_key,
                           dir, steps, CS_ACT_STEP);
    else if (cs_binding_grouped(b)) cs_group_step(b, &s_gop[slot], dir, steps);
    else                            cs_apply_step(b, &rt->op, dir, steps);
}

// Map a filtered ADC reading onto the binding's span, quantized per unit.
static int32_t cs_pot_map(const CsBinding *b, const CsNounDesc *nd, uint16_t filt) {
    float lo, hi;
    cs_span(b, nd, &lo, &hi);
    float pos = (float)filt * (1.0f / 4095.0f);
    if (b->flags & CS_FLAG_REVERSE) pos = 1.0f - pos;
    float v = cs_unit_is_log(nd->unit) ? lo * exp2f(pos * log2f(hi / lo))
                                       : lo + (hi - lo) * pos;
    return cs_quantize(nd, v);
}

static void cs_tick_pot(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    CsRuntime *rt = &s_rt[slot];
    // The ADC input mux is unguarded; safe only because every ADC user (this
    // tick, the temperature read in vendor_commands.c) runs serialized in the
    // core0 main loop.  Do not move either to ISR/timer context.
    adc_select_input(b->gpio[0] - CS_ADC_PIN_FIRST);
    uint16_t raw = adc_read();
    // Truncate-toward-zero divide keeps the EMA directionally symmetric (an
    // arithmetic shift would creep on falling input but stall on rising).
    rt->pot_filt = (uint16_t)((int32_t)rt->pot_filt +
                              (((int32_t)raw - rt->pot_filt) / 8));

    if (rt->pot_settle) {
        // Immediate-takeover boot sync: once the EMA settles, the knob's
        // physical position becomes the value (the group mean for a grouped
        // pot under the offset law).
        if (--rt->pot_settle == 0) {
            rt->pot_sent_q = cs_pot_map(b, nd, rt->pot_filt);
            rt->pot_sent_raw = rt->pot_filt;
            float v = cs_unquantize(nd, rt->pot_sent_q);
            if (cs_binding_grouped(b)) cs_group_adjust(b, &s_gop[slot], v);
            else                       cs_queue_op(b, &rt->op, v);
        }
        return;
    }

    int32_t moved = (int32_t)rt->pot_filt - rt->pot_sent_raw;
    if (moved < 0) moved = -moved;
    if (moved <= CS_POT_DEADBAND) return;
    int32_t q = cs_pot_map(b, nd, rt->pot_filt);
    if (q != rt->pot_sent_q) {
        rt->pot_sent_q = q;
        rt->pot_sent_raw = rt->pot_filt;
        float v = cs_unquantize(nd, q);
        if (cs_binding_grouped(b)) cs_group_adjust(b, &s_gop[slot], v);
        else                       cs_queue_op(b, &rt->op, v);
    }
}

// TON/TOF filter for indicator conditions: the raw condition must hold for
// on_delay/off_delay (0.1 s units) before the output follows it.  Wall-clock
// so a slow loop cannot stretch minute-scale delays; the unsigned subtraction
// absorbs the ~49.7-day uint32 ms wrap.
static uint8_t cs_ind_delay(const CsBinding *b, CsRuntime *rt, uint8_t raw) {
    uint32_t now_ms = (uint32_t)(s_last_tick_us / 1000u);
    if (raw != rt->led_raw) {
        rt->led_raw = raw;
        rt->led_edge_ms = now_ms;
    }
    if (raw != rt->led_cond) {
        uint32_t delay_ms = (uint32_t)(raw ? b->on_delay : b->off_delay) * 100u;
        if (now_ms - rt->led_edge_ms >= delay_ms)
            rt->led_cond = raw;
    }
    return rt->led_cond;
}

static void cs_tick_led(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    CsRuntime *rt = &s_rt[slot];
    uint8_t raw;
    if (cs_binding_grouped(b)) {
        raw = cs_group_ind_cond(b, b->action == CS_ACT_IND_ABOVE);
    } else {
        float v = cs_noun_get(b->noun, b->target, b->index);
        if (b->action == CS_ACT_IND_ABOVE)
            raw = (v >= cs_decode(nd->unit, b->value)) ? 1 : 0;
        else
            raw = ((int)v == (int)b->value) ? 1 : 0;
    }
    uint8_t lit = cs_ind_delay(b, rt, raw);
    if (lit == rt->led_lit) return;
    rt->led_lit = lit;
    gpio_put(b->gpio[0], (b->flags & CS_FLAG_INVERT) ? !lit : lit);
}

static void cs_tick_led_pwm(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    CsRuntime *rt = &s_rt[slot];
    bool grouped = cs_binding_grouped(b);
    uint16_t level;
    if (b->action == CS_ACT_IND_LEVEL) {
        float v = grouped ? cs_group_ind_level(b)
                          : cs_noun_get(b->noun, b->target, b->index);
        float lo, hi;
        cs_span(b, nd, &lo, &hi);
        float norm = cs_unit_is_log(nd->unit)
                   ? log2f(v / lo) / log2f(hi / lo)
                   : (v - lo) / (hi - lo);
        // A zeroed recipe (freq 0) or garbage read yields -inf/NaN here;
        // NaN passes a min/max clamp, so gate explicitly.
        if (!(norm > 0.0f)) norm = 0.0f;
        else if (norm > 1.0f) norm = 1.0f;
        // Square the normalized position; a perceptually even sweep.
        level = (uint16_t)(norm * norm * (float)CS_PWM_WRAP);
    } else {
        uint8_t raw;
        if (grouped) {
            raw = cs_group_ind_cond(b, b->action == CS_ACT_IND_ABOVE);
        } else {
            float v = cs_noun_get(b->noun, b->target, b->index);
            raw = (b->action == CS_ACT_IND_ABOVE)
                ? (v >= cs_decode(nd->unit, b->value) ? 1 : 0)
                : ((int)v == (int)b->value ? 1 : 0);
        }
        level = cs_ind_delay(b, rt, raw) ? CS_PWM_WRAP : 0;
    }
    // Brightness ceiling, linear in duty: the meter's perceptual curve is
    // already baked into `level`, so this only lowers its top end.  Applied
    // before INVERT so an active-low LED keeps a full-rail off state.
    if (b->base_bright)
        level = (uint16_t)(((uint32_t)level * b->base_bright) / 100u);
    if (b->flags & CS_FLAG_INVERT) level = CS_PWM_WRAP - level;
    if (level == rt->pwm_level) return;
    rt->pwm_level = level;
    pwm_set_gpio_level(b->gpio[0], level);
}

// ---------------------------------------------------------------------------
// Pin claim / release and validation
// ---------------------------------------------------------------------------

static uint8_t cs_pin_count(uint8_t type) {
    return (type < CS_TYPE_COUNT) ? s_caps.types[type].pin_count : 0;
}

// True if another ACTIVE binding (not `except_slot`) uses `pin`.
static bool cs_pin_used_by_other(uint8_t pin, uint8_t except_slot) {
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s == except_slot || !s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        for (int i = 0; i < cs_pin_count(b->type); i++)
            if (b->gpio[i] == pin) return true;
    }
    return false;
}

// Sharing probe for button pins: every other active binding on `pin` must be
// a button with the same INVERT sense and a different event (momentary
// bindings count; two bindings may not claim the same pin+event).
static uint8_t cs_check_button_share(const CsBinding *nb, uint8_t except_slot) {
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s == except_slot || !s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        bool uses = false;
        for (int i = 0; i < cs_pin_count(b->type); i++)
            if (b->gpio[i] == nb->gpio[0]) uses = true;
        if (!uses) continue;
        if (b->type != CS_TYPE_BUTTON) return PIN_CONFIG_PIN_IN_USE;
        if ((b->flags ^ nb->flags) & CS_FLAG_INVERT) return CS_STATUS_INVALID_VALUE;
        if (b->event == nb->event) return CS_STATUS_EVENT_IN_USE;
    }
    return PIN_CONFIG_SUCCESS;
}

static void cs_claim_pins(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    switch (b->type) {
        case CS_TYPE_POT:
            adc_gpio_init(b->gpio[0]);
            break;
        case CS_TYPE_LED:
            gpio_init(b->gpio[0]);
            gpio_put(b->gpio[0], (b->flags & CS_FLAG_INVERT) ? 1 : 0);  // off
            gpio_set_dir(b->gpio[0], GPIO_OUT);
            break;
        case CS_TYPE_LED_PWM: {
            uint slice = pwm_gpio_to_slice_num(b->gpio[0]);
            // Wrap/div writes are idempotent (all CS slices use the same
            // config), so sharing a slice between two CS LEDs is safe.
            pwm_set_wrap(slice, CS_PWM_WRAP);
            pwm_set_clkdiv(slice, CS_PWM_CLKDIV);
            pwm_set_gpio_level(b->gpio[0], (b->flags & CS_FLAG_INVERT) ? CS_PWM_WRAP : 0);
            gpio_set_function(b->gpio[0], GPIO_FUNC_PWM);
            pwm_set_enabled(slice, true);
            break;
        }
        case CS_TYPE_DISPLAY:
            // Pin mux, pulls and the I2C peripheral are the module's job.
            cs_display_attach(b);
            break;
        default:  // button / switch / encoder / IR inputs (idempotent re-claims OK)
            for (int i = 0; i < cs_pin_count(b->type); i++) {
                gpio_init(b->gpio[i]);
                gpio_set_dir(b->gpio[i], GPIO_IN);
                if (b->flags & CS_FLAG_INVERT) gpio_pull_down(b->gpio[i]);
                else                           gpio_pull_up(b->gpio[i]);
            }
            break;
    }
}

// Release a slot's pins.  Shared button pins stay claimed while another
// active binding still uses them; a PWM slice is stopped only when no other
// CS PWM LED remains on it.  Caller has already marked the slot inactive.
static void cs_release_pins(uint8_t slot, const CsBinding *b) {
    if (b->type == CS_TYPE_DISPLAY) {
        cs_display_detach();   // stops the I2C peripheral; pins deinit below
    }
    if (b->type == CS_TYPE_IR) {
        cs_ir_detach();
        s_ir_slot = 0xFF;
        s_ir_hold = false;
        // Restore any engaged momentary before its state is discarded, so a
        // remote key held through a rebind/revert cannot leave a parameter
        // stuck at the engaged value.  Then pending dispatches die with the
        // receiver.
        for (uint8_t sub = 0; sub < CS_MAX_IR_COMMANDS; sub++)
            cs_ir_release_momentary(sub);
        memset(s_ir_op, 0, sizeof(s_ir_op));
        memset(s_ir_gop, 0, sizeof(s_ir_gop));
        // Detach aborts a learn in progress; tell the waiting host now,
        // since the tick no longer runs IR work without a component.
        if (cs_ir_learn_take_change())
            notify_push_cs_ir_learn(cs_ir_learn_state(), 0, 0);
    }
    for (int i = 0; i < cs_pin_count(b->type); i++) {
        uint8_t pin = b->gpio[i];
        if (cs_pin_used_by_other(pin, slot)) continue;
        if (b->type == CS_TYPE_LED_PWM) {
            uint slice = pwm_gpio_to_slice_num(pin);
            bool slice_busy = false;
            for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
                if (s == slot || !s_rt[s].active) continue;
                const CsBinding *o = &s_cfg.bindings[s];
                if (o->type == CS_TYPE_LED_PWM &&
                    pwm_gpio_to_slice_num(o->gpio[0]) == slice)
                    slice_busy = true;
            }
            if (!slice_busy) pwm_set_enabled(slice, false);
        }
        gpio_deinit(pin);
        gpio_disable_pulls(pin);
    }
}

static void cs_seed_runtime(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    memset(rt, 0, sizeof(*rt));
    switch (b->type) {
        case CS_TYPE_SWITCH:
            rt->sw_stable = CS_LEVEL_UNKNOWN;
            rt->sw_candidate = CS_LEVEL_UNKNOWN;
            break;
        case CS_TYPE_ENCODER:
            rt->enc_prev = (uint8_t)((gpio_get(b->gpio[0]) ? 2 : 0) |
                                     (gpio_get(b->gpio[1]) ? 1 : 0));
            rt->enc_gap = 0xFFFF;
            break;
        case CS_TYPE_POT:
            adc_select_input(b->gpio[0] - CS_ADC_PIN_FIRST);
            rt->pot_filt = adc_read();
            rt->pot_settle = CS_POT_SEED_TICKS;
            break;
        case CS_TYPE_LED:
            rt->led_lit = CS_LEVEL_UNKNOWN;
            break;
        case CS_TYPE_LED_PWM:
            rt->pwm_level = 0xFFFF;   // force the first write
            break;
        case CS_TYPE_IR:
            cs_ir_attach(b->gpio[0], (b->flags & CS_FLAG_INVERT) != 0);
            s_ir_slot = slot;
            s_ir_hold = false;
            memset(s_ir_op, 0, sizeof(s_ir_op));
            memset(s_ir_gop, 0, sizeof(s_ir_gop));
            break;
        default:   // buttons keep their state in the pin group
            break;
    }
    rt->active = true;
}

static void cs_recount_active(void) {
    s_any_active = false;
    for (int i = 0; i < CS_MAX_BINDINGS; i++)
        if (s_rt[i].active) { s_any_active = true; break; }
}

// Grouped-binding reference check: the group must exist, be valid and
// non-empty, and its kind must match the noun's channel space (DSP_BAND
// nouns take DSP_CH groups; the band index must be valid on every member).
static uint8_t cs_validate_group_ref(const CsBinding *b) {
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    if (b->target >= CS_MAX_GROUPS) return CS_STATUS_INVALID_GROUP;
    if (s_group_status[b->target] != PIN_CONFIG_SUCCESS)
        return CS_STATUS_INVALID_GROUP;
    const CsGroup *g = &s_groups.groups[b->target];
    uint8_t want;
    switch (nd->target_kind) {
        case CS_TARGET_INPUT_CH:  want = CS_TARGET_INPUT_CH;  break;
        case CS_TARGET_OUTPUT_CH: want = CS_TARGET_OUTPUT_CH; break;
        case CS_TARGET_DSP_CH:
        case CS_TARGET_DSP_BAND:  want = CS_TARGET_DSP_CH;    break;
        default: return CS_STATUS_INVALID_GROUP;   // untargeted noun
    }
    if (g->target_kind != want) return CS_STATUS_INVALID_GROUP;
    uint32_t lim = (nd->target_count >= 32) ? 0xFFFFFFFFu
                                            : ((1u << nd->target_count) - 1u);
    uint32_t members = g->member_mask & lim;
    if (!members) return CS_STATUS_INVALID_GROUP;
    for (uint32_t m = members; m; m &= m - 1) {
        uint8_t st = cs_noun_validate_target_ch(b->noun, cs_mask_lowest(m),
                                                b->index);
        if (st != PIN_CONFIG_SUCCESS) return st;
    }
    return PIN_CONFIG_SUCCESS;
}

// Group-flag context rules shared by bindings and macro steps: modifiers
// need GROUP, LINK_ABS is only meaningful on a grouped pot, GROUP_ALL only
// on a grouped boolean indicator condition.
static uint8_t cs_validate_group_flags(uint8_t flags, uint8_t action,
                                       const CsNounDesc *nd) {
    if (!(flags & CS_FLAG_GROUP)) {
        if (flags & (CS_FLAG_LINK_ABS | CS_FLAG_GROUP_ALL))
            return CS_STATUS_INVALID_VALUE;
        return PIN_CONFIG_SUCCESS;
    }
    if ((flags & CS_FLAG_LINK_ABS) &&
        (action != CS_ACT_ADJUST || nd->kind != CS_KIND_CONTINUOUS))
        return CS_STATUS_INVALID_VALUE;
    if ((flags & CS_FLAG_GROUP_ALL) &&
        action != CS_ACT_IND_EQUALS && action != CS_ACT_IND_ABOVE)
        return CS_STATUS_INVALID_VALUE;
    return PIN_CONFIG_SUCCESS;
}

// Value / step / range bounds per noun kind; shared by binding and IR
// command validation.
static uint8_t cs_validate_values(const CsBinding *b, const CsNounDesc *nd) {
    switch (nd->kind) {
        case CS_KIND_CONTINUOUS:
            if ((b->action == CS_ACT_SET || b->action == CS_ACT_IND_ABOVE) &&
                (b->value < nd->min_q || b->value > nd->max_q))
                return CS_STATUS_INVALID_VALUE;
            if (b->step < 0) return CS_STATUS_INVALID_VALUE;
            if ((b->action == CS_ACT_ADJUST || b->action == CS_ACT_IND_LEVEL) &&
                (b->range_min != 0 || b->range_max != 0)) {
                if (b->range_min >= b->range_max ||
                    b->range_min < nd->min_q || b->range_max > nd->max_q)
                    return CS_STATUS_INVALID_VALUE;
            }
            break;
        case CS_KIND_BOOL:
            if ((b->action == CS_ACT_SET || b->action == CS_ACT_IND_EQUALS ||
                 b->action == CS_ACT_MOMENTARY) &&
                (b->value < 0 || b->value > 1))
                return CS_STATUS_INVALID_VALUE;
            break;
        case CS_KIND_ENUM:
            if ((b->action == CS_ACT_SET || b->action == CS_ACT_IND_EQUALS) &&
                (b->value < 0 || b->value >= nd->enum_count))
                return CS_STATUS_INVALID_VALUE;
            break;
        default:
            break;
    }
    return PIN_CONFIG_SUCCESS;
}

// The IR binding is a container: the receiver pin and its idle sense are
// the only payload; nouns/actions belong to the commands.
static uint8_t cs_validate_ir_container(const CsBinding *b, uint8_t slot) {
    if (b->noun != 0 || b->action != 0 || b->event != 0 ||
        b->target != 0 || b->index != 0 ||
        b->value != 0 || b->step != 0 ||
        b->range_min != 0 || b->range_max != 0)
        return CS_STATUS_INVALID_VALUE;
    if (b->flags & (uint8_t)~CS_FLAG_INVERT) return CS_STATUS_INVALID_VALUE;
    // One receiver per device: a second IR slot would need its own capture
    // engine and a partitioned command table.
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s == slot || !s_rt[s].active) continue;
        if (s_cfg.bindings[s].type == CS_TYPE_IR) return CS_STATUS_IR_IN_USE;
    }
    return PIN_CONFIG_SUCCESS;
}

// The display binding is a container: SDA/SCL, model and address are the
// only payload; content comes through the display commands.  SDA must be
// an even GPIO and SCL the odd mux of the same I2C instance, and that
// instance must not carry the I2C target control interface.
static uint8_t cs_validate_display_container(const CsBinding *b, uint8_t slot) {
    if (b->noun != 0 || b->action != 0 || b->event != 0 ||
        b->target != 0 || b->step != 0 ||
        b->range_min != 0 || b->range_max != 0 || b->flags != 0)
        return CS_STATUS_INVALID_VALUE;
    if (b->index == CS_DISP_MODEL_NONE || b->index >= CS_DISP_MODEL_COUNT)
        return CS_STATUS_INVALID_VALUE;
    if (b->value != 0 && (b->value < 0x08 || b->value > 0x77))
        return CS_STATUS_INVALID_VALUE;
    if ((b->gpio[0] & 1) != 0 || (b->gpio[1] & 1) != 1 ||
        ((b->gpio[0] >> 1) & 1) != ((b->gpio[1] >> 1) & 1))
        return CS_STATUS_PIN_NOT_I2C;
    if (i2c_ctrl_live_instance() == (int)((b->gpio[0] >> 1) & 1))
        return CS_STATUS_I2C_IN_USE;
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s == slot || !s_rt[s].active) continue;
        if (s_cfg.bindings[s].type == CS_TYPE_DISPLAY)
            return CS_STATUS_DISPLAY_IN_USE;
    }
    return PIN_CONFIG_SUCCESS;
}

// Full validity check for a proposed binding.  Pin checks run against the
// live device with this slot's own pins already released by the caller.
static uint8_t cs_validate(const CsBinding *b, uint8_t slot) {
    if (b->type >= CS_TYPE_COUNT) return CS_STATUS_INVALID_TYPE;
    if (b->noun >= CS_NOUN_COUNT) return CS_STATUS_INVALID_NOUN;
    if (b->action >= CS_ACT_COUNT) return CS_STATUS_INVALID_ACTION;

    uint8_t gst = cs_validate_group_flags(b->flags, b->action,
                                          &cs_noun_table[b->noun]);
    if (gst != PIN_CONFIG_SUCCESS) return gst;
    for (int i = 0; i < (int)sizeof(b->reserved2); i++)
        if (b->reserved2[i] != 0) return CS_STATUS_INVALID_VALUE;

    // Brightness ceiling: PWM LEDs only, percent 1-100 with 0 = full.  Every
    // other type must carry 0, so a pre-v12 config stays valid unchanged.
    if (b->base_bright > 100) return CS_STATUS_INVALID_VALUE;
    if (b->base_bright != 0 && b->type != CS_TYPE_LED_PWM)
        return CS_STATUS_INVALID_VALUE;

    // Delays filter a boolean indicator condition; everything else (inputs,
    // IND_LEVEL, the IR container) must carry 0.
    if ((b->on_delay != 0 || b->off_delay != 0) &&
        !((b->type == CS_TYPE_LED || b->type == CS_TYPE_LED_PWM) &&
          (b->action == CS_ACT_IND_EQUALS || b->action == CS_ACT_IND_ABOVE)))
        return CS_STATUS_INVALID_VALUE;

    const CsTypeDesc *td = &s_caps.types[b->type];
    const CsNounDesc *nd = &cs_noun_table[b->noun];

    if (b->type == CS_TYPE_IR) {
        // Container slot: no noun/action/value semantics of its own; only
        // the shared pin checks below apply.
        uint8_t st = cs_validate_ir_container(b, slot);
        if (st != PIN_CONFIG_SUCCESS) return st;
    } else if (b->type == CS_TYPE_DISPLAY) {
        uint8_t st = cs_validate_display_container(b, slot);
        if (st != PIN_CONFIG_SUCCESS) return st;
    } else {
        uint16_t bit = CS_ACT_BIT(b->action);
        if (!(td->actions & bit) || !(nd->actions & bit))
            return CS_STATUS_INVALID_ACTION;

        // Events are a button concept; everything else must carry 0.
        if (b->type == CS_TYPE_BUTTON) {
            if (b->event >= CS_EVT_COUNT) return CS_STATUS_INVALID_EVENT;
            // Hold-to-repeat and hold-to-engage both own the hold; they only
            // make sense on the short-press event.
            if ((b->action == CS_ACT_MOMENTARY || (b->flags & CS_FLAG_REPEAT)) &&
                b->event != CS_EVT_PRESS)
                return CS_STATUS_INVALID_EVENT;
        } else if (b->event != 0) {
            return CS_STATUS_INVALID_EVENT;
        }
        if ((b->flags & CS_FLAG_REPEAT) &&
            (b->type != CS_TYPE_BUTTON ||
             (b->action != CS_ACT_INC && b->action != CS_ACT_DEC)))
            return CS_STATUS_INVALID_VALUE;
        if ((b->flags & CS_FLAG_ACCEL) && b->type != CS_TYPE_ENCODER)
            return CS_STATUS_INVALID_VALUE;

        uint8_t tst = cs_binding_grouped(b) ? cs_validate_group_ref(b)
                                            : cs_noun_validate_target(b);
        if (tst != PIN_CONFIG_SUCCESS) return tst;

        uint8_t vst = cs_validate_values(b, nd);
        if (vst != PIN_CONFIG_SUCCESS) return vst;

        // PAGE_VALUE resolves its target at event time; nothing static to
        // check the scalar fields against, so they must be zero.
        if (b->noun == CS_NOUN_PAGE_VALUE &&
            (b->value != 0 || b->step != 0 ||
             b->range_min != 0 || b->range_max != 0))
            return CS_STATUS_INVALID_VALUE;
    }

    // Pins: distinct, valid, unclaimed anywhere; the one sanctioned overlap
    // is several button bindings sharing a GPIO with distinct events.
    uint8_t n = td->pin_count;
    if (n == 2 && b->gpio[0] == b->gpio[1]) return PIN_CONFIG_INVALID_PIN;
    for (int i = 0; i < n; i++) {
        uint8_t pin = b->gpio[i];
        if (td->pin_class == CS_PINCLASS_ADC &&
            (pin < CS_ADC_PIN_FIRST || pin > CS_ADC_PIN_LAST))
            return CS_STATUS_PIN_NOT_ADC;
        if (control_surfaces_owns_pin(pin)) {
            if (b->type != CS_TYPE_BUTTON) return PIN_CONFIG_PIN_IN_USE;
            uint8_t st = cs_check_button_share(b, slot);
            if (st != PIN_CONFIG_SUCCESS) return st;
        } else {
            uint8_t st = ctrl_iface_check_pin(pin);
            if (st != PIN_CONFIG_SUCCESS) return st;
        }
    }

    // PWM LEDs on different GPIOs can still land on the same PWM slice
    // output (e.g. GPIO 0 and 16 are both slice 0 channel A on RP2040).
    if (b->type == CS_TYPE_LED_PWM) {
        uint slice = pwm_gpio_to_slice_num(b->gpio[0]);
        uint chan  = pwm_gpio_to_channel(b->gpio[0]);
        for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
            if (s == slot || !s_rt[s].active) continue;
            const CsBinding *o = &s_cfg.bindings[s];
            if (o->type == CS_TYPE_LED_PWM &&
                pwm_gpio_to_slice_num(o->gpio[0]) == slice &&
                pwm_gpio_to_channel(o->gpio[0]) == chan)
                return CS_STATUS_PWM_CONFLICT;
        }
    }
    return PIN_CONFIG_SUCCESS;
}

// ---------------------------------------------------------------------------
// IR commands.  Each is a button-shaped binding fired by a learned code; a
// stack view re-uses the shared step / press / momentary / validation
// helpers without a second code path.
// ---------------------------------------------------------------------------

static CsBinding cs_ir_view(const IrCommand *c) {
    CsBinding b;
    memset(&b, 0, sizeof(b));
    b.type   = CS_TYPE_IR;    // ignored by the op helpers
    b.noun   = c->noun;
    b.action = c->action;
    b.flags  = c->flags;
    b.target = c->target;
    b.index  = c->index;
    b.value  = c->value;
    b.step   = c->step;
    return b;
}

// A command fires only while the receiver is up and the record validated.
static bool cs_ir_cmd_live(uint8_t sub) {
    return s_ir_slot != 0xFF &&
           s_ir.cmds[sub].protocol != CS_IR_PROTO_NONE &&
           s_ir_cmd_status[sub] == PIN_CONFIG_SUCCESS;
}

static uint8_t cs_validate_ir_cmd(const IrCommand *c) {
    if (c->noun >= CS_NOUN_COUNT) return CS_STATUS_INVALID_NOUN;
    if (c->action >= CS_ACT_COUNT) return CS_STATUS_INVALID_ACTION;
    if (c->protocol >= CS_IR_PROTO_COUNT) return CS_STATUS_INVALID_VALUE;
    if (c->code == 0) return CS_STATUS_INVALID_VALUE;   // 0 = never learned
    if (c->reserved[0] != 0 || c->reserved[1] != 0) return CS_STATUS_INVALID_VALUE;
    // GROUP accepted since caps v10; a grouped remote key composes with
    // hold-to-repeat exactly like a grouped button.
    if (c->flags & (uint8_t)~(CS_FLAG_WRAP | CS_FLAG_REPEAT | CS_FLAG_GROUP))
        return CS_STATUS_INVALID_VALUE;
    if ((c->flags & CS_FLAG_REPEAT) &&
        c->action != CS_ACT_INC && c->action != CS_ACT_DEC)
        return CS_STATUS_INVALID_VALUE;
    if (c->noun == CS_NOUN_PAGE_VALUE && (c->value != 0 || c->step != 0))
        return CS_STATUS_INVALID_VALUE;

    const CsNounDesc *nd = &cs_noun_table[c->noun];
    uint16_t bit = CS_ACT_BIT(c->action);
    if (!(s_caps.types[CS_TYPE_IR].actions & bit) || !(nd->actions & bit))
        return CS_STATUS_INVALID_ACTION;

    CsBinding v = cs_ir_view(c);
    uint8_t tst = (c->flags & CS_FLAG_GROUP) ? cs_validate_group_ref(&v)
                                             : cs_noun_validate_target(&v);
    if (tst != PIN_CONFIG_SUCCESS) return tst;
    return cs_validate_values(&v, nd);
}

// Grouped IR context pool management.  Acquire finds the command's live
// context or claims a free one; NULL when exhausted (the press is dropped).
static CsGroupOp *cs_ir_gop_acquire(uint8_t sub) {
    for (int i = 0; i < CS_IR_GOP_POOL; i++)
        if (s_ir_gop[i].used && s_ir_gop[i].sub == sub) return &s_ir_gop[i].gop;
    for (int i = 0; i < CS_IR_GOP_POOL; i++) {
        if (!s_ir_gop[i].used) {
            s_ir_gop[i].used = true;
            s_ir_gop[i].sub = sub;
            memset(&s_ir_gop[i].gop, 0, sizeof(s_ir_gop[i].gop));
            return &s_ir_gop[i].gop;
        }
    }
    return NULL;
}

static CsGroupOp *cs_ir_gop_find(uint8_t sub) {
    for (int i = 0; i < CS_IR_GOP_POOL; i++)
        if (s_ir_gop[i].used && s_ir_gop[i].sub == sub) return &s_ir_gop[i].gop;
    return NULL;
}

// Best-effort momentary restore before a command's op state is discarded
// (receiver teardown or command overwrite).  A BUSY dispatch is dropped
// with the state; acceptable on a teardown path.
static void cs_ir_release_momentary(uint8_t sub) {
    CsBinding v = cs_ir_view(&s_ir.cmds[sub]);
    CsGroupOp *gop = cs_ir_gop_find(sub);
    if (gop) cs_group_momentary_release(&v, gop);
    CsOpState *op = &s_ir_op[sub];
    if (!op->mom_engaged) return;
    cs_momentary_release(&v, op);
}

uint8_t control_surfaces_apply_ir_cmd(uint8_t sub, const IrCommand *c) {
    if (sub >= CS_MAX_IR_COMMANDS || !c) return CS_STATUS_INVALID_SLOT;
    if (c->protocol == CS_IR_PROTO_NONE) {
        // Clearing a sub-slot: the record must be all-zero (strict, like
        // the binding reserved fields).
        const uint8_t *p = (const uint8_t *)c;
        for (size_t i = 0; i < sizeof(*c); i++)
            if (p[i] != 0) return CS_STATUS_INVALID_VALUE;
    } else {
        uint8_t st = cs_validate_ir_cmd(c);
        if (st != PIN_CONFIG_SUCCESS) return st;   // old command kept intact
    }
    cs_ir_release_momentary(sub);   // editing a held command restores first
    // Drop the sub-slot's group context with its op state: a BUSY leftover
    // would otherwise re-dispatch through the NEW command's noun.
    for (int i = 0; i < CS_IR_GOP_POOL; i++) {
        if (s_ir_gop[i].used && s_ir_gop[i].sub == sub) {
            s_ir_gop[i].used = false;
            memset(&s_ir_gop[i].gop, 0, sizeof(s_ir_gop[i].gop));
        }
    }
    s_ir.cmds[sub] = *c;
    memset(&s_ir_op[sub], 0, sizeof(s_ir_op[sub]));
    s_ir_cmd_status[sub] = PIN_CONFIG_SUCCESS;
    return PIN_CONFIG_SUCCESS;
}

// Fire every live command learned to {protocol, code}.  Multiple commands
// may share one remote button (one press, several actions).
static void cs_ir_fire(uint8_t protocol, uint32_t code, uint8_t evkind) {
    for (uint8_t sub = 0; sub < CS_MAX_IR_COMMANDS; sub++) {
        const IrCommand *c = &s_ir.cmds[sub];
        if (!cs_ir_cmd_live(sub) || c->protocol != protocol || c->code != code)
            continue;
        CsBinding v = cs_ir_view(c);
        CsOpState *op = &s_ir_op[sub];
        bool fire = (evkind == CS_IR_EVT_PRESS) ||
                    (evkind == CS_IR_EVT_REPEAT && (c->flags & CS_FLAG_REPEAT));
        if (c->noun == CS_NOUN_PAGE_VALUE) {
            if (!fire) continue;
            CsGroupOp *gop = cs_ir_gop_acquire(sub);
            if (!gop) continue;
            int dir = (c->action == CS_ACT_DEC) ? -1
                    : (c->action == CS_ACT_INC) ? +1 : 0;
            cs_pagevalue_apply(&v, op, gop, &s_ir_pv_key[sub], dir, 1,
                               c->action == CS_ACT_TOGGLE ? CS_ACT_TOGGLE
                                                          : CS_ACT_STEP);
            continue;
        }
        if (c->flags & CS_FLAG_GROUP) {
            CsGroupOp *gop = (evkind == CS_IR_EVT_RELEASE)
                           ? cs_ir_gop_find(sub) : cs_ir_gop_acquire(sub);
            if (!gop) continue;
            switch (evkind) {
                case CS_IR_EVT_PRESS:
                    if (c->action == CS_ACT_MOMENTARY)
                        cs_group_momentary_engage(&v, gop);
                    else
                        cs_group_press(&v, gop);
                    break;
                case CS_IR_EVT_REPEAT:
                    if (c->flags & CS_FLAG_REPEAT) cs_group_press(&v, gop);
                    break;
                case CS_IR_EVT_RELEASE:
                    if (c->action == CS_ACT_MOMENTARY)
                        cs_group_momentary_release(&v, gop);
                    break;
                default: break;
            }
            continue;
        }
        switch (evkind) {
            case CS_IR_EVT_PRESS:
                if (c->action == CS_ACT_MOMENTARY) cs_momentary_engage(&v, op);
                else                               cs_button_press(&v, op);
                break;
            case CS_IR_EVT_REPEAT:
                if (c->flags & CS_FLAG_REPEAT) cs_button_press(&v, op);
                break;
            case CS_IR_EVT_RELEASE:
                if (c->action == CS_ACT_MOMENTARY) cs_momentary_release(&v, op);
                break;
            default: break;
        }
    }
}

// Per-tick IR processing: decoded events and the button-compatible repeat
// gate (400 ms delay, then 12.5 Hz max).
static void cs_tick_ir(void) {
    if (s_ir_hold) {
        if (s_ir_hold_ticks < 0xFFFF) s_ir_hold_ticks++;
        if (s_ir_fire_gap  < 0xFFFF) s_ir_fire_gap++;
    }

    CsIrEvent ev;
    while (cs_ir_poll(&ev)) {
        switch (ev.kind) {
            case CS_IR_EVT_PRESS:
                s_ir_hold = true;
                s_ir_hold_ticks = 0;
                s_ir_fire_gap = 0;
                cs_ir_fire(ev.protocol, ev.code, CS_IR_EVT_PRESS);
                break;
            case CS_IR_EVT_REPEAT:
                if (s_ir_hold_ticks >= CS_REPEAT_DELAY_TICKS &&
                    s_ir_fire_gap >= CS_REPEAT_RATE_TICKS) {
                    s_ir_fire_gap = 0;
                    cs_ir_fire(ev.protocol, ev.code, CS_IR_EVT_REPEAT);
                }
                break;
            case CS_IR_EVT_RELEASE:
                s_ir_hold = false;
                cs_ir_fire(ev.protocol, ev.code, CS_IR_EVT_RELEASE);
                break;
            default: break;
        }
    }

    // One notify per learn completion (capture or timeout).
    if (cs_ir_learn_take_change()) {
        uint8_t proto = 0;
        uint32_t code = 0;
        uint8_t st = cs_ir_learn_state();
        if (st == CS_IR_LEARN_DONE) (void)cs_ir_learn_result(&proto, &code);
        notify_push_cs_ir_learn(st, proto, code);
    }
}

// ---------------------------------------------------------------------------
// Macros.  One sequencer; steps are button-shaped bindings executed on the
// tick after their pre_delay, re-validated at fire time so a torn edit or a
// since-emptied group skips the step instead of faulting.
// ---------------------------------------------------------------------------

// A step viewed as a binding, for the shared op / validation helpers.
static CsBinding cs_macro_step_view(const CsMacroStep *s) {
    CsBinding b;
    memset(&b, 0, sizeof(b));
    b.type   = CS_TYPE_BUTTON;   // ignored by the op helpers
    b.noun   = s->noun;
    b.action = s->action;
    b.flags  = s->flags;
    b.target = s->target;
    b.index  = s->index;
    b.value  = s->value;
    b.step   = s->step;
    return b;
}

#define CS_MACRO_STEP_ACTS  (CS_ACT_BIT(CS_ACT_SET) | CS_ACT_BIT(CS_ACT_TOGGLE) | \
                             CS_ACT_BIT(CS_ACT_INC) | CS_ACT_BIT(CS_ACT_DEC) | \
                             CS_ACT_BIT(CS_ACT_TRIGGER))

// All-zero marks an empty step (valid, skipped at run time).
static bool cs_macro_step_empty(const CsMacroStep *s) {
    const uint8_t *p = (const uint8_t *)s;
    for (size_t i = 0; i < sizeof(*s); i++)
        if (p[i] != 0) return false;
    return true;
}

static uint8_t cs_validate_macro_step(const CsMacroStep *s) {
    if (cs_macro_step_empty(s)) return PIN_CONFIG_SUCCESS;
    // No nesting; PAGE_VALUE is excluded too (a stored sequence editing
    // "whatever happens to be shown" is non-deterministic).
    if (s->noun >= CS_NOUN_COUNT || s->noun == CS_NOUN_MACRO ||
        s->noun == CS_NOUN_PAGE_VALUE)
        return CS_STATUS_INVALID_STEP;
    if (s->action >= CS_ACT_COUNT ||
        !(CS_MACRO_STEP_ACTS & CS_ACT_BIT(s->action)))
        return CS_STATUS_INVALID_STEP;
    if (s->flags & (uint8_t)~(CS_FLAG_WRAP | CS_FLAG_GROUP))
        return CS_STATUS_INVALID_STEP;
    if (s->reserved != 0) return CS_STATUS_INVALID_STEP;
    const CsNounDesc *nd = &cs_noun_table[s->noun];
    if (!(nd->actions & CS_ACT_BIT(s->action))) return CS_STATUS_INVALID_ACTION;
    CsBinding v = cs_macro_step_view(s);
    uint8_t tst = cs_binding_grouped(&v) ? cs_validate_group_ref(&v)
                                         : cs_noun_validate_target(&v);
    if (tst != PIN_CONFIG_SUCCESS) return tst;
    return cs_validate_values(&v, nd);
}

// Worst validity across a macro's executed steps, for the ext status.
static void cs_macro_recount_status(void) {
    for (uint8_t i = 0; i < CS_MAX_MACROS; i++) {
        uint8_t worst = PIN_CONFIG_SUCCESS;
        uint8_t n = s_macros.macros[i].step_count;
        if (n > CS_MAX_MACRO_STEPS) { s_macro_status[i] = CS_STATUS_INVALID_MACRO; continue; }
        for (uint8_t k = 0; k < n; k++) {
            uint8_t st = cs_validate_macro_step(&s_macros.macros[i].steps[k]);
            if (st != PIN_CONFIG_SUCCESS) worst = st;
        }
        s_macro_status[i] = worst;
    }
}

uint8_t cs_macro_running_index(void) { return s_macro_run; }

uint8_t control_surfaces_macro_fire(uint8_t idx) {
    if (idx >= CS_MAX_MACROS) return CS_STATUS_INVALID_MACRO;
    // Cancel-at-step-boundary: anything already dispatched stands, the old
    // macro's undelivered work is dropped with its op contexts.
    memset(&s_macro_op, 0, sizeof(s_macro_op));
    memset(&s_macro_gop, 0, sizeof(s_macro_gop));
    uint8_t n = s_macros.macros[idx].step_count;
    if (n == 0 || n > CS_MAX_MACRO_STEPS) {
        s_macro_run = 0xFF;              // empty macro: a successful no-op
        return PIN_CONFIG_SUCCESS;
    }
    s_macro_run = idx;
    s_macro_step = 0;
    s_macro_delay = (uint32_t)s_macros.macros[idx].steps[0].pre_delay * 10u;
    s_macro_fired = false;
    return PIN_CONFIG_SUCCESS;
}

void control_surfaces_macro_cancel(void) {
    s_macro_run = 0xFF;
    memset(&s_macro_op, 0, sizeof(s_macro_op));
    memset(&s_macro_gop, 0, sizeof(s_macro_gop));
}

// Advance to `step` (or finish), loading its pre-delay.
static void cs_macro_advance(uint8_t step) {
    const CsMacro *m = &s_macros.macros[s_macro_run];
    if (step >= m->step_count) {
        s_macro_run = 0xFF;
        return;
    }
    s_macro_step = step;
    s_macro_delay = (uint32_t)m->steps[step].pre_delay * 10u;
    s_macro_fired = false;
}

static void cs_tick_macro(void) {
    if (s_macro_run == 0xFF) return;
    const CsMacro *m = &s_macros.macros[s_macro_run];

    if (s_macro_fired) {
        // Drain the step's BUSY-pending work before the next step's delay
        // starts, so sequences serialize even across flash blackouts.
        if (s_macro_op.pending || s_macro_gop.pend_mask) return;
        cs_macro_advance((uint8_t)(s_macro_step + 1));
        return;
    }
    if (s_macro_delay > 0) { s_macro_delay--; return; }

    // A header SET may have cut step_count while this step sat in its
    // pre-delay; the shortened macro ends here instead of firing the cut tail.
    if (s_macro_step >= m->step_count) {
        s_macro_run = 0xFF;
        return;
    }
    const CsMacroStep *s = &m->steps[s_macro_step];
    if (cs_macro_step_empty(s) ||
        cs_validate_macro_step(s) != PIN_CONFIG_SUCCESS) {
        cs_macro_advance((uint8_t)(s_macro_step + 1));   // skip, not fatal
        return;
    }
    CsBinding v = cs_macro_step_view(s);
    // Op state carries over only between consecutive steps addressing the
    // same noun/target (so a repeated INC accumulates against its shadow);
    // otherwise a stale session/shadow would pollute the new step.
    static uint8_t prev_key[4];   // zero-init keeps this out of .data
    static bool prev_key_valid;
    uint8_t key[4] = {s->noun, s->target, s->index,
                      (uint8_t)(s->flags & CS_FLAG_GROUP)};
    if (!prev_key_valid || memcmp(key, prev_key, sizeof(key)) != 0) {
        memset(&s_macro_op, 0, sizeof(s_macro_op));
        memset(&s_macro_gop, 0, sizeof(s_macro_gop));
        memcpy(prev_key, key, sizeof(prev_key));
        prev_key_valid = true;
    }
    s_macro_cur = *s;
    if (cs_binding_grouped(&v)) cs_group_press(&v, &s_macro_gop);
    else                        cs_button_press(&v, &s_macro_op);
    s_macro_fired = true;
}

// ---------------------------------------------------------------------------
// Group / macro apply (deferred SET consumers) and dependent re-validation
// ---------------------------------------------------------------------------

static uint8_t cs_validate_group(const CsGroup *g) {
    if (g->target_kind == CS_TARGET_NONE) {
        // Clearing: the record must be all-zero (strict, like bindings).
        const uint8_t *p = (const uint8_t *)g;
        for (size_t i = 0; i < sizeof(*g); i++)
            if (p[i] != 0) return CS_STATUS_INVALID_VALUE;
        return PIN_CONFIG_SUCCESS;
    }
    uint8_t count;
    switch (g->target_kind) {
        case CS_TARGET_INPUT_CH:  count = NUM_INPUT_CHANNELS;  break;
        case CS_TARGET_OUTPUT_CH: count = NUM_OUTPUT_CHANNELS; break;
        case CS_TARGET_DSP_CH:    count = NUM_CHANNELS;        break;
        default: return CS_STATUS_INVALID_GROUP;
    }
    for (int i = 0; i < (int)sizeof(g->reserved); i++)
        if (g->reserved[i] != 0) return CS_STATUS_INVALID_VALUE;
    uint32_t lim = (count >= 32) ? 0xFFFFFFFFu : ((1u << count) - 1u);
    if (g->member_mask == 0 || (g->member_mask & ~lim))
        return CS_STATUS_INVALID_GROUP;
    return PIN_CONFIG_SUCCESS;
}

// Re-validate one grouped binding after its group was edited: down it
// (releasing pins) when it no longer validates, bring it back up when it
// does again.  Mirrors the boot rule; there is no in-use refusal.  A still-
// valid active binding keeps its runtime (no pot re-seed, no LED glitch);
// only its group context resets, since the member set may have changed.
static void cs_revalidate_grouped_slot(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    if (s_rt[slot].active) {
        // A group edit cannot change pin validity, so an active binding
        // only needs the group-dependent half re-checked.
        uint8_t st = cs_validate_group_ref(b);
        if (st == PIN_CONFIG_SUCCESS) {
            cs_group_momentary_release(b, &s_gop[slot]);
            memset(&s_gop[slot], 0, sizeof(s_gop[slot]));
            s_slot_status[slot] = PIN_CONFIG_SUCCESS;
            return;
        }
        cs_group_momentary_release(b, &s_gop[slot]);
        s_rt[slot].active = false;
        cs_release_pins(slot, b);
        memset(&s_gop[slot], 0, sizeof(s_gop[slot]));
        s_slot_status[slot] = st;
        return;
    }
    // Previously-down slot: full check (its pins were released), reclaim on
    // success so a repaired group resurrects its bindings.
    memset(&s_gop[slot], 0, sizeof(s_gop[slot]));
    uint8_t st = cs_validate(b, slot);
    if (st == PIN_CONFIG_SUCCESS) {
        cs_claim_pins(slot);
        cs_seed_runtime(slot);
    }
    s_slot_status[slot] = st;
}

uint8_t control_surfaces_apply_group(uint8_t idx, const CsGroup *g) {
    if (idx >= CS_MAX_GROUPS || !g) return CS_STATUS_INVALID_SLOT;
    uint8_t st = cs_validate_group(g);
    if (st != PIN_CONFIG_SUCCESS) return st;   // stored group kept intact
    s_groups.groups[idx] = *g;
    s_groups.groups[idx].name[CS_NAME_LEN - 1] = '\0';
    s_group_status[idx] = PIN_CONFIG_SUCCESS;
    // Only bindings referencing THIS group can change validity.
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        const CsBinding *b = &s_cfg.bindings[s];
        if (b->type == CS_TYPE_NONE || b->type == CS_TYPE_IR) continue;
        if (!cs_binding_grouped(b) || b->target != idx) continue;
        cs_revalidate_grouped_slot(s);
    }
    // Grouped IR commands referencing this group revalidate too; a stale
    // kind mismatch would otherwise keep dispatching to the wrong channels.
    for (uint8_t sub = 0; sub < CS_MAX_IR_COMMANDS; sub++) {
        const IrCommand *c = &s_ir.cmds[sub];
        if (c->protocol == CS_IR_PROTO_NONE ||
            !(c->flags & CS_FLAG_GROUP) || c->target != idx)
            continue;
        cs_ir_release_momentary(sub);
        for (int i = 0; i < CS_IR_GOP_POOL; i++) {
            if (s_ir_gop[i].used && s_ir_gop[i].sub == sub) {
                s_ir_gop[i].used = false;
                memset(&s_ir_gop[i].gop, 0, sizeof(s_ir_gop[i].gop));
            }
        }
        s_ir_cmd_status[sub] = cs_validate_ir_cmd(c);
    }
    cs_recount_active();
    cs_rebuild_groups();
    cs_macro_recount_status();
    return PIN_CONFIG_SUCCESS;
}

uint8_t control_surfaces_apply_macro_header(uint8_t idx, const CsMacroHeaderWire *h) {
    if (idx >= CS_MAX_MACROS || !h) return CS_STATUS_INVALID_MACRO;
    if (h->step_count > CS_MAX_MACRO_STEPS) return CS_STATUS_INVALID_MACRO;
    for (int i = 0; i < (int)sizeof(h->reserved); i++)
        if (h->reserved[i] != 0) return CS_STATUS_INVALID_VALUE;
    CsMacro *m = &s_macros.macros[idx];
    memcpy(m->name, h->name, CS_NAME_LEN);
    m->name[CS_NAME_LEN - 1] = '\0';
    m->step_count = h->step_count;
    // A running macro whose tail was just cut finishes at the new boundary
    // (cs_macro_advance re-checks step_count each advance).
    cs_macro_recount_status();
    return PIN_CONFIG_SUCCESS;
}

uint8_t control_surfaces_apply_macro_step(uint8_t idx, uint8_t step,
                                          const CsMacroStep *s) {
    if (idx >= CS_MAX_MACROS) return CS_STATUS_INVALID_MACRO;
    if (step >= CS_MAX_MACRO_STEPS || !s) return CS_STATUS_INVALID_STEP;
    uint8_t st = cs_validate_macro_step(s);
    if (st != PIN_CONFIG_SUCCESS) return st;   // stored step kept intact
    s_macros.macros[idx].steps[step] = *s;
    cs_macro_recount_status();
    return PIN_CONFIG_SUCCESS;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool control_surfaces_owns_pin(uint8_t pin) {
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        if (!s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        for (int i = 0; i < cs_pin_count(b->type); i++)
            if (b->gpio[i] == pin) return true;
    }
    return false;
}

uint8_t control_surfaces_apply_binding(uint8_t slot, const CsBinding *nb) {
    if (slot >= CS_MAX_BINDINGS || !nb) return CS_STATUS_INVALID_SLOT;

    // Release this slot first so the new pins can reuse them; restore the
    // old binding untouched if the new one fails validation.  An engaged
    // momentary restores best-effort before its state is discarded, so a
    // rebind mid-hold cannot leave the parameter stuck at the held value.
    CsBinding old = s_cfg.bindings[slot];
    bool was_active = s_rt[slot].active;
    if (was_active) {
        if (cs_binding_grouped(&old)) cs_group_momentary_release(&old, &s_gop[slot]);
        else                          cs_momentary_release(&old, &s_rt[slot].op);
        s_rt[slot].active = false;
        cs_release_pins(slot, &old);
    }
    memset(&s_gop[slot], 0, sizeof(s_gop[slot]));

    if (nb->type != CS_TYPE_NONE) {
        uint8_t st = cs_validate(nb, slot);
        if (st != PIN_CONFIG_SUCCESS) {
            if (was_active) {
                s_cfg.bindings[slot] = old;
                cs_claim_pins(slot);
                cs_seed_runtime(slot);
            }
            cs_recount_active();
            cs_rebuild_groups();
            return st;
        }
    }

    s_cfg.bindings[slot] = *nb;
    memset(&s_rt[slot], 0, sizeof(s_rt[slot]));
    if (nb->type != CS_TYPE_NONE) {
        cs_claim_pins(slot);
        cs_seed_runtime(slot);
    }
    s_slot_status[slot] = PIN_CONFIG_SUCCESS;
    cs_recount_active();
    cs_rebuild_groups();
    return PIN_CONFIG_SUCCESS;
}

// Bring up every stored binding and IR command from the directory cache.
// Shared by boot init and REQ_CS_REVERT; the caller has already cleared or
// released the live state.  A stored entry that no longer validates is kept
// visible (inactive) so the config survives round-trips; the failure is
// reported in slot_status / ir_cmd_status.
// The 388-byte CsFlashConfig and 260-byte CsIrConfig snapshots are kept in
// separate noinline frames: merged, they would sit on the 2 KB main stack at
// once, above whatever depth the apply path below them needs.
static void __attribute__((noinline)) cs_load_stored_bindings(void) {
    CsFlashConfig stored;
    preset_get_cs_config(&stored);
    if (stored.version > CS_CONFIG_VERSION) return;   // future format stays idle
    for (uint8_t slot = 0; slot < CS_MAX_BINDINGS; slot++) {
        const CsBinding *b = &stored.bindings[slot];
        if (b->type == CS_TYPE_NONE) continue;
        uint8_t st = control_surfaces_apply_binding(slot, b);
        if (st != PIN_CONFIG_SUCCESS) {
            s_cfg.bindings[slot] = *b;
            s_slot_status[slot] = st;
        }
    }
}

static void __attribute__((noinline)) cs_load_stored_ir(void) {
    CsIrConfig ir;
    preset_get_cs_ir_config(&ir);
    if (ir.version > CS_IR_CONFIG_VERSION) return;
    for (uint8_t sub = 0; sub < CS_MAX_IR_COMMANDS; sub++) {
        const IrCommand *c = &ir.cmds[sub];
        if (c->protocol == CS_IR_PROTO_NONE) continue;
        uint8_t st = cs_validate_ir_cmd(c);
        s_ir.cmds[sub] = *c;
        s_ir_cmd_status[sub] = st;
    }
}

// Groups and macros load BEFORE bindings: grouped bindings validate against
// the live group table.  A stored record that fails validation is kept
// (round-trips through save) but marked, so refs treat it as absent.
// Copies straight into the live tables; a stack snapshot of the 1060-byte
// macro blob would not fit the 2 KB main stack beside the binding frames.
static void cs_load_stored_groups_macros(void) {
    preset_get_cs_groups(&s_groups);
    if (s_groups.version > CS_GROUP_CONFIG_VERSION) {
        memset(&s_groups, 0, sizeof(s_groups));   // future format stays idle
        s_groups.version = CS_GROUP_CONFIG_VERSION;
    }
    for (uint8_t i = 0; i < CS_MAX_GROUPS; i++) {
        s_groups.groups[i].name[CS_NAME_LEN - 1] = '\0';
        s_group_status[i] = cs_validate_group(&s_groups.groups[i]);
    }
    preset_get_cs_macros(&s_macros);
    if (s_macros.version > CS_MACRO_CONFIG_VERSION) {
        memset(&s_macros, 0, sizeof(s_macros));
        s_macros.version = CS_MACRO_CONFIG_VERSION;
    }
    for (uint8_t i = 0; i < CS_MAX_MACROS; i++) {
        s_macros.macros[i].name[CS_NAME_LEN - 1] = '\0';
        if (s_macros.macros[i].step_count > CS_MAX_MACRO_STEPS)
            s_macros.macros[i].step_count = CS_MAX_MACRO_STEPS;
    }
    cs_macro_recount_status();
}

static void cs_load_stored(void) {
    cs_load_stored_groups_macros();
    // Display pages load before bindings so a stored display binding's
    // attach sees its stored pages (seeding only fires on an empty table).
    cs_display_load_stored();
    cs_load_stored_bindings();
    cs_load_stored_ir();
    for (uint8_t slot = 0; slot < CS_MAX_BINDINGS; slot++)
        preset_get_cs_name(slot, s_names[slot]);
}

// Shared group/macro/display state reset for init and revert: sequencer
// idle, group contexts cleared, live tables zeroed for the reload.
static void cs_reset_groups_macros(void) {
    memset(&s_groups, 0, sizeof(s_groups));
    s_groups.version = CS_GROUP_CONFIG_VERSION;
    memset(&s_macros, 0, sizeof(s_macros));
    s_macros.version = CS_MACRO_CONFIG_VERSION;
    memset(s_group_status, 0, sizeof(s_group_status));
    memset(s_macro_status, 0, sizeof(s_macro_status));
    memset(s_gop, 0, sizeof(s_gop));
    memset(s_ir_gop, 0, sizeof(s_ir_gop));
    memset(s_ir_pv_key, 0, sizeof(s_ir_pv_key));
    control_surfaces_macro_cancel();
    cs_display_reset();
}

void control_surfaces_init(void) {
    memset(&s_cfg, 0, sizeof(s_cfg));
    memset(s_rt, 0, sizeof(s_rt));
    memset(s_slot_status, 0, sizeof(s_slot_status));
    for (int i = 0; i < CS_MAX_BINDINGS; i++) s_btn[i].pin = CS_GPIO_UNUSED;
    s_cfg.version = CS_CONFIG_VERSION;
    memset(&s_ir, 0, sizeof(s_ir));
    s_ir.version = CS_IR_CONFIG_VERSION;
    memset(s_ir_op, 0, sizeof(s_ir_op));
    memset(s_ir_cmd_status, 0, sizeof(s_ir_cmd_status));
    s_ir_slot = 0xFF;
    s_ir_hold = false;
    cs_reset_groups_macros();
    cs_load_stored();
}

void control_surfaces_revert(void) {
    // Tear down every live binding through the normal path so pins, the PWM
    // slices and the IR receiver release cleanly, then reload from flash.
    CsBinding none;
    memset(&none, 0, sizeof(none));
    for (uint8_t slot = 0; slot < CS_MAX_BINDINGS; slot++) {
        (void)control_surfaces_apply_binding(slot, &none);
        s_slot_status[slot] = PIN_CONFIG_SUCCESS;
    }
    memset(&s_ir, 0, sizeof(s_ir));
    s_ir.version = CS_IR_CONFIG_VERSION;
    memset(s_ir_op, 0, sizeof(s_ir_op));
    memset(s_ir_cmd_status, 0, sizeof(s_ir_cmd_status));
    cs_reset_groups_macros();
    cs_load_stored();
}

void control_surfaces_tick(void) {
    // A macro fired by the host (REQ_CS_MACRO_FIRE) must run with zero
    // bindings active, so the sequencer keeps the tick alive too.
    if (!s_any_active && s_macro_run == 0xFF) return;
    uint64_t now = time_us_64();
    if (now - s_last_tick_us < CS_TICK_INTERVAL_US) return;
    s_last_tick_us = now;
    s_tick_ct++;

    // Retry ops parked on a BUSY dispatch, and retire deferred-apply target
    // shadows once the live value confirms them (or on timeout), before
    // sampling new events.
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        CsRuntime *rt = &s_rt[s];
        if (!rt->active) continue;
        // PAGE_VALUE ops retry through their resolved item, not the noun.
        CsBinding eb = cs_pv_effective(&s_cfg.bindings[s], rt->pv_key);
        const CsBinding *b = &eb;
        CsOpState *op = &rt->op;
        if (op->pending &&
            cs_noun_dispatch(b->noun, b->target, b->index, op->value))
            op->pending = false;
        // Confirm checks are slot-staggered to every 4th tick: the live read
        // plus quantize is the engine's costliest recurring float work on the
        // RP2040, and retiring a shadow a few ms late is harmless.
        if (op->shadow_active) {
            if (++op->shadow_age >= CS_SHADOW_TIMEOUT_TICKS)
                op->shadow_active = false;
            else if (((s_tick_ct + s) & 3u) == 0 &&
                     cs_shadow_confirmed(&cs_noun_table[b->noun],
                                         cs_noun_get(b->noun, b->target, b->index),
                                         op))
                op->shadow_active = false;
        }
    }

    // Grouped ops: retry BUSY members and age out gesture sessions (REL /
    // ADJ / ABS; momentary holds never age).
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        CsGroupOp *gop = &s_gop[s];
        if (!s_rt[s].active || gop->mode == CS_GOP_IDLE) continue;
        CsBinding eb = cs_pv_effective(&s_cfg.bindings[s], s_rt[s].pv_key);
        const CsBinding *b = &eb;
        if (gop->pend_mask) cs_gop_pump(b, gop);
        if (gop->mode != CS_GOP_MOM_ENGAGE && gop->mode != CS_GOP_MOM_RESTORE &&
            gop->age < 0xFFFF && ++gop->age >= CS_GROUP_SESSION_TICKS &&
            gop->pend_mask == 0)
            gop->mode = CS_GOP_IDLE;
    }

    // Macro sequencer: retry its op contexts (with the same session aging),
    // then advance the sequence.  Retries use the fire-time step snapshot,
    // not the live table, so concurrent step edits cannot retarget them.
    if (s_macro_run != 0xFF) {
        CsBinding v = cs_macro_step_view(&s_macro_cur);
        if (s_macro_op.pending &&
            cs_noun_dispatch(v.noun, v.target, v.index, s_macro_op.value))
            s_macro_op.pending = false;
        if (s_macro_op.shadow_active &&
            ++s_macro_op.shadow_age >= CS_SHADOW_TIMEOUT_TICKS)
            s_macro_op.shadow_active = false;
        if (s_macro_gop.pend_mask) cs_gop_pump(&v, &s_macro_gop);
        if (s_macro_gop.mode != CS_GOP_IDLE &&
            s_macro_gop.age < 0xFFFF &&
            ++s_macro_gop.age >= CS_GROUP_SESSION_TICKS &&
            s_macro_gop.pend_mask == 0)
            s_macro_gop.mode = CS_GOP_IDLE;
        cs_tick_macro();
    }

    // Grouped IR contexts: pump BUSY members, age sessions, free idle
    // entries back to the pool (an engaged momentary pins its entry).
    for (int i = 0; i < CS_IR_GOP_POOL; i++) {
        CsIrGop *e = &s_ir_gop[i];
        if (!e->used) continue;
        if (!cs_ir_cmd_live(e->sub)) {
            e->used = false;
            memset(&e->gop, 0, sizeof(e->gop));
            continue;
        }
        CsBinding iv = cs_ir_view(&s_ir.cmds[e->sub]);
        CsBinding v = cs_pv_effective(&iv, s_ir_pv_key[e->sub]);
        if (e->gop.pend_mask) cs_gop_pump(&v, &e->gop);
        if (e->gop.mode != CS_GOP_MOM_ENGAGE && e->gop.mode != CS_GOP_MOM_RESTORE &&
            e->gop.mode != CS_GOP_IDLE &&
            e->gop.age < 0xFFFF && ++e->gop.age >= CS_GROUP_SESSION_TICKS &&
            e->gop.pend_mask == 0)
            e->gop.mode = CS_GOP_IDLE;
        if (e->gop.mode == CS_GOP_IDLE && e->gop.pend_mask == 0)
            e->used = false;
    }

    // Same retry / shadow maintenance for the IR commands' op states.
    for (uint8_t sub = 0; sub < CS_MAX_IR_COMMANDS; sub++) {
        CsOpState *op = &s_ir_op[sub];
        if (!op->pending && !op->shadow_active) continue;
        if (!cs_ir_cmd_live(sub)) { op->pending = false; op->shadow_active = false; continue; }
        CsBinding iv = cs_ir_view(&s_ir.cmds[sub]);
        CsBinding ev = cs_pv_effective(&iv, s_ir_pv_key[sub]);
        const CsBinding *c = &ev;
        if (op->pending &&
            cs_noun_dispatch(c->noun, c->target, c->index, op->value))
            op->pending = false;
        if (op->shadow_active) {
            if (++op->shadow_age >= CS_SHADOW_TIMEOUT_TICKS)
                op->shadow_active = false;
            else if (((s_tick_ct + sub) & 3u) == 0 &&
                     cs_shadow_confirmed(&cs_noun_table[c->noun],
                                         cs_noun_get(c->noun, c->target, c->index),
                                         op))
                op->shadow_active = false;
        }
    }

    if (s_ir_slot != 0xFF)
        cs_tick_ir();

    cs_display_tick();

    // Buttons decode once per pin group, not per binding.
    for (uint8_t i = 0; i < CS_MAX_BINDINGS; i++)
        if (s_btn[i].pin != CS_GPIO_UNUSED) cs_tick_button_group(&s_btn[i]);

    bool pot_done = false;   // one ADC conversion per tick, round-robin
    for (uint8_t i = 0; i < CS_MAX_BINDINGS; i++) {
        uint8_t s = (uint8_t)((s_pot_rr + 1 + i) % CS_MAX_BINDINGS);
        if (!s_rt[s].active) continue;
        switch (s_cfg.bindings[s].type) {
            case CS_TYPE_SWITCH:  cs_tick_switch(s);  break;
            case CS_TYPE_ENCODER: cs_tick_encoder(s); break;
            case CS_TYPE_LED:
                if (((s_tick_ct + s) & (CS_IND_DECIM - 1)) == 0) cs_tick_led(s);
                break;
            case CS_TYPE_LED_PWM:
                if (((s_tick_ct + s) & (CS_IND_DECIM - 1)) == 0) cs_tick_led_pwm(s);
                break;
            case CS_TYPE_POT:
                if (!pot_done) { cs_tick_pot(s); s_pot_rr = s; pot_done = true; }
                break;
            default: break;
        }
    }
}

const CsFlashConfig *control_surfaces_config(void) { return &s_cfg; }

const CsIrConfig *control_surfaces_ir_config(void) { return &s_ir; }

const char (*control_surfaces_names(void))[CS_NAME_LEN] { return s_names; }

const char *control_surfaces_get_name(uint8_t slot) {
    return (slot < CS_MAX_BINDINGS) ? s_names[slot] : NULL;
}

uint8_t control_surfaces_apply_name(uint8_t slot, const char *name) {
    if (slot >= CS_MAX_BINDINGS) return CS_STATUS_INVALID_SLOT;
    memset(s_names[slot], 0, CS_NAME_LEN);
    strncpy(s_names[slot], name, CS_NAME_LEN - 1);
    return PIN_CONFIG_SUCCESS;
}

const CsBinding *control_surfaces_get_binding(uint8_t slot) {
    return (slot < CS_MAX_BINDINGS) ? &s_cfg.bindings[slot] : NULL;
}

const IrCommand *control_surfaces_get_ir_cmd(uint8_t sub) {
    return (sub < CS_MAX_IR_COMMANDS) ? &s_ir.cmds[sub] : NULL;
}

const CsGroupConfig *control_surfaces_group_config(void) { return &s_groups; }

const CsMacroConfig *control_surfaces_macro_config(void) { return &s_macros; }

const CsGroup *control_surfaces_get_group(uint8_t idx) {
    return (idx < CS_MAX_GROUPS) ? &s_groups.groups[idx] : NULL;
}

const CsMacro *control_surfaces_get_macro(uint8_t idx) {
    return (idx < CS_MAX_MACROS) ? &s_macros.macros[idx] : NULL;
}

void control_surfaces_get_ext_status(CsExtStatusPacket *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->max_groups = CS_MAX_GROUPS;
    out->max_macros = CS_MAX_MACROS;
    out->max_macro_steps = CS_MAX_MACRO_STEPS;
    out->macro_running = s_macro_run;
    out->macro_step = (s_macro_run != 0xFF) ? s_macro_step : 0;
    memcpy(out->group_status, s_group_status, sizeof(out->group_status));
    memcpy(out->macro_status, s_macro_status, sizeof(out->macro_status));
}

void control_surfaces_get_status(CsStatusPacket *out) {
    if (!out) return;
    out->last_status = cs_last_status;
    out->last_slot = cs_last_slot;
    out->max_bindings = CS_MAX_BINDINGS;
    out->dirty = s_dirty ? 1 : 0;
    out->active_mask = 0;
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s_rt[s].active) out->active_mask |= (uint16_t)(1u << s);
        out->slot_status[s] = s_slot_status[s];
    }
    out->ir_active_mask = 0;
    out->ir_learn_state = cs_ir_learn_state();
    for (uint8_t sub = 0; sub < CS_MAX_IR_COMMANDS; sub++) {
        if (cs_ir_cmd_live(sub)) out->ir_active_mask |= (uint16_t)(1u << sub);
        out->ir_cmd_status[sub] = s_ir_cmd_status[sub];
    }
}

bool control_surfaces_dirty(void) { return s_dirty; }
void control_surfaces_set_dirty(bool dirty) { s_dirty = dirty; }

uint8_t control_surfaces_ir_learn_control(uint8_t op) {
    if (op == 1) {
        if (s_ir_slot == 0xFF) return CS_STATUS_NO_IR;
        cs_ir_learn_arm();
    } else {
        cs_ir_learn_cancel();
    }
    return PIN_CONFIG_SUCCESS;
}

void control_surfaces_get_ir_learn(uint8_t out[8]) {
    uint8_t proto = 0;
    uint32_t code = 0;
    (void)cs_ir_learn_result(&proto, &code);
    out[0] = cs_ir_learn_state();
    out[1] = proto;
    out[2] = 0;
    out[3] = 0;
    out[4] = (uint8_t)(code & 0xFF);
    out[5] = (uint8_t)((code >> 8) & 0xFF);
    out[6] = (uint8_t)((code >> 16) & 0xFF);
    out[7] = (uint8_t)((code >> 24) & 0xFF);
}

const CsCapsHeader *control_surfaces_caps_header(void) { return &s_caps; }

const CsNounDesc *control_surfaces_noun_desc(uint8_t noun) {
    return (noun < CS_NOUN_COUNT) ? &cs_noun_table[noun] : NULL;
}
