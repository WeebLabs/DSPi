/*
 * control_surfaces.h; user-wired physical controls and indicators on spare GPIOs.
 *
 * A Control Surface binding attaches one physical component (push button,
 * toggle switch, potentiometer, rotary encoder, LED, PWM-dimmed LED) to one
 * firmware parameter (a "noun") through one operation (an "action").
 * Bindings are configured via vendor commands 0x84-0x87, persisted
 * device-global in the preset directory (V9+), and processed by a 1 kHz
 * main-loop tick.
 *
 * Every control action is applied by dispatching the SAME vendor command a
 * host would send, through vendor_dispatch_set/get with CTRL_SOURCE_GPIO.
 * That reuses all existing validation, deferred-apply safety, and host
 * notifications (tagged PARAM_SRC_GPIO); nothing in the apply path is
 * duplicated here.
 *
 * Validity is table-driven, not hand-enumerated: each component type
 * declares the actions it can drive (plus pin count and pin class) and each
 * noun declares the actions it accepts, its value unit, and whether it is
 * addressed by a channel/band target.  A binding is valid iff its action is
 * in both masks, its target/index address an existing channel/band, and its
 * pins pass the shared conflict checks.  Hosts read the same tables via
 * REQ_GET_CS_CAPS, so new types/nouns/actions appear in the UI without
 * host-side hardcoding.
 *
 * Format v2 (config version 2, caps version 2): bindings are 24 bytes and
 * gain explicit event / target / index fields; 16 slots.  Buttons support
 * press / long-press / double-press events (several bindings may share one
 * GPIO with distinct events), hold-to-repeat, and momentary (hold-to-engage)
 * actions; encoders support acceleration; nouns carry a unit so frequency
 * and Q step logarithmically.
 *
 * Caps v3 adds the IR remote component: one CS_TYPE_IR binding holds the
 * receiver GPIO and up to CS_MAX_IR_COMMANDS learned remote buttons live in
 * a separate command table (commands 0x8D-0x8F), each dispatching through
 * the same noun/action machinery as a physical button.  Binding, IR command
 * and slot-name SETs are apply-live-only previews; REQ_CS_SAVE persists the
 * whole live config and REQ_CS_REVERT reloads the stored one.
 *
 * Caps v4 adds the stereo upmixer, psychoacoustic bass, output delay, and
 * preset-reload nouns, plus the CS_UNIT_MS unit (8.8 milliseconds) the
 * delay noun uses.  No structure sizes change; the bump only signals the
 * new unit value to hosts.
 *
 * Caps v5 widened the upmix centre-mode enum to 3 (the third being Off).
 * Nothing else changed; the bump exists for hosts that hard-code the mode
 * labels instead of reading enum_count from the noun descriptor.
 *
 * Caps v6 doubles CS_MAX_IR_COMMANDS to 16.  CsStatusPacket grows to 41 bytes
 * (16-bit ir_active_mask, 16-entry ir_cmd_status) and the persisted CsIrConfig
 * to format v2; hosts must read max_ir_commands rather than assume 8.
 *
 * Caps v7 adds the loudness reference-SPL and intensity nouns.  No structure
 * sizes change; hosts pick them up from noun_count as usual.
 *
 * Caps v8 adds indicator timing (CsBinding on_delay/off_delay, 0.1 s units,
 * carved from reserved2; LED types with IND_EQUALS/IND_ABOVE only) and the
 * INPUT_LEVEL_MAX noun (loudest channel of the active input).  Struct sizes
 * are unchanged; pre-v8 configs carry zeros there, meaning no delay.
 *
 * Caps v9 adds target groups (8 named channel sets a binding or macro step
 * addresses as a unit via CS_FLAG_GROUP) and macros (8 step sequences fired
 * through the CS_NOUN_MACRO noun), commands 0x20-0x26, directory V18.
 * CsBinding/IrCommand/CsStatusPacket are unchanged; the caps header's three
 * reserved bytes become max_groups/max_macros/max_macro_steps.
 * See Documentation/Features/control_surfaces_groups_macros_spec.md.
 *
 * Caps v10 adds the I2C display component (CS_TYPE_DISPLAY container,
 * commands 0x27-0x2B, directory V19), IR command group support
 * (IrCommand.flags accepts CS_FLAG_GROUP), and nouns 53-56 (CPU_LOAD,
 * DISPLAY_PAGE, DISPLAY_EDIT, PAGE_VALUE).  The caps type table grows by
 * one row (the documented self-describing mechanism); every fixed-offset
 * structure is unchanged.
 * See Documentation/Features/control_surfaces_display_spec.md.
 *
 * Caps v11 adds per-line horizontal alignment for the display (two 2-bit
 * fields in CsDisplayCfg.flags) and pins the edit markers to the outer
 * columns of the value line.  No structure sizes change; the bump only
 * tells hosts the flag bits are honoured rather than rejected.
 *
 * Caps v12 adds CsBinding.base_bright, a per-LED_PWM brightness ceiling
 * carved from the spare reserved byte, so a level meter keeps its full
 * sweep while its top end moves.  CsBinding stays 24 bytes and pre-v12
 * configs carry 0 there, meaning full brightness.
 *
 * Caps v13 adds display level bars: CsDisplayPage.flags gains CS_DPAGE_BAR
 * for continuous nouns.  Character panels draw the bar in CGRAM cells (a
 * 2-row panel folds the value into the label row to free one); graphic
 * panels invert the pixels behind the value instead.  No structure sizes
 * change.  See Documentation/Features/control_surfaces_display_spec.md.
 *
 * See Documentation/Features/control_surfaces_spec.md.
 */

#ifndef CONTROL_SURFACES_H
#define CONTROL_SURFACES_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Component types.  Values are wire/flash-persistent; never renumber.
// ---------------------------------------------------------------------------
typedef enum {
    CS_TYPE_NONE    = 0,   // slot disabled
    CS_TYPE_BUTTON  = 1,   // momentary push button (1 GPIO, edge/gesture)
    CS_TYPE_SWITCH  = 2,   // latching toggle switch (1 GPIO, level-follow)
    CS_TYPE_POT     = 3,   // potentiometer / fader (1 ADC-capable GPIO)
    CS_TYPE_ENCODER = 4,   // quadrature rotary encoder (2 GPIOs)
    CS_TYPE_LED     = 5,   // indicator LED (1 GPIO, output)
    CS_TYPE_LED_PWM = 6,   // PWM-dimmed LED (1 GPIO, hardware PWM slice)
    CS_TYPE_IR      = 7,   // IR remote receiver (1 GPIO); a container slot
                           // whose commands live in the IrCommand table
    CS_TYPE_DISPLAY = 8,   // I2C character/OLED display (2 GPIOs: SDA, SCL);
                           // a container slot, content configured via 0x27-0x2A
    CS_TYPE_COUNT
} CsType;

// ---------------------------------------------------------------------------
// Nouns; the firmware parameters a surface can control or indicate.
// Values are wire/flash-persistent; never renumber, append only.
// ---------------------------------------------------------------------------
typedef enum {
    CS_NOUN_USER_VOLUME    = 0,  // continuous dB, shared with the host OS slider
    CS_NOUN_MASTER_VOLUME  = 1,  // continuous dB, device output ceiling
    CS_NOUN_USER_MUTE      = 2,  // bool
    CS_NOUN_LOUDNESS       = 3,  // bool (loudness compensation enable)
    CS_NOUN_CROSSFEED      = 4,  // bool (crossfeed enable)
    CS_NOUN_LEVELLER       = 5,  // bool (volume leveller enable)
    CS_NOUN_PRESET         = 6,  // enum 0..9 (active preset slot)
    CS_NOUN_INPUT_SOURCE   = 7,  // enum 0..2 (USB / SPDIF / I2S)
    CS_NOUN_CLIP           = 8,  // bool latch (any-channel clip); trigger clears
    // --- v2 additions ---
    CS_NOUN_EQ_BYPASS      = 9,  // bool (1 = master EQ bypassed)
    CS_NOUN_LG_SYNC        = 10, // bool (LG Sound Sync enable)
    CS_NOUN_CROSSFEED_PRESET = 11, // enum 0..3 (crossfeed voicing preset)
    CS_NOUN_CROSSFEED_ITD  = 12, // bool (crossfeed ITD enable)
    CS_NOUN_LEVELLER_AMOUNT = 13, // continuous percent 0..100
    CS_NOUN_LEVELLER_SPEED = 14, // enum 0..2 (slow / medium / fast)
    CS_NOUN_LEVELLER_LOOKAHEAD = 15, // bool
    CS_NOUN_PREAMP         = 16, // continuous dB; target = input channel
    CS_NOUN_OUTPUT_GAIN    = 17, // continuous dB; target = output channel
    CS_NOUN_OUTPUT_MUTE    = 18, // bool; target = output channel
    CS_NOUN_OUTPUT_ENABLE  = 19, // bool; target = output channel
    CS_NOUN_FILTER_FREQ    = 20, // continuous Hz; target = channel, index = band
    CS_NOUN_FILTER_GAIN    = 21, // continuous dB; target = channel, index = PEQ band
    CS_NOUN_FILTER_Q       = 22, // continuous Q; target = channel, index = PEQ band
    CS_NOUN_FILTER_TYPE    = 23, // enum (PEQ FilterType); target = channel, index = PEQ band
    CS_NOUN_FILTER_BYPASS  = 24, // bool; target = channel, index = band
    CS_NOUN_SIGGEN         = 25, // bool (test signal generator running)
    CS_NOUN_DAC_MUTE_TEST  = 26, // trigger (pulse the DAC hardware mute ~1 s)
    CS_NOUN_CLIP_CH        = 27, // bool, read-only; target = channel clip latch
    CS_NOUN_LEVEL          = 28, // continuous dB, read-only; target = channel peak meter
    CS_NOUN_SPDIF_LOCK     = 29, // bool, read-only (SPDIF RX locked)
    CS_NOUN_SAMPLE_RATE    = 30, // enum, read-only (0=44.1k 1=48k 2=96k)
    CS_NOUN_USB_STREAMING  = 31, // bool, read-only (USB input actively streaming)
    CS_NOUN_ADAT_ACTIVE    = 32, // bool, read-only (ADAT out streaming, rate ok; RP2350)
    CS_NOUN_LG_PRESENT     = 33, // bool, read-only (LG Sound Sync source detected)
    CS_NOUN_LG_MUTED       = 34, // bool, read-only (LG source present and muted)
    // --- caps v4 additions ---
    CS_NOUN_UPMIX          = 35, // bool (stereo upmixer enable; RP2350)
    CS_NOUN_UPMIX_CENTER_MODE   = 36, // enum 0..1 (Passive / Logic; RP2350)
    CS_NOUN_UPMIX_SURROUND_MODE = 37, // enum 0..2 (Off / Passive / Logic; RP2350)
    CS_NOUN_UPMIX_STRENGTH = 38, // continuous percent 0..100 (RP2350)
    CS_NOUN_UPMIX_WIDTH    = 39, // continuous percent 0..100 (centre width; RP2350)
    CS_NOUN_UPMIX_PRESENCE = 40, // continuous dB -12..+12 (centre presence bell; RP2350)
    CS_NOUN_PSYBASS        = 41, // bool (psychoacoustic bass enable)
    CS_NOUN_PSYBASS_CUTOFF = 42, // continuous Hz 30..300
    CS_NOUN_PSYBASS_HARMONICS = 43, // continuous dB -24..+12 (harmonics mix level)
    CS_NOUN_PSYBASS_DRIVE  = 44, // continuous dB 0..18
    CS_NOUN_PSYBASS_CHARACTER = 45, // continuous percent 0..100 (even<->odd blend)
    CS_NOUN_PSYBASS_ORIGINAL = 46, // continuous dB -60..0 (original low-band level)
    CS_NOUN_OUTPUT_DELAY   = 47, // continuous ms; target = output channel
    CS_NOUN_PRESET_RELOAD  = 48, // trigger (reload the active preset from flash,
                                 // discarding unsaved live edits)
    // --- caps v7 additions ---
    CS_NOUN_LOUDNESS_SPL   = 49, // continuous dB SPL 40..100 (reference listening level)
    CS_NOUN_LOUDNESS_INTENSITY = 50, // continuous percent 0..127 (compensation depth)
    // --- caps v8 additions ---
    CS_NOUN_INPUT_LEVEL_MAX = 51, // continuous dB, read-only; loudest channel of
                                  // the active input (signal-presence sensing)
    // --- caps v9 additions ---
    CS_NOUN_MACRO          = 52, // enum 0..CS_MAX_MACROS-1; SET fires macro
                                 // `value`, IND_EQUALS lights while it runs;
                                 // live read = running index, 255 idle
    // --- caps v10 additions ---
    CS_NOUN_CPU_LOAD       = 53, // continuous percent 0..100, read-only (core 0)
    CS_NOUN_DISPLAY_PAGE   = 54, // enum 0..15: shown display page; steps skip
                                 // empty page slots (like PRESET skips empties)
    CS_NOUN_DISPLAY_EDIT   = 55, // bool: edit mode armed (auto-clears after
                                 // the configured edit_timeout)
    CS_NOUN_PAGE_VALUE     = 56, // virtual: STEP/INC/DEC/TOGGLE the shown
                                 // page's item, resolved at event time
    CS_NOUN_COUNT
} CsNoun;

// Value kinds (CsNounDesc.kind)
#define CS_KIND_CONTINUOUS  0
#define CS_KIND_BOOL        1
#define CS_KIND_ENUM        2

// Value units (CsNounDesc.unit).  Encoding of value/range fields and the
// stepping law follow the unit; see the spec, section 2.1.
#define CS_UNIT_NONE     0   // bool/enum: plain integers
#define CS_UNIT_DB       1   // 8.8 signed fixed point dB; linear stepping
#define CS_UNIT_HZ       2   // plain integer Hz; log stepping (step = 8.8 octaves)
#define CS_UNIT_Q        3   // 8.8 fixed point Q; log stepping (step = 8.8 octaves)
#define CS_UNIT_PERCENT  4   // 8.8 fixed point percent; linear stepping
#define CS_UNIT_MS       5   // 8.8 fixed point milliseconds; linear stepping,
                             // default step 0.1 ms (caps v4+)

// Target kinds (CsNounDesc.target_kind); what CsBinding.target addresses.
#define CS_TARGET_NONE      0   // target/index ignored
#define CS_TARGET_INPUT_CH  1   // target = input channel (0..target_count-1)
#define CS_TARGET_OUTPUT_CH 2   // target = output channel (0..target_count-1)
#define CS_TARGET_DSP_CH    3   // target = DSP channel (inputs then outputs)
#define CS_TARGET_DSP_BAND  4   // target = DSP channel, index = filter band

// Noun descriptor flags (CsNounDesc.dflags)
#define CS_NDF_DEFERRED  0x01   // apply is deferred; engine steps from a target shadow

// ---------------------------------------------------------------------------
// Actions (the "Parameter" of a binding).  Wire/flash-persistent values.
// Control actions (input components) and indicate actions (output
// components) share one namespace; the type/noun masks keep them apart.
// ---------------------------------------------------------------------------
typedef enum {
    CS_ACT_ADJUST     = 0,  // pot: absolute position maps onto a value range
    CS_ACT_STEP       = 1,  // encoder: +/- `step` per detent (enum: next/prev)
    CS_ACT_INC        = 2,  // button: + `step` per press (enum: next; +WRAP = cycle)
    CS_ACT_DEC        = 3,  // button: - `step` per press (enum: previous)
    CS_ACT_TOGGLE     = 4,  // button: invert a bool per press
    CS_ACT_SET        = 5,  // button: set the noun to `value` per press
    CS_ACT_FOLLOW     = 6,  // switch: bool tracks the switch position
    CS_ACT_TRIGGER    = 7,  // button: fire the noun's command (e.g. clip clear)
    CS_ACT_IND_EQUALS = 8,  // LED: lit while noun value == `value`
    CS_ACT_MOMENTARY  = 9,  // button: hold = set to `value`, release = restore
    CS_ACT_IND_ABOVE  = 10, // LED: lit while noun value >= `value`
    CS_ACT_IND_LEVEL  = 11, // PWM LED: brightness follows the noun value
    CS_ACT_COUNT
} CsAction;

#define CS_ACT_BIT(a)  (1u << (a))

// Button events (CsBinding.event; CS_TYPE_BUTTON only, 0 for other types).
// Bindings of button type may share one GPIO when their events differ.
typedef enum {
    CS_EVT_PRESS  = 0,   // short press (default)
    CS_EVT_LONG   = 1,   // held >= 500 ms
    CS_EVT_DOUBLE = 2,   // two presses within 350 ms
    CS_EVT_COUNT
} CsEvent;

// Binding flags
#define CS_FLAG_INVERT   0x01  // input: active-high w/ pull-down (default is
                               // active-low w/ pull-up); LED: drive low = lit
#define CS_FLAG_REVERSE  0x02  // pot/encoder: invert direction
#define CS_FLAG_WRAP     0x04  // enum STEP/INC/DEC wraps around the ends
#define CS_FLAG_ACCEL    0x08  // encoder: fast rotation multiplies the step
#define CS_FLAG_REPEAT   0x10  // button INC/DEC: auto-repeat while held
// Group flags (caps v9).  GROUP re-reads `target` as a group index; the two
// modifiers require it (LINK_ABS: ADJUST drives members identical instead of
// offset-preserving; GROUP_ALL: indicator needs every member, not any).
#define CS_FLAG_GROUP    0x20
#define CS_FLAG_LINK_ABS 0x40
#define CS_FLAG_GROUP_ALL 0x80

// Pin classes (CsTypeDesc.pin_class)
#define CS_PINCLASS_ANY  0
#define CS_PINCLASS_ADC  1     // GPIO 26..28 (ADC0..2 on both platforms)

#define CS_MAX_BINDINGS  16
#define CS_GPIO_UNUSED   0xFF

// Target groups and macros (caps v9).  Sizes are frozen into the flash
// blobs below; growing them later is a config-format bump.
#define CS_MAX_GROUPS       8
#define CS_MAX_MACROS       8
#define CS_MAX_MACRO_STEPS  8

// I2C display (caps v10, line alignment v11); see
// control_surfaces_display_spec.md.
#define CS_MAX_DISPLAY_PAGES  16

// Display models (CsBinding.index on a CS_TYPE_DISPLAY slot).  Wire/flash
// persistent; never renumber.  Bus speed and geometry are fixed per model.
#define CS_DISP_MODEL_NONE           0
#define CS_DISP_MODEL_LCD1602        1   // HD44780 16x2 via PCF8574, 100 kHz
#define CS_DISP_MODEL_LCD2004        2   // HD44780 20x4 via PCF8574, 100 kHz
#define CS_DISP_MODEL_CHAR_OLED_16X2 3   // US2066/RW1063, 400 kHz
#define CS_DISP_MODEL_CHAR_OLED_20X2 4
#define CS_DISP_MODEL_CHAR_OLED_20X4 5
#define CS_DISP_MODEL_SSD1306_128X64 6   // graphic OLED, flash 5x8 font
#define CS_DISP_MODEL_SSD1306_128X32 7
#define CS_DISP_MODEL_SH1106_128X64  8
#define CS_DISP_MODEL_COUNT          9

// Display home-content modes (CsDisplayCfg.mode)
#define CS_DMODE_FIXED          0   // show cfg.home_page
#define CS_DMODE_CYCLE_SELECTED 1   // rotate the active pages at cfg.dwell
#define CS_DMODE_CYCLE_ALL      2   // rotate every displayable noun at cfg.dwell

// CsDisplayCfg.flags
#define CS_DCFG_OVERLAY_ANY  0x01   // overlay also pops unconfigured dispatches
#define CS_DCFG_EDIT_GATED   0x02   // PAGE_VALUE adjusts only while edit armed
                                    // (steps navigate pages while unarmed)
#define CS_DCFG_LABEL_ALIGN  0x0C   // CS_DALIGN_* for the label line
#define CS_DCFG_VALUE_ALIGN  0x30   // CS_DALIGN_* for the value line
#define CS_DCFG_LABEL_ALIGN_SHIFT  2
#define CS_DCFG_VALUE_ALIGN_SHIFT  4

// Horizontal alignment, both CS_DCFG_*_ALIGN fields.  Encoding 3 is
// reserved; the renderer treats it as LEFT.
#define CS_DALIGN_LEFT    0
#define CS_DALIGN_CENTRE  1
#define CS_DALIGN_RIGHT   2

// CsDisplayPage.flags
#define CS_DPAGE_ACTIVE  0x01       // slot in use (all-zero record = empty)
#define CS_DPAGE_GROUP   0x02       // target is a group index
#define CS_DPAGE_LARGE   0x04       // big font on graphic OLEDs (2x scaled)
#define CS_DPAGE_BAR     0x08       // level bar for the value (continuous nouns
                                    // only); character rows draw it in CGRAM
                                    // cells, graphic panels invert behind text

// IR remote control.  One CS_TYPE_IR binding (the receiver) may be live at a
// time; its remote-button commands live in a separate table of sub-slots so
// the whole handset costs one binding slot and one GPIO.
#define CS_MAX_IR_COMMANDS  16

// IR code protocols (IrCommand.protocol).  Wire/flash-persistent values.
// NONE marks an empty sub-slot.  NEC and RC5/RC6 are decoded properly (NEC
// repeat frames drive hold-to-repeat; the RC5/RC6 toggle bit is masked out of
// the code so a learned button matches every press, and its value separates a
// hold from a re-press); everything else falls back to a timing-signature
// hash, matched by exact re-transmission.
#define CS_IR_PROTO_NONE   0
#define CS_IR_PROTO_NEC    1
#define CS_IR_PROTO_RC5    2
#define CS_IR_PROTO_RC6    3
#define CS_IR_PROTO_HASH   4
#define CS_IR_PROTO_COUNT  5

// Learn state (CsStatusPacket.ir_learn_state / REQ_CS_IR_LEARN result read)
#define CS_IR_LEARN_IDLE     0
#define CS_IR_LEARN_ARMED    1   // listening; next decoded press is captured
#define CS_IR_LEARN_DONE     2   // result available (protocol + code)
#define CS_IR_LEARN_TIMEOUT  3   // nothing received within the learn window

// Per-slot user label ("Sub Level", "Mute All", ...), NUL-terminated, set by
// the host app and persisted device-global in the preset directory (V10+)
// so external MCUs and apps on other hosts can read what each control is
// for.  Same 32-byte convention as preset and channel names.  Names are
// slot metadata, independent of the binding: they survive binding changes
// and slot clears, and may be set before a binding exists.  Like bindings
// and IR commands, a name SET is an apply-live-only preview; REQ_CS_SAVE
// persists and REQ_CS_REVERT restores the stored names.
#define CS_NAME_LEN      32

// ---------------------------------------------------------------------------
// Wire / flash structures
// ---------------------------------------------------------------------------

// One binding; 24 bytes, identical on the wire (REQ_SET/GET_CS_BINDING
// payload) and in flash.  value/step/range encoding follows the noun's unit
// (CS_UNIT_*); bool/enum values are plain integers.
// A CS_TYPE_IR binding is a container: gpio[0] is the receiver pin, INVERT
// selects an idle-low receiver (default is idle-high, e.g. TSOP38xx), and
// every other field must be 0.  Its commands live in the IrCommand table.
typedef struct __attribute__((packed)) {
    uint8_t type;          // CsType
    uint8_t noun;          // CsNoun
    uint8_t action;        // CsAction
    uint8_t flags;         // CS_FLAG_*
    uint8_t gpio[2];       // gpio[1] = CS_GPIO_UNUSED unless type needs two
    uint8_t event;         // CsEvent (buttons; 0 otherwise)
    uint8_t target;        // channel index for targeted nouns (else 0)
    uint8_t index;         // filter band for CS_TARGET_DSP_BAND nouns (else 0)
    // Per-LED brightness ceiling (caps v12): scales the final PWM duty, so a
    // meter keeps its full sweep and only its top end moves.  Percent 1-100,
    // 0 = unset = full; LED_PWM only, every other type writes 0.
    uint8_t base_bright;
    int16_t value;         // SET/MOMENTARY target, IND_EQUALS/IND_ABOVE comparand
    int16_t step;          // STEP/INC/DEC size; 0 = per-unit default
    int16_t range_min;     // pot/IND_LEVEL span; both 0 = the noun's full range
    int16_t range_max;
    // Indicator condition timing (caps v8): the raw IND_EQUALS/IND_ABOVE
    // condition must hold continuously this long before the LED follows it
    // (PLC TON/TOF).  0.1 s units, 0 = immediate; LED types only.
    uint16_t on_delay;
    uint16_t off_delay;
    uint8_t reserved2[2];  // write 0; earmarked for an LED-extras flags byte
                           // (a future global Panel Brightness opt-in), since
                           // CsBinding.flags has no free bit left
} CsBinding;

// One IR remote command; 16 bytes, identical on the wire (REQ_SET/GET_CS_IR_CMD
// payload) and in flash.  Semantically a button-shaped binding: the same noun /
// action / target / value / step rules apply (actions INC/DEC/TOGGLE/SET/
// TRIGGER/MOMENTARY; flags WRAP and REPEAT), fired by the learned code instead
// of a GPIO edge.  protocol == CS_IR_PROTO_NONE marks the sub-slot empty (all
// other fields must then be 0).
typedef struct __attribute__((packed)) {
    uint8_t  noun;         // CsNoun
    uint8_t  action;       // CsAction (button subset)
    uint8_t  flags;        // CS_FLAG_WRAP | CS_FLAG_REPEAT
    uint8_t  target;       // channel index for targeted nouns (else 0)
    uint8_t  index;        // filter band for CS_TARGET_DSP_BAND nouns (else 0)
    uint8_t  protocol;     // CS_IR_PROTO_*
    int16_t  value;        // SET/MOMENTARY target
    int16_t  step;         // INC/DEC size; 0 = per-unit default
    uint8_t  reserved[2];  // write 0
    uint32_t code;         // learned code (encoding per protocol; see spec)
} IrCommand;

// Directory-persisted blob (device-global, V9+).  All-zero = every slot
// CS_TYPE_NONE = feature idle; a fresh directory needs no special seeding.
#define CS_CONFIG_VERSION  2
typedef struct __attribute__((packed)) {
    uint8_t   version;     // CS_CONFIG_VERSION
    uint8_t   reserved[3];
    CsBinding bindings[CS_MAX_BINDINGS];
} CsFlashConfig;           // 388 bytes

// IR command table, directory-persisted (device-global, V11+) beside
// cs_config.  All-zero = every sub-slot empty; a fresh directory needs no
// seeding (protocol 0 = CS_IR_PROTO_NONE).  Format v2 grew the sub-slot count
// from 8 to 16; the frozen v1 geometry lives in flash_storage.c.
#define CS_IR_CONFIG_VERSION  2
typedef struct __attribute__((packed)) {
    uint8_t   version;     // CS_IR_CONFIG_VERSION
    uint8_t   reserved[3];
    IrCommand cmds[CS_MAX_IR_COMMANDS];
} CsIrConfig;              // 260 bytes

// One target group; 40 bytes, identical on the wire (REQ_SET/GET_CS_GROUP
// payload) and in flash.  target_kind 0 marks the slot empty (the record
// must then be all-zero).  member_mask bit N = channel N of the kind's
// space; 32-bit because the RP2350 DSP-channel space exceeds 16.
typedef struct __attribute__((packed)) {
    uint8_t  target_kind;  // CS_TARGET_INPUT_CH / _OUTPUT_CH / _DSP_CH; 0 = empty
    uint8_t  reserved[3];  // write 0
    uint32_t member_mask;
    char     name[CS_NAME_LEN];
} CsGroup;                 // 40 bytes

// Group table, directory-persisted (device-global, V18+).  All-zero = no
// groups; a fresh directory needs no seeding.
#define CS_GROUP_CONFIG_VERSION  1
typedef struct __attribute__((packed)) {
    uint8_t  version;      // CS_GROUP_CONFIG_VERSION
    uint8_t  reserved[3];
    CsGroup  groups[CS_MAX_GROUPS];
} CsGroupConfig;           // 324 bytes

// One macro step; 12 bytes, identical on the wire (REQ_SET_CS_MACRO_STEP
// payload) and in flash.  A stripped button-shaped binding: actions SET /
// TOGGLE / INC / DEC / TRIGGER, flags WRAP | GROUP, fired by the sequencer
// after pre_delay (10 ms units).  All-zero = empty step (skipped).
typedef struct __attribute__((packed)) {
    uint8_t  noun;         // CsNoun (not CS_NOUN_MACRO; no nesting)
    uint8_t  action;       // CsAction (step subset)
    uint8_t  flags;        // CS_FLAG_WRAP | CS_FLAG_GROUP
    uint8_t  target;       // channel index, or group index with CS_FLAG_GROUP
    uint8_t  index;        // filter band for CS_TARGET_DSP_BAND nouns (else 0)
    uint8_t  reserved;     // write 0
    int16_t  value;        // as CsBinding.value
    int16_t  step;         // INC/DEC size; 0 = per-unit default
    uint16_t pre_delay;    // delay before this step runs, 10 ms units
} CsMacroStep;             // 12 bytes

typedef struct __attribute__((packed)) {
    char        name[CS_NAME_LEN];
    uint8_t     step_count;    // steps executed = steps[0..step_count-1]; 0 = empty
    uint8_t     reserved[3];   // write 0
    CsMacroStep steps[CS_MAX_MACRO_STEPS];
} CsMacro;                 // 132 bytes

// Macro table, directory-persisted (device-global, V18+).  All-zero = no
// macros.
#define CS_MACRO_CONFIG_VERSION  1
typedef struct __attribute__((packed)) {
    uint8_t  version;      // CS_MACRO_CONFIG_VERSION
    uint8_t  reserved[3];
    CsMacro  macros[CS_MAX_MACROS];
} CsMacroConfig;           // 1060 bytes

// REQ_SET_CS_MACRO payload: name + step count only.  Steps are SET one at a
// time (the whole 132-byte CsMacro exceeds the 64-byte vendor SET buffer);
// hosts should write steps first and the header last so a concurrent fire
// never sees step_count exceed the written steps.
typedef struct __attribute__((packed)) {
    char     name[CS_NAME_LEN];
    uint8_t  step_count;
    uint8_t  reserved[3];  // write 0
} CsMacroHeaderWire;       // 36 bytes

// Display configuration; 12 bytes, identical on the wire
// (REQ_SET/GET_CS_DISPLAY_CFG payload) and inside CsDisplayFlash.
typedef struct __attribute__((packed)) {
    uint8_t  mode;          // CS_DMODE_*
    uint8_t  home_page;     // page slot shown in FIXED mode
    uint16_t dwell;         // cycle period, 0.1 s units (min 10 in cycle modes)
    uint16_t overlay_hold;  // event pop-up hold, 0.1 s units; 0 = overlay off
    uint8_t  brightness;    // 0-255; OLED contrast / backlight where supported
    uint8_t  flags;         // CS_DCFG_*
    uint16_t edit_timeout;  // edit-mode auto-disarm, 0.1 s units; 0 = manual only
    uint8_t  reserved[2];   // write 0
} CsDisplayCfg;             // 12 bytes

// One display page; 4 bytes, wire (REQ_SET/GET_CS_DISPLAY_PAGE) and flash.
// All-zero = empty slot.  Rendering is generic from the noun's kind/unit.
typedef struct __attribute__((packed)) {
    uint8_t noun;           // CsNoun to show
    uint8_t target;         // channel, or group index with CS_DPAGE_GROUP
    uint8_t index;          // filter band for CS_TARGET_DSP_BAND nouns (else 0)
    uint8_t flags;          // CS_DPAGE_*
} CsDisplayPage;            // 4 bytes

// Display blob, directory-persisted (device-global, V19+).  All-zero =
// feature idle; pages seed on first display apply, not at migration.
#define CS_DISPLAY_CONFIG_VERSION  1
typedef struct __attribute__((packed)) {
    uint8_t       version;  // CS_DISPLAY_CONFIG_VERSION
    uint8_t       reserved[3];
    CsDisplayCfg  cfg;
    CsDisplayPage pages[CS_MAX_DISPLAY_PAGES];
} CsDisplayFlash;           // 80 bytes

// REQ_GET_CS_DISPLAY_STATUS response.
typedef struct __attribute__((packed)) {
    uint8_t  init_state;    // 0 down, 1 initializing, 2 live, 3 error/backoff
    uint8_t  current_page;  // shown page slot (0xFF = none/synthesized overlay)
    uint8_t  flags;         // bit0 overlay showing, bit1 edit armed
    uint8_t  model;         // live CS_DISP_MODEL_*
    uint16_t nak_count;     // cumulative I2C aborts (saturating)
    uint8_t  reserved[2];
} CsDisplayStatus;          // 8 bytes

// REQ_GET_CS_EXT_STATUS response.  Group/macro validity mirrors slot_status
// semantics; macro_running is 0xFF when the sequencer is idle.
typedef struct __attribute__((packed)) {
    uint8_t max_groups;        // CS_MAX_GROUPS
    uint8_t max_macros;        // CS_MAX_MACROS
    uint8_t max_macro_steps;   // CS_MAX_MACRO_STEPS
    uint8_t macro_running;     // running macro index; 0xFF = idle
    uint8_t macro_step;        // current step index while running (else 0)
    uint8_t reserved[3];
    uint8_t group_status[CS_MAX_GROUPS];   // stored-record validity
    uint8_t macro_status[CS_MAX_MACROS];   // worst step validity
} CsExtStatusPacket;       // 24 bytes

// Capability descriptors (REQ_GET_CS_CAPS).  wValue = 0xFFFF returns the
// header + type table; wValue = noun index returns that noun's descriptor.
typedef struct __attribute__((packed)) {
    uint16_t actions;      // CS_ACT_BIT mask this component can drive
    uint8_t  pin_count;    // GPIOs consumed (1 or 2)
    uint8_t  pin_class;    // CS_PINCLASS_*
} CsTypeDesc;

typedef struct __attribute__((packed)) {
    uint8_t  caps_version; // capability format version (13); see the file
                           // header for what each version added
    uint8_t  max_bindings; // CS_MAX_BINDINGS
    uint8_t  type_count;   // CS_TYPE_COUNT (table follows, index = CsType)
    uint8_t  noun_count;   // CS_NOUN_COUNT
    CsTypeDesc types[CS_TYPE_COUNT];
    // v3 additions (hosts locate these at offset 4 + 4*type_count).  The
    // CS_TYPE_IR type descriptor's action mask describes what its COMMANDS
    // may do; the container binding itself carries noun/action 0.
    uint8_t  max_ir_commands;  // CS_MAX_IR_COMMANDS
    // v9: carved from the former reserved[3]; pre-v9 hosts read zeros.
    uint8_t  max_groups;       // CS_MAX_GROUPS
    uint8_t  max_macros;       // CS_MAX_MACROS
    uint8_t  max_macro_steps;  // CS_MAX_MACRO_STEPS
} CsCapsHeader;            // 4 + 4*CS_TYPE_COUNT + 4 = 44 bytes at v10

typedef struct __attribute__((packed)) {
    uint8_t  kind;         // CS_KIND_*
    uint8_t  enum_count;   // CS_KIND_ENUM only
    uint16_t actions;      // CS_ACT_BIT mask this noun accepts (0 = unavailable
                           // on this platform, e.g. ADAT_ACTIVE on RP2040)
    int16_t  min_q;        // CS_KIND_CONTINUOUS range, unit-encoded
    int16_t  max_q;
    uint8_t  unit;         // CS_UNIT_*
    uint8_t  target_kind;  // CS_TARGET_*
    uint8_t  target_count; // valid targets 0..target_count-1 (0 if untargeted)
    uint8_t  dflags;       // CS_NDF_*
} CsNounDesc;              // 12 bytes

// REQ_GET_CS_STATUS response.  last_status / last_slot report the most
// recent deferred SET of any CS kind (binding, name, IR command, save,
// revert); IR command slots are reported as 0x80 | sub-slot.
typedef struct __attribute__((packed)) {
    uint8_t  last_status;  // result of the most recent deferred CS SET
    uint8_t  last_slot;    // slot that SET targeted (0x80 | n = IR sub-slot)
    uint8_t  max_bindings; // CS_MAX_BINDINGS
    uint8_t  dirty;        // 1 = live config differs from flash (unsaved preview)
    uint16_t active_mask;  // bit N = binding N live
    uint8_t  slot_status[CS_MAX_BINDINGS];  // per-slot apply status
    // v3 additions (widened to 16 IR sub-slots at caps v6)
    uint16_t ir_active_mask;   // bit N = IR command N live (component up)
    uint8_t  ir_learn_state;   // CS_IR_LEARN_*
    uint8_t  ir_cmd_status[CS_MAX_IR_COMMANDS];  // per-sub-slot apply status
} CsStatusPacket;          // 41 bytes

// Status codes.  0x00..0x05 reuse the shared PIN_CONFIG_* namespace
// (config.h); Control Surfaces extends it from 0x10.
#define CS_STATUS_INVALID_SLOT    0x10
#define CS_STATUS_INVALID_TYPE    0x11
#define CS_STATUS_INVALID_NOUN    0x12
#define CS_STATUS_INVALID_ACTION  0x13  // action not allowed for type+noun
#define CS_STATUS_INVALID_VALUE   0x14  // value/step/range out of bounds
#define CS_STATUS_PIN_NOT_ADC     0x15  // pot on a non-ADC GPIO
#define CS_STATUS_PENDING         0x16  // SET accepted, apply not yet run
#define CS_STATUS_INVALID_TARGET  0x17  // target/index out of range for the noun
#define CS_STATUS_INVALID_EVENT   0x18  // bad event, or event on a non-button
#define CS_STATUS_PWM_CONFLICT    0x19  // PWM LED collides with another on the
                                        // same PWM slice+channel
#define CS_STATUS_EVENT_IN_USE    0x1A  // another button binding already has
                                        // this GPIO+event pair
#define CS_STATUS_BUSY            0x1B  // a previous binding SET is still
                                        // queued for apply; retry shortly
#define CS_STATUS_FLASH_ERROR     0x1C  // directory persist failed (REQ_CS_SAVE)
#define CS_STATUS_IR_IN_USE       0x1D  // another slot already holds the IR
                                        // component (one receiver per device)
#define CS_STATUS_NO_IR           0x1E  // IR command/learn needs a live
                                        // CS_TYPE_IR binding first
#define CS_STATUS_INVALID_GROUP   0x1F  // group reference empty, out of range,
                                        // or kind-incompatible with the noun
#define CS_STATUS_INVALID_MACRO   0x20  // bad macro index or step_count
#define CS_STATUS_INVALID_STEP    0x21  // macro step record invalid
#define CS_STATUS_DISPLAY_IN_USE  0x22  // another slot already holds the display
#define CS_STATUS_PIN_NOT_I2C     0x23  // SDA/SCL not a valid same-instance pair
#define CS_STATUS_I2C_IN_USE      0x24  // instance occupied by the I2C control
                                        // interface (target mode)
#define CS_STATUS_INVALID_PAGE    0x25  // display cfg/page record invalid

// ---------------------------------------------------------------------------
// Public API (all main-loop context)
// ---------------------------------------------------------------------------

// Boot init: load the persisted config and bring up every valid binding.
// Call at the END of core0_init, after preset_boot_load, all audio pin
// claims, notify_init, and the UART/I2C control interfaces, so pin-conflict
// results are truthful and dispatches see initialised state.  A stored
// binding whose pins now collide is kept down (visible in slot_status).
void control_surfaces_init(void);

// 1 kHz poll: debounce buttons/switches, decode gestures, decode encoders,
// read pots, drive LEDs, and dispatch resulting parameter changes.
// Self-throttled with time_us_64; cheap no-op when no binding is active.
// Call once per main loop iteration.
void control_surfaces_tick(void);

// Validate and apply one binding (type CS_TYPE_NONE clears the slot).
// Releases the slot's old pins, claims the new ones, and activates the
// runtime state.  Returns PIN_CONFIG_* / CS_STATUS_*; on failure the old
// binding is restored.  Caller persists on success.
uint8_t control_surfaces_apply_binding(uint8_t slot, const CsBinding *b);

// True if `pin` is claimed by any live binding.  Wired into
// pin_used_by_fixed_peripheral so no other subsystem can take a CS pin.
bool control_surfaces_owns_pin(uint8_t pin);

// Live config (the persistence source for REQ_CS_SAVE) and read-only
// accessors for the vendor GET handlers.
const CsFlashConfig *control_surfaces_config(void);
const CsIrConfig *control_surfaces_ir_config(void);
const char (*control_surfaces_names(void))[CS_NAME_LEN];
const CsBinding *control_surfaces_get_binding(uint8_t slot);  // NULL if bad slot
const IrCommand *control_surfaces_get_ir_cmd(uint8_t sub);    // NULL if bad sub-slot
const char *control_surfaces_get_name(uint8_t slot);          // NULL if bad slot

// Update one live slot name (copied, NUL termination guaranteed).  Live-only,
// like apply_binding; the caller marks the config dirty.  Returns
// PIN_CONFIG_SUCCESS or CS_STATUS_INVALID_SLOT.
uint8_t control_surfaces_apply_name(uint8_t slot, const char *name);
void control_surfaces_get_status(CsStatusPacket *out);
const CsCapsHeader *control_surfaces_caps_header(void);
const CsNounDesc *control_surfaces_noun_desc(uint8_t noun);   // NULL if bad noun

// Validate and apply one IR command (protocol CS_IR_PROTO_NONE clears the
// sub-slot).  Live-only, like apply_binding; the caller marks the config
// dirty.  Commands may be set before the IR component exists; they activate
// when it comes up.  Returns PIN_CONFIG_* / CS_STATUS_*.
uint8_t control_surfaces_apply_ir_cmd(uint8_t sub, const IrCommand *c);

// Validate and apply one group (all-zero record clears the slot).  Live-only
// preview like apply_binding; the caller marks the config dirty.  Re-validates
// every active grouped binding: dependents that no longer validate go down
// with the failure in slot_status (no in-use refusal), and reactivate when a
// later group SET makes them valid again.  Returns PIN_CONFIG_* / CS_STATUS_*.
uint8_t control_surfaces_apply_group(uint8_t idx, const CsGroup *g);

// Validate and apply a macro header (name + step_count) or one step (all-zero
// clears it).  Live-only previews; the caller marks the config dirty.  Steps
// are also re-validated at fire time, so a torn edit skips, never faults.
uint8_t control_surfaces_apply_macro_header(uint8_t idx, const CsMacroHeaderWire *h);
uint8_t control_surfaces_apply_macro_step(uint8_t idx, uint8_t step,
                                          const CsMacroStep *s);

// Fire macro `idx` (cancelling any running macro at its step boundary) or
// cancel the running one.  Main-loop only.  Returns PIN_CONFIG_SUCCESS or
// CS_STATUS_INVALID_MACRO.  Firing an empty macro succeeds as a no-op.
uint8_t control_surfaces_macro_fire(uint8_t idx);
void    control_surfaces_macro_cancel(void);

// Validate and apply the display config / one page (all-zero page clears the
// slot).  Live-only previews under the shared dirty flag; REQ_CS_SAVE
// persists, REQ_CS_REVERT restores.  Returns PIN_CONFIG_* / CS_STATUS_*.
uint8_t control_surfaces_apply_display_cfg(const CsDisplayCfg *c);
uint8_t control_surfaces_apply_display_page(uint8_t idx, const CsDisplayPage *p);

// Live display config (persistence source for REQ_CS_SAVE) and GET accessors.
const CsDisplayFlash *control_surfaces_display_flash(void);
const CsDisplayPage *control_surfaces_get_display_page(uint8_t idx);  // NULL if bad
void control_surfaces_get_display_status(CsDisplayStatus *out);

// Live tables (persistence sources for REQ_CS_SAVE) and read-only accessors
// for the vendor GET handlers.
const CsGroupConfig *control_surfaces_group_config(void);
const CsMacroConfig *control_surfaces_macro_config(void);
const CsGroup *control_surfaces_get_group(uint8_t idx);   // NULL if bad index
const CsMacro *control_surfaces_get_macro(uint8_t idx);   // NULL if bad index
void control_surfaces_get_ext_status(CsExtStatusPacket *out);

// Re-apply the persisted config (bindings + IR commands + slot names) from
// the directory cache, discarding the live preview.  Per-slot failures land
// in slot_status exactly as at boot.  Main-loop only (releases and reclaims
// GPIOs).
void control_surfaces_revert(void);

// Preview dirty flag: set when a live apply diverges from flash, cleared by
// save / revert (main.c owns the transitions around the flash write).
bool control_surfaces_dirty(void);
void control_surfaces_set_dirty(bool dirty);

// Learn result snapshot for the REQ_CS_IR_LEARN result read:
// {state, protocol, 0, 0, code_le32}, 8 bytes.
void control_surfaces_get_ir_learn(uint8_t out[8]);

// Deferred SET (written by the vendor handler, consumed by the main loop;
// same shape as ctrl_set_uart_pending).  cs_last_status / cs_last_slot feed
// REQ_GET_CS_STATUS and report both binding and name SETs.
extern volatile bool    cs_set_binding_pending;
extern uint8_t          cs_set_binding_slot;
extern CsBinding        cs_set_binding_val;
extern volatile uint8_t cs_last_status;
extern volatile uint8_t cs_last_slot;

// Deferred slot-name SET (REQ_SET_CS_NAME); live-only preview like the
// binding SET, consumed in the main loop for the shared status channel.
extern volatile bool    cs_set_name_pending;
extern uint8_t          cs_set_name_slot;
extern char             cs_set_name_val[CS_NAME_LEN];

// Deferred IR-command SET (REQ_SET_CS_IR_CMD); same single-deep handoff
// shape as the binding SET.  Results land in cs_last_status with
// cs_last_slot = 0x80 | sub-slot, plus the per-sub-slot status array.
extern volatile bool    cs_set_ir_cmd_pending;
extern uint8_t          cs_set_ir_cmd_slot;
extern IrCommand        cs_set_ir_cmd_val;

// Deferred group / macro SETs (REQ_SET_CS_GROUP / _CS_MACRO /
// _CS_MACRO_STEP); same single-deep handoff shape as the binding SET.
// Results land in cs_last_status with cs_last_slot = 0x40 | group or
// 0x60 | macro.
extern volatile bool    cs_set_group_pending;
extern uint8_t          cs_set_group_slot;
extern CsGroup          cs_set_group_val;
extern volatile bool    cs_set_macro_hdr_pending;
extern uint8_t          cs_set_macro_hdr_slot;
extern CsMacroHeaderWire cs_set_macro_hdr_val;
extern volatile bool    cs_set_macro_step_pending;
extern uint8_t          cs_set_macro_step_slot;   // macro index
extern uint8_t          cs_set_macro_step_idx;    // step index
extern CsMacroStep      cs_set_macro_step_val;

// Deferred display SETs (REQ_SET_CS_DISPLAY_CFG / _PAGE); same single-deep
// handoff shape.  Results land in cs_last_status with cs_last_slot =
// 0x50 | page (0x50 alone for the cfg).
extern volatile bool    cs_set_disp_cfg_pending;
extern CsDisplayCfg     cs_set_disp_cfg_val;
extern volatile bool    cs_set_disp_page_pending;
extern uint8_t          cs_set_disp_page_slot;
extern CsDisplayPage    cs_set_disp_page_val;

// Deferred save / revert (REQ_CS_SAVE / REQ_CS_REVERT).  Save persists the
// whole live CS config (bindings + IR commands + slot names + groups +
// macros + display) in one directory write; revert re-applies the stored
// config.  Results land in cs_last_status (cs_last_slot = 0xFF).
extern volatile bool    cs_save_pending;
extern volatile bool    cs_revert_pending;

// Learn control (REQ_CS_IR_LEARN): op 1 = arm, anything else = cancel.
// Direct call; every dispatch transport runs on the core0 main loop.
// Returns PIN_CONFIG_SUCCESS or CS_STATUS_NO_IR (arm without a live IR
// component).  Completion is pushed on the notify EP and readable via
// control_surfaces_get_ir_learn.
uint8_t control_surfaces_ir_learn_control(uint8_t op);

// ---------------------------------------------------------------------------
// Internal interface between the engine (control_surfaces.c) and the noun
// catalog (control_surfaces_nouns.c).  Not part of the host-facing API.
// ---------------------------------------------------------------------------

// The noun descriptor table (indexed by CsNoun).
extern const CsNounDesc cs_noun_table[CS_NOUN_COUNT];

// Live value of a noun in its natural units (bool 0/1, enum index,
// continuous dB/Hz/Q/percent).  target/index are pre-validated.
float cs_noun_get(uint8_t noun, uint8_t target, uint8_t index);

// Dispatch a resolved absolute target through the shared vendor-command
// surface.  Returns false only on CTRL_DISPATCH_BUSY (caller retries next
// tick); OK and ERROR both count as done.
bool cs_noun_dispatch(uint8_t noun, uint8_t target, uint8_t index, float value);

// Validate a binding's target/index against the noun's addressing kind and
// the platform's live channel/band layout.  Returns PIN_CONFIG_SUCCESS or
// CS_STATUS_INVALID_TARGET.  Single-target form; grouped bindings validate
// each resolved member through cs_noun_validate_target_ch.
uint8_t cs_noun_validate_target(const CsBinding *b);
uint8_t cs_noun_validate_target_ch(uint8_t noun, uint8_t ch, uint8_t index);

// Grouped-reference check against the live group table (engine-owned);
// exported for the display module's page validation.
uint8_t cs_validate_grouped_target(const CsBinding *b);

// A continuous noun's full range in natural units, decoded from min_q/max_q;
// exported for the display module's level bars.  Returns false for nouns
// with no usable span (non-continuous, or min >= max).
bool cs_noun_span(uint8_t noun, float *lo, float *hi);

// Running macro index (255 = idle); the CS_NOUN_MACRO live read.
uint8_t cs_macro_running_index(void);

#endif // CONTROL_SURFACES_H
