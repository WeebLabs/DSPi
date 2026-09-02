/*
 * control_surfaces_display.c; the Control Surfaces I2C display module.
 *
 * One display component (CS_TYPE_DISPLAY) drives a character LCD/OLED or a
 * font-rendered graphic OLED as the device front panel: an event overlay
 * pops the parameter that just changed, and home content follows the
 * configured mode (fixed page / cycle selected / cycle all).  A page is
 * {noun, target} from the ordinary noun catalog, rendered generically by
 * kind and unit; groups render with their user-assigned names.
 *
 * Main-loop only, on the shared 1 kHz tick, and never blocking: all bus
 * traffic goes through a word ring drained into the I2C TX FIFO a few
 * bytes per tick (the hardware transmits in the background), with delay
 * sentinels for the HD44780 long commands.  A full dirty row lands in a
 * few tens of ticks; the audio pump and the other CS components never
 * wait.  See Documentation/Features/control_surfaces_display_spec.md.
 */

#include "control_surfaces_display.h"
#include "config.h"
#include "flash_storage.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "crossfeed.h"
#include "leveller.h"
#include "upmix.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Geometry / model tables
// ---------------------------------------------------------------------------

#define DISP_MAX_COLS   21    // small-font columns on a 128 px OLED
#define DISP_LARGE_COLS 12    // 2x-scaled font columns (10 px per glyph)

// bar_row is where a CS_DPAGE_BAR page draws its level bar.  When it equals
// value_row the panel has no spare line, so the value merges into the label
// line and the bar takes the row (the 2-row layout); DISP_NO_BAR_ROW means
// the model draws no bar line at all (graphic panels invert behind the text
// instead).  See the feature spec, s5.2.
#define DISP_NO_BAR_ROW 0xFF

typedef struct {
    uint8_t cols, rows;       // text geometry (small font for graphic models)
    uint8_t label_row;        // text row of the label line
    uint8_t value_row;        // text row of the value line
    uint8_t bar_row;          // text row of the level bar, or DISP_NO_BAR_ROW
    uint8_t graphic;          // 0 = character module, 1 = SSD1306, 2 = SH1106
    uint8_t pcf;              // HD44780 behind a PCF8574 backpack
    uint8_t def_addr;
    uint32_t baud;
} DispModelDesc;

static const DispModelDesc s_models[CS_DISP_MODEL_COUNT] = {
    [CS_DISP_MODEL_NONE]           = {0},
    [CS_DISP_MODEL_LCD1602]        = {16, 2, 0, 1, 1, 0, 1, 0x27, 100000},
    [CS_DISP_MODEL_LCD2004]        = {20, 4, 1, 2, 3, 0, 1, 0x27, 100000},
    [CS_DISP_MODEL_CHAR_OLED_16X2] = {16, 2, 0, 1, 1, 0, 0, 0x3C, 400000},
    [CS_DISP_MODEL_CHAR_OLED_20X2] = {20, 2, 0, 1, 1, 0, 0, 0x3C, 400000},
    [CS_DISP_MODEL_CHAR_OLED_20X4] = {20, 4, 1, 2, 3, 0, 0, 0x3C, 400000},
    [CS_DISP_MODEL_SSD1306_128X64] = {21, 8, 0, 3, DISP_NO_BAR_ROW, 1, 0, 0x3C, 400000},
    [CS_DISP_MODEL_SSD1306_128X32] = {21, 4, 0, 2, DISP_NO_BAR_ROW, 1, 0, 0x3C, 400000},
    [CS_DISP_MODEL_SH1106_128X64]  = {21, 8, 0, 3, DISP_NO_BAR_ROW, 2, 0, 0x3C, 400000},
};

// CGRAM bar cells: codes 1-5 fill 1-5 of the 5 glyph columns (code 0 is
// avoided so bar strings stay NUL-terminated C strings).  Building the
// solid cell here too means no dependency on the module's ROM block glyph.
#define DISP_BAR_SUBCOL 5
static const uint8_t s_bar_cgram[DISP_BAR_SUBCOL][8] = {
    {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00},
    {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00},
    {0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x00},
    {0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x00},
    {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00},
};

// HD44780 DDRAM row origins.  US2066 uses 0x20 spacing in 3/4-line mode.
static const uint8_t s_hd_rows[4] = {0x00, 0x40, 0x14, 0x54};
static const uint8_t s_us_rows[4] = {0x00, 0x20, 0x40, 0x60};

// ---------------------------------------------------------------------------
// 5x8 font, ASCII 32..126, column-major, LSB = top row.  The classic
// public-domain 5x7 glyph set; LARGE text is this font pixel-doubled.
// ---------------------------------------------------------------------------
static const uint8_t s_font[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, // ' ' '!'
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14}, // '"' '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, // '$' '%'
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, // '&' '\''
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, // '(' ')'
    {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08}, // '*' '+'
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, // ',' '-'
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02}, // '.' '/'
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, // '0' '1'
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, // '2' '3'
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, // '4' '5'
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, // '6' '7'
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, // '8' '9'
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00}, // ':' ';'
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, // '<' '='
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, // '>' '?'
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, // '@' 'A'
    {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22}, // 'B' 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, // 'D' 'E'
    {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A}, // 'F' 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, // 'H' 'I'
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, // 'J' 'K'
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, // 'L' 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, // 'N' 'O'
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, // 'P' 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31}, // 'R' 'S'
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, // 'T' 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, // 'V' 'W'
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, // 'X' 'Y'
    {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00}, // 'Z' '['
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, // '\\' ']'
    {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40}, // '^' '_'
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, // '`' 'a'
    {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20}, // 'b' 'c'
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, // 'd' 'e'
    {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E}, // 'f' 'g'
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, // 'h' 'i'
    {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00}, // 'j' 'k'
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, // 'l' 'm'
    {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38}, // 'n' 'o'
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, // 'p' 'q'
    {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20}, // 'r' 's'
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, // 't' 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C}, // 'v' 'w'
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, // 'x' 'y'
    {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00}, // 'z' '{'
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, // '|' '}'
    {0x08,0x04,0x08,0x10,0x08},                             // '~'
};

// Pixel-doubling LUT: nibble -> byte with each bit repeated.
static const uint8_t s_dbl[16] = {
    0x00,0x03,0x0C,0x0F,0x30,0x33,0x3C,0x3F,
    0xC0,0xC3,0xCC,0xCF,0xF0,0xF3,0xFC,0xFF,
};

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

// TX word ring: low 8 bits = data, bit 9 = STOP, bit 10 = RESTART (the DW
// data_cmd encoding, written to the register verbatim).  0x8000 | ms is a
// delay sentinel (wait, then continue).
#define TXQ_LEN        64
#define TXW_STOP       0x0200u
#define TXW_DELAY      0x8000u

#define DISP_INIT_RETRY_MS  1000u
#define DISP_RENDER_DECIM   32u    // render/compare every 32 ticks
#define DISP_POLL_DECIM     128u   // host-change poll every 128 ticks

// Driver / init states (CsDisplayStatus.init_state)
#define DSTATE_DOWN   0
#define DSTATE_INIT   1
#define DSTATE_LIVE   2
#define DSTATE_ERROR  3

static CsDisplayFlash s_disp;             // live config + pages (flash on save)
static bool       s_attached = false;
static uint8_t    s_model = 0;
static uint8_t    s_addr = 0;
static uint32_t   s_baud = 0;
static i2c_inst_t *s_i2c = NULL;
static uint8_t    s_state = DSTATE_DOWN;
static uint16_t   s_nak = 0;
static uint32_t   s_ms = 0;               // module millisecond clock (ticks)
static uint32_t   s_retry_at = 0;

static uint16_t   s_txq[TXQ_LEN];
static uint8_t    s_txq_head = 0, s_txq_tail = 0;
static uint32_t   s_delay_until = 0;
static uint32_t   s_last_drain = 0;   // ms of the last word leaving the ring

// A wedged bus (target stretching SCL forever, SDA held low) raises no
// abort; if queued words make no progress this long, fault and re-init.
#define DISP_STALL_MS  500u

// Producer: init script cursor, then row serialization progress.
static uint16_t   s_init_pos = 0;
static bool       s_init_done = false;
static uint8_t    s_row_cur = 0xFF;       // physical text row being sent
static uint8_t    s_row_col = 0;          // progress within it
static uint8_t    s_row_phase = 0;        // 0 = addressing, 1 = payload
static uint8_t    s_dirty = 0;            // bit per physical text row (<= 8)

// Last-transmitted content of the two logical lines (render diffs against
// these; the working strings live on disp_render's stack).
static char       s_label_out[DISP_MAX_COLS + 1];
static char       s_value_out[DISP_MAX_COLS + 1];
static char       s_bar_out[DISP_MAX_COLS + 1];   // character-cell level bar
// Graphic panels carry the bar as an inverted run behind the value row
// instead of a line of their own; 0 = no highlight.
static uint8_t    s_bar_px = 0;

// Content state
static uint8_t    s_cur_page = 0xFF;      // home page currently shown
static uint32_t   s_cycle_at = 0;
static uint8_t    s_cycle_noun = 0;       // CYCLE_ALL cursor
static bool       s_ov_active = false;
static CsDisplayPage s_ov_item;           // overlay item (may be synthesized)
static uint32_t   s_ov_until = 0;
static bool       s_edit = false;
static uint32_t   s_edit_until = 0;
static bool       s_render_now = false;
static float      s_poll_cache[CS_MAX_DISPLAY_PAGES];

// ---------------------------------------------------------------------------
// TX ring and pump
// ---------------------------------------------------------------------------

static uint8_t txq_free(void) {
    return (uint8_t)(TXQ_LEN - 1 - ((s_txq_head - s_txq_tail) & (TXQ_LEN - 1)));
}

static void txq_push(uint16_t w) {
    s_txq[s_txq_head] = w;
    s_txq_head = (uint8_t)((s_txq_head + 1) & (TXQ_LEN - 1));
}

static bool txq_empty(void) { return s_txq_head == s_txq_tail; }

// Abort (NAK, arbitration) recovery: count it, drop everything queued, and
// re-run the init script after a backoff so an unplugged or wedged module
// keeps retrying at ~1 Hz without disturbing the rest of the tick.
static void disp_fault(void) {
    if (s_nak < 0xFFFF) s_nak++;
    s_last_drain = s_ms;   // space stall detections out across retries
    (void)i2c_get_hw(s_i2c)->clr_tx_abrt;
    s_txq_head = s_txq_tail = 0;
    s_delay_until = 0;
    s_init_pos = 0;
    s_init_done = false;
    s_row_cur = 0xFF;
    s_state = DSTATE_ERROR;
    s_retry_at = s_ms + DISP_INIT_RETRY_MS;
}

static void disp_pump(void) {
    i2c_hw_t *hw = i2c_get_hw(s_i2c);
    if (hw->tx_abrt_source) { disp_fault(); return; }
    if (s_delay_until && (int32_t)(s_ms - s_delay_until) < 0) {
        s_last_drain = s_ms;   // deliberate wait is not a stall
        return;
    }
    s_delay_until = 0;
    if (txq_empty()) {
        s_last_drain = s_ms;
        return;
    }
    bool moved = false;
    while (!txq_empty() && i2c_get_write_available(s_i2c) > 0) {
        uint16_t w = s_txq[s_txq_tail];
        if (w & TXW_DELAY) {
            s_delay_until = s_ms + (w & 0x7FFF);
            s_txq_tail = (uint8_t)((s_txq_tail + 1) & (TXQ_LEN - 1));
            s_last_drain = s_ms;
            return;
        }
        hw->data_cmd = w;
        s_txq_tail = (uint8_t)((s_txq_tail + 1) & (TXQ_LEN - 1));
        moved = true;
    }
    if (moved) s_last_drain = s_ms;
    else if ((uint32_t)(s_ms - s_last_drain) >= DISP_STALL_MS)
        disp_fault();   // abort-less wedge: FIFO full and nothing draining
}

// ---------------------------------------------------------------------------
// Model serializers.  Each emits words for one step of work when ring space
// allows; callers loop until space runs out.
// ---------------------------------------------------------------------------

// PCF8574 backpack wiring: P0 RS, P1 RW, P2 EN, P3 backlight, P4-7 data.
#define PCF_RS 0x01
#define PCF_EN 0x04
#define PCF_BL 0x08

// One HD44780 byte via the expander = 4 expander writes (EN pulse per
// nibble); all four ride one I2C transaction (STOP on the last).
static void hd_emit(uint8_t b, bool data, bool stop) {
    uint8_t rs = data ? PCF_RS : 0;
    uint8_t hi = (uint8_t)((b & 0xF0) | rs | PCF_BL);
    uint8_t lo = (uint8_t)(((b << 4) & 0xF0) | rs | PCF_BL);
    txq_push(hi | PCF_EN);
    txq_push(hi);
    txq_push(lo | PCF_EN);
    txq_push(lo | (stop ? TXW_STOP : 0));
}

// Raw single-nibble write (init's 8-bit-mode knocks).
static void hd_emit_nibble(uint8_t hi_nibble) {
    uint8_t v = (uint8_t)((hi_nibble << 4) | PCF_BL);
    txq_push(v | PCF_EN);
    txq_push(v | TXW_STOP);
}

// US2066 / char-OLED: control byte 0x00 = command, 0x40 = data.
static void us_cmd(uint8_t c) { txq_push(0x00); txq_push(c | TXW_STOP); }
static void us_dat(uint8_t d) { txq_push(0x40); txq_push(d | TXW_STOP); }

// SSD1306/SH1106 command stream.
static void ox_cmd1(uint8_t c) { txq_push(0x00); txq_push(c | TXW_STOP); }
static void ox_cmd2(uint8_t c, uint8_t a) {
    txq_push(0x00); txq_push(c); txq_push(a | TXW_STOP);
}

// Init scripts run as step lists so they interleave with ring space.  Each
// step emits a bounded burst; a return of true means the script advanced.
static bool disp_init_step(void) {
    const DispModelDesc *m = &s_models[s_model];
    uint16_t i = s_init_pos;
    if (m->pcf) {
        // Tail of the script: the CGRAM bar cells, one glyph per step (a
        // whole glyph is 9 bytes, so it needs far more ring than a command).
        if (i >= 10) {
            uint8_t g = (uint8_t)(i - 10);
            if (g >= DISP_BAR_SUBCOL) { s_init_done = true; return true; }
            if (txq_free() < 40) return false;
            hd_emit((uint8_t)(0x40 | ((g + 1) * 8)), false, true);
            for (uint8_t k = 0; k < 8; k++)
                hd_emit(s_bar_cgram[g][k], true, k == 7);
            s_init_pos++;
            return true;
        }
        // Classic 4-bit bring-up with the three 8-bit-mode knocks.
        if (txq_free() < 14) return false;
        switch (i) {
            case 0: txq_push(TXW_DELAY | 50); break;
            case 1: hd_emit_nibble(0x3); txq_push(TXW_DELAY | 5); break;
            case 2: hd_emit_nibble(0x3); txq_push(TXW_DELAY | 2); break;
            case 3: hd_emit_nibble(0x3); txq_push(TXW_DELAY | 2); break;
            case 4: hd_emit_nibble(0x2); txq_push(TXW_DELAY | 2); break;
            case 5: hd_emit(0x28, false, true); break;  // 4-bit, 2-line, 5x8
            case 6: hd_emit(0x08, false, true); break;  // display off
            case 7: hd_emit(0x01, false, true); txq_push(TXW_DELAY | 3); break;
            case 8: hd_emit(0x06, false, true); break;  // entry mode
            case 9: hd_emit(0x0C, false, true); break;  // display on
            default: break;                             // unreachable, see above
        }
    } else if (!m->graphic) {
        // US2066 bring-up (Newhaven 3.3 V application sequence).
        if (txq_free() < 8) return false;
        static const uint8_t seq[] = {
            0x2A, 0x71, 0xFF, 0x00,   // function select A, data 0x00 (3.3 V)
            0x28, 0x08,
            0x2A, 0x79, 0xD5, 0x70, 0x78,
            0x09,                     // extended: 3/4-line (5-dot, alt font off)
            0x06, 0x72, 0xFF, 0x00,   // ROM A
            0x2A, 0x79, 0xDA, 0x10, 0xDC, 0x00,
            0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40,
            0x78, 0x28,
            0x01,                     // clear (needs settle)
            0x0C,
        };
        if (i >= sizeof(seq)) {
            // Same CGRAM bar cells as the HD44780 path; the sequence has
            // already returned to the base instruction set by here.
            uint8_t g = (uint8_t)(i - sizeof(seq));
            if (g >= DISP_BAR_SUBCOL) { s_init_done = true; return true; }
            if (txq_free() < 20) return false;
            us_cmd((uint8_t)(0x40 | ((g + 1) * 8)));
            for (uint8_t k = 0; k < 8; k++) us_dat(s_bar_cgram[g][k]);
            s_init_pos++;
            return true;
        }
        if (seq[i] == 0xFF) { us_dat(seq[i + 1]); s_init_pos += 2; return true; }
        uint8_t c = seq[i];
        if (c == 0x09 && m->rows == 2) c = 0x08;   // 2-line variant
        if (c == 0x81) {
            // Contrast honours the configured brightness like the OLEDs do.
            us_cmd(c);
            us_cmd(s_disp.cfg.brightness ? s_disp.cfg.brightness : seq[i + 1]);
            s_init_pos += 2;
            return true;
        }
        us_cmd(c);
        if (c == 0x01) txq_push(TXW_DELAY | 3);
        s_init_pos++;
        return true;
    } else {
        // SSD1306 / SH1106, page addressing mode throughout.  The SH1106
        // knows neither the charge-pump nor the addressing-mode command
        // (its DC-DC defaults on and page mode is its only mode), and 0x14
        // and 0x02 would land in its column-pointer command ranges.
        if (txq_free() < 8) return false;
        uint8_t mux = (uint8_t)(m->rows * 8 - 1);
        bool sh1106 = (m->graphic == 2);
        switch (i) {
            case 0:  ox_cmd1(0xAE); break;
            case 1:  ox_cmd2(0xD5, 0x80); break;
            case 2:  ox_cmd2(0xA8, mux); break;
            case 3:  ox_cmd2(0xD3, 0x00); break;
            case 4:  ox_cmd1(0x40); break;
            case 5:  if (!sh1106) ox_cmd2(0x8D, 0x14); break;   // charge pump on
            case 6:  if (!sh1106) ox_cmd2(0x20, 0x02); break;   // page addressing
            case 7:  ox_cmd1(0xA1); break;
            case 8:  ox_cmd1(0xC8); break;
            case 9:  ox_cmd2(0xDA, m->rows == 8 ? 0x12 : 0x02); break;
            case 10: ox_cmd2(0x81, s_disp.cfg.brightness ? s_disp.cfg.brightness : 0x7F); break;
            case 11: ox_cmd2(0xD9, 0xF1); break;
            case 12: ox_cmd2(0xDB, 0x40); break;
            case 13: ox_cmd1(0xA4); break;
            case 14: ox_cmd1(0xA6); break;
            case 15: ox_cmd1(0xAF); break;
            default: s_init_done = true; return true;
        }
    }
    s_init_pos++;
    return true;
}

// ---------------------------------------------------------------------------
// Row transmission.  Content lives in two logical lines mapped onto
// physical rows; graphic models render font columns on the fly.
// ---------------------------------------------------------------------------

// The text (and, for graphic models, which font) shown on physical row r.
// Value lines render LARGE on graphic models when the shown item asks for
// it; the value then occupies two page rows (value_row and value_row + 1).
static bool disp_value_large(void);

static bool disp_bar_line(void);

static const char *disp_row_text(uint8_t r, bool *large, uint8_t *large_half) {
    const DispModelDesc *m = &s_models[s_model];
    *large = false;
    *large_half = 0;
    // The bar owns its row outright; on 2-row panels that is the value row,
    // and the value has already been folded into the label line.
    if (disp_bar_line() && r == m->bar_row) return s_bar_out;
    if (r == m->label_row) return s_label_out;
    if (m->graphic && disp_value_large()) {
        if (r == m->value_row)     { *large = true; *large_half = 0; return s_value_out; }
        if (r == m->value_row + 1) { *large = true; *large_half = 1; return s_value_out; }
    } else if (r == m->value_row) {
        return s_value_out;
    }
    return NULL;   // blank row
}

// Emit the next chunk of the current dirty row; returns true when the row
// completed.  Character modules: DDRAM address then one char per call
// batch; graphic: page/column setup then font columns.
static bool disp_row_step(void) {
    const DispModelDesc *m = &s_models[s_model];
    bool large; uint8_t half;
    const char *text = disp_row_text(s_row_cur, &large, &half);

    if (!m->graphic) {
        uint8_t base = m->pcf ? s_hd_rows[s_row_cur] : s_us_rows[s_row_cur];
        if (s_row_phase == 0) {
            if (txq_free() < 6) return false;
            if (m->pcf) hd_emit((uint8_t)(0x80 | base), false, true);
            else        us_cmd((uint8_t)(0x80 | base));
            s_row_phase = 1;
            s_row_col = 0;
            return false;
        }
        while (s_row_col < m->cols) {
            if (txq_free() < 6) return false;
            // Line buffers are NUL-padded, so past-the-end reads blanks.
            char c = text ? text[s_row_col] : 0;
            if (c == 0) c = ' ';
            if (m->pcf) hd_emit((uint8_t)c, true, s_row_col == m->cols - 1);
            else        us_dat((uint8_t)c);
            s_row_col++;
        }
        return true;
    }

    // Graphic: one 128-column page per text row.
    if (s_row_phase == 0) {
        if (txq_free() < 8) return false;
        uint8_t col = (m->graphic == 2) ? 2 : 0;   // SH1106 132-col offset
        ox_cmd1((uint8_t)(0xB0 | s_row_cur));
        ox_cmd1((uint8_t)(0x00 | (col & 0x0F)));
        ox_cmd1((uint8_t)(0x10 | (col >> 4)));
        txq_push(0x40);   // open the data transaction
        s_row_phase = 1;
        s_row_col = 0;
        return false;
    }
    uint8_t glyph_w = large ? 10 : 6;
    uint8_t ncols = large ? DISP_LARGE_COLS : DISP_MAX_COLS;
    uint8_t tlen = text ? (uint8_t)strlen(text) : 0;   // once, not per column
    // Graphic level bar: the value row's pixels are inverted up to s_bar_px,
    // giving a lit block with the glyphs knocked out of it.
    bool hl = s_bar_px && (s_row_cur == m->value_row ||
                           (large && s_row_cur == m->value_row + 1));
    while (s_row_col < 128) {
        if (txq_free() < 2) return false;
        uint8_t ci = s_row_col / glyph_w;
        uint8_t cx = s_row_col % glyph_w;
        uint8_t px = 0;
        if (ci < ncols && ci < tlen) {
            char c = text[ci];
            if (c < 32 || c > 126) c = ' ';
            const uint8_t *g = s_font[c - 32];
            if (large) {
                if (cx < 10) {
                    uint8_t fb = g[cx / 2];
                    px = half ? s_dbl[(fb >> 4) & 0xF] : s_dbl[fb & 0xF];
                }
            } else if (cx < 5) {
                px = g[cx];
            }
        }
        if (hl && s_row_col < s_bar_px) px = (uint8_t)~px;
        bool last = (s_row_col == 127);
        txq_push((uint16_t)(px | (last ? TXW_STOP : 0)));
        s_row_col++;
    }
    return true;
}

// Mark the physical rows carrying a logical line dirty.
#define DISP_LINE_LABEL 0
#define DISP_LINE_VALUE 1
#define DISP_LINE_BAR   2

static void disp_dirty_line(uint8_t line) {
    const DispModelDesc *m = &s_models[s_model];
    if (line == DISP_LINE_LABEL) {
        s_dirty |= (uint8_t)(1u << m->label_row);
        return;
    }
    if (line == DISP_LINE_BAR) {
        // Graphic panels carry the bar as inverted pixels on the value row,
        // so a bar change dirties exactly what a value change would.
        if (m->bar_row == DISP_NO_BAR_ROW) line = DISP_LINE_VALUE;
        else { s_dirty |= (uint8_t)(1u << m->bar_row); return; }
    }
    s_dirty |= (uint8_t)(1u << m->value_row);
    if (m->graphic) s_dirty |= (uint8_t)(1u << (m->value_row + 1));
}

// ---------------------------------------------------------------------------
// Content: item resolution, labels, value formatting
// ---------------------------------------------------------------------------

// Short label per noun; keep within ~10 chars so targets still fit.
static const char *const s_noun_label[CS_NOUN_COUNT] = {
    [CS_NOUN_USER_VOLUME] = "Volume",      [CS_NOUN_MASTER_VOLUME] = "Max Volume",
    [CS_NOUN_USER_MUTE] = "Mute",          [CS_NOUN_LOUDNESS] = "Loudness",
    [CS_NOUN_CROSSFEED] = "Crossfeed",     [CS_NOUN_LEVELLER] = "Leveller",
    [CS_NOUN_PRESET] = "Preset",           [CS_NOUN_INPUT_SOURCE] = "Input",
    [CS_NOUN_CLIP] = "Clip",               [CS_NOUN_EQ_BYPASS] = "EQ Bypass",
    [CS_NOUN_LG_SYNC] = "LG Sync",         [CS_NOUN_CROSSFEED_PRESET] = "XF Preset",
    [CS_NOUN_CROSSFEED_ITD] = "XF ITD",    [CS_NOUN_LEVELLER_AMOUNT] = "Lev Amount",
    [CS_NOUN_LEVELLER_SPEED] = "Lev Speed",[CS_NOUN_LEVELLER_LOOKAHEAD] = "Lev Look",
    [CS_NOUN_PREAMP] = "Preamp",           [CS_NOUN_OUTPUT_GAIN] = "Gain",
    [CS_NOUN_OUTPUT_MUTE] = "Mute",        [CS_NOUN_OUTPUT_ENABLE] = "Enable",
    [CS_NOUN_FILTER_FREQ] = "Freq",        [CS_NOUN_FILTER_GAIN] = "Flt Gain",
    [CS_NOUN_FILTER_Q] = "Q",              [CS_NOUN_FILTER_TYPE] = "Flt Type",
    [CS_NOUN_FILTER_BYPASS] = "Flt Byp",   [CS_NOUN_SIGGEN] = "Generator",
    [CS_NOUN_DAC_MUTE_TEST] = "DAC Mute",  [CS_NOUN_CLIP_CH] = "Clip",
    [CS_NOUN_LEVEL] = "Level",             [CS_NOUN_SPDIF_LOCK] = "SPDIF Lock",
    [CS_NOUN_SAMPLE_RATE] = "Rate",        [CS_NOUN_USB_STREAMING] = "USB Stream",
    [CS_NOUN_ADAT_ACTIVE] = "ADAT Out",    [CS_NOUN_LG_PRESENT] = "LG Source",
    [CS_NOUN_LG_MUTED] = "LG Muted",       [CS_NOUN_UPMIX] = "Upmix",
    [CS_NOUN_UPMIX_CENTER_MODE] = "Centre Md", [CS_NOUN_UPMIX_SURROUND_MODE] = "Surr Mode",
    [CS_NOUN_UPMIX_STRENGTH] = "Upmix Str",[CS_NOUN_UPMIX_WIDTH] = "Centre Wid",
    [CS_NOUN_UPMIX_PRESENCE] = "Presence", [CS_NOUN_PSYBASS] = "PsyBass",
    [CS_NOUN_PSYBASS_CUTOFF] = "PB Cutoff",[CS_NOUN_PSYBASS_HARMONICS] = "PB Harm",
    [CS_NOUN_PSYBASS_DRIVE] = "PB Drive",  [CS_NOUN_PSYBASS_CHARACTER] = "PB Char",
    [CS_NOUN_PSYBASS_ORIGINAL] = "PB Orig",[CS_NOUN_OUTPUT_DELAY] = "Delay",
    [CS_NOUN_PRESET_RELOAD] = "Reload",    [CS_NOUN_LOUDNESS_SPL] = "Ref SPL",
    [CS_NOUN_LOUDNESS_INTENSITY] = "Loud Amt", [CS_NOUN_INPUT_LEVEL_MAX] = "In Level",
    [CS_NOUN_MACRO] = "Macro",             [CS_NOUN_CPU_LOAD] = "CPU",
    [CS_NOUN_DISPLAY_PAGE] = "Page",       [CS_NOUN_DISPLAY_EDIT] = "Edit",
    [CS_NOUN_PAGE_VALUE] = "Value",       [CS_NOUN_SUBHARM] = "Subharm",
    [CS_NOUN_SUBHARM_LOW] = "Sub 24-36",  [CS_NOUN_SUBHARM_HIGH] = "Sub 36-56",
    [CS_NOUN_SUBHARM_BOOST] = "LF Boost",
};

static const char *const s_input_label[] = {"USB", "SPDIF", "I2S", "ADAT",
                                            "SPDIF 2", "SPDIF 3", "SPDIF 4"};
static const char *const s_rate_label[]  = {"44.1 kHz", "48 kHz", "96 kHz"};
static const char *const s_xf_preset_label[] = {"Default", "Chu Moy", "Meier", "Custom"};
static const char *const s_lev_speed_label[] = {"Slow", "Medium", "Fast"};
static const char *const s_center_mode_label[]   = {"Sinner", "Logician", "Off"};
static const char *const s_surround_mode_label[] = {"Off", "Sinner", "Logician"};
// Indexed by FilterType; includes the host-only types above the CS cycling
// range (Linkwitz, first-order LP/HP) so a host-set band still reads by name.
// PEQ low/high pass display as High Cut / Low Cut; two tables because the
// 12-column LARGE font needs abbreviated names (edit marker + name <= 12).
static const char *const s_filter_type_label[] = {
    "Flat", "Peaking", "Low Shelf 12dB", "High Shelf 12dB", "High Cut 12dB",
    "Low Cut 12dB", "Notch", "All Pass", "All Pass 1", "Low Shelf 6dB",
    "High Shelf 6dB", "Linkwitz", "High Cut 6dB", "Low Cut 6dB",
};
static const char *const s_filter_type_large[] = {
    "Flat", "Peaking", "LS 12dB/oct", "HS 12dB/oct", "HC 12dB/oct",
    "LC 12dB/oct", "Notch", "All Pass", "All Pass 1", "LS 6dB/oct",
    "HS 6dB/oct", "Linkwitz", "HC 6dB/oct", "LC 6dB/oct",
};
_Static_assert(sizeof(s_filter_type_label) == sizeof(s_filter_type_large),
               "filter-type name tables must cover the same types");

// Growing one of these enums must break the build here, not silently
// reintroduce the numeric fallback on the front panel.
#define DISP_N(t)  (sizeof(t) / sizeof((t)[0]))
_Static_assert(DISP_N(s_input_label) == INPUT_SOURCE_MAX + 1,
               "input-source names must cover the enum");
_Static_assert(DISP_N(s_filter_type_label) == FILTER_HIGHPASS1 + 1,
               "filter-type names must cover the PEQ types");
_Static_assert(DISP_N(s_xf_preset_label) == CROSSFEED_PRESET_CUSTOM + 1,
               "crossfeed preset names must cover the enum");
_Static_assert(DISP_N(s_lev_speed_label) == LEVELLER_SPEED_COUNT,
               "leveller speed names must cover the enum");
#if PICO_RP2350
_Static_assert(DISP_N(s_center_mode_label) == UPMIX_CENTER_OFF + 1,
               "centre mode names must cover the enum");
_Static_assert(DISP_N(s_surround_mode_label) == UPMIX_SURROUND_ADAPTIVE + 1,
               "surround mode names must cover the enum");
#endif

#define DISP_TAB(t)  do { tab = (t); count = (int)(sizeof(t) / sizeof((t)[0])); } while (0)

// Display name for an enum noun's value; NULL falls back to a number.
// large selects the abbreviated names sized for the 12-column LARGE font.
static const char *disp_enum_label(uint8_t noun, int v, bool large) {
    const char *const *tab = NULL;
    int count = 0;
    switch (noun) {
        case CS_NOUN_INPUT_SOURCE:        DISP_TAB(s_input_label); break;
        case CS_NOUN_SAMPLE_RATE:         DISP_TAB(s_rate_label); break;
        case CS_NOUN_CROSSFEED_PRESET:    DISP_TAB(s_xf_preset_label); break;
        case CS_NOUN_LEVELLER_SPEED:      DISP_TAB(s_lev_speed_label); break;
        case CS_NOUN_UPMIX_CENTER_MODE:   DISP_TAB(s_center_mode_label); break;
        case CS_NOUN_UPMIX_SURROUND_MODE: DISP_TAB(s_surround_mode_label); break;
        case CS_NOUN_FILTER_TYPE:
            if (large) DISP_TAB(s_filter_type_large);
            else       DISP_TAB(s_filter_type_label);
            break;
        default: break;
    }
    return (tab && v >= 0 && v < count) ? tab[v] : NULL;
}

// value with one decimal, no printf float support needed.
static void fmt_fix1(char *out, size_t n, float v) {
    int neg = v < 0;
    int q = (int)lroundf(fabsf(v) * 10.0f);
    snprintf(out, n, "%s%d.%d", neg ? "-" : "", q / 10, q % 10);
}

static bool disp_item_readonly(const CsNounDesc *nd) {
    const uint16_t writers = CS_ACT_BIT(CS_ACT_ADJUST) | CS_ACT_BIT(CS_ACT_STEP) |
        CS_ACT_BIT(CS_ACT_INC) | CS_ACT_BIT(CS_ACT_DEC) | CS_ACT_BIT(CS_ACT_SET) |
        CS_ACT_BIT(CS_ACT_TOGGLE) | CS_ACT_BIT(CS_ACT_FOLLOW) |
        CS_ACT_BIT(CS_ACT_MOMENTARY) | CS_ACT_BIT(CS_ACT_TRIGGER);
    return (nd->actions & writers) == 0;
}

// Live value of an item; grouped items read their anchor (lowest member).
// Grouped pages re-validate through the engine so a group edited under
// them (emptied, re-kinded) reads as unavailable, matching bindings.
static float disp_item_get(const CsDisplayPage *p, bool *ok) {
    *ok = true;
    uint8_t target = p->target;
    if (p->flags & CS_DPAGE_GROUP) {
        CsBinding v;
        memset(&v, 0, sizeof(v));
        v.noun = p->noun;
        v.flags = CS_FLAG_GROUP;
        v.target = p->target;
        v.index = p->index;
        if (cs_validate_grouped_target(&v) != PIN_CONFIG_SUCCESS) {
            *ok = false;
            return 0.0f;
        }
        const CsGroupConfig *gc = control_surfaces_group_config();
        target = (uint8_t)__builtin_ctz(gc->groups[p->target].member_mask);
    }
    return cs_noun_get(p->noun, target, p->index);
}

static void disp_format_value(const CsDisplayPage *p, char *out, size_t n,
                              bool large) {
    const CsNounDesc *nd = &cs_noun_table[p->noun];
    bool ok;
    float v = disp_item_get(p, &ok);
    if (!ok) { snprintf(out, n, "--"); return; }

    if (p->noun == CS_NOUN_PRESET) {
        char name[PRESET_NAME_LEN] = {0};
        if (preset_get_name((uint8_t)v, name) == 0 && name[0])
            snprintf(out, n, "%s", name);
        else
            snprintf(out, n, "Preset %d", (int)v + 1);
        return;
    }
    if (p->noun == CS_NOUN_MACRO) {
        if ((int)v >= CS_MAX_MACROS) { snprintf(out, n, "Idle"); return; }
        const CsMacro *m = control_surfaces_get_macro((uint8_t)v);
        if (m && m->name[0]) snprintf(out, n, "%s", m->name);
        else snprintf(out, n, "Macro %d", (int)v + 1);
        return;
    }

    char num[12];
    switch (nd->kind) {
        case CS_KIND_BOOL:
            snprintf(out, n, "%s", (v >= 0.5f) ? "On" : "Off");
            return;
        case CS_KIND_ENUM: {
            const char *name = disp_enum_label(p->noun, (int)v, large);
            if (name) snprintf(out, n, "%s", name);
            else snprintf(out, n, "#%d", (int)v + 1);
            return;
        }
        default:
            break;
    }
    switch (nd->unit) {
        case CS_UNIT_DB:
            fmt_fix1(num, sizeof(num), v);
            snprintf(out, n, "%s dB", num);
            break;
        case CS_UNIT_HZ:
            snprintf(out, n, "%d Hz", (int)lroundf(v));
            break;
        case CS_UNIT_Q:
            fmt_fix1(num, sizeof(num), v);
            snprintf(out, n, "Q %s", num);
            break;
        case CS_UNIT_PERCENT:
            snprintf(out, n, "%d%%", (int)lroundf(v));
            break;
        case CS_UNIT_MS:
            fmt_fix1(num, sizeof(num), v);
            snprintf(out, n, "%s ms", num);
            break;
        default:
            snprintf(out, n, "%d", (int)v);
            break;
    }
}

// `tight` picks the one-letter channel prefixes (O1/I2/C3/C3B4) used when the
// label shares its row with the value on a 2-row bar page.
static void disp_format_label_ex(const CsDisplayPage *p, char *out, size_t n,
                                 bool tight) {
    const CsNounDesc *nd = &cs_noun_table[p->noun];
    const char *base = s_noun_label[p->noun] ? s_noun_label[p->noun] : "?";
    if (p->flags & CS_DPAGE_GROUP) {
        const CsGroupConfig *gc = control_surfaces_group_config();
        const char *gn = (p->target < CS_MAX_GROUPS &&
                          gc->groups[p->target].name[0])
                       ? gc->groups[p->target].name : "Group";
        snprintf(out, n, "%s %s", gn, base);
        return;
    }
    switch (nd->target_kind) {
        case CS_TARGET_INPUT_CH:
            snprintf(out, n, tight ? "I%d %s" : "In%d %s", p->target + 1, base);
            break;
        case CS_TARGET_OUTPUT_CH:
            snprintf(out, n, tight ? "O%d %s" : "Out%d %s", p->target + 1, base);
            break;
        case CS_TARGET_DSP_CH:
            snprintf(out, n, tight ? "C%d %s" : "Ch%d %s", p->target + 1, base);
            break;
        case CS_TARGET_DSP_BAND:
            snprintf(out, n, tight ? "C%dB%d %s" : "Ch%d B%d %s",
                     p->target + 1, p->index + 1, base);
            break;
        default:
            snprintf(out, n, "%s", base);
            break;
    }
}

static void disp_format_label(const CsDisplayPage *p, char *out, size_t n) {
    disp_format_label_ex(p, out, n, false);
}

// Strip the space before a unit suffix ("-12.5 dB" -> "-12.5dB"); buys a
// column on the combined line where every one counts.
static void disp_tighten_value(char *s) {
    char *sp = NULL;
    for (char *c = s; *c; c++) if (*c == ' ') sp = c;
    if (!sp || sp == s || !sp[1]) return;
    memmove(sp, sp + 1, strlen(sp + 1) + 1);
}

// ---------------------------------------------------------------------------
// View selection (overlay > home) and rendering
// ---------------------------------------------------------------------------

static bool disp_page_valid(uint8_t idx) {
    return idx < CS_MAX_DISPLAY_PAGES &&
           (s_disp.pages[idx].flags & CS_DPAGE_ACTIVE);
}

static bool s_view_large = false;
static bool disp_value_large(void) { return s_view_large; }

// The item currently on screen; false when idle with nothing to show.
static bool disp_current_item(CsDisplayPage *out) {
    if (s_ov_active) { *out = s_ov_item; return true; }
    if (s_disp.cfg.mode == CS_DMODE_CYCLE_ALL) {
        CsDisplayPage p = {s_cycle_noun, 0, 0, CS_DPAGE_ACTIVE};
        *out = p;
        return true;
    }
    if (disp_page_valid(s_cur_page)) { *out = s_disp.pages[s_cur_page]; return true; }
    return false;
}

// Alignment field accessor; the reserved encoding reads as LEFT so a blob
// from a future build can never index past the placement cases.
static uint8_t disp_align(uint8_t mask, uint8_t shift) {
    uint8_t a = (uint8_t)((s_disp.cfg.flags & mask) >> shift);
    return (a > CS_DALIGN_RIGHT) ? CS_DALIGN_LEFT : a;
}

// Lay a formatted line out across its physical width, in place: the text
// placed per `align`, then the armed marker(s) in the margin the alignment
// leaves free (right of a left-flushed value, left of a right-flushed one,
// both around a centred one), one blank clear of the text.  Position comes
// from the full width either way, so arming never moves the value; only a
// value long enough to reach a marker gives up columns.
static void disp_lay_out(char *s, uint8_t w, uint8_t align, char marker) {
    char out[DISP_MAX_COLS + 1];
    if (w > DISP_MAX_COLS) w = DISP_MAX_COLS;
    if (w < 4) marker = 0;
    uint8_t need = marker ? ((align == CS_DALIGN_CENTRE) ? 4 : 2) : 0;
    uint8_t len = (uint8_t)strlen(s);
    if (len > w - need) len = (uint8_t)(w - need);
    if (len == 0) marker = 0;   // no lone marker on a blank line
    uint8_t pad = (uint8_t)(w - len);
    uint8_t off = (align == CS_DALIGN_CENTRE) ? (uint8_t)(pad / 2)
                : (align == CS_DALIGN_RIGHT)  ? pad : 0;
    memset(out, ' ', w);
    memcpy(out + off, s, len);
    if (marker && align != CS_DALIGN_RIGHT) {
        uint8_t c = (uint8_t)(off + len + 1);
        if (c > w - 1) c = (uint8_t)(w - 1);
        out[c] = (marker == '>') ? '<' : marker;   // mirrored chevron
    }
    if (marker && align != CS_DALIGN_LEFT)
        out[(off >= 2) ? (uint8_t)(off - 2) : 0] = marker;
    out[w] = '\0';
    memcpy(s, out, (size_t)w + 1);
}

// Whether the shown item draws a level bar, and whether the panel spends a
// text row on it.  Graphic panels invert behind the value instead.
static bool s_view_bar = false;
static bool disp_bar_line(void) {
    return s_view_bar && s_models[s_model].bar_row != DISP_NO_BAR_ROW;
}

// Fraction of the item's full range the live value sits at, 0..1; log-unit
// nouns map logarithmically, as the IND_LEVEL meter LEDs do.  False when the
// noun has no usable span or the read failed.
static bool disp_bar_norm(const CsDisplayPage *p, float *out) {
    float lo, hi;
    if (!cs_noun_span(p->noun, &lo, &hi)) return false;
    bool ok;
    float v = disp_item_get(p, &ok);
    if (!ok) return false;
    const CsNounDesc *nd = &cs_noun_table[p->noun];
    float norm;
    if (nd->unit == CS_UNIT_HZ || nd->unit == CS_UNIT_Q) {
        if (!(lo > 0.0f) || !(v > 0.0f)) return false;
        norm = log2f(v / lo) / log2f(hi / lo);
    } else {
        norm = (v - lo) / (hi - lo);
    }
    // NaN survives a min/max clamp, so gate on the positive test explicitly.
    if (!(norm > 0.0f)) norm = 0.0f;
    else if (norm > 1.0f) norm = 1.0f;
    *out = norm;
    return true;
}

// Fold the value into the label row, flush right, in place.  The value is
// never truncated (it is the number being read); the label gives up columns
// instead, keeping at least one space between them while it can.
static void disp_merge_value(char *label, const char *value, uint8_t cols) {
    uint8_t lw = (uint8_t)strlen(label), vw = (uint8_t)strlen(value);
    if (vw > cols) vw = cols;
    uint8_t room = (uint8_t)(vw < cols ? cols - vw - 1 : 0);   // label columns
    if (lw > room) lw = room;
    memset(label + lw, ' ', (size_t)(cols - vw - lw));
    memcpy(label + cols - vw, value, vw);
    label[cols] = '\0';
}

// Paint a character-cell bar: whole cells from the CGRAM solid glyph, one
// partial cell for the remainder, blanks after.
static void disp_bar_cells(char *out, uint8_t w, float norm) {
    // Casting a negative or NaN float to unsigned is undefined, so gate here
    // as well as at the caller; NaN survives a min/max clamp.
    if (!(norm > 0.0f)) norm = 0.0f;
    else if (norm > 1.0f) norm = 1.0f;
    uint16_t sub = (uint16_t)(norm * (float)w * DISP_BAR_SUBCOL + 0.5f);
    if (sub > (uint16_t)(w * DISP_BAR_SUBCOL)) sub = (uint16_t)(w * DISP_BAR_SUBCOL);
    uint8_t full = (uint8_t)(sub / DISP_BAR_SUBCOL);
    uint8_t rem  = (uint8_t)(sub % DISP_BAR_SUBCOL);
    uint8_t i = 0;
    for (; i < full; i++) out[i] = DISP_BAR_SUBCOL;   // CGRAM code 5 = solid
    if (rem && i < w) out[i++] = (char)rem;
    for (; i < w; i++) out[i] = ' ';
    out[w] = '\0';
}

static void disp_render(void) {
    const DispModelDesc *m = &s_models[s_model];
    CsDisplayPage it;
    char label[DISP_MAX_COLS + 1] = "DSPi";
    char value[DISP_MAX_COLS + 1] = "";
    char bar[DISP_MAX_COLS + 1] = "";
    bool large = false, bar_on = false;
    float norm = 0.0f;
    char marker = 0;         // edit marker glyph, 0 while unarmed
    if (disp_current_item(&it)) {
        large = (it.flags & CS_DPAGE_LARGE) || s_ov_active;
        bar_on = (it.flags & CS_DPAGE_BAR) && disp_bar_norm(&it, &norm);
        // Character modules ignore LARGE, so they keep the long names.
        disp_format_value(&it, value, sizeof(value), large && m->graphic);
        // A read-only item shows a lock instead of the adjust chevron.
        if (s_edit)
            marker = disp_item_readonly(&cs_noun_table[it.noun]) ? '!' : '>';
        // A bar row on a 2-row panel costs the value its own line, so the
        // two share the label row: compact prefixes, value flush right.
        bool merged = bar_on && m->bar_row == m->value_row;
        disp_format_label_ex(&it, label, sizeof(label), merged);
        if (merged) {
            disp_tighten_value(value);
            uint8_t w = m->cols;
            uint8_t vw = (uint8_t)strlen(value);
            if (vw > w) vw = w;
            if (marker && w >= (uint8_t)(vw + 2)) {
                // The value owns the right edge of a merged line, so the
                // marker sits to its left and the label gives up columns.
                uint8_t room = (uint8_t)(w - vw - 2);
                if (strlen(label) > room) label[room] = '\0';
                disp_merge_value(label, value, w);
                label[room] = marker;
            } else {
                disp_merge_value(label, value, w);
            }
            value[0] = '\0';
        }
    }
    // Graphic panels have no bar string to diff, so the pixel extent is the
    // change signal for the inverted run.
    uint8_t bar_px = (bar_on && m->bar_row == DISP_NO_BAR_ROW)
                   ? (uint8_t)(norm * 128.0f + 0.5f) : 0;
    if (bar_px != s_bar_px) {
        s_bar_px = bar_px;
        disp_dirty_line(DISP_LINE_VALUE);
    }
    if (bar_on && m->bar_row != DISP_NO_BAR_ROW)
        disp_bar_cells(bar, m->cols, norm);
    // The merged line owns its own placement; the bar is inherently
    // full-width, so neither takes the configured alignment.
    bool merged = bar_on && m->bar_row == m->value_row;
    if (!merged)
        disp_lay_out(label, m->cols,
                     disp_align(CS_DCFG_LABEL_ALIGN, CS_DCFG_LABEL_ALIGN_SHIFT),
                     0);
    disp_lay_out(value, (large && m->graphic) ? DISP_LARGE_COLS : m->cols,
                 disp_align(CS_DCFG_VALUE_ALIGN, CS_DCFG_VALUE_ALIGN_SHIFT),
                 merged ? 0 : marker);
    if (large != s_view_large) {
        // Glyph pitch changed: the value area must repaint even when the
        // formatted string happens to be identical.
        s_view_large = large;
        disp_dirty_line(DISP_LINE_VALUE);
    }
    if (bar_on != s_view_bar) {
        // Row ownership changed; repaint every line the bar can touch.
        s_view_bar = bar_on;
        disp_dirty_line(DISP_LINE_LABEL);
        disp_dirty_line(DISP_LINE_VALUE);
        disp_dirty_line(DISP_LINE_BAR);
    }
    if (strncmp(label, s_label_out, sizeof(s_label_out)) != 0) {
        memset(s_label_out, 0, sizeof(s_label_out));
        strncpy(s_label_out, label, DISP_MAX_COLS);
        disp_dirty_line(DISP_LINE_LABEL);
    }
    if (strncmp(value, s_value_out, sizeof(s_value_out)) != 0) {
        memset(s_value_out, 0, sizeof(s_value_out));
        strncpy(s_value_out, value, DISP_MAX_COLS);
        disp_dirty_line(DISP_LINE_VALUE);
    }
    if (strncmp(bar, s_bar_out, sizeof(s_bar_out)) != 0) {
        memset(s_bar_out, 0, sizeof(s_bar_out));
        strncpy(s_bar_out, bar, DISP_MAX_COLS);
        disp_dirty_line(DISP_LINE_BAR);
    }
}

// Overlay trigger shared by the dispatch hook and the host-change poll.
static void disp_pop(const CsDisplayPage *item) {
    if (s_disp.cfg.overlay_hold == 0) return;
    s_ov_item = *item;
    s_ov_active = true;
    s_ov_until = s_ms + (uint32_t)s_disp.cfg.overlay_hold * 100u;
    s_render_now = true;
}

// Find a configured page matching an adjusted item.
static int disp_find_page(uint8_t noun, uint8_t target, uint8_t index, bool grouped) {
    for (uint8_t i = 0; i < CS_MAX_DISPLAY_PAGES; i++) {
        const CsDisplayPage *p = &s_disp.pages[i];
        if (!(p->flags & CS_DPAGE_ACTIVE)) continue;
        if (p->noun != noun || p->target != target || p->index != index) continue;
        if (((p->flags & CS_DPAGE_GROUP) != 0) != grouped) continue;
        return i;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Public content hooks (engine-facing)
// ---------------------------------------------------------------------------

void cs_display_note_adjust(uint8_t noun, uint8_t target, uint8_t index,
                            bool grouped) {
    if (!s_attached || s_state != DSTATE_LIVE) return;
    if (noun == CS_NOUN_MACRO || noun == CS_NOUN_DISPLAY_PAGE ||
        noun == CS_NOUN_DISPLAY_EDIT || noun == CS_NOUN_PAGE_VALUE)
        return;
    // Any adjustment counts as edit activity, even one that will not pop.
    if (s_edit && s_disp.cfg.edit_timeout)
        s_edit_until = s_ms + (uint32_t)s_disp.cfg.edit_timeout * 100u;
    int pg = disp_find_page(noun, target, index, grouped);
    if (pg < 0 && !(s_disp.cfg.flags & CS_DCFG_OVERLAY_ANY)) return;
    CsDisplayPage it = {noun, target, index,
                        (uint8_t)(CS_DPAGE_ACTIVE | (grouped ? CS_DPAGE_GROUP : 0))};
    if (pg >= 0) it = s_disp.pages[pg];
    disp_pop(&it);
}

uint8_t cs_display_current_page(void) {
    if (!s_attached) return 0xFF;
    return disp_page_valid(s_cur_page) ? s_cur_page : 0xFF;
}

uint16_t cs_display_page_mask(void) {
    uint16_t m = 0;
    for (uint8_t i = 0; i < CS_MAX_DISPLAY_PAGES; i++)
        if (s_disp.pages[i].flags & CS_DPAGE_ACTIVE) m |= (uint16_t)(1u << i);
    return m;
}

void cs_display_select_page(uint8_t idx) {
    if (!s_attached || !disp_page_valid(idx)) return;
    s_cur_page = idx;
    // Manual navigation dismisses the overlay so the user sees the page.
    s_ov_active = false;
    s_cycle_at = s_ms + (uint32_t)s_disp.cfg.dwell * 100u;
    s_render_now = true;
}

bool cs_display_edit_armed(void) { return s_attached && s_edit; }

void cs_display_set_edit(bool on) {
    if (!s_attached || s_state != DSTATE_LIVE) return;
    s_edit = on;
    if (on && s_disp.cfg.edit_timeout)
        s_edit_until = s_ms + (uint32_t)s_disp.cfg.edit_timeout * 100u;
    s_render_now = true;
}

bool cs_display_resolve_current(CsDisplayPage *out) {
    if (!s_attached || s_state != DSTATE_LIVE) return false;
    return disp_current_item(out);
}

// ---------------------------------------------------------------------------
// Apply / accessors (host-facing, via control_surfaces.h)
// ---------------------------------------------------------------------------

static uint8_t disp_validate_page(const CsDisplayPage *p) {
    if (!(p->flags & CS_DPAGE_ACTIVE)) {
        const uint8_t *b = (const uint8_t *)p;
        for (size_t i = 0; i < sizeof(*p); i++)
            if (b[i] != 0) return CS_STATUS_INVALID_PAGE;
        return PIN_CONFIG_SUCCESS;
    }
    if (p->flags & (uint8_t)~(CS_DPAGE_ACTIVE | CS_DPAGE_GROUP |
                              CS_DPAGE_LARGE | CS_DPAGE_BAR))
        return CS_STATUS_INVALID_PAGE;
    if (p->noun >= CS_NOUN_COUNT) return CS_STATUS_INVALID_NOUN;
    // A bar plots a position within a range; bools and enums have none.
    if (p->flags & CS_DPAGE_BAR) {
        float lo, hi;
        if (!cs_noun_span(p->noun, &lo, &hi)) return CS_STATUS_INVALID_PAGE;
    }
    if (p->noun == CS_NOUN_DISPLAY_PAGE || p->noun == CS_NOUN_DISPLAY_EDIT ||
        p->noun == CS_NOUN_PAGE_VALUE)
        return CS_STATUS_INVALID_PAGE;
    const CsNounDesc *nd = &cs_noun_table[p->noun];
    if (nd->actions == 0) return CS_STATUS_INVALID_NOUN;   // platform-unavailable
    if (p->flags & CS_DPAGE_GROUP) {
        CsBinding v;
        memset(&v, 0, sizeof(v));
        v.noun = p->noun;
        v.flags = CS_FLAG_GROUP;
        v.target = p->target;
        v.index = p->index;
        return cs_validate_grouped_target(&v);
    }
    if (nd->target_kind == CS_TARGET_NONE)
        return (p->target || p->index) ? CS_STATUS_INVALID_TARGET
                                       : PIN_CONFIG_SUCCESS;
    return cs_noun_validate_target_ch(p->noun, p->target, p->index);
}

uint8_t control_surfaces_apply_display_cfg(const CsDisplayCfg *c) {
    if (!c) return CS_STATUS_INVALID_VALUE;
    if (c->mode > CS_DMODE_CYCLE_ALL) return CS_STATUS_INVALID_VALUE;
    if (c->home_page >= CS_MAX_DISPLAY_PAGES) return CS_STATUS_INVALID_PAGE;
    if (c->flags & (uint8_t)~(CS_DCFG_OVERLAY_ANY | CS_DCFG_EDIT_GATED |
                              CS_DCFG_LABEL_ALIGN | CS_DCFG_VALUE_ALIGN))
        return CS_STATUS_INVALID_VALUE;
    if (((c->flags & CS_DCFG_LABEL_ALIGN) >> CS_DCFG_LABEL_ALIGN_SHIFT)
            > CS_DALIGN_RIGHT ||
        ((c->flags & CS_DCFG_VALUE_ALIGN) >> CS_DCFG_VALUE_ALIGN_SHIFT)
            > CS_DALIGN_RIGHT)
        return CS_STATUS_INVALID_VALUE;
    if (c->reserved[0] || c->reserved[1]) return CS_STATUS_INVALID_VALUE;
    if (c->mode != CS_DMODE_FIXED && c->dwell < 10)
        return CS_STATUS_INVALID_VALUE;
    s_disp.cfg = *c;
    if (s_disp.cfg.mode == CS_DMODE_FIXED) s_cur_page = s_disp.cfg.home_page;
    s_cycle_at = s_ms + (uint32_t)s_disp.cfg.dwell * 100u;
    s_render_now = true;
    return PIN_CONFIG_SUCCESS;
}

uint8_t control_surfaces_apply_display_page(uint8_t idx, const CsDisplayPage *p) {
    if (idx >= CS_MAX_DISPLAY_PAGES || !p) return CS_STATUS_INVALID_PAGE;
    uint8_t st = disp_validate_page(p);
    if (st != PIN_CONFIG_SUCCESS) return st;
    s_disp.pages[idx] = *p;
    bool ok;
    s_poll_cache[idx] = (p->flags & CS_DPAGE_ACTIVE) ? disp_item_get(p, &ok) : 0.0f;
    s_render_now = true;
    return PIN_CONFIG_SUCCESS;
}

const CsDisplayFlash *control_surfaces_display_flash(void) { return &s_disp; }

const CsDisplayPage *control_surfaces_get_display_page(uint8_t idx) {
    return (idx < CS_MAX_DISPLAY_PAGES) ? &s_disp.pages[idx] : NULL;
}

void control_surfaces_get_display_status(CsDisplayStatus *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->init_state = s_state;
    out->current_page = cs_display_current_page();
    out->flags = (uint8_t)((s_ov_active ? 1 : 0) | (s_edit ? 2 : 0));
    out->model = s_attached ? s_model : 0;
    out->nak_count = s_nak;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// (Re)build the I2C peripheral: init resets the block (flushing a stuck
// FIFO after an abort-less wedge) and the enable dance sets the fixed TAR.
static void cs_display_periph_up(void) {
    i2c_init(s_i2c, s_baud);
    i2c_hw_t *hw = i2c_get_hw(s_i2c);
    hw->enable = 0;
    hw->tar = s_addr;
    hw->enable = 1;
}

void cs_display_reset(void) {
    memset(&s_disp, 0, sizeof(s_disp));
    s_disp.version = CS_DISPLAY_CONFIG_VERSION;
    s_cur_page = 0xFF;
    s_ov_active = false;
    s_edit = false;
    s_render_now = true;
    memset(s_poll_cache, 0, sizeof(s_poll_cache));
}

void cs_display_load_stored(void) {
    preset_get_cs_display(&s_disp);
    if (s_disp.version > CS_DISPLAY_CONFIG_VERSION) {
        memset(&s_disp, 0, sizeof(s_disp));   // future format stays idle
    }
    s_disp.version = CS_DISPLAY_CONFIG_VERSION;
    if (s_disp.cfg.mode > CS_DMODE_CYCLE_ALL) s_disp.cfg.mode = CS_DMODE_FIXED;
    if (s_disp.cfg.home_page >= CS_MAX_DISPLAY_PAGES) s_disp.cfg.home_page = 0;
    s_disp.cfg.flags &= (CS_DCFG_OVERLAY_ANY | CS_DCFG_EDIT_GATED |
                         CS_DCFG_LABEL_ALIGN | CS_DCFG_VALUE_ALIGN);
    if (s_disp.cfg.mode != CS_DMODE_FIXED && s_disp.cfg.dwell < 10)
        s_disp.cfg.dwell = 10;
    // Full noun/target validation happens HERE, not in the flash sanitizer
    // (it is platform-dependent): a blob written by a different build must
    // never hand the render or PAGE_VALUE paths an out-of-range index.
    for (uint8_t i = 0; i < CS_MAX_DISPLAY_PAGES; i++) {
        if (!(s_disp.pages[i].flags & CS_DPAGE_ACTIVE)) continue;
        if (disp_validate_page(&s_disp.pages[i]) != PIN_CONFIG_SUCCESS)
            memset(&s_disp.pages[i], 0, sizeof(s_disp.pages[i]));
    }
    s_cur_page = s_disp.cfg.home_page;
}

// Seed the default pages the first time a display comes up on an empty
// table (never at migration, so a bare upgrade stays all-zero).
static void disp_seed_pages(void) {
    if (cs_display_page_mask() != 0) return;
    static const CsDisplayPage seed[4] = {
        {CS_NOUN_USER_VOLUME,  0, 0, CS_DPAGE_ACTIVE | CS_DPAGE_LARGE},
        {CS_NOUN_PRESET,       0, 0, CS_DPAGE_ACTIVE},
        {CS_NOUN_INPUT_SOURCE, 0, 0, CS_DPAGE_ACTIVE},
        {CS_NOUN_SAMPLE_RATE,  0, 0, CS_DPAGE_ACTIVE},
    };
    for (uint8_t i = 0; i < 4; i++) s_disp.pages[i] = seed[i];
    if (s_disp.cfg.overlay_hold == 0) s_disp.cfg.overlay_hold = 20;  // 2 s
    if (s_disp.cfg.dwell < 10) s_disp.cfg.dwell = 30;                // 3 s
    if (s_disp.cfg.edit_timeout == 0) s_disp.cfg.edit_timeout = 100; // 10 s
    // Arm-before-edit by default, so a knock on the value control browses
    // rather than moving a parameter.  It needs a DISPLAY_EDIT control to
    // arm with; a host wanting direct adjust clears the bit.
    s_disp.cfg.flags |= CS_DCFG_EDIT_GATED;
    // No dirty mark: seeding is deterministic, so boot and revert reproduce
    // the same live state without flagging an unsaved preview forever.
}

void cs_display_attach(const CsBinding *b) {
    const DispModelDesc *m = &s_models[b->index];
    s_model = b->index;
    s_addr = (b->value != 0) ? (uint8_t)b->value : m->def_addr;
    s_baud = m->baud;
    s_i2c = ((b->gpio[0] >> 1) & 1) ? i2c1 : i2c0;
    cs_display_periph_up();
    gpio_set_function(b->gpio[0], GPIO_FUNC_I2C);
    gpio_set_function(b->gpio[1], GPIO_FUNC_I2C);
    gpio_pull_up(b->gpio[0]);
    gpio_pull_up(b->gpio[1]);
    s_txq_head = s_txq_tail = 0;
    s_delay_until = 0;
    s_init_pos = 0;
    s_init_done = false;
    s_row_cur = 0xFF;
    s_dirty = 0;
    s_nak = 0;
    memset(s_label_out, 0, sizeof(s_label_out));
    memset(s_value_out, 0, sizeof(s_value_out));
    memset(s_bar_out, 0, sizeof(s_bar_out));
    s_bar_px = 0;
    s_view_bar = false;
    s_state = DSTATE_INIT;
    s_attached = true;
    disp_seed_pages();
    s_cur_page = s_disp.cfg.home_page;
    s_render_now = true;
}

void cs_display_detach(void) {
    if (!s_attached) return;
    i2c_deinit(s_i2c);
    s_attached = false;
    s_state = DSTATE_DOWN;
    s_edit = false;
    s_ov_active = false;
}

bool cs_display_live(void) { return s_attached; }

int cs_display_live_instance(void) {
    if (!s_attached) return -1;
    return (s_i2c == i2c1) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void cs_display_tick(void) {
    if (!s_attached) return;
    s_ms++;

    if (s_state == DSTATE_ERROR) {
        if ((int32_t)(s_ms - s_retry_at) < 0) return;
        // Rebuild the peripheral, not just the script: after an abort-less
        // wedge the TX FIFO is still full and only a block reset clears it.
        cs_display_periph_up();
        s_state = DSTATE_INIT;
    }

    // Producer: init script first, then dirty rows.
    if (s_state == DSTATE_INIT) {
        while (!s_init_done && disp_init_step()) {}
        if (s_init_done && txq_empty() && !s_delay_until) {
            s_state = DSTATE_LIVE;
            s_dirty = (uint8_t)((1u << s_models[s_model].rows) - 1);
            s_render_now = true;
            // Seed the change-poll baseline from live values, or the first
            // sweep would pop an overlay for every page whose value is not
            // zero (a spurious cascade at boot / re-attach / revert).
            for (uint8_t i = 0; i < CS_MAX_DISPLAY_PAGES; i++) {
                if (!(s_disp.pages[i].flags & CS_DPAGE_ACTIVE)) continue;
                bool ok;
                s_poll_cache[i] = disp_item_get(&s_disp.pages[i], &ok);
            }
        }
    } else if (s_state == DSTATE_LIVE) {
        if (s_row_cur == 0xFF && s_dirty) {
            s_row_cur = (uint8_t)__builtin_ctz(s_dirty);
            s_row_phase = 0;
            s_row_col = 0;
        }
        if (s_row_cur != 0xFF) {
            while (true) {
                bool done = disp_row_step();
                if (done) {
                    s_dirty &= (uint8_t)~(1u << s_row_cur);
                    s_row_cur = 0xFF;
                    break;
                }
                if (txq_free() < 8) break;
            }
        }
    }
    disp_pump();
    if (s_state != DSTATE_LIVE) return;

    // Timers
    if (s_ov_active && (int32_t)(s_ms - s_ov_until) >= 0) {
        s_ov_active = false;
        s_render_now = true;
    }
    if (s_edit && s_disp.cfg.edit_timeout &&
        (int32_t)(s_ms - s_edit_until) >= 0) {
        s_edit = false;
        s_render_now = true;
    }
    if (!s_ov_active && s_disp.cfg.mode != CS_DMODE_FIXED &&
        (int32_t)(s_ms - s_cycle_at) >= 0) {
        s_cycle_at = s_ms + (uint32_t)s_disp.cfg.dwell * 100u;
        if (s_disp.cfg.mode == CS_DMODE_CYCLE_SELECTED) {
            uint16_t mask = cs_display_page_mask();
            if (mask) {
                uint8_t n = s_cur_page;
                for (uint8_t i = 0; i < CS_MAX_DISPLAY_PAGES; i++) {
                    n = (uint8_t)((n + 1) % CS_MAX_DISPLAY_PAGES);
                    if (mask & (1u << n)) break;
                }
                s_cur_page = n;
            }
        } else {
            // CYCLE_ALL: untargeted, platform-available nouns only.
            uint8_t n = s_cycle_noun;
            for (uint8_t i = 0; i < CS_NOUN_COUNT; i++) {
                n = (uint8_t)((n + 1) % CS_NOUN_COUNT);
                const CsNounDesc *nd = &cs_noun_table[n];
                if (nd->actions == 0 || nd->target_kind != CS_TARGET_NONE) continue;
                if (n == CS_NOUN_DISPLAY_PAGE || n == CS_NOUN_DISPLAY_EDIT ||
                    n == CS_NOUN_PAGE_VALUE || n == CS_NOUN_MACRO) continue;
                break;
            }
            s_cycle_noun = n;
        }
        s_render_now = true;
    }

    // Host-change poll: pop configured pages whose item moved without a CS
    // control doing it.  Writable items only; meters churn constantly.
    if ((s_ms % DISP_POLL_DECIM) == 0) {
        for (uint8_t i = 0; i < CS_MAX_DISPLAY_PAGES; i++) {
            const CsDisplayPage *p = &s_disp.pages[i];
            if (!(p->flags & CS_DPAGE_ACTIVE)) continue;
            const CsNounDesc *nd = &cs_noun_table[p->noun];
            if (disp_item_readonly(nd)) continue;
            bool ok;
            float v = disp_item_get(p, &ok);
            if (!ok) continue;
            if (fabsf(v - s_poll_cache[i]) > 0.05f) {
                s_poll_cache[i] = v;
                disp_pop(p);
            }
        }
    }

    // Render only while no row is mid-transmission: swapping the line
    // buffers under an in-flight row would tear it (half old glyphs, half
    // new) and its completion would clear the fresh dirty bit.
    if ((s_render_now || (s_ms % DISP_RENDER_DECIM) == 0) &&
        s_row_cur == 0xFF) {
        s_render_now = false;
        disp_render();
    }
}
