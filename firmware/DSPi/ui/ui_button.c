#include "ui_button.h"

void ui_btn_init(UiBtn *b) {
    b->raw_prev = false;
    b->down = false;
    b->long_fired = false;
    b->change_ms = 0;
    b->press_ms = 0;
}

UiBtnEvent ui_btn_update(UiBtn *b, bool raw, uint32_t now) {
    if (raw != b->raw_prev) {
        b->raw_prev = raw;
        b->change_ms = now;
    }
    // Debounce: the raw level must have been stable long enough.
    if (raw != b->down && (uint32_t)(now - b->change_ms) >= UI_BTN_DEBOUNCE_MS) {
        b->down = raw;
        if (raw) {
            b->press_ms = b->change_ms;
            b->long_fired = false;
        } else {
            bool was_long = b->long_fired;
            b->long_fired = false;
            return was_long ? UI_BTN_NONE : UI_BTN_SHORT;
        }
    }
    if (b->down && !b->long_fired && (uint32_t)(now - b->press_ms) >= UI_BTN_LONG_MS) {
        b->long_fired = true;
        return UI_BTN_LONG;
    }
    return UI_BTN_NONE;
}

uint8_t ui_btn_hold_pct(const UiBtn *b, uint32_t now) {
    if (!b->down || b->long_fired) return 0;
    uint32_t held = now - b->press_ms;
    if (held < UI_BTN_SHOW_MS) return 0;
    uint32_t pct = held * 100u / UI_BTN_LONG_MS;
    return (uint8_t)(pct > 100u ? 100u : (pct ? pct : 1u));
}

bool ui_btn_is_down(const UiBtn *b) { return b->down; }
