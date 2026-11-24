/*
 * ui_button.h; one-button gesture detector: short press and long press.
 *
 * A short press is reported on release. A long press is reported the moment
 * the hold reaches UI_BTN_LONG_MS (so the user gets feedback while still
 * holding); the release that follows is swallowed. Pure logic, host-testable.
 */
#ifndef UI_BUTTON_H
#define UI_BUTTON_H

#include <stdint.h>
#include <stdbool.h>

#define UI_BTN_DEBOUNCE_MS  25u
#define UI_BTN_LONG_MS      600u
#define UI_BTN_SHOW_MS      150u     // hold progress starts showing after this

typedef enum { UI_BTN_NONE = 0, UI_BTN_SHORT, UI_BTN_LONG } UiBtnEvent;

typedef struct {
    bool     raw_prev;
    bool     down;           // debounced state
    bool     long_fired;
    uint32_t change_ms;      // when raw last changed
    uint32_t press_ms;       // when the debounced press began
} UiBtn;

void       ui_btn_init(UiBtn *b);
UiBtnEvent ui_btn_update(UiBtn *b, bool raw_pressed, uint32_t now_ms);
uint8_t    ui_btn_hold_pct(const UiBtn *b, uint32_t now_ms);   // 0..100 toward long press; 0 if idle
bool       ui_btn_is_down(const UiBtn *b);

#endif
