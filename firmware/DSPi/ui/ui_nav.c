#include "ui_nav.h"

uint8_t ui_menu_len(UiPage page, uint8_t eq_nb) {
    switch (page) {
    case UI_PG_EQ:     return (uint8_t)(eq_nb + 1);
    case UI_PG_FX:     return UI_MENU_FX_ROWS;
    case UI_PG_PRESET: return UI_MENU_PRESET_ROWS;
    case UI_PG_SYS:    return UI_MENU_SYS_ROWS;
    default:           return 0;
    }
}

void ui_nav_clamp(UiNav *n, uint8_t eq_nb) {
    uint8_t len = ui_menu_len(n->page, eq_nb);
    if (!len) { n->in_menu = false; n->cursor = 0; return; }
    if (n->cursor >= len) n->cursor = (uint8_t)(len - 1);
}

static UiAction act(UiActionType t, uint8_t a) { UiAction r = { t, a }; return r; }

UiAction ui_nav_event(UiNav *n, UiBtnEvent ev, uint8_t eq_nb, uint8_t cur_preset) {
    uint8_t len = ui_menu_len(n->page, eq_nb);

    if (ev == UI_BTN_SHORT) {
        if (!n->in_menu) {
            n->page = (UiPage)((n->page + 1) % UI_PG_COUNT);
            n->cursor = 0;
        } else {
            n->cursor = (uint8_t)((n->cursor + 1) % len);
        }
        return act(UA_NONE, 0);
    }

    if (ev != UI_BTN_LONG) return act(UA_NONE, 0);

    if (!n->in_menu) {
        if (len == 0) return act(UA_TOGGLE_MUTE, 0);        // NOW page
        n->in_menu = true;
        n->cursor = (n->page == UI_PG_PRESET && cur_preset < UI_PRESETS) ? cur_preset : 0;
        return act(UA_NONE, 0);
    }

    uint8_t row = n->cursor;
    if (row == len - 1) {                                    // "back"
        n->in_menu = false;
        n->cursor = 0;
        return act(UA_NONE, 0);
    }
    switch (n->page) {
    case UI_PG_EQ:     return act(UA_TOGGLE_BAND, row);
    case UI_PG_FX:     return act(UA_TOGGLE_FX, row);
    case UI_PG_PRESET: return row < UI_PRESETS ? act(UA_LOAD_PRESET, row) : act(UA_SAVE_PRESET, 0);
    case UI_PG_SYS:    return row == 0 ? act(UA_CYCLE_DISP, 0)
                            : row == 1 ? act(UA_CYCLE_LED, 0) : act(UA_CLEAR_CLIP, 0);
    default:           return act(UA_NONE, 0);
    }
}
