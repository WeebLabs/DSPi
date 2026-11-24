/*
 * ui_nav.h; menu navigation with a single button.
 *
 *   Top level:  short = next page      long = open the page's menu
 *                                             (on NOW: toggle mute)
 *   In a menu:  short = next item      long = activate the item
 *                                             (the last item is always "back")
 */
#ifndef UI_NAV_H
#define UI_NAV_H

#include "ui_types.h"
#include "ui_button.h"

typedef enum {
    UA_NONE,
    UA_TOGGLE_MUTE,
    UA_TOGGLE_FX,      // arg = fx index
    UA_TOGGLE_BAND,    // arg = listed band index (0..eq.nb-1)
    UA_LOAD_PRESET,    // arg = slot
    UA_SAVE_PRESET,
    UA_CLEAR_CLIP,
    UA_CYCLE_DISP,
    UA_CYCLE_LED,
} UiActionType;

typedef struct { UiActionType type; uint8_t arg; } UiAction;

typedef struct {
    UiPage  page;
    bool    in_menu;
    uint8_t cursor;
} UiNav;

// Menu row counts, including the trailing "back" row. 0 = page has no menu.
#define UI_MENU_FX_ROWS      (UI_FX_COUNT + 1)
#define UI_MENU_PRESET_ROWS  (UI_PRESETS + 2)      // presets, save, back
#define UI_MENU_SYS_ROWS     4                     // bright, led, clip, back

uint8_t  ui_menu_len(UiPage page, uint8_t eq_nb);
UiAction ui_nav_event(UiNav *n, UiBtnEvent ev, uint8_t eq_nb, uint8_t cur_preset);
void     ui_nav_clamp(UiNav *n, uint8_t eq_nb);

#endif
