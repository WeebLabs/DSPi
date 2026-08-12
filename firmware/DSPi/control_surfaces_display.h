/*
 * control_surfaces_display.h; internal interface between the Control
 * Surfaces engine and the I2C display module (caps v10).
 *
 * The module owns the live display config/pages blob, the content logic
 * (home pages, event overlay, edit mode) and the per-model I2C driver
 * state machines.  Everything runs on the core0 main loop via the shared
 * 1 kHz tick; nothing here may block (see the feature spec, s6).
 * The host-facing apply/accessor API lives in control_surfaces.h.
 *
 * See Documentation/Features/control_surfaces_display_spec.md.
 */

#ifndef CONTROL_SURFACES_DISPLAY_H
#define CONTROL_SURFACES_DISPLAY_H

#include "control_surfaces.h"

// Component lifecycle, called from the engine's claim/release paths.  The
// engine has already validated pins/model/address and set the GPIO mux;
// attach configures the I2C peripheral and starts the init script.
void cs_display_attach(const CsBinding *b);
void cs_display_detach(void);
bool cs_display_live(void);          // component attached (may be initializing)
int  cs_display_live_instance(void); // I2C instance held (0/1), -1 when down;
                                     // the control target must avoid it

// 1 kHz tick: byte-budget I2C pump, timers, decimated render/poll.
void cs_display_tick(void);

// Reset module state and load the stored blob; boot init and revert.
void cs_display_reset(void);
void cs_display_load_stored(void);

// Event hook from the engine's op initiators: the item a control just
// adjusted (grouped = target is a group index).  Drives the overlay.
void cs_display_note_adjust(uint8_t noun, uint8_t target, uint8_t index,
                            bool grouped);

// Page / edit state for the DISPLAY_PAGE, DISPLAY_EDIT and PAGE_VALUE
// nouns.  All are no-ops / idle reads without a live component.
uint8_t  cs_display_current_page(void);      // 0xFF = none / synthesized
uint16_t cs_display_page_mask(void);         // bit N = page N active
void     cs_display_select_page(uint8_t idx);
bool     cs_display_edit_armed(void);
void     cs_display_set_edit(bool on);

// The item currently rendered (page or synthesized overlay), for
// PAGE_VALUE resolution.  False when nothing editable is shown.
bool cs_display_resolve_current(CsDisplayPage *out);

#endif // CONTROL_SURFACES_DISPLAY_H
