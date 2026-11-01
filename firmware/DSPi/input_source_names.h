/*
 * input_source_names.h; user names for input sources and for the board
 *
 * Device-global, stored in the preset directory: one entry per InputSource
 * plus the board name, all staged and flushed together.  Design:
 * Documentation/Features/input_source_names_spec.md and device_name_spec.md.
 */

#ifndef INPUT_SOURCE_NAMES_H
#define INPUT_SOURCE_NAMES_H

#include <stdbool.h>
#include <stdint.h>

#include "audio_input.h"
#include "notify.h"

#define INPUT_SOURCE_NAME_LEN    32
#define DEVICE_NAME_LEN          32
// Fixed flash geometry: one spare entry beyond today's sources.
#define INPUT_SOURCE_NAME_SLOTS  8

_Static_assert(INPUT_SOURCE_MAX < INPUT_SOURCE_NAME_SLOTS,
               "input source name table must cover every source");
_Static_assert(NOTIFY_INPUT_SOURCE_NAME_LEN == INPUT_SOURCE_NAME_LEN &&
               DEVICE_NAME_LEN == INPUT_SOURCE_NAME_LEN,
               "the notify events carry a whole name entry");

// Stage a SET from vendor dispatch (any context).  A leading NUL clears the
// name.  False for an invalid source.
bool input_source_name_stage(uint8_t src, const uint8_t *data, uint16_t len,
                             ParamSource tag);

// Copy the live name (a staged one if unapplied) into out.  All zeros when
// unnamed.  False for an invalid source.
bool input_source_name_get(uint8_t src, char out[INPUT_SOURCE_NAME_LEN]);

// Board name equivalents of the two above.
void device_name_stage(const uint8_t *data, uint16_t len, ParamSource tag);
void device_name_get(char out[DEVICE_NAME_LEN]);

// True while any source or board name is staged.
bool input_source_names_pending(void);

// Persist every staged source and board name in one directory write and
// notify.  Main loop only, inside a flash write bracket.
void input_source_names_apply_pending(void);

#endif // INPUT_SOURCE_NAMES_H
