/*
 * input_source_names.c; user names for input sources
 *
 * See input_source_names.h and Documentation/Features/input_source_names_spec.md.
 */

#include "input_source_names.h"
#include "flash_storage.h"

#include "hardware/sync.h"

#include <stdio.h>
#include <string.h>

// One staging buffer per source so back-to-back renames never overwrite each
// other.  USB stages from IRQ context and UART/I2C from the main loop, so every
// access runs with interrupts disabled.
static char              s_staged[INPUT_SOURCE_NAME_SLOTS][INPUT_SOURCE_NAME_LEN];
static uint8_t           s_staged_tag[INPUT_SOURCE_NAME_SLOTS];
static uint8_t           s_staged_gen[INPUT_SOURCE_NAME_SLOTS];
static volatile uint8_t  s_staged_mask;

bool input_source_name_stage(uint8_t src, const uint8_t *data, uint16_t len,
                             ParamSource tag) {
    if (!input_source_valid(src)) return false;
    uint32_t f = save_and_disable_interrupts();
    memset(s_staged[src], 0, INPUT_SOURCE_NAME_LEN);
    if (len > INPUT_SOURCE_NAME_LEN - 1) len = INPUT_SOURCE_NAME_LEN - 1;
    if (len && data[0] != '\0') {
        memcpy(s_staged[src], data, len);
    }
    s_staged_tag[src] = (uint8_t)tag;
    s_staged_gen[src]++;
    s_staged_mask |= (uint8_t)(1u << src);
    restore_interrupts(f);
    return true;
}

bool input_source_name_get(uint8_t src, char out[INPUT_SOURCE_NAME_LEN]) {
    if (!input_source_valid(src)) return false;
    // A staged name stays staged until the directory cache holds it, so this
    // never reads a cache entry mid-update.
    uint32_t f = save_and_disable_interrupts();
    bool staged = (s_staged_mask & (1u << src)) != 0;
    if (staged) memcpy(out, s_staged[src], INPUT_SOURCE_NAME_LEN);
    restore_interrupts(f);
    if (!staged) memcpy(out, preset_stored_input_source_name(src), INPUT_SOURCE_NAME_LEN);
    out[INPUT_SOURCE_NAME_LEN - 1] = '\0';
    return true;
}

bool input_source_names_pending(void) {
    return s_staged_mask != 0;
}

void input_source_names_apply_pending(void) {
    char    names[INPUT_SOURCE_NAME_SLOTS][INPUT_SOURCE_NAME_LEN];
    uint8_t tags[INPUT_SOURCE_NAME_SLOTS];
    uint8_t gens[INPUT_SOURCE_NAME_SLOTS];

    uint32_t f = save_and_disable_interrupts();
    uint8_t mask = s_staged_mask;
    memcpy(names, s_staged, sizeof(names));
    memcpy(tags, s_staged_tag, sizeof(tags));
    memcpy(gens, s_staged_gen, sizeof(gens));
    restore_interrupts(f);
    if (!mask) return;

    if (preset_set_input_source_names(mask, names) != PRESET_OK) {
        printf("input source names: directory write failed\n");
    }

    // A source restaged while the write ran keeps its bit for the next pass.
    f = save_and_disable_interrupts();
    for (uint8_t s = 0; s < INPUT_SOURCE_NAME_SLOTS; s++) {
        if ((mask & (1u << s)) && s_staged_gen[s] == gens[s])
            s_staged_mask &= (uint8_t)~(1u << s);
    }
    restore_interrupts(f);

    for (uint8_t s = 0; s < INPUT_SOURCE_NAME_SLOTS; s++) {
        if (mask & (1u << s))
            notify_push_input_source_name(s, names[s], (ParamSource)tags[s]);
    }
}
