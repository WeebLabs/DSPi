/*
 * ac3_config.c; see ac3_config.h.
 */

#include "ac3_config.h"

Ac3Config ac3_config = { 1, AC3_DRC_FULL, AC3_DIALNORM_APPLY };

void ac3_config_defaults(Ac3Config *c) {
    c->enabled = 1;
    c->drc_mode = AC3_DRC_FULL;
    c->dialnorm_mode = AC3_DIALNORM_APPLY;
}

bool ac3_config_sanitize(Ac3Config *c) {
    bool changed = false;
    if (c->enabled > 1) { c->enabled = 1; changed = true; }
    if (c->drc_mode > AC3_DRC_MODE_MAX) { c->drc_mode = AC3_DRC_FULL; changed = true; }
    if (c->dialnorm_mode > AC3_DIALNORM_MODE_MAX) { c->dialnorm_mode = AC3_DIALNORM_APPLY; changed = true; }
    return changed;
}
