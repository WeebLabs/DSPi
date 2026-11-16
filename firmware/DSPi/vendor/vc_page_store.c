/*
 * vc_page_store.c: STORE page (0x02). Command set: vendor_command_overhaul_plan.md
 * section 5.2. Every write is queued for the main loop through the same pending
 * flags the legacy opcodes use; bulk transfers run under the vc_bulk.c lock.
 */

#include "vc_core.h"
#include "vc_internal.h"
#include "config.h"
#include "usb_audio.h"
#include "flash_storage.h"
#include "bulk_params.h"
#include "hardware/sync.h"

_Static_assert(sizeof(WireBulkParams) == 6136, "STORE.ALL_PARAMS GET_LEN in pages/store.def");

static VcResult store_preset_save_set(VcCtx *c) {
    pending_preset_save_slot = (uint8_t)c->addr;
    preset_save_pending = true;
    __dmb();
    return VC_OK;
}

static VcResult store_preset_load_set(VcCtx *c) {
    pending_preset_load_slot = (uint8_t)c->addr;
    preset_load_pending = true;
    __dmb();
    return VC_OK;
}

static VcResult store_preset_delete_set(VcCtx *c) {
    preset_delete_mask |= (uint16_t)(1u << c->addr);
    __dmb();
    return VC_OK;
}

static VcResult store_preset_name_get(VcCtx *c) {
    char name[PRESET_NAME_LEN];
    if (preset_get_name((uint8_t)c->addr, name) != PRESET_OK) return VC_ERR_ADDRESS;
    memcpy(c->out, name, PRESET_NAME_LEN);
    return VC_OK;
}

static VcResult store_preset_name_set(VcCtx *c) {
    memset(flash_set_name_buf, 0, PRESET_NAME_LEN);
    size_t copy_len = c->in_len < (PRESET_NAME_LEN - 1) ? c->in_len : (PRESET_NAME_LEN - 1);
    memcpy(flash_set_name_buf, c->in, copy_len);
    flash_set_name_slot = (uint8_t)c->addr;
    __dmb();
    flash_set_name_pending = true;
    return VC_OK;
}

static VcResult store_preset_active_get(VcCtx *c) {
    c->out[0] = preset_get_active();
    return VC_OK;
}

static VcResult store_directory_get(VcCtx *c) {
    uint16_t occupied;
    uint8_t startup, def_slot, last_active, oc_mode, mv_mode;
    preset_get_directory(&occupied, &startup, &def_slot, &last_active, &oc_mode, &mv_mode);
    vc_out_u16(c, 0, occupied);
    c->out[2] = startup;
    c->out[3] = def_slot;
    c->out[4] = last_active;
    c->out[5] = oc_mode;
    c->out[6] = mv_mode;
    return VC_OK;
}

static VcResult store_startup_get(VcCtx *c) {
    uint16_t occupied;
    uint8_t startup, def_slot, last_active, oc_mode, mv_mode;
    preset_get_directory(&occupied, &startup, &def_slot, &last_active, &oc_mode, &mv_mode);
    c->out[0] = startup;
    c->out[1] = def_slot;
    return VC_OK;
}

// preset_set_startup() on the main loop refuses the same values.
static VcResult store_startup_set(VcCtx *c) {
    uint8_t mode = vc_in_u8(c, 0), slot = vc_in_u8(c, 1);
    if (mode > PRESET_STARTUP_LAST_ACTIVE || slot >= PRESET_SLOTS) return VC_ERR_VALUE;
    flash_set_startup_mode = mode;
    flash_set_startup_slot = slot;
    __dmb();
    flash_set_startup_pending = true;
    return VC_OK;
}

static VcResult store_output_config_mode_get(VcCtx *c) {
    uint16_t occupied;
    uint8_t startup, def_slot, last_active, oc_mode, mv_mode;
    preset_get_directory(&occupied, &startup, &def_slot, &last_active, &oc_mode, &mv_mode);
    c->out[0] = oc_mode;
    return VC_OK;
}

static VcResult store_output_config_mode_set(VcCtx *c) {
    uint8_t m = vc_in_u8(c, 0);
    if (m > OUTPUT_CONFIG_MODE_WITH_PRESET) m = OUTPUT_CONFIG_MODE_INDEPENDENT;
    flash_set_output_config_mode_val = m;
    __dmb();
    flash_set_output_config_mode_pending = true;
    return VC_OK;
}

static VcResult store_output_config_save_set(VcCtx *c) {
    (void)c;
    flash_save_output_config_pending = true;
    __dmb();
    return VC_OK;
}

static VcResult store_factory_reset_set(VcCtx *c) {
    (void)c;
    factory_reset_pending = true;
    __dmb();
    return VC_OK;
}

// Reached only through the bulk paths (USB, UART, I2C), with the buffer under the bulk lock.
static VcResult store_all_params_get(VcCtx *c) {
    if (c->out != vc_bulk_buf()) return VC_ERR_BLOCKED;
    bulk_params_collect((WireBulkParams *)c->out);
    return VC_OK;
}

static VcResult store_all_params_set(VcCtx *c) {
    if (c->in != vc_bulk_buf()) return VC_ERR_BLOCKED;
    if (c->in_len != sizeof(WireBulkParams)) return VC_ERR_LENGTH;
    VcResult r = vc_bulk_params_check(c->in);
    if (r != VC_OK) return r;
    bulk_params_pending = true;   // guards bulk_param_buf from here; the lock drops at ACK
    return VC_OK;
}

static VcResult store_all_params_chunk_get(VcCtx *c) { return vc_bulk_chunk_get(c); }
static VcResult store_all_params_chunk_set(VcCtx *c) { return vc_bulk_chunk_set(c); }

#define VC_PAGE_DEF "pages/store.def"
#include "vc_page_table.inc"
