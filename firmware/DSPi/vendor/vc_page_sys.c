/*
 * vc_page_sys.c: SYS page (0x01). Discovery, identity and bootloader.
 * PLATFORM, SERIAL, BUILD_INFO and CORE1_MODE match the legacy opcodes byte for byte.
 */

#include "vc_core.h"
#include "vc_internal.h"
#include "config.h"
#include "build_info.h"
#include "usb_descriptors.h"
#include "pdm_generator.h"
#include "pico/bootrom.h"

static uint8_t sys_implemented_pages(void) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < vc_page_dir_len; i++)
        if (vc_page_dir[i]->count) n++;
    return n;
}

static VcResult sys_protocol_info_get(VcCtx *c) {
    memcpy(c->out, "DSPi", 4);
    vc_out_u8(c, 4, VC_PROTOCOL_MAJOR);
    vc_out_u8(c, 5, VC_PROTOCOL_MINOR);
    vc_out_u8(c, 6, sys_implemented_pages());
    vc_out_u8(c, 7, 0);
    vc_out_u16(c, 8, VC_SET_MAX);
    vc_out_u16(c, 10, VC_REPLY_MAX);
    return VC_OK;
}

static VcResult sys_page_list_get(VcCtx *c) {
    uint16_t n = 0;
    for (uint16_t p = 1; p <= 0xFF && n + 2 <= c->out_cap; p++) {
        const VcPage *pg = vc_find_page((uint8_t)p);
        if (!pg) continue;
        c->out[n++] = pg->page;
        c->out[n++] = pg->revision;
    }
    c->out_len = n;
    return VC_OK;
}

static VcResult sys_page_commands_get(VcCtx *c) {
    const VcPage *pg = vc_find_page((uint8_t)c->addr);
    if (!pg) return VC_ERR_ADDRESS;
    for (uint8_t i = 0; i < pg->count; i++) {
        uint8_t n = pg->cmds[i].cmd;
        if (pg->cmds[i].get) c->out[n >> 3]        |= (uint8_t)(1u << (n & 7));
        if (pg->cmds[i].set) c->out[32 + (n >> 3)] |= (uint8_t)(1u << (n & 7));
    }
    return VC_OK;
}

static VcResult sys_command_info_get(VcCtx *c) {
    const VcCmd *e = vc_find_cmd(vc_find_page((uint8_t)(c->addr >> 8)), (uint8_t)c->addr);
    if (!e) return VC_ERR_ADDRESS;
    vc_out_u16(c, 0, e->flags);
    vc_out_u16(c, 2, e->get ? e->get_len : 0);
    vc_out_u8(c, 4, e->set_min);
    vc_out_u8(c, 5, e->set_max);
    vc_out_u8(c, 6, e->addr_kind);
    vc_out_u8(c, 7, (uint8_t)((e->get ? 1u : 0u) | (e->set ? 2u : 0u)));
    return VC_OK;
}

static VcResult sys_last_result_get(VcCtx *c) {
    vc_last_result(c->src, c->out);
    return VC_OK;
}

static VcResult sys_platform_get(VcCtx *c) {
    c->out[0] = PLATFORM_RP2040;
#if PICO_RP2350
    c->out[0] = PLATFORM_RP2350;
#endif
    c->out[1] = (uint8_t)(FW_VERSION_PACKED >> 8);
    c->out[2] = (uint8_t)(FW_VERSION_PACKED & 0xFF);
    c->out[3] = NUM_OUTPUT_CHANNELS;
    c->out[4] = FW_VERSION_MINOR;
    c->out[5] = FW_VERSION_PATCH;
    c->out[6] = FW_VERSION_BETA;
    return VC_OK;
}

static VcResult sys_serial_get(VcCtx *c) {
    memcpy(c->out, usb_descriptor_str_serial, 16);
    return VC_OK;
}

static VcResult sys_build_info_get(VcCtx *c) {
    strncpy((char *)c->out, BUILD_GIT_DESCRIBE, 47);
    strncpy((char *)c->out + 48, BUILD_DATE, 11);
    return VC_OK;
}

static VcResult sys_core1_mode_get(VcCtx *c) {
    c->out[0] = (uint8_t)core1_mode;
    return VC_OK;
}

// Over USB the reboot waits for the status stage (vc_usb.c). UART and I2C have
// no hook after their reply, so they reboot at once, as the legacy opcode did.
static VcResult sys_enter_bootloader_set(VcCtx *c) {
    if (c->src == CTRL_SOURCE_USB) {
        vc_usb_bootloader_after_status();
        return VC_OK;
    }
    reset_usb_boot(0, 0);
    return VC_OK;
}

#define VC_PAGE_DEF "pages/sys.def"
#include "vc_page_table.inc"
