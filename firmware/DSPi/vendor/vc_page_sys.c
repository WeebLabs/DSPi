/*
 * vc_page_sys.c: SYS page (0x01). Discovery, identity, device name, bootloader
 * and telemetry. Identity and BUFFER_STATS match the legacy opcodes byte for byte.
 */

#include "vc_core.h"
#include "vc_internal.h"
#include "config.h"
#include "build_info.h"
#include "usb_descriptors.h"
#include "pdm_generator.h"
#include "pico/bootrom.h"
#include "usb_audio.h"
#include "audio_pipeline.h"
#include "bg_job.h"
#include "input_source_names.h"
#include "notify.h"
#include "pico/audio_spdif.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"

// Defined in vendor_commands.c.
extern uint16_t vreg_voltage_to_mv(enum vreg_voltage voltage);
extern int16_t read_temperature_cdeg(void);

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

static VcResult sys_device_name_get(VcCtx *c) {
    device_name_get((char *)c->out);
    return VC_OK;
}

// Staged here, written to flash by the main loop with the source names.
static VcResult sys_device_name_set(VcCtx *c) {
    device_name_stage(c->in, c->in_len, vc_param_src(c->src));
    return VC_OK;
}

static VcResult sys_system_status_get(VcCtx *c) {
    vc_out_u32(c, 0, clock_get_hz(clk_sys));
    vc_out_u32(c, 4, audio_state.freq);
    vc_out_u16(c, 8, vreg_voltage_to_mv(vreg_get_voltage()));
    vc_out_u16(c, 10, (uint16_t)read_temperature_cdeg());
    vc_out_u8(c, 12, global_status.cpu0_load);
    vc_out_u8(c, 13, global_status.cpu1_load);
    return VC_OK;
}

static VcResult sys_usb_audio_status_get(VcCtx *c) {
    vc_out_u32(c, 0, usb_audio_packets);
    vc_out_u32(c, 4, usb_audio_ring_overrun_count());
    vc_out_u32(c, 8, usb_audio_worst_packet_gap_us);
    vc_out_u8(c, 12, (uint8_t)usb_audio_alt_set);
    vc_out_u8(c, 13, (uint8_t)usb_audio_mounted);
    return VC_OK;
}

static VcResult sys_buffer_stats_get(VcCtx *c) {
    BufferStatsPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.num_spdif = NUM_SPDIF_INSTANCES;
    pkt.flags = (pdm_enabled ? 0x01 : 0) | (sync_started ? 0x02 : 0);
    pkt.sequence = buffer_stats_sequence++;

    uint consumer_capacity = SPDIF_CONSUMER_BUFFER_COUNT;
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        uint cons_free, cons_prepared, playing;
        get_slot_consumer_stats(i, &cons_free, &cons_prepared, &playing);
        pkt.spdif[i].consumer_free = (uint8_t)cons_free;
        pkt.spdif[i].consumer_prepared = (uint8_t)cons_prepared;
        pkt.spdif[i].consumer_playing = (uint8_t)playing;
        // Occupied buffers, so hidden staging buffers count toward the fill.
        uint cons_fill = (cons_free > consumer_capacity) ? 0 : (consumer_capacity - cons_free);
        pkt.spdif[i].consumer_fill_pct = (uint8_t)(cons_fill * 100 / consumer_capacity);
        pkt.spdif[i].consumer_min_fill_pct = spdif_consumer_min_fill_pct[i];
        pkt.spdif[i].consumer_max_fill_pct = spdif_consumer_max_fill_pct[i];
        // 0.01 % of the 768-frame scale; above 100 % when the ring holds more.
        uint32_t fill_frames;
        if (get_slot_consumer_fill_frames(i, &fill_frames)) {
            uint32_t centi = fill_frames * 10000u / 768u;
            pkt.spdif[i].fill_centi_pct = (centi > 65535u) ? 65535u : (uint16_t)centi;
        }
    }

    if (pdm_enabled) {
        pkt.pdm.dma_fill_pct = pdm_get_dma_fill_pct();
        pkt.pdm.dma_min_fill_pct = pdm_dma_min_fill_pct;
        pkt.pdm.dma_max_fill_pct = pdm_dma_max_fill_pct;
        pkt.pdm.ring_fill_pct = pdm_get_ring_fill_pct();
        pkt.pdm.ring_min_fill_pct = pdm_ring_min_fill_pct;
        pkt.pdm.ring_max_fill_pct = pdm_ring_max_fill_pct;
    }
    vc_reply(c, &pkt, sizeof(pkt));
    return VC_OK;
}
_Static_assert(sizeof(BufferStatsPacket) == 44, "SYS.BUFFER_STATS GET_LEN");

static VcResult sys_pipeline_stats_get(VcCtx *c) {
    vc_out_u32(c, 0, (uint32_t)overruns);
    vc_out_u32(c, 4, output_ring_drops);
    vc_out_u32(c, 8, output_ring_min_prewrite);
    vc_out_u32(c, 12, input_delivery_worst_gap_us);
    vc_out_u32(c, 16, bg_over_budget_count);
    return VC_OK;
}

static VcResult sys_pdm_stats_get(VcCtx *c) {
    vc_out_u32(c, 0, pdm_ring_overruns);
    vc_out_u32(c, 4, pdm_ring_underruns);
    vc_out_u32(c, 8, pdm_dma_overruns);
    vc_out_u32(c, 12, pdm_dma_underruns);
    return VC_OK;
}

static VcResult sys_spdif_tx_stats_get(VcCtx *c) {
    vc_out_u32(c, 0, spdif_underruns);
    vc_out_u32(c, 4, audio_spdif_get_dma_starvations());
    for (uint i = 0; i < 4; i++)
        vc_out_u32(c, (uint16_t)(8 + 4 * i), audio_spdif_get_dma_starvations_instance(i));
    return VC_OK;
}

#define SYS_STATS_RESET_WATERMARKS 0x01u

static VcResult sys_stats_reset_set(VcCtx *c) {
    uint8_t mask = vc_in_u8(c, 0);
    if (mask & (uint8_t)~SYS_STATS_RESET_WATERMARKS) return VC_ERR_VALUE;
    if (mask & SYS_STATS_RESET_WATERMARKS) reset_buffer_watermarks();
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
