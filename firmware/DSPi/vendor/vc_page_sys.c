/*
 * vc_page_sys.c: SYS page (0x01). Discovery, identity, device name, bootloader
 * and telemetry. Identity and BUFFER_STATS keep their 1.1.6 reply layouts byte for byte.
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
#include "vc_pins.h"
#include "pico/audio_spdif.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/adc.h"

// Convert vreg voltage enum to millivolts
static uint16_t vreg_voltage_to_mv(enum vreg_voltage voltage) {
    // Voltage enum values map to specific voltages
    // See hardware/vreg.h for the full table
    static const uint16_t voltage_table[] = {
#if !PICO_RP2040
        550,  // VREG_VOLTAGE_0_55 = 0b00000
        600,  // VREG_VOLTAGE_0_60 = 0b00001
        650,  // VREG_VOLTAGE_0_65 = 0b00010
        700,  // VREG_VOLTAGE_0_70 = 0b00011
        750,  // VREG_VOLTAGE_0_75 = 0b00100
        800,  // VREG_VOLTAGE_0_80 = 0b00101
#endif
        850,  // VREG_VOLTAGE_0_85 = 0b00110
        900,  // VREG_VOLTAGE_0_90 = 0b00111
        950,  // VREG_VOLTAGE_0_95 = 0b01000
        1000, // VREG_VOLTAGE_1_00 = 0b01001
        1050, // VREG_VOLTAGE_1_05 = 0b01010
        1100, // VREG_VOLTAGE_1_10 = 0b01011
        1150, // VREG_VOLTAGE_1_15 = 0b01100
        1200, // VREG_VOLTAGE_1_20 = 0b01101
        1250, // VREG_VOLTAGE_1_25 = 0b01110
        1300, // VREG_VOLTAGE_1_30 = 0b01111
#if !PICO_RP2040
        1350, // VREG_VOLTAGE_1_35 = 0b10000
        1400, // VREG_VOLTAGE_1_40 = 0b10001
        1500, // VREG_VOLTAGE_1_50 = 0b10010
        1600, // VREG_VOLTAGE_1_60 = 0b10011
        1650, // VREG_VOLTAGE_1_65 = 0b10100
        1700, // VREG_VOLTAGE_1_70 = 0b10101
        1800, // VREG_VOLTAGE_1_80 = 0b10110
        1900, // VREG_VOLTAGE_1_90 = 0b10111
        2000, // VREG_VOLTAGE_2_00 = 0b11000
        2350, // VREG_VOLTAGE_2_35 = 0b11001
        2500, // VREG_VOLTAGE_2_50 = 0b11010
        2650, // VREG_VOLTAGE_2_65 = 0b11011
        2800, // VREG_VOLTAGE_2_80 = 0b11100
        3000, // VREG_VOLTAGE_3_00 = 0b11101
        3150, // VREG_VOLTAGE_3_15 = 0b11110
        3300, // VREG_VOLTAGE_3_30 = 0b11111
#endif
    };

#if PICO_RP2040
    // RP2040: offset by 6 entries (0.55-0.80V not available)
    uint8_t index = (voltage >= 6) ? (voltage - 6) : 0;
#else
    uint8_t index = voltage;
#endif

    if (index < sizeof(voltage_table) / sizeof(voltage_table[0])) {
        return voltage_table[index];
    }
    return 1100; // Default fallback
}

// Read ADC temperature sensor and return temperature in centi-degrees C
// Formula from SDK docs (same for RP2040/RP2350):
// T = 27 - (ADC_Voltage - 0.706) / 0.001721
static int16_t read_temperature_cdeg(void) {
    const float conversion_factor = 3.3f / 4095.0f;

    // Temperature sensor channel: auto-detects based on chip variant
    // RP2040, RP2350A (QFN-60): channel 4
    // RP2350B (QFN-80): channel 8
    adc_select_input(NUM_ADC_CHANNELS - 1);
    uint16_t adc_raw = adc_read();
    float voltage = adc_raw * conversion_factor;
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;

    return (int16_t)(temp_c * 100.0f); // Convert to centi-degrees
}

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

// ---- Pin map (pin_availability_query_spec.md section 8) ----

_Static_assert(8 + 4 * PIN_CLAIMS_STEADY_MAX <= VC_REPLY_MAX,
               "SYS.PIN_CLAIMS must hold every claim of a quiet device; add paging first");

static VcResult sys_pin_state_get(VcCtx *c) {
    uint32_t gen = pin_generation_sync(false);
    vc_out_u32(c, 0, gen);
    vc_out_u8(c, 4, pin_gpio_count());
    vc_out_u8(c, 5, pin_state_live()->count);
    vc_out_u16(c, 6, pin_restore_substituted());
    uint64_t m = pin_gpio_mask();
    vc_out_u32(c, 8, (uint32_t)m);
    vc_out_u32(c, 12, (uint32_t)(m >> 32));
    return VC_OK;
}

// Entries sorted by GPIO. Only a change in flight (a Control Surfaces revert) can
// outgrow the reply, and it answers BUSY until the change completes.
static VcResult sys_pin_claims_get(VcCtx *c) {
    uint32_t gen = pin_generation_sync(false);
    const PinState *st = pin_state_live();
    uint16_t need = (uint16_t)(8 + 4u * st->count);
    if (st->overflow || need > c->out_cap) return VC_ERR_BUSY;
    vc_out_u32(c, 0, gen);
    c->out[4] = st->count;
    c->out[5] = c->out[6] = c->out[7] = 0;
    PinClaim *e = (PinClaim *)(void *)(c->out + 8);
    for (uint8_t i = 0; i < st->count; i++) {
        PinClaim v = st->claims[i];
        uint8_t j = i;
        for (; j > 0 && e[j - 1].gpio > v.gpio; j--) e[j] = e[j - 1];
        e[j] = v;
    }
    c->out_len = need;
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

// USB reboots after the status stage (vc_usb.c), UART and I2C after their reply
// has gone out (their poll loops). Internal callers have no reply to wait for.
static VcResult sys_enter_bootloader_set(VcCtx *c) {
    if (c->src == CTRL_SOURCE_USB) {
        vc_usb_bootloader_after_status();
    } else if (c->src == CTRL_SOURCE_UART || c->src == CTRL_SOURCE_I2C) {
        ctrl_reboot_request(c->src);
    } else {
        reset_usb_boot(0, 0);
    }
    return VC_OK;
}

#define VC_PAGE_DEF "pages/sys.def"
#include "vc_page_table.inc"
