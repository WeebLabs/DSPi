/*
 * vc_page_xport.c: XPORT page. Command set: vendor_command_overhaul_plan.md section 5.11.
 * A config SET is applied and persisted on the main loop; its PIN_CONFIG_* result
 * lands in STATUS. GETs return the stored config, the source of truth.
 */

#include "vc_core.h"
#include "vc_pins.h"
#include "config.h"
#include "usb_audio.h"
#include "flash_storage.h"
#include "uart_control.h"
#include "i2c_control.h"
#include "hardware/sync.h"

_Static_assert(sizeof(UartCtrlConfig) == 8 && sizeof(I2cCtrlConfig) == 8,
               "XPORT config payloads are 8 bytes");
_Static_assert(sizeof(CtrlIfaceStatus) == 8, "XPORT.STATUS is 8 bytes");

static VcResult xport_uart_config_get(VcCtx *c) {
    UartCtrlConfig cfg;
    preset_get_ctrl_iface(&cfg, NULL);
    memcpy(c->out, &cfg, sizeof(cfg));
    return VC_OK;
}

// Refuses only what uart_ctrl_validate() refuses regardless of pin ownership. The
// live interface's own pins look taken until the apply tears it down, so the rest waits.
static VcResult xport_uart_config_set(VcCtx *c) {
    UartCtrlConfig cfg;
    memcpy(&cfg, c->in, sizeof(cfg));
    if (cfg.enabled > 1 || cfg.notify_enable > 1 ||
        cfg.baud < UART_CTRL_BAUD_MIN || cfg.baud > UART_CTRL_BAUD_MAX)
        return VC_ERR_VALUE;
    if (!is_valid_gpio_pin(cfg.tx_pin) || !is_valid_gpio_pin(cfg.rx_pin)) return VC_ERR_PIN_INVALID;
    memcpy((void *)&ctrl_set_uart_val, &cfg, sizeof(cfg));
    __dmb();
    ctrl_set_uart_pending = true;
    return VC_OK;
}

static VcResult xport_i2c_config_get(VcCtx *c) {
    I2cCtrlConfig cfg;
    preset_get_ctrl_iface(NULL, &cfg);
    memcpy(c->out, &cfg, sizeof(cfg));
    return VC_OK;
}

// Same split as the UART SET: scalar and GPIO range checks here, ownership on the main loop.
static VcResult xport_i2c_config_set(VcCtx *c) {
    I2cCtrlConfig cfg;
    memcpy(&cfg, c->in, sizeof(cfg));
    if (cfg.enabled > 1 || cfg.address < I2C_CTRL_ADDRESS_MIN || cfg.address > I2C_CTRL_ADDRESS_MAX)
        return VC_ERR_VALUE;
    if (!is_valid_gpio_pin(cfg.sda_pin) || !is_valid_gpio_pin(cfg.scl_pin)) return VC_ERR_PIN_INVALID;
    memcpy((void *)&ctrl_set_i2c_val, &cfg, sizeof(cfg));
    __dmb();
    ctrl_set_i2c_pending = true;
    return VC_OK;
}

static VcResult xport_status_get(VcCtx *c) {
    CtrlIfaceStatus st = {
        .uart_last_status = ctrl_uart_last_status,
        .uart_live        = uart_ctrl_is_live() ? 1 : 0,
        .i2c_last_status  = ctrl_i2c_last_status,
        .i2c_live         = i2c_ctrl_is_live() ? 1 : 0,
        .proto_version    = CTRL_IFACE_PROTO_VERSION,
        .reserved         = {0},
    };
    memcpy(c->out, &st, sizeof(st));
    return VC_OK;
}

#define VC_PAGE_DEF "pages/xport.def"
#include "vc_page_table.inc"
