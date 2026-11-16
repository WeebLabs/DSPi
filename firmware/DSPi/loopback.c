/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * USB audio loopback capture for DSPi (DSPI_LOOPBACK, debug build only).
 *
 *   output slot 0 (audio_pipeline.c)
 *      -> loopback_push_slot0() -> SPSC ring
 *          -> rate-matching servo (fill_audio_packet, per USB frame)
 *              -> isochronous async IN endpoint 0x81 -> USB host
 *
 * Ported from the standalone ~/USBrx recorder's custom UAC1 driver + servo,
 * but the sample source is an internal ring fed by the DSP pipeline instead of
 * a S/PDIF receiver.  The capture IN endpoint is asynchronous to the host's
 * SOF clock in every input mode (USB input is feedback-master on DSPi's own
 * crystal; S/PDIF / I2S input runs on the external source clock), so a servo
 * is required: each USB frame we send a feed-forward number of stereo frames
 * (DSPi's operating rate in samples/ms) plus a small proportional correction
 * that keeps the ring near a target fill level.  Varying the packet size
 * frame-by-frame is the implicit feedback that matches the USB data rate to
 * DSPi's free-running output rate (no resampling).
 *
 * The host must select the matching sample rate; if it does not, the capture
 * streams at the wrong pitch (by design, same contract as USBrx).
 *
 * This entire file compiles to nothing unless DSPI_LOOPBACK is defined.
 */

#ifdef DSPI_LOOPBACK

#include <string.h>

#include "pico.h"                 // __not_in_flash / __not_in_flash_func
#include "tusb.h"
#include "device/usbd_pvt.h"      // usbd_class_driver_t, usbd_edpt_*
#include "class/audio/audio.h"    // AUDIO_* constants (not pulled in when CFG_TUD_AUDIO=0)

#include "loopback.h"
#include "usb_descriptors.h"      // LOOPBACK_* macros, UAC1_REQ_*, UAC1_EP_CTRL_*
#include "usb_audio.h"            // audio_state (for the true output rate)

/* -------------------------------------------------------------------------- */
/* Servo tuning (mirrors USBrx)                                                */
/* -------------------------------------------------------------------------- */
/* Target stereo frames buffered in the ring.  ~5.3 ms @ 48 kHz keeps latency
 * low while leaving ample margin against under/overflow (ring holds 1024). */
#define TARGET_FILL_FRAMES   256

/* The output and USB clocks differ only by a few ppm, so only a tiny
 * correction is ever needed in steady state; the clamp bounds the pull-in
 * rate so the capture is never noticeably time-warped. */
#define SERVO_KP             0.008f   /* (samples/frame) per (frame) of fill error */
#define SERVO_MAX_CORR       2.0f     /* max |correction| in samples/frame */

/* -------------------------------------------------------------------------- */
/* SPSC ring of interleaved 24-bit (sign-extended int32) stereo frames.        */
/*                                                                             */
/* Producer = loopback_push_slot0() (audio callback, core 0 thread).           */
/* Consumer = fill_audio_packet() (tud_task xfer_cb, core 0 thread).           */
/* Both run in core-0 thread context (not IRQ), so there is no true preemption;*/
/* free-running volatile head/tail indices are sufficient.                     */
/* -------------------------------------------------------------------------- */
/* 1024 frames = ~21 ms @ 48 kHz (8 KB).  Kept modest because this BSS competes
 * with the RAM-resident hot text under the XIP build in RP2040's 264 KB. */
#define RING_FRAMES   1024u                 /* power of two */
#define RING_MASK     (RING_FRAMES - 1u)

// Plain .bss (zeroed at startup); keeping an 8 KB zero buffer out of a
// __not_in_flash loaded section avoids bloating the image.  Under the XIP build
// the hot loopback functions carry __not_in_flash_func, so they stay valid
// during flash erase/program.
static int32_t loopback_ring[RING_FRAMES * 2];
static volatile uint32_t ring_head;          /* next frame to write (producer) */
static volatile uint32_t ring_tail;          /* next frame to read  (consumer) */
/* Glitch counters, read by the host via DIAG.LOOPBACK_STATS to tell a capture
 * dropout from a DSP fault; both discontinuities look identical in the audio. */
static volatile uint32_t ring_overflow_count; /* frames dropped, ring full     */
static volatile uint32_t ring_underrun_count; /* silence packets sent, ring dry */

static inline uint32_t ring_count(void) {
    return ring_head - ring_tail;            /* unsigned wrap-safe difference */
}

uint32_t loopback_get_overflow_count(void) { return ring_overflow_count; }
uint32_t loopback_get_underrun_count(void) { return ring_underrun_count; }

void __not_in_flash_func(loopback_push_slot0)(const int32_t *interleaved_s24, uint32_t frames) {
    uint32_t h = ring_head;
    uint32_t t = ring_tail;
    for (uint32_t i = 0; i < frames; i++) {
        if ((h - t) >= RING_FRAMES) {
            /* Ring full (consumer not draining — host not recording, or a
             * stall).  Drop the rest of this block; the servo re-primes. */
            ring_overflow_count += (frames - i);
            break;
        }
        uint32_t idx = (h & RING_MASK) * 2u;
        loopback_ring[idx]     = interleaved_s24[i * 2u];
        loopback_ring[idx + 1] = interleaved_s24[i * 2u + 1u];
        h++;
    }
    ring_head = h;   /* publish after the data writes */
}

/* -------------------------------------------------------------------------- */
/* Servo state (all USB callbacks below run in tud_task() context on core 0)   */
/* -------------------------------------------------------------------------- */
static uint32_t g_last_freq     = 0;
static float    g_nominal_fpf   = 48.0f;   /* nominal stereo frames per USB frame */
static float    g_acc           = 0.0f;    /* fractional-sample accumulator */
static uint32_t g_silence_frames = 48;     /* silence packet size = rate / 1000  */
static bool     g_primed        = false;   /* ring has reached target at least once */

/* IN packet scratch (.bss). Only touched while no transfer is in flight. */
static uint8_t __attribute__((aligned(4))) g_pkt[LOOPBACK_EP_IN_SIZE];

static inline void refresh_rate(void) {
    uint32_t f = audio_state.freq;
    if (f != g_last_freq) {
        g_last_freq      = f;
        g_nominal_fpf    = (float)f / 1000.0f;
        g_silence_frames = f / 1000u;
        g_acc            = 0.0f;
    }
}

static inline uint16_t silence_bytes(void) {
    return (uint16_t)(g_silence_frames * LOOPBACK_BYTES_PER_FRAME);
}

/*
 * Fill 'buf' with the next isochronous IN packet and return its size in bytes.
 * Runs once per USB frame (each time the previous IN packet completes).
 */
static uint16_t __not_in_flash_func(fill_audio_packet)(uint8_t *buf) {
    refresh_rate();

    uint32_t avail = ring_count();   /* stereo frames available */

    /* Prime: wait until the ring fills to the target before streaming. */
    if (!g_primed) {
        if (avail >= (uint32_t)TARGET_FILL_FRAMES) {
            g_primed = true;
            g_acc    = 0.0f;
        } else {
            uint16_t n = silence_bytes();
            memset(buf, 0, n);
            return n;
        }
    }

    /* ---- Rate-matching servo: nominal feed-forward + clamped P correction ---- */
    int   err  = (int)avail - TARGET_FILL_FRAMES;
    float corr = SERVO_KP * (float)err;
    if (corr >  SERVO_MAX_CORR) corr =  SERVO_MAX_CORR;
    if (corr < -SERVO_MAX_CORR) corr = -SERVO_MAX_CORR;

    g_acc += g_nominal_fpf + corr;
    int n = (int)g_acc;            /* floor; g_acc is always >= 0 */
    g_acc -= (float)n;

    if (n < 0) n = 0;
    if (n > LOOPBACK_MAX_FRAMES_PER_PACKET) n = LOOPBACK_MAX_FRAMES_PER_PACKET;
    if ((uint32_t)n > avail) n = (int)avail;

    if (n == 0) {
        /* Real underrun: keep the stream alive and re-prime.  Counted once per
         * episode; the following packets take the prime branch above. */
        ring_underrun_count++;
        g_primed = false;
        uint16_t z = silence_bytes();
        memset(buf, 0, z);
        return z;
    }

    /* ---- Copy + convert n stereo frames to 24-bit LE ---- */
    uint32_t t = ring_tail;
    uint8_t *dst = buf;
    for (int i = 0; i < n; i++) {
        uint32_t idx = (t & RING_MASK) * 2u;
        int32_t l = loopback_ring[idx];
        int32_t r = loopback_ring[idx + 1];
        t++;
        *dst++ = (uint8_t)(l & 0xFF);
        *dst++ = (uint8_t)((l >> 8) & 0xFF);
        *dst++ = (uint8_t)((l >> 16) & 0xFF);
        *dst++ = (uint8_t)(r & 0xFF);
        *dst++ = (uint8_t)((r >> 8) & 0xFF);
        *dst++ = (uint8_t)((r >> 16) & 0xFF);
    }
    ring_tail = t;   /* publish after the data reads */

    return (uint16_t)(n * LOOPBACK_BYTES_PER_FRAME);
}

/* ========================================================================== */
/* Custom UAC1 capture class driver                                            */
/* ========================================================================== */

static struct {
    uint8_t        ac_itf;       /* capture AudioControl interface number */
    uint8_t        as_itf;       /* capture AudioStreaming interface number */
    uint8_t        cur_alt;      /* current AS alternate setting */
    bool           ep_open;      /* iso IN endpoint open */
    const uint8_t *ep_desc;      /* pointer to the iso IN endpoint descriptor */
    uint8_t        pending_cs;   /* deferred SET_CUR control selector */
    uint8_t        pending_len;  /* deferred SET_CUR data length */
} lb;

static uint8_t lb_ctrl_buf[8];   /* SET_CUR data-stage scratch */

/* Fill the scratch packet via the servo and queue it on the IN endpoint. */
static void __not_in_flash_func(lb_arm_in)(uint8_t rhport) {
    uint16_t len = fill_audio_packet(g_pkt);
    usbd_edpt_xfer(rhport, LOOPBACK_IN_ENDPOINT, g_pkt, len);
}

/* Apply an AudioStreaming alternate setting (0 = idle, 1 = streaming). */
static bool lb_apply_alt(uint8_t rhport, uint8_t alt) {
    if (alt > 1) return false;
    if (alt == lb.cur_alt) return true;   /* idempotent SET_INTERFACE */
    lb.cur_alt = alt;

    if (alt == 1) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
        TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *)lb.ep_desc));
#else
        TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *)lb.ep_desc));
#endif
        /* Clear any stale iso EP state (AVAIL/busy) left from a previous alt
         * cycle; stall+clear flushes both the stack and hardware flags. */
        usbd_edpt_stall(rhport, LOOPBACK_IN_ENDPOINT);
        usbd_edpt_clear_stall(rhport, LOOPBACK_IN_ENDPOINT);

        lb.ep_open = true;

        /* Drop any stale audio so capture starts at "now", then re-prime. */
        ring_tail = ring_head;
        g_primed  = false;
        g_acc     = 0.0f;

        lb_arm_in(rhport);   /* queue the first packet */
    } else {
        if (lb.ep_open) {
            usbd_edpt_close(rhport, LOOPBACK_IN_ENDPOINT);
            lb.ep_open = false;
        }
    }
    return true;
}

static void lb_driver_init(void) {
    memset(&lb, 0, sizeof(lb));
}

static bool lb_driver_deinit(void) {
    return true;
}

static void lb_driver_reset(uint8_t rhport) {
    (void)rhport;
    lb.cur_alt = 0;
    lb.ep_open = false;
    g_primed   = false;
}

/* Claim the capture AC + AS interfaces and reserve the iso IN endpoint. */
static uint16_t lb_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    /* Scope strictly to the capture AudioControl interface so this driver
     * never claims the playback function (also AUDIO/CONTROL/alt0). */
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_AUDIO);
    TU_VERIFY(itf_desc->bInterfaceSubClass == AUDIO_SUBCLASS_CONTROL);
    TU_VERIFY(itf_desc->bAlternateSetting == 0);
    TU_VERIFY(itf_desc->bInterfaceNumber == ITF_NUM_LOOPBACK_AC);

    lb.ac_itf = itf_desc->bInterfaceNumber;

    const uint8_t *p   = (const uint8_t *)itf_desc;
    const uint8_t *end = p + max_len;
    uint16_t drv_len = 0;

    /* AC standard interface descriptor */
    drv_len += tu_desc_len(p);
    p       += tu_desc_len(p);

    /* AC class-specific descriptors (header, terminals) */
    while (p < end && tu_desc_type(p) == TUSB_DESC_CS_INTERFACE) {
        drv_len += tu_desc_len(p);
        p       += tu_desc_len(p);
    }

    /* AS interfaces (alt 0 + alt 1) and their CS + endpoint descriptors */
    while (p < end && tu_desc_type(p) == TUSB_DESC_INTERFACE) {
        tusb_desc_interface_t const *as = (tusb_desc_interface_t const *)p;
        if (as->bInterfaceClass != TUSB_CLASS_AUDIO ||
            as->bInterfaceSubClass != AUDIO_SUBCLASS_STREAMING) {
            break;
        }
        lb.as_itf = as->bInterfaceNumber;

        drv_len += tu_desc_len(p);
        p       += tu_desc_len(p);

        while (p < end && tu_desc_type(p) != TUSB_DESC_INTERFACE) {
            if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
                tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;
                if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS &&
                    tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
                    lb.ep_desc = p;   /* remember for iso (de)activation */
#ifdef TUP_DCD_EDPT_ISO_ALLOC
                    usbd_edpt_iso_alloc(rhport, ep->bEndpointAddress, LOOPBACK_EP_IN_SIZE);
#endif
                }
            }
            drv_len += tu_desc_len(p);
            p       += tu_desc_len(p);
        }
    }

    return drv_len;
}

static bool lb_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage == CONTROL_STAGE_SETUP) {
        /* ----- Standard requests on our interfaces ----- */
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
            uint8_t itf = TU_U16_LOW(req->wIndex);

            if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
                uint8_t alt = TU_U16_LOW(req->wValue);
                if (itf == lb.ac_itf) {
                    return (alt == 0) ? tud_control_status(rhport, req) : false;
                }
                if (itf == lb.as_itf) {
                    return lb_apply_alt(rhport, alt) ? tud_control_status(rhport, req) : false;
                }
                return false;
            }
            if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
                static uint8_t alt_resp;
                if (itf == lb.ac_itf)      alt_resp = 0;
                else if (itf == lb.as_itf) alt_resp = lb.cur_alt;
                else                       return false;
                return tud_control_xfer(rhport, req, &alt_resp, 1);
            }
            return false;
        }

        /* ----- Class requests (sampling-frequency control on the EP) ----- */
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
            req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {

            if (TU_U16_LOW(req->wIndex) != LOOPBACK_IN_ENDPOINT) return false;
            uint8_t cs = TU_U16_HIGH(req->wValue);

            if (req->bmRequestType_bit.direction == TUSB_DIR_IN) {
                /* GET_CUR sampling frequency — report DSPi's true output rate. */
                if (req->bRequest == UAC1_REQ_GET_CUR && cs == UAC1_EP_CTRL_SAMPLING_FREQ) {
                    static uint8_t freq[3];
                    uint32_t f = audio_state.freq;
                    freq[0] = (uint8_t)(f & 0xFF);
                    freq[1] = (uint8_t)((f >> 8) & 0xFF);
                    freq[2] = (uint8_t)((f >> 16) & 0xFF);
                    return tud_control_xfer(rhport, req, freq, 3);
                }
                return false;
            } else {
                /* SET_CUR sampling frequency: accept, but the servo tracks
                 * audio_state.freq regardless (DSPi is the clock master). */
                if (req->bRequest == UAC1_REQ_SET_CUR && cs == UAC1_EP_CTRL_SAMPLING_FREQ) {
                    uint16_t len = req->wLength;
                    if (len == 0 || len > sizeof(lb_ctrl_buf)) return false;
                    lb.pending_cs  = cs;
                    lb.pending_len = (uint8_t)len;
                    return tud_control_xfer(rhport, req, lb_ctrl_buf, len);
                }
                return false;
            }
        }
        return false;
    }

    if (stage == CONTROL_STAGE_DATA) {
        /* SET_CUR data stage — value validated but not stored (informational). */
        lb.pending_cs = 0;
        return true;
    }

    return true;   /* CONTROL_STAGE_ACK */
}

static bool __not_in_flash_func(lb_driver_xfer_cb)(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    (void)result;
    (void)xferred_bytes;
    if (ep_addr == LOOPBACK_IN_ENDPOINT) {
        if (lb.ep_open) lb_arm_in(rhport);   /* refill + queue next frame */
        return true;
    }
    return false;
}

const usbd_class_driver_t loopback_uac1_driver = {
    .name            = "DSPi_LB",
    .init            = lb_driver_init,
    .deinit          = lb_driver_deinit,
    .reset           = lb_driver_reset,
    .open            = lb_driver_open,
    .control_xfer_cb = lb_driver_control_xfer_cb,
    .xfer_cb         = lb_driver_xfer_cb,
    .sof             = NULL,
};

#endif // DSPI_LOOPBACK
