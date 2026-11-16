/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * USB audio loopback capture for DSPi (DSPI_LOOPBACK, debug build only).
 *
 * Exposes output slot 0 as a 2-ch 24-bit isochronous IN (recording) endpoint
 * so a host can capture exactly what slot 0 produces (post-DSP, post-mute,
 * output-type-agnostic) without an external S/PDIF-to-USB recorder.  This folds
 * the standalone ~/USBrx recorder's capability into DSPi itself; the sample
 * source is internal (the finalized slot-0 output buffer) rather than a S/PDIF
 * receiver, so no PIO/DMA/spdif_rx is involved.
 *
 * The whole module compiles to nothing unless DSPI_LOOPBACK is defined, so this
 * header is safe to include unconditionally.
 */

#ifndef LOOPBACK_H
#define LOOPBACK_H

#ifdef DSPI_LOOPBACK

#include <stdint.h>

#include "tusb.h"
#include "device/usbd_pvt.h"   // usbd_class_driver_t

#ifdef __cplusplus
extern "C" {
#endif

// Push slot-0's finalized, interleaved 24-bit L/R samples into the loopback
// capture ring.  Called from the audio pipeline (process_input_block) right
// before slot 0's output buffer is handed to the output DMA.
// `interleaved_s24` points at 2*frames int32 words, each holding a sign-
// extended 24-bit sample (range +/-8388607); only the low 24 bits are used.
//
// Read-only with respect to the source buffer — it copies out, never writes
// back — so it cannot perturb inter-slot output alignment.  RAM-resident
// (runs in the audio callback); drops on ring overflow and never blocks.
void loopback_push_slot0(const int32_t *interleaved_s24, uint32_t frames);

// Capture-glitch counters, surfaced via DIAG.LOOPBACK_STATS.  A dropped or inserted
// frame is indistinguishable from a DSP fault in the captured audio, so the
// host samples these around a measurement to tell the two apart.
uint32_t loopback_get_overflow_count(void);   // frames dropped, ring full
uint32_t loopback_get_underrun_count(void);   // silence packets, ring dry

// Custom UAC1 capture class driver.  Registered alongside the playback UAC1
// driver from usbd_app_driver_get_cb() in usb_audio.c.
extern const usbd_class_driver_t loopback_uac1_driver;

#ifdef __cplusplus
}
#endif

#endif // DSPI_LOOPBACK
#endif // LOOPBACK_H
