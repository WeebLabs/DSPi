/*
 * USB Audio SPSC Ring Buffer
 *
 * Lock-free single-producer single-consumer ring buffer for decoupling
 * the USB isochronous audio packet ISR from main-loop DSP processing.
 *
 * Producer: USB audio packet ISR (_as_audio_packet) on Core 0
 * Consumer: main loop (usb_audio_drain_ring) on Core 0
 *
 * Design follows the PDM ring pattern in pdm_generator.c.  Fixed-slot
 * layout avoids wrap-boundary splits and variable-length allocation.
 *
 * Memory barriers:
 *   RP2040 (Cortex-M0+): volatile alone is sufficient (in-order single-bus).
 *   RP2350 (Cortex-M33): __dmb() required before publishing index updates
 *   and after observing them, due to the write buffer.
 *   Both platforms use __dmb() for portability and documentation of intent.
 */

#ifndef USB_AUDIO_RING_H
#define USB_AUDIO_RING_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "hardware/sync.h"   // __dmb()

// Ring geometry.  4 slots = 4ms of jitter absorption at 1 packet/ms.
// The ring should be nearly empty in steady state; its purpose is
// decoupling, not deep buffering.
#define USB_RING_SLOTS      4
#define USB_RING_SLOT_MASK  (USB_RING_SLOTS - 1)

// Maximum payload per slot.  Matches wMaxPacketSize for the 24-bit
// alt-setting at 96 kHz (usb_descriptors.c:214).
#define USB_RING_MAX_PKT    576

// ---------------------------------------------------------------------------
// Slot and ring structures
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t data_len;                  // Actual byte count this packet
    uint8_t  data[USB_RING_MAX_PKT];   // Raw USB audio payload
} usb_audio_slot_t;

typedef struct {
    usb_audio_slot_t slots[USB_RING_SLOTS];
    volatile uint8_t head;              // Written by USB ISR (producer) only
    volatile uint8_t tail;              // Written by main loop (consumer) only
    volatile uint32_t overrun_count;    // Ring-full drops + oversize drops
} usb_audio_ring_t;

// ---------------------------------------------------------------------------
// Producer — called from USB ISR (must be in RAM for flash safety)
// ---------------------------------------------------------------------------

// Push a USB audio packet into the ring.
// Returns true on success, false if the ring is full or the packet is
// oversize.  On failure, overrun_count is incremented and the packet
// is silently dropped (no partial frames — clamping would produce
// malformed data).
static inline bool __not_in_flash_func(usb_audio_ring_push)(
        usb_audio_ring_t *ring, const uint8_t *data, uint16_t len) {

    // Defensive: reject oversize packets rather than clamping.
    if (len > USB_RING_MAX_PKT) {
        ring->overrun_count++;
        return false;
    }

    uint8_t h = ring->head;
    uint8_t next_h = (h + 1) & USB_RING_SLOT_MASK;

    if (next_h == ring->tail) {
        // Ring full — drop packet.
        ring->overrun_count++;
        return false;
    }

    usb_audio_slot_t *slot = &ring->slots[h];
    slot->data_len = len;
    memcpy(slot->data, data, len);

    // Release barrier: ensure slot data is visible before head advances.
    __dmb();
    ring->head = next_h;

    return true;
}

// ---------------------------------------------------------------------------
// Consumer — called from main loop (thread context)
// ---------------------------------------------------------------------------

// Peek at the next available slot without consuming it.
// Returns a pointer to the slot, or NULL if the ring is empty.
// The returned pointer is valid until usb_audio_ring_consume() is called.
static inline usb_audio_slot_t * __not_in_flash_func(usb_audio_ring_peek)(
        usb_audio_ring_t *ring) {

    if (ring->tail == ring->head)
        return NULL;

    // Acquire barrier: ensure we read slot data written before head advanced.
    __dmb();
    return &ring->slots[ring->tail];
}

// Advance the tail after processing a peeked slot.
// Must be called exactly once per successful peek.
static inline void __not_in_flash_func(usb_audio_ring_consume)(
        usb_audio_ring_t *ring) {

    // Release barrier: ensure all reads of the slot are complete before
    // we advance tail (which frees the slot for the producer).
    __dmb();
    ring->tail = (ring->tail + 1) & USB_RING_SLOT_MASK;
}

// ---------------------------------------------------------------------------
// Lifecycle — called from main loop during stream transitions
// ---------------------------------------------------------------------------

// Discard all pending data.  Used on stream stop/start to flush stale
// packets from a previous stream.
static inline void usb_audio_ring_flush(usb_audio_ring_t *ring) {
    __dmb();  // Consistent with barrier discipline in push/peek/consume
    ring->tail = ring->head;
}

#endif // USB_AUDIO_RING_H
