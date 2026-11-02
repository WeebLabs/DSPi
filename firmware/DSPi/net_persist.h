/*
 * net_persist.h - persistent WiFi / Sendspin settings and device key
 * (DSPI_ENABLE_WIFI builds only).
 *
 * Storage: two dedicated 4 KB flash sectors directly below the preset region
 * (A/B, newest valid record wins).  Independent of the preset directory.
 *
 * Threading: net_persist_request_save() is safe from the USB ISR (sets a
 * flag).  The flash write itself runs only from the main loop inside the
 * project's prepare/complete_flash_write_operation bracket:
 *
 *     if (net_persist_commit_needed()) {
 *         prepare_flash_write_operation();
 *         net_persist_save_current();
 *         complete_flash_write_operation_full();
 *     }
 *
 * Secrets (passphrase, private key) are never exposed by any GET command.
 */
#ifndef NET_PERSIST_H
#define NET_PERSIST_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Boot: read flash, apply saved SSID/pass/server through the net_input
 * setters.  Call once before net_input_start().  No flash writes. */
void net_persist_load_into_net(void);

/* ISR-safe: ask the main loop to persist the current net_input values. */
void net_persist_request_save(void);

/* Main loop: consumes a pending request; true if flash must be written
 * (pending AND values differ from what is already stored). */
bool net_persist_commit_needed(void);

/* Main loop, inside the flash bracket: write current values.  true on success
 * (or nothing to do).  NEVER call from an ISR. */
bool net_persist_save_current(void);

/* Dev/diagnostic: true after a successful boot load of a valid record. */
bool net_persist_loaded(void);

#ifdef __cplusplus
}
#endif

#endif /* NET_PERSIST_H */
