/*
 * net_persist.c - persistent WiFi / Sendspin settings and device key.
 * Built only with -DDSPI_ENABLE_WIFI=ON.  See net_persist.h.
 *
 * Flash layout (byte offsets from start of flash):
 *
 *   [ firmware ... ][ free ... ][ NP sector A ][ NP sector B ][ presets: dir + 10 slots ]
 *                                ^ PRESET_BASE-8K              ^ PRESET_BASE .. flash end
 *
 * The two sectors sit immediately below the preset region (which is anchored
 * to PICO_FLASH_SIZE_BYTES by flash_storage.c), so they are above the
 * firmware image on any 2 MB / 4 MB part as long as the image is smaller than
 * flash - 21*4K - 8K (checked at runtime against __flash_binary_end; a write
 * is refused rather than risk overwriting code).  Each save erases and
 * programs only the OLDER sector, so the previous valid record survives a
 * power loss mid-write.
 */
#include "net_persist.h"

#ifdef DSPI_ENABLE_WIFI

#include "net_persist_codec.h"
#include "net_input.h"
#include "sendspin_client.h"
#include "flash_storage.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include <string.h>

#define NP_SECTORS            2u
#define NP_REGION_BYTES       (NP_SECTORS * FLASH_SECTOR_SIZE)

extern char __flash_binary_end;   /* linker symbol, XIP address of image end */

static inline uint32_t np_region_offset(void) {
    return flash_storage_preset_base_offset() - NP_REGION_BYTES;
}
static inline uint32_t np_sector_offset(unsigned idx) {
    return np_region_offset() + idx * FLASH_SECTOR_SIZE;
}

/* Cached view of what is stored in flash, and the key (key may exist only in
 * RAM until the deferred save lands; sendspin_key_load serves it from here). */
static NpPayload  persisted;           /* last known flash content */
static bool       persisted_valid;     /* a valid record exists in flash */
static uint32_t   persisted_seq;
static int        persisted_sector = -1;
static NpPayload  live_key_holder;     /* only .key / FLAG_KEY used */
static bool       booted_loaded;
static bool       scanned;
static volatile bool save_pending;

/* Scan both sectors via XIP; fill newest valid record. */
static void np_scan(void) {
    scanned = true;
    persisted_valid = false;
    persisted_sector = -1;
    memset(&persisted, 0, sizeof(persisted));
    persisted_seq = 0;
    for (unsigned i = 0; i < NP_SECTORS; i++) {
        const uint8_t *p = (const uint8_t *)(XIP_BASE + np_sector_offset(i));
        NpPayload pl;
        uint32_t seq;
        if (!np_decode(p, FLASH_SECTOR_SIZE, &seq, &pl)) continue;
        if (!persisted_valid || np_seq_newer(seq, persisted_seq)) {
            persisted = pl;
            persisted_seq = seq;
            persisted_sector = (int)i;
            persisted_valid = true;
        }
    }
    memset(&live_key_holder, 0, sizeof(live_key_holder));
    if (persisted_valid && (persisted.flags & NP_FLAG_KEY)) {
        live_key_holder.flags = NP_FLAG_KEY;
        memcpy(live_key_holder.key, persisted.key, 32);
    }
}

void net_persist_load_into_net(void) {
    np_scan();
    if (!persisted_valid) return;
    booted_loaded = true;
    if (persisted.flags & NP_FLAG_WIFI) {
        net_wifi_set_credentials(persisted.ssid, persisted.pass);
    }
    if ((persisted.flags & NP_FLAG_SERVER) && persisted.host[0]) {
        net_sendspin_set_server(persisted.host, persisted.port);
    }
}

bool net_persist_loaded(void) { return booted_loaded; }

void net_persist_request_save(void) {
    save_pending = true;
    __dmb();
}

/* Build the payload that represents "now" (getters + held key). */
static void np_build_current(NpPayload *out) {
    memset(out, 0, sizeof(*out));
    const char *ssid = net_wifi_get_ssid();
    const char *pass = net_wifi_get_pass();
    const char *host = net_sendspin_get_host();
    if (ssid && ssid[0]) {
        out->flags |= NP_FLAG_WIFI;
        strncpy(out->ssid, ssid, NP_SSID_CAP - 1);
        if (pass) strncpy(out->pass, pass, NP_PASS_CAP - 1);
    }
    if (host && host[0]) {
        out->flags |= NP_FLAG_SERVER;
        strncpy(out->host, host, NP_HOST_CAP - 1);
        out->port = net_sendspin_get_port();
    }
    if (live_key_holder.flags & NP_FLAG_KEY) {
        out->flags |= NP_FLAG_KEY;
        memcpy(out->key, live_key_holder.key, 32);
    }
}

static bool np_equal_to_persisted(const NpPayload *cur) {
    if (!persisted_valid) {
        /* Nothing stored: only "empty" payload is equivalent to that. */
        return cur->flags == 0;
    }
    NpPayload a = *cur, b = persisted;
    /* strings compared up to NUL only; zero the tails for a byte compare */
    return memcmp(&a, &b, sizeof(a)) == 0;
}

bool net_persist_commit_needed(void) {
    if (!save_pending) return false;
    save_pending = false;
    __dmb();
    if (!scanned) np_scan();
    NpPayload cur;
    np_build_current(&cur);
    return !np_equal_to_persisted(&cur);
}

bool net_persist_save_current(void) {
    if (!scanned) np_scan();
    NpPayload cur;
    np_build_current(&cur);
    if (np_equal_to_persisted(&cur)) return true;

    /* Pick the sector that does NOT hold the newest valid record. */
    unsigned target = (persisted_sector == 0) ? 1u : 0u;
    uint32_t off = np_sector_offset(target);

    /* Never overwrite firmware: the region must lie wholly above the image. */
    uint32_t image_end = (uint32_t)(uintptr_t)&__flash_binary_end - XIP_BASE;
    if (np_region_offset() < image_end) return false;

    uint32_t seq = persisted_valid ? persisted_seq + 1u : 1u;
    uint8_t rec[NP_RECORD_BYTES];
    if (np_encode(rec, sizeof(rec), seq, &cur) == 0) return false;

    bool ok = (flash_storage_write_raw(off, rec, sizeof(rec)) == 0);

    /* Read back through XIP and validate the full record. */
    NpPayload rb;
    uint32_t rb_seq;
    ok = ok && np_decode((const uint8_t *)(XIP_BASE + off), FLASH_SECTOR_SIZE,
                         &rb_seq, &rb)
            && rb_seq == seq && memcmp(&rb, &cur, sizeof(rb)) == 0;
    if (ok) {
        persisted = rb;
        persisted_seq = seq;
        persisted_sector = (int)target;
        persisted_valid = true;
    } else {
        /* Leave the cache describing the (still valid) older record, and
         * retry on the next request. */
        NpPayload keep = live_key_holder;   /* unsaved key must survive */
        np_scan();
        live_key_holder = keep;
        memset(&keep, 0, sizeof(keep));
    }
    memset(rec, 0, sizeof(rec));
    memset(&rb, 0, sizeof(rb));
    memset(&cur, 0, sizeof(cur));
    return ok;
}

/* ------------------------------------------------------------------------ */
/* Sendspin device key (overrides the weak no-op hooks in sendspin_client.c) */
/* ------------------------------------------------------------------------ */

bool sendspin_key_load(uint8_t priv[32]) {
    if (!scanned) np_scan();
    if (!(live_key_holder.flags & NP_FLAG_KEY)) return false;
    memcpy(priv, live_key_holder.key, 32);
    return true;
}

void sendspin_key_store(const uint8_t priv[32]) {
    if (!scanned) np_scan();
    if ((live_key_holder.flags & NP_FLAG_KEY) &&
        memcmp(live_key_holder.key, priv, 32) == 0) {
        return;   /* unchanged */
    }
    live_key_holder.flags = NP_FLAG_KEY;
    memcpy(live_key_holder.key, priv, 32);
    /* Deferred: this runs from the net poll; the main loop writes flash. */
    net_persist_request_save();
}

#endif /* DSPI_ENABLE_WIFI */
