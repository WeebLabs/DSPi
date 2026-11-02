/*
 * net_persist_codec.h - portable record format for persisted network settings.
 *
 * Header-only and hardware-free so tools/net_persist_test can compile it on a
 * host.  Used by net_persist.c (WiFi build only).
 *
 * One record lives in a dedicated 4 KB flash sector (two sectors, A/B):
 *
 *   NpHeader (16 B)  magic 'DNPS' | version | length | seq | crc32
 *   NpPayload        flags | port | ssid[33] | pass[64] | host[64] | key[32]
 *
 * crc32 (IEEE, reflected 0xEDB88320, same as flash_storage.c) covers the
 * header bytes before the crc field plus the payload.  Little-endian layout
 * (RP2350 and every host we test on).
 */
#ifndef NET_PERSIST_CODEC_H
#define NET_PERSIST_CODEC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#define NP_MAGIC        0x53504E44u      /* 'D','N','P','S' little-endian */
#define NP_VERSION      1u

#define NP_FLAG_WIFI    0x01u            /* ssid/pass valid */
#define NP_FLAG_SERVER  0x02u            /* host/port valid */
#define NP_FLAG_KEY     0x04u            /* key valid */

#define NP_SSID_CAP     33
#define NP_PASS_CAP     64
#define NP_HOST_CAP     64

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t length;                     /* payload bytes */
    uint32_t seq;                        /* wrap-safe sequence, newest wins */
    uint32_t crc;
} NpHeader;

typedef struct __attribute__((packed)) {
    uint8_t  flags;
    uint8_t  reserved;                   /* 0 */
    uint16_t port;
    char     ssid[NP_SSID_CAP];
    char     pass[NP_PASS_CAP];
    char     host[NP_HOST_CAP];
    uint8_t  key[32];
} NpPayload;

#define NP_RECORD_BYTES (sizeof(NpHeader) + sizeof(NpPayload))

_Static_assert(sizeof(NpHeader) == 16, "NpHeader layout");
_Static_assert(sizeof(NpPayload) == 197, "NpPayload layout");

static inline uint32_t np_crc32_update(uint32_t crc, const uint8_t *d, size_t n) {
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int j = 0; j < 8; j++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
}

static inline uint32_t np_record_crc(const NpHeader *h, const NpPayload *p) {
    uint32_t crc = 0xFFFFFFFFu;
    crc = np_crc32_update(crc, (const uint8_t *)h, offsetof(NpHeader, crc));
    crc = np_crc32_update(crc, (const uint8_t *)p, sizeof(*p));
    return ~crc;
}

/* Encode into out (>= NP_RECORD_BYTES).  Returns bytes written, 0 on error.
 * Forces NUL termination of the string fields. */
static inline size_t np_encode(uint8_t *out, size_t out_cap, uint32_t seq,
                               const NpPayload *in) {
    if (!out || !in || out_cap < NP_RECORD_BYTES) return 0;
    NpPayload p = *in;
    p.reserved = 0;
    p.ssid[NP_SSID_CAP - 1] = '\0';
    p.pass[NP_PASS_CAP - 1] = '\0';
    p.host[NP_HOST_CAP - 1] = '\0';
    NpHeader h;
    h.magic = NP_MAGIC;
    h.version = (uint16_t)NP_VERSION;
    h.length = (uint16_t)sizeof(p);
    h.seq = seq;
    h.crc = np_record_crc(&h, &p);
    memcpy(out, &h, sizeof(h));
    memcpy(out + sizeof(h), &p, sizeof(p));
    return NP_RECORD_BYTES;
}

/* Validate and decode.  buf may be erased flash (0xFF) or garbage.
 * Returns true only for magic + version + exact length + CRC + sane strings. */
static inline bool np_decode(const uint8_t *buf, size_t len, uint32_t *seq_out,
                             NpPayload *out) {
    if (!buf || !out || len < NP_RECORD_BYTES) return false;
    NpHeader h;
    NpPayload p;
    memcpy(&h, buf, sizeof(h));
    if (h.magic != NP_MAGIC) return false;
    if (h.version != NP_VERSION) return false;
    if (h.length != sizeof(p)) return false;
    memcpy(&p, buf + sizeof(h), sizeof(p));
    if (np_record_crc(&h, &p) != h.crc) return false;
    if (memchr(p.ssid, 0, sizeof(p.ssid)) == NULL) return false;
    if (memchr(p.pass, 0, sizeof(p.pass)) == NULL) return false;
    if (memchr(p.host, 0, sizeof(p.host)) == NULL) return false;
    if (p.flags & ~(NP_FLAG_WIFI | NP_FLAG_SERVER | NP_FLAG_KEY)) return false;
    *out = p;
    if (seq_out) *seq_out = h.seq;
    return true;
}

/* Wrap-safe "a is newer than b". */
static inline bool np_seq_newer(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) > 0;
}

#endif /* NET_PERSIST_CODEC_H */
