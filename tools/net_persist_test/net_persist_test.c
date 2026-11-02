/* Host test for firmware/DSPi/net_persist_codec.h.
 * Build: gcc -std=gnu11 -Wall -Wextra -I../../firmware/DSPi net_persist_test.c -o net_persist_test
 */
#include "net_persist_codec.h"
#include <stdio.h>
#include <stdlib.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main(void) {
    NpPayload in, out;
    uint8_t buf[4096];
    uint32_t seq = 0;

    /* known CRC32 vector: "123456789" -> 0xCBF43926 */
    CHECK((~np_crc32_update(0xFFFFFFFFu, (const uint8_t *)"123456789", 9)) == 0xCBF43926u);

    memset(&in, 0, sizeof(in));
    in.flags = NP_FLAG_WIFI | NP_FLAG_SERVER | NP_FLAG_KEY;
    in.port = 8927;
    strcpy(in.ssid, "MyNetwork");
    strcpy(in.pass, "correct horse battery staple");
    strcpy(in.host, "192.168.1.50");
    for (int i = 0; i < 32; i++) in.key[i] = (uint8_t)(i * 7 + 1);

    /* round trip inside an erased (0xFF) sector */
    memset(buf, 0xFF, sizeof(buf));
    CHECK(np_decode(buf, sizeof(buf), &seq, &out) == false);   /* erased flash */
    CHECK(np_encode(buf, sizeof(buf), 41, &in) == NP_RECORD_BYTES);
    CHECK(np_decode(buf, sizeof(buf), &seq, &out));
    CHECK(seq == 41);
    CHECK(memcmp(&in, &out, sizeof(in)) == 0);

    /* too small buffers */
    CHECK(np_encode(buf, NP_RECORD_BYTES - 1, 1, &in) == 0);
    CHECK(np_decode(buf, NP_RECORD_BYTES - 1, &seq, &out) == false);

    /* every single-bit flip anywhere in the record must be rejected */
    uint8_t good[NP_RECORD_BYTES];
    np_encode(good, sizeof(good), 7, &in);
    for (size_t b = 0; b < NP_RECORD_BYTES * 8; b++) {
        uint8_t t[NP_RECORD_BYTES];
        memcpy(t, good, sizeof(t));
        t[b / 8] ^= (uint8_t)(1u << (b % 8));
        CHECK(np_decode(t, sizeof(t), &seq, &out) == false);
    }

    /* truncation of the program (torn write): tail erased to 0xFF */
    for (size_t keep = 0; keep < NP_RECORD_BYTES; keep += 5) {
        uint8_t t[4096];
        memset(t, 0xFF, sizeof(t));
        memcpy(t, good, keep);
        CHECK(np_decode(t, sizeof(t), &seq, &out) == false);
    }

    /* unterminated strings are forced terminated on encode */
    memset(&in, 'A', sizeof(in));
    in.flags = NP_FLAG_WIFI;
    np_encode(buf, sizeof(buf), 3, &in);
    CHECK(np_decode(buf, sizeof(buf), &seq, &out));
    CHECK(out.ssid[NP_SSID_CAP - 1] == 0 && out.pass[NP_PASS_CAP - 1] == 0 &&
          out.host[NP_HOST_CAP - 1] == 0);

    /* bad version / length with a recomputed valid CRC must still fail */
    np_encode(buf, sizeof(buf), 3, &in);
    NpHeader h; memcpy(&h, buf, sizeof(h));
    h.version = 2;
    { NpPayload p; memcpy(&p, buf + sizeof(h), sizeof(p)); h.crc = np_record_crc(&h, &p); }
    memcpy(buf, &h, sizeof(h));
    CHECK(np_decode(buf, sizeof(buf), &seq, &out) == false);

    /* sequence compare, including wrap */
    CHECK(np_seq_newer(2, 1));
    CHECK(!np_seq_newer(1, 2));
    CHECK(!np_seq_newer(5, 5));
    CHECK(np_seq_newer(0, 0xFFFFFFFFu));

    printf(fails ? "net_persist_test: %d FAILURE(S)\n" : "net_persist_test: all passed\n", fails);
    return fails ? 1 : 0;
}
