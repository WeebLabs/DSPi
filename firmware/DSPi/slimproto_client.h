/*
 * slimproto_client.h - Squeezelite-compatible (Slimproto) PLAYER client for
 * Music Assistant's "Squeezelite/Slimproto" provider (also works against LMS).
 *
 * Second, mutually exclusive network protocol next to Sendspin (net_input.c
 * runs exactly one of them; see net_proto_set()).  lwIP raw API, everything
 * runs from slimproto_client_poll() on the Core 0 main loop; no threads, no
 * heap.  Two TCP connections:
 *   - the Slimproto control connection to <server>:3483 (the player connects
 *     OUT), carrying HELO / STAT / RESP / DSCO / SETD to the server and
 *     strm / cont / codc / aude / audg / setd / serv / vers from it;
 *   - the HTTP stream connection the server asks for with `strm s` (ip, port
 *     and the verbatim HTTP request are in that command).  The body is decoded
 *     into the NET PCM ring: raw PCM (S16/S24, LE/BE, also WAV-wrapped) or a
 *     native FLAC stream (net_flac.c byte-stream mode).
 * Only `flc` and `pcm` are advertised in HELO, so the server transcodes anything
 * else (mp3/aac/ogg are never requested and refused with STMn if they are).
 *
 * RAM: no static state of its own beyond a pointer; the ~3 KB state/buffers are
 * BORROWED from the idle Sendspin client (sendspin_client_mem_loan(), 5.3 KB).
 *
 * Volume: `audg` (16.16 linear gain) is mapped onto net_volume_set_percent() by
 * the inverse of the Sendspin curve (pct = 100 * gain^(2/3)).  Slimproto has no
 * client-to-server volume message, so a local (USB/I2C/UART) volume change
 * cannot be reported back; the next `audg` from the server overrides it.
 *
 * Uses (net_input.h): net_input_ring_* / stage / commit / set_format / flush /
 * wr_pos / rd_pos / set_hold, net_volume_*, net_mute_*, net_input_get_mac,
 * net_sendspin_host (shared server host), net_slimproto_port.
 */
#ifndef SLIMPROTO_CLIENT_H
#define SLIMPROTO_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Same numeric meaning as SENDSPIN_STATE_* (NetStatusPacket.sendspin_state). */
enum {
    SLIMPROTO_STATE_IDLE = 0,       /* not started / stopped */
    SLIMPROTO_STATE_CONNECTING = 1, /* DNS + TCP to the server */
    SLIMPROTO_STATE_HANDSHAKE = 2,  /* HELO sent, waiting for the first server message */
    SLIMPROTO_STATE_READY = 4,      /* registered; no stream open */
    SLIMPROTO_STATE_STREAMING = 5,  /* an HTTP stream is open / being decoded */
    SLIMPROTO_STATE_ERROR = 6       /* failed; waiting for reconnect back-off */
};

void    slimproto_client_start(void);
void    slimproto_client_stop(void);    /* returns the borrowed RAM, closes both pcbs, flushes the ring */
void    slimproto_client_poll(void);
bool    slimproto_client_streaming(void);
uint8_t slimproto_client_state(void);

#ifndef SLIMPROTO_PLAYER_NAME
#define SLIMPROTO_PLAYER_NAME "DSPi"
#endif

#ifdef __cplusplus
}
#endif
#endif
