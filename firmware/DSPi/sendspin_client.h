/*
 * sendspin_client.h - minimal Sendspin (Music Assistant) PLAYER client.
 *
 * Protocol layer only: ws:// WebSocket client over lwIP raw TCP, mandatory
 * Noise KKpsk2 encryption (sendspin_noise.c), JSON control, PCM audio chunks.
 * Everything runs from the polling context (Core 0 main loop); no threads,
 * no lwIP locking. Call sendspin_client_poll() every main-loop pass after
 * cyw43_arch_poll() (pico_cyw43_arch_lwip_poll).
 *
 * Uses (provided by net_input.h):
 *   net_input_ring_capacity / net_input_ring_free_bytes / net_input_stage_pcm /
 *   net_input_commit_pcm / net_input_set_format / net_input_flush /
 *   net_input_ring_fill_bytes / net_volume_get_percent / net_volume_set_percent /
 *   net_mute_get / net_mute_set / net_sendspin_host / net_sendspin_port
 */
#ifndef SENDSPIN_CLIENT_H
#define SENDSPIN_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SENDSPIN_STATE_IDLE = 0,     /* not started / stopped */
    SENDSPIN_STATE_CONNECTING,   /* DNS + TCP + WebSocket upgrade */
    SENDSPIN_STATE_HANDSHAKE,    /* client/init .. Noise handshake */
    SENDSPIN_STATE_SYNCING,      /* encrypted, hello sent, time-sync burst */
    SENDSPIN_STATE_READY,        /* client/state(available) sent, waiting for stream */
    SENDSPIN_STATE_STREAMING,    /* stream/start received, pushing PCM */
    SENDSPIN_STATE_ERROR         /* failed; waiting for reconnect back-off */
};

void    sendspin_client_start(void);
void    sendspin_client_stop(void);
void    sendspin_client_poll(void);
bool    sendspin_client_streaming(void);
uint8_t sendspin_client_state(void);

/*
 * RAM loan for the Slimproto client (slimproto_client.c): the whole static state
 * of this client (about 5.4 KB) is one object.  While Sendspin is stopped it can
 * be borrowed (NULL if the client is running or already loaned); the client then
 * refuses to start until sendspin_client_mem_return().  A borrowed object is
 * zeroed again at the next sendspin_client_start().  Only the protocol selector
 * (net_input.c) may switch between the two clients, always stop one before
 * starting the other.
 */
void *sendspin_client_mem_loan(uint32_t *bytes);
void  sendspin_client_mem_return(void);

/*
 * Optional persistence hooks (weak defaults in sendspin_client.c do nothing).
 * Override in flash code to keep the X25519 static identity across reboots
 * (client_id = base64url(pubkey), and MA remembers it). Phase 1 generates a
 * fresh key every boot from pico_rand.
 */
bool sendspin_key_load(uint8_t priv[32]);          /* true if priv was loaded */
void sendspin_key_store(const uint8_t priv[32]);   /* called after fresh keygen */

/* Tunables (override with -D) */
#ifndef SENDSPIN_CLIENT_NAME
#define SENDSPIN_CLIENT_NAME "DSPi"
#endif
/* buffer_capacity advertised in client/hello = net_input_ring_capacity(), i.e.
 * exactly the PCM ring (32 KB; the server may send audio chunks of up to that
 * size, e.g. 150 ms = 28.8 KB at 48 kHz/16-bit stereo, which are decrypted
 * directly into the ring).  Define SENDSPIN_BUFFER_CAPACITY to advertise less. */

#ifdef __cplusplus
}
#endif
#endif
