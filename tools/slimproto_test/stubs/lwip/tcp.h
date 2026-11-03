/* Host-test stub of the lwIP raw TCP API subset used by slimproto_client.c and
 * sendspin_client.c.  Unlike tools/sendspin_test's stub, struct tcp_pcb is
 * opaque here: test_slimproto.c implements a multi-pcb fake network. */
#ifndef STUB_LWIP_TCP_H
#define STUB_LWIP_TCP_H
#include <stdint.h>
#include <stddef.h>
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"

typedef uint16_t u16_t;
typedef int8_t err_t;
#define ERR_OK 0
#define ERR_MEM (-1)
#define ERR_INPROGRESS (-5)
#define ERR_ABRT (-13)
#define ERR_RST (-14)
#define TCP_WRITE_FLAG_COPY 0x01

struct tcp_pcb;

typedef err_t (*tcp_recv_fn)(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
typedef err_t (*tcp_sent_fn)(void *arg, struct tcp_pcb *pcb, u16_t len);
typedef void (*tcp_err_fn)(void *arg, err_t err);
typedef err_t (*tcp_connected_fn)(void *arg, struct tcp_pcb *pcb, err_t err);

struct tcp_pcb *tcp_new(void);
void tcp_arg(struct tcp_pcb *pcb, void *arg);
void tcp_recv(struct tcp_pcb *pcb, tcp_recv_fn fn);
void tcp_sent(struct tcp_pcb *pcb, tcp_sent_fn fn);
void tcp_err(struct tcp_pcb *pcb, tcp_err_fn fn);
err_t tcp_connect(struct tcp_pcb *pcb, const ip_addr_t *ip, uint16_t port, tcp_connected_fn fn);
err_t tcp_write(struct tcp_pcb *pcb, const void *data, u16_t len, uint8_t flags);
err_t tcp_output(struct tcp_pcb *pcb);
u16_t tcp_sndbuf(struct tcp_pcb *pcb);
void tcp_recved(struct tcp_pcb *pcb, u16_t len);
err_t tcp_close(struct tcp_pcb *pcb);
void tcp_abort(struct tcp_pcb *pcb);
#define tcp_nagle_disable(pcb) ((void)(pcb))
#endif
