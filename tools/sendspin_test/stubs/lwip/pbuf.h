#ifndef STUB_LWIP_PBUF_H
#define STUB_LWIP_PBUF_H
#include <stdint.h>
struct pbuf {
    struct pbuf *next;
    void *payload;
    uint16_t tot_len;
    uint16_t len;
    int ref;
};
void pbuf_free(struct pbuf *p);
void pbuf_cat(struct pbuf *head, struct pbuf *tail);
#endif
