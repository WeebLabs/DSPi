#ifndef STUB_LWIP_DNS_H
#define STUB_LWIP_DNS_H
#include "lwip/tcp.h"
typedef void (*dns_found_callback)(const char *name, const ip_addr_t *ipaddr, void *arg);
err_t dns_gethostbyname(const char *hostname, ip_addr_t *addr, dns_found_callback found, void *arg);
#endif
