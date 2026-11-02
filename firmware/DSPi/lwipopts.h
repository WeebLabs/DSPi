/*
 * lwipopts.h - lwIP configuration for the DSPi WiFi/Sendspin input
 * (-DDSPI_ENABLE_WIFI=ON, RP2350 + Pico 2 W).
 *
 * NO_SYS=1 raw API in "poll" mode: lwIP only runs from cyw43_arch_poll(),
 * called by net_input_poll() on Core 0.  The memory pools are deliberately
 * small: one TCP connection (Sendspin) plus DHCP/DNS.  RAM is the constraint
 * (.bss + malloc heap must fit below the stacks): lwIP heap 4 KB + pbuf pool
 * 6 x 1.5 KB + ~1.5 KB of memp.
 *
 * Throughput target: 48 kHz / 16-bit stereo = 192 KB/s (24-bit: 288 KB/s);
 * TCP_WND = 4 * MSS = 5.8 KB sustains > 700 KB/s at 8 ms RTT (it only
 * starves the stream if the WiFi round trip exceeds ~30 ms).
 */

#ifndef DSPI_LWIPOPTS_H
#define DSPI_LWIPOPTS_H

#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define LWIP_TIMEVAL_PRIVATE        0

// Memory
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4096
#define MEMP_NUM_TCP_SEG            16
#define MEMP_NUM_ARP_QUEUE          4
#define PBUF_POOL_SIZE              6   // >= TCP_WND/MSS (4) + ACK/ARP slack

// Protocols
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    0
#define LWIP_UDP                    1
#define LWIP_TCP                    1
#define LWIP_DHCP                   1
#define LWIP_DNS                    1
#define LWIP_AUTOIP                 0
#define LWIP_IGMP                   0
#define LWIP_ALTCP                  0
#define LWIP_ALTCP_TLS              0
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_STATUS_CALLBACK  0
#define LWIP_NETIF_LINK_CALLBACK    0
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0

// Pools: Sendspin TCP + a spare, DHCP + DNS (+ spare) UDP
#define MEMP_NUM_TCP_PCB            3
#define MEMP_NUM_TCP_PCB_LISTEN     1
#define MEMP_NUM_UDP_PCB            3
#define MEMP_NUM_SYS_TIMEOUT        10

// TCP
#define TCP_MSS                     1460
#define TCP_WND                     (4 * TCP_MSS)
#define TCP_SND_BUF                 (2 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define TCP_QUEUE_OOSEQ             0   // OOO segments would pin scarce pool pbufs
#define LWIP_TCP_KEEPALIVE          1

// Checksums / misc
#define LWIP_CHKSUM_ALGORITHM       3
#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY          0
#define LWIP_PROVIDE_ERRNO          1

#endif // DSPI_LWIPOPTS_H
