/*
 * lwipopts.h - lwIP configuration for the DSPi WiFi/Sendspin input
 * (-DDSPI_ENABLE_WIFI=ON, RP2350 + Pico 2 W).
 *
 * NO_SYS=1 raw API in "poll" mode: lwIP only runs from cyw43_arch_poll(),
 * called by net_input_poll() on Core 0.  The memory pools are deliberately
 * small: one TCP connection (Sendspin) plus DHCP/DNS.
 *
 * RAM is THE constraint, and the number that matters is the malloc heap
 * (__end__ .. __HeapLimit): usb_sound_card_init() takes ~51 KB of it at boot for
 * the four S/PDIF producer pools (4 x AUDIO_BUFFER_COUNT x 1.5 KB), and a
 * failed calloc panics.  Every static byte below therefore comes straight out
 * of that budget (see the DSPI_ENABLE_WIFI heap report in CMakeLists.txt).
 * Static cost of this configuration (from the map file):
 *   pbuf pool 6 x 1.1 KB + lwIP heap 4 KB + pcb/seg/timeout pools ~1 KB
 *   + DNS 0.2 KB + ARP 0.1 KB.
 *
 * Throughput target: 48 kHz / 16-bit stereo = 192 KB/s (24-bit: 288 KB/s).
 * MSS 1024 (announced to the server in the SYN) keeps one pool pbuf at ~1.1 KB
 * instead of ~1.5 KB; TCP_WND = 4 * MSS = 4 KB then sustains > 500 KB/s up to
 * an 8 ms round trip, and still 192 KB/s up to ~21 ms (WiFi RTT is normally
 * 2-10 ms; the 170 ms PCM ring absorbs excursions).  The window is also all
 * that can be pinned in pbufs while the PCM ring is full, so
 * PBUF_POOL_SIZE = window/MSS (4) + 2 for ACK/ARP/DHCP slack.
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
#define MEM_SIZE                    4096    // tcp_write copies (<= 2 x ~1.1 KB) + DHCP/DNS/ARP frames
#define MEMP_NUM_TCP_SEG            16
#define MEMP_NUM_ARP_QUEUE          4
#define PBUF_POOL_SIZE              6   // >= TCP_WND/MSS (4) + ACK/ARP slack
#define IP_REASSEMBLY               0   // no fragments expected; saves the reass pool and pinned pbufs
#define IP_FRAG                     0
#define ARP_TABLE_SIZE              4   // gateway + server + spare
#define DNS_TABLE_SIZE              2
#define DNS_MAX_NAME_LENGTH         64  // net_host is <= 63 chars; default 256 B/entry
#define DNS_MAX_REQUESTS            2

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
#define TCP_MSS                     1024
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
