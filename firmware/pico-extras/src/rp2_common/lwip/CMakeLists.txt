set(LWIP_TEST_PATH "src/core/tcp.c")
if (NOT LWIP_PATH)
    set(LWIP_PATH ${PICO_EXTRAS_PATH}/lib/lwip)
    if (NOT EXISTS ${LWIP_PATH}/${LWIP_TEST_PATH})
        message(WARNING "lwIP submodule has not been initialized; TCP/IP support will be unavailable
         hint: try 'git submodule update --init'.")
    endif()
elseif (NOT EXISTS ${LWIP_PATH}/${LWIP_TEST_PATH})
    message(WARNING "LWIP_PATH specified but content not present.")
endif()

if (EXISTS ${LWIP_PATH}/${LWIP_TEST_PATH})
    message("lwIP available at ${LWIP_PATH}/${LWIP_TEST_PATH}; TCP/IP support is available.")

    add_library(lwip INTERFACE)
    target_sources(lwip INTERFACE
        ${LWIP_PATH}/src/core/altcp.c
        ${LWIP_PATH}/src/core/altcp_alloc.c
        ${LWIP_PATH}/src/core/altcp_tcp.c
        ${LWIP_PATH}/src/core/def.c
        ${LWIP_PATH}/src/core/dns.c
        ${LWIP_PATH}/src/core/inet_chksum.c
        ${LWIP_PATH}/src/core/init.c
        ${LWIP_PATH}/src/core/ip.c
        ${LWIP_PATH}/src/core/mem.c
        ${LWIP_PATH}/src/core/memp.c
        ${LWIP_PATH}/src/core/netif.c
        ${LWIP_PATH}/src/core/pbuf.c
        ${LWIP_PATH}/src/core/raw.c
        ${LWIP_PATH}/src/core/stats.c
        ${LWIP_PATH}/src/core/sys.c
        ${LWIP_PATH}/src/core/tcp.c
        ${LWIP_PATH}/src/core/tcp_in.c
        ${LWIP_PATH}/src/core/tcp_out.c
        ${LWIP_PATH}/src/core/timeouts.c
        ${LWIP_PATH}/src/core/udp.c
        ${LWIP_PATH}/src/core/ipv4/autoip.c
        ${LWIP_PATH}/src/core/ipv4/dhcp.c
        ${LWIP_PATH}/src/core/ipv4/etharp.c
        ${LWIP_PATH}/src/core/ipv4/icmp.c
        ${LWIP_PATH}/src/core/ipv4/igmp.c
        ${LWIP_PATH}/src/core/ipv4/ip4.c
        ${LWIP_PATH}/src/core/ipv4/ip4_addr.c
        ${LWIP_PATH}/src/core/ipv4/ip4_frag.c
        ${LWIP_PATH}/src/netif/ethernet.c
        ${LWIP_PATH}/src/netif/slipif.c
        ${LWIP_PATH}/src/apps/http/httpd.c
        ${LWIP_PATH}/src/apps/http/fs.c

        ${CMAKE_CURRENT_LIST_DIR}/lwip_arch.c
    )

    target_include_directories(lwip INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${LWIP_PATH}/src/include
        ${LWIP_PATH}/src/include/ipv4
        ${LWIP_PATH}/src/include/lwip/apps
    )
endif()
