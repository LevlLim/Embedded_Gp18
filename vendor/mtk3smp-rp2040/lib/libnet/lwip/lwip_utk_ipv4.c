/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Static IPv4, ARP and bounded raw-ICMP qualification for NO_SYS lwIP. */

#include <stddef.h>
#include <stdint.h>

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/etharp.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/ip.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/prot/icmp.h"
#include "lwip/prot/ip4.h"
#include "lwip/raw.h"
#include "lwip/sys.h"

#include "lwip_utk_ipv4.h"

#define UTK_PING_ID             0x554bu
#define UTK_PING_DATA_SIZE      16u
#define UTK_PING_INTERVAL_MS    1000u
#define UTK_PING_MAX_ATTEMPTS   5u
#define UTK_PING_FINAL_WAIT_MS  1000u

static T_LWIP_UTK_IPV4_STATUS ipv4_status = {
    .config_result = ERR_VAL,
    .ping_send_result = ERR_VAL,
    .ping_result = ERR_TIMEOUT,
};
static ip4_addr_t local_address;
static ip4_addr_t local_netmask;
static ip4_addr_t local_gateway;
static ip4_addr_t ping_target;
static ip4_addr_t arp_target;
static struct raw_pcb *ping_pcb;
static uint32_t ping_started_ms;
static uint32_t last_ping_ms;

static void copy_address(uint8_t dst[4], const ip4_addr_t *src)
{
    dst[0] = ip4_addr1(src);
    dst[1] = ip4_addr2(src);
    dst[2] = ip4_addr3(src);
    dst[3] = ip4_addr4(src);
}

static void refresh_arp_status(struct netif *netif)
{
    struct eth_addr *eth;
    const ip4_addr_t *ip;

    if(etharp_find_addr(netif, &arp_target, &eth, &ip) >= 0) {
        for(unsigned int i = 0; i < sizeof(ipv4_status.arp_mac); i++) {
            ipv4_status.arp_mac[i] = eth->addr[i];
        }
        ipv4_status.arp_resolved = 1;
    }
}

static uint8_t ping_recv(void *arg, struct raw_pcb *pcb, struct pbuf *p,
                         const ip_addr_t *source)
{
    struct icmp_echo_hdr echo;
    const struct ip_hdr *ip_header = ip4_current_header();
    uint16_t offset;

    (void)arg;
    (void)pcb;
    if(ip_header == NULL || source == NULL || p == NULL) return 0;
    offset = IPH_HL_BYTES(ip_header);
    if(p->tot_len < offset + sizeof(echo)
       || pbuf_copy_partial(p, &echo, sizeof(echo), offset) != sizeof(echo)) {
        return 0;
    }
    if(ICMPH_TYPE(&echo) != ICMP_ER || ICMPH_CODE(&echo) != 0
       || echo.id != UTK_PING_ID
       || !ip4_addr_eq(ip_2_ip4(source), &ping_target)) {
        return 0;
    }

    ipv4_status.ping_result = ERR_OK;
    ipv4_status.ping_complete = 1;
    ipv4_status.ping_elapsed_ms = sys_now() - ping_started_ms;
    pbuf_free(p);
    return 1;
}

static int32_t send_ping(void)
{
    const uint16_t size = sizeof(struct icmp_echo_hdr) + UTK_PING_DATA_SIZE;
    struct icmp_echo_hdr *echo;
    struct pbuf *p = pbuf_alloc(PBUF_IP, size, PBUF_RAM);
    err_t result;

    if(p == NULL) return ERR_MEM;
    if(p->len != p->tot_len || p->next != NULL) {
        pbuf_free(p);
        return ERR_MEM;
    }

    echo = (struct icmp_echo_hdr *)p->payload;
    ICMPH_TYPE_SET(echo, ICMP_ECHO);
    ICMPH_CODE_SET(echo, 0);
    echo->chksum = 0;
    echo->id = UTK_PING_ID;
    echo->seqno = lwip_htons((uint16_t)(ipv4_status.ping_attempts + 1));
    for(uint16_t i = sizeof(*echo); i < size; i++) {
        ((uint8_t *)echo)[i] = (uint8_t)i;
    }
    echo->chksum = inet_chksum(echo, size);
    result = raw_sendto(ping_pcb, p, (const ip_addr_t *)&ping_target);
    pbuf_free(p);
    return result;
}

int32_t lwip_utk_ipv4_configure(struct netif *netif,
        const char *address, const char *netmask, const char *gateway,
        const char *target)
{
    if(netif == NULL || !ip4addr_aton(address, &local_address)
       || !ip4addr_aton(netmask, &local_netmask)
       || !ip4addr_aton(gateway, &local_gateway)
       || !ip4addr_aton(target, &ping_target)
       || ip4_addr_isany_val(local_address)
       || ip4_addr_isany_val(local_netmask)
       || ip4_addr_isany_val(ping_target)) {
        ipv4_status.config_result = ERR_ARG;
        return ERR_ARG;
    }

    netif_set_addr(netif, &local_address, &local_netmask, &local_gateway);
    arp_target = ip4_addr_netcmp(&ping_target, &local_address, &local_netmask)
               ? ping_target : local_gateway;
    if(ip4_addr_isany_val(arp_target)) {
        ipv4_status.config_result = ERR_RTE;
        return ERR_RTE;
    }

    copy_address(ipv4_status.address, &local_address);
    copy_address(ipv4_status.netmask, &local_netmask);
    copy_address(ipv4_status.gateway, &local_gateway);
    copy_address(ipv4_status.target, &ping_target);
    ipv4_status.config_result = ERR_OK;
    ipv4_status.configured = 1;
    return ERR_OK;
}

int32_t lwip_utk_ipv4_adopt_netif(struct netif *netif)
{
    if(netif == NULL || ip4_addr_isany_val(*netif_ip4_addr(netif))
       || ip4_addr_isany_val(*netif_ip4_netmask(netif))
       || ip4_addr_isany_val(*netif_ip4_gw(netif))) {
        ipv4_status.config_result = ERR_ARG;
        return ERR_ARG;
    }

    local_address = *netif_ip4_addr(netif);
    local_netmask = *netif_ip4_netmask(netif);
    local_gateway = *netif_ip4_gw(netif);
    ping_target = local_gateway;
    arp_target = local_gateway;

    copy_address(ipv4_status.address, &local_address);
    copy_address(ipv4_status.netmask, &local_netmask);
    copy_address(ipv4_status.gateway, &local_gateway);
    copy_address(ipv4_status.target, &ping_target);
    ipv4_status.config_result = ERR_OK;
    ipv4_status.configured = 1;
    return ERR_OK;
}

void lwip_utk_ipv4_poll(struct netif *netif)
{
    uint32_t now;

    if(!ipv4_status.configured
       || !netif_is_up(netif) || !netif_is_link_up(netif)) return;

    now = sys_now();
    refresh_arp_status(netif);
    if(ipv4_status.ping_complete) return;
    if(!ipv4_status.ping_started) {
        ping_pcb = raw_new(IP_PROTO_ICMP);
        if(ping_pcb == NULL) {
            ipv4_status.ping_result = ERR_MEM;
            ipv4_status.ping_complete = 1;
            return;
        }
        raw_recv(ping_pcb, ping_recv, NULL);
        raw_bind_netif(ping_pcb, netif);
        if(raw_bind(ping_pcb, IP_ADDR_ANY) != ERR_OK) {
            raw_remove(ping_pcb);
            ping_pcb = NULL;
            ipv4_status.ping_result = ERR_USE;
            ipv4_status.ping_complete = 1;
            return;
        }
        ipv4_status.ping_started = 1;
        ping_started_ms = now;
        last_ping_ms = now - UTK_PING_INTERVAL_MS;
        (void)etharp_request(netif, &arp_target);
    }

    if(ipv4_status.ping_attempts < UTK_PING_MAX_ATTEMPTS
       && now - last_ping_ms >= UTK_PING_INTERVAL_MS) {
        ipv4_status.ping_send_result = send_ping();
        ipv4_status.ping_attempts++;
        last_ping_ms = now;
    } else if(ipv4_status.ping_attempts >= UTK_PING_MAX_ATTEMPTS
              && now - last_ping_ms >= UTK_PING_FINAL_WAIT_MS) {
        ipv4_status.ping_result = ERR_TIMEOUT;
        ipv4_status.ping_elapsed_ms = now - ping_started_ms;
        ipv4_status.ping_complete = 1;
    }
}

void lwip_utk_ipv4_get_status(T_LWIP_UTK_IPV4_STATUS *status)
{
    if(status != NULL) *status = ipv4_status;
}
