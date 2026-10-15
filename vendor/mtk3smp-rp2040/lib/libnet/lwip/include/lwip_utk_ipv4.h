/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LWIP_UTK_IPV4_H
#define LWIP_UTK_IPV4_H

#include <stdint.h>

struct netif;

typedef struct {
    int32_t config_result;
    uint32_t configured;
    uint32_t arp_resolved;
    uint32_t ping_started;
    uint32_t ping_complete;
    int32_t ping_send_result;
    int32_t ping_result;
    uint32_t ping_attempts;
    uint32_t ping_elapsed_ms;
    uint8_t address[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t target[4];
    uint8_t arp_mac[6];
} T_LWIP_UTK_IPV4_STATUS;

int32_t lwip_utk_ipv4_configure(struct netif *netif,
        const char *address, const char *netmask, const char *gateway,
        const char *target);
int32_t lwip_utk_ipv4_adopt_netif(struct netif *netif);
void lwip_utk_ipv4_poll(struct netif *netif);
void lwip_utk_ipv4_get_status(T_LWIP_UTK_IPV4_STATUS *status);

#endif
