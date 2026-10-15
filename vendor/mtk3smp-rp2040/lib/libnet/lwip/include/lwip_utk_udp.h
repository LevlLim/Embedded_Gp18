/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LWIP_UTK_UDP_H
#define LWIP_UTK_UDP_H

#include <stdint.h>

#define LWIP_UTK_UDP_PACKET_COUNT  64U
#define LWIP_UTK_UDP_PAYLOAD_SIZE  48U
#define LWIP_UTK_UDP_RING          32U

struct netif;

typedef struct {
    int32_t init_result;
    uint32_t initialized;
    uint32_t started;
    uint32_t complete;
    uint32_t closed;
    int32_t send_result;
    int32_t result;
    uint32_t expected_packets;
    uint32_t payload_size;
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_matched;
    uint32_t retries;
    uint32_t send_errors;
    uint32_t corrupt;
    uint32_t unexpected;
    uint32_t elapsed_ms;
    uint8_t target[4];
    uint16_t port;
    /* Ring of recently matched echoes, so the application can print each
       message rather than whichever one happened to be current when it last
       looked.  The exchange runs at roughly 20 ms per packet, far faster
       than any sane reporting interval, so a single "last" slot loses most
       of them.  Producer is the lwIP callback, consumer is the application;
       it only ever advances write_seq, so a reader that falls behind loses
       the oldest entries rather than seeing a torn one. */
    uint8_t ring_payload[LWIP_UTK_UDP_RING][LWIP_UTK_UDP_PAYLOAD_SIZE];
    uint32_t ring_seq[LWIP_UTK_UDP_RING];
    uint16_t ring_len[LWIP_UTK_UDP_RING];
    uint32_t write_seq;		/* total entries ever written */
    uint32_t session_count;	/* runs started, from 0 */
} T_LWIP_UTK_UDP_STATUS;

void lwip_utk_udp_poll(struct netif *netif);
void lwip_utk_udp_get_status(T_LWIP_UTK_UDP_STATUS *status);

#endif
