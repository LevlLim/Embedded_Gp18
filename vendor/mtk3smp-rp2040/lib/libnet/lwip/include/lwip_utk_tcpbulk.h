/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LWIP_UTK_TCPBULK_H
#define LWIP_UTK_TCPBULK_H

#include <stdint.h>

#define LWIP_UTK_TCPBULK_TOTAL_BYTES  32768U

struct netif;

typedef struct {
    int32_t init_result;
    int32_t connect_result;
    int32_t send_result;
    int32_t close_result;
    int32_t result;
    int32_t fatal_error;
    uint32_t initialized;
    uint32_t started;
    uint32_t connected;
    uint32_t complete;
    uint32_t fin_sent;
    uint32_t peer_fin;
    uint32_t closed;
    uint32_t aborted;
    uint32_t total_bytes;
    uint32_t bytes_sent;
    uint32_t bytes_acked;
    uint32_t bytes_received;
    uint32_t mismatches;
    uint32_t first_mismatch;
    uint32_t segments_received;
    uint32_t max_segment;
    uint32_t min_segment;
    uint32_t split_boundaries;
    uint32_t write_retries;
    uint32_t connect_ms;
    uint32_t close_ms;
    uint32_t elapsed_ms;
    uint32_t throughput_bps;
    uint8_t target[4];
    uint16_t port;
} T_LWIP_UTK_TCPBULK_STATUS;

void lwip_utk_tcpbulk_poll(struct netif *netif);
void lwip_utk_tcpbulk_get_status(T_LWIP_UTK_TCPBULK_STATUS *status);

#endif
