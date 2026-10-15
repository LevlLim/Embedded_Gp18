/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LWIP_UTK_TCP_H
#define LWIP_UTK_TCP_H

#include <stdint.h>

#define LWIP_UTK_TCP_RECORD_COUNT  64U
#define LWIP_UTK_TCP_RECORD_SIZE   48U

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
    uint32_t expected_records;
    uint32_t record_size;
    uint32_t records_sent;
    uint32_t records_matched;
    uint32_t bytes_sent;
    uint32_t bytes_acked;
    uint32_t bytes_received;
    uint32_t segments_received;
    uint32_t short_segments;
    uint32_t corrupt;
    uint32_t send_errors;
    uint32_t poll_calls;
    uint32_t guard_blocked;
    uint32_t start_delay_ms;
    uint32_t connect_ms;
    uint32_t close_ms;
    uint32_t elapsed_ms;
    uint8_t target[4];
    uint16_t port;
} T_LWIP_UTK_TCP_STATUS;

void lwip_utk_tcp_poll(struct netif *netif);
void lwip_utk_tcp_get_status(T_LWIP_UTK_TCP_STATUS *status);

#endif
