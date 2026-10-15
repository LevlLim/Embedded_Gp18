/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Bounded raw-TCP echo qualification for the NO_SYS lwIP owner task. */

#include <stddef.h>
#include <stdint.h>

#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"

#include "lwip_utk_tcp.h"
#include "tcp_test_config.h"

#define UTK_TCP_CONNECT_TIMEOUT_MS  10000U
#define UTK_TCP_ECHO_TIMEOUT_MS      2000U
#define UTK_TCP_CLOSE_TIMEOUT_MS     5000U
#define UTK_TCP_SESSION_TIMEOUT_MS  30000U

_Static_assert(UTK_TCP_ECHO_PORT > 0U && UTK_TCP_ECHO_PORT <= 65535U,
               "UTK_TCP_ECHO_PORT must be in the range 1..65535");

static T_LWIP_UTK_TCP_STATUS tcp_status = {
    .init_result = ERR_VAL,
    .connect_result = ERR_VAL,
    .send_result = ERR_VAL,
    .close_result = ERR_VAL,
    .result = ERR_TIMEOUT,
    .fatal_error = ERR_OK,
    .expected_records = LWIP_UTK_TCP_RECORD_COUNT,
    .record_size = LWIP_UTK_TCP_RECORD_SIZE,
    .port = (uint16_t)UTK_TCP_ECHO_PORT,
};
static struct tcp_pcb *tcp_pcb_handle;
static ip4_addr_t tcp_target;
static uint16_t record;
static uint32_t awaiting_echo;
static uint32_t rx_fill;
static uint8_t rx_buffer[LWIP_UTK_TCP_RECORD_SIZE];
static uint32_t started_ms;
static uint32_t connect_started_ms;
static uint32_t last_send_ms;
static uint32_t close_started_ms;

static void copy_address(uint8_t dst[4], const ip4_addr_t *src)
{
    dst[0] = ip4_addr1(src);
    dst[1] = ip4_addr2(src);
    dst[2] = ip4_addr3(src);
    dst[3] = ip4_addr4(src);
}

/* Same construction as the Phase-6 UDP payload, tagged for this phase so a
 * stray datagram from the UDP peer can never satisfy a TCP record. */
static void make_record(uint8_t payload[LWIP_UTK_TCP_RECORD_SIZE],
                        uint16_t value)
{
    uint32_t hash = 2166136261U;
    unsigned int i;

    payload[0] = 'U';
    payload[1] = 'T';
    payload[2] = 'K';
    payload[3] = '7';
    payload[4] = (uint8_t)(value >> 8);
    payload[5] = (uint8_t)value;
    payload[6] = (uint8_t)LWIP_UTK_TCP_RECORD_COUNT;
    payload[7] = (uint8_t)LWIP_UTK_TCP_RECORD_SIZE;
    for(i = 8; i < LWIP_UTK_TCP_RECORD_SIZE - sizeof(hash); i++) {
        payload[i] = (uint8_t)(0x5aU + value * 29U + i * 17U);
    }
    for(i = 0; i < LWIP_UTK_TCP_RECORD_SIZE - sizeof(hash); i++) {
        hash ^= payload[i];
        hash *= 16777619U;
    }
    payload[LWIP_UTK_TCP_RECORD_SIZE - 4U] = (uint8_t)(hash >> 24);
    payload[LWIP_UTK_TCP_RECORD_SIZE - 3U] = (uint8_t)(hash >> 16);
    payload[LWIP_UTK_TCP_RECORD_SIZE - 2U] = (uint8_t)(hash >> 8);
    payload[LWIP_UTK_TCP_RECORD_SIZE - 1U] = (uint8_t)hash;
}

static int record_equal(const uint8_t *left, const uint8_t *right,
                        unsigned int size)
{
    unsigned int i;

    for(i = 0; i < size; i++) {
        if(left[i] != right[i]) return 0;
    }
    return 1;
}

static void finish(int32_t result, uint32_t now)
{
    tcp_status.result = result;
    tcp_status.elapsed_ms = now - started_ms;
    tcp_status.complete = 1;
    awaiting_echo = 0;
}

static void detach_callbacks(struct tcp_pcb *pcb)
{
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 0);
}

/* The error callback is the one path where lwIP has already freed the pcb;
 * dropping the handle first keeps every later cleanup from touching it. */
static void tcp_echo_err(void *arg, err_t err)
{
    uint32_t now = sys_now();

    (void)arg;
    tcp_pcb_handle = NULL;
    tcp_status.aborted = 1;
    tcp_status.closed = 1;
    tcp_status.fatal_error = err;
    if(!tcp_status.complete) finish(err, now);
}

static err_t tcp_echo_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    (void)arg;
    (void)pcb;
    tcp_status.bytes_acked += len;
    return ERR_OK;
}

static err_t tcp_echo_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p,
                           err_t err)
{
    uint8_t expected[LWIP_UTK_TCP_RECORD_SIZE];
    uint32_t now = sys_now();
    uint16_t offset = 0;

    (void)arg;
    if(err != ERR_OK) {
        if(p != NULL) pbuf_free(p);
        finish(err, now);
        return ERR_OK;
    }
    if(p == NULL) {                     /* orderly close from the peer */
        tcp_status.peer_fin = 1;
        return ERR_OK;
    }

    tcp_status.segments_received++;
    tcp_status.bytes_received += p->tot_len;
    if(p->tot_len < LWIP_UTK_TCP_RECORD_SIZE) tcp_status.short_segments++;
    tcp_recved(pcb, p->tot_len);

    /* TCP is a byte stream: a record may arrive split across segments or
     * coalesced with the next one, so reassemble rather than assuming that
     * one segment carries exactly one record. */
    while(offset < p->tot_len && !tcp_status.complete) {
        uint16_t chunk = (uint16_t)(LWIP_UTK_TCP_RECORD_SIZE - rx_fill);

        if(chunk > p->tot_len - offset) chunk = (uint16_t)(p->tot_len - offset);
        if(pbuf_copy_partial(p, &rx_buffer[rx_fill], chunk, offset) != chunk) {
            tcp_status.corrupt++;
            finish(ERR_BUF, now);
            break;
        }
        rx_fill += chunk;
        offset = (uint16_t)(offset + chunk);
        if(rx_fill < LWIP_UTK_TCP_RECORD_SIZE) break;

        rx_fill = 0;
        if(!awaiting_echo) {            /* data the session never asked for */
            tcp_status.corrupt++;
            finish(ERR_VAL, now);
            break;
        }
        make_record(expected, record);
        if(!record_equal(rx_buffer, expected, sizeof(rx_buffer))) {
            tcp_status.corrupt++;
            finish(ERR_VAL, now);
            break;
        }
        tcp_status.records_matched++;
        awaiting_echo = 0;
        record++;
    }

    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_echo_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
    uint32_t now = sys_now();

    (void)arg;
    (void)pcb;
    tcp_status.connect_result = err;
    if(err != ERR_OK) {
        finish(err, now);
        return ERR_OK;
    }
    tcp_status.connected = 1;
    tcp_status.connect_ms = now - connect_started_ms;
    return ERR_OK;
}

static int32_t send_record(void)
{
    uint8_t payload[LWIP_UTK_TCP_RECORD_SIZE];
    err_t result;

    make_record(payload, record);
    result = tcp_write(tcp_pcb_handle, payload, sizeof(payload),
                       TCP_WRITE_FLAG_COPY);
    if(result == ERR_OK) result = tcp_output(tcp_pcb_handle);
    return result;
}

static void initialize(struct netif *netif)
{
    err_t result;

    tcp_status.initialized = 1;
    if(netif == NULL || !ip4addr_aton(UTK_TCP_ECHO_ADDRESS, &tcp_target)
       || ip4_addr_isany_val(tcp_target)) {
        tcp_status.init_result = ERR_ARG;
        tcp_status.complete = 1;
        tcp_status.closed = 1;
        tcp_status.result = ERR_ARG;
        return;
    }
    copy_address(tcp_status.target, &tcp_target);

    tcp_pcb_handle = tcp_new();
    if(tcp_pcb_handle == NULL) {
        tcp_status.init_result = ERR_MEM;
        tcp_status.complete = 1;
        tcp_status.closed = 1;
        tcp_status.result = ERR_MEM;
        return;
    }
    tcp_bind_netif(tcp_pcb_handle, netif);
    tcp_arg(tcp_pcb_handle, NULL);
    tcp_recv(tcp_pcb_handle, tcp_echo_recv);
    tcp_sent(tcp_pcb_handle, tcp_echo_sent);
    tcp_err(tcp_pcb_handle, tcp_echo_err);

    connect_started_ms = sys_now();
    result = tcp_connect(tcp_pcb_handle, (const ip_addr_t *)&tcp_target,
                         (uint16_t)UTK_TCP_ECHO_PORT, tcp_echo_connected);
    if(result != ERR_OK) {
        /* A rejected connect request leaves the pcb ours to release. */
        detach_callbacks(tcp_pcb_handle);
        tcp_abort(tcp_pcb_handle);
        tcp_pcb_handle = NULL;
        tcp_status.init_result = result;
        tcp_status.connect_result = result;
        tcp_status.complete = 1;
        tcp_status.closed = 1;
        tcp_status.result = result;
        return;
    }
    tcp_status.init_result = ERR_OK;
}

/* Called only from the poll task, never from a callback: after the peer's FIN
 * the pcb has already moved to TIME_WAIT and belongs to the stack, so this
 * drops the application's handle rather than freeing the block itself. */
static void release_pcb(void)
{
    struct tcp_pcb *pcb = tcp_pcb_handle;

    if(pcb == NULL) {
        tcp_status.closed = 1;
        return;
    }
    detach_callbacks(pcb);
    tcp_status.close_result = tcp_close(pcb);
    if(tcp_status.close_result != ERR_OK) {
        tcp_abort(pcb);
        tcp_status.aborted = 1;
    }
    tcp_pcb_handle = NULL;
    tcp_status.closed = 1;
}

void lwip_utk_tcp_poll(struct netif *netif)
{
    uint32_t now = sys_now();

    tcp_status.poll_calls++;
    if(tcp_status.complete) {
        if(!tcp_status.closed) release_pcb();
        return;
    }
    if(!tcp_status.initialized) initialize(netif);
    if(tcp_status.complete || tcp_pcb_handle == NULL
       || !netif_is_up(netif) || !netif_is_link_up(netif)
       || ip4_addr_isany_val(*netif_ip4_addr(netif))) {
        tcp_status.guard_blocked++;
        return;
    }

    if(!tcp_status.started) {
        tcp_status.started = 1;
        started_ms = now;
        /* How long the netif guard held the session back after the connect
         * request was stamped.  A large value here means the connect timeout
         * is being measured against a stale base. */
        tcp_status.start_delay_ms = now - connect_started_ms;
    }
    tcp_status.elapsed_ms = now - started_ms;
    if(tcp_status.elapsed_ms >= UTK_TCP_SESSION_TIMEOUT_MS) {
        finish(ERR_TIMEOUT, now);
        return;
    }

    if(!tcp_status.connected) {
        if(now - connect_started_ms >= UTK_TCP_CONNECT_TIMEOUT_MS) {
            tcp_status.connect_result = ERR_TIMEOUT;
            finish(ERR_TIMEOUT, now);
        }
        return;
    }

    /* Half-close first and keep receiving, so the peer's own FIN is observed
     * rather than assumed: an orderly shutdown is part of this gate. */
    if(tcp_status.fin_sent) {
        if(tcp_status.peer_fin) {
            tcp_status.close_ms = now - close_started_ms;
            release_pcb();
            finish(tcp_status.close_result == ERR_OK
                   ? ERR_OK : tcp_status.close_result, now);
        } else if(now - close_started_ms >= UTK_TCP_CLOSE_TIMEOUT_MS) {
            tcp_status.close_ms = now - close_started_ms;
            finish(ERR_TIMEOUT, now);
        }
        return;
    }

    if(tcp_status.records_matched >= LWIP_UTK_TCP_RECORD_COUNT) {
        err_t result = tcp_shutdown(tcp_pcb_handle, 0, 1);

        if(result == ERR_OK) {
            tcp_status.fin_sent = 1;
            close_started_ms = now;
        } else if(result != ERR_MEM) {  /* ERR_MEM simply retries next poll */
            tcp_status.close_result = result;
            finish(result, now);
        }
        return;
    }

    if(awaiting_echo) {
        /* Retransmission is TCP's responsibility, so a missing echo inside
         * this window is a session failure rather than an application retry. */
        if(now - last_send_ms >= UTK_TCP_ECHO_TIMEOUT_MS) finish(ERR_TIMEOUT, now);
        return;
    }

    tcp_status.send_result = send_record();
    if(tcp_status.send_result == ERR_OK) {
        tcp_status.records_sent++;
        tcp_status.bytes_sent += LWIP_UTK_TCP_RECORD_SIZE;
        last_send_ms = now;
        awaiting_echo = 1;
    } else if(tcp_status.send_result != ERR_MEM) {
        tcp_status.send_errors++;
        finish(tcp_status.send_result, now);
    }
}

void lwip_utk_tcp_get_status(T_LWIP_UTK_TCP_STATUS *status)
{
    if(status != NULL) *status = tcp_status;
}
