/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Bounded bulk-TCP echo qualification for the NO_SYS lwIP owner task.
 *
 * Phase 7 proved connect, exchange, and orderly close, but its stop-and-wait
 * 48-byte records each arrived as one segment, so the receive path's stream
 * reassembly was never stressed.  This profile removes record framing
 * entirely: the payload is a pure byte stream whose value at offset i is a
 * function of i alone, so verification is correct at any segment boundary and
 * a segment may carry any number of bytes.  Sending is pipelined rather than
 * stop-and-wait, which is what makes the peer echo full-size segments back.
 */

#include <stddef.h>
#include <stdint.h>

#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"

#include "lwip_utk_tcpbulk.h"
#include "tcp_test_config.h"

#define UTK_BULK_CONNECT_TIMEOUT_MS  10000U
#define UTK_BULK_STALL_TIMEOUT_MS     5000U
#define UTK_BULK_CLOSE_TIMEOUT_MS     5000U
#define UTK_BULK_SESSION_TIMEOUT_MS  30000U
#define UTK_BULK_CHUNK                 64U
/* Large enough that filling the send window costs a handful of tcp_write()
 * calls rather than dozens, which keeps the queued-pbuf count well inside
 * TCP_SND_QUEUELEN.  Both scratch buffers are file-scope on purpose: the
 * radio-owner task shares its 6 KB stack with the whole lwIP call chain. */
#define UTK_BULK_SEND_CHUNK           512U

static T_LWIP_UTK_TCPBULK_STATUS bulk_status = {
    .init_result = ERR_VAL,
    .connect_result = ERR_VAL,
    .send_result = ERR_VAL,
    .close_result = ERR_VAL,
    .result = ERR_TIMEOUT,
    .fatal_error = ERR_OK,
    .total_bytes = LWIP_UTK_TCPBULK_TOTAL_BYTES,
    .min_segment = 0xffffffffU,
    .port = (uint16_t)UTK_TCP_ECHO_PORT,
};
static uint8_t send_scratch[UTK_BULK_SEND_CHUNK];
static uint8_t recv_scratch[UTK_BULK_CHUNK];
static struct tcp_pcb *bulk_pcb;
static ip4_addr_t bulk_target;
static uint32_t send_offset;
static uint32_t recv_offset;
static uint32_t started_ms;
static uint32_t connect_started_ms;
static uint32_t last_progress_ms;
static uint32_t close_started_ms;

static void copy_address(uint8_t dst[4], const ip4_addr_t *src)
{
    dst[0] = ip4_addr1(src);
    dst[1] = ip4_addr2(src);
    dst[2] = ip4_addr3(src);
    dst[3] = ip4_addr4(src);
}

/* Value at a stream offset depends only on that offset, so the verifier never
 * needs to know how the bytes were grouped into segments. */
static uint8_t stream_byte(uint32_t offset)
{
    uint32_t h = offset * 2654435761U;

    h ^= h >> 13;
    h *= 1274126177U;
    return (uint8_t)(h ^ (h >> 17));
}

static void finish(int32_t result, uint32_t now)
{
    bulk_status.result = result;
    bulk_status.elapsed_ms = now - started_ms;
    if(bulk_status.elapsed_ms > 0) {
        bulk_status.throughput_bps =
                (bulk_status.bytes_received * 1000U) / bulk_status.elapsed_ms;
    }
    bulk_status.complete = 1;
}

static void detach_callbacks(struct tcp_pcb *pcb)
{
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 0);
}

static void bulk_err(void *arg, err_t err)
{
    uint32_t now = sys_now();

    (void)arg;
    bulk_pcb = NULL;                    /* lwIP has already freed it */
    bulk_status.aborted = 1;
    bulk_status.closed = 1;
    bulk_status.fatal_error = err;
    if(!bulk_status.complete) finish(err, now);
}

static err_t bulk_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    (void)arg;
    (void)pcb;
    bulk_status.bytes_acked += len;
    last_progress_ms = sys_now();
    return ERR_OK;
}

static err_t bulk_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    uint32_t now = sys_now();
    uint16_t offset = 0;

    (void)arg;
    if(err != ERR_OK) {
        if(p != NULL) pbuf_free(p);
        finish(err, now);
        return ERR_OK;
    }
    if(p == NULL) {
        bulk_status.peer_fin = 1;
        return ERR_OK;
    }

    bulk_status.segments_received++;
    if(p->tot_len > bulk_status.max_segment) bulk_status.max_segment = p->tot_len;
    if(p->tot_len < bulk_status.min_segment) bulk_status.min_segment = p->tot_len;
    /* A segment starting mid-chunk proves the verifier is not relying on the
     * transport to preserve any application framing. */
    if((recv_offset % UTK_BULK_CHUNK) != 0) bulk_status.split_boundaries++;
    bulk_status.bytes_received += p->tot_len;
    tcp_recved(pcb, p->tot_len);
    last_progress_ms = now;

    while(offset < p->tot_len) {
        uint16_t chunk = UTK_BULK_CHUNK;
        uint16_t i;

        if(chunk > p->tot_len - offset) chunk = (uint16_t)(p->tot_len - offset);
        if(pbuf_copy_partial(p, recv_scratch, chunk, offset) != chunk) {
            bulk_status.mismatches++;
            finish(ERR_BUF, now);
            break;
        }
        for(i = 0; i < chunk; i++) {
            if(recv_scratch[i] != stream_byte(recv_offset + i)) {
                if(bulk_status.mismatches == 0) {
                    bulk_status.first_mismatch = recv_offset + i;
                }
                bulk_status.mismatches++;
            }
        }
        recv_offset += chunk;
        offset = (uint16_t)(offset + chunk);
    }

    if(bulk_status.mismatches != 0 && !bulk_status.complete) {
        finish(ERR_VAL, now);
    }
    pbuf_free(p);
    return ERR_OK;
}

static err_t bulk_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
    uint32_t now = sys_now();

    (void)arg;
    (void)pcb;
    bulk_status.connect_result = err;
    if(err != ERR_OK) {
        finish(err, now);
        return ERR_OK;
    }
    bulk_status.connected = 1;
    bulk_status.connect_ms = now - connect_started_ms;
    last_progress_ms = now;
    return ERR_OK;
}

/* Keep the send window full instead of waiting for each echo: that is what
 * makes the peer return MSS-sized segments rather than one write per read. */
static void pump_send(void)
{
    uint32_t queued = 0;

    while(send_offset < bulk_status.total_bytes) {
        uint16_t space = tcp_sndbuf(bulk_pcb);
        uint32_t remaining = bulk_status.total_bytes - send_offset;
        uint16_t chunk = UTK_BULK_SEND_CHUNK;
        uint16_t i;
        err_t result;

        if(space == 0) break;
        if(chunk > space) chunk = space;
        if(chunk > remaining) chunk = (uint16_t)remaining;
        for(i = 0; i < chunk; i++) {
            send_scratch[i] = stream_byte(send_offset + i);
        }

        result = tcp_write(bulk_pcb, send_scratch, chunk, TCP_WRITE_FLAG_COPY);
        if(result == ERR_MEM) {
            bulk_status.write_retries++;
            break;
        }
        bulk_status.send_result = result;
        if(result != ERR_OK) return;
        send_offset += chunk;
        bulk_status.bytes_sent += chunk;
        queued += chunk;
    }
    if(queued > 0) {
        bulk_status.send_result = tcp_output(bulk_pcb);
    }
}

static void initialize(struct netif *netif)
{
    err_t result;

    bulk_status.initialized = 1;
    if(netif == NULL || !ip4addr_aton(UTK_TCP_ECHO_ADDRESS, &bulk_target)
       || ip4_addr_isany_val(bulk_target)) {
        bulk_status.init_result = ERR_ARG;
        bulk_status.complete = 1;
        bulk_status.closed = 1;
        bulk_status.result = ERR_ARG;
        return;
    }
    copy_address(bulk_status.target, &bulk_target);

    bulk_pcb = tcp_new();
    if(bulk_pcb == NULL) {
        bulk_status.init_result = ERR_MEM;
        bulk_status.complete = 1;
        bulk_status.closed = 1;
        bulk_status.result = ERR_MEM;
        return;
    }
    tcp_bind_netif(bulk_pcb, netif);
    tcp_arg(bulk_pcb, NULL);
    tcp_recv(bulk_pcb, bulk_recv);
    tcp_sent(bulk_pcb, bulk_sent);
    tcp_err(bulk_pcb, bulk_err);

    connect_started_ms = sys_now();
    result = tcp_connect(bulk_pcb, (const ip_addr_t *)&bulk_target,
                         (uint16_t)UTK_TCP_ECHO_PORT, bulk_connected);
    if(result != ERR_OK) {
        detach_callbacks(bulk_pcb);
        tcp_abort(bulk_pcb);
        bulk_pcb = NULL;
        bulk_status.init_result = result;
        bulk_status.connect_result = result;
        bulk_status.complete = 1;
        bulk_status.closed = 1;
        bulk_status.result = result;
        return;
    }
    bulk_status.init_result = ERR_OK;
}

static void release_pcb(void)
{
    struct tcp_pcb *pcb = bulk_pcb;

    if(pcb == NULL) {
        bulk_status.closed = 1;
        return;
    }
    detach_callbacks(pcb);
    bulk_status.close_result = tcp_close(pcb);
    if(bulk_status.close_result != ERR_OK) {
        tcp_abort(pcb);
        bulk_status.aborted = 1;
    }
    bulk_pcb = NULL;
    bulk_status.closed = 1;
}

void lwip_utk_tcpbulk_poll(struct netif *netif)
{
    uint32_t now = sys_now();

    if(bulk_status.complete) {
        if(!bulk_status.closed) release_pcb();
        return;
    }
    if(!bulk_status.initialized) initialize(netif);
    if(bulk_status.complete || bulk_pcb == NULL
       || !netif_is_up(netif) || !netif_is_link_up(netif)
       || ip4_addr_isany_val(*netif_ip4_addr(netif))) return;

    if(!bulk_status.started) {
        bulk_status.started = 1;
        started_ms = now;
        last_progress_ms = now;
    }
    bulk_status.elapsed_ms = now - started_ms;
    if(bulk_status.elapsed_ms >= UTK_BULK_SESSION_TIMEOUT_MS) {
        finish(ERR_TIMEOUT, now);
        return;
    }

    if(!bulk_status.connected) {
        if(now - connect_started_ms >= UTK_BULK_CONNECT_TIMEOUT_MS) {
            bulk_status.connect_result = ERR_TIMEOUT;
            finish(ERR_TIMEOUT, now);
        }
        return;
    }

    if(bulk_status.fin_sent) {
        if(bulk_status.peer_fin) {
            bulk_status.close_ms = now - close_started_ms;
            release_pcb();
            finish(bulk_status.close_result == ERR_OK
                   ? ERR_OK : bulk_status.close_result, now);
        } else if(now - close_started_ms >= UTK_BULK_CLOSE_TIMEOUT_MS) {
            bulk_status.close_ms = now - close_started_ms;
            finish(ERR_TIMEOUT, now);
        }
        return;
    }

    if(recv_offset >= bulk_status.total_bytes) {
        err_t result = tcp_shutdown(bulk_pcb, 0, 1);

        if(result == ERR_OK) {
            bulk_status.fin_sent = 1;
            close_started_ms = now;
        } else if(result != ERR_MEM) {
            bulk_status.close_result = result;
            finish(result, now);
        }
        return;
    }

    if(send_offset < bulk_status.total_bytes) pump_send();

    /* Retransmission is TCP's job; a stall this long means the transfer is
     * not going to finish inside the session cap either. */
    if(now - last_progress_ms >= UTK_BULK_STALL_TIMEOUT_MS) {
        finish(ERR_TIMEOUT, now);
    }
}

void lwip_utk_tcpbulk_get_status(T_LWIP_UTK_TCPBULK_STATUS *status)
{
    if(status != NULL) *status = bulk_status;
}
