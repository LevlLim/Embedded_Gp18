/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Bounded raw-UDP echo qualification for the NO_SYS lwIP owner task. */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/udp.h"

#include "lwip_utk_udp.h"
#include "udp_test_config.h"

#define UTK_UDP_REPLY_TIMEOUT_MS      250U
#define UTK_UDP_SESSION_TIMEOUT_MS  30000U
#define UTK_UDP_MAX_ATTEMPTS            3U
/* Pause between runs.  The original session was a one-shot qualification
   gate: 64 packets, then done until reset.  For a demo that is over before
   anyone has finished reading it, so the session now repeats.  Set to 0 to
   restore the single-run behaviour. */
#define UTK_UDP_REPEAT_MS            5000U

_Static_assert(UTK_UDP_ECHO_PORT > 0U && UTK_UDP_ECHO_PORT <= 65535U,
               "UTK_UDP_ECHO_PORT must be in the range 1..65535");

static T_LWIP_UTK_UDP_STATUS udp_status = {
    .init_result = ERR_VAL,
    .send_result = ERR_VAL,
    .result = ERR_TIMEOUT,
    .expected_packets = LWIP_UTK_UDP_PACKET_COUNT,
    .payload_size = LWIP_UTK_UDP_PAYLOAD_SIZE,
    .port = (uint16_t)UTK_UDP_ECHO_PORT,
};
static struct udp_pcb *udp_pcb;
static ip4_addr_t udp_target;
static uint16_t sequence;
static uint32_t sequence_attempts;
static uint32_t awaiting_reply;
static uint32_t started_ms;
static uint32_t last_send_ms;
static uint32_t completed_ms;

static void copy_address(uint8_t dst[4], const ip4_addr_t *src)
{
    dst[0] = ip4_addr1(src);
    dst[1] = ip4_addr2(src);
    dst[2] = ip4_addr3(src);
    dst[3] = ip4_addr4(src);
}

static void make_payload(uint8_t payload[LWIP_UTK_UDP_PAYLOAD_SIZE],
                         uint16_t value)
{
    uint32_t hash = 2166136261U;
    unsigned int i;

    payload[0] = 'U';
    payload[1] = 'T';
    payload[2] = 'K';
    payload[3] = '6';
    payload[4] = (uint8_t)(value >> 8);
    payload[5] = (uint8_t)value;
    payload[6] = (uint8_t)LWIP_UTK_UDP_PACKET_COUNT;
    payload[7] = (uint8_t)LWIP_UTK_UDP_PAYLOAD_SIZE;
    for(i = 8; i < LWIP_UTK_UDP_PAYLOAD_SIZE - sizeof(hash); i++) {
        payload[i] = (uint8_t)(0x5aU + value * 29U + i * 17U);
    }
    for(i = 0; i < LWIP_UTK_UDP_PAYLOAD_SIZE - sizeof(hash); i++) {
        hash ^= payload[i];
        hash *= 16777619U;
    }
    payload[LWIP_UTK_UDP_PAYLOAD_SIZE - 4U] = (uint8_t)(hash >> 24);
    payload[LWIP_UTK_UDP_PAYLOAD_SIZE - 3U] = (uint8_t)(hash >> 16);
    payload[LWIP_UTK_UDP_PAYLOAD_SIZE - 2U] = (uint8_t)(hash >> 8);
    payload[LWIP_UTK_UDP_PAYLOAD_SIZE - 1U] = (uint8_t)hash;
}

static int payload_equal(const uint8_t *left, const uint8_t *right,
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
    udp_status.result = result;
    udp_status.elapsed_ms = now - started_ms;
    udp_status.complete = 1;
    awaiting_reply = 0;
    completed_ms = now;
}

/* Start another run of the same exchange.  write_seq and session_count stay
   monotonic so a reader's cursor keeps working across sessions; everything
   describing one run is cleared. */
static void restart_session(void)
{
    udp_status.started = 0;
    udp_status.complete = 0;
    udp_status.closed = 0;
    udp_status.initialized = 0;
    udp_status.packets_sent = 0;
    udp_status.packets_received = 0;
    udp_status.packets_matched = 0;
    udp_status.retries = 0;
    udp_status.send_errors = 0;
    udp_status.corrupt = 0;
    udp_status.unexpected = 0;
    udp_status.elapsed_ms = 0;
    udp_status.result = 0;
    udp_status.send_result = 0;
    udp_status.session_count++;
    sequence = 0;
    sequence_attempts = 0;
    awaiting_reply = 0;
}

static void udp_echo_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *address, uint16_t port)
{
    uint8_t received[LWIP_UTK_UDP_PAYLOAD_SIZE];
    uint8_t expected[LWIP_UTK_UDP_PAYLOAD_SIZE];
    uint32_t now = sys_now();

    (void)arg;
    (void)pcb;
    if(p == NULL) return;
    if(address == NULL || port != UTK_UDP_ECHO_PORT
       || !ip4_addr_eq(ip_2_ip4(address), &udp_target)
       || !awaiting_reply || udp_status.complete) {
        udp_status.unexpected++;
        pbuf_free(p);
        return;
    }

    udp_status.packets_received++;
    make_payload(expected, sequence);
    if(p->tot_len != LWIP_UTK_UDP_PAYLOAD_SIZE
       || pbuf_copy_partial(p, received, sizeof(received), 0)
              != sizeof(received)
       || !payload_equal(received, expected, sizeof(received))) {
        udp_status.corrupt++;
        pbuf_free(p);
        return;
    }

    udp_status.packets_matched++;
    /* Keep the bytes, not just the verdict.  The comparison above already
       proved they are correct; the application prints them so a human can
       see the round trip rather than trusting a counter. */
    {
        uint32_t slot = udp_status.write_seq % LWIP_UTK_UDP_RING;
        memcpy((void *)udp_status.ring_payload[slot], received,
               sizeof(received));
        udp_status.ring_len[slot] = (uint16_t)sizeof(received);
        udp_status.ring_seq[slot] = sequence;
        udp_status.write_seq++;
    }
    sequence++;
    sequence_attempts = 0;
    awaiting_reply = 0;
    if(sequence >= LWIP_UTK_UDP_PACKET_COUNT) finish(ERR_OK, now);
    pbuf_free(p);
}

static int32_t send_packet(void)
{
    uint8_t payload[LWIP_UTK_UDP_PAYLOAD_SIZE];
    struct pbuf *p;
    err_t result;

    make_payload(payload, sequence);
    p = pbuf_alloc(PBUF_TRANSPORT, sizeof(payload), PBUF_RAM);
    if(p == NULL) return ERR_MEM;
    result = pbuf_take(p, payload, sizeof(payload));
    if(result == ERR_OK) result = udp_send(udp_pcb, p);
    pbuf_free(p);
    return result;
}

static void initialize(struct netif *netif)
{
    err_t result;

    udp_status.initialized = 1;
    if(netif == NULL || !ip4addr_aton(UTK_UDP_ECHO_ADDRESS, &udp_target)
       || ip4_addr_isany_val(udp_target)) {
        udp_status.init_result = ERR_ARG;
        udp_status.complete = 1;
        udp_status.closed = 1;
        udp_status.result = ERR_ARG;
        return;
    }
    copy_address(udp_status.target, &udp_target);

    udp_pcb = udp_new();
    if(udp_pcb == NULL) {
        udp_status.init_result = ERR_MEM;
        udp_status.complete = 1;
        udp_status.closed = 1;
        udp_status.result = ERR_MEM;
        return;
    }
    udp_bind_netif(udp_pcb, netif);
    result = udp_bind(udp_pcb, IP_ADDR_ANY, 0);
    if(result == ERR_OK) {
        result = udp_connect(udp_pcb, (const ip_addr_t *)&udp_target,
                             (uint16_t)UTK_UDP_ECHO_PORT);
    }
    if(result != ERR_OK) {
        udp_remove(udp_pcb);
        udp_pcb = NULL;
        udp_status.init_result = result;
        udp_status.complete = 1;
        udp_status.closed = 1;
        udp_status.result = result;
        return;
    }
    udp_recv(udp_pcb, udp_echo_recv, NULL);
    udp_status.init_result = ERR_OK;
}

void lwip_utk_udp_poll(struct netif *netif)
{
    uint32_t now = sys_now();

    if(udp_status.complete) {
        if(udp_pcb != NULL) {
            udp_remove(udp_pcb);
            udp_pcb = NULL;
            udp_status.closed = 1;
        }
#if UTK_UDP_REPEAT_MS
        /* Only repeat a run that actually got going; a configuration error
           (bad address, no pcb) should stay reported, not spin. */
        if(udp_status.init_result == ERR_OK
           && now - completed_ms >= UTK_UDP_REPEAT_MS) {
            restart_session();
        }
#endif
        return;
    }
    if(!udp_status.initialized) initialize(netif);
    if(udp_status.complete || udp_pcb == NULL
       || !netif_is_up(netif) || !netif_is_link_up(netif)
       || ip4_addr_isany_val(*netif_ip4_addr(netif))) return;

    if(!udp_status.started) {
        udp_status.started = 1;
        started_ms = now;
    }
    udp_status.elapsed_ms = now - started_ms;
    if(udp_status.elapsed_ms >= UTK_UDP_SESSION_TIMEOUT_MS) {
        finish(ERR_TIMEOUT, now);
        return;
    }

    if(awaiting_reply) {
        if(now - last_send_ms < UTK_UDP_REPLY_TIMEOUT_MS) return;
        if(sequence_attempts >= UTK_UDP_MAX_ATTEMPTS) {
            finish(ERR_TIMEOUT, now);
            return;
        }
        awaiting_reply = 0;
    }

    if(sequence_attempts > 0) udp_status.retries++;
    udp_status.send_result = send_packet();
    sequence_attempts++;
    if(udp_status.send_result == ERR_OK) {
        udp_status.packets_sent++;
    } else {
        udp_status.send_errors++;
    }
    last_send_ms = now;
    awaiting_reply = 1;
}

void lwip_utk_udp_get_status(T_LWIP_UTK_UDP_STATUS *status)
{
    if(status != NULL) *status = udp_status;
}
