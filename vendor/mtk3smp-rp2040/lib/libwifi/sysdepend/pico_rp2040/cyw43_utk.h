/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CYW43_UTK_H
#define CYW43_UTK_H

#include <stdint.h>

#define CYW43_UTK_DNS_TEST_HOSTNAME "example.com"

#define CYW43_UDP_RING     32
#define CYW43_UDP_PAYLOAD  48

typedef struct {
    volatile int32_t init_result;
    volatile uint32_t init_complete;
    volatile uint32_t ready;
    volatile uint32_t scan_active;
    volatile uint32_t scan_complete;
    volatile uint32_t scan_count;
    volatile int32_t strongest_rssi;
    volatile uint32_t poll_count;
    volatile uint32_t owner_prc;
    uint8_t mac[6];
    volatile uint32_t join_enabled;
    volatile uint32_t join_started;
    volatile uint32_t join_complete;
    volatile int32_t join_request_result;
    volatile int32_t join_result;
    volatile int32_t link_status;
    volatile uint32_t link_up;
    volatile uint32_t join_elapsed_ms;
    volatile int32_t link_rssi;
    uint8_t bssid[6];
    volatile uint32_t lwip_initialized;
    volatile uint32_t netif_initialized;
    volatile uint32_t netif_up;
    volatile uint32_t netif_link_up;
    volatile uint32_t netif_default;
    volatile uint32_t netif_callbacks_ready;
    volatile uint32_t netif_no_sys;
    volatile uint32_t netif_mtu;
    uint8_t netif_mac[6];
    volatile uint32_t dhcp_enabled;
    volatile uint32_t dhcp_started;
    volatile uint32_t dhcp_complete;
    volatile int32_t dhcp_result;
    volatile uint32_t dhcp_elapsed_ms;
    volatile uint32_t dhcp_tries;
    uint8_t dhcp_address[4];
    uint8_t dhcp_netmask[4];
    uint8_t dhcp_gateway[4];
    uint8_t dhcp_dns[4];
    volatile uint32_t dns_enabled;
    volatile uint32_t dns_started;
    volatile uint32_t dns_complete;
    volatile int32_t dns_request_result;
    volatile int32_t dns_result;
    volatile uint32_t dns_elapsed_ms;
    uint8_t dns_address[4];
    volatile uint32_t udp_enabled;
    volatile int32_t udp_init_result;
    volatile uint32_t udp_initialized;
    volatile uint32_t udp_started;
    volatile uint32_t udp_complete;
    volatile uint32_t udp_closed;
    volatile int32_t udp_send_result;
    volatile int32_t udp_result;
    volatile uint32_t udp_expected_packets;
    volatile uint32_t udp_payload_size;
    volatile uint32_t udp_packets_sent;
    volatile uint32_t udp_packets_received;
    volatile uint32_t udp_packets_matched;
    volatile uint32_t udp_retries;
    volatile uint32_t udp_send_errors;
    volatile uint32_t udp_corrupt;
    volatile uint32_t udp_unexpected;
    volatile uint32_t udp_elapsed_ms;
    uint8_t udp_target[4];
    uint16_t udp_port;
    /* Mirror of the UDP driver's echo ring, so the application can print
       every message rather than whichever was current when it last looked.
       udp_write_seq is the total ever written; a reader tracks its own
       cursor against it and prints what is new. */
    /* Geometry must match LWIP_UTK_UDP_RING / LWIP_UTK_UDP_PAYLOAD_SIZE;
       checked at compile time in cyw43_utk.c.  Readers must use
       CYW43_UDP_RING for their slot arithmetic -- a mismatched modulus reads
       the wrong half of the ring and silently duplicates entries. */
    uint8_t udp_ring_payload[CYW43_UDP_RING][CYW43_UDP_PAYLOAD];
    uint32_t udp_ring_seq[CYW43_UDP_RING];
    uint16_t udp_ring_len[CYW43_UDP_RING];
    uint32_t udp_write_seq;
    volatile uint32_t udp_session_count;
    volatile uint32_t tcp_enabled;
    volatile int32_t tcp_init_result;
    volatile int32_t tcp_connect_result;
    volatile int32_t tcp_send_result;
    volatile int32_t tcp_close_result;
    volatile int32_t tcp_result;
    volatile int32_t tcp_fatal_error;
    volatile uint32_t tcp_initialized;
    volatile uint32_t tcp_started;
    volatile uint32_t tcp_connected;
    volatile uint32_t tcp_complete;
    volatile uint32_t tcp_fin_sent;
    volatile uint32_t tcp_peer_fin;
    volatile uint32_t tcp_closed;
    volatile uint32_t tcp_aborted;
    volatile uint32_t tcp_expected_records;
    volatile uint32_t tcp_record_size;
    volatile uint32_t tcp_records_sent;
    volatile uint32_t tcp_records_matched;
    volatile uint32_t tcp_bytes_sent;
    volatile uint32_t tcp_bytes_acked;
    volatile uint32_t tcp_bytes_received;
    volatile uint32_t tcp_segments_received;
    volatile uint32_t tcp_short_segments;
    volatile uint32_t tcp_corrupt;
    volatile uint32_t tcp_send_errors;
    volatile uint32_t tcp_poll_calls;
    volatile uint32_t tcp_guard_blocked;
    volatile uint32_t tcp_start_delay_ms;
    volatile uint32_t tcp_connect_ms;
    volatile uint32_t tcp_close_ms;
    volatile uint32_t tcp_elapsed_ms;
    uint8_t tcp_target[4];
    uint16_t tcp_port;
    volatile uint32_t bulk_enabled;
    volatile int32_t bulk_init_result;
    volatile int32_t bulk_connect_result;
    volatile int32_t bulk_send_result;
    volatile int32_t bulk_close_result;
    volatile int32_t bulk_result;
    volatile int32_t bulk_fatal_error;
    volatile uint32_t bulk_initialized;
    volatile uint32_t bulk_started;
    volatile uint32_t bulk_connected;
    volatile uint32_t bulk_complete;
    volatile uint32_t bulk_fin_sent;
    volatile uint32_t bulk_peer_fin;
    volatile uint32_t bulk_closed;
    volatile uint32_t bulk_aborted;
    volatile uint32_t bulk_total_bytes;
    volatile uint32_t bulk_bytes_sent;
    volatile uint32_t bulk_bytes_acked;
    volatile uint32_t bulk_bytes_received;
    volatile uint32_t bulk_mismatches;
    volatile uint32_t bulk_first_mismatch;
    volatile uint32_t bulk_segments_received;
    volatile uint32_t bulk_max_segment;
    volatile uint32_t bulk_min_segment;
    volatile uint32_t bulk_split_boundaries;
    volatile uint32_t bulk_write_retries;
    volatile uint32_t bulk_connect_ms;
    volatile uint32_t bulk_close_ms;
    volatile uint32_t bulk_elapsed_ms;
    volatile uint32_t bulk_throughput_bps;
    uint8_t bulk_target[4];
    uint16_t bulk_port;
    volatile int32_t ipv4_config_result;
    volatile uint32_t ipv4_configured;
    volatile uint32_t arp_resolved;
    volatile uint32_t ping_started;
    volatile uint32_t ping_complete;
    volatile int32_t ping_send_result;
    volatile int32_t ping_result;
    volatile uint32_t ping_attempts;
    volatile uint32_t ping_elapsed_ms;
    uint8_t ipv4_address[4];
    uint8_t ipv4_netmask[4];
    uint8_t ipv4_gateway[4];
    uint8_t ping_target[4];
    uint8_t arp_mac[6];
    /* Stamped by the reader, not the radio task: how many torn snapshots the
     * seqlock in cyw43_utk_get_status() has had to discard since boot. */
    uint32_t status_retries;
} T_CYW43_UTK_STATUS;

int32_t cyw43_utk_start(void);
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status);
void cyw43_utk_task(int32_t stacd, void *exinf);
void cyw43_utk_link_state_changed(int up);

#endif
