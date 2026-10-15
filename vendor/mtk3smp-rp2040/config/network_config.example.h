/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.07.B0
 *
 *    Copyright (C) 2006-2023 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2023/11.
 *
 *----------------------------------------------------------------------
 */

/*
 * Copy this file to network_config.h and choose values valid for your LAN.
 * The local copy is ignored by Git.  Verify that UTK_IPV4_ADDRESS is unused
 * before flashing; a duplicate static address disrupts both devices.
 */
#ifndef UTK3_NETWORK_CONFIG_H
#define UTK3_NETWORK_CONFIG_H

/*
 * ---------------------------------------------------------------------------
 * Copy this file to config/network_config.h, fill in the values below, then change
 * the 0 on the next line to 1.
 *
 * It is a value to flip rather than a line to delete, so that an unedited
 * copy fails loudly at compile time instead of building fine and failing to
 * associate at runtime -- and so a careless edit cannot leave the guard
 * half-removed.
 * ---------------------------------------------------------------------------
 */
#define UTK_NETWORK_CONFIG_SET   0

#if !UTK_NETWORK_CONFIG_SET
#error "config/network_config.h: placeholder values. Set the static addressing for your network, then set UTK_NETWORK_CONFIG_SET to 1."
#endif


#define UTK_IPV4_ADDRESS       "192.168.1.50"
#define UTK_IPV4_NETMASK       "255.255.255.0"
#define UTK_IPV4_GATEWAY       "192.168.1.1"

/* Prefer the gateway or another reliable LAN host that answers ICMP echo. */
#define UTK_IPV4_PING_TARGET   "192.168.1.1"

#endif
