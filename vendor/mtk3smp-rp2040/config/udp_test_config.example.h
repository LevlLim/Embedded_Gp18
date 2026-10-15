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
 * Local endpoint for the opt-in Phase-6 UDP echo qualification profile.
 *
 * Copy this file to config/udp_test_config.h and set the address to the
 * Windows PC running tools/windows_udp_echo.ps1. The local copy is ignored
 * by Git because host addresses vary between networks.
 */
#ifndef UTK3_UDP_TEST_CONFIG_H
#define UTK3_UDP_TEST_CONFIG_H

/*
 * ---------------------------------------------------------------------------
 * Copy this file to config/udp_test_config.h, fill in the values below, then change
 * the 0 on the next line to 1.
 *
 * It is a value to flip rather than a line to delete, so that an unedited
 * copy fails loudly at compile time instead of building fine and failing to
 * associate at runtime -- and so a careless edit cannot leave the guard
 * half-removed.
 * ---------------------------------------------------------------------------
 */
#define UTK_UDP_TEST_SET   0

#if !UTK_UDP_TEST_SET
#error "config/udp_test_config.h: placeholder values. Set the address of the host running the echo server, then set UTK_UDP_TEST_SET to 1."
#endif


#define UTK_UDP_ECHO_ADDRESS    "192.168.1.100"
#define UTK_UDP_ECHO_PORT       7007U

#endif
