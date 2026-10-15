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
 * Local credentials for the opt-in WIFI_JOIN=1 qualification profile.
 *
 * Copy this file to config/wifi_credentials.h and edit the copy.  The real
 * credentials file is ignored by Git and must never be committed.
 */
#ifndef WIFI_CREDENTIALS_H
#define WIFI_CREDENTIALS_H

/*
 * ---------------------------------------------------------------------------
 * Copy this file to config/wifi_credentials.h, fill in the values below, then change
 * the 0 on the next line to 1.
 *
 * It is a value to flip rather than a line to delete, so that an unedited
 * copy fails loudly at compile time instead of building fine and failing to
 * associate at runtime -- and so a careless edit cannot leave the guard
 * half-removed.
 * ---------------------------------------------------------------------------
 */
#define UTK_WIFI_CREDENTIALS_SET   0

#if !UTK_WIFI_CREDENTIALS_SET
#error "config/wifi_credentials.h: placeholder values. Set your network SSID and password, then set UTK_WIFI_CREDENTIALS_SET to 1."
#endif


#define UTK_WIFI_SSID       "replace-with-ssid"
#define UTK_WIFI_PASSWORD   "replace-with-password"
#define UTK_WIFI_AUTH       CYW43_AUTH_WPA2_AES_PSK

#endif /* WIFI_CREDENTIALS_H */
