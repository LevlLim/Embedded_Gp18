/*
 * sizes_probe.c
 *
 * Compiled with the ARM cross compiler by `make target-sizes`: each array
 * below is as long as the struct it names, so `arm-none-eabi-nm -S`
 * prints the TARGET's sizeof() for the CR1 message-size table.
 * Never linked into anything.
 */

#include "msg_defs.h"

char sz_motion_cmd[sizeof(msg_motion_cmd_t)];
char sz_barcode[sizeof(msg_barcode_t)];
char sz_obstacle[sizeof(msg_obstacle_t)];
char sz_telemetry[sizeof(msg_telemetry_t)];

/* end of file */
