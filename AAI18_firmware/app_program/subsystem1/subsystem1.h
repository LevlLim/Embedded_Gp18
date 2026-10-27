/*
 * subsystem1.h
 *
 * Subsystem 1 -- WiFi, MQTT and telemetry.  Buddy 1 (Levl Lim), 2501118.
 *
 * Implements Statechart 1 of Design Review V2 section 6.1.
 *
 * RULES THIS SUBSYSTEM EXISTS TO HONOUR
 * -------------------------------------
 * 1. NFR6: a WiFi or MQTT failure must NEVER block, delay or stop any
 *    driving task.  Telemetry runs at the lowest priority, on core 1, and
 *    every hand-off to the network is a non-blocking (TMO_POL) call.
 *
 * 2. Cross-core data.  Telemetry runs on core 1; everything it reads is
 *    produced on core 0.  DI() is core-local, so it is never relied on
 *    here.  Every value Telemetry gathers is either a single aligned word
 *    read through its owner's accessor, or one of the two Motion Control
 *    fields in the snapshot below, which is guarded by a TA_INHERIT mutex
 *    whose critical section is a struct copy and nothing else.
 *
 * 3. lwIP is NO_SYS and runs on the template's radio task (core 0).  Only
 *    that task may call mqtt_publish().  Telemetry therefore never calls
 *    lwIP: it posts packed frames into an outbox message buffer, and the
 *    radio task drains it (template_patches/.../mqtt_utk.c).
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "../msg_defs.h"
#include <stdint.h>
#include <stdbool.h>

/* MQTT topics.  Design Review V2 section 6.1. */
#define S1_TOPIC_STATE      "car18/tele/state"
#define S1_TOPIC_HEARTBEAT  "car18/tele/hb"
#define S1_TOPIC_FAULT      "car18/tele/fault"
#define S1_TOPIC_CMD        "car18/cmd"

/* Topic codes carried in the first byte of an outbox message. */
#define S1_TOPIC_ID_STATE      (0u)
#define S1_TOPIC_ID_HEARTBEAT  (1u)
#define S1_TOPIC_ID_FAULT      (2u)

#define S1_HEARTBEAT_MS     (1000u)

/* READABLE DEBUG LINE (console builds only).  1 = every 200 ms print
     [DBG] side  210/ 300 W  ctr 1000/ 570 B  err   0 ok  duty L 40 R 40
   instead of the hex state frame, so the sensors and steering can be
   watched directly -- including by sliding the parked car across the
   line by hand.  0 = the hex frame, as before. */
#define S1_DEBUG_PRINT      (1)
#define S1_RECONNECT_TARGET_MS (10000u)  /* NFR6 */

typedef enum
{
    S1_WIFI_INIT         = 0,
    S1_WIFI_CONNECTING   = 1,
    S1_MQTT_CONNECTING   = 2,
    S1_CONNECTED         = 3,
    S1_RECONNECTING      = 4
} s1_conn_state_t;

/* Remote command codes: the second byte of a 2-byte car18/cmd payload
   (byte 0 is the monotonic sequence number). */
#define S1_CMD_START        (0x01u)
#define S1_CMD_STOP         (0x02u)
#define S1_CMD_RESET        (0x03u)

/*
 * Motion Control's contribution to each frame.  Written ONLY by Motion
 * Control (s1_publish_motion) and read ONLY by Telemetry, both under the
 * mutex.  encoder_total is a free-running sum that Telemetry differences,
 * so no field is ever read-modify-written by two tasks.
 */
typedef struct
{
    uint32_t encoder_total;    /* free-running |counts|, both wheels   */
    uint8_t  speed_pct;        /* commanded duty, 0..100               */
} s1_motion_fields_t;

/* Called by Motion Control every cycle.  Never blocks: if the mutex is
   busy the update is skipped and the next cycle carries it. */
void s1_publish_motion(uint8_t speed_pct, uint32_t encoder_total);

s1_conn_state_t s1_get_conn_state(void);

/* Longest time the telemetry mutex has been held, in microseconds.  This
   bounds Motion Control's worst-case blocking (test T1). */
uint32_t s1_max_hold_us(void);
void     s1_reset_stats(void);

/*
 * Accept an inbound remote command.  Returns false if the command code is
 * unknown, or the sequence number is stale or a duplicate.  A command is
 * accepted only if its sequence number is strictly newer than the last
 * accepted one, modulo 256.  An unknown command does not consume a
 * sequence number.
 */
bool s1_accept_command(uint8_t seq, uint8_t command);

/* ---------------------------------------------------------------------
 * Hooks called by the template's MQTT client on the RADIO task (core 0).
 * Present only in MQTT builds (TM_MQTT=1).  Plain C types, so the
 * template file can declare them without including application headers.
 * --------------------------------------------------------------------- */
#if defined(TM_MQTT) && (TM_MQTT == 1)
/* Next frame to publish: returns its length (0 if none) and its topic. */
int  app_mqtt_next_publish(const char **pp_topic, unsigned char *p_buf,
                           int buf_len);
/* A complete inbound message. */
void app_mqtt_on_message(const char *p_topic, const unsigned char *p_data,
                         int len);
/* Broker connection came up (non-zero) or went down (zero). */
void app_mqtt_link_state(int connected);
#endif

/* Create the telemetry mutex and the MQTT outbox (phase 1). */
ER s1_comms_init(void);

/* The telemetry task body.  Private to subsystem 1. */
void telemetry_task(INT stacd, void *p_exinf);

#endif /* SUBSYSTEM1_H */

/* end of file */
