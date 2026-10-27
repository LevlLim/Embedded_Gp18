/*
 * subsystem5.h
 *
 * Subsystem 5 -- obstacle avoidance and the Vehicle Controller.
 * Buddy 5 (Jun Hong), 2501666.
 *
 * Implements the system-level statechart (Design Review V2 section 5) and
 * Statechart 5 (section 6.5).
 *
 * DIVISION OF LABOUR
 * ------------------
 *  - Obstacle Scan SENSES: it sweeps, detects, profiles, and watches
 *    D_stop.  It never sends a motion command.
 *  - The Vehicle Controller DECIDES and DRIVES: it is the SOLE sender of
 *    motion commands (so the 3-bit sequence numbers can never collide) and
 *    the SOLE fault-clearing authority (WIFI_LOSS excepted).
 *  They talk through flg_events (Obstacle -> Controller) and a one-byte
 *  request word (Controller -> Obstacle).
 *
 * SENSOR: HC-SR04P at 3.3 V, TRIG on GP0, ECHO direct to GP1, no divider.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../msg_defs.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------
 * Event flags raised by the other subsystems.  The Vehicle Controller is
 * the single consumer.
 * --------------------------------------------------------------------- */
#define S5_EVT_LINE_LOST        (0x0001u)
#define S5_EVT_BARCODE          (0x0002u)
#define S5_EVT_HUMP             (0x0004u)
#define S5_EVT_OBSTACLE         (0x0008u)   /* profile ready             */
#define S5_EVT_MOVE_DONE        (0x0010u)
#define S5_EVT_REMOTE_START     (0x0020u)
#define S5_EVT_REMOTE_STOP      (0x0040u)
#define S5_EVT_RESET            (0x0080u)
#define S5_EVT_OBSTACLE_SEEN    (0x0100u)   /* inside D_detect: stop     */
#define S5_EVT_ALL              (0x01FFu)

/* System-level states.  Packed as 3-bit parent + 2-bit substate. */
typedef enum
{
    S5_PARENT_INIT    = 0,
    S5_PARENT_READY   = 1,
    S5_PARENT_RUNNING = 2,
    S5_PARENT_STOPPED = 3,
    S5_PARENT_FAULT   = 4
} s5_parent_t;

typedef enum
{
    S5_SUB_LINE_FOLLOW    = 0,
    S5_SUB_BARCODE_ACTION = 1,
    S5_SUB_OBSTACLE_AVOID = 2,
    S5_SUB_LINE_SEARCH    = 3
} s5_sub_t;

/* Statechart 5 phases (Obstacle Scan). */
typedef enum
{
    OB_IDLE        = 0,     /* mission not running: no scanning        */
    OB_COARSE_SCAN = 1,     /* sweeping the forward arc while driving  */
    OB_WAIT_STOP   = 2,     /* reported, waiting for the Controller    */
    OB_FINE_SCAN   = 3,     /* car stopped: stepped scan, wide arc     */
    OB_HOLD        = 4      /* profile sent: servo forward, D_stop     */
} s5_obstacle_phase_t;

/* Controller -> Obstacle Scan requests (one byte, single writer). */
typedef enum
{
    S5_REQ_NONE   = 0,
    S5_REQ_SCAN   = 1,      /* the car is stopped: profile now         */
    S5_REQ_RESUME = 2       /* back to coarse scanning                 */
} s5_request_t;

/* ---------------------------------------------------------------------
 * Servo and scanning geometry.  PROVISIONAL.
 * --------------------------------------------------------------------- */
/* SERVO FIXED: 1 holds the HC-SR04P straight ahead and never sweeps (the
   sweep was pulling the sensor's wires off the servo).  The car still
   stops for anything ahead, but cannot profile it: it waits, stopped,
   until the path is clear, then carries on.  0 restores the sweep. */
#define S5_SERVO_FIXED          (1)
#define S5_SERVO_CENTRE_US      (1500u)
#define S5_SERVO_US_PER_DEG     (11u)   /* ~1000 us over 90 degrees      */
#define S5_SERVO_LIMIT_DEG      (60)
#define S5_SETTLE_TICKS         (4u)    /* x 60 ms = 240 ms, A6          */
#define S5_COARSE_ARC_DEG       (30)    /* bounded forward arc, risk R6  */
#define S5_FINE_ARC_DEG         (60)    /* profiling arc, car stopped    */
#define S5_STEP_DEG             (5)
#define S5_FINE_STEPS           (25u)   /* -60..+60 in 5 degree steps    */
/* HC-SR04P readings are noisy: one stray echo (the floor, a cable, a
   reflection) must not stop the car.  A stop or detection needs this
   many readings in a row (x 60 ms), and anything under S5_MIN_VALID_CM is
   treated as no reading (the sensor's own blind zone). */
#define S5_CONFIRM_PINGS        (2u)
#define S5_MIN_VALID_CM         (3u)
#define S5_DSTOP_ARC_DEG        (15)    /* D_stop only counts ahead      */
#define S5_OBJECT_DEPTH_TOL_CM  (10u)   /* same object if within this    */
#define S5_CLEAR_RANGE_CM       (50u)   /* free if nothing nearer        */
#define S5_MAX_RETRIES          (2u)
#define S5_ECHO_TIMEOUT_US      (30000u)
#define S5_NO_ECHO_LIMIT        (10u)   /* consecutive missing pulses    */
#define S5_ROBOT_WIDTH_CM       (18u)
#define S5_ROBOT_LENGTH_CM      (20u)
#define S5_CLEARANCE_MARGIN_CM  (5u)
#define S5_SEARCH_TIMEOUT_MS    (5000u) /* FR8: reacquire within 5 s     */
#define S5_REVERSE_MM           (100u)

/* Post an event to the Vehicle Controller.  Safe from any task. */
void s5_post_event(uint32_t event_bits);

/* Current mission state, already encoded (msg_encode_veh_state).  One
   byte written only by the Controller: safe from any task or core. */
uint8_t s5_get_encoded_state(void);

/* True while the mission is RUNNING. */
bool s5_is_running(void);

/* Latest forward distance in cm (0 = nothing in range), for telemetry.
   One byte, written only by Obstacle Scan. */
uint8_t s5_get_obstacle_distance_cm(void);

/* Controller -> Obstacle Scan. */
void s5_obstacle_request(s5_request_t req);

/* Controller -> Obstacle Scan: watch D_stop straight ahead while a forward
   bypass leg is being driven (on), not while turning in place (off). */
void s5_obstacle_guard(bool b_on);

/* Controller takes the latest profile, if one is waiting. */
bool s5_take_obstacle(msg_obstacle_t *p_out);

/* ---------------------------------------------------------------------
 * Pure helpers, exposed for the host test.
 * --------------------------------------------------------------------- */
/* Echo pulse width to centimetres (343 m/s, there and back). */
uint16_t s5_echo_us_to_cm(uint32_t width_us);

/* Bypass side from left/right clearance; NONE if neither fits. */
msg_side_t s5_choose_side(uint8_t clear_left_cm, uint8_t clear_right_cm,
                          uint8_t needed_cm);

/* Lateral offset in cm of a return at (distance, angle), positive right:
   distance x sin(angle), integer maths. */
int32_t s5_lateral_cm(uint32_t distance_cm, int32_t angle_deg);

/*
 * Build a profile from a fine scan: distances_cm[i] is the return at
 * angle (-S5_FINE_ARC_DEG + i x S5_STEP_DEG), 0 = no return.  Returns
 * false if nothing within detect_cm was seen.
 */
bool s5_build_profile(const uint8_t *p_distances_cm, uint8_t steps,
                      uint8_t detect_cm, msg_obstacle_t *p_profile);

/* Private to subsystem 5. */
ER   s5_obstacle_init(void);
ER   s5_controller_init(void);
void controller_task(INT stacd, void *p_exinf);
void obstacle_task(INT stacd, void *p_exinf);

#endif /* SUBSYSTEM5_H */

/* end of file */
