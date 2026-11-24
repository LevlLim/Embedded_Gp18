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
#define S5_EVT_MANUAL           (0x0200u)   /* calibration move request  */
#define S5_EVT_JUNCTION         (0x0400u)   /* both line sensors black   */
#define S5_EVT_TEST_TURN        (0x0800u)   /* jleft/jright over MQTT    */
#define S5_EVT_ALL              (0x0FFFu)

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
    OB_HOLD        = 4,     /* profile sent: servo forward, D_stop     */
    OB_SIDE        = 5      /* bypass: servo to the side, measuring    */
} s5_obstacle_phase_t;

/* Controller -> Obstacle Scan requests (one byte, single writer). */
typedef enum
{
    S5_REQ_NONE   = 0,
    S5_REQ_SCAN   = 1,      /* the car is stopped: profile now         */
    S5_REQ_RESUME = 2,      /* back to coarse scanning                 */
    S5_REQ_SIDE_L = 3,      /* bypass: look left (obstacle on the left) */
    S5_REQ_SIDE_R = 4,      /* bypass: look right                       */
    S5_REQ_AHEAD  = 5,      /* bypass: look ahead again (D_stop guard)  */
    S5_REQ_MEASURE = 6      /* bypass: straight ahead, report readings  */
} s5_request_t;

/* ---------------------------------------------------------------------
 * Servo and scanning geometry.  PROVISIONAL.
 * --------------------------------------------------------------------- */
/* SERVO FIXED: 1 holds the HC-SR04P straight ahead and never sweeps (the
   sweep was pulling the sensor's wires off the servo).  The car still
   stops for anything ahead, but cannot profile it: it waits, stopped,
   until the path is clear, then carries on.  0 restores the sweep. */
#define S5_SERVO_FIXED          (0)     /* sweep on: arc limited below */
#define S5_SERVO_CENTRE_US      (1500u)
/* SERVO TRIM: the horn is not fitted dead straight, so at the servo's
   own centre the sensor points slightly to one side.  This many degrees
   are added to every servo command (+ = right) so that angle 0 really is
   straight ahead.  Find it with the "SERVO CENTRE" step in the Buddy 5
   self-test: the printed angle at which the sensor faces straight ahead.
   Measured on the car. */
#define S5_SERVO_TRIM_DEG       (-6)    /* measured: straight at raw -6 */
#define S5_SERVO_US_PER_DEG     (11u)   /* ~1000 us over 90 degrees      */
/* SCAN ARC: how far the servo may turn each way, in degrees.  The
   sensor's wires limit it on this car, so MEASURE it: the Buddy 5
   self-test (test-b5) steps the servo out 5 degrees at a time and
   prints each angle -- set this to the largest angle (multiple of 5)
   reached on BOTH sides without the wires pulling.  Everything below
   follows from it.  PROVISIONAL until measured. */
#define S5_SCAN_ARC_DEG         (30)    /* raw limit 40 each side; with */
                                        /* the -6 trim that is 34 left, */
                                        /* 46 right -> 30 both          */
#define S5_STEP_DEG             (5)
#define S5_SERVO_LIMIT_DEG      (S5_SCAN_ARC_DEG)   /* hard clamp        */
#define S5_FINE_ARC_DEG         (S5_SCAN_ARC_DEG)   /* car stopped       */
/* 0 = servo straight ahead while driving (obstacles are on the track);
   it only sweeps after stopping for one.  1 = sweep +-S5_COARSE_ARC_DEG
   while driving too. */
#define S5_DRIVE_SWEEP          (0)
#define S5_COARSE_ARC_DEG       (((S5_SCAN_ARC_DEG / 2) / S5_STEP_DEG) \
                                 * S5_STEP_DEG)     /* while driving     */
#define S5_FINE_STEPS           ((uint32_t)(((2 * S5_FINE_ARC_DEG)     \
                                             / S5_STEP_DEG) + 1))
#define S5_SETTLE_TICKS         (4u)    /* x 60 ms = 240 ms, A6          */
MSG_STATIC_ASSERT((S5_SCAN_ARC_DEG % S5_STEP_DEG) == 0, arc_whole_steps);
MSG_STATIC_ASSERT((S5_SCAN_ARC_DEG >= 10) && (S5_SCAN_ARC_DEG <= 60),
                  arc_in_range);
/* HC-SR04P readings are noisy: one stray echo (the floor, a cable, a
   reflection) must not stop the car.  A stop or detection needs this
   many readings in a row (x 60 ms), and anything under S5_MIN_VALID_CM is
   treated as no reading (the sensor's own blind zone). */
#define S5_CONFIRM_PINGS        (2u)
#define S5_START_GRACE_MS       (800u)  /* ignore the START-pressing hand */
#define S5_MIN_VALID_CM         (5u)    /* 3 cm ghost echoes stopped the car */
#define S5_DSTOP_ARC_DEG        (15)    /* D_stop only counts ahead      */
#define S5_OBJECT_DEPTH_TOL_CM  (10u)   /* same object if within this    */
#define S5_CLEAR_RANGE_CM       (50u)   /* free if nothing nearer        */
#define S5_MAX_RETRIES          (2u)
#define S5_ECHO_TIMEOUT_US      (30000u)
#define S5_NO_ECHO_LIMIT        (10u)   /* consecutive missing pulses    */
#define S5_ROBOT_WIDTH_CM       (18u)
#define S5_ROBOT_LENGTH_CM      (20u)
/* MEASURE: distance from the line sensors back to the wheel axle, mm.
   At a junction the sensors reach the cross line first; the car drives
   on this far so it turns about the junction itself.  PROVISIONAL. */
#define S5_SENSOR_TO_AXLE_MM    (60u)
/* Leg 3 drives distance + width + this + margin past the obstacle.  Was
   the whole car length (20): test 8 drove 63 cm and left the mat. */
/* Bypass side when both sides are equally open:
   0 = the side the obstacle sticks out less on, 1 = LEFT, 2 = RIGHT. */
#ifndef S5_TIE_SIDE
#define S5_TIE_SIDE             (0)
#endif
/* Rejoining the line after a bypass (leg 5): a black stripe counts as
   the track line only if it is S5_LINE_MIN_MM..S5_LINE_MAX_MM wide and
   followed by S5_LINE_WHITE_MM of white.  Barcode bars (run along the
   car's path here) are rejected: test 9 realigned on a barcode sheet. */
#ifndef S5_LINE_MIN_MM
#define S5_LINE_MIN_MM          (8)
#endif
#ifndef S5_LINE_MAX_MM
#define S5_LINE_MAX_MM          (45)
#endif
/* Crossing at 45 degrees the stripe reads much wider: the tape itself
   (18 / sin 45 = 25 mm) PLUS the gap between the two line sensors, which
   cross it one after the other (~24 mm).  45 rejected the real line and
   the car drove past it (bench 17 Nov). */
#ifndef S5_LINE_MAX_45_MM
#define S5_LINE_MAX_45_MM       (80)
#endif
#ifndef S5_LINE_WHITE_MM
#define S5_LINE_WHITE_MM        (25)
#endif
/* Bypass turns: 45 = diagonal out and back (default), 90 = square. */
#ifndef S5_BYPASS_TURN_DEG
#define S5_BYPASS_TURN_DEG      (45u)
#endif
/* How far anything sticks out beyond S5_ROBOT_WIDTH_CM on each side
   (the barcode sensor is on the LEFT).  Added to the sideways leg when
   the obstacle passes on that side. */
#ifndef S5_LEFT_OVERHANG_CM
#define S5_LEFT_OVERHANG_CM     (4)
#endif
#ifndef S5_RIGHT_OVERHANG_CM
#define S5_RIGHT_OVERHANG_CM    (0)
#endif
/* MEASURING THE OBSTACLE while passing it (leg 3): the servo turns to
   its limit towards the obstacle and the car notes how far it drives
   while the sensor sees it (= the obstacle's length).  Once the sensor
   loses it, leg 3 ends early: the far end is (reading x cos 30) ahead of
   the sensor, plus sensor-to-axle, plus S5_PASS_EXTRA_CM + margin.
   A reading this close or closer counts as the obstacle. */
/* Ultrasonic beam spread: the scan sees each edge of an obstacle this far
   outside where it really is, so the width reads 2x this too wide (and
   the bypass swings out too far).  Set it from a real obstacle: (printed
   width - real width) / 2.  0 = no correction. */
#ifndef S5_BEAM_SPREAD_CM
#define S5_BEAM_SPREAD_CM       (3)
#endif

#ifndef S5_BEAM_HALF_DEG
#define S5_BEAM_HALF_DEG        (10)
#endif
/* PATH CHECK, mid-bypass: after turning back towards the line (leg 4)
   the sensor looks along the way to the line.  Anything closer than that
   way is the obstacle not yet passed: turn parallel again, drive
   S5_PATH_STEP_CM further, turn back and look again (up to
   S5_PATH_TRIES times). */
#ifndef S5_PATH_STEP_CM
#define S5_PATH_STEP_CM         (10u)
#endif
#ifndef S5_PATH_TRIES
#define S5_PATH_TRIES           (6u)
#endif
#ifndef S5_SIDE_NEAR_CM
#define S5_SIDE_NEAR_CM         (60u)
#endif
#ifndef S5_US_TO_AXLE_CM                /* ultrasonic ahead of the axle  */
#define S5_US_TO_AXLE_CM        (10u)
#endif
#define S5_SIDE_LOST_PINGS      (3u)    /* far this many pings in a row  */
#ifndef S5_PASS_EXTRA_CM
#define S5_PASS_EXTRA_CM        (8u)
#endif
#define S5_CLEARANCE_MARGIN_CM  (5u)
#define S5_SEARCH_TIMEOUT_MS    (5000u) /* FR8: reacquire within 5 s     */
#define S5_REVERSE_MM           (100u)

/* Post an event to the Vehicle Controller.  Safe from any task. */
void s5_post_event(uint32_t event_bits);

/* CALIBRATION MOVES from the laptop (MQTT text "left", "right", "fwd",
   "back").  Carried out only while the car is parked (READY or STOPPED);
   ignored while driving.  For checking turn angles (S2_TRACK_WIDTH_MM)
   and distances (S2_WHEEL_DIAM_MM) without driving the course. */
#define S5_MANUAL_TURN_DEG      (90u)
#define S5_MANUAL_FWD_MM        (700u)
#define S5_MANUAL_BACK_MM       (200u)
void s5_manual_move(msg_cmd_type_t type, uint32_t arg);

/* Current mission state, already encoded (msg_encode_veh_state).  One
   byte written only by the Controller: safe from any task or core. */
uint8_t s5_get_encoded_state(void);

#if APP_TEST
/* Self-test only: point the servo at any angle (+ = right), ignoring
   S5_SERVO_FIXED and the scan-arc clamp (limited to +-80).  "point"
   includes S5_SERVO_TRIM_DEG (0 = straight ahead); "raw" does not
   (0 = the servo's own centre), for finding the trim. */
void s5_servo_test_point(int32_t angle_deg);
void s5_servo_test_raw(int32_t angle_deg);

/* Ultrasonic bench test (TEST_BUDDY=6 / preset test-us): servo straight,
   prints the sensor's reading 4 times a second, never returns. */
void s5_ultrasonic_bench(void);
#endif

/* True while the mission is RUNNING. */
bool s5_is_running(void);

/* True once the boot self-test / calibration has finished (READY or
   FAULT).  The TEST build waits on this before running the tests. */
bool s5_is_initialised(void);

/* Latest forward distance in cm (0 = nothing in range), for telemetry.
   One byte, written only by Obstacle Scan. */
uint8_t s5_get_obstacle_distance_cm(void);

/* Bypass side look: the latest reading (cm, 0 = nothing / invalid) and
   a count that goes up by one per ping, so a new reading can be told
   from an old one. */
uint8_t s5_get_side_cm(uint32_t *p_count);

/* True if the last profile's obstacle reached an end of the scan arc: its
   real edge was not seen, so its width is a lower bound only. */
bool s5_last_scan_clipped(void);
/* The last profile's obstacle edges, cm from the car's centre line
   (left negative), each pushed out by one scan step for safety. */
void s5_get_edges_cm(int32_t *p_left_cm, int32_t *p_right_cm);

/* Testing without a printed barcode: store a junction command
   (msg_barcode_cmd_t) exactly as if a barcode had been read.  From the
   MQTT text commands jleft / jright / jstraight / juturn. */
void s5_inject_turn(uint8_t cmd);

/* Where the scan servo points now, degrees from straight ahead (+ right,
   trim excluded).  For the debug line. */
int32_t s5_get_servo_angle(void);

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