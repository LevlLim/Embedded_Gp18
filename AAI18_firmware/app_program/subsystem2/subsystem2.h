/*
 * subsystem2.h
 *
 * Subsystem 2 -- Motion Control.  Buddy 2 (Ze Kai), 2501273.
 *
 * Implements Statechart 2 of Design Review V2 section 6.2, which was
 * redrawn after the Week 6 review found the original "receives
 * instructions from the Pico / Interrupt Occurs" version unimplementable.
 *
 * This subsystem is the single motor writer.  No other module may touch
 * PWM slices 4 or 5.  It is also the task that satisfies FR11 and NFR10:
 * it waits on the E-STOP mask of the shared fault flag and brakes within
 * one 10 ms cycle of seeing a bit.
 *
 * PUBLIC INTERFACE: everything another subsystem may use.  Anything else
 * stays private in subsystem2.c.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "../msg_defs.h"
#include <stdint.h>
#include <stdbool.h>

/* Statechart 2 states.  Stored in 3 bits of Motion Control's packed
   control word (task_motion.c). */
typedef enum
{
    M_INIT      = 0,
    M_IDLE      = 1,
    M_LINE_PID  = 2,
    M_MOVE      = 3,
    M_STALL     = 4,
    M_ESTOP     = 5
} s2_state_t;

/*
 * ENCODER FAULT is a DEGRADE condition (Fault Table bit 6), not a state:
 * when one encoder falls silent while the other counts, Motion Control
 * raises FAULT_ENCODER and keeps driving, completing encoder moves on the
 * healthy wheel alone.  Braking is left to the Controller.
 */

/* ---------------------------------------------------------------------
 * Encoder geometry.  PROVISIONAL -- counts per revolution has never been
 * stated (risk R8) and the wheel diameter is nominal.  Both must be
 * measured before any distance or turn figure is trusted.
 * --------------------------------------------------------------------- */
#define S2_COUNTS_PER_REV       (20)    /* PROVISIONAL: measure this      */
#define S2_WHEEL_DIAM_MM        (65)    /* PROVISIONAL: measure this      */
#define S2_TRACK_WIDTH_MM       (140)   /* PROVISIONAL: wheel separation  */

/* PID gains, in 1/256 fixed point.  Integer maths only: the Cortex-M0+ has
   no FPU and Barr-C 5.4a asks that floating point be avoided where fixed
   point will do.  PROVISIONAL -- to be tuned on the bench (A3). */
#define S2_KP_Q8                (120)
#define S2_KI_Q8                (3)     /* was 12: wound up to a 187%    */
#define S2_KD_Q8                (40)    /* correction and spun the car   */
#define S2_INTEGRAL_LIMIT       (1500)  /* caps I at ~18% duty           */

/* SMOOTHING (bench feedback: the car moved in jerks).
   - Each wheel's duty may change by at most S2_SLEW_PCT per 10 ms tick,
     so a step from 0 to cruise takes ~100 ms instead of one tick.  The
     emergency brake (motor_brake) is NOT rate limited.
   - The steering error is low-pass filtered: each tick moves the filtered
     value 1/S2_ERR_FILTER_DIV of the way to the new reading, turning the
     two-sensor error's hard steps (0 / 20 / 50 / 63) into smooth ramps. */
#define S2_SLEW_PCT             (8)     /* was 4: steering lagged, overshot */
#define S2_ERR_FILTER_DIV       (2)     /* was 3: same reason             */

/* Spin turns (line search, barcode, bypass) run at no more than this, so
   the car can stop on the line instead of sweeping past it. */
#define S2_TURN_DUTY_PCT        (25u)

/* Stall detection: drive commanded above this duty with no encoder edge
   for this long means the wheel is not turning. */
#define S2_STALL_DUTY_PCT       (25u)
#define S2_STALL_TIMEOUT_MS     (300u)

/* Encoder health: over this window, one wheel producing at least
   S2_ENC_FAULT_EDGES edges while the other produces none is a fault. */
#define S2_ENC_WINDOW_MS        (500u)
#define S2_ENC_FAULT_EDGES      (10u)

/* line_error older than this is treated as unusable (Statechart 2
   M_LINE_PID).  Line Sensor runs every 5 ms, so three missed updates. */
#define S2_LINE_STALE_MS        (20u)

/* Duty each wheel is actually getting after rate limiting, signed percent
   (+ forward).  Read-only, for the telemetry debug line.  (Implemented in
   motor_control.c; public so nothing outside subsystem 2 needs
   motor_control.h.) */
void motor_get_applied(int32_t *p_left, int32_t *p_right);

/* Current state, for telemetry and for the Controller's own decisions. */
s2_state_t s2_get_state(void);

/* Latest measured wheel speeds, in encoder counts per second. */
void s2_get_speeds(int32_t *p_left, int32_t *p_right);

/* Total distance travelled since start, in millimetres.  Used by the IMU
   subsystem to accumulate hump height over distance rather than by
   double-integrating acceleration. */
int32_t s2_get_distance_mm(void);

/* Send a motion command.  ONLY the Vehicle Controller calls this: one
   writer, so the 3-bit sequence numbers can never collide. */
ER s2_send_command(const msg_motion_cmd_t *p_cmd);

/* Line search: while on, a turn or move stops THE MOMENT either line
   sensor sees the line (checked every 10 ms by Motion Control itself),
   instead of finishing its angle and waiting for the Controller.  The
   move then reports MOVE_DONE as usual.  Cleared by LINE_PID. */
void s2_set_stop_on_line(bool b_on);

/* Create the command buffer (phase 1).  Private to subsystem 2. */
ER s2_motion_init(void);

/* The task body.  Private to subsystem 2. */
void motion_task(INT stacd, void *p_exinf);

#endif /* SUBSYSTEM2_H */

/* end of file */
