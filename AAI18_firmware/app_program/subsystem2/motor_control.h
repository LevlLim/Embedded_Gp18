/*
 * motor_control.h
 *
 * Subsystem 2 -- private interface of the motor output layer.
 *
 * PRIVATE: only subsystem 2's own .c files include this.  Other
 * subsystems use subsystem2.h and send a command; they never call these
 * functions, because Motion Control is the single motor writer.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "subsystem2.h"
#include <stdint.h>
#include <stdbool.h>

/* Configure PWM slices 4 and 5 and leave the motors braked. */
ER motor_init(void);

/* Both pins of both channels LOW.  This is the e-stop action (FR11). */
void motor_brake(void);

/* direction: >0 forward, <0 reverse, 0 brake.  duty is 0..100 percent. */
void motor_set(int8_t dir_left, uint8_t duty_left,
               int8_t dir_right, uint8_t duty_right);

/* Clamp a signed duty request into 0..100 percent. */
uint8_t motor_clamp_duty(int32_t duty_pct);

/* Clear the integral and derivative history.  Called whenever control is
   handed over, so a stale integral cannot kick the motors on resume. */
void motor_pid_reset(void);

/* One PID iteration for one wheel.  Returns a signed duty correction. */
int32_t motor_pid_step(int32_t setpoint, int32_t measured, bool b_left_wheel);

/* Encoder / geometry conversions.  Integer maths only. */
int32_t motor_counts_to_mm(int32_t counts);
int32_t motor_mm_to_counts(int32_t millimetres);
int32_t motor_degrees_to_counts(int32_t degrees);

#endif /* MOTOR_CONTROL_H */

/* end of file */
