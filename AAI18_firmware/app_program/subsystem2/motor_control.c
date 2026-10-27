/*
 * motor_control.c
 *
 * Subsystem 2 -- the motor output layer and the PID controller.
 *
 * ALL motor operation goes through this file (Design Review V2 section
 * 12.3 interface rule).  Nothing else in the firmware writes PWM slices 4
 * or 5.
 *
 * BRAKE SEMANTICS: driving both pins of a channel LOW brakes the motor;
 * both HIGH coasts.  The emergency stop uses brake, not coast, because
 * coasting still carries the car forward.
 */

#include "subsystem2.h"
#include "motor_control.h"
#include "../drivers/rp2040_regs.h"

/* PWM period.  The template's pwm_set_wrap() leaves the slice divider at
   its reset value of 1, so the counter runs at the 125 MHz system clock
   and a wrap of 7812 gives 16 kHz: inside the Robo Pico driver's 20 kHz
   limit and above audible whine.  (An earlier comment claimed a divider
   of 4 and 4 kHz; no divider was ever set.) */
#define PWM_WRAP            (7812u)

/* ---------------------------------------------------------------------
 * Private
 * --------------------------------------------------------------------- */

static int32_t integral_left  = 0;
static int32_t integral_right = 0;
static int32_t prev_err_left  = 0;
static int32_t prev_err_right = 0;

/* Duty actually applied to each wheel, signed (-100..100, + = forward).
   motor_set() moves these toward the request by at most S2_SLEW_PCT per
   call; motor_brake() zeroes them at once. */
static int32_t applied_left  = 0;
static int32_t applied_right = 0;

/*
 * Clamp a duty percentage into 0..100.
 */
EXPORT uint8_t motor_clamp_duty(int32_t duty_pct)
{
    uint8_t result;

    if (duty_pct < 0)
    {
        result = 0u;
    }
    else if (duty_pct > 100)
    {
        result = 100u;
    }
    else
    {
        result = (uint8_t)duty_pct;
    }

    return (result);
}

/*
 * Drive one motor channel.  direction > 0 forward, < 0 reverse, 0 brake.
 */
static void drive_channel(uint32_t pin_a, uint32_t pin_b,
                          int8_t direction, uint8_t duty_pct)
{
    uint32_t level = ((uint32_t)duty_pct * PWM_WRAP) / 100u;

    if (0 == direction)
    {
        /* Brake: both LOW. */
        (void)pwm_set_cc(pin_a, 0u);
        (void)pwm_set_cc(pin_b, 0u);
    }
    else if (direction > 0)
    {
        (void)pwm_set_cc(pin_a, level);
        (void)pwm_set_cc(pin_b, 0u);
    }
    else
    {
        (void)pwm_set_cc(pin_a, 0u);
        (void)pwm_set_cc(pin_b, level);
    }
}

/*
 * Step one wheel's applied duty toward the request, rate limited, and
 * drive the channel with the result.
 */
static void slew_channel(uint32_t pin_a, uint32_t pin_b, int32_t *p_applied,
                         int8_t direction, uint8_t duty_pct)
{
    int32_t target;
    int32_t applied = *p_applied;

    target = (direction > 0) ? (int32_t)duty_pct
           : ((direction < 0) ? -(int32_t)duty_pct : 0);

    if (target > (applied + S2_SLEW_PCT))
    {
        applied = applied + S2_SLEW_PCT;
    }
    else if (target < (applied - S2_SLEW_PCT))
    {
        applied = applied - S2_SLEW_PCT;
    }
    else
    {
        applied = target;
    }

    *p_applied = applied;

    if (applied > 0)
    {
        drive_channel(pin_a, pin_b, 1, (uint8_t)applied);
    }
    else if (applied < 0)
    {
        drive_channel(pin_a, pin_b, -1, (uint8_t)(-applied));
    }
    else
    {
        drive_channel(pin_a, pin_b, 0, 0u);
    }
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT ER motor_init(void)
{
    uint32_t pins[4];
    uint32_t idx;

    pins[0] = PIN_MOTOR_L_A;
    pins[1] = PIN_MOTOR_L_B;
    pins[2] = PIN_MOTOR_R_A;
    pins[3] = PIN_MOTOR_R_B;

    /* The PWM block starts in reset: without this, nothing below has
       any effect and the motors never turn. */
    RP2040_RELEASE_PWM();

    for (idx = 0u; idx < 4u; idx++)
    {
        (void)pwm_set_pin(pins[idx]);
        (void)pwm_set_wrap(pins[idx], PWM_WRAP);
        (void)pwm_set_cc(pins[idx], 0u);
        (void)pwm_set_enabled(pins[idx], TRUE);
    }

    motor_brake();

    return (E_OK);
}

EXPORT void motor_brake(void)
{
    /*
     * The e-stop path.  Both pins of both channels LOW, applied directly
     * with no intermediate state, so the time from calling this to the
     * motors being shorted is a handful of register writes.
     */
    drive_channel(PIN_MOTOR_L_A, PIN_MOTOR_L_B, 0, 0u);
    drive_channel(PIN_MOTOR_R_A, PIN_MOTOR_R_B, 0, 0u);

    applied_left   = 0;
    applied_right  = 0;
    integral_left  = 0;
    integral_right = 0;
    prev_err_left  = 0;
    prev_err_right = 0;
}

EXPORT void motor_set(int8_t dir_left, uint8_t duty_left,
                      int8_t dir_right, uint8_t duty_right)
{
    slew_channel(PIN_MOTOR_L_A, PIN_MOTOR_L_B, &applied_left, dir_left,
                 motor_clamp_duty((int32_t)duty_left));
    slew_channel(PIN_MOTOR_R_A, PIN_MOTOR_R_B, &applied_right, dir_right,
                 motor_clamp_duty((int32_t)duty_right));
}

EXPORT void motor_get_applied(int32_t *p_left, int32_t *p_right)
{
    if ((NULL != p_left) && (NULL != p_right))
    {
        *p_left  = applied_left;
        *p_right = applied_right;
    }
}

EXPORT void motor_pid_reset(void)
{
    integral_left  = 0;
    integral_right = 0;
    prev_err_left  = 0;
    prev_err_right = 0;
}

EXPORT int32_t motor_pid_step(int32_t setpoint, int32_t measured,
                              bool b_left_wheel)
{
    int32_t *p_integral;
    int32_t *p_prev_err;
    int32_t  error;
    int32_t  derivative;
    int32_t  output;

    if (b_left_wheel)
    {
        p_integral = &integral_left;
        p_prev_err = &prev_err_left;
    }
    else
    {
        p_integral = &integral_right;
        p_prev_err = &prev_err_right;
    }

    error = setpoint - measured;

    /*
     * Integral with clamping.  Without the clamp, a stalled wheel winds
     * the term up without limit and the car lurches when the wheel frees
     * -- a classic and very visible PID failure on a line follower.
     */
    *p_integral += error;
    if (*p_integral > S2_INTEGRAL_LIMIT)
    {
        *p_integral = S2_INTEGRAL_LIMIT;
    }
    else if (*p_integral < -S2_INTEGRAL_LIMIT)
    {
        *p_integral = -S2_INTEGRAL_LIMIT;
    }
    else
    {
        /* Within range. */
    }

    derivative  = error - *p_prev_err;
    *p_prev_err = error;

    /* Q8 fixed point throughout; shift back down at the end. */
    output = ((error * S2_KP_Q8)
            + (*p_integral * S2_KI_Q8)
            + (derivative * S2_KD_Q8)) / 256;

    return (output);
}

EXPORT int32_t motor_counts_to_mm(int32_t counts)
{
    /*
     * Circumference = pi * diameter.  355/113 approximates pi to seven
     * digits, which keeps this in integer maths (Barr-C 5.4a).
     */
    int32_t circumference_mm = (S2_WHEEL_DIAM_MM * 355) / 113;

    return ((counts * circumference_mm) / S2_COUNTS_PER_REV);
}

EXPORT int32_t motor_mm_to_counts(int32_t millimetres)
{
    int32_t circumference_mm = (S2_WHEEL_DIAM_MM * 355) / 113;

    if (0 == circumference_mm)
    {
        return (0);
    }

    return ((millimetres * S2_COUNTS_PER_REV) / circumference_mm);
}

EXPORT int32_t motor_degrees_to_counts(int32_t degrees)
{
    /*
     * A spin turn moves each wheel along an arc of the track circle.
     * arc = pi * track_width * (degrees / 360)
     */
    int32_t arc_mm = (((S2_TRACK_WIDTH_MM * 355) / 113) * degrees) / 360;

    return (motor_mm_to_counts(arc_mm));
}

/* end of file */
