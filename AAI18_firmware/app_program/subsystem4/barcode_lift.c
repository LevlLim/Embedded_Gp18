/*
 * barcode_lift.c
 *
 * Subsystem 4 -- servo (GP14) that lifts the barcode IR sensor clear of
 * the track while the car is tilted, and lowers it once level again.
 * Automatic only while RUNNING.  While lifted, barcode reading is off.
 */

#include "subsystem4.h"
#include "../drivers/rp2040_regs.h"

#define LIFT_FRAME_US       (20000u)    /* 50 Hz                         */
#define LIFT_CLK_DIV        (125u)      /* 125 MHz / 125 = 1 us ticks    */
#define LIFT_CENTRE_US      (1500)
#define LIFT_US_PER_DEG     (11)
#define LIFT_LIMIT_DEG      (80)

static volatile bool    b_lifted  = false;
static volatile int32_t angle_now = S4_LIFT_DOWN_DEG;
static uint32_t         level_ms  = 0u;

static void lift_point(int32_t angle_deg)
{
    int32_t angle = angle_deg;

    if (angle > LIFT_LIMIT_DEG)
    {
        angle = LIFT_LIMIT_DEG;
    }
    else if (angle < -LIFT_LIMIT_DEG)
    {
        angle = -LIFT_LIMIT_DEG;
    }
    else
    {
        /* In range. */
    }

    (void)pwm_set_cc(PIN_SERVO_BARCODE,
                     (UW)(LIFT_CENTRE_US + (angle * LIFT_US_PER_DEG)));
    angle_now = angle;
}

static int32_t abs32(int32_t value)
{
    return ((value < 0) ? -value : value);
}

EXPORT void s4_lift_init(void)
{
    RP2040_RELEASE_PWM();
    out_w(PWM_SLICE_DIV(PWM_SLICE_OF(PIN_SERVO_BARCODE)),
          LIFT_CLK_DIV << PWM_DIV_INT_SHIFT);
    (void)pwm_set_pin(PIN_SERVO_BARCODE);
    (void)pwm_set_wrap(PIN_SERVO_BARCODE, LIFT_FRAME_US - 1u);
    lift_point(S4_LIFT_DOWN_DEG);
    (void)pwm_set_enabled(PIN_SERVO_BARCODE, TRUE);
    b_lifted = false;
}

EXPORT void s4_lift_step(bool b_running, uint32_t period_ms)
{
    int32_t pitch = (int32_t)s4_get_tilt_deg();
    int32_t roll  = (int32_t)s4_get_roll_deg();
    bool    b_tilted;
    bool    b_level;

    b_tilted = (abs32(pitch) >= S4_LIFT_START_DEG)
            || (abs32(roll) >= S4_LIFT_START_DEG);
    b_level  = (abs32(pitch) < S4_LIFT_LEVEL_DEG)
            && (abs32(roll) < S4_LIFT_LEVEL_DEG);

    if (!b_running)
    {
        level_ms = 0u;
    }
    else if (!b_lifted)
    {
        if (b_tilted)
        {
            lift_point(S4_LIFT_UP_DEG);
            b_lifted = true;
            level_ms = 0u;
            APP_LOG((UB*)"[S4] barcode sensor UP (pitch %d roll %d)\n",
                    (INT)pitch, (INT)roll);
        }
    }
    else if (b_level)
    {
        level_ms = level_ms + period_ms;
        if (level_ms >= S4_LIFT_LOWER_MS)
        {
            lift_point(S4_LIFT_DOWN_DEG);
            b_lifted = false;
            APP_LOG((UB*)"[S4] barcode sensor down (level again)\n");
        }
    }
    else
    {
        level_ms = 0u;
    }
}

EXPORT bool s4_barcode_lifted(void)
{
    return (b_lifted);
}

/* end of file */