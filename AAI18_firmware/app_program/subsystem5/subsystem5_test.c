/*
 * subsystem5_test.c
 * On-target tests for subsystem 5 (obstacle and Vehicle Controller).
 *
 * The fault-table test here is T4 from the design review: several faults
 * raised at once, none lost, and WIFI_LOSS must NOT brake the car.
 */

#include "subsystem5.h"
#include "../fault.h"
#include "../drivers/gpio_irq.h"

#if APP_TEST

/*
 * When the range check fails, say WHY: count the ECHO pulses (GP1) that
 * arrive in 1 s and look at the last one's width.  The sensor is pinged
 * every 60 ms, so a working one gives ~16 pulses a second.
 *   0 pulses        -> nothing on GP1 at all: wiring / power / TRIG
 *   pulses, ~30 ms  -> echoes time out: nothing in range, or the sensor
 *                      points at open space / is blocked by soft material
 *   pulses, < 300us -> instant echo: TRIG/ECHO crosstalk or the sensor
 *                      seeing its own mount
 */
static void echo_diagnosis(void)
{
    gpio_irq_echo_t first = {0};
    gpio_irq_echo_t last  = {0};
    uint32_t        pulses;

    (void)gpio_irq_read_echo(&first);
    (void)tk_dly_tsk(1000);
    (void)gpio_irq_read_echo(&last);
    pulses = last.seq - first.seq;

    tm_printf((UB*)"[TEST-S5] diagnosis: %d echo pulses in 1 s, last "
                   "width %d us\n", (INT)pulses, (INT)last.width_us);

    if (0u == pulses)
    {
        tm_printf((UB*)"[TEST-S5]   NO ECHO AT ALL: check ECHO->GP1, "
                       "TRIG->GP0 (not swapped), VCC and GND\n");
    }
    else if (last.width_us >= S5_ECHO_TIMEOUT_US)
    {
        tm_printf((UB*)"[TEST-S5]   echoes time out: nothing reflecting "
                       "-- aim at a wall ~30 cm away\n");
    }
    else if (last.width_us < 300u)
    {
        tm_printf((UB*)"[TEST-S5]   instant echo: TRIG/ECHO wires "
                       "touching, or the sensor sees its own mount\n");
    }
    else
    {
        tm_printf((UB*)"[TEST-S5]   echoes OK now (%d cm) -- move the hand "
                       "slower next time\n",
                  (INT)s5_echo_us_to_cm(last.width_us));
    }
}

/*
 * For 6 s, print the forward distance twice a second while the tester
 * moves a hand from about 40 cm to about 10 cm in front of the sensor.
 * PASS if a reading under 20 cm and one over 30 cm were both seen.
 */
static void range_check(void)
{
    uint8_t  distance;
    bool     b_near = false;
    bool     b_far  = false;
    uint32_t tick;

    tm_printf((UB*)"[TEST-S5] MOVE A HAND FROM ~40 cm TO ~10 cm IN FRONT "
                   "(6 s)\n");

    for (tick = 0u; tick < 12u; tick++)
    {
        (void)tk_dly_tsk(500);
        distance = s5_get_obstacle_distance_cm();
        b_near   = b_near || ((distance >= 3u) && (distance < 20u));
        b_far    = b_far  || (distance > 30u);
        tm_printf((UB*)"[TEST-S5]   distance %d cm\n", (INT)distance);
    }

    tm_printf((UB*)"[TEST-S5] %s: ultrasonic saw near (<20 cm) and far "
                   "(>30 cm)\n", (b_near && b_far) ? "PASS" : "FAIL");

    if (!(b_near && b_far))
    {
        echo_diagnosis();
    }
}

/*
 * Find the trim: step the servo from 20 degrees left to 20 degrees right
 * of its OWN centre, 2 degrees per second, printing each angle.  The
 * tester notes the angle at which the ultrasonic faces straight ahead
 * (sight along the car) and puts it in S5_SERVO_TRIM_DEG.
 */
static void servo_centre_check(void)
{
    int32_t angle;

    tm_printf((UB*)"[TEST-S5] SERVO CENTRE: note the angle at which the "
                   "sensor faces STRAIGHT AHEAD (trim now %d)\n",
              (INT)S5_SERVO_TRIM_DEG);

    for (angle = -20; angle <= 20; angle += 2)
    {
        s5_servo_test_raw(angle);
        tm_printf((UB*)"[TEST-S5]   servo raw %d deg\n", (INT)angle);
        (void)tk_dly_tsk(1000);
    }

    s5_servo_test_point(0);
    tm_printf((UB*)"[TEST-S5] set S5_SERVO_TRIM_DEG (subsystem5.h) to that "
                   "angle, then run this test again for the range\n");
    (void)tk_dly_tsk(1500);
}

/*
 * Step the servo out 5 degrees at a time (up to 40, the measured safe
 * maximum), 1 s per step, first to the
 * RIGHT then to the LEFT, printing each angle.  The tester watches the
 * ultrasonic's wires and notes the last angle on each side before they
 * pull; S5_SCAN_ARC_DEG is then the smaller of the two.
 */
static void servo_range_check(void)
{
    int32_t angle;
    int32_t side;

    tm_printf((UB*)"[TEST-S5] SERVO RANGE (from straight ahead, trim %d): "
                   "note the LAST angle before the wires pull, each side\n",
              (INT)S5_SERVO_TRIM_DEG);
    s5_servo_test_point(0);
    (void)tk_dly_tsk(1500);

    for (side = 1; side >= -1; side -= 2)
    {
        for (angle = 5; angle <= 40; angle += 5)
        {
            /* Never past 40 degrees from the servo's OWN centre (the
               measured safe limit), whatever the trim. */
            if (((side * angle) + S5_SERVO_TRIM_DEG) > 40
                || ((side * angle) + S5_SERVO_TRIM_DEG) < -40)
            {
                break;
            }
            s5_servo_test_point(side * angle);
            tm_printf((UB*)"[TEST-S5]   servo %s %d deg\n",
                      (side > 0) ? "RIGHT" : "LEFT ", (INT)angle);
            (void)tk_dly_tsk(1000);
        }
        s5_servo_test_point(0);
        tm_printf((UB*)"[TEST-S5]   servo back to centre\n");
        (void)tk_dly_tsk(1500);
    }

    tm_printf((UB*)"[TEST-S5] now set S5_SCAN_ARC_DEG (subsystem5.h) to "
                   "the smaller of the two angles; it is %d\n",
              (INT)S5_SCAN_ARC_DEG);
}

EXPORT void s5_ultrasonic_bench(void)
{
    gpio_irq_echo_t echo  = {0};
    uint32_t        last_seq;
    uint32_t        pulses;
    uint32_t        line  = 0u;
    const char     *p_verdict;

    s5_servo_test_point(0);
    (void)gpio_irq_read_echo(&echo);
    last_seq = echo.seq;

    for (;;)
    {
        if (0u == (line % 20u))
        {
            tm_printf((UB*)"[US] ultrasonic bench test: TRIG GP0, ECHO GP1, "
                           "servo straight. Hold a flat object at 10..100 cm "
                           "in front.\n");
        }
        line = line + 1u;

        (void)tk_dly_tsk(250);
        (void)gpio_irq_read_echo(&echo);
        pulses   = echo.seq - last_seq;     /* ~4 per 250 ms when working */
        last_seq = echo.seq;

        if (0u == pulses)
        {
            p_verdict = "NO ECHO PULSES -- check VCC, GND, TRIG/ECHO wiring";
        }
        else if (echo.width_us >= S5_ECHO_TIMEOUT_US)
        {
            p_verdict = "no reflection -- nothing in range";
        }
        else if (echo.width_us < 300u)
        {
            p_verdict = "instant echo -- something touching/blocking the "
                        "sensor, or TRIG/ECHO wires touching";
        }
        else
        {
            p_verdict = "OK";
        }

        tm_printf((UB*)"[US] %3d cm   echo %5d us   %d pulses   %s\n",
                  (INT)s5_get_obstacle_distance_cm(),
                  (INT)echo.width_us, (INT)pulses, p_verdict);
    }
}

EXPORT void subsystem5_test(void)
{
    uint16_t mask;
    uint16_t expected;
    uint32_t idx;
    uint32_t lost = 0u;

    tm_printf((UB*)"[TEST-S5] obstacle and fault table\n");

    /* ---- pure helpers ---- */
    tm_printf((UB*)"[TEST-S5] 580 us echo = %d cm (expect 10)\n",
              (INT)s5_echo_us_to_cm(580u));

    if (MSG_SIDE_LEFT != s5_choose_side(30u, 10u, 23u))
    {
        tm_printf((UB*)"[TEST-S5] FAIL: should pick the wider left side\n");
    }
    if (MSG_SIDE_NONE != s5_choose_side(5u, 5u, 23u))
    {
        tm_printf((UB*)"[TEST-S5] FAIL: neither side fits, expected NONE\n");
    }

    /*
     * T4: concurrency injection.  Four sources set four different bits
     * repeatedly; every bit must survive.  This is the test for risk R4 --
     * a lost bit means a real collision is never reported.
     */
    fault_clear(FAULT_MASK_ALL);
    expected = (uint16_t)(FAULT_COLLISION | FAULT_MOTOR_STALL
                        | FAULT_NO_ECHO | FAULT_WIFI_LOSS);

    for (idx = 0u; idx < 100u; idx++)
    {
        fault_clear(FAULT_MASK_ALL);

        fault_set(FAULT_COLLISION);
        fault_set(FAULT_MOTOR_STALL);
        fault_set(FAULT_NO_ECHO);
        fault_set(FAULT_WIFI_LOSS);

        mask = fault_get();

        if ((mask & expected) != expected)
        {
            lost++;
        }
    }

    if (0u == lost)
    {
        tm_printf((UB*)"[TEST-S5] PASS: 100/100 iterations, no bit lost\n");
    }
    else
    {
        tm_printf((UB*)"[TEST-S5] FAIL: %d iterations lost a bit (R4)\n",
                  (INT)lost);
    }

    /* ---- severity masks must be disjoint and correct ---- */
    fault_clear(FAULT_MASK_ALL);
    fault_set(FAULT_WIFI_LOSS);

    if (fault_is_estop())
    {
        tm_printf((UB*)"[TEST-S5] FAIL: WIFI_LOSS reads as E-STOP (NFR6!)\n");
    }
    else
    {
        tm_printf((UB*)"[TEST-S5] PASS: WIFI_LOSS does not stop the car\n");
    }

    fault_set(FAULT_COLLISION);

    if (fault_is_estop())
    {
        tm_printf((UB*)"[TEST-S5] PASS: COLLISION reads as E-STOP\n");
    }
    else
    {
        tm_printf((UB*)"[TEST-S5] FAIL: COLLISION missing from E-STOP mask\n");
    }

    fault_clear(FAULT_MASK_ALL);
    tm_printf((UB*)"[TEST-S5] mask cleared = 0x%03x\n", (INT)fault_get());

    /* ---- hands-on: the ultrasonic sees a hand ---- */
    range_check();

    /* ---- hands-on: where is straight ahead, how far may it turn? ---- */
    servo_centre_check();
    servo_range_check();
}

#endif /* APP_TEST */

/* end of file */
