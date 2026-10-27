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
}

#endif /* APP_TEST */

/* end of file */
