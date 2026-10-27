/*
 * subsystem2_test.c
 * On-target tests for subsystem 2 (Motion Control).
 *
 * WARNING: this test drives the motors.  Put the car on blocks.
 */

#include "subsystem2.h"
#include "motor_control.h"
#include "../fault.h"
#include "../drivers/gpio_irq.h"

#if APP_TEST

/*
 * Verdict for one wheel after the 100 mm FORWARD move: it must count, and
 * count UP.  Counting down means that encoder's A/B wires (or the motor's
 * wires) are the other way round from what the firmware expects.
 */
static void report_wheel(const char *p_name, int32_t delta)
{
    if (delta > 0)
    {
        tm_printf((UB*)"[TEST-S2] PASS: %s encoder +%d counts forward\n",
                  p_name, (INT)delta);
    }
    else if (delta < 0)
    {
        tm_printf((UB*)"[TEST-S2] FAIL: %s encoder counts BACKWARDS (%d) "
                       "-- swap its A/B wires\n", p_name, (INT)delta);
    }
    else
    {
        tm_printf((UB*)"[TEST-S2] FAIL: %s encoder no edges -- check "
                       "wiring / motor power\n", p_name);
    }
}

EXPORT void subsystem2_test(void)
{
    msg_motion_cmd_t cmd;
    gpio_irq_encoders_t enc;
    uint32_t t_start;
    uint32_t t_brake;
    int32_t  before_left;
    int32_t  before_right;

    tm_printf((UB*)"[TEST-S2] motion control -- PUT THE CAR ON BLOCKS\n");
    (void)tk_dly_tsk(3000);

    /* ---- geometry conversions ---- */
    tm_printf((UB*)"[TEST-S2] 100 mm = %d counts, 90 deg = %d counts\n",
              (INT)motor_mm_to_counts(100), (INT)motor_degrees_to_counts(90));

    /* ---- encoders actually count ---- */
    gpio_irq_read_encoders(&enc);
    before_left  = enc.count_left;
    before_right = enc.count_right;
    tm_printf((UB*)"[TEST-S2] encoders before: L=%d R=%d\n",
              (INT)enc.count_left, (INT)enc.count_right);

    cmd.type = MSG_CMD_FORWARD;
    cmd.arg  = 100u;
    cmd.seq  = 1u;
    (void)s2_send_command(&cmd);
    (void)tk_dly_tsk(2000);

    gpio_irq_read_encoders(&enc);
    tm_printf((UB*)"[TEST-S2] encoders after:  L=%d R=%d\n",
              (INT)enc.count_left, (INT)enc.count_right);

    report_wheel("left ", enc.count_left - before_left);
    report_wheel("right", enc.count_right - before_right);

    /*
     * T5 (partial): e-stop reaction time measured in SOFTWARE.
     *
     * This is NOT the full T5 evidence.  The design review asks for a
     * logic-analyser capture from the fault source to both motor pins
     * going LOW, because only that includes the hardware path.  What this
     * measures is the time from setting the bit to Motion Control
     * reporting M_ESTOP, which is the software contribution alone.
     */
    t_start = app_now_ms();
    fault_set(FAULT_COLLISION);

    while ((M_ESTOP != s2_get_state())
           && ((app_now_ms() - t_start) < 500u))
    {
        (void)tk_dly_tsk(1);
    }

    t_brake = app_now_ms() - t_start;
    tm_printf((UB*)"[TEST-S2] e-stop software path = %d ms (FR11 limit 100)\n",
              (INT)t_brake);

    if (t_brake > 100u)
    {
        tm_printf((UB*)"[TEST-S2] FAIL: FR11 missed in software alone\n");
    }
    else
    {
        tm_printf((UB*)"[TEST-S2] PASS (software only; T5 owed)\n");
    }

    fault_clear(FAULT_MASK_ALL);

    tm_printf((UB*)"[TEST-S2] max IO_BANK0 dispatcher = %d us (risk R8)\n",
              (INT)gpio_irq_max_duration_us());
}

#endif /* APP_TEST */

/* end of file */
