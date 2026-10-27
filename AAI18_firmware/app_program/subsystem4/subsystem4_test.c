/*
 * subsystem4_test.c
 * On-target tests for subsystem 4 (IMU and terrain).
 */

#include "subsystem4.h"
#include "i2c0.h"
#include "../fault.h"

#if APP_TEST

/*
 * For 6 s, print the tilt twice a second while the tester lifts the
 * car's nose and then its tail.  PASS if the readings spread over at
 * least 20 degrees (about +10 to -10).
 */
static void tilt_check(void)
{
    int32_t  tilt;
    int32_t  lowest  = 127;
    int32_t  highest = -128;
    uint32_t tick;

    tm_printf((UB*)"[TEST-S4] LIFT THE NOSE, THEN THE TAIL (6 s)\n");

    for (tick = 0u; tick < 12u; tick++)
    {
        (void)tk_dly_tsk(500);
        tilt    = (int32_t)s4_get_tilt_deg();
        lowest  = (tilt < lowest)  ? tilt : lowest;
        highest = (tilt > highest) ? tilt : highest;
        tm_printf((UB*)"[TEST-S4]   tilt %d deg\n", (INT)tilt);
    }

    if ((highest - lowest) >= 20)
    {
        tm_printf((UB*)"[TEST-S4] PASS: tilt range %d..%d deg\n",
                  (INT)lowest, (INT)highest);
    }
    else
    {
        tm_printf((UB*)"[TEST-S4] FAIL: tilt only %d..%d deg -- IMU "
                       "stuck or car not tilted\n",
                  (INT)lowest, (INT)highest);
    }
}

EXPORT void subsystem4_test(void)
{
    uint8_t who = 0u;
    int32_t increment;

    tm_printf((UB*)"[TEST-S4] IMU and terrain\n");

    /* ---- the sensor answers ---- */
    if (i2c0_read_reg(S4_ACCEL_ADDR, S4_REG_WHO_AM_I, &who) >= E_OK)
    {
        tm_printf((UB*)"[TEST-S4] PASS: IMU WHO_AM_I = 0x%02x\n", (INT)who);
    }
    else
    {
        tm_printf((UB*)"[TEST-S4] FAIL: IMU silent -- check I2C0 wiring\n");
    }

    tm_printf((UB*)"[TEST-S4] i2c errors so far = %d\n",
              (INT)i2c0_error_count());

    /* ---- height accumulation is monotonic and signed correctly ---- */
    increment = s4_height_increment_um(10, 100);
    tm_printf((UB*)"[TEST-S4] 10 deg over 100 mm = %d um (expect ~17400)\n",
              (INT)increment);

    if (increment <= 0)
    {
        tm_printf((UB*)"[TEST-S4] FAIL: positive tilt gave no rise\n");
    }

    /* A level chassis must contribute no height, or a flat run would
       accumulate a phantom hump. */
    increment = s4_height_increment_um(0, 1000);

    if (0 == increment)
    {
        tm_printf((UB*)"[TEST-S4] PASS: level chassis adds no height\n");
    }
    else
    {
        tm_printf((UB*)"[TEST-S4] FAIL: level added %d um\n", (INT)increment);
    }

    /* Pitch: 10 deg is tan = 0.176, i.e. 176 mg over 1000 mg. */
    tm_printf((UB*)"[TEST-S4] pitch(176, 1000) = %d deg (expect 10)\n",
              (INT)s4_pitch_deg(176, 1000));

    tm_printf((UB*)"[TEST-S4] tilt now %d deg, state %d\n",
              (INT)s4_get_tilt_deg(), (INT)s4_get_state());

    /* ---- hands-on: tilt follows the car ---- */
    tilt_check();
}

#endif /* APP_TEST */

/* end of file */
