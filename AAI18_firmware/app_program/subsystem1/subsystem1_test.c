/*
 * subsystem1_test.c
 * On-target tests for subsystem 1.  Built only with TEST=1.
 *
 * These run in the initial task while the system is live, so they
 * exercise real timing, not a simulation.
 */

#include "subsystem1.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
    uint32_t hold_us;

    tm_printf((UB*)"[TEST-S1] telemetry\n");

    s1_reset_stats();
    (void)tk_dly_tsk(2000);         /* let ~10 frames publish */

    /*
     * T1 evidence: the longest the telemetry mutex was held.  This bounds
     * Motion Control's worst-case blocking.  If this grows beyond a few
     * hundred microseconds, something other than a struct copy has crept
     * into the critical section.
     */
    hold_us = s1_max_hold_us();
    tm_printf((UB*)"[TEST-S1] max mutex hold = %d us\n", (INT)hold_us);

    if (hold_us > 500u)
    {
        tm_printf((UB*)"[TEST-S1] WARN: hold time high, check R6\n");
    }

    /* Stale-command rejection: the same sequence twice must be refused. */
    (void)s1_accept_command(10u, S1_CMD_STOP);

    if (s1_accept_command(10u, S1_CMD_START))
    {
        tm_printf((UB*)"[TEST-S1] FAIL: duplicate seq accepted\n");
    }
    else
    {
        tm_printf((UB*)"[TEST-S1] PASS: duplicate seq rejected\n");
    }

    if (s1_accept_command(11u, S1_CMD_RESET))
    {
        tm_printf((UB*)"[TEST-S1] PASS: advancing seq accepted\n");
    }
    else
    {
        tm_printf((UB*)"[TEST-S1] FAIL: advancing seq rejected\n");
    }
}

#endif /* APP_TEST */

/* end of file */
