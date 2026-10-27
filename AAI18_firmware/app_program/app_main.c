/*
 * app_main.c
 *
 * Group AAI18 -- entry point.
 *
 * usermain() runs inside the kernel's initial task at priority 1.  It
 * works in two phases so that every kernel object exists before any task
 * can use one:
 *   phase 1  subsystemN_init()   create kernel objects, bring up hardware
 *   phase 2  subsystemN_start()  create and start tasks
 * and then sleeps forever.  Returning would shut the system down.
 *
 * CORE ASSIGNMENT
 * ---------------
 * Every driving task is pinned to core 0 and Telemetry to core 1, so the
 * Telemetry task's own encoding work never competes with the control
 * loop.  The template's WiFi/lwIP service task, however, is pinned to
 * core 0 at priority 4 (above Motion Control), so network packet handling
 * DOES share core 0.  Anything Telemetry reads from a driving task crosses
 * cores, and DI() masks interrupts on the calling core only -- so shared
 * data read by Telemetry is either a single aligned word or protected by
 * a kernel object, never by DI().
 *
 * PERIODIC TASKS
 * --------------
 * tk_dly_tsk(d) waits d plus one tick, and a loop built on it also adds
 * its own execution time to every period.  Periodic tasks are therefore
 * woken by a cyclic handler instead (app_start_periodic_task), and the
 * kernel tick must be 1 ms (app.h checks this at compile time).
 *
 * INIT ORDER MATTERS
 * ------------------
 * Subsystem 5 is initialised FIRST despite its number, because it creates
 * the shared fault bitmask (fault_init) and the event flag that every
 * other subsystem posts to.  Subsystem 2 then creates the IO_BANK0
 * dispatcher that subsystem 5's ECHO timing depends on.
 */

#include "app.h"
#include "fault.h"
#include "msg_defs.h"
#include "subsystem1/subsystem1.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/subsystem5.h"

/*
 * Cyclic handler body: wake the task whose ID was stored in exinf.  Runs
 * in handler context, so it does nothing else.
 */
static void periodic_wakeup(void *p_exinf)
{
    /* exinf carries the task ID, stored by app_start_periodic_task(). */
    (void)tk_wup_tsk((ID)(uintptr_t)p_exinf);
}

EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *p_name)
{
    T_CTSK ctsk = {0};
    ID     tid;
    ER     ercd;

    ctsk.task    = entry;
    ctsk.itskpri = pri;
    ctsk.stksz   = TASK_STACK_SIZE;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

#if TK_SUPPORT_SMP
    if (0u != prc)
    {
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc  = prc;
    }
#else
    (void)prc;
#endif

    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, p_name);
    }
    else
    {
        ercd = tk_sta_tsk(tid, 0);
        CHECK(ercd, p_name);
    }

    return (tid);
}

EXPORT ID app_start_periodic_task(FP entry, PRI pri, UW prc,
                                  const char *p_name, uint32_t period_ms)
{
    T_CCYC ccyc = {0};
    ID     tid;
    ID     cycid;

    tid = app_start_task(entry, pri, prc, p_name);

    if (tid >= E_OK)
    {
        /* The task ID travels to the handler through exinf. */
        ccyc.exinf  = (void *)(uintptr_t)tid;
        ccyc.cycatr = TA_HLNG | TA_STA;
        ccyc.cychdr = (FP)periodic_wakeup;
        ccyc.cyctim = (RELTIM)period_ms;
        ccyc.cycphs = (RELTIM)period_ms;

        cycid = tk_cre_cyc(&ccyc);
        if (cycid < E_OK)
        {
            CHECK(cycid, p_name);
            tid = (ID)cycid;
        }
    }

    return (tid);
}

EXPORT uint32_t app_wait_period(void)
{
    INT missed;

    (void)tk_slp_tsk(TMO_FEVR);

    /* Wake-ups that queued while the previous cycle overran are dropped,
       so an overrun is reported once instead of replayed as a burst. */
    missed = tk_can_wup(TSK_SELF);

    return ((missed > 0) ? (uint32_t)missed : 0u);
}

EXPORT uint32_t app_now_ms(void)
{
    SYSTIM tim;
    uint32_t now = 0u;

    /*
     * tk_get_tim gives the system time in milliseconds as a 64-bit value
     * split into hi and lo.  The low 32 bits give 49 days of range, and
     * every staleness check that uses this works on unsigned differences,
     * so a wrap is handled.
     */
    if (tk_get_tim(&tim) >= E_OK)
    {
        now = (uint32_t)tim.lo;
    }

    return (now);
}

EXPORT INT usermain(void)
{
    T_RVER ver;

    (void)tk_ref_ver(&ver);
    tm_printf((UB*)"\n=== INF2004 Group AAI18 -- Autonomous Robot Car ===\n");
    tm_printf((UB*)"micro T-Kernel spec 0x%04x product 0x%04x\n",
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf((UB*)"dual-core build, usermain on core %d\n",
              tk_get_prc() - 1);
#else
    tm_printf((UB*)"single-core build\n");
#endif
    tm_printf((UB*)"message schema version %d\n", (INT)MSG_SCHEMA_VERSION);

    /* ---- phase 1: kernel objects and hardware ---- */
    /* S5 first: it owns the fault bitmask and the event flag. */
    CHECK(subsystem5_init(), "subsystem5_init");
    /* S2 next: it creates the shared IO_BANK0 dispatcher. */
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem1_init(), "subsystem1_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem4_init(), "subsystem4_init");

    /* ---- phase 2: tasks ---- */
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem3_start(), "subsystem3_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(subsystem5_start(), "subsystem5_start");
    CHECK(subsystem1_start(), "subsystem1_start");

    tm_printf((UB*)"all subsystems started\n\n");

#if APP_TEST
    /* APP_TEST_BUDDY 0 runs every buddy's self-test; 1..5 runs only that
       buddy's (build with TEST_BUDDY=N, or the CMake "test-bN" preset).
       Every subsystem is still initialised and started above, because the
       tests exercise the real tasks. */
    /* Let the boot calibration (3 s, Vehicle Controller) finish first,
       so the tests do not run on top of it. */
    tm_printf((UB*)"[TEST] waiting 4 s for the boot calibration...\n");
    (void)tk_dly_tsk(4000);

    if (0 == APP_TEST_BUDDY)
    {
        tm_printf((UB*)"[TEST] === test run start: all buddies ===\n");
    }
    else
    {
        tm_printf((UB*)"[TEST] === test run start: Buddy %d only ===\n",
                  (INT)APP_TEST_BUDDY);
    }
    if ((0 == APP_TEST_BUDDY) || (1 == APP_TEST_BUDDY))
    {
        subsystem1_test();
    }
    if ((0 == APP_TEST_BUDDY) || (2 == APP_TEST_BUDDY))
    {
        subsystem2_test();
    }
    if ((0 == APP_TEST_BUDDY) || (3 == APP_TEST_BUDDY))
    {
        subsystem3_test();
    }
    if ((0 == APP_TEST_BUDDY) || (4 == APP_TEST_BUDDY))
    {
        subsystem4_test();
    }
    if ((0 == APP_TEST_BUDDY) || (5 == APP_TEST_BUDDY))
    {
        subsystem5_test();
    }
    tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

    (void)tk_slp_tsk(TMO_FEVR);

    return (0);
}

/* end of file */
