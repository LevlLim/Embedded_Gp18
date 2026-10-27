/*
 * subsystem1.c
 *
 * Subsystem 1 -- telemetry: creates its kernel objects and starts the
 * telemetry task.
 */

#include "subsystem1.h"

EXPORT ER subsystem1_init(void)
{
    /* The mutex and the outbox are created next to the code that uses
       them, so their IDs stay private to telemetry_comms.c. */
    return (s1_comms_init());
}

EXPORT ER subsystem1_start(void)
{
    ID tid;

    /*
     * Pinned to core 1 (TP_PRC2), away from the driving tasks.  Note that
     * the template's radio/lwIP task itself is on core 0; see app.h.
     */
    tid = app_start_periodic_task((FP)telemetry_task, PRI_TELEMETRY,
                                  TP_PRC2, "telemetry_task",
                                  PERIOD_TELEMETRY_MS);

    return ((tid < E_OK) ? (ER)tid : E_OK);
}

/* end of file */
