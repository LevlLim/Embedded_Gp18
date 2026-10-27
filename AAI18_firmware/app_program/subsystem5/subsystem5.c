/*
 * subsystem5.c
 *
 * Subsystem 5 -- obstacle and Vehicle Controller: creates the shared fault
 * bitmask, the event flag and the obstacle buffer, configures the servo,
 * trigger and buttons, and starts both tasks.
 */

#include "subsystem5.h"
#include "../fault.h"

EXPORT ER subsystem5_init(void)
{
    ER ercd;

    /* The shared fault bitmask belongs to this subsystem (Fault Table). */
    ercd = fault_init();

    if (E_OK == ercd)
    {
        ercd = s5_controller_init();
    }

    if (E_OK == ercd)
    {
        ercd = s5_obstacle_init();
    }

    return (ercd);
}

EXPORT ER subsystem5_start(void)
{
    ID tid;

    /* The Controller is event-driven: an ordinary task. */
    tid = app_start_task((FP)controller_task, PRI_VEHICLE_CTRL,
                         TP_PRC1, "controller_task");

    if (tid >= E_OK)
    {
        tid = app_start_periodic_task((FP)obstacle_task, PRI_OBSTACLE_SCAN,
                                      TP_PRC1, "obstacle_task",
                                      PERIOD_OBSTACLE_MS);
    }

    return ((tid < E_OK) ? (ER)tid : E_OK);
}

/* end of file */
