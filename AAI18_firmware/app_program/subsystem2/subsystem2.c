/*
 * subsystem2.c
 *
 * Subsystem 2 -- Motion Control: brings up the motors, the shared IO_BANK0
 * dispatcher and the command buffer, then starts its task.
 */

#include "subsystem2.h"
#include "motor_control.h"
#include "../drivers/gpio_irq.h"

EXPORT ER subsystem2_init(void)
{
    ER ercd;

    ercd = motor_init();

    /* The IO_BANK0 dispatcher is shared with subsystem 5 (ECHO), but it is
       created here because Motion Control is the higher-priority user and
       must exist before any wheel turns. */
    if (E_OK == ercd)
    {
        ercd = gpio_irq_init();
    }

    if (E_OK == ercd)
    {
        ercd = s2_motion_init();
    }

    return (ercd);
}

EXPORT ER subsystem2_start(void)
{
    ID tid;

    /* Pinned to core 0 (TP_PRC1) with the sensing tasks it reads from. */
    tid = app_start_periodic_task((FP)motion_task, PRI_MOTION_CONTROL,
                                  TP_PRC1, "motion_task", PERIOD_MOTION_MS);

    return ((tid < E_OK) ? (ER)tid : E_OK);
}

/* end of file */
