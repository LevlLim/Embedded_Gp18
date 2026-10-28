/*
 * subsystem3.c
 *
 * Subsystem 3 -- line and barcode: brings up the ADC sampler and the
 * barcode message buffer, then starts the two tasks.
 *
 * This subsystem also brings up subsystem3/adc_sampler, because Buddy 3 owns
 * the ADC (Design Review V2 section 9).
 */

#include "subsystem3.h"
#include "adc_sampler.h"

/* Preserve successful creations if a later startup step needs a retry. */
static ID g_line_task_id = (ID)0;
static ID g_barcode_task_id = (ID)0;

EXPORT ER subsystem3_init(void)
{
    ER ercd;

    /* Bring up the ADC before anything tries to read a sample. */
    ercd = adc_sampler_init();

    if (E_OK == ercd)
    {
        ercd = s3_barcode_init();
    }

    return (ercd);
}

EXPORT ER subsystem3_start(void)
{
    ID tid = (ID)0;

    if ((ID)0 == g_line_task_id)
    {
        tid = app_start_periodic_task((FP)line_task, PRI_LINE_SENSOR,
                                      TP_PRC1, "line_task", PERIOD_LINE_MS);
        if (tid <= (ID)0)
        {
            return ((tid < (ID)0) ? (ER)tid : E_OBJ);
        }
        g_line_task_id = tid;
    }

    if ((ID)0 == g_barcode_task_id)
    {
        tid = app_start_periodic_task((FP)barcode_task, PRI_BARCODE,
                                      TP_PRC1, "barcode_task",
                                      PERIOD_BARCODE_MS);
        if (tid <= (ID)0)
        {
            return ((tid < (ID)0) ? (ER)tid : E_OBJ);
        }
        g_barcode_task_id = tid;
    }

    return (E_OK);
}

/* end of file */
