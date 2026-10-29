/**
 * @file barcode_bench.c
 * @brief Opt-in stationary GP28 scanner. No controller or motion task starts.
 */
#include "subsystem3.h"

#if S3_BARCODE_BENCH
#include "adc_sampler.h"
#include "../subsystem2/motor_control.h"

#define BENCH_REPORT_MS (50)
#define BENCH_STATUS_MS (1000U)

static void bench_report_task(INT stacd, void * p_exinf)
{
    s3_barcode_result_t discarded = {0};
    adc_sample_t sample = {0U, 0U};
    uint32_t last_status_ms = app_now_ms();
    char const * p_surface = "band";

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        /* Consume notices so repeated scans never depend on controller state.
         * Only this low-priority task prints; the 1 ms decoder never prints.
         */
        (void)s3_take_barcode_result(&discarded);
        s3_report_barcode();
        if ((app_now_ms() - last_status_ms) >= BENCH_STATUS_MS)
        {
            last_status_ms = app_now_ms();
            if (adc_sampler_read(ADC_CH_BARCODE, &sample))
            {
                p_surface = "band";
                if (sample.value >= S3_AIR_LEVEL)
                {
                    p_surface = "no surface";
                }
                else if (sample.value > (S3_BARCODE_THRESHOLD + S3_FIXED_BAND / 2U))
                {
                    p_surface = "black";
                }
                else if (sample.value < (S3_BARCODE_THRESHOLD - S3_FIXED_BAND / 2U))
                {
                    p_surface = "white";
                }
                tm_printf((UB *)"[BENCH] GP28 ADC %d: %s; motors stopped\n",
                          (INT)sample.value, p_surface);
            }
            else
            {
                tm_printf((UB *)"[BENCH] GP28 samples unavailable; check sensor\n");
            }
        }
        (void)tk_dly_tsk(BENCH_REPORT_MS);
    }
}

INT s3_barcode_bench_main(void)
{
    ER error = motor_init();
    ID task = (ID)0;

    /* Initialise the existing motor driver to zero duty, but never start
     * Motion, Controller, IMU or telemetry tasks. START has no drive action.
     */
    motor_brake();
    if (E_OK == error)
    {
        error = subsystem3_init();
    }
    if (E_OK == error)
    {
        error = s3_prepare();
    }
    if (E_OK == error)
    {
        task = app_start_periodic_task((FP)barcode_task, PRI_BARCODE,
            TP_PRC1, "barcode_task", PERIOD_BARCODE_MS);
        error = (task > (ID)0) ? E_OK : E_OBJ;
    }
    if (E_OK == error)
    {
        task = app_start_task((FP)bench_report_task, PRI_TELEMETRY,
                             TP_PRC2, "barcode_bench_report");
        error = (task > (ID)0) ? E_OK : E_OBJ;
    }
    if (E_OK == error)
    {
        s3_set_active(true);
        tm_printf((UB *)"\n[BENCH] READY: hand-scan GP28; motors stopped; no START needed\n");
        tm_printf((UB *)"[BENCH] Hold white 3 s, slide the whole *letter* smoothly, "
                        "then hold white 3 s\n");
        tm_printf((UB *)"[BENCH] Limits: 2..1000 ms per element, 20 s per scan\n");
    }
    else
    {
        s3_set_active(false);
        motor_brake();
        tm_printf((UB *)"[BENCH] setup FAILED (%d); motors stopped; reboot to retry\n",
                  (INT)error);
    }
    (void)tk_slp_tsk(TMO_FEVR);
    return (0);
}
#endif /* S3_BARCODE_BENCH */

/* end of file */
