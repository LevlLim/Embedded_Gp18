/*
 * subsystem3_test.c
 * On-target tests for subsystem 3 (line and barcode).  Built with TEST=1.
 */

#include "subsystem3.h"
#include "adc_sampler.h"

#if APP_TEST

/* One synthetic decode, printed against its expected result. */
static void check_decode(uint16_t width_a, uint16_t width_b,
                         uint16_t width_c, uint16_t width_d,
                         msg_barcode_cmd_t expect, const char *p_label)
{
    uint16_t          widths[4];
    uint8_t           confidence = 0u;
    msg_barcode_cmd_t decoded;

    widths[0] = width_a;
    widths[1] = width_b;
    widths[2] = width_c;
    widths[3] = width_d;

    decoded = s3_decode_widths(widths, 4u, &confidence);

    tm_printf((UB*)"[TEST-S3] %s: %s (cmd %d conf %d)\n", p_label,
              (expect == decoded) ? "PASS" : "FAIL",
              (INT)decoded, (INT)confidence);
}

/*
 * For 6 s, print both line sensors four times a second while the tester
 * slides the car slowly across the line.  PASS for a sensor that read
 * above its threshold (black) at least once AND below it (white) at least
 * once.
 */
static void sweep_check(void)
{
    adc_sample_t side   = {0};
    adc_sample_t centre = {0};
    uint16_t     thr_side;
    uint16_t     thr_ctr;
    bool         b_side_b = false;
    bool         b_side_w = false;
    bool         b_ctr_b  = false;
    bool         b_ctr_w  = false;
    uint32_t     tick;

    s3_get_thresholds(&thr_side, &thr_ctr);
    tm_printf((UB*)"[TEST-S3] SLIDE THE CAR SLOWLY ACROSS THE LINE (6 s)\n");

    for (tick = 0u; tick < 24u; tick++)
    {
        (void)tk_dly_tsk(250);
        (void)adc_sampler_read(ADC_CH_LINE_L, &side);
        (void)adc_sampler_read(ADC_CH_LINE_C, &centre);

        b_side_b = b_side_b || (side.value > thr_side);
        b_side_w = b_side_w || (side.value <= thr_side);
        b_ctr_b  = b_ctr_b  || (centre.value > thr_ctr);
        b_ctr_w  = b_ctr_w  || (centre.value <= thr_ctr);

        tm_printf((UB*)"[TEST-S3]   side %4d/%4d %s   centre %4d/%4d %s\n",
                  (INT)side.value, (INT)thr_side,
                  (side.value > thr_side) ? "B" : "W",
                  (INT)centre.value, (INT)thr_ctr,
                  (centre.value > thr_ctr) ? "B" : "W");
    }

    tm_printf((UB*)"[TEST-S3] %s: side sensor saw black and white\n",
              (b_side_b && b_side_w) ? "PASS" : "FAIL");
    tm_printf((UB*)"[TEST-S3] %s: centre sensor saw black and white\n",
              (b_ctr_b && b_ctr_w) ? "PASS" : "FAIL");
}

EXPORT void subsystem3_test(void)
{
    msg_line_t line;
    uint32_t   word;

    tm_printf((UB*)"[TEST-S3] line and barcode\n");

    /* ---- the ADC is actually running, and never overruns ---- */
    adc_sampler_selftest();

    if (adc_sampler_overruns() > 0u)
    {
        tm_printf((UB*)"[TEST-S3] FAIL: %d ADC overruns\n",
                  (INT)adc_sampler_overruns());
    }

    /* ---- line error is published, timestamped and fresh ---- */
    s3_set_active(true);
    (void)tk_dly_tsk(200);

    if (s3_read_line_word(&word))
    {
        msg_unpack_line(word, &line);
        tm_printf((UB*)"[TEST-S3] line err=%d valid=%d age=%d ms\n",
                  (INT)line.error, (INT)line.b_valid,
                  (INT)msg_line_age_ms(word, app_now_ms()));
    }
    else
    {
        tm_printf((UB*)"[TEST-S3] FAIL: no line reading published\n");
    }

    /* ---- hands-on: both line sensors see black AND white ---- */
    sweep_check();

    /* ---- barcode classification, synthetic widths ---- */
    check_decode(1000u, 1000u, 1000u, 2500u, MSG_BC_STRAIGHT, "N,N,N,W");
    check_decode(2500u, 1000u, 1000u, 1000u, MSG_BC_U_TURN, "W,N,N,N");
    check_decode(10u, 10u, 10u, 10u, MSG_BC_INVALID, "noise widths");

    s3_set_active(false);
}

#endif /* APP_TEST */

/* end of file */
