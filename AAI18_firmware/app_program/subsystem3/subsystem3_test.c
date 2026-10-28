/**
 * @file subsystem3_test.c
 * @brief On-car freshness checks plus the real Code 39 decode contract.
 */
#include "subsystem3.h"
#include "adc_sampler.h"
#include "barcode_mapping.h"
#if APP_TEST
void subsystem3_test(void)
{
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    uint16_t const patterns[3] = {0x094U, 0x109U, 0x094U};
    s3_barcode_result_t result = {0};
    msg_barcode_cmd_t direction = MSG_BC_NONE;
    adc_sample_t sample = {0U, 0U};
    uint8_t character = 0U;
    uint8_t element = 0U;
    uint8_t count = 0U;
    uint8_t channel = 0U;
    uint16_t left_threshold = 0U;
    uint16_t right_threshold = 0U;
    bool b_valid = false;

    APP_LOG((UB *)"[TEST-S3] edge hugging: GP26 LEFT, GP27 RIGHT, "
                   "GP28 BARCODE\n");
    APP_LOG((UB *)"[TEST-S3] L/R: W/W err 0; B/W err -%d; "
                   "W/B err +%d; B/B err 0, junction after %d samples\n",
              (INT)S3_LINE_CORRECTION, (INT)S3_LINE_CORRECTION,
              (INT)S3_JUNCTION_CONFIRM_SAMPLES);
    s3_get_thresholds(&left_threshold, &right_threshold);
    APP_LOG((UB *)"[TEST-S3] thresholds: left %d, right %d "
                   "(black ADC is higher)\n",
              (INT)left_threshold, (INT)right_threshold);
    APP_LOG((UB *)"[TEST-S3] both white also occurs off-track; "
                   "it cannot prove line reacquisition\n");
    adc_sampler_selftest();
    for (channel = 0U; channel < ADC_CH_COUNT; channel++)
    {
        b_valid = adc_sampler_read(channel, &sample);
        APP_LOG((UB *)"[TEST-S3] fresh ch%d: %s\n", (INT)channel,
                  b_valid ? "PASS" : "FAIL");
    }
    for (character = 0U; character < 3U; character++)
    {
        for (element = 0U; element < S3_CHAR_ELEMENTS; element++)
        {
            widths[count] = (0U != (patterns[character]
                & (uint16_t)(1U << (S3_CHAR_ELEMENTS - 1U - element))))
                ? 10000U : 4000U;
            count++;
        }
        if (character < 2U)
        {
            widths[count] = 4000U;
            count++;
        }
    }
    direction = barcode_mapping_direction((uint8_t)'A');
    b_valid = s3_decode_code39(widths, count, 60000U, 60000U, &result)
        && ((uint8_t)'A' == result.letter)
        && (direction >= MSG_BC_STRAIGHT) && (direction <= MSG_BC_U_TURN);
    APP_LOG((UB *)"[TEST-S3] *A* -> A -> %s: %s\n",
              barcode_mapping_name(direction),
              b_valid ? "PASS" : "FAIL");
    APP_LOG((UB *)"[TEST-S3] physical scan/lighting/speed tests still "
                   "require the track; no synthetic motor command sent\n");
    APP_LOG((UB *)"[TEST-S3] max execution: ADC %d, line %d, barcode %d us\n",
              (INT)adc_sampler_max_us(), (INT)s3_line_max_us(),
              (INT)s3_barcode_max_us());
}
#endif
