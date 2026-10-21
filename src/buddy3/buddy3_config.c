/**
 * @file buddy3_config.c
 *
 * @brief Configuration defaults and calibration helpers for Buddy 3.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "buddy3_config.h"

#define DEFAULT_BLACK_ADC                    (1000U)
#define DEFAULT_WHITE_ADC                    (3000U)
#define DEFAULT_LINE_THRESHOLD               (500U)
#define DEFAULT_LINE_HYSTERESIS              (100U)
#define DEFAULT_JUNCTION_THRESHOLD           (750U)
#define DEFAULT_LEFT_WEIGHT_X100             (200U)
#define DEFAULT_BARCODE_THRESHOLD            (500U)
#define DEFAULT_BARCODE_HYSTERESIS           (100U)
#define DEFAULT_BARCODE_MIN_ELEMENT_MS       (2U)
#define DEFAULT_BARCODE_MAX_ELEMENT_MS       (500U)
#define DEFAULT_BARCODE_WIDE_RATIO_X100      (180U)
#define DEFAULT_LINE_SAMPLE_DIVIDER          (5U)
#define DEFAULT_JUNCTION_CONFIRM_SAMPLES     (3U)
#define DEFAULT_LINE_SEARCH_ERROR            (1000)
#define DEFAULT_LINE_LOST_TIME_MS            (1000U)
#define DEFAULT_BARCODE_QUIET_TIME_MS        (20U)
#define DEFAULT_BARCODE_TIMEOUT_MS           (2500U)
#define PERCENT_SCALE                        (100U)

static bool calibration_is_valid (
                buddy3_ir_calibration_t const * const p_calibration);

void buddy3_config_get_default (buddy3_config_t * const p_config)
{
    if (NULL != p_config)
    {
        /*
         * NOTE: These ADC levels are safe placeholder values for software
         * bring-up only. Replace them with measured black and white readings
         * from each installed sensor before track validation.
         */
        p_config->left_calibration.black_adc       = DEFAULT_BLACK_ADC;
        p_config->left_calibration.white_adc       = DEFAULT_WHITE_ADC;
        p_config->center_calibration.black_adc     = DEFAULT_BLACK_ADC;
        p_config->center_calibration.white_adc     = DEFAULT_WHITE_ADC;
        p_config->barcode_calibration.black_adc    = DEFAULT_BLACK_ADC;
        p_config->barcode_calibration.white_adc    = DEFAULT_WHITE_ADC;
        p_config->line_threshold                   = DEFAULT_LINE_THRESHOLD;
        p_config->line_hysteresis                  = DEFAULT_LINE_HYSTERESIS;
        p_config->junction_threshold               = DEFAULT_JUNCTION_THRESHOLD;
        p_config->left_weight_x100                 = DEFAULT_LEFT_WEIGHT_X100;
        p_config->barcode_threshold                = DEFAULT_BARCODE_THRESHOLD;
        p_config->barcode_hysteresis               = DEFAULT_BARCODE_HYSTERESIS;
        p_config->barcode_min_element_ms           = DEFAULT_BARCODE_MIN_ELEMENT_MS;
        p_config->barcode_max_element_ms           = DEFAULT_BARCODE_MAX_ELEMENT_MS;
        p_config->barcode_min_wide_ratio_x100      = DEFAULT_BARCODE_WIDE_RATIO_X100;
        p_config->line_sample_divider              = DEFAULT_LINE_SAMPLE_DIVIDER;
        p_config->junction_confirm_samples         = DEFAULT_JUNCTION_CONFIRM_SAMPLES;
        p_config->line_search_error                = DEFAULT_LINE_SEARCH_ERROR;
        p_config->line_lost_time_ms                = DEFAULT_LINE_LOST_TIME_MS;
        p_config->barcode_quiet_time_ms            = DEFAULT_BARCODE_QUIET_TIME_MS;
        p_config->barcode_timeout_ms               = DEFAULT_BARCODE_TIMEOUT_MS;
    }
}

bool buddy3_config_is_valid (buddy3_config_t const * const p_config)
{
    bool b_is_valid = false;

    if (NULL != p_config)
    {
        bool const b_left_valid = calibration_is_valid(
                                      &p_config->left_calibration);
        bool const b_center_valid = calibration_is_valid(
                                        &p_config->center_calibration);
        bool const b_barcode_valid = calibration_is_valid(
                                         &p_config->barcode_calibration);
        bool const b_line_limits_valid =
            ((p_config->line_hysteresis < p_config->line_threshold)
             && ((p_config->line_threshold + p_config->line_hysteresis)
                 <= BUDDY3_DARKNESS_SCALE));
        bool const b_barcode_limits_valid =
            ((p_config->barcode_hysteresis < p_config->barcode_threshold)
             && ((p_config->barcode_threshold + p_config->barcode_hysteresis)
                 <= BUDDY3_DARKNESS_SCALE));

        b_is_valid =
            (b_left_valid
             && b_center_valid
             && b_barcode_valid
             && b_line_limits_valid
             && b_barcode_limits_valid
             && ((p_config->line_threshold + p_config->line_hysteresis)
                 <= p_config->junction_threshold)
             && (p_config->junction_threshold <= BUDDY3_DARKNESS_SCALE)
             && (0U < p_config->left_weight_x100)
             && (0U < p_config->line_sample_divider)
             && (0U < p_config->junction_confirm_samples)
             && (-((int16_t) BUDDY3_DARKNESS_SCALE)
                 <= p_config->line_search_error)
             && (0 != p_config->line_search_error)
             && (p_config->line_search_error
                 <= (int16_t) BUDDY3_DARKNESS_SCALE)
             && (0U < p_config->line_lost_time_ms)
             && (0U < p_config->barcode_quiet_time_ms)
             && (0U < p_config->barcode_timeout_ms)
             && (0U < p_config->barcode_min_element_ms)
             && (p_config->barcode_min_element_ms
                 < p_config->barcode_max_element_ms)
              && (p_config->barcode_max_element_ms
                  <= BUDDY3_BARCODE_MAX_ELEMENT_MS)
             && (p_config->barcode_quiet_time_ms
                 <= p_config->barcode_max_element_ms)
             && (p_config->barcode_max_element_ms
                 < p_config->barcode_timeout_ms)
             && (PERCENT_SCALE < p_config->barcode_min_wide_ratio_x100));
    }

    return (b_is_valid);
}

uint16_t buddy3_config_normalize_adc (
             uint16_t raw_adc,
             buddy3_ir_calibration_t const * const p_calibration)
{
    int32_t darkness = 0;

    if (calibration_is_valid(p_calibration))
    {
        int32_t numerator = 0;
        int32_t denominator = 1;

        if (p_calibration->white_adc < p_calibration->black_adc)
        {
            numerator = ((int32_t) raw_adc
                         - (int32_t) p_calibration->white_adc)
                        * (int32_t) BUDDY3_DARKNESS_SCALE;
            denominator = (int32_t) p_calibration->black_adc
                          - (int32_t) p_calibration->white_adc;
        }
        else
        {
            numerator = ((int32_t) p_calibration->white_adc
                         - (int32_t) raw_adc)
                        * (int32_t) BUDDY3_DARKNESS_SCALE;
            denominator = (int32_t) p_calibration->white_adc
                          - (int32_t) p_calibration->black_adc;
        }

        darkness = numerator / denominator;

        if (0 > darkness)
        {
            darkness = 0;
        }
        else if ((int32_t) BUDDY3_DARKNESS_SCALE < darkness)
        {
            darkness = (int32_t) BUDDY3_DARKNESS_SCALE;
        }
        else
        {
            /* The normalized value is already in range. */
        }
    }

    return ((uint16_t) darkness);
}

static bool calibration_is_valid (
                buddy3_ir_calibration_t const * const p_calibration)
{
    bool b_is_valid = false;

    if (NULL != p_calibration)
    {
        b_is_valid =
            ((p_calibration->black_adc <= BUDDY3_ADC_MAX_COUNT)
             && (p_calibration->white_adc <= BUDDY3_ADC_MAX_COUNT)
             && (p_calibration->black_adc != p_calibration->white_adc));
    }

    return (b_is_valid);
}

/*** end of file ***/
