/**
 * @file buddy3_config.h
 *
 * @brief Configuration and IR calibration data for the Buddy 3 subsystem.
 */

#ifndef BUDDY3_CONFIG_H
#define BUDDY3_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define BUDDY3_ADC_MAX_COUNT                 (4095U)
#define BUDDY3_DARKNESS_SCALE               (1000U)
#define BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH   (8U)
#define BUDDY3_BARCODE_MAX_ELEMENT_MS        (511U)

typedef struct
{
    uint16_t black_adc;
    uint16_t white_adc;
} buddy3_ir_calibration_t;

typedef struct
{
    buddy3_ir_calibration_t left_calibration;
    buddy3_ir_calibration_t center_calibration;
    buddy3_ir_calibration_t barcode_calibration;
    uint16_t                line_threshold;
    uint16_t                line_hysteresis;
    uint16_t                junction_threshold;
    uint16_t                left_weight_x100;
    uint16_t                barcode_threshold;
    uint16_t                barcode_hysteresis;
    uint16_t                barcode_min_element_ms;
    uint16_t                barcode_max_element_ms;
    uint16_t                barcode_min_wide_ratio_x100;
    uint16_t                line_sample_divider;
    uint16_t                junction_confirm_samples;
    int16_t                 line_search_error;
    uint32_t                line_lost_time_ms;
    uint32_t                barcode_quiet_time_ms;
    uint32_t                barcode_timeout_ms;
} buddy3_config_t;

void     buddy3_config_get_default (buddy3_config_t * const p_config);
bool     buddy3_config_is_valid (buddy3_config_t const * const p_config);
uint16_t buddy3_config_normalize_adc (
             uint16_t raw_adc,
             buddy3_ir_calibration_t const * const p_calibration);

#endif /* BUDDY3_CONFIG_H */

/*** end of file ***/
