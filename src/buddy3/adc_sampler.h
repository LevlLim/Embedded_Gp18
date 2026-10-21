/**
 * @file adc_sampler.h
 *
 * @brief Sole hardware owner for the three Buddy 3 ADC channels.
 */

#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#include <stdbool.h>
#include <stdint.h>

#define ADC_SAMPLER_SNAPSHOT_SIZE_BYTES   (5U)

typedef struct
{
    uint8_t bytes[ADC_SAMPLER_SNAPSHOT_SIZE_BYTES];
} adc_sampler_snapshot_t;

void adc_sampler_init (void);
bool adc_sampler_read_barcode (uint16_t * const p_barcode_raw);
bool adc_sampler_read_line (
         uint16_t * const p_left_raw,
         uint16_t * const p_center_raw);
bool adc_sampler_get_snapshot (adc_sampler_snapshot_t * const p_snapshot);
uint16_t adc_sampler_snapshot_get_left (
             adc_sampler_snapshot_t const * const p_snapshot);
uint16_t adc_sampler_snapshot_get_center (
             adc_sampler_snapshot_t const * const p_snapshot);
uint16_t adc_sampler_snapshot_get_barcode (
             adc_sampler_snapshot_t const * const p_snapshot);

#endif /* ADC_SAMPLER_H */

/*** end of file ***/
