/**
 * @file adc_sampler.c
 *
 * @brief Sole hardware owner for the three Buddy 3 ADC channels.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/adc.h"

#include "adc_sampler.h"

#define LEFT_SENSOR_GPIO       (26U)
#define CENTER_SENSOR_GPIO     (27U)
#define BARCODE_SENSOR_GPIO    (28U)
#define LEFT_ADC_CHANNEL       (0U)
#define CENTER_ADC_CHANNEL     (1U)
#define BARCODE_ADC_CHANNEL    (2U)
#define ADC_HIGH_NIBBLE_MASK   (0x0FU)
#define ADC_NIBBLE_SHIFT       (4U)
#define ADC_BYTE_SHIFT         (8U)

_Static_assert(sizeof(adc_sampler_snapshot_t)
               == ADC_SAMPLER_SNAPSHOT_SIZE_BYTES,
               "ADC snapshot size changed");

static uint16_t g_left_raw = 0U;
static uint16_t g_center_raw = 0U;
static uint16_t g_barcode_raw = 0U;
static bool     g_b_adc_initialized = false;

void adc_sampler_init (void)
{
    adc_init();
    adc_gpio_init(LEFT_SENSOR_GPIO);
    adc_gpio_init(CENTER_SENSOR_GPIO);
    adc_gpio_init(BARCODE_SENSOR_GPIO);

    g_left_raw             = 0U;
    g_center_raw           = 0U;
    g_barcode_raw          = 0U;
    g_b_adc_initialized    = true;
}

bool adc_sampler_read_barcode (uint16_t * const p_barcode_raw)
{
    bool b_sample_read = false;

    if ((NULL != p_barcode_raw) && g_b_adc_initialized)
    {
        adc_select_input(BARCODE_ADC_CHANNEL);
        g_barcode_raw = adc_read();
        *p_barcode_raw = g_barcode_raw;
        b_sample_read = true;
    }

    return (b_sample_read);
}

bool adc_sampler_read_line (
         uint16_t * const p_left_raw,
         uint16_t * const p_center_raw)
{
    bool b_samples_read = false;

    if ((NULL != p_left_raw)
        && (NULL != p_center_raw)
        && g_b_adc_initialized)
    {
        adc_select_input(LEFT_ADC_CHANNEL);
        g_left_raw = adc_read();

        adc_select_input(CENTER_ADC_CHANNEL);
        g_center_raw = adc_read();

        *p_left_raw = g_left_raw;
        *p_center_raw = g_center_raw;
        b_samples_read = true;
    }

    return (b_samples_read);
}

bool adc_sampler_get_snapshot (adc_sampler_snapshot_t * const p_snapshot)
{
    bool b_snapshot_available = false;

    if ((NULL != p_snapshot) && g_b_adc_initialized)
    {
        p_snapshot->bytes[0U] = (uint8_t) g_left_raw;
        p_snapshot->bytes[1U] =
            (uint8_t) (((g_left_raw >> ADC_BYTE_SHIFT)
                        & ADC_HIGH_NIBBLE_MASK)
                       | ((g_center_raw & ADC_HIGH_NIBBLE_MASK)
                          << ADC_NIBBLE_SHIFT));
        p_snapshot->bytes[2U] =
            (uint8_t) (g_center_raw >> ADC_NIBBLE_SHIFT);
        p_snapshot->bytes[3U] = (uint8_t) g_barcode_raw;
        p_snapshot->bytes[4U] =
            (uint8_t) ((g_barcode_raw >> ADC_BYTE_SHIFT)
                       & ADC_HIGH_NIBBLE_MASK);
        b_snapshot_available = true;
    }

    return (b_snapshot_available);
}

uint16_t adc_sampler_snapshot_get_left (
             adc_sampler_snapshot_t const * const p_snapshot)
{
    uint16_t value = 0U;

    if (NULL != p_snapshot)
    {
        value = (uint16_t) p_snapshot->bytes[0U]
                | (uint16_t) (((uint16_t) p_snapshot->bytes[1U]
                               & ADC_HIGH_NIBBLE_MASK)
                              << ADC_BYTE_SHIFT);
    }

    return (value);
}

uint16_t adc_sampler_snapshot_get_center (
             adc_sampler_snapshot_t const * const p_snapshot)
{
    uint16_t value = 0U;

    if (NULL != p_snapshot)
    {
        value = (uint16_t) (((uint16_t) p_snapshot->bytes[1U]
                            >> ADC_NIBBLE_SHIFT)
                           & ADC_HIGH_NIBBLE_MASK)
                | (uint16_t) ((uint16_t) p_snapshot->bytes[2U]
                              << ADC_NIBBLE_SHIFT);
    }

    return (value);
}

uint16_t adc_sampler_snapshot_get_barcode (
             adc_sampler_snapshot_t const * const p_snapshot)
{
    uint16_t value = 0U;

    if (NULL != p_snapshot)
    {
        value = (uint16_t) p_snapshot->bytes[3U]
                | (uint16_t) (((uint16_t) p_snapshot->bytes[4U]
                               & ADC_HIGH_NIBBLE_MASK)
                              << ADC_BYTE_SHIFT);
    }

    return (value);
}

/*** end of file ***/
