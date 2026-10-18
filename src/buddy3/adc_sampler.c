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

static adc_sampler_snapshot_t g_adc_snapshot = { 0U, 0U, 0U };
static bool                   g_b_adc_initialized = false;

void adc_sampler_init (void)
{
    adc_init();
    adc_gpio_init(LEFT_SENSOR_GPIO);
    adc_gpio_init(CENTER_SENSOR_GPIO);
    adc_gpio_init(BARCODE_SENSOR_GPIO);

    g_adc_snapshot.left_raw    = 0U;
    g_adc_snapshot.center_raw  = 0U;
    g_adc_snapshot.barcode_raw = 0U;
    g_b_adc_initialized        = true;
}

bool adc_sampler_read_barcode (uint16_t * const p_barcode_raw)
{
    bool b_sample_read = false;

    if ((NULL != p_barcode_raw) && g_b_adc_initialized)
    {
        adc_select_input(BARCODE_ADC_CHANNEL);
        g_adc_snapshot.barcode_raw = adc_read();
        *p_barcode_raw = g_adc_snapshot.barcode_raw;
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
        g_adc_snapshot.left_raw = adc_read();

        adc_select_input(CENTER_ADC_CHANNEL);
        g_adc_snapshot.center_raw = adc_read();

        *p_left_raw = g_adc_snapshot.left_raw;
        *p_center_raw = g_adc_snapshot.center_raw;
        b_samples_read = true;
    }

    return (b_samples_read);
}

bool adc_sampler_get_snapshot (adc_sampler_snapshot_t * const p_snapshot)
{
    bool b_snapshot_available = false;

    if ((NULL != p_snapshot) && g_b_adc_initialized)
    {
        *p_snapshot = g_adc_snapshot;
        b_snapshot_available = true;
    }

    return (b_snapshot_available);
}

/*** end of file ***/
