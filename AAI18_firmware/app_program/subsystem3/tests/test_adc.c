/**
 * @file test_adc.c
 * @brief Production ADC driver with register-level hardware mocks.
 */
#include <stdio.h>
#include "../adc_sampler.h"
#include "../../drivers/rp2040_regs.h"

static uint32_t mock_read(uint32_t address);
static void mock_write(uint32_t address, uint32_t value);
static void mock_ignore(uint32_t address, uint32_t value);

#define in_w mock_read
#define out_w mock_write
#define set_w mock_ignore
#define clr_w mock_ignore
#include "../adc_sampler.c"

static uint32_t g_clock_us = 1000000U;
static uint32_t g_control_register = 0U;
static uint32_t g_conversions[3] = {0U};
static uint8_t g_bad_channels = 0U;
static ID g_cycle_result = 1;
static unsigned int g_tests = 0U;
static unsigned int g_failed = 0U;

static void check(bool b_valid)
{
    g_tests++;
    if (!b_valid)
    {
        g_failed++;
        printf("ADC failure at check %u\n", g_tests);
    }
}

static uint32_t mock_read(uint32_t address)
{
    uint8_t channel = (uint8_t)
        ((g_control_register & ADC_CS_AINSEL_MASK) >> ADC_CS_AINSEL_SHIFT);
    uint32_t value = 0U;

    if (TIMER_TIMERAWL == address)
    {
        value = g_clock_us;
    }
    else if (RESETS_RESET_DONE == address)
    {
        value = RESETS_RESET_ADC;
    }
    else if (ADC_CS == address)
    {
        value = g_control_register;
        if (0U == (g_bad_channels & (uint8_t)(1U << channel)))
        {
            value |= ADC_CS_READY;
        }
    }
    else if (ADC_RESULT == address)
    {
        value = 100U + channel;
    }
    else
    {
        /* Unused register. */
    }
    return (value);
}

static void mock_write(uint32_t address, uint32_t value)
{
    uint8_t channel = 0U;

    if (ADC_CS == address)
    {
        g_control_register = value;
        channel = (uint8_t)
            ((value & ADC_CS_AINSEL_MASK) >> ADC_CS_AINSEL_SHIFT);
        if (0U != (value & ADC_CS_START_ONCE))
        {
            g_conversions[channel]++;
        }
    }
}

static void mock_ignore(uint32_t address, uint32_t value)
{
    (void)address;
    (void)value;
}

ID tk_cre_cyc(T_CCYC const * p_cycle)
{
    check(1U == p_cycle->cyctim);
    check(NULL != p_cycle->cychdr);
    return (g_cycle_result);
}

void tm_printf(UB * p_format, ...)
{
    (void)p_format;
}

INT tm_sprintf(UB * buffer, UB const * format, ...)
{
    (void)format;
    buffer[0] = (UB)0;
    return (0);
}
void app_log_line(UB const * text) { (void)text; }

int main(void)
{
    adc_sample_t sample = {0U, 0U};
    adc_sample_t batch[8] = {{0U, 0U}};
    uint32_t tick = 0U;
    uint32_t cursor = 0U;
    uint32_t failures = 0U;
    uint8_t count = 0U;
    uint8_t index = 0U;

    g_cycle_result = E_OBJ;
    check(E_OBJ == adc_sampler_init());
    check(!g_b_initialised);
    g_cycle_result = 1;
    check(E_OK == adc_sampler_init());
    check(E_OK == adc_sampler_init());
    check(!adc_sampler_read(ADC_CH_LINE_L, &sample));
    for (tick = 0U; tick < 20U; tick++)
    {
        g_clock_us += 1000U;
        adc_sampler_poll(NULL);
    }
    check(20U == g_conversions[ADC_CH_BARCODE]);
    check(4U == g_conversions[ADC_CH_LINE_L]);
    check(4U == g_conversions[ADC_CH_LINE_C]);
    check(adc_sampler_read(ADC_CH_LINE_C, &sample));
    check(101U == sample.value);
    check((g_clock_us - sample.timestamp_us) <= 5000U);
    cursor = g_clock_us - 10000U;
    count = adc_sampler_since(cursor, batch, 8U);
    check(8U == count);
    for (index = 0U; index < count; index++)
    {
        check(102U == batch[index].value);
        check(batch[index].timestamp_us == (cursor + (index + 1U) * 1000U));
    }
    cursor = batch[count - 1U].timestamp_us;
    count = adc_sampler_since(cursor, batch, 8U);
    check(2U == count);
    cursor = batch[count - 1U].timestamp_us;
    check(0U == adc_sampler_since(cursor, batch, 8U));
    check(!adc_sampler_read(3U, &sample));
    check(!adc_sampler_read(0U, NULL));

    g_clock_us += 1048576U; /* Old 20-bit stamps would alias here. */
    check(!adc_sampler_read(0U, &sample));
    check(0U == adc_sampler_since(cursor, batch, 8U));
    adc_sampler_poll(NULL);
    check(adc_sampler_read(0U, &sample));
    check(g_clock_us == sample.timestamp_us);
    check(1U == adc_sampler_since(cursor, batch, 8U));

    g_bad_channels = (uint8_t)(1U << ADC_CH_BARCODE);
    g_clock_us += 1000U;
    failures = adc_sampler_overruns();
    adc_sampler_poll(NULL);
    check(!adc_sampler_read(ADC_CH_BARCODE, &sample));
    check(adc_sampler_overruns() == failures + 1U);
    g_bad_channels = 0U;
    g_clock_us += 1000U;
    adc_sampler_poll(NULL);
    check(adc_sampler_read(ADC_CH_BARCODE, &sample));
    g_publication |= 1U;
    check(!adc_sampler_read(ADC_CH_BARCODE, &sample));
    check(0U == adc_sampler_since(cursor, batch, 8U));
    g_publication++;

    g_clock_us = 0xFFFFF000U;
    adc_sampler_poll(NULL);
    g_clock_us += 5000U;
    check(adc_sampler_read(ADC_CH_BARCODE, &sample));
    check(5000U == (g_clock_us - sample.timestamp_us));
    g_clock_us += S3_SAMPLE_STALE_US;
    check(!adc_sampler_read(ADC_CH_BARCODE, &sample));

    /* The internal write counter must not turn a full ring into an empty
     * startup ring when it wraps after long operation. */
    adc_sampler_poll(NULL);
    cursor = g_clock_us;
    g_ring_written = UINT32_MAX - 3U;
    for (tick = 0U; tick < 4U; tick++)
    {
        g_clock_us += 1000U;
        adc_sampler_poll(NULL);
    }
    count = adc_sampler_since(cursor, batch, 8U);
    check(4U == count);
    for (index = 0U; index < count; index++)
    {
        check(batch[index].timestamp_us == cursor + (index + 1U) * 1000U);
    }
    printf("ADC regressions: %u/%u passed\n", g_tests - g_failed, g_tests);
    return ((0U == g_failed) ? 0 : 1);
}
