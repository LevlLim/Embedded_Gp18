/**
 * @file adc_sampler.c
 * @brief Sole ADC owner: bounded GP28/GP26/GP27 periodic conversions.
 *
 * Tribunal 1 explicitly keeps free-running timestamps as native 32-bit
 * values. Samples therefore use 12 value bits in a uint16_t and a full
 * uint32_t timestamp. The 32-entry ring is 256 bytes, not the former 1 KiB.
 *
 * One 32-bit publication sequence guards the ISR's multi-field records:
 * odd while writing, even when stable. Readers copy only a stable version.
 * All shared field accesses are atomic loads/stores. No lock, heap, DMA,
 * cross-core interrupt masking, FIFO or unbounded hardware wait is used.
 */
#include "adc_sampler.h"
#include "../drivers/rp2040_regs.h"

#define ADC_VALUE_MASK       (0x0FFFU)
#define ADC_LINE_TICKS       (PERIOD_LINE_MS / ADC_POLL_PERIOD_MS)
#define ADC_RESET_TIMEOUT    (100000U)
#define ADC_SNAPSHOT_TRIES   (2U)

_Static_assert((S3_ADC_RING_LEN > 0U)
    && (0U == (S3_ADC_RING_LEN & (S3_ADC_RING_LEN - 1U))),
    "ADC ring length must be a power of two");

static adc_sample_t g_latest[ADC_CH_COUNT];
static adc_sample_t g_ring[S3_ADC_RING_LEN];
static uint32_t g_publication = 0U;
static uint32_t g_ring_written = 0U;
static uint32_t g_last_poll_us = 0U;
static uint32_t g_failures = 0U;
static uint8_t g_valid_channels = 0U;
static uint8_t g_line_tick = 0U;
static bool g_b_initialised = false;
static uint16_t g_max_poll_us = 0U;

static void adc_sampler_poll(void * p_exinf);

static void store_sample(adc_sample_t * p_target, uint16_t value,
                         uint32_t timestamp_us)
{
    __atomic_store_n(&p_target->value, value, __ATOMIC_RELAXED);
    __atomic_store_n(&p_target->timestamp_us, timestamp_us, __ATOMIC_RELAXED);
}

static void load_sample(adc_sample_t const * p_source, adc_sample_t * p_out)
{
    p_out->value = __atomic_load_n(&p_source->value, __ATOMIC_RELAXED);
    p_out->timestamp_us = __atomic_load_n(&p_source->timestamp_us,
                                         __ATOMIC_RELAXED);
}

uint32_t adc_sampler_now_us(void)
{
    return (RP2040_NOW_US());
}

static bool wait_ready(uint32_t address, uint32_t mask, uint32_t limit)
{
    bool b_ready = false;
    uint32_t poll = 0U;

    for (poll = 0U; poll < limit; poll++)
    {
        if (0U != (in_w(address) & mask))
        {
            b_ready = true;
            break;
        }
    }
    return (b_ready);
}

static void sample_channel(uint8_t channel, uint8_t * p_valid)
{
    uint16_t value = 0U;
    uint32_t stamp = 0U;
    uint32_t index = g_ring_written;
    uint32_t control = ADC_CS_EN
                       | ((uint32_t)channel << ADC_CS_AINSEL_SHIFT);

    out_w(ADC_CS, control);
    out_w(ADC_CS, control | ADC_CS_START_ONCE);
    if (wait_ready(ADC_CS, ADC_CS_READY, S3_ADC_TIMEOUT_POLLS)
        && (0U == (in_w(ADC_CS) & ADC_CS_ERR)))
    {
        value = (uint16_t)(in_w(ADC_RESULT) & ADC_VALUE_MASK);
        stamp = adc_sampler_now_us();
        store_sample(&g_latest[channel], value, stamp);
        *p_valid |= (uint8_t)(1U << channel);
        if (ADC_CH_BARCODE == channel)
        {
            store_sample(&g_ring[index & (S3_ADC_RING_LEN - 1U)],
                         value, stamp);
            /* Keep a full ring full across counter wrap. S3_ADC_RING_LEN
             * has the same masked head as zero without looking empty. */
            __atomic_store_n(&g_ring_written,
                (UINT32_MAX == index) ? S3_ADC_RING_LEN : index + 1U,
                __ATOMIC_RELAXED);
        }
    }
    else
    {
        *p_valid &= (uint8_t)~(1U << channel);
        __atomic_store_n(&g_failures, g_failures + 1U, __ATOMIC_RELAXED);
    }
}

/* Kernel cyclic callback: the kernel supplies the interrupt wrapper. */
static void adc_sampler_poll(void * p_exinf)
{
    uint32_t now_us = adc_sampler_now_us();
    uint32_t sequence = g_publication;
    uint8_t valid = __atomic_load_n(&g_valid_channels, __ATOMIC_RELAXED);
    uint32_t elapsed_us = 0U;

    (void)p_exinf;
    if (g_b_initialised)
    {
        __atomic_store_n(&g_publication, sequence + 1U, __ATOMIC_SEQ_CST);
        if ((now_us - g_last_poll_us) > S3_SAMPLE_STALE_US)
        {
            valid = 0U;
            __atomic_store_n(&g_ring_written, 0U, __ATOMIC_RELAXED);
        }
        sample_channel(ADC_CH_BARCODE, &valid);
        if (0U == g_line_tick)
        {
            sample_channel(ADC_CH_LINE_L, &valid);
            sample_channel(ADC_CH_LINE_R, &valid);
        }
        g_line_tick++;
        if (g_line_tick >= ADC_LINE_TICKS)
        {
            g_line_tick = 0U;
        }
        __atomic_store_n(&g_valid_channels, valid, __ATOMIC_RELAXED);
        __atomic_store_n(&g_last_poll_us, now_us, __ATOMIC_RELAXED);
        __atomic_store_n(&g_publication, sequence + 2U, __ATOMIC_RELEASE);
        elapsed_us = adc_sampler_now_us() - now_us;
        if (elapsed_us > g_max_poll_us)
        {
            __atomic_store_n(&g_max_poll_us,
                (uint16_t)((elapsed_us > 65535U) ? 65535U : elapsed_us),
                __ATOMIC_RELAXED);
        }
    }
}

ER adc_sampler_init(void)
{
    T_CCYC cycle = {0};
    ID h_cycle = (ID)0;
    uint32_t channel = 0U;
    uint32_t pin = 0U;
    ER error = E_OK;

    if (!g_b_initialised)
    {
        clr_w(RESETS_RESET, RESETS_RESET_ADC);
        if (!wait_ready(RESETS_RESET_DONE, RESETS_RESET_ADC,
                        ADC_RESET_TIMEOUT))
        {
            error = E_IO;
        }
        if (E_OK == error)
        {
            for (channel = 0U; channel < ADC_CH_COUNT; channel++)
            {
                pin = PIN_IR_LINE_L + channel;
                clr_w(PADS_GPIO(pin), PADS_IE | PADS_PUE | PADS_PDE);
                set_w(PADS_GPIO(pin), PADS_OD);
                store_sample(&g_latest[channel], 0U, 0U);
            }
            out_w(ADC_FCS, 0U);
            out_w(ADC_DIV, 0U);
            out_w(ADC_CS, ADC_CS_EN);
            if (!wait_ready(ADC_CS, ADC_CS_READY, S3_ADC_TIMEOUT_POLLS))
            {
                error = E_IO;
            }
        }
        if (E_OK == error)
        {
            g_valid_channels = 0U;
            g_ring_written = 0U;
            g_failures = 0U;
            g_line_tick = 0U;
            g_last_poll_us = adc_sampler_now_us();
            cycle.cycatr = TA_HLNG | TA_STA;
            cycle.cychdr = (FP)adc_sampler_poll;
            cycle.cyctim = (RELTIM)ADC_POLL_PERIOD_MS;
            cycle.cycphs = (RELTIM)ADC_POLL_PERIOD_MS;
            h_cycle = tk_cre_cyc(&cycle);
            error = (h_cycle > (ID)0) ? E_OK : E_OBJ;
            /* A failed cycle creation remains retryable. */
            g_b_initialised = (E_OK == error);
        }
    }
    return (error);
}

static bool snapshot_complete(uint32_t sequence)
{
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return (sequence
        == __atomic_load_n(&g_publication, __ATOMIC_RELAXED));
}

bool adc_sampler_read(uint8_t channel, adc_sample_t * p_out)
{
    adc_sample_t sample = {0U, 0U};
    uint32_t sequence = 0U;
    uint32_t poll_us = 0U;
    uint32_t now_us = 0U;
    uint8_t flags = 0U;
    uint8_t attempt = 0U;
    bool b_valid = false;

    if ((channel < ADC_CH_COUNT) && (NULL != p_out) && g_b_initialised)
    {
        for (attempt = 0U; attempt < ADC_SNAPSHOT_TRIES; attempt++)
        {
            sequence = __atomic_load_n(&g_publication, __ATOMIC_ACQUIRE);
            if (0U != (sequence & 1U))
            {
                continue;
            }
            load_sample(&g_latest[channel], &sample);
            poll_us = __atomic_load_n(&g_last_poll_us, __ATOMIC_RELAXED);
            flags = __atomic_load_n(&g_valid_channels, __ATOMIC_RELAXED);
            if (snapshot_complete(sequence))
            {
                now_us = adc_sampler_now_us();
                b_valid = (0U != (flags & (uint8_t)(1U << channel)))
                    && ((now_us - poll_us) <= S3_SAMPLE_STALE_US)
                    && ((now_us - sample.timestamp_us)
                        <= S3_SAMPLE_STALE_US);
                break;
            }
        }
    }
    if (b_valid)
    {
        *p_out = sample;
    }
    return (b_valid);
}

uint8_t adc_sampler_since(uint32_t after_us, adc_sample_t * p_out,
                         uint8_t capacity)
{
    uint32_t sequence = __atomic_load_n(&g_publication, __ATOMIC_ACQUIRE);
    uint32_t newest = 0U;
    uint32_t first = 0U;
    uint32_t scan = 0U;
    uint32_t available = 0U;
    uint32_t difference = 0U;
    uint32_t poll_us = 0U;
    adc_sample_t sample = {0U, 0U};
    uint8_t count = 0U;

    if ((NULL != p_out) && (0U != capacity) && g_b_initialised
        && (0U == (sequence & 1U)))
    {
        newest = __atomic_load_n(&g_ring_written, __ATOMIC_RELAXED);
        poll_us = __atomic_load_n(&g_last_poll_us, __ATOMIC_RELAXED);
        available = (newest < S3_ADC_RING_LEN) ? newest : S3_ADC_RING_LEN;
        first = newest;
        /* At most 32 comparisons; normally only one or two. */
        for (scan = 0U; scan < available; scan++)
        {
            load_sample(&g_ring[(first - 1U) & (S3_ADC_RING_LEN - 1U)],
                        &sample);
            difference = sample.timestamp_us - after_us;
            if ((0U == difference) || (difference > 0x7FFFFFFFU))
            {
                break;
            }
            first--;
        }
        for (scan = first; (scan != newest) && (count < capacity); scan++)
        {
            load_sample(&g_ring[scan & (S3_ADC_RING_LEN - 1U)],
                        &p_out[count]);
            count++;
        }
        if (!snapshot_complete(sequence)
            || ((adc_sampler_now_us() - poll_us) > S3_SAMPLE_STALE_US))
        {
            count = 0U;
        }
    }
    return (count);
}

uint32_t adc_sampler_overruns(void)
{
    return (__atomic_load_n(&g_failures, __ATOMIC_RELAXED));
}

uint16_t adc_sampler_max_us(void)
{
    return (__atomic_load_n(&g_max_poll_us, __ATOMIC_RELAXED));
}

void adc_sampler_selftest(void)
{
    adc_sample_t sample = {0U, 0U};
    uint8_t channel = 0U;

    APP_LOG((UB *)"[ADC] conversion failures=%d, max handler=%d us\n",
              (INT)adc_sampler_overruns(), (INT)adc_sampler_max_us());
    for (channel = 0U; channel < ADC_CH_COUNT; channel++)
    {
        if (adc_sampler_read(channel, &sample))
        {
            APP_LOG((UB *)"[ADC] ch%d=%d, age=%d us\n", (INT)channel,
                      (INT)sample.value,
                      (INT)(adc_sampler_now_us() - sample.timestamp_us));
        }
        else
        {
            APP_LOG((UB *)"[ADC] ch%d unavailable/stale\n", (INT)channel);
        }
    }
}
