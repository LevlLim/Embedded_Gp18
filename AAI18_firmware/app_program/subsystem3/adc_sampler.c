/*
 * adc_sampler.c
 *
 * Group AAI18 -- RP2040 ADC round-robin sampler.
 *
 * Written against the registers directly because the template's BSP has no
 * ADC support and the Pico SDK's hardware/adc.h is not on the application
 * include path.  See drivers/rp2040_regs.h for why.
 *
 * APIs used: out_w, in_w, set_w, clr_w (template register helpers)
 */

#include "adc_sampler.h"
#include "../drivers/rp2040_regs.h"

/* ---------------------------------------------------------------------
 * Packed sample word: value in bits 11:0, timestamp bits 19:0 in 31:12.
 * --------------------------------------------------------------------- */
#define SAMPLE_VALUE_MASK   (0x0FFFu)
#define SAMPLE_TIME_SHIFT   (12u)
#define SAMPLE_TIME_MASK    (0x000FFFFFu)

/* ADC clock is 48 MHz; DIV is 8.8 fixed point, rate = 48 MHz / (1 + DIV). */
#define ADC_CLOCK_HZ        (48000000u)
#define ADC_DIV_INT         ((ADC_CLOCK_HZ / ADC_TOTAL_RATE_SPS) - 1u)

/* ---------------------------------------------------------------------
 * Private data
 * --------------------------------------------------------------------- */

/* Latest sample per channel, packed.  Single writer (the 1 ms poll);
   a single aligned word, so a reader on any task gets one whole sample. */
static volatile uint32_t latest[ADC_CH_COUNT];
static volatile bool     b_have_sample[ADC_CH_COUNT];

/* Barcode history ring.  Written by the 1 ms poll (cyclic handler), read
   by the barcode task.  Each entry is one word and the index is advanced
   after the entry is written, so the reader never sees a half-written
   entry; at 512 ms of history it cannot be overwritten while read. */
static volatile uint32_t ring[ADC_RING_LEN];
static volatile uint32_t ring_write_index = 0u;

static volatile uint32_t overrun_count = 0u;
static volatile uint32_t sample_count  = 0u;   /* diagnostics only */
static bool     b_initialised = false;
static uint8_t  next_channel  = 0u;

/* ---------------------------------------------------------------------
 * Private function prototypes
 * --------------------------------------------------------------------- */

static void adc_sampler_poll(void *p_exinf);

/* ---------------------------------------------------------------------
 * Private functions
 * --------------------------------------------------------------------- */

/*
 * Put one GPIO into analogue mode: disable the digital input buffer and
 * the pulls, and the output driver.
 */
static void adc_pin_to_analogue(uint32_t gpio)
{
    clr_w(PADS_GPIO(gpio), PADS_IE | PADS_PUE | PADS_PDE);
    set_w(PADS_GPIO(gpio), PADS_OD);
}

static uint32_t pack_sample(uint32_t value, uint32_t time_us)
{
    return ((value & SAMPLE_VALUE_MASK)
            | ((time_us & SAMPLE_TIME_MASK) << SAMPLE_TIME_SHIFT));
}

/*
 * Rebuild a full timestamp from a packed sample, relative to 'now'.  The
 * modular difference is the sample's age, correct for any sample younger
 * than 1.05 s.
 */
static void unpack_sample(uint32_t word, uint32_t now_us,
                          adc_sample_t *p_out)
{
    uint32_t age_us;

    age_us = (now_us - (word >> SAMPLE_TIME_SHIFT)) & SAMPLE_TIME_MASK;

    p_out->value        = (uint16_t)(word & SAMPLE_VALUE_MASK);
    p_out->timestamp_us = now_us - age_us;
}

/*
 * Store one FIFO entry.  The FIFO entry carries no channel tag, so the
 * channel is tracked by the rotating counter in adc_sampler_poll.
 */
static void store_sample(uint8_t channel, uint32_t value, uint32_t time_us)
{
    uint32_t word;

    if (channel < ADC_CH_COUNT)
    {
        word                   = pack_sample(value, time_us);
        latest[channel]        = word;
        b_have_sample[channel] = true;
        sample_count           = sample_count + 1u;

        if (ADC_CH_BARCODE == channel)
        {
            ring[ring_write_index % ADC_RING_LEN] = word;
            ring_write_index = ring_write_index + 1u;
        }
    }
}

/*
 * Throw the FIFO away after an overrun and resynchronise the rotation.
 */
static void recover_from_overrun(void)
{
    overrun_count = overrun_count + 1u;

    /* OVER and UNDER are write-1-to-clear.  Clear them with a PLAIN write
       that also restores the FIFO configuration (EN only).  The atomic
       set alias (set_w) is not documented to clear W1C bits: if it does
       not, OVER stays set, every poll lands here, and no sample is ever
       stored -- calibration then reports "4095..0" on every channel. */
    out_w(ADC_FCS, ADC_FCS_EN | ADC_FCS_OVER | ADC_FCS_UNDER);

    while (0u == (in_w(ADC_FCS) & ADC_FCS_EMPTY))
    {
        (void)in_w(ADC_FIFO);
    }

    /* The next conversion the hardware completes is the one AINSEL points
       at now. */
    next_channel = (uint8_t)((in_w(ADC_CS) & ADC_CS_AINSEL_MASK)
                             >> ADC_CS_AINSEL_SHIFT);
}

/* ---------------------------------------------------------------------
 * Public functions
 * --------------------------------------------------------------------- */

EXPORT ER adc_sampler_init(void)
{
    T_CCYC   ccyc = {0};
    ID       cycid;
    uint32_t chan;

    if (b_initialised)
    {
        return (E_OK);
    }

    for (chan = 0u; chan < ADC_CH_COUNT; chan++)
    {
        latest[chan]        = 0u;
        b_have_sample[chan] = false;
    }

    /* Release the ADC from reset and wait for it to come up. */
    clr_w(RESETS_RESET, RESETS_RESET_ADC);
    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_ADC))
    {
        /* Busy wait: this runs once, at init, before any task is started. */
    }

    adc_pin_to_analogue(PIN_IR_LINE_L);
    adc_pin_to_analogue(PIN_IR_LINE_C);
    adc_pin_to_analogue(PIN_IR_BARCODE);

    /* Enable the ADC, no temperature sensor. */
    out_w(ADC_CS, ADC_CS_EN);
    while (0u == (in_w(ADC_CS) & ADC_CS_READY))
    {
        /* Wait for the first calibration to settle. */
    }

    /* 1.5 kS/s in total -- see the header for why it must be this slow
       while the FIFO is drained by polling. */
    out_w(ADC_DIV, (ADC_DIV_INT << 8u));

    /* FIFO: enabled, no DMA request, full 12-bit results (no shift). */
    out_w(ADC_FCS, ADC_FCS_EN);

    ring_write_index = 0u;
    overrun_count    = 0u;
    next_channel     = 0u;

    /* Round-robin across ADC0..ADC2 starting at ADC0, free-running. */
    out_w(ADC_CS, ADC_CS_EN
                | (0u << ADC_CS_AINSEL_SHIFT)
                | (0x07u << ADC_CS_RROBIN_SHIFT)
                | ADC_CS_START_MANY);

    b_initialised = true;

    /* Drain the FIFO every 1 ms from a cyclic handler (see poll). */
    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)adc_sampler_poll;
    ccyc.cyctim = (RELTIM)ADC_POLL_PERIOD_MS;
    ccyc.cycphs = (RELTIM)ADC_POLL_PERIOD_MS;
    cycid       = tk_cre_cyc(&ccyc);

    return ((cycid < E_OK) ? (ER)cycid : E_OK);
}

/*
 * Drain the FIFO.  Runs every 1 ms from a cyclic handler (timer interrupt
 * context), so no task -- including the template's higher-priority USB and
 * Wi-Fi service tasks -- can delay it long enough for the 4-entry FIFO to
 * overflow.  It makes no kernel call.
 */
static void adc_sampler_poll(void *p_exinf)
{
    uint32_t entries[4];
    uint32_t now_us;
    uint32_t level;
    uint32_t idx;

    (void)p_exinf;

    if (b_initialised)
    {
        if (0u != (in_w(ADC_FCS) & ADC_FCS_OVER))
        {
            recover_from_overrun();
        }
        else
        {
            /* Read everything first, then stamp: the newest entry was
               converted "now", each older one one sample gap earlier. */
            now_us = RP2040_NOW_US();
            level  = 0u;

            while ((0u == (in_w(ADC_FCS) & ADC_FCS_EMPTY)) && (level < 4u))
            {
                entries[level] = in_w(ADC_FIFO);
                level          = level + 1u;
            }

            for (idx = 0u; idx < level; idx++)
            {
                if (0u == (entries[idx] & ADC_FIFO_ERR))
                {
                    store_sample(next_channel,
                                 entries[idx] & ADC_FIFO_VAL_MASK,
                                 now_us - ((level - 1u - idx)
                                           * ADC_SAMPLE_GAP_US));
                }

                next_channel = (uint8_t)((next_channel + 1u) % ADC_CH_COUNT);
            }
        }
    }
}

EXPORT bool adc_sampler_read(uint8_t channel, adc_sample_t *p_out)
{
    bool b_ok = false;

    if ((channel < ADC_CH_COUNT) && (NULL != p_out) && b_have_sample[channel])
    {
        /* One word, one sample: no tearing possible. */
        unpack_sample(latest[channel], RP2040_NOW_US(), p_out);
        b_ok = true;
    }

    return (b_ok);
}

/*
 * True if a sample stamped sample_us is strictly newer than after_us: a
 * small positive unsigned difference, correct across the timer wrap.
 */
static bool is_newer(uint32_t sample_us, uint32_t after_us)
{
    uint32_t diff = sample_us - after_us;

    return ((0u != diff) && (diff <= 0x7FFFFFFFu));
}

EXPORT uint8_t adc_sampler_since(uint32_t after_us, adc_sample_t *p_out,
                                 uint8_t max_out)
{
    adc_sample_t sample;
    uint32_t     now_us;
    uint32_t     first;
    uint32_t     oldest;
    uint32_t     newest;
    uint32_t     scan;
    uint8_t      count = 0u;

    if ((NULL != p_out) && (0u != max_out))
    {
        /* One snapshot of the index: the poll may advance it meanwhile. */
        newest = ring_write_index;
        now_us = RP2040_NOW_US();
        oldest = (newest > ADC_RING_LEN) ? (newest - ADC_RING_LEN) : 0u;

        /* Walk BACK from the newest entry to the first one that is new.
           Normally that is one or two steps, so the 1 ms tick does not
           pay for a scan of the whole ring. */
        first = newest;
        while (first > oldest)
        {
            unpack_sample(ring[(first - 1u) % ADC_RING_LEN], now_us, &sample);
            if (!is_newer(sample.timestamp_us, after_us))
            {
                break;
            }
            first = first - 1u;
        }

        /* Copy forward, oldest first. */
        for (scan = first; (scan < newest) && (count < max_out);
             scan++)
        {
            unpack_sample(ring[scan % ADC_RING_LEN], now_us, &p_out[count]);
            count = (uint8_t)(count + 1u);
        }
    }

    return (count);
}

EXPORT uint32_t adc_sampler_overruns(void)
{
    return (overrun_count);
}

EXPORT void adc_sampler_selftest(void)
{
    adc_sample_t sample;
    uint8_t      chan;

    tm_printf((UB*)"[ADC] CS=%08x FCS=%08x DIV=%08x\n",
              in_w(ADC_CS), in_w(ADC_FCS), in_w(ADC_DIV));
    tm_printf((UB*)"[ADC] samples=%d overruns=%d\n", (INT)sample_count,
              (INT)overrun_count);

    for (chan = 0u; chan < ADC_CH_COUNT; chan++)
    {
        if (adc_sampler_read(chan, &sample))
        {
            tm_printf((UB*)"[ADC] ch%d = %d (t=%d us)\n",
                      (INT)chan, (INT)sample.value, (INT)sample.timestamp_us);
        }
        else
        {
            tm_printf((UB*)"[ADC] ch%d = no sample yet\n", (INT)chan);
        }
    }
}

/* end of file */
