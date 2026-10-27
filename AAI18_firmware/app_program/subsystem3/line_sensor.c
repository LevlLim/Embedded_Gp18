/*
 * line_sensor.c
 *
 * Subsystem 3 -- the 5 ms line-following task and calibration.
 *
 * Reads the samples published by subsystem3/adc_sampler.  Never touches an
 * ADC register, which is what keeps the shared-ADC conflict from
 * reappearing (Design Review V2 section 9).
 *
 * LINE READING = ONE WORD (CR1 items 2 and 6)
 * -------------------------------------------
 * The reading Motion Control steers on is error (7 bits), valid (1 bit)
 * and a 24-bit millisecond timestamp, packed into one 32-bit word.  The
 * old three-field struct (12 bytes) was written under DI(), which does
 * not protect a reader on the other core; one aligned word cannot tear
 * on either core, so no lock is needed at all.
 *
 * SINGLE WRITER: every variable below is written by line_task alone,
 * except b_active / b_calibrating (written by the Controller, read here)
 * and the thresholds (written by s3_calibrate in the Controller's INIT,
 * before line following can start; each is one 16-bit store).
 */

#include "subsystem3.h"
#include "adc_sampler.h"
#include "../subsystem5/subsystem5.h"

/* ---------------------------------------------------------------------
 * Private state
 * --------------------------------------------------------------------- */

static volatile uint32_t         published_word = 0u;
static volatile bool             b_published    = false;
static volatile msg_line_state_t line_state     = MSG_LINE_IDLE;

static volatile bool b_active      = false;   /* written by Controller */
static volatile bool b_calibrating = false;   /* written by Controller */

/* Per-channel thresholds: the midpoint between black and white. */
static volatile uint16_t threshold_left   = (uint16_t)(S3_ADC_MAX / 2u);
static volatile uint16_t threshold_centre = (uint16_t)(S3_ADC_MAX / 2u);
static volatile uint16_t band_left        = (uint16_t)S3_HYSTERESIS;
static volatile uint16_t band_centre      = (uint16_t)S3_HYSTERESIS;

/* line_task private. */
static bool    b_was_active  = false;
static bool    b_left_dark   = false;
static bool    b_centre_dark = false;
static int8_t  last_error    = 0;
static uint8_t lost_count    = 0u;

/* ---------------------------------------------------------------------
 * Private functions
 * --------------------------------------------------------------------- */

/*
 * Schmitt-trigger comparison.  The IR sensors return a HIGHER value over
 * black.  A channel turns "dark" above threshold + half the band and
 * "light" only below threshold - half the band, so noise around the
 * threshold cannot flicker the reading.
 */
static bool is_dark(uint16_t value, uint16_t threshold, uint16_t band,
                    bool b_was_dark)
{
    uint32_t half  = (uint32_t)band / 2u;
    uint32_t upper = (uint32_t)threshold + half;
    uint32_t lower = ((uint32_t)threshold > half)
                   ? ((uint32_t)threshold - half) : 0u;
    bool     b_dark = b_was_dark;

    if ((uint32_t)value > upper)
    {
        b_dark = true;
    }
    else if ((uint32_t)value < lower)
    {
        b_dark = false;
    }
    else
    {
        /* Inside the band: keep the previous reading. */
    }

    return (b_dark);
}

/*
 * Steering error from the two line sensors.  Negative = the line is to
 * the car's LEFT (steer left), positive = to its right.  S3_SIDE_SIGN is
 * +1 when the side sensor is right of centre, -1 when left.  With only
 * two sensors the error is coarse (a Week 6 review risk that remains):
 *   centre only     ->   0                  centred
 *   centre + side   -> +-20 toward the side  slightly off
 *   side only       -> +-50 toward the side  well off
 *   neither         -> +-63 keep turning the way we last were; from
 *                      centred, AWAY from the side sensor, since the line
 *                      must have left on the side that has no sensor.
 *
 * (b_left_dark is the GP26 channel, wherever it is mounted.)
 */
static int8_t compute_error(bool *pb_any_line)
{
    int8_t error;

    *pb_any_line = true;

    if (b_centre_dark && b_left_dark)
    {
        error = (int8_t)(20 * S3_SIDE_SIGN);
    }
    else if (b_centre_dark)
    {
        error = 0;
    }
    else if (b_left_dark)
    {
        error = (int8_t)(50 * S3_SIDE_SIGN);
    }
    else
    {
        if (0 == last_error)
        {
            error = (int8_t)(-MSG_LIM_LINE_ERROR * S3_SIDE_SIGN);
        }
        else
        {
            error = (last_error < 0) ? (int8_t)-MSG_LIM_LINE_ERROR
                                     : (int8_t)MSG_LIM_LINE_ERROR;
        }
        *pb_any_line = false;
    }

    if (*pb_any_line)
    {
        last_error = error;
    }

    return (error);
}

/* Publish one reading as one word: a single store, never torn. */
static void publish_reading(int8_t error, bool b_valid)
{
    published_word = msg_pack_line((int32_t)error, b_valid, app_now_ms());
    b_published    = true;
}

/*
 * One active cycle: read both channels, classify, publish.
 */
static void track_once(void)
{
    adc_sample_t s_left;
    adc_sample_t s_centre;
    int8_t       error;
    bool         b_any_line;

    if (adc_sampler_read(ADC_CH_LINE_L, &s_left)
        && adc_sampler_read(ADC_CH_LINE_C, &s_centre))
    {
        b_left_dark   = is_dark(s_left.value, threshold_left, band_left,
                                b_left_dark);
        b_centre_dark = is_dark(s_centre.value, threshold_centre,
                                band_centre, b_centre_dark);
        error         = compute_error(&b_any_line);

        if (b_any_line)
        {
            lost_count = 0u;
            line_state = MSG_LINE_TRACK;
            publish_reading(error, true);
        }
        else if (lost_count < S3_LOST_DEBOUNCE)
        {
            /* Within the debounce window: keep steering, still valid. */
            lost_count = (uint8_t)(lost_count + 1u);
            publish_reading(error, true);
        }
        else
        {
            /* Confirmed loss.  Publish invalid so Motion Control slows,
               and tell the Controller, which decides what to do. */
            if (MSG_LINE_LOST != line_state)
            {
                s5_post_event(S5_EVT_LINE_LOST);
            }
            line_state = MSG_LINE_LOST;
            publish_reading(error, false);
        }
    }
    else
    {
        /* No ADC sample at all: the sampler is not running. */
        publish_reading(0, false);
    }
}

/* Per-channel statistics gathered during calibration. */
typedef struct
{
    uint16_t min_val;
    uint16_t max_val;
    uint32_t sum;
    uint32_t taken;
} cal_stats_t;

/*
 * Collect min/max/sum for one channel over the calibration pass.
 */
static void cal_accumulate(uint8_t channel, cal_stats_t *p_stats)
{
    adc_sample_t sample;

    if (adc_sampler_read(channel, &sample))
    {
        if (sample.value < p_stats->min_val)
        {
            p_stats->min_val = sample.value;
        }
        if (sample.value > p_stats->max_val)
        {
            p_stats->max_val = sample.value;
        }
        p_stats->sum   = p_stats->sum + sample.value;
        p_stats->taken = p_stats->taken + 1u;
    }
}

/* True if the channel itself saw both black and white. */
static bool cal_swept(const cal_stats_t *p_stats)
{
    return ((0u != p_stats->taken) && (p_stats->max_val > p_stats->min_val)
            && ((uint32_t)(p_stats->max_val - p_stats->min_val)
                >= S3_MIN_SWEEP_CONTRAST));
}

/* Hysteresis band for a channel: a quarter of its contrast, clamped. */
static uint16_t band_for(uint32_t contrast)
{
    uint32_t band = contrast / 4u;

    if (band > S3_HYSTERESIS)
    {
        band = S3_HYSTERESIS;
    }
    else if (band < S3_MIN_BAND)
    {
        band = S3_MIN_BAND;
    }
    else
    {
        /* In range. */
    }

    return ((uint16_t)band);
}

/* Mean reading of a channel (0 if it produced no samples). */
static uint32_t cal_mean(const cal_stats_t *p_stats)
{
    return ((0u == p_stats->taken) ? 0u : (p_stats->sum / p_stats->taken));
}

/*
 * Barcode threshold: the channel's own midpoint if it was swept, otherwise
 * halfway between its (white) mean and the line sensors' black reading.
 * PROVISIONAL: assumes the barcode sensor responds like the line sensors.
 */
static void cal_barcode(const cal_stats_t *p_bc, uint32_t black_ref)
{
    uint32_t threshold = 0u;

    if (cal_swept(p_bc))
    {
        threshold = ((uint32_t)p_bc->min_val + p_bc->max_val) / 2u;
    }
    else if ((0u != p_bc->taken)
             && (black_ref >= (cal_mean(p_bc) + S3_MIN_CONTRAST)))
    {
        threshold = (cal_mean(p_bc) + black_ref) / 2u;
    }
    else
    {
        /* No usable reference: keep the current threshold. */
    }

    if (0u != threshold)
    {
        s3_set_barcode_threshold((uint16_t)threshold);
        tm_printf((UB*)"[S3] barcode threshold %d\n", (INT)threshold);
    }
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT ER s3_calibrate(void)
{
    cal_stats_t stats[ADC_CH_COUNT];
    uint32_t    count;
    uint32_t    black_ref = 0u;
    uint8_t     chan;
    ER          ercd = E_OK;

    for (chan = 0u; chan < ADC_CH_COUNT; chan++)
    {
        stats[chan].min_val = (uint16_t)S3_ADC_MAX;
        stats[chan].max_val = 0u;
        stats[chan].sum     = 0u;
        stats[chan].taken   = 0u;
    }

    b_calibrating = true;
    tm_printf((UB*)"[S3] calibrating for 3 s: centre sensor on the line, "
                   "or sweep the car across it\n");

    for (count = 0u; count < S3_CAL_SAMPLES; count++)
    {
        for (chan = 0u; chan < ADC_CH_COUNT; chan++)
        {
            cal_accumulate(chan, &stats[chan]);
        }
        (void)tk_dly_tsk((TMO)S3_CAL_INTERVAL_MS);
    }

    b_calibrating = false;

    tm_printf((UB*)"[S3] cal L %d..%d  C %d..%d  B %d..%d\n",
              (INT)stats[0].min_val, (INT)stats[0].max_val,
              (INT)stats[1].min_val, (INT)stats[1].max_val,
              (INT)stats[2].min_val, (INT)stats[2].max_val);

    if (cal_swept(&stats[ADC_CH_LINE_C]))
    {
        /* CENTRE SWEPT: its own midpoint. */
        threshold_centre = (uint16_t)(((uint32_t)stats[1].min_val
                                       + stats[1].max_val) / 2u);
        band_centre      = band_for((uint32_t)stats[1].max_val
                                    - stats[1].min_val);
        black_ref        = stats[ADC_CH_LINE_C].max_val;
    }
    else if (cal_mean(&stats[ADC_CH_LINE_C])
             >= (cal_mean(&stats[ADC_CH_LINE_L]) + S3_MIN_CONTRAST))
    {
        /* CENTRE STATIC: sitting on the line.  Its white level is not
           seen, so take the side sensor's white as the estimate. */
        black_ref        = cal_mean(&stats[ADC_CH_LINE_C]);
        threshold_centre = (uint16_t)((cal_mean(&stats[ADC_CH_LINE_L])
                                       + black_ref) / 2u);
        band_centre      = band_for(black_ref
                                    - cal_mean(&stats[ADC_CH_LINE_L]));
    }
    else
    {
        ercd = E_IO;
    }

    if (E_OK != ercd)
    {
        /* Centre sensor saw no line at all. */
    }
    else if (cal_swept(&stats[ADC_CH_LINE_L]))
    {
        /* SIDE SWEPT: its own midpoint. */
        threshold_left = (uint16_t)(((uint32_t)stats[0].min_val
                                     + stats[0].max_val) / 2u);
        band_left      = band_for((uint32_t)stats[0].max_val
                                  - stats[0].min_val);
    }
    else
    {
        /* SIDE STATIC: only its white level is known. */
        threshold_left = (uint16_t)(cal_mean(&stats[ADC_CH_LINE_L])
                                    + S3_SIDE_STATIC_OFFSET);
        band_left      = band_for(2u * S3_SIDE_STATIC_OFFSET);
    }

    if (E_OK == ercd)
    {
        tm_printf((UB*)"[S3] thresholds: side %d (+-%d%s)  centre %d "
                       "(+-%d%s)\n",
                  (INT)threshold_left, (INT)(band_left / 2u),
                  cal_swept(&stats[ADC_CH_LINE_L]) ? ", swept" : "",
                  (INT)threshold_centre, (INT)(band_centre / 2u),
                  cal_swept(&stats[ADC_CH_LINE_C]) ? ", swept" : "");
    }
    else
    {
        tm_printf((UB*)"[S3] CAL FAILED: put the centre sensor on the line "
                       "(side sensor on white) or sweep across it\n");

        /* "4095..0" on a channel means it produced no samples at all:
           print the ADC state so the cause is visible on the console. */
        adc_sampler_selftest();
    }

    if (E_OK == ercd)
    {
        cal_barcode(&stats[ADC_CH_BARCODE], black_ref);
    }

    return (ercd);
}

EXPORT void line_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        (void)app_wait_period();

        if (b_active)
        {
            if (!b_was_active)
            {
                /* Rising edge of START: begin from a clean debounce. */
                lost_count = 0u;
            }
            track_once();
        }
        else
        {
            line_state = b_calibrating ? MSG_LINE_CAL : MSG_LINE_IDLE;
            publish_reading(0, false);
        }

        b_was_active = b_active;
    }
}

EXPORT bool s3_read_line_word(uint32_t *p_word)
{
    bool b_ok = false;

    if ((NULL != p_word) && b_published)
    {
        *p_word = published_word;
        b_ok    = true;
    }

    return (b_ok);
}

EXPORT void s3_get_thresholds(uint16_t *p_side, uint16_t *p_centre)
{
    if ((NULL != p_side) && (NULL != p_centre))
    {
        *p_side   = threshold_left;
        *p_centre = threshold_centre;
    }
}

EXPORT msg_line_state_t s3_get_line_state(void)
{
    return (line_state);
}

EXPORT void s3_set_active(bool b_on)
{
    b_active = b_on;
}

EXPORT bool s3_barcode_active(void)
{
    /* Barcode capture follows line tracking: both are off until START. */
    return (b_active);
}

/* end of file */
