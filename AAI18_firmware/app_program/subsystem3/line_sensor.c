/*
 * line_sensor.c
 *
 * Subsystem 3 -- the 5 ms line-following task and calibration.
 *
 * Reads the samples published by subsystem3/adc_sampler.  Never touches an
 * ADC register, which is what keeps the shared-ADC conflict from
 * reappearing (Design Review V2 section 9).
 *
 * SENSOR LAYOUT: two sensors STRADDLE the line, one just outside each
 * edge (subsystem3.h, S3_SWAP_SIDES).
 *   both white  -> centred (or, after a long dark spell, lost outward)
 *   left black  -> the car has drifted right: steer left
 *   right black -> the car has drifted left: steer right
 *   both black  -> a junction: keep straight and tell the Controller
 *
 * LINE READING = ONE WORD (CR1 items 2 and 6)
 * -------------------------------------------
 * The reading Motion Control steers on is error (7 bits), valid (1 bit)
 * and a 24-bit millisecond timestamp, packed into one 32-bit word.  One
 * aligned word cannot tear on either core, so no lock is needed at all.
 *
 * SINGLE WRITER: every variable below is written by line_task alone,
 * except b_active / b_calibrating (written by the Controller, read here)
 * and the thresholds (written by s3_calibrate in the Controller's INIT,
 * before line following can start; each is one 16-bit store).
 */

#include "subsystem3.h"
#include "adc_sampler.h"
#include "../subsystem5/subsystem5.h"

#define LS_TICK_MS          (5u)        /* line_task period               */
#define LS_OUTWARD_TICKS    (S3_OUTWARD_MS / LS_TICK_MS)
#define LS_WHITE_LOST_TICKS (S3_WHITE_LOST_MS / LS_TICK_MS)
#ifndef S3_CURVE_STEP                   /* older subsystem3.h */
#define S3_CURVE_STEP       (8)
#define S3_CURVE_MAX        (32)
#endif
#ifndef S3_JUNCTION_STRAIGHT            /* older subsystem3.h */
#define S3_JUNCTION_STRAIGHT (1)
#endif
#ifndef S3_CORNER_MS                    /* older subsystem3.h */
#define S3_CORNER_MS        (100u)
#endif
#define LS_CORNER_TICKS     (S3_CORNER_MS / LS_TICK_MS)

/* ---------------------------------------------------------------------
 * Private state
 * --------------------------------------------------------------------- */

static volatile uint32_t         published_word = 0u;
static volatile bool             b_published    = false;
static volatile msg_line_state_t line_state     = MSG_LINE_IDLE;

static volatile bool b_active      = false;   /* written by Controller */
static volatile bool b_calibrating = false;   /* written by Controller */

/* Per-channel calibration: index 0 = left sensor, 1 = right sensor. */
typedef struct
{
    uint16_t threshold;     /* midpoint between white and black           */
    uint16_t band;          /* hysteresis band                            */
    uint16_t black;         /* full-black reading, for steering strength  */
} ls_channel_t;

static volatile ls_channel_t chan_cal[2] =
{
    { (uint16_t)(S3_ADC_MAX / 2u), (uint16_t)S3_HYSTERESIS,
      (uint16_t)S3_ADC_MAX },
    { (uint16_t)(S3_ADC_MAX / 2u), (uint16_t)S3_HYSTERESIS,
      (uint16_t)S3_ADC_MAX }
};

/* line_task private. */
static bool     b_was_active   = false;
static bool     b_dark[2]      = { false, false };
static uint16_t dark_ticks[2]  = { 0u, 0u };   /* current dark spell    */
static int8_t   lost_dir       = 0;            /* -1 / +1 while lost    */
static int8_t   last_side      = 0;            /* -1 L / +1 R last seen */
static int32_t  curve_boost    = 0;            /* extra steer in a curve */
static uint16_t white_ticks    = 0u;           /* both white, in a row  */
static int8_t   prev_error     = 0;            /* last published error  */
static int8_t   both_error     = 0;            /* held while both black */
static volatile bool b_force_straight = false; /* written by Controller */
static uint8_t  lost_count     = 0u;
static uint8_t  both_ticks     = 0u;           /* both black, in a row  */
static uint8_t  clear_ticks    = 0u;           /* both white, in a row  */
static bool     b_junction_armed = true;
static volatile uint16_t junction_count = 0u;
static volatile bool     b_line_seen    = false;  /* a sensor on black */

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
    bool     b_now = b_was_dark;

    if ((uint32_t)value > upper)
    {
        b_now = true;
    }
    else if ((uint32_t)value < lower)
    {
        b_now = false;
    }
    else
    {
        /* Inside the band: keep the previous reading. */
    }

    return (b_now);
}

/*
 * Steering strength for a sensor on black: S3_ERR_MIN just over its
 * threshold, rising to S3_ERR_MAX at its full-black reading.
 */
static int32_t strength(uint8_t idx, uint16_t value)
{
    int32_t span = (int32_t)chan_cal[idx].black
                 - (int32_t)chan_cal[idx].threshold;
    int32_t over = (int32_t)value - (int32_t)chan_cal[idx].threshold;
    int32_t mag  = S3_ERR_MIN;

    if ((span > 0) && (over > 0))
    {
        mag = S3_ERR_MIN + ((over * (S3_ERR_MAX - S3_ERR_MIN)) / span);
        mag = (mag > S3_ERR_MAX) ? S3_ERR_MAX : mag;
    }

    return (mag);
}

/* Limit a steering magnitude to the message range. */
static int32_t clamp_error(int32_t magnitude)
{
    return ((magnitude > MSG_LIM_LINE_ERROR) ? MSG_LIM_LINE_ERROR
                                             : magnitude);
}

/*
 * CURVE BOOST.  On a straight the car touches the line on alternate
 * sides; in a steady curve it touches the INSIDE sensor again and again
 * (bench: a tight left circle touched left six times running, steering
 * only 40 then 8 between touches, and drifted out of the circle).  Each
 * new touch on the same side as the last adds S3_CURVE_STEP, up to
 * S3_CURVE_MAX, to both the touch steer and the gentle steer between
 * touches; a touch on the other side clears it.
 */
static void note_touch(int8_t side, uint8_t idx)
{
    if (0u == dark_ticks[idx])
    {
        /* First sample of this dark spell. */
        if (side == last_side)
        {
            curve_boost = curve_boost + S3_CURVE_STEP;
            curve_boost = (curve_boost > S3_CURVE_MAX) ? S3_CURVE_MAX
                                                       : curve_boost;
        }
        else
        {
            curve_boost = 0;
        }
    }
}

/*
 * Steering magnitude for one sensor on black: its strength plus the curve
 * boost, and FULL LOCK once it has stayed on black for S3_OUTWARD_MS --
 * the line is under it and bending away faster than the car is turning
 * (bench: the right sensor stayed black for ~0.6 s in the end loop while
 * the car steered only 32, and it ran off the mat).
 */
static int32_t touch_steer(uint8_t idx, uint16_t value)
{
    int32_t magnitude = clamp_error(strength(idx, value) + curve_boost);

    if (dark_ticks[idx] >= LS_OUTWARD_TICKS)
    {
        magnitude = MSG_LIM_LINE_ERROR;
    }

    return (magnitude);
}

/* Count a dark spell, saturating. */
static void count_dark(uint8_t idx)
{
    if (!b_dark[idx])
    {
        dark_ticks[idx] = 0u;
    }
    else if (dark_ticks[idx] < 0xFFFFu)
    {
        dark_ticks[idx] = (uint16_t)(dark_ticks[idx] + 1u);
    }
    else
    {
        /* Saturated. */
    }
}

/*
 * Junction detector: both black for S3_JUNCTION_SAMPLES in a row reports
 * once; both white for S3_JUNCTION_CLEAR_SAMPLES re-arms it.
 */
static void watch_junction(void)
{
    if (b_dark[0] && b_dark[1])
    {
        clear_ticks = 0u;
        if (both_ticks < 0xFFu)
        {
            both_ticks = (uint8_t)(both_ticks + 1u);
        }
        if (b_junction_armed && (both_ticks >= S3_JUNCTION_SAMPLES))
        {
            b_junction_armed = false;
            junction_count   = (uint16_t)(junction_count + 1u);
            s5_post_event(S5_EVT_JUNCTION);
        }
    }
    else
    {
        both_ticks = 0u;
        if (!b_dark[0] && !b_dark[1] && (clear_ticks < 0xFFu))
        {
            clear_ticks = (uint8_t)(clear_ticks + 1u);
        }
        if (clear_ticks >= S3_JUNCTION_CLEAR_SAMPLES)
        {
            b_junction_armed = true;
        }
    }
}

/*
 * Steering error.  Negative = the line is to the car's LEFT (steer left),
 * positive = to its right.  *pb_any_line is false only while LOST.
 */
static int8_t compute_error(uint16_t left_value, uint16_t right_value,
                            bool *pb_any_line)
{
    int32_t error = 0;

    *pb_any_line = true;

    if (b_dark[0] || b_dark[1])
    {
        white_ticks = 0u;
    }
    else if (white_ticks < 0xFFFFu)
    {
        white_ticks = (uint16_t)(white_ticks + 1u);
    }
    else
    {
        /* Saturated. */
    }

    if (b_dark[0] && b_dark[1])
    {
        /* Both black: a junction OR a sharp corner (both sensors cross
           the bend's outer edge together).  Keep steering the way the
           car already was: straight at a junction met square on, still
           turning in a corner.  (Was: 0, which drove the car straight
           off the mat at the first 90 degree corner.) */
        lost_dir = 0;
        if (0u == both_ticks)
        {
            /* First both-black sample: decide once, hold it.  In a
               corner one sensor had been on the bend for a while; at a
               crossing met at an angle one sensor touched the crossing
               line only moments before the other (bench: an X crossing
               taken at full lock drove the car off the mat). */
            both_error = ((dark_ticks[0] >= LS_CORNER_TICKS)
                          || (dark_ticks[1] >= LS_CORNER_TICKS))
                       ? prev_error : 0;

            /* A STRAIGHT barcode is waiting: this is a junction, not a
               corner, whatever the car was doing (bench: finishing the
               loop late, still turning, it took the junction as a corner
               and went round again). */
            if (b_force_straight || (1 == S3_JUNCTION_STRAIGHT))
            {
                both_error       = 0;
                b_force_straight = false;
            }
        }
        error = (int32_t)both_error;
    }
    else if (b_dark[0])
    {
        note_touch(-1, 0u);
        lost_dir  = 0;
        last_side = -1;
        error     = -touch_steer(0u, left_value);
    }
    else if (b_dark[1])
    {
        note_touch(1, 1u);
        lost_dir  = 0;
        last_side = 1;
        error     = touch_steer(1u, right_value);
    }
    else
    {
        /* Both white: the line is between the sensors OR has slipped out
           past the one that saw it last -- these sensors switch, so the
           two look the same.  Steering gently TOWARD the last sensor
           settles it either way: a line in the middle drifts across to
           the OTHER sensor, a line outside comes back under the SAME
           one.  A long dark spell first means a sharp curve took it
           out: turn fully.  Too long all white: lost. */
        if (0 != lost_dir)
        {
            /* Still lost: nothing has seen the line yet. */
        }
        else if (dark_ticks[0] >= LS_OUTWARD_TICKS)
        {
            lost_dir = -1;
        }
        else if (dark_ticks[1] >= LS_OUTWARD_TICKS)
        {
            lost_dir = 1;
        }
        else if ((0 != last_side) && (white_ticks >= LS_WHITE_LOST_TICKS))
        {
            lost_dir = last_side;
        }
        else
        {
            error = (int32_t)last_side
                  * clamp_error(S3_SEEK_ERROR + curve_boost);
        }

        if (0 != lost_dir)
        {
            error        = (int32_t)lost_dir * MSG_LIM_LINE_ERROR;
            *pb_any_line = false;
        }
    }

    prev_error = (int8_t)error;

    return ((int8_t)error);
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
    adc_sample_t s_right;
    int8_t       error;
    bool         b_any_line;

    if (adc_sampler_read(S3_CH_LEFT, &s_left)
        && adc_sampler_read(S3_CH_RIGHT, &s_right))
    {
        /* The dark-spell length must be read BEFORE this sample ends it,
           so compute_error sees how long the spell was. */
        b_dark[0] = (s_left.value < S3_AIR_LEVEL)
                  && is_dark(s_left.value, chan_cal[0].threshold,
                             chan_cal[0].band, b_dark[0]);
        b_dark[1] = (s_right.value < S3_AIR_LEVEL)
                  && is_dark(s_right.value, chan_cal[1].threshold,
                             chan_cal[1].band, b_dark[1]);
        b_line_seen = b_dark[0] || b_dark[1];
        error     = compute_error(s_left.value, s_right.value, &b_any_line);
        count_dark(0u);
        count_dark(1u);
        watch_junction();

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
            /* Confirmed loss.  Publish invalid so Motion Control turns
               toward the line, and tell the Controller. */
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
    uint32_t airborne;      /* samples ignored: nothing under the sensor */
    uint32_t tail_sum;      /* last S3_CAL_TAIL_SAMPLES: start position  */
    uint32_t tail_taken;
} cal_stats_t;

/*
 * Collect statistics for one channel over the calibration pass.
 */
static void cal_accumulate(uint8_t channel, cal_stats_t *p_stats,
                           bool b_tail)
{
    adc_sample_t sample;

    if (!adc_sampler_read(channel, &sample))
    {
        /* No sample yet: nothing to record. */
    }
    else if (sample.value >= S3_AIR_LEVEL)
    {
        p_stats->airborne = p_stats->airborne + 1u;
    }
    else
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
        if (b_tail)
        {
            p_stats->tail_sum   = p_stats->tail_sum + sample.value;
            p_stats->tail_taken = p_stats->tail_taken + 1u;
        }
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
 * Set one line channel from its statistics.  Swept: midpoint of its white
 * (minimum) and black (maximum).  Not swept: white + S3_STATIC_OFFSET.
 */
static void cal_line_channel(uint8_t idx, const cal_stats_t *p_stats)
{
    uint32_t white = p_stats->min_val;
    uint32_t black;

    if (cal_swept(p_stats))
    {
        black = p_stats->max_val;
    }
    else
    {
        black = white + (2u * S3_STATIC_OFFSET);
    }

    chan_cal[idx].threshold = (uint16_t)((white + black) / 2u);
    chan_cal[idx].band      = band_for(black - white);
    chan_cal[idx].black     = (uint16_t)black;
}

/* True if the channel's start-position mean is below its threshold. */
static bool cal_ends_white(uint8_t idx, const cal_stats_t *p_stats)
{
    uint32_t mean = (0u == p_stats->tail_taken) ? 0u
                  : (p_stats->tail_sum / p_stats->tail_taken);

    return (mean < (uint32_t)chan_cal[idx].threshold);
}

/*
 * Barcode threshold: the channel's own midpoint if it was swept, otherwise
 * halfway between its (white) mean and the line sensors' black reading.
 * PROVISIONAL: assumes the barcode sensor responds like the line sensors.
 */
static void cal_barcode(const cal_stats_t *p_bc, uint32_t black_ref)
{
    uint32_t threshold = 0u;

    (void)black_ref;    /* kept for the signature; no longer needed */

    if (cal_swept(p_bc))
    {
        threshold = ((uint32_t)p_bc->min_val + p_bc->max_val) / 2u;
    }
    else if (0u != p_bc->taken)
    {
        /* Not swept: its white plus the same margin as the line sensors
           (the sensors switch, white ~150 -> black 1000+).  Was: halfway
           to the line sensors' black, which left it at the 2047 default
           when that reference was weak, so the bars (~1400-1900) never
           read black. */
        threshold = (uint32_t)p_bc->min_val + S3_STATIC_OFFSET;
    }
    else
    {
        /* No usable reference: keep the current threshold. */
    }

    if (0u != threshold)
    {
        s3_set_barcode_threshold((uint16_t)threshold);
        APP_LOG((UB*)"[S3] barcode threshold %d\n", (INT)threshold);
    }
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT ER s3_calibrate(void)
{
    cal_stats_t stats[ADC_CH_COUNT];
    uint32_t    count;
    uint32_t    black_ref;
    uint8_t     chan;
    bool        b_tail;
    ER          ercd = E_OK;

    for (chan = 0u; chan < ADC_CH_COUNT; chan++)
    {
        stats[chan].min_val    = (uint16_t)S3_ADC_MAX;
        stats[chan].max_val    = 0u;
        stats[chan].sum        = 0u;
        stats[chan].taken      = 0u;
        stats[chan].airborne   = 0u;
        stats[chan].tail_sum   = 0u;
        stats[chan].tail_taken = 0u;
    }

    b_calibrating = true;
    APP_LOG((UB*)"[S3] calibrating for 3 s: slide the car side to side "
                   "across the line, then END with the line BETWEEN the "
                   "two sensors\n");

    for (count = 0u; count < S3_CAL_SAMPLES; count++)
    {
        b_tail = ((count + S3_CAL_TAIL_SAMPLES) >= S3_CAL_SAMPLES);
        for (chan = 0u; chan < ADC_CH_COUNT; chan++)
        {
            cal_accumulate(chan, &stats[chan], b_tail);
        }

        (void)tk_dly_tsk((TMO)S3_CAL_INTERVAL_MS);
    }

    b_calibrating = false;

    if ((0u != stats[S3_CH_LEFT].airborne)
        || (0u != stats[S3_CH_RIGHT].airborne))
    {
        APP_LOG((UB*)"[S3] WARNING: car lifted during calibration "
                       "(%d readings ignored) -- keep it on the surface\n",
                  (INT)(stats[S3_CH_LEFT].airborne
                        + stats[S3_CH_RIGHT].airborne));
    }

    APP_LOG((UB*)"[S3] cal left %d..%d  right %d..%d  barcode %d..%d\n",
              (INT)stats[S3_CH_LEFT].min_val, (INT)stats[S3_CH_LEFT].max_val,
              (INT)stats[S3_CH_RIGHT].min_val,
              (INT)stats[S3_CH_RIGHT].max_val,
              (INT)stats[ADC_CH_BARCODE].min_val,
              (INT)stats[ADC_CH_BARCODE].max_val);

    if ((0u == stats[S3_CH_LEFT].taken) || (0u == stats[S3_CH_RIGHT].taken))
    {
        APP_LOG((UB*)"[S3] CAL FAILED: a line sensor gave no readings\n");
        adc_sampler_selftest();
        ercd = E_IO;
    }
    else
    {
        cal_line_channel(0u, &stats[S3_CH_LEFT]);
        cal_line_channel(1u, &stats[S3_CH_RIGHT]);

        APP_LOG((UB*)"[S3] thresholds: left %d (%s)  right %d (%s)\n",
                  (INT)chan_cal[0].threshold,
                  cal_swept(&stats[S3_CH_LEFT]) ? "swept" : "white+400",
                  (INT)chan_cal[1].threshold,
                  cal_swept(&stats[S3_CH_RIGHT]) ? "swept" : "white+400");

        if (!cal_ends_white(0u, &stats[S3_CH_LEFT])
            || !cal_ends_white(1u, &stats[S3_CH_RIGHT]))
        {
            APP_LOG((UB*)"[S3] CAL FAILED: the car must END with the line "
                           "between the sensors (left %s, right %s)\n",
                      cal_ends_white(0u, &stats[S3_CH_LEFT]) ? "white"
                                                             : "BLACK",
                      cal_ends_white(1u, &stats[S3_CH_RIGHT]) ? "white"
                                                              : "BLACK");
            ercd = E_IO;
        }
    }

    if (E_OK == ercd)
    {
        black_ref = (chan_cal[0].black > chan_cal[1].black)
                  ? chan_cal[0].black : chan_cal[1].black;
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
                /* Rising edge of START: begin from a clean slate. */
                lost_count       = 0u;
                lost_dir         = 0;
                last_side        = 0;
                curve_boost      = 0;
                prev_error       = 0;
                white_ticks      = 0u;
                dark_ticks[0]    = 0u;
                dark_ticks[1]    = 0u;
                both_ticks       = 0u;
                clear_ticks      = 0u;
                b_junction_armed = true;
                junction_count   = 0u;
            }
            track_once();
        }
        else
        {
            line_state  = b_calibrating ? MSG_LINE_CAL : MSG_LINE_IDLE;
            b_line_seen = false;
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

EXPORT void s3_get_thresholds(uint16_t *p_left, uint16_t *p_right)
{
    if ((NULL != p_left) && (NULL != p_right))
    {
        *p_left  = chan_cal[0].threshold;
        *p_right = chan_cal[1].threshold;
    }
}

EXPORT void s3_force_straight(bool b_on)
{
    b_force_straight = b_on;
}

EXPORT bool s3_line_seen(void)
{
    return (b_line_seen);
}

EXPORT uint16_t s3_get_junction_count(void)
{
    return (junction_count);
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