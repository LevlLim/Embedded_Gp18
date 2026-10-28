/**
 * @file line_sensor.c
 * @brief Line-edge hugging and transactional calibration of all IR inputs.
 *
 * Controller is the sole writer of g_control and calibration. Line task is
 * the sole writer of the packed line/status records. Each public shared
 * record is published with release/acquire loads/stores, never cross-core
 * interrupt masking. Motor decisions remain with Buddies 2 and 5.
 */
#include "line_sensor.h"
#include "barcode_decoder.h"
#include "adc_sampler.h"
#include "buddy3_config.h"
#include "../subsystem5/subsystem5.h"

#define CONTROL_ACTIVE      (0x01U)
#define CONTROL_CALIBRATED  (0x02U)
#define CONTROL_CALIBRATING (0x04U)
#define CONTROL_EPOCH_STEP  (0x10U)
#define CONTROL_EPOCH_MASK  (0xF0U)
#define STATUS_STATE_MASK   (0x07U)
#define STATUS_PUBLISHED    (0x08U)
#define LEFT_DARK_FLAG      (0x10U)
#define RIGHT_DARK_FLAG     (0x20U)
#define SENSOR_FLAGS_MASK   (0x07U)
#define SENSOR_VALID_FLAG   (0x08U)
#define SENSOR_STAMP_SHIFT  (4U)
#define SENSOR_STAMP_MASK   (0x0FFFFFFFU)
#define CAL_THRESHOLD_MASK  (0x0FFFU)
#define CAL_BAND_SHIFT      (12U)
#define US_PER_MS           (1000U)

static uint8_t volatile g_control = 0U;
static uint8_t volatile g_status = (uint8_t)MSG_LINE_IDLE;
static uint32_t volatile g_line_word = 0U;
static uint32_t volatile g_left_cal = S3_ADC_MAX / 2U;
static uint32_t volatile g_right_cal = S3_ADC_MAX / 2U;
/* One atomic record: three sensor flags, valid bit, 28-bit ms timestamp. */
static uint32_t volatile g_sensor_word = 0U;
static int8_t g_last_error = 0;
static uint8_t g_junction_count = 0U;
static uint32_t g_junction_sample_us = 0U;
static uint8_t g_line_epoch = 0U;
static uint16_t g_max_line_us = 0U;

typedef struct
{
    uint16_t minimum;
    uint16_t maximum;
    uint32_t last_sample_us;
    uint8_t samples;
} cal_stats_t;

static bool is_dark(uint16_t value, uint32_t calibration, bool b_was_dark)
{
    uint32_t threshold = calibration & CAL_THRESHOLD_MASK;
    uint32_t half = ((calibration >> CAL_BAND_SHIFT) & 0xFFU) / 2U;
    bool b_dark = b_was_dark;

    if ((uint32_t)value > (threshold + half))
    {
        b_dark = true;
    }
    else if (((uint32_t)value + half) < threshold)
    {
        b_dark = false;
    }
    else
    {
        /* State is retained within the calibrated hysteresis band. */
    }
    return (b_dark);
}

static void publish_reading(int8_t error, bool b_valid, uint32_t age_us,
                            msg_line_state_t state, uint8_t sensors)
{
    uint32_t age_ms = (age_us + US_PER_MS - 1U) / US_PER_MS;
    uint32_t stamp_ms = app_now_ms() - age_ms;
    uint32_t word = msg_pack_line(error, b_valid, stamp_ms);
    uint8_t status = (uint8_t)((uint8_t)state | STATUS_PUBLISHED
                              | (sensors << SENSOR_STAMP_SHIFT));
    uint32_t observation = 0U;

    if (b_valid)
    {
        observation = ((stamp_ms & SENSOR_STAMP_MASK) << SENSOR_STAMP_SHIFT)
                      | SENSOR_VALID_FLAG | sensors;
    }

    __atomic_store_n(&g_line_word, word, __ATOMIC_RELEASE);
    __atomic_store_n(&g_status, status, __ATOMIC_RELEASE);
    __atomic_store_n(&g_sensor_word, observation, __ATOMIC_RELEASE);
}

static int8_t compute_error(bool b_left_dark, bool b_right_dark)
{
    int8_t error = 0;

    if (b_left_dark && !b_right_dark)
    {
        error = (int8_t)-S3_LINE_CORRECTION;
    }
    else if (b_right_dark && !b_left_dark)
    {
        error = (int8_t)S3_LINE_CORRECTION;
    }
    else
    {
        /* Both white is aligned; both black has no steering preference. */
    }
    if (b_left_dark != b_right_dark)
    {
        g_last_error = error;
    }
    return (error);
}

static void mark_lost(int8_t error)
{
    g_junction_count = 0U;
    if (MSG_LINE_LOST != s3_get_line_state())
    {
        s5_post_event(S5_EVT_LINE_LOST);
    }
    publish_reading(error, false, 0U, MSG_LINE_LOST, 0U);
}

static void track_once(void)
{
    adc_sample_t left = {0U, 0U};
    adc_sample_t right = {0U, 0U};
    uint32_t now_us = adc_sampler_now_us();
    uint32_t left_age = 0U;
    uint32_t right_age = 0U;
    uint32_t age_us = 0U;
    uint8_t status = __atomic_load_n(&g_status, __ATOMIC_ACQUIRE);
    uint8_t sensors = 0U;
    bool b_left_dark = false;
    bool b_right_dark = false;
    bool b_fresh = adc_sampler_read(ADC_CH_LINE_L, &left)
                   && adc_sampler_read(ADC_CH_LINE_R, &right);
    int8_t error = 0;

    if (b_fresh)
    {
        /* Compute age after the reads: a conversion can occur during them. */
        now_us = adc_sampler_now_us();
        left_age = now_us - left.timestamp_us;
        right_age = now_us - right.timestamp_us;
        age_us = (left_age > right_age) ? left_age : right_age;
        b_fresh = (age_us <= S3_SAMPLE_STALE_US)
            && (left.value < S3_AIR_LEVEL) && (right.value < S3_AIR_LEVEL);
    }
    if (b_fresh)
    {
        b_left_dark = is_dark(left.value,
            __atomic_load_n(&g_left_cal, __ATOMIC_ACQUIRE),
            (0U != (status & LEFT_DARK_FLAG)));
        b_right_dark = is_dark(right.value,
            __atomic_load_n(&g_right_cal, __ATOMIC_ACQUIRE),
            (0U != (status & RIGHT_DARK_FLAG)));
        sensors = (uint8_t)((b_left_dark ? S3_LINE_LEFT_BLACK : 0U)
                     | (b_right_dark ? S3_LINE_RIGHT_BLACK : 0U));
        error = compute_error(b_left_dark, b_right_dark);
        if (b_left_dark && b_right_dark)
        {
            uint32_t sample_us = now_us - age_us;

            /* Count distinct ADC pairs, not repeated task reads of one pair. */
            if ((0U == g_junction_count)
                || (sample_us != g_junction_sample_us))
            {
                g_junction_sample_us = sample_us;
                if (g_junction_count < S3_JUNCTION_CONFIRM_SAMPLES)
                {
                    g_junction_count++;
                }
            }
            if (g_junction_count >= S3_JUNCTION_CONFIRM_SAMPLES)
            {
                sensors |= S3_LINE_JUNCTION;
            }
        }
        else
        {
            g_junction_count = 0U;
        }
        /* White/white must never start a line-lost search in this layout.
         * TRACK now means usable edge-following input, not proof of a line
         * beneath the car. Junction flags are a separate Buddy 3 interface. */
        publish_reading(error, true, age_us, MSG_LINE_TRACK, sensors);
    }
    else
    {
        /* Data unavailable is not TRACK, even if the last reading was. */
        mark_lost(g_last_error);
    }
}

static bool calibration_word(cal_stats_t const * p_stats,
                             uint32_t * p_calibration)
{
    uint32_t contrast = 0U;
    uint32_t band = 0U;
    uint32_t threshold = 0U;
    bool b_valid = (p_stats->samples >= S3_CAL_MIN_SAMPLES)
                   && (p_stats->maximum > p_stats->minimum)
                   && (p_stats->maximum <= S3_ADC_MAX);

    if (b_valid)
    {
        contrast = (uint32_t)p_stats->maximum - p_stats->minimum;
        b_valid = (contrast >= S3_MIN_CONTRAST);
    }
    if (b_valid)
    {
        threshold = ((uint32_t)p_stats->minimum + p_stats->maximum) / 2U;
        band = contrast / 4U;
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
            /* Use the measured band. */
        }
        b_valid = (threshold > (band / 2U))
                  && ((threshold + band / 2U) < S3_ADC_MAX);
        if (b_valid)
        {
            *p_calibration = threshold | (band << CAL_BAND_SHIFT);
        }
    }
    return (b_valid);
}

ER s3_calibrate(void)
{
    cal_stats_t statistics[ADC_CH_COUNT] = {{0U}};
    uint32_t candidates[ADC_CH_COUNT] = {0U};
    adc_sample_t sample = {0U, 0U};
    uint32_t now_us = 0U;
    uint32_t count = 0U;
    uint8_t channel = 0U;
    bool b_valid = true;
    uint8_t control = __atomic_load_n(&g_control, __ATOMIC_ACQUIRE);

    control = (uint8_t)((control & (uint8_t)~CONTROL_CALIBRATED)
                        | CONTROL_CALIBRATING);
    control = (uint8_t)(control + CONTROL_EPOCH_STEP);
    __atomic_store_n(&g_control, control, __ATOMIC_RELEASE);
    s3_barcode_reset();
    for (channel = 0U; channel < ADC_CH_COUNT; channel++)
    {
        statistics[channel].minimum = (uint16_t)S3_ADC_MAX;
    }
    APP_LOG((UB *)"[S3] 3 s calibration: sweep EACH IR sensor across "
                   "black and white\n");
    for (count = 0U; count < S3_CAL_SAMPLES; count++)
    {
        for (channel = 0U; channel < ADC_CH_COUNT; channel++)
        {
            if (adc_sampler_read(channel, &sample))
            {
                now_us = adc_sampler_now_us();
                if (((now_us - sample.timestamp_us) <= S3_SAMPLE_STALE_US)
                    && (sample.value < S3_AIR_LEVEL)
                    && ((0U == statistics[channel].samples)
                        || (statistics[channel].last_sample_us
                            != sample.timestamp_us)))
                {
                    if (sample.value < statistics[channel].minimum)
                    {
                        statistics[channel].minimum = sample.value;
                    }
                    if (sample.value > statistics[channel].maximum)
                    {
                        statistics[channel].maximum = sample.value;
                    }
                    statistics[channel].last_sample_us = sample.timestamp_us;
                    statistics[channel].samples++;
                }
            }
        }
        (void)tk_dly_tsk((INT)S3_CAL_INTERVAL_MS);
    }
    for (channel = 0U; channel < ADC_CH_COUNT; channel++)
    {
        if (!calibration_word(&statistics[channel], &candidates[channel]))
        {
            b_valid = false;
            APP_LOG((UB *)"[S3] cal ch%d FAILED: samples=%d, %d..%d\n",
                      (INT)channel, (INT)statistics[channel].samples,
                      (INT)statistics[channel].minimum,
                      (INT)statistics[channel].maximum);
        }
    }
    if (b_valid)
    {
        /* Commit all channels only after the whole transaction succeeds. */
        __atomic_store_n(&g_left_cal, candidates[ADC_CH_LINE_L],
                         __ATOMIC_RELEASE);
        __atomic_store_n(&g_right_cal, candidates[ADC_CH_LINE_R],
                         __ATOMIC_RELEASE);
        s3_barcode_calibrate(
            (uint16_t)(candidates[ADC_CH_BARCODE] & CAL_THRESHOLD_MASK),
            (uint16_t)(candidates[ADC_CH_BARCODE] >> CAL_BAND_SHIFT));
        control |= CONTROL_CALIBRATED;
    }
    control &= (uint8_t)~CONTROL_CALIBRATING;
    __atomic_store_n(&g_control, control, __ATOMIC_RELEASE);
    return (b_valid ? E_OK : E_IO);
}

void line_task(INT stacd, void * p_exinf)
{
    uint8_t control = 0U;
    uint32_t started_us = 0U;
    uint32_t elapsed_us = 0U;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)app_wait_period();
        started_us = adc_sampler_now_us();
        control = __atomic_load_n(&g_control, __ATOMIC_ACQUIRE);
        if (g_line_epoch != (control & CONTROL_EPOCH_MASK))
        {
            g_line_epoch = control & CONTROL_EPOCH_MASK;
            g_junction_count = 0U;
            g_last_error = 0;
            publish_reading(0, false, 0U, MSG_LINE_IDLE, 0U);
        }
        if ((CONTROL_ACTIVE | CONTROL_CALIBRATED)
            == (control & (CONTROL_ACTIVE | CONTROL_CALIBRATED
                           | CONTROL_CALIBRATING)))
        {
            track_once();
        }
        else
        {
            g_junction_count = 0U;
            g_last_error = 0;
            publish_reading(0, false, 0U,
                (0U != (control & CONTROL_CALIBRATING))
                    ? MSG_LINE_CAL : MSG_LINE_IDLE, 0U);
        }
        elapsed_us = adc_sampler_now_us() - started_us;
        if (elapsed_us > g_max_line_us)
        {
            __atomic_store_n(&g_max_line_us,
                (uint16_t)((elapsed_us > 65535U) ? 65535U : elapsed_us),
                __ATOMIC_RELAXED);
        }
    }
}

uint16_t s3_line_max_us(void)
{
    return (__atomic_load_n(&g_max_line_us, __ATOMIC_RELAXED));
}

bool s3_read_line_word(uint32_t * p_word)
{
    uint8_t status = __atomic_load_n(&g_status, __ATOMIC_ACQUIRE);
    bool b_valid = (NULL != p_word) && (0U != (status & STATUS_PUBLISHED));

    if (b_valid)
    {
        *p_word = __atomic_load_n(&g_line_word, __ATOMIC_ACQUIRE);
    }
    return (b_valid);
}

msg_line_state_t s3_get_line_state(void)
{
    return ((msg_line_state_t)
        (__atomic_load_n(&g_status, __ATOMIC_ACQUIRE) & STATUS_STATE_MASK));
}

bool s3_read_line_sensors(uint8_t * p_flags)
{
    uint32_t observation = __atomic_load_n(&g_sensor_word, __ATOMIC_ACQUIRE);
    uint32_t stamp_ms = observation >> SENSOR_STAMP_SHIFT;
    uint32_t age_ms = (app_now_ms() - stamp_ms) & SENSOR_STAMP_MASK;
    bool b_valid = (NULL != p_flags) && s3_barcode_active()
        && (0U != (observation & SENSOR_VALID_FLAG))
        && (age_ms <= (S3_SAMPLE_STALE_US / US_PER_MS));

    if (NULL != p_flags)
    {
        *p_flags = b_valid ? (uint8_t)(observation & SENSOR_FLAGS_MASK) : 0U;
    }
    return (b_valid);
}

void s3_set_active(bool b_on)
{
    uint8_t control = __atomic_load_n(&g_control, __ATOMIC_ACQUIRE);

    /* A stop/start between line ticks must also restart confirmation. */
    control = (uint8_t)(control + CONTROL_EPOCH_STEP);
    if (b_on)
    {
        control |= CONTROL_ACTIVE;
    }
    else
    {
        control &= (uint8_t)~CONTROL_ACTIVE;
    }
    __atomic_store_n(&g_control, control, __ATOMIC_RELEASE);
    /* Invalidate observations until the line task publishes in the new mode.
     * Only the line task owns the debounce and last-error fields. */
    __atomic_store_n(&g_sensor_word, 0U, __ATOMIC_RELEASE);
    s3_barcode_reset();
}

bool s3_barcode_active(void)
{
    uint8_t control = __atomic_load_n(&g_control, __ATOMIC_ACQUIRE);

    return ((CONTROL_ACTIVE | CONTROL_CALIBRATED)
        == (control & (CONTROL_ACTIVE | CONTROL_CALIBRATED
                       | CONTROL_CALIBRATING)));
}

void s3_get_thresholds(uint16_t * p_left, uint16_t * p_right)
{
    if ((NULL != p_left) && (NULL != p_right))
    {
        *p_left = (uint16_t)
            (__atomic_load_n(&g_left_cal, __ATOMIC_ACQUIRE)
             & CAL_THRESHOLD_MASK);
        *p_right = (uint16_t)
            (__atomic_load_n(&g_right_cal, __ATOMIC_ACQUIRE)
             & CAL_THRESHOLD_MASK);
    }
}
