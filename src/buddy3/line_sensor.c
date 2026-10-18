/**
 * @file line_sensor.c
 *
 * @brief Converts two calibrated IR readings into line-following data.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "buddy3_config.h"
#include "line_sensor.h"

#define PERCENT_SCALE    (100)

static bool line_sensor_update_black (
                bool b_was_black,
                uint16_t darkness,
                uint16_t threshold,
                uint16_t hysteresis);
static int16_t line_sensor_calculate_error (
                   line_sensor_t const * const p_sensor,
                   uint16_t left_darkness,
                   uint16_t center_darkness);
static int16_t line_sensor_clamp_error (int32_t error);

bool line_sensor_init (
         line_sensor_t * const p_sensor,
         buddy3_config_t const * const p_config)
{
    bool b_initialized = false;

    if ((NULL != p_sensor) && buddy3_config_is_valid(p_config))
    {
        p_sensor->config                 = *p_config;
        p_sensor->missing_start_ms       = 0U;
        p_sensor->last_error             = 0;
        p_sensor->junction_sample_count  = 0U;
        p_sensor->b_left_black           = false;
        p_sensor->b_center_black         = false;
        p_sensor->b_missing_active       = false;
        p_sensor->b_initialized          = true;
        b_initialized                    = true;
    }

    return (b_initialized);
}

bool line_sensor_process (
         line_sensor_t * const p_sensor,
         uint16_t left_raw,
         uint16_t center_raw,
         uint32_t timestamp_ms,
         line_sensor_data_t * const p_data)
{
    bool b_processed = false;

    if ((NULL != p_sensor)
        && (NULL != p_data)
        && p_sensor->b_initialized)
    {
        uint16_t const left_darkness = buddy3_config_normalize_adc(
                                               left_raw,
                                               &p_sensor->config.left_calibration);
        uint16_t const center_darkness = buddy3_config_normalize_adc(
                                                 center_raw,
                                                 &p_sensor->config.center_calibration);

        p_sensor->b_left_black = line_sensor_update_black(
                                     p_sensor->b_left_black,
                                     left_darkness,
                                     p_sensor->config.line_threshold,
                                     p_sensor->config.line_hysteresis);
        p_sensor->b_center_black = line_sensor_update_black(
                                       p_sensor->b_center_black,
                                       center_darkness,
                                       p_sensor->config.line_threshold,
                                       p_sensor->config.line_hysteresis);

        p_data->timestamp_ms    = timestamp_ms;
        p_data->left_raw        = left_raw;
        p_data->center_raw      = center_raw;
        p_data->left_darkness   = left_darkness;
        p_data->center_darkness = center_darkness;
        p_data->error           = p_sensor->last_error;
        p_data->state           = LINE_SENSOR_STATE_INVALID;
        p_data->b_valid         = false;

        if ((!p_sensor->b_left_black) && (!p_sensor->b_center_black))
        {
            uint32_t missing_time_ms = 0U;

            if (!p_sensor->b_missing_active)
            {
                p_sensor->missing_start_ms = timestamp_ms;
                p_sensor->b_missing_active = true;
            }

            missing_time_ms = timestamp_ms - p_sensor->missing_start_ms;
            p_sensor->junction_sample_count = 0U;

            if (p_sensor->config.line_lost_time_ms <= missing_time_ms)
            {
                p_data->state   = LINE_SENSOR_STATE_LOST;
                p_data->b_valid = false;
            }
            else
            {
                if (0 == p_sensor->last_error)
                {
                    p_sensor->last_error = p_sensor->config.line_search_error;
                }

                p_data->error   = p_sensor->last_error;
                p_data->state   = LINE_SENSOR_STATE_SEARCHING;
                p_data->b_valid = true;
            }
        }
        else
        {
            bool const b_junction_level =
                ((p_sensor->config.junction_threshold <= left_darkness)
                 && (p_sensor->config.junction_threshold <= center_darkness));

            p_sensor->b_missing_active = false;
            p_sensor->missing_start_ms = 0U;
            p_sensor->last_error = line_sensor_calculate_error(
                                       p_sensor,
                                       left_darkness,
                                       center_darkness);

            if (b_junction_level)
            {
                if (UINT16_MAX > p_sensor->junction_sample_count)
                {
                    p_sensor->junction_sample_count++;
                }
            }
            else
            {
                p_sensor->junction_sample_count = 0U;
            }

            if (p_sensor->config.junction_confirm_samples
                <= p_sensor->junction_sample_count)
            {
                p_data->state = LINE_SENSOR_STATE_JUNCTION;
            }
            else
            {
                p_data->state = LINE_SENSOR_STATE_TRACKING;
            }

            p_data->error   = p_sensor->last_error;
            p_data->b_valid = true;
        }

        b_processed = true;
    }

    return (b_processed);
}

static bool line_sensor_update_black (
                bool b_was_black,
                uint16_t darkness,
                uint16_t threshold,
                uint16_t hysteresis)
{
    bool b_is_black = false;

    if (b_was_black)
    {
        b_is_black = ((threshold - hysteresis) < darkness);
    }
    else
    {
        b_is_black = ((threshold + hysteresis) <= darkness);
    }

    return (b_is_black);
}

static int16_t line_sensor_calculate_error (
                   line_sensor_t const * const p_sensor,
                   uint16_t left_darkness,
                   uint16_t center_darkness)
{
    int32_t const right_evidence =
        (int32_t) BUDDY3_DARKNESS_SCALE - (int32_t) center_darkness;
    int32_t const left_evidence =
        ((int32_t) left_darkness * (int32_t) p_sensor->config.left_weight_x100)
        / PERCENT_SCALE;
    int32_t const error = right_evidence - left_evidence;

    return (line_sensor_clamp_error(error));
}

static int16_t line_sensor_clamp_error (int32_t error)
{
    int32_t clamped_error = error;

    if (-((int32_t) BUDDY3_DARKNESS_SCALE) > clamped_error)
    {
        clamped_error = -((int32_t) BUDDY3_DARKNESS_SCALE);
    }
    else if ((int32_t) BUDDY3_DARKNESS_SCALE < clamped_error)
    {
        clamped_error = (int32_t) BUDDY3_DARKNESS_SCALE;
    }
    else
    {
        /* The calculated error is already in range. */
    }

    return ((int16_t) clamped_error);
}

/*** end of file ***/
