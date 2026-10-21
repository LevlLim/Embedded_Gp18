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

#define PERCENT_SCALE                     (100)
#define LINE_FLAG_LEFT_BLACK              (0x01U)
#define LINE_FLAG_CENTER_BLACK            (0x02U)
#define LINE_FLAG_MISSING_ACTIVE          (0x04U)
#define LINE_FLAG_INITIALIZED             (0x08U)
#define LINE_MESSAGE_TIMESTAMP_OFFSET     (0U)
#define LINE_MESSAGE_LEFT_RAW_OFFSET      (32U)
#define LINE_MESSAGE_CENTER_RAW_OFFSET    (44U)
#define LINE_MESSAGE_LEFT_DARK_OFFSET     (56U)
#define LINE_MESSAGE_CENTER_DARK_OFFSET   (66U)
#define LINE_MESSAGE_ERROR_OFFSET         (76U)
#define LINE_MESSAGE_STATE_OFFSET         (87U)
#define LINE_MESSAGE_VALID_OFFSET         (90U)
#define LINE_MESSAGE_ADC_BITS             (12U)
#define LINE_MESSAGE_DARKNESS_BITS        (10U)
#define LINE_MESSAGE_ERROR_BITS           (11U)
#define LINE_MESSAGE_STATE_BITS           (3U)
#define LINE_MESSAGE_ERROR_BIAS           (1000)
#define LINE_MESSAGE_RESERVED_MASK        (0xF8U)

_Static_assert(sizeof(line_sensor_message_t)
               == LINE_SENSOR_MESSAGE_SIZE_BYTES,
               "Line message size changed");

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
static bool line_sensor_flag_is_set (
                line_sensor_t const * const p_sensor,
                uint8_t mask);
static void line_sensor_flag_write (
                line_sensor_t * const p_sensor,
                uint8_t mask,
                bool b_value);
static void line_message_write_bits (
                line_sensor_message_t * const p_message,
                uint16_t bit_offset,
                uint8_t bit_count,
                uint32_t value);
static uint32_t line_message_read_bits (
                    line_sensor_message_t const * const p_message,
                    uint16_t bit_offset,
                    uint8_t bit_count);

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
        p_sensor->state_flags            = LINE_FLAG_INITIALIZED;
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
        && line_sensor_flag_is_set(p_sensor, LINE_FLAG_INITIALIZED))
    {
        uint16_t const left_darkness = buddy3_config_normalize_adc(
                                               left_raw,
                                               &p_sensor->config.left_calibration);
        uint16_t const center_darkness = buddy3_config_normalize_adc(
                                                 center_raw,
                                                 &p_sensor->config.center_calibration);

        bool const b_left_black = line_sensor_update_black(
                                      line_sensor_flag_is_set(
                                          p_sensor,
                                          LINE_FLAG_LEFT_BLACK),
                                      left_darkness,
                                      p_sensor->config.line_threshold,
                                      p_sensor->config.line_hysteresis);
        bool const b_center_black = line_sensor_update_black(
                                        line_sensor_flag_is_set(
                                            p_sensor,
                                            LINE_FLAG_CENTER_BLACK),
                                        center_darkness,
                                        p_sensor->config.line_threshold,
                                        p_sensor->config.line_hysteresis);

        line_sensor_flag_write(
            p_sensor,
            LINE_FLAG_LEFT_BLACK,
            b_left_black);
        line_sensor_flag_write(
            p_sensor,
            LINE_FLAG_CENTER_BLACK,
            b_center_black);

        p_data->timestamp_ms    = timestamp_ms;
        p_data->left_raw        = left_raw;
        p_data->center_raw      = center_raw;
        p_data->left_darkness   = left_darkness;
        p_data->center_darkness = center_darkness;
        p_data->error           = p_sensor->last_error;
        p_data->state           = LINE_SENSOR_STATE_INVALID;
        p_data->b_valid         = false;

        if ((!b_left_black) && (!b_center_black))
        {
            uint32_t missing_time_ms = 0U;

            if (!line_sensor_flag_is_set(
                     p_sensor,
                     LINE_FLAG_MISSING_ACTIVE))
            {
                p_sensor->missing_start_ms = timestamp_ms;
                line_sensor_flag_write(
                    p_sensor,
                    LINE_FLAG_MISSING_ACTIVE,
                    true);
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

            line_sensor_flag_write(
                p_sensor,
                LINE_FLAG_MISSING_ACTIVE,
                false);
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

bool line_sensor_message_pack (
         line_sensor_data_t const * const p_data,
         line_sensor_message_t * const p_message)
{
    bool b_packed = false;

    if ((NULL != p_data)
        && (NULL != p_message)
        && (p_data->left_raw <= BUDDY3_ADC_MAX_COUNT)
        && (p_data->center_raw <= BUDDY3_ADC_MAX_COUNT)
        && (p_data->left_darkness <= BUDDY3_DARKNESS_SCALE)
        && (p_data->center_darkness <= BUDDY3_DARKNESS_SCALE)
        && (-((int16_t) BUDDY3_DARKNESS_SCALE) <= p_data->error)
        && (p_data->error <= (int16_t) BUDDY3_DARKNESS_SCALE)
        && ((uint32_t) p_data->state
            <= (uint32_t) LINE_SENSOR_STATE_LOST))
    {
        uint8_t byte_index = 0U;
        uint16_t const encoded_error =
            (uint16_t) ((int32_t) p_data->error
                        + LINE_MESSAGE_ERROR_BIAS);

        for (byte_index = 0U;
             byte_index < LINE_SENSOR_MESSAGE_SIZE_BYTES;
             byte_index++)
        {
            p_message->bytes[byte_index] = 0U;
        }

        line_message_write_bits(
            p_message,
            LINE_MESSAGE_TIMESTAMP_OFFSET,
            32U,
            p_data->timestamp_ms);
        line_message_write_bits(
            p_message,
            LINE_MESSAGE_LEFT_RAW_OFFSET,
            LINE_MESSAGE_ADC_BITS,
            p_data->left_raw);
        line_message_write_bits(
            p_message,
            LINE_MESSAGE_CENTER_RAW_OFFSET,
            LINE_MESSAGE_ADC_BITS,
            p_data->center_raw);
        line_message_write_bits(
            p_message,
            LINE_MESSAGE_LEFT_DARK_OFFSET,
            LINE_MESSAGE_DARKNESS_BITS,
            p_data->left_darkness);
        line_message_write_bits(
            p_message,
            LINE_MESSAGE_CENTER_DARK_OFFSET,
            LINE_MESSAGE_DARKNESS_BITS,
            p_data->center_darkness);
        line_message_write_bits(
            p_message,
            LINE_MESSAGE_ERROR_OFFSET,
            LINE_MESSAGE_ERROR_BITS,
            encoded_error);
        line_message_write_bits(
            p_message,
            LINE_MESSAGE_STATE_OFFSET,
            LINE_MESSAGE_STATE_BITS,
            (uint32_t) p_data->state);
        line_message_write_bits(
            p_message,
            LINE_MESSAGE_VALID_OFFSET,
            1U,
            p_data->b_valid ? 1U : 0U);
        b_packed = true;
    }

    return (b_packed);
}

bool line_sensor_message_unpack (
         line_sensor_message_t const * const p_message,
         line_sensor_data_t * const p_data)
{
    bool b_unpacked = false;

    if ((NULL != p_message) && (NULL != p_data))
    {
        uint16_t const left_darkness = (uint16_t) line_message_read_bits(
                                                    p_message,
                                                    LINE_MESSAGE_LEFT_DARK_OFFSET,
                                                    LINE_MESSAGE_DARKNESS_BITS);
        uint16_t const center_darkness = (uint16_t) line_message_read_bits(
                                                      p_message,
                                                      LINE_MESSAGE_CENTER_DARK_OFFSET,
                                                      LINE_MESSAGE_DARKNESS_BITS);
        uint16_t const encoded_error = (uint16_t) line_message_read_bits(
                                                    p_message,
                                                    LINE_MESSAGE_ERROR_OFFSET,
                                                    LINE_MESSAGE_ERROR_BITS);
        uint8_t const state = (uint8_t) line_message_read_bits(
                                           p_message,
                                           LINE_MESSAGE_STATE_OFFSET,
                                           LINE_MESSAGE_STATE_BITS);

        if ((left_darkness <= BUDDY3_DARKNESS_SCALE)
            && (center_darkness <= BUDDY3_DARKNESS_SCALE)
            && (encoded_error
                <= (uint16_t) (2 * LINE_MESSAGE_ERROR_BIAS))
            && (state <= (uint8_t) LINE_SENSOR_STATE_LOST)
            && (0U == (p_message->bytes[11U]
                       & LINE_MESSAGE_RESERVED_MASK)))
        {
            p_data->timestamp_ms = line_message_read_bits(
                                       p_message,
                                       LINE_MESSAGE_TIMESTAMP_OFFSET,
                                       32U);
            p_data->left_raw = (uint16_t) line_message_read_bits(
                                            p_message,
                                            LINE_MESSAGE_LEFT_RAW_OFFSET,
                                            LINE_MESSAGE_ADC_BITS);
            p_data->center_raw = (uint16_t) line_message_read_bits(
                                              p_message,
                                              LINE_MESSAGE_CENTER_RAW_OFFSET,
                                              LINE_MESSAGE_ADC_BITS);
            p_data->left_darkness = left_darkness;
            p_data->center_darkness = center_darkness;
            p_data->error = (int16_t) ((int32_t) encoded_error
                                      - LINE_MESSAGE_ERROR_BIAS);
            p_data->state = (line_sensor_state_t) state;
            p_data->b_valid = (0U != line_message_read_bits(
                                          p_message,
                                          LINE_MESSAGE_VALID_OFFSET,
                                          1U));
            b_unpacked = true;
        }
    }

    return (b_unpacked);
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

static bool line_sensor_flag_is_set (
                line_sensor_t const * const p_sensor,
                uint8_t mask)
{
    return (0U != (p_sensor->state_flags & mask));
}

static void line_sensor_flag_write (
                line_sensor_t * const p_sensor,
                uint8_t mask,
                bool b_value)
{
    if (b_value)
    {
        p_sensor->state_flags |= mask;
    }
    else
    {
        p_sensor->state_flags &= (uint8_t) (~mask);
    }
}

static void line_message_write_bits (
                line_sensor_message_t * const p_message,
                uint16_t bit_offset,
                uint8_t bit_count,
                uint32_t value)
{
    uint8_t bit_index = 0U;

    for (bit_index = 0U; bit_index < bit_count; bit_index++)
    {
        uint16_t const message_bit = bit_offset + bit_index;
        uint8_t const byte_index = (uint8_t) (message_bit / 8U);
        uint8_t const bit_mask = (uint8_t) (1U << (message_bit % 8U));

        if (0U != (value & ((uint32_t) 1U << bit_index)))
        {
            p_message->bytes[byte_index] |= bit_mask;
        }
    }
}

static uint32_t line_message_read_bits (
                    line_sensor_message_t const * const p_message,
                    uint16_t bit_offset,
                    uint8_t bit_count)
{
    uint32_t value = 0U;
    uint8_t bit_index = 0U;

    for (bit_index = 0U; bit_index < bit_count; bit_index++)
    {
        uint16_t const message_bit = bit_offset + bit_index;
        uint8_t const byte_index = (uint8_t) (message_bit / 8U);
        uint8_t const bit_mask = (uint8_t) (1U << (message_bit % 8U));

        if (0U != (p_message->bytes[byte_index] & bit_mask))
        {
            value |= ((uint32_t) 1U << bit_index);
        }
    }

    return (value);
}

/*** end of file ***/
