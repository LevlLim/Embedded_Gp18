/**
 * @file line_sensor.h
 *
 * @brief Converts two calibrated IR readings into line-following data.
 */

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "buddy3_config.h"

#define LINE_SENSOR_MESSAGE_SIZE_BYTES   (12U)

typedef enum
{
    LINE_SENSOR_STATE_INVALID = 0,
    LINE_SENSOR_STATE_TRACKING,
    LINE_SENSOR_STATE_SEARCHING,
    LINE_SENSOR_STATE_JUNCTION,
    LINE_SENSOR_STATE_LOST
} line_sensor_state_t;

typedef struct
{
    uint32_t            timestamp_ms;
    uint16_t            left_raw;
    uint16_t            center_raw;
    uint16_t            left_darkness;
    uint16_t            center_darkness;
    int16_t             error;
    line_sensor_state_t state;
    bool                b_valid;
} line_sensor_data_t;

typedef struct
{
    uint8_t bytes[LINE_SENSOR_MESSAGE_SIZE_BYTES];
} line_sensor_message_t;

typedef struct
{
    buddy3_config_t config;
    uint32_t        missing_start_ms;
    int16_t         last_error;
    uint16_t        junction_sample_count;
    uint8_t         state_flags;
} line_sensor_t;

bool line_sensor_init (
         line_sensor_t * const p_sensor,
         buddy3_config_t const * const p_config);
bool line_sensor_process (
         line_sensor_t * const p_sensor,
         uint16_t left_raw,
         uint16_t center_raw,
         uint32_t timestamp_ms,
         line_sensor_data_t * const p_data);
bool line_sensor_message_pack (
         line_sensor_data_t const * const p_data,
         line_sensor_message_t * const p_message);
bool line_sensor_message_unpack (
         line_sensor_message_t const * const p_message,
         line_sensor_data_t * const p_data);

#endif /* LINE_SENSOR_H */

/*** end of file ***/
