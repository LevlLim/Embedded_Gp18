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
    buddy3_config_t config;
    uint32_t        missing_start_ms;
    int16_t         last_error;
    uint16_t        junction_sample_count;
    bool            b_left_black;
    bool            b_center_black;
    bool            b_missing_active;
    bool            b_initialized;
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

#endif /* LINE_SENSOR_H */

/*** end of file ***/
