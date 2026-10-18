/**
 * @file buddy3_task.h
 *
 * @brief One-millisecond service and thread-safe outputs for Buddy 3.
 */

#ifndef BUDDY3_TASK_H
#define BUDDY3_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "adc_sampler.h"
#include "barcode_decoder.h"
#include "buddy3_config.h"
#include "line_sensor.h"

typedef struct
{
    adc_sampler_snapshot_t adc_snapshot;
    barcode_decoder_diag_t barcode_diagnostics;
    uint32_t               missed_tick_count;
    uint32_t               adc_failure_count;
    uint32_t               queue_overflow_count;
} buddy3_task_diag_t;

bool buddy3_task_init (buddy3_config_t const * const p_config);
void buddy3_task_process_1ms (uint32_t timestamp_ms);
bool buddy3_task_get_line (line_sensor_data_t * const p_data);
bool buddy3_task_get_barcode (barcode_decoder_event_t * const p_event);
bool buddy3_task_take_line_lost (void);
bool buddy3_task_get_diagnostics (buddy3_task_diag_t * const p_diagnostics);

#endif /* BUDDY3_TASK_H */

/*** end of file ***/
