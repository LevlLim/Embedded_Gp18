/**
 * @file buddy3_task.c
 *
 * @brief One-millisecond service and thread-safe outputs for Buddy 3.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pico/critical_section.h"

#include "adc_sampler.h"
#include "barcode_decoder.h"
#include "buddy3_config.h"
#include "buddy3_task.h"
#include "line_sensor.h"

#define BARCODE_QUEUE_CAPACITY    (4U)

static buddy3_config_t          g_config = { 0 };
static line_sensor_t            g_line_sensor = { 0 };
static barcode_decoder_t        g_barcode_decoder = { 0 };
static line_sensor_data_t       g_line_snapshot = { 0 };
static barcode_decoder_event_t  g_barcode_queue[BARCODE_QUEUE_CAPACITY] = { 0 };
static buddy3_task_diag_t       g_task_diagnostics = { 0 };
static critical_section_t       g_shared_lock = { 0 };
static line_sensor_state_t      g_previous_line_state = LINE_SENSOR_STATE_INVALID;
static uint32_t                 g_last_tick_ms = 0U;
static uint16_t                 g_line_divider_count = 0U;
static uint8_t                  g_barcode_queue_head = 0U;
static uint8_t                  g_barcode_queue_tail = 0U;
static uint8_t                  g_barcode_queue_count = 0U;
static uint32_t                 g_missed_tick_count = 0U;
static uint32_t                 g_adc_failure_count = 0U;
static uint32_t                 g_queue_overflow_count = 0U;
static bool                     g_b_line_available = false;
static bool                     g_b_line_lost_pending = false;
static bool                     g_b_task_initialized = false;

static void buddy3_queue_barcode (
                barcode_decoder_event_t const * const p_event);
static void buddy3_publish_diagnostics (void);

bool buddy3_task_init (buddy3_config_t const * const p_config)
{
    bool b_initialized = false;

    if (buddy3_config_is_valid(p_config))
    {
        critical_section_init(&g_shared_lock);
        adc_sampler_init();

        b_initialized = line_sensor_init(&g_line_sensor, p_config);

        if (b_initialized)
        {
            b_initialized = barcode_decoder_init(&g_barcode_decoder, p_config);
        }

        if (b_initialized)
        {
            g_config = *p_config;
            g_line_snapshot.timestamp_ms    = 0U;
            g_line_snapshot.left_raw        = 0U;
            g_line_snapshot.center_raw      = 0U;
            g_line_snapshot.left_darkness   = 0U;
            g_line_snapshot.center_darkness = 0U;
            g_line_snapshot.error           = 0;
            g_line_snapshot.state           = LINE_SENSOR_STATE_INVALID;
            g_line_snapshot.b_valid         = false;
            g_task_diagnostics.missed_tick_count = 0U;
            g_task_diagnostics.adc_failure_count = 0U;
            g_task_diagnostics.queue_overflow_count = 0U;
            g_previous_line_state = LINE_SENSOR_STATE_INVALID;
            g_last_tick_ms = 0U;
            g_line_divider_count = 0U;
            g_barcode_queue_head = 0U;
            g_barcode_queue_tail = 0U;
            g_barcode_queue_count = 0U;
            g_missed_tick_count = 0U;
            g_adc_failure_count = 0U;
            g_queue_overflow_count = 0U;
            g_b_line_available = false;
            g_b_line_lost_pending = false;
            g_b_task_initialized = true;
        }
    }

    return (b_initialized);
}

void buddy3_task_process_1ms (uint32_t timestamp_ms)
{
    if (g_b_task_initialized)
    {
        uint16_t barcode_raw = 0U;
        bool const b_barcode_read = adc_sampler_read_barcode(&barcode_raw);

        if (0U != g_last_tick_ms)
        {
            uint32_t const tick_difference = timestamp_ms - g_last_tick_ms;

            if (1U < tick_difference)
            {
                g_missed_tick_count += tick_difference - 1U;
            }
        }

        g_last_tick_ms = timestamp_ms;

        if (b_barcode_read)
        {
            barcode_decoder_event_t event = { 0 };
            bool const b_event_ready = barcode_decoder_process_sample(
                                           &g_barcode_decoder,
                                           barcode_raw,
                                           timestamp_ms,
                                           &event);

            if (b_event_ready)
            {
                buddy3_queue_barcode(&event);
            }
        }
        else
        {
            g_adc_failure_count++;
        }

        g_line_divider_count++;

        if (g_config.line_sample_divider <= g_line_divider_count)
        {
            uint16_t left_raw = 0U;
            uint16_t center_raw = 0U;
            bool const b_line_read = adc_sampler_read_line(
                                         &left_raw,
                                         &center_raw);

            g_line_divider_count = 0U;

            if (b_line_read)
            {
                line_sensor_data_t line_data = { 0 };
                bool const b_line_processed = line_sensor_process(
                                                  &g_line_sensor,
                                                  left_raw,
                                                  center_raw,
                                                  timestamp_ms,
                                                  &line_data);

                if (b_line_processed)
                {
                    critical_section_enter_blocking(&g_shared_lock);

                    g_line_snapshot = line_data;
                    g_b_line_available = true;

                    if ((LINE_SENSOR_STATE_LOST == line_data.state)
                        && (LINE_SENSOR_STATE_LOST != g_previous_line_state))
                    {
                        g_b_line_lost_pending = true;
                    }

                    g_previous_line_state = line_data.state;
                    critical_section_exit(&g_shared_lock);
                }
            }
            else
            {
                g_adc_failure_count++;
            }

            buddy3_publish_diagnostics();
        }
    }
}

bool buddy3_task_get_line (line_sensor_data_t * const p_data)
{
    bool b_data_available = false;

    if ((NULL != p_data) && g_b_task_initialized)
    {
        critical_section_enter_blocking(&g_shared_lock);

        if (g_b_line_available)
        {
            *p_data = g_line_snapshot;
            b_data_available = true;
        }

        critical_section_exit(&g_shared_lock);
    }

    return (b_data_available);
}

bool buddy3_task_get_barcode (barcode_decoder_event_t * const p_event)
{
    bool b_event_available = false;

    if ((NULL != p_event) && g_b_task_initialized)
    {
        critical_section_enter_blocking(&g_shared_lock);

        if (0U < g_barcode_queue_count)
        {
            *p_event = g_barcode_queue[g_barcode_queue_tail];
            g_barcode_queue_tail =
                (uint8_t) ((g_barcode_queue_tail + 1U)
                           % BARCODE_QUEUE_CAPACITY);
            g_barcode_queue_count--;
            b_event_available = true;
        }

        critical_section_exit(&g_shared_lock);
    }

    return (b_event_available);
}

bool buddy3_task_take_line_lost (void)
{
    bool b_line_lost = false;

    if (g_b_task_initialized)
    {
        critical_section_enter_blocking(&g_shared_lock);

        b_line_lost = g_b_line_lost_pending;
        g_b_line_lost_pending = false;

        critical_section_exit(&g_shared_lock);
    }

    return (b_line_lost);
}

bool buddy3_task_get_diagnostics (buddy3_task_diag_t * const p_diagnostics)
{
    bool b_diagnostics_available = false;

    if ((NULL != p_diagnostics) && g_b_task_initialized)
    {
        critical_section_enter_blocking(&g_shared_lock);

        *p_diagnostics = g_task_diagnostics;
        critical_section_exit(&g_shared_lock);

        b_diagnostics_available = true;
    }

    return (b_diagnostics_available);
}

static void buddy3_queue_barcode (
                barcode_decoder_event_t const * const p_event)
{
    if (NULL != p_event)
    {
        critical_section_enter_blocking(&g_shared_lock);

        if (BARCODE_QUEUE_CAPACITY <= g_barcode_queue_count)
        {
            g_queue_overflow_count++;
        }
        else
        {
            g_barcode_queue[g_barcode_queue_head] = *p_event;
            g_barcode_queue_head =
                (uint8_t) ((g_barcode_queue_head + 1U)
                           % BARCODE_QUEUE_CAPACITY);
            g_barcode_queue_count++;
        }

        critical_section_exit(&g_shared_lock);
    }
}

static void buddy3_publish_diagnostics (void)
{
    buddy3_task_diag_t diagnostics = { 0 };

    diagnostics.missed_tick_count = g_missed_tick_count;
    diagnostics.adc_failure_count = g_adc_failure_count;
    diagnostics.queue_overflow_count = g_queue_overflow_count;
    (void) adc_sampler_get_snapshot(&diagnostics.adc_snapshot);
    (void) barcode_decoder_get_diagnostics(
               &g_barcode_decoder,
               &diagnostics.barcode_diagnostics);

    {
        critical_section_enter_blocking(&g_shared_lock);

        g_task_diagnostics = diagnostics;
        critical_section_exit(&g_shared_lock);
    }
}

/*** end of file ***/
