/**
 * @file buddy3_main.c
 *
 * @brief Standalone bring-up entry point for the Buddy 3 subsystem.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "buddy3/barcode_decoder.h"
#include "buddy3/buddy3_config.h"
#include "buddy3/buddy3_task.h"
#include "buddy3/line_sensor.h"
#include "buddy3_main.h"

#define STARTUP_DELAY_MS       (1500U)
#define BUDDY3_TICK_MS         (1U)
#define DEBUG_REPORT_PERIOD_MS (100U)

static void buddy3_main_halt (void);

int main (void)
{
    buddy3_config_t config = { 0 };
    absolute_time_t next_tick = { 0 };
    uint32_t last_report_ms = 0U;

    stdio_init_all();
    sleep_ms(STARTUP_DELAY_MS);
    buddy3_config_get_default(&config);

    if (!buddy3_task_init(&config))
    {
#ifndef NDEBUG
        printf("Buddy 3 initialization failed.\n");
#endif
        buddy3_main_halt();
    }

#ifndef NDEBUG
    printf("Buddy 3 bring-up firmware started.\n");
    printf("WARNING: Replace the provisional ADC calibration values.\n");
#endif

    next_tick = delayed_by_ms(get_absolute_time(), BUDDY3_TICK_MS);

    for (;;)
    {
        uint32_t timestamp_ms = 0U;
        barcode_decoder_event_t barcode_event = { 0 };
        bool b_barcode_available = false;
        bool b_line_lost = false;

        sleep_until(next_tick);
        next_tick = delayed_by_ms(next_tick, BUDDY3_TICK_MS);
        timestamp_ms = to_ms_since_boot(get_absolute_time());
        buddy3_task_process_1ms(timestamp_ms);

        b_barcode_available = buddy3_task_get_barcode(&barcode_event);

        while (b_barcode_available)
        {
#ifndef NDEBUG
            uint8_t payload_index = 0U;

            printf("Barcode %lu: ",
                   (unsigned long) barcode_event.sequence_number);

            for (payload_index = 0U;
                 payload_index < barcode_event.payload_length;
                 payload_index++)
            {
                printf("%c", (char) barcode_event.payload[payload_index]);
            }

            printf(" (%s)\n",
                   (BARCODE_DECODER_DIRECTION_FORWARD
                    == barcode_event.direction)
                   ? "forward"
                   : "reverse");
#endif
            b_barcode_available = buddy3_task_get_barcode(&barcode_event);
        }

        b_line_lost = buddy3_task_take_line_lost();

#ifndef NDEBUG
        if (b_line_lost)
        {
            printf("LINE_LOST event.\n");
        }

        if (DEBUG_REPORT_PERIOD_MS <= (timestamp_ms - last_report_ms))
        {
            line_sensor_data_t line_data = { 0 };
            buddy3_task_diag_t diagnostics = { 0 };

            last_report_ms = timestamp_ms;
            (void) buddy3_task_get_diagnostics(&diagnostics);

            if (buddy3_task_get_line(&line_data))
            {
                printf("IR L=%u C=%u B=%u, error=%d, state=%u, valid=%u\n",
                       diagnostics.adc_snapshot.left_raw,
                       diagnostics.adc_snapshot.center_raw,
                       diagnostics.adc_snapshot.barcode_raw,
                       line_data.error,
                       (unsigned int) line_data.state,
                       line_data.b_valid ? 1U : 0U);
            }
        }
#else
        (void) b_line_lost;
        (void) last_report_ms;
#endif
    }
}

static void buddy3_main_halt (void)
{
    for (;;)
    {
        sleep_ms(STARTUP_DELAY_MS);
    }
}

/*** end of file ***/
