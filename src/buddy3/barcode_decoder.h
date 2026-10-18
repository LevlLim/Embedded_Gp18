/**
 * @file barcode_decoder.h
 *
 * @brief Incremental Code 39 decoder for the analogue barcode IR sensor.
 */

#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#include "buddy3_config.h"

#define BARCODE_DECODER_MAX_ELEMENTS   (99U)

typedef enum
{
    BARCODE_DECODER_DIRECTION_FORWARD = 0,
    BARCODE_DECODER_DIRECTION_REVERSE
} barcode_decoder_direction_t;

typedef struct
{
    uint32_t                    timestamp_ms;
    uint32_t                    sequence_number;
    uint8_t                     payload[BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH];
    uint8_t                     payload_length;
    barcode_decoder_direction_t direction;
    bool                        b_valid;
} barcode_decoder_event_t;

typedef struct
{
    uint32_t valid_count;
    uint32_t invalid_count;
    uint32_t timeout_count;
    uint32_t overflow_count;
} barcode_decoder_diag_t;

typedef struct
{
    buddy3_config_t       config;
    barcode_decoder_diag_t diagnostics;
    uint16_t              element_width_ms[BARCODE_DECODER_MAX_ELEMENTS];
    uint16_t              element_count;
    uint32_t              run_start_ms;
    uint32_t              recording_start_ms;
    uint32_t              next_sequence_number;
    bool                  b_color_known;
    bool                  b_is_black;
    bool                  b_recording;
    bool                  b_initialized;
} barcode_decoder_t;

bool barcode_decoder_init (
         barcode_decoder_t * const p_decoder,
         buddy3_config_t const * const p_config);
bool barcode_decoder_process_sample (
         barcode_decoder_t * const p_decoder,
         uint16_t raw_adc,
         uint32_t timestamp_ms,
         barcode_decoder_event_t * const p_event);
bool barcode_decoder_get_diagnostics (
         barcode_decoder_t const * const p_decoder,
         barcode_decoder_diag_t * const p_diagnostics);

#endif /* BARCODE_DECODER_H */

/*** end of file ***/
