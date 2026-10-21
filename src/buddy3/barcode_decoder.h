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
#define BARCODE_DECODER_ELEMENT_BITS   (9U)
#define BARCODE_DECODER_ELEMENT_BYTES  (112U)
#define BARCODE_MESSAGE_SIZE_BYTES     (15U)

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
    uint8_t bytes[BARCODE_MESSAGE_SIZE_BYTES];
} barcode_message_t;

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
    uint8_t               element_width_bits[BARCODE_DECODER_ELEMENT_BYTES];
    uint32_t              run_start_ms;
    uint32_t              recording_start_ms;
    uint32_t              next_sequence_number;
    uint16_t              state_bits;
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
bool barcode_message_pack (
         barcode_decoder_event_t const * const p_event,
         barcode_message_t * const p_message);
bool barcode_message_unpack (
         barcode_message_t const * const p_message,
         barcode_decoder_event_t * const p_event);

#endif /* BARCODE_DECODER_H */

/*** end of file ***/
