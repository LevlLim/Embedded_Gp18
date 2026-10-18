/**
 * @file barcode_decoder.c
 *
 * @brief Incremental Code 39 decoder for the analogue barcode IR sensor.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "barcode_decoder.h"
#include "buddy3_config.h"

#define CODE39_ELEMENT_COUNT          (9U)
#define CODE39_GROUP_SIZE             (10U)
#define CODE39_MIN_CHARACTER_COUNT    (3U)
#define CODE39_CHARACTER_COUNT        (43U)
#define CODE39_ASTERISK_PATTERN       (0x094U)
#define CODE39_WIDE_ELEMENT_COUNT     (3U)
#define CODE39_FIRST_WIDE_INDEX       (6U)
#define PERCENT_SCALE                 (100U)

static char const g_code39_symbols[CODE39_CHARACTER_COUNT + 1U] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static uint16_t const g_code39_patterns[CODE39_CHARACTER_COUNT] =
{
    0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U,
    0x124U, 0x064U, 0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U,
    0x00DU, 0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U,
    0x1C0U, 0x091U, 0x190U, 0x0D0U, 0x085U, 0x184U, 0x0C4U, 0x0A8U,
    0x0A2U, 0x08AU, 0x02AU
};

static bool barcode_update_black (
                bool b_was_black,
                uint16_t darkness,
                uint16_t threshold,
                uint16_t hysteresis);
static void barcode_handle_transition (
                barcode_decoder_t * const p_decoder,
                bool b_new_is_black,
                uint32_t timestamp_ms);
static bool barcode_try_finish (
                barcode_decoder_t * const p_decoder,
                uint32_t timestamp_ms,
                barcode_decoder_event_t * const p_event);
static void barcode_abort_capture (
                barcode_decoder_t * const p_decoder,
                uint32_t timestamp_ms,
                bool b_current_is_black);
static bool barcode_decode_capture (
                barcode_decoder_t const * const p_decoder,
                barcode_decoder_event_t * const p_event);
static bool barcode_decode_orientation (
                barcode_decoder_t const * const p_decoder,
                bool b_reverse,
                barcode_decoder_event_t * const p_event);
static bool barcode_decode_character (
                barcode_decoder_t const * const p_decoder,
                uint16_t character_index,
                bool b_reverse,
                uint8_t * const p_symbol,
                uint16_t * const p_minimum_wide);
static bool barcode_classify_pattern (
                barcode_decoder_t const * const p_decoder,
                uint16_t const element_widths[CODE39_ELEMENT_COUNT],
                uint16_t * const p_pattern,
                uint16_t * const p_minimum_wide);
static uint16_t barcode_get_width (
                    barcode_decoder_t const * const p_decoder,
                    uint16_t logical_index,
                    bool b_reverse);
static bool code39_pattern_to_symbol (
                uint16_t pattern,
                uint8_t * const p_symbol);

bool barcode_decoder_init (
         barcode_decoder_t * const p_decoder,
         buddy3_config_t const * const p_config)
{
    bool b_initialized = false;

    if ((NULL != p_decoder) && buddy3_config_is_valid(p_config))
    {
        uint16_t element_index = 0U;

        p_decoder->config                       = *p_config;
        p_decoder->diagnostics.valid_count      = 0U;
        p_decoder->diagnostics.invalid_count    = 0U;
        p_decoder->diagnostics.timeout_count    = 0U;
        p_decoder->diagnostics.overflow_count   = 0U;
        p_decoder->element_count                = 0U;
        p_decoder->run_start_ms                 = 0U;
        p_decoder->recording_start_ms           = 0U;
        p_decoder->next_sequence_number         = 1U;
        p_decoder->b_color_known                = false;
        p_decoder->b_is_black                   = false;
        p_decoder->b_recording                  = false;
        p_decoder->b_initialized                = true;

        for (element_index = 0U;
             element_index < BARCODE_DECODER_MAX_ELEMENTS;
             element_index++)
        {
            p_decoder->element_width_ms[element_index] = 0U;
        }

        b_initialized = true;
    }

    return (b_initialized);
}

bool barcode_decoder_process_sample (
         barcode_decoder_t * const p_decoder,
         uint16_t raw_adc,
         uint32_t timestamp_ms,
         barcode_decoder_event_t * const p_event)
{
    bool b_event_ready = false;

    if ((NULL != p_decoder)
        && (NULL != p_event)
        && p_decoder->b_initialized)
    {
        uint16_t const darkness = buddy3_config_normalize_adc(
                                      raw_adc,
                                      &p_decoder->config.barcode_calibration);

        p_event->timestamp_ms    = timestamp_ms;
        p_event->sequence_number = 0U;
        p_event->payload_length  = 0U;
        p_event->direction       = BARCODE_DECODER_DIRECTION_FORWARD;
        p_event->b_valid         = false;

        if (!p_decoder->b_color_known)
        {
            p_decoder->b_is_black =
                (p_decoder->config.barcode_threshold <= darkness);
            p_decoder->run_start_ms  = timestamp_ms;
            p_decoder->b_color_known = true;
        }
        else
        {
            bool const b_new_is_black = barcode_update_black(
                                            p_decoder->b_is_black,
                                            darkness,
                                            p_decoder->config.barcode_threshold,
                                            p_decoder->config.barcode_hysteresis);

            if (b_new_is_black != p_decoder->b_is_black)
            {
                barcode_handle_transition(
                    p_decoder,
                    b_new_is_black,
                    timestamp_ms);
            }

            if (p_decoder->b_recording)
            {
                uint32_t const recording_time_ms =
                    timestamp_ms - p_decoder->recording_start_ms;
                uint32_t const current_run_ms =
                    timestamp_ms - p_decoder->run_start_ms;

                if (p_decoder->config.barcode_timeout_ms < recording_time_ms)
                {
                    p_decoder->diagnostics.timeout_count++;
                    barcode_abort_capture(
                        p_decoder,
                        timestamp_ms,
                        p_decoder->b_is_black);
                }
                else if (p_decoder->config.barcode_max_element_ms
                         < current_run_ms)
                {
                    p_decoder->diagnostics.invalid_count++;
                    barcode_abort_capture(
                        p_decoder,
                        timestamp_ms,
                        p_decoder->b_is_black);
                }
                else
                {
                    b_event_ready = barcode_try_finish(
                                        p_decoder,
                                        timestamp_ms,
                                        p_event);
                }
            }
        }
    }

    return (b_event_ready);
}

bool barcode_decoder_get_diagnostics (
         barcode_decoder_t const * const p_decoder,
         barcode_decoder_diag_t * const p_diagnostics)
{
    bool b_diagnostics_available = false;

    if ((NULL != p_decoder)
        && (NULL != p_diagnostics)
        && p_decoder->b_initialized)
    {
        *p_diagnostics = p_decoder->diagnostics;
        b_diagnostics_available = true;
    }

    return (b_diagnostics_available);
}

static bool barcode_update_black (
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

static void barcode_handle_transition (
                barcode_decoder_t * const p_decoder,
                bool b_new_is_black,
                uint32_t timestamp_ms)
{
    uint32_t const run_duration_ms = timestamp_ms - p_decoder->run_start_ms;

    if (!p_decoder->b_recording)
    {
        bool const b_start_detected =
            ((!p_decoder->b_is_black)
             && b_new_is_black
             && (p_decoder->config.barcode_quiet_time_ms <= run_duration_ms));

        if (b_start_detected)
        {
            p_decoder->element_count      = 0U;
            p_decoder->recording_start_ms = timestamp_ms;
            p_decoder->b_recording        = true;
        }
    }
    else if ((run_duration_ms < p_decoder->config.barcode_min_element_ms)
             || (p_decoder->config.barcode_max_element_ms < run_duration_ms))
    {
        p_decoder->diagnostics.invalid_count++;
        barcode_abort_capture(p_decoder, timestamp_ms, b_new_is_black);
    }
    else if (BARCODE_DECODER_MAX_ELEMENTS <= p_decoder->element_count)
    {
        p_decoder->diagnostics.overflow_count++;
        barcode_abort_capture(p_decoder, timestamp_ms, b_new_is_black);
    }
    else
    {
        p_decoder->element_width_ms[p_decoder->element_count] =
            (uint16_t) run_duration_ms;
        p_decoder->element_count++;
    }

    p_decoder->b_is_black  = b_new_is_black;
    p_decoder->run_start_ms = timestamp_ms;
}

static bool barcode_try_finish (
                barcode_decoder_t * const p_decoder,
                uint32_t timestamp_ms,
                barcode_decoder_event_t * const p_event)
{
    bool b_event_ready = false;
    uint32_t const current_run_ms = timestamp_ms - p_decoder->run_start_ms;
    bool const b_end_detected =
        ((!p_decoder->b_is_black)
         && (p_decoder->config.barcode_quiet_time_ms <= current_run_ms));

    if (b_end_detected)
    {
        b_event_ready = barcode_decode_capture(p_decoder, p_event);

        if (b_event_ready)
        {
            p_event->timestamp_ms = timestamp_ms;
            p_event->sequence_number = p_decoder->next_sequence_number;
            p_event->b_valid = true;
            p_decoder->next_sequence_number++;
            p_decoder->diagnostics.valid_count++;
        }
        else
        {
            p_decoder->diagnostics.invalid_count++;
        }

        p_decoder->element_count = 0U;
        p_decoder->b_recording   = false;
    }

    return (b_event_ready);
}

static void barcode_abort_capture (
                barcode_decoder_t * const p_decoder,
                uint32_t timestamp_ms,
                bool b_current_is_black)
{
    p_decoder->element_count      = 0U;
    p_decoder->recording_start_ms = 0U;
    p_decoder->run_start_ms       = timestamp_ms;
    p_decoder->b_is_black         = b_current_is_black;
    p_decoder->b_recording        = false;
}

static bool barcode_decode_capture (
                barcode_decoder_t const * const p_decoder,
                barcode_decoder_event_t * const p_event)
{
    bool b_decoded = barcode_decode_orientation(p_decoder, false, p_event);

    if (!b_decoded)
    {
        b_decoded = barcode_decode_orientation(p_decoder, true, p_event);
    }

    return (b_decoded);
}

static bool barcode_decode_orientation (
                barcode_decoder_t const * const p_decoder,
                bool b_reverse,
                barcode_decoder_event_t * const p_event)
{
    uint16_t const adjusted_count = p_decoder->element_count + 1U;
    uint16_t const character_count = adjusted_count / CODE39_GROUP_SIZE;
    bool b_decoded =
        ((0U == (adjusted_count % CODE39_GROUP_SIZE))
         && (CODE39_MIN_CHARACTER_COUNT <= character_count)
         && (character_count
             <= (BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH + 2U)));
    uint8_t decoded_symbols[BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH + 2U] = { 0U };
    uint16_t character_index = 0U;

    for (character_index = 0U;
         b_decoded && (character_index < character_count);
         character_index++)
    {
        uint16_t minimum_wide = 0U;
        uint8_t symbol = 0U;

        b_decoded = barcode_decode_character(
                        p_decoder,
                        character_index,
                        b_reverse,
                        &symbol,
                        &minimum_wide);

        if (b_decoded && ((character_index + 1U) < character_count))
        {
            uint16_t const gap_index =
                (character_index * CODE39_GROUP_SIZE) + CODE39_ELEMENT_COUNT;
            uint16_t const gap_width = barcode_get_width(
                                           p_decoder,
                                           gap_index,
                                           b_reverse);

            b_decoded = (gap_width < minimum_wide);
        }

        if (b_decoded)
        {
            decoded_symbols[character_index] = symbol;
        }
    }

    if (b_decoded)
    {
        uint16_t const last_index = character_count - 1U;
        uint16_t payload_index = 0U;

        b_decoded = (('*' == decoded_symbols[0U])
                     && ('*' == decoded_symbols[last_index]));

        if (b_decoded)
        {
            p_event->payload_length = (uint8_t) (character_count - 2U);
            p_event->direction = b_reverse
                                 ? BARCODE_DECODER_DIRECTION_REVERSE
                                 : BARCODE_DECODER_DIRECTION_FORWARD;

            for (payload_index = 0U;
                 payload_index < p_event->payload_length;
                 payload_index++)
            {
                p_event->payload[payload_index] =
                    decoded_symbols[payload_index + 1U];
            }
        }
    }

    return (b_decoded);
}

static bool barcode_decode_character (
                barcode_decoder_t const * const p_decoder,
                uint16_t character_index,
                bool b_reverse,
                uint8_t * const p_symbol,
                uint16_t * const p_minimum_wide)
{
    uint16_t element_widths[CODE39_ELEMENT_COUNT] = { 0U };
    uint16_t element_index = 0U;
    uint16_t pattern = 0U;
    bool b_decoded = ((NULL != p_symbol) && (NULL != p_minimum_wide));

    for (element_index = 0U;
         b_decoded && (element_index < CODE39_ELEMENT_COUNT);
         element_index++)
    {
        uint16_t const logical_index =
            (character_index * CODE39_GROUP_SIZE) + element_index;

        element_widths[element_index] = barcode_get_width(
                                            p_decoder,
                                            logical_index,
                                            b_reverse);
    }

    if (b_decoded)
    {
        b_decoded = barcode_classify_pattern(
                        p_decoder,
                        element_widths,
                        &pattern,
                        p_minimum_wide);
    }

    if (b_decoded)
    {
        b_decoded = code39_pattern_to_symbol(pattern, p_symbol);
    }

    return (b_decoded);
}

static bool barcode_classify_pattern (
                barcode_decoder_t const * const p_decoder,
                uint16_t const element_widths[CODE39_ELEMENT_COUNT],
                uint16_t * const p_pattern,
                uint16_t * const p_minimum_wide)
{
    uint16_t sorted_widths[CODE39_ELEMENT_COUNT] = { 0U };
    uint16_t source_index = 0U;
    uint16_t pattern = 0U;
    uint16_t wide_count = 0U;
    bool b_classified =
        ((NULL != p_pattern)
         && (NULL != p_minimum_wide)
         && (NULL != element_widths));

    for (source_index = 0U;
         b_classified && (source_index < CODE39_ELEMENT_COUNT);
         source_index++)
    {
        sorted_widths[source_index] = element_widths[source_index];
    }

    for (source_index = 1U;
         b_classified && (source_index < CODE39_ELEMENT_COUNT);
         source_index++)
    {
        uint16_t const width_to_insert = sorted_widths[source_index];
        uint16_t insert_index = source_index;

        while ((0U < insert_index)
               && (width_to_insert < sorted_widths[insert_index - 1U]))
        {
            sorted_widths[insert_index] = sorted_widths[insert_index - 1U];
            insert_index--;
        }

        sorted_widths[insert_index] = width_to_insert;
    }

    if (b_classified)
    {
        uint32_t const minimum_wide_scaled =
            (uint32_t) sorted_widths[CODE39_FIRST_WIDE_INDEX] * PERCENT_SCALE;
        uint32_t const maximum_narrow_scaled =
            (uint32_t) sorted_widths[CODE39_FIRST_WIDE_INDEX - 1U]
            * p_decoder->config.barcode_min_wide_ratio_x100;

        b_classified = (maximum_narrow_scaled <= minimum_wide_scaled);
    }

    for (source_index = 0U;
         b_classified && (source_index < CODE39_ELEMENT_COUNT);
         source_index++)
    {
        if (sorted_widths[CODE39_FIRST_WIDE_INDEX]
            <= element_widths[source_index])
        {
            pattern |= (uint16_t) (1U
                       << ((CODE39_ELEMENT_COUNT - 1U) - source_index));
            wide_count++;
        }
    }

    if (b_classified)
    {
        b_classified = (CODE39_WIDE_ELEMENT_COUNT == wide_count);
    }

    if (b_classified)
    {
        *p_pattern = pattern;
        *p_minimum_wide = sorted_widths[CODE39_FIRST_WIDE_INDEX];
    }

    return (b_classified);
}

static uint16_t barcode_get_width (
                    barcode_decoder_t const * const p_decoder,
                    uint16_t logical_index,
                    bool b_reverse)
{
    uint16_t physical_index = logical_index;

    if (b_reverse)
    {
        physical_index = (p_decoder->element_count - 1U) - logical_index;
    }

    return (p_decoder->element_width_ms[physical_index]);
}

static bool code39_pattern_to_symbol (
                uint16_t pattern,
                uint8_t * const p_symbol)
{
    bool b_symbol_found = false;
    uint16_t symbol_index = 0U;

    if (NULL != p_symbol)
    {
        if (CODE39_ASTERISK_PATTERN == pattern)
        {
            *p_symbol = (uint8_t) '*';
            b_symbol_found = true;
        }
        else
        {
            for (symbol_index = 0U;
                 (!b_symbol_found) && (symbol_index < CODE39_CHARACTER_COUNT);
                 symbol_index++)
            {
                if (g_code39_patterns[symbol_index] == pattern)
                {
                    *p_symbol = (uint8_t) g_code39_symbols[symbol_index];
                    b_symbol_found = true;
                }
            }
        }
    }

    return (b_symbol_found);
}

/*** end of file ***/
