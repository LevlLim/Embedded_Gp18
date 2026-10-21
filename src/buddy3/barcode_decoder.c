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
#define BARCODE_STATE_COUNT_MASK      (0x007FU)
#define BARCODE_STATE_COLOR_KNOWN     (0x0080U)
#define BARCODE_STATE_IS_BLACK        (0x0100U)
#define BARCODE_STATE_RECORDING       (0x0200U)
#define BARCODE_STATE_INITIALIZED     (0x0400U)
#define BARCODE_MESSAGE_TIMESTAMP     (0U)
#define BARCODE_MESSAGE_SEQUENCE      (32U)
#define BARCODE_MESSAGE_PAYLOAD       (64U)
#define BARCODE_MESSAGE_LENGTH        (112U)
#define BARCODE_MESSAGE_DIRECTION     (116U)
#define BARCODE_MESSAGE_VALID         (117U)
#define BARCODE_MESSAGE_SYMBOL_BITS   (6U)
#define BARCODE_MESSAGE_RESERVED_MASK (0xC0U)

_Static_assert(sizeof(barcode_message_t) == BARCODE_MESSAGE_SIZE_BYTES,
               "Barcode message size changed");
_Static_assert(BARCODE_DECODER_ELEMENT_BYTES
               == (((BARCODE_DECODER_MAX_ELEMENTS
                     * BARCODE_DECODER_ELEMENT_BITS) + 7U) / 8U),
               "Barcode element storage size is inconsistent");

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
static bool code39_symbol_to_code (
                uint8_t symbol,
                uint8_t * const p_code);
static uint8_t barcode_get_element_count (
                   barcode_decoder_t const * const p_decoder);
static void barcode_set_element_count (
                barcode_decoder_t * const p_decoder,
                uint8_t element_count);
static bool barcode_state_is_set (
                barcode_decoder_t const * const p_decoder,
                uint16_t mask);
static void barcode_state_write (
                barcode_decoder_t * const p_decoder,
                uint16_t mask,
                bool b_value);
static void barcode_set_width (
                barcode_decoder_t * const p_decoder,
                uint8_t element_index,
                uint16_t width_ms);
static void barcode_message_write_bits (
                barcode_message_t * const p_message,
                uint16_t bit_offset,
                uint8_t bit_count,
                uint32_t value);
static uint32_t barcode_message_read_bits (
                    barcode_message_t const * const p_message,
                    uint16_t bit_offset,
                    uint8_t bit_count);

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
        p_decoder->run_start_ms                 = 0U;
        p_decoder->recording_start_ms           = 0U;
        p_decoder->next_sequence_number         = 1U;
        p_decoder->state_bits                    = 0U;
        barcode_state_write(
            p_decoder,
            BARCODE_STATE_INITIALIZED,
            true);

        for (element_index = 0U;
             element_index < BARCODE_DECODER_ELEMENT_BYTES;
             element_index++)
        {
            p_decoder->element_width_bits[element_index] = 0U;
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
        && barcode_state_is_set(
               p_decoder,
               BARCODE_STATE_INITIALIZED))
    {
        uint16_t const darkness = buddy3_config_normalize_adc(
                                      raw_adc,
                                      &p_decoder->config.barcode_calibration);

        p_event->timestamp_ms    = timestamp_ms;
        p_event->sequence_number = 0U;
        p_event->payload_length  = 0U;
        p_event->direction       = BARCODE_DECODER_DIRECTION_FORWARD;
        p_event->b_valid         = false;

        if (!barcode_state_is_set(
                 p_decoder,
                 BARCODE_STATE_COLOR_KNOWN))
        {
            barcode_state_write(
                p_decoder,
                BARCODE_STATE_IS_BLACK,
                (p_decoder->config.barcode_threshold <= darkness));
            p_decoder->run_start_ms  = timestamp_ms;
            barcode_state_write(
                p_decoder,
                BARCODE_STATE_COLOR_KNOWN,
                true);
        }
        else
        {
            bool const b_new_is_black = barcode_update_black(
                                            barcode_state_is_set(
                                                p_decoder,
                                                BARCODE_STATE_IS_BLACK),
                                            darkness,
                                            p_decoder->config.barcode_threshold,
                                            p_decoder->config.barcode_hysteresis);

            if (b_new_is_black
                != barcode_state_is_set(
                       p_decoder,
                       BARCODE_STATE_IS_BLACK))
            {
                barcode_handle_transition(
                    p_decoder,
                    b_new_is_black,
                    timestamp_ms);
            }

            if (barcode_state_is_set(
                    p_decoder,
                    BARCODE_STATE_RECORDING))
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
                        barcode_state_is_set(
                            p_decoder,
                            BARCODE_STATE_IS_BLACK));
                }
                else if (p_decoder->config.barcode_max_element_ms
                         < current_run_ms)
                {
                    p_decoder->diagnostics.invalid_count++;
                    barcode_abort_capture(
                        p_decoder,
                        timestamp_ms,
                        barcode_state_is_set(
                            p_decoder,
                            BARCODE_STATE_IS_BLACK));
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
        && barcode_state_is_set(
               p_decoder,
               BARCODE_STATE_INITIALIZED))
    {
        *p_diagnostics = p_decoder->diagnostics;
        b_diagnostics_available = true;
    }

    return (b_diagnostics_available);
}

bool barcode_message_pack (
         barcode_decoder_event_t const * const p_event,
         barcode_message_t * const p_message)
{
    bool b_packed = false;

    if ((NULL != p_event)
        && (NULL != p_message)
        && (p_event->payload_length <= BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH)
        && ((uint32_t) p_event->direction
            <= (uint32_t) BARCODE_DECODER_DIRECTION_REVERSE))
    {
        uint8_t payload_index = 0U;
        uint8_t byte_index = 0U;
        bool b_symbols_valid = true;

        for (byte_index = 0U;
             byte_index < BARCODE_MESSAGE_SIZE_BYTES;
             byte_index++)
        {
            p_message->bytes[byte_index] = 0U;
        }

        barcode_message_write_bits(
            p_message,
            BARCODE_MESSAGE_TIMESTAMP,
            32U,
            p_event->timestamp_ms);
        barcode_message_write_bits(
            p_message,
            BARCODE_MESSAGE_SEQUENCE,
            32U,
            p_event->sequence_number);

        for (payload_index = 0U;
             b_symbols_valid && (payload_index < p_event->payload_length);
             payload_index++)
        {
            uint8_t code = 0U;

            b_symbols_valid = code39_symbol_to_code(
                                  p_event->payload[payload_index],
                                  &code);

            if (b_symbols_valid)
            {
                barcode_message_write_bits(
                    p_message,
                    (uint16_t) (BARCODE_MESSAGE_PAYLOAD
                                + (payload_index
                                   * BARCODE_MESSAGE_SYMBOL_BITS)),
                    BARCODE_MESSAGE_SYMBOL_BITS,
                    code);
            }
        }

        if (b_symbols_valid)
        {
            barcode_message_write_bits(
                p_message,
                BARCODE_MESSAGE_LENGTH,
                4U,
                p_event->payload_length);
            barcode_message_write_bits(
                p_message,
                BARCODE_MESSAGE_DIRECTION,
                1U,
                (uint32_t) p_event->direction);
            barcode_message_write_bits(
                p_message,
                BARCODE_MESSAGE_VALID,
                1U,
                p_event->b_valid ? 1U : 0U);
            b_packed = true;
        }
    }

    return (b_packed);
}

bool barcode_message_unpack (
         barcode_message_t const * const p_message,
         barcode_decoder_event_t * const p_event)
{
    bool b_unpacked = false;

    if ((NULL != p_message) && (NULL != p_event))
    {
        uint8_t const payload_length = (uint8_t) barcode_message_read_bits(
                                                    p_message,
                                                    BARCODE_MESSAGE_LENGTH,
                                                    4U);

        if ((payload_length <= BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH)
            && (0U == (p_message->bytes[14U]
                       & BARCODE_MESSAGE_RESERVED_MASK)))
        {
            uint8_t payload_index = 0U;
            bool b_symbols_valid = true;

            p_event->timestamp_ms = barcode_message_read_bits(
                                        p_message,
                                        BARCODE_MESSAGE_TIMESTAMP,
                                        32U);
            p_event->sequence_number = barcode_message_read_bits(
                                           p_message,
                                           BARCODE_MESSAGE_SEQUENCE,
                                           32U);
            p_event->payload_length = payload_length;
            p_event->direction = (barcode_decoder_direction_t)
                                 barcode_message_read_bits(
                                     p_message,
                                     BARCODE_MESSAGE_DIRECTION,
                                     1U);
            p_event->b_valid = (0U != barcode_message_read_bits(
                                           p_message,
                                           BARCODE_MESSAGE_VALID,
                                           1U));

            for (payload_index = 0U;
                 payload_index < BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH;
                 payload_index++)
            {
                p_event->payload[payload_index] = 0U;
            }

            for (payload_index = 0U;
                 b_symbols_valid && (payload_index < payload_length);
                 payload_index++)
            {
                uint8_t const code = (uint8_t) barcode_message_read_bits(
                    p_message,
                    (uint16_t) (BARCODE_MESSAGE_PAYLOAD
                                + (payload_index
                                   * BARCODE_MESSAGE_SYMBOL_BITS)),
                    BARCODE_MESSAGE_SYMBOL_BITS);

                if (CODE39_CHARACTER_COUNT <= code)
                {
                    b_symbols_valid = false;
                }
                else
                {
                    p_event->payload[payload_index] =
                        (uint8_t) g_code39_symbols[code];
                }
            }

            b_unpacked = b_symbols_valid;
        }
    }

    return (b_unpacked);
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

    if (!barcode_state_is_set(
             p_decoder,
             BARCODE_STATE_RECORDING))
    {
        bool const b_start_detected =
            ((!barcode_state_is_set(
                  p_decoder,
                  BARCODE_STATE_IS_BLACK))
             && b_new_is_black
             && (p_decoder->config.barcode_quiet_time_ms <= run_duration_ms));

        if (b_start_detected)
        {
            barcode_set_element_count(p_decoder, 0U);
            p_decoder->recording_start_ms = timestamp_ms;
            barcode_state_write(
                p_decoder,
                BARCODE_STATE_RECORDING,
                true);
        }
    }
    else if ((run_duration_ms < p_decoder->config.barcode_min_element_ms)
             || (p_decoder->config.barcode_max_element_ms < run_duration_ms))
    {
        p_decoder->diagnostics.invalid_count++;
        barcode_abort_capture(p_decoder, timestamp_ms, b_new_is_black);
    }
    else if (BARCODE_DECODER_MAX_ELEMENTS
             <= barcode_get_element_count(p_decoder))
    {
        p_decoder->diagnostics.overflow_count++;
        barcode_abort_capture(p_decoder, timestamp_ms, b_new_is_black);
    }
    else
    {
        uint8_t const element_count = barcode_get_element_count(p_decoder);

        barcode_set_width(
            p_decoder,
            element_count,
            (uint16_t) run_duration_ms);
        barcode_set_element_count(
            p_decoder,
            (uint8_t) (element_count + 1U));
    }

    barcode_state_write(
        p_decoder,
        BARCODE_STATE_IS_BLACK,
        b_new_is_black);
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
        ((!barcode_state_is_set(
              p_decoder,
              BARCODE_STATE_IS_BLACK))
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

        barcode_set_element_count(p_decoder, 0U);
        barcode_state_write(
            p_decoder,
            BARCODE_STATE_RECORDING,
            false);
    }

    return (b_event_ready);
}

static void barcode_abort_capture (
                barcode_decoder_t * const p_decoder,
                uint32_t timestamp_ms,
                bool b_current_is_black)
{
    barcode_set_element_count(p_decoder, 0U);
    p_decoder->recording_start_ms = 0U;
    p_decoder->run_start_ms       = timestamp_ms;
    barcode_state_write(
        p_decoder,
        BARCODE_STATE_IS_BLACK,
        b_current_is_black);
    barcode_state_write(
        p_decoder,
        BARCODE_STATE_RECORDING,
        false);
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
    uint16_t const adjusted_count =
        (uint16_t) barcode_get_element_count(p_decoder) + 1U;
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
        physical_index =
            ((uint16_t) barcode_get_element_count(p_decoder) - 1U)
            - logical_index;
    }

    {
        uint16_t value = 0U;
        uint8_t bit_index = 0U;
        uint16_t const bit_offset =
            physical_index * BARCODE_DECODER_ELEMENT_BITS;

        for (bit_index = 0U;
             bit_index < BARCODE_DECODER_ELEMENT_BITS;
             bit_index++)
        {
            uint16_t const element_bit = bit_offset + bit_index;
            uint8_t const byte_index = (uint8_t) (element_bit / 8U);
            uint8_t const bit_mask =
                (uint8_t) (1U << (element_bit % 8U));

            if (0U != (p_decoder->element_width_bits[byte_index]
                       & bit_mask))
            {
                value |= (uint16_t) (1U << bit_index);
            }
        }

        return (value);
    }
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

static bool code39_symbol_to_code (
                uint8_t symbol,
                uint8_t * const p_code)
{
    bool b_code_found = false;
    uint8_t code = 0U;

    if (NULL != p_code)
    {
        for (code = 0U;
             (!b_code_found) && (code < CODE39_CHARACTER_COUNT);
             code++)
        {
            if ((uint8_t) g_code39_symbols[code] == symbol)
            {
                *p_code = code;
                b_code_found = true;
            }
        }
    }

    return (b_code_found);
}

static uint8_t barcode_get_element_count (
                   barcode_decoder_t const * const p_decoder)
{
    return ((uint8_t) (p_decoder->state_bits
                       & BARCODE_STATE_COUNT_MASK));
}

static void barcode_set_element_count (
                barcode_decoder_t * const p_decoder,
                uint8_t element_count)
{
    p_decoder->state_bits =
        (uint16_t) ((p_decoder->state_bits
                    & (uint16_t) (~BARCODE_STATE_COUNT_MASK))
                   | ((uint16_t) element_count
                      & BARCODE_STATE_COUNT_MASK));
}

static bool barcode_state_is_set (
                barcode_decoder_t const * const p_decoder,
                uint16_t mask)
{
    return (0U != (p_decoder->state_bits & mask));
}

static void barcode_state_write (
                barcode_decoder_t * const p_decoder,
                uint16_t mask,
                bool b_value)
{
    if (b_value)
    {
        p_decoder->state_bits |= mask;
    }
    else
    {
        p_decoder->state_bits &= (uint16_t) (~mask);
    }
}

static void barcode_set_width (
                barcode_decoder_t * const p_decoder,
                uint8_t element_index,
                uint16_t width_ms)
{
    uint8_t bit_index = 0U;
    uint16_t const bit_offset =
        (uint16_t) element_index * BARCODE_DECODER_ELEMENT_BITS;

    for (bit_index = 0U;
         bit_index < BARCODE_DECODER_ELEMENT_BITS;
         bit_index++)
    {
        uint16_t const element_bit = bit_offset + bit_index;
        uint8_t const byte_index = (uint8_t) (element_bit / 8U);
        uint8_t const bit_mask =
            (uint8_t) (1U << (element_bit % 8U));

        p_decoder->element_width_bits[byte_index] &=
            (uint8_t) (~bit_mask);

        if (0U != (width_ms & (uint16_t) (1U << bit_index)))
        {
            p_decoder->element_width_bits[byte_index] |= bit_mask;
        }
    }
}

static void barcode_message_write_bits (
                barcode_message_t * const p_message,
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

static uint32_t barcode_message_read_bits (
                    barcode_message_t const * const p_message,
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
