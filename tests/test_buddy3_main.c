/**
 * @file test_buddy3_main.c
 *
 * @brief Host tests for line processing and Code 39 barcode decoding.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "adc_sampler.h"
#include "barcode_decoder.h"
#include "buddy3_config.h"
#include "line_sensor.h"
#include "test_buddy3_main.h"

#define TEST_CODE39_A_PATTERN      (0x109U)
#define TEST_CODE39_Z_PATTERN      (0x0D0U)
#define TEST_CODE39_STAR_PATTERN   (0x094U)
#define TEST_CODE39_ELEMENTS       (9U)
#define TEST_BARCODE_ELEMENTS      (29U)
#define TEST_NARROW_TIME_MS        (3U)
#define TEST_WIDE_TIME_MS          (9U)
#define TEST_QUIET_TIME_MS         (25U)
#define TEST_CHARACTER_COUNT       (3U)

static uint32_t g_failure_count = 0U;

static void test_check (bool b_condition, char const * const p_message);
static void test_line_sensor (void);
static void test_line_message (void);
static void test_barcode_symbol (uint8_t symbol, uint16_t pattern);
static void test_barcode_message (void);
static void test_feed_barcode (
                barcode_decoder_t * const p_decoder,
                buddy3_config_t const * const p_config,
                uint8_t symbol,
                uint16_t pattern,
                bool b_reverse,
                barcode_decoder_event_t * const p_event,
                uint32_t * const p_event_count);
static void test_build_widths (
                uint16_t pattern,
                uint16_t widths[TEST_BARCODE_ELEMENTS]);
static void test_feed_level (
                barcode_decoder_t * const p_decoder,
                uint16_t raw_adc,
                uint16_t duration_ms,
                uint32_t * const p_timestamp_ms,
                barcode_decoder_event_t * const p_event,
                uint32_t * const p_event_count);

int main (void)
{
    test_line_sensor();
    test_line_message();
    test_barcode_symbol((uint8_t) 'A', TEST_CODE39_A_PATTERN);
    test_barcode_symbol((uint8_t) 'Z', TEST_CODE39_Z_PATTERN);
    test_barcode_message();

    if (0U == g_failure_count)
    {
        printf("All Buddy 3 host tests passed.\n");
    }
    else
    {
        printf("Buddy 3 host tests failed: %lu.\n",
               (unsigned long) g_failure_count);
    }

    return ((int) g_failure_count);
}

static void test_line_message (void)
{
    line_sensor_data_t source =
    {
        UINT32_MAX,
        BUDDY3_ADC_MAX_COUNT,
        0U,
        BUDDY3_DARKNESS_SCALE,
        0U,
        -((int16_t) BUDDY3_DARKNESS_SCALE),
        LINE_SENSOR_STATE_LOST,
        true
    };
    line_sensor_data_t recovered = { 0 };
    line_sensor_message_t message = { { 0U } };

    test_check(LINE_SENSOR_MESSAGE_SIZE_BYTES == sizeof(message),
               "The packed line message should occupy 12 bytes.");
    test_check(ADC_SAMPLER_SNAPSHOT_SIZE_BYTES
               == sizeof(adc_sampler_snapshot_t),
               "Three 12-bit ADC samples should occupy 5 bytes.");
    test_check(line_sensor_message_pack(&source, &message),
               "Boundary line data should pack successfully.");
    test_check(line_sensor_message_unpack(&message, &recovered),
               "A valid line message should unpack successfully.");
    test_check((source.timestamp_ms == recovered.timestamp_ms)
               && (source.left_raw == recovered.left_raw)
               && (source.center_raw == recovered.center_raw)
               && (source.left_darkness == recovered.left_darkness)
               && (source.center_darkness == recovered.center_darkness)
               && (source.error == recovered.error)
               && (source.state == recovered.state)
               && (source.b_valid == recovered.b_valid),
               "Line data should survive a packed round trip.");

    source.error = (int16_t) BUDDY3_DARKNESS_SCALE;
    test_check(line_sensor_message_pack(&source, &message)
               && line_sensor_message_unpack(&message, &recovered)
               && (source.error == recovered.error),
               "The positive line-error boundary should round trip.");

    source.left_raw = BUDDY3_ADC_MAX_COUNT + 1U;
    test_check(!line_sensor_message_pack(&source, &message),
               "An out-of-range ADC value should be rejected.");

    source.left_raw = BUDDY3_ADC_MAX_COUNT;
    source.state = (line_sensor_state_t) -1;
    test_check(!line_sensor_message_pack(&source, &message),
               "An invalid line state should be rejected.");

    source.state = LINE_SENSOR_STATE_LOST;
    test_check(line_sensor_message_pack(&source, &message),
               "Valid line data should repack after a rejected value.");
    message.bytes[11U] |= 0x80U;
    test_check(!line_sensor_message_unpack(&message, &recovered),
               "A line message with reserved bits set should be rejected.");
}

static void test_check (bool b_condition, char const * const p_message)
{
    if (!b_condition)
    {
        g_failure_count++;
        printf("FAIL: %s\n", p_message);
    }
}

static void test_line_sensor (void)
{
    buddy3_config_t config = { 0 };
    line_sensor_t sensor = { 0 };
    line_sensor_data_t data = { 0 };
    uint16_t sample_index = 0U;

    buddy3_config_get_default(&config);
    test_check(line_sensor_init(&sensor, &config),
               "Line sensor initialization should succeed.");

    (void) line_sensor_process(
               &sensor,
               config.left_calibration.white_adc,
               config.center_calibration.black_adc,
               5U,
               &data);
    test_check(LINE_SENSOR_STATE_TRACKING == data.state,
               "A centered line should be tracked.");
    test_check(0 == data.error,
               "A centered line should have zero error.");
    test_check(data.b_valid,
               "A centered line should be valid.");

    (void) line_sensor_process(
               &sensor,
               config.left_calibration.black_adc,
               config.center_calibration.white_adc,
               10U,
               &data);
    test_check(0 > data.error,
               "A line under the left sensor should have negative error.");

    for (sample_index = 0U;
         sample_index < config.junction_confirm_samples;
         sample_index++)
    {
        (void) line_sensor_process(
                   &sensor,
                   config.left_calibration.black_adc,
                   config.center_calibration.black_adc,
                   (uint32_t) (15U + (sample_index * 5U)),
                   &data);
    }

    test_check(LINE_SENSOR_STATE_JUNCTION == data.state,
               "Repeated black readings should identify a junction candidate.");

    (void) line_sensor_process(
               &sensor,
               config.left_calibration.white_adc,
               config.center_calibration.white_adc,
               100U,
               &data);
    (void) line_sensor_process(
               &sensor,
               config.left_calibration.white_adc,
               config.center_calibration.white_adc,
               100U + config.line_lost_time_ms,
               &data);
    test_check(LINE_SENSOR_STATE_LOST == data.state,
               "A prolonged all-white reading should report line loss.");
    test_check(!data.b_valid,
               "Line data should be invalid after confirmed line loss.");
}

static void test_barcode_symbol (uint8_t symbol, uint16_t pattern)
{
    buddy3_config_t config = { 0 };
    barcode_decoder_t decoder = { 0 };
    barcode_decoder_event_t event = { 0 };
    uint32_t event_count = 0U;

    buddy3_config_get_default(&config);
    test_check(barcode_decoder_init(&decoder, &config),
               "Barcode decoder initialization should succeed.");
    test_feed_barcode(
        &decoder,
        &config,
        symbol,
        pattern,
        false,
        &event,
        &event_count);
    test_check(1U == event_count,
               "A forward barcode should produce one event.");
    test_check((1U == event.payload_length) && (symbol == event.payload[0U]),
               "A forward barcode should decode the expected symbol.");
    test_check(BARCODE_DECODER_DIRECTION_FORWARD == event.direction,
               "A forward barcode should report forward direction.");

    event_count = 0U;
    test_check(barcode_decoder_init(&decoder, &config),
               "Barcode decoder reinitialization should succeed.");
    test_feed_barcode(
        &decoder,
        &config,
        symbol,
        pattern,
        true,
        &event,
        &event_count);
    test_check(1U == event_count,
               "A reversed barcode should produce one event.");
    test_check((1U == event.payload_length) && (symbol == event.payload[0U]),
               "A reversed barcode should decode the expected symbol.");
    test_check(BARCODE_DECODER_DIRECTION_REVERSE == event.direction,
               "A reversed barcode should report reverse direction.");
}

static void test_barcode_message (void)
{
    uint8_t const payload[BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH] =
    {
        (uint8_t) '0', (uint8_t) 'A', (uint8_t) 'Z', (uint8_t) '-',
        (uint8_t) '$', (uint8_t) '/', (uint8_t) '+', (uint8_t) '%'
    };
    barcode_decoder_event_t source = { 0 };
    barcode_decoder_event_t recovered = { 0 };
    barcode_message_t message = { { 0U } };
    uint8_t payload_index = 0U;

    source.timestamp_ms = UINT32_MAX;
    source.sequence_number = UINT32_MAX;
    source.payload_length = BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH;
    source.direction = BARCODE_DECODER_DIRECTION_REVERSE;
    source.b_valid = true;

    for (payload_index = 0U;
         payload_index < BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH;
         payload_index++)
    {
        source.payload[payload_index] = payload[payload_index];
    }

    test_check(BARCODE_MESSAGE_SIZE_BYTES == sizeof(message),
               "The packed barcode message should occupy 15 bytes.");
    test_check(BARCODE_DECODER_ELEMENT_BYTES == 112U,
               "Ninety-nine 9-bit barcode widths should occupy 112 bytes.");
    test_check(barcode_message_pack(&source, &message),
               "A maximum-length Code 39 payload should pack.");
    test_check(barcode_message_unpack(&message, &recovered),
               "A valid barcode message should unpack.");
    test_check((source.timestamp_ms == recovered.timestamp_ms)
               && (source.sequence_number == recovered.sequence_number)
               && (source.payload_length == recovered.payload_length)
               && (source.direction == recovered.direction)
               && (source.b_valid == recovered.b_valid),
               "Barcode metadata should survive a packed round trip.");

    for (payload_index = 0U;
         payload_index < BUDDY3_BARCODE_MAX_PAYLOAD_LENGTH;
         payload_index++)
    {
        test_check(source.payload[payload_index]
                   == recovered.payload[payload_index],
                   "Barcode payload symbols should survive packing.");
    }

    source.payload[0U] = (uint8_t) '*';
    test_check(!barcode_message_pack(&source, &message),
               "A non-payload Code 39 symbol should be rejected.");

    source.payload[0U] = (uint8_t) '0';
    source.direction = (barcode_decoder_direction_t) -1;
    test_check(!barcode_message_pack(&source, &message),
               "An invalid barcode direction should be rejected.");

    source.direction = BARCODE_DECODER_DIRECTION_REVERSE;
    test_check(barcode_message_pack(&source, &message),
               "A valid barcode should repack after a rejected symbol.");
    message.bytes[14U] |= 0x80U;
    test_check(!barcode_message_unpack(&message, &recovered),
               "A barcode message with reserved bits set should be rejected.");
}

static void test_feed_barcode (
                barcode_decoder_t * const p_decoder,
                buddy3_config_t const * const p_config,
                uint8_t symbol,
                uint16_t pattern,
                bool b_reverse,
                barcode_decoder_event_t * const p_event,
                uint32_t * const p_event_count)
{
    uint16_t widths[TEST_BARCODE_ELEMENTS] = { 0U };
    uint32_t timestamp_ms = 0U;
    uint16_t element_index = 0U;

    (void) symbol;
    test_build_widths(pattern, widths);
    test_feed_level(
        p_decoder,
        p_config->barcode_calibration.white_adc,
        TEST_QUIET_TIME_MS,
        &timestamp_ms,
        p_event,
        p_event_count);

    for (element_index = 0U;
         element_index < TEST_BARCODE_ELEMENTS;
         element_index++)
    {
        uint16_t physical_index = element_index;
        bool const b_is_black = (0U == (element_index % 2U));
        uint16_t raw_adc = p_config->barcode_calibration.white_adc;

        if (b_reverse)
        {
            physical_index = (TEST_BARCODE_ELEMENTS - 1U) - element_index;
        }

        if (b_is_black)
        {
            raw_adc = p_config->barcode_calibration.black_adc;
        }

        test_feed_level(
            p_decoder,
            raw_adc,
            widths[physical_index],
            &timestamp_ms,
            p_event,
            p_event_count);
    }

    test_feed_level(
        p_decoder,
        p_config->barcode_calibration.white_adc,
        TEST_QUIET_TIME_MS,
        &timestamp_ms,
        p_event,
        p_event_count);
}

static void test_build_widths (
                uint16_t pattern,
                uint16_t widths[TEST_BARCODE_ELEMENTS])
{
    uint16_t const patterns[TEST_CHARACTER_COUNT] =
    {
        TEST_CODE39_STAR_PATTERN,
        pattern,
        TEST_CODE39_STAR_PATTERN
    };
    uint16_t character_index = 0U;
    uint16_t output_index = 0U;

    for (character_index = 0U;
         character_index < TEST_CHARACTER_COUNT;
         character_index++)
    {
        uint16_t element_index = 0U;

        for (element_index = 0U;
             element_index < TEST_CODE39_ELEMENTS;
             element_index++)
        {
            uint16_t const mask = (uint16_t) (1U
                                  << ((TEST_CODE39_ELEMENTS - 1U)
                                      - element_index));

            widths[output_index] =
                (0U != (patterns[character_index] & mask))
                ? TEST_WIDE_TIME_MS
                : TEST_NARROW_TIME_MS;
            output_index++;
        }

        if ((character_index + 1U) < TEST_CHARACTER_COUNT)
        {
            widths[output_index] = TEST_NARROW_TIME_MS;
            output_index++;
        }
    }
}

static void test_feed_level (
                barcode_decoder_t * const p_decoder,
                uint16_t raw_adc,
                uint16_t duration_ms,
                uint32_t * const p_timestamp_ms,
                barcode_decoder_event_t * const p_event,
                uint32_t * const p_event_count)
{
    uint16_t sample_index = 0U;

    for (sample_index = 0U;
         sample_index < duration_ms;
         sample_index++)
    {
        barcode_decoder_event_t sample_event = { 0 };
        bool const b_event_ready = barcode_decoder_process_sample(
                                       p_decoder,
                                       raw_adc,
                                       *p_timestamp_ms,
                                       &sample_event);

        if (b_event_ready)
        {
            *p_event = sample_event;
            (*p_event_count)++;
        }

        (*p_timestamp_ms)++;
    }
}

/*** end of file ***/
