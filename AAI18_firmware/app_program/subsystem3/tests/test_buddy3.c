/**
 * @file test_buddy3.c
 * @brief Regression checks against production Buddy 3 functions.
 */
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <time.h>
#include "../line_sensor.c"
#include "../barcode_mapping.h"
static msg_barcode_cmd_t test_mapping_direction(uint8_t letter);
#define barcode_mapping_direction test_mapping_direction
#include "../barcode_decoder.c"
#undef barcode_mapping_direction
#include "code39_reference.h"

#define TEST_CHECK(condition) test_check((condition), __LINE__)
#define TEST_NARROW_US (4000U)
#define TEST_WIDE_US (10000U)
#define TEST_QUIET_US (80000U)
#define TEST_SAMPLE_US (1000U)

static unsigned int g_tests = 0U;
static unsigned int g_failed = 0U;
static uint32_t g_now_us = 1000000U;
static uint32_t g_ms_bias = 0U;
static uint32_t g_events = 0U;
static uint32_t g_delay_count = 0U;
static uint32_t g_wait_us = PERIOD_LINE_MS * 1000U;
static uint8_t g_mode = 0U;
static uint8_t g_queue[4][2];
static uint8_t g_queue_count = 0U;
static ER g_send_error = E_OK;
static ID g_create_result = 1;
static uint8_t g_create_calls = 0U;
static bool g_b_invalid_mapping = false;
static adc_sample_t g_mock_batch[S3_ADC_BATCH_LEN];
static uint8_t g_mock_batch_count = 0U;
static bool g_b_frozen = false;
static bool g_b_stop_task = false;
static uint8_t g_task_cycles = 0U;
static jmp_buf g_task_exit;
static uint16_t g_values[3] = {100U, 3000U, 100U};

static msg_barcode_cmd_t test_mapping_direction(uint8_t letter)
{
    return (g_b_invalid_mapping ? MSG_BC_INVALID
                               : barcode_mapping_direction(letter));
}

static void test_check(bool b_condition, int line)
{
    g_tests++;
    if (!b_condition)
    {
        g_failed++;
        printf("FAIL line %d\n", line);
    }
}

uint32_t app_now_ms(void)
{
    return ((g_now_us / 1000U) + g_ms_bias);
}
uint32_t adc_sampler_now_us(void)
{
    return (g_now_us);
}
void tm_printf(UB * p_format, ...)
{
    (void)p_format;
}
void adc_sampler_selftest(void)
{
}
uint32_t adc_sampler_overruns(void)
{
    return (0U);
}
void s5_post_event(uint32_t events)
{
    g_events |= events;
}
ER tk_dly_tsk(INT delay)
{
    g_now_us += (uint32_t)delay * 1000U;
    g_delay_count++;
    return (E_OK);
}
uint32_t app_wait_period(void)
{
    if (g_b_stop_task && (0U == g_task_cycles))
    {
        longjmp(g_task_exit, 1);
    }
    g_task_cycles--;
    g_now_us += g_wait_us;
    return (0U);
}
ID tk_cre_mbf(T_CMBF * p_config)
{
    g_create_calls++;
    TEST_CHECK(2 == p_config->maxmsz);
    TEST_CHECK((INT)APP_MBF_BUFSZ(4U, 2U) == p_config->bufsz);
    return (g_create_result);
}
ER tk_snd_mbf(ID handle, void * p_bytes, INT length, TMO timeout)
{
    ER error = g_send_error;

    (void)handle;
    (void)timeout;
    TEST_CHECK(2 == length);
    if (E_OK == error)
    {
        if (g_queue_count < 4U)
        {
            memcpy(g_queue[g_queue_count], p_bytes, 2U);
            g_queue_count++;
        }
        else
        {
            error = E_TMOUT;
        }
    }
    return (error);
}
INT tk_rcv_mbf(ID handle, void * p_bytes, TMO timeout)
{
    INT length = E_TMOUT;

    (void)handle;
    (void)timeout;
    if (0U != g_queue_count)
    {
        memcpy(p_bytes, g_queue[0], 2U);
        g_queue_count--;
        memmove(g_queue, g_queue + 1, (size_t)g_queue_count * 2U);
        length = 2;
    }
    return (length);
}
bool adc_sampler_read(uint8_t channel, adc_sample_t * p_sample)
{
    bool b_valid = !((1U == g_mode && ADC_CH_LINE_L == channel)
                     || (2U == g_mode && ADC_CH_BARCODE == channel)
                     || (6U == g_mode && ADC_CH_LINE_R == channel)
                     || (3U == g_mode));

    if (b_valid)
    {
        p_sample->timestamp_us = g_b_frozen ? 1000U : g_now_us;
        p_sample->value = g_values[channel];
        if (4U == g_mode)
        {
            p_sample->value = (0U == (g_delay_count & 1U)) ? 100U : 3000U;
        }
        else if ((5U == g_mode) && (ADC_CH_LINE_R == channel))
        {
            p_sample->value = (0U == (g_delay_count & 1U)) ? 100U : 3000U;
        }
        else if (7U == g_mode)
        {
            p_sample->value = (0U == (g_delay_count & 1U)) ? 100U : 350U;
        }
        else if (8U == g_mode)
        {
            p_sample->value = (0U == (g_delay_count & 1U)) ? 100U : 4095U;
        }
        else
        {
            /* Fixed test values. */
        }
    }
    return (b_valid);
}
uint8_t adc_sampler_since(uint32_t after, adc_sample_t * p_out,
                         uint8_t capacity)
{
    uint8_t count = 0U;
    uint8_t index = 0U;

    for (index = 0U; (index < g_mock_batch_count) && (count < capacity); index++)
    {
        if ((g_mock_batch[index].timestamp_us - after) < 0x80000000U
            && g_mock_batch[index].timestamp_us != after)
        {
            p_out[count++] = g_mock_batch[index];
        }
    }
    return (count);
}

static void make_frame(uint16_t pattern, bool b_reverse,
                       uint16_t narrow, uint16_t wide,
                       uint16_t * p_widths)
{
    uint16_t patterns[3] = {CODE39_STAR, pattern, CODE39_STAR};
    uint16_t scratch[S3_FRAME_ELEMENTS] = {0U};
    uint8_t character = 0U;
    uint8_t element = 0U;
    uint8_t count = 0U;
    uint8_t index = 0U;

    for (character = 0U; character < 3U; character++)
    {
        for (element = 0U; element < S3_CHAR_ELEMENTS; element++)
        {
            scratch[count] = (0U != (patterns[character]
                & (uint16_t)(1U << (8U - element)))) ? wide : narrow;
            count++;
        }
        if (character < 2U)
        {
            scratch[count] = narrow;
            count++;
        }
    }
    for (index = 0U; index < S3_FRAME_ELEMENTS; index++)
    {
        p_widths[index] = scratch[b_reverse ? 28U - index : index];
    }
}

static void feed_colour(bool b_dark, uint32_t duration)
{
    adc_sample_t sample = {0U, 0U};
    uint32_t elapsed = 0U;

    sample.value = b_dark ? 3000U : 100U;
    for (elapsed = 0U; elapsed < duration; elapsed += TEST_SAMPLE_US)
    {
        sample.timestamp_us = g_now_us;
        process_sample(&sample);
        g_now_us += TEST_SAMPLE_US;
    }
}

static void feed_frame(uint16_t pattern, bool b_reverse,
                       uint16_t narrow, uint16_t wide)
{
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    uint8_t index = 0U;

    make_frame(pattern, b_reverse, narrow, wide, widths);
    feed_colour(false, TEST_QUIET_US);
    for (index = 0U; index < S3_FRAME_ELEMENTS; index++)
    {
        feed_colour(0U == (index & 1U), widths[index]);
    }
    feed_colour(false, TEST_QUIET_US);
}

static void run_line_once(void)
{
    g_b_stop_task = true;
    g_task_cycles = 1U;
    if (0 == setjmp(g_task_exit))
    {
        line_task(0, NULL);
    }
    g_b_stop_task = false;
}

static void run_barcode_once(void)
{
    g_wait_us = PERIOD_BARCODE_MS * 1000U;
    g_b_stop_task = true;
    g_task_cycles = 1U;
    if (0 == setjmp(g_task_exit))
    {
        barcode_task(0, NULL);
    }
    g_b_stop_task = false;
    g_wait_us = PERIOD_LINE_MS * 1000U;
}

static void feed_elements(uint16_t const * p_widths)
{
    uint8_t index = 0U;

    for (index = 0U; index < S3_FRAME_ELEMENTS; index++)
    {
        feed_colour(0U == (index & 1U), p_widths[index]);
    }
}

static void test_capture_boundaries(void)
{
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    s3_barcode_result_t result = {0};
    uint16_t saved = 0U;
    uint8_t reverse = 0U;

    for (reverse = 0U; reverse < 2U; reverse++)
    {
        g_capture = 0U;
        make_frame(g_letter_patterns[0], 0U != reverse, 4000U, 10000U, widths);
        feed_colour(false, 40000U);
        feed_elements(widths);
        feed_colour(false, 40000U);
        saved = g_latest_result;
        /* This edge completes A's quiet zone AND starts the next barcode B. */
        make_frame(g_letter_patterns[1], 0U != reverse, 4000U, 10000U, widths);
        feed_colour(true, widths[0]);
        TEST_CHECK(saved != g_latest_result);
        TEST_CHECK(s3_read_barcode(&result) && 'A' == result.letter);
        for (uint8_t index = 1U; index < S3_FRAME_ELEMENTS; index++)
        {
            feed_colour(0U == (index & 1U), widths[index]);
        }
        feed_colour(false, 41000U);
        TEST_CHECK(s3_read_barcode(&result) && 'B' == result.letter);
    }

    /* A short quiet zone must still reject the candidate. */
    g_capture = 0U;
    make_frame(g_letter_patterns[2], false, 4000U, 10000U, widths);
    feed_colour(false, 40000U);
    feed_elements(widths);
    saved = g_latest_result;
    feed_colour(false, 39000U);
    feed_colour(true, 1000U);
    TEST_CHECK(saved == g_latest_result);

    /* A long invalid black run must not consume the next valid start edge. */
    g_capture = 0U;
    feed_colour(false, 40000U);
    feed_colour(true, S3_MAX_BAR_US + 2000U);
    feed_colour(false, 40000U);
    feed_elements(widths);
    feed_colour(false, 41000U);
    TEST_CHECK(s3_read_barcode(&result) && 'C' == result.letter);

    /* A failed/future mapping must never be packed into another direction. */
    saved = g_latest_result;
    g_b_invalid_mapping = true;
    feed_frame(g_letter_patterns[0], false, 4000U, 10000U);
    TEST_CHECK(saved == g_latest_result);
    g_b_invalid_mapping = false;
}

static void test_delayed_capture(void)
{
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    uint16_t saved = g_latest_result;

    /* A pending A with old trailing-white samples must not publish after
     * a task delay has made the returned batch too old to finish safely. */
    g_capture = 0U;
    make_frame(g_letter_patterns[0], false, 4000U, 10000U, widths);
    feed_colour(false, 40000U);
    feed_elements(widths);
    feed_colour(false, 39000U);
    g_control = CONTROL_ACTIVE | CONTROL_CALIBRATED;
    g_seen_ticket = g_reset_ticket;
    g_mock_batch_count = 1U;
    g_mock_batch[0].value = 100U;
    g_mock_batch[0].timestamp_us = g_now_us + 1000U;
    g_now_us += 20000U;
    run_barcode_once();
    TEST_CHECK(saved == g_latest_result);
    TEST_CHECK(0U == g_candidate);
    TEST_CHECK(0U == (g_capture & CAPTURE_FLAG));
    g_mock_batch_count = 0U;

    /* A fresh buffered completion still publishes, including its direction. */
    feed_colour(false, 40000U);
    feed_elements(widths);
    feed_colour(false, 39000U);
    g_mock_batch_count = 1U;
    g_mock_batch[0].value = 100U;
    g_mock_batch[0].timestamp_us = g_now_us + 1000U;
    run_barcode_once();
    TEST_CHECK(saved != g_latest_result);
    TEST_CHECK(0U == g_candidate);
    g_mock_batch_count = 0U;
}

static void test_airborne_barcode(void)
{
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    uint16_t saved = g_latest_result;
    adc_sample_t sample = {4095U, 0U};

    g_capture = 0U;
    make_frame(g_letter_patterns[0], false, 4000U, 10000U, widths);
    feed_colour(false, 40000U);
    feed_elements(widths);
    feed_colour(false, 39000U);
    sample.timestamp_us = g_now_us + 1000U;
    process_sample(&sample);
    g_now_us = sample.timestamp_us + 1000U;
    TEST_CHECK(saved == g_latest_result);
    TEST_CHECK(0U == g_candidate && 0U == g_capture);
}

static uint16_t reference_pattern(char const * p_elements)
{
    uint16_t pattern = 0U;
    uint8_t index = 0U;

    for (index = 0U; index < S3_CHAR_ELEMENTS; index++)
    {
        char element = p_elements[index];
        pattern = (uint16_t)(pattern << 1U);
        if ((element >= 'A') && (element <= 'Z'))
        {
            pattern |= 1U;
        }
    }
    return (pattern);
}

static void test_independent_patterns(void)
{
    static uint16_t const narrows[3] = {2000U, 4000U, 8000U};
    static uint16_t const wides[3] = {5000U, 10000U, 20000U};
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    s3_barcode_result_t result = {0};
    uint8_t letter = 0U;
    uint8_t reverse = 0U;
    uint8_t speed = 0U;
    uint8_t previous_sequence = 0U;
    uint32_t quiet_us = 0U;

    TEST_CHECK(CODE39_STAR == reference_pattern(g_star));
    for (letter = 0U; letter < S3_LETTER_COUNT; letter++)
    {
        for (reverse = 0U; reverse < 2U; reverse++)
        {
            for (speed = 0U; speed < 3U; speed++)
            {
                quiet_us = (uint32_t)narrows[speed] * S3_QUIET_NARROW_MULT;
                make_frame(reference_pattern(g_reference[letter]),
                           0U != reverse, narrows[speed], wides[speed], widths);
                TEST_CHECK(s3_decode_code39(widths, S3_FRAME_ELEMENTS,
                    quiet_us, quiet_us, &result));
                TEST_CHECK(result.letter == (uint8_t)('A' + letter));
                TEST_CHECK(MSG_BC_NONE == result.direction);
                previous_sequence = (uint8_t)
                    ((g_latest_result >> SEQUENCE_SHIFT) & 0x03U);
                g_capture = 0U;
                feed_colour(false, quiet_us);
                feed_elements(widths);
                feed_colour(false, quiet_us + TEST_SAMPLE_US);
                TEST_CHECK(s3_read_barcode(&result));
                TEST_CHECK(result.letter == (uint8_t)('A' + letter));
                TEST_CHECK(result.b_reverse == (0U != reverse));
                TEST_CHECK(result.direction
                    == barcode_mapping_direction((uint8_t)('A' + letter)));
                TEST_CHECK(result.sequence == ((previous_sequence + 1U) & 3U));
            }
        }
    }
}

static void test_calibration_reset_and_limits(void)
{
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    s3_barcode_result_t result = {0};
    uint16_t saved = g_latest_result;
    uint32_t calibration = g_calibration;
    uint8_t ticket = g_reset_ticket;

    /* Public threshold updates cancel old capture but preserve the result. */
    g_capture = CAPTURE_FLAG | COLOUR_FLAG;
    g_candidate = 1U;
    g_seen_ticket = ticket;
    s3_barcode_calibrate(0U, S3_HYSTERESIS);
    TEST_CHECK(calibration == g_calibration && ticket == g_reset_ticket);
    s3_set_barcode_threshold(2000U);
    TEST_CHECK(ticket != g_reset_ticket);
    g_control = CONTROL_ACTIVE | CONTROL_CALIBRATED;
    run_barcode_once();
    TEST_CHECK(0U == g_candidate && 0U == g_capture);
    TEST_CHECK(saved == g_latest_result);
    s3_barcode_calibrate(1550U, S3_HYSTERESIS);

    /* Model preemption just after the atomic calibration commit, before
     * its reset ticket is posted. The decoder must notice the new values. */
    g_capture = 0U;
    make_frame(g_letter_patterns[0], false, 4000U, 10000U, widths);
    feed_colour(false, 40000U);
    feed_elements(widths);
    feed_colour(false, 39000U);
    saved = g_latest_result;
    __atomic_store_n(&g_calibration,
        2000U | (S3_HYSTERESIS << 12U), __ATOMIC_RELEASE);
    feed_colour(false, 2000U);
    TEST_CHECK(saved == g_latest_result);
    TEST_CHECK(0U == g_candidate);
    s3_barcode_calibrate(1550U, S3_HYSTERESIS);

    make_frame(g_letter_patterns[0], false, 40000U, 60000U, widths);
    widths[9] = 60000U;
    TEST_CHECK(s3_decode_code39(widths, S3_FRAME_ELEMENTS,
                               500000U, 500000U, &result));
    /* The pure API must enforce the same element limit as live capture. */
    widths[9] = 65000U;
    TEST_CHECK(!s3_decode_code39(widths, S3_FRAME_ELEMENTS,
                                500000U, 500000U, &result));
    TEST_CHECK('A' == result.letter);

    /* A real capture spanning the native microsecond clock rollover. */
    g_now_us = UINT32_MAX - 50000U;
    g_capture = 0U;
    feed_frame(g_letter_patterns[1], true, 4000U, 10000U);
    TEST_CHECK(s3_read_barcode(&result) && 'B' == result.letter
               && result.b_reverse && barcode_mapping_direction('B') == result.direction);
}

static void test_alphabet(void)
{
    uint8_t letter = 0U;
    uint8_t reverse = 0U;
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    s3_barcode_result_t result = {0};
    msg_barcode_t command = {0};

    TEST_CHECK(!s3_read_barcode(&result));
    for (letter = 0U; letter < S3_LETTER_COUNT; letter++)
    {
        for (reverse = 0U; reverse < 2U; reverse++)
        {
            make_frame(g_letter_patterns[letter], (0U != reverse),
                       TEST_NARROW_US, TEST_WIDE_US, widths);
            TEST_CHECK(s3_decode_code39(widths, S3_FRAME_ELEMENTS,
                                       TEST_QUIET_US, TEST_QUIET_US, &result));
            TEST_CHECK((uint8_t)('A' + letter) == result.letter);
            TEST_CHECK((0U != reverse) == result.b_reverse);
            TEST_CHECK(MSG_BC_NONE == result.direction);
            feed_frame(g_letter_patterns[letter], (0U != reverse),
                       TEST_NARROW_US, TEST_WIDE_US);
            TEST_CHECK(s3_read_barcode(&result));
            TEST_CHECK((uint8_t)('A' + letter) == result.letter);
            TEST_CHECK(barcode_mapping_direction(result.letter)
                       == result.direction);
            TEST_CHECK(s3_take_barcode(&command));
            TEST_CHECK(result.direction == command.cmd);
            TEST_CHECK(s3_read_barcode(&result));
            TEST_CHECK(!s3_take_barcode(&command));
        }
    }
    TEST_CHECK(barcode_mapping_is_valid());
    for (letter = (uint8_t)'A'; letter <= (uint8_t)'Z'; letter++)
    {
        msg_barcode_cmd_t direction = barcode_mapping_direction(letter);
        TEST_CHECK(direction >= MSG_BC_STRAIGHT && direction <= MSG_BC_U_TURN);
    }
    TEST_CHECK(MSG_BC_INVALID == barcode_mapping_direction('a'));
    TEST_CHECK(MSG_BC_INVALID == barcode_mapping_direction('0'));
}

static void test_invalid_and_queues(void)
{
    uint16_t widths[S3_FRAME_ELEMENTS] = {0U};
    uint16_t saved = g_latest_result;
    uint8_t index = 0U;
    s3_barcode_result_t result = {0};

    make_frame(g_letter_patterns[0], false, 4000U, 10000U, widths);
    TEST_CHECK(!s3_decode_code39(widths, 28U, 80000U, 80000U, &result));
    TEST_CHECK(!s3_decode_code39(widths, 30U, 80000U, 80000U, &result));
    TEST_CHECK(!s3_decode_code39(NULL, 29U, 80000U, 80000U, &result));
    TEST_CHECK(!s3_decode_code39(widths, 29U, 0U, 80000U, &result));
    TEST_CHECK(!s3_decode_code39(widths, 29U, 80000U, 0U, &result));
    for (index = 0U; index < S3_FRAME_ELEMENTS; index++)
    {
        uint16_t original = widths[index];
        widths[index] = 0U;
        TEST_CHECK(!s3_decode_code39(widths, 29U, 80000U, 80000U, &result));
        widths[index] = original;
    }
    feed_frame(0x034U, false, 4000U, 10000U); /* A digit is not A..Z. */
    TEST_CHECK(saved == g_latest_result);
    feed_colour(false, 80000U);
    feed_colour(true, 4000U);
    feed_colour(false, 4000U);
    feed_colour(true, 4000U);
    feed_colour(false, 10000U);
    feed_colour(true, 40000U); /* Cannot finish a frame on black. */
    TEST_CHECK(saved == g_latest_result);

    g_capture = 0U;
    g_send_error = E_OBJ;
    feed_frame(g_letter_patterns[0], false, 6000U, 16000U);
    TEST_CHECK(s3_read_barcode(&result));
    TEST_CHECK('A' == result.letter && barcode_mapping_direction('A') == result.direction);
    g_send_error = E_OK;
    for (index = 0U; index < 6U; index++)
    {
        feed_frame(g_letter_patterns[index], false, 4000U, 10000U);
    }
    TEST_CHECK(g_queue_count <= 4U);
    TEST_CHECK(s3_take_barcode_result(&result));
    TEST_CHECK('F' == result.letter);
    TEST_CHECK(0U == g_queue_count);
    saved = g_latest_result;
    s3_set_active(false);
    s3_set_active(true);
    TEST_CHECK(saved == g_latest_result);
    TEST_CHECK(0U == g_queue_count);
}

static void test_calibration_and_line(void)
{
    msg_line_t line = {0};
    uint32_t word = 0U;
    uint32_t saved_left = g_left_cal;
    uint32_t saved_right = g_right_cal;
    uint8_t sensors = 0U;
    uint8_t count = 0U;

    g_mode = 1U;
    TEST_CHECK(E_IO == s3_calibrate());
    TEST_CHECK(saved_left == g_left_cal && saved_right == g_right_cal);
    g_mode = 2U;
    TEST_CHECK(E_IO == s3_calibrate());
    g_mode = 3U;
    TEST_CHECK(E_IO == s3_calibrate());
    g_mode = 4U;
    g_b_frozen = true;
    TEST_CHECK(E_IO == s3_calibrate());
    g_b_frozen = false;
    g_mode = 5U;
    g_values[0] = 3900U;
    TEST_CHECK(E_IO == s3_calibrate());
    TEST_CHECK(saved_left == g_left_cal && saved_right == g_right_cal);
    g_mode = 4U;
    g_mode = 7U;
    TEST_CHECK(E_IO == s3_calibrate());
    TEST_CHECK(saved_left == g_left_cal && saved_right == g_right_cal);
    g_mode = 8U;
    TEST_CHECK(E_IO == s3_calibrate());
    TEST_CHECK(saved_left == g_left_cal && saved_right == g_right_cal);
    g_mode = 4U;
    TEST_CHECK(E_OK == s3_calibrate());
    TEST_CHECK((g_left_cal & 0xFFFU) <= S3_ADC_MAX);
    TEST_CHECK((g_right_cal & 0xFFFU) <= S3_ADC_MAX);
    s3_set_active(true);
    TEST_CHECK(s3_barcode_active());
    TEST_CHECK(!s3_read_line_sensors(&sensors));
    g_mode = 0U;
    g_values[0] = 100U;
    g_values[1] = 100U;
    run_line_once();
    TEST_CHECK(s3_read_line_word(&word));
    msg_unpack_line(word, &line);
    TEST_CHECK(line.b_valid && 0 == line.error);
    TEST_CHECK(MSG_LINE_TRACK == s3_get_line_state());
    TEST_CHECK(s3_read_line_sensors(&sensors) && 0U == sensors);
    TEST_CHECK(!s3_read_line_sensors(NULL));
    g_events = 0U;
    /* Long white/white is aligned, not the old debounced line-lost case. */
    for (count = 0U; count < 25U; count++)
    {
        run_line_once();
    }
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(line.b_valid && 0 == line.error);
    TEST_CHECK(0U == (g_events & S5_EVT_LINE_LOST));
    g_values[0] = 3000U;
    run_line_once();
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(line.b_valid && -S3_LINE_CORRECTION == line.error);
    TEST_CHECK(s3_read_line_sensors(&sensors)
               && S3_LINE_LEFT_BLACK == sensors);
    g_values[0] = 100U;
    g_values[1] = 3000U;
    run_line_once();
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(line.b_valid && S3_LINE_CORRECTION == line.error);
    TEST_CHECK(s3_read_line_sensors(&sensors)
               && S3_LINE_RIGHT_BLACK == sensors);

    /* Noise on both sensors is not immediately a confirmed junction. */
    g_values[0] = 3000U;
    run_line_once();
    TEST_CHECK(s3_read_line_sensors(&sensors)
               && 0U == (sensors & S3_LINE_JUNCTION));
    track_once(); /* Duplicate pair must not count twice. */
    TEST_CHECK(1U == g_junction_count);
    g_values[0] = 100U;
    run_line_once();
    TEST_CHECK(0U == g_junction_count);
    g_values[0] = 3000U;
    for (count = 1U; count <= S3_JUNCTION_CONFIRM_SAMPLES; count++)
    {
        run_line_once();
        TEST_CHECK(s3_read_line_sensors(&sensors));
        TEST_CHECK((count >= S3_JUNCTION_CONFIRM_SAMPLES)
                   == (0U != (sensors & S3_LINE_JUNCTION)));
    }
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(line.b_valid && 0 == line.error);
    TEST_CHECK(MSG_LINE_TRACK == s3_get_line_state());
    TEST_CHECK((S3_LINE_LEFT_BLACK | S3_LINE_RIGHT_BLACK | S3_LINE_JUNCTION)
               == sensors);
    for (count = 0U; count < 10U; count++)
    {
        run_line_once();
    }
    TEST_CHECK(S3_JUNCTION_CONFIRM_SAMPLES == g_junction_count);
    /* A pause expires the public observation even without another task tick. */
    g_now_us += S3_SAMPLE_STALE_US + 1000U;
    TEST_CHECK(!s3_read_line_sensors(&sensors) && 0U == sensors);
    g_values[0] = 100U;
    g_values[1] = 100U;
    run_line_once();
    TEST_CHECK(s3_read_line_sensors(&sensors) && 0U == sensors);
    TEST_CHECK(0U == g_junction_count);

    /* No old encounter survives a stop/start between line-task ticks. */
    g_values[0] = 3000U;
    g_values[1] = 3000U;
    run_line_once();
    s3_set_active(false);
    TEST_CHECK(!s3_read_line_sensors(&sensors));
    s3_set_active(true);
    run_line_once();
    TEST_CHECK(1U == g_junction_count);
    TEST_CHECK(s3_read_line_sensors(&sensors)
               && 0U == (sensors & S3_LINE_JUNCTION));

    /* Actual ADC failure/staleness still invalidates line and junction. */
    g_b_frozen = true;
    run_line_once();
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(!line.b_valid && MSG_LINE_LOST == s3_get_line_state());
    TEST_CHECK(0U != (g_events & S5_EVT_LINE_LOST));
    TEST_CHECK(!s3_read_line_sensors(&sensors) && 0U == sensors);
    TEST_CHECK(0U == g_junction_count);
    g_b_frozen = false;
    run_line_once();
    g_mode = 3U;
    run_line_once();
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(!line.b_valid && MSG_LINE_LOST == s3_get_line_state());
    g_mode = 0U;
    TEST_CHECK(!s3_read_line_sensors(&sensors));
    g_mode = 6U;
    run_line_once();
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(!line.b_valid && MSG_LINE_LOST == s3_get_line_state());
    TEST_CHECK(!s3_read_line_sensors(&sensors));
    g_mode = 0U;
    TEST_CHECK(is_dark(1550U, g_right_cal, true));
    TEST_CHECK(!is_dark(1550U, g_right_cal, false));
    g_values[0] = 4095U;
    g_values[1] = 3000U;
    run_line_once();
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(!line.b_valid && MSG_LINE_LOST == s3_get_line_state());
    TEST_CHECK(!s3_read_line_sensors(&sensors));
    g_values[0] = 100U;
    g_values[1] = 4095U;
    run_line_once();
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(!line.b_valid && MSG_LINE_LOST == s3_get_line_state());
    g_values[0] = 100U;
    g_values[1] = 100U;
    run_line_once();
    msg_unpack_line(g_line_word, &line);
    TEST_CHECK(line.b_valid && 0 == line.error);
    /* Expiry works across the observation's 28-bit ms timestamp wrap. */
    g_ms_bias = SENSOR_STAMP_MASK - app_now_ms() - 2U;
    run_line_once();
    TEST_CHECK(s3_read_line_sensors(&sensors) && 0U == sensors);
    g_now_us += S3_SAMPLE_STALE_US + 1000U;
    TEST_CHECK(!s3_read_line_sensors(&sensors));
    g_ms_bias = 0U;
    /* And across the system's full 32-bit ms clock wrap. */
    g_ms_bias = UINT32_MAX - app_now_ms() - 2U;
    run_line_once();
    TEST_CHECK(s3_read_line_sensors(&sensors) && 0U == sensors);
    g_ms_bias = 0U;
    /* The ADC microsecond clock may wrap independently. */
    g_now_us = UINT32_MAX - 2000U;
    run_line_once();
    TEST_CHECK(s3_read_line_sensors(&sensors) && 0U == sensors);
    s3_set_active(false);
    TEST_CHECK(!s3_barcode_active());
    run_line_once();
    TEST_CHECK(!s3_read_line_sensors(&sensors));
    TEST_CHECK(MSG_LINE_IDLE == s3_get_line_state());
}

INT tm_sprintf(UB * buffer, UB const * format, ...)
{
    (void)format;
    buffer[0] = (UB)0;
    return (0);
}
void app_log_line(UB const * text) { (void)text; }

int main(void)
{
    g_create_result = E_OBJ;
    TEST_CHECK(E_OBJ == s3_barcode_init());
    TEST_CHECK(E_OBJ == s3_barcode_init());
    g_create_result = 1;
    TEST_CHECK(E_OK == s3_barcode_init());
    TEST_CHECK(3U == g_create_calls);
    TEST_CHECK(E_OK == s3_barcode_init() && 3U == g_create_calls);
    test_alphabet();
    test_invalid_and_queues();
    test_capture_boundaries();
    test_delayed_capture();
    test_airborne_barcode();
    test_independent_patterns();
    test_calibration_reset_and_limits();
    test_calibration_and_line();
    printf("Buddy 3 regressions: %u/%u passed\n", g_tests - g_failed, g_tests);
    return ((0U == g_failed) ? 0 : 1);
}
