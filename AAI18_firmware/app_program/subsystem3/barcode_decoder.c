/**
 * @file barcode_decoder.c
 * @brief Bounded Code 39 *A*..*Z* capture, decoding and retained results.
 *
 * Samples and capture state have one writer (barcode_task). The latest
 * result is one 16-bit release/acquire snapshot, safe on the other core.
 * Notification congestion never changes whether that snapshot is updated.
 */
#include "barcode_decoder.h"
#include "barcode_mapping.h"
#include "line_sensor.h"
#include "adc_sampler.h"
#include "../subsystem5/subsystem5.h"

#define LETTER_MASK       (0x001FU)
#define DIRECTION_SHIFT   (5U)
#define CONFIDENCE_SHIFT  (7U)
#define SEQUENCE_SHIFT    (10U)
#define REVERSE_SHIFT     (12U)
#define RESERVED_MASK     (0xE000U)
#define COUNT_MASK        (0x1FU)
#define DARK_FLAG         (0x20U)
#define COLOUR_FLAG       (0x40U)
#define CAPTURE_FLAG      (0x80U)
#define CODE39_STAR       (0x094U)
#define CODE39_WIDE_COUNT (3U)
#define CODE39_NARROWS    (6U)
#define RATIO_SCALE       (100U)
#define MIN_SEPARATION    (130U)
#define GOOD_SEPARATION   (150U)
#define BEST_SEPARATION   (180U)

static uint16_t const g_letter_patterns[S3_LETTER_COUNT] =
{
    0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, 0x00DU,
    0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U,
    0x0C1U, 0x1C0U, 0x091U, 0x190U, 0x0D0U
};

static ID g_h_barcode = (ID)0;
static uint16_t volatile g_latest_result = 0U;
static uint32_t volatile g_calibration =
    (S3_ADC_MAX / 2U) | (S3_HYSTERESIS << 12U);
static uint8_t volatile g_reset_ticket = 0U;

/* Private capture: count (5), dark (1), colour known (1), capturing (1). */
static uint8_t g_capture = 0U;
static uint8_t g_seen_ticket = 0U;
static uint32_t g_capture_calibration = 0U;
static uint16_t g_widths[S3_FRAME_ELEMENTS];
static uint16_t g_candidate = 0U;
static uint32_t g_edge_us = 0U;
static uint32_t g_frame_us = 0U;
static uint32_t g_last_sample_us = 0U;
static uint32_t g_leading_us = 0U;
static uint32_t g_quiet_us = 0U;
static uint16_t g_max_cycle_us = 0U;

static bool unpack_result(uint16_t word, s3_barcode_result_t * p_result)
{
    uint8_t code = (uint8_t)(word & LETTER_MASK);
    bool b_valid = (NULL != p_result) && (0U == (word & RESERVED_MASK))
                   && (0U != code) && (code <= S3_LETTER_COUNT);

    if (b_valid)
    {
        p_result->letter = (uint8_t)((uint8_t)'A' + code - 1U);
        p_result->direction = (msg_barcode_cmd_t)
            (((word >> DIRECTION_SHIFT) & 0x03U) + 1U);
        p_result->confidence = (uint8_t)
            ((word >> CONFIDENCE_SHIFT) & 0x07U);
        p_result->sequence = (uint8_t)
            ((word >> SEQUENCE_SHIFT) & 0x03U);
        p_result->b_reverse = (0U != ((word >> REVERSE_SHIFT) & 1U));
    }
    return (b_valid);
}

static uint16_t logical_width(uint16_t const * p_widths, uint8_t index,
                              bool b_reverse)
{
    uint8_t physical = b_reverse
        ? (uint8_t)(S3_FRAME_ELEMENTS - 1U - index) : index;

    return (p_widths[physical]);
}

/* Nine-element insertion sort runs only at a complete frame, not per ADC
 * sample. The largest three must separate cleanly from the six narrows. */
static bool decode_character(uint16_t const * p_widths, uint8_t offset,
                             bool b_reverse, uint16_t * p_pattern,
                             uint16_t * p_narrow, uint8_t * p_confidence)
{
    uint16_t sorted[S3_CHAR_ELEMENTS] = {0U};
    uint16_t value = 0U;
    uint16_t pattern = 0U;
    uint32_t separation = 0U;
    uint8_t index = 0U;
    uint8_t position = 0U;
    bool b_valid = true;

    for (index = 0U; index < S3_CHAR_ELEMENTS; index++)
    {
        value = logical_width(p_widths, (uint8_t)(offset + index),
                              b_reverse);
        if ((value < S3_MIN_BAR_US) || (value > S3_MAX_BAR_US))
        {
            b_valid = false;
        }
        position = index;
        while ((0U != position) && (sorted[position - 1U] > value))
        {
            sorted[position] = sorted[position - 1U];
            position--;
        }
        sorted[position] = value;
    }
    if (b_valid)
    {
        separation = ((uint32_t)sorted[CODE39_NARROWS] * RATIO_SCALE)
                     / sorted[CODE39_NARROWS - 1U];
        b_valid = (separation >= MIN_SEPARATION)
            && ((uint32_t)sorted[CODE39_NARROWS - 1U]
                <= (2U * sorted[0]))
            && ((uint32_t)sorted[S3_CHAR_ELEMENTS - 1U]
                <= (4U * sorted[0]));
    }
    if (b_valid)
    {
        for (index = 0U; index < S3_CHAR_ELEMENTS; index++)
        {
            value = logical_width(p_widths, (uint8_t)(offset + index),
                                  b_reverse);
            pattern = (uint16_t)(pattern << 1U);
            if (value >= sorted[CODE39_NARROWS])
            {
                pattern |= 1U;
            }
        }
        *p_pattern = pattern;
        *p_narrow = sorted[CODE39_NARROWS / 2U];
        *p_confidence = (separation >= BEST_SEPARATION) ? 7U
            : ((separation >= GOOD_SEPARATION) ? 6U : 4U);
    }
    return (b_valid);
}

static bool decode_orientation(uint16_t const * p_widths, bool b_reverse,
                               uint32_t leading_us,
                               s3_barcode_result_t * p_result,
                               uint32_t * p_quiet_us)
{
    uint16_t pattern = 0U;
    uint16_t narrow = 0U;
    uint16_t widest_narrow = 0U;
    uint16_t gap = 0U;
    uint8_t confidence = 7U;
    uint8_t char_confidence = 0U;
    uint8_t character = 0U;
    uint8_t letter_index = 0U;
    uint8_t letter = 0U;
    uint8_t offset = 0U;
    bool b_valid = true;

    for (character = 0U; (character < 3U) && b_valid; character++)
    {
        offset = (uint8_t)(character * (S3_CHAR_ELEMENTS + 1U));
        b_valid = decode_character(p_widths, offset, b_reverse, &pattern,
                                   &narrow, &char_confidence);
        if (b_valid)
        {
            if (char_confidence < confidence)
            {
                confidence = char_confidence;
            }
            if (narrow > widest_narrow)
            {
                widest_narrow = narrow;
            }
            if (1U == character)
            {
                for (letter_index = 0U; letter_index < S3_LETTER_COUNT;
                     letter_index++)
                {
                    if (pattern == g_letter_patterns[letter_index])
                    {
                        letter = (uint8_t)((uint8_t)'A' + letter_index);
                        break;
                    }
                }
                b_valid = (0U != letter);
            }
            else
            {
                b_valid = (CODE39_STAR == pattern);
            }
        }
        if (b_valid && (character < 2U))
        {
            gap = logical_width(p_widths,
                (uint8_t)(offset + S3_CHAR_ELEMENTS), b_reverse);
            b_valid = (gap >= S3_MIN_BAR_US)
                && (gap <= S3_MAX_BAR_US)
                && ((2U * (uint32_t)gap) >= narrow)
                && ((uint32_t)gap <= (2U * narrow));
        }
    }
    if (b_valid)
    {
        *p_quiet_us = (uint32_t)widest_narrow * S3_QUIET_NARROW_MULT;
        if (*p_quiet_us < S3_MIN_QUIET_US)
        {
            *p_quiet_us = S3_MIN_QUIET_US;
        }
        b_valid = (leading_us >= *p_quiet_us);
    }
    if (b_valid)
    {
        p_result->letter = letter;
        /* Direction policy belongs to barcode_mapping.c, not this decoder. */
        p_result->direction = MSG_BC_NONE;
        p_result->confidence = confidence;
        p_result->sequence = 0U;
        p_result->b_reverse = b_reverse;
    }
    return (b_valid);
}

bool s3_decode_code39(uint16_t const * p_widths, uint8_t count,
                     uint32_t leading_white_us, uint32_t trailing_white_us,
                     s3_barcode_result_t * p_result)
{
    s3_barcode_result_t result = {0};
    uint32_t quiet_us = 0U;
    uint32_t frame_us = 0U;
    uint8_t index = 0U;
    bool b_valid = false;

    if ((NULL != p_widths) && (NULL != p_result)
        && (S3_FRAME_ELEMENTS == count))
    {
        b_valid = decode_orientation(p_widths, false, leading_white_us,
                                     &result, &quiet_us);
        if (!b_valid)
        {
            b_valid = decode_orientation(p_widths, true, leading_white_us,
                                         &result, &quiet_us);
        }
        b_valid = b_valid && (trailing_white_us >= quiet_us);
        for (index = 0U; index < S3_FRAME_ELEMENTS; index++)
        {
            frame_us += p_widths[index];
        }
        b_valid = b_valid && ((frame_us + quiet_us) <= S3_MAX_FRAME_US);
    }
    if (b_valid)
    {
        *p_result = result;
    }
    return (b_valid);
}

static void clear_frame(void)
{
    g_capture &= (uint8_t)(DARK_FLAG | COLOUR_FLAG);
    g_candidate = 0U;
    g_quiet_us = 0U;
}

static void stage_frame(void)
{
    s3_barcode_result_t result = {0};
    bool b_valid = decode_orientation(g_widths, false, g_leading_us,
                                     &result, &g_quiet_us);

    if (!b_valid)
    {
        b_valid = decode_orientation(g_widths, true, g_leading_us,
                                     &result, &g_quiet_us);
    }
    if (b_valid)
    {
        g_candidate = (uint16_t)(result.letter - (uint8_t)'A' + 1U)
            | ((uint16_t)result.confidence << CONFIDENCE_SHIFT)
            | ((uint16_t)(result.b_reverse ? 1U : 0U) << REVERSE_SHIFT);
    }
    else
    {
        clear_frame();
    }
}

static void publish_candidate(void)
{
    uint16_t previous = __atomic_load_n(&g_latest_result, __ATOMIC_ACQUIRE);
    uint16_t sequence = (uint16_t)
        ((((previous >> SEQUENCE_SHIFT) & 0x03U) + 1U) & 0x03U);
    uint8_t letter = (uint8_t)
        ((uint8_t)'A' + (g_candidate & LETTER_MASK) - 1U);
    msg_barcode_cmd_t direction = barcode_mapping_direction(letter);
    uint16_t word = 0U;
    uint8_t bytes[S3_BARCODE_WIRE_BYTES] = {0U};
    uint8_t discarded[S3_BARCODE_WIRE_BYTES] = {0U};
    ER error = E_OBJ;

    /* Only the four supported motion commands fit the two direction bits.
     * Reject an unsupported mapping before it can corrupt the retained pair. */
    if ((direction < MSG_BC_STRAIGHT) || (direction > MSG_BC_U_TURN))
    {
        clear_frame();
        return;
    }
    word = g_candidate
        | ((uint16_t)((uint32_t)direction - 1U) << DIRECTION_SHIFT)
        | (uint16_t)(sequence << SEQUENCE_SHIFT);
    bytes[0] = (uint8_t)(word & 0xFFU);
    bytes[1] = (uint8_t)(word >> 8U);

    /* Latest state is committed even if notification delivery fails. */
    __atomic_store_n(&g_latest_result, word, __ATOMIC_RELEASE);
    error = tk_snd_mbf(g_h_barcode, bytes, S3_BARCODE_WIRE_BYTES, TMO_POL);
    if (E_TMOUT == error)
    {
        /* One producer: evict one oldest command and retry once. */
        (void)tk_rcv_mbf(g_h_barcode, discarded, TMO_POL);
        error = tk_snd_mbf(g_h_barcode, bytes, S3_BARCODE_WIRE_BYTES, TMO_POL);
    }
    if (E_OK == error)
    {
        s5_post_event(S5_EVT_BARCODE);
    }
    clear_frame();
}

static void process_sample(adc_sample_t const * p_sample)
{
    uint32_t calibration = __atomic_load_n(&g_calibration, __ATOMIC_ACQUIRE);
    uint32_t threshold = calibration & 0x0FFFU;
    uint32_t half_band = ((calibration >> 12U) & 0xFFU) / 2U;
    bool b_dark = (0U != (g_capture & DARK_FLAG));
    bool b_previous_dark = b_dark;
    uint32_t duration = 0U;
    uint8_t count = (uint8_t)(g_capture & COUNT_MASK);

    if (p_sample->value >= S3_AIR_LEVEL)
    {
        /* No surface is neither a bar nor a quiet zone. */
        g_capture = 0U;
        g_candidate = 0U;
        g_last_sample_us = p_sample->timestamp_us;
        return;
    }

    if ((0U == (g_capture & COLOUR_FLAG))
        || (calibration != g_capture_calibration)
        || ((p_sample->timestamp_us - g_last_sample_us)
            > S3_MAX_SAMPLE_GAP_US))
    {
        /* A gap or a calibration change makes the preceding interval
         * unknowable. Detect the atomic calibration commit here too, even
         * if this task preempted its writer before the reset was posted. */
        g_capture_calibration = calibration;
        g_capture = COLOUR_FLAG;
        if (p_sample->value > threshold)
        {
            g_capture |= DARK_FLAG;
        }
        g_edge_us = p_sample->timestamp_us;
        g_last_sample_us = p_sample->timestamp_us;
        g_candidate = 0U;
    }
    else
    {
        g_last_sample_us = p_sample->timestamp_us;
        if ((uint32_t)p_sample->value > (threshold + half_band))
        {
            b_dark = true;
        }
        else if (((uint32_t)p_sample->value + half_band) < threshold)
        {
            b_dark = false;
        }
        else
        {
            /* Inside hysteresis: keep the previous colour. */
        }
        duration = p_sample->timestamp_us - g_edge_us;
        if ((0U != (g_capture & CAPTURE_FLAG))
            && ((p_sample->timestamp_us - g_frame_us) > S3_MAX_FRAME_US))
        {
            clear_frame();
        }
        if (b_dark != b_previous_dark)
        {
            if ((0U != (g_capture & CAPTURE_FLAG))
                && (0U != g_candidate) && !b_previous_dark)
            {
                /* This white->black edge may finish the preceding quiet
                 * zone and start the next barcode at the very same instant. */
                if (duration >= g_quiet_us)
                {
                    publish_candidate();
                }
                else
                {
                    clear_frame();
                }
            }
            if (0U != (g_capture & CAPTURE_FLAG))
            {
                if ((duration < S3_MIN_BAR_US)
                    || (duration > S3_MAX_BAR_US)
                    || (count >= S3_FRAME_ELEMENTS))
                {
                    clear_frame();
                }
                else
                {
                    g_widths[count] = (uint16_t)duration;
                    count++;
                    g_capture = (uint8_t)
                        ((g_capture & (uint8_t)~COUNT_MASK) | count);
                    if (S3_FRAME_ELEMENTS == count)
                    {
                        stage_frame();
                    }
                }
            }
            if ((0U == (g_capture & CAPTURE_FLAG))
                && b_dark && (duration >= S3_MIN_QUIET_US))
            {
                g_capture = (uint8_t)
                    ((g_capture & (DARK_FLAG | COLOUR_FLAG)) | CAPTURE_FLAG);
                g_frame_us = p_sample->timestamp_us;
                g_leading_us = duration;
            }
            g_edge_us = p_sample->timestamp_us;
            g_capture = b_dark ? (uint8_t)(g_capture | DARK_FLAG)
                : (uint8_t)(g_capture & (uint8_t)~DARK_FLAG);
        }
        else if (0U != (g_capture & CAPTURE_FLAG))
        {
            if (!b_dark && (0U != g_candidate)
                && (duration >= g_quiet_us))
            {
                publish_candidate();
            }
            else if ((0U == g_candidate) && (duration > S3_MAX_BAR_US))
            {
                clear_frame();
            }
            else
            {
                /* A wide black bar is data, never a quiet zone. */
            }
        }
    }
}

ER s3_barcode_init(void)
{
    T_CMBF config = {0};
    ER error = E_OK;
    ID handle = (ID)0;

    if ((ID)0 == g_h_barcode)
    {
        config.mbfatr = TA_TFIFO;
        config.bufsz = (INT)APP_MBF_BUFSZ(S3_BARCODE_QUEUE_DEPTH,
                                        S3_BARCODE_WIRE_BYTES);
        config.maxmsz = S3_BARCODE_WIRE_BYTES;
        handle = tk_cre_mbf(&config);
        if (handle > (ID)0)
        {
            g_h_barcode = handle;
        }
        else
        {
            /* Keep zero on failure so a later startup retry creates it. */
            error = E_OBJ;
        }
    }
    return (error);
}

bool s3_read_barcode(s3_barcode_result_t * p_result)
{
    return (unpack_result(
        __atomic_load_n(&g_latest_result, __ATOMIC_ACQUIRE), p_result));
}

bool s3_take_barcode_result(s3_barcode_result_t * p_result)
{
    uint8_t bytes[S3_BARCODE_WIRE_BYTES] = {0U};
    uint16_t word = 0U;
    uint8_t count = 0U;
    bool b_received = false;
    INT size = 0;

    if (NULL != p_result)
    {
        for (count = 0U; count < S3_BARCODE_QUEUE_DEPTH; count++)
        {
            size = tk_rcv_mbf(g_h_barcode, bytes, TMO_POL);
            if (size < (INT)S3_BARCODE_WIRE_BYTES)
            {
                break;
            }
            word = (uint16_t)((uint16_t)bytes[0]
                              | ((uint16_t)bytes[1] << 8U));
            if (unpack_result(word, p_result))
            {
                b_received = true;
            }
        }
        if (S3_BARCODE_QUEUE_DEPTH == count)
        {
            /* Covers a producer enqueue during the bounded drain. */
            s5_post_event(S5_EVT_BARCODE);
        }
    }
    return (b_received);
}

bool s3_take_barcode(msg_barcode_t * p_barcode)
{
    s3_barcode_result_t result = {0};
    bool b_received = (NULL != p_barcode)
                      && s3_take_barcode_result(&result);

    if (b_received)
    {
        p_barcode->cmd = result.direction;
        p_barcode->confidence = result.confidence;
        p_barcode->seq = result.sequence;
    }
    return (b_received);
}

uint8_t s3_get_last_barcode(void)
{
    s3_barcode_result_t result = {0};
    uint8_t direction = (uint8_t)MSG_BC_NONE;

    if (s3_read_barcode(&result))
    {
        direction = (uint8_t)result.direction;
    }
    return (direction);
}

void s3_barcode_reset(void)
{
    uint8_t bytes[S3_BARCODE_WIRE_BYTES] = {0U};
    uint8_t count = 0U;

    uint8_t ticket = __atomic_load_n(&g_reset_ticket, __ATOMIC_RELAXED);

    __atomic_store_n(&g_reset_ticket, (uint8_t)(ticket + 1U), __ATOMIC_RELEASE);
    if (g_h_barcode > (ID)0)
    {
        for (count = 0U; count < S3_BARCODE_QUEUE_DEPTH; count++)
        {
            if (tk_rcv_mbf(g_h_barcode, bytes, TMO_POL) < E_OK)
            {
                break;
            }
        }
    }
    /* The retained result deliberately survives STOP, START and calibration. */
}

void s3_report_barcode(void)
{
    static uint16_t g_reported = 0U;
    s3_barcode_result_t result = {0};
    uint16_t word = __atomic_load_n(&g_latest_result, __ATOMIC_ACQUIRE);

    if ((word != g_reported) && unpack_result(word, &result))
    {
        APP_LOG((UB *)"[S3] barcode %c -> %s\n", (INT)result.letter,
                  barcode_mapping_name(result.direction));
        g_reported = word;
    }
}

void s3_barcode_calibrate(uint16_t threshold, uint16_t band)
{
    uint32_t half_band = (uint32_t)band / 2U;

    if ((band >= S3_MIN_BAND) && (band <= S3_HYSTERESIS)
        && (threshold > half_band)
        && (((uint32_t)threshold + half_band) < S3_ADC_MAX))
    {
        __atomic_store_n(&g_calibration, (uint32_t)threshold
                         | ((uint32_t)band << 12U), __ATOMIC_RELEASE);
        /* No in-flight frame may combine classifications from two thresholds.
         * The retained letter/direction deliberately remains unchanged. */
        s3_barcode_reset();
    }
}

void s3_set_barcode_threshold(uint16_t threshold)
{
    s3_barcode_calibrate(threshold, S3_HYSTERESIS);
}

void barcode_task(INT stacd, void * p_exinf)
{
    adc_sample_t samples[S3_ADC_BATCH_LEN] = {{0U, 0U}};
    uint8_t count = 0U;
    uint8_t index = 0U;
    uint32_t started_us = 0U;
    uint32_t elapsed_us = 0U;
    uint8_t ticket = 0U;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)app_wait_period();
        started_us = adc_sampler_now_us();
        ticket = __atomic_load_n(&g_reset_ticket, __ATOMIC_ACQUIRE);
        if (!s3_barcode_active() || (g_seen_ticket != ticket))
        {
            g_seen_ticket = ticket;
            g_capture = 0U;
            g_candidate = 0U;
            g_last_sample_us = adc_sampler_now_us();
        }
        else
        {
            count = adc_sampler_since(g_last_sample_us, samples,
                                      S3_ADC_BATCH_LEN);
            /* Reject stale backlog BEFORE feeding it: feeding first can
             * publish an old completed barcode before the age check below. */
            if ((count > 0U)
                && ((adc_sampler_now_us() - samples[count - 1U].timestamp_us)
                    > S3_MAX_SAMPLE_GAP_US))
            {
                g_capture = 0U;
                g_candidate = 0U;
                g_last_sample_us = adc_sampler_now_us();
            }
            else
            {
                for (index = 0U; index < count; index++)
                {
                    process_sample(&samples[index]);
                }
            }
            if ((adc_sampler_now_us() - g_last_sample_us)
                > S3_MAX_SAMPLE_GAP_US)
            {
                /* No samples must not leave an unfinished frame alive. */
                g_capture = 0U;
                g_candidate = 0U;
                g_last_sample_us = adc_sampler_now_us();
            }
        }
        elapsed_us = adc_sampler_now_us() - started_us;
        if (elapsed_us > g_max_cycle_us)
        {
            __atomic_store_n(&g_max_cycle_us,
                (uint16_t)((elapsed_us > 65535U) ? 65535U : elapsed_us),
                __ATOMIC_RELAXED);
        }
    }
}

uint16_t s3_barcode_max_us(void)
{
    return (__atomic_load_n(&g_max_cycle_us, __ATOMIC_RELAXED));
}
