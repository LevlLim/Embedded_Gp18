/*
 * barcode_decoder.c
 *
 * Subsystem 3 -- the 1 ms barcode scan and decode.
 *
 * METHOD (and why it is the only one)
 * -----------------------------------
 * Every 1 ms this task drains the ADC FIFO and walks the new samples
 * looking for threshold crossings on ADC2 (GP28).  A crossing opens or
 * closes an element (bar or space); its width is the difference between
 * the two crossing timestamps.  When no crossing has been seen for longer
 * than the quiet zone, the captured width sequence is classified.
 *
 * WIDTHS ARE RELATIVE.  Elements are classified narrow or wide against the
 * narrowest one in the same symbol, so the decode survives a change in
 * vehicle speed (assumption A1).
 *
 * CR1: element widths are uint16_t microseconds.  A width can never
 * exceed the quiet zone plus one sample gap (asserted in subsystem3.h),
 * so 16 bits always suffice; this halves the width buffer.
 */

#include "subsystem3.h"
#include "adc_sampler.h"
#include "../subsystem5/subsystem5.h"

#define S3_BARCODE_QUEUE_DEPTH  (4u)
#define S3_BATCH_LEN            (8u)

/* ---------------------------------------------------------------------
 * Private state.  All written by barcode_task alone, except
 * threshold_barcode (one 16-bit store from calibration).
 * --------------------------------------------------------------------- */

static ID h_mbf_barcode = (ID)0;

static volatile uint16_t threshold_barcode = (uint16_t)(S3_ADC_MAX / 2u);
static volatile uint8_t  last_barcode      = (uint8_t)MSG_BC_NONE;

static bool     b_prev_dark   = false;
static uint32_t last_cross_us = 0u;
static uint32_t last_seen_us  = 0u;
static bool     b_in_symbol   = false;

static uint16_t widths[S3_MAX_BARS];
static uint8_t  width_count = 0u;
static uint8_t  barcode_seq = 0u;

/* ---------------------------------------------------------------------
 * Private functions
 * --------------------------------------------------------------------- */

/*
 * Close the current symbol: classify it, publish it if valid, reset.
 */
static void finish_symbol(void)
{
    msg_barcode_t     message;
    msg_barcode_cmd_t decoded;
    uint8_t           confidence = 0u;
    uint8_t           buf[MSG_LEN_BARCODE];
    uint8_t           len;

    if (width_count >= 4u)
    {
        decoded = s3_decode_widths(widths, width_count, &confidence);

        /*
         * A failed decode is DISCARDED, never forwarded as a guess: a wrong
         * turn takes the car off the track, a missed one can be re-read.
         */
        if ((MSG_BC_INVALID != decoded) && (MSG_BC_NONE != decoded))
        {
            message.cmd        = decoded;
            message.confidence = confidence;
            message.seq        = (uint8_t)((uint32_t)barcode_seq & 0x03u);
            barcode_seq        = (uint8_t)(barcode_seq + 1u);

            len = msg_pack_barcode(&message, buf, (uint8_t)sizeof(buf));
            if ((0u != len)
                && (E_OK == tk_snd_mbf(h_mbf_barcode, buf, (INT)len,
                                       TMO_POL)))
            {
                last_barcode = (uint8_t)decoded;
                s5_post_event(S5_EVT_BARCODE);
            }
        }
    }

    width_count = 0u;
    b_in_symbol = false;
}

/*
 * Feed one sample through the crossing detector.
 */
static void process_sample(const adc_sample_t *p_sample)
{
    uint32_t width;
    bool     b_dark = ((uint32_t)p_sample->value > (uint32_t)threshold_barcode);

    last_seen_us = p_sample->timestamp_us;

    if (b_dark != b_prev_dark)
    {
        /* Threshold crossing: the previous element just ended. */
        if (b_in_symbol)
        {
            width = p_sample->timestamp_us - last_cross_us;

            if ((width >= S3_MIN_BAR_US) && (width_count < S3_MAX_BARS))
            {
                /* Bounded by the quiet zone, so it fits 16 bits. */
                widths[width_count] = (uint16_t)width;
                width_count         = (uint8_t)(width_count + 1u);
            }
        }

        b_in_symbol   = true;
        last_cross_us = p_sample->timestamp_us;
        b_prev_dark   = b_dark;
    }
    else if (b_in_symbol
             && ((p_sample->timestamp_us - last_cross_us) > S3_QUIET_ZONE_US))
    {
        /* Quiet zone: the symbol has ended. */
        finish_symbol();
    }
    else
    {
        /* Still inside the current element, or idle. */
    }
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT ER s3_barcode_init(void)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (INT)APP_MBF_BUFSZ(S3_BARCODE_QUEUE_DEPTH, MSG_LEN_BARCODE);
    cmbf.maxmsz = (INT)MSG_LEN_BARCODE;

    h_mbf_barcode = tk_cre_mbf(&cmbf);

    return ((h_mbf_barcode < E_OK) ? (ER)h_mbf_barcode : E_OK);
}

/*
 * Classify a width sequence into a command.
 *
 * Pure function: no hardware, no kernel objects, no globals -- the host
 * harness (testing/host) runs it directly with synthetic widths.
 *
 * PLACEHOLDER ENCODING (replace once the real track's symbols are
 * measured): the first four elements carry one bit each, narrow = 0,
 * wide = 1:  0b0001 STRAIGHT  0b0010 LEFT  0b0100 RIGHT  0b1000 U_TURN
 */
EXPORT msg_barcode_cmd_t s3_decode_widths(const uint16_t *p_widths,
                                          uint8_t count,
                                          uint8_t *p_confidence)
{
    uint32_t          narrowest  = 0xFFFFu;
    uint32_t          ratio_q8;
    uint32_t          pattern    = 0u;
    uint8_t           idx;
    uint8_t           wide_count = 0u;
    uint8_t           confidence = 0u;
    msg_barcode_cmd_t result     = MSG_BC_INVALID;

    if ((NULL != p_widths) && (count >= 4u))
    {
        /* Narrowest element in THIS symbol sets the reference. */
        for (idx = 0u; idx < count; idx++)
        {
            if ((0u != p_widths[idx]) && ((uint32_t)p_widths[idx] < narrowest))
            {
                narrowest = p_widths[idx];
            }
        }
    }

    /* All noise-width (or no input): the sample rate did not resolve the
       bars at this speed.  This is the A1 failure mode. */
    if ((NULL != p_widths) && (count >= 4u) && (narrowest >= S3_MIN_BAR_US)
        && (0xFFFFu != narrowest))
    {
        for (idx = 0u; idx < 4u; idx++)
        {
            ratio_q8 = ((uint32_t)p_widths[idx] * 256u) / narrowest;
            pattern  = pattern << 1u;

            if (ratio_q8 >= S3_WIDE_RATIO_Q8)
            {
                pattern    = pattern | 1u;
                wide_count = (uint8_t)(wide_count + 1u);
            }
        }

        switch (pattern)
        {
            case 0x01u:
                result = MSG_BC_STRAIGHT;
            break;

            case 0x02u:
                result = MSG_BC_LEFT;
            break;

            case 0x04u:
                result = MSG_BC_RIGHT;
            break;

            case 0x08u:
                result = MSG_BC_U_TURN;
            break;

            default:
                result = MSG_BC_INVALID;
            break;
        }

        /* Exactly one wide element in a valid pattern is the ideal. */
        if (MSG_BC_INVALID != result)
        {
            confidence = (1u == wide_count) ? 7u : 3u;
        }
    }

    if (NULL != p_confidence)
    {
        *p_confidence = confidence;
    }

    return (result);
}

EXPORT void barcode_task(INT stacd, void *p_exinf)
{
    adc_sample_t batch[S3_BATCH_LEN];
    uint8_t      got;
    uint8_t      idx;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        (void)app_wait_period();

        if (s3_barcode_active())
        {
            got = adc_sampler_since(last_seen_us, batch, S3_BATCH_LEN);

            for (idx = 0u; idx < got; idx++)
            {
                process_sample(&batch[idx]);
            }
        }
        else
        {
            /* Idle: keep the read position at "now", so activation does
               not replay old samples as a phantom symbol. */
            if (adc_sampler_read(ADC_CH_BARCODE, &batch[0]))
            {
                last_seen_us = batch[0].timestamp_us;
            }
            width_count = 0u;
            b_in_symbol = false;
        }
    }
}

EXPORT bool s3_take_barcode(msg_barcode_t *p_bc)
{
    uint8_t buf[MSG_LEN_BARCODE];
    INT     received;
    bool    b_ok = false;

    if (NULL != p_bc)
    {
        received = tk_rcv_mbf(h_mbf_barcode, buf, TMO_POL);
        b_ok     = (received >= (INT)MSG_LEN_BARCODE)
                   && msg_unpack_barcode(buf, MSG_LEN_BARCODE, p_bc);
    }

    return (b_ok);
}

EXPORT uint8_t s3_get_last_barcode(void)
{
    return (last_barcode);
}

EXPORT void s3_set_barcode_threshold(uint16_t threshold)
{
    threshold_barcode = threshold;
}

/* end of file */
