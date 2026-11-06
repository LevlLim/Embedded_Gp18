/*
 * barcode_decoder.c
 *
 * Subsystem 3 -- the 1 ms barcode scan and decode.
 *
 * METHOD
 * ------
 * Every 1 ms this task drains the ADC FIFO and walks the new samples
 * looking for threshold crossings on ADC2 (GP28).  A crossing opens or
 * closes an element (bar or space).  Its width is the DISTANCE the car
 * travelled between the two crossings (wheel encoders, 0.1 mm), not the
 * time: the car changes speed while steering, and a 3 mm bar at the
 * cruise speed lasts ~20 ms while an 18 mm space lasts ~125 ms, too wide
 * a range for a fixed time window.  A white gap longer than
 * S3_BC_QUIET_X10 ends the symbol, which is then decoded.
 *
 * SYMBOLOGY: Code 39, "*X*" -- start '*', one character, stop '*'
 * (29 elements: 3 x 9 plus two inter-character gaps).  Each character is
 * 5 bars and 4 spaces, exactly 3 of them wide (3:1 on the track sheets,
 * LONG 6/18 mm and SHORT 3/9 mm).  The three widest elements of each
 * character are taken as the wide ones, so the decode does not depend on
 * the bar size.  A barcode read BACKWARDS gives "P?P" (a reversed '*' is
 * 'P'), so a failed forward decode is retried reversed.
 */

#include "subsystem3.h"
#include "adc_sampler.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem2/motor_control.h"
#include "../drivers/gpio_irq.h"

/* Defaults; any of these may be set in subsystem3.h instead. */
#ifndef S3_BC_QUIET_X10
#define S3_BC_QUIET_X10     (500)   /* 50 mm of white ends a symbol      */
#endif
/* FIXED THRESHOLD for the barcode IR (measured on the car: white 120-320,
   black 1500-3600).  0 = use the value from calibration instead. */
#ifndef S3_BC_FIXED_THRESHOLD
#define S3_BC_FIXED_THRESHOLD   (700u)
#endif
/* BENCH TEST: 1 = read barcodes while parked too (fwd command, or pushing
   the car by hand over a sheet).  0 = only while RUNNING (normal). */
#ifndef S3_BC_READ_WHEN_PARKED
#define S3_BC_READ_WHEN_PARKED  (1)
#endif
#ifndef S3_BC_HYST              /* hysteresis band around it         */
#define S3_BC_HYST              (200u)
#endif
#ifndef S3_BC_MIN_X10
#define S3_BC_MIN_X10       (10)    /* under 1 mm: noise, ignored        */
#endif
/* Which letter means which command at the next junction.  Others are
   decoded and reported but give no command. */
#ifndef S3_BC_LETTER_LEFT
#define S3_BC_LETTER_LEFT       ('L')
#endif
#ifndef S3_BC_LETTER_RIGHT
#define S3_BC_LETTER_RIGHT      ('R')
#endif
#ifndef S3_BC_LETTER_STRAIGHT
#define S3_BC_LETTER_STRAIGHT   ('S')
#endif
#ifndef S3_BC_LETTER_UTURN
#define S3_BC_LETTER_UTURN      ('U')
#endif

#define BC_MAX_ELEMENTS     (40u)   /* *X* is 29; room for stray edges   */
#define BC_SYMBOL_ELEMENTS  (29u)
#define BC_CHAR_ELEMENTS    (9u)

/* Public, declared here so an older subsystem3.h still builds. */
bool s3_code39_decode(const uint16_t *p_widths, uint8_t count,
                      char *p_letter, bool *pb_reversed,
                      uint8_t *p_confidence);
msg_barcode_cmd_t s3_letter_command(char letter);
char s3_get_last_letter(void);

#define S3_BARCODE_QUEUE_DEPTH  (4u)
#define S3_BATCH_LEN            (8u)

/* ---------------------------------------------------------------------
 * Private state.  All written by barcode_task alone, except
 * threshold_barcode (one 16-bit store from calibration).
 * --------------------------------------------------------------------- */

static ID h_mbf_barcode = (ID)0;

static volatile uint16_t threshold_barcode =
    (0u != S3_BC_FIXED_THRESHOLD) ? (uint16_t)S3_BC_FIXED_THRESHOLD
                                  : (uint16_t)(S3_ADC_MAX / 2u);
static volatile uint8_t  last_barcode      = (uint8_t)MSG_BC_NONE;

static volatile uint8_t  last_letter      = 0u;   /* '?' style, 0 = none */

static bool     b_prev_dark    = false;
static int32_t  last_cross_pos = 0;     /* 0.1 mm                        */
static uint32_t last_seen_us   = 0u;
static bool     b_in_symbol    = false;

static uint16_t widths[BC_MAX_ELEMENTS];   /* distance, 0.1 mm        */
static uint16_t widths_t[BC_MAX_ELEMENTS]; /* time, 0.1 ms (fallback)  */
static uint32_t last_cross_us = 0u;
static uint8_t  width_count = 0u;
static uint8_t  barcode_seq = 0u;

/* ---------------------------------------------------------------------
 * Private functions
 * --------------------------------------------------------------------- */

/*
 * Close the current symbol: classify it, publish it if valid, reset.
 */
static void dump_widths(void);
static void dump_list(const uint16_t *p_w, const char *p_label);

static void finish_symbol(void)
{
    msg_barcode_t     message;
    msg_barcode_cmd_t decoded = MSG_BC_INVALID;
    uint8_t           confidence = 0u;
    uint8_t           buf[MSG_LEN_BARCODE];
    uint8_t           len;
    char              letter = '\0';
    bool              b_reversed = false;
    bool              b_timed = false;
    bool              b_ok = false;

    if ((width_count >= 5u) && (width_count < BC_SYMBOL_ELEMENTS))
    {
        /* Too few edges for "*X*" (29): bars were missed.  Reported so a
           bench test shows something; short runs (a line crossing) are
           not worth a line. */
        APP_LOG((UB*)"[S3] barcode not readable (%d elements, need 29)\n",
                (INT)width_count);
        dump_widths();
    }
    else if (width_count >= BC_SYMBOL_ELEMENTS)
    {
        b_timed = false;
        if (!s3_code39_decode(widths, width_count, &letter, &b_reversed,
                              &confidence))
        {
            /* Distance failed (encoder noise, wheel slip): try the same
               edges timed -- fine while the speed is steady. */
            b_timed = s3_code39_decode(widths_t, width_count, &letter,
                                       &b_reversed, &confidence);
            b_ok    = b_timed;
        }
        else
        {
            b_ok = true;
        }

        if (b_ok)
        {
            decoded     = s3_letter_command(letter);
            last_letter = (uint8_t)letter;
            APP_LOG((UB*)"[S3] barcode '%c'%s%s conf %d -> %s\n",
                    (INT)letter, b_reversed ? " (read backwards)" : "",
                    b_timed ? " (timed)" : "", (INT)confidence,
                    (MSG_BC_LEFT == decoded) ? "LEFT"
                    : ((MSG_BC_RIGHT == decoded) ? "RIGHT"
                    : ((MSG_BC_STRAIGHT == decoded) ? "STRAIGHT"
                    : ((MSG_BC_U_TURN == decoded) ? "U-TURN"
                    : "no command"))));
        }
        else
        {
            APP_LOG((UB*)"[S3] barcode not readable (%d elements)\n",
                    (INT)width_count);
            dump_widths();
        }

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

/* Print the widths of an unreadable symbol, in mm, so a bench run shows
   what the sensor saw (Code 39 LONG: 6 / 18 mm, SHORT: 3 / 9 mm). */
static void dump_list(const uint16_t *p_w, const char *p_label)
{
    UB      line[APP_LOG_MAX];
    uint8_t idx;
    uint8_t pos = 0u;
    INT     n;

    line[0] = (UB)'\0';
    for (idx = 0u; (idx < width_count) && (pos < (APP_LOG_MAX - 8u)); idx++)
    {
        n = tm_sprintf(&line[pos], (UB*)"%d ", (INT)((p_w[idx] + 5u) / 10u));
        pos = (n > 0) ? (uint8_t)(pos + (uint8_t)n) : pos;
    }
    APP_LOG((UB*)"[S3] widths %s (bar first): %s\n", p_label, line);
}

static void dump_widths(void)
{
    dump_list(widths, "mm");
    dump_list(widths_t, "ms");
}

/* Distance travelled, 0.1 mm, from the LEFT wheel only: the right
   encoder picks up motor noise on this car (bench: ~30 % extra counts
   with the motor running), which made the bar widths random. */
static int32_t travel_x10(void)
{
    gpio_irq_encoders_t enc = {0};

    gpio_irq_read_encoders(&enc);

    return (motor_counts_to_mm(enc.count_left * 10, true));
}

/*
 * Feed one sample through the crossing detector.
 */
static void process_sample(const adc_sample_t *p_sample)
{
    int32_t  pos;
    int32_t  width;
    uint32_t dt;
    uint32_t thr   = (uint32_t)threshold_barcode;
    uint32_t half  = S3_BC_HYST / 2u;
    bool     b_dark;

    /* Schmitt trigger: noise around the threshold must not split a bar. */
    if ((uint32_t)p_sample->value > (thr + half))
    {
        b_dark = true;
    }
    else if ((uint32_t)p_sample->value < ((thr > half) ? (thr - half) : 0u))
    {
        b_dark = false;
    }
    else
    {
        b_dark = b_prev_dark;
    }

    last_seen_us = p_sample->timestamp_us;
    pos          = travel_x10();
    width        = pos - last_cross_pos;
    width        = (width < 0) ? -width : width;

    if (b_dark != b_prev_dark)
    {
        /* Threshold crossing: the previous element just ended.  A symbol
           only starts on white -> black, so element 0 is always a bar. */
        if (b_in_symbol)
        {
            if ((width >= S3_BC_MIN_X10) && (width_count < BC_MAX_ELEMENTS))
            {
                dt = (p_sample->timestamp_us - last_cross_us) / 100u;
                widths[width_count]   = (uint16_t)((width > 0xFFFF) ? 0xFFFF
                                                                    : width);
                widths_t[width_count] = (uint16_t)((dt > 0xFFFFu) ? 0xFFFFu
                                                                  : dt);
                width_count           = (uint8_t)(width_count + 1u);
            }
        }
        else if (b_dark)
        {
            b_in_symbol = true;
            width_count = 0u;
        }
        else
        {
            /* Black -> white outside a symbol: not a start. */
        }

        last_cross_pos = pos;
        last_cross_us  = p_sample->timestamp_us;
        b_prev_dark    = b_dark;
    }
    else if (b_in_symbol && (width > S3_BC_QUIET_X10))
    {
        /* No edge for S3_BC_QUIET_X10: the symbol has ended.  White after
           the last bar is the normal case; a long BLACK stretch (the car
           ran off the sheet onto dark floor, or the sensor sees nothing
           under it) ends it too -- no Code 39 element is that wide.  The
           white margin before it was recorded as one more element, and
           the decoder finds "*X*" at the start of the list. */
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

/* Code 39: the nine elements of each character, bar first, wide = 1
   (most significant bit = first element). */
typedef struct
{
    uint16_t pattern;
    char     letter;
} code39_t;

static const code39_t code39[44] =
{
    { 0x034u, '0' }, { 0x121u, '1' }, { 0x061u, '2' }, { 0x160u, '3' },
    { 0x031u, '4' }, { 0x130u, '5' }, { 0x070u, '6' }, { 0x025u, '7' },
    { 0x124u, '8' }, { 0x064u, '9' }, { 0x109u, 'A' }, { 0x049u, 'B' },
    { 0x148u, 'C' }, { 0x019u, 'D' }, { 0x118u, 'E' }, { 0x058u, 'F' },
    { 0x00Du, 'G' }, { 0x10Cu, 'H' }, { 0x04Cu, 'I' }, { 0x01Cu, 'J' },
    { 0x103u, 'K' }, { 0x043u, 'L' }, { 0x142u, 'M' }, { 0x013u, 'N' },
    { 0x112u, 'O' }, { 0x052u, 'P' }, { 0x007u, 'Q' }, { 0x106u, 'R' },
    { 0x046u, 'S' }, { 0x016u, 'T' }, { 0x181u, 'U' }, { 0x0C1u, 'V' },
    { 0x1C0u, 'W' }, { 0x091u, 'X' }, { 0x190u, 'Y' }, { 0x0D0u, 'Z' },
    { 0x085u, '-' }, { 0x184u, '.' }, { 0x0C4u, ' ' }, { 0x094u, '*' },
    { 0x0A8u, '$' }, { 0x0A2u, '/' }, { 0x08Au, '+' }, { 0x02Au, '%' }
};

/* Mark the n widest of the given element indices as wide. */
static void pick_widest(const uint16_t *p_e, const uint8_t *p_idx,
                        uint8_t n_idx, uint8_t n_wide, bool *p_wide)
{
    uint8_t pick;
    uint8_t k;
    uint8_t best;

    for (pick = 0u; pick < n_wide; pick++)
    {
        best = 0xFFu;
        for (k = 0u; k < n_idx; k++)
        {
            if (!p_wide[p_idx[k]]
                && ((0xFFu == best) || (p_e[p_idx[k]] > p_e[best])))
            {
                best = p_idx[k];
            }
        }
        p_wide[best] = true;
    }
}

/* Separation of one element type: narrowest wide / widest narrow, x256.
   0xFFFF if that type has no wide element (nothing to separate). */
static uint32_t separation(const uint16_t *p_e, const uint8_t *p_idx,
                           uint8_t n_idx, const bool *p_wide)
{
    uint32_t min_wide = 0xFFFFu;
    uint32_t max_narr = 0u;
    uint8_t  k;
    uint32_t sep = 0xFFFFu;

    for (k = 0u; k < n_idx; k++)
    {
        if (p_wide[p_idx[k]])
        {
            min_wide = (p_e[p_idx[k]] < min_wide) ? p_e[p_idx[k]] : min_wide;
        }
        else
        {
            max_narr = (p_e[p_idx[k]] > max_narr) ? p_e[p_idx[k]] : max_narr;
        }
    }

    if ((0xFFFFu != min_wide) && (0u != max_narr))
    {
        sep = (min_wide * 256u) / max_narr;
    }

    return (sep);
}

/*
 * One character from nine widths (reading order, bar first).  BARS and
 * SPACES are judged separately: the sensor's spot blurs every edge the
 * same way, so bars all read wider and spaces narrower (or the reverse)
 * and a wide space can come out no wider than a narrow bar.  Every Code
 * 39 character has 2 wide bars + 1 wide space ($ / + %: 0 + 3), so the
 * widest bars are compared with bars and the widest space with spaces.
 * *p_sep_q8 = the weaker of the two separations, x256.
 */
static char code39_char(const uint16_t *p_e, uint32_t *p_sep_q8)
{
    static const uint8_t bars[5]   = { 0u, 2u, 4u, 6u, 8u };
    static const uint8_t spaces[4] = { 1u, 3u, 5u, 7u };
    uint16_t pattern;
    uint32_t sep_b;
    uint32_t sep_s;
    uint8_t  idx;
    uint8_t  pass;
    bool     b_wide[BC_CHAR_ELEMENTS];
    char     letter = '\0';

    *p_sep_q8 = 0u;

    for (pass = 0u; ('\0' == letter) && (pass < 2u); pass++)
    {
        for (idx = 0u; idx < BC_CHAR_ELEMENTS; idx++)
        {
            b_wide[idx] = false;
        }
        /* Pass 0: letters, digits, - . space *.  Pass 1: $ / + %. */
        pick_widest(p_e, bars, 5u, (0u == pass) ? 2u : 0u, b_wide);
        pick_widest(p_e, spaces, 4u, (0u == pass) ? 1u : 3u, b_wide);

        pattern = 0u;
        for (idx = 0u; idx < BC_CHAR_ELEMENTS; idx++)
        {
            pattern = (uint16_t)(((uint32_t)pattern << 1u)
                                 | (b_wide[idx] ? 1u : 0u));
        }

        for (idx = 0u; idx < 44u; idx++)
        {
            if (code39[idx].pattern == pattern)
            {
                letter = code39[idx].letter;
            }
        }

        sep_b     = separation(p_e, bars, 5u, b_wide);
        sep_s     = separation(p_e, spaces, 4u, b_wide);
        *p_sep_q8 = (sep_b < sep_s) ? sep_b : sep_s;
    }

    return (letter);
}

/* Decode "*X*" from 29 widths starting at p_w, in either direction. */
static bool code39_symbol(const uint16_t *p_w, bool b_reverse, char *p_letter,
                          uint32_t *p_sep_q8)
{
    uint16_t e[BC_SYMBOL_ELEMENTS];
    uint32_t sep[3];
    char     ch[3];
    uint8_t  idx;
    uint8_t  c;

    for (idx = 0u; idx < BC_SYMBOL_ELEMENTS; idx++)
    {
        e[idx] = b_reverse ? p_w[BC_SYMBOL_ELEMENTS - 1u - idx] : p_w[idx];
    }

    for (c = 0u; c < 3u; c++)
    {
        ch[c] = code39_char(&e[c * (BC_CHAR_ELEMENTS + 1u)], &sep[c]);
    }

    *p_letter = ch[1];
    *p_sep_q8 = sep[0];
    *p_sep_q8 = (sep[1] < *p_sep_q8) ? sep[1] : *p_sep_q8;
    *p_sep_q8 = (sep[2] < *p_sep_q8) ? sep[2] : *p_sep_q8;

    return (('*' == ch[0]) && ('*' == ch[2]) && ('*' != ch[1])
            && ('\0' != ch[1]) && (*p_sep_q8 >= 333u));  /* >= 1.3 : 1 */
}

/*
 * Find "*X*" in a width sequence.  Pure function (host-tested with widths
 * measured off the track sheets).  Every start is tried, odd ones too:
 * on the car an extra dark element (sheet edge, shadow) came before the
 * first bar, so "*X*" began at element 1 (bench, 6 Nov).
 */
EXPORT bool s3_code39_decode(const uint16_t *p_widths, uint8_t count,
                             char *p_letter, bool *pb_reversed,
                             uint8_t *p_confidence)
{
    uint8_t  start;
    uint32_t sep_q8 = 0u;
    char     letter = '\0';
    bool     b_found = false;
    bool     b_rev   = false;

    if ((NULL != p_widths) && (count >= BC_SYMBOL_ELEMENTS))
    {
        for (start = 0u;
             (!b_found) && ((start + BC_SYMBOL_ELEMENTS) <= count);
             start = (uint8_t)(start + 1u))
        {
            b_found = code39_symbol(&p_widths[start], false, &letter,
                                    &sep_q8);
            if (!b_found)
            {
                b_found = code39_symbol(&p_widths[start], true, &letter,
                                        &sep_q8);
                b_rev   = b_found;
            }
        }
    }

    if (NULL != p_letter)
    {
        *p_letter = b_found ? letter : '\0';
    }
    if (NULL != pb_reversed)
    {
        *pb_reversed = b_rev;
    }
    if (NULL != p_confidence)
    {
        /* 7: wide clearly 2x+ the narrow; 5: just over 1.5x. */
        *p_confidence = (!b_found) ? 0u : ((sep_q8 >= 512u) ? 7u : 5u);
    }

    return (b_found);
}

/* Letter -> junction command (S3_BC_LETTER_* in subsystem3.h). */
EXPORT msg_barcode_cmd_t s3_letter_command(char letter)
{
    msg_barcode_cmd_t cmd = MSG_BC_NONE;

    if (S3_BC_LETTER_LEFT == letter)
    {
        cmd = MSG_BC_LEFT;
    }
    else if (S3_BC_LETTER_RIGHT == letter)
    {
        cmd = MSG_BC_RIGHT;
    }
    else if (S3_BC_LETTER_STRAIGHT == letter)
    {
        cmd = MSG_BC_STRAIGHT;
    }
    else if (S3_BC_LETTER_UTURN == letter)
    {
        cmd = MSG_BC_U_TURN;
    }
    else
    {
        /* Decoded, but not a command letter. */
    }

    return (cmd);
}

/* Kept for the host test's older interface: widths -> command. */
EXPORT msg_barcode_cmd_t s3_decode_widths(const uint16_t *p_widths,
                                          uint8_t count,
                                          uint8_t *p_confidence)
{
    char letter = '\0';
    bool b_rev  = false;
    msg_barcode_cmd_t cmd = MSG_BC_INVALID;

    if (s3_code39_decode(p_widths, count, &letter, &b_rev, p_confidence))
    {
        cmd = s3_letter_command(letter);
    }

    return (cmd);
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

        if ((s3_barcode_active() || (1 == S3_BC_READ_WHEN_PARKED))
            && !s4_barcode_lifted())
        {
            got = adc_sampler_since(last_seen_us, batch, S3_BATCH_LEN);

            for (idx = 0u; idx < got; idx++)
            {
                process_sample(&batch[idx]);
            }
        }
        else
        {
            /* Idle, or the sensor is lifted clear of a ramp: keep the
               read position at "now", so resuming does not replay old
               samples as a phantom symbol. */
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

EXPORT char s3_get_last_letter(void)
{
    return ((char)last_letter);
}

EXPORT uint8_t s3_get_last_barcode(void)
{
    return (last_barcode);
}

EXPORT uint16_t s3_get_barcode_threshold(void)
{
    return (threshold_barcode);
}

EXPORT void s3_set_barcode_threshold(uint16_t threshold)
{
    /* Calibration's value is used only without a fixed threshold. */
    threshold_barcode = (0u != S3_BC_FIXED_THRESHOLD)
                      ? (uint16_t)S3_BC_FIXED_THRESHOLD : threshold;
}

/* end of file */