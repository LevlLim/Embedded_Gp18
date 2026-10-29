/**
 * @file barcode_decoder.h
 * @brief Single-letter Code 39 decoding and retained result access.
 */
#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H
#include "../app.h"
#include "../msg_defs.h"
#include "buddy3_config.h"

/* The driving build retains compact 16-bit microseconds. The opt-in hand
 * scanner uses 32 bits to represent slow bars without truncating durations.
 */
#if S3_BARCODE_BENCH
typedef uint32_t s3_bar_width_t;
#else
typedef uint16_t s3_bar_width_t;
#endif

/* Unpacked API view; persistent/queued results occupy 13 bits in 2 bytes. */
typedef struct
{
    uint8_t letter;
    msg_barcode_cmd_t direction;
    uint8_t confidence;
    uint8_t sequence;
    bool b_reverse;
} s3_barcode_result_t;

/** Pure bounded decoder. Output is unchanged on failure. */
bool s3_decode_code39(s3_bar_width_t const * p_widths, uint8_t count,
                     uint32_t leading_white_us, uint32_t trailing_white_us,
                     s3_barcode_result_t * p_result);

/** Atomic retained snapshot; false until the first valid barcode. */
bool s3_read_barcode(s3_barcode_result_t * p_result);

/** Consume the newest notification; retained state is not consumed. */
bool s3_take_barcode_result(s3_barcode_result_t * p_result);

/** Compatibility interface for the unchanged Vehicle Controller. */
bool s3_take_barcode(msg_barcode_t * p_barcode);
uint8_t s3_get_last_barcode(void);
/** Call from a low-priority reporting task, never from the 1 ms task. */
void s3_report_barcode(void);
uint16_t s3_barcode_max_us(void);

ER s3_barcode_init(void);
void barcode_task(INT stacd, void * p_exinf);
void s3_barcode_reset(void);
/** Accepted calibration updates discard in-flight capture and queued notices;
 * the retained latest valid letter/direction pair survives. */
void s3_barcode_calibrate(uint16_t threshold, uint16_t band);
void s3_set_barcode_threshold(uint16_t threshold);

#endif /* BARCODE_DECODER_H */

