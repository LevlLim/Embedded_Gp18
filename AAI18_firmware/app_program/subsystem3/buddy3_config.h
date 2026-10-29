/**
 * @file buddy3_config.h
 * @brief Measured limits and provisional tuning for Buddy 3.
 */
#ifndef BUDDY3_CONFIG_H
#define BUDDY3_CONFIG_H

/* Enabled only by the separate CMake barcode-bench preset. */
#ifndef S3_BARCODE_BENCH
#define S3_BARCODE_BENCH        (0)
#endif

#define S3_ADC_MAX              (4095U)
/* Measured on this car: airborne 4047..4095, track black below ~3840. */
#define S3_AIR_LEVEL            (3950U)
#define S3_ADC_TIMEOUT_POLLS    (1024U)
#define S3_SAMPLE_STALE_US      (15000U)
#define S3_ADC_RING_LEN         (32U)
#define S3_ADC_BATCH_LEN        (8U)
#define S3_CAL_SAMPLES          (150U)
#define S3_CAL_INTERVAL_MS      (20U)
#define S3_CAL_MIN_SAMPLES      (120U)
#define S3_MIN_CONTRAST         (400U) /* measured noise rejection */
#define S3_HYSTERESIS           (200U)
#define S3_MIN_BAND             (20U)

/* Reused on every boot/reset; no sweep is required to become READY.
 * User-selected existing defaults, 29 October 2026. Higher ADC means black.
 * Manual s3_calibrate() remains available for an explicit maintenance sweep.
 */
#define S3_LEFT_THRESHOLD       (2047U)
#define S3_RIGHT_THRESHOLD      (2047U)
#define S3_BARCODE_THRESHOLD    (2047U)
#define S3_FIXED_BAND           (S3_HYSTERESIS)

#if (S3_FIXED_BAND < S3_MIN_BAND) || (S3_FIXED_BAND > 255U)
#error "Fixed hysteresis must fit the calibration record"
#endif
#if (S3_LEFT_THRESHOLD <= (S3_FIXED_BAND / 2U)) \
 || ((S3_LEFT_THRESHOLD + S3_FIXED_BAND / 2U) >= S3_AIR_LEVEL) \
 || (S3_RIGHT_THRESHOLD <= (S3_FIXED_BAND / 2U)) \
 || ((S3_RIGHT_THRESHOLD + S3_FIXED_BAND / 2U) >= S3_AIR_LEVEL) \
 || (S3_BARCODE_THRESHOLD <= (S3_FIXED_BAND / 2U)) \
 || ((S3_BARCODE_THRESHOLD + S3_FIXED_BAND / 2U) >= S3_AIR_LEVEL)
#error "Fixed thresholds and hysteresis must remain within usable ADC values"
#endif
/* GP26 hugs the LEFT edge; GP27 hugs the RIGHT edge. Both white is aligned.
 * A black reading still means a higher ADC value after sweep calibration. */
#define S3_LINE_CORRECTION      (50)
#define S3_JUNCTION_CONFIRM_SAMPLES (3U)

#if (S3_LINE_CORRECTION <= 0) || (S3_LINE_CORRECTION > 63)
#error "Line correction must fit the signed line-error range (1..63)"
#endif
#if (S3_JUNCTION_CONFIRM_SAMPLES < 1U) || (S3_JUNCTION_CONFIRM_SAMPLES > 255U)
#error "Junction confirmation must fit a nonzero uint8_t counter"
#endif

/* A frame is *letter*: three 9-element symbols and two white gaps. */
#define S3_CHAR_ELEMENTS        (9U)
#define S3_FRAME_ELEMENTS       (29U)
#define S3_MIN_BAR_US           (2000U)
#if S3_BARCODE_BENCH
#define S3_MAX_BAR_US           (1000000U)
#define S3_MAX_FRAME_US         (20000000U)
#else
#define S3_MAX_BAR_US           (60000U)
#define S3_MAX_FRAME_US         (2000000U)
#endif
#define S3_MIN_QUIET_US         (10000U)
#define S3_QUIET_NARROW_MULT    (10U)
#define S3_MAX_SAMPLE_GAP_US    (3000U)
#define S3_BARCODE_QUEUE_DEPTH  (4U)
#define S3_BARCODE_WIRE_BYTES   (2U)
#define S3_LETTER_COUNT         (26U)

#endif /* BUDDY3_CONFIG_H */

