/*
 * subsystem3.h
 *
 * Subsystem 3 -- IR line following and barcode decoding.
 * Buddy 3 (Zhi Kai), 2500466.
 *
 * Implements Statechart 3 of Design Review V2 section 6.3.
 *
 * THE SINGLE AUTHORITATIVE METHOD IS PERIODIC ADC SAMPLING.  Bars are
 * found as threshold crossings in the sample stream and their widths are
 * measured from sample timestamps.  There is no edge capture anywhere in
 * this subsystem, and no module here touches the ADC: subsystem3/adc_sampler
 * owns it.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../msg_defs.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------
 * Calibration and thresholds.  PROVISIONAL -- assumption A2 is still open
 * and these must be set from logged ADC values across the lit and shadowed
 * parts of the real track.
 * --------------------------------------------------------------------- */
/* SIDE SENSOR POSITION, seen from the back of the car.  The second line
   sensor (GP26, ADC0 -- "L" in the calibration printout) sits beside the
   centre one.  1 = on its RIGHT (this car), 0 = on its left.  Getting
   this wrong steers the car AWAY from the line. */
#define S3_SIDE_SENSOR_RIGHT    (1)
#define S3_SIDE_SIGN            ((1 == S3_SIDE_SENSOR_RIGHT) ? 1 : -1)

#define S3_ADC_MAX              (4095u)
/* NOTHING UNDER THE SENSOR.  Lifted off the surface, these IR sensors
   read close to full scale (measured 4047..4095), higher than the black
   line (measured up to ~3840).  Such a reading is "no surface": ignored
   by calibration (lifting the car while sweeping set the centre
   threshold to 2103, above the line itself) and treated as NOT the line
   while driving (a lifted car must count as off the line). */
#define S3_AIR_LEVEL            (3950u)
#define S3_CAL_SAMPLES          (150u)  /* x 20 ms = 3 s of sweeping     */
#define S3_CAL_INTERVAL_MS      (20u)
#define S3_CAL_TAIL_SAMPLES     (25u)   /* last 0.5 s: the start position  */
#define S3_HYSTERESIS           (200u)  /* PROVISIONAL: max full band     */
#define S3_MIN_CONTRAST         (400u)  /* PROVISIONAL: black - white     */

/* The two line sensors are NOT matched on this car (measured on the
   floor): centre ~145 white / ~1000+ black, side ~200 white / ~400 black.
   So each channel gets its own threshold AND its own hysteresis band
   (a quarter of its contrast, at most S3_HYSTERESIS), and a sweep counts
   for a channel once it spans S3_MIN_SWEEP_CONTRAST.  (Both sensors now
   give 1000+ counts of contrast; sitting still on black the centre
   reading wanders ~120, which must NOT count as a sweep.) */
#define S3_MIN_SWEEP_CONTRAST   (400u)  /* 120 took black NOISE for a sweep */
#define S3_MIN_BAND             (20u)   /* hysteresis never below this    */
/* Static calibration never sees the side sensor on black: its threshold
   is its white reading plus this.  Measured after the sensor was lowered:
   side ~150 white, ~1000-1700 black, so +400 sits well clear of white
   (a sensor just at the line's EDGE no longer reads as "on the line")
   and well below black.  A sweep during calibration is still better. */
#define S3_SIDE_STATIC_OFFSET   (400u)

/* EDGE STEERING on the side WITHOUT a sensor.  With the line under the
   centre sensor only, the centre reading falls from its full-black level
   as the sensor slides toward the line's edge.  If the side sensor is
   still white, the car is sliding toward the blind side, so steer back
   in proportion: up to S3_EDGE_MAX_ERROR (of 63) when the reading has
   fallen to the threshold.  Measured on the car: the line was always
   lost on the blind side with no warning before this. */
#define S3_EDGE_MAX_ERROR       (40)
#define S3_LOST_DEBOUNCE        (4u)    /* 4 x 5 ms = 20 ms of all-white  */

/* Barcode geometry.  Bars are classified by RELATIVE width against the
   narrowest bar in the symbol, so a change in vehicle speed does not
   invalidate the decode (assumption A1). */
#define S3_MAX_BARS             (24u)
#define S3_QUIET_ZONE_US        (12000u) /* PROVISIONAL: ends a symbol    */
#define S3_MIN_BAR_US           (600u)   /* PROVISIONAL: noise floor      */
#define S3_WIDE_RATIO_Q8        (512u)   /* wide if >= 2.0x the narrow    */

/* A width is at most the quiet zone plus one sample gap, so 16 bits of
   microseconds always suffice (CR1: was 32). */
MSG_STATIC_ASSERT((S3_QUIET_ZONE_US + 4000u) <= 0xFFFFu, bar_width_fits);

/*
 * Read the latest line reading as ONE packed word (msg_pack_line layout:
 * error, valid flag, 24-bit ms timestamp).  Returns false if nothing has
 * been published yet.  The CALLER must still check the age with
 * msg_line_age_ms(): true means a reading exists, not that it is fresh.
 * Safe from any task on either core: a single aligned load.
 */
bool s3_read_line_word(uint32_t *p_word);

/* Current line sub-state, for telemetry. */
msg_line_state_t s3_get_line_state(void);

/* Begin / end active sensing.  Before START the subsystem samples and
   keeps thresholds live but publishes no error and decodes no barcode. */
void s3_set_active(bool b_on);

/* Run the calibration pass (line AND barcode channels).  Blocks for
   S3_CAL_SAMPLES x S3_CAL_INTERVAL_MS (3 s); the car must be swept across
   the line while it runs.  Called by the Vehicle Controller. */
ER s3_calibrate(void);

/* Barcode capture gate, kept in step with s3_set_active(). */
bool s3_barcode_active(void);

/* Take the next decoded barcode command, if any (Vehicle Controller). */
bool s3_take_barcode(msg_barcode_t *p_bc);

/* Last successfully decoded command, for telemetry (msg_barcode_cmd_t). */
uint8_t s3_get_last_barcode(void);

/* Current line thresholds (side = GP26, centre = GP27), diagnostics. */
void s3_get_thresholds(uint16_t *p_side, uint16_t *p_centre);

/* Set the barcode channel threshold (from calibration). */
void s3_set_barcode_threshold(uint16_t threshold);

/* Private to subsystem 3. */
ER   s3_barcode_init(void);
void line_task(INT stacd, void *p_exinf);
void barcode_task(INT stacd, void *p_exinf);

/* Classify a captured width sequence.  Pure function, no hardware. */
msg_barcode_cmd_t s3_decode_widths(const uint16_t *p_widths, uint8_t count,
                                   uint8_t *p_confidence);

#endif /* SUBSYSTEM3_H */

/* end of file */
