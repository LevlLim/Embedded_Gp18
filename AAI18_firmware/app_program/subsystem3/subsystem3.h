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
/* SENSOR LAYOUT: the two line sensors STRADDLE the line, one just
   outside each edge, seen from the back of the car.  Both read WHITE when
   the car is centred; one reads black when the car drifts; BOTH read
   black at a junction (cross or T), which is where a barcode's turn is
   carried out.  0 = GP26 (ADC0) is the LEFT sensor and GP27 (ADC1) the
   RIGHT one (this car).  1 = the other way round. */
#define S3_SWAP_SIDES           (0)
#define S3_CH_LEFT              ((0 == S3_SWAP_SIDES) ? 0u : 1u)
#define S3_CH_RIGHT             ((0 == S3_SWAP_SIDES) ? 1u : 0u)

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

/* The two line sensors are NOT matched on this car, so each channel
   gets its own threshold and hysteresis band (a quarter of its contrast,
   at most S3_HYSTERESIS).  A channel counts as SWEPT once its readings
   span S3_MIN_SWEEP_CONTRAST: threshold = midpoint of its white and
   black.  A channel that never saw the line during calibration gets
   white + S3_STATIC_OFFSET (measured: white ~150, black 1000-1700). */
#define S3_MIN_SWEEP_CONTRAST   (400u)
#define S3_MIN_BAND             (20u)   /* hysteresis never below this    */
#define S3_STATIC_OFFSET        (400u)

/* STEERING.  One sensor on black: steer toward it, from S3_ERR_MIN (just
   over its threshold) to S3_ERR_MAX (full black: the line is right under
   it).  Error range is +-63.  These sensors SWITCH (white ~150 jumps to
   1000+ within a millimetre), so the reading just over the threshold is
   already "on the line": keep these gentle or the car overshoots. */
#define S3_ERR_MIN              (10)
#define S3_ERR_MAX              (20)

/* LOST.  With straddling sensors "both white" also happens when the line
   has slid right past one sensor and out the far side.  Taken as LOST
   when a sensor had been on black for at least S3_OUTWARD_MS before it
   went white (steering toward it had not brought the line back in: a
   sharp curve), so the car keeps turning that way until a sensor sees the
   line again.  Shorter dark spells are ordinary corrections. */
#define S3_OUTWARD_MS           (350u)

/* BOTH WHITE after a short touch: steer gently toward the sensor that saw
   the line last (S3_SEEK_ERROR, of 63) -- the car weaves slowly between
   the two sensors on a straight, but cannot drive off with the line
   outside them.  All white for S3_WHITE_LOST_MS: LOST, turn toward it. */
#define S3_SEEK_ERROR           (0)

/* CURVE BOOST: each repeated touch on the SAME side (a steady curve) adds
   S3_CURVE_STEP to the steering, up to S3_CURVE_MAX; a touch on the
   other side (a straight) clears it. */
#define S3_CURVE_STEP           (10)
#define S3_CURVE_MAX            (30)
#define S3_WHITE_LOST_MS        (600u)

/* JUNCTION: both sensors black for S3_JUNCTION_SAMPLES in a row (x 5 ms).
   A SHARP CORNER looks the same (both sensors cross the bend together), so
   while both are black the car keeps its previous steering IF one sensor
   had already been on black for S3_CORNER_MS (a corner); otherwise it
   was a crossing met at an angle, and the car goes straight across. */
#define S3_CORNER_MS            (200u)
/* Junction reporting: once per junction; re-armed after both have been
   white for S3_JUNCTION_CLEAR_SAMPLES.  A branch on ONE side only cannot
   be told apart from the car drifting, so it is not reported. */
#define S3_JUNCTION_SAMPLES     (3u)
#define S3_JUNCTION_CLEAR_SAMPLES (20u)
#define S3_LOST_DEBOUNCE        (4u)    /* 4 x 5 ms = 20 ms of LOST        */

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

/* Current line thresholds (left and right sensor), diagnostics. */
void s3_get_thresholds(uint16_t *p_left, uint16_t *p_right);

/* True while either sensor is on black.  With straddling sensors "TRACK"
   also means "both white", so code looking FOR the line (search turns,
   the bypass return leg) must use this instead. */
bool s3_line_seen(void);

/* Controller: a STRAIGHT barcode is waiting, so the next time both sensors
   go black go straight across, even if the car was turning (it would
   otherwise be taken as a corner).  Clears itself at that junction. */
void s3_force_straight(bool b_on);

/* Junctions seen since START (diagnostics). */
uint16_t s3_get_junction_count(void);

/* Barcode channel threshold (diagnostics). */
uint16_t s3_get_barcode_threshold(void);

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