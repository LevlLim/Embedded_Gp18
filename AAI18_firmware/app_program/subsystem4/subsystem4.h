/*
 * subsystem4.h
 *
 * Subsystem 4 -- IMU calibration, terrain and collision.
 * Buddy 4 (Jian Yan), 2500853.
 *
 * Implements Statechart 4 of Design Review V2 section 6.4.
 *
 * A hump is a SEQUENCE, not a reading: the height is only valid if
 * nose-up, crest, nose-down and level occur in that order.  The same
 * vertical spike means terrain on a climb and a collision on flat ground,
 * so the meaning depends on the current state -- which is exactly why a
 * statechart rather than a set of if-statements.
 *
 * HEIGHT METHOD: h += sin(tilt) x wheel_distance, accumulated over the
 * distance the wheels actually travelled.  Double-integrating acceleration
 * was rejected in Week 6 because it drifts badly over an event this short.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../msg_defs.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C addresses.  Accelerometer and magnetometer, no gyroscope -- which
   is why tilt comes from the accelerometer alone and why FR5's +/-5 mm is
   still an open question (assumption A7). */
#define S4_ACCEL_ADDR           (0x19u)
#define S4_MAG_ADDR             (0x1Eu)

/* LIS3DH-style register map.  Confirm against the actual part on the
   bench before trusting: a wrong WHO_AM_I is the first thing the
   self-test checks. */
#define S4_REG_WHO_AM_I         (0x0Fu)
#define S4_REG_CTRL1            (0x20u)
#define S4_REG_CTRL4            (0x23u)
#define S4_REG_CTRL5            (0x24u)
#define S4_REG_FIFO_CTRL        (0x2Eu)
#define S4_REG_FIFO_SRC         (0x2Fu)
#define S4_REG_OUT_X_L          (0x28u)
#define S4_AUTO_INC             (0x80u)  /* OR into reg for burst read   */
#define S4_FIFO_SRC_FSS_MASK    (0x1Fu)  /* samples waiting in the FIFO  */

/* Register values written at init. */
#define S4_CTRL1_400HZ_XYZ      (0x77u)  /* ODR 400 Hz, X/Y/Z enabled    */
#define S4_CTRL4_8G_HR          (0x28u)  /* +/-8 g full scale, high res  */
#define S4_CTRL5_FIFO_EN        (0x40u)
#define S4_FIFO_CTRL_STREAM     (0x80u)

/* +/-8 g, NOT +/-2 g: at +/-2 g the axis saturates at 1999 mg, so the
   2000 mg impact threshold below could never be crossed (collision
   detection was unreachable). */
#define S4_FULL_SCALE_MG        (8000)
#define S4_FIFO_MAX_READ        (8u)     /* samples per cycle (4 expected)*/

/* Thresholds.  ALL PROVISIONAL -- assumptions A7 and A8 are open and
   these must be set from logged data, not guessed at. */
#define S4_IMPACT_MILLI_G       (2000)  /* 2 g horizontal jolt, A8       */
#define S4_IMPACT_SAMPLES       (3u)    /* in a row: 7.5 ms at 400 Hz    */
#define S4_TILT_FAULT_DEG       (30)    /* sustained lean = tipped over  */
#define S4_HUMP_START_DEG       (4)     /* nose-up that opens a hump     */
#define S4_HUMP_LEVEL_DEG       (2)     /* back to level                 */
#define S4_DEBOUNCE_SAMPLES     (3)     /* A7: n_deb                     */
#define S4_HUMP_TIMEOUT_MS      (4000u) /* a hump that never ends        */
#define S4_IMU_FAIL_LIMIT       (3u)    /* consecutive I2C errors        */
#define S4_CAL_SAMPLES          (64u)   /* averaged for the offsets      */
#define S4_CAL_MAX_SPREAD_MG    (60)    /* more movement = not held still*/
#define S4_CAL_RETRIES          (2u)

typedef enum
{
    S4_INIT_CAL     = 0,
    S4_IDLE         = 1,
    S4_LEVEL_RUN    = 2,
    S4_HUMP_RISING  = 3,
    S4_HUMP_CREST   = 4,
    S4_HUMP_FALLING = 5,
    S4_HUMP_DONE    = 6,
    S4_COLLISION    = 7,
    S4_TILT_FAULT   = 8,
    S4_IMU_FAULT    = 9
} s4_state_t;

/* Highest hump peak this run, PACKED (msg_pack_hump): bits 0..6 mm,
   bit 7 saturated.  One byte, so a reader on the other core can never see
   a peak from one hump with the SAT flag of another (FR5, rule R5). */
uint8_t    s4_get_hump_packed(void);
int8_t     s4_get_tilt_deg(void);
s4_state_t s4_get_state(void);

/* True when hump and peak reporting for this run is invalid, because
   IMU_FAULT occurred at some point.  Carried in telemetry. */
bool s4_run_data_invalid(void);

/* Begin / end active sensing.  In IDLE, handling the car triggers
   nothing. */
void s4_set_active(bool b_on);

/* Ask for the run data to be cleared and any latched COLLISION/TILT state
   released.  Called by the Controller on RESET or a new run.  The IMU
   task applies it at the start of its next cycle, so its private state
   is only ever written by the IMU task itself (rule R1). */
void s4_request_reset(void);

/* Pure helpers, exposed for the host test.  Integer maths, no FPU. */
int32_t s4_height_increment_um(int8_t tilt_deg, int32_t distance_mm);
int8_t  s4_pitch_deg(int32_t ax_mg, int32_t az_mg);

/* The task body.  Private to subsystem 4. */
void imu_task(INT stacd, void *p_exinf);

#endif /* SUBSYSTEM4_H */

/* end of file */
