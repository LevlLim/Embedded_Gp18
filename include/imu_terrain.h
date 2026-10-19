/*
 * imu_terrain.h - Buddy 4 (2500853) IMU / terrain subsystem
 * Physics Watchdog: hump estimation, tilt, collision detection.
 *
 * Requirements touched: FR4, FR5 (hump height to +/-5 mm), FR11,
 * NFR10, NFR11 (task must finish inside 4 ms).
 *
 * Change Requirement 1 decisions recorded in Tribunal 1:
 *   - hump height uses 7 bits with a 1 mm step. FR5 allows +/-5 mm, so the
 *     quantisation step has to stay well under that or packing eats the
 *     accuracy budget. 1 mm is a fifth of the tolerance.
 *   - tilt and peak acceleration are packed as offset integers, not floats.
 *   - the raw 400 Hz FIFO samples stay int16 because the sensor already
 *     outputs 16 bit and the samples are thrown away each cycle. Repacking
 *     them costs CPU and saves no lasting RAM.
 *   - tilt x encoder-distance maths stays 32 bit. Only the result is packed.
 */
#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include <stdint.h>

/* ---- field ranges and widths (see docs/buddy4_data_representation.md) ---- */

#define HUMP_MM_MIN         0
#define HUMP_MM_MAX         127         /* 7 bits, 1 mm per step            */
#define HUMP_MM_BITS        7

#define TILT_HALFDEG_MIN    (-90)       /* -45.0 deg                        */
#define TILT_HALFDEG_MAX    90          /* +45.0 deg, 0.5 deg per step      */
#define TILT_OFFSET         90          /* stored as value + 90 -> 0..180   */
#define TILT_BITS           8

#define PEAK_ACC_MIN        0           /* 0.00 g                           */
#define PEAK_ACC_MAX        160         /* 8.00 g at 0.05 g per step        */
#define PEAK_ACC_BITS       8

#define TERRAIN_EVT_BITS    3           /* up to 8 event types              */
#define TERRAIN_TS_BITS     20          /* ms, wraps at ~17.5 min.          */
                                        /* 16 bits would wrap at 65 s and   */
                                        /* our test runs are 60 s and up.   */
#define TERRAIN_SEQ_BITS    4           /* drop detection on the receiver   */

/* ---- event types (3 bits) ---- */
typedef enum {
    TERRAIN_EVT_NONE        = 0,
    TERRAIN_EVT_HUMP_RISING = 1,
    TERRAIN_EVT_HUMP_DONE   = 2,
    TERRAIN_EVT_COLLISION   = 3,
    TERRAIN_EVT_TILT        = 4,
    TERRAIN_EVT_IMU_FAULT   = 5
} terrain_evt_t;

/*
 * Packed inter-task message, IMU/Terrain -> Vehicle Controller.
 * 8 bytes. The unpacked struct this replaces was 28 bytes
 * (see the before/after figures in docs/buddy4_data_representation.md).
 *
 * w0:  [2:0]   event type        3 bits
 *      [3]     valid             1 bit
 *      [10:4]  hump height mm    7 bits
 *      [18:11] tilt + 90         8 bits
 *      [26:19] peak accel        8 bits
 *      [31:27] reserved          5 bits
 *
 * w1:  [19:0]  timestamp ms     20 bits
 *      [23:20] sequence          4 bits
 *      [31:24] reserved          8 bits
 */
typedef struct {
    uint32_t w0;
    uint32_t w1;
} terrain_msg_t;

/* Unpacked view, used inside a task and by the tests. Never sent on a queue. */
typedef struct {
    terrain_evt_t evt;
    uint8_t  valid;
    int32_t  hump_mm;          /* millimetres                     */
    int32_t  tilt_halfdeg;     /* half degrees, -90..+90          */
    int32_t  peak_acc_cg;      /* 0.05 g steps, 0..160            */
    uint32_t t_ms;
    uint8_t  seq;
} terrain_sample_t;

/* ---- packing ---- */

/*
 * Pack a sample into the wire/queue form. Values outside their range are
 * clamped, not wrapped, and *clipped is incremented once per clamped field
 * so the test plan can report how often this happens (Risk R3).
 */
terrain_msg_t terrain_pack(const terrain_sample_t *in, uint16_t *clipped);

/* Reverse of terrain_pack. Lossless for any in-range sample. */
void terrain_unpack(terrain_msg_t msg, terrain_sample_t *out);

/* ---- hump estimation ---- */

/*
 * Hump height from encoder distance and tilt, the Week 6 method, unchanged.
 * Only the representation of the result changed.
 *
 *   height_mm ~= distance_mm * sin(tilt)
 *
 * The multiply is done in int32 on purpose. Packing the intermediate would
 * truncate it. Returns millimetres, not yet clamped.
 */
int32_t terrain_hump_height_mm(int32_t distance_mm, int32_t tilt_halfdeg);

/* ---- task entry ---- */

/* Initialise state. Call once before the task starts. */
void imu_terrain_init(void);

/*
 * One cycle of the task: drain the FIFO, update tilt, decide events, publish.
 * Separated from the kernel loop so it can be timed and unit tested.
 * Returns the number of events published this cycle.
 */
int imu_terrain_step(uint32_t now_ms);

/* Diagnostics for the test plan. */
uint16_t imu_terrain_clip_count(void);
uint32_t imu_terrain_last_cycle_us(void);

#endif /* IMU_TERRAIN_H */
