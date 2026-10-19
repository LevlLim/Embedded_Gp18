/*
 * imu_terrain.c - Buddy 4 (2500853) IMU / terrain task
 *
 * Cycle: drain the accelerometer FIFO (400 Hz, read every 10 ms), update the
 * tilt estimate, decide whether a hump or a collision happened, pack the
 * result and send it to the Vehicle Controller.
 *
 * Timing: the whole cycle has to finish inside 4 ms (NFR11). The packing
 * added by Change Requirement 1 is a handful of shifts and masks per event,
 * not per raw sample, which is why the raw FIFO data is left as int16.
 */
#include <string.h>
#include "imu_terrain.h"
#include "app_bits.h"
#include "fault_flags.h"

/* ------------------------------------------------------------------ */
/* Tuning values. All provisional until measured on the real car.      */
/* ------------------------------------------------------------------ */
#define FIFO_SAMPLES_PER_CYCLE   4      /* 400 Hz drained every 10 ms   */
#define TILT_RISING_HALFDEG      6      /* 3.0 deg starts a hump        */
#define TILT_FLAT_HALFDEG        2      /* 1.0 deg counts as flat       */
#define N_DEB                    3      /* consecutive samples to debounce */
#define IMPACT_THRESHOLD_CG      40     /* 2.00 g in 0.05 g steps (A8)  */
#define FIFO_TIMEOUT_CYCLES      5      /* no data for 50 ms -> fault   */

/* ------------------------------------------------------------------ */
/* Hardware / neighbour hooks. Replaced by the real drivers on target. */
/* ------------------------------------------------------------------ */
int32_t  imu_hal_read_fifo(int16_t *ax, int16_t *ay, int16_t *az, int32_t max);
int32_t  encoder_distance_mm(void);          /* Buddy 2 provides this   */
void     terrain_publish(terrain_msg_t msg); /* mbf to Vehicle Controller */
uint32_t board_time_us(void);

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static struct {
    int32_t  tilt_halfdeg;
    int32_t  peak_acc_cg;
    int32_t  hump_start_mm;
    uint8_t  rising_count;
    uint8_t  flat_count;
    uint8_t  in_hump;
    uint8_t  seq;
    uint8_t  starved_cycles;
    uint16_t clipped;
    uint32_t last_cycle_us;
} s;

void imu_terrain_init(void)
{
    memset(&s, 0, sizeof(s));
}

uint16_t imu_terrain_clip_count(void)  { return s.clipped; }
uint32_t imu_terrain_last_cycle_us(void) { return s.last_cycle_us; }

/* ------------------------------------------------------------------ */
/* Packing                                                             */
/* ------------------------------------------------------------------ */
terrain_msg_t terrain_pack(const terrain_sample_t *in, uint16_t *clipped)
{
    terrain_msg_t m;

    uint32_t hump = bits_clamp_u(in->hump_mm, HUMP_MM_MIN, HUMP_MM_MAX, clipped);
    uint32_t tilt = bits_clamp_u(in->tilt_halfdeg, TILT_HALFDEG_MIN,
                                 TILT_HALFDEG_MAX, clipped) + TILT_OFFSET;
    uint32_t peak = bits_clamp_u(in->peak_acc_cg, PEAK_ACC_MIN,
                                 PEAK_ACC_MAX, clipped);

    m.w0 = BITS_PACK(in->evt,   0,  TERRAIN_EVT_BITS)
         | BITS_PACK(in->valid, 3,  1)
         | BITS_PACK(hump,      4,  HUMP_MM_BITS)
         | BITS_PACK(tilt,      11, TILT_BITS)
         | BITS_PACK(peak,      19, PEAK_ACC_BITS);

    m.w1 = BITS_PACK(in->t_ms, 0,  TERRAIN_TS_BITS)
         | BITS_PACK(in->seq,  20, TERRAIN_SEQ_BITS);

    return m;
}

void terrain_unpack(terrain_msg_t msg, terrain_sample_t *out)
{
    out->evt          = (terrain_evt_t)BITS_UNPACK(msg.w0, 0,  TERRAIN_EVT_BITS);
    out->valid        = (uint8_t)      BITS_UNPACK(msg.w0, 3,  1);
    out->hump_mm      = (int32_t)      BITS_UNPACK(msg.w0, 4,  HUMP_MM_BITS);
    out->tilt_halfdeg = (int32_t)      BITS_UNPACK(msg.w0, 11, TILT_BITS) - TILT_OFFSET;
    out->peak_acc_cg  = (int32_t)      BITS_UNPACK(msg.w0, 19, PEAK_ACC_BITS);
    out->t_ms         = (uint32_t)     BITS_UNPACK(msg.w1, 0,  TERRAIN_TS_BITS);
    out->seq          = (uint8_t)      BITS_UNPACK(msg.w1, 20, TERRAIN_SEQ_BITS);
}

/* ------------------------------------------------------------------ */
/* Hump height from encoder distance and tilt                          */
/*                                                                     */
/* height = distance * sin(tilt). Small angle, so sin(x) ~= x in       */
/* radians is close enough over our 0 to 45 degree range for the       */
/* +/-5 mm we promised. One half-degree step is 0.008727 rad, scaled   */
/* by 65536 that is 572.                                               */
/*                                                                     */
/* The multiply is deliberately int32. This is the intermediate the    */
/* team agreed must NOT be packed.                                     */
/* ------------------------------------------------------------------ */
int32_t terrain_hump_height_mm(int32_t distance_mm, int32_t tilt_halfdeg)
{
    int32_t sin_q16 = tilt_halfdeg * 572;        /* fits easily in int32 */
    int32_t h = (distance_mm * sin_q16) >> 16;
    return (h < 0) ? -h : h;
}

/* ------------------------------------------------------------------ */
/* One task cycle                                                      */
/* ------------------------------------------------------------------ */
static void publish(terrain_evt_t evt, int32_t hump_mm, uint32_t now_ms)
{
    terrain_sample_t smp;
    terrain_msg_t    msg;

    smp.evt          = evt;
    smp.valid        = 1;
    smp.hump_mm      = hump_mm;
    smp.tilt_halfdeg = s.tilt_halfdeg;
    smp.peak_acc_cg  = s.peak_acc_cg;
    smp.t_ms         = now_ms & BITMASK(TERRAIN_TS_BITS);
    smp.seq          = s.seq++ & BITMASK(TERRAIN_SEQ_BITS);

    msg = terrain_pack(&smp, &s.clipped);
    terrain_publish(msg);
}

int imu_terrain_step(uint32_t now_ms)
{
    int16_t ax[FIFO_SAMPLES_PER_CYCLE];
    int16_t ay[FIFO_SAMPLES_PER_CYCLE];
    int16_t az[FIFO_SAMPLES_PER_CYCLE];
    int32_t n, i, published = 0;
    uint32_t t0 = board_time_us();

    n = imu_hal_read_fifo(ax, ay, az, FIFO_SAMPLES_PER_CYCLE);

    /* No samples for several cycles means the sensor or the bus is gone.
     * Collision detection is blind at that point, so the Controller has to
     * know. Buddy 4 owns this bit, nobody else sets it. */
    if (n <= 0) {
        if (++s.starved_cycles >= FIFO_TIMEOUT_CYCLES) {
            fault_set(FLT_IMU_FAULT);
            publish(TERRAIN_EVT_IMU_FAULT, 0, now_ms);
            published++;
        }
        s.last_cycle_us = board_time_us() - t0;
        return published;
    }
    s.starved_cycles = 0;

    /* Averaging the block instead of trusting one sample. A single reading
     * is corrupted by the car's own acceleration. */
    {
        int32_t sum_x = 0, sum_z = 0, peak = 0;
        for (i = 0; i < n; i++) {
            int32_t mag;
            sum_x += ax[i];
            sum_z += az[i];
            mag = (ax[i] < 0 ? -ax[i] : ax[i]) + (ay[i] < 0 ? -ay[i] : ay[i])
                + (az[i] < 0 ? -az[i] : az[i]);
            if (mag > peak) peak = mag;
        }
        /* int32 all the way through, scaled down to the packed unit only at
         * the end. 16384 LSB per g on the +/-2 g range, 0.05 g per step. */
        s.peak_acc_cg  = (peak * 20) / 16384;
        s.tilt_halfdeg = ((sum_x / n) * 180) / 16384;   /* rough, see A7 */
    }

    /* Collision: a real impact exceeds 2 g on at least one sample (A8). */
    if (s.peak_acc_cg >= IMPACT_THRESHOLD_CG) {
        fault_set(FLT_COLLISION);
        publish(TERRAIN_EVT_COLLISION, 0, now_ms);
        published++;
    }

    /* Hump state machine, debounced so chassis vibration does not start one. */
    if (!s.in_hump) {
        if (s.tilt_halfdeg >= TILT_RISING_HALFDEG) {
            if (++s.rising_count >= N_DEB) {
                s.in_hump       = 1;
                s.rising_count  = 0;
                s.hump_start_mm = encoder_distance_mm();
                publish(TERRAIN_EVT_HUMP_RISING, 0, now_ms);
                published++;
            }
        } else {
            s.rising_count = 0;
        }
    } else {
        int32_t tilt_abs = s.tilt_halfdeg < 0 ? -s.tilt_halfdeg : s.tilt_halfdeg;
        if (tilt_abs <= TILT_FLAT_HALFDEG) {
            if (++s.flat_count >= N_DEB) {
                int32_t travelled = encoder_distance_mm() - s.hump_start_mm;
                int32_t height = terrain_hump_height_mm(travelled, TILT_RISING_HALFDEG);
                s.in_hump    = 0;
                s.flat_count = 0;
                publish(TERRAIN_EVT_HUMP_DONE, height, now_ms);
                published++;
            }
        } else {
            s.flat_count = 0;
        }
    }

    s.last_cycle_us = board_time_us() - t0;
    return published;
}
