/*
 * imu_terrain.c
 *
 * Subsystem 4 -- IMU task: calibration, hump sequence, collision, tilt.
 *
 * Runs every 10 ms with a 4 ms worst-case budget (NFR11).  Each cycle it
 * DRAINS the accelerometer FIFO (about 4 samples at 400 Hz): every sample
 * is checked for an impact, so a jolt between ticks is not missed, and
 * the newest one gives the tilt.  Previously one register read per cycle
 * popped only the OLDEST of 32 queued samples, so the data was ~80 ms old
 * against NFR10's 25 ms.
 *
 * SINGLE WRITER: all private state below is written by imu_task alone.
 * The Controller's requests (activate, reset) arrive as flags that this
 * task applies at the top of its own cycle.
 */

#include "subsystem4.h"
#include "../fault.h"
#include "i2c0.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define S4_BYTES_PER_SAMPLE     (6u)

/* ---------------------------------------------------------------------
 * Private state
 * --------------------------------------------------------------------- */

/* Written by the Controller, read here. */
static volatile bool b_active        = false;
static volatile bool b_reset_request = false;

/* Read by other tasks: each is one byte, so each read is whole. */
static volatile s4_state_t state         = S4_INIT_CAL;
static volatile int8_t     tilt_deg      = 0;
static volatile uint8_t    hump_packed   = 0u;   /* msg_pack_hump()  */
static volatile bool       b_run_invalid = false;

static int16_t  offset_x = 0;
static int16_t  offset_y = 0;

static uint8_t  hump_debounce  = 0u;
static uint8_t  tilt_debounce  = 0u;
static int32_t  hump_height_um = 0;
static int32_t  last_distance_mm = 0;
static uint32_t hump_start_ms  = 0u;
static uint32_t peak_mm        = 0u;

static uint8_t  consecutive_fail = 0u;

/* Impact debounce: samples in a row over the threshold, carried across
   batches, and the strongest one seen in the current run of them. */
static uint8_t  impact_run    = 0u;
static int32_t  impact_peak_x = 0;
static int32_t  impact_peak_y = 0;

/*
 * sin(x) in Q8 for 0..45 degrees, one entry per degree.  Beyond 45 degrees
 * the car is in TILT_FAULT, so the table does not need to go further.
 */
static const uint8_t sin_q8[46] =
{
      0,   4,   9,  13,  18,  22,  27,  31,  36,  40,
     44,  49,  53,  58,  62,  66,  71,  75,  79,  83,
     87,  92,  96, 100, 104, 108, 112, 116, 120, 124,
    128, 132, 136, 139, 143, 147, 150, 154, 158, 161,
    165, 168, 171, 175, 178, 181
};

/* ---------------------------------------------------------------------
 * Private functions
 * --------------------------------------------------------------------- */

/* Raw left-justified 16-bit reading to milli-g at +/-8 g full scale. */
static int32_t to_milli_g(int32_t raw)
{
    return ((raw * S4_FULL_SCALE_MG) / 32768);
}

/* Little-endian 16-bit sample from a byte buffer. */
static int16_t get_axis(const uint8_t *p_raw)
{
    /* The cast reinterprets the two's complement 16-bit pattern. */
    return ((int16_t)((uint16_t)p_raw[0] | ((uint16_t)p_raw[1] << 8u)));
}

/*
 * Read up to max_samples queued FIFO samples in ONE burst.  Returns the
 * number read (0 if the FIFO was empty), or a negative error.
 */
static INT read_fifo(uint8_t *p_buf, uint8_t max_samples)
{
    uint8_t src = 0u;
    uint8_t count;
    ER      ercd;
    INT     result;

    ercd = i2c0_read_reg(S4_ACCEL_ADDR, S4_REG_FIFO_SRC, &src);
    if (ercd < E_OK)
    {
        result = (INT)ercd;
    }
    else
    {
        count = (uint8_t)((uint32_t)src & S4_FIFO_SRC_FSS_MASK);
        if (count > max_samples)
        {
            count = max_samples;
        }

        result = (INT)count;

        if (0u != count)
        {
            /* With the FIFO on, auto-increment wraps from OUT_Z_H back to
               OUT_X_L and on to the next sample, so one burst reads them
               all. */
            ercd = i2c0_read_burst(S4_ACCEL_ADDR,
                                   (uint8_t)(S4_REG_OUT_X_L | S4_AUTO_INC),
                                   p_buf,
                                   (uint8_t)(count * S4_BYTES_PER_SAMPLE));
            if (ercd < E_OK)
            {
                result = (INT)ercd;
            }
        }
    }

    return (result);
}

/*
 * INIT_CAL: average S4_CAL_SAMPLES readings with the car held still.  If
 * the readings move too much the car was not still: retry, up to twice.
 */
static bool calibrate(void)
{
    uint8_t  buf[S4_BYTES_PER_SAMPLE];
    int32_t  sum_x;
    int32_t  sum_y;
    int32_t  min_x;
    int32_t  max_x;
    int32_t  axis_x;
    uint32_t count;
    uint32_t attempt;
    bool     b_ok = false;

    for (attempt = 0u; (attempt <= S4_CAL_RETRIES) && !b_ok; attempt++)
    {
        sum_x = 0;
        sum_y = 0;
        min_x = 32767;
        max_x = -32768;

        for (count = 0u; count < S4_CAL_SAMPLES; count++)
        {
            (void)tk_dly_tsk(5);
            if (read_fifo(buf, 1u) <= 0)
            {
                break;
            }
            axis_x = (int32_t)get_axis(&buf[0]);
            sum_x += axis_x;
            sum_y += (int32_t)get_axis(&buf[2]);
            min_x  = (axis_x < min_x) ? axis_x : min_x;
            max_x  = (axis_x > max_x) ? axis_x : max_x;
        }

        if ((S4_CAL_SAMPLES == count)
            && (to_milli_g(max_x - min_x) <= S4_CAL_MAX_SPREAD_MG))
        {
            offset_x = (int16_t)(sum_x / (int32_t)S4_CAL_SAMPLES);
            offset_y = (int16_t)(sum_y / (int32_t)S4_CAL_SAMPLES);
            b_ok     = true;
        }
    }

    return (b_ok);
}

/*
 * Close out a hump and decide whether to believe it.
 */
static void finish_hump(void)
{
    int32_t  height_mm = hump_height_um / 1000;
    uint32_t duration  = app_now_ms() - hump_start_ms;

    /* Validate before reporting: an implausible height or duration is
       DISCARDED rather than published as a wrong value. */
    if ((height_mm > 0) && (height_mm < 200)
        && (duration < S4_HUMP_TIMEOUT_MS))
    {
        if ((uint32_t)height_mm > peak_mm)
        {
            peak_mm     = (uint32_t)height_mm;
            hump_packed = msg_pack_hump(peak_mm);   /* saturates + flags */
        }

        s5_post_event(S5_EVT_HUMP);
    }

    hump_height_um = 0;
}

/*
 * The hump sequence: nose-up, crest, nose-down, level -- in that order.
 */
static void step_hump_machine(int32_t travelled_mm)
{
    switch (state)
    {
        case S4_IDLE:
        case S4_LEVEL_RUN:
            state = S4_LEVEL_RUN;
            if (tilt_deg >= S4_HUMP_START_DEG)
            {
                hump_debounce = (uint8_t)(hump_debounce + 1u);
                if (hump_debounce >= S4_DEBOUNCE_SAMPLES)
                {
                    state          = S4_HUMP_RISING;
                    hump_height_um = 0;
                    hump_start_ms  = app_now_ms();
                    hump_debounce  = 0u;
                }
            }
            else
            {
                hump_debounce = 0u;
            }
        break;

        case S4_HUMP_RISING:
            hump_height_um += s4_height_increment_um(tilt_deg, travelled_mm);
            if (tilt_deg < S4_HUMP_LEVEL_DEG)
            {
                state = S4_HUMP_CREST;
            }
        break;

        case S4_HUMP_CREST:
            if (tilt_deg <= -S4_HUMP_START_DEG)
            {
                state = S4_HUMP_FALLING;
            }
            else if ((app_now_ms() - hump_start_ms) > S4_HUMP_TIMEOUT_MS)
            {
                hump_height_um = 0;     /* never came down: not a hump */
                state          = S4_LEVEL_RUN;
            }
            else
            {
                /* Still on the crest. */
            }
        break;

        case S4_HUMP_FALLING:
            if ((tilt_deg > -S4_HUMP_LEVEL_DEG)
                && (tilt_deg < S4_HUMP_LEVEL_DEG))
            {
                finish_hump();
                state = S4_LEVEL_RUN;
            }
        break;

        default:
            /* COLLISION and TILT_FAULT latch until a reset request; the
               other states are handled by the caller. */
        break;
    }
}

/*
 * Apply the Controller's reset request, in this task's own context.
 */
static void apply_reset_request(void)
{
    if (b_reset_request)
    {
        b_reset_request = false;
        peak_mm         = 0u;
        hump_packed     = 0u;
        b_run_invalid   = (S4_IMU_FAULT == state);
        hump_height_um  = 0;
        hump_debounce   = 0u;
        tilt_debounce   = 0u;
        last_distance_mm = s2_get_distance_mm();

        if ((S4_COLLISION == state) || (S4_TILT_FAULT == state))
        {
            state = S4_IDLE;
        }
    }
}

/*
 * Count an I2C failure; after S4_IMU_FAIL_LIMIT in a row enter the
 * defined degraded behaviour.  IMU_FAULT is DEGRADE, not E-STOP: the car
 * does not brake, but this run's hump data is marked invalid.
 */
static void note_failure(void)
{
    if (consecutive_fail < S4_IMU_FAIL_LIMIT)
    {
        consecutive_fail = (uint8_t)(consecutive_fail + 1u);
    }
    else if (S4_IMU_FAULT != state)
    {
        fault_set(FAULT_IMU);
        b_run_invalid = true;
        state         = S4_IMU_FAULT;
    }
    else
    {
        /* Already reported. */
    }
}

/*
 * Pitch from the newest sample in a batch.  Kept up to date whenever the
 * sensor answers -- parked, latched or driving -- so telemetry always
 * shows the car's real attitude rather than a value frozen at the moment
 * a fault latched.
 */
static void update_tilt(const uint8_t *p_buf, uint8_t count)
{
    uint32_t base = ((uint32_t)count - 1u) * S4_BYTES_PER_SAMPLE;
    int32_t  ax_mg;
    int32_t  az_mg;

    ax_mg = to_milli_g((int32_t)get_axis(&p_buf[base]) - (int32_t)offset_x);
    az_mg = to_milli_g((int32_t)get_axis(&p_buf[base + 4u]));

    tilt_deg = s4_pitch_deg(ax_mg, az_mg);
}

/*
 * Impact test on every sample of a batch.  One sample over the threshold
 * is NOT an impact: motor and gearbox vibration, or a corrupted I2C read
 * while the motors draw current, produces single-sample spikes.  A real
 * collision decelerates the car for several milliseconds, so it takes
 * S4_IMPACT_SAMPLES consecutive samples over the threshold.
 */
static bool detect_impact(const uint8_t *p_buf, uint8_t count)
{
    const int32_t limit_sq = S4_IMPACT_MILLI_G * S4_IMPACT_MILLI_G;
    int32_t ax_mg;
    int32_t ay_mg;
    int32_t mag_sq;
    uint8_t idx;
    bool    b_impact = false;

    for (idx = 0u; idx < count; idx++)
    {
        ax_mg = to_milli_g((int32_t)get_axis(&p_buf[idx * 6u])
                           - (int32_t)offset_x);
        ay_mg = to_milli_g((int32_t)get_axis(&p_buf[(idx * 6u) + 2u])
                           - (int32_t)offset_y);

        /* Horizontal jolt magnitude, squared (no square root needed). */
        mag_sq = (ax_mg * ax_mg) + (ay_mg * ay_mg);

        if (mag_sq > limit_sq)
        {
            if (0u == impact_run)
            {
                impact_peak_x = ax_mg;
                impact_peak_y = ay_mg;
            }
            else if (mag_sq > ((impact_peak_x * impact_peak_x)
                               + (impact_peak_y * impact_peak_y)))
            {
                impact_peak_x = ax_mg;
                impact_peak_y = ay_mg;
            }
            else
            {
                /* Weaker than the peak so far. */
            }

            if (impact_run < 0xFFu)
            {
                impact_run = (uint8_t)(impact_run + 1u);
            }
            if (impact_run >= S4_IMPACT_SAMPLES)
            {
                b_impact = true;
            }
        }
        else
        {
            impact_run = 0u;
        }
    }

    return (b_impact);
}

/*
 * Process one drained batch while active: impact check on every sample,
 * then the tilt-fault and hump logic on the (already updated) tilt.
 */
static void process_batch(const uint8_t *p_buf, uint8_t count)
{
    int32_t distance_now;
    bool    b_driving;

    /* The motion gate separates an impact from the car being picked up
       or knocked at rest (assumption A8): only while driving. */
    b_driving = (M_LINE_PID == s2_get_state()) || (M_MOVE == s2_get_state());

    if (!b_driving)
    {
        impact_run = 0u;
    }

    if (b_driving && detect_impact(p_buf, count))
    {
        fault_set(FAULT_COLLISION);
        state      = S4_COLLISION;
        impact_run = 0u;
        tm_printf((UB*)"[S4] COLLISION: x %d mg  y %d mg (%d samples)\n",
                  (INT)impact_peak_x, (INT)impact_peak_y,
                  (INT)S4_IMPACT_SAMPLES);
    }
    else if ((tilt_deg > S4_TILT_FAULT_DEG) || (tilt_deg < -S4_TILT_FAULT_DEG))
    {
        tilt_debounce = (uint8_t)(tilt_debounce + 1u);
        if (tilt_debounce > S4_DEBOUNCE_SAMPLES)
        {
            fault_set(FAULT_TILT);
            state = S4_TILT_FAULT;
            tm_printf((UB*)"[S4] TILT: %d deg\n", (INT)tilt_deg);
        }
    }
    else
    {
        tilt_debounce = 0u;
        distance_now  = s2_get_distance_mm();
        step_hump_machine(distance_now - last_distance_mm);
        last_distance_mm = distance_now;
    }
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT int32_t s4_height_increment_um(int8_t tilt, int32_t distance_mm)
{
    int32_t magnitude = (tilt < 0) ? -(int32_t)tilt : (int32_t)tilt;

    if (magnitude > 45)
    {
        magnitude = 45;
    }

    /* distance in mm x sin -> micrometres, keeping integer precision. */
    return ((distance_mm * (int32_t)sin_q8[magnitude] * 1000) / 256);
}

/*
 * Pitch from the accelerometer alone (no gyroscope: corrupted by linear
 * acceleration, assumption A7).  atan(t) ~= t / (1 + 0.28 t^2) for |t| <= 1
 * and pi/2 - atan(1/t) beyond; error under 0.3 degrees, integer maths.
 */
EXPORT int8_t s4_pitch_deg(int32_t ax_mg, int32_t az_mg)
{
    int32_t num  = (ax_mg < 0) ? -ax_mg : ax_mg;
    int32_t den  = (az_mg < 0) ? -az_mg : az_mg;
    int32_t ratio_m;
    int32_t atan_mrad;
    bool    b_invert = false;

    if (num > den)
    {
        int32_t swap = num;     /* use atan(1/t) for |t| > 1 */

        num      = den;
        den      = swap;
        b_invert = true;
    }

    ratio_m   = (0 == den) ? 0 : ((num * 1000) / den);           /* 0..1000 */
    atan_mrad = (ratio_m * 1000)
              / (1000 + ((280 * ratio_m * ratio_m) / 1000000));

    if (b_invert)
    {
        atan_mrad = 1571 - atan_mrad;
    }

    atan_mrad = (ax_mg < 0) ? -atan_mrad : atan_mrad;

    /* milliradians to degrees (x 180 / pi / 1000), rounded to nearest */
    return ((int8_t)(((atan_mrad * 573) + ((atan_mrad < 0) ? -5000 : 5000))
                     / 10000));
}

EXPORT void imu_task(INT stacd, void *p_exinf)
{
    uint8_t buf[S4_FIFO_MAX_READ * S4_BYTES_PER_SAMPLE];
    INT     count;

    (void)stacd;
    (void)p_exinf;

    state = S4_INIT_CAL;
    if (calibrate())
    {
        state = S4_IDLE;
    }
    else
    {
        fault_set(FAULT_IMU);
        b_run_invalid = true;
        state         = S4_IMU_FAULT;
    }

    last_distance_mm = s2_get_distance_mm();

    for (;;)
    {
        (void)app_wait_period();

        apply_reset_request();

        count = read_fifo(buf, S4_FIFO_MAX_READ);

        if (count < 0)
        {
            note_failure();
        }
        else
        {
            consecutive_fail = 0u;

            if (count > 0)
            {
                update_tilt(buf, (uint8_t)count);
            }

            if (S4_IMU_FAULT == state)
            {
                /* The sensor came back.  The run stays marked invalid:
                   the data already missed is gone. */
                state = S4_LEVEL_RUN;
            }

            if (!b_active)
            {
                state = ((S4_COLLISION == state) || (S4_TILT_FAULT == state))
                      ? state : S4_IDLE;
                last_distance_mm = s2_get_distance_mm();
            }
            else if ((count > 0) && (S4_COLLISION != state)
                     && (S4_TILT_FAULT != state))
            {
                process_batch(buf, (uint8_t)count);
            }
            else
            {
                /* Nothing new, or latched until a reset request. */
            }
        }
    }
}

EXPORT uint8_t s4_get_hump_packed(void)
{
    return (hump_packed);
}

EXPORT int8_t s4_get_tilt_deg(void)
{
    return (tilt_deg);
}

EXPORT s4_state_t s4_get_state(void)
{
    return (state);
}

EXPORT bool s4_run_data_invalid(void)
{
    return (b_run_invalid);
}

EXPORT void s4_set_active(bool b_on)
{
    b_active = b_on;
}

EXPORT void s4_request_reset(void)
{
    b_reset_request = true;
}

/* end of file */
