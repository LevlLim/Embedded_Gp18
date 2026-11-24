/*
 * imu_terrain.c
 *
 * Subsystem 4 -- IMU task: calibration, hump sequence, tilt, lift stop.
 *
 * Runs every 10 ms with a 4 ms worst-case budget (NFR11).  Each cycle it
 * DRAINS the accelerometer FIFO (about 4 samples at 400 Hz): every sample
 * goes through the lift check, and the newest one gives the tilt.
 * Previously one register read per cycle popped only the OLDEST of 32
 * queued samples, so the data was ~80 ms old against NFR10's 25 ms.
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
static volatile int8_t     tilt_deg      = 0;   /* pitch: + nose up      */
static volatile int8_t     roll_deg      = 0;   /* roll: side to side    */
static int32_t  pitch_x8 = 0;                   /* filter state, x 8    */

/* Side ramp (roll) state.  Written by imu_task only. */
/* Last side ramp (CR1 item 6): ONE word, bits 0-7 = height mm
   (saturated at 255), bits 8-9 = side (0 none, 1 roll+, 2 roll-). */
#define S4_SIDE_MM_MAX          (255u)
#define S4_SIDE_MM_MASK         (0x00FFu)
#define S4_SIDE_UP_POS          (0x0100u)   /* roll+ side up */
#define S4_SIDE_UP_NEG          (0x0200u)   /* roll- side up */
static volatile uint16_t side_word      = 0u;
static bool     b_side_active  = false;
static bool     b_side_reported = false;
static uint8_t  side_debounce  = 0u;
static int32_t  side_peak_x10  = 0;   /* peak steady height, 0.1 mm   */
static int32_t  side_dir       = 0;
static int32_t  side_x16       = 0;   /* side accel, mg x 16 (filter) */
static int32_t  side_base_x16  = 0;   /* its level value              */
static volatile int32_t side_steady_x10 = 0;  /* steady height, 0.1 mm */
static bool     b_side_base_ok = false;
static uint32_t side_start_ms  = 0u;
static int32_t  roll_x8  = 0;
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

/* Calibration result.  Without offsets the tilt is wrong by however the
   chip sits, so tilt / lift E-STOPs are only raised once calibrated
   (bench log: an uncalibrated IMU raised TILT 37 deg on a flat floor). */
static bool     b_calibrated  = false;
static uint32_t cal_samples   = 0u;
static int32_t  cal_spread_mg = 0;
static int32_t  cal_wobble_mg = 0;           /* mean |deviation|        */
static int16_t  cal_x[S4_CAL_SAMPLES];      /* raw X samples, one pass */

/* Lift stop.  rest_z_mg: Z at calibration (car flat on the floor).
   lift_z_mg: smoothed Z.  lift_run: samples in a row off rest, carried
   across batches.  lift_peak_mg: largest smoothed offset this run,
   printed afterwards so S4_LIFT_MG can be set from real driving. */
#define S4_LIFT_SAMPLES   ((S4_LIFT_MS * 400u) / 1000u)  /* 400 Hz ODR */
static int32_t  rest_z_mg     = 1000;
static int32_t  lift_z_mg     = 0;
static bool     b_lift_primed = false;
static uint16_t lift_run      = 0u;
static int32_t  lift_peak_mg  = 0;
static bool     b_was_active  = false;

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
    int32_t  sum_x = 0;
    int32_t  sum_y = 0;
    int32_t  sum_z = 0;
    int32_t  min_x = 32767;
    int32_t  max_x = -32768;
    int32_t  axis_x;
    int32_t  mean_x;
    int32_t  dev_sum = 0;
    int32_t  dev;
    uint32_t count;
    uint32_t idx;
    bool     b_ok = false;

    for (count = 0u; count < S4_CAL_SAMPLES; count++)
    {
        (void)tk_dly_tsk(5);
        if (read_fifo(buf, 1u) <= 0)
        {
            break;
        }
        axis_x         = (int32_t)get_axis(&buf[0]);
        cal_x[count]   = (int16_t)axis_x;
        sum_x         += axis_x;
        sum_y         += (int32_t)get_axis(&buf[2]);
        sum_z         += (int32_t)get_axis(&buf[4]);
        min_x          = (axis_x < min_x) ? axis_x : min_x;
        max_x          = (axis_x > max_x) ? axis_x : max_x;
    }

    cal_samples   = count;
    cal_spread_mg = (0u == count) ? 0 : to_milli_g(max_x - min_x);
    cal_wobble_mg = 0;

    if (S4_CAL_SAMPLES == count)
    {
        /*
         * "Held still" is judged on the MEAN deviation from the average,
         * not the spread (max - min).  The spread is set by the single
         * worst sample: a buzzing servo or one noisy reading made it
         * 234 mg on a car that was not moving (bench log), so calibration
         * never passed.  Vibration averages out of the offsets anyway;
         * only real handling moves the mean deviation.
         */
        mean_x = sum_x / (int32_t)S4_CAL_SAMPLES;
        for (idx = 0u; idx < S4_CAL_SAMPLES; idx++)
        {
            dev      = (int32_t)cal_x[idx] - mean_x;
            dev_sum += (dev < 0) ? -dev : dev;
        }
        cal_wobble_mg = to_milli_g(dev_sum / (int32_t)S4_CAL_SAMPLES);

        if (cal_wobble_mg <= S4_CAL_MAX_WOBBLE_MG)
        {
            offset_x = (int16_t)mean_x;
            offset_y = (int16_t)(sum_y / (int32_t)S4_CAL_SAMPLES);
            rest_z_mg = to_milli_g(sum_z / (int32_t)S4_CAL_SAMPLES);
            b_ok     = true;
        }
    }

    return (b_ok);
}

/*
 * Boot calibration.  The car is usually still being handled right after
 * power-up (USB plugged in, car put on the line), and a single attempt
 * then fails on movement -- the bench logs showed IMU_FAULT on most
 * boots.  Keep trying for S4_CAL_WINDOW_MS (the Controller's own 3 s
 * line calibration also needs the car still, so it settles in that
 * window), and say why if it never succeeds.
 */
static bool calibrate_at_boot(void)
{
    uint32_t start_ms = app_now_ms();
    bool     b_ok     = false;

    while (!b_ok && ((app_now_ms() - start_ms) < S4_CAL_WINDOW_MS))
    {
        b_ok = calibrate();
        if (!b_ok)
        {
            (void)tk_dly_tsk(200);

            /* No data at all: the IMU was probably not configured (it
               did not answer at power-up).  Configure it again. */
            if (0u == cal_samples)
            {
                (void)s4_configure_imu(false);
            }
        }
    }

    if (b_ok)
    {
        APP_LOG((UB*)"[S4] IMU calibrated (wobble %d mg, spread %d mg)\n",
                (INT)cal_wobble_mg, (INT)cal_spread_mg);
    }
    else if (S4_CAL_SAMPLES != cal_samples)
    {
        if (!s4_configure_imu(false))
        {
            APP_LOG((UB*)"[S4] IMU CAL FAILED: IMU not answering on I2C "
                         "-- check SDA GP16, SCL GP17, VCC, GND\n");
        }
        else
        {
            APP_LOG((UB*)"[S4] IMU CAL FAILED: IMU answers but sends no "
                         "samples (%d of %d)\n",
                    (INT)cal_samples, (INT)S4_CAL_SAMPLES);
        }
    }
    else
    {
        APP_LOG((UB*)"[S4] IMU CAL FAILED: car moving (wobble %d mg, "
                     "limit %d; spread %d) -- keep it still at power-up\n",
                (INT)cal_wobble_mg, (INT)S4_CAL_MAX_WOBBLE_MG,
                (INT)cal_spread_mg);
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
            /* LIFTED and TILT_FAULT latch until a reset request; the
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

        b_lift_primed   = false;
        lift_run        = 0u;

        if ((S4_LIFTED == state) || (S4_TILT_FAULT == state))
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
    int32_t  ay_mg;
    int32_t  az_mg;
    int32_t  fwd_mg;
    int32_t  side_mg;

    ax_mg = to_milli_g((int32_t)get_axis(&p_buf[base]) - (int32_t)offset_x);
    ay_mg = to_milli_g((int32_t)get_axis(&p_buf[base + 2u])
                       - (int32_t)offset_y);
    az_mg = to_milli_g((int32_t)get_axis(&p_buf[base + 4u]));

    /* Same atan maths for both axes: pitch from X (front-back), roll
       from Y (side to side), each against Z.  Low-pass filtered (each
       10 ms step moves 1/S4_TILT_FILTER_DIV of the way): driving
       vibration shook the raw angles by +-25 deg (bench log: roll 29 on
       a flat floor), close to the 30 deg TILT fault. */
    /* IMU mounting (subsystem4.h): which chip axis points forward. */
    fwd_mg  = (1 == S4_IMU_SWAP_XY) ? ay_mg : ax_mg;
    side_mg = (1 == S4_IMU_SWAP_XY) ? ax_mg : ay_mg;
    fwd_mg  = (1 == S4_IMU_INVERT_PITCH) ? -fwd_mg : fwd_mg;
    side_mg = (1 == S4_IMU_INVERT_ROLL) ? -side_mg : side_mg;

    pitch_x8 += (((int32_t)s4_pitch_deg(fwd_mg, az_mg) * 8) - pitch_x8)
              / S4_TILT_FILTER_DIV;
    roll_x8  += (((int32_t)s4_pitch_deg(side_mg, az_mg) * 8) - roll_x8)
              / S4_TILT_FILTER_DIV;
    tilt_deg  = (int8_t)(pitch_x8 / 8);
    roll_deg  = (int8_t)(roll_x8 / 8);

    /* Side ramp: the side acceleration itself, kept in 1/16 mg. */
    side_x16 += ((side_mg * 16) - side_x16) / S4_TILT_FILTER_DIV;
}

/*
 * Height of the raised side in 0.1 mm for a change of side acceleration
 * (mg x 16) with one side on the floor: spacing x sin(roll), and
 * sin(roll) = change / 1000 mg.  Then the measured gain.
 */
static int32_t side_height_x10(int32_t change_x16)
{
    int32_t mag = (change_x16 < 0) ? -change_x16 : change_x16;

    mag = (mag > (1000 * 16)) ? (1000 * 16) : mag;       /* <= 1 g    */

    return ((S4_WHEEL_SPACING_MM * mag * S4_SIDE_GAIN_PCT)
            / (1600 * 100));
}

/* 0.1 mm to whole mm, rounded. */
static uint16_t side_mm(int32_t height_x10)
{
    return ((uint16_t)((height_x10 + 5) / 10));
}

/* Report the current side ramp on the console (and car18/log). */
static void report_side(const char *p_when)
{
    uint16_t height_mm = side_mm(side_peak_x10);

    if (height_mm < S4_SIDE_MIN_MM)
    {
        APP_LOG((UB*)"[S4] side ramp %s: %d.%d mm, under %d mm: ignored\n",
                p_when, (INT)(side_peak_x10 / 10),
                (INT)(side_peak_x10 % 10), (INT)S4_SIDE_MIN_MM);
    }
    else
    {
        side_word = (uint16_t)(((height_mm > S4_SIDE_MM_MAX)
                                ? S4_SIDE_MM_MAX : height_mm)
                               | ((side_dir > 0) ? S4_SIDE_UP_POS
                                  : ((side_dir < 0) ? S4_SIDE_UP_NEG
                                     : 0u)));

        APP_LOG((UB*)"[S4] side ramp %s: %s side up, %d.%d mm -> %d mm\n",
                p_when, (side_dir > 0) ? "roll+" : "roll-",
                (INT)(side_peak_x10 / 10), (INT)(side_peak_x10 % 10),
                (INT)height_mm);
    }
}

/*
 * One side ramp (left or right wheels raised): track the peak roll while
 * it lasts and report the raised side's height when the car is level
 * again (or once, if it is still raised after S4_SIDE_REPORT_MS).  Runs
 * whenever the IMU is calibrated, driving or parked, so it can also be
 * tried by lifting one side of the car by hand.
 */
/*
 * True while the car is being STEERED: the two wheels are driven in
 * opposite directions (a spin) or at clearly different duty (> 25 %
 * apart).  A turn pushes the chassis sideways (centripetal
 * acceleration), which the accelerometer cannot tell from a real roll,
 * so a side ramp is neither started nor measured then.  Judged on what
 * the wheels are TOLD to do, not on how fast they turn: a parked car
 * pushed or tilted by hand (duty 0) is never "turning", and a wheel that
 * slows on the climb does not cancel the measurement.
 */
static bool is_turning(void)
{
    int32_t duty_l = 0;
    int32_t duty_r = 0;
    int32_t mag_l;
    int32_t mag_r;
    int32_t diff;

    motor_get_applied(&duty_l, &duty_r);
    mag_l = (duty_l < 0) ? -duty_l : duty_l;
    mag_r = (duty_r < 0) ? -duty_r : duty_r;
    diff  = mag_l - mag_r;
    diff  = (diff < 0) ? -diff : diff;

    return (((duty_l < 0) && (duty_r > 0))           /* spin            */
            || ((duty_l > 0) && (duty_r < 0))
            || ((diff * 4) > (mag_l + mag_r)));      /* >25 % apart     */
}

static void step_side_ramp(void)
{
    static int32_t avg_x16 = 0;       /* steady change, mg x 16        */
    int32_t        steady_x10;
    int32_t        dir;
    bool           b_turn = is_turning();

    if (!b_calibrated)
    {
        b_side_active   = false;
        b_side_base_ok  = false;
        side_debounce   = 0u;
        side_steady_x10 = 0;
    }
    else
    {
        if (!b_side_base_ok)
        {
            side_base_x16  = side_x16;      /* start from where it is */
            avg_x16        = 0;
            b_side_base_ok = true;
        }

        avg_x16 += ((side_x16 - side_base_x16) - avg_x16) / S4_SIDE_HOLD_DIV;
        steady_x10      = side_height_x10(avg_x16);
        side_steady_x10 = steady_x10;
        dir             = (avg_x16 < 0) ? -1 : 1;

        if (!b_side_active)
        {
            side_debounce = ((steady_x10 >= S4_SIDE_START_MM_X10) && !b_turn)
                          ? (uint8_t)(side_debounce + 1u) : 0u;

            if (side_debounce >= (uint8_t)S4_SIDE_DEBOUNCE)
            {
                b_side_active   = true;
                b_side_reported = false;
                side_debounce   = 0u;
                side_peak_x10   = 0;
                side_dir        = dir;
                side_start_ms   = app_now_ms();
                APP_LOG((UB*)"[S4] side ramp start (%s)\n",
                        (dir > 0) ? "roll+" : "roll-");
            }
            else if (steady_x10 < S4_SIDE_LEVEL_MM_X10)
            {
                /* Level: follow the car's own level value slowly, so
                   only a CHANGE counts as height.  Not while it rises
                   towards the start level: that would eat the height. */
                side_base_x16 += (side_x16 - side_base_x16)
                               / S4_SIDE_BASE_DIV;
            }
            else
            {
                /* Maybe starting: hold the level value. */
            }
        }
        else
        {
            /* Peak only once the car has settled on the ramp: a knock
               on the way up (wheel hitting the edge, a hand placing
               the car) is still in the average for ~100 ms. */
            if ((dir == side_dir) && (steady_x10 > side_peak_x10) && !b_turn
                && ((app_now_ms() - side_start_ms) >= S4_SIDE_SETTLE_MS))
            {
                side_peak_x10 = steady_x10;
            }

            side_debounce = (steady_x10 < S4_SIDE_LEVEL_MM_X10)
                          ? (uint8_t)(side_debounce + 1u) : 0u;

            if (side_debounce >= (uint8_t)S4_SIDE_DEBOUNCE)
            {
                report_side("done");
                b_side_active = false;
                side_debounce = 0u;
            }
            else if (!b_side_reported
                     && ((app_now_ms() - side_start_ms) > S4_SIDE_REPORT_MS))
            {
                report_side("still raised");
                b_side_reported = true;
            }
            else
            {
                /* On the ramp. */
            }
        }
    }
}

/*
 * Lift check on every sample of a batch.  Z is smoothed first (motor
 * vibration and the odd bad I2C read are single-sample spikes), then a
 * lift is the smoothed Z more than S4_LIFT_MG from its resting value for
 * S4_LIFT_SAMPLES in a row.  Either direction: the car is accelerated up
 * when picked up and down when the hand stops it.
 */
static bool detect_lift(const uint8_t *p_buf, uint8_t count)
{
    int32_t az_mg;
    int32_t off_mg;
    uint8_t idx;
    bool    b_lift = false;

    for (idx = 0u; idx < count; idx++)
    {
        az_mg = to_milli_g((int32_t)get_axis(&p_buf[(idx * 6u) + 4u]));

        if (!b_lift_primed)
        {
            lift_z_mg     = az_mg;        /* start from where it is */
            b_lift_primed = true;
        }
        lift_z_mg += (az_mg - lift_z_mg) / S4_LIFT_FILTER_DIV;

        off_mg = lift_z_mg - rest_z_mg;
        off_mg = (off_mg < 0) ? -off_mg : off_mg;
        lift_peak_mg = (off_mg > lift_peak_mg) ? off_mg : lift_peak_mg;

        if (off_mg > S4_LIFT_MG)
        {
            lift_run = (lift_run < 0xFFFFu) ? (uint16_t)(lift_run + 1u)
                                            : lift_run;
            b_lift   = b_lift || (lift_run >= S4_LIFT_SAMPLES);
        }
        else
        {
            lift_run = 0u;
        }
    }

    return (b_lift);
}

/*
 * Process one drained batch while active: lift check on every sample,
 * then the tilt-fault and hump logic on the (already updated) tilt.
 */
static void process_batch(const uint8_t *p_buf, uint8_t count)
{
    int32_t distance_now;

    if (!b_calibrated)
    {
        /* Uncalibrated: no resting Z and no offsets, and IMU_FAULT
           (DEGRADE) is already set.  Never E-STOP on this data. */
        lift_run      = 0u;
        tilt_debounce = 0u;
    }
    else if (detect_lift(p_buf, count))
    {
        /* Whenever the run is active, not only while the wheels turn:
           a car lifted during an obstacle scan would otherwise drive off
           in the air when the scan ends. */
        fault_set(FAULT_LIFTED);
        state    = S4_LIFTED;
        lift_run = 0u;
        APP_LOG((UB*)"[S4] LIFTED: Z %d mg, rest %d mg -> motors stopped\n",
                  (INT)lift_z_mg, (INT)rest_z_mg);
    }
    else if ((tilt_deg > S4_TILT_FAULT_DEG) || (tilt_deg < -S4_TILT_FAULT_DEG)
             || (roll_deg > S4_TILT_FAULT_DEG)
             || (roll_deg < -S4_TILT_FAULT_DEG))
    {
        /* Tipped over front-back OR sideways. */
        tilt_debounce = (uint8_t)(tilt_debounce + 1u);
        if (tilt_debounce > S4_DEBOUNCE_SAMPLES)
        {
            fault_set(FAULT_TILT);
            state = S4_TILT_FAULT;
            APP_LOG((UB*)"[S4] TILT: pitch %d roll %d deg (fault above "
                         "%d)\n", (INT)tilt_deg, (INT)roll_deg,
                      (INT)S4_TILT_FAULT_DEG);
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
    b_calibrated = calibrate_at_boot();
    if (b_calibrated)
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
                step_side_ramp();
            }

            if (S4_IMU_FAULT == state)
            {
                /* The sensor came back.  The run stays marked invalid:
                   the data already missed is gone. */
                state = S4_LEVEL_RUN;
            }

            if (b_active && !b_was_active)
            {
                lift_peak_mg  = 0;              /* new run */
                b_lift_primed = false;
                lift_run      = 0u;
            }
            else if (!b_active && b_was_active && b_calibrated)
            {
                APP_LOG((UB*)"[S4] lift check: peak %d mg this run "
                             "(stops above %d)\n", (INT)lift_peak_mg,
                        (INT)S4_LIFT_MG);
            }
            else
            {
                /* No change. */
            }
            b_was_active = b_active;

            if (!b_active)
            {
                state = ((S4_LIFTED == state) || (S4_TILT_FAULT == state))
                      ? state : S4_IDLE;
                last_distance_mm = s2_get_distance_mm();
            }
            else if ((count > 0) && (S4_LIFTED != state)
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

EXPORT uint16_t s4_get_side_height_mm(void)
{
    return ((uint16_t)(side_word & S4_SIDE_MM_MASK));
}

EXPORT uint16_t s4_get_side_live_mm(void)
{
    int32_t steady_x10 = side_steady_x10;       /* one load */

    return (b_side_active ? side_mm(steady_x10) : 0u);
}

EXPORT int8_t s4_get_side_sign(void)
{
    uint16_t word = side_word;                  /* one load */
    int8_t   sign = 0;

    if (0u != (word & S4_SIDE_UP_POS))
    {
        sign = 1;
    }
    else if (0u != (word & S4_SIDE_UP_NEG))
    {
        sign = -1;
    }
    else
    {
        /* No side ramp yet. */
    }

    return (sign);
}

EXPORT int8_t s4_get_roll_deg(void)
{
    return (roll_deg);
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