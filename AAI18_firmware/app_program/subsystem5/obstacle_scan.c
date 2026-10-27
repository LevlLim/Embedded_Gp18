/*
 * obstacle_scan.c
 *
 * Subsystem 5 -- HC-SR04P scanning and obstacle profiling.
 * Statechart 5, Design Review V2 section 6.5.
 *
 * This task SENSES ONLY.  It never sends a motion command: when something
 * enters D_detect it posts S5_EVT_OBSTACLE_SEEN and the Vehicle
 * Controller stops the car and asks for a profile (S5_REQ_SCAN).  The
 * profile goes back through mbf_obstacle with S5_EVT_OBSTACLE.  Keeping
 * one sender of motion commands is what stops two tasks' 3-bit sequence
 * numbers from colliding and silently dropping a STOP.
 *
 * D_STOP is watched only when it means something: while coarse scanning
 * during line following, or while the Controller has the forward guard
 * on during a forward bypass leg, and only for returns within
 * S5_DSTOP_ARC_DEG of straight ahead.  A side return during the profiling
 * sweep, or anything seen while the mission is not running, never latches
 * an E-STOP.
 *
 * CR1: the profiling sweep stores one uint8_t centimetre reading per step
 * (25 bytes); every phase variable is a byte; telemetry reads the forward
 * distance as a single byte.
 */

#include "subsystem5.h"
#include "../fault.h"
#include "../drivers/gpio_irq.h"
#include "../drivers/rp2040_regs.h"

#define S5_OBSTACLE_QUEUE_DEPTH (2u)
#define SERVO_FRAME_US          (20000u)   /* 50 Hz */
#define SERVO_CLK_DIV           (125u)     /* 125 MHz / 125 = 1 us ticks */
#define ECHO_WAIT_MS            (38)

typedef enum
{
    PING_RANGE   = 0,       /* a return inside the timeout             */
    PING_CLEAR   = 1,       /* pulse seen, nothing within range        */
    PING_MISSING = 2        /* no pulse at all: sensor not answering   */
} ping_result_t;

/* ---------------------------------------------------------------------
 * Private state.  Written by obstacle_task alone, except the two request
 * bytes (written by the Controller).
 * --------------------------------------------------------------------- */

static ID h_mbf_obstacle = (ID)0;

static volatile uint8_t request         = (uint8_t)S5_REQ_NONE;
static volatile bool    b_forward_guard = false;
static volatile uint8_t forward_cm      = 0u;

static s5_obstacle_phase_t phase = OB_IDLE;
static int8_t   servo_angle   = 0;
static int8_t   sweep_dir     = 1;
static uint8_t  settle_ticks  = 0u;
static uint8_t  scan_step     = 0u;
static uint8_t  missing_count = 0u;
static uint8_t  dstop_count   = 0u;   /* readings in a row inside D_stop */
static uint8_t  detect_count  = 0u;   /* readings in a row inside D_det  */
static uint32_t run_start_ms  = 0u;   /* updated while NOT running        */
static uint32_t last_echo_seq = 0u;
static uint8_t  scan_cm[S5_FINE_STEPS];

/* sin(0..63 degrees) in Q8. */
static const uint8_t sin_q8[64] =
{
      0,   4,   9,  13,  18,  22,  27,  31,  36,  40,
     44,  49,  53,  58,  62,  66,  71,  75,  79,  83,
     87,  92,  96, 100, 104, 108, 112, 116, 120, 124,
    128, 132, 136, 139, 143, 147, 150, 154, 158, 161,
    165, 168, 171, 175, 178, 181, 184, 187, 190, 193,
    196, 199, 202, 204, 207, 209, 212, 214, 216, 218,
    221, 223, 225, 227
};

/* ---------------------------------------------------------------------
 * Private functions
 * --------------------------------------------------------------------- */

/* Point the servo.  Angle is degrees from straight ahead, positive right. */
static void servo_point(int32_t angle_deg)
{
    int32_t angle = (1 == S5_SERVO_FIXED) ? 0 : angle_deg;

    if (angle > S5_SERVO_LIMIT_DEG)
    {
        angle = S5_SERVO_LIMIT_DEG;
    }
    else if (angle < -S5_SERVO_LIMIT_DEG)
    {
        angle = -S5_SERVO_LIMIT_DEG;
    }
    else
    {
        /* In range. */
    }

    /* 1 us per count, so the compare value is the pulse width. */
    (void)pwm_set_cc(PIN_SERVO_SCAN,
                     (UW)((int32_t)S5_SERVO_CENTRE_US
                          + (angle * (int32_t)S5_SERVO_US_PER_DEG)));
    servo_angle = (int8_t)angle;
}

/*
 * Fire one ranging pulse and read the result.  The dispatcher times the
 * echo; this function only waits for it.
 */
static ping_result_t ping(uint8_t *p_distance_cm)
{
    gpio_irq_echo_t echo;
    ping_result_t   result = PING_MISSING;
    uint32_t        range_cm;

    *p_distance_cm = 0u;

    (void)gpio_set_val(PIN_US_TRIG, 1u);
    (void)tk_dly_tsk(1);            /* >= 10 us trigger pulse */
    (void)gpio_set_val(PIN_US_TRIG, 0u);
    (void)tk_dly_tsk(ECHO_WAIT_MS);

    if (gpio_irq_read_echo(&echo) && (echo.seq != last_echo_seq))
    {
        last_echo_seq = echo.seq;

        if (echo.width_us < S5_ECHO_TIMEOUT_US)
        {
            range_cm       = (uint32_t)s5_echo_us_to_cm(echo.width_us);
            *p_distance_cm = (uint8_t)((range_cm > 255u) ? 255u : range_cm);
            result         = (0u != *p_distance_cm) ? PING_RANGE : PING_CLEAR;
        }
        else
        {
            result = PING_CLEAR;
        }
    }

    return (result);
}

/* True if a return at the current servo angle counts as straight ahead. */
static bool is_ahead(void)
{
    return ((servo_angle <= S5_DSTOP_ARC_DEG)
            && (servo_angle >= -S5_DSTOP_ARC_DEG));
}

/* True if a reading is a real return (not none, not in the blind zone). */
static bool is_valid_cm(uint8_t distance_cm)
{
    return (distance_cm >= S5_MIN_VALID_CM);
}

/* D_stop: a return straight ahead inside the hard stop distance,
   S5_CONFIRM_PINGS times in a row. */
static void check_dstop(uint8_t distance_cm)
{
    if (is_valid_cm(distance_cm) && (distance_cm <= PROV_D_STOP_CM)
        && is_ahead())
    {
        dstop_count = (uint8_t)(dstop_count + 1u);
        if (dstop_count >= S5_CONFIRM_PINGS)
        {
            dstop_count = 0u;
            APP_LOG((UB*)"[S5] D_STOP: obstacle at %d cm\n",
                      (INT)distance_cm);
            fault_set(FAULT_OBSTACLE_STOP);
        }
    }
    else
    {
        dstop_count = 0u;
    }
}

/* Send a profile (or a false alarm, distance 0) to the Controller. */
static void publish_profile(const msg_obstacle_t *p_profile)
{
    uint8_t buf[MSG_LEN_OBSTACLE];

    if (MSG_LEN_OBSTACLE == msg_pack_obstacle(p_profile, buf,
                                              (uint8_t)sizeof(buf)))
    {
        (void)tk_snd_mbf(h_mbf_obstacle, buf, (INT)MSG_LEN_OBSTACLE,
                         TMO_POL);
        s5_post_event(S5_EVT_OBSTACLE);
    }
}

/* Apply a Controller request, in this task's own context. */
static void apply_request(void)
{
    s5_request_t req = (s5_request_t)request;

    request = (uint8_t)S5_REQ_NONE;

    if (S5_REQ_SCAN == req)
    {
        scan_step    = 0u;
        settle_ticks = S5_SETTLE_TICKS;
        servo_point(-S5_FINE_ARC_DEG);
        phase = OB_FINE_SCAN;
    }
    else if (S5_REQ_RESUME == req)
    {
        servo_point(0);
        phase = OB_COARSE_SCAN;
    }
    else
    {
        /* No request. */
    }
}

/* COARSE_SCAN: sweep the bounded forward arc while driving (risk R6). */
static void step_coarse(ping_result_t result, uint8_t distance_cm)
{
    check_dstop(distance_cm);

    if ((PING_RANGE == result) && is_valid_cm(distance_cm)
        && (distance_cm <= PROV_D_DETECT_CM))
    {
        detect_count = (uint8_t)(detect_count + 1u);
    }
    else
    {
        detect_count = 0u;
    }

    if (detect_count >= S5_CONFIRM_PINGS)
    {
        detect_count = 0u;
        APP_LOG((UB*)"[S5] obstacle at %d cm: stopping\n",
                  (INT)distance_cm);
        s5_post_event(S5_EVT_OBSTACLE_SEEN);
        servo_point(0);
        phase = OB_WAIT_STOP;
    }
    else
    {
        servo_point((int32_t)servo_angle
                    + ((int32_t)sweep_dir * S5_STEP_DEG));

        if (servo_angle >= S5_COARSE_ARC_DEG)
        {
            sweep_dir = -1;
        }
        else if (servo_angle <= -S5_COARSE_ARC_DEG)
        {
            sweep_dir = 1;
        }
        else
        {
            /* Sweeping. */
        }
    }
}

/* FINE_SCAN: one reading per step once the servo has settled. */
static void step_fine(uint8_t distance_cm)
{
    msg_obstacle_t profile = {0};

    if (0u != settle_ticks)
    {
        settle_ticks = (uint8_t)(settle_ticks - 1u);
    }
    else if (1 == S5_SERVO_FIXED)
    {
        /* Fixed servo: no sweep, so no profile.  Wait, stopped, until
           nothing is within detection range ahead, then report a false
           alarm so the Controller resumes line following. */
        if ((0u == distance_cm) || (distance_cm > PROV_D_DETECT_CM))
        {
            profile.distance_cm = 0u;
            publish_profile(&profile);
            phase = OB_HOLD;
        }
    }
    else
    {
        scan_cm[scan_step] = distance_cm;
        scan_step          = (uint8_t)(scan_step + 1u);

        if (scan_step < S5_FINE_STEPS)
        {
            servo_point((int32_t)servo_angle + S5_STEP_DEG);
            settle_ticks = 1u;      /* a 5 degree step settles in ~60 ms */
        }
        else
        {
            if (!s5_build_profile(scan_cm, S5_FINE_STEPS,
                                  (uint8_t)PROV_D_DETECT_CM, &profile))
            {
                profile.distance_cm = 0u;   /* false alarm */
            }

            publish_profile(&profile);
            servo_point(0);
            phase = OB_HOLD;
        }
    }
}

/* ---------------------------------------------------------------------
 * Public pure helpers (tested on the host)
 * --------------------------------------------------------------------- */

EXPORT uint16_t s5_echo_us_to_cm(uint32_t width_us)
{
    /* 343 m/s there and back: cm = width_us / 58. */
    uint32_t range_cm = width_us / 58u;

    return ((uint16_t)((range_cm > 0xFFFFu) ? 0xFFFFu : range_cm));
}

EXPORT msg_side_t s5_choose_side(uint8_t clear_left_cm, uint8_t clear_right_cm,
                                 uint8_t needed_cm)
{
    msg_side_t side;

    if ((clear_left_cm < needed_cm) && (clear_right_cm < needed_cm))
    {
        side = MSG_SIDE_NONE;       /* neither side fits */
    }
    else if (clear_left_cm >= clear_right_cm)
    {
        side = MSG_SIDE_LEFT;
    }
    else
    {
        side = MSG_SIDE_RIGHT;
    }

    return (side);
}

EXPORT int32_t s5_lateral_cm(uint32_t distance_cm, int32_t angle_deg)
{
    int32_t magnitude = (angle_deg < 0) ? -angle_deg : angle_deg;
    int32_t lateral;

    if (magnitude > 63)
    {
        magnitude = 63;
    }

    lateral = ((int32_t)distance_cm * (int32_t)sin_q8[magnitude]) / 256;

    return ((angle_deg < 0) ? -lateral : lateral);
}

/* Angle of fine-scan step idx. */
static int32_t step_angle(uint32_t idx)
{
    return (-S5_FINE_ARC_DEG + ((int32_t)idx * S5_STEP_DEG));
}

/*
 * Free lateral room beyond one edge of the object: walk outward from the
 * edge until a return nearer than S5_CLEAR_RANGE_CM blocks it.  With no
 * blocker, the room runs to where the scan arc meets the clear range.
 */
static int32_t clearance_cm(const uint8_t *p_cm, uint8_t steps,
                            int32_t edge_x, uint32_t edge_idx, int32_t dir)
{
    int32_t limit_x = s5_lateral_cm(S5_CLEAR_RANGE_CM,
                                    step_angle((dir < 0) ? 0u : (steps - 1u)));
    int32_t idx     = (int32_t)edge_idx + dir;
    int32_t room;

    while ((idx >= 0) && (idx < (int32_t)steps))
    {
        if ((0u != p_cm[idx]) && (p_cm[idx] < S5_CLEAR_RANGE_CM))
        {
            limit_x = s5_lateral_cm(p_cm[idx], step_angle((uint32_t)idx));
            break;
        }
        idx += dir;
    }

    room = (dir < 0) ? (edge_x - limit_x) : (limit_x - edge_x);

    return ((room < 0) ? 0 : room);
}

EXPORT bool s5_build_profile(const uint8_t *p_distances_cm, uint8_t steps,
                             uint8_t detect_cm, msg_obstacle_t *p_profile)
{
    uint32_t idx;
    uint32_t nearest = steps;
    uint32_t left;
    uint32_t right;
    uint32_t band;
    int32_t  x_left;
    int32_t  x_right;
    bool     b_sat = false;

    if ((NULL == p_distances_cm) || (NULL == p_profile) || (0u == steps))
    {
        return (false);
    }

    for (idx = 0u; idx < steps; idx++)
    {
        if ((0u != p_distances_cm[idx]) && (p_distances_cm[idx] <= detect_cm)
            && ((steps == nearest)
                || (p_distances_cm[idx] < p_distances_cm[nearest])))
        {
            nearest = idx;
        }
    }

    if (steps == nearest)
    {
        return (false);
    }

    /* The object: contiguous returns within the depth tolerance. */
    band  = (uint32_t)p_distances_cm[nearest] + S5_OBJECT_DEPTH_TOL_CM;
    left  = nearest;
    right = nearest;
    while ((left > 0u) && (0u != p_distances_cm[left - 1u])
           && (p_distances_cm[left - 1u] <= band))
    {
        left--;
    }
    while (((right + 1u) < steps) && (0u != p_distances_cm[right + 1u])
           && (p_distances_cm[right + 1u] <= band))
    {
        right++;
    }

    x_left  = s5_lateral_cm(p_distances_cm[left], step_angle(left));
    x_right = s5_lateral_cm(p_distances_cm[right], step_angle(right));

    /* Saturating packing on every field (rule R5). */
    p_profile->distance_cm    = p_distances_cm[nearest];
    p_profile->bearing_deg    = msg_sat_i8(step_angle(nearest),
                                           (int8_t)MSG_LIM_OBST_BEARING,
                                           &b_sat);
    p_profile->width_cm       = (uint8_t)msg_sat_u16(
                                    (uint32_t)(x_right - x_left),
                                    (uint16_t)MSG_MAX_OBST_WIDTH, &b_sat);
    p_profile->clear_left_cm  = (uint8_t)msg_sat_u16(
                                    (uint32_t)clearance_cm(p_distances_cm,
                                        steps, x_left, left, -1),
                                    (uint16_t)MSG_MAX_OBST_CLEAR, &b_sat);
    p_profile->clear_right_cm = (uint8_t)msg_sat_u16(
                                    (uint32_t)clearance_cm(p_distances_cm,
                                        steps, x_right, right, 1),
                                    (uint16_t)MSG_MAX_OBST_CLEAR, &b_sat);
    p_profile->b_saturated    = b_sat;
    p_profile->side           = s5_choose_side(p_profile->clear_left_cm,
                                    p_profile->clear_right_cm,
                                    (uint8_t)(S5_ROBOT_WIDTH_CM
                                              + S5_CLEARANCE_MARGIN_CM));

    return (true);
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT ER s5_obstacle_init(void)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (INT)APP_MBF_BUFSZ(S5_OBSTACLE_QUEUE_DEPTH,
                                     MSG_LEN_OBSTACLE);
    cmbf.maxmsz = (INT)MSG_LEN_OBSTACLE;
    h_mbf_obstacle = tk_cre_mbf(&cmbf);

    /* Ultrasonic trigger is a plain output; ECHO is configured by the
       shared IO_BANK0 dispatcher in subsystem 2's init. */
    (void)gpio_set_pin(PIN_US_TRIG, GPIO_MODE_OUT);
    (void)gpio_set_val(PIN_US_TRIG, 0u);

    /*
     * Servo on PWM slice 6.  The template's pwm_set_wrap() does not set
     * the divider, which resets to 1: the slice would count at 125 MHz and
     * a wrap of 20000 would give 6.25 kHz, which no servo follows.  Divide
     * by 125 so one count is 1 us; TOP = 19999 then gives exactly 50 Hz,
     * and the compare value is the pulse width in microseconds.
     */
    RP2040_RELEASE_PWM();
    out_w(PWM_SLICE_DIV(PWM_SLICE_OF(PIN_SERVO_SCAN)),
          SERVO_CLK_DIV << PWM_DIV_INT_SHIFT);
    (void)pwm_set_pin(PIN_SERVO_SCAN);
    (void)pwm_set_wrap(PIN_SERVO_SCAN, SERVO_FRAME_US - 1u);
    (void)pwm_set_cc(PIN_SERVO_SCAN, S5_SERVO_CENTRE_US);
    (void)pwm_set_enabled(PIN_SERVO_SCAN, TRUE);

    return ((h_mbf_obstacle < E_OK) ? (ER)h_mbf_obstacle : E_OK);
}

EXPORT void obstacle_task(INT stacd, void *p_exinf)
{
    ping_result_t result;
    uint8_t       distance_cm;

    (void)stacd;
    (void)p_exinf;

    servo_point(0);

    for (;;)
    {
        (void)app_wait_period();

        apply_request();
        result = ping(&distance_cm);

        if (is_ahead())
        {
            forward_cm = distance_cm;
        }

        if (!s5_is_running())
        {
            /* Not running: no faults, no events, servo parked ahead. */
            if (OB_IDLE != phase)
            {
                servo_point(0);
                phase = OB_IDLE;
            }
            missing_count = 0u;
            dstop_count   = 0u;
            detect_count  = 0u;
            run_start_ms  = app_now_ms();
        }
        else if ((app_now_ms() - run_start_ms) < S5_START_GRACE_MS)
        {
            /* Just started: the hand that pressed START is still in front
               of the sensor (bench log: "obstacle at 6 cm" the moment the
               car started).  Do not act on it. */
            dstop_count  = 0u;
            detect_count = 0u;
        }
        else
        {
            /* A pulse that never comes means the sensor is not working:
               a DEGRADE fault.  "Nothing in range" is normal and is not. */
            missing_count = (PING_MISSING == result)
                          ? (uint8_t)(missing_count + 1u) : 0u;
            if (missing_count >= S5_NO_ECHO_LIMIT)
            {
                fault_set(FAULT_NO_ECHO);
                missing_count = 0u;
            }

            switch (phase)
            {
                case OB_IDLE:
                    phase = OB_COARSE_SCAN;
                break;

                case OB_COARSE_SCAN:
                    step_coarse(result, distance_cm);
                break;

                case OB_FINE_SCAN:
                    step_fine(distance_cm);
                break;

                case OB_HOLD:
                    if (b_forward_guard)
                    {
                        check_dstop(distance_cm);
                    }
                break;

                default:
                    /* OB_WAIT_STOP: the Controller is stopping the car. */
                break;
            }
        }
    }
}

EXPORT uint8_t s5_get_obstacle_distance_cm(void)
{
    return (forward_cm);
}

EXPORT void s5_obstacle_request(s5_request_t req)
{
    request = (uint8_t)req;
}

EXPORT void s5_obstacle_guard(bool b_on)
{
    b_forward_guard = b_on;
}

EXPORT bool s5_take_obstacle(msg_obstacle_t *p_out)
{
    uint8_t buf[MSG_LEN_OBSTACLE];
    INT     received;
    bool    b_ok = false;

    if (NULL != p_out)
    {
        received = tk_rcv_mbf(h_mbf_obstacle, buf, TMO_POL);
        b_ok     = (received >= (INT)MSG_LEN_OBSTACLE)
                   && msg_unpack_obstacle(buf, MSG_LEN_OBSTACLE, p_out);
    }

    return (b_ok);
}

/* end of file */
