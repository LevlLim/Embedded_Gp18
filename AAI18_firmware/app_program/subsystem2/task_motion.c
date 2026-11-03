/*
 * task_motion.c
 *
 * Subsystem 2 -- the Motion Control task.  Statechart 2, Design Review V2
 * section 6.2.
 *
 * This is the highest-priority application task and the one FR11 depends
 * on.  Its 10 ms cycle (woken by a cyclic handler) is, in order:
 *   1. read the fault flag and brake immediately if any E-STOP bit is set
 *   2. read the encoders
 *   3. take a command, if one is waiting
 *   4. run the state machine and apply the motor outputs
 *   5. publish speed and encoder total to Telemetry
 *
 * PACKED CONTROL WORD (CR1 item 1)
 * --------------------------------
 * The mode, the move's direction and duty, the speed setpoint, the last
 * command sequence and the encoder health flags together need 27 bits.
 * They live in ONE 32-bit word written only by this task (rule R1), so
 * s2_get_state() on any task or core reads the state with a single load.
 *   bits  0..2   state           (s2_state_t, 6 values)
 *   bits  3..4   left direction  (2-bit two's complement: -1, 0, +1)
 *   bits  5..6   right direction
 *   bits  7..13  move duty %     (0..100)
 *   bits 14..20  speed setpoint % (0..100)
 *   bits 21..23  last command sequence
 *   bit  24      a sequence has been seen
 *   bit  25      left encoder judged dead
 *   bit  26      right encoder judged dead
 *
 * Odometry and PID terms stay 32-bit: they are intermediate maths, which
 * Design Review section 7.4 deliberately excludes from packing.
 */

#include "subsystem2.h"
#include "motor_control.h"
#include "../fault.h"
#include "../drivers/gpio_irq.h"
#include "../drivers/rp2040_regs.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem5/subsystem5.h"

#ifndef S2_INNER_REVERSE_MAX_PCT        /* older subsystem2.h */
#define S2_INNER_REVERSE_MAX_PCT (30)
#endif

#define S2_CMD_QUEUE_DEPTH  (4u)

/* Control word fields: shift and width. */
#define CTL_STATE_SHIFT     (0u)
#define CTL_STATE_BITS      (3u)
#define CTL_DIR_L_SHIFT     (3u)
#define CTL_DIR_R_SHIFT     (5u)
#define CTL_DIR_BITS        (2u)
#define CTL_DUTY_SHIFT      (7u)
#define CTL_SETPT_SHIFT     (14u)
#define CTL_PCT_BITS        (7u)
#define CTL_SEQ_SHIFT       (21u)
#define CTL_SEQ_BITS        (3u)
#define CTL_HAVE_SEQ_BIT    (1u << 24u)
#define CTL_ENC_L_BAD_BIT   (1u << 25u)
#define CTL_ENC_R_BAD_BIT   (1u << 26u)

/* ---------------------------------------------------------------------
 * Private state
 * --------------------------------------------------------------------- */

static ID h_mbf_cmd = (ID)0;

/* The packed control word.  Single writer: this task. */
static volatile uint32_t ctl = 0u;

static int32_t  speed_left_cps  = 0;    /* counts per second */
static int32_t  speed_right_cps = 0;
static int32_t  distance_mm     = 0;

static int32_t  prev_count_left  = 0;
static int32_t  prev_count_right = 0;
static int32_t  odometer_left    = 0;   /* counts, signed               */
static int32_t  odometer_right   = 0;
static uint32_t encoder_total    = 0u;  /* |left| + |right|, for FR9    */

static int32_t  target_left   = 0;
static int32_t  target_right  = 0;
static int32_t  move_start_l  = 0;      /* counts at the start of a move */
static int32_t  move_start_r  = 0;
static int32_t  move_total_l  = 0;      /* counts the move needs, > 0    */
static int32_t  move_total_r  = 0;
static int32_t  bal_integral  = 0;      /* wheel balance, per mille.tick */
static bool     b_window_one_wheel = false;    /* see check_encoders() */
static uint32_t drive_start_us = 0u;    /* arms the stall detector      */

static uint32_t window_start_ms    = 0u;
static uint32_t window_edges_left  = 0u;
static uint32_t window_edges_right = 0u;

/* ---------------------------------------------------------------------
 * Control word access
 * --------------------------------------------------------------------- */

static uint32_t ctl_get(uint32_t shift, uint32_t bits)
{
    return ((ctl >> shift) & ((1u << bits) - 1u));
}

static void ctl_put(uint32_t shift, uint32_t bits, uint32_t value)
{
    uint32_t mask = ((1u << bits) - 1u) << shift;

    /* Read-modify-write is safe here: this task is the only writer. */
    ctl = (ctl & ~mask) | ((value << shift) & mask);
}

static s2_state_t get_state(void)
{
    return ((s2_state_t)ctl_get(CTL_STATE_SHIFT, CTL_STATE_BITS));
}

static void set_state(s2_state_t new_state)
{
    ctl_put(CTL_STATE_SHIFT, CTL_STATE_BITS, (uint32_t)new_state);
}

/* Direction is -1, 0 or +1, stored as 2-bit two's complement. */
static int8_t get_dir(uint32_t shift)
{
    uint32_t raw = ctl_get(shift, CTL_DIR_BITS);

    return ((3u == raw) ? (int8_t)-1 : (int8_t)raw);
}

static void put_dir(uint32_t shift, int8_t dir)
{
    uint32_t raw = (dir < 0) ? 3u : ((dir > 0) ? 1u : 0u);

    ctl_put(shift, CTL_DIR_BITS, raw);
}

static uint8_t get_setpoint(void)
{
    return ((uint8_t)ctl_get(CTL_SETPT_SHIFT, CTL_PCT_BITS));
}

static uint8_t get_move_duty(void)
{
    return ((uint8_t)ctl_get(CTL_DUTY_SHIFT, CTL_PCT_BITS));
}

/* ---------------------------------------------------------------------
 * Private functions
 * --------------------------------------------------------------------- */

/*
 * Update speeds and the odometer.  The odometer accumulates COUNTS and
 * converts the total, so integer truncation does not build up a few
 * percent of error every 10 ms (which fed straight into hump height).
 */
static void update_odometry(const gpio_irq_encoders_t *p_enc)
{
    int32_t delta_left  = p_enc->count_left  - prev_count_left;
    int32_t delta_right = p_enc->count_right - prev_count_right;

    prev_count_left  = p_enc->count_left;
    prev_count_right = p_enc->count_right;

    speed_left_cps  = (delta_left  * 1000) / (int32_t)PERIOD_MOTION_MS;
    speed_right_cps = (delta_right * 1000) / (int32_t)PERIOD_MOTION_MS;

    /* Each wheel in its own units (they differ), then the mean. */
    odometer_left  += delta_left;
    odometer_right += delta_right;
    distance_mm     = (motor_counts_to_mm(odometer_left, true)
                       + motor_counts_to_mm(odometer_right, false)) / 2;

    /* Magnitudes; the casts are of non-negative values. */
    encoder_total += (uint32_t)((delta_left  < 0) ? -delta_left  : delta_left)
                   + (uint32_t)((delta_right < 0) ? -delta_right : delta_right);
}

/*
 * Remember when drive began, and reset the encoder-health window, so the
 * stall detector never measures from a time the car was parked.
 */
static void arm_drive(const gpio_irq_encoders_t *p_enc)
{
    drive_start_us     = RP2040_NOW_US();
    window_start_ms    = app_now_ms();
    window_edges_left  = p_enc->edges_left;
    window_edges_right = p_enc->edges_right;
}

/*
 * Stall: drive above the threshold but no encoder edge for the timeout,
 * measured from the LATER of the last edge and the start of this drive.
 */
static bool is_stalled(const gpio_irq_encoders_t *p_enc, uint8_t duty_pct)
{
    uint32_t now_us = RP2040_NOW_US();
    uint32_t since_edge;
    uint32_t since_start;
    bool     b_stalled = false;

    if (duty_pct >= S2_STALL_DUTY_PCT)
    {
        since_edge  = now_us - p_enc->last_edge_us;
        since_start = now_us - drive_start_us;

        if ((since_edge  > (S2_STALL_TIMEOUT_MS * 1000u))
            && (since_start > (S2_STALL_TIMEOUT_MS * 1000u)))
        {
            b_stalled = true;
        }
    }

    return (b_stalled);
}

/*
 * Encoder health, checked once per window while driving.  One wheel
 * counting while the other is silent marks the silent one dead.
 */
static void check_encoders(const gpio_irq_encoders_t *p_enc)
{
    uint32_t left;
    uint32_t right;

    int32_t  duty_l = 0;
    int32_t  duty_r = 0;

    /* A wheel deliberately stopped (a pivot turn) gives no edges by
       design: such a window proves nothing about its encoder. */
    motor_get_applied(&duty_l, &duty_r);
    duty_l = (duty_l < 0) ? -duty_l : duty_l;
    duty_r = (duty_r < 0) ? -duty_r : duty_r;
    if ((duty_l < (int32_t)S2_STALL_DUTY_PCT)
        || (duty_r < (int32_t)S2_STALL_DUTY_PCT))
    {
        b_window_one_wheel = true;
    }

    if ((app_now_ms() - window_start_ms) >= S2_ENC_WINDOW_MS)
    {
        left  = p_enc->edges_left  - window_edges_left;
        right = p_enc->edges_right - window_edges_right;

        if (b_window_one_wheel)
        {
            /* Not both wheels driven all window: no verdict. */
        }
        else if ((left >= S2_ENC_FAULT_EDGES) && (0u == right))
        {
            ctl = ctl | CTL_ENC_R_BAD_BIT;
            fault_set(FAULT_ENCODER);
        }
        else if ((right >= S2_ENC_FAULT_EDGES) && (0u == left))
        {
            ctl = ctl | CTL_ENC_L_BAD_BIT;
            fault_set(FAULT_ENCODER);
        }
        else
        {
            /* Both counting, or both idle: nothing to conclude. */
        }

        arm_drive(p_enc);
        b_window_one_wheel = false;
    }
}

/*
 * Receive one waiting command.  Returns true if it is new (duplicate
 * sequence numbers are ignored, so a retransmission cannot run twice).
 */
static bool take_command(msg_motion_cmd_t *p_cmd)
{
    uint8_t buf[MSG_LEN_MOTION_CMD];
    INT     received;
    bool    b_new = false;

    received = tk_rcv_mbf(h_mbf_cmd, buf, TMO_POL);

    if ((received >= (INT)MSG_LEN_MOTION_CMD)
        && msg_unpack_motion_cmd(buf, MSG_LEN_MOTION_CMD, p_cmd))
    {
        if ((0u == (ctl & CTL_HAVE_SEQ_BIT))
            || ((uint32_t)p_cmd->seq != ctl_get(CTL_SEQ_SHIFT, CTL_SEQ_BITS)))
        {
            ctl_put(CTL_SEQ_SHIFT, CTL_SEQ_BITS, (uint32_t)p_cmd->seq);
            ctl   = ctl | CTL_HAVE_SEQ_BIT;
            b_new = true;
        }
    }

    return (b_new);
}

/*
 * Set up an encoder-measured move.
 */
static void begin_move(const msg_motion_cmd_t *p_cmd,
                       const gpio_irq_encoders_t *p_enc)
{
    int32_t mm     = 0;     /* straight moves */
    int32_t deg    = 0;     /* spin turns     */
    int32_t counts_l;
    int32_t counts_r;
    int8_t  dir_l  = 0;
    int8_t  dir_r  = 0;

    switch (p_cmd->type)
    {
        case MSG_CMD_FORWARD:
            mm     = (int32_t)p_cmd->arg;
            dir_l  = 1;
            dir_r  = 1;
        break;

        case MSG_CMD_REVERSE:
            mm     = (int32_t)p_cmd->arg;
            dir_l  = -1;
            dir_r  = -1;
        break;

        case MSG_CMD_TURN_L:
            deg    = (int32_t)p_cmd->arg;
            dir_l  = -1;
            dir_r  = 1;
        break;

        case MSG_CMD_TURN_R:
            deg    = (int32_t)p_cmd->arg;
            dir_l  = 1;
            dir_r  = -1;
        break;

        case MSG_CMD_U_TURN:
            deg    = 180;
            dir_l  = 1;
            dir_r  = -1;
        break;

        default:
            /* STOP and LINE_PID never reach here. */
        break;
    }

    counts_l = (0 != deg) ? motor_degrees_to_counts(deg, true)
                          : motor_mm_to_counts(mm, true);
    counts_r = (0 != deg) ? motor_degrees_to_counts(deg, false)
                          : motor_mm_to_counts(mm, false);

    target_left  = p_enc->count_left  + ((int32_t)dir_l * counts_l);
    target_right = p_enc->count_right + ((int32_t)dir_r * counts_r);
    move_start_l = p_enc->count_left;
    move_start_r = p_enc->count_right;
    move_total_l = (counts_l > 0) ? counts_l : 1;
    move_total_r = (counts_r > 0) ? counts_r : 1;
    bal_integral = 0;

    put_dir(CTL_DIR_L_SHIFT, dir_l);
    put_dir(CTL_DIR_R_SHIFT, dir_r);
    ctl_put(CTL_DUTY_SHIFT, CTL_PCT_BITS,
            (uint32_t)((((MSG_CMD_TURN_L == p_cmd->type)
                         || (MSG_CMD_TURN_R == p_cmd->type))
                        && (get_setpoint() > S2_TURN_DUTY_PCT))
                       ? S2_TURN_DUTY_PCT : get_setpoint()));

    motor_pid_reset();
    arm_drive(p_enc);
}

/*
 * True when every HEALTHY wheel has reached its target.  A wheel whose
 * encoder is judged dead is ignored, so a move still completes.
 */
static bool move_complete(const gpio_irq_encoders_t *p_enc)
{
    int8_t dir_l = get_dir(CTL_DIR_L_SHIFT);
    int8_t dir_r = get_dir(CTL_DIR_R_SHIFT);
    bool   b_left_done;
    bool   b_right_done;

    b_left_done = (dir_l >= 0) ? (p_enc->count_left >= target_left)
                               : (p_enc->count_left <= target_left);
    b_right_done = (dir_r >= 0) ? (p_enc->count_right >= target_right)
                                : (p_enc->count_right <= target_right);

    b_left_done  = b_left_done  || (0u != (ctl & CTL_ENC_L_BAD_BIT));
    b_right_done = b_right_done || (0u != (ctl & CTL_ENC_R_BAD_BIT));

    return (b_left_done && b_right_done);
}

/* Low-pass filtered steering error, x8 for resolution.  Reset whenever
   line following (re)starts. */
static int32_t filtered_err_x8 = 0;
static volatile bool b_stop_on_line = false;   /* set by the Controller */
static bool    b_last_valid    = false;

/*
 * Drive a measured move with the two wheels kept in step.  The two sides
 * have different gearboxes (1260 vs 776 counts per revolution), so at
 * the same duty they turn at different speeds: a "straight" leg curved
 * and a turn swung round one wheel.  Compare how far through the move
 * each wheel is (per mille of its own target) and take duty from the
 * one that is ahead, giving it to the one behind.  A wheel that has
 * reached its target stops while the other finishes.
 */
static void drive_balanced(const gpio_irq_encoders_t *p_enc, uint8_t duty)
{
    int32_t done_l = p_enc->count_left  - move_start_l;
    int32_t done_r = p_enc->count_right - move_start_r;
    int32_t prog_l;
    int32_t prog_r;
    int32_t trim;
    int32_t limit = (int32_t)duty / 2;
    int32_t duty_l;
    int32_t duty_r;

    done_l = (done_l < 0) ? -done_l : done_l;
    done_r = (done_r < 0) ? -done_r : done_r;
    prog_l = (done_l * 1000) / move_total_l;
    prog_r = (done_r * 1000) / move_total_r;

    /* Proportional + integral.  With P alone the faster wheel stayed
       ~10 % ahead for the whole move (it needs a steady duty difference,
       and P only gives one while there is an error); it then finished
       first and stopped while the other caught up, swinging the car --
       "fwd" curved right.  The integral builds that steady difference
       up, so the progress error returns to zero. */
    bal_integral += prog_l - prog_r;
    if (bal_integral > (limit * S2_MOVE_BALANCE_I_DIV))
    {
        bal_integral = limit * S2_MOVE_BALANCE_I_DIV;
    }
    else if (bal_integral < (-limit * S2_MOVE_BALANCE_I_DIV))
    {
        bal_integral = -limit * S2_MOVE_BALANCE_I_DIV;
    }
    else
    {
        /* Within its range. */
    }

    trim = ((prog_l - prog_r) / S2_MOVE_BALANCE_DIV)
         + (bal_integral / S2_MOVE_BALANCE_I_DIV);
    trim = (trim > limit) ? limit : ((trim < -limit) ? -limit : trim);

    duty_l = (int32_t)duty - trim;
    duty_r = (int32_t)duty + trim;

    /* Never slow a wheel below the duty it needs to keep turning: the
       bench showed the faster (right) wheel cut to ~20 %, stalling, so
       the car pivoted right in steps ("forward, right, forward, right").
       Whatever a wheel cannot give up is added to the other instead. */
    if (duty_l < (int32_t)S2_MIN_RUN_DUTY_PCT)
    {
        duty_r = duty_r + ((int32_t)S2_MIN_RUN_DUTY_PCT - duty_l);
        duty_l = (int32_t)S2_MIN_RUN_DUTY_PCT;
    }
    if (duty_r < (int32_t)S2_MIN_RUN_DUTY_PCT)
    {
        duty_l = duty_l + ((int32_t)S2_MIN_RUN_DUTY_PCT - duty_r);
        duty_r = (int32_t)S2_MIN_RUN_DUTY_PCT;
    }

    /* A wheel that has reached its target stops. */
    duty_l = (prog_l >= 1000) ? 0 : duty_l;
    duty_r = (prog_r >= 1000) ? 0 : duty_r;

    motor_set(get_dir(CTL_DIR_L_SHIFT), motor_clamp_duty(duty_l),
              get_dir(CTL_DIR_R_SHIFT), motor_clamp_duty(duty_r));
}

/* True if a fresh line reading says a sensor is over the line. */
static bool line_in_view(void)
{
    msg_line_t line = {0};
    uint32_t   word = 0u;
    bool       b_seen = false;

    if (s3_read_line_word(&word)
        && (msg_line_age_ms(word, app_now_ms()) <= S2_LINE_STALE_MS))
    {
        msg_unpack_line(word, &line);
        b_seen = line.b_valid && s3_line_seen();
    }

    return (b_seen);
}

/*
 * Drive both wheels from SIGNED duties.  A negative duty runs that wheel
 * BACKWARD (at most S2_INNER_REVERSE_MAX_PCT): when the steering
 * correction is larger than the cruise duty, the inner wheel reverses and
 * the car turns about a point between the wheels, far tighter than with
 * the inner wheel merely stopped (bench: tight circle taken too wide).
 */
static void set_wheels_signed(int32_t left, int32_t right)
{
    int8_t dir_l = 1;
    int8_t dir_r = 1;

    if (left < 0)
    {
        dir_l = -1;
        left  = -left;
        left  = (left > S2_INNER_REVERSE_MAX_PCT) ? S2_INNER_REVERSE_MAX_PCT
                                                  : left;
    }
    if (right < 0)
    {
        dir_r = -1;
        right = -right;
        right = (right > S2_INNER_REVERSE_MAX_PCT)
              ? S2_INNER_REVERSE_MAX_PCT : right;
    }

    motor_set(dir_l, motor_clamp_duty(left), dir_r, motor_clamp_duty(right));
}

/*
 * Line-following step.
 *   fresh and valid  -> steer on the (filtered) error at cruise
 *   fresh but LOST   -> keep turning the way the line went (the error is
 *                       still published, saturated) at reduced speed, so
 *                       the car swings back onto the line in one smooth
 *                       arc instead of driving straight off it
 *   stale (no data)  -> hold straight at half speed (Statechart 2)
 */
static void step_line_pid(void)
{
    msg_line_t line = {0};
    uint32_t   word = 0u;
    int32_t    correction;
    int32_t    base = (int32_t)get_setpoint();
    bool       b_fresh = false;

    if (s3_read_line_word(&word)
        && (msg_line_age_ms(word, app_now_ms()) <= S2_LINE_STALE_MS))
    {
        msg_unpack_line(word, &line);
        b_fresh = true;
    }

    if (b_fresh && !line.b_valid)
    {
        /* LOST: pivot gently toward where the line went -- outer wheel at
           the slow turn duty, inner wheel stopped.  (Was: PID at 3/4
           cruise with a saturated error, a fast swing that carried the
           car straight across the line when it came back into view.) */
        if (line.error > 0)
        {
            motor_set(1, (uint8_t)S2_TURN_DUTY_PCT, 1, 0u);
        }
        else
        {
            motor_set(1, 0u, 1, (uint8_t)S2_TURN_DUTY_PCT);
        }
        motor_pid_reset();
        filtered_err_x8 = (int32_t)line.error * 8;
        b_last_valid    = false;
    }
    else if (b_fresh)
    {
        if (!b_last_valid)
        {
            /* Line found again: kill the turn NOW (unlimited brake, one
               tick), then steer from the current reading.  Without the
               brake the swing's momentum carries it over the line. */
            motor_brake();
            filtered_err_x8 = (int32_t)line.error * 8;
            b_last_valid    = true;
        }
        else
        {
            filtered_err_x8 += (((int32_t)line.error * 8) - filtered_err_x8)
                             / S2_ERR_FILTER_DIV;
            correction = motor_pid_step(0, filtered_err_x8 / 8, true);
            set_wheels_signed(base - correction, base + correction);
        }
    }
    else
    {
        motor_set(1, (uint8_t)(base / 2), 1, (uint8_t)(base / 2));
    }
}

/*
 * E-STOP check.  Returns true while latched.  Commands that arrive while
 * latched are DISCARDED: after the Controller clears the fault, the car
 * must not resume a manoeuvre queued before the stop.
 */
static bool handle_estop(uint16_t fault_bits)
{
    uint8_t buf[MSG_LEN_MOTION_CMD];
    bool    b_latched = (0u != (fault_bits & FAULT_MASK_ESTOP));

    if (b_latched)
    {
        if (M_ESTOP != get_state())
        {
            motor_brake();
            set_state(M_ESTOP);
        }

        while (tk_rcv_mbf(h_mbf_cmd, buf, TMO_POL) > 0)
        {
            /* Drain: nothing queued during an E-STOP survives it. */
        }
    }
    else if (M_ESTOP == get_state())
    {
        /* Cleared by the Controller: back to idle, never straight back
           to driving. */
        motor_pid_reset();
        set_state(M_IDLE);
    }
    else
    {
        /* Not latched, not recovering. */
    }

    return (b_latched);
}

/*
 * Act on a new command.
 */
static void handle_command(const msg_motion_cmd_t *p_cmd,
                           const gpio_irq_encoders_t *p_enc)
{
    switch (p_cmd->type)
    {
        case MSG_CMD_STOP:
            motor_brake();
            set_state(M_IDLE);
        break;

        case MSG_CMD_LINE_PID:
            motor_pid_reset();
            filtered_err_x8 = 0;
            b_last_valid    = true;
            b_stop_on_line  = false;
            arm_drive(p_enc);
            set_state(M_LINE_PID);
        break;

        default:
            begin_move(p_cmd, p_enc);
            set_state(M_MOVE);
        break;
    }
}

/* Larger of the two wheels' applied duty magnitudes, percent. */
static uint8_t applied_max_duty(void)
{
    int32_t left  = 0;
    int32_t right = 0;

    motor_get_applied(&left, &right);
    left  = (left  < 0) ? -left  : left;
    right = (right < 0) ? -right : right;

    return ((uint8_t)((left > right) ? left : right));
}

/*
 * One step of the state machine.  Returns the duty being commanded.
 */
static uint8_t step_state(const gpio_irq_encoders_t *p_enc)
{
    uint8_t duty = 0u;

    switch (get_state())
    {
        case M_LINE_PID:
            duty = get_setpoint();
            step_line_pid();
            check_encoders(p_enc);
        break;

        case M_MOVE:
            if (b_stop_on_line && line_in_view())
            {
                /* Line search: the line is under a sensor NOW.  Stop here
                   (unlimited brake) rather than turning past it. */
                motor_brake();
                set_state(M_IDLE);
                s5_post_event(S5_EVT_MOVE_DONE);
            }
            else if (move_complete(p_enc))
            {
                motor_brake();
                set_state(M_IDLE);
                s5_post_event(S5_EVT_MOVE_DONE);   /* MOVE_DONE */
            }
            else
            {
                duty = get_move_duty();
                drive_balanced(p_enc, duty);
                check_encoders(p_enc);
            }
        break;

        default:
            /* M_INIT, M_IDLE, M_STALL: held braked. */
            motor_brake();
        break;
    }

    /* Stall is judged on the duty the wheels ACTUALLY get (after the
       slew limit, one wheel stopped in a pivot), not the cruise
       setpoint: a 25% pivot was flagged as a 40% stall. */
    if ((0u != duty) && is_stalled(p_enc, applied_max_duty()))
    {
        motor_brake();
        fault_set(FAULT_MOTOR_STALL);
        set_state(M_STALL);
        duty = 0u;
    }

    return (duty);
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT void s2_set_stop_on_line(bool b_on)
{
    b_stop_on_line = b_on;
}

EXPORT ER s2_motion_init(void)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (INT)APP_MBF_BUFSZ(S2_CMD_QUEUE_DEPTH, MSG_LEN_MOTION_CMD);
    cmbf.maxmsz = (INT)MSG_LEN_MOTION_CMD;

    h_mbf_cmd = tk_cre_mbf(&cmbf);

    ctl = 0u;
    set_state(M_INIT);
    ctl_put(CTL_SETPT_SHIFT, CTL_PCT_BITS, PROV_CRUISE_PCT);

    return ((h_mbf_cmd < E_OK) ? (ER)h_mbf_cmd : E_OK);
}

EXPORT void motion_task(INT stacd, void *p_exinf)
{
    gpio_irq_encoders_t enc;
    msg_motion_cmd_t    cmd;
    uint16_t            fault_bits;
    uint8_t             duty = 0u;

    (void)stacd;
    (void)p_exinf;

    set_state(M_IDLE);

    for (;;)
    {
        (void)app_wait_period();

        /* ---- 1. e-stop first ---- */
        fault_bits = fault_get();

        /* ---- 2. encoders (always, so odometry never skips a cycle) ---- */
        gpio_irq_read_encoders(&enc);
        update_odometry(&enc);

        if (!handle_estop(fault_bits))
        {
            /* A DEGRADE fault caps the speed; clearing it restores it. */
            ctl_put(CTL_SETPT_SHIFT, CTL_PCT_BITS,
                    (0u != (fault_bits & FAULT_MASK_DEGRADE))
                    ? PROV_DEGRADE_PCT : PROV_CRUISE_PCT);

            /* ---- 3. commands ---- */
            if (take_command(&cmd))
            {
                handle_command(&cmd, &enc);
            }

            /* ---- 4. state machine ---- */
            duty = step_state(&enc);
        }
        else
        {
            duty = 0u;
        }

        /* ---- 5. Telemetry's two fields, under its mutex ---- */
        s1_publish_motion(duty, encoder_total);
    }
}

EXPORT s2_state_t s2_get_state(void)
{
    return (get_state());
}

EXPORT void s2_get_speeds(int32_t *p_left, int32_t *p_right)
{
    if (NULL != p_left)
    {
        *p_left = speed_left_cps;
    }

    if (NULL != p_right)
    {
        *p_right = speed_right_cps;
    }
}

EXPORT int32_t s2_get_distance_mm(void)
{
    return (distance_mm);
}

EXPORT ER s2_send_command(const msg_motion_cmd_t *p_cmd)
{
    uint8_t buf[MSG_LEN_MOTION_CMD];
    uint8_t len;
    ER      ercd = E_PAR;

    if (NULL != p_cmd)
    {
        len = msg_pack_motion_cmd(p_cmd, buf, (uint8_t)sizeof(buf));

        if (0u != len)
        {
            /* TMO_POL: the Controller must never block behind Motion. */
            ercd = tk_snd_mbf(h_mbf_cmd, buf, (INT)len, TMO_POL);
        }
    }

    return (ercd);
}

/* end of file */