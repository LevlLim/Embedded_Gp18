/*
 * vehicle_controller.c
 *
 * Subsystem 5 -- the mission state machine.
 * System-level statechart, Design Review V2 section 5.
 *
 * THE SINGLE DECISION POINT.  Only this task sends motion commands, and
 * only this task clears fault bits (WIFI_LOSS excepted).  Every other
 * subsystem reports; none of them drives.
 *
 * The emergency-stop path deliberately BYPASSES this task: fault setters
 * write the bit and Motion Control brakes on it directly.  This task then
 * catches up and decides what the mission does next.
 *
 * MISSION STATE IS ONE BYTE (CR1 item 5).  Parent (3 bits) and substate
 * (2 bits) are stored ONLY in their encoded form, so telemetry on the
 * other core reads one byte and can never see a parent from one moment
 * with a substate from another.
 *
 * OBSTACLE BYPASS (FR8), driven as encoder legs, side S = chosen side,
 * every turn S5_BYPASS_TURN_DEG (45: a diagonal out and back, which is
 * shorter and finds the line at an angle; 90: the old square path):
 *   0 turn toward S        1 forward: clear the obstacle sideways
 *   2 turn back            3 forward: pass the obstacle
 *   4 turn back            5 forward toward the line, watching for it
 *   line found during 5 -> 7 creep the axle onto it -> 6 turn toward S
 *   (realign) -> line follow
 * The forward D_stop guard is on only during the forward legs.
 *
 * JUNCTIONS: a barcode does not turn the car at once.  Its command is
 * stored and carried out at the NEXT junction (both line sensors black):
 *   LEFT / RIGHT / U-TURN -> forward S5_SENSOR_TO_AXLE_MM, then turn
 *   STRAIGHT or none      -> carry on following the line
 */

#include "subsystem5.h"
#include "../fault.h"
#include "../drivers/rp2040_regs.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem2/motor_control.h"
#include "../drivers/gpio_irq.h"

#define VC_WAIT_MS              (20)    /* event wait = button poll rate  */
#define VC_BUTTON_DOWN_MASK     (0x07u) /* 3 samples (>= 40 ms) pressed   */
#define VC_LEG_ALIGN            (6u)
#define VC_LEG_CREEP            (7u)    /* sensors on the line: axle up */
#define VC_LEG_CHECK            (8u)    /* after leg 0: side clear?     */
#define VC_LEG_SQUARE           (9u)    /* too close: 45 more, go square */
#define VC_CHECK_PINGS          (3u)    /* side readings for the check   */
#define VC_LEG_PATHCHK          (10u)   /* after leg 4: way to line clear?*/
#define VC_LEG_UNTURN           (11u)   /* not passed yet: parallel again */
#define VC_LEG_STEP             (12u)   /* ...a step further on           */
#define VC_LEG_NONE             (0xFFu)
#ifndef S5_LINE_MAX_45_MM               /* line crossed at 45 degrees:  */
#define S5_LINE_MAX_45_MM       (80)    /* tape + sensor gap, see .h    */
#endif
#ifndef S5_BEAM_HALF_DEG            /* see subsystem5.h */
#define S5_BEAM_HALF_DEG        (10)
#endif
#ifndef S5_PASS_MAX_CM              /* see subsystem5.h */
#define S5_PASS_MAX_CM          (50u)
#endif
#ifndef S5_JUNCTION_TURN_DEG            /* barcode turn at a junction:  */
#define S5_JUNCTION_TURN_DEG    (90u)   /* less lets line following     */
#endif                                  /* finish it                    */
#define VC_SEARCH_STEP_DEG      (20u)
#define VC_MIN_CONFIDENCE       (4u)
#define VC_LOST_GRACE_MS        (150u)  /* settle after a manoeuvre      */
#define VC_LOST_SEARCH_MS       (600u)  /* lost this long before search: */
                                        /* the PID swings back first     */

/* ---------------------------------------------------------------------
 * Private state.  Written by controller_task alone.
 * --------------------------------------------------------------------- */

static ID h_flg_events = (ID)0;

/* The encoded mission state: the ONLY copy of parent and substate. */
static volatile uint8_t mission = 0u;

static uint8_t  cmd_seq         = 0u;
static bool     b_cal_ok        = false;
static uint8_t  start_history   = 0xFFu;   /* 1 = released (active low) */
static uint8_t  stop_history    = 0xFFu;

static msg_obstacle_t obstacle;
static uint8_t  bypass_leg      = VC_LEG_NONE;
static uint8_t  avoid_retries   = 0u;
static uint16_t lateral_mm      = 0u;
static uint8_t  stripe_step     = 0u;   /* leg 5: 0 wait white, 1 white, */
                                        /* 2 on black, 3 white after it  */
static int32_t  stripe_start_mm = 0;
static int32_t  stripe_mid_mm   = 0;
static uint16_t creep_mm        = 0u;
static uint8_t  side_step       = 0u;   /* leg 3: 0 not seen, 1 seeing, */
                                        /* 2 passed and reported         */
static uint32_t side_seen_count = 0u;   /* last side ping handled        */
static int32_t  side_start_mm   = 0;
static uint8_t  side_last_cm    = 0u;   /* last reading on the obstacle  */
static uint8_t  side_far_pings  = 0u;
static uint32_t turn_deg        = S5_BYPASS_TURN_DEG;  /* this bypass  */
static uint8_t  check_pings     = 0u;
static uint8_t  check_min_cm    = 0u;
static uint8_t  path_tries      = 0u;
static int32_t  pass_start_mm   = 0;    /* leg 3 start (left wheel)      */
static uint32_t pass_min_mm     = 0u;   /* past the near face, no more   */
static uint32_t pass_ahead_mm   = 0u;   /* forward already made, leg 1   */
static int32_t  pass_driven_mm  = 0;    /* leg 3 travel, side look blind */

static uint32_t search_start_ms = 0u;
static uint32_t follow_since_ms = 0u;
static uint32_t lost_since_ms   = 0u;
static bool     b_was_lost      = false;
static uint8_t  search_step     = 0u;
/* Manual move from Telemetry (core 1) to the Controller (core 0), CR1
   item 6: ONE word, bits 0-7 = command type, bits 8-23 = argument, so a
   second command can never pair its type with the first one's argument. */
#define VC_MANUAL_ARG_SHIFT     (8u)
static volatile uint32_t manual_word = 0u;    /* written by Telemetry */
static bool     b_search_right  = false;   /* first turn toward the line */
static uint8_t  pending_turn    = (uint8_t)MSG_BC_NONE; /* for next junction */
static uint8_t  junction_step   = 0u;      /* 1 = to the axle, 2 = turning */
static volatile uint8_t injected_turn = 0u;  /* written by Telemetry */

/* ---------------------------------------------------------------------
 * Mission state (one byte)
 * --------------------------------------------------------------------- */

static s5_parent_t get_parent(void)
{
    uint8_t parent = 0u;

    msg_decode_veh_state(mission, &parent, NULL);

    return ((s5_parent_t)parent);
}

static s5_sub_t get_sub(void)
{
    uint8_t sub = 0u;

    msg_decode_veh_state(mission, NULL, &sub);

    return ((s5_sub_t)sub);
}

static void set_mission(s5_parent_t parent, s5_sub_t sub)
{
    /* One store: readers see the old state or the new one, never a mix. */
    mission = msg_encode_veh_state((uint8_t)parent, (uint8_t)sub);
}

/* ---------------------------------------------------------------------
 * Commands
 * --------------------------------------------------------------------- */

static void send(msg_cmd_type_t type, uint32_t arg)
{
    msg_motion_cmd_t cmd;

    cmd.type = type;
    cmd.arg  = msg_sat_u16(arg, (uint16_t)MSG_MAX_CMD_ARG, NULL);
    cmd.seq  = (uint8_t)((uint32_t)cmd_seq & 0x07u);
    cmd_seq  = (uint8_t)(cmd_seq + 1u);

    if (E_OK != s2_send_command(&cmd))
    {
        APP_LOG((UB*)"[S5] motion queue full, cmd %d lost\n", (INT)type);
    }
}

static void enter_line_follow(void)
{
    bypass_leg    = VC_LEG_NONE;
    junction_step = 0u;
    s5_obstacle_guard(false);
    s5_obstacle_request(S5_REQ_RESUME);
    set_mission(S5_PARENT_RUNNING, S5_SUB_LINE_FOLLOW);
    follow_since_ms = app_now_ms();
    b_was_lost      = false;
    s2_set_stop_on_line(false);
    send(MSG_CMD_LINE_PID, 0u);
}

static void enter_running(void)
{
    s3_set_active(true);
    s4_set_active(true);
    avoid_retries = 0u;
    pending_turn  = (uint8_t)MSG_BC_NONE;
    junction_step = 0u;
    s3_force_straight(false);
    enter_line_follow();
    APP_LOG((UB*)"[S5] RUNNING\n");
}

static void enter_halted(s5_parent_t parent)
{
    /* Stop Motion Control too.  An E-STOP brakes it anyway, but RESET
       sent while RUNNING sets no stop bit: Motion stayed in line
       following with the sensors switched off, read "line lost" and
       pivoted left for ever (duty L 0 R 32, err 0 --). */
    send(MSG_CMD_STOP, 0u);
    s2_set_stop_on_line(false);
    s3_set_active(false);
    s4_set_active(false);
    s5_obstacle_guard(false);
    bypass_leg = VC_LEG_NONE;
    set_mission(parent, S5_SUB_LINE_FOLLOW);
}

static void enter_search(void)
{
    msg_line_t line = {0};
    uint32_t   word = 0u;

    bypass_leg = VC_LEG_NONE;
    s5_obstacle_guard(false);
    s5_obstacle_request(S5_REQ_RESUME);     /* keep watching while searching */
    set_mission(S5_PARENT_RUNNING, S5_SUB_LINE_SEARCH);
    search_start_ms = app_now_ms();
    search_step     = 1u;

    /* Turn FIRST toward where the line was last seen: while lost, the
       line word keeps the saturated error in the direction it left. */
    b_search_right = false;
    if (s3_read_line_word(&word))
    {
        msg_unpack_line(word, &line);
        b_search_right = (line.error > 0);
    }

    s2_set_stop_on_line(true);
    send(b_search_right ? MSG_CMD_TURN_R : MSG_CMD_TURN_L,
         VC_SEARCH_STEP_DEG);
}

/* ---------------------------------------------------------------------
 * Obstacle bypass
 * --------------------------------------------------------------------- */

/* Turn toward (b_out) or away from the chosen side. */
static void send_turn(bool b_out)
{
    bool b_left = (MSG_SIDE_LEFT == obstacle.side);

    send((b_left == b_out) ? MSG_CMD_TURN_L : MSG_CMD_TURN_R, turn_deg);
}

/* sin(deg) x 256 for the servo limit (10..60 degrees, every 5). */
static uint32_t sin_q8_deg(uint32_t deg)
{
    static const uint8_t sin_5deg_q8[13] =
    {
        0u, 22u, 44u, 66u, 88u, 108u, 128u, 147u, 165u, 181u, 196u,
        210u, 222u
    };

    return (sin_5deg_q8[((deg > 60u) ? 60u : deg) / 5u]);
}

/* Distance along a diagonal leg that moves the car `side_mm` sideways:
   x 1/sin(45) = x 1.414 at 45 degrees, x 1 at 90. */
static uint32_t diagonal_mm(uint32_t side_mm)
{
    return ((45u == turn_deg) ? ((side_mm * 1414u) / 1000u) : side_mm);
}

static int32_t left_mm(void);

/*
 * How far the front corner on the obstacle side moves TOWARDS the obstacle
 * when the car pivots 45 degrees back towards the line while still beside
 * it: (front reach + half width) x sin 45 - half width.  Added to the
 * sideways clearance so the path check's turn can never hit a long
 * obstacle.  Front reach = sensor-to-axle (the sensor is at the front).
 */
static int32_t turn_swing_cm(void)
{
    int32_t half  = (int32_t)(S5_ROBOT_WIDTH_CM / 2u);
    int32_t reach = (int32_t)S5_US_TO_AXLE_CM;
    int32_t swing = (((reach + half) * (int32_t)sin_q8_deg(45u)) / 256)
                    - half;

    return ((swing > 0) ? swing : 0);
}

/* The obstacle's size: width from the front scan, length from the side
   look while passing it (leg 3). */
static void report_size(void)
{
    int32_t length_cm = (left_mm() - side_start_mm) / 10;

    if (1u == side_step)
    {
        APP_LOG((UB*)"[S5] obstacle size: width %d cm, length over %d cm "
                     "(still beside the car at the end of the leg)\n",
                (INT)obstacle.width_cm, (INT)length_cm);
    }
    else if (0u == side_step)
    {
        APP_LOG((UB*)"[S5] obstacle size: width %d cm, length not "
                     "measured (side sensor never saw it)\n",
                (INT)obstacle.width_cm);
    }
    else if (3u == side_step)
    {
        APP_LOG((UB*)"[S5] obstacle size: width %d cm, length not seen "
                     "by the side look (path check measures it)\n",
                (INT)obstacle.width_cm);
    }
    else
    {
        /* Reported when the sensor lost it (watch_side). */
    }
    side_step = 2u;
}

/*
 * Leg 3: pass the obstacle, looking at its side.
 */
static void start_pass(void)
{
    uint32_t pass_mm;
    uint32_t ahead_mm;

    /* Look at the obstacle while passing it: measures its length
       and ends this leg as soon as it is behind (watch_side). */
    side_step = 0u;
    (void)s5_get_side_cm(&side_seen_count);
    s5_obstacle_request((MSG_SIDE_RIGHT == obstacle.side)
                        ? S5_REQ_SIDE_L : S5_REQ_SIDE_R);
    /* The obstacle's LENGTH is not known (nor assumed): drive
       on until the side look loses it (watch_side), up to
       S5_PASS_MAX_CM.  If the side look never sees it at all,
       stop once the axle is past the near face (pass_min_mm)
       and let the path check (leg 10) find the far end.
       distance_cm is from the SENSOR; the axle is
       S5_US_TO_AXLE_CM behind it.  A 45 degree leg 1 already
       moved the car forward as far as it moved sideways. */
    ahead_mm = (45u == turn_deg) ? (uint32_t)lateral_mm : 0u;
    /* If the side look stays blind, do not turn back at the near face:
       the pivot swings the front corner into the obstacle (test 10, leg 4
       stalled against the block).  Go on by the MEASURED width first (an
       upper guess at its length, nothing fixed); the path check then
       confirms the way is clear before the car heads for the line. */
    pass_mm  = ((uint32_t)obstacle.distance_cm + obstacle.width_cm
                + S5_US_TO_AXLE_CM + S5_CLEARANCE_MARGIN_CM) * 10u;
    pass_min_mm   = (pass_mm > (ahead_mm + 50u))
                    ? (pass_mm - ahead_mm) : 50u;
    pass_start_mm = left_mm();
    pass_ahead_mm = ahead_mm;
    pass_mm = (uint32_t)S5_PASS_MAX_CM * 10u;
    send(MSG_CMD_FORWARD, (pass_mm > pass_min_mm)
                          ? pass_mm : pass_min_mm);
}

/* Issue bypass leg number 'leg'. */
static void run_leg(uint8_t leg)
{
    bypass_leg = leg;
    s5_obstacle_guard((1u == leg) || (3u == leg) || (5u == leg)
                      || (VC_LEG_STEP == leg));

    APP_LOG((UB*)"[S5] bypass leg %d (%s side)\n", (INT)leg,
            (MSG_SIDE_LEFT == obstacle.side) ? "left" : "right");

    switch (leg)
    {
        case 0u:
            turn_deg   = S5_BYPASS_TURN_DEG;
            path_tries = 0u;
            send_turn(true);
        break;

        case VC_LEG_PATHCHK:
            /* Facing the way to the line: anything in it? */
            check_pings  = 0u;
            check_min_cm = 0xFFu;
            (void)s5_get_side_cm(&side_seen_count);
            s5_obstacle_request(S5_REQ_MEASURE);
        break;

        case VC_LEG_UNTURN:
            send_turn(true);            /* parallel to the line again */
        break;

        case VC_LEG_STEP:
            send(MSG_CMD_FORWARD, S5_PATH_STEP_CM * 10u);
        break;

        case VC_LEG_CHECK:
            /* Stopped at 45 degrees: look back towards the obstacle. */
            check_pings  = 0u;
            check_min_cm = 0xFFu;
            (void)s5_get_side_cm(&side_seen_count);
            s5_obstacle_request((MSG_SIDE_RIGHT == obstacle.side)
                                ? S5_REQ_SIDE_L : S5_REQ_SIDE_R);
        break;

        case VC_LEG_SQUARE:
            /* The other 45 degrees: the rest of the bypass is square. */
            send_turn(true);
            turn_deg = 90u;
        break;

        case 1u:
            send(MSG_CMD_FORWARD, diagonal_mm(lateral_mm));
        break;

        case 3u:
            start_pass();
        break;

        case 5u:
            stripe_step = 0u;
            send(MSG_CMD_FORWARD, diagonal_mm(lateral_mm) + 150u);
        break;

        case VC_LEG_CREEP:
            send(MSG_CMD_FORWARD, creep_mm);
        break;

        case VC_LEG_ALIGN:
            send_turn(true);
        break;

        default:
            /* Legs 2 and 4: turn back. */
            if (4u == leg)
            {
                report_size();
                s5_obstacle_request(S5_REQ_AHEAD);
            }
            send_turn(false);
        break;
    }
}

/* Reverse S5_REVERSE_MM and profile again when that is done. */
static void back_off(const char *p_why)
{
    avoid_retries = (uint8_t)(avoid_retries + 1u);
    bypass_leg    = VC_LEG_NONE;
    APP_LOG((UB*)"[S5] %s: back off %d mm and rescan (%d)\n", p_why,
            (INT)S5_REVERSE_MM, (INT)avoid_retries);
    send(MSG_CMD_REVERSE, S5_REVERSE_MM);
}

/*
 * A profile arrived: plan the bypass, back off and rescan, or give up.
 */
static void handle_profile(void)
{
    int32_t edge_cm;
    int32_t edge_left;
    int32_t edge_right;

    if (!s5_take_obstacle(&obstacle))
    {
        return;
    }

    APP_LOG((UB*)"[S5] profile: %d cm at %d deg, width %d cm, clear "
                 "L %d R %d cm -> %s\n", (INT)obstacle.distance_cm,
            (INT)obstacle.bearing_deg, (INT)obstacle.width_cm,
            (INT)obstacle.clear_left_cm, (INT)obstacle.clear_right_cm,
            (MSG_SIDE_LEFT == obstacle.side) ? "LEFT"
            : ((MSG_SIDE_RIGHT == obstacle.side) ? "RIGHT" : "NONE"));

    if (0u == obstacle.distance_cm)
    {
        /* False alarm: nothing confirmed by the fine scan. */
        enter_line_follow();
    }
    else if (s5_last_scan_clipped() && (avoid_retries < S5_MAX_RETRIES))
    {
        /* The obstacle filled the scan out to an end of the arc: its edge
           was not seen, so neither its width nor the side clearance can
           be trusted.  Further back it spans less of the arc. */
        back_off("obstacle fills the scan, its edge unseen");
    }
    else if (MSG_SIDE_NONE != obstacle.side)
    {
        /* Lateral distance from the car's centre line to the obstacle's
           edge on the chosen side, plus half the car and the margin.
           This used to be width/2 either side of the NEAREST reading's
           bearing, which is not the obstacle's centre: nearest at -20
           deg put the right edge at -2 cm instead of +8, so the car went
           out 10 cm too little and its front corner hit (leg 2/3). */
        s5_get_edges_cm(&edge_left, &edge_right);
        edge_cm = (MSG_SIDE_LEFT == obstacle.side) ? -edge_left : edge_right;
        edge_cm = (edge_cm < 0) ? 0 : edge_cm;
        APP_LOG((UB*)"[S5] edges L %d R %d cm from the centre line\n",
                (INT)edge_left, (INT)edge_right);
        /* Going round on the RIGHT puts the obstacle on the car's left,
           where the barcode sensor sticks out (bench 17 Nov: it hit the
           obstacle). */
        edge_cm = edge_cm + ((MSG_SIDE_RIGHT == obstacle.side)
                             ? (int32_t)S5_LEFT_OVERHANG_CM
                             : (int32_t)S5_RIGHT_OVERHANG_CM);
        lateral_mm = msg_sat_u16((uint32_t)(edge_cm
                                 + (int32_t)(S5_ROBOT_WIDTH_CM / 2u)
                                 + turn_swing_cm()
                                 + (int32_t)S5_CLEARANCE_MARGIN_CM) * 10u,
                                 (uint16_t)MSG_MAX_CMD_ARG, NULL);
        run_leg(0u);
    }
    else if (avoid_retries < S5_MAX_RETRIES)
    {
        /* Neither side fits: back off and look again (REVERSE_RETRY). */
        back_off("no side fits");
    }
    else
    {
        /* ABORT: retries exhausted.  Stop and report through the Fault
           Table so the operator sees why. */
        APP_LOG((UB*)"[S5] bypass aborted after %d retries\n",
                  (INT)avoid_retries);
        fault_set(FAULT_OBSTACLE_STOP);
    }
}

/* A bypass leg finished: start the next one. */
static void bypass_move_done(void)
{
    if (VC_LEG_NONE == bypass_leg)
    {
        /* The reverse of a retry finished: profile again. */
        s5_obstacle_request(S5_REQ_SCAN);
    }
    else if (VC_LEG_ALIGN == bypass_leg)
    {
        enter_line_follow();            /* line reacquired (FR8) */
    }
    else if (VC_LEG_CREEP == bypass_leg)
    {
        run_leg(VC_LEG_ALIGN);          /* axle over the line: turn */
    }
    else if ((0u == bypass_leg) && (45u == turn_deg))
    {
        run_leg(VC_LEG_CHECK);          /* is the diagonal clear? */
    }
    else if (VC_LEG_SQUARE == bypass_leg)
    {
        run_leg(1u);
    }
    else if (4u == bypass_leg)
    {
        run_leg(VC_LEG_PATHCHK);        /* is the way to the line clear? */
    }
    else if (VC_LEG_UNTURN == bypass_leg)
    {
        run_leg(VC_LEG_STEP);
    }
    else if (VC_LEG_STEP == bypass_leg)
    {
        run_leg(4u);                    /* turn back and look again */
    }
    else if ((VC_LEG_CHECK == bypass_leg)
             || (VC_LEG_PATHCHK == bypass_leg))
    {
        /* Waiting for readings (poll_running). */
    }
    else if (bypass_leg < 5u)
    {
        run_leg((uint8_t)(bypass_leg + 1u));
    }
    else
    {
        enter_search();                 /* leg 5 ended without the line */
    }
}

/*
 * A manoeuvre finished.  What comes next depends on the substate.
 */
static void handle_move_done(void)
{
    s5_sub_t sub = get_sub();

    if ((S5_SUB_BARCODE_ACTION == sub) && (1u == junction_step))
    {
        /* At the junction centre: now turn. */
        junction_step = 2u;
        if (MSG_BC_LEFT == pending_turn)
        {
            send(MSG_CMD_TURN_L, S5_JUNCTION_TURN_DEG);
        }
        else if (MSG_BC_RIGHT == pending_turn)
        {
            send(MSG_CMD_TURN_R, S5_JUNCTION_TURN_DEG);
        }
        else
        {
            send(MSG_CMD_U_TURN, 180u);
        }
        pending_turn = (uint8_t)MSG_BC_NONE;
    }
    else if (S5_SUB_BARCODE_ACTION == sub)
    {
        junction_step = 0u;
        enter_line_follow();
    }
    else if (S5_SUB_OBSTACLE_AVOID == sub)
    {
        bypass_move_done();
    }
    else if ((S5_SUB_LINE_SEARCH == sub)
             && (MSG_LINE_TRACK == s3_get_line_state()))
    {
        /* The turn stopped because the line came into view. */
        enter_line_follow();
    }
    else if (S5_SUB_LINE_SEARCH == sub)
    {
        /* Widening sweep: alternate sides, one step wider each time. */
        search_step = (uint8_t)(search_step + 1u);
        send((((search_step % 2u) == 1u) == b_search_right)
                 ? MSG_CMD_TURN_R : MSG_CMD_TURN_L,
             (uint32_t)search_step * VC_SEARCH_STEP_DEG);
    }
    else
    {
        /* LINE_FOLLOW: nothing was pending. */
    }
}

/* Name of a barcode command, for the console. */
static const char *turn_name(uint8_t cmd)
{
    const char *p_name = "straight on";

    if (MSG_BC_LEFT == cmd)
    {
        p_name = "turn LEFT";
    }
    else if (MSG_BC_RIGHT == cmd)
    {
        p_name = "turn RIGHT";
    }
    else if (MSG_BC_U_TURN == cmd)
    {
        p_name = "U-TURN";
    }
    else
    {
        /* STRAIGHT or none. */
    }

    return (p_name);
}

/* Keep a junction command for the next junction; a newer one replaces
   it.  STRAIGHT also tells the line sensor not to treat that junction as a
   corner. */
static void store_turn(uint8_t cmd, const char *p_from)
{
    pending_turn = cmd;
    s3_force_straight(MSG_BC_STRAIGHT == cmd);
    APP_LOG((UB*)"[S5] %s %s: will be done at the next junction\n",
            p_from, turn_name(pending_turn));
}

/*
 * Store a decoded barcode for the next junction.  A low-confidence
 * decode is ignored: a missed command is recoverable, a wrong turn is not.
 */
static void handle_barcode(void)
{
    msg_barcode_t bc;
    bool          b_got;

    b_got = s3_take_barcode(&bc);
    if (b_got)
    {
        APP_LOG((UB*)"[S5] barcode cmd %d conf %d%s\n", (INT)bc.cmd,
                  (INT)bc.confidence,
                  (bc.confidence >= VC_MIN_CONFIDENCE) ? "" : " (ignored)");
    }

    if (b_got && (bc.confidence >= VC_MIN_CONFIDENCE)
        && (MSG_BC_INVALID != bc.cmd) && (MSG_BC_NONE != bc.cmd))
    {
        store_turn((uint8_t)bc.cmd, "barcode");
    }
}

/* A test turn from MQTT (jleft, jright, ...): same as a barcode. */
static void handle_test_turn(void)
{
    store_turn(injected_turn, "test");
}

/*
 * Both line sensors black: a junction.  Carry out the stored barcode.
 */
static void handle_junction(void)
{
    if (S5_SUB_LINE_FOLLOW != get_sub())
    {
        /* Mid-manoeuvre (a junction turn crosses the lines): ignore. */
    }
    else if ((MSG_BC_LEFT == pending_turn) || (MSG_BC_RIGHT == pending_turn)
             || (MSG_BC_U_TURN == pending_turn))
    {
        APP_LOG((UB*)"[S5] junction %d: %s\n",
                (INT)s3_get_junction_count(), turn_name(pending_turn));
        set_mission(S5_PARENT_RUNNING, S5_SUB_BARCODE_ACTION);
        junction_step = 1u;
        send(MSG_CMD_FORWARD, S5_SENSOR_TO_AXLE_MM);
    }
    else
    {
        /* STRAIGHT or nothing stored: keep following the line. */
        APP_LOG((UB*)"[S5] junction %d: %s\n",
                (INT)s3_get_junction_count(), turn_name(pending_turn));
        pending_turn = (uint8_t)MSG_BC_NONE;
    }
}

/* Left-wheel distance, mm (the right encoder is noisy on this car). */
static int32_t left_mm(void)
{
    gpio_irq_encoders_t enc = {0};

    gpio_irq_read_encoders(&enc);

    return (motor_counts_to_mm(enc.count_left, true));
}

/*
 * Leg 5: find the track line.  It is a narrow black stripe with white on
 * both sides.  A barcode sheet beside the line is not: its bars run along
 * the car's path here, so the sensor stays black (or white) far longer,
 * or meets the next bar within a few mm.  Turning on the first black
 * (test 9: leg 4 "found" a bar and realigned on the barcode sheet) is
 * gone; once the stripe is confirmed the axle is crept up to its centre.
 */
static void watch_for_line(void)
{
    bool    b_black = s3_line_seen();
    int32_t now_mm  = left_mm();
    int32_t ahead_mm;
    int32_t max_mm  = (45u == turn_deg) ? S5_LINE_MAX_45_MM
                                        : S5_LINE_MAX_MM;

    if (0u == stripe_step)
    {
        stripe_step = b_black ? 0u : 1u;          /* need white first   */
    }
    else if (1u == stripe_step)
    {
        /* On white, waiting for black.  (Was: only "1 and black" here, so
           white on step 1 fell to the final else and reset to 0 -- the
           step flipped 0/1 every poll, and black arriving on a 0 poll was
           ignored for its whole width: the car drove over the line about
           half the time, bench 17 Nov.) */
        if (b_black)
        {
            stripe_start_mm = now_mm;
            stripe_step     = 2u;
        }
    }
    else if (2u == stripe_step)
    {
        if (!b_black)
        {
            ahead_mm = now_mm - stripe_start_mm;  /* stripe width       */
            stripe_mid_mm = stripe_start_mm + (ahead_mm / 2);
            stripe_step   = ((ahead_mm >= S5_LINE_MIN_MM)
                             && (ahead_mm <= max_mm)) ? 3u : 1u;
            if (1u == stripe_step)
            {
                APP_LOG((UB*)"[S5] stripe %d mm: too narrow, not the line\n",
                        (INT)ahead_mm);
            }
        }
        else if ((now_mm - stripe_start_mm) > max_mm)
        {
            APP_LOG((UB*)"[S5] stripe over %d mm: too wide, not the line\n",
                    (INT)max_mm);
            stripe_step = 0u;                     /* too wide: not it   */
        }
        else
        {
            /* Still on the stripe. */
        }
    }
    else if (3u == stripe_step)
    {
        if (b_black)
        {
            APP_LOG((UB*)"[S5] stripe followed by another within %d mm: "
                         "a barcode, not the line\n",
                    (INT)(now_mm - stripe_mid_mm));
            stripe_step = 2u;                     /* another bar: not   */
            stripe_start_mm = now_mm;             /* the line; restart  */
        }
        else if ((now_mm - stripe_mid_mm) >= (S5_LINE_WHITE_MM
                                               + (S5_LINE_MIN_MM / 2)))
        {
            /* Confirmed.  Creep the rest of the way so the AXLE ends
               over the line centre, then turn. */
            ahead_mm = (int32_t)S5_SENSOR_TO_AXLE_MM - (now_mm - stripe_mid_mm);
            creep_mm = (uint16_t)((ahead_mm > 0) ? ahead_mm : 0);
            APP_LOG((UB*)"[S5] line found (%d mm stripe): creep %d mm\n",
                    (INT)((stripe_mid_mm - stripe_start_mm) * 2),
                    (INT)creep_mm);
            stripe_step = 0u;
            run_leg(VC_LEG_CREEP);
        }
        else
        {
            /* Checking for white after the stripe. */
        }
    }
    else
    {
        stripe_step = 0u;
    }
}

/*
 * Leg 3: the servo looks at the obstacle's side.  Note where the sensor
 * first sees it and where it loses it again: the travel between is the
 * obstacle's length.  Once lost, the far end is (last reading x cos 30)
 * ahead of the sensor, so drive just that + sensor-to-axle + the usual
 * extra and margin, then turn back -- instead of the full worst-case pass.
 */
static void watch_side(void)
{
    uint32_t count;
    uint8_t  cm = s5_get_side_cm(&count);
    bool     b_near;
    int32_t  length_cm;
    uint32_t rest_mm;

    if ((0u == side_step)
        && ((left_mm() - pass_start_mm) >= (int32_t)pass_min_mm))
    {
        /* Past the near face and the side look has seen nothing: do not
           guess the length, turn back and let the path check look. */
        APP_LOG((UB*)"[S5] side look saw nothing: turning back, the path "
                     "check finds the far end\n");
        side_step      = 3u;
        pass_driven_mm = left_mm() - pass_start_mm;
        run_leg(4u);                    /* replaces leg 3 */
    }
    else if (count != side_seen_count)
    {
        side_seen_count = count;
        b_near = (0u != cm) && (cm <= S5_SIDE_NEAR_CM);

        if ((0u == side_step) && b_near)
        {
            side_start_mm  = left_mm();
            side_last_cm   = cm;
            side_far_pings = 0u;
            side_step      = 1u;
        }
        else if ((1u == side_step) && b_near)
        {
            side_last_cm   = cm;
            side_far_pings = 0u;
        }
        else if (1u == side_step)
        {
            side_far_pings = (uint8_t)(side_far_pings + 1u);
            if (side_far_pings >= S5_SIDE_LOST_PINGS)
            {
                length_cm = (left_mm() - side_start_mm) / 10;
                rest_mm   = ((((uint32_t)side_last_cm * 866u) / 1000u)
                             + S5_US_TO_AXLE_CM + S5_PASS_EXTRA_CM
                             + S5_CLEARANCE_MARGIN_CM) * 10u;
                APP_LOG((UB*)"[S5] obstacle size: width %d cm, length %d cm"
                             " -> %d mm more, then turn back\n",
                        (INT)obstacle.width_cm, (INT)length_cm,
                        (INT)rest_mm);
                side_step = 2u;
                send(MSG_CMD_FORWARD, rest_mm);     /* replaces leg 3 */
            }
        }
        else
        {
            /* Not seen yet, or already passed. */
        }
    }
    else
    {
        /* No new side reading yet. */
    }
}

/*
 * After the 45 degree turn: is the diagonal clear of the obstacle?  The
 * servo turns to its limit back towards the obstacle; a return at r cm
 * is r x sin(limit) from the line the car's centre will drive along
 * (the sensor is on that line).  If that is less than half the car, the
 * overhang on that side and the margin, the car's side would scrape the
 * obstacle (bench 17 Nov): turn the other 45 degrees and go round
 * square instead.  Nothing seen = clear.
 */
static void check_diagonal(void)
{
    uint32_t count;
    uint8_t  cm = s5_get_side_cm(&count);
    uint32_t need_cm;
    uint32_t clear_cm;

    if (count != side_seen_count)
    {
        side_seen_count = count;
        check_pings     = (uint8_t)(check_pings + 1u);
        if ((0u != cm) && (cm < check_min_cm))
        {
            check_min_cm = cm;
        }
    }

    if (check_pings >= VC_CHECK_PINGS)
    {
        need_cm  = (S5_ROBOT_WIDTH_CM / 2u) + S5_CLEARANCE_MARGIN_CM
                 + ((MSG_SIDE_RIGHT == obstacle.side)
                    ? (uint32_t)S5_LEFT_OVERHANG_CM
                    : (uint32_t)S5_RIGHT_OVERHANG_CM);
        clear_cm = (0xFFu == check_min_cm) ? 0xFFu
                 : (((uint32_t)check_min_cm
                     * sin_q8_deg((uint32_t)S5_SERVO_LIMIT_DEG)) / 256u);
        s5_obstacle_request(S5_REQ_AHEAD);

        if (clear_cm < need_cm)
        {
            APP_LOG((UB*)"[S5] 45 deg path: obstacle %d cm from it, needs "
                         "%d: going round square\n", (INT)clear_cm,
                    (INT)need_cm);
            run_leg(VC_LEG_SQUARE);
        }
        else
        {
            APP_LOG((UB*)"[S5] 45 deg path clear (%d cm, needs %d)\n",
                    (INT)((0xFFu == clear_cm) ? 99u : clear_cm),
                    (INT)need_cm);
            run_leg(1u);
        }
    }
}

/*
 * Mid-bypass, after turning back towards the line: the sensor looks along
 * the way to the line (diagonal or square).  A return nearer than that
 * way is the obstacle, not yet passed: turn parallel, drive a step on,
 * turn back and look again.  Clear (or out of tries): drive to the line.
 */
/*
 * The side look never saw the obstacle: its length is at most how far
 * the axle got past the near face before the path check saw a clear way.
 */
static void report_path_length(void)
{
    int32_t far_mm;

    if (3u == side_step)
    {
        far_mm = (int32_t)pass_ahead_mm + pass_driven_mm
                 + ((int32_t)path_tries * (int32_t)S5_PATH_STEP_CM * 10)
                 - (((int32_t)obstacle.distance_cm
                     + (int32_t)S5_US_TO_AXLE_CM) * 10);
        APP_LOG((UB*)"[S5] obstacle size: width %d cm, length at most "
                     "%d cm (path check)\n",
                (INT)obstacle.width_cm,
                (INT)((far_mm > 0) ? (far_mm / 10) : 0));
    }
    else
    {
        /* Measured by the side look and reported then. */
    }
}

static void check_path(void)
{
    uint32_t count;
    uint8_t  cm = s5_get_side_cm(&count);
    uint32_t way_cm = (diagonal_mm(lateral_mm) / 10u) + 10u;

    if (count != side_seen_count)
    {
        side_seen_count = count;
        check_pings     = (uint8_t)(check_pings + 1u);
        if ((0u != cm) && (cm < check_min_cm))
        {
            check_min_cm = cm;
        }
    }

    if (check_pings >= VC_CHECK_PINGS)
    {
        s5_obstacle_request(S5_REQ_AHEAD);
        if (((uint32_t)check_min_cm < way_cm)
            && (path_tries < S5_PATH_TRIES))
        {
            path_tries = (uint8_t)(path_tries + 1u);
            APP_LOG((UB*)"[S5] obstacle still in the way (%d cm, way to "
                         "the line %d cm): %d cm further on\n",
                    (INT)check_min_cm, (INT)way_cm, (INT)S5_PATH_STEP_CM);
            run_leg(VC_LEG_UNTURN);
        }
        else if ((uint32_t)check_min_cm < way_cm)
        {
            /* Still not past after every step: never drive into it. */
            APP_LOG((UB*)"[S5] obstacle still in the way after %d steps "
                         "(longer than the bypass allows): stopping\n",
                    (INT)path_tries);
            fault_set(FAULT_OBSTACLE_STOP);
        }
        else
        {
            APP_LOG((UB*)"[S5] way to the line clear (%d cm)\n",
                    (INT)((0xFFu == check_min_cm) ? 0u : check_min_cm));
            report_path_length();
            run_leg(5u);
        }
    }
}

/*
 * Polled every loop while RUNNING: line reacquisition and search timeout.
 */
static void poll_running(void)
{
    bool     b_on_line = (MSG_LINE_TRACK == s3_get_line_state());
    bool     b_lost    = (MSG_LINE_LOST == s3_get_line_state());
    s5_sub_t sub       = get_sub();

    /* How long the line has been continuously lost. */
    if (b_lost && !b_was_lost)
    {
        lost_since_ms = app_now_ms();
    }
    b_was_lost = b_lost;

    if ((S5_SUB_LINE_FOLLOW == sub) && b_lost
        && ((app_now_ms() - follow_since_ms) > VC_LOST_GRACE_MS)
        && ((app_now_ms() - lost_since_ms) > VC_LOST_SEARCH_MS))
    {
        enter_search();
    }
    else if ((S5_SUB_OBSTACLE_AVOID == sub) && (5u == bypass_leg))
    {
        watch_for_line();
    }
    else if ((S5_SUB_OBSTACLE_AVOID == sub) && (3u == bypass_leg))
    {
        watch_side();
    }
    else if ((S5_SUB_OBSTACLE_AVOID == sub)
             && (VC_LEG_CHECK == bypass_leg))
    {
        check_diagonal();
    }
    else if ((S5_SUB_OBSTACLE_AVOID == sub)
             && (VC_LEG_PATHCHK == bypass_leg))
    {
        check_path();
    }
    else if (S5_SUB_LINE_SEARCH == sub)
    {
        if (b_on_line)
        {
            enter_line_follow();
        }
        else if ((app_now_ms() - search_start_ms) > S5_SEARCH_TIMEOUT_MS)
        {
            /* Unrecoverable line loss: an E-STOP per the Fault Table. */
            fault_set(FAULT_LINE_LOST_HARD);
        }
        else
        {
            /* Still searching. */
        }
    }
    else
    {
        /* Nothing to poll. */
    }
}

/* ---------------------------------------------------------------------
 * Faults, buttons, start / stop / reset
 * --------------------------------------------------------------------- */

/*
 * Map the fault word onto the mission.  REMOTE_STOP alone is an operator
 * stop (STOPPED); any other E-STOP bit is a FAULT.  DEGRADE bits only cap
 * the speed, which Motion Control applies itself.
 */
static void handle_faults(uint16_t bits)
{
    uint16_t hard = (uint16_t)((uint32_t)bits & FAULT_MASK_ESTOP
                               & ~(uint32_t)FAULT_REMOTE_STOP);
    s5_parent_t parent = get_parent();

    if ((0u != hard) && (S5_PARENT_FAULT != parent))
    {
        APP_LOG((UB*)"[S5] E-STOP: mask 0x%03x\n", (INT)bits);
        enter_halted(S5_PARENT_FAULT);
    }
    else if ((0u != ((uint32_t)bits & FAULT_REMOTE_STOP))
             && (S5_PARENT_RUNNING == parent))
    {
        APP_LOG((UB*)"[S5] STOPPED\n");
        enter_halted(S5_PARENT_STOPPED);
    }
    else
    {
        /* No change. */
    }
}

/* Debounced press: true once, on the third consecutive low sample. */
static bool button_pressed(uint32_t pin, uint8_t *p_history)
{
    uint8_t previous = *p_history;

    *p_history = (uint8_t)(((uint32_t)previous << 1u)
                           | ((0u != gpio_get_val(pin)) ? 1u : 0u));

    return ((0u == ((uint32_t)*p_history & VC_BUTTON_DOWN_MASK))
            && (0u != ((uint32_t)previous & VC_BUTTON_DOWN_MASK)));
}

static void do_reset(void)
{
    /* Sole fault-clearing authority.  WIFI_LOSS belongs to Telemetry. */
    fault_clear((uint16_t)(FAULT_MASK_ALL & ~(uint32_t)FAULT_WIFI_LOSS));
    s4_request_reset();

    if (!b_cal_ok)
    {
        b_cal_ok = (E_OK == s3_calibrate());
    }

    enter_halted(b_cal_ok ? S5_PARENT_READY : S5_PARENT_FAULT);
    APP_LOG((UB*)"[S5] RESET -> %s\n", b_cal_ok ? "READY" : "FAULT");
}

static void do_start(void)
{
    uint16_t hard = (uint16_t)((uint32_t)fault_get() & FAULT_MASK_ESTOP
                               & ~(uint32_t)FAULT_REMOTE_STOP);

    /* START clears only the operator's own stop.  A latched LIFTED,
       TILT or STALL needs an explicit RESET. */
    if (0u == hard)
    {
        fault_clear(FAULT_REMOTE_STOP);
        s4_request_reset();
        enter_running();
    }
}

/*
 * Buttons: STOP sets REMOTE_STOP directly (Motion brakes at once).  START
 * starts from READY/STOPPED and acts as RESET from FAULT.
 */
static uint32_t poll_buttons(void)
{
    uint32_t events = 0u;

    if (button_pressed(PIN_BTN_STOP, &stop_history))
    {
        fault_set(FAULT_REMOTE_STOP);
    }

    if (button_pressed(PIN_BTN_START, &start_history))
    {
        events = (S5_PARENT_FAULT == get_parent()) ? S5_EVT_RESET
                                                   : S5_EVT_REMOTE_START;
    }

    return (events);
}

/*
 * Act on one batch of events.
 */
static void handle_events(uint32_t events)
{
    s5_parent_t parent = get_parent();
    uint32_t    manual;

    if (0u != (events & S5_EVT_MANUAL))
    {
        if ((S5_PARENT_READY == parent) || (S5_PARENT_STOPPED == parent))
        {
            manual = manual_word;                       /* one load */
            APP_LOG((UB*)"[S5] manual move: type %d, %d\n",
                    (INT)(manual & 0xFFu),
                    (INT)(manual >> VC_MANUAL_ARG_SHIFT));
            send((msg_cmd_type_t)(manual & 0xFFu),
                 manual >> VC_MANUAL_ARG_SHIFT);
        }
        else
        {
            APP_LOG((UB*)"[S5] manual move ignored: park the car first\n");
        }
    }

    if (0u != (events & S5_EVT_RESET))
    {
        do_reset();
    }
    else if ((0u != (events & S5_EVT_REMOTE_START))
             && ((S5_PARENT_READY == parent)
                 || (S5_PARENT_STOPPED == parent)))
    {
        do_start();
    }
    else if (S5_PARENT_RUNNING == parent)
    {
        if (0u != (events & S5_EVT_OBSTACLE_SEEN))
        {
            /* Reachable from LINE_FOLLOW, BARCODE_ACTION and LINE_SEARCH:
               stop, so the profile is taken from a fixed position. */
            set_mission(S5_PARENT_RUNNING, S5_SUB_OBSTACLE_AVOID);
            bypass_leg = VC_LEG_NONE;
            send(MSG_CMD_STOP, 0u);
            s5_obstacle_request(S5_REQ_SCAN);
        }
        if (0u != (events & S5_EVT_OBSTACLE))
        {
            handle_profile();
        }
        if (0u != (events & S5_EVT_BARCODE))
        {
            handle_barcode();
        }
        if (0u != (events & S5_EVT_JUNCTION))
        {
            handle_junction();
        }
        if (0u != (events & S5_EVT_TEST_TURN))
        {
            handle_test_turn();
        }
        if (0u != (events & S5_EVT_MOVE_DONE))
        {
            handle_move_done();
        }
        /* LINE_LOST is acted on in poll_running(), level-triggered, so a
           loss during a barcode turn (expected: the car turns off the line)
           neither aborts the turn nor gets forgotten afterwards. */
    }
    else
    {
        /* Not running: other events are ignored. */
    }
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT ER s5_controller_init(void)
{
    T_CFLG cflg = {0};

    /* Event flag: many producers, ONE consumer (this task). */
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0u;
    h_flg_events = tk_cre_flg(&cflg);

    /* Buttons are active low.  Enable the pad pull-up and remove the
       RP2040's reset-default pull-down, so an open button reads high
       whether or not the board fits its own pull-ups. */
    (void)gpio_set_pin(PIN_BTN_START, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_BTN_STOP,  GPIO_MODE_IN);
    set_w(PADS_GPIO(PIN_BTN_START), PADS_PUE);
    clr_w(PADS_GPIO(PIN_BTN_START), PADS_PDE);
    set_w(PADS_GPIO(PIN_BTN_STOP), PADS_PUE);
    clr_w(PADS_GPIO(PIN_BTN_STOP), PADS_PDE);

    set_mission(S5_PARENT_INIT, S5_SUB_LINE_FOLLOW);

    return ((h_flg_events < E_OK) ? (ER)h_flg_events : E_OK);
}

EXPORT void controller_task(INT stacd, void *p_exinf)
{
    UINT events;

    (void)stacd;
    (void)p_exinf;

    /* ---- INIT: self-test, then READY or FAULT ---- */
    b_cal_ok = (E_OK == s3_calibrate());
    if (b_cal_ok)
    {
        enter_halted(S5_PARENT_READY);
        APP_LOG((UB*)"[S5] READY -- press START or publish a command\n");
    }
    else
    {
        enter_halted(S5_PARENT_FAULT);
        APP_LOG((UB*)"[S5] self-test failed: press START to retry\n");
    }

    /* One summary line ~3 s after power-up.  The IMU's own [S4] lines
       come in the first half second, usually before the serial monitor
       has reconnected, so they are easy to miss. */
    APP_LOG((UB*)"[S5] boot summary: line calibration %s, IMU %s\n",
            b_cal_ok ? "OK" : "FAILED (line not between the sensors?)",
            (S4_IMU_FAULT == s4_get_state()) ? "FAULT (see [S4] lines)"
            : ((S4_INIT_CAL == s4_get_state()) ? "still calibrating"
               : "calibrated"));

    for (;;)
    {
        /* Single consumer of flg_events, so clear-on-wait is correct here
           (unlike flg_fault, which has several readers). */
        events = 0u;
        if (tk_wai_flg(h_flg_events, S5_EVT_ALL, TWF_ORW | TWF_CLR,
                       &events, VC_WAIT_MS) < E_OK)
        {
            events = 0u;
        }

        handle_faults(fault_get());
        events |= (UINT)poll_buttons();
        handle_events((uint32_t)events);

        if (S5_PARENT_RUNNING == get_parent())
        {
            poll_running();
        }
    }
}

EXPORT void s5_manual_move(msg_cmd_type_t type, uint32_t arg)
{
    manual_word = ((uint32_t)msg_sat_u16(arg, (uint16_t)MSG_MAX_CMD_ARG,
                                         NULL) << VC_MANUAL_ARG_SHIFT)
                  | ((uint32_t)type & 0xFFu);
    s5_post_event(S5_EVT_MANUAL);
}

EXPORT void s5_inject_turn(uint8_t cmd)
{
    injected_turn = cmd;
    s5_post_event(S5_EVT_TEST_TURN);
}

EXPORT void s5_post_event(uint32_t event_bits)
{
    if (h_flg_events > 0)
    {
        (void)tk_set_flg(h_flg_events, (UINT)event_bits);
    }
}

EXPORT uint8_t s5_get_encoded_state(void)
{
    return (mission);
}

EXPORT bool s5_is_initialised(void)
{
    return (S5_PARENT_INIT != get_parent());
}

EXPORT bool s5_is_running(void)
{
    return (S5_PARENT_RUNNING == get_parent());
}

/* end of file */