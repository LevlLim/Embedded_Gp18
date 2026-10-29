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
 * OBSTACLE BYPASS (FR8), driven as encoder legs, side S = chosen side:
 *   0 turn toward S 90     1 forward: clear the obstacle sideways
 *   2 turn back 90         3 forward: pass the obstacle
 *   4 turn back 90         5 forward toward the line, watching for it
 *   line found during 4/5 -> 6 turn toward S 90 (realign) -> line follow
 * The forward D_stop guard is on only during the forward legs.
 */

#include "subsystem5.h"
#include "../fault.h"
#include "../drivers/rp2040_regs.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define VC_WAIT_MS              (20)    /* event wait = button poll rate  */
#define VC_BUTTON_DOWN_MASK     (0x07u) /* 3 samples (>= 40 ms) pressed   */
#define VC_LEG_ALIGN            (6u)
#define VC_LEG_NONE             (0xFFu)
#define VC_SEARCH_STEP_DEG      (20u)
#define VC_BARCODE_FORWARD_MM   (200u)
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
static bool     b_sensors_ok        = false;
static uint8_t  start_history   = 0xFFu;   /* 1 = released (active low) */
static uint8_t  stop_history    = 0xFFu;

static msg_obstacle_t obstacle;
static uint8_t  bypass_leg      = VC_LEG_NONE;
static uint8_t  avoid_retries   = 0u;
static uint16_t lateral_mm      = 0u;

static uint32_t search_start_ms = 0u;
static uint32_t follow_since_ms = 0u;
static uint32_t lost_since_ms   = 0u;
static bool     b_was_lost      = false;
static uint8_t  search_step     = 0u;
static bool     b_search_right  = false;   /* first turn toward the line */

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
    bypass_leg = VC_LEG_NONE;
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
    enter_line_follow();
    APP_LOG((UB*)"[S5] RUNNING\n");
}

static void enter_halted(s5_parent_t parent)
{
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

    send((b_left == b_out) ? MSG_CMD_TURN_L : MSG_CMD_TURN_R, 90u);
}

/* Issue bypass leg number 'leg'. */
static void run_leg(uint8_t leg)
{
    uint32_t pass_cm;

    bypass_leg = leg;
    s5_obstacle_guard((1u == leg) || (3u == leg) || (5u == leg));

    switch (leg)
    {
        case 0u:
            send_turn(true);
        break;

        case 1u:
            send(MSG_CMD_FORWARD, lateral_mm);
        break;

        case 3u:
            pass_cm = (uint32_t)obstacle.distance_cm + obstacle.width_cm
                    + S5_ROBOT_LENGTH_CM + S5_CLEARANCE_MARGIN_CM;
            send(MSG_CMD_FORWARD, pass_cm * 10u);
        break;

        case 5u:
            send(MSG_CMD_FORWARD, (uint32_t)lateral_mm + 100u);
        break;

        case VC_LEG_ALIGN:
            send_turn(true);
        break;

        default:
            /* Legs 2 and 4: turn back. */
            send_turn(false);
        break;
    }
}

/*
 * A profile arrived: plan the bypass, back off and rescan, or give up.
 */
static void handle_profile(void)
{
    int32_t edge_cm;

    if (!s5_take_obstacle(&obstacle))
    {
        return;
    }

    if (0u == obstacle.distance_cm)
    {
        /* False alarm: nothing confirmed by the fine scan. */
        enter_line_follow();
    }
    else if (MSG_SIDE_NONE != obstacle.side)
    {
        /* Lateral distance from the car's centre line to the obstacle edge
           on the chosen side, plus half the car and the margin. */
        edge_cm = s5_lateral_cm(obstacle.distance_cm,
                                (int32_t)obstacle.bearing_deg);
        edge_cm = (MSG_SIDE_LEFT == obstacle.side)
                ? (((int32_t)obstacle.width_cm / 2) - edge_cm)
                : (((int32_t)obstacle.width_cm / 2) + edge_cm);
        edge_cm = (edge_cm < 0) ? 0 : edge_cm;
        lateral_mm = msg_sat_u16((uint32_t)(edge_cm
                                 + (int32_t)(S5_ROBOT_WIDTH_CM / 2u)
                                 + (int32_t)S5_CLEARANCE_MARGIN_CM) * 10u,
                                 (uint16_t)MSG_MAX_CMD_ARG, NULL);
        run_leg(0u);
    }
    else if (avoid_retries < S5_MAX_RETRIES)
    {
        /* Neither side fits: back off and look again (REVERSE_RETRY). */
        avoid_retries = (uint8_t)(avoid_retries + 1u);
        bypass_leg    = VC_LEG_NONE;
        send(MSG_CMD_REVERSE, S5_REVERSE_MM);
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

/*
 * A manoeuvre finished.  What comes next depends on the substate.
 */
static void handle_move_done(void)
{
    s5_sub_t sub = get_sub();

    if (S5_SUB_BARCODE_ACTION == sub)
    {
        enter_line_follow();
    }
    else if (S5_SUB_OBSTACLE_AVOID == sub)
    {
        if (VC_LEG_NONE == bypass_leg)
        {
            /* The reverse of a retry finished: profile again. */
            s5_obstacle_request(S5_REQ_SCAN);
        }
        else if (VC_LEG_ALIGN == bypass_leg)
        {
            enter_line_follow();        /* line reacquired (FR8) */
        }
        else if (bypass_leg < 5u)
        {
            run_leg((uint8_t)(bypass_leg + 1u));
        }
        else
        {
            enter_search();             /* leg 5 ended without the line */
        }
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

/*
 * Act on a decoded barcode, in line following only.  A low-confidence
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

    if (b_got && (S5_SUB_LINE_FOLLOW == get_sub())
        && (bc.confidence >= VC_MIN_CONFIDENCE))
    {
        set_mission(S5_PARENT_RUNNING, S5_SUB_BARCODE_ACTION);

        if (MSG_BC_STRAIGHT == bc.cmd)
        {
            send(MSG_CMD_FORWARD, VC_BARCODE_FORWARD_MM);
        }
        else if (MSG_BC_LEFT == bc.cmd)
        {
            send(MSG_CMD_TURN_L, 90u);
        }
        else if (MSG_BC_RIGHT == bc.cmd)
        {
            send(MSG_CMD_TURN_R, 90u);
        }
        else if (MSG_BC_U_TURN == bc.cmd)
        {
            send(MSG_CMD_U_TURN, 180u);
        }
        else
        {
            set_mission(S5_PARENT_RUNNING, S5_SUB_LINE_FOLLOW);
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
    else if ((S5_SUB_OBSTACLE_AVOID == sub) && b_on_line
             && ((4u == bypass_leg) || (5u == bypass_leg)))
    {
        run_leg(VC_LEG_ALIGN);          /* back on the line: realign */
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

    /* Reset always restores the fixed profile, including after an optional
       maintenance sweep. There is no calibration delay to retry. */
    b_sensors_ok = (E_OK == s3_prepare());

    enter_halted(b_sensors_ok ? S5_PARENT_READY : S5_PARENT_FAULT);
    APP_LOG((UB*)"[S5] RESET -> %s\n", b_sensors_ok ? "READY" : "FAULT");
}

static void do_start(void)
{
    uint16_t hard = (uint16_t)((uint32_t)fault_get() & FAULT_MASK_ESTOP
                               & ~(uint32_t)FAULT_REMOTE_STOP);

    /* START clears only the operator's own stop.  A latched COLLISION,
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

    /* ---- INIT: load fixed sensor settings, then READY or FAULT ---- */
    b_sensors_ok = (E_OK == s3_prepare());
    if (b_sensors_ok)
    {
        enter_halted(S5_PARENT_READY);
        APP_LOG((UB*)"[S5] READY -- press START or publish a command\n");
    }
    else
    {
        enter_halted(S5_PARENT_FAULT);
        APP_LOG((UB*)"[S5] sensor setup failed: press START to retry\n");
    }

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
