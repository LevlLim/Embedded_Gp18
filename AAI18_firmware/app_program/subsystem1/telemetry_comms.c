/*
 * telemetry_comms.c
 *
 * Subsystem 1 -- telemetry task, snapshot mutex, MQTT outbox and remote
 * commands.
 *
 * DATA PATH
 * ---------
 *   producers (core 0) --single-word accessors--+
 *   Motion Control --s1_publish_motion, mutex---+--> telemetry_task (core 1)
 *        --> msg_pack_telemetry (outside the mutex, rule R6)
 *        --> outbox message buffer --> radio task --> mqtt_publish
 *
 * In a build without MQTT (the default bring-up build) frames are printed
 * to the console as hex instead, so no broker is needed on the bench.
 */

#include "subsystem1.h"
#include "../fault.h"
#include "../drivers/rp2040_regs.h"
#include "../drivers/gpio_irq.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem3/adc_sampler.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#if defined(TM_MQTT) && (TM_MQTT == 1)
#define S1_USE_MQTT         (1)
#else
#define S1_USE_MQTT         (0)
#endif

/* Outbox: topic byte + the largest payload (a log line; a telemetry
   frame is 12 bytes). */
#define S1_OUTBOX_MSG_LEN   (1u + APP_LOG_MAX)
#define S1_OUTBOX_DEPTH     (8u)
MSG_STATIC_ASSERT(APP_LOG_MAX >= MSG_LEN_TELEMETRY, outbox_fits_frame);

/* ---------------------------------------------------------------------
 * Private state
 * --------------------------------------------------------------------- */

static ID h_mtx_snapshot = (ID)0;
static ID h_mbf_outbox   = (ID)0;

/* Guarded by h_mtx_snapshot.  Written by Motion Control only. */
static s1_motion_fields_t motion_fields;

/* Written by the radio task only (link callbacks); read here. */
static volatile s1_conn_state_t conn_state =
    (1 == S1_USE_MQTT) ? S1_WIFI_INIT : S1_CONNECTED;

/* Written by whichever task last held the mutex.  Statistic only. */
static volatile uint32_t max_hold_us = 0u;

/* Accessed only by the task that delivers commands (radio task). */
static uint8_t last_cmd_seq   = 0u;
static bool    b_have_cmd_seq = false;

/* Telemetry task private. */
static uint8_t  frame_seq          = 0u;
static uint32_t last_encoder_total = 0u;

/* ---------------------------------------------------------------------
 * Private
 * --------------------------------------------------------------------- */

static void note_hold(uint32_t held_us)
{
    if (held_us > max_hold_us)
    {
        max_hold_us = held_us;
    }
}

/*
 * Copy Motion Control's fields out under the mutex and nothing else.
 * TMO_POL: skipping one frame is harmless, waiting is not.
 */
static bool take_motion_fields(s1_motion_fields_t *p_copy)
{
    uint32_t t_in;
    bool     b_ok = false;

    if (tk_loc_mtx(h_mtx_snapshot, TMO_POL) >= E_OK)
    {
        t_in    = RP2040_NOW_US();
        *p_copy = motion_fields;
        note_hold(RP2040_NOW_US() - t_in);
        (void)tk_unl_mtx(h_mtx_snapshot);
        b_ok = true;
    }

    return (b_ok);
}

/*
 * Fill a frame from the owners' single-word accessors.  Each read is one
 * aligned load, so none can tear even though the writer is on core 0.
 */
static void gather(msg_telemetry_t *p_frame)
{
    msg_line_t line;
    uint32_t   line_word;
    uint8_t    hump;

    p_frame->veh_state  = s5_get_encoded_state();
    p_frame->line_state = (uint8_t)s3_get_line_state();

    if (s3_read_line_word(&line_word))
    {
        msg_unpack_line(line_word, &line);
        p_frame->line_error   = line.error;
        p_frame->b_line_valid = line.b_valid;
    }
    else
    {
        p_frame->line_error   = 0;
        p_frame->b_line_valid = false;
    }

    hump                     = s4_get_hump_packed();
    p_frame->hump_mm         = msg_hump_mm(hump);
    p_frame->b_hump_sat      = msg_hump_saturated(hump);
    p_frame->b_hump_invalid  = s4_run_data_invalid();
    p_frame->tilt_deg        = s4_get_tilt_deg();
    p_frame->obst_distance_cm = s5_get_obstacle_distance_cm();
    p_frame->last_barcode    = s3_get_last_barcode();
    p_frame->fault_mask      = fault_get();
    p_frame->wifi_state      = (uint8_t)conn_state;
}

/*
 * Hand one payload to the network.  MQTT build: a non-blocking post to the
 * outbox (dropped if full -- telemetry is QoS 0 anyway).  Console build:
 * hex dump.
 */
static void publish(uint8_t topic_id, const uint8_t *p_payload, uint8_t len)
{
#if (1 == S1_USE_MQTT)
    uint8_t msg[S1_OUTBOX_MSG_LEN];
    uint8_t idx;

    if (len < S1_OUTBOX_MSG_LEN)
    {
        msg[0] = topic_id;
        for (idx = 0u; idx < len; idx++)
        {
            msg[idx + 1u] = p_payload[idx];
        }
        (void)tk_snd_mbf(h_mbf_outbox, msg, (INT)len + 1, TMO_POL);
    }
#else
    uint8_t idx;

    tm_printf((UB*)"[TELE] t%d (%d B):", (INT)topic_id, (INT)len);
    for (idx = 0u; idx < len; idx++)
    {
        tm_printf((UB*)" %02x", (INT)p_payload[idx]);
    }
    tm_printf((UB*)"\n");
#endif
}

/* RP2040 reset-cause registers (datasheet 2.10.6 CHIP_RESET, 4.7.6
   WATCHDOG REASON).  They keep their value until the next reset. */
#define S1_CHIP_RESET_REG       (0x40064008u)
#define S1_HAD_POR              (1u << 8)   /* power-on OR brown-out    */
#define S1_HAD_RUN              (1u << 16)  /* RUN pin (reset button)   */
#define S1_WDOG_REASON_REG      (0x40058008u)
#define S1_WDOG_ANY             (0x03u)     /* timer or forced          */

/* Why the Pico last started, for the console. */
static const char *reset_reason(void)
{
    uint32_t chip = in_w(S1_CHIP_RESET_REG);
    const char *p_why = "unknown";

    if (0u != (in_w(S1_WDOG_REASON_REG) & S1_WDOG_ANY))
    {
        p_why = "watchdog / reboot after flashing";
    }
    else if (0u != (chip & S1_HAD_RUN))
    {
        p_why = "reset button (RUN pin)";
    }
    else if (0u != (chip & S1_HAD_POR))
    {
        p_why = "power on OR battery brown-out";
    }
    else
    {
        /* None of the above. */
    }

    return (p_why);
}

/*
 * One line every time the MQTT link comes up.  A car that "just stopped"
 * and then shows "up 9 s" here has restarted: the boot lines were printed
 * before Wi-Fi was back, so they never reached the laptop.
 */
static void report_link_up(void)
{
    static s1_conn_state_t last_state = S1_RECONNECTING;

    if ((S1_CONNECTED == conn_state) && (S1_CONNECTED != last_state))
    {
        APP_LOG((UB*)"[S1] MQTT connected: car up %d s, last reset: %s\n",
                (INT)(app_now_ms() / 1000u), reset_reason());
    }
    last_state = conn_state;
}

/*
 * WIFI_LOSS is the one fault bit Telemetry owns and may clear (Fault
 * Table).  It is REPORT-only, so it never affects driving.
 */
static void update_wifi_fault(void)
{
    if (S1_CONNECTED == conn_state)
    {
        if (0u != (fault_get() & FAULT_WIFI_LOSS))
        {
            fault_clear(FAULT_WIFI_LOSS);
        }
    }
    else
    {
        fault_set(FAULT_WIFI_LOSS);
    }
}

/*
 * One human-readable line: both line sensors (B = above its threshold,
 * i.e. dark, W = light), the thresholds, the steering error the car is
 * using, the duty each wheel is actually getting, and the IMU pitch
 * (+ = nose up) and roll (side to side) in degrees, the encoder totals
 * (left/right, for measuring counts per wheel turn by hand), the latest
 * ultrasonic distance ahead (0 = no echo / nothing in range), flagged if
 * the IMU is in fault.
 */
/* Short mission name for the debug line.  With the fault mask beside it,
   every [DBG] line says WHY the car is stopped, even when the one-off
   "[S5] ..." line about it was lost on a Wi-Fi hiccup. */
static const char *mission_name(void)
{
    static const char *const names[5] = { "INIT", "READY", "RUN",
                                          "STOP", "FAULT" };
    uint8_t parent = 0u;

    msg_decode_veh_state(s5_get_encoded_state(), &parent, NULL);

    return ((parent < 5u) ? names[parent] : "?");
}

static void print_debug_line(void)
{
    adc_sample_t s_left   = {0};
    adc_sample_t s_right  = {0};
    adc_sample_t s_bc     = {0};
    uint16_t     thr_bc;
    msg_line_t   line     = {0};
    uint32_t     word     = 0u;
    uint16_t     thr_l    = 0u;
    uint16_t     thr_r    = 0u;
    int32_t      duty_l   = 0;
    int32_t      duty_r   = 0;
    gpio_irq_encoders_t enc = {0};

    gpio_irq_read_encoders(&enc);
    (void)adc_sampler_read(S3_CH_LEFT, &s_left);
    (void)adc_sampler_read(S3_CH_RIGHT, &s_right);
    (void)adc_sampler_read(ADC_CH_BARCODE, &s_bc);
    thr_bc = s3_get_barcode_threshold();
    s3_get_thresholds(&thr_l, &thr_r);
    motor_get_applied(&duty_l, &duty_r);
    if (s3_read_line_word(&word))
    {
        msg_unpack_line(word, &line);
    }

    /* Single spaces: with the barcode sensor added the longest possible
       line is exactly APP_LOG_MAX (160) characters. */
    APP_LOG((UB*)"[DBG] L %4d/%4d %s R %4d/%4d %s bc %4d/%4d %s"
                   " err %3d %s jct %d duty L %3d R %3d pitch %3d roll %3d"
                   " enc %d/%d us %dcm sv %d ramp %dmm %s f%03x\n",
              (INT)s_left.value, (INT)thr_l,
              ((uint32_t)s_left.value > thr_l) ? "B" : "W",
              (INT)s_right.value, (INT)thr_r,
              ((uint32_t)s_right.value > thr_r) ? "B" : "W",
              (INT)s_bc.value, (INT)thr_bc,
              ((uint32_t)s_bc.value > thr_bc) ? "B" : "W",
              (INT)line.error,
              line.b_valid ? "ok" : "--",
              (INT)s3_get_junction_count(),
              (INT)duty_l, (INT)duty_r,
              (INT)s4_get_tilt_deg(), (INT)s4_get_roll_deg(),
              (INT)enc.count_left, (INT)enc.count_right,
              (INT)s5_get_obstacle_distance_cm(),
              (INT)s5_get_servo_angle(),
              (INT)s4_get_side_live_mm(),
              mission_name(),
              (INT)fault_get());
}

/*
 * Build and publish one state frame.
 */
static void send_state_frame(void)
{
    s1_motion_fields_t motion;
    msg_telemetry_t    frame = {0};
    uint8_t            buf[MSG_LEN_TELEMETRY];
    uint32_t           delta;
    uint8_t            len;

    gather(&frame);

    if (take_motion_fields(&motion))
    {
        /* Single writer (Motion) of the total, single reader here, so
           the difference is race-free.  Saturate, never wrap (R5). */
        delta              = motion.encoder_total - last_encoder_total;
        last_encoder_total = motion.encoder_total;
        frame.encoder_delta = msg_sat_u16(delta,
                                          (uint16_t)MSG_MAX_ENCODER_DELTA,
                                          NULL);
        frame.speed_pct     = motion.speed_pct;
    }

    frame.seq = frame_seq;
    frame_seq = (uint8_t)(frame_seq + 1u);

    len = msg_pack_telemetry(&frame, buf, (uint8_t)sizeof(buf));
    /* Readable line: console, and car18/log over Wi-Fi.  Binary frame:
       always over Wi-Fi; on the console only if the readable line is
       off (two lines every 200 ms would bury everything else). */
    if ((1 == S1_DEBUG_PRINT) && (6 != APP_TEST_BUDDY))
    {
        print_debug_line();     /* not in the ultrasonic bench test */
    }
    if ((0u != len) && ((1 == S1_USE_MQTT)
                        || ((0 == S1_DEBUG_PRINT) && (6 != APP_TEST_BUDDY))))
    {
        publish(S1_TOPIC_ID_STATE, buf, len);
    }
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT ER s1_comms_init(void)
{
    T_CMTX cmtx = {0};
    T_CMBF cmbf = {0};
    ER     ercd = E_OK;

    /*
     * TA_INHERIT bounds Motion Control's blocking behind this Low-priority
     * task to the length of the critical section, which is a struct copy.
     */
    cmtx.mtxatr  = TA_TFIFO | TA_INHERIT;
    cmtx.ceilpri = PRI_MOTION_CONTROL;
    h_mtx_snapshot = tk_cre_mtx(&cmtx);
    if (h_mtx_snapshot < E_OK)
    {
        ercd = (ER)h_mtx_snapshot;
    }

    if ((E_OK == ercd) && (1 == S1_USE_MQTT))
    {
        cmbf.mbfatr = TA_TFIFO;
        cmbf.bufsz  = (INT)APP_MBF_BUFSZ(S1_OUTBOX_DEPTH, S1_OUTBOX_MSG_LEN);
        cmbf.maxmsz = (INT)S1_OUTBOX_MSG_LEN;
        h_mbf_outbox = tk_cre_mbf(&cmbf);
        if (h_mbf_outbox < E_OK)
        {
            ercd = (ER)h_mbf_outbox;
        }
    }

    return (ercd);
}

EXPORT void s1_publish_motion(uint8_t speed_pct, uint32_t encoder_total)
{
    uint32_t t_in;

    if (tk_loc_mtx(h_mtx_snapshot, TMO_POL) >= E_OK)
    {
        t_in                        = RP2040_NOW_US();
        motion_fields.speed_pct     = speed_pct;
        motion_fields.encoder_total = encoder_total;
        note_hold(RP2040_NOW_US() - t_in);
        (void)tk_unl_mtx(h_mtx_snapshot);
    }
}

EXPORT bool s1_accept_command(uint8_t seq, uint8_t command)
{
    uint8_t delta;
    bool    b_known;
    bool    b_fresh;

    b_known = ((S1_CMD_START == command) || (S1_CMD_STOP == command)
               || (S1_CMD_RESET == command));

    /* Newer if 1..127 ahead, modulo 256.  The first command after boot is
       accepted because there is nothing to compare it with; commands are
       published non-retained (section 6.1), so a stale one cannot be
       waiting on the broker at boot. */
    delta   = (uint8_t)(seq - last_cmd_seq);
    b_fresh = (!b_have_cmd_seq) || ((delta > 0u) && (delta < 128u));

    if (b_known && b_fresh)
    {
        last_cmd_seq   = seq;
        b_have_cmd_seq = true;

        if (S1_CMD_START == command)
        {
            s5_post_event(S5_EVT_REMOTE_START);
        }
        else if (S1_CMD_STOP == command)
        {
            /* STOP sets the fault bit DIRECTLY as well as notifying the
               Controller, so Motion Control brakes without waiting for the
               Controller to be scheduled (FR10). */
            fault_set(FAULT_REMOTE_STOP);
            s5_post_event(S5_EVT_REMOTE_STOP);
        }
        else
        {
            s5_post_event(S5_EVT_RESET);
        }
    }
    else
    {
        APP_LOG((UB*)"[S1] rejected command %d seq=%d\n",
                  (INT)command, (INT)seq);
    }

    return (b_known && b_fresh);
}

EXPORT void telemetry_task(INT stacd, void *p_exinf)
{
    uint32_t last_heartbeat_ms = 0u;
    uint16_t last_fault_mask   = 0xFFFFu;   /* forces a first publish */
    uint16_t faults;
    uint8_t  fault_buf[2];
    uint8_t  beat;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        (void)app_wait_period();

        update_wifi_fault();
        report_link_up();
        send_state_frame();

        /* ---- fault topic, published on CHANGE ---- */
        faults = fault_get();
        if (faults != last_fault_mask)
        {
            fault_buf[0] = (uint8_t)((uint32_t)faults & 0xFFu);
            fault_buf[1] = (uint8_t)(((uint32_t)faults >> 8u) & 0xFFu);
            publish(S1_TOPIC_ID_FAULT, fault_buf, 2u);
            last_fault_mask = faults;
        }

        /* ---- 1 s heartbeat (NFR6) ---- */
        if ((app_now_ms() - last_heartbeat_ms) >= S1_HEARTBEAT_MS)
        {
            beat = frame_seq;
            if ((0 == S1_DEBUG_PRINT) || (1 == S1_USE_MQTT))
            {
                publish(S1_TOPIC_ID_HEARTBEAT, &beat, 1u);
            }
            last_heartbeat_ms = app_now_ms();
        }
    }
}

EXPORT s1_conn_state_t s1_get_conn_state(void)
{
    return (conn_state);
}

EXPORT uint32_t s1_max_hold_us(void)
{
    return (max_hold_us);
}

EXPORT void s1_reset_stats(void)
{
    max_hold_us = 0u;
}

/*
 * One log line (see APP_LOG in app.h): to the console, and in an MQTT
 * build also to car18/log, without the trailing newline.  Never blocks:
 * if the outbox is full the Wi-Fi copy is dropped.
 */
EXPORT void app_log_line(const UB *p_text)
{
#if (1 == S1_USE_MQTT)
    uint8_t len = 0u;
#endif

    if (NULL == p_text)
    {
        return;
    }

    (void)tm_putstring(p_text);

#if (1 == S1_USE_MQTT)
    while ((len < APP_LOG_MAX) && ((UB)0 != p_text[len])
           && ((UB)'\n' != p_text[len]))
    {
        len = (uint8_t)(len + 1u);
    }
    if ((0u != len) && (h_mbf_outbox > 0))
    {
        publish(S1_TOPIC_ID_LOG, (const uint8_t *)p_text, len);
    }
#endif
}

/* ---------------------------------------------------------------------
 * MQTT hooks -- run on the template's radio task (core 0).
 * --------------------------------------------------------------------- */
#if (1 == S1_USE_MQTT)

EXPORT int app_mqtt_next_publish(const char **pp_topic, unsigned char *p_buf,
                                 int buf_len)
{
    uint8_t msg[S1_OUTBOX_MSG_LEN];
    INT     got;
    INT     idx;
    int     len = 0;

    got = tk_rcv_mbf(h_mbf_outbox, msg, TMO_POL);

    if ((got > 1) && ((got - 1) <= buf_len) && (NULL != pp_topic))
    {
        switch (msg[0])
        {
            case S1_TOPIC_ID_STATE:
                *pp_topic = S1_TOPIC_STATE;
            break;

            case S1_TOPIC_ID_HEARTBEAT:
                *pp_topic = S1_TOPIC_HEARTBEAT;
            break;

            case S1_TOPIC_ID_LOG:
                *pp_topic = S1_TOPIC_LOG;
            break;

            default:
                *pp_topic = S1_TOPIC_FAULT;
            break;
        }

        for (idx = 1; idx < got; idx++)
        {
            p_buf[idx - 1] = msg[idx];
        }
        len = (int)(got - 1);
    }

    return (len);
}

/* True if the payload is exactly the given lower-case word. */
static bool payload_is(const unsigned char *p_data, int len,
                       const char *p_word)
{
    int  idx    = 0;
    bool b_same = true;

    while (b_same && ('\0' != p_word[idx]))
    {
        b_same = (idx < len) && ((char)p_data[idx] == p_word[idx]);
        idx++;
    }

    return (b_same && (idx == len));
}

static void text_command(const unsigned char *p_data, int len)
{
    uint8_t seq = (uint8_t)(last_cmd_seq + 1u);
    uint8_t command = 0u;

    if (payload_is(p_data, len, "start"))
    {
        command = S1_CMD_START;
    }
    else if (payload_is(p_data, len, "stop"))
    {
        command = S1_CMD_STOP;
    }
    else if (payload_is(p_data, len, "reset"))
    {
        command = S1_CMD_RESET;
    }
    else if (payload_is(p_data, len, "left"))
    {
        s5_manual_move(MSG_CMD_TURN_L, S5_MANUAL_TURN_DEG);
        return;
    }
    else if (payload_is(p_data, len, "right"))
    {
        s5_manual_move(MSG_CMD_TURN_R, S5_MANUAL_TURN_DEG);
        return;
    }
    else if (payload_is(p_data, len, "fwd"))
    {
        s5_manual_move(MSG_CMD_FORWARD, S5_MANUAL_FWD_MM);
        return;
    }
    else if (payload_is(p_data, len, "back"))
    {
        s5_manual_move(MSG_CMD_REVERSE, S5_MANUAL_BACK_MM);
        return;
    }
    else if (payload_is(p_data, len, "jleft"))
    {
        s5_inject_turn((uint8_t)MSG_BC_LEFT);
        return;
    }
    else if (payload_is(p_data, len, "jright"))
    {
        s5_inject_turn((uint8_t)MSG_BC_RIGHT);
        return;
    }
    else if (payload_is(p_data, len, "jstraight"))
    {
        s5_inject_turn((uint8_t)MSG_BC_STRAIGHT);
        return;
    }
    else if (payload_is(p_data, len, "juturn"))
    {
        s5_inject_turn((uint8_t)MSG_BC_U_TURN);
        return;
    }
    else
    {
        /* Unknown word: s1_accept_command() logs and rejects command 0. */
    }

    APP_LOG((UB*)"[S1] remote command: %s\n",
            (S1_CMD_START == command) ? "START"
            : ((S1_CMD_STOP == command) ? "STOP"
               : ((S1_CMD_RESET == command) ? "RESET" : "unknown")));
    (void)s1_accept_command(seq, command);
}

EXPORT void app_mqtt_on_message(const char *p_topic,
                                const unsigned char *p_data, int len)
{
    const char *p_expect = S1_TOPIC_CMD;
    uint32_t    idx      = 0u;
    bool        b_match  = (NULL != p_topic) && (NULL != p_data);

    /* Exact topic match without the C library. */
    while (b_match && ('\0' != p_expect[idx]))
    {
        b_match = (p_topic[idx] == p_expect[idx]);
        idx++;
    }
    b_match = b_match && ('\0' == p_topic[idx]);

    /* Binary command payload (CR1 item 8): [seq][command], 2 bytes. */
    if (b_match && ((int)MSG_LEN_COMMAND == len))
    {
        (void)s1_accept_command(p_data[0], p_data[1]);
    }
    else if (b_match)
    {
        /* Text command for a person at a laptop (mosquitto_pub -m start):
           "start", "stop" or "reset".  It takes the next sequence number
           itself, so it is always fresh. */
        text_command(p_data, len);
    }
    else
    {
        /* Another topic: not for us. */
    }
}

EXPORT void app_mqtt_link_state(int connected)
{
    conn_state = (0 != connected) ? S1_CONNECTED : S1_RECONNECTING;
}

#endif /* S1_USE_MQTT */

/* end of file */
