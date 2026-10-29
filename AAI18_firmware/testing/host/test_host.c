/*
 * test_host.c
 *
 * Group AAI18 -- host-side unit tests for the pure logic.
 *
 * WHY THIS EXISTS
 * ---------------
 * Change Requirement 1 asks for evidence that "the revised representation
 * causes no loss of required functionality".  The encode and decode
 * functions are pure, so they can be exercised exhaustively on a PC, far
 * faster and more thoroughly than on the target.  This is also test T2
 * (min / max / out-of-range for every packed field) and T3 (obstacle
 * profile round trip), and it covers the pure helpers of subsystems 3, 4
 * and 5 (barcode decode, pitch, hump height, obstacle profile).
 *
 * BUILD AND RUN
 *     cd testing/host && make            # every buddy's host tests
 *     make test3                         # Buddy 3's tests only (1..5)
 *     make target-sizes                  # sizeof() on the ARM compiler
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "msg_defs.h"
#include "subsystem2/motor_control.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/subsystem5.h"

static int tests_run    = 0;
static int tests_passed = 0;

#define CHECK_TRUE(cond, label)                                         \
    do {                                                                \
        tests_run++;                                                    \
        if (cond) {                                                     \
            tests_passed++;                                             \
        } else {                                                        \
            printf("  FAIL: %s  (%s:%d)\n", (label), __FILE__, __LINE__); \
        }                                                               \
    } while (0)

#define CHECK_EQ_INT(got, want, label)                                  \
    do {                                                                \
        tests_run++;                                                    \
        if ((long)(got) == (long)(want)) {                              \
            tests_passed++;                                             \
        } else {                                                        \
            printf("  FAIL: %s  got %ld want %ld  (%s:%d)\n",           \
                   (label), (long)(got), (long)(want),                  \
                   __FILE__, __LINE__);                                 \
        }                                                               \
    } while (0)

/* ---------------------------------------------------------------------
 * Motion command: every defined type, argument and sequence (7 x 1024 x
 * 8 combinations), and refusal of the one undefined code.
 * --------------------------------------------------------------------- */
static void test_motion_cmd(void)
{
    msg_motion_cmd_t in;
    msg_motion_cmd_t out;
    uint8_t          buf[MSG_LEN_MOTION_CMD];
    int              failures = 0;
    unsigned         type;
    unsigned         arg;
    unsigned         seq;

    printf("motion command: exhaustive round trip\n");

    for (type = 0u; type <= (unsigned)MSG_CMD_LAST; type++)
    {
        for (arg = 0u; arg <= MSG_MAX_CMD_ARG; arg++)
        {
            for (seq = 0u; seq <= 7u; seq++)
            {
                in.type = (msg_cmd_type_t)type;
                in.arg  = (uint16_t)arg;
                in.seq  = (uint8_t)seq;

                memset(buf, 0, sizeof(buf));
                if ((MSG_LEN_MOTION_CMD != msg_pack_motion_cmd(&in, buf,
                                                               sizeof(buf)))
                    || !msg_unpack_motion_cmd(buf, sizeof(buf), &out)
                    || (out.type != in.type) || (out.arg != in.arg)
                    || (out.seq != in.seq))
                {
                    failures++;
                }
            }
        }
    }

    CHECK_EQ_INT(failures, 0, "motion cmd exhaustive round trip");
    printf("  %u combinations checked, %d bytes per message\n",
           ((unsigned)MSG_CMD_LAST + 1u) * (MSG_MAX_CMD_ARG + 1u) * 8u,
           MSG_LEN_MOTION_CMD);

    /* T2: an undefined code must never become a movement command. */
    buf[0] = 0x07u;                 /* type 7, arg 0, seq 0 */
    buf[1] = 0x00u;
    CHECK_TRUE(!msg_unpack_motion_cmd(buf, sizeof(buf), &out),
               "undefined command type 7 rejected on unpack");

    in.type = (msg_cmd_type_t)7;
    in.arg  = 10u;
    in.seq  = 0u;
    CHECK_EQ_INT(msg_pack_motion_cmd(&in, buf, sizeof(buf)), 0,
                 "undefined command type 7 refused on pack");
}

/* ---------------------------------------------------------------------
 * Saturation: out-of-range must CLAMP, never wrap (rule R5).
 * --------------------------------------------------------------------- */
static void test_saturation(void)
{
    msg_motion_cmd_t in;
    msg_motion_cmd_t out;
    uint8_t          buf[MSG_LEN_MOTION_CMD];
    bool             b_sat;

    printf("saturation: out-of-range must clamp, never wrap\n");

    in.type = MSG_CMD_FORWARD;
    in.arg  = 2000u;
    in.seq  = 1u;
    (void)msg_pack_motion_cmd(&in, buf, sizeof(buf));
    (void)msg_unpack_motion_cmd(buf, sizeof(buf), &out);
    CHECK_EQ_INT(out.arg, MSG_MAX_CMD_ARG, "arg 2000 clamps to 1023");

    b_sat = false;
    CHECK_EQ_INT(msg_sat_u16(300u, 255u, &b_sat), 255, "sat_u16 clamps");
    CHECK_TRUE(b_sat, "sat_u16 reports saturation");

    b_sat = false;
    CHECK_EQ_INT(msg_sat_u16(100u, 255u, &b_sat), 100, "sat_u16 passes");
    CHECK_TRUE(!b_sat, "sat_u16 reports no saturation");

    b_sat = false;
    CHECK_EQ_INT(msg_sat_i8(-200, 63, &b_sat), -63, "sat_i8 clamps negative");
    CHECK_TRUE(b_sat, "sat_i8 reports saturation");

    CHECK_TRUE(msg_saturation_seen(), "sticky saturation flag raised");
}

/* ---------------------------------------------------------------------
 * Telemetry: every field saturates.  These four inputs WRAPPED in schema
 * v1 (tilt 80 decoded as -48, speed 200 as 72, line error 100 as -28).
 * --------------------------------------------------------------------- */
static void test_telemetry_saturation(void)
{
    msg_telemetry_t in;
    msg_telemetry_t out;
    uint8_t         buf[MSG_LEN_TELEMETRY];

    printf("telemetry: saturation on every field\n");

    memset(&in, 0, sizeof(in));
    in.tilt_deg   = 80;
    in.speed_pct  = 200u;
    in.line_error = 100;
    in.hump_mm    = 200u;

    (void)msg_pack_telemetry(&in, buf, sizeof(buf));
    CHECK_TRUE(msg_unpack_telemetry(buf, sizeof(buf), &out), "unpack ok");
    CHECK_EQ_INT(out.tilt_deg, 63, "tilt 80 clamps to 63 (was -48)");
    CHECK_EQ_INT(out.speed_pct, 100, "speed 200 clamps to 100 (was 72)");
    CHECK_EQ_INT(out.line_error, 63, "line error 100 clamps to 63 (was -28)");
    CHECK_EQ_INT(out.hump_mm, 127, "hump 200 clamps to 127 (was 72)");

    in.tilt_deg   = -90;
    in.line_error = -100;
    (void)msg_pack_telemetry(&in, buf, sizeof(buf));
    (void)msg_unpack_telemetry(buf, sizeof(buf), &out);
    CHECK_EQ_INT(out.tilt_deg, -63, "tilt -90 clamps to -63");
    CHECK_EQ_INT(out.line_error, -63, "line error -100 clamps to -63");
}

/* ---------------------------------------------------------------------
 * Telemetry frame: round trip, schema guard, enum guard.
 * --------------------------------------------------------------------- */
static void test_telemetry(void)
{
    msg_telemetry_t in;
    msg_telemetry_t out;
    uint8_t         buf[MSG_LEN_TELEMETRY];

    printf("telemetry frame: round trip and guards\n");

    memset(&in, 0, sizeof(in));
    in.seq              = 42u;
    in.veh_state        = msg_encode_veh_state(2u, 3u);
    in.line_state       = (uint8_t)MSG_LINE_TRACK;
    in.line_error       = -45;
    in.b_line_valid     = true;
    in.speed_pct        = 55u;
    in.hump_mm          = 120u;
    in.b_hump_sat       = true;
    in.b_hump_invalid   = true;
    in.tilt_deg         = -12;
    in.obst_distance_cm = 200u;
    in.fault_mask       = 0x0285u;
    in.wifi_state       = 3u;
    in.last_barcode     = (uint8_t)MSG_BC_RIGHT;
    in.encoder_delta    = 40000u;

    CHECK_EQ_INT(msg_pack_telemetry(&in, buf, sizeof(buf)),
                 MSG_LEN_TELEMETRY, "pack length");
    CHECK_TRUE(msg_unpack_telemetry(buf, sizeof(buf), &out), "unpack ok");

    CHECK_EQ_INT(out.seq,              in.seq,              "seq");
    CHECK_EQ_INT(out.veh_state,        in.veh_state,        "veh_state");
    CHECK_EQ_INT(out.line_state,       in.line_state,       "line_state");
    CHECK_EQ_INT(out.line_error,       in.line_error,       "line_error");
    CHECK_TRUE(out.b_line_valid    ==  in.b_line_valid,     "line_valid");
    CHECK_EQ_INT(out.speed_pct,        in.speed_pct,        "speed");
    CHECK_EQ_INT(out.hump_mm,          in.hump_mm,          "hump_mm");
    CHECK_TRUE(out.b_hump_sat      ==  in.b_hump_sat,       "hump_sat");
    CHECK_TRUE(out.b_hump_invalid  ==  in.b_hump_invalid,   "hump_invalid");
    CHECK_EQ_INT(out.tilt_deg,         in.tilt_deg,         "tilt");
    CHECK_EQ_INT(out.obst_distance_cm, in.obst_distance_cm, "obst distance");
    CHECK_EQ_INT(out.fault_mask,       in.fault_mask,       "fault mask");
    CHECK_EQ_INT(out.wifi_state,       in.wifi_state,       "wifi state");
    CHECK_EQ_INT(out.last_barcode,     in.last_barcode,     "last barcode");
    CHECK_EQ_INT(out.encoder_delta,    in.encoder_delta,    "encoder delta");

    buf[0] = (uint8_t)(MSG_SCHEMA_VERSION + 1u);
    CHECK_TRUE(!msg_unpack_telemetry(buf, sizeof(buf), &out),
               "wrong schema version rejected");

    (void)msg_pack_telemetry(&in, buf, sizeof(buf));
    buf[2] = (uint8_t)(buf[2] | 0xE0u);         /* line_state = 7 */
    CHECK_TRUE(!msg_unpack_telemetry(buf, sizeof(buf), &out),
               "undefined line_state rejected");
}

/* ---------------------------------------------------------------------
 * Obstacle profile (T3): signed bearing, boundaries, enum guard.
 * --------------------------------------------------------------------- */
static void test_obstacle(void)
{
    msg_obstacle_t in;
    msg_obstacle_t out;
    uint8_t        buf[MSG_LEN_OBSTACLE];
    int            bearing;
    int            failures = 0;

    printf("obstacle profile: bearing range, boundaries, guards\n");

    memset(&in, 0, sizeof(in));
    in.distance_cm    = 100u;
    in.width_cm       = 20u;
    in.clear_left_cm  = 10u;
    in.clear_right_cm = 25u;
    in.side           = MSG_SIDE_LEFT;

    for (bearing = -63; bearing <= 63; bearing++)
    {
        in.bearing_deg = (int8_t)bearing;
        (void)msg_pack_obstacle(&in, buf, sizeof(buf));
        if (!msg_unpack_obstacle(buf, sizeof(buf), &out)
            || (out.bearing_deg != in.bearing_deg))
        {
            failures++;
        }
    }
    CHECK_EQ_INT(failures, 0, "bearing -63..+63 round trips with sign");

    in.distance_cm    = (uint8_t)MSG_MAX_OBST_DIST;
    in.width_cm       = 200u;       /* over the 6-bit field: clamps */
    in.bearing_deg    = -63;
    in.clear_left_cm  = 99u;        /* over the 5-bit field: clamps */
    in.clear_right_cm = 0u;
    in.side           = MSG_SIDE_RIGHT;
    in.b_saturated    = true;

    (void)msg_pack_obstacle(&in, buf, sizeof(buf));
    CHECK_TRUE(msg_unpack_obstacle(buf, sizeof(buf), &out), "unpack ok");
    CHECK_EQ_INT(out.distance_cm,    255, "distance max");
    CHECK_EQ_INT(out.width_cm,       MSG_MAX_OBST_WIDTH, "width clamps");
    CHECK_EQ_INT(out.bearing_deg,    -63, "bearing min");
    CHECK_EQ_INT(out.clear_left_cm,  MSG_MAX_OBST_CLEAR, "clear left clamps");
    CHECK_EQ_INT(out.clear_right_cm, 0, "clear right zero");
    CHECK_EQ_INT(out.side,           MSG_SIDE_RIGHT, "side");
    CHECK_TRUE(out.b_saturated, "saturated flag");

    buf[3] = (uint8_t)(buf[3] | 0x80u);         /* side low bit */
    buf[4] = (uint8_t)(buf[4] | 0x01u);         /* side high bit: side 3 */
    CHECK_TRUE(!msg_unpack_obstacle(buf, sizeof(buf), &out),
               "undefined side rejected");
}

/* ---------------------------------------------------------------------
 * Barcode message: confidence saturates, undefined command rejected.
 * --------------------------------------------------------------------- */
static void test_barcode_msg(void)
{
    msg_barcode_t in;
    msg_barcode_t out;
    uint8_t       buf[MSG_LEN_BARCODE];

    printf("barcode message: saturation and guard\n");

    in.cmd        = MSG_BC_LEFT;
    in.confidence = 9u;
    in.seq        = 2u;
    (void)msg_pack_barcode(&in, buf, sizeof(buf));
    CHECK_TRUE(msg_unpack_barcode(buf, sizeof(buf), &out), "unpack ok");
    CHECK_EQ_INT(out.cmd, MSG_BC_LEFT, "command");
    CHECK_EQ_INT(out.confidence, 7, "confidence 9 clamps to 7 (was 1)");
    CHECK_EQ_INT(out.seq, 2, "seq");

    buf[0] = 0x05u;                              /* command 5: undefined */
    CHECK_TRUE(!msg_unpack_barcode(buf, sizeof(buf), &out),
               "undefined barcode command rejected");
}

/* ---------------------------------------------------------------------
 * Single-word records shared across cores: line reading and hump.
 * --------------------------------------------------------------------- */
static void test_shared_words(void)
{
    msg_line_t line;
    uint32_t   word;
    int        error;
    int        failures = 0;
    uint8_t    hump;

    printf("single-word records: line reading, hump result\n");

    for (error = -63; error <= 63; error++)
    {
        word = msg_pack_line(error, (0 == (error & 1)), 123456u);
        msg_unpack_line(word, &line);
        if ((line.error != error) || (line.b_valid != (0 == (error & 1)))
            || (line.time_ms != 123456u))
        {
            failures++;
        }
    }
    CHECK_EQ_INT(failures, 0, "line word round trip, -63..+63");

    word = msg_pack_line(200, true, 0u);
    msg_unpack_line(word, &line);
    CHECK_EQ_INT(line.error, 63, "line word clamps 200 to 63");

    /* Age across the 24-bit wrap: stamped just before, read just after. */
    word = msg_pack_line(0, true, 0x00FFFFF0u);
    CHECK_EQ_INT(msg_line_age_ms(word, 0x01000005u), 21,
                 "line age correct across 24-bit wrap");

    hump = msg_pack_hump(85u);
    CHECK_EQ_INT(msg_hump_mm(hump), 85, "hump 85 mm");
    CHECK_TRUE(!msg_hump_saturated(hump), "hump 85 not saturated");

    hump = msg_pack_hump(150u);
    CHECK_EQ_INT(msg_hump_mm(hump), 127, "hump 150 clamps to 127");
    CHECK_TRUE(msg_hump_saturated(hump), "hump 150 flagged SAT");
}

/* ---------------------------------------------------------------------
 * Vehicle state: 3-bit parent + 2-bit substate in 5 bits.
 * --------------------------------------------------------------------- */
static void test_veh_state(void)
{
    unsigned parent;
    unsigned sub;
    uint8_t  encoded;
    uint8_t  got_parent;
    uint8_t  got_sub;
    int      failures = 0;

    printf("vehicle state: parent + substate encoding\n");

    for (parent = 0u; parent <= 7u; parent++)
    {
        for (sub = 0u; sub <= 3u; sub++)
        {
            encoded = msg_encode_veh_state((uint8_t)parent, (uint8_t)sub);
            msg_decode_veh_state(encoded, &got_parent, &got_sub);

            if ((got_parent != parent) || (got_sub != sub) || (encoded > 31u))
            {
                failures++;
            }
        }
    }

    CHECK_EQ_INT(failures, 0, "all 32 state combinations round trip in 5 bits");
}

/* ---------------------------------------------------------------------
 * Short buffers must be refused, not overrun.
 * --------------------------------------------------------------------- */
static void test_buffer_guards(void)
{
    msg_motion_cmd_t cmd = { MSG_CMD_FORWARD, 100u, 1u };
    msg_obstacle_t   obs;
    uint8_t          small[1];

    printf("buffer guards: short buffers refused\n");

    CHECK_EQ_INT(msg_pack_motion_cmd(&cmd, small, 1u), 0,
                 "pack refuses 1-byte buffer");
    CHECK_TRUE(!msg_unpack_motion_cmd(small, 1u, &cmd),
               "unpack refuses 1-byte buffer");
    CHECK_TRUE(!msg_unpack_obstacle(small, 1u, &obs),
               "obstacle unpack refuses short buffer");
    CHECK_EQ_INT(msg_pack_motion_cmd(NULL, small, sizeof(small)), 0,
                 "pack refuses NULL input");
}

/* ---------------------------------------------------------------------
 * Subsystem 3: barcode width classification.
 * --------------------------------------------------------------------- */
static void test_barcode_decode(void)
{
    static uint16_t const patterns[26] =
    {
        0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, 0x00DU,
        0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
        0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U,
        0x0C1U, 0x1C0U, 0x091U, 0x190U, 0x0D0U
    };
    s3_bar_width_t widths[S3_FRAME_ELEMENTS] = {0U};
    uint16_t physical[S3_FRAME_ELEMENTS] = {0U};
    uint16_t symbols[3] = {0x094U, 0U, 0x094U};
    s3_barcode_result_t result = {0};
    uint8_t letter = 0U;
    uint8_t character = 0U;
    uint8_t element = 0U;
    uint8_t count = 0U;
    uint8_t reverse = 0U;

    printf("subsystem 3: real Code 39 alphabet, both orientations\n");
    for (letter = 0U; letter < 26U; letter++)
    {
        symbols[1] = patterns[letter];
        count = 0U;
        for (character = 0U; character < 3U; character++)
        {
            for (element = 0U; element < 9U; element++)
            {
                widths[count] = (0U != (symbols[character]
                    & (uint16_t)(1U << (8U - element)))) ? 10000U : 4000U;
                count++;
            }
            if (character < 2U)
            {
                widths[count] = 4000U;
                count++;
            }
        }
        for (reverse = 0U; reverse < 2U; reverse++)
        {
            for (element = 0U; element < S3_FRAME_ELEMENTS; element++)
            {
                physical[element] = widths[reverse ? 28U - element : element];
            }
            CHECK_TRUE(s3_decode_code39(physical, count, 80000U, 80000U,
                                        &result), "valid alphabet frame");
            CHECK_EQ_INT(result.letter, 'A' + letter, "actual letter retained");
        }
    }
    physical[0] = 0U;
    CHECK_TRUE(!s3_decode_code39(physical, count, 80000U, 80000U, &result),
               "zero-width frame rejected");
    CHECK_TRUE(!s3_decode_code39(widths, (uint8_t)(count - 1U),
                               80000U, 80000U, &result),
               "incomplete frame rejected");
}

/* ---------------------------------------------------------------------
 * Subsystem 4: pitch and hump height.
 * --------------------------------------------------------------------- */
static void test_imu_maths(void)
{
    printf("subsystem 4: pitch and hump height\n");

    CHECK_EQ_INT(s4_pitch_deg(0, 1000), 0, "level -> 0 deg");
    CHECK_EQ_INT(s4_pitch_deg(176, 1000), 10, "tan 0.176 -> 10 deg");
    CHECK_EQ_INT(s4_pitch_deg(-577, 1000), -30, "tan -0.577 -> -30 deg");
    CHECK_EQ_INT(s4_pitch_deg(1000, 1000), 45, "tan 1 -> 45 deg");
    CHECK_EQ_INT(s4_pitch_deg(1000, 0), 90, "vertical -> 90 deg");

    CHECK_EQ_INT(s4_height_increment_um(0, 1000), 0,
                 "level chassis adds no height");
    CHECK_EQ_INT(s4_height_increment_um(10, 100), 17187,
                 "10 deg over 100 mm = 17.2 mm");
}

/* ---------------------------------------------------------------------
 * Subsystem 5: ranging, side choice and the obstacle profile.
 * --------------------------------------------------------------------- */
static void test_obstacle_profile(void)
{
    uint8_t        scan[S5_FINE_STEPS];
    msg_obstacle_t profile;
    uint32_t       idx;

    printf("subsystem 5: ranging and obstacle profile\n");

    CHECK_EQ_INT(s5_echo_us_to_cm(580u), 10, "580 us echo = 10 cm");
    CHECK_EQ_INT(s5_choose_side(30u, 10u, 23u), MSG_SIDE_LEFT,
                 "wider left side chosen");
    CHECK_EQ_INT(s5_choose_side(5u, 5u, 23u), MSG_SIDE_NONE,
                 "neither side fits -> NONE");
    CHECK_EQ_INT(s5_lateral_cm(50u, -30), -25, "50 cm at -30 deg = -25 cm");

    /* A 25 cm-distant box from -10 to +10 degrees, a wall at 40 cm on the
       far right (50..60 degrees), nothing else in range. */
    memset(scan, 0, sizeof(scan));
    for (idx = 10u; idx <= 14u; idx++)
    {
        scan[idx] = 25u;
    }
    for (idx = 22u; idx <= 24u; idx++)
    {
        scan[idx] = 40u;
    }

    CHECK_TRUE(s5_build_profile(scan, S5_FINE_STEPS, 30u, &profile),
               "profile built");
    CHECK_EQ_INT(profile.distance_cm, 25, "closest point 25 cm");
    CHECK_EQ_INT(profile.bearing_deg, -10, "bearing of closest point");
    CHECK_EQ_INT(profile.width_cm, 8, "width from the angular span");
    CHECK_EQ_INT(profile.clear_left_cm, MSG_MAX_OBST_CLEAR,
                 "open left side saturates at 31 cm");
    CHECK_TRUE(profile.b_saturated, "saturation flagged");
    CHECK_EQ_INT(profile.clear_right_cm, 26, "right side blocked by the wall");
    CHECK_EQ_INT(profile.side, MSG_SIDE_LEFT, "bypass on the left");

    memset(scan, 0, sizeof(scan));
    CHECK_TRUE(!s5_build_profile(scan, S5_FINE_STEPS, 30u, &profile),
               "nothing in range -> no profile (false alarm)");
}

/* ---------------------------------------------------------------------
 * CR1 deliverable 3: message sizes.  "Before" is sizeof() of each
 * unpacked struct on THIS compiler -- measured, not typed in.  The kernel
 * column is what one message costs inside a micro T-Kernel message buffer
 * (4-byte header + body rounded up to 4).  Run `make target-sizes` for
 * the same table on arm-none-eabi-gcc, which uses 1-byte enums.
 * --------------------------------------------------------------------- */
static void report_row(const char *p_name, unsigned before, unsigned after)
{
    printf("  %-18s %7u %7u %6u%%   %7u %7u\n", p_name, before, after,
           (100u * (before - after)) / before,
           (unsigned)APP_MBF_SLOT_BYTES(before),
           (unsigned)APP_MBF_SLOT_BYTES(after));
}

static void report_sizes(void)
{
    printf("\n--- CR1: message size, this compiler (sizeof) ---\n");
    printf("  %-18s %7s %7s %7s   %15s\n", "message", "before", "after",
           "saving", "kernel slot b/a");
    report_row("motion command", (unsigned)sizeof(msg_motion_cmd_t),
               MSG_LEN_MOTION_CMD);
    report_row("barcode command", (unsigned)sizeof(msg_barcode_t),
               MSG_LEN_BARCODE);
    report_row("obstacle profile", (unsigned)sizeof(msg_obstacle_t),
               MSG_LEN_OBSTACLE);
    report_row("telemetry frame", (unsigned)sizeof(msg_telemetry_t),
               MSG_LEN_TELEMETRY);
    printf("  line reading       12 -> 4 B (one word), hump result 3 -> 1 B\n");
}

/* ---------------------------------------------------------------------
 * Subsystem 2: duty clamp, geometry, PID and the duty slew limiter.
 * --------------------------------------------------------------------- */
static void test_motor_maths(void)
{
    int32_t applied_l = 0;
    int32_t applied_r = 0;
    int32_t output    = 0;
    int32_t bound;
    int32_t back_mm;
    int     idx;

    printf("subsystem 2: clamp, geometry, PID, slew limiter\n");

    CHECK_EQ_INT(motor_clamp_duty(-5), 0, "negative duty clamps to 0");
    CHECK_EQ_INT(motor_clamp_duty(150), 100, "duty over 100 clamps to 100");
    CHECK_EQ_INT(motor_clamp_duty(42), 42, "in-range duty unchanged");

    back_mm = motor_counts_to_mm(motor_mm_to_counts(1000));
    CHECK_TRUE((back_mm > 950) && (back_mm <= 1000),
               "1000 mm -> counts -> mm within one count");
    CHECK_TRUE(motor_degrees_to_counts(180)
               >= (2 * motor_degrees_to_counts(90)) - 1,
               "180 deg turn is twice a 90 deg turn");

    /* Line PID is called as pid(0, line_error): a line to the RIGHT
       (positive error) must give a NEGATIVE correction, which speeds the
       left wheel and turns the car right, toward the line. */
    motor_pid_reset();
    CHECK_TRUE(motor_pid_step(0, 20, true) < 0,
               "line to the right -> turn right");
    motor_pid_reset();
    CHECK_TRUE(motor_pid_step(0, -20, true) > 0,
               "line to the left -> turn left");

    /* Integral clamp: a long saturated error cannot wind up past the
       configured limit (the 187% wind-up seen on the car). */
    motor_pid_reset();
    for (idx = 0; idx < 2000; idx++)
    {
        output = motor_pid_step(0, -100, true);
    }
    bound = ((100 * S2_KP_Q8) + (S2_INTEGRAL_LIMIT * S2_KI_Q8)) / 256 + 1;
    CHECK_TRUE(output <= bound, "integral wind-up is capped");

    /* Slew limiter: duty rises S2_SLEW_PCT per call, the brake is
       immediate. */
    motor_brake();
    motor_set(1, 40u, 1, 40u);
    motor_get_applied(&applied_l, &applied_r);
    CHECK_EQ_INT(applied_l, S2_SLEW_PCT, "first step limited to the slew");
    for (idx = 0; idx < 20; idx++)
    {
        motor_set(1, 40u, 1, 40u);
    }
    motor_get_applied(&applied_l, &applied_r);
    CHECK_EQ_INT(applied_r, 40, "duty reaches the request");
    motor_set(-1, 40u, -1, 40u);
    motor_get_applied(&applied_l, &applied_r);
    CHECK_EQ_INT(applied_l, 40 - S2_SLEW_PCT, "reversal is rate limited too");
    motor_brake();
    motor_get_applied(&applied_l, &applied_r);
    CHECK_TRUE((0 == applied_l) && (0 == applied_r),
               "brake is immediate");
}

/* One suite of checks and the buddy who owns the code it tests. */
typedef struct
{
    void       (*p_run)(void);
    int          owner;          /* 1..5, 0 = shared (run for everyone) */
} suite_t;

static const suite_t suites[] =
{
    { test_saturation,           1 },   /* msg_defs field saturation   */
    { test_telemetry_saturation, 1 },
    { test_telemetry,            1 },
    { test_buffer_guards,        1 },
    { test_motion_cmd,           2 },
    { test_motor_maths,          2 },
    { test_barcode_msg,          3 },
    { test_barcode_decode,       3 },
    { test_imu_maths,            4 },
    { test_obstacle,             5 },
    { test_veh_state,            5 },
    { test_obstacle_profile,     5 },
    { test_shared_words,         0 },   /* line word (B3) + hump (B4)  */
};

/*
 *   ./test_host        every suite (the whole team)
 *   ./test_host N      only Buddy N's suites, plus the shared ones
 */
int main(int argc, char **argv)
{
    int    buddy = 0;
    size_t idx;

    if (argc > 1)
    {
        buddy = (int)(argv[1][0] - '0');
        if ((buddy < 1) || (buddy > 5) || ('\0' != argv[1][1]))
        {
            printf("usage: %s [1-5]   (no argument = every buddy)\n",
                   argv[0]);
            return (2);
        }
    }

    if (0 == buddy)
    {
        printf("=== Group AAI18 host tests: all buddies ===\n\n");
    }
    else
    {
        printf("=== Group AAI18 host tests: Buddy %d ===\n\n", buddy);
    }

    for (idx = 0u; idx < (sizeof(suites) / sizeof(suites[0])); idx++)
    {
        if ((0 == buddy) || (0 == suites[idx].owner)
            || (buddy == suites[idx].owner))
        {
            suites[idx].p_run();
        }
    }

    if ((0 == buddy) || (1 == buddy))
    {
        report_sizes();     /* CR1 size table: msg_defs, Buddy 1 */
    }

    printf("\n=== %d/%d checks passed ===\n", tests_passed, tests_run);

    return ((tests_passed == tests_run) ? 0 : 1);
}

/* end of file */
