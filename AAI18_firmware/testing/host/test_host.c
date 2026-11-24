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
#include <inttypes.h>
#include <stdbool.h>

#include "app.h"
#include "msg_defs.h"
#include "subsystem2/motor_control.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/subsystem5.h"

static int32_t tests_run    = 0;
static int32_t tests_passed = 0;

#define CHECK_TRUE(cond, label)                                         \
    do                                                                  \
    {                                                                   \
        tests_run++;                                                    \
        if (cond)                                                       \
        {                                                               \
            tests_passed++;                                             \
        }                                                               \
        else                                                            \
        {                                                               \
            printf("  FAIL: %s  (%s:%d)\n", (label), __FILE__,          \
                   __LINE__);                                           \
        }                                                               \
    } while (0)

#define CHECK_EQ_INT(got, want, label)                                  \
    do                                                                  \
    {                                                                   \
        tests_run++;                                                    \
        if ((int64_t)(want) == (int64_t)(got))                          \
        {                                                               \
            tests_passed++;                                             \
        }                                                               \
        else                                                            \
        {                                                               \
            printf("  FAIL: %s  got %" PRId64 " want %" PRId64          \
                   "  (%s:%d)\n", (label), (int64_t)(got),              \
                   (int64_t)(want), __FILE__, __LINE__);                \
        }                                                               \
    } while (0)

/* ---------------------------------------------------------------------
 * Motion command: every defined type, argument and sequence (7 x 1024 x
 * 8 combinations), and refusal of the one undefined code.
 * --------------------------------------------------------------------- */
static void test_motion_cmd(void)
{
    msg_motion_cmd_t msg_in;
    msg_motion_cmd_t out;
    uint8_t          buf[MSG_LEN_MOTION_CMD];
    int32_t          failures = 0;
    uint32_t         type;
    uint32_t         arg;
    uint32_t         seq;

    printf("motion command: exhaustive round trip\n");

    for (type = 0u; type <= (uint32_t)MSG_CMD_LAST; type++)
    {
        for (arg = 0u; arg <= MSG_MAX_CMD_ARG; arg++)
        {
            for (seq = 0u; seq <= 7u; seq++)
            {
                msg_in.type = (msg_cmd_type_t)type;
                msg_in.arg  = (uint16_t)arg;
                msg_in.seq  = (uint8_t)seq;

                memset(buf, 0, sizeof(buf));
                if ((MSG_LEN_MOTION_CMD != msg_pack_motion_cmd(&msg_in, buf,
                                                               sizeof(buf)))
                    || !msg_unpack_motion_cmd(buf, sizeof(buf), &out)
                    || (out.type != msg_in.type) || (out.arg != msg_in.arg)
                    || (out.seq != msg_in.seq))
                {
                    failures++;
                }
            }
        }
    }

    CHECK_EQ_INT(failures, 0, "motion cmd exhaustive round trip");
    printf("  %u combinations checked, %d bytes per message\n",
           ((uint32_t)MSG_CMD_LAST + 1u) * (MSG_MAX_CMD_ARG + 1u) * 8u,
           MSG_LEN_MOTION_CMD);

    /* T2: an undefined code must never become a movement command. */
    buf[0] = 0x07u;                 /* type 7, arg 0, seq 0 */
    buf[1] = 0x00u;
    CHECK_TRUE(!msg_unpack_motion_cmd(buf, sizeof(buf), &out),
               "undefined command type 7 rejected on unpack");

    msg_in.type = (msg_cmd_type_t)7;
    msg_in.arg  = 10u;
    msg_in.seq  = 0u;
    CHECK_EQ_INT(msg_pack_motion_cmd(&msg_in, buf, sizeof(buf)), 0,
                 "undefined command type 7 refused on pack");
}

/* ---------------------------------------------------------------------
 * Saturation: out-of-range must CLAMP, never wrap (rule R5).
 * --------------------------------------------------------------------- */
static void test_saturation(void)
{
    msg_motion_cmd_t msg_in;
    msg_motion_cmd_t out;
    uint8_t          buf[MSG_LEN_MOTION_CMD];
    bool             b_sat;

    printf("saturation: out-of-range must clamp, never wrap\n");

    msg_in.type = MSG_CMD_FORWARD;
    msg_in.arg  = 2000u;
    msg_in.seq  = 1u;
    (void)msg_pack_motion_cmd(&msg_in, buf, sizeof(buf));
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
    msg_telemetry_t msg_in;
    msg_telemetry_t out;
    uint8_t         buf[MSG_LEN_TELEMETRY];

    printf("telemetry: saturation on every field\n");

    memset(&msg_in, 0, sizeof(msg_in));
    msg_in.tilt_deg   = 80;
    msg_in.speed_pct  = 200u;
    msg_in.line_error = 100;
    msg_in.hump_mm    = 200u;

    (void)msg_pack_telemetry(&msg_in, buf, sizeof(buf));
    CHECK_TRUE(msg_unpack_telemetry(buf, sizeof(buf), &out), "unpack ok");
    CHECK_EQ_INT(out.tilt_deg, 63, "tilt 80 clamps to 63 (was -48)");
    CHECK_EQ_INT(out.speed_pct, 100, "speed 200 clamps to 100 (was 72)");
    CHECK_EQ_INT(out.line_error, 63, "line error 100 clamps to 63 (was -28)");
    CHECK_EQ_INT(out.hump_mm, 127, "hump 200 clamps to 127 (was 72)");

    msg_in.tilt_deg   = -90;
    msg_in.line_error = -100;
    (void)msg_pack_telemetry(&msg_in, buf, sizeof(buf));
    (void)msg_unpack_telemetry(buf, sizeof(buf), &out);
    CHECK_EQ_INT(out.tilt_deg, -63, "tilt -90 clamps to -63");
    CHECK_EQ_INT(out.line_error, -63, "line error -100 clamps to -63");
}

/* ---------------------------------------------------------------------
 * Telemetry frame: round trip, schema guard, enum guard.
 * --------------------------------------------------------------------- */
static void test_telemetry(void)
{
    msg_telemetry_t msg_in;
    msg_telemetry_t out;
    uint8_t         buf[MSG_LEN_TELEMETRY];

    printf("telemetry frame: round trip and guards\n");

    memset(&msg_in, 0, sizeof(msg_in));
    msg_in.seq              = 42u;
    msg_in.veh_state        = msg_encode_veh_state(2u, 3u);
    msg_in.line_state       = (uint8_t)MSG_LINE_TRACK;
    msg_in.line_error       = -45;
    msg_in.b_line_valid     = true;
    msg_in.speed_pct        = 55u;
    msg_in.hump_mm          = 120u;
    msg_in.b_hump_sat       = true;
    msg_in.b_hump_invalid   = true;
    msg_in.tilt_deg         = -12;
    msg_in.obst_distance_cm = 200u;
    msg_in.fault_mask       = 0x0285u;
    msg_in.wifi_state       = 3u;
    msg_in.last_barcode     = (uint8_t)MSG_BC_RIGHT;
    msg_in.encoder_delta    = 40000u;

    CHECK_EQ_INT(msg_pack_telemetry(&msg_in, buf, sizeof(buf)),
                 MSG_LEN_TELEMETRY, "pack length");
    CHECK_TRUE(msg_unpack_telemetry(buf, sizeof(buf), &out), "unpack ok");

    CHECK_EQ_INT(out.seq,              msg_in.seq,              "seq");
    CHECK_EQ_INT(out.veh_state,        msg_in.veh_state,        "veh_state");
    CHECK_EQ_INT(out.line_state,       msg_in.line_state,       "line_state");
    CHECK_EQ_INT(out.line_error,       msg_in.line_error,       "line_error");
    CHECK_TRUE(out.b_line_valid    ==  msg_in.b_line_valid,     "line_valid");
    CHECK_EQ_INT(out.speed_pct,        msg_in.speed_pct,        "speed");
    CHECK_EQ_INT(out.hump_mm,          msg_in.hump_mm,          "hump_mm");
    CHECK_TRUE(out.b_hump_sat      ==  msg_in.b_hump_sat,       "hump_sat");
    CHECK_TRUE(out.b_hump_invalid  ==  msg_in.b_hump_invalid,   "hump_invalid");
    CHECK_EQ_INT(out.tilt_deg,         msg_in.tilt_deg,         "tilt");
    CHECK_EQ_INT(out.obst_distance_cm, msg_in.obst_distance_cm, "obst dist");
    CHECK_EQ_INT(out.fault_mask,       msg_in.fault_mask,       "fault mask");
    CHECK_EQ_INT(out.wifi_state,       msg_in.wifi_state,       "wifi state");
    CHECK_EQ_INT(out.last_barcode,     msg_in.last_barcode,     "last barcode");
    CHECK_EQ_INT(out.encoder_delta,    msg_in.encoder_delta,    "enc delta");

    buf[0] = (uint8_t)(MSG_SCHEMA_VERSION + 1u);
    CHECK_TRUE(!msg_unpack_telemetry(buf, sizeof(buf), &out),
               "wrong schema version rejected");

    (void)msg_pack_telemetry(&msg_in, buf, sizeof(buf));
    buf[2] = (uint8_t)(buf[2] | 0xE0u);         /* line_state = 7 */
    CHECK_TRUE(!msg_unpack_telemetry(buf, sizeof(buf), &out),
               "undefined line_state rejected");
}

/* ---------------------------------------------------------------------
 * Obstacle profile (T3): signed bearing, boundaries, enum guard.
 * --------------------------------------------------------------------- */
static void test_obstacle(void)
{
    msg_obstacle_t msg_in;
    msg_obstacle_t out;
    uint8_t        buf[MSG_LEN_OBSTACLE];
    int32_t        bearing;
    int32_t        failures = 0;

    printf("obstacle profile: bearing range, boundaries, guards\n");

    memset(&msg_in, 0, sizeof(msg_in));
    msg_in.distance_cm    = 100u;
    msg_in.width_cm       = 20u;
    msg_in.clear_left_cm  = 10u;
    msg_in.clear_right_cm = 25u;
    msg_in.side           = MSG_SIDE_LEFT;

    for (bearing = -63; bearing <= 63; bearing++)
    {
        msg_in.bearing_deg = (int8_t)bearing;
        (void)msg_pack_obstacle(&msg_in, buf, sizeof(buf));
        if (!msg_unpack_obstacle(buf, sizeof(buf), &out)
            || (out.bearing_deg != msg_in.bearing_deg))
        {
            failures++;
        }
    }
    CHECK_EQ_INT(failures, 0, "bearing -63..+63 round trips with sign");

    msg_in.distance_cm    = (uint8_t)MSG_MAX_OBST_DIST;
    msg_in.width_cm       = 200u;       /* over the 6-bit field: clamps */
    msg_in.bearing_deg    = -63;
    msg_in.clear_left_cm  = 99u;        /* over the 5-bit field: clamps */
    msg_in.clear_right_cm = 0u;
    msg_in.side           = MSG_SIDE_RIGHT;
    msg_in.b_saturated    = true;

    (void)msg_pack_obstacle(&msg_in, buf, sizeof(buf));
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
    msg_barcode_t msg_in;
    msg_barcode_t out;
    uint8_t       buf[MSG_LEN_BARCODE];

    printf("barcode message: saturation and guard\n");

    msg_in.cmd        = MSG_BC_LEFT;
    msg_in.confidence = 9u;
    msg_in.seq        = 2u;
    (void)msg_pack_barcode(&msg_in, buf, sizeof(buf));
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
    int32_t    error;
    bool       b_even;
    int32_t    failures = 0;
    uint8_t    hump;

    printf("single-word records: line reading, hump result\n");

    for (error = -63; error <= 63; error++)
    {
        b_even = (0 == (error % 2));
        word   = msg_pack_line(error, b_even, 123456u);
        msg_unpack_line(word, &line);
        if ((error != line.error) || (b_even != line.b_valid)
            || (123456u != line.time_ms))
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
    uint32_t parent;
    uint32_t sub;
    uint8_t  encoded;
    uint8_t  got_parent;
    uint8_t  got_sub;
    int32_t  failures = 0;

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
/* Declared in barcode_decoder.c (not yet in subsystem3.h). */
bool s3_code39_decode(const uint16_t *p_widths, uint8_t count,
                      char *p_letter, bool *pb_reversed,
                      uint8_t *p_confidence);
msg_barcode_cmd_t s3_letter_command(char letter);

/* Command letters as set in barcode_decoder.c (S3_BC_LETTER_*). */
#define TEST_LETTER_LEFT        ('M')
#define TEST_LETTER_RIGHT       ('O')
#define TEST_LETTER_STRAIGHT    ('N')
#define TEST_LETTER_UTURN       ('P')

/* Code 39 patterns, 9 elements bar/space alternating, MSB = first. */
static uint16_t c39_pattern(char letter)
{
    static const struct { char letter; uint16_t pattern; } tab[] =
    {
        { '*', 0x094u }, { 'B', 0x049u }, { 'D', 0x019u }, { 'E', 0x118u },
        { 'F', 0x058u }, { 'L', 0x043u }, { 'M', 0x142u }, { 'N', 0x013u },
        { 'O', 0x112u }, { 'P', 0x052u }
    };
    size_t  idx;

    for (idx = 0u; idx < (sizeof(tab) / sizeof(tab[0])); idx++)
    {
        if (tab[idx].letter == letter)
        {
            return (tab[idx].pattern);
        }
    }
    return (0u);
}

/* Widths (0.1 mm) of "*X*": 29 elements, narrow / wide as given. */
static uint8_t c39_symbol(char letter, uint16_t narrow, uint16_t wide,
                          uint16_t *p_out)
{
    const char text[3] = { '*', letter, '*' };
    uint8_t    count = 0u;
    uint32_t   pos;
    int32_t bit;

    for (pos = 0u; pos < 3u; pos++)
    {
        uint16_t pattern = c39_pattern(text[pos]);

        for (bit = 8; bit >= 0; bit--)
        {
            p_out[count] = (0u != (pattern & (1u << (uint32_t)bit)))
                           ? wide : narrow;
            count = (uint8_t)(count + 1u);
        }
        if (pos < 2u)
        {
            p_out[count] = narrow;              /* inter-character gap */
            count = (uint8_t)(count + 1u);
        }
    }
    return (count);
}

static void test_barcode_decode(void)
{
    uint16_t widths[40];
    uint16_t back[40];
    uint8_t  conf = 0u;
    uint8_t  count;
    uint8_t  idx;
    char     letter = '\0';
    bool     b_rev  = false;

    printf("subsystem 3: Code 39 barcode decoding\n");

    count = c39_symbol(TEST_LETTER_STRAIGHT, 100u, 300u, widths);
    CHECK_EQ_INT(count, 29, "*X* is 29 elements");
    CHECK_TRUE(s3_code39_decode(widths, count, &letter, &b_rev, &conf),
               "straight letter decodes");
    CHECK_EQ_INT(letter, TEST_LETTER_STRAIGHT, "straight letter");
    CHECK_TRUE(!b_rev, "read forwards");
    CHECK_EQ_INT(s3_decode_widths(widths, count, &conf), MSG_BC_STRAIGHT,
                 "straight letter -> STRAIGHT");

    /* Read backwards (car driving the other way). */
    count = c39_symbol(TEST_LETTER_UTURN, 100u, 300u, widths);
    for (idx = 0u; idx < count; idx++)
    {
        back[idx] = widths[count - 1u - idx];
    }
    CHECK_EQ_INT(s3_decode_widths(back, count, &conf), MSG_BC_U_TURN,
                 "U-turn letter read backwards -> U_TURN");

    /* Speed independence: the long sheet is the same at half scale. */
    count = c39_symbol(TEST_LETTER_LEFT, 50u, 150u, widths);
    CHECK_EQ_INT(s3_decode_widths(widths, count, &conf), MSG_BC_LEFT,
                 "left letter at half scale -> LEFT");

    /* A stray element (sheet edge) before the first bar. */
    back[0] = 400u;
    count = c39_symbol(TEST_LETTER_RIGHT, 100u, 250u, &back[1]);
    CHECK_EQ_INT(s3_decode_widths(back, (uint8_t)(count + 1u), &conf),
                 MSG_BC_RIGHT, "stray leading element skipped -> RIGHT");

    /* A valid letter that is not a command. */
    count = c39_symbol('E', 100u, 300u, widths);
    CHECK_EQ_INT(s3_decode_widths(widths, count, &conf), MSG_BC_NONE,
                 "non-command letter -> NONE");

    /* Noise: no wide elements at all. */
    for (idx = 0u; idx < 29u; idx++)
    {
        widths[idx] = 100u;
    }
    CHECK_EQ_INT(s3_decode_widths(widths, 29u, &conf), MSG_BC_INVALID,
                 "all-narrow noise rejected");
    CHECK_EQ_INT(s3_decode_widths(widths, 28u, &conf), MSG_BC_INVALID,
                 "too few elements rejected");
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

    /* A 25 cm-distant box echoing from -20 to +20 degrees (really -10 to
       +10: the beam adds S5_BEAM_HALF_DEG each side), a wall at 40 cm on
       the far right (the last two steps), nothing else in range. */
    /* Indices follow the configured scan arc (S5_SCAN_ARC_DEG). */
    memset(scan, 0, sizeof(scan));
    for (idx = (uint32_t)((S5_FINE_ARC_DEG - 10 - S5_BEAM_HALF_DEG)
                          / S5_STEP_DEG);
         idx <= (uint32_t)((S5_FINE_ARC_DEG + 10 + S5_BEAM_HALF_DEG)
                           / S5_STEP_DEG); idx++)
    {
        scan[idx] = 25u;
    }
    for (idx = S5_FINE_STEPS - 2u; idx < S5_FINE_STEPS; idx++)
    {
        scan[idx] = 40u;
    }

    CHECK_TRUE(s5_build_profile(scan, S5_FINE_STEPS, 30u, &profile),
               "profile built");
    CHECK_EQ_INT(profile.distance_cm, 25, "closest point 25 cm");
    CHECK_EQ_INT(profile.bearing_deg, -10 - S5_BEAM_HALF_DEG,
                 "bearing of closest point");
    CHECK_EQ_INT(profile.width_cm, 8 - (2 * S5_BEAM_SPREAD_CM),
                 "width from the angular span, minus the beam spread");
    CHECK_TRUE(profile.clear_left_cm > profile.clear_right_cm,
               "open left side has more room than the walled right");
    CHECK_EQ_INT(profile.clear_left_cm, MSG_MAX_OBST_CLEAR,
                 "left open to the scan edge counts as unlimited room");
    CHECK_EQ_INT(profile.side, MSG_SIDE_LEFT, "bypass on the open left");

    /* Object filling the whole scan: neither side is known to be open. */
    memset(scan, 25u, sizeof(scan));
    CHECK_TRUE(s5_build_profile(scan, S5_FINE_STEPS, 30u, &profile),
               "wall-to-wall object profiled");
    CHECK_EQ_INT(profile.side, MSG_SIDE_NONE,
                 "object across the whole scan -> NONE (back off)");

    memset(scan, 0, sizeof(scan));
    CHECK_TRUE(!s5_build_profile(scan, S5_FINE_STEPS, 30u, &profile),
               "nothing in range -> no profile (false alarm)");

    if (13u == S5_FINE_STEPS)
    {
        /* Bench 17 Nov: echoes -20..+15 deg at ~29 cm.  Beam-corrected
           (10 deg) the edges are -10 / +5 deg, plus one step out: left
           ~-7 cm, right ~+5 cm, both packed in one 16-bit word (CR1). */
        static const uint8_t bench[13] = { 0u, 0u, 29u, 31u, 29u, 29u,
                                           29u, 29u, 30u, 30u, 0u, 0u, 0u };
        /* Test 10: block running off the LEFT end, right edge seen. */
        static const uint8_t t10[13] = { 30u, 29u, 29u, 30u, 30u, 30u,
                                         29u, 30u, 30u, 30u, 32u, 0u, 0u };
        int32_t edge_left;
        int32_t edge_right;

        CHECK_TRUE(s5_build_profile(bench, 13u, 30u, &profile),
                   "bench scan -> profile");
        s5_get_edges_cm(&edge_left, &edge_right);
        CHECK_TRUE((edge_right >= 4) && (edge_right <= 6),
                   "bench scan: right edge ~+5 cm (beam-corrected + step)");
        CHECK_TRUE((edge_left <= -6) && (edge_left >= -9),
                   "bench scan: left edge ~-7 cm (beam-corrected + step)");

        CHECK_TRUE(s5_build_profile(t10, 13u, 30u, &profile),
                   "test 10 scan -> profile");
        CHECK_EQ_INT(profile.side, MSG_SIDE_RIGHT,
                     "test 10: goes round the open right");
    }
}

/* ---------------------------------------------------------------------
 * CR1 deliverable 3: message sizes.  "Before" is sizeof() of each
 * unpacked struct on THIS compiler -- measured, not typed msg_in.  The kernel
 * column is what one message costs inside a micro T-Kernel message buffer
 * (4-byte header + body rounded up to 4).  Run `make target-sizes` for
 * the same table on arm-none-eabi-gcc, which uses 1-byte enums.
 * --------------------------------------------------------------------- */
static void report_row(const char *p_name, uint32_t before, uint32_t after)
{
    printf("  %-18s %7u %7u %6u%%   %7u %7u\n", p_name, before, after,
           (100u * (before - after)) / before,
           (uint32_t)APP_MBF_SLOT_BYTES(before),
           (uint32_t)APP_MBF_SLOT_BYTES(after));
}

static void report_sizes(void)
{
    printf("\n--- CR1: message size, this compiler (sizeof) ---\n");
    printf("  %-18s %7s %7s %7s   %15s\n", "message", "before", "after",
           "saving", "kernel slot b/a");
    report_row("motion command", (uint32_t)sizeof(msg_motion_cmd_t),
               MSG_LEN_MOTION_CMD);
    report_row("barcode command", (uint32_t)sizeof(msg_barcode_t),
               MSG_LEN_BARCODE);
    report_row("obstacle profile", (uint32_t)sizeof(msg_obstacle_t),
               MSG_LEN_OBSTACLE);
    report_row("telemetry frame", (uint32_t)sizeof(msg_telemetry_t),
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
    int32_t idx;

    printf("subsystem 2: clamp, geometry, PID, slew limiter\n");

    CHECK_EQ_INT(motor_clamp_duty(-5), 0, "negative duty clamps to 0");
    CHECK_EQ_INT(motor_clamp_duty(150), 100, "duty over 100 clamps to 100");
    CHECK_EQ_INT(motor_clamp_duty(42), 42, "in-range duty unchanged");

    back_mm = motor_counts_to_mm(motor_mm_to_counts(1000, true), true);
    CHECK_TRUE((back_mm > 990) && (back_mm <= 1000),
               "left: 1000 mm -> counts -> mm round trip");
    back_mm = motor_counts_to_mm(motor_mm_to_counts(1000, false), false);
    CHECK_TRUE((back_mm > 990) && (back_mm <= 1000),
               "right: 1000 mm -> counts -> mm round trip");
    CHECK_TRUE(motor_mm_to_counts(1000, true)
               > motor_mm_to_counts(1000, false),
               "each wheel uses its own counts per revolution");
    CHECK_TRUE(motor_degrees_to_counts(180, true)
               >= (2 * motor_degrees_to_counts(90, true)) - 1,
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
    int32_t      owner;          /* 1..5, 0 = shared (run for everyone) */
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
    int32_t buddy = 0;
    size_t  idx;

    if (argc > 1)
    {
        buddy = (int32_t)(argv[1][0] - '0');
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
