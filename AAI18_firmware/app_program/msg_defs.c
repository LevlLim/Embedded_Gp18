/*
 * msg_defs.c
 *
 * Group AAI18 -- minimum-bit encode and decode helpers.
 *
 * Implements Change Requirement 1 / NFR12.  Every function here is pure
 * apart from the sticky saturation flag: it touches no hardware and no
 * kernel object, so it can be unit tested on a host PC.  testing/host/
 * runs these same functions to produce the round-trip evidence required
 * by section 7.5 of the design review, without needing the robot.
 *
 * Signed fields are stored as two's complement in the stated number of bits
 * and sign-extended on the way out.  Bitwise operators are applied only to
 * unsigned containers (Barr-C 5.3b).
 */

#include "msg_defs.h"

/* ---------------------------------------------------------------------
 * Private data
 * --------------------------------------------------------------------- */

/* Sticky "something was clamped" flag.  Only ever set to true, so a
   concurrent store from another task or core cannot lose information. */
static volatile bool b_saturation_seen = false;

/* ---------------------------------------------------------------------
 * Private helpers
 * --------------------------------------------------------------------- */

/*
 * Record that a clamp happened, both for the caller and for bring-up.
 */
static void note_saturation(bool *pb_saturated)
{
    b_saturation_seen = true;

    if (NULL != pb_saturated)
    {
        *pb_saturated = true;
    }
}

/*
 * Sign-extend the low 'bits' of an unsigned container into a signed value.
 */
static int32_t sign_extend(uint32_t raw, uint8_t bits)
{
    uint32_t sign_bit = (uint32_t)1u << (bits - 1u);
    int32_t  result;

    if (0u != (raw & sign_bit))
    {
        /* Set every bit above the field so the value becomes negative.
           The cast reinterprets the two's complement pattern. */
        result = (int32_t)(raw | ~((sign_bit << 1u) - 1u));
    }
    else
    {
        result = (int32_t)raw;
    }

    return (result);
}

/*
 * Mask a signed value into the low 'bits' of an unsigned container.
 */
static uint32_t to_field(int32_t value, uint8_t bits)
{
    uint32_t mask = ((uint32_t)1u << bits) - 1u;

    /* The cast keeps the two's complement pattern; the mask trims it. */
    return ((uint32_t)value & mask);
}

/*
 * Saturate an unsigned value into a field of 'limit' and return it in a
 * 32-bit container ready for shifting.
 */
static uint32_t sat_field(uint32_t value, uint32_t limit)
{
    uint32_t result = value;

    if (value > limit)
    {
        result = limit;
        note_saturation(NULL);
    }

    return (result);
}

/* True if code is a defined msg_barcode_cmd_t. */
static bool is_barcode_code(uint32_t code)
{
    return ((code <= (uint32_t)MSG_BC_U_TURN)
            || ((uint32_t)MSG_BC_INVALID == code));
}

/* ---------------------------------------------------------------------
 * Public: saturating encoders (design rule R5)
 * --------------------------------------------------------------------- */

uint16_t msg_sat_u16(uint32_t value, uint16_t limit, bool *pb_saturated)
{
    uint16_t result;

    if (value > (uint32_t)limit)
    {
        result = limit;
        note_saturation(pb_saturated);
    }
    else
    {
        result = (uint16_t)value;   /* in range, so no truncation */
    }

    return (result);
}

int8_t msg_sat_i8(int32_t value, int8_t limit, bool *pb_saturated)
{
    int8_t result;

    if (value > (int32_t)limit)
    {
        result = limit;
        note_saturation(pb_saturated);
    }
    else if (value < -(int32_t)limit)
    {
        result = (int8_t)(-limit);  /* limit <= 127, so -limit fits */
        note_saturation(pb_saturated);
    }
    else
    {
        result = (int8_t)value;     /* in range, so no truncation */
    }

    return (result);
}

bool msg_saturation_seen(void)
{
    return (b_saturation_seen);
}

/* ---------------------------------------------------------------------
 * Public: vehicle state (3-bit parent + 2-bit substate)
 * --------------------------------------------------------------------- */

uint8_t msg_encode_veh_state(uint8_t parent, uint8_t substate)
{
    /* Both inputs are masked to their widths, so the result is < 32. */
    return ((uint8_t)((((uint32_t)parent & 0x07u) << 2u)
                      | ((uint32_t)substate & 0x03u)));
}

void msg_decode_veh_state(uint8_t encoded, uint8_t *p_parent,
                          uint8_t *p_substate)
{
    if (NULL != p_parent)
    {
        *p_parent = (uint8_t)(((uint32_t)encoded >> 2u) & 0x07u);
    }

    if (NULL != p_substate)
    {
        *p_substate = (uint8_t)((uint32_t)encoded & 0x03u);
    }
}

/* ---------------------------------------------------------------------
 * Public: motion command, 2 bytes
 *   bits  0..2   command type
 *   bits  3..12  argument (mm or degrees), saturating
 *   bits 13..15  sequence number
 * --------------------------------------------------------------------- */

uint8_t msg_pack_motion_cmd(const msg_motion_cmd_t *p_cmd,
                            uint8_t *p_buf, uint8_t buf_len)
{
    uint32_t word;
    uint8_t  written = 0u;

    /* An undefined command type is refused outright.  Clamping it would
       turn a corrupt value into a real manoeuvre (code 6 is REVERSE). */
    if ((NULL != p_cmd) && (NULL != p_buf) && (buf_len >= MSG_LEN_MOTION_CMD)
        && ((uint32_t)p_cmd->type <= (uint32_t)MSG_CMD_LAST))
    {
        word  = (uint32_t)p_cmd->type;
        word |= sat_field((uint32_t)p_cmd->arg, MSG_MAX_CMD_ARG) << 3u;
        word |= ((uint32_t)p_cmd->seq & 0x07u) << 13u;

        /* Little-endian, byte by byte (rule R3): never cast a struct over
           the buffer, because the Cortex-M0+ faults on unaligned access. */
        p_buf[0] = (uint8_t)(word & 0xFFu);
        p_buf[1] = (uint8_t)((word >> 8u) & 0xFFu);
        written  = MSG_LEN_MOTION_CMD;
    }

    return (written);
}

bool msg_unpack_motion_cmd(const uint8_t *p_buf, uint8_t buf_len,
                           msg_motion_cmd_t *p_cmd)
{
    uint32_t word;
    uint32_t type;
    bool     b_ok = false;

    if ((NULL != p_buf) && (NULL != p_cmd) && (buf_len >= MSG_LEN_MOTION_CMD))
    {
        word = (uint32_t)p_buf[0] | ((uint32_t)p_buf[1] << 8u);
        type = word & 0x07u;

        /* Code 7 is undefined: reject it rather than execute a guess. */
        if (type <= (uint32_t)MSG_CMD_LAST)
        {
            p_cmd->type = (msg_cmd_type_t)type;
            p_cmd->arg  = (uint16_t)((word >> 3u) & 0x03FFu);
            p_cmd->seq  = (uint8_t)((word >> 13u) & 0x07u);
            b_ok        = true;
        }
    }

    return (b_ok);
}

/* ---------------------------------------------------------------------
 * Public: barcode command, 1 byte
 *   bits 0..2  command
 *   bits 3..5  confidence (saturating)
 *   bits 6..7  sequence
 * --------------------------------------------------------------------- */

uint8_t msg_pack_barcode(const msg_barcode_t *p_bc,
                         uint8_t *p_buf, uint8_t buf_len)
{
    uint32_t cmd;
    uint8_t  written = 0u;

    if ((NULL != p_bc) && (NULL != p_buf) && (buf_len >= MSG_LEN_BARCODE))
    {
        cmd = (uint32_t)p_bc->cmd;

        if (!is_barcode_code(cmd))
        {
            cmd = (uint32_t)MSG_BC_INVALID;
            note_saturation(NULL);
        }

        p_buf[0] = (uint8_t)(cmd
                   | (sat_field((uint32_t)p_bc->confidence,
                                MSG_MAX_BARCODE_CONF) << 3u)
                   | (((uint32_t)p_bc->seq & 0x03u) << 6u));
        written = MSG_LEN_BARCODE;
    }

    return (written);
}

bool msg_unpack_barcode(const uint8_t *p_buf, uint8_t buf_len,
                        msg_barcode_t *p_bc)
{
    uint32_t cmd;
    bool     b_ok = false;

    if ((NULL != p_buf) && (NULL != p_bc) && (buf_len >= MSG_LEN_BARCODE))
    {
        cmd = (uint32_t)p_buf[0] & 0x07u;

        if (is_barcode_code(cmd))
        {
            p_bc->cmd        = (msg_barcode_cmd_t)cmd;
            p_bc->confidence = (uint8_t)(((uint32_t)p_buf[0] >> 3u) & 0x07u);
            p_bc->seq        = (uint8_t)(((uint32_t)p_buf[0] >> 6u) & 0x03u);
            b_ok             = true;
        }
    }

    return (b_ok);
}

/* ---------------------------------------------------------------------
 * Public: obstacle profile, 5 bytes
 *   bits  0..7   distance    (8)
 *   bits  8..13  width       (6)
 *   bits 14..20  bearing     (7, signed)
 *   bits 21..25  clear left  (5)
 *   bits 26..30  clear right (5)
 *   bits 31..32  side        (2)
 *   bit  33      saturated flag
 * --------------------------------------------------------------------- */

uint8_t msg_pack_obstacle(const msg_obstacle_t *p_obs,
                          uint8_t *p_buf, uint8_t buf_len)
{
    uint64_t word;
    uint32_t side;
    int8_t   bearing;
    uint8_t  idx;
    uint8_t  written = 0u;

    if ((NULL != p_obs) && (NULL != p_buf) && (buf_len >= MSG_LEN_OBSTACLE))
    {
        bearing = msg_sat_i8((int32_t)p_obs->bearing_deg,
                             (int8_t)MSG_LIM_OBST_BEARING, NULL);
        side    = sat_field((uint32_t)p_obs->side, (uint32_t)MSG_SIDE_RIGHT);

        word  = (uint64_t)p_obs->distance_cm;
        word |= (uint64_t)sat_field((uint32_t)p_obs->width_cm,
                                    MSG_MAX_OBST_WIDTH) << 8u;
        word |= (uint64_t)to_field((int32_t)bearing,
                                   MSG_BITS_OBST_BEARING) << 14u;
        word |= (uint64_t)sat_field((uint32_t)p_obs->clear_left_cm,
                                    MSG_MAX_OBST_CLEAR) << 21u;
        word |= (uint64_t)sat_field((uint32_t)p_obs->clear_right_cm,
                                    MSG_MAX_OBST_CLEAR) << 26u;
        word |= (uint64_t)side << 31u;
        word |= (uint64_t)(p_obs->b_saturated ? 1u : 0u) << 33u;

        for (idx = 0u; idx < MSG_LEN_OBSTACLE; idx++)
        {
            p_buf[idx] = (uint8_t)((word >> (8u * idx)) & 0xFFu);
        }

        written = MSG_LEN_OBSTACLE;
    }

    return (written);
}

bool msg_unpack_obstacle(const uint8_t *p_buf, uint8_t buf_len,
                         msg_obstacle_t *p_obs)
{
    uint64_t word = 0u;
    uint32_t side;
    uint8_t  idx;
    bool     b_ok = false;

    if ((NULL != p_buf) && (NULL != p_obs) && (buf_len >= MSG_LEN_OBSTACLE))
    {
        for (idx = 0u; idx < MSG_LEN_OBSTACLE; idx++)
        {
            word |= (uint64_t)p_buf[idx] << (8u * idx);
        }

        side = (uint32_t)((word >> 31u) & 0x03u);

        if (side <= (uint32_t)MSG_SIDE_RIGHT)
        {
            p_obs->distance_cm    = (uint8_t)(word & 0xFFu);
            p_obs->width_cm       = (uint8_t)((word >> 8u) & 0x3Fu);
            p_obs->bearing_deg    = (int8_t)sign_extend(
                                        (uint32_t)((word >> 14u) & 0x7Fu),
                                        MSG_BITS_OBST_BEARING);
            p_obs->clear_left_cm  = (uint8_t)((word >> 21u) & 0x1Fu);
            p_obs->clear_right_cm = (uint8_t)((word >> 26u) & 0x1Fu);
            p_obs->side           = (msg_side_t)side;
            p_obs->b_saturated    = (0u != ((word >> 33u) & 0x01u));
            b_ok                  = true;
        }
    }

    return (b_ok);
}

/* ---------------------------------------------------------------------
 * Public: telemetry frame, 13 bytes, schema v3.
 *
 *   byte 0       schema version
 *   byte 1       frame sequence
 *   byte 2       veh_state (5) | line_state (3)
 *   byte 3       line_error (7, signed) | line_valid (1)
 *   byte 4       speed_pct (7) | hump_sat (1)
 *   byte 5       hump_mm (7) | hump_invalid (1)
 *   byte 6       tilt_deg (7, signed) | spare (1)
 *   byte 7       obstacle distance (8)
 *   bytes 8..9   fault mask (10) | wifi_state (3) | last barcode (3)
 *   bytes 10..11 encoder delta (16)
 *   byte 12      letter (5, A=1..Z=26, none=0) | reserved (3)
 *
 * Every field is saturated before it is masked, so an out-of-range input
 * can never wrap into a plausible wrong value (rule R5).
 * --------------------------------------------------------------------- */

uint8_t msg_pack_telemetry(const msg_telemetry_t *p_tel,
                           uint8_t *p_buf, uint8_t buf_len)
{
    uint32_t word;
    int8_t   line_error;
    int8_t   tilt;
    uint8_t  written = 0u;

    if ((NULL != p_tel) && (NULL != p_buf) && (buf_len >= MSG_LEN_TELEMETRY))
    {
        line_error = msg_sat_i8((int32_t)p_tel->line_error,
                                (int8_t)MSG_LIM_LINE_ERROR, NULL);
        tilt       = msg_sat_i8((int32_t)p_tel->tilt_deg,
                                (int8_t)MSG_LIM_TILT_DEG, NULL);

        p_buf[0] = (uint8_t)MSG_SCHEMA_VERSION;
        p_buf[1] = p_tel->seq;

        p_buf[2] = (uint8_t)(sat_field((uint32_t)p_tel->veh_state, 0x1Fu)
                   | (sat_field((uint32_t)p_tel->line_state,
                                (uint32_t)MSG_LINE_STATE_LAST) << 5u));

        p_buf[3] = (uint8_t)(to_field((int32_t)line_error,
                                      MSG_BITS_LINE_ERROR)
                   | ((p_tel->b_line_valid ? 1u : 0u) << 7u));

        p_buf[4] = (uint8_t)(sat_field((uint32_t)p_tel->speed_pct,
                                       MSG_MAX_SPEED)
                   | ((p_tel->b_hump_sat ? 1u : 0u) << 7u));

        p_buf[5] = (uint8_t)(sat_field((uint32_t)p_tel->hump_mm,
                                       MSG_MAX_HUMP_MM)
                   | ((p_tel->b_hump_invalid ? 1u : 0u) << 7u));

        p_buf[6] = (uint8_t)to_field((int32_t)tilt, MSG_BITS_TILT_DEG);

        p_buf[7] = p_tel->obst_distance_cm;

        word  = (uint32_t)p_tel->fault_mask & 0x03FFu;
        word |= sat_field((uint32_t)p_tel->wifi_state,
                          MSG_WIFI_STATE_LAST) << 10u;
        word |= (is_barcode_code((uint32_t)p_tel->last_barcode)
                 ? (uint32_t)p_tel->last_barcode
                 : (uint32_t)MSG_BC_INVALID) << 13u;
        p_buf[8] = (uint8_t)(word & 0xFFu);
        p_buf[9] = (uint8_t)((word >> 8u) & 0xFFu);

        p_buf[10] = (uint8_t)((uint32_t)p_tel->encoder_delta & 0xFFu);
        p_buf[11] = (uint8_t)(((uint32_t)p_tel->encoder_delta >> 8u) & 0xFFu);

        p_buf[12] = 0U;
        if (((uint8_t)'A' <= p_tel->last_letter)
            && (p_tel->last_letter <= (uint8_t)'Z'))
        {
            p_buf[12] = (uint8_t)(p_tel->last_letter - (uint8_t)'A' + 1U);
        }
        else if (0U != p_tel->last_letter)
        {
            note_saturation(NULL);
        }
        else
        {
            /* No valid barcode yet. */
        }
        written = MSG_LEN_TELEMETRY;
    }

    return (written);
}

bool msg_unpack_telemetry(const uint8_t *p_buf, uint8_t buf_len,
                          msg_telemetry_t *p_tel)
{
    uint32_t word       = 0u;
    uint32_t line_state = 0u;
    uint32_t wifi_state = 0u;
    uint32_t barcode    = 0u;
    bool     b_ok       = false;

    if ((NULL != p_buf) && (NULL != p_tel) && (buf_len >= MSG_LEN_TELEMETRY)
        && (MSG_SCHEMA_VERSION == p_buf[0]))
    {
        word       = (uint32_t)p_buf[8] | ((uint32_t)p_buf[9] << 8u);
        line_state = ((uint32_t)p_buf[2] >> 5u) & 0x07u;
        wifi_state = (word >> 10u) & 0x07u;
        barcode    = (word >> 13u) & 0x07u;

        b_ok = ((line_state <= (uint32_t)MSG_LINE_STATE_LAST)
                && (wifi_state <= MSG_WIFI_STATE_LAST)
                && is_barcode_code(barcode)
                && (p_buf[12] <= 26U));
    }

    if (b_ok)
    {
        p_tel->schema         = p_buf[0];
        p_tel->seq            = p_buf[1];
        p_tel->veh_state      = (uint8_t)((uint32_t)p_buf[2] & 0x1Fu);
        p_tel->line_state     = (uint8_t)line_state;
        p_tel->line_error     = (int8_t)sign_extend(
                                    (uint32_t)p_buf[3] & 0x7Fu,
                                    MSG_BITS_LINE_ERROR);
        p_tel->b_line_valid   = (0u != ((uint32_t)p_buf[3] & 0x80u));
        p_tel->speed_pct      = (uint8_t)((uint32_t)p_buf[4] & 0x7Fu);
        p_tel->b_hump_sat     = (0u != ((uint32_t)p_buf[4] & 0x80u));
        p_tel->hump_mm        = (uint8_t)((uint32_t)p_buf[5] & 0x7Fu);
        p_tel->b_hump_invalid = (0u != ((uint32_t)p_buf[5] & 0x80u));
        p_tel->tilt_deg       = (int8_t)sign_extend(
                                    (uint32_t)p_buf[6] & 0x7Fu,
                                    MSG_BITS_TILT_DEG);
        p_tel->obst_distance_cm = p_buf[7];
        p_tel->fault_mask     = (uint16_t)(word & 0x03FFu);
        p_tel->wifi_state     = (uint8_t)wifi_state;
        p_tel->last_barcode   = (uint8_t)barcode;
        p_tel->last_letter = (0U == p_buf[12]) ? 0U
            : (uint8_t)((uint8_t)'A' + p_buf[12] - 1U);
        p_tel->encoder_delta  = (uint16_t)((uint32_t)p_buf[10]
                                           | ((uint32_t)p_buf[11] << 8u));
    }

    return (b_ok);
}

/* ---------------------------------------------------------------------
 * Public: single-word line reading
 * --------------------------------------------------------------------- */

uint32_t msg_pack_line(int32_t error, bool b_valid, uint32_t now_ms)
{
    int8_t clamped = msg_sat_i8(error, (int8_t)MSG_LIM_LINE_ERROR, NULL);

    return (to_field((int32_t)clamped, MSG_BITS_LINE_ERROR)
            | ((b_valid ? 1u : 0u) << 7u)
            | ((now_ms & MSG_LINE_TIME_MASK) << 8u));
}

void msg_unpack_line(uint32_t word, msg_line_t *p_line)
{
    if (NULL != p_line)
    {
        p_line->error   = (int8_t)sign_extend(word & 0x7Fu,
                                              MSG_BITS_LINE_ERROR);
        p_line->b_valid = (0u != (word & 0x80u));
        p_line->time_ms = (word >> 8u) & MSG_LINE_TIME_MASK;
    }
}

uint32_t msg_line_age_ms(uint32_t word, uint32_t now_ms)
{
    /* Modular difference in 24 bits: correct across the 4.6 h wrap. */
    return ((now_ms - (word >> 8u)) & MSG_LINE_TIME_MASK);
}

/* ---------------------------------------------------------------------
 * Public: single-byte hump result
 * --------------------------------------------------------------------- */

uint8_t msg_pack_hump(uint32_t peak_mm)
{
    uint32_t packed;

    if (peak_mm > MSG_MAX_HUMP_MM)
    {
        packed = MSG_MAX_HUMP_MM | 0x80u;   /* clamp and flag (rule R5) */
        note_saturation(NULL);
    }
    else
    {
        packed = peak_mm;
    }

    return ((uint8_t)packed);
}

uint8_t msg_hump_mm(uint8_t packed)
{
    return ((uint8_t)((uint32_t)packed & 0x7Fu));
}

bool msg_hump_saturated(uint8_t packed)
{
    return (0u != ((uint32_t)packed & 0x80u));
}

/* end of file */
