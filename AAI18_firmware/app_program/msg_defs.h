/*
 * msg_defs.h
 *
 * Group AAI18 -- shared minimum-bit data representation.
 *
 * This file is the single authoritative definition of every packed field
 * that crosses a subsystem boundary.  It implements Change Requirement 1
 * and requirement NFR12 of Design Review V2, section 7.
 *
 * OWNERSHIP: Buddy 1 owns this file.  Any change must be reviewed by all
 * five buddies before merge, because every subsystem decodes what another
 * subsystem encodes.  Do not copy a layout into your own module.
 *
 * DESIGN RULES IMPLEMENTED HERE (Design Review V2 section 7.2):
 *   R2  Explicit masks and shifts, never C bit-fields.  ARMv6-M has no
 *       bit-field insert/extract instruction and the C standard leaves
 *       bit-field layout implementation-defined.
 *   R3  Byte-by-byte serialisation.  A packed struct is never pointer-cast
 *       over a buffer, because the Cortex-M0+ does not support unaligned
 *       word access and such a read faults.
 *   R4  Little-endian byte order; every MQTT frame carries a schema version.
 *   R5  Every field saturates; none wraps.  A wrapped distance would give a
 *       confident wrong bypass decision.  Every pack function clamps, and
 *       every unpack function REJECTS a code outside its enumeration.
 *   R7  Compile-time range assertions accompany each width, and every
 *       clamp raises a sticky flag (msg_saturation_seen) for bring-up.
 *
 * SINGLE-WORD SHARED RECORDS
 * --------------------------
 * The line reading and the hump result are read by a task on the other
 * core.  Interrupt masking (DI) is core-local and protects nothing there,
 * so each is packed into ONE naturally aligned word, which the Cortex-M0+
 * reads and writes in a single access: no tearing, no lock.
 */

#ifndef MSG_DEFS_H
#define MSG_DEFS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>     /* NULL */

/* ---------------------------------------------------------------------
 * Schema version.  Increment on ANY layout change below.  A subscriber
 * decoding an older layout then fails loudly instead of misreading fields.
 *   v1  initial layout
 *   v2  REVERSE command; hump_invalid and last barcode in the telemetry
 *       frame; saturation on every telemetry field
 *   v3  append a 5-bit retained barcode letter (A=1..Z=26, none=0)
 * --------------------------------------------------------------------- */
#define MSG_SCHEMA_VERSION      (3u)

/* ---------------------------------------------------------------------
 * Compile-time range check (rule R7).  Fails the build if a declared
 * range does not fit its declared width.
 * --------------------------------------------------------------------- */
#define MSG_STATIC_ASSERT(cond, tag) \
    typedef char msg_static_assert_##tag[(cond) ? 1 : -1]

/* ---------------------------------------------------------------------
 * Field widths.  Design Review V2 table 7.1.
 * Values marked PROVISIONAL are derived from an estimated range and must
 * be re-derived once the underlying quantity is measured (assumption A9).
 * --------------------------------------------------------------------- */
#define MSG_BITS_VEH_STATE      (5u)   /* 3-bit parent + 2-bit substate   */
#define MSG_BITS_CMD_TYPE       (3u)   /* 7 commands                      */
#define MSG_BITS_CMD_ARG        (10u)  /* 0..1023 mm or 0..359 deg        */
#define MSG_BITS_CMD_SEQ        (3u)   /* modulo duplicate detection      */
#define MSG_BITS_SPEED          (7u)   /* 0..100 %                        */
#define MSG_BITS_LINE_ERROR     (7u)   /* -63..+63, signed                */
#define MSG_BITS_LINE_STATE     (3u)   /* 5 states                        */
#define MSG_BITS_LINE_TIME      (24u)  /* ms, wraps every 4.6 h           */
#define MSG_BITS_BARCODE_CMD    (3u)   /* 4 commands + none + invalid     */
#define MSG_BITS_BARCODE_CONF   (3u)   /* 0..7 confidence bucket          */
#define MSG_BITS_BARCODE_SEQ    (2u)   /* modulo duplicate detection      */
#define MSG_BITS_HUMP_MM        (7u)   /* 0..127 mm  PROVISIONAL          */
#define MSG_BITS_TILT_DEG       (7u)   /* -63..+63 deg, signed            */
#define MSG_BITS_OBST_DIST      (8u)   /* 0..255 cm  PROVISIONAL          */
#define MSG_BITS_OBST_WIDTH     (6u)   /* 0..63 cm   PROVISIONAL          */
#define MSG_BITS_OBST_BEARING   (7u)   /* -63..+63 deg, signed            */
#define MSG_BITS_OBST_CLEAR     (5u)   /* 0..31 cm each  PROVISIONAL      */
#define MSG_BITS_OBST_SIDE      (2u)   /* left / right / none             */
#define MSG_BITS_FAULT          (10u)  /* 10 defined faults, see fault.h  */
#define MSG_BITS_WIFI_STATE     (3u)   /* 5 states                        */

/* Saturation limits derived from the widths above. */
#define MSG_MAX_CMD_ARG         ((1u << MSG_BITS_CMD_ARG) - 1u)     /* 1023 */
#define MSG_MAX_SPEED           (100u)
#define MSG_MAX_HUMP_MM         ((1u << MSG_BITS_HUMP_MM) - 1u)     /*  127 */
#define MSG_MAX_OBST_DIST       ((1u << MSG_BITS_OBST_DIST) - 1u)   /*  255 */
#define MSG_MAX_OBST_WIDTH      ((1u << MSG_BITS_OBST_WIDTH) - 1u)  /*   63 */
#define MSG_MAX_OBST_CLEAR      ((1u << MSG_BITS_OBST_CLEAR) - 1u)  /*   31 */
#define MSG_MAX_BARCODE_CONF    ((1u << MSG_BITS_BARCODE_CONF) - 1u) /*   7 */
#define MSG_MAX_ENCODER_DELTA   (0xFFFFu)
#define MSG_LIM_LINE_ERROR      (63)
#define MSG_LIM_TILT_DEG        (63)
#define MSG_LIM_OBST_BEARING    (63)
#define MSG_LINE_TIME_MASK      ((1u << MSG_BITS_LINE_TIME) - 1u)

MSG_STATIC_ASSERT(MSG_MAX_SPEED < (1u << MSG_BITS_SPEED), speed_fits);
MSG_STATIC_ASSERT(MSG_LIM_LINE_ERROR < (1 << (MSG_BITS_LINE_ERROR - 1u)),
                  line_error_fits);
MSG_STATIC_ASSERT(MSG_LIM_TILT_DEG < (1 << (MSG_BITS_TILT_DEG - 1u)),
                  tilt_fits);
MSG_STATIC_ASSERT(MSG_LIM_OBST_BEARING < (1 << (MSG_BITS_OBST_BEARING - 1u)),
                  bearing_fits);
MSG_STATIC_ASSERT((MSG_BITS_FAULT + MSG_BITS_WIFI_STATE + MSG_BITS_BARCODE_CMD)
                  <= 16u, fault_word_fits);
MSG_STATIC_ASSERT((MSG_BITS_LINE_ERROR + 1u + MSG_BITS_LINE_TIME) <= 32u,
                  line_word_fits);

/* ---------------------------------------------------------------------
 * Packed message sizes, in bytes.  These are the numbers reported in the
 * Week 13 message-size comparison (Design Review V2 section 7.5).  The
 * "before" figures are NOT written here: testing/host prints sizeof() of
 * the unpacked structs, which is the honest baseline for each compiler.
 * --------------------------------------------------------------------- */
#define MSG_LEN_MOTION_CMD      (2u)
#define MSG_LEN_BARCODE         (1u)
#define MSG_LEN_OBSTACLE        (5u)
#define MSG_LEN_TELEMETRY       (13u)
#define MSG_LEN_COMMAND         (2u)   /* inbound MQTT: seq, command      */

/* ---------------------------------------------------------------------
 * Enumerations.  Barr-C 5.1: lowercase, typedef'd, '_t' suffix.
 * --------------------------------------------------------------------- */
typedef enum
{
    MSG_CMD_STOP      = 0,
    MSG_CMD_LINE_PID  = 1,
    MSG_CMD_FORWARD   = 2,
    MSG_CMD_TURN_L    = 3,
    MSG_CMD_TURN_R    = 4,
    MSG_CMD_U_TURN    = 5,
    MSG_CMD_REVERSE   = 6      /* new in schema v2: back off (FR8 retry) */
} msg_cmd_type_t;

#define MSG_CMD_LAST            (MSG_CMD_REVERSE)

typedef enum
{
    MSG_BC_NONE       = 0,
    MSG_BC_STRAIGHT   = 1,
    MSG_BC_LEFT       = 2,
    MSG_BC_RIGHT      = 3,
    MSG_BC_U_TURN     = 4,
    MSG_BC_INVALID    = 7
} msg_barcode_cmd_t;

typedef enum
{
    MSG_SIDE_NONE     = 0,
    MSG_SIDE_LEFT     = 1,
    MSG_SIDE_RIGHT    = 2
} msg_side_t;

typedef enum
{
    MSG_LINE_IDLE     = 0,
    MSG_LINE_CAL      = 1,
    MSG_LINE_TRACK    = 2,
    MSG_LINE_LOST     = 3,
    MSG_LINE_SEARCH   = 4
} msg_line_state_t;

#define MSG_LINE_STATE_LAST     (MSG_LINE_SEARCH)
#define MSG_WIFI_STATE_LAST     (4u)

/* ---------------------------------------------------------------------
 * Unpacked working structures.  These live in RAM inside one task and are
 * NOT what travels between tasks -- the byte arrays below are.
 * --------------------------------------------------------------------- */
typedef struct
{
    msg_cmd_type_t type;
    uint16_t       arg;        /* mm or degrees, meaning set by type */
    uint8_t        seq;        /* 0..7, wraps */
} msg_motion_cmd_t;

typedef struct
{
    msg_barcode_cmd_t cmd;
    uint8_t           confidence;   /* 0..7 */
    uint8_t           seq;          /* 0..3, wraps */
} msg_barcode_t;

typedef struct
{
    uint8_t    distance_cm;
    uint8_t    width_cm;
    int8_t     bearing_deg;
    uint8_t    clear_left_cm;
    uint8_t    clear_right_cm;
    msg_side_t side;
    bool       b_saturated;    /* true if any field above was clamped */
} msg_obstacle_t;

typedef struct
{
    uint8_t  schema;
    uint8_t  seq;
    uint8_t  veh_state;
    uint8_t  line_state;
    int8_t   line_error;
    bool     b_line_valid;
    uint8_t  speed_pct;
    uint8_t  hump_mm;
    bool     b_hump_sat;
    bool     b_hump_invalid;
    int8_t   tilt_deg;
    uint8_t  obst_distance_cm;
    uint16_t fault_mask;
    uint8_t  wifi_state;
    uint8_t  last_barcode;     /* msg_barcode_cmd_t of the last decode */
    uint16_t encoder_delta;
    uint8_t  last_letter;      /* ASCII A..Z, 0 = no valid decode */
} msg_telemetry_t;

/* Line reading, unpacked form of the single-word record. */
typedef struct
{
    int8_t   error;            /* -63..+63, negative = line to the left */
    bool     b_valid;
    uint32_t time_ms;          /* low 24 bits of app_now_ms()           */
} msg_line_t;

/* ---------------------------------------------------------------------
 * Saturating encoders (rule R5).  Each returns the clamped value and sets
 * *pb_saturated true if clamping occurred, so a clipped reading is never
 * mistaken for a real measurement.  Any clamp also raises a sticky flag.
 * --------------------------------------------------------------------- */
uint16_t msg_sat_u16(uint32_t value, uint16_t limit, bool *pb_saturated);
int8_t   msg_sat_i8(int32_t value, int8_t limit, bool *pb_saturated);

/* True once any field anywhere has been clamped since boot (rule R7
   bring-up check; the target over ten full runs is false, per A9).  A
   sticky flag rather than a counter: setting it is a single store, so
   concurrent callers on either core cannot corrupt it. */
bool msg_saturation_seen(void);

/* ---------------------------------------------------------------------
 * Pack / unpack.  Every function writes or reads a plain byte array.
 * Pack functions return the number of bytes written.
 * Unpack functions return true only if the buffer was long enough, every
 * enumerated field holds a defined code and, for the telemetry frame, the
 * schema version matched.  A false return leaves *p_out unspecified.
 * --------------------------------------------------------------------- */
uint8_t msg_pack_motion_cmd(const msg_motion_cmd_t *p_cmd,
                            uint8_t *p_buf, uint8_t buf_len);
bool    msg_unpack_motion_cmd(const uint8_t *p_buf, uint8_t buf_len,
                              msg_motion_cmd_t *p_cmd);

uint8_t msg_pack_barcode(const msg_barcode_t *p_bc,
                         uint8_t *p_buf, uint8_t buf_len);
bool    msg_unpack_barcode(const uint8_t *p_buf, uint8_t buf_len,
                           msg_barcode_t *p_bc);

uint8_t msg_pack_obstacle(const msg_obstacle_t *p_obs,
                          uint8_t *p_buf, uint8_t buf_len);
bool    msg_unpack_obstacle(const uint8_t *p_buf, uint8_t buf_len,
                            msg_obstacle_t *p_obs);

uint8_t msg_pack_telemetry(const msg_telemetry_t *p_tel,
                           uint8_t *p_buf, uint8_t buf_len);
bool    msg_unpack_telemetry(const uint8_t *p_buf, uint8_t buf_len,
                             msg_telemetry_t *p_tel);

/* Line reading <-> one 32-bit word:
     bits 0..6   error (signed)   bit 7  valid   bits 8..31  time_ms (24) */
uint32_t msg_pack_line(int32_t error, bool b_valid, uint32_t now_ms);
void     msg_unpack_line(uint32_t word, msg_line_t *p_line);

/* Age of a packed line reading in ms, correct across the 24-bit wrap. */
uint32_t msg_line_age_ms(uint32_t word, uint32_t now_ms);

/* Hump result <-> one byte:  bits 0..6 peak mm (saturated), bit 7 SAT. */
uint8_t msg_pack_hump(uint32_t peak_mm);
uint8_t msg_hump_mm(uint8_t packed);
bool    msg_hump_saturated(uint8_t packed);

/* Vehicle state is carried as a 3-bit parent plus a 2-bit substate, because
   the composite RUNNING state does not fit in 3 bits if flattened. */
uint8_t msg_encode_veh_state(uint8_t parent, uint8_t substate);
void    msg_decode_veh_state(uint8_t encoded, uint8_t *p_parent,
                             uint8_t *p_substate);

#endif /* MSG_DEFS_H */

/* end of file */
