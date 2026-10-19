#include "state.h"

#include <tk/tkernel.h>

static T_CMTX telemetry_mutex_config = {
    .mtxatr = TA_TFIFO | TA_INHERIT
};

static ID telemetry_mutex;
/* C bit-fields compress the shared application snapshot in RAM. All
 * read-modify-write operations happen under telemetry_mutex. */
typedef struct {
    unsigned int mission_state : 3;
    unsigned int valid_fields : 5;
    unsigned int fault_flags : 6;
    unsigned int barcode : 4;
    unsigned int hump_peak_mm : 10;
    unsigned int speed_left : 12;
    unsigned int speed_right : 12;
    unsigned int line_error : 12;
    unsigned int encoder_left : 24;
    unsigned int encoder_right : 24;
    unsigned int obstacle_distance_mm : 12;
    unsigned int obstacle_width_mm : 12;
} buddy1_snapshot_t;

_Static_assert(sizeof(buddy1_snapshot_t) == 24U,
               "Packed telemetry snapshot must remain 24 bytes");

static buddy1_snapshot_t telemetry;
static ID events_flag;
static ID estop_flag;
static UINT start_pattern;
static UINT stop_pattern;
static UINT estop_pattern;
static volatile uint32_t start_sequence;
static volatile uint32_t stop_sequence;
static volatile uint32_t acknowledged_stop;

static int lock_telemetry(TMO timeout)
{
    return (telemetry_mutex > 0) &&
           (tk_loc_mtx(telemetry_mutex, timeout) == E_OK);
}

static int valid_signed(int32_t value, int32_t limit)
{
    return (value >= -limit) && (value <= limit);
}

static uint32_t encode_signed(int32_t value, uint32_t bits)
{
    return ((uint32_t)value) & ((1U << bits) - 1U);
}

static int32_t decode_signed(uint32_t value, uint32_t bits)
{
    uint32_t sign = 1U << (bits - 1U);

    if ((value & sign) != 0U) {
        return (int32_t)(value & (sign - 1U)) - (int32_t)sign;
    }
    return (int32_t)value;
}

int32_t buddy1_state_init(void)
{
    ID id;

    telemetry = (buddy1_snapshot_t){0};
    id = tk_cre_mtx(&telemetry_mutex_config);
    if (id <= 0) {
        return id;
    }
    telemetry_mutex = id;
    return E_OK;
}

int buddy1_set_mission(uint32_t state, uint32_t fault_flags)
{
    if ((state > BUDDY1_MISSION_MAX) ||
        ((fault_flags & ~BUDDY1_FAULT_MASK) != 0U)) {
        return 0;
    }
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.mission_state = state;
        telemetry.fault_flags = fault_flags;
        (void)tk_unl_mtx(telemetry_mutex);
        return 1;
    }
    return 0;
}

int buddy1_set_motion(int32_t left_mm_s, int32_t right_mm_s,
                      int32_t left_count, int32_t right_count)
{
    if (!valid_signed(left_mm_s, BUDDY1_SPEED_LIMIT_MM_S) ||
        !valid_signed(right_mm_s, BUDDY1_SPEED_LIMIT_MM_S) ||
        !valid_signed(left_count, BUDDY1_ENCODER_LIMIT) ||
        !valid_signed(right_count, BUDDY1_ENCODER_LIMIT)) {
        return 0;
    }
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.speed_left = encode_signed(left_mm_s, 12U);
        telemetry.speed_right = encode_signed(right_mm_s, 12U);
        telemetry.encoder_left = encode_signed(left_count, 24U);
        telemetry.encoder_right = encode_signed(right_count, 24U);
        telemetry.valid_fields |= BUDDY1_VALID_MOTION;
        (void)tk_unl_mtx(telemetry_mutex);
        return 1;
    }
    return 0;
}

int buddy1_set_line(int32_t line_error)
{
    if (!valid_signed(line_error, BUDDY1_LINE_LIMIT)) {
        return 0;
    }
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.line_error = encode_signed(line_error, 12U);
        telemetry.valid_fields |= BUDDY1_VALID_LINE;
        (void)tk_unl_mtx(telemetry_mutex);
        return 1;
    }
    return 0;
}

int buddy1_set_barcode(int32_t decoded_code)
{
    if ((decoded_code < 0) || (decoded_code > BUDDY1_BARCODE_MAX)) {
        return 0;
    }
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.barcode = decoded_code;
        telemetry.valid_fields |= BUDDY1_VALID_BARCODE;
        (void)tk_unl_mtx(telemetry_mutex);
        return 1;
    }
    return 0;
}

int buddy1_set_hump(int32_t peak_mm)
{
    if ((peak_mm < 0) || (peak_mm > BUDDY1_HUMP_MAX_MM)) {
        return 0;
    }
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.hump_peak_mm = peak_mm;
        telemetry.valid_fields |= BUDDY1_VALID_HUMP;
        (void)tk_unl_mtx(telemetry_mutex);
        return 1;
    }
    return 0;
}

int buddy1_set_obstacle(int32_t distance_mm, int32_t width_mm)
{
    if ((distance_mm < 0) || (distance_mm > BUDDY1_DISTANCE_MAX_MM) ||
        (width_mm < 0) || (width_mm > BUDDY1_WIDTH_MAX_MM)) {
        return 0;
    }
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.obstacle_distance_mm = distance_mm;
        telemetry.obstacle_width_mm = width_mm;
        telemetry.valid_fields |= BUDDY1_VALID_OBSTACLE;
        (void)tk_unl_mtx(telemetry_mutex);
        return 1;
    }
    return 0;
}

int buddy1_get_telemetry(buddy1_telemetry_t *sample)
{
    buddy1_snapshot_t snapshot;

    if ((sample == NULL) || !lock_telemetry(TMO_POL)) {
        return 0;
    }
    snapshot = telemetry;
    (void)tk_unl_mtx(telemetry_mutex);
    *sample = (buddy1_telemetry_t){0};
    sample->mission_state = (uint8_t)snapshot.mission_state;
    sample->valid_fields = (uint8_t)snapshot.valid_fields;
    sample->fault_flags = (uint8_t)snapshot.fault_flags;
    sample->speed_left_mm_s = decode_signed(snapshot.speed_left, 12U);
    sample->speed_right_mm_s = decode_signed(snapshot.speed_right, 12U);
    sample->encoder_left = decode_signed(snapshot.encoder_left, 24U);
    sample->encoder_right = decode_signed(snapshot.encoder_right, 24U);
    sample->line_error = decode_signed(snapshot.line_error, 12U);
    sample->barcode = (uint8_t)snapshot.barcode;
    sample->hump_peak_mm = (uint16_t)snapshot.hump_peak_mm;
    sample->obstacle_distance_mm = (uint16_t)snapshot.obstacle_distance_mm;
    sample->obstacle_width_mm = (uint16_t)snapshot.obstacle_width_mm;
    return 1;
}

void buddy1_attach_event_flags(int32_t events, uint32_t start_bit,
                               uint32_t stop_bit, int32_t estop,
                               uint32_t estop_bit)
{
    events_flag = events;
    estop_flag = estop;
    start_pattern = start_bit;
    stop_pattern = stop_bit;
    estop_pattern = estop_bit;
}

void buddy1_receive_command(enum buddy1_command command)
{
    if (command == BUDDY1_COMMAND_START) {
        start_sequence++;
        if ((events_flag > 0) && (start_pattern != 0U)) {
            (void)tk_set_flg(events_flag, start_pattern);
        }
    } else if (command == BUDDY1_COMMAND_STOP) {
        stop_sequence++;
        if ((estop_flag > 0) && (estop_pattern != 0U)) {
            (void)tk_set_flg(estop_flag, estop_pattern);
        }
        if ((events_flag > 0) && (stop_pattern != 0U)) {
            (void)tk_set_flg(events_flag, stop_pattern);
        }
    }
}

uint32_t buddy1_start_sequence(void)
{
    return start_sequence;
}

uint32_t buddy1_stop_sequence(void)
{
    return stop_sequence;
}

int buddy1_stop_latched(void)
{
    return stop_sequence != acknowledged_stop;
}

void buddy1_acknowledge_stop(uint32_t observed_sequence)
{
    acknowledged_stop = observed_sequence;
}
