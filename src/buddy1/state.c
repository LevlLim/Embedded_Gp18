#include "state.h"

#include <tk/tkernel.h>

static T_CMTX telemetry_mutex_config = {
    .mtxatr = TA_TFIFO | TA_INHERIT
};

static ID telemetry_mutex;
static buddy1_telemetry_t telemetry;
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

int32_t buddy1_state_init(void)
{
    ID id;

    telemetry = (buddy1_telemetry_t){0};
    id = tk_cre_mtx(&telemetry_mutex_config);
    if (id <= 0) {
        return id;
    }
    telemetry_mutex = id;
    return E_OK;
}

void buddy1_set_mission(uint32_t state, uint32_t fault_flags)
{
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.mission_state = state;
        telemetry.fault_flags = fault_flags;
        (void)tk_unl_mtx(telemetry_mutex);
    }
}

void buddy1_set_motion(int32_t left_mm_s, int32_t right_mm_s,
                       int32_t left_count, int32_t right_count)
{
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.speed_left_mm_s = left_mm_s;
        telemetry.speed_right_mm_s = right_mm_s;
        telemetry.encoder_left = left_count;
        telemetry.encoder_right = right_count;
        telemetry.valid_fields |= BUDDY1_VALID_MOTION;
        (void)tk_unl_mtx(telemetry_mutex);
    }
}

void buddy1_set_line(int32_t line_error)
{
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.line_error = line_error;
        telemetry.valid_fields |= BUDDY1_VALID_LINE;
        (void)tk_unl_mtx(telemetry_mutex);
    }
}

void buddy1_set_barcode(int32_t decoded_code)
{
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.barcode = decoded_code;
        telemetry.valid_fields |= BUDDY1_VALID_BARCODE;
        (void)tk_unl_mtx(telemetry_mutex);
    }
}

void buddy1_set_hump(int32_t peak_mm)
{
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.hump_peak_mm = peak_mm;
        telemetry.valid_fields |= BUDDY1_VALID_HUMP;
        (void)tk_unl_mtx(telemetry_mutex);
    }
}

void buddy1_set_obstacle(int32_t distance_mm, int32_t width_mm)
{
    if (lock_telemetry(TMO_FEVR)) {
        telemetry.obstacle_distance_mm = distance_mm;
        telemetry.obstacle_width_mm = width_mm;
        telemetry.valid_fields |= BUDDY1_VALID_OBSTACLE;
        (void)tk_unl_mtx(telemetry_mutex);
    }
}

int buddy1_get_telemetry(buddy1_telemetry_t *sample)
{
    if ((sample == NULL) || !lock_telemetry(TMO_POL)) {
        return 0;
    }
    *sample = telemetry;
    (void)tk_unl_mtx(telemetry_mutex);
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
