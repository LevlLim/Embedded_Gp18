#include <assert.h>
#include <stdint.h>

#include "state.h"
#include <tk/tkernel.h>

static UINT seen_event;
static UINT seen_estop;
static int lock_available = 1;

ID tk_cre_mtx(const T_CMTX *config)
{
    assert((config->mtxatr & TA_INHERIT) != 0U);
    return 1;
}

int32_t tk_loc_mtx(ID mutex, TMO timeout)
{
    (void)mutex;
    (void)timeout;
    return lock_available ? E_OK : -1;
}

int32_t tk_unl_mtx(ID mutex)
{
    (void)mutex;
    return E_OK;
}

int32_t tk_set_flg(ID flag, UINT pattern)
{
    if (flag == 2) {
        seen_event |= pattern;
    } else if (flag == 3) {
        seen_estop |= pattern;
    }
    return E_OK;
}

int main(void)
{
    buddy1_telemetry_t sample;

    assert(buddy1_state_init() == E_OK);
    assert(buddy1_set_mission(7U, BUDDY1_FAULT_IMU) == 1);
    assert(buddy1_set_mission(8U, 0U) == 0);
    assert(buddy1_set_mission(1U, 0x80U) == 0);
    assert(buddy1_set_motion(-2047, 2047, -8388607, 8388607) == 1);
    assert(buddy1_set_motion(-2048, 0, 0, 0) == 0);
    assert(buddy1_set_line(-2047) == 1);
    assert(buddy1_set_barcode(15) == 1);
    assert(buddy1_set_barcode(-1) == 0);
    assert(buddy1_set_hump(1023) == 1);
    assert(buddy1_set_obstacle(4095, 4095) == 1);
    assert(buddy1_set_obstacle(4096, 0) == 0);
    assert(buddy1_get_telemetry(&sample) == 1);
    assert(sample.mission_state == 7U);
    assert(sample.fault_flags == BUDDY1_FAULT_IMU);
    assert(sample.valid_fields == 0x1fU);
    assert(sample.speed_left_mm_s == -2047);
    assert(sample.speed_right_mm_s == 2047);
    assert(sample.encoder_left == -8388607);
    assert(sample.encoder_right == 8388607);
    assert(sample.line_error == -2047);
    assert(sample.barcode == 15U);
    assert(sample.hump_peak_mm == 1023U);
    assert(sample.obstacle_distance_mm == 4095U);
    assert(sample.obstacle_width_mm == 4095U);

    lock_available = 0;
    assert(buddy1_get_telemetry(&sample) == 0);
    lock_available = 1;
    buddy1_attach_event_flags(2, 1U, 2U, 3, 4U);
    buddy1_receive_command(BUDDY1_COMMAND_START);
    buddy1_receive_command(BUDDY1_COMMAND_STOP);
    assert(buddy1_start_sequence() == 1U);
    assert(buddy1_stop_latched());
    assert(seen_event == 3U);
    assert(seen_estop == 4U);
    buddy1_acknowledge_stop(buddy1_stop_sequence());
    assert(!buddy1_stop_latched());
    return 0;
}
