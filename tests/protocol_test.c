#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "protocol.h"

static uint32_t read_bits(const uint8_t *buffer, uint32_t *position,
                          uint32_t bits)
{
    uint32_t index;
    uint32_t value = 0U;

    for (index = 0U; index < bits; index++) {
        value |= ((uint32_t)((buffer[(*position + index) / 8U] >>
                             ((*position + index) % 8U)) & 1U)) << index;
    }
    *position += bits;
    return value;
}

static int32_t signed_value(uint32_t raw, uint32_t bits)
{
    uint32_t sign = 1U << (bits - 1U);
    if ((raw & sign) != 0U) {
        return (int32_t)(raw & (sign - 1U)) - (int32_t)sign;
    }
    return (int32_t)raw;
}

int main(void)
{
    buddy1_telemetry_t sample = {0};
    buddy1_command_tracker_t tracker = {0};
    uint8_t frame[BUDDY1_TELEMETRY_MAX_BYTES + 1U];
    uint8_t command[BUDDY1_COMMAND_BYTES] =
        {0x21U, 0x78U, 0x56U, 0x34U, 0x12U, 0xffU, 0xffU};
    uint32_t length;
    uint32_t position = 0U;

    assert(buddy1_accept_command(&tracker, command, sizeof(command),
                                 0x12345678U) == BUDDY1_COMMAND_START);
    assert(buddy1_accept_command(&tracker, command, sizeof(command),
                                 0x12345678U) == BUDDY1_COMMAND_INVALID);
    command[0] = 0x41U;
    command[5] = 0U;
    command[6] = 0U;
    assert(buddy1_accept_command(&tracker, command, sizeof(command),
                                 0x12345678U) == BUDDY1_COMMAND_STOP);
    assert(buddy1_accept_command(&tracker, command, sizeof(command),
                                 0x12345678U) == BUDDY1_COMMAND_INVALID);
    command[0] = 0x21U;
    command[5] = 0xffU;
    command[6] = 0xffU;
    assert(buddy1_accept_command(&tracker, command, sizeof(command),
                                 0x12345678U) == BUDDY1_COMMAND_INVALID);
    command[5] = 1U;
    command[6] = 0U;
    assert(buddy1_accept_command(&tracker, command, sizeof(command),
                                 0x12345679U) == BUDDY1_COMMAND_INVALID);
    assert(buddy1_accept_command(&tracker, command, 6U,
                                 0x12345678U) == BUDDY1_COMMAND_INVALID);
    command[0] = 0x29U;
    assert(buddy1_accept_command(&tracker, command, sizeof(command),
                                 0x12345678U) == BUDDY1_COMMAND_INVALID);
    command[0] = 0x21U;
    assert(buddy1_accept_command(&tracker, command, sizeof(command),
                                 0x12345678U) == BUDDY1_COMMAND_START);

    sample.session_id = 0x12345678U;
    sample.uptime_ms = UINT32_MAX;
    sample.sequence = UINT16_MAX;
    sample.mission_state = 7U;
    sample.valid_fields = 0x1fU;
    sample.fault_flags = 0x3fU;
    sample.speed_left_mm_s = -BUDDY1_SPEED_LIMIT_MM_S;
    sample.speed_right_mm_s = BUDDY1_SPEED_LIMIT_MM_S;
    sample.encoder_left = -BUDDY1_ENCODER_LIMIT;
    sample.encoder_right = BUDDY1_ENCODER_LIMIT;
    sample.line_error = -BUDDY1_LINE_LIMIT;
    sample.barcode = BUDDY1_BARCODE_MAX;
    sample.hump_peak_mm = BUDDY1_HUMP_MAX_MM;
    sample.obstacle_distance_mm = BUDDY1_DISTANCE_MAX_MM;
    sample.obstacle_width_mm = BUDDY1_WIDTH_MAX_MM;
    memset(frame, 0xa5, sizeof(frame));
    length = buddy1_pack_telemetry(frame, BUDDY1_TELEMETRY_MAX_BYTES,
                                   &sample);
    assert(length == BUDDY1_TELEMETRY_MAX_BYTES);
    assert(frame[BUDDY1_TELEMETRY_MAX_BYTES] == 0xa5U);
    assert(read_bits(frame, &position, 3U) == BUDDY1_WIRE_VERSION);
    assert(read_bits(frame, &position, 2U) == 1U);
    assert(read_bits(frame, &position, 3U) == 0U);
    assert(read_bits(frame, &position, 32U) == sample.session_id);
    assert(read_bits(frame, &position, 32U) == sample.uptime_ms);
    assert(read_bits(frame, &position, 16U) == sample.sequence);
    assert(read_bits(frame, &position, 3U) == sample.mission_state);
    assert(read_bits(frame, &position, 5U) == sample.valid_fields);
    assert(read_bits(frame, &position, 6U) == sample.fault_flags);
    assert(signed_value(read_bits(frame, &position, 12U), 12U) ==
           sample.speed_left_mm_s);
    assert(signed_value(read_bits(frame, &position, 12U), 12U) ==
           sample.speed_right_mm_s);
    assert(signed_value(read_bits(frame, &position, 24U), 24U) ==
           sample.encoder_left);
    assert(signed_value(read_bits(frame, &position, 24U), 24U) ==
           sample.encoder_right);
    assert(signed_value(read_bits(frame, &position, 12U), 12U) ==
           sample.line_error);
    assert(read_bits(frame, &position, 4U) == sample.barcode);
    assert(read_bits(frame, &position, 10U) == sample.hump_peak_mm);
    assert(read_bits(frame, &position, 12U) == sample.obstacle_distance_mm);
    assert(read_bits(frame, &position, 12U) == sample.obstacle_width_mm);
    assert(position == 224U);

    assert(buddy1_pack_telemetry(frame, 1U, &sample) == 0U);
    sample.speed_left_mm_s = BUDDY1_SPEED_LIMIT_MM_S + 1;
    assert(buddy1_pack_telemetry(frame, sizeof(frame), &sample) == 0U);
    sample.speed_left_mm_s = 0;
    sample.valid_fields = 0U;
    assert(buddy1_pack_telemetry(frame, sizeof(frame), &sample) == 13U);
    assert(buddy1_pack_heartbeat(frame, BUDDY1_HEARTBEAT_BYTES,
                                 0x12345678U, 1000U) ==
           BUDDY1_HEARTBEAT_BYTES);
    assert(frame[0] == 0x11U);
    assert(frame[1] == 0x78U);
    assert(frame[5] == 0xe8U);
    assert(buddy1_pack_heartbeat(frame, 8U, 1U, 2U) == 0U);
    return 0;
}
