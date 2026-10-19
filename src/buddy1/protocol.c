#include "protocol.h"

#include <stddef.h>
#include <string.h>

#define BUDDY1_WIRE_TELEMETRY  1U
#define BUDDY1_WIRE_HEARTBEAT  2U

typedef struct {
    uint8_t *buffer;
    uint32_t capacity_bits;
    uint32_t position;
    int valid;
} bit_writer_t;

static void write_bits(bit_writer_t *writer, uint32_t value, uint32_t bits)
{
    uint32_t index;

    if (!writer->valid || (bits > writer->capacity_bits - writer->position)) {
        writer->valid = 0;
        return;
    }
    for (index = 0U; index < bits; index++) {
        if ((value & (1U << index)) != 0U) {
            writer->buffer[(writer->position + index) / 8U] |=
                (uint8_t)(1U << ((writer->position + index) % 8U));
        }
    }
    writer->position += bits;
}

static uint32_t signed_bits(int32_t value, uint32_t bits)
{
    return ((uint32_t)value) & ((1U << bits) - 1U);
}

static int in_signed_range(int32_t value, int32_t limit)
{
    return (value >= -limit) && (value <= limit);
}

static uint32_t read_u32_le(const uint8_t *input)
{
    return ((uint32_t)input[0]) | ((uint32_t)input[1] << 8) |
           ((uint32_t)input[2] << 16) | ((uint32_t)input[3] << 24);
}

uint32_t buddy1_pack_telemetry(uint8_t *buffer, uint32_t capacity,
                               const buddy1_telemetry_t *sample)
{
    bit_writer_t writer;

    if ((buffer == NULL) || (sample == NULL) ||
        (capacity == 0U) || (capacity > UINT32_MAX / 8U)) {
        return 0U;
    }
    if ((sample->mission_state > BUDDY1_MISSION_MAX) ||
        ((sample->valid_fields & ~0x1fU) != 0U) ||
        ((sample->fault_flags & ~BUDDY1_FAULT_MASK) != 0U)) {
        return 0U;
    }
    if (((sample->valid_fields & BUDDY1_VALID_MOTION) != 0U) &&
        (!in_signed_range(sample->speed_left_mm_s, BUDDY1_SPEED_LIMIT_MM_S) ||
         !in_signed_range(sample->speed_right_mm_s, BUDDY1_SPEED_LIMIT_MM_S) ||
         !in_signed_range(sample->encoder_left, BUDDY1_ENCODER_LIMIT) ||
         !in_signed_range(sample->encoder_right, BUDDY1_ENCODER_LIMIT))) {
        return 0U;
    }
    if (((sample->valid_fields & BUDDY1_VALID_LINE) != 0U) &&
        !in_signed_range(sample->line_error, BUDDY1_LINE_LIMIT)) {
        return 0U;
    }
    if (((sample->valid_fields & BUDDY1_VALID_BARCODE) != 0U) &&
        (sample->barcode > BUDDY1_BARCODE_MAX)) {
        return 0U;
    }
    if (((sample->valid_fields & BUDDY1_VALID_HUMP) != 0U) &&
        (sample->hump_peak_mm > BUDDY1_HUMP_MAX_MM)) {
        return 0U;
    }
    if (((sample->valid_fields & BUDDY1_VALID_OBSTACLE) != 0U) &&
        ((sample->obstacle_distance_mm > BUDDY1_DISTANCE_MAX_MM) ||
         (sample->obstacle_width_mm > BUDDY1_WIDTH_MAX_MM))) {
        return 0U;
    }

    memset(buffer, 0, capacity);
    writer = (bit_writer_t){buffer, capacity * 8U, 0U, 1};
    write_bits(&writer, BUDDY1_WIRE_VERSION, 3U);
    write_bits(&writer, BUDDY1_WIRE_TELEMETRY, 2U);
    write_bits(&writer, 0U, 3U);
    write_bits(&writer, sample->session_id, 32U);
    write_bits(&writer, sample->uptime_ms, 32U);
    write_bits(&writer, sample->sequence, 16U);
    write_bits(&writer, sample->mission_state, 3U);
    write_bits(&writer, sample->valid_fields, 5U);
    write_bits(&writer, sample->fault_flags, 6U);
    if ((sample->valid_fields & BUDDY1_VALID_MOTION) != 0U) {
        write_bits(&writer, signed_bits(sample->speed_left_mm_s, 12U), 12U);
        write_bits(&writer, signed_bits(sample->speed_right_mm_s, 12U), 12U);
        write_bits(&writer, signed_bits(sample->encoder_left, 24U), 24U);
        write_bits(&writer, signed_bits(sample->encoder_right, 24U), 24U);
    }
    if ((sample->valid_fields & BUDDY1_VALID_LINE) != 0U) {
        write_bits(&writer, signed_bits(sample->line_error, 12U), 12U);
    }
    if ((sample->valid_fields & BUDDY1_VALID_BARCODE) != 0U) {
        write_bits(&writer, sample->barcode, 4U);
    }
    if ((sample->valid_fields & BUDDY1_VALID_HUMP) != 0U) {
        write_bits(&writer, sample->hump_peak_mm, 10U);
    }
    if ((sample->valid_fields & BUDDY1_VALID_OBSTACLE) != 0U) {
        write_bits(&writer, sample->obstacle_distance_mm, 12U);
        write_bits(&writer, sample->obstacle_width_mm, 12U);
    }
    return writer.valid ? ((writer.position + 7U) / 8U) : 0U;
}

uint32_t buddy1_pack_heartbeat(uint8_t *buffer, uint32_t capacity,
                               uint32_t session_id, uint32_t uptime_ms)
{
    bit_writer_t writer;

    if ((buffer == NULL) || (capacity < BUDDY1_HEARTBEAT_BYTES)) {
        return 0U;
    }
    memset(buffer, 0, BUDDY1_HEARTBEAT_BYTES);
    writer = (bit_writer_t){buffer, BUDDY1_HEARTBEAT_BYTES * 8U, 0U, 1};
    write_bits(&writer, BUDDY1_WIRE_VERSION, 3U);
    write_bits(&writer, BUDDY1_WIRE_HEARTBEAT, 2U);
    write_bits(&writer, 0U, 3U);
    write_bits(&writer, session_id, 32U);
    write_bits(&writer, uptime_ms, 32U);
    return BUDDY1_HEARTBEAT_BYTES;
}

enum buddy1_command buddy1_accept_command(buddy1_command_tracker_t *tracker,
                                           const uint8_t *payload,
                                           uint32_t length,
                                           uint32_t session_id)
{
    uint16_t sequence;
    uint16_t delta;
    uint8_t action;

    if ((tracker == NULL) || (payload == NULL) ||
        (length != BUDDY1_COMMAND_BYTES) ||
        ((payload[0] & 0x9fU) != 0x01U) ||
        (read_u32_le(&payload[1]) != session_id)) {
        return BUDDY1_COMMAND_INVALID;
    }
    action = (payload[0] >> 5) & 0x03U;
    if ((action != BUDDY1_COMMAND_START) &&
        (action != BUDDY1_COMMAND_STOP)) {
        return BUDDY1_COMMAND_INVALID;
    }
    sequence = (uint16_t)((uint16_t)payload[5] |
                          ((uint16_t)payload[6] << 8));
    delta = (uint16_t)(sequence - tracker->last_sequence);
    if (tracker->has_sequence && ((delta == 0U) || (delta >= 0x8000U))) {
        return BUDDY1_COMMAND_INVALID;
    }
    tracker->last_sequence = sequence;
    tracker->has_sequence = 1U;
    return (enum buddy1_command)action;
}
