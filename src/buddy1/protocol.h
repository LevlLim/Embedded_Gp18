#ifndef AAI18_BUDDY1_PROTOCOL_H
#define AAI18_BUDDY1_PROTOCOL_H

#include <stdint.h>

/* Wire version 1: fields are packed least-significant bit first. */
#define BUDDY1_WIRE_VERSION          1U
#define BUDDY1_COMMAND_BYTES         7U
#define BUDDY1_HEARTBEAT_BYTES       9U
#define BUDDY1_TELEMETRY_MAX_BYTES  28U

/* Provisional application ranges. Recheck against track measurements. */
#define BUDDY1_SPEED_LIMIT_MM_S       2047
#define BUDDY1_ENCODER_LIMIT       8388607
#define BUDDY1_LINE_LIMIT             2047
#define BUDDY1_BARCODE_MAX              15
#define BUDDY1_HUMP_MAX_MM            1023
#define BUDDY1_DISTANCE_MAX_MM        4095
#define BUDDY1_WIDTH_MAX_MM           4095
#define BUDDY1_MISSION_MAX               7U
#define BUDDY1_FAULT_MASK             0x3fU

enum buddy1_command {
    BUDDY1_COMMAND_INVALID = 0,
    BUDDY1_COMMAND_START = 1,
    BUDDY1_COMMAND_STOP = 2
};

enum buddy1_valid_field {
    BUDDY1_VALID_MOTION   = (1U << 0),
    BUDDY1_VALID_LINE     = (1U << 1),
    BUDDY1_VALID_BARCODE  = (1U << 2),
    BUDDY1_VALID_HUMP     = (1U << 3),
    BUDDY1_VALID_OBSTACLE = (1U << 4)
};

/* One bit per currently defined fault. Expand the wire version if needed. */
enum buddy1_fault_field {
    BUDDY1_FAULT_MOTOR_STALL = (1U << 0),
    BUDDY1_FAULT_ENCODER     = (1U << 1),
    BUDDY1_FAULT_IMU         = (1U << 2),
    BUDDY1_FAULT_NO_ECHO     = (1U << 3),
    BUDDY1_FAULT_WIFI       = (1U << 4),
    BUDDY1_FAULT_LINE_LOSS  = (1U << 5)
};

/* Logical values at the module boundary; the shared snapshot is packed. */
typedef struct {
    uint32_t uptime_ms;
    uint32_t session_id;
    uint16_t sequence;
    uint8_t mission_state;
    uint8_t valid_fields;
    uint8_t fault_flags;
    int32_t speed_left_mm_s;
    int32_t speed_right_mm_s;
    int32_t encoder_left;
    int32_t encoder_right;
    int32_t line_error;
    uint8_t barcode;
    uint16_t hump_peak_mm;
    uint16_t obstacle_distance_mm;
    uint16_t obstacle_width_mm;
} buddy1_telemetry_t;

typedef struct {
    uint16_t last_sequence;
    uint8_t has_sequence;
} buddy1_command_tracker_t;

/* Zero means malformed, out of range, or insufficient output capacity. */
uint32_t buddy1_pack_telemetry(uint8_t *buffer, uint32_t capacity,
                               const buddy1_telemetry_t *sample);
uint32_t buddy1_pack_heartbeat(uint8_t *buffer, uint32_t capacity,
                               uint32_t session_id, uint32_t uptime_ms);

/* START/STOP are accepted once per increasing sequence, for this boot only. */
enum buddy1_command buddy1_accept_command(buddy1_command_tracker_t *tracker,
                                           const uint8_t *payload,
                                           uint32_t length,
                                           uint32_t session_id);

#endif
