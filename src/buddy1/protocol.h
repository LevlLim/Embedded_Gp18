#ifndef AAI18_BUDDY1_PROTOCOL_H
#define AAI18_BUDDY1_PROTOCOL_H

#include <stdint.h>

enum buddy1_command {
    BUDDY1_COMMAND_INVALID = 0,
    BUDDY1_COMMAND_START,
    BUDDY1_COMMAND_STOP
};

enum buddy1_valid_field {
    BUDDY1_VALID_MOTION   = (1U << 0),
    BUDDY1_VALID_LINE     = (1U << 1),
    BUDDY1_VALID_BARCODE  = (1U << 2),
    BUDDY1_VALID_HUMP     = (1U << 3),
    BUDDY1_VALID_OBSTACLE = (1U << 4)
};

typedef struct {
    uint32_t uptime_ms;
    uint32_t sequence;
    uint32_t mission_state;
    uint32_t valid_fields;
    uint32_t fault_flags;
    int32_t speed_left_mm_s;
    int32_t speed_right_mm_s;
    int32_t encoder_left;
    int32_t encoder_right;
    int32_t line_error;
    int32_t barcode;
    int32_t hump_peak_mm;
    int32_t obstacle_distance_mm;
    int32_t obstacle_width_mm;
} buddy1_telemetry_t;

enum buddy1_command buddy1_parse_command(const uint8_t *payload, uint32_t length);

/* Returns bytes written, or 0 when the buffer is too small. */
uint32_t buddy1_format_telemetry(char *buffer, uint32_t capacity,
                                 const buddy1_telemetry_t *sample);
uint32_t buddy1_format_u32(char *buffer, uint32_t capacity, uint32_t value);

#endif
