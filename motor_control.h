#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Buddy 2 motor-control interface.
 *
 * Hardware facts taken from the supplied design review PDF:
 * - Pico W / Pico SDK, C and µT-Kernel.
 * - Left encoder A/B: GP2/GP3; right encoder A/B: GP4/GP5.
 * - Motor driver PWM/control pins: left GP8/GP9, right GP10/GP11.
 *
 * This module deliberately leaves board-specific PWM/encoder calls behind
 * callbacks, because the PDF does not specify the project's existing driver
 * API or PWM frequency. Callbacks must be connected to the actual board code.
 */

typedef enum {
    MC_CMD_STOP = 0u,
    MC_CMD_FORWARD = 1u,
    MC_CMD_REVERSE = 2u,
    MC_CMD_TURN_LEFT = 3u,
    MC_CMD_TURN_RIGHT = 4u,
    MC_CMD_PIVOT_LEFT = 5u,
    MC_CMD_PIVOT_RIGHT = 6u,
    MC_CMD_RESERVED = 7u
} mc_command_t; /* 3 logical bits: 8 possible codes */

typedef enum {
    MC_STATE_IDLE = 0u,
    MC_STATE_RUNNING = 1u,
    MC_STATE_STOPPING = 2u,
    MC_STATE_FAULT = 3u
} mc_state_t; /* 2 logical bits: 4 states */

/* Five faults use five bits in one logical fault mask. */
#define MC_FAULT_MOTOR_STALL  ((uint8_t)(1u << 0))
#define MC_FAULT_ENCODER      ((uint8_t)(1u << 1))
#define MC_FAULT_IMU          ((uint8_t)(1u << 2))
#define MC_FAULT_NO_ECHO      ((uint8_t)(1u << 3))
#define MC_FAULT_WIFI_LOSS    ((uint8_t)(1u << 4))
#define MC_FAULT_VALID_MASK   ((uint8_t)0x1Fu)

#define MC_SPEED_MAX          100u
#define MC_COMMAND_WIRE_SIZE  2u

/*
 * Compact, deterministic 2-byte command format for queues / binary MQTT:
 * byte 0: bits 0..2 command (0..7); bits 3..7 reserved (must be zero).
 * byte 1: bits 0..6 speed percentage (0..100); bit 7 flags (currently zero).
 * Do not transmit this C struct directly: use mc_encode_command() so layout
 * and byte order do not depend on compiler padding or host endianness.
 */
typedef struct {
    mc_command_t command;
    uint8_t speed_pct;
} mc_command_typed_t;

/* Board-specific adapter. Return true when the operation succeeds. */
typedef struct {
    bool (*set_motor_percent)(void *context, int8_t left_pct, int8_t right_pct);
    void (*lock)(void *context);   /* optional: short lock protecting shared data */
    void (*unlock)(void *context); /* optional; provide both or neither */
    void *context;
} mc_platform_t;

typedef struct {
    uint8_t state;       /* logical range 0..3 */
    uint8_t fault_flags; /* only MC_FAULT_VALID_MASK bits are valid */
    uint8_t target_speed;/* 0..100 percent */
    uint8_t last_command;/* logical range 0..7 */
} mc_status_t;

typedef struct {
    mc_platform_t platform;
    mc_status_t status;
} mc_controller_t;

/* Returns false for invalid inputs or incomplete platform callbacks. */
bool mc_init(mc_controller_t *mc, const mc_platform_t *platform);
bool mc_apply_command(mc_controller_t *mc, mc_command_t command, uint8_t speed_pct);

/* Emergency/safety stop: invokes the hardware adapter with both outputs zero. */
bool mc_stop(mc_controller_t *mc);

/* Set/clear one or more fault bits without disturbing unrelated fault bits. */
bool mc_set_faults(mc_controller_t *mc, uint8_t fault_mask);
bool mc_clear_faults(mc_controller_t *mc, uint8_t fault_mask);

/* Snapshot status. If task/ISR sharing is used, configure lock/unlock callbacks. */
bool mc_get_status(mc_controller_t *mc, mc_status_t *out_status);

/* Compact binary serialization for inter-task queue or MQTT payloads. */
bool mc_encode_command(mc_command_t command, uint8_t speed_pct,
                       uint8_t out[MC_COMMAND_WIRE_SIZE]);
bool mc_decode_command(const uint8_t in[MC_COMMAND_WIRE_SIZE],
                       mc_command_typed_t *out_command);

#ifdef __cplusplus
}
#endif
#endif
