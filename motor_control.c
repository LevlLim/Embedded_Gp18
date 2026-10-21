#include "motor_control.h"

#include <stddef.h>

#define MC_COMMAND_MASK 0x07u
#define MC_BYTE0_RESERVED_MASK 0xF8u
#define MC_SPEED_MASK 0x7Fu
#define MC_BYTE1_RESERVED_MASK 0x80u

static bool mc_platform_valid(const mc_platform_t *platform)
{
    if ((platform == NULL) || (platform->set_motor_percent == NULL)) {
        return false;
    }
    /* A lock pair must be supplied together. */
    if ((platform->lock == NULL) != (platform->unlock == NULL)) {
        return false;
    }
    return true;
}

static void mc_lock(mc_controller_t *mc)
{
    if (mc->platform.lock != NULL) {
        mc->platform.lock(mc->platform.context);
    }
}

static void mc_unlock(mc_controller_t *mc)
{
    if (mc->platform.unlock != NULL) {
        mc->platform.unlock(mc->platform.context);
    }
}

bool mc_init(mc_controller_t *mc, const mc_platform_t *platform)
{
    if ((mc == NULL) || !mc_platform_valid(platform)) {
        return false;
    }

    mc->platform = *platform;
    mc->status.state = (uint8_t)MC_STATE_IDLE;
    mc->status.fault_flags = 0u;
    mc->status.target_speed = 0u;
    mc->status.last_command = (uint8_t)MC_CMD_STOP;

    /* Initialise outputs to a stopped state. */
    if (!mc->platform.set_motor_percent(mc->platform.context, 0, 0)) {
        mc->status.state = (uint8_t)MC_STATE_FAULT;
        mc->status.fault_flags |= MC_FAULT_MOTOR_STALL;
        return false;
    }
    return true;
}

bool mc_apply_command(mc_controller_t *mc, mc_command_t command, uint8_t speed_pct)
{
    int8_t left = 0;
    int8_t right = 0;
    bool ok;

    if ((mc == NULL) || (command > MC_CMD_PIVOT_RIGHT) || (speed_pct > MC_SPEED_MAX)) {
        return false;
    }

    /* Never permit normal motion while any fault is active. */
    mc_lock(mc);
    if (mc->status.fault_flags != 0u) {
        mc_unlock(mc);
        (void)mc->platform.set_motor_percent(mc->platform.context, 0, 0);
        return false;
    }
    mc_unlock(mc);

    switch (command) {
    case MC_CMD_STOP:
        left = 0; right = 0;
        break;
    case MC_CMD_FORWARD:
        left = (int8_t)speed_pct; right = (int8_t)speed_pct;
        break;
    case MC_CMD_REVERSE:
        left = (int8_t)-(int16_t)speed_pct; right = (int8_t)-(int16_t)speed_pct;
        break;
    case MC_CMD_TURN_LEFT:
        /* Differential arc: left wheel stopped, right wheel forward. */
        left = 0; right = (int8_t)speed_pct;
        break;
    case MC_CMD_TURN_RIGHT:
        left = (int8_t)speed_pct; right = 0;
        break;
    case MC_CMD_PIVOT_LEFT:
        left = (int8_t)-(int16_t)speed_pct; right = (int8_t)speed_pct;
        break;
    case MC_CMD_PIVOT_RIGHT:
        left = (int8_t)speed_pct; right = (int8_t)-(int16_t)speed_pct;
        break;
    default:
        return false;
    }

    /* Hardware adapter is expected to map signed percentages to PWM/direction. */
    ok = mc->platform.set_motor_percent(mc->platform.context, left, right);

    mc_lock(mc);
    if (ok) {
        mc->status.last_command = (uint8_t)command;
        mc->status.target_speed = (command == MC_CMD_STOP) ? 0u : speed_pct;
        mc->status.state = (command == MC_CMD_STOP) ?
                           (uint8_t)MC_STATE_IDLE : (uint8_t)MC_STATE_RUNNING;
    } else {
        mc->status.state = (uint8_t)MC_STATE_FAULT;
        mc->status.fault_flags |= MC_FAULT_MOTOR_STALL;
    }
    mc_unlock(mc);
    return ok;
}

bool mc_stop(mc_controller_t *mc)
{
    bool ok;
    if (mc == NULL) {
        return false;
    }

    /* Apply hardware stop before updating metadata to prioritise the stop. */
    ok = mc->platform.set_motor_percent(mc->platform.context, 0, 0);
    mc_lock(mc);
    mc->status.last_command = (uint8_t)MC_CMD_STOP;
    mc->status.target_speed = 0u;
    mc->status.state = ok ? (uint8_t)MC_STATE_IDLE : (uint8_t)MC_STATE_FAULT;
    if (!ok) {
        mc->status.fault_flags |= MC_FAULT_MOTOR_STALL;
    }
    mc_unlock(mc);
    return ok;
}

bool mc_set_faults(mc_controller_t *mc, uint8_t fault_mask)
{
    if ((mc == NULL) || ((fault_mask & (uint8_t)~MC_FAULT_VALID_MASK) != 0u)) {
        return false;
    }
    mc_lock(mc);
    mc->status.fault_flags = (uint8_t)(mc->status.fault_flags | fault_mask);
    if (mc->status.fault_flags != 0u) {
        mc->status.state = (uint8_t)MC_STATE_FAULT;
    }
    mc_unlock(mc);

    /* Any newly asserted fault requests an immediate stop. */
    if (fault_mask != 0u) {
        (void)mc->platform.set_motor_percent(mc->platform.context, 0, 0);
    }
    return true;
}

bool mc_clear_faults(mc_controller_t *mc, uint8_t fault_mask)
{
    if ((mc == NULL) || ((fault_mask & (uint8_t)~MC_FAULT_VALID_MASK) != 0u)) {
        return false;
    }
    mc_lock(mc);
    mc->status.fault_flags = (uint8_t)(mc->status.fault_flags & (uint8_t)~fault_mask);
    if (mc->status.fault_flags == 0u) {
        mc->status.state = (uint8_t)MC_STATE_IDLE;
    }
    mc_unlock(mc);
    return true;
}

bool mc_get_status(mc_controller_t *mc, mc_status_t *out_status)
{
    if ((mc == NULL) || (out_status == NULL)) {
        return false;
    }
    mc_lock(mc);
    *out_status = mc->status;
    mc_unlock(mc);
    return true;
}

bool mc_encode_command(mc_command_t command, uint8_t speed_pct,
                       uint8_t out[MC_COMMAND_WIRE_SIZE])
{
    if ((out == NULL) || (command > MC_CMD_PIVOT_RIGHT) || (speed_pct > MC_SPEED_MAX)) {
        return false;
    }
    out[0] = (uint8_t)command & MC_COMMAND_MASK;
    out[1] = speed_pct & MC_SPEED_MASK;
    return true;
}

bool mc_decode_command(const uint8_t in[MC_COMMAND_WIRE_SIZE],
                       mc_command_typed_t *out_command)
{
    uint8_t command_bits;
    uint8_t speed_bits;
    if ((in == NULL) || (out_command == NULL)) {
        return false;
    }
    if (((in[0] & MC_BYTE0_RESERVED_MASK) != 0u) ||
        ((in[1] & MC_BYTE1_RESERVED_MASK) != 0u)) {
        return false;
    }
    command_bits = in[0] & MC_COMMAND_MASK;
    speed_bits = in[1] & MC_SPEED_MASK;
    if ((command_bits > (uint8_t)MC_CMD_PIVOT_RIGHT) || (speed_bits > MC_SPEED_MAX)) {
        return false;
    }
    out_command->command = (mc_command_t)command_bits;
    out_command->speed_pct = speed_bits;
    return true;
}
