#include "motor_control.h"
#include <assert.h>
#include <stdio.h>

static int8_t last_left;
static int8_t last_right;
static bool adapter_ok = true;

static bool fake_set_motor(void *context, int8_t left, int8_t right)
{
    (void)context;
    last_left = left;
    last_right = right;
    return adapter_ok;
}

int main(void)
{
    mc_controller_t mc;
    mc_status_t status;
    mc_platform_t platform = { fake_set_motor, NULL, NULL, NULL };
    uint8_t wire[MC_COMMAND_WIRE_SIZE];
    mc_command_typed_t decoded;

    assert(mc_init(&mc, &platform));
    assert(last_left == 0 && last_right == 0);
    assert(mc_apply_command(&mc, MC_CMD_FORWARD, 75u));
    assert(last_left == 75 && last_right == 75);
    assert(mc_apply_command(&mc, MC_CMD_PIVOT_LEFT, 40u));
    assert(last_left == -40 && last_right == 40);

    assert(mc_set_faults(&mc, MC_FAULT_ENCODER));
    assert(last_left == 0 && last_right == 0);
    assert(!mc_apply_command(&mc, MC_CMD_FORWARD, 20u));
    assert(mc_set_faults(&mc, MC_FAULT_NO_ECHO));
    assert(mc_clear_faults(&mc, MC_FAULT_ENCODER));
    assert(mc_get_status(&mc, &status));
    assert(status.fault_flags == MC_FAULT_NO_ECHO); /* clearing one bit preserves others */
    assert(mc_clear_faults(&mc, MC_FAULT_NO_ECHO));

    assert(mc_encode_command(MC_CMD_FORWARD, 100u, wire));
    assert(wire[0] == 1u && wire[1] == 100u);
    assert(mc_decode_command(wire, &decoded));
    assert(decoded.command == MC_CMD_FORWARD && decoded.speed_pct == 100u);
    wire[1] = 101u;
    assert(!mc_decode_command(wire, &decoded));
    assert(!mc_encode_command(MC_CMD_FORWARD, 101u, wire));

    adapter_ok = false;
    assert(!mc_apply_command(&mc, MC_CMD_STOP, 0u));
    assert(mc_get_status(&mc, &status));
    assert(status.state == MC_STATE_FAULT);
    puts("motor_control tests passed");
    return 0;
}
