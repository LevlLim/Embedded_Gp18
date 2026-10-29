/** Verify fixed-profile boot/reset and the real controller's START gate. */
#include <stdio.h>
#include <setjmp.h>
#include "../../subsystem5/vehicle_controller.c"

static unsigned int g_checks = 0U;
static unsigned int g_failed = 0U;
static unsigned int g_prepares = 0U;
static unsigned int g_sweeps = 0U;
static unsigned int g_commands = 0U;
static unsigned int g_delays = 0U;
static bool g_b_active = false;
static ER g_prepare_result = E_OK;
static uint16_t g_faults = 0U;
static msg_motion_cmd_t g_command = {0};
static jmp_buf g_wait_exit;

static void check(bool condition)
{
    g_checks++;
    if (!condition)
    {
        g_failed++;
        printf("Controller startup failure at check %u\n", g_checks);
    }
}

ER s3_prepare(void) { g_prepares++; return (g_prepare_result); }
ER s3_calibrate(void) { g_sweeps++; return (E_IO); }
void s3_set_active(bool b_on) { g_b_active = b_on; }
bool s3_read_line_word(uint32_t * p_word) { (void)p_word; return (false); }
msg_line_state_t s3_get_line_state(void) { return (MSG_LINE_TRACK); }
bool s3_take_barcode(msg_barcode_t * p_out) { (void)p_out; return (false); }
void s4_set_active(bool b_on) { (void)b_on; }
void s4_request_reset(void) { }
void s2_set_stop_on_line(bool b_on) { (void)b_on; }
ER s2_send_command(msg_motion_cmd_t const * p_command)
{
    g_commands++;
    g_command = *p_command;
    return (E_OK);
}
void s5_obstacle_guard(bool b_on) { (void)b_on; }
void s5_obstacle_request(s5_request_t request) { (void)request; }
bool s5_take_obstacle(msg_obstacle_t * p_out) { (void)p_out; return (false); }
int32_t s5_lateral_cm(uint32_t distance, int32_t angle)
{
    (void)distance;
    (void)angle;
    return (0);
}
void fault_set(uint16_t bits) { g_faults |= bits; }
void fault_clear(uint16_t bits) { g_faults &= (uint16_t)~bits; }
uint16_t fault_get(void) { return (g_faults); }
uint32_t app_now_ms(void) { return (100U); }
ER tk_dly_tsk(INT delay) { (void)delay; g_delays++; return (E_OK); }
ID tk_cre_flg(T_CFLG * p_config) { (void)p_config; return (1); }
ER tk_set_flg(ID id, UINT bits) { (void)id; (void)bits; return (E_OK); }
ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT * p_bits, TMO timeout)
{
    (void)id;
    (void)bits;
    (void)mode;
    (void)p_bits;
    (void)timeout;
    longjmp(g_wait_exit, 1);
}
UINT gpio_get_val(UINT pin) { (void)pin; return (1U); }
INT gpio_set_pin(UINT pin, UINT mode) { (void)pin; (void)mode; return (0); }
INT tm_sprintf(UB * p_out, UB const * p_format, ...)
{
    (void)p_format;
    p_out[0] = (UB)0;
    return (0);
}
void app_log_line(UB const * p_text) { (void)p_text; }

static void boot_until_ready(void)
{
    set_mission(S5_PARENT_INIT, S5_SUB_LINE_FOLLOW);
    if (0 == setjmp(g_wait_exit))
    {
        controller_task(0, NULL);
    }
}

int main(void)
{
    boot_until_ready();
    check(S5_PARENT_READY == get_parent());
    check(1U == g_prepares && 0U == g_sweeps && 0U == g_delays);
    check(!g_b_active && 0U == g_commands);
    handle_events(S5_EVT_REMOTE_START);
    check(S5_PARENT_RUNNING == get_parent() && g_b_active);
    check(1U == g_commands && MSG_CMD_LINE_PID == g_command.type);

    fault_set(FAULT_REMOTE_STOP);
    handle_faults(g_faults);
    check(S5_PARENT_STOPPED == get_parent() && !g_b_active);
    handle_events(S5_EVT_REMOTE_START);
    check(S5_PARENT_RUNNING == get_parent() && g_b_active);
    check(2U == g_commands && 1U == g_prepares && 0U == g_sweeps);

    fault_set(FAULT_COLLISION);
    handle_faults(g_faults);
    handle_events(S5_EVT_REMOTE_START);
    check(S5_PARENT_FAULT == get_parent() && !g_b_active);
    check(2U == g_commands);
    handle_events(S5_EVT_RESET);
    check(S5_PARENT_READY == get_parent() && !g_b_active);
    check(2U == g_prepares && 0U == g_sweeps && 0U == g_delays);

    /* A setup failure retries the fixed profile; START cannot bypass FAULT. */
    g_prepare_result = E_IO;
    boot_until_ready();
    check(S5_PARENT_FAULT == get_parent() && !g_b_active);
    check(3U == g_prepares && 2U == g_commands);
    handle_events(S5_EVT_REMOTE_START);
    check(S5_PARENT_FAULT == get_parent() && 2U == g_commands);
    g_prepare_result = E_OK;
    handle_events(S5_EVT_RESET);
    check(S5_PARENT_READY == get_parent() && !g_b_active);
    check(4U == g_prepares && 0U == g_sweeps && 0U == g_delays);
    handle_events(S5_EVT_REMOTE_START);
    check(S5_PARENT_RUNNING == get_parent() && g_b_active);
    check(3U == g_commands && MSG_CMD_LINE_PID == g_command.type);
    printf("Controller startup regressions: %u/%u passed\n",
           g_checks - g_failed, g_checks);
    return ((0U == g_failed) ? 0 : 1);
}
