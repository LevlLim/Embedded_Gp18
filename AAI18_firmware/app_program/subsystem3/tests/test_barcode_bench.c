/** Bench entry point must start only the scanner/reporter and never drive. */
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "../barcode_bench.c"

static unsigned int g_checks = 0U;
static unsigned int g_failed = 0U;
static unsigned int g_brakes = 0U;
static unsigned int g_scanners = 0U;
static unsigned int g_reporters = 0U;
static unsigned int g_prepares = 0U;
static bool g_b_active = false;
static ER g_init_result = E_OK;
static ID g_scan_result = 1;
static ID g_report_result = 2;
static jmp_buf g_sleep;

static void check(bool condition)
{
    g_checks++;
    if (!condition)
    {
        g_failed++;
        printf("Bench failure at check %u\n", g_checks);
    }
}

ER motor_init(void) { return (E_OK); }
void motor_brake(void) { g_brakes++; }
ER subsystem3_init(void) { return (g_init_result); }
ER s3_prepare(void) { g_prepares++; return (E_OK); }
void s3_set_active(bool b_on) { g_b_active = b_on; }
void barcode_task(INT code, void * p_context) { (void)code; (void)p_context; }
ID app_start_periodic_task(FP entry, PRI priority, UW processor,
                           char const * p_name, uint32_t period_ms)
{
    g_scanners++;
    check((FP)barcode_task == entry);
    check(PRI_BARCODE == priority && TP_PRC1 == processor);
    check(0 == strcmp("barcode_task", p_name) && 1U == period_ms);
    return (g_scan_result);
}
ID app_start_task(FP entry, PRI priority, UW processor, char const * p_name)
{
    g_reporters++;
    check((FP)bench_report_task == entry);
    check(PRI_TELEMETRY == priority && TP_PRC2 == processor);
    check(0 == strcmp("barcode_bench_report", p_name));
    return (g_report_result);
}
void tm_printf(UB * p_format, ...) { (void)p_format; }
uint32_t app_now_ms(void) { return (0U); }
bool s3_take_barcode_result(s3_barcode_result_t * p_out) { (void)p_out; return (false); }
void s3_report_barcode(void) { }
bool adc_sampler_read(uint8_t channel, adc_sample_t * p_out)
{
    (void)channel;
    (void)p_out;
    return (false);
}
ER tk_dly_tsk(INT delay) { (void)delay; return (E_OK); }
ER tk_slp_tsk(TMO timeout) { check(TMO_FEVR == timeout); longjmp(g_sleep, 1); }

static void boot(void)
{
    g_brakes = 0U;
    g_scanners = 0U;
    g_reporters = 0U;
    g_prepares = 0U;
    g_b_active = false;
    if (0 == setjmp(g_sleep))
    {
        (void)s3_barcode_bench_main();
    }
}

int main(void)
{
    boot();
    check(g_b_active && 1U == g_brakes);
    check(1U == g_scanners && 1U == g_reporters && 1U == g_prepares);
    g_init_result = E_IO;
    boot();
    check(!g_b_active && 2U == g_brakes);
    check(0U == g_scanners && 0U == g_reporters && 0U == g_prepares);
    g_init_result = E_OK;
    g_scan_result = E_OBJ;
    boot();
    check(!g_b_active && 2U == g_brakes);
    check(1U == g_scanners && 0U == g_reporters && 1U == g_prepares);
    g_scan_result = 1;
    g_report_result = E_OBJ;
    boot();
    check(!g_b_active && 2U == g_brakes);
    check(1U == g_scanners && 1U == g_reporters && 1U == g_prepares);
    printf("Bench startup regressions: %u/%u passed\n", g_checks - g_failed, g_checks);
    return ((0U == g_failed) ? 0 : 1);
}
