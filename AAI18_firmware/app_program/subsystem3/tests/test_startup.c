/** Startup retries must not report false success or create duplicate tasks. */
#include <stdio.h>
#include <string.h>
#include "../subsystem3.c"

static unsigned int g_tests = 0U;
static unsigned int g_failed = 0U;
static ER g_adc_result = E_OK;
static ER g_barcode_result = E_OK;
static unsigned int g_barcode_inits = 0U;
static ID g_line_result = E_OBJ;
static ID g_task_result = E_OBJ;
static unsigned int g_line_starts = 0U;
static unsigned int g_barcode_starts = 0U;

static void check(bool condition)
{
    g_tests++;
    if (!condition)
    {
        g_failed++;
        printf("Startup failure at check %u\n", g_tests);
    }
}

ER adc_sampler_init(void) { return (g_adc_result); }
ER s3_barcode_init(void)
{
    g_barcode_inits++;
    return (g_barcode_result);
}
void line_task(INT code, void * context) { (void)code; (void)context; }
void barcode_task(INT code, void * context) { (void)code; (void)context; }
ID app_start_periodic_task(FP entry, PRI priority, UW processor,
                           char const * name, uint32_t period_ms)
{
    (void)entry;
    (void)priority;
    check(TP_PRC1 == processor);
    if (0 == strcmp(name, "line_task"))
    {
        g_line_starts++;
        check(PERIOD_LINE_MS == period_ms);
        return (g_line_result);
    }
    g_barcode_starts++;
    check(0 == strcmp(name, "barcode_task"));
    check(PERIOD_BARCODE_MS == period_ms);
    return (g_task_result);
}

int main(void)
{
    g_adc_result = E_IO;
    check(E_IO == subsystem3_init());
    check(0U == g_barcode_inits);
    g_adc_result = E_OK;
    g_barcode_result = E_OBJ;
    check(E_OBJ == subsystem3_init());
    g_barcode_result = E_OK;
    check(E_OK == subsystem3_init());

    check(E_OBJ == subsystem3_start());
    check(1U == g_line_starts && 0U == g_barcode_starts);
    g_line_result = 11;
    check(E_OBJ == subsystem3_start());
    check(2U == g_line_starts && 1U == g_barcode_starts);
    /* Line task is already running when the barcode task creation retries. */
    g_task_result = 12;
    check(E_OK == subsystem3_start());
    check(2U == g_line_starts && 2U == g_barcode_starts);
    check(E_OK == subsystem3_start());
    check(2U == g_line_starts && 2U == g_barcode_starts);
    printf("Startup regressions: %u/%u passed\n", g_tests - g_failed, g_tests);
    return ((0U == g_failed) ? 0 : 1);
}
