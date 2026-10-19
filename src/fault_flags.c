/*
 * fault_flags.c - atomic set/clear for the shared fault word.
 *
 * Why this file exists at all: the obvious version,
 *
 *     g_faults |= FLT_IMU_FAULT;
 *
 * compiles on Cortex-M0+ to LDR, ORR, STR. Motion Control or the Vehicle
 * Controller can run between the LDR and the STR, and that task's update is
 * then overwritten. A fault bit disappears and nothing reports it.
 *
 * The kernel event flag does the update atomically, so all set and clear
 * operations go through here. No application code touches the word directly.
 */
#include "fault_flags.h"

#ifdef HOST_TEST
/* ---- host build: single threaded, plain variable is enough ---- */
static uint32_t g_fault_word;

void fault_set(uint32_t bits)    { g_fault_word |=  (bits & FLT_ALL_MASK); }
void fault_clear(uint32_t bits)  { g_fault_word &= ~(bits & FLT_ALL_MASK); }
uint32_t fault_read(void)        { return g_fault_word; }

#else
/* ---- target build: uT-Kernel 3.0 event flag ---- */
#include "kernel.h"

extern ID g_flg_fault;   /* created in main, ID shared by all tasks */

void fault_set(uint32_t bits)
{
    /* tk_set_flg ORs the pattern in, atomically. */
    tk_set_flg(g_flg_fault, (UINT)(bits & FLT_ALL_MASK));
}

void fault_clear(uint32_t bits)
{
    /* tk_clr_flg ANDs with the pattern given, so pass the bits to KEEP.
     * Vehicle Controller only. */
    tk_clr_flg(g_flg_fault, (UINT)~(bits & FLT_ALL_MASK));
}

uint32_t fault_read(void)
{
    UINT ptn = 0;
    /* Non-blocking poll of the current pattern. */
    tk_wai_flg(g_flg_fault, FLT_ALL_MASK, TWF_ORW | TWF_BITCLR * 0, &ptn, TMO_POL);
    return (uint32_t)ptn;
}
#endif
