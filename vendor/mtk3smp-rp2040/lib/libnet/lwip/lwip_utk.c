/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Minimal NO_SYS=1 operating-system boundary for lwIP. */

#include <stdint.h>

#include "lwip/sys.h"

extern uint64_t time_us_64(void);

u32_t sys_now(void)
{
    return (u32_t)(time_us_64() / 1000ULL);
}

uint32_t lwip_utk_rand(void)
{
    static uint32_t state = 0x6d2b79f5u;

    state ^= (uint32_t)time_us_64();
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

void lwip_utk_assert(const char *message)
{
    (void)message;
    __asm__ volatile("cpsid i" ::: "memory");
    for(;;) __asm__ volatile("wfi");
}
