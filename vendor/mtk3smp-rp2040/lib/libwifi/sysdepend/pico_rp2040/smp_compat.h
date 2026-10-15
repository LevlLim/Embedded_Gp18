/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * smp_compat.h
 *
 * The CYW43439 service is written against the SMP primitives: a cross-core
 * spinlock for the SDK's hardware-claim mutex, a memory barrier around DMA
 * and PIO state, and the processor query used for status reporting.  Those
 * symbols exist only in an SMP build -- kernel/tkernel/smp_lock.c and
 * kernel/sysdepend/cpu/rp2040/smp_atomic.c compile to nothing when
 * TK_SUPPORT_SMP is 0.
 *
 * There is nothing dual-core about the radio itself, so this header supplies
 * single-core equivalents rather than forcing SMP=1:
 *
 *   - a spinlock with no second core to exclude degrades to masking local
 *     interrupts, which is what the lock was protecting against anyway;
 *   - the barrier stays a real DMB.  It is NOT reduced to a compiler barrier:
 *     the CYW43 path talks to DMA and PIO, so the ordering it needs is
 *     against those masters, not against another CPU;
 *   - there is exactly one processor, and it is processor 1.
 *
 * Declared in plain C types on purpose.  The files that include this avoid
 * <tk/tkernel.h> entirely, because the Pico SDK's ISO C size_t collides with
 * the inherited T-Kernel syslib typedef.
 */

#ifndef CYW43_SMP_COMPAT_H
#define CYW43_SMP_COMPAT_H

#include <stdint.h>

typedef volatile uint32_t T_SPLOCK;

#if TK_SUPPORT_SMP

extern int32_t ISpinLock(T_SPLOCK *lock, uint32_t *intsts);
extern int32_t ISpinUnlock(T_SPLOCK *lock, uint32_t intsts);
extern void    mp_memory_barrir(void);
extern int32_t tk_get_prc(void);

#define CYW43_THIS_PRC()	tk_get_prc()

#else	/* single core */

/* PRIMASK save/mask/restore, matching disint()/set_primask() in
   lib/libtk/sysdepend/cpu/core/armv6m.  Written out here rather than
   declared extern, because set_primask() is a static inline in
   <tk/sysdepend/cpu/core/armv6m/syslib.h> and has no external symbol -- and
   that header cannot be included from this translation unit. */

static inline int32_t ISpinLock(T_SPLOCK *lock, uint32_t *intsts)
{
	uint32_t pm;

	(void)lock;			/* no other core can hold it */
	__asm__ volatile ("mrs %0, primask" : "=r"(pm));
	__asm__ volatile ("cpsid i" : : : "memory");
	*intsts = pm;
	return 0;
}

static inline int32_t ISpinUnlock(T_SPLOCK *lock, uint32_t intsts)
{
	(void)lock;
	__asm__ volatile ("msr primask, %0" : : "r"(intsts) : "memory");
	return 0;
}

/* Still a real barrier: ordering here is against DMA and PIO, not a CPU. */
static inline void mp_memory_barrir(void)
{
	__asm__ volatile ("dmb" : : : "memory");
}

#define CYW43_THIS_PRC()	1

#endif	/* TK_SUPPORT_SMP */

#endif /* CYW43_SMP_COMPAT_H */
