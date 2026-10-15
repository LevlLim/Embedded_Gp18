/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	demo_tasks.h
 *	The console-independent liveness task.
 */

#ifndef DEMO_TASKS_H
#define DEMO_TASKS_H

#include <tk/tkernel.h>

/* Blink task entry.  Deliberately touches nothing the console touches: if
   output stops but the LED keeps blinking, the console is at fault; if the
   LED freezes, the kernel is. */
IMPORT void blink_task(INT stacd, void *exinf);

#endif /* DEMO_TASKS_H */
