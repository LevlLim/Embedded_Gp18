/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	demo_tasks.c
 *	The console-independent liveness task.  See demo_tasks.h.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "usb_console_compat.h"
#include "demo_tasks.h"


/*
 * Blink task: liveness independent of the console, and -- since USB does not
 * enumerate and therefore cannot report on itself -- a readout of how far USB
 * bring-up gets, using the long/short count encoding from Phase 0.
 *
 *	0  (dark)  tusb_init() failed
 *	1          tk_def_int() rejected the USB interrupt
 *	2          NVIC never unmasked USBCTRL_IRQ
 *	3          USB controller not enabled
 *	4          D+ pull-up not asserted, so no host can see the device
 *	5          pull-up on, but not one USB interrupt has ever fired
 *	6          interrupts firing, but no bus reset or connect from the host
 *	7          host talking, enumeration not completing
 *	8          fully enumerated
 *
 * If any console call blocks, this task stops and the LED freezes entirely.
 */
LOCAL void blink_code(INT n)
{
	INT	i;

	for(i = 0; i < n / 5; i++) {		/* long flash = 5 */
		gpio_set_val(BOARD_LED_PIN, 1);
		tk_dly_tsk(600);
		gpio_set_val(BOARD_LED_PIN, 0);
		tk_dly_tsk(300);
	}
	for(i = 0; i < n % 5; i++) {		/* short flash = 1 */
		gpio_set_val(BOARD_LED_PIN, 1);
		tk_dly_tsk(150);
		gpio_set_val(BOARD_LED_PIN, 0);
		tk_dly_tsk(250);
	}
	gpio_set_val(BOARD_LED_PIN, 0);
	tk_dly_tsk(2000);
}

EXPORT void blink_task(INT stacd, void *exinf)
{
	INT	level;

	(void)stacd; (void)exinf;

#if BOARD_DIAG_GPIO_SCAN
	knl_diag_pulse(8);	/* CP8: the blink task is being scheduled */
#endif

	while(1) {
		level = tm_usb_diag_level();
		if(level >= 8) {
			/* Fully up: plain 2 Hz liveness, so a console stall or a
			   blocked kernel is visible at a glance. */
			gpio_set_val(BOARD_LED_PIN, 1);
			tk_dly_tsk(250);
			gpio_set_val(BOARD_LED_PIN, 0);
			tk_dly_tsk(250);
		} else {
			blink_code(level);
		}
	}
}
