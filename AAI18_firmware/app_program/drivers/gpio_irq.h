/*
 * gpio_irq.h
 *
 * Group AAI18 -- the single IO_BANK0 interrupt dispatcher.
 *
 * SHARED OWNERSHIP: Buddy 2 (encoders) and Buddy 5 (ECHO).  Design Review
 * V2 section 9, shared resource "IO_BANK0 GPIO interrupt".
 *
 * WHY ONE DISPATCHER
 * ------------------
 * Every GPIO edge interrupt on the RP2040 -- both encoder A channels and
 * the ultrasonic ECHO line -- arrives on ONE processor interrupt line
 * (IO_IRQ_BANK0).  There is no per-pin vector.  The Week 6 review raised
 * this directly: "all the GPIO IRQs arrive on one IO_BANK0 interrupt, so
 * keep a single short dispatcher".
 *
 * The handler below therefore does the minimum possible work: read which
 * pins fired, update a counter or record a timestamp, clear the latch,
 * return.  No decoding, no kernel call that could block, no rescheduling.
 * Any fault it must raise goes through fault_set_isr(), which is a single
 * event-flag set.
 *
 * UNMEASURED: the ISR cost at the top-speed encoder edge rate is still
 * unknown, because counts-per-revolution and maximum edge rate have never
 * been stated.  That is risk R8.  gpio_irq_max_duration_us() exists so
 * that the first bench session can answer it.
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include "../app.h"
#include <stdint.h>
#include <stdbool.h>

/* Quadrature encoder counters.  NOT packed, deliberately: a free-running
   counter's valid range genuinely is 32 bits, so a 32-bit word already
   satisfies NFR12.  Packing would add a torn read between the dispatcher
   and Motion Control in the highest-priority path for a few bytes of RAM.
   Design Review V2 section 7.4. */
typedef struct
{
    volatile int32_t  count_left;
    volatile int32_t  count_right;
    volatile uint32_t edges_left;      /* unsigned edge total, for rate   */
    volatile uint32_t edges_right;
    volatile uint32_t last_edge_us;    /* for the stall detector          */
} gpio_irq_encoders_t;

/* One completed ECHO pulse.  Written by the dispatcher, read by Obstacle
   Scan.  seq advances last, so a reader can detect a torn read. */
typedef struct
{
    volatile uint32_t rise_us;
    volatile uint32_t width_us;
    volatile uint32_t seq;
    volatile bool     b_pending;       /* rise seen, waiting for fall     */
} gpio_irq_echo_t;

/*
 * Configure GP2/GP4 for both-edge interrupts, GP1 (ECHO) for both-edge, install
 * the dispatcher on INTNO_IO_BANK0 and enable it.  Call once in phase 1,
 * after the pins have been set to input.
 */
ER gpio_irq_init(void);

/* Read the encoder block.  Copies under interrupt disable, so the five
   fields are mutually consistent.  CORE 0 CALLERS ONLY: the dispatcher is
   enabled on core 0 and DI() masks interrupts on the calling core alone,
   so a call from core 1 would not be protected. */
void gpio_irq_read_encoders(gpio_irq_encoders_t *p_out);

/* Read the newest completed echo pulse.  Returns false if none yet, or if
   the sequence number changed during the copy (retry next cycle). */
bool gpio_irq_read_echo(gpio_irq_echo_t *p_out);

/* Longest dispatcher execution seen, in microseconds.  This is the number
   that closes risk R8 and feeds the NFR8 CPU argument. */
uint32_t gpio_irq_max_duration_us(void);

/* Reset the measured maximum, so a timing run can start clean. */
void gpio_irq_reset_stats(void);

#endif /* GPIO_IRQ_H */

/* end of file */
