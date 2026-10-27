/*
 * fault.h
 *
 * Group AAI18 -- the shared fault bitmask.
 *
 * Implements the Fault Table, Design Review V2 section 7.3, and satisfies
 * the part of Change Requirement 1 that asks for fault conditions to use
 * individual bits in a shared bitmask rather than separate integer
 * variables.
 *
 * OWNERSHIP: Buddy 5 owns this file and the Fault Table it encodes.
 * Individual bits are set by the subsystem that detects the fault.
 *
 * WHY THE RULES BELOW MATTER
 * --------------------------
 * Merging ten faults into one word is only safe with three rules attached,
 * and all three are enforced by this module's API rather than by comment:
 *
 *  1. Each task sets ONLY its own bits, through fault_set().  No task ever
 *     performs a read-modify-write on the whole word.  The Cortex-M0+ has
 *     no atomic bit-set instruction, so a read-modify-write from two
 *     sources silently loses one of them -- this is risk R4, and it would
 *     mean a real collision is never reported.
 *
 *  2. Only the Vehicle Controller clears, through fault_clear(), on RESET.
 *     WIFI_LOSS is the single exception: it is self-healing and is in
 *     neither the E-STOP nor the DEGRADE mask, so Telemetry clears it on
 *     reconnect.
 *
 *  3. Motion Control brakes on FAULT_MASK_ESTOP ONLY (it reads the mask
 *     at the start of every 10 ms cycle).  This is the most
 *     important line in the module.  Without it a Wi-Fi-loss bit sharing
 *     the word would brake the car, which breaks NFR6.  Waiters must not
 *     use clear-on-wait semantics, or a second waiter would consume
 *     another task's bits.
 */

#ifndef FAULT_H
#define FAULT_H

#include "app.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------
 * Bit assignments.  Design Review V2 table 7.3.
 * Severity column:  E = E-STOP, D = DEGRADE, R = REPORT-ONLY
 * --------------------------------------------------------------------- */
#define FAULT_COLLISION         (0x0001u)  /* bit 0  E  IMU / Terrain     */
#define FAULT_TILT              (0x0002u)  /* bit 1  E  IMU / Terrain     */
#define FAULT_OBSTACLE_STOP     (0x0004u)  /* bit 2  E  Obstacle Scan     */
#define FAULT_REMOTE_STOP       (0x0008u)  /* bit 3  E  Telemetry / VC    */
#define FAULT_LINE_LOST_HARD    (0x0010u)  /* bit 4  E  Vehicle Controller*/
#define FAULT_MOTOR_STALL       (0x0020u)  /* bit 5  E  Motion Control    */
#define FAULT_ENCODER           (0x0040u)  /* bit 6  D  Motion Control    */
#define FAULT_IMU               (0x0080u)  /* bit 7  D  IMU / Terrain     */
#define FAULT_NO_ECHO           (0x0100u)  /* bit 8  D  Obstacle Scan     */
#define FAULT_WIFI_LOSS         (0x0200u)  /* bit 9  R  Telemetry / Comms */

/*
 * Masks.  Motion Control waits on FAULT_MASK_ESTOP and nothing else.
 */
#define FAULT_MASK_ESTOP        (FAULT_COLLISION      \
                               | FAULT_TILT           \
                               | FAULT_OBSTACLE_STOP  \
                               | FAULT_REMOTE_STOP    \
                               | FAULT_LINE_LOST_HARD \
                               | FAULT_MOTOR_STALL)       /* 0x003F */

#define FAULT_MASK_DEGRADE      (FAULT_ENCODER \
                               | FAULT_IMU     \
                               | FAULT_NO_ECHO)           /* 0x01C0 */

#define FAULT_MASK_REPORT       (FAULT_WIFI_LOSS)         /* 0x0200 */

#define FAULT_MASK_ALL          (FAULT_MASK_ESTOP   \
                               | FAULT_MASK_DEGRADE \
                               | FAULT_MASK_REPORT)

/* ---------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------- */

/* Create the event flag.  Called once from usermain phase 1. */
ER fault_init(void);

/*
 * Set one or more fault bits from task context.  Never read-modify-write.
 * Safe to call from several tasks concurrently: the kernel's set-flag
 * operation ORs the bits in.
 */
void fault_set(uint16_t bits);

/*
 * Set fault bits from interrupt context.  Keeps the handler to
 * set-flag-and-return, with no decoding and no rescheduling inside the
 * handler (Design Review V2 section 7.3, ISR rule).
 */
void fault_set_isr(uint16_t bits);

/*
 * Clear fault bits.  Only the Vehicle Controller may call this, except
 * that Telemetry may clear FAULT_WIFI_LOSS on reconnect.
 */
void fault_clear(uint16_t bits);

/* Read the whole mask without consuming any bit.  Used by Telemetry so the
   operator sees every concurrent fault, not only the first. */
uint16_t fault_get(void);

/* True if any E-STOP bit is currently set. */
bool fault_is_estop(void);

/*
 * Block until any bit in FAULT_MASK_ESTOP is set, or until the timeout
 * expires.  Uses TWF_ORW without clear, so a second waiter cannot consume
 * another task's bits.  Returns E_OK if a bit was seen, E_TMOUT on timeout.
 */
ER fault_wait_estop(uint16_t *p_bits, TMO timeout);

/* Human-readable name for one bit, for console output and test logs. */
const char *fault_name(uint16_t single_bit);

#endif /* FAULT_H */

/* end of file */
