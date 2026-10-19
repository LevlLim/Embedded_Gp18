/*
 * fault_flags.h - shared fault bitmask (Change Requirement 1)
 * Owner of this header: agreed in Tribunal 1, change log row "Each fault is
 * linked to its own variable" -> "Faults become individual bits in one shared
 * fault flag, set and cleared with kernel calls" (Major redesign, all buddies).
 *
 * Ten faults fit in 10 bits instead of ten separate variables.
 *
 * ONE WRITER PER FIELD
 * --------------------
 * The RP2040 is Cortex-M0+. It has no LDREX/STREX and no bit-banding, so
 *   flags |= BIT;
 * is a read, a modify and a write. If a second task writes in between, one
 * update is lost. The team decision was:
 *
 *   - each task sets ONLY the bits it owns (see table below)
 *   - only the Vehicle Controller clears bits
 *   - set/clear go through the kernel event flag, which is atomic
 *
 * Never read-modify-write this word directly from application code.
 *
 *   bit  fault                owner (setter)
 *   ---  -------------------  --------------------
 *    0   MOTOR_STALL          Buddy 2  Motion Control
 *    1   ENCODER_FAULT        Buddy 2  Motion Control
 *    2   IMU_FAULT            Buddy 4  IMU / Terrain
 *    3   COLLISION            Buddy 4  IMU / Terrain
 *    4   NO_ECHO              Buddy 5  Obstacle Scan
 *    5   SERVO_FAULT          Buddy 5  Obstacle Scan
 *    6   LINE_LOST            Buddy 3  Line Sensor
 *    7   BARCODE_FAULT        Buddy 3  Barcode
 *    8   WIFI_LOST            Buddy 1  Comms
 *    9   MQTT_LOST            Buddy 1  Comms
 */
#ifndef FAULT_FLAGS_H
#define FAULT_FLAGS_H

#include <stdint.h>

#define FLT_MOTOR_STALL    (1u << 0)
#define FLT_ENCODER_FAULT  (1u << 1)
#define FLT_IMU_FAULT      (1u << 2)
#define FLT_COLLISION      (1u << 3)
#define FLT_NO_ECHO        (1u << 4)
#define FLT_SERVO_FAULT    (1u << 5)
#define FLT_LINE_LOST      (1u << 6)
#define FLT_BARCODE_FAULT  (1u << 7)
#define FLT_WIFI_LOST      (1u << 8)
#define FLT_MQTT_LOST      (1u << 9)

#define FLT_ALL_MASK       0x03FFu   /* 10 bits total */

/* Bits each buddy is allowed to set. Used by the unit test to prove no two
 * owners share a bit, which is what makes the one-writer rule hold. */
#define FLT_OWNED_BUDDY1   (FLT_WIFI_LOST | FLT_MQTT_LOST)
#define FLT_OWNED_BUDDY2   (FLT_MOTOR_STALL | FLT_ENCODER_FAULT)
#define FLT_OWNED_BUDDY3   (FLT_LINE_LOST | FLT_BARCODE_FAULT)
#define FLT_OWNED_BUDDY4   (FLT_IMU_FAULT | FLT_COLLISION)
#define FLT_OWNED_BUDDY5   (FLT_NO_ECHO | FLT_SERVO_FAULT)

/*
 * Set one or more fault bits. Safe to call from a task or from an ISR.
 * Only pass bits this caller owns.
 */
void fault_set(uint32_t bits);

/*
 * Clear fault bits. Vehicle Controller only. Any other caller is a design
 * error, the bit would be cleared behind the owner's back.
 */
void fault_clear(uint32_t bits);

/* Read the current fault word. Read-only, never used for RMW. */
uint32_t fault_read(void);

#endif /* FAULT_FLAGS_H */
