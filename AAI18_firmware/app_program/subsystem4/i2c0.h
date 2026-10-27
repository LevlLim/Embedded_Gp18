/*
 * i2c0.h
 *
 * Group AAI18 -- blocking I2C0 master for the IMU.
 *
 * OWNERSHIP: Buddy 4.  I2C0 has exactly one device on it (the IMU), which
 * is why the Week 6 decision to reject bus-sharing synchronisation stands:
 * there is no second claimant to arbitrate with.
 *
 * The template's BSP has no I2C support and hardware/i2c.h is not on the
 * application include path, so this is written against the Synopsys
 * DW_apb_i2c registers directly.  See drivers/rp2040_regs.h.
 *
 * TIMING NOTE (NFR11)
 * -------------------
 * These transfers are polled, not interrupt-driven.  At 400 kHz a 6-byte
 * burst read costs roughly 150 us of bus time, and the IMU task has a 4 ms
 * budget per 10 ms cycle, so polling is affordable.  Every call carries a
 * timeout in microseconds: a dead or unplugged sensor must not spin the
 * High-priority IMU task forever, it must return an error so the task can
 * raise FAULT_IMU and let the Controller degrade.  That is the defined
 * safe behaviour the Week 6 review asked for.
 */

#ifndef I2C0_H
#define I2C0_H

#include "../app.h"
#include <stdint.h>
#include <stdbool.h>

/* Default per-transfer timeout.  Generous enough for a 32-byte burst at
   400 kHz, short enough that three consecutive failures still fit inside
   one 10 ms IMU cycle. */
#define I2C0_TIMEOUT_US     (3000u)  /* fits a 48-byte burst at 400 kHz */

/* Bring I2C0 up at 400 kHz on GP16 (SDA) and GP17 (SCL). */
ER i2c0_init(void);

/*
 * Write then read, with a repeated START between (the standard register
 * read pattern).  Returns E_OK, E_TMOUT on timeout, or E_IO if the slave
 * did not acknowledge.
 */
ER i2c0_write_read(uint8_t addr, const uint8_t *p_tx, uint8_t tx_len,
                   uint8_t *p_rx, uint8_t rx_len, uint32_t timeout_us);

/* Convenience wrappers for single-register access. */
ER i2c0_read_reg(uint8_t addr, uint8_t reg, uint8_t *p_value);
ER i2c0_read_burst(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint8_t len);
ER i2c0_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

/* Count of transfers that timed out or were NAKed since init.  Published
   in telemetry so a flaky bus is visible rather than silent. */
uint32_t i2c0_error_count(void);

/*
 * Free a bus held low by the IMU (bus clear: up to 9 clock pulses, then a
 * STOP) and restart the I2C block.  Returns true if SDA is free after.
 * Also done once by i2c0_init().  Not for use while a transfer runs.
 */
bool i2c0_bus_reset(void);

#endif /* I2C0_H */

/* end of file */
