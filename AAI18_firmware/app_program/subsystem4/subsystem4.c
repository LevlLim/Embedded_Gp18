/*
 * subsystem4.c
 *
 * Subsystem 4 -- IMU / Terrain: brings up I2C0 and the accelerometer,
 * then starts its task.
 *
 * This subsystem is the sole user of I2C0, so there is no second
 * claimant to arbitrate with.
 */

#include "subsystem4.h"
#include "i2c0.h"
#include "../fault.h"

/*
 * Write one configuration register, counting a failure.
 */
static void config_write(uint8_t reg, uint8_t value, uint32_t *p_failures)
{
    if (i2c0_write_reg(S4_ACCEL_ADDR, reg, value) < E_OK)
    {
        *p_failures = *p_failures + 1u;
    }
}

/*
 * Identify the IMU and write its configuration.  Returns true if it
 * answered and every write succeeded.  Called at init, and again by the
 * IMU task while calibration gets no data: an IMU that powers up after
 * the Pico (or a bus glitch at power-on) otherwise stays unconfigured --
 * powered down, FIFO off -- for the whole run.
 */
EXPORT bool s4_configure_imu(bool b_verbose)
{
    uint8_t  who      = 0u;
    uint32_t failures = 0u;
    bool     b_ok     = false;

    /*
     * Identify the part before configuring it.  A wrong or absent
     * WHO_AM_I means the bus is miswired or the address is wrong.
     */
    if ((i2c0_read_reg(S4_ACCEL_ADDR, S4_REG_WHO_AM_I, &who) < E_OK)
        && (!i2c0_bus_reset()
            || (i2c0_read_reg(S4_ACCEL_ADDR, S4_REG_WHO_AM_I, &who) < E_OK)))
    {
        /* Not answering, even after a bus clear (SDA still held low, or
           nothing on the bus at all). */
        if (b_verbose)
        {
            APP_LOG((UB*)"[S4] IMU not responding at 0x%02x\n",
                    (INT)S4_ACCEL_ADDR);
        }
    }
    else
    {
        if (b_verbose)
        {
            APP_LOG((UB*)"[S4] IMU WHO_AM_I = 0x%02x\n", (INT)who);
        }

        /* 400 Hz into the FIFO, drained every 10 ms, so a wall impact
           lasting a few milliseconds still lands in the block. */
        config_write(S4_REG_CTRL1, S4_CTRL1_400HZ_XYZ, &failures);
        config_write(S4_REG_CTRL4, S4_CTRL4_8G_HR, &failures);
        config_write(S4_REG_CTRL5, S4_CTRL5_FIFO_EN, &failures);
        config_write(S4_REG_FIFO_CTRL, S4_FIFO_CTRL_STREAM, &failures);

        if ((0u != failures) && b_verbose)
        {
            APP_LOG((UB*)"[S4] %d config writes failed\n", (INT)failures);
        }
        b_ok = (0u == failures);
    }

    return (b_ok);
}

EXPORT ER subsystem4_init(void)
{
    ER ercd;

    ercd = i2c0_init();
    if (ercd < E_OK)
    {
        return (ercd);
    }

    /* Not fatal if it fails here: the IMU task retries while it
       calibrates, and only then reports IMU_FAULT (DEGRADE: the car can
       still follow the line). */
    (void)s4_configure_imu(true);

    return (E_OK);
}

EXPORT ER subsystem4_start(void)
{
    ID tid;

    tid = app_start_periodic_task((FP)imu_task, PRI_IMU_TERRAIN,
                                  TP_PRC1, "imu_task", PERIOD_IMU_MS);

    return ((tid < E_OK) ? (ER)tid : E_OK);
}

/* end of file */
