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

EXPORT ER subsystem4_init(void)
{
    uint8_t  who      = 0u;
    uint32_t failures = 0u;
    ER       ercd;

    ercd = i2c0_init();
    if (ercd < E_OK)
    {
        return (ercd);
    }

    /*
     * Identify the part before configuring it.  A wrong or absent
     * WHO_AM_I means the bus is miswired or the address is wrong.
     */
    if (i2c0_read_reg(S4_ACCEL_ADDR, S4_REG_WHO_AM_I, &who) < E_OK)
    {
        tm_printf((UB*)"[S4] IMU not responding at 0x%02x\n",
                  (INT)S4_ACCEL_ADDR);
        /* Not fatal: the car can still follow the line.  The Controller
           degrades, per the Fault Table. */
        fault_set(FAULT_IMU);
    }
    else
    {
        tm_printf((UB*)"[S4] IMU WHO_AM_I = 0x%02x\n", (INT)who);

        /* 400 Hz into the FIFO, drained every 10 ms, so a wall impact
           lasting a few milliseconds still lands in the block. */
        config_write(S4_REG_CTRL1, S4_CTRL1_400HZ_XYZ, &failures);
        config_write(S4_REG_CTRL4, S4_CTRL4_8G_HR, &failures);
        config_write(S4_REG_CTRL5, S4_CTRL5_FIFO_EN, &failures);
        config_write(S4_REG_FIFO_CTRL, S4_FIFO_CTRL_STREAM, &failures);

        if (0u != failures)
        {
            tm_printf((UB*)"[S4] %d config writes failed\n", (INT)failures);
            fault_set(FAULT_IMU);
        }
    }

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
