/*
 * i2c0.c
 *
 * Group AAI18 -- polled I2C0 master (RP2040 / Synopsys DW_apb_i2c).
 *
 * TRANSACTION DISCIPLINE (fixed in this revision)
 * ------------------------------------------------
 *  - Every transfer ends by waiting for STOP_DET and then CLEARING it.
 *    Previously a read left STOP_DET set, so the next write-only transfer
 *    saw the stale flag, returned at once, and the call after that
 *    disabled the controller in the middle of the write.
 *  - The controller is only disabled (to change TAR) once it is idle and
 *    IC_ENABLE_STATUS confirms it.
 *  - Read commands are queued no faster than the receive FIFO is drained,
 *    so a burst longer than the 16-entry FIFO cannot overflow it.
 *
 * VERIFY ON A SCOPE: the FS_SCL_HCNT/LCNT values assume a 125 MHz clock.
 */

#include "i2c0.h"
#include "../drivers/rp2040_regs.h"

static uint32_t error_count   = 0u;
static bool     b_initialised = false;

/* ---------------------------------------------------------------------
 * Private
 * --------------------------------------------------------------------- */

/* True once the deadline has passed (wrap-safe). */
static bool is_expired(uint32_t deadline_us)
{
    /* The cast reads the wrap-safe difference as signed. */
    return ((int32_t)(RP2040_NOW_US() - deadline_us) >= 0);
}

/*
 * Route one GPIO to the I2C peripheral with its input buffer and pull-up
 * enabled.  The Robo Pico carries external pull-ups; the internal ones let
 * the bus work on a bare breadboard too.
 */
static void i2c_pin_setup(uint32_t gpio)
{
    set_w(PADS_GPIO(gpio), PADS_IE | PADS_PUE);
    clr_w(PADS_GPIO(gpio), PADS_PDE | PADS_OD);

    out_w(GPIO_CTRL(gpio), GPIO_FUNC_I2C);
}

/* Busy-wait a few microseconds (bus recovery only, at init / retry). */
static void wait_us(uint32_t us)
{
    uint32_t start = RP2040_NOW_US();

    while ((RP2040_NOW_US() - start) < us)
    {
        /* Spin. */
    }
}

/*
 * I2C BUS RECOVERY (I2C specification, "bus clear").  If the Pico resets
 * in the middle of a read -- reflashing, the RESET button, a brown-out --
 * while the IMU stays powered, the IMU can be left driving SDA low,
 * waiting for clock pulses that never come.  Every later transfer then
 * fails ("IMU not answering on I2C") until the IMU loses power.  Clock
 * SCL by hand, as a GPIO, up to 9 times until SDA is released, then send
 * a STOP.  Returns true if SDA is high (bus free) at the end.
 */
static bool bus_clear(void)
{
    uint32_t pulse;
    bool     b_free;

    /* Both lines as plain GPIO with pull-ups: SDA input, SCL output. */
    set_w(PADS_GPIO(PIN_IMU_SDA), PADS_IE | PADS_PUE);
    set_w(PADS_GPIO(PIN_IMU_SCL), PADS_IE | PADS_PUE);
    (void)gpio_set_pin(PIN_IMU_SDA, GPIO_MODE_IN);
    (void)gpio_set_val(PIN_IMU_SCL, 1u);
    (void)gpio_set_pin(PIN_IMU_SCL, GPIO_MODE_OUT);
    wait_us(10u);

    for (pulse = 0u; (pulse < 9u) && (0u == gpio_get_val(PIN_IMU_SDA));
         pulse++)
    {
        (void)gpio_set_val(PIN_IMU_SCL, 0u);
        wait_us(5u);
        (void)gpio_set_val(PIN_IMU_SCL, 1u);
        wait_us(5u);
    }

    /* STOP: SDA low -> high while SCL is high. */
    (void)gpio_set_val(PIN_IMU_SCL, 0u);
    (void)gpio_set_val(PIN_IMU_SDA, 0u);
    (void)gpio_set_pin(PIN_IMU_SDA, GPIO_MODE_OUT);
    wait_us(5u);
    (void)gpio_set_val(PIN_IMU_SCL, 1u);
    wait_us(5u);
    (void)gpio_set_pin(PIN_IMU_SDA, GPIO_MODE_IN);   /* release: pull-up */
    wait_us(5u);

    b_free = (0u != gpio_get_val(PIN_IMU_SDA));

    return (b_free);
}

/*
 * Abort and reset the controller so the next call starts clean.
 */
static void i2c_recover(void)
{
    (void)in_w(I2C_CLR_TX_ABRT);
    (void)in_w(I2C_CLR_INTR);
    out_w(I2C_ENABLE, 0u);
    out_w(I2C_ENABLE, 1u);
    error_count = error_count + 1u;
}

/*
 * Wait until the controller is idle, disable it, and wait until the
 * disable has taken effect, so TAR may be written.
 */
static ER i2c_disable_when_idle(uint32_t deadline_us)
{
    ER ercd = E_OK;

    while ((0u != (in_w(I2C_STATUS) & I2C_STATUS_ACTIVITY))
           && (E_OK == ercd))
    {
        ercd = is_expired(deadline_us) ? E_TMOUT : E_OK;
    }

    out_w(I2C_ENABLE, 0u);

    while ((0u != (in_w(I2C_ENABLE_STATUS) & I2C_ENABLE_STATUS_EN))
           && (E_OK == ercd))
    {
        ercd = is_expired(deadline_us) ? E_TMOUT : E_OK;
    }

    return (ercd);
}

/*
 * Push one command word once the TX FIFO has room.
 */
static ER i2c_push(uint32_t cmd, uint32_t deadline_us)
{
    ER ercd = E_OK;

    while ((0u == (in_w(I2C_STATUS) & I2C_STATUS_TFNF)) && (E_OK == ercd))
    {
        ercd = is_expired(deadline_us) ? E_TMOUT : E_OK;
    }

    if (E_OK == ercd)
    {
        out_w(I2C_DATA_CMD, cmd);
    }

    return (ercd);
}

/*
 * Queue the read commands and drain the replies, keeping no more reads
 * outstanding than the RX FIFO can hold.
 */
static ER i2c_read_phase(uint8_t *p_rx, uint8_t rx_len, uint32_t deadline_us)
{
    uint32_t cmd;
    uint8_t  queued   = 0u;
    uint8_t  received = 0u;
    ER       ercd     = E_OK;

    while ((received < rx_len) && (E_OK == ercd))
    {
        if (0u != (in_w(I2C_RAW_INTR_STAT) & I2C_RAW_TX_ABRT))
        {
            ercd = E_IO;    /* no ACK: absent device or wrong address */
        }
        else if (0u != (in_w(I2C_STATUS) & I2C_STATUS_RFNE))
        {
            p_rx[received] = (uint8_t)(in_w(I2C_DATA_CMD) & 0xFFu);
            received       = (uint8_t)(received + 1u);
        }
        else if ((queued < rx_len)
                 && ((uint32_t)(queued - received) < I2C_FIFO_DEPTH))
        {
            cmd = I2C_DATA_CMD_READ;
            if (0u == queued)
            {
                cmd |= I2C_DATA_CMD_RESTART;
            }
            if ((rx_len - 1u) == queued)
            {
                cmd |= I2C_DATA_CMD_STOP;
            }
            ercd   = i2c_push(cmd, deadline_us);
            queued = (uint8_t)(queued + 1u);
        }
        else
        {
            ercd = is_expired(deadline_us) ? E_TMOUT : E_OK;
        }
    }

    return (ercd);
}

/*
 * Wait for the bus STOP, then clear it so the next transfer cannot see a
 * stale flag.
 */
static ER i2c_wait_stop(uint32_t deadline_us)
{
    ER ercd = E_OK;

    while ((0u == (in_w(I2C_RAW_INTR_STAT) & I2C_RAW_STOP_DET))
           && (E_OK == ercd))
    {
        if (0u != (in_w(I2C_RAW_INTR_STAT) & I2C_RAW_TX_ABRT))
        {
            ercd = E_IO;
        }
        else
        {
            ercd = is_expired(deadline_us) ? E_TMOUT : E_OK;
        }
    }

    (void)in_w(I2C_CLR_STOP_DET);

    return (ercd);
}

/* ---------------------------------------------------------------------
 * Public
 * --------------------------------------------------------------------- */

EXPORT ER i2c0_init(void)
{
    if (b_initialised)
    {
        return (E_OK);
    }

    /* Release I2C0 from reset. */
    clr_w(RESETS_RESET, RESETS_RESET_I2C0);
    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C0))
    {
        /* Runs once at init, before any task exists. */
    }

    out_w(I2C_ENABLE, 0u);

    out_w(I2C_CON, I2C_CON_MASTER_MODE
                 | I2C_CON_SPEED_FAST
                 | I2C_CON_7BIT_ADDR
                 | I2C_CON_RESTART_EN
                 | I2C_CON_SLAVE_DISABLE
                 | I2C_CON_TX_EMPTY_CTRL);

    /*
     * 400 kHz from a 125 MHz system clock: one bit period is 312.5 clocks,
     * split roughly 1:2 high:low for fast mode.  VERIFY on a scope.
     */
    out_w(I2C_FS_SPKLEN,  7u);
    out_w(I2C_FS_SCL_HCNT, 95u);
    out_w(I2C_FS_SCL_LCNT, 205u);

    out_w(I2C_TX_TL, 0u);
    out_w(I2C_RX_TL, 0u);

    out_w(I2C_ENABLE, 1u);

    /* Free a bus left stuck by a reset mid-transfer, then hand the pins
       to the I2C block. */
    (void)bus_clear();
    i2c_pin_setup(PIN_IMU_SDA);
    i2c_pin_setup(PIN_IMU_SCL);

    b_initialised = true;

    return (E_OK);
}

EXPORT bool i2c0_bus_reset(void)
{
    bool b_free;

    out_w(I2C_ENABLE, 0u);
    b_free = bus_clear();
    i2c_pin_setup(PIN_IMU_SDA);
    i2c_pin_setup(PIN_IMU_SCL);
    (void)in_w(I2C_CLR_INTR);
    out_w(I2C_ENABLE, 1u);

    return (b_free);
}

EXPORT ER i2c0_write_read(uint8_t addr, const uint8_t *p_tx, uint8_t tx_len,
                          uint8_t *p_rx, uint8_t rx_len, uint32_t timeout_us)
{
    uint32_t deadline_us;
    uint32_t cmd;
    uint8_t  idx;
    ER       ercd;

    if (!b_initialised)
    {
        return (E_OBJ);
    }

    if (((NULL == p_tx) && (0u != tx_len))
        || ((NULL == p_rx) && (0u != rx_len))
        || ((0u == tx_len) && (0u == rx_len)))
    {
        return (E_PAR);
    }

    deadline_us = RP2040_NOW_US() + timeout_us;

    /* Target address may only change with the controller idle and off. */
    ercd = i2c_disable_when_idle(deadline_us);
    out_w(I2C_TAR, (uint32_t)addr);
    (void)in_w(I2C_CLR_INTR);
    out_w(I2C_ENABLE, 1u);

    /* ---- write phase: the last byte carries STOP if nothing is read ---- */
    for (idx = 0u; (idx < tx_len) && (E_OK == ercd); idx++)
    {
        cmd = (uint32_t)p_tx[idx];
        if ((0u == rx_len) && ((tx_len - 1u) == idx))
        {
            cmd |= I2C_DATA_CMD_STOP;
        }
        ercd = i2c_push(cmd, deadline_us);
    }

    /* ---- read phase ---- */
    if ((E_OK == ercd) && (0u != rx_len))
    {
        ercd = i2c_read_phase(p_rx, rx_len, deadline_us);
    }

    /* ---- every transfer ends with a cleared STOP ---- */
    if (E_OK == ercd)
    {
        ercd = i2c_wait_stop(deadline_us);
    }

    if (E_OK != ercd)
    {
        i2c_recover();
    }

    return (ercd);
}

EXPORT ER i2c0_read_reg(uint8_t addr, uint8_t reg, uint8_t *p_value)
{
    return (i2c0_write_read(addr, &reg, 1u, p_value, 1u, I2C0_TIMEOUT_US));
}

EXPORT ER i2c0_read_burst(uint8_t addr, uint8_t reg, uint8_t *p_buf,
                          uint8_t len)
{
    /* Many accelerometers require the MSB of the register address set to
       auto-increment during a burst read.  The IMU driver passes the
       already-decorated register value, so nothing is added here. */
    return (i2c0_write_read(addr, &reg, 1u, p_buf, len, I2C0_TIMEOUT_US));
}

EXPORT ER i2c0_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2];

    tx_buf[0] = reg;
    tx_buf[1] = value;

    return (i2c0_write_read(addr, tx_buf, 2u, NULL, 0u, I2C0_TIMEOUT_US));
}

EXPORT uint32_t i2c0_error_count(void)
{
    return (error_count);
}

/* end of file */
