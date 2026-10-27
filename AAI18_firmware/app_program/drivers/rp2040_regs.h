/*
 * rp2040_regs.h
 *
 * Group AAI18 -- RP2040 register definitions the template does not supply.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * The project template's BSP (include/bsp/.../libbsp.h) provides GPIO and
 * PWM helpers only.  It has no ADC, no I2C and no GPIO-interrupt support,
 * and the application include path (-I../include -I../config
 * -I../kernel/knlinc, see build_make/makefile) does not reach the Pico SDK
 * hardware headers.  So hardware/adc.h, hardware/i2c.h and hardware/gpio.h
 * are NOT available to application code in this build.
 *
 * The three peripherals the design needs -- the shared ADC (section 3),
 * I2C0 for the IMU (section 6.4) and the single IO_BANK0 dispatcher
 * (section 4) -- are therefore written against the registers directly, in
 * the same style as the template's own lib/libbsp/.../libbsp_pwm.c.
 *
 * Addresses below are from the RP2040 datasheet.  Those the template
 * already defines (GPIO_BASE, RESETS_*, out_w/in_w) come from
 * include/sys/sysdepend/cpu/rp2040/sysdef.h and are not repeated here.
 *
 * VERIFY BEFORE TRUSTING: these offsets were written from the datasheet,
 * not read back from silicon.  The first bring-up step for each driver is
 * the register dump in its *_selftest() function.
 */

#ifndef RP2040_REGS_H
#define RP2040_REGS_H

#include <tk/tkernel.h>
#include <sys/machine.h>

/* ---------------------------------------------------------------------
 * ADC (RP2040 datasheet section 4.9)
 * --------------------------------------------------------------------- */
#define ADC_BASE            (0x4004C000u)
#define ADC_CS              (ADC_BASE + 0x00u)
#define ADC_RESULT          (ADC_BASE + 0x04u)
#define ADC_FCS             (ADC_BASE + 0x08u)
#define ADC_FIFO            (ADC_BASE + 0x0Cu)
#define ADC_DIV             (ADC_BASE + 0x10u)

#define ADC_CS_EN           (1u << 0u)
#define ADC_CS_TS_EN        (1u << 1u)
#define ADC_CS_START_ONCE   (1u << 2u)
#define ADC_CS_START_MANY   (1u << 3u)
#define ADC_CS_READY        (1u << 8u)
#define ADC_CS_ERR          (1u << 9u)
#define ADC_CS_AINSEL_SHIFT (12u)
#define ADC_CS_AINSEL_MASK  (0x07u << 12u)
#define ADC_CS_RROBIN_SHIFT (16u)
#define ADC_CS_RROBIN_MASK  (0x1Fu << 16u)

#define ADC_FCS_EN          (1u << 0u)
#define ADC_FCS_SHIFT       (1u << 1u)   /* right-shift result to 8 bits */
#define ADC_FCS_ERR         (1u << 2u)
#define ADC_FCS_DREQ_EN     (1u << 3u)
#define ADC_FCS_EMPTY       (1u << 8u)
#define ADC_FCS_FULL        (1u << 9u)
#define ADC_FCS_UNDER       (1u << 10u)
#define ADC_FCS_OVER        (1u << 11u)
#define ADC_FCS_LEVEL_SHIFT (16u)
#define ADC_FCS_LEVEL_MASK  (0x0Fu << 16u)

#define ADC_FIFO_ERR        (1u << 15u)
#define ADC_FIFO_VAL_MASK   (0x0FFFu)

/* ---------------------------------------------------------------------
 * I2C0 (RP2040 datasheet section 4.3; Synopsys DW_apb_i2c)
 * --------------------------------------------------------------------- */
#define I2C0_BASE               (0x40044000u)
#define I2C_CON                 (I2C0_BASE + 0x00u)
#define I2C_TAR                 (I2C0_BASE + 0x04u)
#define I2C_DATA_CMD            (I2C0_BASE + 0x10u)
#define I2C_SS_SCL_HCNT         (I2C0_BASE + 0x14u)
#define I2C_SS_SCL_LCNT         (I2C0_BASE + 0x18u)
#define I2C_FS_SCL_HCNT         (I2C0_BASE + 0x1Cu)
#define I2C_FS_SCL_LCNT         (I2C0_BASE + 0x20u)
#define I2C_INTR_STAT           (I2C0_BASE + 0x2Cu)
#define I2C_RAW_INTR_STAT       (I2C0_BASE + 0x34u)
#define I2C_RX_TL               (I2C0_BASE + 0x38u)
#define I2C_TX_TL               (I2C0_BASE + 0x3Cu)
#define I2C_CLR_INTR            (I2C0_BASE + 0x40u)
#define I2C_CLR_TX_ABRT         (I2C0_BASE + 0x54u)
#define I2C_ENABLE              (I2C0_BASE + 0x6Cu)
#define I2C_STATUS              (I2C0_BASE + 0x70u)
#define I2C_TXFLR               (I2C0_BASE + 0x74u)
#define I2C_RXFLR               (I2C0_BASE + 0x78u)
#define I2C_CLR_STOP_DET        (I2C0_BASE + 0x60u)
#define I2C_TX_ABRT_SOURCE      (I2C0_BASE + 0x80u)
#define I2C_ENABLE_STATUS       (I2C0_BASE + 0x9Cu)
#define I2C_FS_SPKLEN           (I2C0_BASE + 0xA0u)

#define I2C_CON_MASTER_MODE     (1u << 0u)
#define I2C_CON_SPEED_FAST      (2u << 1u)
#define I2C_CON_7BIT_ADDR       (0u << 3u)
#define I2C_CON_RESTART_EN      (1u << 5u)
#define I2C_CON_SLAVE_DISABLE   (1u << 6u)
#define I2C_CON_TX_EMPTY_CTRL   (1u << 8u)

#define I2C_DATA_CMD_READ       (1u << 8u)
#define I2C_DATA_CMD_STOP       (1u << 9u)
#define I2C_DATA_CMD_RESTART    (1u << 10u)

#define I2C_RAW_TX_ABRT         (1u << 6u)
#define I2C_RAW_STOP_DET        (1u << 9u)

#define I2C_STATUS_ACTIVITY     (1u << 0u)   /* controller busy   */
#define I2C_STATUS_TFNF         (1u << 1u)   /* TX FIFO not full  */
#define I2C_STATUS_RFNE         (1u << 3u)   /* RX FIFO not empty */

#define I2C_ENABLE_STATUS_EN    (1u << 0u)
#define I2C_FIFO_DEPTH          (16u)

/* GPIO function select value for I2C on GP16/GP17 */
#define GPIO_FUNC_I2C           (3u)

/* ---------------------------------------------------------------------
 * IO_BANK0 interrupt (RP2040 datasheet section 2.19.6)
 *
 * All GPIO edge interrupts -- both encoder A channels and ECHO -- arrive
 * on ONE processor interrupt line.  This is why the design calls for a
 * single short dispatcher rather than three separate handlers.
 * --------------------------------------------------------------------- */
/* IO_BANK0_BASE comes from the template's sysdef.h (0x40014000). */
#define IO_BANK0_INTR0          (IO_BANK0_BASE + 0x0F0u)
#define IO_BANK0_PROC0_INTE0    (IO_BANK0_BASE + 0x100u)
#define IO_BANK0_PROC0_INTS0    (IO_BANK0_BASE + 0x120u)

/* Each GPIO occupies 4 bits in these registers; 8 GPIOs per 32-bit word. */
#define IO_BANK0_REG_INDEX(gpio)    ((gpio) / 8u)
#define IO_BANK0_BIT_SHIFT(gpio)    (((gpio) % 8u) * 4u)

#define GPIO_EDGE_LOW           (1u << 2u)
#define GPIO_EDGE_HIGH          (1u << 3u)
#define GPIO_EDGE_BOTH          (GPIO_EDGE_LOW | GPIO_EDGE_HIGH)

/* RP2040 interrupt number for IO_IRQ_BANK0 */
#define INTNO_IO_BANK0          (13)
#define INTPRI_IO_BANK0         (2)

/* ---------------------------------------------------------------------
 * TIMER (RP2040 datasheet section 4.6) -- free-running microsecond timer.
 * Used for ECHO pulse timing and ADC sample timestamps.  Reading TIMELR
 * latches TIMEHR, so TIMELR must be read first; for our purposes the low
 * 32 bits alone give 71 minutes of range, which is ample for one run.
 * --------------------------------------------------------------------- */
/* TIMER_BASE and TIMER_TIMERAWL come from the template's sysdef.h; they
   are not redefined here (doing so produced redefinition warnings). */

/* ---------------------------------------------------------------------
 * PADS_BANK0 pad control (datasheet section 2.19.6.3).  PADS_BANK0_BASE
 * comes from the template's sysdef.h.
 * --------------------------------------------------------------------- */
#define PADS_GPIO(gpio)         (PADS_BANK0_BASE + 0x04u + (4u * (gpio)))
#define PADS_OD                 (1u << 7u)
#define PADS_IE                 (1u << 6u)
#define PADS_PUE                (1u << 3u)
#define PADS_PDE                (1u << 2u)

/* ---------------------------------------------------------------------
 * PWM slice divider (datasheet section 4.5).  The template's pwm_set_wrap
 * never touches DIV, which resets to 1.0, so a slice runs at the full
 * 125 MHz system clock unless the divider is set here.  DIV is 8.4 fixed
 * point: integer part in bits 11:4.
 * --------------------------------------------------------------------- */
#define PWM_SLICE_OF(gpio)      (((gpio) >> 1u) & 0x07u)
#define PWM_SLICE_DIV(slice)    (PWM_BASE + PWM_CHx_DIV + (0x14u * (slice)))
#define PWM_DIV_INT_SHIFT       (4u)

/* ---------------------------------------------------------------------
 * PWM block reset.  The kernel releases PWM from reset only when it uses
 * the block as its physical timer (USE_PTMR), which this configuration
 * does not.  Until released, every PWM register write is ignored: the
 * motors, servo and buzzer stay silent with no error.  Release (never
 * assert) the reset, so calling this twice is harmless.
 * --------------------------------------------------------------------- */
#define RP2040_RELEASE_PWM()                                            \
    do                                                                  \
    {                                                                   \
        clr_w(RESETS_RESET, RESETS_RESET_PWM);                          \
        while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))      \
        {                                                               \
            /* Wait: a few clock cycles, once, at init. */              \
        }                                                               \
    } while (0)

/* Microseconds since boot, 32-bit, wraps every ~71 minutes.
   TIMERAWL is read without latching, which is what we want in an ISR. */
#define RP2040_NOW_US()         (in_w(TIMER_TIMERAWL))

#endif /* RP2040_REGS_H */

/* end of file */
