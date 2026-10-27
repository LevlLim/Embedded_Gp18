/*
 * gpio_irq.c
 *
 * Group AAI18 -- single IO_BANK0 dispatcher for encoders and ECHO.
 *
 * APIs used: tk_def_int, DI/EI (interrupt disable/enable), out_w/in_w/set_w
 */

#include "gpio_irq.h"
#include "rp2040_regs.h"
#include "../fault.h"

/* ---------------------------------------------------------------------
 * Private data.  All of it is written by the dispatcher and read by tasks.
 * --------------------------------------------------------------------- */
static gpio_irq_encoders_t encoders;
static gpio_irq_echo_t     echo;

static volatile uint32_t max_duration_us = 0u;

/* ---------------------------------------------------------------------
 * Private functions
 * --------------------------------------------------------------------- */

/*
 * Enable one edge-interrupt source for a GPIO on processor 0.
 */
static void enable_pin_irq(uint32_t gpio, uint32_t edge_mask)
{
    uint32_t reg   = IO_BANK0_PROC0_INTE0 + (4u * IO_BANK0_REG_INDEX(gpio));
    uint32_t shift = IO_BANK0_BIT_SHIFT(gpio);

    /* Clear any stale latched event for this pin first.  INTR is
       write-1-to-clear, so a PLAIN write of just this pin's bits is used,
       as the Pico SDK does: the atomic set alias is not guaranteed to
       leave the other pins' latched edges alone on a W1C register. */
    out_w(IO_BANK0_INTR0 + (4u * IO_BANK0_REG_INDEX(gpio)),
          (edge_mask << shift));

    set_w(reg, (edge_mask << shift));
}

/*
 * Read the latched status for a GPIO and clear it.
 * Returns the edge bits that fired for this pin.
 */
static uint32_t take_pin_events(uint32_t gpio)
{
    uint32_t idx   = IO_BANK0_REG_INDEX(gpio);
    uint32_t shift = IO_BANK0_BIT_SHIFT(gpio);
    uint32_t sts;
    uint32_t bits;

    sts  = in_w(IO_BANK0_PROC0_INTS0 + (4u * idx));
    bits = (sts >> shift) & 0x0Fu;

    if (0u != bits)
    {
        /* Writing 1 to INTR clears only the bits written (W1C), so a
           plain write never disturbs another pin's pending edge. */
        out_w(IO_BANK0_INTR0 + (4u * idx), (bits << shift));
    }

    return (bits);
}

/*
 * THE DISPATCHER.
 *
 * Kept deliberately short: count, timestamp, clear, return.  Every branch
 * here is a handful of instructions.  Nothing in this function may block,
 * allocate, print, or call a kernel service that can reschedule.
 */
static void io_bank0_isr(UINT intno)
{
    uint32_t start_us;
    uint32_t now_us;
    uint32_t bits;
    uint32_t elapsed;
    uint8_t  phase_b;

    (void)intno;

    start_us = RP2040_NOW_US();

    /* ---- left encoder phase A ---- */
    bits = take_pin_events(PIN_ENC_L_A);
    if (0u != bits)
    {
        phase_b = (uint8_t)gpio_get_val(PIN_ENC_L_B);

        /* Quadrature: on an A edge, the B level gives direction. */
        if (0u != (bits & GPIO_EDGE_HIGH))
        {
            encoders.count_left += (0u == phase_b) ? 1 : -1;
        }
        else
        {
            encoders.count_left += (0u != phase_b) ? 1 : -1;
        }

        encoders.edges_left   = encoders.edges_left + 1u;
        encoders.last_edge_us = start_us;
    }

    /* ---- right encoder phase A ---- */
    bits = take_pin_events(PIN_ENC_R_A);
    if (0u != bits)
    {
        phase_b = (uint8_t)gpio_get_val(PIN_ENC_R_B);

        if (0u != (bits & GPIO_EDGE_HIGH))
        {
            encoders.count_right += (0u == phase_b) ? 1 : -1;
        }
        else
        {
            encoders.count_right += (0u != phase_b) ? 1 : -1;
        }

        encoders.edges_right  = encoders.edges_right + 1u;
        encoders.last_edge_us = start_us;
    }

    /* ---- ultrasonic ECHO ---- */
    bits = take_pin_events(PIN_US_ECHO);
    if (0u != bits)
    {
        now_us = RP2040_NOW_US();

        if (0u != (bits & GPIO_EDGE_HIGH))
        {
            echo.rise_us   = now_us;
            echo.b_pending = true;
        }
        else if (echo.b_pending)
        {
            /* Unsigned subtraction is correct across the 71-minute wrap. */
            echo.width_us  = now_us - echo.rise_us;
            echo.b_pending = false;

            /* seq advances LAST so a reader can detect a torn read. */
            echo.seq = echo.seq + 1u;
        }
        else
        {
            /* Falling edge with no matching rise: ignore. */
        }
    }

    /* Record the worst-case dispatcher cost.  This is the measurement
       that closes risk R8 and feeds the NFR8 CPU utilisation argument. */
    elapsed = RP2040_NOW_US() - start_us;
    if (elapsed > max_duration_us)
    {
        max_duration_us = elapsed;
    }
}

/* ---------------------------------------------------------------------
 * Public functions
 * --------------------------------------------------------------------- */

EXPORT ER gpio_irq_init(void)
{
    T_DINT dint = {0};
    ER     ercd;

    encoders.count_left   = 0;
    encoders.count_right  = 0;
    encoders.edges_left   = 0u;
    encoders.edges_right  = 0u;
    encoders.last_edge_us = 0u;

    echo.rise_us   = 0u;
    echo.width_us  = 0u;
    echo.seq       = 0u;
    echo.b_pending = false;

    /* Encoder and echo pins are inputs. */
    (void)gpio_set_pin(PIN_ENC_L_A, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_ENC_L_B, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_ENC_R_A, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_ENC_R_B, GPIO_MODE_IN);
    (void)gpio_set_pin(PIN_US_ECHO, GPIO_MODE_IN);

    /* Install the one handler for the one IO_BANK0 vector. */
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)io_bank0_isr;

    ercd = tk_def_int((UINT)INTNO_IO_BANK0, &dint);
    if (ercd < E_OK)
    {
        return (ercd);
    }

    enable_pin_irq(PIN_ENC_L_A, GPIO_EDGE_BOTH);
    enable_pin_irq(PIN_ENC_R_A, GPIO_EDGE_BOTH);
    enable_pin_irq(PIN_US_ECHO, GPIO_EDGE_BOTH);

    EnableInt((UINT)INTNO_IO_BANK0, INTPRI_IO_BANK0);

    return (E_OK);
}

EXPORT void gpio_irq_read_encoders(gpio_irq_encoders_t *p_out)
{
    UINT imask;

    if (NULL == p_out)
    {
        return;
    }

    /*
     * Five fields must be mutually consistent, and volatile alone does not
     * give that (the Week 6 review made exactly this point).  A short
     * interrupt-disabled copy does.  The window is a few instructions,
     * which is acceptable even in the 10 ms Motion Control path.
     */
    DI(imask);
    p_out->count_left   = encoders.count_left;
    p_out->count_right  = encoders.count_right;
    p_out->edges_left   = encoders.edges_left;
    p_out->edges_right  = encoders.edges_right;
    p_out->last_edge_us = encoders.last_edge_us;
    EI(imask);
}

EXPORT bool gpio_irq_read_echo(gpio_irq_echo_t *p_out)
{
    uint32_t seq_before;
    uint32_t seq_after;
    bool     b_ok = false;

    if (NULL == p_out)
    {
        return (false);
    }

    seq_before = echo.seq;
    if (0u != seq_before)
    {
        p_out->rise_us  = echo.rise_us;
        p_out->width_us = echo.width_us;
        p_out->seq      = seq_before;
        seq_after       = echo.seq;

        /* If the dispatcher completed another pulse while we copied, the
           copy may be mixed; the caller simply retries next cycle. */
        b_ok = (seq_before == seq_after);
    }

    return (b_ok);
}

EXPORT uint32_t gpio_irq_max_duration_us(void)
{
    return (max_duration_us);
}

EXPORT void gpio_irq_reset_stats(void)
{
    max_duration_us = 0u;
}

/* end of file */
