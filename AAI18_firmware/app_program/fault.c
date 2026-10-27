/*
 * fault.c
 *
 * Group AAI18 -- shared fault bitmask implementation.
 *
 * The whole module exists so that no application code ever touches the
 * fault word directly.  Every path in or out goes through one of the five
 * functions below, which is what makes the "set only your own bits, clear
 * only from the Controller" discipline enforceable rather than advisory.
 *
 * APIs used: tk_cre_flg, tk_set_flg, tk_clr_flg, tk_wai_flg, tk_ref_flg
 */

#include "fault.h"

/* The event flag object.  Private: no other module may reference the ID. */
static ID h_flg_fault = (ID)0;

EXPORT ER fault_init(void)
{
    T_CFLG cflg = {0};

    /* TA_WMUL: several tasks may wait on this flag at once (Motion Control
       waits on the E-STOP mask, the Vehicle Controller on DEGRADE). */
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0u;              /* no fault at startup */

    h_flg_fault = tk_cre_flg(&cflg);

    return ((h_flg_fault < E_OK) ? (ER)h_flg_fault : E_OK);
}

EXPORT void fault_set(uint16_t bits)
{
    if (h_flg_fault > 0)
    {
        /* tk_set_flg ORs the pattern in; it is not a read-modify-write by
           the caller, so two tasks setting different bits cannot lose one
           another's update (risk R4). */
        (void)tk_set_flg(h_flg_fault, (UINT)bits);
    }
}

EXPORT void fault_set_isr(uint16_t bits)
{
    if (h_flg_fault > 0)
    {
        /* Same call, invoked from a handler.  The handler must then return
           immediately: no decoding, no blocking call, no reschedule. */
        (void)tk_set_flg(h_flg_fault, (UINT)bits);
    }
}

EXPORT void fault_clear(uint16_t bits)
{
    if (h_flg_fault > 0)
    {
        /* tk_clr_flg clears the bits present in the pattern given.  The
           kernel takes the complement, so pass the bits to KEEP. */
        (void)tk_clr_flg(h_flg_fault, (UINT)(~(UINT)bits));
    }
}

EXPORT uint16_t fault_get(void)
{
    T_RFLG   rflg = {0};
    uint16_t mask = 0u;

    if (h_flg_fault > 0)
    {
        if (tk_ref_flg(h_flg_fault, &rflg) >= E_OK)
        {
            mask = (uint16_t)(rflg.flgptn & FAULT_MASK_ALL);
        }
    }

    return (mask);
}

EXPORT bool fault_is_estop(void)
{
    return (0u != (fault_get() & FAULT_MASK_ESTOP));
}

EXPORT ER fault_wait_estop(uint16_t *p_bits, TMO timeout)
{
    UINT pattern = 0u;
    ER   ercd;

    if (h_flg_fault <= 0)
    {
        return (E_OBJ);
    }

    /*
     * TWF_ORW  : wake on ANY bit in the mask.
     * No TWF_CLR: the bits are NOT consumed by this wait.  If they were,
     * a second waiter would silently destroy a collision bit this waiter
     * had not yet acted on.  Only fault_clear() removes a bit.
     */
    ercd = tk_wai_flg(h_flg_fault, (UINT)FAULT_MASK_ESTOP, TWF_ORW,
                    &pattern, timeout);

    if ((ercd >= E_OK) && (NULL != p_bits))
    {
        *p_bits = (uint16_t)(pattern & FAULT_MASK_ESTOP);
    }

    return (ercd);
}

EXPORT const char *fault_name(uint16_t single_bit)
{
    const char *p_name;

    switch (single_bit)
    {
        case FAULT_COLLISION:
            p_name = "COLLISION";
        break;

        case FAULT_TILT:
            p_name = "TILT";
        break;

        case FAULT_OBSTACLE_STOP:
            p_name = "OBSTACLE_STOP";
        break;

        case FAULT_REMOTE_STOP:
            p_name = "REMOTE_STOP";
        break;

        case FAULT_LINE_LOST_HARD:
            p_name = "LINE_LOST_HARD";
        break;

        case FAULT_MOTOR_STALL:
            p_name = "MOTOR_STALL";
        break;

        case FAULT_ENCODER:
            p_name = "ENCODER_FAULT";
        break;

        case FAULT_IMU:
            p_name = "IMU_FAULT";
        break;

        case FAULT_NO_ECHO:
            p_name = "NO_ECHO";
        break;

        case FAULT_WIFI_LOSS:
            p_name = "WIFI_LOSS";
        break;

        default:
            p_name = "UNKNOWN";
        break;
    }

    return (p_name);
}

/* end of file */
