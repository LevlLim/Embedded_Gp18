/*
 * host_stubs.c
 *
 * Link-only stand-ins for the kernel, the drivers and the other
 * subsystems, so the PURE helpers inside barcode_decoder.c,
 * obstacle_scan.c and imu_terrain.c can be unit tested on a PC.
 *
 * None of these is ever called by test_host.c: the tasks and hardware
 * paths in those files are not exercised here.  Each stub returns a
 * harmless failure value so an accidental call is visible, not silent.
 */

#include "app.h"
#include "subsystem3/adc_sampler.h"
#include "drivers/gpio_irq.h"
#include "subsystem4/i2c0.h"
#include "fault.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#include "subsystem5/subsystem5.h"

bool adc_sampler_read(uint8_t channel, adc_sample_t *p_out)
{
    (void)channel;
    (void)p_out;
    return (false);
}
uint8_t adc_sampler_since(uint32_t after_us, adc_sample_t *p_out,
                          uint8_t max_out)
{
    (void)after_us;
    (void)p_out;
    (void)max_out;
    return (0u);
}
uint32_t app_now_ms(void) { return (0u); }
void tm_printf(UB *fmt, ...) { (void)fmt; }
uint32_t app_wait_period(void) { return (0u); }
void fault_set(uint16_t bits) { (void)bits; }
bool gpio_irq_read_echo(gpio_irq_echo_t *p_out)
{
    (void)p_out;
    return (false);
}
ER gpio_set_pin(UINT no, UINT mode) { (void)no; (void)mode; return (E_OBJ); }
ER gpio_set_val(UINT no, UINT val) { (void)no; (void)val; return (E_OBJ); }
ER i2c0_read_burst(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint8_t len)
{
    (void)addr;
    (void)reg;
    (void)p_buf;
    (void)len;
    return (E_OBJ);
}
ER i2c0_read_reg(uint8_t addr, uint8_t reg, uint8_t *p_value)
{
    (void)addr;
    (void)reg;
    (void)p_value;
    return (E_OBJ);
}
ER pwm_set_cc(UINT no, UW cc) { (void)no; (void)cc; return (E_OBJ); }
ER pwm_set_enabled(UINT no, BOOL enable)
{
    (void)no;
    (void)enable;
    return (E_OBJ);
}
ER pwm_set_pin(UINT no) { (void)no; return (E_OBJ); }
ER pwm_set_wrap(UINT no, UW wrap) { (void)no; (void)wrap; return (E_OBJ); }
int32_t s2_get_distance_mm(void) { return (0); }
s2_state_t s2_get_state(void) { return (M_IDLE); }
bool s3_barcode_active(void) { return (false); }
bool s5_is_running(void) { return (false); }
void s5_post_event(uint32_t event_bits) { (void)event_bits; }
ID tk_cre_mbf(T_CMBF *p_cmbf) { (void)p_cmbf; return (E_OBJ); }
ER tk_dly_tsk(INT dlytim) { (void)dlytim; return (E_OK); }
INT tk_rcv_mbf(ID mbfid, void *p_msg, TMO tmout)
{
    (void)mbfid;
    (void)p_msg;
    (void)tmout;
    return (E_OBJ);
}
ER tk_snd_mbf(ID mbfid, void *p_msg, INT msgsz, TMO tmout)
{
    (void)mbfid;
    (void)p_msg;
    (void)msgsz;
    (void)tmout;
    return (E_OBJ);
}

/* end of file */
