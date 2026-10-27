#ifndef STUB_LIBBSP_H
#define STUB_LIBBSP_H
#include <tk/tkernel.h>
#define GPIO_MODE_OUT 1
#define GPIO_MODE_IN  0
#define GPIO_NUM 30
#define GPIO_CTRL(n) (0x40014004u + ((n)*8u))
ER gpio_set_pin(UINT no, UINT mode);
ER gpio_set_val(UINT no, UINT val);
UINT gpio_get_val(UINT no);
ER pwm_set_pin(UINT no);
ER pwm_set_wrap(UINT no, UW wrap);
ER pwm_set_cc(UINT no, UW cc);
ER pwm_set_enabled(UINT no, BOOL enable);
#endif
