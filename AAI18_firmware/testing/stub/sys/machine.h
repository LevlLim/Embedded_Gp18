#ifndef STUB_MACHINE_H
#define STUB_MACHINE_H
#define CPU_RP2040 1
#define RESETS_RESET      0x4000C000u
#define RESETS_RESET_DONE 0x4000C008u
#define RESETS_RESET_ADC  0x00000001u
#define RESETS_RESET_I2C0 0x00000008u
#define RESETS_RESET_PWM  0x00004000u
/* Addresses the template's sysdef.h provides to the real build. */
#define IO_BANK0_BASE     0x40014000u
#define PADS_BANK0_BASE   0x4001c000u
#define TIMER_BASE        0x40054000u
#define TIMER_TIMERAWL    (TIMER_BASE + 0x28u)
#define PWM_BASE          0x40050000u
#define PWM_CHx_DIV       0x04u
#endif
