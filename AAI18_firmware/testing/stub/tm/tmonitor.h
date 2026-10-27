#ifndef STUB_TMONITOR_H
#define STUB_TMONITOR_H
#include <tk/tkernel.h>
void tm_printf(UB *fmt, ...);
INT  tm_sprintf(UB *str, const UB *format, ...);
INT  tm_putstring(const UB *buff);
#endif
