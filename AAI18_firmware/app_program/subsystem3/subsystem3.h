/**
 * @file subsystem3.h
 * @brief Buddy 3 facade; only the ADC sampler accesses ADC registers.
 */
#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H
#include "buddy3_config.h"
#include "line_sensor.h"
#include "barcode_decoder.h"

ER subsystem3_init(void);
ER subsystem3_start(void);
#if S3_BARCODE_BENCH
INT s3_barcode_bench_main(void);
#endif
#if APP_TEST
void subsystem3_test(void);
#endif
#endif /* SUBSYSTEM3_H */
