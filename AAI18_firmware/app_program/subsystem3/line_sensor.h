/**
 * @file line_sensor.h
 * @brief Line sensing and transactional three-channel calibration.
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H
#include "../app.h"
#include "../msg_defs.h"

bool s3_read_line_word(uint32_t * p_word);
msg_line_state_t s3_get_line_state(void);
/* Fresh, calibrated edge observations. Zero flags means both WHITE, which
 * is aligned OR missing track: these sensors cannot distinguish those cases.
 * JUNCTION means sustained both-black, not a proof of a T versus a crossing.
 * This level clears when either sensor clears; the Controller must latch an
 * encounter and consume its remembered barcode only once per manoeuvre. */
#define S3_LINE_LEFT_BLACK   (0x01U)
#define S3_LINE_RIGHT_BLACK  (0x02U)
#define S3_LINE_JUNCTION     (0x04U)
bool s3_read_line_sensors(uint8_t * p_flags);
void s3_set_active(bool b_on);
bool s3_barcode_active(void);
ER s3_calibrate(void);
/* Same ABI/order as the former side/centre getter: GP26, then GP27. */
void s3_get_thresholds(uint16_t * p_left, uint16_t * p_right);
void line_task(INT stacd, void * p_exinf);
uint16_t s3_line_max_us(void);

#endif /* LINE_SENSOR_H */

