#ifndef AAI18_BUDDY1_STATE_H
#define AAI18_BUDDY1_STATE_H

#include <stdint.h>
#include "protocol.h"

int32_t buddy1_state_init(void);
void buddy1_set_mission(uint32_t state, uint32_t fault_flags);
void buddy1_set_motion(int32_t left_mm_s, int32_t right_mm_s,
                       int32_t left_count, int32_t right_count);
void buddy1_set_line(int32_t line_error);
void buddy1_set_barcode(int32_t decoded_code);
void buddy1_set_hump(int32_t peak_mm);
void buddy1_set_obstacle(int32_t distance_mm, int32_t width_mm);

/* A missed mutex does not wait for a driving task; the caller can retry. */
int buddy1_get_telemetry(buddy1_telemetry_t *sample);

/* Attach after Buddy 5 creates flg_events and flg_estop. */
void buddy1_attach_event_flags(int32_t events, uint32_t start_bit,
                               uint32_t stop_bit, int32_t estop,
                               uint32_t estop_bit);
void buddy1_receive_command(enum buddy1_command command);
uint32_t buddy1_start_sequence(void);
uint32_t buddy1_stop_sequence(void);
int buddy1_stop_latched(void);
/* Only the Vehicle Controller may acknowledge a stop after braking. */
void buddy1_acknowledge_stop(uint32_t observed_sequence);

#endif
