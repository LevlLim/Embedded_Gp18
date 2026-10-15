#ifndef AAI18_BUDDY1_MQTT_SERVICE_H
#define AAI18_BUDDY1_MQTT_SERVICE_H

#include <stdint.h>

typedef struct {
    uint32_t max_poll_gap_ms;
    uint32_t connect_attempts;
    uint32_t connection_failures;
    uint32_t telemetry_publish_errors;
    uint32_t heartbeat_publish_errors;
} buddy1_mqtt_diagnostics_t;

/* Called only by the existing CYW43/lwIP owner task. */
void buddy1_mqtt_poll(int network_ready, uint32_t now_ms);
int buddy1_mqtt_connected(void);
/* The status task may read this approximate cross-core snapshot. */
void buddy1_mqtt_get_diagnostics(buddy1_mqtt_diagnostics_t *diagnostics);

#endif
