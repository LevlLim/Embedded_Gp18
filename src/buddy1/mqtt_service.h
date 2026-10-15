#ifndef AAI18_BUDDY1_MQTT_SERVICE_H
#define AAI18_BUDDY1_MQTT_SERVICE_H

#include <stdint.h>

/* Called only by the existing CYW43/lwIP owner task. */
void buddy1_mqtt_poll(int network_ready, uint32_t now_ms);
int buddy1_mqtt_connected(void);

#endif
