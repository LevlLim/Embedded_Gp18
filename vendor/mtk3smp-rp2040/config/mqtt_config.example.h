/* Copy to mqtt_config.h, fill in your LAN broker, then set the guard to 1.
 * The populated file is ignored by Git. Use a broker on the same Wi-Fi LAN.
 */
#ifndef AAI18_MQTT_CONFIG_H
#define AAI18_MQTT_CONFIG_H

#define AAI18_MQTT_CONFIG_SET 0
#if !AAI18_MQTT_CONFIG_SET
#error "Set broker details in config/mqtt_config.h and change AAI18_MQTT_CONFIG_SET to 1"
#endif

#define AAI18_MQTT_BROKER_IP   "192.168.1.100"
#define AAI18_MQTT_BROKER_PORT 1883U
#define AAI18_MQTT_CLIENT_ID   "aai18-pico-w"
#define AAI18_MQTT_USER        NULL
#define AAI18_MQTT_PASSWORD    NULL
#define AAI18_TOPIC_COMMAND    "aai18/robot/command"
#define AAI18_TOPIC_TELEMETRY  "aai18/robot/telemetry"
#define AAI18_TOPIC_HEARTBEAT  "aai18/robot/heartbeat"

#endif
