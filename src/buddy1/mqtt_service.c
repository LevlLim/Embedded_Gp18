#include "mqtt_service.h"

#include <stddef.h>
#include <string.h>

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"

#include "mqtt_config.h"
#include "protocol.h"
#include "state.h"

#define BUDDY1_TELEMETRY_PERIOD_MS 200U
#define BUDDY1_HEARTBEAT_PERIOD_MS 1000U
#define BUDDY1_RETRY_PERIOD_MS     1000U
#define BUDDY1_COMMAND_CAPACITY    8U
#define BUDDY1_JSON_CAPACITY     384U

static mqtt_client_t *client;
static struct mqtt_connect_client_info_t connection_info = {
    .client_id = AAI18_MQTT_CLIENT_ID,
    .client_user = AAI18_MQTT_USER,
    .client_pass = AAI18_MQTT_PASSWORD,
    .keep_alive = 15U,
    .will_topic = AAI18_TOPIC_HEARTBEAT,
    .will_msg = "offline",
    .will_qos = 0U,
    .will_retain = 0U
};
static volatile int subscribed;
static int connecting;
static uint32_t next_attempt_ms;
static uint32_t last_telemetry_ms;
static uint32_t last_heartbeat_ms;
static uint32_t publish_sequence;
static char command_buffer[BUDDY1_COMMAND_CAPACITY];
static uint32_t expected_command_length;
static uint32_t command_length;
static int command_topic;

static void on_incoming_topic(void *argument, const char *topic,
                              u32_t total_length)
{
    (void)argument;
    command_topic = (topic != NULL) &&
                    (strcmp(topic, AAI18_TOPIC_COMMAND) == 0) &&
                    (total_length <= BUDDY1_COMMAND_CAPACITY);
    expected_command_length = total_length;
    command_length = 0U;
}

static void on_incoming_data(void *argument, const u8_t *data,
                             u16_t length, u8_t flags)
{
    (void)argument;
    if (!command_topic || (data == NULL) ||
        ((uint32_t)length > BUDDY1_COMMAND_CAPACITY - command_length)) {
        command_topic = 0;
        return;
    }
    memcpy(&command_buffer[command_length], data, length);
    command_length += length;
    if ((flags & MQTT_DATA_FLAG_LAST) != 0U) {
        if (command_length == expected_command_length) {
            buddy1_receive_command(buddy1_parse_command(
                (const uint8_t *)command_buffer, command_length));
        }
        command_topic = 0;
    }
}

static void on_subscribed(void *argument, err_t result)
{
    (void)argument;
    if (result == ERR_OK) {
        subscribed = 1;
    } else {
        subscribed = 0;
        mqtt_disconnect(client);
        next_attempt_ms = sys_now() + BUDDY1_RETRY_PERIOD_MS;
    }
}

static void on_connection(mqtt_client_t *mqtt, void *argument,
                          mqtt_connection_status_t status)
{
    err_t result;

    (void)argument;
    connecting = 0;
    subscribed = 0;
    if (status != MQTT_CONNECT_ACCEPTED) {
        next_attempt_ms = sys_now() + BUDDY1_RETRY_PERIOD_MS;
        return;
    }
    mqtt_set_inpub_callback(mqtt, on_incoming_topic, on_incoming_data, NULL);
    result = mqtt_subscribe(mqtt, AAI18_TOPIC_COMMAND, 0U,
                            on_subscribed, NULL);
    if (result != ERR_OK) {
        mqtt_disconnect(mqtt);
        next_attempt_ms = sys_now() + BUDDY1_RETRY_PERIOD_MS;
    }
}

static void publish_telemetry(uint32_t now_ms)
{
    buddy1_telemetry_t sample;
    char json[BUDDY1_JSON_CAPACITY];
    uint32_t length;

    last_telemetry_ms = now_ms;
    if (!buddy1_get_telemetry(&sample)) {
        return;
    }
    sample.uptime_ms = now_ms;
    sample.sequence = publish_sequence;
    length = buddy1_format_telemetry(json, sizeof(json), &sample);
    if (length == 0U) {
        return;
    }
    if (mqtt_publish(client, AAI18_TOPIC_TELEMETRY, json, (u16_t)length,
                     0U, 0U, NULL, NULL) == ERR_OK) {
        publish_sequence++;
    }
}

static void publish_heartbeat(uint32_t now_ms)
{
    char payload[24];
    uint32_t length;

    last_heartbeat_ms = now_ms;
    length = buddy1_format_u32(payload, sizeof(payload), now_ms);
    if (length != 0U) {
        (void)mqtt_publish(client, AAI18_TOPIC_HEARTBEAT, payload,
                           (u16_t)length, 0U, 0U, NULL, NULL);
    }
}

void buddy1_mqtt_poll(int network_ready, uint32_t now_ms)
{
    ip_addr_t broker;
    err_t result;

    if (!network_ready) {
        if (client != NULL) {
            mqtt_disconnect(client);
        }
        connecting = 0;
        subscribed = 0;
        next_attempt_ms = now_ms + BUDDY1_RETRY_PERIOD_MS;
        return;
    }

    if ((client == NULL) &&
        ((int32_t)(now_ms - next_attempt_ms) < 0)) {
        return;
    }
    if (client == NULL) {
        client = mqtt_client_new();
        if (client == NULL) {
            next_attempt_ms = now_ms + BUDDY1_RETRY_PERIOD_MS;
            return;
        }
    }

    if (!mqtt_client_is_connected(client)) {
        if (connecting) {
            return;
        }
        if ((int32_t)(now_ms - next_attempt_ms) < 0) {
            return;
        }
        next_attempt_ms = now_ms + BUDDY1_RETRY_PERIOD_MS;
        if (!ipaddr_aton(AAI18_MQTT_BROKER_IP, &broker)) {
            return;
        }
        result = mqtt_client_connect(client, &broker,
                                     AAI18_MQTT_BROKER_PORT,
                                     on_connection, NULL, &connection_info);
        if (result != ERR_OK) {
            return;
        }
        connecting = 1;
        return;
    }

    if (!subscribed) {
        return;
    }
    if ((uint32_t)(now_ms - last_telemetry_ms) >=
        BUDDY1_TELEMETRY_PERIOD_MS) {
        publish_telemetry(now_ms);
    }
    if ((uint32_t)(now_ms - last_heartbeat_ms) >=
        BUDDY1_HEARTBEAT_PERIOD_MS) {
        publish_heartbeat(now_ms);
    }
}

int buddy1_mqtt_connected(void)
{
    return subscribed;
}
