#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "state.h"

#if TM_WIFI_CYW43
#include "cyw43_utk.h"

/* The course radio status contains diagnostic packet rings and is larger
 * than this task's stack. Keep its snapshot in static memory. */
static T_CYW43_UTK_STATUS radio_status_snapshot;
#endif
#if TM_WIFI_MQTT
#include "mqtt_service.h"
#endif

#define BUDDY1_STATUS_PRIORITY 15
#define BUDDY1_STATUS_STACK    2048

static void status_task(INT start_code, void *extra)
{
    (void)start_code;
    (void)extra;
    for (;;) {
#if TM_WIFI_CYW43
        cyw43_utk_get_status(&radio_status_snapshot);
        tm_printf((UB *)"[aai18] radio ready=%u scan=%u link=%u dhcp=%u\n",
                  radio_status_snapshot.ready,
                  radio_status_snapshot.scan_complete,
                  radio_status_snapshot.link_up,
                  radio_status_snapshot.dhcp_complete);
#if TM_WIFI_MQTT
        tm_printf((UB *)"[aai18] join started=%u done=%u request=%d result=%d link_status=%d networks=%u\n",
                  radio_status_snapshot.join_started,
                  radio_status_snapshot.join_complete,
                  radio_status_snapshot.join_request_result,
                  radio_status_snapshot.join_result,
                  radio_status_snapshot.link_status,
                  radio_status_snapshot.scan_count);
#endif
#endif
#if TM_WIFI_MQTT
        tm_printf((UB *)"[aai18] MQTT=%s start_seq=%u remote_stop=%d\n",
                  buddy1_mqtt_connected() ? "connected" : "waiting",
                  buddy1_start_sequence(),
                  buddy1_stop_latched());
#else
        tm_printf((UB *)"[aai18] project firmware ready; MQTT disabled\n");
#endif
        (void)tk_dly_tsk(5000);
    }
}

EXPORT INT usermain(void)
{
    T_CTSK task_config = {0};
    ID task;

    if (buddy1_state_init() != E_OK) {
        tm_printf((UB *)"[aai18] telemetry mutex creation failed\n");
        return 1;
    }

    task_config.tskatr = TA_HLNG | TA_RNG3;
    task_config.task = status_task;
    task_config.itskpri = BUDDY1_STATUS_PRIORITY;
    task_config.stksz = BUDDY1_STATUS_STACK;
    task = tk_cre_tsk(&task_config);
    if (task <= 0) {
        tm_printf((UB *)"[aai18] status task creation failed: %d\n", task);
        return 1;
    }
    if (tk_sta_tsk(task, 0) != E_OK) {
        tm_printf((UB *)"[aai18] status task start failed\n");
        return 1;
    }
    tm_printf((UB *)"[aai18] safe project firmware booted\n");
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
