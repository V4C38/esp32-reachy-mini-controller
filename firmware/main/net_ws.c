#include "net_ws.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "net_ws";
static esp_websocket_client_handle_t s_client;
static net_ws_status_t s_status;
static char s_uri[128];
static int s_req_id = 1;
static int64_t s_disconnect_us;
static int64_t s_last_reconnect_us;

static void set_phase(net_ws_phase_t phase)
{
    if (s_status.phase == phase) return;
    net_ws_phase_t prev = s_status.phase;
    s_status.phase = phase;
    s_status.linked = (phase == NET_WS_LINKED);
    if (phase == NET_WS_LINKED) {
        if (prev != NET_WS_LINKED) {
            s_status.reconnects++;
            int64_t now = esp_timer_get_time();
            if (s_disconnect_us > 0) {
                ESP_LOGI(TAG, "connected (down for %lld ms, reconnects=%lu)",
                         (long long)((now - s_disconnect_us) / 1000),
                         (unsigned long)s_status.reconnects);
            } else {
                ESP_LOGI(TAG, "connected (reconnects=%lu)",
                         (unsigned long)s_status.reconnects);
            }
            s_disconnect_us = 0;
            s_last_reconnect_us = now;
        }
    } else if (phase == NET_WS_CONNECTING) {
        ESP_LOGI(TAG, "connecting");
    } else {
        if (s_disconnect_us == 0) s_disconnect_us = esp_timer_get_time();
        s_status.robot_ok = false;
        ESP_LOGW(TAG, "disconnected (send_fails=%lu)",
                 (unsigned long)s_status.send_fails);
    }
}

static void on_event(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        set_phase(NET_WS_LINKED);
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "disconnected event (error_type=%d sock_errno=%d)",
                 (int)data->error_handle.error_type,
                 data->error_handle.esp_transport_sock_errno);
        set_phase(NET_WS_DOWN);
        break;
    case WEBSOCKET_EVENT_ERROR:
        /* Transient write/poll errors fire here while the client is still
         * reconnecting — stay CONNECTING if we still have a client, do not
         * thrash UI to disconnected until DISCONNECTED arrives. */
        if (s_client && s_status.phase == NET_WS_LINKED) {
            set_phase(NET_WS_CONNECTING);
        }
        ESP_LOGW(TAG, "error (phase=%d error_type=%d sock_errno=%d)",
                 (int)s_status.phase,
                 (int)data->error_handle.error_type,
                 data->error_handle.esp_transport_sock_errno);
        break;
    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == WS_TRANSPORT_OPCODES_TEXT && data->data_ptr && data->data_len > 0) {
            cJSON *root = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (!root) break;
            const cJSON *type = cJSON_GetObjectItem(root, "type");
            if (cJSON_IsString(type) && strcmp(type->valuestring, "status_result") == 0) {
                const cJSON *robot = cJSON_GetObjectItem(root, "robot");
                const cJSON *busy = cJSON_GetObjectItem(root, "busy");
                s_status.robot_ok = cJSON_IsTrue(robot);
                s_status.busy = cJSON_IsTrue(busy);
            } else if (cJSON_IsString(type) && strcmp(type->valuestring, "reset_result") == 0) {
                const cJSON *success = cJSON_GetObjectItem(root, "success");
                if (cJSON_IsTrue(success)) {
                    /* Status poll is ~2 s; mark busy immediately for the 1.5 s goto. */
                    s_status.busy = true;
                } else {
                    const cJSON *msg = cJSON_GetObjectItem(root, "message");
                    ESP_LOGW(TAG, "reset failed: %s",
                             cJSON_IsString(msg) ? msg->valuestring : "unknown");
                }
            }
            cJSON_Delete(root);
        }
        break;
    default:
        break;
    }
}

esp_err_t net_ws_start(const char *host, uint16_t port)
{
    if (s_client) {
        net_ws_stop();
    }
    snprintf(s_uri, sizeof(s_uri), "ws://%s:%u/ws", host, (unsigned)port);

    esp_websocket_client_config_t cfg = {0};
    cfg.uri = s_uri;
    cfg.reconnect_timeout_ms = 2000;
    cfg.network_timeout_ms = 5000;
    cfg.buffer_size = 2048;

    s_client = esp_websocket_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "client init failed");
        set_phase(NET_WS_DOWN);
        return ESP_FAIL;
    }
    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY, on_event, NULL);
    set_phase(NET_WS_CONNECTING);
    esp_err_t err = esp_websocket_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "client start failed: %s", esp_err_to_name(err));
        esp_websocket_client_destroy(s_client);
        s_client = NULL;
        set_phase(NET_WS_DOWN);
    }
    return err;
}

void net_ws_stop(void)
{
    if (!s_client) {
        set_phase(NET_WS_DOWN);
        return;
    }
    esp_websocket_client_close(s_client, pdMS_TO_TICKS(1000));
    esp_websocket_client_destroy(s_client);
    s_client = NULL;
    s_status.busy = false;
    set_phase(NET_WS_DOWN);
}

bool net_ws_connected(void)
{
    return s_client && s_status.phase == NET_WS_LINKED &&
           esp_websocket_client_is_connected(s_client);
}

bool net_ws_running(void) { return s_client != NULL; }

net_ws_status_t net_ws_status(void) { return s_status; }

net_ws_phase_t net_ws_phase(void) { return s_status.phase; }

static void send_text(const char *json)
{
    if (!net_ws_connected()) return;
    /* The websocket client treats a timed-out write as a dead transport and
     * aborts the connection. Keep this long enough that expiry means the link
     * is genuinely gone — a healthy send finishes in single-digit ms with
     * WIFI_PS_NONE. Blocking app_task here only stalls the pose stream; the
     * 250 Hz IMU task and LVGL (core 1) keep running. */
    int sent = esp_websocket_client_send_text(
        s_client, json, strlen(json), pdMS_TO_TICKS(1000));
    if (sent < 0) {
        s_status.send_fails++;
        ESP_LOGW(TAG, "send failed (%d) total_fails=%lu",
                 sent, (unsigned long)s_status.send_fails);
    }
}

void net_ws_send_state(const imu_integrate_state_t *imu, bool engaged, float gain, uint32_t seq)
{
    char buf[320];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"controller_state\",\"seq\":%lu,"
             "\"q\":[%.6f,%.6f,%.6f,%.6f],"
             "\"p\":[%.6f,%.6f,%.6f],"
             "\"engaged\":%s,\"gain\":%.3f,\"ready\":%s}",
             (unsigned long)seq,
             imu->q[0], imu->q[1], imu->q[2], imu->q[3],
             imu->p[0], imu->p[1], imu->p[2],
             engaged ? "true" : "false",
             gain,
             imu->ready ? "true" : "false");
    send_text(buf);
}

void net_ws_send_reset(void)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"type\":\"reset\",\"_id\":%d}", s_req_id++);
    send_text(buf);
}

void net_ws_send_status(void)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"type\":\"status\",\"_id\":%d}", s_req_id++);
    send_text(buf);
}
