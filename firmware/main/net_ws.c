#include "net_ws.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "net_ws";
static esp_websocket_client_handle_t s_client;
static net_ws_status_t s_status;
static char s_uri[128];
static int64_t s_disconnect_us;
static SemaphoreHandle_t s_lock;
static bool s_hello_sent;

#define RX_MAX 512

static void status_lock(void)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void status_unlock(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}

static net_host_mode_t parse_mode(const char *s)
{
    if (!s) return NET_HOST_IDLE;
    if (strncmp(s, "resetting", 9) == 0) return NET_HOST_RESETTING;
    if (strncmp(s, "engaged", 7) == 0) return NET_HOST_ENGAGED;
    if (strncmp(s, "fault", 5) == 0) return NET_HOST_FAULT;
    return NET_HOST_IDLE;
}

/* Tiny fixed-buffer field extractors — no heap. Values must be contiguous JSON tokens. */
static bool json_find_str(const char *json, size_t len, const char *key, char *out, size_t out_len)
{
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = json;
    const char *end = json + len;
    size_t plen = strlen(pattern);
    while (p + plen < end) {
        if (memcmp(p, pattern, plen) == 0) {
            p += plen;
            while (p < end && (*p == ' ' || *p == '\t' || *p == ':')) p++;
            if (p >= end || *p != '"') return false;
            p++;
            size_t i = 0;
            while (p < end && *p != '"' && i + 1 < out_len) {
                out[i++] = *p++;
            }
            out[i] = '\0';
            return p < end && *p == '"';
        }
        p++;
    }
    return false;
}

static bool json_find_bool(const char *json, size_t len, const char *key, bool *out)
{
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = json;
    const char *end = json + len;
    size_t plen = strlen(pattern);
    while (p + plen < end) {
        if (memcmp(p, pattern, plen) == 0) {
            p += plen;
            while (p < end && (*p == ' ' || *p == '\t' || *p == ':')) p++;
            if (p + 4 <= end && strncmp(p, "true", 4) == 0) {
                *out = true;
                return true;
            }
            if (p + 5 <= end && strncmp(p, "false", 5) == 0) {
                *out = false;
                return true;
            }
            return false;
        }
        p++;
    }
    return false;
}

static bool json_find_int(const char *json, size_t len, const char *key, int *out)
{
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = json;
    const char *end = json + len;
    size_t plen = strlen(pattern);
    while (p + plen < end) {
        if (memcmp(p, pattern, plen) == 0) {
            p += plen;
            while (p < end && (*p == ' ' || *p == '\t' || *p == ':')) p++;
            char *endp = NULL;
            long v = strtol(p, &endp, 10);
            if (endp == p) return false;
            *out = (int)v;
            return true;
        }
        p++;
    }
    return false;
}

static void set_phase(net_ws_phase_t phase)
{
    status_lock();
    if (s_status.phase == phase) {
        status_unlock();
        return;
    }
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
        }
    } else if (phase == NET_WS_CONNECTING) {
        ESP_LOGI(TAG, "connecting");
        s_status.hello_ok = false;
        s_hello_sent = false;
    } else {
        if (s_disconnect_us == 0) s_disconnect_us = esp_timer_get_time();
        s_status.robot_ok = false;
        s_status.hello_ok = false;
        s_hello_sent = false;
        s_status.busy = false;
        s_status.host_mode = NET_HOST_IDLE;
        ESP_LOGW(TAG, "disconnected (send_fails=%lu)",
                 (unsigned long)s_status.send_fails);
    }
    status_unlock();
}

static void handle_host_json(const char *json, size_t len)
{
    if (len == 0 || len > RX_MAX) return;

    char type[32] = {0};
    if (!json_find_str(json, len, "type", type, sizeof(type))) return;

    if (strcmp(type, "hello") == 0) {
        int ver = 0;
        if (json_find_int(json, len, "protocol_version", &ver) && ver == 2) {
            status_lock();
            s_status.hello_ok = true;
            status_unlock();
            ESP_LOGI(TAG, "host hello ok (protocol 2)");
        } else {
            ESP_LOGE(TAG, "host hello rejected / bad version");
        }
        return;
    }

    if (strcmp(type, "host_state") == 0) {
        bool robot = false;
        char mode[24] = {0};
        json_find_bool(json, len, "robot", &robot);
        json_find_str(json, len, "mode", mode, sizeof(mode));
        net_host_mode_t hm = parse_mode(mode);
        status_lock();
        s_status.robot_ok = robot;
        s_status.host_mode = hm;
        s_status.busy = (hm == NET_HOST_RESETTING || hm == NET_HOST_FAULT);
        status_unlock();
        ESP_LOGI(TAG, "host_state robot=%d mode=%s", (int)robot, mode);
        return;
    }

    if (strcmp(type, "reset_result") == 0) {
        char status[24] = {0};
        json_find_str(json, len, "status", status, sizeof(status));
        status_lock();
        if (strcmp(status, "accepted") == 0 || strcmp(status, "completed") == 0) {
            /* accepted/completed: keep busy until host_state says idle, but
             * completed alone should clear if host_state was lost. */
            if (strcmp(status, "completed") == 0) {
                s_status.busy = false;
                s_status.host_mode = NET_HOST_IDLE;
            } else {
                s_status.busy = true;
                s_status.host_mode = NET_HOST_RESETTING;
            }
        } else if (strcmp(status, "failed") == 0) {
            s_status.busy = false;
            s_status.host_mode = NET_HOST_FAULT;
            char msg[64] = {0};
            json_find_str(json, len, "message", msg, sizeof(msg));
            ESP_LOGW(TAG, "reset failed: %s", msg[0] ? msg : "unknown");
        }
        status_unlock();
        return;
    }

    if (strcmp(type, "error") == 0) {
        char msg[80] = {0};
        json_find_str(json, len, "message", msg, sizeof(msg));
        ESP_LOGW(TAG, "host error: %s", msg[0] ? msg : "unknown");
        return;
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
        if (s_client && net_ws_phase() == NET_WS_LINKED) {
            set_phase(NET_WS_CONNECTING);
        }
        ESP_LOGW(TAG, "error (error_type=%d sock_errno=%d)",
                 (int)data->error_handle.error_type,
                 data->error_handle.esp_transport_sock_errno);
        break;
    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == WS_TRANSPORT_OPCODES_TEXT && data->data_ptr && data->data_len > 0) {
            /* Only accept complete frames that fit our fixed RX budget. */
            if (data->payload_len > RX_MAX) {
                ESP_LOGW(TAG, "dropping oversize frame (%d)", data->payload_len);
                break;
            }
            if (data->data_len != data->payload_len) {
                /* Fragmented — drop rather than allocate a reassembly buffer. */
                ESP_LOGW(TAG, "dropping fragmented frame");
                break;
            }
            char buf[RX_MAX + 1];
            memcpy(buf, data->data_ptr, data->data_len);
            buf[data->data_len] = '\0';
            handle_host_json(buf, (size_t)data->data_len);
        }
        break;
    default:
        break;
    }
}

static void send_text(const char *json)
{
    if (!net_ws_connected()) return;
    int sent = esp_websocket_client_send_text(
        s_client, json, strlen(json), pdMS_TO_TICKS(1000));
    if (sent < 0) {
        status_lock();
        s_status.send_fails++;
        uint32_t fails = s_status.send_fails;
        status_unlock();
        ESP_LOGW(TAG, "send failed (%d) total_fails=%lu", sent, (unsigned long)fails);
    }
}

static void send_hello_if_needed(void)
{
    status_lock();
    bool linked = (s_status.phase == NET_WS_LINKED);
    bool sent = s_hello_sent;
    uint32_t boot = s_status.boot_id;
    if (linked && !sent) s_hello_sent = true;
    status_unlock();
    if (!linked || sent) return;

    char buf[192];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"hello\",\"protocol_version\":2,"
             "\"boot_id\":\"%08lx\",\"device\":\"esp32-reachy-ctl\"}",
             (unsigned long)boot);
    send_text(buf);
}

esp_err_t net_ws_start(const char *host, uint16_t port)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (s_client) {
        net_ws_stop();
    }
    status_lock();
    if (s_status.boot_id == 0) {
        s_status.boot_id = esp_random();
        if (s_status.boot_id == 0) s_status.boot_id = 1;
        s_status.next_op_id = 1;
    }
    status_unlock();

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
    status_lock();
    s_status.busy = false;
    s_status.hello_ok = false;
    s_hello_sent = false;
    status_unlock();
    set_phase(NET_WS_DOWN);
}

bool net_ws_connected(void)
{
    status_lock();
    bool ok = s_client && s_status.phase == NET_WS_LINKED;
    status_unlock();
    return ok && esp_websocket_client_is_connected(s_client);
}

bool net_ws_running(void) { return s_client != NULL; }

net_ws_status_t net_ws_status(void)
{
    net_ws_status_t copy;
    status_lock();
    copy = s_status;
    status_unlock();
    return copy;
}

net_ws_phase_t net_ws_phase(void)
{
    status_lock();
    net_ws_phase_t p = s_status.phase;
    status_unlock();
    return p;
}

void net_ws_service(void)
{
    send_hello_if_needed();
}

void net_ws_send_sample(const imu_integrate_state_t *imu, bool engaged, float gain, uint32_t seq)
{
    send_hello_if_needed();
    status_lock();
    uint32_t boot = s_status.boot_id;
    bool hello = s_status.hello_ok;
    bool busy = s_status.busy;
    status_unlock();
    if (!hello) return;
    if (busy) engaged = false;

    char buf[360];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"sample\",\"boot_id\":\"%08lx\",\"seq\":%lu,"
             "\"q\":[%.6f,%.6f,%.6f,%.6f],"
             "\"p\":[%.6f,%.6f,%.6f],"
             "\"engaged\":%s,\"gain\":%.3f,\"ready\":%s}",
             (unsigned long)boot,
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
    send_hello_if_needed();
    status_lock();
    uint32_t boot = s_status.boot_id;
    uint32_t op = s_status.next_op_id++;
    bool hello = s_status.hello_ok;
    status_unlock();
    if (!hello) {
        ESP_LOGW(TAG, "reset deferred — hello not complete");
        return;
    }
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"reset\",\"boot_id\":\"%08lx\",\"op_id\":%lu}",
             (unsigned long)boot, (unsigned long)op);
    send_text(buf);
}
