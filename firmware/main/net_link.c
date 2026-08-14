#include "net_link.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "diag.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "net_wifi.h"
#include <unistd.h>

static const char *TAG = "net_link";

#define RX_MAX 512
#define HELLO_MAX 480
#define LINK_STALE_US 1000000
#define HELLO_PERIOD_US 2000000

static int s_sock = -1;
static struct sockaddr_in s_dest;
static net_link_status_t s_status;
static SemaphoreHandle_t s_lock;
static int64_t s_last_reply_us;
static int64_t s_last_hello_us;
static int64_t s_unlink_us;
static int s_last_send_ms;
static uint32_t s_pending_op;
static bool s_reset_outcome_pending;
static bool s_reset_outcome_failed;

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

static bool send_raw(const char *json)
{
    if (s_sock < 0) return false;
    int64_t t0 = esp_timer_get_time();
    int n = sendto(s_sock, json, strlen(json), 0,
                   (struct sockaddr *)&s_dest, sizeof(s_dest));
    int64_t dur = esp_timer_get_time() - t0;
    if (n < 0) {
        status_lock();
        s_status.send_fails++;
        uint32_t fails = s_status.send_fails;
        status_unlock();
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOBUFS) {
            ESP_LOGW(TAG, "sendto failed errno=%d total_fails=%lu", errno, (unsigned long)fails);
        }
        return false;
    }
    status_lock();
    s_status.send_ok++;
    s_last_send_ms = (int)(dur / 1000);
    status_unlock();
    return true;
}

static void send_hello(void)
{
    status_lock();
    uint32_t boot = s_status.boot_id;
    uint32_t send_ok = s_status.send_ok;
    uint32_t send_fail = s_status.send_fails;
    uint32_t down_ms = 0;
    if (s_unlink_us > 0) {
        down_ms = (uint32_t)((esp_timer_get_time() - s_unlink_us) / 1000);
    }
    int send_ms = s_last_send_ms;
    status_unlock();

    net_wifi_info_t wifi = net_wifi_info();
    char buf[HELLO_MAX];
    snprintf(buf, sizeof(buf),
             "{\"pv\":4,\"type\":\"hello\",\"boot_id\":\"%08lx\","
             "\"device\":\"esp32-reachy-ctl\","
             "\"diag\":{\"rst\":%d,\"wifi_n\":%lu,\"wifi_r\":%d,\"rssi\":%d,"
             "\"wifi_up\":%d,\"down_ms\":%lu,\"send_ok\":%lu,\"send_fail\":%lu,"
             "\"send_ms\":%d}}",
             (unsigned long)boot,
             diag_reset_reason_code(),
             (unsigned long)wifi.disconnects,
             wifi.last_reason,
             wifi.rssi,
             wifi.connected ? 1 : 0,
             (unsigned long)down_ms,
             (unsigned long)send_ok,
             (unsigned long)send_fail,
             send_ms);
    send_raw(buf);
}

static void handle_reply(const char *json, size_t len)
{
    int pv = 0;
    if (!json_find_int(json, len, "pv", &pv) || pv != 3) return;

    bool robot = false;
    char mode[24] = {0};
    json_find_bool(json, len, "robot", &robot);
    json_find_str(json, len, "mode", mode, sizeof(mode));
    net_host_mode_t hm = parse_mode(mode);

    int op_ack = 0;
    bool have_op = json_find_int(json, len, "op_ack", &op_ack);
    char op_status[24] = {0};
    if (have_op) json_find_str(json, len, "op_status", op_status, sizeof(op_status));

    status_lock();
    s_status.robot_ok = robot;
    s_status.host_mode = hm;
    s_status.busy = (hm == NET_HOST_RESETTING || hm == NET_HOST_FAULT);
    if (have_op && s_pending_op != 0 && (uint32_t)op_ack == s_pending_op) {
        if (strcmp(op_status, "completed") == 0) {
            s_pending_op = 0;
            s_reset_outcome_pending = true;
            s_reset_outcome_failed = false;
        } else if (strcmp(op_status, "failed") == 0) {
            s_pending_op = 0;
            s_reset_outcome_pending = true;
            s_reset_outcome_failed = true;
        }
    }
    status_unlock();
}

static void drain_rx(void)
{
    if (s_sock < 0) return;
    char buf[RX_MAX + 1];
    struct sockaddr_in from;
    socklen_t fromlen;
    for (int i = 0; i < 8; i++) {
        fromlen = sizeof(from);
        int n = recvfrom(s_sock, buf, RX_MAX, 0, (struct sockaddr *)&from, &fromlen);
        if (n < 0) break;
        if (n == 0) continue;
        buf[n] = '\0';
        handle_reply(buf, (size_t)n);
        s_last_reply_us = esp_timer_get_time();
        if (s_unlink_us > 0) {
            uint32_t down_ms = (uint32_t)((s_last_reply_us - s_unlink_us) / 1000);
            ESP_LOGI(TAG, "reply after %lu ms", (unsigned long)down_ms);
            s_unlink_us = 0;
        }
    }
}

static void ensure_boot_id(void)
{
    status_lock();
    if (s_status.boot_id == 0) {
        s_status.boot_id = esp_random();
        if (s_status.boot_id == 0) s_status.boot_id = 1;
        s_status.next_op_id = 1;
    }
    status_unlock();
}

esp_err_t net_link_open(const char *host, uint16_t port)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    net_link_close();
    ensure_boot_id();

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket failed");
        return ESP_FAIL;
    }
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags >= 0) fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    if (!inet_aton(host, &dest.sin_addr)) {
        ESP_LOGE(TAG, "bad host %s", host);
        close(sock);
        return ESP_FAIL;
    }

    s_sock = sock;
    s_dest = dest;
    s_last_reply_us = 0;
    s_last_hello_us = 0;
    s_unlink_us = esp_timer_get_time();
    ESP_LOGI(TAG, "open udp %s:%u", host, (unsigned)port);
    return ESP_OK;
}

void net_link_close(void)
{
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
    s_last_reply_us = 0;
    status_lock();
    s_status.linked = false;
    s_status.busy = false;
    s_status.robot_ok = false;
    s_status.host_mode = NET_HOST_IDLE;
    status_unlock();
}

bool net_link_ready(void)
{
    return s_sock >= 0;
}

bool net_link_linked(void)
{
    if (s_sock < 0 || s_last_reply_us == 0) return false;
    return (esp_timer_get_time() - s_last_reply_us) < LINK_STALE_US;
}

net_link_status_t net_link_status(void)
{
    bool linked = net_link_linked();
    net_link_status_t copy;
    status_lock();
    copy = s_status;
    status_unlock();
    copy.linked = linked;
    return copy;
}

void net_link_service(void)
{
    drain_rx();
    bool linked = net_link_linked();
    status_lock();
    s_status.linked = linked;
    status_unlock();
    if (linked) return;
    if (s_unlink_us == 0) s_unlink_us = esp_timer_get_time();
    int64_t now = esp_timer_get_time();
    if (s_sock >= 0 && (s_last_hello_us == 0 || (now - s_last_hello_us) >= HELLO_PERIOD_US)) {
        send_hello();
        s_last_hello_us = now;
    }
}

bool net_link_send_sample(const imu_integrate_state_t *imu, bool engaged, float gain, uint32_t seq)
{
    if (s_sock < 0) return false;
    status_lock();
    uint32_t boot = s_status.boot_id;
    uint32_t op = s_pending_op;
    status_unlock();

    char buf[360];
    int n;
    if (op != 0) {
        n = snprintf(buf, sizeof(buf),
                     "{\"pv\":4,\"boot_id\":\"%08lx\",\"seq\":%lu,"
                     "\"q\":[%.6f,%.6f,%.6f,%.6f],"
                     "\"engaged\":%s,\"gain\":%.3f,\"ready\":%s,\"op\":%lu}",
                     (unsigned long)boot,
                     (unsigned long)seq,
                     imu->q[0], imu->q[1], imu->q[2], imu->q[3],
                     engaged ? "true" : "false",
                     gain,
                     imu->ready ? "true" : "false",
                     (unsigned long)op);
    } else {
        n = snprintf(buf, sizeof(buf),
                     "{\"pv\":4,\"boot_id\":\"%08lx\",\"seq\":%lu,"
                     "\"q\":[%.6f,%.6f,%.6f,%.6f],"
                     "\"engaged\":%s,\"gain\":%.3f,\"ready\":%s}",
                     (unsigned long)boot,
                     (unsigned long)seq,
                     imu->q[0], imu->q[1], imu->q[2], imu->q[3],
                     engaged ? "true" : "false",
                     gain,
                     imu->ready ? "true" : "false");
    }
    if (n < 0 || n >= (int)sizeof(buf)) return false;
    return send_raw(buf);
}

bool net_link_begin_reset(void)
{
    status_lock();
    if (s_pending_op != 0) {
        status_unlock();
        return true;
    }
    uint32_t op = s_status.next_op_id;
    s_status.next_op_id = op + 1;
    s_pending_op = op;
    s_reset_outcome_pending = false;
    s_reset_outcome_failed = false;
    status_unlock();
    ESP_LOGI(TAG, "reset armed op=%lu", (unsigned long)op);
    return true;
}

bool net_link_take_reset_outcome(bool *failed)
{
    status_lock();
    if (!s_reset_outcome_pending) {
        status_unlock();
        return false;
    }
    s_reset_outcome_pending = false;
    if (failed) *failed = s_reset_outcome_failed;
    status_unlock();
    return true;
}
