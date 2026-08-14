#include <stdio.h>
#include <string.h>

#include "config.h"
#include "diag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"
#include "imu.h"
#include "net_discovery.h"
#include "net_wifi.h"
#include "net_link.h"
#include "pmu.h"
#include "ui.h"

static const char *TAG = "main";

static TaskHandle_t s_imu_task;
static TaskHandle_t s_app_task;

#define LINK_DOWN_DEBOUNCE_US 1500000
#define RE_RESOLVE_US 5000000

static void imu_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / RMC_IMU_HZ);
    TickType_t last = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();
    while (true) {
        vTaskDelayUntil(&last, period);
        int64_t now = esp_timer_get_time();
        float dt = (now - prev_us) / 1e6f;
        prev_us = now;
        diag_note_imu_dt(dt);
        imu_update(dt);
    }
}

static void app_task(void *arg)
{
    (void)arg;
    char host[64] = {0};
    uint16_t port = RMC_LINK_PORT;
    uint32_t seq = 0;
    int64_t last_send = 0;
    int64_t unlink_since = 0;
    bool was_linked = false;
    bool have_host = false;

    ESP_LOGI(TAG, "app_task started");

    while (true) {
        if (ui_connect_pending()) {
            ui_clear_connect_request();
            if (!net_link_linked()) {
                ESP_LOGI(TAG, "connect requested — re-resolving host");
                net_link_close();
                have_host = false;
                unlink_since = 0;
                was_linked = false;
            }
        }

        if (!net_wifi_connected()) {
            have_host = false;
            unlink_since = 0;
            was_linked = false;
            net_link_close();
            ui_set_linked(false);
            diag_log_resources("wifi_lost", s_imu_task, s_app_task);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!have_host) {
            if (!net_discovery_resolve(host, sizeof(host), &port)) {
                ESP_LOGW(TAG, "host resolve failed");
                ui_set_linked(false);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            if (net_link_open(host, port) != ESP_OK) {
                ESP_LOGW(TAG, "link open failed");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            have_host = true;
            /* 5 s unlink budget starts once this socket is live, not during
             * blocking discovery — otherwise we close before the first reply. */
            unlink_since = 0;
        }

        net_link_service();

        int64_t now = esp_timer_get_time();
        bool linked = net_link_linked();
        if (linked) {
            unlink_since = 0;
            ui_set_linked(true);
            if (!was_linked) {
                net_link_status_t st0 = net_link_status();
                ESP_LOGI(TAG, "link up (send_ok=%lu send_fail=%lu boot=%08lx)",
                         (unsigned long)st0.send_ok,
                         (unsigned long)st0.send_fails,
                         (unsigned long)st0.boot_id);
                diag_log_resources("link_up", s_imu_task, s_app_task);
            }
            was_linked = true;
        } else {
            was_linked = false;
            if (unlink_since == 0) unlink_since = now;
            if ((now - unlink_since) >= LINK_DOWN_DEBOUNCE_US) {
                ui_set_linked(false);
            }
            if ((now - unlink_since) >= RE_RESOLVE_US) {
                ESP_LOGW(TAG, "unlinked — re-resolving host");
                net_link_close();
                have_host = false;
                unlink_since = 0;
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        }

        net_link_status_t st = net_link_status();

        if ((now - last_send) >= (1000000 / RMC_SEND_HZ)) {
            imu_integrate_state_t imu;
            imu_get_state(&imu);
            bool engaged = ui_engaged() && imu.ready;
            if (st.host_mode == NET_HOST_RESETTING || st.host_mode == NET_HOST_FAULT) {
                engaged = false;
            }
            if (net_link_send_sample(&imu, engaged, ui_get_gain(), seq)) {
                seq++;
                last_send = now;
            }

#if CONFIG_RMC_TRACE
            {
                float a[3], g[3];
                imu_get_raw_mapped(a, g);
                printf("RMC_TRACE,%lld,"
                       "%.5f,%.5f,%.5f,%.5f,"
                       "%.4f,%.4f,%.4f,"
                       "%.4f,%.4f,%.4f,"
                       "%d,%d\n",
                       (long long)(now / 1000),
                       imu.q[0], imu.q[1], imu.q[2], imu.q[3],
                       a[0], a[1], a[2],
                       g[0], g[1], g[2],
                       imu.still ? 1 : 0, engaged ? 1 : 0);
            }
#endif
        }

        diag_maybe_periodic(s_imu_task, s_app_task);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Reachy Motion Controller");
    diag_log_boot();

    ESP_ERROR_CHECK(ui_init());

    i2c_master_bus_handle_t i2c = bsp_i2c_get_handle();
    if (!i2c) {
        ESP_ERROR_CHECK(bsp_i2c_init());
        i2c = bsp_i2c_get_handle();
    }

    if (pmu_init(i2c) != ESP_OK) {
        ESP_LOGW(TAG, "PMU init skipped — connect a LiPo for untethered use");
    }

    if (imu_init(i2c) != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed — motion disabled until ready");
    }

    if (net_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi failed — check sdkconfig.local");
    } else {
        ESP_LOGI(TAG, "WiFi up, starting tasks");
    }
    ESP_ERROR_CHECK(net_discovery_init());

    /* IMU on core 0 with WiFi/app — LVGL owns core 1 so flush waits are not
     * starved by the 250 Hz integrator. */
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, 4, &s_imu_task, 0);
    xTaskCreatePinnedToCore(app_task, "app", 8192, NULL, 5, &s_app_task, 0);
}
