#include "diag.h"

#include "config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "diag";

static uint32_t s_imu_overruns;
static int64_t s_last_periodic_us;
static int s_reset_reason_code;

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "ext";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
#ifdef ESP_RST_USB
    case ESP_RST_USB: return "usb";
#endif
    default: return "other";
    }
}

static void log_heap(const char *where)
{
    ESP_LOGI(TAG,
             "%s heap free_int=%u free_dma=%u largest_dma=%u free_psram=%u imu_overruns=%lu",
             where,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned long)s_imu_overruns);
}

void diag_log_boot(void)
{
    s_reset_reason_code = (int)esp_reset_reason();
    ESP_LOGI(TAG, "reset_reason=%s (%d)",
             reset_reason_str(esp_reset_reason()),
             s_reset_reason_code);
    log_heap("boot");
}

int diag_reset_reason_code(void)
{
    return s_reset_reason_code;
}

void diag_log_resources(const char *where, TaskHandle_t imu, TaskHandle_t app)
{
    log_heap(where ? where : "res");
    if (imu) {
        ESP_LOGI(TAG, "hwm imu=%u", (unsigned)uxTaskGetStackHighWaterMark(imu));
    }
    if (app) {
        ESP_LOGI(TAG, "hwm app=%u", (unsigned)uxTaskGetStackHighWaterMark(app));
    }
}

void diag_note_imu_dt(float dt_s)
{
    const float limit = 1.5f / (float)RMC_IMU_HZ;
    if (dt_s > limit) {
        s_imu_overruns++;
        if ((s_imu_overruns % 50U) == 1U) {
            ESP_LOGW(TAG, "imu deadline miss dt=%.1f ms (count=%lu)",
                     dt_s * 1000.f, (unsigned long)s_imu_overruns);
        }
    }
}

void diag_maybe_periodic(TaskHandle_t imu, TaskHandle_t app)
{
    int64_t now = esp_timer_get_time();
    if (s_last_periodic_us != 0 && (now - s_last_periodic_us) < 10000000) return;
    s_last_periodic_us = now;
    diag_log_resources("periodic", imu, app);
}
