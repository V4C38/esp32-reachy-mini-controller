#include "net_wifi.h"

#include <string.h>
#include "config.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "net_wifi";
static EventGroupHandle_t s_wifi_event;
#define WIFI_CONNECTED_BIT BIT0
static bool s_connected;

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        xEventGroupClearBits(s_wifi_event, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "disconnected; reconnecting");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_connected = true;
        xEventGroupSetBits(s_wifi_event, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&e->ip_info.ip));
    }
}

esp_err_t net_wifi_start(void)
{
    const char *ssid = CONFIG_RMC_WIFI_SSID;
    const char *pass = CONFIG_RMC_WIFI_PASSWORD;
    if (ssid[0] == '\0') {
        ESP_LOGE(TAG, "CONFIG_RMC_WIFI_SSID is empty — copy sdkconfig.local.example to sdkconfig.local");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Realtime 20 Hz pose stream cannot tolerate DTIM sleep latency.
     * Raises idle current — only matters for the optional LiPo. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "connecting to SSID '%s'", ssid);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(20000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "WiFi connect timeout");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool net_wifi_connected(void) { return s_connected; }
