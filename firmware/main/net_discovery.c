#include "net_discovery.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "mdns.h"
#include "sdkconfig.h"

static const char *TAG = "net_discovery";

esp_err_t net_discovery_init(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) return err;
    mdns_hostname_set("reachy-ctl");
    return ESP_OK;
}

bool net_discovery_resolve(char *out_host, size_t out_len, uint16_t *out_port)
{
    if (CONFIG_RMC_ROBOT_HOST[0] != '\0') {
        strncpy(out_host, CONFIG_RMC_ROBOT_HOST, out_len - 1);
        out_host[out_len - 1] = '\0';
        *out_port = (uint16_t)CONFIG_RMC_ROBOT_PORT;
        ESP_LOGI(TAG, "using override host %s:%u", out_host, (unsigned)*out_port);
        return true;
    }

    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query_ptr("_reachyctl", "_tcp", 3000, 8, &results);
    if (err != ESP_OK || !results) {
        ESP_LOGW(TAG, "mDNS query failed or empty");
        return false;
    }

    bool ok = false;
    for (mdns_result_t *r = results; r; r = r->next) {
        if (r->addr && r->addr->addr.type == ESP_IPADDR_TYPE_V4) {
            snprintf(out_host, out_len, IPSTR, IP2STR(&r->addr->addr.u_addr.ip4));
            *out_port = r->port ? r->port : (uint16_t)CONFIG_RMC_ROBOT_PORT;
            ok = true;
            ESP_LOGI(TAG, "mDNS found %s:%u", out_host, (unsigned)*out_port);
            break;
        }
    }
    mdns_query_results_free(results);
    return ok;
}
