#include "net_discovery.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "sdkconfig.h"

static const char *TAG = "net_discovery";

#define MDNS_ATTEMPTS 3
#define MDNS_PTR_TIMEOUT_MS 5000
#define UDP_PROBE "RMC2?"
#define UDP_REPLY_PREFIX "RMC2 "

static bool copy_ipv4(const mdns_ip_addr_t *addr, char *out_host, size_t out_len)
{
    for (const mdns_ip_addr_t *a = addr; a; a = a->next) {
        if (a->addr.type == ESP_IPADDR_TYPE_V4) {
            snprintf(out_host, out_len, IPSTR, IP2STR(&a->addr.u_addr.ip4));
            return true;
        }
    }
    return false;
}

static bool take_mdns_results(mdns_result_t *results, char *out_host, size_t out_len, uint16_t *out_port)
{
    char hostname[64] = {0};
    uint16_t port = 0;

    for (mdns_result_t *r = results; r; r = r->next) {
        if (r->port) port = r->port;
        if (copy_ipv4(r->addr, out_host, out_len)) {
            *out_port = port ? port : (uint16_t)CONFIG_RMC_ROBOT_PORT;
            ESP_LOGI(TAG, "mDNS found %s:%u", out_host, (unsigned)*out_port);
            return true;
        }
        if (r->hostname && !hostname[0]) {
            strncpy(hostname, r->hostname, sizeof(hostname) - 1);
        }
    }

    if (!hostname[0]) return false;

    esp_ip4_addr_t addr = {0};
    if (mdns_query_a(hostname, 2000, &addr) == ESP_OK) {
        snprintf(out_host, out_len, IPSTR, IP2STR(&addr));
        *out_port = port ? port : (uint16_t)CONFIG_RMC_ROBOT_PORT;
        ESP_LOGI(TAG, "mDNS A %s -> %s:%u", hostname, out_host, (unsigned)*out_port);
        return true;
    }

    ESP_LOGW(TAG, "mDNS got host %s but no A record", hostname);
    return false;
}

static bool mdns_browse(char *out_host, size_t out_len, uint16_t *out_port)
{
    for (int i = 0; i < MDNS_ATTEMPTS; i++) {
        mdns_result_t *results = NULL;
        esp_err_t err = mdns_query_ptr("_reachyctl", "_tcp", MDNS_PTR_TIMEOUT_MS, 8, &results);
        if (err == ESP_OK && results) {
            bool ok = take_mdns_results(results, out_host, out_len, out_port);
            mdns_query_results_free(results);
            if (ok) return true;
        } else {
            ESP_LOGW(TAG, "mDNS PTR empty (try %d/%d)", i + 1, MDNS_ATTEMPTS);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* Well-known hostname from MdnsAdvertiser (server="esp32-motion.local."). */
    esp_ip4_addr_t addr = {0};
    if (mdns_query_a("esp32-motion", 2000, &addr) == ESP_OK) {
        snprintf(out_host, out_len, IPSTR, IP2STR(&addr));
        *out_port = (uint16_t)CONFIG_RMC_ROBOT_PORT;
        ESP_LOGI(TAG, "mDNS A esp32-motion -> %s:%u", out_host, (unsigned)*out_port);
        return true;
    }
    return false;
}

static void send_probe(int sock, uint32_t ip_nbo, uint16_t port)
{
    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    dest.sin_addr.s_addr = ip_nbo;
    (void)sendto(sock, UDP_PROBE, sizeof(UDP_PROBE) - 1, 0,
                 (struct sockaddr *)&dest, sizeof(dest));
}

static bool udp_broadcast_resolve(char *out_host, size_t out_len, uint16_t *out_port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGW(TAG, "udp probe socket failed");
        return false;
    }

    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint16_t port = (uint16_t)CONFIG_RMC_ROBOT_PORT;
    send_probe(sock, htonl(INADDR_BROADCAST), port);

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
            uint32_t bcast = (ip.ip.addr & ip.netmask.addr) | ~ip.netmask.addr;
            send_probe(sock, bcast, port);
        }
    }

    char buf[64];
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &fromlen);
    close(sock);
    if (n <= 0) {
        ESP_LOGW(TAG, "udp probe: no reply");
        return false;
    }
    buf[n] = '\0';

    unsigned p = port;
    char ip[16] = {0};
    if (sscanf(buf, UDP_REPLY_PREFIX "%15[0-9.] %u", ip, &p) < 1) {
        ESP_LOGW(TAG, "udp probe: bad reply");
        return false;
    }
    strncpy(out_host, ip, out_len - 1);
    out_host[out_len - 1] = '\0';
    *out_port = (uint16_t)(p ? p : port);
    ESP_LOGI(TAG, "udp probe found %s:%u", out_host, (unsigned)*out_port);
    return true;
}

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

    if (mdns_browse(out_host, out_len, out_port)) return true;

    /* ESP32-S3 is 2.4 GHz only. A dual-band AP often forwards unicast and
     * subnet broadcast while dropping mDNS multicast (224.0.0.251) between
     * radios — PTR browse comes back empty even though the app is up. */
    return udp_broadcast_resolve(out_host, out_len, out_port);
}
