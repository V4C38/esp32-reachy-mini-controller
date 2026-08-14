#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool connected;
    int rssi;
    uint32_t disconnects;
    int last_reason;
    int last_rssi;
} net_wifi_info_t;

esp_err_t net_wifi_start(void);
bool net_wifi_connected(void);
net_wifi_info_t net_wifi_info(void);
