#pragma once
#include "esp_err.h"
#include <stdbool.h>
esp_err_t net_wifi_start(void);
bool net_wifi_connected(void);
