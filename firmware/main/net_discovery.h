#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t net_discovery_init(void);
/* Resolve host into out_host (IP string). Returns true on success. */
bool net_discovery_resolve(char *out_host, size_t out_len, uint16_t *out_port);
