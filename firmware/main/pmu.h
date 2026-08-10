#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Probe AXP2101 and enable battery charge/discharge path for untethered use. */
esp_err_t pmu_init(i2c_master_bus_handle_t bus);

#ifdef __cplusplus
}
#endif
