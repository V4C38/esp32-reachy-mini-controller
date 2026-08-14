#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Boot banner: reset reason + DMA/internal/PSRAM heap. Call once from app_main. */
void diag_log_boot(void);

/* Snapshot of DMA/internal/PSRAM heap and optional task high-water marks.
 * `where` is a short label (e.g. "wifi", "linked", "periodic"). */
void diag_log_resources(const char *where, TaskHandle_t imu, TaskHandle_t app);

/* Count IMU loop overruns (dt much larger than the 250 Hz period). */
void diag_note_imu_dt(float dt_s);

/* Rate-limited dump from app_task (~every 10 s). */
void diag_maybe_periodic(TaskHandle_t imu, TaskHandle_t app);

/* Numeric esp_reset_reason_t from boot, for hello diagnostics. */
int diag_reset_reason_code(void);
