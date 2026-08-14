#pragma once
#include <stdbool.h>
#include "esp_err.h"

esp_err_t ui_init(void);

/* Link state drives the bottom-center status label: gray Starting... while
 * booting, red DISCONNECTED once unlinked after boot, hidden when linked.
 * Engage is toggled by the BOOT button (GPIO0); eyes follow engage. */
void ui_set_linked(bool linked);
bool ui_linked(void);

bool ui_engaged(void);
float ui_get_gain(void);

/* Cross-core connect request (LVGL core 1 → app_task core 0).
 * Forces host re-resolve; ignored if already linked. */
bool ui_connect_pending(void);
void ui_clear_connect_request(void);
