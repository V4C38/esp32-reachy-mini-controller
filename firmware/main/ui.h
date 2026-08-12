#pragma once
#include <stdbool.h>
#include "esp_err.h"

esp_err_t ui_init(void);

/* Link state only selects the face artwork and the idle brightness. Touch,
 * hold-to-engage and the settings screen stay live while offline. */
void ui_set_linked(bool linked);
bool ui_linked(void);

bool ui_engaged(void);
bool ui_take_reset_request(void);
float ui_get_gain(void);
