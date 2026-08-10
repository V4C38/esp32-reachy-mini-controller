#pragma once
#include <stdbool.h>
#include "lvgl.h"
void ui_settings_create(lv_obj_t *parent, void (*close_cb)(void), void (*reset_cb)(void), float *gain_ptr);
void ui_settings_show(bool show);
