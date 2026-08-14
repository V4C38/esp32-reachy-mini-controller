#pragma once
#include <stdbool.h>
#include "lvgl.h"

void ui_settings_create(lv_obj_t *parent, void (*close_cb)(void), void (*connect_cb)(void), float *gain_ptr);
void ui_settings_show(bool show);
/* CONNECT TO APP lives where RESET POSE was; hidden while the UDP link is up. */
void ui_settings_set_connect_visible(bool visible);
