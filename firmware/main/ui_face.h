#pragma once
#include "lvgl.h"
void ui_face_create(lv_obj_t *parent);
void ui_face_set_mode(int mode); /* 0 disengaged (closed), 1 engaged (open) */
