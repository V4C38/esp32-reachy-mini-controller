#pragma once
#include "lvgl.h"
void ui_face_create(lv_obj_t *parent);
void ui_face_set_mode(int mode); /* 0 idle, 1 engaged, 2 disconnected */

/* Nudge the face to hint at device motion. nx/ny/roll are -1..1. */
void ui_face_set_motion(float nx, float ny, float roll);
