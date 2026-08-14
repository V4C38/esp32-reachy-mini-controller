#include "ui_face.h"

#include "assets/reachy_eyes.h"
#include "lvgl.h"

#define CATCH0_X (-48)
#define CATCH0_Y (-8)
#define CATCH1_X 40
#define CATCH1_Y (-6)

static lv_obj_t *s_eyes;
static lv_obj_t *s_cl0;
static lv_obj_t *s_cl1;

static void make_passthrough(lv_obj_t *obj)
{
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

void ui_face_create(lv_obj_t *parent)
{
    lv_obj_t *root = lv_obj_create(parent);
    lv_obj_set_size(root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(root, lv_color_hex(0xEDEBE6), 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_set_style_pad_all(root, 0, 0);
    make_passthrough(root);

    s_eyes = lv_image_create(root);
    lv_image_set_src(s_eyes, &reachy_eyes_closed);
    lv_obj_center(s_eyes);
    make_passthrough(s_eyes);

    s_cl0 = lv_obj_create(root);
    lv_obj_set_size(s_cl0, 10, 14);
    lv_obj_set_style_radius(s_cl0, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_cl0, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(s_cl0, 0, 0);
    lv_obj_align(s_cl0, LV_ALIGN_CENTER, CATCH0_X, CATCH0_Y);
    lv_obj_add_flag(s_cl0, LV_OBJ_FLAG_HIDDEN);
    make_passthrough(s_cl0);

    s_cl1 = lv_obj_create(root);
    lv_obj_set_size(s_cl1, 8, 12);
    lv_obj_set_style_radius(s_cl1, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_cl1, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(s_cl1, 0, 0);
    lv_obj_align(s_cl1, LV_ALIGN_CENTER, CATCH1_X, CATCH1_Y);
    lv_obj_add_flag(s_cl1, LV_OBJ_FLAG_HIDDEN);
    make_passthrough(s_cl1);
}

void ui_face_set_mode(int mode)
{
    /* 0 disengaged (closed eyes), 1 engaged (open eyes). */
    if (mode == 0) {
        lv_image_set_src(s_eyes, &reachy_eyes_closed);
        lv_obj_add_flag(s_cl0, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cl1, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_image_set_src(s_eyes, &reachy_eyes);
        lv_obj_clear_flag(s_cl0, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cl1, LV_OBJ_FLAG_HIDDEN);
    }
}
