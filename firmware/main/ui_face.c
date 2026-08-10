#include "ui_face.h"

#include <math.h>

#include "assets/reachy_eyes.h"
#include "lvgl.h"

/* Deliberately small: a hint of motion, not a parallax effect. Redrawing the
 * eye bitmap is the expensive part, so it moves in 2 px steps. */
#define FACE_SHIFT_X 8
#define FACE_SHIFT_Y 6
#define FACE_STEP 2
#define CATCH_SHIFT_X 5
#define CATCH_SHIFT_Y 4
#define CATCH_ROLL 4

static lv_obj_t *s_root;
static lv_obj_t *s_eyes;
static lv_obj_t *s_cl0;
static lv_obj_t *s_cl1;
static int s_eye_dx;
static int s_eye_dy;
static int s_cl0_x, s_cl0_y, s_cl1_x, s_cl1_y;

static void make_passthrough(lv_obj_t *obj)
{
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

void ui_face_create(lv_obj_t *parent)
{
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0xEDEBE6), 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 0, 0);
    make_passthrough(s_root);

    s_eyes = lv_image_create(s_root);
    lv_image_set_src(s_eyes, &reachy_eyes);
    lv_obj_center(s_eyes);
    make_passthrough(s_eyes);

    s_cl0 = lv_obj_create(s_root);
    lv_obj_set_size(s_cl0, 10, 14);
    lv_obj_set_style_radius(s_cl0, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_cl0, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(s_cl0, 0, 0);
    make_passthrough(s_cl0);

    s_cl1 = lv_obj_create(s_root);
    lv_obj_set_size(s_cl1, 8, 12);
    lv_obj_set_style_radius(s_cl1, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_cl1, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(s_cl1, 0, 0);
    make_passthrough(s_cl1);

    ui_face_set_motion(0, 0, 0);
}

void ui_face_set_mode(int mode)
{
    /* 0 idle, 1 engaged, 2 disconnected (closed eyes). */
    if (mode == 2) {
        lv_image_set_src(s_eyes, &reachy_eyes_closed);
        lv_obj_add_flag(s_cl0, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cl1, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_image_set_src(s_eyes, &reachy_eyes);
        lv_obj_clear_flag(s_cl0, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cl1, LV_OBJ_FLAG_HIDDEN);
    }
}

static float clamp1(float v)
{
    if (v > 1.f) return 1.f;
    if (v < -1.f) return -1.f;
    return v;
}

static int quantise(float v, int span)
{
    int px = (int)lroundf(v * (float)span / (float)FACE_STEP);
    return px * FACE_STEP;
}

void ui_face_set_motion(float nx, float ny, float roll)
{
    nx = clamp1(nx);
    ny = clamp1(ny);
    roll = clamp1(roll);

    int dx = quantise(nx, FACE_SHIFT_X);
    int dy = quantise(-ny, FACE_SHIFT_Y);
    if (dx != s_eye_dx || dy != s_eye_dy) {
        s_eye_dx = dx;
        s_eye_dy = dy;
        lv_obj_set_style_translate_x(s_eyes, dx, 0);
        lv_obj_set_style_translate_y(s_eyes, dy, 0);
    }

    int cx = dx + (int)lroundf(nx * CATCH_SHIFT_X);
    int cy = dy + (int)lroundf(-ny * CATCH_SHIFT_Y);
    int tilt = (int)lroundf(roll * CATCH_ROLL);
    int c0x = cx - 48, c0y = cy - 8 + tilt;
    int c1x = cx + 40, c1y = cy - 6 - tilt;
    /* Align invalidates the object — only touch LVGL when pixels change. */
    if (c0x != s_cl0_x || c0y != s_cl0_y) {
        s_cl0_x = c0x;
        s_cl0_y = c0y;
        lv_obj_align(s_cl0, LV_ALIGN_CENTER, c0x, c0y);
    }
    if (c1x != s_cl1_x || c1y != s_cl1_y) {
        s_cl1_x = c1x;
        s_cl1_y = c1y;
        lv_obj_align(s_cl1, LV_ALIGN_CENTER, c1x, c1y);
    }
}
