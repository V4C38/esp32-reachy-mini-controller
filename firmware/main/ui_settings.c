#include <stdio.h>
#include "ui_settings.h"

#include "config.h"
#include "lvgl.h"

static lv_obj_t *s_root;
static lv_obj_t *s_value;
static lv_obj_t *s_connect_btn;
static float *s_gain;
static void (*s_on_close)(void);
static void (*s_on_connect)(void);

static void update_label(void)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fx", *s_gain);
    lv_label_set_text(s_value, buf);
}

static void on_slider(lv_event_t *e)
{
    lv_obj_t *sl = lv_event_get_target(e);
    int v = lv_slider_get_value(sl);
    *s_gain = v / 10.f;
    if (*s_gain < GAIN_MIN) *s_gain = GAIN_MIN;
    if (*s_gain > GAIN_MAX) *s_gain = GAIN_MAX;
    update_label();
}

static void on_close(lv_event_t *e)
{
    (void)e;
    if (s_on_close) s_on_close();
}

static void on_connect(lv_event_t *e)
{
    (void)e;
    if (s_on_connect) s_on_connect();
}

void ui_settings_create(lv_obj_t *parent, void (*close_cb)(void), void (*connect_cb)(void), float *gain_ptr)
{
    s_on_close = close_cb;
    s_on_connect = connect_cb;
    s_gain = gain_ptr;

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_root, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(s_root, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *title = lv_label_create(s_root);
    lv_label_set_text(title, "Motion Multiplier");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 16, 16);

    /* Large hit target inset from the bezel — corner taps are noisy after
     * landscape rotation, and CLICKED alone drops presses that jitter. */
    lv_obj_t *close_btn = lv_button_create(s_root);
    lv_obj_set_size(close_btn, 72, 72);
    lv_obj_align(close_btn, LV_ALIGN_TOP_RIGHT, -8, 8);
    lv_obj_set_ext_click_area(close_btn, 20);
    lv_obj_add_flag(close_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(close_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(close_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(close_btn, 0, 0);
    lv_obj_set_style_pad_all(close_btn, 0, 0);
    lv_obj_set_style_shadow_width(close_btn, 0, 0);
    lv_obj_add_event_cb(close_btn, on_close, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(close_btn, on_close, LV_EVENT_SHORT_CLICKED, NULL);

    lv_obj_t *close_ring = lv_obj_create(close_btn);
    lv_obj_set_size(close_ring, 48, 48);
    lv_obj_center(close_ring);
    lv_obj_clear_flag(close_ring, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(close_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(close_ring, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(close_ring, 1, 0);
    lv_obj_set_style_radius(close_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_pad_all(close_ring, 0, 0);

    lv_obj_t *xlab = lv_label_create(close_ring);
    lv_label_set_text(xlab, "X");
    lv_obj_set_style_text_color(xlab, lv_color_hex(0xFFFFFF), 0);
    lv_obj_clear_flag(xlab, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(xlab);

    s_value = lv_label_create(s_root);
    lv_obj_set_style_text_color(s_value, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(s_value, &lv_font_montserrat_48, 0);
    lv_obj_align(s_value, LV_ALIGN_TOP_MID, 0, 70);
    update_label();

    lv_obj_t *slider = lv_slider_create(s_root);
    lv_obj_set_size(slider, 320, 40);
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, 20);
    lv_slider_set_range(slider, 1, 20); /* 0.1 .. 2.0 */
    lv_slider_set_value(slider, (int)(*s_gain * 10.f + 0.5f), LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_add_event_cb(slider, on_slider, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *lo = lv_label_create(s_root);
    lv_label_set_text(lo, "0.1x");
    lv_obj_set_style_text_color(lo, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(lo, slider, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 8);
    lv_obj_t *hi = lv_label_create(s_root);
    lv_label_set_text(hi, "2x");
    lv_obj_set_style_text_color(hi, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(hi, slider, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 8);

    /* SHORT_CLICKED only — CLICKED + SHORT_CLICKED would fire the handler twice. */
    s_connect_btn = lv_button_create(s_root);
    lv_obj_set_size(s_connect_btn, 260, 56);
    lv_obj_align(s_connect_btn, LV_ALIGN_BOTTOM_MID, 0, -24);
    lv_obj_set_ext_click_area(s_connect_btn, 16);
    lv_obj_add_flag(s_connect_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_connect_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(s_connect_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_color(s_connect_btn, lv_color_hex(0xFFFFFF), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(s_connect_btn, LV_OPA_30, LV_STATE_PRESSED);
    lv_obj_set_style_border_color(s_connect_btn, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(s_connect_btn, 1, 0);
    lv_obj_set_style_radius(s_connect_btn, 28, 0);
    lv_obj_add_event_cb(s_connect_btn, on_connect, LV_EVENT_SHORT_CLICKED, NULL);
    lv_obj_t *connect_lab = lv_label_create(s_connect_btn);
    lv_label_set_text(connect_lab, "CONNECT  TO  APP");
    lv_obj_set_style_text_color(connect_lab, lv_color_hex(0xFFFFFF), 0);
    lv_obj_clear_flag(connect_lab, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(connect_lab);
}

void ui_settings_show(bool show)
{
    if (show) {
        lv_obj_clear_flag(s_root, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_root);
    } else {
        lv_obj_add_flag(s_root, LV_OBJ_FLAG_HIDDEN);
    }
}

void ui_settings_set_connect_visible(bool visible)
{
    if (!s_connect_btn) return;
    if (visible) lv_obj_clear_flag(s_connect_btn, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(s_connect_btn, LV_OBJ_FLAG_HIDDEN);
}
