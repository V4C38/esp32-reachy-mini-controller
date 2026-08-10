#include "ui.h"

#include <math.h>

#include "config.h"
#include "ui_face.h"
#include "ui_settings.h"

#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "imu.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lvgl.h"

#define UI_TICK_MS 33

/* Relative rotation, in rad, that deflects the face to its limit. */
#define MOTION_ROT_FS 0.20f
/* How fast the neutral pose trails the device, in seconds. */
#define MOTION_REF_TAU 1.5f
#define MOTION_LPF 0.45f
#define MOTION_ROT_W 0.90f
#define MOTION_POS_W 0.75f

static const char *TAG = "ui";

static bool s_linked;
static bool s_linked_req;
static bool s_engaged;
static bool s_settings;
static bool s_reset_req;
static float s_gain = GAIN_DEFAULT;
static uint32_t s_press_ms;
static uint32_t s_last_tap_ms;
static int s_tap_count;
static bool s_pressing;

static float s_qref[4] = {1.f, 0.f, 0.f, 0.f};
static bool s_qref_valid;
static float s_mx, s_my, s_mr;
static int s_brightness = -1;
static int s_brightness_pending = -1;
static uint32_t s_keepalive_ms;

static float clamp1(float v)
{
    if (v > 1.f) return 1.f;
    if (v < -1.f) return -1.f;
    return v;
}

/* out = conj(a) * b */
static void q_rel(const float a[4], const float b[4], float out[4])
{
    float aw = a[0], ax = -a[1], ay = -a[2], az = -a[3];
    out[0] = aw * b[0] - ax * b[1] - ay * b[2] - az * b[3];
    out[1] = aw * b[1] + ax * b[0] + ay * b[3] - az * b[2];
    out[2] = aw * b[2] - ax * b[3] + ay * b[0] + az * b[1];
    out[3] = aw * b[3] + ax * b[2] - ay * b[1] + az * b[0];
}

/* out = conj(q) * v * q — world vector expressed in body axes. */
static void q_rotate_inv(const float q[4], const float v[3], float out[3])
{
    float qw = q[0], qx = -q[1], qy = -q[2], qz = -q[3];
    float uvx = qy * v[2] - qz * v[1];
    float uvy = qz * v[0] - qx * v[2];
    float uvz = qx * v[1] - qy * v[0];
    float uuvx = qy * uvz - qz * uvy;
    float uuvy = qz * uvx - qx * uvz;
    float uuvz = qx * uvy - qy * uvx;
    out[0] = v[0] + 2.f * (qw * uvx + uuvx);
    out[1] = v[1] + 2.f * (qw * uvy + uuvy);
    out[2] = v[2] + 2.f * (qw * uvz + uuvz);
}

static void persist_gain(void)
{
    nvs_handle_t h;
    if (nvs_open("rmc", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, "gain_x10", (uint32_t)(s_gain * 10.f + 0.5f));
    nvs_commit(h);
    nvs_close(h);
}

static void load_gain(void)
{
    nvs_handle_t h;
    if (nvs_open("rmc", NVS_READONLY, &h) != ESP_OK) return;
    uint32_t v = 10;
    if (nvs_get_u32(h, "gain_x10", &v) == ESP_OK) {
        s_gain = v / 10.f;
        if (s_gain < GAIN_MIN) s_gain = GAIN_MIN;
        if (s_gain > GAIN_MAX) s_gain = GAIN_MAX;
    }
    nvs_close(h);
}

/* Queue a brightness level. Applied only from the LVGL task so panel IO
 * never races a QSPI flush from another context. */
static void request_brightness(void)
{
    int b;
    if (s_engaged || s_settings) b = UI_BRIGHTNESS_ENGAGED;
    else if (s_linked) b = UI_BRIGHTNESS_IDLE;
    else b = UI_BRIGHTNESS_DISCONNECTED;
    s_brightness_pending = b;
}

static void flush_brightness(void)
{
    if (s_brightness_pending < 0 || s_brightness_pending == s_brightness) return;
    s_brightness = s_brightness_pending;
    if (bsp_display_brightness_set(s_brightness) != ESP_OK) {
        /* Panel IO failed — retry next tick instead of caching a lie. */
        s_brightness = -1;
    }
}

static void apply_brightness(void)
{
    request_brightness();
    flush_brightness();
}

/* Must run under the LVGL lock (timer context or bsp_display_lock). */
static void panel_keepalive(void)
{
    /* Brightness alone is not enough — after USB RTS / WiFi the CO5300 can
     * lose Sleep-Out / Display-On while GRAM is empty. */
    (void)bsp_display_reassert();
    s_brightness = -1;
    request_brightness();
    flush_brightness();
    lv_obj_invalidate(lv_screen_active());
}

void ui_reassert_panel(void)
{
    if (!bsp_display_lock(500)) {
        ESP_LOGW(TAG, "panel reassert: LVGL lock timeout");
        return;
    }
    panel_keepalive();
    bsp_display_unlock();
}

static void apply_face(void)
{
    int mode = 2;
    if (s_linked) mode = s_engaged ? 1 : 0;
    ui_face_set_mode(mode);
}

static void set_engaged(bool on)
{
    if (s_engaged == on) return;
    s_engaged = on;
    if (on) {
        float a[3], g[3];
        imu_get_raw_mapped(a, g);
        if (!imu_gravity_sane()) {
            ESP_LOGW(TAG,
                     "IMU_MAP sanity: expected accel ~ (0, +9.8, 0) in hold pose; "
                     "got [%.2f %.2f %.2f] — check IMU_MAP_* in config.h",
                     a[0], a[1], a[2]);
        } else {
            ESP_LOGI(TAG, "engage accel=[%.2f %.2f %.2f] gyro=[%.3f %.3f %.3f]",
                     a[0], a[1], a[2], g[0], g[1], g[2]);
        }
        imu_reset_displacement();
        s_qref_valid = false;
    }
    apply_face();
    apply_brightness();
}

static void settings_open(void)
{
    set_engaged(false);
    s_settings = true;
    ui_settings_show(true);
    apply_brightness();
}

static void settings_close(void)
{
    persist_gain();
    ui_settings_show(false);
    s_settings = false;
    apply_face();
    apply_brightness();
}

static void settings_reset(void)
{
    s_reset_req = true;
    imu_reset_displacement();
    s_qref_valid = false;
}

static void on_touch(lv_event_t *e)
{
    if (s_settings) return;

    lv_event_code_t code = lv_event_get_code(e);
    uint32_t now = lv_tick_get();

    if (code == LV_EVENT_PRESSED) {
        s_pressing = true;
        s_press_ms = now;
        return;
    }
    if (code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) return;

    uint32_t held = now - s_press_ms;
    s_pressing = false;
    set_engaged(false);

    if (held >= TOUCH_ENGAGE_MS) return;

    if (now - s_last_tap_ms < TOUCH_DOUBLE_TAP_MS) s_tap_count++;
    else s_tap_count = 1;
    s_last_tap_ms = now;
    if (s_tap_count >= 2) {
        s_tap_count = 0;
        settings_open();
    }
}

static void motion_update(void)
{
    imu_integrate_state_t st;
    imu_get_state(&st);
    if (!st.ready) return;

    float q[4] = {st.q[0], st.q[1], st.q[2], st.q[3]};
    if (!s_qref_valid) {
        for (int i = 0; i < 4; i++) s_qref[i] = q[i];
        s_qref_valid = true;
    }

    float rel[4];
    q_rel(s_qref, q, rel);
    if (rel[0] < 0.f) {
        for (int i = 0; i < 4; i++) rel[i] = -rel[i];
    }
    /* Small-angle vector part ~= rotation about each body axis, in rad. */
    float rot_x = 2.f * rel[1]; /* pitch, about the right axis */
    float rot_y = 2.f * rel[2]; /* pan, about the up axis */
    float rot_z = 2.f * rel[3]; /* roll, about the screen normal */

    /* The neutral pose trails the device, so a held attitude drifts back to
     * centre while quick moves still deflect the face. */
    float k = (UI_TICK_MS / 1000.f) / MOTION_REF_TAU;
    float dot = 0.f;
    for (int i = 0; i < 4; i++) dot += s_qref[i] * q[i];
    float sign = dot < 0.f ? -1.f : 1.f;
    float n = 0.f;
    for (int i = 0; i < 4; i++) {
        s_qref[i] += k * (sign * q[i] - s_qref[i]);
        n += s_qref[i] * s_qref[i];
    }
    n = sqrtf(n);
    if (n < 1e-6f) {
        s_qref_valid = false;
        return;
    }
    for (int i = 0; i < 4; i++) s_qref[i] /= n;

    float p_body[3];
    q_rotate_inv(q, st.p, p_body);
    float pos_scale = st.p_max > 1e-6f ? 1.f / st.p_max : 0.f;

    float tx = MOTION_ROT_W * (rot_y / MOTION_ROT_FS) + MOTION_POS_W * (p_body[0] * pos_scale);
    float ty = MOTION_ROT_W * (-rot_x / MOTION_ROT_FS) + MOTION_POS_W * (p_body[1] * pos_scale);
    float tr = rot_z / MOTION_ROT_FS;

    s_mx += MOTION_LPF * (clamp1(tx) - s_mx);
    s_my += MOTION_LPF * (clamp1(ty) - s_my);
    s_mr += MOTION_LPF * (clamp1(tr) - s_mr);

    ui_face_set_motion(s_mx, s_my, s_mr);
}

static void ui_timer_cb(lv_timer_t *t)
{
    (void)t;
    uint32_t now = lv_tick_get();

    if (s_linked_req != s_linked) {
        s_linked = s_linked_req;
        apply_face();
        request_brightness();
    }
    flush_brightness();

    if (s_pressing && !s_settings && !s_engaged &&
        (now - s_press_ms) >= TOUCH_ENGAGE_MS) {
        s_tap_count = 0;
        set_engaged(true);
    }

    /* Skip high-rate motion invalidates while settings are open or offline —
     * those used to starve the QSPI path. Still keep the panel alive: after
     * USB-UART RTS resets / WiFi bring-up the CO5300 can lose Sleep-Out,
     * Display-On, GRAM and brightness while the MCU still thinks the last
     * frame is on screen. */
    if (!s_settings && s_linked) {
        motion_update();
        /* DISPON is cheap; covers a mid-session serial open that blanked us. */
        if (now - s_keepalive_ms >= 2000) {
            s_keepalive_ms = now;
            (void)bsp_display_reassert();
        }
    } else if (now - s_keepalive_ms >= 1000) {
        s_keepalive_ms = now;
        panel_keepalive();
    }
}

esp_err_t ui_init(void)
{
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    load_gain();
    lv_display_t *disp = bsp_display_start();
    if (!disp) return ESP_FAIL;
    (void)disp;

    /* Landscape is baked into the local Waveshare BSP at add_disp — do not
     * rotate here. Brightness is applied under the LVGL lock below (once),
     * then only from ui_timer_cb / touch handlers. */
    request_brightness();

    if (!bsp_display_lock(1000)) {
        ESP_LOGE(TAG, "LVGL lock timeout");
        return ESP_FAIL;
    }
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xEDEBE6), 0);
    ui_face_create(scr);
    ui_settings_create(scr, settings_close, settings_reset, &s_gain);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr, on_touch, LV_EVENT_ALL, NULL);
    apply_face();
    flush_brightness();
    lv_timer_create(ui_timer_cb, UI_TICK_MS, NULL);
    bsp_display_unlock();

    ESP_LOGI(TAG, "UI ready gain=%.1f", s_gain);
    return ESP_OK;
}

void ui_set_linked(bool linked) { s_linked_req = linked; }
bool ui_linked(void) { return s_linked; }
bool ui_engaged(void) { return s_engaged; }
float ui_get_gain(void) { return s_gain; }

bool ui_take_reset_request(void)
{
    if (!s_reset_req) return false;
    s_reset_req = false;
    return true;
}
