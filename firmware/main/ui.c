#include "ui.h"

#include <stdatomic.h>

#include "config.h"
#include "ui_face.h"
#include "ui_settings.h"

#include "bsp/esp-bsp.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "imu.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lvgl.h"

#define UI_TICK_MS 33
/* Gray Starting… until first link, then fall back to DISCONNECTED. */
#define BOOT_TIMEOUT_MS 15000

static const char *TAG = "ui";

static bool s_linked;
static bool s_linked_req;
static bool s_booting = true;
static uint32_t s_boot_start_ms;
static bool s_engaged;
static bool s_settings;
static atomic_bool s_connect_req;
static float s_gain = GAIN_DEFAULT;
static uint32_t s_press_ms;
static uint32_t s_last_tap_ms;
static int s_tap_count;
static bool s_btn_stable;      /* true = pressed (GPIO low) */
static bool s_btn_raw;
static uint32_t s_btn_change_ms;

static lv_obj_t *s_unlinked_lab;

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

static void apply_status_label(void)
{
    if (!s_unlinked_lab) return;
    if (s_linked) {
        lv_obj_add_flag(s_unlinked_lab, LV_OBJ_FLAG_HIDDEN);
    } else if (s_booting) {
        lv_label_set_text(s_unlinked_lab, "Starting...");
        lv_obj_set_style_text_color(s_unlinked_lab, lv_color_hex(0x666666), 0);
        lv_obj_clear_flag(s_unlinked_lab, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_label_set_text(s_unlinked_lab, "DISCONNECTED");
        lv_obj_set_style_text_color(s_unlinked_lab, lv_color_hex(0xCC2222), 0);
        lv_obj_clear_flag(s_unlinked_lab, LV_OBJ_FLAG_HIDDEN);
    }
    ui_settings_set_connect_visible(!s_linked && !s_booting);
}

static void apply_face(void)
{
    /* 0 disengaged (closed eyes), 1 engaged (open eyes). */
    ui_face_set_mode(s_engaged ? 1 : 0);
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
    }
    apply_face();
}

static void settings_open(void)
{
    set_engaged(false);
    s_settings = true;
    s_tap_count = 0;
    ui_settings_show(true);
}

static void settings_close(void)
{
    persist_gain();
    ui_settings_show(false);
    s_settings = false;
    s_tap_count = 0;
    apply_face();
}

static void settings_connect(void)
{
    atomic_store(&s_connect_req, true);
    ESP_LOGI(TAG, "connect to app requested");
}

/* Double-tap toggles settings. Touch no longer engages or disengages. */
static void on_touch(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    uint32_t now = lv_tick_get();

    if (code == LV_EVENT_PRESSED) {
        s_press_ms = now;
        return;
    }
    if (code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) return;

    /* Ignore long presses so accidental holds do not count as taps. */
    if ((now - s_press_ms) >= TOUCH_DOUBLE_TAP_MS) return;

    if (now - s_last_tap_ms < TOUCH_DOUBLE_TAP_MS) s_tap_count++;
    else s_tap_count = 1;
    s_last_tap_ms = now;
    if (s_tap_count >= 2) {
        s_tap_count = 0;
        if (s_settings) settings_close();
        else settings_open();
    }
}

/* BOOT (GPIO0, active-low). Never PWR / EXIO4. */
static void poll_boot_button(uint32_t now)
{
    bool raw = gpio_get_level(BUTTON_GPIO) == 0;
    if (raw != s_btn_raw) {
        s_btn_raw = raw;
        s_btn_change_ms = now;
        return;
    }
    if ((now - s_btn_change_ms) < BUTTON_DEBOUNCE_MS) return;
    if (raw == s_btn_stable) return;

    s_btn_stable = raw;
    if (!raw || s_settings) return; /* rising edge of press only */
    set_engaged(!s_engaged);
}

static void ui_timer_cb(lv_timer_t *t)
{
    (void)t;
    uint32_t now = lv_tick_get();

    if (s_linked_req != s_linked) {
        s_linked = s_linked_req;
        if (s_linked) s_booting = false;
        apply_status_label();
    }
    if (s_booting && (now - s_boot_start_ms) >= BOOT_TIMEOUT_MS) {
        s_booting = false;
        apply_status_label();
    }
    poll_boot_button(now);
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

    /* BOOT button only (GPIO0). Do not touch PWR / TCA9554 EXIO4. */
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));
    s_btn_raw = gpio_get_level(BUTTON_GPIO) == 0;
    s_btn_stable = s_btn_raw;
    s_btn_change_ms = 0;

    lv_display_t *disp = bsp_display_start();
    if (!disp) return ESP_FAIL;
    (void)disp;

    /* Landscape is baked into the local Waveshare BSP at add_disp — do not
     * rotate here. Panel stays at stock brightness from init cmds. */
    if (!bsp_display_lock(1000)) {
        ESP_LOGE(TAG, "LVGL lock timeout");
        return ESP_FAIL;
    }
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xEDEBE6), 0);
    ui_face_create(scr);

    s_unlinked_lab = lv_label_create(scr);
    lv_obj_set_style_text_font(s_unlinked_lab, &lv_font_montserrat_14, 0);
    lv_obj_align(s_unlinked_lab, LV_ALIGN_BOTTOM_MID, 0, -12);

    ui_settings_create(scr, settings_close, settings_connect, &s_gain);
    s_boot_start_ms = lv_tick_get();
    apply_status_label();
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr, on_touch, LV_EVENT_ALL, NULL);
    apply_face();
    lv_timer_create(ui_timer_cb, UI_TICK_MS, NULL);
    bsp_display_unlock();

    ESP_LOGI(TAG, "UI ready gain=%.1f", s_gain);
    return ESP_OK;
}

void ui_set_linked(bool linked)
{
    s_linked_req = linked;
    if (linked) s_booting = false;
}

bool ui_linked(void) { return s_linked; }
bool ui_engaged(void) { return s_engaged; }
float ui_get_gain(void) { return s_gain; }

bool ui_connect_pending(void)
{
    return atomic_load(&s_connect_req);
}

void ui_clear_connect_request(void)
{
    atomic_store(&s_connect_req, false);
}
