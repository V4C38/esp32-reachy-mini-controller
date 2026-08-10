#include "imu.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#undef M_PI
#include "qmi8658.h"

#define IMU_PROBE_TIMEOUT_MS 100
#define QMI8658_RESET_REGISTER 0x60
#define QMI8658_RESET_COMMAND 0xB0
#define DEG_TO_RAD 0.01745329252f

static const char *TAG = "imu";
static qmi8658_dev_t s_dev;
static bool s_hw_ok;
static imu_integrate_state_t s_state;
static SemaphoreHandle_t s_lock;
static float s_accel[3];
static float s_gyro[3];

/* Intended idle hold: screen toward user, USB+buttons down → +Y up.
 * Accel should read ~g along +Y. Accept if |ay| is the dominant component
 * and positive, within a loose cone so a mild tip still passes. */
#define GRAVITY_SANE_MIN_Y 6.0f
#define GRAVITY_SANE_MAX_XZ 6.0f

static esp_err_t detect_address(i2c_master_bus_handle_t bus, uint8_t *address)
{
    const uint8_t candidates[] = {QMI8658_ADDRESS_HIGH, QMI8658_ADDRESS_LOW};
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        if (i2c_master_probe(bus, candidates[i], IMU_PROBE_TIMEOUT_MS) == ESP_OK) {
            *address = candidates[i];
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t configure(void)
{
    esp_err_t ret = qmi8658_write_register(&s_dev, QMI8658_RESET_REGISTER, QMI8658_RESET_COMMAND);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = qmi8658_write_register(&s_dev, QMI8658_CTRL1, 0x60);
    if (ret != ESP_OK) return ret;

    if ((ret = qmi8658_set_accel_range(&s_dev, QMI8658_ACCEL_RANGE_8G)) != ESP_OK) return ret;
    if ((ret = qmi8658_set_accel_odr(&s_dev, QMI8658_ACCEL_ODR_250HZ)) != ESP_OK) return ret;
    if ((ret = qmi8658_set_gyro_range(&s_dev, QMI8658_GYRO_RANGE_512DPS)) != ESP_OK) return ret;
    if ((ret = qmi8658_set_gyro_odr(&s_dev, QMI8658_GYRO_ODR_250HZ)) != ESP_OK) return ret;
    qmi8658_set_accel_unit_mps2(&s_dev, true);
    qmi8658_set_gyro_unit_dps(&s_dev, true);
    return qmi8658_enable_sensors(&s_dev, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
}

esp_err_t imu_init(i2c_master_bus_handle_t bus)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    imu_integrate_init(&s_state);
    uint8_t addr = 0;
    esp_err_t ret = detect_address(bus, &addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "QMI8658 not found");
        return ret;
    }
    ret = qmi8658_init(&s_dev, bus, addr);
    if (ret != ESP_OK) return ret;
    ret = configure();
    if (ret != ESP_OK) return ret;
    s_hw_ok = true;
    ESP_LOGI(TAG, "QMI8658 ready at 0x%02x — calibrating (hold still)", addr);

    /* Boot calibration window */
    int attempts = 0;
    while (!s_state.ready && attempts < 5) {
        for (int i = 0; i < 300; i++) {
            qmi8658_data_t data;
            if (qmi8658_read_sensor_data(&s_dev, &data) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(4));
                continue;
            }
            float ax = IMU_MAP_X(data.accelX, data.accelY, data.accelZ);
            float ay = IMU_MAP_Y(data.accelX, data.accelY, data.accelZ);
            float az = IMU_MAP_Z(data.accelX, data.accelY, data.accelZ);
            float gx = IMU_MAP_X(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;
            float gy = IMU_MAP_Y(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;
            float gz = IMU_MAP_Z(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;
            float accel[3] = {ax, ay, az};
            float gyro[3] = {gx, gy, gz};
            s_accel[0] = ax; s_accel[1] = ay; s_accel[2] = az;
            s_gyro[0] = gx; s_gyro[1] = gy; s_gyro[2] = gz;
            if (imu_integrate_calib_sample(&s_state, gyro, accel, 0.004f)) break;
            vTaskDelay(pdMS_TO_TICKS(4));
        }
        attempts++;
        if (!s_state.ready) {
            ESP_LOGW(TAG, "calibration variance high; retry %d", attempts);
        }
    }
    if (!s_state.ready) {
        ESP_LOGE(TAG, "calibration failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "gyro bias [%.4f %.4f %.4f]",
             s_state.gyro_bias[0], s_state.gyro_bias[1], s_state.gyro_bias[2]);
    ESP_LOGI(TAG, "mapped accel [%.2f %.2f %.2f] — expect ~[0 0 +9.8] flat on desk, ~[0 +9.8 0] in hold pose",
             s_accel[0], s_accel[1], s_accel[2]);
    return ESP_OK;
}

bool imu_ready(void) { return s_hw_ok && s_state.ready; }
bool imu_still(void) { return s_state.still; }

void imu_reset_displacement(void)
{
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    imu_integrate_reset_motion(&s_state);
    if (s_lock) xSemaphoreGive(s_lock);
}

void imu_get_state(imu_integrate_state_t *out)
{
    if (!out) return;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_state;
    if (s_lock) xSemaphoreGive(s_lock);
}

bool imu_update(float dt)
{
    if (!s_hw_ok) return false;
    bool data_ready = false;
    if (qmi8658_is_data_ready(&s_dev, &data_ready) != ESP_OK || !data_ready) return false;
    qmi8658_data_t data;
    if (qmi8658_read_sensor_data(&s_dev, &data) != ESP_OK) return false;

    float ax = IMU_MAP_X(data.accelX, data.accelY, data.accelZ);
    float ay = IMU_MAP_Y(data.accelX, data.accelY, data.accelZ);
    float az = IMU_MAP_Z(data.accelX, data.accelY, data.accelZ);
    float gx = IMU_MAP_X(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;
    float gy = IMU_MAP_Y(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;
    float gz = IMU_MAP_Z(data.gyroX, data.gyroY, data.gyroZ) * DEG_TO_RAD;
    float accel[3] = {ax, ay, az};
    float gyro[3] = {gx, gy, gz};

    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_accel[0] = ax; s_accel[1] = ay; s_accel[2] = az;
    s_gyro[0] = gx; s_gyro[1] = gy; s_gyro[2] = gz;
    if (!s_state.ready) {
        imu_integrate_calib_sample(&s_state, gyro, accel, dt);
    } else {
        imu_integrate_step(&s_state, gyro, accel, dt);
    }
    if (s_lock) xSemaphoreGive(s_lock);
    return true;
}

void imu_get_raw_mapped(float accel[3], float gyro[3])
{
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    if (accel) {
        accel[0] = s_accel[0]; accel[1] = s_accel[1]; accel[2] = s_accel[2];
    }
    if (gyro) {
        gyro[0] = s_gyro[0]; gyro[1] = s_gyro[1]; gyro[2] = s_gyro[2];
    }
    if (s_lock) xSemaphoreGive(s_lock);
}

bool imu_gravity_sane(void)
{
    float a[3];
    imu_get_raw_mapped(a, NULL);
    return (a[1] >= GRAVITY_SANE_MIN_Y) &&
           (fabsf(a[0]) <= GRAVITY_SANE_MAX_XZ) &&
           (fabsf(a[2]) <= GRAVITY_SANE_MAX_XZ);
}
