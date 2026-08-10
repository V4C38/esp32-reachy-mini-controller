#include "pmu.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pmu";

#define AXP2101_ADDR                 0x34
#define AXP2101_REG_STATUS1          0x00
#define AXP2101_REG_IC_TYPE          0x03
#define AXP2101_REG_CHARGE_GAUGE     0x18
#define AXP2101_REG_ADC_CHANNEL      0x30
#define AXP2101_REG_ADC0             0x34
#define AXP2101_REG_ADC1             0x35
#define AXP2101_REG_ICC_CHG          0x62
#define AXP2101_REG_CV_CHG           0x64
#define AXP2101_REG_BAT_DET          0x68

#define AXP2101_CHIP_ID              0x4A
#define AXP2101_CHG_CUR_400MA        10
#define AXP2101_CHG_VOL_4V2          3

static i2c_master_dev_handle_t s_dev;

static esp_err_t axp_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 100);
}

static esp_err_t axp_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t axp_update(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t cur = 0;
    esp_err_t err = axp_read(reg, &cur, 1);
    if (err != ESP_OK) return err;
    return axp_write(reg, (uint8_t)((cur & (uint8_t)~mask) | (value & mask)));
}

esp_err_t pmu_init(i2c_master_bus_handle_t bus)
{
    if (!bus) return ESP_ERR_INVALID_ARG;

    if (i2c_master_probe(bus, AXP2101_ADDR, 100) != ESP_OK) {
        ESP_LOGW(TAG, "AXP2101 not found — battery path unavailable");
        return ESP_ERR_NOT_FOUND;
    }

    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP2101_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &s_dev);
    if (err != ESP_OK) return err;

    uint8_t chip = 0;
    err = axp_read(AXP2101_REG_IC_TYPE, &chip, 1);
    if (err != ESP_OK || chip != AXP2101_CHIP_ID) {
        ESP_LOGW(TAG, "unexpected PMU id 0x%02x (err=%s)", chip, esp_err_to_name(err));
        return ESP_FAIL;
    }

    /* Board has no battery NTC on TS — leave it enabled and charging stalls /
     * battery path looks empty when VBUS drops. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(axp_update(AXP2101_REG_ADC_CHANNEL, 0x02, 0x00));
    /* Batt + VBUS + system ADC */
    ESP_ERROR_CHECK_WITHOUT_ABORT(axp_update(AXP2101_REG_ADC_CHANNEL, 0x0D, 0x0D));
    /* Battery presence detect */
    ESP_ERROR_CHECK_WITHOUT_ABORT(axp_update(AXP2101_REG_BAT_DET, 0x01, 0x01));
    /* Cell charge + fuel gauge */
    ESP_ERROR_CHECK_WITHOUT_ABORT(axp_update(AXP2101_REG_CHARGE_GAUGE, 0x0A, 0x0A));
    /* 400 mA / 4.2 V charge profile (Waveshare demo defaults) */
    ESP_ERROR_CHECK_WITHOUT_ABORT(axp_update(AXP2101_REG_ICC_CHG, 0x1F, AXP2101_CHG_CUR_400MA));
    ESP_ERROR_CHECK_WITHOUT_ABORT(axp_update(AXP2101_REG_CV_CHG, 0x03, AXP2101_CHG_VOL_4V2));

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t status1 = 0;
    axp_read(AXP2101_REG_STATUS1, &status1, 1);
    bool bat = (status1 & 0x08) != 0;
    bool vbus = (status1 & 0x20) != 0;

    uint16_t mv = 0;
    if (bat) {
        uint8_t hi = 0, lo = 0;
        if (axp_read(AXP2101_REG_ADC0, &hi, 1) == ESP_OK &&
            axp_read(AXP2101_REG_ADC1, &lo, 1) == ESP_OK) {
            mv = (uint16_t)(((hi & 0x1F) << 8) | lo);
        }
    }

    if (bat) {
        ESP_LOGI(TAG, "AXP2101 ready — battery %u mV (VBUS %s)",
                 (unsigned)mv, vbus ? "in" : "out");
    } else {
        ESP_LOGW(TAG, "AXP2101 ready — no battery detected; unplugging USB will power off");
    }
    return ESP_OK;
}
