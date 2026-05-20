#include "current_sensor.h"

#include <Adafruit_INA219.h>
#include <Arduino.h>
#include <esp_log.h>

#include "display.h"

static const char *TAG = "INA219";

static Adafruit_INA219 ina219;
static bool sensorReady = false;

void initCurrentSensor() {
    ESP_LOGI(TAG, "Initializing INA219 on shared I2C bus...");

    // The display has already called Wire.begin() with the OLED pins; the
    // INA219 lives on the same bus, so we just hand the existing Wire to it.
    // Bus access MUST be serialized with the OLED via displayMutex -- racing
    // a 1KB OLED sendBuffer mid-transaction wedges the I2C controller.
    if (displayMutex == nullptr) {
        ESP_LOGE(TAG, "displayMutex not initialized; call initDisplay() first");
        sensorReady = false;
        return;
    }

    bool ok = false;
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        ok = ina219.begin();
        xSemaphoreGive(displayMutex);
    }

    if (!ok) {
        ESP_LOGE(TAG, "INA219 not found on I2C bus");
        sensorReady = false;
        return;
    }

    // 32V / 1A range gives ~40uA resolution -- plenty for stall detection
    // while keeping headroom for motor inrush. Tune if your shunt differs.
    ina219.setCalibration_32V_1A();
    sensorReady = true;
    ESP_LOGI(TAG, "INA219 ready");
}

float getCurrentMilliAmps(uint16_t samples) {
    if (!sensorReady || samples == 0 || displayMutex == nullptr) {
        return 0.0f;
    }

    float sum = 0.0f;
    uint16_t got = 0;
    for (uint16_t i = 0; i < samples; i++) {
        // Wait long enough to outlast a full OLED frame (~80 ms @ 100 kHz)
        // plus headroom. Never read the bus without the lock -- that races
        // U8g2's sendBuffer and corrupts the I2C controller.
        if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            float ma = ina219.getCurrent_mA();
            xSemaphoreGive(displayMutex);
            sum += fabsf(ma);
            got++;
        } else {
            ESP_LOGW(TAG, "displayMutex timeout; skipping sample");
        }
        vTaskDelay(1);
    }
    if (got == 0) {
        return 0.0f;
    }
    return sum / static_cast<float>(got);
}
