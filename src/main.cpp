#include "Arduino.h"
#include "OneButton.h"
#include "components/HeaderBar.h"
#include "esp_log.h"
#include "ossm/Events.h"
#include "ossm/OSSM.h"
#include "ossm/state/state.h"
#include "services/board.h"
#include "services/communication/mqtt.h"
#include "services/communication/nimble.h"
#include "services/current_sensor.h"
#include "services/display.h"
#include "services/encoder.h"
#include "services/led.h"
#include "services/stepper.h"
#include "services/wm.h"

namespace sml = boost::sml;
using namespace sml;

/*
 *  ██████╗ ███████╗███████╗███╗   ███╗
 * ██╔═══██╗██╔════╝██╔════╝████╗ ████║
 * ██║   ██║███████╗███████╗██╔████╔██║
 * ██║   ██║╚════██║╚════██║██║╚██╔╝██║
 * ╚██████╔╝███████║███████║██║ ╚═╝ ██║
 *  ╚═════╝ ╚══════╝╚══════╝╚═╝     ╚═╝
 *
 * Welcome to the open source sex machine!
 * This is a product of Kinky Makers and is licensed under the MIT license.
 *
 * Research and Desire is a financial sponsor of this project.
 *
 * But our biggest sponsor is you! If you want to support this project, please
 * contribute, fork, branch and share!
 */

OneButton button(Pins::Remote::encoderSwitch, false);

// ===== TEMP MOTOR DIRECTION TEST ============================================
// Bypasses the state machine entirely. Pulses STEP continuously and flips DIR
// every second. Watch the carriage: it should crawl one way for ~1 s, then
// the other way for ~1 s, repeating forever.
//
// REMOVE THIS BLOCK (and the call from setup()) once direction is confirmed.
// ============================================================================
#define MOTOR_DIR_TEST 1
#if MOTOR_DIR_TEST
#include "constants/Pins.h"

static void motorDirTestTask(void *) {
    pinMode(Pins::Driver::motorStepPin, OUTPUT);
    pinMode(Pins::Driver::motorDirectionPin, OUTPUT);
    pinMode(Pins::Driver::motorEnablePin, OUTPUT);

    // Enable the driver. Most stepper/servo drivers treat ENA as active-low.
    // If your motor is silent during the test, try flipping this to HIGH.
    digitalWrite(Pins::Driver::motorEnablePin, LOW);

    bool dir = false;
    uint32_t lastFlip = millis();
    digitalWrite(Pins::Driver::motorDirectionPin, dir);
    ESP_LOGI("DIRTEST", "Starting direction test. DIR=%d", dir);

    // ~2 kHz step rate -> slow, audible crawl. Adjust delayMicroseconds
    // values if you want it faster.
    while (true) {
        if (millis() - lastFlip >= 4000) {
            dir = !dir;
            digitalWrite(Pins::Driver::motorDirectionPin, dir);
            lastFlip = millis();
            ESP_LOGI("DIRTEST", "DIR flipped -> %d", dir);
        }
        digitalWrite(Pins::Driver::motorStepPin, HIGH);
        delayMicroseconds(250);
        digitalWrite(Pins::Driver::motorStepPin, LOW);
        delayMicroseconds(250);
    }
}

static void startMotorDirTest() {
    xTaskCreatePinnedToCore(motorDirTestTask, "dirTest",
                            4 * configMINIMAL_STACK_SIZE, nullptr,
                            configMAX_PRIORITIES - 1, nullptr, 1);
}
#endif
// ===== END TEMP MOTOR DIRECTION TEST ========================================

void setup() {
    // Suppress verbose GPIO configuration logs
    esp_log_level_set("gpio", ESP_LOG_WARN);

#if MOTOR_DIR_TEST
    // Bring up just enough to drive the pins. Skip everything else.
    ESP_LOGI("MAIN", "MOTOR_DIR_TEST mode -- skipping normal init");
    startMotorDirTest();
    return;
#endif

    /** Board setup */
    initBoard();

    ESP_LOGD("MAIN", "Starting OSSM");

    // Display
    initDisplay();

    // INA219 current sensor (shares the display I2C bus)
    initCurrentSensor();

    // Initialize header bar task
    initHeaderBar();

    // Create OSSM instance for backward compatibility (BLE command handling)
    ossm = new OSSM();

    // Initialize state machine after global state is set up
    initStateMachine();

    // ialize LED for BLE and machine status indication
    ESP_LOGI("MAIN", "LED initialized for BLE and machine status indication");
    updateLEDForMachineStatus();  // Set initial LED state

    // // link functions to be called on events.
    button.attachClick([]() { stateMachine->process_event(ButtonPress{}); });
    button.attachDoubleClick(
        []() { stateMachine->process_event(DoublePress{}); });
    button.attachLongPressStart(
        []() { stateMachine->process_event(LongPress{}); });

    xTaskCreatePinnedToCore(
        [](void *pvParameters) {
            while (true) {
                button.tick();
                vTaskDelay(25 / portTICK_PERIOD_MS);
            }
        },
        "buttonTask", 4 * configMINIMAL_STACK_SIZE, nullptr,
        configMAX_PRIORITIES - 1, nullptr, 0);

    // Initialize NimBLE only when in menu.idle state
    xTaskCreatePinnedToCore(
        [](void *pvParameters) {
            bool initialized = false;
            while (true) {
                if ((stateMachine->is("menu.idle"_s) ||
                     stateMachine->is("error.idle"_s)) &&
                    !initialized) {
                    ESP_LOGD("MAIN", "Initializing communication services");
                    initNimble();
                    initWM();
                    initMQTT();
                    initialized = true;
                    vTaskDelete(nullptr);
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        },
        "initNimbleTask", 32 * configMINIMAL_STACK_SIZE, nullptr,
        configMAX_PRIORITIES - 1, nullptr, 0);
};

void loop() { vTaskDelete(nullptr); };
