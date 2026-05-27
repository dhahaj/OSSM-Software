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
#include "utils/update.h"

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

// OneButton(pin, activeLow, pullupActive). The remote PCB ties one side of the
// switch to GND and uses an external pull-up to 3.3V on GPIO 35, so the line
// idles HIGH and pulls LOW when pressed (active-low). pullupActive is false
// because GPIO 35 is input-only and has no internal pull resistors regardless.
OneButton button(Pins::Remote::encoderSwitch, true, false);

// ===== TEMP MOTOR DIRECTION TEST ============================================
// Bypasses the state machine entirely. Pulses STEP continuously and flips DIR
// every second. Watch the carriage: it should crawl one way for ~1 s, then
// the other way for ~1 s, repeating forever.
//
// REMOVE THIS BLOCK (and the call from setup()) once direction is confirmed.
// ============================================================================
#define MOTOR_DIR_TEST 0
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
    ESP_LOGI("DIRTEST", "Starting direction test. DIR=%d", dir);

    // ~2 kHz step rate -> slow, audible crawl. Adjust delayMicroseconds
    // values if you want it faster.
    while (true) {
        if (millis() - lastFlip >= 2000) {
            dir = !dir;
            digitalWrite(Pins::Driver::motorDirectionPin, dir);
            digitalWrite(Pins::Driver::motorEnablePin, dir);  // Ensure driver is enabled
            digitalWrite(Pins::Driver::motorStepPin, dir);

            lastFlip = millis();
            ESP_LOGI("DIRTEST", "DIR flipped -> %d", dir);
        }
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

    // If the user requested an OTA on the previous boot, run it now on a
    // clean heap before any other subsystem (BLE, WiFiManager portal,
    // display, motor, state machine) initializes. Returns immediately if no
    // OTA is pending; never returns if one is (reboots into new firmware on
    // success, or back into normal boot on failure).
    runPendingOTAIfRequested();

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
