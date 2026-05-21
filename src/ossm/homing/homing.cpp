#include "homing.h"

#include "constants/Config.h"
#include "constants/Pins.h"
#include "constants/UserConfig.h"
#include "ossm/Events.h"
#include "ossm/state/calibration.h"
#include "ossm/state/error.h"
#include "ossm/state/state.h"
#include "services/current_sensor.h"
#include "services/led.h"
#include "services/stepper.h"
#include "services/tasks.h"

namespace sml = boost::sml;
using namespace sml;

namespace homing {

void clearHoming() {
    ESP_LOGD("Homing", "Homing started");

    // Set homing active flag for LED indication
    setHomingActive(true);

    calibration.isForward = true;

    // Set acceleration and deceleration in steps/s^2
    stepper->setAcceleration(1000_mm);
    // Set speed in steps/s
    stepper->setSpeedInHz(25_mm);

    // Clear the stored values.
    calibration.measuredStrokeSteps = 0;

    // Recalibrate the current sensor offset (idle motor current in mA).
    calibration.currentSensorOffset = getCurrentMilliAmps(200);
    ESP_LOGI("Homing", "Current sensor idle offset: %.1f mA",
             calibration.currentSensorOffset);
}

static void startHomingTask(void *pvParameters) {
    TickType_t xTaskStartTime = xTaskGetTickCount();

#ifdef AJ_DEVELOPMENT_HARDWARE
    stepper->setCurrentPosition(0);
    stepper->forceStopAndNewPosition(0);
    stateMachine->process_event(Done{});
    vTaskDelete(nullptr);
    return;
#endif

#ifdef FAKE_HOMING
    // Bypass physical homing for bench testing without a hard stop / current
    // sensor. Pretends the carriage homed cleanly with FAKE_STROKE_MM of
    // usable travel and zeros position at the current physical location.
    {
        constexpr float fakeStrokeMm = FAKE_STROKE_MM;
        calibration.measuredStrokeSteps =
            min(float(fakeStrokeMm * Config::Driver::stepsPerMM),
                Config::Driver::maxStrokeSteps);

        stepper->enableOutputs();
        stepper->setCurrentPosition(0);
        stepper->forceStopAndNewPosition(0);

        ESP_LOGW("Homing",
                 "FAKE_HOMING active: pretending stroke = %.1f mm (%ld steps)",
                 fakeStrokeMm, (long)calibration.measuredStrokeSteps);

        setHomingActive(false);
        stateMachine->process_event(Done{});
        vTaskDelete(nullptr);
        return;
    }
#endif

    // Stroke Engine and Simple Penetration treat this differently.
    stepper->enableOutputs();
    stepper->setDirectionPin(Pins::Driver::motorDirectionPin, true);
    // TEMP DEBUG: signs swapped so the first homing pass moves BACKWARD,
    // to verify the backward direction actually moves at all.
    // Revert to (backward ? 1 : -1) once confirmed.
    int16_t sign = stateMachine->is("homing.backward"_s) ? 1 : -1;

    int32_t targetPositionInSteps =
        round(sign * Config::Driver::maxStrokeSteps);

    ESP_LOGD("Homing", "Target position in steps: %d", targetPositionInSteps);
    stepper->moveTo(targetPositionInSteps, false);

    auto isInCorrectState = []() {
        // Add any states that you want to support here.
        return stateMachine->is("homing"_s) ||
               stateMachine->is("homing.forward"_s) ||
               stateMachine->is("homing.backward"_s);
    };

    // run loop for 15second or until loop exits
    while (isInCorrectState()) {
        TickType_t xCurrentTickCount = xTaskGetTickCount();
        // Calculate the time in ticks that the task has been running.
        TickType_t xTicksPassed = xCurrentTickCount - xTaskStartTime;

        // If you need the time in milliseconds, convert ticks to milliseconds.
        // 'portTICK_PERIOD_MS' is the number of milliseconds per tick.
        uint32_t msPassed = xTicksPassed * portTICK_PERIOD_MS;

        if (msPassed > 40000) {
            ESP_LOGE("Homing", "Homing took too long. Check power and restart");
            errorState.message = UserConfig::language.HomingTookTooLong;

            // Clear homing active flag for LED indication
            setHomingActive(false);

            stateMachine->process_event(Error{});
            break;
        }

        // Measure motor current via INA219 (mA), minus the idle offset.
        // Few samples on purpose: we must react in well under a second to
        // back off before a closed-loop driver latches its position-error
        // alarm and disables itself.
        float current =
            getCurrentMilliAmps(3) - calibration.currentSensorOffset;

        // Periodic debug print so the threshold can be tuned from logs.
        static uint32_t lastLog = 0;
        uint32_t now = millis();
        if (now - lastLog > 200) {
            ESP_LOGI("Homing", "Current: %.1f mA (limit %.1f)", current,
                     Config::Driver::sensorlessCurrentLimit);
            lastLog = now;
        }
        bool isCurrentOverLimit =
            current > Config::Driver::sensorlessCurrentLimit;

        if (!isCurrentOverLimit) {
            vTaskDelay(10);  // Increased from 1ms to 10ms to reduce CPU load
            continue;
        }

        ESP_LOGD("Homing", "Current over limit: %f", current);
        stepper->stopMove();

        stepper->setSpeedInHz(250_mm);
        // step away from the hard stop, with your hands in the air!
        int32_t currentPosition = stepper->getCurrentPosition();
        stepper->moveTo(currentPosition - sign * Config::Driver::homingOffsetMn,
                        true);

        // measure and save the current position
        calibration.measuredStrokeSteps =
            min(float(abs(stepper->getCurrentPosition())),
                Config::Driver::maxStrokeSteps);

        stepper->setCurrentPosition(0);
        stepper->forceStopAndNewPosition(0);

        int32_t goToPosition = -sign * calibration.measuredStrokeSteps;
        if (goToPosition < 0){
            goToPosition = goToPosition * UserConfig::afterHomingPosition;
        }

        stepper->moveTo(goToPosition,true);

        // Clear homing active flag for LED indication
        setHomingActive(false);

        stateMachine->process_event(Done{});
        break;
    };

    vTaskDelete(nullptr);
}

void startHoming() {
    int stackSize = 10 * configMINIMAL_STACK_SIZE;
    xTaskCreatePinnedToCore(startHomingTask, "startHomingTask", stackSize,
                            nullptr, configMAX_PRIORITIES - 1,
                            &Tasks::runHomingTaskH, Tasks::operationTaskCore);
}

bool isStrokeTooShort() {
    if (calibration.measuredStrokeSteps > Config::Driver::minStrokeLengthMm) {
        return false;
    }
    errorState.message = UserConfig::language.StrokeTooShort;
    return true;
}

}  // namespace homing
