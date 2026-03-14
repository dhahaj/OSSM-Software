#include "homing.h"

#include "Strings.h"
#include "constants/Config.h"
#include "constants/Pins.h"
#include "constants/UserConfig.h"
#include "ossm/Events.h"
#include "ossm/state/calibration.h"
#include "ossm/state/error.h"
#include "ossm/state/state.h"
#include "services/led.h"
#include "services/stepper.h"
#include "services/tasks.h"

namespace sml = boost::sml;
using namespace sml;

namespace homing {

void clearHoming() {
    ESP_LOGI("Homing", "=== Homing started ===");

    // Set homing active flag for LED indication
    setHomingActive(true);

    calibration.isForward = true;

    // Set acceleration and deceleration in steps/s^2
    stepper->setAcceleration(1000_mm);
    // Set speed in steps/s
    stepper->setSpeedInHz(25_mm);

    // Clear the stored values.
    calibration.measuredStrokeSteps = 0;
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

    // Stroke Engine and Simple Penetration treat this differently.
    stepper->enableOutputs();
    stepper->setDirectionPin(Pins::Driver::motorDirectionPin, false);
    int16_t sign = stateMachine->is("homing.backward"_s) ? 1 : -1;
    const char *phase = (sign == -1) ? "FORWARD" : "BACKWARD";
    ESP_LOGI("Homing", "Phase: %s", phase);

    int32_t targetPositionInSteps =
        round(sign * Config::Driver::maxStrokeSteps);

    ESP_LOGI("Homing", "Target position: %d steps", targetPositionInSteps);
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
            errorState.message = ui::strings::homingTookTooLong;

            // Clear homing active flag for LED indication
            setHomingActive(false);

            stateMachine->process_event(Error{});
            break;
        }

        // Read CL57Y ALM output — active LOW (pulled LOW on stall/alarm).
        bool isAlmTriggered = digitalRead(Pins::Driver::almPin) == LOW;

        if (!isAlmTriggered) {
            vTaskDelay(5);
            continue;
        }

        ESP_LOGI("Homing", "ALM triggered — end-of-stroke detected at position %d steps", stepper->getCurrentPosition());
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

        float strokeMm = calibration.measuredStrokeSteps / Config::Driver::stepsPerMM;
        ESP_LOGI("Homing", "Measured stroke: %.1f mm (%.0f steps)", strokeMm, calibration.measuredStrokeSteps);

        stepper->setCurrentPosition(0);
        stepper->forceStopAndNewPosition(0);

        int32_t goToPosition = -sign * calibration.measuredStrokeSteps;
        if (goToPosition < 0){
            goToPosition = goToPosition * UserConfig::afterHomingPosition;
        }

        stepper->moveTo(goToPosition,true);

        // Clear homing active flag for LED indication
        setHomingActive(false);

        ESP_LOGI("Homing", "=== Homing %s phase complete ===", phase);
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
    float strokeMm = calibration.measuredStrokeSteps / Config::Driver::stepsPerMM;
    float minMm = Config::Driver::minStrokeLengthMm / Config::Driver::stepsPerMM;
    if (calibration.measuredStrokeSteps > Config::Driver::minStrokeLengthMm) {
        ESP_LOGI("Homing", "Stroke validation passed: %.1f mm (min: %.1f mm)", strokeMm, minMm);
        return false;
    }
    ESP_LOGE("Homing", "Stroke too short: %.1f mm (min: %.1f mm)", strokeMm, minMm);
    errorState.message = ui::strings::strokeTooShort;
    return true;
}

}  // namespace homing
