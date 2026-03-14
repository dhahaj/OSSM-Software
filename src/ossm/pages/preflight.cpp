#include "preflight.h"

#include "Strings.h"
#include "constants/Config.h"
#include "constants/Menu.h"
#include "constants/Pins.h"
#include "ossm/Events.h"
#include "ossm/state/menu.h"
#include "ossm/state/state.h"
#include "services/display.h"
#include "services/stepper.h"
#include "services/tasks.h"
#include "ui.h"
#include "utils/analog.h"
#include "utils/format.h"

namespace sml = boost::sml;
using namespace sml;

namespace pages {

static void drawPreflightTask(void *pvParameters) {
    auto menuString = menuStrings[menuState.currentOption];
    float speedPercentage;
    bool loggedWaiting = false;

    auto isInPreflight = []() {
        return stateMachine->is("simplePenetration.preflight"_s) ||
               stateMachine->is("streaming.preflight"_s) ||
               stateMachine->is("strokeEngine.preflight"_s);
    };

    ESP_LOGI("Preflight", "Mode: %s — checking speed knob position", menuString);

    do {
#ifdef AJ_DEVELOPMENT_HARDWARE
        speedPercentage = 0;
#else
        speedPercentage =
            getAnalogAveragePercent(SampleOnPin{Pins::Remote::speedPotPin, 50});
#endif
        if (speedPercentage < Config::Advanced::commandDeadZonePercentage) {
            ESP_LOGI("Preflight", "Speed knob at zero — preflight passed");
            stateMachine->process_event(Done{});
            break;
        };

        if (!loggedWaiting) {
            ESP_LOGW("Preflight", "Speed knob at %.1f%% — turn to zero to continue", speedPercentage);
            loggedWaiting = true;
        }

        if (xSemaphoreTake(displayMutex, 100) == pdTRUE) {
            ui::PreflightData data{menuString, speedPercentage,
                                   ui::strings::speed,
                                   ui::strings::speedWarning};
            ui::drawPreflight(display.getU8g2(), data);
            refreshPage(true, true);
            xSemaphoreGive(displayMutex);
        }

        vTaskDelay(100);
    } while (isInPreflight());

    vTaskDelete(nullptr);
};

void drawPreflight() {
    int stackSize = 3 * configMINIMAL_STACK_SIZE;
    xTaskCreate(drawPreflightTask, "drawPreflightTask", stackSize, nullptr, 1,
                &Tasks::drawPreflightTaskH);
}

}  // namespace pages
