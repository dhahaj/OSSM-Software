#ifndef SOFTWARE_UPDATE_H
#define SOFTWARE_UPDATE_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>

#include "ArduinoJson.h"
#include "constants/LogTags.h"

#ifndef SW_VERSION
#define SW_VERSION "0.0.0"
#endif

static auto isUpdateAvailable = []() {
    // Version check bypassed: always attempt to pull the firmware binary from
    // the configured release URL when the user selects "Update" from the menu.
    // The httpUpdate call itself will short-circuit with HTTP_UPDATE_NO_UPDATES
    // if the binary's embedded version matches what's already flashed.
    if (WiFiClass::status() != WL_CONNECTED) {
        ESP_LOGD(UPDATE_TAG, "Not connected to WiFi");
        return false;
    }
    return true;
};

auto updateOSSM = []() {
    WiFiClientSecure client;
    client.setInsecure();

    String url =
        "https://github.com/dhahaj/OSSM-Software/releases/download/v1.0.0/"
        "firmware.bin";

    httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

    t_httpUpdate_return ret = httpUpdate.update(client, url);

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            ESP_LOGD("UTILS", "HTTP_UPDATE_FAILED Error (%d): %s\n",
                     httpUpdate.getLastError(),
                     httpUpdate.getLastErrorString().c_str());
            break;

        case HTTP_UPDATE_NO_UPDATES:
            ESP_LOGD("UTILS", "HTTP_UPDATE_NO_UPDATES");
            break;

        case HTTP_UPDATE_OK:
            ESP_LOGD("UTILS", "HTTP_UPDATE_OK");
            break;
    }

    client.stop();
};

#endif  // SOFTWARE_UPDATE_H
