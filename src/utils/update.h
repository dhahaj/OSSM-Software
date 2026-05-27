#ifndef SOFTWARE_UPDATE_H
#define SOFTWARE_UPDATE_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "ArduinoJson.h"
#include "constants/LogTags.h"

#ifndef SW_VERSION
#define SW_VERSION "0.0.0"
#endif

// --- Reboot-to-OTA -----------------------------------------------------------
//
// Update.begin() needs a contiguous 4 KB block of internal DRAM that the live
// firmware can't reliably provide — BLE, WiFi, mbedTLS, display, motor task
// and FreeRTOS scaffolding fragment the heap. Rather than fight that, we
// reboot into a "safe-mode" code path that runs before any of those
// subsystems initialize. Heap is essentially untouched (~250 KB free) and the
// OTA succeeds first try.
//
// Flow:
//   1. User picks "Update" in the menu.
//   2. SML action `updateOSSM` writes ota_pending=true to NVS, then ESP.restart().
//   3. On boot, setup() calls runPendingOTAIfRequested() *before* any init.
//   4. If the flag is set: clear it, bring up WiFi STA with saved creds,
//      spawn a 16 KB task that runs httpUpdate against the firmware URL.
//   5. On success httpUpdate reboots into the new image.
//   6. On failure we reboot anyway → normal boot resumes.
//
// Power loss mid-OTA is safe: the OTA partition isn't activated until
// Update.end() succeeds, so an interrupted download leaves the old firmware
// intact.
// -----------------------------------------------------------------------------

static constexpr const char *OTA_PREFS_NAMESPACE = "ossm";
static constexpr const char *OTA_PENDING_KEY = "ota_pending";

// URL of the firmware binary. Served via Cloudflare Worker that proxies the
// GitHub release (follows GitHub's 302 server-side so the ESP32 sees a clean
// 200 — avoids the redirect bug in arduino-esp32's bundled HTTPUpdate).
static constexpr const char *OTA_FIRMWARE_URL =
    "https://ossm-firmware.dhahaj.trade/firmware.bin";

// Task that performs the actual OTA. Spawned from safe-mode boot only — heap
// is wide open here, so no NimBLE quiesce / setBufferSizes contortions
// needed.
inline void otaUpdateTask(void *) {
    ESP_LOGI("UTILS", "OTA safe-mode: free heap=%u, largest block=%u",
             ESP.getFreeHeap(), ESP.getMaxAllocHeap());

    WiFiClientSecure client;
    client.setInsecure();

    httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

    t_httpUpdate_return ret = httpUpdate.update(client, OTA_FIRMWARE_URL);

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            ESP_LOGE("UTILS", "HTTP_UPDATE_FAILED Error (%d): %s",
                     httpUpdate.getLastError(),
                     httpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            ESP_LOGI("UTILS", "HTTP_UPDATE_NO_UPDATES");
            break;
        case HTTP_UPDATE_OK:
            // httpUpdate reboots automatically on success; not reached.
            ESP_LOGI("UTILS", "HTTP_UPDATE_OK");
            break;
    }

    client.stop();

    // Anything other than HTTP_UPDATE_OK ends up here. Reboot back into
    // normal firmware (flag was already cleared before we started).
    ESP_LOGW("UTILS", "OTA did not complete — rebooting to normal firmware");
    delay(1000);
    ESP.restart();
}

// Call once at the very top of setup() before any other init. If an OTA was
// requested on the previous boot, this runs it and never returns. Otherwise
// returns immediately and normal boot continues.
inline void runPendingOTAIfRequested() {
    Preferences prefs;
    prefs.begin(OTA_PREFS_NAMESPACE, false);
    bool pending = prefs.getBool(OTA_PENDING_KEY, false);
    if (!pending) {
        prefs.end();
        return;
    }
    // Clear the flag immediately — if the OTA fails or hangs, the next reboot
    // boots normally instead of getting stuck in a safe-mode loop.
    prefs.putBool(OTA_PENDING_KEY, false);
    prefs.end();

    ESP_LOGI("UTILS", "OTA pending flag set — entering safe-mode update");

    // Bring up WiFi STA using credentials saved by WiFiManager. WiFi.begin()
    // with no args reuses the last-stored SSID/password from the WiFi NVS
    // namespace (esp_wifi maintains this independently of our ossm namespace).
    WiFi.mode(WIFI_STA);
    WiFi.begin();

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 30000) {
        delay(250);
    }

    if (WiFi.status() != WL_CONNECTED) {
        ESP_LOGE("UTILS", "WiFi did not connect — aborting OTA, rebooting");
        delay(1000);
        ESP.restart();
    }

    ESP_LOGI("UTILS", "WiFi connected (%s) — starting OTA",
             WiFi.localIP().toString().c_str());

    // 16 KB stack covers mbedTLS handshake + HTTPClient buffers.
    xTaskCreate(otaUpdateTask, "otaUpdate", 16384, nullptr, 5, nullptr);

    // Park the main task forever — otaUpdateTask reboots us regardless of
    // outcome.
    while (true) {
        delay(1000);
    }
}

// Guard used by the state machine's `update` state — only attempts the
// update if WiFi is up. Always returns true past that check; the actual
// "is there a new version?" question is now answered by the build/release
// process, not by the device.
static auto isUpdateAvailable = []() {
    if (WiFiClass::status() != WL_CONNECTED) {
        ESP_LOGD(UPDATE_TAG, "Not connected to WiFi");
        return false;
    }
    return true;
};

// SML action: triggered when the user picks "Update" from the menu. Persists
// the request to NVS and reboots — the actual download happens in
// runPendingOTAIfRequested() on the next boot, where the heap is clean.
auto updateOSSM = []() {
    ESP_LOGI("UTILS", "Update requested — setting flag and rebooting");

    Preferences prefs;
    prefs.begin(OTA_PREFS_NAMESPACE, false);
    prefs.putBool(OTA_PENDING_KEY, true);
    prefs.end();

    // Brief pause so the "drawUpdating" UI frame and serial log have time to
    // flush before we yank the carpet.
    delay(500);
    ESP.restart();
};

#endif  // SOFTWARE_UPDATE_H
