#ifndef SOFTWARE_UPDATE_H
#define SOFTWARE_UPDATE_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "ArduinoJson.h"
#include "constants/LogTags.h"
#include "services/display.h"

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

// Draw the safe-mode OTA splash. `status` is the line under the title;
// `percent` is 0..100 for a progress bar, or <0 to hide the bar (used while
// connecting WiFi / waiting on the server before download starts).
//
// No mutex needed — in safe mode this is the only task touching the display.
inline void otaDrawStatus(const char *status, int percent) {
    display.clearBuffer();

    // Title (centered, bold-ish)
    display.setFont(u8g2_font_helvB10_tf);
    const char *title = "UPDATING";
    int titleW = display.getStrWidth(title);
    display.drawStr((128 - titleW) / 2, 14, title);

    // Subtitle
    display.setFont(u8g2_font_6x10_tf);
    const char *warn = "Do not power off";
    int warnW = display.getStrWidth(warn);
    display.drawStr((128 - warnW) / 2, 28, warn);

    // Status line
    int statusW = display.getStrWidth(status);
    display.drawStr((128 - statusW) / 2, 44, status);

    // Progress bar (only when we have a real percent to show)
    if (percent >= 0) {
        int p = percent > 100 ? 100 : percent;
        display.drawFrame(14, 52, 100, 8);
        display.drawBox(14, 52, p, 8);
    }

    display.sendBuffer();
}

// Task that performs the actual OTA. Spawned from safe-mode boot only — heap
// is wide open here, so no NimBLE quiesce contortions needed.
inline void otaUpdateTask(void *) {
    ESP_LOGI("UTILS", "OTA safe-mode: free heap=%u, largest block=%u",
             ESP.getFreeHeap(), ESP.getMaxAllocHeap());

    otaDrawStatus("Starting...", 0);

    WiFiClientSecure client;
    client.setInsecure();

    httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

    // Progress callback — fired from inside Update.write() as bytes stream in.
    // Throttle redraws to ~1%-step changes; the OLED is slow over I2C and
    // redrawing on every chunk would bottleneck the download. We also log
    // running throughput once per 10% so you can spot WiFi vs. flash bottlenecks
    // in the serial monitor.
    static int lastPctDrawn = -1;
    static uint32_t otaStartMs = millis();
    static int lastLoggedDecile = -1;
    Update.onProgress([](size_t cur, size_t total) {
        if (total == 0) return;
        int pct = static_cast<int>((cur * 100) / total);
        if (pct != lastPctDrawn) {
            lastPctDrawn = pct;
            otaDrawStatus("Downloading...", pct);
        }
        int decile = pct / 10;
        if (decile != lastLoggedDecile) {
            lastLoggedDecile = decile;
            uint32_t elapsed = millis() - otaStartMs;
            if (elapsed > 0) {
                uint32_t kbps = (cur * 1000) / 1024 / elapsed;
                ESP_LOGI("UTILS", "OTA progress: %d%% (%u / %u bytes) @ %u KB/s",
                         pct, (unsigned)cur, (unsigned)total, kbps);
            }
        }
    });

    otaStartMs = millis();
    t_httpUpdate_return ret = httpUpdate.update(client, OTA_FIRMWARE_URL);
    uint32_t totalMs = millis() - otaStartMs;
    ESP_LOGI("UTILS", "OTA finished in %u ms", totalMs);

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            ESP_LOGE("UTILS", "HTTP_UPDATE_FAILED Error (%d): %s",
                     httpUpdate.getLastError(),
                     httpUpdate.getLastErrorString().c_str());
            otaDrawStatus("Update failed", -1);
            break;
        case HTTP_UPDATE_NO_UPDATES:
            ESP_LOGI("UTILS", "HTTP_UPDATE_NO_UPDATES");
            otaDrawStatus("No update found", -1);
            break;
        case HTTP_UPDATE_OK:
            // httpUpdate reboots automatically on success; not reached.
            ESP_LOGI("UTILS", "HTTP_UPDATE_OK");
            otaDrawStatus("Rebooting...", 100);
            break;
    }

    client.stop();

    // Anything other than HTTP_UPDATE_OK ends up here. Hold the failure
    // message on screen briefly so the user can read it, then reboot back
    // into normal firmware (flag was already cleared before we started).
    ESP_LOGW("UTILS", "OTA did not complete — rebooting to normal firmware");
    delay(3000);
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

    // Bring up the display first so the user has feedback while WiFi tries
    // to connect. initDisplay() also creates the U8g2 framebuffer (~1 KB) —
    // trivial cost on a heap that's otherwise empty.
    initDisplay();
    otaDrawStatus("Connecting WiFi...", -1);

    // Bring up WiFi STA using credentials saved by WiFiManager. WiFi.begin()
    // with no args reuses the last-stored SSID/password from the WiFi NVS
    // namespace (esp_wifi maintains this independently of our ossm namespace).
    WiFi.mode(WIFI_STA);
    // Disable WiFi modem-sleep for the OTA window. Default STA power-save
    // injects ~100 ms latency between TCP packets, which throttles streaming
    // downloads by 2-3×. We're about to reboot regardless, so battery / heat
    // concerns don't apply.
    WiFi.setSleep(false);
    WiFi.begin();

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 30000) {
        delay(250);
    }

    if (WiFi.status() != WL_CONNECTED) {
        ESP_LOGE("UTILS", "WiFi did not connect — aborting OTA, rebooting");
        otaDrawStatus("WiFi failed", -1);
        delay(3000);
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
