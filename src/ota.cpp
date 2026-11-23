#include "ota.h"

#include <Arduino.h>
#include <ArduinoOTA.h>

#include "config.h"
#include "shared.h"

#ifndef OTA_PASSWORD
#error "OTA_PASSWORD is set in platformio.ini [ota]"
#endif

void otaBegin() {
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([] {
        // The control loop brakes on its next tick (1 ms); give it a few.
        sharedSetMode(DriveMode::Stop);
        delay(10);
        Serial.println("OTA: receiving new firmware, robot stopped");
    });
    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
        static unsigned int lastTenth = 11;
        unsigned int tenth = total ? done * 10 / total : 0;
        if (tenth != lastTenth) {
            lastTenth = tenth;
            Serial.printf("OTA: %u %%\n", tenth * 10);
        }
    });
    ArduinoOTA.onEnd([] { Serial.println("OTA: done, restarting"); });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA: failed (error %u), still running the old firmware\n", e); });

    ArduinoOTA.begin();
    Serial.printf("OTA: ready on port %u, hostname %s\n", OTA_PORT, OTA_HOSTNAME);
}

void otaHandle() { ArduinoOTA.handle(); }
