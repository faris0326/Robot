// Line follower main. Which program runs is chosen by the PlatformIO env:
//   robot (default)  control loop on core 1, Wi-Fi telemetry on core 0;
//                    boots in Stop: choose a mode on the phone
//   sensortest       print IR readings, motors stay braked
//   steertest        control runs with motors live (wheels lifted), prints L/R
//   motortest        Phase 1 motor test

#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "control_task.h"
#include "motors.h"
#include "power.h"
#include "sensors.h"
#include "settings.h"
#include "shared.h"
#include "telemetry.h"
#include "tests.h"

void setup() {
    Serial.begin(115200);
    motorsBegin();  // brakes immediately so outputs are defined at boot
    sensorsBegin();
    controlBegin();

    SensorCal cal;
    bool calSaved = settingsLoadCal(cal);
    if (calSaved) sensorsSetCalibration(cal);
    Serial.printf("\nSensor calibration: %s\n", calSaved ? "from flash" : "defaults (not calibrated yet)");
    bool white;
    if (settingsLoadLineWhite(white)) sensorsSetLineWhite(white);

#if defined(MOTOR_TEST)
    motorTest();
#elif defined(SENSOR_TEST)
    Serial.println("\nSensor test. Motors braked.");
#elif defined(STEER_TEST)
    Serial.printf("\nSteer test. LIFT THE WHEELS. Starting in %lu ms\n",
                  (unsigned long)START_DELAY_MS);
    delay(START_DELAY_MS);
#else
    sharedBegin();
    powerBegin();  // INA219 task on core 0

    Tuning t;
    bool saved = settingsLoad(t);
    if (saved) sharedSetTuning(t);
    t = sharedTuning();
    Serial.printf("\nLine follower. Tuning from %s: speed=%.2f KP=%.2f KD=%.3f trim=%.2f. "
                  "%s line. Stopped: choose Auto on the phone\n",
                  saved ? "flash" : "config.h", t.autoSpeed, t.kp, t.kd, t.trim,
                  sensorsLineWhite() ? "White" : "Black");
    Serial.printf("Board: %s. Motor supply %s; IR LED on GPIO %u\n",
                  BOARD == Board::PcbChassis ? "PCB chassis" : "original",
                  INA219_ON_BATTERY ? "measured by the INA219 (full 8.4 V assumed until read)"
                                    : "assumed 8.4 V (not measured)",
                  PIN_IR_LED);
    controlTaskStart();  // core 1
    telemetryStart();    // core 0
#endif
}

void loop() {
#if defined(MOTOR_TEST)
    delay(1000);
#elif defined(SENSOR_TEST)
    sensorTestLoop();
#elif defined(STEER_TEST)
    steerTestLoop();
#else
    vTaskDelete(nullptr);  // all work happens in the pinned tasks
#endif
}
