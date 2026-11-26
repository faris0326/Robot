#include "shared.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "settings.h"

namespace {
portMUX_TYPE    mux = portMUX_INITIALIZER_UNLOCKED;
TelemetrySample latest = {};
QueueHandle_t   lapQueue = nullptr;

portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
DriveCommand command = {BOOT_MODE, 0.0f, 0.0f, 0};
Tuning       tuning = settingsDefaults();  // main() may replace it with the saved values
}  // namespace

void sharedBegin() {
    lapQueue = xQueueCreate(16, sizeof(LapEvent));
}

void sharedPublish(const TelemetrySample& s) {
    portENTER_CRITICAL(&mux);
    latest = s;
    portEXIT_CRITICAL(&mux);
}

TelemetrySample sharedLatest() {
    portENTER_CRITICAL(&mux);
    TelemetrySample s = latest;
    portEXIT_CRITICAL(&mux);
    return s;
}

bool sharedPushLap(const LapEvent& ev) {
    return xQueueSend(lapQueue, &ev, 0) == pdTRUE;
}

bool sharedPopLap(LapEvent& ev) {
    return xQueueReceive(lapQueue, &ev, 0) == pdTRUE;
}

void sharedSetMode(DriveMode mode) {
    portENTER_CRITICAL(&cmdMux);
    command.mode = mode;
    command.throttle = 0.0f;  // never carry a stick position across a mode change
    command.steer = 0.0f;
    portEXIT_CRITICAL(&cmdMux);
}

void sharedSetDrive(float throttle, float steer) {
    uint32_t now = millis();
    portENTER_CRITICAL(&cmdMux);
    command.throttle = throttle;
    command.steer = steer;
    command.driveMs = now;
    portEXIT_CRITICAL(&cmdMux);
}

DriveCommand sharedCommand() {
    portENTER_CRITICAL(&cmdMux);
    DriveCommand c = command;
    portEXIT_CRITICAL(&cmdMux);
    return c;
}

void sharedSetTuning(const Tuning& in) {
    // isnan check first: constrain() passes NaN straight through.
    auto safe = [](float v, float lo, float hi) { return isnan(v) ? lo : constrain(v, lo, hi); };
    Tuning t;
    t.autoSpeed = safe(in.autoSpeed, 0.0f, 1.0f);
    t.manualMax = safe(in.manualMax, 0.0f, 1.0f);
    t.kp = safe(in.kp, 0.0f, 5.0f);
    t.kd = safe(in.kd, 0.0f, 1.0f);
    t.trim = isnan(in.trim) ? 0.0f : constrain(in.trim, -0.9f, 0.9f);
    t.cornerBrakeMs = safe(in.cornerBrakeMs, 0.0f, 300.0f);
    t.pivotSpeed = safe(in.pivotSpeed, 0.1f, 1.0f);
    t.calSpeed = safe(in.calSpeed, 0.1f, 1.0f);
    t.smoothMs = safe(in.smoothMs, 0.0f, 30.0f);
    t.gapMs = safe(in.gapMs, 0.0f, 1000.0f);
    t.cruiseMms = safe(in.cruiseMms, 0.0f, 4000.0f);
    t.cruiseKp = safe(in.cruiseKp, 0.0f, 10.0f);
    t.cruiseKi = safe(in.cruiseKi, 0.0f, 50.0f);
    t.raceTopMms = safe(in.raceTopMms, MAP_MIN_MMS, 4000.0f);
    t.raceGrip = safe(in.raceGrip, 0.5f, 10.0f);
    portENTER_CRITICAL(&cmdMux);
    tuning = t;
    portEXIT_CRITICAL(&cmdMux);
}

Tuning sharedTuning() {
    portENTER_CRITICAL(&cmdMux);
    Tuning t = tuning;
    portEXIT_CRITICAL(&cmdMux);
    return t;
}

namespace {
uint32_t calSeq = 0;
bool     calOk = false;
int8_t   calBad = -1;
}  // namespace

void sharedCalibrationDone(bool ok, int8_t badSensor) {
    portENTER_CRITICAL(&cmdMux);
    calOk = ok;
    calBad = badSensor;
    calSeq++;
    portEXIT_CRITICAL(&cmdMux);
}

uint32_t sharedCalibrationResult(bool& ok, int8_t& badSensor) {
    portENTER_CRITICAL(&cmdMux);
    uint32_t seq = calSeq;
    ok = calOk;
    badSensor = calBad;
    portEXIT_CRITICAL(&cmdMux);
    return seq;
}

namespace {
portMUX_TYPE lossMux = portMUX_INITIALIZER_UNLOCKED;
LossRecord   loss = {};
uint32_t     lossSeq = 0;
}  // namespace

void sharedSetLoss(const LossRecord& r) {
    portENTER_CRITICAL(&lossMux);
    loss = r;
    lossSeq++;
    portEXIT_CRITICAL(&lossMux);
}

uint32_t sharedLoss(LossRecord& r) {
    portENTER_CRITICAL(&lossMux);
    r = loss;
    uint32_t seq = lossSeq;
    portEXIT_CRITICAL(&lossMux);
    return seq;
}
