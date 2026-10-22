#include "control_task.h"

#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "lap_timer.h"
#include "motors.h"
#include "sensors.h"
#include "shared.h"

static_assert(CONTROL_PERIOD_US % 1000 == 0, "control loop is paced by the 1 ms FreeRTOS tick");

namespace {

constexpr TickType_t CONTROL_PERIOD_TICKS = pdMS_TO_TICKS(CONTROL_PERIOD_US / 1000);
constexpr uint32_t   STATS_WINDOW = 20;  // ticks per loop-timing window

uint32_t lostSinceMs = 0;
bool     wasLost = false;
bool     following = false;  // controlUpdate ran on the previous tick
float    appliedTrim = MOTOR_TRIM;

volatile uint32_t busyUsTotal = 0;

// Copy live tuning from the phone into the controller and motor driver.
void applyTuning(const Tuning& t) {
    ControlGains g = controlGetGains();
    if (g.kp != t.kp || g.kd != t.kd || g.base != t.autoSpeed) {
        controlSetGains({t.kp, t.kd, t.autoSpeed});
    }
    if (appliedTrim != t.trim) {
        motorsSetTrim(t.trim);
        appliedTrim = t.trim;
    }
}

void manualStep(const DriveCommand& cmd, const Tuning& t) {
    // Dead-man: no fresh command from the phone -> brake.
    if (millis() - cmd.driveMs > MANUAL_TIMEOUT_MS) {
        motorsStop();
        return;
    }
    float l, r;
    controlManual(cmd.throttle, cmd.steer, t.manualMax, l, r);
    motorsSet(l, r);
}

bool     calibrating = false;
uint32_t calStartMs = 0;

// Calibration spin: right, left, right on the spot while the sensors record
// their min/max (readPosition() does the recording). Then Stop.
void calibrateStep() {
    if (!calibrating) {
        calibrating = true;
        calStartMs = millis();
        sensorsCalibrateBegin();
    }

    uint32_t t = millis() - calStartMs;
    if (t < CAL_DURATION_MS) {
        bool right = t < CAL_DURATION_MS / 4 || t >= CAL_DURATION_MS * 3 / 4;
        float s = right ? CAL_SPIN_SPEED : -CAL_SPIN_SPEED;
        motorsSet(s, -s);
        return;
    }

    motorsStop();
    calibrating = false;
    int8_t bad;
    bool ok = sensorsCalibrateEnd(bad);
    sharedCalibrationDone(ok, bad);
    sharedSetMode(DriveMode::Stop);
}

// One control step: read the line, then drive or brake. Returns the position.
float step(bool running, const DriveCommand& cmd, const Tuning& t, float dt) {
    float pos = readPosition();
    bool wasFollowing = following;
    following = false;

    // Left calibration mode before it finished: keep the old calibration.
    if (calibrating && (cmd.mode != DriveMode::Calibrate || !running)) {
        calibrating = false;
        sensorsCalibrateAbort();
    }

    if (!running || cmd.mode == DriveMode::Stop) {
        motorsStop();
        return pos;
    }
    if (cmd.mode == DriveMode::Calibrate) {
        calibrateStep();
        return pos;
    }
    if (cmd.mode == DriveMode::Manual) {
        manualStep(cmd, t);
        wasLost = false;
        return pos;
    }

    LineState state = lineState();
    if (state != LineState::OnLine) {
        if (!wasLost) {
            wasLost = true;
            lostSinceMs = millis();
        }
        uint32_t timeout = state == LineState::Lost ? LOST_TIMEOUT_MS : GAP_TIMEOUT_MS;
        if (millis() - lostSinceMs > timeout) {
            motorsStop();
            return pos;
        }
    } else {
        wasLost = false;
    }

    // Line following (re)starts: no derivative from a stale position.
    if (!wasFollowing) controlReset();
    following = true;

    float l, r;
    controlUpdate(pos, dt, l, r);
    motorsSet(l, r);
    return pos;
}

void controlTask(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t   lastStartUs = micros();
    uint32_t   seq = 0;

    uint32_t winLoopMax = 0, winPeriodMax = 0, winCount = 0;
    uint16_t loopMaxUs = 0, periodMaxUs = 0;

    for (;;) {
        vTaskDelayUntil(&lastWake, CONTROL_PERIOD_TICKS);

        uint32_t start = micros();
        uint32_t period = start - lastStartUs;
        lastStartUs = start;

        // Sensors are read and published during the start delay too, so the
        // phone shows them before the robot moves.
        bool running = millis() >= START_DELAY_MS;
        DriveCommand cmd = sharedCommand();
        Tuning tune = sharedTuning();
        applyTuning(tune);
        float pos = step(running, cmd, tune, period * 1e-6f);

        LapEvent lap;
        if (LAP_TIMING_ENABLED && running && cmd.mode == DriveMode::Auto &&
            lapTimerUpdate(sensorsBits(), lap)) {
            sharedPushLap(lap);
        }

        uint32_t busy = micros() - start;
        busyUsTotal += busy;
        winLoopMax = max(winLoopMax, busy);
        winPeriodMax = max(winPeriodMax, period);
        if (++winCount >= STATS_WINDOW) {
            loopMaxUs = (uint16_t)min<uint32_t>(winLoopMax, 0xFFFF);
            periodMaxUs = (uint16_t)min<uint32_t>(winPeriodMax, 0xFFFF);
            winLoopMax = winPeriodMax = winCount = 0;
        }

        TelemetrySample s;
        s.seq = ++seq;
        s.timeMs = millis();
        s.sensorBits = sensorsBits();
        s.position = pos;
        s.dutyLeft = motorsLeftDuty();
        s.dutyRight = motorsRightDuty();
        s.lineState = (uint8_t)lineState();
        s.running = running;
        s.mode = cmd.mode;
        s.lap = lapCount();
        s.lastLapMs = lastLapMs();
        s.loopMaxUs = loopMaxUs;
        s.periodMaxUs = periodMaxUs;
        sensorsLevels(s.levels);
        sharedPublish(s);
    }
}

}  // namespace

void controlTaskStart() {
    lapTimerBegin();
    // Above the lwIP task (18), so network work can't pre-empt control.
    xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                            configMAX_PRIORITIES - 3, nullptr, 1);
}

uint32_t controlBusyUsTotal() { return busyUsTotal; }
