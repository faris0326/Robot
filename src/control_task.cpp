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
    if (g.kp != t.kp || g.kd != t.kd || g.base != t.autoSpeed || g.smoothMs != t.smoothMs) {
        controlSetGains({t.kp, t.kd, t.autoSpeed, t.smoothMs});
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

// Corner brake: full reverse on both wheels, once per corner (see config.h).
bool     brakeArmed = false;
uint32_t brakeUntilMs = 0;
uint32_t plainSinceMs = 0;  // following a plain line (no corner) since

// Downhill slow-down: Auto speed x downhillScale for downhillMs once the
// line is back after a crest/gap (see config.h).
uint32_t downhillUntilMs = 0;
bool     inGapLastTick = false;
bool downhill() { return (int32_t)(downhillUntilMs - millis()) > 0; }

// True while braking. Fires when a corner shows or the line is lost, if the
// robot has followed a plain line for CORNER_REARM_MS since the last brake.
bool cornerBrake(LineState state, const Tuning& t) {
    uint32_t now = millis();
    int8_t corner = sensorsCorner();
    if (state != LineState::OnLine || corner != 0) {
        plainSinceMs = now;
    } else if (now - plainSinceMs >= CORNER_REARM_MS) {
        brakeArmed = true;
    }

    if (brakeArmed && t.cornerBrakeMs >= 1.0f && (corner != 0 || state == LineState::Lost)) {
        brakeArmed = false;
        brakeUntilMs = now + (uint32_t)t.cornerBrakeMs;
    }
    return (int32_t)(brakeUntilMs - now) > 0;
}

bool     calibrating = false;
uint32_t calStartMs = 0;

// Calibration spin: right, left, right on the spot while the sensors record
// their min/max (readPosition() does the recording). Then Stop.
void calibrateStep(const Tuning& tune) {
    if (!calibrating) {
        calibrating = true;
        calStartMs = millis();
        sensorsCalibrateBegin();
    }

    uint32_t t = millis() - calStartMs;
    if (t < CAL_DURATION_MS) {
        bool right = t < CAL_DURATION_MS / 4 || t >= CAL_DURATION_MS * 3 / 4;
        float s = right ? tune.calSpeed : -tune.calSpeed;
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

    // The corner brake only arms while following the line in Auto.
    if (!running || cmd.mode != DriveMode::Auto) {
        downhillUntilMs = 0;
        inGapLastTick = false;
        brakeArmed = false;
        brakeUntilMs = 0;
        plainSinceMs = millis();
    }

    if (!running || cmd.mode == DriveMode::Stop) {
        motorsStop();
        return pos;
    }
    if (cmd.mode == DriveMode::Calibrate) {
        calibrateStep(t);
        return pos;
    }
    if (cmd.mode == DriveMode::Manual) {
        manualStep(cmd, t);
        wasLost = false;
        return pos;
    }

    LineState state = lineState();

    // Line back after a crest long enough to be real: the robot is over the
    // top now, so slow down for the way down. (lostSinceMs = gap start.)
    if (inGapLastTick && state == LineState::OnLine && cmd.mode == DriveMode::Auto &&
        millis() - lostSinceMs >= DOWNHILL_MIN_GAP_MS) {
        downhillUntilMs = millis() + (uint32_t)t.downhillMs;
    }
    inGapLastTick = state == LineState::InGap;

    if (state != LineState::OnLine) {
        if (!wasLost) {
            wasLost = true;
            lostSinceMs = millis();
        }
        uint32_t timeout = LOST_TIMEOUT_MS + (state == LineState::InGap ? (uint32_t)t.gapMs : 0);
        if (millis() - lostSinceMs > timeout) {
            motorsStop();
            plainSinceMs = millis();  // re-arm only after following again
            return pos;
        }
    } else {
        wasLost = false;
    }

    // Kill the speed before a sharp corner. Line following restarts with a
    // fresh derivative afterwards (following stays false).
    bool hold = cmd.mode == DriveMode::Hold;
    if (!hold && cornerBrake(state, t)) {
        motorsSet(-1.0f, -1.0f);
        return pos;
    }

    // Line gone from the middle (hill crest, gap): drive straight on for
    // gapMs (the corner brake ignores this case), then search toward the side
    // it was last seen on. The Center test doesn't drive forward: it stops.
    if (state == LineState::InGap) {
        if (hold) {
            motorsStop();
        } else if (millis() - lostSinceMs < (uint32_t)t.gapMs) {
            motorsSet(t.autoSpeed, t.autoSpeed);  // full power to get over the top
        } else if (int8_t side = sensorsLastSide()) {
            float s = side * t.pivotSpeed;
            motorsSet(s, -s);  // line left: left wheel back, right forward
        } else {
            motorsStop();
        }
        return pos;
    }

    // Lost past a sharp corner: spin on the spot toward the side the line
    // went (pos = -1 / +1) until a sensor finds it again.
    if (state == LineState::Lost && pos != 0.0f) {
        float s = pos < 0.0f ? -t.pivotSpeed : t.pivotSpeed;
        motorsSet(s, -s);  // line left: left wheel back, right forward
        return pos;
    }

    // Line following (re)starts: no derivative from a stale position.
    if (!wasFollowing) controlReset();
    following = true;

    // Center test: the same steering with no forward speed, so the robot
    // turns on the spot (line left: left wheel back, right forward).
    if (hold) {
        float c = constrain(controlCorrection(pos, dt), -1.0f, 1.0f);
        motorsSet(c, -c);
        return pos;
    }

    float l, r;
    controlUpdate(pos, dt, l, r, downhill() ? t.downhillScale : 1.0f);
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
        s.crossing = sensorsCrossing();
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
