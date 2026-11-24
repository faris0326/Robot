#include "control.h"

#include <Arduino.h>

#include "config.h"

namespace {
ControlGains gains;
float        smoothPos = 0.0f;  // position after the low-pass (smoothMs)
float        prevPos = 0.0f;
float        deriv = 0.0f;  // filtered d(position)/dt, position units per second
bool         primed = false;
float        cruiseI = 0.0f;  // speed-control integral, mm/s
}  // namespace

void controlBegin() {
    gains = {KP, KD, BASE_SPEED, POSITION_SMOOTH_MS};
    controlReset();
}

void controlSetGains(const ControlGains& g) { gains = g; }
ControlGains controlGetGains() { return gains; }

void controlReset() {
    primed = false;
    deriv = 0.0f;
}

float controlCorrection(float position, float dt) {
    if (!primed) {
        smoothPos = position;
    } else if (dt > 0.0f) {
        float a = dt / (gains.smoothMs * 1e-3f + dt);  // first-order low-pass; 1 when off
        smoothPos += a * (position - smoothPos);
    }

    if (primed && dt > 0.0f) {
        float raw = (smoothPos - prevPos) / dt;
        float a = dt / (KD_FILTER_MS * 1e-3f + dt);  // first-order low-pass
        deriv += a * (raw - deriv);
    }
    prevPos = smoothPos;
    primed = true;

    // Line to the left (position < 0) -> slow the left wheel.
    return gains.kp * smoothPos + gains.kd * deriv;
}

void controlUpdate(float position, float dt, float& left, float& right) {
    controlMix(position, dt, gains.base, left, right);
}

void controlMix(float position, float dt, float base, float& left, float& right) {
    float correction = controlCorrection(position, dt);
    float l = base + correction;
    float r = base - correction;

    // If the outer wheel would exceed full speed, shift both down instead of
    // clipping one, so the steering difference is kept.
    float over = max(l, r) - 1.0f;
    if (over > 0.0f) {
        l -= over;
        r -= over;
    }

    left = l;
    right = r;
}

float controlCruise(float targetMms, float measuredMms, float dt, float kp, float ki, float mmsAtFull) {
    float err = targetMms - measuredMms;
    float out = (targetMms + kp * err + cruiseI) / mmsAtFull;
    bool stuckHigh = out >= 1.0f && err > 0.0f;
    bool stuckLow = out <= CRUISE_MIN_BASE && err < 0.0f;
    if (!stuckHigh && !stuckLow) cruiseI = constrain(cruiseI + ki * err * dt, -mmsAtFull, mmsAtFull);
    return constrain(out, CRUISE_MIN_BASE, 1.0f);
}

void controlCruiseReset() { cruiseI = 0.0f; }

void controlManual(float throttle, float steer, float maxSpeed, float& left, float& right) {
    throttle = constrain(throttle, -1.0f, 1.0f);
    steer = constrain(steer, -1.0f, 1.0f);

    // Steer right -> left wheel faster. Full steer at zero throttle spins in place.
    float l = throttle + steer;
    float r = throttle - steer;

    // Scale both down together so the turn ratio is kept.
    float biggest = max(fabsf(l), fabsf(r));
    if (biggest > 1.0f) {
        l /= biggest;
        r /= biggest;
    }

    left = l * maxSpeed;
    right = r * maxSpeed;
}
