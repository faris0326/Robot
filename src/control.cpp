#include "control.h"

#include <Arduino.h>

#include "config.h"

namespace {
ControlGains gains;
float        prevPos = 0.0f;
float        deriv = 0.0f;  // filtered d(position)/dt, position units per second
bool         primed = false;
}  // namespace

void controlBegin() {
    gains = {KP, KD, BASE_SPEED};
    controlReset();
}

void controlSetGains(const ControlGains& g) { gains = g; }
ControlGains controlGetGains() { return gains; }

void controlReset() {
    primed = false;
    deriv = 0.0f;
}

void controlUpdate(float position, float dt, float& left, float& right) {
    if (primed && dt > 0.0f) {
        float raw = (position - prevPos) / dt;
        float a = dt / (KD_FILTER_MS * 1e-3f + dt);  // first-order low-pass
        deriv += a * (raw - deriv);
    }
    prevPos = position;
    primed = true;

    // Line to the left (position < 0) -> slow the left wheel.
    float correction = gains.kp * position + gains.kd * deriv;
    float l = gains.base + correction;
    float r = gains.base - correction;

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
