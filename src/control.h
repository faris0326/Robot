#pragma once
// Steering controller: line position -> left/right motor duty.
// PD: correction = kp * position + kd * d(position)/dt, with the derivative
// low-pass filtered (KD_FILTER_MS) because digital sensors move in steps.

struct ControlGains {
    float kp;
    float kd;
    float base;      // duty when the line is centred
    float smoothMs;  // position low-pass time constant, 0 = off
};

void         controlBegin();  // load defaults from config.h
void         controlSetGains(const ControlGains& g);  // live tuning
ControlGains controlGetGains();

// Forget the derivative history. Call when line following (re)starts, so a
// stale position doesn't produce a derivative kick.
void controlReset();

// PD steering correction: the speed difference added to the left wheel and
// taken from the right (line left = negative). Smooths the position first
// (smoothMs), then updates the derivative.
float controlCorrection(float position, float dt);

// position: -1..+1 from readPosition(). dt: seconds since the last call.
void controlUpdate(float position, float dt, float& left, float& right);

// The same steering mix around a given base speed (from the speed control).
void controlMix(float position, float dt, float base, float& left, float& right);

// Speed control: the base speed (fraction of the voltage limit) that holds
// targetMms, given the measured speed. mmsAtFull = speed at base 1.0. PI with
// feed-forward; the integral stops while the output is saturated.
float controlCruise(float targetMms, float measuredMms, float dt, float kp, float ki, float mmsAtFull);
void  controlCruiseReset();

// Manual drive: throttle -1..+1 (forward), steer -1..+1 (right) mixed into
// left/right speed, at most maxSpeed (fraction of rated motor voltage).
void controlManual(float throttle, float steer, float maxSpeed, float& left, float& right);
