#pragma once
// Steering controller: line position -> left/right motor duty.
// PD: correction = kp * position + kd * d(position)/dt, with the derivative
// low-pass filtered (KD_FILTER_MS) because digital sensors move in steps.

struct ControlGains {
    float kp;
    float kd;
    float base;  // duty when the line is centred
};

void         controlBegin();  // load defaults from config.h
void         controlSetGains(const ControlGains& g);  // live tuning
ControlGains controlGetGains();

// Forget the derivative history. Call when line following (re)starts, so a
// stale position doesn't produce a derivative kick.
void controlReset();

// PD steering correction: the speed difference added to the left wheel and
// taken from the right (line left = negative). Updates the derivative.
float controlCorrection(float position, float dt);

// position: -1..+1 from readPosition(). dt: seconds since the last call.
void controlUpdate(float position, float dt, float& left, float& right);

// Manual drive: throttle -1..+1 (forward), steer -1..+1 (right) mixed into
// left/right speed, at most maxSpeed (fraction of rated motor voltage).
void controlManual(float throttle, float steer, float maxSpeed, float& left, float& right);
