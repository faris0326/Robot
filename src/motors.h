#pragma once
// DRV8833 dual H-bridge driver. Speed is a signed fraction of the motors'
// rated voltage: +1 = full forward at MOTOR_RATED_V, -1 = full reverse. The
// PWM duty is speed x MAX_DUTY, so the motors never see more than their
// rating. The balance trim then slows the faster motor.

void  motorsBegin();
void  motorsSet(float left, float right);
void  motorsStop();  // active brake on both motors

// Balance trim: > 0 slows the right motor by that fraction, < 0 slows the
// left. Starts at MOTOR_TRIM.
void  motorsSetTrim(float trim);

float motorsLeftDuty();   // last applied speed (after clamp and trim), for telemetry
float motorsRightDuty();
